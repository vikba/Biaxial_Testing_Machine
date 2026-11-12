"""Read-only DSCUSB protocol and background acquisition of calibrated grams."""
from collections import deque
from math import isfinite
from statistics import mean
from threading import Event, Lock, Thread
from time import monotonic

import serial
from serial.tools import list_ports


class ForceDataError(RuntimeError):
    pass


class DSCUSB:
    def __init__(self, port, timeout):
        self.port = serial.Serial(port, 115200, bytesize=8, parity="N", stopbits=1,
                                  timeout=timeout, write_timeout=timeout)

    def query(self, command):
        """Query a numeric parameter, e.g. SYS?; never write calibration."""
        if not command.endswith("?") or not command[:-1].isalnum():
            raise ValueError("Expected a read-only parameter query")
        try:
            self.port.reset_input_buffer()
            request = f"!001:{command}\r".encode("ascii")
            if self.port.write(request) != len(request):
                raise ValueError("incomplete write")
            response = self.port.read_until(b"\r", size=64)
            if not response.endswith(b"\r"):
                raise ValueError("timeout or incomplete response")
            value = float(response.decode("ascii").strip())
            if not isfinite(value):
                raise ValueError("nonfinite response")
            return value
        except (serial.SerialException, OSError, ValueError) as exc:
            raise ForceDataError(f"DSCUSB {self.port.port}, {command}: {exc}") from exc

    def read_force_g(self):
        return self.query("SYS?")

    def get_serial_number(self):
        high, low = self.query("SERH?"), self.query("SERL?")
        if any(not value.is_integer() or not 0 <= value <= 65535 for value in (high, low)):
            raise ForceDataError("Invalid DSCUSB serial number")
        return f"{int(high) * 65536 + int(low):010d}"

    def close(self):
        self.port.close()


def discover(serial_numbers, timeout):
    """Match device-reported serials, independent of COM allocation."""
    found = {}
    try:
        for port in list_ports.comports():
            device = None
            try:
                device = DSCUSB(port.device, timeout)
                number = device.get_serial_number()
            except (ForceDataError, serial.SerialException, OSError):
                if device is not None:
                    device.close()
                continue
            if number not in serial_numbers:
                device.close()
            elif number in found:
                device.close()
                raise ForceDataError(f"Duplicate DSCUSB serial: {number}")
            else:
                found[number] = device
        missing = set(serial_numbers) - found.keys()
        if missing:
            raise ForceDataError(f"DSCUSB devices unavailable: {', '.join(sorted(missing))}")
        return [found[number] for number in serial_numbers]
    except Exception:
        for device in found.values():
            device.close()
        raise


class ForceAcquisition:
    def __init__(self, config):
        self.config = config
        self._lock = Lock()
        self._stop = Event()
        self._buffers = [deque(maxlen=30), deque(maxlen=30)]
        self._timestamps = [None, None]
        self._zeros = (0.0, 0.0)
        self._errors = ["no measurement", "no measurement"]
        self._devices = discover((config.axis1_serial, config.axis2_serial), config.timeout)
        self._thread = Thread(target=self._run, name="DSCUSB acquisition", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            for axis, device in enumerate(self._devices):
                try:
                    timestamp = monotonic()
                    value = device.read_force_g()
                    with self._lock:
                        previous = self._timestamps[axis]
                        if previous is not None and timestamp - previous > self.config.max_data_age:
                            self._buffers[axis].clear()
                        self._buffers[axis].append(value)
                        self._timestamps[axis] = timestamp
                        self._errors[axis] = ""
                except ForceDataError as exc:
                    with self._lock:
                        self._errors[axis] = str(exc)
            self._stop.wait(0.005)

    def _check_fresh(self):
        for axis, timestamp in enumerate(self._timestamps):
            if timestamp is None or monotonic() - timestamp > self.config.max_data_age:
                raise ForceDataError(f"Axis {axis + 1} force data stale: {self._errors[axis]}")

    def read(self, window=5):
        with self._lock:
            self._check_fresh()
            factors = (self.config.axis1_transmission_factor, self.config.axis2_transmission_factor)
            return tuple((mean(list(buffer)[-window:]) - zero) * factor
                         for buffer, zero, factor in zip(self._buffers, self._zeros, factors))

    def zero(self):
        with self._lock:
            self._check_fresh()
            self._zeros = tuple(mean(list(buffer)[-10:]) for buffer in self._buffers)
            return self._zeros

    def close(self):
        self._stop.set()
        self._thread.join()  # Serial reads and writes have bounded timeouts.
        with self._lock:
            self._timestamps = [None, None]
        for device in self._devices:
            device.close()
