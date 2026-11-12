"""Hardware-free tests using real modules and mocked vendor dependencies."""
from threading import Event, Lock
from collections import deque
from types import ModuleType, SimpleNamespace
import json
import sys
import unittest
from unittest.mock import Mock, patch, mock_open

from biaxial_testing_machine import configuration as config, unit

serial = ModuleType("serial")
serial.SerialException = OSError
serial.Serial = Mock()
serial_tools = ModuleType("serial.tools")
serial_tools.list_ports = SimpleNamespace(comports=Mock(return_value=[]))
qt = ModuleType("PyQt6.QtCore")
qt.QObject = object
qt.QMetaObject = qt.QThread = qt.QTimer = qt.Qt = Mock()
qt.pyqtSignal = Mock
qt.pyqtSlot = lambda *args: lambda method: method
zaber = ModuleType("zaber_motion")
zaber.Units = Mock()
ascii_module = ModuleType("zaber_motion.ascii")
ascii_module.Connection = Mock()
with patch.dict(sys.modules, {
    "serial": serial, "serial.tools": serial_tools,
    "PyQt6": ModuleType("PyQt6"), "PyQt6.QtCore": qt,
    "zaber_motion": zaber, "zaber_motion.ascii": ascii_module,
}):
    from biaxial_testing_machine import dscusb as dsc
    from biaxial_testing_machine.hardware import HardwareWorker

# Keep these mock-backed module references local; real Qt tests import fresh modules.
sys.modules.pop("biaxial_testing_machine.hardware", None)
sys.modules.pop("biaxial_testing_machine.dscusb", None)


class ForceTests(unittest.TestCase):
    def cache(self):
        cache = dsc.ForceAcquisition.__new__(dsc.ForceAcquisition)
        cache.config = config.ForceConfig()
        cache._lock = Lock()
        cache._buffers = [deque([1000.0], maxlen=30), deque([2000.0], maxlen=30)]
        cache._timestamps = [dsc.monotonic(), dsc.monotonic()]
        cache._zeros = (0, 0)
        cache._errors = ["", ""]
        return cache

    def test_protocol_and_serial_identity(self):
        port = Mock(port="COM9")
        port.write.side_effect = len
        port.read_until.side_effect = [b"+123.456\r", b"272\r", b"9928\r"]
        with patch.object(serial, "Serial", return_value=port):
            device = dsc.DSCUSB("COM9", .075)
        self.assertEqual(device.read_force_g(), 123.456)
        self.assertEqual(device.get_serial_number(), "0017835720")
        self.assertEqual(port.write.call_args_list[0].args, (b"!001:SYS?\r",))
        with self.assertRaises(ValueError):
            device.query("SGAI=1")

    def test_invalid_responses(self):
        device = dsc.DSCUSB.__new__(dsc.DSCUSB)
        device.port = Mock(port="COM9")
        device.port.write.side_effect = len
        for response in (b"", b"12", b"?\r", b"nan\r", b"inf\r", b"bad\r"):
            with self.subTest(response=response):
                device.port.read_until.return_value = response
                with self.assertRaises(dsc.ForceDataError):
                    device.read_force_g()

    def test_discovery_reversed_ports_and_missing_cleanup(self):
        numbers = ("0017835720", "0017835697")
        first, second = Mock(), Mock()
        first.get_serial_number.return_value = numbers[1]
        second.get_serial_number.return_value = numbers[0]
        ports = [SimpleNamespace(device="COM8"), SimpleNamespace(device="COM3")]
        with patch.object(dsc.list_ports, "comports", return_value=ports), \
                patch.object(dsc, "DSCUSB", side_effect=[first, second]):
            self.assertEqual(dsc.discover(numbers, .075), [second, first])
        with patch.object(dsc.list_ports, "comports", return_value=ports[:1]), \
                patch.object(dsc, "DSCUSB", return_value=first):
            with self.assertRaises(dsc.ForceDataError):
                dsc.discover(numbers, .075)
        first.close.assert_called_once()

    def test_zero_transmission_and_stale_axis(self):
        cache = self.cache()
        self.assertEqual(cache.read(), (500, 1000))
        self.assertEqual(cache.zero(), (1000, 2000))
        cache._buffers = [deque([1200]), deque([2400])]
        self.assertEqual(cache.read(), (100, 200))
        cache._timestamps[1] -= 1
        with self.assertRaises(dsc.ForceDataError):
            cache.read()
        with self.assertRaises(dsc.ForceDataError):
            cache.zero()

    def test_background_timeout_does_not_block_cache(self):
        entered, release = Event(), Event()
        device = Mock()
        def blocked_read():
            entered.set()
            release.wait(2)
            raise dsc.ForceDataError("timeout")
        device.read_force_g.side_effect = blocked_read
        with patch.object(dsc, "discover", return_value=[device, device]):
            cache = dsc.ForceAcquisition(config.ForceConfig())
        try:
            self.assertTrue(entered.wait(1))
            with self.assertRaises(dsc.ForceDataError):
                cache.read()
            self.assertFalse(release.is_set())
        finally:
            release.set()
            cache.close()
        self.assertFalse(cache._thread.is_alive())

    def test_newtons_watchdog_latches_and_stops_both_axes(self):
        worker = HardwareWorker.__new__(HardwareWorker)
        worker.FORCE_AVERAGE_WINDOW = 5
        worker._acquisition = self.cache()
        worker._units = unit.Unit.Newton
        worker._force_fault = None
        worker._motion_blocked = Event()
        worker._axis1, worker._axis2 = Mock(), Mock()
        worker._autoload_timer = Mock()
        worker._stop_timer = Mock()
        worker.signal_force_fault = Mock()
        self.assertEqual(worker.get_forces(), (4.90332, 9.80665))
        worker.arm_force_guard()
        worker._acquisition._timestamps[0] -= 1
        worker._check_force_guard()
        worker._axis1.stop.assert_called_once_with(wait_until_idle=False)
        worker._axis2.stop.assert_called_once_with(wait_until_idle=False)
        worker.signal_force_fault.emit.assert_called_once()
        worker._acquisition._timestamps[0] = dsc.monotonic()
        with self.assertRaises(dsc.ForceDataError):
            worker._allow_motion()
        worker._axis1.stop.side_effect = RuntimeError("axis 1 unavailable")
        with self.assertRaises(RuntimeError):
            worker.stop_motors()
        self.assertEqual(worker._axis2.stop.call_count, 2)

    def test_config_validation(self):
        for kwargs in ({"axis2_serial": "0017835720"}, {"timeout": 0},
                       {"max_data_age": .1}, {"axis1_transmission_factor": float("nan")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                config.ForceConfig(**kwargs)

    def test_config_roundtrip_and_legacy_defaults(self):
        repository = config.ConfigRepository("config.json")
        with patch("builtins.open", mock_open(read_data='{"unit": "gram"}')), \
                patch.object(config.os.path, "exists", return_value=True):
            saved = repository.load()
        self.assertEqual(saved.force.axis2_serial, "0017835697")
        saved.force.axis2_transmission_factor = 1.0
        output = mock_open()
        with patch("builtins.open", output):
            repository.save(saved)
        encoded = "".join(call.args[0] for call in output().write.call_args_list)
        self.assertEqual(json.loads(encoded)["force"]["axis2_transmission_factor"], 1.0)
        with patch("builtins.open", mock_open(read_data=encoded)), \
                patch.object(config.os.path, "exists", return_value=True):
            self.assertEqual(repository.load().force.axis2_transmission_factor, 1.0)

    def test_duplicate_device_cleanup(self):
        devices = [Mock(), Mock()]
        for device in devices:
            device.get_serial_number.return_value = "0017835720"
        ports = [SimpleNamespace(device="COM1"), SimpleNamespace(device="COM2")]
        with patch.object(dsc.list_ports, "comports", return_value=ports), \
                patch.object(dsc, "DSCUSB", side_effect=devices):
            with self.assertRaisesRegex(dsc.ForceDataError, "Duplicate"):
                dsc.discover(("0017835720", "0017835697"), .075)
        for device in devices:
            device.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
