from __future__ import annotations

from dataclasses import dataclass, field
from threading import Event
import collections
from statistics import mean
import time
from typing import Any

import numpy as np
from PyQt6.QtCore import QObject, QMetaObject, QThread, QTimer, Qt, pyqtSignal, pyqtSlot
from scipy.signal import butter, filtfilt

from zaber_motion.ascii import Connection
from zaber_motion import Units
import ginsapy.giutility.connect.PyQStationConnectWin as Qstation

from .ringbuffer import RingBuffer
from .unit import Unit


@dataclass
class CommandRequest:
    method_name: str
    args: tuple[Any, ...] = field(default_factory=tuple)
    kwargs: dict[str, Any] = field(default_factory=dict)
    event: Event = field(default_factory=Event)
    result: Any = None
    error: Exception | None = None


class HardwareWorker(QObject):
    FORCE_AVERAGE_WINDOW = 5

    signal_live_forces = pyqtSignal(float, float)
    signal_hardware_connected = pyqtSignal()
    signal_hardware_disconnected = pyqtSignal()
    signal_error = pyqtSignal(str)
    signal_autoload_started = pyqtSignal(float, float)
    signal_autoload_progress = pyqtSignal(float, float, float, float)
    signal_autoloading_finished = pyqtSignal()
    signal_autoload_rejected = pyqtSignal(str)
    signal_autoload_failed = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._units = Unit.Newton
        self._force1 = self._force2 = 0.0
        self._force1_0 = self._force2_0 = 0.0
        self._pos1_0 = self._pos2_0 = 0.0
        self._mot_init = False
        self._daq_init = False
        self._connected = False
        self._execute_autoloading = False
        self._ip_motor = ""
        self._ip_daq = ""
        self._target_load1 = 0.0
        self._target_load2 = 0.0
        self._latest_forces = (0.0, 0.0)
        self._latest_positions = (0.0, 0.0)
        self._autoload_timer = QTimer(self)
        self._autoload_timer.timeout.connect(self._autoload_step)
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.timeout.connect(self._emit_live_forces)
        self._ringbuffer1 = RingBuffer(20)
        self._ringbuffer2 = RingBuffer(20)
        buffer_size = 30
        self._buffer1 = collections.deque(maxlen=buffer_size)
        self._buffer2 = collections.deque(maxlen=buffer_size)

    @pyqtSlot(object)
    def handle_command(self, request: CommandRequest):
        try:
            method = getattr(self, request.method_name)
            request.result = method(*request.args, **request.kwargs)
        except Exception as exc:  # pragma: no cover - hardware path
            request.error = exc
            self.signal_error.emit(str(exc))
        finally:
            request.event.set()

    def connect_hardware(self, unit: Unit, ip_motor: str, ip_daq: str):
        self._units = unit
        self._ip_motor = ip_motor
        self._ip_daq = ip_daq
        self._connect_motors()
        self._connect_daq()
        self._connected = True
        self._telemetry_timer.start(50)
        self.signal_hardware_connected.emit()

    def disconnect_hardware(self):
        self._execute_autoloading = False
        self._stop_timer(self._autoload_timer)
        self._stop_timer(self._telemetry_timer)

        if hasattr(self, "_axis1"):
            self._axis1.stop()
        if hasattr(self, "_axis2"):
            self._axis2.stop()
        if hasattr(self, "_connection_z"):
            self._connection_z.close()
        if hasattr(self, "_conn_q"):
            self._conn_q.close_connection()

        self._connected = False
        self._mot_init = False
        self._daq_init = False
        self.signal_hardware_disconnected.emit()

    def is_initialized(self):
        return self._mot_init and self._daq_init

    def change_units(self, unit: Unit):
        self._units = unit

    def _connect_motors(self):
        try:
            self._connection_z = Connection.open_tcp(self._ip_motor, Connection.TCP_PORT_CHAIN)
        except Exception as exc:
            raise Exception(
                "Connection with motors failed at IP: "
                + self._ip_motor
                + " Please check the network connection and ensure that the specified IP address is correct."
            ) from exc

        print("Motors were found")
        self._connection_z.enable_alerts()
        device_list = self._connection_z.detect_devices()
        if not device_list:
            raise Exception("No motor devices were found.")
        device = device_list[0]
        self._axis1 = device.get_axis(1)
        self._axis2 = device.get_axis(2)

    def _connect_daq(self):
        self._conn_q = Qstation.ConnectGIns()
        self._conn_q.bufferindex = 0
        init_con_res = self._conn_q.init_connection(str(self._ip_daq))
        if not init_con_res:
            raise Exception(
                "Connection with DAQ failed at IP: "
                + self._ip_daq
                + " Please check the network connection and ensure that the specified IP address is correct."
            )

        self._conn_q.read_controller_name()
        self._conn_q.read_serial_number()
        self._conn_q.read_sample_rate()
        self._conn_q.read_channel_names()
        self._conn_q.read_channel_count()

        self._buffer = self._conn_q.yield_buffer()
        next(self._buffer)
        QThread.msleep(200)
        val1, val2 = self._read_force()
        print("Initial force at channel 1: {}".format(val1))
        print("Initial force at channel 2: {}".format(val2))

    def stop_test(self):
        self._execute_autoloading = False
        self._stop_timer(self._autoload_timer)
        self.stop_motors()

    def stop_motors(self):
        if hasattr(self, "_axis1"):
            self._axis1.stop()
        if hasattr(self, "_axis2"):
            self._axis2.stop()

    def stop_motor1(self):
        self._axis1.stop()

    def stop_motor2(self):
        self._axis2.stop()

    def _read_force(self):
        try:
            value = next(self._buffer)
            value1 = round(value[:, 1].mean(), 5)
            value2 = round(value[:, 2].mean(), 5)
        except Exception:
            print("Empty buffer")
            return None, None
        else:
            self._buffer1.append(value1)
            self._buffer2.append(value2)

        return value1, value2

    def get_forces(self):
        self._read_force()
        force1 = self._average_buffer(self._buffer1, HardwareWorker.FORCE_AVERAGE_WINDOW)
        force2 = self._average_buffer(self._buffer2, HardwareWorker.FORCE_AVERAGE_WINDOW)
        converted = self._convert_mvv(force1 - self._force1_0, force2 - self._force2_0)
        self._latest_forces = converted
        return converted

    def get_av_forces(self, n: int = 3):
        QThread.usleep(500)
        self._read_force()
        force1 = self._average_buffer(self._buffer1, n)
        force2 = self._average_buffer(self._buffer2, n)
        converted = self._convert_mvv(force1 - self._force1_0, force2 - self._force2_0)
        self._latest_forces = converted
        return converted

    def get_positions(self):
        len1 = self._axis1.get_position(Units.LENGTH_MILLIMETRES) - self._pos1_0
        len2 = self._axis2.get_position(Units.LENGTH_MILLIMETRES) - self._pos2_0
        self._latest_positions = (round(len1, 5), round(len2, 5))
        return self._latest_positions

    def zeroForce(self):
        self._force1_0 = mean(list(self._buffer1)[-10:])
        self._force2_0 = mean(list(self._buffer2)[-10:])
        force1_0, force2_0 = self._convert_mvv(self._force1_0, self._force2_0)
        print("Init force 1: {}, force 2: {}".format(force1_0, force2_0))
        self._mot_init = True
        return force1_0, force2_0

    def zeroPosition(self):
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        print("Init pos 1: {}, pos2: {}".format(self._pos1_0, self._pos2_0))
        self._daq_init = True
        return self._pos1_0, self._pos2_0

    def zero_pos1(self):
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        return self._pos1_0

    def zero_pos2(self):
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        return self._pos2_0

    def _average_buffer(self, buffer, window):
        if not buffer:
            return 0.0
        sample_count = min(window, len(buffer))
        return mean(list(buffer)[-sample_count:])

    def _stop_timer(self, timer):
        if not isinstance(timer, QTimer):
            return
        if timer.thread() is QThread.currentThread():
            if timer.isActive():
                timer.stop()
            return
        QMetaObject.invokeMethod(timer, "stop", Qt.ConnectionType.BlockingQueuedConnection)

    def _convert_mvv(self, val1, val2):
        val1 /= 2
        val2 /= 2
        if self._units == Unit.Newton:
            return self._to_newtons(val1, val2)
        return self._to_grams(val1, val2)

    def _to_newtons(self, val1, val2):
        z1 = -0.0006
        z2 = 0.0024
        k1 = 250 / (0.7999 - z1)
        k2 = 250 / (0.8317 - z2)
        return round(k1 * val1, 5), round(k2 * val2, 5)

    def _to_grams(self, val1, val2):
        force1, force2 = self._to_newtons(val1, val2)
        g = 9.81
        return round(force1 * 1000 / g, 3), round(force2 * 1000 / g, 3)

    def moveSamplePosition(self):
        samplePosition = 78
        if self._pos1_0 > 0 and self._pos2_0 > 0:
            self._axis1.move_absolute(
                self._pos1_0,
                Units.LENGTH_MILLIMETRES,
                velocity=5,
                velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
                wait_until_idle=False,
            )
            self._axis2.move_absolute(
                self._pos2_0,
                Units.LENGTH_MILLIMETRES,
                velocity=5,
                velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
                wait_until_idle=False,
            )
        else:
            self._axis1.move_absolute(
                samplePosition,
                Units.LENGTH_MILLIMETRES,
                velocity=5,
                velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
                wait_until_idle=False,
            )
            self._axis2.move_absolute(
                samplePosition,
                Units.LENGTH_MILLIMETRES,
                velocity=5,
                velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
                wait_until_idle=False,
            )

    def initMotZeroPos(self):
        self._axis1.home(wait_until_idle=False)
        self._axis2.home(wait_until_idle=False)
        min_limit = 0
        max_limit = 83
        self._axis1.settings.set("limit.min", min_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis1.settings.set("limit.max", max_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis2.settings.set("limit.min", min_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis2.settings.set("limit.max", max_limit, unit=Units.LENGTH_MILLIMETRES)

    def move_position_ax1(self, pos, vel):
        self._axis1.move_absolute(
            self._pos1_0 + pos,
            Units.LENGTH_MILLIMETRES,
            velocity=vel,
            velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
            wait_until_idle=False,
        )

    def move_position_ax2(self, pos, vel):
        self._axis2.move_absolute(
            self._pos2_0 + pos,
            Units.LENGTH_MILLIMETRES,
            velocity=vel,
            velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
            wait_until_idle=False,
        )

    def move_velocity_ax1(self, speed):
        self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)

    def move_velocity_ax2(self, speed):
        self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)

    def butter_lowpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype="low", analog=False)
        return b, a

    def lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        return filtfilt(b, a, data)

    def perform_autoload(self, target_load1, target_load2):
        self._execute_autoloading = True
        self._target_load1 = target_load1
        self._target_load2 = target_load2
        self._force1, self._force2 = self.get_forces()
        self._init_offset_1 = self._target_load1 - self._force1
        self._init_offset_2 = self._target_load2 - self._force2
        print(f"Starting autoloading. Offset1: {self._init_offset_1}, Offset2: {self._init_offset_2}")

        if self._can_start_autoload():
            self._start_time = round(time.perf_counter(), 5)
            self._current_time = 0
            self._autoload_timer.start(250)
            self.signal_autoload_started.emit(target_load1, target_load2)
            return True

        self._execute_autoloading = False
        message = "Autoload rejected: the difference between current and desired forces is too high."
        self.signal_autoload_rejected.emit(message)
        return False

    def _can_start_autoload(self):
        if self._units == Unit.Newton:
            return abs(self._init_offset_1) < 0.2 and abs(self._init_offset_2) < 0.2
        return abs(self._init_offset_1) < 20 and abs(self._init_offset_2) < 20

    def _autoload_thresholds(self):
        min_threshold = 0.02 if self._units == Unit.Newton else 2.0
        threshold1 = max(min_threshold, 0.05 * abs(self._target_load1))
        threshold2 = max(min_threshold, 0.05 * abs(self._target_load2))
        return threshold1, threshold2

    def _autoload_step(self):
        if self._current_time < 30 and self._execute_autoloading:
            self._current_time = round(time.perf_counter() - self._start_time, 5)
            self._force1, self._force2 = self.get_av_forces(n=3)
            offset1 = self._target_load1 - self._force1
            offset2 = self._target_load2 - self._force2
            threshold1, threshold2 = self._autoload_thresholds()

            if abs(offset1) <= threshold1 and abs(offset2) <= threshold2:
                self.stop_motors()
                self._stop_timer(self._autoload_timer)
                self._execute_autoloading = False
                self.signal_autoloading_finished.emit()
                return

            kp = 1.5 if self._units == Unit.Newton else 0.015
            max_velocity = 0.2
            v1 = min(max_velocity, max(-max_velocity, -kp * offset1))
            v2 = min(max_velocity, max(-max_velocity, -kp * offset2))
            self.signal_autoload_progress.emit(self._force1, self._force2, v1, v2)
            print(f"force1: {self._force1}, force2: {self._force2}, vel1: {v1}, vel2: {v2}")
            self.move_velocity_ax1(v1)
            self.move_velocity_ax2(v2)
            return

        self._execute_autoloading = False
        self.stop_motors()
        self._stop_timer(self._autoload_timer)
        self.signal_autoload_failed.emit("Autoload timed out before reaching the requested load.")

    def _emit_live_forces(self):
        if not self._connected:
            return
        try:
            force1, force2 = self.get_av_forces()
        except Exception as exc:  # pragma: no cover - hardware path
            self.signal_error.emit(str(exc))
            return
        self.signal_live_forces.emit(force1, force2)


class MotorDAQInterface(QObject):
    signal_autoload_started = pyqtSignal(float, float)
    signal_autoloading_finished = pyqtSignal()
    signal_autoloading_progress = pyqtSignal(float, float, float, float)
    signal_autoload_rejected = pyqtSignal(str)
    signal_autoload_failed = pyqtSignal(str)
    signal_live_forces = pyqtSignal(float, float)
    signal_error = pyqtSignal(str)
    signal_hardware_connected = pyqtSignal()
    signal_hardware_disconnected = pyqtSignal()

    _command_requested = pyqtSignal(object)

    def __init__(self, unit, ip_motor="172.31.100.108", ip_daq="172.31.100.107"):
        super().__init__()
        self._units = unit
        self._ip_motor = ip_motor
        self._ip_daq = ip_daq
        self._thread = QThread()
        self._worker = HardwareWorker()
        self._worker.moveToThread(self._thread)
        self._command_requested.connect(self._worker.handle_command)
        self._worker.signal_live_forces.connect(self.signal_live_forces.emit)
        self._worker.signal_hardware_connected.connect(self.signal_hardware_connected.emit)
        self._worker.signal_hardware_disconnected.connect(self.signal_hardware_disconnected.emit)
        self._worker.signal_error.connect(self.signal_error.emit)
        self._worker.signal_autoload_started.connect(self.signal_autoload_started.emit)
        self._worker.signal_autoload_progress.connect(self.signal_autoloading_progress.emit)
        self._worker.signal_autoloading_finished.connect(self.signal_autoloading_finished.emit)
        self._worker.signal_autoload_rejected.connect(self.signal_autoload_rejected.emit)
        self._worker.signal_autoload_failed.connect(self.signal_autoload_failed.emit)
        self._thread.start()
        self._invoke("connect_hardware", unit, ip_motor, ip_daq)

    def _invoke(self, method_name, *args, wait=True, **kwargs):
        request = CommandRequest(method_name=method_name, args=args, kwargs=kwargs)
        self._command_requested.emit(request)
        if not wait:
            return None
        request.event.wait()
        if request.error is not None:
            raise request.error
        return request.result

    def is_initialized(self):
        return self._invoke("is_initialized")

    def change_units(self, unit):
        self._units = unit
        self._invoke("change_units", unit)

    def stop(self):
        try:
            self._invoke("disconnect_hardware")
        finally:
            self._thread.quit()
            self._thread.wait()

    def stop_test(self):
        self._invoke("stop_test")

    def stop_motors(self):
        self._invoke("stop_motors")

    def stop_motor1(self):
        self._invoke("stop_motor1")

    def stop_motor2(self):
        self._invoke("stop_motor2")

    def get_forces(self):
        return self._invoke("get_forces")

    def get_av_forces(self, n=3):
        return self._invoke("get_av_forces", n)

    def get_positions(self):
        return self._invoke("get_positions")

    def zeroForce(self):
        return self._invoke("zeroForce")

    def zeroPosition(self):
        return self._invoke("zeroPosition")

    def zero_pos1(self):
        return self._invoke("zero_pos1")

    def zero_pos2(self):
        return self._invoke("zero_pos2")

    def moveSamplePosition(self):
        self._invoke("moveSamplePosition")

    def initMotZeroPos(self):
        self._invoke("initMotZeroPos")

    def move_position_ax1(self, pos, vel):
        self._invoke("move_position_ax1", pos, vel)

    def move_position_ax2(self, pos, vel):
        self._invoke("move_position_ax2", pos, vel)

    def move_velocity_ax1(self, speed):
        self._invoke("move_velocity_ax1", speed)

    def move_velocity_ax2(self, speed):
        self._invoke("move_velocity_ax2", speed)

    def lowpass_filter(self, data, cutoff, fs, order=5):
        return self._invoke("lowpass_filter", data, cutoff, fs, order)

    def perform_autoload(self, target_load1, target_load2):
        return self._invoke("perform_autoload", target_load1, target_load2)
