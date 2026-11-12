from __future__ import annotations

import logging
from concurrent.futures import CancelledError
from dataclasses import dataclass, field
from math import isfinite
from threading import Event
import time
from typing import Any

from PyQt6.QtCore import QObject, QMetaObject, QThread, QTimer, Qt, pyqtSignal, pyqtSlot

from zaber_motion.ascii import Connection
from zaber_motion import Units
from .configuration import AXIS_TRAVEL_MM, ForceConfig
from .dscusb import ForceAcquisition, ForceDataError

from .unit import Unit


logger = logging.getLogger(__name__)


@dataclass
class CommandRequest:
    method_name: str
    args: tuple[Any, ...] = field(default_factory=tuple)
    kwargs: dict[str, Any] = field(default_factory=dict)
    event: Event = field(default_factory=Event)
    result: Any = None
    error: Exception | None = None
    deadline: float = float("inf")
    cancelled: Event = field(default_factory=Event)


class HardwareWorker(QObject):
    FORCE_AVERAGE_WINDOW = 5

    signal_command_finished = pyqtSignal(object)
    signal_live_forces = pyqtSignal(float, float)
    signal_hardware_connected = pyqtSignal()
    signal_hardware_disconnected = pyqtSignal()
    signal_error = pyqtSignal(str)
    signal_force_fault = pyqtSignal(str)
    signal_autoload_started = pyqtSignal(float, float)
    signal_autoload_progress = pyqtSignal(float, float, float, float)
    signal_autoloading_finished = pyqtSignal()
    signal_autoload_rejected = pyqtSignal(str)
    signal_autoload_failed = pyqtSignal(str)

    def __init__(self, force_config):
        super().__init__()
        self._force_config = force_config
        self._acquisition = None
        self._force_guard = False
        self._force_fault = None
        self._motion_blocked = Event()
        self._units = Unit.Newton
        self._force1 = self._force2 = 0.0
        self._pos1_0 = self._pos2_0 = 0.0
        self._force_zeroed = False
        self._position_zeroed = False
        self._connected = False
        self._execute_autoloading = False
        self._ip_motor = ""
        self._target_load1 = 0.0
        self._target_load2 = 0.0
        self._telemetry_error = None
        self._latest_positions = (0.0, 0.0)
        self._autoload_timer = QTimer(self)
        self._autoload_timer.timeout.connect(self._autoload_step)
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.timeout.connect(self._emit_live_forces)
        self._watchdog = QTimer(self)
        self._watchdog.setInterval(20)
        self._watchdog.timeout.connect(self._check_force_guard)

    @pyqtSlot(object)
    def handle_command(self, request: CommandRequest):
        try:
            if request.method_name not in ("stop_test", "stop_motors", "stop_motor1", "stop_motor2", "disconnect_hardware", "force_fault"):
                if request.cancelled.is_set():
                    raise CancelledError("Hardware command was cancelled.")
                if time.monotonic() >= request.deadline:
                    raise TimeoutError("Hardware command expired.")
            method = getattr(self, request.method_name)
            request.result = method(*request.args, **request.kwargs)
        except Exception as exc:
            request.error = exc
            if self._connected and not request.cancelled.is_set():
                self.force_fault(str(exc))
            elif request.method_name == "connect_hardware":
                try:
                    self.disconnect_hardware()
                except Exception as cleanup:
                    request.error = RuntimeError(f"{exc}; disconnect failed: {cleanup}")
                self.thread().quit()
        finally:
            request.event.set()
            self.signal_command_finished.emit(request)
            if request.method_name == "disconnect_hardware":
                self.thread().quit()

    def connect_hardware(self, unit: Unit, ip_motor: str, ip_daq: str):
        self._units = unit
        self._ip_motor = ip_motor
        self._connect_motors()
        self._connect_daq()
        self._watchdog.start()
        self._connected = True
        self._telemetry_timer.start(50)
        self.signal_hardware_connected.emit()

    def disconnect_hardware(self):
        self._execute_autoloading = self._force_guard = False
        actions = [self.stop_motors]
        actions.extend(timer.stop for timer in (self._autoload_timer, self._telemetry_timer, self._watchdog))
        actions.extend(resource.close for resource in (self._acquisition, getattr(self, "_connection_z", None)) if resource is not None)
        errors = []
        for action in actions:
            try:
                action()
            except Exception as exc:
                errors.append(str(exc))
        self._acquisition = None
        self._connected = self._force_zeroed = self._position_zeroed = False
        self.signal_hardware_disconnected.emit()
        if errors:
            raise RuntimeError("Disconnect failed: " + "; ".join(errors))

    def is_initialized(self):
        return self._force_zeroed and self._position_zeroed

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

        logger.info("Motor connection established")
        self._connection_z.enable_alerts()
        device_list = self._connection_z.detect_devices()
        if not device_list:
            raise Exception("No motor devices were found.")
        device = device_list[0]
        self._axis1 = device.get_axis(1)
        self._axis2 = device.get_axis(2)

    def _connect_daq(self):
        self._acquisition = ForceAcquisition(self._force_config)

    def arm_force_guard(self):
        self.get_forces()  # Refuse to start until both sensors have fresh data.
        if self._force_fault:
            raise ForceDataError(self._force_fault + " Reconnect hardware to reset.")
        self._force_guard = True

    def _check_force_guard(self):
        if not self._force_guard:
            return
        try:
            self.get_forces()
        except Exception as exc:
            self.force_fault(str(exc))

    def force_fault(self, message):
        if self._force_fault:
            return
        self._force_fault = message
        self._motion_blocked.set()
        try:
            self.stop_test()
        except Exception as exc:
            self._force_fault += f" Motor stop failed: {exc}"
        finally:
            self.signal_force_fault.emit(self._force_fault)

    def _allow_motion(self):
        self._check_force_guard()
        if self._force_fault:
            raise ForceDataError(self._force_fault + " Reconnect hardware to reset.")
        if self._motion_blocked.is_set():
            raise RuntimeError("Motion blocked after a hardware error. Reconnect hardware to reset.")

    def stop_test(self):
        self._force_guard = False
        self._execute_autoloading = False
        try:
            self._stop_timer(self._autoload_timer)
        finally:
            self.stop_motors()

    def stop_motors(self):
        errors = []
        for name in ("_axis1", "_axis2"):
            try:
                axis = getattr(self, name, None)
                if axis is not None:
                    axis.stop(wait_until_idle=False)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("Motor stop failed: " + "; ".join(errors))

    def stop_motor1(self):
        self._axis1.stop()

    def stop_motor2(self):
        self._axis2.stop()

    def get_forces(self):
        return self.get_av_forces(self.FORCE_AVERAGE_WINDOW)

    def get_av_forces(self, n=3):
        acquisition = self._acquisition
        if acquisition is None:
            raise ForceDataError("Force acquisition is disconnected")
        values = acquisition.read(max(1, n))
        return self._convert_grams(values)

    def _convert_grams(self, values):
        factor = 9.80665 / 1000 if self._units == Unit.Newton else 1.0
        return tuple(round(value * factor, 5) for value in values)

    def get_positions(self):
        len1 = self._axis1.get_position(Units.LENGTH_MILLIMETRES) - self._pos1_0
        len2 = self._axis2.get_position(Units.LENGTH_MILLIMETRES) - self._pos2_0
        self._latest_positions = (round(len1, 5), round(len2, 5))
        return self._latest_positions

    def zeroForce(self):
        if self._force_guard:
            raise RuntimeError("Stop load control before zeroing force")
        zeros = self._acquisition.zero()
        self._force_zeroed = True
        return self._convert_grams(value * transmission for value, transmission in zip(
            zeros, (self._force_config.axis1_transmission_factor,
                    self._force_config.axis2_transmission_factor)))

    def zeroPosition(self):
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        logger.debug("Position zero: axis 1=%s, axis 2=%s", self._pos1_0, self._pos2_0)
        self._position_zeroed = True
        return self._pos1_0, self._pos2_0

    def zero_pos1(self):
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        return self._pos1_0

    def zero_pos2(self):
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        return self._pos2_0

    def _stop_timer(self, timer):
        if not isinstance(timer, QTimer):
            return
        if timer.thread() is QThread.currentThread():
            if timer.isActive():
                timer.stop()
            return
        QMetaObject.invokeMethod(timer, "stop", Qt.ConnectionType.BlockingQueuedConnection)

    def moveSamplePosition(self):
        self._allow_motion()
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
        self._allow_motion()
        self._axis1.home(wait_until_idle=False)
        self._axis2.home(wait_until_idle=False)
        min_limit = 0
        max_limit = AXIS_TRAVEL_MM
        self._axis1.settings.set("limit.min", min_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis1.settings.set("limit.max", max_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis2.settings.set("limit.min", min_limit, unit=Units.LENGTH_MILLIMETRES)
        self._axis2.settings.set("limit.max", max_limit, unit=Units.LENGTH_MILLIMETRES)

    def move_position_ax1(self, pos, vel):
        self._allow_motion()
        if not isfinite(pos) or not 0 <= self._pos1_0 + pos <= AXIS_TRAVEL_MM or not isfinite(vel) or vel <= 0:
            raise ValueError(f"Axis 1 target must be within 0?{AXIS_TRAVEL_MM:g} mm and speed must be finite and positive.")
        self._axis1.move_absolute(
            self._pos1_0 + pos,
            Units.LENGTH_MILLIMETRES,
            velocity=vel,
            velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
            wait_until_idle=False,
        )

    def move_position_ax2(self, pos, vel):
        self._allow_motion()
        if not isfinite(pos) or not 0 <= self._pos2_0 + pos <= AXIS_TRAVEL_MM or not isfinite(vel) or vel <= 0:
            raise ValueError(f"Axis 2 target must be within 0?{AXIS_TRAVEL_MM:g} mm and speed must be finite and positive.")
        self._axis2.move_absolute(
            self._pos2_0 + pos,
            Units.LENGTH_MILLIMETRES,
            velocity=vel,
            velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND,
            wait_until_idle=False,
        )

    def move_velocity_ax1(self, speed):
        self._allow_motion()
        if not isfinite(speed):
            raise ValueError("Axis 1 speed must be finite.")
        self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)

    def move_velocity_ax2(self, speed):
        self._allow_motion()
        if not isfinite(speed):
            raise ValueError("Axis 2 speed must be finite.")
        self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)

    def perform_autoload(self, target_load1, target_load2):
        self.arm_force_guard()
        self._execute_autoloading = True
        self._target_load1 = target_load1
        self._target_load2 = target_load2
        self._force1, self._force2 = self.get_forces()
        self._init_offset_1 = self._target_load1 - self._force1
        self._init_offset_2 = self._target_load2 - self._force2
        logger.info("Starting autoload: offsets=(%s, %s)", self._init_offset_1, self._init_offset_2)

        if self._can_start_autoload():
            self._start_time = round(time.perf_counter(), 5)
            self._current_time = 0
            self._autoload_timer.start(250)
            self.signal_autoload_started.emit(target_load1, target_load2)
            return True

        self._execute_autoloading = False
        message = "Autoload rejected: the difference between current and desired forces is too high."
        self._force_guard = False
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
        try:
            self._run_autoload_step()
        except Exception as exc:
            self.force_fault(str(exc))

    def _run_autoload_step(self):
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
                self._force_guard = False
                self.signal_autoloading_finished.emit()
                return

            kp = 1.5 if self._units == Unit.Newton else 0.015
            max_velocity = 0.2
            v1 = min(max_velocity, max(-max_velocity, -kp * offset1))
            v2 = min(max_velocity, max(-max_velocity, -kp * offset2))
            self.signal_autoload_progress.emit(self._force1, self._force2, v1, v2)
            self.move_velocity_ax1(v1)
            self.move_velocity_ax2(v2)
            return

        self._execute_autoloading = False
        self.stop_motors()
        self._force_guard = False
        self._stop_timer(self._autoload_timer)
        self.signal_autoload_failed.emit("Autoload timed out before reaching the requested load.")

    def _emit_live_forces(self):
        if not self._connected:
            return
        try:
            force1, force2 = self.get_av_forces()
        except Exception as exc:  # pragma: no cover - hardware path
            if self._telemetry_error != str(exc):
                self._telemetry_error = str(exc)
                self.signal_error.emit(str(exc))
            return
        self._telemetry_error = None
        self.signal_live_forces.emit(force1, force2)


class MotorDAQInterface(QObject):
    COMMAND_TIMEOUT = 5.0

    signal_autoload_started = pyqtSignal(float, float)
    signal_autoloading_finished = pyqtSignal()
    signal_autoloading_progress = pyqtSignal(float, float, float, float)
    signal_autoload_rejected = pyqtSignal(str)
    signal_autoload_failed = pyqtSignal(str)
    signal_command_finished = pyqtSignal(object)
    signal_live_forces = pyqtSignal(float, float)
    signal_error = pyqtSignal(str)
    signal_force_fault = pyqtSignal(str)
    signal_hardware_connected = pyqtSignal()
    signal_hardware_disconnected = pyqtSignal()

    _command_requested = pyqtSignal(object)

    def __init__(self, unit, ip_motor="172.31.100.108", ip_daq="172.31.100.107", force_config=None):
        super().__init__()
        self._units = unit
        self._ip_motor = ip_motor
        self._thread = QThread()
        self._pending = {}
        self._motion_cancelled = Event()
        self._closing = False
        self._worker = HardwareWorker(force_config or ForceConfig())
        self._worker.moveToThread(self._thread)
        self._command_requested.connect(self._worker.handle_command)
        self._worker.signal_command_finished.connect(self._complete_command)
        self._thread.finished.connect(self._worker.deleteLater)
        self._worker.signal_live_forces.connect(self.signal_live_forces.emit)
        self._worker.signal_hardware_connected.connect(self.signal_hardware_connected.emit)
        self._worker.signal_hardware_disconnected.connect(self.signal_hardware_disconnected.emit)
        self._worker.signal_error.connect(self.signal_error.emit)
        self._worker.signal_force_fault.connect(self.signal_force_fault.emit)
        self._worker.signal_autoload_started.connect(self.signal_autoload_started.emit)
        self._worker.signal_autoload_progress.connect(self.signal_autoloading_progress.emit)
        self._worker.signal_autoloading_finished.connect(self.signal_autoloading_finished.emit)
        self._worker.signal_autoload_rejected.connect(self.signal_autoload_rejected.emit)
        self._worker.signal_autoload_failed.connect(self.signal_autoload_failed.emit)
        self._deadline_timer = QTimer(self)
        self._deadline_timer.timeout.connect(self._complete_command)
        self._deadline_timer.start(50)
        self._thread.start()
        self._invoke("connect_hardware", unit, ip_motor, ip_daq, wait=False)

    def _invoke(self, method_name, *args, wait=None, **kwargs):
        cleanup = method_name in ("stop_test", "stop_motors", "stop_motor1", "stop_motor2", "disconnect_hardware", "force_fault")
        if not self._thread.isRunning() or (self._closing and not cleanup):
            raise RuntimeError("Hardware worker is unavailable.")
        if not cleanup and QThread.currentThread().isInterruptionRequested():
            raise CancelledError("Test was stopped.")
        if cleanup:
            self._motion_cancelled.set()
            self._motion_cancelled = Event()
        elif self._worker._motion_blocked.is_set():
            raise RuntimeError("Motion blocked after a hardware error. Reconnect hardware to reset.")
        request = CommandRequest(method_name, args, kwargs,
                                 deadline=time.monotonic() + self.COMMAND_TIMEOUT,
                                 cancelled=self._motion_cancelled if not cleanup else Event())
        if wait is None:
            wait = QThread.currentThread() is not self.thread()
        if not wait:
            self._pending[id(request)] = request
        self._command_requested.emit(request)
        if not wait:
            return None
        if not request.event.wait(self.COMMAND_TIMEOUT):
            request.cancelled.set()
            self._worker._motion_blocked.set()
            message = f"Hardware command {method_name} timed out; motor stop is unconfirmed."
            self._invoke("force_fault", message, wait=False)
            raise TimeoutError(message)
        if request.error is not None:
            raise request.error
        return request.result

    @pyqtSlot()
    @pyqtSlot(object)
    def _complete_command(self, request=None):
        # Also called by the GUI timer, so stalled commands cannot hide indefinitely.
        for pending in list(self._pending.values()):
            if not pending.event.is_set():
                if time.monotonic() < pending.deadline and self._thread.isRunning():
                    continue
                pending.cancelled.set()
                self._worker._motion_blocked.set()
                pending.error = TimeoutError(f"Hardware command {pending.method_name} timed out; motor stop is unconfirmed.")
                if not self._closing and self._thread.isRunning():
                    self.stop()
            if self._pending.pop(id(pending), None) is None:
                continue
            if isinstance(pending.error, CancelledError):
                continue
            if pending.error is not None:
                self.signal_error.emit(str(pending.error))
            self.signal_command_finished.emit(pending)
        if not self._thread.isRunning() and not self._pending:
            self._deadline_timer.stop()

    def force_fault(self, message):
        self._invoke("force_fault", message)

    def arm_force_guard(self):
        self._invoke("arm_force_guard")

    def is_initialized(self):
        return self._thread.isRunning() and not self._closing and self._worker._connected and self._worker.is_initialized()

    def change_units(self, unit):
        self._units = unit
        self._invoke("change_units", unit)

    def stop(self):
        if not self._closing and self._thread.isRunning():
            self._closing = True
            self._invoke("disconnect_hardware", wait=False)

    def stop_test(self):
        self._invoke("stop_test")

    def stop_motors(self):
        self._invoke("stop_motors")

    def stop_motor1(self):
        self._invoke("stop_motor1")

    def stop_motor2(self):
        self._invoke("stop_motor2")

    def get_forces(self):
        return self._worker.get_forces()

    def get_av_forces(self, n=3):
        return self._worker.get_av_forces(n)

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

    def perform_autoload(self, target_load1, target_load2):
        return self._invoke("perform_autoload", target_load1, target_load2)
