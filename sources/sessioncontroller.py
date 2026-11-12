from __future__ import annotations

import os

from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot

from .configuration import MachineConfig
from .displacementtest import DisplacementControlTest
from .hardware import MotorDAQInterface
from .loadtest import LoadControlTest
from .sessionstate import SessionState


class TestSessionController(QObject):
    signal_error = pyqtSignal(str)
    signal_live_forces = pyqtSignal(float, float)
    signal_reset_charts = pyqtSignal()
    signal_save_charts = pyqtSignal()
    signal_ui_block_requested = pyqtSignal(bool)
    signal_test_created = pyqtSignal(object)
    signal_state_changed = pyqtSignal(object)
    signal_status_message = pyqtSignal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._state = SessionState.DISCONNECTED
        self._mot_daq = None
        self._active_test = None
        self._current_config = None
        self._active_tab_index = None

    @property
    def hardware(self):
        return self._mot_daq

    @property
    def active_test(self):
        return self._active_test

    @property
    def state(self):
        return self._state

    def _set_state(self, state: SessionState):
        if self._state == state:
            return
        self._state = state
        self.signal_state_changed.emit(state)

    def connect_hardware(self, config: MachineConfig):
        if self._mot_daq is not None and self._mot_daq._thread.isRunning():
            return self._mot_daq
        self._mot_daq = MotorDAQInterface(
            config.unit_enum,
            ip_motor=config.runtime.ip_motor,
            force_config=config.force,
        )
        self._mot_daq.signal_command_finished.connect(self._handle_hardware_command)
        self._mot_daq.signal_live_forces.connect(self.signal_live_forces.emit)
        self._mot_daq.signal_error.connect(self.signal_error.emit)
        self._mot_daq.signal_force_fault.connect(self._handle_force_fault)
        self._mot_daq.signal_autoload_started.connect(self._handle_autoload_started)
        self._mot_daq.signal_autoloading_finished.connect(self._handle_autoload_finished)
        self._mot_daq.signal_autoload_rejected.connect(self._handle_autoload_rejected)
        self._mot_daq.signal_autoload_failed.connect(self._handle_autoload_failed)
        self._mot_daq.signal_hardware_connected.connect(self._handle_hardware_connected)
        self._mot_daq.signal_hardware_disconnected.connect(self._handle_hardware_disconnected)
        self._current_config = config
        return self._mot_daq

    def disconnect_hardware(self):
        if self._mot_daq is None:
            return
        self._set_state(SessionState.STOPPING)
        self._mot_daq.stop()

    def update_units(self, config: MachineConfig):
        self._current_config = config
        if self._mot_daq is not None:
            self._mot_daq.change_units(config.unit_enum)

    def zero_force(self):
        self._require_hardware()
        self._mot_daq.zeroForce()
        self._refresh_ready_state()

    def zero_position(self):
        self._require_hardware()
        self._mot_daq.zeroPosition()
        self._refresh_ready_state()

    def move_sample_position(self):
        self._require_hardware()
        self._mot_daq.moveSamplePosition()

    def initialize_motors(self):
        self._require_hardware()
        self._mot_daq.initMotZeroPos()

    def move_velocity_ax1(self, speed):
        self._require_hardware()
        self._mot_daq.move_velocity_ax1(speed)

    def move_velocity_ax2(self, speed):
        self._require_hardware()
        self._mot_daq.move_velocity_ax2(speed)

    def stop_motors(self):
        if self._mot_daq is not None:
            self._mot_daq.stop_motors()

    def perform_autoload(self, config: MachineConfig):
        config.validate()
        self._require_hardware()
        if (self._active_test is not None and self._active_test.isRunning()) or self._state in (
            SessionState.RUNNING_PRECONDITION,
            SessionState.RUNNING_TEST,
            SessionState.STOPPING,
        ):
            raise RuntimeError("Stop the current test before starting manual autoload.")
        self._set_state(SessionState.AUTOLOADING)
        result = self._mot_daq.perform_autoload(
            float(config.load_test.tareload1),
            float(config.load_test.tareload2),
        )
        return result

    def prepare_test(self, config: MachineConfig, tab_index: int, allow_existing_sample: bool = False):
        config.validate(tab_index)
        self._require_hardware()
        sample_name = config.runtime.sample_name.strip()
        if not sample_name:
            raise ValueError("Sample name is required.")

        sample_folder = os.path.join(config.runtime.folder, sample_name)
        if os.path.exists(sample_folder) and not allow_existing_sample:
            return "sample_exists"

        self._current_config = config
        self._active_test = self._create_or_update_test(config, tab_index)
        self._active_tab_index = tab_index
        self.signal_test_created.emit(self._active_test)
        return self._active_test

    def start_test(self, config: MachineConfig, tab_index: int, allow_existing_sample: bool = False):
        if self._active_test is not None and self._active_test.isRunning():
            raise RuntimeError("A test is already running.")
        if self._state in (SessionState.AUTOLOADING, SessionState.RUNNING_PRECONDITION,
                            SessionState.RUNNING_TEST, SessionState.STOPPING, SessionState.ERROR):
            raise RuntimeError("Hardware is busy or faulted. Stop or reconnect before starting a test.")

        result = self.prepare_test(config, tab_index, allow_existing_sample=allow_existing_sample)
        if result == "sample_exists":
            return result

        if not self._mot_daq.is_initialized():
            raise RuntimeError("Motors and DAQ not connected or not initialized!")

        self.signal_reset_charts.emit()
        self.signal_ui_block_requested.emit(True)
        self._set_state(SessionState.RUNNING_PRECONDITION if tab_index == 0 else SessionState.RUNNING_TEST)
        result._execute = True
        try:
            result.start()
        except Exception:
            self._active_test = None
            self._active_tab_index = None
            self.stop_test()
            raise
        return result

    def stop_test(self):
        self._set_state(SessionState.STOPPING)
        try:
            if self._active_test is not None:
                self._active_test.stop()
        finally:
            if self._mot_daq is not None and self._mot_daq._thread.isRunning():
                self._mot_daq.stop_test()
            else:
                self.signal_ui_block_requested.emit(False)
                self._set_state(SessionState.DISCONNECTED)

    def shutdown(self):
        try:
            self.stop_test()
        finally:
            self.disconnect_hardware()

    @pyqtSlot(str)
    def _handle_force_fault(self, message):
        self._set_state(SessionState.ERROR)
        try:
            if self._active_test is not None:
                self._active_test.stop()
        finally:
            self.signal_ui_block_requested.emit(False)
            self.signal_error.emit(message + " Reconnect hardware to reset.")

    def _require_hardware(self):
        if self._mot_daq is None or not self._mot_daq._thread.isRunning() or self._mot_daq._closing or not self._mot_daq._worker._connected:
            raise RuntimeError("Connect to motors and DAQ first!")
        if self._state in (SessionState.STOPPING, SessionState.ERROR):
            raise RuntimeError("Hardware is stopping or faulted. Reconnect hardware to reset.")

    def _refresh_ready_state(self):
        if self._mot_daq is None or not self._mot_daq._thread.isRunning():
            self._set_state(SessionState.DISCONNECTED)
        elif self._mot_daq._closing:
            self._set_state(SessionState.STOPPING)
        elif self._mot_daq._worker._motion_blocked.is_set():
            self._set_state(SessionState.ERROR)
        elif self._mot_daq.is_initialized():
            self._set_state(SessionState.READY)
        else:
            self._set_state(SessionState.IDLE)

    def _build_test(self, config: MachineConfig, tab_index: int):
        if tab_index == 0:
            test = LoadControlTest(
                self._mot_daq,
                config.runtime.folder,
                config.runtime.sample_name,
                config.load_test.autoloading,
                float(config.load_test.load_test1),
                float(config.load_test.load_test2),
                float(config.load_test.tareload1),
                float(config.load_test.tareload2),
                float(config.load_test.disp_guess1) / 2,
                float(config.load_test.disp_guess2) / 2,
                float(config.load_test.halfcycltime),
                int(config.load_test.cycleprecond),
                int(config.load_test.cycletest),
            )
        elif tab_index == 2:
            test = DisplacementControlTest(
                self._mot_daq,
                config.runtime.folder,
                config.runtime.sample_name,
                float(config.displacement_test.disptestspeed1) / 120,
                float(config.displacement_test.disptestspeed2) / 120,
                float(config.displacement_test.disptestlen1) / 2,
                float(config.displacement_test.disptestlen2) / 2,
                int(config.displacement_test.disptestcycle),
            )
        else:
            raise ValueError("The selected tab is not supported.")

        test.signal_precond_finished.connect(self._handle_precond_finished)
        test.finished.connect(self._handle_test_finished)
        test.signal_error.connect(self._handle_force_fault)
        if hasattr(test, "signal_phase_changed"):
            test.signal_phase_changed.connect(self._handle_load_test_phase_changed)
        return test

    def _create_or_update_test(self, config: MachineConfig, tab_index: int):
        if self._active_test is None or self._active_tab_index != tab_index:
            return self._build_test(config, tab_index)

        if tab_index == 0:
            self._active_test.update_parameters(
                config.runtime.folder,
                config.runtime.sample_name,
                config.load_test.autoloading,
                float(config.load_test.load_test1),
                float(config.load_test.load_test2),
                float(config.load_test.tareload1),
                float(config.load_test.tareload2),
                float(config.load_test.disp_guess1) / 2,
                float(config.load_test.disp_guess2) / 2,
                float(config.load_test.halfcycltime),
                int(config.load_test.cycleprecond),
                int(config.load_test.cycletest),
            )
            return self._active_test

        if tab_index == 2:
            self._active_test.update_parameters(
                config.runtime.folder,
                config.runtime.sample_name,
                float(config.displacement_test.disptestspeed1) / 120,
                float(config.displacement_test.disptestspeed2) / 120,
                float(config.displacement_test.disptestlen1) / 2,
                float(config.displacement_test.disptestlen2) / 2,
                int(config.displacement_test.disptestcycle),
            )
            return self._active_test

        raise ValueError("The selected tab is not supported.")

    @pyqtSlot()
    def _handle_precond_finished(self):
        self._set_state(SessionState.RUNNING_TEST)
        self.signal_reset_charts.emit()

    @pyqtSlot()
    def _handle_test_finished(self):
        failed = self._state == SessionState.ERROR or (self._active_test is not None and self._active_test._failure is not None)
        self._active_test = None
        self._active_tab_index = None
        if self._mot_daq is not None and self._mot_daq._thread.isRunning() and not self._mot_daq._closing:
            self._mot_daq.stop_test()
        if not failed:
            self.signal_save_charts.emit()
        self.signal_ui_block_requested.emit(False)
        self._refresh_ready_state()

    @pyqtSlot(float, float)
    def _handle_autoload_started(self, _target1, _target2):
        self._set_state(SessionState.AUTOLOADING)

    @pyqtSlot()
    def _handle_autoload_finished(self):
        if self._active_test is not None:
            if getattr(self._active_test, "_state", None) is not None and getattr(self._active_test._state, "name", "") == "PRECONDITIONING":
                self._set_state(SessionState.RUNNING_PRECONDITION)
            else:
                self._set_state(SessionState.RUNNING_TEST)
        else:
            self._refresh_ready_state()

    @pyqtSlot(object)
    def _handle_load_test_phase_changed(self, phase):
        phase_name = getattr(phase, "name", "")
        if phase_name == "AUTOLOAD":
            self._set_state(SessionState.AUTOLOADING)
        elif phase_name.startswith("PRECONDITION"):
            self._set_state(SessionState.RUNNING_PRECONDITION)
        elif phase_name.startswith("TEST"):
            self._set_state(SessionState.RUNNING_TEST)

    @pyqtSlot(str)
    def _handle_autoload_rejected(self, message):
        self.signal_error.emit(message)
        self._refresh_ready_state()

    @pyqtSlot(str)
    def _handle_autoload_failed(self, message):
        self.signal_error.emit(message)
        self._refresh_ready_state()

    @pyqtSlot(object)
    def _handle_hardware_command(self, request):
        if request.error is not None:
            self._set_state(SessionState.ERROR)
            if self._active_test is not None:
                self._active_test.stop()
            self.signal_ui_block_requested.emit(False)
        elif request.method_name in ("zeroForce", "zeroPosition", "stop_test"):
            if self._active_test is None or not self._active_test.isRunning():
                self.signal_ui_block_requested.emit(False)
                self._refresh_ready_state()

    @pyqtSlot()
    def _handle_hardware_connected(self):
        self._set_state(SessionState.IDLE)

    @pyqtSlot()
    def _handle_hardware_disconnected(self):
        self._set_state(SessionState.DISCONNECTED)
