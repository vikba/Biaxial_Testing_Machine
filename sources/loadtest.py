# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

from enum import Enum, auto
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
import numpy as np
import time
from datetime import datetime
import os

from .mechanicaltest import MechanicalTest
from .state import State
from .unit import Unit


class LoadTestPhase(Enum):
    PRECONDITION_STRETCH = auto()
    PRECONDITION_RETURN = auto()
    AUTOLOAD = auto()
    TEST_STRETCH = auto()
    TEST_RETURN = auto()
    FINISHED = auto()


class LoadControlTest(MechanicalTest):
    """
    num_cycles:
    0 - one directional (half cycle)
    1 - one cycle
    etc
    """

    signal_phase_changed = pyqtSignal(object)

    _autoload_timeout = 30.0
    _autoload_settle_time = 0.5
    _max_test_time = 3000

    def __init__(
        self,
        mot_daq,
        folder,
        sam_name,
        autoloading,
        max_force1,
        max_force2,
        tare_load1,
        tare_load2,
        disp1,
        disp2,
        test_duration,
        num_cycles_precond,
        num_cycles_test,
    ):
        super().__init__(mot_daq)
        self._execute = False
        self._mot_daq = mot_daq
        self._max_force1 = max_force1
        self._max_force2 = max_force2
        self._test_duration = test_duration
        self._fl_autoloading = autoloading
        self._tare_load1 = tare_load1
        self._tare_load2 = tare_load2
        self._workfolder = folder
        self._sam_name = sam_name
        self._num_cycles_precond = num_cycles_precond
        self._num_cycles_test = num_cycles_test
        self._disp_guess1 = disp1
        self._disp_guess2 = disp2
        print("Load Control Test created.")

    @property
    def phase(self):
        return getattr(self, "_phase", None)

    def update_parameters(
        self,
        folder,
        sam_name,
        autoloading,
        max_force1,
        max_force2,
        tare_load1,
        tare_load2,
        disp1,
        disp2,
        test_duration,
        num_cycles_precond,
        num_cycles_test,
    ):
        self._workfolder = folder
        self._sam_name = sam_name
        self._fl_autoloading = autoloading
        self._max_force1 = max_force1
        self._max_force2 = max_force2
        self._tare_load1 = tare_load1
        self._tare_load2 = tare_load2
        self._disp_guess1 = disp1
        self._disp_guess2 = disp2
        self._test_duration = test_duration
        self._num_cycles_precond = num_cycles_precond
        self._num_cycles_test = num_cycles_test
        print("Load Control Test updated.")

    def run(self):
        try:
            self._run_test()
        except Exception as exc:
            self._fail(exc)

    def _run_test(self):
        print("LoadControlTest: Started")
        if not self._execute:
            return
        self._state = State.PRECONDITIONING
        self._init_variables()
        self._start_time = time.perf_counter()
        self._phase_started_at = self._start_time
        self._current_time = 0.0
        self._current_cycle_time = 0.0
        self._remaining_precond_half_cycles = 2 * self._num_cycles_precond
        self._remaining_test_half_cycles = 2 * self._num_cycles_test
        self._test_video_reference_ready = False
        self._autoload_settled_since = None
        self._autoload_started_at = None
        self._last_sample_has_video_strain = False

        if self._use_video:
            print(f"Use video. Point1: {self._point1}")
            self.signal_start_stop_tracking.emit(True)
            self.signal_save_image.emit(self._build_image_path("first_frame"))

        self._mot_daq.zeroPosition()
        if not self._execute:
            return
        self._mot_daq.arm_force_guard()
        if not self._execute:
            return
        self._write_points_reference(self._sample_folder())
        self._vel_ax1 = round(self._disp_guess1 / self._test_duration, 5)
        self._vel_ax2 = round(self._disp_guess2 / self._test_duration, 5)
        self._set_file_name("Precond")

        if self._remaining_precond_half_cycles > 0:
            first_phase = LoadTestPhase.PRECONDITION_STRETCH
        else:
            self._state = State.TEST
            self._set_file_name("Test")
            first_phase = self._initial_test_phase()
        self._transition_to(first_phase)

        self._test_timer = QTimer()
        self._test_timer.timeout.connect(self._tick, Qt.ConnectionType.DirectConnection)
        self._test_timer.start(self._sample_time)
        print("Timer started")
        self.exec()

    def _tick(self):
        try:
            self._control_tick()
        except Exception as exc:
            self._fail(exc)

    def _control_tick(self):
        if not self._execute or self._phase == LoadTestPhase.FINISHED:
            self._finish_test()
            return

        self._current_time = round(time.perf_counter() - self._start_time, 5)
        self._current_cycle_time = round(time.perf_counter() - self._phase_started_at, 5)
        self._force1, self._force2 = self._mot_daq.get_forces()
        self._pos1, self._pos2 = self._mot_daq.get_positions()
        self._av_force1 = self._average_recent_load(self._load1, self._force1)
        self._av_force2 = self._average_recent_load(self._load2, self._force2)

        self._record_sample()
        self._apply_phase_control()
        next_phase = self._next_phase_after_current_sample()

        if next_phase is not None:
            self._transition_to(next_phase)

    def _record_sample(self):
        include_video_strain = self._should_record_video_strain()
        self._last_sample_has_video_strain = include_video_strain
        self._update_arrays_emit_data(include_video_strain=include_video_strain)

    def _apply_phase_control(self):
        if self._phase != LoadTestPhase.AUTOLOAD:
            return

        offset1 = self._tare_load1 - self._force1
        offset2 = self._tare_load2 - self._force2
        threshold1, threshold2 = self._autoload_thresholds()
        within_limits = abs(offset1) <= threshold1 and abs(offset2) <= threshold2

        if within_limits:
            if self._autoload_settled_since is None:
                self._autoload_settled_since = time.perf_counter()
            self._mot_daq.stop_motors()
            return

        self._autoload_settled_since = None
        kp = 1.5 if self._mot_daq._units == Unit.Newton else 0.015
        max_velocity = 0.2
        self._vel_ax1 = min(max_velocity, max(-max_velocity, -kp * offset1))
        self._vel_ax2 = min(max_velocity, max(-max_velocity, -kp * offset2))
        self._mot_daq.move_velocity_ax1(self._vel_ax1)
        self._mot_daq.move_velocity_ax2(self._vel_ax2)

    def _next_phase_after_current_sample(self):
        if self._current_time >= self._max_test_time:
            return LoadTestPhase.FINISHED

        if self._phase == LoadTestPhase.PRECONDITION_STRETCH and not self._stretch_should_continue():
            self._remaining_precond_half_cycles -= 1
            if self._remaining_precond_half_cycles <= 0:
                return self._start_main_test()
            return LoadTestPhase.PRECONDITION_RETURN

        if self._phase == LoadTestPhase.PRECONDITION_RETURN and not self._return_should_continue():
            self._remaining_precond_half_cycles -= 1
            self._persist_finished_cycle()
            if self._remaining_precond_half_cycles <= 0:
                return self._start_main_test()
            self._init_variables()
            return LoadTestPhase.PRECONDITION_STRETCH

        if self._phase == LoadTestPhase.AUTOLOAD:
            if time.perf_counter() - self._autoload_started_at >= self._autoload_timeout:
                self._mot_daq.stop_motors()
                self._mot_daq.signal_error.emit("Autoload timed out before reaching the requested load.")
                return LoadTestPhase.FINISHED
            if self._autoload_settled_since is not None and time.perf_counter() - self._autoload_settled_since >= self._autoload_settle_time:
                self._mot_daq.stop_motors()
                return LoadTestPhase.TEST_STRETCH
            return None

        if self._phase == LoadTestPhase.TEST_STRETCH and not self._stretch_should_continue():
            self._remaining_test_half_cycles -= 1
            if self._remaining_test_half_cycles <= 0:
                self._persist_finished_cycle(final_write=True)
                return LoadTestPhase.FINISHED
            return LoadTestPhase.TEST_RETURN

        if self._phase == LoadTestPhase.TEST_RETURN and not self._return_should_continue():
            self._remaining_test_half_cycles -= 1
            self._persist_finished_cycle()
            if self._remaining_test_half_cycles <= 0:
                return LoadTestPhase.FINISHED
            self._init_variables()
            return self._initial_test_phase()

        return None

    def _transition_to(self, phase):
        self._phase = phase
        self._phase_started_at = time.perf_counter()

        if phase == LoadTestPhase.FINISHED:
            self._finish_test()
            return

        if phase == LoadTestPhase.PRECONDITION_STRETCH:
            self._state = State.PRECONDITIONING
            self._vel_ax1 = round(self._disp_guess1 / self._test_duration, 5)
            self._vel_ax2 = round(self._disp_guess2 / self._test_duration, 5)
            self._mot_daq.move_velocity_ax1(-self._vel_ax1)
            self._mot_daq.move_velocity_ax2(-self._vel_ax2)

        elif phase == LoadTestPhase.PRECONDITION_RETURN:
            self._maybe_report_anisotropy_ratio()
            self._retune_displacement_guess()
            self._mot_daq.move_position_ax1(0, self._vel_ax1)
            self._mot_daq.move_position_ax2(0, self._vel_ax2)

        elif phase == LoadTestPhase.AUTOLOAD:
            self._autoload_started_at = time.perf_counter()
            self._autoload_settled_since = None
            self._vel_ax1 = 0.0
            self._vel_ax2 = 0.0
            self._mot_daq.stop_motors()

        elif phase == LoadTestPhase.TEST_STRETCH:
            self._state = State.TEST
            self._vel_ax1 = round(self._disp_guess1 / self._test_duration, 5)
            self._vel_ax2 = round(self._disp_guess2 / self._test_duration, 5)
            self._refresh_marker_reference_points()
            self._test_video_reference_ready = self._use_video and self._has_latest_markers()
            self._mot_daq.move_velocity_ax1(-self._vel_ax1)
            self._mot_daq.move_velocity_ax2(-self._vel_ax2)

        elif phase == LoadTestPhase.TEST_RETURN:
            self._maybe_save_last_frame()
            self._maybe_report_anisotropy_ratio()
            self._retune_displacement_guess()
            self._mot_daq.move_position_ax1(0, self._vel_ax1)
            self._mot_daq.move_position_ax2(0, self._vel_ax2)

        self.signal_phase_changed.emit(phase)

    def _start_main_test(self):
        self._state = State.TEST
        self._set_file_name("Test")
        self._init_variables()
        self._write_points_reference(self._sample_folder())
        self._test_video_reference_ready = False
        self._autoload_started_at = None
        self._autoload_settled_since = None
        print("LoadControlTest: Preconditioning finished. Main test started.")
        self.signal_precond_finished.emit()
        return self._initial_test_phase()

    def _initial_test_phase(self):
        if self._remaining_test_half_cycles <= 0:
            return LoadTestPhase.FINISHED
        if self._fl_autoloading:
            return LoadTestPhase.AUTOLOAD
        return LoadTestPhase.TEST_STRETCH

    def _stretch_should_continue(self):
        return self._av_force1 < self._max_force1 and self._av_force2 < self._max_force2

    def _return_should_continue(self):
        return self._pos1 <= -0.01 or self._pos2 <= -0.01

    def _should_record_video_strain(self):
        if not self._use_video:
            return False
        if self._state == State.PRECONDITIONING:
            return self._has_latest_markers()
        return self._test_video_reference_ready and self._has_latest_markers()

    def _has_latest_markers(self):
        return all(
            hasattr(self, attr)
            for attr in ("_temp_p1", "_temp_p2", "_temp_p3", "_temp_p4")
        )

    def _refresh_marker_reference_points(self):
        if not self._has_latest_markers():
            return
        self._ref_p1 = self._temp_p1
        self._ref_p2 = self._temp_p2
        self._ref_p3 = self._temp_p3
        self._ref_p4 = self._temp_p4

    def _persist_finished_cycle(self, final_write=False):
        if self._state == State.PRECONDITIONING:
            cycle_num = self._num_cycles_precond - self._remaining_precond_half_cycles / 2
            self._writeDataToFile(self._file_name, cycle_num, False)
            return

        cycle_num = self._num_cycles_test - self._remaining_test_half_cycles / 2
        self._writeDataToFile(self._file_name, cycle_num, final_write)

    def _average_recent_load(self, loads, current_force):
        if len(loads) > 5:
            return sum(loads[-5:]) / 5
        return current_force

    def _autoload_thresholds(self):
        if self._mot_daq._units == Unit.Newton:
            min_threshold = 0.02
        else:
            min_threshold = 2.0
        threshold1 = max(min_threshold, 0.05 * abs(self._tare_load1))
        threshold2 = max(min_threshold, 0.05 * abs(self._tare_load2))
        return threshold1, threshold2

    def _maybe_save_last_frame(self):
        if self._use_video and self._remaining_test_half_cycles == 1:
            self.signal_save_image.emit(self._build_image_path("last_frame"))

    def _retune_displacement_guess(self):
        n = int(0.25 * self._current_cycle_time * 1000 / self._sample_time)
        if n < 2:
            return
        if len(self._time_rel) < n or len(self._load1) < n or len(self._load2) < n:
            return

        y = np.array(self._time_rel[-n:])
        x1 = self._load1[-n:]
        x2 = self._load2[-n:]

        coef1 = np.polyfit(x1, y, 1)
        coef2 = np.polyfit(x2, y, 1)
        time_to_load1 = round(coef1[0] * self._max_force1 + coef1[1], 3)
        time_to_load2 = round(coef2[0] * self._max_force2 + coef2[1], 3)
        corr_fact1 = round(
            ((time_to_load1 - self._test_duration) * 0.8 + self._test_duration) / self._test_duration,
            3,
        )
        corr_fact2 = round(
            ((time_to_load2 - self._test_duration) * 0.8 + self._test_duration) / self._test_duration,
            3,
        )
        self._disp_guess1 = self._disp_guess1 * min(corr_fact1, 1.5)
        self._disp_guess2 = self._disp_guess2 * min(corr_fact2, 1.5)

    def _maybe_report_anisotropy_ratio(self):
        required_points = 50
        if not self._should_record_video_strain():
            return
        if (
            len(self._E11) < required_points
            or len(self._E22) < required_points
            or len(self._load1) < required_points
            or len(self._load2) < required_points
        ):
            return

        try:
            slope1, _ = np.polyfit(self._E11[-required_points:], self._load1[-required_points:], 1)
            slope2, _ = np.polyfit(self._E22[-required_points:], self._load2[-required_points:], 1)
            ar = slope1 / slope2 if slope1 > slope2 else slope2 / slope1
            print(f"Anisotropy ratio as slope ratio: {ar}")
        except Exception as exc:
            print(f"An error occurred: {exc}")

    def _finish_test(self):
        if getattr(self, "_phase", None) == LoadTestPhase.FINISHED and not self._execute:
            return
        self._phase = LoadTestPhase.FINISHED
        self._execute = False
        self._dispose_test_timer()
        self.stop_measurement()
        print("LoadControlTest: Test finished")
        self.signal_phase_changed.emit(LoadTestPhase.FINISHED)
        self.signal_test_finished.emit()
        self.quit()

    def _set_file_name(self, prefix):
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        self._file_name = f"{prefix}_{self._sam_name}_{formatted_datetime}"

    def _build_image_path(self, suffix):
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        file_name = f"Test_{self._sam_name}_{formatted_datetime}_{suffix}.jpg"
        return os.path.join(self._sample_folder(), file_name)

    def _sample_folder(self):
        return os.path.join(self._workfolder, self._sam_name)
