# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

import logging
from enum import Enum, auto
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
import time
from datetime import datetime
import os

from .mechanicaltest import MechanicalTest
from .configuration import AXIS_TRAVEL_MM


logger = logging.getLogger(__name__)


class DisplacementTestPhase(Enum):
    STRETCH = auto()
    RETURN = auto()
    FINISHED = auto()


class DisplacementControlTest(MechanicalTest):
    signal_phase_changed = pyqtSignal(object)

    _position_tolerance = 0.08
    _max_test_time = 1000

    def __init__(self, mot_daq, folder, sam_name, vel1, vel2, len1, len2, num_cycles):
        super().__init__(mot_daq)

        self._mot_daq = mot_daq
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2
        self._max_disp1 = len1
        self._max_disp2 = len2
        self._workfolder = folder
        self._sam_name = sam_name
        self._num_cycles = num_cycles
        self._execute = True

    def update_parameters(self, folder, sam_name, vel1, vel2, len1, len2, num_cycles):
        self._workfolder = folder
        self._sam_name = sam_name
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2
        self._max_disp1 = len1
        self._max_disp2 = len2
        self._num_cycles = num_cycles

    def run(self):
        try:
            self._run_test()
        except Exception as exc:
            self._fail(exc)

    def _run_test(self):
        if not self._execute:
            return
        self._init_variables()
        origins = self._mot_daq.zeroPosition()
        if any(not 0 <= origin - length <= AXIS_TRAVEL_MM for origin, length in zip(origins, (self._max_disp1, self._max_disp2))):
            raise ValueError("Requested displacement exceeds available axis travel.")
        if not self._execute:
            return
        self._mot_daq.arm_force_guard()
        if not self._execute:
            return
        self._completed_half_cycles = 0
        self._remaining_half_cycles = 2 * self._num_cycles
        self._start_time = time.perf_counter()
        self._phase_started_at = self._start_time
        self._current_time = 0.0
        self._current_cycle_time = 0.0

        if self._use_video:
            self.signal_start_stop_tracking.emit(True)
            self.signal_save_image.emit(self._build_image_path("first_frame"))

        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        self._file_name = "DTest_" + self._sam_name + "_" + formatted_datetime

        self._pos1, self._pos2 = self._mot_daq.get_positions()
        self._start_pos1 = self._pos1
        self._start_pos2 = self._pos2
        self._stretch_target_pos1 = self._start_pos1 - self._max_disp1
        self._stretch_target_pos2 = self._start_pos2 - self._max_disp2
        self._return_target_pos1 = self._start_pos1
        self._return_target_pos2 = self._start_pos2

        self._transition_to(DisplacementTestPhase.STRETCH)

        self._test_timer = QTimer()
        self._test_timer.timeout.connect(self._tick, Qt.ConnectionType.DirectConnection)
        self._test_timer.start(self._sample_time)
        self.exec()

    def _tick(self):
        try:
            self._snapshot_markers()
            if not self._execute or self._phase == DisplacementTestPhase.FINISHED:
                self._finish_test()
                return
            self._pos1, self._pos2 = self._mot_daq.get_positions()
            self._force1, self._force2 = self._mot_daq.get_forces()
            self._current_time = time.perf_counter() - self._start_time
            self._current_cycle_time = time.perf_counter() - self._phase_started_at
            self._record_sample()
            next_phase = self._next_phase_after_current_sample()
            if next_phase is not None:
                self._transition_to(next_phase)
        except Exception as exc:
            self._fail(exc)

    def _record_sample(self):
        self._update_arrays_emit_data()

    def _next_phase_after_current_sample(self):
        if self._current_time >= self._max_test_time:
            return DisplacementTestPhase.FINISHED

        if self._phase == DisplacementTestPhase.STRETCH and self._is_phase_complete():
            self._completed_half_cycles += 1
            self._remaining_half_cycles -= 1
            if self._remaining_half_cycles <= 0:
                self._write_current_cycle(final_write=True)
                return DisplacementTestPhase.FINISHED
            return DisplacementTestPhase.RETURN

        if self._phase == DisplacementTestPhase.RETURN and self._is_phase_complete():
            self._completed_half_cycles += 1
            self._remaining_half_cycles -= 1
            self._write_current_cycle(final_write=True)
            if self._remaining_half_cycles <= 0:
                return DisplacementTestPhase.FINISHED
            self._init_variables()
            return DisplacementTestPhase.STRETCH

        return None

    def _is_phase_complete(self):
        if self._phase == DisplacementTestPhase.STRETCH:
            return (
                self._pos1 <= self._stretch_target_pos1 + self._position_tolerance
                and self._pos2 <= self._stretch_target_pos2 + self._position_tolerance
            )
        if self._phase == DisplacementTestPhase.RETURN:
            return (
                self._pos1 >= self._return_target_pos1 - self._position_tolerance
                and self._pos2 >= self._return_target_pos2 - self._position_tolerance
            )
        return True

    def _transition_to(self, phase):
        self._phase = phase
        self._phase_started_at = time.perf_counter()

        if phase == DisplacementTestPhase.FINISHED:
            self._finish_test()
            return

        if phase == DisplacementTestPhase.STRETCH:
            logger.debug("Displacement test: stretching sample")
            self._mot_daq.move_position_ax1(self._stretch_target_pos1, self._vel_ax1)
            self._mot_daq.move_position_ax2(self._stretch_target_pos2, self._vel_ax2)
        elif phase == DisplacementTestPhase.RETURN:
            logger.debug("Displacement test: returning sample")
            if self._use_video and self._remaining_half_cycles == 1:
                self.signal_save_image.emit(self._build_image_path("last_frame"))
            self._mot_daq.move_position_ax1(self._return_target_pos1, self._vel_ax1)
            self._mot_daq.move_position_ax2(self._return_target_pos2, self._vel_ax2)

        self.signal_phase_changed.emit(phase)

    def _write_current_cycle(self, final_write):
        cycle_num = self._completed_half_cycles / 2
        self._writeDataToFile(self._file_name, cycle_num, final_write)

    def _finish_test(self):
        if getattr(self, "_phase", None) == DisplacementTestPhase.FINISHED and not self._execute:
            return
        self._phase = DisplacementTestPhase.FINISHED
        self._execute = False
        self._dispose_test_timer()
        logger.info("Displacement-control test finished")
        self.stop_measurement()
        self.signal_phase_changed.emit(DisplacementTestPhase.FINISHED)
        self.signal_test_finished.emit()
        self.quit()

    def _build_image_path(self, suffix):
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        file_name = f"Test_{self._sam_name}_{formatted_datetime}_{suffix}.jpg"
        return os.path.join(self._workfolder, self._sam_name, file_name)
