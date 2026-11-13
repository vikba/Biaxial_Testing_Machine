"""

@author: Viktor B

"""

from __future__ import annotations

import logging
import os
import sys
from copy import deepcopy
from importlib.resources import as_file, files
from pathlib import Path

from PyQt6.QtWidgets import QApplication, QFileDialog, QMainWindow, QMessageBox
from PyQt6.uic import loadUi
from PyQt6.QtCore import QTimer, pyqtSlot, pyqtSignal
from PyQt6.QtGui import QFont
import pyqtgraph as pg

from biaxial_testing_machine.configuration import ConfigRepository, MachineConfig
from biaxial_testing_machine.loadcalculator import LoadCalculatorWindow
from biaxial_testing_machine.machineconfigdialog import MachineConfigDialog
from biaxial_testing_machine.ringbuffer import RingBuffer
from biaxial_testing_machine.sessioncontroller import TestSessionController
from biaxial_testing_machine.videotestlink import VideoTestLink
from biaxial_testing_machine.state import State
from biaxial_testing_machine.sessionstate import SessionState
from biaxial_testing_machine.unit import Unit
from biaxial_testing_machine.camerawindow import VideoThread, VideoWindow
from biaxial_testing_machine.loadtest import LoadControlTest
from biaxial_testing_machine.displacementtest import DisplacementControlTest


logger = logging.getLogger(__name__)


class BiaxMainWindow(QMainWindow):
    signal_stop = pyqtSignal()
    _liveforce_update_time = 50

    def __init__(self):
        super().__init__()

        self._fl_sample_initialized = False
        self._fl_executing = False
        self._live_monitor_enabled = True
        self._video_connected_test = None
        self._mec_test = None
        self._mot_daq = None
        self._warning_blink_enabled = False
        self._warning_color_is_red = False

        with as_file(files(__package__).joinpath("resources/biax_ui.ui")) as ui_path:
            loadUi(str(ui_path), self)

        self.__init_variables()
        self._init_buffers()
        self._init_config_repository()
        self._init_controller()
        self._bind_ui()
        self._init_settings_menu()
        self._configure_charts()
        self.populate_ui(self._config)

        self.setWindowTitle("Biaxial testing machine")
        self._set_force_label_color("black")
        self._reset_status_labels()

    def _init_buffers(self):
        buffer_len = 500
        self._ringbuffer1 = RingBuffer(buffer_len)
        self._ringbuffer2 = RingBuffer(buffer_len)
        self._t_label = [i * self._liveforce_update_time / 1000 for i in range(buffer_len)]

    def _init_config_repository(self):
        self._config_path = os.environ.get("BIAXIAL_CONFIG") or str(
            Path(__file__).resolve().parent.parent / "resources" / "config.local.json"
        )
        self._config_repository = ConfigRepository(self._config_path)
        self.load_config()
        if not self._config.runtime.folder:
            home_dir = os.path.expanduser("~")
            self._config.runtime.folder = os.path.join(home_dir, "Desktop", "Biax_Tests")
        self._apply_sample_config(self._config)

    def _init_controller(self):
        self._session_controller = TestSessionController(self)
        self._session_controller.signal_error.connect(self._show_warning)
        self._session_controller.signal_live_forces.connect(self.__update_live_forces)
        self._session_controller.signal_reset_charts.connect(self.reset)
        self._session_controller.signal_save_charts.connect(self._save_charts)
        self._session_controller.signal_ui_block_requested.connect(self._set_test_running_ui)
        self._session_controller.signal_test_created.connect(self._handle_test_created)

    def _bind_ui(self):
        self.buttonStart.clicked.connect(self.__start_test)
        self.buttonStop.clicked.connect(self.__stop_test)
        self.buttonSampleP.clicked.connect(self.__moveSamplePosition)
        self.buttonInitMot.clicked.connect(self.__initializeMotors)
        self.buttonCamera.clicked.connect(self.__startCamera)
        self.buttonFolder.clicked.connect(self.__openFolderDialog)
        self.buttonZeroForce.clicked.connect(self.__zeroForce)
        self.buttonZeroPosition.clicked.connect(self.__zeroPosition)
        self.buttonConnect.clicked.connect(self.__connect)
        self.buttonCalcLoad.clicked.connect(self.__calculate_loads)
        self.buttonAutoload.clicked.connect(self.__autoload)

        self.buttonMoveCentAx1.pressed.connect(lambda: self.__moveAxis(1, 1))
        self.buttonMoveCentAx2.pressed.connect(lambda: self.__moveAxis(2, 1))
        self.buttonMoveBackAx1.pressed.connect(lambda: self.__moveAxis(1, -1))
        self.buttonMoveBackAx2.pressed.connect(lambda: self.__moveAxis(2, -1))

        self.buttonMoveCentAx1.released.connect(self.__stop_movement)
        self.buttonMoveCentAx2.released.connect(self.__stop_movement)
        self.buttonMoveBackAx1.released.connect(self.__stop_movement)
        self.buttonMoveBackAx2.released.connect(self.__stop_movement)

        self.buttonGrams.toggled.connect(self.__change_units)

    def _init_settings_menu(self):
        menu = self.menuBar().addMenu("Settings")
        self._machine_config_action = menu.addAction("Machine configuration…", self._open_machine_configuration)
        self._disconnect_devices_action = menu.addAction("Disconnect devices", self._disconnect_devices)
        menu.aboutToShow.connect(self._update_settings_actions)
        self._settings_timer = QTimer(self)
        self._settings_timer.timeout.connect(self._update_settings_actions)
        self._settings_timer.start(200)
        self._update_settings_actions()

    def _machine_activity(self):
        controller = self._session_controller
        hardware, test = controller.hardware, controller.active_test
        video = getattr(self, "_video_thread", None)
        devices_running = ((hardware is not None and hardware._thread.isRunning())
                           or (video is not None and video.isRunning()))
        busy = (getattr(self, "_closing", False)
                or controller.state in (SessionState.RUNNING_PRECONDITION, SessionState.RUNNING_TEST,
                                        SessionState.AUTOLOADING, SessionState.STOPPING)
                or (test is not None and test.isRunning()))
        return devices_running, busy

    def _can_edit_machine_configuration(self):
        return not any(self._machine_activity())

    def _update_settings_actions(self):
        connected, busy = self._machine_activity()
        self._machine_config_action.setEnabled(not connected and not busy)
        self._machine_config_action.setToolTip("Stop tests and disconnect devices before editing machine settings.")
        self._disconnect_devices_action.setEnabled(bool(connected and not busy))

    def _disconnect_devices(self):
        connected, busy = self._machine_activity()
        if not connected or busy:
            return
        hardware = self._session_controller.hardware
        if hardware is not None and hardware._thread.isRunning():
            self._session_controller.disconnect_hardware()
        if hasattr(self, "_video_thread"):
            self._video_thread.stop()
            self._video_window.close()
        self._update_settings_actions()

    def _open_machine_configuration(self):
        if self._can_edit_machine_configuration():
            candidate = self.read_ui_into_config(deepcopy(self._config), apply=False)
            MachineConfigDialog(candidate, os.path.abspath(self._config_path), self._save_machine_configuration,
                                self._can_edit_machine_configuration, self).exec()

    def _save_machine_configuration(self, candidate):
        if not self._can_edit_machine_configuration():
            raise RuntimeError("Stop tests and disconnect devices before saving machine settings.")
        errors = candidate.machine_settings_errors()
        if errors:
            raise ValueError(next(iter(errors.values())))
        self._config_repository.save(candidate)
        if candidate.runtime.camera_id != self._config.runtime.camera_id and hasattr(self, "_video_thread"):
            self._video_thread.set_camera_id(candidate.runtime.camera_id)
        self._config = candidate
        self._apply_sample_config(candidate)
        self.statusBar().showMessage("Machine settings saved; the next connection will use them.", 5000)

    def _configure_charts(self):
        self.ChartWidget_1.addLegend()
        font = QFont()
        font.setPointSize(15)

        for chart in (self.ChartWidget_1, self.ChartWidget_2, self.ChartWidget_3):
            chart.setBackground("w")
            chart.getAxis("left").setPen(pg.mkPen(color="k", width=1))
            chart.getAxis("bottom").setPen(pg.mkPen(color="k", width=1))
            chart.getAxis("left").setTextPen(pg.mkPen(color="k"))
            chart.getAxis("bottom").setTextPen(pg.mkPen(color="k"))
            chart.showGrid(x=False, y=False, alpha=0.5)
            chart.getPlotItem().getViewBox().setBorder(pg.mkPen(color="k", width=1))
            chart.getAxis("left").setTickFont(font)
            chart.getAxis("bottom").setTickFont(font)

        self.ChartWidget_2.setLimits(minYRange=1.0)
        self.ChartWidget_3.setLimits(minXRange=0.02, minYRange=2.0)
        self.ChartWidget_2.setLabel("left", "Displacement, mm", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
        self.ChartWidget_2.setLabel("bottom", "Time, s", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
        self.ChartWidget_3.setLabel("left", "Stress, kPa", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
        self.ChartWidget_3.setLabel("bottom", "Strain", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})

    def _apply_sample_config(self, config: MachineConfig):
        self._units = config.unit_enum
        self._work_folder = config.runtime.folder
        self._sam_name = config.runtime.sample_name
        self._thickness = config.sample.thickness
        self._len1 = config.sample.length1
        self._len2 = config.sample.length2
        self._max_stress1 = config.sample.maxstress1
        self._max_stress2 = config.sample.maxstress2
        self._area1 = config.sample.area1
        self._area2 = config.sample.area2

    def load_config(self) -> MachineConfig:
        if os.path.exists(self._config_path):
            self._config = self._config_repository.load()
        else:
            example = Path(__file__).resolve().parent.parent / "resources" / "config.example.json"
            if example.is_file():
                self._config = ConfigRepository(str(example)).load()
            else:
                with as_file(files(__package__).joinpath("resources/config.example.json")) as bundled_example:
                    self._config = ConfigRepository(str(bundled_example)).load()
        self._apply_sample_config(self._config)
        return self._config

    def populate_ui(self, config: MachineConfig):
        self._apply_sample_config(config)
        self.buttonGrams.blockSignals(True)
        self.buttonNewtons.blockSignals(True)
        if config.unit_enum == Unit.Gram:
            self.buttonGrams.setChecked(True)
        else:
            self.buttonNewtons.setChecked(True)
        self.buttonGrams.blockSignals(False)
        self.buttonNewtons.blockSignals(False)

        self.labelFolder.setText(config.runtime.folder)
        self.factorSampleName.setText(config.runtime.sample_name)
        self.factorTareLoad1.setText(config.load_test.tareload1)
        self.factorTareLoad2.setText(config.load_test.tareload2)
        self.factorLoadTest1.setText(config.load_test.load_test1)
        self.factorLoadTest2.setText(config.load_test.load_test2)
        self.factorDispGuess1.setText(config.load_test.disp_guess1)
        self.factorDispGuess2.setText(config.load_test.disp_guess2)
        self.factorTimeAx.setText(config.load_test.halfcycltime)
        self.factorCyclNumPrecond.setText(config.load_test.cycleprecond)
        self.factorCyclNumTest.setText(config.load_test.cycletest)
        self.factor_check_box_autoloading.setChecked(config.load_test.autoloading)
        self.factorSpeedAx1.setText(config.displacement_test.disptestspeed1)
        self.factorSpeedAx2.setText(config.displacement_test.disptestspeed2)
        self.factorLength1.setText(config.displacement_test.disptestlen1)
        self.factorLength2.setText(config.displacement_test.disptestlen2)
        self.factorCyclNumD.setText(config.displacement_test.disptestcycle)
        self._update_units_labels(config.unit_enum)

    def read_ui_into_config(self, config: MachineConfig | None = None, *, apply: bool = True) -> MachineConfig:
        cfg = config or self._config
        cfg.runtime.unit = "gram" if self.buttonGrams.isChecked() else "newton"
        cfg.runtime.folder = self._work_folder
        cfg.runtime.sample_name = self.factorSampleName.text().strip()
        cfg.sample.thickness = self._thickness
        cfg.sample.length1 = self._len1
        cfg.sample.length2 = self._len2
        cfg.sample.maxstress1 = self._max_stress1
        cfg.sample.maxstress2 = self._max_stress2
        cfg.load_test.tareload1 = self.factorTareLoad1.text()
        cfg.load_test.tareload2 = self.factorTareLoad2.text()
        cfg.load_test.load_test1 = self.factorLoadTest1.text()
        cfg.load_test.load_test2 = self.factorLoadTest2.text()
        cfg.load_test.disp_guess1 = self.factorDispGuess1.text()
        cfg.load_test.disp_guess2 = self.factorDispGuess2.text()
        cfg.load_test.halfcycltime = self.factorTimeAx.text()
        cfg.load_test.cycleprecond = self.factorCyclNumPrecond.text()
        cfg.load_test.cycletest = self.factorCyclNumTest.text()
        cfg.load_test.autoloading = self.factor_check_box_autoloading.isChecked()
        cfg.displacement_test.disptestspeed1 = self.factorSpeedAx1.text()
        cfg.displacement_test.disptestspeed2 = self.factorSpeedAx2.text()
        cfg.displacement_test.disptestlen1 = self.factorLength1.text()
        cfg.displacement_test.disptestlen2 = self.factorLength2.text()
        cfg.displacement_test.disptestcycle = self.factorCyclNumD.text()
        if apply:
            self._config = cfg
            self._apply_sample_config(cfg)
        return cfg

    def save_config(self, config: MachineConfig, save_sample_copy: bool = False):
        self._config_repository.save(config)
        if not save_sample_copy or not config.runtime.sample_name:
            return
        folder = os.path.join(config.runtime.folder, config.runtime.sample_name)
        os.makedirs(folder, exist_ok=True)
        self._config_repository.save(config, os.path.join(folder, "config_test.json"))

    def __init_variables(self):
        self._t = []
        self._load1 = []
        self._load2 = []
        self._stress1 = []
        self._stress2 = []
        self._disp1 = []
        self._disp2 = []
        self._E11 = []
        self._E22 = []
        self._v1 = []
        self._v2 = []
        self._strain_disp1 = []
        self._strain_disp2 = []

    @pyqtSlot()
    def reset(self):
        self.__init_variables()
        self.ChartWidget_1.clear()
        self.ChartWidget_2.clear()
        self.ChartWidget_3.clear()
        self._reset_status_labels()

    def closeEvent(self, event):
        if not getattr(self, "_closing", False):
            self._closing = True
            hardware = self._session_controller.hardware
            test = self._session_controller.active_test
            if hardware is not None:
                hardware._thread.finished.connect(self.close)
            if test is not None:
                test.finished.connect(self.close)
            if hasattr(self, "_video_thread"):
                self._video_thread.finished.connect(self.close)
            self.__terminate_application()
        hardware = self._session_controller.hardware
        test = self._session_controller.active_test
        if (hardware is not None and hardware._thread.isRunning()) or (test is not None and test.isRunning()) or (hasattr(self, "_video_thread") and self._video_thread.isRunning()):
            event.ignore()
        else:
            event.accept()

    def __terminate_application(self):
        try:
            self._session_controller.shutdown()
        finally:
            self.signal_stop.emit()
        logger.info("Application shutdown requested")

    def __start_test(self):
        try:
            self.read_ui_into_config(self._config)
            self._config.validate(self.tabWidget.currentIndex())
            self.save_config(self._config)
            result = self._session_controller.start_test(self._config, self.tabWidget.currentIndex())
            if result == "sample_exists":
                reply = QMessageBox.question(
                    self,
                    "Message",
                    f"Do you want to add data to {self._config.runtime.sample_name}?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No,
                )
                if reply == QMessageBox.StandardButton.No:
                    self._set_test_running_ui(False)
                    return
                result = self._session_controller.start_test(
                    self._config,
                    self.tabWidget.currentIndex(),
                    allow_existing_sample=True,
                )
            self.save_config(self._config, save_sample_copy=True)
        except Exception as exc:
            if self._session_controller.active_test is not None and self._session_controller.active_test.isRunning():
                self._session_controller.stop_test()
            self._set_test_running_ui(False)
            self._show_warning(str(exc))
            return

        if result is not None and result != "sample_exists":
            self._fl_executing = True

    def _handle_test_created(self, test):
        if self._mec_test is not None:
            try:
                self._mec_test.signal_update_charts.disconnect(self.__update_charts)
            except Exception:
                pass
        self._mec_test = test
        self._mec_test.signal_update_charts.connect(self.__update_charts)
        self._attach_video_to_test(test)

    def _attach_video_to_test(self, test):
        if not hasattr(self, "_video_thread"):
            return

        if self._video_connected_test is test:
            return
        if self._video_connected_test is not None:
            self._video_test_link.detach()
        self._video_test_link = VideoTestLink(self._video_thread, test, self)
        self._video_connected_test = test

    def __startCamera(self):
        self.read_ui_into_config(self._config)
        try:
            result = self._session_controller.prepare_test(self._config, self.tabWidget.currentIndex())
            if result == "sample_exists":
                reply = QMessageBox.question(
                    self,
                    "Message",
                    f"Do you want to add data to {self._config.runtime.sample_name}?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No,
                )
                if reply == QMessageBox.StandardButton.No:
                    return
                self._session_controller.prepare_test(
                    self._config,
                    self.tabWidget.currentIndex(),
                    allow_existing_sample=True,
                )
        except Exception as exc:
            self._show_warning(str(exc))
            return

        if not hasattr(self, "_video_thread"):
            self._video_thread = VideoThread(self._config.runtime.camera_id)
            self._video_thread.signal_error.connect(self._handle_camera_error)
            self.signal_stop.connect(self._video_thread.stop)
            self._video_window = VideoWindow(self._video_thread)
            self.signal_stop.connect(self._video_window.stop)
        self._video_window.show()
        if not self._video_thread.isRunning():
            self._video_thread._camera_id = self._config.runtime.camera_id or self._video_thread._camera_id
            self._video_thread.reset_flags()
            self._video_thread.start()

        if self._mec_test is not None:
            self._attach_video_to_test(self._mec_test)

    @pyqtSlot(str)
    def _handle_camera_error(self, message):
        test = self._mec_test
        if test is not None and test._use_video and test.isRunning():
            test._fail(RuntimeError("Camera: " + message))
        else:
            if test is not None:
                test._use_video = False
            self._show_warning(message)

    def __connect(self):
        self.read_ui_into_config(self._config)
        try:
            self._mot_daq = self._session_controller.connect_hardware(self._config)
        except Exception as exc:
            self._show_warning(str(exc))

    def __stop_movement(self):
        try:
            self._session_controller.stop_motors()
        except Exception as exc:
            self._show_warning(str(exc))

    def __stop_test(self):
        self._session_controller.stop_test()
        self._ringbuffer1.reset()
        self._ringbuffer2.reset()
        self._fl_executing = False
        self._reset_status_labels()

    def __changeLabelColor(self):
        if not self._warning_blink_enabled:
            self._set_force_label_color("black")
            return

        self._warning_color_is_red = not self._warning_color_is_red
        self._set_force_label_color("red" if self._warning_color_is_red else "black")

    def __zeroForce(self):
        try:
            self._session_controller.zero_force()
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()
        except Exception as exc:
            self._show_warning(str(exc))

    def __zeroPosition(self):
        try:
            self._session_controller.zero_position()
        except Exception as exc:
            self._show_warning(str(exc))

    def __moveSamplePosition(self):
        try:
            self._session_controller.move_sample_position()
        except Exception as exc:
            self._show_warning(str(exc))

    def __initializeMotors(self):
        if self._mot_daq is None:
            self._show_warning("Connect to motors and DAQ first!")
            return
        reply = QMessageBox.question(
            self,
            "Message",
            "Is the sample unmounted?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply == QMessageBox.StandardButton.Yes:
            try:
                self._session_controller.initialize_motors()
            except Exception as exc:
                self._show_warning(str(exc))

    def __moveAxis(self, axis, speed):
        try:
            if axis == 1:
                self._session_controller.move_velocity_ax1(speed)
            else:
                self._session_controller.move_velocity_ax2(speed)
        except Exception as exc:
            self._show_warning(str(exc))

    def __calculate_loads(self):
        self._fl_sample_initialized = True
        self._calc_loads_window = LoadCalculatorWindow(
            self._thickness,
            self._len1,
            self._len2,
            self._max_stress1,
            self._max_stress2,
            self._units,
        )
        self._calc_loads_window.signal_loads_calculated.connect(self.__set_sample_params)
        self._calc_loads_window.show()

    @pyqtSlot(float, float, float, float, float, float, float)
    def __set_sample_params(self, load1, load2, stress1, stress2, len1, len2, thickness):
        self.factorLoadTest1.setText(str(load1))
        self.factorLoadTest2.setText(str(load2))
        self._max_stress1 = stress1
        self._max_stress2 = stress2
        self._len1 = len1
        self._len2 = len2
        self._thickness = thickness
        self._area1 = len1 * thickness
        self._area2 = len2 * thickness

    def __autoload(self):
        self.read_ui_into_config(self._config)
        try:
            self._session_controller.perform_autoload(self._config)
        except Exception as exc:
            self._show_warning(str(exc))

    @pyqtSlot(list)
    def __update_charts(self, array):
        self._update_cycle_label()
        self._t.append(array[0])
        self._load1.append(array[1])
        self._load2.append(array[2])
        self._disp1.append(array[3])
        self._disp2.append(array[4])

        if Unit.Newton == self._units:
            stress1 = array[1] * 1000 / self._area1
            stress2 = array[2] * 1000 / self._area2
        else:
            stress1 = array[1] * 9.80665 / self._area1
            stress2 = array[2] * 9.80665 / self._area2

        self._stress1.append(stress1)
        self._stress2.append(stress2)
        self._E11.append(array[5])
        self._E22.append(array[6])
        self._v1.append(array[7])
        self._v2.append(array[8])
        self._strain_disp1.append(array[3] / self._len1)
        self._strain_disp2.append(array[4] / self._len2)

        try:
            if (
                self._E11[-1] is not None
                and self._E22[-1] is not None
                and abs(self._stress1[-1] - 400) < 5
                and abs(self._stress2[-1] - 400) < 10
            ):
                ar = self._E11[-1] / self._E22[-1] if self._E11[-1] > self._E22[-1] else self._E22[-1] / self._E11[-1]
                logger.info("Anisotropy ratio at 400 kPa: %s", ar)
        except Exception:
            logger.warning("Could not calculate anisotropy ratio", exc_info=True)

        self.ChartWidget_1.clear()
        self.ChartWidget_1.plot(self._t, self._load1, pen=pg.mkPen(color="b", width=2), name="Channel 1")
        self.ChartWidget_1.plot(self._t, self._load2, pen=pg.mkPen(color="r", width=2), name="Channel 2")

        self.ChartWidget_2.clear()
        self.ChartWidget_2.plot(self._t, self._disp1, pen=pg.mkPen(color="b", width=2))
        self.ChartWidget_2.plot(self._t, self._disp2, pen=pg.mkPen(color="r", width=2))

        self._plot_strain_chart()

    def _plot_strain_chart(self):
        self.ChartWidget_3.clear()

        if self._has_video_strain_samples():
            self.ChartWidget_3.setLabel("bottom", "Strain", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
            self._plot_valid_pairs(self._E11, self._stress1, "b")
            self._plot_valid_pairs(self._E22, self._stress2, "r")
            return

        self.ChartWidget_3.setLabel("bottom", "Strain (displacement)", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
        self._plot_valid_pairs(self._strain_disp1, self._stress1, "b")
        self._plot_valid_pairs(self._strain_disp2, self._stress2, "r")

    def _has_video_strain_samples(self):
        return any(value is not None for value in self._E11) or any(value is not None for value in self._E22)

    def _plot_valid_pairs(self, x_values, y_values, color):
        pairs = [(x, y) for x, y in zip(x_values, y_values) if x is not None and y is not None]
        if not pairs:
            return
        x_plot, y_plot = zip(*pairs)
        self.ChartWidget_3.plot(x_plot, y_plot, pen=pg.mkPen(color=color, width=2))

    @pyqtSlot(float, float)
    def __update_live_forces(self, force1, force2):
        if Unit.Gram == self._units:
            self.upperLabel_2.setText(
                "Force 1: {} g | Force 2: {} g".format(round(force1, 2), round(force2, 2))
            )
        else:
            self.upperLabel_2.setText(
                "Force 1: {} N | Force 2: {} N".format(round(force1, 4), round(force2, 4))
            )

        self._set_force_label_color("red" if self._warning_blink_enabled and self._warning_color_is_red else "black")

        self._ringbuffer1.append(force1)
        self._ringbuffer2.append(force2)
        if not self._live_monitor_enabled:
            return
        self.ChartWidget_1.clear()
        self.ChartWidget_1.plot(self._t_label, self._ringbuffer1.get_buffer(), pen=pg.mkPen(color="b", width=2), name="Channel 1")
        self.ChartWidget_1.plot(self._t_label, self._ringbuffer2.get_buffer(), pen=pg.mkPen(color="r", width=2), name="Channel 2")

    @pyqtSlot()
    def _save_charts(self):
        folder = os.path.join(self._work_folder, self._sam_name)
        pixmap_path = os.path.join(folder, "test_charts.png")
        pixmap = self.grab()
        pixmap.save(pixmap_path)

    def __openFolderDialog(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            self._work_folder = folder
            self.labelFolder.setText(folder)
            self._config.runtime.folder = folder

    def __change_units(self):
        if self.buttonGrams.isChecked():
            self._units = Unit.Gram
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()
            self.factorTareLoad1.setText(f"{round(self._read_float(self.factorTareLoad1.text()) * 1000 / 9.80665, 0)}")
            self.factorTareLoad2.setText(f"{round(self._read_float(self.factorTareLoad2.text()) * 1000 / 9.80665, 0)}")
            self.factorLoadTest1.setText(f"{round(self._read_float(self.factorLoadTest1.text()) * 1000 / 9.80665, 0)}")
            self.factorLoadTest2.setText(f"{round(self._read_float(self.factorLoadTest2.text()) * 1000 / 9.80665, 0)}")
        else:
            self._units = Unit.Newton
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()
            self.factorTareLoad1.setText(f"{round(self._read_float(self.factorTareLoad1.text()) * 9.80665 / 1000, 2)}")
            self.factorTareLoad2.setText(f"{round(self._read_float(self.factorTareLoad2.text()) * 9.80665 / 1000, 2)}")
            self.factorLoadTest1.setText(f"{round(self._read_float(self.factorLoadTest1.text()) * 9.80665 / 1000, 2)}")
            self.factorLoadTest2.setText(f"{round(self._read_float(self.factorLoadTest2.text()) * 9.80665 / 1000, 2)}")

        self._update_units_labels(self._units)
        self._config.runtime.unit = "gram" if self._units == Unit.Gram else "newton"
        self._session_controller.update_units(self._config)

    def _update_units_labels(self, units: Unit):
        load_span = 4.0 if units == Unit.Gram else 4.0 * 9.80665 / 1000
        self.ChartWidget_1.setLimits(minYRange=load_span)
        self.ChartWidget_1.setYRange(-load_span / 2, load_span / 2, padding=0)
        self.ChartWidget_1.enableAutoRange(axis="y")
        if units == Unit.Gram:
            self.ChartWidget_1.setLabel("left", "Load, g", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
            unit_text = "g"
        else:
            self.ChartWidget_1.setLabel("left", "Load, N", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
            unit_text = "N"
        self.ChartWidget_1.setLabel("bottom", "Time, s", **{"color": "#000", "font-size": "14pt", "font-family": "Arial"})
        self.label_17.setText(unit_text)
        self.label_18.setText(unit_text)
        self.label_26.setText(unit_text)
        self.label_29.setText(unit_text)

    @pyqtSlot(bool)
    def _set_test_running_ui(self, blocked):
        self._fl_executing = blocked
        self._live_monitor_enabled = not blocked
        self._warning_blink_enabled = blocked
        if blocked:
            self.block_gui()
            self._update_cycle_label()
            self._warning_color_is_red = False
            self._set_force_label_color("black")
            if hasattr(self, "_label_timer"):
                self._label_timer.stop()
            else:
                self._label_timer = QTimer(self)
                self._label_timer.timeout.connect(self.__changeLabelColor)
            self._label_timer.start(1000)
        else:
            self.unblock_gui()
            self._warning_color_is_red = False
            self._set_force_label_color("black")
            if hasattr(self, "_label_timer"):
                self._label_timer.stop()
            self._reset_status_labels()

    def _set_force_label_color(self, color):
        style = f"color: {color}; font-size: 14px;"
        self.upperLabel_1.setStyleSheet(style)
        self.upperLabel_2.setStyleSheet(style)

    def _reset_status_labels(self):
        self.upperLabel_1.setText("Cycle -/-")
        if self._units == Unit.Gram:
            self.upperLabel_2.setText("Force 1: 0 g | Force 2: 0 g")
        else:
            self.upperLabel_2.setText("Force 1: 0 N | Force 2: 0 N")

    def _update_cycle_label(self):
        label = self._build_cycle_label()
        if label is not None:
            self.upperLabel_1.setText(label)

    def _build_cycle_label(self):
        test = self._mec_test
        if test is None:
            return "Cycle -/-"

        if isinstance(test, LoadControlTest):
            total_cycles = test._num_cycles_precond + test._num_cycles_test
            if total_cycles <= 0:
                return "Cycle 0/0"

            state = getattr(test, "_state", None)
            if state == State.PRECONDITIONING:
                remaining_half_cycles = getattr(test, "_remaining_precond_half_cycles", 2 * test._num_cycles_precond)
                completed_half_cycles = 2 * test._num_cycles_precond - remaining_half_cycles
                current_cycle = completed_half_cycles // 2 + 1
            elif state == State.TEST:
                remaining_half_cycles = getattr(test, "_remaining_test_half_cycles", 2 * test._num_cycles_test)
                completed_half_cycles = 2 * test._num_cycles_test - remaining_half_cycles
                current_cycle = test._num_cycles_precond + completed_half_cycles // 2 + 1
            else:
                current_cycle = 1

            current_cycle = max(1, min(current_cycle, total_cycles))
            return f"Cycle {current_cycle}/{total_cycles}"

        if isinstance(test, DisplacementControlTest):
            total_cycles = test._num_cycles
            if total_cycles <= 0:
                return "Cycle 0/0"

            completed_half_cycles = getattr(test, "_completed_half_cycles", 0)
            current_cycle = completed_half_cycles // 2 + 1
            current_cycle = max(1, min(current_cycle, total_cycles))
            return f"Cycle {current_cycle}/{total_cycles}"

        return "Cycle -/-"

    def _show_warning(self, message):
        warning_box = QMessageBox()
        warning_box.setIcon(QMessageBox.Icon.Warning)
        warning_box.setWindowTitle("Warning")
        warning_box.setText(message)
        warning_box.exec()

    def _read_float(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    def block_gui(self):
        self.buttonStart.setDisabled(True)
        self.buttonNewtons.setDisabled(True)
        self.buttonGrams.setDisabled(True)
        self.factorTareLoad1.setDisabled(True)
        self.factorTareLoad2.setDisabled(True)
        self.factorLoadTest1.setDisabled(True)
        self.factorLoadTest2.setDisabled(True)
        self.factorDispGuess1.setDisabled(True)
        self.factorDispGuess2.setDisabled(True)
        self.factorTimeAx.setDisabled(True)
        self.factorCyclNumPrecond.setDisabled(True)
        self.factorCyclNumTest.setDisabled(True)
        self.factorSampleName.setDisabled(True)
        self.buttonFolder.setDisabled(True)
        self.buttonZeroPosition.setDisabled(True)
        self.buttonZeroForce.setDisabled(True)
        self.buttonSampleP.setDisabled(True)
        self.buttonInitMot.setDisabled(True)
        self.buttonMoveCentAx1.setDisabled(True)
        self.buttonMoveCentAx2.setDisabled(True)
        self.buttonMoveBackAx1.setDisabled(True)
        self.buttonMoveBackAx2.setDisabled(True)
        self.buttonAutoload.setDisabled(True)
        self.buttonCalcLoad.setDisabled(True)
        self.buttonConnect.setDisabled(True)
        self.buttonCamera.setDisabled(True)

    def unblock_gui(self):
        self.buttonStart.setDisabled(False)
        self.buttonNewtons.setDisabled(False)
        self.buttonGrams.setDisabled(False)
        self.factorTareLoad1.setDisabled(False)
        self.factorTareLoad2.setDisabled(False)
        self.factorLoadTest1.setDisabled(False)
        self.factorLoadTest2.setDisabled(False)
        self.factorDispGuess1.setDisabled(False)
        self.factorDispGuess2.setDisabled(False)
        self.factorTimeAx.setDisabled(False)
        self.factorCyclNumPrecond.setDisabled(False)
        self.factorCyclNumTest.setDisabled(False)
        self.factorSampleName.setDisabled(False)
        self.buttonFolder.setDisabled(False)
        self.buttonZeroPosition.setDisabled(False)
        self.buttonZeroForce.setDisabled(False)
        self.buttonSampleP.setDisabled(False)
        self.buttonInitMot.setDisabled(False)
        self.buttonMoveCentAx1.setDisabled(False)
        self.buttonMoveCentAx2.setDisabled(False)
        self.buttonMoveBackAx1.setDisabled(False)
        self.buttonMoveBackAx2.setDisabled(False)
        self.buttonAutoload.setDisabled(False)
        self.buttonCalcLoad.setDisabled(False)
        self.buttonConnect.setDisabled(False)
        self.buttonCamera.setDisabled(False)


if __name__ == "__main__":
    app = QApplication([])
    app.setStyle("Fusion")
    window = BiaxMainWindow()
    window.show()
    app.exec()
