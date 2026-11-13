"""Machine settings validation, persistence, and GUI lifecycle checks."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import os
import tempfile
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt6.QtWidgets import QApplication, QDialog

from biaxial_testing_machine.biaxmainwindow import BiaxMainWindow
from biaxial_testing_machine.camerawindow import VideoThread
from biaxial_testing_machine.configuration import ConfigRepository, MachineConfig
from biaxial_testing_machine.machineconfigdialog import MachineConfigDialog
from biaxial_testing_machine.sessioncontroller import TestSessionController
from biaxial_testing_machine.sessionstate import SessionState


class MachineConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "config.local.json"
        self.env = patch.dict(os.environ, {"BIAXIAL_CONFIG": str(self.path)})
        self.env.start()
        self.addCleanup(self.env.stop)
        self.window = BiaxMainWindow()
        self.addCleanup(self.close_window)

    def close_window(self):
        self.window._session_controller._mot_daq = None
        self.window._session_controller._active_test = None
        if hasattr(self.window, "_video_thread"):
            del self.window._video_thread
        self.window.close()

    def dialog(self, save=None, available=None):
        candidate = self.window.read_ui_into_config(deepcopy(self.window._config), apply=False)
        dialog = MachineConfigDialog(
            candidate, str(self.path), save or self.window._save_machine_configuration,
            available or self.window._can_edit_machine_configuration, self.window,
        )
        self.addCleanup(dialog.close)
        return dialog

    def test_cancel_preserves_memory_and_disk_with_unsaved_experiment_fields(self):
        original = deepcopy(self.window._config)
        self.window._config_repository.save(original)
        original_bytes = self.path.read_bytes()
        self.window.factorLoadTest1.setText("321")
        dialog = self.dialog()
        dialog.fields["ip_motor"].setText("192.168.1.22")
        dialog.reject()
        self.assertEqual(self.window._config, original)
        self.assertEqual(self.path.read_bytes(), original_bytes)
        self.assertEqual(self.window.factorLoadTest1.text(), "321")

    def test_save_preserves_experiment_fields_and_updates_next_motor_connection(self):
        self.window.factorLoadTest1.setText("321")
        self.window.factorSampleName.setText("unsaved-sample")
        dialog = self.dialog()
        dialog.fields["ip_motor"].setText("192.168.1.22")
        dialog.fields["axis1_serial"].setText("0000000001")
        dialog.accept()
        self.assertEqual(dialog.result(), QDialog.DialogCode.Accepted)
        saved = ConfigRepository(str(self.path)).load()
        self.assertEqual(saved.load_test.load_test1, "321")
        self.assertEqual(saved.runtime.sample_name, "unsaved-sample")
        self.assertEqual(saved.force.axis1_serial, "0000000001")
        self.assertEqual(self.window._config, saved)
        controller = TestSessionController()
        with patch("biaxial_testing_machine.sessioncontroller.MotorDAQInterface") as hardware:
            controller.connect_hardware(saved)
        self.assertEqual(hardware.call_args.kwargs["ip_motor"], "192.168.1.22")
        self.assertEqual(hardware.call_args.kwargs["force_config"], saved.force)

    def test_invalid_fields_and_force_timing_cannot_save(self):
        for key, value in (
            ("ip_motor", "999.1.2.3"), ("axis1_serial", "123"),
            ("axis2_serial", self.window._config.force.axis1_serial),
            ("axis1_transmission_factor", "nan"), ("axis2_transmission_factor", "-1"),
            ("timeout", "0"), ("max_data_age", "0.01"),
        ):
            with self.subTest(key=key):
                save = Mock()
                dialog = self.dialog(save)
                dialog.fields[key].setText(value)
                dialog.accept()
                save.assert_not_called()
                self.assertTrue(dialog.errors[key].text())
                self.assertNotEqual(dialog.result(), QDialog.DialogCode.Accepted)
        dialog = self.dialog(Mock())
        dialog.auto_camera.setChecked(False)
        dialog.accept()
        self.assertTrue(dialog.errors["camera_id"].text())

    def test_failed_atomic_replace_preserves_file_memory_and_temporary_cleanup(self):
        original = deepcopy(self.window._config)
        self.window._config_repository.save(original)
        original_bytes = self.path.read_bytes()
        dialog = self.dialog()
        dialog.fields["ip_motor"].setText("192.168.1.22")
        with patch("biaxial_testing_machine.configuration.os.replace", side_effect=OSError("disk unavailable")):
            dialog.accept()
        self.assertEqual(self.window._config, original)
        self.assertEqual(self.path.read_bytes(), original_bytes)
        self.assertEqual(list(self.path.parent.glob(".config-*.tmp")), [])
        self.assertIn("disk unavailable", dialog.message.text())
        self.assertNotEqual(dialog.result(), QDialog.DialogCode.Accepted)

    def test_camera_manual_and_auto_selection_replace_retained_identity(self):
        video = VideoThread("DEV_OLD")
        self.window._video_thread = video
        dialog = self.dialog()
        dialog.auto_camera.setChecked(False)
        dialog.fields["camera_id"].setText(" DEV_NEW ")
        dialog.accept()
        self.assertEqual(video._camera_id, "DEV_NEW")
        dialog = self.dialog()
        dialog.auto_camera.setChecked(True)
        dialog.accept()
        self.assertEqual(video._camera_id, "")
        self.assertEqual(self.window._config.runtime.camera_id, "")
        first, second = Mock(), Mock()
        first.get_id.return_value = "DEV_OTHER"
        second.get_id.return_value = "DEV_NEW"
        video.set_camera_id("DEV_NEW")
        self.assertIs(video._select_camera((first, second)), second)
        video.set_camera_id("")
        self.assertIs(video._select_camera((first,)), first)

    def test_activity_guards_cover_threads_states_and_save_recheck(self):
        controller = self.window._session_controller
        for state in (SessionState.RUNNING_PRECONDITION, SessionState.RUNNING_TEST,
                      SessionState.AUTOLOADING, SessionState.STOPPING):
            controller._state = state
            self.window._update_settings_actions()
            self.assertFalse(self.window._machine_config_action.isEnabled())
        controller._state = SessionState.DISCONNECTED
        for attribute, value in (
            ("_mot_daq", SimpleNamespace(_thread=SimpleNamespace(isRunning=lambda: True))),
            ("_active_test", SimpleNamespace(isRunning=lambda: True)),
        ):
            setattr(controller, attribute, value)
            self.assertFalse(self.window._can_edit_machine_configuration())
            setattr(controller, attribute, None)
        dialog = self.dialog()
        self.window._video_thread = SimpleNamespace(isRunning=lambda: True)
        dialog.accept()
        self.assertFalse(self.path.exists())
        with self.assertRaises(RuntimeError):
            self.window._save_machine_configuration(deepcopy(self.window._config))
        self.window._video_thread = SimpleNamespace(isRunning=lambda: False)
        self.assertTrue(self.window._can_edit_machine_configuration())

    def test_disconnect_devices_stops_workers_without_restarting_app(self):
        controller = self.window._session_controller
        hardware = Mock()
        hardware._thread.isRunning.return_value = True
        controller._mot_daq = hardware
        self.window._video_thread = Mock()
        self.window._video_thread.isRunning.return_value = True
        self.window._video_window = Mock()
        self.window._disconnect_devices()
        hardware.stop.assert_called_once()
        self.window._video_thread.stop.assert_called_once()
        self.window._video_window.close.assert_called_once()
        self.assertFalse(self.window._can_edit_machine_configuration())
        hardware._thread.isRunning.return_value = False
        self.window._video_thread.isRunning.return_value = False
        controller._handle_hardware_disconnected()
        self.window._update_settings_actions()
        self.assertTrue(self.window._machine_config_action.isEnabled())

    def test_units_can_change_after_disconnect_before_reconnecting(self):
        controller = self.window._session_controller
        hardware = Mock()
        hardware._thread.isRunning.return_value = False
        controller._mot_daq = hardware
        controller.update_units(self.window._config)
        hardware.change_units.assert_not_called()
        hardware._thread.isRunning.return_value = True
        hardware._closing = True
        controller.update_units(self.window._config)
        hardware.change_units.assert_not_called()
        hardware._closing = False
        controller.update_units(self.window._config)
        hardware.change_units.assert_called_once_with(self.window._config.unit_enum)

    def test_save_leaves_example_and_test_snapshots_unchanged(self):
        example = Path(__file__).resolve().parents[1] / "resources/config.example.json"
        example_bytes = example.read_bytes()
        snapshot = self.path.parent / "config_test.json"
        snapshot.write_text('{"experiment": "previous"}', encoding="utf-8")
        original_snapshot = snapshot.read_bytes()
        self.dialog().accept()
        self.assertEqual(example.read_bytes(), example_bytes)
        self.assertEqual(snapshot.read_bytes(), original_snapshot)

    def test_serialization_failure_preserves_original(self):
        repository = ConfigRepository(str(self.path))
        config = MachineConfig()
        repository.save(config)
        original = self.path.read_bytes()
        config.sample.thickness = float("nan")
        with self.assertRaises(ValueError):
            repository.save(config)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertFalse(list(self.path.parent.glob(".config-*.tmp")))


if __name__ == "__main__":
    unittest.main()
