"""Edit workstation settings without changing experiment controls."""
from copy import deepcopy

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QCheckBox, QDialog, QDialogButtonBox, QFormLayout, QLabel, QLineEdit,
    QVBoxLayout, QWidget,
)


class MachineConfigDialog(QDialog):
    def __init__(self, config, config_path, save_settings, can_edit, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Machine configuration")
        self.setMinimumWidth(480)
        self._config = deepcopy(config)
        self._save_settings, self._can_edit = save_settings, can_edit
        self.fields, self.errors = {}, {}
        layout = QVBoxLayout(self)
        form = QFormLayout()
        layout.addLayout(form)
        for key, label, value in (
            ("ip_motor", "Motor IP address", config.runtime.ip_motor),
            ("camera_id", "Camera device ID", config.runtime.camera_id),
            ("axis1_serial", "Axis 1 sensor serial", config.force.axis1_serial),
            ("axis2_serial", "Axis 2 sensor serial", config.force.axis2_serial),
        ):
            self._add_field(form, key, label, value)
        self.auto_camera = QCheckBox("Auto-select single camera")
        self.auto_camera.setChecked(not config.runtime.camera_id)
        self.auto_camera.toggled.connect(lambda checked: self.fields["camera_id"].setDisabled(checked))
        self.fields["camera_id"].setDisabled(self.auto_camera.isChecked())
        form.insertRow(1, self.auto_camera)
        self.fields["camera_id"].setPlaceholderText("Device ID from Vimba X Viewer")

        self.advanced_toggle = QCheckBox("Show advanced force settings")
        layout.addWidget(self.advanced_toggle)
        advanced = QWidget()
        advanced_form = QFormLayout(advanced)
        for key, label in (
            ("axis1_transmission_factor", "Axis 1 transmission factor"),
            ("axis2_transmission_factor", "Axis 2 transmission factor"),
            ("timeout", "Serial timeout (seconds)"),
            ("max_data_age", "Maximum data age (seconds)"),
        ):
            self._add_field(advanced_form, key, label, getattr(config.force, key))
        layout.addWidget(advanced)
        advanced.hide()
        self.advanced_toggle.toggled.connect(advanced.setVisible)
        path_label = QLabel(f"Configuration file: {config_path}")
        path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        for label in (path_label, QLabel("Saved settings take effect on the next connection.")):
            label.setTextFormat(Qt.TextFormat.PlainText)
            label.setWordWrap(True)
            layout.addWidget(label)
        self.message = QLabel()
        self.message.setWordWrap(True)
        self.message.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.message)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _add_field(self, form, key, label, value):
        self.fields[key] = QLineEdit(str(value))
        self.errors[key] = QLabel()
        self.errors[key].setStyleSheet("color: #b00020;")
        self.errors[key].setWordWrap(True)
        self.errors[key].hide()
        form.addRow(label, self.fields[key])
        form.addRow("", self.errors[key])

    def accept(self):
        if not self._can_edit():
            self.message.setText("Stop tests and disconnect devices before saving machine settings.")
            return
        candidate = deepcopy(self._config)
        errors = {}
        for key, field in self.fields.items():
            section = candidate.runtime if key in ("ip_motor", "camera_id") else candidate.force
            value = field.text().strip()
            if isinstance(getattr(section, key), (int, float)):
                try:
                    value = float(value)
                except ValueError:
                    errors[key] = "Enter a number using a decimal point."
                    continue
            setattr(section, key, value)
        errors.update(candidate.machine_settings_errors())
        if self.auto_camera.isChecked():
            candidate.runtime.camera_id = ""
        elif not candidate.runtime.camera_id:
            errors["camera_id"] = "Enter a camera ID or enable automatic selection."
        for key, label in self.errors.items():
            label.setText(errors.get(key, ""))
            label.setVisible(key in errors)
        self.message.clear()
        if errors:
            first = next(key for key in self.fields if key in errors)
            if any(key in errors for key in ("axis1_transmission_factor", "axis2_transmission_factor", "timeout", "max_data_age")):
                self.advanced_toggle.setChecked(True)
            self.fields[first].setFocus()
            return
        try:
            self._save_settings(candidate)
        except Exception as exc:
            self.message.setText(f"Settings were not saved: {exc}")
            return
        super().accept()
