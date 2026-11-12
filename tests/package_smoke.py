"""Run outside the checkout after installing a wheel: package_smoke.py [module]."""
from importlib.metadata import distribution
from importlib.resources import files
from pathlib import Path
import os
import runpy
import sys
import tempfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import biaxial_testing_machine
from biaxial_testing_machine.biaxmainwindow import BiaxMainWindow
from biaxial_testing_machine.loadcalculator import LoadCalculatorWindow
from biaxial_testing_machine.unit import Unit
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication

project = distribution("biaxial-testing-machine")
assert Path(biaxial_testing_machine.__file__).resolve() == Path(project.locate_file("biaxial_testing_machine/__init__.py")).resolve()
for name in ("biax_ui.ui", "load_calculator.ui", "config.example.json", "icon.ico"):
    assert files("biaxial_testing_machine.resources").joinpath(name).is_file()

original_exec = QApplication.exec


def smoke_exec():
    window = next(widget for widget in QApplication.topLevelWidgets() if isinstance(widget, BiaxMainWindow))
    assert not window.windowIcon().isNull()
    assert window._config_path == os.environ["BIAXIAL_CONFIG"]
    window.save_config(window._config)
    assert Path(window._config_path).is_file()
    calculator = LoadCalculatorWindow(.34, 9, 9, 500, 500, Unit.Newton)
    assert calculator.factorThickness.value() == .34
    calculator.close()
    QTimer.singleShot(0, QApplication.closeAllWindows)
    return original_exec()


QApplication.exec = staticmethod(smoke_exec)
with tempfile.TemporaryDirectory() as folder:
    os.environ["BIAXIAL_CONFIG"] = str(Path(folder) / "settings" / "config.local.json")
    if "module" in sys.argv:
        sys.argv = ["biaxial_testing_machine"]
        try:
            runpy.run_module("biaxial_testing_machine", run_name="__main__")
        except SystemExit as exc:
            assert exc.code == 0
    else:
        entry = next(item for item in project.entry_points if item.group == "console_scripts" and item.name == "biaxial-testing-machine")
        assert entry.load()() == 0
print("Installed package, resources, configuration, and launcher passed")
