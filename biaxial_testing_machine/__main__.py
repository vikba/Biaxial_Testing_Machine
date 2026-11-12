"""Launch the installed desktop application."""
from importlib.resources import as_file, files
import logging
import sys

from PyQt6.QtWidgets import QApplication
from PyQt6.QtGui import QIcon

from .biaxmainwindow import BiaxMainWindow


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    with as_file(files("biaxial_testing_machine.resources").joinpath("icon.ico")) as icon:
        app.setWindowIcon(QIcon(str(icon)))
    window = BiaxMainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
