"""Launch the installed desktop application."""
import logging
import sys

from PyQt6.QtWidgets import QApplication

from .biaxmainwindow import BiaxMainWindow


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = BiaxMainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
