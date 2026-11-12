import logging

from PyQt6.QtWidgets import QApplication 

from sources.biaxmainwindow import BiaxMainWindow

if __name__ == '__main__':  
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()
