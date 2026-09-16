from PyQt6.QtWidgets import QApplication 

from sources.biaxmainwindow import BiaxMainWindow

if __name__ == '__main__':  
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()
