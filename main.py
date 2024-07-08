
#Decrease resolution of images for camera in signal slot mechanism
#Optimize signal slot mechanism (not all array and reduce image resolution for display purposes)
#Move first frame to mechanical test
#Connect folder and writingin the file
#Think about conditions of finishing the test
#Set motor limits in steps
#Check logic of displacement test

from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()


 