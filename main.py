
#Markerk are not updated in temporary variables
#Correct saving of images and data
#Set motor limits in steps

from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 