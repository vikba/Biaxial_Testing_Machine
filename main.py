
#Send stop signal or stop funcaiton is ok?
#Save thread and timer exit
#Correct saving of images and data
#Set motor limits in steps
#Removed -2* in get_positions - check loadcontroltest

from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 