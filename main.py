#Check if all conditions met to start the test
#Check what happens at the end of test
#Removed -2* in get_positions - check loadcontroltest
#Read configuration from file
#Reset video classes after the test
#Freeze the application
#The camera didn't work during load control test (after capt mark)

from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 