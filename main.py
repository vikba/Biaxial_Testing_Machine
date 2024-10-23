
#Check if markers are choosen before starting the test
#Check what happens at the end of test
#Load/save configuration to a file
#Reset video classes after the test
#Freeze the application
#Write cycles in different files
#Save pyqtgraph as img
#Compare formulas with matlab
#Save config in the same folder as sample
#add reset after preconditioning to videothread
#add arrown on an image
#be sure that data are not lost



from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':  
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 