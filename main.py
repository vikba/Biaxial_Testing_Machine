
#Check if all conditions met to start the test
#Check what happens at the end of test
#Reset video classes after the test
#Freeze the application
#Write cycles in different files
#Save pyqtgraph as img
#Compare formulas with matlab
#Save config in the same folder as sample
#add reset after preconditioning to videothread
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

 
 