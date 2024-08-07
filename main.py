
#Check if all conditions met to start the test
#Check what happens at the end of test
#Read configuration from file
#Reset video classes after the test
#Freeze the application
#add ring buffer to mot_daq and fft
#Record only last points in videothread
#Name of the sample creates a workfolder
#Write cycles in different files
#Save pyqtgraph as img
#Compare formulas with matlab
#Save config in the same folder as sample
#Nan afer load control
#problem with double sample name in folder
#add reset after preconditioning to videothread


from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 