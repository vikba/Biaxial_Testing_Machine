
#Check if all conditions met to start the test
#Check what happens at the end of test
#Read configuration from file
#Reset video classes after the test
#Freeze the application
#add ring buffer to mot_daq and fft
#Record only last points in videothread


from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 