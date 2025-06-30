  
#load IP address from json config file
#add anisotropy ratio calculation 
#Check if markers are choosen before starting the test
#Check what happens at the end of test
#Reset video classes after the test
#Freeze the application
#add reset after preconditioning to videothread
#add a point list of all the steps that should be followed
#take first frame on the beginning of the first cycle of test (not the pretest)
#Tare and load values are weird numbers when the program starts



from PyQt6.QtWidgets import QApplication 

from sources import BiaxMainWindow

if __name__ == '__main__':  
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()  

 
 