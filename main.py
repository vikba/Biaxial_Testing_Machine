
#Check if markers are choosen before starting the test
#Check what happens at the end of test
#Reset video classes after the test
#Freeze the application
#add reset after preconditioning to videothread
#add a point list of all the steps that should be followed
#change color font of "axis 1" and "axis 2" words. To be consistent with the color of the charts

   



from PyQt6.QtWidgets import QApplication 

from sources import BiaxMainWindow

if __name__ == '__main__':  
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec() 

 
 