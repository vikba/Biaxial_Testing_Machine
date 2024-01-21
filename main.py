# ------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------

#Connect folder and writingin the file
#Think about conditions of finishing the test
#Set motor limits in steps
#Check logic of displacement test
#Correct folder for saving results
#Add strain
#Change the tracking algorithm in a way so for each group of the marks it will find the closest element
#Check what is going on at the end of load control test



'''
import sys
# Add the directory containing your module/file to the Python path
sys.path.append('C:/Users/vbalashov/Desktop/Biax_App_Python/sources') 

from biaxmainwindow import BiaxMainWindow
from mechanicaltests import LoadControlTest
from camerawindow import *
from mplwidget import * '''

from PyQt6.QtWidgets import QApplication

from sources import BiaxMainWindow
from sources import MplWidget

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()