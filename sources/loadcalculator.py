from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import  pyqtSignal
from PyQt6.uic import loadUi

import os

class LoadCalculatorWindow(QWidget):

    signal_loads_calculated = pyqtSignal(float, float)
    
    
    
    def __init__(self):
        """
        Initializes the video stream window and sets up the interface for displaying the video feed 
        and controlling the webcam. 
        """
        super().__init__()
        self.setWindowTitle("Load calculator")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file_path = os.path.join(script_dir, '..', 'resources', 'load_calculator.ui')

        print(ui_file_path)

        loadUi(ui_file_path, self)

        self.buttonCalcLoad.clicked.connect(self.calcLoads)
        
        
        
    def closeEvent(self, event):
        pass

    def calcLoads(self):
        pass



if __name__ == '__main__':
    print("Started")
    app = QApplication([])
    app.setStyle('Fusion')
    load_calc_window = LoadCalculatorWindow()
    load_calc_window.show()
    app.exec()
