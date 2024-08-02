from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import  pyqtSignal
from PyQt6.uic import loadUi

import os

class LoadCalculatorWindow(QWidget):

    signal_loads_calculated = pyqtSignal(float, float, float, float, float)
    
    
    
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

        self.buttonCalc.clicked.connect(self.calcLoads)
        
        
        
    def closeEvent(self, event):
        pass

    def calcLoads(self):
        if self.radioButton_Stress.isChecked():
            self._thickness = self.factorThickness.value()
            self._len1 = self.factorLen1.value() #mm
            self._len2 = self.factorLen2.value() #mm
            self._target_stress1 = float(self.factorStress1.text()) #kPa 
            self._target_stress2 = float(self.factorStress2.text()) #kPa 

            

            self.load1 = self._target_stress1 * self._len1 * self._thickness / 1000
            self.load2 = self._target_stress2 * self._len2 * self._thickness / 1000

            self.factor_calculatedLoad1.setText(str(self.load1))
            self.factor_calculatedLoad2.setText(str(self.load2))

            self.signal_loads_calculated.emit(self.load1, self.load2, self._len1, self._len2, self._thickness)



if __name__ == '__main__':
    print("Started")
    app = QApplication([])
    app.setStyle('Fusion')
    load_calc_window = LoadCalculatorWindow()
    load_calc_window.show()
    app.exec()
