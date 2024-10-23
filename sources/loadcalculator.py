from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.uic import loadUi

import os

from .unit import Unit


class LoadCalculatorWindow(QWidget):
    """
    Window for calculating loads based on sample dimensions and target stresses.
    Handles unit conversions between grams and newtons.
    """
    
    # Signal to emit calculated loads
    signal_loads_calculated = pyqtSignal(float, float, float, float, float, float, float)
    
    def __init__(self, thickness, len1, len2, stress1, stress2, unit):
        """
        Initialize the LoadCalculatorWindow.
        
        Args:
            thickness (float): Thickness of the sample in mm.
            len1 (float): Length along axis 1 in mm.
            len2 (float): Length along axis 2 in mm.
            stress1 (float): Target stress along axis 1 in kPa.
            stress2 (float): Target stress along axis 2 in kPa.
            unit (Unit): Unit of measurement (Unit.Gram or Unit.Newton).
        """
        super().__init__()
        self.setWindowTitle("Load Calculator")
        
        # Store the unit
        self.unit = unit
        
        # Load the UI file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file_path = os.path.join(script_dir, '..', 'resources', 'load_calculator.ui')
        
        loadUi(ui_file_path, self)
        
        # Connect the Calculate button to the calculation method
        self.buttonCalc.clicked.connect(self.calc_loads)
        
        # Initialize input fields with provided values
        self.factorLen1.setValue(len1)
        self.factorLen2.setValue(len2)
        self.factorThickness.setValue(thickness)
        self.factorStress1.setValue(stress1)
        self.factorStress2.setValue(stress2)
        
        # Set unit labels based on the selected unit
        self._update_unit_labels()
    
    def _update_unit_labels(self):
        """
        Update the unit labels in the UI based on the selected unit.
        """
        if self.unit == Unit.Newton:
            unit_str = "N"
        else:
            unit_str = "g"
        
        # Assuming factor_calculatedLoad1 and factor_calculatedLoad2 are QLabel widgets
        self.label_11.setText(unit_str)
        self.label_12.setText(unit_str)
    
    def _convert_load(self, load_newton):
        """
        Convert load from newtons to grams if needed.
        
        Args:
            load_newton (float): Load in newtons.
        
        Returns:
            float: Load in the desired unit.
        """
        if self.unit == Unit.Gram:
            g = 9.81  # Acceleration due to gravity in m/s²
            return load_newton * 1000 / g  # Convert N to g
        return load_newton  # Already in newtons
    
    def _validate_inputs(self):
        """
        Validate input fields to ensure they contain valid numbers.
        
        Returns:
            bool: True if all inputs are valid, False otherwise.
        """
        try:
            thickness = self.factorThickness.value()
            len1 = self.factorLen1.value()
            len2 = self.factorLen2.value()
            stress1 = self.factorStress1.value()
            stress2 = self.factorStress2.value()
            
            if thickness <= 0 or len1 <= 0 or len2 <= 0 or stress1 <= 0 or stress2 <= 0:
                raise ValueError("All input values must be positive numbers.")
            
            return True
        except ValueError as e:
            self._show_error_message(str(e))
            return False
    
    def _show_error_message(self, message):
        """
        Display an error message to the user.
        
        Args:
            message (str): The error message to display.
        """
        from PyQt6.QtWidgets import QMessageBox
        
        error_box = QMessageBox()
        error_box.setIcon(QMessageBox.Icon.Warning)
        error_box.setWindowTitle("Input Error")
        error_box.setText(message)
        error_box.exec()
    
    @pyqtSlot()
    def calc_loads(self):
        """
        Calculate the loads based on the input dimensions and target stresses.
        Emits the calculated loads through the signal_loads_calculated signal.
        """
        if not self._validate_inputs():
            return
        
        # Retrieve input values
        thickness = self.factorThickness.value()  # in mm
        len1 = self.factorLen1.value()            # in mm
        len2 = self.factorLen2.value()            # in mm
        stress1 = self.factorStress1.value()      # in kPa
        stress2 = self.factorStress2.value()      # in kPa
        
        # Calculate loads: Load = Stress * Area
        # Area = length * thickness (assuming unit length for simplicity)
        # Convert stress from kPa to N/mm² (1 kPa = 1 N/m² = 0.001 N/mm²)
        area1 = len1 * thickness  # in mm²
        area2 = len2 * thickness  # in mm²
        
        load1_newton = stress1 * 0.001 * area1  # in N
        load2_newton = stress2 * 0.001 * area2  # in N

        if self.unit == Unit.Newton:    
            # Convert loads based on the selected unit
            load1 = round(self._convert_load(load1_newton),3)
            load2 = round(self._convert_load(load2_newton), 3)
        else:
            load1 = round(self._convert_load(load1_newton),1)
            load2 = round(self._convert_load(load2_newton), 1)
        
        # Update the UI with calculated loads
        if self.unit == Unit.Newton:
            unit_str = "N"
        else:
            unit_str = "g"
        
        self.factor_calculatedLoad1.setText(f"{round(load1, 3)} {unit_str}")
        self.factor_calculatedLoad2.setText(f"{round(load2, 3)} {unit_str}")
        
        # Emit the calculated loads
        self.signal_loads_calculated.emit(
            load1, load2, stress1, stress2, len1, len2, thickness
        )
    
    
if __name__ == '__main__':
    import sys
    from PyQt6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    # Example usage with Unit.Newton
    load_calc_window = LoadCalculatorWindow(
        thickness=10.0, len1=100.0, len2=100.0,
        stress1=50.0, stress2=50.0, unit=Unit.Newton
    )
    load_calc_window.show()
    sys.exit(app.exec())
