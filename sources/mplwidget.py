# ------------------------------------------------------
# -------------------- mplwidget.py --------------------
# ------------------------------------------------------
#Inherited from QWidget
#To be used QWidget need to be added in Qt Designer
#and then promoted to MplWidget
from PyQt6.QtWidgets import*

from matplotlib.backends.backend_qt5agg import FigureCanvas

from matplotlib.figure import Figure

    
class MplWidget(QWidget):
    
    def __init__(self, parent = None):

        QWidget.__init__(self, parent)

        fig = Figure()
        #fig.tight_layout()

        fig.subplots_adjust(bottom=0.2)  # Adjusts the bottom of the subplot area

        
        self.canvas = FigureCanvas(fig)
    
        vertical_layout = QVBoxLayout()
        vertical_layout.addWidget(self.canvas)
        
        self.canvas.axes = self.canvas.figure.add_subplot(111)
        self.setLayout(vertical_layout)
        
       