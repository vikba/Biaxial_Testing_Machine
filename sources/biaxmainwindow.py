"""

@author: Viktor B

"""

from PyQt6.QtWidgets import*
from PyQt6.uic import loadUi
from PyQt6.QtCore import QTimer

import os

from .mechanicaltests import DisplacementControlTest, LoadControlTest
from .camerawindow import VideoThread, VideoWindow
from .loadcalculator import LoadCalculatorWindow
#from .mplwidget import MplWidget

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

     
class BiaxMainWindow(QMainWindow):
    """
    BiaxMainWindow class inherits from QMainWindow.
    This class is the main application window for a biaxial testing machine.
    It sets up the user interface and connects GUI elements to their functionalities.
    """
    
    def __init__(self):
        """
        Constructor method for initializing the Biaxial testing machine GUI. 
        This method initializes various GUI elements such as buttons, labels, and 
        matplotlib widgets, and sets up event connections for button clicks. It also 
        sets the work folder, reads motor speed, and loads the .ui file. 
        """
        
        QMainWindow.__init__(self)

        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file_path = os.path.join(script_dir, '..', 'resources', 'biax_ui.ui')

        print(ui_file_path)

        # Load the .ui file
        loadUi(ui_file_path, self)

        #loadUi("biax_ui.ui",self)
        
        
        # Initialization of GUI elements
        self.setWindowTitle("Biaxial testing machine")

        #init buttons
        self.buttonStart.clicked.connect(self.__start_measurement)
        self.buttonStop.clicked.connect(self.__stop_movement)
        self.buttonSampleP.clicked.connect(self.__moveSamplePosition)
        self.buttonInitMot.clicked.connect(self.__initializeMotors)
        self.buttonCamera.clicked.connect(self.__startCamera)
        self.buttonFolder.clicked.connect(self.__openFolderDialog)
        self.buttonZeroForce.clicked.connect(self.__zeroForce)
        self.buttonZeroPosition.clicked.connect(self.__zeroPosition)
        self.buttonConnect.clicked.connect(self.__connect)
        self.buttonCalcLoad.clicked.connect(self.__calculateLoads)
        
        self.buttonMoveCentAx1.pressed.connect(self.__moveForwardAxis1)
        self.buttonMoveCentAx2.pressed.connect(self.__moveForwardAxis2)
        self.buttonMoveBackAx1.pressed.connect(self.__moveBackwardAxis1)
        self.buttonMoveBackAx2.pressed.connect(self.__moveBackwardAxis2)
        
        self.buttonMoveCentAx1.released.connect(self.__stop_movement)
        self.buttonMoveCentAx2.released.connect(self.__stop_movement)
        self.buttonMoveBackAx1.released.connect(self.__stop_movement)
        self.buttonMoveBackAx2.released.connect(self.__stop_movement)

        self.factorKp1.valueChanged.connect(self.__updatePID)
        self.factorKi1.valueChanged.connect(self.__updatePID)
        self.factorKd1.valueChanged.connect(self.__updatePID)
        self.factorKp2.valueChanged.connect(self.__updatePID)
        self.factorKi2.valueChanged.connect(self.__updatePID)
        self.factorKd2.valueChanged.connect(self.__updatePID)
        
        self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
        self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
        
        # Get the path of the parent directory (project_directory)
        parent_directory = os.path.dirname(script_dir)

        # Construct the path to the 'tests' folder
        tests_path = os.path.join(parent_directory, 'tests')


        # init work folder
        self._work_folder = tests_path
        print(self._work_folder)
        self.labelFolder.setText(self._work_folder)
        
        #init matplotlib widgets to display charts
        #self.MplWidget_1.canvas.axes.set_ylabel('Load, N')
        #self.MplWidget_1.canvas.axes.set_xlabel('Time, s')
        #self.MplWidget_2.canvas.axes.set_ylabel('Load, N')
        #self.MplWidget_2.canvas.axes.set_xlabel('Displacement, mm')


        #Configure charts
        self.ChartWidget_1.setBackground('w')  # Set background to white
        self.ChartWidget_1.getAxis('left').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_1.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_1.getAxis('left').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_1.getAxis('bottom').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_1.showGrid(x=False, y=False, alpha=0.5)
        self.ChartWidget_1.getPlotItem().getViewBox().setBorder(pg.mkPen(color='k', width=1))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.ChartWidget_1.getAxis('left').setTickFont(font)
        self.ChartWidget_1.getAxis('bottom').setTickFont(font)
        self.ChartWidget_1.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        self.ChartWidget_1.setLabel('bottom', 'Time, s', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})

        # Generate data
        x = [1,2,3,4,5]
        y = [1,1,2,2,3]

        # Plot data with classic line style
        self.ChartWidget_1.plot(x, y, pen=pg.mkPen(color='b', width=1))  # Blue line

        self.ChartWidget_2.setBackground('w')  # Set background to white
        self.ChartWidget_2.getAxis('left').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('left').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_2.getAxis('bottom').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_2.showGrid(x=False, y=False, alpha=0.5)
        self.ChartWidget_2.getPlotItem().getViewBox().setBorder(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('left').setTickFont(font)
        self.ChartWidget_2.getAxis('bottom').setTickFont(font)
        self.ChartWidget_2.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        self.ChartWidget_2.setLabel('bottom', 'Time, s', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})

        self.ChartWidget_3.setBackground('w')  # Set background to white
        self.ChartWidget_3.getAxis('left').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_3.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_3.getAxis('left').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_3.getAxis('bottom').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_3.showGrid(x=False, y=False, alpha=0.5)
        self.ChartWidget_3.getPlotItem().getViewBox().setBorder(pg.mkPen(color='k', width=1))
        self.ChartWidget_3.getAxis('left').setTickFont(font)
        self.ChartWidget_3.getAxis('bottom').setTickFont(font)
        self.ChartWidget_3.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        self.ChartWidget_3.setLabel('bottom', 'Time, s', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})


        
      

        
        #event to stop execution of "start" thread
        #self._mainThread = threading.currentThread()
        
        # Read motor speed
        self._vel_ax1 = -int(self.factorSpeedAx1.text())/60 #Convert from per minute to per second
        self._vel_ax2 = -int(self.factorSpeedAx2.text())/60

        
        self._t = [] 
        self._ch1 = [] 
        self._ch2 = [] 
        self._l1 = [] 
        self._l2 = [] 
        self._E11 = [] 
        self._E22 = [] 
        self._v1 = [] 
        self._v2 = []

        
    def closeEvent(self, event):
        # This method is called when the window is closed.
        self.__terminate_application()

    def __terminate_application(self):
        
        # Terminate Threads
        if hasattr(self, '_mecTest'):
            self._mecTest.stop()

        if hasattr(self, '_video_thread'):
            self._video_thread.stop()
        
        if hasattr(self, '_video_window'):
            self._video_window.close()
            
        if hasattr(self, '_label_timer'):
            self._label_timer.stop()

        if hasattr(self, '_liveforce_timer'):
            self._liveforce_timer.stop()
            
            
        self.close()
        
        print('Application terminated')
        
    
    #functions connected to buttons
    
    def __start_measurement(self):
        """
        Start the measurement process, connect tracking signal, and set up timer for label updates.
        Set end force values, test duration, and cycle number. 
        If load control test object exists, update test parameters and start the test. 
        If not, display a warning message to connect to motors and DAQ first.
        """

        if hasattr(self, '_mecTest'):

            self._liveforce_timer.stop()

            #if hasattr(self, '_video_thread'):
                #self._mecTest.start_stop_tracking_signal.connect(self._video_thread.startStopTracking)
            
            if hasattr(self, '_label_timer'):
                self._label_timer.stop()
                self._label_timer.start(1000)  # 1000 milliseconds = 1 second
            else:
                self._label_timer = QTimer(self)
                self._label_timer.timeout.connect(self.__changeLabelColor)
                self._label_timer.start(1000)  # 1000 milliseconds = 1 second
            
            #Load Control Test
            if isinstance(self._mecTest, LoadControlTest):
                self.end_force1 = float(self.factorForceAx1.text())
                self.end_force2 = float(self.factorForceAx2.text())
                self.test_duration = float(self.factorTimeAx.text())
                self.cycl_num = int(self.factorCyclNum.text())

                #If object of load control test exists just update test parameters
                if self.checkBoxCycl.isChecked():
                    self._mecTest.update_parameters(self.end_force1, self.end_force2, self.test_duration, self.cycl_num)
                else:
                    # 0 - one directional test
                    self._mecTest.update_parameters(self.end_force1, self.end_force2, self.test_duration, 0)

                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Load control test")
                self._mecTest.start()
            
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()

        

    def __connect(self):
        '''
        Connects to the appropriate test based on the current tab in the tabWidget
        
        tabWidget:
        0 - load control
        1 - strain control
        2 - displacement control
        '''

        try:
            if 0 == self.tabWidget.currentIndex():
                
                '''
                Load Control Test!
                '''
                
                # Get test parameters
                self.end_force1 = float(self.factorForceAx1.text())
                self.end_force2 = float(self.factorForceAx2.text())
                self.test_duration = float(self.factorTimeAx.text())
                self.cycl_num = int(self.factorCyclNum.text())
                
                if hasattr(self, '_mecTest') and isinstance(self._mecTest, LoadControlTest):
                    #If object of load control test exists just update test parameters
                    self._mecTest.update_parameters(self.end_force1, self.end_force2, self.test_duration)

                else:
                    #Check if the test should be cycled
                    if self.checkBoxCycl.isChecked():
                        self._mecTest = LoadControlTest(self._work_folder, self.end_force1, self.end_force2, self.test_duration, self.cycl_num)
                    else:
                        #0 - one directional test
                        self._mecTest = LoadControlTest(self._work_folder, self.end_force1, self.end_force2, self.test_duration, 0)


                    self._mecTest.update_matplotlib_signal.connect(self.__update_charts)
                    self._mecTest.update_force_label_signal.connect(self.__updateLabelForce)
        
            elif 2 == self.tabWidget.currentIndex():
                '''
                Displacement Control Test!
                '''
                
                # Calculate velocity
                self._vel_ax1 = -float(self.factorSpeedAx1.text())/60
                self._vel_ax2 = -float(self.factorSpeedAx2.text())/60
                
                if hasattr(self, '_mecTest') and isinstance(self._mecTest, DisplacementControlTest):
                    #If object of displacement control test exists just update test parameters
                    self._mecTest.update_speed(self._vel_ax1, self._vel_ax2)
                    
                else:
                    #Close old object to prevent issues in connection with devices
                    self._mecTest = DisplacementControlTest(self._work_folder, self._vel_ax1, self._vel_ax2)
                    self._mecTest.update_matplotlib_signal.connect(self.__update_charts)
                    self._mecTest.update_force_label_signal.connect(self.__updateLabelForce)
                
                # Update UI labels
                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Displacement control test")
            
    
        except Exception as e:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText(e.__str__())
            warning_box.exec()
            #delattr(self, '_mecTest')
        else:
            self._liveforce_timer = QTimer()
            self._liveforce_timer.timeout.connect(self._mecTest.readForceLive)
            self._liveforce_timer.start(500)

            
    def __stop_movement(self):
        
        
        
        if hasattr(self, '_mecTest'):
            self._mecTest.stop_measurement()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()

        if hasattr(self, '_label_timer'):
            self._label_timer.stop()

        if hasattr(self, '_liveforce_timer'):
            self._liveforce_timer.start(500)
            
        
            
    def __changeLabelColor(self):
        '''
        Function to change color of the upper label during test execution

        Returns
        -------
        None.

        '''
        
        if hasattr(self, 'fl_label_color'):
            if self.fl_label_color:
                self.upperLabel_1.setStyleSheet("color: red; font-size: 14px;")
                self.upperLabel_2.setStyleSheet("color: red; font-size: 14px;")
            else:
                self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
                
            self.fl_label_color = not self.fl_label_color
        else:
            self.fl_label_color = True
        
    def __zeroForce(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.zeroForce()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
        
    def __zeroPosition(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.zeroPosition()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
       
    def __moveSamplePosition(self):
        # MOve to position for sample attachment
        if hasattr(self, '_mecTest'):
            self._mecTest.moveSamplePosition()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
            
    def __initializeMotors(self):

        if hasattr(self, '_mecTest'):
            # Show a message box with Yes/No buttons
            reply = QMessageBox.question(self, 'Message',
                                        "Is the sample unmounted?",
                                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                        QMessageBox.StandardButton.No)

            # Check if 'Yes' was clicked
            if reply == QMessageBox.StandardButton.Yes:
                #Initialize Zero position of the motors
                self._mecTest.initMotZeroPos()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveForwardAxis1(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.moveVelocityAx1(1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveBackwardAxis1(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.moveVelocityAx1(-1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveForwardAxis2(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.moveVelocityAx2(1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveBackwardAxis2(self):
        if hasattr(self, '_mecTest'):
            self._mecTest.moveVelocityAx2(-1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    #Other Functions
            
    def __calculateLoads (self):
        self._calc_loads_window = LoadCalculatorWindow()
        self._calc_loads_window.signal_loads_calculated.connect(self.__setLoads)

        self._calc_loads_window.show()

    
    def __setLoads(self, load1, load2):
        self.end_force1 = load1
        self.end_force2 = load2

        self.factorForceAx1.setText(str(load1))
        self.factorForceAx2.setText(str(load2))
    
    
    def __startCamera(self):
        """
        Method to start the camera. If the '_mecTest' attribute is present, initializes a 
        VideoThread and VideoWindow, connects the start_stop_tracking_signal to 
        _video_window.startStopTracking, and shows the video window. If the '_mecTest' 
        attribute is not present, displays a warning message. 
        """
        
        if hasattr(self, '_mecTest'):
        
            self._video_thread = VideoThread()
            self._video_window = VideoWindow(self._video_thread)

            self._video_thread.signal_marks_recorded.connect(self._mecTest.marksRecorded)
            self._video_thread.change_pixmap_signal.connect(self._video_window.update_image)
            #self._mecTest.start_stop_tracking_signal.connect(self._video_window.startStopTracking)
            self._mecTest.change_pixmap_signal.connect(self._video_window.update_image)

            
            self._video_window.show()
            self._video_thread.start()  
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
        



    
    def __updatePID(self):
        if hasattr(self, '_mecTest') and isinstance(self._mecTest, LoadControlTest):
            P1 = self.factorKp1.value()
            I1 = self.factorKi1.value()
            D1 = self.factorKd1.value()
            P2 = self.factorKp2.value()
            I2 = self.factorKi2.value()
            D2 = self.factorKd2.value()
            self._mecTest.updatePID(P1, I1, D1, P2, I2, D2)
    
        

    def __update_charts(self, t, ch1, ch2, l1, l2, E11, E22, v1, v2):
        """
        Updates charts on matplotlib widgets
        This function is connected to MechanicalTest classes with signal/slot mechanism
        """

        self._t.append(t)
        self._ch1.append(ch1) 
        self._ch2.append(ch2) 
        self._l1.append(l1) 
        self._l2.append(l2) 
        self._E11.append(E11) 
        self._E22.append(E22) 
        self._v1.append(v1) 
        self._v2.append(v2)
        
        t_s = [0, self.test_duration]
        f_s = [0, self.end_force1]
    
        
        """
        self.MplWidget_1.canvas.axes.clear()
        self.MplWidget_1.canvas.axes.plot(self._t, self._ch1)
        self.MplWidget_1.canvas.axes.plot(self._t, self._ch2)
        if 0 == self.tabWidget.currentIndex():
            self.MplWidget_1.canvas.axes.plot(t_s, f_s)
        self.MplWidget_1.canvas.axes.legend(('ch1','ch2', 'setpoint'),loc='upper right')
        self.MplWidget_1.canvas.axes.set_title('Load vs. Time')
        self.MplWidget_1.canvas.axes.set_ylabel('Load, N')
        self.MplWidget_1.canvas.axes.set_xlabel('Time, s')
        self.MplWidget_1.canvas.draw()
        """
        
        self.MplWidget_2.canvas.axes.clear()
        self.MplWidget_2.canvas.axes.plot(self._v1)
        self.MplWidget_2.canvas.axes.plot(self._v2)
        self.MplWidget_2.canvas.axes.legend(('ch1','ch2'),loc='upper right')
        #self.MplWidget_2.canvas.axes.set_title('Displacement vs. Time')
        self.MplWidget_2.canvas.axes.set_ylabel('Velocity')
        self.MplWidget_2.canvas.axes.set_xlabel('Time, s')
        self.MplWidget_2.canvas.draw()
        
        
        self.MplWidget_3.canvas.axes.clear()
        if len(self._E11) == len (self._ch1):
        #if self._mecTest.getMarksStatus():
            #If initial marks are recorded than strain will be calculated and can be plotted
            self.MplWidget_3.canvas.axes.plot(self._E11, self._ch1)
            self.MplWidget_3.canvas.axes.plot(self._E22, self._ch2)
            self.MplWidget_3.canvas.axes.set_xlabel('Strain, %')
        else:
            self.MplWidget_3.canvas.axes.plot(self._l1, self._ch1)
            self.MplWidget_3.canvas.axes.plot(self._l2, self._ch2)
            self.MplWidget_3.canvas.axes.set_xlabel('Displacement, mm')
            
        self.MplWidget_3.canvas.axes.legend(('ch1','ch2'),loc='upper right')
        self.MplWidget_3.canvas.axes.set_title('Load vs. Strain')
        self.MplWidget_3.canvas.axes.set_ylabel('Load, N')
        self.MplWidget_3.canvas.draw()
        
        

        
    def __updateLabelForce(self, force1, force2):
        if force1 is not None:
            self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
            self.upperLabel_1.setText("Force 1: {}".format(force1))
            self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
            self.upperLabel_2.setText("Force 2: {}".format(force2))



    def __openFolderDialog(self):
        self._work_folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if self._work_folder:  # check if a folder is selected
            print("Selected folder: ", self._work_folder)
            self.labelFolder.setText(self._work_folder)
            
        if hasattr(self, '_mecTest'):
            self._mecTest.changeFolder(self._work_folder)

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()