"""

@author: Viktor B

"""

from PyQt6.QtWidgets import*
from PyQt6.uic import loadUi
from PyQt6.QtCore import QTimer, pyqtSlot, pyqtSignal
from PyQt6.QtGui import QFont
#from PyQt5.QtChart import QLineSeries
import pyqtgraph as pg

import os

from .mechanicaltests import DisplacementControlTest, LoadControlTest, MechanicalTest
from .camerawindow import VideoThread, VideoWindow
from .loadcalculator import LoadCalculatorWindow
from .motordaqinterface import MotorDAQInterface
from .ringbuffer import RingBuffer




     
class BiaxMainWindow(QMainWindow):
    """
    BiaxMainWindow class inherits from QMainWindow.
    This class is the main application window for a biaxial testing machine.
    It sets up the user interface and connects GUI elements to their functionalities.
    """

    signal_stop = pyqtSignal()
    
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
        self.buttonStart.clicked.connect(self.__start_test)
        self.buttonStop.clicked.connect(self.__stop_test)
        self.buttonSampleP.clicked.connect(self.__moveSamplePosition)
        self.buttonInitMot.clicked.connect(self.__initializeMotors)
        self.buttonCamera.clicked.connect(self.__startCamera)
        self.buttonFolder.clicked.connect(self.__openFolderDialog)
        self.buttonZeroForce.clicked.connect(self.__zeroForce)
        self.buttonZeroPosition.clicked.connect(self.__zeroPosition)
        self.buttonConnect.clicked.connect(self.__connect)
        self.buttonCalcLoad.clicked.connect(self.__calculateLoads)
        self.buttonAutoload.clicked.connect(self.__autoload)
        
        self.buttonMoveCentAx1.pressed.connect(self.__moveForwardAxis1)
        self.buttonMoveCentAx2.pressed.connect(self.__moveForwardAxis2)
        self.buttonMoveBackAx1.pressed.connect(self.__moveBackwardAxis1)
        self.buttonMoveBackAx2.pressed.connect(self.__moveBackwardAxis2)
        
        self.buttonMoveCentAx1.released.connect(self.__stop_movement)
        self.buttonMoveCentAx2.released.connect(self.__stop_movement)
        self.buttonMoveBackAx1.released.connect(self.__stop_movement)
        self.buttonMoveBackAx2.released.connect(self.__stop_movement)
        
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
        font = QFont()
        font.setPointSize(15)
        self.ChartWidget_1.getAxis('left').setTickFont(font)
        self.ChartWidget_1.getAxis('bottom').setTickFont(font)
        self.ChartWidget_1.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        self.ChartWidget_1.setLabel('bottom', 'Time, s', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})

        self.ChartWidget_2.setBackground('w')  # Set background to white
        self.ChartWidget_2.getAxis('left').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('bottom').setPen(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('left').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_2.getAxis('bottom').setTextPen(pg.mkPen(color='k'))
        self.ChartWidget_2.showGrid(x=False, y=False, alpha=0.5)
        self.ChartWidget_2.getPlotItem().getViewBox().setBorder(pg.mkPen(color='k', width=1))
        self.ChartWidget_2.getAxis('left').setTickFont(font)
        self.ChartWidget_2.getAxis('bottom').setTickFont(font)
        self.ChartWidget_2.setLabel('left', 'Velocity', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
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
        self.ChartWidget_3.setLabel('left', 'Stress, MPa', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        self.ChartWidget_3.setLabel('bottom', 'Strain, %', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        
        self._ringbuffer1 = RingBuffer(100) #Ring buffer for live force display of channel1
        self._ringbuffer2 = RingBuffer(100) #Ring buffer for live force display of channel2

        self.__init_variables()
        

    def __init_variables(self):
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
        event.accept()

    def __terminate_application(self):
        
        self.signal_stop.emit()
        
        # Terminate Threads
        '''if hasattr(self, '_mec_test'):
            self._mec_test.stop()

        if hasattr(self, '_video_thread'):
            self._video_thread.stop()
        
        if hasattr(self, '_video_window'):
            self._video_window.close()
            
        if hasattr(self, '_label_timer'):
            self._label_timer.stop()

        if hasattr(self, '_liveforce_timer'):
            self._liveforce_timer.stop()
            
        self.close()'''
            
            
        
        
        print('Application terminated')
        
    
    #functions connected to buttons
    
    def __start_test(self):
        """
        Start the measurement process, connect tracking signal, and set up timer for label updates.
        Set end force values, test duration, and cycle number. 
        If load control test object exists, update test parameters and start the test. 
        If not, display a warning message to connect to motors and DAQ first.
        """

        #Reset all the variables
        self.__init_variables()

        if hasattr(self, '_mot_daq') and self._mot_daq.is_initialized():

            self._mot_daq.zeroPosition()

            #Setting lavel properties
            self._liveforce_timer.stop()
            
            if hasattr(self, '_label_timer'):
                self._label_timer.stop()
                self._label_timer.start(1000)  # 1000 milliseconds = 1 second
            else:
                self._label_timer = QTimer(self)
                self._label_timer.timeout.connect(self.__changeLabelColor)
                self._label_timer.start(1000)  # 1000 milliseconds = 1 second
            
            
            #Setting michanical test
            self.__create_mec_test()

            if isinstance(self._mec_test, LoadControlTest):
                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Load control test")

            elif isinstance(self._mec_test, DisplacementControlTest):
                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Displacement control test")

            self._mec_test.start()


            
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Motors and DAQ not connected or not initialized!")
            warning_box.exec()

    def __create_mec_test(self):
        '''
        Created an instance of appropriate test based on the tab in the tabWidget
        
        tabWidget:
        0 - load control
        1 - strain control
        2 - displacement control
        '''

        #If there is an old object -stop it
        if hasattr(self, '_mec_test') and isinstance(self._mec_test, LoadControlTest):
            self._mec_test.stop_measurement()

        try:
            #Load control test
            if 0 == self.tabWidget.currentIndex():
                
                
                # Get test parameters
                end_force1 = float(self.factorLoadTest1.text())
                end_force2 = float(self.factorLoadTest2.text())
                disp_guess1 = float(self.factorDispGuess1.text())
                disp_guess2 = float(self.factorDispGuess2.text())
                test_duration = float(self.factorTimeAx.text())
                cycl_num = int(self.factorCyclNumL.text())

                self._mec_test = LoadControlTest(self._mot_daq, self._work_folder, end_force1, end_force2, disp_guess1, disp_guess2, test_duration, cycl_num)
                self.signal_stop.connect(self._mec_test.stop)
                self._mec_test.signal_update_charts.connect(self.__update_charts)
                
                '''if hasattr(self, '_mec_test') and isinstance(self._mec_test, LoadControlTest):
                    #If object of load control test exists just update test parameters
                    self._mec_test.update_parameters(self.end_force1, self.end_force2, self.test_duration, self.cycl_num)

                else:
                    self._mec_test = LoadControlTest(self._mot_daq, self._work_folder, self.end_force1, self.end_force2, self.test_duration, self.cycl_num)
                    
                    self.signal_stop.connect(self._mec_test.stop)
                    self._mec_test.signal_update_charts.connect(self.__update_charts)'''
        
            #Displacement control test
            elif 2 == self.tabWidget.currentIndex():
                
                
                # Calculate velocity
                vel_ax1 = float(self.factorSpeedAx1.text())/120 #convert from mm/min to mm/sec and divide by 2 as one axis pulls sample from 2 sites
                vel_ax2 = float(self.factorSpeedAx2.text())/120

                length1 = float(self.factorLength1.text())
                length2 = float(self.factorLength2.text())

                cycl_num = int(self.factorCyclNumD.text())

                self._mec_test = DisplacementControlTest(self._mot_daq, self._work_folder, vel_ax1, vel_ax2, length1, length2, cycl_num)
                self.signal_stop.connect(self._mec_test.stop)
                self._mec_test.signal_update_charts.connect(self.__update_charts)
                
                '''if hasattr(self, '_mec_test') and isinstance(self._mec_test, DisplacementControlTest):
                    #If object of displacement control test exists just update test parameters
                    self._mec_test.update_parameters(self._vel_ax1, self._vel_ax2, self._length1, self._length2, self.cycl_num)
                    
                else:
                    #Close old object to prevent issues in connection with devices
                    self._mec_test = DisplacementControlTest(self._mot_daq, self._work_folder, self._vel_ax1, self._vel_ax2, self._length1, self._length2, self.cycl_num)
                    self._mec_test.signal_update_charts.connect(self.__update_charts)'''
                
                # Update UI labels
                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Displacement control test")

        except Exception as e:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText(e.__str__())
            warning_box.exec()           

    def __connect(self):
        
        try:
            self._mot_daq = MotorDAQInterface()
            self.signal_stop.connect(self._mot_daq.stop)
    
        except Exception as e:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText(e.__str__())
            warning_box.exec()
            #delattr(self, '_mec_test')
        else:
            self._liveforce_timer = QTimer(self)
            self._liveforce_timer.timeout.connect(self.__updateLabelForce)
            self._liveforce_timer.start(200)

            
    def __stop_movement(self):
        
        
        if hasattr(self, '_mot_daq'):
            self._mot_daq.stop_motors()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()

    def __stop_test(self):
        
        
        if hasattr(self, '_mot_daq'):
            self._mot_daq.stop_motors()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()

        if hasattr(self, '_mec_test'):
            self._mec_test.stop_measurement()

        if hasattr(self, '_label_timer'):
            self._label_timer.stop()

        if hasattr(self, '_liveforce_timer'):
            self._liveforce_timer.start(500)

        self._ringbuffer1.reset()
        self._ringbuffer2.reset()
            
        
            
    def __changeLabelColor(self):
        '''
        Function to change color of the upper label during test execution

        Returns
        -------
        None.

        '''
        
        if hasattr(self, '_fl_label_color'):
            if self._fl_label_color:
                self.upperLabel_1.setStyleSheet("color: red; font-size: 14px;")
                self.upperLabel_2.setStyleSheet("color: red; font-size: 14px;")
            else:
                self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
                
            self._fl_label_color = not self._fl_label_color
        else:
            self._fl_label_color = True
        
    def __zeroForce(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.zeroForce()
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
        
    def __zeroPosition(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.zeroPosition()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
       
    def __moveSamplePosition(self):
        # MOve to position for sample attachment
        if hasattr(self, '_mot_daq'):
            self._mot_daq.moveSamplePosition()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
            
    def __initializeMotors(self):

        if hasattr(self, '_mot_daq'):
            # Show a message box with Yes/No buttons
            reply = QMessageBox.question(self, 'Message',
                                        "Is the sample unmounted?",
                                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                        QMessageBox.StandardButton.No)

            # Check if 'Yes' was clicked
            if reply == QMessageBox.StandardButton.Yes:
                #Initialize Zero position of the motors
                self._mot_daq.initMotZeroPos()
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveForwardAxis1(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.move_velocity_ax1(1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveBackwardAxis1(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.move_velocity_ax1(-1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveForwardAxis2(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.move_velocity_ax2(1)
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    def __moveBackwardAxis2(self):
        if hasattr(self, '_mot_daq'):
            self._mot_daq.move_velocity_ax2(-1)
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

        self.factorLoadTest1.setText(str(load1))
        self.factorLoadTest2.setText(str(load2))

    def __autoload(self):
        if hasattr(self, '_mot_daq'):

            tare_load1 = float(self.factorTareLoad1.text())
            tare_load2 = float(self.factorTareLoad2.text())

            self._mot_daq.autoload(tare_load1, tare_load2)



        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
    
    def __startCamera(self):
        """
        Method to start the camera. If the '_mec_test' attribute is present, initializes a 
        VideoThread and VideoWindow, connects the start_stop_tracking_signal to 
        _video_window.startStopTracking, and shows the video window. If the '_mec_test' 
        attribute is not present, displays a warning message. 
        """
        
        if hasattr(self, '_mot_daq'):

            #Creating an instance of mechanical test
            self.__create_mec_test()
        
            self._video_thread = VideoThread()
            self.signal_stop.connect(self._video_thread.stop)
            self._video_window = VideoWindow(self._video_thread)
            self.signal_stop.connect(self._video_window.stop)

            self._video_thread.signal_markers_recorded.connect(self._mec_test.init_markers)
            self._video_thread.signal_markers_coordinates.connect(self._mec_test.update_markers)
            self._video_thread.signal_change_pixmap.connect(self._video_window.update_image)
            
            self._mec_test.signal_start_stop_tracking.connect(self._video_thread.start_stop_tracking)
            self._mec_test.signal_save_image.connect(self._video_thread.save_image)


            
            self._video_window.show()
            self._video_thread.start()  
        else:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Connect to motors and DAQ first!")
            warning_box.exec()
    
        
    @pyqtSlot(list)
    def __update_charts(self, array):
        """
        Updates charts on matplotlib widgets
        This function is connected to MechanicalTest classes with signal/slot mechanism
        Order:
        self._time, self._ch1[-1], self._ch2[-1], self._l1[-1], self._l2[-1], self._E11[-1], self._E22[-1], self._vel_1[-1],self._vel_2[-1]
        """

        self._t.append(array[0])
        self._ch1.append(array[1]) 
        self._ch2.append(array[2]) 
        self._l1.append(array[3]) 
        self._l2.append(array[4]) 

        if 9 == len(array):
            self._E11.append(array[5]) 
            self._E22.append(array[6]) 
            self._v1.append(array[7]) 
            self._v2.append(array[8])
        
        #t_s = [0, self.test_duration]
        #f_s = [0, self.end_force1]
    
        


        self.ChartWidget_1.clear()
        self.ChartWidget_1.plot(self._t, self._ch1, pen=pg.mkPen(color='b', width=2))  
        self.ChartWidget_1.plot(self._t, self._ch2, pen=pg.mkPen(color='r', width=2))  
        #if 0 == self.tabWidget.currentIndex():
        #    self.ChartWidget_1.plot(t_s, f_s)

        
        self.ChartWidget_2.clear()
        self.ChartWidget_2.plot(self._v1, pen=pg.mkPen(color='b', width=2))  
        self.ChartWidget_2.plot(self._v2, pen=pg.mkPen(color='r', width=2))  

        self.ChartWidget_3.clear()
        if len(self._E11) == len (self._ch1):
            #print(self._E11)
            self.ChartWidget_3.plot(self._E11, self._ch1, pen=pg.mkPen(color='b', width=2)) 
            self.ChartWidget_3.plot(self._E22, self._ch2, pen=pg.mkPen(color='r', width=2)) 
        else:
            self.ChartWidget_3.plot(self._l1, self._ch1, pen=pg.mkPen(color='b', width=2)) 
            self.ChartWidget_3.plot(self._l2, self._ch2, pen=pg.mkPen(color='r', width=2)) 
        
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
        #if self._mec_test.getMarksStatus():
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
        """
        

    def __updateLabelForce(self):
        if self._mot_daq is not None:
            force1, force2 = self._mot_daq.get_forces()

            #update label with current forces
            self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
            self.upperLabel_1.setText("Force 1: {}".format(force1))
            self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
            self.upperLabel_2.setText("Force 2: {}".format(force2))

            #add forces to the ring buffer
            self._ringbuffer1.append(force1)
            self._ringbuffer2.append(force2)

            #draw them in the chart
            self.ChartWidget_1.clear()
            self.ChartWidget_1.plot(self._ringbuffer1.get_buffer(), pen=pg.mkPen(color='b', width=2))  
            self.ChartWidget_1.plot(self._ringbuffer2.get_buffer(), pen=pg.mkPen(color='r', width=2))  





    def __openFolderDialog(self):
        self._work_folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if self._work_folder:  # check if a folder is selected
            print("Selected folder: ", self._work_folder)
            self.labelFolder.setText(self._work_folder)
            
        if hasattr(self, '_mec_test'):
            self._mec_test.changeFolder(self._work_folder)

if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()