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
import json

from .mechanicaltests import DisplacementControlTest, LoadControlTest, MechanicalTest
from .camerawindow import VideoThread, VideoWindow
from .loadcalculator import LoadCalculatorWindow
from .motordaqinterface import MotorDAQInterface
from .ringbuffer import RingBuffer
from .unit import Unit




     
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

        #Flags
        self._fl_sample_initialized = False
        self._fl_executing = False

        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        ui_file_path = os.path.join(script_dir, '..', 'resources', 'biax_ui.ui')

        print(ui_file_path)

        # Load the .ui file
        loadUi(ui_file_path, self)

        #loadUi("biax_ui.ui",self)

        self.__init_variables()

        self._ringbuffer1 = RingBuffer(100) #Ring buffer for live force display of channel1
        self._ringbuffer2 = RingBuffer(100) #Ring buffer for live force display of channel2

        # Construct the path to the desktop folder
        home_dir = os.path.expanduser('~')
        self._work_folder = os.path.join(home_dir, 'Desktop', 'Biax_Tests')

        self._load_test_config()

        self.__change_units()
        
        
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
        self.buttonCalcLoad.clicked.connect(self.__calculate_loads)
        self.buttonAutoload.clicked.connect(self.__autoload)
        
        self.buttonMoveCentAx1.pressed.connect(self.__moveForwardAxis1)
        self.buttonMoveCentAx2.pressed.connect(self.__moveForwardAxis2)
        self.buttonMoveBackAx1.pressed.connect(self.__moveBackwardAxis1)
        self.buttonMoveBackAx2.pressed.connect(self.__moveBackwardAxis2)
        
        self.buttonMoveCentAx1.released.connect(self.__stop_movement)
        self.buttonMoveCentAx2.released.connect(self.__stop_movement)
        self.buttonMoveBackAx1.released.connect(self.__stop_movement)
        self.buttonMoveBackAx2.released.connect(self.__stop_movement)

        self.buttonGrams.toggled.connect(self.__change_units)
        
        self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
        self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")

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
        if self._units == Unit.Newton:
            self.ChartWidget_1.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
        else:
            self.ChartWidget_1.setLabel('left', 'Load, g', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
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
        self.ChartWidget_2.setLabel('left', 'Displacement, mm', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
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
        
        

        


    def _load_test_config(self):

        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'resources', 'config_test.json')

        if os.path.exists(config_path):
            with open(config_path, 'r') as file:
                self._config_test = json.load(file)
        else:
            raise FileNotFoundError(f"The configuration file {config_path} does not exist.")


        unit = self._config_test.get("unit")
        if unit == "gram":
            self._units = Unit.Gram
            self.buttonGrams.setChecked(True)
        else:
            self._units = Unit.Newton
            self.buttonNewtons.setChecked(True)

        
        self._work_folder = self._config_test.get("folder")
        print("Work folder: " + self._work_folder)
        self.labelFolder.setText(self._work_folder)
    
        self._sam_name = "" #self._config_test.get("name")
        self.factorSampleName.setText(self._sam_name)
        self.factorTareLoad1.setText(self._config_test.get("tareload1"))
        self.factorTareLoad2.setText(self._config_test.get("tareload2"))
        self.factorLoadTest1.setText(self._config_test.get("load_test1"))
        self.factorLoadTest2.setText(self._config_test.get("load_test2"))
        self.factorDispGuess1.setText(self._config_test.get("disp_guess1"))
        self.factorDispGuess2.setText(self._config_test.get("disp_guess2"))
        self.factorTimeAx.setText(self._config_test.get("halfcycltime"))
        self.factorCyclNumPrecond.setText(self._config_test.get("cycleprecond"))
        self.factorCyclNumTest.setText(self._config_test.get("cycletest"))

        #Displacement test
        self.factorSpeedAx1.setText(self._config_test.get("disptestspeed1"))
        self.factorSpeedAx2.setText(self._config_test.get("disptestspeed2"))
        self.factorLength1.setText(self._config_test.get("disptestlen1"))
        self.factorLength2.setText(self._config_test.get("disptestlen2"))
        self.factorCyclNumD.setText(self._config_test.get("disptestcycle"))

        self._thickness = self._config_test.get("thickness")
        self._len1 = self._config_test.get("length1")
        self._len2 = self._config_test.get("length2")
        self._max_stress1 = self._config_test.get("maxstress1")
        self._max_stress2 = self._config_test.get("maxstress2")

        self._area1 = self._thickness * self._len1
        self._area2 = self._thickness * self._len2

    def _save_test_config(self):

        if self._units == Unit.Gram:
            self._config_test["unit"] = "gram"
        else:
            self._config_test["unit"] = "newton"


        self._config_test["name"] = self.factorSampleName.text()
        self._config_test["tareload1"] = self.factorTareLoad1.text()
        self._config_test["tareload2"] = self.factorTareLoad2.text()
        self._config_test["load_test1"] = self.factorLoadTest1.text()
        self._config_test["load_test2"] = self.factorLoadTest2.text()
        self._config_test["disp_guess1"] = self.factorDispGuess1.text()
        self._config_test["disp_guess2"] = self.factorDispGuess2.text()
        self._config_test["halfcycltime"] = self.factorTimeAx.text()
        self._config_test["cycleprecond"] = self.factorCyclNumPrecond.text()
        self._config_test["cycletest"] = self.factorCyclNumTest.text()

        #Disp test
        self._config_test["disptestspeed1"] = self.factorSpeedAx1.text()
        self._config_test["disptestspeed2"] = self.factorSpeedAx2.text()
        self._config_test["disptestlen1"] = self.factorLength1.text()
        self._config_test["disptestlen2"] = self.factorLength2.text()
        self._config_test["disptestcycle"] = self.factorCyclNumD.text()

        self._config_test["thickness"] = self._thickness
        self._config_test["length1"] = self._len1
        self._config_test["length2"] = self._len2
        self._config_test["maxstress1"] = self._max_stress1
        self._config_test["maxstress2"] = self._max_stress2
        self._config_test["folder"] = self._work_folder

        
        #Save into program folder
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'resources', 'config_test.json')
        with open(config_path, 'w') as file:
            json.dump(self._config_test, file, indent=4)

        
        #Save second time into sample folder
        os.makedirs(self._work_folder, exist_ok=True)
        config_path2 = os.path.join(self._work_folder, 'config_test.json')
        with open(config_path2, 'w') as file:
            json.dump(self._config_test, file, indent=4)
        

    
    def __init_variables(self):
        self._t = [] 
        self._load1 = [] 
        self._load2 = []
        self._stress1 = []
        self._stress2 = [] 
        self._disp1 = [] 
        self._disp2 = [] 
        self._E11 = [] 
        self._E22 = [] 
        self._v1 = [] 
        self._v2 = []

    @pyqtSlot() #To clear charts then preconditioning or test are finished
    def reset(self):
        self.__init_variables()
        self.ChartWidget_1.clear()
        self.ChartWidget_2.clear()
        self.ChartWidget_3.clear()


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
        self._save_test_config()
        self.block_gui()

        #Motors and DAQ should be initialized
        if not hasattr(self, '_mot_daq') or not self._mot_daq.is_initialized():
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Motors and DAQ not connected or not initialized!")
            warning_box.exec()
            return
        
        '''if not self._fl_sample_initialized:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText("Sample dimensions should be initizlied")
            warning_box.exec()
            self.__calculate_loads()
            return'''

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

        self._fl_executing = True
        self._mec_test.start()


            
        
            

    def __create_mec_test(self):
        '''
        Created an instance of appropriate test based on the tab in the tabWidget
        
        tabWidget:
        0 - load control
        1 - strain control
        2 - displacement control
        '''

        '''#If there is an old object -stop it
        if hasattr(self, '_mec_test') and isinstance(self._mec_test, LoadControlTest):
            self._mec_test.stop_measurement()'''

        try:

            #workfolder is checked when button pressed
            self._sam_name = self.factorSampleName.text()
            #Check if folder with sample name already exist
            #If yes ask user if he wants to add data there
            sam_folder = os.path.join(self._workfolder, self._sam_name)
            if os.path.exists(sam_folder):
                reply = QMessageBox.question(self, 'Message',
                                        f"Do you want to add data to {self._sam_name}?",
                                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                        QMessageBox.StandardButton.No)

                if reply == QMessageBox.StandardButton.No:
                    return
            

            #Load control test
            if 0 == self.tabWidget.currentIndex():
                
                
                # Get test parameters
                end_force1 = float(self.factorLoadTest1.text())
                end_force2 = float(self.factorLoadTest2.text())
                disp_guess1 = float(self.factorDispGuess1.text())/2
                disp_guess2 = float(self.factorDispGuess2.text())/2
                test_duration = float(self.factorTimeAx.text())
                cycl_num_precond = int(self.factorCyclNumPrecond.text())
                cycl_num_test = int(self.factorCyclNumTest.text())
                
                if hasattr(self, '_mec_test') and isinstance(self._mec_test, LoadControlTest):
                    #If object of load control test exists just update test parameters
                    self._mec_test.update_parameters(self._work_folder, self._sam_name, end_force1, end_force2, disp_guess1, disp_guess2, test_duration, cycl_num_precond, cycl_num_test)

                else:
                    self._mec_test = LoadControlTest(self._mot_daq, self._work_folder, self._sam_name, end_force1, end_force2, disp_guess1, disp_guess2, test_duration, cycl_num_precond, cycl_num_test)
                    self.signal_stop.connect(self._mec_test.stop)
                    self._mec_test.signal_update_charts.connect(self.__update_charts)
                    self._mec_test.signal_precond_finished.connect(self.reset)
                    #self._mec_test.signal_test_finished.connect(self.reset)
        
            #Displacement control test
            elif 2 == self.tabWidget.currentIndex():
                
                
                # Calculate velocity
                vel_ax1 = float(self.factorSpeedAx1.text())/120 #convert from mm/min to mm/sec and divide by 2 as one axis pulls sample from 2 sites
                vel_ax2 = float(self.factorSpeedAx2.text())/120
                length1 = float(self.factorLength1.text())/2
                length2 = float(self.factorLength2.text())/2
                cycl_num = int(self.factorCyclNumD.text())

                self._mec_test = DisplacementControlTest(self._mot_daq, self._work_folder, self._sam_name, vel_ax1, vel_ax2, length1, length2, cycl_num)
                self.signal_stop.connect(self._mec_test.stop)
                self._mec_test.signal_update_charts.connect(self.__update_charts)
                self._mec_test.signal_precond_finished.connect(self.reset)
                #self._mec_test.signal_test_finished.connect(self.reset)
                
                
                # Update UI labels
                self.upperLabel_1.setText("Warning!")
                self.upperLabel_2.setText("Displacement control test")

        except Exception as e:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText(e.__str__())
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

    def __connect(self):
        
        try:
            self._mot_daq = MotorDAQInterface(self._units)
            self.signal_stop.connect(self._mot_daq.stop)
    
        except Exception as e:
            warning_box = QMessageBox()
            warning_box.setIcon(QMessageBox.Icon.Warning)
            warning_box.setWindowTitle("Warning")
            warning_box.setText(e.__str__())
            warning_box.exec()
            print("Error")
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
            self._liveforce_timer.start(200)

        self._fl_executing = False
        self._ringbuffer1.reset()
        self._ringbuffer2.reset()
        self.unblock_gui()
            
        
            
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
            
    def __calculate_loads (self):

        self._fl_sample_initialized = True
        self._calc_loads_window = LoadCalculatorWindow(self._thickness, self._len1, self._len2, self._max_stress1, self._max_stress2)
        self._calc_loads_window.signal_loads_calculated.connect(self.__set_sample_params)

        self._calc_loads_window.show()
        

    
    @pyqtSlot(float,float,float,float,float,float,float)
    def __set_sample_params(self, load1, load2, stress1, stress2, len1, len2, thickness):
        self.end_force1 = load1
        self.end_force2 = load2
        self._max_stress1 = stress1
        self._max_stress2 = stress2

        self.factorLoadTest1.setText(str(load1))
        self.factorLoadTest2.setText(str(load2))

        self._len1 = len1
        self._len2 = len2
        self._thickness = thickness

        self._area1 = len1 * thickness #in mm2
        self._area2 = len2 * thickness #in mm2

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
    
    

    
        
    @pyqtSlot(list)
    def __update_charts(self, array):
        """
        Updates charts on matplotlib widgets
        This function is connected to MechanicalTest classes with signal/slot mechanism
        Order:
        self._time, self._load1[-1], self._load2[-1], self._disp1[-1], self._disp2[-1], self._E11[-1], self._E22[-1], self._vel_1[-1],self._vel_2[-1]
        """

        self._t.append(array[0])
        self._load1.append(array[1]) 
        self._load2.append(array[2])
        self._disp1.append(array[3]) 
        self._disp2.append(array[4]) 

        #Stress calculation depends on force units used
        if Unit.Newton == self._units:
            self._stress1.append(array[1]/self._area1)  #MPa
            self._stress2.append(array[2]/self._area2)   
        else:
            self._stress1.append(array[1]*0.0098/self._area1)  #MPa
            self._stress2.append(array[2]*0.0098/self._area2)  #MPa

        if 9 == len(array):
            self._E11.append(array[5]) 
            self._E22.append(array[6]) 
            self._v1.append(array[7]) 
            self._v2.append(array[8])
        
        #t_s = [0, self.test_duration]
        #f_s = [0, self.end_force1]
    
        


        self.ChartWidget_1.clear()
        self.ChartWidget_1.plot(self._t, self._load1, pen=pg.mkPen(color='b', width=2))  
        self.ChartWidget_1.plot(self._t, self._load2, pen=pg.mkPen(color='r', width=2))  
        #if 0 == self.tabWidget.currentIndex():
        #    self.ChartWidget_1.plot(t_s, f_s)

        
        self.ChartWidget_2.clear()
        self.ChartWidget_2.plot(self._t, self._disp1, pen=pg.mkPen(color='b', width=2))  
        self.ChartWidget_2.plot(self._t, self._disp2, pen=pg.mkPen(color='r', width=2))  

        self.ChartWidget_3.clear()
        if len(self._E11) == len (self._load1):
            #print(self._E11)
            self.ChartWidget_3.plot(self._E11, self._stress1, pen=pg.mkPen(color='b', width=2)) 
            self.ChartWidget_3.plot(self._E22, self._stress2, pen=pg.mkPen(color='r', width=2)) 
        else:
            self.ChartWidget_3.plot(self._disp1, self._load1, pen=pg.mkPen(color='b', width=2)) 
            self.ChartWidget_3.plot(self._disp2, self._load2, pen=pg.mkPen(color='r', width=2)) 
        
        """
        self.MplWidget_1.canvas.axes.clear()
        self.MplWidget_1.canvas.axes.plot(self._t, self._load1)
        self.MplWidget_1.canvas.axes.plot(self._t, self._load2)
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
        if len(self._E11) == len (self._load1):
        #if self._mec_test.getMarksStatus():
            #If initial marks are recorded than strain will be calculated and can be plotted
            self.MplWidget_3.canvas.axes.plot(self._E11, self._load1)
            self.MplWidget_3.canvas.axes.plot(self._E22, self._load2)
            self.MplWidget_3.canvas.axes.set_xlabel('Strain, %')
        else:
            self.MplWidget_3.canvas.axes.plot(self._disp1, self._load1)
            self.MplWidget_3.canvas.axes.plot(self._disp2, self._load2)
            self.MplWidget_3.canvas.axes.set_xlabel('Displacement, mm')
            
        self.MplWidget_3.canvas.axes.legend(('ch1','ch2'),loc='upper right')
        self.MplWidget_3.canvas.axes.set_title('Load vs. Strain')
        self.MplWidget_3.canvas.axes.set_ylabel('Load, N')
        self.MplWidget_3.canvas.draw()
        """
        

    def __updateLabelForce(self):
        if self._mot_daq is not None:
            force1, force2 = self._mot_daq.get_av_forces()

            if Unit.Gram == self._units:
                #update label with current forces
                self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_1.setText("Force 1: {} g".format(round(force1,2)))
                self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_2.setText("Force 2: {} g".format(round(force2,2)))
            else:
                #update label with current forces
                self.upperLabel_1.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_1.setText("Force 1: {} N".format(round(force1,4)))
                self.upperLabel_2.setStyleSheet("color: black; font-size: 14px;")
                self.upperLabel_2.setText("Force 2: {} N".format(round(force2,4)))

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
            self._mec_test.changeFolder(self._work_folder, self.factorSampleName.text())

    def __change_units(self):
            

        if self.buttonGrams.isChecked():
            self._units = Unit.Gram
            self.ChartWidget_1.setLabel('left', 'Load, g', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
            self.label_17.setText("g")
            self.label_18.setText("g")
            self.label_26.setText("g")
            self.label_29.setText("g")
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()
        else:
            self._units = Unit.Newton
            self.ChartWidget_1.setLabel('left', 'Load, N', **{'color': '#000', 'font-size': '14pt', 'font-family': 'Arial'})
            self.label_17.setText("N")
            self.label_18.setText("N")
            self.label_26.setText("N")
            self.label_29.setText("N")
            self._ringbuffer1.reset()
            self._ringbuffer2.reset()

        #Update the units in a class responsible for communicaiton with DAQ
        if hasattr(self, '_mot_daq'):
            self._mot_daq.change_units(self._units)


    def block_gui(self):
        self.buttonNewtons.setDisabled(True)
        self.buttonGrams.setDisabled(True)
        self.factorTareLoad1.setDisabled(True)
        self.factorTareLoad2.setDisabled(True)
        self.factorLoadTest1.setDisabled(True)
        self.factorLoadTest2.setDisabled(True)
        self.factorDispGuess1.setDisabled(True)
        self.factorDispGuess2.setDisabled(True)
        self.factorTimeAx.setDisabled(True)
        self.factorCyclNumPrecond.setDisabled(True)
        self.factorCyclNumTest.setDisabled(True)
        self.factorSampleName.setDisabled(True)
        self.buttonFolder.setDisabled(True)
        self.buttonZeroPosition.setDisabled(True)
        self.buttonZeroForce.setDisabled(True)
        self.buttonSampleP.setDisabled(True)
        self.buttonInitMot.setDisabled(True)
        self.buttonMoveCentAx1.setDisabled(True)
        self.buttonMoveCentAx2.setDisabled(True)
        self.buttonMoveBackAx1.setDisabled(True)
        self.buttonMoveBackAx2.setDisabled(True)
        self.buttonAutoload.setDisabled(True)
        self.buttonCalcLoad.setDisabled(True)
        self.buttonConnect.setDisabled(True)
        self.buttonCamera.setDisabled(True)

    def unblock_gui(self):
        self.buttonNewtons.setDisabled(False)
        self.buttonGrams.setDisabled(False)
        self.factorTareLoad1.setDisabled(False)
        self.factorTareLoad2.setDisabled(False)
        self.factorLoadTest1.setDisabled(False)
        self.factorLoadTest2.setDisabled(False)
        self.factorDispGuess1.setDisabled(False)
        self.factorDispGuess2.setDisabled(False)
        self.factorTimeAx.setDisabled(False)
        self.factorCyclNumPrecond.setDisabled(False)
        self.factorCyclNumTest.setDisabled(False)
        self.factorSampleName.setDisabled(False)
        self.buttonFolder.setDisabled(False)
        self.buttonZeroPosition.setDisabled(False)
        self.buttonZeroForce.setDisabled(False)
        self.buttonSampleP.setDisabled(False)
        self.buttonInitMot.setDisabled(False)
        self.buttonMoveCentAx1.setDisabled(False)
        self.buttonMoveCentAx2.setDisabled(False)
        self.buttonMoveBackAx1.setDisabled(False)
        self.buttonMoveBackAx2.setDisabled(False)
        self.buttonAutoload.setDisabled(False)
        self.buttonCalcLoad.setDisabled(False)
        self.buttonConnect.setDisabled(False)
        self.buttonCamera.setDisabled(False)



if __name__ == '__main__':
    #start of the application
    app = QApplication([])
    app.setStyle('Fusion')
    window = BiaxMainWindow()
    window.show()
    app.exec()