# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

from PyQt6.QtCore import QThread, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QMessageBox
import numpy as np
import time
import csv
import cv2
import math
from datetime import datetime

from zaber_motion.ascii import Connection
from zaber_motion import Units
from vimba import *
import ginsapy.giutility.connect.PyQStationConnectWin as Qstation #module with communication functions to Gantner Q.Station under windows environment

from .markersdetection import markersDetection
from .pid import PID


class MechanicalTest (QThread):
    '''
    General class for tensile tests that includes general methods for hardware
    '''

    #Signal to update matplotlib
    update_matplotlib_signal = pyqtSignal(list, list, list, list, list,list,list, list, list)
    #Signal to start/stop tracking marks when test is run
    #start_stop_tracking_signal = pyqtSignal(bool)
    #Signal to update live force in GUI
    update_force_label_signal = pyqtSignal(float, float)
    change_pixmap_signal = pyqtSignal(np.ndarray)
    
    _force1_0 = 0 
    _force2_0 = 0
    _pos1_0 = 0
    _pos2_0 = 0

    
    _sample_time = 0.1  # seconds
   
    
    def __init__(self):
        super().__init__()
        self.__initMorors()
        self.__initDAQ()
        self._initVariables()


        self.use_camera = False
        self.marks_recorded = True
  
        

           
    
    def __del__(self):
        self._axis1.stop()
        self._axis2.stop()
        self._connection_z.close()
        self._conn_q.close_connection() #Connection to DAQ Qstation
        self.quit()
        
        
    def run(self):
        pass
    
    def stop(self):
        """
        Stops movemets, connections and  the execution of the current QThread
        """
        self._execute = False
        self._axis1.stop()
        self._axis2.stop()
        self._connection_z.close() #Connection to Zaber motors
        self._conn_q.close_connection() #Connection to DAQ Qstation
        self.quit()
    
    def stop_measurement(self):
        """
        Stop the measurement by setting the _execute flag to False and stopping axis1 and axis2.
        """
        self._execute = False
        self._axis1.stop()
        self._axis2.stop()
        #self._startForceLive()
        

        #what is it for?
    def setThread(self, thread):
        self.thread = thread
        
        
    def _initVariables(self):
        '''
        Function to initialize variables before each measurement

        Returns
        -------
        None.

        '''

        # Variables to store measuremetns
        self._ch1 = [] #channel1 from load cell
        self._ch2 = [] #channel2
        self._l1 = [] #length
        self._l2 = [] #length
        self._time = [] #time

        self._E11 = []
        self._E22 = []

        self._fl_marks = False #Flag indicating whether marks are recorded
        
        self._vel_1 = []
        self._vel_2 = []
        
        
        # Variable to stop measurement loop
        self._execute = True
        
        #Idle reading
        #self.idleReadForce()
        
        self._force1 = self._force2 = 0
        self._len_ax1 = self._len_ax2 = 0
        
        # Supplementary image to show tracks
        self.img_track = np.zeros((768, 1024, 1), dtype=np.uint8)
        self.img_track.fill(0)
        
        #record start time
        self._start_time = time.perf_counter()
        self._current_time = 0 
    
    def __initMorors(self):
        """
        motors initialization
        """

        ip_zaber = "172.31.100.108"
        
        try:
            self._connection_z = Connection.open_tcp(ip_zaber, Connection.TCP_PORT_CHAIN)
        except:
            raise Exception("Connection with motors failed at IP: " + ip_zaber + " Please check the network connection and ensure that the specified IP address is correct.")
            pass
        else:
            print("Motors were found")
            self._connection_z.enable_alerts()
            device_list = self._connection_z.detect_devices()
            print("Found {} devices".format(len(device_list)))
            device = device_list[0]
            
            
            #get motors 1&2 as private variables of the class
            self._axis1 = device.get_axis(1)
            self._axis2 = device.get_axis(2)
        
        mm_76 = 1595801
        
        #self._axis1.settings.set("limit.max", 5000000)  #76 mm
        #self._axis2.settings.set("limit.max", 5000000) 
        
    
    def __initDAQ(self):
        """
        Initialize the DAQ connection and retrieve information about the controller. 
        If the connection fails, a warning message is displayed. 
        The function also connects the controller buffer and variable, removes the first 
        values from the buffer, and reads load cell data before installing the sample. 
        """
        ip_daq = "172.31.100.107"#Controller IP

        #Initialisation of a buffer connection
        self._conn_q=Qstation.ConnectGIns()
        self._conn_q.bufferindex=0
        init_con_res = self._conn_q.init_connection(str(ip_daq))

        if not init_con_res:
            raise Exception("Connection with DAQ failed at IP: " + ip_daq + " Please check the network connection and ensure that the specified IP address is correct.")
    
        #Return some information of the controller
        self._conn_q.read_controller_name()
        self._conn_q.read_serial_number()
        self._conn_q.read_sample_rate()
        self._conn_q.read_channel_names()
        self._conn_q.read_channel_count()
        
        #Call function to connect Controller buffer and variable 
        self._buffer=self._conn_q.yield_buffer()
        #first data are usually broken. Just read them to 
        next(self._buffer) #remove first values
        time.sleep(0.5)
        
        #read load cell data before installing sample
        val1, val2 = self._readForce()
        print("Initial force at channel 1: {}".format(val1))
        print("Initial force at channel 2: {}".format(val2))
        
    
    def readForceLive(self):
        """
        Updates the live forces and emits a signal with the relative forces along two axes.
        """
        self._force1,self._force2 = self._readForce()
        if self._force1 is not None and self._force2 is not None:
            rel_force_ax1 = self._force1 - self._force1_0
            rel_force_ax2 = self._force2 - self._force2_0
            self.update_force_label_signal.emit(rel_force_ax1, rel_force_ax2)
    
    def _readForce(self):
        """
        Read force from buffer and process the value to return as newtons.
        """
        try:
            value = next(self._buffer)
      
            # Process the value
        
            value1 = round(value[-100:-1,1].mean(),5) #mean last 100 values of the buffer
            value2 = round(value[-100:-1,2].mean(), 5)
            #print('')
            #print("val1 {}".format(value1))
            #print("val2 {}".format(value1))
            
        except: # StopIteration:
            # Handle the case when there are no more items in the generator
            print("Empty buffer")
            return None, None
            #raise Exception('Empty buffer')
            
        return self.__convertToNewtons(value1, value2)
            
            
              
    def zeroForce(self):
        """
        Performs a zero force calibration by sleeping for 0.1 seconds, reading force values, and printing the initial force 1 and force 2 values.
        """
        time.sleep(0.1)
        self._force1_0, self._force2_0 = self._readForce()
        print("Init force 1: {}".format(self._force1_0))
        print("Init force 2: {}".format(self._force2_0))
        
        
    def zeroPosition(self):
        """
        record initial position of the motors
        """
        #record initial position of the motors
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        
        print("Init pos 1: {}".format(self._pos1_0))
        print("Init pos 2: {}".format(self._pos2_0))
        
      
    def __convertToNewtons(self, val1, val2):
        """
        Convert the given values to Newtons using the provided coefficients.

        Args:
            val1: The first value in mV/V to be converted.
            val2: The second value to be converted.

        Returns:
            Tuple containing the converted values for val1 and val2.
        """
        
        k1 = 250/0.7978 #coefficients according to calibration certificate
        k2 = 250/0.8317
        
        return k1*val1, k2*val2
    
            
    def moveSamplePosition(self):
        """
        Moves the sample position to either pos1_0 and pos2_0 if both are greater than 0, 
        or to a default position of 78mm if either pos1_0 or pos2_0 is not greater than 0.
        """
        samplePosition = 78 #mm
        
        #mm_76 = 1595801
        
        if self._pos1_0 >0 and self._pos2_0 > 0:
            self._axis1.move_absolute(self._pos1_0 , Units.LENGTH_MILLIMETRES, velocity=5, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)
            self._axis2.move_absolute(self._pos2_0, Units.LENGTH_MILLIMETRES, velocity=5, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)

        else:
            self._axis1.move_absolute(samplePosition , Units.LENGTH_MILLIMETRES, velocity=5, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)
            self._axis2.move_absolute(samplePosition, Units.LENGTH_MILLIMETRES, velocity=5, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)

    def initMotZeroPos(self):
        """
        Initialize the motors to move at a speed of -5 mm/sec until home position is reached.
        """
        speed = -5 # mm/sec
        
        #start moving the motors
        self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        
    def moveVelocityAx1(self, speed):
        """
        Move the axis 1 with the specified speed.

        :param speed: The speed at which to move the axis.
        :return: None
        """
        self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        
    def moveVelocityAx2(self, speed):
        self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND) 
        
    def _moving_average(self, x, w):
        return np.convolve(x, np.ones(w), 'valid') / w
    
    def changeFolder(self, folder):
        self._workfolder = folder

    def _writeDataToFile(self):
        # Combine the lists
        combined_lists = zip(self._time, self._ch1, self._ch2, self._l1, self._l2, self._E11, self._E22)
        
        #Get current date for filename
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")

        
        # Write to CSV file
        with open('Test_'+formatted_datetime+'.csv', 'w', newline='') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(["Time", "Load_1", "Load_2", "Disp_1", "Disp_2", "E11", "E22"])  # Header row, if needed
            for row in combined_lists:
                writer.writerow(row)

    def marksRecorded(self, lst):
        '''
        This function is connected with signal in video_thread which is emitted when marks are recorded
        '''
        self._fl_marks = True

        # Variables to store markers position
        self.marks_groups = []
        self.group1 = []
        self.group2 = []
        self.group3 = []
        self.group4 = []
        
        self.marks_groups.append(self.group1)
        self.marks_groups.append(self.group2)
        self.marks_groups.append(self.group3)
        self.marks_groups.append(self.group4)


        self.group1.append(lst[0][0])
        self.group2.append(lst[1][0])
        self.group3.append(lst[2][0])
        self.group4.append(lst[3][0])

        print("Marks recorded")
        print(self.marks_groups)

    def getMarksStatus(self):
        """
        Check if the marks are recorded. Required in the main window to decide whether to show strain or displacement
        """
        return self._fl_marks

         
    def _captureImageTrackMarks(self, camera):
        """
        Captures an image and tracks marks using the provided camera. Results are written to _E11, _E22 class attributes
        """
        with camera:
            frame = camera.get_frame ()
            if frame: 
                self._img_cv = frame.as_opencv_image()
                
                #detect markers
                img, coord_temp = markersDetection().detectMarkers(self._img_cv) #detection of Markers
                
                #draw track of the markers
                for group in self.marks_groups:
                    lg = len(group)
                    if lg > 1:
                        x1 = int(group[lg-2][0])
                        y1  = int(group[lg-2][1])
                        x2 = int(group[lg-1][0])
                        y2  = int(group[lg-1][1])
                        cv2.line(self.img_track, (x1,y1), (x2, y2), 200, 1)
                
                #distribute marks in the groups
                for gr in self.marks_groups:
                    if len(gr) > 0:
                        min_dist = 2000
                        el = None
                        #element - pair of (x,y) coordinates of a mark
                        for element in coord_temp: 
                            #last = gr[len(gr) - 1]
                            dist = math.dist(element, gr[-1])
                            #print(dist)
                            #print("dist {} and {} = {}".format(element, last, dist))
                            if dist < min_dist and dist < 70:
                                el = element
                                min_dist = dist
                        if el is not None:
                            gr.append(el)
                            #print(gr)

                        
                

                #Calculate strains
                if len(self.group1) > 1:

                    #calculate strain
                    #along horizontal axis - upper and lower groups]
                    #the algorithm find contours arranges points along y axis of an image
    
                    #Coordinates of initial and final points
                    p1_0 = self.group1[0]
                    p1 = self.group1[-1]
                    p2_0 = self.group2[0]
                    p2 = self.group2[-1]
                    p3_0 = self.group3[0]
                    p3 = self.group3[-1]
                    p4_0 = self.group4[0]
                    p4 = self.group4[-1]

                    L1_0 = (math.dist(p1_0, p2_0) + math.dist(p3_0,p4_0)) / 2
                    L1_last = (math.dist(p1, p2) + math.dist(p3, p4)) / 2

                    if (p1_0[1]-p3_0[1] < p1_0[1]-p4_0[1]):
                        L2_0 = (math.dist(p1_0, p3_0) + math.dist(p2_0,p4_0)) / 2
                        L2_last = (math.dist(p1, p3) + math.dist(p2, p4)) / 2
                    else:
                        L2_0 = (math.dist(p1_0, p4_0) + math.dist(p2_0, p3_0)) / 2
                        L2_last = (math.dist(p1, p4) + math.dist(p2, p3)) / 2
                    
                    
                    # Calculate strains
                    lambda1 = (L1_last - L1_0) / L1_0
                    lambda2 = (L2_last - L2_0) / L2_0

                    E11 = (lambda1*lambda1 - 1)/2
                    E22 = (lambda2*lambda2 - 1)/2

                    self._E11.append(E11)
                    self._E22.append(E22)

                    img = cv2.addWeighted(self.img_track, 0.2, img, 0.8, 0)

                    self.change_pixmap_signal.emit(img)




class DisplacementControlTest(MechanicalTest):
    
     
    def __init__(self, folder, val1, val2):
        super().__init__()
        
        # Set speed
        self._vel_ax1 = val1
        self._vel_ax2 = val2
        
        self._workfolder = folder

        #this timers should be created in class, but not in its parent class

    
    def __del__(self):
        self._execute = False
        
    def update_speed(self, val1, val2):
        
        # Set speed
        self._vel_ax1 = val1
        self._vel_ax2 = val2
        
        
    def run(self):
        """
        This function is implemented when the QTHread starts. Implements displacement control test.

        This function stops the liveforce timer, initializes the variables, starts tracking, 
        moves the motors with specified velocities, and performs a measurement cycle.

        """

        self._initVariables()
        
        #self.thread.startTracking()
        #self.start_stop_tracking_signal.emit(True)
        
        #start moving the motors
        self._axis1.move_velocity(self._vel_ax1, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        self._axis2.move_velocity(self._vel_ax2, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        
        '''
        if self.use_camera:
            #Vimba.get_instance() should always be in with-block
            #This is why camera is in this thread
            with Vimba.get_instance() as vimba:
                cameras = vimba.get_all_cameras()
                if not cameras:
                    raise RuntimeError('No Allied Vision Cameras found')
                    
                #while not exceed max extension or sample rupture
                while self._len_ax1 < 73 and self._len_ax2 < 73 and self._current_time < 400 and self._execute:
                    t = time.perf_counter()
                    self.__makeOneAnalogReading()
                    self.captureImageTrackMarks(cameras[0])
                    print("One cycle is {} sec".format(time.perf_counter()-t))
        
        else:
        
            #while not exceed max extension or sample rupture
            while self._len_ax1 < 73 and self._len_ax2 < 73 and self._current_time < 400 and self._execute:
                t = time.perf_counter()
                self.__makeOneAnalogReading()
                time.sleep(self._sample_time)
                print("One cycle is {} sec".format(time.perf_counter()-t))
        '''
         
        #while not exceed max extension or sample rupture
        while self._len_ax1 < 73 and self._len_ax2 < 73 and self._current_time < 400 and self._execute:
            #t = time.perf_counter()
            self.__oneCycle()
            time.sleep(self._sample_time)
            
        # Stop motors after measurement cycle is finished
        self.stop_measurement()
        #self.start_stop_tracking_signal.emit(False)
        self._writeDataToFile()
        
  
            
            
            
    def __oneCycle(self):
            """
            A function to perform one cycle of reading force values, calculating relative forces, 
            updating lengths, and recording time.
            """
            
            
            #read force values
            try:
                self._force1,self._force2 = self._readForce()
                
            except:
                if np.isnan(self._force1) or np.isnan(self._force2):
                    print('Exception while reading load cell data')
                    #continue
                
            else:
                print('')
                print("force1 {}".format(self._force1))
                print("force2 {}".format(self._force2))
            
            
            #Calculate relative forces
            rel_force_ax1 = self._force1 - self._force1_0
            rel_force_ax2 = self._force2 - self._force2_0
            
            #update length for each axis
            self._len_ax1 = 2*(self._pos1_0 - self._axis1.get_position(Units.LENGTH_MILLIMETRES))
            self._len_ax2 = 2*(self._pos2_0 - self._axis2.get_position(Units.LENGTH_MILLIMETRES))
            #record time
            self._current_time = time.perf_counter() - self._start_time
            
            
            #print("Value1: {}".format(self._force1))
            #print("Value2: {}".format(self._force2))
       
            
            roundDecimals = 5
            #Add them to current variables
            self._ch1.append(round(rel_force_ax1, roundDecimals)) #force channel 1
            self._ch2.append(round(rel_force_ax2, roundDecimals))
            self._l1.append(round(self._len_ax1, roundDecimals))
            self._l2.append(round(self._len_ax2, roundDecimals))
            self._time.append(round(self._current_time, roundDecimals))
         
            
            self.update_matplotlib_signal.emit(self._ch1, self._ch2, self._l1, self._l2, self._time,[],[],[],[])
            

class LoadControlTest(MechanicalTest):
    
    '''
    num_cycles:
    0 - one directional (half cycle)
    1 - one cycle
    etc
    '''

    def __init__(self, folder, end_force1, end_force2, test_duration, num_cycles):
        super().__init__()
        
        self._end_force1 = end_force1
        self._end_force2 = end_force2
        self._test_duration = test_duration
        
        self._workfolder = folder
        self._num_cycles = num_cycles

            # Initialize PID controllers
        Kp1 = 0.06
        Kd1 = 0
        Ki1 = 0.002
        self._pid_1 = PID(Kp1, Ki1, Kd1)
        Kp2 = 0.06
        Kd2 = 0
        Ki2 = 0.002
        self._pid_2 = PID(Kp2, Ki2, Kd2)


    
    def __del__(self):
        self._execute = False
        
    def update_parameters (self, end_force1, end_force2, test_duration, num_cycles):
        self._end_force1 = end_force1
        self._end_force2 = end_force2
        self._test_duration = test_duration
        self._num_cycles = num_cycles
    
    def increase_desired_force(self, start_force, end_force, duration, current_time):
        """
        Calculates desired force over the specified duration for specific current_time.
        Important function to control PID behaviour
    
        :param start_force: Initial force in Newtons
        :param end_force: Final force in Newtons
        :param duration: Duration of the test in seconds
        :param self._current_time: Current time in seconds since the start of the test
        :return: Desired force at the current time
        """
        if current_time >= duration:
            return end_force
        return start_force + (end_force - start_force) * (current_time / duration)
    

    def updatePID(self, P1, I1, D1, P2, I2, D2):
        """
        Update the PID values for two PID controllers when these values are changed in the GUI.
        """
        self._pid_1.setPID(P1, I1, D1)
        self._pid_2.setPID(P2, I2, D2)

       
    def run(self):
        """
        This function is implemented when the QTHread starts. Implements load control test.
        """
        

        #start performing test
        #this function is required to perform test with or without camera

        #I can get camera from Vimba only within with-block
        #This is why such a weird construction

        if self._fl_marks:
            self._sample_time = 0.05
            with Vimba.get_instance () as vimba:
                cams = vimba.get_all_cameras ()
                with cams [0] as cam:
                    
                    cam.Gain.set(10)
                    cam.ExposureTime.set(1500)

                    self.__performTest(cam) #True means to perform test with camera

        else:
            #without camera
            self._sample_time = 0.15
            self.__performTest(None)



    def __performTest(self, cam):

        self._initVariables()
        
        rel_force_ax1 = 0
        rel_force_ax2 = 0

        #initial LOW velocities for motors
        #to stretch a sample velocity should be negarive. It'll become negative before stretching cycle
        vel_ax1 = 0.02#mm/sec
        vel_ax2 = 0.02 #mm/sec


        #One directional test
        if 0 == self._num_cycles:
            self._start_time = time.perf_counter()

            vel_ax1 = - vel_ax1
            vel_ax2 = - vel_ax2

            # Start motors
            self._axis1.move_velocity(vel_ax1, Units.VELOCITY_MILLIMETRES_PER_SECOND)
            self._axis2.move_velocity(vel_ax2, Units.VELOCITY_MILLIMETRES_PER_SECOND)

            #First image will be recorded in camerawindow. Dont wont to capture it in a cycle now.
            '''
            if fl_cam:
                current_datetime = datetime.now()
                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                cv2.imwrite('Test_'+formatted_datetime+'_first_frame.jpg', self._img_cv) '''

            while self._execute and self._current_time < 1.5*self._test_duration and (rel_force_ax1 < self._end_force1 or rel_force_ax2 < self._end_force2):
                #Image capture should go first E11, E22 calculated in this funciton and after emitted in __oneCycle funciton
                if cam is not None:
                    self._captureImageTrackMarks(cam)
                rel_force_ax1, rel_force_ax2 = self.__oneCycle(self._start_time, 0, 0, self._end_force1, self._end_force2)
                
            #Last image frame
            if cam is not None:
                current_datetime = datetime.now()
                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                cv2.imwrite('Test_'+formatted_datetime+'_last_frame.jpg', self._img_cv)
                cv2.imwrite('Test_'+formatted_datetime+'_tracks.jpg', self.img_track)

        else:
            self._start_time = time.perf_counter()
            cycle = 0
            '''
            if fl_cam:
                current_datetime = datetime.now()
                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                cv2.imwrite('Test_'+formatted_datetime+'_first_frame.jpg', self._img_cv) '''

            #loop that counts cycles
            while self._execute and cycle < self._num_cycles:
                #Increase force loop (stretching loop)

                vel_ax1 = - vel_ax1
                vel_ax2 = - vel_ax2

                # Start motors
                self._axis1.move_velocity(vel_ax1, Units.VELOCITY_MILLIMETRES_PER_SECOND)
                self._axis2.move_velocity(vel_ax2, Units.VELOCITY_MILLIMETRES_PER_SECOND)
                
                #half cycle is to increase force, half cycle is to decrease force
                start_half_cycle_time = time.perf_counter()
                cycle += 1
                self._pid_1.reset()
                self._pid_2.reset()

                
                while self._execute and (rel_force_ax1 < self._end_force1 or rel_force_ax2 < self._end_force2):
                    if cam is not None:
                        self._captureImageTrackMarks(cam)
                    rel_force_ax1, rel_force_ax2 = self.__oneCycle(start_half_cycle_time, 0.03, 0.03, self._end_force1, self._end_force2)
                
                time.sleep(0.1)

                #Decreasing force loop
                print("Start decreasing force")
                #Velocity should be positive for back movement
                vel_ax1 = self._l1[-1]/self._test_duration
                vel_ax2 = self._l2[-1]/self._test_duration

                print("Vel1: {}".format(vel_ax1))
                print("Vel2: {}".format(vel_ax2))

                # Start motors
                self._axis1.move_velocity(vel_ax1*0.1, Units.VELOCITY_MILLIMETRES_PER_SECOND)
                self._axis2.move_velocity(vel_ax2*0.1, Units.VELOCITY_MILLIMETRES_PER_SECOND)

                #For the last frame we need to capture highest force
                if cam is not None:
                    current_datetime = datetime.now()
                    formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                    cv2.imwrite('Test_'+formatted_datetime+'_last_frame.jpg', self._img_cv)
                
                start_half_cycle_time = time.perf_counter()
                self._pid_1.reset()
                self._pid_2.reset()
                while self._execute and (rel_force_ax1 > 0.02 or rel_force_ax2 > 0.02):
                    if cam is not None:
                        self._captureImageTrackMarks(cam)
                    rel_force_ax1, rel_force_ax2 = self.__oneCycle(start_half_cycle_time, self._end_force1, self._end_force2, 0.03, 0.03)

                self._axis1.stop()
                self._axis2.stop()
                time.sleep(2)
                

            if cam is not None:
                cv2.imwrite('Test_'+formatted_datetime+'_tracks.jpg', self.img_track)

        
        #After test is finished (all the loops finished)
        # Stop motors after measurement cycle is finished
        self.stop_measurement()
        self._writeDataToFile()

    


    def __oneCycle(self, start_cycle_time, start_force1, start_force2, end_force1, end_force2):

        # Wait before the next loop iteration
        time.sleep(self._sample_time)

        self._current_time = time.perf_counter() - self._start_time
        current_cycle_time = time.perf_counter() - start_cycle_time
        
        # Update desired force based on test time
        desired_force1 = self.increase_desired_force(start_force1, end_force1, self._test_duration, current_cycle_time)
        desired_force2 = self.increase_desired_force(start_force2, end_force2, self._test_duration, current_cycle_time)
        
        self._pid_1.setpoint = desired_force1
        self._pid_2.setpoint = desired_force2

        # Read current forces, positions, time 
        self._force1,self._force2 = self._readForce() #try/except is inside
                
        #update length for each axis
        self._len_ax1 = 2*(self._pos1_0 - self._axis1.get_position(Units.LENGTH_MILLIMETRES))
        self._len_ax2 = 2*(self._pos2_0 - self._axis2.get_position(Units.LENGTH_MILLIMETRES))
        
        #Calculate relative forces
        rel_force_ax1 = self._force1 - self._force1_0
        rel_force_ax2 = self._force2 - self._force2_0
    
        
        #print("self._force12: {}".format(rel_force_ax1))
        #print("self._force2: {}".format(rel_force_ax2))
        #print("Desired Force: {}".format(desired_force1))
        #print("PID_1: {}".format(pid_1))
        #print("PID_2: {}".format(pid_2))
        
        roundDecimals = 5
        #Add them to current variables
        self._ch1.append(round(rel_force_ax1, roundDecimals)) #force channel 1
        self._ch2.append(round(rel_force_ax2, roundDecimals))
        self._l1.append(round(self._len_ax1, roundDecimals))
        self._l2.append(round(self._len_ax2, roundDecimals))
        self._time.append(round(self._current_time, roundDecimals))
        
        self.update_matplotlib_signal.emit(self._time, self._ch1, self._ch2, self._l1, self._l2, self._E11, self._E22, self._vel_1,self._vel_2)

        if len(self._time) > 10:
            corr_force1 = self._moving_average(self._ch1[-10:-1], 4)[-1]
            corr_force2 = self._moving_average(self._ch2[-10:-1], 4)[-1]
        else:
            corr_force1 = rel_force_ax1
            corr_force2 = rel_force_ax2
        
        vel_ax1 = -self._pid_1.updateOutput(corr_force1)
        vel_ax2 = -self._pid_2.updateOutput(corr_force2)
        
        self._vel_1.append(vel_ax1)
        self._vel_2.append(vel_ax2)
    
        # Apply motor output adjustments
        self._axis1.move_velocity(vel_ax1, Units.VELOCITY_MILLIMETRES_PER_SECOND)
            
        # Apply motor output adjustments
        self._axis2.move_velocity(vel_ax2, Units.VELOCITY_MILLIMETRES_PER_SECOND)
    
        
        
        #print("Cycle finished")

        return rel_force_ax1, rel_force_ax2
         
            

        
            
            
    