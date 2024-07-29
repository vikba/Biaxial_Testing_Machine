# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot,  Qt, QTimer, QMetaObject
import numpy as np
import csv
import math
import time
from datetime import datetime

from .pid import PID


class MechanicalTest (QThread):
    '''
    General class for tensile tests that includes general methods for hardware
    '''

    #Signal to update matplotlib
    signal_update_charts = pyqtSignal(list)
    #Signal to start/stop tracking marks when test is run
    signal_start_stop_tracking = pyqtSignal(bool)
    signal_save_image = pyqtSignal(str)
    
    
    _sample_time = 200  #milli seconds
   
    
    def __init__(self, mot_daq):
        super().__init__()

        self._mot_daq = mot_daq

        self._use_video = False #Flag indicating whether marks are recorded
        
        self._init_variables() 
          
    
    #def __del__(self):
        #self.quit()
        
        
    def _init_variables(self):
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
        self._counter = 0 #counter

        self._E11 = []
        self._E22 = []
        
        self._vel_1 = []
        self._vel_2 = []
        
        
        # Variable to stop measurement loop
        self._execute = True
        
        #Idle reading
        #self.idleReadForce()
        
        self._force1 = self._force2 = 0
        self._pos1 = self._pos2 = 0
        
        #record start time
        self._start_time = time.perf_counter()
        self._current_time = 0 
    
        
    
    def run(self):
        #Generation of random signal to test the class
        #This method is redefined in subclassess
        while (self._counter < 100):
            self._sendRandSignal()
            QThread.msleep(100)
    
    @pyqtSlot()
    def stop(self):
        """
        Stops movemets, connections and  the execution of the current QThread
        """
        self._execute = False

        if hasattr(self, "_test_timer") and isinstance(self._test_timer, QTimer):
            self._test_timer.stop()

        self.quit()
        self.wait()
    
    def stop_measurement(self):
        """
        Stop the measurement by setting the _execute flag to False and stopping axis1 and axis2.
        """
        self._execute = False
        self.signal_start_stop_tracking.emit(False)
        self._mot_daq.stop_motors()
        
    
    
    @pyqtSlot(list)
    def init_markers(self, array):

        print("Markers initialized")
        self._use_video = True

        #datastructures to store tracks of marks
        self._marks_groups = []
        self._point1 = []
        self._point2 = []
        self._point3 = []
        self._point4 = []
        
        self._marks_groups.append(self._point1)
        self._marks_groups.append(self._point2)
        self._marks_groups.append(self._point3)
        self._marks_groups.append(self._point4)

        #Temporary coordinates 
        self._temp_p1 = array[0]
        self._temp_p2 = array[1]
        self._temp_p3 = array[2]
        self._temp_p4 = array[3]
    
    @pyqtSlot(list)
    def update_markers(self, array):

        #Temporary coordinates 
        self._temp_p1 = array[0]
        self._temp_p2 = array[1]
        self._temp_p3 = array[2]
        self._temp_p4 = array[3]

    def _update_arrays_emit_data(self):

        """
        Edge and node arrangement for central quad
                     edge 3
                3_______________4
                |               |
                |               |
        edge 2  |               | edge 4
                |               |
                |_______________|
                2    edge 1     1
        """

        #Append common variables for all tests
        
        roundDecimals = 5
        #Add them to current variables
        self._time.append(round(self._current_time, roundDecimals))
        self._ch1.append(round(self._force1, roundDecimals)) #force channel 1
        self._ch2.append(round(self._force2, roundDecimals))
        self._l1.append(round(-2*self._pos1, roundDecimals)) #Movement of sample is twice as holder
        self._l2.append(round(-2*self._pos2, roundDecimals))
        self._vel_1.append(self._vel_ax1)
        self._vel_2.append(self._vel_ax2)
        

        if not self._use_video:
            self.signal_update_charts.emit([self._time[-1], self._ch1[-1], self._ch2[-1], self._l1[-1], self._l2[-1]])
        else:
            #calculate strain
            #along horizontal axis - upper and lower groups]
            #the algorithm find contours arranges points along y axis of an image

            #Append points from a temporary buffer that is updated though signal/slot mechanism
            self._point1.append(self._temp_p1)
            self._point2.append(self._temp_p2)
            self._point3.append(self._temp_p3)
            self._point4.append(self._temp_p4)

            
            #Coordinates of initial and final points
            p1_0 = self._point1[0]
            p1 = self._point1[-1]
            p2_0 = self._point2[0]
            p2 = self._point2[-1]
            p3_0 = self._point3[0]
            p3 = self._point3[-1]
            p4_0 = self._point4[0]
            p4 = self._point4[-1]

            

            #decomposed initial coordinates for each node by axis
            X11 = p1_0[0]; X21 = p1_0[1]
            X12 = p2_0[0]; X22 = p2_0[1]
            X13 = p3_0[0]; X23 = p3_0[1]
            X14 = p4_0[0]; X24 = p4_0[1]

            # component X of displacement on isoparametric coordinates
            dX1_dS1 = 0.25*(X11-X12-X13+X14)
            dX1_dS2 = 0.25*(X11+X12-X13-X14)
            # component Y of displacement on isoparametric coordinates
            dX2_dS1 = 0.25*(X21-X22-X23+X24)
            dX2_dS2 = 0.25*(X21+X22-X23-X24)
            # jacobian of the initial with isoparametric coordinates
            J = dX1_dS1*dX2_dS2 - dX1_dS2*dX2_dS1

            #displacement for each node
            u1 = tuple(a - b for a, b in zip(p1, p1_0))
            u2 = tuple(a - b for a, b in zip(p2, p2_0))
            u3 = tuple(a - b for a, b in zip(p3, p3_0))
            u4 = tuple(a - b for a, b in zip(p4, p4_0))

            #decomposed displacement for each node by axis
            u11 = u1[0]; u21 = u1[1]
            u12 = u2[0]; u22 = u2[1]
            u13 = u3[0]; u23 = u3[1]
            u14 = u4[0]; u24 = u4[1]

            # component X of displacement on isoparametric coordinates
            du1_dS1 = 0.25*(u11-u12-u13+u14)
            du1_dS2 = 0.25*(u11+u12-u13-u14)
            # component Y of displacement on isoparametric coordinates
            du2_dS1 = 0.25*(u21-u22-u23+u24)
            du2_dS2 = 0.25*(u21+u22-u23-u24)

            # gradient of displacement u1 on spatial coordinates
            du1_dX1 = (dX2_dS2*du1_dS1 - dX2_dS1*du1_dS2)/J
            du1_dX2 = (-dX1_dS2*du1_dS1 + dX1_dS1*du1_dS2)/J
            # gradient of displacement u2 on spatial coordinates
            du2_dX1 = (dX2_dS2*du2_dS1 - dX2_dS1*du2_dS2)/J
            du2_dX2 = (-dX1_dS2*du2_dS1 + dX1_dS1*du2_dS2)/J

            # deformation gradient, F
            F11 = du1_dX1 + 1.0                # lambda1
            F12 = du1_dX2                      # kappa1
            F21 = du2_dX1                      # kappa2
            F22 = du2_dX2 + 1.0                # lambda2
            F33 = 1.0/(F11*F22 - F12*F21)      # lambda3

            E11 = 0.5*(F11**2 + F21**2 - 1.0)
            E22 = 0.5*(F22**2 + F12**2 - 1.0)
            E12 = 0.5*(F11*F12 + F22*F21)
            

            

            '''
            #Old formula for strain calculation
            L1_0 = (math.dist(p1_0, p2_0) + math.dist(p3_0,p4_0)) / 2
            L1_last = (math.dist(p1, p2) + math.dist(p3, p4)) / 2

            if (p1_0[1]-p3_0[1] < p1_0[1]-p4_0[1]):
                L2_0 = (math.dist(p1_0, p3_0) + math.dist(p2_0,p4_0)) / 2
                L2_last = (math.dist(p1, p3) + math.dist(p2, p4)) / 2
            else:
                L2_0 = (math.dist(p1_0, p4_0) + math.dist(p2_0, p3_0)) / 2
                L2_last = (math.dist(p1, p4) + math.dist(p2, p3)) / 2

            # Calculate strains
            lambda1 = 1 + (L1_last - L1_0) / L1_0
            lambda2 = 1 + (L2_last - L2_0) / L2_0

            E11 = (lambda1*lambda1 - 1)/2
            E22 = (lambda2*lambda2 - 1)/2'''

            

            self._E11.append(E11)
            self._E22.append(E22)

            self.signal_update_charts.emit([self._time[-1], self._ch1[-1], self._ch2[-1], self._l1[-1], self._l2[-1], self._E11[-1], self._E22[-1], self._vel_1[-1],self._vel_2[-1]])


    def _finish_test(self):
        
        self._use_video = False
        
        self._init_variables()     
    
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
        with open(self._workfolder + '\\Test_'+formatted_datetime+'.csv', 'w', newline='') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(["Time", "Load_1", "Load_2", "Disp_1", "Disp_2", "E11", "E22"])  # Header row, if needed
            for row in combined_lists:
                writer.writerow(row)

        # If videoextensometer is on and more than 10 marker positions were recorded
        # Write to CSV positions of markers
        if hasattr(self, '_point1'):
            combined_lists = zip(self._time, self._point1, self._point2, self._point3, self._point4)
            with open(self._workfolder + '\\Test_'+formatted_datetime+'_markers.csv', 'w', newline='') as file:
                writer = csv.writer(file, delimiter=';')
                writer.writerow(["Time","Marker_1", "Marker_2", "Marker_3", "Marker_4"])  # Header row, if needed
                for row in combined_lists:
                    writer.writerow(row)

            

    def _send_rand_signal(self):
       
        print("Sending random")
        self._counter +=1
        self._ch1 = np.random.rand(1)
        self._ch2 = np.random.rand(1) 
        self._l1 = np.random.rand(1) 
        self._l2 = np.random.rand(1) 
        self._E11 = np.random.rand(1) 
        self._E22 = np.random.rand(1) 
        self._vel_1 = np.random.rand(1) 
        self._vel_2 = np.random.rand(1) 
        self.signal_update_charts.emit([self._counter, self._ch1[-1], self._ch2[-1], self._l1[-1], self._l2[-1], self._E11[-1], self._E22[-1], self._vel_1[-1],self._vel_2[-1]])


class DisplacementControlTest(MechanicalTest):
    
     
    def __init__(self, mot_daq, folder, vel1, vel2, len1, len2, num_cycles):
        super().__init__(mot_daq)

        self._mot_daq = mot_daq
        
        # Set speed
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2

        self._disp1 = len1
        self._disp2 = len2
        
        self._workfolder = folder

        self._num_cycles = num_cycles

        #this timers should be created in class, but not in its parent class
        
    def update_parameters(self, vel1, vel2, len1, len2, num_cycles):
        
        # Set speed
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2

        self._disp1 = len1
        self._disp2 = len2

        self._num_cycles = num_cycles
        
        
    def run(self):
        """
        This function is implemented when the QTHread starts. Implements displacement control test.

        This function stops the liveforce timer, initializes the variables, starts tracking, 
        moves the motors with specified velocities, and performs a measurement cycle.

        """
        
        self._init_variables()

        if (self._use_video):
            print(f"Start of Displacement control test with video")
            self.signal_start_stop_tracking.emit(True)
            
            #Get current date for filename
            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            img_addr = self._workfolder + '\\Test_'+formatted_datetime+'_first_frame.jpg'
            self.signal_save_image.emit(img_addr)
        
        #Here should be a condition to start the test
        if True:

            #Set number of cycles and direction
            self._half_cycle = 2*self._num_cycles #double for each half cycle
            self._direction = 1 #Stretch sample; -1 relax
            self._start_time = time.perf_counter()

            #Set final positions
            self._pos1, self._pos2 = self._mot_daq.get_positions()

            self._start_pos1 = self._pos1
            self._start_pos2 = self._pos2
            self._fin_pos1 = self._start_pos1 - self._disp1/2 #when sample is stretched, the position is decreased
            self._fin_pos2 = self._start_pos2 - self._disp2/2

            #Start movement to final position
            self._mot_daq.move_position_ax1(self._fin_pos1, self._vel_ax1)
            self._mot_daq.move_position_ax2(self._fin_pos2, self._vel_ax2)

            #Start timer to periodically check length and control the test
            
            self._test_timer = QTimer(self)
            self._test_timer.timeout.connect(self._one_cycle)
            #self._test_timer.moveToThread(self)
            self._test_timer.start(200)
            print("Timer started")
            self.exec()
            
        
        

    def _one_cycle(self):

        #Condition to finish the test: positive number of steps left
        if self._current_time < 400 and self._execute and self._half_cycle > 0:

            #Update variables
            self._pos1, self._pos2 = self._mot_daq.get_positions()
            self._force1,self._force2 = self._mot_daq.get_forces()
            self._current_time = time.perf_counter() - self._start_time

            #If Stretch or Relax continue - send data 
            if 0 < self._direction and (self._pos1 >= self._fin_pos1+0.08 or self._pos2 >= self._fin_pos2+0.08) or 0 > self._direction and (self._pos1 < self._fin_pos1-0.08 or self._pos2 < self._fin_pos2-0.08):

                self._update_arrays_emit_data()

                #print(f"positions current: {self._pos1} , final: {self._fin_pos1}")
                #print(f"positions current: {self._pos2} , final: {self._fin_pos2}")
                
            #If Half cycle finished - change direction
            else:
                print("DisplacementControlTest: Half cycle finished")
                self._half_cycle -= 1
                self._direction =  -self._direction

                #Set final posistion based on stretch or relax cycle
                #Stretch cycle
                if 0 < self._direction:
                    self._fin_pos1 = self._start_pos1 - self._disp1/2 
                    self._fin_pos2 = self._start_pos2 - self._disp2/2

                #Relax cycle
                elif 0 > self._direction:
                    self._fin_pos1 = self._start_pos1
                    self._fin_pos2 = self._start_pos2

                if (self._use_video):
                    #Get current date for filename
                    current_datetime = datetime.now()
                    formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                    img_addr = self._workfolder + '\\Test_'+formatted_datetime+'_last_frame.jpg'
                    self.signal_save_image.emit(img_addr)
                
                if self._half_cycle > 0:
                    print(f"positions current: {self._pos1} , final: {self._fin_pos1}")
                    print(f"positions current: {self._pos2} , final: {self._fin_pos2}")
                    
                    #Start movement to final position
                    self._mot_daq.move_position_ax1(self._fin_pos1, self._vel_ax1)
                    self._mot_daq.move_position_ax2(self._fin_pos2, self._vel_ax2)
        
        #Test finished
        else:
            # Stop motors after measurement cycle is finished
            #self._test_timer.stop()
            QMetaObject.invokeMethod(self._test_timer, "stop", Qt.ConnectionType.QueuedConnection)
            print("DisplacementControlTest: Test finished")
            self.stop_measurement()
            self._writeDataToFile()
            self._finish_test()
            
        
        
            
            

class LoadControlTest(MechanicalTest):
    
    '''
    num_cycles:
    0 - one directional (half cycle)
    1 - one cycle
    etc
    '''

    def __init__(self, mot_daq, folder, max_force1, max_force2, test_duration, num_cycles):
        super().__init__(mot_daq)
        self._mot_daq = mot_daq
        
        self._max_force1 = max_force1
        self._max_force2 = max_force2
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

       
    def run2(self):
        """
        This function is implemented when the QTHread starts. Implements load control test.
        """
        

        #start performing test
        #this function is required to perform test with or without camera



        """
        #Get current date for filename
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        cv2.imwrite(self._workfolder + '\Test_'+formatted_datetime+'_first_frame.jpg', img_cv)

        self.__performTest(cam) #True means to perform test with camera
        """


        self._init_variables()

        #initial LOW velocities for motors
        #to stretch a sample velocity should be negarive. It'll become negative before stretching cycle
        vel_ax1 = 0.02#mm/sec
        vel_ax2 = 0.02 #mm/sec

        if (self._use_video):
            print(f"Use video")
            self.signal_start_stop_tracking.emit(True)
            
            #Get current date for filename
            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            img_addr = self._workfolder + '\\Test_'+formatted_datetime+'_first_frame.jpg'
            self.signal_save_image.emit(img_addr)


        #One directional test
        if 0 == self._num_cycles:
            self._start_time = time.perf_counter()

            vel_ax1 = - vel_ax1
            vel_ax2 = - vel_ax2

            # Start motors
            self._mot_daq.move_velocity_ax1(vel_ax1) #in mm/s
            self._mot_daq.move_velocity_ax2(vel_ax2) #in mm/s

            #First image will be recorded in camerawindow. Dont wont to capture it in a cycle now.
            '''
            if fl_cam:
                current_datetime = datetime.now()
                formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                cv2.imwrite('Test_'+formatted_datetime+'_first_frame.jpg', self._img_cv) '''

            while self._execute and self._current_time < 1.5*self._test_duration and (self._force1 < self._end_force1 or self._force2 < self._end_force2):
                #Image capture should go first E11, E22 calculated in this funciton and after emitted in __oneCycle funciton
                self._force1, self._force2 = self.__oneCycle(self._start_time, 0, 0, self._end_force1, self._end_force2)
                

        #Cyclic test
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
                # Start motors
                self._mot_daq.move_velocity_ax1(vel_ax1) #in mm/s
                self._mot_daq.move_velocity_ax2(vel_ax2) #in mm/s
                
                #half cycle is to increase force, half cycle is to decrease force
                start_half_cycle_time = time.perf_counter()
                cycle += 1
                self._pid_1.reset()
                self._pid_2.reset()

                
                while self._execute and (self._force1 < self._end_force1 or self._force2 < self._end_force2):
                    self._force1, self._force2 = self.__oneCycle(start_half_cycle_time, 0.03, 0.03, self._end_force1, self._end_force2)
                
                QThread.msleep(self._sample_time)

                #Decreasing force loop
                print("Start decreasing force")
                #Velocity should be positive for back movement
                vel_ax1 = self._l1[-1]/self._test_duration
                vel_ax2 = self._l2[-1]/self._test_duration

                print("Vel1: {}".format(vel_ax1))
                print("Vel2: {}".format(vel_ax2))

                # Start motors
                # Start motors
                self._mot_daq.move_velocity_ax1(vel_ax1*0.1) #in mm/s
                self._mot_daq.move_velocity_ax2(vel_ax2*0.1) #in mm/s

                start_half_cycle_time = time.perf_counter()
                self._pid_1.reset()
                self._pid_2.reset()
                while self._execute and (self._force1 > 0.02 or self._force2 > 0.02):
                    self._force1, self._force2 = self.__oneCycle(start_half_cycle_time, self._end_force1, self._end_force2, 0.03, 0.03)

                self._mot_daq.stop_motors()
                QThread.msleep(2000)
                

        
        #After test is finished (all the loops finished)
        # Stop motors after measurement cycle is finished
        self.stop_measurement()
        self._writeDataToFile()

    


    def __oneCycle(self, start_cycle_time, start_force1, start_force2, end_force1, end_force2):

        # Wait before the next loop iteration
        QThread.msleep(self._sample_time)

        self._current_time = time.perf_counter() - self._start_time
        current_cycle_time = time.perf_counter() - start_cycle_time
        
        # Update desired force based on test time
        desired_force1 = self.increase_desired_force(start_force1, end_force1, self._test_duration, current_cycle_time)
        desired_force2 = self.increase_desired_force(start_force2, end_force2, self._test_duration, current_cycle_time)
        
        self._pid_1.setpoint = desired_force1
        self._pid_2.setpoint = desired_force2

        # Read current forces, positions, time 
        self._force1,self._force2 = self._mot_daq.get_forces() #try/except is inside
        self._pos1, self._pos2 = self._mot_daq.get_positions()
    
        
        #print("self._force12: {}".format(self._force1))
        #print("self._force2: {}".format(self._force2))
        #print("Desired Force: {}".format(desired_force1))
        #print("PID_1: {}".format(pid_1))
        #print("PID_2: {}".format(pid_2))
        
        
        #Corr force is smoothed force for PID
        if len(self._time) > 10:
            corr_force1 = self._moving_average(self._ch1[-10:-1], 4)[-1]
            corr_force2 = self._moving_average(self._ch2[-10:-1], 4)[-1]
        else:
            corr_force1 = self._force1
            corr_force2 = self._force2
        
        self._vel_ax1 = -self._pid_1.updateOutput(corr_force1)
        self._vel_ax2 = -self._pid_2.updateOutput(corr_force2)
    
        # Apply motor output adjustments
        self._mot_daq.move_velocity_ax1(self._vel_ax1) #mm/s
        self._mot_daq.move_velocity_ax2(self._vel_ax2) #mm/s

        self._update_arrays_emit_data()
        #print("Cycle finished")

        return self._force1, self._force2
    



    def run(self):
        """
        This function is implemented when the QTHread starts. Implements load control test.
        """
        

        #start performing test
        #this function is required to perform test with or without camera

        self._init_variables()

        if (self._use_video):
            print(f"Use video {self._point1}")
            self.signal_start_stop_tracking.emit(True)
            
            #Get current date for filename
            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            img_addr = self._workfolder + '\\Test_'+formatted_datetime+'_first_frame.jpg'
            self.signal_save_image.emit(img_addr)


       
        if True:

            self._start_time = time.perf_counter()

            #Set number of cycles and direction
            self._half_cycle = 2*self._num_cycles #double for each half cycle
            self._direction = 1 #Stretch sample; -1 relax

            #Set final positions
            self._start_force1 = 0.03
            self._start_force2 = 0.03
            self._end_force1 = self._max_force1
            self._end_force2 = self._max_force2

            #Update currect forces
            self._force1,self._force2 = self._mot_daq.get_forces()
            

            #initial LOW velocities for motors
            #to stretch a sample velocity should be negarive. It'll become negative before stretching cycle
            vel_ax1 = - 0.02 #For slow initial stretch
            vel_ax2 = - 0.02

            # Start slow motion
            self._mot_daq.move_velocity_ax1(vel_ax1) #in mm/s
            self._mot_daq.move_velocity_ax2(vel_ax2) #in mm/s

            self._pid_1.reset()
            self._pid_2.reset()

            self._start_half_cycle_time = time.perf_counter()


            #Start timer to periodically check length and control the test
            self._test_timer = QTimer()
            self._test_timer.timeout.connect(self.__one_cycle)
            self._test_timer.start(self._sample_time)
            print("Timer started")
            self.exec()

                

    def __one_cycle(self):

        #Condition to finish the test: positive number of steps left
        if self._current_time < 400 and self._execute and self._half_cycle > 0:

            #If Stretch or Relax half cycle
            if self._direction > 0 and (self._force1 < self._end_force1 or self._force2 < self._end_force2) or self._direction < 0 and (self._force1 > self._end_force1 or self._force2 > self._end_force2):
        

                self._current_time = time.perf_counter() - self._start_time
                current_cycle_time = time.perf_counter() - self._start_half_cycle_time
                
                # Update desired force based on test time
                desired_force1 = self.increase_desired_force(self._start_force1, self._end_force1, self._test_duration, current_cycle_time)
                desired_force2 = self.increase_desired_force(self._start_force2, self._end_force2, self._test_duration, current_cycle_time)
                
                self._pid_1.setpoint = desired_force1
                self._pid_2.setpoint = desired_force2

                # Read current forces, positions, time 
                self._force1,self._force2 = self._mot_daq.get_forces() #try/except is inside
                self._pos1, self._pos2 = self._mot_daq.get_positions()

                #print("self._force12: {}".format(self._force1))
                #print("self._force2: {}".format(self._force2))
                #print("Desired Force: {}".format(desired_force1))
                #print("PID_1: {}".format(pid_1))
                #print("PID_2: {}".format(pid_2))
        
                #Corr force is smoothed force for PID
                if len(self._time) > 10:
                    corr_force1 = self._moving_average(self._ch1[-10:-1], 4)[-1]
                    corr_force2 = self._moving_average(self._ch2[-10:-1], 4)[-1]
                else:
                    corr_force1 = self._force1
                    corr_force2 = self._force2
                
                self._vel_ax1 = -self._pid_1.updateOutput(corr_force1)
                self._vel_ax2 = -self._pid_2.updateOutput(corr_force2)
    
                # Apply motor output adjustments
                self._mot_daq.move_velocity_ax1(self._vel_ax1) #mm/s
                self._mot_daq.move_velocity_ax2(self._vel_ax2) #mm/s

                self._update_arrays_emit_data()
                #print("Cycle finished")`

            #Half cycle finished
            else:
                self._half_cycle -= 1
                self._direction =  -self._direction #Change direction

                self._start_half_cycle_time = time.perf_counter()
                self._pid_1.reset()
                self._pid_2.reset()
                
                #Change behaviour to stretch loop
                if self._direction > 0:
                    #Increase force loop
                    print("LoadControlTest: Start increasing force")

                    self._start_force1 = 0.03
                    self._start_force2 = 0.03
                    self._end_force1 = self._max_force1
                    self._end_force2 = self._max_force2

                    vel_ax1 = -self._max_pos1/self._test_duration #Negative velocity for stretching
                    vel_ax2 = -self._max_pos2/self._test_duration

                    print("Vel1: {}".format(vel_ax1))
                    print("Vel2: {}".format(vel_ax2))

                    # Start motors
                    self._mot_daq.move_velocity_ax1(vel_ax1*0.2) #in mm/s
                    self._mot_daq.move_velocity_ax2(vel_ax2*0.2) #in mm/s

                    

                #Change behaviour to relax loop
                else:
                    #Decrease force loop
                    print("LoadControlTest: Start decreasing force")

                    self._start_force1 = self._max_force1
                    self._start_force2 = self._max_force2
                    self._end_force1 = 0.03
                    self._end_force2 = 0.03

                    #Max length is calculated at maximum stretch
                    self._max_pos1 = self._l1[-1]
                    self._max_pos2 = self._l2[-1]

                    vel_ax1 = self._max_pos1/self._test_duration
                    vel_ax2 = self._max_pos2/self._test_duration

                    print("Vel1: {}".format(vel_ax1))
                    print("Vel2: {}".format(vel_ax2))

                    # Start motors
                    self._mot_daq.move_velocity_ax1(vel_ax1*0.2) #in mm/s
                    self._mot_daq.move_velocity_ax2(vel_ax2*0.2) #in mm/s

                    QThread.msleep(1000)
        
        #When test finished
        else:
            # Stop motors after measurement cycle is finished
            self.stop_measurement()
            self._writeDataToFile()
            print("LoadControlTest: Test finished")
            QMetaObject.invokeMethod(self._test_timer, "stop", Qt.ConnectionType.QueuedConnection)

                    




    
         
            

        
            
            
    
