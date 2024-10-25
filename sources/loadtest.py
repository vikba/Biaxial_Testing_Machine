# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

from PyQt6.QtCore import QThread,  Qt, QTimer, QMetaObject, pyqtSlot
import numpy as np
import time
from datetime import datetime
import os


from .mechanicaltest import MechanicalTest
from .state import State
from .direction import Direction
from .unit import Unit



            
            

class LoadControlTest(MechanicalTest):
    
    '''
    num_cycles:
    0 - one directional (half cycle)
    1 - one cycle
    etc
    '''

    def __init__(self, mot_daq, folder, sam_name, autoloading, max_force1, max_force2, tare_load1, tare_load2, disp1, disp2, test_duration, num_cycles_precond, num_cycles_test):
        super().__init__(mot_daq)
        self._execute = False

        self._mot_daq = mot_daq

        self._mot_daq.signal_autoloading_finished.connect(self.handle_autoloading_finished)
        self._mot_daq.signal_autoloading_progress.connect(self.handle_autoloading_progress)
        
        self._max_force1 = max_force1
        self._max_force2 = max_force2
        self._test_duration = test_duration

        self._fl_autoloading = autoloading

        self._tare_load1 = tare_load1
        self._tare_load2 = tare_load2
        
        self._workfolder = folder
        self._sam_name = sam_name

        self._num_cycles_precond = num_cycles_precond
        self._num_cycles_test = num_cycles_test

        self._disp_guess1 = disp1
        self._disp_guess2 = disp2
    
    
    def update_parameters (self, folder, sam_name, autoloading, max_force1, max_force2, tare_load1, tare_load2, disp1, disp2, test_duration, num_cycles_precond, num_cycles_test):
        self._workfolder = folder
        self._sam_name = sam_name
        self._fl_autoloading = autoloading
        self._end_force1 = max_force1
        self._end_force2 = max_force2
        self._tare_load1 = tare_load1
        self._tare_load2 = tare_load2
        self._disp_guess1 = disp1
        self._disp_guess2 = disp2
        self._test_duration = test_duration
        self._num_cycles_precond = num_cycles_precond
        self._num_cycles_test = num_cycles_test
    
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
    
    @pyqtSlot()
    def handle_autoloading_finished(self):

        if self._execute:
            print("LoadControlTest: Autoloading finished, resuming test.")
            self._start_cycle_time = time.perf_counter()
            # Start stretching
            self._mot_daq.move_velocity_ax1(-self._vel_ax1) #in mm/s
            self._mot_daq.move_velocity_ax2(-self._vel_ax2) #in mm/s

            self._test_timer.start(self._sample_time)
        else:
            print("LoadControlTest: Autoloading finished, test not started yet")

    @pyqtSlot()
    def handle_autoloading_progress(self, force1, force2, vel1, vel2):

        print(f"LoadControlTest: Autoloading progress load1: {force1}, load2: {force2}, vel1:{vel1}, vel2:{vel2}")

        #self.signal_update_charts.emit([self._time_abs[-1], self._force1, self._force2, self._pos1, self._pos2])

        
    



    def run(self):
        """
        This function is implemented when the QTHread starts. Implements load control test.
        """
        

        #start performing test
        #this function is required to perform test with or without camera

        print("LoadControlTest: Started")
        self._execute = True

        self._init_variables()

        print(f"use video: {self._use_video}")

        #Get current date for filename
        current_datetime = datetime.now()
        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
        self._file_name = 'Precond_'+ self._sam_name + '_' +formatted_datetime

        if (self._use_video):
            print(f"Use video. Point1: {self._point1}")
            self.signal_start_stop_tracking.emit(True)
            
            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            img_addr = os.path.join(self._workfolder, self._sam_name, 'Test_'+self._sam_name+'_'+formatted_datetime+'_first_frame.jpg')
            self.signal_save_image.emit(img_addr)


       

        self._start_time = time.perf_counter()
        self._start_cycle_time = time.perf_counter()
        self._current_time = 0

        #Set number of cycles and direction
        self._half_cycle = 2*self._num_cycles_precond #double for each half cycle
        self._direction = Direction.STRETCH

        #Set final positions
        self._start_force1 = 0.0
        self._start_force2 = 0.0
        self._end_force1 = self._max_force1
        self._end_force2 = self._max_force2

        self._mot_daq.zeroPosition()
        
        self._force1,self._force2 = self._mot_daq.get_forces()
        
        self._vel_ax1 = self._disp_guess1/self._test_duration
        self._vel_ax2 = self._disp_guess2/self._test_duration

        # Start slow motion
        self._mot_daq.move_velocity_ax1(-self._vel_ax1) #in mm/s
        self._mot_daq.move_velocity_ax2(-self._vel_ax2) #in mm/s


        #Start timer to periodically check length and control the test
        self._test_timer = QTimer()
        self._test_timer.timeout.connect(self.__one_cycle_labview_alg)
        self._test_timer.start(self._sample_time)
        print("Timer started")
        self.exec()

            


    def __one_cycle_labview_alg(self):

        #Condition to finish the test: positive number of steps left
        if self._current_time < 1000 and self._execute and self._half_cycle > 0:

            # Read current forces, positions, time and record them
            self._current_time = round(time.perf_counter() - self._start_time, 5)
            self._current_cycle_time = round(time.perf_counter() - self._start_cycle_time, 5)
            self._force1,self._force2 = self._mot_daq.get_forces() #try/except is inside
            self._pos1, self._pos2 = self._mot_daq.get_positions()

            if len(self._load1) > 5:
                self._av_force1 = sum(self._load1[-5:])/5
            else:
                self._av_force1 = self._force1

            if len(self._load2) > 5:
                self._av_force2 = sum(self._load2[-5:])/5
            else:
                self._av_force2 = self._force2


            
            self._update_arrays_emit_data()

            #If Stretch or Relax half cycle
            if Direction.STRETCH == self._direction and self._av_force1 < self._end_force1 and self._av_force2 < self._end_force2 or \
                Direction.COMPRESS == self._direction and (self._pos1 <= -0.01  or self._pos2 <= -0.01):
                #(self._force1 > self._end_force1 or self._force2 > self._end_force2):
                
                
                #Stop at new 0 after very first cycle
                if self._state == State.PRECONDITIONING and self._half_cycle == 2*self._num_cycles_precond-1:
                    #If only one of the motor reached zero load - stop it
                    if Direction.COMPRESS == self._direction and self._av_force1 <= 0.05 :
                        self._mot_daq.stop_motor1()
                        self._mot_daq.zero_pos1()
                       
                    if Direction.COMPRESS == self._direction and self._av_force2 <= 0.05:
                        self._mot_daq.stop_motor2()
                        self._mot_daq.zero_pos2()
                    

            #Half cycle finished
            else:
                self._half_cycle -= 1
                
                if Direction.STRETCH == self._direction:
                    self._direction = Direction.COMPRESS
                else:
                    self._direction = Direction.STRETCH
                
                #Change behaviour from relax to stretch loop
                #Setting all the variables but the loop will start after autoloading if performed if needed
                if Direction.STRETCH == self._direction:
                    #Increase force loop
                    #print("LoadControlTest: Start increasing force")

                    self._start_force1 = 0
                    self._start_force2 = 0
                    self._end_force1 = self._max_force1
                    self._end_force2 = self._max_force2

                    self._vel_ax1 = round(self._disp_guess1/self._test_duration, 5)
                    self._vel_ax2 = round(self._disp_guess2/self._test_duration, 5)

                    self._mot_daq.stop_motors()

                    if self._state == State.PRECONDITIONING:
                        self._writeDataToFile(self._file_name, self._num_cycles_precond - self._half_cycle/2, False)
                    elif self._state == State.TEST:
                        fl_txt = (self._half_cycle == 0)
                        self._writeDataToFile(self._file_name, self._num_cycles_test - self._half_cycle/2, fl_txt )

                    self._init_variables()

                    #For autoloading we need to stop the current timer and start autoloading
                    if self._fl_autoloading:
                        QMetaObject.invokeMethod(self._test_timer, "stop", Qt.ConnectionType.QueuedConnection)
                        self._mot_daq.perform_autoload(self._tare_load1, self._tare_load2)
                    else:
                        self._start_cycle_time = time.perf_counter()
                        # Start stretching
                        self._mot_daq.move_velocity_ax1(-self._vel_ax1) #in mm/s
                        self._mot_daq.move_velocity_ax2(-self._vel_ax2) #in mm/s

                    

                #Stretch loop finished. Change behaviour to relax loop
                else:
                    #Decrease force loop
                    #print("LoadControlTest: Start decreasing force")

                    self._start_force1 = self._max_force1
                    self._start_force2 = self._max_force2
                    self._end_force1 = 0
                    self._end_force2 = 0

                    if self._use_video and 1 == self._half_cycle and self._state == State.TEST:
                        #Get current date for filename
                        current_datetime = datetime.now()
                        formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                        img_addr = os.path.join(self._workfolder, self._sam_name,  'Test_'+self._sam_name+'_'+formatted_datetime+'_last_frame.jpg')
                        self.signal_save_image.emit(img_addr)


                    #linear fit on the last 25% of the cycle
                    #correction calculatoin

                    n = int(0.25*self._current_cycle_time*1000/self._sample_time) #calculation of the length of the 25% of the cycle

                    #y = [x - self._start_time + self._start_cycle_time for x in self._time[-n:]] #Last 25% of the cycle
                    y = np.array(self._time_rel[-n:])
                    x1 = self._load1[-n:]
                    x2 = self._load2[-n:]

                    coef1 = np.polyfit(x1, y, 1)
                    coef2 = np.polyfit(x2, y, 1)

                    time_to_load1 = round(coef1[0]*self._max_force1 + coef1[1], 3)
                    time_to_load2 = round(coef2[0]*self._max_force2 + coef2[1], 3)

                    #corr_fact1 = round(((time_to_load1 - current_cycle_time)*0.8 + current_cycle_time) / current_cycle_time, 3)
                    #corr_fact2 = round(((time_to_load2 - current_cycle_time)*0.8 + current_cycle_time) / current_cycle_time, 3)

                    corr_fact1 = round(((time_to_load1 - self._test_duration)*0.8 + self._test_duration) / self._test_duration, 3)
                    corr_fact2 = round(((time_to_load2 - self._test_duration)*0.8 + self._test_duration) / self._test_duration, 3)

                    #print(f"time_to_load1: {time_to_load1}, time_to_load2: {time_to_load2}, current_cycle_time: {current_cycle_time}")
                    #print(f"Corr1: {corr_fact1}, Corr2: {corr_fact2}, Velocity1: {self._vel_ax1}, Velocity2: {self._vel_ax2}")
                    #print(f"Point1: {self._point1}")

                
                    corr_fact1 = min(corr_fact1, 1.5)
                    corr_fact2 = min(corr_fact2, 1.5)

                    self._disp_guess1 = self._disp_guess1 * corr_fact1
                    self._disp_guess2 = self._disp_guess2 * corr_fact2
                    

                    # Start motors
                    #self._mot_daq.move_velocity_ax1(self._vel_ax1) #in mm/s
                    #self._mot_daq.move_velocity_ax2(self._vel_ax2) #in mm/s
                    self._mot_daq.move_position_ax1(0, self._vel_ax1)
                    self._mot_daq.move_position_ax2(0, self._vel_ax2)
                

        #If cycles are over on PRECONDITIONING
        #Change state to TEST
        elif self._half_cycle <= 0 and self._state == State.PRECONDITIONING:
            self._start_time = time.perf_counter()
            self._start_cycle_time = time.perf_counter()
            self._state = State.TEST
            self._half_cycle = 2*self._num_cycles_test
            self._direction = Direction.STRETCH
            self._init_variables()
            #self._mot_daq.zeroPosition()

            print("LoadControlTest: Preconditioning finished. Main test started.")
            self.signal_precond_finished.emit()

            #Get current date for filename
            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            self._file_name = 'Test_'+ self._sam_name + '_' + formatted_datetime
        
        
        #When test finished
        else:
            # Stop motors after measurement cycle is finished
            QMetaObject.invokeMethod(self._test_timer, "stop", Qt.ConnectionType.QueuedConnection)
            self.stop_measurement()
            print("LoadControlTest: Test finished")
            self.signal_test_finished.emit()