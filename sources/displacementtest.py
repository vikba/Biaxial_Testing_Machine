# -*- coding: utf-8 -*-
"""

@author: Viktor B

"""

from PyQt6.QtCore import  Qt, QTimer, QMetaObject
import numpy as np
import time
from datetime import datetime
import os

from .mechanicaltest import MechanicalTest
from .direction import Direction


class DisplacementControlTest(MechanicalTest):
    
     
    def __init__(self, mot_daq, folder, sam_name, vel1, vel2, len1, len2, num_cycles):
        super().__init__(mot_daq)

        self._mot_daq = mot_daq
        
        # Set speed
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2

        self._max_disp1 = len1
        self._max_disp2 = len2
        
        self._workfolder = folder
        self._sam_name = sam_name

        self._num_cycles = num_cycles

        self._execute = True


        #this timers should be created in class, but not in its parent class
        
    def update_parameters(self, sam_name, vel1, vel2, len1, len2, num_cycles):

        self._sam_name = sam_name
        
        # Set speed
        self._vel_ax1 = vel1
        self._vel_ax2 = vel2

        self._max_disp1 = len1
        self._max_disp2 = len2

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
            img_addr = os.path.join(self._workfolder, self._sam_name,  'Test_'+self._sam_name+'_'+formatted_datetime+'_first_frame.jpg')
            self.signal_save_image.emit(img_addr)
        
        #Here should be a condition to start the test
        if True:

            current_datetime = datetime.now()
            formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
            self._file_name = 'DTest_'+ self._sam_name + '_' + formatted_datetime
            
            #Set number of cycles and direction
            self._half_cycle = 2*self._num_cycles #double for each half cycle
            self._direction = Direction.STRETCH

            self._start_time = time.perf_counter()
            self._start_cycle_time = time.perf_counter()
            self._current_time = 0

            #Set final positions
            self._pos1, self._pos2 = self._mot_daq.get_positions()

            self._start_pos1 = self._pos1
            self._start_pos2 = self._pos2
            self._fin_pos1 = self._start_pos1 - self._max_disp1/2 #when sample is stretched, the position is decreased
            self._fin_pos2 = self._start_pos2 - self._max_disp2/2

            #Start movement to final position
            self._mot_daq.move_position_ax1(self._fin_pos1, self._vel_ax1)
            self._mot_daq.move_position_ax2(self._fin_pos2, self._vel_ax2)

            #Start timer to periodically check length and control the test
            
            self._test_timer = QTimer()
            self._test_timer.timeout.connect(self._one_cycle)
            #self._test_timer.moveToThread(self)
            self._test_timer.start(self._sample_time)
            print("Timer started")
            self.exec()
            
        
        

    def _one_cycle(self):

        #Condition to finish the test: positive number of steps left
        if self._current_time < 1000 and self._execute and self._half_cycle > 0:

            #Update variables
            self._pos1, self._pos2 = self._mot_daq.get_positions()
            self._force1,self._force2 = self._mot_daq.get_forces()
            self._current_time = time.perf_counter() - self._start_time
            self._current_cycle_time = time.perf_counter() - self._start_cycle_time

            #If Stretch or Relax continue - send data 
            if Direction.STRETCH == self._direction and (self._pos1 >= self._fin_pos1+0.08 or self._pos2 >= self._fin_pos2+0.08) or \
                Direction.COMPRESS == self._direction and (self._pos1 < self._fin_pos1-0.08 or self._pos2 < self._fin_pos2-0.08):

                self._update_arrays_emit_data()

                #print(f"positions current: {self._pos1} , final: {self._fin_pos1}")
                #print(f"positions current: {self._pos2} , final: {self._fin_pos2}")
                
            #If Half cycle finished - change direction
            else:
                print("DisplacementControlTest: Half cycle finished")
                self._half_cycle -= 1
                if Direction.STRETCH == self._direction:
                    self._direction = Direction.COMPRESS
                else:
                    self._direction = Direction.STRETCH

                #Set final posistion based on stretch or relax cycle
                #Stretch cycle
                if Direction.STRETCH == self._direction:
                    self._fin_pos1 = self._start_pos1 - self._max_disp1/2 
                    self._fin_pos2 = self._start_pos2 - self._max_disp2/2

                    self._writeDataToFile(self._file_name, self._half_cycle/2 +1)
                    self._start_cycle_time = time.perf_counter()

                #Relax cycle
                elif Direction.COMPRESS == self._direction:
                    self._fin_pos1 = self._start_pos1
                    self._fin_pos2 = self._start_pos2

                if (self._use_video):
                    #Get current date for filename
                    current_datetime = datetime.now()
                    formatted_datetime = current_datetime.strftime("%Y_%m_%d_%H_%M")
                    img_addr = os.path.join(self._workfolder, self._sam_name,  'Test_'+self._sam_name+'_'+formatted_datetime+'_last_frame.jpg')
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
            self._finish_test()
            
        
        