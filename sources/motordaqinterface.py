from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal, pyqtSlot, QMetaObject
import numpy as np
from scipy.signal import butter, filtfilt
import collections
from statistics import mean
import time


from zaber_motion.ascii import Connection
from zaber_motion import Units
import ginsapy.giutility.connect.PyQStationConnectWin as Qstation #module with communication functions to Gantner Q.Station under windows environment

from .ringbuffer import RingBuffer
from .unit import Unit



class MotorDAQInterface (QThread):
    '''
    General class to communicate with DAQ and motors equipment
    
    '''
   
    _units = None
    signal_autoloading_finished = pyqtSignal()
    signal_autoloading_progress = pyqtSignal(float, float, float, float)
    
    def __init__(self, unit, ip_motor = "172.31.100.108", ip_daq = "172.31.100.200"):
        super().__init__()

        self._force1 = self._force2 = 0
        self._force1_0 = self._force2_0 = 0
        self._pos1_0 = self._pos2_0 = 0
          
        self._autoload_timer = QTimer(self)
        self._autoload_timer.timeout.connect(self._autoload_step)

        self._execute_autoloading = False
        self._mot_init = False
        self._daq_init = False

        self._ringbuffer1 = RingBuffer(20)
        self._ringbuffer2 = RingBuffer(20)

        # Buffer settings
        buffer_size = 30  # Define the size of the buffer
        self._buffer1 = collections.deque(maxlen=buffer_size)  # Create a buffer with a fixed size
        self._buffer2 = collections.deque(maxlen=buffer_size)

        self._units = unit
        print(f"_units: {self._units}")

        self._ip_motor = ip_motor
        self._ip_daq = ip_daq

        self.__initMorors()
        self.__initDAQ()

    def is_initialized(self):
        return self._mot_init and self._daq_init
    
    def change_units(self, unit):
        self._units = unit


    def run(self):
        # Start the event loop in this thread
        self.exec()
   
    
    def __initMorors(self):
        """
        motors initialization
        """

        
        try:
            self._connection_z = Connection.open_tcp(self._ip_motor, Connection.TCP_PORT_CHAIN)
        except:
            raise Exception("Connection with motors failed at IP: " + self._ip_motor + " Please check the network connection and ensure that the specified IP address is correct.")
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
        
        
    
    def __initDAQ(self):
        """
        Initialize the DAQ connection and retrieve information about the controller. 
        If the connection fails, a warning message is displayed. 
        The function also connects the controller buffer and variable, removes the first 
        values from the buffer, and reads load cell data before installing the sample. 
        """

        #Initialisation of a buffer connection
        self._conn_q=Qstation.ConnectGIns()
        self._conn_q.bufferindex=0
        init_con_res = self._conn_q.init_connection(str(self._ip_daq))

        if not init_con_res:
            raise Exception("Connection with DAQ failed at IP: " + self._ip_daq + " Please check the network connection and ensure that the specified IP address is correct.")
    
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
        QThread.msleep(200)
        
        #read load cell data before installing sample
        val1, val2 = self._read_force()
        print("Initial force at channel 1: {}".format(val1))
        print("Initial force at channel 2: {}".format(val2))
        
    
    @pyqtSlot()
    def stop(self):
        """
        Stops movemets, connections and  the execution of the current QThread
        """
        self._execute_autoloading = False
        QMetaObject.invokeMethod(self._autoload_timer, "stop", Qt.ConnectionType.QueuedConnection)
        self._axis1.stop()
        self._axis2.stop()
        self._connection_z.close() #Connection to Zaber motors
        self._conn_q.close_connection() #Connection to DAQ Qstation
        self.quit()
        self.wait()

    @pyqtSlot()
    def stop_test(self):
        """
        Stops movemets and qtimer
        """
        self._execute_autoloading = False
        QMetaObject.invokeMethod(self._autoload_timer, "stop", Qt.ConnectionType.QueuedConnection)
        self._axis1.stop()
        self._axis2.stop()
        self.quit()
        self.wait()
    
    def stop_motors(self):
        print("motors stopped")
        self._axis1.stop()
        self._axis2.stop()

    def stop_motor1(self):
        self._axis1.stop()

    def stop_motor2(self):
        self._axis2.stop()

        
    
    def _read_force(self):
        """
        Read force from buffer and process the value to return as newtons.
        """
        try:
            value = next(self._buffer)
      
            # Process the value
        
            value1 = round(value[:,1].mean(),5) # mean last 100 values of the buffer
            value2 = round(value[:,2].mean(), 5) # in mV/V
            
        except: # StopIteration:
            # Handle the case when there are no more items in the generator
            print("Empty buffer")
            return None, None
            #raise Exception('Empty buffer')
        else:
            self._buffer1.append(value1)
            self._buffer2.append(value2)
            
        return value1, value2
            
    def get_forces (self):
        force1, force2 = self._read_force()

        '''if len(self._buffer1) > 18:
            self._filtered_data1 = self.lowpass_filter(list(self._buffer1), 1, 5, 5)
            self._filtered_data2 = self.lowpass_filter(list(self._buffer2), 1, 5, 5)

            force1 = self._filtered_data1[-3]
            force2 = self._filtered_data2[-3]'''

        return self.__convert_mVV(force1 - self._force1_0, force2 - self._force2_0)
    
    def get_av_forces (self, n = 3):
        QThread.usleep(500)
        force1, force2 = self._read_force() #to take into account last value of a force

        force1 = mean(list(self._buffer1)[-n:])
        force2 = mean(list(self._buffer2)[-n:])

        '''if len(self._buffer1) > 18:
            self._filtered_data1 = self.lowpass_filter(list(self._buffer1), 1, 5, 5)
            self._filtered_data2 = self.lowpass_filter(list(self._buffer2), 1, 5, 5)

            force1 = self._filtered_data1[-3]
            force2 = self._filtered_data2[-3]'''

        return self.__convert_mVV(force1 - self._force1_0, force2 - self._force2_0)
    
    def get_positions(self):

        #update length for each axis
        len1 = self._axis1.get_position(Units.LENGTH_MILLIMETRES) - self._pos1_0
        len2 = self._axis2.get_position(Units.LENGTH_MILLIMETRES) - self._pos2_0

        return round(len1, 5), round(len2, 5)
    
    
              
    def zeroForce(self):
        """
        Performs a zero force calibration by sleeping for 0.1 seconds, reading force values, and printing the initial force 1 and force 2 values.
        """


        self._force1_0 = mean(list(self._buffer1)[-10:]) #in mV/V
        self._force2_0 = mean(list(self._buffer2)[-10:])


        force1_0, force2_0 = self.__convert_mVV(self._force1_0, self._force2_0)
        print("Init force 1: {}, force 2: {}".format(force1_0, force2_0))

        self._mot_init = True
        
        
    def zeroPosition(self):
        """
        record initial position of the motors
        """
        #record initial position of the motors
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        
        print("Init pos 1: {}, pos2: {}".format(self._pos1_0, self._pos2_0))

        self._daq_init = True

    def zero_pos1(self):
        """
        record initial position of the motor 1
        """
        #record initial position of the motors
        self._pos1_0 = self._axis1.get_position(Units.LENGTH_MILLIMETRES)
        
        print("Init pos 1: {}".format(self._pos1_0))

    def zero_pos2(self):
        """
        record initial position of the motor 1
        """
        #record initial position of the motors
        self._pos2_0 = self._axis2.get_position(Units.LENGTH_MILLIMETRES)
        
        print("Init pos 2: {}".format(self._pos2_0))

        
      
    def __convert_mVV(self, val1, val2):
        """
        convert mV/V into N or g according to the selected unit
        """

        val1 /=2 #correction for block system on a load cell that increaes force in 2 times
        val2 /=2

        if self._units == Unit.Newton:
            return self.__to_newtons(val1, val2)
        else:
            return self.__to_grams(val1, val2)
    
    def __to_newtons(self, val1, val2):
        """
        Convert the given values to Newtons using the provided coefficients.

        Args:
            val1: The first value in mV/V to be converted.
            val2: The second value to be converted.

        Returns:
            Tuple containing the converted values for val1 and val2.
        """

        z1 = -0.0006
        z2 = 0.0024
        
        k1 = 250/(0.7999 - z1) #coefficients according to calibration certificate
        k2 = 250/(0.8317 - z2)
        
        return round(k1*val1, 5), round(k2*val2, 5)
    
    def __to_grams(self, val1, val2):

        force1, force2 = self.__to_newtons(val1, val2)

        g = 9.81 #m/s2
        
        return round(force1*1000/g, 3), round(force2*1000/g, 3)


            
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
        #self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        #self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)

        self._axis1.home(wait_until_idle=False)
        self._axis2.home(wait_until_idle=False)

        # Set the soft limits (in millimeters)
        min_limit = 0    # Adjust this value as necessary
        max_limit = 83  # Adjust this value as necessary
        print(f"Setting movement limits: min {min_limit} mm, max {max_limit} mm")
        self._axis1.settings.set("limit.min", min_limit, unit = Units.LENGTH_MILLIMETRES)
        self._axis1.settings.set("limit.max", max_limit, unit = Units.LENGTH_MILLIMETRES)

        self._axis2.settings.set("limit.min", min_limit, unit = Units.LENGTH_MILLIMETRES)
        self._axis2.settings.set("limit.max", max_limit, unit = Units.LENGTH_MILLIMETRES)

    def move_position_ax1(self, pos, vel):
        self._axis1.move_absolute(self._pos1_0 + pos, Units.LENGTH_MILLIMETRES, velocity=vel, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)

    def move_position_ax2(self, pos, vel):
        self._axis2.move_absolute(self._pos2_0 + pos, Units.LENGTH_MILLIMETRES, velocity=vel, velocity_unit=Units.VELOCITY_MILLIMETRES_PER_SECOND, wait_until_idle=False)
        
    def move_velocity_ax1(self, speed):
        """
        Move the axis 1 with the specified speed.

        :param speed: The speed at which to move the axis.
        :return: None
        """
        self._axis1.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND)
        
    def move_velocity_ax2(self, speed):
        self._axis2.move_velocity(speed, Units.VELOCITY_MILLIMETRES_PER_SECOND) 

    def butter_lowpass(self, cutoff, fs, order=5):
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = filtfilt(b, a, data)
        return y

    def perform_autoload(self, target_load1, target_load2):
    
        self._execute_autoloading = True
        
        self._target_load1 = target_load1
        self._target_load2 = target_load2

        self._force1, self._force2 = self.get_forces()

        self._init_offset_1 = self._target_load1 - self._force1
        self._init_offset_2 = self._target_load2 - self._force2

        print(f"Starting autoloading. Offset1: {self._init_offset_1}, Offset2: {self._init_offset_2}")
        
        if self._units == Unit.Newton and self._init_offset_1 < 0.2 and self._init_offset_2 < 2 \
            or self._units == Unit.Gram and self._init_offset_1 < 20 and self._init_offset_1 < 20:

            self._start_time = round(time.perf_counter(), 5)
            self._current_time = 0
            self._autoload_timer.start(250)

        else:
            print("Autoload: The difference between current force and desired force is too high!")




    def _autoload_step(self):
        
        #Condition to finish the test: positive number of steps left
        if self._current_time < 30 and self._execute_autoloading:

            # Read current forces, positions, time and record them
            self._current_time = round(time.perf_counter() - self._start_time, 5)
            self._force1,self._force2 = self.get_av_forces(n = 3) #try/except is inside

            #caulculate the current offset
            offset1 = self._target_load1 - self._force1
            offset2 = self._target_load2 - self._force2

            threshold = 0.05 * self._target_load1

            # Check if autoloading is complete
            if abs(offset1) <= threshold and abs(offset2) <= threshold:
                self.stop_motors()
                QMetaObject.invokeMethod(self._autoload_timer, "stop", Qt.ConnectionType.QueuedConnection)
                self._execute_autoloading = False
                self.signal_autoloading_finished.emit()  # Notify that autoloading is finished
                return

            # Adjust motor velocities
            if self._units == Unit.Newton:
                Kp = 1.5
            else:
                Kp = 0.015

            max_velocity = 0.2 #mm/sec
            v1 = min(max_velocity, max(-max_velocity, -Kp * offset1))
            v2 = min(max_velocity, max(-max_velocity, -Kp * offset2))

            # Emit progress signal
            #self.signal_autoloading_progress.emit(self._force1, self._force2, v1, v2)

            print(f"force1: {self._force1}, force2: {self._force2}, vel1: {v1}, vel2: {v2}")

            # Command the motors
            self.move_velocity_ax1(v1)
            self.move_velocity_ax2(v2)

        else:
            self._execute_autoloading = False
            self.stop_motors()
            QMetaObject.invokeMethod(self._autoload_timer, "stop", Qt.ConnectionType.QueuedConnection)
        
            

         
            

        
            
            
    
