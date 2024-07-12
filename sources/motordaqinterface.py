from PyQt6.QtCore import QThread


from zaber_motion.ascii import Connection
from zaber_motion import Units
import ginsapy.giutility.connect.PyQStationConnectWin as Qstation #module with communication functions to Gantner Q.Station under windows environment



class MotorDAQInterface (QThread):
    '''
    General class to communicate with DAQ and motors equipment
    '''

    
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
          
    
    def __del__(self):
        self._axis1.stop()
        self._axis2.stop()
        self._connection_z.close()
        self._conn_q.close_connection() #Connection to DAQ Qstation
        self.quit()
        
        
    def _initVariables(self):
        '''
        Function to initialize variables before each measurement

        Returns
        -------
        None.

        '''
        
        self._force1 = self._force2 = 0
        self._len_ax1 = self._len_ax2 = 0
        
    
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
        QThread.sleep(0.5)
        
        #read load cell data before installing sample
        val1, val2 = self._readForce()
        print("Initial force at channel 1: {}".format(val1))
        print("Initial force at channel 2: {}".format(val2))
        
    
    def run(self):
        #Generation of random signal to test the class
        #This method is redefined in subclassess
        while (self._counter < 100):
            self._sendRandSignal()
            QThread.sleep(0.1)
    
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
        self.signal_start_stop_tracking.emit(False)
        #self._startForceLive()
        
    
    def readForceLive(self):
        """
        Called by QTimer in BiaxMainWindow.
        Updates the live forces and emits a signal with the relative forces along two axes.
        """
        self._force1,self._force2 = self._readForce()
        if self._force1 is not None and self._force2 is not None:
            self._rel_force_ax1 = self._force1 - self._force1_0
            self._rel_force_ax2 = self._force2 - self._force2_0
            self.signal_update_force_label.emit(self._rel_force_ax1, self._rel_force_ax2)
    
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
        QThread.sleep(0.1)
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
        
    