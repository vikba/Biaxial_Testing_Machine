import numpy as np

class PID:
    '''
    This PID class is used to implement a Proportional Integral Derivative (PID) controller.
    '''

    def __init__(self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.setpoint = 0
        self.integral = 0
        self.prev_error = 0

    def reset (self):
        self.setpoint = 0
        self.integral = 0
        self.prev_error = 0

    def setPID (self, P, I, D):
        self.Kp = P
        self.Ki = I
        self.Kd = D

    def setKp(self, P):
        self.Kp = P

    def updateOutput(self, measured_value):
        """
        Update the controller with a new measured value.

        Args:
            measured_value: The measured value from the system.

        Returns:
            The output of the controller after updating with the measured value.
        """
        error = self.setpoint - measured_value
        if not np.isnan(error):
            self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        return round(output,5)