#!/usr/bin/env python3
import os
import rospy
from smbus2 import SMBus
import numpy as np

class PID:
    def __init__(self):
        self.bus = SMBus(12)
        self.delta_time = 1/20  # Time interval for PID control
        self.integral = 0  # Integral term for PID control
    
    def run(self, last_error, prev_correction):
        while not rospy.is_shutdown():
            # Read sensor data from the bus
            read = self.bus.read_byte_data(62, 17)
            read = bin(read)[2:].zfill(8)  # Convert the data to binary representation

            self.kp = float(rospy.get_param("/p", 0.05))  # Get the proportional gain from ROS parameter server
            self.ki = float(rospy.get_param("/i", 0.001))  # Get the integral gain from ROS parameter server
            self.kd = float(rospy.get_param("/d", 0.015))  # Get the derivative gain from ROS parameter server

            sensor = []  # List to store activated sensor indices
            for indx, nr in enumerate(read):
                if nr == "1":
                    sensor.append(indx + 1)

            if len(sensor) > 0:
                error = 4.5 - np.average(sensor)  # Calculate the average error
            else:
                error = 0

            self.integral = self.integral + (error + last_error) * self.delta_time / 2  # Update the integral term
            self.integral = max(min(self.integral, 2), -2)  # Limit the integral term within a range
            derivative = (error - last_error) / self.delta_time  # Calculate the derivative term
            last_error = error

            # Calculate the correction using the PID formula
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            if correction > 1 or correction < -1:
                correction = prev_correction  # Limit the correction within a range
            else:
                prev_correction = correction

            return sensor, correction, prev_correction, last_error
