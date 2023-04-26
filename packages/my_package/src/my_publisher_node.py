#!/usr/bin/env python3
import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from smbus2 import SMBus
import time
import pid
speed = WheelsCmdStamped()
class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # Publisherid ja subscriberid
        self.pub = rospy.Publisher('/shelby/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.odometry = rospy.Subscriber('/shelby/odometry', String, self.odometry_list)

        #--------------------------VARIABLES----------------------------------------------------------------------------------

        self.integral = 0
        self.kp = 0.5
        self.ki = 0
        self.kd = 0
        self.set_speed = 0.358005
        self.last_error = 0
        self.delta_time = 1/20
        self.odometry_info = 0
        self.correction = 0
        self.error = 0
        self.prev_correction = 0

        self.right_values = [[3,4,5,6,7,8],
                            [4,5,6,7,8],
                            [5,6,7,8],
                            [6,7,8]]
            
        self.left_values = [[1,2,3,4,5,6],
                            [1,2,3,4,5],
                            [1,2,3,4],
                            [1,2,3]]
        
        self.short_line_values = [[1,2,4,5],
                                [1,3,4],
                                [1,4,5],
                                [1,4],
                                [1,5,6],
                                [1,2,4],
                                [2,3,5,6]]

        #---------------------------------------------------------------------------------------------------------------------

    def odometry_list(self, data):
        self.odometry_info = data.data
        
    def print_data_with_interval(self, correction):
        self.print_counter = self.print_counter + 1
        if self.print_counter == 15:
            print("---| P =", rospy.get_param("/p"),
                "|---| I =", rospy.get_param("/i"),
                "|---| D =", rospy.get_param("/d"),
                '|---| Speed =', rospy.get_param("/maxvel"),
                '|---| Correction =', round(correction, 3),
                "|---")
            self.print_counter = 0

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        self.bus.close()
        time.sleep(0.2)
        rospy.on_shutdown()
        
    def run(self):
        rate = rospy.Rate(20)
        self.bus = SMBus(12)

        #----------------------------------MAIN CODE--------------------------------------------------------------------------
        while not rospy.is_shutdown():     
            read = self.bus.read_byte_data(62,17)
            read = bin(read)[2:].zfill(8) #joonelugeri andmete lugemine binaaris nii, et väljastatav väärtus oleks alati 8-kohaline

            self.kp = float(rospy.get_param("/p")) 
            self.ki = float(rospy.get_param("/i"))
            self.kd = float(rospy.get_param("/d")) 

            if self.odometry_info == "odometry in progress":
                print(self.odometry_info)

            elif self.odometry_info == "odometry NOT in progress":
                sensor = [] #joonelugeri tuvastused vahemikus 1-8   
                for indx, nr in enumerate(read):
                    if nr == "1":
                        sensor.append(indx + 1)
                if len(sensor) > 0: 

                    #-------------------------SELECTING SHORTER ROUTE---------------------------------------------------------------
                    if sensor in self.short_line_values:
                        speed.vel_left = self.set_speed 
                        speed.vel_right = self.set_speed*0.5
                        self.pub.publish(speed)
                        time.sleep(0.75)
                        print("Short line")

                    #-------------------------90 DEG CORNER SOLUTION----------------------------------------------------------------

                    elif sensor in self.right_values:
                        self.error = 4.5 - 8
                        print("Right turn")
                    elif sensor in self.left_values:
                        self.error = 4.5 - 1
                        print("Left turn")
                    else:
                        self.error = 4.5 - np.average(sensor)

                else:
                    self.error = 0
                
                #-----------------------------------PID CONTROLLER------------------------------------------------------------------

                #correction, self.last_error = pid.PID.PID_controller(self.error, self.last_error, self.kp, self.ki, self.kd)
                self.integral = self.integral + (self.error + self.last_error)*self.delta_time/2
                self.integral = max(min(self.integral,2), -2) #integrali piirang
                derivative = (self.error - self.last_error)/self.delta_time

                kp_adjusted = self.kp
                ki_adjusted = self.ki
                kd_adjusted = self.kd

                # Calculate the PID correction factor
                correction = kp_adjusted * self.error + ki_adjusted * self.integral + kd_adjusted * derivative

                if correction > 1 or correction < -1:
                    correction = self.prev_correction
                else:
                    self.prev_correction = correction
                    
                speed.vel_left = self.set_speed - correction
                speed.vel_right = self.set_speed + correction

                #-----------------------------------CONTINUE DRIVING WHILE OFF THE LINE / MAX AND MIN SPEED--------------------------

                if len(sensor) == 0: 
                    speed.vel_left = self.previous_left
                    speed.vel_right = self.previous_right
                self.previous_left = speed.vel_left
                self.previous_right = speed.vel_right
                speed.vel_left = max(-0.1, min(speed.vel_left, 1)) 
                speed.vel_right = max(-0.1, min(speed.vel_right, 1)) 
                print(correction)
                self.pub.publish(speed)
                rate.sleep()          
            else:
                print("No odometry or PID in progress")
            rate.sleep()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin() #paneb koodi uuesti käima, loopi