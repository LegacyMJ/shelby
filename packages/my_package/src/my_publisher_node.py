#!/usr/bin/env python3
import os
import numpy as np
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
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
        self.set_speed = 0.35
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

    #------------------------------------------------------------------------------------------------------------------------

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
    
    def select_shorter_route(self, sensor, short_line_values):
        if sensor in short_line_values:
            speed.vel_left = self.set_speed 
            speed.vel_right = self.set_speed*0.5
            self.pub.publish(speed)
            time.sleep(0.75)
            print("Short line")

    def turn_90_deg(self, sensor, right_values, left_values):
        if sensor in right_values:
            self.error = 4.5 - 8
            self.set_speed = 0.15
            print("Right turn")
        elif sensor in left_values:
            self.error = 4.5 - 1
            self.set_speed = 0.15
            print("Left turn")
        else:
            self.error = 4.5 - np.average(sensor)

    def set_speed_and_speedlimit(self, correction, sensor):
        speed.vel_left = self.set_speed - correction
        speed.vel_right = self.set_speed + correction
        if len(sensor) == 0: 
            speed.vel_left = self.previous_left
            speed.vel_right = self.previous_right
        self.previous_left = speed.vel_left
        self.previous_right = speed.vel_right
        speed.vel_left = max(-0.0, min(speed.vel_left, 0.5)) 
        speed.vel_right = max(-0.0, min(speed.vel_right, 0.5))
        print(correction)
        self.pub.publish(speed)
        self.set_speed = 0.35

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        self.bus.close()
        time.sleep(0.2)
        rospy.on_shutdown()
        
    def run(self):
        rate = rospy.Rate(20)

        #----------------------------------MAIN CODE--------------------------------------------------------------------------
        while not rospy.is_shutdown():     

            if self.odometry_info == "odometry in progress":
                print(self.odometry_info)

            elif self.odometry_info == "odometry NOT in progress":

                #-----------------------------------SET SENSOR AND PID VALUES-------------------------------------------------------------
                sensor, correction, self.prev_correction, self.last_error = pid.PID().run(self.last_error, self.prev_correction)
                print(correction, sensor)

                #-----------------------------------SELECTING SHORTER ROUTE---------------------------------------------------------------
                self.select_shorter_route(sensor, self.short_line_values)

                #-----------------------------------90 DEG CORNER SOLUTION----------------------------------------------------------------
                self.turn_90_deg(sensor, self.right_values, self.left_values)

                #-----------------------------------CONTINUE DRIVING WHILE OFF THE LINE / MAX AND MIN SPEED-------------------------------
                self.set_speed_and_speedlimit(correction, sensor)
                rate.sleep()          
            else:
                print("No odometry or PID in progress")
            rate.sleep()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin() #paneb koodi uuesti käima, loopi