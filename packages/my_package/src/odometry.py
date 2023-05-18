#!/usr/bin/env python3
import os
import numpy as np
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import Range
import time

speed = WheelsCmdStamped()
class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # Publisherid ja subscriberid
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.odom = rospy.Publisher('/shelby/odometry', String, queue_size=10)
        self.pub = rospy.Publisher('/shelby/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/shelby/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/shelby/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/shelby/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.right = 0
        self.left = 0
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.n_tot = 135
        self.range = 0
        self.wtravel = 0
        self.wtraveltmp = 0

    def callback(self, data):
        self.range = data.range
    def rightwheel(self, data):
        self.right = data.data
    def leftwheel(self, data):
        self.left = data.data

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.2)
        rospy.on_shutdown()

    def talker(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            R = 0.0318 
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right
            self.rotation_wheel_left = (2*np.pi/self.n_tot)*self.delta_ticks_left 
            self.rotation_wheel_right = (2*np.pi/self.n_tot)*self.delta_ticks_right 
            d_left = R * self.rotation_wheel_left 
            d_right = R * self.rotation_wheel_right 
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right
            self.wtravel = round(((d_left + d_right)*100)/2, 1) 
            self.range = round(self.range*100, 1) 

            if self.range <= 38: 
                self.odom.publish("odometry in progress")
                while self.range <= 38: 
                    speed.vel_left, speed.vel_right = 0.36, 0.05
                    self.pub.publish(speed)
                    self.range = round(self.range*100, 1)
                    print("odometry part 1")
                time.sleep(0.2)
                while self.wtraveltmp < 30: 
                    speed.vel_left, speed.vel_right = 0.3, 0.3
                    self.pub.publish(speed)
                    self.wtraveltmp = self.wtraveltmp + self.wtravel
                    print("odometry part 2")
                time.sleep(0.3)
                speed.vel_left, speed.vel_right = 0.05, 0.45 
                self.pub.publish(speed)
                time.sleep(1.5)
                speed.vel_left, speed.vel_right = 0.35, 0.05  
                print("odometry part 3")
                self.pub.publish(speed)
                #time.sleep(1.0)
                self.wtraveltmp = 0
                
            else:
                self.odom.publish("odometry NOT in progress")
            rate.sleep()


if __name__ == '__main__':
    node = MyPublisherNode(node_name="odometry")
    node.talker()
    rospy.spin()