#!/usr/bin/env python3
import os
import numpy as np
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
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
        self.range = 1
        self.range_cm = 0

    def callback(self, data):
        self.range = data.range

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.2)
        rospy.on_shutdown()

    def talker(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.range_cm = round(self.range*100, 1) 
            if self.range_cm <= 38: 
                self.odom.publish("odometry in progress")
                while self.range_cm <= 38: 
                    speed.vel_left, speed.vel_right = 0.36, 0.05
                    self.pub.publish(speed)
                    self.range_cm = round(self.range*100, 1)
                    print("odometry part 1")
                time.sleep(0.2)
                speed.vel_left, speed.vel_right = 0.05, 0.45 
                self.pub.publish(speed)
                time.sleep(1.5)
                speed.vel_left, speed.vel_right = 0.35, 0.05  
                print("odometry part 3")
                self.pub.publish(speed)
                time.sleep(1.0)
                
            else:
                self.odom.publish("odometry NOT in progress")
            rate.sleep()


if __name__ == '__main__':
    node = MyPublisherNode(node_name="odometry")
    node.talker()
    rospy.spin()