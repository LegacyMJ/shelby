#!/usr/bin/env python3
import os
import rospy
from smbus2 import SMBus
import numpy as np

class PID:
    def __init__(self):
        self.bus = SMBus(12)
        self.delta_time = 1/20
        self.integral = 0
    
    def run(self, last_error, prev_correction):
        while not rospy.is_shutdown():     
            read = self.bus.read_byte_data(62,17)
            read = bin(read)[2:].zfill(8) #joonelugeri andmete lugemine binaaris nii, et väljastatav väärtus oleks alati 8-kohaline

            self.kp = float(rospy.get_param("/p", 0)) 
            self.ki = float(rospy.get_param("/i", 0))
            self.kd = float(rospy.get_param("/d", 0)) 

            sensor = [] #joonelugeri tuvastused vahemikus 1-8   
            for indx, nr in enumerate(read):
                if nr == "1":
                    sensor.append(indx + 1)
            
            if len(sensor) > 0:
                error = 4.5 - np.average(sensor)
            else:
                error = 0

            self.integral = self.integral + (error + last_error)*self.delta_time/2
            self.integral = max(min(self.integral,2), -2) 
            derivative = (error - last_error)/self.delta_time
            last_error = error
            correction = self.kp * error + self.ki * self.integral + self.kd * derivative

            if correction > 1 or correction < -1:
                    correction = prev_correction
            else:
                prev_correction = correction

            return sensor, correction, prev_correction, last_error




'''
import tkinter as tk
import rospy

root = tk.Tk()
root.geometry('500x360')
root.resizable(False, False)
root.title('PID Controller')

slider_length = 360

#p_param = rospy.get_param('/p')
#print(p_param)

def p_upd(p_val):
    rospy.set_param('/p', p_val)
    print('P', p_val)
    
def i_upd(i_val):
    rospy.set_param('/i', i_val)
    print('I ', i_val)
    
def d_upd(d_val):
    rospy.set_param('/d', d_val)
    print('D ', d_val)
    
p = tk.Scale(root,
             from_ = 0.0,
             to = 0.5,
             orient = 'horizontal',
             resolution = 0.001,
             length = slider_length,
             command = p_upd)
p.grid(row = 1,
       column = 1,
       padx = 20,
       pady = 20,)

p_label = tk.Label(text= 'P')
p_label.grid(row = 1, padx = 5)

i = tk.Scale(root,
             from_ = 0.0,
             to = 0.1,
             orient = 'horizontal',
             resolution = 0.001,
             length = slider_length,
             command = i_upd)
i.grid(row = 2,
       column = 1,
       padx = 20,
       pady = 20)

i_label = tk.Label(text= 'I')
i_label.grid(row = 2, padx = 5)

d = tk.Scale(root,
             from_ = 0.0,
             to = 0.1,
             orient = 'horizontal',
             resolution = 0.0001,
             length = slider_length,
             command = d_upd)
d.grid(row = 3,
       column = 1,
       padx = 20,
       pady = 20)

d_label = tk.Label(text= 'D')
d_label.grid(row = 3, padx = 5)


root.mainloop() '''

#p,i,d = 0.045, 0.001, 0.015