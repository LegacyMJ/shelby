delta_time = 1/20

def PID_controller(kp, ki, kd, error):
    integral = integral + (error + last_error)*delta_time/2
    integral = max(min(integral,2), -2) 
    derivative = (error - last_error)/delta_time
    last_error = error

    correction = kp * error + ki* integral + kd * derivative

    if correction > 1 or correction < -1:
        correction = prev_correction
    else:
        prev_correction = correction




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
             to = 0.5,
             orient = 'horizontal',
             resolution = 0.001,
             length = slider_length,
             command = d_upd)
d.grid(row = 3,
       column = 1,
       padx = 20,
       pady = 20)

d_label = tk.Label(text= 'D')
d_label.grid(row = 3, padx = 5)


root.mainloop() '''
