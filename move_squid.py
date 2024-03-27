import time
import numpy as np
import cv2
#from plot_module import show_graph
from dynamixel_controller import Dynamixel
from time import sleep

def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)

def go_forward(time) :
    IDs = [1,2,3,4]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    print(q_dynamixel)
    write_position(q_dynamixel, IDs)

def go_reverse(time) :
    IDs = [1,2,3,4]
    a = 27
    c = 63
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)

def go_right(time) : 
    IDs = [3,4]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    write_position(2048, [1,2])

def go_left(time) : 
    IDs = [1,2]
    a = 45
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180 
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    write_position(2048, [3,4])
    
servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")

servo.begin_communication()

servo.set_operating_mode("position", ID = "all")
    
write_position(2048, [1,2,3,4]) #180°
sleep(1)

'''read_position = np.empty(1000)
read_velocity = np.empty(1000)
function_value = np.empty(1000)
error = np.empty(1000)'''

timer = time.time()

while True :

    t = time.time() - timer

    if t < 10 : 
        go_forward(t)

    '''function_value[i] = q   
    read_position[i] = servo.read_position(4)
    read_velocity[i] = servo.read_velocity(4)
    error[i] = 180*(function_value[i] - read_position[i])/position_center'''

    if t > 10 and t < 20 :
        go_reverse(t)

    if t > 20 :
        go_left(t)

    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    '''show_graph([read_position, function_value] , time, ["real values", "function values"]) 
    show_graph([error] , time, ["Error in degree"])
    show_graph([read_velocity] , time, ["Velocity"])'''

servo.end_communication()

