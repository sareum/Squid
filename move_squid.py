import time
import numpy as np
import keyboard
#from plot_module import show_graph
import matplotlib.pyplot as plt
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

# Turn on interactive mode for Matplotlib
plt.ion()  

start_time = time.time()
timer = []
t = 0

function_value =[]
read_position = []
read_velocity = []
error = []

while t < 5 :

    t = time.time() - start_time

    timer.append(t)

    function_value.append(set_position(t, 45, 180, 1))
    go_forward(t)
    read_position.append(180*servo.read_position(4)/2048)
    read_velocity.append(servo.read_velocity(4))
    error.append(function_value[-1] - read_position[-1])

    if keyboard.is_pressed('q'):
        print("You pressed the 'q' key.")
        break

# Position plot
plt.figure(1)
plt.xlabel('Time (s)')
plt.ylabel('Angular position (deg)')
plt.title('Angular position of the motor')
plt.plot(timer, function_value, 'b-', label='Function value')
plt.plot(timer, read_position, 'r-', markersize=10, label='Real angular position')
plt.legend()

# Show plots
plt.tight_layout()
plt.show()

servo.end_communication()

