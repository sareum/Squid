import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep

servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")

servo.begin_communication()

servo.set_operating_mode("position", ID = "all")

def set_position(time, a, T) : 
    position = a*np.sin(2*np.pi/T*time)
    return position

position_max = 2560 #225°
position_min = 1536 #135°
position_center = 2048 #180°

motor_rpm = 53 #rpm
time_per_increment = 60/motor_rpm/(2*position_center) #Seconds per position increment

servo.write_position(2048, ID="all") #180°
sleep(1)

i = 0
num_cycles = 10
T = 1 #Period
a = 45 #Amplitude en degré
c = position_center

Straight = 1
Turn_right = 0
Turn_left = 0

#read_position = np.empty(1000)
#read_velocity = np.empty(1000)
#function_value = np.empty(1000)
#error = np.empty(1000)

timer = time.time()

while Straight :

    t = time.time() - timer

    q = int(set_position(t,a,T))
    q_dynamixel = (q * 4096 / 360) + c #Angle in dynamixel units
    servo.write_position(q_dynamixel, ID="all")

    #function_value[i] = q   
    #read_position[i] = servo.read_position(4)
    #read_velocity[i] = servo.read_velocity(4)
    #error[i] = 180*(function_value[i] - read_position[i])/position_center

    #i+=1

    if t > T*num_cycles : 
        break

#show_graph([read_position, function_value] , time, ["real values", "function values"]) 
#show_graph([error] , time, ["Error in degree"])
#show_graph([read_velocity] , time, ["Velocity"])

servo.end_communication()

