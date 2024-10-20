import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep
import serial
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
import smbus
import struct

TEENSY_I2C_ADDRESS = 0x08
# Creazione dell'oggetto bus
bus = smbus.SMBus(1)


def set_position(time, a, c, T) :
    position = a*np.sin(2*np.pi/T*time) + c
    return position
def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)
def go_forward(time) :
    IDs = [1]
    a = 20
    c = 180
    T = 1
    a_dyna = a * 2048/180
    c_dyna = c * 2048/180
    q_dynamixel = set_position(time, a_dyna, c_dyna, T)
    write_position(q_dynamixel, IDs)
    return q_dynamixel
#Create Dynamixel object

servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0") #probably change it
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")"/dev/tty.usbserial-FT78LT9E"


def read_sensors():
    right = []
    left = []

    try:
        data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
        qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
        left = [qW1, qX1, qY1, qZ1]
        right = [qW2, qX2, qY2, qZ2]
        print(right)
    except OSError as e:
        print(f"Errore di comunicazione I2C: {e}")
        time.sleep(1)

    #choose data1 or data2 based on the tentacle to test    
    return left


#begin comunication with motor
servo.begin_communication()
#print("setting up motors... ")
servo.set_operating_mode("position", ID = "all")

#set motor position as 180 and fix it

write_position(2048, 4)

time.sleep(2)
#Read the data and set it as the base rotation matrix
tic = time.time()

while time.time()-tic < 2:
    quat_base = read_sensors()

quat_base_scalar_last = [quat_base[1], quat_base[2], quat_base[3], quat_base[0]]

R_base = np.array(R.from_quat(quat_base_scalar_last).as_matrix())
R_base = R_base/np.linalg.norm(R_base, 2)

print("got the first matrix, START MOVING ")

write_position(2560, 4)

t0 = time.time()

while time.time()-t0 < 5:
    variable = read_sensors()    


quat2_scalar_last = [variable[1], variable[2], variable[3], variable[0]]
new_matrix = np.array(R.from_quat(quat2_scalar_last).as_matrix())
norm_2 = np.linalg.norm(new_matrix, 2)
new_matrix = new_matrix / norm_2  
relative = np.dot(R_base.T,new_matrix)  

vec = R.from_matrix(relative).as_rotvec()

print(vec)
servo.end_communication()