import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep
import serial
from scipy.spatial.transform import Rotation as R
import numpy as np
import csv
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

#create a comunication for the teensy:
ser = serial.Serial(
    port='/dev/ttyACM0',        # Sostituisci con la tua porta seriale, ad esempio '/dev/ttyUSB0' su Linux
    baudrate=115200,      # Imposta il baudrate
    timeout=1           # Timeout in secondi per la lettura
)
if ser.is_open:
    print("Connected to the teensy-setup!")
#begin comunication with motor
servo.begin_communication()
servo.set_operating_mode("position", ID = "all")
#set motor position as 180
write_position(2048, [1])
time.sleep(5)
#Read the data and set it as the base rotation matrix
quat_base = []
while sum(quat_base) == 0:
    quat_base = (((ser.readline()).decode('utf-8')).strip()).split(',')
    quat_base = vector = [float(value) for value in quat_base]
    time.sleep(0.01)
print(quat_base)
quat_base = [quat_base[1], quat_base[2], quat_base[3], quat_base[0]]

R_base = np.array(R.from_quat(quat_base).as_matrix())
norm_2 = np.linalg.norm(R_base, 2)
R_base = R_base / norm_2
#print("got the first matrix: ")
print(R_base)
write_position(2560,1)
t0 = time.time()
variable = []
while time.time()-t0 < 5:
    variable = (((ser.readline()).decode('utf-8')).strip()).split(',')
    variable = vector = [float(value) for value in variable]
    #print(variable)
    time.sleep(0.01)
print(variable)
variable = [variable[1], variable[2], variable[3], variable[0]]

new_matrix = np.array(R.from_quat(variable).as_matrix())
norm_2 = np.linalg.norm(new_matrix, 2)
new_matrix = new_matrix / norm_2
print("got the second matrix: ")
print(new_matrix)
realtive = np.dot(R_base.T,new_matrix)
print(realtive)
rotation_vector = R.from_matrix(realtive).as_rotvec()
rotation_vector = rotation_vector/np.linalg.norm(rotation_vector)
print("rotation_Vector: ")
print(rotation_vector)
servo.end_communication()