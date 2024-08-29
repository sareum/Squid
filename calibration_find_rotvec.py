import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep
import serial
from scipy.spatial.transform import Rotation as R
import numpy as np
import board 
import busio
from adafruit_lsm6ds import LSM6DSOX
import adafruit_lis3mdl
import ahrs
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

############## I2C setup###################
#IMU calibration data:
hard_calibr = [-3.35, -0.74, -40.79]
soft_calib = [0.96, 0.02, 0.01, 0.02, 0.96, 0.00, 0.01, 0.00, 1.08]
gyro_calib = [0.05, -0.01, -0.01]
mag_field_magnitude = 45.00

# Configura I2C1
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializza il primo set di sensori
lsm6dsox_1 = LSM6DSOX(i2c, address=0x6A)
lis3mdl_1 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1C)


def read_sensors():
    data1 = []
    data2 = []
    
    try:
        # Leggi i dati dai sensori del primo set
        acc_1 = lsm6dsox_1.acceleration
        mag_1 = lis3mdl_1.magnetic
        gyro_1 = lsm6dsox_1.gyro
        
 

        # Combina i dati in una lista
        data1 = list(mag_1) + list(gyro_1) + list(acc_1)

    except OSError as e:
        if e.errno == 121:
            print("Errore di I/O remoto: Impossibile comunicare con uno o più sensori.")
            # Gestione dell'errore I/O remoto, restituisci valori di default o liste vuote
            data1 = [0] * 9  # Lista di zeri come default

        elif e.errno == 5:
            print("Errore di I/O: Problema di input/output durante la comunicazione con i sensori.")
            # Gestione dell'errore I/O, restituisci valori di default o liste vuote
            data1 = [0] * 9  # Lista di zeri come default
 
        else:
            # Rilancia l'eccezione se è un altro tipo di errore OSError
            raise
    except Exception as e:
        print(f"Errore imprevisto durante la lettura dei sensori: {e}")
        # Gestione di errori generici, restituisci valori di default o liste vuote
        data1 = [0] * 9  # Lista di zeri come default


    return data1

def correction(data):
    mx = data[0]-hard_calibr[0]
    my = data[1]-hard_calibr[1]
    mz = data[2]-hard_calibr[2]
    magX = mx * soft_calib[0] + my * soft_calib[1] + mz * soft_calib[2]
    magY= mx * soft_calib[3] + my * soft_calib[4] + mz * soft_calib[5]
    magZ = mx * soft_calib[6] + my * soft_calib[7] + mz * soft_calib[8]
    gyX = data[3] - gyro_calib[0]
    gyY = data[4] - gyro_calib[1]
    gyZ = data[5] - gyro_calib[2]
    return [magX, magY, magZ, gyX, gyY, gyZ, data[6], data[7], data[8]]

dt = 0.01# time intervall between two data
#begin comunication with motor
servo.begin_communication()
print("setting up motors... ")
servo.set_operating_mode("position", ID = "all")

calibration_data1 =[]
calibration_data2 = []
iData = 0

while iData<500:
    data1 =  read_sensors()
    #data1 = correction(data1)
    calibration_data1.append(data1)
    iData +=1
    time.sleep(0.01)

calibration_data1 = np.array(calibration_data1)
calibration_data2 = np.array(calibration_data2)
acc_data1 = (calibration_data1[:,6:9])
gyr_data1 = (calibration_data1[:,3:6])
mag_data1 = (calibration_data1[:,0:3])


#get the first readings
ekf1 = ahrs.filters.mahony.Mahony(gyr=gyr_data1, acc=acc_data1, mag=mag_data1, frequency=100.0)

q0_1 = ekf1.Q

q0_1 = q0_1[-1]/np.linalg.norm(q0_1[-1])

print("completed  EKF calibration")

#set motor position as 180
write_position(2048, [1])

time.sleep(2)
#Read the data and set it as the base rotation matrix
quat_base = []
iQ0 = 0
tic = time.time()

while time.time()-tic <10:
    data1 = read_sensors()
    data1 = correction(data1)
    
    acc_data1 = [data1[6],data1[7],data1[8]]
    gyr_data1 = [data1[3],data1[4],data1[5]]
    mag_data1 = [data1[0],data1[1],data1[2]]

    if iQ0 == 0:
        quat1 = ekf1.updateMARG(q0_1, gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
        iQ0 +=1
    else:
        quat1 = ekf1.updateMARG(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
    quat_base.append(list(quat1))
    time.sleep(0.01)
R_base = []
quat1_scalar_last =[]
for iElement in range(0, len(quat_base)):
    print(iElement)
    print(quat_base[iElement])
    print(quat_base[iElement][1])
    quat_base[iElement]  = quat_base[iElement]/np.linalg.norm(quat_base[iElement])
    quat1_scalar_last.append([quat_base[iElement][1], quat_base[iElement][2], quat_base[iElement][3], quat_base[iElement][0]]) #SCALAR LAST AS DEFAULT!!!!
    print(quat1_scalar_last)
    R_base.append(np.array(R.from_quat(quat1_scalar_last[iElement]).as_matrix()))
    norm_2 = np.linalg.norm(R_base[iElement], 2)
    R_base[iElement] = R_base[iElement] / norm_2
    
print("got the first matrix: ")
write_position(2560, 1)

time.sleep(2)

t0 = time.time()
variable = []

quat2 = quat1[-1]
while time.time()-t0 < 10:
    data1 = read_sensors()
    data1 = correction(data1)
    
    acc_data1 = [data1[6],data1[7],data1[8]]
    gyr_data1 = [data1[3],data1[4],data1[5]]
    mag_data1 = [data1[0],data1[1],data1[2]]

    quat2 = ekf1.updateMARG(quat2, gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
    variable.append(quat2)

    time.sleep(0.01)

new_matrix = []
realtive = []
rotation_vector = []
quat2_scalar_last = []
quat1 = quat1/np.linalg.norm(quat1)
for iElement in range(0, len(variable)):
    variable[iElement]  = variable[iElement]/np.linalg.norm(variable[iElement])
    quat2_scalar_last.append([variable[iElement][1], variable[iElement][2], variable[iElement][3], variable[iElement][0]]) #SCALAR LAST AS DEFAULT!!!!
    new_matrix.append( np.array(R.from_quat(quat2_scalar_last[iElement]).as_matrix()))
    norm_2 = np.linalg.norm(new_matrix[iElement], 2)
    new_matrix[iElement] = new_matrix[iElement] / norm_2    
    realtive.append( np.dot(R_base[iElement].T,new_matrix[iElement]))
    rotation_vector.append(R.from_matrix(realtive[iElement]).as_rotvec())
    rotation_vector[iElement] = rotation_vector[iElement]/np.linalg.norm(rotation_vector[iElement])


print("got the second matrix: ")


def appiattisci(lista):
    for elemento in lista:
        if isinstance(elemento[0], list):
            for sub_elemento in elemento:
                yield sub_elemento
        else:
            yield elemento

def scrivi_csv(dati, nome_file):
    with open(nome_file, mode='w', newline='') as file_csv:
        writer = csv.writer(file_csv)
        writer.writerows(appiattisci(dati))
scrivi_csv(R_base,"180.csv")
scrivi_csv(new_matrix,"non180.csv")
servo.end_communication()