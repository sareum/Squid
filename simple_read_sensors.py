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
dt = 0.0115
#IMU calibration data:
hard_calibr = [-3.35, -0.74, -40.79]
soft_calib = [0.96, 0.02, 0.01, 0.02, 0.96, 0.00, 0.01, 0.00, 1.08]
gyro_calib = [0.05, -0.01, -0.01]
mag_field_magnitude = 45.00
calibration_data1 = []
iData  = 0
while iData <30:
    data1 =  read_sensors()
    #data1 = correction(data1)
    calibration_data1.append(data1)
    iData +=1
    print(iData)

calibration_data1 = np.array(calibration_data1)
#calibration_data2 = np.array(calibration_data2)
acc_data1 = (calibration_data1[:, 6:9])
gyr_data1 = (calibration_data1[:, 3:6])
mag_data1 = (calibration_data1[:, 0:3])


#get the first readings
ekf1 = ahrs.filters.mahony.Mahony(gyr=gyr_data1, acc=acc_data1, mag=mag_data1, frequency=87)
q0_1 = ekf1.Q
q0_1 = q0_1[-1]/np.linalg.norm(q0_1[-1])
iQ0 = 0
while True:
    tic = time.time()
    data = read_sensors()
    data = correction(data)
    acc_data1 = [data1[6],data1[7],data1[8]]
    gyr_data1 = [data1[3],data1[4],data1[5]]
    mag_data1 = [data1[0],data1[1],data1[2]]
    
    if iQ0 == 0:
        quat1 = ekf1.updateMARG(q0_1, gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
        iQ0 +=1
    else:
        quat1 = ekf1.updateMARG(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
    print(quat1)
    print("elapsed: ", time.time()-tic)