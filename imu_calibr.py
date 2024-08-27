import time 
import board 
import busio
from adafruit_lsm6ds import LSM6DSOX
import adafruit_lis3mdl
import numpy as np 
import ahrs
#from ahrs.filters import EKF



hard_calibr = [-3.35, -0.74, -40.79]
soft_calib = [0.96, 0.02, 0.01, 0.02, 0.96, 0.00, 0.01, 0.00, 1.08]
gyro_calib = [0.05, -0.01, -0.01]
mag_field_magnitude = 45.00

# Configura I2C1
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializza il primo set di sensori
lsm6dsox_1 = LSM6DSOX(i2c, address=0x6A)
lis3mdl_1 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1E)

# Inizializza il secondo set di sensori con indirizzo modificato
lsm6dsox_2 = LSM6DSOX(i2c, address=0x6B)
lis3mdl_2 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1C)

sample_period = 0.1 # Periodo di campionamento (in secondi)
beta = 1 # Fattore di correzione dell'errore (può essere regolato)

def read_sensors():
    data1 = []
    data2 = []
    # Leggi i dati dai sensori del primo set
    acc_1 = lsm6dsox_1.acceleration
    mag_1 = lis3mdl_1.magnetic
    gyro_1 = lsm6dsox_1. gyro
    # Leggi i dati dai sensori del secondo set
    acc_2 = lsm6dsox_2. acceleration
    mag_2 = lis3mdl_2. magnetic
    gyro_2 = lsm6dsox_2.gyro
    data1 = list(mag_1)+ list(gyro_1)+ list(acc_1)
    data2 = list(mag_2)+ list(gyro_2)+ list(acc_2)
    return data1, data2

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

def main():
    try:
        data1, data2 = read_sensors()
        data1 = correction(data1)
        data2 = correction(data2)

        '''        acc_data1 = [[data1[6],data1[7],data1[8]],[data1[6],data1[7],data1[8]],[data1[6],data1[7],data1[8]],[data1[6],data1[7],data1[8]]]
        gyr_data1 = [[data1[3],data1[4],data1[5]],[data1[3],data1[4],data1[5]],[data1[3],data1[4],data1[5]],[data1[3],data1[4],data1[5]]]
        mag_data1 = [[data1[0],data1[1],data1[2]],[data1[0],data1[1],data1[2]],[data1[0],data1[1],data1[2]],[data1[0],data1[1],data1[2]]]'''
        acc_data1 = [data1[6],data1[7],data1[8]]
        gyr_data1 = [data1[3],data1[4],data1[5]]
        mag_data1 = [data1[0],data1[1],data1[2]]
        q01 = ahrs.common.orientation.acc2q(np.array(acc_data1))
        acc_data2 = [data2[6],data2[7],data2[8]]
        gyr_data2 = [data2[3],data2[4],data2[5]]
        mag_data2 = [data2[0],data2[1],data2[2]]
        q02 = ahrs.common.orientation.acc2q(np.array(acc_data2))
        #get the first readings
        ekf1 = ahrs.filters.ekf.EKF(gyr=np.array(gyr_data1), acc=np.array(acc_data1), mag=np.array(mag_data1), frequency=10.0,frame = 'NED',q0 = q01)
        ekf2 = ahrs.filters.ekf.EKF(gyr=np.array(gyr_data2), acc=np.array(acc_data2), mag=np.array(mag_data2), frequency=10.0,frame = 'NED',q0 = q02)

        q0_1 = ekf1.Q/np.linalg.norm(ekf1.Q)
        q0_2 = ekf2.Q/np.linalg.norm(ekf2.Q)
        q0_1 = q0_1[0]/np.linalg.norm(q0_1[0])
        q0_2 = q0_2[0]/np.linalg.norm(q0_2[0])
        i = 0
        dt = 0.1 #time beteween two consecutive readings in seconds
        while True:
            data1, data2 = read_sensors()
            
            data1 = correction(data1)
            data2 = correction(data2)

            acc_data1 = [data1[6],data1[7],data1[8]]
            gyr_data1 = [data1[3],data1[4],data1[5]]
            mag_data1 = [data1[0],data1[1],data1[2]]

            acc_data2 = [data2[6],data2[7],data2[8]]
            gyr_data2 = [data2[3],data2[4],data2[5]]
            mag_data2 = [data2[0],data2[1],data2[2]]
            
            if i == 0:
                quat1 = ekf1.update(q0_1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                quat2 = ekf2.update(q0_2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)
                i +=1
            else:
                quat1 = ekf1.update(quat1,gyr=gyr_data1, acc=acc_data1, mag=mag_data1, dt=dt)
                quat2 = ekf2.update(quat2,gyr=gyr_data2, acc=acc_data2, mag=mag_data2, dt=dt)

            print("quat1: ", quat1)
            print("quat1: ", quat2)
            time.sleep (0.1)
    except KeyboardInterrupt:
        print("Programma terminato.")

if __name__ == "__main__":
    main()