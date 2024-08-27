import time 
import board 
import busio
from adafruit_lsm6ds import LSM6DSOX
import adafruit_lis3mdl
import numpy as np 
import quaternion 
import madgwickahrs

# Configura I2C1
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializza il primo set di sensori
lsm6dsox_1 = LSM6DSOX(i2c, address=0x6A)
lis3mdl_1 = adafruit_lis3mdl.LIS3MDL(i2c, address=0x1E)

# Inizializza il secondo set di sensori con indirizzo modificato
lsm6dsox_2 = LSM6DSOX(i2c, address=0x6B)
lis3mdl_2 = adafruit_lis3mdl. LISBMDL(i2c, address=0x1C)

sample_period = 0.1 # Periodo di campionamento (in secondi)
beta = 1 # Fattore di correzione dell'errore (può essere regolato)

ahrs = madgwickahrs(sampleperiod=sample_period, beta=beta)
def read_sensors():
    # Leggi i dati dai sensori del primo set
    acc_1 = lsm6dsox_1.acceleration
    mag_1 = lis3mdl_1.magnetic
    gyro_1 = lsm6dsox_1. gyro
    # Leggi i dati dai sensori del secondo set
    acc_2 = lsm6dsox_2. acceleration
    mag_2 = lis3mdl_2. magnetic
    gyro_2 = lsm6dsox_2.gyro
    # Stampa i risultati del primo set di sensori
    print(f"LSM6DSOX 1 Accelerometer: X={acc_1[0]: 2f} m/s^2, Y={acc_1[1]: 2f} m/s^2, Z={acc_1[2]: 2f} m/s^2")
    print(f"LISMDL 1 Magnetometer: X={mag_1[0]: 2f} microT, Y={mag_1[1]: 2f} microT, Z={mag_1[2]: 2f} microT")
    print(f"LSM6DSOX 1 Gyro: X={gyro_1[0]:.2f} m/s^2, Y={gyro_1[1]:.2f} m/s^2, Z={gyro_1[2]:.2f} m/s^2")
    # Stampa i risultati del secondo set di sensori
    print(f"LSM6DSOX 2 Accelerometer: X={acc_2[0]: 2f} m/s^2, Y={acc_2[1]:.2f} m/s^2, Z={acc_2[2]:.2f} m/s^2")
    print(f"LISMDL 2 Magnetometer: X={mag_2[0]: 2f} microT, Y={mag_2[1]: 2f} T, Z={mag_2[2]: 2f} microT")
    print(f"LSM6DSOX 2 Gyro: X={gyro_2[0]:.2f} m/s^2, Y={gyro_2[1]: 2f} m/s^2, Z={gyro_2[2]:.2f} m/s^2")

def main():
    try:
        while True:
            acc_1 = lsm6dsox_1.acceleration
            mag_1 = lis3mdl_1.magnetic
            gyro_1 = lsm6dsox_1.gyro
            ahrs.update(gyro_1, acc_1, mag_1)
            quaternion = ahrs. quaternion
            print("Quaternione:", quaternion)
            time.sleep (0.1)
    except KeyboardInterrupt:
        print("Programma terminato.")

if __name__ == "__main__":
    main()