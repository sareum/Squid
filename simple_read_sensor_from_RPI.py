import time
import numpy as np
import socket
import errno
from dynamixel_controller import Dynamixel
from time import sleep
import board 
import ahrs
import serial
import smbus
import struct
import math
import re


# Configurazione
PROTOCOL = 'TCP'
IP = '192.168.137.246' # Network number
PORT = 12345 # Same for client and server
BUFFER_SIZE = 1024  # Dimensione del buffer in byte
NUM_PACKETS = 10000  # Numero di pacchetti da inviare

TEENSY_I2C_ADDRESS  = 0x08
# Inizializza il bus I2C (bus 1 per Raspberry Pi 3 e successivi)
bus = smbus.SMBus(1)


def read_sensors():
    right = []
    left = []

    try:
        data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
        qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
        left = [qW1, qX1, qY1, qZ1]
        right = [qW2, qX2, qY2, qZ2]
    except OSError as e:
        print(f"Errore di comunicazione I2C: {e}")
        

    #choose data1 or data2 based on the tentacle to test    
    return right, left

while True:
    try:
        tic = time.time()
        quat1, quat2 = read_sensors()
        print("1: ",quat1)
        print("2: ",quat2)
        print("Time: ",time.time()-tic)
        time.sleep(0.001)
    except KeyboardInterrupt:
        print("Programma terminato.")