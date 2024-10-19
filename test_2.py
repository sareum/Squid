import time
from dynamixel_controller import Dynamixel
from time import sleep, time
import argparse
import numpy as np
import socket
import errno
import smbus
import struct
import re

# Configurazione
PROTOCOL = 'TCP'
IP = '172.20.10.11' # Network number
PORT = 12345 # Same for client and server
BUFFER_SIZE = 1024  # Dimensione del buffer in byte
NUM_PACKETS = 10000  # Numero di pacchetti da inviare

#I2C Bus for teensy communication
TEENSY_I2C_ADDRESS  = 0x08
bus = smbus.SMBus(1)


def velocity_security(velocity):
    # Velocity security
    if velocity > MAX_VELOCITY: 
        velocity = MAX_VELOCITY
    if velocity < -MAX_VELOCITY: 
        velocity = -MAX_VELOCITY
    return velocity

def wave_position(current_time, amplitude, period):
    # Calculate the position using a sine wave formula
    return amplitude * np.sin(2 * np.pi * (current_time / period))+500

'''Main loop'''
if __name__ == "__main__":
    UNIT_SCALE = 0.229  # [rev/min]
    MAX_VELOCITY = 314  # [rev/min]
    parser = argparse.ArgumentParser(description="give velocity in rpm")
    parser.add_argument("--vel", required=False, type=int)
    args = parser.parse_args()

    velocity = 140
    if args.vel is not None:
        velocity = args.vel    

    # Velocity security
    velocity = velocity_security(velocity)

    servo = Dynamixel(ID=1, descriptive_device_name="XW430-T200", 
                      series_name="xm", baudrate=3000000, port_name="/dev/ttyUSB0")
    servo.begin_communication()
    servo.set_operating_mode("position")

    # Initial position
    servo.write_position(200)
    sleep(1)

    # Wave parameters
    amplitude = 200  # Maximum position offset from the center
    period = 2     # Time for one complete wave cycle in seconds
    start_time = time()

    try:
        while True:
            current_time = time() - start_time
            position = wave_position(current_time, amplitude, period)

            # Write the calculated position to the servo
            servo.write_position(position)
            print(f"Current Position: {position:.2f}")

            # Sleep for a short period to control the update rate
            sleep(0.1)

    except KeyboardInterrupt:
        print("Program finished")
    finally:
        servo.end_communication()