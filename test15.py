import time
import numpy as np
import socket
import errno
from dynamixel_controller import Dynamixel
from time import sleep
import smbus
import struct
import re

# Configuration
PROTOCOL = 'TCP'
IP = '172.20.10.11'
PORT = 12345
BUFFER_SIZE = 1024
NUM_PACKETS = 10000

# I2C Bus for Teensy communication
TEENSY_I2C_ADDRESS = 0x08
bus = smbus.SMBus(1)

# Global variables to track motor state
was_closing = False
its_opening = False

####################### MOTOR COMMAND ###################

def triangle_wave_position(t, a, T, rise_time_ratio, fall_time_ratio):
    global its_opening
    global was_closing
    period = T
    peak_value = 200
    valley_value = peak_value - a  
    rise_time = rise_time_ratio * period
    fall_time = fall_time_ratio * period

    t_mod = t % period  

    if t_mod < rise_time: 
        position = peak_value - (peak_value - valley_value) * (t_mod / rise_time)
        if was_closing:
            its_opening = True
            was_closing = False
    elif rise_time < t_mod < rise_time + fall_time: 
        position = valley_value + (peak_value - valley_value) * ((t_mod - rise_time) / fall_time)
        was_closing = True
    else:
        position = peak_value

    return position, t_mod

def write_motor_position_triangle(t, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right,
                                  a_left, T_left, rise_time_ratio_left, fall_time_ratio_left,
                                  phase_offset_left=0):
    # Motor IDs (Only motors 1 and 2)
    ID_right = [1]
    ID_left = [2]

    t_right = t
    t_left = t + phase_offset_left

    q_dynamixel_right, t_mod_right = triangle_wave_position(t_right, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right)
    q_dynamixel_left, t_mod_left = triangle_wave_position(t_left, a_left, T_left, rise_time_ratio_left, fall_time_ratio_left)

    position_motor_step_right = q_dynamixel_right * 2048 / 180
    position_motor_step_left = q_dynamixel_left * 2048 / 180

    servo.write_position(position_motor_step_right, ID_right)
    servo.write_position(position_motor_step_left, ID_left)

    data = [q_dynamixel_right, q_dynamixel_left]
    return data, t_mod_right, t_mod_left

############### MOTOR SETUP ########################
servo = Dynamixel(
    ID=[1, 2],
    descriptive_device_name="XW430-T200R test motor", 
    series_name=["xm", "xm"],
    baudrate=3000000,
    port_name="/dev/ttyUSB0"
)

servo.begin_communication()
servo.set_operating_mode("position", ID="all")

a_right = 120
a_left = 120
T_right = 2
T_left = 2
rise_time_ratio_right = 0.75
fall_time_ratio_right = 0.25
rise_time_ratio_left = 0.75
fall_time_ratio_left = 0.25

print("End motor setup")

############## END MOTOR SETUP #################

def read_sensors():
    right = []
    left = []

    try:
        data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
        print(data)
        qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
        left = [qW1, qX1, qY1, qZ1]
        right = [qW2, qX2, qY2, qZ2]
    except OSError as e:
        print(f"Error in I2C communication: {e}")

    return right, left

'''Main loop'''
if __name__ == "__main__":
    t_start = time.time()
    phase_offset_left = 0
    phase_increment = 0
    phase_max = 2
    num_cycles = 12
    cycle_time = T_right * num_cycles

    try:
        while True:
            t = time.time() - t_start

            if t >= cycle_time:
                print("Completed 12 cycles.")
                break

            if int(t) % 5 == 0:
                phase_offset_left = (phase_offset_left + phase_increment) % (phase_max + phase_increment)

            data, t_mod_right, t_mod_left = write_motor_position_triangle(
                t, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right,
                a_left, T_left, rise_time_ratio_left, fall_time_ratio_left,
                phase_offset_left=phase_offset_left
            )
            print(f"Right Motor Position: {data[0]}, Left Motor Position: {data[1]} with Phase Offset: {phase_offset_left}")

            sleep(0.0001)

    except KeyboardInterrupt:
        print("Program finished")
    finally:
        servo.end_communication()
