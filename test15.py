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
