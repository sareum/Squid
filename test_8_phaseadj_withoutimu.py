import threading
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
IP = '172.20.10.11'  # Network number
PORT = 12345  # Same for client and server
BUFFER_SIZE = 1024  # Buffer size in bytes
NUM_PACKETS = 10000  # Number of packets to send

# I2C Bus for Teensy communication
TEENSY_I2C_ADDRESS = 0x08
bus = smbus.SMBus(1)

# Global variables to track motor state and phase offset
was_closing = False
its_opening = False
phase_offset_left = 0  # Initial phase offset for the left motors
phase_increment = 0.25  # Increment phase offset by 0.5 when Enter is pressed
phase_max = 2  # Maximum phase offset

####################### MOTOR COMMAND ###################

def triangle_wave_position(t, a, T, rise_time_ratio, fall_time_ratio):
    global its_opening
    global was_closing
    period = T
    peak_value = 200  # Fixed value corresponding to closed tentacles
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
    # Motor IDs for right and left sides
    ID_right = [1, 4]
    ID_left = [2, 3]

    # Apply fixed phase offset to right and varying phase offset to left motors
    t_right = t  # No phase offset for the right motors
    t_left = t + phase_offset_left  # Varying phase offset for the left motors

    # Calculate positions for the right and left motors with the phase-offset time values
    q_dynamixel_right, t_mod_right = triangle_wave_position(t_right, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right)
    q_dynamixel_left, t_mod_left = triangle_wave_position(t_left, a_left, T_left, rise_time_ratio_left, fall_time_ratio_left)

    # Convert angles to motor positions (Dynamixel step units)
    position_motor_step_right = q_dynamixel_right * 2048 / 180  # Scaling to Dynamixel units
    position_motor_step_left = q_dynamixel_left * 2048 / 180

    # Send positions to the motors
    servo.write_position(position_motor_step_right, ID_right)
    servo.write_position(position_motor_step_left, ID_left)

    data = [q_dynamixel_right, q_dynamixel_left]
    return data, t_mod_right, t_mod_left

############### MOTOR SETUP ########################
servo = Dynamixel(ID=[1, 2, 3, 4], descriptive_device_name="XW430-T200R test motor", 
                  series_name=["xm", "xm", "xm", "xm"], baudrate=3000000, port_name="/dev/ttyUSB0") 

servo.begin_communication()
servo.set_operating_mode("position", ID="all")

# Triangular wave parameters
a_right = 80
a_left = 80                                                                                                                                                                                                                                                              
T_right = 1  # Period for right motors
T_left = 1   # Period for left motors
rise_time_ratio_right = 0.5  # Moving inward - thrust stroke
fall_time_ratio_right = 0.5  # Moving outward - return stroke
rise_time_ratio_left = 0.5
fall_time_ratio_left = 0.5

print("End motor setup")

############## END MOTOR SETUP #################



def oscillation_loop():
    ''' This function runs continuously to control motor oscillation '''
    global phase_offset_left

    t_start = time.time()

    while True:
        # Calculate time for motion control
        t = time.time() - t_start

        # Write motor positions in a triangular wave pattern
        data, t_mod_right, t_mod_left = write_motor_position_triangle(
            t, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right,
            a_left, T_left, rise_time_ratio_left, fall_time_ratio_left,
            phase_offset_left=phase_offset_left
        )
        print(f"Right Motor Position: {data[0]}, Left Motor Position: {data[1]} with Phase Offset: {phase_offset_left}")

        # Sleep for a short period to control the update rate
        sleep(0.000001)

def input_thread():
    ''' This function waits for user input to increment phase offset '''
    global phase_offset_left

    while True:
        user_input = input("Press Enter to increase phase offset or 'q' to quit: ")
        if user_input.strip().lower() == 'q':
            break  # Exit the loop if the user inputs 'q'
        else:
            phase_offset_left = (phase_offset_left + phase_increment) % (phase_max + phase_increment)
            print(f"Phase Offset incremented to: {phase_offset_left}")

'''Main loop'''
if __name__ == "__main__":
    try:
        # Start the oscillation loop in a separate thread
        oscillation_thread = threading.Thread(target=oscillation_loop)
        oscillation_thread.start()

        # Run the input thread in the main program
        input_thread()

    except KeyboardInterrupt:
        print("Program finished")
    finally:
        servo.end_communication()
