import threading
import time
import numpy as np
from dynamixel_controller import Dynamixel
from time import sleep

# Configuration
PROTOCOL = 'TCP'
IP = '172.20.10.11'
PORT = 12345
BUFFER_SIZE = 1024
NUM_PACKETS = 10000

# Global variables
was_closing = False
its_opening = False
phase_offset_left = 0  
phase_increment = 0.5  
phase_max = 2  
stop_requested = False  

# Initial triangular wave parameters
a_right = 120
a_left = 120
T_right = 2  
T_left = 2   
x = 0.1  

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
    ID_right = [1, 3]
    ID_left = [2, 4]

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
servo = Dynamixel(ID=[1, 2, 3, 4], descriptive_device_name="XW430-T200R test motor", 
                  series_name=["xm", "xm", "xm", "xm"], baudrate=3000000, port_name="/dev/ttyUSB0") 

servo.begin_communication()
servo.set_operating_mode("position", ID="all")

rise_time_ratio_right = 1 - x  
fall_time_ratio_right = x  
rise_time_ratio_left = 1 - x
fall_time_ratio_left = x

print("End motor setup")

############## END MOTOR SETUP #################

def oscillation_loop():
    global phase_offset_left
    global rise_time_ratio_right, fall_time_ratio_right
    global rise_time_ratio_left, fall_time_ratio_left
    global stop_requested

    t_start = time.time()

    while True:
        t = time.time() - t_start
        data, t_mod_right, t_mod_left = write_motor_position_triangle(
            t, a_right, T_right, rise_time_ratio_right, fall_time_ratio_right,
            a_left, T_left, rise_time_ratio_left, fall_time_ratio_left,
            phase_offset_left=phase_offset_left
        )
        print(f"Right Motor Position: {data[0]}, Left Motor Position: {data[1]} with Phase Offset: {phase_offset_left}")

        # Check if stop was requested, and stop only at end of cycle
        if stop_requested and (t_mod_right < 0.01 or t_mod_left < 0.01):
            print("Stopping motor at end of cycle.")
            break  

        sleep(0.005) 

def input_x_value():
    global x
    global rise_time_ratio_right, fall_time_ratio_right
    global rise_time_ratio_left, fall_time_ratio_left

    while True:
        user_input = input("Enter the x value (0 to 1) to start: ")
        try:
            x = float(user_input)
            if 0 <= x <= 1:
                rise_time_ratio_right = 1 - x
                fall_time_ratio_right = x
                rise_time_ratio_left = 1 - x
                fall_time_ratio_left = x
                print(f"x value updated to {x}. Rise time and fall time ratios updated.")
                break  
            else:
                print("Invalid input. Please enter a value between 0 and 1.")
        except ValueError:
            print("Invalid input. Please enter a valid number between 0 and 1.")

def input_thread():
    global phase_offset_left
    global stop_requested

    while True:
        user_input = input("Press Enter to increase phase offset, 's' to stop, or 'q' to quit: ")
        if user_input.strip().lower() == 'q':
            break  
        elif user_input.strip().lower() == 's':
            stop_requested = True  
            break
        else:
            phase_offset_left = (phase_offset_left + phase_increment) % (phase_max + phase_increment)
            print(f"Phase Offset incremented to: {phase_offset_left}")

'''Main loop'''
if __name__ == "__main__":
    try:
        input_x_value()

        oscillation_thread = threading.Thread(target=oscillation_loop)
        oscillation_thread.start()

        input_thread()

        if stop_requested:
            oscillation_thread.join()

    except KeyboardInterrupt:
        print("Stop requested by keyboard interrupt; waiting for end of cycle.")
        stop_requested = True
        oscillation_thread.join()
    finally:
        servo.end_communication()
