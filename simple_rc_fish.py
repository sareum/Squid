from comms_wrapper import Arduino
from dynamixel_controller import Dynamixel
import time
import numpy as np
from utility import *
from copy import deepcopy as cp
from gamepad import Gamepad
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arduino_port", default="/dev/ttyUSB0", help="port of arduino")
    parser.add_argument("--dyn_port", default="/dev/ttyUSB1", help="port of dynamixel")
    args = parser.parse_args()

    offset = 0
    amp_increment = 0.2
    offset_increment = 0.1
    period_increment = 0.01

    # Connect gamepad
    gp = Gamepad()

    # Connect loadcell arduino
    arduino = Arduino( descriptiveDeviceName="Fishtail setup arduino", portName=args.arduino_port, baudrate=115200)
    arduino.connect_and_handshake()

    # Connect dynamixel
    dyn_id = 12
    dynamixel = Dynamixel(dyn_id, "Fishtail setup dynamixel", args.dyn_port, 1000000, "xl")
    dynamixel.begin_communication()
    dynamixel.set_operating_mode("position", dyn_id)

    # Sine wave properties
    amp_angle = 0
    period = 5
    temp_period = cp(period)

    robot_paused = True

    sequence = 1

    # Pump state
    pump_state = [0,0]

    timer = cp(time.time())
    print_timer = cp(time.time())
    while 1:
        t = time.time() - timer

        # Turn robot on and off
        if gp.button_data["circle"] == 1:
            dynamixel.write_profile_velocity(0)
            dynamixel.write_to_address(1, 1, 65, ID=dyn_id)
            timer = cp(time.time())
            robot_paused = False

        if gp.button_data["cross"] == 1:
            dynamixel.write_to_address(0, 1, 65, ID=dyn_id)
            robot_paused = True

        # Change sequence
        if gp.button_data["up"] == 1:
            sequence = 1
        elif gp.button_data["right"] == 1:
            sequence = 2
        elif gp.button_data["down"] == 1:
            sequence = 3
        elif gp.button_data["left"] == 1:
            sequence = 4

        # Change amplitude
        amp_joystick = gp.axis_data["Ry"]*-1
        if abs(amp_joystick) > 0.2:
            # Amp increase
            if amp_joystick > 0:
                if amp_angle < 20:
                    amp_angle += amp_joystick * amp_increment
            else:
                if amp_angle > 0:
                    amp_angle += amp_joystick * amp_increment

        if abs(amp_angle) < 0.05:
            true_amp = 0
        else:
            true_amp = amp_angle

        # Change offset
        offset_joystick = gp.axis_data["Lx"]
        if abs(offset_joystick) > 0.2:
            # Offset increase
            if offset_joystick > 0:
                if offset < 15:
                    offset += offset_joystick * offset_increment
            else:
                if offset > -15:
                    offset += offset_joystick * offset_increment

        period_joystick = ((gp.axis_data["Ltrigger"] + 1) - (gp.axis_data["Rtrigger"] + 1))/2
        if abs(period_joystick) > 0.2:
            # Period increase
            if period_joystick > 0:
                if temp_period < 5:
                    temp_period += period_joystick * period_increment
            else:
                if temp_period > 1.5:
                    temp_period += period_joystick * period_increment
        else:
            # if there is a change in period
            if abs(period - temp_period) > 0.1:
                if abs(np.sin( 2 * np.pi * t / period)- np.sin( 2 * np.pi * t / temp_period)) < 0.2:
                    print("Change period")
                    period = cp(temp_period)
            else:
                period = cp(temp_period)

        print_time = time.time() - print_timer
        if print_time > 0.1:
            print("\n\n==========================")
            print("seq:",sequence, "  amp:",round(true_amp,2),  "  offset:",round(offset,2), "  demand period:", round(temp_period,2), "  applied period:", round(period,2))
            print("==========================")
            print_timer = cp(time.time())
            
        period_timer = t%period

        if sequence == 1:
            # Sequence 1
            pump_state = [0,0]

        elif sequence == 2:
            # Sequence 2
            pump_state = [1,1]

        elif sequence == 3:
            # Sequence 3
            if period_timer < period/4:
                pump_state = [1,1]
            elif period_timer > period/2 and period_timer < period*3/4:
                pump_state = [1,1]
            else:
                pump_state = [0,0]

        elif sequence == 4:
            # Sequence 4
            if period_timer > period/4 and period_timer < period/2:
                pump_state = [1,1]
            elif period_timer > period*3/4:
                pump_state = [1,1]
            else:
                pump_state = [0,0]

        # Dynamixel stuff
        if robot_paused:
            arduino.send_message([0,0])
            dynamixel.write_profile_velocity(20)
            demand_angle = 0
            demand_angle_dynamixel_units = (demand_angle * 4096 / 360) + 2048 + 4096 * (offset/ 360)
            dynamixel.write_position(demand_angle_dynamixel_units, dyn_id)

        else:
            demand_angle = amp_angle * np.sin( 2 * np.pi * t / period) 
            demand_angle_dynamixel_units = (demand_angle * 4096 / 360) + 2048 + 4096 * (offset/ 360)
            dynamixel.write_position(demand_angle_dynamixel_units, dyn_id)

            # Send message to arduino
            arduino.send_message(pump_state)

        time.sleep(0.001)
if __name__ == "__main__":
    main()