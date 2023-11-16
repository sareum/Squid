from comms_wrapper import Arduino
from dynamixel_controller import Dynamixel
import time
import numpy as np
import os
from utility import *
from copy import deepcopy as cp
from gamepad import Gamepad
import argparse
import os
import shutil

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arduino_port", default="/dev/ttyUSB0", help="port of arduino")
    parser.add_argument("--dyn_port", default="/dev/ttyUSB1", help="port of dynamixel")
    args = parser.parse_args()

    offset = 7

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
    amp_angle = 20
    period = 3
    num_cycles = 10
    #num_cycles = 10 / period

    # Pump state
    pump_state = [0,0]

    timer = time.time()
    while 1:
        # Do something with gamepad
        if gp.button_data["circle"] == 1:
            dynamixel.write_to_address(1, 1, 65, ID=dyn_id)
        else:
            dynamixel.write_to_address(0, 1, 65, ID=dyn_id)

        # t = time.time() - timer
        # period_timer = t%period

        # if args.seq == 1:
        #     # Sequence 1
        #     pump_state = [0,0]

        # elif args.seq == 2:
        #     # Sequence 2
        #     pump_state = [1,1]

        # elif args.seq == 3:
        #     # Sequence 3
        #     if period_timer < period/4:
        #         pump_state = [1,1]
        #     elif period_timer > period/2 and period_timer < period*3/4:
        #         pump_state = [1,1]
        #     else:
        #         pump_state = [0,0]

        # elif args.seq == 4:
        #     # Sequence 4
        #     if period_timer > period/4 and period_timer < period/2:
        #         pump_state = [1,1]
        #     elif period_timer > period*3/4:
        #         pump_state = [1,1]
        #     else:
        #         pump_state = [0,0]

        # # Dynamixel stuff
        # demand_angle = amp_angle * np.sin( 2 * np.pi * t / period) 
        # demand_angle_dynamixel_units = (demand_angle * 4096 / 360) + 2048 + 4096 * (offset/ 360)
        # dynamixel.write_position(demand_angle_dynamixel_units, dyn_id)

        # Send message to arduino
        # arduino.send_message(pump_state)


        time.sleep(0.001)
        
if __name__ == "__main__":
    main()
