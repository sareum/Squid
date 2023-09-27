from comms_wrapper import Arduino
from dynamixel_controller import Dynamixel
import time
import numpy as np
import os
from utility import *
from copy import deepcopy as cp
import argparse
import os
import shutil

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq", type=int, default=1, choices=[1,2,3,4], help="1 = off, 2 = on, 3 = seq3, 4 = seq4")
    parser.add_argument("--fname", type=str, help="filename of saving file")
    parser.add_argument("--cycles", type=int, help="number of actuation cycles.")
    parser.add_argument("--period", type=float, default=3, help="length of period in seconds")
    parser.add_argument("--amp", type=int, default=20, help="amplitude angle in degrees")
    parser.add_argument("--arduino_port", default="/dev/ttyUSB0", help="port of arduino")
    parser.add_argument("--dyn_port", default="/dev/ttyUSB1", help="port of dynamixel")
    args = parser.parse_args()

    offset = 7
    config = {"seq":args.seq, "cycles":args.cycles, "period":args.period, "amp":args.amp}

    print("config:", config)

    # Connect loadcell arduino
    arduino = Arduino( descriptiveDeviceName="Fishtail setup arduino", portName=args.arduino_port, baudrate=115200)
    arduino.connect_and_handshake()

    # Connect dynamixel
    dyn_id = 12
    dynamixel = Dynamixel(dyn_id, "Fishtail setup dynamixel", args.dyn_port, 1000000, "xl")
    dynamixel.begin_communication()
    dynamixel.set_operating_mode("position", dyn_id)

    # Sine wave properties
    amp_angle = args.amp
    period = args.period
    num_cycles = args.cycles
    #num_cycles = 10 / period

    # Pump state
    pump_state = [0,0]
    
    # Recording
    time_data = []
    lc1_data =[]
    lc2_data = []
    rpm_data = []
    deg_data = []

    timer = time.time()
    while 1:
        t = time.time() - timer
        period_timer = t%period

        if args.seq == 1:
            # Sequence 1
            pump_state = [0,0]

        elif args.seq == 2:
            # Sequence 2
            pump_state = [1,1]

        elif args.seq == 3:
            # Sequence 3
            if period_timer < period/4:
                pump_state = [1,1]
            elif period_timer > period/2 and period_timer < period*3/4:
                pump_state = [1,1]
            else:
                pump_state = [0,0]

        elif args.seq == 4:
            # Sequence 4
            if period_timer > period/4 and period_timer < period/2:
                pump_state = [1,1]
            elif period_timer > period*3/4:
                pump_state = [1,1]
            else:
                pump_state = [0,0]

        # Dynamixel stuff
        demand_angle = amp_angle * np.sin( 2 * np.pi * t / period) 
        demand_angle_dynamixel_units = (demand_angle * 4096 / 360) + 2048 + 4096 * (offset/ 360)
        dynamixel.write_position(demand_angle_dynamixel_units, dyn_id)

        time.sleep(0.001)
        
        raw_pos = cp(dynamixel.read_position(dyn_id))
        raw_vel = cp(dynamixel.read_velocity(dyn_id))
        
        posdeg = (raw_pos - 2048) * 360 / 4096
        rpm = raw_vel * 0.229

        # Receive message from arduino
        arduino.receive_message()

        time_data.append(t)
        lc2_data.append(float(arduino.receivedMessages["lc2"]))
        rpm_data.append(float(rpm))
        deg_data.append(float(posdeg))

        # Send message to arduino
        arduino.send_message(pump_state)

        if t > period * num_cycles:
            pump_state = [0,0]
            arduino.send_message(pump_state)
            time.sleep(2)
            break

    base_name = "seq" + str(args.seq) + "_per" + str(args.period) + "_" + args.fname
    if os.path.exists(base_name):
        shutil.rmtree(base_name)

    os.mkdir(base_name)

    filename = base_name + "/" + base_name
    plot_and_save_data(plottingData= ([time_data, lc2_data], [time_data, lc2_data]),
                        xAxisLabel= ("time", "time"), 
                        yAxisLabel =("load(g)", "load(g)"),
                        label = (["loadcell 2"], ["loadcell 2"]), 
                        savingData= (time_data, lc2_data, rpm_data, deg_data),
                        savingData_names = ["time", "loadcell2", "dyn rpm", "dyn deg"], 
                        filename= filename,
                        saveDir= os.getcwd(),
                        display_plot= True, 
                        saveData = True, 
                        saveFig = True,
                        figsize = (6,8))

    save_to_json(filename, config)
        
if __name__ == "__main__":
    main()
