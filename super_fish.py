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
    parser.add_argument("--fname", type=str, help="filename of saving file")
    parser.add_argument("--amp", type=int, default=20, help="amplitude angle in degrees")
    parser.add_argument("--offset", type=int, default=7, help="offset in amplitude angle")
    parser.add_argument("--arduino_port", default="/dev/ttyUSB0", help="port of arduino")
    parser.add_argument("--dyn_port", default="/dev/ttyUSB1", help="port of dynamixel")
    parser.add_argument("--seq_list", type=list, default=[1])
    parser.add_argument("--period_list", type=list, default=[1])
    parser.add_argument("--cycles_list", type=list, default=[3])
    args = parser.parse_args()

    seq_list = [float(x) for x in args.seq_list]
    cycles_list = [float(x) for x in args.cycles_list]
    period_list = [float(x) for x in args.period_list]
    
    offset = args.offset
    config = {"seq":seq_list, "cycles":cycles_list, "period":period_list, "amp":args.amp}

    print("config:", config)

    # Connect loadcell arduino
    arduino = Arduino( descriptiveDeviceName="Fishtail setup arduino", portName=args.arduino_port, baudrate=115200)
    arduino.connect_and_handshake()

    # Connect dynamixel
    dyn_id = 12
    dynamixel = Dynamixel(dyn_id, "Fishtail setup dynamixel", args.dyn_port, 1000000, "xl")
    dynamixel.begin_communication()
    dynamixel.set_operating_mode("position", dyn_id)
    
    # Pump state
    pump_state = [0,0]
    
    # Recording
    time_data = []
    lc1_data =[]
    lc2_data = []
    rpm_data = []
    deg_data = []

    local_timer = cp(time.time())
    global_timer = cp(time.time())
    period_timer = 0
    
    cycle_count = 0
    idx_count = 0

    amp_angle = args.amp
    period = period_list[idx_count]
    sequence = seq_list[idx_count]
    
    while 1:
        t = time.time() - local_timer
        temp = t%period
        
        # if there is a change in cycle
        if temp - period_timer < 0.2:
            cycle_count += 1
            if cycle_count == cycles_list[idx_count]:
                idx_count += 1

                if idx_count == len(cycle_count):
                    break

                period = period_list[idx_count]
                sequence = seq_list[idx_count]
                cycle_count = 0

        period_timer = cp(temp)

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

        t_global = time.time() - global_timer
        time_data.append(t_global)
        lc2_data.append(float(arduino.receivedMessages["lc2"]))
        rpm_data.append(float(rpm))
        deg_data.append(float(posdeg))

        # Send message to arduino
        arduino.send_message(pump_state)

    pump_state = [0,0]
    arduino.send_message(pump_state)
    time.sleep(2)
    
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
