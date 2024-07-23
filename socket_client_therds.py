import time
import numpy as np
import socket
import json
from dynamixel_controller import Dynamixel
from threading import Thread, Lock

HOST = '10.20.30.10'
PORT = 12345

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the address and port
server_address = (HOST, PORT)
server_socket.bind(server_address)

# Listen for incoming connections
server_socket.listen(1)

print("Waiting for a connection...")
client_socket, client_address = server_socket.accept()

servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")

def set_position(time, a, c, T):
    return a * np.sin(2 * np.pi / T * time) + c

def write_motor_position_sin(time, a_right, c_right, T_right, a_left, c_left, T_left):
    ID_right = [1,2]
    ID_left = [3,4]

    a_dyna_right = a_right * 2048 / 180
    c_dyna_right = c_right * 2048 / 180 
    a_dyna_left = a_left * 2048 / 180
    c_dyna_left = c_left * 2048 / 180 

    q_dynamixel_right = set_position(time, a_dyna_right, c_dyna_right, T_right)
    q_dynamixel_left = set_position(time, a_dyna_left, c_dyna_left, T_left)

    servo.write_position(q_dynamixel_right, ID_right)
    servo.write_position(q_dynamixel_left, ID_left)
    
    data = [q_dynamixel_right, q_dynamixel_left]
    return data

servo.begin_communication()
servo.set_operating_mode("position", ID="all")

servo.write_position(2040, [1,2,3,4])
time.sleep(1)

lock = Lock()
data_to_send = []
running = True

def communication_thread():
    global data_to_send
    while running:
        data = client_socket.recv(1024)
        if not data:
            running = False
            break
        data = json.loads(data.decode())
        
        
        t = time.time() - start_time
        motor_command = write_motor_position_sin(t, data["a_right"], data["c_right"], data["T_right"], 
                                                 data["a_left"], data["c_left"], data["T_left"])

        motor_command_right = 180 * motor_command[0] / 2048
        motor_command_left = 180 * motor_command[1] / 2048

        read_position_right = 180 * servo.read_position(1) / 2048
        read_position_left = 180 * servo.read_position(3) / 2048

        lock.acquire()
        data_to_send.append({
            "Motor_position_right": read_position_right,
            "Motor_position_left": read_position_left,
            "Motor_command_right": motor_command_right,
            "Motor_command_left": motor_command_left,
            "time": t
        })
        lock.release()

start_time = time.time()
comm_thread = Thread(target=communication_thread)
comm_thread.start()


while running:
    lock.acquire()
    if len(data_to_send) > 0:
        json_position = json.dumps(data_to_send.pop(0))
        client_socket.send(json_position.encode())
    lock.release()
    time.sleep(0.01)

comm_thread.join()
client_socket.close()
servo.end_communication()
print("Connection closed, communication ended.")