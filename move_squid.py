import time
import numpy as np
import pygame
#from plot_module import show_graph
from dynamixel_controller import Dynamixel
from time import sleep

def set_position(time, a, c, T) : 
    position = a*np.sin(2*np.pi/T*time) + c 
    return position

def write_position(q_dynamixel, IDs) :
    servo.write_position(q_dynamixel, ID=IDs)

# Initialize Pygame
pygame.init()

servo = Dynamixel(ID=[1,2,3,4], descriptive_device_name="XW430-T200R test motor", 
                    series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/ttyUSB0")
                    #series_name=["xm","xm","xm","xm"], baudrate=3000000, port_name="/dev/tty.usbserial-FT78LT9E")

servo.begin_communication()

servo.set_operating_mode("position", ID = "all")
    
write_position(2048, [1,2,3,4]) #180°
sleep(1)

i = 0
num_cycles = 10
IDs = [1,2,3,4]
T = 1 #Period

# Real values
a = 45 #Amplitude en degré
c = 180 # Center position

# Dynamixel values
a_dyna = a * 2048/180
c_dyna = c * 2048/180 

#read_position = np.empty(1000)
#read_velocity = np.empty(1000)
#function_value = np.empty(1000)
#error = np.empty(1000)

timer = time.time()

# Set up the display
screen = pygame.display.set_mode((640, 480))

running = True

while running :

    t = time.time() - timer

    q_dynamixel = int(set_position(t,a_dyna, c_dyna, T))
    write_position(q_dynamixel, IDs)

    #function_value[i] = q   
    #read_position[i] = servo.read_position(4)
    #read_velocity[i] = servo.read_velocity(4)
    #error[i] = 180*(function_value[i] - read_position[i])/position_center

    if t > T*num_cycles :
        a = 27
        c = 63
        T = 1
        a_dyna = a * 2048/180
        c_dyna = c * 2048/180


    if t > 2*T*num_cycles :
        write_position(2048, [1,2]) #180°
        a = 45
        c = 180
        T = 1
        a_dyna = a * 2048/180
        c_dyna = c * 2048/180
        IDs = [3,4]

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False


#show_graph([read_position, function_value] , time, ["real values", "function values"]) 
#show_graph([error] , time, ["Error in degree"])
#show_graph([read_velocity] , time, ["Velocity"])

servo.end_communication()
pygame.quit()

