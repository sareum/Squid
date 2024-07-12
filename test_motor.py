from dynamixel_controller import Dynamixel
from time import sleep
#import keyboard
import argparse


def velocity_security(velocity):
    #velocity security
    if velocity > MAX_VELOCITY: velocity = MAX_VELOCITY
    if velocity < -MAX_VELOCITY: velocity = -MAX_VELOCITY
    return velocity

'''main loop'''
if __name__=="__main__":
    UNIT_SCALE = 0.229 #[rev/min]
    MAX_VELOCITY = 314 #72[rev/min]
    parser = argparse.ArgumentParser(
        description="give velocity in rpm"
        )
    parser.add_argument("--vel", required=False, type=int)
    args = parser.parse_args()

    velocity = 140
    if args.vel is not None:
        velocity = args.vel    

    #velocity security
    if velocity > MAX_VELOCITY: velocity = MAX_VELOCITY
    if velocity < -MAX_VELOCITY: velocity = -MAX_VELOCITY

    servo = Dynamixel(ID=1, descriptive_device_name="XW430-T200", 
                        series_name="xm", baudrate=3000000, port_name="/dev/ttyUSB0")
    servo.begin_communication()
    servo.set_operating_mode("position")

    servo.write_position(0)
    sleep(1)

        '''if user_input == 'w':
            velocity += 10
            velocity = velocity_security(velocity)
            servo.write_position(velocity)
            print("actual velocity:", velocity, "in rpm: ", velocity*UNIT_SCALE)
        if user_input == 's':
            velocity -= 10
            velocity = velocity_security(velocity)
            servo.write_velocity(velocity)
            print("actual velocity:", velocity, "in rpm: ", velocity*UNIT_SCALE)
        elif user_input == 'q':
            break
        # if keyboard.is_pressed("q"):
        #     print("q pressed, ending loop")
        #     break
        #sleep(0.1)'''
            
    servo.end_communication()






