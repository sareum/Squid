from comms_wrapper import Arduino
from dynamixel_controller import Dynamixel
import time

def main():
    arduino = Arduino( descriptiveDeviceName="Fishtail setup arduino", portName="/dev/ttyUSB0", baudrate=115200)
    arduino.connect_and_handshake()

    while 1:
        arduino.receive_message()
        print(arduino.receivedMessages)
        time.sleep(0.1)
        
if __name__ == "__main__":
    main()
