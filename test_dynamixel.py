from dynamixel_controller import Dynamixel

def main():
    dyn_id = 12
    dynamixel = Dynamixel(dyn_id, "Fishtail setup dynamixel", "/dev/ttyUSB1", 1000000, "xl")
    dynamixel.begin_communication()
    dynamixel.set_operating_mode("position", dyn_id)
        
if __name__ == "__main__":
    main()
