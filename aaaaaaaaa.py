import smbus
import struct
import time
# Indirizzo I2C del Teensy
TEENSY_I2C_ADDRESS = 0x08
# Creazione dell'oggetto bus
bus = smbus.SMBus(1)
def read_quaternions():
    # Lettura di 32 byte (8 float a 4 byte ciascuno)
    data = bus.read_i2c_block_data(TEENSY_I2C_ADDRESS, 0, 32)
    # Conversione dei dati binari in float
    qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2 = struct.unpack('f' * 8, bytearray(data))
    return qW1, qX1, qY1, qZ1, qW2, qX2, qY2, qZ2
def main():
    while True:
        try:
            # Legge i dati quaternion dal Teensy
            quaternions = read_quaternions()
            print(f"qW1: {quaternions[0]}, qX1: {quaternions[1]}, qY1: {quaternions[2]}, qZ1: {quaternions[3]}")
            print(f"qW2: {quaternions[4]}, qX2: {quaternions[5]}, qY2: {quaternions[6]}, qZ2: {quaternions[7]}")
            print("-" * 50)
            time.sleep(0.1)
        except OSError as e:
            print(f"Errore di comunicazione I2C: {e}")
            time.sleep(1)
if __name__ == "__main__":
    main()






