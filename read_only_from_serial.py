import serial
import time

serial_port = '/dev/ttyACM0'  # Cambia questo con la tua porta
baud_rate = 115200  # Questo deve corrispondere al baud rate impostato nel Teensy
ser = serial.Serial(serial_port, baud_rate, timeout=1)
ser.close()

#ser = serial.Serial(serial_port,baud_rate,timeout= 1)
ser.open()
ser.reset_input_buffer()
ser.reset_output_buffer()
print(f"Connessione aperta sulla porta {serial_port} con baud rate {baud_rate}")

while True:
    #initial_time = time.time()
    if ser.in_waiting > 0:
        # Legge una riga di dati dalla seriale
        
        serial_reads = ser.readline().decode('utf-8').rstrip()
        print("dati: ",serial_reads)
        time.sleep(0.06)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

