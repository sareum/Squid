import time
import smbus
# Imposta l'indirizzo I2C del Teensy
I2C_ADDRESS = 0x08
# Inizializza il bus I2C (bus 1 per Raspberry Pi 3 e successivi)
bus = smbus.SMBus(1)
def read_data():
    try:
        # Leggi una sequenza di 16 byte dal dispositivo I2C
        data = bus.read_i2c_block_data(I2C_ADDRESS, 0, 16)
        # Converti i byte in stringa (assumendo che il Teensy invii dati ASCII)
        return ''.join(chr(i) for i in data)
    except OSError as e:
        print(f"Errore durante la lettura: {e}")
        return None
while True:
    tic = time.time()
    received_data = read_data()
    if received_data:
        print(f"Ricevuto: {received_data}")
    print(time.time()-tic)
    #time.sleep(1) # Aspetta un secondo prima di leggere di nuovo