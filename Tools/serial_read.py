import serial

def read_serial(port, baudrate=2000000, timeout=1):
    # Apre la connessione seriale
    ser = serial.Serial(port, baudrate, timeout=timeout)

    print(f"Connesso alla porta: {port} con baudrate: {baudrate}")
    
    try:
        while True:
            if ser.in_waiting > 0:
                # Legge una linea dalla porta seriale
                line = ser.readline().decode('utf-8').rstrip()
                print(f"Ricevuto: {line}")
    except KeyboardInterrupt:
        print("Interruzione da tastiera. Chiudo la porta seriale.")
    finally:
        ser.close()
        print("Porta seriale chiusa.")

if __name__ == "__main__":
    # Specifica la porta seriale e il baudrate (modifica questi parametri secondo le tue esigenze)
    serial_port = '/dev/ttyACM0'  # Cambia con la tua porta seriale (es. 'COM3' su Windows o '/dev/ttyUSB0' su Linux)
    baudrate = 2000000

    read_serial(serial_port, baudrate)
