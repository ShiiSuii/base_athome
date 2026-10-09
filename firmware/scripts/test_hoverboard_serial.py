import serial
import time
import struct
import sys

PORT = '/dev/ttyUSB0'
BAUD = 115200
START_FRAME = 0xABCD

def send_command(ser, steer, speed):
    # struct format: uint16_t start, int16_t steer, int16_t speed, uint16_t checksum
    # checksum = start ^ steer ^ speed
    checksum = START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    # Empaquetar en little-endian
    packet = struct.pack('<HhhH', START_FRAME, steer, speed, checksum)
    ser.write(packet)

def main():
    print(f"Intentando abrir {PORT} a {BAUD} baudios...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error al abrir el puerto: {e}")
        print("Asegúrate de tener permisos (sudo usermod -a -G dialout $USER)")
        sys.exit(1)
        
    print("Puerto abierto con éxito. Escuchando...")
    
    # Intento 1: Solo escuchar a ver si la placa manda telemetría sola
    end_time = time.time() + 3
    received_data = False
    
    while time.time() < end_time:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"RECIBIDO (Crudo): {data.hex()}")
            received_data = True
            
    if not received_data:
        print("No se recibió nada automáticamente. Intentando enviar un comando neutral...")
        # Enviar comando de velocidad 0
        send_command(ser, 0, 0)
        time.sleep(0.1)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"RECIBIDO TRAS COMANDO: {data.hex()}")
        else:
            print("No hubo respuesta de la placa tras enviar el comando.")
            print("Posibles causas:")
            print("1. El firmware no está instalado o es otra variante.")
            print("2. Los cables TX y RX del FTDI están al revés (prueba invertirlos).")
            print("3. La placa no está encendida.")

    ser.close()

if __name__ == "__main__":
    main()
