import serial
import struct
import time

def send_command(ser, steer, speed):
    start = 0xABCD
    # Es vital aplicar la máscara & 0xFFFF en Python para emular el XOR de enteros de 16-bits de C
    checksum = start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    # Empaquetamos en Little Endian: uint16, int16, int16, uint16
    packet = struct.pack('<HhhH', start, steer, speed, checksum)
    ser.write(packet)

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Conectado a /dev/ttyUSB0. Iniciando prueba de motores...")
        
        # Enviar comandos constantemente porque el firmware tiene un Timeout que apaga 
        # los motores si no recibe datos en un período de tiempo.
        print("Enviando velocidad muy baja (Speed=50, Steer=0) durante 3 segundos...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            send_command(ser, 0, 50)
            time.sleep(0.05)  # Enviar cada 50ms
            
        print("Deteniendo motores...")
        for _ in range(10):
            send_command(ser, 0, 0)
            time.sleep(0.05)
            
        ser.close()
        print("Prueba finalizada con éxito.")
    except Exception as e:
        print(f"Error crítico en la comunicación serial: {e}")

if __name__ == '__main__':
    main()
