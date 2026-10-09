import serial
import struct
import time

def send_command(ser, steer, speed):
    start = 0xABCD
    checksum = start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    packet = struct.pack('<HhhH', start, steer, speed, checksum)
    ser.write(packet)

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
        print("Conectado a /dev/ttyUSB0. Iniciando prueba de lazo cerrado (TX y RX)...")
        
        # Tamaño esperado del struct SerialFeedback
        # start(2) + cmd1(2) + cmd2(2) + speedR(2) + speedL(2) + batV(2) + temp(2) + cmdLed(2) + checksum(2)
        # = 18 bytes (sin odometría)
        
        start_time = time.time()
        rx_working = False
        
        while time.time() - start_time < 3.0:
            # Enviamos el comando a la placa (Speed=50)
            send_command(ser, 0, 50)
            
            # Leemos la telemetría que nos devuelve la placa
            if ser.in_waiting >= 18:
                data = ser.read(18)
                if len(data) == 18:
                    try:
                        # '<HhhhhhhhH' = 9 elementos de 2 bytes = 18 bytes
                        start, cmd1, cmd2, speedR, speedL, batV, temp, cmdLed, checksum = struct.unpack('<HhhhhhhhH', data)
                        if start == 0xABCD:
                            print(f"[Telemetría] La placa reporta estar recibiendo -> Steer (cmd1): {cmd1}, Speed (cmd2): {cmd2}")
                            if cmd2 == 50:
                                rx_working = True
                    except struct.error:
                        pass
                        
            time.sleep(0.05)
            
        print("\nDeteniendo motores...")
        send_command(ser, 0, 0)
        ser.close()
        
        if rx_working:
            print("\n>>> RESULTADO: ¡ÉXITO! La comunicación es bidireccional. La placa escucha y responde correctamente.")
        else:
            print("\n>>> RESULTADO: ¡FALLO! Recibimos datos (TX de la placa OK), pero la placa reporta Speed=0. El cable TX de tu conversor NO está haciendo contacto con el RX de la placa.")
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
