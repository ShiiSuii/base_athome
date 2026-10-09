import serial
import struct
import time
import sys

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("=====================================================")
        print("   Diagnóstico de Motores y Sensores Hall")
        print("=====================================================")
        print("Asegúrate de que la placa esté ENCENDIDA (hace pitido).")
        print("Gira ambas ruedas con la mano para ver si los")
        print("sensores Hall las detectan correctamente.")
        print("Presiona Ctrl+C para salir.\n")
        
        buffer = bytearray()
        last_print = time.time()
        
        while True:
            # Leer todos los datos disponibles
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            
            # Buscar el inicio de trama 0xABCD (en little endian es \xcd\xab)
            # El struct SerialFeedback tiene 18 bytes (sin odometria)
            while len(buffer) >= 18:
                # \xcd\xab en bytes
                if buffer[0] == 0xcd and buffer[1] == 0xab:
                    data = buffer[:18]
                    del buffer[:18]
                    
                    try:
                        start, cmd1, cmd2, speedR, speedL, batV, temp, cmdLed, checksum = struct.unpack('<HhhhhhhhH', data)
                        if start == 0xABCD:
                            # Imprimir a un máximo de 10 Hz para no saturar la terminal
                            if time.time() - last_print > 0.1:
                                has_error = (cmdLed & 1) != 0
                                sys.stdout.write(f"\rSpeed L: {speedL:<5} | Speed R: {speedR:<5} | VBat: {batV/100.0:.1f}V | Err: {'SI' if has_error else 'NO'}   ")
                                sys.stdout.flush()
                                last_print = time.time()
                    except struct.error:
                        pass
                else:
                    # Descartar el primer byte si no es el inicio de la trama
                    del buffer[0]
                    
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nSaliendo del diagnóstico...")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
