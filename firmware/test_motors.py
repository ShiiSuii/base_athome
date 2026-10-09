import serial
import time
import sys

def main():
    try:
        # Abre el puerto
        s = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        print("Conectado a /dev/ttyUSB0.")
        print("Enviando CMD 0.4 (40% de potencia)... Presiona Ctrl+C para frenar.")
    except Exception as e:
        print(f"Error abriendo puerto: {e}")
        sys.exit(1)
        
    try:
        while True:
            # Enviamos la velocidad deseada (40%)
            s.write(b"CMD 0.4\n")
            
            # Leemos todo lo que el Arduino imprima en pantalla
            while s.in_waiting > 0:
                line = s.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(line)
            
            # Pausa de 100ms
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nFrenando motor (CMD 0.0) y saliendo...")
        # Apagón de seguridad
        s.write(b"CMD 0.0\n")
        time.sleep(0.1) # Damos tiempo a que el Arduino procese antes de matar el puerto
        s.close()
        sys.exit(0)

if __name__ == '__main__':
    main()
