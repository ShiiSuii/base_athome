import serial
import struct
import time
import threading
import sys

# Variables globales para compartir entre hilos
current_steer = 0
current_speed = 0
running = True

def send_command(ser, steer, speed):
    """ Empaqueta y envía el comando a la hoverboard """
    start = 0xABCD
    checksum = start ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)
    packet = struct.pack('<HhhH', start, steer, speed, checksum)
    ser.write(packet)

def serial_worker(port='/dev/ttyUSB0', baudrate=115200):
    """ Hilo que envía continuamente el último comando para evitar el timeout de la placa """
    global current_steer, current_speed, running
    
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"[*] Conectado a {port}")
    except Exception as e:
        print(f"[!] Error conectando a {port}: {e}")
        running = False
        return

    while running:
        try:
            send_command(ser, current_steer, current_speed)
            # Leer el buffer para limpiarlo y evitar que se acumule telemetría
            if ser.in_waiting:
                ser.read(ser.in_waiting)
        except Exception as e:
            print(f"[!] Error de escritura: {e}")
            break
        
        # Enviar cada 50ms (20Hz)
        time.sleep(0.05)
        
    # Al salir, asegurarse de apagar los motores
    for _ in range(5):
        try:
            send_command(ser, 0, 0)
            time.sleep(0.05)
        except:
            pass
            
    ser.close()
    print("[*] Puerto serial cerrado.")

def main():
    global current_steer, current_speed, running
    
    print("==================================================")
    print("      Control de Hoverboard por Terminal")
    print("==================================================")
    print("Instrucciones:")
    print(" - Ingresa los comandos en el formato: Steer Speed")
    print(" - Ejemplo para avanzar: 0 150")
    print(" - Ejemplo para girar: 100 0")
    print(" - Rango permitido por defecto: -1000 a 1000")
    print(" - Escribe 's' o '0 0' para frenar (Speed 0, Steer 0)")
    print(" - Escribe 'q' o 'quit' para salir del programa")
    print("==================================================\n")
    
    # Iniciar hilo de comunicación serial
    t = threading.Thread(target=serial_worker, daemon=True)
    t.start()
    
    # Dar tiempo a que el hilo conecte y muestre mensaje
    time.sleep(0.5) 
    
    if not running:
        sys.exit(1)
        
    try:
        while running:
            user_input = input("Comando (Steer Speed) > ").strip().lower()
            
            if user_input in ['q', 'quit', 'exit']:
                print("Saliendo...")
                break
                
            if user_input in ['s', 'stop']:
                current_steer = 0
                current_speed = 0
                print("-> FRENANDO (0, 0)")
                continue
                
            parts = user_input.split()
            if len(parts) != 2:
                print("[!] Formato inválido. Usa: Steer Speed (ej. 0 150)")
                continue
                
            try:
                steer_val = int(parts[0])
                speed_val = int(parts[1])
                
                # Actualizar variables globales que lee el hilo
                current_steer = steer_val
                current_speed = speed_val
                print(f"-> Nuevo comando establecido: Steer={current_steer}, Speed={current_speed}")
                
            except ValueError:
                print("[!] Por favor ingresa números enteros.")
                
    except KeyboardInterrupt:
        print("\nSaliendo...")
        
    finally:
        # Parar el hilo y frenar los motores
        current_steer = 0
        current_speed = 0
        running = False
        t.join()

if __name__ == '__main__':
    main()
