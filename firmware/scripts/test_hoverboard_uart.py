import serial
import time

try:
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
    print("Conectado a /dev/ttyUSB0. Escuchando telemetría...")
    
    start_time = time.time()
    data_received = False
    
    while time.time() - start_time < 3.0:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"[{time.time() - start_time:.2f}s] Datos recibidos ({len(data)} bytes): {data.hex()}")
            data_received = True
            time.sleep(0.1)
            
    if not data_received:
        print("No se recibieron datos. Verifica las conexiones de TX/RX.")
        
    ser.close()
except Exception as e:
    print(f"Error: {e}")
