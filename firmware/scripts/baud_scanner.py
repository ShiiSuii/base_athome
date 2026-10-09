import serial
import time
bauds = [115200, 57600, 38400, 19200, 9600]
print("--- INICIANDO ESCANEO DE BAUD RATES ---")
for b in bauds:
    print(f"Probando {b}...")
    try:
        s = serial.Serial('/dev/ttyUSB0', b, timeout=0.2)
        # mandar ping
        s.write(b'\xcd\xab\x00\x00\x00\x00\xcd\xab')
        s.flush()
        time.sleep(0.3)
        if s.in_waiting > 0:
            data = s.read(s.in_waiting)
            if b'\xcd\xab' in data:
                print(f"✅ ¡HEADER 0xABCD ENCONTRADO EN {b} BAUDIOS!")
                print(data.hex(" "))
                exit(0)
            else:
                print(f"Basura ({len(data)} bytes).")
        else:
            print("Nada.")
        s.close()
    except Exception as e:
        print(f"Error: {e}")
print("--- ESCANEO FINALIZADO SIN ÉXITO ---")
