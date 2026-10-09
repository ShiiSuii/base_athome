import serial
import time
import struct
import sys

PORT = '/dev/ttyUSB0'
BAUD = 115200
START_FRAME = 0xABCD

# Struct de Feedback de EFeru Hoverboard FOC (18 bytes)
# uint16_t start, int16_t cmd1, int16_t cmd2, int16_t speedR_meas, int16_t speedL_meas
# int16_t batVoltage, int16_t boardTemp, uint16_t cmdLed, uint16_t checksum
FORMAT = '<HhhhhhhHH'
PACKET_SIZE = 18

def main():
    print(f"Abriendo {PORT} a {BAUD} baudios...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        print(f"Error al abrir puerto: {e}")
        sys.exit(1)

    print("Sincronizando con el flujo de telemetría de Hoverboard (EFeru)...")
    
    buffer = bytearray()
    valid_packets = 0
    end_time = time.time() + 5
    
    while time.time() < end_time and valid_packets < 5:
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting)
            
            # Buscar el inicio de trama (0xABCD en little endian -> CD AB)
            while len(buffer) >= PACKET_SIZE:
                # El header 0xABCD en little endian es [0xCD, 0xAB]
                if buffer[0] == 0xCD and buffer[1] == 0xAB:
                    packet = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]
                    
                    try:
                        parsed = struct.unpack(FORMAT, packet)
                        start, cmd1, cmd2, speedR, speedL, batVol, temp, cmdLed, checksum = parsed
                        
                        # Validar checksum: start ^ cmd1 ^ cmd2 ^ speedR ^ speedL ^ batVol ^ temp ^ cmdLed
                        calc_checksum = (start ^ (cmd1 & 0xFFFF) ^ (cmd2 & 0xFFFF) ^ 
                                         (speedR & 0xFFFF) ^ (speedL & 0xFFFF) ^ 
                                         (batVol & 0xFFFF) ^ (temp & 0xFFFF) ^ cmdLed)
                                         
                        if checksum == calc_checksum:
                            print(f"\n✅ TELEMETRÍA VÁLIDA #{valid_packets+1}")
                            print(f"  Voltaje Batería: {batVol / 100.0:.2f} V")
                            print(f"  Temperatura:     {temp / 10.0:.1f} °C")
                            print(f"  Velocidad Izq:   {speedL}")
                            print(f"  Velocidad Der:   {speedR}")
                            valid_packets += 1
                        else:
                            print("⚠️ Checksum inválido, ignorando paquete.")
                            
                    except Exception as e:
                        print("Error al desempaquetar:", e)
                else:
                    # Si no es el inicio, descartamos 1 byte y seguimos buscando
                    buffer.pop(0)

    if valid_packets > 0:
        print("\n¡ÉXITO! La placa del hoverboard tiene el firmware EFeru instalado y está enviando datos correctos por UART.")
    else:
        print("\nNo se logró decodificar paquetes válidos. Puede que el baudrate sea distinto o sea un firmware diferente.")

    ser.close()

if __name__ == "__main__":
    main()
