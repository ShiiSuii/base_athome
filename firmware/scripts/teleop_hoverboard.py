#!/usr/bin/env python3
import serial
import struct
import time
import sys
import select
import tty
import termios
import threading

PORT = '/dev/ttyUSB0'
BAUD = 115200

# Protocol Constants
START_FRAME = 0xABCD
FEEDBACK_SIZE = 22 # 22 bytes with ENABLE_ODOMETRY

# Global State
speed = 0
steer = 0
running = True

# Feedback State
feedback_data = {
    'batVoltage': 0.0,
    'boardTemp': 0.0,
    'speedR': 0,
    'speedL': 0,
    'wheelR_cnt': 0,
    'wheelL_cnt': 0,
    'valid_packets': 0,
    'last_rx': 0.0
}

def rx_thread_func(ser):
    global running, feedback_data
    buffer = bytearray()
    
    while running:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer += data
                
                while len(buffer) >= FEEDBACK_SIZE:
                    if buffer[0] == 0xCD and buffer[1] == 0xAB: # 0xABCD little-endian
                        pkt = buffer[:FEEDBACK_SIZE]
                        buffer = buffer[FEEDBACK_SIZE:]
                        
                        try:
                            start, c1, c2, spR, spL, wR, wL, bat, temp, cmdLed, chk = struct.unpack('<HhhhhhhHHH', pkt)
                            
                            # Verify checksum
                            calc_chk = (start ^ (c1 & 0xFFFF) ^ (c2 & 0xFFFF) ^ (spR & 0xFFFF) ^ (spL & 0xFFFF) ^ 
                                        (wR & 0xFFFF) ^ (wL & 0xFFFF) ^ (bat & 0xFFFF) ^ (temp & 0xFFFF) ^ (cmdLed & 0xFFFF)) & 0xFFFF
                            
                            feedback_data['batVoltage'] = bat / 100.0
                            feedback_data['boardTemp'] = temp / 10.0
                            feedback_data['speedR'] = spR
                            feedback_data['speedL'] = spL
                            feedback_data['wheelR_cnt'] = wR
                            feedback_data['wheelL_cnt'] = wL
                            feedback_data['valid_packets'] += 1
                            feedback_data['last_rx'] = time.time()
                        except Exception:
                            pass
                    else:
                        buffer.pop(0)
            time.sleep(0.005)
        except Exception:
            break

def get_key():
    fd = sys.stdin.fileno()
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    global speed, steer, running
    
    print(f"[*] Conectando al puerto {PORT} a {BAUD} baudios...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[-] Error abriendo {PORT}: {e}")
        return

    # Start RX background thread
    rx_thread = threading.Thread(target=rx_thread_func, args=(ser,), daemon=True)
    rx_thread.start()

    # Terminal settings for non-blocking key reads
    old_settings = None
    if sys.stdin.isatty():
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    print("\n=======================================================")
    print("      CONTROL TELEOP - HOVERBOARD FOC (RIGHT / USART3)  ")
    print("=======================================================")
    print("  Controles de Teclado:                                ")
    print("    [W] / [↑] : Acelerar Adelante (+50)                ")
    print("    [S] / [↓] : Retroceder (-50)                       ")
    print("    [A] / [←] : Girar Izquierda (-40)                  ")
    print("    [D] / [→] : Girar Derecha (+40)                    ")
    print("    [ESPACIO] : Freno de emergencia (Detener)          ")
    print("    [Q]       : Salir                                  ")
    print("=======================================================\n")

    STEP_SPEED = 50
    STEP_STEER = 40
    MAX_SPEED = 400
    MAX_STEER = 300

    last_tx_time = 0.0
    last_ui_time = 0.0

    try:
        while running:
            # 1. Check keyboard
            ch = get_key()
            if ch:
                if ch in ['w', 'W', 'A']: # 'A' is up arrow in escape sequences
                    speed = min(speed + STEP_SPEED, MAX_SPEED)
                elif ch in ['s', 'S', 'B']: # 'B' is down arrow
                    speed = max(speed - STEP_SPEED, -MAX_SPEED)
                elif ch in ['a', 'A_KEY', 'D']: # 'D' is left arrow
                    steer = max(steer - STEP_STEER, -MAX_STEER)
                elif ch in ['d', 'D_KEY', 'C']: # 'C' is right arrow
                    steer = min(steer + STEP_STEER, MAX_STEER)
                elif ch == ' ':
                    speed = 0
                    steer = 0
                elif ch in ['q', 'Q', '\x03']: # \x03 is Ctrl+C
                    running = False
                    break

            # 2. Transmit command packet at 50Hz (every 20ms)
            now = time.time()
            if now - last_tx_time >= 0.02:
                # Calculate checksum: start ^ steer ^ speed
                chk = (START_FRAME ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF
                cmd_pkt = struct.pack('<HhhH', START_FRAME, steer, speed, chk)
                ser.write(cmd_pkt)
                ser.flush()
                last_tx_time = now

            # 3. Print HUD in real-time
            if now - last_ui_time >= 0.1:
                rx_age = now - feedback_data['last_rx'] if feedback_data['last_rx'] > 0 else 999.0
                rx_status = "🟢 CONECTADO" if rx_age < 0.5 else "🔴 SIN DATOS"
                
                hud = (f"\r[{rx_status}] "
                       f"CMD -> Vel: {speed:4d} | Giro: {steer:4d}  ||  "
                       f"TELEMETRÍA -> Bat: {feedback_data['batVoltage']:5.2f}V | "
                       f"Temp: {feedback_data['boardTemp']:4.1f}°C | "
                       f"RPM: [L:{feedback_data['speedL']:4d}, R:{feedback_data['speedR']:4d}] | "
                       f"Odom: [L:{feedback_data['wheelL_cnt']:5d}, R:{feedback_data['wheelR_cnt']:5d}]")
                sys.stdout.write(hud)
                sys.stdout.flush()
                last_ui_time = now

            time.sleep(0.005)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        # Send zero stop command before closing
        try:
            stop_chk = (START_FRAME ^ 0 ^ 0) & 0xFFFF
            ser.write(struct.pack('<HhhH', START_FRAME, 0, 0, stop_chk))
            ser.flush()
            ser.close()
        except Exception:
            pass
        print("\n\n[+] Teleop finalizado de forma segura. Motores detenidos.")

if __name__ == '__main__':
    main()
