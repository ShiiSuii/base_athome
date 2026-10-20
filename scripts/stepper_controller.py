#!/usr/bin/env python3
"""
NEMA23 Stepper Motor Terminal Controller
-----------------------------------------
Interactive CLI & Command Line Interface for controlling a stepper motor
connected to an Arduino Nano via USB Serial.
"""

import sys
import os
import time
import argparse
import select
import termios
import tty
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 'pyserial' module is not installed. Install it via: pip install pyserial")
    sys.exit(1)


def find_arduino_port():
    """Auto-detect connected USB serial port for Arduino Nano, safely ignoring Hoverboard."""
    # 1. Chequeo directo por ID persistente de udev para el chip CH340 de Arduino Nano
    by_id_nano = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    if os.path.exists(by_id_nano):
        return by_id_nano

    ports = list(serial.tools.list_ports.comports())
    candidates = []
    for p in ports:
        port_name = p.device
        desc = p.description.lower()
        hwid = p.hwid.lower()

        # Ignorar explícitamente el conversor FTDI del Hoverboard (VID:PID 0403:6001 en /dev/ttyUSB0)
        if "0403:6001" in hwid or "ft232r" in desc or (port_name == "/dev/ttyUSB0" and "ftdi" in desc):
            continue

        # Identificar chip CH340 / CH341 / QinHeng / Arduino Nano / ACM
        if "1a86" in hwid or "ch340" in desc or "ch341" in desc or "arduino" in desc or "acm" in port_name.lower():
            return port_name

        if "usb" in port_name.lower() and port_name != "/dev/ttyUSB0":
            candidates.append(port_name)

    if candidates:
        return candidates[0]

    # Fallback seguro evitando /dev/ttyUSB0
    for dev in ["/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB2"]:
        if os.path.exists(dev):
            return dev
    return None


class StepperController:
    def __init__(self, port=None, baudrate=115200, timeout=1.0):
        self.port = port or find_arduino_port()
        self.baudrate = baudrate
        self.ser = None

        if not self.port:
            print("[!] Warning: No Arduino serial port detected automatically.")
        else:
            self.connect()

    def connect(self, port=None):
        if port:
            self.port = port
        if not self.port:
            print("[!] Cannot connect: No port specified.")
            return False

        try:
            print(f"[*] Connecting to Arduino on {self.port} at {self.baudrate} baud...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2.0)  # Wait for Arduino reset after connection
            self.ser.reset_input_buffer()
            print(f"[✓] Connected successfully to {self.port}")
            self.read_responses(timeout_sec=0.5)
            return True
        except PermissionError:
            print(f"\n[X] ERROR DE PERMISOS en {self.port}.")
            print(f"    👉 Ejecuta en la terminal: sudo chmod 666 {self.port}")
            print(f"    O agrega tu usuario al grupo: sudo usermod -aG dialout $USER\n")
            self.ser = None
            return False
        except Exception as e:
            if "Permission denied" in str(e) or "Errno 13" in str(e):
                print(f"\n[X] ERROR DE PERMISOS en {self.port}.")
                print(f"    👉 Ejecuta en la terminal: sudo chmod 666 {self.port}")
                print(f"    O agrega tu usuario al grupo: sudo usermod -aG dialout $USER\n")
            else:
                print(f"[X] Connection failed on {self.port}: {e}")
            self.ser = None
            return False

    def send_cmd(self, command: str, print_response=True):
        if not self.ser or not self.ser.is_open:
            print("[!] Serial port is not connected.")
            return False

        try:
            full_cmd = command.strip() + "\n"
            self.ser.write(full_cmd.encode('utf-8'))
            self.ser.flush()
            if print_response:
                time.sleep(0.1)
                self.read_responses()
            return True
        except Exception as e:
            print(f"[X] Error sending command '{command}': {e}")
            return False

    def read_responses(self, timeout_sec=0.2):
        if not self.ser:
            return
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"  [Arduino] {line}")
            else:
                time.sleep(0.02)

    def close(self):
        if self.ser and self.ser.is_open:
            self.send_cmd("S", print_response=False)
            self.ser.close()
            print("[*] Serial port closed.")


def get_key_nonblocking():
    """Read a single character or arrow key sequence from standard input without blocking."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                # Escape sequence for arrow keys
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.01)
                if rlist2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        rlist3, _, _ = select.select([sys.stdin], [], [], 0.01)
                        if rlist3:
                            ch3 = sys.stdin.read(1)
                            return f"\x1b[{ch3}"
                return '\x1b'
            return ch
        return None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def interactive_keyboard_mode(controller: StepperController):
    """Live control using Arrow keys and Keyboard shortcuts."""
    print("\n=======================================================")
    print("        LIVE KEYBOARD STEPPER CONTROL MODE             ")
    print("=======================================================")
    print(" Controls:")
    print("  [RIGHT ARROW]  -> Rotate Forward (Clockwise)")
    print("  [LEFT ARROW]   -> Rotate Reverse (Counter-Clockwise)")
    print("  [UP ARROW]     -> Increase Speed (Faster)")
    print("  [DOWN ARROW]   -> Decrease Speed (Slower)")
    print("  [SPACEBAR]     -> Emergency STOP")
    print("  [q] or [ESC]   -> Exit Keyboard Mode")
    print("=======================================================\n")

    try:
        while True:
            key = get_key_nonblocking()
            if not key:
                continue

            if key == '\x1b[C':  # Right Arrow
                print(">> Action: FORWARD")
                controller.send_cmd("F")
            elif key == '\x1b[D':  # Left Arrow
                print(">> Action: REVERSE")
                controller.send_cmd("R")
            elif key == '\x1b[A':  # Up Arrow
                print(">> Action: FASTER (+)")
                controller.send_cmd("+")
            elif key == '\x1b[B':  # Down Arrow
                print(">> Action: SLOWER (-)")
                controller.send_cmd("-")
            elif key == ' ':
                print(">> Action: STOP")
                controller.send_cmd("S")
            elif key == 'q' or key == 'Q' or key == '\x1b':
                print("[*] Exiting Live Keyboard Mode...")
                break
    except KeyboardInterrupt:
        print("\n[*] Exiting Keyboard Mode...")


def print_help_menu():
    print("""
============================================================
              NEMA23 STEPPER TERMINAL COMMANDS
============================================================
  forward / f         : Rotate forward continuous (CW)
  reverse / r         : Rotate reverse continuous (CCW)
  stop / s            : Emergency stop motor
  +                   : Increase speed (faster)
  -                   : Decrease speed (slower)
  speed <us> / v <us> : Set step pulse delay in microseconds (e.g. speed 400)
  move <steps> / m    : Move relative steps (e.g. move 400 or move -200)
  angle <deg> / a     : Move relative angle (e.g. angle 90 or angle -45)
  keys / k            : Enter Live Arrow-Keys Control Mode
  status / ?          : Print motor telemetry/status
  connect <port>      : Connect or switch serial port
  help                : Show this menu
  exit / quit         : Exit application
============================================================
""")


def interactive_cli_menu(controller: StepperController):
    print_help_menu()
    while True:
        try:
            prompt_str = f"stepper({controller.port or 'disconnected'})> "
            user_input = input(prompt_str).strip()
            if not user_input:
                continue

            parts = user_input.split()
            cmd_name = parts[0].lower()

            if cmd_name in ["exit", "quit"]:
                print("[*] Goodbye!")
                break
            elif cmd_name in ["help", "h"]:
                print_help_menu()
            elif cmd_name in ["forward", "f"]:
                controller.send_cmd("F")
            elif cmd_name in ["reverse", "r"]:
                controller.send_cmd("R")
            elif cmd_name in ["stop", "s"]:
                controller.send_cmd("S")
            elif cmd_name == "+":
                controller.send_cmd("+")
            elif cmd_name == "-":
                controller.send_cmd("-")
            elif cmd_name in ["keys", "k", "keyboard"]:
                interactive_keyboard_mode(controller)
            elif cmd_name in ["status", "?"]:
                controller.send_cmd("?")
            elif cmd_name in ["speed", "v"]:
                if len(parts) > 1:
                    controller.send_cmd(f"V {parts[1]}")
                else:
                    print("Usage: speed <interval_us> (e.g., speed 300)")
            elif cmd_name in ["move", "m"]:
                if len(parts) > 1:
                    controller.send_cmd(f"M {parts[1]}")
                else:
                    print("Usage: move <steps> (e.g., move 400)")
            elif cmd_name in ["angle", "a"]:
                if len(parts) > 1:
                    controller.send_cmd(f"A {parts[1]}")
                else:
                    print("Usage: angle <degrees> (e.g., angle 90)")
            elif cmd_name == "connect":
                target_port = parts[1] if len(parts) > 1 else None
                controller.connect(target_port)
            else:
                # Pass raw command directly to Arduino
                controller.send_cmd(user_input)

        except (KeyboardInterrupt, EOFError):
            print("\n[*] Exiting CLI...")
            break


def main():
    parser = argparse.ArgumentParser(description="Terminal Stepper Motor Controller for Arduino Nano")
    parser.add_argument("--port", "-p", help="Serial port (e.g., /dev/ttyUSB1, /dev/ttyACM0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--action", "-a", choices=["forward", "reverse", "stop", "status", "keys"], help="Direct action to perform")
    parser.add_argument("--move", "-m", type=int, help="Move relative steps (e.g. 400)")
    parser.add_argument("--angle", type=float, help="Move relative angle in degrees (e.g. 90.0)")
    parser.add_argument("--speed", type=int, help="Set pulse interval in microseconds (e.g. 400)")

    args = parser.parse_args()

    controller = StepperController(port=args.port, baudrate=args.baud)

    # Direct action mode
    if args.action or args.move is not None or args.angle is not None or args.speed is not None:
        if args.speed:
            controller.send_cmd(f"V {args.speed}")
        if args.action == "forward":
            controller.send_cmd("F")
        elif args.action == "reverse":
            controller.send_cmd("R")
        elif args.action == "stop":
            controller.send_cmd("S")
        elif args.action == "status":
            controller.send_cmd("?")
        elif args.action == "keys":
            interactive_keyboard_mode(controller)

        if args.move is not None:
            controller.send_cmd(f"M {args.move}")
        if args.angle is not None:
            controller.send_cmd(f"A {args.angle}")

        time.sleep(0.5)
        controller.close()
    else:
        # Interactive CLI mode
        try:
            interactive_cli_menu(controller)
        finally:
            controller.close()


if __name__ == "__main__":
    main()
