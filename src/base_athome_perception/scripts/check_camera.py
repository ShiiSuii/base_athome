#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Diagnóstico y Verificación de Estado de la Cámara (ZED Mini / RealSense)
# ==============================================================================

import os
import subprocess
import sys
import time

CYAN = "\033[96m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
RESET = "\033[0m"


def print_banner(text):
    print(f"\n{BOLD}{CYAN}=== {text} ==={RESET}")


def check_local_usb():
    print_banner("1. Puertos USB Locales (PC Base)")
    try:
        out = subprocess.check_output(["lsusb"], text=True)
        lines = out.strip().split("\n")
        zed_found = False
        realsense_found = False
        for l in lines:
            if "2b03:" in l.lower() or "stereolabs" in l.lower():
                print(f"  {GREEN}✔ Encontrada cámara Stereolabs ZED: {l}{RESET}")
                zed_found = True
            elif "8086:0b07" in l.lower() or "realsense" in l.lower():
                print(f"  {GREEN}✔ Encontrada cámara Intel RealSense: {l}{RESET}")
                realsense_found = True

        if not zed_found and not realsense_found:
            print(f"  {YELLOW}⚠ No se detectó cámara ZED ni RealSense conectada por USB directo a esta PC.{RESET}")
            print(f"    (Si la cámara está conectada a la Jetson Orin en el cabezal, revisaremos la red a continuación).")
    except Exception as e:
        print(f"  {RED}Error ejecutando lsusb: {e}{RESET}")

    # Verificar /dev/video*
    v_devs = [f"/dev/{d}" for d in os.listdir("/dev") if d.startswith("video")] if os.path.exists("/dev") else []
    if v_devs:
        print(f"  {GREEN}✔ Dispositivos V4L2 locales detectados: {sorted(v_devs)}{RESET}")
    else:
        print(f"  {YELLOW}ℹ No hay dispositivos /dev/video* locales.{RESET}")


def check_jetson_network():
    print_banner("2. Conexión de Red con Jetson Orin (Cabezal)")
    carrier_file = "/sys/class/net/enp1s0/carrier"
    has_carrier = False
    if os.path.exists(carrier_file):
        try:
            with open(carrier_file, "r") as f:
                has_carrier = (f.read().strip() == "1")
        except Exception:
            pass

    if has_carrier:
        print(f"  {GREEN}✔ Cable Ethernet (enp1s0) CONECTADO físicamente (Carrier = 1).{RESET}")
    else:
        print(f"  {RED}✖ Cable Ethernet (enp1s0) DESCONECTADO (Link Down / Carrier = 0).{RESET}")
        print(f"    👉 Asegúrate de que el cable Ethernet entre la PC base y la Jetson esté enchufado y la Jetson encendida.")
        return

    # Probar Ping a 10.42.0.12
    try:
        ret = subprocess.run(["ping", "-c", "1", "-W", "1", "10.42.0.12"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if ret.returncode == 0:
            print(f"  {GREEN}✔ Jetson Orin (10.42.0.12) RESPONDE al ping.{RESET}")
            # Probar SSH rápido para ver si el driver está corriendo
            try:
                ssh_out = subprocess.check_output(
                    ["ssh", "-o", "ConnectTimeout=2", "-o", "BatchMode=yes", "ubuntu@10.42.0.12", "lsusb | grep -i 2b03 || lsusb"],
                    text=True,
                    timeout=3
                )
                if "2b03:" in ssh_out.lower() or "stereolabs" in ssh_out.lower():
                    print(f"  {GREEN}✔ Cámara ZED Mini DETECTADA en el bus USB de la Jetson!{RESET}")
                else:
                    print(f"  {YELLOW}ℹ Dispositivos USB en la Jetson:\n{ssh_out.strip()}{RESET}")
            except Exception as e:
                print(f"  {YELLOW}ℹ No se pudo consultar USB remoto por SSH: {e}{RESET}")
        else:
            print(f"  {YELLOW}⚠ Jetson (10.42.0.12) no responde al ping todavía (posiblemente iniciando).{RESET}")
    except Exception as e:
        print(f"  {RED}Error al probar ping: {e}{RESET}")


def check_ros2_topics():
    print_banner("3. Tópicos de Cámara ROS 2 Activos")
    try:
        out = subprocess.check_output(["ros2", "topic", "list"], text=True, timeout=4)
        topics = [t.strip() for t in out.strip().split("\n") if t.strip()]
        cam_topics = [
            t for t in topics
            if any(k in t.lower() for k in ["zed", "camera", "image", "rgb"])
        ]
        if cam_topics:
            print(f"  {GREEN}✔ Se encontraron los siguientes tópicos de imagen activos:{RESET}")
            for ct in cam_topics:
                print(f"    - {BOLD}{ct}{RESET}")
        else:
            print(f"  {YELLOW}⚠ No hay tópicos de cámara publicando actualmente en el dominio ROS 2.{RESET}")
    except Exception:
        print(f"  {YELLOW}⚠ No se detectaron nodos ROS 2 activos en este momento.{RESET}")


def main():
    print(f"{BOLD}==================================================")
    print(f"  VERIFICACION DE ESTADO DE LA CAMARA (BASE@HOME)")
    print(f"=================================================={RESET}")

    check_local_usb()
    check_jetson_network()
    check_ros2_topics()

    print(f"\n{BOLD}{CYAN}=== Resumen de Comandos para Probar ==={RESET}")
    print(f"  - Si la cámara publica por ROS 2:")
    print(f"      {GREEN}rostros{RESET}    -> Inicia reconocimiento facial con auto-descubrimiento ZED")
    print(f"      {GREEN}poses{RESET}      -> Inicia detector de posturas humanas con ZED")
    print(f"      {GREEN}receptionist{RESET} -> Inicia la prueba completa de HRI Receptionist")
    print(f"  - Si conectas la cámara ZED por USB directo a la PC base:")
    print(f"      {GREEN}rostros --webcam 0{RESET}")
    print(f"      {GREEN}poses --webcam 0{RESET}")
    print(f"==================================================\n")


if __name__ == "__main__":
    main()
