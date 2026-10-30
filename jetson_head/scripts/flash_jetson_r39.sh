#!/bin/bash
# HOME-11: flashea JetPack 7.2.1 (Jetson Linux r39.2.1, Ubuntu 24.04) en la microSD
# de la Jetson Orin Nano Dev Kit (P3767-0005, Super) desde la NUC por USB-C en
# modo recovery. BORRA LA MICROSD Y ACTUALIZA EL QSPI. Ejecutar en la NUC.
#
#   flash_jetson_r39.sh prepare   # extrae rootfs, apply_binaries, usuario, red, SSH (no toca la Jetson)
#   flash_jetson_r39.sh flash     # requiere Jetson en recovery (lsusb 0955:7523) y CONFIRM=BORRAR_MICROSD
#
# Variables: JUSER (ubuntu), JPASS (obligatoria para prepare), JHOST (ubuntu-desktop).
set -euo pipefail
BASE="${BASE:-$HOME/jetson_flash/r39.2.1}"
L4T="$BASE/Linux_for_Tegra"
BSP="$BASE/Jetson_Linux_R39.2.1_aarch64.tbz2"
ROOTFS_TAR="$BASE/Tegra_Linux_Sample-Root-Filesystem_R39.2.1_aarch64.tbz2"
JUSER="${JUSER:-ubuntu}"; JHOST="${JHOST:-ubuntu-desktop}"
JETSON_IP=10.42.0.12/24; NUC_IP=10.42.0.1
BOARD=jetson-orin-nano-devkit-super

prepare() {
    : "${JPASS:?definir JPASS}"
    [ -d "$L4T" ] || tar xf "$BSP" -C "$BASE"
    if [ ! -e "$L4T/rootfs/etc/os-release" ]; then
        sudo tar xpf "$ROOTFS_TAR" -C "$L4T/rootfs/"
    fi
    cd "$L4T"
    sudo ./tools/l4t_flash_prerequisites.sh
    sudo ./apply_binaries.sh
    sudo ./tools/l4t_create_default_user.sh -u "$JUSER" -p "$JPASS" -n "$JHOST" --accept-license
    # Red fija hacia la NUC en cualquier adaptador USB-Ethernet (enx*), sin ruta por defecto
    sudo tee "$L4T/rootfs/etc/NetworkManager/system-connections/robot-nuc.nmconnection" >/dev/null <<EOF
[connection]
id=robot-nuc
type=ethernet
autoconnect=true
autoconnect-priority=10
match-device=interface-name:enx*

[ipv4]
method=manual
address1=$JETSON_IP,$NUC_IP
never-default=true

[ipv6]
method=ignore
EOF
    sudo chmod 600 "$L4T/rootfs/etc/NetworkManager/system-connections/robot-nuc.nmconnection"
    # Clave publica de la NUC (solo la .pub) para entrar por SSH sin pantalla
    PUB=$(ls "$HOME"/.ssh/id_ed25519.pub "$HOME"/.ssh/id_rsa.pub 2>/dev/null | head -1 || true)
    UHOME="$L4T/rootfs/home/$JUSER"
    UIDGID=$(sudo awk -F: -v u="$JUSER" '$1==u{print $3":"$4}' "$L4T/rootfs/etc/passwd")
    sudo install -d -m 700 -o "${UIDGID%:*}" -g "${UIDGID#*:}" "$UHOME/.ssh"
    sudo install -m 600 -o "${UIDGID%:*}" -g "${UIDGID#*:}" "$PUB" "$UHOME/.ssh/authorized_keys"
    sudo ls "$L4T/rootfs/usr/sbin/sshd" >/dev/null && echo "openssh-server presente en el rootfs"
    echo "prepare OK"
}

flash() {
    lsusb | grep -q "0955:7523" || { echo "Jetson no esta en recovery (esperado 0955:7523 NVIDIA Corp. APX)"; exit 1; }
    [ "${CONFIRM:-}" = "BORRAR_MICROSD" ] || { echo "Falta CONFIRM=BORRAR_MICROSD"; exit 1; }
    cd "$L4T"
    sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device mmcblk0p1 \
        -c tools/kernel_flash/flash_l4t_t234_nvme.xml \
        -p "-c bootloader/generic/cfg/flash_t234_qspi.xml" \
        --showlogs --network usb0 "$BOARD" internal
}

case "${1:-}" in
    prepare) prepare ;;
    flash) flash ;;
    *) echo "uso: $0 prepare|flash"; exit 2 ;;
esac
