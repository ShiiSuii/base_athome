#!/usr/bin/env bash
# HOME-66: Diagnostico y endurecimiento de estabilidad para AnyDesk en la NUC.
#
# Modos de uso:
#   scripts/anydesk_estable.sh [--check]   # (Por defecto) Solo lectura, sin cambios ni sudo.
#   sudo scripts/anydesk_estable.sh --apply  # Aplica cambios con respaldo e idempotencia (sin reiniciar AnyDesk ni red).
#   sudo scripts/anydesk_estable.sh --revert # Revierte los cambios desde el respaldo (sin reiniciar AnyDesk ni red).
#
# Seguridad:
# - Nunca reinicia ni detiene AnyDesk, NetworkManager, GDM ni la NUC.
# - Nunca toca enp1s0 (enlace dedicado NUC <-> Jetson), firewall, usuarios, sudoers ni contrasenias.
# - Nunca lee ni modifica /etc/anydesk/service.conf ni /etc/anydesk/system.conf (credenciales/hashes).

set -uo pipefail

MODE="${1:---check}"
BACKUP_DIR="${ANYDESK_BACKUP_DIR:-/var/backups/base_athome_anydesk}"
ORIG_DIR="$BACKUP_DIR/orig"
TARGET_USER="${SUDO_USER:-athome}"

DROPIN_DIR="/etc/systemd/system/anydesk.service.d"
DROPIN_FILE="$DROPIN_DIR/base-athome-estable.conf"
NM_CONF_DIR="/etc/NetworkManager/conf.d"
NM_POWERSAVE_FILE="$NM_CONF_DIR/90-base-athome-wifi-powersave-off.conf"
SYSCTL_FILE="/etc/sysctl.d/90-base-athome-anydesk-keepalive.conf"
GDM_CONF="/etc/gdm3/custom.conf"
WIFI_IF="wlo1"
PROTECTED_IF="enp1s0"

OK_COUNT=0
WARN_COUNT=0

ok()   { echo "  [OK]    $*"; OK_COUNT=$((OK_COUNT + 1)); }
warn() { echo "  [FALTA] $*"; WARN_COUNT=$((WARN_COUNT + 1)); }
info() { echo "  [INFO]  $*"; }
err()  { echo "  [ERROR] $*" >&2; }

usage() {
    cat <<'EOF'
Uso: scripts/anydesk_estable.sh [--check | --apply | --revert | --help]

  --check   (Por defecto) Verifica el estado actual sin modificar nada. No requiere sudo.
  --apply   Aplica mejoras de estabilidad con respaldo previo e idempotencia. Requiere sudo.
            NO reinicia AnyDesk ni NetworkManager para no cortar una sesion activa.
  --revert  Restaura la configuracion original desde el respaldo. Requiere sudo.
            NO reinicia AnyDesk ni NetworkManager.
EOF
}

run_gsettings_get() {
    local schema="$1" key="$2"
    if [ "${EUID:-$(id -u)}" -eq 0 ] && id "$TARGET_USER" >/dev/null 2>&1; then
        local uid
        uid="$(id -u "$TARGET_USER")"
        sudo -u "$TARGET_USER" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus" \
            gsettings get "$schema" "$key" 2>/dev/null || true
    else
        gsettings get "$schema" "$key" 2>/dev/null || true
    fi
}

run_gsettings_set() {
    local schema="$1" key="$2" val="$3"
    if [ "${EUID:-$(id -u)}" -eq 0 ] && id "$TARGET_USER" >/dev/null 2>&1; then
        local uid
        uid="$(id -u "$TARGET_USER")"
        if [ -S "/run/user/$uid/bus" ]; then
            sudo -u "$TARGET_USER" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$uid/bus" \
                gsettings set "$schema" "$key" "$val" 2>/dev/null || return 1
            return 0
        fi
        return 1
    else
        gsettings set "$schema" "$key" "$val" 2>/dev/null || return 1
        return 0
    fi
}

check_status() {
    echo "=== HOME-66: Verificacion de estabilidad de AnyDesk (modo solo lectura) ==="
    echo "Fecha: $(date -Is)"
    echo

    echo "1. Servicio systemd (anydesk.service)"
    local active enabled restart pidfile nrestarts
    active="$(systemctl is-active anydesk.service 2>/dev/null || echo "unknown")"
    enabled="$(systemctl is-enabled anydesk.service 2>/dev/null || echo "unknown")"
    restart="$(systemctl show -p Restart --value anydesk.service 2>/dev/null || echo "unknown")"
    pidfile="$(systemctl show -p PIDFile --value anydesk.service 2>/dev/null || echo "unknown")"
    nrestarts="$(systemctl show -p NRestarts --value anydesk.service 2>/dev/null || echo "?")"

    if [ "$active" = "active" ] && [ "$enabled" = "enabled" ]; then
        ok "anydesk.service activo y habilitado (NRestarts=$nrestarts)"
    else
        warn "anydesk.service estado=$active, habilitado=$enabled"
    fi

    if [ "$restart" = "always" ] || [ "$restart" = "on-failure" ]; then
        ok "Politica de reinicio automatico activa: Restart=$restart"
    else
        warn "Restart=$restart en anydesk.service (si el demonio crashea no se levanta solo)"
    fi

    if [ -f "$DROPIN_FILE" ]; then
        ok "Drop-in presente: $DROPIN_FILE"
    else
        warn "Falta drop-in de systemd: $DROPIN_FILE"
    fi
    info "PIDFile efectivo: $pidfile"
    echo

    echo "2. Red Wi-Fi ($WIFI_IF) y perfiles de NetworkManager ($PROTECTED_IF intacto)"
    if [ -f "$NM_POWERSAVE_FILE" ] && grep -Eq '^[[:space:]]*wifi\.powersave[[:space:]]*=[[:space:]]*2([[:space:]]|$)' "$NM_POWERSAVE_FILE"; then
        ok "Ahorro de energia Wi-Fi desactivado en $NM_POWERSAVE_FILE (wifi.powersave=2)"
    else
        warn "Falta $NM_POWERSAVE_FILE con wifi.powersave = 2"
    fi

    if command -v nmcli >/dev/null 2>&1; then
        local utec_auto urubots_auto urubots_prio urubots_retries
        utec_auto="$(nmcli -g connection.autoconnect connection show "UTEC-Invitados" 2>/dev/null || echo "no-existe")"
        if [ "$utec_auto" = "no-existe" ]; then
            ok "Perfil abierto 'UTEC-Invitados' no existe"
        elif [ "$utec_auto" = "no" ]; then
            ok "Perfil abierto 'UTEC-Invitados' tiene autoconnect=no (no robara la conexion a Urubots_5G)"
        else
            warn "Perfil abierto 'UTEC-Invitados' tiene autoconnect=$utec_auto (provoco cambio de red y corte en el log)"
        fi

        urubots_auto="$(nmcli -g connection.autoconnect connection show "Urubots_5G" 2>/dev/null || echo "no-existe")"
        urubots_prio="$(nmcli -g connection.autoconnect-priority connection show "Urubots_5G" 2>/dev/null || echo "?")"
        urubots_retries="$(nmcli -g connection.autoconnect-retries connection show "Urubots_5G" 2>/dev/null || echo "?")"
        if [ "$urubots_auto" = "yes" ] && [ "${urubots_prio:-0}" -ge 100 ] 2>/dev/null && [ "$urubots_retries" = "0" ]; then
            ok "Perfil 'Urubots_5G': autoconnect=yes, priority=$urubots_prio, retries=0 (infinitos)"
        elif [ "$urubots_auto" = "no-existe" ]; then
            info "Perfil 'Urubots_5G' no encontrado en nmcli"
        else
            warn "Perfil 'Urubots_5G': autoconnect=$urubots_auto, priority=$urubots_prio, retries=$urubots_retries (recomendado: yes, >=100, 0)"
        fi
    else
        info "nmcli no disponible para inspeccionar perfiles"
    fi
    echo

    echo "3. Keepalive TCP del kernel (mitigacion de timeouts de NAT/firewall a los 15 min)"
    local ka_time ka_intvl ka_probes
    ka_time="$(sysctl -n net.ipv4.tcp_keepalive_time 2>/dev/null || echo "7200")"
    ka_intvl="$(sysctl -n net.ipv4.tcp_keepalive_intvl 2>/dev/null || echo "75")"
    ka_probes="$(sysctl -n net.ipv4.tcp_keepalive_probes 2>/dev/null || echo "9")"
    if [ -f "$SYSCTL_FILE" ] && [ "$ka_time" -le 300 ] 2>/dev/null; then
        ok "Keepalive TCP activo: time=${ka_time}s, intvl=${ka_intvl}s, probes=${ka_probes} ($SYSCTL_FILE)"
    else
        warn "Keepalive TCP en valores altos: time=${ka_time}s (>900s de corte observado); falta $SYSCTL_FILE"
    fi
    echo

    echo "4. Sesion grafica X11, GDM y bloqueo de pantalla (usuario: $TARGET_USER)"
    if [ -r "$GDM_CONF" ] && grep -Eq '^[[:space:]]*WaylandEnable[[:space:]]*=[[:space:]]*false' "$GDM_CONF"; then
        ok "GDM con WaylandEnable=false en $GDM_CONF (X11 activo)"
    else
        warn "No se confirmo WaylandEnable=false en $GDM_CONF"
    fi

    local lock_enabled idle_delay sleep_ac
    lock_enabled="$(run_gsettings_get org.gnome.desktop.screensaver lock-enabled)"
    idle_delay="$(run_gsettings_get org.gnome.desktop.session idle-delay)"
    sleep_ac="$(run_gsettings_get org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type)"

    if [ "$lock_enabled" = "false" ]; then
        ok "Bloqueo de pantalla GNOME desactivado (lock-enabled=false)"
    else
        warn "screensaver lock-enabled=${lock_enabled:-desconocido} (si se bloquea la sesion impide reconectar sin desbloqueo)"
    fi
    info "GNOME idle-delay=${idle_delay:-?}, sleep-inactive-ac-type=${sleep_ac:-?}"
    echo

    echo "5. Respaldo y configuracion manual en GUI"
    if [ -f "$ORIG_DIR/.initialized" ]; then
        info "Respaldo previo disponible en: $ORIG_DIR"
    else
        info "Sin respaldo aplicado aun en: $ORIG_DIR"
    fi
    info "Acceso desatendido (Unattended Access): verificar manualmente en la interfaz de AnyDesk (ver docs/ANYDESK_DIAGNOSTICO.txt)."
    echo "=== Resumen: $OK_COUNT OK, $WARN_COUNT pendientes ==="
    return 0
}

require_root() {
    local action="$1"
    if [ "${EUID:-$(id -u)}" -ne 0 ]; then
        err "$action requiere privilegios de root (sudo)."
        err "No se realizo ningun cambio. Ejecute con sudo cuando este en el lugar."
        exit 1
    fi
}

save_initial_backup() {
    mkdir -p "$ORIG_DIR"
    if [ -f "$ORIG_DIR/.initialized" ]; then
        info "Respaldo original ya existe en $ORIG_DIR (se conserva intacto para idempotencia)."
        return 0
    fi

    info "Creando respaldo inicial en $ORIG_DIR ..."
    date -Is > "$ORIG_DIR/timestamp.txt"

    if [ -f "$DROPIN_FILE" ]; then
        cp -a "$DROPIN_FILE" "$ORIG_DIR/base-athome-estable.conf.bak"
        echo "present" > "$ORIG_DIR/dropin.state"
    else
        echo "absent" > "$ORIG_DIR/dropin.state"
    fi

    if [ -f "$NM_POWERSAVE_FILE" ]; then
        cp -a "$NM_POWERSAVE_FILE" "$ORIG_DIR/90-base-athome-wifi-powersave-off.conf.bak"
        echo "present" > "$ORIG_DIR/nm_powersave.state"
    else
        echo "absent" > "$ORIG_DIR/nm_powersave.state"
    fi

    if [ -f "$SYSCTL_FILE" ]; then
        cp -a "$SYSCTL_FILE" "$ORIG_DIR/90-base-athome-anydesk-keepalive.conf.bak"
        echo "present" > "$ORIG_DIR/sysctl.state"
    else
        echo "absent" > "$ORIG_DIR/sysctl.state"
    fi

    {
        echo "net.ipv4.tcp_keepalive_time=$(sysctl -n net.ipv4.tcp_keepalive_time 2>/dev/null || echo 7200)"
        echo "net.ipv4.tcp_keepalive_intvl=$(sysctl -n net.ipv4.tcp_keepalive_intvl 2>/dev/null || echo 75)"
        echo "net.ipv4.tcp_keepalive_probes=$(sysctl -n net.ipv4.tcp_keepalive_probes 2>/dev/null || echo 9)"
    } > "$ORIG_DIR/sysctl_values.env"

    if command -v nmcli >/dev/null 2>&1; then
        nmcli -g connection.autoconnect connection show "UTEC-Invitados" 2>/dev/null > "$ORIG_DIR/nm_utec_autoconnect.val" || rm -f "$ORIG_DIR/nm_utec_autoconnect.val"
        if nmcli -g UUID connection show "Urubots_5G" >/dev/null 2>&1; then
            nmcli -g connection.autoconnect connection show "Urubots_5G" > "$ORIG_DIR/nm_urubots_autoconnect.val" 2>/dev/null || true
            nmcli -g connection.autoconnect-priority connection show "Urubots_5G" > "$ORIG_DIR/nm_urubots_priority.val" 2>/dev/null || true
            nmcli -g connection.autoconnect-retries connection show "Urubots_5G" > "$ORIG_DIR/nm_urubots_retries.val" 2>/dev/null || true
            nmcli -g 802-11-wireless.powersave connection show "Urubots_5G" > "$ORIG_DIR/nm_urubots_powersave.val" 2>/dev/null || true
        fi
    fi

    local lock_val ulock_val
    lock_val="$(run_gsettings_get org.gnome.desktop.screensaver lock-enabled)"
    ulock_val="$(run_gsettings_get org.gnome.desktop.screensaver ubuntu-lock-on-suspend)"
    [ -n "$lock_val" ] && echo "$lock_val" > "$ORIG_DIR/gsettings_lock_enabled.val"
    [ -n "$ulock_val" ] && echo "$ulock_val" > "$ORIG_DIR/gsettings_ubuntu_lock.val"

    touch "$ORIG_DIR/.initialized"
    ok "Respaldo inicial guardado en $ORIG_DIR"
}

write_file_idempotent() {
    local target="$1" content="$2"
    mkdir -p "$(dirname "$target")"
    if [ -f "$target" ] && [ "$(cat "$target")" = "$content" ]; then
        info "Sin cambios en $target (ya estaba aplicado)"
        return 1
    fi
    printf '%s\n' "$content" > "$target"
    chmod 0644 "$target"
    ok "Actualizado $target"
    return 0
}

apply_changes() {
    require_root "--apply"
    echo "=== HOME-66: Aplicando configuracion estable para AnyDesk (sin reiniciar servicios) ==="
    save_initial_backup

    # 1. Drop-in de systemd para anydesk.service
    local dropin_content
    dropin_content="$(cat <<'EOF'
# Gestionado por scripts/anydesk_estable.sh (HOME-66)
[Unit]
Wants=network-online.target
After=network-online.target systemd-user-sessions.service
StartLimitIntervalSec=0

[Service]
PIDFile=/run/anydesk.pid
Restart=always
RestartSec=5
EOF
)"
    if write_file_idempotent "$DROPIN_FILE" "$dropin_content"; then
        systemctl daemon-reload
        ok "systemctl daemon-reload ejecutado (anydesk.service NO fue reiniciado)"
    fi

    # 2. Desactivar ahorro de energia Wi-Fi en NetworkManager
    local nm_content
    nm_content="$(cat <<'EOF'
# Gestionado por scripts/anydesk_estable.sh (HOME-66)
# 2 = desactiva power save en interfaces Wi-Fi (iwlwifi wlo1)
[connection]
wifi.powersave = 2
EOF
)"
    write_file_idempotent "$NM_POWERSAVE_FILE" "$nm_content" || true

    # 3. Ajustar perfiles Wi-Fi en disco sin cortar la conexion activa y sin tocar enp1s0
    if command -v nmcli >/dev/null 2>&1; then
        local ctype cdev
        ctype="$(nmcli -g connection.type connection show "UTEC-Invitados" 2>/dev/null || true)"
        cdev="$(nmcli -g connection.interface-name connection show "UTEC-Invitados" 2>/dev/null || true)"
        if [ "$ctype" = "802-11-wireless" ] && [ "$cdev" != "$PROTECTED_IF" ]; then
            nmcli connection modify "UTEC-Invitados" connection.autoconnect no
            ok "Perfil Wi-Fi 'UTEC-Invitados' fijado en connection.autoconnect=no (en disco)"
        fi

        ctype="$(nmcli -g connection.type connection show "Urubots_5G" 2>/dev/null || true)"
        cdev="$(nmcli -g connection.interface-name connection show "Urubots_5G" 2>/dev/null || true)"
        if [ "$ctype" = "802-11-wireless" ] && [ "$cdev" != "$PROTECTED_IF" ]; then
            nmcli connection modify "Urubots_5G" \
                connection.autoconnect yes \
                connection.autoconnect-priority 100 \
                connection.autoconnect-retries 0 \
                802-11-wireless.powersave 2
            ok "Perfil Wi-Fi 'Urubots_5G' priorizado (autoconnect=yes, priority=100, retries=0, powersave=2)"
        fi
    fi

    # 4. Keepalive TCP
    local sysctl_content
    sysctl_content="$(cat <<'EOF'
# Gestionado por scripts/anydesk_estable.sh (HOME-66)
net.ipv4.tcp_keepalive_time = 120
net.ipv4.tcp_keepalive_intvl = 30
net.ipv4.tcp_keepalive_probes = 6
EOF
)"
    write_file_idempotent "$SYSCTL_FILE" "$sysctl_content" || true
    sysctl -w net.ipv4.tcp_keepalive_time=120 net.ipv4.tcp_keepalive_intvl=30 net.ipv4.tcp_keepalive_probes=6 >/dev/null
    ok "Keepalive TCP activo en kernel (120s / 30s / 6 probes)"

    # 5. Desactivar bloqueo de pantalla GNOME
    if run_gsettings_set org.gnome.desktop.screensaver lock-enabled false; then
        run_gsettings_set org.gnome.desktop.screensaver ubuntu-lock-on-suspend false || true
        ok "Bloqueo de pantalla GNOME desactivado (lock-enabled=false) para $TARGET_USER"
    else
        info "No se pudo contactar el bus D-Bus de $TARGET_USER; ejecute en sesion grafica:"
        info "  gsettings set org.gnome.desktop.screensaver lock-enabled false"
    fi

    echo
    echo "AVISO IMPORTANTE:"
    echo "  - NO se reinicio AnyDesk ni NetworkManager para evitar cortar la sesion remota."
    echo "  - El drop-in de systemd (Restart=always) ya quedo cargado con daemon-reload."
    echo "  - Cuando este fisicamente frente a la NUC, configure el Acceso Desatendido en la"
    echo "    interfaz de AnyDesk (ver docs/ANYDESK_DIAGNOSTICO.txt) y, si desea aplicar en"
    echo "    caliente el ahorro de energia Wi-Fi, reconecte Wi-Fi localmente."
}

revert_changes() {
    require_root "--revert"
    echo "=== HOME-66: Revirtiendo cambios de AnyDesk desde $ORIG_DIR ==="
    if [ ! -f "$ORIG_DIR/.initialized" ]; then
        err "No existe un respaldo inicial en $ORIG_DIR. Nada que revertir."
        exit 1
    fi

    if [ "$(cat "$ORIG_DIR/dropin.state" 2>/dev/null)" = "present" ] && [ -f "$ORIG_DIR/base-athome-estable.conf.bak" ]; then
        cp -a "$ORIG_DIR/base-athome-estable.conf.bak" "$DROPIN_FILE"
    else
        rm -f "$DROPIN_FILE"
        rmdir --ignore-fail-on-non-empty "$DROPIN_DIR" 2>/dev/null || true
    fi
    systemctl daemon-reload
    ok "Revertido drop-in de anydesk.service (daemon-reload ejecutado sin reiniciar el servicio)"

    if [ "$(cat "$ORIG_DIR/nm_powersave.state" 2>/dev/null)" = "present" ] && [ -f "$ORIG_DIR/90-base-athome-wifi-powersave-off.conf.bak" ]; then
        cp -a "$ORIG_DIR/90-base-athome-wifi-powersave-off.conf.bak" "$NM_POWERSAVE_FILE"
    else
        rm -f "$NM_POWERSAVE_FILE"
    fi
    ok "Revertido $NM_POWERSAVE_FILE"

    if command -v nmcli >/dev/null 2>&1; then
        if [ -f "$ORIG_DIR/nm_utec_autoconnect.val" ]; then
            nmcli connection modify "UTEC-Invitados" connection.autoconnect "$(cat "$ORIG_DIR/nm_utec_autoconnect.val")" 2>/dev/null || true
        fi
        if [ -f "$ORIG_DIR/nm_urubots_autoconnect.val" ]; then
            nmcli connection modify "Urubots_5G" \
                connection.autoconnect "$(cat "$ORIG_DIR/nm_urubots_autoconnect.val")" \
                connection.autoconnect-priority "$(cat "$ORIG_DIR/nm_urubots_priority.val" 2>/dev/null || echo 0)" \
                connection.autoconnect-retries "$(cat "$ORIG_DIR/nm_urubots_retries.val" 2>/dev/null || echo -1)" 2>/dev/null || true
        fi
        ok "Revertidas propiedades de perfiles Wi-Fi en NetworkManager"
    fi

    if [ "$(cat "$ORIG_DIR/sysctl.state" 2>/dev/null)" = "present" ] && [ -f "$ORIG_DIR/90-base-athome-anydesk-keepalive.conf.bak" ]; then
        cp -a "$ORIG_DIR/90-base-athome-anydesk-keepalive.conf.bak" "$SYSCTL_FILE"
    else
        rm -f "$SYSCTL_FILE"
    fi
    if [ -f "$ORIG_DIR/sysctl_values.env" ]; then
        while IFS='=' read -r k v; do
            [ -n "$k" ] && [ -n "$v" ] && sysctl -w "$k=$v" >/dev/null 2>&1 || true
        done < "$ORIG_DIR/sysctl_values.env"
    fi
    ok "Revertidos parametros sysctl de keepalive TCP"

    if [ -f "$ORIG_DIR/gsettings_lock_enabled.val" ]; then
        run_gsettings_set org.gnome.desktop.screensaver lock-enabled "$(cat "$ORIG_DIR/gsettings_lock_enabled.val")" || true
    fi
    if [ -f "$ORIG_DIR/gsettings_ubuntu_lock.val" ]; then
        run_gsettings_set org.gnome.desktop.screensaver ubuntu-lock-on-suspend "$(cat "$ORIG_DIR/gsettings_ubuntu_lock.val")" || true
    fi
    ok "Revertida configuracion de bloqueo de pantalla GNOME"
    echo "AVISO: No se reinicio AnyDesk ni NetworkManager."
}

case "$MODE" in
    --check)
        check_status
        ;;
    --apply)
        apply_changes
        ;;
    --revert)
        revert_changes
        ;;
    -h|--help)
        usage
        ;;
    *)
        err "Opcion no reconocida: $MODE"
        usage >&2
        exit 2
        ;;
esac
