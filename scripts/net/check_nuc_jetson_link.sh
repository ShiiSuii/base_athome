#!/bin/bash
# HOME-01E: diagnostico del enlace Ethernet NUC <-> Jetson. Ejecutar en la NUC.
# Solo lectura: no modifica red, servicios ni configuracion. La prueba DDS usa un
# dominio aislado (TEST_DOMAIN, default 77) y un topico sintetico, nunca el 10.
#
#   scripts/net/check_nuc_jetson_link.sh            # reporte en pantalla + archivo
#   JETSON_IP=10.42.0.12 TEST_DOMAIN=77 scripts/net/check_nuc_jetson_link.sh
set -u

NUC_IF="${NUC_IF:-enp1s0}"
# Vacio = autodetectar la interfaz que tiene JETSON_IP (hoy el USB RTL8153; enP8p1s0 sin carrier)
JETSON_IF="${JETSON_IF:-}"
JETSON_IP="${JETSON_IP:-10.42.0.12}"
JETSON_USER="${JETSON_USER:-ubuntu}"
TEST_DOMAIN="${TEST_DOMAIN:-77}"
OUT="${OUT:-$HOME/base_athome_ws/log/net_checks/$(date +%Y%m%d_%H%M%S).txt}"
# ControlMaster: reusar una conexion para que la medicion de reloj no incluya el handshake
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=8 -o ControlMaster=auto
     -o ControlPath="/tmp/.home01e-%r@%h" -o ControlPersist=120 "$JETSON_USER@$JETSON_IP")

[ "$TEST_DOMAIN" = "10" ] && { echo "TEST_DOMAIN=10 prohibido (dominio operativo)"; exit 2; }
mkdir -p "$(dirname "$OUT")"
exec > >(tee "$OUT") 2>&1

PASS=0; FAIL=0
ok()   { echo "  [OK]   $*"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
info() { echo "  [..]   $*"; }

echo "=== HOME-01E $(date -Is) — NUC $(hostname) ==="

echo "1. Enlace fisico en la NUC ($NUC_IF)"
if [ "$(cat /sys/class/net/$NUC_IF/carrier 2>/dev/null)" = "1" ]; then
    ok "carrier=1, velocidad $(cat /sys/class/net/$NUC_IF/speed 2>/dev/null) Mb/s, MTU $(cat /sys/class/net/$NUC_IF/mtu)"
else
    bad "sin carrier en $NUC_IF: revisar cable/puerto. Nada mas puede pasar."; echo "Reporte: $OUT"; exit 1
fi
info "$(ip -br addr show "$NUC_IF")"
ip -4 addr show "$NUC_IF" | grep -q "inet 10.42.0.1/" && ok "IP 10.42.0.1 presente" || bad "falta 10.42.0.1 (perfil NM 'Wired connection 1' shared?)"

echo "2. Alcance IP"
info "ruta: $(ip route get "$JETSON_IP" 2>&1 | head -1)"
if ping -c 5 -i 0.3 -W 2 "$JETSON_IP" > /tmp/.ping.$$ 2>&1; then
    ok "$(tail -2 /tmp/.ping.$$ | tr '\n' ' ')"
else
    bad "ping a $JETSON_IP falla"; tail -3 /tmp/.ping.$$
    info "vecinos: $(ip neigh show dev "$NUC_IF" | tr '\n' ';')"
fi
rm -f /tmp/.ping.$$

echo "3. SSH (clave ya autorizada; host key debe coincidir con known_hosts)"
if "${SSH[@]}" true 2>/tmp/.ssh.$$; then ok "SSH OK"; else bad "SSH falla: $(cat /tmp/.ssh.$$)"; rm -f /tmp/.ssh.$$; echo "Reporte: $OUT"; exit 1; fi
rm -f /tmp/.ssh.$$

echo "4. Estado del lado Jetson"
"${SSH[@]}" "IF=${JETSON_IF:-\$(ip -o -4 addr | awk -v ip=$JETSON_IP 'index(\$4, ip\"/\")==1{print \$2; exit}')}; IF=\${IF:-enP8p1s0}; cat /sys/class/net/\$IF/carrier 2>/dev/null; ip -br addr show \$IF; systemctl is-active jetson-head; systemctl show jetson-head -p NRestarts --value" > /tmp/.j.$$ 2>&1
mapfile -t J < /tmp/.j.$$; rm -f /tmp/.j.$$
[ "${J[0]:-0}" = "1" ] && ok "carrier Jetson=1" || bad "carrier Jetson=${J[0]:-?}"
info "${J[1]:-}"
info "jetson-head: ${J[2]:-?}, reinicios acumulados: ${J[3]:-?}"
info "si jetson-head arranco antes del cable puede estar en modo solo-localhost: reiniciarlo a mano tras revisar"

echo "5. Reloj (TF/timeouts requieren < 50 ms)"
read -r SKEW CLK_RTT < <(python3 "$(dirname "$0")/clock_offset.py" "$JETSON_USER@$JETSON_IP" 2>/dev/null || echo "9999 0")
info "estimacion NTP sobre SSH: rtt ${CLK_RTT} ms"
python3 -c "import sys; sys.exit(0 if abs($SKEW) < 50 else 1)" && ok "desfase ${SKEW} ms" || bad "desfase ${SKEW} ms (revisar NTP/chrony, sin cambiarlo a ciegas)"

echo "6. Configuracion DDS"
info "NUC   : $(grep -oE 'NetworkInterface[^/]*' ~/cyclonedds.xml | head -1); peers $(grep -oE 'Peer address="[^"]+"' ~/cyclonedds.xml | tr '\n' ' ')"
info "Jetson: $("${SSH[@]}" "grep -oE 'NetworkInterface[^/]*' ~/cyclonedds.xml | head -1; grep -oE 'Peer address=\"[^\"]+\"' ~/cyclonedds.xml | tr '\n' ' '")"

echo "7. Prueba DDS aislada (dominio $TEST_DOMAIN, topico /test/eth_link_status)"
NUC_ENV="source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$TEST_DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/cyclonedds.xml"
J_ENV="source /opt/ros/\$(ls /opt/ros | head -1)/setup.bash; export ROS_DOMAIN_ID=$TEST_DOMAIN RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/$JETSON_USER/cyclonedds.xml"
# Jetson publica, NUC escucha
"${SSH[@]}" "$J_ENV; timeout 25 ros2 topic pub -r 2 /test/eth_link_status std_msgs/msg/String '{data: jetson_to_nuc}'" >/dev/null 2>&1 &
PUB=$!
if bash -c "$NUC_ENV; timeout 20 ros2 topic echo --once /test/eth_link_status std_msgs/msg/String" 2>&1 | grep -q jetson_to_nuc; then
    ok "Jetson -> NUC recibido"
else
    bad "Jetson -> NUC no recibido (Humble<->Jazzy o DDS)"
fi
kill $PUB 2>/dev/null; wait $PUB 2>/dev/null
# NUC publica, Jetson escucha
bash -c "$NUC_ENV; timeout 25 ros2 topic pub -r 2 /test/eth_link_status std_msgs/msg/String '{data: nuc_to_jetson}'" >/dev/null 2>&1 &
PUB=$!
if "${SSH[@]}" "$J_ENV; timeout 20 ros2 topic echo --once /test/eth_link_status std_msgs/msg/String" 2>&1 | grep -q nuc_to_jetson; then
    ok "NUC -> Jetson recibido"
else
    bad "NUC -> Jetson no recibido"
fi
kill $PUB 2>/dev/null; wait $PUB 2>/dev/null

echo "8. Throughput (si iperf3 existe en ambos; no se instala nada)"
if command -v iperf3 >/dev/null && "${SSH[@]}" 'command -v iperf3' >/dev/null; then
    "${SSH[@]}" 'timeout 15 iperf3 -s -1' >/dev/null 2>&1 & sleep 1
    info "$(iperf3 -c "$JETSON_IP" -t 5 2>&1 | grep -E 'sender|receiver' | tr -s ' ' | tr '\n' ';')"
else
    info "iperf3 no disponible; omitido"
fi

echo "=== Resultado: $PASS OK, $FAIL FAIL — reporte en $OUT ==="
[ "$FAIL" -eq 0 ]
