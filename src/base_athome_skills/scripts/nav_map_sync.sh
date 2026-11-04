#!/bin/bash
# Hace que Nav2 (ya corriendo) use el mapa guardado en disco: map_server solo lo lee al arrancar.
# Requiere el entorno ROS cargado. rc=0 sincronizado; 1 Nav2 usa otro archivo o la recarga fallo; 2 Nav2 no corre.
MAP=${1:-$HOME/mapa_base_athome.yaml}
current=$(timeout 8 ros2 param get /map_server yaml_filename 2>/dev/null | sed -n 's/^String value is: //p')
if [ -z "$current" ]; then echo "Nav2 no esta corriendo: va a leer $MAP al arrancar."; exit 2; fi
if [ "$(readlink -f "$current")" != "$(readlink -f "$MAP")" ]; then
  echo "AVISO: Nav2 tiene abierto $current, no $MAP."; exit 1
fi
if timeout 20 ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$MAP'}" 2>/dev/null | grep -q 'result=0'; then
  echo "Nav2 recargo el mapa guardado: $MAP"; exit 0
fi
echo "AVISO: Nav2 no pudo recargar $MAP."; exit 1
