#!/usr/bin/env bash
# Abre el tablero de debug. Lo arranca si no esta corriendo; solo muestra, no da ordenes al robot.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${DEBUG_PORT:-8780}"
URL="http://127.0.0.1:${PORT}"
if ! ss -ltn | grep -q "127.0.0.1:${PORT} "; then
  LOG="${HOME}/.local/share/base_athome/debug_dashboard.log"
  mkdir -p "$(dirname "$LOG")"
  setsid env -u PYTHONPATH /usr/bin/python3 "$HERE/server.py" >"$LOG" 2>&1 < /dev/null &
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    ss -ltn | grep -q "127.0.0.1:${PORT} " && break
    sleep 0.3
  done
fi
echo "Tablero: $URL"
xdg-open "$URL" >/dev/null 2>&1 &
