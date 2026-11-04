#!/bin/bash
# Procesos de una sesion: se cierran juntos al salir, tambien si se cierra la terminal (HUP).
# Cada proceso lleva la marca BASE_ATHOME_SESSION=<pid del script>; si el script murio sin limpiar (kill -9, corte),
# la sesion siguiente encuentra lo que quedo vivo y lo cierra. Un stack duplicado de Nav2 o de ejecutores no es inofensivo.
SESSION_PIDS=()

session_stop_orphans() {
  local p sid found=0
  for p in $(pgrep -u "$(id -u)" 2>/dev/null); do
    sid=$(tr '\0' '\n' 2>/dev/null < "/proc/$p/environ" | sed -n 's/^BASE_ATHOME_SESSION=//p')
    [ -n "$sid" ] && [ "$sid" != "$$" ] || continue
    [ -d "/proc/$sid" ] && continue
    kill -TERM "$p" 2>/dev/null && found=$((found + 1))
  done
  [ "$found" -gt 0 ] && { echo "Cerre $found procesos de una sesion anterior que quedaron abiertos."; sleep 3; }
  return 0
}

session_cleanup() {
  trap - EXIT HUP INT TERM
  local i g
  for ((i=${#SESSION_PIDS[@]}-1; i>=0; i--)); do kill -TERM -- -"${SESSION_PIDS[i]}" 2>/dev/null; sleep 0.5; done
  for g in "${SESSION_PIDS[@]}"; do
    for _ in $(seq 20); do pgrep -g "$g" >/dev/null 2>&1 || break; sleep 0.5; done
    pgrep -g "$g" >/dev/null 2>&1 && echo "AVISO: quedo abierto el grupo $g"
  done
  [ -n "${SESSION_LOG:-}" ] && echo "logs: $SESSION_LOG"
}

session_init() {   # session_init <carpeta de logs>
  SESSION_LOG=$1; mkdir -p "$SESSION_LOG"
  export BASE_ATHOME_SESSION=$$
  session_stop_orphans
  trap session_cleanup EXIT
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM
}

session_start() {  # session_start <nombre> <comando...>
  local name=$1; shift
  setsid "$@" > "$SESSION_LOG/$name.log" 2>&1 &
  SESSION_PIDS+=($!)
}
