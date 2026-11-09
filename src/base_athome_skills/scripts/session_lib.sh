#!/bin/bash
# Cada descendiente hereda BASE_ATHOME_SESSION=<pid del script>.
# Solo se senalizan procesos con esa marca; nunca se buscan comandos por nombre.
SESSION_PIDS=()
SESSION_ORPHANS_STOPPED=0

session_stop_marked() {  # <pid de sesion | orphans>; imprime cuantos encontro
  python3 - "$1" "$$" "${SESSION_TERM_TIMEOUT:-10}" <<'PYTHON'
import os
from pathlib import Path
import signal
import sys
import time

mode, owner, grace = sys.argv[1:]

def live(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return False

def marked():
    result = {}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or proc.name in (owner, str(os.getpid())):
            continue
        try:
            if proc.stat().st_uid != os.getuid() or not live(proc.name):
                continue
            # No se imprime ni conserva ningun otro campo del entorno.
            sid = next((v.split(b"=", 1)[1].decode() for v in
                        (proc / "environ").read_bytes().split(b"\0")
                        if v.startswith(b"BASE_ATHOME_SESSION=")), "")
            if sid.isdigit():
                result[int(proc.name)] = sid
        except (OSError, UnicodeError):
            continue
    return result

initial = marked()
sessions = ({sid for sid in initial.values() if sid != owner and not live(sid)}
            if mode == "orphans" else {mode})
seen = set()

def remaining():
    return {pid: sid for pid, sid in marked().items() if sid in sessions}

def send(pid, sid, sig):
    # El pidfd evita senalizar otro proceso si se recicla el PID entre inspeccion y kill.
    try:
        fd = os.pidfd_open(pid)
        try:
            if marked().get(pid) == sid:
                signal.pidfd_send_signal(fd, sig)
        finally:
            os.close(fd)
    except ProcessLookupError:
        pass

end = time.monotonic() + float(grace)
while True:
    active = remaining()
    for pid, sid in active.items():
        if pid not in seen:
            send(pid, sid, signal.SIGTERM)
            seen.add(pid)
    if not active or time.monotonic() >= end:
        break
    time.sleep(0.1)
# Incluye descendientes que se separaron del grupo (setsid) y nuevos hijos durante TERM.
end = time.monotonic() + 2.0
while active:
    for pid, sid in active.items():
        send(pid, sid, signal.SIGKILL)
    time.sleep(0.05)
    active = remaining()
    if time.monotonic() >= end:
        break
if active:
    print("AVISO: procesos de sesion aun vivos: " + ", ".join(map(str, active)), file=sys.stderr)
    sys.exit(1)
print(len(seen))
PYTHON
}

session_stop_orphans() {
  SESSION_ORPHANS_STOPPED=$(session_stop_marked orphans) || return 1
  if [ "$SESSION_ORPHANS_STOPPED" -gt 0 ]; then
    echo "Cerre $SESSION_ORPHANS_STOPPED procesos de una sesion anterior que quedaron abiertos."
  fi
}

session_cleanup() {
  local status=$?
  trap - EXIT
  trap '' HUP INT TERM   # otro Ctrl-C no debe interrumpir la limpieza
  local pid
  if session_stop_marked "$$" >/dev/null; then
    for pid in "${SESSION_PIDS[@]}"; do
      # Bash recoge los hijos muertos; nunca esperar sin limite uno que perdio la marca.
      kill -0 "$pid" 2>/dev/null || wait "$pid" 2>/dev/null || :
    done
  else
    status=1
  fi
  [ -n "${SESSION_LOG:-}" ] && echo "logs: $SESSION_LOG"
  exit "$status"
}

session_init() {   # session_init <carpeta de logs>
  SESSION_LOG=$1; mkdir -p "$SESSION_LOG"
  export BASE_ATHOME_SESSION=$$
  trap session_cleanup EXIT
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM
  session_stop_orphans
}

session_start() {  # session_start <nombre> <comando...>
  local name=$1; shift
  setsid "$@" > "$SESSION_LOG/$name.log" 2>&1 &
  SESSION_PIDS+=("$!")
}

session_run_terminal() {  # nodo visible, con stdin de la terminal y wait interrumpible
  # <&0 explicito: Bash redirige stdin a /dev/null en tareas asincronas sin job control.
  setsid "$@" <&0 &
  SESSION_PIDS+=("$!")
  wait "${SESSION_PIDS[-1]}"
}
