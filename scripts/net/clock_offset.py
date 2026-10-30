#!/usr/bin/env python3
"""Mide el desfase de reloj NUC->Jetson estilo NTP sobre una sola sesion SSH.

Uso: clock_offset.py usuario@host   -> imprime "offset_ms rtt_ms" (muestra de menor RTT)
"""
import shlex
import subprocess
import sys
import time

REMOTE = "import sys,time\nfor _ in sys.stdin: print(repr(time.time()), flush=True)"


def main() -> int:
    proc = subprocess.Popen(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", sys.argv[1],
         "python3 -u -c " + shlex.quote(REMOTE)],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    samples = []
    for _ in range(10):
        t0 = time.time()
        proc.stdin.write("x\n")
        proc.stdin.flush()
        remote = float(proc.stdout.readline())
        t1 = time.time()
        samples.append((t1 - t0, remote - (t0 + t1) / 2))
    proc.stdin.close()
    proc.wait(timeout=5)
    rtt, offset = min(samples)
    print(f"{offset * 1000:.1f} {rtt * 1000:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
