#!/usr/bin/env python3
"""Auto-aprobador acotado de prompts de permiso de Antigravity en un worker de Orca.

Uso: agy_autoapprove.py <dispatch_id> <terminal_handle> <worktree_path> <minutos>

Aprueba solo comandos con prefijos seguros (opcion 1 = "Yes, run") y creacion/edicion
de archivos dentro del worktree. Todo lo demas queda esperando y se registra en el log
para que lo revise el coordinador. Nunca aprueba rm, sudo, ssh, red, ros2, push, etc.
"""
import json
import os
import re
import subprocess
import sys
import time

ORCA = os.path.expanduser('~/.config/orca/linux-orca-cli-shim/orca')
SAFE = re.compile(
    r'^(PYTHONPATH="[^"]*" |ROS_DOMAIN_ID=7[0-9] |orca orchestration|'
    r'git (status|log|diff|add|commit|show|rev-parse|branch|checkout -b|switch)|'
    r'ls|cat|head|tail|wc|mkdir|cd |pwd|echo|grep|find|python3?|[\w/.~-]*\.venv/bin/|'
    r'~/ros2_vision_venv/bin/|/home/athome/ros2_vision_venv/bin/|pip3? install|uv |pytest|source |'
    r'bash scripts/|chmod \+x|test |\[ |sed -n|touch |cp |mv |colcon (build|test)|xacro|check_urdf)')
DENY = re.compile(r'\b(rm|sudo|ssh|scp|curl|wget|dd|mkfs|shutdown|reboot|systemctl|ros2 (topic pub|service call|action)|'
                  r'push|reset --hard|clean -f|ROS_DOMAIN_ID=10)\b')
FILE_PROMPT = re.compile(r'Allow (creation|edit|modification|write)|Accept this file edit')


def tail(dispatch):
    out = subprocess.run([ORCA, 'orchestration', 'worker-read', '--dispatch', dispatch, '--source', 'auto', '--json'],
                         capture_output=True, text=True, timeout=30).stdout
    try:
        res = json.loads(out)['result']
    except Exception:
        return None, None
    status = (res.get('projection') or {}).get('outcome')
    return '\n'.join(map(str, (res.get('terminal') or {}).get('tail') or [])), status


def send(term, key):
    subprocess.run([ORCA, 'terminal', 'send', '--terminal', term, '--text', key, '--json'],
                   capture_output=True, timeout=30)


def log(msg):
    print(f'{time.strftime("%H:%M:%S")} {msg}', flush=True)


def main():
    dispatch, term, worktree, minutes = sys.argv[1], sys.argv[2], sys.argv[3].split('@@')[0], float(sys.argv[4])
    end = time.time() + minutes * 60
    last, last_at = None, 0.0
    while time.time() < end:
        text, status = tail(dispatch)
        if status and status != 'in_progress':
            log(f'worker {status}; fin')
            return
        if text:
            screen = text[-2500:]
            if 'Run this command?' in screen:
                # El comando puede venir partido en varias lineas si la terminal es angosta
                m = re.findall(r'Requesting permission for:\s*\n(.*?)\n\s*Run this command\?', screen, re.S)
                cmd = ' '.join(l.strip() for l in m[-1].splitlines() if l.strip()) if m else ''
                ok = bool(cmd) and bool(SAFE.match(cmd)) and not DENY.search(cmd)
                key = ('cmd', cmd)
                if key == last and ok and time.time() - last_at > 30:
                    send(term, '\r')
                    log(f'REINTENTO: {cmd}')
                    last_at = time.time()
                elif key != last:
                    if ok:
                        send(term, '1')
                        log(f'APROBADO: {cmd}')
                        last_at = time.time()
                    else:
                        log(f'PENDIENTE (no aprobado): {cmd}')
                    last = key
            elif FILE_PROMPT.search(screen):
                paths = re.findall(r'(/[\w./!@-]+)', screen)
                inside = all(p.startswith(worktree) or (len(p) > 30 and worktree.startswith(p))
                             or not p.startswith('/home') for p in paths[-5:])
                key = ('file', screen[-300:])
                if key != last:
                    if inside:
                        send(term, '1')
                        log('ARCHIVO APROBADO')
                    else:
                        log('ARCHIVO PENDIENTE (fuera del worktree?)')
                    last = key
            else:
                last = None
        time.sleep(8)


if __name__ == '__main__':
    main()
