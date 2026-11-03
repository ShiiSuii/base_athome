#!/usr/bin/env python3
"""Hook PreToolUse de Antigravity: aprobacion automatica ACOTADA (AGENTS.md §6).

Aprueba solo comandos inocuos (inspeccion Git, git add/commit, lectura de archivos, pytest, colcon,
xacro) ejecutados dentro de los worktrees de Base@Home. Todo lo demas -> "ask" (decide el humano);
nunca "deny" para no trabar al humano. No es un auto-aprobador ciego: cada segmento de una linea
compuesta (&&, ||, ;, |) tiene que estar en la lista; sustitucion de comandos, redirecciones a
archivos, sudo, ssh, red, borrados, push y el dominio ROS 10 siempre preguntan.
Registro de decisiones: ~/.gemini/config/agy_bounded_allow.log
"""
import json
import os
import re
import shlex
import sys
import time

WORKSPACE_ROOTS = ("/home/athome/base_athome_ws", "/home/athome/orca/workspaces/base_athome_ws/")
LOG = os.path.expanduser("~/.gemini/config/agy_bounded_allow.log")

READ_ONLY = {"ls", "cat", "head", "tail", "wc", "grep", "rg", "pwd", "echo", "which", "tree", "file", "stat",
             "diff", "sort", "uniq", "cut", "basename", "dirname", "realpath", "true", "date", "md5sum", "sha256sum"}
GIT_OK = {"status", "diff", "log", "show", "rev-parse", "ls-files", "grep", "add", "commit", "blame",
          "describe", "shortlog", "cat-file", "merge-base", "check-ignore"}
GIT_LIST_ONLY = {"branch": {"--show-current", "-a", "-v", "-vv", "--list", "-r"},
                 "stash": {"list"}, "worktree": {"list"}, "remote": set()}  # remote: nunca (URL con token)
FORBIDDEN_TOKENS = re.compile(r"`|\$\(|<\(|>\(|\bsudo\b|\bssh\b|\bscp\b|\brsync\b|\bcurl\b|\bwget\b|"
                              r"\brm\b|\bmv\b|\bchmod\b|\bchown\b|\bkill\b|\bpkill\b|\bsystemctl\b|"
                              r"ROS_DOMAIN_ID=10\b|\bpip3?\s+install\b|--force\b|\bpush\b|\breset\b|\bclean\b|"
                              r"\brebase\b|\bdd\b|\bmkfs\b|\.env\b|\bgh\b")
REDIRECT_OK = re.compile(r"\s*(2>&1|>\s*/dev/null|2>\s*/dev/null)\s*")


def _segment_ok(seg: str) -> bool:
    seg = REDIRECT_OK.sub(" ", seg).strip()
    if not seg:
        return True
    if re.search(r"[<>]", seg):
        return False  # escritura/lectura de archivos por redireccion
    try:
        argv = shlex.split(seg)
    except ValueError:
        return False
    while argv and re.fullmatch(r"[A-Z_][A-Z0-9_]*=[^ ]*", argv[0]):
        name, val = argv[0].split("=", 1)
        if name == "ROS_DOMAIN_ID" and val not in ("42", "77"):
            return False
        argv = argv[1:]
    if argv[:2] == ["env", "-u"]:
        argv = argv[3:]
    if not argv:
        return True
    cmd, args = os.path.basename(argv[0]), argv[1:]
    if cmd in READ_ONLY:
        return True
    if cmd == "find":
        return not any(a in ("-exec", "-execdir", "-delete", "-ok", "-fprint", "-fls") for a in args)
    if cmd in ("cd", "source", "."):
        target = args[0] if args else ""
        return cmd == "cd" and not target.startswith("/") or target.startswith("/opt/ros/jazzy/") \
            or target.endswith("install/setup.bash") or target.startswith(WORKSPACE_ROOTS)
    if cmd == "mkdir":
        return all(not a.startswith("/") and ".." not in a for a in args if a != "-p")
    if cmd == "git":
        if not args:
            return False
        sub = args[0]
        if sub in GIT_OK:
            return True
        if sub in GIT_LIST_ONLY:
            return len(args) == 1 and sub != "remote" or (len(args) > 1 and set(args[1:]) <= GIT_LIST_ONLY[sub])
        return False
    if cmd in ("pytest", "py.test"):
        return True
    if re.fullmatch(r"python3?(\.\d+)?", cmd):
        return args[:2] == ["-m", "pytest"] or args[:2] == ["-m", "py_compile"] or args[:1] == ["--version"]
    if cmd == "colcon":
        return bool(args) and args[0] in ("build", "test", "test-result", "list", "info")
    if cmd in ("xacro", "check_urdf"):
        return True
    if cmd == "ros2":
        return len(args) >= 2 and args[0] in ("pkg", "interface") and args[1] in ("list", "show", "prefix", "xml")
    return False


def decide(command: str, workspaces) -> dict:
    inside = any(str(w).startswith(WORKSPACE_ROOTS) for w in (workspaces or []))
    if not inside:
        return {"decision": "ask", "reason": "fuera de los worktrees de Base@Home"}
    if FORBIDDEN_TOKENS.search(command):
        return {"decision": "ask", "reason": "contiene una operacion que requiere confirmacion humana"}
    segments = re.split(r"\s*(?:&&|\|\||;|\|)\s*", command.strip())
    if all(_segment_ok(s) for s in segments):
        # El hook de Orca responde siempre "ask" y gana la decision mas restrictiva; por eso ademas se
        # otorga un permiso temporal para ESTE comando exacto (cada token se compara como regex anclada).
        grant = "command(" + " ".join(re.escape(t) for t in command.strip().split()) + ")"
        return {"decision": "allow", "reason": "operacion inocua (lista acotada de AGENTS.md §6)",
                "permissionOverrides": [grant]}
    return {"decision": "ask", "reason": "fuera de la lista acotada"}


def main():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        print(json.dumps({"decision": "ask"}))
        return
    call = payload.get("toolCall") or {}
    if call.get("name") != "run_command":
        print(json.dumps({"decision": "ask"}))
        return
    cmd = str((call.get("args") or {}).get("CommandLine", ""))
    out = decide(cmd, payload.get("workspacePaths"))
    try:
        with open(LOG, "a") as f:
            f.write(json.dumps({"t": time.strftime("%Y-%m-%dT%H:%M:%S"), "decision": out["decision"],
                                "cmd": cmd[:300]}) + "\n")
    except OSError:
        pass
    print(json.dumps(out))


if __name__ == "__main__":
    main()
