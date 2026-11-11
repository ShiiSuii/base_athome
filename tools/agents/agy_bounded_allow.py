#!/usr/bin/env python3
"""Hook PreToolUse de Antigravity: aprobacion automatica ACOTADA (AGENTS.md §6).

Aprueba solo comandos inocuos (inspeccion Git, git add/commit, lectura de archivos, pytest, colcon,
xacro, graphify query/explain/path y true) dentro de los worktrees de Base@Home.
Todo lo demas -> "ask" (decide el humano);
nunca "deny" para no trabar al humano. No es un auto-aprobador ciego: cada segmento de una linea
compuesta (&&, ||, ;, |) tiene que estar en la lista; sustitucion de comandos, redirecciones a
archivos (salvo /dev/null), sudo, ssh, red, borrados, push y el dominio ROS 10 siempre preguntan.
Solo se aceptan > /dev/null (stdout/stderr) y 2>&1; no se evalua ni ejecuta el shell.
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
             "diff", "sort", "uniq", "cut", "basename", "dirname", "realpath", "true", "date", "md5sum", "sha256sum",
             # Solo lectura del sistema (pedido del usuario 2026-11-11: menos aprobaciones en el laboratorio)
             "ps", "pgrep", "ss", "uptime", "free", "df", "du", "nproc", "lsof", "jq", "column", "tr", "nl", "id",
             "whoami", "hostname", "nvidia-smi", "lscpu"}
# Dominios ROS del laboratorio de simulacion; el 10 (robot real) nunca.
LAB_DOMAINS = {"42", "55", "64", "77", "88", "91", "93"}
GIT_OK = {"status", "diff", "log", "show", "rev-parse", "ls-files", "grep", "add", "commit", "blame",
          "describe", "shortlog", "cat-file", "merge-base", "check-ignore"}
GIT_LIST_ONLY = {"branch": {"--show-current", "-a", "-v", "-vv", "--list", "-r"},
                 "stash": {"list"}, "worktree": {"list"}, "remote": set()}  # remote: nunca (URL con token)
FORBIDDEN_TOKENS = re.compile(r"`|\$\(|<\(|>\(|\bsudo\b|\bssh\b|\bscp\b|\brsync\b|\bcurl\b|\bwget\b|"
                              r"\brm\b|\bmv\b|\bchmod\b|\bchown\b|\bkill\b|\bpkill\b|\bsystemctl\b|"
                              r"ROS_DOMAIN_ID=10\b|\bpip3?\s+install\b|--force\b|\bpush\b|\breset\b|\bclean\b|"
                              r"\brebase\b|\bdd\b|\bmkfs\b|\.env\b|\bgh\b")


def _inside_workspace(path: str) -> bool:
    resolved = os.path.realpath(path)
    return os.path.isabs(path) and any(
        os.path.commonpath((resolved, os.path.realpath(root))) == os.path.realpath(root)
        for root in WORKSPACE_ROOTS
    )


def _shell_tokens(command: str):
    """Tokeniza palabras y operadores sin perder comillas ni adyacencia de descriptores."""
    i = 0
    while i < len(command):
        if command[i].isspace():
            i += 1
            continue
        start = i
        if command[i] in ";&|<>()":
            i += 1
            while i < len(command) and command[i] in ";&|<>()":
                i += 1
            yield command[start:i], True, start, i
            continue
        quote = None
        while i < len(command):
            char = command[i]
            if char == "\\" and quote != "'":
                i += 2
                continue
            if quote:
                if char == quote:
                    quote = None
            elif char in "\"'":
                quote = char
            elif char.isspace() or char in ";&|<>()":
                break
            i += 1
        # shlex valida comillas/escapes y conserva operadores citados como argumentos.
        word, = shlex.split(command[start:i])
        yield word, False, start, i


def _command_segments(command: str):
    tokens = list(_shell_tokens(command))
    segments, argv = [], []
    i = 0
    last_word_index = None
    while i < len(tokens):
        value, operator, start, _ = tokens[i]
        if not operator:
            argv.append(value)
            last_word_index = i
        elif value in ("&&", "||", ";", "|"):
            if not argv:
                raise ValueError("segmento vacio")
            segments.append(argv)
            argv = []
            last_word_index = None
        elif value in (">", ">&"):
            fd = None
            if last_word_index == i - 1 and tokens[i - 1][3] == start:
                raw = command[tokens[i - 1][2]:start]
                if raw.isascii() and raw.isdigit():
                    fd = argv.pop()
            i += 1
            if i >= len(tokens) or tokens[i][1]:
                raise ValueError("redireccion incompleta")
            target = tokens[i][0]
            if not ((value == ">" and fd in (None, "1", "2") and target == "/dev/null")
                    or (value == ">&" and fd == "2" and target == "1")):
                raise ValueError("redireccion fuera de la lista")
            last_word_index = None
        else:
            raise ValueError("operador fuera de la lista")
        i += 1
    if not argv:
        raise ValueError("segmento vacio")
    return segments + [argv]


def _segment_ok(argv, directories) -> bool:
    while argv and re.fullmatch(r"[A-Z_][A-Z0-9_]*=[^ ]*", argv[0]):
        name, val = argv[0].split("=", 1)
        if name == "ROS_DOMAIN_ID" and val not in LAB_DOMAINS:
            return False
        argv = argv[1:]
    if argv[:2] == ["env", "-u"] and len(argv) >= 3:
        return _segment_ok(argv[3:], directories)
    if not argv:
        return False
    cmd, args = os.path.basename(argv[0]), argv[1:]
    if cmd in READ_ONLY:
        return True
    if cmd == "fuser":  # mirar quien usa un archivo; -k mataria procesos
        return not any(a.startswith("-") and "k" in a.lstrip("-") for a in args)
    if cmd == "orca":  # solo mensajeria de orquestacion y lectura
        return (len(args) >= 2 and args[0] == "orchestration" and args[1] in
                ("check", "send", "ask", "reply", "worker-list", "inbox", "worker-read")) or args[:2] == ["skills", "get"]
    if cmd == "find":
        return not any(a in ("-exec", "-execdir", "-delete", "-ok", "-fprint", "-fls") for a in args)
    if cmd == "cd":
        if len(args) != 1 or not args[0] or args[0].startswith(("-", "~")):
            return False
        targets = [os.path.realpath(os.path.join(base, args[0])) for base in directories]
        if not all(_inside_workspace(target) for target in targets):
            return False
        # Un cd puede fallar o correr en un pipeline: validar tambien los directorios anteriores.
        directories[:] = sorted(set(directories + targets))
        return True
    if cmd in ("source", "."):
        if len(args) != 1:
            return False
        targets = [os.path.realpath(os.path.join(base, args[0])) for base in directories]
        return all(target.startswith("/opt/ros/jazzy/") or
                   (_inside_workspace(target) and target.endswith("/install/setup.bash"))
                   for target in targets)
    if cmd == "mkdir":
        return all(not a.startswith("/") and ".." not in a for a in args if a != "-p")
    if cmd == "git":
        # git -C <worktree del proyecto> <subcomando seguro>: mismo criterio, otro directorio.
        if args[:1] == ["-C"]:
            if len(args) < 3 or not _inside_workspace(os.path.realpath(args[1])):
                return False
            args = args[2:]
        if not args:
            return False
        sub = args[0]
        if sub in GIT_OK:
            return True
        if sub in GIT_LIST_ONLY:
            return len(args) == 1 and sub != "remote" or (len(args) > 1 and set(args[1:]) <= GIT_LIST_ONLY[sub])
        return False
    if cmd == "graphify":
        return bool(args) and args[0] in ("query", "explain", "path")
    if cmd in ("pytest", "py.test"):
        return True
    if re.fullmatch(r"python3?(\.\d+)?", cmd):
        return args[:2] == ["-m", "pytest"] or args[:2] == ["-m", "py_compile"] or args[:1] == ["--version"]
    if cmd == "colcon":
        return bool(args) and args[0] in ("build", "test", "test-result", "list", "info")
    if cmd in ("xacro", "check_urdf"):
        return True
    if cmd == "ros2":
        if len(args) >= 2 and args[0] in ("pkg", "interface") and args[1] in ("list", "show", "prefix", "xml"):
            return True
        # Lectura del grafo ROS (solo con dominio de laboratorio, validado arriba si se paso ROS_DOMAIN_ID)
        return len(args) >= 2 and ((args[0] in ("topic",) and args[1] in ("list", "info", "hz", "echo", "type", "bw"))
                                   or (args[0] in ("node", "service", "action") and args[1] in ("list", "info", "type"))
                                   or (args[0] == "param" and args[1] in ("get", "list", "describe")))
    return False


def decide(command: str, workspaces, cwd=None) -> dict:
    directories = [str(w) for w in (workspaces or [])]
    if not directories or not all(_inside_workspace(w) for w in directories):
        return {"decision": "ask", "reason": "fuera de los worktrees de Base@Home"}
    if cwd is not None:
        if not isinstance(cwd, str) or not _inside_workspace(cwd):
            return {"decision": "ask", "reason": "directorio de ejecucion fuera de los worktrees"}
        directories = [cwd]
    if FORBIDDEN_TOKENS.search(command):
        return {"decision": "ask", "reason": "contiene una operacion que requiere confirmacion humana"}
    try:
        # Expansiones y saltos de linea quedan fuera de esta gramatica acotada.
        if any(char in command for char in ("$", "`", "\n", "\r", "\x00")):
            raise ValueError("expansion o salto de linea")
        segments = _command_segments(command)
    except ValueError:
        return {"decision": "ask", "reason": "sintaxis de shell fuera de la lista acotada"}
    if all(_segment_ok(s, directories) for s in segments):
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
    args = call.get("args") or {}
    cmd = str(args.get("CommandLine", ""))
    out = decide(cmd, payload.get("workspacePaths"), args.get("Cwd"))
    try:
        with open(LOG, "a") as f:
            f.write(json.dumps({"t": time.strftime("%Y-%m-%dT%H:%M:%S"), "decision": out["decision"],
                                "cmd": cmd[:300]}) + "\n")
    except OSError:
        pass
    print(json.dumps(out))


if __name__ == "__main__":
    main()
