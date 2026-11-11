---
name: relevo-cuota
description: "Use whenever you are the observer/orchestrator of Orca workers and one of them stops on a quota, usage-limit or rate-limit error (Codex 'You've hit your usage limit', Gemini 'Individual quota reached', OpenCode/freellmapi rate limits), or an escalation shows stopReason=errored with no progress. Hands the task to another free/available agent in the same worktree so work never stops, and schedules the original agent to resume when its quota resets."
---

# Relevo por cuota (Robbie@Home)

Regla del usuario (2026-11-11): si un modelo se traba por falta de tokens/cuota, el trabajo **pasa enseguida a otro agente**
que siga laburando; cuando el original recupera cuota, retoma desde lo que dejó el relevo. Sin preguntar al usuario.

## 1. Detectar (en cada escalación o revisión)

```bash
orca orchestration check --run run_168fe474b6a8          # escalaciones: stopReason=errored / needs-input
orca terminal read --terminal <term_handle> --json | python3 -c "import json,sys;print('\n'.join(json.load(sys.stdin)['result']['terminal']['tail'][-12:]))"
```

Es cuota si el final de la terminal dice alguno de estos:
- Codex: `You've hit your usage limit ... try again at <HORA>`
- Antigravity/Gemini: `Individual quota reached ... Resets in <N>h`
- OpenCode / freellmapi: `rate limit`, `429`, `quota`, `insufficient balance`

Anotar: **dispatch, task, worktree, hora de recuperación**. Si no es cuota (error de código, permiso), NO es este caso.

## 2. Elegir el relevo (cadena de respaldo)

| Se trabó | Relevo inmediato | Notas |
|---|---|---|
| Codex (OpenAI) | OpenCode gratis (Zen, `deepseek-v4.1-flash-free` por defecto) | Si Zen está limitado: `--model freellmapi/auto` |
| Gemini 4 Argon | el mismo worker con `--model gemini-3.8-flash-high` | Relanzar con `--task <id> --retry-of <dispatch>` |
| Gemini 3.8 Flash | OpenCode gratis | |
| OpenCode Zen | OpenCode con `--model freellmapi/auto` | freellmapi: `~/freellmapi` debe estar corriendo (puerto 3001) |
| freellmapi | OpenCode Zen gratis | |

**Nunca** un modelo pago (OpenCode tiene whitelist de modelos gratis). Nunca relevar tareas de Nivel C ni del robot real.

## 3. Lanzar el relevo en el MISMO worktree

Escribir la spec de relevo en `/home/athome/sim_lab/specs/relevo_<worktree>.txt`:

```
RELEVO de <agente> (sin cuota hasta <HORA>). Tarea original: /home/athome/sim_lab/specs/<spec_original>.txt (leela completa).
ANTES DE NADA: git log --oneline -15, git status, y el ledger ~/sim_lab/ledger.tsv para saber donde quedo.
No dupliques corridas ya encoladas (ps -eo args | grep gz_turno). Reglas de la spec original (dominio, gz_turno, sin push).
Itera como la spec. A la hora <HORA> el agente original retoma: deja todo commiteado y una linea de estado en el ledger
(que hiciste, que funciona, que falta, ultima falla) antes de cada pausa larga.
```

```bash
orca orchestration worker-start --spec-file /home/athome/sim_lab/specs/relevo_<worktree>.txt \
  --agent opencode --worktree path:/home/athome/orca/workspaces/base_athome_ws/<worktree> \
  --task-title "Relevo: <tarea>" --keep-open --json
# Gemini Argon agotado: mismo worker con Flash
orca orchestration worker-stop --dispatch <dispatch>
orca orchestration worker-start --task <task_id> --retry-of <dispatch> --worktree path:<worktree> \
  --agent antigravity --model gemini-3.8-flash-high --keep-open --json
```

## 4. Programar la vuelta del original

```bash
# en segundo plano (no bloquea): espera a la hora de recuperación y le avisa
now=$(date +%s); t=$(date -d "<HH:MM+2min>" +%s); [ $t -gt $now ] && sleep $((t-now)); \
orca orchestration send --to dispatch:<relevo> --subject "Devolve el worktree" --body "Termina el paso, commitea, linea de estado en el ledger y worker_done." ; \
orca orchestration send --to dispatch:<original> --subject "Retoma" --body "Tu cuota volvio. ANTES: git log, git status y ledger (trabajo un relevo en tu worktree); integra lo bueno y segui con tu tarea."
```

No usar `pkill -f` con un patrón que también aparezca en tu propio comando (te cortás a vos mismo).

## 5. Registrar

- Una línea en `~/sim_lab/ledger.tsv` (`agente=orquestador`, `prueba=relevo`, cambio_hecho = "relevo <agente> -> <relevo> hasta <HORA>").
- Si es una regla nueva (otro modelo, otro error), actualizar esta skill y `AGENTS.md` §6.
- Avisar al usuario en una línea: quién se trabó, quién siguió y cuándo vuelve.
