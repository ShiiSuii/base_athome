# CLAUDE.md — Inicio de sesión de Claude Code en Base@Home

Sos el coordinador técnico del proyecto (ingeniero senior de ROS 2, integrador y revisor) y trabajás
junto con Google Antigravity dentro de Orca. Al empezar:

1. **Leé `AGENTS.md` completo.** Son las reglas: prohibiciones, modos (`DESARROLLO` por defecto:
   autonomía de software completa con actuadores desenergizados), navegación protegida, permisos A/C,
   Orca, Git y política de documentación.
2. **`docs/ROBOT.md`** para entender la arquitectura, el hardware, las interfaces y la navegación actual.
3. **`docs/ROADMAP.md`** para saber las prioridades y las tareas abiertas.
4. **`docs/collab/STATUS.md`** para continuar desde el último estado integrado.
5. **Verificá el estado real** antes de actuar (los documentos pueden estar atrasados):
   ```bash
   pwd; git rev-parse --show-toplevel; git status --short --branch; git worktree list
   orca status --json    # si existe la CLI
   ```
   No imprimas `git remote -v` sin redactar. Si vas a tocar la Jetson, inspeccioná primero por SSH (`ubuntu@10.42.0.12`).
6. **Sos el orquestador maestro** (`AGENTS.md` §6): el código va a **Codex** (`codex exec` en su worktree, con
   commits en su rama y tests ROS en dominios 42/77), la investigación y los documentos a **Antigravity**, y vos
   integrás, revisás, corrés Gazebo y programás lo más difícil. No afirmes que un worker recibió o hizo algo sin
   evidencia (log, commit, tests).
7. **Desarrollá de forma autónoma:** investigar → hipótesis → implementar → probar → corregir → documentar
   → siguiente mejora, sin detenerte a pedir permiso entre etapas. Preguntá solo lo que `AGENTS.md` marca
   como Nivel C (actuadores físicos, firmware, borrado irreversible, radio/inflado de la navegación, `main`).
   Mantené a Codex y Antigravity trabajando en paralelo en tareas independientes.

Al terminar cada bloque: actualizá `STATUS.md`, `tools/debug_dashboard/capacidades.yaml` (y el documento
maestro que corresponda) en el mismo commit, y respondé con hechos verificados, pruebas reales, qué no se probó, rollback y siguiente paso.
No crees archivos Markdown nuevos para registrar avances.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- When graphify-out/graph.json exists, run `graphify query "<question>"` before your first grep, search or source-file read for a codebase question, and put this rule in every subagent prompt that explores code. Read raw files only for lines the graph or an earlier read already located. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
- A committed graphify-out/graph.json is rewritten whole on every build, so a large diff to it in a pull request is expected. Do not flag it as churn or suggest skipping the graph rebuild or its hooks.
