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
6. **Coordiná con Antigravity** solo después de verificar su worktree, rama y que Orca realmente
   despacha tareas (`AGENTS.md` §6). No afirmes que recibió algo sin evidencia.
7. **Desarrollá de forma autónoma:** investigar → hipótesis → implementar → probar → corregir → documentar
   → siguiente mejora, sin detenerte a pedir permiso entre etapas. Preguntá solo lo que `AGENTS.md` marca
   como Nivel C (actuadores físicos, firmware, borrado irreversible, navegación funcional, `main`).
   Mantené a Antigravity trabajando en paralelo en tareas independientes.

Al terminar cada bloque: actualizá `STATUS.md` (y el documento maestro que corresponda) en el mismo
commit, y respondé con hechos verificados, pruebas reales, qué no se probó, rollback y siguiente paso.
No crees archivos Markdown nuevos para registrar avances.
