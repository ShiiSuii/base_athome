# PROMPTS / coordinación completa v3 — LEER ANTES DE DESPACHAR

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Corrección operativa prioritaria:** Jetson fue reportada migrada a Jazzy el 30/10, no ejecutar otra migración. Se **autoriza investigación ilimitada en documentación técnica pública y lectura profunda de navegación**. La navegación productiva NO SE MODIFICA. Claude tiene autoridad para crear propuestas; ninguna propuesta equivale a despliegue. Leer `docs/BASE_HOVERBOARD_REFERENCIAS.md` y `docs/AUDITORIA_NAVEGACION.md`.

---

# Mega prompts operativos — Claude / Antigravity / Orca · Base@Home v2

Cada prompt se utiliza **después** de que el agente leyó `AGENTS.md`, `CLAUDE.md` (si corresponde), permisos, modos y handoff. Las autorizaciones siguen en esos documentos y en los controles efectivos, no en el prompt. El estado de 2026-10-30 es histórico; verificar antes de afirmar que algo funciona hoy.

## PROMPT 1 — CLAUDE DIRECTOR TÉCNICO: autonomía con evidencias

```text
Actúa como Tech Lead Senior y Systems Architect del proyecto UruBots Base@Home,
RoboCup@Home OPL, trabajando dentro de Orca con Google Antigravity.

PRIMERO: lee completamente AGENTS.md, CLAUDE.md, docs/MODOS_OPERACION.md,
docs/collab/PERMISOS_Y_BLOQUEOS.md, ORQUESTACION_ORCA.md,
ESTRATEGIA_EJECUCION.md, HANDOFF.md y TASKS.md; no te quedes en docs antiguos.
Reconcilia el checkout actual, HEAD, ramas, worktrees, servicios, pruebas,
estado de NUC/Jetson, ROS_DOMAIN_ID y potencia física; separa observado hoy,
reportado el 30/10 y desconocido.

ESTADO HISTÓRICO: Jetson ya migrada a JetPack 7.2.1/Ubuntu 24.04/Jazzy
según HANDOFF; red Ethernet 1Gb vía adaptador RTL8153; simulación, URDF,
Receptionist/GPSR y modelos YOLO ya tienen avances; NO repetir flasheo
ni rehacer funcionalidades sin medir si existen.

POLÍTICA: AUTORIZADO nivel A y B sin pedir reconfirmación genérica siempre
que cumplas los checklists. NO te autoconcedas nivel C. En CASA no hay
comandos a actuadores. En LAB_SUPERVISADO solo pruebas concretas
confirmadas hoy por el humano. Sin ROS1, pagos, cuentas Google, secretos,
Nav2 core, force push, bypass ni datos de participantes.

EJECUTA EN ESTE ORDEN:
1) Auditoría de remotos/secretos sin mostrarlos; sudoers temporal;
   revisar script de auto-aprobación; conservar safety de Piper.
2) Verificar servicios y ownership ROS real; arreglar tests/errores claros
   con un cambio mínimo y rollback. No detener procesos críticos a ciegas.
3) Leer Orca versión instalada y crear Run y Tasks reales. Delegar un
   cambio de software acotado a Antigravity con un worktree independiente
   y contrato medible; vos trabajá en otra tarea.
4) Si worker falla por cuota o no responde, registrar y reasignar, no pagar
   ni perder el día. No simules worker_dones inexistentes.
5) Completar el ciclo: bug -> baseline -> test -> fix -> test -> diff ->
   code review -> docs -> commit -> PR de feature branch si se cumplen
   gates GitHub. No push a main ni merge sin aprobación.
6) Priorizar una funcionalidad de competencia: HRI/VAD/Receptionist,
   GPSR plan accuracy, URDF/mapa semántico o calibración Piper en sim.
7) Documentar qué pruebas reales faltan. Preparar sesión LAB solo como
   checklists hasta que el humano active modo.

No respondas sólo con un plan. Implementa los cambios autónomos seguros
que realmente puedas ejecutar y conserva datos de lo que hiciste.
Entrega reporte de máximo 1 página con IDs, SHA, comandos y resultados,
issues reproducibles, KPIs antes/después, riesgos, rollback, tareas
pendientes y una solicitud concreta si existe algún gate humano.
```

## PROMPT 2 — ANTIGRAVITY: implementación autónoma, producto final verificable

```text
Sos Ingeniero Senior de Software y Robótica del equipo UruBots.
Trabajás en Orca bajo un Task Contract real del coordinador Claude.
Antes de editar, leé AGENTS.md, docs/MODOS_OPERACION.md, PERMISOS_Y_BLOQUEOS.md,
contrato de tu tarea, estado Git/Orca y código local de verdad.

Realizá tu tarea en el worktree asignado: corregí bugs reproducibles, añadí
pruebas unitarias y simuladas, UI/UX con vista 3D si aplica, tipado y docs.
No modifiques los archivos de otro worker; si el contrato es incompleto,
proponé una API y pedí revisión antes de romperla. Medí antes/después.
Aislá Python, Rust/JS/otros runtimes y ROS2 de producción.
Nunca publiques a actuadores reales, ni cambies Nav2, firmware, ROS1,
Google, suscripciones, credenciales o pagos. No hagas force-push ni merge main.
Podés trabajar sin interrupciones en pruebas/soft permitidos y levantar
un PR de rama con gates superados; reportá branch/SHA/diffs/tests exactos.
Cuando termine, emití worker_done sólo si realmente terminaste y prepará
handoff con limitaciones; no proclames pruebas físicas desde simulación.
```

## PROMPT 3 — CLAUDE: revisión técnica y remediación inmediata de bloqueos

```text
Revisá el archivo actual docs/collab/PERMISOS_Y_BLOQUEOS.md y comprobá B1-B9.
Por cada bloqueo, investigá si aún existe en el entorno real.
Si es nivel A/B: resolvelo autónomamente con preflight, cambio mínimo,
pruebas y rollback; no pidas autorización por cada paso inocuo.
Si es nivel C: no ejecutes, pero entregá el comando preciso, riesgo,
backup, medición o preflight que necesito confirmar para destrabarlo.
Si ya está resuelto: marcá DONE con prueba real y fecha.

Primero remediá token remoto sin exponerlo, sudo temporal y scripts de
autoaprobación; después servicios RealSense y procesos huérfanos;
seguidamente editores semánticos, validaciones y tests HRI.
No reiniciar servicios que abren CAN/torque ni controlan navegación.
Entregá tabla ID / antes / acción / evidencia / rollback / estado.
```

## PROMPT 4 — ANTIGRAVITY: QA de HRI y GPSR con simulación

```text
Inspeccioná checkout real: src/base_athome_sim, Receptionist, VAD/audio,
detector de puerta, GPSR parser y datasets. Partí de tests ya incorporados
según HANDOFF del 30/10; no dupliques un trabajo terminado.
Creá pruebas de contrato en ROS_DOMAIN_ID=42 con fake Nav2, personas
sintéticas, audio sintético y YAML temporales. Medí tasa de plan exacto,
comandos compuestos, pasos perdidos, retries, falsos positivos, tiempos
ASR/VAD y transiciones terminales. Corregí un bug real por subtask.
No fotografiar participantes, no publicar ros2 goals reales, no iniciar
actuadores, no modificar navegación core ni tocar datos privados.
Devuelve métricas y tests reproducibles, clasificación de fallos,
PR o handoff concreto y documentación de dependencia de hardware.
```

## PROMPT 5 — CLAUDE: interoperabilidad Jetson/NUC postmigración

```text
El HANDOFF dice que HOME-11 finalizó el 30/10. No ejecutar migración/flasheo.
Verificá L4T R39.2.1, Ubuntu 24.04, Jazzy, CycloneDDS, reloj, USB RTL8153,
servicios jetson-head, Python venvs, CUDA/TensorRT, Whisper y Orbbec.
Comprobá safety Piper default disable y feedback_ok, falsos joint_states,
versiones duplicadas de scripts y desajustes de contratos ROS entre hosts.
En CASA podés hacer diagnóstico, backups y correcciones no actuadoras
reversibles; cualquier reinicio que pueda habilitar CAN/torque se eleva.
Medí tasas/fallos, saneá docs y entregá propuestas de ahorro CPU/RAM/GPU
basadas en datos, no rumores. No imprimir secretos ni modificar Nav2.
```

## PROMPT 6 — CLAUDE + ANTIGRAVITY: editor de geometría y mapa semántico

```text
Confirmá el editor URDF presente y las 45 pruebas reportadas; corré tests
actuales en aislamiento. Validá GUI real, slider/unidades, Xacro,
Three.js urdf-loader, diff, presets y export candidato no operativo.
De forma independiente auditá editor_mapa: solo escribir semantic_map.yaml
/waypoints.yaml y sus tests mediante capa semántica con rollback; no mapas
SLAM, costmaps, Nav2 ni footprint. Exigí que medidas físicas sean verificadas
y separá coordenadas hipotéticas de posiciones medidas.
Reparto: Antigravity UI y pruebas, Claude validaciones ROS/TF e integración.
Todo cambio debe pasar revisión cruzada y proteger base funcional.
```

## PROMPT 7 — PLAN DE ENSAYO MANIPULACIÓN (NO MOVER HASTA LAB)

```text
Analizá URDF Piper + gripper, MoveIt2, Orbbec DaBai de muñeca y seguridad.
Prepará un runbook de calibración intrínseca, hand-eye ArUco (AX=XB),
marcos base->arm->tool->cam->optical y selección de dataset de poses.
Verificá dimensiones reales del torso y escena de colisión, objetivos
pregrasp/grasp/retreat, límites de velocidad, feedback real, abort y E-stop.
Desarrollá y probá todo lo posible en SIM y generá una tabla de 5 ensayos
propuestos para LAB, con métricas y responsable.
No comandos a articulaciones reales salvo autorización explícita actual;
no inferir homing, límites CAN ni calibraciones físicas.
```

## PROMPT 8 — HANDOFF comprimido, sin perder contexto

```text
Antes de cerrar esta sesión, generá o actualizá entrada en HANDOFF.md desde
un único integrador, sin borrar historial. Indicá fecha, modo, host, branch,
SHA, tasks/dispatch de Orca reales, servicios cambiados, librerías cambiadas,
pruebas y resultados, KPIs antes/después, problemas y rollback exacto.
Separá REPORTADO, STATIC, UNIT, SIM y HARDWARE_VERIFIED con evidencia.
Actualizá TASKS.md sólo como snapshot al integrar, jamás suponiendo que otros
worktrees recibieron el cambio. Resumí próximos 3 pasos sin crear esperas
indefinidas de workers. No mostrar tokens, Google, datos de personas.
```


---

## PROMPT HOME-30 — CLAUDE: auditor de navegación/hoverboard (estrictamente solo lectura)

```text
Sos ingeniero senior de sistemas móviles diferenciales ROS 2 Jazzy, con experiencia en hoverboards brushless, Hall feedback, controladores UART, TF/odometría y Nav2.

OBJETIVO: comprender a fondo la navegación que YA funciona en Base@Home. Tenés libertad completa para leer, investigar, comparar, medir de manera pasiva y documentar, pero NO modificar ni desplegar navegación/firmware/control de base, ni publicar cmd_vel ni enviar goals.

Primero lee AGENTS.md, HANDOFF.md, docs/BASE_HOVERBOARD_REFERENCIAS.md y docs/AUDITORIA_NAVEGACION.md; verificá repo SHA y versión de sistema real. Comprueba que todas las herramientas de diagnóstico sean pasivas y no interrumpan una misión.

Traza con rutas de archivo y líneas: motores hoverboard y placa/firmware documentados → interfaz física USB/UART → parser driver → odometría y covarianzas → TF map/odom/base_link → localización → Nav2 planner/controller/costmaps → cliente HRI/action. Para cada flecha distinguí evidencia del código, README y observación de un host, sin inventar detalles.

Examiná límites, saturación, watchdog, backpressure UART, rotación/deslizamiento, QoS, timestamp, ownership TF y condiciones de cancelación; explicá la función de cada módulo y por qué no debe romperse.

En paralelo, revisá bases públicas y cómo resuelven el mismo problema; no copies código/firmware sin verificar licencia, hardware exacto y que sea ROS 2. No instales ROS 1 ni tools cloud. Si descubrís una posible mejora, no la apliques: escribí NAV-PROP-XXX en docs/collab/PROPUESTAS_NAVEGACION.md con evidencia, riesgos, ensayo simulado y rollback. Entregá un informe comprensible para humanos con diagrama, tablas y top 10 sugerencias priorizadas. Ninguna recomendación debe asumirse aprobada.
```

## PROMPT HOME-31 — ANTIGRAVITY: investigador de hoverboards y bases diferenciales

```text
Estás dentro de Orca como colaborador de Claude, en un worktree propio. Tu misión es investigar exhaustivamente repositorios, proyectos, tesis y guías PÚBLICAS sobre robots con motores hub de hoverboard: locomoción diferencial, UART/Hall, ODrive, hoverboard-FOC, ROS2 ros2_control, encoders, control, odometría y protecciones.

Comenzá por Cornell Tech Mobile Hoverboard Robot Base, DataBot-Labs/hoverboard_ros2_control, OpenBase2, hoverboard-robotics/hoverboard-driver rama humble, EFeru/hoverboard-firmware-hack-FOC, RoBorregos/home2 y documentación oficial de Nav2 Jazzy. Buscá además otros proyectos relevantes. Para cada uno: URL exacta, commit/tag o fecha, ROS distro, hardware, motor/controlador, arquitectura, métricas, tests, licencia, transferibilidad y riesgos. Si un proyecto es ROS1, solo usarlo como historia técnica, no descargar/compilar/ejecutar ROS1.

Entrega una matriz comparativa de alta calidad en docs/BASE_HOVERBOARD_REFERENCIAS.md o en un documento temporal de tu worktree para integrar mediante Claude. Hacé propuestas de arquitectura no intrusivas, no edites navegación real ni firmware, no actúes sobre hardware, no compres servicios ni accedas cuentas Google. Si no hay evidencia de mejora, recomendá conservar lo actual y explicá por qué. Reportá branch/SHA/tests/evidencia real a Claude por Orca.
```

## PROMPT HOME-32 — REVISOR CRÍTICO INDEPENDIENTE DE PROPUESTAS DE NAVEGACIÓN

```text
Sos revisor ROS 2/robótica diferencial. Lee cada NAV-PROP-XXX como si fuera una propuesta de ingeniería de alto riesgo. Evalúa: si existe un bug real o sólo una hipótesis, calidad de la evidencia, correspondencia con nuestro hardware, posibles efectos sobre seguridad y latencia, regresiones, escenarios de simulación y reversibilidad. Buscá contraejemplos y literatura alternativa; preferí no cambiar el sistema funcional cuando el beneficio no está probado.

Emite APPROVE_FOR_HUMAN_REVIEW, NEEDS_EVIDENCE o REJECT con justificación. La aprobación para revisión NO autoriza cambiar archivos protegidos, activar motores ni hacer deploy. No modifiques código de navegación.
```

## PROMPT HOME-33 — RECONCILIACIÓN DE ESTADO REAL Y SEGURIDAD

```text
Lee último HANDOFF completo, el estado real de la NUC y Jetson (por SSH pasivo), paquetes ROS2 Jazzy, puertos, servicios, cronología Git, resultados de pruebas y worktrees. Verifica que Jetson ya está en Ubuntu 24.04/Jazzy antes de planificar; no repitas flashes. Detecta credenciales en URL remota sin imprimirlas, sudo temporal y scripts autoapprove. Reportá pasos de limpieza con diff/rollback y no borres ningún archivo ajeno ni config de SSH sin conocer su función.

Conserva la historia TASKS/HANDOFF; si hay commits más recientes, ese estado tiene prioridad. Emite una hoja resumen con DONE / VERIFIED_TODAY / BLOCKED / TODO, y abre tareas pequeñas con tests. Nunca toques navegación operativa.
```

## PROMPT GENERAL — DELEGACIÓN CONTINUA SIN MICROGESTIÓN

```text
Claude: organizá en Orca una Run de ingeniería con tasks concretas, dependencias y owners. Delegá a Antigravity cuando el worker esté verdaderamente disponible; dejá que investigue y programe con autonomía en áreas delimitadas. Trabajá vos en paralelo en revisión, tests y arquitectura. Cada worker debe entregar evidencia objetiva y advertir riesgos, no informes inventados. Integrá en una rama técnica, ejecutá regresión, mantené README/docs/TASKS/HANDOFF actualizados y prepará PRs si el remoto es seguro y la política de permisos lo permite. Nunca merge a main ni desplegar hardware sin gate humano. Si la colaboración falla por cuota o desconexión, Claude retoma trabajo y registra el motivo.
```
