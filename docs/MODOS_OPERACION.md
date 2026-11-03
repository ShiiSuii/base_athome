# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Modos de operación y pruebas seguras — Base@Home

Este documento distingue **autorización para desarrollar software** de **autorización para desplazar físicamente hardware**. El robot puede tener algunas placas USB alimentadas aunque motores y fuentes de potencia estén apagados. No asumir estado por la mera presencia de tópicos ROS.

## CASA (por defecto, toda sesión sin nueva confirmación)

**Máquinas:** NUC y Jetson pueden estar encendidas por Ethernet. **Hardware real:** estado no confirmado o actuadores apagados. Permitido: auditoría, SSH, simulación, tests offline, `colcon` aislado, HRI con datos sintéticos, cámara de prueba sin torque y mantenimiento reversible siguiendo runbook. Prohibido: motor base, Piper, gripper, Dynamixel pan-tilt, elevador, publicación de acciones físicas, desactivar interlocks. Dominio sim recomendado `ROS_DOMAIN_ID=42`, `ATHOME_SIM=1`; comprobar interfaces y servicios antes de lanzar.

## LAB_SUPERVISADO (solo durante una sesión física autorizada)

**Activación:** el usuario expresa en esta sesión qué subsistemas energizó y autoriza **una tarea física acotada**, con humano presencial. Registrar en `docs/collab/HANDOFF.md`: timestamp, operador, E-stop, alimentación, zona despejada, límites de velocidad/fuerza/torque, realimentación, dispositivo, objetivo, parada y duración de autorización. No cargar permisos de la sesión pasada automáticamente.

**Ejemplo de autorización válida:** "En esta sesión estoy al lado del robot, el Piper está energizado, E-stop accesible, sin personas en el volumen de trabajo; autorizo 1 movimiento de prueba [pose especificada] a velocidad conservadora, primero modo teach/planificación y abortá ante error de feedback".

**Preflight antes de CADA movimiento:**
1. Chequear límites físicos, temperatura/corriente/tensión, ejes/cables y alimentación correctas; salida de emergencia accessible.
2. Identificar `node`, controlador propietario y comando exacto; comprobar realimentación fresca y articulaciones en posición real, no ceros sintéticos.
3. Verificar modelo URDF, zonas de colisión y camino; para MoveIt comparar planning scene con medidas **físicas**, no geometría inventada.
4. Velocidad inicial baja según documentación del fabricante y tipo de ensayo; detener si no hay feedback, si se pierde CAN/ROS, si hay ruido/corriente anómala, colisión, persona cerca o cambio de estado inesperado.
5. Ejecutar **una** acción, leer feedback, documentar resultado, deshabilitar torque si así lo requiere el procedimiento del fabricante. No encadenar goals autónomos sin un contrato de ensayo.

LAB_SUPERVISADO **no** autoriza conducir base ni elevar torso: requieren solicitud separada y procedimiento propio. Cámara es sensor, y puede usarse en CASA si no dispara actuadores ni interfiere en servicio productivo.

## COMPETENCIA / DEMO REAL

La persona responsable inicia un runbook versionado, indicando prueba RoboCup, ubicación, quién observa, límites, roles, plan de reversión y qué nodos se lanzarán. No modificar parámetros de control en plena prueba para "hacer que funcione".

## Hardware aislado no significa ejecución aislada

Un worktree Orca no crea sandbox de USB/CAN, puertos, DDS ni servicios. El dominio de ROS 2 aísla descubrimiento ROS en condiciones normales, **no** procesos que abren hardware por cuenta propia. Inspeccionar los launch y la variable `ROS_DOMAIN_ID`/`RMW_IMPLEMENTATION` antes de ejecutar.

## Criterios de evidencia

`STATIC_PASS`: revisión estática; `UNIT_PASS`: tests; `SIM_PASS`: simulación; `NUC_VERIFIED` y `JETSON_VERIFIED`: host y software realmente observados; `REAL_VERIFIED`: ensayo físico con log y responsable. Jamás usar `REAL_VERIFIED` para URDF/Gazebo/mock.
