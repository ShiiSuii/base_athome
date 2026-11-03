# Jetson Orin Nano — estado posterior a la migración, mantenimiento y comprobación

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## No repetir migración completada

El `HANDOFF.md` aportado por el usuario informa que el **30/10/2026** Claude y el equipo migraron la Jetson Orin Nano a JetPack 7.2.1 (Jetson Linux R39.2.1), Ubuntu 24.04 y ROS 2 Jazzy, y verificaron ROS2 con la NUC. Este dato es del informe; ChatGPT no tiene acceso en vivo al equipo y no lo certifica hoy. El README público todavía menciona Humble/22.04: tratar esa parte como probablemente desactualizada.

### Lista de verificación segura para Claude (por SSH si autorizado)

```bash
# Comandos de lectura en la Jetson. Validar host key; no imprimir datos de usuarios ni secretos.
cat /etc/os-release
uname -r
dpkg-query -W -f='${Version}
' nvidia-l4t-core 2>/dev/null || true
ros2 --help >/dev/null && env | grep -E 'ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION' || true
ip -br addr
systemctl status jetson-head --no-pager
journalctl -u jetson-head -n 30 --no-pager
```

Antes de entrar por SSH, verificar que no hay experimentos activos ni comandos que arranquen servicios de actuadores. No publicar contenido sensible en reportes o repo.

## Tareas de software y regresión autorizables

- Comprobar versiones ROS2/Jazzy, drivers CAN `gs_usb`, Orbbec, Dynamixel, RØDE, Whisper, CUDA/TensorRT y venvs.
- Verificar que control Piper requiere feedback real y `auto_enable=false`, que no hay doble publicación de `joint_states` ni arranque de torque por systemd.
- Mantener dependencias de percepción congeladas y en entornos separados; un export Ultralytics actualizó `numpy` y rompió `cv2` según handoff. Comparar freeze/rollback.
- Verificar que `jetson-head` no cambia límites ni activa motores cuando faltan 12/24/36 V.
- Revisar habilitación de servicios `systemd` con propiedad/impacto/rollback. Los componentes hardware que son seguros en lectura no autorizan movimiento.
- Antes de cualquier mantenimiento que requiera `sudo`, analizar privilegios residuales de flasheo `99-jetson-flash-temp` en NUC y `90-ubuntu-nopasswd` en Jetson; revocar o acotar con humano de ser necesario.

## Pruebas sin intervención física

| Prueba | Resultado del HANDOFF 30/10 | Qué verificar hoy |
|---|---|---|
| Red NUC-Jetson | Ethernet por USB 1 Gb/s, 8/8 checks, desfasaje −11,9 ms | carrier, IP y estado DDS |
| ROS2 Jazzy interhost | `ros2 node list` de NUC veía nodos Jetson | versiones, RMW, domain y nodos |
| Orbbec DaBai | streaming reportado a ~20 Hz | rate pasivo y frame IDs |
| Piper | `auto_enable=False`, feedback guard | log seguro y no torque |
| Pan-tilt | `connected=false` sin 12 V | logs, no torques |
| Whisper | `base.en` probado con 6 % WER en una muestra previa | reproducibilidad con dataset permitido |
| YOLO TensorRT | 24,5 Hz del nodo, 4,3 ms de inferencia en prueba anterior | dataset/sistema/versión, fps medidos |

Si una diferencia aparece, documentar primero y reparar componentes de software por rama con rollback, sin entrar en modo de flasheo.

## Runbook original archivado

El plan original está conservado en `docs/archive/MIGRACION_JETSON_PLAN_INICIAL.md` y el runbook del flasheo en `docs/RUNBOOK_MIGRACION_JETSON.md`, ambos **solo referencia histórica**. No ejecutar `dd`, instalación ISO, QSPI/UEFI, particionado o reinstalación salvo nueva autorización explícita con backup restaurable y destino validado.

Documentos oficiales: https://developer.nvidia.com/embedded/jetpack/downloads ; https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html .
