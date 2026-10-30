# Runbook: reconexión del cable NUC ↔ Jetson (HOME-01E / HOME-10)

Preparado 2026-10-30 por Claude Code. Actuadores siguen **desenergizados**. Todo lo de abajo es diagnóstico o software.

## 0. Estado de partida (ya hecho)
- Jetson con los fixes de seguridad **desplegados** (2026-10-30 00:20, respaldo `~/deploy_backups/20261030_002013/before.tgz`): Piper `auto_enable=False`, pan-tilt `auto_torque=False` + servicio `/pan_tilt/set_torque`, whisper `tiny`.
- **No** desplegado todavía: `start_mapping_head.sh` con fallback solo-localhost (abriría CAN/serial sin cable → requiere OK humano).
- `jetson-head.service` sigue en bucle (falla al crear DDS) hasta que haya link.

> **Ejecutado 2026-10-30 ~09:50**: pasos 1–4 OK (enlace por el adaptador USB de la Jetson). Resultado en `docs/network-topology.md` y `docs/ros-interfaces.md`. Quedan las decisiones del §5.

## 1. Conectar el cable y verificar link (humano)
- NUC `enp1s0` (puerto PCI, MAC 50:a1:32:73:cf:cb) ↔ Jetson `enP8p1s0`.
- Opcional en la NUC: `cat /sys/class/net/enp1s0/carrier` → `1`.

## 2. Diagnóstico automático (un comando)
```bash
cd ~/base_athome_ws && scripts/net/check_nuc_jetson_link.sh
```
Revisa carrier, IP `10.42.0.1`, ping, SSH, estado de `jetson-head`, desfase de reloj (NTP sobre SSH), configuración CycloneDDS y una prueba DDS **en dominio 77** en ambos sentidos (Humble↔Jazzy). Reporte en `log/net_checks/`.

| Resultado | Siguiente paso |
|---|---|
| Todo OK | ir a 3 |
| Sin carrier | cable/puerto (no es software) |
| Sin `10.42.0.1` | NM "Wired connection 1" (shared) no levantó: `nmcli con up "Wired connection 1"` (humano; afecta red) |
| Ping OK, DDS falla | anotar; probable incompatibilidad Humble↔Jazzy o XML. No tocar Nav2. Usar dominio 77 para probar |

## 3. Servicio de la cabeza en la Jetson
Con link, el servicio debería arrancar solo en el próximo reinicio automático (cada 5 s). Verificar:
```bash
ssh ubuntu@10.42.0.12 'systemctl status jetson-head --no-pager | head -5; journalctl -u jetson-head -n 30 --no-pager -o cat'
```
Esperado en el log: `PiperArmController ... Brazo DESHABILITADO (usar /arm/set_enable)` y `Torque deshabilitado (auto_torque=false)`. **Si aparece "Brazo habilitado", detener el servicio** (`sudo systemctl stop jetson-head`) y avisar.

## 4. Interfaces ROS entre hosts (solo lectura, dominio 10)
```bash
ros2 node list; ros2 topic list -t | grep -E "arm|pan_tilt|voice|arm_camera"
ros2 topic hz /pan_tilt/joint_states    # sin publicar nada
```
Completar tasas reales en `docs/ros-interfaces.md`. **No** usar `ros2 topic pub`/`service call` a actuadores.

## 5. Decisiones humanas pendientes (en este orden)
1. Desplegar `start_mapping_head.sh` con fallback localhost: `DEPLOY_SCRIPTS=start_mapping_head.sh jetson_head/scripts/deploy_to_jetson.sh --apply`.
2. Revocar el token de GitHub del remoto `origin` de la NUC.
3. Push de la rama `claude/home-00-audit` / PR.
4. HOME-11 (JetPack 7.2.1): elegir destino (**recomendado: NVMe nuevo**, conservar microSD), ventana y método.
5. `MAXN_SUPER` en la Jetson (depende de la fuente de alimentación).
6. Energizar subsistemas: seguir checklist por etapas (E-stop primero).
