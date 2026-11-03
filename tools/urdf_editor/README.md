# Base@Home URDF Editor — fase 1 (preview local)

Herramienta **local** para inspeccionar en 3D el robot Base@Home y probar en
*preview* cambios a las propiedades de
`src/hoverboard_base/urdf/xacro/properties.xacro`. El URDF se genera con el
`xacro` real a partir de `src/hoverboard_base/urdf/robot.urdf.xacro`.

**Esta herramienta NO modifica el repositorio, NO publica `/robot_description`,
NO usa nodos ROS, NO toca TF/Nav2/MoveIt ni hardware.** Solo escucha en
`127.0.0.1`. Pendientes de la especificación original: sección "Fase siguiente" (abajo).

## Instalacion (venv local, sin sudo)

Requisitos del sistema (ya presentes en la NUC con ROS 2 Jazzy):
`xacro` (`ros-jazzy-xacro`) y `check_urdf` (`liburdfdom-tools`). Si ROS no esta
sourceado, el backend agrega `/opt/ros/jazzy` al entorno de esos subprocesos.

```bash
cd tools/urdf_editor
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Arranque

```bash
cd tools/urdf_editor
.venv/bin/python -m backend              # http://127.0.0.1:8765
.venv/bin/python -m backend --port 9000  # puerto alternativo
```

Abrir `http://127.0.0.1:8765/` en el navegador. El host esta fijo en
`127.0.0.1` (no hay opcion para exponerlo en la red). Para usarlo desde otra PC,
usar un tunel SSH: `ssh -L 8765:127.0.0.1:8765 <nuc>`.

## Tests

```bash
cd tools/urdf_editor
.venv/bin/python -m pytest -q
```

`pytest.ini` desactiva los plugins pytest de ROS (`launch_testing`, `ament_*`)
que entran por `PYTHONPATH` cuando ROS esta sourceado.

## API (solo 127.0.0.1)

| Metodo | Ruta | Descripcion |
| --- | --- | --- |
| GET | `/api/params` | Las 40 propiedades: valor literal, tipo (`numeric`/`derived`/`other`), editable, protegida, unidad y rango. |
| GET | `/api/model/urdf` | URDF generado sin cambios + `check_urdf`, links, joints, meshes. |
| POST | `/api/model/preview` | `{"values": {"chassis_length": 0.5}}` → URDF de preview, avisos, diff de properties. |
| POST | `/api/model/export` | `{"values": {...}, "label": "opcional"}` → escribe `candidates/properties_<UTC>[_label].xacro` y `.diff`. |
| GET | `/meshes/<pkg>/<ruta>` | Sirve STL/DAE solo de paquetes permitidos (`piper_description`), confinado a su carpeta. |

Ejemplo:

```bash
curl -s http://127.0.0.1:8765/api/params | python3 -m json.tool | head
curl -s -X POST http://127.0.0.1:8765/api/model/preview \
     -H 'Content-Type: application/json' -d '{"values":{"column_height":1.4}}' | python3 -c \
     'import json,sys; d=json.load(sys.stdin); print(d["check_urdf"]["ok"]); print(d["diff"])'
```

## Alcance y reglas de seguridad

- **Whitelist explicita** en `config/editable_params.json` (12 propiedades con
  unidad, min, max, step). Todo lo demas es solo lectura.
- **Protegidas** (rechazadas con 422 aunque se envien): `track_width`,
  `wheel_radius`, `wheel_width`, `wheel_offset_x`, `laser_offset_{x,y,z}`,
  `torso_lift_top_z`, `torso_switch_stroke`, `torso_floor_stroke`.
- **Derivadas `${...}`** (`head_base_x`, `prism_offset_x`, `prism_offset_z`,
  `screen_offset_x`, `screen_offset_z`): solo lectura; su formula se preserva
  byte a byte y `xacro` las recalcula en el preview.
- Valores: solo numeros JSON finitos dentro de rango. Strings, booleanos,
  `NaN`/`Infinity`, expresiones → 422. El backend **nunca evalua** expresiones
  enviadas por el navegador; solo sustituye el texto del atributo `value` de la
  propiedad (no se reescribe el XML; comentarios y formato quedan intactos) y
  re-verifica que solo cambiaron las propiedades pedidas.
- Preview: copia `src/hoverboard_base/urdf/` a un directorio temporal, aplica el
  cambio en la copia y ejecuta `xacro` + `check_urdf` (sin `shell=True`, con timeout).
- Export: solo dentro de `tools/urdf_editor/candidates/` (gitignored), nombre con
  timestamp UTC, etiqueta restringida a `[A-Za-z0-9_-]{1,40}`, `O_EXCL|O_NOFOLLOW`,
  rechaza `candidates/` si es un symlink. Solo exporta si `check_urdf` pasa.
- Payload maximo 16 KiB. Sin CORS (mismo origen). Sin docs interactivas expuestas.

## Limitaciones conocidas (fase 1)

- Masas/inercias **no** se recalculan (aviso en UI). Footprint/costmap de Nav2
  **no** cambian; un cambio fisico real requiere revision manual futura.
- Meshes: no se copian al directorio temporal (xacro no los lee); se verifica su
  existencia y se sirven en solo lectura desde `src/piper_description/meshes/`.
  Si falta uno, la UI dibuja un placeholder magenta y lista el aviso.
- La transformacion `cube_to_arm_joint` (anclaje del Piper) sigue fija en
  `arm_piper.xacro`; parametrizarla es tarea futura y requiere tocar `src/`.
- Sin presets, deshacer multi-paso, vista visual/collision separada, poses de
  articulaciones ni chequeo de colisiones (fase siguiente de la spec).
- Aplicar un candidato al repo es un paso **manual y revisado** (copiar el
  `.xacro` candidato sobre `properties.xacro` en una rama, regenerar con
  `build_urdf.sh`, revisar diff y PR). El editor no lo hace.
- Probado en la NUC (Ubuntu 24.04, Python 3.12, Jazzy). No probado en otros SO.

## Fase siguiente (pendiente; tarea HOME-05 en `docs/ROADMAP.md`)

Resumen de la especificación original (`git show ac9b069:docs/URDF_EDITOR_SPEC.md`):

- **UI:** grupos Base / Columna-Elevador / Busto / Cabeza-Cámaras / Brazo / Calibraciones protegidas;
  unidades visibles (guardar siempre SI); botones Deshacer, Restaurar, Guardar candidato; barra
  "Datos solo simulados / No conectado a motores".
- **Vista:** colores distintos para `visual` y `collision` con activación independiente; selección por
  clic; árbol de links/joints; vista de TF; sliders de pose de torso, pan-tilt y Piper **solo en el
  modelo** (nunca comandos ROS/CAN/serial).
- **Editables con aviso (solo candidatos):** montaje de cabeza, RealSense, cámara de muñeca y anclaje
  del Piper (`cube_to_arm_joint`): afectan TF y calibración; verificar `optical_frame` y hand-eye.
- **Nunca en la UI:** `track_width`, `wheel_radius`, extrínsecos del LiDAR, límites de brazo/elevador/
  pan-tilt, footprint o parámetros de Nav2. Sin botones "Aplicar a Nav2" ni "Mover robot".
- **Validaciones extra:** nombres de link/joint únicos, árbol conexo sin ciclos, ejes y límites de
  joints no editados intactos, convenciones REP-103, avisos de colisiones iniciales y de
  masa/inercia pendientes de medir.
- **Presets** versionados, comparación de perfiles, chequeo básico de intersecciones, botón para
  generar diff/PR del perfil (sin push).
- **Tests adversos:** NaN/negativos, caracteres raros, falta de xacro o de paquetes ROS, meshes
  ausentes, request malicioso, dos agentes editando a la vez; prueba visual con captura.
- Referencias: [urdf-loaders](https://github.com/gkjohnson/urdf-loaders),
  [urdf-editor](https://github.com/gkjohnson/urdf-editor), [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf).

## Licencias de terceros

Ver `static/vendor/VENDOR.md`: three.js 0.186.1 (MIT) y urdf-loader 0.13.1 (Apache-2.0).

## Rollback

La herramienta vive solo en `tools/urdf_editor/`. Para quitarla: borrar esa
carpeta (o revertir el commit). No hay cambios fuera de ella.
