# Contexto vigente v3 — lectura antes del documento siguiente

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Nota de uso:** El editor URDF fue reportado como implementado (45 tests) el 30/10; usar esta especificación como checklist de mejoras, no pedir reimplementación sin inspección.

---

# Especificación funcional — Editor paramétrico URDF para Base@Home

**Objetivo:** crear una aplicación web **local**, simple y segura, que permita inspeccionar el robot Base@Home en 3D y modificar sus parámetros geométricos mientras se perfecciona la estructura física. Es una tarea de desarrollo para Claude Code y Antigravity; no asumir que la aplicación ya existe.

## 1. Archivos del proyecto verificados en repositorio público (29-10-2026)

- Entrada: `src/hoverboard_base/urdf/robot.urdf.xacro`
- Parámetros: `src/hoverboard_base/urdf/xacro/properties.xacro`
- Subsistemas: `base.xacro`, `torso.xacro`, `head.xacro`, `arm_piper.xacro`, `materials.xacro`

El punto de entrada incluye los módulos; `properties.xacro` ya contiene propiedades y expresiones derivadas. **Nunca reemplazar toda esta arquitectura por un URDF creado desde cero**. Antes de codificar, inspeccionar versión exacta del checkout y cómo se genera hoy el `robot.urdf` empleado por launch y `robot_state_publisher`.

Parámetros existentes de referencia (solo valores observados en repositorio, comprobar en checkout y medir físicamente):

| Propiedad Xacro | Valor visto | Descripción | Política |
| --- | ---: | --- | --- |
| `chassis_length` | 0.48 m | largo base | editable en preview |
| `chassis_width` | 0.48 m | ancho base | editable en preview |
| `chassis_height` | 0.05 m | alto chasis | editable en preview |
| `column_height` | 1.30 m | largo columna | editable en preview |
| `column_width` | 0.06 m | ancho columna | editable en preview |
| `carriage_length` | 0.22 m | largo plataforma elevadora | editable en preview |
| `carriage_width` | 0.38 m | ancho plataforma elevadora | editable en preview |
| `prism_size_x/y/z` | 0.25 / 0.35 / 0.35 m | volumen estructural del torso | editable en preview |
| `head_base_z` | 0.12 m | montaje pan-tilt | editable con aviso |
| `track_width` | 0.5620 m | trocha cinemática calibrada | protegido |
| `wheel_radius` | 0.0853 m | radio efectivo calibrado | protegido |
| `laser_offset_{x,y,z}` | 0.22 / 0.00 / 0.18 m | extrínsecos LiDAR | protegido |
| `torso_lift_top_z`, `torso_switch_stroke`, `torso_floor_stroke` | verificar | límites físicos elevador | protegido |

Las propiedades dependientes (`head_base_x`, `prism_offset_x`, `prism_offset_z`) contienen fórmulas `${...}`: preservar su dependencia en Xacro, nunca convertirlas silenciosamente a constantes. En `arm_piper.xacro` la transformación `cube_to_arm_joint` tiene valores fijos: proponer parametrizarlos sin tocar los ejes ni límites internos del Piper.

## 2. Diseño recomendado

**Backend:** Python + FastAPI, ejecución local con Uvicorn escuchando exclusivamente `127.0.0.1` por defecto.

**Frontend:** Three.js + `urdf-loader` (https://github.com/gkjohnson/urdf-loaders), interfaz minimalista responsive. Alternativamente reutilizar una herramienta madura si satisface requisitos, licencia y validaciones. Examinar https://github.com/gkjohnson/urdf-editor como referencia, no asumir que está listo para este caso.

**Generador:** `xacro` real instalado en entorno ROS cuando corresponda; `check_urdf` o validación URDF equivalente, validación XML/TF, sin renderizar expresiones Xacro arbitrarias desde el navegador. Si faltan meshes `package://`, mostrar placeholder geométrico y mensaje; no permitir fallos silenciosos.

**Datos editables:** un perfil YAML/JSON versionado con valores permitidos; parsear el Xacro existente como fuente inicial y elaborar una estrategia segura para actualizar propiedades. No usar regex global para modificar XML. Mantener Xacro original, generar variación temporal y mostrar diff antes de escribir un candidato. Preservar comentarios, includes y expresiones. Crear backups y permitir reversión.

**Carpetas sugeridas, sujetas a auditoría:** `tools/urdf_editor/` con `backend/`, `frontend/`, `tests/`, `README.md`. No introducir una segunda fuente de verdad sin explicitar sincronización.

## 3. Diseño de pantalla

- Cabecera: `Base@Home URDF Editor`, nombre del preset, estado de validación, botones `Deshacer`, `Restaurar`, `Guardar candidato`, `Exportar URDF`.
- Columna izquierda: lista de grupos `Base`, `Columna/Elevador`, `Busto`, `Cabeza/Cámaras`, `Brazo Piper`, `Calibraciones protegidas`; inputs en cm o mm con unidades visibles (guardar siempre SI en metros/radianes), límites, descripción.
- Centro: vista 3D manipulable (orbit, zoom, pan), ejes X/Y/Z, grilla y referencia de suelo; click sobre parte para resaltado y selección.
- Panel derecho: propiedades seleccionadas, árbol de links/joints, diferencias respecto a preset anterior, advertencias sobre colisión/TF.
- Barra inferior: validaciones, consola explicativa y `Datos solo simulados / No conectado a motores`.

**Acciones clave:** al cambiar dimensiones, actualizar preview mediante generación segura; mostrar color distinto para geometría `visual` y `collision`, permitir activar/desactivar ambas. Pose slider del torso y articulaciones solo en modelo visual, nunca emitir comandos CAN/serial/ROS hacia hardware.

## 4. Ejemplo de contrato de API local (ajustable)

- `GET /api/model/info`: propiedades, fuentes, versión, editable/protegido, mallas disponibles.
- `GET /api/model/urdf`: URDF generado y validado para preview, con referencias resueltas de forma segura.
- `POST /api/model/preview`: recibir mapa tipado de propiedades permitidas -> validar -> generar URDF temporal y warnings; sin escribir en fuente.
- `POST /api/model/validate`: informe Xacro, XML, topología, articulaciones, límites, meshes y advertencias de inercia/colisión.
- `POST /api/presets`: guardar perfil versionado, nombre validado y ruta permitida.
- `POST /api/model/export`: escribir candidato **solo en una ruta controlada** con respaldo, diff y confirmación explícita en UI; no desplegar.

Restringir CORS, tamaños de payload, nombres de archivos y paths; no usar `shell=True`, no permitir paths arbitrarios, no exponer endpoints para ejecutar comandos, no hacer escrituras fuera del proyecto, localhost únicamente. Aplicar protección contra enlaces simbólicos/path traversal. Los resultados deben ser atómicos y trazables.

## 5. Separación de cambios visuales y cinemáticos

**Editable de manera inmediata:** geometrías del chasis (solo preview), dimensiones columna, carriage, prisma, carcasa de pantalla y soportes no críticos.

**Editable con advertencia y revisión:** montaje de cabeza, RealSense, cámara muñeca y anclaje exterior del Piper. Estos afectan TF/calibración, por lo que solo crear candidatos. Verificar consistencia de `optical_frame`, colisiones y datos de hand-eye.

**No editable en UI general:** `track_width`, `wheel_radius`, controlador de base, extrínsecos LiDAR usados en navegación, límites del brazo, elevador, pan-tilt, acciones/servicios/velocidades y estados del robot real. No ofrecer un botón para `Aplicar a Nav2` ni `Mover robot`.

El cambio geométrico de chasis **no cambia automáticamente footprint/costmap**. Advertir que la geometría física final podría exigir una revisión manual futura de márgenes/colisiones. La navegación actual se mantiene intacta por pedido explícito del usuario.

Al cambiar dimensiones deben revisarse consistencia de masa, inercia, centro de gravedad y zonas de colisión; no ajustar masas ni inercias automáticamente sin un modelo físico validado. Mostrar aviso `Datos inerciales pendientes de recalcular/medir` cuando corresponda.

## 6. Validaciones mínimas

- XML bien formado; `xacro` termina correctamente sin expresiones sin resolver.
- No hay nombres de `link`/`joint` duplicados ni referencias inexistentes; árbol conectado, sin ciclos.
- Todas las dimensiones físicas son numéricas finitas y positivas; constraints por propiedad; límites de joints coherentes.
- Articulaciones no modificadas mantienen ejes, límites y frames esperados.
- Transformación brazo/cámara/elevador representada con convenciones URDF/REP-103 consistentes.
- Geometría `visual`/`collision` activable, warnings sobre colisiones iniciales y geometrías que sobresalen de la base.
- Mallas `package://piper_description/...` resueltas correctamente o fallback evidente.
- Cambios se pueden deshacer; el Xacro original puede regenerarse exactamente desde Git.
- Prohibido mover motores o modificar nodos ROS en ejecución.

## 7. Etapas y criterios de aceptación

**MVP (primero):** página abre en localhost; carga URDF generado; panel de 8–12 propiedades de geometría; preview 3D; generación temporal de Xacro->URDF; validaciones, presets y exportación segura; README con instrucciones de instalación para PC normal y Linux con ROS 2. Funciona sin hardware real.

**Fase siguiente:** vista de TF, poses de simulación pan/tilt/torso/Piper, comparación de perfiles, chequeo básico de intersecciones, botón para crear diff/PR del perfil (sin push directo), integración opcional con RViz cuando haya ROS.

**No incluido en MVP:** control en vivo, calibración autónoma de hardware, actualización de parámetros Nav2, CAD paramétrico completo, física dinámica o garantía de ausencia de colisiones en robot real.

## 8. Tests y entrega

- Tests unitarios de validación, rangos, presets, rutas, exportación, Xacro y rollback.
- Tests de integración front/backend sobre un Xacro de ejemplo y el archivo del robot actual.
- Casos adversos: valores negativos o NaN, caracteres extraños, subida de archivos, falta de Xacro, paquetes ROS ausentes, mallas no disponibles, request malicioso y cambio simultáneo desde dos agentes.
- Captura o video de la web probando modificación de base/columna sin afectar código de navegación.
- Documentar comandos exactos y evidencias; no afirmar que se ejecutó una prueba de hardware.

Fuentes: `src/hoverboard_base/urdf/xacro/properties.xacro` del repositorio; https://github.com/gkjohnson/urdf-loaders; https://github.com/agilexrobotics/agx_arm_urdf
