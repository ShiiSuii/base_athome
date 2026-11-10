# STATUS.md — Estado vivo de Base@Home

Corto y actualizado: se **reemplaza** lo viejo, no se acumula (`AGENTS.md` §9). Lo histórico se
recupera por commit (§8). Editado por el integrador (Claude).

**Última actualización:** 2026-11-10 (tarde) · Claude Code · `DESARROLLO` · Restaurant nuevo (barman, voz robot yes/no, rostros, brazo bloqueado hasta calibrar) integrado con 907 tests; **falla en Gazebo**: el detector de clientes no responde a tiempo y no ve personas simuladas (Codex investigando). Arena: con radio 0.44 no pasa puertas de 0.90 m.

## Integración 2026-11-10 (Claude)

- **Corrida real 10/11 17:14: el SLAM saltó 5,5 m durante la vuelta continua.** El robot solo giraba en el lugar, pero el mapa lo pasó de (0,0) a (5,34, 1,35), dentro de un obstáculo. Resultado: "Collision Ahead" en el Spin, "Start occupied" (error 205) en las 13 poses del cliente y "I could not reach the bar". Causa probable: girar a 0,8 rad/s es demasiado rápido para el emparejamiento de slam_toolbox. Bajar la velocidad del Spin de Restaurant es configuración de navegación (Nivel C, decisión del usuario). Arreglado: el robot gira hacia el barman con el rumbo de odometría aunque la vuelta se haya cortado, la cabeza lo sigue con una detección de frente (no con el mapa corrido) y, si tampoco puede girar, apunta la cabeza por ángulo. La cara se guarda con la única cara centrada aunque haya gente atrás. 234 tests.
- **Revisión de bugs (10/11 noche):** `people_here` no procesaba ningún cuadro si el pan-tilt dejaba de dar feedback (falla USB conocida) y devolvía 0 personas sin explicar por qué. Ahora mira igual y lo cuenta como `head_stale` en el log. Import sin usar en `arm_manipulation`. Un test leía el archivo real de poses y dejó de valer cuando el usuario calibró las 3 con `restaurantgrasping`. Revisado y sin problemas: entrega (`_face_label`), plazo del barrido de Gemini (se reinicia en `finally`), corte de la vuelta continua y flujo de la pinza. 372 tests de Restaurant y bag_finder.
- **Restaurant, corrida real 10/11 tarde:** barman bien (1,58 m, mano en 18 cuadros) y 2 clientes en la vuelta continua. Al final quedó en un bucle: en la barra, Nav2 cortaba el Spin ("Collision Ahead") y repetía la búsqueda. Ahora, si no puede girar, busca con barrido de cabeza y la base quieta. Se llama Robbie y se presenta al empezar. Queda en el log si guardó o reconoció cada cara (sin audio no apunta la cabeza y no guarda la cara). Radio de exclusión del barman: 1,0 m. Abierto: el cliente 1 a 2,6 m no tuvo camino (error 208 en las 15 poses), y de cerca la cámara lo vio sin profundidad.
- **Tablero debug con Restaurant:** la matriz suma §2.5 (ítems 5.1 a 5.8, 2360 pts, según la hoja oficial 2026.2). El tablero la muestra como prueba con 6 capacidades: 1960 pts en camino y 400 faltan (bandeja suelta). Nueva tarjeta con `restaurant`, `ARM=1 restaurant` y `restaurantgrasping`. Pasan los 5 tests del tablero.
- **Restaurant, vuelta continua (pedido del usuario):** barman y clientes se buscan con un solo Nav2 Spin de 360° (0,8 rad/s, unos 10 s) en vez de 12 paradas (1 a 1,5 min). bag_finder `people_here` escanea durante todo el giro, se corta con `stop_scan` y guarda el rumbo de cada persona en odom (ruedas). El giro hacia el barman sale de ese rumbo menos el yaw odom actual, y después se corrige mirando. Para clientes: gira 30°, avisa "I am looking for customers" y completa la vuelta. Sin cambios en Nav2. Si no hay `scan_turn_fn`, queda el modo por pasos. 249 tests. Sin probar en el robot: a 46°/s puede haber imágenes movidas.
- **Restaurant, versión simple del brazo (pedido del usuario):** el brazo va a `restaurant_pregrasp` con la pinza abierta; el barman pone el primer objeto y dice "robot yes"; la pinza cierra, se confirma por el ancho que hay algo y viaja en postgrasp. Si cierra vacía, abre y guarda el brazo. El segundo objeto va a la bandeja. El agarre por visión queda para después. El aviso de búsqueda de clientes ahora se dice después del primer giro. El cliente queda a 0,8 m (antes 0,9). 242 tests. Sin probar en el robot ni en simulación.
- **Restaurant, ideas de otros equipos (10/11):** se investigó con Codex, 2 Antigravity y un subagente de Claude (EIC walkie, SEU, RoBorregos, Tidyboy, Pumas, TDPs de 24 equipos). Se implementó: acercamiento en dos etapas también para los clientes de la vuelta (a más de 2,2 m: paso intermedio y re-detección con la cámara al frente; si no ve a nadie, barrido); aviso "I will now turn around to look for customers" y cabeza al frente antes de esa vuelta; agarre de uno de los dos primeros objetos; el objeto viaja en postgrasp. El brazo exige solo pregrasp, postgrasp y handover calibradas (carry opcional). Faltan enseñar esas 3 poses con `restaurant_arm_teach` y lanzar con `ARM=1`. 217 tests de sesión y brazo. Sin probar en el robot.
- **Firmware de la base (Nivel C, sin flashear):** se apagaba con el robot quieto 8 min aunque la NUC mandara velocidad 0, y sin comandos sonaban 3 beeps sin parar hasta apagarse. Hay binario nuevo `firmware_bin/hover_lenzod_usart3_odometry_keepalive.bin` y parche en `firmware/patches/`: compila sin advertencias. Falta: hacer una copia de la placa con `st-flash read`, flashear con el usuario presente y probar.
- **Barman por vuelta completa (pedido del usuario, probando en el robot real):** `_find_barman` hace 12 vistas de 30° con la base girando en el lugar y la cámara al frente. Usa el modo nuevo `people_here` de bag_finder: todas las personas con profundidad medida, sin mano levantada (antes solo aceptaba manos levantadas). Junta las vistas (`people_here.py`, a 0.7 m es la misma persona), se pone de frente a la más cercana al robot, la sigue con la cabeza y pregunta. Si un giro falla, mira una vez más y no gira de nuevo. Gemini 4 Argon dejó a medias la cámara al frente antes de navegar: llamaba a `_look_forward`, que no existía (`AttributeError` al ir a un cliente). Lo completó Claude. El plazo del barrido de Gemini es equivalente a ab8f64d de Codex, que no se integró. 1690 tests (percepción, skills y sim) y build OK. Sin probar en el robot ni en Gazebo.
- **Arreglo del barrido de Restaurant (Codex, ab8f64d en `codex/rest-simfix`, sin integrar):** `people_scan.py` calcula cuánto dura cada barrido y el nodo espera eso más un margen; el barrido corta en su plazo. 281 tests pasan; merge limpio con HEAD. Falta Gazebo y robot. Ya no hace falta: Gemini hizo el mismo plazo en `bag_finder.py` y quedó integrado con el barman nuevo.
- **`nuc_aliases.sh`:** cada terminal mostraba "syntax error near unexpected token `(`" porque `~/.bashrc` define `alias gpsr` (misión) antes de la función `gpsr()` (parser). La función pasa a llamarse `gpsr_parser`; `gpsr` sigue siendo la misión. Verificado con `bash -i`: sin errores.
- **Restaurant nuevo (bfe4e0c, c279518, fea6723, 4f30f54):** busca al barman girando la base 30° por paso con la cámara al frente, frena al ver a alguien, lo sigue con la cabeza y pregunta "Are you the barman?"; guarda cara y posición. Clientes: "Searching for people." y pan -90, +90, 0. Pedido con "robot yes/no", 3 intentos. Barra: primer ítem al mostrador y agarre (pregrasp/grasp/postgrasp/carry), resto a bandeja; entrega "three, two, one" justo antes de abrir. Brazo apagado por defecto (`ARM=1`) y bloqueado mientras las 4 poses de `restaurant_arm_poses.yaml` tengan `calibrated: false`. Caras fuera del repo, borradas en cada prueba. `order_asr.py` (errores de Whisper) aún sin conectar.
- **Gazebo Restaurant 10/11: FALLA.** "The customer detector did not respond in time" en cada búsqueda: el nodo espera 85 s y el barrido de bag_finder tarda ~103 s; además bag_finder no vio ninguna persona simulada (máx 0 en ~175 cuadros). Codex en `codex/rest-simfix`. Log: `/tmp/restaurant_180262`.
- **Antigravity:** `agy_bounded_allow.py` ahora aprueba cadenas `a && b` si cada parte está permitida (6b9a936); hay que reinstalar la copia de `~/.gemini/config/` para que valga.
- **Reglas Restaurant 2026:** en `docs/MATRIZ_REGLAS_HABILIDADES.txt` §5 (b121344).

- **Restaurant (034c221):** el trabajo del 09/11 de Antigravity y Codex que había quedado sin commit (ver abajo). Cambió el barrido inicial: ahora recorre `0, -90, 0, +90, 0` y junta hasta 2 clientes, en vez de cortar al primero. Inflado 0.54 m solo en el overlay de Restaurant.
- **Arena (cf72c67, Codex):** prueba estática con la huella del robot: 29 waypoints y 406 pares conectados, 5 puertas transitables, poses de Restaurant libres. Se movieron dos puntos de escenario que caían sobre muebles inflados. Falta Gazebo.
- **Arena en Gazebo (10/11, Claude): falla con el radio del robot.** Recorrida de 8 lugares con Nav2 real y `nav2_params.yaml` del robot (radio 0.44): **0/7 destinos**, "no valid path found". En el costmap global las puertas de 0.90 m quedan con costo inscrito (99) salvo **una sola celda central (94)**: con 0.44 m el robot ocupa 0.88 m. Con el perfil de simulación `eval` (radio 0.36, inflado 0.75): llegó al dormitorio y volvió a home, pero en la puerta de la cocina el controlador repite "RegulatedPurePursuitController detected collision ahead" y luego queda "Start occupied". Consecuencia para el robot real: con 0.44 no pasa puertas de arena de 0.80–0.90 m. **Decisión del usuario (Nivel C):** radio, tras medir el brazo. Logs: `/tmp/sim_commander_125573` (0.44) y `/tmp/sim_commander_131599` (0.36).
- **Receptionist (4a9ad7f, Codex):** describe la ropa del invitado 1 al invitado 2 (≤20 palabras); `--wait-doorbell` espera el timbre hasta 30 s (apagado por defecto). `--start-doorbell-audio` abre el micrófono con sounddevice solo mientras alguien escucha; en la NUC falta instalar `python3-sounddevice` (apt, pedir al usuario).
- **TDP:** la actualización de Codex quedó en la rama `codex/tdp` (2ee93bc) sin integrar, porque `tdp/urubots/main.tex` tiene cambios sin commit del usuario desde el 09/11 16:24.
- **Codex no puede commitear:** su sandbox monta `.git` de solo lectura; commitea el integrador.
- **Rollback:** `git revert` del commit correspondiente.

## Restaurant — correcciones 2026-11-09 (Antigravity + Codex)

- **Estimación de cliente lejano + avance progresivo SLAM, barrido `0° → -90° → 0° → +90° → 0°` y task planning para 2 clientes (2026-11-09, Antigravity):**
  - **Percepción lejana (`bag_vision.py`, `raised_hand.py`, `bag_finder.py`):** cuando `person_map_position` (mínimo 12 píxeles de profundidad en torso/rostro) falla porque la persona está lejos, `person_estimated_map_position` recupera primero profundidad rala coherente ($\ge 4$ píxeles en torso/hombros) y, si no hay depth válido, estima distancia por escala aparente pinhole (`_pinhole_person_depth` con altura de torso/cabeza/hombros/bbox y matriz intrínseca `K`, acotada a $[1.8, 6.0]$ m) proyectada sobre el rayo medido por TF (`T, R`). `TrackedCustomer` / `RaisedHandsAggregator` marcan `"estimated": True` solo cuando no hubo muestras con depth real (si hay depth real, prefiere la mediana medida).
  - **Avance progresivo sin mapa (`restaurant_session.py`):** si un cliente tiene `"estimated": True` (o está a $\ge 3.5$ m sin plan directo a $0.9$ m), `progressive_advance_candidates` + `_approach_customer` navegan primero un tramo intermedio seguro dentro del costmap SLAM hacia el cliente, re-detectan de cerca (`_relocalize_customer_near`) con profundidad real de la RealSense y luego completan la aproximación final a $0.9$ m.
  - **Barrido inicial (`bag_finder.py`, `restaurant_node.py`):** `RAISED_HANDS_PANS = (0.0, -90.0, 0.0, 90.0, 0.0)` (centro fijo $0^\circ \rightarrow 90^\circ$ derecha $\rightarrow$ centro $0^\circ \rightarrow 90^\circ$ izquierda $\rightarrow$ centro $0^\circ$), ampliando el límite de software de pan en `bag_finder.py` de $\pm 65^\circ$ a $\pm 90^\circ$ (acorde a `[-1.57, 1.57]` rad del nodo `pan_tilt_node`). En `restaurant_node.py` se envía `stop_on_first=False`, `head_dps=12.0`, `view_s=2.5` para completar el barrido y detectar hasta 2 clientes.
  - **Task planning para 2 clientes (`restaurant_session.py`, `restaurant_node.py`, `restaurant.py`):** `rank_customers` prioriza clientes con posición medida sobre estimada y menor distancia desde la barra; con `batch_orders=True` (activo por defecto en `RestaurantNode`), cuando ve 2 clientes toma primero el pedido del cliente 1, va directo a la mesa del cliente 2 a tomar su pedido, y luego regresa al `kitchen_bar` para preparar y entregar cada orden. También se amplió `DEFAULT_MENU` con bebidas, frutas y snacks estándar de RoboCup@Home, se subió la espera de entrega en bandeja (`delivery_wait_s=7.0` s) y en pinza (`give_blocking` a `8.0` s), y el silencio previo de TTS (`speaker.py`) de `0.5` s a `0.8` s.
  - **Evidencia:** `UNIT_PASS` (`254 passed in 2.96s` en `test_person_depth.py`, `test_bag_vision.py`, `test_raised_hands_mode.py`, `test_pose_debug.py`, `test_speaker.py`, `test_restaurant_session.py`, `test_restaurant.py`); `STATIC_PASS` (`colcon build --symlink-install --packages-select base_athome_perception base_athome_skills` exit 0, `git diff --check` limpio). Sin tocar archivos protegidos de navegación ni enviar comandos físicos a actuadores (modo `DESARROLLO`, solo NUC y Jetson encendidas). Validación física pendiente en sesión supervisada. Rollback: revertir diff propio en `bag_vision.py`, `raised_hand.py`, `bag_finder.py`, `speaker.py`, `arm_manipulation.py`, `restaurant.py`, `restaurant_session.py`, `restaurant_node.py` y sus tests.

- Distancia corregida en software (2026-11-09, Codex): `bag_vision.py` reemplaza el único parche central por superficie interior del torso y fallback facial con medidas reales; rechaza muestras aisladas y superficies ambiguas. `test_person_depth.py` reprodujo 4 fallos antes y ahora pasa: incluye barrido completo con depth agujereado que confirma al cliente y retorna en la primera vista. `UNIT_PASS`: 237 tests (person_depth, bag_vision, raised_hands_mode, pose_debug, speaker, restaurant_session), 2.86 s; build percepción+skills y grafo actualizados al cerrar. Rama `claude/home-41-piper-mission`, HEAD base 84f5f9b; cambios sin commit. Archivos propios de esta corrección: bag_vision.py, test_person_depth.py, ROBOT y STATUS. Sin modificar configuración de cámara, servicios, navegación ni actuadores; prueba física pendiente tras relanzamiento por el usuario. Rollback: restaurar solo person_map_position anterior y retirar helpers nuevos mediante diff inverso propio; no descartar los demás cambios de la sesión.

- Audio/distancia 2026-11-09: speaker antepone **500 ms de silencio PCM al mismo WAV**, probado preservando bytes de voz antes de paplay; no solo espera entre procesos. Restaurant conserva el error del detector y explica distancia no medible/gesto sin confirmar/timeout, sin afirmar ausencia de personas cuando hubo gesto. Sesión nueva 116238 demuestra código diagnóstico cargado: 93 cuadros, 17 manos, 17 posiciones inválidas, cero fallos TF/depth-topic/intrínsecos. Muestra numérica pasiva de profundidad 640×480 16UC1: **12.9 % válido**, centro cero, percentiles de píxeles válidos 1576/2216/3850 mm; no se guardaron imágenes. Emisor=1, láser=150, autoexposición=true (solo consultas). La causa de baja cobertura depth sigue pendiente; no se inventan posiciones ni se habilita navegación sin localización. `UNIT_PASS`: 208 tests, 4.51 s; build percepción+skills+hoverboard_base **3 paquetes, 6.93 s, exit 0**; import desde build de este workspace confirma helper de silencio instalado. Sin reinicios/runtime changes: usuario debe cerrar y relanzar Restaurant para cargar código Python nuevo. Archivos de esta etapa: speaker/test_speaker, restaurant_node/session/test_restaurant_session y STATUS; cambios propios sin commit sobre base 84f5f9b, rollback con diff inverso de esos cambios, preservando trabajo ajeno.

- Seguimiento de cliente solicitado 2026-11-09: `restaurant_node` renueva cada 0.5 s una lease `track_customer` hacia bag_finder; vence en 1.5 s sin renovación. Activo al acercarse, saludar, escuchar, confirmar y entregar; se cancela al volver a barra, terminar o parar. BagFinder asocia por posición mapa (<0.8 m), apunta a nariz/torso con pasos máximos 2°, exige RGB/feedback frescos y TF/profundidad; no habilita torque, no barre al perderlo ni selecciona al distractor por tamaño. Quitado el comando fijo (0,-10) al llegar. No corrige por sí mismo la falta de localización que impide aceptar clientes. Build perception+skills exit 0 (2 paquetes); `UNIT_PASS`: 202 tests (restaurant_session, pose_debug, raised_hands_mode), 4.86 s; `git diff --check` correcto, graphify actualizado. Código no cargado en la sesión activa, sin comandos físicos ni reinicios; prueba supervisada pendiente tras relanzar Restaurant. Archivos: restaurant_node/session, bag_finder, tests restaurant_session/pose_debug, STATUS y capacidades. Rollback: revertir únicamente este diff propio; rama `claude/home-41-piper-mission`, base `84f5f9b`.

- Diagnóstico de clientes 2026-11-09, sesión 113278: 75 cuadros, hasta 2 personas, 27 observaciones de mano levantada y cero clientes; `no_raised_hand` era un motivo genérico incorrecto. Consulta pasiva posterior confirma TF map→camera_color_optical_frame y publicadores depth/camera_info, pero no demuestra su validez durante esas observaciones. Código actualizado (sin relanzar): distingue `unlocalized_raised_hand`/`unconfirmed_raised_hand` y registra rechazos por TF, profundidad, intrínsecos o posición inválida; 78 tests passed, diff check correcto. También corregida referencia accidental a `head_options` fuera de alcance en `_search_person`, introducida en b7da36c; no era la ruta de Restaurant. Pendiente próxima corrida para aislar causa; no se afirma corrección de detección física. Archivos: bag_finder.py y test_raised_hands_mode.py más este registro; rollback de estos cambios mediante diff inverso propio, sin descartar cambios ajenos. Sin cambios de hosts/servicios ni comandos a actuadores.

- Monitoreo pasivo durante prueba del usuario, sesión `/tmp/restaurant_113278` (2026-11-09): debug 3.0–5.6 Hz durante barrido; parámetros consultados local/global inflación **0.54 m**, driver base mínimo angular **165** (167 aún no aplicado). Avisos de cabeza sin llegar al objetivo, tilt medido -6°, continúa con feedback quieto; Nav2 registra inflación insuficiente para comprobación optimizada de colisiones, revisar antes de dar por validado el margen. Sin modificaciones de runtime ni comandos enviados por Codex; corte al primer cliente aún no verificado físicamente en esta observación.

- Ajuste posterior autorizado expresamente por el usuario: overlay `nav2_restaurant.yaml`, inflación local/global 0.59 → 0.54 m; radio físico 0.44 m intacto. Base de trabajo `84f5f9b`, rama `claude/home-41-piper-mission`. Solo archivo de configuración: sin recarga ni reinicios, validación física pendiente. `UNIT_PASS`: 115 tests de restaurant_session; `STATIC_PASS`: YAML local/global 0.54 m y radio físico 0.44 m, `git diff --check` correcto. Rollback: quitar las dos entradas `inflation_layer` agregadas en el overlay. Detección lejana: código exige confianza YOLO 0.45, articulaciones 0.40 y profundidad 0.3–6 m; cámara 640×480. Causa del caso observado aún no verificada; faltan distancia aproximada y confirmar si aparece caja/esqueleto en debug.

- Rama `claude/home-41-piper-mission`; base `7c8fecc`; commits `b7da36c` (barrido), `db7cd31` (audio), `10b4233` (base); debug en el commit que contiene esta entrada.
- `CODIGO`: primer escaneo con base quieta, pan -60..+60 a 6 °/s, vistas de 3 s y presupuesto de 80 s; vuelve al confirmar el primer cliente. GPSR conserva sus valores por defecto.
- `CODIGO`: Piper completa un WAV temporal antes de tomar el lock de reproducción; `stop()` termina síntesis o reproducción; se borra el WAV. Sin shell ni tubería de audio.
- Corrección explícita del usuario: **base** `min_steer_raw` 165 → 167 (+1 % redondeado); `angular_scale=260` y pan-tilt `pwm_limit=600` intactos. **No aplicado al driver activo**, sin despliegue Jetson. Para aplicar: cerrar Restaurant con Ctrl-C de forma supervisada, ejecutar `colcon build --symlink-install --packages-select hoverboard_base`, `source install/setup.bash` y relanzar `restaurant` cuando el humano autorice la sesión física.
- Debug: el executor ya era multihilo y el timer tenía callback group separado; `serve_lock` suprimía el timer durante pedidos. Ahora el timer no depende de ese lock ni de cabeza quieta/profundidad; `pose_lock` serializa el modelo Ultralytics compartido.
- `UNIT_PASS`: `python3 -m pytest -q src/base_athome_skills/test/test_restaurant_session.py src/base_athome_perception/test/test_raised_hands_mode.py src/base_athome_perception/test/test_speaker.py src/base_athome_perception/test/test_pose_debug.py` → **195 passed**, 5.44 s, exit 0. `STATIC_PASS`: build symlink-install de `base_athome_perception base_athome_skills` → **2 packages finished**, 4.56 s, exit 0; `git diff --check` sin errores.
- Verificación pasiva dominio 10 del proceso **anterior**: `/bag_finder/debug_image` 2.89–2.99 Hz en reposo. No demuestra el código nuevo durante barrido. Al intentar relanzar únicamente bag_finder, PID 91118 ya no existía; el guard abortó **antes de señalar procesos o lanzar otro**. Sin cambios en hosts/servicios, sin comandos a actuadores, sin validación física, audio Bluetooth ni fuerza real probados. Próximo paso: iniciar sesión supervisada y observar debug en barrido; no iniciar pedidos desde este worker.
- Archivos propios: restaurant_node/session y tests; bag_finder y tests raised_hands/pose_debug; speaker y test; hoverboard_driver/hoverboard_params; este STATUS y capacidades.yaml. Archivos ajenos de navegación, RViz, calibraciones y TDP conservados sin stagear; única excepción autorizada de navegación: mínimo angular de base solicitado expresamente.
- Rollback: `git revert <sha-debug>`, `git revert 10b4233`, `git revert db7cd31`, `git revert b7da36c` (revisar conflictos si hay cambios posteriores). Copia previa del script de debug en `/tmp/restaurant_90980/debug_fix_backup/bag_finder.py`; logs originales intactos. `graphify update .`: exit 0, 7008 nodos/14298 aristas; advierte 121 archivos sin extractor.

## 1. Estado comprobado al pausar

| Ítem | Estado | Evidencia |
|---|---|---|
| NUC | Ubuntu 24.04, kernel 7.0.0-42 (reiniciada hoy), Jazzy, CycloneDDS, dominio 10 | `NUC_VERIFIED` |
| Jetson | L4T R39.2.1, Jazzy; `jetson-head` activo | `JETSON_VERIFIED` |
| Piper | energizado, **habilitado** con feedback válido (`/arm/status` leído 2026-11-04; deshabilitarlo lo deja caer: hacerlo con el usuario presente); controlador md5 `2e347c49` | `JETSON_VERIFIED` |
| Pan-tilt | **falla eléctrica abierta**: 118 desconexiones USB del U2D2 el 04/11, solo ese puerto (`Cannot enable. Maybe the USB cable is bad?`, error -71). Si la revisión inicial falla, el receptionist desactiva el seguimiento. `pan_tilt_params.yaml` no existe en la Jetson: usa valores por defecto del nodo. **Sentido del pan corregido** (`pan_direction` +1, lo observó el usuario desde atrás): falta confirmarlo con `V` del calibrador. Cable cambiado: 3 caídas a las 19:32, falta probarlo con torque y movimiento (`M`) | `JETSON_VERIFIED` |
| Teleop Meta Quest 2 | **en pausa** por pedido del usuario; nodo detenido en la Jetson. Fidelidad validada en sim; seguimiento real pendiente | `SIM_PASS`; real parcial |
| Simulación | cerrada | — |
| Commander | alias `commander`: pedido por micrófono (inglés), dice el plan y actúa con "yes"; "stop" cancela; web en `:8770`. Diálogo probado con ASR y ejecutor falsos (6 tests). Whisper real: 3 frases bien, ~1,4 s; el resto sin resultado en 14 s porque el ASR detectó voz continua (posible ruido del ambiente). **Nunca ejecutó una misión en el robot.** Agarre real: exige `torso_state` corriendo | `UNIT_PASS`/`SIM_PASS`; real `PENDIENTE` |
| Tests offline | Jetson scripts 32 + Quest 18, percepción 122, skills 76 (1 skipped), bringup (elevador) 6, sim 7, pan-tilt 8, agentes 40 | `UNIT_PASS` 2026-11-04 (skills) |

## 2. Rama y Git

- Rama `claude/home-41-piper-mission` **subida a GitHub** el 2026-11-04 (repo público). `main` = `1da2388` sin cambios: merge a `main` = decisión humana (PR pendiente). HOME-SEC: el token viejo sigue sin revocar.
- Cambios del usuario sin commitear (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`
  `waypoints.yaml` y `~/waypoints_base_athome.yaml` quedaron vacíos por un test (ver §6); regenerados desde `semantic_map.yaml` el 2026-11-04.
  (incluye la `home` re-enseñada `[73.87, 169.72, -138.02, 26.69, -15.7, -117.17]`).
- Reparación tras el reinicio de la NUC: commit vacío `c98344d` rehecho como `f16f457`; respaldo `~/git_backup_20261103_165442`; `core.fsync=committed`.

## 2a. Percepción de la bolsa unificada y simulación de lazo cerrado (2026-11-05)

- **Módulo único:** `base_athome_perception/bag_vision.py`, usado por `bag_finder`, `calibrar_bolsa` y `arm_manipulation`. Antes eran tres copias que divergían: el debug mostraba números que la misión no usaría.
- **Los 1557 cm:** se descartan los rayos de muñeca a menos de 20° bajo el horizonte, los de más de 1.6 m y las bolsas cortadas por el borde, siempre con motivo. Que esa sea la causa exacta es hipótesis compatible: falta medir ángulo y altura con TF en el robot.
- **Cabeza:** mide con los puntos de profundidad de la caja (sin piso ni fondo), no con la cara frontal. Una bolsa cortada solo se acepta en la primera medición; `refine_bag` la vuelve a medir de cerca y exige verla entera.
- **Asociación:** la muñeca confirma la bolsa de la cabeza, con compuerta de 35 cm; varias sin referencia no se adivinan.
- **Bug real:** la conversión de la bolsa a la base usaba el último `/amcl_pose`, atrasado; daba 7-9 cm de error de agarre. Ahora usa TF (`_pose_now`).
- **Gesto:** `MIN_POINT_ANGLE_DEG` de 35 a 25: con 35 no leía una bolsa a menos de 1 m de la persona. Sin validar con personas reales.
- **Simulación:** `sim_scene.py` dibuja RGB y profundidad con z-buffer; `bag_finder` real corre contra esa imagen. Con `ATHOME_SIM=1` no usa YOLO: el detector es por color y la pose viene de keypoints simulados.
- **Escenarios (simulación, distancia TCP-objeto al cerrar):** normal 1.6 cm; persona lejos 5.6 cm (OK con poco margen; el límite simulado es 6 cm y sin resolver: la cabeza quedó 5 cm corrida); persona moviéndose 2.0 cm; bolsa distractora 1.9 cm; cámara de muñeca desalineada 3° 3.4 cm; gesto a 1.4 m de la bolsa: rechazo correcto. En la tanda previa, con el código anterior a los últimos cambios: sin gesto y sin persona fallan con su motivo; bolsa de 45 cm 1.8 cm.
- **Pruebas:** `pytest` de perception, skills y sim: 297 passed, dominio 77. YOLO real: `yolo_people` y `detect_all` corrieron sobre una imagen de muestra y `bag_finder` real arrancó en dominio 77.
- **No probado:** nada en el robot real ni con bolsas reales. La simulación no demuestra seguridad física ni que la pinza sujete una bolsa deformable. Con la muñeca, la bolsa suele quedar cortada de cerca y se agarra con lo medido por la cabeza.
- **Rollback:** revertir el commit de este bloque.

## 2b. Prueba real de "take out the trash": paso a paso (nunca corrió en el robot)

Antes de empezar: E-stop a mano, zona despejada y otra persona atenta. Bolsa liviana con nudo o asa arriba, sin nada bajo en el camino: el lidar no ve por debajo de 38 cm. Probar cada etapa antes de pasar a la siguiente. Si algo se mueve raro: E-stop.

**A. Sin mover nada**
1. Energizar, abrir una terminal y actualizar: `cd ~/base_athome_ws && colcon build --symlink-install --packages-select base_athome_perception base_athome_sim base_athome_skills && source install/setup.bash`.
2. `commander` en esa terminal. No decirle nada todavía. Si falta algo, lo avisa. RViz: confirmar la ubicación en el mapa.
3. En otra terminal, `revisar_basura`. Solo lee: arreglar cada `FALTA` con lo que indica y repetir hasta que diga `Todo listo`.
4. Cerrar `commander` con Ctrl-C. `calibrar_bolsa` no mueve nada: poner la bolsa delante, medir con cinta y comparar con lo que dice cada cámara. Las coordenadas salen desde el piso, bajo el centro del robot. Anotar el error. Cerrar con Q.

**B. Partes por separado, con actuadores**
5. `calibrar_pantilt`: `T`, `V`, `M`, sin caídas.
6. Brazo en `pre_grasp_floor`; `calibrar_elevador` solo sube hasta el sensor superior y confirma la ubicación.
7. Brazo energizado y habilitado (si no, el controlador rechaza el comando: mirar su log). Pinza en el aire, con el nudo de la bolsa entre los dedos: `ros2 topic pub --once /arm/gripper_cmd std_msgs/msg/Float32 "{data: 0.0}"`. Mirar con `ros2 topic echo --once /arm/status` cuánto queda `gripper_mm`. Tiene que quedar **por encima de 2 mm**: si no, la misión lo daría por agarre vacío. Volver a abrir con `{data: 105.0}`. Fuerza del controlador: 1.5 N·m, sin medir sobre una bolsa real.

**C. Misión completa**
8. `commander`. Bolsa a 1-2 m del robot y persona a más de 65 cm de la bolsa. Decir en inglés "Take out the trash" y responder "Yes".
9. Cuando diga "Please point at the bag", señalar con el brazo estirado y sostenerlo hasta que mire la bolsa. Después apartarse: se acerca, baja el elevador, agarra, sube y va a `trash_drop`, donde la deja. "Stop" cancela.
10. Copiar `/tmp/commander_<pid>/`, porque se borra al reiniciar. Anotar: distancia real entre la pinza y la bolsa, lo que dijo cada paso y si la muñeca confirmó.

**Qué mirar, según la simulación con la pose real**
- La medición de la cabeza es la que manda el agarre: en simulación la muñeca casi nunca ve la bolsa entera de cerca. Si el paso 4 da más de 3-4 cm de error, frenar y corregir eso antes de agarrar.
- La pose del 05/11 19:05 deja poco alcance. Si dice `fuera del alcance`, enseñar la pose un poco más cerca del robot, con `ensenar_pregrasp`.
- Antes del tramo final se para 30 cm más atrás para volver a medir, y el tramo final se guía por la bolsa: puede girar un poco en el lugar con el brazo afuera. Dejar libre alrededor.
- El agarre es 4 cm debajo de la parte más alta. Si la bolsa es baja o el nudo está a un costado, puede tomar aire.
- `no vi a nadie señalando`: estirar más el brazo. Con menos de 25° respecto del cuerpo no cuenta como gesto.
- `agarre vacío`: revisar el paso 7. Hoy no se confirma con esfuerzo.

**Volver atrás:** `git revert` del commit de este bloque; la pose anterior está en `pre_grasp_floor_previous`.

## 2c. Agarre de bolsa fina (2026-11-05)

- Para bolsas la pinza cierra a 0 mm y agarra 4 cm debajo de la parte más alta. Antes cerraba a 50 mm y daba por vacío cualquier ancho de 55 mm o menos: en el robot una bolsa se habría escapado.
- Simulación (distancia de la pinza al nudo, nudo de 10 mm): normal 2.5 cm, persona lejos 5.6 cm (límite 6, sin resolver), persona moviéndose 2.5, distractora 2.3, bolsa de 45 cm 1.9, muñeca desalineada 3° 3.4, pose real enseñada 1.2. Todas completaron la misión.
- `revisar_basura`: verificación previa de solo lectura. Probada contra la simulación, no contra el robot; allí indicó la pose de piso como faltante porque el archivo de poses de simulación no la tenía.
- **Sin probar:** nada en el robot. El nudo simulado es un punto rígido: no demuestra que una bolsa real no se escape.

## 2d. La misión de la basura en Gazebo (2026-11-05)

- **Correr:** `GAZEBO=1 GZ_GUI=1 ATHOME_SIM_BAG=1.5,1.2,0.30 ATHOME_SIM_PERSON=2.6,0.3 ATHOME_SIM_OBJECT_LABEL=handbag simulation/scripts/sim_commander.sh "take out the trash"`. Sin `GZ_GUI` corre sin ventana. Guía para el equipo: `simulation/README.md`.
- **Qué viene de Gazebo:** física, lidar, RealSense con profundidad, cámara de muñeca, bolsa (cae y se engancha con una unión al cerrar la pinza). **Sigue siendo simplificado:** Nav2 falso, base movida por velocidad y sin gravedad (no simula ruedas), detector por color, persona con keypoints simulados, voz simulada, sin simular el firmware real.
- **Resultado:** 1 corrida completa en pantalla, pinza a 1.7 cm del nudo, con la pose real de piso. Antes fallaron 6, cada una por un problema distinto.
- **Arreglos que valen para el robot real:** la cabeza centra la bolsa antes de medirla; se vuelve a medir 30 cm antes del tramo final; el tramo final se guía por la bolsa medida, no por 40 cm fijos (Nav2 real tiene 18 cm de tolerancia); frena 4 cm antes del límite de alcance.
- **Alcance con la pose del 05/11 19:05:** con esa orientación a z = 0.20 m, x = 0.55 m llega con y ≥ 9 cm; x = 0.57 m solo con y ≥ 15 cm; x = 0.60 m nunca (IK numérica). Enseñar la pose más cerca da más margen.
- **Clon limpio:** en esta NUC, clon nuevo de `development` (`b296808`), con `HOME` vacío y sin escenarios previos: compila y completa la misión, pinza a 1.6 cm. No prueba una máquina distinta: los paquetes de sistema ya estaban instalados y el entorno de ROS podría heredar algo del workspace principal.
- **Carpeta:** todo lo propio de simulación pasó a `simulation/`; el código del robot sigue en `src/` y es el mismo.
- **Sin probar:** nada en el robot real. En Gazebo hay 2 corridas completas, sin una tanda de repetición ni los escenarios de falla.

## 2e. Nav2 real en Gazebo (2026-11-06)

- `NAV2=1` levanta los servidores de Nav2 con `nav2_params.yaml` del robot, sin modificarlo, y un mapa generado del mundo con `simulation/scripts/hacer_mapa.py`. El simulador deja de ofrecer su acción falsa y su localización.
- **Resultado:** 1 corrida completa, pinza a 3.8 cm del nudo. El planificador avisa `Inflation layer ... not set sufficiently` para colisiones no circulares: sale de los parámetros del robot, revisarlo.
- **Sigue simplificado:** base por velocidad sin ruedas, detector por color, persona con puntos simulados, voz escrita.
- **Sin probar:** nada de esto en el robot real.

## 2f. Plan B, correcciones por voz y regresión en Gazebo con Nav2 (2026-11-06)

- **Plan B:** si falla el agarre del piso, sube el elevador sin mover el brazo, dice que le pongan el asa entre los dedos, espera 5 s, cierra y confirma por ancho. Si queda vacía, aborta. Cerrar la pinza con una persona cerca: probar con el usuario, con la bolsa ya puesta y nadie tocando los dedos.
- **Correcciones por voz:** si no entiende el gesto o no ve la bolsa, dice qué corregir y reintenta hasta 3 veces.
- **Regresión (Gazebo + Nav2, una corrida por escenario, `simulation/scripts/sim_escenarios.py`):** normal 1.7 cm; persona lejos 3.0; persona moviéndose 1.1; distractora 1.4; bolsa de 45 cm 2.7; muñeca desalineada 3° 2.1; sin gesto, sin persona y gesto mal apuntado fallan en el paso esperado; plan B completo; plan B sin que la persona dé la bolsa falla al recibirla. `normal` y `mueve` se corrieron con código de dos momentos; una tanda completa con el código final queda pendiente.
- **Receptionist:** ya no pregunta "Is that correct?". `ATHOME_HRI_CONFIRM=1` la reactiva.
- **Investigaciones en `docs/`, de Antigravity con Flash, verificar antes de usar:** `INVESTIGACION_{BOLSA_PERCEPCION,BOLSA_AGARRE,PICK_PLACE,HRI,GPSR,VOZ,CALIBRACION,REGLAS_GENERALES,SIM_REALISTA,LLM_VLM}.txt`, `PAPERS_REVISION.txt`.
- **Riesgos del reglamento a confirmar (`INVESTIGACION_REGLAS_GENERALES.txt`):** E-stop común a todos los actuadores, arranque por teclado, lidar ciego bajo 38 cm, parlante Bluetooth, sin protocolo de ayuda humana.
- **Sin probar:** nada de esto en el robot real.

## 2g. Revisión adversaria y regresión final (2026-11-06)

- **Corregido (revisión de Antigravity, `REVISION_BOLSA` en su rama):** agarre de respaldo sin que otro detector pise el objetivo; tramo final que no da "listo" con la bolsa corrida a un costado y frena antes de que choque el brazo; bolsas negras medidas por el rayo al piso si no hay profundidad; un pedido por vez en `bag_finder`; gesto que salta se descarta.
- **Regresión en Gazebo + Nav2, una corrida por escenario:** normal 3.0 cm; persona lejos 1.6; moviéndose 1.6; distractora 2.0; bolsa de 45 cm 2.6, tras corregir que frenaba por la bolsa misma; muñeca desalineada 2.4; los 3 casos de falla, en el paso esperado; plan B 1.2; plan B sin manos falla al recibir. `lejos` falló una vez por el gesto antes de ensanchar la tolerancia: **inestable, no resuelto del todo**.
- **Nuevo:** arranque del receptionist por puerta o voz, sin teclado (HOME-88): "start", "go" o "ready" pueden dispararlo por una charla cercana, probar con cuidado. GPSR a plan (HOME-89): de 30 comandos, 26 compilan y 1 con skills ya disponibles. `follow_logic.py` sin conectar (HOME-92). `revisar_competencia.py`, solo lectura (HOME-93), probado solo con tests.
- **Sin probar:** nada de esto en el robot real.

## 2h. Seguir personas (2026-11-06)

- "follow me" funciona en Gazebo con Nav2: `bag_finder` mueve la cabeza y publica `/bag_finder/people`, `follow_logic.py` decide y Nav2 recibe el objetivo actualizado. Termina cuando la persona queda quieta 8 s y cerca. Con persona que camina: 3 de 3 con el código final; antes falló por una predicción que se iba de largo.
- **Limitaciones:** sin re-identificación, puede pasarse a otra persona. En simulación el detector es de keypoints, no YOLO. Velocidades sin calibrar en el robot. **Riesgo real:** seguir con el lidar a 38 cm no ve lo bajo.
- Atributos visuales (`person_attributes.py`) y timbre (`doorbell_detector.py`) están con tests y sin conectar. Mundo `casa_pick_place` con su mapa, sin misión de Pick and Place todavía.
- `.gitignore` excluía los mapas `.pgm`: en un clon nuevo Nav2 no tenía mapa. Corregido.
- **Clon limpio con Nav2 (`b8ca8e7`):** clon nuevo con `HOME` vacío, en esta NUC, que ya tiene ROS y Gazebo: compila y completa "take out the trash" con la pinza a 1.4 cm. Sin probar en otra máquina.
- Durante el seguimiento, la cámara usa el TF del instante de la imagen; si los relojes de NUC y Jetson no coinciden, usa el último. Sincronizar con chrony sigue pendiente.

## 2i. Graphify, grafo de conocimiento del código (2026-11-06)

- `graphifyy` 0.9.103 en `~/.local/share/graphify-venv`, enlazado en `~/.local/bin/graphify`. Instalado con `graphify install --project`: skill en `.claude/skills/graphify/`, sección en `CLAUDE.md` y hooks en `.claude/settings.json`, que recuerdan consultar el grafo.
- **Uso:** `graphify query "..."`, `graphify path A B`, `graphify explain X`; tras cambiar código, `graphify update .`. Salida local en `graphify-out/`, sin subir.
- **Probado:** grafo de código en 9 s, 5415 nodos y 11060 aristas; una consulta encontró `wrist_measure` en `bag_vision.py`, mezclada con resultados poco útiles, como `camera` del editor URDF. Solo código: documentos e imágenes requieren `/graphify` y gastan tokens.
- **Otros integrantes:** sin `graphify` instalado, los hooks fallan al ejecutarse; instalar con `pip install graphifyy==0.9.103` en un entorno aparte.
- **Rollback:** `graphify claude uninstall --project`, o revertir el commit de este bloque.

## 2j. El commander como GPSR (2026-11-06)

- **Alias `gpsr`:** mismo commander con `gpsr:=true`. Espera `/start_signal/door_open`, va a `instruction_point`, atiende hasta 3 órdenes una por vez y vuelve al punto después de cada una. Con 3 órdenes no entendidas, sugiere un operador propio. `commander` sigue igual. **Al abrirse la puerta el robot se mueve solo, sin "yes":** E-stop a mano. Las órdenes siguen pidiendo "yes".
- **Gramática oficial:** si el compilador propio no entiende, se usa `tools/gpsr_parser` con las skills disponibles (`compile_gpsr_actions`). De los 500 comandos de `tools/gpsr_parser/data/commands.jsonl`, 121 compilan por esa vía, con un mapa que contiene todos sus lugares; lo demás pide reconocer personas por nombre, gesto o postura, o responder preguntas, y se rechaza entero. Compilar no significa que se ejecute bien.
- **Probado:** tests con ASR y ejecutor falsos; sesión en el simulador liviano con voz guionada: dos órdenes hasta terminar y vuelta al punto. La primera se perdió porque el guion arrancó tarde, y eso llevó a que el silencio no gaste intentos.
- **Sin probar:** detector de puerta real, Whisper con órdenes reales, robot real. Falta marcar `instruction_point` en el mapa real.

## 2k. LLM local en el commander (2026-11-06)

- Qwen2.5 1.5B Q4 en `~/.local/share/llm-venv`, servidor local en `127.0.0.1:8771`. Si el compilador y la gramática oficial no entienden, propone acciones limitadas por gramática a nuestros lugares y objetos; pasa por el validador y pide "yes". El VLM no está hecho.
- **Medido en la NUC:** 1.9 GiB, 0.4 s de mediana y 5 s de máximo. Frases de reserva: 14/16 correctas, 0/8 órdenes falsas. Son pocas frases.
- **Sin probar:** sesión de voz completa con la LLM y Whisper real.

## 2l. Tablero `debug` (2026-11-06)

- `debug` abre `http://127.0.0.1:8780` (`tools/debug_dashboard/`). Solo lee: no da órdenes al robot. Corre con baja prioridad.
- **Se actualiza solo:** rama y commit, procesos vivos, puntos de la matriz, skills del registro, comandos GPSR que compilan y resultados de `sim_escenarios.py`. Marca con ⚠ una capacidad si su código cambió después de la última prueba.
- **A mano:** la etapa de cada capacidad en `capacidades.yaml`, al cerrar cada bloque. `test_estado.py` falla si un ítem de la matriz queda sin capacidad.
- Restaurant y Final no están desglosados en la matriz. Pick and Place tiene 840 pts sin desglosar.
- **Rediseño visual (2026-11-06):** Interfaz modernizada (`frontend-design`), sección "¿Qué puedo probar ya mismo?", métricas de madurez por etapa, buscador en vivo, filtros interactivos y actualización de capacidades (`vlm` y `abrir_puerta` pasaron de `falta` a `tests` con `8e516ab`).


## 2m. Apertura de puerta, cliente VLM y transición de coordinación (2026-11-06)

- **Transición de coordinación:** Cuota de Claude agotada hasta el lunes. Antigravity toma el desarrollo y soporte directo con el usuario. Grafo de conocimiento del código actualizado con `graphify update .` (5615 nodos, 11491 aristas, 325 comunidades).
- **Apertura de puerta de palanca:** `door_open.py` (lógica pura y rutina de pasos sin ROS), `door_open.yaml` (parámetros clamp) y `abrir_puerta.py` (ventana interactiva de calibración y enseñanza de poses con OpenCV y MoveIt/Piper). Tests en `test_door_open.py`: 6 tests passing (`UNIT_PASS`).
- **Servidor y cliente VLM:**
  - Servidor `vlm_server.py` en `src/base_athome_skills/scripts/vlm_server.py` (SmolVLM 500M Instruct Q8 + mmproj en `127.0.0.1:8772`). Modelos descargados en `~/.local/share/base_athome/models/`.
  - Cliente puro Python `vlm.py` en `src/base_athome_skills/base_athome_skills/mission/vlm.py` (`ask_vlm`, `ask_vlm_cv`, timeouts, base64 sin ROS ni excepciones fatales). Tests en `test_vlm.py`: 6 tests passing (`UNIT_PASS`).
- **Skills Vault independiente:** Creado repo en `~/claude-skills-vault` con `frontend-design`, `mempalace`, `headroom`, `nvidia-skills`, `playwright`, `omniroute` y `graphify`. Enlazado por `sync.sh` a `~/.claude/skills/` y `~/.gemini/config/skills/`.
- **Codex CLI:** Instalado `codex-cli 0.167.0` en `/home/athome/.local/bin/codex` para vinculación de cuenta OpenAI y trabajo como agente en Orca.
- **Sin probar:** Apertura de puerta real (requiere sesión supervisada con palanca real y E-stop); inferencia del VLM contra la cámara RealSense en tiempo real.


## 2n. VLM en el commander y grafo del código (2026-11-09)

- **VLM:** `describe_view` mira con la RealSense de la cabeza, le pregunta a SmolVLM 500M (`vlm_server.py`, `127.0.0.1:8772`) y dice la respuesta. `commander`/`gpsr` lo levantan solos si está el modelo. No mueve nada, así que no pide "yes"; si falla, no vuelve a casa.
- **Frases:** "what do you see", "describe the room", "how many cups do you see", "¿qué ves?". La gramática oficial ahora también compila `count` y `describe` de objetos (va al lugar y pregunta). Nombres de personas: siguen sin hacerse.
- **LLM:** nueva acción `describe`. Medido hoy en la NUC: 14/16 frases bien, 0/8 órdenes falsas, máximo 3,2 s.
- **Medido en la NUC:** el VLM ocupa 835 MiB y tarda unos 5 s; respondió bien 4 de 4 imágenes de prueba (no del robot). **Sin probar:** con la cámara real, por voz y con el resto del stack corriendo.
- **Grafo:** `grafo` (o `debug grafo`) abre el grafo de graphify en `http://127.0.0.1:8780/grafo`. Se regenera con `graphify update .`.
- **Revisado:** los commits `8e516ab` y `12445d7` de Antigravity (puerta, cliente VLM, tablero). Los tests de puerta y VLM pasan.

## 2o. Navegación: el lidar no llega a los costmaps, perfil de evaluación (2026-11-09)

- **Hallazgo (comprobado en Gazebo):** en `nav2_params.yaml` la fuente `scan` no define `max_obstacle_height`; en Jazzy vale 0.0 y el lidar está a 0.38 m, así que los dos costmaps descartan todos sus puntos. Nav2 solo ve el mapa estático: no marca personas ni muebles movidos. Con una persona en el camino, el robot pasó por su centro (0.06 m). El robot real usa el mismo archivo. Corregirlo toca la navegación protegida: **lo decide el usuario**.
- **Perfiles solo de simulación** (`simulation/base_athome_sim/config/`, con `ATHOME_NAV_PROFILE`): `altura` (solo esa corrección) y `eval` (corrección, radio 0.36, inflado 0.75/2.5, costmap local 5 m, progress checker 12 s, AMCL 5 m y `bt_eval.xml`, que espera antes de limpiar y retrocede 15 cm).
- **Mundo `nav_eval`** (puerta de 0.85 m) y `simulation/scripts/nav_eval.py`. Resultados (llegó / distancia mínima al centro de la persona):

| Escenario | robot | altura | eval |
|---|---|---|---|
| Persona quieta en el camino | sí, 0.06 m (la atraviesa) | sí, 0.87 m | sí, 1.04 m |
| Puerta de 0.85 m | no (sin camino) | no | sí |
| Persona en la puerta que se corre a los 12 s | no | no | sí |

- **Sin probar:** en el robot; el radio 0.36 supone que el brazo no sobresale del chasis (medirlo). Una sola corrida por caso.
- **Antigravity (integrado, solo tests):** `front_dock.py` (HOME-101, 37 tests; concluye seguir con lógica propia y no `opennav_docking`), Restaurant (HOME-102: `raised_hand.py`, `restaurant.py`; Restaurant sí está en el RuleBook 2026.2 §5.5, exige SLAM sin mapa previo) y la pestaña "Resumen" del tablero (HOME-103). El plan de Restaurant es un borrador: usa mesas que no existen y anuncia un pedido antes de escucharlo.

## 2p. Docking frontal en el ejecutor (2026-11-09)

- Skill `dock_front` (usa `front_dock.py` de HOME-101): "approach the wall", "acercate a la puerta", "get close to the table". Mueve la base con `cmd_vel` lento como `dock_table`, tope 1.5 m, y pide "yes".
- **Gazebo (mundo `nav_eval`):** después de Nav2 avanzó 0.45 m y frenó con el lidar a 0.29 m de la pared, o sea 0.27 m del frente del chasis (pedido 0.25). **Sin probar:** puerta cerrada, persona delante y robot real.
- No se integró el chequeo trasero propuesto para salir de la mesa: el lidar está adelante y no ve detrás del chasis; daría una seguridad falsa.

## 2q. Navegación del robot: lidar en los costmaps (2026-11-09, autorizado por el usuario)

- `nav2_params.yaml`: `max_obstacle_height: 2.0` en la fuente `scan` de los dos costmaps, costmap local 5 x 5 m, progress checker 12 s, AMCL `laser_max_range` 5.0. `base_athome_bt.xml`: espera 3 s antes de limpiar el costmap local y 4 s (antes 1) antes de los reintentos.
- **Sin cambiar** (falta medir el brazo): `robot_radius` 0.44 e inflado. Sigue sin pasar puertas de 0.85 m.
- **Gazebo con esta configuración:** persona quieta en el camino → la rodea a 0.88 m de su centro (antes la atravesaba). Persona en la puerta de 0.85 m → no llega (el radio). Una corrida por caso.
- **Sin probar en el robot real.** Primera prueba sugerida: Nav2 con el robot quieto y RViz en `/local_costmap/costmap`; pararse a 1 m adelante y verificar que se marca. Rollback: `git revert` de este commit.

## 2r. AVISO: con el lidar en los costmaps, el robot se ve a sí mismo (2026-11-09)

- **Medido en Gazebo:** el lidar ve el costado izquierdo del robot (rayos a 116–130°, 0.30–0.34 m; en base_link x≈0.0–0.08, y≈0.24–0.30). Con `max_obstacle_height: 2.0` eso marca al robot como obstáculo y Nav2 responde "Start occupied". **No usar Nav2 en el robot real hasta integrar el filtro** (`/scan_nav`, en curso con Codex) o confirmar con el lidar real que no pasa.
- Restaurant en Gazebo: SLAM activo con el launch oficial de slam_toolbox, RViz con mapa, clientes sentados visibles, barrido suave de 15°/s. Ve a los dos clientes; no llega por el punto anterior.

## 2s. Restaurant completo en Gazebo y lidar solo hacia adelante (2026-11-09)

- **Filtro `/scan_nav`** (`scan_self_filter`, base_athome_bringup): descarta los puntos dentro de la caja del cuerpo del robot y los rayos fuera de ±100° (`max_abs_angle_deg`). Lo usan los costmaps (robot real, Gazebo y Restaurant) y el SLAM de Restaurant; AMCL sigue con `/scan` de 270°. Sin probar con el lidar real: verificar que la caja cubra el brazo.
- **Restaurant (Fable):** las poses de llegada se eligen con el mismo criterio que el planner y se validan con `compute_path_to_pose` antes de mandar el goal. **Gazebo:** atendió a los dos clientes (pedido por voz simulada, barman, entrega) en el salón original (8 x 6 m) y en uno agrandado (12 x 9 m, ahora por defecto). Una corrida por mundo.
- También: SLAM con el launch oficial de slam_toolbox (antes quedaba sin configurar), barrido del pan-tilt a 15°/s en pasos de 1.5°, Nav2 cancelado de forma segura y "stop" por voz durante la navegación (Codex), elevador bloqueado en Restaurant, GPSR por gesto/postura/ropa (285/500 comandos compilan).
- **Sin probar en el robot real.** Antes: confirmar el filtro con el lidar real y que Nav2 planifique desde la posición inicial.

## 2t. Restaurant en el robot real: arreglos del primer intento (2026-11-09)

- **No llegaba a los clientes:** Nav2 decía "Robot is out of bounds of the costmap" en (0,0): el costmap global de Restaurant tomaba el tamaño del mapa SLAM recién empezado. Ahora es una ventana móvil de 20 x 20 m (solo en `nav2_restaurant.yaml`). `/scan_nav` se publica reliable.
- **No detectaba la mano levantada:** el pan-tilt queda trabado en pan 27°, tilt -6° y el barrido salteaba todas las vistas. Ahora mira desde donde está la cabeza. Además, **al empezar gira la base** +90°, -180° y +90° con el Spin de Nav2 (la cabeza recta) y busca manos en cada orientación.
- **Visual:** RViz se abre solo con mapa, costmaps, lidar completo y filtrado, plan, cámara y la pestaña "Detecciones OpenPose (pose y manos)" (`/bag_finder/debug_image`, esqueleto COCO de YOLOv8-pose).
- **Arranque y cierre:** Ctrl-C cierra todos los nodos (KILL a los 10 s solo de la sesión); los restos de una corrida muerta se cierran y se espera a que salgan del grafo; los chequeos buscan el programa real, no cualquier línea con "commander".
- **Bluetooth:** el parlante se desconectaba en cada pausa: WirePlumber suspendía el sink, fallaba al liberar el transporte ("Failed to release transport") y el parlante cortaba ("Connection reset by peer"). Arreglo de raíz: `~/.config/wireplumber/bluetooth.lua.d/51-base-athome-no-suspend.lua` (copia en `src/base_athome_perception/config/wireplumber/`; instalar copiándolo y `systemctl --user restart wireplumber`). Comprobado: 30 s después de sonar sigue conectado y RUNNING. Además, bip suave (6 %) cada 5 s y reconexión automática del parlante emparejado si se cae. **Hoy el parlante quedó por cable** (salida de auriculares de la NUC): sin Bluetooth no se bipea ni se reconecta; para volver al Bluetooth, `AUDIO_SPEAKER=bluetooth`.
- **Sin probar en el robot:** el giro inicial, la ventana móvil y el keepalive (tests y revisión). El pan-tilt trabado hay que revisarlo físicamente.

## 3. Cómo retomar la teleoperación con el Quest (pendiente: prueba completa en el brazo real)

1. Quest por USB a la **Jetson**, depuración USB autorizada (`adb devices` → `device`).
2. **Cerrar todas las pestañas del navegador del Quest** (las viejas se restauran y mandan datos sin pose: fue la causa
   de "no se mueve"). Abrir una sola: `http://localhost:8765` (debe decir **v 7**).
3. Lanzar en la Jetson (robot real, dominio 10):
   ```bash
   source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=10 CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
   cd ~ && python3 quest_teleop_node.py --ros-args -p mode:=cartesian -p real:=true -p max_joint_dps:=20.0 \
     -p max_wrist_dps:=40.0 -p max_lead_deg:=5.0
   ```
   Simulación: `ROS_DOMAIN_ID=42`, sin `real:=true`, con `base_athome_sim` en la NUC usando `~/cyclonedds.xml` (red).
4. Habilitar el brazo a mano (`/arm/set_enable`; el nodo nunca lo hace). En el visor: gatillo sobre la página → **GRIP apuntando hacia donde mira el robot** (calibra el frente) → **mantener X**: primero va a HOME (15°/s), después la pinza copia la mano;
   soltar X = frena en el lugar; Y = giro on/off; gatillo = pinza abierta ↔ 50 %.
5. Medir precisión real: `/quest_teleop/status` → `track_mm`, `track_deg` (objetivo vs FK de lo medido).

## 4. Procesos fuera de lo normal (limpiar al retomar)

| Dónde | Proceso | Qué hacer |
|---|---|---|
| Jetson | `pan_tilt_node` lanzado a mano (`/tmp/pan_tilt_manual.log`), reemplaza al de `jetson-head` | dejar: al reiniciar `jetson-head` vuelve solo, con el mismo código |
| Jetson | `adb` server + regla udev `/etc/udev/rules.d/51-meta-quest.rules` | dejar (necesarios para el Quest) |
| Jetson | `ros-jazzy-control-msgs` y `adb` instalados por apt | rollback: `~/deploy_backups/{control_msgs,adb}_20261103/installed.txt` |

## 5. Tareas activas y bloqueos

| Quién | Tarea | Estado |
|---|---|---|
| Humano | Pan-tilt: revisar fuente de 12 V bajo carga, cableado del power hub del U2D2 y masa común; cambiar cable USB del U2D2 | **bloquea** el seguimiento de rostro |
| Humano | HOME-SEC: revocar el token (también está en el registro de Orca); contraseña de la Jetson | pendiente |
| Humano + Claude | **Prioridad: cualificación 2027, entrega 6/12/2026** (HOME-62, `docs/VIDEO_CUALIFICACION.txt`). Receptionist en el robot: seguimiento y carga corregidos, falta probarlos | en curso |
| Claude | Cuota agotada hasta el lunes 09/11 (pausa de ejecución; retomará al recuperarse) | **en pausa** |
| Antigravity | Toma coordinación técnica y desarrollo: completados rutina de puerta, cliente VLM (`vlm.py`), tests (`UNIT_PASS`), `graphify update` y vault de skills | **en curso** |
| Antigravity | HOME-65 ideas de papers, worktree `antigravity-home-65-papers`, base `af07316`, dispatch `ctx_b7bbeff893e3` | despachada; revisar con `git diff` y referencias al terminar |
| Antigravity | HOME-66 AnyDesk, worktree `antigravity-home-66-anydesk`, dispatch `ctx_0f1f8e86ab0a`; sudo por ayudante local | despachada; ambos frenados en una aprobación al 2026-11-05 |
| Claude (retoma) | En pausa, retomar después (detalle en `ROADMAP.md`): commander con Nav2 real y personas (HOME-57, HOME-49), docking (HOME-58), detector World en Jetson (HOME-59), `guide_person` (HOME-60), teleop (HOME-52) | guardado |
| Humano | TDP: seguir `docs/TDP_INSTRUCTIVO.txt` desde la otra computadora | pendiente |
| Claude | Receptionist: probar en el robot el barrido nuevo (2 filas, ~16 s), seguimiento leve y la revisión del pan-tilt | código listo (`SIM_PASS` parcial) |
| Claude | HOME-46 mesa en la escena MoveIt; HOME-43 correr `torso_state` con el ESP32 conectado | pendiente |

## 6. Cambios de esta sesión (detalle en Git)

- Puerta y VLM (2026-11-06): rutina de apertura de puerta de palanca (`door_open.py`, `abrir_puerta.py`, `door_open.yaml`, 6 tests passing en `test_door_open.py`); cliente VLM en `base_athome_skills/mission/vlm.py` (`ask_vlm`, `ask_vlm_cv`, 6 tests passing en `test_vlm.py`); grafo de conocimiento actualizado con `graphify update .` (5615 nodos, 11491 aristas); vault de skills creado en `~/claude-skills-vault` y enlazado. Sin cambios en archivos protegidos de navegación. Rollback: revertir el commit.


- Sacar la basura (HOME-63): "take out the trash" → la cabeza ubica el gesto y la bolsa → frenar lejos → `pre_grasp_floor` (`ensenar_pregrasp`; sin enseñar, una calculada) → avanzar con elevador arriba → bajar → agarrar con la cámara de muñeca y YOLO-World → punto `trash_drop`. `SIM_PASS` con MoveIt. Base del modelo a 22 cm (32 cm del piso, medido); columna y brazo sin cambios. Sin probar en el robot. Rollback: revertir el commit.

- Receptionist: el detector de muebles se carga en el pre-vuelo (bloqueaba la ventana ~4 s en medio del barrido) y el seguimiento de rostro responde más rápido (error máximo simulado de 16,6° a menos de 12° con un invitado que se mueve). Parámetros sin probar en el robot.

- Receptionist: espera ESPACIO al terminar de cargar (`--no-wait` lo omite; en competencia no se permite teclado, §3.6.8) y `P` repite la revisión del pan-tilt. Calibrador: `calibrar_pantilt`. `SIM_PASS`/tests con nodo falso; la cabeza real solo se leyó.
- **Incidente:** `commander` dejaba procesos vivos al cerrar la terminal (HUP sin trampa): el 04/11 quedaron 4 ejecutores reales y un Nav2 completo duplicando al de `navegacion`. Cerrados por grupo con el robot quieto. Arreglo: `session_lib.sh` cierra también con HUP y limpia restos de sesiones anteriores.

- Mapa: `map_server` solo lee el archivo al arrancar, así que un Nav2 abierto seguía con el mapa viejo. Ahora el editor lo recarga al guardar y `commander` al arrancar (`nav_map_sync.sh`, servicio `load_map`), sin cambios en archivos protegidos. Verificado el 2026-11-04: `/map` = PGM, los 200 obstáculos agregados son letales en el costmap global y AMCL no saltó.

- Commander + objetos: `places.py` (objetos y muebles), `command_parser.py` (número, nombre, clase; pide el número si hay ambigüedad), web con la lista numerada. Editor: aproximación frente al borde de la mesa, fuera de ella. Rollback: revertir el commit.
- **Incidente:** `test_semantic_map_manager_objects_integration` escribía los `waypoints` reales al llamar `save_data` (los vació en corridas del 03/11 y 04/11). Test aislado; archivos regenerados con door y living_room. Si existían otros waypoints solo en esos archivos, no se pueden recuperar.

- Commander (HOME-57, reemplaza HOME-40, cuyo worker terminó sin commits): `command_parser.py`, `commander_node.py`, `sim_commander.sh`. Rollback: revertir el commit; no toca al ejecutor.
- `0c282ec`…`1a901f3` — Teleop Meta Quest 2 (WebXR por `adb reverse`, sin WiFi): pose 6D con embrague, IK DLS (modelo DH
  del SDK, validado contra `piper_kinematics`), filtro de temblor, sensibilidad de giro, modo real con X de hombre
  muerto + HOME primero + freno al soltar, Quest despierto sin el visor, gatillo pinza abierta/50 %.
- `a4e67b5` — Pan-tilt: desenvoltura de vueltas (el tilt quedaba corrido 1 vuelta → sobrecarga HW 32), `pwm_limit`, `position_p_gain`.
- `c9e4121`, `e10d7f3`, `0e71811`, `f97e6f0`, `b236945` — Receptionist: revisión del pan-tilt al arrancar, centrado en la
  puerta, seguimiento leve del rostro, barrido en dos filas más bajo y lento (~16 s), sin frases de pantalla táctil.
- `2ed3295`, `42134a2`, `f16f457`, `ea3016a` — Manipulación: FollowJointTrajectory desplegado, pinza paralela a la
  trayectoria (arregla "trayectoria perdida"), objetivo retenido, agarre multiorientación (~22–25 cm), ACM por muestreo,
  estado del elevador, misiones compuestas (`demo_fetch_bottle` SUCCEEDED en sim).
- Incidentes: `move_group` del usuario terminado por error (corregido, la sim solo cierra lo propio); commit vacío tras reinicio (reparado).

## 7. Decisiones vigentes

| Fecha | Decisión |
|---|---|
| 03/11 | Modo `DESARROLLO`: autonomía de software con actuadores; movimiento físico solo con supervisión y sesión acordada. |
| 03/11 | Teleop del brazo real: siempre HOME primero y X como hombre muerto; el nodo nunca habilita el brazo. |
| 03/11 | Navegación: estudiar sí, modificar no. Solo ROS 2; sin Google ni pagos. |
