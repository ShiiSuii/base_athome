# Revisión HRI y micrófono del Receptionist (30-10-2026)

Síntoma reportado: "siempre pasa lo mismo".
- Dice "I could not hear you" sin haber escuchado.
- La segunda vez que recibe a alguien, el micrófono "deja de funcionar".
- Las respuestas tardan entre 5 y 10 s.
- El tracking es demasiado agresivo.

Este documento reúne:
- las causas encontradas y su corrección;
- qué hacen otros equipos;
- la nueva modalidad sin audio (pantalla táctil);
- lo que todavía queda pendiente.

## 1. Causas encontradas y corregidas

| # | Bug | Causa raíz | Corrección | Commit |
|---|---|---|---|---|
| 1 | "I could not hear you" antes de escuchar | `state_start_time = now` se tomaba **antes** de los `say()` bloqueantes. Al volver del habla, el timeout de escucha ya había vencido | Las 22 transiciones usan `time.time()` después de hablar | 35aee85 |
| 2 | Latencia de 5 a 10 s; respuestas con texto viejo | Whisper transcribía **continuamente** el ruido de la sala y bloqueaba la CPU. El *fallback* de temperatura reintentaba hasta 5 veces | Escucha **por pedido** (`/voice/listen_trigger`) con ventana fija de 6 s; Silero VAD; `temperature=0`; `without_timestamps`; `max_new_tokens=64`; una transcripción a la vez. Una ventana con solo ruido bajó de 3,07 s a 0,04 s | 398f876 |
| 3 | Transcripciones como "Robin, Charlie, Jordan…" sin que nadie hablara | Con audio vacío, Whisper repite su `initial_prompt` (*prompt echo*) | Filtro `prompt_echo` en `whisper_node` | 398f876 |
| 4 | Micrófono "muerto" en el 2.º invitado | El índice de la tarjeta RØDE cambia entre arranques y la ganancia quedaba baja. El *mute* durante el TTS cortaba la ventana | `set_max_gain()` busca la tarjeta por nombre y reafirma el 100 % cada 30 s. *Failsafe* de *mute* (25 s como máximo). Se registra en el log cuando el *mute* corta una ventana. *Watchdog* de audio | 398f876 / 1dbd6d7 |
| 5 | Orden del diálogo confuso | Pedía datos antes de presentarse y sin decir qué responder | Orden: (1) quién es el robot, (2) frase exacta a decir, (3) beep, (4) 6 s de escucha. El beep suena **antes** de abrir el micrófono para no grabarlo | 1dbd6d7 |
| 6 | Tracking agresivo | El control pan-tilt perseguía cada detección | Tracking desactivado por defecto; la cabeza se centra al saludar. `--track-faces` activa un modo suave | 35aee85 |
| 7 | **"CHALLENGE COMPLETADA" repetido cada ~10 s** (lo encontró la simulación) | `STATE_CHALLENGE_COMPLETE` nunca cambiaba de estado y se re-ejecutaba en cada ciclo | Nuevo estado terminal `STATE_DONE` | este commit |
| 8 | **El invitado no se reconocía después** (lo encontró la simulación) | `assess_face_quality` descarta rostros < 50 px. Con el invitado lejos (más de 2 m) no había ninguna muestra válida y, al vencer el tiempo, se guardaba un **vector aleatorio** mientras el robot decía "I have enough details" | Si no hay muestras válidas a los 3,5 s, el robot pide "come a little closer" y da más tiempo. Al vencer el tiempo se usan las muestras de baja calidad antes que un vector aleatorio. Si no hay ninguna, lo registra como ERROR | este commit |
| 9 | **El invitado presentado como "Charlie"** (lo encontró la simulación) | En la sala, "la única persona sin nombre" se asumía anfitrión, aunque fuera el invitado parado junto al robot | El anfitrión se elige primero entre personas **sentadas**. Solo si no hay nadie sentado se usa la regla anterior, con un WARN | este commit |

Con la simulación, el flujo completo de 1 invitado pasó de "1 muestra aleatoria, Charlie y un
cierre repetido" a "10 muestras, Robin reconocido en la sala y un solo cierre". La salida real
está en `docs/SIMULACION.md`.

## 2. Qué hacen otros equipos (fuentes públicas revisadas)

- **fbot@Home** (`fbotathome/fbot_hri`):
  - escucha como **servicio por pedido** (`/fbot_speech/sr/speech_recognizer`, con `prompt` e idioma);
  - modelo `distil-small.en` en CPU;
  - VAD doble, Silero (0,6) más WebRTC;
  - termina la grabación por **silencio de 1,2 s**, con grabación mínima de 3 s y *timeout* de micrófono de 10 s;
  - `initial_prompt` con la **lista oficial de nombres** (archivo `fbot_hri_bringup/config/fbot_stt_recepcionist.yaml`);
  - *hotword* opcional (`/fbot_speech/bhd/detected`).
- **RoBorregos FRIDA** (`RoBorregos/home2`, `hri/packages/speech/scripts/`), con una cadena de nodos:
  - `audio_capturer`;
  - `noise_cancellation` (DeepFilterNet, que se apaga solo si no corre en tiempo real);
  - `voice_detection` (umbral de energía + VAD, `SILENCE_LIMIT` de 1,5 s, y similitud para descartar el eco del propio TTS);
  - `kws_oww` (palabra clave con openWakeWord);
  - `audio_feedback` (sonido al pasar a "escuchando");
  - `respeaker` (dirección de llegada de la voz).
- **Práctica común en receptionist** (equipos de OPL y DSPL):
  - preguntar nombre y bebida (en varios equipos, por separado);
  - **comparar contra el vocabulario oficial** en lugar de aceptar texto libre;
  - confirmar con sí o no;
  - usar un medio alternativo (pantalla o tablet) cuando el ruido del estadio impide el ASR.

**Qué ya adoptamos:**
- escucha por pedido;
- Silero VAD;
- `initial_prompt` con el vocabulario (`guest_vocabulary.whisper_prompt()`);
- *matching* contra la lista oficial (`vocab.match_name` / `match_drink`);
- beep antes de escuchar;
- filtro de eco del *prompt*.

**Qué falta, priorizado** (ver la sección 4):
- cortar la ventana por silencio en lugar de la ventana fija de 6 s;
- descartar el eco del TTS;
- reducir el ruido.

## 3. Nueva modalidad sin audio: pantalla del invitado

Archivos: `base_athome_perception/guest_input_server.py` (servidor web sin dependencias) y su
integración en `hri_receptionist_challenge.py`.

- **Dónde:** `http://127.0.0.1:8090` en la pantalla del robot (alias `pantalla_invitado`), o desde
  un celular en la misma red, en `http://<IP NUC>:8090`.
- **Qué muestra:**
  - la pregunta actual del robot (se actualiza sola);
  - botones grandes con los **10 nombres y 12 bebidas oficiales** (`tools/gpsr_parser/data`);
  - un campo "Other" para escribir;
  - botones YES / NO.
- **Cómo llega al robot:** cada toque llega por `on_input` y se publica en `/hri/guest_input` como
  JSON: `{"type": "name|drink|answer", "value": "..."}`. Otro nodo, por ejemplo el secuenciador de
  misiones, puede usarlo.
- **Prioridad:** en `WAIT_NAME_DRINK`, lo tocado en la pantalla **tiene prioridad sobre la voz** y
  pasa por el mismo *matching* de vocabulario. El saludo agrega "You can also tap your answer on my
  screen."
- **Prueba en la simulación:** se tocó "Jordan" y "Milk", y el robot guardó al invitado sin
  micrófono.
- **Reglamento:** verificar en el rulebook 2026.2 si la entrada táctil puntúa igual que la voz.
  Úsenla como **respaldo** cuando el ASR falle dos veces. No la presenten como modo principal sin
  confirmarlo.

## 4. Pendientes recomendados (no implementados)

1. **Fin de habla por VAD en `whisper_node`:** cortar a 1,2 s de silencio, con mínimo de 1,5 s y
   máximo de 8 s, como fbot. Resta entre 2 y 4 s por turno. Cambio solo en la Jetson y reversible.
2. **Descartar el eco del TTS:** silenciar mientras habla ya existe (`/voice/mute`). Falta comparar
   la transcripción con la última frase dicha por el robot, como hace FRIDA.
3. **Reducción de ruido** (DeepFilterNet o *noisereduce*) **si** el CPU de la Jetson alcanza.
   Medir con `jetson_stats`.
4. **Confirmación sí/no por la pantalla** cuando la similitud con el vocabulario sea baja.
5. **Grabaciones de prueba** (como `hri/benchmarks/stt` de FRIDA) para medir la precisión de ASR
   con el RØDE real y ruido de fondo. La simulación no cubre el audio.
