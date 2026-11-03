# Micrófono y ASR: cómo escucha el robot (rediseño del 03-11-2026)

Todo lo que el robot hace por voz depende de esta cadena:
- recibir invitados (receptionist);
- recibir órdenes (secuenciador de misiones, GPSR);
- registrar personas (test de rostros).

Este documento explica:
- qué fallaba;
- cómo funciona ahora;
- cómo probarlo;
- qué hacer cuando falla.

Manual especializado de operación (se conserva aparte de `docs/ROBOT.md` porque se usa para probar,
diagnosticar y revertir la cadena de voz). Interfaces y arquitectura general: `docs/ROBOT.md`.

## 1. Qué fallaba (con evidencia)

| Problema | Evidencia | Efecto |
|---|---|---|
| **Ventana fija de 6 s** desde el pedido | `whisper_node` grababa siempre 6 s; frase de 3,3 s → transcrita a los 6 s + 1,7 s | Respuesta lenta. Si el invitado empezaba tarde o hablaba largo, la frase se cortaba o se perdía |
| **Compuerta "voz = WebRTC VAD + energía > umbral"** | Log del 30-10: `Escuchando ventana fija de 6.0 s...` → `Ventana de escucha terminada sin voz.` | Con el invitado lejos o hablando bajo, el audio se tiraba **sin transcribirlo** → "I could not hear you" |
| **Pedido y texto sin identificador** | Cada app tomaba "el próximo texto" de `/voice/transcription` con un timeout fijo (ventana + 5 s) | Un texto atrasado contestaba la pregunta siguiente. Sin voz, la app solo se enteraba por timeout. Con la Jetson caída, el robot repetía "I could not hear you" sin decir que el micrófono no funcionaba |
| **Transcripción descartada si había otra en curso** | `transcribe_lock.acquire(blocking=False)` → "se descarta este audio" | Respuesta perdida en silencio |
| **El *mute* cancelaba la escucha** | Si el robot hablaba durante la ventana, se hacía `reset_state()` | Pedido perdido |
| **Vocabulario equivocado al escuchar órdenes** | `mission_sequencer` publicaba siempre el *prompt* de nombres y bebidas | Whisper sesgado hacia nombres al escuchar "go to the living room" |
| **Interpretación frágil** | "ice tea" → `Tea`; "Jewels" no era Jules; "Know that is wrong" (por "No, that is wrong") no era un NO | Datos mal guardados y sin confirmación |

## 2. Cómo funciona ahora

### Jetson: `jetson_head/src/audio_asr/` (`whisper_node.py` + `asr_core.py`)

1. **Captura continua** del RØDE a 48 kHz → 16 kHz, en bloques de 32 ms.
   - **Silero VAD en *streaming*** sobre cada bloque. Viene con faster-whisper; medido: 0,34 ms por bloque en la Orin Nano.
   - El nodo estima el ruido de fondo de forma continua.
2. **Pedido de escucha con id:**
   - Formas de pedirlo:
     - tópico `/voice/listen_request`, con JSON `{"id", "prompt", "start_timeout", "end_silence", "max_speech"}`;
     - o el servicio `/voice/listen_trigger` de siempre, cuya respuesta trae `{"id"}`.
   - Un pedido nuevo reemplaza al anterior, que termina como `cancelled`.
3. ***Endpointing*:**
   - espera a que la persona **empiece** a hablar, hasta `start_timeout` (8 s);
   - graba mientras habla;
   - corta cuando hay **1,1 s de silencio**, como fbot (1,2 s) y FRIDA (1,5 s);
   - máximo 15 s de frase.
   - **Pre-roll:** si la persona ya estaba hablando al llegar el pedido, se conserva el comienzo.
   - **Voz débil** (invitado lejos): Silero ≥ 0,12 con energía ≥ 4 veces el ruido cuenta como voz. Se transcribe con un VAD más permisivo.
4. **Mientras el robot habla** (`/voice/mute`):
   - esos bloques se anulan, pero **el pedido sigue abierto**;
   - la espera de `start_timeout` se pausa.
5. **Transcripción:**
   - con cola: nunca se descarta audio;
   - `base.en` int8 en CPU (comparación abajo), *beam* 3, con el *prompt* de **cada pedido**.
   - Filtros: alucinaciones ("Thank you."), eco del *prompt*, repeticiones en bucle ("Robin, Robin, Robin…") y etiquetas `[MUSIC]`.
6. **Resultado SIEMPRE explícito** en `/voice/result`:
   - eventos `listening` → `speech_start` → `speech_end` → `result`;
   - `status` puede ser `ok`, `no_speech`, `rejected`, `cancelled` o `error`, con tiempos y motivo.
   - Además, el texto se publica en `/voice/transcription` para mantener compatibilidad.
7. **Ganancia automática:** baja un 10 % si la voz satura (pico ≥ 0,97) y sube si llega débil. El rango es 60–100 %.
8. **`/voice/debug_inject`** (solo para pruebas): mezcla un WAV con el micrófono **en vivo**, como si alguien hablara delante del robot.

### NUC: `base_athome_perception/voice_client.py` (cliente único)

```python
self.voice = VoiceClient(node)
rid = self.voice.request(prompt=vocab.whisper_prompt())   # después del beep
res = self.voice.poll(rid)      # None mientras escucha; luego VoiceResult(status, text, info)
res = self.voice.listen(prompt) # versión bloqueante (para hilos propios)
```

- **Estado `offline`** en 1,5 s si el ASR no publica `/voice/status`, es decir, si la Jetson o el nodo están caídos. La app avisa al invitado y ofrece la pantalla.
- **Teclado y pantalla:** `inject(text)` responde el pedido abierto por el mismo camino que la voz. Lo usa la tecla [T] del receptionist.
- **Nodo viejo en la Jetson** (sin `/voice/result`): usa el modo anterior (Trigger + `/voice/transcription`) sin romperse.

### Interpretación (`guest_vocabulary.py`, `guest_dialog.py`)

- **Nombres:**
  - en la posición de nombre ("my name is X", "I'm X", "call me X"), tolerancia alta;
  - comparación por texto **y fonética** ("Jewels" → Jules, "Axle" → Axel, "Adele" → Adel, "rob in" → Robin);
  - alias en `NAME_ALIASES`.
- **Bebidas:**
  - n-gramas de 3, 2 y 1 palabras, con la más específica primero ("iced tea" antes que "tea");
  - alias (`DRINK_ALIASES`: "ice tea", "coke" → Cola, …);
  - en la posición de bebida ("my favorite drink is X") se acepta más error.
- **Sí/no:** `parse_yes_no()`. Incluye errores típicos de Whisper ("Know that is wrong" → NO) y frases hechas ("that's right").

### Apps

- **Receptionist:**
  - cada escucha usa `VoiceClient` y decide por el `status` (ya no por un timeout);
  - `ok` → interpreta; un solo dato → pide solo el que falta;
  - `no_speech` → "I could not hear you. Please come a little closer…";
  - `offline` → "I am having trouble with my microphone. Please tap… on my screen";
  - después de 2 fallos se ofrece la pantalla.
  - **Nueva confirmación por sí/no:** "Let me check. Your name is X, and your favorite drink is Y. Is that correct?". Con NO, vuelve a preguntar. Si en 2 intentos no hay respuesta clara, sigue con lo entendido. Lo tocado en la pantalla no necesita confirmación.
- **Secuenciador de misiones:**
  - escucha órdenes con un ***prompt* de órdenes** (lugares y objetos del mapa semántico), no con el de nombres;
  - confirma ("I understood: go to the living room. Is that right?");
  - avisa si el micrófono está caído.
- **`test_face_recognition`:** usa `VoiceClient` (resultado explícito en lugar de 8 s fijos).
- **Simulación (`base_athome_sim`):** imita el mismo protocolo (id, `/voice/result`, `no_speech`, espera pausada con *mute*).

## 3. Mediciones (salida real, 03-11-2026)

**Modelos en la Jetson (CPU, int8)**

17 frases sintéticas, sin camino acústico:

| Modelo | s/frase | Exactas | Nota |
|---|---|---|---|
| **base.en** (elegido) | **1,50** | 15/17 | Los 2 errores ("Know that is wrong", "and to bring") los resuelve la interpretación |
| small.en | 4,50 | 16/17 | 3 veces más lento |
| distil-small.en | 4,05 | 11/17 | Repeticiones en bucle ("Jules, Jules, Jules…") |

**Camino completo con voz inyectada sobre el micrófono real**

Comando: `asr_benchmark --inject`. Son 16 frases: 10 nombres oficiales con bebidas, 2 frases libres, 4 de sí/no.

| Condición | Entendidas | Demora desde que termina de hablar |
|---|---|---|
| Volumen normal | **16/16** | mediana 2,5 s (antes: lo que faltaba de los 6 s + 1,7 s) |
| Voz al 25 % (≈ 2–3 m) | **16/16** | 2,5 s |
| Voz al 8 % (≈ nivel del ruido) | **16/16** | 2,5 s |
| Empieza a hablar a los 5 s | **16/16** | 2,5 s |
| Voz al 3 % (bajo el ruido) | 10/16 | entiende partes y pide lo que falta |

**Prueba de *mute* con el nodo real**

| Caso | Resultado |
|---|---|
| Voz mientras el robot habla | Ignorada; la espera quedó en pausa (resultado a 7,1 s = 4 s de *mute* + 3 s) |
| Voz después de 5 s de robot hablando | Escuchada (`ok`) |
| Sin voz | `no_speech` exactamente a los 3,0 s |

**Simulación (dominio 42)**

- **Receptionist** con invitado automático:
  - Robin / Iced Tea entendidos;
  - confirmación "yes" → True;
  - rostro memorizado.
- **Secuenciador de misiones:**
  - "Go to the living room" → confirmación → "Yes, that is correct." → misión completada;
  - `no_speech` → vuelve a preguntar.

**Tests**

- `audio_asr/test/test_asr_core.py`: 12 tests.
- `base_athome_perception/test`: 111 tests, incluidos `test_voice_client.py` y `test_voice_understanding.py`.
- `base_athome_sim/test`: 6 tests.

## 4. Cómo probar (alias de la NUC)

```bash
escuchar --n 5          # una persona real: beep, habla, y muestra texto, nombre/bebida, sí/no y tiempos
prueba_microfono        # 16 frases mezcladas con el ruido REAL del RØDE (sin parlante)
prueba_microfono --gain 0.25 --late   # voz lejana y que tarda en empezar
microfono_logs          # log en vivo del ASR en la Jetson
voz                     # /voice/result: cada escucha con su id, estado y tiempos
```

> El parlante HDMI de la pantalla **no sirve** para probar el camino acústico: se suspende y al
> despertar se come el comienzo, y el sonido que llega no es inteligible (medido). Usar
> `prueba_microfono` (inyección) o una persona real (`escuchar`).

## 5. Diagnóstico rápido

| Síntoma | Qué mirar |
|---|---|
| El robot dice "trouble with my microphone" | `head_status`; `ros2 topic echo /voice/status`. `NO_MIC` = RØDE desconectado (`arecord -l` en la Jetson); sin mensajes = `jetson-head` caído o red |
| "I could not hear you" con alguien hablando | `microfono_logs`: la línea `sin voz (... VAD max X, RMS max Y, ruido Z)`. RMS max ≈ ruido → la persona está lejos o el micrófono apunta a otro lado (el VideoMic GO II es direccional: apuntarlo al invitado) |
| Entiende mal un nombre | Agregar la variante a `NAME_ALIASES` en `guest_vocabulary.py`, con el texto real del log |
| Lento | `microfono_logs`: `ASR x.xx s`. Si supera 2,5 s, revisar la carga de la CPU (`jetson_stats`), porque YOLO TensorRT comparte la Jetson |

## 6. Rollback

- **Jetson:** copia anterior en `~/deploy_backups/audio_asr_20261103_094455/` (en la Jetson). Para volver:
  ```bash
  cp -a ~/deploy_backups/audio_asr_20261103_094455/audio_asr/* ~/athome_ws/src/audio_asr/audio_asr/
  ```
  Después, `colcon build --packages-select audio_asr` y `sudo systemctl restart jetson-head`.
- **NUC:** `git revert` del commit. `VoiceClient` funciona también con el nodo viejo (modo compatible).

## 7. Qué NO se probó

- **Una persona real hablando al RØDE con el nodo nuevo.** El camino acústico con el parlante HDMI no fue confiable. Se validó con voz inyectada sobre el ruido real del micrófono. Primer paso con el robot: `escuchar --n 10` con 2–3 personas a 1 m y a 2 m.
- Ruido de competencia (público, música). Para eso: grabaciones en `--wav-dir`.
- El receptionist y las misiones en el robot real con estos cambios (solo en simulación).
- La ganancia automática con saturación real.

## 8. Historial de correcciones del receptionist (30-10-2026)

Síntomas reportados entonces: "I could not hear you" sin haber escuchado, micrófono "muerto" con el
2.º invitado, respuestas de 5–10 s, tracking agresivo.

| Bug | Causa | Corrección | Commit |
|---|---|---|---|
| "I could not hear you" antes de escuchar | el tiempo de escucha se medía antes de los `say()` bloqueantes | las transiciones usan el tiempo posterior al habla | `35aee85` |
| Latencia 5–10 s y textos viejos | Whisper transcribía continuamente el ruido | escucha por pedido, Silero VAD, `temperature=0`, una transcripción a la vez (ventana de ruido 3,07 s → 0,04 s) | `398f876` |
| Nombres transcritos sin que nadie hable | eco del `initial_prompt` con audio vacío | filtro `prompt_echo` | `398f876` |
| Micrófono "muerto" en el 2.º invitado | el índice ALSA del RØDE cambia y la ganancia quedaba baja; el *mute* cortaba la ventana | ganancia por nombre de tarjeta reafirmada cada 30 s; failsafe de *mute* (25 s) | `398f876`, `1dbd6d7` |
| Diálogo confuso | pedía datos antes de presentarse | orden: quién es el robot → frase a decir → beep → escucha | `1dbd6d7` |
| Tracking agresivo | seguía cada detección | desactivado por defecto; `--track-faces` modo suave | `35aee85` |
| Cierre repetido cada ~10 s | estado final sin salida | `STATE_DONE` | `01ecbf7` |
| Invitado guardado con vector aleatorio | rostros < 50 px descartados | pide acercarse; usa muestras de baja calidad; si no hay, ERROR | `01ecbf7` |
| Invitado presentado como "Charlie" | anfitrión = "persona sin nombre" | anfitrión elegido entre personas sentadas | `01ecbf7` |

## 9. Pantalla del invitado (modalidad sin audio)

- `base_athome_perception/guest_input_server.py` en `http://127.0.0.1:8090` (alias `pantalla_invitado`)
  o desde un celular en `http://<IP NUC>:8090`.
- Muestra la pregunta actual, botones con los **10 nombres y 12 bebidas oficiales**
  (`tools/gpsr_parser/data`), campo "Other" y YES/NO. Publica `/hri/guest_input` como
  `{"type": "name|drink|answer", "value": "..."}`; `VoiceClient.inject()` responde el pedido abierto.
- En `WAIT_NAME_DRINK` lo tocado tiene prioridad sobre la voz y pasa por el mismo *matching*.
- Reglamento: la interfaz alternativa penaliza hasta −20 % → usarla como **respaldo** tras 2 fallos de ASR.

## 10. Qué hacen otros equipos (fuentes públicas)

- **fbot@Home** (`fbotathome/fbot_hri`): STT como servicio por pedido con `prompt`; `distil-small.en`
  en CPU; Silero + WebRTC; fin por 1,2 s de silencio; `initial_prompt` con la lista oficial de nombres.
- **RoBorregos FRIDA** (`RoBorregos/home2`, `hri/packages/speech`): captura → DeepFilterNet →
  detección de voz (1,5 s de silencio, descarte del eco del TTS) → openWakeWord → sonido de "escuchando".
- **Práctica común:** comparar contra el vocabulario oficial, confirmar con sí/no, pantalla de respaldo.
- **Pendiente para nosotros:** reducción de ruido (DeepFilterNet o *noisereduce*) solo si la CPU de la
  Jetson alcanza, medido con ruido real de competencia.
