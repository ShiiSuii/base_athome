# RoboCup@Home 2026.2 — análisis del reglamento para Base@Home

Fuente: `rulebook.pdf` del release oficial **2026.2** (publicado 2026-05-11, "Version 2026 Rev-1, revision 2026-05-11"), https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2 — sha256 `d35e93c7f291a59a823691905b47dcd7e8de14ab3a4c546aca55ba20b30ce128`. Leído completo el 2026-10-30. Las citas de sección refieren a ese PDF.

## 1. Cambios de fondo que afectan nuestro plan

- **Una sola liga: OPL** (sub-ligas disueltas, §1.5, B.8).
- **Pruebas 2026: HRI Challenge, Pick and Place, GPSR, Doing Laundry, Restaurant + Finals** (§5, §6). **"Give Me a Hand" y "Help Me Carry" NO existen en 2026.2.** El backlog (`AGENTS.md` P2, `TASKS.md` HOME-13) los menciona → reemplazar HOME-13 por Doing Laundry o descartarlo.
- **Cada día se corren las mismas 4 pruebas** (la mejor puntuación de cada una cuenta, §3.5.1-2); Restaurant se corre **una sola vez**, fuera de la arena y sin mapa previo.
- Todas las pruebas regulares duran **7:00 min** (Restaurant 15:00, Final 10:00).
- Finalistas: top 3 con ≤9 equipos (§3.5.1). Ranking final = 50 % Final + 50 % suma de puntos (§6.1).

## 2. Requisitos generales con impacto técnico directo

| Regla | Qué exige | Impacto en Base@Home |
|---|---|---|
| §3.3.2 Paro de emergencia | Botón rojo, único rojo, **todo** se detiene al instante | Piper, pan-tilt, elevador y base deben cortar con el E-stop. Hoy el software de la Jetson auto-habilitaba actuadores (S-01/S-03, corregido en rama). Verificar cadena eléctrica del E-stop (PENDIENTE en `power-domains.md`). |
| §3.3.2 Marcas | "must not display any artificial marks or patterns" | El marcador ArUco de calibración no puede quedar montado en el robot en las pruebas. |
| §3.3.2 Obstacle avoidance obligatorio, inspección con persona delante | Nav2 (sin tocar) debe frenar ante una persona | Verificar en ensayo, solo lectura. |
| §3.2.8 / B.3 Red | Solo red de la arena (VLAN por equipo); **hotspots prohibidos = descalificación**; la red puede fallar | El enlace NUC↔Jetson debe ser **Ethernet cableado** (no WiFi); revisar que ninguna máquina emita hotspot (NUC tiene perfiles WiFi `Rescue` etc.; Jetson tiene `l4tbr0`/USB gadget — confirmar que no emiten WiFi). |
| §3.4 External computing | ≤5 equipos externos, solo por red de arena, aviso al TC un mes antes; el robot debe operar seguro sin ellos. Se permiten APIs en la nube | Todo lo crítico on-board. Política del proyecto: nada de APIs pagas. |
| §3.6.8 Start signal | Apertura de puerta (robot fuera) o señal manual aprobada (sin mouse/teclado) | Implementar detección de puerta abierta con el Hokuyo y un "start button" (p. ej. soltar E-stop o botón en pantalla táctil). Usar señal alternativa cuesta −100 en varias pruebas. |
| §3.6.9 Restart | 1 por intento, solo antes de 40 pts, 60 s | Scripts de arranque rápidos e idempotentes (el robot debe estar listo <1 min, §3.6.14). |
| §3.6.14 Inactividad | 30 s quieto = retirado; puede pedir 1 extensión ≤3 min | El secuenciador debe hablar/actuar y pedir tiempo explícitamente durante procesos largos (p. ej. inferencia). |
| §3.7 Asistencia humana | Último recurso; el robot debe pedirla explícitamente y **verificar** el resultado | Diseñar fallbacks "pedir ayuda + verificar" en el secuenciador. |
| §3.7.4 ASR | Operador por defecto sin penalidad; custom −10 %; gestos −20 %; interfaz alternativa hasta −20 % | Whisper en la Jetson es pieza central; tener interfaz táctil alternativa aprobada como plan B. |
| §4.4 Inspección | E-stop, evitar persona, voz alta y clara, dispositivos externos, velocidad/dimensiones, señal de inicio, 3 min | Preparar checklist de inspección (ver §5). |

## 3. Pruebas: puntos máximos y lo que pide cada una

| Prueba | Máx. | Habilidades clave | Estado del robot (código actual) |
|---|---|---|---|
| **HRI Challenge** (§5.1) | 1450 | timbre, abrir puerta, mirar al que habla y hacia donde navega, ofrecer asiento, describir al 1er invitado, presentar, recibir bolsa por handover, **seguir al anfitrión**, sin preguntas de confirmación | Receptionist existente (NUC) + pan-tilt + Whisper. Análisis de brechas: tarea HOME-06a (Antigravity) |
| **Pick and Place** (§5.2) | 3515 | ordenar mesa (lavavajillas, basura, alacena por categoría), servir desayuno, opcional: abrir lavavajillas, servir leche/cereal | Piper + Orbbec eye-in-hand; agarre de Pringles ya prototipado. **La prueba con más puntos**; los "common objects" (Red Bull, Pringles, cubo Rubik, Colgate) se pueden entrenar desde ya |
| **GPSR** (§5.3) | 1490 | entender 3 comandos (generados con CommandGenerator y re-frasados por LLM), mostrar plan, ejecutar; bonus 200 por intercalarlos | No hay parser de comandos; HOME-15 |
| **Doing Laundry** (§5.4) | 4415 | sacar remeras de canasto/lavarropas, doblar, apilar | Requiere bimanual en gran parte; con un solo Piper: sacar de lavarropas (300), levantar prenda (100) e intentar un doblez simple (800) |
| **Restaurant** (§5.5) | 2360 | detectar cliente que llama/saluda, navegar en lugar desconocido (sin mapa previo), tomar pedido, llevar 2 objetos | Requiere SLAM online (existe slam_toolbox en el repo, sin tocar Nav2) + gestos |
| **Final** (§6) | 8450 (sin máx.) | encontrar y resolver ≥8 problemas: basura al tacho, objetos fuera de lugar, personas con pedidos (GPSR), cerrar lavavajillas, mover canasto, **abrir puerta** (600) | Integración de todo lo anterior |

### Puntos "baratos" para un robot con un brazo

1. **Pick and Place**: navegar a la mesa (15) + reconocer objetos (12×10) + primer agarre (+100) + agarres (50 c/u) + colocar (40 c/u). Un agarre-colocación correcto de 2–3 objetos ya suma ~400–500.
2. **HRI**: mirar a quien habla (2×50), ofrecer asiento (2×100), presentar con nombre/bebida (4×30), **no hacer preguntas de confirmación** (4×15), mirar al invitado correcto (2×50), seguir al anfitrión (200). Todo es software sobre hardware que ya existe.
3. **GPSR**: entender el comando (3×80) + mostrar un plan (3×100) = 540 sin manipular, si el parser es bueno.

## 4. Brechas principales detectadas (preliminar)

1. `STATE_CONFIRM_DETAILS` hace una **afirmación** ("I noted that..."), no una pregunta → el bonus 4×15 se conserva; el riesgo es memorizar un dato mal transcrito (−40 c/u). Detalle completo en `docs/hri_challenge_2026_gap_analysis.md` (HOME-06a): faltan bolsa+seguir anfitrión (300), atributo visual (80), timbre (60), bebida y mirada en la presentación.
2. **Sin detección de timbre** (2×30) — existe `socrob/doorbell_detector` (YAMNet, ROS 2) reutilizable, falta verificar licencia.
3. **Seguir al anfitrión** (200) — hay `calibrate_tracking.py` y tracking de personas; falta el comportamiento follow con Nav2 (como cliente, sin modificar Nav2).
4. **Handover de bolsa** (50) con el Piper — requiere modo seguro (el robot no debe "atacar" la mano, §5.1 regla 5).
5. **GPSR**: no existe parser. Usar el `CommandGenerator` oficial para generar datos de prueba offline.
6. **Señal de inicio por puerta abierta** — no implementada (necesaria en 4 de 5 pruebas).
7. **Restaurant**: SLAM online + detección de saludo/gestos.
8. **Doing Laundry**: no abordado.

## 5. Checklist de inspección (borrador para preparar)

- [ ] E-stop rojo único, corta base + brazo + elevador + pan-tilt (verificación eléctrica, PENDIENTE)
- [ ] Sin cables sueltos, sin cinta, sin bordes filosos; robot ≤ 200×70 cm
- [ ] Freno ante persona delante (Nav2, ensayo)
- [ ] Voz alta y clara; salida de audio con plug para transmisión RF
- [ ] Señal manual de inicio aprobada (botón/soltar E-stop) y detección de puerta abierta
- [ ] Ninguna red WiFi propia emitida (NUC y Jetson)
- [ ] Sin marcadores ArUco ni patrones artificiales visibles
- [ ] Lista de dispositivos externos (si se usa laptop) informada al TC con un mes de anticipación

## 6. Otros puntos útiles

- **Premio Open-source software** (§1.7.6): biblioteca independiente del framework, documentada, con paper de 4 páginas un mes antes. Candidato natural: el editor URDF paramétrico (HOME-04/05) si se generaliza.
- **Skill certificates** (§1.7.5): ≥75 % de los puntos de una habilidad en todas las pruebas.
- **Calificación** (§3.1): TDP de 8 páginas + video ≤10 min resolviendo al menos una prueba + web en inglés con código abierto (se valora).
- Fechas de RoboCup 2026: 30-jun a 6-jul-2026, Incheon (ya pasó); este análisis aplica para entrenamiento y competencias locales/abiertas que usen 2026.2. **El reglamento 2027 no está publicado como release**; `master` no es normativo.
