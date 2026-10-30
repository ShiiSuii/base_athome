# Dominios de alimentación

2026-10-29. **No hay evidencia eléctrica verificada.** No se infieren tensiones, fusibles ni pines.

| Dominio | Cargas | Estado declarado por el usuario | Evidencia |
|---|---|---|---|
| Cómputo NUC | NUC | ENCENDIDO | NUC responde (confirmado) |
| Cómputo Jetson | Jetson + periféricos USB | ENCENDIDO | SSH OK; USB enumerados |
| Tracción | motores hoverboard / controlador | APAGADO | PENDIENTE |
| Brazo | Piper (CAN) | APAGADO | PENDIENTE |
| Elevador | NEMA23 + driver + ESP32-S3 | APAGADO | PENDIENTE |
| Cabeza | 2× XL430 (12 V según log del nodo: *"Ensure 12V power is turned on"*) | APAGADO | VERIFICADO_CODIGO (mensaje), no medido |

Preguntas para el equipo: batería/BMS, fusibles por rama, paro de emergencia y qué dominios corta, reguladores para Jetson/NUC.

**Nota de seguridad:** el software de la Jetson arranca drivers de actuadores automáticamente (S-01, S-03 en `software-audit.md`); energizar un dominio con el enlace Ethernet restaurado puede habilitar torque sin acción humana hasta desplegar los fixes.
