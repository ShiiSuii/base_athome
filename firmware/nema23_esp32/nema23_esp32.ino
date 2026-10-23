/*
 * ==============================================================================
 * NEMA 23 Stepper Motor Firmware for ESP32-S3 (Elevator with Dual Endstops)
 * Pin 7 -> Sensor SUPERIOR (TOP / ARRIBA)
 * Pin 8 -> Sensor INFERIOR (BOTTOM / ABAJO)
 * Baudrate: 115200 baud
 * ==============================================================================
 */

#include <Arduino.h>

// ==========================================
// PIN CONFIGURATION (Safe ESP32-S3 GPIOs)
// ==========================================
// GPIO 4  -> PUL+ / STEP (PUL- to GND)
// GPIO 5  -> DIR+ / DIRECTION (DIR- to GND)
// GPIO 6  -> ENA+ / ENABLE (ENA- to GND) [Opcional]
// GPIO 7  -> SENSOR FINAL DE CARRERA SUPERIOR (TOP / ARRIBA)
// GPIO 8  -> SENSOR FINAL DE CARRERA INFERIOR (BOTTOM / ABAJO)
const int PIN_STEP          = 4;
const int PIN_DIR           = 5;
const int PIN_ENABLE        = 6;
const int PIN_LIMIT_TOP     = 7;  // Sensor SUPERIOR (Arriba)
const int PIN_LIMIT_BOTTOM  = 8;  // Sensor INFERIOR (Abajo)

// Logica del pin Enable en drivers industriales (TB6600/DM542):
// LOW habilita el puente de potencia, HIGH apaga el torque (eje libre).
const bool ENA_ACTIVE_LEVEL = LOW; 

// ==========================================
// PARAMETROS DE MOTOR Y DRIVER
// ==========================================
const uint32_t STEPS_PER_REV   = 200;   // Motor NEMA 23 estandar (1.8 deg/paso)
uint32_t       microstepping   = 8;     // 8 microsteps = 1600 pasos/vuelta
const uint32_t PULSE_WIDTH_US  = 4;     // Ancho de pulso HIGH (4us ideal para 25kHz en optoacoplador)

// Limites de velocidad (retraso entre pasos en microsegundos)
const uint32_t MIN_STEP_DELAY_US = 40;    // Maxima velocidad permitida (25 kHz)
const uint32_t MAX_STEP_DELAY_US = 4000;  // Minima velocidad permitida (~250 pasos/s)
const uint32_t STEP_DELTA_US     = 25;    // Variacion fina por comando + / -

// Sentido fisico de giro del motor del mecanismo de elevacion:
// LOW = SUBIR (hacia sensor superior Pin 7)
// HIGH = BAJAR (hacia sensor inferior Pin 8)
bool DIR_UP_IS_FORWARD = false; // LOW = SUBIR, HIGH = BAJAR

// ==========================================
// ESTADO INTERNO
// ==========================================
enum MotionMode {
  MODE_IDLE,
  MODE_CONTINUOUS,
  MODE_TARGET_STEPS
};

MotionMode current_mode = MODE_IDLE;
bool current_dir_state = false; // Estado actual del pin DIR

// Velocidad de crucero por defecto: 200 us (~5000 pasos/s = 3.1 rev/s = 187.5 RPM)
uint32_t target_step_delay_us = 200; 
uint32_t continuous_live_delay_us = 1200; // Para rampa suave en modo continuo

int32_t target_total_steps = 0;       // Total de pasos del movimiento actual
int32_t steps_executed = 0;           // Pasos completados
int64_t total_odometry_steps = 0;     // Posicion acumulada absoluta
int64_t max_travel_steps = 0;         // Carrera total medida en calibracion
bool is_calibrated = false;           // Bandera de calibracion completada

unsigned long last_step_time_us = 0;
bool motor_enabled = true;

// Longitud de rampa de aceleracion (configurable)
uint32_t custom_ramp_steps = 250;

// Buffer de comandos seriales
char serial_buf[64];
uint8_t buf_idx = 0;

// ==========================================
// LECTURA DE FINALES DE CARRERA (Bajo consumo)
// ==========================================
// Lectura de finales de carrera con filtro antirruido e integracion (Debounce):
// Si la lectura preliminar es LOW, confirma tras 500us y 1ms para ignorar
// cualquier pico de conmutacion inductivo generado por los cables del motor.
bool isTopLimitHit() {
  if (digitalRead(PIN_LIMIT_TOP) != LOW) return false;
  delayMicroseconds(500);
  if (digitalRead(PIN_LIMIT_TOP) != LOW) return false;
  delay(1);
  return (digitalRead(PIN_LIMIT_TOP) == LOW);
}

bool isBottomLimitHit() {
  if (digitalRead(PIN_LIMIT_BOTTOM) != LOW) return false;
  delayMicroseconds(500);
  if (digitalRead(PIN_LIMIT_BOTTOM) != LOW) return false;
  delay(1);
  return (digitalRead(PIN_LIMIT_BOTTOM) == LOW);
}

bool isMovingUp() {
  return (current_dir_state == DIR_UP_IS_FORWARD);
}

void setDriverEnabled(bool enable) {
  motor_enabled = enable;
  if (enable) {
    digitalWrite(PIN_ENABLE, ENA_ACTIVE_LEVEL);
  } else {
    digitalWrite(PIN_ENABLE, !ENA_ACTIVE_LEVEL);
  }
}

void setDirection(bool dir) {
  current_dir_state = dir;
  digitalWrite(PIN_DIR, dir ? HIGH : LOW);
  delayMicroseconds(5); // Tiempo de establecimiento de direccion
}

void stepOnce() {
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(PIN_STEP, LOW);
  
  if (isMovingUp()) {
    total_odometry_steps++;
  } else {
    total_odometry_steps--;
  }
}

// Rampa de aceleracion/desaceleracion suave para movimientos por pasos/grados
uint32_t calculateDynamicDelay(int32_t current_step, int32_t total_steps, uint32_t target_delay) {
  if (total_steps < 30) {
    return max(target_delay, (uint32_t)600);
  }

  int32_t ramp_len = min((int32_t)custom_ramp_steps, (int32_t)(total_steps / 3));
  if (ramp_len < 10) ramp_len = 10;

  uint32_t start_delay = max(target_delay * 2, (uint32_t)1200);
  if (start_delay < target_delay) start_delay = target_delay;
  
  // Aceleracion
  if (current_step < ramp_len) {
    float factor = (float)current_step / (float)ramp_len;
    return start_delay - (uint32_t)((start_delay - target_delay) * factor);
  }
  
  // Desaceleracion
  int32_t remaining = total_steps - current_step;
  if (remaining < ramp_len) {
    float factor = (float)remaining / (float)ramp_len;
    return start_delay - (uint32_t)((start_delay - target_delay) * factor);
  }

  return target_delay;
}

float getRPM(uint32_t delay_us) {
  if (delay_us == 0) return 0.0;
  float steps_per_rev = (float)STEPS_PER_REV * (float)microstepping;
  float steps_per_sec = 1000000.0 / (float)delay_us;
  return (steps_per_sec / steps_per_rev) * 60.0;
}

void printStatus() {
  Serial.print("[STATUS] Modo: ");
  if (current_mode == MODE_IDLE) Serial.print("IDLE");
  else if (current_mode == MODE_CONTINUOUS) Serial.print("CONTINUOUS");
  else if (current_mode == MODE_TARGET_STEPS) Serial.print("STEPPING");

  Serial.print(" | Accion: ");
  Serial.print(isMovingUp() ? "SUBIENDO (Hacia Pin 7)" : "BAJANDO (Hacia Pin 8)");

  Serial.print(" | Retraso: ");
  Serial.print(target_step_delay_us);
  Serial.print(" us (");
  if (target_step_delay_us > 0) {
    Serial.print(1000000 / target_step_delay_us);
    Serial.print(" pasos/s | ~");
    Serial.print(getRPM(target_step_delay_us), 1);
    Serial.print(" RPM)");
  }
  Serial.print(" | Limites: [TOP(Pin7)=");
  Serial.print(isTopLimitHit() ? "PULSADO" : "LIBRE");
  Serial.print(", BOT(Pin8)=");
  Serial.print(isBottomLimitHit() ? "PULSADO" : "LIBRE");
  Serial.print("] | Restante: ");
  Serial.print(current_mode == MODE_TARGET_STEPS ? (target_total_steps - steps_executed) : 0);
  Serial.print(" | Pos: ");
  Serial.print((long)total_odometry_steps);
  if (is_calibrated) {
    Serial.print(" / ");
    Serial.print((long)max_travel_steps);
    Serial.print(" [CALIBRADO]");
  }
  Serial.println();
}

void setDelayUs(uint32_t val) {
  if (val < MIN_STEP_DELAY_US) val = MIN_STEP_DELAY_US;
  if (val > MAX_STEP_DELAY_US) val = MAX_STEP_DELAY_US;
  target_step_delay_us = val;
  Serial.print("[OK] Velocidad fijada: ");
  Serial.print(target_step_delay_us);
  Serial.print(" us (");
  Serial.print(1000000 / target_step_delay_us);
  Serial.print(" pasos/s | ~");
  Serial.print(getRPM(target_step_delay_us), 1);
  Serial.println(" RPM)");
}

void setRPM(float rpm) {
  if (rpm <= 0.0) return;
  float steps_per_rev = (float)STEPS_PER_REV * (float)microstepping;
  float steps_per_sec = (rpm / 60.0) * steps_per_rev;
  if (steps_per_sec <= 0.0) return;
  uint32_t delay_us = (uint32_t)(1000000.0 / steps_per_sec);
  setDelayUs(delay_us);
}

// Rutina de Calibracion completa (Fondo + Techo + Posicionamiento Superior)
void executeCalibration() {
  Serial.println("\n========================================================");
  Serial.println("  INICIANDO RUTINA COMPLETA DE CALIBRACION (BOT + TOP)   ");
  Serial.println("========================================================");
  setDriverEnabled(true);
  
  const uint32_t turbo_delay_us = 40;  // MAXIMA VELOCIDAD (~25,000 pasos/s | ~900 RPM)
  const uint32_t start_delay_us = 650; // Arranque suave para vencer inercia
  const uint32_t ramp_total_steps = 300;
  uint32_t current_delay_us = start_delay_us;

  // ----------------------------------------------------
  // PASO 1: Bajar hacia el sensor INFERIOR (Pin 8)
  // ----------------------------------------------------
  Serial.println("[*] [1/3] Calibracion: Bajando hacia sensor inferior (Pin 8)...");
  setDirection(!DIR_UP_IS_FORWARD); // BAJAR hacia Pin 8

  // Si ya esta presionado en el fondo al inicio, despegar subiendo un poco
  if (isBottomLimitHit()) {
    setDirection(DIR_UP_IS_FORWARD); // SUBIR
    for (int i = 0; i < 400; i++) {
      digitalWrite(PIN_STEP, HIGH);
      delayMicroseconds(PULSE_WIDTH_US);
      digitalWrite(PIN_STEP, LOW);
      delayMicroseconds(140);
    }
    delay(50);
    setDirection(!DIR_UP_IS_FORWARD); // Volver a BAJAR
  }

  unsigned long t0 = millis();
  int bottom_hit_count = 0;
  int ramp_step = 0;

  while (true) {
    if (millis() - t0 > 60000) {
      Serial.println("[X] Timeout de Calibracion (60s): Sensor inferior (Pin 8) no detectado.");
      return;
    }

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(current_delay_us);

    // Rampa de aceleracion rapida
    if (current_delay_us > turbo_delay_us && ramp_step < ramp_total_steps) {
      ramp_step++;
      current_delay_us = start_delay_us - ((start_delay_us - turbo_delay_us) * ramp_step / ramp_total_steps);
    }

    // Filtro antirruido por integracion y verificacion en silencio
    if (digitalRead(PIN_LIMIT_BOTTOM) == LOW) {
      bottom_hit_count++;
      if (bottom_hit_count >= 40) {
        delay(3);
        if (isBottomLimitHit()) {
          break; // Fondo confirmado
        } else {
          bottom_hit_count = 0;
        }
      }
    } else {
      bottom_hit_count = 0;
    }
  }

  // Toque fino de precision inferior
  delay(50);
  setDirection(DIR_UP_IS_FORWARD); // SUBIR
  for (int i = 0; i < 250; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(140);
  }
  delay(50);

  setDirection(!DIR_UP_IS_FORWARD); // BAJAR
  bottom_hit_count = 0;
  t0 = millis();
  while (true) {
    if (millis() - t0 > 6000) break;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(250);

    if (digitalRead(PIN_LIMIT_BOTTOM) == LOW) {
      bottom_hit_count++;
      if (bottom_hit_count >= 15) {
        delay(2);
        if (isBottomLimitHit()) break;
        else bottom_hit_count = 0;
      }
    } else {
      bottom_hit_count = 0;
    }
  }

  // Fondo fijado como CERO absoluto
  total_odometry_steps = 0;
  Serial.println("[✓] Sensor inferior alcanzado y fijado como ORIGEN (0 pasos).");

  // Despegue inferior seguro (100 pasos)
  delay(50);
  setDirection(DIR_UP_IS_FORWARD); // SUBIR
  for (int i = 0; i < 100; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(180);
  }
  total_odometry_steps = 100;
  delay(100);

  // ----------------------------------------------------
  // PASO 2: Subir hacia el sensor SUPERIOR (Pin 7) y medir carrera
  // ----------------------------------------------------
  Serial.println("[*] [2/3] Calibracion: Subiendo hacia sensor superior (Pin 7) y midiendo carrera...");
  setDirection(DIR_UP_IS_FORWARD); // SUBIR

  current_delay_us = start_delay_us;
  ramp_step = 0;
  t0 = millis();
  int top_hit_count = 0;

  while (true) {
    if (millis() - t0 > 60000) {
      Serial.println("[X] Timeout de Calibracion (60s): Sensor superior (Pin 7) no detectado.");
      return;
    }

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(current_delay_us);
    total_odometry_steps++;

    if (current_delay_us > turbo_delay_us && ramp_step < ramp_total_steps) {
      ramp_step++;
      current_delay_us = start_delay_us - ((start_delay_us - turbo_delay_us) * ramp_step / ramp_total_steps);
    }

    if (digitalRead(PIN_LIMIT_TOP) == LOW) {
      top_hit_count++;
      if (top_hit_count >= 40) {
        delay(3);
        if (isTopLimitHit()) break;
        else top_hit_count = 0;
      }
    } else {
      top_hit_count = 0;
    }
  }

  // Toque fino de precision superior
  delay(50);
  setDirection(!DIR_UP_IS_FORWARD); // BAJAR
  for (int i = 0; i < 250; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(140);
    total_odometry_steps--;
  }
  delay(50);

  setDirection(DIR_UP_IS_FORWARD); // SUBIR
  top_hit_count = 0;
  t0 = millis();
  while (true) {
    if (millis() - t0 > 6000) break;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(250);
    total_odometry_steps++;

    if (digitalRead(PIN_LIMIT_TOP) == LOW) {
      top_hit_count++;
      if (top_hit_count >= 15) {
        delay(2);
        if (isTopLimitHit()) break;
        else top_hit_count = 0;
      }
    } else {
      top_hit_count = 0;
    }
  }

  max_travel_steps = total_odometry_steps;
  float revs = (float)max_travel_steps / (float)(STEPS_PER_REV * microstepping);

  Serial.print("[✓] Sensor superior alcanzado. Carrera total medida: ");
  Serial.print((long)max_travel_steps);
  Serial.print(" pasos (~");
  Serial.print(revs, 2);
  Serial.println(" vueltas de motor).");

  // ----------------------------------------------------
  // PASO 3: Posicionarse bien arriba cerca del sensor superior
  // ----------------------------------------------------
  Serial.println("[*] [3/3] Posicionando elevador cerca del sensor superior (margen de seguridad)...");
  
  // Retroceder bajando 250 pasos (~1 mm) para dejar el microswitch libre y en reposo (0.00 mA)
  const int32_t top_margin_steps = 250;
  setDirection(!DIR_UP_IS_FORWARD); // BAJAR
  for (int i = 0; i < top_margin_steps; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(180);
    total_odometry_steps--;
  }

  is_calibrated = true;
  current_mode = MODE_IDLE;

  Serial.println("========================================================");
  Serial.println("[✓] ¡Calibracion completada con exito!");
  Serial.print("    Carrera util: 0 a ");
  Serial.print((long)max_travel_steps);
  Serial.println(" pasos");
  Serial.print("    Posicion actual: ");
  Serial.print((long)total_odometry_steps);
  Serial.println(" pasos (Tope Superior de trabajo)");
  Serial.println("========================================================");
}

void executeHoming() {
  executeCalibration();
}

void parseCommand(const char* cmd) {
  while (*cmd == ' ') cmd++;
  if (*cmd == '\0') return;

  // 1. Palabras clave de calibracion y homing
  if (strncasecmp(cmd, "CALIBRATE", 9) == 0 || strncasecmp(cmd, "CAL", 3) == 0 ||
      strncasecmp(cmd, "HOME", 4) == 0 || strcasecmp(cmd, "H") == 0) {
    executeCalibration();
    return;
  }
  if (strncasecmp(cmd, "TOP", 3) == 0 || strncasecmp(cmd, "ARRIBA", 6) == 0) {
    if (!is_calibrated) {
      Serial.println("[!] ERROR: Debes calibrar primero ejecutando 'CALIBRATE' o 'HOME'.");
      return;
    }
    int64_t target_pos = max_travel_steps - 250;
    int64_t diff = target_pos - total_odometry_steps;
    if (diff == 0) {
      Serial.println("[OK] Ya se encuentra en la posicion TOP.");
      return;
    }
    char move_cmd[32];
    snprintf(move_cmd, sizeof(move_cmd), "M %ld", (long)diff);
    parseCommand(move_cmd);
    return;
  }
  if (strncasecmp(cmd, "BOT", 3) == 0 || strncasecmp(cmd, "ABAJO", 5) == 0) {
    if (!is_calibrated) {
      Serial.println("[!] ERROR: Debes calibrar primero ejecutando 'CALIBRATE' o 'HOME'.");
      return;
    }
    int64_t target_pos = 100;
    int64_t diff = target_pos - total_odometry_steps;
    if (diff == 0) {
      Serial.println("[OK] Ya se encuentra en la posicion BOTTOM.");
      return;
    }
    char move_cmd[32];
    snprintf(move_cmd, sizeof(move_cmd), "M %ld", (long)diff);
    parseCommand(move_cmd);
    return;
  }
  if (strncasecmp(cmd, "INVERT_DIR", 10) == 0) {
    DIR_UP_IS_FORWARD = !DIR_UP_IS_FORWARD;
    Serial.print("[OK] Sentido invertido. SUBIR es ahora: ");
    Serial.println(DIR_UP_IS_FORWARD ? "HIGH" : "LOW");
    return;
  }
  if (strncasecmp(cmd, "RPM", 3) == 0) {
    float rpm_val = atof(cmd + 3);
    if (rpm_val > 0.0) setRPM(rpm_val);
    else Serial.println("[!] Uso: rpm <valor> (ejemplo: rpm 150)");
    return;
  }
  if (strncasecmp(cmd, "SPEED", 5) == 0) {
    long val = atol(cmd + 5);
    if (val > 0) setDelayUs((uint32_t)val);
    else {
      Serial.print("[!] Rango: "); Serial.print(MIN_STEP_DELAY_US);
      Serial.print(" a "); Serial.print(MAX_STEP_DELAY_US); Serial.println(" us");
    }
    return;
  }
  if (strncasecmp(cmd, "FAST", 4) == 0) { setDelayUs(100); return; }
  if (strncasecmp(cmd, "TURBO", 5) == 0) { setDelayUs(50); return; }
  if (strncasecmp(cmd, "NORMAL", 6) == 0) { setDelayUs(250); return; }
  if (strncasecmp(cmd, "SLOW", 4) == 0) { setDelayUs(600); return; }

  if (strncasecmp(cmd, "RAMP", 4) == 0) {
    long r = atol(cmd + 4);
    if (r >= 10 && r <= 2000) {
      custom_ramp_steps = (uint32_t)r;
      Serial.print("[OK] Rampa: "); Serial.print(custom_ramp_steps); Serial.println(" pasos.");
    }
    return;
  }
  if (strncasecmp(cmd, "STATUS", 6) == 0 || strcmp(cmd, "?") == 0) {
    printStatus();
    return;
  }
  if (strncasecmp(cmd, "MICRO", 5) == 0) {
    long m = atol(cmd + 5);
    if (m > 0 && m <= 64) {
      microstepping = (uint32_t)m;
      Serial.print("[OK] Microstepping: 1/"); Serial.println(microstepping);
    }
    return;
  }
  if (strncasecmp(cmd, "ENABLE", 6) == 0) {
    setDriverEnabled(true);
    Serial.println("[OK] Driver HABILITADO.");
    return;
  }
  if (strncasecmp(cmd, "DISABLE", 7) == 0) {
    setDriverEnabled(false);
    current_mode = MODE_IDLE;
    Serial.println("[OK] Driver DESHABILITADO (Eje libre).");
    return;
  }

  // Comandos de SUBIR explicito
  if (strncasecmp(cmd, "SUBIR", 5) == 0 || strncasecmp(cmd, "UP", 2) == 0) {
    if (isTopLimitHit()) {
      Serial.println("[!] ERROR: Sensor superior (Pin 7) activado. No se puede subir mas.");
      return;
    }
    setDirection(DIR_UP_IS_FORWARD);
    setDriverEnabled(true);
    continuous_live_delay_us = max(target_step_delay_us * 2, (uint32_t)1200);
    current_mode = MODE_CONTINUOUS;
    Serial.print("[OK] SUBIENDO continuo con rampa hacia ");
    Serial.print(target_step_delay_us);
    Serial.println(" us.");
    return;
  }

  // Comandos de BAJAR explicito
  if (strncasecmp(cmd, "BAJAR", 5) == 0 || strncasecmp(cmd, "DOWN", 4) == 0) {
    if (isBottomLimitHit()) {
      Serial.println("[!] ERROR: Sensor inferior (Pin 8) activado. No se puede bajar mas.");
      return;
    }
    setDirection(!DIR_UP_IS_FORWARD);
    setDriverEnabled(true);
    continuous_live_delay_us = max(target_step_delay_us * 2, (uint32_t)1200);
    current_mode = MODE_CONTINUOUS;
    Serial.print("[OK] BAJANDO continuo con rampa hacia ");
    Serial.print(target_step_delay_us);
    Serial.println(" us.");
    return;
  }

  // 2. Comandos de un solo caracter
  char c = cmd[0];

  // F / f = Subir (adelante)
  if (c == 'F' || c == 'f') {
    if (isTopLimitHit()) {
      Serial.println("[!] ERROR: Sensor superior (Pin 7) activado. No se puede subir mas.");
      return;
    }
    setDirection(DIR_UP_IS_FORWARD); // Subir
    setDriverEnabled(true);
    continuous_live_delay_us = max(target_step_delay_us * 2, (uint32_t)1200);
    current_mode = MODE_CONTINUOUS;
    Serial.print("[OK] SUBIENDO continuo hacia sensor superior (Pin 7) a ");
    Serial.print(target_step_delay_us);
    Serial.println(" us.");
  }
  // R / r = Bajar (atras)
  else if (c == 'R' || c == 'r') {
    if (isBottomLimitHit()) {
      Serial.println("[!] ERROR: Sensor inferior (Pin 8) activado. No se puede bajar mas.");
      return;
    }
    setDirection(!DIR_UP_IS_FORWARD); // Bajar
    setDriverEnabled(true);
    continuous_live_delay_us = max(target_step_delay_us * 2, (uint32_t)1200);
    current_mode = MODE_CONTINUOUS;
    Serial.print("[OK] BAJANDO continuo hacia sensor inferior (Pin 8) a ");
    Serial.print(target_step_delay_us);
    Serial.println(" us.");
  }
  // Parada de emergencia
  else if (c == 'S' || c == 's') {
    current_mode = MODE_IDLE;
    target_total_steps = 0;
    steps_executed = 0;
    Serial.println("[OK] Parada de emergencia ejecutada.");
  }
  // Aumentar velocidad (+)
  else if (c == '+') {
    if (target_step_delay_us > MIN_STEP_DELAY_US + STEP_DELTA_US) {
      setDelayUs(target_step_delay_us - STEP_DELTA_US);
    } else {
      setDelayUs(MIN_STEP_DELAY_US);
    }
  }
  // Reducir velocidad (-)
  else if (c == '-') {
    if (target_step_delay_us + STEP_DELTA_US <= MAX_STEP_DELAY_US) {
      setDelayUs(target_step_delay_us + STEP_DELTA_US);
    } else {
      setDelayUs(MAX_STEP_DELAY_US);
    }
  }
  // Comando V <us>
  else if (c == 'V' || c == 'v') {
    long val = atol(cmd + 1);
    if (val > 0) setDelayUs((uint32_t)val);
  }
  // Movimiento relativo por pasos: M <steps>
  else if (c == 'M' || c == 'm') {
    long steps = atol(cmd + 1);
    if (steps == 0) {
      Serial.println("[!] Pasos = 0. Nada que mover.");
      return;
    }
    bool wants_up = (steps > 0);

    if (wants_up && isTopLimitHit()) {
      Serial.println("[!] ERROR: Sensor superior (Pin 7) activado. No se puede subir.");
      return;
    }
    if (!wants_up && isBottomLimitHit()) {
      Serial.println("[!] ERROR: Sensor inferior (Pin 8) activado. No se puede bajar.");
      return;
    }

    setDirection(wants_up ? DIR_UP_IS_FORWARD : !DIR_UP_IS_FORWARD);
    target_total_steps = (steps > 0) ? steps : -steps;
    steps_executed = 0;
    setDriverEnabled(true);
    current_mode = MODE_TARGET_STEPS;
    Serial.print("[OK] ");
    Serial.print(wants_up ? "SUBIENDO " : "BAJANDO ");
    Serial.print(target_total_steps);
    Serial.print(" pasos a ");
    Serial.print(target_step_delay_us);
    Serial.println(" us/paso con rampa.");
  }
  // Movimiento relativo por grados: A <deg>
  else if (c == 'A' || c == 'a') {
    float deg = atof(cmd + 1);
    if (deg == 0.0) {
      Serial.println("[!] Grados = 0.0. Nada que mover.");
      return;
    }
    float total_steps_per_rev = (float)STEPS_PER_REV * (float)microstepping;
    long steps = (long)round((deg / 360.0) * total_steps_per_rev);
    
    bool wants_up = (deg > 0.0);

    if (wants_up && isTopLimitHit()) {
      Serial.println("[!] ERROR: Sensor superior (Pin 7) activado. No se puede subir.");
      return;
    }
    if (!wants_up && isBottomLimitHit()) {
      Serial.println("[!] ERROR: Sensor inferior (Pin 8) activado. No se puede bajar.");
      return;
    }

    setDirection(wants_up ? DIR_UP_IS_FORWARD : !DIR_UP_IS_FORWARD);
    target_total_steps = (steps > 0) ? steps : -steps;
    steps_executed = 0;
    setDriverEnabled(true);
    current_mode = MODE_TARGET_STEPS;
    Serial.print("[OK] ");
    Serial.print(wants_up ? "SUBIENDO " : "BAJANDO ");
    Serial.print(abs(deg), 2);
    Serial.print(" deg (");
    Serial.print(target_total_steps);
    Serial.print(" pasos) a ");
    Serial.print(target_step_delay_us);
    Serial.println(" us.");
  }
  else {
    Serial.print("[!] Comando desconocido: '");
    Serial.print(cmd);
    Serial.println("'. Usa ?, F, R, SUBIR, BAJAR, S, +, -, SPEED <us>, RPM <rpm>, FAST, NORMAL, SLOW, M, A, HOME.");
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  // Configurar finales de carrera con PULLUP interno
  pinMode(PIN_LIMIT_TOP, INPUT_PULLUP);    // Pin 7
  pinMode(PIN_LIMIT_BOTTOM, INPUT_PULLUP); // Pin 8

  digitalWrite(PIN_STEP, LOW);
  setDirection(DIR_UP_IS_FORWARD);
  setDriverEnabled(true);

  delay(500);
  Serial.println("\n========================================================");
  Serial.println(" ESP32-S3 NEMA23 ELEVATOR CONTROLLER WITH ENDSTOPS      ");
  Serial.println(" Pin 7 = TOP (Superior) | Pin 8 = BOTTOM (Inferior)     ");
  Serial.println("========================================================");
  printStatus();
}

void loop() {
  // 1. Recepcion no bloqueante de comandos USB Serial
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf_idx > 0) {
        serial_buf[buf_idx] = '\0';
        parseCommand(serial_buf);
        buf_idx = 0;
      }
    } else {
      if (buf_idx < sizeof(serial_buf) - 1) {
        serial_buf[buf_idx++] = c;
      }
    }
  }

  // 2. Comprobacion de seguridad de los Finales de Carrera
  if (current_mode != MODE_IDLE) {
    if (isMovingUp() && isTopLimitHit()) {
      current_mode = MODE_IDLE;
      target_total_steps = 0;
      steps_executed = 0;
      Serial.println("\n[!] DETENCION DE SEGURIDAD: Sensor SUPERIOR (Pin 7) activado!");
    } else if (!isMovingUp() && isBottomLimitHit()) {
      current_mode = MODE_IDLE;
      target_total_steps = 0;
      steps_executed = 0;
      Serial.println("\n[!] DETENCION DE SEGURIDAD: Sensor INFERIOR (Pin 8) activado!");
    }
  }

  // 3. Generador de pasos con aceleracion
  unsigned long now_us = micros();

  // MODO 1: GIRO CONTINUO
  if (current_mode == MODE_CONTINUOUS) {
    if (now_us - last_step_time_us >= continuous_live_delay_us) {
      last_step_time_us = now_us;
      stepOnce();

      if (continuous_live_delay_us > target_step_delay_us) {
        uint32_t step_down = (continuous_live_delay_us - target_step_delay_us > 200) ? 4 : 1;
        continuous_live_delay_us -= step_down;
      } else if (continuous_live_delay_us < target_step_delay_us) {
        continuous_live_delay_us += 2;
      }
    }
  }
  // MODO 2: MOVIMIENTO POR PASOS O GRADOS
  else if (current_mode == MODE_TARGET_STEPS) {
    uint32_t active_delay = calculateDynamicDelay(steps_executed, target_total_steps, target_step_delay_us);
    
    if (now_us - last_step_time_us >= active_delay) {
      last_step_time_us = now_us;
      stepOnce();
      steps_executed++;
      
      if (steps_executed >= target_total_steps) {
        current_mode = MODE_IDLE;
        Serial.print("[✓] Movimiento finalizado. Posicion total: ");
        Serial.println((long)total_odometry_steps);
      }
    }
  }
}
