// SimplifiedRamp.ino
// Código simplificado agregando la interrupción de lectura de Pulsos M en D2

#define M_PIN 2   // Lectura de señal de velocidad (Hall/Pulsos)
#define EL_PIN 4  // Activación
#define ZF_PIN 8  // Dirección
#define VR_PIN 9  // Velocidad (PWM)

float target_speed = 0.0;
int current_pwm = 0;
bool current_dir = true;

// Contadores de la señal M
volatile unsigned long pulse_count = 0;
unsigned long last_pulse_count = 0;

unsigned long last_ramp_time = 0;
unsigned long last_telemetry_time = 0;

char cmd_buffer[32];
int cmd_idx = 0;

// Rutina de interrupción que cuenta cada pulso del driver
void pulse_isr() {
  pulse_count++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(EL_PIN, INPUT); // FLOTANTE - Evitamos que mande GND o 5V
  pinMode(ZF_PIN, OUTPUT);
  pinMode(VR_PIN, OUTPUT);
  pinMode(M_PIN, INPUT_PULLUP);
  
  // Enganchar la interrupción al Pin 2 (D2)
  attachInterrupt(digitalPinToInterrupt(M_PIN), pulse_isr, RISING);
  
  // Subir la frecuencia del timer 1 (pin 9 y 10) a 31 kHz para el JYQD
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  
  // No tocamos EL_PIN aquí
  digitalWrite(ZF_PIN, HIGH);
  analogWrite(VR_PIN, 0);
  
  Serial.println("Listo. Lectura M (D2) activada. Escribe 'CMD 0.5' para acelerar 50%.");
}

void loop() {
  unsigned long now = millis();
  
  // 1. Leer Serial sin bloquear
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buffer[cmd_idx] = '\0';
        if (strncmp(cmd_buffer, "CMD ", 4) == 0) {
          target_speed = atof(cmd_buffer + 4); 
          if (target_speed > 1.0) target_speed = 1.0;
          if (target_speed < -1.0) target_speed = -1.0;
        }
        cmd_idx = 0;
      }
    } else {
      if (cmd_idx < 30) cmd_buffer[cmd_idx++] = c;
    }
  }
  
  // 2. Rampa ULTRA suave cada 20ms (antes 5ms). 
  // Combinado con un paso de 1, toma 5.1 segundos llegar a máxima velocidad.
  if (now - last_ramp_time >= 20) {
    last_ramp_time = now;
    
    bool target_dir = (target_speed >= 0.0);
    int target_pwm = abs(target_speed) * 255;
    
    if (target_dir != current_dir && current_pwm > 0) {
       // Si cambiamos de dirección, la frenada también es progresiva
       current_pwm -= 1;
       if (current_pwm < 0) current_pwm = 0;
    } 
    else if (target_dir != current_dir && current_pwm == 0) {
       current_dir = target_dir;
       digitalWrite(ZF_PIN, current_dir ? HIGH : LOW);
    } 
    else {
       // Aceleración lentísima, sube de a 1 punto
       if (current_pwm < target_pwm) current_pwm += 1;
       if (current_pwm > target_pwm) current_pwm -= 1;
       if (abs(current_pwm - target_pwm) <= 1) current_pwm = target_pwm; 
    }
    
    // EL lo dejamos tranquilo. El driver se controla solo con VR.
    
    analogWrite(VR_PIN, current_pwm);
  }
  
  // 3. Imprimir telemetría con lectura "M" calculada cada 500ms
  if (now - last_telemetry_time >= 500) {
    float dt = (now - last_telemetry_time) / 1000.0;
    last_telemetry_time = now;
    
    // Leer los pulsos con los interrupts pausados momentáneamente
    noInterrupts();
    unsigned long current_pulses = pulse_count;
    interrupts();
    
    // Calcular PPS (Pulsos por segundo)
    unsigned long dp = current_pulses - last_pulse_count;
    last_pulse_count = current_pulses;
    
    float pps = dp / dt;
    if (!current_dir) pps = -pps; // Le damos signo según la dirección solicitada
    
    Serial.print("Vel. Objetivo: "); Serial.print(target_speed);
    Serial.print(" | PWM Cur: "); Serial.print(current_pwm);
    Serial.print(" | PPS (M): "); Serial.print(pps);
    Serial.print(" | Pulsos: "); Serial.println(current_pulses);
  }
}
