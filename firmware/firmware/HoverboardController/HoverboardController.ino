/**
 * HoverboardController.ino
 * Low-level controller for Custom Hoverboard Platform using JYQD/ZS-X11H dual drivers.
 */

#define L_M_PIN  2
#define L_EL_PIN 4
#define L_ZF_PIN 8
#define L_VR_PIN 9

#define R_M_PIN  3
#define R_EL_PIN 7
#define R_ZF_PIN 12
#define R_VR_PIN 10

#define TIMEOUT_MS 500
#define RAMP_INTERVAL_MS 5
#define VELOCITY_INTERVAL_MS 100
#define TELEMETRY_INTERVAL_MS 100
#define PWM_STEP 2 // Accelerates 2 per 5ms (~600ms full ramp)

struct MotorState {
  uint8_t m_pin;
  uint8_t el_pin;
  uint8_t zf_pin;
  uint8_t vr_pin;

  float target_speed; // -1.0 to +1.0
  int current_pwm;
  int target_pwm_internal;
  
  bool current_dir; // true = forward, false = backward
  bool target_dir;
  
  float pps; // pulses per second
};

MotorState left_motor = {L_M_PIN, L_EL_PIN, L_ZF_PIN, L_VR_PIN, 0.0, 0, 0, true, true, 0.0};
MotorState right_motor = {R_M_PIN, R_EL_PIN, R_ZF_PIN, R_VR_PIN, 0.0, 0, 0, true, true, 0.0};

volatile unsigned long l_pulses = 0;
volatile unsigned long r_pulses = 0;
unsigned long last_l_pulses = 0;
unsigned long last_r_pulses = 0;

unsigned long last_cmd_time = 0;
unsigned long last_ramp_time = 0;
unsigned long last_vel_time = 0;
unsigned long last_telemetry_time = 0;

char cmd_buffer[64];
uint8_t cmd_idx = 0;

void l_pulse_isr() { l_pulses++; }
void r_pulse_isr() { r_pulses++; }

void setup() {
  Serial.begin(115200);
  
  // Set pins
  pinMode(L_M_PIN, INPUT_PULLUP);
  pinMode(L_EL_PIN, OUTPUT);
  pinMode(L_ZF_PIN, OUTPUT);
  pinMode(L_VR_PIN, OUTPUT);
  
  pinMode(R_M_PIN, INPUT_PULLUP);
  pinMode(R_EL_PIN, OUTPUT);
  pinMode(R_ZF_PIN, OUTPUT);
  pinMode(R_VR_PIN, OUTPUT);
  
  // Safe defaults immediately
  digitalWrite(L_EL_PIN, LOW);
  digitalWrite(R_EL_PIN, LOW);
  analogWrite(L_VR_PIN, 0);
  analogWrite(R_VR_PIN, 0);
  
  // Set initial direction (assume HIGH is forward, subject to physical test)
  digitalWrite(L_ZF_PIN, HIGH);
  digitalWrite(R_ZF_PIN, HIGH); 

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(L_M_PIN), l_pulse_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(R_M_PIN), r_pulse_isr, RISING);
  
  last_cmd_time = millis(); // Give 500ms grace on boot
}

void parseSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_idx > 0) {
        cmd_buffer[cmd_idx] = '\0';
        
        if (strncmp(cmd_buffer, "CMD", 3) == 0) {
          char* token = strtok(cmd_buffer + 4, " ");
          if (token != NULL) {
            float left_req = atof(token);
            token = strtok(NULL, " ");
            if (token != NULL) {
              float right_req = atof(token);
              
              left_motor.target_speed = constrain(left_req, -1.0, 1.0);
              right_motor.target_speed = constrain(right_req, -1.0, 1.0);
              
              if (left_motor.target_speed != 0.0) {
                left_motor.target_dir = (left_motor.target_speed > 0);
              }
              if (right_motor.target_speed != 0.0) {
                right_motor.target_dir = (right_motor.target_speed > 0);
              }
              
              last_cmd_time = millis();
            }
          }
        }
        cmd_idx = 0;
      }
    } else {
      if (cmd_idx < 63) {
        cmd_buffer[cmd_idx++] = c;
      }
    }
  }
}

void updateRamp(MotorState &m) {
  if (m.target_dir != m.current_dir) {
    // Must stop before reversing direction securely
    m.target_pwm_internal = 0;
    
    if (m.current_pwm > 0) {
      m.current_pwm -= PWM_STEP;
      if (m.current_pwm < 0) m.current_pwm = 0;
    } else {
      // Reached 0 PWM. Wait for pulses to settle (pps near 0) before toggling ZF.
      if (abs(m.pps) < 1.0) { 
        m.current_dir = m.target_dir;
        digitalWrite(m.zf_pin, m.current_dir ? HIGH : LOW);
      }
    }
  } else {
    m.target_pwm_internal = (int)(abs(m.target_speed) * 255.0);
    
    if (m.current_pwm < m.target_pwm_internal) {
      m.current_pwm += PWM_STEP;
      if (m.current_pwm > m.target_pwm_internal) m.current_pwm = m.target_pwm_internal;
    } else if (m.current_pwm > m.target_pwm_internal) {
      m.current_pwm -= PWM_STEP;
      if (m.current_pwm < 0) m.current_pwm = 0;
    }
  }
  
  analogWrite(m.vr_pin, m.current_pwm);
  
  // Enable or Disable EL Driver Safely
  if (m.target_speed == 0.0 && m.current_pwm == 0) {
    digitalWrite(m.el_pin, LOW); // Stop motor entirely
  } else {
    digitalWrite(m.el_pin, HIGH); // Enable motor
  }
}

void loop() {
  unsigned long now = millis();
  
  parseSerial();
  
  // Watchdog
  if (now - last_cmd_time > TIMEOUT_MS) {
    left_motor.target_speed = 0.0;
    right_motor.target_speed = 0.0;
    
    // Emergency stop (Hard cutoff instead of ramp to strictly enforce safety on disconnected serial)
    left_motor.current_pwm = 0;
    right_motor.current_pwm = 0;
    digitalWrite(L_EL_PIN, LOW);
    digitalWrite(R_EL_PIN, LOW);
    analogWrite(L_VR_PIN, 0);
    analogWrite(R_VR_PIN, 0);
  }
  
  // Velocity calculation (100ms)
  if (now - last_vel_time >= VELOCITY_INTERVAL_MS) {
    noInterrupts();
    unsigned long curr_l = l_pulses;
    unsigned long curr_r = r_pulses;
    interrupts();
    
    unsigned long dl = curr_l - last_l_pulses;
    unsigned long dr = curr_r - last_r_pulses;
    
    last_l_pulses = curr_l;
    last_r_pulses = curr_r;
    
    float dt = (now - last_vel_time) / 1000.0; // Seconds elapsed
    last_vel_time = now;
    
    left_motor.pps = (dl / dt) * (left_motor.current_dir ? 1.0 : -1.0);
    right_motor.pps = (dr / dt) * (right_motor.current_dir ? 1.0 : -1.0);
  }
  
  // Ramping calculation (5ms)
  if (now - last_ramp_time >= RAMP_INTERVAL_MS) {
    last_ramp_time = now;
    updateRamp(left_motor);
    updateRamp(right_motor);
  }
  
  // Telemetry loop (100ms)
  if (now - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
    last_telemetry_time = now;
    
    noInterrupts();
    unsigned long curr_l_print = l_pulses;
    unsigned long curr_r_print = r_pulses;
    interrupts();
    
    // Format: TEL L_PULSES R_PULSES L_PPS R_PPS L_PWM R_PWM L_DIR R_DIR
    Serial.print("TEL ");
    Serial.print(curr_l_print); Serial.print(" ");
    Serial.print(curr_r_print); Serial.print(" ");
    Serial.print(left_motor.pps, 2); Serial.print(" ");
    Serial.print(right_motor.pps, 2); Serial.print(" ");
    Serial.print(left_motor.current_pwm); Serial.print(" ");
    Serial.print(right_motor.current_pwm); Serial.print(" ");
    Serial.print(left_motor.current_dir ? 1 : -1); Serial.print(" ");
    Serial.println(right_motor.current_dir ? 1 : -1);
  }
}
