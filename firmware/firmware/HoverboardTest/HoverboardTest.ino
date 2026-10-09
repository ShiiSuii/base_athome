void setup() {
  Serial.begin(115200);
  
  pinMode(4, OUTPUT); // L_EL
  pinMode(8, OUTPUT); // L_ZF
  pinMode(9, OUTPUT); // L_VR
  
  // ALTA FRECUENCIA PARA EL JYQD: 
  // Pone el Timer1 del Arduino Nano (Pines 9 y 10) a 31250 Hz (31 kHz).
  // Muchos drivers JYQD ignoran el PWM si la frecuencia es estándar (490Hz) 
  // debido a que los capacitores internos no lo estabilizan bien.
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  
  Serial.println("==============================================");
  Serial.println(" INICIANDO TEST AUTOMATICO - MOTOR IZQUIERDO");
  Serial.println("==============================================");
  delay(3000);
}

void loop() {
  // CONFIGURAR DIRECCION FIJA
  digitalWrite(8, HIGH); // Hacia un lado

  // --- PRUEBA 1: EL a 5V (HIGH) y Velocidad 30% ---
  Serial.println("---> Probando: Pin EL = 5V (HIGH) y Velocidad = 30%");
  digitalWrite(4, HIGH);
  analogWrite(9, 76);
  delay(4000);
  analogWrite(9, 0); // Para poder notar si frena
  delay(1000);
  
  // --- PRUEBA 2: EL a GND (LOW) y Velocidad 30% ---
  Serial.println("---> Probando: Pin EL = GND (LOW) y Velocidad = 30%");
  digitalWrite(4, LOW);
  analogWrite(9, 76);
  delay(4000);
  analogWrite(9, 0);
  delay(1000);

  // --- PRUEBA 3: EL a 5V (HIGH) y Velocidad 80% ---
  Serial.println("---> Probando: Pin EL = 5V (HIGH) y Velocidad = 80%");
  digitalWrite(4, HIGH);
  analogWrite(9, 204);
  delay(4000);
  analogWrite(9, 0);
  delay(1000);
  
  // --- PRUEBA 4: EL pin FLOTANTE (Alta impedancia) y Velocidad 80% ---
  Serial.println("---> Probando: Pin EL = FLOTANTE (desconectado virtualmente) y Vel = 80%");
  pinMode(4, INPUT); // Pasa a estado flotante (Hi-Z)
  analogWrite(9, 204);
  delay(4000);
  analogWrite(9, 0);
  pinMode(4, OUTPUT); // Restaura
  delay(1000);

  // --- PRUEBA 5: EL a GND (LOW) y Velocidad 80% ---
  Serial.println("---> Probando: Pin EL = GND (LOW) y Velocidad = 80%");
  digitalWrite(4, LOW);
  analogWrite(9, 204);
  delay(4000);
  analogWrite(9, 0);

  // Parada de seguridad antes de repetir
  digitalWrite(4, LOW); 
  Serial.println("==============================================");
  Serial.println(" FIN DEL CICLO. PAUSANDO 5 SEGUNDOS... ");
  Serial.println("==============================================");
  delay(5000);
}
