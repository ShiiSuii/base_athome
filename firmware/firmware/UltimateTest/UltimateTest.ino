// UltimateTest.ino
// Este código no tiene serial, no tiene rampas, no usa el pin EL, nada.
// Simplemente empuja un voltaje de velocidad fijo apenas enciendes el Arduino.

#define ZF_PIN 8
#define VR_PIN 9

void setup() {
  pinMode(ZF_PIN, OUTPUT);
  pinMode(VR_PIN, OUTPUT);
  
  // Sube la frecuencia de señal para sea un voltaje compatible para la placa
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  
  digitalWrite(ZF_PIN, HIGH);
  analogWrite(VR_PIN, 127); // 50% fijo constante para siempre.
}

void loop() {
  // Vacío. El motor debería estar girando al 50%.
}
