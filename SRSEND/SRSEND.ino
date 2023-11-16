#define TRIGGER_PIN 9  // Change to your connected pin

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
}

void loop() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  delay(60);  // Pause between pulses (60ms recommended for HC-SR04)
}