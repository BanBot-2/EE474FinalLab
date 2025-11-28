#include <Arduino.h>

#define BUZZER_PIN 6

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {
  tone(BUZZER_PIN, 1000); // 1kHz tone
  delay(1000);
  noTone(BUZZER_PIN);
  delay(1000);
}