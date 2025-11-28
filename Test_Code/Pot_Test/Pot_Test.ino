#include <Arduino.h>

#define POT_PIN 1

void setup() {
  Serial.begin(115200);
}

void loop() {
  int potValue = analogRead(POT_PIN);
  int mappedHour = map(potValue, 0, 4095, 0, 23);
  int mappedMinute = map(potValue, 0, 4095, 0, 59);
  Serial.printf("Raw: %d  Hour: %d  Minute: %d\n", potValue, mappedHour, mappedMinute);
  delay(500);
}