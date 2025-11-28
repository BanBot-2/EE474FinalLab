#include <Arduino.h>

#define PHOTO_PIN 4

void setup() {
  Serial.begin(115200);
}

void loop() {
  int lightLevel = analogRead(PHOTO_PIN);
  Serial.printf("Light Level: %d\n", lightLevel);
  delay(500);
}