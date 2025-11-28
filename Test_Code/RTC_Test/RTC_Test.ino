#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

#define RTC_SCL 35
#define RTC_SDA 36

RTC_DS3231 rtc;

TwoWire RTC_I2C_BUS = TwoWire(1);

void setup() {
  Serial.begin(115200);
  RTC_I2C_BUS.begin(RTC_SDA, RTC_SCL);
  
  if (!rtc.begin(&RTC_I2C_BUS)) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  // Uncomment if you need to set time once
  //rtc.adjust(DateTime(2025, 11, 25, 17, 18, 0));
}

void loop() {
  DateTime now = rtc.now();
  Serial.printf("Time: %02d:%02d:%02d  Temp: %.2f C\n",
                now.hour(), now.minute(), now.second(),
                rtc.getTemperature());
  delay(1000);
}