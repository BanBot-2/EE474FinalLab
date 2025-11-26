#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LCD_SCL 13
#define LCD_SDA 14

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Wire.begin(LCD_SDA, LCD_SCL);
  
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("LCD Test OK");
}

void loop() {
  lcd.setCursor(0,1);
  lcd.print(millis()/1000);
  delay(500);
}