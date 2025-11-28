/**
 * @file Final_Lab.ino
 * @author Yehoshua Luna & Carter Lee
 * @date 2025-11-25
 * @brief Digital alarm clock project using RTC module, LCD module, IR remote, LEDs, button, and photoresistor.
 *
 * @version 2.0
 * @details
 * ### Update History
 * - **2025-11-25** (Update 2): Yehoshua wrote task function drafts
 * - **2025-11-20** (Update 1): Skeleton code by Carter
 */


#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <IRRemoteESP32.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Pin definitions
#define POT_PIN 1
#define BUTTON_PIN 5
#define PHOTO_PIN 4
#define BUZZER_PIN 6
#define RED_LED_PIN 7
#define BLUE_LED_PIN 17
#define LCD_SCL 13
#define LCD_SDA 14
#define IR_PIN 48
#define RTC_SCL 35
#define RTC_SDA 36

// I2C bus setup
TwoWire RTC_I2C_BUS = TwoWire(1);

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RTC Setup
RTC_DS3231 rtc();

// Shared state
volatile bool alarmTriggered = false;
int alarmHour = 7;     // default alarm time
int alarmMinute = 0;   

// Queues for inter-task communication
QueueHandle_t lightQueue;
QueueHandle_t tempQueue;

// Function declarations
void TaskUpdateTime(void *parameter);
void TaskAlarmControl(void *parameter);
void TaskSensorRead(void *parameter);

void setup() {
  Serial.begin(115200);

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize RTC
  if (!rtc.begin(&RTC_I2C_BUS)) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // Initialize Pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  // Create queues
  lightQueue = xQueueCreate(10, sizeof(int));
  tempQueue  = xQueueCreate(10, sizeof(float));

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(TaskUpdateTime, "UpdateTime", 4096, NULL, 1, NULL, 0);   // Core 0
  xTaskCreatePinnedToCore(TaskAlarmControl, "AlarmCtrl", 2048, NULL, 1, NULL, 1);  // Core 1
  xTaskCreatePinnedToCore(TaskSensorRead, "SensorRead", 2048, NULL, 1, NULL, 0);   // Core 0
}

void loop() {
  // Nothing here, FreeRTOS scheduler runs tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------- TASKS ----------------

// Task: Update and display time
void TaskUpdateTime(void *parameter) {
  while (true) {
    DateTime now = rtc.now();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Time: ");
    lcd.print(now.hour());
    lcd.print(":");
    lcd.print(now.minute());

    lcd.setCursor(0, 1);
    lcd.print("Alarm: ");
    lcd.print(alarmHour);
    lcd.print(":");
    lcd.print(alarmMinute);

    // Debug
    Serial.printf("Current Time: %02d:%02d\n", now.hour(), now.minute());

    vTaskDelay(pdMS_TO_TICKS(1000)); // update every second
  }
}

// Task: Alarm control logic
void TaskAlarmControl(void *parameter) {
  while (true) {
    DateTime now = rtc.now();

    // Check if alarm time matches
    if (now.hour() == alarmHour && now.minute() == alarmMinute && !alarmTriggered) {
      alarmTriggered = true;
      Serial.println("Alarm Triggered!");

      // Activate buzzer and LEDs
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(BLUE_LED_PIN, HIGH);
    }

    // Check button to stop alarm
    if (digitalRead(BUTTON_PIN) == LOW && alarmTriggered) {
      alarmTriggered = false;
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(BLUE_LED_PIN, LOW);
      Serial.println("Alarm Stopped.");
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // check 5 times per second
  }
}

// Task: Read sensors (photoresistor, potentiometer, RTC temp)
void TaskSensorRead(void *parameter) {
  while (true) {
    int lightLevel = analogRead(PHOTO_PIN);
    int potValue   = analogRead(POT_PIN);

    // Map potentiometer to alarm minutes/hours
    alarmHour   = map(potValue, 0, 4095, 0, 23);
    alarmMinute = (map(potValue, 0, 4095, 0, 59));

    // Send light level to queue
    xQueueSend(lightQueue, &lightLevel, portMAX_DELAY);

    // Read RTC temperature
    float tempC = rtc.getTemperature();
    xQueueSend(tempQueue, &tempC, portMAX_DELAY);

    Serial.printf("Light: %d, Temp: %.2f C\n", lightLevel, tempC);

    vTaskDelay(pdMS_TO_TICKS(500)); // sample twice per second
  }
}