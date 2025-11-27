/**
 * @file Final_Project.ino
 * @author Yehoshua Luna & Carter Lee
 * @date 2025-11-25
 * @brief Digital alarm clock project using RTC module, LCD module, IR remote, LEDs, button, and photoresistor.
 *
 * @version 2.0
 * @details
 * ### Update History
 * - **2025-11-26** (Update 3): Yehoshua finished base functions
 * - **2025-11-25** (Update 2): Yehoshua wrote task function drafts
 * - **2025-11-20** (Update 1): Skeleton code by Carter
 */


// =========================================== Includes ===========================================
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <driver/timer.h>


// ============================================ Macros ============================================

// Pin definitions
#define POT_PIN 1
#define PHOTO_PIN 4
#define BUTTON_PIN 5
#define BUZZER_PIN 6
#define RED_LED_PIN 7
#define BLUE_LED_PIN 17

#define LCD_SCL 13
#define LCD_SDA 14
#define RTC_SCL 35
#define RTC_SDA 36

#define IR_PIN 37

// Configuration
#define POT_THRESH 200
#define DEBOUNCE_MS 200
#define BACKLIGHT_THRESH 1000

#define IR_INTERVAL_MS 32         // 32Hz
#define RTC_INTERVAL_MS 100       // 10Hz
#define LCD_INTERVAL_MS 20        // 50Hz
#define ALARM_INTERVAL_MS 1000    // 1Hz
#define PHOTO_POT_INTERVAL_MS 50  // 20Hz


// ========================================= Object Setup =========================================
TwoWire RTC_I2C_BUS = TwoWire(1);    // Separate I2C bus for RTC
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD setup on global Wire I2C bus
RTC_DS3231 rtc;                      // Initializes RTC library object


// =========================================== Handles ============================================
QueueHandle_t xAlarmQueue;
QueueHandle_t xRemoteQueue;

TaskHandle_t xTaskRTC;
TaskHandle_t xTaskLCD;

SemaphoreHandle_t xEnabledMutex;
SemaphoreHandle_t xTimeTempMutex;
SemaphoreHandle_t xPhotoPotMutex;

hw_timer_t *xDebounceTimer = NULL;


// ======================================= Global Variables =======================================
DateTime currentTime = DateTime((uint32_t)0);
float currentTemp = 0.0;

uint16_t potReading = 0;
uint16_t photoReading = 0;

bool alarmEnabled = false;
bool alarmTriggered = false;

volatile uint64_t lastDebounceTime = 0;

uint8_t bell[8] = {
  0x04,  //   *
  0x0E,  //  ***
  0x0E,  //  ***
  0x0E,  //  ***
  0x1F,  // *****
  0x04,  //   *
  0x00,  //
  0x00   //
};


// ========================================== Interrupts ==========================================
void IRAM_ATTR buttonInterrupt() {
  bool buttonState = gpio_get_level((gpio_num_t)BUTTON_PIN);
  uint64_t debounceTime = timerRead(xDebounceTimer);

  if (debounceTime - lastDebounceTime > DEBOUNCE_MS) {
    lastDebounceTime = debounceTime;

    if (buttonState == LOW) {
      vTaskNotifyGiveFromISR(xTaskLCD, NULL);
    }
  }
}


// ======================================== Task Functions ========================================
void vTaskLCD(void *pvParameters) {
  const char *days[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
  char upperDisplayBuffer[14];  // DAY  HH:MM AM
  char lowerDisplayBuffer[16];  // MM/DD/YY TT.T*C
  const char *ampm;

  typedef enum {
    STATE_TIME,       // Normal time display mode
    STATE_SET_HOUR,   // Alarm hour-setting mode
    STATE_SET_MINUTE  // Alarm minute-setting mode
  } ClockState;

  ClockState currentState = STATE_TIME;

  DateTime lastTime = DateTime((uint32_t)0);
  DateTime alarmTime = DateTime((uint32_t)0);
  uint8_t hourMap = 0;
  uint8_t minuteMap = 0;

  float lastTemp = 0.0;
  uint16_t lastPot = 0;
  uint8_t lastHourMap = 0;
  uint8_t lastMinuteMap = 0;

  bool backlight = true;
  bool updateLCD = true;

  lcd.createChar(0, bell);  // Creates bell character
  lcd.backlight();          // Starts with backlight on

  while (true) {
    switch (currentState) {
      case STATE_TIME:
        // Checks if potentiometer was moved
        if (xSemaphoreTake(xPhotoPotMutex, portMAX_DELAY)) {
          if (abs(potReading - lastPot) > POT_THRESH) {
            currentState = STATE_SET_HOUR;  // Moves to next state
          }

          xSemaphoreGive(xPhotoPotMutex);  // Gives back mutex
        }

        // Checks for new button notifications
        if (ulTaskNotifyTake(pdTRUE, 0)) {
          if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
            alarmEnabled = !alarmEnabled;  // Toggles state
            xSemaphoreGive(xEnabledMutex);   // Gives back mutex
          }

          updateLCD = true;           // Updates LCD
          xTaskNotifyGive(xTaskRTC);  // Notify to toggle alarm state
        }

        // Checks for time updates
        if (xSemaphoreTake(xTimeTempMutex, portMAX_DELAY)) {
          if (lastTime.minute() != currentTime.minute() || lastTemp != currentTemp) {
            // Updates time and temperature
            lastTime = currentTime;
            lastTemp = currentTemp;
            updateLCD = true;  // Updates LCD
          }

          xSemaphoreGive(xTimeTempMutex);  // Gives back mutex
        }

        // Checks for LCD updates
        if (updateLCD) {
          // Determines whether AM or PM
          if (lastTime.isPM()) {
            ampm = "PM";
          } else {
            ampm = "AM";
          }

          sprintf(upperDisplayBuffer, "%s  %02d:%02d %s", days[lastTime.dayOfTheWeek()], lastTime.twelveHour(), lastTime.minute(), ampm);         // Formats time
          sprintf(lowerDisplayBuffer, "%02d/%02d/%02d  %2.1f%cC", lastTime.month(), lastTime.day(), lastTime.year() % 100, lastTemp, (char)223);  // Formats date/temp

          lcd.clear();
          lcd.print(upperDisplayBuffer);

          if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
            if (alarmEnabled) {
              lcd.setCursor(15, 0);
              lcd.write(0);  // Prints bell
            }

            xSemaphoreGive(xEnabledMutex);  // Gives back mutex
          }

          lcd.setCursor(0, 1);
          lcd.print(lowerDisplayBuffer);
          updateLCD = false;
        }
        break;

      case STATE_SET_HOUR:
        // Checks for new button notifications
        if (ulTaskNotifyTake(pdTRUE, 0)) {
          currentState = STATE_SET_MINUTE;  // Moves to next state
        }

        // Grabs new poteniometer reading once ready
        if (xSemaphoreTake(xPhotoPotMutex, portMAX_DELAY)) {
          lastPot = potReading;
          xSemaphoreGive(xPhotoPotMutex);          // Gives back mutex
          hourMap = map(lastPot, 0, 4095, 0, 23);  // Maps reading to hour
        }

        // Checks if mapped value changed
        if (hourMap != lastHourMap) {
          lastHourMap = hourMap;
          lcd.clear();
          lcd.print("Alarm hour: ");
          lcd.print(hourMap);
        }
        break;

      case STATE_SET_MINUTE:
        // Checks for new button notifications
        if (ulTaskNotifyTake(pdTRUE, 0)) {
          alarmTime = DateTime(0, 0, 0, lastHourMap, lastMinuteMap, 0);  // Sets alarm time
          xQueueSend(xAlarmQueue, &alarmTime, portMAX_DELAY);            // Sends new alarm time

          if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
            alarmEnabled = true;          // Marks alarm as enabled upon receiving mutex
            xSemaphoreGive(xEnabledMutex);  // Gives back mutex
          }

          currentState = STATE_TIME;
          updateLCD = true;
        }

        // Grabs new poteniometer reading once ready
        if (xSemaphoreTake(xPhotoPotMutex, portMAX_DELAY)) {
          lastPot = potReading;
          xSemaphoreGive(xPhotoPotMutex);            // Gives back mutex
          minuteMap = map(lastPot, 0, 4095, 0, 59);  // Maps reading to hour
        }

        // Checks if mapped value changed
        if (minuteMap != lastMinuteMap) {
          lastMinuteMap = minuteMap;
          lcd.setCursor(0, 1);
          lcd.print("Alarm minute: ");
          lcd.print(minuteMap);
          lcd.print(" ");
        }
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(LCD_INTERVAL_MS));
  }
}


void vTaskRTC(void *pvParameters) {
  DateTime alarmTime;
  rtc.disableAlarm(1);  // Disables any past alarm
  rtc.clearAlarm(1);    // Clears any past alarm flag

  while (true) {
    // Waits for time and temperature semaphore to be available
    if (xSemaphoreTake(xTimeTempMutex, portMAX_DELAY)) {
      // Fetches and stores current time and temperature
      currentTime = rtc.now();
      currentTemp = rtc.getTemperature();

      xSemaphoreGive(xTimeTempMutex);  // Gives mutex back
    }

    // Updates alarm time if queue has new data
    if (xQueueReceive(xAlarmQueue, &alarmTime, 0)) {
      rtc.disableAlarm(1);                       // Disables alarm
      rtc.clearAlarm(1);                         // Clears alarm flag
      rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets new alarm time
    }

    // Toggles alarm if notification given
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
        if (alarmEnabled) {
          rtc.disableAlarm(1);  // Disables alarm
          rtc.clearAlarm(1);    // Clears alarm flag
        } else {
          rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets alarm
        }

        xSemaphoreGive(xEnabledMutex);  // gives back mutex
      }
    }

    if (rtc.alarmFired(1)) {

    }

    vTaskDelay(pdMS_TO_TICKS(RTC_INTERVAL_MS));
  }
}


void vTaskIR(void *pvParameters) {
  while (true) {
    int level = digitalRead(IR_PIN);
    if (level == LOW) {
    }

    vTaskDelay(pdMS_TO_TICKS(IR_INTERVAL_MS));
  }
}


void vTaskAlarm(void *pvParameters) {

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}


void vTaskPhotoPot(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(xPhotoPotMutex, portMAX_DELAY)) {
      // Fetches analog readings
      potReading = analogRead(POT_PIN);
      photoReading = analogRead(PHOTO_PIN);

      xSemaphoreGive(xPhotoPotMutex);  // Gives back mutex
    }

    vTaskDelay(pdMS_TO_TICKS(PHOTO_POT_INTERVAL_MS));
  }
}


// ============================================ Setup =============================================
void setup() {
  Serial.begin(115200);                 // Opens serial connection
  Wire.begin(LCD_SDA, LCD_SCL);         // Initializes LCD I2C bus, which is just global Wire bus
  RTC_I2C_BUS.begin(RTC_SDA, RTC_SCL);  // Initializes separate RTC I2C bus

  // Initializes other peripheral pins
  pinMode(IR_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  lcd.init();               // Initializes LCD module
  rtc.begin(&RTC_I2C_BUS);  // Initializes RTC module

  xDebounceTimer = timerBegin(1000);  // Initializes debounce hardware timer at 1kHz

  // Creates semaphores
  xEnabledMutex = xSemaphoreCreateMutex();
  xTimeTempMutex = xSemaphoreCreateMutex();
  xPhotoPotMutex = xSemaphoreCreateMutex();

  // Creates queues
  xAlarmQueue = xQueueCreate(5, sizeof(DateTime));
  xRemoteQueue = xQueueCreate(5, sizeof(uint16_t));

  // Creates tasks
  xTaskCreatePinnedToCore(vTaskRTC, "TaskRTC", 4096, NULL, 4, &xTaskRTC, 1);
  xTaskCreatePinnedToCore(vTaskLCD, "TaskLCD", 8192, NULL, 5, &xTaskLCD, 0);
  xTaskCreatePinnedToCore(vTaskPhotoPot, "TaskPhotoPot", 2048, NULL, 3, NULL, 1);
  //xTaskCreatePinnedToCore(vTaskIR, "TaskIR", 2048, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vTaskAlarm, "TaskAlarm", 4096, NULL, 1, NULL, 1);

  // Attaches button interupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, CHANGE);
}


// ============================================= Loop =============================================
void loop(){};  // Blank because FreeRTOS handles all task scheduling
