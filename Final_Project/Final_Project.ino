/**
 * @file Final_Project.ino
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


// =========================================== Includes ===========================================
#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include <IRRemoteESP32.h>
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

#define ALARM_PIN 47
#define IR_PIN 48

// Configuration
#define DEBOUNCE_MS 50
#define BACKLIGHT_THRESH 1000

#define IR_INTERVAL_MS 32         // 32Hz
#define RTC_INTERVAL_MS 20        // 50Hz
#define LCD_INTERVAL_MS 250       // 4Hz
#define POT_INTERVAL_US 50000     // 20Hz
#define PHOTO_INTERVAL_US 100000  // 10Hz


// ========================================= Object Setup =========================================
TwoWire RTC_I2C_BUS = TwoWire(1);    // Separate I2C bus for RTC
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD setup on global Wire I2C bus
RTC_DS3231 rtc;                      // Initializes RTC library object


// =========================================== Handles ============================================
QueueHandle_t xAlarmQueue;
QueueHandle_t xRemoteQueue;

TaskHandle_t xTaskIR;
TaskHandle_t xTaskRTC;
TaskHandle_t xTaskLCD;
TaskHandle_t xTaskAlarm;

esp_timer_handle_t tPotTimer;
esp_timer_handle_t tPhotoTimer;

SemaphoreHandle_t xTimeTempSemaphore;


// ======================================= Global Variables =======================================
DateTime currentTime;
float currentTemp;

volatile unsigned long lastChangeTime = 0;

uint8_t bell[8] = {
  0x04, //   *  
  0x0E, //  *** 
  0x0E, //  *** 
  0x0E, //  *** 
  0x1F, // ***** 
  0x04, //   *  
  0x00, //      
  0x00  //     
};


// ========================================== Interrupts ==========================================
void IRAM_ATTR potTimer(void *arg) {
  xTaskNotifyFromISR(xTaskLCD, 0x01, eSetBits, NULL);
}


void IRAM_ATTR photoTimer(void *arg) {
  xTaskNotifyFromISR(xTaskLCD, 0x02, eSetBits, NULL);
}


void IRAM_ATTR buttonInterrupt() {
  unsigned long now = millis();
  bool state = digitalRead(BUTTON_PIN);

  if (now - lastChangeTime > DEBOUNCE_MS) {
    lastChangeTime = now;

    if (state == LOW) {
      xTaskNotifyFromISR(xTaskLCD, 0x03, eSetBits, NULL);
    }
  }
}


// ======================================== Task Functions ========================================
void vTaskLCD(void *pvParameters) {
  bool backlight = true;
  char displayBuffer[6]; // HH:MM

  uint32_t notification;
  uint16_t potReading = 0;
  uint16_t photoReading = 0;

  lcd.createChar(0, bell); // Creates bell character
  lcd.backlight(); // Starts with backlight on

  while (true) {
    // Checks for new notifications
    if (xTaskNotifyWait(0, 0xFF, &notification, 0) == pdTRUE) {

      // Handles photo timer notification (0x02)
      if (notification & 0x02) {
        photoReading = analogRead(PHOTO_PIN);

        if (photoReading >= BACKLIGHT_THRESH && backlight) {
          lcd.noBacklight();
          backlight = false;
        } else if (photoReading < BACKLIGHT_THRESH && !backlight) {
          lcd.backlight();
          backlight = true;
        }
      }

      // Handles button notification (0x03)
      if (notification & 0x03) {
        xTaskNotifyGive(xTaskRTC); // Notify alarm to toggle state
      }
    }

    // Updates the LCD display every cycle
    if (xSemaphoreTake(xTimeTempSemaphore, portMAX_DELAY) == pdTRUE) {
      sprintf(displayBuffer, "%02d:%02d", currentTime.twelveHour(), currentTime.minute());

      lcd.clear();
      lcd.print("Time: ");
      lcd.print(displayBuffer);

      lcd.setCursor(15, 0);
      lcd.print(0);

      lcd.setCursor(0, 1);
      lcd.print("Temp: ");
      lcd.print(currentTemp);
      lcd.print((char)223);  // degree symbol
      lcd.print("C");

      xSemaphoreGive(xTimeTempSemaphore);
    }

    vTaskDelay(pdMS_TO_TICKS(LCD_INTERVAL_MS));
  }
}


void vTaskRTC(void *pvParameters) {
  DateTime alarmTime;
  bool armed = false;

  rtc.clearAlarm(1);  // Clears any past alarm

  while (true) {
    // Waits for time and temperature variables to be open
    if (xSemaphoreTake(xTimeTempSemaphore, portMAX_DELAY) == pdTRUE) {
      // Fetches current time and temperature
      currentTime = rtc.now();
      currentTemp = rtc.getTemperature();

      xSemaphoreGive(xTimeTempSemaphore);  // Returns semaphore
    }

    // Updates alarm time if queue has new data
    if (xQueueReceive(xAlarmQueue, &alarmTime, 0)) {
      rtc.clearAlarm(1);                         // Clears alarm
      rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets new alarm time
    }

    // Toggles alarm if notification given
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      if (armed) {
        rtc.disableAlarm(1);  // Disables alarm
      } else {
        rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets alarm
      }
      armed = !armed;  // Toggles state
    }

    // Sends notification if alarm fired
    if (rtc.alarmFired(1)) {
    }

    vTaskDelay(pdMS_TO_TICKS(RTC_INTERVAL_MS));
  }
}


void vTaskIR(void *pvParameters) {

  while (true) {

    vTaskDelay(pdMS_TO_TICKS(IR_INTERVAL_MS));
  }
}


void vTaskAlarm(void *pvParameters) {

  while (true) {
  }
}


// ============================================ Setup =============================================
void setup() {
  // Timer configurations
  const esp_timer_create_args_t pot_args = { .callback = &potTimer, .arg = NULL, .name = "Pot Timer" };
  const esp_timer_create_args_t photo_args = { .callback = &photoTimer, .arg = NULL, .name = "Photo Timer" };

  Serial.begin(115200);                 // Opens serial connection
  Wire.begin(LCD_SDA, LCD_SCL);         // Initializes LCD I2C bus, which is just global Wire bus
  RTC_I2C_BUS.begin(RTC_SDA, RTC_SCL);  // Initializes separate RTC I2C bus

  lcd.init();               // Initializes LCD module
  rtc.begin(&RTC_I2C_BUS);  // Initializes RTC module

  // Initializes other peripheral pins
  pinMode(IR_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(ALARM_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  // Creates semaphore
  xTimeTempSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(xTimeTempSemaphore);

  // Creates queues
  xAlarmQueue = xQueueCreate(5, sizeof(DateTime));
  xRemoteQueue = xQueueCreate(5, sizeof(uint16_t));

  // Creates tasks
  xTaskCreatePinnedToCore(vTaskRTC, "TaskRTC", 2048, NULL, 0, &xTaskRTC, 0);
  xTaskCreatePinnedToCore(vTaskLCD, "TaskLCD", 2048, NULL, 1, &xTaskLCD, 1);

  // Creates timer interrupts
  esp_timer_create(&pot_args, &tPotTimer);
  esp_timer_create(&photo_args, &tPhotoTimer);

  // Starts timer interrupts
  esp_timer_start_periodic(tPotTimer, POT_INTERVAL_US);
  esp_timer_start_periodic(tPhotoTimer, PHOTO_INTERVAL_US);

  // Attaches button interupt
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, CHANGE);
}


// ============================================= Loop =============================================
void loop(){};  // Blank because FreeRTOS handles all task scheduling
