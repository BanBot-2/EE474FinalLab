/**
 * @file Final_Project.ino
 * @author Yehoshua Luna & Carter Lee
 * @date 2025-11-27
 * @brief Digital alarm clock project using RTC module, LCD module, IR remote, LEDs, button, and photoresistor.
 *
 * @version 4.0
 * @details
 * ### Update History
 * - **2025-11-26** (Update 4): Yehoshua wrapped up final code
 * - **2025-11-26** (Update 3): Yehoshua finished base function code
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
/** @def POT_PIN
 * @brief Analog pin connected to the potentiometer for setting the alarm time. */
#define POT_PIN 1
/** @def PHOTO_PIN
 * @brief Analog pin connected to the photoresistor (LDR) for ambient light sensing. */
#define PHOTO_PIN 4
/** @def BUTTON_PIN
 * @brief Digital pin connected to the alarm toggle/snooze button. Configured with PULLUP. */
#define BUTTON_PIN 5
/** @def BUZZER_PIN
 * @brief Digital pin connected to the buzzer for the alarm sound. */
#define BUZZER_PIN 6
/** @def RED_LED_PIN
 * @brief Digital pin connected to the Red LED, used for alarm visual indication. */
#define RED_LED_PIN 7
/** @def BLUE_LED_PIN
 * @brief Digital pin connected to the Blue LED, used for alarm visual indication. */
#define BLUE_LED_PIN 17

/** @def LCD_SCL
 * @brief I2C SCL pin for the LCD module. */
#define LCD_SCL 13
/** @def LCD_SDA
 * @brief I2C SDA pin for the LCD module. */
#define LCD_SDA 14
/** @def RTC_SCL
 * @brief I2C SCL pin for the RTC module (on bus 1). */
#define RTC_SCL 35
/** @def RTC_SDA
 * @brief I2C SDA pin for the RTC module (on bus 1). */
#define RTC_SDA 36

/** @def IR_PIN
 * @brief Digital pin connected to the IR receiver module. */
#define IR_PIN 37

// Configuration
/** @def POT_THRESH
 * @brief Threshold value for detecting a significant change in potentiometer reading (used for alarm setting mode transition). */
#define POT_THRESH 400
/** @def IR_DEBOUNCE_MS
 * @brief Debounce time in milliseconds for the IR remote input, preventing rapid accidental triggers. */
#define IR_DEBOUNCE_MS 250
/** @def BACKLIGHT_THRESH
 * @brief Analog reading threshold from the photoresistor to determine if the LCD backlight should be turned off (high light) or on (low light). */
#define BACKLIGHT_THRESH 1280
/** @def BUTTON_DEBOUNCE_US
 * @brief Debounce time in microseconds for the button interrupt, preventing spurious triggers. */
#define BUTTON_DEBOUNCE_US 200000

/** @def IR_INTERVAL_MS
 * @brief Task delay in milliseconds for the IR task (approx 64Hz loop rate). */
#define IR_INTERVAL_MS 16
/** @def RTC_INTERVAL_MS
 * @brief Task delay in milliseconds for the RTC task (approx 32Hz loop rate). */
#define RTC_INTERVAL_MS 32
/** @def LCD_INTERVAL_MS
 * @brief Task delay in milliseconds for the LCD update task (approx 50Hz loop rate). */
#define LCD_INTERVAL_MS 20
/** @def ALARM_INTERVAL_MS
 * @brief Interrupt timer interval in milliseconds for the alarm LED flashing pattern (2Hz toggle rate). */
#define ALARM_INTERVAL_MS 500
/** @def PHOTO_POT_INTERVAL_MS
 * @brief Task delay in milliseconds for the photoresistor and potentiometer reading task (approx 20Hz loop rate). */
#define PHOTO_POT_INTERVAL_MS 50


// ========================================= Object Setup =========================================
/** @brief Separate I2C bus object for the RTC module. */
TwoWire RTC_I2C_BUS = TwoWire(1);
/** @brief LiquidCrystal_I2C object for controlling the 16x2 LCD display. */
LiquidCrystal_I2C lcd(0x27, 16, 2);
/** @brief RTC_DS3231 object for interacting with the DS3231 Real-Time Clock module. */
RTC_DS3231 rtc;


// =========================================== Handles ============================================
/** @brief Queue handle for transmitting potentiometer readings (uint16_t) to the LCD task. */
QueueHandle_t xPotQueue;
/** @brief Queue handle for transmitting photoresistor readings (uint16_t) to the LCD task. */
QueueHandle_t xPhotoQueue;
/** @brief Queue handle for transmitting the newly set alarm time (DateTime) from the LCD task to the RTC task. */
QueueHandle_t xAlarmQueue;

/** @brief Task handle for the Real-Time Clock management task. */
TaskHandle_t xTaskRTC;
/** @brief Task handle for the Liquid Crystal Display update and alarm setting task. */
TaskHandle_t xTaskLCD;
/** @brief Task handle for the Alarm LED flashing pattern task. */
TaskHandle_t xTaskAlarm;
/** @brief Task handle for the Infrared remote receiver handling task. */
TaskHandle_t xTaskIR;

/** @brief Mutex to protect access to the \ref alarmEnabled global variable. */
SemaphoreHandle_t xEnabledMutex;
/** @brief Mutex to protect access to the \ref currentTime and \ref currentTemp global variables. */
SemaphoreHandle_t xTimeTempMutex;

/** @brief Hardware timer handle used for button debounce timing. */
hw_timer_t *xDebounceTimer = NULL;
/** @brief Hardware timer handle used to trigger the alarm flashing pattern. */
hw_timer_t *xAlarmTimer = NULL;


// ======================================= Global Variables =======================================
/** @brief Global variable storing the current time and date fetched from the RTC module.
 * Protected by \ref xTimeTempMutex. */
DateTime currentTime = DateTime((uint32_t)0);
/** @brief Volatile variable storing the last time the button was registered as pressed/released, used for hardware interrupt debouncing. */
volatile uint64_t lastDebounceTime = 0;
/** @brief Global variable storing the current temperature reading from the RTC module in degrees Celsius.
 * Protected by \ref xTimeTempMutex. */
float currentTemp = 0.0;
/** @brief Flag indicating whether the alarm is currently enabled (`true`) or disabled (`false`).
 * Protected by \ref xEnabledMutex. */
bool alarmEnabled = false;

/** @brief Custom character array for the LCD display, representing a small bell icon to indicate the alarm is enabled. */
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
/**
 * @brief Interrupt Service Routine (ISR) triggered by a change on the button pin (\ref BUTTON_PIN).
 *
 * This ISR implements debouncing logic using a hardware timer. If the button state change is stable,
 * it notifies the \ref xTaskLCD to handle the alarm enable/disable toggle or mode change.
 */
void IRAM_ATTR buttonInterrupt() {
  bool buttonState = gpio_get_level((gpio_num_t)BUTTON_PIN);
  uint64_t debounceTime = timerRead(xDebounceTimer);

  if (debounceTime - lastDebounceTime > BUTTON_DEBOUNCE_US) {
    lastDebounceTime = debounceTime;

    if (buttonState == LOW) {
      vTaskNotifyGiveFromISR(xTaskLCD, NULL);
    }
  }
}

/**
 * @brief Interrupt Service Routine (ISR) triggered by a falling edge on the IR receiver pin (\ref IR_PIN).
 *
 * This ISR notifies the \ref xTaskIR to process the IR signal.
 */
void IRAM_ATTR remoteInterrupt() {
  vTaskNotifyGiveFromISR(xTaskIR, NULL);
}

/**
 * @brief Interrupt Service Routine (ISR) triggered by the hardware alarm timer (\ref xAlarmTimer).
 *
 * This ISR notifies the \ref xTaskAlarm to toggle the LED flash pattern.
 */
void IRAM_ATTR alarmInterrupt() {
  vTaskNotifyGiveFromISR(xTaskAlarm, NULL);
}


// ======================================== Task Functions ========================================
/**
 * @brief FreeRTOS Task for managing the Liquid Crystal Display (LCD) and handling alarm setting logic.
 *
 * This task is responsible for:
 * 1. Reading ambient light from \ref xPhotoQueue to control the LCD backlight.
 * 2. Displaying the current time and temperature.
 * 3. Toggling the alarm state based on button input.
 * 4. Transitioning to alarm setting modes (hour and minute) based on potentiometer movement.
 * 5. Reading potentiometer input to set the alarm hour and minute, and sending the new time via \ref xAlarmQueue.
 *
 * @param pvParameters Not used.
 */
void vTaskLCD(void *pvParameters) {
  const char *days[] = { "SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT" };
  char upperDisplayBuffer[32];  // DAY  HH:MM AM
  char lowerDisplayBuffer[32];  // MM/DD/YY TT.T*C
  const char *ampm;

  typedef enum {
    STATE_TIME,       // Normal time display mode
    STATE_SET_HOUR,   // Alarm hour-setting mode
    STATE_SET_MINUTE  // Alarm minute-setting mode
  } ClockState;

  ClockState currentState = STATE_TIME;

  DateTime lastTime = DateTime((uint32_t)0);
  DateTime alarmTime = DateTime((uint32_t)0);
  float lastTemp = 0.0;

  uint16_t potReading = 0;
  uint16_t photoReading = 0;
  uint16_t lastPotReading = 0;

  uint8_t hourMap = 0;
  uint8_t minuteMap = 0;
  uint8_t lastHourMap = 0;
  uint8_t lastMinuteMap = 0;

  bool backlight = true;
  bool updateLCD = true;

  lcd.createChar(0, bell);  // Creates bell character
  lcd.backlight();          // Starts with backlight on

  while (true) {
    xQueueReceive(xPhotoQueue, &photoReading, 0);  // Fetches most recent light reading

    // Checks for backlight updates
    if (photoReading < BACKLIGHT_THRESH) {
      if (backlight == false) {
        lcd.backlight();
        backlight = true;
      }
    } else {
      if (backlight == true) {
        lcd.noBacklight();
        backlight = false;
      }
    }

    switch (currentState) {
      case STATE_TIME:
        // Fetches most recent potentiometer readings
        if (xQueueReceive(xPotQueue, &potReading, 0)) {
          // Checks if potentiometer was moved
          if (abs(potReading - lastPotReading) > POT_THRESH) {
            currentState = STATE_SET_HOUR;  // Moves to next state
          }
        }

        // Checks for new button notifications
        if (ulTaskNotifyTake(pdTRUE, 0)) {
          if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
            alarmEnabled = !alarmEnabled;   // Toggles alarm state
            xSemaphoreGive(xEnabledMutex);  // Gives back mutex
          }

          xTaskNotifyGive(xTaskRTC);  // Notify to toggle alarm state
          updateLCD = true;           // Updates LCD flag
        }

        // Checks for time updates
        if (xSemaphoreTake(xTimeTempMutex, portMAX_DELAY)) {
          if (lastTime.minute() != currentTime.minute() || lastTemp != currentTemp) {
            // Updates time and temperature
            lastTime = currentTime;
            lastTemp = currentTemp;
            updateLCD = true;  // Updates LCD flag
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
        if (xQueueReceive(xPotQueue, &lastPotReading, 0)) {
          hourMap = map(lastPotReading, 0, 4095, 0, 23);  // Maps reading to hour
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
          if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
            alarmEnabled = true;            // Marks alarm as enabled upon receiving mutex
            xSemaphoreGive(xEnabledMutex);  // Gives back mutex
          }

          alarmTime = DateTime(0, 0, 0, lastHourMap, lastMinuteMap, 0);  // Sets alarm time
          xQueueSend(xAlarmQueue, &alarmTime, portMAX_DELAY);            // Sends new alarm time
          currentState = STATE_TIME;
          updateLCD = true;
        }

        // Grabs new poteniometer reading once ready
        if (xQueueReceive(xPotQueue, &lastPotReading, 0)) {
          minuteMap = map(lastPotReading, 0, 4095, 0, 59);  // Maps reading to minute
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

/**
 * @brief FreeRTOS Task for managing the Real-Time Clock (RTC) module.
 *
 * This task is responsible for:
 * 1. Periodically reading the current time and temperature from the RTC and updating \ref currentTime and \ref currentTemp (protected by \ref xTimeTempMutex).
 * 2. Receiving new alarm times from \ref xAlarmQueue and configuring the RTC alarm.
 * 3. Handling notifications from \ref xTaskLCD to enable/disable the RTC alarm setting.
 * 4. Checking if the RTC alarm has fired and starting/stopping the physical alarm (buzzer and LED timers).
 *
 * @param pvParameters Not used.
 */
void vTaskRTC(void *pvParameters) {
  DateTime alarmTime;         // Holds alarm set time
  bool alarmStarted = false;  // Tracks whether alarm has started

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
      rtc.clearAlarm(1);                         // Clears alarm flag
      rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets new alarm time
    }

    // Toggles alarm if notification given
    if (ulTaskNotifyTake(pdTRUE, 0)) {
      if (xSemaphoreTake(xEnabledMutex, portMAX_DELAY)) {
        if (alarmEnabled) {
          rtc.setAlarm1(alarmTime, DS3231_A1_Hour);  // Sets alarm
        } else {
          rtc.setAlarm1(DateTime((uint32_t)0), DS3231_A1_Date);  // Dummy alarm time
          rtc.clearAlarm(1);                                     // Clears alarm flag
        }

        xSemaphoreGive(xEnabledMutex);  // Gives back mutex
      }
    }

    // Manages alarm indicators
    if (rtc.alarmFired(1)) {
      if (!alarmStarted) {
        // Starts buzzer and LEDs
        analogWrite(BUZZER_PIN, 128);
        digitalWrite(RED_LED_PIN, HIGH);
        digitalWrite(BLUE_LED_PIN, HIGH);

        timerRestart(xAlarmTimer);  // Resets timer counter
        timerStart(xAlarmTimer);    // Starts pattern timer
        alarmStarted = true;        // Updates flag
      }
    } else {
      if (alarmStarted) {
        alarmStarted = false;    // Updates flag
        timerStop(xAlarmTimer);  // Stops pattern timer

        // Turns off buzzer and LEDs
        analogWrite(BUZZER_PIN, 0);
        digitalWrite(RED_LED_PIN, LOW);
        digitalWrite(BLUE_LED_PIN, LOW);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(RTC_INTERVAL_MS));
  }
}

/**
 * @brief FreeRTOS Task to handle the blinking pattern for the alarm LEDs.
 *
 * This task waits for notifications from the \ref alarmInterrupt (triggered by \ref xAlarmTimer)
 * and toggles the state of the Red and Blue LEDs (\ref RED_LED_PIN, \ref BLUE_LED_PIN) to create a flashing effect.
 *
 * @param pvParameters Not used.
 */
void vTaskAlarm(void *pvParameters) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                 // Waits for notification
    digitalWrite(RED_LED_PIN, !digitalRead(RED_LED_PIN));    // Toggles red LED
    digitalWrite(BLUE_LED_PIN, !digitalRead(BLUE_LED_PIN));  // Toggles blue LED
  }
}

/**
 * @brief FreeRTOS Task to handle the Infrared (IR) remote input.
 *
 * This task processes notifications from the \ref remoteInterrupt and implements an IR
 * debouncing mechanism. If a valid IR pulse count is detected (indicating a remote button press),
 * it sends a notification to \ref xTaskLCD, simulating a button press (e.g., for snooze/confirm).
 *
 * @param pvParameters Not used.
 */
void vTaskIR(void *pvParameters) {
  uint32_t pulseCountIR = 0;

  while (true) {
    pulseCountIR = ulTaskNotifyTake(pdTRUE, 0);
    
    if (pulseCountIR > 8) {
      xTaskNotifyGive(xTaskLCD);
      vTaskDelay(pdMS_TO_TICKS(IR_DEBOUNCE_MS));
      ulTaskNotifyTake(pdTRUE, 0);
    }

    vTaskDelay(IR_INTERVAL_MS);
  }
}

/**
 * @brief FreeRTOS Task for reading the Photoreistor (LDR) and Potentiometer (POT) analog values.
 *
 * This task reads the analog inputs from \ref POT_PIN and \ref PHOTO_PIN and sends the values
 * to their respective queues (\ref xPotQueue, \ref xPhotoQueue) for other tasks to use.
 *
 * @param pvParameters Not used.
 */
void vTaskPhotoPot(void *pvParameters) {
  uint16_t potReading = 0;
  uint16_t photoReading = 0;

  while (true) {
    potReading = analogRead(POT_PIN);
    photoReading = analogRead(PHOTO_PIN);

    xQueueSend(xPotQueue, &potReading, portMAX_DELAY);
    xQueueSend(xPhotoQueue, &photoReading, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(PHOTO_POT_INTERVAL_MS));
  }
}


// ============================================ Setup =============================================
/**
 * @brief System initialization function.
 *
 * Initializes serial communication, I2C buses for LCD and RTC, configures all peripheral pins,
 * initializes the LCD and RTC modules, creates FreeRTOS synchronization primitives (queues, mutexes),
 * creates all FreeRTOS tasks, and attaches hardware interrupts.
 */
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

  xDebounceTimer = timerBegin(1000000);  // Initializes debounce hardware timer at 1MHz
  xAlarmTimer = timerBegin(1000);        // Initializes alarm hardware timer at 1kHz

  // Creates semaphores
  xEnabledMutex = xSemaphoreCreateMutex();
  xTimeTempMutex = xSemaphoreCreateMutex();

  // Creates queues
  xPotQueue = xQueueCreate(5, sizeof(uint16_t));
  xPhotoQueue = xQueueCreate(5, sizeof(uint16_t));
  xAlarmQueue = xQueueCreate(5, sizeof(DateTime));

  // Creates tasks
  xTaskCreatePinnedToCore(vTaskRTC, "TaskRTC", 4096, NULL, 5, &xTaskRTC, 1);
  xTaskCreatePinnedToCore(vTaskLCD, "TaskLCD", 4096, NULL, 3, &xTaskLCD, 0);
  xTaskCreatePinnedToCore(vTaskPhotoPot, "TaskPhotoPot", 2048, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(vTaskAlarm, "TaskAlarm", 2048, NULL, 6, &xTaskAlarm, 0);
  xTaskCreatePinnedToCore(vTaskIR, "TaskIR", 2048, NULL, 2, &xTaskIR, 1);

  // Attaches interupts
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), remoteInterrupt, FALLING);
  timerAttachInterrupt(xAlarmTimer, &alarmInterrupt);
  timerAlarm(xAlarmTimer, ALARM_INTERVAL_MS, true, 0);  // Configures timer alarm settings
  timerStop(xAlarmTimer);                               // Initially stops the timer
}


// ============================================= Loop =============================================
/**
 * @brief The main Arduino loop function.
 *
 * This function is left blank because all control and scheduling is handled by FreeRTOS tasks.
 */
void loop(){};  // Blank because FreeRTOS handles all task scheduling
