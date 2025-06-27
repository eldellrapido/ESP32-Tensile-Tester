/*
ESP32 Dual-Core Tensile Machine - Phase 2A: Professional Data Acquisition
- FreeRTOS Event Groups for task coordination
- Dedicated 80Hz data acquisition task on Core 1
- Circular buffer for synchronized force/position logging
- NAU7802 load cell integration with calibration
- Paginated menu system for calibration management
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <EEPROM.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h"

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
NAU7802 myScale;

// GPIO pins
const int DIR = 16;
const int STEP = 15;
const int EN = 14;
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

// Motor parameters
const int microMode = 8;      
const int gearRatio = 50;     
const int motorStepsPerRev = 200 * microMode;  
const int outputStepsPerRev = motorStepsPerRev * gearRatio; 

// Tensile machine calculations
const float mmPerRev = 8.0f;
const float mmPerStep = mmPerRev / outputStepsPerRev;

// EEPROM locations for NAU7802 calibration
#define EEPROM_SIZE 100
#define LOCATION_CALIBRATION_FACTOR 0
#define LOCATION_ZERO_OFFSET 10
#define LOCATION_SETTINGS_VALID 20

// FreeRTOS Event Group for task coordination
EventGroupHandle_t testEventFlags;
#define EVT_TEST_START       (1 << 0)
#define EVT_TEST_STOP        (1 << 1)
#define EVT_CALIBRATE_START  (1 << 2)
#define EVT_CALIBRATE_DONE   (1 << 3)
#define EVT_TARE_REQUEST     (1 << 4)
#define EVT_DATA_READY       (1 << 5)

// Data acquisition circular buffer
#define BUFFER_SIZE 1000  // ~12.5 seconds at 80Hz
#define MAX_TEST_DURATION_POINTS (BUFFER_SIZE * 10)  // Auto-stop after ~2 minutes
#define AUTO_STOP_ON_BUFFER_FULL true  // Compile-time toggle

struct TestDataPoint {
  uint64_t timestamp_us;    // Microseconds since test start for precision
  float force_N;            // Force in Newtons  
  float position_mm;        // Position in mm
  uint32_t step_count;      // Raw step count
  bool valid;               // Data point validity
};

TestDataPoint dataBuffer[BUFFER_SIZE];
volatile uint16_t bufferWriteIndex = 0;
volatile bool bufferWrapped = false;
volatile uint32_t totalDataPoints = 0;

// Live test data (always current)
struct LiveTestData {
  float currentForce_N;
  float peakForce_N;
  float currentPosition_mm;
  uint32_t totalSteps;          // Updated in dataAcquisitionTask, not ISR
  uint64_t testStartTime_us;    // Microsecond precision start time
  bool testActive;
  bool scaleCalibrated;
  bool bufferOverflow;          // Warning flag for data overflow
  bool sensorSaturated;         // ADC clipping detection
} liveData = {0};

// Hardware timer
hw_timer_t* stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Shared variables between cores (volatile for thread safety)
volatile bool moveStepper = false;
volatile uint32_t stepperSpeed = 1000;
volatile bool stepperDir = true;
volatile uint32_t stepsTaken = 0;
volatile uint16_t totalRotations = 0;
volatile bool motorEnabled = true;
volatile bool stepState = false;
volatile bool timerRunning = false;
volatile int32_t jogStepCounter = 0;

// Task handles
TaskHandle_t uiTaskHandle = NULL;
TaskHandle_t dataTaskHandle = NULL;

// UI variables
float jogDistanceMM = 0.0f;

// UI states with pagination
enum SystemState {
  STATE_MENU_MAIN,
  STATE_MENU_CALIBRATION,
  STATE_JOG,
  STATE_TEST,
  STATE_CALIBRATING_TARE,
  STATE_CALIBRATING_WEIGHT,
  STATE_ULTIMATE_RESULT
};

enum CalibrationStep {
  CAL_IDLE,
  CAL_TARE_WAIT,
  CAL_WEIGHT_WAIT,
  CAL_COMPLETE
};

enum MenuPage {
  PAGE_MAIN = 0,
  PAGE_CALIBRATION = 1,
  PAGE_COUNT = 2
};

SystemState currentState = STATE_MENU_MAIN;
CalibrationStep calStep = CAL_IDLE;
MenuPage currentPage = PAGE_MAIN;
uint8_t menuIndex = 0;
uint8_t blinkCounter = 0;  // For blinking menu cursor

// Menu configurations
const uint8_t mainMenuItemCount = 6;
const uint8_t calMenuItemCount = 4;

// Configurable test modes and rates
enum TestMode {
  TEST_ULTIMATE = 0,
  TEST_GRAPH = 1,
  TEST_FATIGUE = 2,    // Future expansion
  TEST_CREEP = 3       // Future expansion
};
#define TEST_MAX_AVAILABLE 2  // Only Ultimate/Graph currently implemented

float jogRateOptions[] = {0.2f, 0.4f, 0.8f};
uint8_t jogRateIndex = 0;
float testRate_mmps = 0.2f;
TestMode currentTestMode = TEST_ULTIMATE;

// Button debounce
unsigned long lastDebounceA = 0, lastDebounceB = 0, lastDebounceC = 0;

// ===== UTILITY FUNCTIONS =====
uint32_t getStepsTakenSafe() {
  // Thread-safe read of steps taken from ISR
  portENTER_CRITICAL(&timerMux);
  uint32_t steps = stepsTaken; // Read ISR-updated variable
  portEXIT_CRITICAL(&timerMux);
  return steps;
}

float readForceN() {
  if (!liveData.scaleCalibrated) return 0.0f;
  
  if (myScale.available()) {
    int32_t rawReading = myScale.getReading();
    
    // Check for ADC saturation (24-bit signed: ±8,388,607)
    if (abs(rawReading) > 8000000) {  // Near saturation threshold
      liveData.sensorSaturated = true;
    }
    
    return myScale.getWeight(true, 4); // Allow negative, average 4 readings
  }
  return liveData.currentForce_N; // Return last known value if not ready
}

void recordDataPoint() {
  if (!liveData.testActive) return;
  
  // Check for buffer overflow
  if (totalDataPoints >= MAX_TEST_DURATION_POINTS && AUTO_STOP_ON_BUFFER_FULL) {
    liveData.bufferOverflow = true;
    stopTest();
    Serial.println("Test auto-stopped: Maximum data points reached");
    return;
  }
  
  TestDataPoint& point = dataBuffer[bufferWriteIndex];
  point.timestamp_us = esp_timer_get_time() - liveData.testStartTime_us;
  point.force_N = liveData.currentForce_N;
  point.position_mm = liveData.currentPosition_mm;
  point.step_count = getStepsTakenSafe(); // Use ISR-safe step count
  point.valid = true;
  
  bufferWriteIndex = (bufferWriteIndex + 1) % BUFFER_SIZE;
  if (bufferWriteIndex == 0) bufferWrapped = true;
  totalDataPoints++;
  
  xEventGroupSetBits(testEventFlags, EVT_DATA_READY);
}

void printBufferToSerial() {
  Serial.println("=== TEST DATA EXPORT ===");
  Serial.println("Timestamp_us,Force_N,Position_mm,Step_Count");
  
  uint16_t startIndex = bufferWrapped ? bufferWriteIndex : 0;
  uint16_t count = bufferWrapped ? BUFFER_SIZE : bufferWriteIndex;
  
  for (uint16_t i = 0; i < count; i++) {
    uint16_t index = (startIndex + i) % BUFFER_SIZE;
    TestDataPoint& point = dataBuffer[index];
    
    if (point.valid) {
      Serial.printf("%llu,%.3f,%.3f,%lu\n", 
        point.timestamp_us, point.force_N, 
        point.position_mm, point.step_count);
    }
  }
  Serial.println("=== END DATA ===");
}

const char* getTestModeString(TestMode mode) {
  switch (mode) {
    case TEST_ULTIMATE: return "Ultimate";
    case TEST_GRAPH: return "Graph";
    case TEST_FATIGUE: return "Fatigue";
    case TEST_CREEP: return "Creep";
    default: return "Unknown";
  }
}

void handleSerialCommand(String command) {
  command.trim();
  command.toLowerCase();
  
  if (command == "export") {
    printBufferToSerial();
  } 
  else if (command.startsWith("cal ")) {
    float weight = command.substring(4).toFloat();
    if (weight > 0 && liveData.scaleCalibrated) {
      myScale.calculateCalibrationFactor(weight, 64);
      saveCalibrationData();
      Serial.printf("Calibrated with %.1fg\n", weight);
    } else {
      Serial.println("Invalid weight or scale not detected");
    }
  }
  else if (command == "tare") {
    if (liveData.scaleCalibrated) {
      myScale.calculateZeroOffset(64);
      saveCalibrationData();
      Serial.println("Scale tared");
    } else {
      Serial.println("Scale not detected");
    }
  }
  else if (command == "calstatus") {
    if (liveData.scaleCalibrated) {
      Serial.printf("Cal Factor: %.3f\n", myScale.getCalibrationFactor());
      Serial.printf("Zero Offset: %ld\n", myScale.getZeroOffset());
    } else {
      Serial.println("Scale not calibrated");
    }
  }
  else if (command == "status") {
    Serial.println("=== SYSTEM STATUS ===");
    Serial.printf("Scale: %s\n", liveData.scaleCalibrated ? "Calibrated" : "Not calibrated");
    Serial.printf("Test active: %s\n", liveData.testActive ? "Yes" : "No");
    Serial.printf("Current force: %.2f N\n", liveData.currentForce_N);
    Serial.printf("Current position: %.2f mm\n", liveData.currentPosition_mm);
    Serial.printf("Data points: %lu\n", totalDataPoints);
    Serial.printf("Buffer overflow: %s\n", liveData.bufferOverflow ? "Yes" : "No");
    Serial.printf("Sensor saturated: %s\n", liveData.sensorSaturated ? "Yes" : "No");
    if (liveData.scaleCalibrated) {
      Serial.printf("Cal Factor: %.3f\n", myScale.getCalibrationFactor());
      Serial.printf("Zero Offset: %ld\n", myScale.getZeroOffset());
    }
  }
  else if (command == "reset") {
    Serial.println("Restarting system...");
    esp_restart();  // Lower-level restart
  }
  else if (command == "help") {
    Serial.println("Available commands:");
    Serial.println("  export - Export test data as CSV");
    Serial.println("  cal <weight> - Calibrate with known weight in grams");
    Serial.println("  tare - Zero the scale");
    Serial.println("  calstatus - Show calibration values");
    Serial.println("  status - Show system status");
    Serial.println("  reset - Restart system");
    Serial.println("  help - Show this help");
  }
  else {
    Serial.println("Unknown command. Type 'help' for available commands.");
  }
}

// ===== EEPROM FUNCTIONS =====
void saveCalibrationData() {
  EEPROM.put(LOCATION_CALIBRATION_FACTOR, myScale.getCalibrationFactor());
  EEPROM.put(LOCATION_ZERO_OFFSET, myScale.getZeroOffset());
  EEPROM.put(LOCATION_SETTINGS_VALID, (uint8_t)0xAA); // Magic number
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM");
}

bool loadCalibrationData() {
  uint8_t valid;
  EEPROM.get(LOCATION_SETTINGS_VALID, valid);
  
  if (valid == 0xAA) {
    float calFactor;
    int32_t zeroOffset;
    EEPROM.get(LOCATION_CALIBRATION_FACTOR, calFactor);
    EEPROM.get(LOCATION_ZERO_OFFSET, zeroOffset);
    
    myScale.setCalibrationFactor(calFactor);
    myScale.setZeroOffset(zeroOffset);
    
    Serial.print("Loaded calibration: factor=");
    Serial.print(calFactor, 3);
    Serial.print(", offset=");
    Serial.println(zeroOffset);
    
    return true;
  }
  
  Serial.println("No valid calibration found in EEPROM");
  return false;
}

// ===== MENU FUNCTIONS =====
const char* getMainMenuLabel(uint8_t index) {
  static char buffer[40];
  switch (index) {
    case 0: return "Jog Mode";
    case 1: snprintf(buffer, sizeof(buffer), "Jog Rate [%.1f mm/s]", jogRateOptions[jogRateIndex]); return buffer;
    case 2: return "Test Mode";
    case 3: snprintf(buffer, sizeof(buffer), "Test Rate [%.1f mm/s]", testRate_mmps); return buffer;
    case 4: snprintf(buffer, sizeof(buffer), "Test Type [%s]", getTestModeString(currentTestMode)); return buffer;
    case 5: return "Calibration >";
    default: return "";
  }
}

const char* getCalMenuLabel(uint8_t index) {
  static char buffer[40];
  switch (index) {
    case 0: snprintf(buffer, sizeof(buffer), "Status [%s]", liveData.scaleCalibrated ? "Ready" : "Needed"); return buffer;
    case 1: return "Guided Calibration";
    case 2: return "Tare Scale Only";
    case 3: return "< Back to Main";
    default: return "";
  }
}

// ===== HARDWARE TIMER INTERRUPT =====
void IRAM_ATTR stepTimerCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (moveStepper && motorEnabled) {
    stepState = !stepState;
    if (stepState)
      REG_WRITE(GPIO_OUT_W1TS_REG, (1 << STEP));
    else
      REG_WRITE(GPIO_OUT_W1TC_REG, (1 << STEP));

    if (stepState) {
      stepsTaken++;
      jogStepCounter += (stepperDir ? 1 : -1);
      // Removed liveData.totalSteps update from ISR for thread safety
      
      if (stepsTaken >= motorStepsPerRev) {
        stepsTaken = 0;
        totalRotations++;
        if (totalRotations > 30000) totalRotations = 0;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ===== TIMER FUNCTIONS =====
void startStepTimer(uint32_t sps) {
  if (sps == 0) return stopStepTimer();
  
  if (stepTimer) {
    timerEnd(stepTimer);
    stepTimer = NULL;
    delay(10);
  }
  
  if (sps > 10000) sps = 10000;
  
  stepTimer = timerBegin(1000000);
  if (!stepTimer) return;
  
  uint64_t alarmValue = 500000 / sps;
  if (alarmValue < 50) alarmValue = 50;
  
  timerAttachInterrupt(stepTimer, &stepTimerCallback);
  timerAlarm(stepTimer, alarmValue, true, 0);
  timerRunning = true;
}

void stopStepTimer() {
  if (stepTimer) {
    timerEnd(stepTimer);
    stepTimer = NULL;
  }
  REG_WRITE(GPIO_OUT_W1TC_REG, (1 << STEP));
  stepState = false;
  timerRunning = false;
}

// ===== TENSILE MACHINE FUNCTIONS =====
void startJog(float rate_mmps) {
  uint32_t sps = rate_mmps / mmPerStep;
  if (sps < 10) sps = 10;
  if (sps > 10000) sps = 10000;
  
  stepperSpeed = sps;
  digitalWrite(DIR, stepperDir);
  startStepTimer(sps);
}

void startTest() {
  liveData.testActive = true;
  liveData.testStartTime_us = esp_timer_get_time();
  liveData.peakForce_N = 0.0f;
  liveData.bufferOverflow = false;
  bufferWriteIndex = 0;
  bufferWrapped = false;
  totalDataPoints = 0;
  
  stepperDir = true; // Always forward for tests
  moveStepper = true;
  startJog(testRate_mmps);
  
  xEventGroupSetBits(testEventFlags, EVT_TEST_START);
  Serial.printf("Test started: %s mode\n", getTestModeString(currentTestMode));
}

void stopTest() {
  liveData.testActive = false;
  moveStepper = false;
  stopStepTimer();
  
  // Calculate test duration
  float testDuration_s = (float)(esp_timer_get_time() - liveData.testStartTime_us) / 1000000.0f;
  
  xEventGroupSetBits(testEventFlags, EVT_TEST_STOP);
  Serial.printf("Test completed in %.2f seconds, %lu points collected\n", 
                testDuration_s, totalDataPoints);
  Serial.print("Peak force: ");
  Serial.print(liveData.peakForce_N, 2);
  Serial.println(" N");
}

// ===== CORE 1: DATA ACQUISITION TASK =====
void dataAcquisitionTask(void* pvParameters) {
  Serial.println("Core 1: Data acquisition task started");
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t frequency = pdMS_TO_TICKS(12); // ~80Hz
  
  for (;;) {
    // Update current position and total steps (thread-safe)
    liveData.currentPosition_mm = jogStepCounter * mmPerStep;
    liveData.totalSteps = getStepsTakenSafe(); // Use corrected function name
    
    // Read force if scale is available
    if (liveData.scaleCalibrated) {
      float newForce = readForceN();
      liveData.currentForce_N = newForce;
      
      // Track peak force during testing
      if (liveData.testActive && newForce > liveData.peakForce_N) {
        liveData.peakForce_N = newForce;
      }
      
      // Record data point if test is active
      if (liveData.testActive) {
        recordDataPoint();
        
        // Ultimate test: check for force drop (break detection)
        if (currentTestMode == TEST_ULTIMATE && liveData.peakForce_N > 5.0f) {
          static float lastForce = 0;
          static uint8_t dropCount = 0;
          
          if (newForce < lastForce * 0.85f) { // 15% drop
            dropCount++;
            if (dropCount >= 5) { // Confirm drop over 5 readings
              stopTest();
              currentState = STATE_ULTIMATE_RESULT;
            }
          } else {
            dropCount = 0;
          }
          lastForce = newForce;
        }
      }
    }
    
    // Handle tare requests
    EventBits_t bits = xEventGroupGetBits(testEventFlags);
    if (bits & EVT_TARE_REQUEST) {
      if (liveData.scaleCalibrated) {
        myScale.calculateZeroOffset(64);
        saveCalibrationData();
        Serial.println("Scale tared");
      }
      xEventGroupClearBits(testEventFlags, EVT_TARE_REQUEST);
    }
    
    vTaskDelayUntil(&lastWakeTime, frequency);
  }
}

// ===== CORE 1: UI TASK =====
void uiTask(void* pvParameters) {
  Serial.println("Core 1: UI task started");
  for (;;) {
    handleButtons();
    jogDistanceMM = jogStepCounter * mmPerStep;
    updateDisplay();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void handleButtons() {
  unsigned long now = millis();
  bool a = !digitalRead(BUTTON_A);
  bool b = !digitalRead(BUTTON_B);
  bool c = !digitalRead(BUTTON_C);
  static bool la = false, lb = false, lc = false;

  switch (currentState) {
    case STATE_MENU_MAIN:
      if (currentPage == PAGE_MAIN) {
        if (a && !la && now - lastDebounceA > 200) {
          lastDebounceA = now;
          menuIndex = (menuIndex + mainMenuItemCount - 1) % mainMenuItemCount;
        }
        if (b && !lb && now - lastDebounceB > 200) {
          lastDebounceB = now;
          menuIndex = (menuIndex + 1) % mainMenuItemCount;
        }
        if (c && !lc && now - lastDebounceC > 200) {
          lastDebounceC = now;
          switch (menuIndex) {
            case 0: 
              currentState = STATE_JOG; 
              jogStepCounter = 0; 
              break;
            case 1: 
              jogRateIndex = (jogRateIndex + 1) % 3; 
              break;
            case 2: 
              currentState = STATE_TEST; 
              startTest();
              break;
            case 3: 
              testRate_mmps += 0.05f; 
              if (testRate_mmps > 1.0f) testRate_mmps = 0.05f; 
              break;
            case 4: 
              currentTestMode = (TestMode)((currentTestMode + 1) % TEST_MAX_AVAILABLE);
              break;
            case 5:
              currentPage = PAGE_CALIBRATION;
              currentState = STATE_MENU_CALIBRATION;
              menuIndex = 0;
              break;
          }
        }
      }
      break;

    case STATE_MENU_CALIBRATION:
      if (a && !la && now - lastDebounceA > 200) {
        lastDebounceA = now;
        menuIndex = (menuIndex + calMenuItemCount - 1) % calMenuItemCount;
      }
      if (b && !lb && now - lastDebounceB > 200) {
        lastDebounceB = now;
        menuIndex = (menuIndex + 1) % calMenuItemCount;
      }
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        switch (menuIndex) {
          case 0: break; // Status display only
          case 1: 
            currentState = STATE_CALIBRATING_TARE;
            calStep = CAL_TARE_WAIT;
            Serial.println("GUIDED CALIBRATION: Remove all weight from scale, then press C");
            break;
          case 2: 
            xEventGroupSetBits(testEventFlags, EVT_TARE_REQUEST);
            Serial.println("Taring scale...");
            break;
          case 3:
            currentPage = PAGE_MAIN;
            currentState = STATE_MENU_MAIN;
            menuIndex = 0;
            break;
        }
      }
      break;

    case STATE_JOG:
      if ((a && !la) || (b && !lb)) {
        moveStepper = false;
        stopStepTimer();
        delay(5);
      }
      
      if (a && !la) {
        lastDebounceA = now;
        stepperDir = false;
        moveStepper = true;
        startJog(jogRateOptions[jogRateIndex]);
      } else if (!a && la) {
        moveStepper = false;
        stopStepTimer();
      }

      if (b && !lb) {
        lastDebounceB = now;
        stepperDir = true;
        moveStepper = true;
        startJog(jogRateOptions[jogRateIndex]);
      } else if (!b && lb) {
        moveStepper = false;
        stopStepTimer();
      }

      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        moveStepper = false;
        stopStepTimer();
        currentState = STATE_MENU_MAIN;
      }
      break;

    case STATE_TEST:
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        stopTest();
        currentState = STATE_MENU_MAIN;
      }
      break;

    case STATE_ULTIMATE_RESULT:
      // Auto-return to menu after 10 seconds
      static uint32_t resultStartTime = 0;
      if (resultStartTime == 0) resultStartTime = millis();
      
      if (millis() - resultStartTime > 10000 || (c && !lc)) {
        resultStartTime = 0;
        currentState = STATE_MENU_MAIN;
      }
      break;

    case STATE_CALIBRATING_TARE:
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        if (liveData.scaleCalibrated && calStep == CAL_TARE_WAIT) {
          myScale.calculateZeroOffset(64);
          currentState = STATE_CALIBRATING_WEIGHT;
          calStep = CAL_WEIGHT_WAIT;
          Serial.println("Tare complete. Place known weight (e.g., 100g), enter weight in serial, then press C");
        }
      }
      break;
      
    case STATE_CALIBRATING_WEIGHT:
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        if (liveData.scaleCalibrated && calStep == CAL_WEIGHT_WAIT) {
          // In real implementation, would read weight from serial
          // For now, just assume 100g calibration
          myScale.calculateCalibrationFactor(100.0f, 64); // Assume 100g weight
          saveCalibrationData();
          calStep = CAL_COMPLETE;
          Serial.println("Calibration complete! Scale is ready for use.");
          currentState = STATE_MENU_CALIBRATION;
        }
      }
      break;
  }

  la = a; lb = b; lc = c;
}

void updateDisplay() {
  display.clearDisplay();
  blinkCounter = (blinkCounter + 1) % 10; // Blink cycle for cursor
  
  switch (currentState) {
    case STATE_MENU_MAIN:
      display.fillScreen(SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
      
      display.setCursor(18, 1);
      display.println("=== MAIN MENU ===");
      display.setCursor(1, 12);
      
      for (uint8_t i = 0; i < mainMenuItemCount; i++) {
        if (i == menuIndex) {
          // Blinking cursor or inverted text
          if (blinkCounter < 5) {
            display.fillRect(0, display.getCursorY(), 128, 8, SH110X_BLACK);
            display.setTextColor(SH110X_WHITE);
            display.print(">");
            display.setCursor(display.getCursorX() + 1, display.getCursorY());
            display.println(getMainMenuLabel(i));
            display.setTextColor(SH110X_BLACK);
          } else {
            display.print(">");
            display.setCursor(display.getCursorX() + 1, display.getCursorY());
            display.println(getMainMenuLabel(i));
          }
        } else {
          display.print(" ");
          display.setCursor(display.getCursorX() + 1, display.getCursorY());
          display.println(getMainMenuLabel(i));
        }
        display.setCursor(1, display.getCursorY() + 1);
      }
      break;
      
    case STATE_MENU_CALIBRATION:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(15, 1);
      display.println("=== CALIBRATION ===");
      display.setCursor(1, 12);
      
      for (uint8_t i = 0; i < calMenuItemCount; i++) {
        if (i == menuIndex) {
          if (blinkCounter < 5) {
            display.fillRect(0, display.getCursorY(), 128, 8, SH110X_WHITE);
            display.setTextColor(SH110X_BLACK);
            display.print(">");
            display.setCursor(display.getCursorX() + 1, display.getCursorY());
            display.println(getCalMenuLabel(i));
            display.setTextColor(SH110X_WHITE);
          } else {
            display.print(">");
            display.setCursor(display.getCursorX() + 1, display.getCursorY());
            display.println(getCalMenuLabel(i));
          }
        } else {
          display.print(" ");
          display.setCursor(display.getCursorX() + 1, display.getCursorY());
          display.println(getCalMenuLabel(i));
        }
        display.setCursor(1, display.getCursorY() + 1);
      }
      break;
      
    case STATE_JOG:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(23, 1);
      display.println("=== JOG MODE ===");
      
      display.setCursor(1, 12);
      display.print("Rate: ");
      display.print(jogRateOptions[jogRateIndex]);
      display.println(" mm/s");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Dist: ");
      display.print(jogDistanceMM, 1);
      display.println(" mm");
      
      if (liveData.scaleCalibrated) {
        display.setCursor(1, display.getCursorY() + 1);
        display.print("Force: ");
        display.print(liveData.currentForce_N, 1);
        display.println(" N");
      }
      
      display.setCursor(1, display.getCursorY() + 2);
      display.println("A:REV B:FWD C:EXIT");
      break;
      
    case STATE_TEST:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(20, 1);
      display.println("=== TEST MODE ===");
      
      display.setCursor(1, 12);
      display.print("Rate: ");
      display.print(testRate_mmps);
      display.println(" mm/s");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Type: ");
      display.println(getTestModeString(currentTestMode));
      
      if (liveData.scaleCalibrated) {
        display.setCursor(1, display.getCursorY() + 1);
        display.print("Force: ");
        display.print(liveData.currentForce_N, 1);
        display.println(" N");
        
        display.setCursor(1, display.getCursorY() + 1);
        display.print("Peak: ");
        display.print(liveData.peakForce_N, 1);
        display.println(" N");
      }
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Pos: ");
      display.print(liveData.currentPosition_mm, 1);
      display.println(" mm");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Data: ");
      display.print(totalDataPoints);
      display.println(" pts");
      
      // Show test duration
      if (liveData.testActive) {
        float testTime_s = (float)(esp_timer_get_time() - liveData.testStartTime_us) / 1000000.0f;
        display.setCursor(1, display.getCursorY() + 1);
        display.print("Time: ");
        display.print(testTime_s, 1);
        display.println("s");
      }
      
      if (liveData.bufferOverflow) {
        display.setCursor(1, display.getCursorY() + 1);
        display.println("! BUFFER FULL !");
      }
      
      if (liveData.sensorSaturated) {
        display.setCursor(1, display.getCursorY() + 1);
        display.println("! SENSOR CLIPPED !");
      }
      
      // Progress indicator for buffer usage
      {
        uint16_t bufferPercent = (totalDataPoints * 100) / BUFFER_SIZE;
        if (bufferPercent > 50) { // Only show when buffer is getting full
          display.setCursor(1, display.getCursorY() + 1);
          display.print("Buf: ");
          display.print(bufferPercent);
          display.println("%");
        }
      }
      
      display.setCursor(1, display.getCursorY() + 1);
      display.println("C = STOP");
      break;
      
    case STATE_ULTIMATE_RESULT:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(22, 1);
      display.println("=== RESULTS ===");
      
      display.setCursor(1, 12);
      display.println("ULTIMATE STRENGTH:");
      display.setCursor(1, display.getCursorY() + 2);
      display.print("Max Force: ");
      display.print(liveData.peakForce_N, 1);
      display.println(" N");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Final Pos: ");
      display.print(liveData.currentPosition_mm, 1);
      display.println(" mm");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Final Force: ");
      display.print(liveData.currentForce_N, 1);
      display.println(" N");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Data Points: ");
      display.println(totalDataPoints);
      
      // Show test duration
      if (liveData.testStartTime_us > 0) {
        float testDuration_s = (float)(esp_timer_get_time() - liveData.testStartTime_us) / 1000000.0f;
        display.setCursor(1, display.getCursorY() + 1);
        display.print("Duration: ");
        display.print(testDuration_s, 1);
        display.println("s");
      }
      
      if (liveData.sensorSaturated) {
        display.setCursor(1, display.getCursorY() + 1);
        display.println("! CLIPPED DATA !");
      }
      
      display.setCursor(1, display.getCursorY() + 2);
      display.println("Auto-return in 10s");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("or press C");
      break;
      
    case STATE_CALIBRATING_TARE:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(25, 1);
      display.println("=== TARE ===");
      
      display.setCursor(1, 15);
      display.println("Remove all weight");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("from scale");
      
      // Show current force reading
      if (liveData.scaleCalibrated) {
        display.setCursor(1, display.getCursorY() + 2);
        display.print("Force: ");
        display.print(liveData.currentForce_N, 2);
        display.println(" N");
      }
      
      display.setCursor(1, display.getCursorY() + 2);
      display.println("Press C when ready");
      break;
      
    case STATE_CALIBRATING_WEIGHT:
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      display.setCursor(18, 1);
      display.println("=== CALIBRATE ===");
      
      display.setCursor(1, 15);
      display.println("Place known weight");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("Enter weight in");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("serial: cal 100");
      
      // Show current force reading
      if (liveData.scaleCalibrated) {
        display.setCursor(1, display.getCursorY() + 2);
        display.print("Force: ");
        display.print(liveData.currentForce_N, 2);
        display.println(" N");
      }
      
      display.setCursor(1, display.getCursorY() + 1);
      display.println("Press C to complete");
      break;
  }
  display.display();
}

// ===== SETUP AND MAIN LOOP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Tensile Machine - Phase 2A");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Initialize Event Groups
  testEventFlags = xEventGroupCreate();
  if (testEventFlags == NULL) {
    Serial.println("Failed to create event group!");
    while(1);
  }

  // GPIO initialization
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(EN, LOW);

  // Initialize I2C and NAU7802
  Wire.begin();
  Wire.setClock(400000);
  
  if (myScale.begin() == false) {
    Serial.println("NAU7802 not detected. Continuing without load cell...");
    liveData.scaleCalibrated = false;
  } else {
    Serial.println("NAU7802 detected!");
    myScale.setSampleRate(NAU7802_SPS_80);
    myScale.setGain(NAU7802_GAIN_16);
    myScale.setLDO(NAU7802_LDO_3V0);
    myScale.calibrateAFE();
    
    liveData.scaleCalibrated = loadCalibrationData();
    if (!liveData.scaleCalibrated) {
      Serial.println("Scale needs calibration");
    }
  }

  // Initialize display
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed!");
  }
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);

  // Initialize buttons
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // Create tasks
  xTaskCreatePinnedToCore(dataAcquisitionTask, "DataTask", 8192, NULL, 2, &dataTaskHandle, 1);
  xTaskCreatePinnedToCore(uiTask, "UITask", 8192, NULL, 1, &uiTaskHandle, 1);
  
  Serial.println("System initialized successfully!");
  Serial.println("Phase 2A: Professional data acquisition ready");
  Serial.println();
  Serial.println("Special commands:");
  Serial.println("  'export' - Export test data as CSV");
  Serial.println("  'cal <weight>' - Calibrate with known weight");
  Serial.println("  'tare' - Zero the scale");
  Serial.println("  'calstatus' - Show calibration values");
  Serial.println("  'status' - Show system status");
  Serial.println("  'reset' - Restart system");
  Serial.println("  'help' - Show all commands");
}

void loop() {
  // Main loop on Core 0 - handle serial commands and minimal coordination
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
  
  vTaskDelay(pdMS_TO_TICKS(1000));
}
