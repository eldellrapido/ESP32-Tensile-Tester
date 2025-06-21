/*
ESP32 Dual-Core Stepper Motor Demo - Tensile Machine Version
- Uses updated timer API (Core 3.x)
- IRAM-safe GPIO handling with REG_WRITE
- Menu-based UI for tensile testing
- Tracks distance traveled during jog
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "driver/gpio.h"
#include "soc/gpio_reg.h"

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

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
const float mmPerRev = 8.0f;  // Lead screw: 8mm per revolution
const float mmPerStep = mmPerRev / outputStepsPerRev;

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
volatile uint32_t interruptCount = 0;
volatile bool timerRunning = false;
volatile int32_t jogStepCounter = 0; // ISR-safe counter for jog distance

// UI variables
float jogDistanceMM = 0.0f; // Converted in main loop
TaskHandle_t uiTaskHandle = NULL;

// UI states
enum SystemState {
  STATE_MENU,
  STATE_JOG,
  STATE_TEST,
  STATE_ULTIMATE_RESULT
};

SystemState currentState = STATE_MENU;
uint8_t menuIndex = 0;
const uint8_t menuItemCount = 5;

// Configurable rates and test type
float jogRateOptions[] = {0.2f, 0.4f, 0.8f};
uint8_t jogRateIndex = 0;
float testRate_mmps = 0.2f;
bool isUltimateTest = true;

// Button debounce
unsigned long lastDebounceA = 0, lastDebounceB = 0, lastDebounceC = 0;

// Menu item labels
const char* getMenuItemLabel(uint8_t index) {
  static char buffer[40];
  switch (index) {
    case 0: return "Jog Mode";
    case 1: snprintf(buffer, sizeof(buffer), "Jog Rate [%.1f mm/s]", jogRateOptions[jogRateIndex]); return buffer;
    case 2: return "Test Mode";
    case 3: snprintf(buffer, sizeof(buffer), "Test Rate [%.1f mm/s]", testRate_mmps); return buffer;
    case 4: snprintf(buffer, sizeof(buffer), "Test Type [%s]", isUltimateTest ? "Ultimate" : "Graph"); return buffer;
    default: return "";
  }
}

// ===== HARDWARE TIMER INTERRUPT =====
void IRAM_ATTR stepTimerCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCount++;
  if (moveStepper && motorEnabled) {
    stepState = !stepState;
    if (stepState)
      REG_WRITE(GPIO_OUT_W1TS_REG, (1 << STEP));
    else
      REG_WRITE(GPIO_OUT_W1TC_REG, (1 << STEP));

    if (stepState) {
      stepsTaken++;
      jogStepCounter += (stepperDir ? 1 : -1);
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
  
  Serial.print("Starting timer at ");
  Serial.print(sps);
  Serial.print(" steps/sec...");
  
  if (stepTimer) {
    timerEnd(stepTimer);
    stepTimer = NULL;
    delay(10);
  }
  
  // Safety check
  if (sps > 10000) {
    Serial.print(" (capped at 10kHz)");
    sps = 10000;
  }
  
  stepTimer = timerBegin(1000000);
  if (!stepTimer) {
    Serial.println(" FAILED");
    return;
  }
  
  uint64_t alarmValue = 500000 / sps;
  if (alarmValue < 50) {
    alarmValue = 50;
    sps = 500000 / alarmValue;
    Serial.print(" (adjusted for stability)");
  }
  
  timerAttachInterrupt(stepTimer, &stepTimerCallback);
  timerAlarm(stepTimer, alarmValue, true, 0);
  timerRunning = true;
  Serial.println(" SUCCESS");
}

void stopStepTimer() {
  Serial.print("Stopping timer...");
  if (stepTimer) {
    timerEnd(stepTimer);
    stepTimer = NULL;
  }
  REG_WRITE(GPIO_OUT_W1TC_REG, (1 << STEP));
  stepState = false;
  timerRunning = false;
  Serial.println(" done");
}

// ===== TENSILE MACHINE FUNCTIONS =====
void startJog(float rate_mmps) {
  // Convert mm/s to steps/sec with safety checks
  uint32_t sps = rate_mmps / mmPerStep;
  
  // Safety limits
  if (sps < 10) sps = 10;      // Minimum practical speed
  if (sps > 10000) sps = 10000; // Maximum safe speed
  
  stepperSpeed = sps;
  digitalWrite(DIR, stepperDir);
  startStepTimer(sps);
  
  Serial.print("Jog started: ");
  Serial.print(rate_mmps);
  Serial.print(" mm/s (");
  Serial.print(sps);
  Serial.println(" sps)");
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
    case STATE_MENU:
      if (a && !la && now - lastDebounceA > 200) {
        lastDebounceA = now;
        menuIndex = (menuIndex + menuItemCount - 1) % menuItemCount;
      }
      if (b && !lb && now - lastDebounceB > 200) {
        lastDebounceB = now;
        menuIndex = (menuIndex + 1) % menuItemCount;
      }
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        switch (menuIndex) {
          case 0: 
            currentState = STATE_JOG; 
            jogStepCounter = 0; 
            Serial.println("Entered jog mode");
            break;
          case 1: 
            jogRateIndex = (jogRateIndex + 1) % 3; 
            Serial.print("Jog rate changed to: ");
            Serial.println(jogRateOptions[jogRateIndex]);
            break;
          case 2: 
            currentState = STATE_TEST; 
            startJog(testRate_mmps); 
            Serial.println("Started test mode");
            break;
          case 3: 
            testRate_mmps += 0.05f; 
            if (testRate_mmps > 1.0f) testRate_mmps = 0.05f; 
            Serial.print("Test rate changed to: ");
            Serial.println(testRate_mmps);
            break;
          case 4: 
            isUltimateTest = !isUltimateTest; 
            Serial.print("Test type changed to: ");
            Serial.println(isUltimateTest ? "Ultimate" : "Graph");
            break;
        }
      }
      break;

    case STATE_JOG:
      // Stop any existing movement first
      if ((a && !la) || (b && !lb)) {
        moveStepper = false;
        stopStepTimer();
        delay(5); // Brief pause between operations
      }
      
      if (a && !la) {
        lastDebounceA = now;
        stepperDir = false;  // Reverse
        moveStepper = true;
        startJog(jogRateOptions[jogRateIndex]);
      } else if (!a && la) {
        moveStepper = false;
        stopStepTimer();
      }

      if (b && !lb) {
        lastDebounceB = now;
        stepperDir = true;   // Forward
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
        currentState = STATE_MENU;
        Serial.println("Exited jog mode");
      }
      break;

    case STATE_TEST:
      if (c && !lc && now - lastDebounceC > 200) {
        lastDebounceC = now;
        moveStepper = false;
        stopStepTimer();
        currentState = STATE_MENU;
        Serial.println("Test stopped, returned to menu");
      }
      break;
  }

  la = a; lb = b; lc = c;
}

void updateDisplay() {
  display.clearDisplay();
  
  switch (currentState) {
    case STATE_MENU:
      // Main menu: white background, black text
      display.fillScreen(SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
      
      // Compact centered title
      display.setCursor(18, 1);  // Tighter top margin
      display.println("=== MAIN MENU ===");
      display.setCursor(1, 12);  // Start menu items closer to title
      
      for (uint8_t i = 0; i < menuItemCount; i++) {
        if (i == menuIndex) {
          display.print(">");
          display.setCursor(display.getCursorX() + 1, display.getCursorY()); // 1px gap after cursor
        } else {
          display.print(" ");
          display.setCursor(display.getCursorX() + 1, display.getCursorY()); // 1px gap after space
        }
        display.println(getMenuItemLabel(i));
        display.setCursor(1, display.getCursorY() + 1);  // Just 1px spacing for more items
      }
      break;
      
    case STATE_JOG:
      // Submenu: black background, white text
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      // Compact title
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
      
      display.setCursor(1, display.getCursorY() + 1);
      display.println("A:REV B:FWD C:EXIT");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("(CCW) (CW)");
      break;
      
    case STATE_TEST:
      // Submenu: black background, white text
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      // Compact title
      display.setCursor(20, 1);
      display.println("=== TEST MODE ===");
      
      display.setCursor(1, 12);
      display.print("Rate: ");
      display.print(testRate_mmps);
      display.println(" mm/s");
      
      display.setCursor(1, display.getCursorY() + 1);
      display.print("Type: ");
      display.println(isUltimateTest ? "Ultimate" : "Graph");
      
      display.setCursor(1, display.getCursorY() + 2);
      display.println("Running...");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("C = STOP");
      break;
      
    case STATE_ULTIMATE_RESULT:
      // Submenu: black background, white text
      display.fillScreen(SH110X_BLACK);
      display.setTextColor(SH110X_WHITE);
      
      // Compact title
      display.setCursor(22, 1);
      display.println("=== RESULTS ===");
      
      display.setCursor(1, 12);
      display.println("Max Force:");
      display.setCursor(1, display.getCursorY() + 1);
      display.println("[placeholder]");
      
      display.setCursor(1, display.getCursorY() + 2);
      display.println("Returning...");
      break;
  }
  display.display();
}

// ===== SETUP AND MAIN LOOP =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 ESP32 Tensile Machine - Final Version");

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(EN, LOW);
  Serial.println("GPIO pins initialized");

  if (!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed!");
  } else {
    Serial.println("OLED initialized successfully");
  }
  display.setRotation(1);  // Keep original landscape orientation
  display.setTextSize(1);      // Use size 1 (6x8 pixels per character)
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);  // Black text on white background

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  delay(100);
  Serial.println("Buttons initialized");

  xTaskCreatePinnedToCore(uiTask, "UITask", 8192, NULL, 1, &uiTaskHandle, 1);
  
  Serial.println("System initialized successfully!");
  Serial.println("Controls:");
  Serial.println("Menu: A=Up B=Down C=Select");
  Serial.println("Jog: A=Reverse B=Forward C=Exit");
  Serial.println();
}

void loop() {
  // Main loop on Core 0 - minimal workload
  vTaskDelay(pdMS_TO_TICKS(1000));
}
