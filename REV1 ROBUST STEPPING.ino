/*
ESP32 Dual-Core Stepper Motor Demo - Final Version
- Uses updated timer API (Core 3.x)
- IRAM-safe GPIO handling
- Proper button edge detection
- Smart OLED updates (including step counter)
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

const int DIR = 16;
const int STEP = 15;
const int EN = 14;
#define BUTTON_A 9
#define BUTTON_B 6
#define BUTTON_C 5

const int microMode = 8;
const int gearRatio = 50;
const int motorStepsPerRev = 200 * microMode;
const int outputStepsPerRev = motorStepsPerRev * gearRatio;

hw_timer_t* stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool moveStepper = false;
volatile uint32_t stepperSpeed = 1000;
volatile bool stepperDir = true;
volatile uint32_t stepsTaken = 0;
volatile uint16_t totalRotations = 0;
volatile bool motorEnabled = true;
volatile bool stepState = false;
volatile uint32_t interruptCount = 0;
volatile bool timerRunning = false;

TaskHandle_t uiTaskHandle = NULL;

enum TestMode {
  MODE_STOPPED = 0,
  MODE_SLOW,
  MODE_MEDIUM,
  MODE_FAST,
  MODE_ULTRA,
  MODE_LUDICROUS,
  MODE_COUNT
};

const char* modeNames[] = {
  "STOPPED", "SLOW", "MEDIUM", "FAST", "ULTRA", "LUDICROUS"
};

const uint32_t modeSpeeds[] = { 0, 800, 2000, 4000, 6000, 8000 };
TestMode currentMode = MODE_STOPPED;
unsigned long lastDisplayUpdate = 0;
unsigned long lastDebounceA = 0, lastDebounceB = 0, lastDebounceC = 0;

void IRAM_ATTR stepTimerCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCount++;
  if (moveStepper && motorEnabled) {
    stepState = !stepState;
    if (stepState)
      GPIO.out_w1ts = (1 << STEP);
    else
      GPIO.out_w1tc = (1 << STEP);

    if (stepState) {
      stepsTaken++;
      if (stepsTaken >= motorStepsPerRev) {
        stepsTaken = 0;
        totalRotations++;
        if (totalRotations > 30000) totalRotations = 0;
      }
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void uiTask(void* pvParameters) {
  Serial.println("Core 1: UI task started");
  for (;;) {
    handleButtons();
    updateMotorControl();
    updateDisplay();
    printStats();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 ESP32 Stepper Demo - Final Version");

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(EN, LOW);
  Serial.println("GPIO pins initialized");  Serial.println("GPIO pins initialized");

  if (!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed!");
  } else {
    Serial.println("OLED initialized successfully");
  }
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  delay(100);  // Allow pull-ups to settle
  Serial.println("Buttons initialized");

  showStartup();
  delay(2000);

  xTaskCreatePinnedToCore(uiTask, "UITask", 8192, NULL, 1, &uiTaskHandle, 1);
  
  Serial.println("System initialized successfully!");
  Serial.println("Controls:");
  Serial.println("Button A: Change mode/speed");
  Serial.println("Button B: Start/Stop motor");
  Serial.println("Button C: Change direction");
  Serial.println();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

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

void showStartup() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("FINAL VERSION");
  display.setCursor(5, 20);
  display.print("STEPPER DEMO");
  display.setCursor(0, 35);
  display.print("HW Timer: Core 0");
  display.setCursor(0, 45);
  display.print("UI Task: Core 1");
  display.setCursor(0, 60);
  display.print("Initializing...");
  display.display();
}

void handleButtons() {
  unsigned long now = millis();
  bool buttonA = !digitalRead(BUTTON_A);
  bool buttonB = !digitalRead(BUTTON_B);
  bool buttonC = !digitalRead(BUTTON_C);
  
  // Static variables for edge detection
  static bool lastButtonA = false;
  static bool lastButtonB = false;
  static bool lastButtonC = false;

  if (buttonA && !lastButtonA && now - lastDebounceA > 200) {
    currentMode = (TestMode)((currentMode + 1) % MODE_COUNT);
    Serial.print("Mode changed to: ");
    Serial.println(modeNames[currentMode]);
    lastDebounceA = now;
  }
  if (buttonB && !lastButtonB && now - lastDebounceB > 200) {
    moveStepper = !moveStepper;
    if (moveStepper) {
      portENTER_CRITICAL(&timerMux);
      stepsTaken = 0;
      totalRotations = 0;
      portEXIT_CRITICAL(&timerMux);
      Serial.println("Motor STARTED - counters reset");
    } else {
      Serial.println("Motor STOPPED");
    }
    lastDebounceB = now;
  }
  if (buttonC && !lastButtonC && now - lastDebounceC > 200) {
    stepperDir = !stepperDir;
    Serial.print("Direction changed to: ");
    Serial.println(stepperDir ? "CW" : "CCW");
    lastDebounceC = now;
  }
  
  // Update previous button states
  lastButtonA = buttonA;
  lastButtonB = buttonB;
  lastButtonC = buttonC;
}

void updateMotorControl() {
  uint32_t newSpeed = (currentMode != MODE_STOPPED) ? modeSpeeds[currentMode] : 0;
  digitalWrite(DIR, stepperDir);

  bool speedChanged = (newSpeed != stepperSpeed);
  stepperSpeed = newSpeed;
  bool shouldRun = moveStepper && stepperSpeed > 0;

  if (shouldRun && !timerRunning) startStepTimer(stepperSpeed);
  else if (!shouldRun && timerRunning) stopStepTimer();
  else if (shouldRun && speedChanged) startStepTimer(stepperSpeed);

  digitalWrite(EN, motorEnabled ? LOW : HIGH);
}

void updateDisplay() {
  static TestMode lastMode = MODE_COUNT;
  static bool lastState = false;
  static bool lastDir = false;
  static uint32_t lastSteps = 0;
  static uint16_t lastRotations = 0;

  if (millis() - lastDisplayUpdate < 200) return;
  
  // Get current step counts
  uint32_t currentSteps, currentRotations;
  portENTER_CRITICAL(&timerMux);
  currentSteps = stepsTaken;
  currentRotations = totalRotations;
  portEXIT_CRITICAL(&timerMux);
  
  // Check if anything changed
  if (currentMode == lastMode && moveStepper == lastState && 
      stepperDir == lastDir && currentSteps == lastSteps && 
      currentRotations == lastRotations) {
    return;  // No changes, skip update
  }

  lastDisplayUpdate = millis();
  lastMode = currentMode;
  lastState = moveStepper;
  lastDir = stepperDir;
  lastSteps = currentSteps;
  lastRotations = currentRotations;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Mode: %s\n", modeNames[currentMode]);
  display.print("Speed: ");
  if (moveStepper && stepperSpeed > 0) {
    if (stepperSpeed >= 1000) {
      display.print(stepperSpeed / 1000.0, 1);
      display.print("k");
    } else {
      display.print(stepperSpeed);
    }
    display.print(" sps\n");
  } else {
    display.print("STOPPED\n");
  }

  display.printf("Dir: %s  %c\n", stepperDir ? "CW" : "CCW", timerRunning ? 'T' : '-');
  display.printf("Steps: %dr+%d\n", currentRotations, currentSteps);
  
  float angle = fmod(((currentRotations * motorStepsPerRev + currentSteps) / (float)outputStepsPerRev) * 360.0, 360.0);
  display.printf("Angle: %.1f\370\n", angle);
  display.print("A:Mode B:Run C:Dir");
  display.display();
}

void printStats() {
  static unsigned long lastPrint = 0;
  static uint32_t lastCount = 0;
  if (millis() - lastPrint > 3000) {
    uint32_t now = interruptCount;
    
    // Thread-safe step counter read
    uint32_t currentSteps, currentRotations;
    portENTER_CRITICAL(&timerMux);
    currentSteps = stepsTaken;
    currentRotations = totalRotations;
    portEXIT_CRITICAL(&timerMux);
    
    Serial.printf("Mode: %s | Speed: %d sps | Steps: %dr+%d | Dir: %s | Moving: %s | Timer: %s | Int/s: %d | Heap: %d\n",
                  modeNames[currentMode], stepperSpeed, currentRotations, currentSteps,
                  stepperDir ? "CW" : "CCW", moveStepper ? "YES" : "NO", 
                  timerRunning ? "RUN" : "STOP", (now - lastCount) / 3, esp_get_free_heap_size());
    lastCount = now;
    lastPrint = millis();
  }
}
