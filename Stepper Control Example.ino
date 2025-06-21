/*
ESP32 Dual-Core Stepper Motor Demo - STABLE VERSION
Uses proper ESP32 timer API to prevent crashes
Core 0: Hardware timer-based step generation (rock solid)
Core 1: OLED display, button handling, user interface
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// OLED Display
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// TMC2209 pins
const int DIR = 16;
const int STEP = 15; 
const int EN = 14;

// OLED Button pins (ESP32-S3 Feather)
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

// Motor parameters
const int microMode = 8;      
const int gearRatio = 50;     
const int motorStepsPerRev = 200 * microMode;  
const int outputStepsPerRev = motorStepsPerRev * gearRatio; 

// Hardware timer (back to simpler approach that works)
hw_timer_t * stepTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Shared variables between cores (volatile for thread safety)
volatile bool moveStepper = false;
volatile uint32_t stepperSpeed = 1000;  // Steps per second
volatile bool stepperDir = true;
volatile uint16_t stepsTaken = 0;       // Keep this small - reset at motor rev
volatile uint16_t totalRotations = 0;   // Track rotations separately
volatile bool motorEnabled = true;
volatile bool stepState = false;
volatile uint32_t interruptCount = 0;   // Debug: count timer interrupts

// Task handles
TaskHandle_t uiTaskHandle = NULL;

// UI variables (Core 1 only)
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
  "STOPPED",
  "SLOW", 
  "MEDIUM",
  "FAST",
  "ULTRA",
  "LUDICROUS"
};

const uint32_t modeSpeeds[] = {
  0,      // STOPPED
  800,    // SLOW - 800 steps/sec (conservative start)
  2000,   // MEDIUM - 2000 steps/sec  
  4000,   // FAST - 4000 steps/sec
  6000,   // ULTRA - 6000 steps/sec
  8000,   // LUDICROUS - 8000 steps/sec (theoretical max for stable operation)
};

TestMode currentMode = MODE_STOPPED;
unsigned long lastDisplayUpdate = 0;

// ===== HARDWARE TIMER INTERRUPT =====

// Timer callback - MUST BE FAST AND SAFE!
void IRAM_ATTR stepTimerCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // Always count interrupts for debugging
  interruptCount++;
  
  if (moveStepper && motorEnabled) {
    stepState = !stepState;
    digitalWrite(STEP, stepState ? HIGH : LOW);
    
    // Count steps on rising edge only
    if (stepState) {
      stepsTaken++;
      
      // Reset at motor revolution to keep numbers small
      if (stepsTaken >= motorStepsPerRev) {
        stepsTaken = 0;
        totalRotations++;
        
        // Prevent any possible overflow
        if (totalRotations > 30000) {
          totalRotations = 0;
        }
      }
    }
  }
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ===== CORE 1: UI AND CONTROL =====

// Core 1 task - everything else
void uiTask(void *pvParameters) {
  Serial.println("Core 1: UI task started");
  
  while (true) {
    handleButtons();
    updateMotorControl();
    updateDisplay();
    printStats();
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz update rate - slower to reduce load
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🚀 ESP32 Stepper Demo - STABLE VERSION");
  
  // Initialize stepper pins first
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, HIGH);
  digitalWrite(EN, LOW);  // Enable motor
  
  // Initialize OLED
  delay(250);
  if (!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed!");
  }
  display.clearDisplay();
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  
  // Initialize buttons
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  
  // Show startup screen
  showStartup();
  delay(2000);
  
  // Create hardware timer (Arduino Core 3.x compatible)
  stepTimer = timerBegin(1000000);  // 1MHz timer frequency
  timerAttachInterrupt(stepTimer, &stepTimerCallback);
  
  Serial.println("Hardware timer created");
  
  // Create timer but don't start it yet
  
  // Create UI task on Core 1 only (let Core 0 handle timer interrupts)
  xTaskCreatePinnedToCore(
    uiTask,               // Task function
    "UITask",             // Task name
    8192,                 // Stack size
    NULL,                 // Parameters
    1,                    // Priority
    &uiTaskHandle,        // Task handle
    1                     // Core 1 for UI
  );
  
  Serial.println("System initialized successfully!");
  Serial.println("Hardware timer on Core 0");
  Serial.println("UI task on Core 1");
  Serial.println();
  Serial.println("Controls:");
  Serial.println("Button A: Change mode/speed (SLOW→MEDIUM→FAST→ULTRA→LUDICROUS)");
  Serial.println("Button B: Start/Stop motor (TOGGLE - single press)");
  Serial.println("Button C: Change direction (TOGGLE - single press)");
  Serial.println();
  Serial.println("Speed modes:");
  Serial.println("  SLOW: 800 sps       MEDIUM: 2000 sps");
  Serial.println("  FAST: 4000 sps      ULTRA: 6000 sps");
  Serial.println("  LUDICROUS: 8000 sps (theoretical max)");
}

void loop() {
  // Main loop runs on Core 0, but just handles basic maintenance
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void startStepTimer(uint32_t stepsPerSecond) {
  if (stepTimer == NULL || stepsPerSecond == 0) return;
  
  // Stop timer first
  timerStop(stepTimer);
  
  // Safety check - don't exceed theoretical maximum
  if (stepsPerSecond > 10000) {
    Serial.println("WARNING: Speed capped at 10kHz for safety");
    stepsPerSecond = 10000;
  }
  
  // Calculate alarm value in timer ticks
  // Timer runs at 1MHz, we need 2 events per step (HIGH and LOW)
  uint64_t alarmValue = 500000 / stepsPerSecond;  // Half period in microseconds
  
  // Hardware limit check - ESP32 timer minimum is ~50μs for stable operation
  if (alarmValue < 50) {
    Serial.println("WARNING: Timer period too short, adjusting to minimum safe value");
    alarmValue = 50;
    stepsPerSecond = 500000 / alarmValue;
  }
  
  // Configure alarm with autoreload, unlimited count
  timerAlarm(stepTimer, alarmValue, true, 0);  // alarm_value, autoreload, reload_count
  
  // Explicitly start the timer (this might be the missing piece!)
  timerStart(stepTimer);
  
  Serial.print("Timer configured and started at ");
  Serial.print(stepsPerSecond);
  Serial.print(" steps/sec (alarm every ");
  Serial.print(alarmValue);
  Serial.println(" μs)");
}

void stopStepTimer() {
  if (stepTimer != NULL) {
    timerStop(stepTimer);
  }
  digitalWrite(STEP, LOW);
  stepState = false;
  Serial.println("Timer stopped");
}

void showStartup() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("STABLE MODE");
  display.setCursor(5, 20);
  display.print("STEPPER DEMO");
  display.setCursor(0, 35);
  display.print("HW Timer: Core 0");
  display.setCursor(0, 45);
  display.print("UI Task: Core 1");
  display.setCursor(0, 60);
  display.print("Starting...");
  display.display();
}

void handleButtons() {
  unsigned long now = millis();

  // Read current button states
  bool buttonA = !digitalRead(BUTTON_A);
  bool buttonB = !digitalRead(BUTTON_B);
  bool buttonC = !digitalRead(BUTTON_C);

  // Static variables to track previous button states for edge detection
  static bool lastButtonA = false;
  static bool lastButtonB = false;
  static bool lastButtonC = false;

  // Independent timestamps for debouncing
  static unsigned long lastButtonATime = 0;
  static unsigned long lastButtonBTime = 0;
  static unsigned long lastButtonCTime = 0;
  const unsigned long debounce = 100;  // 100ms debounce per button

  // Button A - Mode change (on press, not hold)
  if (buttonA && !lastButtonA && (now - lastButtonATime > debounce)) {
    currentMode = (TestMode)((currentMode + 1) % MODE_COUNT);
    Serial.print("Mode changed to: ");
    Serial.println(modeNames[currentMode]);
    lastButtonATime = now;
  }

  // Button B - Start/Stop toggle (on press, not hold)
  if (buttonB && !lastButtonB && (now - lastButtonBTime > debounce)) {
    moveStepper = !moveStepper;
    if (moveStepper) {
      stepsTaken = 0;  // Reset step counter
      totalRotations = 0;  // Reset rotation counter
      Serial.println("Motor started - counters reset");
    } else {
      Serial.println("Motor stopped");
    }
    lastButtonBTime = now;
  }

  // Button C - Direction change (on press, not hold)
  if (buttonC && !lastButtonC && (now - lastButtonCTime > debounce)) {
    stepperDir = !stepperDir;
    Serial.print("Direction changed to: ");
    Serial.println(stepperDir ? "CW" : "CCW");
    lastButtonCTime = now;
  }

  // Update previous button states
  lastButtonA = buttonA;
  lastButtonB = buttonB;
  lastButtonC = buttonC;
}

void updateMotorControl() {
  // Update motor speed based on current mode
  uint32_t newSpeed = 0;
  
  if (currentMode != MODE_STOPPED) {
    newSpeed = modeSpeeds[currentMode];
  }
  
  // Update direction pin every time (simple and reliable)
  digitalWrite(DIR, stepperDir ? HIGH : LOW);
  
  // Only update timer if speed changed
  if (newSpeed != stepperSpeed) {
    stepperSpeed = newSpeed;
    
    if (moveStepper && stepperSpeed > 0) {
      startStepTimer(stepperSpeed);
    } else {
      stopStepTimer();
    }
  }
  
  // Handle motor enable/disable
  digitalWrite(EN, motorEnabled ? LOW : HIGH);
}

void updateDisplay() {
  if (millis() - lastDisplayUpdate < 200) return;  // 5Hz update rate
  lastDisplayUpdate = millis();
  
  display.clearDisplay();
  
  // Mode
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.print(modeNames[currentMode]);
  
  // Status and speed
  display.setCursor(0, 12);
  display.print("Speed: ");
  if (moveStepper && stepperSpeed > 0) {
    if (stepperSpeed >= 1000) {
      display.print(stepperSpeed / 1000.0, 1);
      display.print("k");
    } else {
      display.print(stepperSpeed);
    }
    display.print(" sps");
  } else {
    display.print("STOPPED");
  }
  
  // Direction
  display.setCursor(0, 24);
  display.print("Dir: ");
  display.print(stepperDir ? "CW" : "CCW");
  
  // Step count with rotations
  display.setCursor(0, 36);
  display.print("Steps: ");
  if (totalRotations > 0) {
    display.print(totalRotations);
    display.print("r+");
    display.print(stepsTaken);
  } else {
    display.print(stepsTaken);
  }
  
  // Output position in degrees  
  float totalSteps = (totalRotations * motorStepsPerRev) + stepsTaken;
  float outputDegrees = (totalSteps / outputStepsPerRev) * 360.0;
  display.setCursor(0, 48);
  display.print("Angle: ");
  display.print(fmod(outputDegrees, 360.0), 1);
  display.print("°");
  
  // Controls
  display.setCursor(0, 60);
  display.print("A:Mode B:Run C:Dir");
  
  display.display();
}

void printStats() {
  static unsigned long lastStats = 0;
  static uint32_t lastInterruptCount = 0;
  
  if (millis() - lastStats > 3000) {  // Every 3 seconds
    uint32_t currentInterrupts = interruptCount;
    uint32_t interruptsPerSecond = (currentInterrupts - lastInterruptCount) / 3;
    
    Serial.print("Mode: ");
    Serial.print(modeNames[currentMode]);
    Serial.print(" (");
    Serial.print(modeSpeeds[currentMode]);
    Serial.print(" sps)");
    Serial.print(", Steps: ");
    Serial.print(totalRotations);
    Serial.print("r+");
    Serial.print(stepsTaken);
    Serial.print(", Dir: ");
    Serial.print(stepperDir ? "CW" : "CCW");
    Serial.print(", Interrupts/sec: ");
    Serial.print(interruptsPerSecond);
    Serial.print(", moveStepper: ");
    Serial.print(moveStepper ? "YES" : "NO");
    Serial.print(", Free heap: ");
    Serial.print(esp_get_free_heap_size());
    Serial.println(" bytes");
    
    lastStats = millis();
    lastInterruptCount = currentInterrupts;
  }
}

// Utility functions
void setMotorSpeed(uint32_t stepsPerSecond) {
  stepperSpeed = stepsPerSecond;
  if (moveStepper && stepperSpeed > 0) {
    startStepTimer(stepperSpeed);
  } else {
    stopStepTimer();
  }
}

void startMotor() {
  moveStepper = true;
  stepsTaken = 0;
  totalRotations = 0;
  if (stepperSpeed > 0) {
    startStepTimer(stepperSpeed);
  }
}

void stopMotor() {
  moveStepper = false;
  stopStepTimer();
}

uint32_t getTotalSteps() {
  return (totalRotations * motorStepsPerRev) + stepsTaken;
}
