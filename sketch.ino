/**
 * @brief Secure Industrial Loading Gate - Scenario 1
 * @details Three modes: Autonomous, Manual, Safety Halt (Option C)
 * @CBNumber : CB015780
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// ===================== PINS =====================
#define PIN_SERVO      13
#define PIN_PIR        23
#define PIN_POT        34
#define PIN_BUZZER     12
#define PIN_BTN        14
#define PIN_TRIG       5
#define PIN_ECHO       18
#define PIN_LED        2

// ===================== SETTINGS =====================
#define OLED_ADDR      0x3C
#define GATE_CLOSED    0
#define GATE_OPEN      90
#define SAFE_DIST      20      // cm
#define STEP_MS        15      // servo step speed

// ===================== STATES =====================
enum State {
  IDLE,
  OPENING,
  HOLD,
  CLOSING,
  MANUAL,
  SAFETY
};

// ===================== OBJECTS =====================
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
Servo gate;

// ===================== VARIABLES =====================
State state = IDLE;

int angle = GATE_CLOSED;
long dist = 999;
unsigned long holdTime = 4000;

unsigned long tState = 0;
unsigned long tServo = 0;
unsigned long tBuzz  = 0;
unsigned long tOled  = 0;
unsigned long tSerial = 0;
unsigned long tLed   = 0;
unsigned long tBuzzTone = 0;

bool buzzOn = false;
bool ledOn  = false;
bool manOpen = false;

// --- Hardware Timer for Buzzer (Safety Halt) ---
hw_timer_t *buzzTimer = NULL;
volatile bool buzzToggle = false;
volatile bool timerBuzzActive = false;

/**
 * @brief Toggle the buzzer output using the ESP32 hardware timer interrupt.
 * @return void
 */
void IRAM_ATTR onBuzzTimer() {
  if (timerBuzzActive) {
    buzzToggle = !buzzToggle;
    digitalWrite(PIN_BUZZER, buzzToggle);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }
}

// ===================== FUNCTION PROTOTYPES =====================
void readSensors();
long getDistance();
void handleSerial();
void runFSM();
void showOLED();
void showSerial();
void updateLED();

// ===================== SETUP =====================
/**
 * @brief Initialise all pins and peripherals.
 * @return void
 */
void setup() {
  Serial.begin(115200);
  Serial.println("Industrial Gate Controller Ready");

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_POT, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_LED, HIGH);

  buzzTimer = timerBegin(1000000);                 
  timerAttachInterrupt(buzzTimer, &onBuzzTimer);
  timerAlarm(buzzTimer, 250, true, 0);             

  ESP32PWM::allocateTimer(0);
  gate.setPeriodHertz(50);
  gate.attach(PIN_SERVO, 500, 2400);
  gate.write(GATE_CLOSED);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed");
    while (1);
  }

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(10, 20);
  oled.println("INDUSTRIAL GATE");
  oled.setCursor(10, 35);
  oled.println("AUTO - READY");
  oled.display();
}

// ===================== MAIN LOOP =====================
/**
 * @brief Execute the main non-blocking control loop.
 * @return void
 */
void loop() {
  readSensors();
  handleSerial();
  runFSM();
  showOLED();
  showSerial();
  updateLED();
}

// ===================== SENSORS =====================
/**
 * @brief Read the potentiometer and ultrasonic sensor.
 * @return void
 */
void readSensors() {
  int pot = analogRead(PIN_POT);

  // ADC value 0-4095 controls hold delay from 3-10 seconds.
  holdTime = map(pot, 0, 4095, 3000, 10000);

  dist = getDistance();
}

/**
 * @brief Measure distance using the HC-SR04 ultrasonic sensor.
 * @return Distance in centimetres, or 999 when no echo is received.
 */
long getDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(PIN_TRIG, LOW);

  long us = pulseIn(PIN_ECHO, HIGH, 25000);

  if (us == 0) {
    return 999;
  }

  return us * 0.0343 / 2;
}

// ===================== STATE MACHINE =====================
/**
 * @brief Run the gate finite state machine and handle state transitions.
 * @return void
 */
void runFSM() {
  unsigned long now = millis();

  switch (state) {

    // ---------- IDLE ----------
    case IDLE:

      digitalWrite(PIN_BUZZER, LOW);
      buzzOn = false;

      if (digitalRead(PIN_PIR) == HIGH) {
        Serial.println("[AUTO] Vehicle detected - Opening");
        state = OPENING;
        tState = now;
      }
      break;

    // ---------- OPENING ----------
    case OPENING:
      if (now - tServo >= STEP_MS) {
        tServo = now;

        if (angle < GATE_OPEN) {
          angle++;
          gate.write(angle);
        } else {
          Serial.println("[AUTO] Gate fully open");
          state = HOLD;
          tState = now;
        }
      }
      break;

    // ---------- HOLD OPEN ----------
    case HOLD:
      if (now - tState >= holdTime) {
        Serial.println("[AUTO] Hold finished - Closing");
        state = CLOSING;
        tState = now;
      }
      break;

    // ---------- CLOSING ----------
    case CLOSING:

      // Safety check
      if (dist < SAFE_DIST) {
        Serial.println("!! SAFETY HALT - Obstacle < 20cm");
        state = SAFETY;
        tState = now;
        break;
      }

      if (now - tServo >= STEP_MS) {
        tServo = now;

        if (angle > GATE_CLOSED) {
          angle--;
          gate.write(angle);
        } else {
          Serial.println("[AUTO] Gate closed - Ready");
          state = IDLE;
        }
      }
      break;

    // ---------- MANUAL ----------
    case MANUAL:

      // Safety still active
      if (dist < SAFE_DIST && angle > GATE_CLOSED) {
        Serial.println("!! SAFETY HALT in Manual Mode");
        state = SAFETY;
        tState = now;
        break;
      }

      // Button controls gate
      if (digitalRead(PIN_BTN) == LOW) {

        tState = now; 
        if (now - tServo >= STEP_MS) {
          tServo = now;

          if (!manOpen && angle < GATE_OPEN) {
            angle++;
            gate.write(angle);

            if (angle >= GATE_OPEN) {
              manOpen = true;
            }

          } else if (manOpen && angle > GATE_CLOSED) {
            angle--;
            gate.write(angle);

            if (angle <= GATE_CLOSED) {
              manOpen = false;
            }
          }
        }
      }

      // Timeout back to Auto
      if (now - tState >= 10000) {
        Serial.println("[MANUAL] Timeout -> Auto");
        state = IDLE;
        tState = now;
      }
      break;

    // ---------- SAFETY HALT (Option C) ----------
    case SAFETY:

      timerBuzzActive = true;

      // Reset only if path is clear
      if (digitalRead(PIN_BTN) == LOW) {

        if (dist >= SAFE_DIST) {
          Serial.println("[RESET] Path clear - Re-opening");

          timerBuzzActive = false;

          state = OPENING;
          tState = now;

        } else {
          Serial.println("[RESET] BLOCKED - Obstacle still there");
        }
      }
      break;
  }

  // Enter Manual Mode by holding button for 2 seconds while IDLE
  if (state == IDLE && digitalRead(PIN_BTN) == LOW) {

    if (now - tState >= 2000) {
      Serial.println("[MODE] Manual Mode");

      state = MANUAL;
      tState = now;
      manOpen = false;
    }

  } else if (state == IDLE) {
    tState = now;
  }
}

// ===================== OLED =====================
/**
 * @brief Update the OLED display at approximately 10 Hz.
 * @return void
 */
void showOLED() {
  if (millis() - tOled < 100) {
    return;
  }

  tOled = millis();

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println(" Industrial Gate");
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  oled.setCursor(0, 16);
  oled.print("MODE: ");

  if (state == IDLE)
    oled.println("AUTO READY");
  else if (state == OPENING)
    oled.println("OPENING...");
  else if (state == HOLD)
    oled.println("GATE OPEN");
  else if (state == CLOSING)
    oled.println("CLOSING...");
  else if (state == MANUAL)
    oled.println("MANUAL");
  else if (state == SAFETY)
    oled.println("SAFETY BLOCK!");

  oled.setCursor(0, 32);

  if (state == SAFETY) {
    oled.print("OBSTACLE: ");
    oled.print(dist);
    oled.println(" cm");

  } else if (state == HOLD || state == IDLE) {
    oled.print("HOLD: ");
    oled.print(holdTime / 1000.0, 1);
    oled.println(" s");

  } else if (state == MANUAL) {
    oled.println("PRESS BTN TO MOVE");

  } else {
    oled.println("PATH: CLEAR");
  }

  if (state == SAFETY) {
    oled.fillRect(0, 50, 128, 14, SSD1306_WHITE);

    oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    oled.setCursor(16, 53);
    oled.print("PRESS RESET BTN");

    oled.setTextColor(SSD1306_WHITE);

  } else {
    oled.drawLine(0, 48, 127, 48, SSD1306_WHITE);

    oled.setCursor(0, 53);
    oled.print("STATUS: NORMAL");
  }

  oled.display();
}

// ===================== SERIAL OUTPUT =====================
/**
 * @brief Print gate status information to the UART terminal every second.
 * @return void
 */
void showSerial() {
  if (millis() - tSerial < 1000) {
    return;
  }

  tSerial = millis();

  Serial.print("[DATA] MODE: ");

  if (state == IDLE ||
      state == OPENING ||
      state == HOLD ||
      state == CLOSING) {

    Serial.print("AUTO");

  } else if (state == MANUAL) {

    Serial.print("MANUAL");

  } else {

    Serial.print("SAFETY");
  }

  Serial.print(" | GATE: ");

  if (angle <= 0)
    Serial.print("CLOSED");
  else if (angle >= 90)
    Serial.print("OPEN");
  else
    Serial.print("MOVING");

  Serial.print(" | DIST: ");

  if (dist >= 999) {
    Serial.print("CLEAR");
  } else {
    Serial.print(dist);
    Serial.print("cm");
  }

  Serial.print(" | HOLD: ");
  Serial.print(holdTime / 1000);
  Serial.println("s");
}

// ===================== LED =====================
/**
 * @brief Control the status LED.
 * @details LED is continuously ON during normal operation and flashes
 *          rapidly during a safety halt.
 * @return void
 */
void updateLED() {
  unsigned long now = millis();

  if (state == SAFETY) {

    if (now - tLed >= 80) {
      tLed = now;

      ledOn = !ledOn;
      digitalWrite(PIN_LED, ledOn);
    }

  } else {

    digitalWrite(PIN_LED, HIGH);
  }
}

// ===================== SERIAL INPUT =====================
/**
 * @brief Handle UART commands for manual, automatic, reset and help functions.
 * @return void
 */
void handleSerial() {

  if (Serial.available()) {

    char c = Serial.read();

    switch (c) {
      
      case 'm':
      case 'M':
        if (state != SAFETY) {
          Serial.println("[UART] Manual Mode");
          state = MANUAL;
          tState = millis();
          manOpen = (angle >= GATE_OPEN);
        } else {
          Serial.println("[UART] SAFETY latched - clear obstacle and reset");
        }
        break;

      case 'a':
      case 'A':
        if (state != SAFETY) {
          Serial.println("[UART] Auto Mode");
          state = IDLE;
        } else {
          Serial.println("[UART] SAFETY latched - clear obstacle and reset");
        }
        break;

      case 'r':
      case 'R':
        Serial.println("[UART] Reset command");

        if (state == SAFETY) {

          dist = getDistance();

          if (dist >= SAFE_DIST) {

            Serial.println("[RESET] Path clear - Re-opening");

            timerBuzzActive = false;

            state = OPENING;
            tState = millis();

          } else {

            Serial.print("[RESET] BLOCKED at ");
            Serial.print(dist);
            Serial.println(" cm");
          }

        } else {

          Serial.println("[UART] Not in Safety Halt");
        }
        break;

      case 'h':
      case 'H':
        Serial.println("--- COMMANDS ---");
        Serial.println("m = Manual");
        Serial.println("a = Auto");
        Serial.println("r = Reset");
        Serial.println("h = Help");
        break;
    }
  }
}