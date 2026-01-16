#include <Arduino.h>

// ==========================================
// ESP32 Speedometer - CALIBRATED VERSION
// Based on Real-World Data (79 km/h test)
// ==========================================

// --- CALIBRATION ---
// We adjusted these to match your ratio of ~4.38
const float WHEEL_DIAMETER_INCHES = 13.2; // Adjusted for tire expansion/reality
const int MOTOR_POLE_PAIRS = 6;           // Corrected from 24 -> 6 based on data

// --- CONFIGURATION ---
const unsigned long DEBOUNCE_TIME_US = 0; // State machine handles noise, no delay needed
const int HALL_PIN_1 = 14; 
const int HALL_PIN_2 = 27; 
const int HALL_PIN_3 = 26; 

volatile long transitionCount = 0;
volatile int lastEncodedPattern = 0;
unsigned long lastCalculationTime = 0;
const int CALCULATION_INTERVAL = 1000;

// Valid Sequence Table (Standard 120 degree motor)
int8_t validSequence[8][2] = {
  {0, 0}, {3, 5}, {6, 3}, {2, 1}, {5, 6}, {1, 4}, {4, 2}, {0, 0}
};

void IRAM_ATTR handleHallChange() {
  int h1 = digitalRead(HALL_PIN_1);
  int h2 = digitalRead(HALL_PIN_2);
  int h3 = digitalRead(HALL_PIN_3);

  int currentPattern = (h1 << 2) | (h2 << 1) | h3;

  if (currentPattern == 0 || currentPattern == 7) return;

  // Only count valid rotation steps
  if (currentPattern == validSequence[lastEncodedPattern][0] || 
      currentPattern == validSequence[lastEncodedPattern][1]) {
      transitionCount++;
  }
  lastEncodedPattern = currentPattern;
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN_1, INPUT_PULLUP);
  pinMode(HALL_PIN_2, INPUT_PULLUP);
  pinMode(HALL_PIN_3, INPUT_PULLUP);

  // Initialize State
  int h1 = digitalRead(HALL_PIN_1);
  int h2 = digitalRead(HALL_PIN_2);
  int h3 = digitalRead(HALL_PIN_3);
  lastEncodedPattern = (h1 << 2) | (h2 << 1) | h3;

  attachInterrupt(digitalPinToInterrupt(HALL_PIN_1), handleHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_2), handleHallChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_3), handleHallChange, CHANGE);

  Serial.println("ESP32 Speedometer Calibrated");
}

void loop() {
  if (millis() - lastCalculationTime >= CALCULATION_INTERVAL) {
    
    noInterrupts();
    long transitions = transitionCount;
    transitionCount = 0;
    interrupts();
    
    lastCalculationTime = millis();

    // Math:
    // 6 transitions per electrical rev
    // Total steps per wheel rev = PolePairs * 6
    float steps_per_rev = MOTOR_POLE_PAIRS * 6.0;
    
    float revolutions = (float)transitions / steps_per_rev;
    float rpm = revolutions * (60000.0 / CALCULATION_INTERVAL);

    float circumference_m = WHEEL_DIAMETER_INCHES * 0.0254 * 3.14159;
    float speed_mpm = rpm * circumference_m;
    float speed_kmh = (speed_mpm * 60) / 1000;

    Serial.print("Trans: "); Serial.print(transitions);
    Serial.print(" | RPM: "); Serial.print(rpm, 0);
    Serial.print(" | Speed: "); Serial.print(speed_kmh, 1);
    Serial.println(" km/h");
  }
}