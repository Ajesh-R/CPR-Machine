/*
 * CPR Machine with Motor Speed Control, Hall Effect Quadrature Encoder Feedback, LCD Display, and Oxygen Cylinder Relay
 * 
 * Components:
 * - Arduino board
 * - Motor driver module
 * - Hall effect quadrature encoder (OE-37)
 * - I2C LCD display
 * - Relay for oxygen cylinder
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Motor pin
const int motorPin = 9;

// Encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Relay pin for oxygen cylinder
const int oxygenRelayPin = 7;

// LCD display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// PID constants
double kp = 2, ki = 5, kd = 1;
double input = 0, output = 0, setpoint = 110;
double errSum = 0, lastErr = 0;

// CPR cycle counters
int compressionCount = 0;
int cycleCount = 0;

// Quadrature encoder variables
volatile long encoderPos = 0;
volatile bool newRotation = false;

void setup() {
  // Initialize motor pin
  pinMode(motorPin, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Initialize oxygen relay pin
  pinMode(oxygenRelayPin, OUTPUT);
  digitalWrite(oxygenRelayPin, LOW);

  // Initialize LCD display
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("CPR Machine");

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Update LCD display
  lcd.setCursor(0, 1);
  lcd.print("Comp: ");
  lcd.print(compressionCount);
  lcd.print(" Cycle: ");
  lcd.print(cycleCount);

  // Check for new rotation
  if (newRotation) {
    compressionCount++;
    newRotation = false;
  }

  // Perform CPR cycle
  if (compressionCount >= 30) {
    lcd.setCursor(14, 1);
    lcd.print("AIR");
    // Ventilation phase
    digitalWrite(oxygenRelayPin, HIGH); // Open oxygen cylinder valve
    delay(2000); // Ventilate for 2 seconds
    digitalWrite(oxygenRelayPin, LOW); // Close oxygen cylinder valve
    compressionCount = 0;
    cycleCount++;
    lcd.clear();
  }

  // Wait for a short period
  delay(50);
}

void updateEncoder() {
  static uint8_t prevStateA, prevStateB;
  uint8_t currStateA = digitalRead(encoderPinA);
  uint8_t currStateB = digitalRead(encoderPinB);

  if (currStateA != prevStateA || currStateB != prevStateB) {
    if (currStateA == currStateB) {
      encoderPos++;
    } else {
      encoderPos--;
    }

    if (encoderPos % 7 == 0) {
      newRotation = true;
    }

    prevStateA = currStateA;
    prevStateB = currStateB;
  }
}
