#include <Arduino.h>

// ======================================================
// Adafruit DRV8871 + Encoder Example
// Arduino Mega
// ======================================================

#define MOTOR_IN1 9
#define MOTOR_IN2 10

#define ENCODER_A 2
#define ENCODER_B 3

volatile long encoderTicks = 0;

// Adjust as needed
const int TICKS_PER_REV = 20;

unsigned long lastRPMTime = 0;
long lastTicks = 0;

void runForOneSecond();
void rampUP();
void rampDown();

// ======================================================
// Encoder interrupt
// ======================================================
void encoderISR() {
  encoderTicks++;
}

// ======================================================
void setup() {

  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    encoderISR,
    RISING
  );

  Serial.println("DRV8871 Encoder Test");
}

// ======================================================
void loop() {

  // -------------------------
  // Forward
  // -------------------------
  digitalWrite(MOTOR_IN1, LOW);
  analogWrite(MOTOR_IN2, 200);

  runForOneSecond();

  // Stop briefly
  analogWrite(MOTOR_IN2, 0);
  delay(250);

  // -------------------------
  // Backward
  // -------------------------
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_IN1, 200);

  runForOneSecond();

  // Stop briefly
  analogWrite(MOTOR_IN1, 0);
  delay(250);
}

// ======================================================
void runForOneSecond() {

  unsigned long startTime = millis();

  while (millis() - startTime < 1000) {

    if (millis() - lastRPMTime >= 200) {

      long currentTicks = encoderTicks;
      long deltaTicks = currentTicks - lastTicks;

      float revs = (float)deltaTicks / TICKS_PER_REV;

      float rpm = revs * 60.0 / 0.2;

      Serial.print("RPM: ");
      Serial.println(rpm);

      lastTicks = currentTicks;
      lastRPMTime = millis();
    }
  }
}

// ======================================================
