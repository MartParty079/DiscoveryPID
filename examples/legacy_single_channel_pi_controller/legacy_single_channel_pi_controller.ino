/*
  Legacy single-channel PI motor controller.

  This file preserves the previous src/main.cpp implementation as an example.
  It counts CHANGE edges from Hall A only; Hall B quadrature decoding is disabled.
  The active project code is located at src/main.cpp.
*/

#include <Arduino.h>

const int HALL_A_PIN = 2;
const int HALL_B_PIN = 3;
const int PWM_PIN = 10;
const int DIR_PIN = 9;

const int PULSES_PER_REV = 20;
const int COUNTS_PER_REV = PULSES_PER_REV * 4;

float KP = 1.2;
float KI = 0.8;

const unsigned long CONTROL_INTERVAL_MS = 20;

volatile long encoderCount = 0;
long previousEncoderCount = 0;

float targetRPM = 150.0;
float measuredRPM = 0.0;
float integralTerm = 0.0;

unsigned long lastControlTime = 0;

void updateEncoder()
{
    // Legacy implementation: count every Hall A transition as positive.
    encoderCount++;

    // Full quadrature decoding was planned but disabled:
    // bool A = digitalRead(HALL_A_PIN);
    // bool B = digitalRead(HALL_B_PIN);
    // encoderCount += (A == B) ? 1 : -1;
}

void setMotorOutput(float controlSignal)
{
    controlSignal = constrain(controlSignal, -250, 250);

    digitalWrite(DIR_PIN, controlSignal >= 0 ? HIGH : LOW);
    analogWrite(PWM_PIN, abs((int)controlSignal));
}

void setup()
{
    Serial.begin(115200);

    pinMode(HALL_A_PIN, INPUT);
    // pinMode(HALL_B_PIN, INPUT);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), updateEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), updateEncoder, CHANGE);

    Serial.println("Legacy PI Motor Speed Controller");
    Serial.println("Send target RPM via Serial");
}

void loop()
{
    delay(100);

    if (Serial.available())
    {
        targetRPM = Serial.parseFloat();
        Serial.print("New Target RPM: ");
        Serial.println(targetRPM);
    }

    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL_MS)
    {
        float dt = (now - lastControlTime) / 1000.0;
        lastControlTime = now;

        noInterrupts();
        long currentCount = encoderCount;
        interrupts();

        long deltaCounts = currentCount - previousEncoderCount;
        previousEncoderCount = currentCount;

        float revs = (float)deltaCounts / COUNTS_PER_REV;
        measuredRPM = (revs / dt) * 60.0;

        float error = targetRPM - measuredRPM;
        integralTerm += error * dt;
        integralTerm = constrain(integralTerm, -300, 300);

        float controlSignal = (KP * error) + (KI * integralTerm);
        setMotorOutput(controlSignal);

        Serial.print("Target: ");
        Serial.print(targetRPM);
        Serial.print(" RPM | Measured: ");
        Serial.print(measuredRPM);
        Serial.print(" RPM | PWM: ");
        Serial.print(controlSignal);
        Serial.print(" | Count: ");
        Serial.print(currentCount);
        Serial.print(" | Delta: ");
        Serial.println(deltaCounts);
    }
}
