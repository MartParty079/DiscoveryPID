/*
  Legacy two-channel PI controller example.

  Preserved from GPT2.cpp for reference only. This version treats the DRV8871
  like a direction-plus-PWM driver, so it should not be used as the active
  hardware implementation. The corrected DRV8871 control logic is in
  src/main.cpp.
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

float targetRPM = 0.0;
float measuredRPM = 0.0;
float integralTerm = 0.0;

unsigned long lastControlTime = 0;

void updateEncoder()
{
    bool A = digitalRead(HALL_A_PIN);
    bool B = digitalRead(HALL_B_PIN);

    if (A == B)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}

void setMotorOutput(float controlSignal)
{
    controlSignal = constrain(controlSignal, -255, 255);

    if (controlSignal >= 0)
    {
        digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIR_PIN, LOW);
    }

    int pwmValue = abs(static_cast<int>(controlSignal));
    analogWrite(PWM_PIN, pwmValue);
}

void setup()
{
    Serial.begin(115200);

    pinMode(HALL_A_PIN, INPUT_PULLUP);
    pinMode(HALL_B_PIN, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), updateEncoder, CHANGE);

    Serial.println("Legacy PI Motor Speed Controller");
    Serial.println("Send target RPM via Serial");
}

void loop()
{
    if (Serial.available())
    {
        targetRPM = Serial.parseFloat();
        Serial.print("New Target RPM: ");
        Serial.println(targetRPM);
    }

    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL_MS)
    {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        noInterrupts();
        long currentCount = encoderCount;
        interrupts();

        long deltaCounts = currentCount - previousEncoderCount;
        previousEncoderCount = currentCount;

        float revs = static_cast<float>(deltaCounts) / COUNTS_PER_REV;
        measuredRPM = (revs / dt) * 60.0f;

        float error = targetRPM - measuredRPM;
        integralTerm += error * dt;
        integralTerm = constrain(integralTerm, -300.0f, 300.0f);

        float controlSignal = (KP * error) + (KI * integralTerm);
        setMotorOutput(controlSignal);

        Serial.print("Target: ");
        Serial.print(targetRPM);
        Serial.print(" RPM | Measured: ");
        Serial.print(measuredRPM);
        Serial.print(" RPM | PWM: ");
        Serial.println(controlSignal);
    }
}
