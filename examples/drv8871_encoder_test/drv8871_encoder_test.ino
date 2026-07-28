/*
  DRV8871 + encoder hardware test for Arduino Mega 2560.

  This example runs the motor forward and backward at a fixed PWM command and
  prints a basic encoder-derived speed estimate. It is not the active PI
  controller; see src/main.cpp for the project firmware.
*/

#include <Arduino.h>

#define MOTOR_IN1 9
#define MOTOR_IN2 10
#define ENCODER_A 2
#define ENCODER_B 3

volatile long encoderTicks = 0;

// Legacy estimate retained from the original test file.
const int TICKS_PER_REV = 20;

unsigned long lastRPMTime = 0;
long lastTicks = 0;

void runForOneSecond();

void encoderISR()
{
    encoderTicks++;
}

void setup()
{
    Serial.begin(115200);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

    Serial.println("DRV8871 Encoder Test");
}

void loop()
{
    // Forward: hold IN1 low and PWM IN2.
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, 200);
    runForOneSecond();

    analogWrite(MOTOR_IN2, 0);
    delay(250);

    // Backward: hold IN2 low and PWM IN1.
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, 200);
    runForOneSecond();

    analogWrite(MOTOR_IN1, 0);
    delay(250);
}

void runForOneSecond()
{
    unsigned long startTime = millis();

    while (millis() - startTime < 1000)
    {
        if (millis() - lastRPMTime >= 200)
        {
            noInterrupts();
            long currentTicks = encoderTicks;
            interrupts();

            long deltaTicks = currentTicks - lastTicks;
            float revs = static_cast<float>(deltaTicks) / TICKS_PER_REV;
            float rpm = revs * 60.0f / 0.2f;

            Serial.print("RPM: ");
            Serial.println(rpm);

            lastTicks = currentTicks;
            lastRPMTime = millis();
        }
    }
}
