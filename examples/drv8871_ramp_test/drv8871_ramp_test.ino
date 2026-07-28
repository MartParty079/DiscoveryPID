/*
  Basic Adafruit DRV8871 forward/reverse ramp test.

  Preserved from example.cpp. This is a standalone hardware example and is not
  compiled as part of the active PlatformIO firmware in src/main.cpp.
*/

#include <Arduino.h>

#define MOTOR_IN1 9
#define MOTOR_IN2 10

void setup()
{
    Serial.begin(115200);
    Serial.println("DRV8871 ramp test");

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
}

void loop()
{
    // Ramp forward: IN1 low, PWM on IN2.
    digitalWrite(MOTOR_IN1, LOW);
    for (int pwm = 0; pwm < 255; ++pwm)
    {
        analogWrite(MOTOR_IN2, pwm);
        delay(10);
    }

    delay(1000);

    for (int pwm = 255; pwm >= 0; --pwm)
    {
        analogWrite(MOTOR_IN2, pwm);
        delay(10);
    }

    // Ramp backward: IN2 low, PWM on IN1.
    digitalWrite(MOTOR_IN2, LOW);
    for (int pwm = 0; pwm < 255; ++pwm)
    {
        analogWrite(MOTOR_IN1, pwm);
        delay(10);
    }

    delay(1000);

    for (int pwm = 255; pwm >= 0; --pwm)
    {
        analogWrite(MOTOR_IN1, pwm);
        delay(10);
    }
}
