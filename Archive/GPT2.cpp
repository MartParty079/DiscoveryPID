/*
  DC Motor Speed Controller with PI Feedback
  ------------------------------------------
  - Reads a quadrature encoder using 2 hall effect sensors
  - Controls motor speed with a PI controller
  - Outputs PWM to a motor driver

  Hardware Assumptions:
  ---------------------
  Encoder Hall Sensors:
    HALL_A -> pin 2  (interrupt pin)
    HALL_B -> pin 3  (interrupt pin)

  Motor Driver:
    PWM_PIN -> PWM speed pin
    DIR_PIN -> direction pin

  Serial Commands:
    Send desired RPM through Serial Monitor
    Example:
      150
      300
      -200

  Notes:
  ------
  - Tune KP and KI for your system
  - Works with common drivers like DRV8871, BTS7960, L298N, etc.
*/

#include <Arduino.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================

const int HALL_A_PIN = 2; // Green wire
const int HALL_B_PIN = 3; // White wire

const int PWM_PIN = 10; // IN2
const int DIR_PIN = 9; // IN1

//const int Motor_IN2 = 10; // IN2
//const int Motor_IN1 = 9; // IN1


// =====================================================
// ENCODER SETTINGS
// =====================================================

// Pulses per revolution from ONE channel
// Adjust for your encoder
const int PULSES_PER_REV = 20;

// Quadrature gives 4x counts
const int COUNTS_PER_REV = PULSES_PER_REV * 4;

// =====================================================
// PI CONTROLLER SETTINGS
// =====================================================

float KP = 1.2;
float KI = 0.8;

// =====================================================
// CONTROL LOOP SETTINGS
// =====================================================

const unsigned long CONTROL_INTERVAL_MS = 20;

// =====================================================
// GLOBAL VARIABLES
// =====================================================

volatile long encoderCount = 0;

long previousEncoderCount = 0;

float targetRPM = 0.0;
float measuredRPM = 0.0;

float integralTerm = 0.0;

unsigned long lastControlTime = 0;

// =====================================================
// QUADRATURE ENCODER ISR
// =====================================================

void updateEncoder()
{
    bool A = digitalRead(HALL_A_PIN);
    bool B = digitalRead(HALL_B_PIN);

    // Quadrature decoding
    if (A == B)
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }
}

// =====================================================
// MOTOR OUTPUT
// =====================================================

void setMotorOutput(float controlSignal)
{
    // Clamp output
    controlSignal = constrain(controlSignal, -255, 255);

    // Direction
    if (controlSignal >= 0)
    {
        digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIR_PIN, LOW);
    }

    // PWM magnitude
    int pwmValue = abs((int)controlSignal);

    analogWrite(PWM_PIN, pwmValue);
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
    Serial.begin(115200);

    pinMode(HALL_A_PIN, INPUT_PULLUP);
    pinMode(HALL_B_PIN, INPUT_PULLUP);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    //pinMode(Motor_IN2, OUTPUT);
    //pinMode(Motor_IN1, OUTPUT);


    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), updateEncoder, CHANGE);

    Serial.println("PI Motor Speed Controller");
    Serial.println("Send target RPM via Serial");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
    // ---------------------------------------------
    // Read target RPM from serial
    // ---------------------------------------------
    if (Serial.available())
    {
        targetRPM = Serial.parseFloat();

        Serial.print("New Target RPM: ");
        Serial.println(targetRPM);
    }

    // ---------------------------------------------
    // Run control loop
    // ---------------------------------------------
    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL_MS)
    {
        float dt = (now - lastControlTime) / 1000.0;

        lastControlTime = now;

        // -----------------------------------------
        // Measure RPM
        // -----------------------------------------

        noInterrupts();
        long currentCount = encoderCount;
        interrupts();

        long deltaCounts = currentCount - previousEncoderCount;

        previousEncoderCount = currentCount;

        float revs = (float)deltaCounts / COUNTS_PER_REV;

        measuredRPM = (revs / dt) * 60.0;

        // -----------------------------------------
        // PI Controller
        // -----------------------------------------

        float error = targetRPM - measuredRPM;

        // Integral accumulation
        integralTerm += error * dt;

        // Anti-windup clamp
        integralTerm = constrain(integralTerm, -300, 300);

        // PI equation
        float controlSignal =
            (KP * error) +
            (KI * integralTerm);

        // -----------------------------------------
        // Output to motor
        // -----------------------------------------

        setMotorOutput(controlSignal);

        // -----------------------------------------
        // Debug Output
        // -----------------------------------------

        Serial.print("Target: ");
        Serial.print(targetRPM);

        Serial.print(" RPM | Measured: ");
        Serial.print(measuredRPM);

        Serial.print(" RPM | PWM: ");
        Serial.println(controlSignal);
    }
}