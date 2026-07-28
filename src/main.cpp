/*
  DiscoveryPID - Adafruit 4416 PI Motor Speed Controller
  ======================================================

  ACTIVE HARDWARE
  ---------------
  Microcontroller:
    Arduino Mega 2560 / ATmega2560

  Motor:
    Adafruit Product 4416
    Geared DC motor with two magnetic Hall encoder outputs
    Nominal motor voltage: 7 VDC
    Approximate gearbox ratio: 20.4:1

  Motor driver:
    Adafruit Product 3190
    DRV8871 brushed DC motor driver breakout

  WIRING
  ------
  Adafruit 4416 encoder cable:
    Black  -> Arduino GND
    Blue   -> Arduino 5 V
    White  -> Arduino Mega digital pin 2 (encoder channel A)
    Yellow -> Arduino Mega digital pin 3 (encoder channel B)

  Adafruit 4416 motor wires:
    Red and green -> DRV8871 OUT1 and OUT2 terminals
    Swapping these two wires reverses the physical motor direction.

  DRV8871 control and power:
    IN1 -> Arduino Mega PWM pin 9
    IN2 -> Arduino Mega PWM pin 10
    VM  -> External motor supply near the motor's nominal 7 V
    GND -> External supply ground and Arduino GND

  Do not connect an external motor-supply voltage to the Arduino 5 V pin.
  The Arduino and DRV8871 must share ground so the input logic has a common
  voltage reference.

  PROGRAM OPERATION
  -----------------
    1. Decode both encoder channels using CHANGE interrupts.
    2. Convert encoder counts into motor-shaft RPM.
    3. Divide motor RPM by the gearbox ratio to obtain output-shaft RPM.
    4. Compare output RPM with the requested target RPM.
    5. Calculate a PI controller command with conditional anti-windup.
    6. Drive the DRV8871 using PWM on IN1 or IN2, depending on direction.
    7. Print test data for controller tuning and JMP Bayesian optimization.

  SERIAL COMMAND
  --------------
  Open the serial monitor at 115200 baud and send a numeric output-shaft RPM.
  Example:
    100

  The startup target is 0 RPM so the motor does not begin moving immediately
  after power-up or firmware upload.
*/

#include <Arduino.h>

// =====================================================
// ARDUINO MEGA 2560 PIN ASSIGNMENTS
// =====================================================
// Pins 2 and 3 support external interrupts on the Mega 2560.
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;

// Both DRV8871 inputs are connected to PWM-capable Mega pins.
// The DRV8871 is not controlled as a separate DIR + PWM device. For forward
// motion one input is held LOW while the other receives PWM; reverse swaps them.
const uint8_t MOTOR_IN1_PIN = 9;
const uint8_t MOTOR_IN2_PIN = 10;

// =====================================================
// ADAFRUIT 4416 ENCODER AND GEARBOX CALIBRATION
// =====================================================
// Adafruit lists the exact gearbox ratio as 30613:1500, approximately 20.4087:1.
// Using the exact published ratio reduces systematic output-RPM error.
const float GEAR_RATIO = 30613.0f / 1500.0f;

// Each encoder channel produces approximately 7 cycles per motor revolution.
// Full x4 quadrature decoding counts both edges on both channels:
//   7 cycles/channel x 4 edge transitions = 28 counts/motor revolution.
const int COUNTS_PER_MOTOR_REV = 28;

// If measured RPM has the opposite sign from the commanded direction, change
// this value from +1 to -1 instead of rewiring the encoder channels.
const int8_t ENCODER_DIRECTION_SIGN = 1;

// =====================================================
// PI CONTROLLER PARAMETERS
// =====================================================
// KP controls the immediate correction based on current speed error.
// KI controls correction based on accumulated speed error over time.
// These initial values will later be optimized experimentally using JMP.
const float KP = 2.0f;
const float KI = 5.0f;

// Controller sample interval. 20 ms corresponds to a nominal 50 Hz update rate.
const uint32_t CONTROL_INTERVAL_MS = 20;

// Limit applied to the stored integral state as an additional safety boundary.
// Conditional anti-windup below already prevents integration during saturation.
const float INTEGRAL_LIMIT = 300.0f;

// =====================================================
// ENCODER STATE SHARED WITH INTERRUPTS
// =====================================================
// volatile is required because these variables are modified inside an ISR.
volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

// Quadrature transition lookup table.
// The table receives a four-bit value containing previous AB and current AB.
// It returns +1, -1, or 0 for each valid, reverse, or invalid transition.
const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// =====================================================
// CONTROL AND MEASUREMENT STATE
// =====================================================
// Start stopped. A target must be entered through the serial monitor.
float targetOutputRPM = 0.0f;

float measuredOutputRPM = 0.0f;
float integralError = 0.0f;

long previousEncoderCount = 0;
uint32_t previousControlTime = 0;

// =====================================================
// QUADRATURE ENCODER INTERRUPT SERVICE ROUTINE
// =====================================================
// This function runs whenever either encoder channel changes. Keep ISR work
// minimal because normal program execution pauses while the ISR is running.
void encoderISR()
{
    // Combine channels A and B into one two-bit state: AB.
    const uint8_t currentState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

    // Combine the previous and current states into a lookup-table index.
    const uint8_t transitionIndex =
        (previousEncoderState << 2) | currentState;

    encoderCount += QUADRATURE_TABLE[transitionIndex];
    previousEncoderState = currentState;
}

// =====================================================
// DRV8871 MOTOR OUTPUT
// =====================================================
// The DRV8871 has two H-bridge control inputs, IN1 and IN2:
//   Forward:  IN1 = LOW, IN2 = PWM
//   Reverse:  IN2 = LOW, IN1 = PWM
//   Stop:     IN1 = LOW, IN2 = LOW
//
// The sign of command selects direction. Its magnitude selects PWM duty cycle.
void setMotorOutput(float command)
{
    const int pwm = constrain(static_cast<int>(fabs(command)), 0, 255);

    if (pwm == 0)
    {
        // Coast/stop with both H-bridge inputs low.
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, 0);
        return;
    }

    if (command > 0.0f)
    {
        // Forward: force IN1 low before applying PWM to IN2.
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, pwm);
    }
    else
    {
        // Reverse: force IN2 low before applying PWM to IN1.
        analogWrite(MOTOR_IN2_PIN, 0);
        analogWrite(MOTOR_IN1_PIN, pwm);
    }
}

// =====================================================
// ARDUINO STARTUP
// =====================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(25);

    // The Hall outputs are read with internal pull-ups, matching Adafruit's
    // encoder examples and providing defined logic levels at startup.
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);

    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    // Ensure the motor driver starts with both inputs low.
    setMotorOutput(0.0f);

    // Capture the real initial AB state before enabling interrupts.
    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

    // Full x4 decoding: run the same ISR on every edge of both channels.
    attachInterrupt(
        digitalPinToInterrupt(ENC_A_PIN),
        encoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(ENC_B_PIN),
        encoderISR,
        CHANGE
    );

    previousControlTime = millis();

    Serial.println("DiscoveryPID ready - Arduino Mega 2560");
    Serial.println("Hardware: Adafruit 4416 motor + DRV8871 driver");
    Serial.println("Send target output RPM as a number");
}

// =====================================================
// CONTINUOUS CONTROL LOOP
// =====================================================
void loop()
{
    // -------------------------------------------------
    // READ A NEW OUTPUT-SHAFT RPM TARGET
    // -------------------------------------------------
    if (Serial.available() > 0)
    {
        targetOutputRPM = Serial.parseFloat();

        // Reset accumulated error whenever a new test target is entered. This
        // keeps one experiment's integral state from contaminating the next.
        integralError = 0.0f;

        Serial.print("New target output RPM: ");
        Serial.println(targetOutputRPM);
    }

    // -------------------------------------------------
    // RUN THE TIMED PI CONTROL UPDATE
    // -------------------------------------------------
    const uint32_t now = millis();

    if (now - previousControlTime >= CONTROL_INTERVAL_MS)
    {
        // Use actual elapsed time to reduce error caused by loop-time variation.
        const float dt =
            static_cast<float>(now - previousControlTime) / 1000.0f;

        previousControlTime = now;

        // ---------------------------------------------
        // COPY ENCODER COUNT ATOMICALLY
        // ---------------------------------------------
        // long is wider than the ATmega2560's 8-bit data bus, so interrupts are
        // briefly disabled to prevent reading a partially updated value.
        noInterrupts();
        const long currentEncoderCount = encoderCount;
        interrupts();

        const long deltaCount =
            currentEncoderCount - previousEncoderCount;

        previousEncoderCount = currentEncoderCount;

        // ---------------------------------------------
        // CALCULATE MOTOR AND OUTPUT RPM
        // ---------------------------------------------
        const float motorRPM =
            ENCODER_DIRECTION_SIGN *
            (static_cast<float>(deltaCount) / COUNTS_PER_MOTOR_REV) *
            (60.0f / dt);

        measuredOutputRPM = motorRPM / GEAR_RATIO;

        // ---------------------------------------------
        // CALCULATE PI CONTROLLER OUTPUT
        // ---------------------------------------------
        const float error = targetOutputRPM - measuredOutputRPM;

        // Calculate a candidate integral state first. It is accepted only when
        // the corresponding motor command is inside the available PWM range.
        const float proposedIntegral = constrain(
            integralError + (error * dt),
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        );

        const float proposedCommand =
            (KP * error) +
            (KI * proposedIntegral);

        // Conditional anti-windup prevents accumulating additional error while
        // the motor command is saturated at its maximum forward or reverse PWM.
        if (proposedCommand < 255.0f && proposedCommand > -255.0f)
        {
            integralError = proposedIntegral;
        }

        float motorCommand =
            (KP * error) +
            (KI * integralError);

        motorCommand = constrain(motorCommand, -255.0f, 255.0f);

        setMotorOutput(motorCommand);

        // ---------------------------------------------
        // PRINT DATA FOR TESTING AND JMP ANALYSIS
        // ---------------------------------------------
        // Print every fifth 50 Hz update, giving approximately 10 samples/sec.
        static uint8_t printDivider = 0;

        if (++printDivider >= 5)
        {
            printDivider = 0;

            Serial.print("Target=");
            Serial.print(targetOutputRPM);

            Serial.print(" OutRPM=");
            Serial.print(measuredOutputRPM, 2);

            Serial.print(" MotorRPM=");
            Serial.print(motorRPM, 1);

            Serial.print(" Error=");
            Serial.print(error, 2);

            Serial.print(" Integral=");
            Serial.print(integralError, 2);

            Serial.print(" Counts=");
            Serial.print(currentEncoderCount);

            Serial.print(" dC=");
            Serial.print(deltaCount);

            Serial.print(" PWM=");
            Serial.println(motorCommand, 1);
        }
    }
}
