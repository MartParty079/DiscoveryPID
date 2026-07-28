// Adafruit_4416_PI_Controller.ino
// Active DiscoveryPID firmware for Arduino Mega 2560 + Adafruit 4416 motor
// + Adafruit DRV8871 motor-driver breakout.

#include <Arduino.h>
#include <stdlib.h>

// Printed during startup so the uploaded build can be verified in the serial
// monitor before the motor is tested.
const char FIRMWARE_VERSION[] = "DiscoveryPID safety-v2 2026-07-28";

// =====================================================
// HARDWARE PIN ASSIGNMENTS
// =====================================================
// Encoder white/yellow signal wires connect to Mega interrupt pins 2 and 3.
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;

// The DRV8871 has two H-bridge inputs. Both are connected to PWM-capable pins.
// It is not a separate direction-pin + PWM-pin driver.
const uint8_t MOTOR_IN1_PIN = 9;
const uint8_t MOTOR_IN2_PIN = 10;

// =====================================================
// MOTOR AND ENCODER CALIBRATION
// =====================================================
// Adafruit publishes the exact gearbox ratio as 30613:1500 (~20.4087:1).
const float GEAR_RATIO = 30613.0f / 1500.0f;

// Adafruit specifies 14 counts per motor revolution when CHANGE edges from one
// encoder channel are counted. This program uses both channels with full x4
// quadrature decoding, so the expected count is 28 per motor revolution.
const int COUNTS_PER_MOTOR_REV = 28;

// Change this to -1 if positive motor command produces negative measured RPM.
// Do this before PI tuning; a wrong sign can drive the controller to full PWM.
const int8_t ENCODER_DIRECTION_SIGN = 1;

// =====================================================
// PI CONTROLLER SETTINGS
// =====================================================
// These are initial test gains only. They still require safe bench tuning and
// will later be optimized with JMP Bayesian Optimization.
const float KP = 2.0f;
const float KI = 5.0f;

// 20 ms gives a nominal 50 Hz controller update rate.
const uint32_t CONTROL_INTERVAL_MS = 20;

// Additional limit on stored integral error.
const float INTEGRAL_LIMIT = 300.0f;

// =====================================================
// ENCODER STATE
// =====================================================
// volatile is required because these variables are changed inside interrupts.
volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

// Quadrature transition table. Each index combines the previous AB state and
// current AB state. Valid forward/reverse transitions return +1 or -1.
const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// =====================================================
// CONTROLLER STATE
// =====================================================
// Start at zero so the motor cannot begin moving immediately after reset.
float targetOutputRPM = 100.0f;
float measuredOutputRPM = 0.0f;
float integralError = 0.0f;

long previousEncoderCount = 0;
uint32_t previousControlTime = 0;

// =====================================================
// QUADRATURE ENCODER INTERRUPT
// =====================================================
// This ISR runs on every CHANGE edge of both encoder channels. Keep it short.
void encoderISR()
{
    const uint8_t currentState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

    const uint8_t transitionIndex =
        (previousEncoderState << 2) | currentState;

    encoderCount += QUADRATURE_TABLE[transitionIndex];
    previousEncoderState = currentState;
}

// =====================================================
// DRV8871 MOTOR OUTPUT
// =====================================================
// DRV8871 control:
//   Forward: IN1 LOW, PWM on IN2
//   Reverse: IN2 LOW, PWM on IN1
//   Stop:    IN1 LOW, IN2 LOW
void setMotor(float command)
{
    const int pwm = constrain(static_cast<int>(fabs(command)), 0, 255);

    if (pwm == 0)
    {
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, 0);
        return;
    }

    if (command > 0.0f)
    {
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, pwm);
    }
    else
    {
        analogWrite(MOTOR_IN2_PIN, 0);
        analogWrite(MOTOR_IN1_PIN, pwm);
    }
}

// =====================================================
// NON-BLOCKING SERIAL TARGET INPUT
// =====================================================
// A complete numeric command is accepted when Enter sends a newline. This
// avoids Serial.parseFloat() accidentally interpreting a leftover line ending
// as a new zero-RPM command.
bool readTargetRPM(float &newTargetRPM)
{
    static char inputBuffer[24];
    static uint8_t inputIndex = 0;

    while (Serial.available() > 0)
    {
        const char incoming = static_cast<char>(Serial.read());

        if (incoming == '\r')
        {
            continue;
        }

        if (incoming == '\n')
        {
            inputBuffer[inputIndex] = '\0';

            if (inputIndex == 0)
            {
                return false;
            }

            char *parseEnd = nullptr;

            // AVR-LibC in the installed PlatformIO AVR toolchain exposes
            // strtod(), while strtof() is not declared. AVR double and float
            // use the same 32-bit representation, so this cast loses no
            // additional precision on the ATmega2560.
            const float parsedValue =
                static_cast<float>(strtod(inputBuffer, &parseEnd));

            const bool validNumber = (parseEnd != inputBuffer);
            inputIndex = 0;

            if (validNumber)
            {
                newTargetRPM = parsedValue;
                return true;
            }

            return false;
        }

        if (inputIndex < sizeof(inputBuffer) - 1)
        {
            inputBuffer[inputIndex++] = incoming;
        }
        else
        {
            // Discard an overlength command instead of overflowing the buffer.
            inputIndex = 0;
        }
    }

    return false;
}

// =====================================================
// ARDUINO STARTUP
// =====================================================
void setup()
{
    Serial.begin(115200);

    // Internal pull-ups provide defined encoder input states at startup.
    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);

    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    // Force a stopped state before enabling the encoder interrupts.
    setMotor(0.0f);

    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

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

    Serial.println("DiscoveryPID ready");
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.println("Direction reversal guard: ON");
    Serial.println("Send target output RPM and press Enter");
}

// =====================================================
// MAIN CONTROL LOOP
// =====================================================
void loop()
{
    // Read a new setpoint without blocking the controller timing.
    float newTargetRPM = 0.0f;
    if (readTargetRPM(newTargetRPM))
    {
        targetOutputRPM = newTargetRPM;

        // Prevent integral history from one test carrying into the next test.
        integralError = 0.0f;

        Serial.print("New target output RPM: ");
        Serial.println(targetOutputRPM);
    }

    const uint32_t now = millis();

    if (now - previousControlTime >= CONTROL_INTERVAL_MS)
    {
        // Use the actual elapsed time to reduce sensitivity to loop jitter.
        const float dt =
            static_cast<float>(now - previousControlTime) / 1000.0f;

        previousControlTime = now;

        // Copy the multi-byte encoder count atomically on the 8-bit ATmega2560.
        noInterrupts();
        const long currentCount = encoderCount;
        interrupts();

        const long deltaCount = currentCount - previousEncoderCount;
        previousEncoderCount = currentCount;

        // Convert encoder movement to motor RPM, then account for the gearbox.
        const float motorRPM =
            ENCODER_DIRECTION_SIGN *
            (static_cast<float>(deltaCount) / COUNTS_PER_MOTOR_REV) *
            (60.0f / dt);

        measuredOutputRPM = motorRPM / GEAR_RATIO;

        // PI controller error is based on gearbox output-shaft speed.
        const float error = targetOutputRPM - measuredOutputRPM;

        // Propose a new integral value, but accept it only if the resulting
        // command is not saturated. This is conditional integral anti-windup.
        const float proposedIntegral = constrain(
            integralError + error * dt,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT
        );

        const float proposedCommand =
            KP * error + KI * proposedIntegral;

        if (proposedCommand < 255.0f && proposedCommand > -255.0f)
        {
            integralError = proposedIntegral;
        }

        float motorCommand = KP * error + KI * integralError;
        motorCommand = constrain(motorCommand, -255.0f, 255.0f);

        setMotor(motorCommand);

        // Print approximately ten rows per second for testing and JMP import.
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
            Serial.print(currentCount);
            Serial.print(" dC=");
            Serial.print(deltaCount);
            Serial.print(" PWM=");
            Serial.println(motorCommand, 1);
        }
    }
}
