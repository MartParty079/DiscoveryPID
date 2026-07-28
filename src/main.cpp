// DiscoveryPID JMP autotuning firmware
// Arduino Mega 2560 + Adafruit 4416 geared motor + DRV8871 driver.

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

const char FIRMWARE_VERSION[] = "DiscoveryPID jmp-autotune-v1 2026-07-28";

// =====================================================
// HARDWARE
// =====================================================
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;
const uint8_t MOTOR_IN1_PIN = 9;
const uint8_t MOTOR_IN2_PIN = 10;

// =====================================================
// MOTOR / ENCODER CALIBRATION
// =====================================================
const float GEAR_RATIO = 30613.0f / 1500.0f;
const int COUNTS_PER_MOTOR_REV = 28;
const int8_t ENCODER_DIRECTION_SIGN = 1;

// =====================================================
// CONTROLLER SETTINGS
// =====================================================
// These are runtime variables on the autotuning branch. They can be changed
// over USB serial without rebuilding or uploading the firmware again.
float controllerP = 2.0f;
float controllerI = 5.0f;

const float MIN_P_GAIN = 0.0f;
const float MAX_P_GAIN = 10.0f;
const float MIN_I_GAIN = 0.0f;
const float MAX_I_GAIN = 50.0f;

const uint32_t CONTROL_INTERVAL_MS = 10;
const uint8_t PRINT_DIVIDER = 10; // 10 ms control x 10 = about 10 rows/second.
const float INTEGRAL_LIMIT = 300.0f;
const bool PREVENT_ACTIVE_DIRECTION_REVERSAL = true;

// =====================================================
// ENCODER STATE
// =====================================================
volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// =====================================================
// CONTROLLER STATE
// =====================================================
float targetOutputRPM = 0.0f;
float measuredOutputRPM = 0.0f;
float integralError = 0.0f;
unsigned long trialNumber = 0;

long previousEncoderCount = 0;
uint32_t previousControlTime = 0;

// =====================================================
// ENCODER INTERRUPT
// =====================================================
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
// Forward: IN1 LOW, PWM on IN2
// Reverse: IN2 LOW, PWM on IN1
// Stop:    both LOW
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
// SERIAL COMMANDS
// =====================================================
bool parseNumber(const char *text, float &value)
{
    char *parseEnd = nullptr;
    const float parsedValue = static_cast<float>(strtod(text, &parseEnd));

    if (parseEnd == text)
    {
        return false;
    }

    while (*parseEnd == ' ' || *parseEnd == '\t')
    {
        ++parseEnd;
    }

    if (*parseEnd != '\0')
    {
        return false;
    }

    value = parsedValue;
    return true;
}

void setTargetRPM(float newTargetRPM)
{
    targetOutputRPM = newTargetRPM;
    integralError = 0.0f;

    Serial.print("ACK RPM=");
    Serial.println(targetOutputRPM, 3);
}

void printControllerStatus()
{
    Serial.print("STATUS Trial=");
    Serial.print(trialNumber);
    Serial.print(" P=");
    Serial.print(controllerP, 5);
    Serial.print(" I=");
    Serial.print(controllerI, 5);
    Serial.print(" Target=");
    Serial.println(targetOutputRPM, 3);
}

void processSerialCommand(char *command)
{
    if (strcmp(command, "STOP") == 0)
    {
        setTargetRPM(0.0f);
        setMotor(0.0f);
        Serial.println("ACK STOP");
        return;
    }

    if (strcmp(command, "STATUS") == 0)
    {
        printControllerStatus();
        return;
    }

    char *equalsSign = strchr(command, '=');
    if (equalsSign != nullptr)
    {
        *equalsSign = '\0';
        const char *key = command;
        const char *valueText = equalsSign + 1;

        float value = 0.0f;
        if (!parseNumber(valueText, value))
        {
            Serial.println("ERR INVALID_NUMBER");
            return;
        }

        if (strcmp(key, "P") == 0 || strcmp(key, "KP") == 0)
        {
            if (value < MIN_P_GAIN || value > MAX_P_GAIN)
            {
                Serial.println("ERR P_OUT_OF_RANGE");
                return;
            }

            controllerP = value;
            integralError = 0.0f;
            Serial.print("ACK P=");
            Serial.println(controllerP, 5);
            return;
        }

        if (strcmp(key, "I") == 0 || strcmp(key, "KI") == 0)
        {
            if (value < MIN_I_GAIN || value > MAX_I_GAIN)
            {
                Serial.println("ERR I_OUT_OF_RANGE");
                return;
            }

            controllerI = value;
            integralError = 0.0f;
            Serial.print("ACK I=");
            Serial.println(controllerI, 5);
            return;
        }

        if (strcmp(key, "RPM") == 0 || strcmp(key, "TARGET") == 0)
        {
            setTargetRPM(value);
            return;
        }

        if (strcmp(key, "TRIAL") == 0)
        {
            if (value < 0.0f)
            {
                Serial.println("ERR TRIAL_OUT_OF_RANGE");
                return;
            }

            trialNumber = static_cast<unsigned long>(value);
            Serial.print("ACK TRIAL=");
            Serial.println(trialNumber);
            return;
        }

        Serial.println("ERR UNKNOWN_COMMAND");
        return;
    }

    // A plain number remains a valid target command for backward compatibility.
    float numericTarget = 0.0f;
    if (parseNumber(command, numericTarget))
    {
        setTargetRPM(numericTarget);
        return;
    }

    Serial.println("ERR UNKNOWN_COMMAND");
}

void readSerialCommands()
{
    static char inputBuffer[48];
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

            if (inputIndex > 0)
            {
                processSerialCommand(inputBuffer);
            }

            inputIndex = 0;
            continue;
        }

        if (inputIndex < sizeof(inputBuffer) - 1)
        {
            inputBuffer[inputIndex++] = incoming;
        }
        else
        {
            inputIndex = 0;
            Serial.println("ERR COMMAND_TOO_LONG");
        }
    }
}

// =====================================================
// STARTUP
// =====================================================
void setup()
{
    Serial.begin(115200);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    setMotor(0.0f);

    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

    previousControlTime = millis();

    Serial.println("DiscoveryPID ready");
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.println("Direction reversal guard: ON");
    Serial.println("Commands: P=<gain>, I=<gain>, RPM=<target>, TRIAL=<n>, STOP, STATUS");
    printControllerStatus();
}

// =====================================================
// MAIN CONTROL LOOP
// =====================================================
void loop()
{
    readSerialCommands();

    const uint32_t now = millis();
    if (now - previousControlTime < CONTROL_INTERVAL_MS)
    {
        return;
    }

    const float dt =
        static_cast<float>(now - previousControlTime) / 1000.0f;
    previousControlTime = now;

    noInterrupts();
    const long currentCount = encoderCount;
    interrupts();

    const long deltaCount = currentCount - previousEncoderCount;
    previousEncoderCount = currentCount;

    const float motorRPM =
        ENCODER_DIRECTION_SIGN *
        (static_cast<float>(deltaCount) / COUNTS_PER_MOTOR_REV) *
        (60.0f / dt);

    measuredOutputRPM = motorRPM / GEAR_RATIO;
    const float error = targetOutputRPM - measuredOutputRPM;

    const float proposedIntegral = constrain(
        integralError + error * dt,
        -INTEGRAL_LIMIT,
        INTEGRAL_LIMIT
    );

    const float proposedCommand =
        controllerP * error + controllerI * proposedIntegral;

    const bool withinPwmRange =
        proposedCommand < 255.0f && proposedCommand > -255.0f;

    bool directionAllowed = true;
    if (PREVENT_ACTIVE_DIRECTION_REVERSAL)
    {
        if (targetOutputRPM > 0.0f && proposedCommand < 0.0f)
        {
            directionAllowed = false;
        }
        else if (targetOutputRPM < 0.0f && proposedCommand > 0.0f)
        {
            directionAllowed = false;
        }
    }

    if (withinPwmRange && directionAllowed)
    {
        integralError = proposedIntegral;
    }

    float motorCommand =
        controllerP * error + controllerI * integralError;

    if (fabs(targetOutputRPM) < 0.01f)
    {
        motorCommand = 0.0f;
        integralError = 0.0f;
    }
    else if (PREVENT_ACTIVE_DIRECTION_REVERSAL)
    {
        if (targetOutputRPM > 0.0f)
        {
            motorCommand = constrain(motorCommand, 0.0f, 255.0f);
        }
        else
        {
            motorCommand = constrain(motorCommand, -255.0f, 0.0f);
        }
    }
    else
    {
        motorCommand = constrain(motorCommand, -255.0f, 255.0f);
    }

    setMotor(motorCommand);

    static uint8_t printDivider = 0;
    if (++printDivider >= PRINT_DIVIDER)
    {
        printDivider = 0;

        Serial.print("Trial=");
        Serial.print(trialNumber);
        Serial.print(" P=");
        Serial.print(controllerP, 5);
        Serial.print(" I=");
        Serial.print(controllerI, 5);
        Serial.print(" Target=");
        Serial.print(targetOutputRPM, 2);
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
