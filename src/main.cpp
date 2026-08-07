// DiscoveryPID position-hold firmware
// Arduino Mega 2560 + Adafruit 4416 geared motor + DRV8871.
//
// Startup:
//   1. Motor is disabled.
//   2. Move the output stick left and right.
//   3. Hold it at the desired upright/home position.
//   4. After sufficient travel, one reversal, and 1.5 s stillness,
//      the current encoder count becomes Home = 0 degrees.
//   5. The controller actively returns the stick to Home after disturbances.
//
// Control architecture:
//   Outer position PID -> target output RPM
//   Inner speed PI     -> DRV8871 PWM
//
// JMP changes KP/KI/KD at runtime and requests a repeatable TEST=<degrees>.
// TEST first moves to the offset using conservative fixed setup gains, then
// switches to the candidate PID for the return-to-home response.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const char FIRMWARE_VERSION[] =
    "DiscoveryPID position-hold-v1.1 2026-08-07";

// ============================================================================
// HARDWARE
// ============================================================================

constexpr uint8_t ENC_A_PIN = 2;
constexpr uint8_t ENC_B_PIN = 3;
constexpr uint8_t MOTOR_IN1_PIN = 9;
constexpr uint8_t MOTOR_IN2_PIN = 10;

// ============================================================================
// MOTOR / ENCODER CALIBRATION
// ============================================================================

constexpr float GEAR_RATIO = 30613.0f / 1500.0f;
constexpr float COUNTS_PER_MOTOR_REV = 28.0f;
constexpr int8_t ENCODER_DIRECTION_SIGN = 1;

constexpr float COUNTS_PER_OUTPUT_REV =
    COUNTS_PER_MOTOR_REV * GEAR_RATIO;

constexpr float DEGREES_PER_OUTPUT_COUNT =
    360.0f / COUNTS_PER_OUTPUT_REV;

// ============================================================================
// TIMING
// ============================================================================

constexpr uint32_t CONTROL_INTERVAL_MS = 10;
constexpr uint8_t TELEMETRY_DIVIDER = 10;  // FAST mode: about 10 rows/second.
constexpr uint8_t HUMAN_TELEMETRY_DIVIDER = 50; // HUMAN mode: about 2 rows/second.

enum class TelemetryMode : uint8_t
{
    QUIET = 0,
    HUMAN = 1,
    FAST = 2
};

TelemetryMode telemetryMode = TelemetryMode::HUMAN;

// ============================================================================
// CALIBRATION
// ============================================================================

constexpr long CALIBRATION_MIN_SPAN_COUNTS = 48;  // About 30 degrees total.
constexpr uint8_t CALIBRATION_MIN_REVERSALS = 1;
constexpr uint32_t CALIBRATION_STILL_MS = 1500;

// ============================================================================
// POSITION PID — RUNTIME/JMP TUNABLE
// Units:
//   Kp: RPM / degree
//   Ki: RPM / (degree * second)
//   Kd: RPM / (degree / second), derivative on measured position
// ============================================================================

float positionKp = 0.40f;
float positionKi = 0.00f;
float positionKd = 0.050f;

constexpr float MIN_POSITION_KP = 0.0f;
constexpr float MAX_POSITION_KP = 3.0f;
constexpr float MIN_POSITION_KI = 0.0f;
constexpr float MAX_POSITION_KI = 1.0f;
constexpr float MIN_POSITION_KD = 0.0f;
constexpr float MAX_POSITION_KD = 0.20f;

float maximumReturnRPM = 15.0f;
constexpr float MIN_RETURN_RPM = 5.0f;
constexpr float MAX_RETURN_RPM = 60.0f;

constexpr float POSITION_INTEGRAL_LIMIT = 80.0f;
constexpr float POSITION_DEADBAND_DEG = 0.45f;
constexpr float POSITION_EXIT_DEADBAND_DEG = 0.90f;
constexpr float POSITION_DEADBAND_RPM = 1.20f;

// ============================================================================
// INNER SPEED PI
// Preserves the previously tuned speed-loop defaults.
// ============================================================================

float speedKp = 2.0f;
float speedKi = 5.0f;

constexpr float MIN_SPEED_KP = 0.0f;
constexpr float MAX_SPEED_KP = 10.0f;
constexpr float MIN_SPEED_KI = 0.0f;
constexpr float MAX_SPEED_KI = 50.0f;

constexpr float SPEED_INTEGRAL_LIMIT = 300.0f;
constexpr float MAX_PWM_COMMAND = 225.0f;

// ============================================================================
// TEST PROFILE
// ============================================================================

constexpr float MIN_TEST_OFFSET_DEG = 5.0f;
constexpr float MAX_TEST_OFFSET_DEG = 30.0f;

constexpr float SETUP_POSITION_KP = 0.65f;
constexpr float SETUP_POSITION_KI = 0.00f;
constexpr float SETUP_POSITION_KD = 0.025f;
constexpr float SETUP_MAX_RPM = 15.0f;

constexpr float TEST_OFFSET_TOLERANCE_DEG = 1.5f;
constexpr float TEST_OFFSET_TOLERANCE_RPM = 2.0f;
constexpr uint32_t TEST_OFFSET_SETTLE_MS = 350;
constexpr uint32_t TEST_DWELL_MS = 500;
constexpr uint32_t TEST_RETURN_SETTLE_MS = 1000;
constexpr uint32_t TEST_RETURN_TIMEOUT_MS = 8000;
constexpr float TEST_HOME_TOLERANCE_DEG = 1.2f;
constexpr float TEST_HOME_TOLERANCE_RPM = 1.5f;

// ============================================================================
// SAFETY
// ============================================================================

constexpr float MAX_ALLOWED_POSITION_DEG = 75.0f;
constexpr float MAX_ALLOWED_OUTPUT_RPM = 70.0f;
constexpr float REVERSAL_SPEED_THRESHOLD_RPM = 1.5f;
constexpr uint32_t REVERSAL_ZERO_WAIT_MS = 20;
constexpr float STALL_ERROR_THRESHOLD_DEG = 3.0f;
constexpr float STALL_PWM_THRESHOLD = 170.0f;
constexpr uint32_t STALL_TIMEOUT_MS = 1200;

// ============================================================================
// MODE
// ============================================================================

enum class Mode : uint8_t
{
    CALIBRATING = 0,
    HOLD = 1,
    TEST_OUT = 2,
    TEST_DWELL = 3,
    TEST_RETURN = 4,
    STOPPED = 5,
    FAULT = 6
};

// ============================================================================
// ENCODER
// ============================================================================

volatile long encoderCount = 0;
volatile uint8_t previousEncoderState = 0;

const int8_t QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

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

long readEncoderAtomic()
{
    noInterrupts();
    const long value = encoderCount;
    interrupts();
    return value;
}

// ============================================================================
// GLOBAL STATE
// ============================================================================

Mode mode = Mode::CALIBRATING;
bool homed = false;
bool positionDeadbandActive = false;

unsigned long trialNumber = 0;

long homeEncoderCount = 0;
long previousEncoderCount = 0;

float measuredOutputRPM = 0.0f;
float rawOutputRPM = 0.0f;
float measuredMotorRPM = 0.0f;

float targetPositionDeg = 0.0f;
float positionErrorDeg = 0.0f;
float positionIntegral = 0.0f;
float targetOutputRPM = 0.0f;

float speedErrorRPM = 0.0f;
float speedIntegral = 0.0f;
float appliedMotorCommand = 0.0f;

uint32_t previousControlTime = 0;
uint32_t modeEnteredAt = 0;
uint32_t conditionStartedAt = 0;
uint32_t stallStartedAt = 0;
uint32_t reversalZeroStartedAt = 0;

int8_t appliedDirection = 0;

// Calibration state.
long calibrationMinimumCount = 0;
long calibrationMaximumCount = 0;
int8_t calibrationLastDirection = 0;
uint8_t calibrationReversals = 0;
uint32_t calibrationLastMotionAt = 0;

// Test state.
float testOffsetDeg = 20.0f;

// Fault text is intentionally short for RAM safety.
char faultCode[32] = "NONE";

// ============================================================================
// MOTOR OUTPUT
// ============================================================================

// Forward: IN1 LOW, PWM on IN2
// Reverse: IN2 LOW, PWM on IN1
// Stop: both LOW
void setMotorRaw(float command)
{
    const int pwm = constrain(
        static_cast<int>(fabsf(command)),
        0,
        static_cast<int>(MAX_PWM_COMMAND)
    );

    if (pwm == 0)
    {
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, 0);
        appliedMotorCommand = 0.0f;
        return;
    }

    if (command > 0.0f)
    {
        analogWrite(MOTOR_IN1_PIN, 0);
        analogWrite(MOTOR_IN2_PIN, pwm);
        appliedMotorCommand = static_cast<float>(pwm);
    }
    else
    {
        analogWrite(MOTOR_IN2_PIN, 0);
        analogWrite(MOTOR_IN1_PIN, pwm);
        appliedMotorCommand = -static_cast<float>(pwm);
    }
}

void resetControllers()
{
    positionIntegral = 0.0f;
    speedIntegral = 0.0f;
    targetOutputRPM = 0.0f;
    speedErrorRPM = 0.0f;
    positionDeadbandActive = false;
    reversalZeroStartedAt = 0;
    appliedDirection = 0;
    setMotorRaw(0.0f);
}

void enterMode(Mode newMode)
{
    mode = newMode;
    modeEnteredAt = millis();
    conditionStartedAt = 0;
}

void enterFault(const char *code)
{
    strncpy(faultCode, code, sizeof(faultCode) - 1);
    faultCode[sizeof(faultCode) - 1] = '\0';

    enterMode(Mode::FAULT);
    resetControllers();

    Serial.print("EVENT FAULT ");
    Serial.println(faultCode);
}

// Zero-speed reversal interlock.
float applyReversalInterlock(float desiredCommand, uint32_t now)
{
    const int8_t desiredDirection =
        desiredCommand > 0.5f ? 1 :
        desiredCommand < -0.5f ? -1 : 0;

    if (desiredDirection == 0)
    {
        if (fabsf(measuredOutputRPM) <= REVERSAL_SPEED_THRESHOLD_RPM)
        {
            appliedDirection = 0;
        }

        reversalZeroStartedAt = 0;
        return 0.0f;
    }

    if (appliedDirection == 0)
    {
        appliedDirection = desiredDirection;
        return desiredCommand;
    }

    if (desiredDirection == appliedDirection)
    {
        reversalZeroStartedAt = 0;
        return desiredCommand;
    }

    // Opposite direction requested: coast to nearly zero before reversing.
    if (fabsf(measuredOutputRPM) > REVERSAL_SPEED_THRESHOLD_RPM)
    {
        reversalZeroStartedAt = 0;
        return 0.0f;
    }

    if (reversalZeroStartedAt == 0)
    {
        reversalZeroStartedAt = now;
        return 0.0f;
    }

    if (now - reversalZeroStartedAt < REVERSAL_ZERO_WAIT_MS)
    {
        return 0.0f;
    }

    appliedDirection = desiredDirection;
    reversalZeroStartedAt = 0;
    return desiredCommand;
}

// ============================================================================
// POSITION HELPERS
// ============================================================================

float countsToPositionDeg(long count)
{
    return static_cast<float>(count - homeEncoderCount) *
           DEGREES_PER_OUTPUT_COUNT *
           ENCODER_DIRECTION_SIGN;
}

void setCurrentPositionAsHome()
{
    homeEncoderCount = readEncoderAtomic();
    homed = true;
    targetPositionDeg = 0.0f;
    resetControllers();
    enterMode(Mode::HOLD);

    Serial.println("EVENT HOMED");
}

void beginCalibration()
{
    homed = false;
    resetControllers();
    enterMode(Mode::CALIBRATING);

    const long current = readEncoderAtomic();
    calibrationMinimumCount = current;
    calibrationMaximumCount = current;
    calibrationLastDirection = 0;
    calibrationReversals = 0;
    calibrationLastMotionAt = millis();

    Serial.println(
        "EVENT CALIBRATION_BEGIN move left/right, then hold at home"
    );
}

void updateCalibration(long currentCount, long deltaCount, uint32_t now)
{
    setMotorRaw(0.0f);

    if (currentCount < calibrationMinimumCount)
    {
        calibrationMinimumCount = currentCount;
    }

    if (currentCount > calibrationMaximumCount)
    {
        calibrationMaximumCount = currentCount;
    }

    const int8_t direction =
        deltaCount > 0 ? 1 :
        deltaCount < 0 ? -1 : 0;

    if (direction != 0)
    {
        calibrationLastMotionAt = now;

        if (
            calibrationLastDirection != 0 &&
            direction != calibrationLastDirection
        )
        {
            ++calibrationReversals;
        }

        calibrationLastDirection = direction;
    }

    const long span =
        calibrationMaximumCount - calibrationMinimumCount;

    const bool movedEnough =
        span >= CALIBRATION_MIN_SPAN_COUNTS;

    const bool reversedEnough =
        calibrationReversals >= CALIBRATION_MIN_REVERSALS;

    const bool stillLongEnough =
        now - calibrationLastMotionAt >= CALIBRATION_STILL_MS;

    if (movedEnough && reversedEnough && stillLongEnough)
    {
        setCurrentPositionAsHome();
    }
}

// ============================================================================
// CONTROLLERS
// ============================================================================

float calculatePositionSpeedTarget(
    float errorDeg,
    float outputPositionVelocityDegPerSec,
    float dt,
    bool useSetupController
)
{
    const float kp =
        useSetupController ? SETUP_POSITION_KP : positionKp;

    const float ki =
        useSetupController ? SETUP_POSITION_KI : positionKi;

    const float kd =
        useSetupController ? SETUP_POSITION_KD : positionKd;

    const float rpmLimit =
        useSetupController ? SETUP_MAX_RPM : maximumReturnRPM;

    // Hysteretic deadband for normal home holding.
    if (!useSetupController)
    {
        if (
            positionDeadbandActive &&
            fabsf(errorDeg) >= POSITION_EXIT_DEADBAND_DEG
        )
        {
            positionDeadbandActive = false;
        }

        if (
            !positionDeadbandActive &&
            fabsf(errorDeg) <= POSITION_DEADBAND_DEG &&
            fabsf(measuredOutputRPM) <= POSITION_DEADBAND_RPM
        )
        {
            positionDeadbandActive = true;
        }

        if (positionDeadbandActive)
        {
            positionIntegral *= 0.90f;
            return 0.0f;
        }
    }
    else
    {
        positionDeadbandActive = false;
    }

    const float proposedIntegral = constrain(
        positionIntegral + errorDeg * dt,
        -POSITION_INTEGRAL_LIMIT,
        POSITION_INTEGRAL_LIMIT
    );

    const float proposedRPM =
        kp * errorDeg +
        ki * proposedIntegral -
        kd * outputPositionVelocityDegPerSec;

    const bool notSaturated =
        proposedRPM < rpmLimit &&
        proposedRPM > -rpmLimit;

    if (notSaturated)
    {
        positionIntegral = proposedIntegral;
    }

    const float rpmCommand =
        kp * errorDeg +
        ki * positionIntegral -
        kd * outputPositionVelocityDegPerSec;

    return constrain(rpmCommand, -rpmLimit, rpmLimit);
}

float calculateSpeedPWM(float rpmTarget, float dt)
{
    speedErrorRPM = rpmTarget - measuredOutputRPM;

    if (fabsf(rpmTarget) < 0.01f)
    {
        speedIntegral = 0.0f;
        return 0.0f;
    }

    const float proposedIntegral = constrain(
        speedIntegral + speedErrorRPM * dt,
        -SPEED_INTEGRAL_LIMIT,
        SPEED_INTEGRAL_LIMIT
    );

    const float proposedCommand =
        speedKp * speedErrorRPM +
        speedKi * proposedIntegral;

    if (
        proposedCommand < MAX_PWM_COMMAND &&
        proposedCommand > -MAX_PWM_COMMAND
    )
    {
        speedIntegral = proposedIntegral;
    }

    return constrain(
        speedKp * speedErrorRPM + speedKi * speedIntegral,
        -MAX_PWM_COMMAND,
        MAX_PWM_COMMAND
    );
}

// ============================================================================
// TEST STATE MACHINE
// ============================================================================

void beginTest(float requestedOffsetDeg)
{
    if (!homed)
    {
        Serial.println("ERR NOT_HOMED");
        return;
    }

    if (
        fabsf(requestedOffsetDeg) < MIN_TEST_OFFSET_DEG ||
        fabsf(requestedOffsetDeg) > MAX_TEST_OFFSET_DEG
    )
    {
        Serial.println("ERR TEST_OFFSET_OUT_OF_RANGE");
        return;
    }

    testOffsetDeg = requestedOffsetDeg;
    targetPositionDeg = testOffsetDeg;
    resetControllers();
    enterMode(Mode::TEST_OUT);

    Serial.print("ACK TEST=");
    Serial.println(testOffsetDeg, 3);
    Serial.println("EVENT TEST_OUT");
}

void updateTestState(uint32_t now)
{
    if (mode == Mode::TEST_OUT)
    {
        const bool atOffset =
            fabsf(positionErrorDeg) <= TEST_OFFSET_TOLERANCE_DEG &&
            fabsf(measuredOutputRPM) <= TEST_OFFSET_TOLERANCE_RPM;

        if (atOffset)
        {
            if (conditionStartedAt == 0)
            {
                conditionStartedAt = now;
            }

            if (now - conditionStartedAt >= TEST_OFFSET_SETTLE_MS)
            {
                enterMode(Mode::TEST_DWELL);
                Serial.println("EVENT TEST_DWELL");
            }
        }
        else
        {
            conditionStartedAt = 0;
        }

        if (now - modeEnteredAt > 7000)
        {
            targetPositionDeg = 0.0f;
            resetControllers();
            enterMode(Mode::HOLD);
            Serial.println("EVENT TEST_PREP_TIMEOUT");
        }
    }
    else if (mode == Mode::TEST_DWELL)
    {
        if (now - modeEnteredAt >= TEST_DWELL_MS)
        {
            targetPositionDeg = 0.0f;
            resetControllers();
            enterMode(Mode::TEST_RETURN);
            Serial.println("EVENT TEST_RETURN");
        }
    }
    else if (mode == Mode::TEST_RETURN)
    {
        const bool settledHome =
            fabsf(positionErrorDeg) <= TEST_HOME_TOLERANCE_DEG &&
            fabsf(measuredOutputRPM) <= TEST_HOME_TOLERANCE_RPM;

        if (settledHome)
        {
            if (conditionStartedAt == 0)
            {
                conditionStartedAt = now;
            }

            if (now - conditionStartedAt >= TEST_RETURN_SETTLE_MS)
            {
                targetPositionDeg = 0.0f;
                enterMode(Mode::HOLD);
                Serial.println("EVENT TEST_COMPLETE");
            }
        }
        else
        {
            conditionStartedAt = 0;
        }

        if (now - modeEnteredAt >= TEST_RETURN_TIMEOUT_MS)
        {
            targetPositionDeg = 0.0f;
            enterMode(Mode::HOLD);
            Serial.println("EVENT TEST_TIMEOUT");
        }
    }
}

// ============================================================================
// SERIAL
// ============================================================================

bool parseNumber(const char *text, float &value)
{
    char *parseEnd = nullptr;

    const float parsedValue =
        static_cast<float>(strtod(text, &parseEnd));

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

void printIdentity()
{
    Serial.print("Firmware: ");
    Serial.println(FIRMWARE_VERSION);
}

void printStatus()
{
    const long currentCount = readEncoderAtomic();
    const float currentPosition = countsToPositionDeg(currentCount);

    Serial.print("STATUS Trial=");
    Serial.print(trialNumber);

    Serial.print(" Mode=");
    Serial.print(static_cast<uint8_t>(mode));

    Serial.print(" Homed=");
    Serial.print(homed ? 1 : 0);

    Serial.print(" KP=");
    Serial.print(positionKp, 6);

    Serial.print(" KI=");
    Serial.print(positionKi, 6);

    Serial.print(" KD=");
    Serial.print(positionKd, 6);

    Serial.print(" PosDeg=");
    Serial.print(currentPosition, 4);

    Serial.print(" PosTarget=");
    Serial.print(targetPositionDeg, 4);

    Serial.print(" MaxRPM=");
    Serial.print(maximumReturnRPM, 3);

    Serial.print(" Fault=");
    Serial.print(mode == Mode::FAULT ? 1 : 0);

    Serial.print(" Telem=");
    Serial.println(static_cast<uint8_t>(telemetryMode));
}

void acknowledgeGain(const char *name, float value)
{
    Serial.print("ACK ");
    Serial.print(name);
    Serial.print("=");
    Serial.println(value, 6);
}

void processSerialCommand(char *command)
{
    if (strcmp(command, "IDENTIFY") == 0)
    {
        printIdentity();
        return;
    }

    if (strcmp(command, "STATUS") == 0)
    {
        printStatus();
        return;
    }

    if (strcmp(command, "STOP") == 0)
    {
        targetPositionDeg = 0.0f;
        enterMode(Mode::STOPPED);
        resetControllers();
        Serial.println("ACK STOP");
        return;
    }

    if (strcmp(command, "HOLD") == 0)
    {
        if (!homed)
        {
            Serial.println("ERR NOT_HOMED");
            return;
        }

        targetPositionDeg = 0.0f;
        resetControllers();
        enterMode(Mode::HOLD);
        Serial.println("ACK HOLD");
        return;
    }

    if (
        strcmp(command, "ZERO") == 0 ||
        strcmp(command, "HOME") == 0
    )
    {
        setCurrentPositionAsHome();
        Serial.println("ACK ZERO");
        return;
    }

    if (strcmp(command, "CALIBRATE") == 0)
    {
        beginCalibration();
        Serial.println("ACK CALIBRATE");
        return;
    }

    if (strcmp(command, "CLEARFAULT") == 0)
    {
        strncpy(faultCode, "NONE", sizeof(faultCode));

        if (homed)
        {
            targetPositionDeg = 0.0f;
            resetControllers();
            enterMode(Mode::HOLD);
        }
        else
        {
            beginCalibration();
        }

        Serial.println("ACK CLEARFAULT");
        return;
    }

    char *equalsSign = strchr(command, '=');

    if (equalsSign == nullptr)
    {
        Serial.println("ERR UNKNOWN_COMMAND");
        return;
    }

    *equalsSign = '\0';

    const char *key = command;
    const char *valueText = equalsSign + 1;

    float value = 0.0f;

    if (!parseNumber(valueText, value))
    {
        Serial.println("ERR INVALID_NUMBER");
        return;
    }

    if (strcmp(key, "TELEM") == 0)
    {
        const int modeValue = static_cast<int>(value);

        if (modeValue < 0 || modeValue > 2 || fabsf(value - modeValue) > 0.001f)
        {
            Serial.println("ERR TELEM_OUT_OF_RANGE");
            return;
        }

        telemetryMode = static_cast<TelemetryMode>(modeValue);
        Serial.print("ACK TELEM=");
        Serial.println(modeValue);
        return;
    }

    if (strcmp(key, "KP") == 0)
    {
        if (value < MIN_POSITION_KP || value > MAX_POSITION_KP)
        {
            Serial.println("ERR KP_OUT_OF_RANGE");
            return;
        }

        positionKp = value;
        resetControllers();
        acknowledgeGain("KP", positionKp);
        return;
    }

    if (strcmp(key, "KI") == 0)
    {
        if (value < MIN_POSITION_KI || value > MAX_POSITION_KI)
        {
            Serial.println("ERR KI_OUT_OF_RANGE");
            return;
        }

        positionKi = value;
        resetControllers();
        acknowledgeGain("KI", positionKi);
        return;
    }

    if (strcmp(key, "KD") == 0)
    {
        if (value < MIN_POSITION_KD || value > MAX_POSITION_KD)
        {
            Serial.println("ERR KD_OUT_OF_RANGE");
            return;
        }

        positionKd = value;
        resetControllers();
        acknowledgeGain("KD", positionKd);
        return;
    }

    if (strcmp(key, "SP") == 0)
    {
        if (value < MIN_SPEED_KP || value > MAX_SPEED_KP)
        {
            Serial.println("ERR SP_OUT_OF_RANGE");
            return;
        }

        speedKp = value;
        resetControllers();
        acknowledgeGain("SP", speedKp);
        return;
    }

    if (strcmp(key, "SI") == 0)
    {
        if (value < MIN_SPEED_KI || value > MAX_SPEED_KI)
        {
            Serial.println("ERR SI_OUT_OF_RANGE");
            return;
        }

        speedKi = value;
        resetControllers();
        acknowledgeGain("SI", speedKi);
        return;
    }

    if (strcmp(key, "MAXRPM") == 0)
    {
        if (value < MIN_RETURN_RPM || value > MAX_RETURN_RPM)
        {
            Serial.println("ERR MAXRPM_OUT_OF_RANGE");
            return;
        }

        maximumReturnRPM = value;
        Serial.print("ACK MAXRPM=");
        Serial.println(maximumReturnRPM, 3);
        return;
    }

    if (strcmp(key, "POS") == 0)
    {
        if (!homed)
        {
            Serial.println("ERR NOT_HOMED");
            return;
        }

        if (fabsf(value) > MAX_ALLOWED_POSITION_DEG)
        {
            Serial.println("ERR POS_OUT_OF_RANGE");
            return;
        }

        targetPositionDeg = value;
        resetControllers();
        enterMode(Mode::HOLD);

        Serial.print("ACK POS=");
        Serial.println(targetPositionDeg, 3);
        return;
    }

    if (strcmp(key, "TEST") == 0)
    {
        beginTest(value);
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
}

void readSerialCommands()
{
    static char inputBuffer[96];
    static uint8_t inputIndex = 0;

    while (Serial.available() > 0)
    {
        const char incoming =
            static_cast<char>(Serial.read());

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

// ============================================================================
// TELEMETRY
// ============================================================================

void printTelemetry(
    long currentCount,
    long deltaCount,
    float currentPositionDeg
)
{
    Serial.print("Trial=");
    Serial.print(trialNumber);

    Serial.print(" Mode=");
    Serial.print(static_cast<uint8_t>(mode));

    Serial.print(" Homed=");
    Serial.print(homed ? 1 : 0);

    Serial.print(" KP=");
    Serial.print(positionKp, 5);

    Serial.print(" KI=");
    Serial.print(positionKi, 5);

    Serial.print(" KD=");
    Serial.print(positionKd, 5);

    Serial.print(" PosDeg=");
    Serial.print(currentPositionDeg, 3);

    Serial.print(" PosTarget=");
    Serial.print(targetPositionDeg, 3);

    Serial.print(" PosError=");
    Serial.print(positionErrorDeg, 3);

    Serial.print(" PosIntegral=");
    Serial.print(positionIntegral, 3);

    Serial.print(" SpeedTarget=");
    Serial.print(targetOutputRPM, 3);

    Serial.print(" OutRPM=");
    Serial.print(measuredOutputRPM, 3);

    Serial.print(" MotorRPM=");
    Serial.print(measuredMotorRPM, 1);

    Serial.print(" SpeedError=");
    Serial.print(speedErrorRPM, 3);

    Serial.print(" Counts=");
    Serial.print(currentCount);

    Serial.print(" dC=");
    Serial.print(deltaCount);

    Serial.print(" PWM=");
    Serial.println(appliedMotorCommand, 1);
}

// ============================================================================
// STARTUP
// ============================================================================

void setup()
{
    Serial.begin(115200);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    setMotorRaw(0.0f);

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

    previousEncoderCount = readEncoderAtomic();
    previousControlTime = millis();

    printIdentity();

    Serial.println(
        "Commands: IDENTIFY, STATUS, CALIBRATE, ZERO, HOLD, STOP, "
        "KP=<>, KI=<>, KD=<>, SP=<>, SI=<>, MAXRPM=<>, POS=<>, "
        "TEST=<deg>, TRIAL=<>, TELEM=0|1|2"
    );

    beginCalibration();
    printStatus();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

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

    const long currentCount = readEncoderAtomic();
    const long deltaCount = currentCount - previousEncoderCount;

    previousEncoderCount = currentCount;

    measuredMotorRPM =
        ENCODER_DIRECTION_SIGN *
        (static_cast<float>(deltaCount) / COUNTS_PER_MOTOR_REV) *
        (60.0f / dt);

    rawOutputRPM = measuredMotorRPM / GEAR_RATIO;

    // Low-pass the quantized 10 ms encoder-speed estimate.
    measuredOutputRPM =
        0.75f * measuredOutputRPM +
        0.25f * rawOutputRPM;

    const float currentPositionDeg =
        homed ? countsToPositionDeg(currentCount) : 0.0f;

    positionErrorDeg =
        homed ? (targetPositionDeg - currentPositionDeg) : 0.0f;

    if (mode == Mode::CALIBRATING)
    {
        updateCalibration(currentCount, deltaCount, now);
    }
    else if (
        mode == Mode::STOPPED ||
        mode == Mode::FAULT ||
        !homed
    )
    {
        setMotorRaw(0.0f);
    }
    else
    {
        updateTestState(now);

        if (
            fabsf(currentPositionDeg) > MAX_ALLOWED_POSITION_DEG ||
            fabsf(measuredOutputRPM) > MAX_ALLOWED_OUTPUT_RPM
        )
        {
            enterFault("LIMIT");
        }
        else
        {
            const bool useSetupController =
                mode == Mode::TEST_OUT ||
                mode == Mode::TEST_DWELL;

            const float positionVelocityDegPerSec =
                measuredOutputRPM * 6.0f;

            targetOutputRPM =
                calculatePositionSpeedTarget(
                    positionErrorDeg,
                    positionVelocityDegPerSec,
                    dt,
                    useSetupController
                );

            float desiredPWM =
                calculateSpeedPWM(targetOutputRPM, dt);

            desiredPWM =
                applyReversalInterlock(desiredPWM, now);

            setMotorRaw(desiredPWM);

            // Stall protection only when the controller is asking for motion
            // because a static holding torque is valid near home.
            const bool stallProtectionEnabled =
                mode == Mode::TEST_OUT ||
                mode == Mode::TEST_RETURN;

            const bool possibleStall =
                stallProtectionEnabled &&
                fabsf(positionErrorDeg) >= STALL_ERROR_THRESHOLD_DEG &&
                fabsf(appliedMotorCommand) >= STALL_PWM_THRESHOLD &&
                deltaCount == 0;

            if (possibleStall)
            {
                if (stallStartedAt == 0)
                {
                    stallStartedAt = now;
                }

                if (now - stallStartedAt >= STALL_TIMEOUT_MS)
                {
                    enterFault("STALL");
                }
            }
            else
            {
                stallStartedAt = 0;
            }
        }
    }

    static uint8_t fastTelemetryDivider = 0;
    static uint8_t humanTelemetryDivider = 0;

    if (telemetryMode == TelemetryMode::FAST)
    {
        if (++fastTelemetryDivider >= TELEMETRY_DIVIDER)
        {
            fastTelemetryDivider = 0;
            printTelemetry(currentCount, deltaCount, currentPositionDeg);
        }
    }
    else if (telemetryMode == TelemetryMode::HUMAN)
    {
        const bool worthShowing =
            mode == Mode::CALIBRATING ||
            mode == Mode::FAULT ||
            fabsf(positionErrorDeg) >= 0.9f ||
            fabsf(measuredOutputRPM) >= 1.0f ||
            fabsf(appliedMotorCommand) >= 1.0f;

        if (worthShowing && ++humanTelemetryDivider >= HUMAN_TELEMETRY_DIVIDER)
        {
            humanTelemetryDivider = 0;

            Serial.print("LIVE Pos=");
            Serial.print(currentPositionDeg, 2);
            Serial.print("deg Err=");
            Serial.print(positionErrorDeg, 2);
            Serial.print("deg RPM=");
            Serial.print(measuredOutputRPM, 1);
            Serial.print(" PWM=");
            Serial.print(appliedMotorCommand, 0);
            Serial.print(" Mode=");
            Serial.println(static_cast<uint8_t>(mode));
        }

        if (!worthShowing)
        {
            humanTelemetryDivider = 0;
        }
    }
}
