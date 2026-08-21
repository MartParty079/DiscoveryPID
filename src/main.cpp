// DiscoveryPID position-hold firmware
// Arduino Mega 2560 + Adafruit 4416 geared motor + DRV8871.
//
// Control architecture:
//   TEST_OUT: best-scoring completed PID so far -> signed PWM directly
//   TEST_RETURN: current candidate PID -> signed PWM directly
//
// The position PID runs locally on the Mega every 10 ms. Encoder quadrature
// edges are counted asynchronously by interrupts. Opposite-sign PID output is
// applied immediately, so the D term can generate reverse torque before the
// target is crossed. There is NO separate hardware-brake algorithm: PID=0
// means motor coast. Braking torque comes only from signed PID output.
// FAST telemetry is a compact sample emitted on every 10 ms control update so
// recorded position traces preserve the controller's full 100 Hz time base.

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const char FIRMWARE_VERSION[] =
    "DiscoveryPID direct-position-pid-v3.8-final-approach 2026-08-20";

constexpr int EEPROM_HOME_MAGIC_ADDRESS = 0;
constexpr uint32_t EEPROM_HOME_MAGIC = 0x44504944UL; // "DPID"

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
constexpr float COUNTS_PER_MOTOR_REV = 56.0f;
constexpr int8_t ENCODER_DIRECTION_SIGN = 1;
constexpr float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;
constexpr float DEGREES_PER_OUTPUT_COUNT = 360.0f / COUNTS_PER_OUTPUT_REV;

// ============================================================================
// TIMING
// ============================================================================

constexpr uint32_t CONTROL_INTERVAL_MS = 10;       // 100 Hz closed-loop PID.
constexpr uint8_t TELEMETRY_DIVIDER = 1;           // FAST: one row/control tick = 100 Hz.
constexpr uint8_t HUMAN_TELEMETRY_DIVIDER = 50;    // HUMAN: ~2 rows/s.

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

constexpr long CALIBRATION_MIN_SPAN_COUNTS = 96;  // ~30 output degrees total.
constexpr uint8_t CALIBRATION_MIN_REVERSALS = 1;
constexpr uint32_t CALIBRATION_STILL_MS = 1500;

// ============================================================================
// DIRECT POSITION PID — RUNTIME/JMP TUNABLE
//
// PWM = Kp*positionError + Ki*integral - Kd*measuredPositionVelocity
//
// Kd therefore becomes genuine braking torque when approach velocity makes
// the net PID command oppose the current shaft motion.
// ============================================================================

float positionKp = 1.35f;
float positionKi = 0.40f;
float positionKd = 0.10f;

constexpr float MIN_POSITION_KP = 0.0f;
constexpr float MAX_POSITION_KP = 1000.0f;
constexpr float MIN_POSITION_KI = 0.0f;
constexpr float MAX_POSITION_KI = 500.0f;
constexpr float MIN_POSITION_KD = 0.0f;
constexpr float MAX_POSITION_KD = 200.0f;

constexpr float MAX_PWM_COMMAND = 225.0f;
float maximumReturnPWM = 225.0f;

// Full breakaway floor far from home. For TEST_RETURN it now tapers to zero
// earlier so the last few degrees are controlled by P/I/D rather than a forced
// minimum motor command.
float minimumDrivePWM = 80.0f;
constexpr float RETURN_MIN_PWM_TAPER_START_DEG = 8.0f;
constexpr float RETURN_MIN_PWM_TAPER_END_DEG = 3.0f;

// Encoder velocity is quantized at the 10 ms control rate. Filtering suppresses
// one-count derivative noise while retaining useful D-term braking response.
constexpr float OUTPUT_RPM_FILTER_ALPHA = 0.15f;

constexpr float POSITION_INTEGRAL_LIMIT = 80.0f;
constexpr float POSITION_I_ENABLE_BAND_DEG = 2.0f;
constexpr float POSITION_I_MAX_PWM = 12.0f;
constexpr float POSITION_I_BLEED_PER_SEC = 8.0f;
float previousPositionErrorDeg = 0.0f;

constexpr float POSITION_DEADBAND_DEG = 0.45f;
constexpr float POSITION_EXIT_DEADBAND_DEG = 0.90f;
constexpr float POSITION_DEADBAND_RPM = 1.20f;

long homePaddingPulses = 2;
constexpr uint8_t POSITION_SETTLE_REQUIRED_SAMPLES = 5;

// ============================================================================
// TEST PROFILE
// ============================================================================

constexpr float MIN_TEST_OFFSET_DEG = 5.0f;
constexpr float MAX_TEST_OFFSET_DEG = 1000000.0f;

float setupKp = 1.35f;
float setupKi = 0.40f;
float setupKd = 0.10f;

constexpr float TEST_OFFSET_TOLERANCE_DEG = 1.5f;
constexpr float TEST_OFFSET_TOLERANCE_RPM = 2.0f;
constexpr uint32_t TEST_OFFSET_SETTLE_MS = 350;
constexpr uint32_t TEST_DWELL_MS = 500;
constexpr uint32_t TEST_PREP_TIMEOUT_MS = 45000;
constexpr uint32_t TEST_RETURN_SETTLE_MS = 1000;
constexpr uint32_t TEST_RETURN_TIMEOUT_MS = 45000;
constexpr float TEST_HOME_TOLERANCE_DEG = 1.2f;
constexpr float TEST_HOME_TOLERANCE_RPM = 1.5f;

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

    const uint8_t transitionIndex = (previousEncoderState << 2) | currentState;
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
bool positionSettledLatch = false;
uint8_t positionSettleSamples = 0;

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
float appliedMotorCommand = 0.0f;

uint32_t previousControlTime = 0;
uint32_t modeEnteredAt = 0;
uint32_t conditionStartedAt = 0;

long calibrationMinimumCount = 0;
long calibrationMaximumCount = 0;
int8_t calibrationLastDirection = 0;
uint8_t calibrationReversals = 0;
uint32_t calibrationLastMotionAt = 0;

float testOffsetDeg = 300.0f;
char faultCode[32] = "NONE";

// ============================================================================
// MOTOR OUTPUT
// ============================================================================

// DRV8871 usage for this pure-PID study:
//   IN1=0 IN2=0 -> coast
//   IN1=0 IN2=PWM -> one signed drive direction
//   IN1=PWM IN2=0 -> opposite signed drive direction
// Hardware IN1=1 / IN2=1 braking is intentionally NOT used.

void setMotorCoast()
{
    analogWrite(MOTOR_IN1_PIN, 0);
    analogWrite(MOTOR_IN2_PIN, 0);
    appliedMotorCommand = 0.0f;
}

void setMotorDrive(float command)
{
    const int pwm = constrain(
        static_cast<int>(lroundf(fabsf(command))),
        0,
        static_cast<int>(MAX_PWM_COMMAND)
    );

    if (pwm == 0)
    {
        setMotorCoast();
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

void applyClosedLoopMotorCommand(float desiredCommand)
{
    // No coast-before-reverse interlock and no separate brake state.
    // Every 10 ms, the signed PID output is applied directly. If Kd makes the
    // command oppose motion, that signed reverse PWM is the braking torque.
    setMotorDrive(desiredCommand);
}

void resetControllers()
{
    positionIntegral = 0.0f;
    previousPositionErrorDeg = 0.0f;
    targetOutputRPM = 0.0f;
    positionDeadbandActive = false;
    positionSettledLatch = false;
    positionSettleSamples = 0;
    setMotorCoast();
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

// ============================================================================
// POSITION / HOME HELPERS
// ============================================================================

float countsToPositionDeg(long count)
{
    return static_cast<float>(count - homeEncoderCount) *
           DEGREES_PER_OUTPUT_COUNT * ENCODER_DIRECTION_SIGN;
}

bool persistentHomeValid()
{
    uint32_t storedMagic = 0;
    EEPROM.get(EEPROM_HOME_MAGIC_ADDRESS, storedMagic);
    return storedMagic == EEPROM_HOME_MAGIC;
}

void setPersistentHomeValid(bool valid)
{
    const uint32_t value = valid ? EEPROM_HOME_MAGIC : 0UL;
    EEPROM.put(EEPROM_HOME_MAGIC_ADDRESS, value);
}

void restoreCurrentPositionAsHomeAfterReset()
{
    homeEncoderCount = readEncoderAtomic();
    homed = true;
    targetPositionDeg = 0.0f;
    resetControllers();
    enterMode(Mode::HOLD);
    Serial.println("EVENT HOME_RESTORED");
}

void setCurrentPositionAsHome()
{
    homeEncoderCount = readEncoderAtomic();
    homed = true;
    targetPositionDeg = 0.0f;
    resetControllers();
    enterMode(Mode::HOLD);
    setPersistentHomeValid(true);
    Serial.println("EVENT HOMED");
}

void beginCalibration()
{
    setPersistentHomeValid(false);
    homed = false;
    resetControllers();
    enterMode(Mode::CALIBRATING);

    const long current = readEncoderAtomic();
    calibrationMinimumCount = current;
    calibrationMaximumCount = current;
    calibrationLastDirection = 0;
    calibrationReversals = 0;
    calibrationLastMotionAt = millis();

    Serial.println("EVENT CALIBRATION_BEGIN move left/right, then hold at home");
}

long targetEncoderCount()
{
    const float countOffset =
        targetPositionDeg / (DEGREES_PER_OUTPUT_COUNT * ENCODER_DIRECTION_SIGN);
    return homeEncoderCount + lroundf(countOffset);
}

bool updatePositionSettledLatch(long currentCount, long deltaCount)
{
    const bool settlingAllowed =
        homed && (mode == Mode::HOLD || mode == Mode::TEST_RETURN);

    if (!settlingAllowed)
    {
        positionSettledLatch = false;
        positionSettleSamples = 0;
        return false;
    }

    const long errorCounts = targetEncoderCount() - currentCount;

    if (positionSettledLatch)
    {
        if (labs(errorCounts) > homePaddingPulses + 1)
        {
            positionSettledLatch = false;
            positionSettleSamples = 0;
        }
        else
        {
            positionIntegral = 0.0f;
            targetOutputRPM = 0.0f;
            setMotorCoast();
            return true;
        }
    }

    const bool withinEncoderResolution = labs(errorCounts) <= homePaddingPulses;
    const bool noNewEncoderCount = deltaCount == 0;

    if (withinEncoderResolution && noNewEncoderCount)
    {
        if (positionSettleSamples < POSITION_SETTLE_REQUIRED_SAMPLES)
        {
            ++positionSettleSamples;
        }

        if (positionSettleSamples >= POSITION_SETTLE_REQUIRED_SAMPLES)
        {
            positionSettledLatch = true;
            positionIntegral = 0.0f;
            targetOutputRPM = 0.0f;
            setMotorCoast();

            if (mode == Mode::HOLD && fabsf(targetPositionDeg) < 0.001f)
            {
                setPersistentHomeValid(true);
            }
            return true;
        }
    }
    else
    {
        positionSettleSamples = 0;
    }

    return false;
}

void updateCalibration(long currentCount, long deltaCount, uint32_t now)
{
    setMotorCoast();

    if (currentCount < calibrationMinimumCount) calibrationMinimumCount = currentCount;
    if (currentCount > calibrationMaximumCount) calibrationMaximumCount = currentCount;

    const int8_t direction = deltaCount > 0 ? 1 : deltaCount < 0 ? -1 : 0;

    if (direction != 0)
    {
        calibrationLastMotionAt = now;
        if (calibrationLastDirection != 0 && direction != calibrationLastDirection)
        {
            ++calibrationReversals;
        }
        calibrationLastDirection = direction;
    }

    const long span = calibrationMaximumCount - calibrationMinimumCount;
    const bool movedEnough = span >= CALIBRATION_MIN_SPAN_COUNTS;
    const bool reversedEnough = calibrationReversals >= CALIBRATION_MIN_REVERSALS;
    const bool stillLongEnough = now - calibrationLastMotionAt >= CALIBRATION_STILL_MS;

    if (movedEnough && reversedEnough && stillLongEnough)
    {
        setCurrentPositionAsHome();
    }
}

// ============================================================================
// CONTROLLER
// ============================================================================

float calculateDirectPositionPWM(
    float errorDeg,
    float outputPositionVelocityDegPerSec,
    float dt,
    bool useSetupController
)
{
    const float kp = useSetupController ? setupKp : positionKp;
    const float ki = useSetupController ? setupKi : positionKi;
    const float kd = useSetupController ? setupKd : positionKd;

    targetOutputRPM = 0.0f;

    if (!useSetupController)
    {
        if (positionDeadbandActive && fabsf(errorDeg) >= POSITION_EXIT_DEADBAND_DEG)
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
        if (fabsf(errorDeg) <= TEST_OFFSET_TOLERANCE_DEG)
        {
            positionIntegral = 0.0f;
            return 0.0f;
        }
    }

    const bool crossedTarget =
        previousPositionErrorDeg != 0.0f &&
        ((errorDeg > 0.0f && previousPositionErrorDeg < 0.0f) ||
         (errorDeg < 0.0f && previousPositionErrorDeg > 0.0f));

    if (crossedTarget)
    {
        positionIntegral = 0.0f;
    }
    previousPositionErrorDeg = errorDeg;

    if (fabsf(errorDeg) > POSITION_I_ENABLE_BAND_DEG || ki <= 0.0f)
    {
        if (ki <= 0.0f)
        {
            positionIntegral = 0.0f;
        }
        else
        {
            const float bleed = constrain(POSITION_I_BLEED_PER_SEC * dt, 0.0f, 1.0f);
            positionIntegral *= (1.0f - bleed);
        }
    }
    else
    {
        const float proposedIntegral = constrain(
            positionIntegral + errorDeg * dt,
            -POSITION_INTEGRAL_LIMIT,
            POSITION_INTEGRAL_LIMIT
        );

        const float proposedIntegralPWM = constrain(
            ki * proposedIntegral,
            -POSITION_I_MAX_PWM,
            POSITION_I_MAX_PWM
        );

        const float proposedPWM =
            kp * errorDeg + proposedIntegralPWM - kd * outputPositionVelocityDegPerSec;

        const bool insideLimit =
            proposedPWM < maximumReturnPWM && proposedPWM > -maximumReturnPWM;
        const bool unwindPositive =
            proposedPWM >= maximumReturnPWM && errorDeg < 0.0f;
        const bool unwindNegative =
            proposedPWM <= -maximumReturnPWM && errorDeg > 0.0f;

        if (insideLimit || unwindPositive || unwindNegative)
        {
            positionIntegral = proposedIntegral;
        }
    }

    const float integralPWM = constrain(
        ki * positionIntegral,
        -POSITION_I_MAX_PWM,
        POSITION_I_MAX_PWM
    );

    float command =
        kp * errorDeg + integralPWM - kd * outputPositionVelocityDegPerSec;

    command = constrain(command, -maximumReturnPWM, maximumReturnPWM);

    float effectiveMinimumDrivePWM = minimumDrivePWM;

    if (!useSetupController && minimumDrivePWM > 0.0f)
    {
        const float absErrorDeg = fabsf(errorDeg);
        if (absErrorDeg <= RETURN_MIN_PWM_TAPER_END_DEG)
        {
            effectiveMinimumDrivePWM = 0.0f;
        }
        else if (absErrorDeg < RETURN_MIN_PWM_TAPER_START_DEG)
        {
            const float taper =
                (absErrorDeg - RETURN_MIN_PWM_TAPER_END_DEG) /
                (RETURN_MIN_PWM_TAPER_START_DEG - RETURN_MIN_PWM_TAPER_END_DEG);
            effectiveMinimumDrivePWM = minimumDrivePWM * taper;
        }
    }

    if (
        effectiveMinimumDrivePWM > 0.0f &&
        fabsf(errorDeg) > 0.0f &&
        fabsf(command) > 0.0f &&
        fabsf(command) < effectiveMinimumDrivePWM
    )
    {
        command = command > 0.0f
            ? effectiveMinimumDrivePWM
            : -effectiveMinimumDrivePWM;
    }

    return command;
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

    if (!isfinite(requestedOffsetDeg) || fabsf(requestedOffsetDeg) < 0.001f)
    {
        Serial.println("ERR INVALID_TEST_OFFSET");
        return;
    }

    setPersistentHomeValid(false);
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
            if (conditionStartedAt == 0) conditionStartedAt = now;
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

        if (now - modeEnteredAt > TEST_PREP_TIMEOUT_MS)
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
            if (conditionStartedAt == 0) conditionStartedAt = now;
            if (now - conditionStartedAt >= TEST_RETURN_SETTLE_MS)
            {
                targetPositionDeg = 0.0f;
                enterMode(Mode::HOLD);
                setPersistentHomeValid(true);
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
    const float parsedValue = static_cast<float>(strtod(text, &parseEnd));
    if (parseEnd == text) return false;
    while (*parseEnd == ' ' || *parseEnd == '\t') ++parseEnd;
    if (*parseEnd != '\0') return false;
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

    Serial.print("STATUS Trial="); Serial.print(trialNumber);
    Serial.print(" Mode="); Serial.print(static_cast<uint8_t>(mode));
    Serial.print(" Homed="); Serial.print(homed ? 1 : 0);
    Serial.print(" HomePad="); Serial.print(homePaddingPulses);
    Serial.print(" MinPWM="); Serial.print(minimumDrivePWM, 3);
    Serial.print(" SKP="); Serial.print(setupKp, 6);
    Serial.print(" SKI="); Serial.print(setupKi, 6);
    Serial.print(" SKD="); Serial.print(setupKd, 6);
    Serial.print(" Settled="); Serial.print(positionSettledLatch ? 1 : 0);
    Serial.print(" KP="); Serial.print(positionKp, 6);
    Serial.print(" KI="); Serial.print(positionKi, 6);
    Serial.print(" KD="); Serial.print(positionKd, 6);
    Serial.print(" PosDeg="); Serial.print(currentPosition, 4);
    Serial.print(" PosTarget="); Serial.print(targetPositionDeg, 4);
    Serial.print(" MaxPWM="); Serial.print(maximumReturnPWM, 3);
    Serial.print(" TestMax="); Serial.print(MAX_TEST_OFFSET_DEG, 1);
    Serial.print(" DegPerCount="); Serial.print(DEGREES_PER_OUTPUT_COUNT, 5);
    Serial.print(" Fault="); Serial.print(mode == Mode::FAULT ? 1 : 0);
    Serial.print(" Telem="); Serial.println(static_cast<uint8_t>(telemetryMode));
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

    if (strcmp(command, "ZERO") == 0 || strcmp(command, "HOME") == 0)
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

    if (strcmp(key, "MAXPWM") == 0)
    {
        if (!isfinite(value))
        {
            Serial.println("ERR MAXPWM_INVALID");
            return;
        }
        maximumReturnPWM = constrain(value, 0.0f, MAX_PWM_COMMAND);
        Serial.print("ACK MAXPWM=");
        Serial.println(maximumReturnPWM, 3);
        return;
    }

    if (strcmp(key, "HOMEPAD") == 0)
    {
        if (!isfinite(value))
        {
            Serial.println("ERR HOMEPAD_INVALID");
            return;
        }
        long requestedPadding = lroundf(value);
        if (requestedPadding < 0) requestedPadding = 0;
        homePaddingPulses = requestedPadding;
        positionSettledLatch = false;
        positionSettleSamples = 0;
        Serial.print("ACK HOMEPAD=");
        Serial.println(homePaddingPulses);
        return;
    }

    if (strcmp(key, "MINPWM") == 0)
    {
        if (!isfinite(value))
        {
            Serial.println("ERR MINPWM_INVALID");
            return;
        }
        minimumDrivePWM = constrain(value, 0.0f, MAX_PWM_COMMAND);
        Serial.print("ACK MINPWM=");
        Serial.println(minimumDrivePWM, 3);
        return;
    }

    if (strcmp(key, "SKP") == 0)
    {
        if (!isfinite(value) || value < MIN_POSITION_KP || value > MAX_POSITION_KP)
        {
            Serial.println("ERR SKP_OUT_OF_RANGE");
            return;
        }
        setupKp = value;
        Serial.print("ACK SKP=");
        Serial.println(setupKp, 6);
        return;
    }

    if (strcmp(key, "SKI") == 0)
    {
        if (!isfinite(value) || value < MIN_POSITION_KI || value > MAX_POSITION_KI)
        {
            Serial.println("ERR SKI_OUT_OF_RANGE");
            return;
        }
        setupKi = value;
        Serial.print("ACK SKI=");
        Serial.println(setupKi, 6);
        return;
    }

    if (strcmp(key, "SKD") == 0)
    {
        if (!isfinite(value) || value < MIN_POSITION_KD || value > MAX_POSITION_KD)
        {
            Serial.println("ERR SKD_OUT_OF_RANGE");
            return;
        }
        setupKd = value;
        Serial.print("ACK SKD=");
        Serial.println(setupKd, 6);
        return;
    }

    if (strcmp(key, "POS") == 0)
    {
        if (!homed)
        {
            Serial.println("ERR NOT_HOMED");
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
        const char incoming = static_cast<char>(Serial.read());
        if (incoming == '\r') continue;

        if (incoming == '\n')
        {
            inputBuffer[inputIndex] = '\0';
            if (inputIndex > 0) processSerialCommand(inputBuffer);
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

void printTelemetry(uint32_t controlTimeMs, long currentCount, float currentPositionDeg)
{
    // Compact 100 Hz trace. Short field names keep the stream comfortably
    // below the 115200-baud link budget while preserving every control sample.
    // U=controller ms, T=trial, M=mode, H=homed, P=position, G=target,
    // E=position error, X=error counts, R=output RPM, C=encoder count, W=PWM.
    Serial.print("F U="); Serial.print(controlTimeMs);
    Serial.print(" T="); Serial.print(trialNumber);
    Serial.print(" M="); Serial.print(static_cast<uint8_t>(mode));
    Serial.print(" H="); Serial.print(homed ? 1 : 0);
    Serial.print(" P="); Serial.print(currentPositionDeg, 3);
    Serial.print(" G="); Serial.print(targetPositionDeg, 3);
    Serial.print(" E="); Serial.print(positionErrorDeg, 3);
    Serial.print(" X="); Serial.print(targetEncoderCount() - currentCount);
    Serial.print(" R="); Serial.print(measuredOutputRPM, 3);
    Serial.print(" C="); Serial.print(currentCount);
    Serial.print(" W="); Serial.println(appliedMotorCommand, 1);
}

// ============================================================================
// STARTUP / MAIN LOOP
// ============================================================================

void setup()
{
    Serial.begin(115200);

    pinMode(ENC_A_PIN, INPUT_PULLUP);
    pinMode(ENC_B_PIN, INPUT_PULLUP);
    pinMode(MOTOR_IN1_PIN, OUTPUT);
    pinMode(MOTOR_IN2_PIN, OUTPUT);

    setMotorCoast();

    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(ENC_A_PIN)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B_PIN));

    attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);

    previousEncoderCount = readEncoderAtomic();
    previousControlTime = millis();

    printIdentity();
    Serial.println(
        "Commands: IDENTIFY, STATUS, CALIBRATE, ZERO, HOLD, STOP, "
        "KP=<>, KI=<>, KD=<>, SKP=<>, SKI=<>, SKD=<>, MAXPWM=<>, "
        "HOMEPAD=<pulses>, MINPWM=<>, POS=<>, TEST=<deg>, TRIAL=<>, TELEM=0|1|2"
    );

    if (persistentHomeValid())
    {
        restoreCurrentPositionAsHomeAfterReset();
    }
    else
    {
        beginCalibration();
    }

    printStatus();
}

void loop()
{
    readSerialCommands();

    const uint32_t now = millis();
    if (now - previousControlTime < CONTROL_INTERVAL_MS) return;

    const float dt = static_cast<float>(now - previousControlTime) / 1000.0f;
    previousControlTime = now;

    // EncoderCount itself is maintained asynchronously on every A/B edge.
    // This atomic snapshot is converted to arm position on every 10 ms control
    // tick, so control and logged FAST position samples share the same 100 Hz
    // source sample.
    const long currentCount = readEncoderAtomic();
    const long deltaCount = currentCount - previousEncoderCount;
    previousEncoderCount = currentCount;

    measuredMotorRPM =
        ENCODER_DIRECTION_SIGN *
        (static_cast<float>(deltaCount) / COUNTS_PER_MOTOR_REV) *
        (60.0f / dt);

    rawOutputRPM = measuredMotorRPM / GEAR_RATIO;
    measuredOutputRPM =
        (1.0f - OUTPUT_RPM_FILTER_ALPHA) * measuredOutputRPM +
        OUTPUT_RPM_FILTER_ALPHA * rawOutputRPM;

    const float currentPositionDeg =
        homed ? countsToPositionDeg(currentCount) : 0.0f;

    positionErrorDeg =
        homed ? (targetPositionDeg - currentPositionDeg) : 0.0f;

    if (mode == Mode::CALIBRATING)
    {
        updateCalibration(currentCount, deltaCount, now);
    }
    else if (mode == Mode::STOPPED || mode == Mode::FAULT || !homed)
    {
        setMotorCoast();
    }
    else
    {
        updateTestState(now);

        const bool settledAndMotorOff =
            updatePositionSettledLatch(currentCount, deltaCount);

        if (!settledAndMotorOff)
        {
            const bool useSetupController =
                mode == Mode::TEST_OUT || mode == Mode::TEST_DWELL;

            const float positionVelocityDegPerSec = measuredOutputRPM * 6.0f;

            const float desiredPWM = calculateDirectPositionPWM(
                positionErrorDeg,
                positionVelocityDegPerSec,
                dt,
                useSetupController
            );

            applyClosedLoopMotorCommand(desiredPWM);
        }
    }

    static uint8_t fastTelemetryDivider = 0;
    static uint8_t humanTelemetryDivider = 0;

    if (telemetryMode == TelemetryMode::FAST)
    {
        if (++fastTelemetryDivider >= TELEMETRY_DIVIDER)
        {
            fastTelemetryDivider = 0;
            printTelemetry(now, currentCount, currentPositionDeg);
        }
    }
    else if (telemetryMode == TelemetryMode::HUMAN)
    {
        const bool worthShowing =
            !positionSettledLatch &&
            (fabsf(positionErrorDeg) >= 0.9f ||
             fabsf(measuredOutputRPM) >= 1.0f ||
             fabsf(appliedMotorCommand) >= 1.0f);

        if (worthShowing && ++humanTelemetryDivider >= HUMAN_TELEMETRY_DIVIDER)
        {
            humanTelemetryDivider = 0;
            Serial.print("LIVE Pos="); Serial.print(currentPositionDeg, 2);
            Serial.print("deg Err="); Serial.print(positionErrorDeg, 2);
            Serial.print("deg RPM="); Serial.print(measuredOutputRPM, 1);
            Serial.print(" PWM="); Serial.print(appliedMotorCommand, 0);
            Serial.print(" Mode="); Serial.println(static_cast<uint8_t>(mode));
        }

        if (!worthShowing) humanTelemetryDivider = 0;
    }
}
