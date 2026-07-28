/*
  Adafruit 4416 PI Motor Speed Controller
  =======================================

  This is the active DiscoveryPID controller firmware.

  The program:
    1. Reads both quadrature encoder channels with interrupts.
    2. Converts encoder movement into motor-shaft RPM.
    3. Divides by the gearbox ratio to calculate output-shaft RPM.
    4. Uses a PI controller to make output RPM follow targetRPM.
    5. Sends signed direction and PWM commands to the motor driver.
    6. Prints operating data for testing and future JMP Bayesian optimization.

  Hardware mapping:
    Encoder channel A -> digital pin 2
    Encoder channel B -> digital pin 3
    Direction output  -> digital pin 9
    PWM output        -> digital pin 10

  Serial input:
    Send a number at 115200 baud to change the target output-shaft RPM.
    Example: 100

  Important:
    - GEAR_RATIO and CPR_X4 must match the physical drivetrain and encoder.
    - KP and KI are the PI gains that will be tuned during testing.
    - targetRPM and rpmOut refer to the gearbox OUTPUT shaft.
    - motorRPM refers to the motor shaft before the gearbox reduction.
*/

#include <Arduino.h>

// =====================================================
// HARDWARE PIN ASSIGNMENTS
// =====================================================
// uint8_t is appropriate because Arduino pin numbers are small nonnegative
// integers. Keeping the assignments together makes hardware changes simple.
const uint8_t ENC_A = 2;
const uint8_t ENC_B = 3;
const uint8_t DIR_PIN = 9;
const uint8_t PWM_PIN = 10;

// =====================================================
// DRIVETRAIN AND ENCODER CALIBRATION
// =====================================================
// Motor speed is divided by this ratio to obtain output-shaft speed.
// A ratio of 20.4 means the motor turns 20.4 revolutions for every output
// shaft revolution.
const float GEAR_RATIO = 20.4f;

// Encoder counts per MOTOR revolution when using full x4 quadrature decoding.
// The encoder produces 7 cycles per channel, and both rising and falling edges
// on both channels are counted: 7 cycles x 4 edges = 28 counts/revolution.
const int CPR_X4 = 28;

// =====================================================
// PI CONTROLLER GAINS
// =====================================================
// KP controls the immediate response to current speed error.
// KI controls the correction based on accumulated speed error over time.
// These values are expected to be tuned using controlled experiments and JMP.
const float KP = 2.0f;
const float KI = 5.0f;

// Controller sample period in milliseconds.
// 20 ms corresponds to a nominal control frequency of 50 Hz.
const uint32_t Ts_ms = 20;

// =====================================================
// ENCODER STATE SHARED WITH INTERRUPTS
// =====================================================
// These variables are modified inside the interrupt service routine, so they
// are marked volatile to prevent the compiler from assuming they stay constant.
volatile long encCount = 0;
volatile uint8_t prevState = 0;

// =====================================================
// CONTROLLER AND MEASUREMENT STATE
// =====================================================
// Requested output-shaft speed in RPM. This can be changed through Serial.
float targetRPM = 100.0f;

// Most recently calculated output-shaft speed in RPM.
float rpmOut = 0.0f;

// Accumulated error used by the integral portion of the PI controller.
float integ = 0.0f;

// Encoder count captured during the previous controller update.
long lastCount = 0;

// Time of the previous controller update in milliseconds since startup.
uint32_t lastT = 0;

// =====================================================
// QUADRATURE ENCODER LOOKUP TABLE
// =====================================================
// Each encoder state is represented by two bits: AB.
// Combining the previous AB state and current AB state creates a 4-bit index:
//
//   index = previous_AB << 2 | current_AB
//
// The lookup table returns:
//   +1 for one valid direction step
//   -1 for one valid step in the opposite direction
//    0 for no movement or an invalid/noisy transition
//
// This compact method avoids multiple conditional branches inside the ISR.
const int8_t qem[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// =====================================================
// ENCODER INTERRUPT SERVICE ROUTINE
// =====================================================
// This routine runs whenever either encoder channel changes state. It must stay
// short because the main program is paused while the interrupt is executing.
void encoderISR()
{
    // Read both encoder channels and combine them into a two-bit AB state.
    uint8_t currentState =
        (static_cast<uint8_t>(digitalRead(ENC_A)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B));

    // Build the lookup-table index from the previous and current AB states.
    uint8_t tableIndex = (prevState << 2) | currentState;

    // Add the signed movement represented by this encoder transition.
    encCount += qem[tableIndex];

    // Save the current state for the next encoder transition.
    prevState = currentState;
}

// =====================================================
// MOTOR DRIVER OUTPUT
// =====================================================
// The PI controller creates a signed command u:
//   positive u -> one direction
//   negative u -> opposite direction
//   abs(u)     -> PWM duty command from 0 to 255
void setMotor(float u)
{
    // Select direction from the sign of the controller output.
    bool direction = (u >= 0.0f);

    // Convert the controller output into an Arduino 8-bit PWM value.
    // fabs() removes the sign because direction is handled separately.
    int pwm = constrain(static_cast<int>(fabs(u)), 0, 255);

    digitalWrite(DIR_PIN, direction ? HIGH : LOW);
    analogWrite(PWM_PIN, pwm);
}

// =====================================================
// ARDUINO STARTUP
// =====================================================
// setup() runs once after power-up or reset.
void setup()
{
    // Start serial communication for target commands and test-data output.
    Serial.begin(115200);

    // Configure both quadrature encoder signals as digital inputs.
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);

    // Configure the motor-driver control signals as outputs.
    pinMode(DIR_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    // Read the encoder's initial AB state before enabling interrupts. This
    // prevents the first transition from being compared with an assumed state.
    prevState =
        (static_cast<uint8_t>(digitalRead(ENC_A)) << 1) |
        static_cast<uint8_t>(digitalRead(ENC_B));

    // Run encoderISR() on both rising and falling edges of both channels.
    // This provides full x4 quadrature decoding.
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), encoderISR, CHANGE);

    // Initialize controller timing so the first sample uses a valid elapsed time.
    lastT = millis();

    Serial.println("DiscoveryPID controller ready");
    Serial.println("Send target output RPM as a number");
}

// =====================================================
// CONTINUOUS CONTROL LOOP
// =====================================================
// loop() repeatedly checks for serial commands and runs the PI controller at
// approximately the requested sample period.
void loop()
{
    // -------------------------------------------------
    // READ A NEW OUTPUT-SHAFT RPM TARGET
    // -------------------------------------------------
    // parseFloat() reads the next number received through the Serial Monitor.
    // The previous target remains active until another value is sent.
    if (Serial.available() > 0)
    {
        targetRPM = Serial.parseFloat();

        Serial.print("New target output RPM: ");
        Serial.println(targetRPM);
    }

    // -------------------------------------------------
    // RUN THE TIMED CONTROL UPDATE
    // -------------------------------------------------
    uint32_t now = millis();

    if (now - lastT >= Ts_ms)
    {
        // Use the actual elapsed time rather than assuming it is exactly Ts_ms.
        // This reduces error caused by small variations in loop execution time.
        float dt = (now - lastT) / 1000.0f;
        lastT = now;

        // ---------------------------------------------
        // COPY THE ENCODER COUNT SAFELY
        // ---------------------------------------------
        // encCount can change during an interrupt. Interrupts are disabled only
        // long enough to copy it, avoiding a partially updated long integer.
        noInterrupts();
        long currentCount = encCount;
        interrupts();

        // Count movement that occurred during this controller sample.
        long deltaCount = currentCount - lastCount;
        lastCount = currentCount;

        // ---------------------------------------------
        // CALCULATE MOTOR AND OUTPUT-SHAFT RPM
        // ---------------------------------------------
        // Convert encoder counts/sample into motor revolutions/sample, then
        // multiply by 60/dt to obtain motor revolutions per minute.
        float motorRPM =
            (static_cast<float>(deltaCount) / CPR_X4) * (60.0f / dt);

        // Apply the gearbox reduction to calculate output-shaft RPM.
        rpmOut = motorRPM / GEAR_RATIO;

        // ---------------------------------------------
        // CALCULATE THE PI CONTROLLER COMMAND
        // ---------------------------------------------
        // Positive error means the output shaft is moving slower than requested.
        float error = targetRPM - rpmOut;

        // Calculate a candidate integral value but do not immediately accept it.
        float proposedIntegral = integ + error * dt;

        // Calculate a preliminary controller command using the candidate integral.
        float unsaturatedCommand =
            (KP * error) +
            (KI * proposedIntegral);

        // Conditional-integration anti-windup:
        // Only store additional integral error when the proposed controller output
        // is not saturated. This reduces integral buildup beyond motor capability.
        if (unsaturatedCommand < 255.0f && unsaturatedCommand > -255.0f)
        {
            integ = proposedIntegral;
        }

        // Recalculate the PI command using the accepted integral state.
        float controlCommand =
            (KP * error) +
            (KI * integ);

        // Limit the final command to the Arduino PWM range.
        controlCommand = constrain(controlCommand, -255.0f, 255.0f);

        // Apply direction and PWM to the motor driver.
        setMotor(controlCommand);

        // ---------------------------------------------
        // PRINT TEST DATA
        // ---------------------------------------------
        // The controller runs nominally at 50 Hz, but printing every fifth update
        // reduces serial traffic to approximately 10 records per second.
        // These fields can later be converted into CSV columns for JMP analysis.
        static int printDivider = 0;

        if (++printDivider >= 5)
        {
            printDivider = 0;

            Serial.print("Target=");
            Serial.print(targetRPM);

            Serial.print(" OutRPM=");
            Serial.print(rpmOut, 1);

            Serial.print(" MotorRPM=");
            Serial.print(motorRPM, 0);

            Serial.print(" Counts=");
            Serial.print(currentCount);

            Serial.print(" dC=");
            Serial.print(deltaCount);

            Serial.print(" PWM=");
            Serial.println(controlCommand);
        }
    }
}
