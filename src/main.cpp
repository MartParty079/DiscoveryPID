/*
  DC Motor Speed Controller with PI Feedback
  ------------------------------------------
  This program measures motor speed from a Hall-effect encoder and adjusts
  the motor driver's PWM command to hold a requested shaft speed.

  Main program flow:
    1. Count encoder transitions in an interrupt service routine (ISR).
    2. Convert the number of new counts into measured RPM.
    3. Compare measured RPM with the target RPM.
    4. Calculate a PI correction from proportional and integral error.
    5. Apply motor direction and PWM output.
    6. Print test data for tuning and later JMP analysis.

  Hardware Assumptions:
  ---------------------
  Encoder Hall Sensors:
    HALL_A -> pin 2  (interrupt pin)
    HALL_B -> pin 3  (interrupt pin, currently disabled in software)

  Motor Driver:
    PWM_PIN -> PWM speed input
    DIR_PIN -> direction input

  Serial Commands:
    Send the desired RPM as a number through the Serial Monitor.

    Examples:
      150
      300
      -200

  Important configuration notes:
    - Tune KP and KI for the actual motor, load, and sample interval.
    - Verify PULSES_PER_REV against the encoder datasheet and the selected
      edge-counting method before trusting the reported RPM.
    - The current ISR counts transitions from Hall A only. The full
      two-channel quadrature direction logic is retained below but disabled.
    - The output method is intended for motor drivers with separate PWM and
      direction inputs, such as a DRV8871-style interface.
*/

#include <Arduino.h>

// =====================================================
// PIN DEFINITIONS
// =====================================================
// These constants map the software signals to physical Arduino pins.
// Keeping all pin assignments in one block makes wiring changes easier and
// prevents hard-coded pin numbers from being scattered throughout the code.

const int HALL_A_PIN = 2; // Green encoder wire; active interrupt input.
const int HALL_B_PIN = 3; // White encoder wire; reserved for quadrature decoding.

const int PWM_PIN = 10; // Motor-driver IN2 / PWM speed command.
const int DIR_PIN = 9;  // Motor-driver IN1 / direction command.

// Previous direct motor-input names are retained for reference only.
// const int Motor_IN2 = 10;
// const int Motor_IN1 = 9;

// =====================================================
// ENCODER SETTINGS
// =====================================================
// These values convert interrupt counts into mechanical revolutions.
// They must match both the encoder hardware and the ISR counting method.

// Nominal pulses produced by one encoder channel per revolution.
// Confirm whether this refers to the motor shaft or output shaft.
const int PULSES_PER_REV = 20;

// A full quadrature decoder can count rising and falling edges on both
// channels, producing four counts per encoder pulse. The current ISR only
// watches Hall A, so this multiplier must be verified during calibration.
const int COUNTS_PER_REV = PULSES_PER_REV * 4;

// =====================================================
// PI CONTROLLER SETTINGS
// =====================================================
// KP determines how strongly the controller reacts to the present RPM error.
// KI determines how strongly it reacts to accumulated error over time.
// These are intentionally non-constant so future serial or JMP integration
// can update them during controller optimization experiments.

float KP = 1.2;
float KI = 0.8;

// =====================================================
// CONTROL LOOP SETTINGS
// =====================================================
// Requested period between PI calculations. A 20 ms interval corresponds to
// a nominal 50 Hz controller update rate. The delay currently used in loop()
// can make the effective update rate slower than this requested interval.

const unsigned long CONTROL_INTERVAL_MS = 20;

// =====================================================
// GLOBAL STATE VARIABLES
// =====================================================
// Variables shared between the interrupt routine, control calculation, and
// serial interface are stored here so their values persist between loop calls.

// Updated inside the encoder ISR. "volatile" tells the compiler that this
// value can change unexpectedly outside the normal program flow.
volatile long encoderCount = 0;

// Encoder count saved at the previous control update. The difference between
// this value and the new count gives the movement during one sample period.
long previousEncoderCount = 0;

// Requested and calculated motor speeds in revolutions per minute.
float targetRPM = 150.0;
float measuredRPM = 0.0;

// Running integral of RPM error used by the I portion of the PI controller.
float integralTerm = 0.0;

// Timestamp of the previous controller calculation, measured in milliseconds
// since the Arduino started.
unsigned long lastControlTime = 0;

// =====================================================
// ENCODER INTERRUPT SERVICE ROUTINE
// =====================================================
// This function runs immediately whenever Hall A changes state. ISR code
// should remain short because normal program execution pauses while it runs.
// The present implementation counts speed only and does not detect direction.

void updateEncoder()
{
    // Full quadrature decoding would read both channels and use their relative
    // states to decide whether the shaft moved forward or backward.
    // bool A = digitalRead(HALL_A_PIN);
    // bool B = digitalRead(HALL_B_PIN);

    // Count every detected CHANGE edge on Hall A. This always increments,
    // meaning reverse rotation is not represented by a negative count.
    encoderCount++;

    // Optional full quadrature direction logic, currently disabled.
    // if (A == B)
    // {
    //     encoderCount++;
    // }
    // else
    // {
    //     encoderCount--;
    // }
}

// =====================================================
// MOTOR OUTPUT FUNCTION
// =====================================================
// Converts the signed PI controller command into two hardware signals:
//   - The sign selects motor direction.
//   - The magnitude selects PWM duty cycle and therefore drive effort.

void setMotorOutput(float controlSignal)
{
    // Limit the requested drive command to the usable analogWrite range used
    // by this project. Clamping also prevents extreme PI values from being
    // sent directly to the motor driver.
    controlSignal = constrain(controlSignal, -250, 250);

    // Positive and zero commands select the forward direction. Negative
    // commands select reverse. Reverse control is only reliable when encoder
    // direction sensing is also enabled.
    if (controlSignal >= 0)
    {
        digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIR_PIN, LOW);
    }

    // analogWrite expects a nonnegative PWM magnitude. Direction has already
    // been assigned above, so only the absolute controller output is used.
    int pwmValue = abs((int)controlSignal);

    // Send the PWM command to the motor driver.
    analogWrite(PWM_PIN, pwmValue);
}

// =====================================================
// ARDUINO STARTUP
// =====================================================
// setup() runs once after power-up or reset. It initializes communication,
// configures the I/O pins, and enables the encoder interrupt.

void setup()
{
    // Serial data is used for target commands and test-data output.
    Serial.begin(115200);

    // Encoder input. Hall B remains disabled until full quadrature decoding
    // is enabled in updateEncoder().
    pinMode(HALL_A_PIN, INPUT);
    // pinMode(HALL_B_PIN, INPUT);

    // Motor-driver command outputs.
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    // Previous direct motor-input configuration retained for reference.
    // pinMode(Motor_IN2, OUTPUT);
    // pinMode(Motor_IN1, OUTPUT);

    // Call updateEncoder() on every rising or falling transition of Hall A.
    attachInterrupt(digitalPinToInterrupt(HALL_A_PIN), updateEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(HALL_B_PIN), updateEncoder, CHANGE);

    // Startup messages confirm that the controller is running and explain how
    // to change the commanded speed.
    Serial.println("PI Motor Speed Controller");
    Serial.println("Send target RPM via Serial");
}

// =====================================================
// CONTINUOUS PROGRAM LOOP
// =====================================================
// loop() repeats for as long as the Arduino is powered. It accepts new target
// commands and executes the timed measurement-and-control sequence.

void loop()
{
    // Reduce the amount of serial traffic and CPU activity. Note that this
    // 100 ms blocking delay currently limits how often the 20 ms control test
    // below can actually be reached.
    delay(100);

    // -------------------------------------------------
    // READ A NEW TARGET RPM FROM SERIAL
    // -------------------------------------------------
    // Serial.parseFloat() converts the next numeric serial value into a float.
    // The previous target remains active until another number is received.
    if (Serial.available())
    {
        targetRPM = Serial.parseFloat();

        Serial.print("New Target RPM: ");
        Serial.println(targetRPM);
    }

    // -------------------------------------------------
    // CHECK WHETHER IT IS TIME FOR A CONTROL UPDATE
    // -------------------------------------------------
    // millis() is used instead of an absolute schedule so the controller keeps
    // operating correctly as the Arduino uptime increases.
    unsigned long now = millis();

    if (now - lastControlTime >= CONTROL_INTERVAL_MS)
    {
        // Convert elapsed milliseconds into seconds for RPM and integral math.
        // Using the measured elapsed time compensates for some loop jitter.
        float dt = (now - lastControlTime) / 1000.0;

        // Save the current time as the start point for the next sample.
        lastControlTime = now;

        // -------------------------------------------------
        // COPY THE ENCODER COUNT SAFELY
        // -------------------------------------------------
        // encoderCount can change inside the ISR. Interrupts are disabled only
        // long enough to copy it, preventing a partially updated long integer
        // from being read by the main program.
        noInterrupts();
        long currentCount = encoderCount;
        interrupts();

        // Determine how many encoder counts occurred during this sample.
        long deltaCounts = currentCount - previousEncoderCount;
        previousEncoderCount = currentCount;

        // -------------------------------------------------
        // CONVERT ENCODER COUNTS TO RPM
        // -------------------------------------------------
        // First convert counts into revolutions, then divide by elapsed seconds
        // to obtain revolutions per second, and finally multiply by 60.
        float revs = (float)deltaCounts / COUNTS_PER_REV;
        measuredRPM = (revs / dt) * 60.0;

        // -------------------------------------------------
        // CALCULATE THE PI CONTROLLER OUTPUT
        // -------------------------------------------------
        // Error is positive when the motor is slower than requested and
        // negative when it is faster than requested.
        float error = targetRPM - measuredRPM;

        // Numerically integrate error over time. Multiplying by dt makes the
        // integral less dependent on the exact controller sample interval.
        integralTerm += error * dt;

        // Anti-windup limits stored integral error while the motor command is
        // saturated or the target cannot be reached.
        integralTerm = constrain(integralTerm, -300, 300);

        // PI equation:
        //   output = proportional response + accumulated integral response
        float controlSignal =
            (KP * error) +
            (KI * integralTerm);

        // -------------------------------------------------
        // APPLY THE CONTROLLER COMMAND TO THE MOTOR
        // -------------------------------------------------
        setMotorOutput(controlSignal);

        // -------------------------------------------------
        // PRINT CONTROLLER DATA FOR TESTING AND TUNING
        // -------------------------------------------------
        // These values allow the response to be reviewed in the Serial Monitor
        // and can later be converted to structured CSV data for JMP Bayesian
        // optimization and comparison between KP/KI combinations.
        Serial.print("Target: ");
        Serial.print(targetRPM);

        Serial.print(" RPM | Measured: ");
        Serial.print(measuredRPM);

        Serial.print(" RPM | PWM: ");
        Serial.print(controlSignal);

        Serial.print("Count: ");
        Serial.print(currentCount);

        Serial.print("  Delta: ");
        Serial.print(deltaCounts);

        Serial.println();
    }
}
