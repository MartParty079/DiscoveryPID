#include <Arduino.h>

// --- EDIT HERE: wiring, motor/encoder, and experiment settings ---
// Replacing a motor: check encoder counts, gear ratio, direction, PWM limit,
// and fixed setup/recovery gains below. Retune the trial gains for the new motor.
// Changing boards: also update platformio.ini and check pin capabilities.
constexpr byte EA = 2;  // Encoder channel A; must support attachInterrupt().
constexpr byte EB = 3;  // Encoder channel B; must support attachInterrupt().
constexpr byte M1 = 9;  // Motor driver input for negative motion; PWM-capable pin.
constexpr byte M2 = 10; // Motor driver input for positive motion; PWM-capable pin.

// CHANGE for a different encoder or gearbox. This calculation assumes the
// encoder measures the motor shaft, before the gearbox. Both edges of both
// channels are counted: use decoded counts/revolution, not an unconverted PPR.
constexpr float ENCODER_COUNTS_PER_REV = 56.0f;
constexpr float GEAR_RATIO = 30613.0f / 1500.0f; // Motor turns per output turn.
constexpr float DEG = 360.0f / (ENCODER_COUNTS_PER_REV * GEAR_RATIO);
// DEG converts one encoder count into degrees at the controlled output shaft.
// If the encoder is on the output shaft, use its counts/rev and a ratio of 1.
// Positive motor output must increase encoder count. If direction is wrong,
// reverse either the motor direction wiring or encoder A/B order, then verify.

constexpr float MAXPWM = 225; // CHANGE output cap for the motor/driver (0-255).
// Fixed gains move to the start angle and recover Home after each trial.
// CHANGE these for a different motor, load, or supply; JMP does not tune them.
constexpr float SKP = 1.35f; // Position error correction.
constexpr float SKI = .4f;   // Correction from accumulated position error.
constexpr float SKD = .1f;   // Damping from measured movement speed.
constexpr float INTEGRAL_LIMIT = 2000.0f; // Maximum accumulated error, deg*s.

constexpr float TEST_ANGLE = 300;        // CHANGE trial start angle, in degrees.
constexpr float START_ERROR = 1.5f;      // Allowed error at the start angle, deg.
constexpr float START_SPEED = 12;        // Allowed speed there, deg/s.
constexpr uint32_t START_HOLD_MS = 850;  // Stay within both limits this long.
constexpr long HOME_COUNTS = 2;          // Allowed distance from Home, counts.
constexpr byte HOME_SAMPLES = 5;         // Unmoving samples before recovery ends.
constexpr uint32_t CONTROL_PERIOD_MS = 10; // Minimum update interval; 10 = 100 Hz.
constexpr unsigned long SERIAL_BAUD = 115200; // Match Python and platformio.ini.
constexpr unsigned long SERIAL_TIMEOUT_MS = 10; // Wait for a serial line, ms.
// If changing the angle or timing, review tools/run_position_trial.py:
// it measures negative-position overshoot, uses 5 s return/50 s overall limits,
// and waits up to 20 s for recovery. Its settling rule also uses 2 counts/5 samples.
// Review JMP response limits if the experiment's angle or scoring changes.

// --- Runtime values: updated by the program, not hardware settings ---
volatile long enc = 0;  // Total signed encoder count since startup.
volatile byte prev = 0; // Previous A/B pin state used to detect direction.
// Direction lookup for encoder transitions; normally leave this table alone.
const int8_t q[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// EDIT these defaults for manual trials. Python/JMP overrides them with KP=,
// KI=, KD= before each automated trial; edit the JSL seeds/range for that tuning.
float kp = 1.35f; // Trial return: response to current position error.
float ki = .4f;   // Trial return: response to accumulated position error.
float kd = .1f;   // Trial return: damping from measured speed.
float I = 0;      // Accumulated position error (deg*s); reset for each move.
float target = 0; // Requested output angle relative to Home, in degrees.
float pwm = 0;    // Last applied signed motor command, also sent in telemetry.
long home = 0;    // Encoder count defined as the zero-degree position.
long last = 0;    // Previous control sample's encoder count; used for velocity.
// Modes: 0 = idle, 1 = move to TEST_ANGLE, 2 = trial return, 3 = recovery.
byte mode = 0;      // Which part of the experiment is currently running.
byte stable = 0;    // Consecutive stationary samples during Home recovery.
uint32_t tick = 0;  // Time of the previous control update, in milliseconds.
uint32_t at300 = 0; // When start-angle settling began; 0 means not settling yet.

// --- Encoder access ---
// Called automatically on every encoder edge; add or subtract a position count.
void isr() {
  // s is the current two-bit A/B state (0-3).
  byte s = (digitalRead(EA) << 1) | digitalRead(EB);
  enc += q[(prev << 2) | s];
  prev = s;
}

// Return a consistent copy of the encoder position, even while it is changing.
long count() {
  // A multi-byte encoder count must not change partway through an AVR read.
  noInterrupts();
  long x = enc; // Local snapshot, protected from interruption while being copied.
  interrupts();
  return x;
}

// --- Motor output and positional PID ---
// Apply u: its sign chooses direction and its size chooses motor drive strength.
void drive(float u) {
  int p = constrain((int)lroundf(fabsf(u)), 0, (int)MAXPWM); // Rounded PWM magnitude.
  pwm = u > 0 ? p : u < 0 ? -p : 0;

  // Clear the opposite direction output before applying the requested PWM.
  if (pwm > 0) {
    analogWrite(M1, 0);
    analogWrite(M2, p);
  } else if (pwm < 0) {
    analogWrite(M2, 0);
    analogWrite(M1, p);
  } else {
    analogWrite(M1, 0);
    analogWrite(M2, 0);
  }
}

// Calculate motor effort from error and speed; this function does not drive pins.
// e = target minus position (deg), v = speed (deg/s), dt = elapsed seconds.
// setup = true selects fixed gains; false selects the trial return gains.
float pid(float e, float v, float dt, bool setup) {
  // Use fixed gains for setup/recovery and candidate gains for the trial.
  // P, K, D are the selected proportional, integral, and derivative gains.
  float P = setup ? SKP : kp, K = setup ? SKI : ki, D = setup ? SKD : kd;
  I = constrain(I + e * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  // Derivative acts on measured velocity; bound the final motor command.
  return constrain(P * e + K * I - D * v, -MAXPWM, MAXPWM);
}

// Call the present position Home. This does not physically move or find Home.
void zero() {
  // Define the current encoder position as Home and stop the controller.
  home = count();
  target = 0;
  I = 0;
  mode = 0;
  drive(0);
}

// --- Newline-delimited serial commands ---
// Read one command string s and perform it; recognized commands reply "OK".
// STOP disables drive; HOME moves to the saved zero; ZERO changes that zero.
void command(String s) {
  s.trim();
  if (s == "ZERO") {
    zero();
    Serial.println("OK");
  } else if (s == "STOP") {
    mode = 0;
    drive(0);
    Serial.println("OK");
  } else if (s == "TEST") {
    // First position the arm with fixed gains, then test the return gains.
    target = TEST_ANGLE;
    I = 0;
    mode = 1;
    at300 = 0;
    Serial.println("OK");
  } else if (s == "HOME") {
    // Recover to Home using fixed gains, independent of the trial gains.
    target = 0;
    I = 0;
    mode = 3;
    stable = 0;
    Serial.println("OK");
  } else if (s.startsWith("KP=")) {
    kp = s.substring(3).toFloat();
    Serial.println("OK");
  } else if (s.startsWith("KI=")) {
    ki = s.substring(3).toFloat();
    Serial.println("OK");
  } else if (s.startsWith("KD=")) {
    kd = s.substring(3).toFloat();
    Serial.println("OK");
  }
}

// --- Arduino initialization ---
// Runs once after power-up/reset: prepare pins, serial, and encoder tracking.
// Place the arm at physical Home first: startup defines that position as zero.
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMEOUT_MS);
  pinMode(EA, INPUT_PULLUP);
  pinMode(EB, INPUT_PULLUP);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  // Decode both edges of both encoder channels.
  prev = (digitalRead(EA) << 1) | digitalRead(EB);
  attachInterrupt(digitalPinToInterrupt(EA), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EB), isr, CHANGE);
  last = count();
  zero();
}

// --- Control loop and telemetry ---
// Runs repeatedly: accept commands, measure movement, drive the active move,
// and send measurements to Python. In idle mode, no PID command is applied.
void loop() {
  if (Serial.available()) command(Serial.readStringUntil('\n'));

  // Use the configured update interval and actual elapsed time for calculations.
  uint32_t now = millis(); // Time since startup, in milliseconds.
  if (now - tick < CONTROL_PERIOD_MS) return;
  float dt = (now - tick) / 1000.0f; // Seconds since the last control update.
  tick = now;

  long c = count(); // Current absolute encoder count.
  long dc = c - last; // Count change since the last control update.
  last = c;
  float p = (c - home) * DEG; // Output position relative to Home, in degrees.
  float e = target - p;      // Remaining distance to target, signed degrees.
  float v = dc * DEG / dt;   // Output speed, signed degrees per second.

  if (mode == 1) {
    drive(pid(e, v, dt, true));
    // Wait until position and speed stay within the configured start limits.
    if (fabsf(e) <= START_ERROR && fabsf(v) <= START_SPEED) {
      if (!at300) at300 = now;
      if (now - at300 >= START_HOLD_MS) {
        target = 0;
        I = 0;
        mode = 2;
        // The Python worker uses this timestamp to measure return settling.
        Serial.print("RETURN,");
        Serial.println(now);
      }
    } else at300 = 0;
  } else if (mode == 2) {
    // Keep testing the candidate gains until the worker sends STOP/HOME.
    drive(pid(e, v, dt, false));
  } else if (mode == 3) {
    drive(pid(e, v, dt, true));
    // Count stationary samples near Home, then stop and notify the worker.
    if (labs(c - home) <= HOME_COUNTS && dc == 0) {
      if (++stable >= HOME_SAMPLES) {
        mode = 0;
        drive(0);
        Serial.println("HOME");
      }
    } else stable = 0;
  }

  // Active-mode CSV: D,time_ms,mode,position_deg,error_deg,encoder_count,pwm.
  if (mode) {
    Serial.print("D,");
    Serial.print(now);
    Serial.print(",");
    Serial.print(mode);
    Serial.print(",");
    Serial.print(p, 3);
    Serial.print(",");
    Serial.print(e, 3);
    Serial.print(",");
    Serial.print(c);
    Serial.print(",");
    Serial.println(pwm, 0);
  }
}
