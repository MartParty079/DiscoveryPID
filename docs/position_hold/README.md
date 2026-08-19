# Position hold and JMP Bayesian optimization

## Control architecture

The scored return uses one direct position PID:

```text
error_deg = target_deg - measured_deg
PWM = Kp*error + Ki*integral(error) - Kd*measured_position_velocity
```

Derivative is applied to measured position velocity to avoid derivative kick.

The firmware includes:

- conditional anti-windup
- integral separation outside +/-10 deg
- integral reset on target crossing
- configurable minimum nonzero PWM floor
- configurable encoder-count home padding
- reversal interlock
- persistent safe-home marker across normal USB resets

## Encoder scale

The ISR counts both channels on every valid transition (x4 quadrature decoding):

- motor encoder: 56 transitions/rev internally
- gearbox ratio: 30613 / 1500
- output counts/rev: about 1142.9
- output resolution: about 0.315 deg/count

## Homing

Run:

```powershell
& ".\scripts\position_v2\home.ps1"
```

The helper explicitly sends `CALIBRATE`, which clears the persisted home marker. Move the output left/right, return to the desired zero position, and hold still. The helper exits after `EVENT HOMED` and frees the serial port.

Calibration requires sufficient encoder span, at least one reversal, and about 1.5 s stillness at the selected home.

## Physical test state machine

1. `TEST_OUT`: move from home to `testOffsetDeg` using the setup PID.
2. `TEST_DWELL`: hold the displaced target long enough to be repeatable.
3. `EVENT TEST_RETURN`: reset controller state and switch to the candidate PID; **settling timer starts here**.
4. `TEST_RETURN`: return to zero.
5. Settling time ends on the first return sample with `abs(ErrCounts) <= homePaddingPulses`.
6. `EVENT TEST_COMPLETE`: safe home marker is restored.

Displacement and dwell time are not part of the reported settling time.

## Current JMP study

Defaults in `jmp/DiscoveryPID_Position_V2.jsl`:

```text
testOffsetDeg = 300
returnMaxPWM = 225
homePaddingPulses = 2
minimumDrivePWM = 60

Kp: 0.5 .. 100
Ki: 0   .. 50
Kd: 0   .. 20
```

Seed points span the enlarged design region. After valid completed runs exist, the lowest-score valid PID is reused for the displacement leg. A timeout or missing-settling row is excluded from setup-PID selection.

## Trial validity and warnings

The Python worker always writes a finite diagnostic result row. JMP warns and continues when a run is invalid, including `TEST_PREP_TIMEOUT`, no `EVENT TEST_RETURN`, no return samples, missing settling time, or return timeout before entering the home window.

These rows retain a penalty score for traceability but are not eligible to become the best setup PID.

## Serial commands

```text
IDENTIFY
STATUS
CALIBRATE
ZERO
HOLD
STOP

KP=<value>
KI=<value>
KD=<value>
SKP=<value>
SKI=<value>
SKD=<value>
MAXPWM=<value>
HOMEPAD=<counts>
MINPWM=<value>
POS=<deg>
TEST=<deg>
TRIAL=<n>
TELEM=0|1|2
```

`KP/KI/KD` are the candidate return gains. `SKP/SKI/SKD` are the setup/displacement gains supplied by JMP.

## Generated data

Each JSL launch starts a fresh session and writes timestamped files under generated `data/position_autotune_*` folders. These outputs are ignored by Git so collaborators do not commit experimental run data accidentally.
