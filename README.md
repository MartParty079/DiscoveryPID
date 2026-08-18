# DiscoveryPID

Arduino Mega 2560 position-control testbed with JMP Bayesian PID optimization.

## Current position-control stack

The active return controller is a **single standard position PID that outputs PWM directly**:

```text
position error -> Kp + Ki + Kd -> PWM -> DRV8871 -> motor
```

There is no inner speed PI in the scored return path.

Current experiment defaults:

- test displacement: **300 deg**
- return PWM ceiling: **225**
- minimum nonzero drive PWM: **60**
- home window: **+/-2 encoder counts**
- Kp search: **0.5 to 100**
- Ki search: **0 to 50**
- Kd search: **0 to 20**
- prep timeout: **45 s**
- return timeout: **45 s**

The initial displacement uses the best valid completed PID from the current JMP session. Run 1 falls back to its current candidate PID.

## Settling-time definition

The optimization metric intentionally excludes displacement time.

```text
move to 300 deg
settle/dwell at displacement
EVENT TEST_RETURN       <- timer starts
candidate PID returns
enter +/-2 encoder counts of home
                         <- timer stops
```

If a trial does not produce a valid return measurement, JMP shows a warning, saves a finite penalty row, and continues. Invalid rows cannot become the best setup PID.

## Quick start

From PowerShell in the repository:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

& ".\scripts\position_v2\build_upload.ps1"
& ".\scripts\position_v2\home.ps1"
& ".\scripts\position_v2\open_jmp.ps1"
```

In JMP, press `Ctrl+R`, then start the study.

For a smaller hardware check before a 300-degree run:

```powershell
& ".\scripts\position_v2\smoke_test.ps1"
```

## Hardware

- Arduino Mega 2560
- Adafruit 4416 geared DC motor with quadrature encoder
- DRV8871 motor driver
- encoder A/B: pins 2/3
- motor IN1/IN2: pins 9/10
- serial: 115200 baud
- default port in scripts/JSL: COM7

The firmware counts all valid A/B transitions (x4 decoding), so the internal encoder scale is 56 transitions per motor revolution.

## Repository layout

- `src/main.cpp` - firmware
- `tools/run_position_trial.py` - one physical JMP trial + scoring
- `tools/home_position.py` - forced fresh homing/calibration
- `jmp/DiscoveryPID_Position_V2.jsl` - current Bayesian study template
- `scripts/position_v2/` - build, home, smoke-test, and JMP helpers
- `docs/position_hold/README.md` - protocol and control details

## Collaboration

Use the PowerShell helpers instead of editing machine-specific paths into the tracked JSL. `open_jmp.ps1` creates a temporary runtime copy configured for the current checkout.

Current work should be done on feature branches and proposed through pull requests. Generated study data is intentionally ignored by Git.
