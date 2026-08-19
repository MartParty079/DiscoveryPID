# DiscoveryPID

Arduino Mega 2560 position-control testbed with JMP Bayesian PID optimization.

## Latest supported code

The repository default branch, `main`, is the current supported **Direct PID / 300-degree Bayesian study**.

- firmware: `DiscoveryPID direct-position-pid-v3.4-300deg-10xrange 2026-08-18`
- default/latest branch: `main`
- test displacement: **300 deg**
- Kp search: **0.5 to 1000**
- Ki search: **0 to 500**
- Kd search: **0 to 200**

The scored return controller is a single standard position PID that outputs PWM directly:

```text
position error -> Kp + Ki + Kd -> PWM -> DRV8871 -> motor
```

There is no inner speed PI in the scored return path.

## Recommended terminal launcher

The easiest way to use the project is now:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
.\DiscoveryPID.ps1
```

The interactive menu can:

- switch/update to the latest supported code
- switch between code versions / Git branches
- switch or add GitHub forks/remotes
- choose and remember the serial port
- build + upload firmware
- force fresh homing/calibration
- launch JMP
- run the small smoke test
- run the full 300-degree physical control test
- show firmware, branch, fork, port, and Git status

Pressing **Enter** at the main menu selects **Use latest code**. It does not move the motor.

Direct commands are also available:

```powershell
.\DiscoveryPID.ps1 latest
.\DiscoveryPID.ps1 build
.\DiscoveryPID.ps1 home
.\DiscoveryPID.ps1 jmp
.\DiscoveryPID.ps1 smoke
.\DiscoveryPID.ps1 control
.\DiscoveryPID.ps1 branch
.\DiscoveryPID.ps1 fork
.\DiscoveryPID.ps1 status
```

For the full workflow:

```powershell
.\DiscoveryPID.ps1 full
```

That performs:

```text
latest code -> build/upload -> home -> launch JMP
```

Local choices such as active fork and COM port are stored under `.git` and are not committed, so collaborators can use different machines without editing tracked files.

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

## Current experiment defaults

- return PWM ceiling: **225**
- minimum nonzero drive PWM: **60**
- home window: **+/-2 encoder counts**
- prep timeout: **45 s**
- return timeout: **45 s**
- Python overall timeout: **60 s**

The initial displacement uses the best valid completed PID from the current JMP session. Run 1 falls back to its current candidate PID.

## Hardware

- Arduino Mega 2560
- Adafruit 4416 geared DC motor with quadrature encoder
- DRV8871 motor driver
- encoder A/B: pins 2/3
- motor IN1/IN2: pins 9/10
- serial: 115200 baud
- default port: COM7

The firmware counts all valid A/B transitions (x4 decoding), so the internal encoder scale is 56 transitions per motor revolution.

## Repository layout

- `DiscoveryPID.ps1` - interactive project launcher and direct CLI commands
- `config/discoverypid.json` - tracked latest-code/default profile metadata
- `src/main.cpp` - firmware
- `tools/run_position_trial.py` - one physical JMP trial + scoring
- `tools/home_position.py` - forced fresh homing/calibration
- `jmp/DiscoveryPID_Position_V2.jsl` - current Bayesian study template
- `scripts/position_v2/` - low-level build/home/test/JMP helpers
- `docs/position_hold/README.md` - protocol and control details

## Collaboration

Use `DiscoveryPID.ps1` for normal work. The launcher refuses code-version switching when the working tree has uncommitted changes and uses fast-forward-only pulls, reducing accidental merge damage.

Use **Switch code version / branch** to move between code lines and **Switch/add GitHub fork / remote** to collaborate from another fork. Machine-specific checkout paths, COM ports, and active fork choices do not need to be committed.

Generated study data is intentionally ignored by Git.
