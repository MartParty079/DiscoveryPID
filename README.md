# DiscoveryPID barebones

Branch: `barebones-pid-jsl-bayesopt`.

Only three active files matter:

- `src/main.cpp` — 100 Hz pure positional PID firmware.
- `tools/run_position_trial.py` — serial bridge that returns only settling time and overshoot.
- `jmp/DiscoveryPID_Position_V2.jsl` — JMP Bayesian Optimization over Kp, Ki, Kd in 0–3.

Each JSL run creates a new experiment folder under `data/logs`, named
`DiscoveryPID_YYYYMMDD_HHMMSS` using the PC's local experiment-start time.
All seed and optimization trials from that run save their CSVs in that folder,
with filenames `DiscoveryPID_YYYYMMDD_HHMMSS.csv` using each trial's start time.
Numeric suffixes prevent collisions for both folders and files. JSL prints the
experiment folder path in its log. Direct Python trial invocations without the
optional experiment-directory argument continue saving directly in `data/logs`.
Each CSV includes all received Arduino serial lines: startup messages,
acknowledgments, positioning, the measured return, and Home recovery.
Each row includes the PC receive timestamp,
elapsed seconds, tested KP/KI/KD, and the full raw serial line. CSV quoting keeps
commas in Arduino messages inside the raw-line cell. Rows are flushed as they
arrive, and any final partial line is saved on exit. The worker still prints only
`settling_time,overshoot` to stdout for JMP.

The candidate controller is `u = Kp*error + Ki*integral - Kd*position_velocity`, PWM limited to ±225. Every nonzero command has a minimum PWM magnitude of 37, preserving direction; zero still stops the motor. This minimum applies to trial, setup, and recovery motion. The serial `pwm` field reports the actual signed PWM applied after rounding and limits. The fixed 300° setup and recovery controller is PID 1.35/0.4/0.1. Overshoot never ends a run; it is measured up to 300°. A return ends when it is stationary within ±2 encoder counts for five 100 Hz samples or at the 5 s cutoff.

Before flashing/resetting, put the arm physically at Home. Each trial re-zeros at Home after the previous fixed-PID recovery.

Build/flash with PlatformIO, then open `jmp/DiscoveryPID_Position_V2.jsl` in JMP. Edit the three path/port lines at the top only if your local paths change.
