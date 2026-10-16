# DiscoveryPID barebones

Branch: `barebones-pid-jsl-bayesopt`.

Only three active files matter:

- `src/main.cpp` — 100 Hz pure positional PID firmware.
- `tools/run_position_trial.py` — serial bridge that returns only settling time and overshoot.
- `jmp/DiscoveryPID_Position_V2.jsl` — JMP Bayesian Optimization over Kp, Ki, Kd in 0–3.

The candidate controller is only `u = Kp*error + Ki*integral - Kd*position_velocity`, PWM limited to ±225 with no floor, taper, deadband, gain scheduling, or near-home logic. The fixed 300° setup and recovery controller is PID 1.35/0.4/0.1. Overshoot never ends a run; it is measured up to 300°. A return ends when it is stationary within ±2 encoder counts for five 100 Hz samples or at the 5 s cutoff.

Before flashing/resetting, put the arm physically at Home. Each trial re-zeros at Home after the previous fixed-PID recovery.

Build/flash with PlatformIO, then open `jmp/DiscoveryPID_Position_V2.jsl` in JMP. Edit the three path/port lines at the top only if your local paths change.
