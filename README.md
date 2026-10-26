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
`settling_time,overshoot` to stdout for valid measurements. Invalid trials print
`ERROR,status`, which JSL records in a separate Failed Trials table before
continuing the experiment. Failed trials are excluded from optimization. Each CSV also has a
matching `.json` summary containing status, diagnostic detail, setup duration,
return duration, and measured results (null for invalid trials).

Setup has its own 12-second limit to receive `RETURN`; the return then gets its
full 20-second window, independent of time spent positioning. A return that does
not settle scores 20 seconds. A setup timeout, incompatible firmware, or lost
return telemetry is an invalid trial, not a 20-second settling measurement.
If return telemetry cannot complete the measurement, a PC watchdog ends the
wait after 21 seconds from receipt of `RETURN` (one second of serial grace).
Home recovery retains its separate 20-second limit.
Setup accepts position within +/-3 degrees of 300 degrees, with speed at most
12 degrees/second continuously for 850 ms. The wider setup tolerance does not
change the return-settling tolerance.

The 115-trial budget counts attempted trials, including failures. JSL attempts
all seed trials even if earlier ones fail, then uses remaining attempts to
replenish seed measurements until four valid measurements are available before
requesting optimization candidates. If no valid trials can be collected, it
finishes with a failure report rather than throwing a trial-timeout error.
Results and failures are saved as `results.jmp` and `failed_trials.jmp` in the
experiment folder. This allows continuation after failed trials; it does not
fix the underlying setup or Home recovery behavior.

Upload the updated firmware before using this worker. Serial data now contains
`D,time_ms,mode,position_deg,error_deg,encoder_count,pwm,relative_count,p_term,i_term,d_term,pid_raw`.
The `relative_count` field is exactly `encoder_count - home`; the original absolute-count
field is preserved. Python tests the relative count for settling: within +/-75
counts and unchanged for five consecutive comparisons. This widens the band while retaining the
unchanged-count stationarity rule and corrected Home reference.

The appended diagnostic fields contain signed P, I, and D contributions in PWM
units and their sum (`pid_raw`) before output saturation, rounding, or the PWM
minimum. The I field reflects the accepted integral state after anti-windup.
The existing `pwm` field remains the actual signed output applied to the driver.
All fields are captured in each CSV's Raw Serial Line; the earlier field
positions remain unchanged for compatibility with the Python worker.

The candidate controller is `u = Kp*error + Ki*integral - Kd*position_velocity`, PWM limited to ±225. Every nonzero command has a minimum PWM magnitude of 37, preserving direction; zero still stops the motor. This minimum applies to trial, setup, and recovery motion. The serial `pwm` field reports the actual signed PWM applied after rounding and limits. The fixed 300° setup and recovery controller is PID 1.35/0.4/0.1. Overshoot never ends a run; it is measured up to 300°. A return ends when it is stationary within ±75 encoder counts for five 100 Hz samples or at the 20 s cutoff.

Before flashing/resetting, put the arm physically at Home. Each trial re-zeros at Home after the previous fixed-PID recovery.

The fixed setup/recovery PID uses conditional-integration anti-windup: it rejects
an integral update if the proposed output exceeds the PWM cap and that update
would push farther into saturation. Updates that unwind the integral remain
allowed. Candidate return PID integration, gains, minimum PWM, and acceptance
tolerances are unchanged for this experiment.

Build/flash with PlatformIO, then open `jmp/DiscoveryPID_Position_V2.jsl` in JMP. Edit the three path/port lines at the top only if your local paths change.

The JSON field final_counts_from_home is the signed relative encoder count in
the last return-test sample, before Home recovery. It is null if no return
sample was received. The settling band is inclusive (+/-75 counts, about
+/-2.54 degrees), still requiring five consecutive unchanged-count comparisons.

The 115-attempt budget targets roughly 100 minutes when trials use their full
12-second setup, 20-second return, and 20-second recovery windows, plus serial
and JMP overhead. Early settling or failed setup can shorten the experiment;
this run count is an estimate, not a fixed-duration timer.
