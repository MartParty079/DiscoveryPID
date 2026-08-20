# JMP scripts

## Current position study

Use the helper:

```powershell
& ".\scripts\position_v2\open_jmp.ps1"
```

It opens a temporary runtime copy of `jmp/DiscoveryPID_Position_V2.jsl` with machine-specific repository/Python paths filled in automatically.

The current study drives the 300-degree direct-position-PID experiment and launches `tools/run_position_trial.py` directly.

### Bayesian objective

The current study uses **multi-response Bayesian Optimization**. `Score` is still recorded for diagnostics and for choosing the best completed PID used during the displacement/setup leg, but `Score` is **not** the Bayesian response.

Bayesian Optimization uses these response columns simultaneously:

- `Return Failure` — 0 for `COMPLETE`, 1 for timeout/error.
- `Settling Time (s)` — release-to-home time; sub-1 second is preferred.
- `PWM Chatter` — excess PWM total variation beyond a clean drive-and-stop command.
- `Home Entry RPM` — output speed at first entry into the home window.
- `Overshoot (deg)` — maximum opposite-side error after the first target crossing.
- `Oscillations` — repeated target crossings after the first crossing.

Each response has a `Response Limits` desirability function. JMP combines those response desirabilities when selecting the next Bayesian run.

The current PID search box is:

```text
Kp: 0.5 .. 200
Ki: 0.0 .. 100
Kd: 0.0 .. 50
step: 0.5
```

The seed design progresses from conservative gains to an intentionally aggressive final seed.

### Timing and invalid runs

Current timing metric:

```text
EVENT TEST_RETURN -> first entry into +/-homePaddingPulses
```

Displacement/prep time is excluded. Every seed and Bayesian trial has the same **3.0 second hard return cutoff**. If the home window is not reached by then, the run is marked `TEST_SETTLING_TIMEOUT`, `Return Failure` is set to 1, the recovery motion is kept outside the scored response data, and the study continues.

Only `COMPLETE` rows can become the best setup PID.

## Legacy speed autotune

`DiscoveryPID_AutoTune.jsl` and `tools/run_pid_trial.py` belong to the older speed-PI autotune workflow. They are retained for reference but are not used by the current position study.
