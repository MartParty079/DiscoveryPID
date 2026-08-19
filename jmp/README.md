# JMP scripts

## Current position study

Use the helper:

```powershell
& ".\scripts\position_v2\open_jmp.ps1"
```

It opens a temporary runtime copy of `jmp/DiscoveryPID_Position_V2.jsl` with machine-specific repository/Python paths filled in automatically.

The current study drives the 300-degree direct-position-PID experiment and launches `tools/run_position_trial.py` directly.

Current timing metric:

```text
EVENT TEST_RETURN -> first entry into +/-homePaddingPulses
```

Displacement/prep time is excluded.

## Legacy speed autotune

`DiscoveryPID_AutoTune.jsl` and `tools/run_pid_trial.py` belong to the older speed-PI autotune workflow. They are retained for reference but are not used by the current position study.
