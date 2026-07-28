DiscoveryPID JSL Bayesian Autotuning
====================================

Branch
------

    jmp-autotune

Purpose
-------

JMP Pro 19 or newer owns the iterative tuning loop:

    JMP Bayesian Optimization
        -> recommends P and I
        -> JSL launches one physical experiment
        -> Python sends P/I/RPM to the Mega and records the response
        -> Python returns one scalar Error value
        -> JSL appends the completed run to the JMP table
        -> JMP recommends the next P/I combination

The Arduino remains responsible for real-time PI control. Python remains the
hardware and safety supervisor. JSL remains the experiment-loop supervisor.

Files
-----

    jmp/DiscoveryPID_AutoTune.jsl
        Main JSL seed and Bayesian loop.

    tools/run_pid_trial.py
        Executes one guarded physical run and returns one Error result.

    data/autotune/DiscoveryPID_Optimization.jmp
        JMP working table generated after the script starts.

    data/autotune/DiscoveryPID_Optimization.csv
        Compact cumulative table containing only P, I, and Error.

    data/autotune/DiscoveryPID_Response_History.csv
        Cumulative time-series table for Graph Builder.

    data/autotune/runs/
        Raw response CSV for each physical run.

    data/autotune/results/
        One-row result CSV returned to JSL for each run.

Error definition
----------------

For each completed run:

    1. Ignore the first 2 seconds by default.
    2. Calculate a five-sample rolling-average output RPM.
    3. Calculate the absolute difference between target RPM and smoothed RPM.
    4. Average those absolute differences.

The scalar returned to JMP is therefore:

    Error = mean(abs(Target RPM - Smoothed Output RPM))

This penalizes overshoot, undershoot, and sustained oscillation while reducing
the effect of single-sample encoder quantization.

Safety behavior
---------------

The Python worker refuses to start a motor trial unless it detects:

    Firmware: DiscoveryPID jmp-autotune-v1 2026-07-28

It then verifies the TRIAL, P, and I acknowledgements before sending the RPM
command. It stops and rejects a run for wrong firmware, gain mismatch, reverse
PWM at a positive target, excessive output RPM, sustained high PWM, missing
encoder motion, serial failure, or user interruption.

The JSL file defaults to supervised mode. It asks for confirmation before every
physical run. Change this line only after supervised testing is reliable:

    supervisedMode = 1;

Use
---

1. Switch to and update the branch:

       git fetch origin
       git switch jmp-autotune
       git pull origin jmp-autotune

2. Upload the autotune firmware to the Mega.

3. Install the Python serial dependency if needed:

       py -m pip install -r .\tools\requirements.txt

4. Close every PlatformIO or Arduino serial monitor.

5. Open this file in JMP Pro 19 or newer:

       jmp\DiscoveryPID_AutoTune.jsl

6. Check the USER SETTINGS section. The default port is COM7.

7. Run the JSL script.

The script first collects any missing seed combinations. It then launches JMP
Bayesian Optimization with one candidate per batch, runs that candidate, fills
its Error value, saves the table, and repeats. It stops when Error is at or below
2.5 RPM, after 10 Bayesian runs, after a safety abort, or when the operator
selects Stop Autotune.

Generated outputs can be committed to Git when desired. Raw response files can
become large, so review them before committing an extended campaign.
