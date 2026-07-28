DiscoveryPID JMP Autotune Branch
================================

Branch name
-----------

    jmp-autotune

This branch is separate from the manual controller on main.

Switch to the autotune branch
-----------------------------

    git fetch origin
    git switch jmp-autotune
    git pull origin jmp-autotune

Upload the autotune firmware
----------------------------

Close every serial monitor, then run:

    pio run --target clean
    pio run
    pio run --target upload

The serial startup should include:

    Firmware: DiscoveryPID jmp-autotune-v1 2026-07-28
    Direction reversal guard: ON

Run one P/I test
----------------

Example: trial 1, P=1.25, I=2.75, target=100 RPM, duration=30 seconds.

    py .\tools\serial_to_excel.py --port COM5 --trial 1 --p 1.25 --i 2.75 --target 100 --duration 30

Replace COM5 with the Arduino Mega's actual COM port.

Files created for every run
---------------------------

    data\logs\DiscoveryPID_YYYYMMDD_HHMMSS.xlsx
    data\logs\DiscoveryPID_YYYYMMDD_HHMMSS.csv
    data\logs\DiscoveryPID_YYYYMMDD_HHMMSS_JMP.csv

The Excel workbook contains these worksheets:

    Telemetry  - complete test data, including P and I.
    JMP Data   - only P, I, and Error.
    Summary    - run-level statistics and plots.
    Metadata   - test settings and output paths.

The JMP CSV contains exactly:

    P,I,Error

Error is calculated as:

    Error = Target RPM - Output RPM

Runtime serial commands
-----------------------

    P=1.25
    I=2.75
    RPM=100
    TRIAL=1
    STATUS
    STOP

The active P and I gains are printed in every telemetry line. The controller
starts at 0 RPM and retains the active-direction-reversal safety guard.

Return to manual tuning
-----------------------

Stop the motor, close serial programs, then run:

    git switch main
    git pull origin main
    pio run --target clean
    pio run
    pio run --target upload
