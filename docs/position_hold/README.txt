DiscoveryPID POSITION HOLD + JMP BAYESIAN OPTIMIZATION
========================================================

FILES
-----
main.cpp
    Arduino Mega firmware. Replace repo src/main.cpp with this file.

run_position_trial.py
    Guarded Python worker. Copy to:
    tools/run_position_trial.py

DiscoveryPID_Position_Bayesian.jsl
    JMP automatic Bayesian Optimization loop. Copy to:
    jmp/DiscoveryPID_Position_Bayesian.jsl


WHAT IT DOES
------------
1. Power on: motor remains disabled.
2. Move the stick left and right.
3. Hold it upright/perpendicular.
4. After about 30 degrees of total travel, at least one reversal, and
   1.5 seconds motionless, firmware prints:
       EVENT HOMED
5. Position hold activates.
6. Manual knocks/displacements are corrected automatically.
7. JMP tests candidate outer-loop Kp/Ki/Kd settings.
8. A conservative fixed controller moves the stick to 20 degrees.
9. Candidate PID returns it to home.
10. Python scores:
      - settling time
      - overshoot
      - oscillation count
      - integrated absolute error
      - final error
11. JMP Bayesian Optimization chooses the next Kp/Ki/Kd.

The existing inner speed PI remains at SP=2 and SI=5.


INSTALL
-------
From the repo folder:

1. Back up current work:
       git add -A
       git commit -m "Save speed autotune before position control"

2. Copy files:
       main.cpp -> src/main.cpp
       run_position_trial.py -> tools/run_position_trial.py
       DiscoveryPID_Position_Bayesian.jsl -> jmp/DiscoveryPID_Position_Bayesian.jsl

3. Verify Python:
       & "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m pip install pyserial

4. Build/upload:
       $pio = "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe"
       & $pio run
       & $pio run --target upload

5. Open serial monitor:
       & $pio device monitor --port COM7 --baud 115200

6. Calibrate:
       - Move stick left/right.
       - Hold it upright.
       - Wait for EVENT HOMED.
       - Gently move it and verify it returns.
       - Type STOP if behavior is unsafe.
       - Close monitor with Ctrl+C.

7. Copy the Python worker and JSL into the listed repo locations.

8. Open DiscoveryPID_Position_Bayesian.jsl in JMP and press Ctrl+R.

9. Click Arm and Start once.


IMPORTANT SERIAL DETAIL
-----------------------
The Python worker opens COM7 with DTR and RTS disabled to avoid resetting
the Mega between trials. If your USB driver still resets it on the first
worker connection:

1. Run JMP once.
2. If it reports NOT_HOMED, close JMP.
3. Reopen the serial monitor.
4. Calibrate again and wait for EVENT HOMED.
5. Close the monitor.
6. Restart JMP.

Subsequent worker connections should leave the Mega running.


NORMAL COMMANDS
---------------
IDENTIFY
STATUS
CALIBRATE
ZERO
HOLD
STOP
CLEARFAULT

KP=0.8
KI=0.05
KD=0.025

SP=2
SI=5

MAXRPM=35
POS=10
TEST=20
TRIAL=1


SAFETY
------
- Keep the first tests unloaded and mechanically clear.
- Keep a power cutoff accessible.
- STOP immediately if the direction is wrong.
- If position correction moves away from home, change:
      ENCODER_DIRECTION_SIGN = 1
  to:
      ENCODER_DIRECTION_SIGN = -1
  rebuild and upload.
- Firmware limits:
      Position: 75 degrees
      Output speed: 70 RPM
      PWM: 225
      Test displacement: 5 to 30 degrees
- The controller enters FAULT on limit or stall detection.


NEW STUDY
---------
Change this line in the JSL file:
    studyName = "PositionStudy_001";

Use PositionStudy_002, PositionStudy_003, etc. for separate studies.


SUCCESS CRITERIA
----------------
The Python worker marks Success=1 when:
- settling time <= 2.0 seconds
- overshoot <= 1.0 degree
- oscillations = 0
- final error <= 0.8 degree

JMP stops on the first successful combination or maxTotalRuns.
