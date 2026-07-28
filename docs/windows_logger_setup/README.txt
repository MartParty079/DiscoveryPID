DiscoveryPID Windows Logger Setup
=================================

Purpose
-------
This guide explains how to set up another Windows computer to:

1. Clone the DiscoveryPID Git repository.
2. Install the Python packages needed by the serial logger.
3. Find the Arduino Mega COM port.
4. Run a timed RPM test.
5. Find the generated Excel and CSV files.


1. Clone the repository
-----------------------
Open Windows PowerShell or the VS Code terminal and run:

    git clone https://github.com/MartParty079/DiscoveryPID.git
    cd .\DiscoveryPID

If the repository is already downloaded, navigate to the folder that contains:

    platformio.ini
    src
    tools
    examples

For a path containing spaces, use quotes:

    cd "C:\Users\TheirName\Documents\GITHUB\PID Discovery\DiscoveryPID"

Confirm the logger exists:

    Test-Path .\tools\serial_to_excel.py

Expected result:

    True


2. Confirm Python is installed
------------------------------
Run:

    py --version

If the py command works, continue to the next step.


3. Install the required Python packages
---------------------------------------
From the DiscoveryPID project folder, run:

    py -m pip install -r .\tools\requirements.txt

This installs:

    pyserial
    XlsxWriter

Verify the packages:

    py -c "import serial, xlsxwriter; print('Logger dependencies installed')"

Expected result:

    Logger dependencies installed

If pip is unavailable, run:

    py -m ensurepip --upgrade
    py -m pip install --upgrade pip
    py -m pip install pyserial XlsxWriter


4. Connect the Arduino Mega
---------------------------
Connect the Arduino Mega with USB.

Find the COM port using one of these methods:

    pio device list

or:

    Windows Device Manager -> Ports (COM & LPT)

Example:

    Arduino Mega 2560 (COM5)

Replace COM5 in later commands with the actual port.


5. Close all serial monitors
----------------------------
Before running the logger, close anything already using the Arduino COM port:

- PlatformIO Serial Monitor
- Arduino Serial Monitor
- PuTTY
- Any other terminal connected to the same COM port

Only one program can use the COM port at a time.


6. Run a 60-second, 100 RPM test
--------------------------------
From the DiscoveryPID project folder, run:

    py .\tools\serial_to_excel.py --port COM5 --target 100 --duration 60

The logger will:

1. Open the Arduino at 115200 baud.
2. Send a target of 100 RPM.
3. Record data for 60 seconds.
4. Send a 0 RPM stop command when finished.
5. Save an Excel workbook and CSV file.

To let the logger detect the COM port automatically:

    py .\tools\serial_to_excel.py --target 100 --duration 60

To log until manually stopped:

    py .\tools\serial_to_excel.py --port COM5 --target 100

Press Ctrl+C to stop logging. The script will send 0 RPM and finalize the Excel workbook.


7. Find the recorded files
--------------------------
The files are saved under:

    data\logs\

Example files:

    DiscoveryPID_20260728_163500.xlsx
    DiscoveryPID_20260728_163500.csv

Open the folder with:

    explorer .\data\logs

The CSV is continuously flushed during the test and is useful for JMP and recovery.
The XLSX workbook contains telemetry, summary values, metadata, and charts.


Complete setup sequence
-----------------------
Copy and run these commands one section at a time:

    git clone https://github.com/MartParty079/DiscoveryPID.git
    cd .\DiscoveryPID

    py -m pip install -r .\tools\requirements.txt

    py -c "import serial, xlsxwriter; print('Logger dependencies installed')"

    pio device list

    py .\tools\serial_to_excel.py --port COM5 --target 100 --duration 60

    explorer .\data\logs


Troubleshooting
---------------

Error: can't open file serial_to_excel.py

Cause:
The terminal is not inside the DiscoveryPID project folder.

Fix:

    cd "FULL\PATH\TO\DiscoveryPID"
    Test-Path .\tools\serial_to_excel.py

The Test-Path command must return True.


Error: ModuleNotFoundError: No module named 'serial'

Fix:

    py -m pip install -r .\tools\requirements.txt


Error: Could not open COM5, access denied, or port busy

Fix:
Close the PlatformIO or Arduino serial monitor and verify the correct COM port.


Error: py is not recognized

Use PlatformIO's Python instead:

    C:\Users\TheirName\.platformio\penv\Scripts\python.exe -m pip install -r .\tools\requirements.txt

Then run:

    C:\Users\TheirName\.platformio\penv\Scripts\python.exe .\tools\serial_to_excel.py --port COM5 --target 100 --duration 60


Firmware verification
---------------------
After uploading the current firmware, the serial startup output should include:

    Firmware: DiscoveryPID safety-v2 2026-07-28
    Direction reversal guard: ON

For a positive target RPM, the controller should not apply a negative PWM command.
