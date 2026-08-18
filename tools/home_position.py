#!/usr/bin/env python3
"""Force a fresh position-home calibration and wait for EVENT HOMED."""

from __future__ import annotations

import argparse
import sys
import time

import serial
from serial import SerialException


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.20)
    return parser.parse_args()


def open_port(name: str, baud: int, timeout: float) -> serial.Serial:
    port = serial.Serial()
    port.port = name
    port.baudrate = baud
    port.timeout = timeout
    port.write_timeout = 1.0
    port.dtr = False
    port.rts = False
    port.open()
    port.dtr = False
    port.rts = False
    return port


def send(port: serial.Serial, command: str) -> None:
    port.write((command + "\n").encode("ascii"))
    port.flush()


def main() -> int:
    args = parse_args()

    try:
        port = open_port(args.port, args.baud, args.timeout)
    except (SerialException, OSError) as exc:
        print(f"ERROR: could not open {args.port}: {exc}", file=sys.stderr)
        return 2

    try:
        # Give a Mega that reset on serial-open a moment to finish startup.
        time.sleep(0.75)
        port.reset_input_buffer()

        send(port, "IDENTIFY")
        identify_deadline = time.monotonic() + 2.0
        while time.monotonic() < identify_deadline:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(line)
                if line.startswith("Firmware:"):
                    break

        print("\nForcing a fresh calibration...", flush=True)
        send(port, "CALIBRATE")

        calibration_seen = False
        while True:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            print(line, flush=True)

            if "EVENT CALIBRATION_BEGIN" in line and not calibration_seen:
                calibration_seen = True
                print(
                    "\nMove the gearbox output left/right through enough travel,\n"
                    "then place it at the desired 0-degree home and hold it still.\n",
                    flush=True,
                )

            if "EVENT HOMED" in line:
                print("\nHome saved. COM port released for JMP.", flush=True)
                return 0

            if "EVENT FAULT" in line:
                print("ERROR: firmware entered FAULT during homing.", file=sys.stderr)
                return 3

    except KeyboardInterrupt:
        print("\nHoming cancelled.")
        return 130
    finally:
        port.close()


if __name__ == "__main__":
    raise SystemExit(main())
