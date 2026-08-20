#!/usr/bin/env python3
"""Interactive exact-gain return check for DiscoveryPID.

The motor is placed in STOP/coast so the mechanism can be displaced by hand.
When the user presses Enter, HOLD is enabled with the requested PID gains and
serial telemetry is shown until the firmware reports its home-settled latch.
No host-side PID quantization or rounding is applied.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

import serial


STATUS_POS_RE = re.compile(r"\bPosDeg=([-+0-9.eE]+)")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--kp", required=True, type=float)
    p.add_argument("--ki", required=True, type=float)
    p.add_argument("--kd", required=True, type=float)
    p.add_argument("--minimum-drive-pwm", type=float, default=0.0)
    p.add_argument("--return-max-pwm", type=float, default=225.0)
    p.add_argument("--settle-timeout", type=float, default=5.0)
    return p.parse_args()


def write_command(ser: serial.Serial, command: str) -> None:
    ser.write((command + "\n").encode("ascii"))
    ser.flush()


def read_line(ser: serial.Serial) -> str:
    raw = ser.readline()
    if not raw:
        return ""
    return raw.decode("utf-8", errors="replace").strip()


def drain(ser: serial.Serial, duration: float = 0.25) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        read_line(ser)


def command_wait(
    ser: serial.Serial,
    command: str,
    expected_prefix: str,
    timeout: float = 2.0,
    *,
    echo: bool = True,
) -> str:
    write_command(ser, command)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = read_line(ser)
        if not line:
            continue
        if echo:
            print(line)
        if line.startswith(expected_prefix):
            return line
    raise RuntimeError(f"No expected response to {command!r}: {expected_prefix!r}")


def status(ser: serial.Serial, timeout: float = 2.0, *, echo: bool = False) -> str:
    return command_wait(ser, "STATUS", "STATUS ", timeout, echo=echo)


def exact_number(value: float) -> str:
    # 12 significant digits is well beyond the useful precision of AVR float,
    # while avoiding any project/JMP grid quantization.
    return format(value, ".12g")


def main() -> int:
    a = parse_args()

    print("DiscoveryPID exact PID manual return check")
    print(f"Port: {a.port}")
    print(
        "Requested PID (NO host rounding): "
        f"Kp={exact_number(a.kp)} Ki={exact_number(a.ki)} Kd={exact_number(a.kd)}"
    )
    print(f"Hard return cutoff: {a.settle_timeout:g} s")
    print("Start with the mechanism physically at its intended HOME position.")
    print()

    try:
        with serial.Serial(a.port, 115200, timeout=0.05, write_timeout=1.0) as ser:
            # Opening a Mega serial port commonly resets the board. Allow startup
            # and persistent-home restoration to complete before touching control.
            time.sleep(2.2)
            drain(ser, 0.25)

            initial = status(ser, echo=True)
            if "Homed=1" not in initial:
                print(
                    "ERROR: Firmware is not homed. Run '.\\DiscoveryPID.ps1 home' "
                    "with the mechanism at the intended zero, then retry.",
                    file=sys.stderr,
                )
                return 2

            # Send the requested values directly to firmware. These are not passed
            # through the JMP 0.001 grid or the legacy 0.5-step worker.
            command_wait(ser, f"KP={exact_number(a.kp)}", "ACK KP=")
            command_wait(ser, f"KI={exact_number(a.ki)}", "ACK KI=")
            command_wait(ser, f"KD={exact_number(a.kd)}", "ACK KD=")
            command_wait(
                ser,
                f"MINPWM={exact_number(a.minimum_drive_pwm)}",
                "ACK MINPWM=",
            )
            command_wait(
                ser,
                f"MAXPWM={exact_number(a.return_max_pwm)}",
                "ACK MAXPWM=",
            )

            # STOP leaves the position reference/home intact but commands coast,
            # which is exactly what is needed for a hand displacement.
            command_wait(ser, "STOP", "ACK STOP")
            print()
            print("Motor is now COASTING. Manually displace the mechanism.")
            input("When ready to release it back to HOME, press Enter... ")

            displaced_status = status(ser)
            match = STATUS_POS_RE.search(displaced_status)
            displaced_position = float(match.group(1)) if match else float("nan")
            print(f"Release position: {displaced_position:.4f} deg")

            command_wait(ser, "TELEM=2", "ACK TELEM=2", echo=False)
            start = time.monotonic()
            command_wait(ser, "HOLD", "ACK HOLD")
            print("Return started. Live telemetry:")

            next_status_at = start
            settled = False
            final_status = ""

            while time.monotonic() - start < a.settle_timeout:
                now = time.monotonic()
                if now >= next_status_at:
                    write_command(ser, "STATUS")
                    next_status_at = now + 0.10

                line = read_line(ser)
                if not line:
                    continue

                if line.startswith("Trial="):
                    print(line)
                elif line.startswith("STATUS "):
                    final_status = line
                    if "Settled=1" in line:
                        settled = True
                        break
                elif line.startswith("EVENT ") or line.startswith("ERR "):
                    print(line)

            elapsed = time.monotonic() - start
            command_wait(ser, "TELEM=1", "ACK TELEM=1", echo=False)

            if settled:
                final_status = status(ser, echo=True)
                print()
                print(f"HOME settled latch reached in approximately {elapsed:.3f} s.")
                print("Controller remains in HOLD at home.")
                return 0

            # A timed-out TEST is over: stop applying PID output rather than
            # allowing an unsuccessful controller to keep driving indefinitely.
            try:
                command_wait(ser, "STOP", "ACK STOP", echo=False)
            except RuntimeError:
                write_command(ser, "STOP")
            final_status = status(ser, echo=True)

            print()
            print(
                f"TIMEOUT: HOME settled latch was not reached within {a.settle_timeout:.1f} s.",
                file=sys.stderr,
            )
            print("Run ended; motor commanded to STOP/coast.")
            return 3

    except KeyboardInterrupt:
        print("\nInterrupted. Attempting STOP/coast for safety.", file=sys.stderr)
        try:
            if "ser" in locals() and ser.is_open:
                write_command(ser, "STOP")
        except Exception:
            pass
        return 130
    except (serial.SerialException, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
