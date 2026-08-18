#!/usr/bin/env python3
"""Run one guarded return-to-home position trial for JMP Bayesian optimization."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
import time
from pathlib import Path

import serial
from serial import SerialException

PAIR_RE = re.compile(
    r"([A-Za-z][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))"
)

EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.3-300deg"

TELEMETRY_FIELDS = {
    "Trial",
    "Mode",
    "Homed",
    "KP",
    "KI",
    "KD",
    "PosDeg",
    "PosTarget",
    "PosError",
    "ErrCounts",
    "SpeedTarget",
    "OutRPM",
    "Counts",
    "PWM",
}

RESULT_HEADERS = [
    "Run",
    "KP",
    "KI",
    "KD",
    "Score",
    "Settling Time (s)",
    "Return Timer Started",
    "Overshoot (deg)",
    "Oscillations",
    "IAE (deg*s)",
    "Peak Output RPM",
    "Final Error (deg)",
    "Success",
    "Status",
    "Samples",
    "Firmware",
    "Raw File",
]

RAW_HEADERS = [
    "PC Timestamp",
    "Elapsed Time (s)",
    "Return Time (s)",
    "Run",
    "Mode",
    "KP",
    "KI",
    "KD",
    "Position (deg)",
    "Position Target (deg)",
    "Position Error (deg)",
    "Speed Target (RPM)",
    "Output RPM",
    "Encoder Counts",
    "PWM Command",
    "Raw Serial Line",
]

HISTORY_HEADERS = [
    "Run",
    "Return Time (s)",
    "KP",
    "KI",
    "KD",
    "Position (deg)",
    "Position Error (deg)",
    "Output RPM",
    "PWM Command",
]


def arguments() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)

    p.add_argument("--port", required=True)
    p.add_argument("--trial", required=True, type=int)

    p.add_argument("--kp", required=True, type=float)
    p.add_argument("--ki", required=True, type=float)
    p.add_argument("--kd", required=True, type=float)

    p.add_argument("--test-offset", type=float, default=300.0)
    p.add_argument("--baud", type=int, default=115200)

    p.add_argument("--result-file", required=True, type=Path)
    p.add_argument("--raw-file", required=True, type=Path)
    p.add_argument("--history-file", required=True, type=Path)

    p.add_argument("--expected-firmware", default=EXPECTED_FW)

    p.add_argument("--overall-timeout", type=float, default=60.0)

    p.add_argument("--success-settling-seconds", type=float, default=1.0)
    p.add_argument("--success-overshoot-deg", type=float, default=5.0)
    p.add_argument("--success-final-error-deg", type=float, default=1.2)

    p.add_argument("--return-max-pwm", type=float, default=225.0)
    p.add_argument("--home-padding-pulses", type=int, default=2)
    p.add_argument("--minimum-drive-pwm", type=float, default=60.0)
    p.add_argument("--setup-kp", type=float, required=True)
    p.add_argument("--setup-ki", type=float, required=True)
    p.add_argument("--setup-kd", type=float, required=True)

    return p.parse_args()


def numeric_pairs(line: str) -> dict[str, float]:
    return {key: float(value) for key, value in PAIR_RE.findall(line)}


def telemetry(line: str) -> dict[str, float] | None:
    values = numeric_pairs(line)
    return values if TELEMETRY_FIELDS.issubset(values) else None


def send(port: serial.Serial, text: str) -> None:
    port.write((text + "\n").encode("ascii"))
    port.flush()
    time.sleep(0.04)


def open_without_reset(port_name: str, baud: int) -> serial.Serial:
    """
    Open with DTR/RTS disabled to avoid resetting the Mega between JMP trials.

    The first connection on some USB drivers may still reset the board. If that
    occurs, recalibrate once, close the monitor, and run the study again.
    """
    port = serial.Serial()
    port.port = port_name
    port.baudrate = baud
    port.timeout = 0.15
    port.write_timeout = 1.0
    port.dtr = False
    port.rts = False
    port.open()

    # Reassert after opening because some Windows drivers change line state.
    port.dtr = False
    port.rts = False
    time.sleep(0.75)
    return port


def identify(port: serial.Serial, token: str) -> str:
    """
    Identify the firmware robustly on Windows/Arduino Mega.

    Opening COM7 can occasionally disturb the first serial command even with
    DTR/RTS disabled. Retry IDENTIFY rather than failing on one malformed line.
    """
    port.reset_input_buffer()

    for attempt in range(1, 7):
        send(port, "IDENTIFY")
        deadline = time.monotonic() + 1.5

        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue

            print(line, flush=True)

            if line.startswith("Firmware:"):
                firmware = line.partition(":")[2].strip()
                if token not in firmware:
                    raise RuntimeError(f"Wrong firmware: {firmware}")
                return firmware

            # A partially received first command can produce UNKNOWN_COMMAND.
            # Ignore it here and retry IDENTIFY.
            if line.startswith("ERR UNKNOWN_COMMAND"):
                break

        port.reset_input_buffer()
        time.sleep(0.25 * attempt)

    raise RuntimeError(
        "No firmware identification received after 6 IDENTIFY attempts"
    )


def wait_for_ack(
    port: serial.Serial,
    prefix: str,
    expected: float | int | None = None,
    timeout: float = 2.0,
) -> None:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(line, flush=True)

        if line.startswith(prefix):
            if expected is None:
                return

            actual = float(line.split("=", 1)[1])
            if math.isclose(actual, float(expected), abs_tol=1e-4):
                return

            raise RuntimeError(
                f"{prefix} value mismatch: expected {expected}, got {actual}"
            )

        if line.startswith("ERR "):
            raise RuntimeError(line)

    raise RuntimeError(f"Missing acknowledgement: {prefix}")


def get_status(port: serial.Serial) -> dict[str, float]:
    port.reset_input_buffer()
    send(port, "STATUS")

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue

        print(line, flush=True)

        if line.startswith("STATUS "):
            return numeric_pairs(line)

    raise RuntimeError("No STATUS response")


def configure(port: serial.Serial, a: argparse.Namespace) -> None:
    status = get_status(port)

    # A Mega can reset when COM7 opens. Give startup a moment to restore home.
    if int(status.get("Homed", 0)) != 1:
        time.sleep(0.75)
        status = get_status(port)

    # Run 1 starts immediately after the user physically homes the mechanism.
    # If the USB open reset cleared volatile home state, re-zero the CURRENT
    # physical position before the very first trial.
    if int(status.get("Homed", 0)) != 1 and a.trial == 1:
        print(
            "INFO: Run 1 lost volatile home during COM7 open; "
            "re-zeroing current physical position.",
            flush=True,
        )
        port.reset_input_buffer()
        send(port, "ZERO")
        wait_for_ack(port, "ACK ZERO", None, timeout=2.0)
        status = get_status(port)

    if int(status.get("Homed", 0)) != 1:
        raise RuntimeError(
            "NOT_HOMED_AFTER_SERIAL_OPEN"
        )

    if int(status.get("Fault", 0)) != 0:
        raise RuntimeError("Controller is in FAULT mode")

    commands: list[tuple[str, str, float | int | None]] = [
        (f"TRIAL={a.trial}", "ACK TRIAL=", a.trial),
        (f"KP={a.kp:.9g}", "ACK KP=", a.kp),
        (f"KI={a.ki:.9g}", "ACK KI=", a.ki),
        (f"KD={a.kd:.9g}", "ACK KD=", a.kd),
        (f"MAXPWM={a.return_max_pwm:.9g}", "ACK MAXPWM=", a.return_max_pwm),
        (f"HOMEPAD={a.home_padding_pulses}", "ACK HOMEPAD=", a.home_padding_pulses),
        (f"MINPWM={a.minimum_drive_pwm:.9g}", "ACK MINPWM=", a.minimum_drive_pwm),
        (f"SKP={a.setup_kp:.9g}", "ACK SKP=", a.setup_kp),
        (f"SKI={a.setup_ki:.9g}", "ACK SKI=", a.setup_ki),
        (f"SKD={a.setup_kd:.9g}", "ACK SKD=", a.setup_kd),
        ("TELEM=2", "ACK TELEM=", 2),
        ("HOLD", "ACK HOLD", None),
    ]

    for command, acknowledgement, expected in commands:
        port.reset_input_buffer()
        send(port, command)
        wait_for_ack(port, acknowledgement, expected)

    status = get_status(port)

    checks = {
        "Trial": float(a.trial),
        "KP": a.kp,
        "KI": a.ki,
        "KD": a.kd,
        "MaxPWM": a.return_max_pwm,
        "HomePad": float(a.home_padding_pulses),
        "MinPWM": a.minimum_drive_pwm,
        "SKP": a.setup_kp,
        "SKI": a.setup_ki,
        "SKD": a.setup_kd,
        "Homed": 1.0,
    }

    for key, expected in checks.items():
        actual = status.get(key, math.nan)
        if not math.isclose(actual, expected, abs_tol=1e-4):
            raise RuntimeError(
                f"STATUS mismatch for {key}: expected {expected}, got {actual}"
            )


def recover_home(port: serial.Serial, timeout: float = 20.0) -> bool:
    """Return the mechanism to a settled HOLD before releasing the serial port."""
    try:
        port.reset_input_buffer()
        send(port, "HOLD")
        wait_for_ack(port, "ACK HOLD", None, timeout=2.0)
    except (SerialException, OSError, RuntimeError):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            status = get_status(port)
        except (SerialException, OSError, RuntimeError):
            return False

        if (
            int(status.get("Homed", 0)) == 1
            and int(status.get("Settled", 0)) == 1
        ):
            return True

        time.sleep(0.15)

    return False


def sign_outside_deadband(value: float, deadband: float = 0.8) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def settling_time(
    rows: list[dict[str, float]],
    home_padding_pulses: int,
) -> float | None:
    """Release-to-home settling time.

    Time zero is EVENT TEST_RETURN: the instant firmware releases the displaced
    hold and switches the target back to home under the candidate PID.

    The measured settling time is the first return telemetry sample satisfying:

        abs(ErrCounts) <= home_padding_pulses

    Displacement/prep time is excluded completely.
    No RPM or extra dwell requirement is applied to this metric.
    """
    for row in rows:
        error_counts = row.get("error_counts", math.nan)

        if not math.isfinite(error_counts):
            continue

        if abs(error_counts) <= home_padding_pulses:
            return float(row["return_time"])

    return None

def overshoot_and_oscillations(
    rows: list[dict[str, float]],
) -> tuple[float, int]:
    if not rows:
        return 0.0, 0

    initial_sign = 0
    first_cross_index: int | None = None

    for index, row in enumerate(rows):
        sign = sign_outside_deadband(row["error"])
        if initial_sign == 0 and sign != 0:
            initial_sign = sign
        elif initial_sign != 0 and sign == -initial_sign:
            first_cross_index = index
            break

    if first_cross_index is None:
        return 0.0, 0

    opposite_errors = [
        abs(row["error"])
        for row in rows[first_cross_index:]
        if sign_outside_deadband(row["error"]) == -initial_sign
    ]

    overshoot = max(opposite_errors, default=0.0)

    oscillations = 0
    previous_sign = -initial_sign

    for row in rows[first_cross_index + 1 :]:
        sign = sign_outside_deadband(row["error"])
        if sign == 0:
            continue
        if sign != previous_sign:
            oscillations += 1
            previous_sign = sign

    return overshoot, oscillations


def integrated_absolute_error(rows: list[dict[str, float]]) -> float:
    if len(rows) < 2:
        return 0.0

    result = 0.0

    for previous, current in zip(rows, rows[1:]):
        dt = max(0.0, current["return_time"] - previous["return_time"])
        result += (
            0.5
            * (abs(previous["error"]) + abs(current["error"]))
            * dt
        )

    return result


def write_raw(path: Path, rows: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(RAW_HEADERS)

        for row in rows:
            writer.writerow(
                [
                    row["timestamp"],
                    f'{row["elapsed"]:.4f}',
                    (
                        ""
                        if row["return_time"] < 0
                        else f'{row["return_time"]:.4f}'
                    ),
                    int(row["trial"]),
                    int(row["mode"]),
                    row["kp"],
                    row["ki"],
                    row["kd"],
                    row["position"],
                    row["target"],
                    row["error"],
                    row["speed_target"],
                    row["rpm"],
                    int(row["counts"]),
                    row["pwm"],
                    row["raw"],
                ]
            )


def append_history(
    path: Path,
    run: int,
    kp: float,
    ki: float,
    kd: float,
    rows: list[dict[str, float | str]],
) -> None:
    return_rows = [row for row in rows if row["return_time"] >= 0]

    if not return_rows:
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0

    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)

        if new_file:
            writer.writerow(HISTORY_HEADERS)

        for row in return_rows:
            writer.writerow(
                [
                    run,
                    f'{row["return_time"]:.4f}',
                    kp,
                    ki,
                    kd,
                    row["position"],
                    row["error"],
                    row["rpm"],
                    row["pwm"],
                ]
            )


def write_result(
    a: argparse.Namespace,
    firmware: str,
    status: str,
    rows: list[dict[str, float | str]],
) -> int:
    return_rows = [
        row
        for row in rows
        if row["return_time"] >= 0
    ]

    # If this is zero, EVENT TEST_RETURN was never observed and there is no
    # legitimate release-to-home timing measurement.
    return_timer_started = int(len(return_rows) > 0)

    settling = settling_time(
        return_rows,
        a.home_padding_pulses,
    )

    overshoot, oscillations = overshoot_and_oscillations(return_rows)
    iae = integrated_absolute_error(return_rows)
    peak_rpm = max((abs(row["rpm"]) for row in return_rows), default=0.0)

    final_window = (
        return_rows[-5:]
        if len(return_rows) >= 5
        else return_rows
    )

    final_error = (
        statistics.fmean(abs(row["error"]) for row in final_window)
        if final_window
        else 999.0
    )

    timed_out = status in {"TEST_TIMEOUT", "TEST_PREP_TIMEOUT"}

    settling_for_score = (
        settling
        if settling is not None
        else a.overall_timeout
    )

    abnormal_status = status not in {
        "COMPLETE",
        "TEST_TIMEOUT",
        "TEST_PREP_TIMEOUT",
    }

    # FAST-RETURN objective:
    # settling time dominates. Overshoot/oscillation/final error remain secondary
    # penalties so JMP does not "win" by blasting through home without settling.
    score = (
        10.0 * settling_for_score
        + 0.25 * overshoot
        + 0.25 * float(oscillations)
        + 0.01 * iae
        + 0.25 * final_error
        + (25.0 if timed_out else 0.0)
        + (500.0 if abnormal_status else 0.0)
    )

    # Never emit inf/nan into CSV. JMP can infer a Character response column
    # from non-numeric tokens, which breaks Response Limits/Bayesian Optimization.
    if not math.isfinite(score):
        score = 9999.0

    success = int(
        status == "COMPLETE"
        and settling is not None
        and settling <= a.success_settling_seconds
        and overshoot <= a.success_overshoot_deg
        and oscillations == 0
        and final_error <= a.success_final_error_deg
    )

    a.result_file.parent.mkdir(parents=True, exist_ok=True)

    with a.result_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(RESULT_HEADERS)
        writer.writerow(
            [
                a.trial,
                f"{a.kp:.8f}",
                f"{a.ki:.8f}",
                f"{a.kd:.8f}",
                f"{score:.8f}",
                (
                    ""
                    if settling is None
                    else f"{settling:.8f}"
                ),
                return_timer_started,
                f"{overshoot:.8f}",
                oscillations,
                f"{iae:.8f}",
                f"{peak_rpm:.8f}",
                f"{final_error:.8f}",
                success,
                status,
                len(return_rows),
                firmware,
                str(a.raw_file.resolve()),
            ]
        )

    print(
        "RESULT "
        f"Run={a.trial} "
        f"KP={a.kp:.6g} "
        f"KI={a.ki:.6g} "
        f"KD={a.kd:.6g} "
        f"Score={score:.6f} "
        f"Settling={settling if settling is not None else 'MISSING'} "
        f"ReturnTimerStarted={return_timer_started} "
        f"Success={success} "
        f"Status={status}",
        flush=True,
    )

    # Always emit a finite diagnostic row. The JSL decides whether to warn,
    # continue, or exclude the row from best-setup-PID selection.
    return 0


def main() -> int:
    a = arguments()

    if not (
        a.trial >= 1
        and math.isfinite(a.kp)
        and math.isfinite(a.ki)
        and math.isfinite(a.kd)
        and math.isfinite(a.test_offset)
        and math.isfinite(a.return_max_pwm)
        and a.home_padding_pulses >= 0
        and math.isfinite(a.minimum_drive_pwm)
        and math.isfinite(a.setup_kp)
        and math.isfinite(a.setup_ki)
        and math.isfinite(a.setup_kd)
        and a.overall_timeout > 0.0
    ):
        # Still emit a numeric result rather than killing the JMP study.
        rows: list[dict[str, float | str]] = []
        return write_result(
            a,
            "",
            "INVALID_ARGUMENTS",
            rows,
        )

    a.raw_file.parent.mkdir(parents=True, exist_ok=True)
    a.result_file.parent.mkdir(parents=True, exist_ok=True)

    if a.result_file.exists():
        a.result_file.unlink()

    rows: list[dict[str, float | str]] = []
    firmware = ""
    status = "FAILED"
    port: serial.Serial | None = None

    try:
        port = open_without_reset(a.port, a.baud)
        firmware = identify(port, a.expected_firmware)
        configure(port, a)

        port.reset_input_buffer()
        send(port, f"TEST={a.test_offset:.8g}")
        wait_for_ack(port, "ACK TEST=", a.test_offset, timeout=2.0)

        start = time.monotonic()
        return_start: float | None = None
        test_complete = False

        while time.monotonic() - start < a.overall_timeout:
            line = port.readline().decode(
                "utf-8",
                errors="replace",
            ).strip()

            if not line:
                continue

            print(line, flush=True)

            if line == "EVENT TEST_RETURN":
                return_start = time.monotonic()
                continue

            if line == "EVENT TEST_COMPLETE":
                status = "COMPLETE"
                test_complete = True
                break

            if line == "EVENT TEST_TIMEOUT":
                status = "TEST_TIMEOUT"
                test_complete = True
                break

            if line == "EVENT TEST_PREP_TIMEOUT":
                status = "TEST_PREP_TIMEOUT"
                test_complete = True
                break

            if line.startswith("EVENT FAULT"):
                status = line.replace("EVENT ", "", 1).replace(" ", "_")
                raise RuntimeError(line)

            values = telemetry(line)
            if values is None:
                continue

            if int(values["Trial"]) != a.trial:
                raise RuntimeError("Telemetry trial mismatch")

            for key, expected in (
                ("KP", a.kp),
                ("KI", a.ki),
                ("KD", a.kd),
            ):
                if not math.isclose(
                    values[key],
                    expected,
                    abs_tol=1e-4,
                ):
                    raise RuntimeError(
                        f"Telemetry {key} mismatch"
                    )

            if int(values["Homed"]) != 1:
                raise RuntimeError("Controller lost home state")

            position = values["PosDeg"]
            rpm = values["OutRPM"]
            pwm = values["PWM"]

            now = time.monotonic()
            elapsed = now - start

            rows.append(
                {
                    "timestamp": time.strftime(
                        "%Y-%m-%dT%H:%M:%S"
                    ),
                    "elapsed": elapsed,
                    "return_time": (
                        -1.0
                        if return_start is None
                        else now - return_start
                    ),
                    "trial": float(a.trial),
                    "mode": values["Mode"],
                    "kp": a.kp,
                    "ki": a.ki,
                    "kd": a.kd,
                    "position": position,
                    "target": values["PosTarget"],
                    "error": values["PosError"],
                    "error_counts": values["ErrCounts"],
                    "speed_target": values["SpeedTarget"],
                    "rpm": rpm,
                    "counts": values["Counts"],
                    "pwm": pwm,
                    "raw": line,
                }
            )

        if not test_complete:
            status = "WORKER_TIMEOUT"

    except (SerialException, OSError, RuntimeError, ValueError) as error:
        error_text = str(error).strip()
        status = "CONTROL_ERROR_" + re.sub(
            r"[^A-Za-z0-9_]+",
            "_",
            error_text,
        ).strip("_")[:120]
        print(f"ERROR: {error_text}", flush=True)

    except KeyboardInterrupt:
        status = "ABORT_USER"

    finally:
        if port is not None:
            recovered = recover_home(port)
            if not recovered:
                print(
                    "WARNING: mechanism did not confirm settled home before COM close",
                    flush=True,
                )
            try:
                send(port, "TELEM=1")
            except Exception:
                pass
            port.close()

    write_raw(a.raw_file, rows)
    append_history(
        a.history_file,
        a.trial,
        a.kp,
        a.ki,
        a.kd,
        rows,
    )

    return write_result(
        a,
        firmware,
        status,
        rows,
    )


if __name__ == "__main__":
    raise SystemExit(main())
