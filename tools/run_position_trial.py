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

EXPECTED_FW = "DiscoveryPID position-hold-v1"

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

    p.add_argument("--test-offset", type=float, default=20.0)
    p.add_argument("--baud", type=int, default=115200)

    p.add_argument("--result-file", required=True, type=Path)
    p.add_argument("--raw-file", required=True, type=Path)
    p.add_argument("--history-file", required=True, type=Path)

    p.add_argument("--expected-firmware", default=EXPECTED_FW)

    p.add_argument("--overall-timeout", type=float, default=20.0)
    p.add_argument("--settle-band-deg", type=float, default=1.2)
    p.add_argument("--settle-rpm", type=float, default=1.5)
    p.add_argument("--settle-hold-seconds", type=float, default=1.0)

    p.add_argument("--success-settling-seconds", type=float, default=2.0)
    p.add_argument("--success-overshoot-deg", type=float, default=1.0)
    p.add_argument("--success-final-error-deg", type=float, default=0.8)

    p.add_argument("--max-position-deg", type=float, default=65.0)
    p.add_argument("--max-output-rpm", type=float, default=65.0)
    p.add_argument("--max-pwm", type=float, default=225.0)

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
    time.sleep(0.20)
    return port


def identify(port: serial.Serial, token: str) -> str:
    port.reset_input_buffer()
    send(port, "IDENTIFY")

    deadline = time.monotonic() + 2.5
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

    raise RuntimeError("No firmware identification received")


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

    if int(status.get("Homed", 0)) != 1:
        raise RuntimeError(
            "Controller is not homed. Wiggle the stick left/right, "
            "hold it at center until EVENT HOMED, then restart JMP."
        )

    if int(status.get("Fault", 0)) != 0:
        raise RuntimeError("Controller is in FAULT mode")

    commands: list[tuple[str, str, float | int | None]] = [
        (f"TRIAL={a.trial}", "ACK TRIAL=", a.trial),
        (f"KP={a.kp:.9g}", "ACK KP=", a.kp),
        (f"KI={a.ki:.9g}", "ACK KI=", a.ki),
        (f"KD={a.kd:.9g}", "ACK KD=", a.kd),
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
        "Homed": 1.0,
    }

    for key, expected in checks.items():
        actual = status.get(key, math.nan)
        if not math.isclose(actual, expected, abs_tol=1e-4):
            raise RuntimeError(
                f"STATUS mismatch for {key}: expected {expected}, got {actual}"
            )


def sign_outside_deadband(value: float, deadband: float = 0.8) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def settling_time(
    rows: list[dict[str, float]],
    band_deg: float,
    rpm_limit: float,
    hold_seconds: float,
) -> float | None:
    if not rows:
        return None

    start_index = 0

    for index, row in enumerate(rows):
        inside = (
            abs(row["error"]) <= band_deg
            and abs(row["rpm"]) <= rpm_limit
        )

        if not inside:
            start_index = index + 1
            continue

        start_time = rows[start_index]["return_time"]
        current_time = row["return_time"]

        if current_time - start_time >= hold_seconds:
            return start_time

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

    settling = settling_time(
        return_rows,
        a.settle_band_deg,
        a.settle_rpm,
        a.settle_hold_seconds,
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
        else math.inf
    )

    timed_out = status in {"TEST_TIMEOUT", "TEST_PREP_TIMEOUT"}

    settling_for_score = (
        settling
        if settling is not None
        else a.overall_timeout
    )

    score = (
        settling_for_score
        + 2.0 * overshoot
        + 1.5 * float(oscillations)
        + 0.05 * iae
        + 0.5 * final_error
        + (50.0 if timed_out else 0.0)
    )

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
                f"{overshoot:.8f}",
                oscillations,
                f"{iae:.8f}",
                f"{peak_rpm:.8f}",
                (
                    ""
                    if not math.isfinite(final_error)
                    else f"{final_error:.8f}"
                ),
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
        f"Success={success} "
        f"Status={status}",
        flush=True,
    )

    return 0 if status in {"COMPLETE", "TEST_TIMEOUT", "TEST_PREP_TIMEOUT"} else 4


def main() -> int:
    a = arguments()

    if not (
        a.trial >= 1
        and 0.0 <= a.kp <= 3.0
        and 0.0 <= a.ki <= 1.0
        and 0.0 <= a.kd <= 0.20
        and 5.0 <= abs(a.test_offset) <= 30.0
        and a.overall_timeout > 5.0
    ):
        print("Invalid trial arguments", file=sys.stderr)
        return 2

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

            if abs(position) > a.max_position_deg:
                status = "ABORT_MAX_POSITION"
                raise RuntimeError(status)

            if abs(rpm) > a.max_output_rpm:
                status = "ABORT_MAX_OUTPUT_RPM"
                raise RuntimeError(status)

            if abs(pwm) > a.max_pwm + 0.5:
                status = "ABORT_MAX_PWM"
                raise RuntimeError(status)

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
        print(f"ERROR: {error}", file=sys.stderr, flush=True)

    except KeyboardInterrupt:
        status = "ABORT_USER"

    finally:
        if port is not None:
            try:
                send(port, "HOLD")
                send(port, "TELEM=1")
            except (SerialException, OSError):
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
