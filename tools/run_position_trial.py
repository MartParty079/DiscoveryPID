#!/usr/bin/env python3
"""Run one DiscoveryPID physical return-to-home trial for JMP optimization."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import time
from pathlib import Path

import serial
from serial import SerialException

EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange"
SERIAL_FLOAT_TOL = 1e-3
ACK_RETRIES = 3
ACK_TIMEOUT_SECONDS = 2.0
ACK_RETRY_DELAY_SECONDS = 0.15

PAIR_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))")

TELEMETRY_FIELDS = {
    "Trial", "Mode", "Homed", "KP", "KI", "KD", "PosDeg", "PosTarget",
    "PosError", "ErrCounts", "SpeedTarget", "OutRPM", "Counts", "PWM",
}

RESULT_HEADERS = [
    "Run", "KP", "KI", "KD", "Score", "Settling Time (s)",
    "Return Timer Started", "Overshoot (deg)", "Oscillations", "IAE (deg*s)",
    "Peak Output RPM", "Final Error (deg)", "Success", "Status", "Samples",
    "Firmware", "Raw File",
]

RAW_HEADERS = [
    "PC Timestamp", "Elapsed Time (s)", "Return Time (s)", "Run", "Mode",
    "KP", "KI", "KD", "Position (deg)", "Position Target (deg)",
    "Position Error (deg)", "Speed Target (RPM)", "Output RPM",
    "Encoder Counts", "PWM Command", "Raw Serial Line",
]

HISTORY_HEADERS = [
    "Run", "Return Time (s)", "KP", "KI", "KD", "Position (deg)",
    "Position Error (deg)", "Output RPM", "PWM Command",
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
    p.add_argument("--max-settling-seconds", type=float, default=3.0)
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


def round_half(value: float) -> float:
    """Round nonnegative PID coefficients to the nearest 0.5."""
    return math.floor(value * 2.0 + 0.5) / 2.0


def quantize_pid(a: argparse.Namespace) -> None:
    original = (a.kp, a.ki, a.kd, a.setup_kp, a.setup_ki, a.setup_kd)
    a.kp = round_half(a.kp)
    a.ki = round_half(a.ki)
    a.kd = round_half(a.kd)
    a.setup_kp = round_half(a.setup_kp)
    a.setup_ki = round_half(a.setup_ki)
    a.setup_kd = round_half(a.setup_kd)
    rounded = (a.kp, a.ki, a.kd, a.setup_kp, a.setup_ki, a.setup_kd)
    if original != rounded:
        print(
            "INFO: PID values rounded to 0.5 steps: "
            f"candidate=({a.kp:g},{a.ki:g},{a.kd:g}) "
            f"setup=({a.setup_kp:g},{a.setup_ki:g},{a.setup_kd:g})",
            flush=True,
        )


def numeric_pairs(line: str) -> dict[str, float]:
    return {key: float(value) for key, value in PAIR_RE.findall(line)}


def telemetry(line: str) -> dict[str, float] | None:
    values = numeric_pairs(line)
    return values if TELEMETRY_FIELDS.issubset(values) else None


def send(port: serial.Serial, text: str) -> None:
    port.write((text + "\n").encode("ascii"))
    port.flush()
    time.sleep(0.05)


def open_without_reset(port_name: str, baud: int) -> serial.Serial:
    port = serial.Serial()
    port.port = port_name
    port.baudrate = baud
    port.timeout = 0.15
    port.write_timeout = 1.0
    port.dtr = False
    port.rts = False
    port.open()
    port.dtr = False
    port.rts = False
    time.sleep(0.75)
    return port


def ack_matches(line: str, prefix: str, expected: float | int | None) -> bool:
    if not line.startswith(prefix):
        return False
    if expected is None:
        return True
    try:
        actual = float(line.split("=", 1)[1])
    except (IndexError, ValueError):
        return False
    if not math.isclose(actual, float(expected), abs_tol=SERIAL_FLOAT_TOL):
        raise RuntimeError(
            f"{prefix} value mismatch: expected {expected}, got {actual}"
        )
    return True


def send_with_ack(
    port: serial.Serial,
    command: str,
    prefix: str,
    expected: float | int | None = None,
    timeout: float = ACK_TIMEOUT_SECONDS,
) -> None:
    """Send an idempotent command and retry transient serial acknowledgement loss."""
    last_error = f"Missing acknowledgement: {prefix}"

    for attempt in range(1, ACK_RETRIES + 1):
        port.reset_input_buffer()
        send(port, command)
        deadline = time.monotonic() + timeout
        retry_unknown = False

        while time.monotonic() < deadline:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(line, flush=True)

            if ack_matches(line, prefix, expected):
                return

            if line.startswith("ERR UNKNOWN_COMMAND"):
                last_error = line
                retry_unknown = True
                break

            if line.startswith("ERR "):
                raise RuntimeError(line)

        if attempt < ACK_RETRIES:
            reason = last_error if retry_unknown else f"Missing acknowledgement: {prefix}"
            print(
                f"WARNING: {reason}; retrying {command} ({attempt + 1}/{ACK_RETRIES})",
                flush=True,
            )
            time.sleep(ACK_RETRY_DELAY_SECONDS * attempt)

    raise RuntimeError(f"Missing acknowledgement: {prefix} after {ACK_RETRIES} attempts")


def identify(port: serial.Serial, token: str) -> str:
    for attempt in range(1, 7):
        port.reset_input_buffer()
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
            if line.startswith("ERR UNKNOWN_COMMAND"):
                break
        time.sleep(0.20 * attempt)
    raise RuntimeError("No firmware identification received")


def get_status(port: serial.Serial, retries: int = 3) -> dict[str, float]:
    for attempt in range(1, retries + 1):
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
        if attempt < retries:
            time.sleep(0.15 * attempt)
    raise RuntimeError("No STATUS response")


def configure(port: serial.Serial, a: argparse.Namespace) -> None:
    status = get_status(port)
    if int(status.get("Homed", 0)) != 1:
        time.sleep(0.75)
        status = get_status(port)

    if int(status.get("Homed", 0)) != 1 and a.trial == 1:
        print("INFO: Run 1 restoring current physical position as home.", flush=True)
        send_with_ack(port, "ZERO", "ACK ZERO", None)
        status = get_status(port)

    if int(status.get("Homed", 0)) != 1:
        raise RuntimeError("NOT_HOMED_AFTER_SERIAL_OPEN")
    if int(status.get("Fault", 0)) != 0:
        raise RuntimeError("Controller is in FAULT mode")

    commands: list[tuple[str, str, float | int | None]] = [
        (f"TRIAL={a.trial}", "ACK TRIAL=", a.trial),
        (f"KP={a.kp:g}", "ACK KP=", a.kp),
        (f"KI={a.ki:g}", "ACK KI=", a.ki),
        (f"KD={a.kd:g}", "ACK KD=", a.kd),
        (f"MAXPWM={a.return_max_pwm:g}", "ACK MAXPWM=", a.return_max_pwm),
        (f"HOMEPAD={a.home_padding_pulses}", "ACK HOMEPAD=", a.home_padding_pulses),
        (f"MINPWM={a.minimum_drive_pwm:g}", "ACK MINPWM=", a.minimum_drive_pwm),
        (f"SKP={a.setup_kp:g}", "ACK SKP=", a.setup_kp),
        (f"SKI={a.setup_ki:g}", "ACK SKI=", a.setup_ki),
        (f"SKD={a.setup_kd:g}", "ACK SKD=", a.setup_kd),
        ("TELEM=2", "ACK TELEM=", 2),
        ("HOLD", "ACK HOLD", None),
    ]
    for command, prefix, expected in commands:
        send_with_ack(port, command, prefix, expected)

    status = get_status(port)
    checks = {
        "Trial": float(a.trial), "KP": a.kp, "KI": a.ki, "KD": a.kd,
        "MaxPWM": a.return_max_pwm, "HomePad": float(a.home_padding_pulses),
        "MinPWM": a.minimum_drive_pwm, "SKP": a.setup_kp,
        "SKI": a.setup_ki, "SKD": a.setup_kd, "Homed": 1.0,
    }
    for key, expected in checks.items():
        actual = status.get(key, math.nan)
        if not math.isclose(actual, expected, abs_tol=SERIAL_FLOAT_TOL):
            raise RuntimeError(f"STATUS mismatch for {key}: expected {expected}, got {actual}")


def recover_home(port: serial.Serial, timeout: float = 20.0) -> bool:
    try:
        send_with_ack(port, "HOLD", "ACK HOLD", None)
    except (SerialException, OSError, RuntimeError):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            status = get_status(port, retries=1)
        except (SerialException, OSError, RuntimeError):
            time.sleep(0.15)
            continue
        if int(status.get("Homed", 0)) == 1 and int(status.get("Settled", 0)) == 1:
            return True
        time.sleep(0.15)
    return False


def sign_outside_deadband(value: float, deadband: float = 0.8) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0


def settling_time(rows: list[dict[str, float | str]], home_padding_pulses: int) -> float | None:
    for row in rows:
        error_counts = float(row.get("error_counts", math.nan))
        if math.isfinite(error_counts) and abs(error_counts) <= home_padding_pulses:
            return float(row["return_time"])
    return None


def overshoot_and_oscillations(rows: list[dict[str, float | str]]) -> tuple[float, int]:
    if not rows:
        return 0.0, 0
    initial_sign = 0
    first_cross_index: int | None = None
    for index, row in enumerate(rows):
        sign = sign_outside_deadband(float(row["error"]))
        if initial_sign == 0 and sign != 0:
            initial_sign = sign
        elif initial_sign != 0 and sign == -initial_sign:
            first_cross_index = index
            break
    if first_cross_index is None:
        return 0.0, 0
    opposite = [
        abs(float(row["error"])) for row in rows[first_cross_index:]
        if sign_outside_deadband(float(row["error"])) == -initial_sign
    ]
    overshoot = max(opposite, default=0.0)
    oscillations = 0
    previous_sign = -initial_sign
    for row in rows[first_cross_index + 1:]:
        sign = sign_outside_deadband(float(row["error"]))
        if sign == 0:
            continue
        if sign != previous_sign:
            oscillations += 1
            previous_sign = sign
    return overshoot, oscillations


def integrated_absolute_error(rows: list[dict[str, float | str]]) -> float:
    result = 0.0
    for previous, current in zip(rows, rows[1:]):
        dt = max(0.0, float(current["return_time"]) - float(previous["return_time"]))
        result += 0.5 * (abs(float(previous["error"])) + abs(float(current["error"]))) * dt
    return result


def write_raw(path: Path, rows: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(RAW_HEADERS)
        for row in rows:
            writer.writerow([
                row["timestamp"], f'{float(row["elapsed"]):.4f}',
                "" if float(row["return_time"]) < 0 else f'{float(row["return_time"]):.4f}',
                int(float(row["trial"])), int(float(row["mode"])), row["kp"], row["ki"], row["kd"],
                row["position"], row["target"], row["error"], row["speed_target"], row["rpm"],
                int(float(row["counts"])), row["pwm"], row["raw"],
            ])


def append_history(
    path: Path, run: int, kp: float, ki: float, kd: float,
    rows: list[dict[str, float | str]],
) -> None:
    return_rows = [row for row in rows if float(row["return_time"]) >= 0]
    if not return_rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        if new_file:
            writer.writerow(HISTORY_HEADERS)
        for row in return_rows:
            writer.writerow([
                run, f'{float(row["return_time"]):.4f}', kp, ki, kd,
                row["position"], row["error"], row["rpm"], row["pwm"],
            ])


def write_result(
    a: argparse.Namespace, firmware: str, status: str,
    rows: list[dict[str, float | str]],
) -> int:
    return_rows = [row for row in rows if float(row["return_time"]) >= 0]
    return_timer_started = int(bool(return_rows))
    settling = settling_time(return_rows, a.home_padding_pulses)
    overshoot, oscillations = overshoot_and_oscillations(return_rows)
    iae = integrated_absolute_error(return_rows)
    peak_rpm = max((abs(float(row["rpm"])) for row in return_rows), default=0.0)
    final_window = return_rows[-5:]
    final_error = (
        statistics.fmean(abs(float(row["error"])) for row in final_window)
        if final_window else 999.0
    )
    timed_out = status in {"TEST_TIMEOUT", "TEST_PREP_TIMEOUT", "TEST_SETTLING_TIMEOUT"}
    abnormal = status not in {"COMPLETE", "TEST_TIMEOUT", "TEST_PREP_TIMEOUT", "TEST_SETTLING_TIMEOUT"}
    settling_for_score = settling if settling is not None else a.max_settling_seconds
    score = (
        10.0 * settling_for_score + 0.25 * overshoot + 0.25 * oscillations
        + 0.01 * iae + 0.25 * final_error
        + (25.0 if timed_out else 0.0) + (500.0 if abnormal else 0.0)
    )
    if not math.isfinite(score):
        score = 9999.0
    success = int(
        status == "COMPLETE" and settling is not None
        and settling <= a.success_settling_seconds
        and overshoot <= a.success_overshoot_deg
        and oscillations == 0
        and final_error <= a.success_final_error_deg
    )

    a.result_file.parent.mkdir(parents=True, exist_ok=True)
    with a.result_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(RESULT_HEADERS)
        writer.writerow([
            a.trial, f"{a.kp:.8f}", f"{a.ki:.8f}", f"{a.kd:.8f}", f"{score:.8f}",
            "" if settling is None else f"{settling:.8f}", return_timer_started,
            f"{overshoot:.8f}", oscillations, f"{iae:.8f}", f"{peak_rpm:.8f}",
            f"{final_error:.8f}", success, status, len(return_rows), firmware,
            str(a.raw_file.resolve()),
        ])
    print(
        f"RESULT Run={a.trial} KP={a.kp:g} KI={a.ki:g} KD={a.kd:g} "
        f"Score={score:.6f} Settling={settling if settling is not None else 'MISSING'} "
        f"ReturnTimerStarted={return_timer_started} Success={success} Status={status}",
        flush=True,
    )
    return 0


def main() -> int:
    a = arguments()
    quantize_pid(a)

    valid = (
        a.trial >= 1 and all(math.isfinite(v) for v in (
            a.kp, a.ki, a.kd, a.test_offset, a.return_max_pwm,
            a.minimum_drive_pwm, a.setup_kp, a.setup_ki, a.setup_kd,
        ))
        and a.home_padding_pulses >= 0
        and a.overall_timeout > 0.0
        and a.max_settling_seconds > 0.0
    )
    if not valid:
        return write_result(a, "", "INVALID_ARGUMENTS", [])

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

        send_with_ack(
            port,
            f"TEST={a.test_offset:g}",
            "ACK TEST=",
            a.test_offset,
        )
        print("INFO: TEST command accepted; waiting for physical TEST_OUT motion.", flush=True)

        start = time.monotonic()
        return_start: float | None = None
        settling_reached = False
        test_complete = False
        observed_test_out = False

        while time.monotonic() - start < a.overall_timeout:
            now = time.monotonic()
            if (
                return_start is not None and not settling_reached
                and now - return_start >= a.max_settling_seconds
            ):
                status = "TEST_SETTLING_TIMEOUT"
                test_complete = True
                print(
                    f"ERROR: settling exceeded {a.max_settling_seconds:.3f} s; recovering home.",
                    flush=True,
                )
                break

            line = port.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(line, flush=True)

            if line == "EVENT TEST_OUT":
                observed_test_out = True
                continue
            if line == "EVENT TEST_RETURN":
                return_start = time.monotonic()
                print("INFO: release-to-home timer started.", flush=True)
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
                raise RuntimeError(line)

            values = telemetry(line)
            if values is None:
                continue
            if int(values["Trial"]) != a.trial:
                raise RuntimeError("Telemetry trial mismatch")
            for key, expected in (("KP", a.kp), ("KI", a.ki), ("KD", a.kd)):
                if not math.isclose(values[key], expected, abs_tol=SERIAL_FLOAT_TOL):
                    raise RuntimeError(f"Telemetry {key} mismatch: expected {expected}, got {values[key]}")
            if int(values["Homed"]) != 1:
                raise RuntimeError("Controller lost home state")

            mode = int(values["Mode"])
            if mode == 2:
                observed_test_out = True
            if mode == 4 and return_start is None:
                return_start = time.monotonic()
                print("WARNING: inferred TEST_RETURN timer start from telemetry Mode=4.", flush=True)

            now = time.monotonic()
            elapsed = now - start
            return_time = -1.0 if return_start is None else now - return_start
            if return_time >= 0 and abs(values["ErrCounts"]) <= a.home_padding_pulses:
                settling_reached = True

            rows.append({
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "elapsed": elapsed,
                "return_time": return_time,
                "trial": float(a.trial),
                "mode": float(mode),
                "kp": a.kp,
                "ki": a.ki,
                "kd": a.kd,
                "position": values["PosDeg"],
                "target": values["PosTarget"],
                "error": values["PosError"],
                "error_counts": values["ErrCounts"],
                "speed_target": values["SpeedTarget"],
                "rpm": values["OutRPM"],
                "counts": values["Counts"],
                "pwm": values["PWM"],
                "raw": line,
            })

        if not observed_test_out and not test_complete:
            raise RuntimeError("TEST accepted but TEST_OUT motion was never observed")
        if not test_complete:
            status = "WORKER_TIMEOUT"

    except (SerialException, OSError, RuntimeError, ValueError) as error:
        error_text = str(error).strip()
        status = "CONTROL_ERROR_" + re.sub(r"[^A-Za-z0-9_]+", "_", error_text).strip("_")[:120]
        print(f"ERROR: {error_text}", flush=True)
    except KeyboardInterrupt:
        status = "ABORT_USER"
    finally:
        if port is not None:
            recovered = recover_home(port)
            if not recovered:
                print("WARNING: mechanism did not confirm settled home before COM close", flush=True)
            try:
                send_with_ack(port, "TELEM=1", "ACK TELEM=", 1, timeout=1.0)
            except Exception:
                pass
            port.close()

    write_raw(a.raw_file, rows)
    append_history(a.history_file, a.trial, a.kp, a.ki, a.kd, rows)
    return write_result(a, firmware, status, rows)


if __name__ == "__main__":
    raise SystemExit(main())
