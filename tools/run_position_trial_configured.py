#!/usr/bin/env python3
"""Run DiscoveryPID trials with config-driven PID resolution and home-reach metrics.

The legacy worker remains the serial/control transport. This wrapper makes the
current study behavior authoritative: 0.001-grid candidate gains, fixed
outbound/recovery PID, compact 100 Hz telemetry, a five-second hard return
window, stable HomePad completion, continuous Home Shortfall feedback, and an
exact firmware identity check before any physical motion.
"""

from __future__ import annotations

import csv
import inspect
import json
import math
import statistics
import sys
import time
from pathlib import Path

from serial import SerialException

import run_position_trial as worker


REPO = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO / "config" / "discoverypid.json"

_original_arguments = worker.arguments
_active_args = None


def load_config() -> dict:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def configured_step() -> float:
    step = float(load_config()["pidCoefficientStep"])
    if not math.isfinite(step) or step <= 0.0:
        raise RuntimeError("pidCoefficientStep must be finite and greater than zero")
    return step


def configured_deadline() -> float:
    deadline = float(load_config()["maxSettlingTimeSeconds"])
    if not math.isfinite(deadline) or deadline <= 0.0:
        raise RuntimeError("maxSettlingTimeSeconds must be finite and greater than zero")
    return deadline


def fixed_setup_pid() -> tuple[float, float, float]:
    setup = load_config()["fixedSetupPID"]
    return float(setup["kp"]), float(setup["ki"]), float(setup["kd"])


def configured_arguments():
    global _active_args
    config = load_config()
    a = _original_arguments()
    a.setup_kp, a.setup_ki, a.setup_kd = fixed_setup_pid()
    a.expected_firmware = str(config["firmwareId"])
    a.host_settling_time = None
    _active_args = a
    print(f"INFO: required firmware = {a.expected_firmware}", flush=True)
    return a


def compact_aware_telemetry(line: str) -> dict[str, float] | None:
    """Accept the legacy verbose row and the compact firmware 100 Hz row."""
    values = worker.numeric_pairs(line)

    if worker.TELEMETRY_FIELDS.issubset(values):
        return values

    compact_fields = {"U", "T", "M", "H", "P", "G", "E", "X", "R", "C", "W"}
    if not line.startswith("F ") or not compact_fields.issubset(values):
        return None
    if _active_args is None:
        return None

    return {
        "ControllerMs": values["U"],
        "Trial": values["T"],
        "Mode": values["M"],
        "Homed": values["H"],
        "KP": float(_active_args.kp),
        "KI": float(_active_args.ki),
        "KD": float(_active_args.kd),
        "PosDeg": values["P"],
        "PosTarget": values["G"],
        "PosError": values["E"],
        "ErrCounts": values["X"],
        "SpeedTarget": 0.0,
        "OutRPM": values["R"],
        "Counts": values["C"],
        "PWM": values["W"],
    }


def quantize(value: float, step: float) -> float:
    return math.floor((value / step) + 0.5) * step


def quantize_pid(a) -> None:
    """Quantize only the candidate PID; fixed setup PID is not a search factor."""
    step = configured_step()
    original = (a.kp, a.ki, a.kd)
    a.kp = quantize(a.kp, step)
    a.ki = quantize(a.ki, step)
    a.kd = quantize(a.kd, step)
    a.setup_kp, a.setup_ki, a.setup_kd = fixed_setup_pid()

    if original != (a.kp, a.ki, a.kd):
        print(
            f"INFO: candidate PID rounded to {step:g}: "
            f"({a.kp:g},{a.ki:g},{a.kd:g})",
            flush=True,
        )
    print(
        f"INFO: fixed outbound/recovery PID = "
        f"({a.setup_kp:g},{a.setup_ki:g},{a.setup_kd:g})",
        flush=True,
    )


def configured_recover_home(port, timeout: float = 20.0) -> bool:
    """Recover using the fixed setup PID, never the failed candidate."""
    kp, ki, kd = fixed_setup_pid()
    try:
        worker.send_with_ack(port, f"KP={kp:g}", "ACK KP=", kp)
        worker.send_with_ack(port, f"KI={ki:g}", "ACK KI=", ki)
        worker.send_with_ack(port, f"KD={kd:g}", "ACK KD=", kd)
        worker.send_with_ack(port, "HOLD", "ACK HOLD", None)
    except (SerialException, OSError, RuntimeError):
        return False

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            status = worker.get_status(port, retries=1)
        except (SerialException, OSError, RuntimeError):
            time.sleep(0.10)
            continue
        if int(status.get("Homed", 0)) == 1 and int(status.get("Settled", 0)) == 1:
            return True
        time.sleep(0.10)
    return False


def return_rows(rows):
    return [row for row in rows if float(row["return_time"]) >= 0.0]


def home_reach_metrics(rows, home_padding_pulses: int, test_offset: float) -> tuple[int, float]:
    """Return (Home Reached, Home Shortfall deg).

    Home is reached if a 100 Hz sample enters HomePad or two consecutive return
    samples cross exact zero. Otherwise shortfall is the closest absolute
    position error observed. This preserves useful gradient information from
    stalled/short returns instead of collapsing every failure to one value.
    """
    rr = return_rows(rows)
    if not rr:
        return 0, abs(float(test_offset))

    closest = min(abs(float(row["error"])) for row in rr)
    previous_error = None
    reached = False

    for row in rr:
        error = float(row["error"])
        error_counts = float(row.get("error_counts", math.nan))
        if math.isfinite(error_counts) and abs(error_counts) <= home_padding_pulses:
            reached = True
            break
        if previous_error is not None and error != 0.0 and previous_error != 0.0:
            if (error > 0.0) != (previous_error > 0.0):
                reached = True
                break
        previous_error = error

    return int(reached), 0.0 if reached else closest


def configured_write_result(a, firmware: str, status: str, rows) -> int:
    rr = return_rows(rows)
    timer_started = int(bool(rr))
    home_reached, home_shortfall = home_reach_metrics(
        rows, a.home_padding_pulses, a.test_offset
    )

    completion_time = getattr(a, "host_settling_time", None)
    if status == "COMPLETE":
        if completion_time is None:
            completion_time = float(rr[-1]["return_time"]) if rr else a.max_settling_seconds
        settling = min(float(completion_time), float(a.max_settling_seconds))
    else:
        settling = float(a.max_settling_seconds)

    entry_rpm = worker.home_entry_rpm(rr, a.home_padding_pulses)
    overshoot, oscillations = worker.overshoot_and_oscillations(rr)
    chatter = worker.pwm_chatter_penalty(rr, a.return_max_pwm)
    iae = worker.integrated_absolute_error(rr)
    peak_rpm = max((abs(float(row["rpm"])) for row in rr), default=0.0)
    final_window = rr[-5:]
    final_error = (
        statistics.fmean(abs(float(row["error"])) for row in final_window)
        if final_window else abs(float(a.test_offset))
    )

    output_status = status
    if status in {"TEST_SETTLING_TIMEOUT", "TEST_TIMEOUT"}:
        output_status = "HOME_NOT_SETTLED_TIMEOUT" if home_reached else "HOME_SHORTFALL_TIMEOUT"

    cfg = load_config()
    score_cfg = cfg["scoreObjective"]
    score = (
        float(score_cfg["homeShortfallWeight"]) * home_shortfall
        + float(score_cfg["settlingTimeWeight"]) * settling
        + float(score_cfg["overshootWeight"]) * overshoot
        + float(score_cfg["oscillationWeight"]) * oscillations
    )
    if output_status not in {
        "COMPLETE", "HOME_SHORTFALL_TIMEOUT", "HOME_NOT_SETTLED_TIMEOUT"
    }:
        score += float(score_cfg["abnormalPenalty"])
    if not math.isfinite(score):
        score = 99999.0

    success = int(output_status == "COMPLETE")

    headers = [
        "Run", "KP", "KI", "KD", "Score", "Home Reached",
        "Home Shortfall (deg)", "Settling Time (s)", "Return Timer Started",
        "Overshoot (deg)", "Oscillations", "PWM Chatter", "Home Entry RPM",
        "IAE (deg*s)", "Peak Output RPM", "Final Error (deg)", "Success",
        "Status", "Samples", "Firmware", "Raw File",
    ]

    a.result_file.parent.mkdir(parents=True, exist_ok=True)
    with a.result_file.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerow([
            a.trial, f"{a.kp:.8f}", f"{a.ki:.8f}", f"{a.kd:.8f}", f"{score:.8f}",
            home_reached, f"{home_shortfall:.8f}", f"{settling:.8f}", timer_started,
            f"{overshoot:.8f}", oscillations, f"{chatter:.8f}",
            "" if entry_rpm is None else f"{entry_rpm:.8f}", f"{iae:.8f}",
            f"{peak_rpm:.8f}", f"{final_error:.8f}", success, output_status,
            len(rr), firmware, str(a.raw_file.resolve()),
        ])

    print(
        f"RESULT Run={a.trial} KP={a.kp:g} KI={a.ki:g} KD={a.kd:g} "
        f"HomeReached={home_reached} Shortfall={home_shortfall:.4f}deg "
        f"Settling={settling:.4f}s Overshoot={overshoot:.4f}deg "
        f"Oscillations={oscillations} Status={output_status}",
        flush=True,
    )
    return 0


def patch_worker_main() -> None:
    """Make HomePad stability and the five-second deadline authoritative."""
    source = inspect.getsource(worker.main)

    old_timeout = (
        "return_start is not None and not settling_reached\n"
        "                and now - return_start >= a.max_settling_seconds"
    )
    new_timeout = (
        "return_start is not None\n"
        "                and now - return_start >= a.max_settling_seconds"
    )
    if source.count(old_timeout) != 1:
        raise RuntimeError("worker timeout patch point was not found exactly once")
    source = source.replace(old_timeout, new_timeout, 1)

    old_state = "        observed_test_out = False\n"
    new_state = (
        "        observed_test_out = False\n"
        "        host_home_stable_samples = 0\n"
        "        host_previous_count = None\n"
    )
    if source.count(old_state) != 1:
        raise RuntimeError("worker host-settle state patch point was not found exactly once")
    source = source.replace(old_state, new_state, 1)

    old_event = (
        '            if line == "EVENT TEST_COMPLETE":\n'
        '                status = "COMPLETE"\n'
        '                test_complete = True\n'
        '                break\n'
    )
    new_event = (
        '            if line == "EVENT TEST_COMPLETE":\n'
        '                status = "COMPLETE"\n'
        '                if return_start is not None:\n'
        '                    a.host_settling_time = time.monotonic() - return_start\n'
        '                test_complete = True\n'
        '                break\n'
    )
    if source.count(old_event) != 1:
        raise RuntimeError("worker TEST_COMPLETE patch point was not found exactly once")
    source = source.replace(old_event, new_event, 1)

    old_sample = (
        '            return_time = -1.0 if return_start is None else now - return_start\n'
        '            if return_time >= 0 and abs(values["ErrCounts"]) <= a.home_padding_pulses:\n'
        '                settling_reached = True\n'
    )
    new_sample = (
        '            return_time = -1.0 if return_start is None else now - return_start\n'
        '            if return_time >= 0:\n'
        '                current_encoder_count = int(values["Counts"])\n'
        '                inside_home_pad = abs(values["ErrCounts"]) <= a.home_padding_pulses\n'
        '                no_new_count = (host_previous_count is not None and current_encoder_count == host_previous_count)\n'
        '                if inside_home_pad and no_new_count:\n'
        '                    host_home_stable_samples += 1\n'
        '                else:\n'
        '                    host_home_stable_samples = 0\n'
        '                host_previous_count = current_encoder_count\n'
        '                if host_home_stable_samples >= 5:\n'
        '                    a.host_settling_time = return_time\n'
        '                    status = "COMPLETE"\n'
        '                    test_complete = True\n'
        '                    print(f"INFO: HomePad stable for 5 samples; settled at {return_time:.4f} s.", flush=True)\n'
        '                    break\n'
        '                if inside_home_pad:\n'
        '                    settling_reached = True\n'
    )
    if source.count(old_sample) != 1:
        raise RuntimeError("worker 100 Hz HomePad patch point was not found exactly once")
    source = source.replace(old_sample, new_sample, 1)

    old_print = '            print(line, flush=True)'
    new_print = (
        '            if not line.startswith("F "):\n'
        '                print(line, flush=True)'
    )
    if source.count(old_print) != 1:
        raise RuntimeError("worker console-throttle patch point was not found exactly once")
    source = source.replace(old_print, new_print, 1)

    namespace: dict = {}
    exec(compile(source, worker.__file__, "exec"), worker.__dict__, namespace)
    worker.main = namespace["main"]


def force_configured_deadline_argument() -> None:
    deadline_text = f"{configured_deadline():g}"
    option = "--max-settling-seconds"
    if option in sys.argv:
        index = sys.argv.index(option)
        if index + 1 >= len(sys.argv):
            raise RuntimeError(f"{option} was supplied without a value")
        sys.argv[index + 1] = deadline_text
    else:
        sys.argv.extend([option, deadline_text])
    print(f"INFO: hard return deadline = {deadline_text} s", flush=True)


worker.arguments = configured_arguments
worker.telemetry = compact_aware_telemetry
worker.quantize_pid = quantize_pid
worker.recover_home = configured_recover_home
worker.write_result = configured_write_result
patch_worker_main()


if __name__ == "__main__":
    force_configured_deadline_argument()
    raise SystemExit(worker.main())
