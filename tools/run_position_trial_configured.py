#!/usr/bin/env python3
"""Run a DiscoveryPID physical trial using configured PID resolution and deadline.

This wrapper keeps the proven physical-trial implementation in
run_position_trial.py while replacing its legacy hard-coded 0.5 PID
quantization, enforcing the configured hard return-settling deadline, and
accepting the compact 100 Hz FAST telemetry emitted by the firmware.
"""

from __future__ import annotations

import inspect
import json
import math
import sys
from pathlib import Path

import run_position_trial as worker


REPO = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO / "config" / "discoverypid.json"

# The configured worker captures the active argparse namespace so compact FAST
# telemetry does not need to repeat static KP/KI/KD values 100 times per second.
# Those gains are already written to and verified against STATUS before motion.
_original_arguments = worker.arguments
_active_args = None


def load_config() -> dict:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))


def configured_step() -> float:
    step = float(load_config()["pidCoefficientStep"])
    if not math.isfinite(step) or step <= 0.0:
        raise RuntimeError("pidCoefficientStep must be a finite value greater than zero")
    return step


def configured_deadline() -> float:
    deadline = float(load_config()["maxSettlingTimeSeconds"])
    if not math.isfinite(deadline) or deadline <= 0.0:
        raise RuntimeError("maxSettlingTimeSeconds must be a finite value greater than zero")
    return deadline


def configured_arguments():
    global _active_args
    _active_args = _original_arguments()
    return _active_args


def compact_aware_telemetry(line: str) -> dict[str, float] | None:
    """Accept legacy verbose rows plus the compact 100 Hz FAST row.

    Compact firmware format:
      F U=<ms> T=<trial> M=<mode> H=<homed> P=<pos> G=<target>
        E=<error> X=<error counts> R=<rpm> C=<counts> W=<pwm>
    """
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
    """Round a nonnegative PID coefficient to the nearest configured step."""
    return math.floor((value / step) + 0.5) * step


def quantize_pid(a) -> None:
    step = configured_step()
    original = (a.kp, a.ki, a.kd, a.setup_kp, a.setup_ki, a.setup_kd)

    a.kp = quantize(a.kp, step)
    a.ki = quantize(a.ki, step)
    a.kd = quantize(a.kd, step)
    a.setup_kp = quantize(a.setup_kp, step)
    a.setup_ki = quantize(a.setup_ki, step)
    a.setup_kd = quantize(a.setup_kd, step)

    rounded = (a.kp, a.ki, a.kd, a.setup_kp, a.setup_ki, a.setup_kd)
    if original != rounded:
        print(
            f"INFO: PID values rounded to {step:g} steps: "
            f"candidate=({a.kp:g},{a.ki:g},{a.kd:g}) "
            f"setup=({a.setup_kp:g},{a.setup_ki:g},{a.setup_kd:g})",
            flush=True,
        )
    else:
        print(
            f"INFO: PID resolution {step:g}: "
            f"candidate=({a.kp:g},{a.ki:g},{a.kd:g}) "
            f"setup=({a.setup_kp:g},{a.setup_ki:g},{a.setup_kd:g})",
            flush=True,
        )


def enforce_hard_settling_deadline() -> None:
    """Patch the legacy worker for the current configured runtime behavior."""
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
        raise RuntimeError(
            "Could not patch legacy worker hard-settling deadline safely; "
            "expected timeout condition was not found exactly once"
        )

    # At 100 Hz, echoing every compact sample to the PowerShell/JMP console can
    # become the bottleneck. Keep every row for metrics/CSV, but print events,
    # acknowledgements, STATUS, and errors only.
    old_print = '            print(line, flush=True)'
    new_print = (
        '            if not line.startswith("F "):\n'
        '                print(line, flush=True)'
    )
    if source.count(old_print) != 1:
        raise RuntimeError(
            "Could not patch legacy worker 100 Hz console throttle safely; "
            "expected serial print statement was not found exactly once"
        )

    namespace: dict = {}
    patched = source.replace(old_timeout, new_timeout, 1)
    patched = patched.replace(old_print, new_print, 1)
    exec(compile(patched, worker.__file__, "exec"), worker.__dict__, namespace)
    worker.main = namespace["main"]


def force_configured_deadline_argument() -> None:
    """Make config maxSettlingTimeSeconds authoritative for every run."""
    deadline_text = f"{configured_deadline():g}"
    option = "--max-settling-seconds"
    if option in sys.argv:
        index = sys.argv.index(option)
        if index + 1 >= len(sys.argv):
            raise RuntimeError(f"{option} was supplied without a value")
        sys.argv[index + 1] = deadline_text
    else:
        sys.argv.extend([option, deadline_text])
    print(f"INFO: hard settling deadline = {deadline_text} s", flush=True)


# Keep automated/JMP PID values on the configured grid rather than the legacy
# 0.5 grid, consume every compact 100 Hz arm-position sample, and keep the
# return deadline active until the run truly completes.
worker.arguments = configured_arguments
worker.telemetry = compact_aware_telemetry
worker.quantize_pid = quantize_pid
enforce_hard_settling_deadline()


if __name__ == "__main__":
    force_configured_deadline_argument()
    raise SystemExit(worker.main())
