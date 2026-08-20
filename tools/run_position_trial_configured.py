#!/usr/bin/env python3
"""Run a DiscoveryPID physical trial using the configured PID coefficient step.

This wrapper keeps the proven physical-trial implementation in
run_position_trial.py unchanged while replacing its legacy hard-coded 0.5 PID
quantization with config/discoverypid.json::pidCoefficientStep.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import run_position_trial as worker


REPO = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO / "config" / "discoverypid.json"


def configured_step() -> float:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    step = float(config["pidCoefficientStep"])
    if not math.isfinite(step) or step <= 0.0:
        raise RuntimeError("pidCoefficientStep must be a finite value greater than zero")
    return step


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


# run_position_trial.main() resolves quantize_pid through its module globals at
# runtime, so replacing this function removes the legacy 0.5 quantizer without
# duplicating any serial, recovery, scoring, or CSV logic.
worker.quantize_pid = quantize_pid


if __name__ == "__main__":
    raise SystemExit(worker.main())
