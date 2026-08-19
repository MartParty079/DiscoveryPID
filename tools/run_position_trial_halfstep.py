#!/usr/bin/env python3
"""Run a DiscoveryPID trial with all PID coefficients quantized to 0.5 steps."""

from __future__ import annotations

import math
import sys


GAIN_FLAGS = {
    "--kp",
    "--ki",
    "--kd",
    "--setup-kp",
    "--setup-ki",
    "--setup-kd",
}


def round_half(value: float) -> float:
    """Round nonnegative PID gains to the nearest 0.5."""
    return math.floor(value * 2.0 + 0.5) / 2.0


def quantize_gain_arguments(argv: list[str]) -> list[str]:
    result = list(argv)

    for index, token in enumerate(result[:-1]):
        if token not in GAIN_FLAGS:
            continue

        original = float(result[index + 1])
        rounded = round_half(original)
        result[index + 1] = f"{rounded:.1f}"

        if not math.isclose(original, rounded, abs_tol=1e-12):
            print(
                f"INFO: {token} rounded from {original:.9g} to {rounded:.1f}",
                flush=True,
            )

    return result


def main() -> int:
    sys.argv = quantize_gain_arguments(sys.argv)

    # Import only after argv is quantized so the existing worker remains the
    # single source of truth for serial protocol, scoring, timeout, and recovery.
    from run_position_trial import main as run_trial

    return run_trial()


if __name__ == "__main__":
    raise SystemExit(main())
