#!/usr/bin/env python3
"""Run a DiscoveryPID trial with PID gains quantized to 0.5 steps.

This wrapper also makes the serial command handshake tolerant of occasional
Arduino/USB acknowledgement loss. A command is retried when its acknowledgement
is missing or the firmware reports a transient UNKNOWN_COMMAND. True value
mismatches still fail immediately.
"""

from __future__ import annotations

import math
import sys
import time
from typing import Callable


GAIN_FLAGS = {
    "--kp",
    "--ki",
    "--kd",
    "--setup-kp",
    "--setup-ki",
    "--setup-kd",
}

ACK_RETRIES = 3
ACK_RETRY_DELAY_SECONDS = 0.15


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


def command_from_ack(prefix: str, expected: float | int | None) -> str | None:
    """Reconstruct the command associated with an acknowledgement prefix."""
    if not prefix.startswith("ACK "):
        return None

    command_name = prefix[4:].rstrip("=")

    if expected is None:
        return command_name

    if isinstance(expected, int):
        value_text = str(expected)
    else:
        value_text = f"{float(expected):.9g}"

    return f"{command_name}={value_text}"


def install_ack_retry(worker: object) -> None:
    """Patch the base worker's acknowledgement wait with bounded retries."""
    original_wait: Callable[..., None] = worker.wait_for_ack

    def wait_for_ack_with_retry(
        port,
        prefix: str,
        expected: float | int | None = None,
        timeout: float = 2.0,
    ) -> None:
        command = command_from_ack(prefix, expected)
        last_error: RuntimeError | None = None

        for attempt in range(1, ACK_RETRIES + 1):
            try:
                original_wait(port, prefix, expected, timeout)
                return
            except RuntimeError as exc:
                message = str(exc)
                retryable = (
                    message.startswith("Missing acknowledgement:")
                    or message.startswith("ERR UNKNOWN_COMMAND")
                )

                if not retryable or command is None:
                    raise

                last_error = exc
                if attempt >= ACK_RETRIES:
                    break

                print(
                    f"WARNING: {message}; retrying {command} "
                    f"({attempt + 1}/{ACK_RETRIES})",
                    flush=True,
                )

                try:
                    port.reset_input_buffer()
                except Exception:
                    pass

                time.sleep(ACK_RETRY_DELAY_SECONDS * attempt)
                worker.send(port, command)

        if last_error is not None:
            raise RuntimeError(
                f"{last_error} after {ACK_RETRIES} attempts"
            ) from last_error

    worker.wait_for_ack = wait_for_ack_with_retry


def main() -> int:
    sys.argv = quantize_gain_arguments(sys.argv)

    # Import only after argv is quantized so the existing worker remains the
    # single source of truth for serial protocol, scoring, timeout, and recovery.
    import run_position_trial as worker

    install_ack_retry(worker)
    return worker.main()


if __name__ == "__main__":
    raise SystemExit(main())
