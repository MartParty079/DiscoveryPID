#!/usr/bin/env python3
"""Run one guarded DiscoveryPID trial for the JMP/JSL autotune loop."""
from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import serial
from serial import SerialException

PAIR_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=(-?(?:\d+(?:\.\d*)?|\.\d+))")
EXPECTED_FW = "DiscoveryPID jmp-autotune-v1"
FIELDS = {"Trial", "P", "I", "Target", "OutRPM", "MotorRPM", "Error", "Integral", "Counts", "dC", "PWM"}
RESULT_HEADERS = ["Run", "P", "I", "Error", "Raw MAE", "Mean Output RPM", "Minimum Output RPM", "Maximum Output RPM", "Output RPM Std Dev", "Samples", "Evaluation Samples", "Status", "Firmware", "Raw File"]
RAW_HEADERS = ["PC Timestamp", "Elapsed Time (s)", "Run", "P", "I", "Target RPM", "Output RPM", "Smoothed Output RPM", "Signed Error (RPM)", "Absolute Error (RPM)", "Smoothed Absolute Error (RPM)", "Motor RPM", "Integral", "Encoder Counts", "Delta Counts", "PWM Command", "Raw Serial Line"]
HISTORY_HEADERS = ["Run", "Elapsed Time (s)", "P", "I", "Target RPM", "Output RPM", "Smoothed Output RPM", "Signed Error (RPM)", "Absolute Error (RPM)", "PWM Command"]


def args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True)
    p.add_argument("--trial", required=True, type=int)
    p.add_argument("--p", required=True, type=float)
    p.add_argument("--i", required=True, type=float)
    p.add_argument("--target", required=True, type=float)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--warmup", type=float, default=2.0)
    p.add_argument("--smoothing-samples", type=int, default=5)
    p.add_argument("--result-file", required=True, type=Path)
    p.add_argument("--raw-file", required=True, type=Path)
    p.add_argument("--history-file", required=True, type=Path)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--expected-firmware", default=EXPECTED_FW)
    p.add_argument("--max-output-rpm", type=float, default=150.0)
    p.add_argument("--max-pwm", type=float, default=225.0)
    p.add_argument("--max-pwm-seconds", type=float, default=0.75)
    p.add_argument("--no-motion-seconds", type=float, default=2.0)
    return p.parse_args()


def pairs(line: str) -> dict[str, float]:
    return {k: float(v) for k, v in PAIR_RE.findall(line)}


def telemetry(line: str) -> dict[str, float] | None:
    values = pairs(line)
    return values if FIELDS.issubset(values) else None


def send(port: serial.Serial, text: str) -> None:
    port.write((text + "\n").encode("ascii"))
    port.flush()
    time.sleep(0.05)


def stop(port: serial.Serial) -> None:
    for text in ("STOP", "0"):
        try:
            send(port, text)
        except (SerialException, OSError):
            break


def verify_firmware(port: serial.Serial, token: str) -> str:
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(line, flush=True)
        if line.startswith("Firmware:"):
            fw = line.partition(":")[2].strip()
            if token not in fw:
                raise RuntimeError(f"Wrong firmware: {fw}")
            return fw
    raise RuntimeError("No firmware identification received")


def configure(port: serial.Serial, a: argparse.Namespace) -> None:
    stop(port)
    port.reset_input_buffer()
    for text in (f"TRIAL={a.trial}", f"P={a.p:.8g}", f"I={a.i:.8g}", "STATUS"):
        send(port, text)
    ok = {"trial": False, "p": False, "i": False, "status": False}
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not all(ok.values()):
        line = port.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        print(line, flush=True)
        if line.startswith("ACK TRIAL="):
            ok["trial"] = int(float(line.split("=", 1)[1])) == a.trial
        elif line.startswith("ACK P="):
            ok["p"] = math.isclose(float(line.split("=", 1)[1]), a.p, abs_tol=1e-4)
        elif line.startswith("ACK I="):
            ok["i"] = math.isclose(float(line.split("=", 1)[1]), a.i, abs_tol=1e-4)
        elif line.startswith("STATUS "):
            v = pairs(line)
            ok["status"] = int(v.get("Trial", -1)) == a.trial and math.isclose(v.get("P", math.nan), a.p, abs_tol=1e-4) and math.isclose(v.get("I", math.nan), a.i, abs_tol=1e-4) and abs(v.get("Target", 99)) < 0.01
    missing = [k for k, v in ok.items() if not v]
    if missing:
        raise RuntimeError("Missing controller confirmation: " + ", ".join(missing))


def write_result(a: argparse.Namespace, status: str, fw: str, rows: list[dict]) -> int:
    evaluation = [r for r in rows if r["elapsed"] >= a.warmup]
    score = raw_mae = mean_out = min_out = max_out = std_out = None
    if evaluation:
        score = statistics.fmean(r["smooth_abs"] for r in evaluation)
        raw_mae = statistics.fmean(r["abs_error"] for r in evaluation)
        out = [r["output"] for r in evaluation]
        mean_out, min_out, max_out = statistics.fmean(out), min(out), max(out)
        std_out = statistics.pstdev(out) if len(out) > 1 else 0.0
    if status == "COMPLETE" and not evaluation:
        status = "FAILED_NO_EVALUATION_SAMPLES"
    a.result_file.parent.mkdir(parents=True, exist_ok=True)
    with a.result_file.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(RESULT_HEADERS)
        w.writerow([a.trial, a.p, a.i, "" if score is None else f"{score:.8f}", "" if raw_mae is None else f"{raw_mae:.8f}", "" if mean_out is None else f"{mean_out:.8f}", "" if min_out is None else f"{min_out:.8f}", "" if max_out is None else f"{max_out:.8f}", "" if std_out is None else f"{std_out:.8f}", len(rows), len(evaluation), status, fw, str(a.raw_file.resolve())])
    if status == "COMPLETE" and score is not None:
        print(f"RESULT Run={a.trial} P={a.p:.6g} I={a.i:.6g} Error={score:.6f}", flush=True)
        return 0
    return 4


def append_history(a: argparse.Namespace, rows: list[dict]) -> None:
    a.history_file.parent.mkdir(parents=True, exist_ok=True)
    new = not a.history_file.exists() or a.history_file.stat().st_size == 0
    with a.history_file.open("a", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if new:
            w.writerow(HISTORY_HEADERS)
        for r in rows:
            w.writerow([a.trial, f'{r["elapsed"]:.4f}', a.p, a.i, a.target, r["output"], f'{r["smoothed"]:.5f}', r["error"], r["abs_error"], r["pwm"]])


def main() -> int:
    a = args()
    if not (0 <= a.p <= 10 and 0 <= a.i <= 50 and a.trial >= 1 and 0 <= a.warmup < a.duration and a.smoothing_samples >= 1 and a.max_output_rpm > abs(a.target)):
        print("Invalid trial arguments", file=sys.stderr)
        return 2
    a.raw_file.parent.mkdir(parents=True, exist_ok=True)
    a.result_file.parent.mkdir(parents=True, exist_ok=True)
    if a.result_file.exists():
        a.result_file.unlink()
    rows: list[dict] = []
    fw, status, port = "", "FAILED_BEFORE_START", None
    try:
        port = serial.Serial(a.port, a.baud, timeout=0.2, write_timeout=1.0)
        fw = verify_firmware(port, a.expected_firmware)
        configure(port, a)
        port.reset_input_buffer()
        send(port, f"RPM={a.target:.8g}")
        status, start = "RUNNING", time.monotonic()
        smooth = deque(maxlen=a.smoothing_samples)
        initial_counts = first_motion = high_pwm_since = None
        with a.raw_file.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(RAW_HEADERS)
            f.flush()
            while True:
                elapsed = time.monotonic() - start
                if elapsed >= a.duration:
                    status = "COMPLETE"
                    break
                line = port.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                v = telemetry(line)
                if v is None:
                    print(line, flush=True)
                    continue
                if int(v["Trial"]) != a.trial or not math.isclose(v["P"], a.p, abs_tol=1e-4) or not math.isclose(v["I"], a.i, abs_tol=1e-4) or not math.isclose(v["Target"], a.target, abs_tol=0.05):
                    raise RuntimeError("Telemetry configuration mismatch")
                output, pwm, counts = v["OutRPM"], v["PWM"], int(v["Counts"])
                smooth.append(output)
                smoothed = statistics.fmean(smooth)
                error, smooth_error = a.target - output, a.target - smoothed
                r = {"elapsed": elapsed, "output": output, "smoothed": smoothed, "error": error, "abs_error": abs(error), "smooth_abs": abs(smooth_error), "pwm": pwm}
                rows.append(r)
                w.writerow([datetime.now().isoformat(timespec="milliseconds"), f"{elapsed:.4f}", a.trial, a.p, a.i, a.target, output, f"{smoothed:.5f}", error, abs(error), f"{abs(smooth_error):.5f}", v["MotorRPM"], v["Integral"], counts, int(v["dC"]), pwm, line])
                f.flush()
                if abs(output) > a.max_output_rpm:
                    status = "ABORT_MAX_OUTPUT_RPM"
                    raise RuntimeError(status)
                if (a.target > 0 and pwm < -0.1) or (a.target < 0 and pwm > 0.1):
                    status = "ABORT_REVERSE_COMMAND"
                    raise RuntimeError(status)
                if abs(pwm) > a.max_pwm:
                    high_pwm_since = elapsed if high_pwm_since is None else high_pwm_since
                    if elapsed - high_pwm_since >= a.max_pwm_seconds:
                        status = "ABORT_SUSTAINED_HIGH_PWM"
                        raise RuntimeError(status)
                else:
                    high_pwm_since = None
                initial_counts = counts if initial_counts is None else initial_counts
                if first_motion is None and counts != initial_counts:
                    first_motion = elapsed
                if first_motion is None and elapsed >= a.no_motion_seconds:
                    status = "ABORT_NO_ENCODER_MOTION"
                    raise RuntimeError(status)
    except (SerialException, OSError, RuntimeError, ValueError) as e:
        print(f"ERROR: {e}", file=sys.stderr, flush=True)
        if status in {"FAILED_BEFORE_START", "RUNNING"}:
            status = "FAILED"
    except KeyboardInterrupt:
        status = "ABORT_USER"
    finally:
        if port is not None:
            stop(port)
            port.close()
    if rows:
        append_history(a, rows)
    return write_result(a, status, fw, rows)


if __name__ == "__main__":
    raise SystemExit(main())
