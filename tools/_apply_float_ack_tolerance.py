from pathlib import Path

worker_path = Path("tools/run_position_trial.py")
ci_path = Path(".github/workflows/ci.yml")

worker = worker_path.read_text(encoding="utf-8")
anchor = 'EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange"\n'
constant = (
    anchor
    + "\n# Arduino Mega uses 32-bit float values and prints gains with finite serial precision.\n"
    + "# Keep protocol verification strict, but allow sub-millith gain quantization.\n"
    + "SERIAL_FLOAT_ABS_TOLERANCE = 1e-3\n"
)
if "SERIAL_FLOAT_ABS_TOLERANCE" not in worker:
    if anchor not in worker:
        raise RuntimeError("EXPECTED_FW anchor not found")
    worker = worker.replace(anchor, constant, 1)

count = worker.count("abs_tol=1e-4")
if count < 2:
    raise RuntimeError(f"Expected at least 2 strict serial tolerances, found {count}")
worker = worker.replace(
    "abs_tol=1e-4",
    "abs_tol=SERIAL_FLOAT_ABS_TOLERANCE",
)
worker_path.write_text(worker, encoding="utf-8", newline="\n")

ci = ci_path.read_text(encoding="utf-8")
ci_anchor = "          assert 'default=60.0' in py\n"
ci_add = (
    ci_anchor
    + "          assert 'SERIAL_FLOAT_ABS_TOLERANCE = 1e-3' in py\n"
    + "          assert 'abs_tol=1e-4' not in py\n"
)
if "SERIAL_FLOAT_ABS_TOLERANCE = 1e-3" not in ci:
    if ci_anchor not in ci:
        raise RuntimeError("CI anchor not found")
    ci = ci.replace(ci_anchor, ci_add, 1)
ci_path.write_text(ci, encoding="utf-8", newline="\n")

print("Updated serial float verification tolerance to 1e-3")
