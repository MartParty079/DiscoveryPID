from pathlib import Path
import json


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Anchor not found for {label}")
    return text.replace(old, new, 1)


# -----------------------------------------------------------------------------
# Python worker: quantize every candidate/setup gain before serial verification.
# -----------------------------------------------------------------------------
worker_path = Path("tools/run_position_trial.py")
worker = worker_path.read_text(encoding="utf-8")

telemetry_anchor = '''def telemetry(line: str) -> dict[str, float] | None:\n    values = numeric_pairs(line)\n    return values if TELEMETRY_FIELDS.issubset(values) else None\n\n\n'''
telemetry_new = telemetry_anchor + '''def round_gain_half(value: float) -> float:\n    \"\"\"Quantize nonnegative PID gains to the nearest 0.5.\"\"\"\n    return math.floor(value * 2.0 + 0.5) / 2.0\n\n\n'''
if "def round_gain_half" not in worker:
    worker = replace_once(worker, telemetry_anchor, telemetry_new, "worker half-step helper")

main_anchor = '''def main() -> int:\n    a = arguments()\n\n'''
main_new = '''def main() -> int:\n    a = arguments()\n\n    # JMP can propose many decimal places. The Arduino test is intentionally\n    # performed only on 0.5-step PID coefficients so the table, serial command,\n    # and physical controller all refer to the same realizable values.\n    original_candidate = (a.kp, a.ki, a.kd)\n    original_setup = (a.setup_kp, a.setup_ki, a.setup_kd)\n\n    a.kp = round_gain_half(a.kp)\n    a.ki = round_gain_half(a.ki)\n    a.kd = round_gain_half(a.kd)\n    a.setup_kp = round_gain_half(a.setup_kp)\n    a.setup_ki = round_gain_half(a.setup_ki)\n    a.setup_kd = round_gain_half(a.setup_kd)\n\n    if original_candidate != (a.kp, a.ki, a.kd):\n        print(\n            \"INFO: rounded candidate PID to nearest 0.5: \"\n            f\"KP={a.kp:g} KI={a.ki:g} KD={a.kd:g}\",\n            flush=True,\n        )\n\n    if original_setup != (a.setup_kp, a.setup_ki, a.setup_kd):\n        print(\n            \"INFO: rounded setup PID to nearest 0.5: \"\n            f\"KP={a.setup_kp:g} KI={a.setup_ki:g} KD={a.setup_kd:g}\",\n            flush=True,\n        )\n\n'''
if "original_candidate = (a.kp, a.ki, a.kd)" not in worker:
    worker = replace_once(worker, main_anchor, main_new, "worker main quantization")

worker_path.write_text(worker, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# JMP: quantize BO proposals before running and store the actual tested values.
# -----------------------------------------------------------------------------
jsl_path = Path("jmp/DiscoveryPID_Position_V2.jsl")
jsl = jsl_path.read_text(encoding="utf-8")
jsl = replace_once(
    jsl,
    'studyName = "PositionStudy_DirectPID_v39_300Deg10xRange_3s";',
    'studyName = "PositionStudy_DirectPID_v40_300Deg10xRange_3s_HalfStep";',
    "study name",
)
jsl = replace_once(
    jsl,
    'projectDirectory || "data/position_autotune_directpid_v39_300deg10xrange_3s";',
    'projectDirectory || "data/position_autotune_directpid_v40_300deg10xrange_3s_halfstep";',
    "data root",
)

as_number_anchor = '''asNumber = Function( {value},\n    Num( Char( value ) )\n);\n\n'''
as_number_new = as_number_anchor + '''roundToHalf = Function( {value},\n    Round( value * 2 ) / 2\n);\n\n'''
if "roundToHalf = Function" not in jsl:
    jsl = replace_once(jsl, as_number_anchor, as_number_new, "JSL round helper")

seed_anchor = '''        seedKp = seedSettings[seedIndex][1];\n        seedKi = seedSettings[seedIndex][2];\n        seedKd = seedSettings[seedIndex][3];\n\n'''
seed_new = '''        seedKp = roundToHalf( seedSettings[seedIndex][1] );\n        seedKi = roundToHalf( seedSettings[seedIndex][2] );\n        seedKd = roundToHalf( seedSettings[seedIndex][3] );\n\n'''
jsl = replace_once(jsl, seed_anchor, seed_new, "seed quantization")

proposal_anchor = '''        proposedKp = Column( dt, "KP" )[newRow];\n        proposedKi = Column( dt, "KI" )[newRow];\n        proposedKd = Column( dt, "KD" )[newRow];\n\n'''
proposal_new = '''        proposedKp = roundToHalf( Column( dt, "KP" )[newRow] );\n        proposedKi = roundToHalf( Column( dt, "KI" )[newRow] );\n        proposedKd = roundToHalf( Column( dt, "KD" )[newRow] );\n\n        // Store the exact coefficients that will be physically tested.\n        // Bayesian Optimization therefore learns from realizable 0.5-step gains.\n        Column( dt, "KP" )[newRow] = proposedKp;\n        Column( dt, "KI" )[newRow] = proposedKi;\n        Column( dt, "KD" )[newRow] = proposedKd;\n\n'''
jsl = replace_once(jsl, proposal_anchor, proposal_new, "Bayesian proposal quantization")

startup_anchor = '''                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\\!N" ||\n                "Minimum drive PWM: " || Char( minimumDrivePWM ) || "\\!N" ||\n'''
startup_new = '''                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\\!N" ||\n                "PID coefficient increment: 0.5\\!N" ||\n                "Minimum drive PWM: " || Char( minimumDrivePWM ) || "\\!N" ||\n'''
jsl = replace_once(jsl, startup_anchor, startup_new, "startup display")

jsl_path.write_text(jsl, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# Runtime launcher session folder.
# -----------------------------------------------------------------------------
launcher_path = Path("scripts/position_v2/open_jmp.ps1")
launcher = launcher_path.read_text(encoding="utf-8")
launcher = replace_once(
    launcher,
    'data\\position_autotune_directpid_v39_300deg10xrange_3s\\sessions',
    'data\\position_autotune_directpid_v40_300deg10xrange_3s_halfstep\\sessions',
    "launcher session path",
)
launcher_path.write_text(launcher, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# Tracked profile metadata.
# -----------------------------------------------------------------------------
config_path = Path("config/discoverypid.json")
cfg = json.loads(config_path.read_text(encoding="utf-8"))
cfg["profileName"] = "Direct PID / 300-degree Bayesian study - 10x PID range / 3 s settling max / 0.5 PID steps"
cfg["studyName"] = "PositionStudy_DirectPID_v40_300Deg10xRange_3s_HalfStep"
cfg["pidCoefficientStep"] = 0.5
config_path.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")

# -----------------------------------------------------------------------------
# README.
# -----------------------------------------------------------------------------
readme_path = Path("README.md")
readme = readme_path.read_text(encoding="utf-8")
readme = replace_once(
    readme,
    '- maximum scored settling time: **3.0 s**\n',
    '- maximum scored settling time: **3.0 s**\n- PID coefficients are tested in **0.5 increments**\n',
    "README feature list",
)
readme = replace_once(
    readme,
    '- Python overall timeout: **60 s**\n',
    '- Python overall timeout: **60 s**\n- PID coefficient increment: **0.5**\n',
    "README defaults",
)
readme_path.write_text(readme, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# CI regression checks.
# -----------------------------------------------------------------------------
ci_path = Path(".github/workflows/ci.yml")
ci = ci_path.read_text(encoding="utf-8")
ci = replace_once(
    ci,
    "          assert cfg['maxSettlingTimeSeconds'] == 3.0\n",
    "          assert cfg['maxSettlingTimeSeconds'] == 3.0\n          assert cfg['pidCoefficientStep'] == 0.5\n",
    "CI config step",
)
ci = replace_once(
    ci,
    "          assert 'studyName = \"PositionStudy_DirectPID_v39_300Deg10xRange_3s\";' in jsl\n",
    "          assert 'studyName = \"PositionStudy_DirectPID_v40_300Deg10xRange_3s_HalfStep\";' in jsl\n          assert 'roundToHalf = Function' in jsl\n          assert 'proposedKp = roundToHalf' in jsl\n          assert 'def round_gain_half' in py\n          assert 'original_candidate = (a.kp, a.ki, a.kd)' in py\n",
    "CI study checks",
)
ci_path.write_text(ci, encoding="utf-8", newline="\n")

# Remove temporary patch infrastructure from the final branch commit.
Path("tools/_apply_float_ack_tolerance.py").unlink(missing_ok=True)
Path(".github/workflows/apply-half-step-pid.yml").unlink(missing_ok=True)

print("Applied 0.5-step PID quantization and cleaned temporary patch files.")
