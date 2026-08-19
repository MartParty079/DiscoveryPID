from pathlib import Path
import json


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Missing anchor for {label}")
    return text.replace(old, new, 1)

# -----------------------------------------------------------------------------
# Python worker
# -----------------------------------------------------------------------------
path = Path("tools/run_position_trial.py")
text = path.read_text(encoding="utf-8")

text = replace_once(
    text,
    '    p.add_argument("--overall-timeout", type=float, default=60.0)\n',
    '    p.add_argument("--overall-timeout", type=float, default=60.0)\n'
    '    p.add_argument("--max-settling-seconds", type=float, default=3.0)\n',
    "worker max-settling argument",
)

text = replace_once(
    text,
    '    timed_out = status in {"TEST_TIMEOUT", "TEST_PREP_TIMEOUT"}\n\n'
    '    settling_for_score = (\n'
    '        settling\n'
    '        if settling is not None\n'
    '        else a.overall_timeout\n'
    '    )\n\n'
    '    abnormal_status = status not in {\n'
    '        "COMPLETE",\n'
    '        "TEST_TIMEOUT",\n'
    '        "TEST_PREP_TIMEOUT",\n'
    '    }\n',
    '    timed_out = status in {\n'
    '        "TEST_TIMEOUT",\n'
    '        "TEST_PREP_TIMEOUT",\n'
    '        "TEST_SETTLING_TIMEOUT",\n'
    '    }\n\n'
    '    settling_for_score = (\n'
    '        settling\n'
    '        if settling is not None\n'
    '        else a.max_settling_seconds\n'
    '    )\n\n'
    '    abnormal_status = status not in {\n'
    '        "COMPLETE",\n'
    '        "TEST_TIMEOUT",\n'
    '        "TEST_PREP_TIMEOUT",\n'
    '        "TEST_SETTLING_TIMEOUT",\n'
    '    }\n',
    "worker timeout scoring",
)

text = replace_once(
    text,
    '        and a.overall_timeout > 0.0\n',
    '        and a.overall_timeout > 0.0\n'
    '        and a.max_settling_seconds > 0.0\n',
    "worker argument validation",
)

text = replace_once(
    text,
    '        return_start: float | None = None\n'
    '        test_complete = False\n\n'
    '        while time.monotonic() - start < a.overall_timeout:\n'
    '            line = port.readline().decode(\n',
    '        return_start: float | None = None\n'
    '        settling_reached = False\n'
    '        test_complete = False\n\n'
    '        while time.monotonic() - start < a.overall_timeout:\n'
    '            if (\n'
    '                return_start is not None\n'
    '                and not settling_reached\n'
    '                and time.monotonic() - return_start >= a.max_settling_seconds\n'
    '            ):\n'
    '                status = "TEST_SETTLING_TIMEOUT"\n'
    '                test_complete = True\n'
    '                print(\n'
    '                    "ERROR: settling time exceeded "\n'
    '                    f"{a.max_settling_seconds:.3f} s without entering "\n'
    '                    f"+/-{a.home_padding_pulses} encoder counts; "\n'
    '                    "ending this trial and recovering home.",\n'
    '                    flush=True,\n'
    '                )\n'
    '                break\n\n'
    '            line = port.readline().decode(\n',
    "worker live settling timeout",
)

text = replace_once(
    text,
    '            now = time.monotonic()\n'
    '            elapsed = now - start\n\n'
    '            rows.append(\n'
    '                {\n'
    '                    "timestamp": time.strftime(\n',
    '            now = time.monotonic()\n'
    '            elapsed = now - start\n'
    '            return_time = (\n'
    '                -1.0\n'
    '                if return_start is None\n'
    '                else now - return_start\n'
    '            )\n\n'
    '            if (\n'
    '                return_time >= 0.0\n'
    '                and abs(values["ErrCounts"]) <= a.home_padding_pulses\n'
    '            ):\n'
    '                settling_reached = True\n\n'
    '            rows.append(\n'
    '                {\n'
    '                    "timestamp": time.strftime(\n',
    "worker live settling latch",
)

text = replace_once(
    text,
    '                    "return_time": (\n'
    '                        -1.0\n'
    '                        if return_start is None\n'
    '                        else now - return_start\n'
    '                    ),\n',
    '                    "return_time": return_time,\n',
    "worker stored return time",
)

path.write_text(text, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# JMP study
# -----------------------------------------------------------------------------
path = Path("jmp/DiscoveryPID_Position_V2.jsl")
text = path.read_text(encoding="utf-8")

text = replace_once(
    text,
    'studyName = "PositionStudy_DirectPID_v38_300Deg10xRange";',
    'studyName = "PositionStudy_DirectPID_v39_300Deg10xRange_3s";',
    "JMP study name",
)

text = replace_once(
    text,
    'homePaddingPulses = 2;\n\n'
    '// MINIMUM NONZERO MOTOR PWM.',
    'homePaddingPulses = 2;\n\n'
    '// Maximum allowed release-to-home settling time.\n'
    '// Timer starts at EVENT TEST_RETURN and stops on first entry into homePaddingPulses.\n'
    'maxSettlingTimeSeconds = 3.0;\n\n'
    '// MINIMUM NONZERO MOTOR PWM.',
    "JMP settling limit setting",
)

text = replace_once(
    text,
    'projectDirectory || "data/position_autotune_directpid_v38_300deg10xrange";',
    'projectDirectory || "data/position_autotune_directpid_v39_300deg10xrange_3s";',
    "JMP data root",
)

text = replace_once(
    text,
    '                "Home target padding: +/-" || Char( Round( homePaddingPulses ) ) || " encoder pulses\\!N" ||\n'
    '                "Minimum drive PWM: " || Char( minimumDrivePWM ) || "\\!N" ||',
    '                "Home target padding: +/-" || Char( Round( homePaddingPulses ) ) || " encoder pulses\\!N" ||\n'
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\\!N" ||\n'
    '                "Minimum drive PWM: " || Char( minimumDrivePWM ) || "\\!N" ||',
    "JMP startup summary",
)

text = replace_once(
    text,
    '            "--home-padding-pulses", Char( Round( homePaddingPulses ) ),\n'
    '            "--minimum-drive-pwm", Char( minimumDrivePWM ),',
    '            "--home-padding-pulses", Char( Round( homePaddingPulses ) ),\n'
    '            "--max-settling-seconds", Char( maxSettlingTimeSeconds ),\n'
    '            "--minimum-drive-pwm", Char( minimumDrivePWM ),',
    "JMP worker argument",
)

old_warning = '''                    If(\n                        resultList[10] == "TEST_PREP_TIMEOUT",\n\n                        "WARNING: RETURN TIMER NEVER STARTED.\\!N\\!N" ||\n'''
new_warning = '''                    If(\n                        resultList[10] == "TEST_SETTLING_TIMEOUT",\n\n                        "ERROR: SETTLING TIME EXCEEDED " ||\n                        Char( maxSettlingTimeSeconds ) ||\n                        " SECONDS.\\!N\\!N" ||\n                        "The candidate did not enter +/-" ||\n                        Char( Round( homePaddingPulses ) ) ||\n                        " encoder counts of home before the settling deadline.\\!N" ||\n                        "This run is saved as an invalid penalty row and JMP will continue to the next trial.",\n\n                        resultList[10] == "TEST_PREP_TIMEOUT",\n\n                        "WARNING: RETURN TIMER NEVER STARTED.\\!N\\!N" ||\n'''
text = replace_once(text, old_warning, new_warning, "JMP settling warning")

path.write_text(text, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# Launcher session directory
# -----------------------------------------------------------------------------
path = Path("scripts/position_v2/open_jmp.ps1")
text = path.read_text(encoding="utf-8")
text = replace_once(
    text,
    'data\\position_autotune_directpid_v38_300deg10xrange\\sessions',
    'data\\position_autotune_directpid_v39_300deg10xrange_3s\\sessions',
    "launcher session directory",
)
path.write_text(text, encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# Tracked profile config
# -----------------------------------------------------------------------------
path = Path("config/discoverypid.json")
cfg = json.loads(path.read_text(encoding="utf-8"))
cfg["profileName"] = "Direct PID / 300-degree Bayesian study - 10x PID range / 3 s settling max"
cfg["studyName"] = "PositionStudy_DirectPID_v39_300Deg10xRange_3s"
cfg["maxSettlingTimeSeconds"] = 3.0
path.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8", newline="\n")

# -----------------------------------------------------------------------------
# README
# -----------------------------------------------------------------------------
path = Path("README.md")
text = path.read_text(encoding="utf-8")
text = replace_once(
    text,
    '- Kd search: **0 to 200**\n',
    '- Kd search: **0 to 200**\n- maximum scored settling time: **3.0 s**\n',
    "README latest profile",
)
text = replace_once(
    text,
    'enter +/-2 encoder counts of home\n'
    '                         <- timer stops\n',
    'enter +/-2 encoder counts of home\n'
    '                         <- timer stops\n'
    '3.0 s without entering home window\n'
    '                         <- trial errors, gets a penalty, then study continues\n',
    "README settling definition",
)
text = replace_once(
    text,
    '- return timeout: **45 s**\n'
    '- Python overall timeout: **60 s**\n',
    '- maximum scored settling time: **3.0 s**\n'
    '- firmware return/recovery timeout: **45 s**\n'
    '- Python overall timeout: **60 s**\n',
    "README defaults",
)
path.write_text(text, encoding="utf-8", newline="\n")

print("Applied 3-second release-to-home settling timeout.")
