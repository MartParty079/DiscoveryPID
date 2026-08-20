#!/usr/bin/env python3
"""Build the machine-specific JMP runtime script from the tracked JSL template."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


def args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo", required=True, type=Path)
    p.add_argument("--port", required=True)
    p.add_argument("--python", required=True, dest="python_executable")
    p.add_argument("--output", required=True, type=Path)
    return p.parse_args()


def replace_required(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Runtime JSL build failed: {label} source text not found")
    return text.replace(old, new, 1)


def regex_required(text: str, pattern: str, replacement: str, label: str) -> str:
    updated, count = re.subn(pattern, replacement, text, count=1, flags=re.S)
    if count != 1:
        raise RuntimeError(f"Runtime JSL build failed: {label} patch count={count}")
    return updated


def jsl_path(path: Path) -> str:
    return path.resolve().as_posix()


def jsl_number(value: float | int) -> str:
    return f"{float(value):g}"


def response_limit_block(response: dict[str, object]) -> str:
    if str(response.get("goal", "")).lower() != "minimize":
        raise RuntimeError(f"Only minimize response goals are supported: {response}")

    name = str(response["name"])
    lower_x, lower_d = response["lower"]  # type: ignore[misc]
    middle_x, middle_d = response["middle"]  # type: ignore[misc]
    upper_x, upper_d = response["upper"]  # type: ignore[misc]
    importance = response["importance"]

    return f'''    Column( dt, "{name}" ) << Set Property(
        "Response Limits",
        {{Lower( {jsl_number(lower_x)}, {jsl_number(lower_d)} ),
         Middle( {jsl_number(middle_x)}, {jsl_number(middle_d)} ),
         Upper( {jsl_number(upper_x)}, {jsl_number(upper_d)} ),
         Goal( Minimize ), Importance( {jsl_number(importance)} ), Show Limits( 1 )}}
    );'''


def main() -> int:
    a = args()
    repo = a.repo.resolve()
    config = json.loads((repo / "config" / "discoverypid.json").read_text(encoding="utf-8"))
    source = repo / "jmp" / "DiscoveryPID_Position_V2.jsl"
    worker = repo / "tools" / "run_position_trial_configured.py"
    text = source.read_text(encoding="utf-8")

    kp_low, kp_high = config["pidRanges"]["kp"]
    ki_low, ki_high = config["pidRanges"]["ki"]
    kd_low, kd_high = config["pidRanges"]["kd"]
    max_settle = float(config["maxSettlingTimeSeconds"])
    pid_step = float(config["pidCoefficientStep"])
    if pid_step <= 0.0:
        raise RuntimeError("pidCoefficientStep must be greater than zero")
    pid_step_jsl = jsl_number(pid_step)
    study_name = config["studyName"]
    data_root_name = config["dataRoot"]
    bo_config = config["bayesianOptimization"]
    minimum_rsquare = float(bo_config["minimumRSquare"])
    responses = bo_config["responses"]
    transforms = config["dataPolicy"]["responseTransforms"]
    chatter_cap = float(transforms["pwmChatter"]["cap"])
    overshoot_cap = float(transforms["overshootDeg"]["cap"])
    entry_rpm_cap = float(transforms["homeEntryRpm"]["cap"])
    oscillation_cap = float(transforms["oscillations"]["cap"])

    # Machine-specific paths / identity.
    text = regex_required(
        text,
        r'projectDirectory\s*=\s*"[^"]*";',
        f'projectDirectory =\n    "{jsl_path(repo)}/";',
        "project directory",
    )
    text = regex_required(
        text,
        r'pythonExecutable\s*=\s*"[^"]*";',
        f'pythonExecutable =\n    "{Path(a.python_executable).as_posix()}";',
        "Python executable",
    )
    text = regex_required(
        text,
        r'portName\s*=\s*"[^"]*";',
        f'portName = "{a.port}";',
        "serial port",
    )
    text = regex_required(
        text,
        r'workerScript\s*=\s*projectDirectory\s*\|\|\s*"tools/run_position_trial\.py";',
        f'workerScript =\n    "{jsl_path(worker)}";',
        "worker path",
    )
    text = replace_required(
        text,
        'studyName = "PositionStudy_DirectPID_v39_300Deg10xRange_3s";',
        f'studyName = "{study_name}";',
        "study identity",
    )
    text = replace_required(
        text,
        'data/position_autotune_directpid_v39_300deg10xrange_3s',
        f'data/{data_root_name}',
        "data root",
    )

    # Experimental region is sourced from config so Manual and JMP stay aligned.
    text = replace_required(text, 'kpLow = 0.5;', f'kpLow = {kp_low:.1f};', "Kp low")
    text = replace_required(text, 'kpHigh = 1000.0;', f'kpHigh = {kp_high:.1f};', "Kp high")
    text = replace_required(text, 'kiLow = 0.0;', f'kiLow = {ki_low:.1f};', "Ki low")
    text = replace_required(text, 'kiHigh = 500.0;', f'kiHigh = {ki_high:.1f};', "Ki high")
    text = replace_required(text, 'kdLow = 0.0;', f'kdLow = {kd_low:.1f};', "Kd low")
    text = replace_required(text, 'kdHigh = 200.0;', f'kdHigh = {kd_high:.1f};', "Kd high")

    seed_lines = ["seedSettings = {"]
    for index, (kp, ki, kd) in enumerate(config["seedSettings"]):
        comma = "," if index < len(config["seedSettings"]) - 1 else ""
        seed_lines.append(
            f"    {{{jsl_number(kp)}, {jsl_number(ki)}, {jsl_number(kd)}}}{comma}"
        )
    seed_lines.append("};")
    text = regex_required(
        text,
        r'seedSettings\s*=\s*\{.*?\n\};',
        "\n".join(seed_lines),
        "seed design",
    )

    # Add bounded Bayesian responses plus raw diagnostics to the live table.
    old_columns = '''        New Column(
            "Oscillations",
            Numeric,
            Ordinal,
            Format( "Best", 12 )
        ),

        New Column(
            "IAE (deg*s)",'''
    new_columns = '''        New Column(
            "Oscillations",
            Numeric,
            Ordinal,
            Format( "Best", 12 )
        ),

        New Column(
            "PWM Chatter",
            Numeric,
            Continuous,
            Format( "Fixed Dec", 12, 6 )
        ),

        New Column(
            "Raw PWM Chatter",
            Numeric,
            Continuous,
            Format( "Fixed Dec", 12, 6 )
        ),

        New Column(
            "Home Entry RPM",
            Numeric,
            Continuous,
            Format( "Fixed Dec", 12, 6 )
        ),

        New Column(
            "Return Failure",
            Numeric,
            Continuous,
            Format( "Best", 12 )
        ),

        New Column(
            "Raw Overshoot (deg)",
            Numeric,
            Continuous,
            Format( "Fixed Dec", 12, 6 )
        ),

        New Column(
            "IAE (deg*s)",'''
    text = replace_required(text, old_columns, new_columns, "multi-response columns")

    old_limits = '''    // Safe here because Score was just created explicitly as Numeric.
    Column( dt, "Score" ) <<
        Set Property(
            "Response Limits",
            {
                Goal( Minimize ),
                Importance( 1 ),
                Show Limits( 0 )
            }
        );

    dt;
);'''

    response_blocks = "\n".join(response_limit_block(response) for response in responses)
    new_limits = f'''    // Score remains a diagnostic/setup-ranking value only.
    Column( dt, "Score" ) <<
        Set Property(
            "Response Limits",
            {{Goal( Minimize ), Importance( 1 ), Show Limits( 0 )}}
        );

    // MULTI-RESPONSE BAYESOPT DESIRABILITY.
    // v48 deliberately bounds every modeled response so a single violent or
    // failed trial cannot dominate a Gaussian process. Raw chatter/overshoot
    // remain available as diagnostics and in the per-run worker CSV.
{response_blocks}

    dt;
);'''
    text = replace_required(text, old_limits, new_limits, "response desirability")

    text = replace_required(
        text,
        '            "Score,Settling Time (s),Overshoot (deg),Oscillations,IAE (deg*s)," ||',
        '            "Score,Settling Time (s),Overshoot (deg),Raw Overshoot (deg),Oscillations,PWM Chatter,Raw PWM Chatter,Home Entry RPM,Return Failure,IAE (deg*s)," ||',
        "compact CSV header",
    )
    text = replace_required(
        text,
        '''                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "IAE (deg*s)" )[rowIndex] ) || "," ||''',
        '''                Char( Column( dt, "Raw Overshoot (deg)" )[rowIndex] ) || "," ||
                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "PWM Chatter" )[rowIndex] ) || "," ||
                Char( Column( dt, "Raw PWM Chatter" )[rowIndex] ) || "," ||
                Char( Column( dt, "Home Entry RPM" )[rowIndex] ) || "," ||
                Char( Column( dt, "Return Failure" )[rowIndex] ) || "," ||
                Char( Column( dt, "IAE (deg*s)" )[rowIndex] ) || "," ||''',
        "compact CSV response values",
    )

    old_result_list = '''            asNumber( Column( resultDT, "Overshoot (deg)" )[1] ),
            asNumber( Column( resultDT, "Oscillations" )[1] ),
            asNumber( Column( resultDT, "IAE (deg*s)" )[1] ),
            asNumber( Column( resultDT, "Peak Output RPM" )[1] ),
            asNumber( Column( resultDT, "Final Error (deg)" )[1] ),
            asNumber( Column( resultDT, "Success" )[1] ),
            statusText,
            asNumber( Column( resultDT, "Samples" )[1] )'''
    new_result_list = '''            asNumber( Column( resultDT, "Overshoot (deg)" )[1] ),
            asNumber( Column( resultDT, "Oscillations" )[1] ),
            asNumber( Column( resultDT, "PWM Chatter" )[1] ),
            asNumber( Column( resultDT, "Home Entry RPM" )[1] ),
            asNumber( Column( resultDT, "IAE (deg*s)" )[1] ),
            asNumber( Column( resultDT, "Peak Output RPM" )[1] ),
            asNumber( Column( resultDT, "Final Error (deg)" )[1] ),
            asNumber( Column( resultDT, "Success" )[1] ),
            statusText,
            asNumber( Column( resultDT, "Samples" )[1] )'''
    text = replace_required(text, old_result_list, new_result_list, "result-list metrics")

    # Old result-list status/sample positions move from 10/11 to 12/13.
    text = text.replace("resultList[10]", "resultList[12]")
    text = text.replace("resultList[11]", "resultList[13]")

    old_save = '''    Column( dt, "Overshoot (deg)" )[rowIndex] = resultList[4];
    Column( dt, "Oscillations" )[rowIndex] = resultList[5];
    Column( dt, "IAE (deg*s)" )[rowIndex] = resultList[6];
    Column( dt, "Peak Output RPM" )[rowIndex] = resultList[7];
    Column( dt, "Final Error (deg)" )[rowIndex] = resultList[8];
    Column( dt, "Success" )[rowIndex] = resultList[9];
    Column( dt, "Status" )[rowIndex] = resultList[12];

    saveSnapshot( dt );'''
    new_save = f'''    // Preserve raw diagnostics, but feed bounded/conditioned responses to
    // Bayesian Optimization. This prevents the 280-deg overshoot and 1000+
    // chatter outliers seen in v47 from overwhelming their GP models.
    Column( dt, "Raw Overshoot (deg)" )[rowIndex] = resultList[4];
    Column( dt, "Overshoot (deg)" )[rowIndex] =
        Min( {overshoot_cap:g}, Max( 0.0, resultList[4] ) );
    Column( dt, "Oscillations" )[rowIndex] =
        Min( {oscillation_cap:g}, Max( 0.0, resultList[5] ) );
    Column( dt, "Raw PWM Chatter" )[rowIndex] = resultList[6];
    Column( dt, "PWM Chatter" )[rowIndex] =
        Min( {chatter_cap:g}, Log( 1 + Max( 0.0, resultList[6] ) ) );
    Column( dt, "Home Entry RPM" )[rowIndex] =
        Min( {entry_rpm_cap:g}, Max( 0.0, resultList[7] ) );
    Column( dt, "IAE (deg*s)" )[rowIndex] = resultList[8];
    Column( dt, "Peak Output RPM" )[rowIndex] = resultList[9];
    Column( dt, "Final Error (deg)" )[rowIndex] = resultList[10];
    Column( dt, "Success" )[rowIndex] = resultList[11];
    Column( dt, "Status" )[rowIndex] = resultList[12];
    Column( dt, "Return Failure" )[rowIndex] = If( resultList[12] == "COMPLETE", 0, 1 );

    saveSnapshot( dt );
    // Keep the live optimization table visible between every physical run.
    dt << Bring Window To Front;'''
    text = replace_required(text, old_save, new_save, "save conditioned multi-response metrics")

    multi_y = '''                Y(
                    :Name( "Return Failure" ),
                    :Name( "Settling Time (s)" ),
                    :Name( "PWM Chatter" ),
                    :Name( "Home Entry RPM" ),
                    :Name( "Overshoot (deg)" ),
                    :Oscillations
                ),'''
    text = replace_required(text, '                Y( :Score ),', multi_y, "multi-response BayesOpt Y")

    # Let JMP exploit moderately predictive models instead of forcing most
    # post-seed runs into MaxPro space filling. v47 used 0.50 and almost every
    # response except Return Failure fell below that threshold.
    text = replace_required(
        text,
        '                    Minimum RSquare( 0.50 )',
        f'                    Minimum RSquare( {minimum_rsquare:.2f} )',
        "BayesOpt minimum R-square",
    )

    # Quantize every physically tested gain to the configured PID step.
    seed_kp_quant = f'        seedKp = Round( seedSettings[seedIndex][1] / {pid_step_jsl} ) * {pid_step_jsl};'
    seed_ki_quant = f'        seedKi = Round( seedSettings[seedIndex][2] / {pid_step_jsl} ) * {pid_step_jsl};'
    seed_kd_quant = f'        seedKd = Round( seedSettings[seedIndex][3] / {pid_step_jsl} ) * {pid_step_jsl};'
    proposed_kp_quant = f'        proposedKp = Round( Column( dt, "KP" )[newRow] / {pid_step_jsl} ) * {pid_step_jsl};'
    proposed_ki_quant = f'        proposedKi = Round( Column( dt, "KI" )[newRow] / {pid_step_jsl} ) * {pid_step_jsl};'
    proposed_kd_quant = f'''        proposedKd = Round( Column( dt, "KD" )[newRow] / {pid_step_jsl} ) * {pid_step_jsl};
        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;'''

    text = replace_required(
        text,
        '        seedKp = seedSettings[seedIndex][1];',
        seed_kp_quant,
        "seed Kp quantization",
    )
    text = replace_required(
        text,
        '        seedKi = seedSettings[seedIndex][2];',
        seed_ki_quant,
        "seed Ki quantization",
    )
    text = replace_required(
        text,
        '        seedKd = seedSettings[seedIndex][3];',
        seed_kd_quant,
        "seed Kd quantization",
    )
    text = replace_required(
        text,
        '        proposedKp = Column( dt, "KP" )[newRow];',
        proposed_kp_quant,
        "Bayesian Kp quantization",
    )
    text = replace_required(
        text,
        '        proposedKi = Column( dt, "KI" )[newRow];',
        proposed_ki_quant,
        "Bayesian Ki quantization",
    )
    text = replace_required(
        text,
        '        proposedKd = Column( dt, "KD" )[newRow];',
        proposed_kd_quant,
        "Bayesian Kd quantization",
    )

    # Failed physical trials remain finite training rows and never abort the study.
    text = regex_required(
        text,
        r'\s*If\(\s*Contains\( statusText, "CONTROL_ERROR_" \) == 1,.*?Throw\( statusText \);\s*\);\s*(?=\s*If\(\s*Is Missing\( scoreValue \))',
        "\n",
        "CONTROL_ERROR continuation",
    )

    # Startup window explicitly states the active multi-response design.
    text = replace_required(
        text,
        '                "Study: " ||\n                studyName ||\n                "\\!NSession: " || sessionName ||\n                "\\!N\\!N" ||',
        '                "Study: " ||\n                studyName ||\n                "\\!NSession: " || sessionName ||\n                "\\!NBayesOpt mode: MULTI-RESPONSE DESIRABILITY\\!N" ||\n                "Responses: failure, settling, bounded chatter, home-entry RPM, bounded overshoot, oscillations\\!N" ||\n                "v48 response shaping: bounded/log-compressed for GP stability\\!N\\!N" ||',
        "startup multi-response banner",
    )
    text = replace_required(
        text,
        '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\\!N" ||',
        f'                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s HARD CUTOFF\\!N" ||\n                "Timeout settling response: {max_settle:.1f} s (censored), Return Failure=1\\!N" ||\n                "BayesOpt model gate: minimum R-square {minimum_rsquare:.2f}\\!N" ||\n                "PID coefficient rounding: nearest {pid_step_jsl}\\!N" ||',
        "startup timeout/model description",
    )

    # Keep the data table present from the moment the session starts.
    text = replace_required(
        text,
        'Current Data Table( dt );\nsaveSnapshot( dt );',
        'Current Data Table( dt );\nsaveSnapshot( dt );\ndt << Bring Window To Front;',
        "initial live table",
    )

    # Normalize the quote helper for Windows JMP.
    text = text.replace('    "\\!\\\"" || Char( value ) || "\\!\\\""', '    "\\!\"" || Char( value ) || "\\!\""')

    # Final runtime validation before anything physical is launched.
    required = [
        study_name,
        f"kpLow = {kp_low:.1f};",
        f"kpHigh = {kp_high:.1f};",
        f"kiLow = {ki_low:.1f};",
        f"kiHigh = {ki_high:.1f};",
        f"kdLow = {kd_low:.1f};",
        f"kdHigh = {kd_high:.1f};",
        f"Minimum RSquare( {minimum_rsquare:.2f} )",
        "MULTI-RESPONSE BAYESOPT DESIRABILITY",
        "v48 response shaping: bounded/log-compressed for GP stability",
        ':Name( "Return Failure" )',
        ':Name( "Settling Time (s)" )',
        ':Name( "PWM Chatter" )',
        ':Name( "Home Entry RPM" )',
        ':Name( "Overshoot (deg)" )',
        ':Oscillations',
        'Column( dt, "Raw PWM Chatter" )[rowIndex]',
        'Column( dt, "Raw Overshoot (deg)" )[rowIndex]',
        f'Min( {chatter_cap:g}, Log( 1 + Max( 0.0, resultList[6] ) ) )',
        f'Min( {overshoot_cap:g}, Max( 0.0, resultList[4] ) )',
        'Column( dt, "Return Failure" )[rowIndex]',
        'dt << Bring Window To Front;',
        '"--max-settling-seconds", Char( maxSettlingTimeSeconds )',
        proposed_kp_quant.strip(),
        proposed_ki_quant.strip(),
        f'proposedKd = Round( Column( dt, "KD" )[newRow] / {pid_step_jsl} ) * {pid_step_jsl};',
        jsl_path(worker),
    ]
    missing = [token for token in required if token not in text]
    if missing:
        raise RuntimeError(f"Runtime JSL validation failed; missing: {missing}")
    if 'Y( :Score )' in text:
        raise RuntimeError("Runtime JSL validation failed: scalar Score is still BayesOpt Y")

    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(text, encoding="utf-8", newline="\n")
    print(f"Built validated runtime JSL: {a.output}")
    print("BayesOpt: MULTI-RESPONSE desirability with bounded/log-compressed responses")
    print("Responses: Return Failure, Settling Time, PWM Chatter, Home Entry RPM, Overshoot, Oscillations")
    print(f"PID range: Kp {kp_low:g}-{kp_high:g}, Ki {ki_low:g}-{ki_high:g}, Kd {kd_low:g}-{kd_high:g}")
    print(f"PID coefficient rounding: nearest {pid_step:g}")
    print(f"Physical worker: {worker}")
    print(f"BayesOpt minimum R-square: {minimum_rsquare:.2f}")
    print(f"Settling timeout: {max_settle:.1f} s; timeout rows export Settling Time={max_settle:.1f} and Return Failure=1")
    print(f"Response caps: chatter={chatter_cap:g}, overshoot={overshoot_cap:g} deg, entryRPM={entry_rpm_cap:g}, oscillations={oscillation_cap:g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
