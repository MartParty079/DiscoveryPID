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


def main() -> int:
    a = args()
    repo = a.repo.resolve()
    config = json.loads((repo / "config" / "discoverypid.json").read_text(encoding="utf-8"))
    source = repo / "jmp" / "DiscoveryPID_Position_V2.jsl"
    worker = repo / "tools" / "run_position_trial.py"
    text = source.read_text(encoding="utf-8")

    kp_low, kp_high = config["pidRanges"]["kp"]
    ki_low, ki_high = config["pidRanges"]["ki"]
    kd_low, kd_high = config["pidRanges"]["kd"]
    max_settle = float(config["maxSettlingTimeSeconds"])
    study_name = config["studyName"]
    data_root_name = config["dataRoot"]

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
        seed_lines.append(f"    {{{kp:.1f}, {ki:.1f}, {kd:.1f}}}{comma}")
    seed_lines.append("};")
    text = regex_required(
        text,
        r'seedSettings\s*=\s*\{.*?\n\};',
        "\n".join(seed_lines),
        "seed design",
    )

    # Add physical smoothness/failure responses to the live table.
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
    new_limits = '''    // Score remains a diagnostic/setup-ranking value only.
    Column( dt, "Score" ) <<
        Set Property(
            "Response Limits",
            {Goal( Minimize ), Importance( 1 ), Show Limits( 0 )}
        );

    // MULTI-RESPONSE BAYESOPT DESIRABILITY.
    // Lower is better for every response. A 3-second non-return is exported as
    // Settling Time=3.0 and Return Failure=1, so no BayesOpt Y is left blank.
    Column( dt, "Return Failure" ) << Set Property(
        "Response Limits",
        {Lower( 0.0, 1.0 ), Middle( 0.5, 0.15 ), Upper( 1.0, 0.0 ),
         Goal( Minimize ), Importance( 6 ), Show Limits( 1 )}
    );
    Column( dt, "Settling Time (s)" ) << Set Property(
        "Response Limits",
        {Lower( 0.60, 1.0 ), Middle( 1.00, 0.85 ), Upper( 3.00, 0.0 ),
         Goal( Minimize ), Importance( 5 ), Show Limits( 1 )}
    );
    Column( dt, "PWM Chatter" ) << Set Property(
        "Response Limits",
        {Lower( 0.0, 1.0 ), Middle( 0.5, 0.85 ), Upper( 2.5, 0.0 ),
         Goal( Minimize ), Importance( 5 ), Show Limits( 1 )}
    );
    Column( dt, "Home Entry RPM" ) << Set Property(
        "Response Limits",
        {Lower( 0.0, 1.0 ), Middle( 2.0, 0.85 ), Upper( 10.0, 0.0 ),
         Goal( Minimize ), Importance( 5 ), Show Limits( 1 )}
    );
    Column( dt, "Overshoot (deg)" ) << Set Property(
        "Response Limits",
        {Lower( 0.0, 1.0 ), Middle( 1.0, 0.75 ), Upper( 5.0, 0.0 ),
         Goal( Minimize ), Importance( 3 ), Show Limits( 1 )}
    );
    Column( dt, "Oscillations" ) << Set Property(
        "Response Limits",
        {Lower( 0.0, 1.0 ), Middle( 1.0, 0.10 ), Upper( 2.0, 0.0 ),
         Goal( Minimize ), Importance( 5 ), Show Limits( 1 )}
    );

    dt;
);'''
    text = replace_required(text, old_limits, new_limits, "response desirability")

    text = replace_required(
        text,
        '            "Score,Settling Time (s),Overshoot (deg),Oscillations,IAE (deg*s)," ||',
        '            "Score,Settling Time (s),Overshoot (deg),Oscillations,PWM Chatter,Home Entry RPM,Return Failure,IAE (deg*s)," ||',
        "compact CSV header",
    )
    text = replace_required(
        text,
        '''                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "IAE (deg*s)" )[rowIndex] ) || "," ||''',
        '''                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "PWM Chatter" )[rowIndex] ) || "," ||
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
    new_save = '''    Column( dt, "Overshoot (deg)" )[rowIndex] = resultList[4];
    Column( dt, "Oscillations" )[rowIndex] = resultList[5];
    Column( dt, "PWM Chatter" )[rowIndex] = resultList[6];
    Column( dt, "Home Entry RPM" )[rowIndex] = resultList[7];
    Column( dt, "IAE (deg*s)" )[rowIndex] = resultList[8];
    Column( dt, "Peak Output RPM" )[rowIndex] = resultList[9];
    Column( dt, "Final Error (deg)" )[rowIndex] = resultList[10];
    Column( dt, "Success" )[rowIndex] = resultList[11];
    Column( dt, "Status" )[rowIndex] = resultList[12];
    Column( dt, "Return Failure" )[rowIndex] = If( resultList[12] == "COMPLETE", 0, 1 );

    saveSnapshot( dt );
    // Keep the live optimization table visible between every physical run.
    dt << Bring Window To Front;'''
    text = replace_required(text, old_save, new_save, "save multi-response metrics")

    multi_y = '''                Y(
                    :Name( "Return Failure" ),
                    :Name( "Settling Time (s)" ),
                    :Name( "PWM Chatter" ),
                    :Name( "Home Entry RPM" ),
                    :Name( "Overshoot (deg)" ),
                    :Oscillations
                ),'''
    text = replace_required(text, '                Y( :Score ),', multi_y, "multi-response BayesOpt Y")

    # Quantize every physically tested gain to the 0.5 grid.
    text = replace_required(
        text, '        seedKp = seedSettings[seedIndex][1];',
        '        seedKp = Round( seedSettings[seedIndex][1] * 2 ) / 2;', "seed Kp quantization")
    text = replace_required(
        text, '        seedKi = seedSettings[seedIndex][2];',
        '        seedKi = Round( seedSettings[seedIndex][2] * 2 ) / 2;', "seed Ki quantization")
    text = replace_required(
        text, '        seedKd = seedSettings[seedIndex][3];',
        '        seedKd = Round( seedSettings[seedIndex][3] * 2 ) / 2;', "seed Kd quantization")
    text = replace_required(
        text, '        proposedKp = Column( dt, "KP" )[newRow];',
        '        proposedKp = Round( Column( dt, "KP" )[newRow] * 2 ) / 2;', "Bayesian Kp quantization")
    text = replace_required(
        text, '        proposedKi = Column( dt, "KI" )[newRow];',
        '        proposedKi = Round( Column( dt, "KI" )[newRow] * 2 ) / 2;', "Bayesian Ki quantization")
    text = replace_required(
        text,
        '        proposedKd = Column( dt, "KD" )[newRow];',
        '''        proposedKd = Round( Column( dt, "KD" )[newRow] * 2 ) / 2;
        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;''',
        "Bayesian Kd quantization",
    )

    # Failed physical trials remain finite training rows and never abort the study.
    text = regex_required(
        text,
        r'\s*If\(\s*Contains\( statusText, "CONTROL_ERROR_" \) == 1,.*?Throw\( statusText \);\s*\);\s*(?=\s*If\(\s*Is Missing\( scoreValue \))',
        "\n",
        "CONTROL_ERROR continuation",
    )

    # Startup window explicitly states that this is multi-response optimization.
    text = replace_required(
        text,
        '                "Study: " ||\n                studyName ||\n                "\\!NSession: " || sessionName ||\n                "\\!N\\!N" ||',
        '                "Study: " ||\n                studyName ||\n                "\\!NSession: " || sessionName ||\n                "\\!NBayesOpt mode: MULTI-RESPONSE DESIRABILITY\\!N" ||\n                "Responses: failure, settling, chatter, home-entry RPM, overshoot, oscillations\\!N\\!N" ||',
        "startup multi-response banner",
    )
    text = replace_required(
        text,
        '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\\!N" ||',
        f'                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s HARD CUTOFF\\!N" ||\n                "Timeout settling response: {max_settle:.1f} s (censored), Return Failure=1\\!N" ||',
        "startup timeout description",
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
        f"kpHigh = {kp_high:.1f};",
        "MULTI-RESPONSE BAYESOPT DESIRABILITY",
        ':Name( "Return Failure" )',
        ':Name( "Settling Time (s)" )',
        ':Name( "PWM Chatter" )',
        ':Name( "Home Entry RPM" )',
        ':Name( "Overshoot (deg)" )',
        ':Oscillations',
        'Column( dt, "Return Failure" )[rowIndex]',
        'dt << Bring Window To Front;',
        '"--max-settling-seconds", Char( maxSettlingTimeSeconds )',
    ]
    missing = [token for token in required if token not in text]
    if missing:
        raise RuntimeError(f"Runtime JSL validation failed; missing: {missing}")
    if 'Y( :Score )' in text:
        raise RuntimeError("Runtime JSL validation failed: scalar Score is still BayesOpt Y")

    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(text, encoding="utf-8", newline="\n")
    print(f"Built validated runtime JSL: {a.output}")
    print("BayesOpt: MULTI-RESPONSE desirability")
    print("Responses: Return Failure, Settling Time, PWM Chatter, Home Entry RPM, Overshoot, Oscillations")
    print(f"PID range: Kp {kp_low:g}-{kp_high:g}, Ki {ki_low:g}-{ki_high:g}, Kd {kd_low:g}-{kd_high:g}")
    print(f"Settling timeout: {max_settle:.1f} s; timeout rows export Settling Time={max_settle:.1f} and Return Failure=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
