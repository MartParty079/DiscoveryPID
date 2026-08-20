param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceScript = Join-Path $repo "jmp\DiscoveryPID_Position_V2.jsl"
$workerScript = Join-Path $repo "tools\run_position_trial.py"
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"

if (-not (Test-Path $sourceScript)) {
    throw "JMP script not found: $sourceScript"
}
if (-not (Test-Path $workerScript)) {
    throw "Physical trial worker not found: $workerScript"
}
if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

$studyName = "PositionStudy_DirectPID_v44_MultiResponseFastSmooth_300Deg"
$sessionDir = Join-Path $repo "data\position_autotune_directpid_v44_multiresponse_fastsmooth\sessions"
New-Item -ItemType Directory -Force $sessionDir | Out-Null

# Create a machine-specific runtime copy. Physical-control logic remains in the
# tracked Python worker; this copy injects paths/port, current ranges, the seed
# design, response columns, desirability goals, and exact 0.5-step PID values.
$runtimeScript = Join-Path $env:TEMP "DiscoveryPID_Position_V2_runtime.jsl"
$text = Get-Content $sourceScript -Raw
$repoJsl = ($repo -replace '\\', '/') + '/'
$pyJsl = ($py -replace '\\', '/')
$workerJsl = ($workerScript -replace '\\', '/')

$text = [regex]::Replace(
    $text,
    'projectDirectory\s*=\s*"[^"]*";',
    "projectDirectory =`r`n    `"$repoJsl`";",
    1
)
$text = [regex]::Replace(
    $text,
    'pythonExecutable\s*=\s*"[^"]*";',
    "pythonExecutable =`r`n    `"$pyJsl`";",
    1
)
$text = [regex]::Replace(
    $text,
    'portName\s*=\s*"[^"]*";',
    "portName = `"$Port`";",
    1
)
$text = [regex]::Replace(
    $text,
    'workerScript\s*=\s*projectDirectory\s*\|\|\s*"tools/run_position_trial\.py";',
    "workerScript =`r`n    `"$workerJsl`";",
    1
)

$text = $text.Replace(
    'studyName = "PositionStudy_DirectPID_v39_300Deg10xRange_3s";',
    "studyName = `"$studyName`";"
)
$text = $text.Replace(
    'data/position_autotune_directpid_v39_300deg10xrange_3s',
    'data/position_autotune_directpid_v44_multiresponse_fastsmooth'
)

# Tight search region. These are the only ranges presented to JMP.
$text = $text.Replace('kpHigh = 1000.0;', 'kpHigh = 200.0;')
$text = $text.Replace('kiHigh = 500.0;', 'kiHigh = 100.0;')
$text = $text.Replace('kdHigh = 200.0;', 'kdHigh = 50.0;')

# Progressive seed design. Early seeds are conservative but should still move
# the 300-degree return. Later seeds progressively increase authority and are
# intentionally allowed to approach underdamped/aggressive behavior.
$seedBlock = @'
seedSettings = {
    {4.0,    0.0,  1.0},
    {6.0,    0.0,  1.0},
    {8.0,    0.0,  1.5},
    {10.0,   0.0,  2.0},
    {15.0,   0.5,  2.5},
    {25.0,   1.0,  4.0},
    {40.0,   2.0,  6.0},
    {70.0,   4.0, 10.0},
    {120.0,  8.0, 18.0},
    {200.0, 15.0, 30.0}
};
'@
$text = [regex]::Replace(
    $text,
    '(?s)seedSettings\s*=\s*\{.*?\n\};',
    $seedBlock.TrimEnd(),
    1
)

# Add smoothness measurements and an explicit failure response to the JMP
# working table. Return Failure is 0 only for a COMPLETE return and 1 for any
# timeout/control error, so failed 3-second trials are visible to BayesOpt.
$metricColumnsOld = @'
        New Column(
            "Oscillations",
            Numeric,
            Ordinal,
            Format( "Best", 12 )
        ),

        New Column(
            "IAE (deg*s)",
'@
$metricColumnsNew = @'
        New Column(
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
            "IAE (deg*s)",
'@
$text = $text.Replace($metricColumnsOld, $metricColumnsNew)

# Replace the old scalar-only response goal with explicit multi-response
# desirability functions. Lower response values are better. The scalar Score is
# retained for diagnostics and best-completed setup selection, but BayesOpt no
# longer uses Score as its Y response.
$responseLimitsOld = @'
    // Safe here because Score was just created explicitly as Numeric.
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
);
'@
$responseLimitsNew = @'
    // Score remains a diagnostic ranking only.
    Column( dt, "Score" ) <<
        Set Property(
            "Response Limits",
            {
                Goal( Minimize ),
                Importance( 1 ),
                Show Limits( 0 )
            }
        );

    // Multi-response Bayesian desirability. A response at its Lower value has
    // desirability 1; the Upper value has desirability 0.
    Column( dt, "Return Failure" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.0, 1.0 ),
                Middle( 0.5, 0.15 ),
                Upper( 1.0, 0.0 ),
                Goal( Minimize ),
                Importance( 6 ),
                Show Limits( 1 )
            }
        );

    Column( dt, "Settling Time (s)" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.60, 1.0 ),
                Middle( 1.00, 0.85 ),
                Upper( 3.00, 0.0 ),
                Goal( Minimize ),
                Importance( 5 ),
                Show Limits( 1 )
            }
        );

    Column( dt, "PWM Chatter" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.0, 1.0 ),
                Middle( 0.5, 0.85 ),
                Upper( 2.5, 0.0 ),
                Goal( Minimize ),
                Importance( 5 ),
                Show Limits( 1 )
            }
        );

    Column( dt, "Home Entry RPM" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.0, 1.0 ),
                Middle( 2.0, 0.85 ),
                Upper( 10.0, 0.0 ),
                Goal( Minimize ),
                Importance( 5 ),
                Show Limits( 1 )
            }
        );

    Column( dt, "Overshoot (deg)" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.0, 1.0 ),
                Middle( 1.0, 0.75 ),
                Upper( 5.0, 0.0 ),
                Goal( Minimize ),
                Importance( 3 ),
                Show Limits( 1 )
            }
        );

    Column( dt, "Oscillations" ) <<
        Set Property(
            "Response Limits",
            {
                Lower( 0.0, 1.0 ),
                Middle( 1.0, 0.10 ),
                Upper( 2.0, 0.0 ),
                Goal( Minimize ),
                Importance( 5 ),
                Show Limits( 1 )
            }
        );

    dt;
);
'@
$text = $text.Replace($responseLimitsOld, $responseLimitsNew)

# Keep every physical response and the derived failure flag in the compact CSV.
$text = $text.Replace(
    '            "Score,Settling Time (s),Overshoot (deg),Oscillations,IAE (deg*s)," ||',
    '            "Score,Settling Time (s),Overshoot (deg),Oscillations,PWM Chatter,Home Entry RPM,Return Failure,IAE (deg*s)," ||'
)

$compactRowsOld = @'
                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "IAE (deg*s)" )[rowIndex] ) || "," ||
'@
$compactRowsNew = @'
                Char( Column( dt, "Oscillations" )[rowIndex] ) || "," ||
                Char( Column( dt, "PWM Chatter" )[rowIndex] ) || "," ||
                Char( Column( dt, "Home Entry RPM" )[rowIndex] ) || "," ||
                Char( Column( dt, "Return Failure" )[rowIndex] ) || "," ||
                Char( Column( dt, "IAE (deg*s)" )[rowIndex] ) || "," ||
'@
$text = $text.Replace($compactRowsOld, $compactRowsNew)

# Pull the new physical metrics from Python's one-row result file.
$resultListOld = @'
            asNumber( Column( resultDT, "Overshoot (deg)" )[1] ),
            asNumber( Column( resultDT, "Oscillations" )[1] ),
            asNumber( Column( resultDT, "IAE (deg*s)" )[1] ),
            asNumber( Column( resultDT, "Peak Output RPM" )[1] ),
            asNumber( Column( resultDT, "Final Error (deg)" )[1] ),
            asNumber( Column( resultDT, "Success" )[1] ),
            statusText,
            asNumber( Column( resultDT, "Samples" )[1] )
'@
$resultListNew = @'
            asNumber( Column( resultDT, "Overshoot (deg)" )[1] ),
            asNumber( Column( resultDT, "Oscillations" )[1] ),
            asNumber( Column( resultDT, "PWM Chatter" )[1] ),
            asNumber( Column( resultDT, "Home Entry RPM" )[1] ),
            asNumber( Column( resultDT, "IAE (deg*s)" )[1] ),
            asNumber( Column( resultDT, "Peak Output RPM" )[1] ),
            asNumber( Column( resultDT, "Final Error (deg)" )[1] ),
            asNumber( Column( resultDT, "Success" )[1] ),
            statusText,
            asNumber( Column( resultDT, "Samples" )[1] )
'@
$text = $text.Replace($resultListOld, $resultListNew)

# Result-list positions after adding Chatter and Home Entry RPM:
# 1 Score, 2 Settle, 3 Timer, 4 Overshoot, 5 Osc, 6 Chatter, 7 EntryRPM,
# 8 IAE, 9 PeakRPM, 10 FinalError, 11 Success, 12 Status, 13 Samples.
$text = $text.Replace('resultList[10]', 'resultList[12]')
$text = $text.Replace('resultList[11]', 'resultList[13]')

$saveMetricsOld = @'
    Column( dt, "Overshoot (deg)" )[rowIndex] = resultList[4];
    Column( dt, "Oscillations" )[rowIndex] = resultList[5];
    Column( dt, "IAE (deg*s)" )[rowIndex] = resultList[6];
    Column( dt, "Peak Output RPM" )[rowIndex] = resultList[7];
    Column( dt, "Final Error (deg)" )[rowIndex] = resultList[8];
    Column( dt, "Success" )[rowIndex] = resultList[9];
    Column( dt, "Status" )[rowIndex] = resultList[12];
'@
$saveMetricsNew = @'
    Column( dt, "Overshoot (deg)" )[rowIndex] = resultList[4];
    Column( dt, "Oscillations" )[rowIndex] = resultList[5];
    Column( dt, "PWM Chatter" )[rowIndex] = resultList[6];
    Column( dt, "Home Entry RPM" )[rowIndex] = resultList[7];
    Column( dt, "IAE (deg*s)" )[rowIndex] = resultList[8];
    Column( dt, "Peak Output RPM" )[rowIndex] = resultList[9];
    Column( dt, "Final Error (deg)" )[rowIndex] = resultList[10];
    Column( dt, "Success" )[rowIndex] = resultList[11];
    Column( dt, "Status" )[rowIndex] = resultList[12];
    Column( dt, "Return Failure" )[rowIndex] =
        If( resultList[12] == "COMPLETE", 0, 1 );
'@
$text = $text.Replace($saveMetricsOld, $saveMetricsNew)

# Multi-response Bayesian Optimization. JMP models each physical response and
# combines the Response Limits through overall desirability. Score stays in the
# table but is intentionally absent from Y().
$multiResponseY = @'
                Y(
                    :Name( "Return Failure" ),
                    :Name( "Settling Time (s)" ),
                    :Name( "PWM Chatter" ),
                    :Name( "Home Entry RPM" ),
                    :Name( "Overshoot (deg)" ),
                    :Oscillations
                ),
'@
$text = $text.Replace('                Y( :Score ),', $multiResponseY.TrimEnd())

# Quantize both initial seeds and Bayesian proposals to the nearest 0.5.
$text = $text.Replace(
    '        seedKp = seedSettings[seedIndex][1];',
    '        seedKp = Round( seedSettings[seedIndex][1] * 2 ) / 2;'
)
$text = $text.Replace(
    '        seedKi = seedSettings[seedIndex][2];',
    '        seedKi = Round( seedSettings[seedIndex][2] * 2 ) / 2;'
)
$text = $text.Replace(
    '        seedKd = seedSettings[seedIndex][3];',
    '        seedKd = Round( seedSettings[seedIndex][3] * 2 ) / 2;'
)
$text = $text.Replace(
    '        proposedKp = Column( dt, "KP" )[newRow];',
    '        proposedKp = Round( Column( dt, "KP" )[newRow] * 2 ) / 2;'
)
$text = $text.Replace(
    '        proposedKi = Column( dt, "KI" )[newRow];',
    '        proposedKi = Round( Column( dt, "KI" )[newRow] * 2 ) / 2;'
)
$text = $text.Replace(
    '        proposedKd = Column( dt, "KD" )[newRow];',
@'
        proposedKd = Round( Column( dt, "KD" )[newRow] * 2 ) / 2;

        // Store the exact gains that are physically tested.
        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;
'@
)

$text = $text.Replace(
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\!N" ||',
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s HARD CUTOFF\!N" || "Target response: under 1.0 s with a smooth home entry\!N" || "BayesOpt: multi-response desirability (failure + speed + smoothness + overshoot + oscillation)\!N" || "PID ranges: Kp 0.5-200, Ki 0-100, Kd 0-50\!N" || "PID coefficient increment: 0.5\!N" ||'
)

# A failed physical run is an invalid penalty row, not a reason to kill the
# entire optimization. The Python worker prints the concrete control error.
$text = [regex]::Replace(
    $text,
    '(?s)\s*If\(\s*Contains\( statusText, "CONTROL_ERROR_" \) == 1,.*?Throw\( statusText \);\s*\);\s*(?=\s*If\(\s*Is Missing\( scoreValue \))',
    "`r`n",
    1
)

# Normalize quote helper for JMP on Windows.
$text = $text.Replace('    "\!\"" || Char( value ) || "\!\""', '    "\!"" || Char( value ) || "\!""')

# Refuse to launch a partially patched runtime study. This catches stale study
# identity, ranges, seed design, data columns, response goals, or loss of the
# common 3-second cutoff before any physical motion is requested.
if ($text -notmatch [regex]::Escape($workerJsl)) {
    throw "Runtime JSL validation failed: physical worker path was not injected."
}
if ($text -notmatch 'proposedKp\s*=\s*Round\(') {
    throw "Runtime JSL validation failed: 0.5-step Bayesian quantization missing."
}
if ($text -match 'Throw\( statusText \)') {
    throw "Runtime JSL validation failed: physical CONTROL_ERROR hard-stop still present."
}
if ($text -notmatch [regex]::Escape($studyName)) {
    throw "Runtime JSL validation failed: current study identity missing."
}
if ($text -notmatch 'kpHigh\s*=\s*200\.0;' -or
    $text -notmatch 'kiHigh\s*=\s*100\.0;' -or
    $text -notmatch 'kdHigh\s*=\s*50\.0;') {
    throw "Runtime JSL validation failed: PID ranges are not 200/100/50."
}
if ($text -notmatch 'maxSettlingTimeSeconds\s*=\s*3\.0;' -or
    $text -notmatch '"--max-settling-seconds", Char\( maxSettlingTimeSeconds \)') {
    throw "Runtime JSL validation failed: common 3-second return cutoff missing."
}
if ($text -notmatch '\{4\.0,\s+0\.0,\s+1\.0\}' -or
    $text -notmatch '\{200\.0,\s+15\.0,\s+30\.0\}') {
    throw "Runtime JSL validation failed: clean progressive seed design missing."
}
if ($text -notmatch 'New Column\(\s*"PWM Chatter"' -or
    $text -notmatch 'New Column\(\s*"Home Entry RPM"' -or
    $text -notmatch 'New Column\(\s*"Return Failure"') {
    throw "Runtime JSL validation failed: multi-response data columns missing."
}
if ($text -notmatch 'Column\( dt, "PWM Chatter" \)\[rowIndex\] = resultList\[6\]' -or
    $text -notmatch 'Column\( dt, "Home Entry RPM" \)\[rowIndex\] = resultList\[7\]' -or
    $text -notmatch 'Column\( dt, "Return Failure" \)\[rowIndex\]') {
    throw "Runtime JSL validation failed: multi-response result mapping missing."
}
if ($text -notmatch 'Char\( Column\( dt, "PWM Chatter" \)\[rowIndex\] \)' -or
    $text -notmatch 'Char\( Column\( dt, "Home Entry RPM" \)\[rowIndex\] \)' -or
    $text -notmatch 'Char\( Column\( dt, "Return Failure" \)\[rowIndex\] \)') {
    throw "Runtime JSL validation failed: compact CSV multi-response columns missing."
}
if ($text -notmatch ':Name\( "Return Failure" \)' -or
    $text -notmatch ':Name\( "Settling Time \(s\)" \)' -or
    $text -notmatch ':Name\( "PWM Chatter" \)' -or
    $text -notmatch ':Name\( "Home Entry RPM" \)' -or
    $text -notmatch ':Name\( "Overshoot \(deg\)" \)' -or
    $text -notmatch ':Oscillations') {
    throw "Runtime JSL validation failed: Bayesian Optimization is not using all multi-response Y columns."
}
if ($text -match 'Y\(\s*:Score\s*\)') {
    throw "Runtime JSL validation failed: stale scalar Score Bayesian response remains."
}
if ($text -notmatch 'Importance\( 6 \)' -or
    $text -notmatch 'Lower\( 0\.60, 1\.0 \)' -or
    $text -notmatch 'Upper\( 3\.00, 0\.0 \)') {
    throw "Runtime JSL validation failed: multi-response desirability limits missing."
}

[System.IO.File]::WriteAllText(
    $runtimeScript,
    $text,
    (New-Object System.Text.UTF8Encoding($false))
)

Write-Host "Launching DiscoveryPID JMP physical study" -ForegroundColor Cyan
Write-Host "  Code branch : main"
Write-Host "  Port        : $Port"
Write-Host "  Worker      : $workerScript"
Write-Host "  Study       : $studyName"
Write-Host "  PID range   : Kp 0.5-200 | Ki 0-100 | Kd 0-50"
Write-Host "  PID step    : 0.5"
Write-Host "  Displacement: 300 deg"
Write-Host "  BayesOpt    : MULTI-RESPONSE desirability"
Write-Host "  Responses   : failure, settling, chatter, entry RPM, overshoot, oscillations"
Write-Host "  Target      : <1.0 s, smooth stop"
Write-Host "  Settle max  : 3.0 s HARD CUTOFF (seeds + Bayesian)"
Write-Host "  Score       : diagnostic/setup ranking only; NOT a BayesOpt Y"
Write-Host "  ACK retries : 3"
Write-Host "  Runtime JSL : $runtimeScript"
Invoke-Item $runtimeScript
