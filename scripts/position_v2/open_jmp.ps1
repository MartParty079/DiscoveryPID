param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceScript = Join-Path $repo "jmp\DiscoveryPID_Position_V2.jsl"
$workerScript = Join-Path $repo "tools\run_position_trial_halfstep.py"
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"

if (-not (Test-Path $sourceScript)) {
    throw "JMP script not found: $sourceScript"
}

if (-not (Test-Path $workerScript)) {
    throw "Half-step trial worker not found: $workerScript"
}

if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

# JMP's flat-session save path must exist before the JSL runs.
$sessionDir = Join-Path $repo "data\position_autotune_directpid_v40_300deg10xrange_3s_halfstep\sessions"
New-Item -ItemType Directory -Force $sessionDir | Out-Null

# Build a runtime copy so collaborators do not need to edit machine-specific
# paths or serial ports in the tracked JSL. The tracked JSL remains a readable
# template; this launcher adds the current 0.5-step experimental policy.
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

# Give half-step sessions their own study/data identity.
$text = $text.Replace(
    'studyName = "PositionStudy_DirectPID_v39_300Deg10xRange_3s";',
    'studyName = "PositionStudy_DirectPID_v40_300Deg10xRange_3s_HalfStep";'
)
$text = $text.Replace(
    'data/position_autotune_directpid_v39_300deg10xrange_3s',
    'data/position_autotune_directpid_v40_300deg10xrange_3s_halfstep'
)

# Quantize every seed and Bayesian proposal to the nearest 0.5 before the
# physical worker is called. The Bayesian row is overwritten with those exact
# tested values so JMP learns from the controller settings that actually ran.
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

        // Record the exact half-step gains used by the physical controller.
        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;
'@
)

# Show the quantization policy on the arm/start dialog.
$text = $text.Replace(
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\!N" ||',
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\!N" || "PID coefficient increment: 0.5\!N" ||'
)

# Normalize the quote helper in case a JSL copy was produced through a JSON
# transport that escaped its embedded quote one extra time.
$text = $text.Replace('    "\!\"" || Char( value ) || "\!\""', '    "\!"" || Char( value ) || "\!""')

[System.IO.File]::WriteAllText(
    $runtimeScript,
    $text,
    (New-Object System.Text.UTF8Encoding($false))
)

Write-Host "Launching JMP runtime study" -ForegroundColor Cyan
Write-Host "  Port: $Port"
Write-Host "  PID step: 0.5"
Write-Host "  Runtime JSL: $runtimeScript"
Invoke-Item $runtimeScript
