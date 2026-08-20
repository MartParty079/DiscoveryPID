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

$studyName = "PositionStudy_DirectPID_v42_FastSmooth_300Deg"
$sessionDir = Join-Path $repo "data\position_autotune_directpid_v42_fastsmooth\sessions"
New-Item -ItemType Directory -Force $sessionDir | Out-Null

# Create a machine-specific runtime copy. Physical-control logic remains in the
# tracked Python worker; this copy only injects paths/port, the current seed
# design, and snaps JMP's table values to the 0.5 PID grid so the recorded
# factors equal what the motor actually runs.
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
    'data/position_autotune_directpid_v42_fastsmooth'
)

# Progressive seed design. The first points are intentionally conservative and
# should establish usable physical responses. The later points progressively
# increase authority and are allowed to be aggressive so Bayesian optimization
# sees the transition from slow/stable to fast/potentially underdamped.
$seedBlock = @'
seedSettings = {
    {0.5,    0.0,  0.5},
    {1.0,    0.0,  0.5},
    {2.0,    0.0,  0.5},
    {5.0,    0.0,  1.0},
    {10.0,   0.0,  1.0},
    {20.0,   0.5,  2.0},
    {40.0,   1.0,  3.0},
    {80.0,   2.0,  5.0},
    {160.0,  4.0,  8.0},
    {320.0,  8.0, 12.0}
};
'@
$text = [regex]::Replace(
    $text,
    '(?s)seedSettings\s*=\s*\{.*?\n\};',
    $seedBlock.TrimEnd(),
    1
)

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
    '                "Maximum settling time: " || Char( maxSettlingTimeSeconds ) || " s\!N" || "Target response: under 1.0 s with a smooth home entry\!N" || "PID coefficient increment: 0.5\!N" ||'
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

# Refuse to launch a partially patched runtime script. This keeps JMP from
# silently optimizing without the physical worker connected or with stale seeds.
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
if ($text -notmatch '\{0\.5,\s+0\.0,\s+0\.5\}' -or $text -notmatch '\{320\.0,\s+8\.0,\s+12\.0\}') {
    throw "Runtime JSL validation failed: fast/smooth progressive seed design missing."
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
Write-Host "  PID step    : 0.5"
Write-Host "  Displacement: 300 deg"
Write-Host "  Target      : <1.0 s, smooth stop"
Write-Host "  Settle max  : 3.0 s"
Write-Host "  ACK retries : 3"
Write-Host "  Runtime JSL : $runtimeScript"
Invoke-Item $runtimeScript