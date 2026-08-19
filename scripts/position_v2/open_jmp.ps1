param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$sourceScript = Join-Path $repo "jmp\DiscoveryPID_Position_V2.jsl"
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"

if (-not (Test-Path $sourceScript)) {
    throw "JMP script not found: $sourceScript"
}

if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

# JMP's flat-session save path must exist before the JSL runs.
$sessionDir = Join-Path $repo "data\position_autotune_directpid_v38_300deg10xrange\sessions"
New-Item -ItemType Directory -Force $sessionDir | Out-Null

# Build a runtime copy so collaborators do not need to edit machine-specific
# paths or serial ports in the tracked JSL. The tracked file remains clean.
$runtimeScript = Join-Path $env:TEMP "DiscoveryPID_Position_V2_runtime.jsl"
$text = Get-Content $sourceScript -Raw
$repoJsl = ($repo -replace '\\', '/') + '/'
$pyJsl = ($py -replace '\\', '/')

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
Write-Host "  Runtime JSL: $runtimeScript"
Invoke-Item $runtimeScript
