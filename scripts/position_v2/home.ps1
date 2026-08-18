param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"

if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

Set-Location $repo
& $py ".\tools\home_position.py" --port $Port
exit $LASTEXITCODE
