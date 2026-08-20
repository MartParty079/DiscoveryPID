param(
    [string]$Port = "COM7",
    [double]$TestOffsetDeg = 300.0,
    [double]$Kp = 10.0,
    [double]$Ki = 0.0,
    [double]$Kd = 1.0
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$config = Get-Content (Join-Path $repo "config\discoverypid.json") -Raw | ConvertFrom-Json
$minimumDrivePWM = [double]$config.minimumDrivePWM
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"

if (-not (Test-Path $py)) { throw "PlatformIO Python not found: $py" }

Set-Location $repo
$dir = ".\data\position_autotune_control_test"
New-Item -ItemType Directory -Force $dir | Out-Null

Write-Host "DirectPID physical control test" -ForegroundColor Cyan
Write-Host "Expected: home -> $TestOffsetDeg deg -> candidate PID return home"
Write-Host "Controller: 100 Hz active-brake direct PID | PWM floor: $minimumDrivePWM"

& $py ".\tools\run_position_trial.py" `
    --port $Port `
    --trial 1 `
    --kp $Kp `
    --ki $Ki `
    --kd $Kd `
    --test-offset $TestOffsetDeg `
    --return-max-pwm 225 `
    --home-padding-pulses 2 `
    --minimum-drive-pwm $minimumDrivePWM `
    --setup-kp $Kp `
    --setup-ki $Ki `
    --setup-kd $Kd `
    --result-file "$dir\result.csv" `
    --raw-file "$dir\raw.csv" `
    --history-file "$dir\history.csv"

if (Test-Path "$dir\result.csv") {
    Write-Host "\nResult:"
    Get-Content "$dir\result.csv"
}

exit $LASTEXITCODE
