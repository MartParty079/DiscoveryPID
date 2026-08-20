param(
    [Parameter(Position = 0)]
    [ValidateSet("menu", "home", "test", "jmp", "manual", "help")]
    [string]$Command = "menu",

    [string]$Port = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repo = (Resolve-Path $PSScriptRoot).Path
$configPath = Join-Path $repo "config\discoverypid.json"
$settingsPath = Join-Path $repo ".git\discoverypid-user.json"

if (-not (Test-Path (Join-Path $repo ".git"))) {
    throw "Run DiscoveryPID.ps1 from the DiscoveryPID Git checkout."
}

if (-not (Test-Path $configPath)) {
    throw "Project configuration not found: $configPath"
}

$project = Get-Content $configPath -Raw | ConvertFrom-Json

function Get-SavedPort {
    $defaultPort = [string]$project.defaultPort

    if (Test-Path $settingsPath) {
        try {
            $saved = Get-Content $settingsPath -Raw | ConvertFrom-Json
            if ($saved.Port) {
                return [string]$saved.Port
            }
        }
        catch {
            Write-Warning "Ignoring unreadable local port settings."
        }
    }

    return $defaultPort
}

function Save-Port([string]$Value) {
    [pscustomobject]@{ Port = $Value } |
        ConvertTo-Json |
        Set-Content -Path $settingsPath -Encoding UTF8
}

$script:ActivePort = Get-SavedPort
if ($Port) {
    $script:ActivePort = $Port.ToUpperInvariant()
    Save-Port $script:ActivePort
}

function Get-Python {
    $py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
    if (Test-Path $py) { return $py }

    $command = Get-Command python -ErrorAction SilentlyContinue
    if ($command) { return $command.Source }

    throw "Python was not found."
}

function Round-PidValue([double]$Value) {
    $step = [double]$project.pidCoefficientStep
    if ($step -le 0) {
        throw "pidCoefficientStep must be greater than zero."
    }

    return [Math]::Round(
        $Value / $step,
        0,
        [MidpointRounding]::AwayFromZero
    ) * $step
}

function Read-PidValue([string]$Name, [double]$Minimum, [double]$Maximum) {
    while ($true) {
        $text = Read-Host "$Name [$Minimum .. $Maximum]"
        $value = 0.0

        if (-not [double]::TryParse($text, [ref]$value)) {
            Write-Warning "Enter a number."
            continue
        }

        if ($value -lt $Minimum -or $value -gt $Maximum) {
            Write-Warning "$Name must be between $Minimum and $Maximum."
            continue
        }

        return Round-PidValue $value
    }
}

function Start-Home {
    Write-Host "HOME mode - fresh calibration on $($script:ActivePort)" -ForegroundColor Cyan
    & (Join-Path $repo "scripts\position_v2\home.ps1") -Port $script:ActivePort
    if ($LASTEXITCODE -ne 0) { throw "Homing failed." }
}

function Start-Test {
    $kp = [double]$project.defaultPID.kp
    $ki = [double]$project.defaultPID.ki
    $kd = [double]$project.defaultPID.kd

    Write-Host "TEST mode" -ForegroundColor Cyan
    Write-Host ("Default baseline: Kp={0:F3} Ki={1:F3} Kd={2:F3}, displacement=300 deg" -f $kp, $ki, $kd)

    & (Join-Path $repo "scripts\position_v2\control_test.ps1") `
        -Port $script:ActivePort `
        -TestOffsetDeg 300.0 `
        -Kp $kp `
        -Ki $ki `
        -Kd $kd

    if ($LASTEXITCODE -ne 0) { throw "Physical TEST mode failed." }
}

function Start-Jmp {
    Write-Host "JMP mode - Bayesian physical study" -ForegroundColor Cyan
    & (Join-Path $repo "scripts\position_v2\open_jmp.ps1") -Port $script:ActivePort
}

function Start-Manual {
    $pidStep = [double]$project.pidCoefficientStep
    $setupKp = [double]$project.defaultPID.kp
    $setupKi = [double]$project.defaultPID.ki
    $setupKd = [double]$project.defaultPID.kd

    Write-Host ""
    Write-Host "MANUAL PID mode" -ForegroundColor Cyan
    Write-Host "Enter one PID set for a single 300-degree physical trial."
    Write-Host "Values are rounded to the nearest $pidStep before running."
    Write-Host ("Outbound/setup PID stays at default Kp={0:F3} Ki={1:F3} Kd={2:F3}." -f $setupKp, $setupKi, $setupKd)
    Write-Host ""

    $kp = Read-PidValue "Kp" ([double]$project.pidRanges.kp[0]) ([double]$project.pidRanges.kp[1])
    $ki = Read-PidValue "Ki" ([double]$project.pidRanges.ki[0]) ([double]$project.pidRanges.ki[1])
    $kd = Read-PidValue "Kd" ([double]$project.pidRanges.kd[0]) ([double]$project.pidRanges.kd[1])
    $minimumDrivePWM = [double]$project.minimumDrivePWM

    Write-Host ""
    Write-Host "Exact return PID to run:" -ForegroundColor Yellow
    Write-Host ("  Kp = {0:F3}" -f $kp)
    Write-Host ("  Ki = {0:F3}" -f $ki)
    Write-Host ("  Kd = {0:F3}" -f $kd)
    Write-Host ("  Setup PID = {0:F3} / {1:F3} / {2:F3}" -f $setupKp, $setupKi, $setupKd)
    Write-Host "  Displacement = 300 deg"
    Write-Host "  Settling max = 3.0 s HARD CUTOFF"
    Write-Host "  PWM floor = $minimumDrivePWM (tapers near home)"
    Write-Host "  Controller = 100 Hz active-brake direct PID"
    Write-Host ""

    $confirm = Read-Host "Run this physical trial? [Y/n]"
    if ($confirm -match '^[Nn]') {
        Write-Host "Manual trial cancelled."
        return
    }

    $py = Get-Python
    $worker = Join-Path $repo "tools\run_position_trial_configured.py"
    $stamp = Get-Date -Format "yyyyMMdd_HHmmss"
    $dir = Join-Path $repo "data\manual_trials\$stamp"
    New-Item -ItemType Directory -Force $dir | Out-Null

    $result = Join-Path $dir "result.csv"
    $raw = Join-Path $dir "response.csv"
    $history = Join-Path $dir "history.csv"

    Write-Host "Running manual physical trial..." -ForegroundColor Cyan

    & $py $worker `
        --port $script:ActivePort `
        --trial 1 `
        --kp $kp `
        --ki $ki `
        --kd $kd `
        --test-offset 300 `
        --return-max-pwm 225 `
        --home-padding-pulses 2 `
        --max-settling-seconds 3 `
        --minimum-drive-pwm $minimumDrivePWM `
        --setup-kp $setupKp `
        --setup-ki $setupKi `
        --setup-kd $setupKd `
        --result-file $result `
        --raw-file $raw `
        --history-file $history

    $code = $LASTEXITCODE

    Write-Host ""
    if (Test-Path $result) {
        Write-Host "Manual result:" -ForegroundColor Cyan
        Get-Content $result
        Write-Host ""
        Write-Host "Saved under: $dir"
    }

    if ($code -ne 0) {
        throw "Manual physical trial worker exited with code $code."
    }
}

function Show-Help {
    Write-Host @"
DiscoveryPID modes

  .\DiscoveryPID.ps1 home
      Force fresh home/calibration.

  .\DiscoveryPID.ps1 test
      Run the configured default PID physical test:
      Kp=$($project.defaultPID.kp), Ki=$($project.defaultPID.ki), Kd=$($project.defaultPID.kd), 300-degree displacement.

  .\DiscoveryPID.ps1 jmp
      Launch the current JMP Bayesian study.
      Current PID limits and rounding come from config/discoverypid.json.

  .\DiscoveryPID.ps1 manual
      Prompt for return Kp/Ki/Kd and run one physical trial.
      The outbound/setup move uses the configured default PID.
      PID values are rounded to the nearest $($project.pidCoefficientStep) and use configured limits.

Run .\DiscoveryPID.ps1 with no command for the four-option menu.
Optional port override: .\DiscoveryPID.ps1 manual -Port COM8
"@
}

function Show-Menu {
    while ($true) {
        Clear-Host
        Write-Host "DiscoveryPID" -ForegroundColor Cyan
        Write-Host $project.profileName -ForegroundColor DarkCyan
        Write-Host "Port: $($script:ActivePort)"
        Write-Host ("Default PID: Kp={0:F3} Ki={1:F3} Kd={2:F3}" -f [double]$project.defaultPID.kp, [double]$project.defaultPID.ki, [double]$project.defaultPID.kd)
        Write-Host "PID limits: Kp <= $($project.pidRanges.kp[1]), Ki <= $($project.pidRanges.ki[1]), Kd <= $($project.pidRanges.kd[1])"
        Write-Host "PID rounding: nearest $($project.pidCoefficientStep)"
        Write-Host "PWM floor: $($project.minimumDrivePWM)"
        Write-Host ""
        Write-Host "[1] HOME   - recalibrate / set zero"
        Write-Host "[2] TEST   - default PID 300-degree physical test"
        Write-Host "[3] JMP    - Bayesian optimization"
        Write-Host "[4] MANUAL - enter Kp / Ki / Kd"
        Write-Host "[P] PORT   - change serial port"
        Write-Host "[Q] QUIT"
        Write-Host ""

        $choice = Read-Host "Mode"

        try {
            switch -Regex ($choice) {
                '^[1Hh]$' { Start-Home; break }
                '^[2Tt]$' { Start-Test; break }
                '^[3Jj]$' { Start-Jmp; break }
                '^[4Mm]$' { Start-Manual; break }
                '^[Pp]$' {
                    $newPort = Read-Host "Serial port [$($script:ActivePort)]"
                    if ($newPort) {
                        $script:ActivePort = $newPort.ToUpperInvariant()
                        Save-Port $script:ActivePort
                    }
                    continue
                }
                '^[Qq]$' { return }
                default { continue }
            }
        }
        catch {
            Write-Host ""
            Write-Host $_.Exception.Message -ForegroundColor Red
        }

        Write-Host ""
        [void](Read-Host "Press Enter for menu")
    }
}

switch ($Command) {
    "home"   { Start-Home }
    "test"   { Start-Test }
    "jmp"    { Start-Jmp }
    "manual" { Start-Manual }
    "help"   { Show-Help }
    "menu"   { Show-Menu }
}