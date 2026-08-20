param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$configPath = Join-Path $repo "config\discoverypid.json"
$builder = Join-Path $repo "tools\build_jmp_runtime.py"
$worker = Join-Path $repo "tools\run_position_trial.py"
$template = Join-Path $repo "jmp\DiscoveryPID_Position_V2.jsl"

foreach ($path in @($configPath, $builder, $worker, $template)) {
    if (-not (Test-Path $path)) { throw "Required JMP study file not found: $path" }
}

$config = Get-Content $configPath -Raw | ConvertFrom-Json
$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

$sessionDir = Join-Path $repo ("data\{0}\sessions" -f [string]$config.dataRoot)
New-Item -ItemType Directory -Force $sessionDir | Out-Null
$runtimeScript = Join-Path $env:TEMP "DiscoveryPID_Position_V2_runtime.jsl"

# Build the runtime JSL from config + the tracked template. The builder refuses
# to emit a runtime if multi-response Y columns, response limits, PID ranges,
# timeout handling, or live-table visibility are missing.
& $py $builder `
    --repo $repo `
    --port $Port `
    --python $py `
    --output $runtimeScript

if ($LASTEXITCODE -ne 0) {
    throw "JMP runtime build failed with exit code $LASTEXITCODE."
}

# A 3-second timeout is a right-censored bad settling response, not an unknown
# response. Keep every BayesOpt Y numeric while Return Failure explicitly marks
# the run as invalid. Likewise, no home entry gets the worst configured entry
# RPM response instead of a blank Y cell.
$text = Get-Content $runtimeScript -Raw
$censorAnchor = @'
    Close( resultDT, No Save );

    // ---------------------------------------------------------------
    // RELEASE-TO-HOME MEASUREMENT WARNING
'@
$censorReplacement = @'
    Close( resultDT, No Save );

    // Keep all MULTI-RESPONSE Y values numeric. A missing settling time means
    // the hard return deadline was not satisfied; record the censor limit.
    If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );
    If( Is Missing( resultList[7] ), resultList[7] = 10.0 );

    // ---------------------------------------------------------------
    // RELEASE-TO-HOME MEASUREMENT WARNING
'@
if (-not $text.Contains($censorAnchor)) {
    throw "Runtime JSL validation failed: timeout-censor insertion point missing."
}
$text = $text.Replace($censorAnchor, $censorReplacement)

# Validate the final file that JMP will actually execute.
$required = @(
    'MULTI-RESPONSE BAYESOPT DESIRABILITY',
    'BayesOpt mode: MULTI-RESPONSE DESIRABILITY',
    ':Name( "Return Failure" )',
    ':Name( "Settling Time (s)" )',
    ':Name( "PWM Chatter" )',
    ':Name( "Home Entry RPM" )',
    ':Name( "Overshoot (deg)" )',
    ':Oscillations',
    'kpHigh = 30.0;',
    'If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );',
    'dt << Bring Window To Front;'
)
foreach ($token in $required) {
    if (-not $text.Contains($token)) {
        throw "Runtime JSL validation failed: missing $token"
    }
}
if ($text.Contains('Y( :Score )')) {
    throw "Runtime JSL validation failed: scalar Score is still being used as BayesOpt Y."
}

[System.IO.File]::WriteAllText(
    $runtimeScript,
    $text,
    (New-Object System.Text.UTF8Encoding($false))
)

Write-Host "" 
Write-Host "Launching DiscoveryPID JMP physical study" -ForegroundColor Cyan
Write-Host "  Code branch : main"
Write-Host "  Port        : $Port"
Write-Host "  Study       : $($config.studyName)"
Write-Host "  BayesOpt    : MULTI-RESPONSE desirability" -ForegroundColor Yellow
Write-Host "  Responses   : failure | settling | chatter | entry RPM | overshoot | oscillations"
Write-Host "  Score       : diagnostic/setup ranking only; NOT a BayesOpt Y"
Write-Host "  PID range   : Kp $($config.pidRanges.kp[0])-$($config.pidRanges.kp[1]) | Ki $($config.pidRanges.ki[0])-$($config.pidRanges.ki[1]) | Kd $($config.pidRanges.kd[0])-$($config.pidRanges.kd[1])"
Write-Host "  PID step    : $($config.pidCoefficientStep)"
Write-Host "  Displacement: $($config.testOffsetDeg) deg"
Write-Host "  Target      : <1.0 s, smooth stop"
Write-Host "  Settle max  : $($config.maxSettlingTimeSeconds) s HARD CUTOFF"
Write-Host "  Timeout row : Settling=$($config.dataPolicy.timeoutSettlingResponseSeconds) s + Return Failure=1"
Write-Host "  Live table  : visible and brought to front after every run" -ForegroundColor Green
Write-Host "  Runtime JSL : $runtimeScript"
Write-Host ""

Invoke-Item $runtimeScript
