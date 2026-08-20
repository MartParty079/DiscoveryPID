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
$minimumDrivePWM = [double]$config.minimumDrivePWM
$minimumRSquare = [double]$config.bayesianOptimization.minimumRSquare
$missingEntryRpm = [double]$config.dataPolicy.missingHomeEntryRpmResponse
$catastrophicOvershootDeg = [double]$config.dataPolicy.catastrophicOvershootDeg
$pidStep = [double]$config.pidCoefficientStep
$chatterCap = [double]$config.dataPolicy.responseTransforms.pwmChatter.cap
$overshootCap = [double]$config.dataPolicy.responseTransforms.overshootDeg.cap
$entryRpmCap = [double]$config.dataPolicy.responseTransforms.homeEntryRpm.cap
$oscillationCap = [double]$config.dataPolicy.responseTransforms.oscillations.cap

$py = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\python.exe"
if (-not (Test-Path $py)) {
    $command = Get-Command python -ErrorAction SilentlyContinue
    if (-not $command) { throw "Python not found." }
    $py = $command.Source
}

$sessionDir = Join-Path $repo ("data\{0}\sessions" -f [string]$config.dataRoot)
New-Item -ItemType Directory -Force $sessionDir | Out-Null
$runtimeScript = Join-Path $env:TEMP "DiscoveryPID_Position_V2_runtime.jsl"

& $py $builder `
    --repo $repo `
    --port $Port `
    --python $py `
    --output $runtimeScript

if ($LASTEXITCODE -ne 0) {
    throw "JMP runtime build failed with exit code $LASTEXITCODE."
}

$text = Get-Content $runtimeScript -Raw

$minimumDriveLiteral = ("minimumDrivePWM = {0:F1};" -f $minimumDrivePWM)
if (-not $text.Contains('minimumDrivePWM = 60.0;')) {
    throw "Runtime JSL validation failed: minimumDrivePWM template value missing."
}
$text = $text.Replace('minimumDrivePWM = 60.0;', $minimumDriveLiteral)

# Expose the quality-failure threshold inside the runtime JSL. The physical
# worker remains unchanged; v49 classifies completed-but-wild responses before
# they are admitted to the Bayesian training table or best-setup selection.
$settleLiteral = ("maxSettlingTimeSeconds = {0:F1};" -f [double]$config.maxSettlingTimeSeconds)
if (-not $text.Contains($settleLiteral)) {
    throw "Runtime JSL validation failed: maxSettlingTimeSeconds literal missing."
}
$overshootLiteral = ("catastrophicOvershootDeg = {0:F1};" -f $catastrophicOvershootDeg)
$text = $text.Replace(
    $settleLiteral,
    $settleLiteral + "`n" + $overshootLiteral
)

# Missing settling/entry measurements are right-censored bad responses, not
# unknown values. Keep every BayesOpt Y numeric before the row reaches JMP.
$censorPattern = '(?m)^    Close\( resultDT, No Save \);\r?\n\r?\n    // ---------------------------------------------------------------\r?\n    // RELEASE-TO-HOME MEASUREMENT WARNING'
$censorMatches = [regex]::Matches($text, $censorPattern)
if ($censorMatches.Count -ne 1) {
    throw "Runtime JSL validation failed: expected one timeout-censor insertion point, found $($censorMatches.Count)."
}
$censorReplacement = @"
    Close( resultDT, No Save );

    // Keep all MULTI-RESPONSE Y values numeric. A missing settling time means
    // the hard return deadline was not satisfied; record the censor limit.
    If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );
    If( Is Missing( resultList[7] ), resultList[7] = $missingEntryRpm );

    // ---------------------------------------------------------------
    // RELEASE-TO-HOME MEASUREMENT WARNING
"@
$text = [regex]::Replace($text, $censorPattern, $censorReplacement, 1)

# A trial that technically returns but flies far past home is not a valid
# controller. Mark it as a failure in the JMP table so it cannot become the
# best setup PID and Return Failure learns the unstable region.
$qualityOld = @'
    Column( dt, "Status" )[rowIndex] = resultList[12];
    Column( dt, "Return Failure" )[rowIndex] = If( resultList[12] == "COMPLETE", 0, 1 );
'@
$qualityNew = @'
    qualityStatus = resultList[12];
    If(
        resultList[4] > catastrophicOvershootDeg,
        qualityStatus = "EXCESSIVE_OVERSHOOT";
        Print(
            "QUALITY FAILURE: raw overshoot " || Char( resultList[4] ) ||
            " deg exceeded " || Char( catastrophicOvershootDeg ) || " deg."
        );
    );
    Column( dt, "Status" )[rowIndex] = qualityStatus;
    Column( dt, "Return Failure" )[rowIndex] = If( qualityStatus == "COMPLETE", 0, 1 );
'@
if (-not $text.Contains($qualityOld)) {
    throw "Runtime JSL validation failed: v48 quality-status block missing."
}
$text = $text.Replace($qualityOld, $qualityNew)

# JMP can intentionally request a replicate. After 0.5-step quantization that
# caused v48 to execute the same PID many times. v49 replaces an already-tested
# proposal with the nearest untested point on the configured 0.5-grid.
$duplicateOld = @'
        proposedKd = Round( Column( dt, "KD" )[newRow] * 2 ) / 2;
        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;
'@
$duplicateNew = @'
        proposedKd = Round( Column( dt, "KD" )[newRow] * 2 ) / 2;

        originalProposedKp = proposedKp;
        originalProposedKi = proposedKi;
        originalProposedKd = proposedKd;

        If(
            combinationAlreadyCompleted( dt, proposedKp, proposedKi, proposedKd ),

            nearestFound = 0;
            nearestDistance = 1e99;
            nearestKp = .;
            nearestKi = .;
            nearestKd = .;

            For(
                candidateKp = kpLow,
                candidateKp <= kpHigh + 0.000001,
                candidateKp += 0.5,

                For(
                    candidateKi = kiLow,
                    candidateKi <= kiHigh + 0.000001,
                    candidateKi += 0.5,

                    For(
                        candidateKd = kdLow,
                        candidateKd <= kdHigh + 0.000001,
                        candidateKd += 0.5,

                        candidateDistance =
                            ((candidateKp - originalProposedKp) / Max( 0.5, kpHigh - kpLow )) ^ 2 +
                            ((candidateKi - originalProposedKi) / Max( 0.5, kiHigh - kiLow )) ^ 2 +
                            ((candidateKd - originalProposedKd) / Max( 0.5, kdHigh - kdLow )) ^ 2;

                        If(
                            !combinationAlreadyCompleted(
                                dt,
                                candidateKp,
                                candidateKi,
                                candidateKd
                            ) & candidateDistance < nearestDistance,

                            nearestFound = 1;
                            nearestDistance = candidateDistance;
                            nearestKp = candidateKp;
                            nearestKi = candidateKi;
                            nearestKd = candidateKd;
                        );
                    );
                );
            );

            If(
                nearestFound,

                proposedKp = nearestKp;
                proposedKi = nearestKi;
                proposedKd = nearestKd;

                Print(
                    "BAYESIAN DUPLICATE REPLACED: (" ||
                    Char( originalProposedKp ) || ", " ||
                    Char( originalProposedKi ) || ", " ||
                    Char( originalProposedKd ) || ") -> nearest untested (" ||
                    Char( proposedKp ) || ", " ||
                    Char( proposedKi ) || ", " ||
                    Char( proposedKd ) || ")"
                );,

                dt << Delete Rows( newRow );
                saveSnapshot( dt );
                stopRequested = 1;
                stopReason = "All focused 0.5-grid PID combinations have already been tested.";
                Break();
            );
        );

        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;
'@
if (-not $text.Contains($duplicateOld)) {
    throw "Runtime JSL validation failed: quantized Bayesian proposal block missing."
}
$text = $text.Replace($duplicateOld, $duplicateNew)

# Keep the live data grid in the foreground before each physical run as well as
# after saved results. The BayesOpt report can exist, but the table stays visible.
$text = $text.Replace(
    "            resultList =`n                runPhysicalTrial(",
    "            dt << Bring Window To Front;`n            resultList =`n                runPhysicalTrial("
)
$text = $text.Replace(
    "        resultList =`n            runPhysicalTrial(",
    "        dt << Bring Window To Front;`n        resultList =`n            runPhysicalTrial("
)

$minimumRSquareLiteral = ("Minimum RSquare( {0:F2} )" -f $minimumRSquare)
$required = @(
    'MULTI-RESPONSE BAYESOPT DESIRABILITY',
    'BayesOpt mode: MULTI-RESPONSE DESIRABILITY',
    'v48 response shaping: bounded/log-compressed for GP stability',
    ':Name( "Return Failure" )',
    ':Name( "Settling Time (s)" )',
    ':Name( "PWM Chatter" )',
    ':Name( "Home Entry RPM" )',
    ':Name( "Overshoot (deg)" )',
    ':Oscillations',
    'Raw PWM Chatter',
    'Raw Overshoot (deg)',
    'kpLow = 20.0;',
    'kpHigh = 26.0;',
    'kiLow = 5.0;',
    'kiHigh = 8.0;',
    'kdLow = 0.5;',
    'kdHigh = 4.0;',
    $minimumRSquareLiteral,
    $minimumDriveLiteral,
    $overshootLiteral,
    'If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );',
    'qualityStatus = "EXCESSIVE_OVERSHOOT";',
    'BAYESIAN DUPLICATE REPLACED',
    'nearest untested',
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
if ([regex]::Matches($text, 'dt << Bring Window To Front;').Count -lt 4) {
    throw "Runtime JSL validation failed: live table is not being foregrounded at all required stages."
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
Write-Host "  Responses   : failure | settling | bounded chatter | entry RPM | bounded overshoot | OSCILLATION COUNT" -ForegroundColor Yellow
Write-Host "  Conditioning: chatter=ln(1+raw) capped $chatterCap | overshoot cap $overshootCap deg | entry cap $entryRpmCap RPM | oscillation count cap $oscillationCap" -ForegroundColor Green
Write-Host "  Failure     : timeout/error OR raw overshoot > $catastrophicOvershootDeg deg"
Write-Host "  Duplicates  : rounded PID repeats replaced by nearest untested $pidStep-grid point" -ForegroundColor Green
Write-Host "  Model gate  : minimum R-square $minimumRSquare"
Write-Host "  Score       : diagnostic/setup ranking only; NOT a BayesOpt Y"
Write-Host "  PID range   : Kp $($config.pidRanges.kp[0])-$($config.pidRanges.kp[1]) | Ki $($config.pidRanges.ki[0])-$($config.pidRanges.ki[1]) | Kd $($config.pidRanges.kd[0])-$($config.pidRanges.kd[1])"
Write-Host "  PID step    : $pidStep"
Write-Host "  PWM floor   : $minimumDrivePWM (tapers near home)"
Write-Host "  Controller  : 100 Hz pure signed PID; reverse PWM is PID braking; zero PWM coasts" -ForegroundColor Green
Write-Host "  Firmware    : UNCHANGED v3.7 pure bidirectional PID"
Write-Host "  Displacement: $($config.testOffsetDeg) deg"
Write-Host "  Target      : <1.0 s, smooth stop, zero oscillations"
Write-Host "  Settle max  : $($config.maxSettlingTimeSeconds) s HARD CUTOFF"
Write-Host "  Timeout row : Settling=$($config.dataPolicy.timeoutSettlingResponseSeconds) s + Return Failure=1"
Write-Host "  Raw metrics : preserved in worker run CSV; raw chatter/overshoot also shown in JMP table"
Write-Host "  Live table  : kept visible/foregrounded through the study" -ForegroundColor Green
Write-Host "  Runtime JSL : $runtimeScript"
Write-Host ""

Invoke-Item $runtimeScript
