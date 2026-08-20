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
if ($pidStep -le 0) { throw "pidCoefficientStep must be greater than zero." }
$pidStepLiteral = $pidStep.ToString("0.###############", [Globalization.CultureInfo]::InvariantCulture)
$pidStepRegex = [regex]::Escape($pidStepLiteral)
$chatterCap = [double]$config.dataPolicy.responseTransforms.pwmChatter.cap
$overshootCap = [double]$config.dataPolicy.responseTransforms.overshootDeg.cap
$entryRpmCap = [double]$config.dataPolicy.responseTransforms.homeEntryRpm.cap
$oscillationCap = [double]$config.dataPolicy.responseTransforms.oscillations.cap
$taperStartDeg = [double]$config.returnSmoothing.minimumPwmTaperStartDeg
$taperEndDeg = [double]$config.returnSmoothing.minimumPwmTaperEndDeg
$integralBandDeg = [double]$config.returnSmoothing.integralEnableBandDeg
$integralMaxPWM = [double]$config.returnSmoothing.maximumIntegralContributionPWM

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

# Expose the quality-failure threshold inside the runtime JSL. v50 classifies
# completed-but-wild responses before they are admitted to the Bayesian table.
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
# controller. Mark it as a failure so it cannot become the best setup PID.
$qualityPattern = '(?m)^\s*Column\( dt, "Status" \)\[rowIndex\] = resultList\[12\];\r?\n\s*Column\( dt, "Return Failure" \)\[rowIndex\] = If\( resultList\[12\] == "COMPLETE", 0, 1 \);'
$qualityMatches = [regex]::Matches($text, $qualityPattern)
if ($qualityMatches.Count -ne 1) {
    throw "Runtime JSL validation failed: expected one quality-status insertion point, found $($qualityMatches.Count)."
}
$qualityReplacement = @'
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
$text = [regex]::Replace($text, $qualityPattern, $qualityReplacement, 1)

# JMP can intentionally request a replicate. With 0.001-resolution gains the
# design space is far too large for an exhaustive grid scan, so a duplicate is
# replaced by the nearest available axial neighbor on the configured grid.
$duplicatePatternTemplate = '(?m)^\s*proposedKd = Round\( Column\( dt, "KD" \)\[newRow\] / {0} \) \* {0};\r?\n\s*Column\( dt, "KP" \)\[newRow\] = proposedKp;\r?\n\s*Column\( dt, "KI" \)\[newRow\] = proposedKi;\r?\n\s*Column\( dt, "KD" \)\[newRow\] = proposedKd;'
$duplicatePattern = $duplicatePatternTemplate -f $pidStepRegex
$duplicateMatches = [regex]::Matches($text, $duplicatePattern)
if ($duplicateMatches.Count -ne 1) {
    throw "Runtime JSL validation failed: expected one $pidStepLiteral-quantized Bayesian proposal block, found $($duplicateMatches.Count)."
}
$duplicateReplacement = @"
        proposedKd = Round( Column( dt, "KD" )[newRow] / $pidStepLiteral ) * $pidStepLiteral;

        originalProposedKp = proposedKp;
        originalProposedKi = proposedKi;
        originalProposedKd = proposedKd;

        If(
            combinationAlreadyCompleted( dt, proposedKp, proposedKi, proposedKd ),

            nearestFound = 0;
            nearestKp = .;
            nearestKi = .;
            nearestKd = .;

            // Search outward in $pidStepLiteral increments. Only six axial
            // neighbors are checked at each radius, so 0.001 resolution stays fast.
            For(
                neighborRadius = 1,
                neighborRadius <= 1000 & !nearestFound,
                neighborRadius++,

                neighborOffset = neighborRadius * $pidStepLiteral;

                candidateKp = originalProposedKp + neighborOffset;
                If(
                    !nearestFound &
                    candidateKp <= kpHigh + 0.0000001 &
                    !combinationAlreadyCompleted( dt, candidateKp, originalProposedKi, originalProposedKd ),
                    nearestFound = 1;
                    nearestKp = candidateKp;
                    nearestKi = originalProposedKi;
                    nearestKd = originalProposedKd;
                );

                candidateKp = originalProposedKp - neighborOffset;
                If(
                    !nearestFound &
                    candidateKp >= kpLow - 0.0000001 &
                    !combinationAlreadyCompleted( dt, candidateKp, originalProposedKi, originalProposedKd ),
                    nearestFound = 1;
                    nearestKp = candidateKp;
                    nearestKi = originalProposedKi;
                    nearestKd = originalProposedKd;
                );

                candidateKi = originalProposedKi + neighborOffset;
                If(
                    !nearestFound &
                    candidateKi <= kiHigh + 0.0000001 &
                    !combinationAlreadyCompleted( dt, originalProposedKp, candidateKi, originalProposedKd ),
                    nearestFound = 1;
                    nearestKp = originalProposedKp;
                    nearestKi = candidateKi;
                    nearestKd = originalProposedKd;
                );

                candidateKi = originalProposedKi - neighborOffset;
                If(
                    !nearestFound &
                    candidateKi >= kiLow - 0.0000001 &
                    !combinationAlreadyCompleted( dt, originalProposedKp, candidateKi, originalProposedKd ),
                    nearestFound = 1;
                    nearestKp = originalProposedKp;
                    nearestKi = candidateKi;
                    nearestKd = originalProposedKd;
                );

                candidateKd = originalProposedKd + neighborOffset;
                If(
                    !nearestFound &
                    candidateKd <= kdHigh + 0.0000001 &
                    !combinationAlreadyCompleted( dt, originalProposedKp, originalProposedKi, candidateKd ),
                    nearestFound = 1;
                    nearestKp = originalProposedKp;
                    nearestKi = originalProposedKi;
                    nearestKd = candidateKd;
                );

                candidateKd = originalProposedKd - neighborOffset;
                If(
                    !nearestFound &
                    candidateKd >= kdLow - 0.0000001 &
                    !combinationAlreadyCompleted( dt, originalProposedKp, originalProposedKi, candidateKd ),
                    nearestFound = 1;
                    nearestKp = originalProposedKp;
                    nearestKi = originalProposedKi;
                    nearestKd = candidateKd;
                );
            );

            If(
                nearestFound,

                proposedKp = Round( nearestKp / $pidStepLiteral ) * $pidStepLiteral;
                proposedKi = Round( nearestKi / $pidStepLiteral ) * $pidStepLiteral;
                proposedKd = Round( nearestKd / $pidStepLiteral ) * $pidStepLiteral;

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
                stopReason = "No untested $pidStepLiteral-grid neighbor found within 1000 steps of the proposed PID.";
                Break();
            );
        );

        Column( dt, "KP" )[newRow] = proposedKp;
        Column( dt, "KI" )[newRow] = proposedKi;
        Column( dt, "KD" )[newRow] = proposedKd;
"@
$text = [regex]::Replace($text, $duplicatePattern, $duplicateReplacement, 1)

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
$pidQuantizationLiteral = "proposedKd = Round( Column( dt, `"KD`" )[newRow] / $pidStepLiteral ) * $pidStepLiteral;"
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
    $pidQuantizationLiteral,
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
Write-Host "  Duplicates  : rounded PID repeats replaced by nearest untested $pidStepLiteral-grid neighbor" -ForegroundColor Green
Write-Host "  Model gate  : minimum R-square $minimumRSquare"
Write-Host "  Score       : diagnostic/setup ranking only; NOT a BayesOpt Y"
Write-Host "  PID range   : Kp $($config.pidRanges.kp[0])-$($config.pidRanges.kp[1]) | Ki $($config.pidRanges.ki[0])-$($config.pidRanges.ki[1]) | Kd $($config.pidRanges.kd[0])-$($config.pidRanges.kd[1])"
Write-Host "  PID step    : $pidStepLiteral (nearest 0.001)" -ForegroundColor Green
Write-Host "  PWM floor   : $minimumDrivePWM; taper $taperStartDeg deg -> $taperEndDeg deg, then floor=0" -ForegroundColor Green
Write-Host "  Integral    : enabled only inside +/-$integralBandDeg deg; contribution limited to +/-$integralMaxPWM PWM" -ForegroundColor Green
Write-Host "  Controller  : 100 Hz pure signed PID; reverse PWM is PID braking; zero PWM coasts" -ForegroundColor Green
Write-Host "  Firmware    : v3.8 final-approach smoothing (flash required)" -ForegroundColor Yellow
Write-Host "  Displacement: $($config.testOffsetDeg) deg"
Write-Host "  Target      : <1.0 s, smooth stop, zero oscillations"
Write-Host "  Settle max  : $($config.maxSettlingTimeSeconds) s HARD CUTOFF"
Write-Host "  Timeout row : Settling=$($config.dataPolicy.timeoutSettlingResponseSeconds) s + Return Failure=1"
Write-Host "  Raw metrics : preserved in worker run CSV; raw chatter/overshoot also shown in JMP table"
Write-Host "  Live table  : kept visible/foregrounded through the study" -ForegroundColor Green
Write-Host "  Runtime JSL : $runtimeScript"
Write-Host ""

Invoke-Item $runtimeScript
