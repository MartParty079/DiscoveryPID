param(
    [string]$Port = "COM7"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$configPath = Join-Path $repo "config\discoverypid.json"
$builder = Join-Path $repo "tools\build_jmp_runtime.py"
$worker = Join-Path $repo "tools\run_position_trial_configured.py"
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
$responseNames = @($config.bayesianOptimization.responses | ForEach-Object { [string]$_.name })
$expectedResponses = @('Settling Time (s)', 'Overshoot (deg)', 'Oscillations')
if (($responseNames -join '|') -ne ($expectedResponses -join '|')) {
    throw "JMP configuration must use exactly: Settling Time (s), Overshoot (deg), Oscillations."
}

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

# Expose the quality-failure threshold inside the runtime JSL.
$settleLiteral = ("maxSettlingTimeSeconds = {0:F1};" -f [double]$config.maxSettlingTimeSeconds)
if (-not $text.Contains($settleLiteral)) {
    throw "Runtime JSL validation failed: maxSettlingTimeSeconds literal missing."
}
$overshootLiteral = ("catastrophicOvershootDeg = {0:F1};" -f $catastrophicOvershootDeg)
$text = $text.Replace(
    $settleLiteral,
    $settleLiteral + "`n" + $overshootLiteral
)

# Missing measurements remain numeric. Settling is a modeled response; entry RPM
# is retained only as a diagnostic column.
$censorPattern = '(?m)^    Close\( resultDT, No Save \);\r?\n\r?\n    // ---------------------------------------------------------------\r?\n    // RELEASE-TO-HOME MEASUREMENT WARNING'
$censorMatches = [regex]::Matches($text, $censorPattern)
if ($censorMatches.Count -ne 1) {
    throw "Runtime JSL validation failed: expected one timeout-censor insertion point, found $($censorMatches.Count)."
}
$censorReplacement = @"
    Close( resultDT, No Save );

    If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );
    If( Is Missing( resultList[7] ), resultList[7] = $missingEntryRpm );

    // ---------------------------------------------------------------
    // RELEASE-TO-HOME MEASUREMENT WARNING
"@
$text = [regex]::Replace($text, $censorPattern, $censorReplacement, 1)

# Keep failure status as a diagnostic, but because Return Failure is no longer a
# Bayesian Y, force failed/non-complete rows to the worst values of the three
# actual modeled responses. This prevents a no-motion failure from looking like
# zero overshoot / zero oscillation and becoming attractive to JMP.
$qualityPattern = '(?m)^\s*Column\( dt, "Status" \)\[rowIndex\] = resultList\[12\];\r?\n\s*Column\( dt, "Return Failure" \)\[rowIndex\] = If\( resultList\[12\] == "COMPLETE", 0, 1 \);'
$qualityMatches = [regex]::Matches($text, $qualityPattern)
if ($qualityMatches.Count -ne 1) {
    throw "Runtime JSL validation failed: expected one quality-status insertion point, found $($qualityMatches.Count)."
}
$qualityReplacement = @"
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

    If(
        qualityStatus != "COMPLETE",
        Column( dt, "Settling Time (s)" )[rowIndex] = maxSettlingTimeSeconds;
        Column( dt, "Overshoot (deg)" )[rowIndex] = $overshootCap;
        Column( dt, "Oscillations" )[rowIndex] = $oscillationCap;
        Print(
            "THREE-RESPONSE FAILURE PENALTY: Settling=" || Char( maxSettlingTimeSeconds ) ||
            " Overshoot=$overshootCap Oscillations=$oscillationCap"
        );
    );
"@
$text = [regex]::Replace($text, $qualityPattern, $qualityReplacement, 1)

# Replace repeated fine-grid proposals with a nearby untested point without
# attempting an exhaustive 3-D scan.
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

# Keep the data table available during physical execution. JMP's Bayesian report
# still contains the three response model/profiler plots.
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
$kpLowLiteral = ("kpLow = {0:F1};" -f [double]$config.pidRanges.kp[0])
$kpHighLiteral = ("kpHigh = {0:F1};" -f [double]$config.pidRanges.kp[1])
$kiLowLiteral = ("kiLow = {0:F1};" -f [double]$config.pidRanges.ki[0])
$kiHighLiteral = ("kiHigh = {0:F1};" -f [double]$config.pidRanges.ki[1])
$kdLowLiteral = ("kdLow = {0:F1};" -f [double]$config.pidRanges.kd[0])
$kdHighLiteral = ("kdHigh = {0:F1};" -f [double]$config.pidRanges.kd[1])
$workerLiteral = $worker.Replace('\', '/')
$responseTokens = @(
    ':Name( "Settling Time (s)" )',
    ':Name( "Overshoot (deg)" )',
    ':Name( "Oscillations" )'
)

$required = @(
    'THREE-RESPONSE BAYESOPT DESIRABILITY',
    'BayesOpt mode: THREE-RESPONSE DESIRABILITY',
    'JMP models/profilers plot only these three responses',
    $responseTokens[0],
    $responseTokens[1],
    $responseTokens[2],
    'Raw PWM Chatter',
    'Raw Overshoot (deg)',
    $kpLowLiteral,
    $kpHighLiteral,
    $kiLowLiteral,
    $kiHighLiteral,
    $kdLowLiteral,
    $kdHighLiteral,
    $minimumRSquareLiteral,
    $minimumDriveLiteral,
    $overshootLiteral,
    $pidQuantizationLiteral,
    $workerLiteral,
    'If( Is Missing( resultList[2] ), resultList[2] = maxSettlingTimeSeconds );',
    'THREE-RESPONSE FAILURE PENALTY',
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
Write-Host "  BayesOpt    : THREE-RESPONSE desirability" -ForegroundColor Yellow
Write-Host "  JMP Y       : Settling Time (s) | Overshoot (deg) | Oscillations" -ForegroundColor Yellow
Write-Host "  Priority    : fastest settling with essentially zero overshoot and zero oscillations" -ForegroundColor Green
Write-Host "  Desirability: settling importance 10 | overshoot 10 | oscillations 8"
Write-Host "  Failure rows: forced to Settling=$($config.maxSettlingTimeSeconds), Overshoot=$overshootCap, Oscillations=$oscillationCap" -ForegroundColor Yellow
Write-Host "  Diagnostics : chatter, entry RPM, score, Return Failure, raw overshoot remain saved but are NOT JMP responses"
Write-Host "  JMP plots   : model/profiler output uses only the three responses above" -ForegroundColor Green
Write-Host "  Duplicates  : rounded PID repeats replaced by nearest untested $pidStepLiteral-grid neighbor" -ForegroundColor Green
Write-Host "  Model gate  : minimum R-square $minimumRSquare"
Write-Host "  Score       : diagnostic/setup ranking only; NOT a BayesOpt Y"
Write-Host "  PID range   : Kp $($config.pidRanges.kp[0])-$($config.pidRanges.kp[1]) | Ki $($config.pidRanges.ki[0])-$($config.pidRanges.ki[1]) | Kd $($config.pidRanges.kd[0])-$($config.pidRanges.kd[1])"
Write-Host "  PID step    : $pidStepLiteral (nearest $pidStepLiteral)" -ForegroundColor Green
Write-Host "  Worker      : $worker" -ForegroundColor Green
Write-Host "  PWM floor   : $minimumDrivePWM; taper $taperStartDeg deg -> $taperEndDeg deg, then floor=0" -ForegroundColor Green
Write-Host "  Integral    : enabled only inside +/-$integralBandDeg deg; contribution limited to +/-$integralMaxPWM PWM" -ForegroundColor Green
Write-Host "  Controller  : 100 Hz pure signed PID; reverse PWM is PID braking; zero PWM coasts" -ForegroundColor Green
Write-Host "  Firmware    : v3.8 final-approach smoothing (flash required)" -ForegroundColor Yellow
Write-Host "  Displacement: $($config.testOffsetDeg) deg"
Write-Host "  Settle max  : $($config.maxSettlingTimeSeconds) s HARD CUTOFF"
Write-Host "  Runtime JSL : $runtimeScript"
Write-Host ""

Invoke-Item $runtimeScript
