param(
    [switch]$NoClean,
    [string]$Port = ""
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$pio = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"

if (-not (Test-Path $pio)) {
    $command = Get-Command platformio -ErrorAction SilentlyContinue
    if (-not $command) { throw "PlatformIO not found." }
    $pio = $command.Source
}

Set-Location $repo

if (-not $NoClean) {
    & $pio run --target clean
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

& $pio run
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$uploadArgs = @("run", "--target", "upload")
if ($Port) {
    $uploadArgs += @("--upload-port", $Port)
    Write-Host "Uploading to $Port" -ForegroundColor Cyan
}

& $pio @uploadArgs
exit $LASTEXITCODE
