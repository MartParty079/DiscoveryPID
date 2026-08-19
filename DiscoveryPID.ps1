param(
    [Parameter(Position = 0)]
    [ValidateSet(
        "menu", "latest", "full", "build", "home", "jmp", "smoke",
        "control", "status", "branch", "fork", "port", "update", "help"
    )]
    [string]$Command = "menu",

    [string]$Port = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$repo = (Resolve-Path $PSScriptRoot).Path
$configPath = Join-Path $repo "config\discoverypid.json"

if (-not (Test-Path (Join-Path $repo ".git"))) {
    throw "Run DiscoveryPID.ps1 from a Git checkout. .git was not found at: $repo"
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    throw "Git was not found on PATH."
}

if (-not (Test-Path $configPath)) {
    throw "Project configuration was not found: $configPath"
}

$project = Get-Content $configPath -Raw | ConvertFrom-Json
$settingsPath = Join-Path $repo ".git\discoverypid-user.json"

function Get-UserSettings {
    $defaults = [ordered]@{
        Port = [string]$project.defaultPort
        Remote = [string]$project.defaultRemote
    }

    if (Test-Path $settingsPath) {
        try {
            $saved = Get-Content $settingsPath -Raw | ConvertFrom-Json
            if ($saved.Port) { $defaults.Port = [string]$saved.Port }
            if ($saved.Remote) { $defaults.Remote = [string]$saved.Remote }
        }
        catch {
            Write-Warning "Ignoring unreadable local settings: $settingsPath"
        }
    }

    [pscustomobject]$defaults
}

function Save-UserSettings {
    param([Parameter(Mandatory)]$Settings)

    $Settings |
        ConvertTo-Json |
        Set-Content -Path $settingsPath -Encoding UTF8
}

$script:settings = Get-UserSettings
if ($Port) {
    $script:settings.Port = $Port
    Save-UserSettings $script:settings
}

function Invoke-Git {
    param([Parameter(Mandatory)][string[]]$Arguments)

    & git -C $repo @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Git command failed: git $($Arguments -join ' ')"
    }
}

function Get-GitText {
    param([Parameter(Mandatory)][string[]]$Arguments)

    $text = (& git -C $repo @Arguments 2>$null | Out-String).Trim()
    $code = $LASTEXITCODE
    if ($code -ne 0) { return "" }
    $text
}

function Get-CurrentBranch {
    Get-GitText @("branch", "--show-current")
}

function Test-DirtyWorktree {
    [bool](Get-GitText @("status", "--porcelain"))
}

function Assert-CleanForSwitch {
    if (Test-DirtyWorktree) {
        throw @"
Your working tree has uncommitted changes.
Commit or stash them before switching code versions/forks.

Use menu option S to see status.
"@
    }
}

function Get-RemoteNames {
    @(Get-GitText @("remote") -split "`r?`n" | Where-Object { $_ })
}

function Get-SelectedRemote {
    $remotes = Get-RemoteNames
    if ($script:settings.Remote -and $remotes -contains $script:settings.Remote) {
        return [string]$script:settings.Remote
    }
    if ($remotes -contains [string]$project.defaultRemote) {
        return [string]$project.defaultRemote
    }
    if ($remotes.Count -gt 0) {
        return [string]$remotes[0]
    }
    ""
}

function Pause-Menu {
    Write-Host ""
    [void](Read-Host "Press Enter to return to the menu")
}

function Get-FirmwareId {
    $path = Join-Path $repo "src\main.cpp"
    if (-not (Test-Path $path)) { return "unknown" }
    $text = Get-Content $path -Raw
    if ($text -match '"(DiscoveryPID [^"]+)"') {
        return $Matches[1]
    }
    "unknown"
}

function Show-ProjectStatus {
    $branch = Get-CurrentBranch
    $remote = Get-SelectedRemote
    $remoteUrl = if ($remote) { Get-GitText @("remote", "get-url", $remote) } else { "none" }
    $dirty = Test-DirtyWorktree

    Write-Host ""
    Write-Host "DiscoveryPID status" -ForegroundColor Cyan
    Write-Host "-------------------"
    Write-Host ("Profile       : {0}" -f $project.profileName)
    Write-Host ("Firmware      : {0}" -f (Get-FirmwareId))
    Write-Host ("Branch        : {0}" -f $branch)
    Write-Host ("Latest branch : {0}/{1}" -f $project.latestRemote, $project.latestBranch)
    Write-Host ("Active remote : {0}" -f $remote)
    Write-Host ("Remote URL    : {0}" -f $remoteUrl)
    Write-Host ("Serial port   : {0}" -f $script:settings.Port)
    Write-Host ("Working tree  : {0}" -f $(if ($dirty) { "CHANGED" } else { "clean" }))

    if ($dirty) {
        Write-Host ""
        & git -C $repo status --short
    }
}

function Use-LatestCode {
    $remote = [string]$project.latestRemote
    $branch = [string]$project.latestBranch
    $current = Get-CurrentBranch

    if ($current -ne $branch) {
        Assert-CleanForSwitch
    }

    Write-Host "Fetching latest supported code from $remote/$branch..." -ForegroundColor Cyan
    Invoke-Git @("fetch", $remote, "--prune")

    & git -C $repo show-ref --verify --quiet "refs/heads/$branch"
    $localExists = ($LASTEXITCODE -eq 0)

    if ($localExists) {
        Invoke-Git @("switch", $branch)
    }
    else {
        Invoke-Git @("switch", "--track", "-c", $branch, "$remote/$branch")
    }

    Invoke-Git @("pull", "--ff-only", $remote, $branch)
    Write-Host "Latest code is active: $branch" -ForegroundColor Green
}

function Update-CurrentCode {
    $branch = Get-CurrentBranch
    $remote = Get-SelectedRemote

    if (-not $branch) { throw "Detached HEAD: switch to a branch first." }
    if (-not $remote) { throw "No Git remote is configured." }

    Write-Host "Updating $branch from $remote..." -ForegroundColor Cyan
    Invoke-Git @("fetch", $remote, "--prune")

    & git -C $repo ls-remote --exit-code --heads $remote $branch *> $null
    if ($LASTEXITCODE -ne 0) {
        throw "Remote '$remote' does not contain branch '$branch'. Use the Code Version or Fork menu."
    }

    Invoke-Git @("pull", "--ff-only", $remote, $branch)
    Write-Host "Updated $branch." -ForegroundColor Green
}

function Select-CodeVersion {
    Assert-CleanForSwitch
    Write-Host "Fetching branch lists..." -ForegroundColor Cyan
    Invoke-Git @("fetch", "--all", "--prune")

    $items = @()
    $locals = @(Get-GitText @("for-each-ref", "--format=%(refname:short)", "refs/heads") -split "`r?`n" | Where-Object { $_ })
    $remotes = @(Get-GitText @("for-each-ref", "--format=%(refname:short)", "refs/remotes") -split "`r?`n" | Where-Object { $_ -and $_ -notmatch '/HEAD$' })

    foreach ($name in $locals) {
        $items += [pscustomobject]@{ Type = "local"; Ref = $name; Label = "LOCAL   $name" }
    }
    foreach ($name in $remotes) {
        $items += [pscustomobject]@{ Type = "remote"; Ref = $name; Label = "REMOTE  $name" }
    }

    if ($items.Count -eq 0) {
        throw "No branches were found."
    }

    Write-Host ""
    Write-Host "Code versions / Git branches" -ForegroundColor Cyan
    for ($i = 0; $i -lt $items.Count; $i++) {
        Write-Host ("[{0,2}] {1}" -f ($i + 1), $items[$i].Label)
    }
    Write-Host "[ Q] Cancel"

    $choice = Read-Host "Choose code version"
    if ($choice -match '^[Qq]$') { return }

    $number = 0
    if (-not [int]::TryParse($choice, [ref]$number) -or $number -lt 1 -or $number -gt $items.Count) {
        Write-Warning "Invalid selection."
        return
    }

    $item = $items[$number - 1]
    if ($item.Type -eq "local") {
        Invoke-Git @("switch", $item.Ref)
    }
    else {
        $slash = $item.Ref.IndexOf('/')
        if ($slash -lt 1) { throw "Unexpected remote branch name: $($item.Ref)" }
        $remote = $item.Ref.Substring(0, $slash)
        $branch = $item.Ref.Substring($slash + 1)

        if ($locals -contains $branch) {
            Invoke-Git @("switch", $branch)
        }
        else {
            Invoke-Git @("switch", "--track", "-c", $branch, $item.Ref)
        }

        $script:settings.Remote = $remote
        Save-UserSettings $script:settings
    }

    Write-Host "Active code: $(Get-CurrentBranch)" -ForegroundColor Green
}

function Select-ForkRemote {
    $remotes = Get-RemoteNames
    $selected = Get-SelectedRemote

    Write-Host ""
    Write-Host "GitHub forks / remotes" -ForegroundColor Cyan
    for ($i = 0; $i -lt $remotes.Count; $i++) {
        $name = $remotes[$i]
        $url = Get-GitText @("remote", "get-url", $name)
        $mark = if ($name -eq $selected) { "*" } else { " " }
        Write-Host ("{0} [{1,2}] {2}  {3}" -f $mark, ($i + 1), $name, $url)
    }
    Write-Host "  [ A] Add a fork/remote"
    Write-Host "  [ Q] Cancel"

    $choice = Read-Host "Choose remote"
    if ($choice -match '^[Qq]$') { return }

    if ($choice -match '^[Aa]$') {
        $url = Read-Host "Fork URL (example: https://github.com/user/DiscoveryPID.git)"
        if (-not $url) { return }

        $suggested = "fork"
        if ($url -match 'github\.com[:/](?<owner>[^/]+)/') {
            $suggested = $Matches.owner
        }

        $name = Read-Host "Remote name [$suggested]"
        if (-not $name) { $name = $suggested }
        if (Get-RemoteNames -contains $name) {
            throw "Remote '$name' already exists."
        }

        Invoke-Git @("remote", "add", $name, $url)
        Invoke-Git @("fetch", $name, "--prune")
        $script:settings.Remote = $name
        Save-UserSettings $script:settings
        Write-Host "Active collaboration remote: $name" -ForegroundColor Green
        return
    }

    $number = 0
    if (-not [int]::TryParse($choice, [ref]$number) -or $number -lt 1 -or $number -gt $remotes.Count) {
        Write-Warning "Invalid selection."
        return
    }

    $script:settings.Remote = $remotes[$number - 1]
    Save-UserSettings $script:settings
    Write-Host "Active collaboration remote: $($script:settings.Remote)" -ForegroundColor Green
}

function Select-SerialPort {
    Write-Host "Current serial port: $($script:settings.Port)" -ForegroundColor Cyan
    $newPort = Read-Host "New port (example COM7; Enter keeps current)"
    if ($newPort) {
        $script:settings.Port = $newPort.ToUpperInvariant()
        Save-UserSettings $script:settings
        Write-Host "Serial port saved: $($script:settings.Port)" -ForegroundColor Green
    }
}

function Build-AndUpload {
    & (Join-Path $repo "scripts\position_v2\build_upload.ps1") -Port $script:settings.Port
    if ($LASTEXITCODE -ne 0) { throw "Build/upload failed." }
}

function Start-Homing {
    & (Join-Path $repo "scripts\position_v2\home.ps1") -Port $script:settings.Port
    if ($LASTEXITCODE -ne 0) { throw "Homing failed." }
}

function Start-Jmp {
    & (Join-Path $repo "scripts\position_v2\open_jmp.ps1") -Port $script:settings.Port
}

function Start-SmokeTest {
    & (Join-Path $repo "scripts\position_v2\smoke_test.ps1") -Port $script:settings.Port
    if ($LASTEXITCODE -ne 0) { throw "Smoke test failed." }
}

function Start-ControlTest {
    & (Join-Path $repo "scripts\position_v2\control_test.ps1") -Port $script:settings.Port
    if ($LASTEXITCODE -ne 0) { throw "Control test failed." }
}

function Run-FullWorkflow {
    Write-Host "Full workflow: latest code -> build/upload -> home -> JMP" -ForegroundColor Cyan
    Use-LatestCode
    Build-AndUpload
    Start-Homing
    Start-Jmp
}

function Show-HelpText {
    Write-Host @"
DiscoveryPID terminal launcher

Interactive:
  .\DiscoveryPID.ps1

Direct commands:
  .\DiscoveryPID.ps1 latest    Switch/update to the latest supported code
  .\DiscoveryPID.ps1 full      Latest -> build/upload -> home -> JMP
  .\DiscoveryPID.ps1 build     Build and upload firmware
  .\DiscoveryPID.ps1 home      Force fresh homing/calibration
  .\DiscoveryPID.ps1 jmp       Launch the current JMP study
  .\DiscoveryPID.ps1 smoke     Run the smaller hardware smoke test
  .\DiscoveryPID.ps1 control   Run the full physical control test
  .\DiscoveryPID.ps1 status    Show branch, firmware, fork, port, and Git status
  .\DiscoveryPID.ps1 branch    Choose a code version / Git branch
  .\DiscoveryPID.ps1 fork      Choose or add a GitHub fork / remote
  .\DiscoveryPID.ps1 port      Change the saved serial port
  .\DiscoveryPID.ps1 update    Fast-forward the current branch from active remote

Optional one-time port override:
  .\DiscoveryPID.ps1 home -Port COM8

Local choices are stored under .git and are not committed.
"@
}

function Show-Menu {
    while ($true) {
        Clear-Host
        $branch = Get-CurrentBranch
        $remote = Get-SelectedRemote

        Write-Host "DiscoveryPID" -ForegroundColor Cyan
        Write-Host $project.profileName -ForegroundColor DarkCyan
        Write-Host ""
        Write-Host "Code   : $branch"
        Write-Host "Fork   : $remote"
        Write-Host "Port   : $($script:settings.Port)"
        Write-Host "Latest : $($project.latestBranch)"
        Write-Host ""
        Write-Host "[1] Use latest code (default)" -ForegroundColor Green
        Write-Host "[2] Build + upload firmware"
        Write-Host "[3] Home / recalibrate"
        Write-Host "[4] Launch JMP"
        Write-Host "[5] Full workflow: latest -> build/upload -> home -> JMP"
        Write-Host "[6] Smoke test (small displacement)"
        Write-Host "[7] Full 300-degree control test"
        Write-Host "[8] Switch code version / branch"
        Write-Host "[9] Switch/add GitHub fork / remote"
        Write-Host "[P] Change serial port"
        Write-Host "[U] Update current branch from active fork"
        Write-Host "[S] Project / Git status"
        Write-Host "[H] Help"
        Write-Host "[Q] Exit"
        Write-Host ""

        $choice = Read-Host "Choose [1]"
        if (-not $choice) { $choice = "1" }

        try {
            switch -Regex ($choice) {
                '^1$' { Use-LatestCode; Pause-Menu }
                '^2$' { Build-AndUpload; Pause-Menu }
                '^3$' { Start-Homing; Pause-Menu }
                '^4$' { Start-Jmp; Pause-Menu }
                '^5$' { Run-FullWorkflow; Pause-Menu }
                '^6$' { Start-SmokeTest; Pause-Menu }
                '^7$' { Start-ControlTest; Pause-Menu }
                '^8$' { Select-CodeVersion; Pause-Menu }
                '^9$' { Select-ForkRemote; Pause-Menu }
                '^[Pp]$' { Select-SerialPort; Pause-Menu }
                '^[Uu]$' { Update-CurrentCode; Pause-Menu }
                '^[Ss]$' { Show-ProjectStatus; Pause-Menu }
                '^[Hh]$' { Show-HelpText; Pause-Menu }
                '^[Qq]$' { return }
                default { Write-Warning "Unknown choice: $choice"; Pause-Menu }
            }
        }
        catch {
            Write-Host ""
            Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
            Pause-Menu
        }
    }
}

Set-Location $repo

switch ($Command) {
    "menu"    { Show-Menu }
    "latest"  { Use-LatestCode }
    "full"    { Run-FullWorkflow }
    "build"   { Build-AndUpload }
    "home"    { Start-Homing }
    "jmp"     { Start-Jmp }
    "smoke"   { Start-SmokeTest }
    "control" { Start-ControlTest }
    "status"  { Show-ProjectStatus }
    "branch"  { Select-CodeVersion }
    "fork"    { Select-ForkRemote }
    "port"    { Select-SerialPort }
    "update"  { Update-CurrentCode }
    "help"    { Show-HelpText }
}
