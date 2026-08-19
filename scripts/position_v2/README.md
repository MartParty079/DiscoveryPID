# Position V2 helper scripts

For normal use, start from the repository root with the interactive launcher:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force
.\DiscoveryPID.ps1
```

The launcher wraps these lower-level helpers and also provides branch/fork/port selection.

## Low-level commands

Build and upload:

```powershell
& ".\scripts\position_v2\build_upload.ps1" -Port COM7
```

Force a fresh home/calibration:

```powershell
& ".\scripts\position_v2\home.ps1" -Port COM7
```

Launch JMP with runtime checkout/Python/COM-port substitution:

```powershell
& ".\scripts\position_v2\open_jmp.ps1" -Port COM7
```

Run the smaller hardware smoke test:

```powershell
& ".\scripts\position_v2\smoke_test.ps1" -Port COM7
```

Run the full physical test:

```powershell
& ".\scripts\position_v2\control_test.ps1" -Port COM7
```

The scripts resolve the repository root from their own locations, so the checkout can live anywhere. `open_jmp.ps1` generates a temporary runtime JSL rather than modifying the tracked JSL, keeping collaborator-specific paths and ports out of Git.
