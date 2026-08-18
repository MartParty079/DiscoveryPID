# Position V2 helper scripts

Run these from PowerShell. The scripts resolve the repository root from their own location, so the checkout can live anywhere.

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force

& ".\scripts\position_v2\build_upload.ps1"
& ".\scripts\position_v2\home.ps1"
& ".\scripts\position_v2\open_jmp.ps1"
```

`home.ps1` forces a fresh `CALIBRATE`, waits for `EVENT HOMED`, and releases COM7 automatically.

`open_jmp.ps1` creates the session output directory and generates a temporary runtime JSL with checkout-specific Python/repository paths. This keeps the tracked JSL clean for collaboration.

Use the smaller 45-degree smoke test before a large-angle study if hardware has changed:

```powershell
& ".\scripts\position_v2\smoke_test.ps1"
```

The full physical test defaults to 300 degrees:

```powershell
& ".\scripts\position_v2\control_test.ps1"
```
