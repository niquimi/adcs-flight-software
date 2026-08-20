# Build the SIL C++ server, start it, then launch the Basilisk scenario.
# Optional: set VIZARD_EXE to your Vizard binary to open it automatically.
$ErrorActionPreference = 'Stop'

$Root = $PSScriptRoot
Set-Location $Root

Write-Host '==> Configuring SIL/cpp'
cmake -S SIL/cpp -B SIL/cpp/build

Write-Host '==> Building Release'
cmake --build SIL/cpp/build --config Release

$Sensor = Join-Path $Root 'SIL\cpp\build\Release\sensor_receiver.exe'
if (-not (Test-Path $Sensor)) {
    Write-Error "Build finished but executable not found: $Sensor"
}

Write-Host '==> Starting sensor_receiver.exe'
Start-Process cmd.exe -ArgumentList '/k', "cd /d `"$Root`" && .\SIL\cpp\build\Release\sensor_receiver.exe"

Start-Sleep -Seconds 2

$Python = Join-Path $Root '.venv\Scripts\python.exe'
if (-not (Test-Path $Python)) {
    Write-Error "Python venv not found at $Python. Create it first (see README)."
}

Write-Host '==> Opening CMD with basic_orbit_vizard.py'
Start-Process cmd.exe -ArgumentList '/k', "cd /d `"$Root`" && `"$Python`" .\BasiliskSim\scenarios\basic_orbit_vizard.py"

$Vizard = $env:VIZARD_EXE
if ([string]::IsNullOrWhiteSpace($Vizard)) {
    Write-Host 'Vizard skipped. Set VIZARD_EXE to launch Vizard automatically.'
} elseif (-not (Test-Path $Vizard)) {
    Write-Warning "VIZARD_EXE not found: $Vizard"
} else {
    Write-Host '==> Opening Vizard'
    Start-Process $Vizard -WorkingDirectory (Split-Path $Vizard)
}

Write-Host 'Done. sensor_receiver and Basilisk are running.'
