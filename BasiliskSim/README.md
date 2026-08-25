# BasiliskSim

Dynamics environment for [`FlightSoftware/`](../FlightSoftware/README.md): LEO plant, sensors, demo battery, and Vizard. The ADCS and mode director are the C++ FSW, not this package.

## Vehicle and mission

Generic mini-satellite ADCS demo, 150 kg, LEO sun-synchronous orbit ~622 km,
revisit 1–2 times/day over a region of interest — not a CubeSat, not continuous
monitoring. Hub mass, inertia, and SSO inclination are set in
`scenarios/basic_orbit_vizard.py`.

### Battery / energy (demo)

- Panels on +Z: `P_gen ∝ max(0, CSS_Z) * eclipse_shadow` (umbra → generation ≈ 0).
- `P_load` from FSW `ModeId` over SIL (Detumble &gt; Pointing &gt; Standby &gt; Safe) plus optional `|τ|`.
- Capacity `E_cap ≈ 4e5 W·s` (~111 Wh) so SOC moves over 1–2 LEO orbits.
- FSW `EpsManager`: enter Safe if SOC &lt; 0.25; leave Safe if SOC &gt; 0.35 (level deadband, no time hold).
- Vizard GenericStorage bars: battery SOC and FSW mode from the status packet.
- `SensorPacket.batteryLevel` (108 B telemetry) feeds the estimator and onboard EPS.

## Requirements

Basilisk runs in a local `.venv` with Python 3.11.

```powershell
.\.venv\Scripts\Activate.ps1
```

From scratch:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r BasiliskSim\requirements.txt
```

The aerospace Basilisk package is `bsk`. Do not `pip install Basilisk` — that is a different project.

## Offline run (Vizard `.bin`, no SIL)

```powershell
.\.venv\Scripts\python.exe -c "import BasiliskSim.scenarios.basic_orbit_vizard as s; s.run(show_plots=True, log_sensors=True, real_time=False, show_live_stream=False, enable_sil_bridge=False)"
```

This creates spacecraft `ADCS_TestSat` (150 kg hub) in LEO SSO (~622 km, i ≈ 97.9°)
and propagates 120 minutes by default. The dynamics task is 60 Hz.

If Vizard support is present, this file is written (gitignored):

```text
BasiliskSim/data/_VizFiles/basic_orbit_vizard_UnityViz.bin
```

Open that `.bin` in Vizard to check visualization.

## SIL mode (C++ + optional live Vizard)

Default `basic_orbit_vizard.py` starts in SIL:

- `real_time=True` — `ClockSynch` with `accelFactor=10` (1 s wall ≈ 10 s sim)
- `show_live_stream=True` — Vizard DirectComm on `127.0.0.1:5556`
- `enable_sil_bridge=True` — telemetry every 0.1 s to `127.0.0.1:5557`
- `boot_standby_s=60` — FSW stays in Standby for 60 s of sim time (launcher separation). The six FSW test scripts leave this at 0.

### Startup order

1. Vizard DirectComm on `127.0.0.1:5556` (optional)
2. Build and run the C++ SIL server (see [`../SIL/README.md`](../SIL/README.md)):

   ```powershell
   cmake -S SIL/cpp -B SIL/cpp/build
   cmake --build SIL/cpp/build --config Release
   .\SIL\cpp\build\Release\sensor_receiver.exe
   ```

3. Simulation:

   ```powershell
   .\.venv\Scripts\python.exe .\BasiliskSim\scenarios\basic_orbit_vizard.py
   ```

The Python bridge uses **one socket** at `127.0.0.1:5557`: it sends telemetry and
receives RW/MTB commands plus FSW mode status.

Ports: `5556` Vizard, `5557` SIL bidirectional.

Bridge modules: `BasiliskSim/bridge/` (`sensor_socket_bridge.py`, `sil_protocol.py`, `battery_soc_model.py`).

## Reproducible FSW test scenarios

`basic_orbit_vizard.run()` accepts initial conditions and a scenario `name` (used for Vizard `.bin` and `sensor_data_{name}.png`). Each thin script below sets ICs and documents the expected mode chain in its docstring.

Same startup order as above (Vizard optional → `sensor_receiver` → scenario). `run_sil.ps1` still launches the default demo.

| Script | ICs | What it exercises |
|---|---|---|
| `scenarios/detumble_to_pointing.py` | ω ≈ (0.04, −0.02, 0.03), SOC 0.55, ~40 min | Standby → Detumble → Pointing (nadir) |
| `scenarios/quiet_to_pointing.py` | ω ≈ 0, SOC 0.55, ~20 min | Standby → Pointing (skip Detumble) |
| `scenarios/safe_entry.py` | SOC 0.22, moderate ω, ~30 min | Safe from t = 0; sun-point +Z until SOC > 0.35 |
| `scenarios/safe_from_pointing.py` | SOC 0.28, quiet, ~45 min | Pointing first; SOC drain → Safe preempts |
| `scenarios/high_tumble.py` | ω ≈ (0.12, −0.08, 0.10), SOC 0.55, ~40 min | Detumble with RW saturation (τ_max 0.2 N·m) |
| `scenarios/eclipse_coast.py` | SOC 0.55, quiet, f = 0° | Pointing through umbra; nadir coast when `css_valid` false |

FSW thresholds (for reading console `Mode=`): Detumble enter `‖ω‖ > 0.015` rad/s; exit / Pointing quiet `‖ω‖ < 0.005` for 5 s; Safe enter SOC < 0.25, exit SOC > 0.35.

```powershell
.\.venv\Scripts\python.exe .\BasiliskSim\scenarios\detumble_to_pointing.py
```

Replace the script name for other cases. Observe mode transitions on the C++ console, Vizard GenericStorage, and SOC bar — no automated mode assertions yet.

## What the scenario does

- Basilisk process + dynamics task.
- Spacecraft with ADCS sensors (CSS, IMU, magnetometer) and SOC.
- 3 reaction wheels and 3 magnetorquers on body X/Y/Z.
- Earth as central gravity.
- LEO SSO from classical orbital elements.
- Publishes SOC and FSW mode to Vizard (`GenericStorage`) and, if SIL is on, in `SensorPacket`.
- Optional Vizard export and/or SIL telemetry.
