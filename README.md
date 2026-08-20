# Onboard ADCS Flight Software

C++17 flight software for a LEO spacecraft: **attitude determination**, **attitude control**, and an autonomous **mode director**.

[Basilisk](https://hanspeterschaub.info/basilisk/) is the dynamics environment used to exercise it. A thin software-in-the-loop (SIL) link carries sensors in and actuator commands out. The GNC logic lives in [`FlightSoftware/`](FlightSoftware/README.md) — it does **not** copy attitude from the simulator.

<img src="visualization_gif.gif" width="720" />

*Basilisk LEO plant → TCP SIL → `FlightSoftware::step` → RW torque back. Battery SOC and FSW mode come from the status packet, not from truth in the simulator.*

## Mode director

`FlightSoftware::selectNextMode` is the only place that changes mode. Each mode only computes torque. Safe is a supervisor: it can preempt any mode on low battery. The default demo also holds **Standby for 60 s of sim time** (launcher separation) via `run(boot_standby_s=60)`; the FSW test scenarios leave this at 0.

```mermaid
stateDiagram-v2
    [*] --> Standby: boot
    Standby --> Detumble: ||ω|| > 0.015 rad/s
    Standby --> Pointing: ||ω|| < 0.005 and ref valid
    Detumble --> Pointing: ||ω|| < 0.005 for 5 s and ref valid
    Detumble --> Standby: rates settled, ref invalid
    Pointing --> Standby: ref invalid
    Standby --> Safe: SOC < 0.25
    Detumble --> Safe: SOC < 0.25
    Pointing --> Safe: SOC < 0.25
    Safe --> Detumble: SOC > 0.35 and ||ω|| > 0.015
    Safe --> Standby: SOC > 0.35 and ||ω|| ≤ 0.015
```

| Mode | What it flies | Why it exists |
|---|---|---|
| **Standby** | Zero RW / MTB | Idle; dispatcher into Detumble or Pointing |
| **Detumble** | Rate damping (`Kp = 0`) | Kill tumble before pointing |
| **Pointing** | 2-axis PD, nadir (default) or sun | Mission attitude |
| **Safe** | Sun-point body +Z (`SunReference`) | Energy: panels toward the sun when SOC is low |

Battery uses a **level** deadband (enter &lt; 0.25, exit &gt; 0.35), not a time hold. Rate damping uses a 5 s dwell under 0.005 rad/s so Pointing does not start on a single quiet sample. The `‖ω‖ > 0.015` Detumble entry applies from **Standby** (and **Safe** on exit), not from Pointing — the pointing controller can legitimately exceed that rate during acquisition. Pointing/sun references must be valid (`nadir_valid` or `css_valid`) before leaving Detumble or Standby for Pointing; if the reference drops in Pointing, the director returns to Standby.

Each `step()`:

1. Estimate spacecraft state (onboard models + sensors).
2. Choose the next mode (optional boot Standby hold, then Safe, then the table above).
3. `exit` / `enter` if the mode changed.
4. Run **that** mode’s controller. `active_mode` is telemetry of the mode that flew, not a request.

## Attitude determination and control

```mermaid
flowchart TB
    CSS[6-face CSS] --> CssWls[Body sun heading]
    Gyro[Gyro] --> Filter
    Mag[Magnetometer] --> TRIAD
    CssWls --> TRIAD
    Kepler[Kepler orbit] --> Nadir
    Kepler --> Dipole[Centered dipole B_N]
    SunN[Sun model] --> TRIAD
    Dipole --> TRIAD
    TRIAD --> Filter[Complementary filter + gyro bias]
    Filter --> CBN[C_BN]
    CBN --> Nadir
    Filter --> Omega["ω = gyro − b̂"]
    CssWls --> Ref[Sun / nadir reference]
    Nadir --> Ref
    Omega --> Modes[Mode director + PD]
    Ref --> Modes
    SOC[Battery SOC] --> Modes
    Modes --> RW[Reaction-wheel torque]
```

Onboard chain (truth vectors in the SIL packet are logs only):

- CSS → unit sun in body
- Kepler two-body `r_BN` from mission elements → nadir
- Analytic sun direction and IGRF-2020 centered dipole in ECI
- TRIAD (sun + mag) seeds / corrects a complementary `C_BN` filter; bias `b̂` is estimated in sunlight and frozen in eclipse
- PD on attitude error + bias-corrected rate; RW mapper saturates and applies the reaction-torque sign

## Exercising the FSW

Basilisk is the LEO SSO plant (sensors, wheels, eclipse, demo battery). SIL is only the link: 108-byte sensors in, 32-byte commands out (RW, optional MTB, and `ModeId` every cycle).

```mermaid
flowchart LR
    subgraph env [Basilisk plant]
        Sensors[CSS / IMU / TAM]
        Plant[Spacecraft + RW + MTB]
        SOC[Battery SOC]
        Viz[Vizard]
    end
    subgraph sil [SIL]
        TCP[TCP :5557]
    end
    subgraph fsw [Flight software]
        Step["FlightSoftware::step"]
    end
    Sensors -->|SensorPacket 108 B| TCP --> Step
    Step -->|RW / MTB / ModeId| TCP
    TCP --> Plant
    TCP -->|ModeId| SOC
    SOC --> Viz
    Plant --> Viz
```

| Piece | Role |
|---|---|
| `FlightSoftware/` | ADCS + mode director (`step` once per packet) |
| `SIL/cpp/` | TCP server, packed protocol, CRC-32 |
| `BasiliskSim/` | Plant, SOC, Vizard, Python bridge |

Ports: **5556** Vizard DirectComm, **5557** SIL duplex.

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r BasiliskSim\requirements.txt
cmake -S SIL/cpp -B SIL/cpp/build
cmake --build SIL/cpp/build --config Release
.\SIL\cpp\build\Release\sensor_receiver.exe
.\.venv\Scripts\python.exe .\BasiliskSim\scenarios\basic_orbit_vizard.py
```

Or `.\run_sil.ps1` from the repo root (`VIZARD_EXE` optional). Details: [FlightSoftware](FlightSoftware/README.md), [SIL](SIL/README.md), [BasiliskSim](BasiliskSim/README.md).

**Reproducible FSW scenarios** — thin scripts under `BasiliskSim/scenarios/` with fixed initial conditions (rates, SOC, orbit phase) and docstrings describing the expected mode chain (Detumble, Pointing, Safe, eclipse coast). Same SIL startup; pick a case instead of the default demo, e.g. `.\.venv\Scripts\python.exe .\BasiliskSim\scenarios\detumble_to_pointing.py`. Full table and thresholds: [BasiliskSim/README.md](BasiliskSim/README.md#reproducible-fsw-test-scenarios).

## Layout

```text
FlightSoftware/     ADCS + mode director  ← this is the product
SIL/cpp/            I/O adapter (TCP, CRC, packed structs)
BasiliskSim/        Dynamics environment (plant, SOC, Vizard)
```

## Limitations (demo)

- Demo EPS (mode + `|τ|` load, small capacity so SOC moves in 1–2 orbits).
- Detumble on reaction wheels only; magnetorquers are in the plant but unused.
- Pointing is two-axis (no yaw / ground-track constraint).
- Orbit / sun / mag models match this scenario, not GPS/TLE or full-order IGRF.
- No unit tests; SIL can print estimator-vs-truth residuals.

## License

MIT — see [LICENSE](LICENSE).
