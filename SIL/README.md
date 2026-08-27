# SIL: Software-In-the-Loop

I/O adapter between the Basilisk plant and [`FlightSoftware/`](../FlightSoftware/README.md). This folder is not the GNC: it packs sensors, optionally injects a SIL-only fault, calls `FlightSoftware::step` once per packet, and sends status / estimated MRP / RW / MTB back.

## Architecture

```text
Basilisk (Python bridge, TCP client)
        ↕  same socket :5557
C++ SIL server (accepts 1 connection)
  ← BootConfig (32 B)        once after connect (`boot_standby_s`)
  ← SensorPacket (56 B)      gyro / mag / CSS / SOC
  → CommandPacket (32 B)     status, estimated MRP, RW, MTB
  FaultInjector (optional)   before `step()`, env-gated, not in the protocol
```

- **Clock**: Basilisk `ClockSynch` with `accelFactor=10` in the default scenario (1 s wall ≈ 10 s sim).
- **Vizard**: DirectComm at `tcp://127.0.0.1:5556`.

TCP is duplex. After connect the plant sends one 32-byte `PACKET_BOOT_CONFIG`, then 56-byte sensor frames. The server replies with 32-byte commands. Plant truth (`sigma_BN`, `r_BN`, …) stays in Basilisk; it is **not** on `SensorPacket`.

## Telemetry (`sensor_packet.h`) — 56 bytes

Little-endian, 14 floats (`<14f` in Python).

| Field | Type | Unit / notes |
|---|---|---|
| `timestamp_s` | float | s |
| `gyro_x/y/z` | float | rad/s |
| `mag_x/y/z` | float | nT |
| `css_p/m{x,y,z}` | float | 6 orthogonal CSS |
| `batteryLevel` | float | SOC 0..1 |

SOC is integrated in Basilisk (`BatterySocModel`):
`SOC += (P_gen - P_load) / E_cap * dt`. `P_gen` uses CSS_Z and eclipse.
`P_load` uses the **FSW `ModeId`** from `PACKET_FSW_STATUS`, plus an optional `|τ|` term.

## Commands (`command_packets.h`) — 32 bytes

Shared header (`CommandHeader`, 16 bytes):

| Field | Type | Notes |
|---|---|---|
| `healthCheck` | uint32 | `0x53494C43` (`SILC`) |
| `version` | uint16 | `1` |
| `packet_type` | uint16 | `1` RW, `2` MTB, `3` FSW status, `4` boot config, `6` FSW attitude |
| `sequence` | uint32 | counter |
| `timestamp_s` | float | s |

Payload (12 bytes) + `crc32` (4 bytes):

| Type | Fields | Unit |
|---|---|---|
| `RWTorqueCommand` | `torque_Nm[3]` | N·m body X/Y/Z |
| `MTBDipoleCommand` | `dipole_Am2[3]` | A·m² body X/Y/Z |
| `FswStatusCommand` | `mode`, `flags`, `tmr_mismatch_count`, `gyro_bias_degph` | `ModeId` 0–3; flags: css=1, nadir=2, att=4, triad=8, tmr_mismatch=16, tmr_no_majority=32 |
| `BootConfigCommand` | `boot_standby_s` + 8 B pad | Plant → FSW, once after connect. Hold Standby until this sim time (0 = off) |
| `FswAttitudeCommand` | `sigma_BN[3]` | Estimated MRP from filtered `C_BN` (zeros if attitude invalid) |

`crc32` covers every byte of the packet **except** the last 4 (same algorithm as `crc32.h`).
The C++ loop sends status and estimated MRP **every** cycle, then RW/MTB if the command flags are set. The Python bridge does **not** print RW/MTB torques. Every 5 s of sim time it prints one `SIL TM` line: mode, geodesic **Att** (FSW MRP vs plant `sc.sigma_BN`), `|b|`, TMR count, validity flags, SOC. Mode changes still print immediately (`SIL FSW mode=`).

## Fault injection (SIL only)

`FaultInjector` runs in `sil_connection.cpp` **before** `FlightSoftware::step`. It is compiled into `sensor_receiver` (`ADCS_ENABLE_FAULT_INJECTION`). Off unless these environment variables are set (sim time `timestamp_s`, one shot each):

| Variable | Effect |
|---|---|
| `ADCS_INJECT_MODE_AT` | XOR `0x04` into TMR replica 0 of `ModeId` (e.g. Pointing `2` → `6`) |
| `ADCS_INJECT_SOC_AT` | Force `SensorPacket.batteryLevel = 0.10` (below the EPS Safe enter threshold) |

```powershell
$env:ADCS_INJECT_MODE_AT="120"
$env:ADCS_INJECT_SOC_AT="180"
.\SIL\cpp\build\Release\sensor_receiver.exe
```

Unset both for a clean run. Expected console:

- Mode inject while in Pointing: `INJECT ModeId replica0 2->6` then `FDIR TMR ModeId mismatch(repaired)` — **no** new `Mode=` line (TMR restored Pointing).
- SOC inject **after** boot Standby ends: `INJECT SOC=0.10` then `Mode=Safe` (EPS). During the 60 s default boot hold, the director stays in Standby and ignores Safe.

Not on the wire: no extra packet type. Telecommands come later.

## C++ files (`SIL/cpp`)

| File | Role |
|---|---|
| `sil_connection.cpp` | `main`, TCP server, SIL loop |
| `fault_injector.cpp` | Env-gated ModeId replica bit-flip and SOC drop |
| `sensor_receiver.cpp` | `receiveSensorPacket` — 56 B |
| `command_sender.cpp` | `sendRwTorque` / `sendMtbDipole` / `sendFswStatus` / `sendFswAttitude` |
| `boot_config_receiver.cpp` | `receiveBootConfig` — plant → FSW once after connect |
| `sensor_packet.h` / `command_packets.h` / `crc32.h` | Binary protocol |

## Build

```powershell
cmake -S SIL/cpp -B SIL/cpp/build
cmake --build SIL/cpp/build --config Release
```

## Startup order

1. Vizard DirectComm on `127.0.0.1:5556` (optional)
2. C++ server on `:5557`:

   ```powershell
   .\SIL\cpp\build\Release\sensor_receiver.exe
   ```

3. Basilisk:

   ```powershell
   .\.venv\Scripts\python.exe .\BasiliskSim\scenarios\basic_orbit_vizard.py
   ```

## Ports

| Port | Use |
|---|---|
| 5556 | Vizard DirectComm |
| 5557 | SIL duplex (telemetry + commands) |
