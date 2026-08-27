# SIL: Software-In-the-Loop

I/O adapter between the Basilisk plant and [`FlightSoftware/`](../FlightSoftware/README.md). This folder is not the GNC: it packs sensors, optionally injects a SIL-only fault, calls `FlightSoftware::step` once per packet, and sends RW / MTB / `ModeId` back.

## Architecture

```text
Basilisk (Python bridge, TCP client)
        ↕  same socket :5557
C++ SIL server (accepts 1 connection)
  ← BootConfig (32 B)        once after connect (`boot_standby_s`)
  ← SensorPacket (108 B)     telemetry
  → CommandPacket (32 B)     RW / MTB / FSW status
  FaultInjector (optional)   before `step()`, env-gated, not in the protocol
```

- **Clock**: Basilisk `ClockSynch` with `accelFactor=10` in the default scenario (1 s wall ≈ 10 s sim).
- **Vizard**: DirectComm at `tcp://127.0.0.1:5556`.

TCP is duplex. After connect the plant sends one 32-byte `PACKET_BOOT_CONFIG`, then 108-byte sensor frames. The server replies with 32-byte commands.

## Telemetry (`sensor_packet.h`) — 108 bytes

Little-endian, 27 floats (`<27f` in Python).

| Field | Type | Unit / notes |
|---|---|---|
| `timestamp_s` | float | s |
| `gyro_x/y/z` | float | rad/s |
| `mag_x/y/z` | float | nT |
| `css_p/m{x,y,z}` | float | 6 orthogonal CSS |
| `batteryLevel` | float | SOC 0..1 |
| `eclipse_shadow` | float | 0..1 (1 = sun, 0 = umbra). Verification; FSW must not use it as a sensor |
| `r_BN_*` | float | m, Earth-centered. Verification |
| `sun_N_*`, `B_N_*` | float | unit inertial. Verification / compare |
| `sigma_BN_*` | float | MRP truth. Verification; TRIAD must not use it |

SOC is integrated in Basilisk (`BatterySocModel`):
`SOC += (P_gen - P_load) / E_cap * dt`. `P_gen` uses CSS_Z and eclipse.
`P_load` uses the **FSW `ModeId`** from `PACKET_FSW_STATUS`, plus an optional `|τ|` term.

## Commands (`command_packets.h`) — 32 bytes

Shared header (`CommandHeader`, 16 bytes):

| Field | Type | Notes |
|---|---|---|
| `healthCheck` | uint32 | `0x53494C43` (`SILC`) |
| `version` | uint16 | `1` |
| `packet_type` | uint16 | `1` RW, `2` MTB, `3` FSW status, `4` boot config |
| `sequence` | uint32 | counter |
| `timestamp_s` | float | s |

Payload (12 bytes) + `crc32` (4 bytes):

| Type | Fields | Unit |
|---|---|---|
| `RWTorqueCommand` | `torque_Nm[3]` | N·m body X/Y/Z |
| `MTBDipoleCommand` | `dipole_Am2[3]` | A·m² body X/Y/Z |
| `FswStatusCommand` | `mode` (uint8) + 11 B pad | `ModeId`: 0 Standby, 1 Detumble, 2 Pointing, 3 Safe |
| `BootConfigCommand` | `boot_standby_s` + 8 B pad | Plant → FSW, once after connect. Hold Standby until this sim time (0 = off) |

`crc32` covers every byte of the packet **except** the last 4 (same algorithm as `crc32.h`).
The C++ loop sends status **every** cycle, then RW/MTB if the command flags are set.

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
| `sensor_receiver.cpp` | `receiveSensorPacket` — 108 B |
| `command_sender.cpp` | `sendRwTorque` / `sendMtbDipole` / `sendFswStatus` |
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
