# SIL: Software-In-the-Loop

I/O adapter between the Basilisk plant and [`FlightSoftware/`](../FlightSoftware/README.md). This folder is not the GNC: it packs sensors, optionally injects a SIL-only fault, calls `FlightSoftware::step` once per packet, and sends status / estimated MRP / RW / MTB back. The CMake project here also builds `fsw_tests` (Health + director, no sockets).

## Architecture

```text
Basilisk (Python bridge, TCP client)
        ↕  socket :5557
C++ SIL server (accepts 1 plant connection)
  ← BootConfig (32 B)        once after connect (`boot_standby_s`)
  ← SensorPacket (56 B)      gyro / mag / CSS / SOC
  → CommandPacket (32 B)     status, estimated MRP, RW, MTB
  FaultInjector (optional)   before `step()`: env vars and/or `TC_INJECT_FAULT`

Operator TC console (optional, second TCP client)
        →  socket :5558
  → TelecommandPacket (32 B) on demand (not every sensor cycle)
  → FlightSoftware::applyTelecommand  (SIL keeps TC_INJECT_FAULT)
```

- **Clock**: Basilisk `ClockSynch` with `accelFactor=10` in the default scenario (1 s wall ≈ 10 s sim).
- **Vizard**: DirectComm at `tcp://127.0.0.1:5556`.

TCP on **:5557** is duplex for the plant only. After connect the bridge sends one 32-byte `PACKET_BOOT_CONFIG`, then 56-byte sensor frames. The server replies with 32-byte downlink commands. Plant truth (`sigma_BN`, `r_BN`, …) stays in Basilisk; it is **not** on `SensorPacket`.

**:5558** is a separate listener (`TelecommandLink`) for operator telecommands. Connect with [`tc_terminal.py`](tc_terminal.py) while the sim runs; each line sends one `PACKET_TELECOMMAND`. The SIL loop polls the TC socket after each sensor packet, logs `TC recv …`, routes `TC_INJECT_FAULT` to `FaultInjector`, and passes other opcodes to `FlightSoftware::applyTelecommand`.

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
| `packet_type` | uint16 | `1` RW, `2` MTB, `3` FSW status, `4` boot config, `5` TC, `6` FSW attitude |
| `sequence` | uint32 | counter |
| `timestamp_s` | float | s |

Payload (12 bytes) + `crc32` (4 bytes):

| Type | Fields | Unit |
|---|---|---|
| `RWTorqueCommand` | `torque_Nm[3]` | N·m body X/Y/Z |
| `MTBDipoleCommand` | `dipole_Am2[3]` | A·m² body X/Y/Z |
| `FswStatusCommand` | `mode`, `flags`, `tmr_mismatch_count`, `gyro_bias_degph`, `health_flags`, `last_tc_opcode`, `last_tc_arg0`, `health_event_count` | `ModeId` 0–3. **flags:** css=1, nadir=2, att=4, triad=8, tmr_mismatch=16, tmr_no_majority=32, **mode_forced=64**. **health_flags:** dt_back=1, dt_skip=2, gyro_oor=4, mag_oor=8, css_range=16, css_incoh=32, att_stale=64. **last_tc_***: ACK of the last opcode that reached `applyTelecommand` (`TC_INJECT_FAULT` is SIL-only and is not ACKed). Packet still 32 B (former `pad[4]`). |
| `BootConfigCommand` | `boot_standby_s` + 8 B pad | Plant → FSW, once after connect. Hold Standby until this sim time (0 = off) |
| `FswAttitudeCommand` | `sigma_BN[3]` | Estimated MRP from filtered `C_BN` (zeros if attitude invalid) |
| `TelecommandPacket` | `opcode`, `arg0`, `arg1` | Operator uplink on **:5558** only (`TelecommandOpcode` 0–9) |

`crc32` covers every byte of the packet **except** the last 4 (same algorithm as `crc32.h`).
The C++ loop sends status and estimated MRP **every** cycle, then RW/MTB if the command flags are set. The Python bridge does **not** print RW/MTB torques. Every 5 s of sim time it prints one `SIL TM` line: mode, geodesic **Att** (FSW MRP vs plant `sc.sigma_BN`), `|b|`, TMR count, validity flags, SOC, **health=**, **ack=**, **nH=**. Mode changes still print immediately (`SIL FSW mode=`). After `force 2` on `:5558`, expect `ack=1,2` on the next TM line.

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

## Telecommand console

Opcodes live in `command_packets.h`. Dispatch is onboard except `TC_INJECT_FAULT` (SIL `FaultInjector`). Effects: [FlightSoftware README](../FlightSoftware/README.md#operator-telecommands).

```powershell
.\.venv\Scripts\python.exe .\SIL\tc_terminal.py
# tc> force 2
# tc> unforce
# tc> inject 1
```

Expected on the C++ console after the next sensor cycle: `TC recv opcode=…` then a `Mode=` line if ForceMode changed the mode.

## C++ files (`SIL/cpp`)

| File | Role |
|---|---|
| `sil_connection.cpp` | `main`, TCP server, SIL loop, TC poll |
| `telecommand_link.cpp` | Second port `:5558`, non-blocking TC recv |
| `fault_injector.cpp` | Env-gated ModeId replica bit-flip and SOC drop |
| `sensor_receiver.cpp` | `receiveSensorPacket` — 56 B |
| `command_sender.cpp` | `sendRwTorque` / `sendMtbDipole` / `sendFswStatus` / `sendFswAttitude` |
| `boot_config_receiver.cpp` | `receiveBootConfig` — plant → FSW once after connect |
| `sensor_packet.h` / `command_packets.h` / `crc32.h` | Binary protocol |

## Build

```powershell
cmake -S SIL/cpp -B SIL/cpp/build
cmake --build SIL/cpp/build --config Release
ctest --test-dir SIL/cpp/build -C Release --output-on-failure
```

`fsw_tests` does not need sockets or Basilisk. Visual Studio is multi-config: pass `-C Release` (or `Debug`) to `ctest`.

## Startup order

1. Vizard DirectComm on `127.0.0.1:5556` (optional)
2. C++ server (`:5557` plant + `:5558` TC):

   ```powershell
   .\SIL\cpp\build\Release\sensor_receiver.exe
   ```

3. Optional TC console (while sim is running or waiting for Basilisk):

   ```powershell
   .\.venv\Scripts\python.exe .\SIL\tc_terminal.py
   ```

4. Basilisk:

   ```powershell
   .\.venv\Scripts\python.exe .\BasiliskSim\scenarios\basic_orbit_vizard.py
   ```

## Ports

| Port | Use |
|---|---|
| 5556 | Vizard DirectComm |
| 5557 | SIL plant duplex (sensors in, status/RW/MTB out) |
| 5558 | Operator telecommands (`tc_terminal.py` → `applyTelecommand`) |
