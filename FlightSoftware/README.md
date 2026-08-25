# Flight Software

This directory is the product: onboard ADCS, mode director, EPS energy policy, and an FDIR stub.

`SIL/cpp` calls `FlightSoftware::step` once per sensor packet. Basilisk is only the plant on the other side of that call.

## Cycle

```text
sensors → StateEstimator → SpacecraftState
        → EPS (SOC deadband) + FDIR (no-op)
        → ModeDirector::selectNextMode  (boot hold, then force_safe, then table below)
        → enter/exit if mode changed
        → active mode: error + PD + RW map
        → AttitudeCommand (torques + ModeId telemetry)
```

Modes never pick the next mode. `AttitudeCommand.active_mode` is what flew this cycle. `FlightSoftware` wires EPS and FDIR into director flags; it does not contain the transition table.

## Mode director

Implemented in `ModeDirector::selectNextMode` (`mode_director.cpp`). Rate thresholds live on the mode classes (`kEnterDetumbleRateRadps`, `kExitRateRadps`, `kExitHold_s`). SOC thresholds live on `EpsManager` (`kEnterSafeSoc` 0.25, `kExitSafeSoc` 0.35). Optional `bootStandbyDuration_s` (SIL `PACKET_BOOT_CONFIG`) holds Standby until that sim time — 60 s in the default demo, 0 in the FSW test scenarios.

The director does not read SOC. It sees `force_safe` (EPS low-SOC **or** FDIR) and `allow_exit_safe` (EPS recovered **and** FDIR not forcing Safe). FDIR currently always reports `force_safe = false`.

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

While `t < bootStandbyDuration_s`, the director stays in Standby (no Detumble, Pointing, or Safe — boot hold preempts EPS). Pointing does **not** re-enter Detumble on body rate — the PD law can exceed the Standby tumble threshold during normal acquisition; only **Standby** (and **Safe** on exit) use `‖ω‖ > 0.015` to enter Detumble.

| Mode | Law | Notes |
|---|---|---|
| Standby | Command zeros (clears residual RW/MTB) | Dispatcher |
| Detumble | PD, `Kp = 0`, `Kd` on `−ω` | `ratesSettled()` is 5 s under 0.005 rad/s |
| Pointing | 2-axis PD on sun or nadir error | Default target: nadir (`−Z` to Earth) |
| Safe | Same sun error as Pointing-sun (`SunReference`) | Energy supervisor; +Z panels |

`ref_ok` is `nadir_valid` or `css_valid` according to the pointing target. In eclipse, nadir can stay valid (filter coasts); sun-pointing does not.

## EPS and FDIR

`EpsManager` owns the SOC deadband (enter Safe below 0.25, leave above 0.35). The battery itself is still integrated in Basilisk; onboard EPS is policy on the telemetered SOC, not a second energy model. `modeLoadW` matches the plant's mode loads and is not applied to torque yet.

`FdirManager::evaluate` is a stub (`force_safe = false`). It runs every cycle so later detectors can preempt the director without changing the mode table.

## ADCS pipeline

`StateEstimator` turns raw gyro, mag, and 6 CSS into `SpacecraftState`:

| Block | Output |
|---|---|
| `CssWls` | Unit sun in body, `css_valid` |
| `OrbitPropagator` | Kepler `r_BN_N` → `nadir_N` / `nadir_B` |
| `SunModel` | Unit `sun_N` |
| `DipoleMagModel` | Unit `B_N` (IGRF-2020 centered dipole) |
| `Triad` | `C_triad` when sun and mag are usable |
| `AttitudeFilter` | `C_BN`, `gyro_bias`; lock held through eclipse |
| then | `ω = gyro − b̂` for control and the director |

Pointing errors: sun aligns body +Z with `ŝ_B`; nadir aligns body `−Z` with nadir. Both leave the boresight rotation unconstrained (`Kp_z = 0`).

## Layout

```text
types.h                    ModeId, SpacecraftState, AttitudeCommand
flight_software.h/.cpp     step, reset; wires estimator, EPS, FDIR, director
mode_director.h/.cpp       transition table (boot hold, force_safe, rates, ref)
eps/                       SOC Safe deadband
fdir/                      stub; can force Safe later
operation_mode.h           enter / update / exit
modes/                     Standby, Detumble, Pointing, Safe
estimation/                CSS, Kepler, sun/mag, TRIAD, filter
pointing/                  SunReference, NadirReference
control/                   PD
actuators/                 body torque → RW (reaction sign + saturate)
math/                      vec3, mat3, DCM / MRP
```
