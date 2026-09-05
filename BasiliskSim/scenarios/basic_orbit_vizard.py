from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    from Basilisk.architecture import messaging
    from Basilisk.simulation import (
        coarseSunSensor,
        eclipse,
        imuSensor,
        magneticFieldCenteredDipole,
        magnetometer,
        MtbEffector,
        reactionWheelStateEffector,
        simSynch,
        spacecraft,
        vizInterface,
    )
    from Basilisk.utilities import (
        SimulationBaseClass,
        macros,
        orbitalMotion,
        simHelpers,
        simIncludeGravBody,
        simIncludeRW,
        simSetPlanetEnvironment,
        vizSupport,
    )
    from BasiliskSim.bridge.battery_soc_model import BatterySocModel
    from BasiliskSim.bridge.sensor_socket_bridge import SensorSocketBridge
except ImportError as exc:
    raise SystemExit(
        "Basilisk is not installed in this Python environment. "
        "Install it first, then run this script again."
    ) from exc


VIZARD_FRAMES_PER_SECOND = 60
SIM_RATE_SECONDS = 1.0 / VIZARD_FRAMES_PER_SECOND
SIM_DURATION_MINUTES = 120.0
# Wall clock: sim_time / accel. Eclipse ~t=650 s → ~65 s wall at 10x (was ~5 min at 2x).
STREAM_ACCELERATION_FACTOR = 2.0
# Known body-frame gyro bias for AttitudeFilter SIL. 0.01 °/s = 36 °/h.
# Unestimated → ~21° walk in a 35 min eclipse; ki should lock |b| to ~36 °/h in sun.
IMU_GYRO_BIAS_DEGPS = [0.01, 0.0, 0.0]
# White noise only. TAM default A=I turns senNoiseStd into an unbounded walk.
TAM_NOISE_STD_T = 50e-9  # 50 nT ≈ 0.1° on a ~30 µT LEO field
# CSS setAMatrix is not usable from Python (A stays I → walk). White noise is
# added on the SIL packet; Basilisk CSS stays clean for Vizard/SOC.
CSS_NOISE_STD = 0.02  # on 0–2 scale; skipped when all diodes are dark (eclipse)
SPICE_EPOCH = "2021 MAY 04 07:47:48.965 (UTC)"
SENSOR_LOG_INTERVAL_SECONDS = 60.0
SIL_TELEMETRY_HOST = "127.0.0.1"
SIL_TELEMETRY_PORT = 5557
SIL_TELEMETRY_INTERVAL_SECONDS = 0.1
MTB_MAX_DIPOLE_AM2 = 10.0
# Demo: start in the Safe deadband (enter < 0.25, exit > 0.35) so Detumble/Pointing
# can run before SOC drops into Safe.
DEFAULT_INITIAL_SOC = 0.3
DEFAULT_OMEGA_BN_B = (0.01, -0.005, 0.02)
DEFAULT_SIGMA_BN = (0.0, 0.0, 0.0)
DEFAULT_TRUE_ANOMALY_DEG = 80.0
DEFAULT_SCENARIO_NAME = "basic_orbit_vizard"


def print_sensor_snapshots(
    time_s: np.ndarray,
    css_logs: list,
    css_names: list[str],
    imu_log,
    tam_log,
    eclipse_log,
    battery_log=None,
) -> None:
    """Print sensor readings to the terminal at each logged time step."""
    print("\n--- Sensor log (every {:.0f} s) ---".format(SENSOR_LOG_INTERVAL_SECONDS))
    for idx, t in enumerate(time_s):
        css_values = [css_logs[i].OutputData[idx] for i in range(len(css_logs))]
        gyro = imu_log.AngVelPlatform[idx]
        mag_nt = tam_log.tam_S[idx] * 1e9
        shadow = eclipse_log.shadowFactor[idx]
        css_text = ", ".join(f"{name}={value:.3f}" for name, value in zip(css_names, css_values))
        soc_text = ""
        if battery_log is not None:
            capacity = max(battery_log.storageCapacity[idx], 1e-9)
            soc = battery_log.storageLevel[idx] / capacity
            soc_text = f" | SOC={soc:.3f}"
        print(
            f"t={t:7.1f}s | eclipse={shadow:.3f}{soc_text} | CSS [{css_text}] | "
            f"gyro(rad/s)=[{gyro[0]:+.4f}, {gyro[1]:+.4f}, {gyro[2]:+.4f}] | "
            f"TAM(nT)=[{mag_nt[0]:+.1f}, {mag_nt[1]:+.1f}, {mag_nt[2]:+.1f}]"
        )


def plot_sensor_data(
    time_s: np.ndarray,
    css_logs: list,
    css_names: list[str],
    imu_log,
    tam_log,
    eclipse_log,
    output_path: Path,
    show_plots: bool,
    rw_log=None,
    num_rw: int = 0,
    rw_labels: list[str] | None = None,
    battery_log=None,
    mode_log=None,  # reserved for future mode overlay
) -> None:
    """Plot logged sensor data and optionally save to disk."""
    extra = int(rw_log is not None) + int(battery_log is not None)
    n_rows = 4 + extra
    fig, axes = plt.subplots(n_rows, 1, figsize=(11, 2.5 * n_rows), sharex=True)
    row = 0

    for css_idx, name in enumerate(css_names):
        axes[row].plot(
            time_s,
            css_logs[css_idx].OutputData,
            label=name,
            color=simHelpers.getLineColor(css_idx, len(css_names)),
        )
    axes[row].set_ylabel("CSS signal")
    axes[row].set_title("Coarse sun sensors (0 = eclipse/out of FOV, up to ~2 in sun)")
    axes[row].legend(loc="upper right")
    axes[row].grid(True, alpha=0.3)
    row += 1

    gyro = imu_log.AngVelPlatform
    for axis_idx in range(3):
        axes[row].plot(
            time_s,
            gyro[:, axis_idx],
            label=f"ω{axis_idx}",
            color=simHelpers.getLineColor(axis_idx, 3),
        )
    axes[row].set_ylabel("Angular rate (rad/s)")
    axes[row].set_title("Gyroscope (IMU)")
    axes[row].legend(loc="upper right")
    axes[row].grid(True, alpha=0.3)
    row += 1

    mag_nt = tam_log.tam_S * 1e9
    for axis_idx in range(3):
        axes[row].plot(
            time_s,
            mag_nt[:, axis_idx],
            label=f"B{axis_idx}",
            color=simHelpers.getLineColor(axis_idx, 3),
        )
    axes[row].set_ylabel("Field (nT)")
    axes[row].set_title("Magnetometer (TAM)")
    axes[row].legend(loc="upper right")
    axes[row].grid(True, alpha=0.3)
    row += 1

    axes[row].plot(time_s, eclipse_log.shadowFactor, color="#aa0000", label="shadowFactor")
    axes[row].set_ylabel("Shadow factor")
    axes[row].set_title("Eclipse (1 = full sun, 0 = umbra)")
    axes[row].set_ylim(-0.05, 1.05)
    axes[row].legend(loc="upper right")
    axes[row].grid(True, alpha=0.3)
    row += 1

    if rw_log is not None:
        rw_speed_rpm = rw_log.wheelSpeeds[:, :num_rw] / macros.RPM
        labels = rw_labels or [f"RW{idx}" for idx in range(num_rw)]
        for rw_idx in range(num_rw):
            axes[row].plot(
                time_s,
                rw_speed_rpm[:, rw_idx],
                label=labels[rw_idx],
                color=simHelpers.getLineColor(rw_idx, num_rw),
            )
        axes[row].set_ylabel("Speed (RPM)")
        axes[row].set_title("Reaction wheel")
        axes[row].legend(loc="upper right")
        axes[row].grid(True, alpha=0.3)
        row += 1

    if battery_log is not None:
        capacity = np.maximum(battery_log.storageCapacity, 1e-9)
        soc = battery_log.storageLevel / capacity
        axes[row].plot(time_s, soc, color="#007744", label="SOC")
        axes[row].axhline(0.25, color="#cc6600", linestyle="--", linewidth=1, label="enter Safe")
        axes[row].axhline(0.35, color="#2266aa", linestyle="--", linewidth=1, label="exit Safe")
        axes[row].set_ylabel("SOC")
        axes[row].set_ylim(-0.05, 1.05)
        axes[row].set_title("Battery SOC (CSS +Z panels) and Safe thresholds")
        axes[row].legend(loc="upper right")
        axes[row].grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Sensor plot saved: {output_path}")
    if show_plots:
        plt.show()
    plt.close(fig)


def run(
    show_live_stream: bool = False,
    broadcast_stream: bool = False,
    real_time: bool = False,
    enable_sil_bridge: bool = False,
    log_sensors: bool = False,
    show_plots: bool = False,
    initial_soc: float = DEFAULT_INITIAL_SOC,
    duration_min: float = SIM_DURATION_MINUTES,
    omega_BN_B: tuple[float, float, float] = DEFAULT_OMEGA_BN_B,
    sigma_BN: tuple[float, float, float] = DEFAULT_SIGMA_BN,
    true_anomaly_deg: float = DEFAULT_TRUE_ANOMALY_DEG,
    name: str = DEFAULT_SCENARIO_NAME,
    boot_standby_s: float = 0.0,
) -> None:
    """Run an orbital simulation with ADCS sensors and optional Vizard live stream."""
    output_dir = PROJECT_ROOT / "BasiliskSim" / "data"
    output_dir.mkdir(parents=True, exist_ok=True)
    viz_file = output_dir / name
    viz_output_file = output_dir / "_VizFiles" / f"{name}_UnityViz.bin"

    print(
        f"Scenario '{name}': duration={duration_min:.0f} min, "
        f"SOC={initial_soc:.2f}, omega={omega_BN_B}, f={true_anomaly_deg:.1f} deg"
        + (f", boot Standby {boot_standby_s:.0f} s" if boot_standby_s > 0.0 else "")
    )

    sim = SimulationBaseClass.SimBaseClass()
    process_name = "dynamicsProcess"
    task_name = "dynamicsTask"
    sim_process = sim.CreateNewProcess(process_name)
    sim_process.addTask(sim.CreateNewTask(task_name, macros.sec2nano(SIM_RATE_SECONDS)))

    sat = spacecraft.Spacecraft()
    sat.ModelTag = "ADCS_TestSat"
    # Demo mini-sat (~150 kg): hub mass/inertia about Bc (critical for attitude dynamics).
    sat.hub.mHub = 150.0  # kg
    sat.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    sat.hub.IHubPntBc_B = [
        [20.0, 0.0, 0.0],
        [0.0, 18.0, 0.0],
        [0.0, 0.0, 15.0],
    ]

    gravity_factory = simIncludeGravBody.gravBodyFactory()
    earth = gravity_factory.createEarth()
    earth.isCentralBody = True
    gravity_factory.createSun()
    gravity_factory.addBodiesTo(sat)

    spice = gravity_factory.createSpiceInterface(time=SPICE_EPOCH)
    sim.AddModelToTask(task_name, spice, -1)

    earth_msg = spice.planetStateOutMsgs[0]
    sun_msg = spice.planetStateOutMsgs[1]

    # LEO SSO ~622 km altitude (a = 7000 km); RAAN not tuned for a specific LTAN yet.
    orbit = orbitalMotion.ClassicElements()
    orbit.a = 7000.0 * 1000.0
    orbit.e = 0.001
    orbit.i = 97.9 * macros.D2R  # sun-synchronous for ~622 km
    orbit.Omega = 30.0 * macros.D2R
    orbit.omega = 10.0 * macros.D2R
    orbit.f = true_anomaly_deg * macros.D2R

    position, velocity = orbitalMotion.elem2rv(earth.mu, orbit)
    sat.hub.r_CN_NInit = position
    sat.hub.v_CN_NInit = velocity
    sat.hub.sigma_BNInit = [[sigma_BN[0]], [sigma_BN[1]], [sigma_BN[2]]]
    sat.hub.omega_BN_BInit = [[omega_BN_B[0]], [omega_BN_B[1]], [omega_BN_B[2]]]

    rw_factory = simIncludeRW.rwFactory()
    for label, axis in [
        ("RW_X", [1.0, 0.0, 0.0]),
        ("RW_Y", [0.0, 1.0, 0.0]),
        ("RW_Z", [0.0, 0.0, 1.0]),
    ]:
        rw_factory.create(
            "Honeywell_HR16",
            axis,
            maxMomentum=50.0,
            Omega=0.0,
            label=label,
        )
    rw_effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rw_factory.addToSpacecraft("ReactionWheels", rw_effector, sat)
    num_rw = rw_factory.getNumOfDevices()
    rw_labels = list(rw_factory.rwList.keys())

    rw_cmd_data = messaging.ArrayMotorTorqueMsgPayload()
    rw_cmd_data.motorTorque = [0.0] * num_rw
    rw_cmd_msg = messaging.ArrayMotorTorqueMsg().write(rw_cmd_data, 0)
    rw_effector.rwMotorCmdInMsg.subscribeTo(rw_cmd_msg)

    # Three magnetorquers aligned with body X/Y/Z (same axes as the RWs).
    num_mtb = 3
    mtb_config = messaging.MTBArrayConfigMsgPayload()
    mtb_config.numMTB = num_mtb
    gt_matrix = [0.0] * 108
    for mtb_idx, axis_idx in enumerate(range(3)):
        # GtMatrix_B is 3×36 row-major: row * 36 + col
        gt_matrix[axis_idx * 36 + mtb_idx] = 1.0
    mtb_config.GtMatrix_B = gt_matrix
    mtb_config.maxMtbDipoles = [MTB_MAX_DIPOLE_AM2] * num_mtb + [0.0] * (36 - num_mtb)
    mtb_config_msg = messaging.MTBArrayConfigMsg().write(mtb_config)

    mtb_cmd_data = messaging.MTBCmdMsgPayload()
    mtb_cmd_data.mtbDipoleCmds = [0.0] * 36
    mtb_cmd_msg = messaging.MTBCmdMsg().write(mtb_cmd_data, 0)

    mtb_effector = MtbEffector.MtbEffector()
    mtb_effector.ModelTag = "Magnetorquers"
    mtb_effector.mtbCmdInMsg.subscribeTo(mtb_cmd_msg)
    mtb_effector.mtbParamsInMsg.subscribeTo(mtb_config_msg)
    # magInMsg is wired after mag_field is created below.

    sim.AddModelToTask(task_name, rw_effector, 2)
    sim.AddModelToTask(task_name, sat)

    eclipse_obj = eclipse.Eclipse()
    eclipse_obj.addSpacecraftToModel(sat.scStateOutMsg)
    eclipse_obj.addPlanetToModel(earth_msg)
    eclipse_obj.sunInMsg.subscribeTo(sun_msg)
    sim.AddModelToTask(task_name, eclipse_obj)

    mag_field = magneticFieldCenteredDipole.MagneticFieldCenteredDipole()
    mag_field.ModelTag = "EarthMagField"
    simSetPlanetEnvironment.centeredDipoleMagField(mag_field, "earth")
    mag_field.addSpacecraftToModel(sat.scStateOutMsg)
    mag_field.planetPosInMsg.subscribeTo(earth_msg)
    sim.AddModelToTask(task_name, mag_field)

    mtb_effector.magInMsg.subscribeTo(mag_field.envOutMsgs[0])
    sat.addDynamicEffector(mtb_effector)
    sim.AddModelToTask(task_name, mtb_effector, 1)

    tam = magnetometer.Magnetometer()
    tam.ModelTag = "TAM"
    tam.scaleFactor = 1.0
    # A=0 → e[k]=w[k] (white). Default A=I is a random walk, not white noise.
    tam.setAMatrix(np.zeros((3, 3)))
    tam.senNoiseStd = [TAM_NOISE_STD_T] * 3
    tam.walkBounds = [0.0, 0.0, 0.0]
    tam.stateInMsg.subscribeTo(sat.scStateOutMsg)
    tam.magInMsg.subscribeTo(mag_field.envOutMsgs[0])
    sim.AddModelToTask(task_name, tam)

    imu = imuSensor.ImuSensor()
    imu.ModelTag = "Gyro"
    imu.senRotBias = [b * macros.D2R for b in IMU_GYRO_BIAS_DEGPS]
    imu.PMatrixGyro = np.zeros((3, 3))
    imu.setWalkBoundsGyro([0.0, 0.0, 0.0])
    imu.scStateInMsg.subscribeTo(sat.scStateOutMsg)
    sim.AddModelToTask(task_name, imu)
    gyro_bias_degph = float(np.linalg.norm(IMU_GYRO_BIAS_DEGPS)) * 3600.0
    print(
        f"IMU gyro bias (known): {IMU_GYRO_BIAS_DEGPS} deg/s "
        f"(|b|={gyro_bias_degph:.1f} deg/h). "
        f"AttitudeFilter should converge to this in sun and freeze in eclipse."
    )
    print(
        f"TAM white noise: {TAM_NOISE_STD_T * 1e9:.0f} nT (A=0). "
        f"CSS white noise (SIL packet): {CSS_NOISE_STD} (0–2 scale)."
    )

    def make_css(model_tag: str, n_hat_b: list[float], css_group_id: int = 0) -> coarseSunSensor.CoarseSunSensor:
        css = coarseSunSensor.CoarseSunSensor()
        css.ModelTag = model_tag
        # 90° FOV: each face covers a hemisphere; ±axes reconstruct a signed sun vector.
        # (80° + only +X/+Y/+Z clipped the sun into the positive octant and broke TRIAD.)
        css.fov = 90.0 * macros.D2R
        css.scaleFactor = 2.0
        css.kellyFactor = 0.0
        css.senNoiseStd = 0.0  # white CSS noise is applied in the SIL bridge
        css.walkBounds = -1.0
        # Constructor default is phi=45°. Reset does not rebuild nHat, but
        # setUnitDirectionVectorWithPerturbation does — keep angles consistent.
        css.phi = float(np.arctan2(n_hat_b[2], np.hypot(n_hat_b[0], n_hat_b[1])))
        css.theta = float(np.arctan2(n_hat_b[1], n_hat_b[0]))
        css.setUnitDirectionVectorWithPerturbation(0.0, 0.0)
        css.nHat_B = n_hat_b
        css.CSSGroupID = css_group_id
        css.sunInMsg.subscribeTo(sun_msg)
        css.stateInMsg.subscribeTo(sat.scStateOutMsg)
        css.sunEclipseInMsg.subscribeTo(eclipse_obj.eclipseOutMsgs[0])
        sim.AddModelToTask(task_name, css)
        return css

    # Six body-face CSS; FSW CssWls builds s_B from the six raw diodes.
    css_list = [
        make_css("CSS_PX", [1.0, 0.0, 0.0]),
        make_css("CSS_MX", [-1.0, 0.0, 0.0]),
        make_css("CSS_PY", [0.0, 1.0, 0.0]),
        make_css("CSS_MY", [0.0, -1.0, 0.0]),
        make_css("CSS_PZ", [0.0, 0.0, 1.0]),
        make_css("CSS_MZ", [0.0, 0.0, -1.0]),
    ]
    css_names = [css.ModelTag for css in css_list]

    # Battery SOC: P_gen from +Z panel + eclipse, P_load from FSW ModeId over SIL.
    battery = BatterySocModel(
        css_z=css_list[4],
        eclipse_obj=eclipse_obj,
        initial_soc=initial_soc,
    )
    sim.AddModelToTask(task_name, battery, -1)
    print(
        f"Battery SOC model: initial={initial_soc:.2f}, "
        f"E_cap={battery.e_cap_ws:.0f} W*s, P_gen_max={battery.p_gen_max_w:.0f} W"
    )

    sensor_log_interval_ns = macros.sec2nano(SENSOR_LOG_INTERVAL_SECONDS)
    simulation_time_ns = macros.min2nano(duration_min)
    tam_log = tam.tamDataOutMsg.recorder(sensor_log_interval_ns)
    imu_log = imu.sensorOutMsg.recorder(sensor_log_interval_ns)
    eclipse_log = eclipse_obj.eclipseOutMsgs[0].recorder(sensor_log_interval_ns)
    css_logs = [css.cssDataOutMsg.recorder(sensor_log_interval_ns) for css in css_list]
    rw_log = rw_effector.rwSpeedOutMsg.recorder(sensor_log_interval_ns)
    battery_log = battery.battery_msg.recorder(sensor_log_interval_ns)
    mode_log = battery.mode_msg.recorder(sensor_log_interval_ns)
    for logger in [tam_log, imu_log, eclipse_log, rw_log, battery_log, mode_log, *css_logs]:
        sim.AddModelToTask(task_name, logger)

    if enable_sil_bridge:
        sil_bridge = SensorSocketBridge(
            tam=tam,
            imu=imu,
            css_list=css_list,
            eclipse_obj=eclipse_obj,
            rw_cmd_msg=rw_cmd_msg,
            rw_cmd_data=rw_cmd_data,
            mtb_cmd_msg=mtb_cmd_msg,
            mtb_cmd_data=mtb_cmd_data,
            num_rw=num_rw,
            num_mtb=num_mtb,
            battery=battery,
            sc_state_msg=sat.scStateOutMsg,
            earth_msg=earth_msg,
            sun_msg=sun_msg,
            mag_env_msg=mag_field.envOutMsgs[0],
            mag_field=mag_field,
            host=SIL_TELEMETRY_HOST,
            port=SIL_TELEMETRY_PORT,
            interval_s=SIL_TELEMETRY_INTERVAL_SECONDS,
            css_noise_std=CSS_NOISE_STD,
            boot_standby_s=boot_standby_s,
        )
        sim.AddModelToTask(task_name, sil_bridge, -1)
        print(
            f"SIL bridge bidirectional on {SIL_TELEMETRY_HOST}:{SIL_TELEMETRY_PORT} "
            f"(telemetry every {SIL_TELEMETRY_INTERVAL_SECONDS:.1f} s)"
        )

    generic_sensors = []
    for label, normal, position in [
        ("Gyro", [0.0, 1.0, 0.0], [0.0, 0.3, 0.0]),
        ("TAM", [0.0, 0.0, 1.0], [0.0, 0.0, 0.3]),
        ("CSS_Z", [0.0, 0.0, 1.0], [0.0, 0.0, 0.45]),
    ]:
        sensor_viz = vizInterface.GenericSensor()
        sensor_viz.label = label
        sensor_viz.r_SB_B = position
        sensor_viz.normalVector = normal
        sensor_viz.fieldOfView.push_back(25.0 * macros.D2R)
        sensor_viz.size = 0.15
        color = "yellow" if label == "CSS_Z" else "cyan"
        sensor_viz.color = vizInterface.IntVector(vizSupport.toRGBA255(color))
        generic_sensors.append(sensor_viz)

    # Vizard storage panels: battery SOC + FSW mode id from SIL status.
    # IMPORTANT: assign a MsgReader — calling .subscribeTo on the property does not stick (SWIG copy).
    battery_storage = vizInterface.GenericStorage()
    battery_storage.label = "Battery SOC"
    battery_storage.type = "Battery"
    battery_storage.units = "%"
    battery_storage.thresholds = vizInterface.IntVector([25, 35])
    battery_storage.color = vizInterface.IntVector(
        vizSupport.toRGBA255("red")
        + vizSupport.toRGBA255("orange")
        + vizSupport.toRGBA255("green")
    )
    battery_reader = messaging.PowerStorageStatusMsgReader()
    battery_reader.subscribeTo(battery.battery_msg)
    battery_storage.batteryStateInMsg = battery_reader

    mode_storage = vizInterface.GenericStorage()
    mode_storage.label = "FSW Mode"
    mode_storage.type = "Operating Mode"
    mode_storage.units = "1=Stby 2=Det 3=Pnt 4=Safe"
    mode_storage.thresholds = vizInterface.IntVector([25, 50, 75])
    mode_storage.color = vizInterface.IntVector(
        vizSupport.toRGBA255("green")
        + vizSupport.toRGBA255("yellow")
        + vizSupport.toRGBA255("blue")
        + vizSupport.toRGBA255("red")
    )
    mode_reader = messaging.PowerStorageStatusMsgReader()
    mode_reader.subscribeTo(battery.mode_msg)
    mode_storage.batteryStateInMsg = mode_reader

    css_z_storage = vizInterface.GenericStorage()
    css_z_storage.label = "CSS_Z signal"
    css_z_storage.type = "Coarse Sun Sensor"
    css_z_storage.currentValue = 0.0
    css_z_storage.maxValue = css_list[4].scaleFactor
    css_z_storage.units = "0-2"
    css_z_storage.thresholds = vizInterface.IntVector([10, 50])
    css_z_storage.color = vizInterface.IntVector(
        vizSupport.toRGBA255("gray")
        + vizSupport.toRGBA255("orange")
        + vizSupport.toRGBA255("yellow")
    )
    css_z_reader = messaging.PowerStorageStatusMsgReader()
    css_z_reader.subscribeTo(battery.css_z_msg)
    css_z_storage.batteryStateInMsg = css_z_reader

    # Keep Python wrappers alive for the whole run (vizInterface holds C++ pointers).
    generic_storage = [battery_storage, mode_storage, css_z_storage]
    battery_storage.this.disown()
    mode_storage.this.disown()
    css_z_storage.this.disown()

    if real_time or show_live_stream or broadcast_stream:
        clock_sync = simSynch.ClockSynch()
        clock_sync.ModelTag = "clockSync"
        clock_sync.accelFactor = STREAM_ACCELERATION_FACTOR
        clock_sync.displayTime = False
        sim.AddModelToTask(task_name, clock_sync)
        print(
            f"ClockSynch accelFactor={STREAM_ACCELERATION_FACTOR:.1f} "
            f"(1 s wall ≈ {STREAM_ACCELERATION_FACTOR:.0f} s sim)"
        )

    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(
            sim,
            task_name,
            sat,
            saveFile=str(viz_file),
            liveStream=show_live_stream,
            broadcastStream=broadcast_stream,
            cssList=[css_list],
            genericSensorList=[generic_sensors],
            genericStorageList=[generic_storage],
            rwEffectorList=[rw_effector],
        )
        vizSupport.setInstrumentGuiSetting(
            viz,
            spacecraftName=sat.ModelTag,
            viewCSSPanel=True,
            viewCSSCoverage=True,
            viewCSSBoresight=True,
            showCSSLabels=True,
            showGenericSensorLabels=True,
            showGenericStoragePanel=1,
        )
        vizSupport.setActuatorGuiSetting(
            viz,
            spacecraftName=sat.ModelTag,
            viewRWPanel=True,
            viewRWHUD=True,
            showRWLabels=True,
        )
        viz.reqComAddress = "127.0.0.1"
        viz.pubComAddress = "127.0.0.1"
        print(f"Vizard output: {viz_output_file}")
        if show_live_stream:
            print("DirectComm socket: tcp://127.0.0.1:5556")
        if broadcast_stream:
            print("Receive Only socket: tcp://127.0.0.1:5570")
    else:
        print("Vizard support was not found. The simulation will still run.")

    sim.InitializeSimulation()
    sim.ConfigureStopTime(simulation_time_ns)
    sim.ExecuteSimulation()
    print(f"Simulation complete. Final SOC={battery.soc:.3f}")

    time_s = tam_log.times() * macros.NANO2SEC
    plot_file = output_dir / f"sensor_data_{name}.png"

    if log_sensors:
        print_sensor_snapshots(
            time_s,
            css_logs,
            css_names,
            imu_log,
            tam_log,
            eclipse_log,
            battery_log=battery_log,
        )

    if log_sensors or show_plots:
        plot_sensor_data(
            time_s,
            css_logs,
            css_names,
            imu_log,
            tam_log,
            eclipse_log,
            plot_file,
            show_plots,
            rw_log=rw_log,
            num_rw=num_rw,
            rw_labels=rw_labels,
            battery_log=battery_log,
            mode_log=mode_log,
        )


if __name__ == "__main__":
    run(
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
        initial_soc=DEFAULT_INITIAL_SOC,
        boot_standby_s=60.0,
    )
