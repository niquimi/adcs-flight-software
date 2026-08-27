"""TCP bridge: telemetry and commands on one bidirectional SIL socket."""

from __future__ import annotations

import math
import socket
from typing import Sequence

from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import macros
import numpy as np

from BasiliskSim.bridge.battery_soc_model import BatterySocModel
from BasiliskSim.bridge.sil_protocol import (
    COMMAND_PACKET_SIZE,
    PACKET_FSW_STATUS,
    PACKET_MTB_DIPOLE_CMD,
    PACKET_RW_TORQUE_CMD,
    pack_boot_config,
    pack_sensor_packet,
    unpack_command_packet,
)


COMMAND_LOG_EPSILON = 1e-12


def _normalize3(v: Sequence[float]) -> list[float] | None:
    n2 = float(v[0]) * float(v[0]) + float(v[1]) * float(v[1]) + float(v[2]) * float(v[2])
    if n2 < 1e-24:
        return None
    inv = 1.0 / math.sqrt(n2)
    return [float(v[0]) * inv, float(v[1]) * inv, float(v[2]) * inv]


def _sub3(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float]:
    return (
        float(a[0]) - float(b[0]),
        float(a[1]) - float(b[1]),
        float(a[2]) - float(b[2]),
    )


class SensorSocketBridge(sysModel.SysModel):
    """Send sensor telemetry and apply inbound RW/MTB commands on one TCP link."""

    def __init__(
        self,
        tam,
        imu,
        css_list: Sequence,
        eclipse_obj,
        rw_cmd_msg: messaging.ArrayMotorTorqueMsg,
        rw_cmd_data: messaging.ArrayMotorTorqueMsgPayload,
        mtb_cmd_msg: messaging.MTBCmdMsg,
        mtb_cmd_data: messaging.MTBCmdMsgPayload,
        num_rw: int,
        num_mtb: int,
        battery: BatterySocModel | None = None,
        sc_state_msg=None,
        earth_msg=None,
        sun_msg=None,
        mag_env_msg=None,
        mag_field=None,
        host: str = "127.0.0.1",
        port: int = 5557,
        interval_s: float = 10.0,
        css_noise_std: float = 0.0,
        boot_standby_s: float = 0.0,
    ) -> None:
        super().__init__()
        self.ModelTag = "SensorSocketBridge"
        self.tam = tam
        self.imu = imu
        self.css_list = list(css_list)
        self.eclipse_obj = eclipse_obj
        self.rw_cmd_msg = rw_cmd_msg
        self.rw_cmd_data = rw_cmd_data
        self.mtb_cmd_msg = mtb_cmd_msg
        self.mtb_cmd_data = mtb_cmd_data
        self.num_rw = num_rw
        self.num_mtb = num_mtb
        self.battery = battery
        self.sc_state_msg = sc_state_msg
        self.earth_msg = earth_msg
        self.sun_msg = sun_msg
        self.mag_env_msg = mag_env_msg
        self.mag_field = mag_field
        self.host = host
        self.port = port
        self.interval_ns = macros.sec2nano(interval_s)
        self.css_noise_std = float(css_noise_std)
        self.boot_standby_s = float(boot_standby_s)
        self._rng = np.random.default_rng()

        self.sock: socket.socket | None = None
        self.rx_buffer = bytearray()
        self.last_sent_ns = 0
        self._refs_log_ns = -10**18
        print(
            "SIL bridge refs: "
            f"sc={'yes' if sc_state_msg is not None else 'NO'} "
            f"earth={'yes' if earth_msg is not None else 'NO'} "
            f"sun={'yes' if sun_msg is not None else 'NO'} "
            f"mag_msg={'yes' if mag_env_msg is not None else 'NO'} "
            f"mag_field={'yes' if mag_field is not None else 'NO'}"
        )

    def Reset(self, currentSimNanos: int) -> int:
        self.last_sent_ns = 0
        self.rx_buffer.clear()
        self._close_socket()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.host, self.port))
        except OSError as exc:
            self._close_socket()
            raise ConnectionError(
                f"Could not connect to SIL server at {self.host}:{self.port}. "
                "Start the C++ sensor_receiver first "
                f"(.\\SIL\\cpp\\build\\Release\\sensor_receiver.exe)."
            ) from exc
        self.sock.sendall(pack_boot_config(self.boot_standby_s))
        self.sock.setblocking(False)
        print(
            f"SIL bridge connected to {self.host}:{self.port} "
            "(bidirectional: telemetry out, commands in)"
            + (
                f", boot Standby {self.boot_standby_s:.0f} s"
                if self.boot_standby_s > 0.0
                else ""
            )
        )
        return super().Reset(currentSimNanos)

    def UpdateState(self, currentSimNanos: int) -> int:
        self._poll_commands(currentSimNanos)

        if self.sock is None:
            return super().UpdateState(currentSimNanos)

        if currentSimNanos - self.last_sent_ns < self.interval_ns:
            return super().UpdateState(currentSimNanos)

        imu_data = self.imu.sensorOutMsg.read()
        tam_data = self.tam.tamDataOutMsg.read()
        eclipse_data = self.eclipse_obj.eclipseOutMsgs[0].read()
        css_raw = [css.cssDataOutMsg.read().OutputData for css in self.css_list]
        css_values = list(css_raw[:6]) + [0.0] * max(0, 6 - len(css_raw))
        # Basilisk CSS A=I cannot be set to 0 from Python; add white noise here.
        # Skip when all diodes are dark so eclipse does not fake a sun vector.
        if self.css_noise_std > 0.0 and max(css_values) >= 0.05:
            css_values = [
                float(v + self._rng.normal(0.0, self.css_noise_std))
                for v in css_values
            ]
        gyro = imu_data.AngVelPlatform
        mag_nt = [component * 1e9 for component in tam_data.tam_S]
        timestamp_s = currentSimNanos * macros.NANO2SEC
        battery_level = self.battery.soc if self.battery is not None else 1.0

        # Inertial references matching Basilisk sensors (not FSW onboard models).
        # r_BN verification is Earth-centered: SPICE sc.r_BN_N is SSB/sun-centered.
        sun_N = (0.0, 0.0, 0.0)
        B_N = (0.0, 0.0, 0.0)
        r_BN = (0.0, 0.0, 0.0)
        sigma_BN = (0.0, 0.0, 0.0)
        ref_err = ""
        try:
            r_sc = None
            if self.sc_state_msg is not None:
                sc = self.sc_state_msg.read()
                r_sc = list(sc.r_BN_N)
                sigma_BN = (
                    float(sc.sigma_BN[0]),
                    float(sc.sigma_BN[1]),
                    float(sc.sigma_BN[2]),
                )
                if self.earth_msg is not None:
                    r_earth = list(self.earth_msg.read().PositionVector)
                    r_BN = _sub3(r_sc, r_earth)
                else:
                    r_BN = (float(r_sc[0]), float(r_sc[1]), float(r_sc[2]))
                    ref_err += "earth_msg_none;"
            else:
                ref_err += "sc_msg_none;"
            if r_sc is not None and self.sun_msg is not None:
                sun = self.sun_msg.read()
                r_sun = list(sun.PositionVector)
                s_vec = [
                    float(r_sun[0]) - float(r_sc[0]),
                    float(r_sun[1]) - float(r_sc[1]),
                    float(r_sun[2]) - float(r_sc[2]),
                ]
                s_hat = _normalize3(s_vec)
                if s_hat is not None:
                    sun_N = (s_hat[0], s_hat[1], s_hat[2])
                else:
                    ref_err += "sun_norm_fail;"
            else:
                ref_err += "sun_msgs_none;"
            # Mag inertial truth: dipole envOut in N (same model the FSW dipole should match).
            # Fallback: TAM rotated by truth attitude (includes sensor noise).
            b_raw = None
            b_src = ""
            if self.mag_field is not None and len(self.mag_field.envOutMsgs) > 0:
                try:
                    mf = self.mag_field.envOutMsgs[0].read()
                    b_raw = [float(mf.magField_N[0]), float(mf.magField_N[1]), float(mf.magField_N[2])]
                    b_src = "envOut"
                    if _normalize3(b_raw) is None:
                        b_raw = None
                        ref_err += "envOut_zero;"
                except Exception as exc:  # noqa: BLE001
                    ref_err += f"envOut:{type(exc).__name__};"
                    b_raw = None
            if b_raw is None:
                try:
                    sc_m = self.sc_state_msg.read() if self.sc_state_msg is not None else None
                    if sc_m is not None:
                        C_BN = np.array(rbk.MRP2C(sc_m.sigma_BN), dtype=float)
                        b_B = np.array(
                            [float(mag_nt[0]) * 1e-9, float(mag_nt[1]) * 1e-9, float(mag_nt[2]) * 1e-9],
                            dtype=float,
                        )
                        b_N_vec = C_BN.T @ b_B
                        b_raw = [float(b_N_vec[0]), float(b_N_vec[1]), float(b_N_vec[2])]
                        b_src = "tam_truth_att"
                except Exception as exc:  # noqa: BLE001
                    ref_err += f"tam_truth:{type(exc).__name__};"
                    b_raw = None
            if b_raw is None:
                try:
                    mf = self.tam.magInMsg.read()
                    b_raw = [float(mf.magField_N[0]), float(mf.magField_N[1]), float(mf.magField_N[2])]
                    b_src = "tam_magIn"
                    if _normalize3(b_raw) is None:
                        b_raw = None
                        ref_err += "tam_magIn_zero;"
                except Exception as exc:  # noqa: BLE001
                    ref_err += f"tam_magIn:{type(exc).__name__};"
                    b_raw = None
            if b_raw is None:
                ref_err += "mag_unavailable;"
            else:
                b_hat = _normalize3(b_raw)
                if b_hat is not None:
                    B_N = (b_hat[0], b_hat[1], b_hat[2])
                    if b_src:
                        ref_err += f"B_src={b_src};"
                else:
                    ref_err += "mag_norm_fail;"
        except Exception as exc:  # noqa: BLE001 — SIL diagnostics must not drop telemetry
            ref_err += f"exc:{type(exc).__name__}:{exc};"

        if currentSimNanos - self._refs_log_ns >= macros.sec2nano(2.0):
            self._refs_log_ns = currentSimNanos
            print(
                f"SIL refs t={timestamp_s:.1f}s sun_N={sun_N} B_N={B_N} err='{ref_err}'"
            )

        packet = pack_sensor_packet(
            timestamp_s,
            gyro,
            mag_nt,
            css_values,
            battery_level,
        )
        try:
            # sendall needs a blocking socket; restore non-blocking afterwards.
            self.sock.setblocking(True)
            self.sock.sendall(packet)
            self.sock.setblocking(False)
        except OSError as exc:
            print(f"SIL telemetry send failed: {exc}")
            self._close_socket()
            return super().UpdateState(currentSimNanos)

        self.last_sent_ns = currentSimNanos
        return super().UpdateState(currentSimNanos)

    def _poll_commands(self, currentSimNanos: int) -> None:
        if self.sock is None:
            return
        try:
            chunk = self.sock.recv(4096)
        except BlockingIOError:
            return
        except OSError as exc:
            print(f"SIL command recv failed: {exc}")
            self._close_socket()
            return

        if not chunk:
            print("SIL server disconnected")
            self._close_socket()
            return

        self.rx_buffer.extend(chunk)
        while len(self.rx_buffer) >= COMMAND_PACKET_SIZE:
            raw = bytes(self.rx_buffer[:COMMAND_PACKET_SIZE])
            del self.rx_buffer[:COMMAND_PACKET_SIZE]
            try:
                cmd = unpack_command_packet(raw)
            except ValueError as exc:
                print(f"SIL command rejected: {exc}")
                continue
            self._apply_command(cmd, currentSimNanos)

    def _apply_command(self, cmd: dict, currentSimNanos: int) -> None:
        packet_type = cmd["packet_type"]
        seq = cmd["sequence"]

        if packet_type == PACKET_FSW_STATUS:
            if self.battery is not None:
                prev = self.battery.mode
                self.battery.set_mode_id(cmd["mode"])
                if self.battery.mode != prev:
                    print(
                        f"SIL FSW mode={self.battery.mode} seq={seq} "
                        f"SOC={self.battery.soc:.3f}"
                    )
            return

        values = cmd["values"]

        if packet_type == PACKET_RW_TORQUE_CMD:
            torques = [0.0] * max(self.num_rw, 3)
            for idx in range(min(self.num_rw, 3)):
                torques[idx] = values[idx]
            self.rw_cmd_data.motorTorque = torques[: self.num_rw]
            self.rw_cmd_msg.write(self.rw_cmd_data, currentSimNanos)
            if self.battery is not None:
                self.battery.note_actuator_command(rw_torques=values[:3])
            if any(abs(value) > COMMAND_LOG_EPSILON for value in values[:3]):
                soc_info = (
                    f" | SOC={self.battery.soc:.3f}"
                    if self.battery is not None
                    else ""
                )
                print(
                    f"SIL RW torque cmd seq={seq} "
                    f"u=[{values[0]:+.4f}, {values[1]:+.4f}, {values[2]:+.4f}] Nm"
                    f"{soc_info}"
                )
            return

        if packet_type == PACKET_MTB_DIPOLE_CMD:
            dipoles = [0.0] * 36
            for idx in range(min(self.num_mtb, 3)):
                dipoles[idx] = values[idx]
            self.mtb_cmd_data.mtbDipoleCmds = dipoles
            self.mtb_cmd_msg.write(self.mtb_cmd_data, currentSimNanos)
            if self.battery is not None:
                self.battery.note_actuator_command(mtb_dipoles=values[:3])
            if any(abs(value) > COMMAND_LOG_EPSILON for value in values[:3]):
                print(
                    f"SIL MTB dipole cmd seq={seq} "
                    f"m=[{values[0]:+.4f}, {values[1]:+.4f}, {values[2]:+.4f}] Am2"
                )

    def _close_socket(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.rx_buffer.clear()
