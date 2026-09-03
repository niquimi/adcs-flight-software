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
    PACKET_FSW_ATTITUDE,
    PACKET_FSW_STATUS,
    PACKET_MTB_DIPOLE_CMD,
    PACKET_RW_TORQUE_CMD,
    pack_boot_config,
    pack_sensor_packet,
    unpack_command_packet,
)


_MODE_NAMES = ("Standby", "Detumble", "Pointing", "Safe")
_TM_LOG_INTERVAL_S = 5.0


def _att_error_deg(sigma_est: Sequence[float], sigma_true: Sequence[float]) -> float:
    """Geodesic angle between estimated and plant MRP, degrees."""
    c_est = np.array(rbk.MRP2C(list(sigma_est)), dtype=float)
    c_true = np.array(rbk.MRP2C(list(sigma_true)), dtype=float)
    trace = float(np.trace(c_est @ c_true.T))
    c = max(-1.0, min(1.0, 0.5 * (trace - 1.0)))
    return math.degrees(math.acos(c))


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
        self._sigma_true = (0.0, 0.0, 0.0)
        self._sigma_est: tuple[float, float, float] | None = None
        self._last_status: dict | None = None
        self._last_attitude_t = -1.0
        self._tm_log_s = -_TM_LOG_INTERVAL_S

    def Reset(self, currentSimNanos: int) -> int:
        self.last_sent_ns = 0
        self.rx_buffer.clear()
        self._close_socket()
        self._sigma_est = None
        self._last_status = None
        self._last_attitude_t = -1.0
        self._tm_log_s = -_TM_LOG_INTERVAL_S

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

        if self.sc_state_msg is not None:
            sc = self.sc_state_msg.read()
            self._sigma_true = (
                float(sc.sigma_BN[0]),
                float(sc.sigma_BN[1]),
                float(sc.sigma_BN[2]),
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
        self._poll_commands(currentSimNanos)
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
        got_status = False
        while len(self.rx_buffer) >= COMMAND_PACKET_SIZE:
            raw = bytes(self.rx_buffer[:COMMAND_PACKET_SIZE])
            del self.rx_buffer[:COMMAND_PACKET_SIZE]
            try:
                cmd = unpack_command_packet(raw)
            except ValueError as exc:
                print(f"SIL command rejected: {exc}")
                continue
            self._apply_command(cmd, currentSimNanos)
            if cmd["packet_type"] == PACKET_FSW_STATUS:
                got_status = True
        if got_status:
            self._maybe_print_tm()

    def _apply_command(self, cmd: dict, currentSimNanos: int) -> None:
        packet_type = cmd["packet_type"]

        if packet_type == PACKET_FSW_STATUS:
            if self.battery is not None:
                prev = self.battery.mode
                self.battery.set_mode_id(cmd["mode"])
                if self.battery.mode != prev:
                    name = _MODE_NAMES[cmd["mode"]] if cmd["mode"] < 4 else str(cmd["mode"])
                    print(
                        f"SIL FSW mode={name} t={cmd['timestamp_s']:.1f}s "
                        f"SOC={self.battery.soc:.3f}"
                    )
            self._last_status = cmd
            return

        if packet_type == PACKET_FSW_ATTITUDE:
            self._sigma_est = cmd["values"]
            self._last_attitude_t = cmd["timestamp_s"]
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
            return

        if packet_type == PACKET_MTB_DIPOLE_CMD:
            dipoles = [0.0] * 36
            for idx in range(min(self.num_mtb, 3)):
                dipoles[idx] = values[idx]
            self.mtb_cmd_data.mtbDipoleCmds = dipoles
            self.mtb_cmd_msg.write(self.mtb_cmd_data, currentSimNanos)
            if self.battery is not None:
                self.battery.note_actuator_command(mtb_dipoles=values[:3])

    def _maybe_print_tm(self) -> None:
        st = self._last_status
        if st is None:
            return
        t = float(st["timestamp_s"])
        if t - self._tm_log_s < _TM_LOG_INTERVAL_S:
            return
        self._tm_log_s = t

        flags = int(st["flags"])
        mode = int(st["mode"])
        name = _MODE_NAMES[mode] if mode < 4 else str(mode)
        soc = f"{self.battery.soc:.3f}" if self.battery is not None else "?"

        if not (flags & 4):
            att = "invalid"
        elif (
            self._sigma_est is None
            or abs(self._last_attitude_t - t) > 1e-3
        ):
            att = "n/a"
        else:
            att = f"{_att_error_deg(self._sigma_est, self._sigma_true):.2f} deg"

        print(
            f"SIL TM t={t:.1f}s mode={name}"
            f" Att={att}"
            f" |b|={st['gyro_bias_degph']:.1f} deg/h"
            f" TMR={st['tmr_mismatch_count']}"
            f" flags=0x{flags:02x}"
            f" css={int(bool(flags & 1))} nadir={int(bool(flags & 2))}"
            f" att={int(bool(flags & 4))} triad={int(bool(flags & 8))}"
            f" SOC={soc}"
            f" health=0x{int(st.get('health_flags', 0)):02x}"
            f" ack={int(st.get('last_tc_opcode', 0))},{int(st.get('last_tc_arg0', 0))}"
            f" nH={int(st.get('health_event_count', 0))}"
        )

    def _close_socket(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.rx_buffer.clear()
