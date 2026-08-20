"""Simple spacecraft battery SOC integrator for ADCS demo / SIL.

SOC_{k+1} = clamp(SOC_k + (P_gen - P_load) / E_cap * dt, 0, 1)

P_gen  ∝ max(0, css_z) * eclipse_shadow   (panels on +Z; shadowFactor 1=sun, 0=umbra)
P_load = P_mode[FSW ModeId] + k_rw * Σ|τ|
"""

from __future__ import annotations

from typing import Sequence

from Basilisk.architecture import messaging, sysModel
from Basilisk.utilities import macros


# Demo capacity: ~111 Wh so SOC moves visibly over 1–2 LEO orbits.
E_CAP_WS = 4.0e5
P_GEN_MAX_W = 90.0
CSS_SCALE = 2.0  # matches CoarseSunSensor.scaleFactor in the scenario

# Mode loads (W): Detumble/Pointing > Standby > Safe (minimum ADCS).
P_LOAD_W = {
    "standby": 28.0,
    "detumble": 55.0,
    "pointing": 50.0,
    "safe": 24.0,
}
P_RW_PER_NM = 80.0  # optional actuator term

# Matches FlightSoftware/types.h ModeId.
MODE_BY_ID = {
    0: "standby",
    1: "detumble",
    2: "pointing",
    3: "safe",
}
MODE_ID = {name: mode_id for mode_id, name in MODE_BY_ID.items()}


class BatterySocModel(sysModel.SysModel):
    """Integrate SOC every sim step and publish Vizard PowerStorageStatus messages."""

    def __init__(
        self,
        css_z,
        eclipse_obj,
        initial_soc: float = 0.55,
        e_cap_ws: float = E_CAP_WS,
        p_gen_max_w: float = P_GEN_MAX_W,
    ) -> None:
        super().__init__()
        self.ModelTag = "BatterySocModel"
        self.css_z = css_z
        self.eclipse_obj = eclipse_obj
        self.e_cap_ws = float(e_cap_ws)
        self.p_gen_max_w = float(p_gen_max_w)

        self.soc = float(min(1.0, max(0.0, initial_soc)))
        self.mode = "standby"
        self.last_css_z = 0.0
        self.last_tau_abs_sum = 0.0
        self.last_net_power_w = 0.0
        self._last_ns: int | None = None

        self.battery_msg = messaging.PowerStorageStatusMsg()
        self.mode_msg = messaging.PowerStorageStatusMsg()
        self.css_z_msg = messaging.PowerStorageStatusMsg()
        self._publish(0)

    def Reset(self, CurrentSimNanos: int) -> int:
        self._last_ns = None
        self._publish(CurrentSimNanos)
        return super().Reset(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int) -> int:
        if self._last_ns is None:
            self._last_ns = CurrentSimNanos
            self._publish(CurrentSimNanos)
            return super().UpdateState(CurrentSimNanos)

        dt = (CurrentSimNanos - self._last_ns) * macros.NANO2SEC
        self._last_ns = CurrentSimNanos
        if dt <= 0.0:
            return super().UpdateState(CurrentSimNanos)

        css_z = float(self.css_z.cssDataOutMsg.read().OutputData)
        self.last_css_z = css_z
        shadow = float(self.eclipse_obj.eclipseOutMsgs[0].read().shadowFactor)
        # shadowFactor: 1 = full sun, 0 = umbra → P_gen ≈ 0 in eclipse.
        irradiance = max(0.0, css_z / CSS_SCALE) * max(0.0, min(1.0, shadow))
        p_gen = self.p_gen_max_w * irradiance

        p_load = P_LOAD_W.get(self.mode, P_LOAD_W["standby"])
        p_load += P_RW_PER_NM * self.last_tau_abs_sum
        self.last_net_power_w = p_gen - p_load

        self.soc = min(1.0, max(0.0, self.soc + self.last_net_power_w / self.e_cap_ws * dt))
        self._publish(CurrentSimNanos)
        return super().UpdateState(CurrentSimNanos)

    def note_actuator_command(
        self,
        rw_torques: Sequence[float] | None = None,
        mtb_dipoles: Sequence[float] | None = None,
    ) -> None:
        """Update the |τ| term of P_load. Mode comes from FSW status, not torque."""
        del mtb_dipoles
        torques = list(rw_torques or [])
        if torques:
            self.last_tau_abs_sum = sum(abs(float(t)) for t in torques)

    def set_mode(self, mode: str) -> None:
        key = mode.lower()
        if key not in P_LOAD_W:
            raise ValueError(f"Unknown mode '{mode}', expected one of {list(P_LOAD_W)}")
        self.mode = key

    def set_mode_id(self, mode_id: int) -> None:
        name = MODE_BY_ID.get(int(mode_id))
        if name is None:
            raise ValueError(f"Unknown FSW mode id {mode_id}")
        self.mode = name

    def _publish(self, current_ns: int) -> None:
        # Vizard GenericStorage bars use storageLevel/storageCapacity (show as %).
        bat = messaging.PowerStorageStatusMsgPayload()
        bat.storageCapacity = 100.0
        bat.storageLevel = self.soc * 100.0
        bat.currentNetPower = self.last_net_power_w
        self.battery_msg.write(bat, current_ns)

        mode_payload = messaging.PowerStorageStatusMsgPayload()
        # Vizard bar: Standby=1 .. Safe=4 so id 0 is not an empty gauge.
        mode_payload.storageCapacity = float(len(MODE_BY_ID))
        mode_payload.storageLevel = float(MODE_ID[self.mode] + 1)
        mode_payload.currentNetPower = 0.0
        self.mode_msg.write(mode_payload, current_ns)

        css_z_payload = messaging.PowerStorageStatusMsgPayload()
        css_z_payload.storageCapacity = CSS_SCALE
        css_z_payload.storageLevel = max(0.0, min(CSS_SCALE, self.last_css_z))
        css_z_payload.currentNetPower = 0.0
        self.css_z_msg.write(css_z_payload, current_ns)
