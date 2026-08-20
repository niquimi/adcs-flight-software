"""FSW test: quiet start → Standby → Pointing (skip Detumble).

Initial conditions:
  omega ≈ 0  (‖ω‖ < ω_out = 0.005)
  SOC = 0.55
  duration ≈ 20 min

Expected FSW sequence:
  Standby → Pointing when nadir_valid (no Detumble if rates stay low)

Launch:
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\quiet_to_pointing.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="quiet_to_pointing",
        duration_min=20.0,
        initial_soc=0.55,
        omega_BN_B=(0.0, 0.0, 0.0),
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
