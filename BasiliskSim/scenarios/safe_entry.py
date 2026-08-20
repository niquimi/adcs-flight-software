"""FSW test: Safe supervisor from t = 0 (low SOC).

Initial conditions:
  SOC = 0.22  (< 0.25 enter threshold)
  omega ≈ (0.01, -0.005, 0.02) rad/s  (moderate tumble)
  duration ≈ 30 min

Expected FSW sequence:
  Safe immediately (sun-point +Z, rate damping)
  Stays in Safe until SOC > 0.35, then Detumble or Standby from ‖ω‖

Launch:
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\safe_entry.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="safe_entry",
        duration_min=30.0,
        initial_soc=0.22,
        omega_BN_B=(0.01, -0.005, 0.02),
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
