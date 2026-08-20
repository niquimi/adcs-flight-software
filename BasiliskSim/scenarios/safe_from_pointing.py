"""FSW test: Pointing then Safe when SOC drains.

Initial conditions:
  SOC = 0.28  (in deadband; starts nominal, drains into Safe)
  omega ≈ 0  (quiet → Standby → Pointing path)
  duration ≈ 45 min

Expected FSW sequence:
  Standby → Pointing (nadir)
  SOC falls below 0.25 → Safe preempts any mode
  Sun-point +Z until SOC > 0.35

Launch:
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\safe_from_pointing.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="safe_from_pointing",
        duration_min=45.0,
        initial_soc=0.28,
        omega_BN_B=(0.0, 0.0, 0.0),
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
