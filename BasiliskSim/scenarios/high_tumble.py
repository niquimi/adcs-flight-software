"""FSW test: high tumble → Detumble with RW saturation.

Initial conditions:
  omega ≈ (0.12, -0.08, 0.10) rad/s  (‖ω‖ >> ω_in)
  SOC = 0.55
  duration ≈ 40 min

Expected FSW sequence:
  Standby → Detumble (PD hits τ_max = 0.2 N·m per axis)
  After 5 s under 0.005 rad/s + nadir_valid → Pointing

Watch C++ torque commands and Vizard RW panel for saturation.

Launch:
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\high_tumble.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="high_tumble",
        duration_min=40.0,
        initial_soc=0.55,
        omega_BN_B=(0.12, -0.08, 0.10),
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
