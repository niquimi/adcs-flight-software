"""FSW test: moderate tumble → Detumble → Pointing (nadir).

Initial conditions:
  omega ≈ (0.04, -0.02, 0.03) rad/s  (‖ω‖ > ω_in = 0.015)
  SOC = 0.55  (above Safe band; no energy supervisor)
  duration ≈ 40 min

Expected FSW sequence (C++ console `Mode=`):
  Standby → Detumble (rate damping) → Pointing (after ‖ω‖ < 0.005 for 5 s and nadir_valid)

Launch (with sensor_receiver already running on :5557):
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\detumble_to_pointing.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="detumble_to_pointing",
        duration_min=40.0,
        initial_soc=0.55,
        omega_BN_B=(0.04, -0.02, 0.03),
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
