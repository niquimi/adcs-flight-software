"""FSW test: Pointing through eclipse (nadir coast, filter holds attitude).

Initial conditions:
  omega ≈ 0, SOC = 0.55
  true_anomaly = 0 deg  (shift orbit phase vs default f=80; eclipse earlier in run)
  duration ≈ 40 min

Expected FSW sequence:
  Standby → Pointing (nadir)
  In umbra: css_valid false but nadir_valid should remain (Kepler + C_BN coast)
  Should NOT drop to Standby solely because sun sensors are dark
  Attitude filter freezes gyro-bias update without TRIAD

Launch:
  .\\.venv\\Scripts\\python.exe .\\BasiliskSim\\scenarios\\eclipse_coast.py
"""

from BasiliskSim.scenarios.basic_orbit_vizard import run

if __name__ == "__main__":
    run(
        name="eclipse_coast",
        duration_min=40.0,
        initial_soc=0.55,
        omega_BN_B=(0.0, 0.0, 0.0),
        true_anomaly_deg=0.0,
        real_time=True,
        show_live_stream=True,
        enable_sil_bridge=True,
        log_sensors=False,
        show_plots=False,
    )
