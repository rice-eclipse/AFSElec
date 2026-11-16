# Flight replay simulator

Runs a flight's `nav_*.csv` back through the flight software with different settings and writes a
new CSV with the same columns as the board's log.

```powershell
.\sim\run_sim.ps1 flights\nav_003.csv              # -> flights\nav_003_sim.csv
.\sim\run_sim.ps1 flights\nav_003.csv out.csv
.\sim\run_sim.ps1 flights\nav_003.csv -FromFirmwareConfig   # reset sim\config.h to the firmware's first
```

The script rebuilds the simulator when anything has changed, then runs it. It prints the board's
state and roll-test changes next to the simulated ones. It needs CMake and Visual Studio 2022 with
the C++ workload.

## Settings

| File | What it holds |
|---|---|
| `sim/config.h` | A copy of the firmware's `config.h`. Edit it freely: state machine thresholds, pad hold, high-g, roll test, log rates, Mach lockout, servo centers. The firmware's own `config.h` is never touched. |
| `sim/sim_settings.h` | Sim-only options (step size, pad temperature, synthetic magnetometer) and `simEkfConfig()` for EKF tuning (`NavEKF::Config`). |

The simulator compiles the firmware's own `NavEKF.cpp`, `flight_state.cpp` and `roll_test.cpp`,
so it behaves exactly like the board. `sim.cpp` copies the nav loop from `navTask()` in `loop.cpp`.
If you change that loop, make the same change in `sim.cpp`.

## Inputs taken from the flight log

| Sensor | Columns |
|---|---|
| Gyro | `p_dps`, `q_dps`, `r_dps` plus `bg_*_dps` (the board removed its bias estimate; the sim adds it back) |
| Accelerometer | `ax_mps2`, `ay_mps2`, `az_mps2` plus `ba_*_mps2` |
| Barometer | `pressure_mbar`, fed whenever it changes |
| KX134 magnitude | `high_g_accel_g`, used only for the high-g switch |

The EKF and every output come from the simulation. Each input row is split into 1 ms steps
(`SIM_STEP_US`), so the hold timers, pad hold and roll test run at close to the board's resolution.

## Limitations

These come from what the board logs:

- **IMU data is averaged per row:** 10 Hz on the pad, 300 Hz in flight. Velocity and attitude
  changes are kept exactly, but short peaks are smoothed.
  - Launch detection can come up to one pad row (100 ms) later than on the board.
  - Making the pad thresholds stricter than the board's has no effect inside a row.
- **No magnetometer:** it isn't logged, so heading starts at 0. This affects `yaw_deg`,
  `pitch_deg`, `tilt_az_deg`, `north_m` and `east_m`. It does not affect tilt, altitude, speed or
  the flight states.
  - `SIM_MAG_FROM_LOGGED_ATTITUDE` makes the heading follow the board's estimate instead.
- **High-g:** the log only has the accelerometer the board actually used. Where the KX134 had
  taken over, its values are already in `ax`–`az`. Changing `HIGHG_*` changes the `high_g` column,
  but not the values fed to the EKF.
- **Not simulated:**
  - `ekf_load_pct`, `ekf_max_us` and `dropped` are always 0.
  - The serial `state` command isn't replayed. With `NAV_SERIAL_STATE_MANUAL` on, the state stays
    LAUNCHPAD.
- **Pad temperature:** recovered from the board's own `mach` and `speed_mps` columns, or set with
  `SIM_PAD_TEMP_C`.
- **Log rate:** a higher log rate than the original doesn't add detail. The extra rows just repeat
  the input's resolution.
