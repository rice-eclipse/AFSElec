# Flight Log Reference

How the flight computer moves through a flight, when each flight state changes, and what every column of the navigation log (`flights/nav_NNN.csv` on the SD card) means.

All numbers below are the current defaults in [config.h](config.h). If a setting is changed there, this document's numbers change with it.

---

## 1. Sequence from power-on

| Step | What happens | Duration |
|---|---|---|
| Boot tests | Each sensor and the SD card are tested. **Green LED** turns on if every test passes and every sensor starts. | a few seconds |
| Servo sweep | Every servo sweeps ±10° around its center. | 10 s |
| Alignment (`phase` = 1) | **Keep the rocket still.** Sensors are averaged for 0.5 s, then the EKF starts and learns the gyro biases. The KX134 is checked against the ICM45686 here. | 5 s |
| Pad (`phase` = 2, LAUNCHPAD) | Waiting for launch. Pad hold keeps the EKF from drifting (see §2). Servos go to center for 2 s, then are released to save power. **Blue LED** = pitch and yaw both under 10°. | as long as needed |
| Flight | The states below. | — |

The log file is created when navigation starts, so it begins with the alignment rows.

---

## 2. Flight states and when they change

```
LAUNCHPAD ──launch──▶ POWERED_ASCENT ──burnout──▶ CRUISING ──apogee──▶ DESCENT ──touchdown──▶ LANDED
    0                       1                        2                    3                      4
                            └───────────── apogee (if burnout was missed) ─────▶
```

- **Forward only:** states only move forward and never go back. LANDED is final.
- **When checks run:** the checks run on every IMU sample (about 6.4 kHz), but only after alignment ends.
- **Hold times:** every condition has a **hold time**. It must stay true without a break for that long, so one noisy sample can't trigger a change. The hold restarts if the condition drops out even once.
- **Detection delay:** a state changes at the end of its hold time, so the logged change is **later than the actual event** by about that hold time. Allow for this when lining up events.
- **Data sources:** all altitudes and vertical speeds come from the EKF. Acceleration and rotation are bias-corrected IMU readings.

### LAUNCHPAD → POWERED_ASCENT (launch)

Either of these:

| Trigger | Condition | Hold |
|---|---|---|
| Main | Total acceleration above **3 g** (`LAUNCH_ACCEL_G`) | **50 ms** (`LAUNCH_HOLD_MS`) |
| Backup | Altitude above **30 m** **and** climbing faster than **15 m/s** | **250 ms** |

- Liftoff is therefore about **50 ms before** the logged change.
- The backup catches a launch the accelerometer misses, for example a slow-starting motor.
- 3 g for 50 ms doesn't happen from normal handling on the pad.

**At launch:**
- The EKF switches to flight mode.
- All servos are driven to center and held.
- Pad hold stops.
- Logging switches to 300 Hz.

### POWERED_ASCENT → CRUISING (burnout)

| Condition | Hold |
|---|---|
| At least **300 ms** since launch was detected (`MIN_BURN_MS`), **then** acceleration along the nose below **0.5 g** (`BURNOUT_AXIAL_G`) | **100 ms** (`BURNOUT_HOLD_MS`) |

- While the motor burns, acceleration along the nose is several g. After burnout only drag acts, which shows as slightly **negative** acceleration along the nose. The 0.5 g threshold sits between the two.
- The 300 ms minimum ignores the first moments of the burn. The 100 ms hold only starts counting once that minimum has passed.
- Actual burnout is about **100 ms before** the logged change.
- **Missed burnout:** if the climb rate stays below 0 for 300 ms while still in POWERED_ASCENT, the state goes straight to DESCENT.

**At burnout:** the roll test schedule starts (see §3).

### CRUISING → DESCENT (apogee)

| Condition | Hold |
|---|---|
| At least **1 s** since burnout (`MIN_COAST_MS`), **then** EKF climb rate below 0 | **300 ms** (`APOGEE_HOLD_MS`) |
| …**or**, after that same 1 s minimum: altitude **10 m** below the highest altitude reached (`APOGEE_ALT_DROP_M`) | none |

- The highest altitude is tracked from launch.
- Actual apogee is about **300 ms before** the logged change. The altitude-drop trigger is a backup in case the climb-rate estimate is off.

**At apogee:** if the roll test is still running it ends, and the tabs are centered.

### DESCENT → LANDED (touchdown)

**All** of these at once, held for **3 s** (`LANDED_HOLD_MS`):

| Condition | Value |
|---|---|
| Vertical speed (up or down) | under **1 m/s** |
| Total acceleration | within **1 g ± 0.2 g** |
| Rotation rate | under **20 °/s** |
| Altitude | stays within a **3 m** band for the whole 3 s |

- A steady descent under a parachute also reads about 1 g and little rotation. The vertical-speed and altitude-band checks are what tell it apart from being on the ground.
- Touchdown is about **3 s before** the logged change.

**At landing:**
- Logging drops back to 10 Hz.
- The servos are released (`SERVO_RELEASE_AFTER_LANDING`).

### Expected timeline for this rocket (116 m/s at burnout, ~7.7 s coast)

| Time after liftoff | Event |
|---|---|
| ~0.05 s | POWERED_ASCENT logged |
| burn time + ~0.1 s | CRUISING logged |
| burnout + 0.3 s | Roll test starts |
| burnout + ~6.2 s | Roll test ends (climb rate below 15 m/s), tabs centered |
| burnout + ~7.7 s | Apogee |
| apogee + ~0.3 s | DESCENT logged |
| touchdown + 3 s | LANDED logged |

### Bench testing (serial `state` command)

- With `NAV_SERIAL_STATE_CMD = true`, sending `state` on the serial port moves to the next state.
- With `NAV_SERIAL_STATE_MANUAL = true` as well, the automatic checks are paused, and the roll test runs its full 6.5 s without the climb-rate stop.
- **Both must be `false` for flight.**

---

## 3. Other automatic behaviour

| Feature | What it does | Shown in |
|---|---|---|
| **Pad hold** | While on the pad and truly still (rotation under 3°/s, acceleration within 1 g ± 0.05 g), the EKF is told 100 times a second that the rocket isn't moving. Its altitude zero slowly follows weather pressure changes, with a 20 s time constant. Moments when the rocket is moved are skipped. | `north_m`, `east_m`, `alt_m` stay near 0 |
| **High-g switch** | Above **15 g** on the KX134, the ICM45686 accelerometer (±16 g) is ignored and the KX134 is used instead, until it drops below **13 g**. The gyro always comes from the ICM45686. Turned off for the flight if the two accelerometers disagree by more than 0.25 g during alignment. | `high_g`, `high_g_accel_g` |
| **Barometer lockout** | Between **Mach 0.9 and 1.1** the barometer isn't used by the EKF, because shock waves corrupt its reading. Not reached by this rocket (peak about Mach 0.34). | `baro_locked` |
| **Roll test** | During CRUISING, all four fin tabs (PWM1–4) get the same angle in **doublets**: +A for 0.3 s, then −A for 0.3 s, then center for 0.4 s, repeated once a second. A cycles through 5°, 10°, 15°. It ends when the climb rate drops below 15 m/s, 6.5 s after burnout, or at apogee, whichever comes first. It **aborts** for good if the roll rate goes over 360°/s. | `tab_deg`, `tab_us`, `roll_test`, `doublet` |
| **Servo power** | Released on the pad after 2 s at center. Held at center from launch. The roll test moves the tabs during coast. Released again after landing. | `tab_us` = 0 while released |

---

## 4. Log rate

| When | Rows per second |
|---|---|
| Alignment, LAUNCHPAD, LANDED | 10 (`NAV_LOG_IDLE_HZ`) |
| POWERED_ASCENT, CRUISING, DESCENT | 300 (`NAV_LOG_FLIGHT_HZ`) |
| Any state change | one extra row immediately |

Every sensor runs at full speed and the EKF uses every sample all the time. The log rate only sets how often a snapshot is written.

---

## 5. Column reference

**Axis names:** "IMU axes" means the ICM45686's axes: **+x = board right, +y = nose, +z = out of the board**.

**Averaged vs snapshot columns:** columns marked *averaged* are the mean of every IMU sample since the previous row, so nothing between rows is lost. All other columns are the value at the moment the row was taken.

### Timing and status

| Column | Meaning | Source |
|---|---|---|
| `t_us` | Time since navigation started, µs | Pico timer |
| `seq` | Row number from 0. Gaps = rows dropped before reaching the SD card. | Row counter |
| `phase` | 1 = aligning, 2 = navigating | Nav task |
| `in_flight` | EKF flight mode: 1 = flight. Stays 1 once set. | EKF. Set at launch (by the state machine or the EKF's own more-than-2-g-upward check). |
| `flight_state` | 0 launchpad, 1 powered ascent, 2 cruising, 3 descent, 4 landed | State machine (§2) |
| `mach` | Mach number | EKF speed ÷ speed of sound. Speed of sound uses the pad temperature, minus 6.5 °C per km of altitude. |
| `baro_locked` | 1 = barometer not used by the EKF (Mach 0.9–1.1) | §3 |
| `high_g` | 1 = KX134 is the accelerometer in use | §3 |
| `high_g_accel_g` | KX134 total acceleration, g (0 if the KX134 isn't in use for this flight) | KX134, in IMU axes, plus the offset measured during alignment |

### Rotation, acceleration and air data

| Column | Meaning | Source |
|---|---|---|
| `p_dps` | Rotation rate about IMU x (board right), °/s, *averaged* | ICM45686 gyro minus the EKF gyro bias |
| `q_dps` | Rotation rate about IMU y (nose), °/s, *averaged*. **This is the roll rate.** | Same |
| `r_dps` | Rotation rate about IMU z (out of the board), °/s, *averaged* | Same |
| `roll_rate_dps` | Rotation rate about the nose axis, °/s, *averaged* (equals `q_dps` on this board) | Same |
| `ax_mps2`, `ay_mps2`, `az_mps2` | Measured acceleration in IMU axes, m/s², *averaged*. Gravity is included: about +9.81 on the upward axis at rest. | The accelerometer in use (ICM45686, or KX134 when `high_g` = 1) minus the EKF accel bias |
| `speed_mps` | Total speed, m/s, relative to the ground (wind not included) | EKF |
| `pressure_mbar` | Latest barometer reading, mbar (recorded even during the lockout) | MS5611 |
| `qbar_pa` | Dynamic pressure ½ρV², Pa | ρ from `pressure_mbar` and the same temperature as `mach`; V = `speed_mps` |

### Roll test

| Column | Meaning | Source |
|---|---|---|
| `tab_deg` | Commanded tab angle from center, ° (the same on all four tabs) | Roll test |
| `tab_us` | Pulse width sent to servo 1 (PWM1), µs. 0 = released. Tabs 2–4 differ only by their center values. | Servo driver |
| `roll_test` | 0 waiting, 1 running, 2 done, 3 aborted | Roll test |
| `doublet` | Doublet number while running (1, 2, …) | Roll test |

### Attitude (EKF)

| Column | Meaning |
|---|---|
| `qw`, `qx`, `qy`, `qz` | Attitude quaternion, rotating IMU axes into north-east-down |
| `roll_deg` | Rotation about the nose, °. Clockwise seen from the tail. 0 when the board's right edge faces north. |
| `pitch_deg` | Nose tilt toward east, ° (0 = vertical) |
| `yaw_deg` | Nose tilt toward north, ° (0 = vertical) |
| `tilt_deg` | Angle of the nose from vertical, ° |
| `tilt_az_deg` | Compass direction the nose leans toward, 0–360° (0 = north, 90 = east). Magnetic north unless `MAG_DECLINATION_DEG` is set. |

### Position and velocity (EKF, relative to the starting point)

| Column | Meaning |
|---|---|
| `north_m`, `east_m` | Horizontal position, m |
| `alt_m` | Altitude above the pad, m (IMU only while `baro_locked` = 1) |
| `vn_mps`, `ve_mps`, `vd_mps` | Velocity north, east and **down**, m/s. Climb rate = −`vd_mps`. |
| `baro_alt_m` | Altitude from the barometer alone, m, for comparison with `alt_m`. Uses the pad pressure, which follows the weather while on the pad. |

### Filter uncertainty (1-sigma: about 68% of the time the true value is within this)

| Column | Meaning |
|---|---|
| `tilt_std_deg` | Uncertainty of the rocket's tilt, ° |
| `heading_std_deg` | Uncertainty of heading, ° |
| `alt_std_m` | Uncertainty of altitude, m |
| `horiz_std_m` | Uncertainty of horizontal position, m |

### Sensor biases (EKF estimates)

| Column | Meaning |
|---|---|
| `ba_x_mps2`, `ba_y_mps2`, `ba_z_mps2` | Estimated accelerometer error in IMU axes, m/s² |
| `bg_x_dps`, `bg_y_dps`, `bg_z_dps` | Estimated gyro error in IMU axes, °/s (learned mostly during alignment and pad hold) |

### Performance and diagnostics

| Column | Meaning |
|---|---|
| `ekf_load_pct` | Share of time since the previous row spent in EKF calculations, % of core 0 |
| `ekf_max_us` | Longest single EKF calculation since the previous row, µs |
| `imu_n` | IMU samples used by the EKF since navigation started (running total) |
| `baro_n` | Barometer readings used by the EKF (running total; readings skipped during the lockout aren't counted) |
| `mag_n` | Magnetometer readings used by the EKF (running total) |
| `dropped` | Rows lost because the SD logger fell behind (running total). Doesn't include SD write errors. |

---

## 6. Notes for analysis

- **Use `t_us` for time.** Row spacing changes with the flight state, and there are extra rows at state changes.
- **Averaged columns lag slightly.** The rate and acceleration columns are averages over the interval up to each row, so treat them as half an interval (about 1.7 ms at 300 Hz) earlier than the other columns in the same row.
- **Roll acceleration:** differentiate `q_dps` (or `roll_rate_dps`) with smoothing, for example a Savitzky–Golay filter over 20–40 ms.
- **Roll test fit:** for each doublet, fit `roll_accel = L_δ·tab_deg + L_p·roll_rate + L₀`. Include the centered part after the doublet, because that's what gives L_p (damping) and L₀ (bias). Then plot L_δ against `qbar_pa`; expect a straight line.
  - `tab_deg` is the commanded angle. The real tab lags it by the servo response time, which needs measuring on the bench.
  - All four tabs move together, so L_δ is the effect of all four.
- **Event times:** logged state changes are later than the real events by about the hold times in §2: liftoff ~50 ms, burnout ~100 ms, apogee ~300 ms, touchdown ~3 s.
