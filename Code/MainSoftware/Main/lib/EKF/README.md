# EKF: asynchronous Extended Kalman Filters

Allocation-free Kalman filters for the flight computer, in three layers:

| File | What it is |
|---|---|
| `NavEKF.h/.cpp` | **Full 3D navigation (INS)**: orientation, position relative to the pad, altitude, velocity (15-state error-state EKF) |
| `AltitudeEKF.h/.cpp` | Simpler 1D filter: altitude and vertical velocity only (3 states) |
| `AsyncEKF.h` | Generic asynchronous EKF core for any state size `N` (header-only template) |
| `EKFMatrix.h` | Fixed-size float matrices (`Mat<R,C>`, `Vec<N>`), Cholesky inverse |
| `EKFRotation.h` | 3-vectors, quaternions, rotation matrices, Euler angles |
| `EKF_test.h/.cpp` | Bench test: simulated 1D and 3D flights with asynchronous, noisy sensors |

Link with `target_link_libraries(<target> EKF)`. It is plain C++17 with no Pico SDK dependency,
so the same code compiles and runs on a PC for testing.

## Why asynchronous

The sensors all run at different rates: IMU 6400 Hz, KX134 up to 25.6 kHz, barometer ~1 kHz,
magnetometer 1 kHz, TMP117 64 Hz. Their readings also don't arrive in a fixed order. A fixed-rate
filter would have to resample all of that onto one clock. Instead, the filter state carries its own
timestamp, and each sensor is fused whenever its data arrives:

```
IMU sample at t   ->  integrate the state forward to t, then hold the new sample as the input
baro sample at t  ->  integrate forward to t, then update with the pressure
baro sample late  ->  (t is already behind the state) fuse it now, comparing it with what the
                      sensor should have seen at t (h - v*lag); rejected as stale if > maxLagUs
```

## NavEKF: full 3D navigation

Estimates **orientation** (quaternion, roll/pitch/yaw, tilt from vertical), **position** relative to
the pad (north, east, altitude), **velocity**, and the **accelerometer and gyro biases**.

Frames: the world frame is NED (north, east, down) with the origin at the pad, so `altitude() = -down`.
The body frame is the IMU's own axes. **Feed accel, gyro and magnetometer in the same body frame**: the
MMC5983MA's axes have to be rotated into the ICM45686's axes before calling `magnetometer()`.

```cpp
#include "NavEKF.h"

static NavEKF nav;   // ~2.5 kB: keep it global/static, not on a task stack

// On the pad, at rest: average ~0.5 s of accel and mag, and some baro readings
NavEKF::Config cfg;
cfg.padTemperatureK   = 273.15f + baro.getTemperature();
cfg.magDeclinationDeg = 11.0f;                  // local declination (east +), so yaw = true heading
cfg.rocketAxis[0] = 1; cfg.rocketAxis[1] = 0; cfg.rocketAxis[2] = 0;   // nose direction in the IMU frame
nav.begin(padPressureMbar, accelAvg_mps2, magAvg, time_us_64(), cfg);

// Every IMU sample (convert g -> m/s^2 and deg/s -> rad/s):
nav.imu(tUs, ekf::vec3(d.ax, d.ay, d.az) * NavEKF::G,
             ekf::vec3(d.gx, d.gy, d.gz) * NavEKF::DEG2RAD);
// Every baro reading / every magnetometer reading (rotated into the IMU frame):
nav.baroPressure(tUs, baro.getPressure());
nav.magnetometer(tUs, ekf::vec3(m.x, m.y, m.z));
// On the pad, ~100 Hz, while certainly not moving (stop before arming / at the first motion):
nav.stationary(tUs);

// Outputs
nav.roll(); nav.pitch(); nav.yaw();        // deg (ZYX; ill-defined at pitch ~ +-90)
nav.rocketAngles();                       // deg, rocket roll/pitch/yaw: 0 with nose up, fine near vertical
nav.tilt(); nav.tiltAzimuth();             // deg: rocket axis from vertical, and which way it leans
nav.quaternion();                          // body -> NED
nav.north(); nav.east(); nav.altitude();   // m from the pad
nav.velocityNED(); nav.verticalSpeed();    // m/s
nav.tiltStd(); nav.headingStd(); nav.altitudeStd(); nav.horizontalStd();
nav.inFlight();
```

How it works:

* **Strapdown integration at the full IMU rate.** The nominal state (position, velocity, quaternion)
  is integrated on every IMU sample. The 15x15 covariance is propagated every 5 ms
  (`covarianceStepS`) from the accumulated delta-velocity/angle, so 6.4 kHz IMU data is affordable.
* **Error-state EKF.** The filter estimates small errors (position, velocity, a 3-component
  rotation, the two biases) and folds them into the nominal state after every update. Attitude is
  therefore always a proper unit quaternion, with no singularities.
* **Pad alignment.** `begin()` gets tilt from gravity and heading from the magnetometer.
  `stationary()` then adds zero-velocity and zero-rotation-rate updates, which learn the gyro biases
  (to ~0.003 deg/s) and the vertical accel bias, and keep the tilt locked. On the pad the
  magnetometer only corrects heading.
* **In flight.** The gyro carries the attitude. The magnetometer corrects heading and tilt about
  the axes perpendicular to the Earth's field: its reference is re-anchored to the final pad
  attitude at launch. The barometer corrects altitude, vertical speed and the accel bias.
  Readings whose field strength changes by more than 30 % are rejected, which guards against
  disturbance from motor casings or pyro currents.
* **Barometer is vertical-only by default** (`baroVerticalOnly`). Otherwise an unmodeled vertical
  error, such as accel scale factor or transonic pressure, gets "explained" by rotating the
  attitude, and that corrupts the horizontal position. In the simulation, letting the baro correct
  attitude made the attitude error 6x worse.
* **Latency.** Late measurements are compared with what the sensor saw at its own timestamp:
  altitude `h - v*lag`, and the attitude rotated back by `w*lag` for the magnetometer.

Limits:

* **Tilt alignment accuracy = horizontal accel bias / g.** On a still pad a sideways accel bias
  looks exactly like a tilt (no filter can separate them), e.g. 10 mg means ~0.6°. **Calibrate the
  accelerometer offsets** (6-position) before flight. The bias along the gravity axis is learned.
* **Horizontal position drifts without GPS.** A tilt error ε during the boost becomes a sideways
  velocity error of about ε × a_boost × t_burn, and position error grows from there. With 0.4°
  alignment and a 3 s, 10 g boost this is ~2 m/s, so ~70–90 m by the end of the simulated
  flight. `horizontalStd()` tracks it. `position()` accepts a GPS fix (converted to local NED)
  and removes the drift.
* **High-g:** the ICM45686 clips at ±32 g. Above that, substitute the KX134 axes into `imu()`.
* **Magnetometer calibration:** hard and soft-iron calibration matters. The default `magNoise` of
  0.02 (~1.1°) allows for some residual error.

### Outliers: soft gates

With hard gates (`softGates = false`), a filter that drifts away from the barometer while it is
confident rejects every reading. It then runs on the IMU alone until its own uncertainty has grown
enough, and jumps back in one step. In flight this happened from burnout: about 37 s on the IMU
alone, then a jump of about 200 m.

With soft gates, an outlier is still fused, but its noise is inflated so that it counts as a
gate-sized error. Each sample pulls only a little, so a drift is corrected within a second or two
and the altitude never jumps.

The cost is that a barometer error lasting more than a fraction of a second is now partly
followed. A 3 mbar, 1.5 s disturbance moves the altitude by up to ~30 m instead of ~0. The
transonic Mach lockout in the nav loop still removes the barometer completely near Mach 1.

### Tuning (`NavEKF::Config`)

| Field | Default | Meaning |
|---|---|---|
| `accelNoiseDensity` | 0.3 (m/s²)/√Hz | Flight accel uncertainty (vibration, scale, misalignment) |
| `padAccelNoiseDensity` | 0.01 | Pad accel uncertainty |
| `gyroNoiseDensity` | 0.002 (rad/s)/√Hz | Gyro noise |
| `accelBiasWalk` / `gyroBiasWalk` | 0.05 / 3e-4 | Bias drift rates. Larger keeps the biases re-learned in flight and stops the altitude uncertainty from collapsing |
| `softGates` | true | Measurements past their gate are fused at reduced weight instead of being thrown away (see below) |
| `softGateExponent` | 0.5 | How hard outliers are de-weighted: 0.5 = Huber, 1 = stronger |
| `baroNoiseMbar`, `baroGate` | 0.1 mbar, 16 | As in AltitudeEKF |
| `baroVerticalOnly` | true | Baro may not correct attitude/horizontal states |
| `magNoise` | 0.02 | 1σ of the normalized field direction |
| `magStrengthTolerance` | 0.3 | Readings whose strength differs from the pad by more than this are de-weighted in proportion (rejected if `softGates` is off) |
| `magDeclinationDeg` | 0 | Declination (east +). 0 means yaw is relative to magnetic north |
| `zeroVelocityStd`, `zeroRateStd` | 0.02 m/s, 0.002 rad/s | `stationary()` noise |
| `rocketAxis` | {1, 0, 0} | Nose direction in the IMU frame (used by `tilt()`) |
| `rocketRollRefAxis` | {0, 1, 0} | Body axis perpendicular to the nose that sets roll 0 in `rocketAngles()` |
| `launchAccel` | 2 g | Upward acceleration that switches to flight mode (or call `setInFlight(true)`) |
| `covarianceStepS` | 5 ms | Covariance propagation interval |
| `maxLagUs` | 50 ms | How late a measurement may be and still be fused |

Cost and memory:
* Per IMU sample: about 300 flops plus one sin/cos.
* Per 5 ms covariance step: about 7 k multiply-adds.
* Per baro update: about 0.5 k.
* Per magnetometer update: about 1.5 k.
* Object size: about 2.5 kB.
* Stack: the 15×15 temporaries need up to about 9.5 kB, so run it in a task with at least 16 kB (4096 words) of stack.

## AltitudeEKF: altitude and vertical velocity only

The original 3-state filter. It is cheaper and needs only the axial accelerometer and the baro,
but it assumes the rocket's long axis is vertical (1.5 % error at 10° tilt). Use NavEKF when the
attitude matters.

```cpp
#include "AltitudeEKF.h"

AltitudeEKF alt;
AltitudeEKF::Config cfg;
cfg.padTemperatureK = 273.15f + baro.getTemperature();
cfg.baroNoiseMbar   = 0.07f;                 // MS5611 at OSR 256
alt.begin(padPressureMbar, time_us_64(), cfg);

alt.accel(tUs, i.az * AltitudeEKF::G);      // every accel sample (axial axis, m/s^2)
alt.baroPressure(tUs, baro.getPressure());  // every baro reading
alt.zeroVelocity(tUs);                      // on the pad, ~100 Hz: learns the accel bias

float h = alt.altitude(), v = alt.velocity();
```

* It fuses raw pressure through the ISA barometric formula. That nonlinear measurement is what
  makes it an *extended* KF.
* On the pad it assumes a quiet accelerometer, so `zeroVelocity()` learns the bias in a few
  seconds. Above `launchAccel` it switches to the larger flight noise.
* Outliers beyond ~4σ (`baroGate`) are rejected. `baroLockoutSpeed` (off by default) distrusts
  the baro above a set speed.
* `velocity()` crosses zero at apogee.

| Field | Default | Meaning |
|---|---|---|
| `accelNoiseDensity` | 0.5 (m/s²)/√Hz | Flight accel uncertainty (includes tilt) |
| `padAccelNoiseDensity` | 0.02 | Pad accel uncertainty |
| `accelBiasWalk` | 0.02 (m/s²)/√s | Accel bias drift |
| `baroNoiseMbar` | 0.1 mbar | Baro 1σ. If it's right, `meanBaroNis()` ≈ 1 |
| `baroGate` | 16 | Outlier gate (NIS; 16 ≈ 4σ). `ekf::GATE_OFF` disables it |
| `baroLockoutSpeed` / `Scale` | 0 (off) / 100 | Distrust baro above this speed |
| `maxLagUs` | 50 ms | How late a measurement may be and still be fused |
| `padTemperatureK` | 288.15 K | Pad air temperature for the barometric formula |
| `launchAccel` | 2 g | Pad-to-flight noise switch |

## AsyncEKF<N>: for your own models

```cpp
struct MyModel {                       // N = 4, say
    float u;                           // control input, held between samples
    void propagate(ekf::Vec<4>& x, float dt);                          // x = f(x, u, dt)
    void jacobian(const ekf::Vec<4>& x, float dt, ekf::Mat<4,4>& F);   // df/dx
    void processNoise(const ekf::Vec<4>& x, float dt, ekf::Mat<4,4>& Q);
};

ekf::AsyncEKF<4> f;
MyModel model;
f.reset(x0, P0, tUs);
if (f.syncTo(model, measTimeUs, /*maxLagUs*/ 20000)) {
    ekf::UpdateResult r = f.update<2>(y, H, R, ekf::GATE_2DOF_999);   // y = z - h(x)
}
```

* Measurements can have any size `M`, and each sensor has its own `H` and `R`. Status values are
  Accepted, Gated (outlier), Deweighted (outlier fused with `setSoftGate(true)`), Stale, Numerical
  and Skipped. Each result also carries its NIS. `accepted()` is true for Accepted and Deweighted.
* `updateMasked()` lets a measurement correct only the states you choose (a "consider" update).
  It keeps the covariance correct, using the expanded Joseph form.
* `setJosephForm(false)` switches to the cheaper P − K H P covariance update, which costs N²M
  instead of 2N³. NavEKF uses it.
* Long gaps are predicted in steps of at most `setMaxPredictStep()` (default 10 ms). `P` is kept
  symmetric and floored. Any update that would produce NaN/Inf is rolled back.

## Bench test

```cpp
bool EKF_test(bool verbose = false);   // needs no hardware; call from a task with >= 16 kB stack
```

**1D (AltitudeEKF), 45 s simulated flight:** 5 s on the pad, a 3 s, 10 g boost, coast to
1746 m, and a drogue descent.
* Accel at 1600 Hz with jitter, bias, a 1 % scale error and noise.
* Baro at 200 Hz, delivered 0–3 ms late, with one 30 mbar spike.
* Results:
  * altitude error 0.10 m RMS, 0.33 m max
  * velocity error 0.59 m/s max
  * apogee matched to the sample (1746 m at 22.86 s)
  * bias learned to 0.004 m/s²
  * spike rejected

**3D (NavEKF), same profile with real attitude motion:**
* The rocket leaves an 85° rail and rolls at 0.5 rev/s while pitching over. It flips nose-down
  at apogee, then swings ±15° while spinning under the drogue.
* Sensors:
  * 3-axis accel: 0.2 m/s² axial bias, 5 mg cross-axis bias (calibrated level), 0.5 % scale error, noise
  * gyro: 0.5 °/s bias, noise
  * magnetometer: 100 Hz, −8.5° declination
  * baro: as in the 1D test
* Results, over 6 random seeds:

  | Measure | Result |
  |---|---|
  | Pad tilt error | 0.39–0.43° |
  | Pad heading error | 0.24–0.38° |
  | Gyro bias error on the pad | 0.00005 rad/s |
  | Attitude error through the whole flight | 0.51–0.60° RMS (max 0.7–1.4°) |
  | Altitude error | 0.10 m RMS |
  | Apogee timing | within 20 ms |
  | Horizontal drift (no GPS) | 65–94 m on a 428 m-range flight, within 3σ of the filter's own uncertainty |
  | Stale updates, numerical failures | 0 |

To run it on a PC (from a Visual Studio x64 prompt), compile `EKF_test.cpp`, `AltitudeEKF.cpp`
and `NavEKF.cpp` with a 3-line `main()` that calls `EKF_test(true)`.

## Possible extensions

* **GPS:** `NavEKF::position()` is ready for any local-NED fix.
* **Dual accelerometer:** fuse the KX134 above the ICM45686's ±32 g range.
* **Accel scale factors as states**, if flight data shows a vertical scale error.
