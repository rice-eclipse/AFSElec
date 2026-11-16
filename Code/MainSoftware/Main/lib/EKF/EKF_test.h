/**
 * EKF_test.h - Bench test for the EKF library. Needs no hardware: it simulates a flight.
 *
 * Checks the matrix math, then flies a simulated rocket (pad -> 10 g boost -> coast -> apogee ->
 * drogue descent) through AltitudeEKF with asynchronous, noisy sensors:
 *   - accelerometer at 1600 Hz with timing jitter, bias, 1 % scale error and noise
 *   - barometer at 200 Hz, delivered 0-3 ms late (out of order with the accelerometer), with noise
 *     and one deliberate 30 mbar spike that must be rejected
 * and checks bias estimation on the pad, altitude/velocity error, apogee timing, filter
 * consistency (mean NIS) and outlier rejection.
 *
 * Then flies a 3D simulated rocket through NavEKF: rail at 85 deg, 0.5 rev/s roll, pitch-over,
 * a nose-down flip at apogee and a swinging, spinning drogue descent, with a 3-axis accel
 * (bias, scale error, noise), gyro (0.5 deg/s bias, noise), magnetometer and the same baro.
 * Checks pad alignment (tilt, heading, gyro bias), attitude / altitude / velocity error through
 * the flight, horizontal drift against the filter's own uncertainty, apogee, outliers and NIS.
 *
 * Plain C++ (stdio + math only), so it runs on the board and on a PC.
 * Takes a few seconds on the RP2350 (the 3D part dominates); call it from a task with >= 16 kB stack.
 */
#pragma once

bool EKF_test(bool verbose = false);
