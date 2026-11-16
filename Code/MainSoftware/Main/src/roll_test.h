/**
 * roll_test.h - In-flight roll-effectiveness test: open-loop doublets on the fin tabs during coast.
 *
 * Purpose: measure how roll acceleration depends on tab deflection at different speeds (dynamic
 * pressure). Afterwards, fit for each doublet
 *     roll_accel = L_delta * deflection + L_p * roll_rate + L_0
 * and plot L_delta against qbar_pa (expected: a straight line through 0).
 *
 * Schedule (all settings in config.h, "Roll test"):
 *   - starts ROLL_TEST_START_MS after burnout (the CRUISING state), so burnout transients settle
 *   - doublets: +amp for ROLL_TEST_STEP_MS, -amp for ROLL_TEST_STEP_MS, then center for
 *     ROLL_TEST_REST_MS (the rest shows the roll damping and the bias); amp cycles through
 *     ROLL_TEST_AMPS_DEG
 *   - ends (tab centered) when the vertical speed falls below ROLL_TEST_STOP_VS_MPS, after
 *     ROLL_TEST_MAX_MS, or when the state leaves CRUISING, whichever comes first
 *     (in bench mode, NAV_SERIAL_STATE_CMD + NAV_SERIAL_STATE_MANUAL, the climb-rate stop is skipped)
 *   - aborts for good (tab centered) if |roll rate| exceeds ROLL_TEST_ABORT_DPS
 * It is open loop on purpose: the commands never depend on the measured motion (except the abort),
 * so the input and the response stay uncorrelated and the fit is clean.
 *
 * Plain C++ with no Pico SDK dependency: the nav task calls update() on every IMU sample and
 * writes the returned deflection to the servo.
 */
#pragma once

#include <stdint.h>
#include "flight_state.h"

class RollTest {
public:
    enum class Status : uint8_t {
        Waiting = 0,   // before the test window (pad, powered ascent, first ms of coast)
        Running = 1,
        Done    = 2,   // finished normally; tab centered for the rest of the flight
        Aborted = 3,   // roll rate limit hit; tab centered for the rest of the flight
    };

    /**
     * @param tUs        now
     * @param state      current flight state
     * @param stateSinceUs time the current state was entered (= burnout while CRUISING)
     * @param vertSpeed  EKF vertical speed, m/s, up positive
     * @param rollRateDps roll rate about the rocket's nose axis, deg/s
     * @return tab deflection to command, deg from the servo's center (0 = centered)
     */
    float update(uint64_t tUs, FlightState state, uint64_t stateSinceUs, float vertSpeed, float rollRateDps);

    Status status() const { return status_; }
    uint8_t doublet() const { return doublet_; }   // 1, 2, ... while running (0 before)
    float deflection() const { return deflection_; }

private:
    float finish(Status s);
    Status status_ = Status::Waiting;
    uint8_t doublet_ = 0;
    float deflection_ = 0.0f;
};

const char* rollTestStatusName(RollTest::Status s);
