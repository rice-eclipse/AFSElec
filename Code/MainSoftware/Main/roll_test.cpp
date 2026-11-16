/**
 * roll_test.cpp - In-flight roll-effectiveness test (see roll_test.h).
 */
#include "roll_test.h"

#include <math.h>
#include "config.h"

const char* rollTestStatusName(RollTest::Status s)
{
    switch (s) {
        case RollTest::Status::Waiting: return "WAITING";
        case RollTest::Status::Running: return "RUNNING";
        case RollTest::Status::Done:    return "DONE";
        case RollTest::Status::Aborted: return "ABORTED";
    }
    return "?";
}

float RollTest::finish(Status s)
{
    status_ = s;
    deflection_ = 0.0f;
    return 0.0f;
}

float RollTest::update(uint64_t tUs, FlightState state, uint64_t stateSinceUs, float vertSpeed, float rollRateDps)
{
    if (status_ == Status::Done || status_ == Status::Aborted) return 0.0f;

    if (state != FlightState::Cruising) {
        // Before coast: keep waiting. After coast (descent, landed): the window is over.
        if (state == FlightState::Descent || state == FlightState::Landed) return finish(Status::Done);
        return 0.0f;
    }

    const uint32_t tMs = (uint32_t)((tUs - stateSinceUs) / 1000);   // since burnout
    if (fabsf(rollRateDps) > ROLL_TEST_ABORT_DPS) return finish(Status::Aborted);
    if (tMs < ROLL_TEST_START_MS) return 0.0f;
    // Bench (manual "state" mode): the board isn't climbing, so the climb-rate stop is skipped and the
    // test runs until ROLL_TEST_MAX_MS
    constexpr bool bench = NAV_SERIAL_STATE_CMD && NAV_SERIAL_STATE_MANUAL;
    if (tMs >= ROLL_TEST_MAX_MS || (!bench && vertSpeed < ROLL_TEST_STOP_VS_MPS)) return finish(Status::Done);

    // Doublet number k, and where we are inside it
    constexpr uint32_t cycleMs = 2 * ROLL_TEST_STEP_MS + ROLL_TEST_REST_MS;
    constexpr uint32_t ampCount = sizeof(ROLL_TEST_AMPS_DEG) / sizeof(ROLL_TEST_AMPS_DEG[0]);
    const uint32_t k = (tMs - ROLL_TEST_START_MS) / cycleMs;
    const uint32_t inCycle = (tMs - ROLL_TEST_START_MS) % cycleMs;
    const float amp = ROLL_TEST_AMPS_DEG[k % ampCount];

    status_ = Status::Running;
    doublet_ = (uint8_t)(k + 1 > 255 ? 255 : k + 1);
    deflection_ = inCycle < ROLL_TEST_STEP_MS ? amp : inCycle < 2 * ROLL_TEST_STEP_MS ? -amp : 0.0f;
    return deflection_;
}
