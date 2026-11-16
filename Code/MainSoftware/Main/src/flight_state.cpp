/**
 * flight_state.cpp - Rocket flight state machine (see flight_state.h).
 */
#include "flight_state.h"

#include <math.h>
#include "config.h"

const char* flightStateName(FlightState s)
{
    switch (s) {
        case FlightState::Launchpad:     return "LAUNCHPAD";
        case FlightState::PoweredAscent: return "POWERED_ASCENT";
        case FlightState::Cruising:      return "CRUISING";
        case FlightState::Descent:       return "DESCENT";
        case FlightState::Landed:        return "LANDED";
    }
    return "?";
}

float machNumber(float speedMps, float altM, float padTempK)
{
    // Standard-atmosphere lapse rate from the pad temperature; a = sqrt(gamma * R * T)
    float tK = padTempK - 0.0065f * (altM > 0.0f ? altM : 0.0f);
    if (tK < 216.65f) tK = 216.65f;   // tropopause
    return speedMps / sqrtf(1.4f * 287.05f * tK);
}

bool FlightStateMachine::Hold::update(bool cond, uint64_t tUs, uint32_t holdMs)
{
    if (!cond) { active = false; return false; }
    if (!active) { active = true; sinceUs = tUs; }
    return tUs - sinceUs >= (uint64_t)holdMs * 1000;
}

void FlightStateMachine::enter(FlightState s, uint64_t tUs)
{
    state_ = s;
    stateSinceUs_ = tUs;
    launch_.reset(); launchBackup_.reset(); burnout_.reset(); apogee_.reset(); landed_.reset(); impact_.reset();
}

bool FlightStateMachine::advance(uint64_t tUs)
{
    if (state_ == FlightState::Landed) return false;
    enter((FlightState)((uint8_t)state_ + 1), tUs);
    return true;
}

bool FlightStateMachine::update(uint64_t tUs, const FlightInputs& in)
{
    const FlightState before = state_;
    const uint64_t inStateUs = tUs - stateSinceUs_;
    if (state_ != FlightState::Launchpad && in.altM > maxAlt_) maxAlt_ = in.altM;

    switch (state_) {
    case FlightState::Launchpad: {
        const bool accel = launch_.update(in.accelG > LAUNCH_ACCEL_G, tUs, LAUNCH_HOLD_MS);
        const bool backup = launchBackup_.update(in.altM > LAUNCH_BACKUP_ALT_M &&
                                                 in.vertSpeed > LAUNCH_BACKUP_VS_MPS, tUs, LAUNCH_BACKUP_HOLD_MS);
        if (accel || backup) {
            maxAlt_ = in.altM;
            enter(FlightState::PoweredAscent, tUs);
        }
        break;
    }
    case FlightState::PoweredAscent:
        if (inStateUs >= (uint64_t)MIN_BURN_MS * 1000 &&
            burnout_.update(in.axialG < BURNOUT_AXIAL_G, tUs, BURNOUT_HOLD_MS))
            enter(FlightState::Cruising, tUs);
        else if (apogee_.update(in.vertSpeed < 0.0f, tUs, APOGEE_HOLD_MS))   // missed burnout: still catch apogee
            enter(FlightState::Descent, tUs);
        break;

    case FlightState::Cruising:
        if (inStateUs >= (uint64_t)MIN_COAST_MS * 1000 &&
            (apogee_.update(in.vertSpeed < 0.0f, tUs, APOGEE_HOLD_MS) ||
             in.altM < maxAlt_ - APOGEE_ALT_DROP_M))
            enter(FlightState::Descent, tUs);
        break;

    case FlightState::Descent: {
        // Settled on the ground: slow, ~1 g and not rotating, all held for LANDED_HOLD_MS
        const bool still = fabsf(in.vertSpeed) < LANDED_VS_MPS &&
                           fabsf(in.accelG - 1.0f) < LANDED_ACCEL_TOL_G &&
                           in.gyroDps < LANDED_GYRO_DPS;
        const bool settled = landed_.update(still, tUs, LANDED_HOLD_MS);
        // Or the touchdown impact itself, once the parachute opening shock is long past
        const bool impact = inStateUs >= (uint64_t)LANDED_IMPACT_AFTER_MS * 1000 &&
                            impact_.update(in.accelG > LANDED_IMPACT_G, tUs, LANDED_IMPACT_HOLD_MS);
        if (settled || impact)
            enter(FlightState::Landed, tUs);
        break;
    }
    case FlightState::Landed:
        break;   // final state
    }
    return state_ != before;
}
