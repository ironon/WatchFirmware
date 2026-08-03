// prox_v0_dummy.h — the "v0" stub proximity engine.
//
// PURPOSE: bisect a failing enforcement run. When a commitment does not fire,
// the cause is either (a) the proximity engine reported the wrong answer, or
// (b) the enforcement logic downstream of it mishandled the right answer. Those
// two look identical from the outside — nothing beeps — and chasing (a) when the
// bug is in (b) wastes a whole evening of hardware time.
//
// With V0_ENABLED, the proximity verdict is replaced by a constant chosen to
// make every criterion FAIL. If enforcement still does not fire, the engine is
// exonerated and the bug is downstream. If it does fire correctly, enforcement
// is exonerated and the bug is in the engine or its inputs.
//
// WHAT IT DOES NOT BYPASS, deliberately. This stubs the SENSOR, not the state
// machine — otherwise it would prove nothing about the logic under test:
//   * donning grace (§5.4.4) still short-circuits to met while it is running;
//   * phoneAway's PHONE_AWAY_TOLERANCE_S still has to elapse before enforcing;
//   * escalation profiles, step timing, anchor beeping, and the poll cadence are
//     all untouched.
// So a V0 run exercises the real enforcement path end to end, with only the
// proximity answer forced.
//
// It also skips the scan and the GATT connect entirely, which is a second
// benefit: no radio traffic means no Option A central/peripheral collisions and
// no scan-timing variance, so a V0 run is deterministic and repeats exactly.
//
// This lives in the WATCH firmware, not in proximity_engine/, on purpose: the
// engine is a shared library that the anchor also links, and a flag that forces
// noncompliance has no business being reachable from there.
//
// >>> SET BACK TO 0 BEFORE ANY REAL USE. <<<
// With this on, every commitment is permanently noncompliant, so the watch will
// alarm for the entire duration of every window and never stop. setup() prints a
// loud banner every boot so a forgotten flag cannot be mistaken for a bug.

#ifndef PROX_V0_DUMMY_H
#define PROX_V0_DUMMY_H

#define V0_ENABLED 0

#if V0_ENABLED

// The forced verdict per criterion — whichever value makes that criterion's own
// test fail. These are the inverses of the checks in
// is_enforcement_condition_met(), so if those change, change these with them:
//
//   STAY_NEAR   returns (prox == PROX_NEAR)  -> force AWAY
//   GET_AWAY    returns (prox == PROX_AWAY)  -> force NEAR
//   PHONE_AWAY  near_phone = undocked || (prox == PROX_NEAR) -> force NEAR
//
// AMBIGUOUS is deliberately never used: it resolves to the *compliant* side for
// every criterion (that is the fail-safe), so it would produce the opposite of
// what this stub is for.
static inline ProxProximity v0_forced_proximity(Criteria c) {
    switch (c) {
        case STAY_NEAR: return PROX_AWAY;
        case GET_AWAY:  return PROX_NEAR;
        default:        return PROX_NEAR;   // PHONE_AWAY: forces near_phone
    }
}

#endif  // V0_ENABLED
#endif  // PROX_V0_DUMMY_H
