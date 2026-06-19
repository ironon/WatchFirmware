#pragma once
// ============================================================
//  Watch Proximity Transport / Orchestration
//
//  These are the watch-side TRANSPORT and INTERPRETATION pieces that the
//  shared proximity engine (../proximity_engine) does NOT provide:
//    - the directed BLE GATT query to an anchor (connect/write/read/disconnect)
//    - score interpretation (NEAR / AWAY / AMBIGUOUS) + threshold rule
//    - the WiFi-AP feeding step that primes the engine's scan buffer
//
//  The proximity ALGORITHM (vector assembly, serialization, scoring) lives in
//  the shared engine and is reached through "proximity.h".
// ============================================================
#include <Arduino.h>
#include <stdint.h>
#include "proximity.h"

// When a proximity GATT connection cannot be established, a recent advertisement
// RSSI at or below this level is treated as strong evidence the watch is FAR
// from the anchor (connection establishment fails ~-88 dBm, well before ad
// reception does, so a failed connect + weak ad ≈ out of range).
// (Re-homed from the old src/proximity.h — not provided by the shared engine.)
#define PROX_FAR_RSSI_THRESHOLD_DBM          -85

// MTU we request before connecting to an anchor.
// (Re-homed from the old src/proximity.h.)
#define BLE_REQUESTED_MTU                    512

// Proximity decision returned by prox_interpret_score().
// (Re-homed from the old src/proximity.h — not provided by the shared engine.)
enum ProxProximity { PROX_NEAR, PROX_AWAY, PROX_AMBIGUOUS };

// Refresh / reuse the cached WiFi AP list (subject to the existing cache
// interval and the "skip when WiFi not associated" rule) and feed each AP into
// the shared engine via prox_ingest_scan_result(bssid, PROX_TYPE_WIFI, rssi).
//
// Call this immediately BEFORE prox_build_scan_vector() so the engine's scan
// buffer (which it drains on each build) contains the WiFi APs in addition to
// the BLE devices fed by the scan callback. Mirrors the WiFi logic that used to
// live inside the old prox_build_scan_vector().
void prox_feed_wifi_aps();

// Connect to the anchor at bleMac_be (using addr_type, the NimBLE address type
// as it was advertised — PUBLIC vs RANDOM must match or the connect fails),
// write the scan vector, read the score. Blocks until complete or timed out.
// Returns true on success; result is valid only if true is returned.
bool prox_query_anchor(const uint8_t bleMac_be[6],
                       uint8_t addr_type,
                       const ProxScanVector &vec,
                       ProxScoreResult &result);

// Interpret a raw score as NEAR / AWAY / AMBIGUOUS using the threshold rule.
ProxProximity prox_interpret_score(uint8_t score);
