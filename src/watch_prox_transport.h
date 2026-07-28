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

// PROX_FAR_RSSI_THRESHOLD_DBM (the "failed connect + weak advertisement ⇒ far"
// level) now lives in proximity.h: as of engine v2.1 the rule is a log-LR fed to
// the HMM through prox_note_connect_failure(), so the engine owns the constant.

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
// out_dock (optional, phoneAway/Mode B): if non-null, also reads the anchor's
// Dock Status characteristic and sets *out_dock to 1 (docked), 0 (undocked), or
// -1 (unknown — char absent or read failed; caller treats unknown as docked).
// calib_phase: 0xFF (default) leaves the anchor's Calibration Mode untouched
// (enforcement path). 0/1/2 (NONE/INSIDE/EDGE) is written to the anchor's …000F
// char (write-with-response) on the same connection, before the vector, so the
// anchor routes training by phase (calibration-v2). out_near_threshold
// (optional): receives the anchor's calibrated per-anchor cutoff from the 3rd
// byte of the score char (0 = uncalibrated).
bool prox_query_anchor(const uint8_t bleMac_be[6],
                       uint8_t addr_type,
                       const ProxScanVector &vec,
                       ProxScoreResult &result,
                       int8_t *out_dock = nullptr,
                       uint8_t calib_phase = 0xFF,
                       uint8_t *out_near_threshold = nullptr);

// Calibration-v2 FINALIZE: connect to the anchor, write …000F = FINALIZE (3),
// read back the result frame (0x01 [thr][inside_n u16][edge_n u16][conf]) and
// disconnect. Returns true on success with the parsed fields.
bool prox_finalize_anchor(const uint8_t bleMac_be[6],
                          uint8_t addr_type,
                          uint8_t *out_thr,
                          uint16_t *out_inside_n,
                          uint16_t *out_edge_n,
                          uint8_t *out_confidence);

// Interpret a raw score as NEAR / AWAY / AMBIGUOUS. When near_threshold is
// non-zero, the per-anchor calibrated cutoff is used (NEAR ≥ thr; AMBIGUOUS in
// [thr - PROX_NEAR_HYST_U8, thr); else AWAY). Otherwise the global rule applies.
ProxProximity prox_interpret_score(uint8_t score, uint8_t near_threshold = 0);
