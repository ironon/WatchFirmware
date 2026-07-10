#pragma once
// ============================================================
//  Proximity Engine — Anchor side
//  Shared structs and anchor-side API.  See firmware_spec_v2.md §4.10
// ============================================================
#include <Arduino.h>
#include <stdint.h>

// ── Constants ────────────────────────────────────────────────
#define PROX_MAX_DEVICES                      60
#define PROX_MIN_DEVICE_COUNT                  8
#define PROX_MIN_MTU_BYTES                   256
#define PROX_MISSING_RSSI_DBM              (-100)
#define PROX_MIN_FINGERPRINT_WEIGHT          5.0f
#define PROX_ALPHA_W0                     2000.0f
#define PROX_COLLECT_SCORE_THRESHOLD         0.75f
#define PROX_COLLECT_AMBIGUITY_MARGIN_DBM     10
#define PROX_NVS_PERSIST_INTERVAL_S          300
#define ANCHOR_PROX_BLE_SCAN_INTERVAL_MS    2000
#define ANCHOR_PROX_BLE_SCAN_DURATION_MS     500
#define ANCHOR_PROX_WIFI_SCAN_INTERVAL_S      30
#define ANCHOR_PROX_DEVICE_STALE_MS        10000
#define ANCHOR_PROX_MAX_FINGERPRINT_DEVICES  128
#define ANCHOR_NEAR_RSSI_THRESHOLD_DBM       -70
#define PROX_CONFIDENCE_THRESHOLD_U8         170

// ProxScoreResult.flags bits
#define PROX_FLAG_FINGERPRINT_ACTIVE  0x01
#define PROX_FLAG_LOW_DEVICE_COUNT    0x02

// ── Shared data structures ────────────────────────────────────
struct ProxDevice {
    uint8_t mac[6];   // BLE MAC (big-endian / on-air) or WiFi BSSID
    uint8_t type;     // 0 = BLE device, 1 = WiFi AP
    int8_t  rssi;     // signal strength in dBm
};

struct ProxScanVector {
    ProxDevice devices[PROX_MAX_DEVICES];
    uint8_t    count;
};

struct ProxScoreResult {
    uint8_t score;  // 0 = definitely away, 255 = definitely near
    uint8_t flags;
};

// ── Anchor-side API ───────────────────────────────────────────

// Call once in setup() after NimBLEDevice::init().
// own_mac_be: this anchor's BLE MAC in big-endian (on-air) byte order.
// Loads device registry + fingerprint from NVS.
void prox_init(const uint8_t own_mac_be[6]);

// Launch background BLE and WiFi scan FreeRTOS tasks.
// Call after NimBLE is initialised and WiFi mode is set.
void prox_begin_tasks();

// Deserialise a proximity vector from the §6.3.1 wire format.
// Returns false if buf is malformed or zero-length.
bool prox_deserialize_vector(const uint8_t *buf, size_t len, ProxScanVector &out);

// Compute a proximity score for the given watch scan vector (§4.10.3).
// Thread-safe: acquires the live-cache mutex internally.
ProxScoreResult prox_compute_score(const ProxScanVector &watch_vec);

// Conditionally accept a sample into the fingerprint (§4.10.4).
void prox_maybe_update_fingerprint(const ProxScanVector &watch_vec,
                                    const ProxScoreResult &result);

// Replace the current fingerprint with an app-provided blob (§4.10.5 / §6.3.2).
// Returns false on parse error.
bool prox_load_fingerprint_blob(const uint8_t *blob, size_t len);

// Copy the last computed score into a 2-byte buffer [score, flags]
// for writing to the Proximity Score characteristic.
void prox_get_last_score(uint8_t out[2]);

// Force an immediate NVS persist (e.g. before deep sleep).
void prox_persist_now();
