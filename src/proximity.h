#pragma once
// ============================================================
//  Proximity Engine — Watch side
//  Shared structs and watch-side API.  See firmware_spec_v2.md §5.4 / §6.3
// ============================================================
#include <Arduino.h>
#include <stdint.h>

// ── Constants (must match anchor values) ─────────────────────
#define PROX_MAX_DEVICES                      60
#define PROX_MIN_DEVICE_COUNT                  8
#define PROX_MIN_MTU_BYTES                   256
#define PROX_MISSING_RSSI_DBM              (-100)
#define PROX_MIN_FINGERPRINT_WEIGHT          5.0f
#define PROX_ALPHA_W0                     2000.0f
#define PROX_COLLECT_SCORE_THRESHOLD         0.75f
#define PROX_COLLECT_AMBIGUITY_MARGIN_DBM     10
#define ANCHOR_NEAR_RSSI_THRESHOLD_DBM       -70
#define PROX_CONFIDENCE_THRESHOLD_U8         170
#define BLE_REQUESTED_MTU                    512

// ProxScoreResult.flags bits
#define PROX_FLAG_FINGERPRINT_ACTIVE  0x01
#define PROX_FLAG_LOW_DEVICE_COUNT    0x02

// Internal BLE device cache size (all observed BLE devices, not just anchors)
#define PROX_BLE_CACHE_SIZE  128

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

// Proximity decision returned by prox_interpret_score()
enum ProxProximity { PROX_NEAR, PROX_AWAY, PROX_AMBIGUOUS };

// ── Watch-side API ────────────────────────────────────────────

// Initialise the watch-side BLE device cache.  Call once in setup().
void prox_init_watch();

// Update the BLE device cache with a single observation.
// mac_be: 6-byte BLE MAC in big-endian (on-air) order.
// Called from the BLE scan callback for every observed device.
void prox_update_ble_cache(const uint8_t mac_be[6], uint8_t type, int8_t rssi);

// Build a ProxScanVector from the current BLE device cache plus a fresh
// WiFi AP scan.  Fills *out and returns the number of entries inserted.
// Truncates to PROX_MAX_DEVICES keeping the strongest RSSI values.
int prox_build_scan_vector(ProxScanVector &out);

// Serialise a ProxScanVector into buf using the §6.3.1 wire format.
// Returns the number of bytes written, or 0 on error.
// buf must be at least 1 + count*8 bytes.
size_t prox_serialize_vector(const ProxScanVector &vec, uint8_t *buf, size_t buf_len);

// Connect to the anchor at bleMac_be, write the scan vector, read the score.
// Blocks until the operation completes or times out.
// Returns true on success; result is valid only if true is returned.
bool prox_query_anchor(const uint8_t bleMac_be[6],
                       const ProxScanVector &vec,
                       ProxScoreResult &result);

// Interpret a raw score as NEAR / AWAY / AMBIGUOUS using the threshold rule.
ProxProximity prox_interpret_score(uint8_t score);
