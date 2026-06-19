// ============================================================
//  Proximity Engine — Watch platform seam
//
//  The shared proximity engine (../proximity_engine) declares these three
//  functions `extern "C"` and calls them for its clock and NVS needs. The
//  platform layer (this file) provides the concrete implementations.
//
//  See proximity.h "Platform seam" section.
// ============================================================
#include "proximity.h"
#include <Arduino.h>
#include <Preferences.h>

// Dedicated NVS namespace for the proximity engine's own blobs (fingerprint /
// co-location config). Kept separate from the watch's existing namespaces
// ("watch", "schedule", "dbg") to avoid key collisions.
static const char* PROX_NVS_NAMESPACE = "proxeng";

extern "C" {

uint32_t prox_platform_now_ms(void) {
    return millis();
}

// Load a stored blob by key. Returns 1 on success and sets *out_len to the
// number of bytes copied into buf (capped at cap). Returns 0 if the key is
// missing or on any failure (the engine only consumes the data when the
// returned length matches the expected struct size).
int prox_platform_nvs_load(const char* key, void* buf, size_t cap, size_t* out_len) {
    Preferences p;
    if (!p.begin(PROX_NVS_NAMESPACE, true /* read-only */)) {
        return 0;
    }
    if (!p.isKey(key)) {
        p.end();
        return 0;
    }
    size_t n = p.getBytes(key, buf, cap);
    p.end();
    if (n == 0) {
        return 0;
    }
    if (out_len) *out_len = n;
    return 1;
}

// Save a blob under key. Returns 1 on success, 0 on failure.
int prox_platform_nvs_save(const char* key, const void* buf, size_t len) {
    Preferences p;
    if (!p.begin(PROX_NVS_NAMESPACE, false /* read-write */)) {
        return 0;
    }
    size_t n = p.putBytes(key, buf, len);
    p.end();
    return (n == len) ? 1 : 0;
}

} // extern "C"
