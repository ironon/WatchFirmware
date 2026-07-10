// ============================================================
//  Proximity Engine — Anchor side
//  See firmware_spec_v2.md §4.10 and §6.3
// ============================================================
#include "proximity.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include <string.h>

// ── Internal device registry entry ───────────────────────────

struct DeviceEntry {
    uint8_t mac[6];
    uint8_t type;       // 0=BLE, 1=WiFi AP
    bool    valid;
    // Weighted Welford accumulators
    float   mu;         // running mean RSSI
    float   M;          // sum of w*(x-mu_prev)*(x-mu_new)  (unnormalised variance numerator)
    float   W;          // cumulative weight
};

// ── Internal live scan cache entry ───────────────────────────

struct CacheEntry {
    uint8_t  mac[6];
    uint8_t  type;
    int8_t   rssi;
    uint32_t last_updated_ms;
    bool     valid;
};

#define CACHE_SIZE  (ANCHOR_PROX_MAX_FINGERPRINT_DEVICES * 2)

// ── Module globals ────────────────────────────────────────────

static DeviceEntry  g_devices[ANCHOR_PROX_MAX_FINGERPRINT_DEVICES];
static int          g_device_count = 0;

static CacheEntry       g_cache[CACHE_SIZE];
static SemaphoreHandle_t g_cache_mutex = nullptr;

static uint8_t g_own_mac[6];   // anchor's own BLE MAC, big-endian

static ProxScoreResult g_last_score = {0, 0};

static uint32_t g_last_persist_ms = 0;

// ── MAC helpers ───────────────────────────────────────────────

static bool mac_eq(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

// ── Device registry helpers ───────────────────────────────────

static int find_device(const uint8_t *mac, uint8_t type) {
    for (int i = 0; i < g_device_count; i++) {
        if (g_devices[i].valid && g_devices[i].type == type
            && mac_eq(g_devices[i].mac, mac))
            return i;
    }
    return -1;
}

static int find_or_add_device(const uint8_t *mac, uint8_t type) {
    int idx = find_device(mac, type);
    if (idx >= 0) return idx;
    if (g_device_count >= ANCHOR_PROX_MAX_FINGERPRINT_DEVICES) return -1;
    idx = g_device_count++;
    DeviceEntry &d = g_devices[idx];
    memcpy(d.mac, mac, 6);
    d.type  = type;
    d.valid = true;
    d.mu    = 0.0f;
    d.M     = 0.0f;
    d.W     = 0.0f;
    return idx;
}

// ── Live scan cache helpers ───────────────────────────────────

static void cache_update(const uint8_t *mac, uint8_t type, int8_t rssi) {
    uint32_t now = millis();
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (g_cache[i].valid && g_cache[i].type == type
            && mac_eq(g_cache[i].mac, mac)) {
            g_cache[i].rssi            = rssi;
            g_cache[i].last_updated_ms = now;
            return;
        }
    }
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (!g_cache[i].valid) {
            memcpy(g_cache[i].mac, mac, 6);
            g_cache[i].type            = type;
            g_cache[i].rssi            = rssi;
            g_cache[i].last_updated_ms = now;
            g_cache[i].valid           = true;
            return;
        }
    }
    // Cache full: overwrite oldest entry
    uint32_t oldest_t = millis();
    int oldest_i = 0;
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (g_cache[i].last_updated_ms < oldest_t) {
            oldest_t = g_cache[i].last_updated_ms;
            oldest_i = i;
        }
    }
    memcpy(g_cache[oldest_i].mac, mac, 6);
    g_cache[oldest_i].type            = type;
    g_cache[oldest_i].rssi            = rssi;
    g_cache[oldest_i].last_updated_ms = now;
    g_cache[oldest_i].valid           = true;
}

static int8_t cache_lookup(const uint8_t *mac, uint8_t type) {
    uint32_t now = millis();
    for (int i = 0; i < CACHE_SIZE; i++) {
        if (!g_cache[i].valid) continue;
        if (g_cache[i].type != type) continue;
        if (!mac_eq(g_cache[i].mac, mac)) continue;
        if (now - g_cache[i].last_updated_ms > ANCHOR_PROX_DEVICE_STALE_MS)
            return INT8_MIN; // stale
        return g_cache[i].rssi;
    }
    return INT8_MIN;
}

// ── Background BLE scan task ──────────────────────────────────

class ProxBLEScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        // Convert NimBLE little-endian address to big-endian (on-air) format.
        const uint8_t *native = dev->getAddress().getBase()->val;
        uint8_t mac[6];
        for (int i = 0; i < 6; i++) mac[i] = native[5 - i];
        int8_t rssi = (int8_t)dev->getRSSI();

        if (g_cache_mutex) {
            xSemaphoreTake(g_cache_mutex, portMAX_DELAY);
            cache_update(mac, 0, rssi);
            xSemaphoreGive(g_cache_mutex);
        }
    }
};

static ProxBLEScanCB g_ble_scan_cb;

static void ble_scan_task(void *) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&g_ble_scan_cb, false);
    scan->setActiveScan(false);
    scan->setInterval(100);
    scan->setWindow(99);
    scan->setMaxResults(0);

    while (true) {
        if (!scan->isScanning()) {
            scan->start(ANCHOR_PROX_BLE_SCAN_DURATION_MS, false);
        }
        vTaskDelay(pdMS_TO_TICKS(ANCHOR_PROX_BLE_SCAN_INTERVAL_MS));
    }
}

// ── Background WiFi scan task ─────────────────────────────────

static void wifi_scan_task(void *) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)ANCHOR_PROX_WIFI_SCAN_INTERVAL_S * 1000));

        wifi_scan_config_t cfg = {};
        cfg.show_hidden = false;
        cfg.scan_type   = WIFI_SCAN_TYPE_ACTIVE;
        esp_err_t err = esp_wifi_scan_start(&cfg, true); // blocking
        if (err != ESP_OK) continue;

        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count == 0) continue;

        wifi_ap_record_t *aps = (wifi_ap_record_t*)malloc(ap_count * sizeof(wifi_ap_record_t));
        if (!aps) { esp_wifi_scan_stop(); continue; }

        esp_wifi_scan_get_ap_records(&ap_count, aps);
        esp_wifi_clear_ap_list();

        if (g_cache_mutex) xSemaphoreTake(g_cache_mutex, portMAX_DELAY);
        for (int i = 0; i < ap_count; i++) {
            // WiFi BSSID is already in big-endian (on-air) order
            cache_update(aps[i].bssid, 1, (int8_t)aps[i].rssi);
        }
        if (g_cache_mutex) xSemaphoreGive(g_cache_mutex);

        free(aps);
    }
}

// ── NVS persistence ───────────────────────────────────────────

static void load_from_nvs() {
    Preferences p;
    if (!p.begin("prox", true)) return;

    uint16_t n = p.getUShort("dev_cnt", 0);
    if (n > ANCHOR_PROX_MAX_FINGERPRINT_DEVICES) n = ANCHOR_PROX_MAX_FINGERPRINT_DEVICES;

    size_t blob_sz = n * 19u; // 6 mac + 1 type + 4 mu + 4 M + 4 W
    if (blob_sz == 0) { p.end(); return; }

    uint8_t *buf = (uint8_t*)malloc(blob_sz);
    if (!buf) { p.end(); return; }

    size_t got = p.getBytes("fp_blob", buf, blob_sz);
    p.end();
    if (got != blob_sz) { free(buf); return; }

    g_device_count = 0;
    const uint8_t *ptr = buf;
    for (int i = 0; i < (int)n; i++) {
        DeviceEntry &d = g_devices[i];
        memcpy(d.mac, ptr, 6); ptr += 6;
        d.type  = *ptr++;
        d.valid = true;
        memcpy(&d.mu, ptr, 4); ptr += 4;
        memcpy(&d.M,  ptr, 4); ptr += 4;
        memcpy(&d.W,  ptr, 4); ptr += 4;
    }
    g_device_count = (int)n;
    free(buf);
    Serial.printf("[PROX] Loaded %d devices from NVS\n", g_device_count);
}

static void save_to_nvs() {
    if (g_device_count == 0) return;

    size_t blob_sz = (size_t)g_device_count * 19u;
    uint8_t *buf = (uint8_t*)malloc(blob_sz);
    if (!buf) return;

    uint8_t *ptr = buf;
    for (int i = 0; i < g_device_count; i++) {
        const DeviceEntry &d = g_devices[i];
        memcpy(ptr, d.mac, 6); ptr += 6;
        *ptr++ = d.type;
        memcpy(ptr, &d.mu, 4); ptr += 4;
        memcpy(ptr, &d.M,  4); ptr += 4;
        memcpy(ptr, &d.W,  4); ptr += 4;
    }

    Preferences p;
    if (p.begin("prox", false)) {
        p.putUShort("dev_cnt", (uint16_t)g_device_count);
        p.putBytes("fp_blob", buf, blob_sz);
        p.end();
    }
    free(buf);
    g_last_persist_ms = millis();
    Serial.printf("[PROX] Persisted %d devices to NVS\n", g_device_count);
}

// ── Public API ────────────────────────────────────────────────

void prox_init(const uint8_t own_mac_be[6]) {
    memcpy(g_own_mac, own_mac_be, 6);
    memset(g_devices, 0, sizeof(g_devices));
    memset(g_cache,   0, sizeof(g_cache));
    g_device_count = 0;
    g_cache_mutex  = xSemaphoreCreateMutex();
    load_from_nvs();
    g_last_persist_ms = millis();
    Serial.println("[PROX] Proximity engine initialised");
}

void prox_begin_tasks() {
    xTaskCreatePinnedToCore(ble_scan_task,  "prox_ble",  4096, nullptr, 1, nullptr, 0);
    xTaskCreatePinnedToCore(wifi_scan_task, "prox_wifi", 4096, nullptr, 1, nullptr, 0);
    Serial.println("[PROX] Background scan tasks started");
}

bool prox_deserialize_vector(const uint8_t *buf, size_t len, ProxScanVector &out) {
    if (!buf || len < 1) return false;
    out.count = buf[0];
    if (out.count > PROX_MAX_DEVICES) out.count = PROX_MAX_DEVICES;
    size_t expected = 1u + (size_t)out.count * 8u; // 1 count + N*(6 mac+1 type+1 rssi_wire)
    if (len < expected) { out.count = 0; return false; }
    for (int i = 0; i < out.count; i++) {
        const uint8_t *p = buf + 1 + i * 8;
        memcpy(out.devices[i].mac, p, 6);
        out.devices[i].type = p[6];
        // RSSI is encoded as rssi_wire = rssi + 128; decode back
        out.devices[i].rssi = (int8_t)((int)p[7] - 128);
    }
    return true;
}

ProxScoreResult prox_compute_score(const ProxScanVector &watch_vec) {
    ProxScoreResult res = {0, 0};

    // ── Low device count fallback ──────────────────────────────
    if (watch_vec.count < PROX_MIN_DEVICE_COUNT) {
        res.flags |= PROX_FLAG_LOW_DEVICE_COUNT;
        // Check if anchor's own MAC is in the vector
        for (int i = 0; i < watch_vec.count; i++) {
            if (watch_vec.devices[i].type == 0 &&
                mac_eq(watch_vec.devices[i].mac, g_own_mac)) {
                int8_t rssi = watch_vec.devices[i].rssi;
                res.score = (rssi >= ANCHOR_NEAR_RSSI_THRESHOLD_DBM) ? 200 : 50;
                g_last_score = res;
                return res;
            }
        }
        res.score = 50;
        g_last_score = res;
        return res;
    }

    // ── Signal A — Live Pearson Correlation ────────────────────
    // Build intersection: devices in both watch_vec and non-stale cache
    float sum_wa = 0, sum_w = 0, sum_a = 0, sum_w2 = 0, sum_a2 = 0;
    int   n_shared = 0;

    // Collect pairs
    float ws[PROX_MAX_DEVICES], as_[PROX_MAX_DEVICES];

    xSemaphoreTake(g_cache_mutex, portMAX_DELAY);
    for (int i = 0; i < watch_vec.count; i++) {
        const ProxDevice &d = watch_vec.devices[i];
        int8_t a_rssi = cache_lookup(d.mac, d.type);
        if (a_rssi == INT8_MIN) continue;
        ws[n_shared]  = (float)d.rssi;
        as_[n_shared] = (float)a_rssi;
        n_shared++;
    }
    xSemaphoreGive(g_cache_mutex);

    float rho = 0.0f;
    if (n_shared >= 2) {
        float w_mean = 0, a_mean = 0;
        for (int i = 0; i < n_shared; i++) { w_mean += ws[i]; a_mean += as_[i]; }
        w_mean /= n_shared; a_mean /= n_shared;

        float num = 0, dw = 0, da = 0;
        for (int i = 0; i < n_shared; i++) {
            float dw_i = ws[i] - w_mean;
            float da_i = as_[i] - a_mean;
            num += dw_i * da_i;
            dw  += dw_i * dw_i;
            da  += da_i * da_i;
        }
        float denom = sqrtf(dw) * sqrtf(da);
        rho = (denom > 1e-6f) ? (num / denom) : 0.0f;
        if (rho < 0.0f) rho = 0.0f;
        if (rho > 1.0f) rho = 1.0f;
    }

    // ── Signal B — Fingerprint Log-Likelihood ──────────────────
    // Count qualifying fingerprint devices
    int n_active = 0;
    float W_total = 0.0f;
    for (int i = 0; i < g_device_count; i++) {
        const DeviceEntry &d = g_devices[i];
        if (!d.valid) continue;
        W_total += d.W;
        if (d.W >= PROX_MIN_FINGERPRINT_WEIGHT) n_active++;
    }

    float L = 0.0f;
    bool  use_signal_b = (n_active >= PROX_MIN_DEVICE_COUNT);

    if (use_signal_b) {
        res.flags |= PROX_FLAG_FINGERPRINT_ACTIVE;
        float log_ll = 0.0f;
        int   n_fp   = 0;

        for (int i = 0; i < g_device_count; i++) {
            const DeviceEntry &d = g_devices[i];
            if (!d.valid || d.W < PROX_MIN_FINGERPRINT_WEIGHT) continue;

            float sigma_sq = (d.W > 1e-6f) ? (d.M / d.W) : 1.0f;
            if (sigma_sq < 1.0f) sigma_sq = 1.0f; // clamp to avoid log(0)

            // Find watch RSSI for this device; use sentinel if absent
            float w_rssi = (float)PROX_MISSING_RSSI_DBM;
            for (int j = 0; j < watch_vec.count; j++) {
                if (watch_vec.devices[j].type == d.type &&
                    mac_eq(watch_vec.devices[j].mac, d.mac)) {
                    w_rssi = (float)watch_vec.devices[j].rssi;
                    break;
                }
            }

            float diff = w_rssi - d.mu;
            log_ll += -0.5f * logf(2.0f * (float)M_PI * sigma_sq)
                     - (diff * diff) / (2.0f * sigma_sq);
            n_fp++;
        }

        if (n_fp > 0) {
            // Normalise via sigmoid scaled to expected range per device
            // Center: n_fp * (-3.0), spread: n_fp * 5.0
            float center = (float)n_fp * (-3.0f);
            float spread = (float)n_fp * 5.0f;
            float x = (log_ll - center) / spread;
            L = 1.0f / (1.0f + expf(-x));
        }
    }

    // ── Blend ──────────────────────────────────────────────────
    float alpha = expf(-W_total / PROX_ALPHA_W0);
    float score_f = alpha * rho + (1.0f - alpha) * L;
    if (score_f < 0.0f) score_f = 0.0f;
    if (score_f > 1.0f) score_f = 1.0f;

    res.score = (uint8_t)(score_f * 255.0f);
    g_last_score = res;
    return res;
}

void prox_maybe_update_fingerprint(const ProxScanVector &watch_vec,
                                    const ProxScoreResult &result) {
    float score_norm = result.score / 255.0f;
    if (score_norm < PROX_COLLECT_SCORE_THRESHOLD) return;

    // Ambiguity check: find this anchor's RSSI in the watch vector
    int8_t own_rssi = INT8_MIN;
    for (int i = 0; i < watch_vec.count; i++) {
        if (watch_vec.devices[i].type == 0 &&
            mac_eq(watch_vec.devices[i].mac, g_own_mac)) {
            own_rssi = watch_vec.devices[i].rssi;
            break;
        }
    }
    if (own_rssi == INT8_MIN) return; // own anchor not visible; can't check ambiguity

    // Check no competing anchor RSSI is within PROX_COLLECT_AMBIGUITY_MARGIN_DBM of own
    // (We can't identify other anchors without their UUIDs, so we compare all BLE devices
    //  that aren't our own MAC.  This is a conservative approximation.)
    for (int i = 0; i < watch_vec.count; i++) {
        const ProxDevice &pd = watch_vec.devices[i];
        if (pd.type != 0) continue;
        if (mac_eq(pd.mac, g_own_mac)) continue;
        int diff = (int)pd.rssi - (int)own_rssi;
        if (diff < 0) diff = -diff;
        if (diff < PROX_COLLECT_AMBIGUITY_MARGIN_DBM) return; // ambiguous
    }

    float w_n = score_norm;

    // Update each device in the watch vector
    for (int i = 0; i < watch_vec.count; i++) {
        const ProxDevice &pd = watch_vec.devices[i];
        int idx = find_or_add_device(pd.mac, pd.type);
        if (idx < 0) continue;
        DeviceEntry &d = g_devices[idx];
        float x = (float)pd.rssi;
        float W_new = d.W + w_n;
        float mu_prev = d.mu;
        d.mu = mu_prev + (w_n / W_new) * (x - mu_prev);
        d.M  = d.M + w_n * (x - mu_prev) * (x - d.mu);
        d.W  = W_new;
    }

    // For registered devices NOT in the watch vector, substitute missing sentinel
    float w_absent = w_n * 0.5f;
    for (int i = 0; i < g_device_count; i++) {
        DeviceEntry &d = g_devices[i];
        if (!d.valid) continue;
        bool in_vec = false;
        for (int j = 0; j < watch_vec.count; j++) {
            if (watch_vec.devices[j].type == d.type &&
                mac_eq(watch_vec.devices[j].mac, d.mac)) {
                in_vec = true; break;
            }
        }
        if (!in_vec) {
            float x = (float)PROX_MISSING_RSSI_DBM;
            float W_new = d.W + w_absent;
            float mu_prev = d.mu;
            d.mu = mu_prev + (w_absent / W_new) * (x - mu_prev);
            d.M  = d.M + w_absent * (x - mu_prev) * (x - d.mu);
            d.W  = W_new;
        }
    }

    // Throttled NVS persist
    if (millis() - g_last_persist_ms > (uint32_t)PROX_NVS_PERSIST_INTERVAL_S * 1000UL) {
        save_to_nvs();
    }
}

bool prox_load_fingerprint_blob(const uint8_t *blob, size_t len) {
    if (!blob || len < 2) return false;

    uint16_t n;
    memcpy(&n, blob, 2);
    if (n > ANCHOR_PROX_MAX_FINGERPRINT_DEVICES) return false;

    size_t expected = 2u + (size_t)n * 19u; // 6+1+4+4+4 per entry
    if (len < expected) return false;

    memset(g_devices, 0, sizeof(g_devices));
    g_device_count = 0;

    const uint8_t *ptr = blob + 2;
    for (int i = 0; i < (int)n; i++) {
        DeviceEntry &d = g_devices[i];
        memcpy(d.mac, ptr, 6); ptr += 6;
        d.type  = *ptr++;
        d.valid = true;
        float mu_in, sigma_sq_in, W_in;
        memcpy(&mu_in,       ptr, 4); ptr += 4;
        memcpy(&sigma_sq_in, ptr, 4); ptr += 4;
        memcpy(&W_in,        ptr, 4); ptr += 4;
        d.mu = mu_in;
        // Convert sigma_sq back to M: M = sigma_sq * W (biased estimator approximation)
        d.M  = (W_in > 1e-6f) ? (sigma_sq_in * W_in) : 0.0f;
        d.W  = W_in;
    }
    g_device_count = (int)n;

    save_to_nvs();
    Serial.printf("[PROX] Loaded fingerprint blob: %d devices\n", g_device_count);
    return true;
}

void prox_get_last_score(uint8_t out[2]) {
    out[0] = g_last_score.score;
    out[1] = g_last_score.flags;
}

void prox_persist_now() {
    save_to_nvs();
}
