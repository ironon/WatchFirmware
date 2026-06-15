// ============================================================
//  Proximity Engine — Watch side
//  See firmware_spec_v2.md §5.4 and §6.3
// ============================================================
#include "proximity.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

// Anchor proximity GATT UUIDs (must match anchor firmware)
#define ANCHOR_SERVICE_UUID           "4A0F0001-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_VECTOR_CHAR_UUID  "4A0F0008-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_SCORE_CHAR_UUID   "4A0F0009-F8CE-11EE-8001-020304050607"

// ── BLE device cache ─────────────────────────────────────────

struct BLECacheEntry {
    uint8_t  mac[6];         // big-endian (on-air)
    uint8_t  type;           // always 0 for BLE cache
    int8_t   rssi;
    uint32_t last_seen_ms;
    bool     valid;
};

static BLECacheEntry g_ble_cache[PROX_BLE_CACHE_SIZE];

static bool mac_eq(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

// ── Public API ────────────────────────────────────────────────

void prox_init_watch() {
    memset(g_ble_cache, 0, sizeof(g_ble_cache));
}

void prox_update_ble_cache(const uint8_t mac_be[6], uint8_t type, int8_t rssi) {
    uint32_t now = millis();
    // Update existing entry
    for (int i = 0; i < PROX_BLE_CACHE_SIZE; i++) {
        if (g_ble_cache[i].valid && g_ble_cache[i].type == type
            && mac_eq(g_ble_cache[i].mac, mac_be)) {
            g_ble_cache[i].rssi         = rssi;
            g_ble_cache[i].last_seen_ms = now;
            return;
        }
    }
    // New entry: find empty slot
    for (int i = 0; i < PROX_BLE_CACHE_SIZE; i++) {
        if (!g_ble_cache[i].valid) {
            memcpy(g_ble_cache[i].mac, mac_be, 6);
            g_ble_cache[i].type         = type;
            g_ble_cache[i].rssi         = rssi;
            g_ble_cache[i].last_seen_ms = now;
            g_ble_cache[i].valid        = true;
            return;
        }
    }
    // Cache full: overwrite oldest
    uint32_t oldest_t = millis();
    int oldest_i = 0;
    for (int i = 0; i < PROX_BLE_CACHE_SIZE; i++) {
        if (g_ble_cache[i].last_seen_ms < oldest_t) {
            oldest_t = g_ble_cache[i].last_seen_ms;
            oldest_i = i;
        }
    }
    memcpy(g_ble_cache[oldest_i].mac, mac_be, 6);
    g_ble_cache[oldest_i].type         = type;
    g_ble_cache[oldest_i].rssi         = rssi;
    g_ble_cache[oldest_i].last_seen_ms = now;
    g_ble_cache[oldest_i].valid        = true;
}

int prox_build_scan_vector(ProxScanVector &out) {
    out.count = 0;

    // Collect BLE devices from cache (entries seen within 10 seconds)
    uint32_t now = millis();
    for (int i = 0; i < PROX_BLE_CACHE_SIZE && out.count < PROX_MAX_DEVICES; i++) {
        if (!g_ble_cache[i].valid) continue;
        if (now - g_ble_cache[i].last_seen_ms > 10000) continue; // stale
        ProxDevice &pd = out.devices[out.count++];
        memcpy(pd.mac, g_ble_cache[i].mac, 6);
        pd.type = 0; // BLE
        pd.rssi = g_ble_cache[i].rssi;
    }

    // Collect WiFi APs via a fresh scan
    wifi_scan_config_t cfg = {};
    cfg.show_hidden = false;
    cfg.scan_type   = WIFI_SCAN_TYPE_ACTIVE;
    if (esp_wifi_scan_start(&cfg, true) == ESP_OK) { // blocking scan
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count > 0) {
            wifi_ap_record_t *aps = (wifi_ap_record_t*)malloc(ap_count * sizeof(wifi_ap_record_t));
            if (aps) {
                esp_wifi_scan_get_ap_records(&ap_count, aps);
                esp_wifi_clear_ap_list();
                for (int i = 0; i < ap_count && out.count < PROX_MAX_DEVICES; i++) {
                    ProxDevice &pd = out.devices[out.count++];
                    memcpy(pd.mac, aps[i].bssid, 6); // BSSID already big-endian
                    pd.type = 1; // WiFi AP
                    pd.rssi = (int8_t)aps[i].rssi;
                }
                free(aps);
            }
        }
    }

    // If over PROX_MAX_DEVICES, keep the strongest RSSI entries
    if (out.count > PROX_MAX_DEVICES) {
        // Simple selection sort on the top PROX_MAX_DEVICES by RSSI
        for (int i = 0; i < PROX_MAX_DEVICES; i++) {
            int best = i;
            for (int j = i + 1; j < out.count; j++) {
                if (out.devices[j].rssi > out.devices[best].rssi) best = j;
            }
            if (best != i) {
                ProxDevice tmp = out.devices[i];
                out.devices[i] = out.devices[best];
                out.devices[best] = tmp;
            }
        }
        out.count = PROX_MAX_DEVICES;
    }

    return out.count;
}

size_t prox_serialize_vector(const ProxScanVector &vec, uint8_t *buf, size_t buf_len) {
    size_t needed = 1u + (size_t)vec.count * 8u;
    if (buf_len < needed) return 0;

    buf[0] = vec.count;
    for (int i = 0; i < vec.count; i++) {
        uint8_t *p = buf + 1 + i * 8;
        memcpy(p, vec.devices[i].mac, 6);
        p[6] = vec.devices[i].type;
        // Encode RSSI as unsigned: rssi_wire = rssi + 128
        p[7] = (uint8_t)((int)vec.devices[i].rssi + 128);
    }
    return needed;
}

bool prox_query_anchor(const uint8_t bleMac_be[6],
                       const ProxScanVector &vec,
                       ProxScoreResult &result) {
    // Convert big-endian MAC to NimBLE little-endian format
    uint8_t mac_le[6];
    for (int i = 0; i < 6; i++) mac_le[i] = bleMac_be[5 - i];
    NimBLEAddress addr(mac_le, BLE_ADDR_PUBLIC);

    NimBLEClient *client = NimBLEDevice::createClient();
    if (!client) return false;

    // Set desired MTU before connecting so it is negotiated on connection
    NimBLEDevice::setMTU(BLE_REQUESTED_MTU);

    client->setConnectionParams(12, 12, 0, 400); // fast connection
    if (!client->connect(addr)) {
        NimBLEDevice::deleteClient(client);
        return false;
    }

    NimBLERemoteService *svc = client->getService(ANCHOR_SERVICE_UUID);
    if (!svc) {
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    NimBLERemoteCharacteristic *vecChar =
        svc->getCharacteristic(ANCHOR_PROX_VECTOR_CHAR_UUID);
    NimBLERemoteCharacteristic *scoreChar =
        svc->getCharacteristic(ANCHOR_PROX_SCORE_CHAR_UUID);

    if (!vecChar || !scoreChar) {
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    // Serialise the vector; truncate to fit negotiated MTU if necessary
    uint16_t mtu = client->getMTU();
    ProxScanVector send_vec = vec;
    size_t max_payload = (mtu > 3) ? (mtu - 3) : 20;
    while (send_vec.count > 0) {
        size_t sz = 1u + (size_t)send_vec.count * 8u;
        if (sz <= max_payload) break;
        send_vec.count--;
        if (mtu < PROX_MIN_MTU_BYTES) {
            Serial.printf("[PROX] MTU %d below minimum; truncating vector\n", mtu);
        }
    }

    uint8_t buf[1 + PROX_MAX_DEVICES * 8];
    size_t sz = prox_serialize_vector(send_vec, buf, sizeof(buf));
    if (sz == 0) {
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    // Write vector (with response)
    if (!vecChar->writeValue(buf, sz, true)) {
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    // Read score
    std::string score_val = scoreChar->readValue();
    if (score_val.size() < 2) {
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    result.score = (uint8_t)score_val[0];
    result.flags = (uint8_t)score_val[1];

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return true;
}

ProxProximity prox_interpret_score(uint8_t score) {
    if (score >= PROX_CONFIDENCE_THRESHOLD_U8) return PROX_NEAR;
    if (score <= (255 - PROX_CONFIDENCE_THRESHOLD_U8)) return PROX_AWAY;
    return PROX_AMBIGUOUS;
}
