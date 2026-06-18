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

// WiFi AP cache — APs are stationary, so scan results are reused between queries
// (refreshed at most every PROX_WIFI_SCAN_INTERVAL_MS). See prox_build_scan_vector.
static ProxDevice g_wifi_cache[PROX_MAX_DEVICES];
static uint8_t    g_wifi_cache_count = 0;
static uint32_t   g_wifi_cache_ms    = 0;  // millis() of last successful scan; 0 = never

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

    // Collect BLE devices from cache (entries seen within PROX_CACHE_STALE_MS)
    uint32_t now = millis();
    uint8_t ble_count = 0;
    for (int i = 0; i < PROX_BLE_CACHE_SIZE && out.count < PROX_MAX_DEVICES; i++) {
        if (!g_ble_cache[i].valid) continue;
        if (now - g_ble_cache[i].last_seen_ms > PROX_CACHE_STALE_MS) continue; // stale
        ProxDevice &pd = out.devices[out.count++];
        memcpy(pd.mac, g_ble_cache[i].mac, 6);
        pd.type = 0; // BLE
        pd.rssi = g_ble_cache[i].rssi;
        ble_count++;
    }

    // WiFi APs: stationary, so we reuse cached scan results and only rescan when
    // the cache is stale (§8.2). The scan is skipped entirely when WiFi is not
    // associated — anchor proximity then runs on BLE alone. This removes the
    // single largest per-query radio cost when WiFi is in use, and all of it
    // when WiFi is off.
    if (WiFi.status() == WL_CONNECTED &&
        (g_wifi_cache_ms == 0 || now - g_wifi_cache_ms > PROX_WIFI_SCAN_INTERVAL_MS)) {
        wifi_scan_config_t cfg = {};
        cfg.show_hidden = false;
        cfg.scan_type   = WIFI_SCAN_TYPE_ACTIVE;
        if (esp_wifi_scan_start(&cfg, true) == ESP_OK) { // blocking scan
            uint16_t ap_count = 0;
            esp_wifi_scan_get_ap_num(&ap_count);
            wifi_ap_record_t *aps = (ap_count > 0)
                ? (wifi_ap_record_t*)malloc(ap_count * sizeof(wifi_ap_record_t)) : nullptr;
            g_wifi_cache_count = 0;
            if (aps) {
                esp_wifi_scan_get_ap_records(&ap_count, aps);
                for (int i = 0; i < ap_count && g_wifi_cache_count < PROX_MAX_DEVICES; i++) {
                    ProxDevice &pd = g_wifi_cache[g_wifi_cache_count++];
                    memcpy(pd.mac, aps[i].bssid, 6); // BSSID already big-endian
                    pd.type = 1; // WiFi AP
                    pd.rssi = (int8_t)aps[i].rssi;
                }
                free(aps);
            }
            esp_wifi_clear_ap_list();
            g_wifi_cache_ms = now;
        }
    }

    // Append cached APs (empty if never scanned / WiFi never up). Drop the cache
    // if it has gone fully stale (e.g. WiFi disconnected long ago).
    if (g_wifi_cache_ms != 0 && now - g_wifi_cache_ms <= 2 * PROX_WIFI_SCAN_INTERVAL_MS) {
        for (int i = 0; i < g_wifi_cache_count && out.count < PROX_MAX_DEVICES; i++) {
            out.devices[out.count++] = g_wifi_cache[i];
        }
    } else {
        g_wifi_cache_count = 0;
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

    Serial.printf("[PROX] Scan vector: %d devices (%d BLE + %d WiFi); min for full score=%d\n",
                  out.count, ble_count, out.count - ble_count, PROX_MIN_DEVICE_COUNT);
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
                       uint8_t addr_type,
                       const ProxScanVector &vec,
                       ProxScoreResult &result) {
    // Convert big-endian MAC to NimBLE little-endian format
    // print everything about this query
    // Serial.printf("[PROX] Querying anchor %02X:%02X:%02X:%02X:%02X:%02X (addr_type=%d) with %d devices\n",
    //               bleMac_be[0], bleMac_be[1], bleMac_be[2],
    //               bleMac_be[3], bleMac_be[4], bleMac_be[5],
    //               addr_type, vec.count);
    // NimBLEAddress(const uint8_t[6], type) expects the address in big-endian
    // (on-air, MSB-first) order — it std::reverse_copy's into its native LE
    // store internally. bleMac_be is already big-endian, so pass it directly.
    // (Previously this reversed bleMac_be first, causing a double-reversal: the
    // connect targeted a byte-swapped address that no peer owned, so every
    // connect timed out and the anchor never saw a request.)
    NimBLEAddress addr(bleMac_be, addr_type);
    // Serial.printf("[PROX] Target addr (type=%d): %s\n", addr_type, addr.toString().c_str());

    // The ESP32 controller cannot reliably initiate a connection while a scan is
    // in progress. Stop the background scan first (and report it, so a recurring
    // "scan was active" line flags this as the failure mode if connects still fail).
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan && scan->isScanning()) {
        Serial.println("[PROX] Scan was active — stopping before connect");
        scan->stop();
    }

    NimBLEClient *client = NimBLEDevice::createClient();
    if (!client) {
        Serial.println("[PROX] FAIL: createClient() returned null");
        return false;
    }
    // Serial.println("[PROX] Client created");

    // Set desired MTU before connecting so it is negotiated on connection
    NimBLEDevice::setMTU(BLE_REQUESTED_MTU);
    // Serial.printf("[PROX] Requested MTU=%d\n", BLE_REQUESTED_MTU);

    client->setConnectionParams(12, 12, 0, 400); // fast connection
    // Serial.printf("[PROX] Connecting to %s ...\n", addr.toString().c_str());
    unsigned long t_connect = millis();
    if (!client->connect(addr)) {
        Serial.printf("[PROX] FAIL: connect() returned false after %lu ms (last rc=%d)\n",
                      millis() - t_connect, client->getLastError());
        NimBLEDevice::deleteClient(client);
        return false;
    }
    // Serial.printf("[PROX] Connected in %lu ms. Connected=%d RSSI=%d\n",
                //   millis() - t_connect, client->isConnected(), client->getRssi());

    // Serial.printf("[PROX] Discovering service %s ...\n", ANCHOR_SERVICE_UUID);
    NimBLERemoteService *svc = client->getService(ANCHOR_SERVICE_UUID);
    if (!svc) {
        // Serial.println("[PROX] FAIL: anchor service not found on peer");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }
    // Serial.println("[PROX] Service found");

    NimBLERemoteCharacteristic *vecChar =
        svc->getCharacteristic(ANCHOR_PROX_VECTOR_CHAR_UUID);
    NimBLERemoteCharacteristic *scoreChar =
        svc->getCharacteristic(ANCHOR_PROX_SCORE_CHAR_UUID);

    Serial.printf("[PROX] Characteristics: vec=%s score=%s\n",
                  vecChar ? "found" : "MISSING",
                  scoreChar ? "found" : "MISSING");
    if (!vecChar || !scoreChar) {
        Serial.println("[PROX] FAIL: required characteristic missing");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    // Serialise the vector; truncate to fit negotiated MTU if necessary
    uint16_t mtu = client->getMTU();
    // Serial.printf("[PROX] Negotiated MTU=%d\n", mtu);
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
    if (send_vec.count != vec.count) {
        Serial.printf("[PROX] Vector truncated from %d to %d devices to fit MTU\n",
                      vec.count, send_vec.count);
    }

    uint8_t buf[1 + PROX_MAX_DEVICES * 8];
    size_t sz = prox_serialize_vector(send_vec, buf, sizeof(buf));
    if (sz == 0) {
        // Serial.println("[PROX] FAIL: prox_serialize_vector returned 0 bytes");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }
    // Serial.printf("[PROX] Writing %u-byte vector (%d devices) to vector char ...\n",
                //   (unsigned)sz, send_vec.count);

    // Write vector (with response)
    if (!vecChar->writeValue(buf, sz, true)) {
        // Serial.printf("[PROX] FAIL: writeValue() returned false (rc=%d)\n",
                    //   client->getLastError());
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }
    // Serial.println("[PROX] Vector write acknowledged");

    // Read score
    // Serial.println("[PROX] Reading score char ...");
    std::string score_val = scoreChar->readValue();
    // Serial.printf("[PROX] Score read returned %u bytes\n", (unsigned)score_val.size());
    if (score_val.size() < 2) {
        // Serial.println("[PROX] FAIL: score read too short (<2 bytes)");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }

    result.score = (uint8_t)score_val[0];
    result.flags = (uint8_t)score_val[1];
    Serial.printf("[PROX] SUCCESS: score=%d flags=0x%02X\n", result.score, result.flags);

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return true;
}

ProxProximity prox_interpret_score(uint8_t score) {
    if (score >= PROX_CONFIDENCE_THRESHOLD_U8) return PROX_NEAR;
    if (score <= (255 - PROX_CONFIDENCE_THRESHOLD_U8)) return PROX_AWAY;
    return PROX_AMBIGUOUS;
}
