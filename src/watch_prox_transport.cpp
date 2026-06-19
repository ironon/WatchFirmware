// ============================================================
//  Watch Proximity Transport / Orchestration
//  See watch_prox_transport.h and firmware_spec_v2.md §5.4 / §6.3 / §8.2
// ============================================================
#include "watch_prox_transport.h"
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

// Anchor proximity GATT UUIDs (must match anchor firmware)
#define ANCHOR_SERVICE_UUID           "4A0F0001-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_VECTOR_CHAR_UUID  "4A0F0008-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_SCORE_CHAR_UUID   "4A0F0009-F8CE-11EE-8001-020304050607"

// WiFi APs are stationary, so we reuse cached scan results between queries and
// only rescan when the cache is stale (a blocking, power-hungry scan). Re-homed
// from the old src/proximity.h. See §8.2.
#define PROX_WIFI_SCAN_INTERVAL_MS  300000  // 5 minutes

// ── WiFi AP cache ─────────────────────────────────────────────
// APs are stationary, so scan results are reused between queries (refreshed at
// most every PROX_WIFI_SCAN_INTERVAL_MS). Re-homed from the old proximity.cpp;
// previously lived inside prox_build_scan_vector().
static ProxDevice g_wifi_cache[PROX_MAX_DEVICES];
static uint8_t    g_wifi_cache_count = 0;
static uint32_t   g_wifi_cache_ms    = 0;  // millis() of last successful scan; 0 = never

// Refresh / reuse the cached WiFi APs and feed them into the shared engine's
// scan buffer. Preserves the exact behavior of the old prox_build_scan_vector:
//   - rescan only when WiFi associated AND cache stale (or never scanned)
//   - reuse cached APs while the cache is within 2x the refresh interval
//   - drop the cache once it is fully stale
void prox_feed_wifi_aps() {
    uint32_t now = millis();

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
                    pd.type = PROX_TYPE_WIFI;
                    pd.rssi = (int8_t)aps[i].rssi;
                }
                free(aps);
            }
            esp_wifi_clear_ap_list();
            g_wifi_cache_ms = now;
        }
    }

    // Feed cached APs into the engine (empty if never scanned / WiFi never up).
    // Drop the cache if it has gone fully stale (e.g. WiFi disconnected long ago).
    if (g_wifi_cache_ms != 0 && now - g_wifi_cache_ms <= 2 * PROX_WIFI_SCAN_INTERVAL_MS) {
        for (int i = 0; i < g_wifi_cache_count; i++) {
            prox_ingest_scan_result(g_wifi_cache[i].mac, PROX_TYPE_WIFI, g_wifi_cache[i].rssi);
        }
    } else {
        g_wifi_cache_count = 0;
    }
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
    size_t sz = prox_serialize_vector(&send_vec, buf, sizeof(buf));
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
