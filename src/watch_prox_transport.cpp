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
#define ANCHOR_DOCK_STATUS_CHAR_UUID  "4A0F000D-F8CE-11EE-8001-020304050607"  // phone docking (§4.11)
#define ANCHOR_CALIB_MODE_CHAR_UUID   "4A0F000F-F8CE-11EE-8001-020304050607"  // calibration-v2 phase

// WiFi APs are stationary, so we reuse cached scan results between queries and
// only rescan when the cache is stale (a blocking, power-hungry scan). Re-homed
// from the old src/proximity.h. See §8.2.
#define PROX_WIFI_SCAN_INTERVAL_MS  300000  // 5 minutes

// Bound on the blocking central connect to the anchor. NimBLE's default is 30 s;
// that is far too long here. If a connect collides with the phone reconnecting
// to the watch (the single C3 radio can't do both at once — the Option A crash
// window), the connect can stall for many seconds and blow the calibration phase
// deadline. A short timeout lets the connect fail fast so the caller's retry loop
// can recover on the next tick. See §8.5 and firmware_spec_v2.md calibration bugs.
#define PROX_CONNECT_TIMEOUT_MS     5000

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
                       ProxScoreResult &result,
                       int8_t *out_dock,
                       uint8_t calib_phase,
                       uint8_t *out_near_threshold) {
    if (out_dock) *out_dock = -1;  // unknown until read (fail-open on the dock signal)
    if (out_near_threshold) *out_near_threshold = 0;
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
    client->setConnectTimeout(PROX_CONNECT_TIMEOUT_MS); // fail fast on a radio collision (default 30 s)
    Serial.printf("[PROX] Connecting to %s ...\n", addr.toString().c_str());
    unsigned long t_connect = millis();
    if (!client->connect(addr)) {
        Serial.printf("[PROX] FAIL: connect() returned false after %lu ms (last rc=%d)\n",
                      millis() - t_connect, client->getLastError());
        NimBLEDevice::deleteClient(client);
        return false;
    }
    Serial.printf("[PROX] Connected in %lu ms (RSSI=%d), discovering service...\n",
                  millis() - t_connect, client->getRssi());

    // Serial.printf("[PROX] Discovering service %s ...\n", ANCHOR_SERVICE_UUID);
    NimBLERemoteService *svc = client->getService(ANCHOR_SERVICE_UUID);
    if (!svc) {
        Serial.println("[PROX] FAIL: anchor service not found on peer");
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

    // Calibration-v2: set the anchor's phase on this connection before submitting
    // the vector so it routes training correctly (INSIDE trains, EDGE collects).
    // The anchor remembers the phase across reconnects; enforcement (0xFF) skips
    // this. Best-effort: a pre-v2 anchor lacks …000F — degrade to unphased.
    if (calib_phase != 0xFF) {
        NimBLERemoteCharacteristic *calibChar =
            svc->getCharacteristic(ANCHOR_CALIB_MODE_CHAR_UUID);
        if (calibChar) {
            uint8_t p = calib_phase;
            if (!calibChar->writeValue(&p, 1, true))
                Serial.printf("[CALIB] WARN: …000F phase write failed (rc=%d)\n",
                              client->getLastError());
            else
                Serial.printf("[CALIB] anchor phase set to %u\n", (unsigned)calib_phase);
        } else {
            Serial.println("[CALIB] WARN: anchor has no …000F (pre-v2) — unphased");
        }
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
        Serial.printf("[PROX] FAIL: writeValue() returned false (rc=%d)\n",
                      client->getLastError());
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
    // 3rd byte (calibration-v2): the anchor's per-anchor calibrated near-threshold
    // (0 = uncalibrated). Absent on a pre-v2 anchor (2-byte score) → leave 0.
    if (out_near_threshold && score_val.size() >= 3)
        *out_near_threshold = (uint8_t)score_val[2];
    Serial.printf("[PROX] SUCCESS: score=%d flags=0x%02X thr=%u\n", result.score, result.flags,
                  (out_near_threshold ? (unsigned)*out_near_threshold : 0u));

    // Phone docking (§4.11): for phoneAway the caller wants to know whether the
    // phone is still docked at this anchor. Read the Dock Status characteristic
    // on the same connection. Absent char / short read → leave as -1 (unknown),
    // which the caller treats as docked (fail-open on the dock signal).
    if (out_dock) {
        NimBLERemoteCharacteristic *dockChar =
            svc->getCharacteristic(ANCHOR_DOCK_STATUS_CHAR_UUID);
        if (dockChar) {
            std::string dv = dockChar->readValue();
            if (dv.size() >= 1) {
                *out_dock = (dv[0] != 0) ? 1 : 0;
                Serial.printf("[PROX] Dock status: %s\n", *out_dock ? "docked" : "UNDOCKED");
            }
        }
    }

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return true;
}

ProxProximity prox_interpret_score(uint8_t score, uint8_t near_threshold) {
    if (near_threshold != 0) {
        // Calibration-v2: per-anchor cutoff with a hysteresis band just below it.
        if (score >= near_threshold) return PROX_NEAR;
        uint8_t lo = (near_threshold > PROX_NEAR_HYST_U8)
                       ? (uint8_t)(near_threshold - PROX_NEAR_HYST_U8) : 0;
        if (score >= lo) return PROX_AMBIGUOUS;
        return PROX_AWAY;
    }
    if (score >= PROX_CONFIDENCE_THRESHOLD_U8) return PROX_NEAR;
    if (score <= (255 - PROX_CONFIDENCE_THRESHOLD_U8)) return PROX_AWAY;
    return PROX_AMBIGUOUS;
}

bool prox_finalize_anchor(const uint8_t bleMac_be[6],
                          uint8_t addr_type,
                          uint8_t *out_thr,
                          uint16_t *out_inside_n,
                          uint16_t *out_edge_n,
                          uint8_t *out_confidence) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan && scan->isScanning()) scan->stop();

    NimBLEAddress addr(bleMac_be, addr_type);
    NimBLEClient *client = NimBLEDevice::createClient();
    if (!client) { Serial.println("[CALIB] FINALIZE: createClient null"); return false; }
    NimBLEDevice::setMTU(BLE_REQUESTED_MTU);
    client->setConnectionParams(12, 12, 0, 400);
    client->setConnectTimeout(PROX_CONNECT_TIMEOUT_MS); // fail fast on a radio collision (default 30 s)
    Serial.printf("[CALIB] FINALIZE connecting to %s ...\n", addr.toString().c_str());
    if (!client->connect(addr)) {
        Serial.printf("[CALIB] FINALIZE: connect failed (rc=%d)\n", client->getLastError());
        NimBLEDevice::deleteClient(client);
        return false;
    }
    NimBLERemoteService *svc = client->getService(ANCHOR_SERVICE_UUID);
    NimBLERemoteCharacteristic *calibChar =
        svc ? svc->getCharacteristic(ANCHOR_CALIB_MODE_CHAR_UUID) : nullptr;
    if (!calibChar) {
        Serial.println("[CALIB] FINALIZE: anchor has no …000F char");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return false;
    }
    // Write FINALIZE (3). The anchor computes + persists the threshold and stashes
    // the result frame as the char value; read it back on the same connection.
    uint8_t fin = 3; // CALIB_PHASE_FINALIZE
    bool ok = false;
    if (calibChar->writeValue(&fin, 1, true)) {
        std::string frame = calibChar->readValue();
        if (frame.size() >= 7 && (uint8_t)frame[0] == 0x01) {
            if (out_thr)       *out_thr = (uint8_t)frame[1];
            if (out_inside_n)  *out_inside_n = (uint8_t)frame[2] | ((uint16_t)(uint8_t)frame[3] << 8);
            if (out_edge_n)    *out_edge_n = (uint8_t)frame[4] | ((uint16_t)(uint8_t)frame[5] << 8);
            if (out_confidence)*out_confidence = (uint8_t)frame[6];
            ok = true;
            Serial.printf("[CALIB] FINALIZE frame: thr=%u inside=%u edge=%u conf=%u\n",
                          (unsigned)(uint8_t)frame[1],
                          (unsigned)((uint8_t)frame[2] | ((uint16_t)(uint8_t)frame[3] << 8)),
                          (unsigned)((uint8_t)frame[4] | ((uint16_t)(uint8_t)frame[5] << 8)),
                          (unsigned)(uint8_t)frame[6]);
        } else {
            Serial.printf("[CALIB] FINALIZE: bad frame (%u bytes)\n", (unsigned)frame.size());
        }
    } else {
        Serial.printf("[CALIB] FINALIZE: write failed (rc=%d)\n", client->getLastError());
    }
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return ok;
}
