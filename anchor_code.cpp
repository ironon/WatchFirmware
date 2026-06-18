// ============================================================
//  Anchor Firmware  — ESP32-C3-WROOM
//  See firmware_spec_v2.md for full specification.
// ============================================================
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_crc.h>
#include <esp_random.h>
#include <esp_mac.h>
#include <time.h>
#include <string.h>

#include "proximity.h"

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


    
// ============================================================
//  Pin definitions
// ============================================================
#define BUZZER_PIN          21
#define FACTORY_RESET_PIN    9   // BOOT button on ESP32-C3-WROOM
#define SERVO_PIN           10   // SG90 servo signal

// ============================================================
//  Debug mode  — set to 1 to enable audio debug indicators
//  BLE client connected    : 1 medium beep  (300 ms)
//  BLE client disconnected : 2 short beeps  (100 ms each)
//  WiFi creds received     : 1 short beep   (100 ms)
//  WiFi connected          : 2 medium beeps (200 ms each)
//  WiFi failed             : 3 rapid beeps  (80 ms each)
//  Schedule received OK    : 3 medium beeps (200 ms each)
// ============================================================
#define DEBUG_MODE 1

// ============================================================
//  Constants
// ============================================================
#define ANCHOR_UDP_PORT                  5555
#define ANCHOR_WIFI_RETRY_INTERVAL_S       30
#define IDENTIFY_BEEP_DURATION_MS         800
#define ANCHOR_MAX_BEEP_MINUTES_DEFAULT    30
#define MAX_EVENTS                         64

// Servo constants (SG90, GPIO 10)
#define SERVO_CLOSED_DEGREES             180   // locked position
#define SERVO_OPEN_DEGREES               160   // unlocked position
#define SERVO_MOVE_DURATION_MS           1000   // time to let servo reach target
#define SERVO_LEDC_FREQ                   50   // 50 Hz PWM
#define SERVO_LEDC_RES                    14   // 14-bit resolution (ESP32-C3 max)
#define SERVO_LEDC_CHANNEL                 0   // LEDC channel for servo PWM

// ============================================================
//  Shared UUIDs  (must match mobile app and watch firmware)
// ============================================================
#define ANCHOR_SERVICE_UUID          "4A0F0001-F8CE-11EE-8001-020304050607"
#define ANCHOR_IDENTIFY_CHAR_UUID    "4A0F0002-F8CE-11EE-8001-020304050607"
#define ANCHOR_WIFI_CRED_CHAR_UUID   "4A0F0003-F8CE-11EE-8001-020304050607"
#define ANCHOR_SETTINGS_CHAR_UUID    "4A0F0004-F8CE-11EE-8001-020304050607"
#define ANCHOR_SCHED_CTRL_CHAR_UUID  "4A0F0005-F8CE-11EE-8001-020304050607"
#define ANCHOR_SCHED_DATA_CHAR_UUID  "4A0F0006-F8CE-11EE-8001-020304050607"
#define ANCHOR_TOGGLE_CHAR_UUID           "4A0F0007-F8CE-11EE-8001-020304050607"
// Proximity engine characteristics (v2)
#define ANCHOR_PROX_VECTOR_CHAR_UUID      "4A0F0008-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_SCORE_CHAR_UUID       "4A0F0009-F8CE-11EE-8001-020304050607"
#define ANCHOR_FINGERPRINT_CTRL_CHAR_UUID "4A0F000A-F8CE-11EE-8001-020304050607"
#define ANCHOR_FINGERPRINT_DATA_CHAR_UUID "4A0F000B-F8CE-11EE-8001-020304050607"

// ============================================================
//  Enumerations
// ============================================================
enum RecurrenceType         : uint8_t { ONCE=0, DAILY=1, WEEKLY=2, MONTHLY=3 };
enum Criteria               : uint8_t { GET_AWAY=0, STAY_NEAR=1, GET_OFF_WIFI=2, GET_ON_WIFI=3 };
enum EnforcementProfile     : uint8_t {
    STRICT_SILENT=0, NORMAL_SILENT=1, LOOSE_SILENT=2,
    STRICT_BOTH=3,   NORMAL_BOTH=4,   LOOSE_BOTH=5,
    STRICT_BUZZ=6,   NORMAL_BUZZ=7,   LOOSE_BUZZ=8
};
enum AnchorEnforcementProfile : uint8_t { AP_LIGHT=0, AP_MEDIUM=1, AP_HARD=2 };

// ============================================================
//  Event struct
// ============================================================
struct Event {
    uint8_t  id[16];
    int64_t  referenceDate;
    uint16_t startTime;           // minutes since midnight
    uint16_t endTime;             // minutes since midnight
    RecurrenceType   recurrenceType;
    uint8_t  dayOfWeek;
    uint8_t  dayOfMonth;
    Criteria criteria;
    EnforcementProfile     profile;
    AnchorEnforcementProfile anchorProfile;
    bool     hasAnchorProfile;
    bool     negate;
    bool     hasAnchorId;
    uint8_t  anchorId[16];
    char     wifiSSID[65];
    uint8_t  beepAnchors[8][16];
    uint8_t  beepAnchorCount;
};

// ============================================================
//  Globals
// ============================================================
static Preferences prefs;

static uint8_t g_uuid[16];
static char    g_uuid_str[37];
static char    g_wifi_ssid[64];
static char    g_wifi_pass[64];
static uint16_t g_max_beep_minutes = ANCHOR_MAX_BEEP_MINUTES_DEFAULT;

// Today's schedule (in RAM)
static Event g_events[MAX_EVENTS];
static int   g_event_count     = 0;
static bool  g_schedule_loaded = false;

// Servo state
enum ServoState : uint8_t { SERVO_CLOSED = 0, SERVO_OPEN_STATE = 1 };
static ServoState g_servo_state = SERVO_CLOSED;

// BLE
static NimBLEServer         *pServer             = nullptr;
static NimBLECharacteristic *pWifiCredChar        = nullptr;
static NimBLECharacteristic *pSettingsChar        = nullptr;
static NimBLECharacteristic *pSchedCtrlChar       = nullptr;
static NimBLECharacteristic *pToggleChar          = nullptr;
static NimBLECharacteristic *pProxScoreChar       = nullptr;

// Fingerprint BLE transfer state
static uint8_t  *g_fp_buf            = nullptr;
static uint32_t  g_fp_expected_len   = 0;
static uint32_t  g_fp_received_len   = 0;
static bool      g_fp_xfer_active    = false;

// WiFi / networking
static bool      g_wifi_connected       = false;
static uint32_t  g_last_wifi_retry_ms   = 0;
static WiFiUDP   g_udp;
static WebServer g_http_server(80);

// Beeping state
static bool     g_beeping             = false;
static bool     g_beep_phase_on       = false;
static uint8_t  g_active_event_id[16] = {};
static bool     g_has_active_event    = false;
static AnchorEnforcementProfile g_beep_profile;
static uint32_t g_beep_phase_start_ms = 0;
static uint32_t g_beep_start_ms       = 0;

// Schedule BLE transfer
static uint8_t  *g_sched_buf            = nullptr;
static uint32_t  g_sched_expected_len   = 0;
static uint32_t  g_sched_received_len   = 0;
static bool      g_sched_xfer_active    = false;

// Midnight tracking
static int g_last_day = -1;

// ============================================================
//  UUID helpers
// ============================================================

static void uuid_to_str(const uint8_t *b, char *out) {
    sprintf(out,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        b[0],b[1],b[2],b[3], b[4],b[5], b[6],b[7],
        b[8],b[9], b[10],b[11],b[12],b[13],b[14],b[15]);
}

static void generate_uuid_v4(uint8_t *out) {
    esp_fill_random(out, 16);
    out[6] = (out[6] & 0x0F) | 0x40;
    out[8] = (out[8] & 0x3F) | 0x80;
}

static bool uuid_eq(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 16) == 0;
}

// ============================================================
//  Schedule deserialization
// ============================================================

static bool deserialize_schedule(const uint8_t *data, uint32_t len,
                                  Event *events, int *count) {
    if (len < 2) return false;
    uint32_t pos = 0;

    uint16_t n;
    memcpy(&n, data + pos, 2); pos += 2;
    if (n > MAX_EVENTS) n = MAX_EVENTS;

    for (int i = 0; i < (int)n; i++) {
        Event &e = events[i];
        memset(&e, 0, sizeof(Event));

        if (pos + 16 > len) return false;
        memcpy(e.id, data + pos, 16); pos += 16;

        if (pos + 8 > len) return false;
        memcpy(&e.referenceDate, data + pos, 8); pos += 8;

        if (pos + 2 > len) return false;
        memcpy(&e.startTime, data + pos, 2); pos += 2;

        if (pos + 2 > len) return false;
        memcpy(&e.endTime, data + pos, 2); pos += 2;

        if (pos + 1 > len) return false;
        e.recurrenceType = (RecurrenceType)data[pos++];

        if (pos + 1 > len) return false;
        e.dayOfWeek  = data[pos++];

        if (pos + 1 > len) return false;
        e.dayOfMonth = data[pos++];

        if (pos + 1 > len) return false;
        e.criteria = (Criteria)data[pos++];

        if (pos + 1 > len) return false;
        e.profile = (EnforcementProfile)data[pos++];

        if (pos + 1 > len) return false;
        uint8_t ap = data[pos++];
        if (ap == 0xFF) {
            e.hasAnchorProfile = false;
            e.anchorProfile    = AP_LIGHT;
        } else {
            e.hasAnchorProfile = true;
            e.anchorProfile    = (AnchorEnforcementProfile)ap;
        }

        if (pos + 1 > len) return false;
        e.negate = data[pos++] != 0;

        if (pos + 1 > len) return false;
        e.hasAnchorId = data[pos++] != 0;

        if (pos + 16 > len) return false;
        memcpy(e.anchorId, data + pos, 16); pos += 16;

        if (pos + 1 > len) return false;
        uint8_t ssid_len = data[pos++];
        if (ssid_len > 0) {
            if (pos + ssid_len > len) return false;
            uint8_t copy_len = ssid_len < 64 ? ssid_len : 64;
            memcpy(e.wifiSSID, data + pos, copy_len);
            e.wifiSSID[copy_len] = 0;
            pos += ssid_len;
        }

        if (pos + 1 > len) return false;
        e.beepAnchorCount = data[pos++];
        if (e.beepAnchorCount > 8) e.beepAnchorCount = 8;
        for (int j = 0; j < e.beepAnchorCount; j++) {
            if (pos + 16 > len) return false;
            memcpy(e.beepAnchors[j], data + pos, 16); pos += 16;
        }
    }
    *count = (int)n;
    return true;
}

// ============================================================
//  Beeping helpers
// ============================================================

static uint32_t minutes_since_midnight() {
    struct tm ti;
    if (!getLocalTime(&ti)) return 0;
    return (uint32_t)(ti.tm_hour * 60 + ti.tm_min);
}

static Event* find_event_by_id(const uint8_t *id) {
    for (int i = 0; i < g_event_count; i++) {
        if (uuid_eq(g_events[i].id, id)) return &g_events[i];
    }
    return nullptr;
}

static bool anchor_in_beep_list(const Event *e) {
    for (int i = 0; i < e->beepAnchorCount; i++) {
        if (uuid_eq(e->beepAnchors[i], g_uuid)) return true;
    }
    return false;
}

static void start_beeping(const Event *e) {
    if (!e->hasAnchorProfile) return;
    g_beeping           = true;
    g_beep_profile      = e->anchorProfile;
    g_beep_phase_on     = true;
    g_beep_phase_start_ms = millis();
    g_beep_start_ms     = millis();
    memcpy(g_active_event_id, e->id, 16);
    g_has_active_event  = true;
    digitalWrite(BUZZER_PIN, HIGH);
}

static void stop_beeping() {
    g_beeping          = false;
    g_has_active_event = false;
    digitalWrite(BUZZER_PIN, LOW);
}

// on/off phase durations per AnchorEnforcementProfile
static void beep_durations(AnchorEnforcementProfile p,
                            uint32_t *on_ms, uint32_t *off_ms) {
    switch (p) {
        case AP_LIGHT:  *on_ms = 3000; *off_ms = 60000; break;
        case AP_MEDIUM: *on_ms = 3000; *off_ms = 30000; break;
        case AP_HARD:   *on_ms = 4000; *off_ms = 10000; break;
        default:        *on_ms = 3000; *off_ms = 30000; break;
    }
}

static void update_beeping() {
    if (!g_beeping) return;

    // max_beep_minutes guard
    if ((millis() - g_beep_start_ms) > (uint32_t)g_max_beep_minutes * 60000UL) {
        stop_beeping(); return;
    }

    // event end time guard
    if (g_has_active_event) {
        Event *e = find_event_by_id(g_active_event_id);
        if (e && minutes_since_midnight() >= e->endTime) {
            stop_beeping(); return;
        }
    }

    uint32_t on_ms, off_ms;
    beep_durations(g_beep_profile, &on_ms, &off_ms);
    uint32_t elapsed = millis() - g_beep_phase_start_ms;

    if (g_beep_phase_on) {
        if (elapsed >= on_ms) {
            g_beep_phase_on       = false;
            g_beep_phase_start_ms = millis();
            digitalWrite(BUZZER_PIN, LOW);
        }
    } else {
        if (elapsed >= off_ms) {
            g_beep_phase_on       = true;
            g_beep_phase_start_ms = millis();
            digitalWrite(BUZZER_PIN, HIGH);
        }
    }
}

// ============================================================
//  Servo management
// ============================================================

// Converts degrees to a 14-bit LEDC duty for a 50Hz servo.
// SG90 pulse range: 0.5ms (0°) to 2.5ms (180°), period = 20ms.
// 14-bit max = 16384 counts.
static uint32_t degrees_to_duty(uint8_t degrees) {
    float pulse_ms = 0.5f + (degrees / 180.0f) * 2.0f;
    return (uint32_t)((pulse_ms / 20.0f) * 16384.0f);
}

// Attach → set angle → wait → detach.  Blocks for SERVO_MOVE_DURATION_MS.
static void servo_manager_command(uint8_t degrees) {
    ledcSetup(SERVO_LEDC_CHANNEL, SERVO_LEDC_FREQ, SERVO_LEDC_RES);
    ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
    ledcWrite(SERVO_LEDC_CHANNEL, degrees_to_duty(degrees));
    delay(SERVO_MOVE_DURATION_MS);
    ledcDetachPin(SERVO_PIN);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);   // hold pin LOW so it doesn't float after detach
    g_servo_state = (degrees == SERVO_CLOSED_DEGREES) ? SERVO_CLOSED : SERVO_OPEN_STATE;
    // Keep the Toggle characteristic value in sync so reads return the current state.
    if (pToggleChar) {
        uint8_t v = (uint8_t)g_servo_state;
        pToggleChar->setValue(&v, 1);
    }
    Serial.printf("[SERVO] Moved to %d° — state: %s\n",
                  degrees, g_servo_state == SERVO_CLOSED ? "closed" : "open");
}

// ============================================================
//  Enforcement helpers
// ============================================================

// Returns true if the current time falls within any event window that
// involves this anchor (either as anchorId or in beepAnchors).
static bool anchor_is_in_active_enforcement_event() {
    if (!g_schedule_loaded) return false;
    uint32_t now_min = minutes_since_midnight();
    for (int i = 0; i < g_event_count; i++) {
        const Event &e = g_events[i];
        if (e.negate) continue;
        if (now_min < e.startTime || now_min >= e.endTime) continue;
        if (e.hasAnchorId && uuid_eq(e.anchorId, g_uuid)) return true;
        if (anchor_in_beep_list(&e)) return true;
    }
    return false;
}

// If an enforcement event is active and the servo is open, close it.
// Called after every schedule update (spec §4.9 auto-close rule).
static void check_auto_close() {
    if (g_servo_state == SERVO_OPEN_STATE && anchor_is_in_active_enforcement_event()) {
        Serial.println("[SERVO] Auto-close: enforcement event active");
        servo_manager_command(SERVO_CLOSED_DEGREES);
    }
}

// ============================================================
//  UDP command handling
// ============================================================

static void handle_udp() {
    int pkt_size = g_udp.parsePacket();
    if (pkt_size < 33) return;

    uint8_t buf[33];
    g_udp.read(buf, 33);

    uint8_t  cmd        = buf[0];
    uint8_t *event_uuid = buf + 17;

    if (cmd == 0x01) {   // WATCH_REMOVED
        if (!g_schedule_loaded) return;
        Event *e = find_event_by_id(event_uuid);
        if (!e || e->negate) return;
        if (!anchor_in_beep_list(e)) return;
        uint32_t now_min = minutes_since_midnight();
        if (now_min < e->startTime || now_min >= e->endTime) return;
        if (!g_beeping) start_beeping(e);
    } else if (cmd == 0x02) {  // WATCH_WORN
        if (g_has_active_event && uuid_eq(g_active_event_id, event_uuid)) {
            stop_beeping();
        }
    }
}

// ============================================================
//  HTTP schedule endpoint  (POST /schedule)
// ============================================================

static void handle_schedule_post() {
    // Read raw body
    if (!g_http_server.hasArg("plain")) {
        const char fail = 0x00;
        g_http_server.send(400, "application/octet-stream",
                           String(&fail, 1));
        return;
    }
    const String &body = g_http_server.arg("plain");
    size_t total = body.length();

    // Last 4 bytes = CRC32 (little-endian)
    if (total < 4) {
        const char fail = 0x00;
        g_http_server.send(400, "application/octet-stream",
                           String(&fail, 1));
        return;
    }
    const uint8_t *raw      = (const uint8_t *)body.c_str();
    uint32_t       data_len = total - 4;
    uint32_t       crc_recv;
    memcpy(&crc_recv, raw + data_len, 4);
    uint32_t crc_calc = esp_crc32_le(0, raw, data_len);

    if (crc_calc != crc_recv) {
        uint8_t fail = 0x00;
        g_http_server.send(400, "application/octet-stream",
                           String((char*)&fail, 1));
        return;
    }

    int new_count = 0;
    if (!deserialize_schedule(raw, data_len, g_events, &new_count)) {
        uint8_t fail = 0x00;
        g_http_server.send(400, "application/octet-stream",
                           String((char*)&fail, 1));
        return;
    }

    g_event_count    = new_count;
    g_schedule_loaded = true;
    check_auto_close();

    uint8_t ok = 0x01;
    g_http_server.send(200, "application/octet-stream",
                       String((char*)&ok, 1));
}

// ============================================================
//  Debug beep helper
// ============================================================

static void debug_beeps(int count, uint32_t on_ms, uint32_t off_ms = 100) {
#if DEBUG_MODE
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(on_ms);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) delay(off_ms);
    }
#endif
}

// ============================================================
//  BLE server connection callbacks
// ============================================================

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        Serial.println("[BLE] Client connected");
       
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        Serial.println("[BLE] Client disconnected");
       
        NimBLEDevice::getAdvertising()->start();
    }
};

// ============================================================
//  BLE GATT callbacks
// ============================================================

class IdentifyCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) override {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(IDENTIFY_BEEP_DURATION_MS);
        digitalWrite(BUZZER_PIN, LOW);
    }
};

class WifiCredCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        JsonDocument doc;
        uint8_t resp = 0x00;
        if (!deserializeJson(doc, val.c_str())
            && doc["ssid"].is<const char*>()
            && doc["password"].is<const char*>()) {
            strncpy(g_wifi_ssid, doc["ssid"].as<const char*>(), 63);
            strncpy(g_wifi_pass, doc["password"].as<const char*>(), 63);
            prefs.begin("anchor", false);
            prefs.putString("ssid", g_wifi_ssid);
            prefs.putString("pass", g_wifi_pass);
            prefs.end();

            WiFi.begin(g_wifi_ssid, g_wifi_pass);
            uint32_t t = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(100);
            if (WiFi.status() == WL_CONNECTED) {
                g_wifi_connected = true;
                resp = 0x01;
                MDNS.begin(g_uuid_str);
                g_udp.begin(ANCHOR_UDP_PORT);
                debug_beeps(2, 200);  // WiFi connected
            } else {
                debug_beeps(3, 80);   // WiFi failed
            }
        }
        pChar->setValue(&resp, 1);
        pChar->notify();
    }
};

class SettingsCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.size() >= 2) {
            uint16_t mbm;
            memcpy(&mbm, val.data(), 2);
            g_max_beep_minutes = mbm;
            prefs.begin("anchor", false);
            prefs.putUShort("max_beep", g_max_beep_minutes);
            prefs.end();
        }
        uint8_t resp = 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
    }
};

class SchedCtrlCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t cmd  = (uint8_t)val[0];
        uint8_t resp = 0x00;

        if (cmd == 0x01 && val.size() >= 5) {          // BEGIN
            uint32_t total_len;
            memcpy(&total_len, val.data() + 1, 4);
            if (g_sched_buf) { free(g_sched_buf); g_sched_buf = nullptr; }
            g_sched_buf = (uint8_t*)malloc(total_len);
            if (g_sched_buf) {
                g_sched_expected_len = total_len;
                g_sched_received_len = 0;
                g_sched_xfer_active  = true;
                resp = 0x01;
            }
        } else if (cmd == 0x02 && val.size() >= 5) {   // END
            uint32_t crc_recv;
            memcpy(&crc_recv, val.data() + 1, 4);
            if (g_sched_xfer_active && g_sched_buf
                && g_sched_received_len == g_sched_expected_len) {
                uint32_t crc_calc = esp_crc32_le(0, g_sched_buf, g_sched_received_len);
                if (crc_calc == crc_recv) {
                    int cnt = 0;
                    if (deserialize_schedule(g_sched_buf, g_sched_received_len, g_events, &cnt)) {
                        g_event_count    = cnt;
                        g_schedule_loaded = true;
                        check_auto_close();
                        resp = 0x01;
                        debug_beeps(3, 200);  // schedule received OK
                    }
                }
            }
            if (g_sched_buf) { free(g_sched_buf); g_sched_buf = nullptr; }
            g_sched_xfer_active = false;
        } else if (cmd == 0x03) {                       // ABORT
            if (g_sched_buf) { free(g_sched_buf); g_sched_buf = nullptr; }
            g_sched_xfer_active = false;
            resp = 0x01;
        }
        pChar->setValue(&resp, 1);
        pChar->notify();
    }
};

class SchedDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        if (!g_sched_xfer_active || !g_sched_buf) return;
        std::string val = pChar->getValue();
        uint32_t remaining = g_sched_expected_len - g_sched_received_len;
        uint32_t to_copy   = (uint32_t)val.size() < remaining
                               ? (uint32_t)val.size() : remaining;
        memcpy(g_sched_buf + g_sched_received_len, val.data(), to_copy);
        g_sched_received_len += to_copy;
    }
};

class ToggleCallback : public NimBLECharacteristicCallbacks {
    // Read: return current servo state (0x00 = closed, 0x01 = open).
    // NimBLE calls onRead before sending the value, so we just keep the
    // characteristic value in sync via servo_manager_command and here.
    void onRead(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        uint8_t v = (uint8_t)g_servo_state;
        pChar->setValue(&v, 1);
    }

    // Write: 0x00 = close (always accepted), 0x01 = open (rejected if
    // an enforcement event involving this anchor is currently active).
    // Response: 0x01 accepted, 0x02 rejected.
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t cmd  = (uint8_t)val[0];
        uint8_t resp;

        if (cmd == 0x00) {          // close — always accepted
            servo_manager_command(SERVO_CLOSED_DEGREES);
            resp = 0x01;
        } else if (cmd == 0x01) {   // open — rejected during enforcement
            if (anchor_is_in_active_enforcement_event()) {
                Serial.println("[SERVO] Open rejected: enforcement event active");
                resp = 0x02;
            } else {
                servo_manager_command(SERVO_OPEN_DEGREES);
                resp = 0x01;
            }
        } else {
            resp = 0x00;            // unknown command
        }

        // Auto-close check on every Toggle write (spec §4.9).
        check_auto_close();

        pChar->setValue(&resp, 1);
        pChar->notify();
    }
};

void beep(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}
#define bp beep(1)
// ============================================================
//  Proximity GATT callbacks  (v2)
// ============================================================

class ProxVectorCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        Serial.println("[PROX] Received vector data from watch");
        bp;
        std::string val = pChar->getValue();
        if (val.empty()) {
            uint8_t fail = 0x00;
            pChar->setValue(&fail, 1);
            pChar->notify();
            return;
        }

        ProxScanVector vec;
        if (!prox_deserialize_vector((const uint8_t*)val.data(), val.size(), vec)) {
            uint8_t fail = 0x00;
            pChar->setValue(&fail, 1);
            pChar->notify();
            return;
        }

        ProxScoreResult result = prox_compute_score(vec);

        // Update the Proximity Score characteristic before responding
        if (pProxScoreChar) {
            uint8_t score_buf[2];
            prox_get_last_score(score_buf);
            pProxScoreChar->setValue(score_buf, 2);
            pProxScoreChar->notify();
        }

        prox_maybe_update_fingerprint(vec, result);

        uint8_t ok = 0x01;
        pChar->setValue(&ok, 1);
        pChar->notify();
        Serial.printf("[PROX] Query: score=%d flags=0x%02X\n", result.score, result.flags);
    }
};

class FingerprintCtrlCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
         Serial.println("[PROX] Received fingerprint control command from app");
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t cmd  = (uint8_t)val[0];
        uint8_t resp = 0x00;
        bp;
        bp;
        if (cmd == 0x01 && val.size() >= 5) {           // BEGIN
            uint32_t total_len;
            memcpy(&total_len, val.data() + 1, 4);
            if (g_fp_buf) { free(g_fp_buf); g_fp_buf = nullptr; }
            g_fp_buf = (uint8_t*)malloc(total_len);
            if (g_fp_buf) {
                g_fp_expected_len = total_len;
                g_fp_received_len = 0;
                g_fp_xfer_active  = true;
                resp = 0x01;
            }
        } else if (cmd == 0x02 && val.size() >= 5) {    // END
            uint32_t crc_recv;
            memcpy(&crc_recv, val.data() + 1, 4);
            if (g_fp_xfer_active && g_fp_buf
                && g_fp_received_len == g_fp_expected_len) {
                uint32_t crc_calc = esp_crc32_le(0, g_fp_buf, g_fp_received_len);
                if (crc_calc == crc_recv) {
                    if (prox_load_fingerprint_blob(g_fp_buf, g_fp_received_len)) {
                        resp = 0x01;
                        Serial.println("[PROX] Fingerprint blob loaded from app");
                    }
                }
            }
            if (g_fp_buf) { free(g_fp_buf); g_fp_buf = nullptr; }
            g_fp_xfer_active = false;
        } else if (cmd == 0x03) {                        // ABORT
            if (g_fp_buf) { free(g_fp_buf); g_fp_buf = nullptr; }
            g_fp_xfer_active = false;
            resp = 0x01;
        }

        pChar->setValue(&resp, 1);
        pChar->notify();
    }
};

class FingerprintDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        if (!g_fp_xfer_active || !g_fp_buf) return;
        std::string val = pChar->getValue();
        uint32_t remaining = g_fp_expected_len - g_fp_received_len;
        uint32_t to_copy   = (uint32_t)val.size() < remaining
                               ? (uint32_t)val.size() : remaining;
        memcpy(g_fp_buf + g_fp_received_len, val.data(), to_copy);
        g_fp_received_len += to_copy;
    }
};

// ============================================================
//  BLE iBeacon advertising
// ============================================================

static void start_ble_advertising() {
    NimBLEAdvertising     *pAdv = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advData;

    // Custom manufacturer-specific data (Impulse company ID = 0xFFFF)
    // Using 0xFFFF instead of Apple's 0x004C so iOS passes the data through
    // to third-party apps rather than stripping it.
    uint8_t msd[25];
    msd[0] = 0xFF; msd[1] = 0xFF;  // Impulse (custom, not Apple)
    msd[2] = 0x02;                  // type marker (kept for parse compatibility)
    msd[3] = 0x15;                  // length marker (kept for parse compatibility)
    memcpy(msd + 4, g_uuid, 16);    // UUID big-endian
    msd[20] = 0x4A; msd[21] = 0x0F; // Major = 0x4A0F (Impulse namespace fingerprint)
    msd[22] = 0x00; msd[23] = 0x00; // Minor = 0
    msd[24] = 0xC5;                  // TX power (-59 dBm)

    advData.setManufacturerData(std::string((char*)msd, 25));
    pAdv->setAdvertisementData(advData);

    // Scan response: service UUID + anchor UUID as service data.
    // iOS strips Apple manufacturer data from scan results, so the phone app
    // identifies anchors via the service UUID instead. The 16-byte service data
    // payload carries this anchor's UUID so the app knows which anchor it found
    // without having to connect.
    NimBLEAdvertisementData scanResp;
    scanResp.addServiceUUID(ANCHOR_SERVICE_UUID);
    scanResp.setServiceData(NimBLEUUID(ANCHOR_SERVICE_UUID),
                            std::string((char*)g_uuid, 16));
    pAdv->setScanResponseData(scanResp);

    pAdv->setMinInterval(160);  // 100 ms  (160 × 0.625 ms)
    pAdv->setMaxInterval(160);
    pAdv->start();
}

// ============================================================
//  WiFi helpers
// ============================================================

static void setup_network_services() {
    MDNS.begin(g_uuid_str);
    g_udp.begin(ANCHOR_UDP_PORT);
    g_http_server.on("/schedule", HTTP_POST, handle_schedule_post);
    g_http_server.begin();
}

static void try_connect_wifi() {
    if (strlen(g_wifi_ssid) == 0) return;
    Serial.printf("[WiFi] Connecting to %s ...\n", g_wifi_ssid);
    WiFi.begin(g_wifi_ssid, g_wifi_pass);
    uint32_t t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(100);
    if (WiFi.status() == WL_CONNECTED) {
        g_wifi_connected = true;
        Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
        setup_network_services();
        configTime(0, 0, "pool.ntp.org");
    } else {
        Serial.println("[WiFi] Connection failed");
    }
}

// ============================================================
//  Factory reset  (hold FACTORY_RESET_PIN LOW for 5 s)
// ============================================================

static void check_factory_reset() {
    static uint32_t press_start = 0;
    static bool     was_pressed = false;
    bool pressed = (digitalRead(FACTORY_RESET_PIN) == LOW);
    if (pressed && !was_pressed) {
        press_start = millis();
        was_pressed = true;
    } else if (!pressed) {
        was_pressed = false;
    } else if (pressed && (millis() - press_start >= 5000)) {
        Serial.println("[SYS] Factory reset triggered!");
        stop_beeping();
        prefs.begin("anchor", false);
        prefs.clear();
        prefs.end();
        ESP.restart();
    }
}

// ============================================================
//  Midnight schedule clear
// ============================================================

static void check_midnight_clear() {
    struct tm ti;
    if (!getLocalTime(&ti)) return;
    if (ti.tm_mday != g_last_day) {
        if (g_last_day != -1) {
            g_event_count    = 0;
            g_schedule_loaded = false;
            stop_beeping();
            Serial.println("[SCHED] New day — schedule cleared");
        }
        g_last_day = ti.tm_mday;
    }
}


void indicate_sucessful_startup() {
    // 3 quick beeps
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        delay(200);
    }
}

// ============================================================
//  setup() / loop()
// ============================================================

void setup() {
    Serial.begin(115200);

    // disable brownout
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

    delay(200);
    Serial.println("\n\n[SYS] Starting Anchor Firmware");

    Serial.println("[SYS] 1/12 - Setting pin modes");
    pinMode(BUZZER_PIN, OUTPUT);
     Serial.println("[SYS] 1/12 - Setting pin modes A");
    pinMode(FACTORY_RESET_PIN,  INPUT_PULLUP);
     Serial.println("[SYS] 1/12 - Setting pin modes B");
    digitalWrite(BUZZER_PIN, LOW);
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);   // prevent floating signal before LEDC init

    // Boot servo to closed position then detach (spec §4.9 default state).
    ledcSetup(SERVO_LEDC_CHANNEL, SERVO_LEDC_FREQ, SERVO_LEDC_RES);
        Serial.println("[SYS] 1/12 - Setting pin modes C");
    ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
    ledcWrite(SERVO_LEDC_CHANNEL, degrees_to_duty(SERVO_CLOSED_DEGREES));
        Serial.println("[SYS] 1/12 - Setting pin modes D");

        Serial.println("[SYS] 1/12 - Setting pin modes E");
    ledcDetachPin(SERVO_PIN);
        Serial.println("[SYS] 1/12 - Setting pin modes F");
    g_servo_state = SERVO_CLOSED;
    Serial.println("[SERVO] Initialized to closed position");

    Serial.println("[SYS] 2/12 - Opening NVS");
    prefs.begin("anchor", true);
    bool has_uuid = prefs.isKey("uuid");
    if (has_uuid) prefs.getBytes("uuid", g_uuid, 16);
    strlcpy(g_wifi_ssid, prefs.getString("ssid", "").c_str(), sizeof(g_wifi_ssid));
    strlcpy(g_wifi_pass, prefs.getString("pass", "").c_str(), sizeof(g_wifi_pass));
    g_max_beep_minutes = prefs.getUShort("max_beep", ANCHOR_MAX_BEEP_MINUTES_DEFAULT);
    prefs.end();
    Serial.println("[SYS] 3/12 - NVS loaded");

    if (!has_uuid) {
        generate_uuid_v4(g_uuid);
        prefs.begin("anchor", false);
        prefs.putBytes("uuid", g_uuid, 16);
        prefs.end();
        Serial.println("[SYS] Generated new UUID");
    }
    uuid_to_str(g_uuid, g_uuid_str);
    Serial.printf("[SYS] Anchor UUID: %s\n", g_uuid_str);

    Serial.println("[SYS] 4/12 - NimBLE init");
    NimBLEDevice::init(g_uuid_str);

    // Initialise proximity engine with this anchor's own BLE MAC (big-endian)
    {
        uint8_t own_mac[6];
        esp_read_mac(own_mac, ESP_MAC_BT);
        prox_init(own_mac);
    }
    Serial.println("[SYS] 5/12 - Creating GATT server");
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    Serial.println("[SYS] 6/12 - Creating GATT service");
    NimBLEService *svc = pServer->createService(ANCHOR_SERVICE_UUID);

    Serial.println("[SYS] 7/12 - Creating characteristics");
    auto *pIdChar = svc->createCharacteristic(ANCHOR_IDENTIFY_CHAR_UUID,
                                               NIMBLE_PROPERTY::WRITE_NR);
    pIdChar->setCallbacks(new IdentifyCallback());

    pWifiCredChar = svc->createCharacteristic(ANCHOR_WIFI_CRED_CHAR_UUID,
                                               NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pWifiCredChar->setCallbacks(new WifiCredCallback());

    pSettingsChar = svc->createCharacteristic(ANCHOR_SETTINGS_CHAR_UUID,
                                               NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pSettingsChar->setCallbacks(new SettingsCallback());

    pSchedCtrlChar = svc->createCharacteristic(ANCHOR_SCHED_CTRL_CHAR_UUID,
                                                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pSchedCtrlChar->setCallbacks(new SchedCtrlCallback());

    auto *pSchedDataChar = svc->createCharacteristic(ANCHOR_SCHED_DATA_CHAR_UUID,
                                                      NIMBLE_PROPERTY::WRITE_NR);
    pSchedDataChar->setCallbacks(new SchedDataCallback());

    pToggleChar = svc->createCharacteristic(ANCHOR_TOGGLE_CHAR_UUID,
                                             NIMBLE_PROPERTY::READ |
                                             NIMBLE_PROPERTY::WRITE |
                                             NIMBLE_PROPERTY::NOTIFY);
    pToggleChar->setCallbacks(new ToggleCallback());
    // Initial value: closed (0x00)
    { uint8_t v = 0x00; pToggleChar->setValue(&v, 1); }

    // ── Proximity engine characteristics (v2) ──────────────────
    auto *pProxVecChar = svc->createCharacteristic(ANCHOR_PROX_VECTOR_CHAR_UUID,
                                                    NIMBLE_PROPERTY::WRITE |
                                                    NIMBLE_PROPERTY::NOTIFY);
    pProxVecChar->setCallbacks(new ProxVectorCallback());

    pProxScoreChar = svc->createCharacteristic(ANCHOR_PROX_SCORE_CHAR_UUID,
                                                NIMBLE_PROPERTY::READ |
                                                NIMBLE_PROPERTY::NOTIFY);
    { uint8_t init[2] = {0, 0}; pProxScoreChar->setValue(init, 2); }

    auto *pFpCtrlChar = svc->createCharacteristic(ANCHOR_FINGERPRINT_CTRL_CHAR_UUID,
                                                   NIMBLE_PROPERTY::WRITE |
                                                   NIMBLE_PROPERTY::NOTIFY);
    pFpCtrlChar->setCallbacks(new FingerprintCtrlCallback());

    auto *pFpDataChar = svc->createCharacteristic(ANCHOR_FINGERPRINT_DATA_CHAR_UUID,
                                                   NIMBLE_PROPERTY::WRITE_NR);
    pFpDataChar->setCallbacks(new FingerprintDataCallback());

    Serial.println("[SYS] 8/12 - Starting GATT service");
    svc->start();

    Serial.println("[SYS] 11/12 - Starting BLE advertising");
    start_ble_advertising();
    Serial.println("[BLE] Advertising started");

    Serial.println("[SYS] 12/12 - Setting WiFi mode + connecting");
    WiFi.mode(WIFI_STA);
    if (strlen(g_wifi_ssid) > 0) {
        Serial.printf("[WiFi] Saved SSID: %s\n", g_wifi_ssid);
        try_connect_wifi();
    } else {
        Serial.println("[WiFi] No saved credentials");
    }

    // Start proximity background scan tasks after WiFi mode is configured
    prox_begin_tasks();

    Serial.println("[SYS] Setup complete");

    indicate_sucessful_startup();
}


void loop() {
    check_factory_reset();
    check_midnight_clear();
    update_beeping();

    if (g_wifi_connected) {
        if (WiFi.status() != WL_CONNECTED) {
            g_wifi_connected     = false;
            g_last_wifi_retry_ms = millis();
        } else {
            g_http_server.handleClient();
            handle_udp();
        }
    } else if (strlen(g_wifi_ssid) > 0) {
        if (millis() - g_last_wifi_retry_ms >
            (uint32_t)ANCHOR_WIFI_RETRY_INTERVAL_S * 1000UL) {
            g_last_wifi_retry_ms = millis();
            try_connect_wifi();
        }
    }

    delay(10);
}
