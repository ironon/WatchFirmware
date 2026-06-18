// ============================================================
//  Watch Firmware  — ESP32-C3-WROOM
//  See firmware_spec_v2.md for full specification.
// ============================================================
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_crc.h>
#include <esp_random.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <time.h>
#include <string.h>
#include "imu.h"
#include "proximity.h"
#include <ArduinoLog.h>
#include <esp_system.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  Pin definitions
// ============================================================
#define BUZZER_PIN    21
#define VIBRO_PIN     10

#define BATT_ADC_PIN   3   // ADC, Vbat/2
// Worn detection — ITR8307 reflective opto-interrupter (replaces the old
// capacitive touch sensor). IR_EMIT drives the IR LED; IR_REC reads the
// phototransistor on an ADC-capable pin.
#define IR_REC  0          // ADC1_CH0 — phototransistor output
#define IR_EMIT 1          // IR LED drive (active HIGH)


// ============================================================
//  Debug mode  — set to 1 to enable audio debug indicators
//  Scan start  : 1 short beep
//  Anchor found: 2 quick beeps
// ============================================================
#define DEBUG_MODE 0

// ============================================================
//  Constants
// ============================================================
#define ENFORCEMENT_POLL_INTERVAL_S                60  // poll cadence while condition NOT met
#define ENFORCEMENT_POLL_INTERVAL_MET_S           180  // backed-off cadence while condition IS met (§8.2)
#define BLE_SCAN_INTERVAL_S                        60
#define ENFORCEMENT_SCAN_DURATION_MS              300
#define ENFORCEMENT_SCAN_INTERVAL_MS             1000  // BLE scanner slot interval during enforcement
#define ENFORCEMENT_SCAN_WINDOW_MS                100  // BLE scanner active window (~10% duty cycle)
#define ENFORCEMENT_QUERY_SCAN_DURATION_MS        700  // bounded scan run right before a proximity query
                                                       // to refresh the cache (aligns scan with query)
#define PROX_QUERY_SCAN_DUTY_MS                   100  // pre-query scan only: window == interval == ~100%
                                                       // duty for dense device capture (restored after)
#define ENFORCEMENT_IDLE_BEFORE_SLEEP_MS         30000
#define MINIMUM_BLE_DELAY_DORMANT               5000  // ms before re-arming IMU interrupt in DORMANT
#define MINIMUM_BLE_DELAY_ENFORCEMENT           3000  // ms before re-arming IMU interrupt in ENFORCEMENT
#define WIFI_SCAN_INTERVAL_S             120
#define DORMANT_TO_SLEEP_IDLE_MS        1500
// While disconnected, the watch surfaces from DORMANT_SLEEP at least this often
// to advertise and accept a phone connection (§8.4). Motion is NOT a wake source
// in DORMANT, so this heartbeat is the only way to reach a still, idle watch.
// Lower = snappier app connect, higher = less battery. Tunable.
#define DISCONNECTED_ADV_HEARTBEAT_MS   6000
#define ANCHOR_WIFI_RETRY_INTERVAL_S      30
#define ANCHOR_SEEN_TIMEOUT_S             20

#define DORMANT_SCAN_INTERVAL_MS         300
#define DORMANT_SCAN_WINDOW_MS           200

#define ANCHOR_UDP_PORT                 5555
#define UDP_RETRY_COUNT                    3
#define UDP_RETRY_TIMEOUT_MS             500
#define MDNS_RESOLVE_TIMEOUT_MS         1000

#define UNPAIRED_BEEP_DURATION_MS        200
#define UNPAIRED_VIBRATE_DURATION_MS     300

// Worn detection via ITR8307 reflective IR sensor (tune per hardware)
#define IR_WORN_DEBOUNCE_SAMPLES           5     // consecutive equal samples to flip state
#define IR_WORN_SAMPLE_INTERVAL_MS      1000     // ms between reflection samples
#define IR_EMIT_SETTLE_US                500     // emitter rise/settle before ADC read
#define IR_WORN_THRESHOLD                300     // min ambient-subtracted ADC delta to count as worn
#define IR_WORN_HIGHER_MEANS_WORN       true     // false if the readout is inverted (skin lowers the delta)

// Enforcement escalation
#define INTERVAL_DECREMENT_MS           2000

// BLE MTU
#define BLE_MTU_BYTES                     23

// Schedule
#define MAX_EVENTS_PER_DAY                64
#define MAX_ANCHOR_RECORDS                16
#define MAX_SEEN_ANCHORS                  32
#define MAX_WIFI_CREDS                     4
#define MAX_UNREACHABLE_QUEUE              8

// Continuous enforcement flag value
#define DURATION_CONTINUOUS       0xFFFFFFFFUL

// ============================================================
//  BLE advertising during dormant sleep
//  When true: BLE radio stays active (modem sleep) so the
//  mobile app can wake the watch via an incoming connection.
//  Increases sleep current; use a long ad interval to mitigate.
// ============================================================
#define ADVERTISE_DURING_SLEEP    false
#define SLEEP_ADV_MIN_INTERVAL_MS 1000   // ms  (1 s)
#define SLEEP_ADV_MAX_INTERVAL_MS 2000   // ms  (2 s)

// ============================================================
//  Battery event logger
// ============================================================
#define BATT_LOG_MAX_ENTRIES        128
#define BATT_HEARTBEAT_INTERVAL_MS 300000UL  // periodic voltage sample every 5 min

// ============================================================
//  Shared UUIDs  (must match anchor firmware and mobile app)
// ============================================================
#define ANCHOR_SERVICE_UUID               "4A0F0001-F8CE-11EE-8001-020304050607"
// Proximity engine characteristics (v2 — must match anchor firmware)
#define ANCHOR_PROX_VECTOR_CHAR_UUID      "4A0F0008-F8CE-11EE-8001-020304050607"
#define ANCHOR_PROX_SCORE_CHAR_UUID       "4A0F0009-F8CE-11EE-8001-020304050607"

#define WATCH_SERVICE_UUID           "4A0F0010-F8CE-11EE-8001-020304050607"
#define WATCH_WIFI_CRED_CHAR_UUID    "4A0F0011-F8CE-11EE-8001-020304050607"
#define WATCH_SCHED_CTRL_CHAR_UUID   "4A0F0012-F8CE-11EE-8001-020304050607"
#define WATCH_SCHED_DATA_CHAR_UUID   "4A0F0013-F8CE-11EE-8001-020304050607"
#define WATCH_SETTINGS_CHAR_UUID     "4A0F0014-F8CE-11EE-8001-020304050607"
#define WATCH_SEEN_ANCHORS_CHAR_UUID "4A0F0015-F8CE-11EE-8001-020304050607"
#define WATCH_STATUS_CHAR_UUID       "4A0F0016-F8CE-11EE-8001-020304050607"
#define WATCH_ANCHOR_IP_CHAR_UUID    "4A0F0017-F8CE-11EE-8001-020304050607"

// ============================================================
//  Enumerations
// ============================================================
enum RecurrenceType          : uint8_t { ONCE=0, DAILY=1, WEEKLY=2, MONTHLY=3 };
enum Criteria                : uint8_t { GET_AWAY=0, STAY_NEAR=1, GET_OFF_WIFI=2, GET_ON_WIFI=3 };
enum EnforcementProfile      : uint8_t {
    STRICT_SILENT=0, NORMAL_SILENT=1, LOOSE_SILENT=2,
    STRICT_BOTH=3,   NORMAL_BOTH=4,   LOOSE_BOTH=5,
    STRICT_BUZZ=6,   NORMAL_BUZZ=7,   LOOSE_BUZZ=8
};
enum AnchorEnforcementProfile : uint8_t { AP_LIGHT=0, AP_MEDIUM=1, AP_HARD=2 };
enum ActivityState            { STATE_UNPAIRED, STATE_DORMANT, STATE_DORMANT_SLEEP, STATE_ENFORCEMENT };

// ============================================================
//  Data structures
// ============================================================


struct Event {
    uint8_t  id[16];
    int64_t  referenceDate;
    uint16_t startTime;
    uint16_t endTime;
    RecurrenceType       recurrenceType;
    uint8_t  dayOfWeek;
    uint8_t  dayOfMonth;
    Criteria criteria;
    EnforcementProfile   profile;
    AnchorEnforcementProfile anchorProfile;
    bool     hasAnchorProfile;
    bool     negate;
    bool     hasAnchorId;
    uint8_t  anchorId[16];
    char     wifiSSID[65];
    uint8_t  beepAnchors[8][16];
    uint8_t  beepAnchorCount;
};

struct AnchorRecord {
    uint8_t  uuid[16];
    char     name[32];
    uint8_t  bleMac[6];      // BLE MAC address, big-endian (on-air). Populated on first ad seen.
    uint8_t  bleAddrType;    // NimBLE address type as advertised (PUBLIC/RANDOM); used for connect
    int8_t   lastRSSI;
    uint32_t lastSeen;       // Unix timestamp (updated by BLE scan)
    uint32_t ipAddress;      // network byte order; 0 = unknown
    uint32_t ipLastUpdated;
    uint8_t  lastProxScore;  // last proximity score from this anchor (not persisted)
    bool     valid;
    bool     bleMacValid;    // true once bleMac has been populated
};

struct SeenAnchor {
    uint8_t  uuid[16];
    uint8_t  bleMac[6];      // BLE MAC (big-endian, on-air) captured from advertisement
    uint8_t  bleAddrType;    // NimBLE address type as advertised (PUBLIC/RANDOM)
    bool     bleMacValid;
    int8_t   rssi;
    uint32_t lastSeenMs;
    bool     valid;
};

struct WifiCred {
    char ssid[64];
    char pass[64];
};

struct UnreachableNotification {
    uint8_t  anchor_uuid[16];
    char     anchor_name[32];
    uint32_t timestamp;
};

// ============================================================
//  Enforcement profile table
// ============================================================

struct ProfileStep {
    bool     motor_on;
    bool     buzzer_on;
    uint32_t duration_ms;   // DURATION_CONTINUOUS = runs until condition met
};

struct ProfileDefinition {
    EnforcementProfile id;
    bool               loops;
    ProfileStep        steps[2];
    int                step_count;
    uint32_t           floor_interval_ms;  // 0 = N/A
};

static const ProfileDefinition PROFILE_TABLE[] = {
    // strictSilent — motor on continuously
    { STRICT_SILENT, false,
      {{true,  false, DURATION_CONTINUOUS}, {false,false,0}}, 1, 0 },
    // normalSilent — 2s on, 30s off, floor 5s
    { NORMAL_SILENT, true,
      {{true,  false, 2000}, {false, false, 30000}}, 2, 5000 },
    // looseSilent — 2s on, 30s off, floor 10s
    { LOOSE_SILENT, true,
      {{true,  false, 2000}, {false, false, 30000}}, 2, 10000 },
    // strictBoth — motor+buzzer continuously
    { STRICT_BOTH, false,
      {{true,  true,  DURATION_CONTINUOUS}, {false,false,0}}, 1, 0 },
    // normalBoth
    { NORMAL_BOTH, true,
      {{true,  true,  2000}, {false, false, 30000}}, 2, 5000 },
    // looseBoth
    { LOOSE_BOTH, true,
      {{true,  true,  2000}, {false, false, 30000}}, 2, 10000 },
    // strictBuzz — buzzer continuously
    { STRICT_BUZZ, false,
      {{false, true,  DURATION_CONTINUOUS}, {false,false,0}}, 1, 0 },
    // normalBuzz
    { NORMAL_BUZZ, true,
      {{false, true,  2000}, {false, false, 30000}}, 2, 5000 },
    // looseBuzz
    { LOOSE_BUZZ, true,
      {{false, true,  2000}, {false, false, 30000}}, 2, 10000 },
};
static const int PROFILE_COUNT = sizeof(PROFILE_TABLE) / sizeof(PROFILE_TABLE[0]);

static const ProfileDefinition* find_profile(EnforcementProfile id) {
    for (int i = 0; i < PROFILE_COUNT; i++)
        if (PROFILE_TABLE[i].id == id) return &PROFILE_TABLE[i];
    return &PROFILE_TABLE[0];
}

// ============================================================
//  Global state
// ============================================================

static Preferences prefs;

// Identity
static uint8_t g_watch_uuid[16];
static char    g_watch_uuid_str[37];

// Activity state
static ActivityState g_activity_state = STATE_UNPAIRED;
static bool          g_is_paired      = false;

// Connectivity flags
static bool g_bt_connected   = false;
static bool g_wifi_connected = false;

// Settings
static bool    g_disconnected_is_dormant = true;
static bool    g_away_is_dormant         = true;
static int16_t g_tz_offset_min           = 0;   // minutes east of UTC

// Today's event list
static Event g_today_events[MAX_EVENTS_PER_DAY];
static int   g_today_event_count = 0;

// Active event (pointer into g_today_events)
static Event *g_active_event = nullptr;

// Raw schedule blob (stored in NVS, parsed on boot / on push)
static uint8_t  *g_sched_blob     = nullptr;
static uint32_t  g_sched_blob_len = 0;

// Anchor records  (app-managed)
static AnchorRecord g_anchor_records[MAX_ANCHOR_RECORDS];

// Seen anchors  (from BLE scanning)
static SeenAnchor g_seen_anchors[MAX_SEEN_ANCHORS];

// WiFi credentials list
static WifiCred g_wifi_creds[MAX_WIFI_CREDS];
static int      g_wifi_cred_count = 0;
static char     g_current_ssid[64] = {};

// Unreachable anchor notification queue
static UnreachableNotification g_unreachable_queue[MAX_UNREACHABLE_QUEUE];
static int                     g_unreachable_count = 0;

// Worn detection
static bool    g_worn                                    = false;
static uint8_t g_worn_buffer[IR_WORN_DEBOUNCE_SAMPLES]   = {};
static int     g_worn_buf_idx                            = 0;
static int     g_worn_buf_fill                           = 0;
static uint32_t g_last_worn_sample_ms                    = 0;

// Enforcement profile playback state
struct EnforcementState {
    int      step_idx;
    int      cycle_count;
    uint32_t current_wait_ms;    // decrements each cycle
    uint32_t step_start_ms;
    bool     condition_met;
    bool     active;
};
static EnforcementState g_enf = {};

// Timers
static uint32_t g_last_ble_scan_ms      = 0;
static uint32_t g_last_wifi_scan_ms     = 0;
static uint32_t g_last_enf_poll_ms      = 0;
static uint32_t g_last_wifi_retry_ms    = 0;
static uint32_t g_last_activity_ms      = 0;  // for DORMANT→SLEEP idle timer
// Timestamp of the last processed motion interrupt. Non-zero means INT1 is still
// latched HIGH and the re-arm is pending. Reset to 0 when lis3dh_clear_int1() fires.
static uint32_t g_last_motion_ms        = 0;

// BLE server / characteristics
static NimBLEServer         *g_ble_server        = nullptr;
static NimBLECharacteristic *g_seen_anchors_char = nullptr;
static NimBLECharacteristic *g_status_char       = nullptr;

// Schedule BLE transfer
static uint8_t  *g_sched_xfer_buf  = nullptr;
static uint32_t  g_sched_xfer_exp  = 0;
static uint32_t  g_sched_xfer_rcvd = 0;
static bool      g_sched_xfer_act  = false;

// UDP
static WiFiUDP g_udp;

// ---- Battery event log ring buffer ----
struct BattLogEntry {
    uint32_t millis_ts;
    float    voltage;
    char     event[40];
};
static BattLogEntry g_batt_log[BATT_LOG_MAX_ENTRIES];
static uint8_t      g_batt_log_head          = 0;   // oldest entry index
static uint8_t      g_batt_log_count         = 0;   // valid entry count (0–128)
static uint32_t     g_last_batt_heartbeat_ms = 0;

// ============================================================
//  UUID helpers
// ============================================================

static void uuid_to_str(const uint8_t *b, char *out) {
    sprintf(out,
        "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        b[0],b[1],b[2],b[3], b[4],b[5], b[6],b[7],
        b[8],b[9], b[10],b[11],b[12],b[13],b[14],b[15]);
}

String convertAnchoridToString(const uint8_t *anchorId) {
    char str[37];
    uuid_to_str(anchorId, str);
    return String(str);
}


static void generate_uuid_v4(uint8_t *out) {
    esp_fill_random(out, 16);
    out[6] = (out[6] & 0x0F) | 0x40;
    out[8] = (out[8] & 0x3F) | 0x80;
}

static double get_battery_voltage() {
    // pin 3 is connected to a 1/2 voltage divider from the battery, so multiply ADC reading by 2
    uint16_t adc_reading = analogRead(BATT_ADC_PIN);
    float voltage = adc_reading * (3.1 / 4095.0) * 2; // voltage divider
    return voltage;
}
static bool uuid_eq(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 16) == 0;
}

static bool uuid_is_zero(const uint8_t *a) {
    for (int i = 0; i < 16; i++) if (a[i]) return false;
    return true;
}

// ============================================================
//  Hardware output helpers
// ============================================================


static uint32_t voltage_to_percentage(uint32_t mv) {
    // Piecewise-linear lookup table mapping voltage (mV) to remaining capacity (%).
    // Points are derived from a typical 3.7V Li-ion discharge curve at moderate load.
    // The curve has a steep drop at full charge, a long flat plateau around 3.7–3.9V,
    // and another steep drop near empty — so a simple linear voltage scale would be
    // badly wrong. Interpolating between these points keeps the displayed percentage
    // proportional to actual energy remaining.
    static const struct { uint32_t mv; uint32_t pct; } curve[] = {
        { 4200, 100 },
        { 4150,  95 },
        { 4110,  90 },
        { 4080,  85 },
        { 4020,  80 },
        { 3980,  75 },
        { 3950,  70 },
        { 3910,  65 },
        { 3870,  60 },
        { 3830,  55 },
        { 3790,  50 },
        { 3750,  45 },
        { 3710,  40 },
        { 3670,  35 },
        { 3630,  30 },
        { 3590,  25 },
        { 3550,  20 },
        { 3510,  15 },
        { 3450,  10 },
        { 3370,   5 },
        { 3270,   0 },
    };
    static const int N = sizeof(curve) / sizeof(curve[0]);

    if (mv >= curve[0].mv)     return 100;
    if (mv <= curve[N - 1].mv) return 0;

    for (int i = 0; i < N - 1; i++) {
        if (mv <= curve[i].mv && mv > curve[i + 1].mv) {
            // Linear interpolation between the two surrounding points (integer only).
            uint32_t v_hi  = curve[i].mv;
            uint32_t v_lo  = curve[i + 1].mv;
            uint32_t p_hi  = curve[i].pct;
            uint32_t p_lo  = curve[i + 1].pct;
            return p_lo + (mv - v_lo) * (p_hi - p_lo) / (v_hi - v_lo);
        }
    }
    return 0;
}
// Returns battery voltage in millivolts. GPIO 3 carries Vbat/2.
// ADC_11db attenuation → ~2450mV full scale at raw 4095.
static uint16_t read_battery_mv() {
    int raw = analogRead(BATT_ADC_PIN) * 2;
    uint32_t pin_mv = (uint32_t)(raw * (3100.0 / 4095.0));

    uint32_t device_percentage = voltage_to_percentage(pin_mv);
    return pin_mv;
}

static void set_motor(bool on)  { digitalWrite(VIBRO_PIN,  on ? HIGH : LOW); }
// set_motor should do nothing for now
// static void set_motor(bool on)  { }
static void set_buzzer(bool on) { digitalWrite(BUZZER_PIN, on ? HIGH : LOW); }
static void outputs_off()       { set_motor(false); set_buzzer(false); }

// ============================================================
//  Forward declarations
// ============================================================
static void recalculate_and_rearm();
static void push_watch_status();
static void batt_log(const char *event);
static void notify_seen_anchors();
static void enter_enforcement(Event *e);
static struct AnchorRecord *ensure_anchor_record(const uint8_t *uuid);
static void exit_enforcement();
static void check_enforcement_condition();
static bool is_enforcement_condition_met(const Event *e);
static void send_watch_removed_to_anchors(const Event *e);
static void send_watch_worn_to_anchors(const Event *e);
static void on_worn_state_changed(bool is_worn);

// ============================================================
//  Schedule deserialization
// ============================================================

static bool deserialize_events(const uint8_t *data, uint32_t len,
                                Event *out, int *count, int max_count) {
    if (len < 2) return false;
    uint32_t pos = 0;
    uint16_t n;
    memcpy(&n, data + pos, 2); pos += 2;
    if (n > (uint16_t)max_count) n = (uint16_t)max_count;

    for (int i = 0; i < (int)n; i++) {
        Event &e = out[i];
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
        e.hasAnchorProfile = (ap != 0xFF);
        e.anchorProfile    = e.hasAnchorProfile ? (AnchorEnforcementProfile)ap : AP_LIGHT;

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
            uint8_t cl = ssid_len < 64 ? ssid_len : 64;
            memcpy(e.wifiSSID, data + pos, cl);
            e.wifiSSID[cl] = 0;
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
//  Local time helpers
// ============================================================

// Returns local Unix timestamp for current time
static time_t local_now() {
    return time(nullptr) + (time_t)g_tz_offset_min * 60;
}

// Returns minutes since local midnight for the current moment
static uint32_t local_minutes_now() {
    time_t lt = local_now();
    struct tm ti;
    gmtime_r(&lt, &ti);
    return (uint32_t)(ti.tm_hour * 60 + ti.tm_min);
}

// Returns seconds until a given local-time minute-offset from midnight
// Returns 0 if already past
static int32_t seconds_until_minute(uint32_t target_min) {
    uint32_t now_min = local_minutes_now();
    if (target_min <= now_min) return 0;
    return (int32_t)((target_min - now_min) * 60);
}

// day-of-week: 1=Mon, 7=Sun (matches spec)
static uint8_t local_day_of_week() {
    time_t lt = local_now();
    struct tm ti;
    gmtime_r(&lt, &ti);
    // tm_wday: 0=Sun, 1=Mon...6=Sat → remap to 1-Mon..7-Sun
    return (ti.tm_wday == 0) ? 7 : (uint8_t)ti.tm_wday;
}

static uint8_t local_day_of_month() {
    time_t lt = local_now();
    struct tm ti;
    gmtime_r(&lt, &ti);
    return (uint8_t)ti.tm_mday;
}

// Returns the local calendar date as a comparable integer: YYYYMMDD
static int32_t local_date_int() {
    time_t lt = local_now();
    struct tm ti;
    gmtime_r(&lt, &ti);
    return (ti.tm_year + 1900) * 10000 + (ti.tm_mon + 1) * 100 + ti.tm_mday;
}

// Returns YYYYMMDD for a given UTC Unix timestamp, in local time
static int32_t date_int_of_utc(int64_t utc_ts) {
    time_t lt = (time_t)utc_ts + (time_t)g_tz_offset_min * 60;
    struct tm ti;
    gmtime_r(&lt, &ti);
    return (ti.tm_year + 1900) * 10000 + (ti.tm_mon + 1) * 100 + ti.tm_mday;
}

// ============================================================
//  Schedule recalculation
// ============================================================

// All raw events are stored in g_sched_blob. This function filters them
// for today and stores the result in g_today_events.
static void recalculate_day() {
    g_today_event_count = 0;
    if (!g_sched_blob || g_sched_blob_len < 2) return;

    // Parse all events
    static Event all_events[MAX_EVENTS_PER_DAY * 2];
    int all_count = 0;
    if (!deserialize_events(g_sched_blob, g_sched_blob_len,
                             all_events, &all_count, MAX_EVENTS_PER_DAY * 2))
        return;

    int32_t today = local_date_int();
    uint8_t dow   = local_day_of_week();
    uint8_t dom   = local_day_of_month();

    // Step 1+2: collect recurring and specific events that apply today
    // Use a simple merge map (find by UUID, overwrite)
    static Event merged[MAX_EVENTS_PER_DAY];
    static bool  merged_valid[MAX_EVENTS_PER_DAY];
    static uint8_t merged_ids[MAX_EVENTS_PER_DAY][16];
    int merged_count = 0;

    auto find_merged_slot = [&](const uint8_t *id) -> int {
        for (int i = 0; i < merged_count; i++)
            if (merged_valid[i] && uuid_eq(merged_ids[i], id)) return i;
        return -1;
    };
    auto add_or_replace = [&](const Event &e) {
        int slot = find_merged_slot(e.id);
        if (slot < 0 && merged_count < MAX_EVENTS_PER_DAY) {
            slot = merged_count++;
            merged_valid[slot] = true;
            memcpy(merged_ids[slot], e.id, 16);
        }
        if (slot >= 0) merged[slot] = e;
    };

    // Recurring events
    for (int i = 0; i < all_count; i++) {
        const Event &e = all_events[i];
        bool applies = false;
        switch (e.recurrenceType) {
            case DAILY:   applies = true; break;
            case WEEKLY:  applies = (e.dayOfWeek  == dow); break;
            case MONTHLY: applies = (e.dayOfMonth == dom); break;
            case ONCE:    applies = false; break;
        }
        if (applies) add_or_replace(e);
    }

    // Specific (once) events for today
    for (int i = 0; i < all_count; i++) {
        const Event &e = all_events[i];
        if (e.recurrenceType == ONCE && date_int_of_utc(e.referenceDate) == today)
            add_or_replace(e);
    }

    // Step 4: remove negated events; Step 5: copy to output
    g_today_event_count = 0;
    for (int i = 0; i < merged_count; i++) {
        if (!merged_valid[i] || merged[i].negate) continue;
        if (g_today_event_count < MAX_EVENTS_PER_DAY)
            g_today_events[g_today_event_count++] = merged[i];
    }

    // Sort by startTime (simple insertion sort for small N)
    for (int i = 1; i < g_today_event_count; i++) {
        Event key = g_today_events[i];
        int j = i - 1;
        while (j >= 0 && g_today_events[j].startTime > key.startTime) {
            g_today_events[j + 1] = g_today_events[j];
            j--;
        }
        g_today_events[j + 1] = key;
    }

    // Ensure an AnchorRecord exists for every anchor referenced by today's
    // schedule, so the proximity engine and beep delivery can reach them over
    // BLE without waiting on the app's IP push (§3.3). IP remains optional.
    for (int i = 0; i < g_today_event_count; i++) {
        const Event &e = g_today_events[i];
        if (e.hasAnchorId) ensure_anchor_record(e.anchorId);
        for (int j = 0; j < e.beepAnchorCount; j++)
            ensure_anchor_record(e.beepAnchors[j]);
    }
}

// ============================================================
//  Schedule NVS persistence
// ============================================================

static void save_schedule_blob(const uint8_t *data, uint32_t len) {
    prefs.begin("schedule", false);
    prefs.putBytes("blob", data, len);
    prefs.putUInt("blob_len", len);
    prefs.end();
}

static void load_schedule_blob() {
    prefs.begin("schedule", true);
    uint32_t len = prefs.getUInt("blob_len", 0);
    if (len > 0) {
        if (g_sched_blob) free(g_sched_blob);
        g_sched_blob = (uint8_t*)malloc(len);
        if (g_sched_blob) {
            prefs.getBytes("blob", g_sched_blob, len);
            g_sched_blob_len = len;
        }
    }
    prefs.end();
}

// ============================================================
//  RTC / event boundary arming
// ============================================================
//
// The watch uses millis()-based timers for upcoming event
// boundaries in the same day. For DORMANT_SLEEP, the timer
// wakeup fires via esp_sleep_enable_timer_wakeup().

static uint32_t g_next_boundary_min = 0xFFFF; // minutes since midnight

// Compute the next event boundary (start or end) strictly after 'now_min'
static uint32_t next_boundary_after(uint32_t now_min) {
    uint32_t best = 0xFFFF;
    for (int i = 0; i < g_today_event_count; i++) {
        if (g_today_events[i].startTime > now_min && g_today_events[i].startTime < best)
            best = g_today_events[i].startTime;
        if (g_today_events[i].endTime > now_min && g_today_events[i].endTime < best)
            best = g_today_events[i].endTime;
    }
    return best;
}

static void arm_next_boundary() {
    uint32_t now_min       = local_minutes_now();
    g_next_boundary_min    = next_boundary_after(now_min);
}

// ============================================================
//  BLE scan (watch as central — looking for anchor iBeacons)
// ============================================================

static NimBLEScan *g_ble_scan = nullptr;

static void update_seen_anchor(const uint8_t *uuid, const uint8_t *mac_be,
                               uint8_t addr_type, int rssi) {
    uint32_t now_ts = (uint32_t)time(nullptr);

    // Update anchor_records if this UUID is known
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, uuid)) {
            Serial.println("[SCAN] Updating anchor record for " + String(g_anchor_records[i].name) + " with RSSI " + String(rssi));
            if (mac_be && !g_anchor_records[i].bleMacValid) {
                memcpy(g_anchor_records[i].bleMac, mac_be, 6);
                g_anchor_records[i].bleAddrType = addr_type;
                g_anchor_records[i].bleMacValid = true;
            }
            g_anchor_records[i].lastRSSI = (int8_t)rssi;
            g_anchor_records[i].lastSeen = now_ts;
            return;
        }
    }

    // Otherwise track in seen_anchors
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++) {
        if (g_seen_anchors[i].valid && uuid_eq(g_seen_anchors[i].uuid, uuid)) {
            Serial.println("[SCAN] Updating seen anchor with RSSI "+ convertAnchoridToString(g_seen_anchors[i].uuid) + "   RSSI:" + String(rssi));
            if (mac_be && !g_seen_anchors[i].bleMacValid) {
                memcpy(g_seen_anchors[i].bleMac, mac_be, 6);
                g_seen_anchors[i].bleAddrType = addr_type;
                g_seen_anchors[i].bleMacValid = true;
            }
            g_seen_anchors[i].rssi       = (int8_t)rssi;
            g_seen_anchors[i].lastSeenMs = millis();
            return;
        }
    }

    // Newly discovered anchor
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++) {
        if (!g_seen_anchors[i].valid) {
            g_seen_anchors[i].valid      = true;
            memcpy(g_seen_anchors[i].uuid, uuid, 16);
            if (mac_be) {
                memcpy(g_seen_anchors[i].bleMac, mac_be, 6);
                g_seen_anchors[i].bleAddrType = addr_type;
                g_seen_anchors[i].bleMacValid = true;
            } else {
                g_seen_anchors[i].bleMacValid = false;
            }
            g_seen_anchors[i].rssi       = (int8_t)rssi;
            g_seen_anchors[i].lastSeenMs = millis();
            if (g_bt_connected) notify_seen_anchors();
            break;
        }
    }
}

// Ensure an AnchorRecord exists for the given anchor UUID, creating one in a
// free slot if necessary. This decouples record existence from the app's IP
// push (§3.3): any anchor the schedule references becomes a first-class record
// so the proximity engine can connect to it over BLE. If the anchor's BLE MAC
// was already captured during discovery (seen_anchors), copy it in immediately
// so the first proximity poll doesn't have to wait for the next advertisement.
// Returns the record, or nullptr if the record table is full.
static AnchorRecord *ensure_anchor_record(const uint8_t *uuid) {
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, uuid))
            return &g_anchor_records[i];
    }

    int slot = -1;
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (!g_anchor_records[i].valid) { slot = i; break; }
    }
    if (slot < 0) {
        Serial.println("[PROX] ensure_anchor_record: record table full");
        return nullptr;
    }

    AnchorRecord &r = g_anchor_records[slot];
    r = AnchorRecord{};            // zero-init: ipAddress=0, bleMacValid=false, etc.
    memcpy(r.uuid, uuid, 16);
    r.valid = true;

    // Promote a previously-seen anchor's captured MAC/RSSI, if available.
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++) {
        if (g_seen_anchors[i].valid && uuid_eq(g_seen_anchors[i].uuid, uuid)) {
            if (g_seen_anchors[i].bleMacValid) {
                memcpy(r.bleMac, g_seen_anchors[i].bleMac, 6);
                r.bleAddrType = g_seen_anchors[i].bleAddrType;
                r.bleMacValid = true;
            }
            r.lastRSSI = g_seen_anchors[i].rssi;
            r.lastSeen = (uint32_t)time(nullptr);
            break;
        }
    }

    Serial.println("[PROX] Created anchor record for " + convertAnchoridToString(uuid) +
                   (r.bleMacValid ? " (MAC known)" : " (awaiting advertisement)"));
    return &r;
}

void beep(int count) {
    for (int i = 0; i < count; i++) {
        set_buzzer(true);
        delay(100);
        set_buzzer(false);
        delay(100);
    }
}

class WatchScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        int8_t rssi = (int8_t)dev->getRSSI();

        // Convert NimBLE little-endian address to big-endian (on-air) format.
        // getBase()->val holds the 6-byte address in NimBLE's internal (LE) order.
        const uint8_t *native = dev->getAddress().getBase()->val;
        uint8_t mac_be[6];
        for (int i = 0; i < 6; i++) mac_be[i] = native[5 - i];
        uint8_t addr_type = dev->getAddress().getType();

        // Feed every BLE device into the proximity scan cache (§5.4 — ProxScanVector
        // includes all observed BLE devices, not just Impulse anchors).
        prox_update_ble_cache(mac_be, 0, rssi);

        // ── Impulse anchor identification (unchanged logic) ──────
        if (!dev->haveManufacturerData()) return;
        std::string mfr = dev->getManufacturerData();

#if DEBUG_MODE
        Serial.printf("[SCAN] Device: \"%s\" (%s)  mfr_len=%d  bytes: %02X %02X %02X %02X\n",
            dev->getName().c_str(),
            dev->getAddress().toString().c_str(),
            (int)mfr.size(),
            mfr.size() > 0 ? (uint8_t)mfr[0] : 0,
            mfr.size() > 1 ? (uint8_t)mfr[1] : 0,
            mfr.size() > 2 ? (uint8_t)mfr[2] : 0,
            mfr.size() > 3 ? (uint8_t)mfr[3] : 0);
#endif

        if (mfr.size() < 25) return;
        if ((uint8_t)mfr[0] != 0xFF || (uint8_t)mfr[1] != 0xFF) return;
        if ((uint8_t)mfr[2] != 0x02 || (uint8_t)mfr[3] != 0x15) return;
        // Major field must equal 0x4A0F — Impulse namespace fingerprint
        if ((uint8_t)mfr[20] != 0x4A || (uint8_t)mfr[21] != 0x0F) return;

        uint8_t anchor_uuid[16];
        memcpy(anchor_uuid, mfr.data() + 4, 16);

        Serial.printf("[SCAN] Impulse anchor confirmed! RSSI=%d\n", rssi);

#if DEBUG_MODE
        digitalWrite(BUZZER_PIN, HIGH); delay(60);
        digitalWrite(BUZZER_PIN, LOW);  delay(80);
        digitalWrite(BUZZER_PIN, HIGH); delay(60);
        digitalWrite(BUZZER_PIN, LOW);
#endif

        // Populate bleMac in the AnchorRecord if we know this anchor
        for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
            if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, anchor_uuid)) {
                if (!g_anchor_records[i].bleMacValid) {
                    memcpy(g_anchor_records[i].bleMac, mac_be, 6);
                    g_anchor_records[i].bleAddrType = addr_type;
                    g_anchor_records[i].bleMacValid = true;
                }
                g_anchor_records[i].lastRSSI = rssi;
                g_anchor_records[i].lastSeen = (uint32_t)time(nullptr);
                return;
            }
        }

        update_seen_anchor(anchor_uuid, mac_be, addr_type, rssi);
    }
};

static WatchScanCallbacks g_scan_cb;

// duration_ms: 0 = scan indefinitely (used in DORMANT for thorough discovery).
//              >0 = bounded scan (used in ENFORCEMENT; 300 ms covers 3 anchor ad windows).
static void start_ble_scan(uint32_t duration_ms = 300) {
    if (!g_ble_scan) return;
    if (g_ble_scan->isScanning()) return;
#if DEBUG_MODE
    digitalWrite(BUZZER_PIN, HIGH); delay(80);
    digitalWrite(BUZZER_PIN, LOW);
#endif
    g_ble_scan->start(duration_ms, false, true);
    g_last_ble_scan_ms = millis();
    batt_log("ble_scan_start");
}

// One-shot ACTIVE, full-duty BLE scan to densely populate the proximity cache
// immediately before a query, then restore the low-power passive enforcement
// scan settings. Active scanning (which transmits scan requests and costs more
// power) is used ONLY here — never for the background discovery/duty-cycle scans.
// Blocking: returns once the bounded scan completes.
static void prox_aligned_active_scan(uint32_t duration_ms) {
    if (!g_ble_scan) return;
    if (g_ble_scan->isScanning()) g_ble_scan->stop();

    // Temporarily crank to active + ~100% duty (window == interval).
    g_ble_scan->setActiveScan(true);
    g_ble_scan->setInterval(PROX_QUERY_SCAN_DUTY_MS);
    g_ble_scan->setWindow(PROX_QUERY_SCAN_DUTY_MS);

    g_ble_scan->start(duration_ms, false, true);
    g_last_ble_scan_ms = millis();
    batt_log("prox_active_scan");
    uint32_t t0 = millis();
    while (g_ble_scan->isScanning() && millis() - t0 < duration_ms + 200) {
        delay(10);
    }
    if (g_ble_scan->isScanning()) g_ble_scan->stop();

    // Restore the low-power passive enforcement scan configuration.
    g_ble_scan->setActiveScan(false);
    g_ble_scan->setInterval(ENFORCEMENT_SCAN_INTERVAL_MS);
    g_ble_scan->setWindow(ENFORCEMENT_SCAN_WINDOW_MS);
}

// ============================================================
//  RSSI lookup for a given anchor UUID
// ============================================================

// Returns RSSI of most recent advertisement, or INT8_MIN if not seen recently
static int8_t get_anchor_rssi(const uint8_t *uuid) {
    uint32_t now_ts = (uint32_t)time(nullptr);
    // Check anchor_records first (RSSI updated via scan callback)
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (!g_anchor_records[i].valid) continue;
        if (!uuid_eq(g_anchor_records[i].uuid, uuid)) continue;
        if (now_ts - g_anchor_records[i].lastSeen > ANCHOR_SEEN_TIMEOUT_S) return INT8_MIN;
        return g_anchor_records[i].lastRSSI;
    }
    // Check seen_anchors
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++) {
        if (!g_seen_anchors[i].valid) continue;
        if (!uuid_eq(g_seen_anchors[i].uuid, uuid)) continue;
        if (millis() - g_seen_anchors[i].lastSeenMs > (uint32_t)ANCHOR_SEEN_TIMEOUT_S * 1000) return INT8_MIN;
        return g_seen_anchors[i].rssi;
    }
    return INT8_MIN;
}

// ============================================================
//  WiFi helpers
// ============================================================

static void try_connect_saved_wifi() {
    if (g_wifi_cred_count == 0) return;
    // Attempt each saved credential
    for (int i = 0; i < g_wifi_cred_count; i++) {
        WiFi.begin(g_wifi_creds[i].ssid, g_wifi_creds[i].pass);
        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 5000) delay(50);
        if (WiFi.status() == WL_CONNECTED) {
            g_wifi_connected = true;
            strlcpy(g_current_ssid, g_wifi_creds[i].ssid, sizeof(g_current_ssid));
            g_udp.begin(0); // any local port for sending
            configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");
            Serial.printf("[WiFi] Connected to %s\n", g_current_ssid);
            batt_log("wifi_connect");
            return;
        }
    }
}

static void save_wifi_creds() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < g_wifi_cred_count; i++) {
        JsonObject o = arr.add<JsonObject>();
        o["s"] = g_wifi_creds[i].ssid;
        o["p"] = g_wifi_creds[i].pass;
    }
    char buf[512];
    serializeJson(doc, buf, sizeof(buf));
    prefs.begin("watch", false);
    prefs.putString("wifi_creds", buf);
    prefs.end();
}

static void load_wifi_creds() {
    prefs.begin("watch", true);
    String s = prefs.getString("wifi_creds", "[]");
    prefs.end();
    JsonDocument doc;
    if (deserializeJson(doc, s)) return;
    JsonArray arr = doc.as<JsonArray>();
    g_wifi_cred_count = 0;
    for (JsonObject o : arr) {
        if (g_wifi_cred_count >= MAX_WIFI_CREDS) break;
        strlcpy(g_wifi_creds[g_wifi_cred_count].ssid, o["s"] | "", 63);
        strlcpy(g_wifi_creds[g_wifi_cred_count].pass, o["p"] | "", 63);
        g_wifi_cred_count++;
    }
}

// ============================================================
//  UDP to anchors
// ============================================================

// Encode the 33-byte UDP command packet into buf
static void encode_udp_cmd(uint8_t *buf, uint8_t cmd,
                             const uint8_t *watch_uuid, const uint8_t *event_uuid) {
    buf[0] = cmd;
    memcpy(buf + 1,  watch_uuid, 16);
    memcpy(buf + 17, event_uuid, 16);
}

// Attempt UDP send with retries; returns true on any successful delivery
static bool udp_send_with_retry(uint32_t ip_nbo, uint16_t port,
                                 const uint8_t *payload, int len) {
    IPAddress ip(ip_nbo);  // network byte order → IPAddress accepts uint32_t host order
    // Convert NBO to host order
    uint32_t host_order = ntohl(ip_nbo);
    IPAddress dest(host_order);
    for (int attempt = 0; attempt < UDP_RETRY_COUNT; attempt++) {
        if (g_udp.beginPacket(dest, port)) {
            g_udp.write(payload, len);
            if (g_udp.endPacket()) return true;
        }
        delay(UDP_RETRY_TIMEOUT_MS);
    }
    return false;
}

static void queue_unreachable(const uint8_t *anchor_uuid, const char *name) {
    if (g_unreachable_count >= MAX_UNREACHABLE_QUEUE) return;
    UnreachableNotification &n = g_unreachable_queue[g_unreachable_count++];
    memcpy(n.anchor_uuid, anchor_uuid, 16);
    strlcpy(n.anchor_name, name ? name : "", sizeof(n.anchor_name));
    n.timestamp = (uint32_t)time(nullptr);
}

static void send_to_anchors(uint8_t cmd, const Event *e) {
    if (!g_wifi_connected) return;
    uint8_t pkt[33];
    encode_udp_cmd(pkt, cmd, g_watch_uuid, e->id);

    for (int ai = 0; ai < e->beepAnchorCount; ai++) {
        const uint8_t *anchor_uuid = e->beepAnchors[ai];
        bool success = false;

        // Attempt 1: known IP from anchor_records
        AnchorRecord *rec = nullptr;
        for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
            if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, anchor_uuid)) {
                rec = &g_anchor_records[i];
                break;
            }
        }
        if (rec && rec->ipAddress != 0) {
            success = udp_send_with_retry(rec->ipAddress, ANCHOR_UDP_PORT, pkt, 33);
        }

        // Attempt 2: mDNS fallback
        if (!success) {
            char mdns_name[64];
            char uuid_str[37];
            uuid_to_str(anchor_uuid, uuid_str);
            snprintf(mdns_name, sizeof(mdns_name), "%s.local", uuid_str);
            IPAddress resolved;
            if (MDNS.queryHost(mdns_name, MDNS_RESOLVE_TIMEOUT_MS)) {
                // MDNS.queryHost doesn't return IP directly on all versions;
                // use the hosts list approach
                int n_hosts = MDNS.queryHost(mdns_name);
                if (n_hosts > 0) {
                    // not all Arduino ESPmDNS APIs return IP from queryHost this way;
                    // fallback to broadcast if mDNS doesn't resolve
                }
            }
            // Attempt 3: broadcast (best-effort)
            if (!success) {
                IPAddress broadcast(255, 255, 255, 255);
                if (g_udp.beginPacket(broadcast, ANCHOR_UDP_PORT)) {
                    g_udp.write(pkt, 33);
                    g_udp.endPacket();
                    // broadcast = best effort, no success tracking
                }
                queue_unreachable(anchor_uuid, rec ? rec->name : "");
            }
        }
    }
}

static void send_watch_removed_to_anchors(const Event *e) { send_to_anchors(0x01, e); }
static void send_watch_worn_to_anchors(const Event *e)    { send_to_anchors(0x02, e); }

// ============================================================
//  Watch Status characteristic
// ============================================================

static void push_watch_status() {
    if (!g_status_char) return;

    // Build payload
    uint8_t buf[256];
    int pos = 0;

    // activity_state (0=dormant, 1=enforcement, 2=dormant_sleep)
    uint8_t act = (g_activity_state == STATE_DORMANT)       ? 0 :
                  (g_activity_state == STATE_ENFORCEMENT)   ? 1 :
                  (g_activity_state == STATE_DORMANT_SLEEP) ? 2 : 0;
    buf[pos++] = act;
    buf[pos++] = g_bt_connected   ? 1 : 0;
    buf[pos++] = g_wifi_connected ? 1 : 0;
    buf[pos++] = g_worn           ? 1 : 0;
    uint16_t batt_mv = read_battery_mv();
    Serial.printf("[STATUS] activity=%d  BT=%d  WiFi=%d  worn=%d  batt_mv=%d\n",
        act, g_bt_connected, g_wifi_connected, g_worn, batt_mv);
    // print what time it is in local time, and events on the schedule today
    {   
        // if no wifi, we may not have a valid time yet, so local_now() may be wrong. print a warning about this
        if (!g_wifi_connected) {
            Serial.println("[STATUS] WARNING: WiFi not connected, local time may be inaccurate");
        }

        time_t lt = local_now();
        struct tm ti;
        gmtime_r(&lt, &ti);
        Serial.printf("[STATUS] local %04d-%02d-%02d %02d:%02d:%02d\n",
                      ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                      ti.tm_hour, ti.tm_min, ti.tm_sec);

        if (g_today_event_count == 0) {
            Serial.println("[SCHED] No events today");
        } else {
            for (int i = 0; i < g_today_event_count; i++) {
                const Event &e = g_today_events[i];
                int sh = e.startTime / 60;
                int sm = e.startTime % 60;
                int eh = e.endTime / 60;
                int em = e.endTime % 60;
                const char *crit = "?";
                switch (e.criteria) {
                    case GET_AWAY:    crit = "GET_AWAY"; break;
                    case STAY_NEAR:   crit = "STAY_NEAR"; break;
                    case GET_OFF_WIFI:crit = "GET_OFF_WIFI"; break;
                    case GET_ON_WIFI: crit = "GET_ON_WIFI"; break;
                }
                char id_str[37]; uuid_to_str(e.id, id_str);
                Serial.printf("[SCHED] %02d:%02d-%02d:%02d id=%s crit=%s prof=%d",
                              sh, sm, eh, em, id_str, crit, (int)e.profile);
                if (e.hasAnchorId) {
                    Serial.print(" anchor=");
                    Serial.println(convertAnchoridToString(e.anchorId));
                } else if (e.criteria == GET_ON_WIFI || e.criteria == GET_OFF_WIFI) {
                    Serial.print(" ssid="); Serial.println(e.wifiSSID);
                } else {
                    Serial.println();
                }

                if (e.beepAnchorCount > 0) {
                    for (int j = 0; j < e.beepAnchorCount; j++) {
                        char bstr[37]; uuid_to_str(e.beepAnchors[j], bstr);
                        Serial.printf("[SCHED]   beep anchor %d: %s\n", j, bstr);
                    }
                }
            }
        }
    }
    
    memcpy(buf + pos, &batt_mv, 2); pos += 2;  // battery millivolts, little-endian

    // active event UUID (zeros if none)
    if (g_active_event && !uuid_is_zero(g_active_event->id)) {
        memcpy(buf + pos, g_active_event->id, 16);
    } else {
        memset(buf + pos, 0, 16);
    }
    pos += 16;

    buf[pos++] = (uint8_t)g_unreachable_count;
    for (int i = 0; i < g_unreachable_count; i++) {
        const UnreachableNotification &n = g_unreachable_queue[i];
        memcpy(buf + pos, n.anchor_uuid, 16); pos += 16;
        uint8_t nlen = (uint8_t)strlen(n.anchor_name);
        buf[pos++] = nlen;
        memcpy(buf + pos, n.anchor_name, nlen); pos += nlen;
        uint32_t ts = n.timestamp;
        memcpy(buf + pos, &ts, 4); pos += 4;
    }

    g_status_char->setValue(buf, pos);
    if (g_bt_connected) {
        g_status_char->notify();
        // Clear unreachable queue after delivery
        g_unreachable_count = 0;
    }
}

// ============================================================
//  Seen Anchors characteristic
// ============================================================

static void notify_seen_anchors() {
    if (!g_seen_anchors_char) return;
    Serial.printf("[BLE] notify_seen_anchors — bt_connected=%d\n", g_bt_connected);
    uint8_t buf[1 + MAX_SEEN_ANCHORS * 21];
    int pos = 0;
    int count = 0;
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++)
        if (g_seen_anchors[i].valid) count++;

    buf[pos++] = (uint8_t)count;
    for (int i = 0; i < MAX_SEEN_ANCHORS; i++) {
        if (!g_seen_anchors[i].valid) continue;
        memcpy(buf + pos, g_seen_anchors[i].uuid, 16); pos += 16;
        // RSSI encoded as rssi + 128
        buf[pos++] = (uint8_t)(g_seen_anchors[i].rssi + 128);
        // last_seen as uint32
        uint32_t ls = (uint32_t)time(nullptr);
        memcpy(buf + pos, &ls, 4); pos += 4;
    }
    g_seen_anchors_char->setValue(buf, pos);
    if (g_bt_connected) g_seen_anchors_char->notify();
}

// ============================================================
//  Enforcement condition checking
// ============================================================


static bool is_enforcement_condition_met(const Event *e) {
    if (!e) return false;
    switch (e->criteria) {
        case STAY_NEAR:
        case GET_AWAY: {
            // Find the anchor record to get its BLE MAC
            AnchorRecord *rec = nullptr;
            for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
                Serial.printf("[PROX] Checking anchor record %d: valid=%d  uuid=%s\n", i, g_anchor_records[i].valid, convertAnchoridToString(g_anchor_records[i].uuid).c_str());
                if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, e->anchorId)) {
                    rec = &g_anchor_records[i];
                    break;
                }
            }

            if (!rec || !rec->bleMacValid) {
                // Unknown anchor or no MAC yet — treat as AMBIGUOUS and apply fail-safe
                Serial.println("[PROX] Anchor MAC unknown — treating as AMBIGUOUS");
                return (e->criteria == STAY_NEAR); // stayNear: AMBIGUOUS→NEAR (met); getAway: AMBIGUOUS→AWAY (met)
            }

            // Align a fresh BLE scan with this query so the cache reflects 'now'
            // rather than relying on a possibly-stale periodic fallback scan.
            // Uses a one-shot active, full-duty scan for dense capture (restored
            // to low-power passive afterward); runs once per poll, blocking.
            prox_aligned_active_scan(ENFORCEMENT_QUERY_SCAN_DURATION_MS);

            // Build scan vector from BLE cache + WiFi AP scan
            ProxScanVector vec;
            prox_build_scan_vector(vec);

            // print amount of dimensions in vector
            Serial.printf("[PROX] Scan vector has %d dimensions\n", vec.count);

            // Query the anchor via GATT
            ProxScoreResult result;
            bool ok = prox_query_anchor(rec->bleMac, rec->bleAddrType, vec, result);

            if (!ok) {
                // Connection establishment fails at much weaker RSSI (~-88 dBm)
                // than advertisement reception, so a failed connect together
                // with a weak recent advertisement RSSI is strong evidence the
                // watch is FAR from the anchor. Use that rather than a blind
                // AMBIGUOUS fail-safe (which would wrongly treat stayNear as met
                // when the user has actually walked out of connection range).
                uint32_t now_ts = (uint32_t)time(nullptr);
                bool rssi_fresh = rec->lastSeen != 0 &&
                                  (now_ts - rec->lastSeen) <= (uint32_t)ANCHOR_SEEN_TIMEOUT_S;
                if (rssi_fresh && rec->lastRSSI <= PROX_FAR_RSSI_THRESHOLD_DBM) {
                    Serial.printf("[PROX] Connect failed; recent ad RSSI %d <= %d → treating as AWAY (far)\n",
                                  rec->lastRSSI, PROX_FAR_RSSI_THRESHOLD_DBM);
                    // Far: stayNear → not near → enforce; getAway → away → met.
                    return (e->criteria == GET_AWAY);
                }
                Serial.println("[PROX] Anchor query failed — AMBIGUOUS fail-safe");
                return (e->criteria == STAY_NEAR);
            }

            rec->lastProxScore = result.score;
            Serial.printf("[PROX] Score=%d flags=0x%02X\n", result.score, result.flags);

            ProxProximity prox = prox_interpret_score(result.score);
            if (prox == PROX_AMBIGUOUS) {
                // Fail-safe: stayNear→NEAR (condition met), getAway→AWAY (condition met)
                prox = (e->criteria == STAY_NEAR) ? PROX_NEAR : PROX_AWAY;
            }

            if (e->criteria == STAY_NEAR) return (prox == PROX_NEAR);
            else                          return (prox == PROX_AWAY);
        }
        case GET_ON_WIFI:
            return g_wifi_connected &&
                   strncmp(g_current_ssid, e->wifiSSID, 63) == 0;
        case GET_OFF_WIFI:
            return !g_wifi_connected ||
                   strncmp(g_current_ssid, e->wifiSSID, 63) != 0;
    }
    return false;
}

// ============================================================
//  Enforcement profile playback
// ============================================================

static void enforcement_start(EnforcementProfile prof) {
 
    const ProfileDefinition *def = find_profile(prof);
    g_enf.step_idx        = 0;
    g_enf.cycle_count     = 0;
    g_enf.step_start_ms   = millis();
    g_enf.condition_met   = false;
    g_enf.active          = true;
    // Initial wait duration = second step's duration (step_idx 1)
    g_enf.current_wait_ms = (def->step_count > 1) ? def->steps[1].duration_ms : 0;
    // Apply first step outputs
    const ProfileStep &s = def->steps[0];
    set_motor(s.motor_on);
    set_buzzer(s.buzzer_on);
}

static void enforcement_stop() {
    g_enf.active = false;
    outputs_off();
}

static void enforcement_update() {
    if (!g_enf.active) return;
    const ProfileDefinition *def = find_profile(g_active_event->profile);
    const ProfileStep &step = def->steps[g_enf.step_idx];

    // Continuous step: stays until condition is met (handled externally)
    if (step.duration_ms == DURATION_CONTINUOUS) return;

    uint32_t elapsed = millis() - g_enf.step_start_ms;

    // Determine step duration (wait step may have been decremented)
    uint32_t dur = step.duration_ms;
    if (g_enf.step_idx == 1 && def->loops && def->floor_interval_ms > 0) {
        dur = g_enf.current_wait_ms;
    }

    if (elapsed < dur) return;

    // Advance to next step
    g_enf.step_idx++;
    if (g_enf.step_idx >= def->step_count) {
        if (!def->loops) {
            enforcement_stop();
            return;
        }
        g_enf.step_idx = 0;
        g_enf.cycle_count++;
        // Decrement wait step for next cycle
        if (def->floor_interval_ms > 0 && def->step_count > 1) {
            if (g_enf.current_wait_ms > INTERVAL_DECREMENT_MS + def->floor_interval_ms)
                g_enf.current_wait_ms -= INTERVAL_DECREMENT_MS;
            else
                g_enf.current_wait_ms = def->floor_interval_ms;
        }
    }

    g_enf.step_start_ms = millis();
    const ProfileStep &next = def->steps[g_enf.step_idx];
    set_motor(next.motor_on);
    set_buzzer(next.buzzer_on);
}

// Adaptive enforcement poll cadence (§8.2): back off when the user is compliant
// and stable, poll quickly when not. The IMU motion interrupt still forces an
// immediate re-check, so this only suppresses redundant polling of a still,
// compliant user.
static uint32_t enforcement_poll_interval_ms() {
    uint32_t s = g_enf.condition_met ? ENFORCEMENT_POLL_INTERVAL_MET_S
                                     : ENFORCEMENT_POLL_INTERVAL_S;
    return s * 1000UL;
}

static void check_enforcement_condition() {
    if (!g_active_event) return;
    bool met = is_enforcement_condition_met(g_active_event);
    if (met && !g_enf.condition_met) {
        // Condition became met → stop outputs
        g_enf.condition_met = true;
        enforcement_stop();
    } else if (!met && g_enf.condition_met) {
        // Condition became not-met → start enforcement
        g_enf.condition_met = false;
        Serial.println("[ENF] Condition not met — starting enforcement");
        enforcement_start(g_active_event->profile);
    } else if (!met && !g_enf.condition_met) {
        // Still not met: advance profile playback
        enforcement_update();
    }
}

// ============================================================
//  State transitions
// ============================================================

static bool should_enforce() {
    if (!g_bt_connected && !g_wifi_connected && !g_disconnected_is_dormant) return false;
    if (!g_bt_connected && g_wifi_connected  && !g_away_is_dormant) return false;
    return true;
}

static void enter_enforcement(Event *e) {
    Serial.printf("[STATE] ENFORCEMENT event %02x...\n", e->id[0]);
    g_activity_state = STATE_ENFORCEMENT;
    batt_log("enforcement_enter");
    g_active_event   = e;

    // Reconfigure BLE scanner to low duty cycle to reduce radio-on time.
    // Stop any running scan first — NimBLE requires the scanner to be idle
    // before setInterval/setWindow take effect.
    if (g_ble_scan->isScanning()) g_ble_scan->stop();
    g_ble_scan->setInterval(ENFORCEMENT_SCAN_INTERVAL_MS);
    g_ble_scan->setWindow(ENFORCEMENT_SCAN_WINDOW_MS);

    if (g_wifi_connected) {
        // Keep WiFi connected for all enforcement types.
        // Anchor-based events need WiFi AP RSSI in the scan vector (§5.4.1).
        // Use modem sleep between operations to reduce idle current.
        esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    }

    g_enf.condition_met = is_enforcement_condition_met(e);
    if (!g_enf.condition_met) enforcement_start(e->profile);
    push_watch_status();
}

static void exit_enforcement() {
    Serial.println("[STATE] DORMANT");
    g_activity_state = STATE_DORMANT;
    batt_log("enforcement_exit");
    g_active_event   = nullptr;
    enforcement_stop();

    // Restore DORMANT BLE scan duty cycle (90%).
    if (g_ble_scan->isScanning()) g_ble_scan->stop();
    g_ble_scan->setInterval(DORMANT_SCAN_INTERVAL_MS);
    g_ble_scan->setWindow(DORMANT_SCAN_WINDOW_MS);

    if (g_wifi_connected) {
        esp_wifi_set_ps(WIFI_PS_NONE);
    }

    arm_next_boundary();
    g_last_activity_ms = millis();
    push_watch_status();
}

static void recalculate_and_rearm() {
  
    recalculate_day();
    arm_next_boundary();

    if (!should_enforce()) {
        if (g_activity_state == STATE_ENFORCEMENT) exit_enforcement();
        return;
    }

    uint32_t now_min = local_minutes_now();
    // Check if currently inside any event
    for (int i = 0; i < g_today_event_count; i++) {
   
        Event &e = g_today_events[i];
        if (now_min >= e.startTime && now_min < e.endTime) {
            
            if (g_activity_state != STATE_ENFORCEMENT || g_active_event != &e) {

               
                enter_enforcement(&e);
            }
            return;
        }
    }
    // Not in any event window
    if (g_activity_state == STATE_ENFORCEMENT) exit_enforcement();
}

// ============================================================
//  Light sleep
// ============================================================

static void enter_dormant_sleep() {
    Serial.println("[STATE] DORMANT_SLEEP");
    g_activity_state = STATE_DORMANT_SLEEP;
    push_watch_status();

    uint32_t now_min  = local_minutes_now();
    uint32_t next_min = next_boundary_after(now_min);
    uint64_t sleep_us;
    if (next_min == 0xFFFF) {
        // Sleep until midnight
        uint32_t mins_left = 1440 - now_min;
        sleep_us = (uint64_t)mins_left * 60ULL * 1000000ULL;
    } else {
        int32_t secs = seconds_until_minute(next_min);
        sleep_us = (uint64_t)(secs > 0 ? secs : 1) * 1000000ULL;
    }

    // Heartbeat: while disconnected, never sleep past the advertise heartbeat so
    // a still, idle watch periodically surfaces to advertise and accept a phone
    // connection (§8.4). The caller only enters dormant sleep when disconnected.
    uint64_t heartbeat_us = (uint64_t)DISCONNECTED_ADV_HEARTBEAT_MS * 1000ULL;
    if (sleep_us > heartbeat_us) sleep_us = heartbeat_us;

    esp_sleep_enable_timer_wakeup(sleep_us);

    // Motion is NOT a wake source in DORMANT (§8.4): outside an enforcement window
    // there is nothing for motion to trigger, and waking on every wrist movement
    // was the single largest battery drain. Wake on the timer only. Detach the
    // edge ISR and disable the INT1 GPIO wake (an earlier enforcement sleep may
    // have enabled it) so wrist motion stays muted until the next heartbeat.
    detachInterrupt(digitalPinToInterrupt(LIS3DH_INT1));
    lis3dh_clear_int1();
    data_ready        = false;
    g_last_motion_ms  = 0;
    gpio_wakeup_disable((gpio_num_t)LIS3DH_INT1);

#if ADVERTISE_DURING_SLEEP
    // Keep BLE radio alive (modem sleep) so the phone can connect and wake
    // the CPU. Use a long advertising interval to minimise current draw.
    {
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        // NimBLE interval units are 0.625 ms
        pAdv->setMinInterval(SLEEP_ADV_MIN_INTERVAL_MS * 8 / 5);
        pAdv->setMaxInterval(SLEEP_ADV_MAX_INTERVAL_MS * 8 / 5);
        pAdv->start();
    }
    esp_ble_sleep_enable(); // not a real function
#endif

    batt_log("pre_sleep");

    esp_light_sleep_start();

    // After wakeup: clear the interrupt that woke us, then re-arm the ISR.
    lis3dh_clear_int1();
    data_ready       = false;
    g_last_motion_ms = 0;
    attachInterrupt(digitalPinToInterrupt(LIS3DH_INT1), lis3dh_isr, RISING);

#if ADVERTISE_DURING_SLEEP
    // Restore normal advertising interval now that we're awake.
    {
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        pAdv->setMinInterval(0);  // revert to NimBLE defaults
        pAdv->setMaxInterval(0);
    }
#endif

    Serial.println("[STATE] Woke from light sleep");
    batt_log("wake_from_sleep");
    g_activity_state   = STATE_DORMANT;
    g_last_activity_ms = millis();
    NimBLEDevice::getAdvertising()->start();
    recalculate_and_rearm();
}

// ============================================================
//  Enforcement light sleep
// ============================================================

static void enter_enforcement_sleep() {
    // Preconditions: condition_met == true, user still, no scan in progress.
    // Stays in STATE_ENFORCEMENT; wakes to re-check the condition.

    // Stop any active BLE scan before sleeping.
    if (g_ble_scan && g_ble_scan->isScanning()) g_ble_scan->stop();

    // Sleep until the next poll (adaptive cadence — this path runs only while
    // compliant, so it uses the backed-off MET interval), capped at time until
    // event end so the event never terminates late.
    uint64_t sleep_us = (uint64_t)enforcement_poll_interval_ms() * 1000ULL;
    if (g_active_event) {
        uint32_t now_min = local_minutes_now();
        if (g_active_event->endTime > now_min) {
            uint64_t us_to_end = (uint64_t)(g_active_event->endTime - now_min) * 60ULL * 1000000ULL;
            if (us_to_end < sleep_us) sleep_us = us_to_end;
        }
    }
    if (sleep_us < 1000000ULL) sleep_us = 1000000ULL; // minimum 1 s

    esp_sleep_enable_timer_wakeup(sleep_us);

    detachInterrupt(digitalPinToInterrupt(LIS3DH_INT1));
    lis3dh_clear_int1();
    data_ready       = false;
    g_last_motion_ms = 0;   // cancel any pending deferred re-arm

    esp_sleep_enable_gpio_wakeup();
    gpio_wakeup_enable((gpio_num_t)LIS3DH_INT1, GPIO_INTR_HIGH_LEVEL);

#if ADVERTISE_DURING_SLEEP
    {
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        pAdv->setMinInterval(SLEEP_ADV_MIN_INTERVAL_MS * 8 / 5);
        pAdv->setMaxInterval(SLEEP_ADV_MAX_INTERVAL_MS * 8 / 5);
        pAdv->start();
    }
    esp_ble_sleep_enable();
#endif

    batt_log("enf_pre_sleep");
    esp_light_sleep_start();

    // Wakeup — re-arm ISR.
    lis3dh_clear_int1();
    data_ready       = false;
    g_last_motion_ms = 0;
    attachInterrupt(digitalPinToInterrupt(LIS3DH_INT1), lis3dh_isr, RISING);

#if ADVERTISE_DURING_SLEEP
    {
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        pAdv->setMinInterval(0);
        pAdv->setMaxInterval(0);
    }
#endif

    Serial.println("[STATE] Woke from enforcement sleep");
    batt_log("enf_wake_from_sleep");
    g_last_activity_ms = millis();
    NimBLEDevice::getAdvertising()->start();

    // Force an immediate condition re-check on the next loop iteration.
    g_last_enf_poll_ms = 0;

    // For anchor-based events, run a bounded BLE scan to refresh the device
    // cache, then the upcoming condition check will build a fresh ProxScanVector.
    if (g_active_event &&
        (g_active_event->criteria == STAY_NEAR || g_active_event->criteria == GET_AWAY)) {
        start_ble_scan(ENFORCEMENT_SCAN_DURATION_MS);
    }
}

// ============================================================
//  Worn detection
// ============================================================

static void on_worn_state_changed(bool is_worn) {
    Serial.printf("[WORN] %s\n", is_worn ? "worn" : "removed");
    batt_log(is_worn ? "worn" : "removed");
    if (g_activity_state == STATE_ENFORCEMENT && g_active_event
        && g_active_event->beepAnchorCount > 0) {
        if (is_worn)  send_watch_worn_to_anchors(g_active_event);
        else          send_watch_removed_to_anchors(g_active_event);
    }
    push_watch_status();
}

// Pulse the ITR8307 emitter and return the ambient-subtracted reflection level.
// Reads the phototransistor with the IR LED off (ambient baseline) and on
// (lit), returning (lit - ambient). With a non-inverting readout, skin contact
// reflects more IR and raises this delta; an inverting readout lowers it.
static int ir_sample_reflection() {
    // Ambient baseline with emitter off
    digitalWrite(IR_EMIT, LOW);
    delayMicroseconds(IR_EMIT_SETTLE_US);
    int ambient = analogRead(IR_REC);

    // Lit reading with emitter on
    digitalWrite(IR_EMIT, HIGH);
    delayMicroseconds(IR_EMIT_SETTLE_US);
    int lit = analogRead(IR_REC);

    digitalWrite(IR_EMIT, LOW);
    return lit - ambient;
}

// Decide whether a reflection delta indicates the watch is being worn.
// IR_WORN_HIGHER_MEANS_WORN selects the circuit polarity.
static bool ir_reflection_indicates_worn(int reflection) {
    return IR_WORN_HIGHER_MEANS_WORN ? (reflection >=  IR_WORN_THRESHOLD)
                                     : (reflection <= -IR_WORN_THRESHOLD);
}

static void update_worn_state() {
    if (millis() - g_last_worn_sample_ms < IR_WORN_SAMPLE_INTERVAL_MS) return;
    g_last_worn_sample_ms = millis();

    int     reflection = ir_sample_reflection();
    uint8_t reading    = ir_reflection_indicates_worn(reflection) ? 1 : 0;
    g_worn_buffer[g_worn_buf_idx] = reading;
    g_worn_buf_idx = (g_worn_buf_idx + 1) % IR_WORN_DEBOUNCE_SAMPLES;
    if (g_worn_buf_fill < IR_WORN_DEBOUNCE_SAMPLES) g_worn_buf_fill++;

    if (g_worn_buf_fill < IR_WORN_DEBOUNCE_SAMPLES) return;

    // Check if all readings are the same
    bool all_same = true;
    uint8_t val   = g_worn_buffer[0];
    for (int i = 1; i < IR_WORN_DEBOUNCE_SAMPLES; i++) {
        if (g_worn_buffer[i] != val) { all_same = false; break; }
    }
    if (!all_same) return;

    bool new_worn = (val == 1);
    if (new_worn != g_worn) {
        g_worn = new_worn;
        on_worn_state_changed(new_worn);
    }
}

// ============================================================
//  BLE GATT callbacks (Watch ↔ Phone)
// ============================================================

class WatchServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
        g_bt_connected     = true;
        g_last_activity_ms = millis();
        batt_log("ble_connect");

        if (g_activity_state == STATE_UNPAIRED) {
            g_is_paired = true;
            prefs.begin("watch", false);
            prefs.putBool("paired", true);
            prefs.end();
            set_buzzer(true);
            delay(UNPAIRED_BEEP_DURATION_MS);
            set_buzzer(false);
            set_motor(true);
            delay(UNPAIRED_VIBRATE_DURATION_MS);
            set_motor(false);
            g_activity_state = STATE_DORMANT;
        }
        recalculate_and_rearm();
        push_watch_status();
        NimBLEDevice::getAdvertising()->start();
    }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        g_bt_connected     = false;
        batt_log("ble_disconnect");
        g_last_activity_ms = millis();
        recalculate_and_rearm();
        push_watch_status();
        NimBLEDevice::getAdvertising()->start();
    }
};

class WatchWifiCredCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        JsonDocument doc;
        if (deserializeJson(doc, val.c_str())) return;
        const char *ssid = doc["ssid"] | "";
        const char *pass = doc["password"] | "";
        if (!ssid || strlen(ssid) == 0) return;

        // Save to creds list (replace if SSID already exists)
        bool found = false;
        for (int i = 0; i < g_wifi_cred_count; i++) {
            if (strncmp(g_wifi_creds[i].ssid, ssid, 63) == 0) {
                strlcpy(g_wifi_creds[i].pass, pass, 63);
                found = true; break;
            }
        }
        if (!found && g_wifi_cred_count < MAX_WIFI_CREDS) {
            strlcpy(g_wifi_creds[g_wifi_cred_count].ssid, ssid, 63);
            strlcpy(g_wifi_creds[g_wifi_cred_count].pass, pass, 63);
            g_wifi_cred_count++;
        }
        save_wifi_creds();

        // Attempt connection immediately
        WiFi.begin(ssid, pass);
        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < 8000) delay(50);
        if (WiFi.status() == WL_CONNECTED) {
            g_wifi_connected = true;
            strlcpy(g_current_ssid, ssid, sizeof(g_current_ssid));
            g_udp.begin(0);
            configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");
            recalculate_and_rearm();
        }
        push_watch_status();
        g_last_activity_ms = millis();
    }
};

class WatchSchedCtrlCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.empty()) return;
        uint8_t cmd  = (uint8_t)val[0];
        uint8_t resp = 0x00;

        if (cmd == 0x01 && val.size() >= 5) {          // BEGIN
            uint32_t total;
            memcpy(&total, val.data() + 1, 4);
            if (g_sched_xfer_buf) { free(g_sched_xfer_buf); g_sched_xfer_buf = nullptr; }
            g_sched_xfer_buf = (uint8_t*)malloc(total);
            if (g_sched_xfer_buf) {
                g_sched_xfer_exp  = total;
                g_sched_xfer_rcvd = 0;
                g_sched_xfer_act  = true;
                resp = 0x01;
            }
        } else if (cmd == 0x02 && val.size() >= 5) {   // END
            uint32_t crc_recv;
            memcpy(&crc_recv, val.data() + 1, 4);
            if (g_sched_xfer_act && g_sched_xfer_buf
                && g_sched_xfer_rcvd == g_sched_xfer_exp) {
                uint32_t crc_calc = esp_crc32_le(0, g_sched_xfer_buf, g_sched_xfer_rcvd);
                if (crc_calc == crc_recv) {
                    if (g_sched_blob) free(g_sched_blob);
                    g_sched_blob     = g_sched_xfer_buf;
                    g_sched_blob_len = g_sched_xfer_rcvd;
                    g_sched_xfer_buf = nullptr;
                    save_schedule_blob(g_sched_blob, g_sched_blob_len);
                    recalculate_and_rearm();
                    resp = 0x01;
                }
            }
            if (g_sched_xfer_buf) { free(g_sched_xfer_buf); g_sched_xfer_buf = nullptr; }
            g_sched_xfer_act = false;
        } else if (cmd == 0x03) {                       // ABORT
            if (g_sched_xfer_buf) { free(g_sched_xfer_buf); g_sched_xfer_buf = nullptr; }
            g_sched_xfer_act = false;
            resp = 0x01;
        }
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

class WatchSchedDataCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
       
        if (!g_sched_xfer_act || !g_sched_xfer_buf) return;
        std::string val = pChar->getValue();
        uint32_t remaining = g_sched_xfer_exp - g_sched_xfer_rcvd;
        uint32_t to_copy   = (uint32_t)val.size() < remaining
                               ? (uint32_t)val.size() : remaining;
        memcpy(g_sched_xfer_buf + g_sched_xfer_rcvd, val.data(), to_copy);
        g_sched_xfer_rcvd += to_copy;
        g_last_activity_ms  = millis();
    }
};

class WatchSettingsCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.size() < 4) return;
        bool    new_disconn = val[0] != 0;
        bool    new_away    = val[1] != 0;
        int16_t new_tz;
        memcpy(&new_tz, val.data() + 2, 2);

        bool tz_changed = (new_tz != g_tz_offset_min);
        g_disconnected_is_dormant = new_disconn;
        g_away_is_dormant         = new_away;
        g_tz_offset_min           = new_tz;

        prefs.begin("watch", false);
        prefs.putBool("disc_dorm", g_disconnected_is_dormant);
        prefs.putBool("away_dorm", g_away_is_dormant);
        prefs.putShort("tz_off",   g_tz_offset_min);
        prefs.end();

        if (tz_changed) {
            configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");
            recalculate_and_rearm();
        }

        uint8_t resp = 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

void indicate_successful_boot() {
    beep(3);
}
class WatchAnchorIpCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.size() < 1) return;
        uint8_t n = (uint8_t)val[0];
        size_t pos = 1;
        for (int i = 0; i < n; i++) {
            if (pos + 24 > val.size()) break;  // 16 UUID + 4 IP + 4 timestamp
            const uint8_t *uuid = (const uint8_t*)val.data() + pos; pos += 16;
            uint32_t ip_nbo;
            memcpy(&ip_nbo, val.data() + pos, 4); pos += 4;
            uint32_t ts;
            memcpy(&ts, val.data() + pos, 4); pos += 4;

            Serial.println("[ENF] Processing anchor IP update");
            // Find or create anchor record
            int slot = -1;
            for (int j = 0; j < MAX_ANCHOR_RECORDS; j++) {
                if (g_anchor_records[j].valid && uuid_eq(g_anchor_records[j].uuid, uuid)) {
                    slot = j; break;
                }
            }
            if (slot < 0) {
                for (int j = 0; j < MAX_ANCHOR_RECORDS; j++) {
                    if (!g_anchor_records[j].valid) { slot = j; break; }
                }
            }
            if (slot >= 0) {
                Serial.printf("[ENF] Anchor record slot %d updated: uuid=%s ip=%u.%u.%u.%u ts=%lu\n",
                              slot, convertAnchoridToString(uuid).c_str(),
                              (ip_nbo >> 24) & 0xFF, (ip_nbo >> 16) & 0xFF,
                              (ip_nbo >> 8) & 0xFF, ip_nbo & 0xFF,
                              (unsigned long)ts);
                g_anchor_records[slot].valid         = true;
                memcpy(g_anchor_records[slot].uuid, uuid, 16);
                g_anchor_records[slot].ipAddress     = ip_nbo;
                g_anchor_records[slot].ipLastUpdated = ts;
            }
        }
        uint8_t resp = 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

// ============================================================
//  setup()
// ============================================================

// ============================================================
//  Battery event log functions
// ============================================================

static void batt_log_load() {
    g_batt_log_head = 0;
    g_batt_log_count = 0;
    memset(g_batt_log, 0, sizeof(g_batt_log));
    Log.notice("BATT NVS persistence disabled (RAM-only log)" CR);
}

static void batt_log_clear() {
    g_batt_log_head  = 0;
    g_batt_log_count = 0;
}

static void batt_log(const char *event) {
    float    voltage = (float)get_battery_voltage();
    uint32_t ts      = millis();

    uint8_t write_idx;
    if (g_batt_log_count < BATT_LOG_MAX_ENTRIES) {
        write_idx = (g_batt_log_head + g_batt_log_count) % BATT_LOG_MAX_ENTRIES;
        g_batt_log_count++;
    } else {
        // Ring is full — overwrite oldest slot and advance head
        write_idx       = g_batt_log_head;
        g_batt_log_head = (g_batt_log_head + 1) % BATT_LOG_MAX_ENTRIES;
    }

    BattLogEntry &e = g_batt_log[write_idx];
    e.millis_ts = ts;
    e.voltage   = voltage;
    strncpy(e.event, event, sizeof(e.event) - 1);
    e.event[sizeof(e.event) - 1] = '\0';

    Log.notice("BATT [%l ms] %DV %s" CR, (long)ts, (double)voltage, event);
}

// Send the full log over serial in response to the dump trigger (0xAB).
// Format: BATTLOG_START / millis_ms,voltage_V,event / BATTLOG_END
static void batt_log_dump_serial() {
    Serial.println("BATTLOG_START");
    for (int i = 0; i < (int)g_batt_log_count; i++) {
        uint8_t idx = (g_batt_log_head + (uint8_t)i) % BATT_LOG_MAX_ENTRIES;
        const BattLogEntry &e = g_batt_log[idx];
        Serial.printf("%u,%.3f,%s\n", e.millis_ts, e.voltage, e.event);
    }
    Serial.println("BATTLOG_END");
}

// --- Crash checkpoint helpers ---
// Writes step number to NVS so next boot can report where we died.
static void chk_set(uint8_t step) {
    Preferences p;
    p.begin("dbg", false);
    p.putUChar("chk", step);
    p.end();
}
static void print_debug(const char *msg) { // prints message only if debug mode is on
    if (!DEBUG_MODE) return;
    Serial.println(msg);
}
static void chk_report_and_clear() {
    Preferences p;
    p.begin("dbg", false);
    uint8_t last = p.getUChar("chk", 0);
    p.putUChar("chk", 0);
    p.end();
    if (last == 0) return;  // clean boot, nothing to report
    // Beep the step number: e.g. step 7 = pause, then 7 short beeps
    delay(1000);
    for (uint8_t i = 0; i < last; i++) {
        digitalWrite(BUZZER_PIN, HIGH); delay(200);
        digitalWrite(BUZZER_PIN, LOW);  delay(750);
    }
    delay(800);
}

void beep_delay() {
    return;
}

#define bp beep_delay()
void setup() {

    // disable brownouts  
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    

    Serial.begin(115200);
    uint32_t serialWait = millis();
    while (!Serial && millis() - serialWait < 2000) delay(10);
    delay(100);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);

    delay(1000);

    bp;

    batt_log_clear();

    prox_init_watch();

    // Hardware pins first so buzzer works for crash reporting
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRO_PIN,  OUTPUT);
    // ITR8307 reflective IR worn sensor
    pinMode(IR_EMIT, OUTPUT);
    digitalWrite(IR_EMIT, LOW);                       // emitter off until sampled
    pinMode(IR_REC, INPUT);
    analogSetPinAttenuation(IR_REC, ADC_11db);        // full-range phototransistor readings
    pinMode(BATT_ADC_PIN, INPUT);
    analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);  // effective range 0–2.45V; Vbat/2 max ~2.1V
    outputs_off();


    bp;

    // chk_report_and_clear();  // beep last crash step if any

    // CHK 1: SPI / IMU init
    chk_set(1);
    delay(100);
    pinMode(LIS3DH_CS, OUTPUT);
    digitalWrite(LIS3DH_CS, HIGH);
    SPI.begin(LIS3DH_SCK, LIS3DH_MISO, LIS3DH_MOSI, LIS3DH_CS);
      if (!lis3dh_init()) {
        Serial.println("[SYS] Init failed. Check wiring. Halting.");
   
    } else {
        Serial.println("[SYS] IMU initialized");
    }

    bp;


    // CHK 2: NVS load
    chk_set(2);
    delay(100);
    prefs.begin("watch", true);
    g_is_paired       = prefs.getBool("paired", false);
    g_disconnected_is_dormant = prefs.getBool("disc_dorm", true);
    g_away_is_dormant         = prefs.getBool("away_dorm", true);
    g_tz_offset_min           = prefs.getShort("tz_off", 0);
    bool has_uuid     = prefs.isKey("uuid");
    if (has_uuid) prefs.getBytes("uuid", g_watch_uuid, 16);
    prefs.end();

    if (!has_uuid) {
        generate_uuid_v4(g_watch_uuid);
        prefs.begin("watch", false);
        prefs.putBytes("uuid", g_watch_uuid, 16);
        prefs.end();
    }
    uuid_to_str(g_watch_uuid, g_watch_uuid_str);
    Serial.printf("[SYS] Watch UUID: %s\n", g_watch_uuid_str);


    bp;


    // CHK 3: load WiFi creds + schedule blob
    chk_set(3);
    bp;

    load_wifi_creds();
    bp;

    load_schedule_blob();
    bp;

    batt_log_load();
    bp;

    g_activity_state = g_is_paired ? STATE_DORMANT : STATE_UNPAIRED;

    
    bp;

    // CHK 4: WiFi mode
    chk_set(4);
    bp;
    delay(200);
    bp;
    WiFi.mode(WIFI_STA);
    


    bp; 

    // CHK 4b: WiFi connect
    chk_set(14);
    if (g_wifi_cred_count > 0) try_connect_saved_wifi();


    bp;

    // CHK 5: NimBLE init
    chk_set(5);
    delay(500);
    NimBLEDevice::init(g_watch_uuid_str);
    // Boost BLE TX power to extend usable connection range. Connection
    // establishment fails at much weaker RSSI than advertisement reception, so
    // the default (~0 dBm) leaves the proximity query failing while ads are
    // still heard. (Boost the anchor's TX power the same way for full benefit.)
    NimBLEDevice::setPower(9);  // +9 dBm (max on ESP32-C3)

    bp;


    // CHK 6: GATT server + service
    chk_set(6);
    g_ble_server = NimBLEDevice::createServer();
    g_ble_server->setCallbacks(new WatchServerCallbacks());
    NimBLEService *svc = g_ble_server->createService(WATCH_SERVICE_UUID);


    bp; 
    
    // CHK 7: GATT characteristics
    chk_set(7);
    auto *wifiCredChar = svc->createCharacteristic(WATCH_WIFI_CRED_CHAR_UUID,
                                                    NIMBLE_PROPERTY::WRITE_NR);
    wifiCredChar->setCallbacks(new WatchWifiCredCallback());

    auto *schedCtrlChar = svc->createCharacteristic(WATCH_SCHED_CTRL_CHAR_UUID,
                                                     NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    schedCtrlChar->setCallbacks(new WatchSchedCtrlCallback());

    auto *schedDataChar = svc->createCharacteristic(WATCH_SCHED_DATA_CHAR_UUID,
                                                     NIMBLE_PROPERTY::WRITE_NR);
    schedDataChar->setCallbacks(new WatchSchedDataCallback());

    auto *settingsChar = svc->createCharacteristic(WATCH_SETTINGS_CHAR_UUID,
                                                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    settingsChar->setCallbacks(new WatchSettingsCallback());

    g_seen_anchors_char = svc->createCharacteristic(WATCH_SEEN_ANCHORS_CHAR_UUID,
                                                     NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    g_status_char = svc->createCharacteristic(WATCH_STATUS_CHAR_UUID,
                                               NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    auto *anchorIpChar = svc->createCharacteristic(WATCH_ANCHOR_IP_CHAR_UUID,
                                                    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    anchorIpChar->setCallbacks(new WatchAnchorIpCallback());

    bp; 
    // CHK 8: start service + advertising
    chk_set(8);
    svc->start();
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(WATCH_SERVICE_UUID);
    pAdv->start();
    Serial.println("[BLE] Watch advertising started");

    // CHK 9: BLE scanner setup
    chk_set(9);
    g_ble_scan = NimBLEDevice::getScan();
    g_ble_scan->setScanCallbacks(&g_scan_cb, true);
    g_ble_scan->setActiveScan(false);
    // Interval and window in ms. Window must be <= interval.
    // Equal values = continuous scan (100% duty cycle). A small gap (e.g.
    // 200/180) leaves the radio breathing room for the concurrent advertising
    // role without meaningfully reducing detection rate.
    g_ble_scan->setInterval(DORMANT_SCAN_INTERVAL_MS);
    g_ble_scan->setWindow(DORMANT_SCAN_WINDOW_MS);
    g_ble_scan->setMaxResults(0);

    // CHK 10: schedule calculation + boot-time recovery
    chk_set(10);
    if (g_is_paired) {
        recalculate_and_rearm();
        uint32_t now_min = local_minutes_now();
        for (int i = 0; i < g_today_event_count; i++) {
            if (now_min >= g_today_events[i].startTime &&
                now_min <  g_today_events[i].endTime   &&
                should_enforce()) {
                enter_enforcement(&g_today_events[i]);
                break;
            }
        }
    }

    g_last_activity_ms = millis();

    {
        time_t utc = time(nullptr);
        time_t local = utc + (time_t)g_tz_offset_min * 60;
        struct tm utc_tm, local_tm;
        gmtime_r(&utc, &utc_tm);
        gmtime_r(&local, &local_tm);
        Serial.printf("[TIME] tz_offset_min=%d\n", g_tz_offset_min);
        Serial.printf("[TIME] UTC:   %04d-%02d-%02d %02d:%02d:%02d\n",
            utc_tm.tm_year+1900, utc_tm.tm_mon+1, utc_tm.tm_mday,
            utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec);
        Serial.printf("[TIME] Local: %04d-%02d-%02d %02d:%02d:%02d\n",
            local_tm.tm_year+1900, local_tm.tm_mon+1, local_tm.tm_mday,
            local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec);
        Serial.printf("[TIME] Minutes since local midnight: %lu\n", (unsigned long)local_minutes_now());
    }

    Serial.println("[SYS] Boot complete");
    batt_log("boot_complete");
    // indicate_successful_boot();
    delay(1000);
    set_motor(true);
    delay(500);
    set_motor(false);
    while (!data_ready) delay(10);
    
}

// ============================================================
//  loop()
// ============================================================

// Midnight tracking
static int g_last_mday = -1;

void loop() {
    uint32_t now_ms  = millis();
    uint32_t now_min = local_minutes_now();

  

    // ---- IMU motion interrupt ----
    if (data_ready) {
        data_ready = false;
        if (g_activity_state == STATE_ENFORCEMENT) {
            // Motion is actionable only during enforcement (responsiveness).
            // Do NOT clear INT1 here — it stays latched HIGH, muting the ISR for
            // MINIMUM_BLE_DELAY_ENFORCEMENT; the deferred re-arm block clears it.
            g_last_motion_ms   = now_ms;
            g_last_activity_ms = now_ms;
            // check_enforcement_condition() runs its own aligned active scan
            // immediately before the query, so no separate scan is needed here.
            check_enforcement_condition();
        } else {
            // DORMANT / UNPAIRED: motion is not actionable and must not keep the
            // watch awake or reset the idle timer (§8.4). Clear the latch and
            // drop it so the next motion can re-fire harmlessly.
            lis3dh_clear_int1();
            g_last_motion_ms = 0;
        }
    }

    // ---- Deferred IMU interrupt re-arm ----
    if (g_last_motion_ms != 0) {
        uint32_t rearm_delay = (g_activity_state == STATE_ENFORCEMENT)
                               ? MINIMUM_BLE_DELAY_ENFORCEMENT
                               : MINIMUM_BLE_DELAY_DORMANT;
        if (now_ms - g_last_motion_ms >= rearm_delay) {
            lis3dh_clear_int1();
            g_last_motion_ms = 0;
        }
    }

    // ---- Midnight rollover ----
    {
        struct tm ti;
        if (getLocalTime(&ti) && ti.tm_mday != g_last_mday) {
            if (g_last_mday != -1) {
                Serial.println("[SCHED] Midnight — recalculating");
                recalculate_and_rearm();
            }
            g_last_mday = ti.tm_mday;
        }
    }

    if (g_activity_state == STATE_UNPAIRED) {
        delay(10);
        return;
    }

    // ---- Worn detection ----
    update_worn_state();

    // ---- WiFi watchdog ----
    if (g_wifi_connected && WiFi.status() != WL_CONNECTED) {
        g_wifi_connected = false;
        batt_log("wifi_disconnect");
        g_current_ssid[0] = 0;
        g_last_wifi_retry_ms = now_ms;
        recalculate_and_rearm();
        push_watch_status();
    }

    // ---- WiFi scan / reconnect (DORMANT) ----
    if (g_activity_state == STATE_DORMANT) {
        if (!g_wifi_connected &&
            (now_ms - g_last_wifi_retry_ms) > (uint32_t)ANCHOR_WIFI_RETRY_INTERVAL_S * 1000UL) {
            g_last_wifi_retry_ms = now_ms;
            try_connect_saved_wifi();
            if (g_wifi_connected) recalculate_and_rearm();
            g_last_activity_ms = now_ms;
        }
        // Periodic WiFi scan  (for SSID discovery — triggers reconnect)
        if ((now_ms - g_last_wifi_scan_ms) > (uint32_t)WIFI_SCAN_INTERVAL_S * 1000UL) {
            g_last_wifi_scan_ms = now_ms;
            if (!g_wifi_connected) try_connect_saved_wifi();
        }
    }

    // ---- BLE scan for anchors ----
    if (g_activity_state == STATE_DORMANT) {
        // DORMANT: scan on a fixed interval for anchor discovery and RSSI updates.
        if (!g_ble_scan->isScanning() &&
            ((now_ms - g_last_ble_scan_ms) > (uint32_t)BLE_SCAN_INTERVAL_S * 1000UL || g_last_ble_scan_ms == 0)) {
            start_ble_scan();
            g_last_activity_ms = now_ms;
        }
    }
    // ENFORCEMENT no longer runs a standalone periodic fallback scan (§8.2): each
    // poll and each motion-triggered check refreshes the cache with an aligned
    // active scan immediately before the query, and the watch light-sleeps
    // between polls when compliant — so a separate background scan was redundant.

    // ---- Event boundary check ----
    if (g_activity_state == STATE_DORMANT && g_next_boundary_min <= now_min) {
        // Check for an event starting now
        for (int i = 0; i < g_today_event_count; i++) {
            if (g_today_events[i].startTime == now_min && should_enforce()) {
                enter_enforcement(&g_today_events[i]);
                break;
            }
        }
        arm_next_boundary();
        g_last_activity_ms = now_ms;
    }

    // ---- Enforcement state ----
    if (g_activity_state == STATE_ENFORCEMENT) {
        // Check event end time
        if (g_active_event && now_min >= g_active_event->endTime) {
            exit_enforcement();
        } else {
            // Periodic condition poll (adaptive cadence — §8.2)
            if ((now_ms - g_last_enf_poll_ms) > enforcement_poll_interval_ms()) {
                g_last_enf_poll_ms = now_ms;

                check_enforcement_condition();
            } else {
                // Advance profile playback between polls
                if (!g_enf.condition_met) enforcement_update();
            }
        }
    }

    // ---- Enforcement idle sleep ----
    // Don't sleep while a phone is connected (§8.4) — light sleep drops the BLE
    // link, which would kill an in-progress config session.
    if (g_activity_state == STATE_ENFORCEMENT &&
        !g_bt_connected &&
        g_enf.condition_met &&
        !data_ready &&
        !(g_ble_scan && g_ble_scan->isScanning()) &&
        (now_ms - g_last_activity_ms) > ENFORCEMENT_IDLE_BEFORE_SLEEP_MS) {
        enter_enforcement_sleep();
        // Returns here after wakeup, still in STATE_ENFORCEMENT.
    }

    // ---- Transition to DORMANT_SLEEP ----
    // While connected, stay in DORMANT (modem sleep, link alive) so the app can
    // configure the watch; resume sleeping once the phone disconnects (§8.4).
    if (g_activity_state == STATE_DORMANT &&
        !g_bt_connected &&
        (now_ms - g_last_activity_ms) > DORMANT_TO_SLEEP_IDLE_MS) {
        // Motion no longer blocks DORMANT sleep — it is not a wake source here
        // and is ignored above (§8.4), so the idle timer alone governs sleep.
        enter_dormant_sleep();
        // Returns here after wakeup (already transitioned back to DORMANT)
    }

    // ---- Serial command handler ----
    // 0xAB  → dump full battery log as CSV over serial
    // 0xAC  → clear the battery log
    while (Serial.available() > 0) {
        uint8_t cmd = (uint8_t)Serial.read();
        if (cmd == 0xAB) {
            batt_log_dump_serial();
        } else if (cmd == 0xAC) {
            batt_log_clear();
            Serial.println("BATTLOG_CLEARED");
        }
    }

    // ---- Battery heartbeat (periodic voltage sample) ----
    if (now_ms - g_last_batt_heartbeat_ms >= BATT_HEARTBEAT_INTERVAL_MS) {
        g_last_batt_heartbeat_ms = now_ms;
        batt_log("heartbeat");
    }


    delay(10);
}
