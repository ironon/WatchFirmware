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
#include <esp_timer.h>
#include <esp_wifi.h>
#include <time.h>
#include <sys/time.h>   // settimeofday() for the Time characteristic (§5.6)
#include <string.h>
#include "imu.h"
#include "led_status.h"           // LED status ring (§5.7)
#include "proximity.h"            // shared proximity engine (../proximity_engine)
#include "watch_prox_transport.h" // re-homed transport/interpret + WiFi feeding
#include <ArduinoLog.h>
#include <esp_system.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  Pin definitions
// ============================================================
#define BUZZER_PIN    21
#define VIBRO_PIN     10

// On the current hardware rev the vibration motor and the LED status ring share
// GPIO 10 (the motor is disabled by a PCB switch). Set DISABLE_MOTOR to 1 to
// compile the motor driver out entirely so it never drives GPIO 10 — FastLED
// then owns the pin and the enforcement alarm blink renders cleanly. Set to 0
// on the next rev (ring → GPIO 7, motor restored on GPIO 10). When 1, all motor
// output (incl. enforcement vibration) is suppressed; buzzer behaviour is
// unchanged.
#define DISABLE_MOTOR 1

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
//  Battery usage measurement
//  Set to 1 to accumulate sleep/awake timing and wakeup metrics
//  for the overnight battery sanity check. See the "Battery usage
//  instrumentation" section further down for what is measured and
//  how to read it (serial 0xAD dump / 0xAE reset). Zero runtime cost
//  when 0 — all instrumentation compiles out.
// ============================================================
#define MEASURE_USAGE 1

// ============================================================
//  Awake-burst diagnostics
//  Set to 1 to log, over serial, how long the watch stays awake between
//  DORMANT sleeps and to flag any single loop iteration that blocks for too
//  long (e.g. a blocking call). Use this to chase battery/awake-time issues:
//    [DIAG] slow loop iter=NNNNms ...   → something blocked this iteration
//    [DIAG] DORMANT awake=NNNNms loops=N max_iter=NNms  → per-wake awake burst
//  A healthy idle watch should show awake bursts well under ~1 s.
// ============================================================
#define DIAG_AWAKE 1

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
// Mode B (phoneAway): grace period the user may be NEAR the docked phone before
// enforcement kicks in — "a quick check is fine." Uses wall-clock (time()) so it
// survives enforcement light sleep. (Tunable; could become a per-event field.)
#define PHONE_AWAY_TOLERANCE_S                     60
#define MINIMUM_BLE_DELAY_DORMANT               5000  // ms before re-arming IMU interrupt in DORMANT
#define MINIMUM_BLE_DELAY_ENFORCEMENT           3000  // ms before re-arming IMU interrupt in ENFORCEMENT
#define WIFI_SCAN_INTERVAL_S             120
// Blocking WiFi connect wait — used only on the boot path and user-initiated
// credential writes, never in the DORMANT heartbeat loop (that path is async).
#define WIFI_CONNECT_TIMEOUT_MS         5000
// Idle time in DORMANT before dropping back to light sleep. In a still, idle,
// disconnected watch the wake exists only to emit the advertise heartbeat (~a
// few adv events) and run any periodic scan that is due — there is no in-flight
// work to settle — so this dwell is almost pure waste. Trimmed from 1500 ms to
// keep the awake burst short (the dominant idle-drain term, see MEASURE_USAGE).
// A BLE scan in progress separately holds the watch awake until it finishes
// (see the DORMANT→SLEEP gate), so lowering this does not truncate scans.
// Tradeoff (tunable): a phone scanning at the wrong instant may wait one extra
// ~6 s heartbeat to connect.
#define DORMANT_TO_SLEEP_IDLE_MS         250
// While disconnected, the watch surfaces from DORMANT_SLEEP at least this often
// to advertise and accept a phone connection (§8.4). Motion is NOT a wake source
// in DORMANT, so this heartbeat is the only way to reach a still, idle watch.
// Lower = snappier app connect, higher = less battery. Tunable.
#define DISCONNECTED_ADV_HEARTBEAT_MS   6000
#define ANCHOR_WIFI_RETRY_INTERVAL_S      30
#define ANCHOR_SEEN_TIMEOUT_S             20
// Max time setup() waits for the IMU's first motion interrupt before continuing
// boot. A watch placed down perfectly still emits no motion, so this must never
// block unbounded or the CPU stays fully awake (watch unusable) until disturbed.
#define IMU_FIRST_INT_TIMEOUT_MS        2000

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
#define IR_WORN_THRESHOLD                500     // min ambient-subtracted ADC delta to count as worn
#define IR_WORN_HIGHER_MEANS_WORN       false     // false if the readout is inverted (skin lowers the delta)

// Enforcement escalation
#define INTERVAL_DECREMENT_MS           2000

// BLE MTU
#define BLE_MTU_BYTES                     23

// Schedule
#define MAX_EVENTS_PER_DAY                64
#define SCHEDULE_FORMAT_VERSION         0x02   // leading schedule-blob version byte (§6.2)
// Commitment integrity (§9.10)
#define SETTLE_WINDOW_MIN_DEFAULT        120   // minutes without edits before an event settles (§9.2)
#define SETTLE_WINDOW_FLOOR_MIN           30   // settle-window clamp (§9.8)
#define SETTLE_WINDOW_CEIL_MIN           240
#define LOOSEN_DELAY_H                    24   // hours a quarantined loosening waits (§9.3)
#define LOOSEN_FREE_HORIZON_H             24   // far-future loosenings apply immediately (§9.3)
#define PENDING_QUEUE_MAX                 16   // max quarantined entries (§9.3)
#define MAX_SETTLE_RECORDS                16   // per-event settle-state slots tracked in NVS (§9.2)
#define PASS_BUDGET_DEFAULT                2   // emergency passes per rolling window (§9.6)
#define PASS_WINDOW_DAYS                   7   // rolling pass-window length, elapsed basis (§9.6)
#define PASS_MAX_STAMPS                   16   // spend-stamp storage cap (>= any sane allowance)
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
#define WATCH_LED_CONFIG_CHAR_UUID   "4A0F0018-F8CE-11EE-8001-020304050607"
#define WATCH_TIME_CHAR_UUID         "4A0F0019-F8CE-11EE-8001-020304050607"
#define WATCH_PENDING_CHAR_UUID      "4A0F001A-F8CE-11EE-8001-020304050607"  // §9.5
#define WATCH_PASS_CHAR_UUID         "4A0F001B-F8CE-11EE-8001-020304050607"  // §9.6

// ============================================================
//  Enumerations
// ============================================================
enum RecurrenceType          : uint8_t { ONCE=0, DAILY=1, WEEKLY=2, MONTHLY=3 };
enum Criteria                : uint8_t { GET_AWAY=0, STAY_NEAR=1, GET_OFF_WIFI=2, GET_ON_WIFI=3,
                                          PHONE_AWAY=4 };  // Mode B: phone docked at anchorId; stay away from it
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
    uint16_t donningGraceS;   // §5.4.4: watch-side enforcement grace after donning (0 = none)
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
static bool     g_disconnected_is_dormant = true;
static bool     g_away_is_dormant         = true;
static int16_t  g_tz_offset_min           = 0;   // minutes east of UTC
static uint16_t g_settle_window_min       = SETTLE_WINDOW_MIN_DEFAULT;  // §9.2/§9.8 (stored; gating is Phase 3)

// Today's event list
static Event g_today_events[MAX_EVENTS_PER_DAY];
static int   g_today_event_count = 0;

// Shared scratch for parsing the full schedule blob (all recurrence types)
// before filtering to a single day. Used by recalculate_day() and the §9 diff
// gate; both run synchronously on the main task, never concurrently.
static Event g_all_events_scratch[MAX_EVENTS_PER_DAY * 2];

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
// phoneAway tolerance: wall-clock time() the watch first went NEAR the docked
// phone in the active event (0 = currently away/compliant). See PHONE_AWAY_TOLERANCE_S.
static uint32_t g_phone_near_since_ts   = 0;
// Donning grace (§5.4.4): wall-clock time() at which the active event's donning
// grace expires (0 = no grace active). Uses time() so it survives light sleep,
// like the phoneAway tolerance. While now < deadline the condition short-circuits
// to met. Cleared when the watch is removed or the event ends.
static time_t   g_grace_deadline        = 0;
// True while grace was active on the previous enforcement iteration, so the loop
// can force an immediate re-check the instant grace expires (§5.4.4).
static bool     g_grace_was_active      = false;
static uint32_t g_last_activity_ms      = 0;  // for DORMANT→SLEEP idle timer
#if DIAG_AWAKE
static uint32_t g_dbg_burst_start_ms = 0;   // millis the current awake burst began
static uint32_t g_dbg_last_loop_ms   = 0;   // millis at the top of the previous loop iteration
static uint32_t g_dbg_loop_count     = 0;   // loop iterations in the current awake burst
static uint32_t g_dbg_max_iter_ms    = 0;   // longest single iteration in the current burst
#endif
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

// ---- LED status helpers (§5.7) ----
static time_t local_now();  // fwd decl (defined below) — for the DORMANT clock

// Snapshot current watch state for the LED renderer. Decouples led_status.cpp
// from the state machine — it just gets the bits it needs, plus the current
// local time for the DORMANT analog clock (§5.7.4).
static LedStatusInput led_status_input() {
    LedStatusInput in;
    in.unpaired      = (g_activity_state == STATE_UNPAIRED);
    in.enforcing     = (g_activity_state == STATE_ENFORCEMENT);
    in.condition_met = g_enf.condition_met;
    time_t lt = local_now();
    struct tm ti;
    gmtime_r(&lt, &ti);
    in.hour   = ti.tm_hour;
    in.minute = ti.tm_min;
    return in;
}

// Map the ESP light-sleep wakeup cause to the LED wake-cause flash. In DORMANT
// motion is not a wake source (§8.4), so a dormant wake is normally TIMER; an
// enforcement wake may be TIMER or GPIO (motion). Anything else (e.g. a BLE
// controller wake) shows the BLE flash.
static LedWakeCause led_wake_cause_from_esp() {
    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: return LED_WAKE_TIMER;
        case ESP_SLEEP_WAKEUP_GPIO:  return LED_WAKE_MOTION;
        default:                     return LED_WAKE_BLE;
    }
}

static void set_motor(bool on)  {
#if DISABLE_MOTOR
    (void)on;  // motor compiled out — GPIO 10 is the LED ring this rev
#else
    digitalWrite(VIBRO_PIN, on ? HIGH : LOW);
#endif
}
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
    if (len < 3) return false;   // 1 version + 2 count
    uint32_t pos = 0;
    uint8_t ver = data[pos++];
    if (ver != SCHEDULE_FORMAT_VERSION) return false;   // unknown version → reject (§6.2)
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

        if (pos + 2 > len) return false;
        memcpy(&e.donningGraceS, data + pos, 2); pos += 2;   // §5.4.4

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
    if (!g_sched_blob || g_sched_blob_len < 3) return;

    // Parse all events
    Event *all_events = g_all_events_scratch;
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
//  Anchor record NVS persistence
// ============================================================
// Persists the known-anchor table (UUID → bleMac/bleAddrType/name/IP) so the
// watch can issue a directed proximity connect immediately on boot, instead of
// having to re-discover each anchor's MAC from a fresh advertisement first.
// Saved only on durable changes (MAC first captured, IP pushed) to limit flash
// wear, not on transient RSSI updates.

static void save_anchor_records() {
    prefs.begin("anchors", false);
    prefs.putBytes("recs", g_anchor_records, sizeof(g_anchor_records));
    prefs.end();
}

static void load_anchor_records() {
    prefs.begin("anchors", true);
    size_t n = prefs.getBytesLength("recs");
    if (n == sizeof(g_anchor_records)) {
        prefs.getBytes("recs", g_anchor_records, sizeof(g_anchor_records));
    }
    prefs.end();
    // Reset transient fields that aren't meaningful across a reboot, so freshness
    // checks (e.g. the connect-fail RSSI fallback) don't trust stale values.
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (!g_anchor_records[i].valid) continue;
        g_anchor_records[i].lastRSSI      = 0;
        g_anchor_records[i].lastSeen      = 0;
        g_anchor_records[i].lastProxScore = 0;
    }
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
                save_anchor_records();  // durable: persist newly-captured MAC
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
    save_anchor_records();  // persist new record (and any promoted MAC)
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
        prox_ingest_scan_result(mac_be, PROX_TYPE_BLE, rssi);

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
                    save_anchor_records();  // durable: persist newly-captured MAC
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

// Promote a freshly-associated WiFi link: record SSID, open the UDP socket, kick
// off NTP, flag connected. Shared by the blocking boot path and the async
// detector in loop() so both finish the connection identically.
static void on_wifi_associated(const char *ssid) {
    g_wifi_connected = true;
    strlcpy(g_current_ssid, ssid, sizeof(g_current_ssid));
    g_udp.begin(0); // any local port for sending
    configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");
    Serial.printf("[WiFi] Connected to %s\n", g_current_ssid);
    batt_log("wifi_connect");
}

// Blocking connect — boot / user-initiated only (never the DORMANT loop).
static void try_connect_saved_wifi() {
    if (g_wifi_cred_count == 0) return;
    // Attempt each saved credential
    for (int i = 0; i < g_wifi_cred_count; i++) {
        Serial.printf("[WiFi] Attempting to connect to saved SSID \"%s\"\n", g_wifi_creds[i].ssid);
        Serial.printf("[WiFi] Password: \"%s\"\n", g_wifi_creds[i].pass);
        WiFi.begin(g_wifi_creds[i].ssid, g_wifi_creds[i].pass);
        uint32_t t = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t < WIFI_CONNECT_TIMEOUT_MS) delay(50);
        if (WiFi.status() == WL_CONNECTED) {
            on_wifi_associated(g_wifi_creds[i].ssid);
            Serial.printf("[WiFi] Connected to %s\n", g_wifi_creds[i].ssid);

            return;
        }
        Serial.printf("[WiFi] Failed to connect to %s\n", g_wifi_creds[i].ssid);
    }
}

// Non-blocking reconnect for the DORMANT heartbeat path: kick off a single
// association attempt (round-robin across saved creds) and return immediately.
// Association completes asynchronously; the WiFi connect detector in loop()
// calls on_wifi_associated() once WL_CONNECTED is observed. This keeps a still,
// idle wake from busy-waiting up to WIFI_CONNECT_TIMEOUT_MS × creds.
static uint8_t g_wifi_try_idx = 0;
static void try_connect_saved_wifi_async() {
    if (g_wifi_cred_count == 0) return;
    if (WiFi.status() == WL_CONNECTED) return;
    uint8_t i = g_wifi_try_idx % g_wifi_cred_count;
    g_wifi_try_idx++;
    WiFi.begin(g_wifi_creds[i].ssid, g_wifi_creds[i].pass);
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
                    case PHONE_AWAY:  crit = "PHONE_AWAY"; break;
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
    
    // battery_pct (§5.6): 1 byte, 0–100 (0xFF if not available). The battery
    // sense is reserved/non-functional on this hardware rev, but we emit the
    // curve-derived percentage so the wire layout matches the spec (and the app).
    uint32_t batt_pct = voltage_to_percentage(batt_mv);
    buf[pos++] = (batt_pct > 100) ? 100 : (uint8_t)batt_pct;

    // active event UUID (zeros if none)
    if (g_active_event && !uuid_is_zero(g_active_event->id)) {
        memcpy(buf + pos, g_active_event->id, 16);
    } else {
        memset(buf + pos, 0, 16);
    }
    pos += 16;

    // condition_met (§5.6, added v0.6): 1 while the enforcement condition is met,
    // grace is active, or not enforcing; 0 only while actively alarming.
    uint8_t condition_met = (g_activity_state == STATE_ENFORCEMENT && !g_enf.condition_met) ? 0 : 1;
    buf[pos++] = condition_met;

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


// Perform an anchor-based proximity query and return NEAR / AWAY / AMBIGUOUS.
// AMBIGUOUS = "could not determine" (anchor not discovered, connect failed
// without a clear far-RSSI signal, or a mid-range score). Callers map AMBIGUOUS
// to their own fail-safe / fail-open direction. Shared by stayNear, getAway and
// phoneAway (Mode B), which all reduce to "how close is the watch to anchorId?".
// out_dock (optional): for phoneAway, receives the anchor's Dock Status
// (1 docked / 0 undocked / -1 unknown). Left -1 if the query never reaches the
// anchor (MAC unknown / connect fail) — callers treat unknown as docked.
static ProxProximity query_anchor_proximity(const Event *e, int8_t *out_dock = nullptr) {
    if (out_dock) *out_dock = -1;
    // Align a fresh BLE scan with this query FIRST. Besides refreshing the RF
    // cache, this is the only thing that (re)discovers the anchor's BLE MAC (the
    // scan callbacks populate the AnchorRecord as a side effect), so it must run
    // before the MAC check — there is no separate background scan in enforcement.
    prox_aligned_active_scan(ENFORCEMENT_QUERY_SCAN_DURATION_MS);

    AnchorRecord *rec = nullptr;
    for (int i = 0; i < MAX_ANCHOR_RECORDS; i++) {
        if (g_anchor_records[i].valid && uuid_eq(g_anchor_records[i].uuid, e->anchorId)) {
            rec = &g_anchor_records[i];
            break;
        }
    }
    if (!rec || !rec->bleMacValid) {
        Serial.println("[PROX] Anchor MAC unknown after scan — AMBIGUOUS");
        return PROX_AMBIGUOUS;
    }

    // BLE devices were fed into the engine's scan buffer by the aligned scan
    // above; feed WiFi APs, then build (build DRAINS the engine's buffer).
    prox_feed_wifi_aps();
    ProxScanVector vec;
    prox_build_scan_vector(&vec);
    Serial.printf("[PROX] Scan vector has %d dimensions\n", vec.count);

    ProxScoreResult result;
    if (!prox_query_anchor(rec->bleMac, rec->bleAddrType, vec, result, out_dock)) {
        // Connection establishment fails at much weaker RSSI (~-88 dBm) than
        // advertisement reception, so a failed connect + a weak recent ad RSSI
        // is strong evidence the watch is FAR from the anchor.
        uint32_t now_ts = (uint32_t)time(nullptr);
        bool rssi_fresh = rec->lastSeen != 0 &&
                          (now_ts - rec->lastSeen) <= (uint32_t)ANCHOR_SEEN_TIMEOUT_S;
        if (rssi_fresh && rec->lastRSSI <= PROX_FAR_RSSI_THRESHOLD_DBM) {
            Serial.printf("[PROX] Connect failed; recent ad RSSI %d <= %d → AWAY (far)\n",
                          rec->lastRSSI, PROX_FAR_RSSI_THRESHOLD_DBM);
            return PROX_AWAY;
        }
        Serial.println("[PROX] Anchor query failed — AMBIGUOUS");
        return PROX_AMBIGUOUS;
    }

    rec->lastProxScore = result.score;
    Serial.printf("[PROX] Score=%d flags=0x%02X\n", result.score, result.flags);
    return prox_interpret_score(result.score);
}

// Donning grace (§5.4.4): true while the active event's grace window is running.
static bool grace_active() {
    return g_grace_deadline != 0 && time(nullptr) < g_grace_deadline;
}

static bool is_enforcement_condition_met(const Event *e) {
    if (!e) return false;
    // Donning grace (§5.4.4): during the grace window the condition short-circuits
    // to met — no motor/buzzer output — regardless of the physical criteria.
    if (grace_active()) return true;
    switch (e->criteria) {
        case STAY_NEAR:
        case GET_AWAY: {
            ProxProximity prox = query_anchor_proximity(e);
            if (prox == PROX_AMBIGUOUS)  // fail-safe toward the compliant outcome
                prox = (e->criteria == STAY_NEAR) ? PROX_NEAR : PROX_AWAY;
            if (e->criteria == STAY_NEAR) return (prox == PROX_NEAR);
            else                          return (prox == PROX_AWAY);
        }
        case PHONE_AWAY: {
            // Mode B: the phone is docked at anchorId, so proximity to that anchor
            // ≈ proximity to the phone. The user must stay away from it.
            //  - The anchor reports whether the phone is still docked (§4.11). The
            //    phone is "with the user" if the user went to the dock (prox NEAR)
            //    OR the phone left the dock (undocked). dock==-1 (unknown) → docked
            //    (fail-open on the dock signal).
            //  - Fail OPEN: an uncertain/degraded watch↔anchor link (AMBIGUOUS)
            //    resolves to compliant — never false-alarm on a bad connection.
            //  - Tolerance: a brief NEAR or brief undock ("a quick check is fine")
            //    is allowed; only sustained access past PHONE_AWAY_TOLERANCE_S enforces.
            int8_t dock = -1;
            ProxProximity prox = query_anchor_proximity(e, &dock);
            bool undocked   = (dock == 0);
            bool near_phone = undocked || (prox == PROX_NEAR);
            if (!near_phone) { g_phone_near_since_ts = 0; return true; }  // compliant; reset grace

            uint32_t now_ts = (uint32_t)time(nullptr);
            if (g_phone_near_since_ts == 0) g_phone_near_since_ts = now_ts;
            uint32_t near_for = now_ts - g_phone_near_since_ts;
            if (near_for < (uint32_t)PHONE_AWAY_TOLERANCE_S) {
                Serial.printf("[PROX] phoneAway: near-phone (%s) for %us < %ds grace — compliant\n",
                              undocked ? "undocked" : "at dock", (unsigned)near_for, PHONE_AWAY_TOLERANCE_S);
                return true;   // within grace → treat as met
            }
            Serial.printf("[PROX] phoneAway: near-phone (%s) for %us >= grace — enforcing\n",
                          undocked ? "undocked" : "at dock", (unsigned)near_for);
            return false;      // grace exceeded → enforce
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
    g_phone_near_since_ts = 0;   // reset phoneAway tolerance grace for the new event

    // §5.4.4 window-start worn check: if this event beeps anchors and the watch
    // is already unworn at window start, notify beepAnchors immediately — do not
    // wait for a worn transition (a watch on the charger never transitions).
    if (e->beepAnchorCount > 0 && !g_worn) {
        Serial.println("[GRACE] Unworn at window start — sending WATCH_REMOVED");
        send_watch_removed_to_anchors(e);
    }

    // §5.4.4 donning grace: if the watch is already worn when the window opens,
    // start the grace from the window start (donning just before the window must
    // not be punished). If unworn, grace begins when the watch is next donned.
    g_grace_deadline   = 0;
    g_grace_was_active = false;
    if (e->donningGraceS > 0 && g_worn) {
        g_grace_deadline   = time(nullptr) + e->donningGraceS;
        g_grace_was_active = true;
        Serial.printf("[GRACE] Worn at window start — grace %us\n", e->donningGraceS);
    }

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
    g_grace_deadline   = 0;   // clear donning grace (§5.4.4)
    g_grace_was_active = false;
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
//  Commitment Integrity — §9 (Phases 2 + 3)
//
//  The watch is the root of trust: the app proposes, the watch disposes.
//  Every schedule push runs through the diff gate (§9.3): tightenings and new
//  events apply immediately; loosenings are quarantined into a pending queue
//  for LOOSEN_DELAY_H hours unless a §9.3 exception applies (unsettled event
//  still above its settled baseline, or a far-future event). The queue is
//  promoted autonomously (§9.4) and exposed via the Pending Changes
//  characteristic …001A (§9.5). The emergency-pass ledger (§9.6), time
//  hardening (§9.7), and settings gating (§9.8) are Phase 3.
//
//  All integrity timers count MONOTONIC ELAPSED TIME via a persisted
//  accumulator (the same pattern as the MEASURE_USAGE instrumentation), never
//  wall clock — a Time write cannot accelerate them (§9.2).
// ============================================================

// EnforcementProfile strictness rank: strict(2) > normal(1) > loose(0).
static int profile_strictness(EnforcementProfile p) { return 2 - ((int)p % 3); }
// EnforcementProfile output category: 0=silent, 1=both, 2=buzz (id/3).
static int profile_output(EnforcementProfile p) { return (int)p / 3; }

// Does output category `a` bind at least as hard as `b` in the §9.1 partial
// order? both dominates silent and buzz; silent vs buzz are non-comparable.
static bool output_ge(int a, int b) {
    if (a == b) return true;
    if (a == 1) return true;    // both >= anything
    return false;               // a != both and a != b → below or non-comparable
}

// True iff `neu` binds at least as hard as `old` across every §9.1 dimension
// (i.e. the change is a tightening or leaves the event unchanged). Any loosening
// or non-comparable field makes the whole change unacceptable (returns false).
static bool event_binds_at_least_as_hard(const Event &old_e, const Event &neu) {
    // Negate added is always a loosening (§9.1); negate removed re-arms the
    // day, which binds harder, so it falls through.
    if (neu.negate && !old_e.negate) return false;

    // Window: new must contain old (start no later, end no earlier).
    if (neu.startTime > old_e.startTime) return false;
    if (neu.endTime   < old_e.endTime)   return false;

    // Criteria: any change is non-comparable.
    if (neu.criteria != old_e.criteria) return false;

    // Target (anchorId / wifiSSID): any change is non-comparable.
    if (neu.hasAnchorId != old_e.hasAnchorId) return false;
    if (neu.hasAnchorId && memcmp(neu.anchorId, old_e.anchorId, 16) != 0) return false;
    if (!neu.hasAnchorId && strncmp(neu.wifiSSID, old_e.wifiSSID, 64) != 0) return false;

    // Profile: must be >= in both strictness and output (partial order §9.1).
    if (profile_strictness(neu.profile) < profile_strictness(old_e.profile)) return false;
    if (!output_ge(profile_output(neu.profile), profile_output(old_e.profile))) return false;

    // beepAnchors: new must be a superset of old (every old anchor still present).
    for (int i = 0; i < old_e.beepAnchorCount; i++) {
        bool present = false;
        for (int j = 0; j < neu.beepAnchorCount; j++)
            if (uuid_eq(old_e.beepAnchors[i], neu.beepAnchors[j])) { present = true; break; }
        if (!present) return false;
    }

    // anchorProfile: hard(2) > medium(1) > light(0); new must be >= old.
    if (old_e.hasAnchorProfile) {
        if (!neu.hasAnchorProfile) return false;
        if ((int)neu.anchorProfile < (int)old_e.anchorProfile) return false;
    }

    // donningGraceS: decrease is tightening, increase is loosening (§9.1).
    if (neu.donningGraceS > old_e.donningGraceS) return false;

    // Recurrence: new occurrence set must cover old. Conservative — accept only
    // an unchanged recurrence, or a widening to daily (daily ⊇ any set).
    bool recurrence_same = (neu.recurrenceType == old_e.recurrenceType &&
                            neu.dayOfWeek  == old_e.dayOfWeek &&
                            neu.dayOfMonth == old_e.dayOfMonth);
    if (!recurrence_same && neu.recurrenceType != DAILY) return false;

    return true;
}

// ---- Elapsed-time accumulator (§9.2) --------------------------------------
// Monotonic elapsed seconds spanning reboots: persisted base + esp_timer time
// this boot (esp_timer advances across light sleep). Saved on the 5-min battery
// heartbeat and on every integrity mutation, so a crash loses at most a few
// minutes — timers then mature slightly LATE, never early.

static uint64_t g_integ_boot_us        = 0;   // esp_timer at integrity init
static uint32_t g_integ_elapsed_base_s = 0;   // seconds carried over from prior boots (NVS)

static uint32_t integ_elapsed_now() {
    return g_integ_elapsed_base_s +
           (uint32_t)((esp_timer_get_time() - g_integ_boot_us) / 1000000ULL);
}

static void integ_save_elapsed() {
    prefs.begin("integ", false);
    prefs.putUInt("elap", integ_elapsed_now());
    prefs.end();
}

// ---- Persistent state (§9.2/§9.3/§9.6) ------------------------------------

// Per-event settle state. An event is identified by (uuid, once?, once-date):
// a recurring event and a once-event (e.g. its negate) legitimately share a
// UUID, so the UUID alone is not a unique key within the schedule blob.
struct SettleRecord {
    bool     valid;
    bool     settled;             // baseline reflects the current event state
    bool     has_baseline;
    bool     once;                // key: recurrenceType == once
    uint8_t  uuid[16];            // key
    int32_t  once_date;           // key: local YYYYMMDD for once events, 0 otherwise
    uint32_t last_edit_elapsed;   // elapsed stamp of the last accepted change
    Event    baseline;            // snapshot as of the most recent settle
};

// One quarantined loosening (§9.3). Stores the FULL proposed post-change state
// so promotion needs no app involvement.
struct PendingEntry {
    bool     valid;
    uint8_t  change_type;    // wire values (§9.5): 0=delete, 1=loosen-modify, 2=negate-day, 3=setting
    uint8_t  uuid[16];       // event UUID (zero-filled for setting entries)
    bool     once;           // key of the target schedule entry (types 0/1)
    int32_t  once_date;
    uint32_t apply_after;    // elapsed-seconds stamp at which this promotes
    Event    proposed;       // types 1/2: complete proposed event (incl. the negate event)
    int32_t  negate_date;    // type 2: local YYYYMMDD being negated
    uint8_t  setting_id;     // type 3: 1=disc_dorm, 2=away_dorm, 3=settle_window, 4=pass_allowance
    uint16_t setting_value;  // type 3
};

static SettleRecord g_settle[MAX_SETTLE_RECORDS]   = {};
static PendingEntry g_pending[PENDING_QUEUE_MAX]   = {};

// Emergency-pass ledger (§9.6): allowance + elapsed stamps of spends. 0 = free slot.
static uint8_t  g_pass_allowance               = PASS_BUDGET_DEFAULT;
static uint32_t g_pass_spends[PASS_MAX_STAMPS] = {};

static NimBLECharacteristic *g_pending_char = nullptr;
static NimBLECharacteristic *g_pass_char    = nullptr;

// Second full-schedule scratch: the diff gate needs current and proposed
// parsed simultaneously. Same synchronous-main-task discipline as
// g_all_events_scratch.
static Event g_proposed_scratch[MAX_EVENTS_PER_DAY * 2];

static void save_settle_records() {
    prefs.begin("integ", false);
    prefs.putBytes("settle", g_settle, sizeof(g_settle));
    prefs.putUInt("elap", integ_elapsed_now());
    prefs.end();
}

static void save_pending_queue() {
    prefs.begin("integ", false);
    prefs.putBytes("pending", g_pending, sizeof(g_pending));
    prefs.putUInt("elap", integ_elapsed_now());
    prefs.end();
}

static void save_pass_ledger() {
    prefs.begin("integ", false);
    prefs.putUChar("allow", g_pass_allowance);
    prefs.putBytes("spends", g_pass_spends, sizeof(g_pass_spends));
    prefs.putUInt("elap", integ_elapsed_now());
    prefs.end();
}

// ---- Event-key helpers ------------------------------------------------------

static bool event_is_once(const Event &e) { return e.recurrenceType == ONCE; }

static int32_t event_once_date(const Event &e) {
    return event_is_once(e) ? date_int_of_utc(e.referenceDate) : 0;
}

static int find_event_index(const Event *arr, int n,
                            const uint8_t *uuid, bool once, int32_t date) {
    for (int i = 0; i < n; i++) {
        if (uuid_eq(arr[i].id, uuid) &&
            event_is_once(arr[i]) == once &&
            event_once_date(arr[i]) == date) return i;
    }
    return -1;
}

// ---- Settle-record helpers (§9.2) ------------------------------------------

static SettleRecord* find_settle_record(const uint8_t *uuid, bool once, int32_t date) {
    for (int i = 0; i < MAX_SETTLE_RECORDS; i++) {
        SettleRecord &r = g_settle[i];
        if (r.valid && uuid_eq(r.uuid, uuid) && r.once == once && r.once_date == date)
            return &r;
    }
    return nullptr;
}

// Records an accepted change to the event: stamps last_edit and marks it
// unsettled (existing baseline is retained until the next settle). Creates the
// record if missing — a brand-new event starts with no baseline, which is the
// §9.3 free first-setup window.
static void mark_event_edited(const Event &e) {
    bool    once = event_is_once(e);
    int32_t date = event_once_date(e);
    SettleRecord *r = find_settle_record(e.id, once, date);
    if (!r) {
        for (int i = 0; i < MAX_SETTLE_RECORDS; i++) {
            if (!g_settle[i].valid) { r = &g_settle[i]; break; }
        }
        if (!r) return;   // table full: event stays untracked (treated as settled — conservative)
        memset(r, 0, sizeof(*r));
        r->valid = true;
        memcpy(r->uuid, e.id, 16);
        r->once      = once;
        r->once_date = date;
    }
    r->last_edit_elapsed = integ_elapsed_now();
    r->settled           = false;
}

static void remove_settle_record(const Event &e) {
    SettleRecord *r = find_settle_record(e.id, event_is_once(e), event_once_date(e));
    if (r) r->valid = false;
}

// ---- Pending-queue helpers (§9.3/§9.5) --------------------------------------

// Builds the §9.5 wire payload into buf; returns its length.
static int pending_build_payload(uint8_t *buf) {
    uint32_t now_el = integ_elapsed_now();
    int pos = 1, n = 0;
    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        const PendingEntry &p = g_pending[i];
        if (!p.valid) continue;
        memcpy(buf + pos, p.uuid, 16); pos += 16;
        buf[pos++] = p.change_type;
        uint32_t secs = (p.apply_after > now_el) ? (p.apply_after - now_el) : 0;
        memcpy(buf + pos, &secs, 4); pos += 4;
        n++;
    }
    buf[0] = (uint8_t)n;
    return pos;
}

// Re-publishes the queue on the Pending Changes characteristic (§9.5) and
// notifies. Called whenever an entry is added, promoted, or canceled.
static void pending_publish() {
    if (!g_pending_char) return;
    uint8_t buf[1 + PENDING_QUEUE_MAX * 21];
    int len = pending_build_payload(buf);
    g_pending_char->setValue(buf, len);
    g_pending_char->notify();
}

// Cancels all event-type pending entries for a UUID (§9.3: a newly accepted
// change to an event supersedes its pending entries). Returns true if any
// entry was removed.
static bool cancel_pending_for_uuid(const uint8_t *uuid) {
    bool changed = false;
    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        PendingEntry &p = g_pending[i];
        if (p.valid && p.change_type <= 2 && uuid_eq(p.uuid, uuid)) {
            p.valid = false;
            changed = true;
        }
    }
    return changed;
}

static bool cancel_pending_for_setting(uint8_t setting_id) {
    bool changed = false;
    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        PendingEntry &p = g_pending[i];
        if (p.valid && p.change_type == 3 && p.setting_id == setting_id) {
            p.valid = false;
            changed = true;
        }
    }
    return changed;
}

// Finds the slot for a new pending entry. An existing entry for the same
// logical target is REPLACED (the newer request wins and its 24 h restarts):
// delete/modify share a bucket per event key; negate-day is keyed by date;
// settings by setting id. Returns nullptr when the queue is full (§9.3: the
// loosening is dropped, not applied).
static PendingEntry* pending_slot_for(uint8_t type, const uint8_t *uuid,
                                      bool once, int32_t date, uint8_t setting_id) {
    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        PendingEntry &p = g_pending[i];
        if (!p.valid) continue;
        if (type <= 1 && p.change_type <= 1 &&
            uuid_eq(p.uuid, uuid) && p.once == once && p.once_date == date) return &p;
        if (type == 2 && p.change_type == 2 &&
            uuid_eq(p.uuid, uuid) && p.negate_date == date) return &p;
        if (type == 3 && p.change_type == 3 && p.setting_id == setting_id) return &p;
    }
    for (int i = 0; i < PENDING_QUEUE_MAX; i++)
        if (!g_pending[i].valid) return &g_pending[i];
    return nullptr;
}

// Queues a quarantined event change (types 0/1/2). `proposed` is null for a
// delete. Returns true if the entry was stored (false = queue full → dropped).
static bool queue_pending_event(uint8_t type, const Event &target,
                                const Event *proposed, int32_t negate_date) {
    int32_t key_date = (type == 2) ? negate_date : event_once_date(target);
    PendingEntry *p = pending_slot_for(type, target.id,
                                       (type == 2) ? true : event_is_once(target),
                                       key_date, 0);
    if (!p) {
        Serial.println("[INTEG] Pending queue full — loosening dropped");
        return false;
    }
    memset(p, 0, sizeof(*p));
    p->valid       = true;
    p->change_type = type;
    memcpy(p->uuid, target.id, 16);
    p->once        = (type == 2) ? true : event_is_once(target);
    p->once_date   = key_date;
    p->apply_after = integ_elapsed_now() + (uint32_t)LOOSEN_DELAY_H * 3600UL;
    if (proposed) p->proposed = *proposed;
    p->negate_date = negate_date;
    return true;
}

static bool queue_pending_setting(uint8_t setting_id, uint16_t value) {
    PendingEntry *p = pending_slot_for(3, nullptr, false, 0, setting_id);
    if (!p) {
        Serial.println("[INTEG] Pending queue full — setting loosening dropped");
        return false;
    }
    memset(p, 0, sizeof(*p));
    p->valid         = true;
    p->change_type   = 3;
    p->apply_after   = integ_elapsed_now() + (uint32_t)LOOSEN_DELAY_H * 3600UL;
    p->setting_id    = setting_id;
    p->setting_value = value;
    return true;
}

// ---- Schedule re-serialization (§6.2 v2 blob) --------------------------------
// The stored schedule is no longer the raw received blob: the gate composes
// old + accepted changes, so the watch must write its own blobs.

static uint8_t* serialize_schedule(const Event *evts, int count, uint32_t *out_len) {
    uint32_t sz = 3;   // version byte + count u16
    for (int i = 0; i < count; i++) {
        uint32_t ssid_len = strnlen(evts[i].wifiSSID, 64);
        sz += 56 + ssid_len + 16u * evts[i].beepAnchorCount;
    }
    uint8_t *buf = (uint8_t*)malloc(sz);
    if (!buf) return nullptr;

    uint32_t pos = 0;
    buf[pos++] = SCHEDULE_FORMAT_VERSION;
    uint16_t n = (uint16_t)count;
    memcpy(buf + pos, &n, 2); pos += 2;
    for (int i = 0; i < count; i++) {
        const Event &e = evts[i];
        memcpy(buf + pos, e.id, 16); pos += 16;
        memcpy(buf + pos, &e.referenceDate, 8); pos += 8;
        memcpy(buf + pos, &e.startTime, 2); pos += 2;
        memcpy(buf + pos, &e.endTime, 2); pos += 2;
        buf[pos++] = (uint8_t)e.recurrenceType;
        buf[pos++] = e.dayOfWeek;
        buf[pos++] = e.dayOfMonth;
        buf[pos++] = (uint8_t)e.criteria;
        buf[pos++] = (uint8_t)e.profile;
        buf[pos++] = e.hasAnchorProfile ? (uint8_t)e.anchorProfile : 0xFF;
        buf[pos++] = e.negate ? 1 : 0;
        memcpy(buf + pos, &e.donningGraceS, 2); pos += 2;
        buf[pos++] = e.hasAnchorId ? 1 : 0;
        memcpy(buf + pos, e.anchorId, 16); pos += 16;
        uint8_t ssid_len = (uint8_t)strnlen(e.wifiSSID, 64);
        buf[pos++] = ssid_len;
        if (ssid_len) { memcpy(buf + pos, e.wifiSSID, ssid_len); pos += ssid_len; }
        buf[pos++] = e.beepAnchorCount;
        for (int j = 0; j < e.beepAnchorCount; j++) {
            memcpy(buf + pos, e.beepAnchors[j], 16); pos += 16;
        }
    }
    *out_len = pos;
    return buf;
}

// Serializes and stores the given event set as the current schedule (RAM + NVS).
static bool store_current_schedule(const Event *evts, int count) {
    uint32_t len = 0;
    uint8_t *blob = serialize_schedule(evts, count, &len);
    if (!blob) return false;
    if (g_sched_blob) free(g_sched_blob);
    g_sched_blob     = blob;
    g_sched_blob_len = len;
    save_schedule_blob(blob, len);
    return true;
}

// ---- Calendar helpers -------------------------------------------------------

// Days since 1970-01-01 for a civil date (Howard Hinnant's algorithm).
static int64_t days_from_civil(int y, int m, int d) {
    y -= m <= 2;
    int      era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153u * (unsigned)(m + (m > 2 ? -3 : 9)) + 2u) / 5u + (unsigned)d - 1u;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

// UTC epoch for a local date (YYYYMMDD) at a given minutes-since-midnight.
static int64_t local_ymd_to_utc_epoch(int32_t ymd, uint32_t minute_of_day) {
    int y = ymd / 10000, m = (ymd / 100) % 100, d = ymd % 100;
    return days_from_civil(y, m, d) * 86400LL +
           (int64_t)minute_of_day * 60 - (int64_t)g_tz_offset_min * 60;
}

// Seconds (wall clock) until the event's next window start; UINT32_MAX if it
// never occurs again (a once event whose date has passed). Used only for the
// §9.3 far-future exception — this is schedule-domain time, not an integrity
// timer, so the wall clock is the correct basis here.
static uint32_t next_occurrence_start_s(const Event &e) {
    time_t now_utc = time(nullptr);
    for (int off = 0; off <= 400; off++) {
        time_t lt = now_utc + (time_t)g_tz_offset_min * 60 + (time_t)off * 86400;
        struct tm ti; gmtime_r(&lt, &ti);
        int32_t dint = (ti.tm_year + 1900) * 10000 + (ti.tm_mon + 1) * 100 + ti.tm_mday;
        uint8_t dow  = (ti.tm_wday == 0) ? 7 : (uint8_t)ti.tm_wday;
        bool applies = false;
        switch (e.recurrenceType) {
            case DAILY:   applies = true;                              break;
            case WEEKLY:  applies = (e.dayOfWeek  == dow);             break;
            case MONTHLY: applies = (e.dayOfMonth == ti.tm_mday);      break;
            case ONCE:    applies = (event_once_date(e) == dint);      break;
        }
        if (!applies) continue;
        int64_t diff = local_ymd_to_utc_epoch(dint, e.startTime) - (int64_t)now_utc;
        if (diff > 0)
            return (diff > 0xFFFFFFFELL) ? 0xFFFFFFFEu : (uint32_t)diff;
    }
    return 0xFFFFFFFFu;
}

static bool event_is_currently_active(const uint8_t *uuid) {
    return g_activity_state == STATE_ENFORCEMENT && g_active_event &&
           uuid_eq(g_active_event->id, uuid);
}

// ---- §9.3 loosening exceptions ------------------------------------------------

// True if a loosening of `cur` (to `proposed`, or a delete when null) applies
// immediately instead of being quarantined.
static bool loosening_applies_immediately(const Event &cur, const Event *proposed) {
    // Unsettled-event exception: within the settle window, edits that stay at
    // or above the settled baseline are free; an event with no baseline yet is
    // in its first-setup window and fully free. A missing record is treated as
    // settled (conservative).
    SettleRecord *rec = find_settle_record(cur.id, event_is_once(cur), event_once_date(cur));
    if (rec && !rec->settled) {
        if (!rec->has_baseline) return true;
        if (proposed && event_binds_at_least_as_hard(rec->baseline, *proposed)) return true;
    }
    // No-escape exception: loosening an event that is not active now and whose
    // next occurrence is beyond the horizon grants no in-the-moment escape.
    if (!event_is_currently_active(cur.id) &&
        next_occurrence_start_s(cur) > (uint32_t)LOOSEN_FREE_HORIZON_H * 3600UL)
        return true;
    return false;
}

// ---- §9.3 schedule-diff gate ---------------------------------------------------

// Runs the full commitment-integrity diff gate on a CRC- and version-valid
// proposed blob. Returns the §6.2 ctrl response: 0x01 full accept, 0x03 one or
// more loosenings quarantined, 0x00 malformed.
static uint8_t apply_schedule_diff_gate(const uint8_t *blob, uint32_t len) {
    Event *cur = g_all_events_scratch;
    int    nc  = 0;
    if (g_sched_blob && g_sched_blob_len >= 3)
        deserialize_events(g_sched_blob, g_sched_blob_len, cur, &nc, MAX_EVENTS_PER_DAY * 2);

    Event *prop = g_proposed_scratch;
    int    np   = 0;
    if (!deserialize_events(blob, len, prop, &np, MAX_EVENTS_PER_DAY * 2))
        return 0x00;
    int np_prop = np;   // proposed entries; kept-alive current events append after

    static bool drop[MAX_EVENTS_PER_DAY * 2];
    memset(drop, 0, sizeof(drop));

    bool any_quarantine = false;
    bool pending_changed = false;

    // Pass 1 — proposed entries: adds and modifications.
    for (int i = 0; i < np_prop; i++) {
        Event &b = prop[i];
        int ai = find_event_index(cur, nc, b.id, event_is_once(b), event_once_date(b));

        if (ai < 0 && b.negate && event_is_once(b)) {
            // One-time negate added. If it targets a live event with the same
            // UUID it cancels a day — a loosening (§9.1).
            int ti = -1;
            for (int j = 0; j < nc; j++)
                if (uuid_eq(cur[j].id, b.id) && !cur[j].negate) { ti = j; break; }
            if (ti >= 0) {
                const Event &target = cur[ti];
                int32_t nd = event_once_date(b);
                bool immediate = false;
                SettleRecord *rec = find_settle_record(target.id, event_is_once(target),
                                                       event_once_date(target));
                if (rec && !rec->settled && !rec->has_baseline)
                    immediate = true;   // free first-setup window
                if (!immediate && !event_is_currently_active(target.id)) {
                    int64_t su = local_ymd_to_utc_epoch(nd, target.startTime) - (int64_t)time(nullptr);
                    // Far-future negate grants no escape; a past-date negate is inert.
                    if (su > (int64_t)LOOSEN_FREE_HORIZON_H * 3600 || su <= 0) immediate = true;
                }
                if (immediate) {
                    mark_event_edited(b);
                    pending_changed |= cancel_pending_for_uuid(b.id);
                } else {
                    queue_pending_event(2, target, &b, nd);
                    drop[i] = true;
                    any_quarantine = true;
                    pending_changed = true;
                }
            } else {
                mark_event_edited(b);   // negate with no live target: inert, accept
            }
            continue;
        }

        if (ai < 0) {
            // New event: always a tightening (§9.1) → apply, open its settle window.
            mark_event_edited(b);
            continue;
        }

        Event &a = cur[ai];
        if (memcmp(&a, &b, sizeof(Event)) == 0) continue;   // unchanged

        bool accept = event_binds_at_least_as_hard(a, b) ||   // tightening
                      loosening_applies_immediately(a, &b);   // §9.3 exceptions
        if (accept) {
            mark_event_edited(b);
            pending_changed |= cancel_pending_for_uuid(b.id);
        } else {
            queue_pending_event(1, a, &b, 0);
            b = a;                    // keep the current state in the result
            any_quarantine = true;
            pending_changed = true;
        }
    }

    // Pass 2 — deletions: current entries absent from the proposal.
    for (int j = 0; j < nc; j++) {
        Event &a = cur[j];
        if (find_event_index(prop, np_prop, a.id, event_is_once(a), event_once_date(a)) >= 0)
            continue;   // still present (possibly restored to current state above)

        if (a.negate && event_is_once(a)) {
            // Removing a negate re-arms the canceled day → tightening → accept.
            remove_settle_record(a);
            pending_changed |= cancel_pending_for_uuid(a.id);
            continue;
        }
        if (loosening_applies_immediately(a, nullptr)) {
            remove_settle_record(a);
            pending_changed |= cancel_pending_for_uuid(a.id);
        } else {
            queue_pending_event(0, a, nullptr, 0);
            if (np < MAX_EVENTS_PER_DAY * 2) { prop[np] = a; drop[np] = false; np++; }
            any_quarantine = true;
            pending_changed = true;
        }
    }

    // Compact the result and make it the current schedule.
    int nr = 0;
    for (int i = 0; i < np; i++)
        if (!drop[i]) prop[nr++] = prop[i];

    if (!store_current_schedule(prop, nr)) return 0x00;
    recalculate_and_rearm();
    save_settle_records();
    if (pending_changed) {
        save_pending_queue();
        pending_publish();
    }

    Serial.printf("[INTEG] Diff gate: %d event(s) stored%s\n", nr,
                  any_quarantine ? ", loosenings quarantined" : "");
    return any_quarantine ? 0x03 : 0x01;
}

// ---- §9.2 settle promotion + §9.4 autonomous pending promotion -----------------

// Settles events whose window has elapsed with no accepted change: snapshots
// their current state into settled_baseline.
static void settle_tick() {
    uint32_t now_el   = integ_elapsed_now();
    uint32_t window_s = (uint32_t)g_settle_window_min * 60UL;
    bool any_due = false;
    for (int i = 0; i < MAX_SETTLE_RECORDS; i++) {
        SettleRecord &r = g_settle[i];
        if (r.valid && !r.settled && now_el - r.last_edit_elapsed >= window_s) {
            any_due = true; break;
        }
    }
    if (!any_due) return;

    Event *cur = g_all_events_scratch;
    int    nc  = 0;
    if (g_sched_blob && g_sched_blob_len >= 3)
        deserialize_events(g_sched_blob, g_sched_blob_len, cur, &nc, MAX_EVENTS_PER_DAY * 2);

    for (int i = 0; i < MAX_SETTLE_RECORDS; i++) {
        SettleRecord &r = g_settle[i];
        if (!r.valid || r.settled || now_el - r.last_edit_elapsed < window_s) continue;
        int idx = find_event_index(cur, nc, r.uuid, r.once, r.once_date);
        if (idx < 0) { r.valid = false; continue; }   // event gone — drop the record
        r.baseline     = cur[idx];
        r.has_baseline = true;
        r.settled      = true;
        Serial.printf("[INTEG] Event %02x… settled — baseline updated\n", r.uuid[0]);
    }
    save_settle_records();
}

// Applies matured pending entries (§9.4). Runs with no app involvement on every
// wake; a sleeping watch promotes late, never early. Entries whose effect has
// expired are dropped.
static void promote_pending() {
    uint32_t now_el = integ_elapsed_now();
    bool any_mature = false;
    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        if (g_pending[i].valid && g_pending[i].apply_after <= now_el) { any_mature = true; break; }
    }
    if (!any_mature) return;

    Event *cur = g_all_events_scratch;
    int    nc  = 0;
    if (g_sched_blob && g_sched_blob_len >= 3)
        deserialize_events(g_sched_blob, g_sched_blob_len, cur, &nc, MAX_EVENTS_PER_DAY * 2);

    int32_t today = local_date_int();
    bool sched_changed = false;

    for (int i = 0; i < PENDING_QUEUE_MAX; i++) {
        PendingEntry &p = g_pending[i];
        if (!p.valid || p.apply_after > now_el) continue;

        switch (p.change_type) {
            case 0: {   // delete
                int idx = find_event_index(cur, nc, p.uuid, p.once, p.once_date);
                if (idx >= 0) {
                    remove_settle_record(cur[idx]);
                    for (int k = idx; k < nc - 1; k++) cur[k] = cur[k + 1];
                    nc--;
                    sched_changed = true;
                    Serial.printf("[INTEG] Promoted delete of %02x…\n", p.uuid[0]);
                }
                break;   // target already gone → drop silently
            }
            case 1: {   // loosen-modify
                if (p.once && p.once_date < today) break;   // expired once event → drop
                int idx = find_event_index(cur, nc, p.uuid, p.once, p.once_date);
                if (idx >= 0) {
                    cur[idx] = p.proposed;
                    mark_event_edited(p.proposed);
                    sched_changed = true;
                    Serial.printf("[INTEG] Promoted loosening of %02x…\n", p.uuid[0]);
                }
                break;
            }
            case 2: {   // negate-day
                if (p.negate_date < today) break;           // day already passed → drop
                int idx = find_event_index(cur, nc, p.uuid, true, p.negate_date);
                if (idx >= 0) cur[idx] = p.proposed;
                else if (nc < MAX_EVENTS_PER_DAY * 2) cur[nc++] = p.proposed;
                mark_event_edited(p.proposed);
                sched_changed = true;
                Serial.printf("[INTEG] Promoted negate-day %ld for %02x…\n",
                              (long)p.negate_date, p.uuid[0]);
                break;
            }
            case 3: {   // setting change
                switch (p.setting_id) {
                    case 1: g_disconnected_is_dormant = (p.setting_value != 0); break;
                    case 2: g_away_is_dormant         = (p.setting_value != 0); break;
                    case 3: {
                        uint16_t sw = p.setting_value;
                        if (sw < SETTLE_WINDOW_FLOOR_MIN) sw = SETTLE_WINDOW_FLOOR_MIN;
                        if (sw > SETTLE_WINDOW_CEIL_MIN)  sw = SETTLE_WINDOW_CEIL_MIN;
                        g_settle_window_min = sw;
                        break;
                    }
                    case 4:
                        g_pass_allowance = (uint8_t)p.setting_value;
                        save_pass_ledger();
                        break;
                }
                prefs.begin("watch", false);
                prefs.putBool("disc_dorm", g_disconnected_is_dormant);
                prefs.putBool("away_dorm", g_away_is_dormant);
                prefs.putUShort("settle_min", g_settle_window_min);
                prefs.end();
                Serial.printf("[INTEG] Promoted setting %d = %u\n",
                              p.setting_id, p.setting_value);
                break;
            }
        }
        p.valid = false;
    }

    if (sched_changed) {
        store_current_schedule(cur, nc);
        recalculate_and_rearm();
        save_settle_records();
    }
    save_pending_queue();
    pending_publish();
}

// ---- §9.7 time hardening --------------------------------------------------------

// True if setting the clock/timezone to (new_utc, new_tz) would place the
// current local time outside the CURRENTLY ACTIVE event's window (the previous
// time is inside by definition of "active"). Such writes are rejected (0x02).
static bool time_write_would_escape_active_window(int64_t new_utc, int16_t new_tz) {
    if (g_activity_state != STATE_ENFORCEMENT || !g_active_event) return false;
    time_t cur_lt = time(nullptr) + (time_t)g_tz_offset_min * 60;
    time_t new_lt = (time_t)new_utc + (time_t)new_tz * 60;
    struct tm ct, nt;
    gmtime_r(&cur_lt, &ct);
    gmtime_r(&new_lt, &nt);
    bool same_day = (ct.tm_year == nt.tm_year && ct.tm_mon == nt.tm_mon &&
                     ct.tm_mday == nt.tm_mday);
    uint32_t new_min = (uint32_t)(nt.tm_hour * 60 + nt.tm_min);
    bool inside = same_day &&
                  new_min >= g_active_event->startTime &&
                  new_min <  g_active_event->endTime;
    return !inside;
}

// ---- §9.6 emergency-pass ledger ---------------------------------------------------

// Drops spend stamps older than the rolling window; returns spends in window.
static int pass_prune_and_count() {
    uint32_t now_el = integ_elapsed_now();
    uint32_t win_s  = (uint32_t)PASS_WINDOW_DAYS * 86400UL;
    int cnt = 0;
    for (int i = 0; i < PASS_MAX_STAMPS; i++) {
        if (g_pass_spends[i] == 0) continue;
        if (now_el - g_pass_spends[i] >= win_s) g_pass_spends[i] = 0;
        else cnt++;
    }
    return cnt;
}

// ---- Init + per-wake tick ---------------------------------------------------------

static void integrity_init() {
    g_integ_boot_us = esp_timer_get_time();
    prefs.begin("integ", true);
    g_integ_elapsed_base_s = prefs.getUInt("elap", 0);
    if (prefs.getBytesLength("settle") == sizeof(g_settle))
        prefs.getBytes("settle", g_settle, sizeof(g_settle));
    if (prefs.getBytesLength("pending") == sizeof(g_pending))
        prefs.getBytes("pending", g_pending, sizeof(g_pending));
    g_pass_allowance = prefs.getUChar("allow", PASS_BUDGET_DEFAULT);
    if (prefs.getBytesLength("spends") == sizeof(g_pass_spends))
        prefs.getBytes("spends", g_pass_spends, sizeof(g_pass_spends));
    prefs.end();

    // Bootstrap: schedule events that predate settle tracking (first boot of
    // this firmware) are treated as SETTLED at their current state, so a
    // loosening right after the update is quarantined rather than free. New
    // events created through the gate get the normal first-setup window.
    if (g_sched_blob && g_sched_blob_len >= 3) {
        Event *cur = g_all_events_scratch;
        int    nc  = 0;
        bool changed = false;
        if (deserialize_events(g_sched_blob, g_sched_blob_len, cur, &nc,
                               MAX_EVENTS_PER_DAY * 2)) {
            for (int i = 0; i < nc; i++) {
                const Event &e = cur[i];
                if (find_settle_record(e.id, event_is_once(e), event_once_date(e)))
                    continue;
                for (int s = 0; s < MAX_SETTLE_RECORDS; s++) {
                    if (g_settle[s].valid) continue;
                    SettleRecord &r = g_settle[s];
                    memset(&r, 0, sizeof(r));
                    r.valid = true;
                    memcpy(r.uuid, e.id, 16);
                    r.once              = event_is_once(e);
                    r.once_date         = event_once_date(e);
                    r.last_edit_elapsed = integ_elapsed_now();
                    r.baseline          = e;
                    r.has_baseline      = true;
                    r.settled           = true;
                    changed = true;
                    break;
                }
            }
        }
        if (changed) save_settle_records();
    }
    Serial.printf("[INTEG] Init: elapsed=%lus allowance=%d\n",
                  (unsigned long)integ_elapsed_now(), g_pass_allowance);
}

// Runs the integrity timers. Called from the main loop (which executes after
// every wake: boot, RTC boundary, midnight, motion, BLE) on a 1 s cadence.
static void integrity_tick() {
    settle_tick();
    promote_pending();
}

// ============================================================
//  Battery usage instrumentation  (MEASURE_USAGE)
// ============================================================
// Sanity-check tooling for the overnight battery test. When MEASURE_USAGE is 1
// the firmware accumulates:
//   • total time in light sleep vs awake vs deep sleep (deep sleep is never used
//     by this firmware, so that bucket stays 0 — its presence confirms that).
//   • the average wake-to-wake interval — the full sleep+work cycle period. With
//     the disconnected heartbeat this is expected near DISCONNECTED_ADV_HEARTBEAT_MS
//     (6 s) PLUS the forced awake time before re-sleep, so a value materially
//     above 6 s is itself the signal that awake overhead is inflating the cycle.
//   • the average per-sleep duration (should sit at the ~6 s heartbeat target).
//   • the average and maximum awake-burst duration — how long the CPU stays awake
//     between sleeps. In steady DORMANT idle this is governed by
//     DORMANT_TO_SLEEP_IDLE_MS (1500 ms): the watch cannot re-sleep until it has
//     been idle that long after each wake, so ~1.5 s awake per ~7.5 s cycle (~20%
//     awake duty) is expected — this is the proposed drain-diagnostic metric.
//   • what woke the watch from each sleep (timer vs GPIO/motion). In DORMANT,
//     motion must NOT be a wake source (§8.4); any dormant GPIO wake means the
//     INT1 mute leaked and is flagged loudly.
//
// Timing uses esp_timer_get_time(), which ESP-IDF advances across light sleep by
// the real time slept, so the delta around esp_light_sleep_start() is the actual
// sleep duration even if the watch woke early.
//
// Read it over serial: send 0xAD to dump a summary, 0xAE to reset the counters.
// A summary is also printed automatically on every battery heartbeat (5 min).
#if MEASURE_USAGE
static uint64_t g_use_start_us             = 0;  // esp_timer at this boot's measurement start
static uint64_t g_use_elapsed_base_us      = 0;  // elapsed wall-time carried over from prior boots (NVS)
static uint64_t g_use_light_sleep_us       = 0;  // total actual light-sleep time
static uint64_t g_use_deep_sleep_us        = 0;  // total deep-sleep time (always 0 here)
static uint32_t g_use_sleep_count          = 0;  // completed sleeps (= wakeups)
static uint64_t g_use_dormant_sleep_us     = 0;
static uint32_t g_use_dormant_sleep_count  = 0;
static uint64_t g_use_enf_sleep_us         = 0;
static uint32_t g_use_enf_sleep_count      = 0;

// Awake-burst tracking: wake-out-of-sleep → entry-into-next-sleep.
static uint64_t g_use_awake_since_us       = 0;  // start of current burst (0 = none open)
static uint64_t g_use_awake_burst_total_us = 0;
static uint64_t g_use_awake_burst_max_us   = 0;
static uint32_t g_use_awake_burst_count    = 0;

// Wake-to-wake interval (full sleep+work cycle period).
static uint64_t g_use_last_wake_us           = 0;
static uint64_t g_use_wake_interval_total_us = 0;
static uint32_t g_use_wake_interval_count    = 0;

// Wake-cause tally. In DORMANT we expect TIMER only (motion muted per §8.4).
static uint32_t g_use_wake_timer        = 0;
static uint32_t g_use_wake_gpio         = 0;
static uint32_t g_use_wake_other        = 0;
static uint32_t g_use_dormant_gpio_wake = 0;  // motion leaking through in DORMANT

// ── NVS persistence ───────────────────────────────────────────────────────
// So the overnight totals survive an unexpected reset (brownout/watchdog) and
// the "plug in afterwards" workflow. NVS footprint is held CONSTANT and tiny:
// a single namespace ("usage") with a single fixed key ("m") holding one packed
// struct. Re-writing the same key overwrites in place — NVS marks the prior copy
// stale and garbage-collects it, so this never accumulates new entries or grows
// the partition. Written only on the 5-min heartbeat (≈288 writes/night), never
// per sleep, to keep flash wear negligible.
#define USAGE_NVS_MAGIC 0x55534731UL   // 'USG1' — struct version marker

struct UsagePersist {
    uint32_t magic;
    uint64_t elapsed_us;
    uint64_t light_sleep_us;
    uint64_t deep_sleep_us;
    uint32_t sleep_count;
    uint64_t dormant_sleep_us;
    uint32_t dormant_sleep_count;
    uint64_t enf_sleep_us;
    uint32_t enf_sleep_count;
    uint64_t awake_burst_total_us;
    uint64_t awake_burst_max_us;
    uint32_t awake_burst_count;
    uint64_t wake_interval_total_us;
    uint32_t wake_interval_count;
    uint32_t wake_timer;
    uint32_t wake_gpio;
    uint32_t wake_other;
    uint32_t dormant_gpio_wake;
};

// Total wall time measured, spanning prior boots: persisted base + time this boot.
static uint64_t usage_elapsed_us() {
    return g_use_elapsed_base_us + (esp_timer_get_time() - g_use_start_us);
}

static void usage_load() {
    UsagePersist p;
    prefs.begin("usage", true);
    size_t n  = prefs.getBytesLength("m");
    bool   ok = (n == sizeof(p)) &&
                (prefs.getBytes("m", &p, sizeof(p)) == sizeof(p)) &&
                (p.magic == USAGE_NVS_MAGIC);
    prefs.end();
    if (!ok) { g_use_elapsed_base_us = 0; return; }   // first run / stale layout

    g_use_elapsed_base_us        = p.elapsed_us;
    g_use_light_sleep_us         = p.light_sleep_us;
    g_use_deep_sleep_us          = p.deep_sleep_us;
    g_use_sleep_count            = p.sleep_count;
    g_use_dormant_sleep_us       = p.dormant_sleep_us;
    g_use_dormant_sleep_count    = p.dormant_sleep_count;
    g_use_enf_sleep_us           = p.enf_sleep_us;
    g_use_enf_sleep_count        = p.enf_sleep_count;
    g_use_awake_burst_total_us   = p.awake_burst_total_us;
    g_use_awake_burst_max_us     = p.awake_burst_max_us;
    g_use_awake_burst_count      = p.awake_burst_count;
    g_use_wake_interval_total_us = p.wake_interval_total_us;
    g_use_wake_interval_count    = p.wake_interval_count;
    g_use_wake_timer             = p.wake_timer;
    g_use_wake_gpio              = p.wake_gpio;
    g_use_wake_other             = p.wake_other;
    g_use_dormant_gpio_wake      = p.dormant_gpio_wake;
}

static void usage_save() {
    UsagePersist p;
    memset(&p, 0, sizeof(p));
    p.magic                  = USAGE_NVS_MAGIC;
    p.elapsed_us             = usage_elapsed_us();
    p.light_sleep_us         = g_use_light_sleep_us;
    p.deep_sleep_us          = g_use_deep_sleep_us;
    p.sleep_count            = g_use_sleep_count;
    p.dormant_sleep_us       = g_use_dormant_sleep_us;
    p.dormant_sleep_count    = g_use_dormant_sleep_count;
    p.enf_sleep_us           = g_use_enf_sleep_us;
    p.enf_sleep_count        = g_use_enf_sleep_count;
    p.awake_burst_total_us   = g_use_awake_burst_total_us;
    p.awake_burst_max_us     = g_use_awake_burst_max_us;
    p.awake_burst_count      = g_use_awake_burst_count;
    p.wake_interval_total_us = g_use_wake_interval_total_us;
    p.wake_interval_count    = g_use_wake_interval_count;
    p.wake_timer             = g_use_wake_timer;
    p.wake_gpio              = g_use_wake_gpio;
    p.wake_other             = g_use_wake_other;
    p.dormant_gpio_wake      = g_use_dormant_gpio_wake;

    prefs.begin("usage", false);
    prefs.putBytes("m", &p, sizeof(p));   // same key → overwrite, constant footprint
    prefs.end();
}

static void usage_init() {
    g_use_start_us       = esp_timer_get_time();
    g_use_awake_since_us = g_use_start_us;   // boot opens the first awake burst
    // g_use_last_wake_us stays 0 so the first wake after a reboot does not record
    // a bogus wake-interval spanning the downtime.
    usage_load();                            // resume prior totals if present
}

static void usage_reset() {
    uint64_t now = esp_timer_get_time();
    g_use_start_us = now;
    g_use_elapsed_base_us = 0;
    g_use_light_sleep_us = g_use_deep_sleep_us = 0;
    g_use_sleep_count = 0;
    g_use_dormant_sleep_us = g_use_enf_sleep_us = 0;
    g_use_dormant_sleep_count = g_use_enf_sleep_count = 0;
    g_use_awake_burst_total_us = g_use_awake_burst_max_us = 0;
    g_use_awake_burst_count = 0;
    g_use_awake_since_us = now;
    g_use_last_wake_us = 0;
    g_use_wake_interval_total_us = 0;
    g_use_wake_interval_count = 0;
    g_use_wake_timer = g_use_wake_gpio = g_use_wake_other = 0;
    g_use_dormant_gpio_wake = 0;
    usage_save();   // overwrite the persisted snapshot with the cleared counters
}

// Call immediately before esp_light_sleep_start(): closes the current awake burst.
static void usage_before_sleep() {
    uint64_t now = esp_timer_get_time();
    if (g_use_awake_since_us != 0) {
        uint64_t burst = now - g_use_awake_since_us;
        g_use_awake_burst_total_us += burst;
        if (burst > g_use_awake_burst_max_us) g_use_awake_burst_max_us = burst;
        g_use_awake_burst_count++;
        g_use_awake_since_us = 0;
    }
}

// Call immediately after esp_light_sleep_start() with the measured sleep time.
static void usage_after_sleep(uint64_t slept_us, bool is_enforcement) {
    uint64_t now = esp_timer_get_time();
    g_use_light_sleep_us += slept_us;
    g_use_sleep_count++;
    if (is_enforcement) { g_use_enf_sleep_us += slept_us; g_use_enf_sleep_count++; }
    else                { g_use_dormant_sleep_us += slept_us; g_use_dormant_sleep_count++; }

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: g_use_wake_timer++; break;
        case ESP_SLEEP_WAKEUP_GPIO:  g_use_wake_gpio++;
            if (!is_enforcement) g_use_dormant_gpio_wake++;
            break;
        default:                     g_use_wake_other++; break;
    }

    if (g_use_last_wake_us != 0) {
        g_use_wake_interval_total_us += (now - g_use_last_wake_us);
        g_use_wake_interval_count++;
    }
    g_use_last_wake_us   = now;
    g_use_awake_since_us = now;   // open the next awake burst
}

static void usage_report() {
    uint64_t elapsed = usage_elapsed_us();   // spans prior boots (persisted base + this boot)
    if (elapsed == 0) elapsed = 1;
    uint64_t sleep_us = g_use_light_sleep_us + g_use_deep_sleep_us;
    uint64_t awake_us = (elapsed > sleep_us) ? (elapsed - sleep_us) : 0;

    double avg_sleep_s = g_use_sleep_count
        ? (g_use_light_sleep_us / 1e6 / g_use_sleep_count) : 0.0;
    double avg_wake_int_s = g_use_wake_interval_count
        ? (g_use_wake_interval_total_us / 1e6 / g_use_wake_interval_count) : 0.0;
    double avg_burst_s = g_use_awake_burst_count
        ? (g_use_awake_burst_total_us / 1e6 / g_use_awake_burst_count) : 0.0;

    Serial.println("USAGE_START");
    Serial.printf("elapsed_s=%.1f\n", elapsed / 1e6);
    Serial.printf("light_sleep_s=%.1f (%.1f%%)\n",
                  g_use_light_sleep_us / 1e6, 100.0 * g_use_light_sleep_us / (double)elapsed);
    Serial.printf("awake_s=%.1f (%.1f%%)\n",
                  awake_us / 1e6, 100.0 * awake_us / (double)elapsed);
    Serial.printf("deep_sleep_s=%.1f (deep sleep is never used by this firmware)\n",
                  g_use_deep_sleep_us / 1e6);
    Serial.printf("sleeps=%lu  dormant=%lu  enforcement=%lu\n",
                  (unsigned long)g_use_sleep_count,
                  (unsigned long)g_use_dormant_sleep_count,
                  (unsigned long)g_use_enf_sleep_count);
    Serial.printf("avg_sleep_duration_s=%.2f  (heartbeat target ~%.1f)\n",
                  avg_sleep_s, DISCONNECTED_ADV_HEARTBEAT_MS / 1000.0);
    Serial.printf("avg_wakeup_interval_s=%.2f  (sleep+awake cycle period)\n", avg_wake_int_s);
    Serial.printf("avg_awake_burst_s=%.2f  max_awake_burst_s=%.2f\n",
                  avg_burst_s, g_use_awake_burst_max_us / 1e6);
    Serial.printf("wake_cause: timer=%lu gpio=%lu other=%lu\n",
                  (unsigned long)g_use_wake_timer,
                  (unsigned long)g_use_wake_gpio,
                  (unsigned long)g_use_wake_other);
    if (g_use_dormant_gpio_wake > 0)
        Serial.printf("WARNING: %lu DORMANT GPIO wakeups — motion is leaking through "
                      "as a wake source (should be 0 per spec 8.4)\n",
                      (unsigned long)g_use_dormant_gpio_wake);
    Serial.println("USAGE_END");
}
#endif // MEASURE_USAGE

// ============================================================
//  Light sleep
// ============================================================

static void enter_dormant_sleep() {
    Serial.println("[STATE] DORMANT_SLEEP");
#if DIAG_AWAKE
    if (g_dbg_burst_start_ms != 0)
        Serial.printf("[DIAG] DORMANT awake=%lums loops=%lu max_iter=%lums\n",
                      (unsigned long)(millis() - g_dbg_burst_start_ms),
                      (unsigned long)g_dbg_loop_count,
                      (unsigned long)g_dbg_max_iter_ms);
#endif
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

    // Motion is NOT a wake source in DORMANT (§8.4): outside an enforcement
    // window there is nothing for it to trigger, and waking on every wrist
    // movement was the single largest battery drain. Detach the edge ISR and mute
    // the INT1 GPIO wake entirely — DORMANT_SLEEP wakes on the timer (and BLE)
    // only. (The former look-to-wake wrist-raise wake source has been removed; the
    // clock stays lit on the ring instead of relying on a glance to light it.)
    detachInterrupt(digitalPinToInterrupt(LIS3DH_INT1));
    data_ready        = false;
    g_last_motion_ms  = 0;
    lis3dh_clear_int1();
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

    // Analog clock stays lit through DORMANT_SLEEP (§5.7.4): the SK6805 ring
    // latches whatever frame it was last sent, so paint the current time now and
    // it remains visible for the whole sleep. led_update() renders the clock
    // (minute-gated), so this refreshes it to the current minute before we sleep.
    led_update(led_status_input());

#if MEASURE_USAGE
    usage_before_sleep();
    uint64_t _use_t0 = esp_timer_get_time();
#endif
    esp_light_sleep_start();
#if MEASURE_USAGE
    usage_after_sleep(esp_timer_get_time() - _use_t0, /*is_enforcement=*/false);
#endif

    // After wakeup: clear the INT1 latch and re-arm the edge ISR for the
    // awake/enforce path (motion is actionable again once we may enter enforcement).
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
    // The analog clock is repainted by led_update() on the next loop pass (and
    // was refreshed just before sleep); no wake-cause flash in DORMANT — the clock
    // owns the ring so a still, disconnected watch always shows the time.
    batt_log("wake_from_sleep");
    g_activity_state   = STATE_DORMANT;
    g_last_activity_ms = millis();
    NimBLEDevice::getAdvertising()->start();
    recalculate_and_rearm();
#if DIAG_AWAKE
    // Start tracking the new awake burst that begins now.
    g_dbg_burst_start_ms = millis();
    g_dbg_last_loop_ms   = 0;
    g_dbg_loop_count     = 0;
    g_dbg_max_iter_ms    = 0;
#endif
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
    // Donning grace (§5.4.4): never sleep past the grace deadline, so the forced
    // re-check on expiry is not slept through.
    if (grace_active()) {
        uint64_t us_to_grace = (uint64_t)(g_grace_deadline - time(nullptr)) * 1000000ULL;
        if (us_to_grace < sleep_us) sleep_us = us_to_grace;
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

    // Ring off through enforcement sleep too (§5.7 priority 1).
    led_off();

#if MEASURE_USAGE
    usage_before_sleep();
    uint64_t _use_t0 = esp_timer_get_time();
#endif
    esp_light_sleep_start();
#if MEASURE_USAGE
    usage_after_sleep(esp_timer_get_time() - _use_t0, /*is_enforcement=*/true);
#endif

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
    led_note_wake(led_wake_cause_from_esp());  // momentary wake-cause flash (§5.7)
    batt_log("enf_wake_from_sleep");
    g_last_activity_ms = millis();
    NimBLEDevice::getAdvertising()->start();

    // Force an immediate condition re-check on the next loop iteration.
    g_last_enf_poll_ms = 0;

    // For anchor-based events, run a bounded BLE scan to refresh the device
    // cache, then the upcoming condition check will build a fresh ProxScanVector.
    if (g_active_event &&
        (g_active_event->criteria == STAY_NEAR || g_active_event->criteria == GET_AWAY ||
         g_active_event->criteria == PHONE_AWAY)) {
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

    // Donning grace (§5.4.4): donning starts a fresh grace period; removing the
    // watch during grace cancels it. Either way, force an immediate condition
    // re-check rather than waiting for the next poll boundary.
    if (g_activity_state == STATE_ENFORCEMENT && g_active_event) {
        if (is_worn) {
            if (g_active_event->donningGraceS > 0) {
                g_grace_deadline   = time(nullptr) + g_active_event->donningGraceS;
                g_grace_was_active = true;
                g_last_enf_poll_ms = 0;   // re-check on next loop iteration
                Serial.printf("[GRACE] Donned — grace %us\n", g_active_event->donningGraceS);
            }
        } else if (g_grace_deadline != 0) {
            g_grace_deadline   = 0;
            g_grace_was_active = false;
            g_last_enf_poll_ms = 0;       // re-check immediately
            Serial.println("[GRACE] Removed during grace — cancelled");
        }
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
    Serial.println("[WORN] Sampling IR reflection");
    g_last_worn_sample_ms = millis();

    int     reflection = ir_sample_reflection();
    Serial.println("[WORN] reflection=" + String(reflection));
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
        Serial.printf("[WORN] State changed to %s\n", g_worn ? "worn" : "removed");
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
        Serial.printf("[WIFI] Attempting connection to SSID: %s\n", ssid);
        Serial.printf("[WIFI] Password: %s\n", pass);
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
                // Reject unknown format versions (§6.2, lockstep): a v1 parser
                // would misread the version byte as the low byte of the count.
                bool version_ok = (g_sched_xfer_rcvd >= 1 &&
                                   g_sched_xfer_buf[0] == SCHEDULE_FORMAT_VERSION);
                if (crc_calc == crc_recv && version_ok) {
                    // Full §9.3 commitment-integrity diff gate: tightenings and
                    // new events apply immediately; loosenings are quarantined
                    // into the pending queue (24 h) unless a §9.3 exception
                    // applies. 0x01 = accepted in full, 0x03 = one or more
                    // changes quarantined (read …001A), 0x00 = malformed.
                    // (The Phase-1 0x04 full rejection is superseded: under the
                    // full gate active-event loosenings are quarantined, §6.2.)
                    resp = apply_schedule_diff_gate(g_sched_xfer_buf, g_sched_xfer_rcvd);
                }
                // else: CRC or version failure → resp stays 0x00 (§6.2)
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

        // §9.7 timezone guard: a tz change that would place the current local
        // time outside the CURRENTLY ACTIVE event's window is rejected with
        // 0x02 (the whole write is discarded — the app should retry without
        // the escaping tz).
        bool tz_changed = (new_tz != g_tz_offset_min);
        if (tz_changed &&
            time_write_would_escape_active_window((int64_t)time(nullptr), new_tz)) {
            Serial.println("[INTEG] Settings tz change escapes active window — rejected (0x02)");
            uint8_t resp = 0x02;
            pChar->setValue(&resp, 1);
            pChar->notify();
            g_last_activity_ms = millis();
            return;
        }

        // §9.8 settings gating: dormancy flags true→false and settle-window
        // increases are loosenings → quarantined (pending type 3); the reverse
        // directions bind harder and apply immediately. Non-gated fields in the
        // same write still apply; any quarantined field makes the response 0x03.
        bool quarantined = false;
        bool pending_changed = false;

        if (new_disconn != g_disconnected_is_dormant) {
            if (!new_disconn) {   // true→false = enforce less often = loosening
                queue_pending_setting(1, 0);
                quarantined = pending_changed = true;
                Serial.println("[INTEG] DISCONNECTED_IS_DORMANT false quarantined");
            } else {
                g_disconnected_is_dormant = true;
                pending_changed |= cancel_pending_for_setting(1);
            }
        }
        if (new_away != g_away_is_dormant) {
            if (!new_away) {
                queue_pending_setting(2, 0);
                quarantined = pending_changed = true;
                Serial.println("[INTEG] AWAY_IS_DORMANT false quarantined");
            } else {
                g_away_is_dormant = true;
                pending_changed |= cancel_pending_for_setting(2);
            }
        }

        // settle_window_min (uint16, v0.5, payload 4→6 bytes): clamp to
        // [30,240]; growing the free-edit window is a loosening → quarantined.
        if (val.size() >= 6) {
            uint16_t sw;
            memcpy(&sw, val.data() + 4, 2);
            if (sw < SETTLE_WINDOW_FLOOR_MIN) sw = SETTLE_WINDOW_FLOOR_MIN;
            if (sw > SETTLE_WINDOW_CEIL_MIN)  sw = SETTLE_WINDOW_CEIL_MIN;
            if (sw > g_settle_window_min) {
                queue_pending_setting(3, sw);
                quarantined = pending_changed = true;
                Serial.printf("[INTEG] settle_window_min %u quarantined\n", sw);
            } else if (sw != g_settle_window_min) {
                g_settle_window_min = sw;
                pending_changed |= cancel_pending_for_setting(3);
            }
        }

        g_tz_offset_min = new_tz;

        prefs.begin("watch", false);
        prefs.putBool("disc_dorm", g_disconnected_is_dormant);
        prefs.putBool("away_dorm", g_away_is_dormant);
        prefs.putShort("tz_off",   g_tz_offset_min);
        prefs.putUShort("settle_min", g_settle_window_min);
        prefs.end();

        if (pending_changed) {
            save_pending_queue();
            pending_publish();
        }

        if (tz_changed) {
            configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");
            recalculate_and_rearm();
        }

        uint8_t resp = quarantined ? 0x03 : 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

// Time characteristic (§5.6): the app sets the watch's absolute UTC clock over
// BLE, since NTP requires WiFi. Payload: [utc_epoch int64][tz_offset_minutes int16].
class WatchTimeCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        if (val.size() < 10) return;   // 8 (epoch) + 2 (tz)
        int64_t utc_epoch;
        memcpy(&utc_epoch, val.data(), 8);
        int16_t new_tz;
        memcpy(&new_tz, val.data() + 8, 2);

        // §9.7 time hardening: a clock write whose effect would end or skip the
        // CURRENTLY ACTIVE enforcement window (new local time outside the
        // active event's window) is rejected with 0x02. Integrity timers are
        // immune to clock changes regardless (elapsed basis, §9.2).
        if (time_write_would_escape_active_window(utc_epoch, new_tz)) {
            Serial.println("[INTEG] Time write escapes active window — rejected (0x02)");
            uint8_t resp = 0x02;
            pChar->setValue(&resp, 1);
            pChar->notify();
            g_last_activity_ms = millis();
            return;
        }

        g_tz_offset_min = new_tz;
        prefs.begin("watch", false);
        prefs.putShort("tz_off", g_tz_offset_min);
        prefs.end();

        // Apply the tz offset for localtime conversion. configTime also (re)starts
        // SNTP, which is harmless: it only overrides our clock if WiFi NTP later
        // succeeds (more accurate), and never does so on a WiFi-less watch.
        configTime((long)g_tz_offset_min * 60, 0, "pool.ntp.org");

        // Set the absolute UTC system clock.
        struct timeval tv;
        tv.tv_sec  = (time_t)utc_epoch;
        tv.tv_usec = 0;
        settimeofday(&tv, nullptr);

        Serial.printf("[TIME] Set via BLE: epoch=%lld tz=%d\n", (long long)utc_epoch, (int)new_tz);

        // Recompute today's schedule and re-arm RTC boundaries against the new time.
        recalculate_and_rearm();

        uint8_t resp = 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

// Pending Changes characteristic (§9.5, …001A): read returns the current
// quarantine queue with live countdowns; notifications fire on add/promote/
// cancel via pending_publish(). Rebuild on every read so the "seconds until
// apply" fields are current.
class WatchPendingCallback : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        uint8_t buf[1 + PENDING_QUEUE_MAX * 21];
        int len = pending_build_payload(buf);
        pChar->setValue(buf, len);
    }
};

// Emergency Pass characteristic (§9.6, …001B). The rolling pass ledger lives
// on the watch so clearing app data cannot replenish passes.
//   SPEND (0x01 + uuid16 + date u32 YYYYMMDD): if spends-in-window < allowance,
//   apply a one-day negate for that event/date IMMEDIATELY — deliberately
//   bypassing the §9.3 gate (the pass is the sanctioned escape valve) — and
//   respond 0x01 + remaining; else 0x02 + 0x00. 0x00 on malformed payload.
//   SET_ALLOWANCE (0x02 + u8): lowering applies immediately (0x01); raising is
//   a loosening → pending queue, type 3 (0x03).
// Read returns [allowance][remaining] + a regen countdown per spent pass.
// Response codes are conveyed by value + notify, matching the codebase's
// established with-response pattern.
class WatchPassCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        uint8_t resp[2] = { 0x00, 0x00 };
        int     resp_len = 1;

        if (!val.empty() && (uint8_t)val[0] == 0x01 && val.size() >= 21) {
            // ---- SPEND ----
            const uint8_t *uuid = (const uint8_t*)val.data() + 1;
            uint32_t date;
            memcpy(&date, val.data() + 17, 4);

            int spent     = pass_prune_and_count();
            int remaining = (int)g_pass_allowance - spent;
            if (remaining <= 0) {
                resp[0] = 0x02; resp[1] = 0x00; resp_len = 2;
                Serial.println("[PASS] Spend rejected — budget exhausted");
            } else {
                // Build the one-day negate: a once event carrying the target
                // event's UUID with negate=true, dated so date_int_of_utc()
                // lands on the given local day (noon avoids boundary drift).
                Event neg;
                memset(&neg, 0, sizeof(neg));
                memcpy(neg.id, uuid, 16);
                neg.recurrenceType = ONCE;
                neg.negate         = true;
                neg.referenceDate  = local_ymd_to_utc_epoch((int32_t)date, 720);

                Event *cur = g_all_events_scratch;
                int    nc  = 0;
                if (g_sched_blob && g_sched_blob_len >= 3)
                    deserialize_events(g_sched_blob, g_sched_blob_len, cur, &nc,
                                       MAX_EVENTS_PER_DAY * 2);
                int idx = find_event_index(cur, nc, neg.id, true, (int32_t)date);
                if (idx >= 0) cur[idx] = neg;
                else if (nc < MAX_EVENTS_PER_DAY * 2) cur[nc++] = neg;

                if (store_current_schedule(cur, nc)) {
                    recalculate_and_rearm();
                    // Record the spend (elapsed basis — clock writes can't age it).
                    for (int i = 0; i < PASS_MAX_STAMPS; i++) {
                        if (g_pass_spends[i] == 0) {
                            g_pass_spends[i] = integ_elapsed_now();
                            break;
                        }
                    }
                    save_pass_ledger();
                    resp[0] = 0x01; resp[1] = (uint8_t)(remaining - 1); resp_len = 2;
                    Serial.printf("[PASS] Spent on %02x… for %lu — %d remaining\n",
                                  uuid[0], (unsigned long)date, remaining - 1);
                }
                // store failure → resp stays 0x00
            }
        } else if (!val.empty() && (uint8_t)val[0] == 0x02 && val.size() >= 2) {
            // ---- SET_ALLOWANCE ----
            uint8_t na = (uint8_t)val[1];
            if (na <= g_pass_allowance) {
                g_pass_allowance = na;   // lowering (or unchanged) binds harder → immediate
                save_pass_ledger();
                if (cancel_pending_for_setting(4)) {
                    save_pending_queue();
                    pending_publish();
                }
                resp[0] = 0x01;
                Serial.printf("[PASS] Allowance set to %d\n", na);
            } else {
                queue_pending_setting(4, na);   // raising = loosening → quarantine (§9.6)
                save_pending_queue();
                pending_publish();
                resp[0] = 0x03;
                Serial.printf("[PASS] Allowance raise to %d quarantined\n", na);
            }
        }
        // else: malformed → 0x00

        pChar->setValue(resp, resp_len);
        pChar->notify();
        g_last_activity_ms = millis();
    }

    void onRead(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        uint8_t buf[2 + PASS_MAX_STAMPS * 4];
        int spent     = pass_prune_and_count();
        int remaining = (int)g_pass_allowance - spent;
        if (remaining < 0) remaining = 0;
        buf[0] = g_pass_allowance;
        buf[1] = (uint8_t)remaining;
        int pos = 2;
        uint32_t now_el = integ_elapsed_now();
        uint32_t win_s  = (uint32_t)PASS_WINDOW_DAYS * 86400UL;
        for (int i = 0; i < PASS_MAX_STAMPS; i++) {
            if (g_pass_spends[i] == 0) continue;
            uint32_t regen = g_pass_spends[i] + win_s;
            uint32_t secs  = (regen > now_el) ? (regen - now_el) : 0;
            memcpy(buf + pos, &secs, 4); pos += 4;
        }
        pChar->setValue(buf, pos);
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
        save_anchor_records();  // persist updated UUID→IP table
        uint8_t resp = 0x01;
        pChar->setValue(&resp, 1);
        pChar->notify();
        g_last_activity_ms = millis();
    }
};

// LED Configuration characteristic (§5.6): recolour/retune the status ring at
// runtime. Apply + persist the write, then refresh the readable value to the
// full live config so a subsequent Read reflects what was applied.
static NimBLECharacteristic *g_led_cfg_char = nullptr;

static void led_cfg_publish_value() {
    if (!g_led_cfg_char) return;
    uint8_t buf[2 + LED_SLOT_COUNT * 8];
    size_t n = led_serialize_config(buf, sizeof(buf));
    if (n) g_led_cfg_char->setValue(buf, n);
}

class WatchLedConfigCallback : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo&) override {
        std::string val = pChar->getValue();
        led_apply_config((const uint8_t*)val.data(), val.size());
        led_cfg_publish_value();  // Read now returns the live config
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

static void print_debug(const char *msg) { // prints message only if debug mode is on
    if (!DEBUG_MODE) return;
    Serial.println(msg);
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

    prox_init();

    // Hardware pins first so buzzer works for crash reporting
    pinMode(BUZZER_PIN, OUTPUT);
#if !DISABLE_MOTOR
    pinMode(VIBRO_PIN,  OUTPUT);  // skipped when disabled so GPIO 10 is left for the LED ring
#endif
    // ITR8307 reflective IR worn sensor
    pinMode(IR_EMIT, OUTPUT);
    digitalWrite(IR_EMIT, LOW);                       // emitter off until sampled
    pinMode(IR_REC, INPUT);
    analogSetPinAttenuation(IR_REC, ADC_11db);        // full-range phototransistor readings
    pinMode(BATT_ADC_PIN, INPUT);
    analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);  // effective range 0–2.45V; Vbat/2 max ~2.1V
    outputs_off();

    // LED status ring (§5.7). Loads persisted colours from NVS and clears the
    // ring. On this hardware rev the ring shares GPIO 10 with the (PCB-disabled)
    // vibration motor; see led_status.cpp. Motor code is left untouched.
    led_init();


    bp;


    // CHK 1: SPI / IMU init
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
    delay(100);
    prefs.begin("watch", true);
    g_is_paired       = prefs.getBool("paired", false);
    g_disconnected_is_dormant = prefs.getBool("disc_dorm", true);
    g_away_is_dormant         = prefs.getBool("away_dorm", true);
    g_tz_offset_min           = prefs.getShort("tz_off", 0);
    g_settle_window_min       = prefs.getUShort("settle_min", SETTLE_WINDOW_MIN_DEFAULT);
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
    bp;

    load_wifi_creds();
    bp;

    load_schedule_blob();
    bp;

    // Commitment integrity (§9): start the elapsed-time accumulator, load the
    // settle records / pending queue / pass ledger, and bootstrap settle state
    // for any events that predate tracking. Must follow load_schedule_blob().
    integrity_init();
    bp;

    // Restore the known-anchor table (UUID→MAC/IP) so proximity can connect on
    // boot without re-discovering each anchor's MAC. Must precede the schedule
    // recalc, whose ensure_anchor_record() dedups against these by UUID.
    load_anchor_records();
    bp;

    batt_log_load();
    bp;

    g_activity_state = g_is_paired ? STATE_DORMANT : STATE_UNPAIRED;

    
    bp;

    // CHK 4: WiFi mode
    bp;
    delay(200);
    bp;
    WiFi.mode(WIFI_STA);
    


    bp; 

    // CHK 4b: WiFi connect
    if (g_wifi_cred_count > 0) try_connect_saved_wifi();


    bp;

    // CHK 5: NimBLE init
    delay(500);
    NimBLEDevice::init(g_watch_uuid_str);
    // Boost BLE TX power to extend usable connection range. Connection
    // establishment fails at much weaker RSSI than advertisement reception, so
    // the default (~0 dBm) leaves the proximity query failing while ads are
    // still heard. (Boost the anchor's TX power the same way for full benefit.)
    NimBLEDevice::setPower(9);  // +9 dBm (max on ESP32-C3)
    NimBLEDevice::setDeviceName("Impulse Watch");

    bp;


    // CHK 6: GATT server + service
    g_ble_server = NimBLEDevice::createServer();
    g_ble_server->setCallbacks(new WatchServerCallbacks());
    NimBLEService *svc = g_ble_server->createService(WATCH_SERVICE_UUID);


    bp; 
    
    // CHK 7: GATT characteristics
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

    g_led_cfg_char = svc->createCharacteristic(WATCH_LED_CONFIG_CHAR_UUID,
                                               NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
    g_led_cfg_char->setCallbacks(new WatchLedConfigCallback());
    led_cfg_publish_value();  // seed the readable value with the current config

    auto *timeChar = svc->createCharacteristic(WATCH_TIME_CHAR_UUID,
                                               NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    timeChar->setCallbacks(new WatchTimeCallback());

    // Pending Changes (§9.5, …001A): authoritative quarantine queue.
    g_pending_char = svc->createCharacteristic(WATCH_PENDING_CHAR_UUID,
                                               NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    g_pending_char->setCallbacks(new WatchPendingCallback());
    {   // Seed the readable value with the current (loaded) queue.
        uint8_t pbuf[1 + PENDING_QUEUE_MAX * 21];
        int plen = pending_build_payload(pbuf);
        g_pending_char->setValue(pbuf, plen);
    }

    // Emergency Pass (§9.6, …001B): on-watch rolling pass ledger.
    g_pass_char = svc->createCharacteristic(WATCH_PASS_CHAR_UUID,
                                            NIMBLE_PROPERTY::READ |
                                            NIMBLE_PROPERTY::WRITE |
                                            NIMBLE_PROPERTY::NOTIFY);
    g_pass_char->setCallbacks(new WatchPassCallback());

    bp;
    // CHK 8: start service + advertising
    svc->start();
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("Impulse Watch");
    pAdv->addServiceUUID(WATCH_SERVICE_UUID);
    pAdv->start();
    Serial.println("[BLE] Watch advertising started");

    // CHK 9: BLE scanner setup
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
#if MEASURE_USAGE
    usage_init();
    usage_report();
#endif
    // indicate_successful_boot();
    delay(1000);
#if !DISABLE_MOTOR
    set_motor(true);
    delay(500);
    set_motor(false);
#endif
    // Wait briefly for the IMU's first interrupt as a liveness check, but never
    // block boot on it — a still watch produces no motion, and an unbounded wait
    // here pins the CPU fully awake until the watch is disturbed (this was the
    // ~60 s boot awake-burst seen in MEASURE_USAGE). Time out and continue; the
    // motion ISR works normally once we're in loop().
    {
        uint32_t t0 = millis();
        while (!data_ready && millis() - t0 < IMU_FIRST_INT_TIMEOUT_MS) delay(10);
        if (!data_ready)
            Serial.println("[SYS] No IMU interrupt within timeout — continuing boot");
    }

    // Boot flourish on the LED ring, then paint the initial status (blue
    // dormant, orange enforcing, or off if unpaired) so the ring reflects state
    // immediately after boot. Done after the motor pulse above, which briefly
    // drives the shared GPIO 10.
    led_startup_animation();
    led_update(led_status_input());
#if DIAG_AWAKE
    g_dbg_burst_start_ms = millis();
#endif
}

// ============================================================
//  loop()
// ============================================================

// Midnight tracking
static int g_last_mday = -1;

void loop() {
    uint32_t now_ms  = millis();
    uint32_t now_min = local_minutes_now();

#if DIAG_AWAKE
    // Per-iteration timing: a healthy loop turns over in ~10–20 ms (the trailing
    // delay(10) plus light work). A much longer gap means a blocking call ran
    // this iteration — the prime suspect for unexpected awake time / battery.
    {
        uint32_t iter = (g_dbg_last_loop_ms == 0) ? 0 : (now_ms - g_dbg_last_loop_ms);
        g_dbg_last_loop_ms = now_ms;
        if (iter > g_dbg_max_iter_ms) g_dbg_max_iter_ms = iter;
        g_dbg_loop_count++;
        if (iter > 150)
            Serial.printf("[DIAG] slow loop iter=%lums state=%d (a blocking call ran)\n",
                          (unsigned long)iter, (int)g_activity_state);
    }
#endif



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
    // Use a NON-BLOCKING local-time read here. getLocalTime() blocks for up to
    // 5 s while the RTC is unset (no WiFi/NTP sync yet) — and being on the hot
    // loop path it stalled EVERY iteration, pinning the CPU awake for seconds
    // each heartbeat (a major battery drain, unrelated to the look feature).
    // local_now() is time()-based and returns immediately; we just gate on a
    // plausibly-set clock (year > 2016) so an unsynced clock is a no-op.
    {
        time_t lt = local_now();
        struct tm ti;
        gmtime_r(&lt, &ti);
        if (ti.tm_year > (2016 - 1900) && ti.tm_mday != g_last_mday) {
            if (g_last_mday != -1) {
                Serial.println("[SCHED] Midnight — recalculating");
                recalculate_and_rearm();
            }
            g_last_mday = ti.tm_mday;
        }
    }

    if (g_activity_state == STATE_UNPAIRED) {
        led_update(led_status_input());  // ring stays off until paired (§5.7)
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

    // ---- WiFi connect detector (async reconnect completed) ----
    // The DORMANT reconnect path (below) kicks off WiFi.begin() without blocking;
    // the association lands here on a later loop iteration once WL_CONNECTED.
    if (!g_wifi_connected && WiFi.status() == WL_CONNECTED) {
        on_wifi_associated(WiFi.SSID().c_str());
        recalculate_and_rearm();
        push_watch_status();
        g_last_activity_ms = now_ms;
    }

    // ---- WiFi scan / reconnect (DORMANT) ----
    if (g_activity_state == STATE_DORMANT) {
        if (!g_wifi_connected &&
            (now_ms - g_last_wifi_retry_ms) > (uint32_t)ANCHOR_WIFI_RETRY_INTERVAL_S * 1000UL) {
            g_last_wifi_retry_ms = now_ms;
            // Non-blocking: success is promoted by the connect detector above, so
            // the heartbeat wake is not stalled busy-waiting on association.
            try_connect_saved_wifi_async();
        }
        // Periodic WiFi scan  (for SSID discovery — triggers reconnect)
        if ((now_ms - g_last_wifi_scan_ms) > (uint32_t)WIFI_SCAN_INTERVAL_S * 1000UL) {
            g_last_wifi_scan_ms = now_ms;
            if (!g_wifi_connected) try_connect_saved_wifi_async();
        }
    }

    // ---- BLE scan for anchors ----
    if (g_activity_state == STATE_DORMANT) {
        // DORMANT: scan on a fixed interval for anchor discovery and RSSI updates.
        if (!g_ble_scan->isScanning() &&
            ((now_ms - g_last_ble_scan_ms) > (uint32_t)BLE_SCAN_INTERVAL_S * 1000UL || g_last_ble_scan_ms == 0)) {
            // Do NOT reset g_last_activity_ms here: the scan runs async and the
            // DORMANT→SLEEP gate already blocks sleep while a scan is in flight,
            // so the watch stays awake exactly for the ~300 ms scan and then
            // sleeps — instead of being pinned awake for a full idle dwell after.
            start_ble_scan();
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
        // Donning grace expiry (§5.4.4): the instant grace ends, force an
        // immediate condition re-check rather than waiting for the poll boundary.
        if (g_grace_was_active && !grace_active()) {
            g_grace_was_active = false;
            g_grace_deadline   = 0;
            g_last_enf_poll_ms = 0;
            Serial.println("[GRACE] Expired — forcing condition re-check");
        }
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
        !(g_ble_scan && g_ble_scan->isScanning()) &&
        (now_ms - g_last_activity_ms) > DORMANT_TO_SLEEP_IDLE_MS) {
        // Motion no longer blocks DORMANT sleep — it is not a wake source here
        // and is ignored above (§8.4), so the idle timer alone governs sleep.
        // An in-flight discovery scan does block sleep so it can finish (§ above).
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
#if MEASURE_USAGE
        else if (cmd == 0xAD) {
            usage_report();
        } else if (cmd == 0xAE) {
            usage_reset();
            Serial.println("USAGE_RESET");
        }
#endif
    }

    // ---- Commitment-integrity tick (§9.2/§9.4) ----
    // Settle promotion + autonomous pending promotion. The loop runs after
    // every wake (boot, RTC boundary, midnight, motion, BLE), so this covers
    // all §9.4 promotion points; a sleeping watch promotes late, never early.
    {
        static uint32_t s_last_integ_tick_ms = 0;
        if (now_ms - s_last_integ_tick_ms >= 1000) {
            s_last_integ_tick_ms = now_ms;
            integrity_tick();
        }
    }

    // ---- Battery heartbeat (periodic voltage sample) ----
    if (now_ms - g_last_batt_heartbeat_ms >= BATT_HEARTBEAT_INTERVAL_MS) {
        g_last_batt_heartbeat_ms = now_ms;
        batt_log("heartbeat");
        integ_save_elapsed();   // §9.2 elapsed accumulator: same cadence as usage_save
#if MEASURE_USAGE
        usage_report();
        usage_save();   // persist snapshot (single fixed NVS key, overwritten in place)
#endif
    }

    // ---- LED status tick (§5.7) ----
    // Resolve the current slot, advance the alarm blink, and expire any wake
    // flash. Output-only; never affects enforcement/sleep/radio behaviour.
    led_update(led_status_input());

    delay(10);
}
