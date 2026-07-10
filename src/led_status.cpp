// ============================================================
//  LED Status Indicator (Watch) — firmware_spec_v2.md §5.7
// ============================================================
#include "led_status.h"

#include <Arduino.h>
#include <FastLED.h>
#include <Preferences.h>
#include <string.h>

// ── Hardware / timing constants (§7) ────────────────────────
// Data line for the SK6805 ring. Current hardware rev: GPIO 10, shared with the
// vibration motor, which is disabled by a PCB switch — so the ring is the only
// thing physically driven on GPIO 10 right now. The next rev moves the ring to
// GPIO 7 and restores the motor on GPIO 10; ONLY this constant changes. This
// module never touches the motor driver (set_motor lives in main.cpp).
#define LED_RING_DATA_PIN            10
#define LED_RING_COUNT               12
#define LED_RING_DEFAULT_BRIGHTNESS  40
#define LED_WAKE_FLASH_MS            600   // momentary wake-cause flash duration
#define LED_ALARM_BLINK_ON_MS        250   // default ENFORCE_ALARM blink on-time
#define LED_ALARM_BLINK_OFF_MS       250   // default ENFORCE_ALARM blink off-time

// ── Persisted configuration ─────────────────────────────────
// Packed so the NVS blob size is deterministic (1 + 6*7 = 43 bytes) and matches
// across builds. Kept private to this translation unit; the wire format used by
// the app (8 bytes/slot, with the slot id) is handled in (de)serialisation.
struct __attribute__((packed)) LedSlotCfg {
    uint8_t  r, g, b;
    uint16_t on_ms, off_ms;   // blink; on_ms==0 → steady
};
struct __attribute__((packed)) LedConfig {
    uint8_t    brightness;
    LedSlotCfg slots[LED_SLOT_COUNT];
};

static CRGB      s_leds[LED_RING_COUNT];
static LedConfig s_cfg;
static bool      s_inited = false;

// Wake-cause flash bookkeeping.
static LedWakeCause s_flash_cause    = LED_WAKE_NONE;
static uint32_t     s_flash_start_ms = 0;

// Last rendered frame, to avoid redundant FastLED.show() calls each tick.
static CRGB    s_last_color  = CRGB::Black;
static uint8_t s_last_bright = 0;
static bool    s_have_last   = false;

// ── Config defaults / persistence ───────────────────────────
static void led_set_defaults() {
    s_cfg.brightness = LED_RING_DEFAULT_BRIGHTNESS;
    s_cfg.slots[LED_SLOT_DORMANT]       = {  0,   0, 255, 0, 0};                                  // blue
    s_cfg.slots[LED_SLOT_ENFORCE_IDLE]  = {255,  90,   0, 0, 0};                                  // orange
    s_cfg.slots[LED_SLOT_ENFORCE_ALARM] = {255,   0,   0, LED_ALARM_BLINK_ON_MS, LED_ALARM_BLINK_OFF_MS}; // red, blinking
    s_cfg.slots[LED_SLOT_WAKE_MOTION]   = {  0, 255,   0, 0, 0};                                  // green
    s_cfg.slots[LED_SLOT_WAKE_TIMER]    = {  0, 180, 255, 0, 0};                                  // light blue
    s_cfg.slots[LED_SLOT_WAKE_BLE]      = {255, 255, 255, 0, 0};                                  // white
}

void led_load_config() {
    led_set_defaults();
    Preferences p;
    if (p.begin("led", true)) {
        if (p.isKey("cfg")) {
            LedConfig tmp;
            if (p.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp)) s_cfg = tmp;
        }
        p.end();
    }
}

static void led_persist_config() {
    Preferences p;
    if (p.begin("led", false)) {
        p.putBytes("cfg", &s_cfg, sizeof(s_cfg));
        p.end();
    }
}

// ── Rendering ───────────────────────────────────────────────
static void led_show_color(const CRGB &c) {
    if (s_have_last && c == s_last_color && s_cfg.brightness == s_last_bright) return;
    FastLED.setBrightness(s_cfg.brightness);
    fill_solid(s_leds, LED_RING_COUNT, c);
    FastLED.show();
    s_last_color  = c;
    s_last_bright = s_cfg.brightness;
    s_have_last   = true;
}

static CRGB slot_color(LedStatusSlot s) {
    const LedSlotCfg &c = s_cfg.slots[s];
    return CRGB(c.r, c.g, c.b);
}

static LedStatusSlot slot_for_wake(LedWakeCause c) {
    switch (c) {
        case LED_WAKE_MOTION: return LED_SLOT_WAKE_MOTION;
        case LED_WAKE_TIMER:  return LED_SLOT_WAKE_TIMER;
        case LED_WAKE_BLE:    return LED_SLOT_WAKE_BLE;
        default:              return LED_SLOT_DORMANT;
    }
}

void led_init() {
    led_load_config();
    FastLED.addLeds<WS2812B, LED_RING_DATA_PIN, GRB>(s_leds, LED_RING_COUNT);
    FastLED.setBrightness(s_cfg.brightness);
    FastLED.clear(true);
    s_last_color  = CRGB::Black;
    s_last_bright = s_cfg.brightness;
    s_have_last   = true;
    s_inited      = true;
}

void led_off() {
    if (!s_inited) return;
    led_show_color(CRGB::Black);
}
void led_actual_off() {
    if (!s_inited) return;
    FastLED.clear(true);
    s_last_color  = CRGB::Black;
    s_last_bright = s_cfg.brightness;
    s_have_last   = true;
}

void led_show_time(int hour, int minute, int second) {
 // makes an analog clock out of the 12 leds, with the hour hand in red and the minute hand in green
    
    if (!s_inited) return;
    CRGB c[LED_RING_COUNT];
    fill_solid(c, LED_RING_COUNT, CRGB::Black);
    if (11 - (hour % 12) == 11 - (minute / 5)) {
        // hour and minute hands overlap, make it yellow
        c[11 - (hour % 12)] = CRGB::Yellow;
    } else {
        c[11 - (hour % 12)] = CRGB::Red;
        c[11 - (minute / 5)] = CRGB::Green;
    }
    FastLED.setBrightness(s_cfg.brightness);
    memcpy(s_leds, c, sizeof(c));
    FastLED.show();
    s_last_color  = CRGB::Black;  // force a repaint on the next tick
    s_last_bright = s_cfg.brightness;
    s_have_last   = true;
    // show the second hand in blue, updating it each second for 5 seconds
    for (int i = 0; i < 25; i++) {
       fill_solid(s_leds, LED_RING_COUNT, CRGB::Black);
        float percentage_through_minute = (float)second / 60.0f;
        // in the case which the second hand is between two leds, we will light up both at different brightness levels to simulate the second hand being in between two leds
        double led_float_index = percentage_through_minute * LED_RING_COUNT;
        double percent_high = ceil(led_float_index) - led_float_index;
        double percent_low = 1.0 - percent_high;
        int led_index_high = (int)ceil(led_float_index) % LED_RING_COUNT;
        int led_index_low = (int)floor(led_float_index) % LED_RING_COUNT
;       
        led_index_high = 11 - led_index_high;  // invert the index to match the physical layout of the leds
        led_index_low = 11 - led_index_low;    // invert the index to match

        // make the ceil of the index percent_high brightness blue, make the floor of the index percent_low brightness blue
        s_leds[led_index_high] = CRGB::Blue;
        s_leds[led_index_low] = CRGB::Blue;
        
        if (led_index_high == led_index_low) {
            s_leds[led_index_high] = CRGB::Blue;
       
        } else {
            s_leds[led_index_high].fadeLightBy((uint8_t)(255 * percent_high));
            s_leds[led_index_low].fadeLightBy((uint8_t)(255 * percent_low));
        }
 
         if (11 - (hour % 12) == 11 - (minute / 5)) {
            // hour and minute hands overlap, make it yellow
            s_leds[11 - (hour % 12)] = CRGB::Yellow;
        } else {
            s_leds[11 - (hour % 12)] = CRGB::Red;
            s_leds[11 - (minute / 5)] = CRGB::Green;
        }

        FastLED.setBrightness(s_cfg.brightness);

        FastLED.show();
   
        delay(1000);
        second++;
            
    }
    
    led_actual_off();
 
}
// Blocking boot animation (§5.7 is silent on the visuals — this is a flourish).
// Three acts: a rainbow comet that laps the ring and accelerates, a short
// rainbow shimmer, then a white flash that fades out. Leaves the ring cleared
// and the de-dup state consistent so the next led_update() repaints normally.
void led_startup_animation() {
    if (!s_inited) return;
    const uint8_t saved_bright = s_cfg.brightness;
    FastLED.setBrightness(64);  // a touch brighter just for the show

    // Act 1 — rainbow comet: a hue-cycling head with a fading trail spins around
    // the ring a few times, speeding up each lap.
    int frame_ms = 38;
    for (int lap = 0; lap < 3; lap++) {
        for (int i = 0; i < LED_RING_COUNT; i++) {
            fadeToBlackBy(s_leds, LED_RING_COUNT, 90);            // decay the trail
            uint8_t hue = (uint8_t)((lap * LED_RING_COUNT + i) * (256 / LED_RING_COUNT));
            s_leds[i] = CHSV(hue, 255, 255);
            FastLED.show();
            delay(frame_ms);
        }
        if (frame_ms > 16) frame_ms -= 10;
    }

    // Act 2 — rainbow shimmer: a full-ring rainbow that rotates briefly.
    for (int t = 0; t < LED_RING_COUNT * 2; t++) {
        fill_rainbow(s_leds, LED_RING_COUNT, (uint8_t)(t * 8), 256 / LED_RING_COUNT);
        FastLED.show();
        delay(22);
    }

    // Act 3 — white flash, then fade to black and hand the ring back.
    fill_solid(s_leds, LED_RING_COUNT, CRGB::White);
    FastLED.show();
    delay(60);
    for (int k = 0; k < 16; k++) {
        fadeToBlackBy(s_leds, LED_RING_COUNT, 40);
        FastLED.show();
        delay(16);
    }

    FastLED.clear(true);
    FastLED.setBrightness(saved_bright);
    s_last_color  = CRGB::Black;
    s_last_bright = saved_bright;
    s_have_last   = true;
}

void led_note_wake(LedWakeCause cause) {
    if (cause == LED_WAKE_NONE) return;
    s_flash_cause    = cause;
    s_flash_start_ms = millis();
}

void led_update(const LedStatusInput &in) {
    if (!s_inited) return;
    uint32_t now = millis();

    // Expire the wake-cause flash.
    if (s_flash_cause != LED_WAKE_NONE && (now - s_flash_start_ms) >= LED_WAKE_FLASH_MS)
        s_flash_cause = LED_WAKE_NONE;

    // Priority 1: unpaired → ring off (no status to show until paired).
    if (in.unpaired) { led_show_color(CRGB::Black); return; }

    // Priority 2: active enforcement alarm (condition not met) → blink the alarm
    // colour. Overrides the wake flash so the visual alarm tracks the buzzer.
    if (in.enforcing && !in.condition_met) {
        const LedSlotCfg &a = s_cfg.slots[LED_SLOT_ENFORCE_ALARM];
        CRGB col(a.r, a.g, a.b);
        if (a.on_ms > 0 && a.off_ms > 0) {
            uint32_t period = (uint32_t)a.on_ms + a.off_ms;
            if ((now % period) >= a.on_ms) col = CRGB::Black;
        }
        led_show_color(col);
        return;
    }

    // Priority 3: momentary wake-cause flash.
    if (s_flash_cause != LED_WAKE_NONE) {
        led_show_color(slot_color(slot_for_wake(s_flash_cause)));
        return;
    }

    // Priority 4: steady activity colour.
    // led_show_color(slot_color(in.enforcing ? LED_SLOT_ENFORCE_IDLE : LED_SLOT_DORMANT));
}

// ── App configuration (de)serialisation (§5.6) ──────────────
// Wire layout: [brightness][N] then N × [slot_id, R, G, B, on_lo, on_hi, off_lo,
// off_hi]. A write may carry any subset of slots; unlisted slots keep their
// values. Unknown slot ids are skipped.
bool led_apply_config(const uint8_t *data, size_t len) {
    if (len < 2) return false;
    uint8_t brightness = data[0];
    uint8_t n          = data[1];
    if (len < (size_t)2 + (size_t)n * 8) return false;

    s_cfg.brightness = brightness;
    const uint8_t *p = data + 2;
    for (uint8_t i = 0; i < n; i++, p += 8) {
        uint8_t slot = p[0];
        if (slot >= LED_SLOT_COUNT) continue;
        LedSlotCfg &c = s_cfg.slots[slot];
        c.r = p[1]; c.g = p[2]; c.b = p[3];
        c.on_ms  = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
        c.off_ms = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
    }

    led_persist_config();
    FastLED.setBrightness(s_cfg.brightness);
    s_have_last = false;  // force a repaint on the next tick
    return true;
}

size_t led_serialize_config(uint8_t *out, size_t cap) {
    const size_t need = 2 + (size_t)LED_SLOT_COUNT * 8;
    if (cap < need) return 0;
    out[0] = s_cfg.brightness;
    out[1] = LED_SLOT_COUNT;
    uint8_t *p = out + 2;
    for (uint8_t i = 0; i < LED_SLOT_COUNT; i++, p += 8) {
        const LedSlotCfg &c = s_cfg.slots[i];
        p[0] = i;
        p[1] = c.r; p[2] = c.g; p[3] = c.b;
        p[4] = (uint8_t)(c.on_ms  & 0xFF); p[5] = (uint8_t)(c.on_ms  >> 8);
        p[6] = (uint8_t)(c.off_ms & 0xFF); p[7] = (uint8_t)(c.off_ms >> 8);
    }
    return need;
}
