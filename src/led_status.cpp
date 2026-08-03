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
// While an escalation profile is between bursts (buzzer and motor both idle) the
// ring holds a dim steady alarm colour instead of blinking: light and sound rest
// together, but enforcement is still visibly in progress.
#define LED_ALARM_QUIET_LEVEL        48    // of 255
// ENFORCE_GRACE bar: at least this many LEDs are lit the moment the tolerance
// window opens, so a bar at 0% still reads as "lit and counting" rather than as
// a ring that has gone dark.
#define LED_GRACE_MIN_LIT            1

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

// DORMANT analog-clock bookkeeping. The clock is repainted only when the shown
// minute changes (once per minute), or when another state (alarm, flash, off)
// clobbered the ring and it must be redrawn on return.
static int  s_last_clock_min = -1;
static bool s_clock_dirty    = true;

// Last rendered frame, to avoid redundant FastLED.show() calls each tick.
static CRGB    s_last_color  = CRGB::Black;
static uint8_t s_last_bright = 0;
static bool    s_have_last   = false;
// Same idea for the grace progress bar, which is not a solid frame: how many
// LEDs the last painted bar lit, or -1 if the ring currently holds something
// else. Kept mutually exclusive with s_have_last so exactly one cache can ever
// claim the ring.
static int     s_last_grace_lit = -1;

// ── Config defaults / persistence ───────────────────────────
static void led_set_defaults() {
    s_cfg.brightness = LED_RING_DEFAULT_BRIGHTNESS;
    s_cfg.slots[LED_SLOT_DORMANT]       = {  0,   0, 255, 0, 0};                                  // blue
    s_cfg.slots[LED_SLOT_ENFORCE_IDLE]  = {255,  90,   0, 0, 0};                                  // orange
    s_cfg.slots[LED_SLOT_ENFORCE_ALARM] = {255,   0,   0, LED_ALARM_BLINK_ON_MS, LED_ALARM_BLINK_OFF_MS}; // red, blinking
    s_cfg.slots[LED_SLOT_WAKE_MOTION]   = {  0, 255,   0, 0, 0};                                  // green
    s_cfg.slots[LED_SLOT_WAKE_TIMER]    = {  0, 180, 255, 0, 0};                                  // light blue
    s_cfg.slots[LED_SLOT_WAKE_BLE]      = {255, 255, 255, 0, 0};                                  // white
    s_cfg.slots[LED_SLOT_ENFORCE_GRACE] = {255, 200,   0, 0, 0};                                  // yellow, progress bar (timings unused)
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
    s_last_color     = c;
    s_last_bright    = s_cfg.brightness;
    s_have_last      = true;
    s_last_grace_lit = -1;   // the ring no longer holds a bar
}

// Paint `lit` of the 12 LEDs as a filling arc, the rest dark. Fills clockwise
// from 12 o'clock using the same index mapping as led_show_time (index 11 is the
// top of the ring and the indices run backwards), so the bar sweeps the way the
// clock hands do rather than against them.
static void led_show_progress(const CRGB &c, int lit) {
    if (lit < LED_GRACE_MIN_LIT) lit = LED_GRACE_MIN_LIT;
    if (lit > LED_RING_COUNT)    lit = LED_RING_COUNT;
    if (s_last_grace_lit == lit && c == s_last_color && s_cfg.brightness == s_last_bright)
        return;

    fill_solid(s_leds, LED_RING_COUNT, CRGB::Black);
    for (int k = 0; k < lit; k++) s_leds[LED_RING_COUNT - 1 - k] = c;
    FastLED.setBrightness(s_cfg.brightness);
    FastLED.show();

    s_last_grace_lit = lit;
    s_last_color     = c;
    s_last_bright    = s_cfg.brightness;
    // The ring holds a multi-colour frame, so the solid-colour cache is stale:
    // the next led_show_color must repaint even if it asks for the same colour.
    s_have_last      = false;
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
    s_last_color     = CRGB::Black;
    s_last_bright    = s_cfg.brightness;
    s_have_last      = true;
    s_last_grace_lit = -1;
    s_inited         = true;
}

void led_off() {
    if (!s_inited) return;
    led_show_color(CRGB::Black);
}
void led_actual_off() {
    if (!s_inited) return;
    FastLED.clear(true);
    s_last_color     = CRGB::Black;
    s_last_bright    = s_cfg.brightness;
    s_have_last      = true;
    s_last_grace_lit = -1;
}

void led_show_time(int hour, int minute) {
    // Analog clock on the 12-LED ring: hour hand red, minute hand green, yellow
    // when they overlap. No second hand — the ring holds this frame through sleep
    // and is repainted only when the minute changes (§5.7.4).
    if (!s_inited) return;

    fill_solid(s_leds, LED_RING_COUNT, CRGB::Black);
    int hour_pos = 11 - (hour % 12);
    int min_pos  = 11 - (minute / 5);
    if (hour_pos == min_pos) {
        s_leds[hour_pos] = CRGB::Yellow;
    } else {
        s_leds[hour_pos] = CRGB::Red;
        s_leds[min_pos]  = CRGB::Green;
    }
    FastLED.setBrightness(s_cfg.brightness);
    FastLED.show();

    // The ring now holds an explicit multi-colour frame rather than a single solid
    // colour, so invalidate the solid-colour de-dup cache: the next led_show_color
    // (a flash, an alarm, or the enforcement off-before-sleep) must repaint.
    s_have_last      = false;
    s_last_grace_lit = -1;
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
    s_last_color     = CRGB::Black;
    s_last_bright    = saved_bright;
    s_have_last      = true;
    s_last_grace_lit = -1;
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
    if (in.unpaired) { s_clock_dirty = true; led_show_color(CRGB::Black); return; }

    // Priority 2: active enforcement alarm (condition not met) → blink the alarm
    // colour. Overrides the wake flash so the visual alarm tracks the buzzer.
    //
    // The blink is locked to the escalation profile's output envelope rather than
    // free-running on millis(). Free-running, the ring and the buzzer drifted
    // against each other: a 2 s buzz could begin mid-blink and the ring kept
    // flashing through the 30 s silence, so the two read as unrelated alarms.
    // Locked, every burst starts on a rising red edge and the ring goes quiet
    // when the sound does.
    if (in.enforcing && !in.condition_met) {
        s_clock_dirty = true;
        const LedSlotCfg &a = s_cfg.slots[LED_SLOT_ENFORCE_ALARM];
        CRGB col(a.r, a.g, a.b);
        if (!in.output_active) {
            // Between bursts: hold a dim steady alarm colour. Not black — a dark
            // ring during a 30 s gap is indistinguishable from "enforcement
            // stopped", which is the opposite of what is happening.
            col.nscale8_video(LED_ALARM_QUIET_LEVEL);
        } else if (a.on_ms > 0 && a.off_ms > 0) {
            uint32_t period = (uint32_t)a.on_ms + a.off_ms;
            // Phase from the output's own rising edge, so the first ON coincides
            // with the buzzer's. Unsigned subtraction is millis()-rollover safe.
            uint32_t phase  = (now - in.output_since_ms) % period;
            if (phase >= a.on_ms) col = CRGB::Black;
        }
        led_show_color(col);
        return;
    }

    // Priority 2b: grace/tolerance countdown. The criterion is FAILING right now
    // and the only thing keeping the buzzer off is a tolerance window that is
    // running out (phoneAway's PHONE_AWAY_TOLERANCE_S — "a quick check is fine").
    // Ranked directly below the alarm and above the wake flash, because it is the
    // one moment where a glance at the wrist changes what the user does next.
    //
    // Rendered as a filling arc, not a blink or a pulse. The ring has 12 LEDs and
    // the window is finite, so it can say *how much* time is left rather than
    // just "something is about to happen": one more LED lights every
    // window/12 seconds (~5 s for phoneAway's 60 s), and a full ring means the
    // alarm is next. Nothing about it flashes, so it cannot be confused with the
    // 250 ms alarm blink directly above.
    if (in.enforcing && in.condition_met && in.in_grace) {
        s_clock_dirty = true;
        const LedSlotCfg &g = s_cfg.slots[LED_SLOT_ENFORCE_GRACE];
        // 0 → MIN_LIT, 255 → the full ring.
        int lit = LED_GRACE_MIN_LIT +
                  ((int)in.grace_progress_u8 * (LED_RING_COUNT - LED_GRACE_MIN_LIT)) / 255;
        led_show_progress(CRGB(g.r, g.g, g.b), lit);
        return;
    }

    // Priority 3: momentary wake-cause flash (enforcement wakes only — DORMANT no
    // longer flashes on wake; it just shows the clock).
    if (s_flash_cause != LED_WAKE_NONE) {
        s_clock_dirty = true;
        led_show_color(slot_color(slot_for_wake(s_flash_cause)));
        return;
    }

    // Enforcement, condition met: no dedicated output (retain prior behaviour).
    if (in.enforcing) { s_clock_dirty = true; return; }

    // Priority 4: DORMANT (paired) → keep the analog clock lit, always on. Repaint
    // only when the shown minute changes, or after another state clobbered the
    // ring, so the display refreshes once per minute rather than every loop.
    if (s_clock_dirty || in.minute != s_last_clock_min) {
        led_show_time(in.hour, in.minute);
        s_last_clock_min = in.minute;
        s_clock_dirty    = false;
    }
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
