// ============================================================
//  LED Status Indicator (Watch) — firmware_spec_v2.md §5.7
// ============================================================
// Drives the 12-LED SK6805 ring as a passive status indicator. This module is
// output-only: it never changes enforcement, sleep, scanning, or radio
// behaviour. All colours/blink timings are runtime-configurable by the app via
// WATCH_LED_CONFIG_CHAR_UUID and persisted to NVS; see led_apply_config().
//
// FastLED is contained entirely in led_status.cpp so the rest of the firmware
// (and its own CRGB-free world) never has to include it.
#pragma once

#include <stddef.h>
#include <stdint.h>

// One configurable status per "slot". Values are the on-wire slot ids used by
// the app's LED Configuration characteristic (§5.6).
enum LedStatusSlot : uint8_t {
    LED_SLOT_DORMANT       = 0,  // awake, dormant, nothing else to show   (default blue)
    LED_SLOT_ENFORCE_IDLE  = 1,  // enforcement window, condition met       (default orange)
    LED_SLOT_ENFORCE_ALARM = 2,  // enforcement, condition NOT met (blinks) (default red)
    LED_SLOT_WAKE_MOTION   = 3,  // momentary flash: woke on motion         (default green)
    LED_SLOT_WAKE_TIMER    = 4,  // momentary flash: woke on RTC/timer      (default light blue)
    LED_SLOT_WAKE_BLE      = 5,  // momentary flash: woke on incoming BLE   (default white)
    LED_SLOT_COUNT         = 6,
};

// Why the watch just came out of light sleep, used to pick the wake-cause flash.
enum LedWakeCause : uint8_t {
    LED_WAKE_NONE = 0,
    LED_WAKE_MOTION,
    LED_WAKE_TIMER,
    LED_WAKE_BLE,
};

// Snapshot of the bits of watch state the renderer needs, passed in by the
// caller so this module stays decoupled from the state machine internals.
struct LedStatusInput {
    bool unpaired;       // activity_state == UNPAIRED  → ring off
    bool enforcing;      // activity_state == ENFORCEMENT
    bool condition_met;  // enforcement condition currently satisfied
    int  hour;           // current local hour (0–23), for the DORMANT analog clock
    int  minute;         // current local minute (0–59), for the DORMANT analog clock
};

// FastLED init + load persisted config. Call once in setup().
void led_init();

// (Re)load slot colours / brightness from NVS, falling back to defaults.
void led_load_config();

// Apply an app LED Configuration write payload (§5.6) and persist it.
// Returns false if the payload is malformed (length < 2 + N*8).
bool led_apply_config(const uint8_t *data, size_t len);

// Serialise the full current config into `out` (the READ payload, §5.6).
// Returns bytes written, or 0 if `cap` is too small. Needs 2 + LED_SLOT_COUNT*8.
size_t led_serialize_config(uint8_t *out, size_t cap);

// Paint the ring as an analog clock: hour hand red, minute hand green, yellow
// when they overlap (§5.7.4). Leaves the frame lit — the SK6805 ring latches it,
// so the clock stays visible through DORMANT_SLEEP. In DORMANT this is driven by
// led_update() once per minute; there is no second hand.
void led_show_time(int hour, int minute);

// Blocking "hello" animation played once after a successful boot. Restores the
// ring to cleared state when done, ready for the first led_update().
void led_startup_animation();

// Begin a momentary wake-cause flash; expires after LED_WAKE_FLASH_MS.
void led_note_wake(LedWakeCause cause);

// Clear the ring immediately. Call right before entering enforcement light sleep
// (§5.7 priority 1). DORMANT_SLEEP no longer clears the ring — it leaves the
// analog clock lit through sleep.
void led_off();

void led_actual_off();

// Resolve the current slot, advance the alarm blink, expire the wake flash, and
// repaint only if the rendered frame changed. Tick from loop() every iteration.
void led_update(const LedStatusInput &in);
