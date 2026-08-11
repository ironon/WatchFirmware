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
    // Enforcement, criterion currently FAILING, but a tolerance window is still
    // suppressing the alarm — "you are near the dock; you have a bit longer".
    // Rendered as a filling progress bar around the ring rather than a blink, so
    // it never reads as an alarm and it says how much longer, not just "soon".
    // This slot's on_ms/off_ms are unused (the bar is steady); only the colour
    // applies. Appended rather than inserted on purpose: the values above are
    // on-wire slot ids in the app's LED Configuration characteristic, so
    // renumbering them would silently remap every configured colour.
    LED_SLOT_ENFORCE_GRACE = 6,  // grace/tolerance countdown              (default yellow)
    // What the watch currently believes about where it is relative to the event's
    // anchor — the verdict enforcement acted on, not the raw score. Shown for the
    // whole of an enforcement window (§5.7.5).
    //
    // Separate from ENFORCE_IDLE/ENFORCE_ALARM because compliance and proximity
    // are different facts and the ring only ever showed the first. On getAway they
    // are inverted (NEAR is the failing state); on stayNear they agree; during
    // donning grace compliance is asserted regardless of either. A user watching an
    // orange ring could not tell "it thinks I'm away" from "it hasn't decided" from
    // "grace is holding it quiet", which is exactly the confusion this resolves.
    // Appended, not inserted — see the note above on on-wire slot ids.
    LED_SLOT_PROX_NEAR     = 7,  // engine says NEAR (in the room)         (default magenta)
    LED_SLOT_PROX_AWAY     = 8,  // engine says AWAY (out of the room)     (default green)
    LED_SLOT_COUNT         = 9,
};

// Mirrors ProxProximity from the engine, kept as its own type so this module
// never has to include proximity.h. Values match deliberately.
enum LedProxVerdict : uint8_t {
    LED_PROX_NEAR  = 0,
    LED_PROX_AWAY  = 1,
    LED_PROX_AMBIG = 2,   // engine abstained, or no query has run yet
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
    // True when condition_met is only true BECAUSE a grace/tolerance window is
    // running — i.e. the criterion is actually failing right now and the alarm
    // starts when the window expires. Currently set for phoneAway's
    // PHONE_AWAY_TOLERANCE_S. Ignored unless condition_met is also true.
    bool in_grace;
    // How far through that window we are: 0 = just started, 255 = about to
    // expire. Drives how much of the ring is lit. Ignored unless in_grace.
    uint8_t grace_progress_u8;
    // Current enforcement output envelope, so the alarm ring rises and falls with
    // the buzzer/motor instead of free-running against it. `output_active` is true
    // while either output is being driven; `output_since_ms` is the millis() of
    // the last transition between the two, and anchors the blink phase so the
    // ring's first ON edge coincides with the buzzer's. Both ignored unless
    // enforcing && !condition_met.
    bool     output_active;
    uint32_t output_since_ms;
    // The last proximity verdict enforcement acted on, and whether it is recent
    // enough to show. `prox_fresh` false renders as AMBIG regardless of verdict —
    // a stale answer displayed as current is worse than admitting ignorance, and
    // the first poll of a window has not happened yet. Ignored unless enforcing.
    LedProxVerdict prox_verdict;
    bool           prox_fresh;
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

// Clear the ring immediately. DORMANT_SLEEP no longer clears the ring — it leaves
// the analog clock lit through sleep.
void led_off();

// Latch a two-LED "pilot" in the proximity colour and leave it lit. Call instead
// of led_off() before entering enforcement light sleep: the SK6805 holds the
// frame with the CPU asleep, so the watch keeps saying what it thinks for the
// whole interval rather than going dark between polls.
//
// Two LEDs, not twelve: an enforcement window sleeps most of its length, and the
// point is legibility, not brightness. This is the same standing cost DORMANT
// already pays to keep the analog clock lit through its own sleep (§5.7.4), and
// it replaces a ring that was fully off — during which the watch was
// indistinguishable from one that had stopped enforcing.
void led_show_prox_pilot(LedProxVerdict verdict, bool fresh);

void led_actual_off();

// Resolve the current slot, advance the alarm blink, expire the wake flash, and
// repaint only if the rendered frame changed. Tick from loop() every iteration.
void led_update(const LedStatusInput &in);
