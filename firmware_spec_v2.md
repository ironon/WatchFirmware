# ADHD Habit Enforcement Watch & Anchor — Firmware Specification

**Version:** 0.6 (Draft — see Section 0 for spec status, firmware parity, and the change log)
**Scope:** Firmware for the ESP32-C3-WROOM Watch and ESP32-C3-WROOM Anchor devices, intended as a complete reference for a coding agent.

---

## 0. Spec Status, Versioning & Change Log

This spec changes rapidly. Rules for maintaining this section:
- **Bump the version** (0.x → 0.x+1) on any normative change — behavior, wire format, constant defaults — and add a change-log entry (date + bullet list) at the same time.
- **Keep the parity table current:** update statuses whenever firmware work lands or an audit is performed, and stamp the audit date. The table tracks *spec vs. code*, not aspirations.
- Wire-format changes that require the app and firmware to move in lockstep must be **flagged as lockstep** in both the change log and MOBILE_APP_SPEC §0.

### 0.1 Spec ↔ firmware parity

Code homes: watch `WatchFIrmware/src`, anchor `AnchorFirmware/src`, shared proximity `proximity_engine`. **Last audited: 2026-07-12.**

| Spec area | Watch | Anchor | Notes |
|---|---|---|---|
| Schedule system + recurrence (§5.3) | ✅ | ✅ | Watch verified: `recalculate_day()` with daily/weekly/monthly, NVS blob persistence, midnight re-arm. **Anchor now implements §4.7** (2026-07-11): full blob persisted to NVS, local `anchor_recalculate_day()` (daily/weekly/monthly + once-by-date + negate + sort) on receipt/boot/midnight; the recurrence-day bug is fixed (matching runs against today's computed list). |
| GATT surface (§4.4, §5.6) | ✅ `…0011`–`…001B` present | ✅ `…0002`–`…000D` present | Verified by UUID grep. Pending Changes `…001A` (Read+Notify) and Emergency Pass `…001B` (Read+Write+Notify) created 2026-07-12. |
| Schedule transfer (§6.2) | ✅ v2 blob | ✅ v2 blob | **Format version byte (0x02) now implemented on watch + anchor** (2026-07-11); unknown version → 0x00 / HTTP 400. `donning_grace_s` field parsed on both. Extended Settings payloads landed (watch +`settle_window_min`, anchor +`tz_offset`). Lockstep with the app's `ScheduleEncoder` — flash together. |
| Proximity engine (§4.10, §5.4.1) | ✅ | ✅ | Shared module, working per commit history. |
| Enforcement / worn / LED / look-to-wake (§5.2–§5.7) | 🟡 | — | Believed implemented; not re-audited line-by-line this pass. |
| Phone docking (§4.11, `phoneAway` §5.4.1) | 🟡 | 🟡 | Dock characteristics present in code; behavior unaudited. |
| **Commitment integrity (§9)** | ✅ Phases 1–3 | — | **Phase 2 done** (2026-07-11): full §9.1/§9.3 diff gate (per-key event diff, tighten-immediate / loosen-quarantine, unsettled-above-baseline + far-future exceptions), §9.2 settle state in NVS on the monotonic elapsed basis, NVS pending queue (full proposed state, supersede + expiry rules), §9.4 autonomous promotion, Pending Changes `…001A` (Read+Notify, §9.5 format), END responds `0x01`/`0x03` (`0x04` fast-path retired per §6.2). **Phase 3 done** (2026-07-12): Emergency Pass `…001B` (SPEND applies a one-day negate bypassing the gate; SET_ALLOWANCE raise quarantined `0x03` / lower immediate; Read returns allowance/remaining/regen countdowns, all elapsed-basis), §9.7 time + Settings-tz hardening (`0x02` reject when a clock/tz write would escape the active window), §9.8 settings gating (dormancy `true→false` and settle-window increases quarantined, reverse immediate). On-hardware validation pending. |
| Donning grace + window-start unworn beep (§5.4.4) | ✅ | ✅ (blob parse) | **Done** (2026-07-11). Watch: window-start WATCH_REMOVED when unworn, `donningGraceS` grace with deadline/short-circuit/sleep-cap/expiry-recheck/removal-cancel. Watch Status `condition_met` byte added. Anchor parses `donning_grace_s` for wire parity. |
| Anchor schedule persistence + midnight recalc (§4.7 as of v0.5) | — | ✅ | **Done** (2026-07-11): NVS blob persist + SNTP/tz-based local recalc on receipt/boot/midnight; fail-open when no valid time or no stored schedule. |

Legend: ✅ implemented/verified · 🟡 believed implemented, unverified · ⚠️ diverges from spec · ❌ not started

### 0.2 Change log

**Anchor audit — 2026-07-12** (code fixes in `AnchorFirmware/src/main.cpp`; no wire-format change)
- **Field bug — spontaneous restarts:** several contributing causes fixed in the anchor. (1) Schedule store + `anchor_recalculate_day` ran inside the BLE SchedCtrl-END callback, i.e. on the NimBLE **host task**, racing the loop task's readers of `g_sched_blob`/`g_events` (use-after-free / corruption); now deferred to the loop task. (2) BLE BEGIN transfers `malloc`'d an untrusted 4-byte length with no cap; now bounded to 64 KB. Also added `esp_reset_reason()` + persisted reboot-counter / last-abnormal-reason logging at boot so field reboots are diagnosable. **Note:** the anchor disables the brownout detector (`RTC_CNTL_BROWN_OUT_REG=0`), so genuine rail dips will surface as `PANIC`/garbage resets, *not* `ESP_RST_BROWNOUT`, when reading that log.
- **Field bug — intermittent Identify beep:** the Identify callback beeped inline with a blocking `delay()` on the BLE host task and drove the buzzer pin in parallel with the loop's alarm state machine (single-core race). Reworked to a non-blocking request serviced by the loop with priority over the alarm; loop is now the single owner of the buzzer pin.
- **Conformance — HTTP `/schedule`:** the WiFi-credential BLE write started only mDNS + UDP on first connect, so the HTTP endpoint (§7) never listened until a later reconnect; now routed through `setup_network_services()`.
- **Pin divergence (flagged, unverified — needs hardware):** anchor firmware uses **GPIO 21** for the buzzer and **GPIO 9** (BOOT) for factory reset; §1's anchor table lists the buzzer on GPIO 12 and does not mention a reset pin. Field beeping works, so the code pins appear correct and §1 is stale — left unchanged pending hardware confirmation. `SERVO_MOVE_DURATION_MS` is 1000 ms in code vs. 500 ms in §7 (tunable).

**v0.6 — 2026-07-10**
- **New §5.4.4 "Donning Grace & Window-Start Worn Handling"** (unblocks the Sunrise Lock template): the watch sends WATCH_REMOVED at window start if already unworn (transition-independent), and a new per-event **`donningGraceS`** field (uint16 seconds, 0 = none) suppresses watch-side enforcement for N seconds after the watch is put on.
- §3.2 / §6.2: Event struct and schedule blob gain `donning_grace_s` (inserted after `negate`). The v2 blob layout has never been implemented, so `SCHEDULE_FORMAT_VERSION` stays `0x02` — the field ships in the same lockstep batch.
- §5.6: Watch Status payload gains a **`condition_met`** byte (after `active_event_id`) so the app can distinguish "actively alarming" from "in-window, compliant" — lockstep.
- §9.1: classification row for `donningGraceS` (decrease = tightening, increase = loosening).

**v0.5 — 2026-07-10**
- **New §9 "Commitment Integrity — The Watch as Root of Trust":** on-watch schedule-diff gate (tighten = immediate, loosen = quarantined 24 h), canonical tighten/loosen classification shared with the app, on-watch settle timers + settled baselines, pending queue with autonomous promotion, **Pending Changes** characteristic (`…001A`), on-watch **emergency-pass ledger** + characteristic (`…001B`), time-write hardening, settings gating, and a 3-phase rollout plan.
- §4.7 rewritten: the anchor **persists the full schedule blob to NVS** and recomputes today's event list locally (on receipt / boot / midnight). The app's **midnight push is removed** (a phone cannot reliably run at midnight); replaced by staleness-based foreground push.
- §5.1.4: mid-day schedule pushes now route through the §9.3 diff gate instead of blind replacement.
- §6.2: schedule blob gains a leading **format version byte** (`0x02`) — **lockstep** with app + anchor; schedule END responses extended (`0x03` partial-quarantine, `0x04` active-event rejection).
- §5.6 watch Settings payload extended: +`settle_window_min u16` (clamped 30–240, gated per §9.8); §4.4 anchor Settings payload extended: +`tz_offset_minutes int16` (needed for §4.7 local midnight recalc) — both **lockstep** with the app.
- §9.6/§5.6: Emergency Pass characteristic gains a SET_ALLOWANCE opcode (`0x02`). §9.3: pending entries store the full proposed state; a newer accepted edit to the same event cancels its pending entries. §9.4: expired pending entries are dropped. §9.7: guard extended to timezone changes via Settings.
- Audit: watch recurrence support **verified present** (no watch work needed); anchor recurrence/persistence divergence logged in §0.1.

**v0.4 and earlier** — pre-changelog; see git history. (v0.4 added the proximity engine, LED status ring, look-to-wake, analog-clock ring mode.)

---

## Table of Contents

1. Hardware Reference
2. System Overview
3. Shared Data Structures
4. Anchor Firmware
5. Watch Firmware
   - 5.1 State Machine
   - 5.2 Worn Detection
   - 5.3 Schedule System
   - 5.4 Enforcement
   - 5.5 Anchor Communication
   - 5.6 BLE GATT Service (Watch ↔ Phone)
   - 5.7 LED Status Indicator
6. Inter-Device Communication Protocols
7. Constants & Tunables
8. Power & Radio Optimization (Watch)
9. Commitment Integrity — The Watch as Root of Trust

---

**Note on modularity:** All code implementing the proximity fingerprinting algorithm — on both the watch and the anchor — must reside in a single shared file, `proximity.cpp`, with a corresponding `proximity.h`. Sections of the spec that describe this algorithm are labelled **(proximity.cpp)**. No proximity logic should appear in other firmware files; other files call only the public API declared in `proximity.h`.

---

## 1. Hardware Reference

### Watch (ESP32-C3-WROOM)

| Function              | Pin      | Notes                                              |
|-----------------------|----------|----------------------------------------------------|
| Active buzzer         | GPIO 12  | HIGH = on                                          |
| SPI MISO              | GPIO 2   |                                                    |
| SPI MOSI              | GPIO 7   |                                                    |
| SPI CLK               | GPIO 6   |                                                    |
| IMU chip select       | GPIO 8   |                                                    |
| IMU interrupt INT1    | GPIO 4   | Configured as motion wakeup source                 |
| IR worn emitter       | GPIO 1   | IR LED drive; HIGH = on                             |
| IR worn receiver      | GPIO 0   | ADC; phototransistor reflection level              |
| Vibration motor       | GPIO 10  | HIGH = on                                          |
| LED status ring       | GPIO 10  | Addressable LED data line. See note below.         |
| Battery voltage sense | GPIO 3   | ADC, Vbat/2 (non-functional, reserved for future) |

**IMU:** LIS3DHTR, connected via SPI.
**Worn sensor:** ITR8307/S17/TR8 (C81632) reflective opto-interrupter — a GaAs IR LED emitter (GPIO 1) paired with an NPN phototransistor receiver read on the ADC (GPIO 0). Skin held against the sensor reflects the emitted IR back into the phototransistor.
**LED status ring:** 12× SK6805-EC15 addressable LEDs in a ring, WS2812-compatible (800 kHz, GRB byte order), driven on a single data line via FastLED. Used purely as a status indicator (Section 5.7); it never affects enforcement logic.

**LED data-pin note (load-bearing — read before wiring the firmware):** On the current hardware revision the ring shares **GPIO 10** with the vibration motor, which is disabled by a PCB switch — so on this revision the motor is non-functional and GPIO 10 carries LED data. The next hardware revision moves the ring to **GPIO 7** (currently SPI MOSI) and restores the vibration motor on GPIO 10. The data pin is therefore a single named constant `LED_RING_DATA_PIN` (Section 7) that is the only thing that changes between revisions. **The LED feature must not modify, gate, or otherwise interfere with the vibration motor driver code on GPIO 10** — on hardware where the motor is active the LED ring is simply not wired to that pin.

### Anchor (ESP32-C3-WROOM)

| Function          | Pin      | Notes                                      |
|-------------------|----------|--------------------------------------------|
| Active buzzer     | GPIO 12  | HIGH = on                                  |
| SG90 servo signal | GPIO 10  | PWM; 180° = closed/locked, 160° = open    |

The anchor uses the ESP32-C3's built-in WiFi and BLE radios.

---

## 2. System Overview

The system consists of two device types:

**Anchors** are fixed devices placed at meaningful locations around a home. They advertise their identity over BLE and receive commands over WiFi. They hold a copy of the daily schedule and beep when instructed. Each anchor also runs a continuous proximity engine: it scans the surrounding RF environment (BLE devices and WiFi APs) at all times, maintains a statistical fingerprint of its installation location, and computes proximity scores on demand when the watch submits its own RF scan vector over a GATT connection.

**Watches** are wearable devices. They enforce a user-defined schedule of location and WiFi presence requirements by vibrating and/or buzzing when the user is not complying. They communicate with a companion mobile app over BLE and WiFi, and with anchors over WiFi.

---

## 3. Shared Data Structures

These data structures are used by both the watch firmware and the anchor firmware. The encoding used when transmitting over the wire is defined in Section 6.

### 3.1 Enumerations

```
enum RecurrenceType {
    once    = 0,
    daily   = 1,
    weekly  = 2,
    monthly = 3,
}

// Criteria defines what physical condition the watch enforces.
enum Criteria {
    getAway     = 0,   // User must be away from anchorId
    stayNear    = 1,   // User must be near anchorId
    getOffWifi  = 2,   // User must disconnect from wifiSSID
    getOnWifi   = 3,   // User must connect to wifiSSID
    phoneAway   = 4,   // Mode B: the user's phone is docked at anchorId; the user
                       // must stay away from it (phone-distance commitment).
}

// EnforcementProfile defines the watch's behavior during enforcement.
// All profiles are described fully in Section 5.4.
enum EnforcementProfile {
    strictSilent  = 0,
    normalSilent  = 1,
    looseSilent   = 2,
    strictBoth    = 3,
    normalBoth    = 4,
    looseBoth     = 5,
    strictBuzz    = 6,
    normalBuzz    = 7,
    looseBuzz     = 8,
}

// AnchorEnforcementProfile defines the anchor beeping pattern when the watch
// is removed during an enforcement event.
enum AnchorEnforcementProfile {
    light   = 0,   // beep 3s, wait 60s, repeat
    medium  = 1,   // beep 3s, wait 30s, repeat
    hard    = 2,   // beep 4s, wait 10s, repeat
}
```

### 3.2 Event Struct

```
struct Event {
    String  id;                     // UUID v4, 16 bytes
    int64   referenceDate;          // Unix timestamp (seconds). For 'once': the specific
                                    // date. For recurring: the origin/anchor date from
                                    // which recurrence is calculated.
    uint16  startTime;              // Minutes since midnight (0–1439)
    uint16  endTime;                // Minutes since midnight (0–1439). Must be > startTime.
    RecurrenceType recurrenceType;
    uint8   dayOfWeek;              // 1–7 (Mon–Sun). Required if weekly; 0 otherwise.
    uint8   dayOfMonth;             // 1–31. Required if monthly; 0 otherwise.
    Criteria criteria;
    EnforcementProfile profile;
    bool    negate;                 // If true, this event cancels a recurring event for
                                    // one specific day. See Section 5.3.
    uint16  donningGraceS;          // Seconds of watch-side enforcement suppression after the
                                    // watch is put on during this event's window (0 = none).
                                    // See Section 5.4.4.
    String? anchorId;               // UUID of target anchor. Null if WiFi-based criteria.
    String? wifiSSID;               // Target WiFi network name. Null if anchor-based criteria.

    // Anchor beep-on-removal fields:
    List<String> beepAnchors;       // UUIDs of anchors that should beep if the watch is
                                    // removed during this event. May be empty.
    AnchorEnforcementProfile? anchorProfile;  // Null if beepAnchors is empty.
}
```

**Validation rules (enforced at the app level before transmission; the firmware may assert but not correct):**
- If `recurrenceType == weekly`, `dayOfWeek` must be 1–7.
- If `recurrenceType == monthly`, `dayOfMonth` must be 1–31.
- If `recurrenceType == once` or `daily`, `dayOfWeek` and `dayOfMonth` must be 0.
- Exactly one of `anchorId` or `wifiSSID` must be non-null, except when `negate == true`, in which case both may be null.
- If `beepAnchors` is non-empty, `anchorProfile` must be non-null.
- `endTime > startTime` (events may not span midnight).
- `donningGraceS` must be 0–1800 (app-enforced clamp; the firmware does not correct out-of-range values).

### 3.3 Anchor Device Record

Used by the watch to track known anchors:

**Record lifecycle:** An `AnchorRecord` is created whenever the watch first needs to track an anchor — on first advertisement discovery, or when a loaded schedule references the anchor's UUID (as `anchorId` or in `beepAnchors`). The app's IP-table push only *fills in* the optional `ipAddress` field of an existing record; it must never be the sole means by which a record comes to exist. `bleMac` (from BLE advertisement) is the load-bearing field for proximity and beep delivery; `ipAddress` is a best-effort optimization with mDNS/broadcast fallbacks (§5.5.1) and may remain null.

```
struct AnchorRecord {
    String  uuid;           // UUID v4
    String  name;           // Human-readable name, assigned in mobile app
    uint8   bleMac[6];      // BLE MAC address of this anchor (captured from advertisement).
                            // Used by the watch to initiate a directed GATT connection without
                            // a full BLE scan. Populated on first advertisement seen.
    int8    lastRSSI;       // Last observed advertisement RSSI in dBm. Retained for discovery
                            // filtering but no longer used for near/away classification.
    int64   lastSeen;       // Unix timestamp of last BLE advertisement seen
    String? ipAddress;      // Last known IPv4 address, null if never seen on WiFi
    int64   ipLastUpdated;  // Unix timestamp when ipAddress was last confirmed by app
    uint8   lastProxScore;  // Last proximity score returned by this anchor (0–255). Updated
                            // each time the watch completes a proximity query. Not persisted.
}
```

---

### 3.4 Shared Proximity Structures **(proximity.cpp)**

These structures are used by both the watch (to build and transmit scan vectors) and the anchor (to receive, compute, and store). They are defined in `proximity.h`.

```
// One entry in a scan vector. Represents a single observed RF emitter.
struct ProxDevice {
    uint8   mac[6];     // BLE MAC address or WiFi BSSID (6 bytes in both cases)
    uint8   type;       // 0 = BLE device, 1 = WiFi AP
    int8    rssi;       // Signal strength in dBm (signed)
}

// A complete RF environment snapshot. Assembled by the watch before each proximity query.
struct ProxScanVector {
    ProxDevice  devices[PROX_MAX_DEVICES];
    uint8       count;  // Number of valid entries (0 to PROX_MAX_DEVICES)
}

// Result returned by the anchor after scoring a watch's scan vector.
struct ProxScoreResult {
    uint8   score;  // Proximity likelihood: 0 = definitely away, 255 = definitely here.
                    // Computed as a blend of live correlation and fingerprint likelihood,
                    // scaled to uint8. See Section 4.10.
    uint8   flags;
    // Flag bits:
    //   bit 0 (PROX_FLAG_FINGERPRINT_ACTIVE): fingerprint has >= PROX_MIN_DEVICE_COUNT
    //          devices with sufficient samples to meaningfully contribute to the score.
    //   bit 1 (PROX_FLAG_LOW_DEVICE_COUNT): fewer than PROX_MIN_DEVICE_COUNT devices
    //          were seen in the watch's vector; score is degraded and unreliable.
}
```

---

### 4.1 Overview

The anchor is a minimal device that:
1. Advertises its UUID over BLE continuously.
2. Accepts BLE connections for initial setup (WiFi credentials, identify command) and for proximity queries from the watch.
3. Connects to WiFi and listens for UDP commands from the watch.
4. Holds a daily schedule copy and beeps when appropriate.
5. Manages an SG90 servo on GPIO 10 that mechanically locks or releases the anchor's strap, preventing the anchor from being moved during an enforcement event.
6. Runs the proximity engine continuously **(proximity.cpp)**: scans surrounding BLE devices and WiFi APs, maintains a statistical location fingerprint, and computes proximity scores on demand.

### 4.2 Identity & First Boot

On first boot, the anchor checks NVS (non-volatile storage) for an existing UUID. If none is found (factory-fresh state), it generates a random UUID v4, stores it to NVS, and uses it permanently. The UUID never changes unless a factory reset is performed.

**Factory reset procedure:** Hold the factory reset trigger (implementation detail: a GPIO pad or button defined in hardware config) for 5 continuous seconds. The firmware erases the NVS partition and reboots. On next boot, a new UUID is generated.

### 4.3 BLE Advertising

The anchor advertises continuously using the **iBeacon format**:
- **UUID:** The anchor's stored UUID.
- **Major:** `0x4A0F` — fixed Impulse namespace fingerprint. The watch uses this value to filter out non-Impulse iBeacons (AirTags, Tile trackers, etc.) during BLE scanning.
- **Minor:** `0x0000` (unused).
- **Advertising interval:** 100ms.
- **Connectable:** Yes. The anchor must remain connectable at all times to support setup and identify operations.

The anchor also emits a **scan response packet** containing:
- The `ANCHOR_SERVICE_UUID` as a service UUID, allowing the mobile app to identify anchors by service UUID on platforms (iOS) where Apple manufacturer data is stripped from scan results.
- The anchor's 16-byte UUID as service data under `ANCHOR_SERVICE_UUID`, so the app can read the anchor's identity from the scan response without connecting.

### 4.4 Anchor GATT Service

The anchor exposes one custom BLE GATT service for setup operations.

**Service UUID:** `ANCHOR_SERVICE_UUID` (a fixed constant defined in the shared codebase, used by both anchor firmware and the mobile app).

#### Characteristic: Identify (Write, No Response)

**UUID:** `ANCHOR_IDENTIFY_CHAR_UUID`

On any write (payload is ignored), the anchor activates its buzzer for `IDENTIFY_BEEP_DURATION_MS` (default: 800ms), then stops. This allows the mobile app user to identify which physical anchor corresponds to which UUID.

#### Characteristic: WiFi Credentials (Write, With Response)

**UUID:** `ANCHOR_WIFI_CRED_CHAR_UUID`

**Payload (JSON, UTF-8):**
```json
{ "ssid": "<network name>", "password": "<password>" }
```

On write: the anchor stores the credentials to NVS and attempts to connect to the given WiFi network. It responds with:
- `0x01` on successful connection.
- `0x00` on failure (connection timeout or wrong credentials).

The anchor stores only one WiFi credential pair. Writing new credentials overwrites the old ones.

#### Characteristic: Anchor Settings (Write, With Response)

**UUID:** `ANCHOR_SETTINGS_CHAR_UUID`

**Payload (binary, little-endian):**
```
[2 bytes: max_beep_minutes (uint16)]
[2 bytes: tz_offset_minutes (int16, signed, minutes east of UTC)]
```

`max_beep_minutes` is the maximum number of minutes the anchor will beep continuously for a single watch-removed event, regardless of event end time. Default: 30. This is a device-level setting, not per-event.

`tz_offset_minutes` gives the anchor the timezone it needs to compute local midnight and day-of-week/day-of-month for its local schedule recalculation (§4.7). Wall-clock time itself comes from SNTP once WiFi-connected.

On write: stores settings to NVS, responds with `0x01`.

> **Lockstep note:** the payload grew from 2 to 4 bytes in spec v0.5 — update the app in the same batch as the schedule version byte.

#### Characteristic: Schedule Transfer Control (Write, With Response)

**UUID:** `ANCHOR_SCHED_CTRL_CHAR_UUID`

See Section 6.2 for the full schedule transfer protocol. The anchor uses the same protocol as the watch.

#### Characteristic: Schedule Transfer Data (Write, No Response)

**UUID:** `ANCHOR_SCHED_DATA_CHAR_UUID`

See Section 6.2.

---

#### Characteristic: Proximity Vector (Write With Response) **(proximity.cpp)**

**UUID:** `ANCHOR_PROX_VECTOR_CHAR_UUID`

The watch writes its current RF scan vector to this characteristic. The anchor synchronously computes a proximity score and stores it in the Proximity Score characteristic. See Section 6.3 for the wire format of the vector payload.

**On write:**
1. Deserialise the `ProxScanVector` from the payload (Section 6.3.1).
2. Call `prox_compute_score(watch_vec)` to get a `ProxScoreResult` (Section 4.10.3).
3. Store the result in `ANCHOR_PROX_SCORE_CHAR_UUID` so the watch can read it immediately.
4. Call `prox_maybe_update_fingerprint(watch_vec, result)` to conditionally accept the sample into the self-supervised training pipeline (Section 4.10.4).
5. Respond `0x01` on success. Respond `0x00` if the payload is malformed or zero-length.

MTU negotiation to `BLE_REQUESTED_MTU` (512 bytes) must be completed before this write. If the negotiated MTU is below `PROX_MIN_MTU_BYTES`, the watch truncates the vector to fit; the anchor accepts whatever it receives.

---

#### Characteristic: Proximity Score (Read + Notify) **(proximity.cpp)**

**UUID:** `ANCHOR_PROX_SCORE_CHAR_UUID`

The watch reads this characteristic immediately after a successful write to `ANCHOR_PROX_VECTOR_CHAR_UUID`. The anchor also sends a BLE notification whenever the stored value changes, allowing a connected mobile app to observe proximity scores in real time (useful for debugging and fingerprinting progress feedback).

**Payload (2 bytes):**
```
[1 byte: score   (0–255; see ProxScoreResult.score)]
[1 byte: flags   (see ProxScoreResult.flags)]
```

The characteristic is initialised to `{score: 0, flags: 0}` on boot and is updated synchronously before the ATT Write Response for `ANCHOR_PROX_VECTOR_CHAR_UUID` is sent, so a Read immediately following that Write is guaranteed to see the new value.

---

#### Characteristic: Fingerprint Transfer Control (Write With Response) **(proximity.cpp)**

**UUID:** `ANCHOR_FINGERPRINT_CTRL_CHAR_UUID`

Used by the mobile app to upload a complete pre-built fingerprint blob to the anchor. Uses the same three-phase BEGIN / DATA / END protocol as the schedule transfer (Section 6.2), with the blob format defined in Section 6.3.2.

On receiving END with a matching CRC: deserialise the fingerprint blob, replace the anchor's in-memory fingerprint and device registry, persist to NVS, respond `0x01`.
On CRC mismatch: discard buffer, respond `0x00`. App must retry from BEGIN.

---

#### Characteristic: Fingerprint Transfer Data (Write, No Response) **(proximity.cpp)**

**UUID:** `ANCHOR_FINGERPRINT_DATA_CHAR_UUID`

Data chunks for the fingerprint upload. See Section 6.2 (same protocol as schedule data).

---

#### Characteristic: Toggle (Read + Write, With Response)

**UUID:** `ANCHOR_TOGGLE_CHAR_UUID`

Controls the SG90 servo that mechanically locks or releases the anchor's strap. See Section 4.9 for full servo management behavior.

**Write payload (1 byte):**
- `0x00` — Close (lock): rotate servo to `SERVO_CLOSED_DEGREES` (180°).
- `0x01` — Open (unlock): rotate servo to `SERVO_OPEN_DEGREES` (160°).

**Read payload (1 byte):**
- `0x00` — Servo is currently in the closed/locked position.
- `0x01` — Servo is currently in the open/unlocked position.

**Response codes (returned after write):**
- `0x01` — Command accepted and executed.
- `0x02` — Command rejected: an enforcement event is currently active that involves this anchor (see Section 4.9 for the exact check). Only open commands can be rejected; close commands are always accepted.

#### Characteristic: Dock Register (Write, With Response) **(phone docking, §4.11)**

**UUID:** `ANCHOR_DOCK_REGISTER_CHAR_UUID` (`4A0F000C-F8CE-11EE-8001-020304050607`)

Written by the **mobile app** after the phone connects to this anchor, to mark *its own* BLE connection as "the docking phone." The anchor records that connection's handle and thereafter monitors its link RSSI to decide whether the phone is docked (§4.11). This is required because the anchor also accepts transient connections from watches (proximity queries) and from the setup app — it must know which connection to measure.

**Payload (1 byte):** `0x01` — register the writing connection as the docking phone. `0x00` — unregister. Responds `0x01`.

#### Characteristic: Dock Status (Read + Notify) **(phone docking, §4.11)**

**UUID:** `ANCHOR_DOCK_STATUS_CHAR_UUID` (`4A0F000D-F8CE-11EE-8001-020304050607`)

Read by the **watch** during a `phoneAway` proximity query (§5.4.1) to learn whether the phone is still docked at this anchor. The anchor notifies on change.

**Payload (2 bytes):**
```
[1 byte: docked   (0x01 = phone docked, 0x00 = undocked / no registered phone)]
[1 byte: rssi      (last anchor↔phone link RSSI, encoded as rssi + 128; 0 if none)]
```

### 4.5 WiFi Operation

After successfully connecting to WiFi, the anchor:
1. Obtains an IP address via DHCP.
2. Registers itself on the local network via mDNS as `<UUID>.local` for discoverability.
3. Opens a UDP socket on port `ANCHOR_UDP_PORT` (defined constant) and listens for commands.

If WiFi disconnects, the anchor continues BLE advertising and attempts reconnection every `ANCHOR_WIFI_RETRY_INTERVAL_S` seconds using stored credentials.

### 4.6 UDP Command Handling

All UDP datagrams received on `ANCHOR_UDP_PORT` are parsed as follows:

**Packet format (binary, little-endian):**
```
[1 byte:  command]
[16 bytes: watch UUID]
[16 bytes: event UUID]
```

**Commands:**
- `0x01` — WATCH_REMOVED: The watch has been taken off during an enforcement event.
- `0x02` — WATCH_WORN: The watch has been put back on.

**On WATCH_REMOVED:**
1. Look up the current event from the internal daily schedule (see Section 4.7).
2. Check whether this anchor's UUID appears in the event's `beepAnchors` list.
3. If yes, begin the anchor enforcement beeping pattern defined by the event's `anchorProfile`.
4. Continue beeping until:
   - A WATCH_WORN command is received for the same event UUID, OR
   - The event's `endTime` is reached (per the internal schedule), OR
   - `max_beep_minutes` of continuous beeping has elapsed.
5. If this anchor's UUID is not in `beepAnchors`, ignore the command.

**On WATCH_WORN:**
1. Stop all beeping immediately.

### 4.7 Anchor Schedule Management

The anchor receives the same full schedule blob as the watch (recurring + one-time events, Section 6.2), **persists it to NVS** on successful receipt, and computes the current day's sorted event list locally via `recalculate_day` (Section 5.3) — on receipt, on boot, and at each local midnight. The anchor never depends on a nightly push to know today's schedule. It uses the day's list only to:
- Determine whether it should respond to a WATCH_REMOVED command (by checking if its UUID is in `beepAnchors` of the current event).
- Know when to stop beeping (event `endTime`).

**Local-time dependency:** computing "today" (day-of-week / day-of-month) and local midnight requires wall-clock time plus a timezone offset. The anchor obtains time via SNTP once WiFi-connected and receives `tz_offset_minutes` in its Settings write (§4.4). If the anchor has no valid time (never synced since boot), it behaves as if it has no schedule — no beep windows (fail-open).

The schedule is pushed by the mobile app over WiFi using an HTTP POST to `http://<anchor-ip>/schedule`. See Section 6.2 for the schedule encoding.

The app pushes the schedule to all known anchor IPs unconditionally — it does not wait for or check whether any anchor is online before sending. If an anchor is offline at push time, it simply misses the update; its persisted schedule keeps recurring events correct until the next successful push. The app pushes the schedule in the following situations:
- Whenever the user modifies the schedule in the app (to all known anchors).
- Whenever a new anchor IP is added to the app's anchor table.
- Opportunistically when the app comes to the foreground and the last successful push to an anchor is stale (older than ~12 h). (There is deliberately **no midnight push** — a mobile app cannot reliably execute at midnight; the anchor's local midnight recalculation replaces it.)

If the anchor has no stored schedule at all (factory-fresh, or NVS cleared), it treats `beepAnchors` as empty for all events and will not beep.

### 4.8 Anchor Beeping Patterns

Beeping is driven by the `anchorProfile` field of the active event:

| Profile | Pattern                     |
|---------|-----------------------------|
| light   | Buzzer ON 3s, OFF 60s, repeat |
| medium  | Buzzer ON 3s, OFF 30s, repeat |
| hard    | Buzzer ON 4s, OFF 10s, repeat |

Beeping stops under the conditions listed in Section 4.6.

### 4.9 Servo Management

The anchor uses an SG90 servo on GPIO 10 to mechanically lock or release its strap.

#### Default State

On boot, the servo is commanded to the closed position (`SERVO_CLOSED_DEGREES` = 180°), then detached. The internally tracked servo state is initialized to `closed`.

#### ServoManager Behavior

The servo must never actively hold a position. The pattern for every command is:

1. Attach the servo (begin sending PWM on GPIO 10).
2. Write the target angle (`SERVO_CLOSED_DEGREES` or `SERVO_OPEN_DEGREES`).
3. Wait `SERVO_MOVE_DURATION_MS` for the servo to physically reach the position.
4. Detach the servo (stop PWM output on GPIO 10).
5. Update internal servo state to `open` or `closed` accordingly.

#### Enforcement Check

An enforcement event is considered **active and involving this anchor** if the current local time falls within any event's `[startTime, endTime)` window in the day's schedule, AND at least one of the following is true:
- This anchor's UUID matches `event.anchorId`.
- This anchor's UUID appears in `event.beepAnchors`.

#### Toggle Command Logic

```
function handle_toggle_command(value: uint8) -> uint8:
    if value == 0x00:  // close
        servo_manager_command(SERVO_CLOSED_DEGREES)
        return 0x01  // always accepted

    else if value == 0x01:  // open
        if anchor_is_in_active_enforcement_event():
            return 0x02  // rejected
        servo_manager_command(SERVO_OPEN_DEGREES)
        return 0x01  // accepted
```

#### Auto-Close on Enforcement Start

When the anchor's schedule determines that an enforcement event has begun (i.e., the current time has entered a `[startTime, endTime)` window for an event that involves this anchor):
- If the internal servo state is `open`, immediately issue a close command via `servo_manager_command(SERVO_CLOSED_DEGREES)`.
- This auto-close bypasses the enforcement check and always executes.

The anchor evaluates enforcement boundaries by checking the schedule at the start of each event window. Because the anchor does not have an RTC-callback system as sophisticated as the watch, it checks whether any event boundary has been crossed on each schedule update push and on each Toggle write. For time-accurate auto-close, the app is responsible for ensuring the anchor receives the schedule before events begin.

---

### 4.10 Anchor Proximity Engine **(proximity.cpp)**

The proximity engine runs entirely on the anchor. Its responsibilities are: continuously scanning the surrounding RF environment; storing and incrementally updating a statistical fingerprint of the anchor's location; and computing a proximity score when the watch submits its own scan vector. All functions described in this section are implemented in `proximity.cpp`.

#### 4.10.1 Device Registry

The anchor maintains a **global device registry**: an ordered list of every RF emitter (BLE or WiFi) it has ever observed, identified by `{mac[6], type}`. Each entry has an integer index that is stable for the lifetime of the NVS storage partition. The registry is capped at `ANCHOR_PROX_MAX_FINGERPRINT_DEVICES` entries. When the cap is reached, new devices are silently ignored.

The registry and all fingerprint data are persisted to NVS under a dedicated namespace (`prox`) and loaded on boot.

#### 4.10.2 Continuous Background Scanning **(proximity.cpp)**

The anchor runs two background tasks that keep its live scan cache current:

**BLE scan task:** Repeats indefinitely. Each cycle runs a passive BLE scan for `ANCHOR_PROX_BLE_SCAN_DURATION_MS` milliseconds, collecting all observable advertisements (not filtered by Impulse Major — all BLE devices are included). The cache entry for each seen device is updated to its most recent RSSI. Devices not seen in the current cycle are retained in the cache but their RSSI is aged: after `ANCHOR_PROX_DEVICE_STALE_MS` milliseconds without an update, the entry is marked stale and excluded from score computations. The BLE scan task sleeps for `ANCHOR_PROX_BLE_SCAN_INTERVAL_MS` between cycles.

**WiFi scan task:** Repeats indefinitely. Each cycle calls `esp_wifi_scan_start()` in station mode (the anchor does not need to be connected) to enumerate nearby APs with their BSSIDs and RSSI values. Results update the same live scan cache. The WiFi scan task sleeps for `ANCHOR_PROX_WIFI_SCAN_INTERVAL_S` seconds between cycles.

Both tasks share a single `live_scan_cache: map<{mac, type}, {rssi, last_updated_ms}>`, protected by a mutex.

#### 4.10.3 Score Computation **(proximity.cpp)**

`prox_compute_score(watch_vec: ProxScanVector) -> ProxScoreResult`

This function is called synchronously when the watch writes to `ANCHOR_PROX_VECTOR_CHAR_UUID`. It blends two signals:

**Signal A — Live Pearson Correlation:**

Constructs a shared device set: the intersection of devices present in both `watch_vec` and the anchor's non-stale `live_scan_cache`. Computes Pearson correlation over this set:

$$\rho = \frac{\sum_d (w_d - \bar{w})(a_d - \bar{a})}{\sqrt{\sum_d (w_d - \bar{w})^2} \cdot \sqrt{\sum_d (a_d - \bar{a})^2}}$$

where $w_d$ is the watch's RSSI for device $d$ and $a_d$ is the anchor's cached RSSI for device $d$. $\rho \in [-1, 1]$; clamp to $[0, 1]$ (negative correlation is treated as zero proximity).

**Signal B — Fingerprint Log-Likelihood:**

If the fingerprint has at least `PROX_MIN_DEVICE_COUNT` devices with `W >= PROX_MIN_FINGERPRINT_WEIGHT`, compute the Gaussian Naive Bayes log-likelihood of `watch_vec` against the stored fingerprint. For each device $d$ in the fingerprint:

$$\log P_d = -\frac{1}{2}\log(2\pi\sigma_d^2) - \frac{(w_d - \mu_d)^2}{2\sigma_d^2}$$

where $w_d$ is taken from `watch_vec` if present, or the sentinel value `PROX_MISSING_RSSI_DBM` if absent. Normalise the total log-likelihood to $[0, 1]$ using a sigmoid scaled to the expected range for the device count, yielding $L \in [0, 1]$.

**Blending:**

The blend weight $\alpha$ is a function of $W_{total}$, the total cumulative weight across all fingerprint devices:

$$\alpha(W_{total}) = e^{-W_{total} / PROX\_ALPHA\_W0}$$

So $\alpha = 1.0$ when no fingerprint exists (pure correlation), and decays toward $0.0$ as the fingerprint accumulates weight.

$$\text{score}_{float} = \alpha \cdot \rho + (1 - \alpha) \cdot L$$

Scale to uint8: `result.score = (uint8)(score_float * 255)`.

**Low device count fallback:**

If `watch_vec.count < PROX_MIN_DEVICE_COUNT`, set `PROX_FLAG_LOW_DEVICE_COUNT` in `result.flags` and fall back to a single raw RSSI comparison: if the watch's vector contains an entry for this anchor's own BLE MAC with RSSI >= `ANCHOR_NEAR_RSSI_THRESHOLD_DBM`, set `result.score = 200`; otherwise `result.score = 50`. This ensures the system degrades gracefully in RF-sparse environments rather than failing silently.

Set `PROX_FLAG_FINGERPRINT_ACTIVE` in `result.flags` if Signal B was used.

#### 4.10.4 Self-Supervised Fingerprint Training **(proximity.cpp)**

`prox_maybe_update_fingerprint(watch_vec: ProxScanVector, result: ProxScoreResult)`

Called after every score computation. Accepts the sample into the fingerprint's weighted Welford accumulator if and only if the following gating conditions are all met:

1. `result.score / 255.0 >= PROX_COLLECT_SCORE_THRESHOLD` — the blended score is high enough to imply the watch is near this anchor.
2. The sample is **unambiguous**: no other anchor in the watch's vector has a raw RSSI within `PROX_COLLECT_AMBIGUITY_MARGIN_DBM` of this anchor's own advertisement RSSI as seen in the watch vector. This check uses the raw RSSI of known anchor MACs extracted from `watch_vec` to detect boundary situations.

If both conditions are met, update the fingerprint for every device $d$ in `watch_vec` using the **weighted Welford algorithm**, with sample weight $w_n = \text{result.score} / 255.0$:

$$\mu_n = \mu_{n-1} + \frac{w_n}{W_n}(x_n - \mu_{n-1})$$
$$M_n = M_{n-1} + w_n(x_n - \mu_{n-1})(x_n - \mu_n)$$
$$\sigma^2_n = \frac{M_n}{W_{n-1}} \quad \text{for } W_{n-1} > 0$$
$$W_n = W_{n-1} + w_n$$

where $x_n = w_d$ (the watch's RSSI for device $d$). For devices in the registry but absent from `watch_vec`, substitute `PROX_MISSING_RSSI_DBM` as $x_n$ with a reduced weight of $w_n / 2$ — this teaches the fingerprint what "not visible" looks like at this location.

New devices seen in `watch_vec` that are not yet in the device registry are added to the registry (up to the cap) before updating.

Persist updated fingerprint to NVS every `PROX_NVS_PERSIST_INTERVAL_S` seconds (not on every sample, to limit flash wear).

#### 4.10.5 App-Provided Fingerprint Upload **(proximity.cpp)**

When the mobile app transmits a fingerprint blob via `ANCHOR_FINGERPRINT_CTRL_CHAR_UUID` / `ANCHOR_FINGERPRINT_DATA_CHAR_UUID`, the anchor calls:

`prox_load_fingerprint(blob: FingerprintBlob)`

This completely replaces the in-memory fingerprint and device registry with the contents of the blob (Section 6.3.2), then persists to NVS. The app-provided fingerprint takes precedence over all self-supervised data accumulated prior to the upload. Self-supervised accumulation resumes immediately after the upload completes, building on top of the app-provided baseline.

#### 4.10.6 BLE Connection Behaviour During Proximity Query

The anchor must remain connectable at all times (per Section 4.3). When the watch connects for a proximity query, the anchor services the GATT operations in parallel with its background scan tasks — the connection does not pause or interrupt the scan loop. MTU negotiation to `BLE_REQUESTED_MTU` (512 bytes) is initiated by the watch immediately after connection; the anchor accepts whatever MTU the stack negotiates.

### 4.11 Phone Docking Detection

Used only for `phoneAway` (Mode B) commitments. The anchor confirms the phone is physically docked at it, so the watch can distinguish "user is away from the docked phone" (compliant) from "the phone left the dock with the user" (a violation the watch↔anchor proximity alone cannot see).

**Mechanism:**
1. The mobile app connects the phone to the anchor and writes `0x01` to `ANCHOR_DOCK_REGISTER_CHAR_UUID`. The anchor stores that connection's handle as the docking-phone handle. The app keeps this connection alive for the whole window (the user is told to leave the app open and disable low-power mode; maintaining the link is the user's responsibility).
2. The anchor polls the RSSI of the registered connection every `DOCK_RSSI_POLL_MS` (default 2000 ms) via the host `ble_gap_conn_rssi()` call.
3. **`docked` = (a docking-phone connection is registered and live) AND (its RSSI ≥ `DOCK_RSSI_THRESHOLD_DBM`, default −60 dBm).** A docked phone sits on/beside the anchor, so its link RSSI is very strong; the phone moving even a few feet away, or the connection dropping (weak link / app closed / low-power), drops `docked` to false.
4. The anchor writes `{docked, rssi}` to `ANCHOR_DOCK_STATUS_CHAR_UUID` and notifies on change. On disconnect of the docking-phone handle, the anchor clears it and reports undocked.

Because a lost/weak link is reported as *undocked* (not "unknown"), the burden is on the user to keep the phone docked and the app healthy — consistent with the product's honesty framing (the anchor-docking path is the reliable one).

---

## 5. Watch Firmware

### 5.1 State Machine

#### 5.1.1 Connectivity Flags

The watch maintains two independent boolean flags at all times:

- `bt_connected` — true when the companion phone app has an active BLE connection.
- `wifi_connected` — true when the watch has an active WiFi connection.

These flags are updated immediately on connect/disconnect events and are independent of the activity state.

The named connectivity states from the product design map to flag combinations as follows (for documentation and app status reporting only):

| Name           | bt_connected | wifi_connected |
|----------------|--------------|----------------|
| wild           | true         | false          |
| away           | false        | true           |
| disconnected   | false        | false          |

**Settings governing connectivity behavior:**

| Setting                    | Default | Effect                                                                 |
|----------------------------|---------|------------------------------------------------------------------------|
| `DISCONNECTED_IS_DORMANT`  | true    | If true, the watch enforces schedule normally when both flags are false |
| `AWAY_IS_DORMANT`          | true    | If true, the watch enforces schedule normally when only wifi is true    |

When a connectivity flag becomes true and enforcement was previously suspended due to it being false, immediately run `recalculate_day(today)` and re-arm RTC callbacks.

#### 5.1.2 Activity States

The watch has three activity states:

```
UNPAIRED
DORMANT
DORMANT_SLEEP
ENFORCEMENT
```

**UNPAIRED**

Factory default. The watch has never been paired with a phone.

Behavior:
- Does not read from IMU.
- IR worn sensor has no effect.
- No schedule enforcement.
- Only waits for an incoming BLE connection request.

Transitions:
- → DORMANT on first successful BLE connection. On transition: emit a short beep (`UNPAIRED_BEEP_DURATION_MS`) and a short vibration (`UNPAIRED_VIBRATE_DURATION_MS`).

This state is permanent until factory reset once any BLE connection has ever been made.

**DORMANT**

The watch is ready to enforce but no event is currently active. This is the nominal resting state.

Behavior:
- Periodically scans for BLE advertisements from known and unknown anchors (interval: `BLE_SCAN_INTERVAL_S`). An advertisement is only accepted as an Impulse anchor if it is a valid iBeacon **and** its Major field equals `0x4A0F` (the Impulse namespace fingerprint). Advertisements failing this check are silently ignored, filtering out third-party iBeacons such as AirTags.
- On discovering an unknown anchor UUID, adds it to the seen-anchors list and notifies the app via the Seen Anchors GATT characteristic.
- Periodically scans for known WiFi networks (interval: `WIFI_SCAN_INTERVAL_S`). On finding a known network, attempts connection.
- Processes incoming BLE GATT writes from the app (schedule updates, settings, etc.).
- When it has completed all pending work, transitions to DORMANT_SLEEP.
- **Motion interrupt re-arm delay:** After processing an IMU interrupt, the firmware does not immediately re-arm the interrupt. Instead it waits `MINIMUM_BLE_DELAY_DORMANT` milliseconds before clearing the LIS3DH INT1 latch (which allows the next motion event to fire the ISR). While INT1 is latched HIGH no new rising edge is generated, so the interrupt is effectively muted for this period. This prevents rapid continuous movement from repeatedly resetting the `DORMANT_TO_SLEEP_IDLE_MS` timer and blocking the transition to DORMANT_SLEEP.

Transitions:
- → DORMANT_SLEEP when no pending callbacks within `DORMANT_TO_SLEEP_IDLE_MS` and IMU reports no significant motion.
- → ENFORCEMENT when an RTC callback fires for an event start time.

**DORMANT_SLEEP**

The watch is in ESP32 light sleep mode to conserve battery. Before entering this state, the firmware sets an RTC wakeup timer for the next event boundary (start or end time), or for midnight if no more events remain today.

Wakeup sources (any of these transitions to DORMANT):
- RTC timer fires.
- IMU motion interrupt fires on GPIO 4.
- Incoming BLE packet received.

**ENFORCEMENT**

An event's time window is active and the watch is monitoring and enforcing its criteria.

Behavior:
- Polls the enforcement condition every `ENFORCEMENT_POLL_INTERVAL_S` seconds.
- Additionally checks the condition immediately on IMU motion interrupt (GPIO 4), as significant movement may indicate the user has complied.
- **Motion interrupt re-arm delay:** After processing an IMU interrupt, the firmware waits `MINIMUM_BLE_DELAY_ENFORCEMENT` milliseconds before re-arming the interrupt (clearing the LIS3DH INT1 latch). This limits how often motion can trigger a fresh BLE scan during normal activity, preventing battery drain from continuous movement that has no bearing on the enforcement criteria.
- If condition is NOT met: activates the event's `EnforcementProfile` pattern (Section 5.4).
- If condition IS met: ceases all motor/buzzer output immediately. If the user has also been still for `ENFORCEMENT_IDLE_BEFORE_SLEEP_MS`, enters enforcement light sleep (Section 5.4.3).
- Monitors the IR worn sensor. If the watch is determined to be not worn (Section 5.2), sends WATCH_REMOVED UDP datagrams to all anchors in `beepAnchors`. When worn again, sends WATCH_WORN.

Transitions:
- → DORMANT when the RTC callback fires for the event's `endTime`.

#### 5.1.3 Boot-Time Recovery

On every boot:
1. Load stored schedule from flash.
2. Load stored settings from NVS.
3. Determine current time from RTC (if available) or request time sync from phone/NTP.
4. Call `recalculate_day(today)`.
5. Find whether the current time falls within any event's `[startTime, endTime)` window.
   - If yes: enter ENFORCEMENT immediately for that event.
   - If no: enter DORMANT, arm RTC callback for the next upcoming event boundary.
   - If no more events remain today: enter DORMANT_SLEEP until midnight.

#### 5.1.4 Mid-Day Schedule Update

When the app pushes a new schedule while the watch is active:
1. Deserialize and validate the new schedule — this is the **proposed** schedule.
2. Run the **commitment-integrity diff gate** (§9.3): tightening changes and new events are accepted into the stored schedule immediately; loosening changes to settled events are quarantined into the pending queue instead of applied. (Until §9 ships, per the §9.9 phasing, this step degrades to Phase-1 active-event protection, and before that to a straight replacement.)
3. Store the resulting schedule to flash.
4. Call `recalculate_day(today)`.
5. Cancel all existing RTC callbacks.
6. Re-arm RTC callbacks for all future event boundaries in the updated schedule. Boundaries whose time has already passed today are skipped.
7. Check whether current time falls in an active event window and enter or exit ENFORCEMENT accordingly.

#### 5.1.5 Midnight Initialization

Every day at midnight (00:00:00 local time), regardless of current state:
1. Call `recalculate_day(new date)`.
2. Cancel all existing RTC callbacks.
3. Arm RTC callbacks for every `startTime` and `endTime` boundary in today's event list.
4. Arm an RTC callback for the following midnight.

---

### 5.2 Worn Detection

The watch determines whether it is being worn using the ITR8307 reflective IR sensor: an IR LED emitter (GPIO 1) and a phototransistor receiver read on the ADC (GPIO 0). Skin held against the sensor reflects emitted IR back into the phototransistor, raising the measured reflection level.

Each sample pulses the emitter and subtracts an ambient baseline: the receiver ADC is read once with the emitter **off** (ambient IR) and once with it **on** (lit), and the worn decision is based on the difference `lit - ambient`. This rejects ambient IR (sunlight, indoor lighting) and keeps the emitter off except during the brief sample window.

#### Constants (all values must be defined as named constants for easy tuning)

```
IR_WORN_THRESHOLD              // Minimum ambient-subtracted ADC delta (lit - ambient) that
                                // counts as worn. Tune empirically. Default: 300.
IR_WORN_HIGHER_MEANS_WORN      // Circuit polarity. true: skin contact raises the delta
                                // (non-inverting readout); false: skin contact lowers it.
                                // Set per hardware testing. Default: true.
IR_EMIT_SETTLE_US              // Emitter rise/settle time before each ADC read. Default: 500us.
IR_WORN_DEBOUNCE_SAMPLES       // Number of consecutive consistent readings required to change
                                // worn state. Default: 5.
IR_WORN_SAMPLE_INTERVAL_MS     // Time between sensor samples during debounce. Default: 1000ms.
                                // At 5 samples × 1000ms, this gives a 5-second debounce.
```

#### Logic

The worn state is a boolean, initially assumed `false` on boot.

```
function sample_reflection() -> int:
    // Pulse the emitter and return the ambient-subtracted reflection level.
    set_gpio(IR_EMIT, LOW)
    delay_us(IR_EMIT_SETTLE_US)
    ambient = adc_read(IR_REC)
    set_gpio(IR_EMIT, HIGH)
    delay_us(IR_EMIT_SETTLE_US)
    lit = adc_read(IR_REC)
    set_gpio(IR_EMIT, LOW)
    return lit - ambient

function sample_worn_state() -> bool:
    reflection = sample_reflection()
    if IR_WORN_HIGHER_MEANS_WORN:
        return reflection >= IR_WORN_THRESHOLD
    else:
        return reflection <= -IR_WORN_THRESHOLD

function update_worn_state():
    // This function is called on a timer every IR_WORN_SAMPLE_INTERVAL_MS.
    // It maintains a rolling buffer of the last IR_WORN_DEBOUNCE_SAMPLES readings.
    append sample_worn_state() to rolling_buffer
    if all values in rolling_buffer are the same:
        new_worn = rolling_buffer[0]
        if new_worn != current_worn_state:
            current_worn_state = new_worn
            on_worn_state_changed(new_worn)

function on_worn_state_changed(is_worn: bool):
    if is_worn:
        // Watch put on: send WATCH_WORN to relevant anchors if in enforcement
        if activity_state == ENFORCEMENT and active_event.beepAnchors is not empty:
            send_watch_worn_to_anchors(active_event)
    else:
        // Watch taken off: send WATCH_REMOVED to relevant anchors if in enforcement
        if activity_state == ENFORCEMENT and active_event.beepAnchors is not empty:
            send_watch_removed_to_anchors(active_event)
```

**Note:** The debounce applies symmetrically to both the removal and re-engagement transitions. A 5-second sustained reading change is required in both directions before the worn state changes.

See §5.4.4 for the **window-start worn check** (beep-anchors are notified immediately if the watch is already unworn when a window begins — transitions alone are not sufficient) and the **donning-grace** mechanic.

---

### 5.3 Schedule System

#### 5.3.1 Storage

The schedule is stored in two separate structures in flash/NVS:

- `recurring_events`: A list of `Event` structs where `recurrenceType != once`.
- `specific_events`: A map from date (YYYY-MM-DD string key) to a list of `Event` structs where `recurrenceType == once`.

Both are written atomically when a schedule push is received from the app.

#### 5.3.2 recalculate_day(date) -> List\<Event\>

This function computes the full ordered event list for a given date. It is the single source of truth for what events are active on any day.

```
function recalculate_day(date: Date) -> List<Event>:

    // Step 1: Get recurring events that apply to this date
    recurring = get_recurring_events(date)

    // Step 2: Get specific (one-time) events for this date
    specific = specific_events[date] ?? []

    // Step 3: Merge. Specific events override recurring events with the same UUID.
    merged = {}  // map from UUID to Event
    for event in recurring:
        merged[event.id] = event
    for event in specific:
        merged[event.id] = event  // overwrites if UUID matches

    // Step 4: Remove negated events
    result = [event for event in merged.values() if not event.negate]

    // Step 5: Sort by startTime ascending
    result.sort(by: event.startTime)

    return result
```

#### 5.3.3 get_recurring_events(date) -> List\<Event\>

```
function get_recurring_events(date: Date) -> List<Event>:
    result = []
    for event in recurring_events:
        if event.recurrenceType == daily:
            result.append(event)
        else if event.recurrenceType == weekly:
            if day_of_week(date) == event.dayOfWeek:
                result.append(event)
        else if event.recurrenceType == monthly:
            if day_of_month(date) == event.dayOfMonth:
                result.append(event)
    return result
```

#### 5.3.4 Negate Mechanic

A specific (one-time) event with `negate = true` and a UUID matching a recurring event causes that recurring event to be deleted for that one day only. The negating event itself is also removed (Step 4 above). The net result is the recurring event simply does not appear in that day's schedule.

#### 5.3.5 Timezone

The watch stores a `timezone_offset_minutes` value (int16, e.g. `-300` for UTC-5). All `startTime` and `endTime` fields in Event structs are in local time (minutes since local midnight). All Unix timestamps in the schedule are UTC. The watch converts between UTC and local time using `timezone_offset_minutes` when computing RTC wakeup targets and when determining the current local time.

The timezone value is pushed by the mobile app as part of the Settings characteristic write (Section 5.6).

---

### 5.4 Enforcement

#### 5.4.1 Condition Checking

During ENFORCEMENT state, the watch checks whether the active event's criteria is currently satisfied.

**Anchor-based criteria (`getAway`, `stayNear`) — Proximity Query **(proximity.cpp):**

1. Collect a `ProxScanVector` by performing a combined BLE + WiFi scan (see "BLE scan strategy" and "WiFi during enforcement" below). Include all observed BLE devices and WiFi APs, not just Impulse anchors.
2. Look up the `AnchorRecord` for `event.anchorId` to obtain its `bleMac`.
3. Initiate a BLE connection to that anchor using `bleMac`. Negotiate MTU to `BLE_REQUESTED_MTU` immediately after connection.
4. Write the serialised `ProxScanVector` to `ANCHOR_PROX_VECTOR_CHAR_UUID`.
5. Read `ANCHOR_PROX_SCORE_CHAR_UUID` to obtain the `ProxScoreResult`.
6. Disconnect from the anchor.
7. Store `result.score` in `anchor_record.lastProxScore`.
8. Interpret the score:
   - If `result.score >= PROX_CONFIDENCE_THRESHOLD_U8` → watch is confidently **NEAR**.
   - If `result.score <= (255 - PROX_CONFIDENCE_THRESHOLD_U8)` → watch is confidently **AWAY**.
   - Otherwise → **AMBIGUOUS**. Apply the fail-safe rule:
     - For `stayNear`: treat ambiguous as **NEAR** (avoids false-positive enforcement).
     - For `getAway`: treat ambiguous as **AWAY** (avoids false-positive enforcement).
9. For `stayNear`: condition is met if watch is NEAR or AMBIGUOUS.
10. For `getAway`: condition is met if watch is AWAY or AMBIGUOUS.

If the BLE connection to the anchor fails or times out, treat as AMBIGUOUS and apply the fail-safe rule above. Do not retry in the same poll cycle; the next scheduled poll or motion interrupt will retry.

**Phone-distance criterion (`phoneAway`) — Mode B **(proximity.cpp):**

`phoneAway` is a phone-distance commitment: the user's phone is docked at `anchorId`, so proximity to that anchor is a proxy for proximity to the phone. It reuses the exact anchor proximity query above (steps 1–8), reads the anchor's **Dock Status** in the same connection (§4.11), and fuses the two:

1. Run the proximity query to `anchorId`, yielding NEAR / AWAY / AMBIGUOUS, and read the Dock Status characteristic → `docked` ∈ {docked, undocked, unknown}.
2. Compute **`near_phone`** = `undocked` **OR** (`prox == NEAR`). The phone is "with the user" whether the user went to the dock (NEAR) *or* the phone left the dock (undocked). `unknown` docking (anchor didn't report, e.g. old firmware or read failure) is treated as *docked* — a dock read-failure must not by itself trigger enforcement (fail-open on the dock signal).
3. **Fail-open (reliability rule).** Unlike `getAway`/`stayNear`, if the watch↔anchor query itself is AMBIGUOUS (degraded/failed link) the result resolves to **compliant** — the product must never fire a phone-distance alarm on an uncertain connection (overview §3.2/§8). Only a confident `near_phone` is actionable.
4. **Tolerance.** A confident `near_phone` does not enforce immediately — "a quick check is fine." The watch records the wall-clock time it first went NEAR (`g_phone_near_since_ts`, using `time()` so the grace survives enforcement light sleep). The condition is met (compliant) until `near_phone` has held continuously for `PHONE_AWAY_TOLERANCE_S` (default 60 s); past that, enforcement begins. Any not-`near_phone` reading resets the grace timer. (A brief undock — picking the phone up to glance at it — is therefore tolerated exactly like a brief approach.)

The anchor treats a `phoneAway` window like any other event for beeping/servo purposes; the distinct interpretation lives on the watch, aside from the docking detection the anchor performs (§4.11).

**WiFi-based criteria (`getOnWifi`, `getOffWifi`):**

1. Check current WiFi connection status.
2. For `getOnWifi`: condition is met if `wifi_connected == true` and current SSID matches `event.wifiSSID`.
3. For `getOffWifi`: condition is met if `wifi_connected == false` or current SSID does not match `event.wifiSSID`.

**Polling:**

- The condition is checked on a timer every `ENFORCEMENT_POLL_INTERVAL_S` seconds (default: 30).
- The condition is also checked immediately when the IMU motion interrupt fires on GPIO 4, as significant user movement may mean the condition has just been satisfied.
- When the condition transitions from not-met to met: all motor/buzzer output stops immediately.
- When the condition transitions from met to not-met: the event's enforcement profile begins (or resumes) from its beginning.

**BLE scan strategy during enforcement:**

BLE activity during ENFORCEMENT is driven by motion, not by a fixed timer, to avoid running the radio while the user is stationary.

- **WiFi-based events** (`getOnWifi`, `getOffWifi`): No BLE activity is performed. The enforcement condition is determined entirely from the WiFi connection state.
- **Anchor-based events** (`getAway`, `stayNear`):
  - When the IMU motion interrupt fires, the watch immediately collects a `ProxScanVector` and performs a proximity query (steps 1–10 above) before evaluating the condition.
  - A fallback timer (`ENFORCEMENT_BLE_FALLBACK_SCAN_INTERVAL_S`, default: 300 s) ensures a proximity query is still performed periodically even when the user is completely still.
  - `BLE_SCAN_INTERVAL_S` applies only in DORMANT state; it is not used during ENFORCEMENT.
  - The BLE scan phase of the vector collection uses a bounded duration of `ENFORCEMENT_SCAN_DURATION_MS` milliseconds (default: 300 ms). DORMANT scans continue to use duration 0 (indefinite) for thorough anchor discovery.
  - The scan uses a **reduced duty cycle**: scan interval `ENFORCEMENT_SCAN_INTERVAL_MS` (default: 1000 ms) with active window `ENFORCEMENT_SCAN_WINDOW_MS` (default: 100 ms), giving a ~10% duty cycle. DORMANT uses 200 ms / 180 ms (90%). The scanner is reconfigured to DORMANT settings on exit from ENFORCEMENT.

**WiFi during enforcement:**

- **Anchor-based events** (`getAway`, `stayNear`): WiFi remains connected. The proximity engine requires WiFi AP RSSI values in the scan vector, so disconnecting WiFi during anchor-based enforcement is no longer done. The radio is put into **modem sleep** (`WIFI_PS_MAX_MODEM`) between active operations to reduce idle current draw. A WiFi AP scan (`esp_wifi_scan_start()`) is performed at the start of each proximity query cycle to refresh AP RSSI values.
- **WiFi-based events** (`getOnWifi`, `getOffWifi`): WiFi is kept connected since the enforcement condition depends on it, with modem sleep as above.
- The `wifi_suspended_for_enforcement` flag is removed from the firmware. No automatic WiFi disconnection occurs on enforcement entry.

#### 5.4.2 Enforcement Light Sleep

When the enforcement condition is met and the user has been still long enough, the watch enters a light sleep to conserve battery. The watch **remains in `STATE_ENFORCEMENT`** throughout — this is not a transition to `DORMANT_SLEEP`.

**Entry conditions (all must be true):**
- `condition_met == true` (no buzzer/motor output needed)
- `data_ready == false` (no pending IMU interrupt)
- No BLE scan currently in progress
- `(millis() - last_activity_ms) > ENFORCEMENT_IDLE_BEFORE_SLEEP_MS`

**Sleep duration:** `ENFORCEMENT_POLL_INTERVAL_S` seconds, capped at the number of seconds remaining until the event's `endTime`, so the event never ends late.

**Wakeup sources:** identical to `DORMANT_SLEEP` — RTC timer, IMU motion on GPIO 4, and BLE connection if `ADVERTISE_DURING_SLEEP` is enabled.

**On wakeup:**
1. Re-arm the IMU ISR.
2. Reset `last_enf_poll_ms` to force an immediate condition re-check on the next loop iteration.
3. For anchor-based events: collect a `ProxScanVector` (BLE scan `ENFORCEMENT_SCAN_DURATION_MS` + WiFi AP scan) and perform a proximity query so the upcoming condition check has a current score.
4. Resume normal enforcement loop.

---

#### 5.4.3 Enforcement Profiles

Enforcement profiles define the pattern of motor and/or buzzer output while the condition is not met. Each profile is a state machine that loops until the condition is met or the event ends.

Profiles are implemented as a data-driven table. Adding a new profile requires only adding a new entry to this table — no new control-flow code.

**Profile table:**

```
struct ProfileStep {
    bool    motor_on;
    bool    buzzer_on;
    uint32  duration_ms;
}

struct ProfileDefinition {
    EnforcementProfile  id;
    bool                loops;
    List<ProfileStep>   steps;
    uint32?             floor_interval_ms;  // If not null, the wait step shrinks by
                                            // INTERVAL_DECREMENT_MS each cycle until
                                            // it reaches floor_interval_ms.
}

const INTERVAL_DECREMENT_MS = 2000;  // 2 seconds, applies to all escalating profiles
```

**Strict Silent:**
```
steps: [
    { motor_on: true, buzzer_on: false, duration_ms: CONTINUOUS }
]
loops: false  // single continuous step
```
"CONTINUOUS" means: motor stays on until condition is met; no looping.

**Normal Silent:**
```
steps: [
    { motor_on: true,  buzzer_on: false, duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },  // wait, decrements by 2s each cycle
]
loops: true
floor_interval_ms: 5000  // wait step floors at 5 seconds
```

**Loose Silent:**
```
steps: [
    { motor_on: true,  buzzer_on: false, duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },  // wait, decrements by 2s each cycle
]
loops: true
floor_interval_ms: 10000  // wait step floors at 10 seconds
```

**Strict Both:**
```
steps: [
    { motor_on: true, buzzer_on: true, duration_ms: CONTINUOUS }
]
loops: false
```

**Normal Both:**
```
steps: [
    { motor_on: true,  buzzer_on: true,  duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },
]
loops: true
floor_interval_ms: 5000
```

**Loose Both:**
```
steps: [
    { motor_on: true,  buzzer_on: true,  duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },
]
loops: true
floor_interval_ms: 10000
```

**Strict Buzz:**
```
steps: [
    { motor_on: false, buzzer_on: true, duration_ms: CONTINUOUS }
]
loops: false
```

**Normal Buzz:**
```
steps: [
    { motor_on: false, buzzer_on: true,  duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },
]
loops: true
floor_interval_ms: 5000
```

**Loose Buzz:**
```
steps: [
    { motor_on: false, buzzer_on: true,  duration_ms: 2000 },
    { motor_on: false, buzzer_on: false, duration_ms: 30000 },
]
loops: true
floor_interval_ms: 10000
```

**Escalating wait step logic (for all looping profiles):**

Each time through the loop, the wait step's duration is reduced by `INTERVAL_DECREMENT_MS`. This reduction accumulates across cycles. When the wait duration would fall below `floor_interval_ms`, it is clamped to `floor_interval_ms` and stays there.

Example for Normal Silent:
- Cycle 1: vibrate 2s, wait 30s
- Cycle 2: vibrate 2s, wait 28s
- Cycle 3: vibrate 2s, wait 26s
- ...
- Cycle 14: vibrate 2s, wait 5s
- Cycle 15+: vibrate 2s, wait 5s (floored)

---

#### 5.4.4 Donning Grace & Window-Start Worn Handling

Two rules (added in v0.6) that make wake-up-style events (Sunrise Lock) work. Both are generic Event mechanics, not a special mode.

**Window-start worn check:** on entering ENFORCEMENT for an event with non-empty `beepAnchors`, the watch immediately evaluates the current worn state. If the watch is **not worn** at window start, it sends WATCH_REMOVED to the event's `beepAnchors` right away — it does not wait for a worn *transition*. (Rationale: a watch charging on the nightstand has been unworn all night; the transition-based path in §5.2 alone would never fire, and the anchor would stay silent through the wake-up window.) The anchor needs no new behavior: it already validates the event window on receipt (§4.6).

**Donning grace (`donningGraceS`):** if an event has `donningGraceS > 0`, the watch suppresses its own enforcement for that many seconds after the watch is put on:

- Compute `grace_deadline = donned_at + donningGraceS`, where `donned_at` is the time of the most recent not-worn → worn transition — or the window start, if the watch is already worn when the window begins (donning the watch just before the window shouldn't be punished).
- While `now < grace_deadline`, the enforcement condition check (§5.4.1) short-circuits to **met** (the same pattern as the `phoneAway` tolerance): no motor/buzzer output, and the watch may enter enforcement light sleep as usual (cap the sleep at the grace deadline so expiry is not slept through). Anchor beeping is independently quieted by the WATCH_WORN transition (§5.2).
- When the grace expires, force an immediate condition re-check — do not wait for the next poll boundary.
- Taking the watch off during grace cancels it (and triggers WATCH_REMOVED as normal); putting it back on starts a fresh grace period.

**Worked example** (Sunrise Lock: `getAway` from the bedroom anchor, beep on the nightstand anchor, `donningGraceS = 300`): at wake time the window opens with the watch on the charger → unworn at window start → the nightstand anchor sounds. The user puts the watch on → WATCH_WORN quiets the anchor and the 300 s grace begins. Still in the bedroom when the grace ends → the watch's `getAway` enforcement starts (vibration/buzzer per profile), and removing the watch again re-triggers the anchor. Out of the bedroom → condition met → quiet. Every parameter is user-set the night before.

**Classification (§9.1):** decreasing `donningGraceS` is a tightening; increasing it is a loosening.

---

### 5.5 Anchor Communication (Watch Side)

#### 5.5.1 Sending Watch-Removed / Watch-Worn

When `on_worn_state_changed` fires during ENFORCEMENT for an event with non-empty `beepAnchors`:

```
function send_watch_removed_to_anchors(event: Event):
    command = WATCH_REMOVED (0x01)
    send_to_anchors(command, event)

function send_watch_worn_to_anchors(event: Event):
    command = WATCH_WORN (0x02)
    send_to_anchors(command, event)

function send_to_anchors(command: uint8, event: Event):
    payload = encode_udp_command(command, watch_uuid, event.id)
    for anchor_uuid in event.beepAnchors:
        record = anchor_records[anchor_uuid]
        success = false

        // Attempt 1: direct IP (from app-provided AnchorRecord)
        if record.ipAddress != null:
            success = udp_send(record.ipAddress, ANCHOR_UDP_PORT, payload,
                               retries: UDP_RETRY_COUNT,
                               timeout_ms: UDP_RETRY_TIMEOUT_MS)

        // Attempt 2: mDNS fallback
        if not success:
            ip = mdns_resolve(anchor_uuid + ".local",
                              timeout_ms: MDNS_RESOLVE_TIMEOUT_MS)
            if ip != null:
                success = udp_send(ip, ANCHOR_UDP_PORT, payload,
                                   retries: UDP_RETRY_COUNT,
                                   timeout_ms: UDP_RETRY_TIMEOUT_MS)

        // Attempt 3: UDP broadcast
        if not success:
            udp_send("255.255.255.255", ANCHOR_UDP_PORT, payload, retries: 1)
            // Broadcast is best-effort; no success tracking

        // If still unreachable after direct IP and mDNS attempts, queue notification
        if not success:
            queue_unreachable_notification(anchor_uuid, record.name)
```

#### 5.5.2 Unreachable Anchor Notifications

When an anchor cannot be reached, the watch queues a notification:

```
struct UnreachableNotification {
    String anchor_uuid;
    String anchor_name;
    int64  timestamp;
}
```

The next time `bt_connected == true`, the watch sends all queued notifications to the app via the Watch Status GATT characteristic (see Section 5.6).

---

### 5.6 BLE GATT Service (Watch ↔ Phone)

The watch exposes one custom GATT service.

**Service UUID:** `WATCH_SERVICE_UUID` (fixed constant, shared between firmware and app).

---

#### Characteristic: WiFi Credentials (Write, No Response)

**UUID:** `WATCH_WIFI_CRED_CHAR_UUID`

**Payload (JSON, UTF-8):**
```json
{ "ssid": "<network name>", "password": "<password>" }
```

On write:
1. Attempt WiFi connection.
2. On success: save credentials to NVS credentials list, set `wifi_connected = true`.
3. On failure: save credentials to NVS credentials list anyway (for future auto-connect attempts).
4. Notify app of result via Watch Status characteristic.

The watch maintains a list of saved WiFi credential pairs in NVS. It attempts to connect to any of them when WiFi scanning finds a matching network.

---

#### Characteristics: Schedule Transfer Control & Data

**Control UUID:** `WATCH_SCHED_CTRL_CHAR_UUID` (Write, With Response)
**Data UUID:** `WATCH_SCHED_DATA_CHAR_UUID` (Write, No Response)

See Section 6.2 for the full schedule transfer protocol.

---

#### Characteristic: Settings (Write, With Response)

**UUID:** `WATCH_SETTINGS_CHAR_UUID`

**Payload (binary, little-endian):**
```
[1 byte:  DISCONNECTED_IS_DORMANT (0 or 1)]
[1 byte:  AWAY_IS_DORMANT (0 or 1)]
[2 bytes: timezone_offset_minutes (int16, signed, e.g. -300 for UTC-5)]
[2 bytes: settle_window_min (uint16, minutes; clamped to 30–240, §9.8)]
```

On write: stores all settings to NVS, responds with `0x01`. If a timezone change occurs, re-arm all RTC callbacks using the new offset.

**Gating (§9.7/§9.8):** dormancy-flag changes `true→false` and `settle_window_min` increases are loosenings — quarantined into the pending queue rather than applied; a write containing any quarantined field responds `0x03` (the other fields apply immediately). A timezone change that would place the current local time outside the **currently active** event's window is rejected with `0x02` (§9.7).

> **Lockstep note:** the payload grew from 4 to 6 bytes in spec v0.5 — update the app in the same batch as the schedule version byte.

---

#### Characteristic: Seen Anchors (Read + Notify)

**UUID:** `WATCH_SEEN_ANCHORS_CHAR_UUID`

**Payload (binary, little-endian):**
```
[1 byte:  anchor count N]
for each anchor (N entries):
    [16 bytes: UUID]
    [1 byte:   last RSSI, encoded as (rssi + 128) to make it unsigned]
    [4 bytes:  last_seen Unix timestamp (uint32)]
```

The watch sends a BLE notification on this characteristic whenever a new anchor UUID is discovered. The app can also read this characteristic at any time to get the current full list.

---

#### Characteristic: Watch Status (Notify + Read)

**UUID:** `WATCH_STATUS_CHAR_UUID`

The watch pushes a notification any time any field in this payload changes.

**Payload (binary, little-endian):**
```
[1 byte:  activity_state (0=dormant, 1=enforcement, 2=dormant_sleep)]
[1 byte:  bt_connected (0 or 1)]
[1 byte:  wifi_connected (0 or 1)]
[1 byte:  worn (0 or 1)]
[1 byte:  battery_pct (0–100; 0xFF if not yet available)]
[16 bytes: active_event_id (UUID of current enforcement event; zero-filled if none)]
[1 byte:  condition_met (1 = enforcement condition currently met, grace active, or not
          enforcing; 0 = actively alarming. Added v0.6 — lockstep.)]
[1 byte:  unreachable_anchor_count N]
for each unreachable notification (N entries):
    [16 bytes: anchor UUID]
    [1 byte:   anchor name length L]
    [L bytes:  anchor name (UTF-8)]
    [4 bytes:  timestamp (uint32 Unix)]
```

After delivering queued unreachable notifications, the watch clears them from its queue.

---

#### Characteristic: Anchor IP Table (Write, With Response)

**UUID:** `WATCH_ANCHOR_IP_CHAR_UUID`

The app writes the current known IP addresses of all anchors to this characteristic. This is how the watch learns anchor IPs without relying solely on mDNS.

**Payload (binary, little-endian):**
```
[1 byte:  entry count N]
for each entry (N entries):
    [16 bytes: anchor UUID]
    [4 bytes:  IPv4 address (uint32, network byte order)]
    [4 bytes:  timestamp of IP confirmation (uint32 Unix)]
```

On write: updates the watch's `AnchorRecord` table entries. Responds with `0x01`.

---

#### Characteristic: Time (Write, With Response)

**UUID:** `WATCH_TIME_CHAR_UUID` (`4A0F0019-F8CE-11EE-8001-020304050607`)

Lets the mobile app set the watch's absolute wall-clock time over BLE. This is required because the watch otherwise obtains time only from NTP, which needs an active WiFi connection; in WiFi-less setups the clock would never be set (schedule windows, `phoneAway` grace, and logging all depend on correct time).

**Payload (binary, little-endian):**
```
[8 bytes: utc_epoch (int64, Unix seconds, UTC)]
[2 bytes: tz_offset_minutes (int16, signed, minutes east of UTC, e.g. -300 for UTC-5)]
```

On write: the watch sets its system clock (`settimeofday`) to `utc_epoch` and updates `timezone_offset_minutes` (equivalent to the timezone field of the Settings characteristic, §5.6). It then re-arms RTC callbacks and runs `recalculate_day(today)` so schedule boundaries reflect the corrected time. Responds with `0x01`.

The app pushes time on pairing, on every app-foreground while connected, and after any timezone change. The `tz_offset_minutes` here and in the Settings characteristic must be kept consistent; a write to either updates the same stored offset.

---

#### Characteristic: LED Configuration (Write With Response, Read)

**UUID:** `WATCH_LED_CONFIG_CHAR_UUID`

The app uses this characteristic to recolour and re-tune the LED status ring at runtime (Section 5.7). Every visible status (dormant, enforcement-idle, enforcement-alarm, and the per-wake-cause flashes) is a configurable *slot*; the colour and blink timing of any slot can be changed without a firmware update. The watch persists the configuration to NVS and applies it immediately.

**Write payload (binary, little-endian):**
```
[1 byte:  global_brightness (0–255; passed to FastLED.setBrightness)]
[1 byte:  slot count N]
for each slot (N entries):
    [1 byte:  slot_id        (see LedStatusSlot enum, Section 5.7)]
    [1 byte:  R]
    [1 byte:  G]
    [1 byte:  B]
    [2 bytes: blink_on_ms    (uint16; 0 = steady, no blink)]
    [2 bytes: blink_off_ms   (uint16; ignored when blink_on_ms == 0)]
```

A write may carry any subset of slots (`N` from 0 to the number of slots); unlisted slots keep their current values. A slot listed with `blink_on_ms == 0` is rendered as a steady colour. `global_brightness` is always applied. Unknown `slot_id` values are skipped.

On write: clamp/store the listed slots and brightness to NVS, re-render the ring against the current status, respond `0x01`. Respond `0x00` if the payload is malformed (length shorter than `2 + N*8`).

**Read payload:** the current full configuration, in the same layout as a write that lists **every** slot (i.e. `N` = total slot count), so the app can read back the live colours.

---

#### Characteristic: Pending Changes (Read + Notify) **(commitment integrity, §9.5)**

**UUID:** `WATCH_PENDING_CHAR_UUID` (`4A0F001A-F8CE-11EE-8001-020304050607`)

Exposes the watch's authoritative queue of quarantined loosening changes (§9.3–9.5), so the app can render pending state truthfully and recover it after a reinstall.

**Payload (little-endian):**
```
[1 byte: pending count N]
for each entry (N entries):
    [16 bytes: event UUID (zero-filled for non-event entries)]
    [1 byte:   change type (0=delete, 1=loosen-modify, 2=negate-day, 3=setting change)]
    [4 bytes:  seconds until apply (uint32, elapsed-time basis)]
```

Notifies whenever the queue changes (entry added, promoted, or rejected).

---

#### Characteristic: Emergency Pass (Write With Response + Read) **(commitment integrity, §9.6)**

**UUID:** `WATCH_PASS_CHAR_UUID` (`4A0F001B-F8CE-11EE-8001-020304050607`)

The rolling emergency-pass ledger lives on the watch (§9.6). The app spends and inspects passes through this characteristic.

**Write payload (spend):**
```
[1 byte:   0x01 (SPEND)]
[16 bytes: event UUID]
[4 bytes:  local date as YYYYMMDD (uint32)]
```
**Response:** `0x01` + `[1 byte: passes remaining]` on success; `0x02` + `[0x00]` if the budget is exhausted; `0x00` on malformed payload.

**Write payload (set allowance):**
```
[1 byte: 0x02 (SET_ALLOWANCE)]
[1 byte: new allowance]
```
**Response:** `0x01` — applied immediately (allowance lowered or unchanged); `0x03` — quarantined (an allowance raise is a loosening → pending queue, §9.6).

**Read payload:**
```
[1 byte: allowance]
[1 byte: remaining]
for each spent pass still inside the rolling window:
    [4 bytes: seconds until it regenerates (uint32, elapsed-time basis)]
```

---

### 5.7 LED Status Indicator

The watch carries a ring of 12 SK6805 addressable LEDs (hardware: Section 1) used as a passive status indicator. It conveys, at a glance, what the watch is doing — whether it is asleep, resting, or actively enforcing — and *why* it last woke. The ring is **output-only**: nothing in this section may change enforcement, sleep, scanning, or radio behaviour. All colours and blink timings are runtime-configurable by the app via `WATCH_LED_CONFIG_CHAR_UUID` (Section 5.6) and persisted to NVS; the values below are defaults only.

The ring is driven through FastLED at `LED_RING_DEFAULT_BRIGHTNESS`, on `LED_RING_DATA_PIN` (Section 1 / Section 7). All twelve LEDs always show the same colour (the ring is a single logical indicator, not a progress dial), with the sole exception of the blink behaviour described below.

#### 5.7.1 Status Slots

Each distinct thing the ring can show is a **slot**. A slot has an RGB colour and an optional blink (`on_ms` / `off_ms`); the app addresses slots by `slot_id`.

```
enum LedStatusSlot {
    LED_SLOT_DORMANT        = 0,   // awake, DORMANT, nothing else to show       (default: blue)
    LED_SLOT_ENFORCE_IDLE   = 1,   // awake, ENFORCEMENT, condition currently met (default: orange)
    LED_SLOT_ENFORCE_ALARM  = 2,   // ENFORCEMENT, condition NOT met (alarming)   (default: red, blinking)
    LED_SLOT_WAKE_MOTION    = 3,   // momentary flash: woke on IMU motion         (default: green)
    LED_SLOT_WAKE_TIMER     = 4,   // momentary flash: woke on RTC/timer          (default: light blue)
    LED_SLOT_WAKE_BLE       = 5,   // momentary flash: woke on incoming BLE        (default: white)
}
```

**Default slot table** (RGB before global brightness scaling):

| Slot                   | Default RGB     | Blink (on/off ms) | Meaning                                                     |
|------------------------|-----------------|-------------------|------------------------------------------------------------|
| `LED_SLOT_DORMANT`     | `0,   0,   255` | steady            | Awake and dormant (the nominal resting "I'm here" colour).  |
| `LED_SLOT_ENFORCE_IDLE`| `255, 90,  0`   | steady            | In an enforcement window, condition met — no output needed. |
| `LED_SLOT_ENFORCE_ALARM`| `255, 0,   0`  | `250 / 250`       | In enforcement, condition not met — actively enforcing.     |
| `LED_SLOT_WAKE_MOTION` | `0,   255, 0`   | steady*           | Just woke from light sleep because of motion.               |
| `LED_SLOT_WAKE_TIMER`  | `0,   180, 255` | steady*           | Just woke from light sleep on the RTC/event timer.          |
| `LED_SLOT_WAKE_BLE`    | `255, 255, 255` | steady*           | Just woke from light sleep on an incoming BLE packet.       |

\* Wake-cause slots are shown as a momentary flash of `LED_WAKE_FLASH_MS` (Section 7), not continuously; see resolution rules below.

#### 5.7.2 Rendering Resolution

The ring shows exactly one slot at any instant. The watch re-evaluates which slot to display on every relevant event (sleep entry, wake, activity-state change, enforcement condition transition) using this fixed priority — highest first:

1. **Asleep → ring OFF.** Whenever the watch is in `DORMANT_SLEEP` or enforcement light sleep (Section 5.4.2), all LEDs are cleared (`FastLED.clear()` + `show()`) *immediately before* the `esp_light_sleep_start()` call. "Off" means every pixel black; there is no separate power-enable line (GPIO 21 is the buzzer and is owned by existing firmware — the LED feature must not touch it). The ring draws only its quiescent leakage while asleep.

2. **Active enforcement alarm.** While `activity_state == ENFORCEMENT` **and** the enforcement condition is **not met** (i.e. a profile pattern from Section 5.4.3 is running), the ring shows `LED_SLOT_ENFORCE_ALARM`, blinking per its `on_ms`/`off_ms`. This overrides the steady state colour so the visual alarm tracks the buzzer/motor alarm. The instant the condition transitions back to met, the ring drops to priority 4.

3. **Wake-cause flash (momentary).** On any transition *out of* light sleep into an awake state, the watch flashes the slot for the wake source that fired, for `LED_WAKE_FLASH_MS`:
   - IMU motion interrupt (GPIO 4) → `LED_SLOT_WAKE_MOTION` (green).
   - RTC / event-boundary timer → `LED_SLOT_WAKE_TIMER` (light blue).
   - Incoming BLE packet/connection → `LED_SLOT_WAKE_BLE` (white).

   The flash is non-blocking: it does not delay scanning, the proximity query, or condition evaluation — it is a colour the renderer holds until `LED_WAKE_FLASH_MS` has elapsed, after which the ring falls through to priority 4 (or to priority 2 if by then the watch is in an active alarm). If the watch goes back to sleep before the flash elapses, priority 1 wins and the ring goes off.

   Note on wake sources by state: per Section 8.4, motion is a wake source **only during ENFORCEMENT**; in `DORMANT_SLEEP` the watch wakes on the RTC timer (and BLE, when reachable) only. The renderer simply colours whichever source actually woke it, so a dormant wake is normally light blue (timer) or white (BLE), and a green motion flash appears only inside enforcement.

4. **Steady activity colour.** When awake with none of the above active, the ring shows the colour for the current activity state:
   - `DORMANT` → `LED_SLOT_DORMANT` (blue).
   - `ENFORCEMENT`, condition met → `LED_SLOT_ENFORCE_IDLE` (orange).
   - `UNPAIRED` → ring off (no status to show until paired). This is a rendering choice, not a separate slot.

So a typical enforcement wake looks like: green flash (motion) → settles to orange (in-window, compliant) → turns red-blinking the moment the condition is found not met → back to orange when the user complies → off when the watch light-sleeps again. A typical dormant wake looks like: light-blue flash (timer) → blue (dormant) → off on return to sleep.

#### 5.7.3 Implementation Notes

- The renderer is a single function, e.g. `led_render(LedStatusSlot active_slot)`, plus a small `led_update()` ticked from the main loop that (a) resolves the current slot via the priority rules, (b) advances the alarm blink phase, and (c) expires the wake-cause flash. No status logic lives in the BLE or enforcement modules — they call `led_note_wake(cause)` / `led_note_state_change()` and let the renderer decide.
- Slot colours, brightness, and blink timings load from NVS on boot (falling back to the defaults above) before the first render, so the very first wake already reflects any app customisation.
- The ring must be cleared before *every* light-sleep entry path (DORMANT_SLEEP and enforcement light sleep) to avoid leaving an LED lit through sleep; this is the only hard ordering requirement relative to the existing sleep code.

#### 5.7.4 Analog clock display mode

Because the 12-LED ring maps naturally onto a clock face (one LED per hour position), the watch can render the current time directly on the ring via `led_show_time(hour, minute)`. This is an opt-in display, separate from the status-slot resolver above (Section 5.7.2) — it paints the ring explicitly rather than through `led_update()`, and the next status tick repaints over it.

- LED index → clock position: the ring is wired so index `0` is the 12-o'clock LED's neighbour; the renderer maps a clock position `p` (0 = 12 o'clock) to LED index `11 - p`, so positions advance clockwise around the physical ring.
- **Hour hand:** the LED at `hour % 12` lit **red**.
- **Minute hand:** the LED at `minute / 5` (nearest 5-minute mark) lit **green**.
- **Overlap:** when the hour and minute hands fall on the same LED, that single LED is lit **yellow** instead of stacking colours.
- All other LEDs are off. Brightness uses the configured ring brightness. The hand colours are currently fixed in code (not yet exposed as configurable slots).

---

## 6. Inter-Device Communication Protocols

### 6.1 UDP Command Packet (Watch → Anchor)

All UDP messages from the watch to anchors use this fixed 33-byte format:

```
[1 byte:  command]
    0x01 = WATCH_REMOVED
    0x02 = WATCH_WORN
[16 bytes: watch UUID]
[16 bytes: event UUID]
```

No response is expected from the anchor. The watch uses the retry logic defined in Section 5.5.1.

### 6.2 Schedule Transfer Protocol

Used for both BLE (watch) and HTTP (anchor). The schedule is encoded as a binary blob and transferred using a 3-phase protocol.

#### Phase 1: BEGIN

Write to the control characteristic / HTTP endpoint with:
```
[1 byte:  0x01 (BEGIN)]
[4 bytes: total data length in bytes (uint32, little-endian)]
```

#### Phase 2: DATA

Write sequential chunks to the data characteristic. Each chunk is at most `BLE_MTU_BYTES - 3` bytes (to fit BLE ATT packet). For HTTP, the body is the full blob in one request.

**Schedule binary encoding (little-endian):**
```
[1 byte:   format version (currently 0x02; receivers reject unknown versions)]
[2 bytes:  event count N (uint16)]
for each Event (N entries):
    [16 bytes: UUID (raw bytes)]
    [8 bytes:  referenceDate (int64 Unix timestamp)]
    [2 bytes:  startTime (uint16, minutes since midnight)]
    [2 bytes:  endTime (uint16, minutes since midnight)]
    [1 byte:   recurrenceType]
    [1 byte:   dayOfWeek (0 if unused)]
    [1 byte:   dayOfMonth (0 if unused)]
    [1 byte:   criteria]
    [1 byte:   enforcementProfile]
    [1 byte:   anchorProfile (0xFF if null)]
    [1 byte:   negate (0 or 1)]
    [2 bytes:  donning_grace_s (uint16 seconds; 0 = none. See §5.4.4. Added v0.6, same lockstep batch)]
    [1 byte:   anchorId presence flag (0=absent, 1=present)]
    [16 bytes: anchorId UUID (zero-filled if absent)]
    [1 byte:   wifiSSID length L (0 if absent)]
    [L bytes:  wifiSSID (UTF-8, omitted if L=0)]
    [1 byte:   beepAnchors count M]
    for each beepAnchor (M entries):
        [16 bytes: anchor UUID]
```

#### Phase 3: END

Write to the control characteristic with:
```
[1 byte:  0x02 (END)]
[4 bytes: CRC32 of the full data blob (uint32, little-endian)]
```

On receiving END:
- Compute CRC32 of the received buffer.
- If mismatch: discard buffer, respond `0x00`. App must retry from BEGIN.
- If matches: check the leading format version byte (unknown version → discard, respond `0x00`), then:
  - **Anchor:** deserialize, persist per §4.7, respond `0x01`.
  - **Watch:** run the §9.3 commitment-integrity diff gate and respond:
    - `0x01` — accepted in full.
    - `0x03` — accepted, but one or more loosening changes were quarantined into the pending queue (app should read the Pending Changes characteristic, §9.5).
    - `0x04` — rejected in full: the push would loosen the **currently active** event (§9.3 Phase-1 rule; under the full diff gate such changes are quarantined and reported via `0x03` instead).

> **Version-byte note (lockstep):** the leading version byte was added in spec v0.5. A v1 parser would misread it as the low byte of the event count, so the app's `ScheduleEncoder`, the watch, and the anchor must all update together — coordinate the flash.

**ABORT command:**
```
[1 byte: 0x03 (ABORT)]
```
Discards any in-progress transfer immediately.

#### HTTP endpoint for anchors

The app pushes the schedule to each anchor at:

```
POST http://<anchor-ip>/schedule
Content-Type: application/octet-stream
Body: <full schedule blob (no framing, entire blob in body)>
```

The anchor responds:
- `200 OK` with body `0x01` on success.
- `400 Bad Request` with body `0x00` on CRC or parse failure.

The CRC is appended to the HTTP body as the final 4 bytes (little-endian uint32) after all event data.

---

### 6.3 Proximity Protocols **(proximity.cpp)**

#### 6.3.1 Proximity Vector Payload (Watch → Anchor)

Written by the watch to `ANCHOR_PROX_VECTOR_CHAR_UUID`. All fields little-endian.

```
[1 byte:  device count N  (0 to PROX_MAX_DEVICES)]
for each device (N entries):
    [6 bytes: mac    (BLE MAC address or WiFi BSSID, big-endian as seen on-air)]
    [1 byte:  type   (0x00 = BLE device, 0x01 = WiFi AP)]
    [1 byte:  rssi   (signed int8, dBm; cast to uint8 for wire: rssi_wire = rssi + 128)]
Total: 1 + N × 8 bytes
Maximum at PROX_MAX_DEVICES=60: 481 bytes — fits within a 512-byte negotiated MTU.
```

The watch assembles the vector immediately before initiating the GATT connection. It includes every BLE device and WiFi AP observed during the preceding scan cycle. If the total device count exceeds `PROX_MAX_DEVICES`, the watch truncates to the `PROX_MAX_DEVICES` entries with the strongest RSSI values. If the negotiated MTU is below `PROX_MIN_MTU_BYTES`, the watch further truncates the vector to fit within `negotiated_mtu - 3` bytes before transmitting; the firmware logs a warning.

The anchor deserialises RSSI values as: `rssi_dbm = (int8)(rssi_wire - 128)`.

#### 6.3.2 Fingerprint Blob (App → Anchor)

Transmitted by the mobile app to the anchor using the BEGIN / DATA / END protocol (Section 6.2) via `ANCHOR_FINGERPRINT_CTRL_CHAR_UUID` and `ANCHOR_FINGERPRINT_DATA_CHAR_UUID`. The CRC32 covers the entire blob body (everything after the BEGIN frame). All fields little-endian.

```
[2 bytes:  device count N  (uint16, 0 to ANCHOR_PROX_MAX_FINGERPRINT_DEVICES)]
for each device (N entries):
    [6 bytes: mac       (BLE MAC or WiFi BSSID)]
    [1 byte:  type      (0x00 = BLE, 0x01 = WiFi AP)]
    [4 bytes: mu        (float32, mean RSSI in dBm)]
    [4 bytes: sigma_sq  (float32, variance in dBm²; must be > 0)]
    [4 bytes: W         (float32, cumulative Welford weight; 0.0 if no samples yet)]
Total per entry: 19 bytes
Total blob: 2 + N × 19 bytes
Example at N=60: 1142 bytes — requires chunked transfer.
```

On deserialisation, the anchor replaces its device registry with the N entries in order. Any previous self-supervised data is discarded. Entries with `W == 0.0` are loaded into the registry but are excluded from Signal B (fingerprint log-likelihood) until `W >= PROX_MIN_FINGERPRINT_WEIGHT`; they still participate in Signal A (live correlation) immediately.

---

## 7. Constants & Tunables

All of the following must be defined as named constants in the firmware. Values marked "(tunable)" are expected to require adjustment during hardware testing.

```
// Timing
ENFORCEMENT_POLL_INTERVAL_S              = 30   // (tunable) seconds between condition checks
BLE_SCAN_INTERVAL_S                      = 30   // seconds between BLE anchor scans in DORMANT state only
ENFORCEMENT_BLE_FALLBACK_SCAN_INTERVAL_S = 300  // (tunable) fallback proximity query interval during ENFORCEMENT
                                                 // for anchor-based events when no motion is detected
ENFORCEMENT_SCAN_DURATION_MS             = 300  // (tunable) BLE scan duration for vector collection during
                                                 // enforcement; 300 ms covers 3 anchor advertisement windows
ENFORCEMENT_IDLE_BEFORE_SLEEP_MS         = 3000 // (tunable) ms of no motion before entering enforcement sleep
WIFI_SCAN_INTERVAL_S               = 60        // seconds between WiFi scans in dormant
DORMANT_TO_SLEEP_IDLE_MS           = 5000      // ms of inactivity before entering dormant-sleep
ANCHOR_WIFI_RETRY_INTERVAL_S       = 30        // anchor WiFi reconnect attempt interval
ANCHOR_SEEN_TIMEOUT_S              = 10        // seconds after which an anchor is considered unseen
                                               // for discovery purposes (does not affect proximity scoring)

// UDP
ANCHOR_UDP_PORT                    = 5555      // (tunable) UDP listen port on anchors
UDP_RETRY_COUNT                    = 3         // attempts before marking anchor unreachable
UDP_RETRY_TIMEOUT_MS               = 500       // ms timeout per UDP attempt
MDNS_RESOLVE_TIMEOUT_MS            = 1000      // ms timeout for mDNS resolution

// Anchor settings
ANCHOR_MAX_BEEP_MINUTES_DEFAULT    = 30        // default max continuous beeping per event
IDENTIFY_BEEP_DURATION_MS          = 800       // duration of identify beep

// Anchor servo
SERVO_CLOSED_DEGREES               = 180       // PWM angle for the closed/locked strap position
SERVO_OPEN_DEGREES                 = 160       // PWM angle for the open/unlocked strap position
SERVO_MOVE_DURATION_MS             = 500       // (tunable) time to wait after commanding angle before detaching

// Watch pairing feedback
UNPAIRED_BEEP_DURATION_MS          = 200       // beep on first BLE connection
UNPAIRED_VIBRATE_DURATION_MS       = 300       // vibrate on first BLE connection

// Worn detection (all tunable — ITR8307 behavior must be validated on hardware)
IR_WORN_THRESHOLD                  = 300       // (tunable) min ambient-subtracted ADC delta for worn
IR_WORN_HIGHER_MEANS_WORN          = true      // (tunable) circuit polarity; false if readout inverted
IR_EMIT_SETTLE_US                  = 500       // emitter rise/settle before each ADC read
IR_WORN_DEBOUNCE_SAMPLES           = 5         // consecutive samples required to change worn state
IR_WORN_SAMPLE_INTERVAL_MS         = 1000      // ms between reflection samples

// Enforcement profile escalation
INTERVAL_DECREMENT_MS              = 2000      // ms to subtract from wait step each cycle

// BLE
BLE_REQUESTED_MTU                  = 512       // MTU requested during negotiation on every GATT connection
                                               // (both watch–anchor and app–watch/anchor). The stack negotiates
                                               // down if the peer cannot support this value.
ENFORCEMENT_SCAN_INTERVAL_MS       = 1000      // (tunable) BLE scanner slot interval during enforcement
ENFORCEMENT_SCAN_WINDOW_MS         = 100       // (tunable) BLE scanner active window during enforcement (~10% duty)
MINIMUM_BLE_DELAY_DORMANT          = 15000     // (tunable) ms to wait before re-arming the IMU interrupt in DORMANT;
                                               // prevents rapid movement from blocking entry to DORMANT_SLEEP
MINIMUM_BLE_DELAY_ENFORCEMENT      = 30000     // (tunable) ms to wait before re-arming the IMU interrupt in ENFORCEMENT;
                                               // limits how often continuous movement can trigger a fresh proximity query

// Schedule
MAX_EVENTS_PER_DAY                 = 64        // maximum events recalculate_day will return

// LED status ring (Section 5.7) — colours/timings below are NVS defaults, overridable at runtime
LED_RING_DATA_PIN                  = 10        // (hardware-rev dependent) data line for the SK6805 ring.
                                               // Current rev: GPIO 10 (vibration motor disabled by PCB switch).
                                               // Next rev: 7 (ring) with the motor restored on GPIO 10. ONLY
                                               // this constant changes between revs; never touch motor code.
LED_RING_COUNT                     = 12        // number of LEDs in the ring
LED_RING_DEFAULT_BRIGHTNESS        = 40        // (tunable) FastLED master brightness, 0–255
LED_WAKE_FLASH_MS                  = 600       // (tunable) duration of the momentary wake-cause flash before the
                                               // ring settles to the steady activity colour
LED_ALARM_BLINK_ON_MS              = 250       // (tunable) default ENFORCE_ALARM blink on-time
LED_ALARM_BLINK_OFF_MS             = 250       // (tunable) default ENFORCE_ALARM blink off-time
// Default slot colours (R, G, B) — see LedStatusSlot enum, Section 5.7
LED_DEFAULT_DORMANT_RGB            = {0,   0,   255}   // blue       — awake & dormant
LED_DEFAULT_ENFORCE_IDLE_RGB       = {255, 90,  0}     // orange     — enforcing, condition met
LED_DEFAULT_ENFORCE_ALARM_RGB      = {255, 0,   0}     // red        — enforcing, condition not met (blinks)
LED_DEFAULT_WAKE_MOTION_RGB        = {0,   255, 0}     // green      — woke on motion
LED_DEFAULT_WAKE_TIMER_RGB         = {0,   180, 255}   // light blue — woke on RTC/timer
LED_DEFAULT_WAKE_BLE_RGB           = {255, 255, 255}   // white      — woke on incoming BLE

// ── Proximity engine (proximity.cpp) ──────────────────────────────────────────
// Watch-side vector assembly
PROX_MAX_DEVICES                   = 60        // maximum BLE + WiFi entries in a ProxScanVector;
                                               // if more devices are seen, keep the 60 with strongest RSSI
PROX_MIN_DEVICE_COUNT              = 8         // minimum number of devices in a ProxScanVector to use the
                                               // proximity engine; below this the watch falls back to
                                               // single-anchor raw RSSI (Signal A only, no fingerprint)
PROX_MIN_MTU_BYTES                 = 256       // minimum negotiated MTU (bytes) for the proximity vector write;
                                               // if negotiated MTU < this value, vector is truncated to fit and
                                               // firmware logs a warning. Set to fit at least PROX_MIN_DEVICE_COUNT
                                               // entries: 3 (ATT header) + 1 (count) + 8*8 = 68 bytes minimum.

// Watch-side score interpretation
PROX_CONFIDENCE_THRESHOLD_U8       = 170       // (tunable) score threshold (0–255) above which the watch treats
                                               // the anchor as NEAR; below (255 - this value) treated as AWAY;
                                               // between the two bounds is AMBIGUOUS. 170 ≈ 0.667 of full scale.
ANCHOR_NEAR_RSSI_THRESHOLD_DBM     = -70       // (tunable) fallback RSSI cutoff used only when
                                               // PROX_FLAG_LOW_DEVICE_COUNT is set in the score result

// Anchor-side score computation
PROX_MISSING_RSSI_DBM              = -100      // sentinel RSSI (dBm) substituted for devices absent from a
                                               // watch vector during fingerprint log-likelihood computation
PROX_MIN_FINGERPRINT_WEIGHT        = 5.0       // minimum cumulative Welford weight W for a device to contribute
                                               // to Signal B (fingerprint log-likelihood)
PROX_ALPHA_W0                      = 2000.0    // (tunable) total fingerprint weight at which the blend factor α
                                               // has decayed to e^-1 ≈ 0.37 (i.e. Signal B contributes ~63%);
                                               // at W_total=0 α=1.0 (pure correlation), at W_total>>W0 α→0

// Anchor-side self-supervised training
PROX_COLLECT_SCORE_THRESHOLD       = 0.75      // (tunable) minimum blended score (0.0–1.0) required to accept a
                                               // watch vector as a self-supervised training sample
PROX_COLLECT_AMBIGUITY_MARGIN_DBM  = 10        // (tunable) minimum dBm margin by which this anchor must exceed
                                               // any competing anchor's raw RSSI (as seen in the watch vector)
                                               // for the sample to be considered unambiguous
PROX_NVS_PERSIST_INTERVAL_S        = 300       // seconds between NVS fingerprint persist operations; limits
                                               // flash wear while allowing recovery on unexpected reboot

// Anchor-side continuous background scanning
ANCHOR_PROX_BLE_SCAN_INTERVAL_MS   = 2000      // (tunable) ms between anchor BLE scan cycles
ANCHOR_PROX_BLE_SCAN_DURATION_MS   = 500       // (tunable) duration of each anchor BLE scan cycle (ms)
ANCHOR_PROX_WIFI_SCAN_INTERVAL_S   = 30        // (tunable) seconds between anchor WiFi AP scan cycles
ANCHOR_PROX_DEVICE_STALE_MS        = 10000     // ms without an RSSI update before a cache entry is marked stale
                                               // and excluded from score computations
ANCHOR_PROX_MAX_FINGERPRINT_DEVICES = 128      // maximum entries in the anchor device registry / fingerprint;
                                               // new devices beyond this cap are silently ignored

// Mode B — phone-distance (phoneAway) commitments
PHONE_AWAY_TOLERANCE_S             = 60        // (tunable, watch) grace the user may be near/holding the phone
                                               // before enforcement; brief "quick checks" don't fire
DOCK_RSSI_THRESHOLD_DBM            = -60       // (tunable, anchor) anchor↔phone link RSSI at/above which the
                                               // phone counts as docked; below (or link lost) → undocked
DOCK_RSSI_POLL_MS                  = 2000      // (tunable, anchor) how often the anchor samples the docking-phone
                                               // link RSSI and updates Dock Status
```

---

## 8. Power & Radio Optimization (Watch)

The radio (BLE + WiFi) dominates the watch's energy budget; the MCU spends most of the day in light sleep with the radio off (`ADVERTISE_DURING_SLEEP = false`). The estimates below assume ~8 h/day inside enforcement windows (user mostly compliant, so the watch light-sleeps between polls) and ~16 h/day idle. Absolute figures are order-of-magnitude; the **ranking** is what matters.

### 8.1 Radio cost ranking (per day)

| Rank | Operation | Cadence | ~Energy/day | Notes |
|------|-----------|---------|-------------|-------|
| 1 | WiFi AP scan in scan-vector build | per proximity query | ~40 mAh | Only if WiFi associated. Blocking active scan. APs are stationary → mostly wasted. |
| 2 | WiFi connected idle (modem sleep) | continuous during anchor enforcement | ~40 mAh | Only if associated. DTIM wakeups. |
| 3 | Pre-query active BLE scan | per proximity query | ~12–17 mAh | Largest BLE consumer. Full-duty + active TX. |
| 4 | GATT proximity connection | per proximity query | ~3 mAh | connect + MTU + write + read + disconnect. |
| 5 | Continuous advertising (peripheral) | while awake | ~2–5 mAh | Modest. |
| 6 | DORMANT anchor-discovery scan | 300 ms / 60 s, motion-gated | ~1–3 mAh | Only while awake. |
| 7 | Enforcement fallback BLE scan | low-duty, periodic | <1 mAh | Redundant once each poll refreshes the cache. |
| — | DORMANT_SLEEP (radio off) | 16 h | ~2 mAh | Light-sleep baseline. |

WiFi dominates **when associated**; otherwise the per-query active BLE scan dominates.

### 8.2 Implemented optimizations

These are mandatory watch behaviors, not just suggestions:

1. **WiFi AP scan caching + association gate.** WiFi APs are physically stationary, so the watch does **not** rescan them on every proximity query. AP scan results are cached and reused for `PROX_WIFI_SCAN_INTERVAL_MS` (default 5 min); a fresh scan runs only when the cache is stale. The watch **skips the WiFi scan entirely when not associated** (`WiFi.status() != WL_CONNECTED`) — anchor proximity is computed from BLE alone in that case. (Addresses ranks 1–2.)

2. **Bounded pre-query scan.** The cache-refresh scan that precedes each query is bounded by `ENFORCEMENT_QUERY_SCAN_DURATION_MS` (default 700 ms, reduced from 1500 ms). Device counts far exceed `PROX_MIN_DEVICE_COUNT` in typical environments, so the shorter scan preserves fidelity while roughly halving the largest BLE cost. (Addresses rank 3.)

3. **Adaptive enforcement poll interval.** When the enforcement condition is **met and stable**, the watch backs the poll interval off to `ENFORCEMENT_POLL_INTERVAL_MET_S` (default 180 s); when **not met**, it polls at `ENFORCEMENT_POLL_INTERVAL_S` (default 60 s). The IMU motion interrupt still forces an immediate re-check, so responsiveness to the user actually moving is unchanged — only redundant polling of a still, compliant user is reduced. (Addresses ranks 3–4.)

4. **No redundant fallback scan.** The standalone periodic enforcement fallback scan is removed: every poll (and every motion-triggered check) already runs a fresh aligned scan immediately before building the vector, and the watch light-sleeps between polls when compliant, so a separate background scan served no purpose. (Addresses rank 7.)

### 8.3 Optimization tunables

```
PROX_WIFI_SCAN_INTERVAL_MS         = 300000    // (tunable) ms between actual WiFi AP scans; cached results are
                                               // reused in between. APs are stationary, so this can be large.
ENFORCEMENT_QUERY_SCAN_DURATION_MS = 700       // (tunable) bounded duration of the pre-query cache-refresh scan
ENFORCEMENT_POLL_INTERVAL_S        = 60        // (tunable) poll cadence while the condition is NOT met
ENFORCEMENT_POLL_INTERVAL_MET_S    = 180       // (tunable) backed-off poll cadence while the condition IS met
ENFORCEMENT_IDLE_BEFORE_SLEEP_MS   = 30000     // (tunable) still-time before entering enforcement light sleep
DORMANT_TO_SLEEP_IDLE_MS           = 1500      // (tunable) idle time in DORMANT before dropping to DORMANT_SLEEP
WIFI_SCAN_INTERVAL_S               = 120       // (tunable) DORMANT WiFi reconnect-scan cadence
```

### 8.4 Motion-wake gating & idle reachability

Wrist-worn motion is the watch's largest *potential* battery drain: at the original sensitivity, casual movement woke the watch every few seconds, and each wake held the CPU awake long enough that an active user could keep it out of sleep most of the day (estimated tens to hundreds of mAh/day). The wakeups bought nothing outside an enforcement window. This is addressed in four parts (the fourth, look-to-wake, deliberately re-introduces a *narrow* orientation-gated wake source):

1. **High-pass-filtered motion interrupt.** The LIS3DH interrupt generator runs through its high-pass filter (`CTRL_REG2` HPIS1), so `MOTION_THRESHOLD_MG` measures *dynamic* acceleration with gravity removed — consistent in every wrist orientation, instead of the original gravity-relative threshold that was hair-trigger in some resting positions and dead in others. Because motion now only matters during enforcement (part 2), the threshold and `MOTION_DURATION_MS` are tuned for *responsiveness* — low enough that normal-pace movement (e.g. walking away from a stayNear anchor) reliably triggers a re-check, with a short duration filter so brief gait peaks still register — rather than for idle battery, which motion no longer affects.

2. **Motion is a wake source only during ENFORCEMENT.** In `DORMANT_SLEEP`, general high-pass motion (IA1) is not a wake source; motion received while in DORMANT is cleared and ignored without resetting the idle timer. (When look-to-wake is enabled — part 4 — INT1 stays armed for the *look* generator instead, not general motion.) During an enforcement window, motion still wakes the watch and forces an immediate condition re-check (responsiveness preserved exactly).

3. **Idle reachability without motion.** Because a still watch no longer wakes on motion, two mechanisms keep it reachable by the phone app:
   - **No idle sleep while connected.** Neither `DORMANT_SLEEP` nor enforcement light sleep is entered while `bt_connected` is true, so an in-progress configuration session is never dropped by the radio powering down.
   - **Advertise heartbeat.** While disconnected, `DORMANT_SLEEP` caps its wake timer at `DISCONNECTED_ADV_HEARTBEAT_MS` (default 6 s). The watch surfaces each interval, advertises during its brief awake window, and accepts a connection if a phone is scanning — so a still, idle watch is always reachable within ~one heartbeat, with no button or motion required.

4. **Look-to-wake (wrist-raise), `LOOK_WAKEUP_ENABLED`.** The LIS3DH's second interrupt generator (IA2) detects the "looking at the watch" posture and wakes the watch from `DORMANT_SLEEP` through the shared INT1 line. The watch face / LED ring sit on the **−Z** axis, so holding the face up to the sky points −Z up and the accelerometer reads `az ≈ −1 g` — a **Z-low** event. IA2 is configured `AOI=1 | ZLIE` (`INT2_CFG = 0x90`) reading the *raw* signal (HPF off) so it sees gravity/orientation; it fires when `az < −LOOK_TILT_THRESHOLD_MG` held for `LOOK_DURATION_MS`. The two generators are OR'd onto the single INT1 pin and demuxed in software (`lis3dh_read_clear_sources()` returns a `LIS3DH_SRC_MOTION | LIS3DH_SRC_LOOK` bitmask).

   The wake is a one-shot driven by a `g_look_consumed` flag, and INT1 stays GPIO-armed in both phases (only *which* generator drives it changes, via `lis3dh_set_int1_routing(motion_ia1, look_ia2)`):
   - **Armed** (`consumed == false`): route IA2 only. A wrist-raise wakes the watch immediately; on a real look it plays the boot/flourish animation and sets `consumed = true`. General motion stays muted for battery.
   - **Shown** (`consumed == true`): route IA1 (motion) only — *not* the face-up level interrupt, which would re-latch every `LOOK_DURATION_MS` and spin the CPU while the watch is held up. Instead the motion of *lowering* the wrist wakes the watch promptly to re-check orientation; once `lis3dh_is_look_lowered()` reports `az > −LOOK_CLEAR_THRESHOLD_MG`, `consumed` clears and the next glance is armed again. Held perfectly still it makes no motion, so it still sleeps the full heartbeat. The `LOOK_CLEAR` vs `LOOK_TILT` gap is the hysteresis that stops a borderline hold from flip-flopping the one-shot.

   Set `LOOK_WAKEUP_ENABLED = 0` to compile the feature out entirely (reverts to timer-only DORMANT sleep with motion muted).

   > **Tuning note (current build):** the `LOOK_*` and `MOTION_*` thresholds below are deliberately **overpermissive** for the active testing/tuning pass — the intent is many false positives so the gesture fires on the gentlest glance. They should be tightened once the wake plumbing is confirmed end-to-end. Watch the `[DIAG] wake az=…` serial line to see what each wrist motion actually produces.

This is a **latency-vs-battery** tradeoff: a shorter heartbeat connects faster but costs more idle power (the watch is awake during each heartbeat window). A full BLE-controller-wake implementation via `esp_pm` automatic light sleep would make idle reachability essentially free (sub-mA, zero latency) and is the recommended future refactor.

```
MOTION_THRESHOLD_MG                = 48        // (tunable, imu.cpp) OVERPERMISSIVE: dynamic accel to trigger; low
                                               // so the LOWERING wrist motion reliably wakes to re-arm look-to-wake
MOTION_DURATION_MS                 = 20        // (tunable, imu.cpp) motion must persist this long to count;
                                               // short so brief gait peaks still register
LOOK_TILT_THRESHOLD_MG             = 48        // (tunable, imu.cpp) OVERPERMISSIVE RAISE: az below −0.048 g — barely
                                               // any face-up tilt required; held for LOOK_DURATION_MS
LOOK_DURATION_MS                   = 40        // (tunable, imu.cpp) IA2 hold time (2 ODR samples); nonzero only so
                                               // the level interrupt doesn't re-latch instantly when parked face-up
LOOK_CLEAR_THRESHOLD_MG            = 600       // (tunable, imu.cpp) OVERPERMISSIVE LOWERED: az risen above −0.6 g
                                               // counts as "lowered" and re-arms the one-shot (hysteresis vs RAISE)
DISCONNECTED_ADV_HEARTBEAT_MS      = 6000      // (tunable) max DORMANT_SLEEP interval while disconnected, so the
                                               // watch periodically advertises and is reachable by the app
```

---

## 9. Commitment Integrity — The Watch as Root of Trust

**Principle:** the watch, not the app, authoritatively enforces the product's self-binding asymmetry — *instant to tighten, slow to loosen*. The app is an authoring terminal: it is deletable, reinstallable, and replaceable by a generic BLE tool, so any loosening guarantee implemented only app-side can be bypassed in minutes. The watch therefore validates every proposed change itself: **the app proposes, the watch disposes.** The app mirrors these rules for previews (MOBILE_APP_SPEC §8.9), but only the watch's verdict is binding.

This is anti-*impulse* hardening, not anti-adversary security. A determined user with a flashing cable always wins; the goal is that no escape is available in the moment, from the phone, in a few taps (`impulse_overview.md`: "hard to beat on impulse," never "impossible to cheat").

### 9.1 Canonical tighten/loosen classification

Every proposed change to an event is classified **tightening**, **loosening**, or **non-comparable**. Non-comparable ⇒ treated as **loosening** (conservative: if the watch can't prove a change binds at least as hard, it is not instant). This table is the single canonical definition; the app implements identical rules so its previews match the watch's verdicts.

| Field changed | Tightening | Loosening | Non-comparable |
|---|---|---|---|
| Event added | always | — | — |
| Event deleted | — | always | — |
| Window (`startTime`/`endTime`) | new window ⊇ old (starts earlier and/or ends later) | new window ⊆ old | partial shift (e.g. both start and end move later) |
| Recurrence | new occurrence set ⊇ old (e.g. weekly→daily, same window) | new set ⊆ old (daily→weekly) | different days (weekly Tue→Thu) |
| `profile` | ≥ in both dimensions of the partial order below | ≤ in both | otherwise |
| `criteria` | — | — | any change |
| `anchorId` / `wifiSSID` target | — | — | any change |
| `beepAnchors` | superset | subset | overlapping-but-different sets |
| `anchorProfile` | hard > medium > light | reverse | — |
| `donningGraceS` | decrease | increase | — |
| One-time `negate` added against a recurring event | — | always (cancels a day) | — |

**EnforcementProfile partial order** — two independent dimensions:
- Strictness: `strict > normal > loose`.
- Output: `both > buzz` and `both > silent`; `buzz` vs `silent` are non-comparable.

A profile change is tightening only if ≥ in both dimensions (and > in at least one); loosening only if ≤ in both; anything else is non-comparable.

**Multi-field edits:** an edit is tightening only if **every** changed field classifies as tightening; a single loosening or non-comparable field makes the whole change a loosening.

### 9.2 On-watch settle state

Alongside the schedule blob, the watch persists per event (NVS):
- `last_edit_elapsed` — elapsed-time stamp of the last accepted change to this event.
- `settled_baseline` — snapshot of the event as of its most recent settle (absent for events that have never settled).

An event **settles** when `SETTLE_WINDOW_MIN` (default 120) minutes of elapsed time pass with no accepted change; at that instant its current state is copied into `settled_baseline`.

**Elapsed-time basis:** all integrity timers — settle windows, pending `apply_after`, pass-window aging — count **monotonic elapsed time** using a persisted elapsed-time accumulator spanning reboots (the same pattern as the usage instrumentation's persisted elapsed base), **never wall-clock time**. A clock write therefore cannot accelerate any integrity timer.

### 9.3 Schedule-diff gate

Replaces blind replacement in §5.1.4. On receiving a CRC-valid, version-valid schedule blob over BLE:

1. Deserialize as the **proposed** schedule.
2. Diff against the **current** schedule per event UUID; classify each change per §9.1.
3. Partition the changes:
   - **Apply immediately:** tightenings; new events; any change to an **unsettled** event whose resulting state is still ≥ its `settled_baseline` (or that has no baseline yet — the free first-setup window).
   - **Apply immediately (no-escape exception):** loosenings of an event that is **not currently active** and whose next occurrence starts more than `LOOSEN_FREE_HORIZON_H` (24) hours away — loosening a far-future commitment grants no in-the-moment escape.
   - **Quarantine:** all other loosenings (including below-baseline edits of unsettled events), as pending-queue entries with `apply_after = now_elapsed + LOOSEN_DELAY_H` (24) hours.
4. New current schedule = old schedule + immediately-applied changes. Store to flash, `recalculate_day(today)`, re-arm callbacks.
5. Append quarantined changes to the **pending queue** (NVS, capacity `PENDING_QUEUE_MAX`). If the queue is full, further loosenings are dropped (not applied); the app sees the truth via §9.5.
6. Respond per §6.2: `0x01` full accept / `0x03` partial quarantine.

**Pending-entry semantics:** a quarantined entry stores the **full proposed post-change state** (the complete new Event, a deletion marker, a negate date, or a setting value) — not just a classification — so promotion needs no app involvement. A subsequently **accepted** change to the same event (or setting) **cancels** any pending entries for it: the newer intent wins, and re-requesting the loosening restarts its 24 h. (Canceling a pending loosening only ever binds harder, so this rule opens no escape.) Cancellations are reported via §9.5, and the app must surface them.

**Phase-1 minimum (ship first):** before the full diff engine exists, the watch must at minimum reject any push that would delete, shorten, weaken, or negate the **currently active** event — compare the active event only; if it changed loosening-wise, discard the entire push and respond `0x04`. This single check closes the "push an empty schedule at 6am" bypass.

### 9.4 Autonomous promotion

The watch promotes its own pending queue with no app involvement: on every wake (RTC boundary, midnight, boot), it applies any pending entries whose `apply_after` has been reached, then re-runs `recalculate_day` and re-arms callbacks, and notifies §9.5. Promotion therefore happens *no earlier than* its nominal time — a sleeping watch may promote slightly late, never early. Entries whose effect has already expired at promotion time (e.g. a negate-day whose date has passed) are dropped, not applied.

### 9.5 Pending Changes characteristic

See §5.6 (`WATCH_PENDING_CHAR_UUID`, `…001A`) for the wire format. The queue exposed there is the authoritative pending state; the app renders it and can rebuild its own view from it after a reinstall.

### 9.6 Emergency pass ledger

The rolling pass budget (MOBILE_APP_SPEC §8.10) lives on the watch, so clearing app data cannot replenish passes. Ledger: allowance (`pass_allowance`, default `PASS_BUDGET_DEFAULT` = 2), plus elapsed-time stamps of spends within the last `PASS_WINDOW_DAYS` (7) days.

On a SPEND write (§5.6, `…001B`): if spends-in-window < allowance, apply a one-day negate for the given event/date **immediately — deliberately bypassing §9.3** (the pass is the sanctioned escape valve, spendable even on the currently active event), record the spend, respond success + remaining. Otherwise respond exhausted. Raising `pass_allowance` is a loosening ⇒ routed through the pending queue (change type 3); lowering it applies immediately.

### 9.7 Time hardening

- A Time write (§5.6, `…0019`) whose effect would end or skip the **currently active** enforcement window (new local time falls outside the active event's window while the previous time was inside) is rejected with `0x02`. All other time writes are accepted normally.
- The same guard applies to **timezone changes** arriving via the Settings characteristic (§5.6): a `tz_offset_minutes` change that would place the current local time outside the active event's window is rejected with `0x02`.
- NTP, when available, takes precedence over app-supplied time.
- Integrity timers are immune to clock changes regardless (elapsed-time basis, §9.2).

### 9.8 Settings gating

Settings writes route through the same classifier. For the dormancy flags (§5.1.1, where `true` = keep enforcing in that connectivity state): `true→false` is a loosening ⇒ quarantined (pending type 3); `false→true` is a tightening ⇒ immediate. Growing `SETTLE_WINDOW_MIN` (a longer free-edit window) is a loosening ⇒ quarantined; shrinking it is immediate. The settle-window value is clamped to `[SETTLE_WINDOW_FLOOR_MIN, SETTLE_WINDOW_CEIL_MIN]` = **[30, 240] minutes** on write. A Settings write containing any quarantined field responds `0x03` (non-gated fields apply immediately); an all-immediate write responds `0x01`.

### 9.9 Phasing

| Phase | Scope |
|---|---|
| **1** | Active-event protection only: reject pushes that loosen the currently active event (`0x04`). Smallest possible change; closes the worst bypass. |
| **2** | Full diff gate (§9.1–9.3), on-watch settle state (§9.2), pending queue + autonomous promotion (§9.4), Pending Changes characteristic (§9.5). |
| **3** | On-watch pass ledger (§9.6), time hardening (§9.7), settings gating (§9.8). |

Until a phase ships, the app's mirrored policy (MOBILE_APP_SPEC §8.9/§8.10) is the only enforcement of that slice — acceptable during development, not for release hardware.

### 9.10 Constants

```
SETTLE_WINDOW_MIN            = 120    // (gated-configurable, §9.8) minutes without edits before an event settles
SETTLE_WINDOW_FLOOR_MIN      = 30     // lowest allowed settle-window setting (minutes; clamp on write)
SETTLE_WINDOW_CEIL_MIN       = 240    // highest allowed settle-window setting (minutes; clamp on write)
LOOSEN_DELAY_H               = 24     // hours a quarantined loosening waits before promotion
LOOSEN_FREE_HORIZON_H        = 24     // loosenings of events starting further out than this apply immediately
PASS_BUDGET_DEFAULT          = 2      // emergency passes per rolling window (raising = gated, §9.6)
PASS_WINDOW_DAYS             = 7      // rolling pass-window length, elapsed-time basis
PENDING_QUEUE_MAX            = 16     // max quarantined entries; further loosenings are dropped when full
SCHEDULE_FORMAT_VERSION      = 0x02   // leading version byte of the schedule blob (§6.2)
```

---

*End of specification v0.6*
