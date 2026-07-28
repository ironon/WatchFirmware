# FIRMWARE_SPEC v0.9 Amendment — Proximity Engine v2.1 Integration

**Applies to:** ADHD Habit Enforcement Watch & Anchor — Firmware Specification v0.8
**Produces:** v0.9
**Companion:** `proximity_engine_spec_v2.1.md` (algorithm rationale & math). This amendment is the normative source for wire formats, GATT, constants, power-state behavior, and build order. Apply it to the master spec per that spec's own §0 maintenance rules.

**How to apply:** each numbered part below either (a) inserts a change-log/parity entry, (b) **replaces** a named section's text in full, or (c) **inserts** a new section. Unmentioned sections are unchanged. All wire changes in this amendment form **one lockstep batch** (Phase 2 in §10-A); Phase 1 is deliberately wire-clean and can ship alone.

---

## Part 1 — §0.2 Change log: insert entry

**v0.9 — (date of merge)** (Phase 2 is a lockstep wire batch — flag in MOBILE_APP_SPEC §0; Phase 1 is watch-only, no wire change)

- **Proximity engine v2.1 (proximity.cpp, both roles).** Root-cause fix for flat/uninformative indoor RSSI: multipath fading is only separable from distance in the frequency and spatial domains, so the engine adds (a) a **motion-conditioned two-state HMM** whose NEAR/AWAY state can only flip when the LIS3DH says the wrist could have moved, (b) **motion-gated integration** with honest effective-sample accounting (a still wrist stops accumulating evidence instead of averaging a frozen fade into false confidence), (c) an anchor **beacon schedule** cycling (channel × TX power) slots tagged in the previously-unused iBeacon **Minor** field, yielding per-channel RSSI, a cross-channel-spread range feature, and **stepped-power packet-delivery-ratio** — a thresholded feature immune to amplitude noise. See `proximity_engine_spec_v2.1.md`.
- **Phase 1 (watch only, no wire change):** IMU motion channel (`prox_ingest_imu_burst`, sleep-interval verdicts from the existing IA1 wake source), HMM replacing the threshold interpretation in §5.4.1 step 8 and the coloc hysteresis machine, connect-failure evidence as a log-LR, and a third poll tier `ENFORCEMENT_POLL_INTERVAL_STILL_S` (600 s) when met + confident + STILL. DORMANT's v0.7 zero-IMU-interrupt guarantee is **unchanged** — all v2 machinery runs only during ENFORCEMENT and calibration bursts.
- **Phase 2 (lockstep batch — watch + anchor + app together):**
  - **Anchor advertising (§4.3):** Minor carries `(slot_id << 12) | cycle_seq`; Major/scan-response untouched. New beacon-schedule task (§4.12).
  - **Proximity vector format v2 (§6.3.1):** leading version byte `0x02` + feature trailer (motion state, neff, per-anchor PDR/spread/per-channel RSSI).
  - **Proximity Score payload grows 2 → 3 bytes (§4.4):** `[score][flags][neff]`.
  - **Anchor scoring (§4.10.3):** `prox_compute_score2` folds trailer features via location-trained distributions; new per-peer-anchor side table + **away-training gate** (§4.10.8); training gate becomes motion-qualified (§4.10.4).
  - **Calibration START gains a leg byte (§5.6):** near/away legs; away leg trains away distributions at burst speed.
  - **Observation window:** anchor-based enforcement queries use a full-duty `PROX_OBSERVE_WINDOW_MS` (1800 ms) scan in place of the 700 ms pre-query scan; power offset by the STILL poll tier (§8 amendment).
- **Deferred (Phase 4, feasibility-gated):** Wi-Fi FTM tie-breaker (needs anchor APSTA under IDF 4.4.7 — Spike S5) and **reversed-link Wi-Fi CSI** (watch transmits an ESP-NOW ping burst — stock 2.0.17 TX; the **anchor** captures CSI and folds channel-shape features into the score — engine spec §3.5). CSI rides the already-planned anchor pioarduino/IDF 5.x migration (§10.1) with a CSI-enabled build; watch-side CSI capture is explicitly ruled out (compile-time option absent from Arduino 2.0.17's precompiled libs; 4.4.7 RX coexistence risk). Do not start either without its spike passing.
- New constants: §7 amendment block. New flags: `PROX_FLAG_V2_TRAILER`, `PROX_FLAG_TXLO_MISCAL`.

## Part 2 — §0.1 Parity table: insert rows

| Spec area | Watch | Anchor | Notes |
|---|---|---|---|
| Prox v2 P1: IMU channel + HMM + STILL poll tier (§5.4.1, §5.4.5) | ❌ | — | Watch-only; no wire change; ship independently. |
| Prox v2 P2: beacon schedule (§4.3, §4.12) | — | ❌ | Spikes S1/S2 gate design details (channel map, slot rate). |
| Prox v2 P2: vector v2 + score2 + side table (§4.10, §6.3) | ❌ | ❌ | **Lockstep** with app parser (MOBILE_APP_SPEC). |
| Prox v2 P2: calibration leg byte (§5.6, §4.10.7) | ❌ | — | Same lockstep batch. |
| Prox v2 P3: coupling detector, per-channel δ (Mode C) | ❌ | — | After P2 field validation. |
| Prox v2 P4: FTM / CSI | ❌ | ❌ | Spike-gated; see §10.1 toolchain notes. |

---

## Part 3 — §4.3 BLE Advertising: **replace** the bullet list

The anchor advertises continuously using the **iBeacon format**:

- **UUID:** the anchor's stored UUID.
- **Major:** `0x4A0F` — fixed Impulse namespace fingerprint (watch filter unchanged).
- **Minor:** **beacon-schedule slot tag** (was constant `0x0000`). When `BEACON_SCHEDULE_ENABLE = 1`:

  `Minor = (slot_id << 12) | (cycle_seq & 0x0FFF)`

  where `slot_id` ∈ 0–5 identifies the current (channel, TX power) slot per §4.12 and `cycle_seq` increments once per full schedule cycle. Major and Minor are transmitted **big-endian on air** per iBeacon convention, so `slot_id` occupies the high nibble of the *first* Minor byte — verify against the watch-side parse in the P2 round-trip test. When the schedule is disabled, Minor = `0x0000` and behavior is exactly v0.8. Watches treat all-zero Minors as "no schedule" and degrade gracefully (Phase-1 features only).
- **Advertising interval:** `BEACON_ADV_INTERVAL_MS` (50 ms) while the schedule is enabled; 100 ms legacy otherwise.
- **Connectable:** yes, **in every slot** — setup, identify, and proximity queries must work regardless of the schedule's current slot. (Spike S2 validates this on the NimBLE host; its fallback is `BEACON_SLOT_MS = 500`.)

The scan response packet is unchanged (service UUID + 16-byte identity service data).

## Part 4 — new §4.12 Beacon Schedule Task **(proximity.cpp)**: **insert** after §4.11

A timer-driven anchor task advances a fixed 6-slot cycle every `BEACON_SLOT_MS` (250 ms):

| slot | channels | TX power |
|---|---|---|
| 0/1/2 | 37 / 38 / 39 | `PROX_QUERY_TX_POWER_DBM` (+9) |
| 3/4/5 | 37 / 38 / 39 | `BEACON_TX_LO_DBM` (−21, S1-calibrated) |

On each slot boundary the engine calls the platform seam `prox_platform_set_beacon_slot(channel_map, tx_power_dbm, minor)`; the platform performs the advertising stop → reconfigure (channel map, TX power, Minor) → start sequence on the NimBLE host task. Requirements:

1. The reconfiguration must not tear down active GATT connections or reject incoming connects (S2 acceptance test).
2. TX power is set via the platform's advertising-TX-power API before restart.
3. **S1 fallback:** if per-slot channel restriction is unavailable on this host stack, collapse to 2 slots (HI/LO, all channels, `slot_id` ∈ {0, 3}); the watch detects this from the observed slot_id set and marks channel features unknown. PDR is unaffected.
4. Mode C anchor pairs phase-offset their cycles by `BEACON_SCHEDULE_EPOCH_OFFSET_MS` (750 ms), seeded from the anchor UUID's low bit, so TX_LO slots don't collide.
5. The schedule runs whenever the anchor is powered; it does not depend on WiFi state and must not perturb the §4.10.2 scan tasks (S2 measures scan-task starvation).

## Part 5 — §4.10.3 Score Computation: **append**

**v2 scoring (`prox_compute_score2`).** When the vector carries the v2 trailer (§6.3.1), the v1 blended score is converted to log-odds and summed with per-feature log-likelihood-ratios before the final logistic → `uint8`:

- $\ell_{\text{pdr}}$ from the trailer's LO-slot hit counts via Beta-binomial tables (`PDR_NEAR_ALPHA_BETA` / `PDR_AWAY_ALPHA_BETA`, replaced by trained counts once available);
- $\ell_{s_{ch}}$ from the cross-channel spread against trained near/away Gaussians;
- $\ell_{ch}$ from per-channel RSSI against the per-peer-anchor side table (§4.10.8).

Each term **abstains** (adds 0) when its feature is marked unknown (0xFF / zero slots) or its distributions are untrained (`W < PROX_MIN_FINGERPRINT_WEIGHT`). Result: `ProxScoreResult2 {score, flags, neff}` where `neff = min(trailer.neff, internal cap)`; `PROX_FLAG_V2_TRAILER` is set when trailer features contributed. The v1 `prox_compute_score()` entry point remains as an internal wrapper (drops `neff`) for host-side tests; after the P2 lockstep flash the characteristic accepts **only** format `0x02` — any other leading byte gets response `0x00` via the existing malformed-payload path (§6.3.1). There is no mixed-version parsing.

## Part 6 — §4.10.4 Self-Supervised Training: **append**

**Motion qualification (v2).** A sample additionally requires `trailer.motion_state ∈ {FIDGET, LOCOMOTION}`; its Welford weight becomes $w_n = (\text{score}/255)\cdot\min(1, \text{trailer.neff}/\text{NEFF\_TRAIN\_MIN})$. Vectors reporting `motion_state = UNKNOWN` train only while the fingerprint is younger than `PROX_TRAIN_BOOTSTRAP_W` total weight (bootstrap allowance); calibration-leg vectors always qualify regardless of motion state. Rationale: a stationary watch parked in a fade must not pump frozen identical samples into the fingerprint and collapse $\sigma^2$ around a fading artifact.

## Part 7 — new §4.10.8 Feature Distributions & Peer-Anchor Side Table **(proximity.cpp)**: **insert**

A dedicated NVS table (`prox2` namespace), ≤ `ANCHOR_PROX_MAX_PEER_ANCHORS` (8) entries keyed by anchor BLE MAC. Per entry, for **near** and **away** classes: per-channel Welford stats $(\mu_{ch}, \sigma^2_{ch}, W_{ch})$ for ch ∈ {37, 38, 39}, Beta counts $(\alpha, \beta)$ for TX_LO PDR, and Gaussian stats for $s_{ch}$. Training sources:

- **Near:** vectors passing the §4.10.4 gate (motion-qualified).
- **Away:** vectors with $\text{score}/255 \le$ `PROX_TRAIN_AWAY_THRESHOLD` (0.25) **and** this anchor's raw RSSI in the vector ≤ `PROX_FAR_RSSI_THRESHOLD_DBM` (or absent). Deliberately conservative — mirrors the near gate's unambiguity requirement.
- **Calibration legs** (§4.10.7 amendment) at weight 1.0 regardless of score, keyed by the leg byte.

Variance floors (`PROX_MIN_VARIANCE`) apply throughout. Persisted on the existing `PROX_NVS_PERSIST_INTERVAL_S` cadence. The main 128-entry registry and blob format (§6.3.2) are **unchanged**; the side table has its own compact NVS blob and is *not* included in the app fingerprint transfer (it is anchor-authored only; regenerates from calibration in minutes).

## Part 8 — §4.10.7 Calibration Burst: **append**

**Leg byte (v2).** START gains a trailing `[1 byte: leg]` — `0x01` near (default; `0x00` from older apps maps to near), `0x02` away. During an away leg the watch runs the identical burst loop but flags each vector's trailer `CALIB_AWAY`, and the anchor routes those samples to the away distributions of §4.10.8 instead of the near gate. The app's guided flow becomes two short legs: "walk around the anchor" then "stand where you'd normally be when away." If a near leg measures TX_LO PDR < 0.8 at arm's length, the score result sets `PROX_FLAG_TXLO_MISCAL` and the app should offer the S1 level table.

**Burst timing (v2):** each calibration query now spans a full observation window, so the effective cadence is ~`PROX_OBSERVE_WINDOW_MS` + connect overhead (≈ 2.2 s/query). `CALIB_QUERY_INTERVAL_MS` is raised 600 → 2000 and acts as a floor; the default 75 s session yields ~30 queries. App-side completion targets keyed to accepted-sample counts should assume this rate.

## Part 9 — §5.4.1 Condition Checking: **replace** the anchor-based procedure's steps 1 and 8, and the polling paragraph

**Step 1 (replaces):** Collect the scan vector using a **full-duty observation window** of `PROX_OBSERVE_WINDOW_MS` (1800 ms) — long enough to cover one full beacon-schedule cycle plus one slot at any phase — instead of the v0.8 700 ms scan. During the window, the firmware concurrently samples an IMU burst (`IMU_BURST_SAMPLES` @ `IMU_BURST_HZ`) and feeds it to `prox_ingest_imu_burst()` (§5.4.5); the radio and the SPI accelerometer read do not contend. Slot-tagged advertisements accumulate per-channel RSSI and LO-slot hit/covered counts; the WiFi AP cache rules (§8.2) are unchanged.

**Step 8 (replaces):** Feed the returned `ProxScoreResult2` to `prox_hmm_tick()`. The HMM (see `proximity_engine_spec_v2.1.md` §6) maintains a NEAR/AWAY posterior whose transitions are gated by the motion state — flips are effectively locked while the wrist is provably STILL and unlocked by LOCOMOTION — and returns the same three-way `NEAR / AWAY / AMBIGUOUS` decision the v0.8 threshold produced, with the identical criterion-dependent fail-safe resolution of AMBIGUOUS. Connect failures are reported via `prox_note_connect_failure()` (replacing the ad-hoc v0.8 §2.7 rule) and enter the HMM as watch-local AWAY evidence; a tick with no query still yields the held decision. On ENFORCEMENT entry for an anchor-based or `phoneAway` event, call `prox_hmm_reset(criterion)`.

**Polling (append to the Polling paragraph):** a third tier is added: when the condition is met **and** the HMM posterior is confident (outside the ambiguous band) **and** the motion state is STILL, the poll interval backs off to `ENFORCEMENT_POLL_INTERVAL_STILL_S` (600 s). Any IA1 motion interrupt exits this tier immediately and forces a re-check, so responsiveness to actual movement is unchanged; the enforcement light-sleep duration cap (§5.4.2) uses whichever poll interval is currently in force.

## Part 10 — new §5.4.5 IMU Motion Channel **(proximity.cpp seams; imu.cpp glue)**: **insert**

Wholly contained in ENFORCEMENT (DORMANT keeps v0.7's zero-IMU-interrupt guarantee). Three glue obligations on the firmware:

1. On every wake from **enforcement light sleep**, call `prox_note_sleep_interval(slept_ms, motion_woke)` where `motion_woke` is whether IA1 was the wake cause. A motionless sleep interval is authoritative STILL evidence for its whole span.
2. Forward each awake IA1 firing to `prox_note_motion_interrupt()` (in the existing ISR-drain path, after the re-arm delay logic — do not change `MINIMUM_BLE_DELAY_ENFORCEMENT` semantics).
3. During each observation window, read `IMU_BURST_SAMPLES` accelerometer triples at `IMU_BURST_HZ` over SPI and call `prox_ingest_imu_burst()`.

The engine classifies STILL / FIDGET / LOCOMOTION / UNKNOWN and exposes `prox_motion_state()` (consumed by the poll-tier logic in §5.4.1 and by trailer assembly in §6.3.1). Thresholds `IMU_STILL_VAR` / `IMU_LOCO_VAR` are S3-tuned; until S3 lands, ship conservative defaults with UNKNOWN→LOCOMOTION mapping so misclassification fails toward v0.8 behavior, never toward a frozen decision.

## Part 11 — §5.6 Calibration Control characteristic: **replace** the Write payload block

```
START: [1 byte: 0x01][16 bytes: target anchor UUID][2 bytes: duration_s (uint16)][1 byte: leg]
        leg: 0x01 = near (0x00 tolerated as near), 0x02 = away (§4.10.7/§4.10.8)
STOP:  [1 byte: 0x00]
```

(Notify/Read progress payload unchanged; `accepted` counts near-gate or away-gate acceptances per the active leg.)

## Part 12 — §6.3.1 Proximity Vector Payload: **replace** in full

Written by the watch to `ANCHOR_PROX_VECTOR_CHAR_UUID`. All fields little-endian. **Format v2** (lockstep — v1 parsers are retired in the same flash):

```
[1 byte:  format version = 0x02]
[1 byte:  device count N (0..PROX_MAX_DEVICES)]
N × device entries (8 bytes each, identical to v0.8):
    [6 bytes: mac][1 byte: type][1 byte: rssi+128]
[1 byte:  trailer flags]        bit0 CALIB_AWAY; bit1 CHANNELS_VALID (0 under S1 fallback)
[1 byte:  motion_state]         0=STILL 1=FIDGET 2=LOCOMOTION 3=UNKNOWN
[1 byte:  neff]                 effective sample count, capped 255 (Q4 → integer floor)
[1 byte:  anchor feature count K (0..4; target anchor first, then rivals)]
K × anchor feature entries (12 bytes each):
    [6 bytes: anchor BLE MAC]
    [1 byte:  pdr_hits]         TX_LO slots heard in the integrator's current window
    [1 byte:  pdr_slots]        TX_LO slots covered (0 ⇒ PDR unknown/abstain)
    [1 byte:  s_ch]             cross-channel spread, dB, clamped 0..63; 0xFF unknown
    [3 bytes: r37, r38, r39]    per-channel TX_HI RSSI as rssi+128; 0 = unseen
```

Max size at N=60, K=4: `2 + 480 + 4 + 48 = 534` bytes — exceeds a 512 MTU by 22, so at full occupancy the watch trims device entries (weakest-RSSI first) to fit `negotiated_mtu − 3`, never the trailer; `PROX_MIN_MTU_BYTES` unchanged. The anchor rejects version bytes it doesn't know with response `0x00` (existing malformed-payload path).

## Part 13 — §4.4 Proximity Score characteristic: **replace** the payload block

**Payload (3 bytes):**
```
[1 byte: score]   0–255
[1 byte: flags]   v0.8 bits, plus bit 3 PROX_FLAG_V2_TRAILER, bit 4 PROX_FLAG_TXLO_MISCAL
[1 byte: neff]    effective-sample echo (0 when unknown)
```
Grown 2 → 3 bytes: **lockstep** with the app's debug/score parser (the notify consumer). Reads shorter than 3 bytes are not supported after the batch.

## Part 14 — §7 Constants: **insert** block

```
// ── Proximity engine v2.1 (proximity.cpp) ────────────────────────────────
// Beacon schedule (anchor)
BEACON_SCHEDULE_ENABLE             = 1
BEACON_SLOT_MS                     = 250     // (S2 fallback: 500; scale OBSERVE window to match)
BEACON_ADV_INTERVAL_MS             = 50
BEACON_TX_LO_DBM                   = -21     // (tunable; S1 provides the per-level range table)
BEACON_SCHEDULE_EPOCH_OFFSET_MS    = 750

// Observation & features (watch)
PROX_OBSERVE_WINDOW_MS             = 1800    // full-duty; ≥ 6×SLOT + SLOT + margin
PROX_TRAIN_AWAY_THRESHOLD          = 0.25
PROX_TRAIN_BOOTSTRAP_W             = 50.0    // motion_state=UNKNOWN vectors may train below this W_total
PROX_V2_AUTHORITATIVE              = 0       // P1 ships at 0 (shadow logging); flip to 1 after §10-A P1 hardware check
CALIB_QUERY_INTERVAL_MS            = 2000    // was 600; each burst query spans a full observation window (§4.10.7)
ANCHOR_PROX_MAX_PEER_ANCHORS       = 8
PDR_NEAR_ALPHA_BETA                = {8, 2}  // Beta prior for TX_LO PDR | near
PDR_AWAY_ALPHA_BETA                = {1, 9}

// IMU motion channel (watch; §5.4.5)
IMU_BURST_SAMPLES                  = 32
IMU_BURST_HZ                       = 50
IMU_STILL_VAR                      = TBD     // (S3-tuned, LIS3DH LSB² after HP filter)
IMU_LOCO_VAR                       = TBD     // (S3-tuned)
IMU_LOCO_MIN_INTS                  = 2
IMU_STALE_MS                       = 5000
NEFF_LOCO_PER_S                    = 8       // Q4 N_eff credit per second of LOCOMOTION
NEFF_FIDGET_PER_S                  = 2
NEFF_TRAIN_MIN                     = 8
INTEG_STILL_WEIGHT                 = 26      // ≈0.10, Q8
INTEG_STILL_RELAX_S                = 30

// HMM (watch)
HMM_PFLIP_STILL                    = 1e-4    // per-tick, time-scaled between irregular ticks
HMM_PFLIP_FIDGET                   = 1e-3
HMM_PFLIP_MOVE                     = 0.15
HMM_TAU_NEAR                       = 0.80
HMM_TAU_AWAY                       = 0.20
LL_CONNFAIL_AWAY_Q8                = -768    // −3.0 log-LR in Q8

// Poll tiers (watch; §5.4.1)
ENFORCEMENT_POLL_INTERVAL_STILL_S  = 600     // met + HMM-confident + STILL

// Coupling detector (Phase 3)
CPL_WINDOW_S                       = 12
CPL_BIN_MS                         = 250
CPL_MIN_MOTION_BINS                = 8

// Reversed-link CSI (Phase 4 — provisional; do not implement before S4 passes)
CSI_PING_FRAMES                    = 4       // watch ESP-NOW frames per observation window
CSI_DEF_FS_NEAR_MU / _SIGMA        = 0.18 / 0.10   // flatness | near/LOS (Q8 on-device)
CSI_DEF_FS_AWAY_MU / _SIGMA        = 0.45 / 0.18
```

`ENFORCEMENT_QUERY_SCAN_DURATION_MS` (700) remains defined for the DORMANT-side aligned discovery scan; anchor-based enforcement queries now use `PROX_OBSERVE_WINDOW_MS` instead (§5.4.1).

## Part 15 — §8 Power & Radio Optimization: **append** subsection 8.5

**8.5 v2.1 deltas.** The observation window raises the per-query cost of rank 3 (pre-query scan) by ~2.6× (1800 ms vs 700 ms, full duty). Compensation: the STILL poll tier cuts query frequency by 3.3× (600 s vs 180 s) for the met-and-still bulk of typical windows, which is where nearly all queries occur for a compliant user — net budget neutral-to-positive there. During violations (60 s polls) the 2.6× applies unmitigated but violations are short by product design. The IMU adds **zero idle cost**: no new interrupt sources (IA1 usage unchanged), bursts execute inside already-awake observation windows, DORMANT untouched. Anchor-side: schedule reconfig CPU is negligible and TX_LO slots reduce mean radiated power; anchors are externally powered. Not-polling a STILL wrist is *correct*, not merely cheap — a stationary user cannot change proximity class, which is the HMM's premise; the IA1 interrupt restores full responsiveness the instant that premise breaks.

---

## Part 16 — §10 Build Order: **insert** as §10-A "Proximity v2.1 roadmap (coding agent)"

Execute phases strictly in order. Every phase ends green on its acceptance tests before the next begins. All algorithm code lands in `proximity_engine/proximity.cpp|.h`; glue lands in the named firmware files only.

### Phase S0 — Spikes (throwaway branch `spike/prox-v2`, ~write-up in `tests/prox_v2_spikes.md`)

| Spike | Question | Pass criterion | Fallback if fail |
|---|---|---|---|
| **S1a** | Which BLE adv TX-power levels does the C3 + this stack actually emit, and at which level does the 1–4 ft link close while ~10 ft does not? | A level with ≥90% slot-hit at 3 ft and ≤30% at 10 ft LOS | Pick best available level; widen PDR distributions |
| **S1b** | Can advertising be restricted to a single channel per slot on this NimBLE host (param/ext-adv/vendor-HCI)? | Slot-tagged adverts observed on exactly one channel | §4.12 item 3: power-only 2-slot schedule; `CHANNELS_VALID=0` |
| **S2** | Does a 250 ms adv stop/reconfig/start cycle keep the anchor connectable and its scan tasks fed? | 100 consecutive proximity connects succeed during schedule; scan-cache staleness unchanged | `BEACON_SLOT_MS=500`, `PROX_OBSERVE_WINDOW_MS=3600` |
| **S3** | LIS3DH burst thresholds: STILL/FIDGET/LOCOMOTION confusion on ≥2 real wrists (desk work, typing, walking, sleeping) | STILL false-LOCOMOTION < 20%, LOCOMOTION false-STILL ≈ 0 | Raise `IMU_STILL_VAR`; misclass toward LOCOMOTION is safe by design |
| **S4** *(pre-P4 only)* | Reversed-link CSI (engine spec §3.5): (a) **first step** — grep the anchor's target IDF-5.x framework sdkconfig for `CONFIG_ESP_WIFI_CSI_ENABLED`; if absent, the build route is Arduino-as-IDF-component with an owned sdkconfig; (b) anchor CSI RX callback + BLE + STA stable 60 min on IDF 5.x, receiving frames from watch pings; (c) watch ESP-NOW TX burst beside NimBLE on stock 4.4.7, stable | Flat heap, no WDT/panic, tagged CSI frames received end-to-end | CSI stays deferred; all other phases unaffected (CSI is non-load-bearing by design) |
| **S5** *(pre-P4 only)* | Anchor APSTA + FTM responder + BLE under 4.4.7: stable, and does FTM to a C3 initiator range within ±2 m indoors? | 60 min stability + ranging sanity | FTM deferred; consider post-IDF-5.x anchor migration |

### Phase P1 — Watch-only inference upgrade (no wire change; ship alone)

**Files:** `proximity.cpp/.h` (integrator, motion channel, HMM, LUT tables), `imu.cpp` (burst read + seam calls), `main.cpp` watch (sleep-interval note, poll tier, `prox_hmm_reset` on ENFORCEMENT entry, step-8 replacement).

**Work items:**
1. LUT generator (build-time script or constexpr tables): `logit8`, `neff_gain`, `logsumexp_corr`.
2. Motion channel per §5.4.5 (UNKNOWN→LOCOMOTION default until S3 constants land).
3. Motion-gated integrator (Q-format; §4.2 of the engine spec).
4. Two-state HMM (`prox_hmm_reset/tick/decision`), time-scaled transitions, connect-failure evidence via `prox_note_connect_failure` (delete the ad-hoc v0.8 rule at its call site and route through the seam).
5. Poll-tier logic + light-sleep cap interaction (§5.4.2 uses the in-force interval).
6. Coloc pipeline: swap EWMA → integrator, hysteresis machine → HMM (Modes B/C share the decision layer).

**Acceptance (host-side unit tests in `proximity_engine/tests/`, pure C, no hardware):**
- *Frozen-fade test:* constant deep-fade RSSI stream + STILL motion → posterior never flips from the initialized state; variance report re-inflates.
- *Walk-approach test:* synthetic path-loss trace with $\sigma = 5$ dB fading + LOCOMOTION → flips within 3 ticks of crossing, zero flips during the still segments.
- *Teleport-rejection test:* 20 dB step with STILL throughout → no flip for ≥ `1/PFLIP_STILL` tick-seconds equivalent.
- *IMU-stale test:* stale IMU ⇒ behavior equals a PFLIP_MOVE-only HMM (≈ v0.8 with smoothing).
- On-hardware: one evening of shadow logging — v2 decision logged alongside the shipped v0.8 decision (both computed; v0.8 still authoritative behind `PROX_V2_AUTHORITATIVE=0`) — then flip the flag.

### Phase P2 — Beacon schedule + feature pipeline (lockstep batch)

**Files:** anchor `main.cpp` (schedule timer + platform seam impl, per §4.12), `proximity.cpp/.h` (Minor encode/decode, window feature extraction, vector v2 build/parse, `prox_compute_score2`, side table + away gate, distributions), watch `main.cpp` (observation window swap, trailer assembly), app (vector/score/calibration parser — MOBILE_APP_SPEC batch).

**Order within P2:** (1) anchor schedule + Minor (harmless to v0.8 watches — they ignore Minor); (2) watch window + slot attribution + local feature extraction, logged only; (3) vector v2 + score2 + side table, flashed as the lockstep batch; (4) calibration leg byte + away training.

**Acceptance:**
- Slot attribution: watch logs per-channel RSSI histograms; at a fixed near position the three channels agree within ~6 dB; across the room, spread visibly grows (this is the feature working).
- PDR monotonicity: median PDR over 20 windows ≥0.9 at 2 ft, ≤0.3 at 10 ft (S1a numbers).
- Wire: round-trip fuzz test of vector v2 encode/parse (host-side); anchor rejects bad version bytes.
- End-to-end: the §4.10.7 two-leg calibration populates near+away distributions and `prox_compute_score2` separates the calibration positions by ≥ 40 score points at `neff ≥ 8`.
- Regression: with `BEACON_SCHEDULE_ENABLE=0`, byte-identical behavior to P1.

### Phase P3 — Mode C sharpening + coupling detector

Per-channel $\delta$ with epoch offset; coupling detector (`CPL_*`) added as a coloc factor; `PROX_FLAG_TXLO_MISCAL` surfacing. Acceptance: dorm-geometry bench test — phone parked 10 ft, watch at desk: zero false NEAR over 2 h including a deliberate stationary-in-fade placement; phone picked up and pocketed while walking: NEAR within 2 ticks.

### Phase P4 — Reversed-link CSI + FTM (only after S4/S5 pass; do not pre-build)

**CSI (reversed link; engine spec §3.5).** Prerequisite: the anchor's pioarduino/IDF 5.x migration (existing §10.1 item — do it on its own branch first) built with `CONFIG_ESP_WIFI_CSI_ENABLED=y`. Then:

- **Watch** (stock 4.4.7, TX only): implement `prox_platform_espnow_ping(channel, CSI_PING_FRAMES)`; call it at the start of each observation window when the target anchor's WiFi channel is known. Frames carry the watch MAC + a nonce. Channel source: same-AP association when available, else the anchor's WiFi Status channel byte (below).
- **Anchor**: register the CSI callback; filter on the querying watch's MAC; reduce each frame in-callback to $(\mu_{sc}, \text{FS}, \text{notch})$ in fixed point (α-max-β-min amplitude, no sqrt, raw CSI never buffered); feed `prox_ingest_csi()`. Two new LR terms in `prox_compute_score2` with §4.10.8-style trained near/away distributions (calibration legs train them like every other feature; defaults `CSI_DEF_*` until then). Each term abstains when no tagged frames arrived for the query.
- **Wire (P4 lockstep batch):** WiFi Status `…000E` appends `[1 byte: wifi_channel]` (1–13; 0 = not connected) after `schedule_crc`. No proximity-vector change — the features are anchor-side. Flag lockstep in MOBILE_APP_SPEC §0 as usual.
- **Acceptance:** with the beacon schedule disabled (isolating CSI), score separation between the two calibration positions improves measurably over PDR-off/spread-off baseline; anchor heap flat over 24 h of query load; a watch with no same-channel path degrades silently (terms abstain, P2 behavior byte-identical).

**FTM:** trigger-gated ambiguity-dwell tie-breaker via `prox_ingest_ftm()`; anchor APSTA FTM responder; gated entirely on S5. Wire TBD at that point.

### Cross-cutting rules for the agent

- **proximity.cpp modularity is inviolate** (master spec's modularity note): no HMM/integrator/feature code outside the shared module; firmware files implement only the documented seams.
- **No FPU on hot paths:** any new `log/exp/sqrt` at tick rate is a defect; use the LUTs.
- **Fail-open bias preserved:** every new abstention/unknown path must resolve toward the v0.8 criterion-satisfying behavior; grep-test that AMBIGUOUS handling call sites are unchanged.
- **Shadow-then-flip:** each phase runs its new decision path in shadow (logged, non-authoritative) on hardware before the authority flag flips.
- **Parity table + change log** updated at each phase landing, per §0 rules; P2's app-side changes are flagged lockstep in MOBILE_APP_SPEC §0.
