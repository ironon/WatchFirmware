# Impulse Proximity Engine — Design Specification

**Version:** 1.0
**Scope:** All RF-proximity logic for the Impulse watch and anchor system.
**Module:** Every algorithm described here is implemented in a single shared pair of files, `proximity.cpp` / `proximity.h`. No other firmware file contains proximity logic; they call only the public API in `proximity.h`. Role-specific code is gated with `PROXIMITY_ROLE_WATCH` / `PROXIMITY_ROLE_ANCHOR` compile-time switches.

---

## 0. Overview

The proximity engine answers spatial questions from radio signal strength (RSSI) alone, on devices with no FPU and tight power budgets. It provides three related capabilities:

| Mode | Question answered | Primary signal | Where it runs |
|------|-------------------|----------------|---------------|
| **A. Multi-location fingerprinting** | "Which of my N anchor locations is the watch nearest?" | Watch + anchor RF backgrounds | Anchor computes, watch queries |
| **B. Binary co-location** | "Is the watch within ~1–3 ft of the phone (or a parked phone's anchor)?" | Direct watch↔phone RSSI + RF environment | Watch, with optional anchor assist |
| **C. Dorm differential ranging** | "Is the user at their phone, or across a small room (8–12 ft) from it?" | Direct watch↔phone RSSI + two-anchor RSSI difference | Watch, with two anchors |

All three share the same underlying physics and a common fusion approach (probabilistic likelihood ratios), so they live in one module and reuse one another's primitives.

---

## 1. Shared Foundations

### 1.1 Log-distance path loss

All modes rest on the log-distance path loss model:

$$\text{RSSI}(d) = A - 10\,n \log_{10}(d) + X_\sigma$$

where $A$ is the reference RSSI at unit distance, $n$ is the path loss exponent ($n \approx 2$ in free space / clean line-of-sight, $n \approx 3\text{–}4$ in cluttered indoor space), $d$ is distance, and $X_\sigma$ is zero-mean Gaussian shadowing/multipath noise with standard deviation $\sigma \approx 3\text{–}5$ dB per single reading indoors.

The decisive property is that the gradient grows as distance shrinks:

$$\frac{d(\text{RSSI})}{dd} = -\frac{10\,n}{d \ln 10} \;\propto\; \frac{1}{d}$$

RSSI carries the most information in the near field, which is exactly why fine distinctions at short range are answerable while far-field distinctions are not.

### 1.2 Ratio invariance

Because the model is logarithmic, the *separation* between two distance bands depends only on the ratio of distances, not their absolute values, and the unknown reference term $A$ cancels:

$$\Delta\text{RSSI} = 10\,n \log_{10}\!\left(\frac{d_2}{d_1}\right)$$

This is why thresholds based on signal *differences* are far more robust than thresholds on absolute RSSI: the hardware- and room-dependent term $A$ drops out.

---

## 2. Mode A — Multi-Location Fingerprinting

The watch determines which anchor location it is nearest by comparing its current RF background against each anchor. Each anchor owns its own fingerprint and computes its own score; the watch submits a scan vector and reads back a score.

### 2.1 Scan vector

The watch assembles a `ProxScanVector`: a list of every observed RF emitter as `{mac[6], type (BLE/WiFi), rssi}`, capped at `PROX_MAX_DEVICES` (strongest-RSSI entries kept on overflow). This includes all BLE devices and all WiFi APs, not just Impulse anchors — every stable emitter adds discriminating power.

**Assembly mechanics (watch side).** The vector is built from two caches, not a fresh blocking scan of both radios per query:

- **BLE:** a rolling cache of every advertisement seen within `PROX_CACHE_STALE_MS` (entries older than this are dropped). Immediately before each query the watch runs one bounded, **active, full-duty** scan (`ENFORCEMENT_QUERY_SCAN_DURATION_MS`) to refresh the cache so it reflects "now." That same aligned scan is also the mechanism that (re)discovers the target anchor's BLE MAC — it must run *before* the MAC is looked up, since the watch cannot issue a directed connect to an anchor whose MAC it has never captured.
- **WiFi:** AP scan results are reused from a cache refreshed at most every `PROX_WIFI_SCAN_INTERVAL_MS`. Because APs are stationary, rescanning every query wastes radio; and the WiFi scan is **skipped entirely when the watch is not associated** (`WiFi.status() != WL_CONNECTED`), in which case the vector is BLE-only. Anchor proximity remains fully functional on BLE alone.

### 2.2 Device registry (anchor side)

Each anchor maintains an ordered registry of every `{mac, type}` it has ever seen, capped at `ANCHOR_PROX_MAX_FINGERPRINT_DEVICES`, persisted to NVS. Registry index is stable for the storage lifetime and is what fingerprint statistics are keyed on.

### 2.3 Continuous background scanning (anchor side)

Two looping tasks keep a live RSSI cache current: a BLE scan (`ANCHOR_PROX_BLE_SCAN_DURATION_MS` every `ANCHOR_PROX_BLE_SCAN_INTERVAL_MS`) and a WiFi AP scan (every `ANCHOR_PROX_WIFI_SCAN_INTERVAL_S`). Cache entries unseen for `ANCHOR_PROX_DEVICE_STALE_MS` are marked stale and excluded from scoring.

### 2.4 Score = blend of two signals

**Signal A — live Pearson correlation.** Over the set of devices present in both the watch vector and the anchor's non-stale cache:

$$\rho = \frac{\sum_d (w_d - \bar{w})(a_d - \bar{a})}{\sqrt{\sum_d (w_d - \bar{w})^2}\,\sqrt{\sum_d (a_d - \bar{a})^2}}, \qquad \rho \to \text{clamp}[0,1]$$

This requires **zero training** and works the instant an anchor is installed: it asks "does the watch see what the anchor sees right now?"

**Signal B — fingerprint log-likelihood.** When the fingerprint holds at least `PROX_MIN_DEVICE_COUNT` devices with cumulative weight $W_d \ge$ `PROX_MIN_FINGERPRINT_WEIGHT`, compute the Gaussian Naive Bayes log-likelihood of the watch vector:

$$\log P = \sum_d \left[ -\tfrac{1}{2}\log(2\pi\sigma_d^2) - \frac{(w_d - \mu_d)^2}{2\sigma_d^2} \right]$$

with absent devices substituted by sentinel `PROX_MISSING_RSSI_DBM`. The average per-device log-likelihood is mapped to $L \in [0,1]$ via a logistic.

**Blend.** The weight shifts from correlation to fingerprint as the fingerprint accumulates total weight $W_{\text{total}}$:

$$\alpha(W_{\text{total}}) = e^{-W_{\text{total}} / W_0}, \qquad \text{score} = \alpha\,\rho + (1-\alpha)\,L$$

At $W_{\text{total}} = 0$, $\alpha = 1$ (pure correlation). As weight grows, the fingerprint dominates. The score is scaled to a `uint8` $[0,255]$.

### 2.5 Self-supervised training

When a score is both high (`score/255 ≥ PROX_COLLECT_SCORE_THRESHOLD`) and **unambiguous** (this anchor beats every competing anchor's raw RSSI in the watch vector by `PROX_COLLECT_AMBIGUITY_MARGIN_DBM`), the watch vector is accepted as a training sample. The fingerprint updates via **weighted Welford** with sample weight $w_n = \text{score}/255$:

$$W_n = W_{n-1} + w_n$$
$$\mu_n = \mu_{n-1} + \frac{w_n}{W_n}(x_n - \mu_{n-1})$$
$$M_n = M_{n-1} + w_n (x_n - \mu_{n-1})(x_n - \mu_n)$$
$$\sigma_n^2 = \frac{M_n}{W_n}, \quad \text{clamped to } \ge \text{PROX\_MIN\_VARIANCE}$$

The variance floor is essential: a stationary anchor would otherwise drive $\sigma^2 \to 0$ and become catastrophically overconfident. No raw samples are ever stored — the fingerprint *is* $(\mu, \sigma^2, W, M)$ per device. The optional manual "walk around" calibration simply injects many high-weight samples quickly, fast-forwarding what self-supervision does slowly.

### 2.6 Fail-safe / low device count

If the watch vector has fewer than `PROX_MIN_DEVICE_COUNT` devices, the engine sets `PROX_FLAG_LOW_DEVICE_COUNT` and falls back to a single raw RSSI comparison against the target anchor's own MAC (score 200 if that MAC's RSSI ≥ `ANCHOR_NEAR_RSSI_THRESHOLD_DBM`, else 50; 50 if the anchor isn't in the vector at all).

### 2.7 Watch-side interpretation, transport & discovery

The anchor returns a `uint8` score; the watch maps it to a three-way decision:

- `score ≥ PROX_CONFIDENCE_THRESHOLD_U8` → **NEAR**
- `score ≤ 255 − PROX_CONFIDENCE_THRESHOLD_U8` → **AWAY**
- otherwise → **AMBIGUOUS**, resolved toward whichever outcome keeps the active enforcement criterion satisfied (favouring few false-positive enforcement events).

**Transport.** The score is fetched over a directed BLE GATT connection to the anchor, using the anchor's advertised MAC **and advertised address type** (public vs random — they must match or the connect fails). The MAC/type, once captured from an advertisement, is **persisted to NVS** so the watch can connect on boot without re-approaching the anchor to rediscover it. TX power is boosted (≈ +9 dBm) because BLE connection establishment fails at meaningfully weaker RSSI than advertisement reception. The watch stops its background scan before initiating the connect (the radio cannot reliably scan and initiate simultaneously).

**Connect-failure fail-safe.** When the query cannot complete — anchor MAC still unknown after the aligned discovery scan, or the connection times out — the watch does not blindly return AMBIGUOUS. Because connection establishment fails (~−88 dBm) well before advertisement reception does, a failed connect *combined with* a recent advertisement RSSI from the target anchor at or below `PROX_FAR_RSSI_THRESHOLD_DBM` is treated as strong evidence the watch is **far** (resolves to AWAY). This correctly drives `stayNear` enforcement when the user walks out of connection range, rather than silently treating "can't connect" as "near." If no recent advertisement is available, it falls back to the criterion-satisfying AMBIGUOUS direction.

---

## 3. Mode B — Binary Co-Location Detection

**Goal:** decide, with minimal false positives and false negatives, whether the user is within ~1–3 ft of their phone. Used to enforce phone-free blocks of time. This is *easier* than Mode A because it is binary and there is a direct ranging signal between watch and phone.

**Shipped implementation (docked variant).** The primary product scenario docks the phone at an anchor, which reduces Mode B to Mode A plus a docking check, and avoids the phone's RF-scanning limits entirely (§3.5):
- The watch enforces distance from the **docking anchor** using the Mode A query (proximity to the anchor ≈ proximity to the docked phone). This is the `phoneAway` criterion (firmware spec §5.4.1).
- The **anchor** confirms the phone is docked by holding a persistent BLE link to it and thresholding that link's RSSI (firmware spec §4.11), reporting a `docked` flag the watch reads during its query. The watch fuses them as `near_phone = undocked OR (prox == NEAR)`, so both "user goes to the dock" and "phone leaves the dock" are caught.
- Fail-open on a degraded watch↔anchor link; a brief approach *or* brief undock is tolerated (`PHONE_AWAY_TOLERANCE_S`).

The full two-factor fusion below (direct watch↔phone RSSI + environment co-location) is the fallback for a *bare-phone* Mode B (no docking anchor) and the foundation Mode C builds on.

### 3.1 Two independent factors

The detector fuses two signals that fail in different ways:

| | Strength | Failure mode |
|---|----------|--------------|
| **Factor 1: direct watch↔phone RSSI** | Highest resolution; near-field gradient is steep | Body shadowing (phone in back pocket) can mimic distance |
| **Factor 2: RF environment co-location** | Immune to body shadowing | Saturates in small/sparse environments |

Because their failure modes are independent, fusing them covers each other's blind spots.

### 3.2 Factor 1 — direct RSSI likelihood ratio

The watch↔phone RSSI $r$ is read directly off the BLE connection (no scan needed). Smooth with an EWMA ($\alpha \approx 0.3$) to suppress multipath spikes, then evaluate as a likelihood ratio between the two hypotheses:

$$\text{LR}_{\text{range}} = \frac{\mathcal{N}(r;\,\mu_{\text{near}}, \sigma_{\text{near}}^2)}{\mathcal{N}(r;\,\mu_{\text{away}}, \sigma_{\text{away}}^2)}$$

### 3.3 Factor 2 — environment co-location via difference variance

For each external emitter $d$ seen by **both** watch and phone, take the difference of their readings:

$$\Delta_d = w_d - p_d$$

Each reading is $P_d - 10n\log_{10}(\text{dist}_d) + \text{offset}$. If watch and phone are 1–3 ft apart, their distances to any *external* emitter tens of feet away are nearly equal, the path-loss terms cancel, and

$$\Delta_d \approx c + \varepsilon \quad \forall d$$

i.e. a constant hardware offset $c$ plus noise. Co-location therefore appears as **low spread of $\Delta_d$ across devices**, automatically invariant to $c$. The statistic and its likelihood ratio:

$$s = \operatorname{std}_d(\Delta_d), \qquad \text{LR}_{\text{env}} = \frac{p(s \mid \text{near})}{p(s \mid \text{away})}$$

This is more physically grounded than a raw correlation because it assumes the *correct* slope of exactly 1 between the two views, rather than any positive slope.

### 3.4 Fusion — Naive Bayes likelihood ratio

The factors fail independently, so multiply their likelihood ratios (equivalently, sum log-LRs):

$$\text{odds}_{\text{near}} = \frac{\pi_{\text{near}}}{\pi_{\text{away}}}\cdot\text{LR}_{\text{range}}\cdot\text{LR}_{\text{env}}, \qquad P(\text{near}) = \frac{\text{odds}_{\text{near}}}{1 + \text{odds}_{\text{near}}}$$

Declare NEAR when $P(\text{near}) > \tau$. Raising $\tau$ trades false negatives for fewer false positives.

### 3.5 Phone scanning limits → use the anchor for Factor 2

Phones are poor RF scanners: iOS gives apps no WiFi scans and restricts BLE; both platforms randomize BLE MACs, so watch and phone may see the same physical device under different addresses and fail to match it. **When the phone is parked at a known anchor, compute Factor 2 against the *anchor's* RF background instead** — clean ESP32-to-ESP32 matching using the Mode A engine — and confirm the phone↔anchor pairing separately via their direct RSSI. When matching against a phone's limited view is unavoidable, prefer emitters with *stable advertised identifiers* (own anchors, iBeacon UUIDs, fixed BLE service UUIDs, WiFi BSSIDs on Android) over raw MACs.

---

## 4. Mode C — Dorm-Room Differential Ranging

**Target scenario:** a student in a small dorm room parks their phone 8–12 ft away (likely line-of-sight) during a phone-free block, and must not trigger false positives while compliant, nor false negatives if they pick the phone up.

### 4.1 The margin you actually have

Mean separation between the 1–3 ft "near" band and the 8–12 ft "away" band (using midpoints 2 ft and 10 ft):

$$\Delta\text{RSSI}_{\text{mean}} = 10\,n\log_{10}\!\left(\frac{10}{2}\right) = 10\,n\log_{10}(5) \approx 7.0\,n \text{ dB}$$

≈ 14 dB at $n=2$, ≈ 21 dB at $n=3$. But the error rate is governed by the **worst-case adjacent-edge gap** (3 ft vs 8 ft):

$$\Delta\text{RSSI}_{\text{edge}} = 10\,n\log_{10}\!\left(\frac{8}{3}\right) \approx 4.3\,n \text{ dB}$$

≈ 8.5 dB at $n=2$, up to ~13 dB at $n=3$. With single-reading noise $\sigma \approx 3\text{–}5$ dB, one instantaneous sample only yields ~2σ of separation — the source of errors. The job of Mode C is to turn that ~8 dB into a reliable decision.

### 4.2 Why Factor 2 is demoted here

In a small room, the external emitters Factor 2 relies on are 20–40 ft away. The path-loss cancellation in $\Delta_d$ holds essentially as well at 1 ft as at 12 ft of intra-room separation, so $\operatorname{std}_d(\Delta_d)$ is dominated by per-device noise rather than the user's position. Factor 2 can confirm "same room" but is **blind to within-room distance**. It is kept only as a gross same-room sanity check; the fine distinction rests on ranging.

### 4.3 Lever 1 — temporal integration (largest single win)

The decision is not time-critical, so integrate over a 15–30 s window. Averaging $N$ readings reduces effective noise:

$$\sigma_{\text{eff}} = \frac{\sigma}{\sqrt{N_{\text{eff}}}}$$

where $N_{\text{eff}} < N$ because consecutive readings are correlated (fading decorrelates over a few hundred ms of movement). A conservative $N_{\text{eff}} \approx 10\text{–}30$ pulls $\sigma_{\text{eff}}$ to ~1 dB, converting the 8.5 dB worst-case gap into ~8σ of clean separation. Integration suppresses *both* error types: a multipath spike from the parked phone cannot survive 30 s of averaging (no false positive), and a user cannot hold the phone for 30 s and average out to "across the room" (no false negative). BLE's 37-channel frequency hopping means time-averaging also averages over frequency-selective fading for free.

### 4.4 Lever 2 — differential ranging with two anchors (robustness multiplier)

Place the phone-parking anchor $P$ on one side of the room and a second anchor $D$ at the desk where the student studies. The watch measures RSSI to both and uses the **difference**:

$$\delta = R_P - R_D$$

On the phone side $\delta$ is strongly positive; at the desk it flips negative; the room midpoint is a roughly constant crossover. Why this beats absolute thresholding:

- **Common-mode cancellation.** Body shadowing, ambient dorm-WiFi 2.4 GHz noise, and the watch's own antenna gain affect $R_P$ and $R_D$ alike and subtract out of $\delta$.
- **No absolute calibration needed.** Only the sign and relative magnitude of $\delta$ matter — precisely the stable part of RSSI, not the hardware-dependent absolute level.
- **Both references are ESP32s,** immune to the phone's scanning and MAC-randomization limits.

Direct watch↔phone RSSI remains the primary "phone in hand" signal; $\delta$ robustly confirms which half of the room the watch is in. The "pocket the phone but stay at the desk" cheat is caught because direct watch↔phone RSSI rises regardless of anchor geometry.

### 4.5 Lever 3 — calibration (mandatory at these margins)

Hardcoded thresholds underperform because $A$ and $n$ vary widely across hardware and rooms. A 20 s two-point routine — "hold your phone normally," then "set it where you'll park it and step to your desk" — directly measures the near and away distributions for that user, hardware, and room (including the true LOS path-loss exponent). The decision threshold is then set at the statistically optimal crossover between the measured Gaussians, biased toward the preferred error. Ship path-loss defaults so it works uncalibrated; treat calibration as an accuracy boost (often widening the clean-LOS gap to 12–15 dB).

### 4.6 Lever 4 — RSSI variance & IMU coupling (confirming feature)

A phone in the same hand / on the same body as the watch moves with it as a near-rigid pair: direct RSSI shows **high mean, low variance**. A parked phone shows **low mean** regardless of variance. So $\operatorname{Var}(R_{wp})$ over the window is a useful confirming feature — high-mean + low-variance is a strong "holding" signature. Optionally, correlating RSSI flatness against watch IMU acceleration directly tests physical coupling (phone moving in lockstep) — a v2 enhancement.

### 4.7 Fusion & hysteresis

All active factors produce log-likelihood ratios summed into log-odds, passed through a logistic to $P(\text{near})$. A two-threshold hysteresis state machine ($\tau_{\text{high}}, \tau_{\text{low}}$) with a debounce counter prevents the decision from flapping: the state flips to NEAR only after $P > \tau_{\text{high}}$ persists for `COLOC_DEBOUNCE_SAMPLES`, and back to AWAY only after $P < \tau_{\text{low}}$ persists likewise.

### 4.8 Fail-safe direction

For an anti-distraction product, the trust-eroding error is buzzing a *compliant* student at their desk. The ambiguous band therefore resolves toward **away/compliant**, relying on the sustained-high "holding" signature (which integration makes very hard to miss) to catch genuine phone use.

---

## 5. Unified Decision Pipeline

```
            ┌──────────── per-reading (≈1 Hz) ────────────┐
  BLE/WiFi  │                                              │
  scan  ───▶│  ingest_scan_result()  → live RSSI buffers   │
  conn RSSI │  ingest_link_rssi(R_wp, R_P, R_D)            │
            │       │                                       │
            │       ▼  EWMA smoothing + ring buffers        │
            └───────┼───────────────────────────────────────┘
                    ▼  (on decision tick, every COLOC_DECIDE_INTERVAL_S)
        ┌─────────────────────────────────────────────┐
        │ Factor 1  LR_range(  integrated R_wp )        │
        │ Factor 2  LR_env(    std of Δ_d )  [Mode B]   │
        │ Factor δ  LR_diff(   integrated δ ) [Mode C]  │
        │ Factor V  LR_var(    Var(R_wp) )              │
        └───────────────┬─────────────────────────────┘
                        ▼  sum log-LRs + prior → logistic
                  P(near) ──▶ hysteresis state machine ──▶ NEAR / AWAY
```

Mode A's `prox_compute_score()` plugs in as the anchor-side implementation of Factor 2 when a parked-phone anchor is available.

---

## 6. ESP32 Performance Notes

The ESP32-C3 has **no hardware FPU** — floating-point and especially `log`/`exp` are software-emulated. The engine is structured to keep transcendental calls off the hot path:

- **Fingerprint devices cache** $\log\text{-norm} = -\tfrac{1}{2}\log(2\pi\sigma^2)$ and $\text{inv2var} = 1/(2\sigma^2)$, recomputed only on Welford update. Per-query scoring is then pure multiply-add plus one `sqrt` for Pearson.
- **Co-location config caches** all Gaussian normalization constants at `coloc_finalize_calibration()`. Per-decision cost is a handful of multiply-adds and a single `exp` in the final logistic.
- All per-decision work is $O(\text{devices})$ over ≤ `PROX_MAX_DEVICES` entries — sub-millisecond even with software FP. The only real cost is the radio scan, already paid by Mode A.

---

## 7. Constants & Tunables

```
// ── Mode A: fingerprinting ───────────────────────────────────────────────
PROX_MAX_DEVICES                    = 60     // max entries in a scan vector
PROX_MIN_DEVICE_COUNT               = 8      // below this → raw-RSSI fallback
PROX_MIN_MTU_BYTES                  = 256    // min MTU for vector write
PROX_CONFIDENCE_THRESHOLD_U8        = 170    // (tunable) NEAR cutoff (≈0.667)
PROX_MISSING_RSSI_DBM               = -100   // sentinel for absent device
PROX_MIN_FINGERPRINT_WEIGHT         = 5.0    // min W for a device to score (Signal B)
PROX_MIN_VARIANCE                   = 4.0    // dBm² variance floor (≈2 dB σ)
PROX_ALPHA_W0                       = 2000.0 // (tunable) weight where α→e^-1
PROX_LL_CENTER                      = -3.0   // (tunable) logistic center, avg log-lik
PROX_LL_SCALE                       = 0.5    // (tunable) logistic scale
PROX_COLLECT_SCORE_THRESHOLD        = 0.75   // (tunable) min score to train
PROX_COLLECT_AMBIGUITY_MARGIN_DBM   = 10     // (tunable) margin over rival anchors
PROX_NVS_PERSIST_INTERVAL_S         = 300    // fingerprint persist period
ANCHOR_PROX_BLE_SCAN_INTERVAL_MS    = 2000
ANCHOR_PROX_BLE_SCAN_DURATION_MS    = 500
ANCHOR_PROX_WIFI_SCAN_INTERVAL_S    = 30
ANCHOR_PROX_DEVICE_STALE_MS         = 10000
ANCHOR_PROX_MAX_FINGERPRINT_DEVICES = 128
ANCHOR_NEAR_RSSI_THRESHOLD_DBM      = -70    // low-device-count fallback near/away cut
// ── Mode A: watch-side vector assembly, transport & interpretation ───────
PROX_CACHE_STALE_MS                 = 30000  // BLE advert cache retention for vector building
PROX_WIFI_SCAN_INTERVAL_MS          = 300000 // min interval between real WiFi AP scans (cached between)
ENFORCEMENT_QUERY_SCAN_DURATION_MS  = 700    // bounded active full-duty scan run before each query
PROX_FAR_RSSI_THRESHOLD_DBM         = -85    // recent advert ≤ this + connect failure ⇒ classify AWAY (far)
PROX_QUERY_TX_POWER_DBM             = 9      // boosted TX power for connection range

// ── Modes B/C: co-location ───────────────────────────────────────────────
COLOC_EWMA_ALPHA                    = 0.30   // per-reading smoothing
COLOC_WINDOW_SAMPLES                = 30     // integration window (~30 s @1 Hz)
COLOC_DECIDE_INTERVAL_S             = 2      // decision tick period
COLOC_DEBOUNCE_SAMPLES              = 3      // ticks before a state flip commits
COLOC_TAU_HIGH                      = 0.80   // P(near) to flip → NEAR
COLOC_TAU_LOW                       = 0.45   // P(near) to flip → AWAY
COLOC_PRIOR_NEAR                    = 0.5    // default prior
COLOC_ENV_MIN_SHARED_DEVICES        = 4      // below this, drop Factor 2
COLOC_ENV_RSSI_FLOOR_DBM            = -85    // ignore weak shared devices in Δ
// Default (uncalibrated) distributions, overwritten by calibration:
COLOC_DEF_RWP_NEAR_MU               = -55.0  // dBm at ~1–3 ft
COLOC_DEF_RWP_NEAR_SIGMA            = 5.0
COLOC_DEF_RWP_AWAY_MU               = -72.0  // dBm at ~8–12 ft LOS
COLOC_DEF_RWP_AWAY_SIGMA            = 6.0
COLOC_DEF_S_NEAR_MU                 = 2.0    // std(Δ) when co-located
COLOC_DEF_S_NEAR_SIGMA              = 1.5
COLOC_DEF_S_AWAY_MU                 = 7.0
COLOC_DEF_S_AWAY_SIGMA              = 3.0
COLOC_DEF_DELTA_NEAR_MU             = 8.0    // R_P - R_D at phone side
COLOC_DEF_DELTA_NEAR_SIGMA          = 5.0
COLOC_DEF_DELTA_AWAY_MU             = -8.0   // at desk
COLOC_DEF_DELTA_AWAY_SIGMA          = 5.0
COLOC_VAR_NEAR_MU                   = 2.0    // Var(R_wp) when held
COLOC_VAR_NEAR_SIGMA                = 2.0
COLOC_VAR_AWAY_MU                   = 6.0
COLOC_VAR_AWAY_SIGMA                = 4.0
```

---

## 8. `proximity.cpp` Public API & Integration Seams

Algorithm code is wholly contained in `proximity.cpp`. Platform glue (BLE/WiFi scanning, NVS, clock) is injected through these documented seams so the module stays portable and testable:

```c
// Provided by the platform, called by proximity.cpp:
uint32_t prox_platform_now_ms(void);
bool     prox_platform_nvs_load(const char* key, void* buf, size_t* len);
bool     prox_platform_nvs_save(const char* key, const void* buf, size_t len);

// Called by the platform scan callbacks, feeding proximity.cpp:
void prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi); // anchor cache / watch vector
void coloc_ingest_link_rssi(int8_t r_wp, bool have_pd, int8_t r_p, int8_t r_d); // co-location ranging
```

The public algorithm API (full signatures in `proximity.h`) covers: Mode A scoring and training (`prox_compute_score`, `prox_maybe_update_fingerprint`, `prox_load_fingerprint`), watch-side vector assembly (`prox_build_scan_vector`), and the co-location pipeline (`coloc_init`, `coloc_tick`, `coloc_decision`, plus the calibration entry points `coloc_calib_begin/add_sample/finalize`).
