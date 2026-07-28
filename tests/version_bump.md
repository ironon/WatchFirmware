# Toolchain-bump spike — results & write-up

> Branch: `spike/toolchain-bump` (local, not pushed; do not merge). Off `calibration-v2`.
> Target hardware: Impulse watch (ESP32-C3, 4 MB flash, 320 KB RAM, USB-Serial/JTAG).
> This file is a self-contained handoff for the next agent. It covers the question, the
> result, every blocker hit and how it was fixed, the probe, the test log, and open issues.

## TL;DR

- **Spike question:** does upgrading the watch firmware from the frozen official toolchain
  (Arduino-ESP32 2.0.17 / ESP-IDF 4.4.7) to the pioarduino fork (Arduino-ESP32 3.3.11 /
  ESP-IDF 5.5.5) let the chip hold **peripheral + central + WiFi simultaneously, without
  crashing**? On IDF 4.4 this three-way concurrency asserted `ble_hs_timer_exp` and
  rebooted, which forced the "Option A" calibration workaround (watch only central-connects
  to the anchor while the phone is disconnected).
- **RESULT: FAIL — the bump does NOT fix the crash.** With the config adjusted so the central
  connect actually succeeds (see §7), the proper three-way test (phone connected + central
  connect to anchor + WiFi up) crashes in **`ble_hs_timer_exp`** (ble_hs.c:470) →
  `ble_gap_update_next_exp` (ble_gap.c:1460) — the **same NimBLE host-timer code path** as the
  original IDF 4.4 crash. It just manifests differently: a Load access fault (dereferencing a
  corrupted `ble_gap_update_entries` linked-list pointer, `MTVAL=0x42a0`) instead of an assert
  message. Reproducible: phone connects → ~4 s → `Guru Meditation Error: Core 0 panic'ed
  (Load access fault)` → reboot, every time.
- **Option A must stay.** The bump is not viable for retiring Option A with NimBLE-Arduino
  2.5.0. The crash is a genuine NimBLE concurrency bug (central+peripheral+WiFi corrupts the
  gap-update entry list), not an artifact of the spike's config — the backtrace is entirely
  inside NimBLE's host timer, and the config knobs (MAX_ACT, WiFi buffers) don't touch it.

### How we got here (the two sub-problems that had to be solved to reach the real test)
1. **rc=519** (first attempt, `MAX_ACT=3`): every central connect was rejected by the
   controller with HCI "Memory Capacity Exceeded". Fixed by raising `CONFIG_BT_CTRL_BLE_MAX_ACT`
   3 → 5 (with `MAX_EVENTS_PER_DAY=24` absorbing the heap cost). After this, the central
   connect succeeds (`Connected in ~400 ms`). See §7.
2. **The real crash** (once central connects): `ble_hs_timer_exp` → `ble_gap_update_next_exp`
   dereferences a corrupted pointer and panics. This is the actual three-way concurrency bug,
   now confirmed present on IDF 5.5. See §6.

## 1. The question and why it mattered

The watch must sometimes be a BLE **peripheral** (to the phone/app), a BLE **central** (to
an "anchor"), and a WiFi **station** all at once. On the frozen toolchain (IDF 4.4.7) this
three-way concurrency crashed the NimBLE host with `ble_hs_timer_exp` almost immediately.
That crash is the sole reason calibration uses "Option A": the app starts calibration,
**disconnects the phone**, the watch then central-connects to the anchor autonomously, and
the app reconnects afterward to read the result. Option A is fragile and masks three
separate bugs (a WiFi-scan heap leak, live-connected calibration, and a class of
calibration boundary-collision bugs).

The spike's job: get the watch firmware building on pioarduino, flash it, and reproduce the
previously-crashing three-way concurrency to see if the new stack survives. If yes → delete
Option A and fix the three bugs. If it still crashes → abandon the bump.

## 2. Final toolchain

- **Platform:** `pioarduino` fork, tag `55.03.311` → Arduino-ESP32 **3.3.11** / ESP-IDF
  **5.5.5**. (The task originally named the `54.03.xx` line / IDF 5.4; the user chose the
  newest stable `55.03.311` for a more decisive result. The `54.03.21-2` tag is a valid
  fallback if `55.03.311` proves problematic.)
  `platformio.ini`:
  ```
  platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.311/platform-espressif32.zip
  ```
- **NimBLE-Arduino:** `^2.0.0` resolved to **2.5.0** (unchanged from baseline; compatible
  with Arduino 3.x, with caveats below).
- **PlatformIO core:** 6.1.19 — no upgrade needed.
- Zero source changes were required for Arduino 3.x / IDF 5.x **API compatibility** at
  build time. The codebase uses no LEDC; `analogRead`, `esp_wifi_scan_*`, `esp_wifi_set_ps`
  all compiled unchanged. All blockers were **runtime/behavioral** (see §4).

## 3. The coexistence probe (`COEX_SPIKE`)

A temporary, self-contained probe was added to `src/main.cpp` (all edits tagged `// SPIKE:`),
gated by `-DCOEX_SPIKE` in `build_flags` and `#ifdef COEX_SPIKE` in code. It does NOT touch
the app/anchor.

- Function: `coex_spike_tick(uint32_t now_ms)`, called from `loop()` before the state machine.
- Behavior: once `g_bt_connected && g_wifi_connected`, it arms and runs a ~3 s cadence loop,
  15 iterations. Each iteration: `prox_aligned_active_scan()` → pick a known anchor (first
  valid in `g_anchor_records`/`g_seen_anchors`) → `find_anchor_ble_addr()` →
  `prox_feed_wifi_aps()` + `prox_build_scan_vector()` → `prox_query_anchor()` (the central
  connect + GATT exchange — **the forbidden three-way call**) → log and repeat. It reuses
  the existing transport machinery in `src/watch_prox_transport.cpp`.
- Logs (prefix `[SPIKE]`): arm message, per-iter `central_connect=OK/FAIL
  phone_still_connected=… ad_rssi=… dt=…ms score=… flags=…`, and a final
  `DONE — 15 iterations complete … (PASS)` or `ABORT … link dropped (likely FAIL)`.
- The probe is **inert unless phone+WiFi are both connected**, so it never interferes with
  normal operation or with calibration (which disconnects the phone — Option A).

## 4. Blockers hit and how each was fixed (the journey)

The build compiled clean on the first try, but the firmware would not run on the watch.
Four sequential runtime blockers, each independent:

### 4.1 Flash-size overflow (build-time, trivial)
IDF 5.5 framework is ~330 KB larger than IDF 4.4. The default 1.25 MB app partition
overflowed (104 %). Fix: `board_build.partitions = huge_app.csv` (3 MB app on 4 MB flash).
The firmware uses NVS only (no SPIFFS/OTA on this path), so this is safe and behavior-neutral
for the concurrency test. `huge_app.csv` was copied into the project root because the
`custom_sdkconfig` IDF-lib rebuild resolves `board_build.partitions` relative to the project.

### 4.2 BOD boot brownout (firmware never reached app code)
Symptom: `E BOD: Brownout detector was triggered` + `rst:0x3 (RTC_SW_SYS_RST)` in a tight
boot loop, before any app output. The old IDF 4.4 firmware boots fine on the same hardware
(verified by restoring the official `espressif32@6.12.0` platform and flashing it —
`[SYS] Boot complete`, battery 4.30 V).
Root cause: the IDF 5.5 app startup (radio/PHY init during `app_main`) sags the watch's 3.3 V
rail below BOD **level 7 (~2.94 V, the most-sensitive threshold)**. Reducing flash to
DIO/40 MHz did **not** help — it's intrinsic to the IDF 5.5 boot inrush on this watch's
power path.
Fix: disable BOD via pioarduino's `custom_sdkconfig` mechanism (which rebuilds the IDF libs
from source with custom Kconfig). `sdkconfig.spike`:
```
CONFIG_ESP_BROWNOUT_DET=n
CONFIG_ESP_BROWNOUT_USE_INTR=n
CONFIG_BROWNOUT_DET=n            # deprecated IDF-4 alias — MUST also be unset
CONFIG_ESP32C3_BROWNOUT_DET=n    #   (the aliases `select` the new symbol and
CONFIG_ESP_SYSTEM_BROWNOUT_INTR=n #    force it back on if left =y)
```
Gotcha that cost a rebuild: setting only the new `CONFIG_ESP_BROWNOUT_DET=n` does NOT work —
the deprecated IDF-4 alias symbols (`CONFIG_BROWNOUT_DET` etc.) `select` the new symbol and
force it back to `y`. All five must be unset.
> Caveat: BOD is disabled in this build. A concurrency crash that presents as a brownout
> would be masked. The original failure was a NimBLE **assert**/Guru Meditation, which is
> not masked — so the test remains meaningful. Re-enable BOD (and fix the power path)
> before any production use.

### 4.3 NimBLE init failure → watch invisible in LightBlue/app
After BOD was disabled the watch booted, but `[BLE] Watch advertising started` was a lie:
`NimBLEAdvertising::start()` returns `false` when `!NimBLEDevice::m_synced`, and the host
never synced because `NimBLEDevice::init()` itself returned **FAIL**.
Root cause: **heap exhaustion.** IDF 5.5's runtime is much heavier than IDF 4.4's; the app's
large static schedule buffers left only **~8–9 KB free heap**, and NimBLE needs ~30+ KB to
initialize. Top BSS symbols (from `riscv32-esp-elf-nm`): `g_all_events_scratch` 32 KB,
`g_proposed_scratch` 32 KB, `g_today_events` 16 KB, `recalculate_day::merged` 16 KB — ~96 KB
of event scratch. (`MAX_EVENTS_PER_DAY=64`, `sizeof(Event)=256` because of `beepAnchors[8][16]`.)
On IDF 4.4 the runtime was leaner, so NimBLE fit; IDF 5.5 pushed heap below the init floor.
Fix (two parts):
- `MAX_EVENTS_PER_DAY` **64 → 24** in `src/main.cpp` (frees ~50 KB BSS; heap jumped to ~40 KB).
  Spike-only behavior change: watch supports 24 events/day instead of 64.
- `sdkconfig.spike`: trim BLE controller (`MAX_ACT 6→3`, `MAX_CONNECTIONS 3→2`,
  `NVS_PERSIST off`) and WiFi buffers (static RX 8→4, dynamic TX/RX 32→8, BA 6→2). This
  freed only ~1 KB on its own — the app's schedule buffers were the real consumer — but
  it was left in as headroom.
- Also: `pAdv->start()` is now retried until the host syncs (IDF 5.5 sync is slower than
  4.4; the original single call raced and silently failed). The loop re-arms advertising
  every 5 s if it's off.
After this: `NimBLEDevice::init() -> OK`, `isAdvertising=yes`, watch visible in LightBlue.

### 4.4 WiFi `ESP_ERR_WIFI_STATE` ("sta is connecting, cannot set config")
After creds were saved, `WiFi.begin()` failed with `STA clear config failed! 0x3006`.
Root cause: two Arduino-ESP32 3.x behavior changes (2.x tolerated both):
1. **Stacked begins.** The BLE WiFi-cred characteristic callback runs on the **NimBLE host
   task**, while the DORMANT async reconnect runs on the **main loop task** — they can call
   `WiFi.begin()` concurrently. 3.x errors; 2.x didn't.
2. **Lingering driver state.** After a cred fails (the while-loop times out), the WiFi
   driver is *still* "connecting" to that SSID, so the next cred's `WiFi.begin()` can't set
   config. 2.x handled this internally; 3.x does not.
Fix: a `spike_wifi_begin(ssid, pass)` wrapper used in all three `WiFi.begin()` call sites:
- connecting-flag guard + 15 s timeout (prevents stacked begins);
- `WiFi.disconnect(false)` + `delay(50)` before each `WiFi.begin()` (stops any in-progress
  association so the new config can be set);
- the flag is cleared on both success (`on_wifi_associated`) and failure (after the while
  loop) so sequential cred attempts aren't blocked.
After this: `[WiFi] Connected to …`, `WiFi=1`, NTP syncs.

### 4.5 BLE debug instrumentation (to diagnose 4.3/4.4)
Added `// SPIKE:`-marked debug, all removable by `rg SPIKE`:
- setup(): log `NimBLEDevice::init()` result + retry `pAdv->start()` until synced.
- loop(): periodic `[SPIKE-BLE]` status (adv/scan/bt_connected/state/freeHeap), re-arm adv if off.
- scan callback: per-device log during a debug scan.
- serial commands (monitor `send_on_enter`): `ble` `adv` `advoff` `scan` `info` `heap` `help`.
- Do **not** call `NimBLEDevice::getAddress()` at boot — it triggers a lazy `esp_nimble_hci_init`
  that asserts (`npl_freertos_mutex_pend … mu->handle`) before the host task is ready.

## 5. How to build / flash / run

```bash
cd /home/ironon/Impulse/ImpulseFirmware/WatchFIrmware   # note the capitalization
git checkout spike/toolchain-bump
~/.platformio/penv/bin/pio run -t upload        # first build after editing sdkconfig.spike
                                                # does a full IDF-lib rebuild (~4 min) and
                                                # needs network (fetches IDF components)
~/.platformio/penv/bin/pio device monitor       # 115200; type a command + Enter
```
- The watch light-sleeps when DORMANT, which drops USB-CDC. **Press the reset button to
  upload** (the upload fails with `Protocol error` / "port busy" while it sleeps).
- pioarduino's platform has `name: espressif32`, so installing it **overwrites the official
  `espressif32` platform** in `~/.platformio/platforms/`. To restore the official 2.0.17
  platform: remove the dir and `pio pkg install -p "espressif32@6.12.0"`.

## 6. Test result (the real three-way concurrency test)

Conditions: phone connected via the app's **normal connection screen** (peripheral link,
*not* calibration — so the phone stays connected), WiFi associated ("Kompsci LAN"), anchor
powered nearby. With `MAX_ACT=5` (§7), the central connect now succeeds, so the actual
three-way concurrency is exercised. **It crashes.**

Representative log (reproducible every time the phone connects):
```
[SPIKE] phone(peripheral)+WiFi both connected — arming coexistence probe
[SPIKE] target anchor resolved
[PROX] Connecting to ac:a7:04:94:93:6a ...
[PROX] Connected in 400 ms (RSSI=-83), discovering service...
Guru Meditation Error: Core  0 panic'ed (Load access fault). Exception was unhandled.
Rebooting...
rst:0xc (RTC_SW_CPU_RST)
```
The crash follows the phone connecting by ~4 s (one probe iteration: scan → central connect
→ GATT/MTU exchange → host-timer tick → panic).

Decoded backtrace (`riscv32-esp-elf-addr2line` on `firmware.elf`):
```
ble_gap_update_next_exp   ble_gap.c:1460     ← crash (Load access fault, MTVAL=0x000042a0)
  ble_gap_update_timer    ble_gap.c:3042
    ble_gap_timer         ble_gap.c:3099
      ble_hs_conn_timer   ble_hs_conn.c:574
        ble_hs_timer_exp  ble_hs.c:470       ← the SAME function as the original IDF 4.4 crash
          nimble_port_run
            NimBLEDevice::host_task
```
The faulting line (ble_gap.c:1460) is `ticks = entry->exp_os_ticks - now;` inside
`SLIST_FOREACH` over the `ble_gap_update_entries` list — it dereferences `entry`, which is a
corrupted pointer (`0x42a0`). A `ble_gap_mtu_event` (the central connection's MTU exchange)
was being processed around the same time. So: central+peripheral+WiFi corrupts NimBLE's
gap-update linked list, and the next host-timer tick dereferences the bad pointer and panics.

**This is the same `ble_hs_timer_exp` crash as IDF 4.4**, just manifesting as a Load access
fault instead of an assert. The bump did not fix it.

## 7. rc=519 — resolved (was blocking the real test)

- First attempt (`MAX_ACT=3`): every central connect was rejected in 0–1 ms with
  `rc=519 = 0x207 = BLE_HS_ERR_HCI_BASE (0x200) + 0x07` = HCI **"Memory Capacity Exceeded"**.
  Cause: advertising + peripheral connection + central initiation = 3 BLE activities, exactly
  the `MAX_ACT=3` limit; the controller had no headroom to allocate the central connection.
- Fix: raise `CONFIG_BT_CTRL_BLE_MAX_ACT` 3 → 5 (and `MAX_CONNECTIONS` 2 → 3) in
  `sdkconfig.spike`. `MAX_EVENTS_PER_DAY=24` (§4.3) freed ~50 KB heap, which absorbs the
  larger controller activity table — NimBLE `init()` still succeeds, runtime heap ~35 KB.
  After this, the central connect succeeds (`Connected in ~400 ms`, `rc=519` gone).
- This fix is **necessary but not sufficient** — it unblocked the real test, which then hit
  the `ble_hs_timer_exp` crash (§6).

## 8. Conclusions

1. **The bump does NOT fix the three-way concurrency crash.** With the central connect made
   to succeed (`MAX_ACT=5`), the proper test (phone connected + central to anchor + WiFi)
   panics in `ble_hs_timer_exp` → `ble_gap_update_next_exp` — the same NimBLE host-timer
   path as the original IDF 4.4 crash. Reproducible (~4 s after phone connect, every time).
2. **Option A must stay.** The bump (pioarduino/IDF 5.5.5 + NimBLE-Arduino 2.5.0) is not
   viable for retiring Option A. The crash is a genuine NimBLE concurrency bug, not a config
   artifact (backtrace is entirely inside NimBLE's host timer).
3. **The spike's question is answered: NO.** Abandon the bump for the purpose of fixing the
   calibration concurrency crash. Keep the IDF 4.4 / Arduino 2.0.17 toolchain and Option A.
4. **The earlier "PASS" was an artifact of rc=519**: with `MAX_ACT=3` the central connect
   never succeeded, so the crash path was never reached — the probe completed 15 iterations
   simply because every central connect was rejected before it could trigger the bug. That
   was not a real pass; §6 is the real test.

## 9. Open issues / follow-ups for the next agent

1. **`ble_hs_timer_exp` crash is NOT fixed by the bump** (§6) — this is the spike's
   definitive finding. If the project still wants to retire Option A, the avenues are:
   - Try a **different NimBLE version**. The spike used NimBLE-Arduino 2.5.0 (`^2.0.0`).
     A newer 2.x point release or the 3.x line may have fixed the gap-update-list
     corruption. Check h2zero/NimBLE-Arduino changelog/issues for `ble_hs_timer_exp` /
     `ble_gap_update_next_exp` / concurrent-connection fixes.
   - Use **IDF's bundled NimBLE** (via `framework = arduino, espidf` with NimBLE as an
     IDF component) instead of the NimBLE-Arduino library — the IDF-bundled version may
     have fixes the Arduino wrapper lacks.
   - Investigate the **MTU exchange** angle: `ble_gap_mtu_event` appears in the backtrace
     near the crash. The probe calls `NimBLEDevice::setMTU(512)` before connecting (see
     `src/watch_prox_transport.cpp`). A smaller MTU or skipping the MTU request might
     avoid the corruption — worth a quick test, but it's a workaround, not a fix.
   - These are beyond this spike's scope (the spike was: does the bump as-is fix it → no).
2. **rc=519 is resolved** (`MAX_ACT=5`, §7) — kept in `sdkconfig.spike` as the working value.
3. **BOD disabled** (`sdkconfig.spike`). Re-enable for production; the watch's power path
   (LDO / bulk caps) can't handle the IDF 5.5 boot inrush at BOD level 7. Either lower the
   BOD threshold (e.g. level 0 ≈ 2.45 V) instead of disabling, or fix the hardware.
4. **`MAX_EVENTS_PER_DAY=24`** is a spike-only reduction (was 64). A real migration needs a
   proper RAM budget regardless (IDF 5.5 is RAM-heavy on this 320 KB part). The
   `beepAnchors[8][16]` field (128 B) dominates `sizeof(Event)=256`; the schedule scratch
   buffers (`g_all_events_scratch`, `g_proposed_scratch`) are ~32 KB each at 64 events.
5. **Pre-existing "target has no BLE MAC yet" calibration bug** — `find_anchor_ble_addr()`
   returns false even when the scan sees the anchor with a captured MAC. Seen in the earlier
   calibration run (`[CALIB] target has no BLE MAC yet … — retry` ×14, `accepted=0`). This is
   one of the calibration boundary-collision bugs Option A masks; it blocks calibration
   regardless of the bump. Not caused by the spike; needs its own investigation.
6. **ADC API warning** — `[E][esp32-hal-adc.c] Pin is not configured as analog channel` on
   `analogRead(BATT_ADC_PIN)` / `analogRead(IR_REC)`. Arduino 3.x wants `analogAttach`/pin
   config before `analogRead`. Non-fatal (battery still reports ~4.3 V, boot completes).
   One-line fix per `analogRead` call in a real migration.
7. **`huge_app.csv` drops OTA/SPIFFS.** Fine for the spike; a real migration needs a custom
   partition table that fits the larger IDF 5.5 app while keeping OTA.
8. **`NimBLEDevice::getAddress()` at boot asserts** (§4.5) — don't use it before the host is
   synced. If the BLE MAC is needed, read it via `esp_read_mac(buf, ESP_MAC_BT)`.

## 10. Files changed on `spike/toolchain-bump` (all reversible)

- `platformio.ini` — pioarduino platform URL, `huge_app.csv`, `custom_sdkconfig = file://sdkconfig.spike`, `-DCOEX_SPIKE`.
- `sdkconfig.spike` (new) — BOD disable (5 symbols) + BT/WiFi memory trim.
- `huge_app.csv` (new, copied from the framework) — needed by the `custom_sdkconfig` IDF rebuild.
- `src/main.cpp`:
  - `MAX_EVENTS_PER_DAY` 64 → 24.
  - `coex_spike_tick()` + its `loop()` call (`#ifdef COEX_SPIKE`).
  - BLE debug helpers + serial commands (`// SPIKE:`).
  - `spike_wifi_begin()` guard + `WiFi.disconnect(false)`; flag clear on success/failure;
    `WiFi.status()` in failure logs.
  - `pAdv->start()` retry-until-synced in setup; periodic adv re-arm in loop.

Commits (in order, on `spike/toolchain-bump`):
```
6139a84  spike: toolchain bump to pioarduino 55.03.311 + COEX_SPIKE probe
195ab4d  spike: disable BOD via custom_sdkconfig to fix IDF 5.5 boot brownout
550161d  spike: fix NimBLE init (heap exhaustion) + add BLE debug
6554a31  spike: fix WiFi connect on Arduino 3.x (ESP_ERR_WIFI_STATE)
```

### Revert
- Discard the whole spike: `git checkout calibration-v2`.
- Keep the toolchain, drop just the probe: remove `-DCOEX_SPIKE` from `platformio.ini`
  (the probe code is fenced by `#ifdef COEX_SPIKE`).
- Find every spike edit: `rg SPIKE src/main.cpp platformio.ini sdkconfig.spike huge_app.csv`.
- New files to delete on full revert: `sdkconfig.spike`, `huge_app.csv`, `tests/version_bump.md`.

## 11. Did NOT do (per the spike's constraints)

- Did not touch `AnchorFirmware`, `proximity_engine`, or `impulse_app`.
- Did not push, open PRs, or merge into `calibration-v2`/master.
- Did not remove Option A "for real" or refactor calibration.
- Did not flash hardware until the user explicitly invited testing on the connected watch.
