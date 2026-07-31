# Impulse Watch Firmware

ESP32-C3 firmware for the Impulse watch — the wearable that holds the schedule, decides whether you're meeting a commitment, and escalates when you aren't.

Full specification: **[firmware_spec_v2.md](firmware_spec_v2.md)** (§5 is the watch). Product framing: **[impulse_overview.md](impulse_overview.md)**.

---

## What it is

Impulse is a habit-enforcement system built on **self-binding**: you decide in a calm moment what your day should look like, and the hardware holds you to it later, when a tireder version of you would rather renegotiate. Three parts — a watch, anchors placed around the home, and a phone app. This repo is the watch.

The watch is the **root of trust** for the whole system. It owns the schedule and the clock, and it is the thing that decides whether a commitment is being met. The app can propose changes; the watch decides when they take effect (see [§9 of the spec](firmware_spec_v2.md)). That placement is deliberate — an enforcement mechanism that lives on the device being enforced is one the user can always win against.

## What it does

- **Holds the schedule** in NVS and wakes itself at each commitment window boundary, sleeping in between.
- **Checks the criterion** during an active window — near an anchor, away from an anchor, on or off a named WiFi network, or away from the phone — and escalates through vibration and sound when it isn't met.
- **Knows when it's worn**, via a reflective IR opto-interrupter. Taking the watch off mid-commitment doesn't end enforcement; it hands enforcement to the anchors, which beep in the rooms that matter.
- **Locates itself** relative to anchors using the shared [proximity engine](../proximity_engine/README.md): it builds an RF scan vector, submits it to the anchor over GATT, and folds the returned score into a motion-conditioned HMM.
- **Talks to the phone** over BLE GATT (schedule pushes, status, time sync) and to anchors over WiFi (UDP commands, HTTP schedule distribution).
- **Reports status** on a 12-LED ring, and logs battery events for the power work.

## State machine (§5.1)

```
UNPAIRED  ──first BLE connection──►  DORMANT  ◄──►  DORMANT_SLEEP
                                        │
                              window boundary / RTC
                                        ▼
                                   ENFORCEMENT
```

- **UNPAIRED** — factory default. No IMU, no enforcement; waits for a first BLE connection.
- **DORMANT** — nominal resting state. Periodic BLE scans for anchor iBeacons (filtered on Major `0x4A0F`, so AirTags and other third-party beacons are ignored), periodic WiFi scans, processes app writes. Zero IMU interrupts.
- **DORMANT_SLEEP** — light sleep between dormant work.
- **ENFORCEMENT** — an event window is active. This is where the proximity engine, the IMU motion channel, and the poll/escalate loop live. Poll cadence is tiered: 60 s when the criterion isn't met, 180 s when it is, and 600 s when it's met *and* the wrist is provably still.

## Layout

| Path | What's in it |
|---|---|
| [src/main.cpp](src/main.cpp) | The state machine, schedule handling, GATT server, WiFi/UDP, enforcement loop. Large; navigate by the `// ====` section banners. |
| [src/watch_prox_transport.cpp](src/watch_prox_transport.cpp) | GATT client to the anchor's proximity service — vector submission, score read, session reuse. |
| [src/prox_platform.cpp](src/prox_platform.cpp) | Platform seams the proximity engine needs (clock, NVS). |
| [src/imu.cpp](src/imu.cpp) | LIS3DH over SPI: motion interrupt config and the per-poll accelerometer burst. |
| [src/led_status.cpp](src/led_status.cpp) | Status ring. Never affects enforcement logic. |
| [firmware_spec_v2.md](firmware_spec_v2.md) | The specification for watch **and** anchor, including the pin map, GATT contract, and wire formats. |
| [firmware_spec_v0.9_amendment.md](firmware_spec_v0.9_amendment.md) | Proximity-v2 delta: beacon schedule, vector trailer, phase map. |

The proximity algorithms are **not** in this repo. They live in [`proximity_engine`](../proximity_engine/README.md), symlinked in as a PlatformIO library and built with `-DPROXIMITY_ROLE_WATCH`. Nothing here should contain proximity logic; this side only supplies platform seams and calls the public API.

## Build & flash

```sh
~/.platformio/penv/bin/pio run              # build
~/.platformio/penv/bin/pio run -t upload    # flash
~/.platformio/penv/bin/pio device monitor   # serial (send-on-enter is on)
```

Builds clean at **79.5% flash / 53.2% RAM** — headroom is real but not generous, so watch the flash number when adding libraries.

The toolchain is **pinned to `espressif32@6.9.0`** (Arduino-ESP32 2.0.17 / ESP-IDF 4.4.7) and the pin is load-bearing. A bare `platform = espressif32` resolves to whatever is registered globally under that name, which a 2026-07 spike briefly hijacked with an Arduino 3.x / IDF 5.x fork — silently producing ~56 KB of extra flash and a false partition overflow. Don't unpin it casually.

> **Flashing gotcha.** Once the watch enters light sleep, USB CDC goes away and it cannot be flashed without a physical reset press. If you're working without hands on the hardware, flash a build that doesn't sleep, or make sure someone can reach the button.

## Status

Firmware v0.8 shipped; v0.9 (proximity v2) is in progress on `prox-v2-p1`, now merged to `master`. Phase 1 of the engine — motion channel, integrator, HMM — plus the watch-local parts of Phase 2 (observation window, stepped-power PDR) are implemented and validated on hardware. The v2 verdict is currently computed and logged **beside** the shipped v0.8 threshold decision rather than replacing it (`PROX_V2_AUTHORITATIVE = 0`); the flip is a deliberate separate step.

Two things to know before touching hardware:

- **The build tree is tracked in git.** A CMake `build/` directory is under version control and generates spurious diffs on every local build. It should be gitignored and untracked.
- **The LED ring shares GPIO 10 with the vibration motor** on the current hardware revision, where the motor is disabled by a PCB switch. The next revision moves the ring and restores the motor. The data pin is a single named constant for exactly this reason — see the spec's LED note before wiring anything.
