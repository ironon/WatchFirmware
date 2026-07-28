#pragma once

#include <Arduino.h>
#include <SPI.h>

// ---- Pin mapping ----
#define LIS3DH_MOSI  7 // is 7 on old boards, 20 on new
#define LIS3DH_SCK   6
#define LIS3DH_MISO  2
#define LIS3DH_CS    8
#define LIS3DH_INT1  4

// Motion (IA1) is a wake source only during ENFORCEMENT; DORMANT_SLEEP has no
// motion interrupt at all (timer + BLE wakes only, §8.4). The former look-to-wake
// (wrist-raise, IA2) feature has been removed — the dormant watch keeps the
// analog clock lit on the ring and simply redraws it each minute instead.

struct Accel {
    float x, y, z;  // in g
};

// Volatile flag set by the motion-interrupt ISR.
// Set when acceleration on any axis exceeds MOTION_THRESHOLD_MG.
extern volatile bool data_ready;

// Call once from setup() after SPI.begin().
// Returns true on success, false if the sensor is not detected.
bool lis3dh_init();

// ISR for the INT1 pin — sets data_ready. Exposed so main.cpp can
// re-attach it after waking from light sleep.
void IRAM_ATTR lis3dh_isr();

// Read the latest X/Y/Z acceleration values (in g).
Accel read_accel();

// ---- Proximity engine v2.1 motion channel (§5.4.5 obligation 3) -------------
// One short accelerometer burst per proximity query, sampled *while the radio
// scans* so it costs no extra awake time, then handed to the engine's motion
// classifier via prox_ingest_imu_burst().
//
// Interleaved use (inside the pre-query scan's wait loop):
//     imu_burst_begin();
//     while (scanning) { imu_burst_service(); delay(10); }
//     imu_burst_submit();
//
// Blocking use (no scan to hide behind — e.g. deciding the poll tier just
// before enforcement light sleep):
//     imu_burst_blocking();
void imu_burst_begin();
void imu_burst_service();   // samples at most one triple, when one is due
void imu_burst_submit();    // hands whatever was collected to the engine
void imu_burst_blocking();  // begin + service + submit, ~IMU_BURST_SAMPLES/IMU_BURST_HZ ms

// Clear the latched INT1 activity interrupt.
// Must be called after each data_ready event to allow the next one to fire.
void lis3dh_clear_int1();
