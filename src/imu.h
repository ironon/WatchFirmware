#pragma once

#include <Arduino.h>
#include <SPI.h>

// ---- Pin mapping ----
#define LIS3DH_MOSI  7
#define LIS3DH_SCK   6
#define LIS3DH_MISO  2
#define LIS3DH_CS    8
#define LIS3DH_INT1  4

struct Accel {
    float x, y, z;  // in g
};

// Volatile flag set by the motion-interrupt ISR.
// Set when acceleration on any axis exceeds MOTION_THRESHOLD_MG.
extern volatile bool data_ready;

// Call once from setup() after SPI.begin().
// Returns true on success, false if the sensor is not detected.
bool lis3dh_init();

// Read the latest X/Y/Z acceleration values (in g).
Accel read_accel();

// Clear the latched INT1 activity interrupt.
// Must be called after each data_ready event to allow the next one to fire.
void lis3dh_clear_int1();
