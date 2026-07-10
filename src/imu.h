#pragma once

#include <Arduino.h>
#include <SPI.h>

// ---- Pin mapping ----
#define LIS3DH_MOSI  7 // is 7 on old boards, 20 on new
#define LIS3DH_SCK   6
#define LIS3DH_MISO  2
#define LIS3DH_CS    8
#define LIS3DH_INT1  4

// ---- Look-to-wake (wrist-raise) feature ----
// When enabled, the LIS3DH's second interrupt generator (IA2) detects the
// "looking at the watch" posture and wakes the watch from DORMANT_SLEEP through
// the shared INT1 line (the two generators are OR'd on INT1 and demuxed in
// software). The watch face and the LED ring sit on the -Z axis, so face-to-sky
// means -Z points up and the +Z accelerometer reads ≈ -1g — a "Z-low" event.
// Set to 0 to compile the feature out entirely (reverts to timer-only DORMANT
// sleep with motion muted, per §8.4).
#define LOOK_WAKEUP_ENABLED 1

// Interrupt-source bitmask returned by lis3dh_read_clear_sources().
#define LIS3DH_SRC_MOTION 0x01  // generator IA1 fired (general high-pass motion)
#define LIS3DH_SRC_LOOK   0x02  // generator IA2 fired (wrist-raise / look gesture)

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

// Clear the latched INT1 activity interrupt.
// Must be called after each data_ready event to allow the next one to fire.
void lis3dh_clear_int1();

// Read and clear BOTH interrupt source registers, returning a LIS3DH_SRC_*
// bitmask of which generators fired. Use this (instead of lis3dh_clear_int1)
// on the wake path where general motion (IA1) and the look gesture (IA2) must
// be distinguished.
uint8_t lis3dh_read_clear_sources();

// Enable/disable routing of the general-motion generator (IA1) to the INT1 pin.
// The look generator (IA2) routing is left intact. Used to mute motion as a
// wake source during DORMANT_SLEEP while keeping look-to-wake armed (§8.4).
void lis3dh_set_motion_routing(bool enabled);

// Set exactly which generators drive the INT1 pin: IA1 (general motion) and/or
// IA2 (the wrist-raise / Z-low look). Lets the sleep path choose to wake on a
// RAISE (look only) while armed, or on the LOWERING motion (motion only) after a
// raise has been shown — without the face-up level interrupt spinning the CPU.
void lis3dh_set_int1_routing(bool motion_ia1, bool look_ia2);

// True once the wrist has been LOWERED clearly out of the look posture (az risen
// above -LOOK_CLEAR_THRESHOLD). Hysteresis vs the IA2 raise trigger so a
// borderline hold doesn't flip-flop. Used to re-arm the one-shot raise detector.
bool lis3dh_is_look_lowered();
