#include "imu.h"

#include <math.h>

// ---- Register addresses ----
#define REG_WHO_AM_I  0x0F
#define REG_CTRL_REG1 0x20
#define REG_CTRL_REG2 0x21
#define REG_CTRL_REG3 0x22
#define REG_CTRL_REG4 0x23
#define REG_CTRL_REG5 0x24
#define REG_REFERENCE 0x26
#define REG_OUT_X_L   0x28
#define REG_CTRL_REG6 0x25
#define REG_INT1_CFG  0x30
#define REG_INT1_SRC  0x31
#define REG_INT1_THS  0x32
#define REG_INT1_DUR  0x33
#define REG_INT2_CFG  0x34
#define REG_INT2_SRC  0x35
#define REG_INT2_THS  0x36
#define REG_INT2_DUR  0x37

// INT_SRC bit positions (shared by INT1_SRC / INT2_SRC).
#define SRC_BIT_IA    0x40  // an interrupt was generated
#define SRC_BIT_ZL    0x10  // Z-low event  (+Z pointing down → watch face to sky)

// CTRL_REG3 routing bits.
#define CR3_I1_IA1    0x40  // route generator 1 (motion) to INT1 pin
#define CR3_I1_IA2    0x20  // route generator 2 (look)   to INT1 pin

// Motion-wake tuning. The interrupt generator runs the accelerometer through a
// high-pass filter (CTRL_REG2 below), so the threshold is measured against
// *dynamic* acceleration with gravity removed — i.e. it represents real motion
// intensity and behaves the same in every wrist orientation. Without the HPF
// the threshold competed with the ~1 g gravity vector, making sensitivity wildly
// orientation-dependent and hair-trigger in common resting positions.
// Motion only wakes/triggers the watch during ENFORCEMENT (it is ignored in
// DORMANT), so sensitivity here is about catching the user moving — e.g. walking
// away from a stayNear anchor between polls — not about idle battery. Keep it low
// enough that normal-pace movement registers; brief gait peaks must still pass,
// so the duration filter is short.
#define MOTION_THRESHOLD_MG   48    // dynamic accel (gravity removed) to trigger; 16 mg/LSB at ±2g
                                    //   (overpermissive tuning: low so the LOWERING wrist motion reliably
                                    //   produces a wake to re-arm look-to-wake — expect easy motion wakes)
#define MOTION_DURATION_MS    20    // motion must persist this long to count (rejects single-sample spikes)
#define IMU_ODR_HZ            50     // CTRL_REG1 ODR (1 INT1_DUR step = 1/ODR)

// Look-to-wake (IA2) tuning. Unlike IA1, IA2 sees the RAW signal (no high-pass),
// so the threshold compares against gravity + any dynamic acceleration. The
// watch face / LED ring is on -Z, so face-to-sky → -Z up → az ≈ -1g (a Z-low
// event). The DURATION is the load-bearing tunable here: because the raw signal
// includes motion, a short duration fires on the *jolt* of the wrist swing
// (waking the watch while the static orientation isn't actually face-up yet);
// requiring the Z-low condition to hold for ~250 ms rejects those transients so
// the interrupt fires on the steady "holding the watch up to read it" posture
// rather than the motion of getting there.
// Both are tunable; if a comfortable glance doesn't trigger, lower the threshold
// (less steep tilt required) and/or the duration. Watch the [DIAG] wake az=...
// serial line to see what your wrist actually produces.
// NOTE: these are deliberately OVERPERMISSIVE right now (testing/tuning pass). The
// goal is many false positives so the gesture fires on the gentlest glance — tighten
// later once the wake plumbing is confirmed working end-to-end.
#define LOOK_TILT_THRESHOLD_MG   48  // RAISE: az below just -0.048 g — barely any face-up tilt required.
                                     //    Fires on almost any upward wrist angle, held...
#define LOOK_DURATION_MS         40  // ...this long (2 ODR samples). Short hold so a quick glance counts; kept
                                     //    nonzero only so the level interrupt doesn't re-latch instantly when parked.
#define LOOK_CLEAR_THRESHOLD_MG 600  // LOWERED: az risen back above -0.6 g — i.e. anything that isn't held nearly
                                     //    flat face-up counts as "lowered", so the one-shot re-arms very eagerly.

// ---- SPI command bits ----
#define SPI_READ  0x80
#define SPI_MULTI 0x40

static SPISettings lis3dh_spi_settings(10000000, MSBFIRST, SPI_MODE3);

volatile bool data_ready = false;

void IRAM_ATTR lis3dh_isr() {
    
    data_ready = true;
}

// ---------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------

static void write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(lis3dh_spi_settings);
    digitalWrite(LIS3DH_CS, LOW);
    SPI.transfer(reg & 0x3F);
    SPI.transfer(val);
    digitalWrite(LIS3DH_CS, HIGH);
    SPI.endTransaction();
}

static uint8_t read_reg(uint8_t reg) {
    SPI.beginTransaction(lis3dh_spi_settings);
    digitalWrite(LIS3DH_CS, LOW);
    SPI.transfer(SPI_READ | (reg & 0x3F));
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(LIS3DH_CS, HIGH);
    SPI.endTransaction();
    return val;
}

static void read_burst(uint8_t reg, uint8_t *buf, uint8_t len) {
    SPI.beginTransaction(lis3dh_spi_settings);
    digitalWrite(LIS3DH_CS, LOW);
    SPI.transfer(SPI_READ | SPI_MULTI | (reg & 0x3F));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(LIS3DH_CS, HIGH);
    SPI.endTransaction();
}

// ---------------------------------------------------------------
// Init
// ---------------------------------------------------------------

bool lis3dh_init() {
    delay(5);

    uint8_t id = read_reg(REG_WHO_AM_I);
    if (id != 0x33) {
        Serial.printf("LIS3DH not found! WHO_AM_I = 0x%02X (expected 0x33)\n", id);
        return false;
    }
    Serial.println("LIS3DH found!");

    // CTRL_REG1 = 0x47: ODR=50Hz, normal mode, all axes enabled
    write_reg(REG_CTRL_REG1, 0x47);

    // CTRL_REG2 = 0x01: high-pass filter, normal mode (HPM=00), enabled for the
    // AOI1 interrupt generator (HPIS1=1). Removes the gravity/DC component so the
    // motion threshold reflects real movement intensity regardless of orientation.
    // FDS=0 leaves the data registers (read_accel) unfiltered.
    write_reg(REG_CTRL_REG2, 0x01);

    // CTRL_REG3: route IA1 (general motion) to the INT1 pin. With look-to-wake
    // enabled, also route IA2 (wrist-raise) onto the SAME pin — the two
    // generators are OR'd on INT1 and demuxed in software via the SRC registers.
#if LOOK_WAKEUP_ENABLED
    write_reg(REG_CTRL_REG3, CR3_I1_IA1 | CR3_I1_IA2);
#else
    write_reg(REG_CTRL_REG3, CR3_I1_IA1);
#endif

    // CTRL_REG4 = 0x88: BDU=1, ±2g full scale, high-res 12-bit, 4-wire SPI
    write_reg(REG_CTRL_REG4, 0x88);

    // CTRL_REG5: latch INT1 (LIR_INT1), and INT2 too (LIR_INT2) when look-to-wake
    // is enabled, so a wake survives until the matching SRC register is read.
#if LOOK_WAKEUP_ENABLED
    write_reg(REG_CTRL_REG5, 0x08 | 0x02);
#else
    write_reg(REG_CTRL_REG5, 0x08);
#endif

    // INT1_THS: threshold in 16mg steps at ±2g (dynamic accel, post-HPF)
    write_reg(REG_INT1_THS, (uint8_t)(MOTION_THRESHOLD_MG / 16));

    // INT1_DUR: motion must persist this many ODR samples before the interrupt
    // fires (1 step = 1/ODR). Filters brief jolts so only sustained motion wakes.
    write_reg(REG_INT1_DUR, (uint8_t)(MOTION_DURATION_MS * IMU_ODR_HZ / 1000));

    // INT1_CFG = 0x2A: high-event on X, Y, Z (OR combination)
    write_reg(REG_INT1_CFG, 0x2A);

#if LOOK_WAKEUP_ENABLED
    // ---- IA2: wrist-raise ("look at the watch") detector ----
    // Face/LED ring is on -Z, so face-to-sky → -Z up → az ≈ -1g → a Z-low event.
    // IA2 reads the RAW signal (HPIS2 left 0 in CTRL_REG2) so it sees gravity.
    // INT2_CFG = 0x90 → AOI=1 with ZLIE only: the interrupt asserts ONLY while
    // az < -THS, i.e. only when the watch is held face-up. This matters a lot: a
    // 6D/all-zones config asserts in nearly every orientation, which pins INT1
    // HIGH and makes light sleep return immediately (the watch then never sleeps).
    // With ZLIE-only, all the normal resting orientations keep INT1 LOW, so
    // DORMANT_SLEEP works and only a real raise wakes the watch. INT2_DUR forces
    // the condition to persist (orientation, not the motion spike of raising) —
    // without it the raw-signal interrupt chatters on the swing itself. The
    // parked-face-up edge is handled in firmware (the GPIO wake is not armed
    // while currently face-up; see enter_dormant_sleep).
    write_reg(REG_INT2_THS, (uint8_t)(LOOK_TILT_THRESHOLD_MG / 16));
    write_reg(REG_INT2_DUR, (uint8_t)(LOOK_DURATION_MS * IMU_ODR_HZ / 1000));
    write_reg(REG_INT2_CFG, 0x90);  // AOI=1 | ZLIE
#endif

    // Initialise the high-pass filter reference and clear any pending interrupts.
    read_reg(REG_REFERENCE);
    read_reg(REG_INT1_SRC);
#if LOOK_WAKEUP_ENABLED
    read_reg(REG_INT2_SRC);
#endif

    pinMode(LIS3DH_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(LIS3DH_INT1), lis3dh_isr, RISING);

    // Cover the remaining race: if a new sample arrived between the drain read
    // and attachInterrupt, INT1 is already HIGH and we'd wait forever.
    if (digitalRead(LIS3DH_INT1)) {
        data_ready = true;
    }

    return true;
}

// ---------------------------------------------------------------
// Read acceleration
// ---------------------------------------------------------------

static float to_g(int16_t raw) {
    // Data is left-justified; shift right 4 to right-justify (12-bit HR mode).
    // Sensitivity at ±2g = 1 mg/LSB.
    return (raw >> 4) * 0.001f;
}

Accel read_accel() {
    uint8_t buf[6];
    read_burst(REG_OUT_X_L, buf, 6);

    int16_t rx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);

    return { to_g(rx), to_g(ry), to_g(rz) };
}

// Clears the latched INT1 activity interrupt by reading INT1_SRC.
// Must be called after each interrupt fires or INT1 stays HIGH.
void lis3dh_clear_int1() {
    read_reg(REG_INT1_SRC);
}

// Reads (and thereby clears) both interrupt source registers, returning a
// LIS3DH_SRC_* bitmask of which generators fired. IA1 = general motion; IA2 =
// the wrist-raise gesture, recognised as the Z-low (face-to-sky) zone.
uint8_t lis3dh_read_clear_sources() {
    uint8_t out = 0;
    uint8_t s1 = read_reg(REG_INT1_SRC);
    if (s1 & SRC_BIT_IA) out |= LIS3DH_SRC_MOTION;
#if LOOK_WAKEUP_ENABLED
    uint8_t s2 = read_reg(REG_INT2_SRC);
    // Treat it as a look only when the recognised zone is Z-low (face up).
    if ((s2 & SRC_BIT_IA) && (s2 & SRC_BIT_ZL)) out |= LIS3DH_SRC_LOOK;
#endif
    return out;
}

// Toggle routing of the general-motion generator (IA1) to the INT1 pin while
// leaving the look generator (IA2) routing untouched. Lets DORMANT_SLEEP arm
// only the wrist-raise wake source and keep motion muted for battery (§8.4).
void lis3dh_set_motion_routing(bool enabled) {
    uint8_t v = read_reg(REG_CTRL_REG3) & ~CR3_I1_IA1;
    if (enabled) v |= CR3_I1_IA1;
    write_reg(REG_CTRL_REG3, v);
}

void lis3dh_set_int1_routing(bool motion_ia1, bool look_ia2) {
    uint8_t v = read_reg(REG_CTRL_REG3) & ~(CR3_I1_IA1 | CR3_I1_IA2);
    if (motion_ia1) v |= CR3_I1_IA1;
    if (look_ia2)   v |= CR3_I1_IA2;
    write_reg(REG_CTRL_REG3, v);
}

// True once the wrist is clearly LOWERED out of the look posture (az risen above
// -LOOK_CLEAR_THRESHOLD). The gap vs the IA2 raise trigger is hysteresis so a
// borderline hold doesn't flip-flop the one-shot raise detector. The raise
// itself is NOT firmware-confirmed (we trust the duration-filtered IA2 latch);
// this instantaneous read is only used to detect the steadier "lowered" state.
bool lis3dh_is_look_lowered() {
    Accel a = read_accel();
    return a.z > -(LOOK_CLEAR_THRESHOLD_MG / 1000.0f);
}
