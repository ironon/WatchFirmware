#include "imu.h"

// ---- Register addresses ----
#define REG_WHO_AM_I  0x0F
#define REG_CTRL_REG1 0x20
#define REG_CTRL_REG3 0x22
#define REG_CTRL_REG4 0x23
#define REG_CTRL_REG5 0x24
#define REG_OUT_X_L   0x28
#define REG_INT1_CFG  0x30
#define REG_INT1_SRC  0x31
#define REG_INT1_THS  0x32
#define REG_INT1_DUR  0x33

// Activity threshold in mg (16mg per LSB at ±2g).
// Raise to reduce sensitivity; lower to increase it.
#define MOTION_THRESHOLD_MG  1056   // 256mg ≈ light wrist movement

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

    // CTRL_REG3 = 0x40: route IA1 (activity interrupt) to INT1 pin
    write_reg(REG_CTRL_REG3, 0x40);

    // CTRL_REG4 = 0x88: BDU=1, ±2g full scale, high-res 12-bit, 4-wire SPI
    write_reg(REG_CTRL_REG4, 0x88);

    // CTRL_REG5 = 0x08: latch INT1 — stays HIGH until INT1_SRC is read
    write_reg(REG_CTRL_REG5, 0x08);

    // INT1_THS: threshold in 16mg steps at ±2g
    write_reg(REG_INT1_THS, (uint8_t)(MOTION_THRESHOLD_MG / 16));

    // INT1_DUR: minimum duration before interrupt fires (1 step = 1/ODR = 20ms at 50Hz)
    // 2 steps = 40ms — filters out single-sample spikes
    write_reg(REG_INT1_DUR, 0x02);

    // INT1_CFG = 0x2A: high-event on X, Y, Z (OR combination)
    write_reg(REG_INT1_CFG, 0x2A);

    // Clear any pending interrupt from init by reading INT1_SRC
    read_reg(REG_INT1_SRC);

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
