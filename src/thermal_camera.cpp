#include "thermal_camera.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <cmath>
#include <cstdint>

LOG_MODULE_REGISTER(ThermalCamera);

ThermalCamera::ThermalCamera(const i2c_dt_spec &i2c) : i2c_(i2c)
{
    k_mutex_init(&frameMutex_);
    k_sem_init(&frameReadySem_, 0, 1);
}

int ThermalCamera::init()
{
    sensor_init_timestamp_S = k_uptime_get() / 1000;
    restore_EPROM();
}

bool ThermalCamera::isWarm(void)
{
    int64_t elapsed_ms = (k_uptime_get() / 1000) - sensor_init_timestamp_S;
    return elapsed_ms >= MLX90640_WARMUP_S;
}

/**
 * @brief I2C adapter to hide the i2c object
 *
 * @param reg entry register
 * @param buf buffer to read data into
 * @param len number of bytes contained in the data
 * @return int
 */
int ThermalCamera::i2c_read(uint16_t reg, uint8_t *buf, size_t len)
{
    if (!i2c_is_ready_dt(&i2c_))
    {
        LOG_ERR("I2C device is not ready and cannot read");
        return -ENODEV;
    }

    uint8_t reg_addr[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};

    int ret = i2c_write_read_dt(&i2c_, reg_addr, sizeof(reg_addr), buf, len);
    if (ret < 0)
    {
        LOG_ERR("I2C read failed at reg 0x%x: %d", reg, ret);
        return ret;
    }
    return 0;
}
/**
 * @brief Restores the EPROM values based on the formulas set out in the datasheet
 *
 * @return int
 */
int ThermalCamera::restore_EPROM()
{
    // Read entire EEPROM into member cache: 0x2400-0x273F = 832 words
    int ret = i2c_read(0x2400, (uint8_t *)eeprom_cache_, sizeof(eeprom_cache_));
    if (ret < 0)
    {
        LOG_ERR("Failed to read EEPROM: %d", ret);
        return ret;
    }

    auto e = [this](uint16_t addr) -> uint16_t { return eeprom_cache_[addr - 0x2400]; };

    // ---- §11.1.1  VDD sensor parameters ----
    int8_t kvdd_ee = (int8_t)((e(0x2433) & 0xFF00) >> 8);
    calibData.kv_vdd = (double)kvdd_ee * (double)(1 << 5);

    uint8_t vdd25_ee = (uint8_t)(e(0x2433) & 0x00FF);
    calibData.vdd_25 = ((double)vdd25_ee - 256.0) * (double)(1 << 5) - (double)(1 << 13);

    // ---- §11.1.2  Ta sensor parameters ----
    int16_t kv_ptat_raw = (int16_t)((e(0x2432) & 0xFC00) >> 10);
    if (kv_ptat_raw > 31)
        kv_ptat_raw -= 64; // 6-bit sign extension
    calibData.kv_ptat = (double)kv_ptat_raw / (double)(1 << 12);

    int16_t kt_ptat_raw = (int16_t)(e(0x2432) & 0x03FF);
    if (kt_ptat_raw > 511)
        kt_ptat_raw -= 1024; // 10-bit sign extension
    calibData.kt_ptat = (double)kt_ptat_raw / (double)(1 << 3);

    // Alpha_PTAT: bits[15:12] of 0x2410, 4-bit unsigned, offset by +8 and scaled
    calibData.alpha_ptat = (double)((e(0x2410) & 0xF000) >> 12) / 4.0 + 8.0;

    // PTAT_25: EE[0x2431], 16-bit signed
    calibData.v_ptat_25 = (double)(int16_t)e(0x2431);

    // ---- §11.1.7  Gain ----
    calibData.gain = (double)(int16_t)e(0x2430);

    // ---- §11.1.8  KsTa ----
    int8_t ks_ta_ee = (int8_t)((e(0x243C) & 0xFF00) >> 8);
    calibData.ks_ta = (double)ks_ta_ee / (double)(1LL << 13);

    // ---- Scale values — must be set before per-pixel helpers run ----
    uint16_t scale_reg = e(0x2438);
    calibData.k_v_scale = (double)((scale_reg & 0x0F00) >> 8);
    calibData.k_ta_scale_1 = (double)(((scale_reg & 0x00F0) >> 4) + 8);
    calibData.k_ta_scale_2 = (double)(scale_reg & 0x000F);

    // ---- §11.1.17 ADC resolution control ----
    calibData.resolution = (e(0x2438) & 0x3000) >> 12;

    // ---- §11.1.16 TGC ----
    int8_t tgc_ee = (int8_t)(e(0x243C) & 0x00FF);
    calibData.tgc = (double)tgc_ee / (double)(1 << 5);

    // ---- §11.1.9  Corner temperatures ----
    uint16_t ct_reg = e(0x243F);
    int ct_step = (int)((ct_reg & 0x3000) >> 12) * 10;
    calibData.ct[0] = -40.0f;
    calibData.ct[1] = 0.0f;
    calibData.ct[2] = (float)(((ct_reg & 0x00F0) >> 4) * ct_step);
    calibData.ct[3] = calibData.ct[2] + (float)(((ct_reg & 0x0F00) >> 8) * ct_step);

    // ---- §11.1.10 KsTo ----
    int ks_to_scale = (int)(ct_reg & 0x000F) + 8;
    double ks_to_den = (double)(1LL << ks_to_scale);
    calibData.ks_to[0] = (float)((double)(int8_t)(e(0x243D) & 0xFF) / ks_to_den);
    calibData.ks_to[1] = (float)((double)(int8_t)((e(0x243D) >> 8) & 0xFF) / ks_to_den);
    calibData.ks_to[2] = (float)((double)(int8_t)(e(0x243E) & 0xFF) / ks_to_den);
    calibData.ks_to[3] = (float)((double)(int8_t)((e(0x243E) >> 8) & 0xFF) / ks_to_den);

    // ---- §11.1.12 CP sensitivity (alpha) ----
    int cp_alpha_scale = (int)((e(0x2420) & 0xF000) >> 12) + 27;
    double cp_alpha_den = (double)(1LL << cp_alpha_scale);

    int16_t cp_alpha_ee = (int16_t)(e(0x2439) & 0x03FF);
    if (cp_alpha_ee > 511)
        cp_alpha_ee -= 1024; // 10-bit sign extension

    int16_t cp_P1P0_ratio = (int16_t)((e(0x2439) & 0xFC00) >> 10);
    if (cp_P1P0_ratio > 31)
        cp_P1P0_ratio -= 64; // 6-bit sign extension

    calibData.cp_alpha[0] = (float)((double)cp_alpha_ee / cp_alpha_den);
    calibData.cp_alpha[1] = (float)((double)cp_alpha_ee * (1.0 + (double)cp_P1P0_ratio / 128.0) / cp_alpha_den);

    // ---- §11.1.13 CP offset ----
    int16_t cp_off0 = (int16_t)(e(0x243A) & 0x03FF);
    if (cp_off0 > 511)
        cp_off0 -= 1024; // 10-bit sign extension

    int16_t cp_off1_delta = (int16_t)((e(0x243A) & 0xFC00) >> 10);
    if (cp_off1_delta > 31)
        cp_off1_delta -= 64; // 6-bit sign extension

    calibData.cp_offset[0] = (float)cp_off0;
    calibData.cp_offset[1] = (float)(cp_off0 + cp_off1_delta);

    // ---- §11.1.14 CP Kv ---- (Compensation Pixel)
    int8_t kv_cp_ee = (int8_t)((e(0x243B) & 0xFF00) >> 8);
    calibData.kv_cp = (float)((double)kv_cp_ee / (double)(1LL << (int)calibData.k_v_scale));

    // ---- §11.1.15 CP Kta ----
    int8_t kta_cp_ee = (int8_t)(e(0x243B) & 0x00FF);
    calibData.kta_cp = (float)((double)kta_cp_ee / (double)(1LL << (int)calibData.k_ta_scale_1));

    // ---- Per-pixel restoration §11.1.3–§11.1.6 ----
    for (uint8_t row = 1; row <= FRAME_COLS; row++)
    {
        for (uint8_t col = 1; col <= FRAME_ROWS; col++)
        {
            int pix = (row - 1) * 32 + (col - 1);
            calibData.offset[pix] = (float)restore_Offset(row, col);
            calibData.alpha[pix] = (float)restore_Sensitivity(row, col);
            calibData.kta[pix] = (float)restore_kta(row, col);
            calibData.kv[pix] = (float)restore_Ta(row, col);
        }
    }

    return 0;
}

/**
 * @brief  Restores the individual pixels offset calculation defined in §11.1.3 in the datasheet
 *
 * @param row Row of the pixel where the offset is being calculated
 * @param col Column of the pixel where the offset is being calculated
 * @return double The offset value
 */
double ThermalCamera::restore_Offset(uint8_t row, uint8_t col)
{
    auto e = [this](uint16_t addr) -> uint16_t { return eeprom_cache_[addr - 0x2400]; };

    int occ_rem_scale = e(0x2410) & 0x000F;
    int occ_col_scale = (e(0x2410) & 0x00F0) >> 4;
    int occ_row_scale = (e(0x2410) & 0x0F00) >> 8;

    int16_t offset_ref = (int16_t)e(0x2411);

    // OCC row: packed 4 nibbles per word, 6 words starting at 0x2412
    int row_idx = row - 1;
    int occ_row_raw = (e(0x2412 + row_idx / 4) >> ((row_idx % 4) * 4)) & 0xF;
    int8_t occ_row = (occ_row_raw > 7) ? (int8_t)(occ_row_raw - 16) : (int8_t)occ_row_raw;

    // OCC col: packed 4 nibbles per word, 8 words starting at 0x2418
    int col_idx = col - 1;
    int occ_col_raw = (e(0x2418 + col_idx / 4) >> ((col_idx % 4) * 4)) & 0xF;
    int8_t occ_col = (occ_col_raw > 7) ? (int8_t)(occ_col_raw - 16) : (int8_t)occ_col_raw;

    // Per-pixel: bits[15:10], 6-bit signed
    uint16_t pix_word = e(0x2440 + row_idx * 32 + col_idx);
    int pix_off_raw = (int)(pix_word >> 10);
    if (pix_off_raw > 31)
        pix_off_raw -= 64;

    return (double)offset_ref + (double)occ_row * (double)(1LL << occ_row_scale) +
           (double)occ_col * (double)(1LL << occ_col_scale) + (double)pix_off_raw * (double)(1LL << occ_rem_scale);
}

//
/**
 * @brief Restores the individual pixels sensitivity offset calculation defined in §11.1.4  Per-pixel sensitivity
 * (alpha) in the datasheet
 *
 * @param row Row of the pixel where the offset is being calculated
 * @param col Column of the pixel where the offset is being calculated
 * @return double
 */
double ThermalCamera::restore_Sensitivity(uint8_t row, uint8_t col)
{
    auto e = [this](uint16_t addr) -> uint16_t { return eeprom_cache_[addr - 0x2400]; };

    uint16_t scale_reg = e(0x2420);
    int acc_rem_scale = scale_reg & 0x000F;
    int acc_col_scale = (scale_reg & 0x00F0) >> 4;
    int acc_row_scale = (scale_reg & 0x0F00) >> 8;
    int alpha_scale = (int)((scale_reg & 0xF000) >> 12) + 30;

    uint16_t alpha_ref = e(0x2421);

    // ACC row: packed 4 nibbles per word, 6 words starting at 0x2422
    int row_idx = row - 1;
    int acc_row_raw = (e(0x2422 + row_idx / 4) >> ((row_idx % 4) * 4)) & 0xF;
    int8_t acc_row = (acc_row_raw > 7) ? (int8_t)(acc_row_raw - 16) : (int8_t)acc_row_raw;

    // ACC col: packed 4 nibbles per word, 8 words starting at 0x2428
    int col_idx = col - 1;
    int acc_col_raw = (e(0x2428 + col_idx / 4) >> ((col_idx % 4) * 4)) & 0xF;
    int8_t acc_col = (acc_col_raw > 7) ? (int8_t)(acc_col_raw - 16) : (int8_t)acc_col_raw;

    // Per-pixel: bits[9:4], 6-bit signed
    uint16_t pix_word = e(0x2440 + row_idx * 32 + col_idx);
    int pix_alpha_raw = (int)((pix_word >> 4) & 0x3F);
    if (pix_alpha_raw > 31)
        pix_alpha_raw -= 64;

    double alpha_raw = (double)alpha_ref + (double)acc_row * (double)(1LL << acc_row_scale) +
                       (double)acc_col * (double)(1LL << acc_col_scale) +
                       (double)pix_alpha_raw * (double)(1LL << acc_rem_scale);

    return alpha_raw / (double)(1LL << alpha_scale);
}

/**
 * @brief Per pixel ambient temperature adjustment calculation as defined in §11.1.6 (Per-pixel Kta)
 *
 * @param row Row of the pixel where the kta is being calculated
 * @param col Column of the pixel where the kta is being calculated
 * @return double
 */
double ThermalCamera::restore_kta(uint8_t row, uint8_t col)
{
    auto e = [this](uint16_t addr) -> uint16_t { return eeprom_cache_[addr - 0x2400]; };

    // Row/col parity selects one of four 8-bit signed Kta_RC values
    bool row_odd = (row % 2 == 1);
    bool col_odd = (col % 2 == 1);

    int8_t kta_rc_ee;
    if (row_odd && col_odd)
        kta_rc_ee = (int8_t)((e(0x2436) >> 8) & 0xFF); // RoCo
    else if (!row_odd && col_odd)
        kta_rc_ee = (int8_t)(e(0x2436) & 0xFF); // ReCo
    else if (row_odd && !col_odd)
        kta_rc_ee = (int8_t)((e(0x2437) >> 8) & 0xFF); // RoCe
    else
        kta_rc_ee = (int8_t)(e(0x2437) & 0xFF); // ReCe

    // Per-pixel: bits[3:1], 3-bit signed
    int pix_idx = (row - 1) * 32 + (col - 1);
    int kta_pix_raw = (int)((e(0x2440 + pix_idx) >> 1) & 0x7);
    if (kta_pix_raw > 3)
        kta_pix_raw -= 8;

    return ((double)kta_rc_ee + (double)kta_pix_raw * (double)(1LL << (int)calibData.k_ta_scale_2)) /
           (double)(1LL << (int)calibData.k_ta_scale_1);
}

// §11.1.5  Per-pixel Kv
/**
 * @brief
 *
 * @param row Row of the pixel where the kv is being calculated
 * @param col Column of the pixel where the kv is being calculated
 * @return double
 */
double ThermalCamera::restore_Ta(uint8_t row, uint8_t col)
{
    auto e = [this](uint16_t addr) -> uint16_t { return eeprom_cache_[addr - 0x2400]; };

    // Kv has no per-pixel term; only row/col parity selects a 4-bit signed value
    bool row_odd = (row % 2 == 1);
    bool col_odd = (col % 2 == 1);

    uint16_t kv_reg = e(0x2434);
    int kv_raw;
    if (row_odd && col_odd)
        kv_raw = (kv_reg >> 12) & 0xF; // RoCo
    else if (!row_odd && col_odd)
        kv_raw = (kv_reg >> 8) & 0xF; // ReCo
    else if (row_odd && !col_odd)
        kv_raw = (kv_reg >> 4) & 0xF; // RoCe
    else
        kv_raw = kv_reg & 0xF; // ReCe

    int8_t kv_ee = (kv_raw > 7) ? (int8_t)(kv_raw - 16) : (int8_t)kv_raw;

    return (double)kv_ee / (double)(1LL << (int)calibData.k_v_scale);
}

int ThermalCamera::calculate_frame()
{
    frame_constants_valid_ = false;

    // §11.2.2.2  Supply voltage
    uint16_t v_s_raw;
    i2c_read(0x072A, (uint8_t *)&v_s_raw, sizeof(v_s_raw));
    double dv = ((double)(int16_t)v_s_raw - calibData.vdd_25) / calibData.kv_vdd;
    vdd = dv + 3.3;

    // §11.2.2.3  Ambient temperature
    uint16_t v_ptat_raw;
    i2c_read(0x0720, (uint8_t *)&v_ptat_raw, sizeof(v_ptat_raw));
    int16_t v_ptat = (int16_t)v_ptat_raw;

    uint16_t v_be_raw;
    i2c_read(0x0700, (uint8_t *)&v_be_raw, sizeof(v_be_raw));
    int16_t v_be = (int16_t)v_be_raw;

    double v_ptat_art = ldexp((double)v_ptat / ((double)v_ptat * calibData.alpha_ptat + (double)v_be), 18);
    t_a = (v_ptat_art / (1.0 + calibData.kv_ptat * dv) - calibData.v_ptat_25) / calibData.kt_ptat + 25.0;

    // §11.2.2.4  Gain
    uint16_t r_gain_raw;
    i2c_read(0x070A, (uint8_t *)&r_gain_raw, sizeof(r_gain_raw));
    k_gain = calibData.gain / (double)(int16_t)r_gain_raw;

    // §11.2.2.6.1–6.2  CP pixel gain + offset/Ta/VDD compensation
    uint16_t cp_sp0_raw, cp_sp1_raw;
    i2c_read(0x0708, (uint8_t *)&cp_sp0_raw, sizeof(cp_sp0_raw));
    i2c_read(0x0728, (uint8_t *)&cp_sp1_raw, sizeof(cp_sp1_raw));

    double cp_ta_factor = 1.0 + (double)calibData.kta_cp * (t_a - 25.0);
    double cp_vdd_factor = 1.0 + (double)calibData.kv_cp * (vdd - 3.3);
    cp_os_[0] = (double)(int16_t)cp_sp0_raw * k_gain - (double)calibData.cp_offset[0] * cp_ta_factor * cp_vdd_factor;
    cp_os_[1] = (double)(int16_t)cp_sp1_raw * k_gain - (double)calibData.cp_offset[1] * cp_ta_factor * cp_vdd_factor;

    // Read all pixel RAM 0x0400–0x06FF (768 words)
    int ret = i2c_read(0x0400, (uint8_t *)ram_cache_pix_, sizeof(ram_cache_pix_));
    if (ret < 0)
    {
        LOG_ERR("Failed to read pixel RAM: %d", ret);
        return ret;
    }

    frame_constants_valid_ = true;

    // §11.2.2.5  Per-pixel temperature calculation
    for (uint8_t row = 1; row <= FRAME_COLS; row++)
        for (uint8_t col = 1; col <= FRAME_ROWS; col++)
            irSensorBuff.pixels[row - 1][col - 1] = (float)_calculate_pixel_temp(row, col);

    frame_constants_valid_ = false;
    return 0;
}

double ThermalCamera::_calculate_pixel_temp(uint8_t row, uint8_t col)
{
    if (!frame_constants_valid_)
    {
        LOG_WRN("_calculate_pixle_temp called without valid frame constants");
        return (double)NAN;
    }

    int pix = (row - 1) * 32 + (col - 1);

    // §11.2.2.5.1  Gain compensation
    double pix_gain = (double)(int16_t)ram_cache_pix_[pix] * k_gain;

    // §11.2.2.5.3  Offset, Ta, VDD compensation (Ta0 = 25°C, VddV0 = 3.3V)
    double pix_os = pix_gain - (double)calibData.offset[pix] * (1.0 + (double)calibData.kta[pix] * (t_a - 25.0)) *
                                   (1.0 + (double)calibData.kv[pix] * (vdd - 3.3));

    // §11.2.2.7  TGC gradient compensation (chess pattern, emissivity = 1)
    int pattern = ((row - 1) % 2) ^ ((col - 1) % 2);
    double v_ir_comp = pix_os - calibData.tgc * ((1 - pattern) * cp_os_[0] + pattern * cp_os_[1]);

    // §11.2.2.8  Normalize to sensitivity
    double alpha_cp = (1 - pattern) * (double)calibData.cp_alpha[0] + pattern * (double)calibData.cp_alpha[1];
    double alpha_comp =
        ((double)calibData.alpha[pix] - calibData.tgc * alpha_cp) * (1.0 + calibData.ks_ta * (t_a - 25.0));

    // §11.2.2.9  Object temperature — basic range (Tr ≈ Ta - 8°C, emissivity = 1)
    // ks_to[1] is the range-2 sensitivity slope coefficient (0°C to CT3)
    double t_r = t_a - 8.0;
    double t_aK4 = pow(t_a + 273.15, 4.0);
    double t_rK4 = pow(t_r + 273.15, 4.0);
    double t_ar = t_rK4 - (t_rK4 - t_aK4); // epsilon = 1 simplifies to t_aK4

    double s_x = (double)calibData.ks_to[1] * pow(pow(alpha_comp, 3.0) * v_ir_comp + pow(alpha_comp, 4.0) * t_ar, 0.25);

    return pow(v_ir_comp / (alpha_comp * (1.0 - (double)calibData.ks_to[1] * 273.15) + s_x) + t_ar, 0.25) - 273.15;
}

int ThermalCamera::updateStatusRegister()
{

    uint8_t buff;

    ThermalCamera::i2c_read(0x8000, &buff, 2);

    ThermalCamera::sReg.newDataAvailable = (buff & 0x0008);
    ThermalCamera::sReg.nextSubpageReady = (buff & 0x0001);
    return 0;
}

bool ThermalCamera::newData()
{
    updateStatusRegister();
    return sReg.newDataAvailable;
}
