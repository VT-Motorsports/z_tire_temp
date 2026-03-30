#pragma once

#include "thermal_camera.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define MLX_NODE DT_NODELABEL(mlx90640)

static constexpr uint8_t FRAME_COLS = 24;
static constexpr uint8_t FRAME_ROWS = 32;

static constexpr uint16_t TOTAL_PIXELS = FRAME_ROWS * FRAME_COLS;

struct ThermalFrame
{
    float pixels[FRAME_COLS][FRAME_ROWS];
    bool warmedData = false;
    uint32_t frameId = 0;
};

struct statusRegister
{
    bool newDataAvailable;
    int nextSubpageReady;
};

struct calibrationData
{
    // ===== GLOBAL =====
    double gain;
    double vdd_25;
    double kv_vdd;

    double ta_25;
    double kv_ptat;
    double kt_ptat;
    double ks_ta;
    double k_v_scale;
    double k_ta_scale_1;

    double tgc;
    int resolution;

    // ===== PER PIXEL =====
    float offset[TOTAL_PIXELS];
    float alpha[TOTAL_PIXELS];
    float kta[TOTAL_PIXELS];
    float kv[TOTAL_PIXELS];

    // ===== CP (compensation pixel) =====
    float cp_offset[2];
    float cp_alpha[2];

    // ===== OPTIONAL (advanced) =====
    float ks_to[4];
    float ct[4];

    // ===== DERIVED / SCALE =====
    double alpha_ptat;      // §11.1.2.3
    double k_ta_scale_2;    // §11.1.6 – fine scale for per-pixel Kta
    float  kv_cp;           // §11.1.14
    float  kta_cp;          // §11.1.15
};

class ThermalCamera
{
  public:
    // Keep the bus dependency injected so the camera stays testable.
    explicit ThermalCamera(const i2c_dt_spec &i2c);

    int init();
    int captureFrame();
    bool waitForFrame(k_timeout_t timeout = K_FOREVER);
    bool getLatestFrame(ThermalFrame &outFrame, uint32_t &outFrameId);
    bool isWarm() const;

  private:
    static constexpr uint16_t MLX_STATUS_REG_ADDR = 0x8000;
    static constexpr uint16_t MLX_MEASURE_ENTRY_REG_ADDR = 0x0400;

    statusRegister sReg;
    calibrationData calibData;

    int getEPROMData();
    double restore_Ta(uint8_t row, uint8_t col);
    double restore_Offset(uint8_t row, uint8_t col);
    double restore_Sensitivity(uint8_t row,uint8_t col);
    double restore_kta(uint8_t row,uint8_t col);
    

    int updateStatusRegister();

    int i2c_read(uint16_t reg, uint8_t *buf, size_t len);
    int i2c_write(uint16_t reg, const uint8_t *buf, size_t len);
    int readNewData(uint32_t frameID, ThermalFrame &frame);
    void publishFrame();

    i2c_dt_spec i2c_;

    uint8_t mRecentSubpage();
    bool newData();

    ThermalFrame sensorBuf1_;
    ThermalFrame sensorBuf2_;

    ThermalFrame publishedFrame_;

    struct k_mutex frameMutex_;
    struct k_sem frameReadySem_;
    uint32_t nextFrameId_ = 1;
    bool initialized_ = false;

    uint16_t eeprom_cache_[832];   // raw EEPROM words 0x2400-0x273F, populated by getEPROMData()
};
