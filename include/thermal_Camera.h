#pragma once

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>


#define MLX_NODE DT_NODELABEL(mlx90640)

static constexpr uint8_t FRAME_COLS = 24;
static constexpr uint8_t FRAME_ROWS = 32;

struct ThermalFrame
{
    float pixels[FRAME_COLS][FRAME_ROWS];
    bool warmedData = false;
    uint32_t frameId = 0;
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
};
