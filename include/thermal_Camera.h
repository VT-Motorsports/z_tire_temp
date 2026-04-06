#pragma once

#include "thermal_camera.h"
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define MLX_NODE DT_NODELABEL(mlx90640)

static constexpr uint8_t FRAME_COLS = 24;
static constexpr uint8_t FRAME_ROWS = 32;
static constexpr uint16_t MLX90640_WARMUP_S = (4 * 60);

static constexpr uint16_t TOTAL_PIXELS = FRAME_ROWS * FRAME_COLS;


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
    explicit ThermalCamera();

    int init();
    bool waitForFrame(k_timeout_t timeout = K_FOREVER);
    bool getLatestFrame(ThermalFrame &outFrame, uint32_t &outFrameId);

  private:

    int calculate_frame();

    void publishFrame();


    uint8_t mRecentSubpage();
    bool newData();

    // frame made accessible to the thermal pipline class/outsiders
    ThermalFrame publishedFrame_;

    struct k_mutex frameMutex_;
    struct k_sem frameReadySem_;
    uint32_t nextFrameId_ = 1;

    bool isWarm(void);
    uint16_t sensor_init_timestamp_S;

};
