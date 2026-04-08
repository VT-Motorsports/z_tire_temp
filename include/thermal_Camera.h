#pragma once

#include "thermal_camera.h"
#include <cstddef>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include "MLX/MLX90640_API.h"
#include "zephyr/kernel/thread.h"

#define MLX_NODE DT_NODELABEL(mlx90640)
#define CAMERA_THREAD_STACK_SIZE 2048

static constexpr uint8_t MLX906040_I2C_ADRR = 0x32;
static constexpr uint8_t FRAME_COLS = 24;
static constexpr uint8_t FRAME_ROWS = 32;
static constexpr uint16_t MLX90640_WARMUP_S = (4 * 60);
static constexpr uint16_t TOTAL_PIXELS = FRAME_ROWS * FRAME_COLS;
static constexpr uint16_t TOTAL_FRAME_BUFF_AMT = 834;

struct ThermalFrame
{
    float pixels[TOTAL_PIXELS];
    bool warmedData = false;
    uint32_t frameId = 0;
    uint8_t subPage = 0;
};

class ThermalCamera
{
  public:
    explicit ThermalCamera();

    int init();
    int close();
    bool isWarm();
    ThermalFrame *getUpdatedFrame();
    static void logFrame(ThermalFrame &frame);

  private:
    // --- Thread configuration ---
    static constexpr int CAPTURE_THREAD_PRIORITY = 4;
    static constexpr size_t CAPTURE_THREAD_STACK_SIZE = CAMERA_THREAD_STACK_SIZE;

    // --- Sensor state ---
    paramsMLX90640 params;
    uint16_t sensor_init_timestamp_S;

    // --- Frame buffers ---
    ThermalFrame thermFrameBuff1;
    ThermalFrame thermFrameBuff2;
    ThermalFrame *externFramePtr = &thermFrameBuff1;   // Frame that is accessible with getUpdated frame
    ThermalFrame *internalFramePtr = &thermFrameBuff2; // Frame that updateFrame updates

    // --- Synchronization ---
    struct k_mutex frameMutex_;
    struct k_sem frameReadySem_;
    uint32_t nextFrameId_ = 1;

    // --- Threading ---
    k_thread upFrameThread;
    k_tid_t upFrameThreadPtr = nullptr;
    bool running_ = false;

    // --- Internal methods ---
    int updateFrame();
    static void updateFrameThreadEntry(void *p1, void *p2, void *p3);
    int getFrame(ThermalFrame &outFrame);
};
