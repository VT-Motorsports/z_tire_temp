#pragma once

#include <cstdint>
#include <zephyr/kernel.h>

#include "can.h"
#include "thermal_camera.h"

#define PIPE_THREAD_STACK_SIZE  32768


// TODO:Talk with pujan about how this is supposed to work
enum CAN_MSG_CODES
{
  AVERAGE_PIXEL_MSG = 0x300
};

class ThermalPipline
{
  public:
    explicit ThermalPipline(ThermalCamera &camera, CanBus &can);

    int start();
    void close();

    bool printData = false;

  private:
    static constexpr int THREAD_PRIORITY = 5;
    static constexpr size_t THREAD_STACK_SIZE = 16384;

    bool pushSummaryToCan();

    static void threadEntry(void *p1, void *p2, void *p3);
    void processingLoop();

    // Processing functions
    float getAveragePixel(ThermalFrame &frame);
    static uint16_t encodeTemp(const float &temp);


    void printSimple(ThermalFrame &Frame);

    CanBus &can_;
    ThermalCamera &camera_;
    struct k_thread workerThread_;
    k_tid_t workerTid_ = nullptr;
    uint32_t lastProcessedFrameId_ = 0;
    bool running_ = false;
    ThermalFrame *workingFrame_{};
};
