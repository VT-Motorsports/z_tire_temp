#pragma once

#include <zephyr/kernel.h>

#include "thermal_camera.h"

class ThermalPipline
{
  public:
    explicit ThermalPipline(ThermalCamera &camera);

    int start();
    void stop();
    bool isRunning();


  private:
    static constexpr int THREAD_PRIORITY = 5;
    static constexpr size_t THREAD_STACK_SIZE = 2048;

    static void threadEntry(void *p1, void *p2, void *p3);
    void processingLoop();
    void processFrame(const ThermalFrame &frame);

    ThermalCamera &camera_;
    struct k_thread workerThread_;
    k_tid_t workerTid_ = nullptr;
    uint32_t lastProcessedFrameId_ = 0; 
    bool running_ = false;
    ThermalFrame workingFrame_{};
};
