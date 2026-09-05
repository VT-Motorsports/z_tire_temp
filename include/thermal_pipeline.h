#pragma once

#include <cstdint>
#include <zephyr/kernel.h>
#include <errno.h>
#include <span>

#include "can.h"
#include "thermal_camera.h"
#include "zephyr/kernel/thread.h"

#define PIPE_THREAD_STACK_SIZE 32768
// %.4g float: up to 10 characters (e.g. -3.403e+38), plus a comma.
// Prefix: 10 frame-ID digits, comma, 2 row digits; final byte is the NUL.
static constexpr size_t PRINT_FRAMES_ROW_BUFFER_SIZE = 14 + FRAME_COLS * 11;

#define PRINT_FRAMES_STACK_SIZE ROUND_UP(sizeof(ThermalFrame) + PRINT_FRAMES_ROW_BUFFER_SIZE + 2048, 1024)

#define CAMERA_PROCESSING_SEGMENTS 7

// TODO: Implement the correct CAN codes and create DBC
enum CAN_MSG_CODES
{
    AVERAGE_PIXEL_MSG = 0x300
};

class ThermalPipeline
{
  public:
    explicit ThermalPipeline(ThermalCamera &camera, CanBus &can);

    int start();
    void close();

    bool printData = false;

  private:
    static constexpr int PROCESSING_THREAD_PRIO = K_LOWEST_THREAD_PRIO + 3;
    static constexpr int PRINT_FRAMES_PRIO = K_LOWEST_THREAD_PRIO + 1;
    static constexpr size_t PRINT_QUEUE_LEN = 5;

    bool pushSummaryToCan();

    static void threadEntry(void *p1, void *p2, void *p3);
    void processingLoop();

    // Processing functions
    float getAveragePixel(ThermalFrame &frame);
    int segementCameraData(ThermalFrame &frame, float (&buf)[CAMERA_PROCESSING_SEGMENTS], uint8_t seg_height);
    static uint16_t encodeTemp(const float &temp);

    void printSimple(ThermalFrame &Frame);

    // PrintFrames Thread
    struct k_thread printFramesThread;
    k_tid_t printFramesTID = nullptr;

    // PrintFrames Queue
    struct k_msgq printFramesQueue;
    ThermalFrame msqQBuff[PRINT_QUEUE_LEN];

    // Process Thermal Frames data Thread
    struct k_thread processingThread;
    k_tid_t processingTID = nullptr;

    static void printFramesEntry(void *instance, void *, void *);
    int printFramesThreadWrk();

    CanBus &can_;
    ThermalCamera &camera_;

    uint32_t lastProcessedFrameId_ = 0;
    bool running_ = false;
    ThermalFrame *workingFrame_{};
};
