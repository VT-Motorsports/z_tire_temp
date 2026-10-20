#pragma once

#include <cstdint>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <errno.h>
#include <span>
#include <vector>
#include <utility>


#include "can.h"
#include "thermal_Camera.h"
#include "zephyr/kernel/thread.h"

#define PIPE_THREAD_STACK_SIZE 32768
// %.4g float: up to 10 characters (e.g. -3.403e+38), plus a comma.
// Prefix: 10 frame-ID digits, comma, 2 row digits; final byte is the NUL.
static constexpr size_t PRINT_FRAMES_ROW_BUFFER_SIZE = 14 + FRAME_COLS * 11;

#define PRINT_FRAMES_STACK_SIZE ROUND_UP(sizeof(ThermalFrame) + PRINT_FRAMES_ROW_BUFFER_SIZE + 2048, 1024)

#define CAMERA_PROCESSING_SEGMENTS 7

// TODO: Implement the correct CAN codes and create DBC


enum class CansMsgCodes : uint32_t 
{
  TempData = 0x330,
  FrameDataIdentifier = 0x331
};


enum class PipeCanFrameTypes
{
  AVERAGED_STRIPS,
  FULL_FRAME,
};

enum class PipePrintModes
{
  NONE,
  STREAM,
  LOG_STATUS,
};

class ThermalPipeline
{
  public:
    explicit ThermalPipeline(ThermalCamera &camera, CanBus &can);

    int start();
    void close();

    void setPrintMode(PipePrintModes mode);


  private:
    static constexpr int PROCESSING_THREAD_PRIO = 6;
    static constexpr int PRINT_FRAMES_PRIO = 7;
    static constexpr size_t PRINT_QUEUE_LEN = 5;
    static constexpr uint8_t CAN_SIZE = 8;

    PipePrintModes pipePrintMode = PipePrintModes::LOG_STATUS;

    const struct device *uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    bool pushSummaryToCan();

    static void threadEntry(void *p1, void *p2, void *p3);
    void processingLoop();

    // Processing functions
    float getAveragePixel(ThermalFrame &frame);
    static int segmentCameraData(ThermalFrame &frame, float (&buf)[CAMERA_PROCESSING_SEGMENTS]);


    /**
     * @brief Generate a framestrip given two points p1 and p2.
     *
     * Given the two points a averaged strip is placed into the stripBuff with p1 acting as the top left point 
     * and p2 as the inclusive bottom right, forming a rectangle. 
     *
     * @param frame Reference to the thermal frame 
     * @param stripBuff Vector float containing the strip data
     * @param p1 the top left start index of the strip in form float pair (x,y).
     * @param p2 the bottom right (inclusive) stop index of the strip. 
     * @param avg boolean flag that if true averages all columns in the strip. 
     * @return 0 on success EINVAL if p1 or p2 are out of range or p2 is > p1
     */
    static int getFrameStrip(ThermalFrame &frame, std::vector<float> &stripBuff, std::pair<size_t, size_t> p1, std::pair<size_t, size_t> p2, bool avg = true);
    
    static uint16_t encodeTemp(const float &temp);

    // PrintFrames Thread
    struct k_thread printFramesThread;
    k_tid_t printFramesTID = nullptr;

    // SOH (status of health) messages
    const uint16_t messageIntervalSec = 5;
    int64_t lastMessageTime = k_uptime_get();
    int64_t statusStartTime = 0;
    uint32_t statusStartFrameId = 0;

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

    friend class testAccess;
};
