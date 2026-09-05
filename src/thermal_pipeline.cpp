#include "thermal_pipeline.h"
#include "can.h"
#include "thermal_camera.h"
#include "zephyr/drivers/can.h"
#include "zephyr/kernel.h"
#include "zephyr/kernel/thread_stack.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/util.h"
#include "zephyr/sys/printk.h"
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdint.h>

LOG_MODULE_REGISTER(ThermalPipline);

K_THREAD_STACK_DEFINE(printFramesStack, PRINT_FRAMES_STACK_SIZE);
K_THREAD_STACK_DEFINE(processingStack, PIPE_THREAD_STACK_SIZE);

ThermalPipeline::ThermalPipeline(ThermalCamera &camera, CanBus &can) : can_(can), camera_(camera)
{
}

int ThermalPipeline::start()
{
    can_.start();

    running_ = true;

    processingTID = k_thread_create(&processingThread, processingStack, K_THREAD_STACK_SIZEOF(processingStack),
                                    threadEntry, this, nullptr, nullptr, PROCESSING_THREAD_PRIO, 0, K_NO_WAIT);

    LOG_INF("Thermal Pipline initalized");


    k_msgq_init(&printFramesQueue,
                reinterpret_cast<char *>(msqQBuff),
                sizeof(ThermalFrame),
                PRINT_QUEUE_LEN
                );


    printFramesTID = k_thread_create(&printFramesThread, printFramesStack, K_THREAD_STACK_SIZEOF(printFramesStack),
                                     printFramesEntry, this, nullptr, nullptr, PRINT_FRAMES_PRIO, 0, K_NO_WAIT);

    LOG_INF("Thermal Pipeline print frames thread innitialized");

    return 0;
}

void ThermalPipeline::threadEntry(void *p1, void *p2, void *p3)
{
    LOG_INF("---ENTERED PIPE THREAD---");
    static_cast<ThermalPipeline *>(p1)->processingLoop();
}

void ThermalPipeline::processingLoop()
{

    // TODO: implement error handling
    while (running_)
    {

        ThermalFrame *framePtr = camera_.getUpdatedFrame();

        if (!framePtr)
        {
            LOG_DBG("Frame not updated frame not ready");
            k_sleep(K_MSEC(10));
            continue;
        }
        else
        {
            lastProcessedFrameId_ = framePtr->frameId;
        }

        if (framePtr->frameId > lastProcessedFrameId_ + 1)
        {
            LOG_DBG("Skipped frame processing taken too long");
        }

        if (printData)
        {
            k_msgq_put(&printFramesQueue, framePtr, K_NO_WAIT);
        }

        uint16_t avg = encodeTemp(getAveragePixel(*framePtr));

        // LITTLE ENDIAN
        struct can_frame cnframe{
            .id = AVERAGE_PIXEL_MSG, .dlc = 2, .flags = 0, .data = {(uint8_t)avg, (uint8_t)(avg >> 8)}};

        int ret = can_.send(&cnframe, K_MSEC(10));

        if (ret != 0)
        {

            LOG_DBG("ERROR: CAN message not sent");
        }
    }
}

/**
 * @brief Encode thermal temp to pass with CAN (Multiplys by 10 and encodes as uint8)
 *
 * @param temp
 * @return uint16_t
 */
uint16_t ThermalPipeline::encodeTemp(const float &temp)
{
    return static_cast<uint16_t>(temp * 10.0f);
    // 37.5C -> 375C (uint16)

    // NOTE THIS METHOD CANNOT STORE VALUES OF THE RANGE t < .09
    // All values save the first decimal point and encode as a uint16_t
}

int ThermalPipeline::segementCameraData(ThermalFrame &frame, float (&buf)[CAMERA_PROCESSING_SEGMENTS],
                                        uint8_t seg_height)
{

    //    Camera Frame
    //     --------
    //
    //     ||||||||  <- Strips have a height that is determined by *seg_height* and detemine how "tall" the strips are
    //
    //     --------

    seg_height = MIN(seg_height, FRAME_ROWS);
    LOG_WRN("The configured segmentation height (%i) was greater than the avalible rows (%i). It was clamped.",
            seg_height, FRAME_COLS);

    int seg_start = FRAME_COLS / 2;
    int center_width = seg_height / 2;

    uint8_t seg_width = std::ceil((double)FRAME_COLS / CAMERA_PROCESSING_SEGMENTS);

    // with truncation this will be at most 1 short
    for (int row = seg_height - center_width; row < seg_start + center_width; row++)
    {
        for (int col = 0; col < FRAME_COLS; col++)
        {

            int seg_num = static_cast<int>(col / seg_width);

            buf[seg_num] = frame.pixels[row * FRAME_COLS + col];
        }
    }

    // Handles the case where the number of segements doesnt equally split up the thermal camera.
    if (FRAME_COLS % CAMERA_PROCESSING_SEGMENTS != 0)
    {

        for (int i = 0; i < CAMERA_PROCESSING_SEGMENTS - 1; i++)
            buf[i] /= seg_width;

        buf[CAMERA_PROCESSING_SEGMENTS - 2] /= FRAME_COLS - ((CAMERA_PROCESSING_SEGMENTS - 1) * seg_width);
    }
    else
    {

        for (int i = 0; i < CAMERA_PROCESSING_SEGMENTS; i++)
            buf[i] /= seg_width;
    }

    return 0;
}

float ThermalPipeline::getAveragePixel(ThermalFrame &frame)
{

    double sum{0};

    for (int row = 0; row < FRAME_ROWS; row++)
    {
        for (int col = 0; col < FRAME_COLS; col++)
        {
            sum += frame.pixels[row * FRAME_COLS + col];
        }
    }
    return sum / TOTAL_PIXELS;
}

void ThermalPipeline::close()
{

    camera_.close();

    can_.stop();

    if (!running_ && processingTID == nullptr)
    {
        LOG_DBG("Tried ending process without initializing");
        return;
    }

    running_ = false;

    k_thread_abort(processingTID);

    k_thread_join(processingTID, K_FOREVER);
}

void ThermalPipeline::printSimple(ThermalFrame &Frame)
{

    static char rowBuf[FRAME_COLS * 6 + 1];

    for (int row = 0; row < FRAME_ROWS; row++)
    {
        int offset = 0;
        for (int col = 0; col < FRAME_COLS; col++)
        {
            float t = Frame.pixels[row * FRAME_COLS + col];
            int whole = (int)t;
            int frac = (int)((t - (float)whole) * 10);
            if (frac < 0)
            {
                frac = -frac;
            }
            offset += snprintf(rowBuf + offset, sizeof(rowBuf) - offset, "%3d.%d ", whole, frac);
        }
        LOG_INF("%02d: %s", row, rowBuf);
    }
}

void ThermalPipeline::printFramesEntry(void *instance, void *, void *)
{

    static_cast<ThermalPipeline *>(instance)->printFramesThreadWrk();
}

/**
 * @brief A function that uses message queues to send data through the USART to the consumer. 
 * 
 * @return int (maybe should be void)
 */

int ThermalPipeline::printFramesThreadWrk()
{
    // CSV: frame_id,row_index,pixel_0,...,pixel_31 (zero-based rows).
    char rowBuf[PRINT_FRAMES_ROW_BUFFER_SIZE];

    while (true)
    {
        ThermalFrame dataFrame;

        k_msgq_get(&printFramesQueue, &dataFrame, K_FOREVER);

        for (int row = 0; row < FRAME_ROWS; row++)
        {
            size_t offset = std::snprintf(rowBuf, sizeof(rowBuf), "%lu,%d",
                                          static_cast<unsigned long>(dataFrame.frameId), row);
            for (int col = 0; col < FRAME_COLS; col++)
            {
                int written = std::snprintf(rowBuf + offset, sizeof(rowBuf) - offset, ",%.4g",
                                            static_cast<double>(dataFrame.pixels[row * FRAME_COLS + col]));
                if (written < 0 || static_cast<size_t>(written) >= sizeof(rowBuf) - offset)
                {
                    LOG_ERR("Failed to format thermal frame row");
                    break;
                }
                offset += static_cast<size_t>(written);
                if (col == FRAME_COLS - 1)
                {
                    printk("%s\n", rowBuf);
                }
            }
        }
    }
    return 0;
}
