#include "thermal_pipline.h"
#include "can.h"
#include "thermal_camera.h"
#include "zephyr/drivers/can.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"
#include <cstdint>

LOG_MODULE_REGISTER(ThermalPipline);


K_THREAD_STACK_DEFINE(processingStack, PIPE_THREAD_STACK_SIZE);

ThermalPipline::ThermalPipline(ThermalCamera &camera, CanBus &can) : can_(can), camera_(camera)
{
}

int ThermalPipline::start()
{
    can_.start();

    running_ = true;

    workerTid_ = k_thread_create(&workerThread_, processingStack, THREAD_STACK_SIZE, threadEntry, this, nullptr,
                                 nullptr, THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("Thermal Pipline initalized");                                 
    return 0;
}

void ThermalPipline::threadEntry(void *p1, void *p2, void *p3)
{
    LOG_INF("---ENTERED PIPE THREAD---");
    static_cast<ThermalPipline *>(p1)->processingLoop();
}

void ThermalPipline::processingLoop()
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

        if (printData){
            camera_.logFrame(*framePtr);
        }




        uint16_t avg = encodeTemp(getAveragePixel(*framePtr));

        // LITTLE ENDIAN
        struct can_frame cnframe
        {
            .id = AVERAGE_PIXEL_MSG,
            .dlc = 2,
            .flags = 0,
            .data = {(uint8_t)avg,(uint8_t)(avg>>8) }
        };

        int ret = can_.send(&cnframe, K_MSEC(10));

        if (ret !=0){

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
uint16_t ThermalPipline::encodeTemp(const float &temp)
{
    return static_cast<uint16_t>(temp  * 10.0f);
    // 37.5C -> 375C (uint16)

    // NOTE THIS METHOD CANNOT STORE VALUES OF THE RANGE t < .09
    // All values save the first decimal point and encode as a uint16_t
}



float ThermalPipline::getAveragePixel(ThermalFrame &frame)
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

void ThermalPipline::close()
{

    camera_.close();

    can_.stop();

    if (!running_ && workerTid_ == nullptr)
    {
        LOG_DBG("Tried ending process without initializing");
        return;
    }

    running_ = false;

    k_thread_abort(workerTid_);

    k_thread_join(workerTid_, K_FOREVER);
}

void ThermalPipline::printSimple(ThermalFrame &Frame){


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