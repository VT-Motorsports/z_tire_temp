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

    return 0;
}

void ThermalPipline::threadEntry(void *p1, void *p2, void *p3)
{
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
            break;
        }
        else
        {
            lastProcessedFrameId_ = framePtr->frameId;
            // Will this break on priming?
        }

        if (framePtr->frameId > lastProcessedFrameId_ + 1)
        {
            LOG_DBG("Skipped frame processing taken too long");
            break;
        }

        if (printData){
            camera_.logFrame(*framePtr);
        }




        uint16_t avg = encodeTemp(getAveragePixel(*framePtr));

        struct can_frame cnframe
        {
            .id = AVERAGE_PIXEL_MSG,
            .dlc = 2,
            .flags = 0,
            .data = {(uint8_t)encodeTemp(avg),uint8_t() }
        };

        int ret = can_.send(&cnframe, K_MSEC(10));

        if (ret !=0){

            LOG_DBG("ERROR: CAN message not sent");

        }

    }
}

/**
 * @brief Encode thermal temp to pass with CAN
 *
 * @param temp
 * @return uint16_t
 */
uint16_t ThermalPipline::encodeTemp(float &temp)
{
    return static_cast<uint16_t>(temp + 100.0f);
    // 37.5C -> 375C (uint16)
}

// not sure if this is correct but want an overload that can do lvalue and rvalue params
uint16_t ThermalPipline::encodeTemp(float temp){ 
    return static_cast<uint16_t>(temp + 100.0f);
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
