#include "thermal_pipeline.h"
#include "can.h"
#include "thermal_Camera.h"
#include <cstddef>
#include <sys/_types.h>
#include <vector>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/debug/cpu_load.h>
#include "zephyr/kernel.h"
#include "zephyr/kernel/thread_stack.h"
#include "zephyr/logging/log.h"
#include "zephyr/sys/byteorder.h"
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


    switch (pipePrintMode) {

        case PipePrintModes::STREAM: {
            k_msgq_init(&printFramesQueue,
            reinterpret_cast<char *>(msqQBuff),
            sizeof(ThermalFrame),
            PRINT_QUEUE_LEN
            );

            printFramesTID = k_thread_create(&printFramesThread, printFramesStack, K_THREAD_STACK_SIZEOF(printFramesStack),
                                        printFramesEntry, this, nullptr, nullptr, PRINT_FRAMES_PRIO, 0, K_NO_WAIT);

            LOG_INF("Thermal Pipeline print frames thread innitialized");
            break;
        }

        default:

            break;
    }

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
            LOG_ERR("Frame ");
            k_sleep(K_MSEC(10));
            continue;
        }
        else
        {
            lastProcessedFrameId_ = framePtr->frameId;
        }

        
        // Doing this with event flags would be the ideal solution 
        switch (pipePrintMode) {

            case PipePrintModes::LOG_STATUS: {

                const int64_t now = k_uptime_get();

                if (statusStartTime == 0)
                {
                    statusStartTime = now;
                    statusStartFrameId = lastProcessedFrameId_;
                }
                
                if ((now - lastMessageTime) >= messageIntervalSec *1000){

                    lastMessageTime = now;

                    float loadPer =  static_cast<float>((cpu_load_get(false))/10);

                    const int64_t elapsed = now - statusStartTime;
                    if (elapsed > 0)
                    {
                        const uint32_t frames = lastProcessedFrameId_ - statusStartFrameId;
                        const float fps = 1000.0f * frames / elapsed;
                        LOG_INF("PIPE SOH: FRAME FPS: %.2f CPU usage: %.2f", static_cast<double>(fps), loadPer);
                    }

                    statusStartTime = now;
                    statusStartFrameId = lastProcessedFrameId_;
                }
                break;
            }

            case PipePrintModes::STREAM:
                k_msgq_put(&printFramesQueue, framePtr, K_NO_WAIT); 
                break;

            case PipePrintModes::NONE:
                
                break;        
        }

        std::vector<float> strip;

        getFrameStrip(
            *framePtr,
            strip,
            {0,10},
            {24,11},
            true
        );


        sendInfoFrame(PipeCanFrameTypes::AVERAGED_STRIPS);

        sendTempsOverCan(strip, framePtr->frameId);
    
    }

}



int ThermalPipeline::sendTempsOverCan(const std::vector<float> &temps, uint16_t frameID){

    printk("CAN strip (frame %u): [", static_cast<unsigned int>(frameID));
    for (size_t i = 0; i < temps.size(); ++i) {
        printk("%s%.2f", i == 0 ? "" : ", ", static_cast<double>(temps[i]));
    }
    printk("]\n");

    if (temps.size() > UINT16_MAX){
        return -EMSGSIZE;
    }
    
    constexpr size_t HEADER_SIZE_BYTES = 4;
    constexpr size_t TEMP_SIZE_BYTES = sizeof(uint16_t);
    constexpr size_t TEMPS_PER_FRAME = (CAN_SIZE - HEADER_SIZE_BYTES) / TEMP_SIZE_BYTES;
    static_assert(TEMPS_PER_FRAME > 0, "CAN payload must fit a temperature");

    
    for (float temp : temps) {
        if (!std::isfinite(temp) || temp < 0.0f ||
            temp * 10.0f > static_cast<float>(UINT16_MAX)) {
            return -ERANGE;
        }
    }

    for (size_t offset = 0; offset < temps.size();) {
        const size_t remaining = temps.size() - offset;
        const size_t count = remaining < TEMPS_PER_FRAME ? remaining : TEMPS_PER_FRAME;

        struct can_frame frame{};
        frame.id = static_cast<uint32_t>(CansMsgCodes::TempData);
        frame.dlc = static_cast<uint8_t>(HEADER_SIZE_BYTES + count * TEMP_SIZE_BYTES);

        // Big-endian: frame ID, first temperature index, then uint16_t temperatures.
        sys_put_be16(frameID, &frame.data[0]);
        sys_put_be16(static_cast<uint16_t>(offset), &frame.data[2]);

        for (size_t i = 0; i < count; ++i) {
            sys_put_be16(encodeTemp(temps[offset + i]),
                         &frame.data[HEADER_SIZE_BYTES + i * TEMP_SIZE_BYTES]);
        }

        const int ret = can_.send(&frame, K_MSEC(10));

        if (ret != 0)
        {
            // LOG_ERR("PIPE: CAN message not sent: %d", ret); TODO: RE-ENABLE
            return ret;
        }
        offset += count;
    }


    return 0;

}

/** 
* @brief 
* 
* @param frameType enum that determines the status message sent
* @param frameId if sending Full frames and or strips the ID of the frame 
* @return int: 0 on success
*/
int ThermalPipeline::sendInfoFrame(PipeCanFrameTypes frameType, int16_t frameId){

    
    struct can_frame infoFrame{};

    
    // This currently exists to send device info on CAN as well as info about transmission data. 

    switch (frameType) {
    
        case PipeCanFrameTypes::FULL_FRAME:
        case PipeCanFrameTypes::AVERAGED_STRIPS:
        case PipeCanFrameTypes::PARTIAL_FRAME:

            infoFrame.id = static_cast<uint32_t>(CansMsgCodes::FrameDataIdentifier);

            sys_put_be16(deviceId, &infoFrame.data[0]);

            infoFrame.data[2] = static_cast<uint8_t>(frameType);

            sys_put_be16(frameId, &infoFrame.data[3]);

            infoFrame.dlc = 5;

            break;

        default:
            return -EINVAL;
            break;
    }

    int ret = can_.send(&infoFrame, K_MSEC(10));


    if (ret != 0 ){
        LOG_ERR("PIPE: Sending Thermal Data info frame failed");
        return ret;
    }

    return 0;

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
int ThermalPipeline::getFrameStrip(
        ThermalFrame &frame,
        std::vector<float> &stripBuff,
        std::pair<size_t, size_t> p1,
        std::pair<size_t, size_t> p2,
        bool avg
    ){

    if (
        (p1.first > p2.first) || (p1.second > p2.second) ||
        (p1.first >= FRAME_COLS) || (p1.second >= FRAME_ROWS)){
        return -EINVAL;
    }

    stripBuff.clear();

    for (size_t col = p1.first; col <= p2.first; col ++){
        for ( size_t row = p1.second; row<= p2.second; row ++){

            if (row == p1.second || !avg){
                stripBuff.push_back(frame.pixels[row * FRAME_COLS + col]);
            }else {
                stripBuff[col-p1.first] += frame.pixels[row * FRAME_COLS + col];
            }
        }
    }    

    if (avg){ //could move into last calls of double loop for small savings
        for (float &stripVal: stripBuff){
            size_t height = p2.second - p1.second + 1;
            stripVal /= height;
        }
    }


    return 0;
}

int ThermalPipeline::segmentCameraData(ThermalFrame &frame, float (&buf)[CAMERA_PROCESSING_SEGMENTS]){

    static bool logConfigErr = true;

    // If this is made dynamic this has to become runtime.
    constexpr uint16_t SEG_WIDTH =  ((FRAME_COLS + CAMERA_PROCESSING_SEGMENTS -1) / CAMERA_PROCESSING_SEGMENTS); 
    constexpr uint16_t SEG_OVERSHOOT = SEG_WIDTH * CAMERA_PROCESSING_SEGMENTS - FRAME_COLS; 

    if (SEG_OVERSHOOT != 0  && logConfigErr) LOG_WRN("The configured segmentation number doesn't evenly divide into the COL number");

    for (float &seg : buf) {
    seg = 0.0f;
    }


    for (size_t col = 0; col < FRAME_COLS; col ++){
        for ( size_t row = 0; row< FRAME_ROWS; row ++){
            size_t seg_number = (col+(SEG_OVERSHOOT/2))/SEG_WIDTH;
            buf[seg_number] += frame.pixels[row * FRAME_COLS + col];
        }
    }


    for (int seg_num = 0; seg_num < CAMERA_PROCESSING_SEGMENTS; ++seg_num) {
        int columns = SEG_WIDTH;

        if (seg_num == 0) {
            columns -= SEG_OVERSHOOT / 2;
        }
        if (seg_num == CAMERA_PROCESSING_SEGMENTS - 1) {
            columns -= SEG_OVERSHOOT - SEG_OVERSHOOT / 2;
        }

        buf[seg_num] /= FRAME_ROWS * columns;
    }


    logConfigErr = false;

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
                int written = std::snprintf(rowBuf + offset, sizeof(rowBuf) - offset, ",%.3g",
                                            static_cast<double>(dataFrame.pixels[row * FRAME_COLS + col]));
                if (written < 0 || static_cast<size_t>(written) >= sizeof(rowBuf) - offset)
                {
                    LOG_ERR("Failed to format thermal frame row");
                    break;
                }
                offset += static_cast<size_t>(written);
                if (col == FRAME_COLS - 1)
                {
                    
                    for (size_t i = 0; i < offset; ++i)
                    {
                    uart_poll_out(uart_dev, static_cast<unsigned char>(rowBuf[i]));
                    }
                    uart_poll_out(uart_dev, '\r');
                    uart_poll_out(uart_dev, '\n');

                }
            }
        }
    }
    return 0;
}


void ThermalPipeline::setPrintMode(PipePrintModes mode){
    pipePrintMode = mode;
}
