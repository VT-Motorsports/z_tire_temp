#include "thermal_pipline.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(ThermalPipline);

K_THREAD_STACK_DEFINE(processingStack, 2048);

ThermalPipline::ThermalPipline(ThermalCamera &camera) : camera_(camera)
{
}

int ThermalPipline::start()
{
    running_ = true;

    workerTid_ = k_thread_create(&workerThread_, processingStack, THREAD_STACK_SIZE, threadEntry,
                                 this, nullptr, nullptr, THREAD_PRIORITY, 0, K_NO_WAIT);

    return 0;
}

void ThermalPipline::threadEntry(void *p1, void *p2, void *p3)
{
    static_cast<ThermalPipline *>(p1)->processingLoop();
}

void ThermalPipline::processingLoop()
{
    while (running_)
    {
        // TODO:implement processing loop 
    }
}

void ThermalPipline::close()
{

    camera_.close();


    if (!running_ && workerTid_ == nullptr)
    {
        LOG_DBG("Tried ending process without initializing");
        return;
    }

    running_ = false;

    k_thread_abort(workerTid_);

    k_thread_join(workerTid_, K_FOREVER);
}
