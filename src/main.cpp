#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <syscalls/can.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <zephyr/debug/cpu_load.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/mem_stats.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_heap.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include "can.h"
#include "thermal_Camera.h"
#include "thermal_pipeline.h"

#ifndef __cplusplus
#error "__cplusplus not defined! Build system is compiling as C!"
#endif

LOG_MODULE_REGISTER(main);

CAN_MSGQ_DEFINE(main_can_rx_msgq, 1000);

/*
int main(void)
{

    LOG_INF("Main Innit");

    const struct device *can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

    static CanBus can;
    can.init(can_dev);

    static ThermalCamera MLX{};
    MLX.init();
    MLX.setRefreshRate(TC_RefreshRate::REFRESH_32_HZ);

    static ThermalPipeline pipe{MLX, can};
    pipe.setPrintMode(PipePrintModes::LOG_STATUS);
    pipe.start();

    k_sleep(K_FOREVER);
}
*/

int main(void)
{
    LOG_INF("Main Innit");

    static ThermalCamera MLX{};
    MLX.init();
    MLX.setRefreshRate(TC_RefreshRate::REFRESH_32_HZ);


    int64_t now_ms = k_uptime_get();
    int64_t lastMessageTime_ms = now_ms;

    while (true) {

        now_ms = k_uptime_get();
        MLX.getUpdatedFrame();
        
        if (now_ms - lastMessageTime_ms >= 1000){

            double fps = 1000/(double)(k_uptime_get()-now_ms);
            float loadPer =  static_cast<float>((cpu_load_get(false))/10);
            LOG_INF("PIPE SOH: FRAME FPS: %.2f CPU usage: %.2f", static_cast<double>(fps), loadPer);

            lastMessageTime_ms = now_ms;
        }

    }
    
}




