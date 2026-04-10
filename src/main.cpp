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
#include "thermal_camera.h"
#include "thermal_pipline.h"

#ifndef __cplusplus
#error "__cplusplus not defined! Build system is compiling as C!"
#endif

LOG_MODULE_REGISTER(main);

CAN_MSGQ_DEFINE(main_can_rx_msgq, 1000);


void send_heartbeat(CanBus can)
{
    static uint32_t heartbeat_counter = 0;

    struct can_frame heartbeat_frame = {.id = 0x100, // Heartbeat message ID
                                        .dlc = 8,
                                        .flags = 0,
                                        .data = {(uint8_t)(heartbeat_counter >> 24), (uint8_t)(heartbeat_counter >> 16),
                                                 (uint8_t)(heartbeat_counter >> 8), (uint8_t)(heartbeat_counter), 0xEF,
                                                 0xFF, 0xFF, 0xFF}};

    int ret = can.send(&heartbeat_frame, K_NO_WAIT); 
    if (ret == 0)
    {
    }
    else
    {
        LOG_ERR("Heartbeat send failed: %d", ret);
    }

    heartbeat_counter++;
}

int main(void)

{

    LOG_INF("Main Innit");

    const struct device* can_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));

    CanBus can;
    can.init(can_dev);

    ThermalCamera MLX{};
    MLX.init();

    
    ThermalPipline pipe{MLX,can};

    pipe.start();
    pipe.printData = true;
    

}