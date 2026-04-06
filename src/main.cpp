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

#include "adc.h"
#include "hardware.h"
#include "system.h"
#include "vehicle_state.h"

#ifndef __cplusplus
#error "__cplusplus not defined! Build system is compiling as C!"
#endif

LOG_MODULE_REGISTER(main);

CAN_MSGQ_DEFINE(main_can_rx_msgq, 1000);


void send_heartbeat(Hardware &hw)
{
    static uint32_t heartbeat_counter = 0;

    struct can_frame heartbeat_frame = {.id = 0x100, // Heartbeat message ID
                                        .dlc = 8,
                                        .flags = 0,
                                        .data = {(uint8_t)(heartbeat_counter >> 24), (uint8_t)(heartbeat_counter >> 16),
                                                 (uint8_t)(heartbeat_counter >> 8), (uint8_t)(heartbeat_counter), 0xEF,
                                                 0xFF, 0xFF, 0xFF}};

    // TODO: Replace the CAN interface here as to remove abstraction created by hardware class
    int ret = hw.can1.send(&heartbeat_frame, K_NO_WAIT); 
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


}