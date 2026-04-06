#include "thermal_camera.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(ThermalCamera);

ThermalCamera::ThermalCamera()
{
    k_mutex_init(&frameMutex_);
    k_sem_init(&frameReadySem_, 0, 1);
}

int ThermalCamera::init()
{
    sensor_init_timestamp_S = k_uptime_get() / 1000;

    return 0;
}

bool ThermalCamera::isWarm(void)
{
    int64_t elapsed_ms = (k_uptime_get() / 1000) - sensor_init_timestamp_S;
    return elapsed_ms >= MLX90640_WARMUP_S;
}

