// hardware.cpp
#include "hardware.h"
#include "adc.h"
#include "vehicle_state.h"
#include "zephyr/drivers/can.h"
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hardware);

int Hardware::init()
{
    LOG_INF("Initializing hardware...");

    if (initializeADCs() != 0)
    {
        LOG_ERR("Failed to initialize ADCs");
        return -1;
    }

    if (initializeGPIOs() != 0)
    {
        LOG_ERR("Failed to initialize GPIOs");
        return -2;
    }

    if (initializeCANs() != 0)
    {
        LOG_ERR("Failed to initialize CANs");
        return -3;
    }

    LOG_INF("Hardware initialized successfully");
    return 0;
}

Hardware::Hardware(VehicleState *state) : can1(state), can2(state), vehicle(state)
{
}

int Hardware::initializeADCs()
{
    LOG_INF("Initializing hardware...");

    // Get ADC device
    adc_dev_ = DEVICE_DT_GET(DT_NODELABEL(adc1));
    if (!device_is_ready(adc_dev_))
    {
        LOG_ERR("ADC device adc1 not ready");
        return -1;
    }

    /*
     * Devicetree -> physical pin mapping (from your DTS + pinout):
     *  chan0: PA0  -> ADC1_INP16 (reg = 16)
     *  chan1: PA1  -> ADC1_INP17 (reg = 17)
     *  chan2: PA2  -> ADC1_INP14 (reg = 14)
     *  chan3: PA3  -> ADC1_INP15 (reg = 15)
     *  chan4: PA4  -> ADC1_INP18 (reg = 18)
     *  chan5: PA5  -> ADC1_INP19 (reg = 19)
     *  chan6: PA6  -> ADC1_INP3  (reg = 3)
     *  chan7: PA7  -> ADC1_INP7  (reg = 7)
     *
     * Note: these values must match the DTS channel@N { reg = <...>; } entries,
     * not "channel@N" indices.
     */

    if (adc_chan0.init(adc_dev_, 16) != 0)
    {
        LOG_ERR("Failed to init adc_chan0 (PA0/INP16)");
        return -10;
    }
    if (adc_chan1.init(adc_dev_, 17) != 0)
    {
        LOG_ERR("Failed to init adc_chan1 (PA1/INP17)");
        return -11;
    }
    if (adc_chan2.init(adc_dev_, 14) != 0)
    {
        LOG_ERR("Failed to init adc_chan2 (PA2/INP14)");
        return -12;
    }
    if (adc_chan3.init(adc_dev_, 15) != 0)
    {
        LOG_ERR("Failed to init adc_chan3 (PA3/INP15)");
        return -13;
    }
    if (adc_chan4.init(adc_dev_, 18) != 0)
    {
        LOG_ERR("Failed to init adc_chan4 (PA4/INP18)");
        return -14;
    }
    if (adc_chan5.init(adc_dev_, 19) != 0)
    {
        LOG_ERR("Failed to init adc_chan5 (PA5/INP19)");
        return -15;
    }
    if (adc_chan6.init(adc_dev_, 3) != 0)
    {
        LOG_ERR("Failed to init adc_chan6 (PA6/INP3)");
        return -16;
    }
    if (adc_chan7.init(adc_dev_, 7) != 0)
    {
        LOG_ERR("Failed to init adc_chan7 (PA7/INP7)");
        return -17;
    }

    LOG_INF("Hardware initialized successfully");
    return 0;
}

uint16_t Hardware::getADCValue(uint8_t channel)
{
    static AdcChannel *const adc_table[8] = {&adc_chan0, &adc_chan1, &adc_chan2, &adc_chan3,
                                             &adc_chan4, &adc_chan5, &adc_chan6, &adc_chan7};

    if (channel >= 8)
    {
        return -1;
    }

    return adc_table[channel]->read_raw();
}

int Hardware::initializeGPIOs()
{
    // Get GPIO ports
    gpioe_ = DEVICE_DT_GET(DT_NODELABEL(gpioe));
    gpioc_ = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    gpioa_ = DEVICE_DT_GET(DT_NODELABEL(gpioa));

    if (!gpioe_ || !gpioc_ || !gpioa_)
    {
        LOG_ERR("Failed to get GPIO ports");
        return -1;
    }

    // Initialize LEDs (PE2-PE6)
    if (led_yellow.init(gpioe_, 2, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init led_yellow");
        return -10;
    }
    if (led_orange.init(gpioe_, 3, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init led_orange");
        return -11;
    }
    if (led_red.init(gpioe_, 4, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init led_red");
        return -12;
    }
    if (led_blue.init(gpioe_, 5, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init led_blue");
        return -13;
    }
    if (led_green.init(gpioe_, 6, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init led_green");
        return -14;
    }

    // Initialize control signals
    if (horn_signal.init(gpioc_, 8, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init horn_signal");
        return -20;
    }
    if (drive_enable.init(gpioc_, 9, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init drive_enable");
        return -21;
    }
    if (air_ctrl.init(gpioa_, 8, GPIO_OUTPUT_INACTIVE) != 0)
    {
        LOG_ERR("Failed to init air_ctrl");
        return -22;
    }

    LOG_INF("GPIOs initialized");
    return 0;
}

int Hardware::initializeCANs()
{
    // Get CAN devices
    can1_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan1));
    can2_dev = DEVICE_DT_GET(DT_NODELABEL(fdcan2));

    if (!can1_dev || !can2_dev)
    {
        LOG_ERR("Failed to get CAN devices");
        return -1;
    }

    // Initialize CAN1 (1 Mbps)
    if (can1.init(can1_dev, 1000000, 875) != 0)
    {
        LOG_ERR("Failed to init CAN1");
        return -10;
    }

    if (can1.start() != 0)
    {
        LOG_ERR("Failed to start CAN1");
        return -11;
    }

    // Initialize CAN2 (1 Mbps)
    if (can2.init(can2_dev, 1000000, 875) != 0)
    {
        LOG_ERR("Failed to init CAN2");
        return -20;
    }

    if (can2.start() != 0)
    {
        LOG_ERR("Failed to start CAN2");
        return -21;
    }

    can1.set_mode(CAN_MODE_LOOPBACK);

    LOG_INF("CANs initialized");
    return 0;
}