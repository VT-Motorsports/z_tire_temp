#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>


static const constexpr uint8_t FRAME_COLS = 24;
static const constexpr uint8_t FRAME_ROWS = 32;

struct ThermalFrame{
    float pixles[FRAME_COLS][FRAME_ROWS];
    bool warmedData = false;
};



class ThermalSensor
{

    public:

    // Dont pass the driver explicitly to make dependency injection easy
    explicit ThermalSensor(const i2c_dt_spec& i2c); 

    ThermalFrame getCurFrame();


    private:
    
    ThermalFrame curFrame;

    ThermalFrame _bufFrame1;
    ThermalFrame _bufFrame2;

    
    



};

