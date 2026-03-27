#include "thermal_camera.h"
#include "zephyr/drivers/i2c.h"
#include "zephyr/kernel.h"
#include "zephyr/logging/log.h"


LOG_MODULE_REGISTER(ThermalCamera);


ThermalCamera::ThermalCamera(const i2c_dt_spec &i2c) : i2c_(i2c)
{
    k_mutex_init(&frameMutex_);
    k_sem_init(&frameReadySem_,0,1);

}


/**
 * @brief I2C adapter to hide the i2c object
 * 
 * @param reg entry register
 * @param buf buffer to read data into 
 * @param len number of bytes contained in the data
 * @return int 
 */
int ThermalCamera::i2c_read(uint16_t reg, uint8_t *buf, size_t len){
    
    if(i2c_is_ready_dt(&i2c_)){
        LOG_ERR("I2C device is not ready and cannot read");
    }

    int ret = i2c_burst_read_dt(&i2c_, reg, buf, len);
    if (ret < 0) {
        LOG_ERR("I2C read failed at reg 0x%x: %d", reg, ret);
        return ret;  
    }
    return 0; 
}


bool ThermalCamera::newData(){
    return false;



}



