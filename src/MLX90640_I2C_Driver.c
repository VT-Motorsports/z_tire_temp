#include "MLX90640_I2C_Driver.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mlx90640_i2c);

// Driver owns the I2C bus reference; ThermalCamera no longer holds it.
static const struct i2c_dt_spec mlx_i2c = I2C_DT_SPEC_GET(DT_NODELABEL(mlx90640));

void MLX90640_I2CInit(void)
{
    if (!i2c_is_ready_dt(&mlx_i2c)) {
        LOG_ERR("MLX90640 I2C bus not ready");
    }
}

int MLX90640_I2CGeneralReset(void)
{
    // General call: address 0x00, command byte 0x06 resets all I2C devices on the bus.
    uint8_t cmd = 0x06;
    int ret = i2c_write(mlx_i2c.bus, &cmd, sizeof(cmd), 0x00);
    if (ret < 0) {
        LOG_ERR("I2C general reset failed: %d", ret);
        return -MLX90640_I2C_WRITE_ERROR;
    }
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data)
{
    (void)slaveAddr; // default address is defined in the DT spec

    uint8_t reg[2] = { (uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF) };
    uint16_t nBytes = nMemAddressRead * sizeof(uint16_t);

    int ret = i2c_write_read_dt(&mlx_i2c, reg, sizeof(reg), (uint8_t *)data, nBytes);
    if (ret < 0) {
        LOG_ERR("I2C read failed at 0x%04x: %d", startAddress, ret);
        return -MLX90640_I2C_NACK_ERROR;
    }

    // Sensor sends each 16-bit word big-endian; swap to host order.
    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        uint8_t *p = (uint8_t *)&data[i];
        data[i] = ((uint16_t)p[0] << 8) | p[1];
    }

    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    (void)slaveAddr; // address is owned by the dt_spec

    uint8_t buf[4] = {
        (uint8_t)(writeAddress >> 8),
        (uint8_t)(writeAddress & 0xFF),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };

    int ret = i2c_write_dt(&mlx_i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("I2C write failed at 0x%04x: %d", writeAddress, ret);
        return -MLX90640_I2C_WRITE_ERROR;
    }
    return MLX90640_NO_ERROR;
}

void MLX90640_I2CFreqSet(int freq)
{
    // I2C speed is set in DTS; Zephyr doesn't support runtime frequency changes
    // on most drivers. Keep as no-op to satisfy the API contract.
    (void)freq;
}
