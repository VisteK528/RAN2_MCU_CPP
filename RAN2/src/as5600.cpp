#include "../include/as5600.hpp"

// Coefficient for converting 12-bit information(0-4095) about angle to degrees (0-360)
static float coefficient = 360/4096.f;

// Raw angle register map
static const uint8_t raw_angle_low = 0x0D; // Raw angle (7:0)
static const uint8_t raw_angle_high = 0x0C; // Raw angle (11:8)

static const uint8_t angle_low = 0x0F; // Angle (7:0)
static const uint8_t angle_high = 0x0E; // Angle (11:8)

/// @brief \n Reads data from single 1 byte (8 bits) AS5600 data register
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @param REGISTER_ADDRESS type: uint8_t - hexadecimal address of AS5600 sensor register to be read
/// @returns data type: uint8_t - raw data read from the selected AS5500 register
static uint8_t as5600_read_from_single_register(I2C_HandleTypeDef* i2c, uint8_t ADDRESS, uint8_t REGISTER_ADDRESS){
    uint8_t data = 0;
    HAL_I2C_Mem_Read(i2c, ADDRESS<<1, REGISTER_ADDRESS, 1, &data, 1, 1000);
    return data;
}

/// @brief \n Reads data from single 2 byte (16 bits) AS5600 data register
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @param REGISTER_LSB_ADDRESS type: uint8_t - hexadecimal address of low byte of AS5600 sensor register to be read
/// @param REGISTER_MSB_ADDRESS type: uint8_t - hexadecimal address of high byte of AS5600 sensor register to be read
/// @returns data type: uint16_t - raw data read from the selected AS5500 register
static uint16_t as5600_read_from_double_register(I2C_HandleTypeDef* i2c, uint8_t ADDRESS, uint8_t REGISTER_LSB_ADDRESS,
                                          uint8_t REGISTER_MSB_ADDRESS){
    uint16_t data, high_byte, low_byte;
    low_byte = as5600_read_from_single_register(i2c, ADDRESS, REGISTER_LSB_ADDRESS);
    high_byte = (uint16_t)as5600_read_from_single_register(i2c, ADDRESS, REGISTER_MSB_ADDRESS) << 8;
    data = high_byte | low_byte;
    return data;
}

uint8_t as5600_check_presence(I2C_HandleTypeDef* i2c, uint8_t ADDRESS){
    HAL_StatusTypeDef device_status = HAL_I2C_IsDeviceReady(i2c, ADDRESS<<1, 5, 1000);

    if(device_status != HAL_OK){
        return 1;
    }
    return 0;
}

uint8_t as5600_init(I2C_HandleTypeDef* i2c, uint8_t ADDRESS){
    // 1. Check if the sensor is available at given address by reading one empty byte from the sensor

    if(as5600_check_presence(i2c, ADDRESS) == 1){
        return 1;
    }

    // Check if the magnet is present by reading the 0x0B STATUS register
    uint8_t status = as5600_get_magnet_status(i2c, ADDRESS);
    if(status == 0){
        return 0;
    }
    else if(status == 1){
        return 2;
    }
    else if(status == 2){
        return 3;
    }
    return 0;
}

uint8_t as5600_get_magnet_status(I2C_HandleTypeDef* i2c, uint8_t ADDRESS){
    // Check if the magnet is present by reading the 0x0B STATUS register
    typedef enum{MAGNET_HIGH=0x08, MAGNET_LOW=0x10, MAGNET_DETECTED=0x20} STATUS;
    uint8_t status = as5600_read_from_single_register(i2c, ADDRESS, 0x0B);

    if(status == MAGNET_LOW){
        return 1;
    }
    else if(status == MAGNET_HIGH){
        return 2;
    }
    return 0;
}

float as5600_read_raw_angle(I2C_HandleTypeDef *i2c, uint8_t ADDRESS)
{
    float value;
    int raw_value = as5600_read_from_double_register(i2c, ADDRESS, raw_angle_low, raw_angle_high);
    value = (float)raw_value*coefficient;
    return value;
}

float as5600_read_angle(I2C_HandleTypeDef *i2c, uint8_t ADDRESS)
{
    int raw_value = as5600_read_from_double_register(i2c, ADDRESS, angle_low, angle_high);
    return (float)raw_value;
}
