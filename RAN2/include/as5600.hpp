#ifndef RAN2_INCLUDE_AS5600_HPP_
#define RAN2_INCLUDE_AS5600_HPP_

#include <stdio.h>
#include "gpio.h"
#include "i2c.h"

/// @brief \n Initializes AS5600 sensor by checking if it is available at given address and if the magnet is present
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @returns type: uint8_t If the sensor was initialized successfully function returns 0,
/// otherwise the function returns 1.
uint8_t as5600_init(I2C_HandleTypeDef* i2c, uint8_t ADDRESS);

/// @brief \n Checks if the magnet is present and if its magnetic field optimal for sensor's normal activity.
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @returns type: uint8_t If the magnet is present and its magnetic field influence is sufficient the function
/// returns 0, otherwise the function returns 1.
uint8_t as5600_get_magnet_status(I2C_HandleTypeDef* i2c, uint8_t ADDRESS);

/// @brief \n Reads raw angle from AS5600 RAW_ANGLE register
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @returns data type: float - raw angle
float as5600_read_raw_angle(I2C_HandleTypeDef* i2c, uint8_t ADDRESS);

/// @brief \n Reads angle from AS5600 ANGLE register
/// @param i2c type: i2c_inst_t* -  raspberry pi pico i2c instance
/// @param ADDRESS type: uint8_t - hexadecimal i2c address of AS5600 sensor
/// @returns data type: float - angle
float as5600_read_angle(I2C_HandleTypeDef* i2c, uint8_t ADDRESS);


#endif /* RAN2_INCLUDE_AS5600_HPP_ */
