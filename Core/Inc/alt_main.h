#ifndef RAN2_MCU_CPP_ALT_MAIN_H
#define RAN2_MCU_CPP_ALT_MAIN_H

#include "../RAN2/include/robot.hpp"
#include "../RAN2/include/read_gcode.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"

#include <string>
#include <cstring>
#include <stdio.h>

#define LINE_MAX_LENGTH	80

#ifdef __cplusplus
extern "C"
{
#endif

int alt_main();

#ifdef __cplusplus
}
#endif

#endif //RAN2_MCU_CPP_ALT_MAIN_H
