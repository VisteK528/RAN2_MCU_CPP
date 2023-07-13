#ifndef RAN2_MCU_CPP_READ_GCODE_H
#define RAN2_MCU_CPP_READ_GCODE_H


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

uint8_t parseMessage(char* letter, float* value, const char* line, uint16_t* char_counter);

#endif //RAN2_MCU_CPP_READ_GCODE_H
