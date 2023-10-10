#ifndef RAN2_MCU_CPP_READ_GCODE_H
#define RAN2_MCU_CPP_READ_GCODE_H


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string>
#include <vector>
#include "../../libraries/SDP/include/sdp.h"

uint8_t parseMessage(char* letter, float* value, const char* line, uint16_t* char_counter);
std::vector<std::string> splitCommandFile(SDP_Message* message);
std::vector<std::string> splitCommandFile(const std::string& message);
std::vector<std::string> splitCommandFile(uint8_t* message, uint32_t length);


#endif //RAN2_MCU_CPP_READ_GCODE_H
