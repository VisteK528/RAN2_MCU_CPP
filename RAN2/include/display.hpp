#ifndef RAN2_MCU_CPP_DISPLAY_HPP
#define RAN2_MCU_CPP_DISPLAY_HPP

#include "hagl.h"
#include "rgb565.h"
#include "errors.hpp"
#include <string>
#include <sstream>
#include <iomanip>

class Display{
public:
    Display();
    void init();
    void refresh();
    void printStatus(operation_status status);
    void printCommand(wchar_t* command);
    void printCustomString(wchar_t* string, uint8_t row);
private:
    void printHeader();
    hagl_backend_t* display;
};

#endif //RAN2_MCU_CPP_DISPLAY_HPP
