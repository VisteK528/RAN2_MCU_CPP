#ifndef RAN2_MCU_CPP_DISPLAY_HPP
#define RAN2_MCU_CPP_DISPLAY_HPP

#include "hagl.h"
#include "rgb565.h"
#include "errors.hpp"
#include <string>
#include <sstream>
#include <iomanip>
//#include "../../tft_lcd/fonts/font7x13B-ISO8859-1.h"

class Display{
public:
    Display();
    void init();
    void refresh();
    void printStatus(operation_status status);
    void printCommand(wchar_t* command);
private:
    void printHeader();
    hagl_backend_t* display;
};

#endif //RAN2_MCU_CPP_DISPLAY_HPP
