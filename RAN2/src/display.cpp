#include "../include/display.hpp"
#include "font6x9.h"
#include "font5x7.h"
#include "font10x20_ISO8859_1.h"

static std::wstring code_to_hex(operation_result_code code){
    std::wstringstream stream;
    stream << "0x" << std::setfill(L'0') << std::setw(sizeof(operation_module_code)*2) << std::hex << code;
    return stream.str();
}

static std::wstring module_to_hex(operation_module_code module){
    std::wstringstream stream;
    stream << "0x" << std::setfill(L'0') << std::setw(sizeof(operation_module_code)*2) << std::hex << module;
    return stream.str();
}

Display::Display() {

}

void Display::init() {
    display = hagl_init();

    hagl_clear(display);
    printHeader();
    refresh();
}

void Display::refresh() {
    hagl_flush(display);
}

void Display::printHeader() {
    hagl_put_text(display, L"RAN3 Software©", 0, 0, rgb565(255, 255, 255), font10x20_ISO8859_1);
    hagl_put_text(display, L"Command: ", 0, 50, rgb565(255, 255, 255), font6x9);

    hagl_put_text(display, L"Result   Module    Code", 0, 80, rgb565(255, 255, 255), font6x9);
}

void Display::printStatus(operation_status status) {
    std::wstring w_string;
    const wchar_t* status_wchar_string;

    hagl_fill_rectangle_xywh(display, 0, 90, 160, 9, rgb565(0, 0, 0));

    if(status.result == in_progress){
        w_string += L"In progress...  ";

        status_wchar_string = w_string.c_str();
        hagl_put_text(display, status_wchar_string, 0, 90, rgb565(255, 255, 0), font6x9);
        refresh();
    }
    else{
        if(status.result == success){
            w_string += L"Success";

            status_wchar_string = w_string.c_str();
            hagl_put_text(display, status_wchar_string, 0, 90, rgb565(0, 255, 0), font6x9);
        }
        else{
            w_string += L"Failure";

            status_wchar_string = w_string.c_str();
            hagl_put_text(display, status_wchar_string, 0, 90, rgb565(255, 0, 0), font6x9);
        }
        w_string.clear();
        w_string += module_to_hex(status.module);
        w_string += L"      ";
        w_string += code_to_hex(status.code);

        status_wchar_string = w_string.c_str();
        hagl_put_text(display, status_wchar_string, 60, 90, rgb565(255, 255, 255), font6x9);
        refresh();
    }
}

void Display::printCommand(wchar_t* command) {
    hagl_fill_rectangle_xywh(display, 0, 60, 160-10+6*9, 9, rgb565(0, 0, 0));

    hagl_put_text(display, command, 0, 60, rgb565(255, 255, 255), font6x9);
    hagl_flush(display);
}