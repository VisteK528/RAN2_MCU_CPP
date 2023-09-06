#include "../include/alt_main.hpp"
#include "../include/robot.hpp"
#include <cstring>
#include <string>
#include <iomanip>
#include "../Inc/spi.h"
#include "../include/display.hpp"

#define LINE_MAX_LENGTH	80

static char line_buffer[LINE_MAX_LENGTH + 1];
static wchar_t line_buffer_display[LINE_MAX_LENGTH + 1];
static uint32_t line_length;
bool start = false;

operation_status robot_operation_status;
operation_status background_robot_operation_status;
uint8_t to_be_displayed = 0;

Robot my_robot;
coordinates coords[6] = {};
float angles[6] = {};
std::string coords_header;
std::string coords_data;

static void clearCharArray(wchar_t* w_array, uint16_t length){
    for(uint16_t i = 0; i < length; i++){
        w_array[i] = 0;
    }
}

static void convertCharArrayToWChar(const char* array, wchar_t* w_array, uint16_t length){
    for(uint16_t i = 0; i < length; i++){
        w_array[i] = (wchar_t)array[i];
    }
}

static std::string generateCoordinatesHeader(){
    std::stringstream display_string;
    display_string << std::setfill (' ') << std::setw(10) << "X";
    display_string << std::setfill (' ') << std::setw(10) << "Y";
    display_string << std::setfill (' ') << std::setw(10) << "Z";
    return display_string.str();
}

static std::string generateCoordinatesString(coordinates* coords){
    std::stringstream display_string;
    display_string << " " <<std::fixed << std::setprecision(4) << std::setfill (' ') << std::setw(9) << coords->x << " ";
    display_string << std::fixed << std::setprecision(4) << std::setfill (' ') << std::setw(9) << coords->y << " ";
    display_string << std::fixed << std::setprecision(4) << std::setfill (' ') << std::setw(9) << coords->z;
    return display_string.str();
}

int __io_putchar(int ch)
{
    if (ch == '\n') {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return 1;
}

uint8_t line_append(uint8_t value)
{
    if (value == '\r' || value == '\n') {
        if (line_length > 0) {
            line_buffer[line_length] = '\0';
            line_length = 0;
            return 0;
        }
    }
    else {
        if (line_length >= LINE_MAX_LENGTH) {
            line_length = 0;
        }
        if(((uint8_t)value == 8  || (uint8_t)value == 127 ) && line_length > 0){
            line_buffer[--line_length] = 0;
        }
        else{
            line_buffer[line_length++] = value;
        }
    }
    return 1;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2)
    {
        lcd_transfer_done();
    }
}

int alt_main() {
    HAL_TIM_Base_Start_IT(&htim1);

    Display display;
    display.init();

    printf("RAN2 Software MCU©\n");

    my_robot = buildRobot();
    my_robot.systemsCheck();

    start = true;

    while (1) {
        uint8_t uart_value = 0;
        if (HAL_UART_Receive(&huart2, &uart_value, 1, 0) == HAL_OK) {
            if (line_append(uart_value) == 0) {
                robot_operation_status.result = in_progress;
                display.printStatus(robot_operation_status);

                robot_operation_status = executeGCODE(my_robot, line_buffer);
                memset(line_buffer, 0, 80);
                if (robot_operation_status.result == success) {
                    printf("%d\n", 0x00);                       // Success
                } else {
                    printf("%d\n", 0x01);                       // Failure
                }
                display.printStatus(robot_operation_status);

                // Getting coordinates of End Effector from robot and then printing them on display, to be optimised
                my_robot.getRobotArmCoordinates(coords);
                coords_header = generateCoordinatesHeader();
                convertCharArrayToWChar(coords_header.c_str(), line_buffer_display, coords_header.length());
                display.printCustomString(line_buffer_display, 0);
                clearCharArray(line_buffer_display, LINE_MAX_LENGTH + 1);
                coords_data = generateCoordinatesString(&coords[5]);
                convertCharArrayToWChar(coords_data.c_str(), line_buffer_display, coords_data.length());
                display.printCustomString(line_buffer_display, 1);
                clearCharArray(line_buffer_display, LINE_MAX_LENGTH + 1);

            } else if (uart_value != '\0' && line_length > 0) {
                printf("Got: %s\n", line_buffer);

                convertCharArrayToWChar(line_buffer, line_buffer_display, LINE_MAX_LENGTH + 1);
                display.printCommand(line_buffer_display);
            }

            fflush(stdin);
            fflush(stdout);
        }

        if (to_be_displayed > 0) {
            display.printStatus(background_robot_operation_status);
            to_be_displayed--;
        }

    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim != &htim1){
        DriverHandleCallback(htim);
    }
    else if(start && !my_robot.getMovement()){
        if(my_robot.systemsCheck().result == success){
            background_robot_operation_status = my_robot.updateEncoders();
            if(background_robot_operation_status.result == failure){
                to_be_displayed++;
            }
        }
    }

}