#include "../include/alt_main.hpp"
#include "../include/robot.hpp"
#include <cstring>
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

static void convertCharArrayToWChar(const char* array, wchar_t* w_array, uint16_t length){
    for(uint16_t i = 0; i < length; i++){
        w_array[i] = (wchar_t)array[i];
    }
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
        // odebraliśmy znak końca linii
        if (line_length > 0) {
            // jeśli bufor nie jest pusty to dodajemy 0 na końcu linii
            line_buffer[line_length] = '\0';
            // przetwarzamy dane
            // zaczynamy zbieranie danych od nowa
            line_length = 0;
            return 0;
        }
    }
    else {
        if (line_length >= LINE_MAX_LENGTH) {
            // za dużo danych, usuwamy wszystko co odebraliśmy dotychczas
            line_length = 0;
        }
        // dopisujemy wartość do bufora
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
    printf("Starting...\n");

    my_robot = buildRobot();
    my_robot.systemsCheck();

    printf("Status ready!\n");
    printf("Command: \n");

    MagneticEncoderData data;
    start = true;

    while (1) {
        //Test
        //my_robot.getEncoderData(6, &data);
        //printf("Position: %f\tVelocity: %f\tAcceleration: %f\n", data.position, data.velocity, data.acceleration);

        uint8_t uart_value;
        if (HAL_UART_Receive(&huart2, &uart_value, 1, 0) == HAL_OK) {
            if (line_append(uart_value) == 0) {
                robot_operation_status.result = in_progress;
                display.printStatus(robot_operation_status);

                robot_operation_status = executeGCODE(my_robot, line_buffer);
                memset(line_buffer, 0, 80);
                if (robot_operation_status.result == success) {
                    printf("Command executed successfully\n");
                } else {
                    printf("Command execution failed\n");
                }
                display.printStatus(robot_operation_status);

            } else if (uart_value != '\0' && line_length >= 0) {
                printf("Command: %s\n", line_buffer);

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
    if(htim == &htim1 && start){
        if(my_robot.systemsCheck().result == success){
            background_robot_operation_status = my_robot.updateEncoders();
            if(background_robot_operation_status.result == failure){
                to_be_displayed++;
            }
        }
    }
    else{
        DriverHandleCallback(htim);
    }
}