#include "../include/alt_main.hpp"
#include "../include/robot.hpp"
#include "hagl.h"
#include "font6x9.h"
#include "font5x7.h"
#include "font10x20_ISO8859_1.h"
//#include "../../tft_lcd/fonts/font7x13B-ISO8859-1.h"
#include "rgb565.h"
#include <cstring>
#include "../Inc/spi.h"

#define LINE_MAX_LENGTH	80


static char line_buffer[LINE_MAX_LENGTH + 1];
static wchar_t line_buffer_display[LINE_MAX_LENGTH + 1];
static uint32_t line_length;

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
        if((uint8_t)value == 8 && line_length > 0){
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

int alt_main(){
    hagl_backend_t* display = hagl_init();

    hagl_clear(display);
    hagl_put_text(display, L"RAN3 Soft©", 10, 10, rgb565(255, 255, 255), font10x20_ISO8859_1);
    hagl_put_text(display, L"Command: ", 0, 50, rgb565(255, 255, 255), font6x9);
    hagl_put_text(display, L"Status: ", 0, 60, rgb565(255, 255, 255), font6x9);
    hagl_flush(display);

    printf("RAN2 Software MCU©\n");
    printf("Starting...\n");

    Robot my_robot = buildRobot();
    execution_status status;

    printf("Status ready!\n");
    printf("Command: \n");

    while (1)
    {
        uint8_t uart_value;
        if (HAL_UART_Receive(&huart2, &uart_value, 1, 0) == HAL_OK){
            if(line_append(uart_value) == 0){
                status = executeGCODE(my_robot, line_buffer);
                memset(line_buffer, 0, 80);
                if(status == success){
                    printf("Command executed successfully\n");

                    hagl_fill_rectangle_xywh(display, 6*9, 60, 160-10+6*9, 9, rgb565(0, 0, 0));
                    hagl_put_text(display, L"Succeeded", 6*8, 60, rgb565(255, 255, 255), font6x9);
                    hagl_flush(display);
                }
                else{
                    printf("Command execution failed\n");

                    hagl_fill_rectangle_xywh(display, 6*9, 60, 160-10+6*9, 9, rgb565(0, 0, 0));
                    hagl_put_text(display, L"Failed", 6*8, 60, rgb565(255, 255, 255), font6x9);
                    hagl_flush(display);
                }

            }
            else if(uart_value != '\0' && line_length >= 0){
                printf("Command: %s\n", line_buffer);

                hagl_fill_rectangle_xywh(display, 6*9, 50, 160-10+6*9, 9, rgb565(0, 0, 0));

                convertCharArrayToWChar(line_buffer, line_buffer_display, LINE_MAX_LENGTH + 1);
                hagl_put_text(display, line_buffer_display, 6*9, 50, rgb565(255, 255, 255), font6x9);
                hagl_flush(display);
            }

            fflush(stdin);
            fflush(stdout);
        }
    }
}