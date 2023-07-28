#include "../include/alt_main.hpp"

#include "../include/robot.hpp"
#include "../include/read_gcode.h"
#include <string>
#include <cstring>

#define LINE_MAX_LENGTH	80

static char line_buffer[LINE_MAX_LENGTH + 1];
static uint32_t line_length;

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
        line_buffer[line_length++] = value;
    }
    return 1;
}

int alt_main(){
    printf("RAN2 Software MCU©\n");
    printf("Starting...\n");

    Robot my_robot = buildRobot();
    my_robot.home();
    my_robot.move2Default();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);


    while (1)
    {
        uint8_t uart_value;
        if (HAL_UART_Receive(&huart2, &uart_value, 1, 0) == HAL_OK){
            if(uart_value != '\0'){
                printf("Got: %c\n", uart_value);
            }

            if(line_append(uart_value) == 0){
                printf("Command: %s\n", line_buffer);

                // Check which command to choose
                char letter;
                float value;
                uint16_t counter = 0;
                while(parseMessage(&letter, &value, line_buffer, &counter) == 1){
                    switch(letter){
                        case 'N':
                            printf("Done\n");
                            break;
                        case 'G':
                            switch ((int)value) {
                                case 28:                // Home joints
                                    printf("%c  %f\n", letter, value);
                                    break;

                            }
                            break;
                        case 'M':
                            switch ((int)value) {
                                case 30:                // End of the programme
                                    break;
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
            fflush(stdin);
            fflush(stdout);
        }
    }
}