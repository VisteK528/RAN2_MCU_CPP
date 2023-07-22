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

    //Robot my_robot = buildRobot();
    GPIO_PIN waist_step, waist_dir, waist_en, waist_endstop_pin;
    waist_step.gpio_port = J1_STEP_GPIO_Port;
    waist_step.gpio_pin = J1_STEP_Pin;
    waist_dir.gpio_port = J1_DIR_GPIO_Port;
    waist_dir.gpio_pin = J1_DIR_Pin;
    waist_en.gpio_port = J1_EN_GPIO_Port;
    waist_en.gpio_pin = J1_EN_Pin;

    waist_endstop_pin.gpio_port = J1_ENDSTOP_GPIO_Port;
    waist_endstop_pin.gpio_pin = J1_ENDSTOP_Pin;

    uint8_t joint_number = 0;

    std::unique_ptr<Driver> waist_driver = std::make_unique<TMC2209>(joint_number, waist_step, waist_dir, waist_en, 20, 0.9f, 8);
    std::shared_ptr<Endstop> waist_endstop = std::make_shared<Endstop>(waist_endstop_pin, ENDSTOP_TYPE::UP);

    std::unique_ptr<Joint> waist_joint = std::make_unique<Joint>(joint_number, waist_driver, waist_endstop, 125,
                                                                 drivers::DIRECTION::ANTICLOCKWISE);
    waist_joint->setMaxVelocity(3.2);
    waist_joint->setMaxAcceleration(1.5);

    waist_joint->homeJoint();
    //waist_joint->move2Pos(90, true);

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
                            waist_joint->move2Pos(value, true);
                            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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