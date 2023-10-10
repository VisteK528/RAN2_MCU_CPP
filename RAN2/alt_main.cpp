#include "alt_main.hpp"
#include "include/robot_build.hpp"
#include <cstring>
#include <string>
#include <iomanip>
#include "spi.h"
#include "usbd_cdc_if.h"

// USB Transmission variables
uint8_t RAN3_USB_Receive_Buffer[2048];
uint8_t RAN3_USB_Receive_Flag = 0;
uint32_t RAN3_USB_Receive_Length = 0;

bool start = false;

operation_status robot_operation_status;
operation_status background_robot_operation_status;

Robot my_robot;
coordinates coords[6] = {};
uint16_t counter = 0;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {

    }
}

int alt_main() {
    //HAL_TIM_Base_Start_IT(&htim1);

    my_robot = buildRobot();
    robot_operation_status = my_robot.systemsCheck(true);
    start = true;

    HAL_StatusTypeDef status = HAL_BUSY;
    std::string response;

    while (1) {

        if(RAN3_USB_Receive_Flag == 1){
            std::vector<std::string> commands = {};
            commands = splitCommandFile(RAN3_USB_Receive_Buffer, RAN3_USB_Receive_Length);
            for(auto& command: commands){
                HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

                // Get ready status
                robot_operation_status.result = in_progress;

                // Execute command
                response.clear();
                robot_operation_status = executeGCODE(my_robot, response, command.c_str());

                // Add command result
                if(robot_operation_status.result == success) {
                    response.insert(0, "0\t");
                } else {
                    response.insert(0, "1\t");
                }
                response += "\r\n";

                // Send response
                CDC_Transmit_FS((uint8_t*)response.c_str(), response.length());

                // Getting coordinates of End Effector from robot and then printing them on display, to be optimised
                my_robot.getRobotArmCoordinates(coords);
            }
            fflush(stdin);
            fflush(stdout);
            RAN3_USB_Receive_Flag = 0;
        }

    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim != &htim1){
        DriverHandleCallback(htim);
    }
    else if(start && !my_robot.getMovement()){
        if(my_robot.systemsCheck(false).result == success){
            background_robot_operation_status = my_robot.updateEncoders();
            if(background_robot_operation_status.result == failure){
            }
        }
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // GPIO_PIN = 4 / PC_2 GPIO EXT2
    if(GPIO_Pin == 4){
        my_robot.safeguardTrigger();
        counter++;
    }
}