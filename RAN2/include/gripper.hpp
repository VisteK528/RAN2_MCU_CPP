#ifndef RAN2_MCU_CPP_SERVO_HPP
#define RAN2_MCU_CPP_SERVO_HPP

#include "../../Core/Inc/adc.h"
#include "../../Core/Inc/tim.h"
#include "gpio.h"
#include "errors.hpp"
#include <cmath>

/*  Operation_status information
 *
 *  Module codes used in this module:
 *  - End effector (gripper)                                                            0x00
 *
 *  Operation result codes:
 *  Result                                                                              Code
 *  Operation ended successfully                                                        0x00
 *  Operation continue                                                                  0x01
 *
 ** */


class Gripper{
public:
    Gripper(GPIO_PIN servo_pwm_pin, GPIO_PIN feedback_pin, ADC_HandleTypeDef* adc, TIM_HandleTypeDef* tim);

    operation_status initGripper();
    operation_status enableGripper();
    operation_status disableGripper();
    operation_status setPosition(float position);
    float getCurrentPosition();
    operation_status getGripperStatus();
    float readCurrentPosition();
private:
    void rawSetPosition(float position);

    ADC_HandleTypeDef* adc;
    TIM_HandleTypeDef* tim;

    GPIO_PIN servo_pwm_pin;
    GPIO_PIN feedback_pin;

    uint16_t position_ms;
    float position = 0;
    float set_position = 0;
    float read_position = 0;

    operation_status current_status;
};

#endif //RAN2_MCU_CPP_SERVO_HPP
