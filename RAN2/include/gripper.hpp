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
 *  Gripper feedback signal not available (voltage too low)                             0x02
 *  Gripper feedback signal not working properly    (voltage too high)                  0x03
 *
 *  Gripper PWM signal cannot be enabled                                                0x04
 *
 ** */


namespace effector{

    #define GRIPPER_LOW             0.69f
    #define GRIPPER_HIGH            2.32f
    #define DEFAULT_SAMPLES         200

    class Gripper{
        public:
            Gripper(GPIO_PIN servo_pwm_pin, GPIO_PIN feedback_pin, ADC_HandleTypeDef* adc, TIM_HandleTypeDef* tim);

            operation_status initGripper();
            operation_status checkGripper();
            operation_status enableGripper();
            operation_status disableGripper();
            operation_status setPosition(float position);
            float getCurrentPosition() const;
            operation_status getGripperStatus() const;
            float readCurrentPosition();
            bool smartClose();
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
};

#endif //RAN2_MCU_CPP_SERVO_HPP
