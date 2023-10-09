#ifndef RAN2_MCU_CPP_SERVO_HPP
#define RAN2_MCU_CPP_SERVO_HPP

#include "../../Core/Inc/adc.h"
#include "../../Core/Inc/tim.h"
#include "gpio.h"
#include "errors.hpp"
#include <cmath>
#include "utilities.hpp"

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

    #define GRIPPER_LOW                             0.69f
    #define GRIPPER_HIGH                            2.32f

    #define GRIPPER_FEEDBACK_DISCONNECTED_LEVEL     0.49f
    #define GRIPPER_FEEDBACK_ERROR_LEVEL            2.82f
    #define SMART_GRIPPER_CLOSED_DIFFERENCE         0.15f

    #define DEFAULT_SAMPLES                         200

    class Gripper{
        public:
            Gripper(GPIO_PIN servo_pwm_pin, GPIO_PIN feedback_pin, ADC_HandleTypeDef* adc, TIM_HandleTypeDef* tim);

            /// @brief Enables gripper, sets its position to 0%, initializes variables. Checks availability of systems
            /// used by the gripper (PWM Generator and ADC)
            /// @returns Status of the operation
            operation_status initGripper();

            /// @brief Checks availability of systems used by the gripper (PWM Generator and ADC)
            /// @returns Status of the operation with possible errors
            operation_status checkGripper();

            /// @brief Enables gripper (activates PWM signal)
            /// @returns Status of the operation with possible errors
            operation_status enableGripper();

            /// @brief Disables gripper (disables PWM signal)
            /// @returns Status of the operation with possible errors
            operation_status disableGripper();

            /// @brief Moves the gripper instantly to set position (from 0% to 100% close)
            /// @returns Status of the operation with possible errors
            operation_status setPosition(float position);

            /// Returns currently set position
            float getCurrentPosition() const;

            /// Returns current gripper status
            operation_status getGripperStatus() const;

            /// @brief Reads value from the feedback pin using the ADC and maps read voltage to the close percentage
            float readCurrentPosition();

            /// @brief Method used in order to grab objects of unknown dimensions. Gripper slowly closes,
            /// constantly monitoring feedback signal and stops when the difference between estimated voltage level
            /// and measured voltage level is greater than certain value.
            /// @attention Experimental function
            /// @returns Status of gripper where true means gripper is closed on target
            bool smartClose();
        private:
            void rawSetPosition(float position);

            ADC_HandleTypeDef* adc;
            TIM_HandleTypeDef* tim;

            GPIO_PIN servo_pwm_pin;
            GPIO_PIN feedback_pin;

            uint16_t position_ms;
            float position = 0;
            float read_position = 0;

            operation_status current_status;
    };
};

#endif //RAN2_MCU_CPP_SERVO_HPP
