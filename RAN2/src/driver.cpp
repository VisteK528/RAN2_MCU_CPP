#include "../include/driver.hpp"

volatile static drivers::speedRampData movement_data[6];

drivers::Driver::Driver(uint8_t joint_number, GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth,
                        float motor_resolution, uint16_t driver_resolution) {

    this->joint_number = joint_number;

    // Control pins
    this->step = step;
    this->direction = direction;
    this->enable = enable;

    // Variables associated with driver / motor driven by the driver
    this->gear_teeth = gear_teeth;
    this->motor_resolution = motor_resolution;
    this->driver_resolution = driver_resolution;
}

uint16_t drivers::Driver::getGearTeeth() const {
    return gear_teeth;
}

float drivers::Driver::getMotorResolution() const {
    return motor_resolution;
}

uint16_t drivers::Driver::getDriverResolution() const {
    return driver_resolution;
}

void drivers::Driver::setDirection(drivers::DIRECTION movement_direction) {
    current_direction = movement_direction;

    if(movement_direction == ANTICLOCKWISE){
        HAL_GPIO_WritePin(direction.gpio_port, direction.gpio_pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(direction.gpio_port, direction.gpio_pin, GPIO_PIN_RESET);
    }
}

drivers::DIRECTION drivers::Driver::getMotorDirection() const {
    return this->current_direction;
}

void drivers::Driver::startMovement(uint8_t joint, int mode, float max_speed_delay, float start_delay,
                                    int step_count_goal){
    movement_data[joint].step_count = 0;
    switch (mode) {
        case 0:
            movement_data[joint].step_delay = start_delay;
            movement_data[joint].min_delay = max_speed_delay;

            movement_data[joint].accel_count = 0;
            movement_data[joint].mode = mode;
            movement_data[joint].isMoving = true;
            break;
        case 1:
            movement_data[joint].run_state = 0;
            movement_data[joint].step_delay = start_delay;
            movement_data[joint].min_delay = max_speed_delay;
            movement_data[joint].step_count_goal = step_count_goal;

            movement_data[joint].accel_count = 0;
            movement_data[joint].decel_val = -(float)step_count_goal * 0.6f;
            movement_data[joint].decel_start = step_count_goal + movement_data[joint].decel_val;
            movement_data[joint].mode = mode;
            movement_data[joint].isMoving = true;
            break;
        case 2:
            movement_data[joint].step_delay = start_delay;
            movement_data[joint].mode = mode;
            movement_data[joint].isMoving = true;
            break;
        default:
            break;

    }

    if(joint >= 0 && joint < 6 && mode >= 0 && mode < 3){
        if(joint == 0){
            HAL_TIM_Base_Start_IT(&htim1);
        }
        else if(joint == 1){
            HAL_TIM_Base_Start_IT(&htim2);
        }
        else if(joint == 2){
            HAL_TIM_Base_Start_IT(&htim3);
        }
        else if(joint == 3){
            HAL_TIM_Base_Start_IT(&htim4);
        }
        else if(joint == 4){
            HAL_TIM_Base_Start_IT(&htim8);
        }
        else if(joint == 5){
            HAL_TIM_Base_Start_IT(&htim9);
        }
    }

    /*  CPU Frequency: 80MHz
     *  Prescaler: 799(800)       ->      CPU Frequency / Prescaler: 100KHz
     *
     *
     * */
}

void drivers::Driver::stopMovement(uint8_t joint) {
    movement_data[joint].isMoving = false;
}

bool drivers::Driver::checkMovement(uint8_t joint) {
    return movement_data[joint].isMoving;
}

void drivers::Driver::enableMotor() {
    HAL_GPIO_WritePin(enable.gpio_port, enable.gpio_pin, GPIO_PIN_RESET);
    this->motor_enabled = true;
}

void drivers::Driver::disableMotor() {
    HAL_GPIO_WritePin(enable.gpio_port, enable.gpio_pin, GPIO_PIN_SET);
    this->motor_enabled = false;
}

bool drivers::Driver::isMotorEnabled() {
    return motor_enabled;
}

static void accelerateToVelocity(GPIO_PIN step_pin, uint8_t joint, TIM_HandleTypeDef *htim){
    HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
    movement_data[joint].accel_count++;
    movement_data[joint].step_count++;

    movement_data[joint].step_delay = movement_data[joint].step_delay - ((2.f * movement_data[joint].step_delay) / (4.f * (float)movement_data[joint].accel_count + 1));

    htim->Instance->ARR = movement_data[joint].step_delay;

    if(movement_data[joint].step_delay <= movement_data[joint].min_delay){
        HAL_TIM_Base_Stop_IT(htim);
        movement_data[joint].isMoving = false;
    }
}

void DriverHandleCallback(TIM_HandleTypeDef* htim){
    uint8_t joint = 255;
    if (htim->Instance == TIM10) {
        joint = 0;
    } else if(htim->Instance == TIM4){
        joint = 1;
    }
    else if(htim->Instance == TIM3){
        joint = 2;
    }
    else if(htim->Instance == TIM5){
        joint = 3;
    }
    else if(htim->Instance == TIM9){
        joint = 4;
    }
    else if(htim->Instance == TIM11){
        joint = 5;
    }

    if(joint < 6){
        GPIO_PIN step_pin;
        if(joint == 0){
            step_pin.gpio_port = J1_STEP_GPIO_Port;
            step_pin.gpio_pin = J1_STEP_Pin;
        }
        else if(joint == 1){
            step_pin.gpio_port = J2_STEP_GPIO_Port;
            step_pin.gpio_pin = J2_STEP_Pin;
        }
        else if(joint == 2){
            step_pin.gpio_port = J3_STEP_GPIO_Port;
            step_pin.gpio_pin = J3_STEP_Pin;
        }
        else if(joint == 3){
            step_pin.gpio_port = J4_STEP_GPIO_Port;
            step_pin.gpio_pin = J4_STEP_Pin;
        }
        else if(joint == 4){
            step_pin.gpio_port = J5_STEP_GPIO_Port;
            step_pin.gpio_pin = J5_STEP_Pin;
        }
        else if(joint == 5){
            step_pin.gpio_port = J6_STEP_GPIO_Port;
            step_pin.gpio_pin = J6_STEP_Pin;
        }

        if(!movement_data[joint].isMoving){
            HAL_TIM_Base_Stop_IT(htim);
        }

        switch (movement_data[joint].mode) {
            case 0:
                accelerateToVelocity(step_pin, joint, htim);
                break;
            case 1:
                switch(movement_data[joint].run_state) {
                    case 0:
                        // Accel
                        HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
                        movement_data[joint].accel_count++;
                        movement_data[joint].step_count++;

                        movement_data[joint].step_delay = movement_data[joint].step_delay - ((2.f * movement_data[joint].step_delay) / (4.f * (float)movement_data[joint].accel_count + 1));

                        htim->Instance->ARR = movement_data[joint].step_delay;

                        if(movement_data[joint].step_delay <= movement_data[joint].min_delay){
                            movement_data[joint].run_state = 1;
                        }

                        if(movement_data[joint].step_count >= movement_data[joint].decel_start){
                            movement_data[joint].accel_count = movement_data[joint].decel_val;
                            movement_data[joint].run_state = 2;
                        }
                        break;
                    case 1:
                        // Run

                        HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
                        movement_data[joint].step_count++;
                        movement_data[joint].step_delay = movement_data[joint].min_delay;

                        if(movement_data[joint].step_count >= movement_data[joint].decel_start){
                            movement_data[joint].accel_count = movement_data[joint].decel_val;
                            movement_data[joint].run_state = 2;
                        }
                        break;
                    case 2:
                        // Decel
                        HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
                        movement_data[joint].accel_count++;
                        movement_data[joint].step_count++;

                        htim->Instance->ARR = movement_data[joint].step_delay;

                        if(movement_data[joint].step_count > 0){
                            movement_data[joint].step_delay = movement_data[joint].step_delay - ((2.f * movement_data[joint].step_delay) / (4.f * (float)movement_data[joint].accel_count + 1));
                        }

                        if(movement_data[joint].step_count >= movement_data[joint].step_count_goal){
                            movement_data[joint].run_state = 3;
                        }
                        break;
                    case 3:
                        // Stop
                        HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
                        HAL_TIM_Base_Stop_IT(htim);
                        movement_data[joint].isMoving = false;
                        break;
                }
                break;
            case 2:
                HAL_GPIO_TogglePin(step_pin.gpio_port, step_pin.gpio_pin);
                if(!movement_data[joint].isMoving){
                    HAL_TIM_Base_Stop_IT(htim);
                }
                break;
            default:
                break;

        }
    }
}
