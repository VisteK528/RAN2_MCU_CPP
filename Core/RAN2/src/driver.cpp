#include "../include/driver.hpp"

volatile static speedRampData movement_data[6];

drivers::Driver::Driver(uint8_t joint_number, GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth,
                        float motor_resolution, uint16_t driver_resolution) {
    this->joint_number = joint_number;

    this->step = step;
    this->direction = direction;
    this->enable = enable;
    this->gear_teeth = gear_teeth;
    this->motor_resolution = motor_resolution;
    this->driver_resolution = driver_resolution;
}

void drivers::Driver::moveDelay(float delay) {
    HAL_GPIO_WritePin(step.gpio_port, step.gpio_pin, GPIO_PIN_RESET);
    HAL_Delay(delay*1000);
    HAL_GPIO_WritePin(step.gpio_port, step.gpio_pin, GPIO_PIN_SET);
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

float drivers::Driver::getMaxSpeed() const {
    return max_speed;
}

void drivers::Driver::setMaxSpeed(float speed) {
    this->max_speed = speed;
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

drivers::DM556::DM556(uint8_t joint_number, GPIO_PIN pulse, GPIO_PIN direction, GPIO_PIN enable,
                      uint16_t driver_resolution, float motor_resolution, uint16_t gear_teeth): Driver(
                              joint_number, pulse, direction, enable, gear_teeth, motor_resolution, driver_resolution) {
    this->max_speed = 2000;
    this->max_acceleration = 0.05;
}

void drivers::Driver::initializeMovement(uint8_t joint_number, int mode, float max_speed_delay, float d0,
                                         int iterations) {
    movement_data[joint_number].step_count = 0;
    switch (mode) {
        case 0:
            movement_data[joint_number].step_delay = d0;
            movement_data[joint_number].min_delay = max_speed_delay;

            movement_data[joint_number].accel_count = 0;
            movement_data[joint_number].mode = mode;
            movement_data[joint_number].isMoving = true;
            HAL_TIM_Base_Start_IT(&htim10);
            break;
        case 1:
            movement_data[joint_number].run_state = 0;
            movement_data[joint_number].step_delay = d0;
            movement_data[joint_number].min_delay = max_speed_delay;
            movement_data[joint_number].step_count_goal = iterations;

            movement_data[joint_number].accel_count = 0;
            movement_data[joint_number].decel_val = -(float)iterations*0.6f;
            movement_data[joint_number].decel_start = iterations+movement_data[joint_number].decel_val;
            movement_data[joint_number].mode = mode;
            movement_data[joint_number].isMoving = true;
            HAL_TIM_Base_Start_IT(&htim10);
            break;
        case 2:
            movement_data[joint_number].step_delay = d0;
            movement_data[joint_number].mode = mode;
            movement_data[joint_number].isMoving = true;
            HAL_TIM_Base_Start_IT(&htim10);
            break;
        default:
            break;

    }

    /*  CPU Frequency: 80MHz
     *  Prescaler: 799(800)       ->      CPU Frequency / Prescaler: 100KHz
     *
     *
     * */
}

int drivers::Driver::getCount(uint8_t joint_number) const {
    return movement_data[joint_number].step_count;
}

void drivers::Driver::stopMovement(uint8_t joint_number) {
    movement_data[joint_number].isMoving = false;
}

bool drivers::Driver::getMovement(uint8_t joint_number) {
    return movement_data[joint_number].isMoving;
}

static void accelerateToVelocity(uint8_t joint_number, TIM_HandleTypeDef *htim){
    HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);
    movement_data[joint_number].accel_count++;
    movement_data[joint_number].step_count++;

    movement_data[joint_number].step_delay = movement_data[joint_number].step_delay - ((2.f * movement_data[joint_number].step_delay) / (4.f * (float)movement_data[joint_number].accel_count + 1));

    htim->Instance->ARR = movement_data[joint_number].step_delay;

    if(movement_data[joint_number].step_delay <= movement_data[joint_number].min_delay){
        HAL_TIM_Base_Stop_IT(htim);
        movement_data[joint_number].isMoving = false;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t joint_number = 255;
    if (htim->Instance == TIM10) {
        joint_number = 0;
    }

    if(joint_number < 6){
        switch (movement_data[joint_number].mode) {
            case 0:
                accelerateToVelocity(joint_number, htim);
                break;
            case 1:
                switch(movement_data[joint_number].run_state) {
                    case 0:
                        // Accel
                        HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);
                        movement_data[joint_number].accel_count++;
                        movement_data[joint_number].step_count++;

                        movement_data[joint_number].step_delay = movement_data[joint_number].step_delay - ((2.f * movement_data[joint_number].step_delay) / (4.f * (float)movement_data[joint_number].accel_count + 1));

                        htim->Instance->ARR = movement_data[joint_number].step_delay;

                        if(movement_data[joint_number].step_delay <= movement_data[joint_number].min_delay){
                            movement_data[joint_number].run_state = 1;
                        }

                        if(movement_data[joint_number].step_count >= movement_data[joint_number].decel_start){
                            movement_data[joint_number].accel_count = movement_data[joint_number].decel_val;
                            movement_data[joint_number].run_state = 2;
                        }
                        break;
                    case 1:
                        // Run

                        HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);;
                        movement_data[joint_number].step_count++;
                        movement_data[joint_number].step_delay = movement_data[joint_number].min_delay;

                        if(movement_data[joint_number].step_count >= movement_data[joint_number].decel_start){
                            movement_data[joint_number].accel_count = movement_data[joint_number].decel_val;
                            movement_data[joint_number].run_state = 2;
                        }
                        break;
                    case 2:
                        // Decel
                        HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);
                        movement_data[joint_number].accel_count++;
                        movement_data[joint_number].step_count++;

                        htim->Instance->ARR = movement_data[joint_number].step_delay;

                        if(movement_data[joint_number].step_count > 0){
                            movement_data[joint_number].step_delay = movement_data[joint_number].step_delay - ((2.f * movement_data[joint_number].step_delay) / (4.f * (float)movement_data[joint_number].accel_count + 1));
                        }

                        if(movement_data[joint_number].step_count >= movement_data[joint_number].step_count_goal){
                            movement_data[joint_number].run_state = 3;
                        }
                        break;
                    case 3:
                        // Stop
                        HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);
                        HAL_TIM_Base_Stop_IT(htim);
                        movement_data[joint_number].isMoving = false;
                        break;
                }
                break;
            case 2:
                HAL_GPIO_TogglePin(J1_STEP_GPIO_Port, J1_STEP_Pin);
                if(!movement_data[joint_number].isMoving){
                    HAL_TIM_Base_Stop_IT(htim);
                }
                break;
            default:
                break;

        }
    }
}

