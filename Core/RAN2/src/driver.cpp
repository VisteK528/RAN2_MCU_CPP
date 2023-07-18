#include "../include/driver.hpp"

Driver::Driver(GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution,
                        uint16_t driver_resolution){
    this->step = step;
    this->direction = direction;
    this->enable = enable;
    this->gear_teeth = gear_teeth;
    this->motor_resolution = motor_resolution;
    this->driver_resolution = driver_resolution;
}

void Driver::moveDelay(float delay) {
    HAL_GPIO_WritePin(step.gpio_port, step.gpio_pin, GPIO_PIN_RESET);
    HAL_Delay(delay*1000);
    HAL_GPIO_WritePin(step.gpio_port, step.gpio_pin, GPIO_PIN_SET);
}

uint16_t Driver::getGearTeeth() const {
    return this->gear_teeth;
}

float Driver::getMotorResolution() const {
    return motor_resolution;
}

uint16_t Driver::getDriverResolution() const {
    return driver_resolution;
}

float Driver::getMaxSpeed() const {
    return max_speed;
}

void Driver::setMaxSpeed(float speed) {
    this->max_speed = speed;
}

void Driver::setDirection(DIRECTION movement_direction) {
    current_direction = movement_direction;

    if(movement_direction == ANTICLOCKWISE){
        HAL_GPIO_WritePin(direction.gpio_port, direction.gpio_pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(direction.gpio_port, direction.gpio_pin, GPIO_PIN_RESET);
    }
}

DM556::DM556(GPIO_PIN pulse, GPIO_PIN direction, GPIO_PIN enable, uint16_t driver_resolution,
                      float motor_resolution, uint16_t gear_teeth): Driver(pulse, direction, enable, gear_teeth,
                                                                           motor_resolution, driver_resolution) {
    this->max_speed = 2000;
    this->max_acceleration = 0.05;
}



