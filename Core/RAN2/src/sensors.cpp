#include "../include/sensors.hpp"

Endstop::Endstop(GPIO_PIN pin, ENDSTOP_TYPE type) {
    this->signal_pin = pin.gpio_pin;
    this->signal_pin_port = pin.gpio_port;
    this->type = type;
};

bool Endstop::checkSensor() {
    bool state = HAL_GPIO_ReadPin(signal_pin_port, signal_pin);

    if(type == ENDSTOP_TYPE::UP){
        if(state){
            return false;
        }
        return true;
    }

    if(HAL_GPIO_ReadPin(signal_pin_port, signal_pin)){
        return true;
    }
    return false;
}