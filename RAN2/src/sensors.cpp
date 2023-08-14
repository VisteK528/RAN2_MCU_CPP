#include "../include/sensors.hpp"

Endstop::Endstop(GPIO_PIN pin, ENDSTOP_TYPE type) {
    this->signal_pin = pin.gpio_pin;
    this->signal_pin_port = pin.gpio_port;
    this->type = type;
};

bool Endstop::checkSensor() {
    GPIO_PinState state = HAL_GPIO_ReadPin(signal_pin_port, signal_pin);

    if(type == ENDSTOP_TYPE::UP){
        if(state == GPIO_PIN_SET){
            return false;
        }
        return true;
    }

    else{
    	if(state == GPIO_PIN_RESET){
    		return false;
    	}
    	return true;
    }
}
