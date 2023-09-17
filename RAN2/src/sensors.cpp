#include "../include/sensors.hpp"

Endstop::Endstop(GPIO_PIN pin, ENDSTOP_TYPE type) {
    this->signal_pin = pin;
    this->type = type;
};

bool Endstop::checkEndstop() {
    GPIO_PinState state = HAL_GPIO_ReadPin(signal_pin.gpio_port, signal_pin.gpio_pin);

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
