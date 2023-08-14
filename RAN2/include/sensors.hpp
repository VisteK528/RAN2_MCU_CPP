#ifndef ROBOTARMNUMBER2CPP_SENSORS_HPP
#define ROBOTARMNUMBER2CPP_SENSORS_HPP

#include "gpio.h"
#include <iostream>

typedef enum{UP, DOWN} ENDSTOP_TYPE;

class Endstop{
public:
    Endstop(GPIO_PIN , ENDSTOP_TYPE type);
    Endstop() = default;
    ~Endstop() = default;
    bool checkSensor();
private:
    GPIO_TypeDef* signal_pin_port;
    uint16_t signal_pin;
    ENDSTOP_TYPE type;
};
#endif //ROBOTARMNUMBER2CPP_SENSORS_HPP
