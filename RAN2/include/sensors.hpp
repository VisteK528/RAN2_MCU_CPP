#ifndef ROBOTARMNUMBER2CPP_SENSORS_HPP
#define ROBOTARMNUMBER2CPP_SENSORS_HPP

#include "gpio.h"
#include <iostream>
#include "utilities.hpp"

typedef enum{UP, DOWN} ENDSTOP_TYPE;

class Endstop{
public:
    Endstop(GPIO_PIN , ENDSTOP_TYPE type);
    Endstop() = default;
    ~Endstop() = default;

    /// Checks if the endstop was triggered and returns result
    bool checkEndstop();
private:
    GPIO_PIN signal_pin;
    ENDSTOP_TYPE type;
};
#endif //ROBOTARMNUMBER2CPP_SENSORS_HPP
