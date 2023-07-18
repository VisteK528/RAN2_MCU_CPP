#ifndef RAN2_MCU_CPP_DRIVER_HPP
#define RAN2_MCU_CPP_DRIVER_HPP

#include "../../Inc/gpio.h"

typedef enum{CLOCKWISE, ANTICLOCKWISE} DIRECTION;

class Driver {
public:
    Driver(GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution, uint16_t driver_resolution=8);

    void moveDelay(float delay);
    [[nodiscard]] uint16_t getGearTeeth() const;
    float getMotorResolution() const;
    uint16_t getDriverResolution() const;
    float getMaxSpeed() const;
    void setMaxSpeed(float speed);
    void setDirection(DIRECTION movement_direction);

protected:
    GPIO_PIN direction;
    GPIO_PIN enable;
    GPIO_PIN step;

    float motor_resolution;
    uint16_t driver_resolution;
    uint16_t gear_teeth=25;

    float max_speed;
    float max_acceleration;

    DIRECTION current_direction=CLOCKWISE;

};

class TMC2209: public Driver{
public:
    TMC2209(GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution,
            uint16_t driver_resolution=8): Driver(step, direction, enable, gear_teeth, motor_resolution, driver_resolution){};
    TMC2209() = default;
};

class DM556: public Driver{
public:
    DM556(GPIO_PIN pulse, GPIO_PIN direction, GPIO_PIN enable, uint16_t driver_resolution, float motor_resolution,
          uint16_t gear_teeth);
    DM556() = default;
};



#endif //RAN2_MCU_CPP_DRIVER_HPP
