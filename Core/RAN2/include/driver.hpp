#ifndef ROBOTARMNUMBER2CPP_DRIVER_HPP
#define ROBOTARMNUMBER2CPP_DRIVER_HPP

#include "../../Inc/gpio.h"

namespace drivers{
    typedef enum{CLOCKWISE, ANTICLOCKWISE} DIRECTION;

    class Driver {
    protected:
        uint8_t direction;
        uint8_t enable;
        uint8_t step;

        float motor_resolution;
        uint16_t driver_resolution;
        uint16_t gear_teeth;

        float max_speed;
        float max_acceleration;
    public:
        Driver(uint8_t step, uint8_t direction, uint8_t enable, uint16_t gear_teeth, float motor_resolution, uint16_t driver_resolution=8);
        Driver() = default;

        /**
         * \brief Function takes delay value in ms and generates impulse on step PIN with given interval
         * */
        void moveDelay(float delay);
        uint16_t getGearTeeth() const;
        float getMotorResolution() const;
        uint16_t getDriverResolution() const;
        float getMaxSpeed() const;
        void setMaxSpeed(float speed);
        void setDirection(DIRECTION direction);

    };

    class TMC2209: public Driver{
    public:
        TMC2209(uint8_t step, uint8_t direction, uint8_t enable, uint16_t gear_teeth, float motor_resolution,
                uint16_t driver_resolution=8): Driver(step, direction, enable, gear_teeth, motor_resolution, driver_resolution){};
        TMC2209() = default;
    };

    class DM556: public Driver{
    public:
        DM556(uint8_t pulse, uint8_t direction, uint8_t enable, uint16_t driver_resolution, float motor_resolution,
              uint16_t gear_teeth);
        DM556() = default;
    };
};



#endif //ROBOTARMNUMBER2CPP_DRIVER_HPP
