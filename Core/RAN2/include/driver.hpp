#ifndef RAN2_MCU_CPP_DRIVER_HPP
#define RAN2_MCU_CPP_DRIVER_HPP

#include "../../Inc/gpio.h"
#include "../../Inc/tim.h"
#include "utilities.hpp"

#define ACCEL           (unsigned char) 0
#define MOVE_STEPS      (unsigned char) 1
#define RUN_CONST       (unsigned char) 2

namespace drivers{
    typedef enum{CLOCKWISE, ANTICLOCKWISE} DIRECTION;

    class Driver {
    protected:
        GPIO_PIN enable;
        GPIO_PIN step;

        float motor_resolution;
        uint16_t driver_resolution;
        uint16_t gear_teeth;

        float max_speed;
        float max_acceleration;

        DIRECTION current_direction=CLOCKWISE;
    public:
        GPIO_PIN direction;
        uint8_t joint_number;

        Driver(uint8_t joint_number, GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution, uint16_t driver_resolution=8);
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
        void setDirection(DIRECTION movement_direction);
        void initializeMovement(uint8_t joint_number, int mode, float max_speed_delay, float d0, int iterations);
        bool getMovement(uint8_t joint_number);
        void stopMovement(uint8_t joint_number);
        void disableStepper();
        void enableStepper();
        int getCount(uint8_t joint_number) const;

    };

    class TMC2209: public Driver{
    public:
        TMC2209(uint8_t joint_number, GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution,
                uint16_t driver_resolution=8): Driver(joint_number, step, direction, enable, gear_teeth, motor_resolution, driver_resolution){};
        TMC2209() = default;
    };

    class DM556: public Driver{
    public:
        DM556(uint8_t joint_number, GPIO_PIN pulse, GPIO_PIN direction, GPIO_PIN enable, uint16_t driver_resolution, float motor_resolution,
              uint16_t gear_teeth);
        DM556() = default;
    };
};

typedef struct {
    unsigned char mode : 3;
    //! What part of the speed ramp we are in.
    unsigned char run_state : 3;
    //! Direction stepper motor should move.
    unsigned char dir : 1;
    //! Peroid of next timer delay. At start this value set the accelration rate.
    float step_delay;
    //! What step_pos to start decelaration
    unsigned int decel_start;
    //! Sets deceleration rate.
    signed int decel_val;
    //! Minimum time delay (max speed)
    float min_delay;
    //! Counter used when accelerateing/decelerateing to calculate step_delay.
    int accel_count;

    int step_count;
    int step_count_goal;

    bool isMoving;
} speedRampData;

#endif //RAN2_MCU_CPP_DRIVER_HPP
