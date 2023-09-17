#ifndef RAN2_MCU_CPP_DRIVER_HPP
#define RAN2_MCU_CPP_DRIVER_HPP

#include "gpio.h"
#include "tim.h"
#include "utilities.hpp"

namespace drivers{
    typedef enum{CLOCKWISE, ANTICLOCKWISE} DIRECTION;


    #define ACCEL           (unsigned char) 0
    #define MOVE_STEPS      (unsigned char) 1
    #define RUN_CONST       (unsigned char) 2

    typedef struct {
        unsigned char mode : 3;
        //! What part of the speed ramp we are in.
        unsigned char run_state : 3;
        //! Direction stepper motor should move.
        unsigned char dir : 1;
        //! Peroid of next timer delay. At start this value set the accelration rate.
        float step_delay;
        //! What step_pos to start decelaration
        int decel_start;
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

    class Driver {
        private:
            GPIO_PIN direction;
            GPIO_PIN enable;
            GPIO_PIN step;
        
            float motor_resolution;
            uint16_t driver_resolution;
            uint16_t gear_teeth;
        
            bool motor_enabled = true;
        
            DIRECTION current_direction=CLOCKWISE;
        public:
            uint8_t joint_number;
        
            Driver(uint8_t joint_number, GPIO_PIN step, GPIO_PIN direction, GPIO_PIN enable, uint16_t gear_teeth, float motor_resolution, uint16_t driver_resolution=8);
            Driver() = default;

            /// Returns number of teeth on the pulley mounted on the motor shaft
            uint16_t getGearTeeth() const;

            /// Returns raw resolution of the motor driven by the driver (typically 1.8 deg or 0.9 deg)
            float getMotorResolution() const;

            /// Returns driver resolution, meaning number of microsteps for "typical" motor step
            uint16_t getDriverResolution() const;

            /// Returns current motor movement direction, either CLOCKWISE or ANTICLOCKWISE
            DIRECTION getMotorDirection() const;

            /// Sets the direction of the next movement
            void setDirection(DIRECTION movement_direction);

            // Movement methods

            /// @brief Initializes the movement through low-level layer of control, by setting the movement parameters
            /// @param joint - Joint number (motor number) which will be set in motion
            /// @param mode - Movement mode to be selected. Either acceleration movement, run with constant speed
            /// movement or movement by certain number of steps (microsteps)
            /// @param max_speed_delay - Minimal delay between two pulses giving the maximum available motor speed.
            /// Value given in microseconds
            /// @param start_delay - Starting delay given in microseconds between first two pulses
            /// @param step_count_goal - Number of steps (pulses) to be done / (generated) before the movement finishes
            void startMovement(uint8_t joint, int mode, float max_speed_delay, float start_delay, int step_count_goal);

            /// Checks if the motor is currently in motion and returns the result
            bool checkMovement(uint8_t joint);

            /// Immediately stops the motion of the given joint (motor associated with the joint) regardless of
            /// the current motion state
            void stopMovement(uint8_t joint);

            // Motor control functions
            /// Disables the motor by setting the ENABLE PIN to high
            void disableMotor();
            /// Enables the motor by setting the ENABLE PIN to low
            void enableMotor();

            /// Checks if the motor is enabled and returns the result
            bool isMotorEnabled();
        };
};

void DriverHandleCallback(TIM_HandleTypeDef* htim);

#endif //RAN2_MCU_CPP_DRIVER_HPP
