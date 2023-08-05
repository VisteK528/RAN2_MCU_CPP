#ifndef RAN2_MCU_CPP_MOVEMENT_HPP
#define RAN2_MCU_CPP_MOVEMENT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "utilities.hpp"

class Movement{
public:
    Movement(float motor_step, uint16_t driver_microstep, uint16_t motor_shaft_gear_teeth, uint16_t joint_gear_teeth);
    Movement() = default;
    float motorVel(float phase_time);
    float phaseTime(float joint_ang_vel);
    float jointVelFromMotorVel(float motor_ang_vel);
    float motorVelFromJointVel(float joint_ang_vel);
    float getGearSpeedRatio() const;
    std::vector<float> calculateSteps(unsigned int steps, float max_speed, float acceleration);
    std::vector<float> accelerateToVelocity(float velocity, float acceleration = 0.5);

    float getMinDelay(float max_speed);
    float getStartDelay(float acceleration) const;

private:
    float motor_step;
    uint16_t driver_microstep;
    float one_pulse_step;

    uint16_t motor_shaft_gear_teeth;
    uint16_t joint_gear_teeth;

    float speed_gear_ratio;
    float torque_gear_ratio;


};

#endif //RAN2_MCU_CPP_MOVEMENT_HPP
