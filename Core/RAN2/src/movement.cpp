#include <algorithm>
#include "../include/movement.hpp"

Movement::Movement(float motor_step, uint16_t driver_microstep, uint16_t motor_shaft_gear_teeth,
                   uint16_t joint_gear_teeth) {
    this->motor_step = motor_step;
    this->driver_microstep = driver_microstep;
    this->motor_shaft_gear_teeth = motor_shaft_gear_teeth;
    this->joint_gear_teeth = joint_gear_teeth;

    this->one_pulse_step = deg2Rad(motor_step/(float)driver_microstep);
    this->speed_gear_ratio = (float)motor_shaft_gear_teeth/(float)joint_gear_teeth;
    this->torque_gear_ratio = 1/speed_gear_ratio;
}

float Movement::motorVel(float phase_time) {
    float term = (360.f/motor_step) * (phase_time * (float)driver_microstep);
    float angular_velocity = (2*(float)M_PI)/term;
    return angular_velocity;
}

float Movement::phaseTime(float joint_ang_vel) {
    float motor_ang_vel = motorVelFromJointVel(joint_ang_vel);
    return one_pulse_step/motor_ang_vel;
}

float Movement::jointVelFromMotorVel(float motor_ang_vel) {
    return motor_ang_vel * speed_gear_ratio;
}

float Movement::motorVelFromJointVel(float joint_ang_vel) {
    return joint_ang_vel / speed_gear_ratio;
}

float Movement::getGearSpeedRatio() const {
    return speed_gear_ratio;
}

float Movement::getMinDelay(float max_speed) {
    return seconds2Microseconds(one_pulse_step/motorVelFromJointVel(max_speed));
}

float Movement::getStartDelay(float acceleration) const {
    float angle = one_pulse_step;
    const float constant = 0.13568198123907316536355537605674f;

    return 2000000.f * std::sqrt(2.f * angle / acceleration) * constant;
}

/*
std::vector<float> Movement::calculateSteps(unsigned int steps, float max_speed, float acceleration) {
    float motor_max_speed = motorVelFromJointVel(max_speed);
    float motor_max_speed_delay = seconds2Microseconds(one_pulse_step/motor_max_speed);
    std::vector<float> delays;
    float angle = one_pulse_step;
    const float constant = 0.13568198123907316536355537605674;

    float c0 = 2000000.f * std::sqrt(2.f * angle / acceleration) * constant;

    int iterations = 0;
    bool even = false;
    if(steps % 2 == 0){
        iterations = (int)steps/2;
        even = true;
    }
    else{
        iterations = (int)steps/2 + 1;
    }

    delays.reserve(iterations);

    float delay = 0;
    for(int i = 0; i < iterations; i++){
        delay = c0;
        if(i > 0){
            delay = delays[i - 1] - ((2.f * delays[i - 1]) / (4.f * i + 1));
        }

        if(delay < motor_max_speed_delay){
            delay = motor_max_speed_delay;
        }
        delays.push_back(delay);
    }

    std::vector<float> delays_buffer;
    delays_buffer = delays;
    if(!even){
        delays_buffer.pop_back();
    }
    std::reverse(delays_buffer.begin(), delays_buffer.end());

    delays.insert(delays.end(), delays_buffer.begin(), delays_buffer.end());

    std::vector<float> seconds_delays;
    seconds_delays.reserve(delays.size());
    for(const float copy_delay: delays){
        seconds_delays.push_back(microseconds2Seconds(copy_delay));
    }

    return seconds_delays;
}

std::vector<float> Movement::accelerateToVelocity(float velocity, float acceleration) {
    std::vector<float> delays;
    float angle = one_pulse_step;
    const float constant = 0.13568198123907316536355537605674f;

    float c0 = 2000000.f * std::sqrt(2.f * angle / acceleration) * constant;

    int i = 0;
    float delay = 0;
    float current_velocity = 0;

    while(true){
        delay = c0;
        if(i > 0){
            delay = delays[i - 1] - ((2.f * delays[i - 1]) / (4.f * i + 1));
        }
        delays.push_back(delay);

        current_velocity = jointVelFromMotorVel(motorVel(microseconds2Seconds(delay)));
        if(current_velocity >= velocity){
            break;
        }
        i++;
    }

    std::vector<float> seconds_delays;
    seconds_delays.reserve(delays.size());
    for(const float copy_delay: delays){
        seconds_delays.push_back(microseconds2Seconds(copy_delay));
    }

    return seconds_delays;
}*/
