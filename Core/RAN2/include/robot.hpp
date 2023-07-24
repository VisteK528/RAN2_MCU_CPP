#ifndef ROBOTARMNUMBER2CPP_ROBOT_HPP
#define ROBOTARMNUMBER2CPP_ROBOT_HPP

#include "joint.hpp"
#include <memory>
#include <unordered_map>
#include "../../Inc/gpio.h"

class Robot{
public:
    Robot(std::vector<std::unique_ptr<Joint>>& joints);
    Robot()=default;

    void home();
    void homeJoint(int joint_number);
    void wait(uint32_t seconds);
private:

    std::unordered_map<int, std::unique_ptr<Joint>> joints;
    bool homed = false;
};

Robot buildRobot();


#endif //ROBOTARMNUMBER2CPP_ROBOT_HPP
