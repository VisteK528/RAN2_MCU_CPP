#ifndef ROBOTARMNUMBER2CPP_ROBOT_HPP
#define ROBOTARMNUMBER2CPP_ROBOT_HPP

#include <iostream>
#include "joint.hpp"
#include <memory>
#include <unordered_map>

class Robot{
public:
    Robot(std::vector<std::unique_ptr<Joint>> joints);
    Robot()=default;

    void move2Position(float x, float y, float z, ALIGNMENT alignment);
    void home();
    void homeJoint(int joint_number);
    void wait(float seconds);
private:

    std::unordered_map<int, std::unique_ptr<Joint>> joints;
    bool homed = false;
};

Robot buildRobot();


#endif //ROBOTARMNUMBER2CPP_ROBOT_HPP
