#include "../include/joint.hpp"

Joint::Joint(std::unique_ptr<Driver>& driver, std::shared_ptr<Endstop> sensor, uint16_t gear_teeth,
             DIRECTION homing_direction) {
    this->driver = std::move(driver);

    this->endstop = std::move(sensor);

    this->gear_teeth = gear_teeth;
    this->homing_direction = homing_direction;

    this->movement = Movement(this->driver->getMotorResolution(), this->driver->getDriverResolution(), this->driver->getGearTeeth(), gear_teeth);
}

void Joint::setHomingSteps(uint16_t homing_steps) {
    this->homing_steps = homing_steps;
}

void Joint::setHomingVelocity(float homing_velocity) {
    this->homing_velocity = homing_velocity;
}

void Joint::setHomingAcceleration(float homing_acceleration) {
    this->homing_acceleration = homing_acceleration;
}

void Joint::setMaxVelocity(float max_velocity) {
    this->max_velocity = max_velocity;
}

void Joint::setMaxAcceleration(float max_acceleration) {
    this->max_acceleration = max_acceleration;
}

void Joint::setMinPosition(float min_position) {
    this->min_pos = min_position;
}

void Joint::setMaxPosition(float max_position) {
    this->max_pos = max_position;
}

void Joint::setOffset(float offset) {
    this->offset = offset;
}

void Joint::setBaseAngle(float base_angle) {
    this->base_angle = base_angle;
}

unsigned int Joint::degrees2Steps(float degrees) {
    int steps = ((degrees / (float)driver->getMotorResolution()) / (driver->getGearTeeth() / (float)gear_teeth)) * driver->getMotorResolution();
    return steps;
}

void Joint::moveBySteps(unsigned int steps, DIRECTION direction, float max_velocity, float max_acceleration) {
    std::vector<float> delays = movement.calculateSteps(steps, max_velocity, max_acceleration);

    this->driver->setDirection(direction);

    for(const float delay: delays){
        this->driver->moveDelay(delay);
    }
}

void Joint::move2Pos(float position) {
    if(homed){
        if(min_pos < position && position < max_pos){
            float new_position = joint_position - position;

            DIRECTION move_direction;
            if(homing_direction == ANTICLOCKWISE){
                if(new_position >= 0){
                    move_direction = ANTICLOCKWISE;
                }
                else{
                    move_direction = CLOCKWISE;
                }
            }
            else{
                if(new_position >= 0){
                    move_direction = CLOCKWISE;
                }
                else{
                    move_direction = ANTICLOCKWISE;
                }
            }

            new_position = std::abs(new_position);
            unsigned int steps = degrees2Steps(new_position);
            moveBySteps(steps, move_direction, max_velocity, max_acceleration);
            joint_position = position;
        }
    }
}

void Joint::homeJoint() {
    DIRECTION first_direction;
    DIRECTION second_direction;
    if(homing_direction == ANTICLOCKWISE){
        first_direction = ANTICLOCKWISE;
        second_direction = CLOCKWISE;
    }
    else{
        first_direction = CLOCKWISE;
        second_direction = ANTICLOCKWISE;
    }

    std::vector<float> acceleration_delays = movement.accelerateToVelocity(homing_velocity);
    std::vector<float> acceleration_delays2 = movement.accelerateToVelocity(homing_velocity/5.f);

    float phase_time1 = movement.phaseTime(homing_velocity);
    float phase_time2 = movement.phaseTime(homing_velocity/5);

    while(true){
        driver->setDirection(first_direction);

        for(const float delay: acceleration_delays){
            driver->moveDelay(delay);
            if(endstop->checkSensor()){
                break;
            }
        }

        if(endstop->checkSensor()){
            moveBySteps(homing_steps, second_direction, max_velocity, max_acceleration);

            driver->setDirection(first_direction);

            for(const float delay: acceleration_delays2){
                driver->moveDelay(delay);

                if(endstop->checkSensor()){
                    break;
                }
            }
            if(endstop->checkSensor()){
                break;
            }
            else{
                while(true){
                    driver->moveDelay(phase_time2);

                    if(endstop->checkSensor()){
                        break;
                    }
                }
                break;
            }
        }
        else{
            while(true){
                driver->moveDelay(phase_time1);

                if(endstop->checkSensor()){
                    break;
                }
            }

            if(endstop->checkSensor()){
                moveBySteps(homing_steps, second_direction, max_velocity, max_acceleration);

                driver->setDirection(first_direction);

                for(const float delay: acceleration_delays2){
                    driver->moveDelay(delay);

                    if(endstop->checkSensor()){
                        break;
                    }
                }
                if(endstop->checkSensor()){
                    break;
                }
                else{
                    while(true){
                        driver->moveDelay(phase_time2);

                        if(endstop->checkSensor()){
                            break;
                        }
                    }
                    break;
                }
            }
        }
    }
    joint_position = 0;
    if(offset != 0){
        if(offset < 0){
            float offset_abs = std::abs(offset);

            float gear_ratio = movement.getGearSpeedRatio();

            int steps = (((offset_abs / driver->getMotorResolution()) / gear_ratio) * driver->getDriverResolution());

            moveBySteps(steps, first_direction, max_velocity, max_acceleration);
        }
        else{
            move2Pos(offset);
        }
    }
    joint_position = 0;
    homed = true;
}



