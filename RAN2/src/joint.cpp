#include "../include/joint.hpp"

Joint::Joint(uint8_t joint_number, std::unique_ptr<drivers::Driver>& driver,
             uint16_t gear_teeth, DIRECTION homing_direction, std::shared_ptr<Endstop> sensor, std::shared_ptr<MagneticEncoder> encoder) {
    
    this->joint_number = joint_number;
    this->driver = std::move(driver);

    this->gear_teeth = gear_teeth;
    this->homing_direction = homing_direction;

    this->movement = Movement(this->driver->getMotorResolution(), this->driver->getDriverResolution(),
                              this->driver->getGearTeeth(), gear_teeth);

    if(sensor != nullptr){
        setEndstopHoming();
        endstop_homing = true;
        encoder_homing = false;
        status = operational;
    }
    else if(sensor == nullptr && encoder != nullptr){
        if(encoder->getDegPerRotation() == 1.f){
            endstop_homing = false;
            encoder_homing = true;
            status = operational;
        }
    }
    else{
        status = homing_device_error;
    }
    this->endstop = sensor;
    this->encoder = encoder;
}

JOINT_STATUS Joint::getJointStatus() {
    return this->status;
}

uint8_t Joint::getJointNumber() const {
    return this->joint_number;
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
    float gear_ratio = (float)this->gear_teeth/(float)driver->getGearTeeth();
    float steps = (degrees/driver->getMotorResolution())*gear_ratio*(float)driver->getDriverResolution()*2.f;
    return (unsigned int)steps;
}

void Joint::accelerateJoint(drivers::DIRECTION direction, float velocity, float acceleration) {
    this->driver->setDirection(direction);
    this->driver->initializeMovement(joint_number, ACCEL, movement.getMinDelay(velocity),
                                     movement.getStartDelay(acceleration), 0);
}

void Joint::moveJoint(drivers::DIRECTION direction, float velocity) {
    this->driver->setDirection(direction);
    this->driver->initializeMovement(joint_number, RUN_CONST, 0, movement.phaseTime(velocity), 0);
}

void Joint::moveJointBySteps(unsigned int steps, drivers::DIRECTION direction, float max_velocity,
                             float max_acceleration, bool blocking) {
    this->driver->setDirection(direction);
    this->driver->initializeMovement(joint_number, MOVE_STEPS, movement.getMinDelay(max_velocity),
                                     movement.getStartDelay(max_acceleration), steps);
    if(blocking){
        while(driver->getMovement(joint_number));
    }
}

void Joint::move2Pos(float position, bool blocking) {
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

            new_position = fabsf(new_position);
            unsigned int steps = degrees2Steps(new_position);
            moveJointBySteps(steps, move_direction, max_velocity, max_acceleration, blocking);
            joint_position = position;
        }
    }
}

bool Joint::setEndstopHoming() {
    endstop_homing = true;
    endstop_homing = false;
    return true;
}

bool Joint::setEncoderHoming() {
    if(encoder != nullptr){
        if(encoder->getDegPerRotation() == 1.f){
            endstop_homing = false;
            encoder_homing = true;
            return true;
        }
    }
    return false;
}

void Joint::homeJoint() {
    DIRECTION first_direction;
    DIRECTION second_direction;
    if (homing_direction == ANTICLOCKWISE) {
        first_direction = ANTICLOCKWISE;
        second_direction = CLOCKWISE;
    } else {
        first_direction = CLOCKWISE;
        second_direction = ANTICLOCKWISE;
    }

    if (endstop_homing) {
        while (true) {
            accelerateJoint(first_direction, homing_velocity, homing_acceleration);
            while (driver->getMovement(joint_number) && !endstop->checkSensor());
            driver->stopMovement(joint_number);


            if (endstop->checkSensor()) {
                moveJointBySteps(homing_steps, second_direction, max_velocity, max_acceleration, true);

                accelerateJoint(first_direction, homing_velocity / 5.f, homing_acceleration);
                while (driver->getMovement(joint_number) && !endstop->checkSensor());
                driver->stopMovement(joint_number);

                if (endstop->checkSensor()) {
                    break;
                } else {
                    moveJoint(first_direction, homing_velocity / 5.f);
                    while (driver->getMovement(joint_number) && !endstop->checkSensor());
                    driver->stopMovement(joint_number);
                    break;
                }
            } else {
                moveJoint(first_direction, homing_velocity);
                while (driver->getMovement(joint_number) && !endstop->checkSensor());
                driver->stopMovement(joint_number);

                if (endstop->checkSensor()) {
                    moveJointBySteps(homing_steps, second_direction, max_velocity, max_acceleration, true);

                    accelerateJoint(first_direction, homing_velocity / 5.f, homing_acceleration);
                    while (driver->getMovement(joint_number) && !endstop->checkSensor());
                    driver->stopMovement(joint_number);

                    if (endstop->checkSensor()) {
                        break;
                    } else {
                        moveJoint(first_direction, homing_velocity / 5.f);
                        while (driver->getMovement(joint_number) && !endstop->checkSensor());
                        driver->stopMovement(joint_number);
                        break;
                    }
                }
            }
        }
    } else {
        while (true) {
            accelerateJoint(first_direction, homing_velocity, homing_acceleration);
            while (driver->getMovement(joint_number) && !encoder->homeEncoder());
            driver->stopMovement(joint_number);


            if (encoder->homeEncoder()) {
                break;
            } else {
                moveJoint(first_direction, homing_velocity);
                while (driver->getMovement(joint_number) && !encoder->homeEncoder());
                driver->stopMovement(joint_number);

                break;
            }
        }
    }
    joint_position = 0;
    homed = true;
    if(offset != 0){
        if(offset < 0){
            unsigned int steps = degrees2Steps(std::abs(offset));
            moveJointBySteps(steps, first_direction, max_velocity, max_acceleration, true);
        }
        else{
            move2Pos(offset, true);
        }
    }
    joint_position = 0;
}

float Joint::getEncoderPosition() {
    if(encoder != nullptr){
        return encoder->getPosition();
    }
    return 0;
}

void Joint::enableMotor() {
    this->driver->enableMotor();
}

void Joint::disableMotor() {
    this->driver->disableMotor();
}

bool Joint::isEnabled() {
    return this->driver->isEnabled();
}



