#include "../include/joint.hpp"

static operation_status operation_status_init_joint(uint8_t joint_number, operation_result result, operation_result_code code){
    operation_module_code module;
    switch (joint_number) {
        case 0:
            module = joint1;
            break;
        case 1:
            module = joint2;
            break;
        case 2:
            module = joint3;
            break;
        case 3:
            module = joint4;
            break;
        case 4:
            module = joint5;
            break;
        case 5:
            module = joint6;
            break;
        default:
            return operation_status_init(joint1, failure, code);

    }
    return operation_status_init(module, result, code);
}

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
        joint_status = operation_status_init_joint(joint_number, success, 0x00);
    }
    else if(sensor == nullptr && encoder != nullptr){
        if(encoder->getDegPerRotation() == 1.f){
            endstop_homing = false;
            encoder_homing = true;
            joint_status = operation_status_init_joint(joint_number, success, 0x00);
        }
        else{
            joint_status = operation_status_init_joint(joint_number, failure, 0x02);
        }
    }
    else{
        joint_status = operation_status_init_joint(joint_number, failure, 0x03);
    }
    this->endstop = sensor;
    this->encoder = encoder;
}

operation_status Joint::checkJointStatus() {
    if(encoder != nullptr){
        joint_status = encoder->checkEncoder();

        if(joint_status.result == failure){
            return joint_status;
        }
    }
    joint_status = operation_status_init_joint(joint_number, success, 0x00);
    return joint_status;
}

operation_status Joint::getJointStatus() {
    return this->joint_status;
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

operation_status Joint::move2Pos(float position, bool blocking) {
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
            return operation_status_init_joint(joint_number, success, 0x00);
        }
    }
    return operation_status_init_joint(joint_number, failure, 0x06);
}

operation_status Joint::setEndstopHoming() {
    endstop_homing = true;
    endstop_homing = false;

    if(endstop != nullptr){
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    else{
        return operation_status_init_joint(joint_number, failure, 0x04);
    }
}

operation_status Joint::setEncoderHoming() {
    if(encoder != nullptr){
        if(encoder->getDegPerRotation() == 1.f){
            endstop_homing = false;
            encoder_homing = true;
            return operation_status_init_joint(joint_number, success, 0x00);
        }
        else{
            return operation_status_init_joint(joint_number, failure, 0x02);
        }
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::homeJoint() {
    DIRECTION first_direction;
    DIRECTION second_direction;
    if (homing_direction == ANTICLOCKWISE) {
        first_direction = ANTICLOCKWISE;
        second_direction = CLOCKWISE;
    } else {
        first_direction = CLOCKWISE;
        second_direction = ANTICLOCKWISE;
    }

    if(!driver->isEnabled()){
        return operation_status_init_joint(joint_number, failure, 0x07);
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
        operation_status encoder_status;
        encoder_status = encoder->checkEncoder();

        if(encoder_status.result == failure){
            return encoder_status;
        }

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
    return operation_status_init_joint(joint_number, success, 0x00);
}

operation_status Joint::enableMotor() {
    this->driver->enableMotor();
    return operation_status_init_joint(joint_number, success, 0x00);
}

operation_status Joint::disableMotor() {
    this->driver->disableMotor();
    return operation_status_init_joint(joint_number, success, 0x00);
}

bool Joint::isMotorEnabled() {
    return this->driver->isEnabled();
}

bool Joint::encoderAvailable() {
    if(this->encoder != nullptr){
        return true;
    }
    return false;
}

operation_status Joint::updateEncoder() {
    operation_status status;
    if(this->encoder != nullptr){
        status = this->encoder->updateParameters();

        if(status.result == failure){
            return status;
        }
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::getEncoderData(MagneticEncoderData *data) {
    if(encoder != nullptr){
        data->position = encoder->getPosition();
        data->velocity = encoder->getVelocity();
        data->acceleration = encoder->getAcceleration();
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

