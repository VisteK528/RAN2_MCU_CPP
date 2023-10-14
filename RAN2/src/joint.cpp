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

    this->motor_step = this->driver->getMotorResolution();
    this->driver_microstep = this->driver->getDriverResolution();
    this->motor_shaft_gear_teeth = this->driver->getGearTeeth();

    this->one_pulse_step = deg2Rad(motor_step/(float)driver_microstep);
    this->speed_gear_ratio = (float)motor_shaft_gear_teeth/(float)gear_teeth;
    this->torque_gear_ratio = 1/speed_gear_ratio;

    if(sensor != nullptr){
        setEndstopHoming();
        homing_type = HOMING_TYPE::endstop;
        joint_status = operation_status_init_joint(joint_number, success, 0x00);
    }
    else if(sensor == nullptr && encoder != nullptr){
        if(encoder->getDegPerRotation() == 1.f){
            homing_type = HOMING_TYPE::encoder;
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

operation_status Joint::updateJointStatus() {
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

void Joint::setDirection(drivers::DIRECTION direction) {
    this->homed = false;
    this->homing_direction = direction;
}

unsigned int Joint::degrees2Steps(float degrees) {
    float gear_ratio = (float)this->gear_teeth/(float)driver->getGearTeeth();
    float steps = (degrees/driver->getMotorResolution())*gear_ratio*(float)driver->getDriverResolution()*2.f;
    return (unsigned int)steps;
}

float Joint::motorVel(float phase_time) {
    float term = (360.f/motor_step) * (phase_time * (float)driver_microstep);
    float angular_velocity = (2*(float)M_PI)/term;
    return angular_velocity;
}

float Joint::phaseTime(float joint_ang_vel) {
    float motor_ang_vel = motorVelFromJointVel(joint_ang_vel);
    return one_pulse_step/motor_ang_vel;
}

float Joint::jointVelFromMotorVel(float motor_ang_vel) {
    return motor_ang_vel * speed_gear_ratio;
}

float Joint::motorVelFromJointVel(float joint_ang_vel) {
    return joint_ang_vel / speed_gear_ratio;
}

float Joint::getGearSpeedRatio() const {
    return speed_gear_ratio;
}

float Joint::getMinDelay(float max_speed) {
    return seconds2Microseconds(one_pulse_step/motorVelFromJointVel(max_speed));
}

float Joint::getStartDelay(float acceleration) const {
    float angle = one_pulse_step;
    const float constant = 0.13568198123907316536355537605674f;
    return 2000000.f * std::sqrt(2.f * angle / acceleration) * constant;
}


void Joint::accelerateJoint(drivers::DIRECTION direction, float velocity, float acceleration) {
    this->driver->setDirection(direction);
    this->driver->startMovement(joint_number, ACCEL, getMinDelay(velocity),
                                getStartDelay(acceleration), 0);
}

void Joint::moveJoint(drivers::DIRECTION direction, float velocity) {
    this->driver->setDirection(direction);
    this->driver->startMovement(joint_number, RUN_CONST, 0, phaseTime(velocity), 0);
}

operation_status Joint::stopJoint() {
    this->driver->stopMovement(joint_number);
    this->safeguard_stop = true;
    return operation_status_init_joint(joint_number, success, 0x00);
}

operation_status Joint::startJoint() {
    this->safeguard_stop = false;
    return operation_status_init_joint(joint_number, success, 0x00);
}

void Joint::moveJointBySteps(unsigned int steps, drivers::DIRECTION direction, float max_velocity,
                             float max_acceleration, bool blocking) {
    this->driver->setDirection(direction);
    this->driver->startMovement(joint_number, MOVE_STEPS, getMinDelay(max_velocity),
                                getStartDelay(max_acceleration), steps);
    if(blocking){
        while(driver->checkMovement(joint_number));
    }
}

operation_status Joint::isMovementPossible(float position) const {
    if(min_pos <= position && position <= max_pos) {
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    return operation_status_init_joint(joint_number, failure, 0x0a);
}

operation_status Joint::move2Pos(float position, bool blocking) {
    if(safeguard_stop){
        return operation_status_init_joint(joint_number, failure, 0x0b);
    }

    if(homed){
        if(min_pos <= position && position <= max_pos){
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
        return operation_status_init_joint(joint_number, failure, 0x0a);
    }
    return operation_status_init_joint(joint_number, failure, 0x06);
}

operation_status Joint::setEndstopHoming() {
    homing_type = HOMING_TYPE ::endstop;

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
            homing_type = HOMING_TYPE ::encoder;
            return operation_status_init_joint(joint_number, success, 0x00);
        }
        else{
            return operation_status_init_joint(joint_number, failure, 0x02);
        }
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::setSmartEncoderHoming() {
    if(encoder != nullptr && homing_type == HOMING_TYPE::encoder){
        smart_encoder_homing = true;
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    return operation_status_init_joint(joint_number, failure, 0x08);
}

operation_status Joint::disableSmartEncoderHoming() {
    if(encoder != nullptr && homing_type == HOMING_TYPE::encoder){
        if(smart_encoder_homing){
            smart_encoder_homing = false;
            return operation_status_init_joint(joint_number, success, 0x00);
        }
        return operation_status_init_joint(joint_number, failure, 0x09);
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::homeJoint() {
    DIRECTION first_direction;
    DIRECTION second_direction;

    if(!driver->isMotorEnabled()){
        return operation_status_init_joint(joint_number, failure, 0x07);
    }
    else if(safeguard_stop){
        return operation_status_init_joint(joint_number, failure, 0x0b);
    }

    if (homing_direction == ANTICLOCKWISE) {
        first_direction = ANTICLOCKWISE;
        second_direction = CLOCKWISE;
    } else {
        first_direction = CLOCKWISE;
        second_direction = ANTICLOCKWISE;
    }

    if (homing_type == HOMING_TYPE::endstop) {
        while (!safeguard_stop) {
            accelerateJoint(first_direction, homing_velocity, homing_acceleration);
            while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
            driver->stopMovement(joint_number);


            if (endstop->checkEndstop()) {
                moveJointBySteps(homing_steps, second_direction, max_velocity, max_acceleration, true);

                accelerateJoint(first_direction, homing_velocity / 5.f, homing_acceleration);
                while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
                driver->stopMovement(joint_number);

                if (endstop->checkEndstop()) {
                    break;
                } else {
                    moveJoint(first_direction, homing_velocity / 5.f);
                    while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
                    driver->stopMovement(joint_number);
                    break;
                }
            } else {
                moveJoint(first_direction, homing_velocity);
                while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
                driver->stopMovement(joint_number);

                if (endstop->checkEndstop()) {
                    moveJointBySteps(homing_steps, second_direction, max_velocity, max_acceleration, true);

                    accelerateJoint(first_direction, homing_velocity / 5.f, homing_acceleration);
                    while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
                    driver->stopMovement(joint_number);

                    if (endstop->checkEndstop()) {
                        break;
                    } else {
                        moveJoint(first_direction, homing_velocity / 5.f);
                        while (driver->checkMovement(joint_number) && !endstop->checkEndstop() && !safeguard_stop);
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

        if(smart_encoder_homing)
        {
            encoder->updateRawAngle();
            float position = encoder->getRawPosition();
            float homing_position = encoder->getHomingPosition();

            if (position - homing_position < 0) {
                if(encoder->getEncoderDirection() == EncoderCW){
                    first_direction = ANTICLOCKWISE;
                }
                else{
                    first_direction = CLOCKWISE;
                }
            } else {
                if(encoder->getEncoderDirection() == EncoderCW){
                    first_direction = CLOCKWISE;
                }
                else{
                    first_direction = ANTICLOCKWISE;
                }
            }
        }

        while (!safeguard_stop) {
            accelerateJoint(first_direction, homing_velocity, homing_acceleration);
            while (driver->checkMovement(joint_number) && !encoder->homeEncoder() && !safeguard_stop){
                HAL_Delay(3);
            }
            driver->stopMovement(joint_number);


            if (encoder->homeEncoder()) {
                break;
            } else {
                moveJoint(first_direction, homing_velocity);
                while (driver->checkMovement(joint_number) && !encoder->homeEncoder() && !safeguard_stop){
                    HAL_Delay(3);
                }
                driver->stopMovement(joint_number);

                break;
            }
        }
    }

    if(safeguard_stop){
        return operation_status_init_joint(joint_number, failure, 0x0b);
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
    return this->driver->isMotorEnabled();
}

bool Joint::isMoving() {
    return this->driver->checkMovement(joint_number);
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
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::getEncoderData(MagneticEncoderData *data) {
    if(encoder != nullptr){
        operation_status encoder_status = encoder->checkEncoder();
        if(encoder_status.result != success){
            return encoder_status;
        }
        data->position = encoder->getPosition();
        data->velocity = encoder->getVelocity();
        data->acceleration = encoder->getAcceleration();
        data->rawPosition = encoder->getRawPosition();
        return operation_status_init_joint(joint_number, success, 0x00);
    }
    return operation_status_init_joint(joint_number, failure, 0x05);
}

operation_status Joint::getJointPosition(float *position) {
    *position = this->joint_position;
    return operation_status_init_joint(joint_number, success, 0x00);
}
