#include "../include/gripper.hpp"

Gripper::Gripper(GPIO_PIN servo_pwm_pin, GPIO_PIN feedback_pin, ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim) {
    this->servo_pwm_pin = servo_pwm_pin;
    this->feedback_pin = feedback_pin;

    this->tim = tim;

    // Enable timer
    HAL_TIM_Base_Start_IT(tim);


}

operation_status Gripper::enableGripper() {
    HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start
    return operation_status_init(gripper, success, 0x00);
}

operation_status Gripper::disableGripper() {
    HAL_TIM_PWM_Stop(tim, TIM_CHANNEL_1);
    return operation_status_init(gripper, success, 0x00);
}

operation_status Gripper::setPosition(float position) {
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, position_ms);
}

operation_status Gripper::getGripperStatus() {

}

float Gripper::getCurrentPosition() {
    return this->position;
}

float Gripper::readCurrentPosition() {
    // 0.5 V - fully open
    // 2.2 V - fully closed
    // From Pololu documentaion, but to be checked


}
