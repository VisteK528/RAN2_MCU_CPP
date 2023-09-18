#include "../include/gripper.hpp"

static float measureADC(ADC_HandleTypeDef* adc, uint16_t number_of_samples){
    float measured = 0;
    for(uint16_t i = 0; i < number_of_samples; i++){
        HAL_ADC_Start(adc);
        HAL_ADC_PollForConversion(adc, HAL_MAX_DELAY);
        uint32_t value = HAL_ADC_GetValue(adc);
        measured += 3.3f * (float)value / 4096.0f;;
    }
    measured /= (float)number_of_samples;
    return measured;
}

effector::Gripper::Gripper(GPIO_PIN servo_pwm_pin, GPIO_PIN feedback_pin, ADC_HandleTypeDef *adc, TIM_HandleTypeDef *tim) {
    this->servo_pwm_pin = servo_pwm_pin;
    this->feedback_pin = feedback_pin;

    this->tim = tim;
    this->adc = adc;

    // Enable timer
    HAL_StatusTypeDef status;
    status = HAL_TIM_Base_Start_IT(tim);
    if(status == HAL_ERROR){
        current_status = operation_status_init(gripper, failure, 0x04);
    }

    float feedback_level = measureADC(adc, DEFAULT_SAMPLES);
    if(feedback_level < GRIPPER_LOW - 0.2){
        current_status = operation_status_init(gripper, failure, 0x02);
    }
    else if(feedback_level > GRIPPER_HIGH + 0.5){
        current_status = operation_status_init(gripper, failure, 0x03);
    }

    current_status = operation_status_init(gripper, success, 0x00);
}

operation_status effector::Gripper::checkGripper() {
    // Enable timer
    HAL_StatusTypeDef status;
    status = HAL_TIM_Base_Start_IT(tim);
    if(status == HAL_ERROR){
        current_status = operation_status_init(gripper, failure, 0x04);
    }

    float feedback_level = measureADC(adc, DEFAULT_SAMPLES);
    if(feedback_level < GRIPPER_LOW - 0.2){
        current_status = operation_status_init(gripper, failure, 0x02);
    }
    else if(feedback_level > GRIPPER_HIGH + 0.5){
        current_status = operation_status_init(gripper, failure, 0x03);
    }

    current_status = operation_status_init(gripper, success, 0x00);
    return current_status;
}

operation_status effector::Gripper::initGripper() {
    enableGripper();
    rawSetPosition(0);
    position = 0;
    set_position = 0;
    read_position = readCurrentPosition();
    return operation_status_init(gripper, success, 0x00);
}

operation_status effector::Gripper::enableGripper() {
    HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
    return operation_status_init(gripper, success, 0x00);
}

operation_status effector::Gripper::disableGripper() {
    HAL_TIM_PWM_Stop(tim, TIM_CHANNEL_1);
    return operation_status_init(gripper, success, 0x00);
}

void effector::Gripper::rawSetPosition(float desired_position) {
    position_ms = lroundf(((GRIPPER_HIGH-GRIPPER_LOW)*desired_position/100.f + GRIPPER_LOW)*1000);
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, position_ms);
}

operation_status effector::Gripper::setPosition(float new_position) {
    rawSetPosition(new_position);
    return operation_status_init(gripper, success, 0x00);
}

bool effector::Gripper::smartClose() {
    float i = 0;
    while(i < 90){
        setPosition(i);

        volatile float estimated, measured;
        HAL_Delay(25);

        measured = measureADC(adc, DEFAULT_SAMPLES);
        estimated = ((float)i/100.f)*(GRIPPER_HIGH-GRIPPER_LOW) + GRIPPER_LOW;

        // First check
        if(fabsf(estimated-measured) > 0.15){

            // Second check
            HAL_Delay(250);
            measured = measureADC(adc, DEFAULT_SAMPLES);
            if(fabsf(estimated-measured) > 0.15) {
                break;
            }
        }
        HAL_Delay(25);
        i += 2;
    }
    return true;
}

operation_status effector::Gripper::getGripperStatus() const{
    return current_status;
}

float effector::Gripper::getCurrentPosition() const {
    return this->position;
}

float effector::Gripper::readCurrentPosition() {
    // 0.5 V - fully open
    // 2.2 V - fully closed
    // From Pololu documentaion, but to be checked
    // y - percentage of gripper closed
    // x - measured voltage
    // y = (x - 0.69)/(2.33 0.69) * 100
    float measured_voltage = measureADC(adc, DEFAULT_SAMPLES);
    read_position = (measured_voltage - GRIPPER_LOW)/(GRIPPER_HIGH-GRIPPER_LOW)*100;
    return read_position;
}
