#include "Servo.h"

int maxServoValue = 200;   // corresponds to 2 ms pulse (~full 180°)
int minServoValue = 150;   // corresponds to 1 ms pulse (~0°)

void InitServo(TIM_HandleTypeDef *htim){
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    // Move servo to initial (min) position
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, minServoValue);
}

void RunServo(float data, TIM_HandleTypeDef *htim){
    if(data < 0) data = 0;
    if(data > 10) data = 10;

    // Map 0–10 to minServoValue–maxServoValue
    float output = minServoValue + ((maxServoValue - minServoValue) * data) / 10;

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, output);
}