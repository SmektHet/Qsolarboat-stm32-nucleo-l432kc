#include "Servo.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_tim.h"

int maxServoValue = 800;
int minServoValue = 400;


void InitServo(TIM_HandleTypeDef htim2){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 600);

}

void RunServo(int data, TIM_HandleTypeDef htim2){
    int output = 400 + data * 40;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
}