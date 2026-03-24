#ifndef SERVO_H
#define SERVO_H

#include "stm32l4xx_hal.h"

// Servo limits (can tweak per board if needed)
extern int maxServoValue;
extern int minServoValue;

// Function prototypes (FIXED)
void InitServo(TIM_HandleTypeDef *htim);
void RunServo(float data, TIM_HandleTypeDef *htim);

#endif // SERVO_H