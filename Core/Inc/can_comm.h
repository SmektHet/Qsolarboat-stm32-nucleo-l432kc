#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "imu.h"
#include "stm32l4xx_hal.h"

void initCAN(CAN_HandleTypeDef *hcan1);
void SendCANData(CAN_HandleTypeDef *hcan1, const float *corrected_distance, const FusedOrientation *fusedOrientation);

#endif