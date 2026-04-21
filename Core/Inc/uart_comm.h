#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define UART_RX_BUFFER_SIZE 128

extern uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];

void UART_Comm_Init(UART_HandleTypeDef *huart);
void Modbus_Send_Request(UART_HandleTypeDef *huart, uint8_t slaveAddress);

#endif