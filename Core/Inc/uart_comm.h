#ifndef UART_COMM_H
#define UART_COMM_H

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "modbus_crc.h"
#include "imu.h"

#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define UART_RX_BUFFER_SIZE 128
#define ACCUM_BUFFER_SIZE   256

void UART_Restart_DMA(UART_HandleTypeDef *huart1);
void Modbus_Send_Request(UART_HandleTypeDef *huart1, uint8_t slaveAddress);
void print_data(uint8_t addr);
void buffer_consume(uint16_t size);
uint16_t get_imu_frame_size(uint8_t *buf, uint16_t len);
void process_uart_stream(UART_HandleTypeDef *huart1);
void UART_Comm_Init(UART_HandleTypeDef *huart1);
#endif