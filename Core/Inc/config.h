#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// #define DEBUG_PRINTS // Comment out to disable debug prints over UART2
// #define USE_SINGLE_IMU // Comment out to use all three IMUs, otherwise only the middle one will be used

//! Use 0x00 as placeholder for indexing
static const uint8_t imu_addresses[] = { 0x50, 0x51, 0x52 };
static const uint8_t us_addresses[]  = { 0x01, 0x02, 0x00 };
#define SENSOR_PAIR_COUNT 3

#define IMU_TIMEOUT_MS 20
#define ULTRASONIC_TIMEOUT_MS 50

#define UART_RX_BUFFER_SIZE 128
#define ACCUM_BUFFER_SIZE 256

#endif