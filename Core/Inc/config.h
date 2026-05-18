#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// #define DEBUG_PRINTS // Comment out to disable debug prints over UART2
// #define USE_SINGLE_IMU // Comment out to use all three IMUs, otherwise only the middle one will be used

#define SENSOR_COUNT 4

//? 0x01 and 0x02 for ultrasonic sensors, 0x50 for the IMU
//! The ultrasonic sensor may NOT be requested after each other, otherwise the response from the first one may be overwritten by the second
static const uint8_t sensor_addresses[SENSOR_COUNT] = {0x02, 0x50, 0x51, 0x52};
#define RESPONSE_TIMEOUT 50
#define UART_RX_BUFFER_SIZE 128
#define ACCUM_BUFFER_SIZE   256

#endif