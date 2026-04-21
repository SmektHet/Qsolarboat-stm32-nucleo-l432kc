#include "uart_comm.h"
#include "modbus_crc.h"

static uint8_t tx_data[8];

void UART_Comm_Init(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void Modbus_Send_Request(UART_HandleTypeDef *huart, uint8_t addr)
{
    tx_data[0] = addr;
    tx_data[1] = 0x03; // Function code for Read holding Registers (0x03)

    if (addr == 0x01 || addr == 0x02)
    {
        tx_data[2] = 0x02; // Register address (0x0222)
        tx_data[3] = 0x22; 
        tx_data[4] = 0x00; // Number of registers to read (0x0001)
        tx_data[5] = 0x01;
    } else if (addr == 0x50 || addr == 0x51 || addr == 0x52) {
        tx_data[2] = 0x00; // Register address (0x0034) start at AX
        tx_data[3] = 0x34; 
        tx_data[4] = 0x00; // Number of registers to read (0x000C) 12 registers for AX, AY, AZ, GX, GY, GZ, HX, HY, HZ, Roll, Pitch, Yaw
        tx_data[5] = 0x0C;
    }

    uint16_t crc = crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = (crc >> 8) & 0xFF;

    HAL_UART_Transmit(huart, tx_data, 8, 1000);
}