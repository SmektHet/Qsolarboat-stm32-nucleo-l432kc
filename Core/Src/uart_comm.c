#include "uart_comm.h"

extern UART_HandleTypeDef huart2;

/* ================= DATA ================= */
IMU_Data imu_data[3];   // 0x50, 0x51, 0x52
float distance[2];      // 0x01, 0x02

static uint8_t tx_data[8];
uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_accum[ACCUM_BUFFER_SIZE];
uint16_t rx_len = 0;

/* ================= HELPERS ================= */
static int imu_index(uint8_t addr)
{
    if (addr >= 0x50 && addr <= 0x52)
        return addr - 0x50;
    return -1;
}

static int dist_index(uint8_t addr)
{
    if (addr == 0x01) return 0;
    if (addr == 0x02) return 1;
    return -1;
}

/* ================= DEBUG ================= */
static void debug_print_bytes(const char *label, uint8_t *buf, uint16_t len)
{
    char msg[256];
    int pos = snprintf(msg, sizeof(msg), "%s [%d]: ", label, len);

    for (uint16_t i = 0; i < len && pos < sizeof(msg) - 4; i++)
        pos += snprintf(msg + pos, sizeof(msg) - pos, "%02X ", buf[i]);

    pos += snprintf(msg + pos, sizeof(msg) - pos, "\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, pos, 100);
}

/* ================= UART DMA ================= */
void UART_Restart_DMA(UART_HandleTypeDef *huart1)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1->hdmarx, DMA_IT_HT);
}

/* ================= MODBUS REQUEST ================= */
void Modbus_Send_Request(UART_HandleTypeDef *huart1, uint8_t addr)
{
    tx_data[0] = addr;
    tx_data[1] = 0x03;

    uint16_t reg = 0, count = 0;

    if (addr == 0x01 || addr == 0x02)
    {
        reg = 0x0222;
        count = 0x0001;
    }
    else if (addr >= 0x50 && addr <= 0x52)
    {
        reg = 0x0034;
        count = 0x000C;
    }

    tx_data[2] = reg >> 8;
    tx_data[3] = reg & 0xFF;
    tx_data[4] = count >> 8;
    tx_data[5] = count & 0xFF;

    uint16_t crc = crc16(tx_data, 6);
    tx_data[6] = crc & 0xFF;
    tx_data[7] = crc >> 8;

    HAL_UART_Transmit(huart1, tx_data, 8, 1000);
}

/* ================= PRINT ================= */
void print_data(uint8_t addr)
{
    char msg[256];
    int len = 0;

    if (addr == 0x01 || addr == 0x02)
    {
        int idx = dist_index(addr);
        len = snprintf(msg, sizeof(msg),
            "[0x%02X] DIST: %.2f\r\n",
            addr, distance[idx]);
    }
    else if (addr >= 0x50 && addr <= 0x52)
    {
        int idx = imu_index(addr);
        IMU_Data *d = &imu_data[idx];

        len = snprintf(msg, sizeof(msg),
            "[0x%02X] ACC: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | ANG: %.2f %.2f %.2f\r\n",
            addr,
            d->acc[0], d->acc[1], d->acc[2],
            d->gyro[0], d->gyro[1], d->gyro[2],
            d->angle[0], d->angle[1], d->angle[2]);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
}

/* ================= BUFFER ================= */
void buffer_consume(uint16_t len)
{
    if (len >= rx_len) { rx_len = 0; return; }

    memmove(rx_accum, rx_accum + len, rx_len - len);
    rx_len -= len;
}

/* ================= STREAM PROCESS ================= */
void process_uart_stream(UART_HandleTypeDef *huart1)
{
    while (rx_len >= 5)
    {
        /* ===== HARD SYNC ===== */
        if (!(
            (rx_accum[0] == 0x01 || rx_accum[0] == 0x02 ||
             rx_accum[0] == 0x50 || rx_accum[0] == 0x51 || rx_accum[0] == 0x52)
            && rx_accum[1] == 0x03))
        {
            buffer_consume(1);
            continue;
        }

        /* ===== FRAME SIZE ===== */
        uint8_t byte_count = rx_accum[2];
        uint16_t frame_size = 3 + byte_count + 2;

        if (rx_len < frame_size)
            return;

        /* ===== CRC ===== */
        uint16_t crc_rx   = rx_accum[frame_size - 2] | (rx_accum[frame_size - 1] << 8);
        uint16_t crc_calc = crc16(rx_accum, frame_size - 2);

        if (crc_rx != crc_calc)
        {
            debug_print_bytes("CRC FAIL", rx_accum, frame_size);
            buffer_consume(1);
            continue;
        }

        /* ===== VALID FRAME ===== */
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

        uint8_t addr = rx_accum[0];

        /* DISTANCE */
        int d_idx = dist_index(addr);
        if (d_idx >= 0)
        {
            uint16_t value = (rx_accum[3] << 8) | rx_accum[4];
            distance[d_idx] = (float)value;

            print_data(addr);
        }

        /* IMU */
        int i_idx = imu_index(addr);
        if (i_idx >= 0)
        {
            IMU_Parse(rx_accum, frame_size, &imu_data[i_idx]);
            print_data(addr);
        }

        buffer_consume(frame_size);
    }
}

/* ================= CALLBACKS ================= */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart1, uint16_t Size)
{
    if (huart1->Instance != USART1) return;

    if ((rx_len + Size) > ACCUM_BUFFER_SIZE)
        rx_len = 0;

    memcpy(&rx_accum[rx_len], uart1_rx_buffer, Size);
    rx_len += Size;
    
    process_uart_stream(huart1);
    UART_Restart_DMA(huart1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart1)
{
    if (huart1->Instance != USART1) return;

    __HAL_UART_CLEAR_OREFLAG(huart1);
    __HAL_UART_CLEAR_FEFLAG(huart1);
    __HAL_UART_CLEAR_NEFLAG(huart1);

    UART_Restart_DMA(huart1);
}

void UART_Comm_Init(UART_HandleTypeDef *huart1)
{
    UART_Restart_DMA(huart1);
}