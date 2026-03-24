/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : UART To CAN transmit test with LED blink (NUCLEO-L432KC)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include "modbus_crc.h"
#include "stm32l4xx_hal.h"
#include "MadgwickAHRS.h"
#include "imu.h"
#include "uart_comm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 128
#define ACCUM_BUFFER_SIZE   256
#define SENSOR_COUNT 2
#define RESPONSE_TIMEOUT 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
IMU_Data data;
uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_accum[ACCUM_BUFFER_SIZE];
uint16_t rx_len = 0;


uint32_t last_request_time = 0;
uint8_t waiting_for_response = 1;
//? 0x01 and 0x02 for ultrasonic sensors, 0x50 for the IMU
//! The ultrasonic sensor may NOT be requested after each other, otherwise the response from the first one may be overwritten by the second
// uint8_t sensor_addresses[SENSOR_COUNT] = {0x50,0x02, 0x50,0x02, 0x50};
uint8_t sensor_addresses[SENSOR_COUNT] = {0x02, 0x50};
uint8_t current_sensor_index = 0;
typedef enum {
    STATE_INIT = 0,
    STATE_SEND_REQUEST,
    STATE_WAIT_RESPONSE,
    STATE_PROCESS_DATA
} AppState;
AppState state = STATE_INIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void print_data(void)
{
    char msg[256];

    int len = snprintf(msg, sizeof(msg),
        "DIST: %.2f | ACC: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | ANG: %.2f %.2f %.2f\r\n",
        data.distance,
        data.acc[0], data.acc[1], data.acc[2],
        data.gyro[0], data.gyro[1], data.gyro[2],
        data.angle[0], data.angle[1], data.angle[2]
    );

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);
}

void buffer_consume(uint16_t len)
{
    if(len >= rx_len)
    {
      rx_len = 0;
        return;
    }
    memmove(rx_accum, rx_accum + len, rx_len - len);
    rx_len -= len;
  }

uint16_t get_imu_frame_size(uint8_t *buf, uint16_t len)
{
    // minimal IMU frame
    if(len < 7) return 0;

    // Try to detect valid CRC dynamically
    for(uint16_t size = 7; size <= len; size++)
    {
        uint16_t crc_rx = buf[size - 2] | (buf[size - 1] << 8);
        uint16_t crc_calc = crc16(buf, size - 2);

        if(crc_rx == crc_calc)
        {
            return size; // found valid frame
        }
    }

    return 0;
}

void process_uart_stream(void)
{
  while(rx_len >= 5)
  {
    uint8_t addr = rx_accum[0];
    if(addr == 0x01 || addr == 0x02)
    {
      if(rx_len < 7)
          return;

      uint16_t crc_rx = rx_accum[5] | (rx_accum[6] << 8);
      uint16_t crc_calc = crc16(rx_accum, 5);

      if(crc_rx == crc_calc)
      { 
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
          uint16_t value = (rx_accum[3] << 8) | rx_accum[4];
          data.distance = (float)value;
          buffer_consume(7);
          continue;
      }
      else
      {
          // bad alignment → shift 1 byte
          buffer_consume(1);
          continue;
      }
    }

    else if(addr == 0x50)
    {
      uint16_t imu_size = get_imu_frame_size(rx_accum, rx_len);

      if(imu_size == 0 || rx_len < imu_size)
          return; // wait for more data

      if(IMU_Parse(rx_accum, imu_size, &data) == 0)
      {   
          HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
          print_data();
          buffer_consume(imu_size);
          continue;
      }
      else
      {
          // parsing failed → shift
          buffer_consume(1);
          continue;
      }
    }

    // =========================
    // UNKNOWN BYTE
    // =========================
    else
    {
        buffer_consume(1);
    }
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance != USART1) return;

    // Prevent overflow
    if((rx_len + Size) > ACCUM_BUFFER_SIZE)
    {
        rx_len = 0; // reset if overflow
    }

    // Append incoming fragment
    memcpy(&rx_accum[rx_len], uart1_rx_buffer, Size);
    rx_len += Size;

    // Process accumulated stream
    process_uart_stream();

    // Restart DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Clear error flags
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);

        // Restart DMA reception
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  /* USER CODE BEGIN 2 */

  /* Start CAN peripheral */
  HAL_CAN_Start(&hcan1);
  TxHeader.StdId = 0x123;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 1; 
  TxHeader.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t request_timestamp = 0;
  while (1)
  {
    switch(state)
    {
      case STATE_INIT:
      {
          current_sensor_index = 0;
          state = STATE_SEND_REQUEST;
          break;
      }

      case STATE_SEND_REQUEST:
      {
          HAL_Delay(1); // ensure previous frame done

          Modbus_Send_Request(&huart1, sensor_addresses[current_sensor_index]);
          request_timestamp = HAL_GetTick();

          state = STATE_WAIT_RESPONSE;
          break;
      }

      case STATE_WAIT_RESPONSE:
      {
          if ((HAL_GetTick() - request_timestamp) > RESPONSE_TIMEOUT) //! ADD RECIEVE FUNCTION FOR EARLIER RESPONSE
          {
              state = STATE_PROCESS_DATA;
          }
          break;
      }

      case STATE_PROCESS_DATA:
      {
        
        current_sensor_index++;
        if(current_sensor_index >= SENSOR_COUNT)
        {
          float distance_mm = data.distance;
          
          float roll_rad = deg_to_rad(data.angle[0]);
          float pitch_rad = deg_to_rad(data.angle[1]);
          
          float h_cm = corrected_height(distance_mm, roll_rad, pitch_rad);
          float foil_deg = foil_angle(h_cm);
          
          char msg[64];
          int len = snprintf(msg, sizeof(msg), "Foil: %.2f deg\r\n", foil_deg);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);

              current_sensor_index = 0;
          }
          
          state = STATE_SEND_REQUEST;
          break;
      }

      default:
      {
          state = STATE_INIT;
          break;
      }
    }
  }
  /* USER CODE END 3 */
}



//! ------------------ DON'T EDIT BELOW THIS LINE ------------------
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
