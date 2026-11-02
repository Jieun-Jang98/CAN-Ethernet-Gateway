/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_WHEEL_SPEED       0x386U
#define CAN_ID_STEERING_ANGLE    0x2B0U
#define CAN_ID_STEERING_TORQUE   0x251U
#define CAN_ID_STEERING_COMMAND  0x340U
#define CAN_ID_SPEED_COMMAND     0x341U

#define CAN_SENSOR_PERIOD_MS     20U
#define CAN_COMMAND_TIMEOUT_MS   100U
#define CAN_MAX_DATA_LENGTH      8U
#define TWO_PI_F                 6.28318530718f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
static CAN_RxHeaderTypeDef can_rx_header;
static uint32_t can_tx_mailbox;
static uint32_t can_last_sensor_tick;

/* Watch these variables in the debugger while testing the CAN bus. */
volatile uint32_t can_tx_count = 0U;
volatile uint32_t can_rx_count = 0U;
volatile uint32_t can_tx_error_count = 0U;
volatile uint32_t can_rx_error_count = 0U;
volatile uint32_t can_last_hal_error = 0U;
volatile uint32_t can_last_rx_id = 0U;
volatile uint8_t can_last_rx_dlc = 0U;
volatile uint8_t can_last_rx_is_extended = 0U;
volatile uint8_t can_last_rx_data[CAN_MAX_DATA_LENGTH] = {0U};

/* Currently transmitted simulated vehicle data. */
volatile float sim_wheel_speed_fl_kph = 0.0f;
volatile float sim_wheel_speed_fr_kph = 0.0f;
volatile float sim_steering_angle_deg = 0.0f;
volatile float sim_steering_column_torque = 0.0f;

/* Last decoded ADAS_STEER_CMD (0x340). */
volatile float command_target_angle_deg = 0.0f;
volatile uint16_t command_torque_limit = 0U;
volatile uint8_t command_steering_enable = 0U;
volatile uint8_t command_steering_reason = 0U;
volatile uint8_t command_steering_alive = 0U;
volatile uint32_t command_steering_rx_count = 0U;
volatile uint32_t command_steering_last_tick = 0U;
volatile uint8_t command_steering_timed_out = 1U;

/* Last decoded ADAS_SPEED_CMD (0x341). */
volatile float command_target_speed_kph = 0.0f;
volatile uint8_t command_deceleration_percent = 0U;
volatile uint8_t command_speed_enable = 0U;
volatile uint8_t command_speed_reason = 0U;
volatile uint8_t command_speed_alive = 0U;
volatile uint32_t command_speed_rx_count = 0U;
volatile uint32_t command_speed_last_tick = 0U;
volatile uint8_t command_speed_timed_out = 1U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void Gateway_CAN_Init(void);
static void Gateway_CAN_Process(void);
static void Gateway_CAN_ReceivePending(uint32_t now);
static void Gateway_CAN_TransmitSensors(uint32_t now);
static HAL_StatusTypeDef CAN_SendFrame(uint32_t std_id, uint32_t dlc,
                                       uint8_t data[CAN_MAX_DATA_LENGTH]);
static uint32_t CAN_ExtractIntel(const uint8_t *data, uint32_t start_bit,
                                 uint32_t bit_length);
static void CAN_InsertIntel(uint8_t *data, uint32_t start_bit,
                            uint32_t bit_length, uint32_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float ClampFloat(float value, float minimum, float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static uint32_t CAN_ExtractIntel(const uint8_t *data, uint32_t start_bit,
                                 uint32_t bit_length)
{
  uint32_t result = 0U;
  uint32_t bit;

  for (bit = 0U; bit < bit_length; bit++)
  {
    uint32_t absolute_bit = start_bit + bit;
    uint32_t value = (uint32_t)((data[absolute_bit / 8U] >>
                                 (absolute_bit % 8U)) & 1U);
    result |= value << bit;
  }
  return result;
}

static void CAN_InsertIntel(uint8_t *data, uint32_t start_bit,
                            uint32_t bit_length, uint32_t value)
{
  uint32_t bit;

  for (bit = 0U; bit < bit_length; bit++)
  {
    uint32_t absolute_bit = start_bit + bit;
    uint8_t mask = (uint8_t)(1U << (absolute_bit % 8U));
    if (((value >> bit) & 1U) != 0U)
    {
      data[absolute_bit / 8U] |= mask;
    }
    else
    {
      data[absolute_bit / 8U] &= (uint8_t)(~mask);
    }
  }
}

static HAL_StatusTypeDef CAN_SendFrame(uint32_t std_id, uint32_t dlc,
                                       uint8_t data[CAN_MAX_DATA_LENGTH])
{
  CAN_TxHeaderTypeDef header = {0};

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
  {
    can_tx_error_count++;
    can_last_hal_error = HAL_CAN_GetError(&hcan1);
    return HAL_BUSY;
  }

  header.StdId = std_id;
  header.ExtId = 0U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &can_tx_mailbox) != HAL_OK)
  {
    can_tx_error_count++;
    can_last_hal_error = HAL_CAN_GetError(&hcan1);
    return HAL_ERROR;
  }

  can_tx_count++;
  return HAL_OK;
}

static void Gateway_CAN_Init(void)
{
  CAN_FilterTypeDef filter = {0};

  /* 16-bit list filter: accept only custom commands 0x340 and 0x341. */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh = (uint16_t)(CAN_ID_STEERING_COMMAND << 5U);
  filter.FilterIdLow = (uint16_t)(CAN_ID_SPEED_COMMAND << 5U);
  filter.FilterMaskIdHigh = (uint16_t)(CAN_ID_STEERING_COMMAND << 5U);
  filter.FilterMaskIdLow = (uint16_t)(CAN_ID_SPEED_COMMAND << 5U);
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    can_last_hal_error = HAL_CAN_GetError(&hcan1);
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    can_last_hal_error = HAL_CAN_GetError(&hcan1);
    Error_Handler();
  }

  can_last_sensor_tick = HAL_GetTick();
}

static void Gateway_CAN_DecodeSteeringCommand(const uint8_t *data, uint32_t now)
{
  int16_t target_angle_raw = (int16_t)CAN_ExtractIntel(data, 0U, 16U);

  command_target_angle_deg = (float)target_angle_raw * 0.1f;
  command_torque_limit = (uint16_t)CAN_ExtractIntel(data, 16U, 11U);
  command_steering_enable = (uint8_t)CAN_ExtractIntel(data, 27U, 1U);
  command_steering_reason = (uint8_t)CAN_ExtractIntel(data, 28U, 4U);
  command_steering_alive = (uint8_t)CAN_ExtractIntel(data, 36U, 4U);
  command_steering_last_tick = now;
  command_steering_rx_count++;
  command_steering_timed_out = 0U;
}

static void Gateway_CAN_DecodeSpeedCommand(const uint8_t *data, uint32_t now)
{
  uint16_t target_speed_raw = (uint16_t)CAN_ExtractIntel(data, 0U, 14U);

  command_target_speed_kph = (float)target_speed_raw * 0.03125f;
  command_deceleration_percent = (uint8_t)CAN_ExtractIntel(data, 16U, 8U);
  command_speed_enable = (uint8_t)CAN_ExtractIntel(data, 24U, 1U);
  command_speed_reason = (uint8_t)CAN_ExtractIntel(data, 25U, 4U);
  command_speed_alive = (uint8_t)CAN_ExtractIntel(data, 32U, 4U);
  command_speed_last_tick = now;
  command_speed_rx_count++;
  command_speed_timed_out = 0U;
}

static void Gateway_CAN_ReceivePending(uint32_t now)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U)
  {
    uint8_t rx_data[CAN_MAX_DATA_LENGTH] = {0U};
    uint32_t index;

    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can_rx_header, rx_data) != HAL_OK)
    {
      can_rx_error_count++;
      can_last_hal_error = HAL_CAN_GetError(&hcan1);
      break;
    }

    can_rx_count++;
    can_last_rx_is_extended = (can_rx_header.IDE == CAN_ID_EXT) ? 1U : 0U;
    can_last_rx_id = (can_last_rx_is_extended != 0U) ?
                     can_rx_header.ExtId : can_rx_header.StdId;
    can_last_rx_dlc = (uint8_t)can_rx_header.DLC;

    for (index = 0U; index < CAN_MAX_DATA_LENGTH; index++)
    {
      can_last_rx_data[index] = rx_data[index];
    }

    if ((can_rx_header.IDE == CAN_ID_STD) && (can_rx_header.DLC == 8U))
    {
      if (can_rx_header.StdId == CAN_ID_STEERING_COMMAND)
      {
        Gateway_CAN_DecodeSteeringCommand(rx_data, now);
        BSP_LED_Toggle(LED2);
      }
      else if (can_rx_header.StdId == CAN_ID_SPEED_COMMAND)
      {
        Gateway_CAN_DecodeSpeedCommand(rx_data, now);
        BSP_LED_Toggle(LED2);
      }
    }
  }
}

static void Gateway_CAN_TransmitSensors(uint32_t now)
{
  float time_seconds;
  float base_speed;
  int32_t speed_fl_raw;
  int32_t speed_fr_raw;
  int32_t angle_raw;
  int32_t torque_raw;
  uint8_t wheel_data[CAN_MAX_DATA_LENGTH] = {0U};
  uint8_t angle_data[CAN_MAX_DATA_LENGTH] = {0U};
  uint8_t torque_data[CAN_MAX_DATA_LENGTH] = {0U};

  if ((uint32_t)(now - can_last_sensor_tick) < CAN_SENSOR_PERIOD_MS)
  {
    return;
  }
  can_last_sensor_tick = now;

  time_seconds = (float)now * 0.001f;

  /* Smooth, independent wave periods make the data look vehicle-like. */
  base_speed = 70.0f + 60.0f * sinf(TWO_PI_F * time_seconds / 120.0f);
  sim_wheel_speed_fl_kph = ClampFloat(base_speed, 0.0f, 511.96875f);
  sim_wheel_speed_fr_kph = ClampFloat(
      base_speed + 0.25f * sinf(TWO_PI_F * time_seconds / 11.0f),
      0.0f, 511.96875f);
  sim_steering_angle_deg =
      3.5f * sinf(TWO_PI_F * time_seconds / 18.0f) +
      1.5f * sinf(TWO_PI_F * time_seconds / 7.0f);
  sim_steering_column_torque =
      75.0f * sinf(TWO_PI_F * time_seconds / 13.0f) +
      25.0f * sinf(TWO_PI_F * time_seconds / 5.0f);

  speed_fl_raw = (int32_t)lroundf(sim_wheel_speed_fl_kph / 0.03125f);
  speed_fr_raw = (int32_t)lroundf(sim_wheel_speed_fr_kph / 0.03125f);
  angle_raw = (int32_t)lroundf(sim_steering_angle_deg / 0.1f);
  torque_raw = (int32_t)lroundf(sim_steering_column_torque + 1024.0f);

  speed_fl_raw = (int32_t)ClampFloat((float)speed_fl_raw, 0.0f, 16383.0f);
  speed_fr_raw = (int32_t)ClampFloat((float)speed_fr_raw, 0.0f, 16383.0f);
  torque_raw = (int32_t)ClampFloat((float)torque_raw, 0.0f, 2047.0f);

  CAN_InsertIntel(wheel_data, 0U, 14U, (uint32_t)speed_fl_raw);
  CAN_InsertIntel(wheel_data, 16U, 14U, (uint32_t)speed_fr_raw);
  CAN_InsertIntel(angle_data, 0U, 16U, (uint16_t)((int16_t)angle_raw));
  CAN_InsertIntel(torque_data, 0U, 11U, (uint32_t)torque_raw);

  (void)CAN_SendFrame(CAN_ID_WHEEL_SPEED, 8U, wheel_data);
  (void)CAN_SendFrame(CAN_ID_STEERING_ANGLE, 5U, angle_data);
  (void)CAN_SendFrame(CAN_ID_STEERING_TORQUE, 8U, torque_data);
}

static void Gateway_CAN_Process(void)
{
  uint32_t now = HAL_GetTick();

  Gateway_CAN_ReceivePending(now);
  Gateway_CAN_TransmitSensors(now);

  if ((command_steering_rx_count == 0U) ||
      ((uint32_t)(now - command_steering_last_tick) > CAN_COMMAND_TIMEOUT_MS))
  {
    command_steering_timed_out = 1U;
  }
  if ((command_speed_rx_count == 0U) ||
      ((uint32_t)(now - command_speed_last_tick) > CAN_COMMAND_TIMEOUT_MS))
  {
    command_speed_timed_out = 1U;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED2);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Gateway_CAN_Init();
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Gateway_CAN_Process();
  }
  /* USER CODE END 3 */
}

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

