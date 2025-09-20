/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//滤波器编�????
#define CAN_FILTER(x) ((x) << 3)

//接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展�????
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

//数据帧或遥控�????
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Motor GM6020;

CAN_RxHeaderTypeDef header;
uint8_t CAN_Rx_Data[8];
uint16_t IT = 0;
uint8_t success = 0;
uint16_t testflag = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef tx_hander;
  uint32_t used_mailbox;

  assert_param(hcan != NULL);

  tx_hander.StdId = ID;
  tx_hander.ExtId = 0;
  tx_hander.IDE = CAN_ID_STD;
  tx_hander.RTR = CAN_RTR_DATA;
  tx_hander.DLC = Length;

  return HAL_CAN_AddTxMessage(hcan, &tx_hander, Data, &used_mailbox);
}

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef can_filter_init;
  
  assert_param(hcan != NULL);

  if(Object_Para & 0x02)
  {
    //数据�????
    can_filter_init.FilterIdHigh = ID << 3 << 16;
    can_filter_init.FilterIdLow = ID << 3 | ((Object_Para & 0x03) << 1);
    can_filter_init.FilterMaskIdHigh = Mask_ID << 3 <<16;
    can_filter_init.FilterMaskIdLow = Mask_ID << 3 | ((Object_Para & 0x03) << 1);
  }
  else
  {
    //遥控�????
    can_filter_init.FilterIdHigh = ID << 5;
    can_filter_init.FilterIdLow = ((Object_Para & 0x03) << 1);
    can_filter_init.FilterMaskIdHigh = Mask_ID << 5;
    can_filter_init.FilterMaskIdLow = ((Object_Para & 0x03) << 1);
  }
  can_filter_init.FilterBank = Object_Para >> 3;
  can_filter_init.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;
  can_filter_init.FilterActivation = ENABLE;
  can_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter_init.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &can_filter_init);
}

void Send_Motor_V(uint16_t ID, float voltage)
{
  if(voltage > 24.0f)  voltage = 24.0f;
  else if(voltage < -24.0f)  voltage = -24.0f;

  uint16_t raw_voltage = (uint16_t)(voltage * 1000.0f);
  uint8_t CAN_Tx_Data[8];
  CAN_Tx_Data[0] = (raw_voltage >> 8) & 0xff;
  CAN_Tx_Data[1] = raw_voltage & 0xff;

  CAN_Send_Data(&hcan1, ID, CAN_Tx_Data, 8);
}

void Motor_Feedback(uint8_t *data)
{
  GM6020.raw_angle = (data[0] << 8) | data[1];
  GM6020.raw_rpm = (data[2] << 8) | data[3];
  GM6020.raw_current = (data[4] << 8) | data[5];
  GM6020.raw_temperature = data[6];

  GM6020.current_angle = (GM6020.raw_angle / 8191.0f) * 360.0f;
  GM6020.current = (GM6020.raw_current / 16384.0f) * 3.0f;
  GM6020.rpm = GM6020.raw_rpm * 1.0f;

  GM6020.raw_current_angle = (data[0] << 8) | data[1];
  int16_t delta = GM6020.raw_current_angle - GM6020.raw_last_angle;

  if(delta > 4096)  
  {
    delta -= 8191;
    GM6020.circle_count--;
  }
  else if(delta < -4096)
  {
    delta += 8191;
    GM6020.circle_count++;
  }

  GM6020.total_angle += (delta / 8191.0f) * 360.0f;
  GM6020.raw_last_angle = GM6020.raw_current_angle;
 
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
  CAN_Init(&hcan1);
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(13) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_TYPE, 0x205, 0x7ff);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN1)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &header, CAN_Rx_Data);
    IT++;
    if(header.StdId == 0x205)
    {
      // IT++;
      Motor_Feedback(CAN_Rx_Data);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
