/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "app_threadx.h"
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "stdarg.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SRAM_SET_D2        __attribute__((section(".RAM_D2")))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint32_t fishPrintf(uint8_t *buf, const char *str, ...) {
//    va_list ap;
//    va_start(ap, str);
//    uint32_t len = vsnprintf((char *) buf, 512, str, ap);
//    va_end(ap);
//    return len;
//}
//
//void CANFilterConfig(void) {
//    FDCAN_FilterTypeDef Filter;
//    Filter.IdType = FDCAN_STANDARD_ID;
//    Filter.FilterIndex = 0;
//    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//    Filter.FilterType = FDCAN_FILTER_MASK;
//    Filter.FilterID1 = 0x0000;
//    Filter.FilterID2 = 0x0000;
//
//    HAL_FDCAN_ConfigFilter(&hfdcan1, &Filter);
//    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
//    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//
//    Filter.IdType = FDCAN_STANDARD_ID;
//    Filter.FilterIndex = 0;
//    Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
//    Filter.FilterType = FDCAN_FILTER_MASK;
//    Filter.FilterID1 = 0x0000;
//    Filter.FilterID2 = 0x0000;
//
//    HAL_FDCAN_ConfigFilter(&hfdcan2, &Filter);
//    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
//    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
//
//    HAL_FDCAN_Start(&hfdcan1);
//    HAL_FDCAN_Start(&hfdcan2);
//}
//
//__PACKED_STRUCT  DM_Motor_t {
//    uint16_t pst;
//    int16_t rpm;
//    int16_t torque;
//    uint8_t tmp_coil;
//    uint8_t tmp_pcb;
//};
//
//SRAM_SET_D2 struct DM_Motor_t motor[4];
//SRAM_SET_D2 uint8_t RxData1[8];
//SRAM_SET_D2 uint8_t tx_buf[128];
//
//void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
//    FDCAN_RxHeaderTypeDef rx_header;
//    HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO1, &rx_header, RxData1);
//
//    switch (rx_header.Identifier) {
//        case 0x301:
//            motor[0].pst = RxData1[0] << 8 | RxData1[1];
//            motor[0].rpm = RxData1[2] << 8  | RxData1[3];
//            motor[0].torque = RxData1[4] << 8 | RxData1[5];
//            motor[0].tmp_coil = RxData1[6];
//            motor[0].tmp_pcb = RxData1[7];
//            break;
//        case 0x302:
//            motor[1].pst = RxData1[0] << 8 | RxData1[1];
//            motor[1].rpm = RxData1[2] << 8 | RxData1[3];
//            motor[1].torque = RxData1[4] << 8 | RxData1[5];
//            motor[1].tmp_coil = RxData1[6];
//            motor[1].tmp_pcb = RxData1[7];
//            break;
//        case 0x303:
//            motor[2].pst = RxData1[0] << 8 | RxData1[1];
//            motor[2].rpm = RxData1[2] << 8 | RxData1[3];
//            motor[2].torque = RxData1[4] << 8 | RxData1[5];
//            motor[2].tmp_coil = RxData1[6];
//            motor[2].tmp_pcb = RxData1[7];
//            break;
//        case 0x304:
//            motor[3].pst = RxData1[0] << 8 | RxData1[1];
//            motor[3].rpm = RxData1[2] << 8 | RxData1[3];
//            motor[3].torque = RxData1[4] << 8 | RxData1[5];
//            motor[3].tmp_coil = RxData1[6];
//            motor[3].tmp_pcb = RxData1[7];
//            break;
//        default:
//            break;
//    }
//}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();
  MX_SPI6_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_USART10_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
//
//    CANFilterConfig();
//    FDCAN_TxHeaderTypeDef TxHeader1 = {0};
//    TxHeader1.BitRateSwitch = FDCAN_BRS_OFF;
//    TxHeader1.FDFormat = FDCAN_CLASSIC_CAN;
//    TxHeader1.Identifier = 0x3FE;
//    TxHeader1.IdType = FDCAN_STANDARD_ID;
//    TxHeader1.DataLength = FDCAN_DLC_BYTES_8;
//    TxHeader1.TxFrameType = FDCAN_DATA_FRAME;
//
//    uint8_t data_tx[8] = {0};
//    uint8_t data_tx_zero[8] = {0};
//    data_tx[0] = (-100)>>8;
//    data_tx[1] = (-100)&0xFF;
//    data_tx[2] = (100)>>8;
//    data_tx[3] = (100)&0xFF;
//    data_tx[4] = (100)>>8;
//    data_tx[5] = (100)&0xFF;
//    data_tx[6] = (-100)>>8;
//    data_tx[7] = (-100)&0xFF;
  /* USER CODE END 2 */

  MX_ThreadX_Init();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//        uint16_t len = fishPrintf(tx_buf, "pst0=%d, rpm0=%d, tqr0=%d\r\n", motor[0].pst, motor[0].rpm, motor[0].torque);
//        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
//        HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
//        HAL_Delay(10);
//        HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//        len = fishPrintf(tx_buf, "pst1=%d, rpm1=%d, tqr1=%d\r\n", motor[1].pst, motor[1].rpm, motor[1].torque);
//        SCB_CleanInvalidateDCache_by_Addr((uint32_t *) tx_buf, len);
//        HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
//        HAL_Delay(10);
//        if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_RESET){
//            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader1, data_tx);
//        }else{
//            HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader1, data_tx_zero);
//        }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_USART10;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 16;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 3;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void TIM23_IRQHandler() {
    HAL_IncTick();
    LL_TIM_ClearFlag_UPDATE(TIM23);
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23) {
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
    while (1) {
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
