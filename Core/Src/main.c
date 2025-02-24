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
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* CS1引脚控制宏定义 */
#define SPI_CS1_LOW()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define SPI_CS1_HIGH()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)

/* CS2引脚控制宏定义 */
#define SPI_CS2_LOW()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define SPI_CS2_HIGH()      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static uint8_t Is_ADC_Data_Valid(uint32_t adc_value);
static float Convert_ADC_To_Voltage(uint32_t adc_value);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void UART_SendChar(const char* str);  // 串口发送函数声明


static uint8_t SPI_TransmitReceive(uint8_t data);
void SPI_Transmit(uint8_t data);



uint32_t adc_value = 0;
uint8_t msb = 0;  // 读取高8位
uint8_t mid = 0;  // 读取中8位
uint8_t lsb = 0;  // 读取低8位
/* ADS1220参考电压 */
#define VREF              2.048f    // 参考电压2.048V
#define ADC_FSR           8388608.0f // 2^23, ADC满量程范围


/* 将24位ADC数据转换为电压值 */
static float Convert_ADC_To_Voltage(uint32_t adc_value)
{
    int32_t signed_value;
    float voltage;
    
    // 如果是负数（最高位为1）
    if(adc_value & 0x800000) {
        signed_value = (int32_t)(adc_value | 0xFF000000);
    } else {
        signed_value = (int32_t)adc_value;
    }
    
    // 将补码转换为电压值
    voltage = ((float)signed_value * VREF) / ADC_FSR;
    
    return voltage;
}



/* 检查数据是否有效 */
static uint8_t Is_ADC_Data_Valid(uint32_t adc_value)
{
    // 检查是否有异常的高位（24位以上应该都是0或1）
    uint32_t high_bits = (adc_value >> 23) & 0x1FF;  // 检查高9位
    return (high_bits == 0x000 || high_bits == 0x1FF);  // 应该全0或全1
}

void SPI_Transmit(uint8_t data) {
    uint8_t txData[] = {data};  // 发送数据缓冲区
    uint8_t rxData[1];          // 接收数据缓冲区

    /* 同时发送和接收数据 */
    if(HAL_SPI_TransmitReceive(&hspi1, txData, rxData, sizeof(txData), 100) != HAL_OK)
    {
        Error_Handler();
    }

    /* 打印发送和接收的数据 */
    char str[50];
    sprintf(str, "SPI: TX=0x%02X, RX=0x%02X\r\n", txData[0], rxData[0]);
    UART_SendChar(str);
}

/* SPI发送接收函数 */
static uint8_t SPI_TransmitReceive(uint8_t data) {
    uint8_t txData[] = {data};  // 发送数据缓冲区
    uint8_t rxData[1];          // 接收数据缓冲区

    /* 同时发送和接收数据 */
    if(HAL_SPI_TransmitReceive(&hspi1, txData, rxData, sizeof(txData), 100) != HAL_OK)
    {
        Error_Handler();
    }

    /* 打印发送和接收的数据 */
    // char str[50];
    // sprintf(str, "SPI: TX=0x%02X, RX=0x%02X\r\n", txData[0], rxData[0]);
    // UART_SendChar(str);

    return rxData[0];
}

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  UART_SendChar("System Start!\r\n");
  HAL_Delay(1); 


  SPI_CS1_LOW();
      /* 发送复位命令 */
    SPI_Transmit(0x06);  // RESET命令

    HAL_Delay(1); 


    SPI_Transmit(0x43);  // WREG命令，写寄存器3
    SPI_Transmit(0x00);  // 配置数据 - 寄存器0：PGA=1, AIN0/AIN1
    SPI_Transmit(0xD4);  // 配置数据 - 寄存器1：DR=20SPS, 连续转换模式
    SPI_Transmit(0x10);  // 配置数据 - 寄存器2：IDAC关闭
    SPI_Transmit(0x00);  // 配置数据 - 寄存器3：默认设置
  HAL_Delay(1);

    SPI_Transmit(0x23);
      HAL_Delay(1);

    SPI_Transmit(0x08);  
    HAL_Delay(1);

  SPI_CS1_HIGH();

  HAL_Delay(1); 
    UART_SendChar("--------System Start!\r\n");

    MX_GPIO_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  // hspi1.Instance = SPI1;
  // hspi1.Init.Mode = SPI_MODE_MASTER;
  // hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  // hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  // hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  // hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  // hspi1.Init.NSS = SPI_NSS_SOFT;
  // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  // hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  // hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  // hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  // hspi1.Init.CRCPolynomial = 0x0;
  // hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  // hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  // hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  // hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  // hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  // hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  // hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  // hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  // hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  // hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;


    /* SPI1 参数配置 */
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;    // CPOL = 0
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;        // CPHA = 1
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;  // 确保SCLK周期>150ns
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;



  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }






  /* USER CODE BEGIN SPI1_Init 2 */
  /* CS引脚配置 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  




  /* CS1引脚配置 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* CS1默认高电平 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  
  /* CS2引脚配置 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* CS2默认高电平 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart4) != HAL_OK)  // 启用FIFO模式
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);



/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 串口发送函数定义
void UART_SendChar(const char* str)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)str, strlen(str), 100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_4)
 {

    UART_SendChar("---222t!\r\n");

    SPI_CS1_LOW();
    // HAL_Delay(1);
    uint32_t adc_value = 0;
    uint8_t msb = SPI_TransmitReceive(0xFF);  // 读取高8位
    uint8_t mid = SPI_TransmitReceive(0xFF);  // 读取中8位
    uint8_t lsb = SPI_TransmitReceive(0xFF);  // 读取低8位
    adc_value = (uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb;
    SPI_CS1_HIGH();

    if(Is_ADC_Data_Valid(adc_value))
    {
      float voltage = Convert_ADC_To_Voltage(adc_value);
                
      /* 将浮点数分解为整数部分和小数部分 */
      int32_t int_part = (int32_t)voltage;
      int32_t decimal_part = (int32_t)((voltage - int_part) * 1000000); // 保留6位小数
      if(decimal_part < 0) decimal_part = -decimal_part; // 确保小数部分为正

      /* 使用UART发送ADC和电压值 */
      char uart_buffer[64];
      sprintf(uart_buffer, "ADC Raw: 0x%06lX (%ld), Voltage: %ld.%06ld V\r\n", 
              adc_value, (int32_t)adc_value, int_part, decimal_part);
      UART_SendChar(uart_buffer);

    }
    else
    {
        UART_SendChar("Invalid ADC data.\r\n");
    }


    // /* 读取24位ADC数据 */
    // uint32_t adc_value = 0;
    // uint8_t msb = SPI_TransmitReceive(0xFF);  // 读取高8位
    // uint8_t mid = SPI_TransmitReceive(0xFF);  // 读取中8位
    // uint8_t lsb = SPI_TransmitReceive(0xFF);  // 读取低8位
    // adc_value = (uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb;
    // SPI_CS1_HIGH();

    // /* 打印ADC值 */
    // char uart_buffer[32];
    // sprintf(uart_buffer, "ADC Value: %lu\r\n", adc_value);
    // UART_SendChar(uart_buffer);
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
