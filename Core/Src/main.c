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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"  /* 包含电机控制模块头文件 */
#include "crsf.h"   /* 包含CRSF协议接收模块头文件 */
#include <string.h>  /* 包含字符串处理函数 */
#include <stdio.h>   /* 包含标准输入输出函数 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON()    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_OFF()   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_TOGGLE() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* 电池电压变量 */
float batteryVoltage = 0.0f;
/* 调试用：原始ADC值 */
uint32_t debug_adc_value = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendJoystickData(void);
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* 初始化电机控制模块 */
  MotorControl_Init();
  
  /* 初始化CRSF协议接收模块 */
  CRSF_Init();
  
  /* 执行ADC自校准 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* 点亮LED指示灯，表示系统初始化完成 */
  LED_ON();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    
    /* 发送摇杆数据到USART1 */
    SendJoystickData();
    
    /* 将左摇杆Y轴数值映射到电机1转速 */
    /* crsf_data.Left_Y 范围: 0 ~ 100, 0=停止, 100=全速 */
    /* 电机速度范围: 0 ~ 1000 */
    int16_t motor1_speed = (int16_t)(crsf_data.Left_Y * 10.0f);
    Motor_SetSpeed(MOTOR_1, motor1_speed);
    
    /* 延时10ms，控制频率约100Hz */
    HAL_Delay(10);
    
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  读取电池电压
  * @param  无
  * @retval 电池电压值 (V)
  */
float ReadBatteryVoltage(void)
{
  uint32_t adc_sum = 0;
  float voltage = 0.0f;
  int sample_count = 10;
  
  /* 多次采样取平均 */
  for(int i = 0; i < sample_count; i++)
  {
    /* 启动ADC转换 */
    HAL_ADC_Start(&hadc1);
    
    /* 等待转换完成 */
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
      /* 累加ADC值 */
      adc_sum += HAL_ADC_GetValue(&hadc1);
    }
    
    /* 停止ADC转换 */
    HAL_ADC_Stop(&hadc1);
    
    /* 短延时，避免采样过快 */
    HAL_Delay(1);
  }
  
  /* 计算平均ADC值 */
  uint32_t adc_avg = adc_sum / sample_count;
  debug_adc_value = adc_avg;  /* 保存原始ADC值用于调试 */
  
  /* 计算电压值
   * 参考电压: 3.3V
   * ADC分辨率: 12位 (0-4095) - 过采样后实际为16位精度
   * 分压电阻: R1=27kΩ, R3=16.9kΩ
   * 电压计算公式: Vbat = (adc_value * 3.3V / 4095) * (27k + 16.9k) / 16.9k
   *                      = (adc_value * 3.3V / 4095) * 2.598
   */
  voltage = (float)adc_avg * 3.3f / 4095.0f * 2.598f;
  
  return voltage;
}

/**
  * @brief  发送摇杆数据到USART1
  * @param  无
  * @retval 无
  */
void SendJoystickData(void)
{
  char buffer[150];
  
  /* 读取电池电压 */
  batteryVoltage = ReadBatteryVoltage();
  
  /* 按照CSV格式格式化数据 - 添加原始ADC值用于调试 */
  sprintf(buffer, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%.2f,%lu\r\n",
          crsf_data.Left_X, crsf_data.Left_Y, crsf_data.Right_X, crsf_data.Right_Y,
          crsf_data.S1, crsf_data.S2,
          crsf_data.A, crsf_data.B, crsf_data.C, crsf_data.D, crsf_data.E, crsf_data.F,
          batteryVoltage, debug_adc_value);
  
  /* 通过USART1发送数据 */
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

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
