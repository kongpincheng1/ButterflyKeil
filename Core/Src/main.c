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
/* 霍尔传感器变量 */
float hallSensorValue = 0.0f;      /* 第一个霍尔传感器 */
float hallSensorValue2 = 0.0f;     /* 第二个霍尔传感器 */
/* 调试用：原始ADC值 */
uint32_t debug_adc_value = 0;
uint32_t debug_hall_adc_value = 0;
uint32_t debug_hall_adc_value2 = 0;
/* 霍尔传感器统计数据 */
float hall_min = 4095.0f;
float hall_max = 0.0f;
float hall_sum_total = 0.0f;
int hall_count = 0;
float hall_avg_total = 0.0f;

/* 电机控制参数 - 可配置 */
float hall_threshold_low = 1.2f;    /* 低位阈值（V） */
float hall_threshold_high = 2.2f;   /* 高位阈值（V） */

/* 电机4霍尔传感器参数 - 独立配置 */
float hall_threshold_low_m4 = 1.3f;    /* 电机4低位阈值（V） */
float hall_threshold_high_m4 = 2.3f;   /* 电机4高位阈值（V） */

/* 定义运行状态 */
typedef enum {
    FLAP_UP,
    FLAP_DOWN
} FlapState_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_limit;
    float integral_limit;
} SyncPid_t;

static FlapState_t current_state = FLAP_UP;
static FlapState_t current_state_m4 = FLAP_DOWN;
float k_comp;
float k_comp_m4;
float stand_wing_target_m1 = 2.20f;      /* 立翅目标霍尔电压，可按实测调整 */
float stand_wing_target_m4 = 1.00f;      /* 立翅目标霍尔电压，可按实测调整 */
float stand_wing_deadband = 0.03f;       /* 立翅保持死区 */
float stand_wing_kp = 1800.0f;           /* 立翅位置控制比例系数 */
int16_t stand_wing_min_speed = 120;      /* 立翅最小驱动速度，克服静摩擦 */
int16_t stand_wing_max_speed = 420;      /* 立翅最大驱动速度 */
static uint8_t stand_wing_mode_active = 0;
static float sync_phase_error = 0.0f;
static float sync_pid_output = 0.0f;
static float wing_phase_left = 0.0f;
static float wing_phase_right = 0.0f;
static int16_t sync_speed_motor1 = 0;
static int16_t sync_speed_motor4 = 0;
static uint32_t crsf_rx_recover_tick = 0;

static SyncPid_t flap_sync_pid = {
    .kp = 420.0f,
    .ki = 10.0f,
    .kd = 90.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output_limit = 220.0f,
    .integral_limit = 0.30f
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SendJoystickData(void);
static void InitFlapStates(void);
void Flap_Control_Logic(int16_t base_speed);
void Flap_Control_Logic_Motor4(int16_t base_speed);
static void SyncPid_Reset(SyncPid_t *pid);
static void UpdateStandWingControl(void);
static int16_t CalcStandWingSpeed(float hall_value, float target_value, int8_t hall_polarity);
static void UpdateFlapSyncSpeed(int16_t base_speed, int16_t *speed_m1, int16_t *speed_m4);
void ReadSensors(void);
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
  
  /* 初始化电机控制参数 */
  k_comp=(hall_threshold_high-hall_threshold_low)*0.4*0.5/1000.0f ;
  k_comp_m4=(hall_threshold_high_m4-hall_threshold_low_m4)*0.4*0.5/1000.0f ;

  /* 根据上电时霍尔位置初始化两个翅膀的拍动方向 */
  ReadSensors();
  InitFlapStates();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((HAL_GetTick() - CRSF_GetLastRxTick()) > 500U &&
        (HAL_GetTick() - crsf_rx_recover_tick) > 200U)
    {
      CRSF_RestartRx();
      crsf_rx_recover_tick = HAL_GetTick();
    }

    ReadSensors();
    
    if (crsf_data.F == 1) {
      if (stand_wing_mode_active == 0) {
        stand_wing_mode_active = 1;
        SyncPid_Reset(&flap_sync_pid);
      }
      UpdateStandWingControl();
    } else {
      if (stand_wing_mode_active == 1) {
        stand_wing_mode_active = 0;
        InitFlapStates();
      }

      /* 基于霍尔传感器值的扑翼控制 */
      /* 当霍尔传感器值在低位死区上限和高位死区下限之间时，电机正转 */
      /* 当霍尔传感器值低于低位阈值或高于高位阈值时，电机反转 */
      /* 当霍尔传感器值在死区范围内时，保持当前方向不变 */
      /* 速度由摇杆控制 */
      int16_t base_speed = (int16_t)(crsf_data.Left_Y * 10.0f);
      UpdateFlapSyncSpeed(base_speed, &sync_speed_motor1, &sync_speed_motor4);
      Flap_Control_Logic(sync_speed_motor1);
      Flap_Control_Logic_Motor4(sync_speed_motor4);
    }
    /* 发送摇杆数据到USART1 */
    SendJoystickData();
    /* 延时1ms，控制频率约200Hz */
    HAL_Delay(1);
    
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
  * @brief  读取电池电压和霍尔传感器值
  * @param  无
  * @retval 无
  */
void ReadSensors(void)
{
  uint32_t adc_buf[3] = {0};

  HAL_ADC_Start(&hadc1);

  // 等待并读取通道2（电池）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_buf[0] = HAL_ADC_GetValue(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC); // 清标志
  }

  // 等待并读取通道3（霍尔1）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_buf[1] = HAL_ADC_GetValue(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC); // 清标志
  }

  // 等待并读取通道4（霍尔2）
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    adc_buf[2] = HAL_ADC_GetValue(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC); // 清标志
  }

  HAL_ADC_Stop(&hadc1);

  debug_adc_value = adc_buf[0];
  debug_hall_adc_value = adc_buf[1];
  debug_hall_adc_value2 = adc_buf[2];

  batteryVoltage = (float)adc_buf[0] * 3.3f / 4095.0f * 2.598f;
  hallSensorValue = (float)adc_buf[1] * 3.3f / 4095.0f;
  hallSensorValue2 = (float)adc_buf[2] * 3.3f / 4095.0f;
}
/**
  * @brief  发送摇杆数据到USART1
  * @param  无
  * @retval 无
  */
void SendJoystickData(void)
{
  char buffer[150];
  
  
  /* 按照CSV格式格式化数据 - 添加原始ADC值和霍尔传感器数据用于调试 */
  sprintf(buffer, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%.2f,%lu,%.3f,%lu,%.3f,%lu,%.3f,%.3f,%.3f,%d,%d\r\n",
          crsf_data.Left_X, crsf_data.Left_Y, crsf_data.Right_X, crsf_data.Right_Y,
          crsf_data.S1, crsf_data.S2,
          crsf_data.A, crsf_data.B, crsf_data.C, crsf_data.D, crsf_data.E, crsf_data.F,
          batteryVoltage, debug_adc_value,
          hallSensorValue, debug_hall_adc_value,
          hallSensorValue2, debug_hall_adc_value2,
          sync_phase_error, wing_phase_left, wing_phase_right,
          current_state, current_state_m4);
  
  /* 使用非阻塞方式发送数据，设置较短的超时时间 */
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 10);
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t ClampMotorSpeed(int32_t speed)
{
    if (speed < 0) {
        return 0;
    }
    if (speed > 1000) {
        return 1000;
    }
    return (int16_t)speed;
}

static int16_t ClampSignedMotorSpeed(int32_t speed)
{
    if (speed < -1000) {
        return -1000;
    }
    if (speed > 1000) {
        return 1000;
    }
    return (int16_t)speed;
}

static float WrapPhaseError(float error)
{
    if (error > 0.5f) {
        return error - 1.0f;
    }
    if (error < -0.5f) {
        return error + 1.0f;
    }
    return error;
}

static float WrapPhase01(float phase)
{
    if (phase >= 1.0f) {
        return phase - 1.0f;
    }
    if (phase < 0.0f) {
        return phase + 1.0f;
    }
    return phase;
}

static void SyncPid_Reset(SyncPid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

static float SyncPid_Update(SyncPid_t *pid, float error)
{
    float derivative = error - pid->prev_error;

    pid->integral += error;
    pid->integral = ClampFloat(pid->integral, -pid->integral_limit, pid->integral_limit);

    pid->prev_error = error;

    return ClampFloat((pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative),
                      -pid->output_limit,
                      pid->output_limit);
}

static float GetWingPhase(float hall_value, float threshold_low, float threshold_high, FlapState_t state)
{
    float span = threshold_high - threshold_low;
    float normalized;

    if (span <= 0.01f) {
        return 0.0f;
    }

    normalized = ClampFloat((hall_value - threshold_low) / span, 0.0f, 1.0f);

    if (state == FLAP_UP) {
        return normalized * 0.5f;
    }

    return 0.5f + ((1.0f - normalized) * 0.5f);
}

static void InitFlapStates(void)
{
    float hall_mid = (hall_threshold_low + hall_threshold_high) * 0.5f;
    float hall_mid_m4 = (hall_threshold_low_m4 + hall_threshold_high_m4) * 0.5f;

    current_state = (hallSensorValue >= hall_mid) ? FLAP_DOWN : FLAP_UP;
    current_state_m4 = (hallSensorValue2 >= hall_mid_m4) ? FLAP_DOWN : FLAP_UP;

    SyncPid_Reset(&flap_sync_pid);
}

static int16_t CalcStandWingSpeed(float hall_value, float target_value, int8_t hall_polarity)
{
    float error = target_value - hall_value;
    float abs_error = (error >= 0.0f) ? error : -error;
    int32_t command;

    if (abs_error <= stand_wing_deadband) {
        return 0;
    }

    command = (int32_t)(error * stand_wing_kp);

    if (command > 0 && command < stand_wing_min_speed) {
        command = stand_wing_min_speed;
    } else if (command < 0 && command > -stand_wing_min_speed) {
        command = -stand_wing_min_speed;
    }

    if (command > stand_wing_max_speed) {
        command = stand_wing_max_speed;
    } else if (command < -stand_wing_max_speed) {
        command = -stand_wing_max_speed;
    }

    return ClampSignedMotorSpeed((int32_t)hall_polarity * command);
}

static void UpdateStandWingControl(void)
{
    int16_t speed_m1 = CalcStandWingSpeed(hallSensorValue, stand_wing_target_m1, 1);
    int16_t speed_m4 = CalcStandWingSpeed(hallSensorValue2, stand_wing_target_m4, -1);

    sync_phase_error = 0.0f;
    sync_pid_output = 0.0f;
    wing_phase_left = GetWingPhase(hallSensorValue, hall_threshold_low, hall_threshold_high, current_state);
    wing_phase_right = GetWingPhase(hallSensorValue2, hall_threshold_low_m4, hall_threshold_high_m4, current_state_m4);

    Motor_SetSpeed(MOTOR_1, speed_m1);
    Motor_SetSpeed(MOTOR_4, speed_m4);
}

static void UpdateFlapSyncSpeed(int16_t base_speed, int16_t *speed_m1, int16_t *speed_m4)
{
    const int16_t sync_enable_speed = 80;
    const float sync_deadband = 0.01f;
    int16_t speed_abs = (base_speed >= 0) ? base_speed : (int16_t)(-base_speed);
    float abs_phase_error;
    float target_right_phase;

    wing_phase_left = GetWingPhase(hallSensorValue, hall_threshold_low, hall_threshold_high, current_state);
    wing_phase_right = GetWingPhase(hallSensorValue2, hall_threshold_low_m4, hall_threshold_high_m4, current_state_m4);

    if (speed_abs < sync_enable_speed) {
        SyncPid_Reset(&flap_sync_pid);
        sync_phase_error = 0.0f;
        sync_pid_output = 0.0f;
        *speed_m1 = speed_abs;
        *speed_m4 = speed_abs;
        return;
    }

    /* 蝴蝶双翼需要反相同步：右翼目标相位始终比左翼超前半个周期。 */
    target_right_phase = WrapPhase01(wing_phase_left + 0.5f);
    sync_phase_error = WrapPhaseError(target_right_phase - wing_phase_right);
    abs_phase_error = (sync_phase_error >= 0.0f) ? sync_phase_error : -sync_phase_error;

    if (abs_phase_error < sync_deadband) {
        sync_phase_error = 0.0f;
    }

    sync_pid_output = SyncPid_Update(&flap_sync_pid, sync_phase_error);

    /* 误差为正说明右翼落后于反相目标，需要加快右翼并稍微减慢左翼。 */
    *speed_m1 = ClampMotorSpeed((int32_t)speed_abs - (int32_t)sync_pid_output);
    *speed_m4 = ClampMotorSpeed((int32_t)speed_abs + (int32_t)sync_pid_output);
}

void Flap_Control_Logic(int16_t speed)
{
    static int filter_count = 0;
    const int FILTER_THRESHOLD = 2; // 略微减小滤波，提高响应速度

    // 1. 计算绝对速度，用于线性补偿
    uint16_t abs_speed = (speed > 0) ? speed : -speed;

    // 2. 计算动态阈值 (线性缩减边界)
    // 假设 speed 最大为 1000，k_comp 为 0.0005，则最大偏移量为 0.5V
    float dyn_threshold_high = hall_threshold_high - (k_comp * (float)abs_speed);
    float dyn_threshold_low = hall_threshold_low + (k_comp * (float)abs_speed);


    switch (current_state)
    {
        case FLAP_UP:
            Motor_SetSpeed(MOTOR_1, speed);
            // 使用动态高位阈值
            if (hallSensorValue > dyn_threshold_high) {
                filter_count++;
                if(filter_count >= FILTER_THRESHOLD) {
                    current_state = FLAP_DOWN;
                    filter_count = 0;
                }
            } else {
                filter_count = 0;
            }
            break;
            
        case FLAP_DOWN:
            Motor_SetSpeed(MOTOR_1, -speed);
            // 使用动态低位阈值
            if (hallSensorValue < dyn_threshold_low) {
                filter_count++;
                if(filter_count >= FILTER_THRESHOLD) {
                    current_state = FLAP_UP;
                    filter_count = 0;
                }
            } else {
                filter_count = 0;
            }
            break;
    }
}

void Flap_Control_Logic_Motor4(int16_t speed)
{
    static int filter_count_m4 = 0;
    const int FILTER_THRESHOLD_M4 = 2;

    uint16_t abs_speed_m4 = (speed > 0) ? speed : -speed;

    float dyn_threshold_high_m4 = hall_threshold_high_m4 - (k_comp_m4 * (float)abs_speed_m4);
    float dyn_threshold_low_m4 = hall_threshold_low_m4 + (k_comp_m4 * (float)abs_speed_m4);


    switch (current_state_m4)
    {
        case FLAP_UP:
            Motor_SetSpeed(MOTOR_4, -speed);
            if (hallSensorValue2 > dyn_threshold_high_m4) {
                filter_count_m4++;
                if(filter_count_m4 >= FILTER_THRESHOLD_M4) {
                    current_state_m4 = FLAP_DOWN;
                    filter_count_m4 = 0;
                }
            } else {
                filter_count_m4 = 0;
            }
            break;

        case FLAP_DOWN:
            Motor_SetSpeed(MOTOR_4, speed);
            if (hallSensorValue2 < dyn_threshold_low_m4) {
                filter_count_m4++;
                if(filter_count_m4 >= FILTER_THRESHOLD_M4) {
                    current_state_m4 = FLAP_UP;
                    filter_count_m4 = 0;
                }
            } else {
                filter_count_m4 = 0;
            }
            break;
    }
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
