/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机控制模块源文件
  *          实现电机初始化、速度控制、方向控制等功能
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
#include "motor.h"
#include <stdlib.h>  /* 用于abs函数 */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  电机状态数组，用于存储4个电机的当前状态
  */
static Motor_Status_t motorStatus[4] = {
  {0, MOTOR_DIR_FORWARD, 0},  /* 电机1初始状态 */
  {0, MOTOR_DIR_FORWARD, 0},  /* 电机2初始状态 */
  {0, MOTOR_DIR_FORWARD, 0},  /* 电机3初始状态 */
  {0, MOTOR_DIR_FORWARD, 0}   /* 电机4初始状态 */
};

/* Private function prototypes -----------------------------------------------*/

/**
  * @brief  获取电机对应的TIM通道
  * @param  motorId: 电机编号
  * @retval uint32_t: TIM通道
  */
static uint32_t Motor_GetTIMChannel(Motor_ID_t motorId);

/**
  * @brief  设置电机方向GPIO
  * @param  motorId: 电机编号
  * @param  direction: 方向
  * @retval None
  */
static void Motor_SetDirectionGPIO(Motor_ID_t motorId, Motor_Direction_t direction);

/**
  * @brief  限制速度值在有效范围内
  * @param  speed: 输入速度值
  * @retval int16_t: 限制后的速度值
  */
static int16_t Motor_LimitSpeed(int16_t speed);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  获取电机对应的TIM通道
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval uint32_t: TIM通道 (TIM_CHANNEL_1 ~ TIM_CHANNEL_4)
  * @note   私有函数，仅内部使用
  */
static uint32_t Motor_GetTIMChannel(Motor_ID_t motorId)
{
  switch (motorId)
  {
    case MOTOR_1:
      return TIM_CHANNEL_1;  /* 电机1对应TIM3通道1 */
    case MOTOR_2:
      return TIM_CHANNEL_2;  /* 电机2对应TIM3通道2 */
    case MOTOR_3:
      return TIM_CHANNEL_3;  /* 电机3对应TIM3通道3 */
    case MOTOR_4:
      return TIM_CHANNEL_4;  /* 电机4对应TIM3通道4 */
    default:
      return TIM_CHANNEL_1;  /* 默认返回通道1 */
  }
}

/**
  * @brief  设置电机方向GPIO
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  direction: 方向 (MOTOR_DIR_FORWARD 或 MOTOR_DIR_BACKWARD)
  * @retval None
  * @note   私有函数，仅内部使用
  *         通过设置方向引脚电平控制电机转向
  */
static void Motor_SetDirectionGPIO(Motor_ID_t motorId, Motor_Direction_t direction)
{
  GPIO_PinState pinState = (direction == MOTOR_DIR_FORWARD) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  
  switch (motorId)
  {
    case MOTOR_1:
      /* 电机1方向控制: PA8 (DIR1) */
      HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, pinState);
      break;
      
    case MOTOR_2:
      /* 电机2方向控制: PA11 (DIR2) */
      HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, pinState);
      break;
      
    case MOTOR_3:
      /* 电机3方向控制: PA12 (DIR3) */
      HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, pinState);
      break;
      
    case MOTOR_4:
      /* 电机4方向控制: PA15 (DIR4) */
      HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, pinState);
      break;
      
    default:
      break;
  }
}

/**
  * @brief  限制速度值在有效范围内
  * @param  speed: 输入速度值
  * @retval int16_t: 限制后的速度值
  * @note   私有函数，仅内部使用
  *         确保速度值在 MOTOR_SPEED_MIN 和 MOTOR_SPEED_MAX 之间
  */
static int16_t Motor_LimitSpeed(int16_t speed)
{
  if (speed > MOTOR_SPEED_MAX)
  {
    return MOTOR_SPEED_MAX;
  }
  else if (speed < MOTOR_SPEED_MIN)
  {
    return MOTOR_SPEED_MIN;
  }
  return speed;
}

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  电机控制模块初始化
  * @retval None
  * @note   启动TIM3的4个PWM通道，初始化所有电机为停止状态
  *         应在系统初始化完成后调用
  */
void MotorControl_Init(void)
{
  /* 启动TIM3的4个PWM通道 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  /* 电机1 PWM */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  /* 电机2 PWM */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  /* 电机3 PWM */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  /* 电机4 PWM */
  
  /* 初始化所有电机状态为停止 */
  for (uint8_t i = 0; i < 4; i++)
  {
    motorStatus[i].speed = 0;
    motorStatus[i].direction = MOTOR_DIR_FORWARD;
    motorStatus[i].isRunning = 0;
  }
  
  /* 设置所有电机PWM占空比为0 (停止状态) */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/**
  * @brief  设置指定电机的速度
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  speed: 速度值 (-1000 ~ 1000)
  *         正值: 正转, 负值: 反转, 0: 停止
  * @retval None
  * @note   速度值会自动限制在有效范围内
  *         内部自动计算PWM占空比并设置方向
  */
void Motor_SetSpeed(Motor_ID_t motorId, int16_t speed)
{
  /* 参数有效性检查 */
  if (motorId < MOTOR_1 || motorId > MOTOR_4)
  {
    return;  /* 无效的电机编号，直接返回 */
  }
  
  /* 限制速度值在有效范围内 */
  int16_t limitedSpeed = Motor_LimitSpeed(speed);
  
  /* 确定方向 */
  Motor_Direction_t direction;
  uint16_t absSpeed;
  
  if (limitedSpeed >= 0)
  {
    direction = MOTOR_DIR_FORWARD;  /* 正转 */
    absSpeed = (uint16_t)limitedSpeed;
  }
  else
  {
    direction = MOTOR_DIR_BACKWARD; /* 反转 */
    absSpeed = (uint16_t)(-limitedSpeed);
  }
  
  /* 计算PWM占空比: (绝对速度 / 1000) * PWM周期 */
  uint32_t dutyCycle = (uint32_t)(((float)absSpeed / 1000.0f) * MOTOR_PWM_PERIOD);
  
  /* 设置方向 */
  Motor_SetDirectionGPIO(motorId, direction);
  
  /* 设置PWM占空比 */
  uint32_t channel = Motor_GetTIMChannel(motorId);
  __HAL_TIM_SET_COMPARE(&htim3, channel, dutyCycle);
  
  /* 更新电机状态 */
  uint8_t index = (uint8_t)(motorId - 1);
  motorStatus[index].speed = limitedSpeed;
  motorStatus[index].direction = direction;
  motorStatus[index].isRunning = (limitedSpeed != 0) ? 1 : 0;
}

/**
  * @brief  设置指定电机的方向
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  direction: 方向 (MOTOR_DIR_FORWARD 或 MOTOR_DIR_BACKWARD)
  * @retval None
  * @note   仅设置方向，不改变速度
  *         如需同时设置速度和方向，建议使用 Motor_SetSpeed 函数
  */
void Motor_SetDirection(Motor_ID_t motorId, Motor_Direction_t direction)
{
  /* 参数有效性检查 */
  if (motorId < MOTOR_1 || motorId > MOTOR_4)
  {
    return;
  }
  
  /* 设置方向GPIO */
  Motor_SetDirectionGPIO(motorId, direction);
  
  /* 更新状态 */
  uint8_t index = (uint8_t)(motorId - 1);
  motorStatus[index].direction = direction;
}

/**
  * @brief  设置指定电机的PWM占空比
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  dutyCycle: 占空比值 (0 ~ MOTOR_PWM_PERIOD)
  * @retval None
  * @note   如需设置速度百分比，建议使用 Motor_SetSpeed 函数
  *         此函数直接操作PWM寄存器，不改变方向设置
  */
void Motor_SetPWMDuty(Motor_ID_t motorId, uint32_t dutyCycle)
{
  /* 参数有效性检查 */
  if (motorId < MOTOR_1 || motorId > MOTOR_4)
  {
    return;
  }
  
  /* 限制占空比范围 */
  if (dutyCycle > MOTOR_PWM_PERIOD)
  {
    dutyCycle = MOTOR_PWM_PERIOD;
  }
  
  /* 设置PWM占空比 */
  uint32_t channel = Motor_GetTIMChannel(motorId);
  __HAL_TIM_SET_COMPARE(&htim3, channel, dutyCycle);
  
  /* 更新状态 */
  uint8_t index = (uint8_t)(motorId - 1);
  motorStatus[index].isRunning = (dutyCycle != 0) ? 1 : 0;
}

/**
  * @brief  停止指定电机
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval None
  * @note   将电机速度设为0，PWM占空比设为0
  */
void Motor_Stop(Motor_ID_t motorId)
{
  Motor_SetSpeed(motorId, 0);
}

/**
  * @brief  停止所有电机
  * @retval None
  * @note   同时停止4个电机，将所有PWM占空比设为0
  */
void Motor_StopAll(void)
{
  Motor_SetSpeed(MOTOR_1, 0);
  Motor_SetSpeed(MOTOR_2, 0);
  Motor_SetSpeed(MOTOR_3, 0);
  Motor_SetSpeed(MOTOR_4, 0);
}

/**
  * @brief  获取指定电机的当前状态
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval Motor_Status_t: 电机状态结构体
  * @note   返回包含速度、方向和运行状态的结构体
  */
Motor_Status_t Motor_GetStatus(Motor_ID_t motorId)
{
  Motor_Status_t status = {0, MOTOR_DIR_FORWARD, 0};
  
  if (motorId >= MOTOR_1 && motorId <= MOTOR_4)
  {
    uint8_t index = (uint8_t)(motorId - 1);
    status = motorStatus[index];
  }
  
  return status;
}

/**
  * @brief  测试指定电机
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval None
  * @note   执行以下测试序列:
  *         1. 正转50%速度，持续2秒
  *         2. 反转50%速度，持续2秒
  *         3. 停止，持续1秒
  *         使用HAL_Delay实现延时，会阻塞程序运行
  */
void Motor_Test(Motor_ID_t motorId)
{
  /* 正转测试: 50%速度，持续2秒 */
  Motor_SetSpeed(motorId, 500);
  HAL_Delay(2000);
  
  /* 反转测试: 50%速度，持续2秒 */
  Motor_SetSpeed(motorId, -500);
  HAL_Delay(2000);
  
  /* 停止测试: 停止1秒 */
  Motor_SetSpeed(motorId, 0);
  HAL_Delay(1000);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
