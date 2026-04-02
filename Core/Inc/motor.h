/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机控制模块头文件
  *          提供电机初始化、速度控制、方向控制等功能接口
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  电机编号枚举
  */
typedef enum
{
  MOTOR_1 = 1,  /*!< 电机1 (1a端口) */
  MOTOR_2 = 2,  /*!< 电机2 (1b端口) */
  MOTOR_3 = 3,  /*!< 电机3 (2a端口) */
  MOTOR_4 = 4   /*!< 电机4 (2b端口) */
} Motor_ID_t;

/**
  * @brief  电机方向枚举
  */
typedef enum
{
  MOTOR_DIR_FORWARD  = 0,  /*!< 正转 */
  MOTOR_DIR_BACKWARD = 1   /*!< 反转 */
} Motor_Direction_t;

/**
  * @brief  电机状态结构体
  */
typedef struct
{
  int16_t speed;              /*!< 当前速度值 (-1000 ~ 1000) */
  Motor_Direction_t direction; /*!< 当前方向 */
  uint8_t isRunning;          /*!< 运行状态 (0:停止, 1:运行中) */
} Motor_Status_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  电机速度限制定义，这里是归一化的速度
  */
#define MOTOR_SPEED_MAX     1000   /*!< 最大速度值 */
#define MOTOR_SPEED_MIN    -1000   /*!< 最小速度值 (反向最大) */
#define MOTOR_SPEED_STOP      0    /*!< 停止速度 */

/**
  * @brief  PWM周期值 (与TIM3配置对应)
  */
#define MOTOR_PWM_PERIOD    65535  /*!< PWM自动重装载值 */

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  电机控制模块初始化
  * @retval None
  * @note   启动TIM3的PWM输出，初始化所有电机为停止状态
  */
void MotorControl_Init(void);

/**
  * @brief  设置指定电机的速度
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  speed: 速度值 (-1000 ~ 1000)
  *         正值: 正转, 负值: 反转, 0: 停止
  * @retval None
  * @note   速度值会自动限制在有效范围内
  */
void Motor_SetSpeed(Motor_ID_t motorId, int16_t speed);

/**
  * @brief  设置指定电机的方向
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  direction: 方向 (MOTOR_DIR_FORWARD 或 MOTOR_DIR_BACKWARD)
  * @retval None
  */
void Motor_SetDirection(Motor_ID_t motorId, Motor_Direction_t direction);

/**
  * @brief  设置指定电机的PWM占空比
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @param  dutyCycle: 占空比值 (0 ~ MOTOR_PWM_PERIOD)
  * @retval None
  * @note   如需设置速度百分比，建议使用 Motor_SetSpeed 函数
  */
void Motor_SetPWMDuty(Motor_ID_t motorId, uint32_t dutyCycle);

/**
  * @brief  停止指定电机
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval None
  */
void Motor_Stop(Motor_ID_t motorId);

/**
  * @brief  停止所有电机
  * @retval None
  */
void Motor_StopAll(void);

/**
  * @brief  获取指定电机的当前状态
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval Motor_Status_t: 电机状态结构体
  */
Motor_Status_t Motor_GetStatus(Motor_ID_t motorId);

/**
  * @brief  测试指定电机
  * @param  motorId: 电机编号 (MOTOR_1 ~ MOTOR_4)
  * @retval None
  * @note   执行正转-反转-停止的测试序列
  */
void Motor_Test(Motor_ID_t motorId);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
