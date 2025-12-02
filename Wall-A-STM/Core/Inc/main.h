/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ORANGE_Pin GPIO_PIN_2
#define LED_ORANGE_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOE
#define USER_SWITCH_1_Pin GPIO_PIN_15
#define USER_SWITCH_1_GPIO_Port GPIOC
#define USER_SWITCH_2_Pin GPIO_PIN_9
#define USER_SWITCH_2_GPIO_Port GPIOI
#define USER_SWITCH_4_Pin GPIO_PIN_11
#define USER_SWITCH_4_GPIO_Port GPIOI
#define PRI_MOTOR_SLEEP_Pin GPIO_PIN_1
#define PRI_MOTOR_SLEEP_GPIO_Port GPIOF
#define PRI_MOTOR_NFAULT_Pin GPIO_PIN_2
#define PRI_MOTOR_NFAULT_GPIO_Port GPIOF
#define ADC_PRI_MOTOR_TEMPERATURE_Pin GPIO_PIN_6
#define ADC_PRI_MOTOR_TEMPERATURE_GPIO_Port GPIOF
#define ADC_SEC_MOTOR_TEMPERATURE_Pin GPIO_PIN_7
#define ADC_SEC_MOTOR_TEMPERATURE_GPIO_Port GPIOF
#define ADC_POWER_SUPPLIES_MOTOR_TEMPERATURE_Pin GPIO_PIN_8
#define ADC_POWER_SUPPLIES_MOTOR_TEMPERATURE_GPIO_Port GPIOF
#define ADC_SENSOR_2_Pin GPIO_PIN_0
#define ADC_SENSOR_2_GPIO_Port GPIOC
#define ADC_SENSOR_1_Pin GPIO_PIN_2
#define ADC_SENSOR_1_GPIO_Port GPIOC
#define ADC_SENSOR_3_Pin GPIO_PIN_3
#define ADC_SENSOR_3_GPIO_Port GPIOC
#define ADC_PRI_MOTOR_CURRENT_1_Pin GPIO_PIN_3
#define ADC_PRI_MOTOR_CURRENT_1_GPIO_Port GPIOA
#define ADC_PRI_MOTOR_CURRENT_2_Pin GPIO_PIN_4
#define ADC_PRI_MOTOR_CURRENT_2_GPIO_Port GPIOA
#define SERVO_PWM1_Pin GPIO_PIN_5
#define SERVO_PWM1_GPIO_Port GPIOA
#define ADC_SEC_MOTOR_CURRENT_1_Pin GPIO_PIN_6
#define ADC_SEC_MOTOR_CURRENT_1_GPIO_Port GPIOA
#define ADC_SEC_MOTOR_CURRENT_2_Pin GPIO_PIN_0
#define ADC_SEC_MOTOR_CURRENT_2_GPIO_Port GPIOB
#define ADC_SENSOR_1B1_Pin GPIO_PIN_1
#define ADC_SENSOR_1B1_GPIO_Port GPIOB
#define SWITCH_4_Pin GPIO_PIN_2
#define SWITCH_4_GPIO_Port GPIOB
#define SWITCH_3_Pin GPIO_PIN_11
#define SWITCH_3_GPIO_Port GPIOF
#define SWITCH_2_Pin GPIO_PIN_12
#define SWITCH_2_GPIO_Port GPIOF
#define SWITCH_1_Pin GPIO_PIN_13
#define SWITCH_1_GPIO_Port GPIOF
#define SENSOR_PULSE_3_Pin GPIO_PIN_14
#define SENSOR_PULSE_3_GPIO_Port GPIOF
#define SENSOR_PULSE_1_Pin GPIO_PIN_15
#define SENSOR_PULSE_1_GPIO_Port GPIOF
#define SENSOR_PULSE_2_Pin GPIO_PIN_0
#define SENSOR_PULSE_2_GPIO_Port GPIOG
#define SENSOR_PULSE_4_Pin GPIO_PIN_1
#define SENSOR_PULSE_4_GPIO_Port GPIOG
#define WHEEL_PWM1_Pin GPIO_PIN_9
#define WHEEL_PWM1_GPIO_Port GPIOE
#define WHEEL_PWM2_Pin GPIO_PIN_11
#define WHEEL_PWM2_GPIO_Port GPIOE
#define WHEEL_PWM3_Pin GPIO_PIN_13
#define WHEEL_PWM3_GPIO_Port GPIOE
#define WHEEL_PWM4_Pin GPIO_PIN_14
#define WHEEL_PWM4_GPIO_Port GPIOE
#define SERVO_PWM3_Pin GPIO_PIN_10
#define SERVO_PWM3_GPIO_Port GPIOB
#define SERVO_PWM4_Pin GPIO_PIN_11
#define SERVO_PWM4_GPIO_Port GPIOB
#define SECONDARY_MOTOR_PWM1_Pin GPIO_PIN_10
#define SECONDARY_MOTOR_PWM1_GPIO_Port GPIOH
#define SECONDARY_MOTOR_PWM2_Pin GPIO_PIN_11
#define SECONDARY_MOTOR_PWM2_GPIO_Port GPIOH
#define PGOOD_24V_Pin GPIO_PIN_14
#define PGOOD_24V_GPIO_Port GPIOB
#define ENABLE_POWER_SUPPLIES_Pin GPIO_PIN_15
#define ENABLE_POWER_SUPPLIES_GPIO_Port GPIOB
#define ENCODER_RIGHT_2_Pin GPIO_PIN_12
#define ENCODER_RIGHT_2_GPIO_Port GPIOD
#define ENCODER_RIGHT_1_Pin GPIO_PIN_13
#define ENCODER_RIGHT_1_GPIO_Port GPIOD
#define SECONDARY_MOTOR_IN1_Pin GPIO_PIN_3
#define SECONDARY_MOTOR_IN1_GPIO_Port GPIOG
#define SECONDARY_MOTOR_IN2_Pin GPIO_PIN_4
#define SECONDARY_MOTOR_IN2_GPIO_Port GPIOG
#define SECONDARY_MOTOR_IN3_Pin GPIO_PIN_5
#define SECONDARY_MOTOR_IN3_GPIO_Port GPIOG
#define SECONDARY_MOTOR_IN4_Pin GPIO_PIN_6
#define SECONDARY_MOTOR_IN4_GPIO_Port GPIOG
#define ENCODER_LEFT_2_Pin GPIO_PIN_6
#define ENCODER_LEFT_2_GPIO_Port GPIOC
#define ENCODER_LEFT_1_Pin GPIO_PIN_7
#define ENCODER_LEFT_1_GPIO_Port GPIOC
#define SENSOR_INPUT_CAPTURE_2_Pin GPIO_PIN_8
#define SENSOR_INPUT_CAPTURE_2_GPIO_Port GPIOC
#define SENSOR_INPUT_CAPTURE_4_Pin GPIO_PIN_9
#define SENSOR_INPUT_CAPTURE_4_GPIO_Port GPIOC
#define ETH_OSC_OUT_Pin GPIO_PIN_8
#define ETH_OSC_OUT_GPIO_Port GPIOA
#define DEBUG_TX_Pin GPIO_PIN_9
#define DEBUG_TX_GPIO_Port GPIOA
#define DEBUG_RX_Pin GPIO_PIN_10
#define DEBUG_RX_GPIO_Port GPIOA
#define SERVO_TX_Pin GPIO_PIN_5
#define SERVO_TX_GPIO_Port GPIOD
#define SERVO_RX_Pin GPIO_PIN_6
#define SERVO_RX_GPIO_Port GPIOD
#define SERVO_PWM2_Pin GPIO_PIN_3
#define SERVO_PWM2_GPIO_Port GPIOB
#define SENSOR_INPUT_CAPTURE_1_Pin GPIO_PIN_4
#define SENSOR_INPUT_CAPTURE_1_GPIO_Port GPIOB
#define SENSOR_INPUT_CAPTURE_3_Pin GPIO_PIN_5
#define SENSOR_INPUT_CAPTURE_3_GPIO_Port GPIOB
#define SERVO_ROBOTIS_TX_ENABLE_Pin GPIO_PIN_7
#define SERVO_ROBOTIS_TX_ENABLE_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
