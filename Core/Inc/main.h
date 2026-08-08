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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FLASOR_Pin GPIO_PIN_2
#define FLASOR_GPIO_Port GPIOE
#define KORNA_Pin GPIO_PIN_3
#define KORNA_GPIO_Port GPIOE
#define SELENOID_VALF_Pin GPIO_PIN_4
#define SELENOID_VALF_GPIO_Port GPIOE
#define YSB_Pin GPIO_PIN_5
#define YSB_GPIO_Port GPIOE
#define SISTEM_Pin GPIO_PIN_6
#define SISTEM_GPIO_Port GPIOE
#define YEDEK_Pin GPIO_PIN_13
#define YEDEK_GPIO_Port GPIOC
#define FORWARD_Pin GPIO_PIN_0
#define FORWARD_GPIO_Port GPIOC
#define BACK_Pin GPIO_PIN_1
#define BACK_GPIO_Port GPIOC
#define H2_ADC_Pin GPIO_PIN_4
#define H2_ADC_GPIO_Port GPIOA
#define H2_DAC_Pin GPIO_PIN_5
#define H2_DAC_GPIO_Port GPIOA
#define MUX_ADC_Pin GPIO_PIN_1
#define MUX_ADC_GPIO_Port GPIOB
#define MUX_S0_Pin GPIO_PIN_7
#define MUX_S0_GPIO_Port GPIOE
#define MUX_S1_Pin GPIO_PIN_8
#define MUX_S1_GPIO_Port GPIOE
#define MUX_S2_Pin GPIO_PIN_9
#define MUX_S2_GPIO_Port GPIOE
#define MUX_EN_Pin GPIO_PIN_10
#define MUX_EN_GPIO_Port GPIOE
#define RF_PARAMETRE_Pin GPIO_PIN_10
#define RF_PARAMETRE_GPIO_Port GPIOD
#define LED8_Pin GPIO_PIN_4
#define LED8_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_5
#define LED7_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_7
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
