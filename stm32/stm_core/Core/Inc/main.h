/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define SP_IN_Pin GPIO_PIN_3
#define SP_IN_GPIO_Port GPIOE
#define SAI_FS_Pin GPIO_PIN_4
#define SAI_FS_GPIO_Port GPIOE
#define SAI_SCK_Pin GPIO_PIN_5
#define SAI_SCK_GPIO_Port GPIOE
#define MIC_SD_Pin GPIO_PIN_6
#define MIC_SD_GPIO_Port GPIOE
#define GPS_TX_Pin GPIO_PIN_0
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_1
#define GPS_RX_GPIO_Port GPIOA
#define EP_SDA_Pin GPIO_PIN_2
#define EP_SDA_GPIO_Port GPIOB
#define WL_RX_Pin GPIO_PIN_7
#define WL_RX_GPIO_Port GPIOE
#define WL_TX_Pin GPIO_PIN_8
#define WL_TX_GPIO_Port GPIOE
#define EP_CS_Pin GPIO_PIN_11
#define EP_CS_GPIO_Port GPIOE
#define EP_EN_Pin GPIO_PIN_12
#define EP_EN_GPIO_Port GPIOE
#define EP_BUSY_Pin GPIO_PIN_13
#define EP_BUSY_GPIO_Port GPIOE
#define EP_DC_Pin GPIO_PIN_14
#define EP_DC_GPIO_Port GPIOE
#define EP_RES_Pin GPIO_PIN_15
#define EP_RES_GPIO_Port GPIOE
#define EC_EN_Pin GPIO_PIN_10
#define EC_EN_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_12
#define BT_RX_GPIO_Port GPIOB
#define BT_TX_Pin GPIO_PIN_13
#define BT_TX_GPIO_Port GPIOB
#define EP_SCK_Pin GPIO_PIN_10
#define EP_SCK_GPIO_Port GPIOC
#define GY_SCL_Pin GPIO_PIN_6
#define GY_SCL_GPIO_Port GPIOB
#define GY_SDA_Pin GPIO_PIN_7
#define GY_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
