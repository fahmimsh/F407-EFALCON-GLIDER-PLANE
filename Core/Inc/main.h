/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*---KODE INTERUPT---*/
typedef struct
{
    volatile uint8_t flag;     /* Timeout event flag */
    uint16_t timer;             /* Timeout duration in msec */
    uint16_t prevCNDTR;         /* Holds previous value of DMA_CNDTR */
} DMA_Event_t;
#define DMA_BUF_SIZE 500
#define DMA_TIMEOUT_MS 10      /* DMA Timeout duration in msec */
/*----KODE IMU -------*/
typedef struct{
	float YAW;
	float PITCH;
	float ROLL;
}IMU_DATA;
/*------KODE TIM----*/
typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint32_t RisingEdgeVal;
	uint32_t FallingEdgeVal;
	uint32_t DutyCycleVal;
	uint32_t FrequencyVal;
	bool onRisingEdge;
	bool onFallingEdge;
}PWM_DATA;
typedef enum {
	FLY_MODE_OFF,
	FLY_MODE_ON,
	FLY_MODE_HOLD
} FLY_MODE;
volatile float sensorYaw, sensorPitch, sensorRoll;
volatile float inputYaw, inputPitch, inputRoll;
volatile int inputThrottle, inputFlyMode;
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
#define RC_CH6_Pin GPIO_PIN_3
#define RC_CH6_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
