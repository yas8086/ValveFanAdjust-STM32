/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
#define TEMPERATURE_NUMBER  10
volatile extern uint16_t temperature;
extern uint16_t temperature_counter;    //�¶ȼ�����
extern uint16_t temperature_record[10]; //ÿ���¶ȼ�¼
extern uint16_t  pretemperature_counter; //��һ�¶ȼ�����
extern uint16_t  pretemperature;       //��¼��һ���¶�
extern uint16_t  temperature_rate;     //��������
/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
uint16_t bubble_sort(uint16_t a[],int n);
void Temprature_record_function(void);
void MCP41010_write_bite(uint8_t value);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

