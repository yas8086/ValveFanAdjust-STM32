/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */
	 
extern uint16_t  FC_I;                 				     //FC����
extern uint16_t  FC_U;                 				     //FC��ѹ
extern uint32_t  Load_power;                                //���ع���
extern uint16_t  LI_I;                 				     //LI����
extern uint16_t  LI_U;                 				     //LI��ѹ
extern uint16_t  LI_charge_I;                 				 //LI������
extern uint16_t  LOAD_U;                                    //���ص�ѹ
extern float     ntc_R;                                     //��������ֵ���ͺ�3950��
extern uint16_t  FC_pressure;
extern uint16_t  gas_cylinder_pressure;					 //��ƿѹ��
extern uint16_t  shell_temperature;                         //����¶�
extern float     shell_ntc_R;                               //��������ֵ���ͺ�3950��
extern uint16_t  PWM_VAL;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void Task_Get_ADC_data(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

