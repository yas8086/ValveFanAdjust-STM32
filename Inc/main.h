/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	
#include "ds18b20.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// 任务结构
typedef struct _TASK_COMPONENTS
{
    uint8_t Run;                 				     // 程序运行标记：0-不运行，1运行
    uint16_t Timer;             					 // 计时器
    uint16_t ItvTime;              					 // 任务运行间隔时间
    void (*TaskHook)(void);   						 // 要运行的任务函数
} TASK_COMPONENTS;       							 // 任务定义



	
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
	
/************************************************************/
/*                                                          */
/************************************************************/
/*                                                          */
/*                    控制参数                              */
/*                                                          */
/************************************************************/
#define TEMPERATURE_NUMBER  			 10          //温度记录个数
#define FC_START_POWER                   90000         //电堆进气阀打开功率（开机功率）单位：100 * W
#define FC_CLOSE_POWER                   80000         //电堆进气阀关闭功率（关机功率）单位：100 * W
#define EXHAUST_VALVES_COUNTER_NUMBER 	 2800  		 //累计电流值
#define EXHAUST_TIME  		 		     300   	     //正常排气时间
#define EXHAUST_SPECIFIC_TIME 			 300000 	 //特定强制排气间隔5min
#define EXHAUST_SPECIFIC_TIMING 		 700  	 	 //特定排气时间2s
#define FC_START_VOLTAGE       			 2000  		 //电堆启动电压   20V
#define FC_CLOSE_VOLTAGE      			 1500   	 //电堆关闭电压   15V
#define FC_START_EXHAUST_TIME 			 300  		 //电堆开机持续排气时间2秒
#define MIN_RUNING_PWM_VAL    			 2    		 //电堆运行时风扇最小转速
#define MIN_PWM_VAL           			 0     		 //风扇最小转速
#define MAX_PWM_VAL            			 100   		 //风扇最大转速
#define SHELL_TEMPERATURE_HIGH			 2100      //外壳温度过高开启风扇  单位:100*℃

#define FUNCTION_TEXT         0//功能测试代码（1为开，0为关）
#define EXHAUST_ON   		 		  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)       //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET)        //排气阀关闭	2019-03-29修改：常闭  2019-04-04修改为：LED灯控制排气
#define EXHAUST_OFF           HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)     // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)      	//排气阀打开
#define INLET_OFF             HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)     //进气阀关闭
#define INLET_ON	            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)     //进气阀打开
#define FC_SWITCH_ON 		      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)				//FC到负载通道打开
#define FC_SWITCH_ON_2		    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET)			//FC到负载通道打开
#define FC_SWITCH_OFF 		    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET)
#define FC_SWITCH_OFF_2		    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET)
#define LI_SWITCH_ON 		      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET)
#define LI_SWITCH_ON_2		    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)		
#define LI_SWITCH_OFF 	      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET)			//Li到负载通道关闭
#define LI_SWITCH_OFF_2		    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET)				//Li到负载通道关闭（漏电端）
#define SHELL_FAN_ON		      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)     		//外壳风扇开启
#define SHELL_FAN_OFF	        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)			//外壳风扇关闭
#define FAN_ON             	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)			//风扇打开
#define FAN_OFF             		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)		//风扇关闭
#define LI_CHARGE_ON           	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET) //打开充电通道
#define LI_CHARGE_OFF           HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)   //关闭充电通道

#define MIN_PWM           			 0     		 //最小PWM
#define MAX_PWM            			 100   		 //最大PWM
#define USART1_MAX_RECV_LEN      20
#define TASKS_MAX                7
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Output_LI_switch_control_2_Pin GPIO_PIN_13
#define Output_LI_switch_control_2_GPIO_Port GPIOC
#define Output_FC_switch_control_2_Pin GPIO_PIN_14
#define Output_FC_switch_control_2_GPIO_Port GPIOC
#define ADC_charge_I_Pin GPIO_PIN_0
#define ADC_charge_I_GPIO_Port GPIOA
#define ADC_FC_I_Pin GPIO_PIN_1
#define ADC_FC_I_GPIO_Port GPIOA
#define ADC_LOAD_U_Pin GPIO_PIN_2
#define ADC_LOAD_U_GPIO_Port GPIOA
#define ADC_LI_U_Pin GPIO_PIN_3
#define ADC_LI_U_GPIO_Port GPIOA
#define ADC_FC_U_Pin GPIO_PIN_4
#define ADC_FC_U_GPIO_Port GPIOA
#define ADC_FC_temperature_Pin GPIO_PIN_5
#define ADC_FC_temperature_GPIO_Port GPIOA
#define ADC_shell_temperature_Pin GPIO_PIN_6
#define ADC_shell_temperature_GPIO_Port GPIOA
#define ADC_pressure_Pin GPIO_PIN_7
#define ADC_pressure_GPIO_Port GPIOA
#define ADC_environment_temperature_Pin GPIO_PIN_0
#define ADC_environment_temperature_GPIO_Port GPIOB
#define ADC_LI_I_Pin GPIO_PIN_1
#define ADC_LI_I_GPIO_Port GPIOB
#define FAN_control_Pin GPIO_PIN_2
#define FAN_control_GPIO_Port GPIOB
#define exhaust_valve_Pin GPIO_PIN_12
#define exhaust_valve_GPIO_Port GPIOB
#define Fan_pwm_Pin GPIO_PIN_13
#define Fan_pwm_GPIO_Port GPIOB
#define Inlet_valve_Pin GPIO_PIN_14
#define Inlet_valve_GPIO_Port GPIOB
#define Output_LI_charge_control_Pin GPIO_PIN_15
#define Output_LI_charge_control_GPIO_Port GPIOB
#define Output_FC_switch_control_Pin GPIO_PIN_8
#define Output_FC_switch_control_GPIO_Port GPIOA
#define Output_LI_switch_control_Pin GPIO_PIN_9
#define Output_LI_switch_control_GPIO_Port GPIOA
#define Output_shell_fan_switch_control_Pin GPIO_PIN_10
#define Output_shell_fan_switch_control_GPIO_Port GPIOA
#define Output_heat_tube_switch_control_Pin GPIO_PIN_15
#define Output_heat_tube_switch_control_GPIO_Port GPIOA
#define Output_LED_control_Pin GPIO_PIN_4
#define Output_LED_control_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_8
#define SPI1_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

volatile extern uint32_t ADC_data[10];
extern uint8_t mcp41010[2];

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
