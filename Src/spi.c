/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */
uint8_t  temperature_record_counter;
uint16_t max_temperature;
uint16_t min_temperature;

uint8_t mcp41010[2];
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI1_ENABLE();

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_5);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void MCP41010_write_bite(uint8_t value)
{
	mcp41010[0] = 17; //�Ե�λ��0д����
	//0~255 ---> 10K~0K
	mcp41010[1] = value; //������������ֵ��һ��//153-->6A  //175-->8A
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	
	HAL_SPI_Transmit(&hspi1,mcp41010,2,1000);
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	
}

//ð�����򲢼����ÿ����������
uint16_t bubble_sort(uint16_t a[],int n)//nΪ����a��Ԫ�ظ���
{
	uint16_t b[n];
	for(uint16_t count=0; count<n;count++)
	{
		b[count]=a[count];
	}
    //һ������N-1�ֱȽ�
    for(int i=0; i<n-1; i++)
    {
        //ÿһ�ֱȽ�ǰn-1-i������������õ����i�����ñȽ�
        for(int j=0; j<n-1-i; j++)
        {
            if(b[j] > b[j+1])
            {
                int temp = b[j];
                b[j] = b[j+1];
                b[j+1]=temp;
            }
        }
    }
	
	max_temperature = b[n-1];
	min_temperature = b[0];
	
	return ((b[n-1] - b[0]) / n); //����0.01��
	
}

void Temprature_record_function(void)
{
	//ÿ���¼�¶�
	if(temperature_counter > 1000)
	{
		
		temperature_record[TEMPERATURE_NUMBER - 1] = temperature;
		
		for(int i=0;i<9;i++)
		{
			temperature_record[i] = temperature_record[i+1];
			
		}

		
		if(temperature_record_counter == 10)
		{
			temperature_record_counter = 0;
			
		}

		temperature_counter = 0;
		
		temperature_rate = bubble_sort(temperature_record,TEMPERATURE_NUMBER);//������������
	}

	
	//ÿ5���¼һ���¶�
	if(pretemperature_counter > 5000)
	{
		pretemperature = temperature;
		
		pretemperature_counter = 0; //��������0
	}
}
/* USER CODE END 1 */
