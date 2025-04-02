#include "ds18b20.h"
#include "tim.h"	
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//DS18B20驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
  


//复位DS18B20A*********************************************************************************************************
void DS18B20_RstA(void)	   
{       
    
	  DS18B20_IO_OUTA(); 	//SET PG11 OUTPUT
    DS18B20_DQ_OUTA=0; 	//拉低DQ
		
    delay_us(750);    	//拉低750us
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //输出1
    DS18B20_DQ_OUTA=1; 	//DQ=1 
  	delay_us(15);     	//15US
}
//等待DS18B20的回应
//返回1:未检测到DS18B20的存在
//返回0:存在
uint8_t DS18B20_CheckA(void) 	   
{   
	uint8_t retry=0;
	DS18B20_IO_INA();	//SET PG11 INPUT	 
    while (DS18B20_DQ_INA&&retry<200)
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=200)return 1;
	else retry=0;
    while (!DS18B20_DQ_INA&&retry<240)
	{
		retry++;
		delay_us(1);
	};
	if(retry>=240)return 1;	    
	return 0;
}
//从DS18B20读取一个位
//返回值：1/0
uint8_t DS18B20_Read_BitA(void)
{
    uint8_t data;
	DS18B20_IO_OUTA();	//SET PG11 OUTPUT
    DS18B20_DQ_OUTA=0;
	delay_us(2);
    DS18B20_DQ_OUTA=1;
	DS18B20_IO_INA();	//SET PG11 INPUT
	delay_us(12);
	if(DS18B20_DQ_INA)data=1;
    else data=0;	 
    delay_us(50);           
    return data;
}
//从DS18B20读取一个字节
//返回值：读到的数据
uint8_t DS18B20_Read_ByteA(void)     
{        
    uint8_t i,j,dat;
    dat=0;
	for (i=1;i<=8;i++) 
	{
        j=DS18B20_Read_BitA();
        dat=(j<<7)|(dat>>1);
    }						    
    return dat;
}
//写一个字节到DS18B20
//dat：要写入的字节
void DS18B20_Write_ByteA(uint8_t dat)     
 {             
    uint8_t j;
    uint8_t testb;
	DS18B20_IO_OUTA();	//SET PG11 OUTPUT;
    for (j=1;j<=8;j++) 
	{
        testb=dat&0x01;
        dat=dat>>1;
        if (testb) 
        {
            DS18B20_DQ_OUTA=0;	// Write 1
            delay_us(2);                            
            DS18B20_DQ_OUTA=1;
            delay_us(60);             
        }
        else 
        {
            DS18B20_DQ_OUTA=0;	// Write 0
            delay_us(60);             
            DS18B20_DQ_OUTA=1;
            delay_us(2);                          
        }
    }
}
//开始温度转换
void DS18B20_StartA(void) 
{   						               
    DS18B20_RstA();	   
	
	  DS18B20_CheckA();	 
	  
    DS18B20_Write_ByteA(0xcc);	// skip rom
    DS18B20_Write_ByteA(0x44);	// convert
} 

//初始化DS18B20的IO口 DQ 同时检测DS的存在
//返回1:不存在
//返回0:存在    	 
uint8_t DS18B20_InitA(void)
{
 	GPIO_InitTypeDef GPIO_InitStruct = {0};


  /*Configure GPIO pins : PBPin PBPin PBPin PBPin */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);    //输出1
	DS18B20_RstA();
  
	return DS18B20_CheckA();
}  
//从ds18b20得到温度值
//精度：0.1C
//返回值：温度值 （-550~1250） 
short int DS18B20_Get_TempA(void)
{
    uint8_t temp;
    uint8_t TL,TH;
	short int tem;
    DS18B20_StartA ();  			// ds1820 start convert
    DS18B20_RstA();
    DS18B20_CheckA();	 
    DS18B20_Write_ByteA(0xcc);	// skip rom
    DS18B20_Write_ByteA(0xbe);	// convert	    
    TL=DS18B20_Read_ByteA(); 	// LSB   
    TH=DS18B20_Read_ByteA(); 	// MSB  
	    	  
    if(TH>7)
    {
        TH=~TH;
        TL=~TL; 
        temp=0;					//温度为负  
    }else temp=1;				//温度为正	  	  
    tem=TH; 					//获得高八位
    tem<<=8;    
    tem+=TL;					//获得底八位
    tem=(float)tem*0.625;		//转换     
	if(temp)return tem*0.1; 		//返回温度值
	else return -tem*0.1;    
}

