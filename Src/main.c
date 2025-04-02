/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

volatile  uint16_t temperature;              //FC温度
uint8_t	  FC_Start_flag;          				   //FC启停标志
uint16_t  FC_I;                 				     //FC电流
uint16_t  FC_U;                 				     //FC电压
uint32_t  Load_power;                        //负载功率
uint16_t  LI_I;                 				     //LI电流
uint16_t  LI_U;                 				     //LI电压
uint16_t  LI_charge_I;                 			 //LI充电电流
uint16_t  LOAD_U;                            //负载电压
uint16_t  FC_pressure;                 			 //FC进气压力
uint16_t  gas_cylinder_pressure;					   //气瓶压力
uint16_t  shell_temperature;                         //外壳温度
float     environment_temperature;                   //环境温度

uint16_t  temperature_record[TEMPERATURE_NUMBER];    //每秒温度记录
uint8_t   Exhaust_valves_flag;   					 //开启排气标志; 置1:进入排气;  置0：不需要排气
uint8_t   Exhaust_status;       				   //排气阀开关状态
uint8_t   Inlet_valves_flag;     					 //开启进气阀标志: 置1:需打开进气;  置0：不需要打开
uint16_t  temperature_counter;						 //温度计数器(最大计数655.35秒)
float     ntc_R;                                     //热敏电阻值（型号3950）
float     shell_ntc_R;                               //热敏电阻值（型号3950）
uint16_t  pretemperature_counter;					 //上一温度计数器
uint16_t  pretemperature;       					 //记录上一次温度 5s更新一次
uint16_t  temperature_rate;    						 //升温速率
uint16_t  Exhaust_valves_counter; 					 //排气阀间隔计数器
uint32_t  Exhaust_specific_counter;					 //排气阀特殊计数器（定时一段时间内强排几秒）
uint8_t   temperature_tendency; 					 //温度走势 1:升温; 0:降温或稳定

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
bool 			FC_Start_flag_;											//开机标志位
//排气阀-------------------------
uint8_t   Exhaust_status;       				     	//排气阀开关状态
uint8_t   Exhaust_valves_flag;   					 		//开启排气标志
uint16_t  Exhaust_time_counter; 							//排气阀正常排气时间计数器
//正常排气
uint16_t  Exhaust_time_Normal=500;						//排气阀正常排气时间
uint16_t  Exhaust_interval_Normal=12000;			//排气阀正常排气间隔
uint16_t  Exhaust_interval_Normal_counter; 		//排气阀正常排气间隔计数器
//强制排气
uint32_t  Exhaust_time_Specific=2000;					//排气阀强制排气时间
uint32_t  Exhaust_interval_Specific=300000;		//排气阀强制排气间隔
uint32_t  Exhaust_interval_Specific_counter;	//排气阀强制排气间隔计数器
//手动排气
uint8_t   Exhaust_valves_Manual_flag;   			//手动开启排气标志
uint8_t   Exhaust_Manual_status;       				//排气阀手动开关状态
uint32_t  Exhaust_time_Manual=1000;						//排气阀手动排气时间
uint32_t  Exhaust_time_Manual_counter;				//排气阀手动排气时间计数器
//风扇---------------------------
uint16_t   PWM_VAL = 13;               				//风扇转速百分比
//串口---------------------------
volatile uint8_t Uart_ReceiveData_flag;                     //串口接收数据标志
uint8_t USART1_REC_BUF[USART1_MAX_RECV_LEN];        				//接收数据包缓冲区
uint8_t USART2_REC_BUF_Engine[4];                   //发动机数据包
volatile bool sendDataFlag = false;																	//是否回应数据标志位
volatile uint8_t	UART1_temp[1];              								//当前接收字节
volatile uint16_t USART1_REC_STA=0;													//当前字节是连续的第几位
volatile uint8_t respondDeviceID,respondDeviceValue;					//回应数据的ID和Value
uint16_t RecCrc, CalcCrc;														//接收的CRC校验码和计算的CRC校验码
volatile uint16_t CntRx1 = 0;																		//串口1接收时间标志位,7ms接收不到新数据，判定为1个包

//热敏电阻（B值：3950）温度对照数组 （-30℃ ~ 240℃）   一行10℃
float Thermistor3950[]={
		200.2039,	187.3164,	175.3536,	164.2428,	153.9176,	144.3169,	135.3851,	127.071,	119.3276,	112.112,	
		105.3847,	99.1093,	93.2524,	87.7834,	82.674,	77.8981,	73.4319,	69.2531,	65.3415,	61.6781,	
		58.2457,	55.0282,	52.0106,	49.1794,	46.5218,	44.026,	41.6813,	39.4773,	37.4049,	35.4554,	
		33.6206,	31.8931,	30.266,	28.7328,	27.2875,	25.9246,	24.6387,	23.4251,	22.2793,	21.1971,	
		20.1746,	19.208,	18.2941,	17.4296,	16.6115,	15.8371,	15.1039,	14.4092,	13.751,	13.127,	
		12.5353,	11.9741,	11.4415,	10.936,	10.4559,	10,	9.5668,	9.1551,	8.7636,	8.3913,	
		8.0371,	7.7001,	7.3793,	7.0738,	6.7828,	6.5055,	6.2413,	5.9894,	5.7492,	5.5201,	
		5.3015,	5.0928,	4.8936,	4.7034,	4.5217,	4.3481,	4.1822,	4.0236,	3.872,	3.7269,	
		3.5882,	3.4554,	3.3283,	3.2066,	3.0901,	2.9784,	2.8715,	2.769,	2.6707,	2.5765,	
		2.4862,	2.3995,	2.3163,	2.2365,	2.1599,	2.0864,	2.0157,	1.9479,	1.8827,	1.8201,	
		1.7598,	1.7019,	1.6463,	1.5927,	1.5412,	1.4917,	1.444,	1.3981,	1.3539,	1.3113,	
		1.2703,	1.2308,	1.1928,	1.1561,	1.1208,	1.0867,	1.0538,	1.0221,	0.9915,	0.962,	
		0.9336,	0.9061,	0.8796,	0.854,	0.8292,	0.8054,	0.7823,	0.76,	0.7385,	0.7176,	
		0.6975,	0.6781,	0.6592,	0.641,	0.6234,	0.6064,	0.5899,	0.574,	0.5586,	0.5436,	
		0.5291,	0.5151,	0.5016,	0.4884,	0.4757,	0.4633,	0.4514,	0.4398,	0.4285,	0.4177,	
		0.4071,	0.3968,	0.3869,	0.3773,	0.3679,	0.3588,	0.35,	0.3415,	0.3332,	0.3251,	
		0.3173,	0.3097,	0.3023,	0.2951,	0.2882,	0.2814,	0.2748,	0.2684,	0.2622,	0.2562,	
		0.2503,	0.2446,	0.239,	0.2336,	0.2284,	0.2233,	0.2183,	0.2134,	0.2087,	0.2041,	
		0.1997,	0.1953,	0.1911,	0.187,	0.183,	0.1791,	0.1753,	0.1715,	0.1679,	0.1644,	
		0.161,	0.1576,	0.1544,	0.1512,	0.1481,	0.1451,	0.1421,	0.1392,	0.1364,	0.1337,	
		0.131,	0.1284,	0.1259,	0.1234,	0.121,	0.1186,	0.1163,	0.1141,	0.1119,	0.1097,	
		0.1076,	0.1056,	0.1036,	0.1016,	0.0997,	0.0979,	0.096,	0.0943,	0.0925,	0.0908,	
		0.0892,	0.0875,	0.086,	0.0844,	0.0829,	0.0814,	0.08,	0.0785,	0.0771,	0.0758,	
		0.0745,	0.0732,	0.0719,	0.0706,	0.0694,	0.0682,	0.0671,	0.0659,	0.0648,	0.0637,	
		0.0626,	0.0616,	0.0606,	0.0596,	0.0586,	0.0576,	0.0567,	0.0557,	0.0548,	0.0539,	
		0.0531,	0.0522,	0.0514,	0.0506,	0.0498,	0.049,	0.0482,	0.0474,	0.0467,	0.046,	
		0.0453,	0.0446,	0.0439,	0.0432,	0.0425,	0.0419,	0.0413,	0.0406,	0.04,	0.0394,	
		0.0388
	};

//函数定义
void get_rec_data(void);
void USART3_Respond(void);
void response_data_send(uint8_t deviceID,uint8_t deviceValue);
uint16_t crc16_calc(uint8_t *crc_buf , uint8_t crc_leni);
uint8_t crc8_calc(uint8_t *data, uint8_t len);

void Task_USART3_Printf(void);
void Task_Exhaust_valves_count(void);
void Task_Exhaust_valves_control(void);
void Task_PWM_control(void);
void TaskRemarks(void);
void TaskProcess(void);

/**************************************************************************************
* Variable definition                            
**************************************************************************************/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CRC校验
uint16_t crc16_calc(uint8_t *crc_buf , uint8_t crc_leni)
{
	unsigned char i, j;
	unsigned int crc_sumx;

	crc_sumx = 0xFFFF;

	for (i = 0; i < crc_leni; i++) 
	{
		crc_sumx ^= *(crc_buf + i);
		for (j = 0; j < 8; j++) {
			if (crc_sumx & 0x01) {
				crc_sumx >>= 1;
				crc_sumx ^= 0xA001;
			} else {
				crc_sumx >>= 1;
			}
		}
	}
	
	return crc_sumx;
}

//递归二分查找
int recurbinary(float *a, float key, int low, int high)
{
	int mid = (low+high)/2;
	
	if(a[low] < key)
	{
		return low;
	}
	else if(a[high] > key)
	{
		return high;
	}
	
	if(a[mid+1]<=key && a[mid-1]>=key) 
	{
		if(a[mid+1]<=key&&a[mid]>=key)
		{
			return mid+1;
		}else
		{
			return mid;
		}
	}
	else if(a[mid] < key)
	{
		return recurbinary(a, key, low, mid);
	}
	else 
	{
		return recurbinary(a, key, mid, high);
	}
}

//装载参数数据
void USART1_Parameters_Send(void)
{
#if FUNCTION_TEXT
	printf("****************BEGIN LINE********************\r\n");
	
	printf("temperature = %.2f 摄氏度 \r\n",(float)temperature*0.01);
	printf("PWM_VAL = %d \r\n",PWM_VAL);
	printf("environment_temperature = %.2f 摄氏度 \r\n",environment_temperature);
	printf("shell_temperaturen = %.2f 摄氏度 \r\n",(float)shell_temperature*0.01);
	printf("Exhaust_status = %d \r\n",Exhaust_status);

	printf("****************END LINE**********************\r\n");
#else
	
	uint8_t Plane_data[14] = { 0 };

	Plane_data[0] = 0x0E;   //地址
	Plane_data[1] = 0x01;		//功能码：参数通信帧
	Plane_data[2] = 0x00;		//起始地址
	Plane_data[3] = 0x04;		//读取个数
	Plane_data[4] = (temperature+4000) >> 8;
	Plane_data[5] = (temperature+4000) & 0xff;
	Plane_data[6] = shell_temperature >> 8;
	Plane_data[7] = shell_temperature & 0xff;
	Plane_data[8] = (uint16_t)environment_temperature >> 8;
	Plane_data[9] = (uint16_t)environment_temperature & 0xff;
	Plane_data[10] = Exhaust_status >> 8;
	Plane_data[11] = 0x00;
	uint16_t crc = crc16_calc(Plane_data, 12);
	Plane_data[12] = crc >> 8;
	Plane_data[13] = crc & 0xff;

	HAL_UART_Transmit(&huart1,Plane_data,14,1000);

#endif

}

//装载应答数据
void response_data_send(uint8_t deviceID,uint8_t deviceValue)
{
	#if FUNCTION_TEXT
	printf("****************BEGIN LINE********************\r\n");
	
	printf("****************END LINE**********************\r\n");
	
	#else
	uint8_t data_send[6] = {0};
	data_send[0] = 0x0E;				//地址码
	data_send[1] = 0x02;				//功能码：应答数据帧
	data_send[2] = deviceID;
	data_send[3] = deviceValue;		
	uint16_t resCrc = crc16_calc(data_send, 4);
	data_send[4] = resCrc >> 8;
	data_send[5] = resCrc & 0xff;

	HAL_UART_Transmit(&huart1,data_send,6,1000);

	#endif

}

//解包函数
//0C 02 deviceid value1 value2 value3 value4 CRC1 CRC2
void get_rec_data(void)
{
	if (USART1_REC_BUF[0] == 0x0C && USART1_REC_BUF[1] == 0x02)
	{
		//deviceid：风扇01、 电磁阀参数更改02、 电磁阀手动开启03
		//PWM通讯样例：0C 02 01 00 00 00 0D E7 2A
		if(USART1_REC_BUF[2] == 0x01)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[7] << 8) | USART1_REC_BUF[8];
			CalcCrc = crc16_calc(USART1_REC_BUF, 7);
			if(RecCrc != CalcCrc){
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x02;           //CRC校验错误
			}
			else{
				if(USART1_REC_BUF[6] >= 100)
					USART1_REC_BUF[6] = 100;
				PWM_VAL = USART1_REC_BUF[6];
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x01;           //设置成功
			}
		}
		//排气参数更改通讯样例：0C 02 02 27 10 00 00 53 A4
		else if(USART1_REC_BUF[2] == 0x02)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[7] << 8) | USART1_REC_BUF[8];
			CalcCrc = crc16_calc(USART1_REC_BUF, 7);
			if(RecCrc != CalcCrc){
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x02;
			}
			else{
				Exhaust_time_Normal = ((uint16_t)USART1_REC_BUF[3] << 8) | USART1_REC_BUF[4];
				Exhaust_interval_Normal = ((uint16_t)USART1_REC_BUF[5] << 8) | USART1_REC_BUF[6];
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x01;
			}
		}
		//手动排气通讯样例：0C 02 03 00 00 00 01 22 53
		else if(USART1_REC_BUF[1] == 0x03)
		{
			RecCrc = ((uint16_t)USART1_REC_BUF[7] << 8) | USART1_REC_BUF[8];
			CalcCrc = crc16_calc(USART1_REC_BUF, 7);
			if(RecCrc != CalcCrc){
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x02;
			}
			else{
				Exhaust_valves_Manual_flag = 1;
				respondDeviceID = USART1_REC_BUF[2];
				respondDeviceValue = 0x01;
			}
		}
		else{
			respondDeviceID = 0xFF;					//设备码错误
			respondDeviceValue = 0xFF;
		}
	}
	else{
		respondDeviceID = 0xFE;						//地址码错误
		respondDeviceValue = 0xFE;
	}
	sendDataFlag = true;	
}



//串口任务
void Task_USART1_Respond(void)
{
	USART1_Parameters_Send();
	if(Uart_ReceiveData_flag){
		get_rec_data();
		memset(USART1_REC_BUF, 0, sizeof (USART1_REC_BUF));
		if(sendDataFlag == true){
			response_data_send(respondDeviceID,respondDeviceValue);
			sendDataFlag = false;
		}
		USART1_REC_STA = 0;
		Uart_ReceiveData_flag = 0;
	}
}

//排气任务：0.05s执行一次
void Task_Exhaust_valves_control(void)
{
	//FC开机完成进入正常电流累加排气流程
	if(FC_Start_flag == 1)
	{
		//开启手动排气----------------------------
		if(Exhaust_valves_Manual_flag == 1 && Exhaust_status == 0 )
		{
			EXHAUST_ON;
			Exhaust_time_counter = 0;
			Exhaust_valves_flag = 0;
			Exhaust_status = 1;
			Exhaust_interval_Normal_counter = 0;//
		}
		//结束手动排气
		if((Exhaust_time_counter > Exhaust_time_Manual) && Exhaust_status == 1 && Exhaust_valves_Manual_flag == 1)
		{
			Exhaust_time_counter = 0;		
			Exhaust_status = 0;			
			EXHAUST_OFF;
			Exhaust_valves_Manual_flag = 0;
		}
		
		
		//开启自动排气-----------------------------
		if(Exhaust_valves_flag == 1 && Exhaust_status == 0 )
		{
			EXHAUST_ON;
			Exhaust_time_counter = 0; 
			Exhaust_valves_flag = 0;
			Exhaust_status = 1;		
		}
		
		//结束自动排气：区分强制排气和正常排气
		if(Exhaust_interval_Specific_counter > Exhaust_interval_Specific)
		{
			if( (Exhaust_time_counter > Exhaust_time_Specific)  && Exhaust_status == 1)
			{
				Exhaust_time_counter = 0;   
				Exhaust_interval_Specific_counter = 0; 
				Exhaust_status = 0;        
				EXHAUST_OFF; 
			}
		}
		else 
		{
			if( (Exhaust_time_counter > Exhaust_time_Normal)  && Exhaust_status == 1)
			{
				Exhaust_time_counter = 0;	
				Exhaust_status = 0;		
				EXHAUST_OFF;
			}
		}
	}
}

//排气检测任务：0.1s执行一次
void Task_Exhaust_valves_count(void)
{
	//正常排气时间到
	if( (Exhaust_interval_Normal_counter > Exhaust_interval_Normal) && Exhaust_valves_flag == 0 && Exhaust_status == 0)
	{
		Exhaust_valves_flag = 1;
	}
	//强制排气时间到
	if( (Exhaust_specific_counter > EXHAUST_SPECIFIC_TIME) && Exhaust_valves_flag == 0 && Exhaust_status == 0)
	{
		Exhaust_valves_flag = 1;
	}
			
}

void Task_NTC_temp(void)
{
	printf("NTC任务\n");
	

		temperature = ((recurbinary(Thermistor3950, ntc_R, 0, 270) - 30)*100); //单位:100 * ℃
		
		shell_temperature = (recurbinary(Thermistor3950, shell_ntc_R, 0, 270) - 30)*100;	//单位:100 * ℃
					
		

}


void Task_environment_temperature(void)
{
	environment_temperature = DS18B20_Get_TempA();
}

//第三个数字=n，表示n毫秒执行一次
static TASK_COMPONENTS TaskComps[] = 
{
  {0, 30, 30, Task_Get_ADC_data},                           // ADC采集数据每(0.03*ADC_AVERAGE_TIME)秒一次
  {0, 100, 100, Task_USART1_Respond},                     	// 串口1数据发送
  {0, 200, 200, Task_NTC_temp},           			  					// 每0.2s刷新一次温度值;每5s刷新一次pretemperature;每1s刷新一次temperature_record
	{0, 75, 75, Task_Exhaust_valves_control},                 // FC排气到时，控制排气持续时间(理论排气0.225s）
	{0, 100, 100, Task_Exhaust_valves_count},    			  			// FC排气计数器控制排气标志位
	{0, 1500, 1500,Task_environment_temperature}			  			// 环境温度采集
};

/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskRemarks(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
         if (TaskComps[i].Timer)          // 时间不为0
        {
            TaskComps[i].Timer--;         // 减去一个节拍
            if (TaskComps[i].Timer == 0)       // 时间减完了
            {
                 TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
                 TaskComps[i].Run = 1;           // 任务可以运行
            }
        }
    }
}


/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理
* EntryParameter : None
* ReturnValue    : None
**************************************************************************************/
void TaskProcess(void)
{
    uint8_t i;
    for (i=0; i<TASKS_MAX; i++)               // 逐个任务时间处理
    {
         if (TaskComps[i].Run)                // 时间不为0
        {
             TaskComps[i].TaskHook();         // 运行任务
             TaskComps[i].Run = 0;            // 标志清0
        }
    }   
}

//定时器2中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim2))
	{
		TaskRemarks();
		temperature_counter++;          				//温度计数器累加
		pretemperature_counter++;	  						//上一温度计数器累加
		Exhaust_time_counter++;       					//自动排气时间计数器
		Exhaust_time_Manual_counter++;    			//手动排气时间计数器
		Exhaust_interval_Normal_counter++;     	//正常排气间隔计数器
		Exhaust_interval_Specific_counter++;   	//强制排气间隔计数器
		Exhaust_specific_counter++;     				//排气阀特殊计数器（定时一段时间内强排几秒）
		
		Exhaust_interval_Normal_counter++;
		
		if(CntRx1)				    
			{
				if(--CntRx1==0)	    	    //7ms后没有新数据来
				{
					USART1_REC_STA|=1<<15;
					Uart_ReceiveData_flag = 1;
				}
			}
	}
}

void Initialize_function(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET); //打开LED灯
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET); //拉高SPI1片选电平
	PWM_VAL = MIN_PWM_VAL;
	TIM_SetCompare3(TIM1,PWM_VAL);
	FAN_ON;//打开电堆风扇
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	Initialize_function();//初始化电堆参数
	
	//启动串口接收
	HAL_UART_Receive_IT(&huart1, (uint8_t *)UART1_temp, 1);
	//开启定时器中断----->1ms
	__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_DBGMCU_EnableDBGStopMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//自动控制
		#if (1-FUNCTION_TEXT)
			TaskProcess();
		#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
