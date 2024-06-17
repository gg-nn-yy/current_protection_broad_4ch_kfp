/**
 *******************************************************************************
 * @file    ads1118_driver.c
 * @author  gao_ning_yuan
 * @version V1.0
 * @date    2024 - 6 - 9
 * @brief   ads1118驱动程序.c文件
 *******************************************************************************
 * 本文件使用STM32标准库，实现了STM32F030硬件SPI驱动ADS1118，对外暴露的接口有：
 * vAds1118_DriverInit():     初始化驱动接口
 * vAds1118_FilterRegCb():    滤波器注册回调接口
 * fAds1118_GetTemperature(): 获取温度接口
 * vAds1118_Task():           任务接口，须在 main()函数 或 线程 中周期性调用
 * vAds1118_TestTask():       测试任务接口
 *
 * @attention
 * _x_ads1118_read_data()：与ADS1118交换数据函数，该函数采用状态机的方式实现，被
   vAds1118_Task()接口调用

 * ADS1118有两个可通过SPl接口访问的寄存器：转换寄存器 和 配置寄存器
   转换寄存器：二进制补码格式包含最后一次转换的结果，上电复位初始值全为零。
   配置寄存器：允许用户修改ADS1118的工作模式和查询设备的状态。16位配置寄存器可用
               于控制ADS1118的工作模式、输入选择、数据速率、满量程范围和温度传感
			   器模式。
 *******************************************************************************
 */



/* ======================== 头文件 ========================================== */
#include <stdio.h>
#include <stddef.h>
#include "stm32f0xx.h"

#include "debug.h"
#include "systick.h"

#include "ads1118_driver.h"



/* ======================== 全局变量定义区 ================================== */
/* 电压分度表，单位mv */
static const float g_voltage_arr[16] = {
	0,     0.317, 1.735, 2.147, 2.96, 3.184, 3.391, 3.806, 
	4.095, 4.633, 6.058, 8.86,  11.8, 14.74, 17.7,  20.64
};


/* 温度分度表，单位℃ */
static const float g_temperature_arr[16] = {
	0,      8.00,   43.00,  58.00,  72.61,  78.00,  83.00,  93.00,
	100.00, 113.00, 148.00, 217.99, 290.15, 360.64, 430.78, 500
};


/* ads1118硬件驱动相关信息 */
static ads1118_drv_t g_ads1118_hw = {
	.spi 		= SPI1,
	.spi_rcc	= RCC_APB2Periph_SPI1,
	
	.rcc		= RCC_AHBPeriph_GPIOA,
	.gpio		= GPIOA,
	
	.mosi_pin	= GPIO_Pin_7,  	// 复用推挽推挽输出
	.miso_pin	= GPIO_Pin_6,  	// 复用推挽推挽输出
	.sck_pin	= GPIO_Pin_5,  	// 复用推挽推挽输出
	.nss_pin	= GPIO_Pin_11,	// 推挽输出
	.cs0_pin	= GPIO_Pin_11,  // 推挽输出
	.cs1_pin	= GPIO_Pin_12, 	// 推挽输出
	.cs2_pin	= GPIO_Pin_4, 	// 推挽输出
};


/* ads1118相关数据 */
static ads1118_info_t g_ads1118[] = {
	[0] = {
		.nss_pin 				= GPIO_Pin_11,
		.state   				= READY,
		.prv_sys_time 			= 0,
		.chx  					= ADS1118_CH_COLD,
		.ch[ADS1118_CH_COLD]  	= {
			.spi_cmd 		= ADS1118_TEMP_CMD, 		// 获取温度指令
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH01_WORK]  = {
			.spi_cmd 		= ADS1118_CH01_ADC_CMD,	// 获取01通道差分ADC值指令
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH23_WORK]  = {
			.spi_cmd 		= ADS1118_CH23_ADC_CMD,	// 获取23通道差分ADC值指令
			.adc_value 		= 0,
		},
	},
	
	[1] = {
		.nss_pin 				= GPIO_Pin_12,
		.state   				= READY,
		.prv_sys_time 			= 0,
		.chx  					= ADS1118_CH_COLD,
		
		.ch[ADS1118_CH_COLD]	= {
			.spi_cmd 		= ADS1118_TEMP_CMD, 		// 获取温度指令
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH01_WORK]  = {
			.spi_cmd 		= ADS1118_CH01_ADC_CMD,	// 获取01通道差分ADC值指令
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH23_WORK]  = {
			.spi_cmd 		= ADS1118_CH23_ADC_CMD,	// 获取23通道差分ADC值指令
			.adc_value 		= 0,
		},
	},
	
};

#define ADS1118_NUM    ( sizeof(g_ads1118) / sizeof(g_ads1118[0]) )

static float (*g_kalman_filter)(float temp, uint8_t chipx, uint8_t chx) = NULL;



/* ======================== 宏函数定义区 ==================================== */
/* 片选线 */
#define __SPI_W_NSS(x) \
GPIO_WriteBit(g_ads1118_hw.gpio, g_ads1118_hw.nss_pin, (BitAction)x)

/* SPI起始 与 结束 */
#define __SPI_START()              __SPI_W_NSS(0)
#define __SPI_STOP()               __SPI_W_NSS(1)


/*******************************************************************************
 * @brief 将 ADC值 转换为 温度值
 * @param[in]  adc: adc值
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __ADC_TO_TEMPERATURE(adc)											\
( adc & 0x8000 ) ?                                                        	\
	(-(~((adc >> 2) - 1) & 0x3fff) * 0.03125f) : ((adc >> 2) * 0.03125f)


/*******************************************************************************
 * @brief 将 ADC值 转换为 电压值
 * @param[in]  adc: adc值
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __ADC_TO_VOLTAGE(adc)												\
( adc & 0x8000 ) ? (-(~(adc - 1)) * 7.8125f / 1000) : (adc * 7.8125f / 1000)


/*******************************************************************************
 * @brief 将 温度值 转换为 电压值
 * @param[in] temp: 温度值
 * @param[in] index: 索引
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __TEMPERATURE_TO_VOLTAGE(temp, index) 								\
 g_voltage_arr[index - 1] +	 												\
(g_voltage_arr[index] - g_voltage_arr[index - 1]) * 						\
	( (temp - g_temperature_arr[index - 1]) /                             	\
	  (g_temperature_arr[index] - g_temperature_arr[index-1]) )


/*******************************************************************************
 * @brief 将 电压值 转换为 温度值
 * @param[in] voltage: 电压值
 * @param[in] index: 索引
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __VOLTAGE_TO_TEMPERATURE(voltage, index) 							\
g_temperature_arr[index-1] +												\
(g_temperature_arr[index] - g_temperature_arr[index - 1]) * 				\
	( (voltage - g_voltage_arr[index-1]) /                                	\
	  (g_voltage_arr[index] - g_voltage_arr[index - 1]) )



/* ======================== 局部接口定义区 ================================== */
/*******************************************************************************
 * @brief GPIO初始化
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _gpio_init(void)
{
	/* 开启时钟                                                               */
	RCC_AHBPeriphClockCmd(g_ads1118_hw.rcc, ENABLE);
	
	/* 复用配置                                                               */
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource7, GPIO_AF_0);
	
	/* MISO_PIN、MOSI_PIN、SCK_PIN 复用推挽输出 */
	GPIO_InitTypeDef  gpio_init_para;
	gpio_init_para.GPIO_Pin   = g_ads1118_hw.miso_pin 	|
								g_ads1118_hw.mosi_pin 	|
								g_ads1118_hw.sck_pin;	
	gpio_init_para.GPIO_Mode  = GPIO_Mode_AF;  	 		// 输入
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// 推挽
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// 无上下拉
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO口速度为50MHz
	GPIO_Init(g_ads1118_hw.gpio, &gpio_init_para);

	
	/* NSS_PIN、CS0_PIN、CS1_PIN、CS2_PIN 推挽输出 */
	gpio_init_para.GPIO_Pin   = g_ads1118_hw.nss_pin	|
								g_ads1118_hw.cs0_pin	|
								g_ads1118_hw.cs1_pin	|
								g_ads1118_hw.cs2_pin;
	gpio_init_para.GPIO_Mode  = GPIO_Mode_OUT;  	 	// 输出
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// 推挽
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// 上拉
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO口速度为50MHz
	GPIO_Init(g_ads1118_hw.gpio, &gpio_init_para);
	
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.nss_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs0_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs1_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs2_pin);
}


/*******************************************************************************
 * @brief 硬件SPI初始化
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _spi_init(void)
{	
	/* 开启时钟                                                               */
	RCC_APB2PeriphClockCmd(g_ads1118_hw.spi_rcc, ENABLE);
	
	/* 硬件SPI初始化配置                                                      */
	SPI_InitTypeDef spi_init_para = {
		.SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_128,// SPI_BaudRate_Prescaler
		.SPI_CPHA 				= SPI_CPHA_2Edge, 			// SPI_Clock_Phase
		.SPI_CPOL 				= SPI_CPOL_Low, 			// SPI_Clock_Polarity
		.SPI_CRCPolynomial 		= 7,
		.SPI_DataSize 			= SPI_DataSize_16b,
		.SPI_Direction 			= SPI_Direction_2Lines_FullDuplex, // SPI_data_direction
		.SPI_FirstBit 			= SPI_FirstBit_MSB,
		.SPI_Mode 				= SPI_Mode_Master,
		.SPI_NSS 				= SPI_NSS_Soft,
	};
	SPI_Init(g_ads1118_hw.spi, &spi_init_para);
	
	SPI_RxFIFOThresholdConfig(g_ads1118_hw.spi, SPI_RxFIFOThreshold_HF);
	
	/* 使能硬件SPI                                                            */
	SPI_Cmd(g_ads1118_hw.spi, ENABLE);
	
	/* 初始默认不选中从机                                                     */
	__SPI_STOP();
}


/*******************************************************************************
 * @brief  SPI读写数据
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static int16_t _spi_swap_data(uint16_t send_data)
{
	__SPI_START();
	
	while( SET != SPI_I2S_GetFlagStatus(g_ads1118_hw.spi, SPI_I2S_FLAG_TXE) );
	SPI_I2S_SendData16(g_ads1118_hw.spi, send_data);
	while( SET != SPI_I2S_GetFlagStatus(g_ads1118_hw.spi, SPI_I2S_FLAG_RXNE) );
	
	__SPI_STOP();
	return ( SPI_I2S_ReceiveData16(g_ads1118_hw.spi) );
}


/*******************************************************************************
 * @brief  ads1118获取数据（基于状态机）
 *
 * @param[in]  spi_cmd: spi通信指令
 * @param[out] p_state: 芯片状态
 * @param[out] prv_sys_time; 系统时间
 * @param[out] p_data:  要保存数据的地址
 *
 * @return ads1118状态
		@retval READY                                                     就绪态
		@retval RUNING	                                                  运行态
		@retval COMPLETE                                                  完成态
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _ads1118_read_data( uint16_t spi_cmd,  ads1118_state_t *p_state, 
	                            uint64_t *prv_sys_time,    uint16_t *p_data  )
{
	switch ( *p_state ) {
		
		/* 就绪态：交换数据，记录系统时间，将ads1118的状态切换为运行态        */
		case READY:
			*p_data = _spi_swap_data(spi_cmd);
			*prv_sys_time = ulSysRunTimeGet();
			*p_state = RUNING;
			break;
		
		/* 运行态：计算时间窗，若大于运行时间窗就将ads1118的状态切换为完成态  */
		case RUNING:
			if ( ADS1118_RUN_TIME <= ulSysRunTimeGet() - *prv_sys_time ) {
				*p_state = COMPLETE;
			}
			break;
		
		/* 其它态：将ads1118的状态切换为就绪态                                */
		default:
			*p_state = READY;
			break;
		
	} // end switch
}


/*******************************************************************************
 * @brief 二分查表法，查找 相等 或 最接近且大于目标值 的元素的索引
 *
 * @param[in] array_to_search：要查找的顺序表（升序浮点数数组）
 * @param[in] len_of_array： 数组大小
 * @param[in] value_to_lookfor：要查找的值
 *
 * @return 如果未找到目标值，则返回右边界索引（最接近目标值的元素索引）
           如果数组中没有大于目标值的元素，则返回 0。
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static uint8_t _binary_search( const float arr_to_search[],
	                               uint32_t len_of_arr,   float target )
{
	uint8_t left  = 0;
	uint8_t right = len_of_arr - 1;
	uint8_t mid   = 0;
	uint8_t index = 0;
	
	while ( left <= right ) {
		
		mid = left + (right - left) / 2;
		
		if ( target < arr_to_search[mid] ) {
			right = mid - 1;
			index = mid;
		}
		else if ( target == arr_to_search[mid] ) {
			return mid;
		}
		else {
			left = mid + 1;
		}
		
	} // end while
	
	return index;
}



/* ======================== 对外接口定义区 ================================== */
/*******************************************************************************
 * @brief ads1118驱动初始化
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_DriverInit(void)
{
	_gpio_init();
	g_ads1118_hw.nss_pin = g_ads1118_hw.cs0_pin;
	_spi_init();
	
	/* 等待电源稳定，以确保通电复位完成；至少等待50μs                        */
	for (uint8_t i = 0; i < 255; i++) {
		__NOP();
	}
}

	
/*******************************************************************************
 * @brief 滤波器注册回调函数
 *
 * @param[out] pf_median_avg_filter;中位值平均滤波器指针
 * @param[out] pf_lowpass_filterr;一阶低通滤波器指针
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_FilterRegCb( float (*pf_filter)(float temperature, uint8_t chipx,
	                                           uint8_t chx) ) 
{
	/* 注册滤波器                                                             */
	g_kalman_filter = pf_filter;
}


/*******************************************************************************
 * @brief ads1118数据转换任务
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_Task(void)
{
	uint8_t last_chx = 0;

	for ( uint8_t chipx = 0; chipx < ADS1118_NUM; chipx++ ) {
		
		/* 1, 切换片选线，指定芯片                                            */
		g_ads1118_hw.nss_pin = g_ads1118[chipx].nss_pin;
		
		last_chx = (g_ads1118[chipx].chx + ADS1118_CH_NUM - 1) % ADS1118_CH_NUM;
		
		/* 2, 交换数据                                                        */
		_ads1118_read_data( g_ads1118[chipx].ch[g_ads1118[chipx].chx].spi_cmd,
							&g_ads1118[chipx].state,
							&g_ads1118[chipx].prv_sys_time,
							&g_ads1118[chipx].ch[last_chx].adc_value );

		/* 3, 若数据交换完成，更新指定通道                                    */
		if ( COMPLETE == g_ads1118[chipx].state ) {
			g_ads1118[chipx].chx = \
				(ads1118_ch_no_t)((g_ads1118[chipx].chx + 1) % ADS1118_CH_NUM);
		}
		
	} // end for
	
}


/*******************************************************************************
 * @brief ads1118获取温度
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_GetTemperature(float temperature[][ADS1118_CH_NUM - 1])
{
	for ( uint8_t chipx = 0; chipx < ADS1118_NUM; chipx++ ) {
		
		for ( uint8_t chx = 0; chx < ADS1118_CH_NUM - 1; chx++ ) {
			
			/* 1, 计算电压和: 1.1, 计算冷端温度
		                      1.2, 二分查表找 冷端电压 对应的 电压温度表 索引
	                          1.3, 计算电压和（冷端电压 + 工作端电压）        */
			float cold_temperature = \
				__ADC_TO_TEMPERATURE(g_ads1118[chipx].ch[ADS1118_CH_COLD].adc_value);

			uint8_t index = \
				_binary_search(g_temperature_arr, 16, cold_temperature);
			
			float voltage = \
				( __TEMPERATURE_TO_VOLTAGE(cold_temperature, index)    	) + \
				( __ADC_TO_VOLTAGE(g_ads1118[chipx].ch[chx].adc_value) );
			
			/* 2, 计算最终温度: 2.1, 二分查表找索引
	                            2.2, 若索引为零，工作端为室温，赋值冷端温度
                                     若索引不为零，将电压和转换为温度值并赋值 */
			index = _binary_search(g_voltage_arr, 16, voltage);
			
			float temperature_before_filter = ( ( 0 == index ) ? \
				cold_temperature : (__VOLTAGE_TO_TEMPERATURE(voltage, index)));
			
			/* 3, 滤波                                                        */
			temperature[chipx][chx] = \
				g_kalman_filter(temperature_before_filter, chipx, chx);
			
			#if ( ADS1118_DEBUG == 1 )
			__ADS1118_LOG_INFO();
			#endif

		} // end for
		
	} // end for
}
