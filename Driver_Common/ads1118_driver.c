/**
 *******************************************************************************
 * @file    ads1118_driver.c
 * @author  gao_ning_yuan
 * @version V1.0
 * @date    2024 - 6 - 9
 * @brief   ads1118��������.c�ļ�
 *******************************************************************************
 * ���ļ�ʹ��STM32��׼�⣬ʵ����STM32F030Ӳ��SPI����ADS1118�����Ⱪ¶�Ľӿ��У�
 * vAds1118_DriverInit():     ��ʼ�������ӿ�
 * vAds1118_FilterRegCb():    �˲���ע��ص��ӿ�
 * fAds1118_GetTemperature(): ��ȡ�¶Ƚӿ�
 * vAds1118_Task():           ����ӿڣ����� main()���� �� �߳� �������Ե���
 * vAds1118_TestTask():       ��������ӿ�
 *
 * @attention
 * _x_ads1118_read_data()����ADS1118�������ݺ������ú�������״̬���ķ�ʽʵ�֣���
   vAds1118_Task()�ӿڵ���

 * ADS1118��������ͨ��SPl�ӿڷ��ʵļĴ�����ת���Ĵ��� �� ���üĴ���
   ת���Ĵ����������Ʋ����ʽ�������һ��ת���Ľ�����ϵ縴λ��ʼֵȫΪ�㡣
   ���üĴ����������û��޸�ADS1118�Ĺ���ģʽ�Ͳ�ѯ�豸��״̬��16λ���üĴ�������
               �ڿ���ADS1118�Ĺ���ģʽ������ѡ���������ʡ������̷�Χ���¶ȴ���
			   ��ģʽ��
 *******************************************************************************
 */



/* ======================== ͷ�ļ� ========================================== */
#include <stdio.h>
#include <stddef.h>
#include "stm32f0xx.h"

#include "debug.h"
#include "systick.h"

#include "ads1118_driver.h"



/* ======================== ȫ�ֱ��������� ================================== */
/* ��ѹ�ֶȱ���λmv */
static const float g_voltage_arr[16] = {
	0,     0.317, 1.735, 2.147, 2.96, 3.184, 3.391, 3.806, 
	4.095, 4.633, 6.058, 8.86,  11.8, 14.74, 17.7,  20.64
};


/* �¶ȷֶȱ���λ�� */
static const float g_temperature_arr[16] = {
	0,      8.00,   43.00,  58.00,  72.61,  78.00,  83.00,  93.00,
	100.00, 113.00, 148.00, 217.99, 290.15, 360.64, 430.78, 500
};


/* ads1118Ӳ�����������Ϣ */
static ads1118_drv_t g_ads1118_hw = {
	.spi 		= SPI1,
	.spi_rcc	= RCC_APB2Periph_SPI1,
	
	.rcc		= RCC_AHBPeriph_GPIOA,
	.gpio		= GPIOA,
	
	.mosi_pin	= GPIO_Pin_7,  	// ���������������
	.miso_pin	= GPIO_Pin_6,  	// ���������������
	.sck_pin	= GPIO_Pin_5,  	// ���������������
	.nss_pin	= GPIO_Pin_11,	// �������
	.cs0_pin	= GPIO_Pin_11,  // �������
	.cs1_pin	= GPIO_Pin_12, 	// �������
	.cs2_pin	= GPIO_Pin_4, 	// �������
};


/* ads1118������� */
static ads1118_info_t g_ads1118[] = {
	[0] = {
		.nss_pin 				= GPIO_Pin_11,
		.state   				= READY,
		.prv_sys_time 			= 0,
		.chx  					= ADS1118_CH_COLD,
		.ch[ADS1118_CH_COLD]  	= {
			.spi_cmd 		= ADS1118_TEMP_CMD, 		// ��ȡ�¶�ָ��
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH01_WORK]  = {
			.spi_cmd 		= ADS1118_CH01_ADC_CMD,	// ��ȡ01ͨ�����ADCֵָ��
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH23_WORK]  = {
			.spi_cmd 		= ADS1118_CH23_ADC_CMD,	// ��ȡ23ͨ�����ADCֵָ��
			.adc_value 		= 0,
		},
	},
	
	[1] = {
		.nss_pin 				= GPIO_Pin_12,
		.state   				= READY,
		.prv_sys_time 			= 0,
		.chx  					= ADS1118_CH_COLD,
		
		.ch[ADS1118_CH_COLD]	= {
			.spi_cmd 		= ADS1118_TEMP_CMD, 		// ��ȡ�¶�ָ��
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH01_WORK]  = {
			.spi_cmd 		= ADS1118_CH01_ADC_CMD,	// ��ȡ01ͨ�����ADCֵָ��
			.adc_value 		= 0,
		},
		
		.ch[ADS1118_CH23_WORK]  = {
			.spi_cmd 		= ADS1118_CH23_ADC_CMD,	// ��ȡ23ͨ�����ADCֵָ��
			.adc_value 		= 0,
		},
	},
	
};

#define ADS1118_NUM    ( sizeof(g_ads1118) / sizeof(g_ads1118[0]) )

static float (*g_kalman_filter)(float temp, uint8_t chipx, uint8_t chx) = NULL;



/* ======================== �꺯�������� ==================================== */
/* Ƭѡ�� */
#define __SPI_W_NSS(x) \
GPIO_WriteBit(g_ads1118_hw.gpio, g_ads1118_hw.nss_pin, (BitAction)x)

/* SPI��ʼ �� ���� */
#define __SPI_START()              __SPI_W_NSS(0)
#define __SPI_STOP()               __SPI_W_NSS(1)


/*******************************************************************************
 * @brief �� ADCֵ ת��Ϊ �¶�ֵ
 * @param[in]  adc: adcֵ
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __ADC_TO_TEMPERATURE(adc)											\
( adc & 0x8000 ) ?                                                        	\
	(-(~((adc >> 2) - 1) & 0x3fff) * 0.03125f) : ((adc >> 2) * 0.03125f)


/*******************************************************************************
 * @brief �� ADCֵ ת��Ϊ ��ѹֵ
 * @param[in]  adc: adcֵ
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __ADC_TO_VOLTAGE(adc)												\
( adc & 0x8000 ) ? (-(~(adc - 1)) * 7.8125f / 1000) : (adc * 7.8125f / 1000)


/*******************************************************************************
 * @brief �� �¶�ֵ ת��Ϊ ��ѹֵ
 * @param[in] temp: �¶�ֵ
 * @param[in] index: ����
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __TEMPERATURE_TO_VOLTAGE(temp, index) 								\
 g_voltage_arr[index - 1] +	 												\
(g_voltage_arr[index] - g_voltage_arr[index - 1]) * 						\
	( (temp - g_temperature_arr[index - 1]) /                             	\
	  (g_temperature_arr[index] - g_temperature_arr[index-1]) )


/*******************************************************************************
 * @brief �� ��ѹֵ ת��Ϊ �¶�ֵ
 * @param[in] voltage: ��ѹֵ
 * @param[in] index: ����
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define __VOLTAGE_TO_TEMPERATURE(voltage, index) 							\
g_temperature_arr[index-1] +												\
(g_temperature_arr[index] - g_temperature_arr[index - 1]) * 				\
	( (voltage - g_voltage_arr[index-1]) /                                	\
	  (g_voltage_arr[index] - g_voltage_arr[index - 1]) )



/* ======================== �ֲ��ӿڶ����� ================================== */
/*******************************************************************************
 * @brief GPIO��ʼ��
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _gpio_init(void)
{
	/* ����ʱ��                                                               */
	RCC_AHBPeriphClockCmd(g_ads1118_hw.rcc, ENABLE);
	
	/* ��������                                                               */
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(g_ads1118_hw.gpio, GPIO_PinSource7, GPIO_AF_0);
	
	/* MISO_PIN��MOSI_PIN��SCK_PIN ����������� */
	GPIO_InitTypeDef  gpio_init_para;
	gpio_init_para.GPIO_Pin   = g_ads1118_hw.miso_pin 	|
								g_ads1118_hw.mosi_pin 	|
								g_ads1118_hw.sck_pin;	
	gpio_init_para.GPIO_Mode  = GPIO_Mode_AF;  	 		// ����
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// ����
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// ��������
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO���ٶ�Ϊ50MHz
	GPIO_Init(g_ads1118_hw.gpio, &gpio_init_para);

	
	/* NSS_PIN��CS0_PIN��CS1_PIN��CS2_PIN ������� */
	gpio_init_para.GPIO_Pin   = g_ads1118_hw.nss_pin	|
								g_ads1118_hw.cs0_pin	|
								g_ads1118_hw.cs1_pin	|
								g_ads1118_hw.cs2_pin;
	gpio_init_para.GPIO_Mode  = GPIO_Mode_OUT;  	 	// ���
	gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// ����
	gpio_init_para.GPIO_PuPd  = GPIO_PuPd_UP; 			// ����
	gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO���ٶ�Ϊ50MHz
	GPIO_Init(g_ads1118_hw.gpio, &gpio_init_para);
	
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.nss_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs0_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs1_pin);
	GPIO_SetBits(g_ads1118_hw.gpio, g_ads1118_hw.cs2_pin);
}


/*******************************************************************************
 * @brief Ӳ��SPI��ʼ��
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _spi_init(void)
{	
	/* ����ʱ��                                                               */
	RCC_APB2PeriphClockCmd(g_ads1118_hw.spi_rcc, ENABLE);
	
	/* Ӳ��SPI��ʼ������                                                      */
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
	
	/* ʹ��Ӳ��SPI                                                            */
	SPI_Cmd(g_ads1118_hw.spi, ENABLE);
	
	/* ��ʼĬ�ϲ�ѡ�дӻ�                                                     */
	__SPI_STOP();
}


/*******************************************************************************
 * @brief  SPI��д����
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
 * @brief  ads1118��ȡ���ݣ�����״̬����
 *
 * @param[in]  spi_cmd: spiͨ��ָ��
 * @param[out] p_state: оƬ״̬
 * @param[out] prv_sys_time; ϵͳʱ��
 * @param[out] p_data:  Ҫ�������ݵĵ�ַ
 *
 * @return ads1118״̬
		@retval READY                                                     ����̬
		@retval RUNING	                                                  ����̬
		@retval COMPLETE                                                  ���̬
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void _ads1118_read_data( uint16_t spi_cmd,  ads1118_state_t *p_state, 
	                            uint64_t *prv_sys_time,    uint16_t *p_data  )
{
	switch ( *p_state ) {
		
		/* ����̬���������ݣ���¼ϵͳʱ�䣬��ads1118��״̬�л�Ϊ����̬        */
		case READY:
			*p_data = _spi_swap_data(spi_cmd);
			*prv_sys_time = ulSysRunTimeGet();
			*p_state = RUNING;
			break;
		
		/* ����̬������ʱ�䴰������������ʱ�䴰�ͽ�ads1118��״̬�л�Ϊ���̬  */
		case RUNING:
			if ( ADS1118_RUN_TIME <= ulSysRunTimeGet() - *prv_sys_time ) {
				*p_state = COMPLETE;
			}
			break;
		
		/* ����̬����ads1118��״̬�л�Ϊ����̬                                */
		default:
			*p_state = READY;
			break;
		
	} // end switch
}


/*******************************************************************************
 * @brief ���ֲ�������� ��� �� ��ӽ��Ҵ���Ŀ��ֵ ��Ԫ�ص�����
 *
 * @param[in] array_to_search��Ҫ���ҵ�˳������򸡵������飩
 * @param[in] len_of_array�� �����С
 * @param[in] value_to_lookfor��Ҫ���ҵ�ֵ
 *
 * @return ���δ�ҵ�Ŀ��ֵ���򷵻��ұ߽���������ӽ�Ŀ��ֵ��Ԫ��������
           ���������û�д���Ŀ��ֵ��Ԫ�أ��򷵻� 0��
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



/* ======================== ����ӿڶ����� ================================== */
/*******************************************************************************
 * @brief ads1118������ʼ��
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
	
	/* �ȴ���Դ�ȶ�����ȷ��ͨ�縴λ��ɣ����ٵȴ�50��s                        */
	for (uint8_t i = 0; i < 255; i++) {
		__NOP();
	}
}

	
/*******************************************************************************
 * @brief �˲���ע��ص�����
 *
 * @param[out] pf_median_avg_filter;��λֵƽ���˲���ָ��
 * @param[out] pf_lowpass_filterr;һ�׵�ͨ�˲���ָ��
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_FilterRegCb( float (*pf_filter)(float temperature, uint8_t chipx,
	                                           uint8_t chx) ) 
{
	/* ע���˲���                                                             */
	g_kalman_filter = pf_filter;
}


/*******************************************************************************
 * @brief ads1118����ת������
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_Task(void)
{
	uint8_t last_chx = 0;

	for ( uint8_t chipx = 0; chipx < ADS1118_NUM; chipx++ ) {
		
		/* 1, �л�Ƭѡ�ߣ�ָ��оƬ                                            */
		g_ads1118_hw.nss_pin = g_ads1118[chipx].nss_pin;
		
		last_chx = (g_ads1118[chipx].chx + ADS1118_CH_NUM - 1) % ADS1118_CH_NUM;
		
		/* 2, ��������                                                        */
		_ads1118_read_data( g_ads1118[chipx].ch[g_ads1118[chipx].chx].spi_cmd,
							&g_ads1118[chipx].state,
							&g_ads1118[chipx].prv_sys_time,
							&g_ads1118[chipx].ch[last_chx].adc_value );

		/* 3, �����ݽ�����ɣ�����ָ��ͨ��                                    */
		if ( COMPLETE == g_ads1118[chipx].state ) {
			g_ads1118[chipx].chx = \
				(ads1118_ch_no_t)((g_ads1118[chipx].chx + 1) % ADS1118_CH_NUM);
		}
		
	} // end for
	
}


/*******************************************************************************
 * @brief ads1118��ȡ�¶�
 *
 * @param void
 *
 * @return void
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vAds1118_GetTemperature(float temperature[][ADS1118_CH_NUM - 1])
{
	for ( uint8_t chipx = 0; chipx < ADS1118_NUM; chipx++ ) {
		
		for ( uint8_t chx = 0; chx < ADS1118_CH_NUM - 1; chx++ ) {
			
			/* 1, �����ѹ��: 1.1, ��������¶�
		                      1.2, ���ֲ���� ��˵�ѹ ��Ӧ�� ��ѹ�¶ȱ� ����
	                          1.3, �����ѹ�ͣ���˵�ѹ + �����˵�ѹ��        */
			float cold_temperature = \
				__ADC_TO_TEMPERATURE(g_ads1118[chipx].ch[ADS1118_CH_COLD].adc_value);

			uint8_t index = \
				_binary_search(g_temperature_arr, 16, cold_temperature);
			
			float voltage = \
				( __TEMPERATURE_TO_VOLTAGE(cold_temperature, index)    	) + \
				( __ADC_TO_VOLTAGE(g_ads1118[chipx].ch[chx].adc_value) );
			
			/* 2, ���������¶�: 2.1, ���ֲ��������
	                            2.2, ������Ϊ�㣬������Ϊ���£���ֵ����¶�
                                     ��������Ϊ�㣬����ѹ��ת��Ϊ�¶�ֵ����ֵ */
			index = _binary_search(g_voltage_arr, 16, voltage);
			
			float temperature_before_filter = ( ( 0 == index ) ? \
				cold_temperature : (__VOLTAGE_TO_TEMPERATURE(voltage, index)));
			
			/* 3, �˲�                                                        */
			temperature[chipx][chx] = \
				g_kalman_filter(temperature_before_filter, chipx, chx);
			
			#if ( ADS1118_DEBUG == 1 )
			__ADS1118_LOG_INFO();
			#endif

		} // end for
		
	} // end for
}
