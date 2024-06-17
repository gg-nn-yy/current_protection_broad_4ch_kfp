#ifndef __ADS1118_DRIVER_H__
#define __ADS1118_DRIVER_H__

/* ======================== ͷ�ļ� ========================================== */
#include "stm32f0xx.h"



/* ======================== �궨�� ========================================== */
/* ADS1118ͨ������ */
#define ADS1118_SS_START        0X8000    //0--��NO effect,1-->start ADC

#define ADS1118_MUX_AIN0_AIN1   0X0000    //000 = AINP is AIN0 and AINN is AIN1 (default)
#define ADS1118_MUX_AIN0_AIN3   0x1000    //001 = AINP is AIN0 and AINN is AIN3
#define ADS1118_MUX_AIN1_AIN3   0X2000    //010 = AINP is AIN1 and AINN is AIN3
#define ADS1118_MUX_AIN2_AIN3   0X3000    //011 = AINP is AIN2 and AINN is AIN3
#define ADS1118_MUX_AIN0        0X4000    //100 = AINP is AIN0 and AINN is GND
#define ADS1118_MUX_AIN1        0X5000    //101 = AINP is AIN1 and AINN is GND
#define ADS1118_MUX_AIN2        0X6000    //110 = AINP is AIN2 and AINN is GND
#define ADS1118_MUX_AIN3        0X7000    //111 = AINP is AIN3 and AINN is GND

#define ADS1118_PGA_6144        0X0000    //000 = FSR is ��6.144 V
#define ADS1118_PGA_4096        0X0200    //001 = FSR is ��4.096
#define ADS1118_PGA_2048        0X0400    //010 = FSR is ��2.048 V (default)
#define ADS1118_PGA_1024        0X0600    //011 = FSR is ��1.024 V
#define ADS1118_PGA_0512        0X0800    //100 = FSR is ��0.512 V
#define ADS1118_PGA_0256        0X0A00    //101 = FSR is ��0.256 V
#define ADS1118_PGA_0256_1      0X0C00    //101 = FSR is ��0.256 V
#define ADS1118_PGA_0256_2		0X0E00	  //111 = FSR is ��0.256 V

#define ADS1118_Continuous_MODE 0X0000    //0->Continuous
#define ADS1118_Sigle_SHOT_MODE 0X0100    //1->SIGNEL ADC

#define ADS1118_DR_8SPS         0X0000    //000 = 8 SPS
#define ADS1118_DR_16SPS        0X0020    //001 = 16 SPS
#define ADS1118_DR_32SPS        0X0040    //010 = 32 SPS
#define ADS1118_DR_64SPS        0X0060    //011 = 64 SPS
#define ADS1118_DR_128SPS       0X0080    //100 = 128 SPS (default)
#define ADS1118_DR_250SPS       0X00A0    //101 = 250 SPS
#define ADS1118_DR_470SPS       0X00C0    //110 = 475 SPS
#define ADS1118_DR_860SPS       0X00E0    //111 = 860 SPS

#define ADS1118_ADC_MODE        0x0000    //ADC MODE
#define ADS1118_Temp_MODE       0X0010    //Temperature sensor mode

#define ADS1118_PUUP_DIS        0X0000    //inside pullup disabled
#define ADS1118_PUUP_EN         0x0008    //inside pullup enabled

#define ADS1118_NOP_UPDATA      0X0003    //update the Config register


/* ADS1118��ȡ����¶�ָ�ADS1118_Temp_MODE�� */
#define ADS1118_TEMP_CMD 	ADS1118_SS_START		|	\
							ADS1118_MUX_AIN0_AIN1 	|	\
							ADS1118_PGA_0256_2 		|	\
							ADS1118_Continuous_MODE |	\
							ADS1118_DR_128SPS 		|	\
							ADS1118_Temp_MODE 		|	\
							ADS1118_PUUP_EN			|	\
							ADS1118_NOP_UPDATA

/* ADS1118��ȡ01ͨ�����ADCֵָ�ADS1118_ADC_MODE��ADS1118_MUX_AIN0_AIN1�� */
#define ADS1118_CH01_ADC_CMD 	ADS1118_SS_START		|	\
								ADS1118_MUX_AIN0_AIN1 	|	\
								ADS1118_PGA_0256_2 		|	\
								ADS1118_Continuous_MODE |	\
								ADS1118_DR_128SPS 		|	\
								ADS1118_ADC_MODE 		|	\
								ADS1118_PUUP_EN			|	\
								ADS1118_NOP_UPDATA

/* ADS1118��ȡ23ͨ�����ADCֵָ�ADS1118_ADC_MODE��ADS1118_MUX_AIN2_AIN3�� */
#define ADS1118_CH23_ADC_CMD 	ADS1118_SS_START		|	\
								ADS1118_MUX_AIN2_AIN3 	|	\
								ADS1118_PGA_0256_2 		|	\
								ADS1118_Continuous_MODE |	\
								ADS1118_DR_128SPS 		|	\
								ADS1118_ADC_MODE 		|	\
								ADS1118_PUUP_EN			|	\
								ADS1118_NOP_UPDATA
								
								
#define ADS1118_BUF_SIZE 10  // �˲���������С
#define ADS1118_RUN_TIME 20 // ads1118ͨ��ת������ʱ��



/* ======================== �Զ������� ====================================== */
/* ads1118Ӳ�����������Ϣ���Ͷ��� */
typedef struct {
	SPI_TypeDef *	spi;
	uint32_t		spi_rcc;

	uint32_t		rcc;
	GPIO_TypeDef*	gpio;
	
	uint16_t		mosi_pin;
	uint16_t		miso_pin;
	uint16_t		sck_pin;
	uint16_t		nss_pin;
	
	uint16_t		cs0_pin;
	uint16_t		cs1_pin;
	uint16_t		cs2_pin;
	
} ads1118_drv_t;


/* ads1118ͨ�����ö�����Ͷ��� */
typedef enum {
	ADS1118_CH01_WORK = 0, 	// ͨ��01������
	ADS1118_CH23_WORK,		// ͨ��23������
	ADS1118_CH_COLD, 		// ���ͨ��
	ADS1118_CH_NUM,         // ͨ������
} ads1118_ch_no_t;


/* ads1118״̬ö�����Ͷ��� */
typedef enum {
	READY 		= 0,	// ����̬
	RUNING		= 1,	// ����̬
	COMPLETE	= 2,	// ���̬
} ads1118_state_t;


/* ads1118ͨ�������Ϣ���Ͷ��� */
typedef struct {
	const uint16_t 	spi_cmd;
	uint16_t 		adc_value;
} ads1118_ch_info_t;


/* ads1118���������Ϣ���Ͷ��� */
typedef struct {
	uint16_t	      nss_pin;						// Ƭѡ����
	
	/* ״̬�������Ϣ */
	ads1118_state_t	  state;                       	// оƬ״̬
	uint64_t    	  prv_sys_time;					// ϵͳʱ��
	
	/* ���������Ϣ */
	ads1118_ch_no_t   chx; 							// ��ǰ��ѡ�е�ͨ��
	ads1118_ch_info_t ch[ADS1118_CH_NUM];			// ͨ����Ϣ
} ads1118_info_t;



/* ======================== �ӿ������� ====================================== */
void vAds1118_DriverInit(void);

void vAds1118_FilterRegCb( float (*pf_filter)(float temperature, uint8_t chipx,
	                                           uint8_t chx) );
	
void vAds1118_GetTemperature(float temperature[][ADS1118_CH_NUM - 1]);

void vAds1118_Task(void);

#endif /* __ADS1118_DRIVER_H__ */
