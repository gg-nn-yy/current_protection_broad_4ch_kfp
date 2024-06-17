/* ======================== 头文件 ========================================== */
#include "led_driver.h"



/* ======================== 全局变量定义区 ================================== */
static led_hw_info_t g_leds_hw_info[] = {
	{RCC_AHBPeriph_GPIOA, GPIOA, GPIO_Pin_8},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_0},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_1},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_3},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_4},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_5},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_6},
	{RCC_AHBPeriph_GPIOB, GPIOB, GPIO_Pin_7},
};

#define LED_NUM_MAX	(sizeof(g_leds_hw_info) / sizeof(g_leds_hw_info[0]))



/* ======================== 宏函数定义区 ==================================== */



/* ======================== 局部接口定义区 ================================== */



/* ======================== 对外接口定义区 ================================== */
/*******************************************************************************
 * @brief  led硬件初始化
 *
 * @param  无
 *
 * @return 无
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vLed_DriverInit(void)
{
	for ( uint8_t i = 0; i < LED_NUM_MAX ; i++ ) {
		/* 使能端口时钟                                                       */
		RCC_AHBPeriphClockCmd(g_leds_hw_info[i].rcc, ENABLE);

		/* 引脚配置                                                           */
		GPIO_InitTypeDef  gpio_init_para;
		gpio_init_para.GPIO_Mode  = GPIO_Mode_OUT;  	 	// 输出
		gpio_init_para.GPIO_OType = GPIO_OType_PP;       	// 推挽
		gpio_init_para.GPIO_Pin   = g_leds_hw_info[i].pin;
		gpio_init_para.GPIO_PuPd  = GPIO_PuPd_NOPULL; 		// 无上下拉
		gpio_init_para.GPIO_Speed = GPIO_Speed_50MHz;	 	// IO口速度为50MHz
		GPIO_Init(g_leds_hw_info[i].gpio, &gpio_init_para);
	}
}


/*******************************************************************************
 * @brief  点亮LED
 *
 * @param  led_no：LED标号，0 - LED_NUM_MAX
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vLed_TurnOn(uint8_t led_no)
{
	if ( led_no >= LED_NUM_MAX ) {
		return;
	}
	
	GPIO_SetBits(g_leds_hw_info[led_no].gpio, g_leds_hw_info[led_no].pin);
}


/*******************************************************************************
 * @brief  熄灭LED
 *
 * @param  led_no：LED标号，0 - LED_NUM_MAX
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vLed_TurnOff(uint8_t led_no)
{
	if ( led_no >= LED_NUM_MAX ) {
		return;
	}
	
	GPIO_ResetBits(g_leds_hw_info[led_no].gpio , g_leds_hw_info[led_no].pin);
}


/*******************************************************************************
 * @brief  翻转LED
 *
 * @param  led_no：LED标号，0 - LED_NUM_MAX
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vLed_Toggle(uint8_t led_no)
{
	if ( led_no >= LED_NUM_MAX ) {
		return;
	}
	
	GPIO_WriteBit(
		g_leds_hw_info[led_no].gpio,
	    g_leds_hw_info[led_no].pin,
        (BitAction)(1 - GPIO_ReadOutputDataBit( g_leds_hw_info[led_no].gpio,
		                                       g_leds_hw_info[led_no].pin )));
}


/*******************************************************************************
 * @brief  LED测试任务
 *
 * @param 
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vLed_TestTask1(void)
{
	vLed_Toggle(LED1);
}

void vLed_TestTask2(void)
{
	vLed_Toggle(LED2);
}

void vLed_TestTask3(void)
{
	vLed_Toggle(LED3);
}

void vLed_TestTask4(void)
{
	vLed_Toggle(LED4);
}

void vLed_TestTask5(void)
{
	vLed_Toggle(LED5);
}

void vLed_TestTask6(void)
{
	vLed_Toggle(LED6);
}

void vLed_TestTask7(void)
{
	vLed_Toggle(LED7);
}

void vLed_TestTask8(void)
{
	vLed_Toggle(LED8);
}
