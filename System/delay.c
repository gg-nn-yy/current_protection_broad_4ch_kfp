/* ======================== 头文件 ========================================== */
#include "delay.h" 
#include "systick.h"



/* ======================== 全局变量定义区 ================================== */



/* ======================== 宏函数定义区 ==================================== */



/* ======================== 局部接口定义区 ================================== */



/* ======================== 对外接口定义区 ================================== */



/******************************************************************************
 * @brief 毫秒级延时函数
 * @param nMs，延时时间n毫秒
 * @return 	无
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vDelayNms(uint32_t nMs)
{
	uint64_t sys_time = ulSysRunTimeGet();

	while ( (ulSysRunTimeGet() - sys_time) <= nMs) {}
}