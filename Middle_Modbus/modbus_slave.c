/* ------------------------库文件------------------------------------------- */
#include <stdint.h>
#include <string.h>



/* ------------------------驱动头文件--------------------------------------- */
#include "mb.h"
#include "mbutils.h"
#include "modbus_slave.h"



/* ------------------------应用头文件--------------------------------------- */



/* ------------------------全局变量定义区----------------------------------- */
static modbus_funcCb_t g_modbus_func_cb = {NULL};



/* ------------------------宏函数定义区------------------------------------- */



/* ------------------------局部接口定义区----------------------------------- */



/* ------------------------对外接口定义区----------------------------------- */
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer,    USHORT usAddress,
                              USHORT usNRegs,    eMBRegisterMode eMode )
{
	eMBErrorCode state;
	
	if (MB_REG_READ == eMode) {
		state = g_modbus_func_cb.pxRead_registers( usAddress,
		                                            usNRegs,
		                                            pucRegBuffer );
	} else {
		state = g_modbus_func_cb.pxWrite_registers( usAddress,
		                                             usNRegs,
		                                             pucRegBuffer );
	}
	
	return state;
}


/******************************************************************************
 * @brief  modbus从机初始化
 *
 * @param  无
 *
 * @return 无
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vModbusSlaveInit(modbus_slave_instance_t *mb_instance)
{
	eMBInit( MB_RTU,
	         mb_instance->slave_address,
	         0,
	         mb_instance->baud_rate,
	         MB_PAR_NONE );
	
	eMBEnable();
//	g_modbusFuncCb.ReadRegs  = mbInstance->cb.ReadRegs;
//	g_modbusFuncCb.WriteRegs = mbInstance->cb.WriteRegs;
	g_modbus_func_cb = mb_instance->callback;
}
