/* ------------------------���ļ�------------------------------------------- */
#include <stdint.h>
#include <string.h>



/* ------------------------����ͷ�ļ�--------------------------------------- */
#include "mb.h"
#include "mbutils.h"
#include "modbus_slave.h"



/* ------------------------Ӧ��ͷ�ļ�--------------------------------------- */



/* ------------------------ȫ�ֱ���������----------------------------------- */
static modbus_funcCb_t g_modbus_func_cb = {NULL};



/* ------------------------�꺯��������------------------------------------- */



/* ------------------------�ֲ��ӿڶ�����----------------------------------- */



/* ------------------------����ӿڶ�����----------------------------------- */
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
 * @brief  modbus�ӻ���ʼ��
 *
 * @param  ��
 *
 * @return ��
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
