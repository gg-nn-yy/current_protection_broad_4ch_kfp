#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
/* ------------------------���ļ�------------------------------------------- */
#include <stdint.h>



/* ------------------------����ͷ�ļ�--------------------------------------- */
#include "mb.h"



/* ------------------------Ӧ��ͷ�ļ�--------------------------------------- */



/* ------------------------�궨��------------------------------------------- */



/* ------------------------�Զ�������--------------------------------------- */
/* modbus�ص��������Ͷ��壬��������дӦ�ûص�����                            */
typedef struct
{
	eMBErrorCode (*pxRead_registers)( uint8_t start_address,
	                                   uint8_t regNum,
	                                   uint8_t *buf );
	
	eMBErrorCode (*pxWrite_registers)( uint8_t start_address,
	                                    uint8_t regNum,
	                                    uint8_t *buf );
} modbus_funcCb_t;


/* modbus�ӻ�ʵ�����Ͷ���                                                    */
typedef struct
{
	uint8_t slave_address;  	// �ӻ���ַ
	uint32_t baud_rate;  		// ������
	modbus_funcCb_t callback;  	// �ص�����
} modbus_slave_instance_t;



/* ------------------------�ӿ�������--------------------------------------- */
void vModbusSlaveInit(modbus_slave_instance_t *mb_instance);

#endif
