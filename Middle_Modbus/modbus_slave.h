#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
/* ------------------------库文件------------------------------------------- */
#include <stdint.h>



/* ------------------------驱动头文件--------------------------------------- */
#include "mb.h"



/* ------------------------应用头文件--------------------------------------- */



/* ------------------------宏定义------------------------------------------- */



/* ------------------------自定义类型--------------------------------------- */
/* modbus回调函数类型定义，包括读、写应用回调函数                            */
typedef struct
{
	eMBErrorCode (*pxRead_registers)( uint8_t start_address,
	                                   uint8_t regNum,
	                                   uint8_t *buf );
	
	eMBErrorCode (*pxWrite_registers)( uint8_t start_address,
	                                    uint8_t regNum,
	                                    uint8_t *buf );
} modbus_funcCb_t;


/* modbus从机实例类型定义                                                    */
typedef struct
{
	uint8_t slave_address;  	// 从机地址
	uint32_t baud_rate;  		// 波特率
	modbus_funcCb_t callback;  	// 回调函数
} modbus_slave_instance_t;



/* ------------------------接口声明区--------------------------------------- */
void vModbusSlaveInit(modbus_slave_instance_t *mb_instance);

#endif
