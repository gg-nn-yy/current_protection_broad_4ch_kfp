#ifndef __MODBUS_APP_H__
#define __MODBUS_APP_H__

/* ======================== 头文件 ========================================== */
#include <stdint.h>



/* ======================== 宏定义 ========================================== */
#define R (1 << 0)
#define W (1 << 1)



/* ======================== 自定义类型 ====================================== */
/* modbus寄存器实例类型 */
typedef struct {
	uint8_t property;                 // 读写属性      
	uint16_t address;                 // 寄存器地址    
	uint16_t min_value;               // 最小值        
	uint16_t max_value;               // 最大值        
	void (*readCb)(uint16_t *value); // 寄存器读控制项
	void (*writeCb)(uint16_t value); // 寄存器写控制项
} mb_register_instance_t;



/* ======================== 接口声明区 ====================================== */
void vModbus_AppInit(void);

void vModbus_Task(void);



#endif /* __MODBUS_APP_H__ */
