#ifndef __MODBUS_APP_H__
#define __MODBUS_APP_H__

/* ======================== ͷ�ļ� ========================================== */
#include <stdint.h>



/* ======================== �궨�� ========================================== */
#define R (1 << 0)
#define W (1 << 1)



/* ======================== �Զ������� ====================================== */
/* modbus�Ĵ���ʵ������ */
typedef struct {
	uint8_t property;                 // ��д����      
	uint16_t address;                 // �Ĵ�����ַ    
	uint16_t min_value;               // ��Сֵ        
	uint16_t max_value;               // ���ֵ        
	void (*readCb)(uint16_t *value); // �Ĵ�����������
	void (*writeCb)(uint16_t value); // �Ĵ���д������
} mb_register_instance_t;



/* ======================== �ӿ������� ====================================== */
void vModbus_AppInit(void);

void vModbus_Task(void);



#endif /* __MODBUS_APP_H__ */
