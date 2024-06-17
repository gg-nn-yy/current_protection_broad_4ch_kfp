#ifndef __DEBUG_H__
#define __DEBUG_H__

/* ======================== ͷ�ļ� ========================================== */



/* ======================== �꺯�������� ==================================== */
/* ��ӡADS1118��Ϣ */
#define __ADS1118_LOG_INFO()												\
	if ( (ADS1118_NUM - 1) != chipx || (ADS1118_CH_NUM - 2) != chx ) {	\
		continue;															\
	}																		\
																			\
	printf(	"%.1f,\t"														\
			"%.1f,\t"														\
			"%.1f,\t"														\
			"%.1f,\t"														\
			"%.1f,\t"														\
			"%d\n", 														\
						temperature[0][0],									\
						temperature[0][1],									\
						temperature[1][0],									\
						temperature[1][1],									\
						temperature_before_filter,							\
						150 )												\

	
/* ��ӡ��������Ϣ */
#define __FILTER_KALMAN_LOG_INFO()	\
	printf( "%.2f,\t"				\
		    "%.2f,\t"				\
	        "%.2f\n",				\
						kfp->P,		\
						kfp->P_, 	\
						kfp->K  )	\
						
						
/* ��ӡ�¶���Ϣ */
#define __SENSOR_TEMPERATURE_LOG_INFO()					\
printf(	"%.1f, \t"										\
		"%.1f, \t"										\
		"%.1f, \t"										\
		"%.1f  \n",										\
					temperature[0][ADS1118_CH01_WORK],	\
					temperature[0][ADS1118_CH23_WORK],	\
					temperature[1][ADS1118_CH01_WORK],	\
					temperature[1][ADS1118_CH23_WORK] )	\
					
					

/* ======================== �궨�� ========================================== */
#define KALMAN_DEBUG 	0
#define ADS1118_DEBUG 	1
#define SENSOR_DEBUG 	0

	

/* ======================== �Զ������� ====================================== */



/* ======================== �ӿ������� ====================================== */



#endif /* __DEBUG_H__ */
