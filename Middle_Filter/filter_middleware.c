/* ======================== 头文件 ========================================== */
#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"                  // Device header

#include "debug.h"

#include "ads1118_driver.h"
#include "filter_middleware.h"


/* ======================== 全局变量定义区 ================================== */
static kfp_t g_kfp_filter[2][2] = {
	/*        P_,  P,  out, K,  Q,    R */
    [0][0] = {0,  0,  0,   0,  0.1,  1},
	[0][1] = {0,  0,  0,   0,  0.1,  1},
	[1][0] = {0,  0,  0,   0,  0.1,  1},
	[1][1] = {0,  0,  0,   0,  0.1,  1},
	
};



/* ======================== 宏函数定义区 ==================================== */



/* ======================== 局部接口定义区 ================================== */
/*******************************************************************************
 * @brief 卡尔曼滤波器，状态转移矩阵为1，状态检测转换矩阵为1
 *
 * @param
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static float _f_kalman_filter(kfp_t *kfp, float input)
{
    if ( 0 == kfp->out ) {
        kfp->out = input;
        return input;
    }
	
	/* 1, 预测 1.1, 先验估计
	           1.2, 先验估计协方差 = 上一时刻的先验估计协方差 + 过程噪声协方差*/
    kfp->P = kfp->P_ + kfp->Q;
	
	/* 2, 更新                                                                */
    /* 2.1, 更新卡尔曼增益方程：卡尔曼增益 = 
            k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）      
	        Kk = ( Pk_ * Ct ) / ( C * Pk_ * Ct + R )                          */
    kfp->K = kfp->P / ( kfp->P + kfp->R );
	
    /* 2.2, 更新最优值方程：k时刻状态变量的最优值 = 
            先验估计值 + 卡尔曼增益 * （测量值 - 先验估计值）                   
	        Xk = Xk_ + Kk * ( Yk - C * Xk_ )                                  */
    kfp->out += kfp->K * ( input - kfp->out );
	
    /* 2.3, 更新协方差方程：Pk = ( I - Kk * C ) * Pk_                         */
    kfp->P_ = ( 1 - kfp->K ) * kfp->P;
	
	#if ( KALMAN_DEBUG == 1 )
	__FILTER_KALMAN_LOG_INFO();
	#endif
	
    /* 3, 输出滤波结果                                                        */
    return kfp->out;
}


static float _filter(float input, uint8_t chipx, uint8_t chx)
{
	return (_f_kalman_filter(&g_kfp_filter[chipx][chx], input));
}


	
/* ======================== 对外接口定义区 ================================== */
/*******************************************************************************
 * @brief 初始化，为各个模块注册滤波器
 *
 * @param void
 *
 * @return 
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void vFilter_Init(void)
{
	vAds1118_FilterRegCb( _filter );
}
