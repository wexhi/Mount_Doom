#ifndef _BSP_SYSTICK_H
#define _BSP_SYSTICK_H

#include "main.h"


#define delay_ms  	SysTick_Delay_Ms
#define delay_us   	SysTick_Delay_Us

#define Delay_ms(x) Delay_us(100*x)	 //单位ms


void SysTick_Init(void);
void Delay_us(__IO uint32_t nTime);

void SysTick_Delay_Us( __IO uint32_t us);
void SysTick_Delay_Ms( __IO uint32_t ms);


#endif
