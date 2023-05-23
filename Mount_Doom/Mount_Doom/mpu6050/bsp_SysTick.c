  
#include "bsp_SysTick.h"
#include "core_cm3.h"
//#include "misc.h"

static __IO uint32_t TimingDelay;

/**************************************************************************
Function: SysTick_Init
Input   : none
Output  : none
�������ܣ���શ�ʱ����ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0��汾
	if (SysTick_Config(SystemCoreClock / 100000))	// ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
}


/**************************************************************************
Function: Delay_us
Input   : nTime
Output  : none
�������ܣ�us��ʱ����
��ڲ���: us��ʱ����10usΪһ����λ 
����  ֵ����
**************************************************************************/	 	
//Delay_us( 1 ) ��ʵ�ֵ���ʱΪ 1 * 10us = 10us
void Delay_us(__IO uint32_t nTime)
{ 
	TimingDelay = nTime;	

	// ʹ�ܵδ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

/**************************************************************************
Function: TimingDelay_Decrement
Input   : none
Output  : none
�������ܣ���ȡ���ĳ���
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
}

#if 0
// ��� �̼��⺯�� �� core_cm3.h��
static __INLINE uint32_t SysTick_Config(uint32_t ticks)
{ 
  // reload �Ĵ���Ϊ24bit�����ֵΪ2^24
	if (ticks > SysTick_LOAD_RELOAD_Msk)  return (1);
  
  // ���� reload �Ĵ����ĳ�ʼֵ	
  SysTick->LOAD  = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;
	
	// �����ж����ȼ�Ϊ 1<<4-1 = 15�����ȼ�Ϊ���
  NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1); 
	
	// ���� counter ��������ֵ
  SysTick->VAL   = 0;
	
	// ����systick ��ʱ��Ϊ 72M
	// ʹ���ж�
	// ʹ��systick
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | 
                   SysTick_CTRL_TICKINT_Msk   | 
                   SysTick_CTRL_ENABLE_Msk;                    
  return (0); 
}
#endif


/**************************************************************************
Function: SysTick_Delay_Us
Input   : us 
Output  : none
�������ܣ�us��ʱ����
��ڲ���: us��ʱʱ�� 
����  ֵ����
**************************************************************************/	 	
// couter ��1��ʱ�� ���� 1/systick_clk
// ��counter �� reload ��ֵ��С��0��ʱ��Ϊһ��ѭ��������������ж���ִ���жϷ������
// ͬʱ CTRL �� countflag λ����1
// ��һ��ѭ����ʱ��Ϊ reload * (1/systick_clk)

void SysTick_Delay_Us( __IO uint32_t us)
{
	uint32_t i;
	SysTick_Config(SystemCoreClock/1000000);
	
	for(i=0;i<us;i++)
	{
		// ����������ֵ��С��0��ʱ��CRTL�Ĵ�����λ16����1	
		while( !((SysTick->CTRL)&(1<<16)) );
	}
	// �ر�SysTick��ʱ��
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


/**************************************************************************
Function: SysTick_Delay_Ms
Input   : ms 
Output  : none
�������ܣ�ms��ʱ����
��ڲ���: ms��ʱʱ�� 
����  ֵ����
**************************************************************************/	 	

void SysTick_Delay_Ms( __IO uint32_t ms)
{
	uint32_t i;	
	SysTick_Config(SystemCoreClock/1000);
	
	for(i=0;i<ms;i++)
	{
		// ����������ֵ��С��0��ʱ��CRTL�Ĵ�����λ16����1
		// ����1ʱ����ȡ��λ����0
		while( !((SysTick->CTRL)&(1<<16)) );
	}
	// �ر�SysTick��ʱ��
	SysTick->CTRL &=~ SysTick_CTRL_ENABLE_Msk;
}


/*********************************************END OF FILE**********************/
