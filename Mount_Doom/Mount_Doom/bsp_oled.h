/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-03-02

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-03-02

All rights reserved
***********************************************/

#ifndef __OLED_H
#define __OLED_H			  	 
#include "Header.h"







//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr() PBout(4)=0   //RST
#define OLED_RST_Set() PBout(4)=1   //RST

#define OLED_RS_Clr() PBout(3)=0    //DC
#define OLED_RS_Set() PBout(3)=1    //DC

#define OLED_SCLK_Clr()  PCout(14)=0  //SCL
#define OLED_SCLK_Set()  PCout(14)=1   //SCL

#define OLED_SDIN_Clr()  PBout(5)=0   //SDA
#define OLED_SDIN_Set()  PBout(5)=1   //SDA

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据





extern uint8_t OLED_GRAM[128][8];	 

//OLED控制用函数
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
void OLED_ShowNumber(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p);	
void OLED_Refresh_GRAM_Line(uint8_t line);

#endif  
	 
