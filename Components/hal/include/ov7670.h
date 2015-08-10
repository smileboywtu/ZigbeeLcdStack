#ifndef _OV7670_H
#define _OV7670_H
#include "sccb.h"

#define OV7670_WRST		P0_0		//写指针复位
#define OV7670_RCK		P0_5		//读数据时钟
#define OV7670_RRST		P0_1  		//读指针复位
#define OV7670_CS		P0_6    	//片选信号(OE)
#define OV7670_WREN		P0_4		//写入FIFO使能
#define OV7670_VSYNC  	        P2_0		//同步信号检测IO
															  					 
#define OV7670_DATA   		P1&0xFF	        //数据输入端口
/////////////////////////////////////////									
	    				 
uint8   OV7670_Init(void);		  	   		 
void OV7670_Light_Mode(uint8 mode);
void OV7670_Color_Saturation(uint8 sat);
void OV7670_Brightness(uint8 bright);
void OV7670_Contrast(uint8 contrast);
void OV7670_Special_Effects(uint8 eft);
void OV7670_Window_Set(uint16 sx,uint16 sy,uint16 width,uint16 height);


#endif





















