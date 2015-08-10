#ifndef __SCCB_H
#define __SCCB_H

#include "ioCC2530.h"
#include "hal_types.h"

// this is for delay_ms and delay_us
#define NOP() asm("NOP")
// reverse for CC2530
// P2_3 connect to SDA, P2_4 connect to SCL
// A(10) B C D(13) E F(15)
// Input P2DIR ----0--- 0xF7  P2INP ----0---
#define SCCB_SDA_IN()  {P2SEL &= 0xFD; P2DIR &= 0xF7; P2INP &= 0x77;}
// OutPut P2DIR ---- 
#define SCCB_SDA_OUT() {P2SEL &= 0xFD; P2DIR |= 0x08; P2INP &= 0x77;}

//IO操作函数	 
#define SCCB_SCL    		P2_4	 	//SCL
#define SCCB_SDA    		P2_3 		//SDA	 

#define SCCB_READ_SDA    	P2_3  		//输入SDA    
#define SCCB_ID   			0X42  		//OV7670的ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
uint8 SCCB_WR_Byte(uint8 dat);
uint8 SCCB_RD_Byte(void);
uint8 SCCB_WR_Reg(uint8 reg,uint8 data);
uint8 SCCB_RD_Reg(uint8 reg);
void delay_ms(uint16 msec);
void delay_us(uint16 usec);
#endif

