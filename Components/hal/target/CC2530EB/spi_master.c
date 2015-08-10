/*******************************************************************************
date   : 2010/08/20
writer ：on the way

Port   ：P0_2,P0_3,P0_4,P0_5  这四个端口是用于UART0外设配置
         P1_0，P1_1           这两个端口是LED1和LED2
         P1_4,P1_5,P1_6,P1_7  这四个端口用于SPI通信

fuction：SPI通信主模式
         配置好寄存器之后，直接进入发送数据的状态
*******************************************************************************/
/*-------------------------------------------------------------------------------- 
    Master                 Slave 
-------------          ------------- 
|           |          |           | 
|P1_4   SSN |--------->|SSN    P1_4| 
|           |          |           | 
|P1_5   SCK |--------->|SCK    P1_5| 
|           |          |           | 
|P1_6   MOSI|--------->|MOSI   P1_7| 
|           |          |           | 
|P1_7   MISO|<---------|MISO   P1_6| 
|           |          |           | 
-------------          ------------- 
--------------------------------------------------------------------------------*/
// Master Mode
#include "ioCC2530.h"
#include "ADXL345SPI.h"

#define SSN     P1_4

#define LOW 	0
#define HIGH 	1

typedef unsigned char byte;

/***************************Port Init OK*************************/
void init_port(void)
{
   PERCFG |= 0x02;        // PERCFG.U1CFG = 1 
   P1SEL |= 0xE0;         // P1_7, P1_6, and P1_5 are peripherals 
   P1SEL &= ~0x10;        // P1_4 is GPIO (SSN) 
   P1DIR |= 0x10;         // SSN is set as output 
}


// in cc 2530 will get 4 MHZ
// this means we must use the data rate of 3200 HZ
/********* init_Baudrate ********/
void init_Baudrate(void)
{
  // Set baud rate to max (system clock frequency / 8) 
  // Assuming a 26 MHz crystal (CC1110Fx/CC2510Fx), 
  // max baud rate = 26 MHz / 8 = 3.25 MHz.  
  U1BAUD = 0;   // BAUD_M = 0 
  U1GCR |= 15;   // BAUD_E = 17
}

/********************SPI Init Master******************/

void Init_Spi(void)
{
  init_port();      //初始化端口
  init_Baudrate();  //初始化波特率
  
  // SPI Master Mode 
  U1CSR &= ~0xA0;   //选择为SPI为Master
  
  // Configure phase, polarity, and bit order 
  // this  configure should be done before the 
  // adxl345 powered
  // if adxl345 is powers, you should keep the CS be high
  SSN = HIGH;
  //U1GCR &= ~0xC0;      // CPOL = CPHA = 0
  //U1GCR |= 0x20;       // ORDER = 1 
  
  // CPOL = CPHA = 1 ORDER = 1 
  U1GCR |= 0xC0;
  U1GCR |= 0x20;
}

void spiWait(byte counter)
{
  while(counter--);
}

/*********** Read a byte From ADXL345 *************/
void ADXL345Read(unsigned char multi, unsigned char len, unsigned char* buffer, unsigned char startReg)
{
  unsigned char i;
  // -------- 0/1 000 0000
  // --------- 1 for read
  unsigned char firstByte = 0x80;	
  
  if(1 == multi)
  {
	firstByte |= 0x40;
  }
	
  // fill reg address
  firstByte |= startReg;
  
  // send the reg address
  // select the slave 
  SSN = LOW;
  U1DBUF = firstByte; //加入要发送的数据
  while (!U1TX_BYTE);  
  U1TX_BYTE = 0;
  
  // prepare to read
  for (i = 0; i < len; i++) 
  { 
    while (U1RX_BYTE); 
	buffer[i] = U1DBUF;
    U1RX_BYTE = 0;
	
	if(!multi)
	{
	  break;
	}
  }
  
  // stop
  SSN = HIGH;
}

void ADXL345Write(unsigned char multi, unsigned char len, unsigned char* buffer, unsigned char startReg)
{
    unsigned char i;
  // -------- 0/1 000 0000
  // --------- 0 for write
  unsigned char firstByte = 0x00;	
  
  if(1 == multi)
  {
	firstByte |= 0x40;
  }
	
  // fill reg address
  firstByte |= startReg;
  
  // send the reg address
  // select the slave 
  SSN = LOW;
  U1DBUF = firstByte; 
  while (!U1TX_BYTE); 
  U1TX_BYTE = 0;
  
  // prepare to write
  for (i = 0; i < len; i++) 
  { 
	U1DBUF = buffer[i];
    while (!U1TX_BYTE); 
    U1TX_BYTE = 0;
	
	if(!multi)
	{
	  break;
	}
  }

  // stop
  SSN = HIGH;
}

void InitADXL345( void )
{
  Init_Spi();
  // use 4 -wrie mode
  // default is 4 wire so do nothing 
}