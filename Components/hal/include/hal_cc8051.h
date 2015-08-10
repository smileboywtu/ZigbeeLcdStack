/******************************************************************************
    Filename: hal_cc8051.h

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#ifndef HAL_CC8051_H
#define HAL_CC8051_H

/******************************************************************************
 * INCLUDES
 */

/******************************************************************************
 * CONSTANTS
 */
#define BIT0              0x01
#define BIT1              0x02
#define BIT2              0x04
#define BIT3              0x08
#define BIT4              0x10
#define BIT5              0x20
#define BIT6              0x40
#define BIT7              0x80


/******************************************************************************
 * MACROS
 */

#define MCU_IO_TRISTATE   1             // Used as "func" for the macros below
#define MCU_IO_PULLUP     2
#define MCU_IO_PULLDOWN   3


//-----------------------------------------------------------------------------
//  Macros for simple configuration of IO pins on TI LPW SoCs
//-----------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL(port, pin)   MCU_IO_PERIPHERAL_PREP(port, pin)
#define MCU_IO_INPUT(port, pin, func)  MCU_IO_INPUT_PREP(port, pin, func)
#define MCU_IO_OUTPUT(port, pin, val)  MCU_IO_OUTPUT_PREP(port, pin, val)
#define MCU_IO_SET(port, pin, val)     MCU_IO_SET_PREP(port, pin, val)
#define MCU_IO_SET_HIGH(port, pin)     MCU_IO_SET_HIGH_PREP(port, pin)
#define MCU_IO_SET_LOW(port, pin)      MCU_IO_SET_LOW_PREP(port, pin)
#define MCU_IO_TGL(port, pin)          MCU_IO_TGL_PREP(port, pin)
#define MCU_IO_GET(port, pin)          MCU_IO_GET_PREP(port, pin)

#define MCU_IO_DIR_INPUT(port, pin)    MCU_IO_DIR_INPUT_PREP(port, pin)
#define MCU_IO_DIR_OUTPUT(port, pin)   MCU_IO_DIR_OUTPUT_PREP(port, pin)


//----------------------------------------------------------------------------------
//  Macros for internal use (the macros above need a new round in the preprocessor)
//----------------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL_PREP(port, pin)   st( P##port##SEL |= BM(pin); )

#define MCU_IO_INPUT_PREP(port, pin, func)  st( P##port##SEL &= ~BM(pin); \
                                                P##port##DIR &= ~BM(pin); \
                                                switch (func) { \
                                                case MCU_IO_PULLUP: \
                                                    P##port##INP &= ~BM(pin); \
                                                    P2INP &= ~BM(port + 5); \
                                                    break; \
                                                case MCU_IO_PULLDOWN: \
                                                    P##port##INP &= ~BM(pin); \
                                                    P2INP |= BM(port + 5); \
                                                    break; \
                                                default: \
                                                    P##port##INP |= BM(pin); \
                                                    break; } )

#define MCU_IO_OUTPUT_PREP(port, pin, val)  st( P##port##SEL &= ~BM(pin); \
                                                P##port##_##pin## = val; \
                                                P##port##DIR |= BM(pin); )
//端口状态的设置
#define MCU_IO_SET_HIGH_PREP(port, pin)     st( P##port##_##pin## = 1; )
#define MCU_IO_SET_LOW_PREP(port, pin)      st( P##port##_##pin## = 0; )

#define MCU_IO_SET_PREP(port, pin, val)     st( P##port##_##pin## = val; )
#define MCU_IO_TGL_PREP(port, pin)          st( P##port##_##pin## ^= 1; )
#define MCU_IO_GET_PREP(port, pin)          (P##port## & BM(pin))
//端口方向的设置
#define MCU_IO_DIR_INPUT_PREP(port, pin)    st( P##port##DIR &= ~BM(pin); )
#define MCU_IO_DIR_OUTPUT_PREP(port, pin)   st( P##port##DIR |= BM(pin); )

//以下是参考cc2430的程序而改写的
/******************************************************************************
*******************              Commonly used types        *******************
******************************************************************************/
typedef unsigned char       BOOL;

// Data
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

// Unsigned numbers
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

// Signed numbers
typedef signed char         INT8;
typedef signed short        INT16;
typedef signed long         INT32;

typedef signed   char   int8;
typedef unsigned char   uint8;

typedef signed   short  int16;
typedef unsigned short  uint16;

typedef signed   long   int32;
typedef unsigned long   uint32;


// Common values
#ifndef FALSE
   #define FALSE 0
#endif

#ifndef TRUE
   #define TRUE 1
#endif

#ifndef NULL
   #define NULL 0
#endif

#ifndef HIGH
   #define HIGH 1
#endif

#ifndef LOW
   #define LOW 0
#endif

/******************************************************************************
*******************        Bit, byte and word macros        *******************
******************************************************************************/

#define UPPER_BYTE(a) ((BYTE) (((WORD)(a)) >> 8))
#define HIBYTE(a) UPPER_BYTE(a)

#define LOWER_BYTE(a) ((BYTE) ( (WORD)(a))      )
#define LOBYTE(a) LOWER_BYTE(a)

#define SET_WORD(regH, regL, word) \
   do{                             \
      (regH) = UPPER_BYTE((word)); \
      (regL) = LOWER_BYTE((word)); \
   }while(0)

#define READ_RFR16(reg) ((((WORD) ##reg##H) << 8) + ##reg##L)
#define WRITE_RFR16(reg, value) do { ##reg##H = HIBYTE(value); ##reg##L = LOBYTE(value); } while (0)

#define GET_WORD(regH, regL, word) \
   do{                             \
      word = (WORD)regH << 8;      \
      word |= regL;                \
   }while(0)


// Macros for configuring IO peripheral location:
// Example usage:
//   IO_PER_LOC_TIMER1_AT_PORT0_PIN234();
//   IO_PER_LOC_TIMER4_AT_PORT2_PIN03();
//   IO_PER_LOC_USART1_AT_PORT0_PIN2345();

#define IO_PER_LOC_TIMER1_AT_PORT0_PIN234() do { PERCFG = (PERCFG&~0x40)|0x00; } while (0)
#define IO_PER_LOC_TIMER1_AT_PORT1_PIN012() do { PERCFG = (PERCFG&~0x40)|0x40; } while (0)

#define IO_PER_LOC_TIMER3_AT_PORT1_PIN34()  do { PERCFG = (PERCFG&~0x20)|0x00; } while (0)
#define IO_PER_LOC_TIMER3_AT_PORT1_PIN67()  do { PERCFG = (PERCFG&~0x20)|0x20; } while (0)

#define IO_PER_LOC_TIMER4_AT_PORT1_PIN01()  do { PERCFG = (PERCFG&~0x10)|0x00; } while (0)
#define IO_PER_LOC_TIMER4_AT_PORT2_PIN03()  do { PERCFG = (PERCFG&~0x10)|0x10; } while (0)

#define IO_PER_LOC_SPI1_AT_PORT0_PIN2345()  do { PERCFG = (PERCFG&~0x08)|0x00; } while (0)
#define IO_PER_LOC_SPI1_AT_PORT1_PIN4567()  do { PERCFG = (PERCFG&~0x08)|0x08; } while (0)

#define IO_PER_LOC_SPI0_AT_PORT0_PIN2345()  do { PERCFG = (PERCFG&~0x04)|0x00; } while (0)
#define IO_PER_LOC_SPI0_AT_PORT1_PIN2345()  do { PERCFG = (PERCFG&~0x04)|0x04; } while (0)

#define IO_PER_LOC_UART1_AT_PORT0_PIN2345() do { PERCFG = (PERCFG&~0x02)|0x00; } while (0)
#define IO_PER_LOC_UART1_AT_PORT1_PIN4567() do { PERCFG = (PERCFG&~0x02)|0x02; } while (0)

#define IO_PER_LOC_UART0_AT_PORT0_PIN2345() do { PERCFG = (PERCFG&~0x01)|0x00; } while (0)
#define IO_PER_LOC_UART0_AT_PORT1_PIN2345() do { PERCFG = (PERCFG&~0x01)|0x01; } while (0)


// Actual MCU pin configuration:
//
// Peripheral I/O signal     Alt1       Alt2
// -------------------------------------------
// Timer1 channel0           P0.2       P1.2
// Timer1 channel1           P0.3       P1.1
// Timer1 channel2           P0.4       P1.0
// Timer3 channel0           P1.3       P1.6
// Timer3 channel1           P1.4       P1.7
// Timer4 channel0           P1.0       P2.0
// Timer4 channel1           P1.1       P2.3
// USART0 TXD/MOSI           P0.3       P1.5
// USART0 RXD/MISO           P0.2       P1.4
// USART0 RTS/SCK            P0.5       P1.3
// USART0 CTS/SS_N           P0.4       P1.2
// USART1 TXD/MOSI           P0.4       P1.6
// USART1 RXD/MISO           P0.5       P1.7
// USART1 RTS/SCK            P0.3       P1.5
// USART1 CTS/SS_N           P0.2       P1.4


// Macros for configuring IO direction:
// Example usage:
//   IO_DIR_PORT_PIN(0, 3, IO_IN);    // Set P0_3 to input
//   IO_DIR_PORT_PIN(2, 1, IO_OUT);   // Set P2_1 to output

#define IO_DIR_PORT_PIN(port, pin, dir)  \
   do {                                  \
      if (dir == IO_OUT)                 \
         P##port##DIR |= (0x01<<(pin));  \
      else                               \
         P##port##DIR &= ~(0x01<<(pin)); \
   }while(0)

// Where port={0,1,2}, pin={0,..,7} and dir is one of:
#define IO_IN   0
#define IO_OUT  1

// Macros for configuring IO input mode:
// Example usage:
//   IO_IMODE_PORT_PIN(0,0,IO_IMODE_PUD);
//   IO_IMODE_PORT_PIN(2,0,IO_IMODE_TRI);
//   IO_IMODE_PORT_PIN(1,3,IO_IMODE_PUD);

#define IO_IMODE_PORT_PIN(port, pin, imode) \
   do {                                     \
      if (imode == IO_IMODE_TRI)            \
         P##port##INP |= (0x01<<(pin));     \
      else                                  \
         P##port##INP &= ~(0x01<<(pin));    \
   } while (0)

// where imode is one of:
#define IO_IMODE_PUD  0 // Pull-up/pull-down
#define IO_IMODE_TRI  1 // Tristate

// Macro for configuring IO drive mode:
// Example usage:
//   IO_PUD_PORT(0, IO_PULLUP);
//   IO_PUD_PORT(1, IO_PULLDOWN);
//   IO_PUD_PORT(2, IO_PULLUP);

#define IO_PUD_PORT(port, pud)        \
   do {                               \
      if (pud == IO_PULLDOWN)         \
         P2INP |= (0x01 << (port+5)); \
      else                            \
         P2INP &= ~(0x01 << (port+5));\
   } while (0)

#define IO_PULLUP          0
#define IO_PULLDOWN        1

// Macros for function select (General purpose I/O / Peripheral function):
// Example usage:
//   IO_FUNC_PORT_PIN(0, 0, IO_FUNC_PERIPH);
//   IO_FUNC_PORT_PIN(0, 1, IO_FUNC_GIO);
//   IO_FUNC_PORT_PIN(2, 3, IO_FUNC_PERIPH);

#define IO_FUNC_PORT_PIN(port, pin, func)  \
   do {                                    \
      if((port == 2) && (pin == 3)){       \
         if (func) {                       \
            P2SEL |= 0x02;                 \
         } else {                          \
            P2SEL &= ~0x02;                \
         }                                 \
      }                                    \
      else if((port == 2) && (pin == 4)){  \
         if (func) {                       \
            P2SEL |= 0x04;                 \
         } else {                          \
            P2SEL &= ~0x04;                \
         }                                 \
      }                                    \
      else{                                \
         if (func) {                       \
            P##port##SEL |= (0x01<<(pin)); \
         } else {                          \
            P##port##SEL &= ~(0x01<<(pin));\
        }                                  \
      }                                    \
   } while (0)

// where func is one of:
#define IO_FUNC_GIO     0 // General purpose I/O
#define IO_FUNC_PERIPH  1 // Peripheral function

// Macros for configuring the ADC input:
// Example usage:
//   IO_ADC_PORT0_PIN(0, IO_ADC_EN);
//   IO_ADC_PORT0_PIN(4, IO_ADC_DIS);
//   IO_ADC_PORT0_PIN(6, IO_ADC_EN);

#define IO_ADC_PORT0_PIN(pin, adcEn) \
   do {                              \
      if (adcEn)                     \
         APCFG |= (0x01<<pin);      \
      else                           \
         APCFG &= ~(0x01<<pin); }   \
   while (0)

// where adcEn is one of:
#define IO_ADC_EN           1 // ADC input enabled
#define IO_ADC_DIS          0 // ADC input disab

/**********************************功耗模式************************************/
// Macro for getting the clock division factor
#define CLKSPD  ( CLKCONCMD & 0x07 )

// Macro for getting the timer tick division factor
#define TICKSPD ((CLKCONCMD & 0x38) >> 3)


// Macro for setting power mode
//mode的取值为0,1,2,3
#define SET_POWER_MODE(mode)                   \
   do {                                        \
      if(mode == 0)        { SLEEPCMD &= ~0x03; } \
      else if (mode == 3)  { SLEEPCMD |= 0x03;  } \
      else { SLEEPCMD &= ~0x03; SLEEPCMD |= mode;  } \
      PCON |= 0x01;                            \
      asm("NOP");                              \
   }while (0)


/******************************************************************************
*******************       Interrupt functions/macros        *******************
******************************************************************************/

// Macros which simplify access to interrupt enables, interrupt flags and
// interrupt priorities. Increases code legibility.
// 对于中断的使能，中断标志位和中断优先级，增加了代码的可读性
//******************************************************************************

#define INT_ON   1
#define INT_OFF  0
#define INT_SET  1
#define INT_CLR  0

// Global interrupt enables
//IEN0，IEN1，IEN2这几个寄存器CC2430和CC2530
#define INT_GLOBAL_ENABLE(on) EA=(!!on)

#define DISABLE_ALL_INTERRUPTS() (IEN0 = IEN1 = IEN2 = 0x00)

#define INUM_RFERR 0
#define INUM_ADC   1
#define INUM_URX0  2
#define INUM_URX1  3
#define INUM_ENC   4
#define INUM_ST    5
#define INUM_P2INT 6
#define INUM_UTX0  7
#define INUM_DMA   8
#define INUM_T1    9
#define INUM_T2    10
#define INUM_T3    11
#define INUM_T4    12
#define INUM_P0INT 13
#define INUM_UTX1  14
#define INUM_P1INT 15
#define INUM_RF    16
#define INUM_WDT   17

#define NBR_OF_INTERRUPTS 18

// Macro used together with the INUM_* constants
// to enable or disable certain interrupts.
// Example usage:
//   INT_ENABLE(INUM_RFERR, INT_ON);
//   INT_ENABLE(INUM_URX0, INT_OFF);
//   INT_ENABLE(INUM_T1, INT_ON);
//   INT_ENABLE(INUM_T2, INT_OFF);
#define INT_ENABLE(inum, on)                        \
   do {                                             \
      if      (inum==INUM_RFERR) { RFERRIE = on; }  \
      else if (inum==INUM_ADC)   { ADCIE   = on; }  \
      else if (inum==INUM_URX0)  { URX0IE  = on; }  \
      else if (inum==INUM_URX1)  { URX1IE  = on; }  \
      else if (inum==INUM_ENC)   { ENCIE   = on; }  \
      else if (inum==INUM_ST)    { STIE    = on; }  \
      else if (inum==INUM_P2INT) { (on) ? (IEN2 |= 0x02) : (IEN2 &= ~0x02); } \
      else if (inum==INUM_UTX0)  { (on) ? (IEN2 |= 0x04) : (IEN2 &= ~0x04); } \
      else if (inum==INUM_DMA)   { DMAIE   = on; }  \
      else if (inum==INUM_T1)    { T1IE    = on; }  \
      else if (inum==INUM_T2)    { T2IE    = on; }  \
      else if (inum==INUM_T3)    { T3IE    = on; }  \
      else if (inum==INUM_T4)    { T4IE    = on; }  \
      else if (inum==INUM_P0INT) { P0IE    = on; }  \
      else if (inum==INUM_UTX1)  { (on) ? (IEN2 |= 0x08) : (IEN2 &= ~0x08); } \
      else if (inum==INUM_P1INT) { (on) ? (IEN2 |= 0x10) : (IEN2 &= ~0x10); } \
      else if (inum==INUM_RF)    { (on) ? (IEN2 |= 0x01) : (IEN2 &= ~0x01); } \
      else if (inum==INUM_WDT)   { (on) ? (IEN2 |= 0x20) : (IEN2 &= ~0x20); } \
   } while (0)

// Macro used together with the INUM_* constants
// to read the interrupt flags.
// Example usage:
//   if (INT_GETFLAG(INUM_URX0))
//     ...
//   while (!INT_GETFLAG(INUM_URX0));

#define INT_GETFLAG(inum) (                       \
   (inum==INUM_RFERR)       ? RFERRIF           : \
   (inum==INUM_ADC)         ? ADCIF             : \
   (inum==INUM_URX0)        ? URX0IF            : \
   (inum==INUM_URX1)        ? URX1IF            : \
   (inum==INUM_ENC)         ? ENCIF_0           : \
   (inum==INUM_ST)          ? STIF              : \
   (inum==INUM_P2INT)       ? P2IF              : \
   (inum==INUM_UTX0)        ? UTX0IF            : \
   (inum==INUM_DMA)         ? DMAIF             : \
   (inum==INUM_T1)          ? T1IF              : \
   (inum==INUM_T2)          ? T2IF              : \
   (inum==INUM_T3)          ? T3IF              : \
   (inum==INUM_T4)          ? T4IF              : \
   (inum==INUM_P0INT)       ? P0IF              : \
   (inum==INUM_UTX1)        ? UTX1IF            : \
   (inum==INUM_P1INT)       ? P1IF              : \
   (inum==INUM_RF)          ? S1CON &= ~0x03    : \
   (inum==INUM_WDT)         ? WDTIF             : \
   0                                              \
)

// Macro used to set or clear certain interrupt flags.
// Example usage:
//   INT_SETFLAG(INUM_URX0, INT_SET;
//   INT_SETFLAG(INUM_T3, INT_CLR);
#define INT_SETFLAG(inum, f)                     \
   do {                                          \
      if      (inum==INUM_RFERR) { RFERRIF= f; } \
      else if (inum==INUM_ADC)   { ADCIF  = f; } \
      else if (inum==INUM_URX0)  { URX0IF = f; } \
      else if (inum==INUM_URX1)  { URX1IF = f; } \
      else if (inum==INUM_ENC)   { ENCIF_1 = ENCIF_0 = f; } \
      else if (inum==INUM_ST)    { STIF  = f;  } \
      else if (inum==INUM_P2INT) { P2IF  = f;  } \
      else if (inum==INUM_UTX0)  { UTX0IF= f;  } \
      else if (inum==INUM_DMA)   { DMAIF = f;  } \
      else if (inum==INUM_T1)    { T1IF  = f;  } \
      else if (inum==INUM_T2)    { T2IF  = f;  } \
      else if (inum==INUM_T3)    { T3IF  = f;  } \
      else if (inum==INUM_T4)    { T4IF  = f;  } \
      else if (inum==INUM_P0INT) { P0IF  = f;  } \
      else if (inum==INUM_UTX1)  { UTX1IF= f;  } \
      else if (inum==INUM_P1INT) { P1IF  = f;  } \
      else if (inum==INUM_RF)    { (f) ? (S1CON |= 0x03) : (S1CON &= ~0x03); } \
      else if (inum==INUM_WDT)   { WDTIF = f;  } \
   } while (0)



// Macro for setting interrupt group priority
// Example usage:
//   INT_PRIORITY(RFERR_RF_DMA, 3);
#define INT_PRIORITY(group, pri)                      \
   do {                                               \
      if (pri == 0) { IP0 &= ~group; IP1 &= ~group; } \
      if (pri == 1) { IP0 |=  group; IP1 &= ~group; } \
      if (pri == 2) { IP0 &= ~group; IP1 |=  group; } \
      if (pri == 3) { IP0 |=  group; IP1 |=  group; } \
   } while (0)
// Where pri is one of:
//   0 = Level 0 (lowest priority)
//   1 = Level 1
//   2 = Level 2
//   3 = Level 3 (highest priority)
     
// Where group is one of
#define RFERR_RF_DMA    0x01 // Group IP0
#define ADC_T1_P2INT    0x02 // Group IP1
#define URX0_T2_UTX0    0x04 // Group IP2
#define URX1_T3_UTX1    0x08 // Group IP3
#define ENCT_T4_P1IN    0x10 // Group IP4
#define ST_P0INT_WDT    0x20 // Group IP5

/***********************************************************************************
***************        clock macros/functions        ********************************
************************************************************************************/

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL (void *)0
#endif

#ifndef SUCCESS
#define SUCCESS 0
#endif

#ifndef FAILED
#define FAILED  1
#endif


/***********************************************************************************
* MACROS
*/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BM
#define BM(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif


/* uint32 processing */
#define BREAK_UINT32( var, ByteNum ) \
    (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
    ((uint32)((uint32)((Byte0) & 0x00FF) \
        + ((uint32)((Byte1) & 0x00FF) << 8) \
            + ((uint32)((Byte2) & 0x00FF) << 16) \
                + ((uint32)((Byte3) & 0x00FF) << 24)))

#define HI_UINT32(a) ((uint16) (((uint32)(a)) >> 16))
#define LO_UINT32(a) ((uint16) ((uint32)(a)))


/* uint16 processing */
#define BUILD_UINT16(loByte, hiByte) \
    ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((uint16)(a) >> 8) & 0xFF)
#define LO_UINT16(a) ((uint16)(a) & 0xFF)


/* uint16 processing */
#define BUILD_UINT8(hiByte, loByte) \
    ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((uint8)(a) >> 4) & 0x0F)
#define LO_UINT8(a) ((uint8)(a) & 0x0F)

/***********************************************************************************
 * Host to network byte order macros
 */
#ifdef BIG_ENDIAN
#define UINT16_HTON(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint16)); )
#define UINT16_NTOH(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint16)); )

#define UINT32_HTON(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint32)); )
#define UINT32_NTOH(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint32)); )
#else
#define UINT16_HTON(x)
#define UINT16_NTOH(x)

#define UINT32_HTON(x)
#define UINT32_NTOH(x)
#endif


/*
*  This macro is for use by other macros to form a fully valid C statement.
*  Without this, the if/else conditionals could show unexpected behavior.
*
*  For example, use...
*    #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
*  instead of ...
*    #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
*  or
*    #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
*  The last macro would not behave as expected in the if/else construct.
*  The second to last macro will cause a compiler error in certain uses
*  of if/else construct
*
*  It is not necessary, or recommended, to use this macro where there is
*  already a valid C statement.  For example, the following is redundant...
*    #define CALL_FUNC()   st(  func();  )
*  This should simply be...
*    #define CALL_FUNC()   func()
*
* (The while condition below evaluates false without generating a
*  constant-controlling-loop type of warning on most compilers.)
*/
#define st(x)      do { x } while (__LINE__ == -1)



/***********************************************************************************
    Filename:     hal_mcu.c

    Description:

***********************************************************************************/


/*******************************************************************************
 * CONSTANTS
 */

/* SEE DATA SHEET FOR DETAILS ABOUT THE FOLLOWING BIT MASKS */

// Parameters for cc2530ClockSetMainSrc
#define CLOCK_SRC_XOSC      0  // High speed Crystal Oscillator Control
#define CLOCK_SRC_HFRC      1  // Low power RC Oscillator

// Parameters for cc2530ClockSelect32k
#define CLOCK_32K_XTAL      0  // 32.768 Hz crystal oscillator
#define CLOCK_32K_RCOSC     1  // 32 kHz RC oscillator

// Bit masks to check CLKCON register
#define CLKCON_OSC32K_BM    0x80  // bit mask, for the slow 32k clock oscillator
#define CLKCON_OSC_BM       0x40  // bit mask, for the system clock oscillator
#define CLKCON_TICKSPD_BM   0x38  // bit mask, for timer ticks output setting
#define CLKCON_CLKSPD_BM    0x01  // bit maks, for the clock speed

#define TICKSPD_DIV_1       (0x00 << 3)
#define TICKSPD_DIV_2       (0x01 << 3)
#define TICKSPD_DIV_4       (0x02 << 3)
#define TICKSPD_DIV_8       (0x03 << 3)
#define TICKSPD_DIV_16      (0x04 << 3)
#define TICKSPD_DIV_32      (0x05 << 3)
#define TICKSPD_DIV_64      (0x06 << 3)
#define TICKSPD_DIV_128     (0x07 << 3)

// Bit masks to check SLEEPSTA register
#define SLEEP_XOSC_STB_BM   0x40  // bit mask, check the stability of XOSC
#define SLEEP_HFRC_STB_BM   0x20  // bit maks, check the stability of the High-frequency RC oscillator
#define SLEEP_OSC_PD_BM     0x04  // bit mask, power down system clock oscillator(s)

#define NOP()  asm("NOP")

/*******************************************************************************
 * MACROS
 */

// Macro for checking status of the high frequency RC oscillator.
#define CC2530_IS_HFRC_STABLE(x)    (SLEEPSTA & SLEEP_HFRC_STB_BM)

// Macro for checking status of the crystal oscillator
#define CC2530_IS_XOSC_STABLE(x)    (SLEEPSTA & SLEEP_XOSC_STB_BM)

// Macro for getting the clock division factor
#define CC2530_GET_CLKSPD(x)        (CLKCONSTA & CLKCON_CLKSPD_BM)


// Macro for getting the timer tick division factor.
#define CC2530_GET_TICKSPD(x)       ((CLKCONSTA & CLKCON_TICKSPD_BM) >> 3)

// Macro for setting the clock division factor, x value from 0b000 to 0b111
#define CC2530_SET_TICKSPD(x)        st( CLKCONCMD = ((((x) << 3) & 0x38)  \
                                                    | (CLKCONCMD & 0xC7)); \
		)

// Macro for setting the timer tick division factor, x value from 0b000 to 0b111
#define CC2530_SET_CLKSPD(x)         st( CLKCONCMD = (((x) & 0x07)         \
                                                    | (CLKCONCMD & 0xF8)); \
									 )

// Macro for waiting for clock updates
#define CC2530_WAIT_CLK_UPDATE()    st( uint8 ____clkcon; \
                                        uint8 ____clkconsta; \
                                        ____clkcon = CLKCONCMD; \
                                        do { \
                                          ____clkconsta = CLKCONSTA; \
                                        } while (____clkconsta != ____clkcon); )


/******************************************************************************
* @fn  clockSetMainSrc
*
* @brief  Function for setting the main system clock source.
*         The function turns off the clock source that is not being used.
*         TICKSPD is set to the same frequency as the source.
*
* @param  uint8 source (one of CLOCK_SRC_HFRC or CLOCK_SRC_XOSC)
*
* @return void
*
******************************************************************************/
void clockSetMainSrc(uint8 source)
{
    register uint8 osc32k_bm = CLKCONCMD & CLKCON_OSC32K_BM;

    // Source can have the following values:
    // CLOCK_SRC_XOSC   0x00  High speed Crystal Oscillator (XOSC)
    // CLOCK_SRC_HFRC   0x01  Low power RC Oscillator (HFRC)
    
    SLEEPCMD &= ~SLEEP_OSC_PD_BM;       // power up both oscillators
    while (!CC2530_IS_HFRC_STABLE() || ((SLEEPSTA & SLEEP_OSC_PD_BM)!=0));// wait until the oscillator is stable
    NOP();

    if (source == CLOCK_SRC_HFRC){
        CLKCONCMD = (osc32k_bm | CLKCON_OSC_BM | TICKSPD_DIV_2 | CLKCON_CLKSPD_BM);
    }
    else if (source == CLOCK_SRC_XOSC){
        CLKCONCMD = (osc32k_bm | TICKSPD_DIV_1);
    }
    CC2530_WAIT_CLK_UPDATE();
    SLEEPCMD |= SLEEP_OSC_PD_BM;        // power down the unused oscillator
}

/******************************************************************************
* @fn  clockSelect32k
*
* @brief  Function for selecting source for the 32kHz oscillator
*
* @param  uint8 source (one of CLOCK_32K_XTAL or CLOCK_32K_RCOSC)
*
* @return uint8 - SUCCESS or FAILED
*
******************************************************************************/
uint8 clockSelect32k(uint8 source)
{
    // System clock source must be high frequency RC oscillator before
    // changing 32K source. 
    if( !(CLKCONSTA & CLKCON_OSC_BM) )
      return FAILED;
    
    if (source == CLOCK_32K_XTAL){
        CLKCONCMD &= ~CLKCON_OSC32K_BM;
    }
    else if (source == CLOCK_32K_RCOSC){
        CLKCONCMD |= CLKCON_OSC32K_BM;
    }
    CC2530_WAIT_CLK_UPDATE();
    
    return SUCCESS;
}

/***********************************************************************************
* @fn          halMcuInit
*
* @brief       Set Main Clock source to XOSC
*
* @param       none
*
* @return      none
*/
void halMcuInit(void)
{
  // if 32k clock change fails, set system clock to HF RC and try again
  if(clockSelect32k(CLOCK_32K_XTAL) != SUCCESS) {
    clockSetMainSrc(CLOCK_SRC_HFRC);
    if(clockSelect32k(CLOCK_32K_XTAL) != SUCCESS) {
      for(;;);
    }
  }
  clockSetMainSrc(CLOCK_SRC_XOSC);
}

/***********************************************************************************
* @fn          halMcuWaitUs
*
* @brief       Busy wait function. Waits the specified number of microseconds. Use
*              assumptions about number of clock cycles needed for the various
*              instructions. This function assumes a 32 MHz clock.
*
*              NB! This function is highly dependent on architecture and compiler!
*
* @param       uint16 usec - number of microseconds delays
*
* @return      none
*/

#pragma optimize=none
void halMcuWaitUs(uint16 usec)
{
    usec>>= 1;
    while(usec--)
    {
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
    }
}
/***********************************************************************************
* @fn          halMcuWaitMs
*
* @brief       Busy wait function. Waits the specified number of milliseconds. Use
*              assumptions about number of clock cycles needed for the various
*              instructions.
*
*              NB! This function is highly dependent on architecture and compiler!
*
* @param       uint16 millisec - number of milliseconds delay
*
* @return      none
*/
#pragma optimize=none
void halMcuWaitMs(uint16 msec)
{
    while(msec--)
        halMcuWaitUs(1000);
}
/******************************************************************************
* @fn  halMcuReset
*
* @brief
* Resets the MCU. This utilize the watchdog timer as there is no other way
* for a software reset. The reset will not occur until ~2 ms.
* NB: The function will not return! (hangs until reset)
*
* Parameters:
*
* @param  void
*
* @return void
*
******************************************************************************/
void halMcuReset(void)
{
    const uint8 WDT_INTERVAL_MSEC_2=                    0x03;   // after ~2 ms

    WDCTL = ((WDCTL & 0xFC) | (WDT_INTERVAL_MSEC_2 & 0x03));
    // Start watchdog
    WDCTL &= ~0x04;     // Select watchdog mode
    WDCTL |= 0x08;      // Enable timer
    while(1);                                   // Halt here until reset
}






/******************************************************************************
*******************           Timer macros/functions        *******************
*******************************************************************************
General:
The timers/counters can be configured in a number of ways. The following
functions allow basic configuration of the timers as interrupt timers,
pulse width modulators (PWM) and capture timers. Other uses require manual
configuration of the timers/counters.

Generally 3 steps are nescessary to start a timer:

   TIMERx_INIT();
   BOOL halSetTimerxPeriod(period);
   TIMERx_RUN(TRUE);

where x is the timer number. Please see the function / macro in question for
details.

All timers can generate interrupts. The configuration of interrupts is not
included in the HAL.

******************************************************************************/

#define CLR_TIMER34_IF( bitMask )\
  TIMIF = ( TIMIF & 0x40 ) | ( 0x3F & (~bitMask) )

#define CLR_TIMER1_IF( bitMask )\
  T1CTL = ( T1CTL & 0x0F ) | ( 0xF0 & (~bitMask) )


/******************************************************************************
* @fn  halSetTimer1Period
*
* @brief
*      This function sets up timer 1 to run with a given period. If _period_ is
*      set to 0, maximum period length will be used. The first time the timer
*      is used the macro TIMER1_INIT() should be run to clear all settings. The
*      timer is started and stopped with the macro TIMER1_RUN(TRUE / FALSE).
*
* Parameters:
*
* @param  DWORD	 period
*         The desired timer period in u-seconds.
*
* @return WORD
*         The timer value written to the register if the configuration was
*         successful and 0 if the period could not be achieved. This return
*         value can be used for determining pulse widths when the timer is
*         used in PWM mode.
*
******************************************************************************/


// Macro for initialising timer 1. Resets all involved registers and disables
// all interrupt masks.
#define TIMER1_INIT()   \
   do {                 \
      T1CTL  = 0x00;    \
      T1CCTL0 = 0x00;   \
      T1CCTL1 = 0x00;   \
      T1CCTL2 = 0x00;   \
      TIMIF &= ~0x40;   \
   } while (0)

// Macro for configuring a channel of timer 1 for PWM. Channel may be
// either 1 or 2.
#define TIMER1_PWM_CONFIG(channel)               \
   do {                                          \
      T1CCTL##channel## = 0x24;                  \
      if(PERCFG&0x40) {                          \
         if(channel == 0x01){                    \
            IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH);\
         }                                       \
         else {                                  \
            IO_FUNC_PORT_PIN(1,0,IO_FUNC_PERIPH);\
         }                                       \
      }                                          \
      else {                                     \
         if(channel == 0x01){                    \
            IO_FUNC_PORT_PIN(0,3,IO_FUNC_PERIPH);\
         }                                       \
         else {                                  \
            IO_FUNC_PORT_PIN(0,4,IO_FUNC_PERIPH);\
         }                                       \
      }                                          \
   } while(0)

// Macro for changing the pulse length of a timer in PWM mode. The value is
// not scaled and the user must verify that it is correct. _channel_ is the
// channel (1 or 2) configured for PWM operation, whereas _value_ is the
// 16 bit word giving the pulse length. This argument should be shorter than
// or equal to the value returned from the function halSetTimer1Period(...).
#define TIMER1_SET_PWM_PULSE_LENGTH(channel, value) \
   do {                                             \
      T1CC##channel##L = (BYTE)value;               \
      T1CC##channel##H = (BYTE)(value >> 8);        \
   } while(0)


// Macro for configuring a channel of timer 1 for capture.
#define TIMER1_CAPTURE_CHANNEL(channel, edge)      \
   do {                                            \
      T1CCTL ##channel = edge;                     \
      if(PERCFG&0x40) {                            \
         if(channel == 0x01){                      \
            IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH);  \
         }                                         \
         else {                                    \
            IO_FUNC_PORT_PIN(1,0,IO_FUNC_PERIPH);  \
         }                                         \
      }                                            \
      else {                                       \
         if(channel == 0x01){                      \
            IO_FUNC_PORT_PIN(0,3,IO_FUNC_PERIPH);  \
         }                                         \
         else {                                    \
            IO_FUNC_PORT_PIN(0,4,IO_FUNC_PERIPH);  \
         }                                         \
      }                                            \
   } while(0)

// Where _edge_ is either
#define POS_EDGE 0x01  // Capture when a positive edge on the channel input is detected
#define NEG_EDGE 0x02  // Capture when a negative edge on the channel input is detected
#define ANY_EDGE 0x03  // Capture when either a positive or a negative edge on the
                       // channel input is detected.

// Macro for enabling or disabling overflow interrupts of timer 1.
#define TIMER1_ENABLE_OVERFLOW_INT(val) \
   (TIMIF =  (val) ? TIMIF | 0x40 : TIMIF & ~0x40)


/******************************************************************************
* @fn  halSetTimer2Period
*
* @brief
*      This function sets the period and overflow counter value of the MAC timer
*      (timer 2). The timer can be set up with 320 u-second periods according to
*      IEEE 802.15.4 or as a normal counter with 1 m-second period by using the
*      option TIMER2_MAC_TIMER or TIMER2_NORMAL_TIMER respectively. The value of
*      _period_ gives the number of periods (320 u-seconds or 1 m-seconds) to
*      generate a compare event. The timer is set up to compensate for any clock
*      division. The timer is also set up to be synchronised with the 32.768 KHz
*      clock when entering or leaving power mode 0. When starting synchronously
*      from power mode 1 or 2, the timer value is updated by adding the time
*      passed since PM 0 was left. This time is kept by the 32.768 KHz clock.
*      This way the time is kept as if the chip had been in power mode 0 the
*      whole time. The timer must be started with the macro
*      TIMER2_RUN(TRUE) or MAC_TIMER_RUN(TRUE). The macro TIMER2_INIT() should be
*      run in advance to reset all register values.
*
* Parameters:
*
* @param  BYTE	 mode
*         Determines which time period Timer 2 is to use. The period of Timer 2
*         is either 320 u-seconds (TIMER2_MAC_TIMER) or 1 m-second
*         (TIMER2_NORMAL_TIMER).
* @param  DWORD	 period
*         This value indicates how many periods (320 u-second or 1 m-second) to
*         pass before an overflow compare event is generated.
*
* @return BOOL
          Returns 0 if period is too large, 1 otherwise.
*
******************************************************************************/
BOOL halSetTimer2Period(BYTE mode, DWORD period);

// _mode_ may be of the following:
#define TIMER2_MAC_TIMER    0x01  // Counts 320 u-second periods
#define TIMER2_NORMAL_TIMER 0x02  // Uses the timer as a normal timer with 1 m-second period.

// Macro for initialising timer 2
#define TIMER2_INIT()  \
   do {                \
      T2THD = 0x00;    \
      T2TLD = 0x00;    \
      T2CMP = 0x00;    \
      T2OF0 = 0x00;    \
      T2OF1 = 0x00;    \
      T2OF2 = 0x00;    \
      T2CAPHPH = 0x00; \
      T2CAPLPL = 0x00; \
      T2PEROF0 = 0x00; \
      T2PEROF1 = 0x00; \
      T2PEROF2 = 0x00; \
      T2CNF = 0x06;    \
   } while (0)

#define TIMER2_ENABLE_OVERFLOW_COMP_INT(val) (T2PEROF2 =  (val) ? T2PEROF2 | 0x20 : T2PEROF2 & ~0x20)


/******************************************************************************
* @fn  halSetTimer34Period
*
* @brief
*      This function sets the period of timer 3 or 4 according to the value of
*      _timer_. The two timers are identical. Clock division is used to fit the
*      desired period within the timer range. If the period is too short or too
*      long the function returns 0. If the period is successfully set, the
*      function returns the BYTE value written to the timer register. This
*      value can be used to set the pulse length if the timer is used for PWM.
*      If _period_ is set to 0, maximum timeout value will be used.
*
* Parameters:
*
* @param  BYTE	 timer
*         Indicates which timer to configure. Must be either 3 or 4
*         (0x03 or 0x04).
* @param  DWORD	 period - Describe value.
*         The desired period in microseconds.
*
* @return BYTE
*         The value written to the TxCC0 register. The timer is incremented up
*         to this value before the timer is reset. This value may be used to
*         set the pulse length in PWM mode.
*
******************************************************************************/
BYTE halSetTimer34Period(BYTE timer, DWORD period){
  BYTE div = 0;

  if(TICKSPD > 5) { // Checking that the period is not too short.
    if( (period < 2*(TICKSPD-5)) && (period != 0) ){
      return 0;
    }
  }

  if(period == 0){  // If period is 0, max period length and max prescaler
    div = 7;  // division is used.
    period = 255;
  } else {
    period = ((period*32) >> TICKSPD);// Determining how many timer ticks the period consist of
    while(period > 255){              // If the period is too long, the prescaler division is
      period = (period >> 1);         // increased.
      div++;
      if(div > 7){                    // If the period is too long when using max prescaler division,
        return 0;                     // 0 is returned.
      }
    }
  }

  if(timer == 4){
    // Timer 4 selected
    T4CTL |= (div << 5);              // Setting prescaler value
    T4CC0 = (BYTE) period;            // Setting timer value.
  } else if(timer == 3){
    // Timer 3 selected
    T3CTL |= (div << 5);              // Setting prescaler value
    T3CC0 = (BYTE) period;            // Setting timer value.
  } else {
    return 0;
  }

  return period;
}

// Macro for initialising timer 3 or 4
#define TIMER34_INIT(timer)   \
   do {                       \
      T##timer##CTL   = 0x06; \
      T##timer##CCTL0 = 0x00; \
      T##timer##CC0   = 0x00; \
      T##timer##CCTL1 = 0x00; \
      T##timer##CC1   = 0x00; \
   } while (0)

// Macro for enabling overflow interrupt
#define TIMER34_ENABLE_OVERFLOW_INT(timer, val)   \
   (T##timer##CTL =  (val) ? T##timer##CTL | 0x08 : T##timer##CTL & ~0x08)


// Macro for configuring channel 1 of timer 3 or 4 for PWM mode.
#define TIMER34_PWM_CONFIG(timer)                 \
   do{                                            \
      T##timer##CCTL1 = 0x24;                     \
      if(timer == 3){                             \
         if(PERCFG & 0x20) {                      \
            IO_FUNC_PORT_PIN(1,7,IO_FUNC_PERIPH); \
         }                                        \
         else {                                   \
            IO_FUNC_PORT_PIN(1,4,IO_FUNC_PERIPH); \
         }                                        \
      }                                           \
      else {                                      \
         if(PERCFG & 0x10) {                      \
             IO_FUNC_PORT_PIN(2,3,IO_FUNC_PERIPH);\
         }                                        \
         else {                                   \
            IO_FUNC_PORT_PIN(1,1,IO_FUNC_PERIPH); \
         }                                        \
      }                                           \
   } while(0)

// Macro for setting pulse length of the timer in PWM mode
#define TIMER34_SET_PWM_PULSE_LENGTH(timer, value) \
   do {                                            \
      T##timer##CC1 = (BYTE)value;                 \
   } while (0)


// Macros for turning timers on or off
#define TIMER1_RUN(value)      (T1CTL = (value) ? T1CTL | 0x02 : T1CTL & ~0x03)
#define TIMER2_RUN(value)      (T2CNF = (value) ? T2CNF | 0x01  : T2CNF & ~0x01)
// MAC-timer == timer 2
#define MAC_TIMER_RUN(value)   do{ TIMER2_RUN(value); }while(0)
#define TIMER3_RUN(value)      (T3CTL = (value) ? T3CTL | 0x10 : T3CTL & ~0x10)
#define TIMER4_RUN(value)      (T4CTL = (value) ? T4CTL | 0x10 : T4CTL & ~0x10)

// Macro for enabling/ disabling interrupts from the channels of timer 1, 3 or 4.
#define TIMER_CHANNEL_INTERRUPT_ENABLE(timer, channel, value) \
   do{                                                        \
      if(value){                                              \
         T##timer##CCTL##channel## |= 0x40;                   \
      } else {                                                \
         T##timer##CCTL##channel## &= ~0x40;                  \
      }                                                       \
   } while(0)



/******************************************************************************
*******************             ADC macros/functions        *******************
*******************************************************************************

These functions/macros simplifies usage of the ADC.

******************************************************************************/

// Macro for setting up a single conversion. If ADCCON1.STSEL = 11, using this
// macro will also start the conversion.
#define ADC_SINGLE_CONVERSION(settings) \
  do{                                   \
    ADCCON3 = (settings);               \
  }while(0)

// Macro for setting up a single conversion
#define ADC_SEQUENCE_SETUP(settings)    \
  do{                                   \
    ADCCON2 = (settings);               \
  }while(0)

// Where _settings_ are the following:
// Reference voltage:
#define ADC_REF_1_25_V      0x00     // Internal 1.25V reference
#define ADC_REF_P0_7        0x40     // External reference on AIN7 pin
#define ADC_REF_AVDD        0x80     // AVDD_SOC pin
#define ADC_REF_P0_6_P0_7   0xC0     // External reference on AIN6-AIN7 differential input

// Resolution (decimation rate):
#define ADC_8_BIT           0x00     //  64 decimation rate
#define ADC_10_BIT          0x10     // 128 decimation rate
#define ADC_12_BIT          0x20     // 256 decimation rate
#define ADC_14_BIT          0x30     // 512 decimation rate

// Input channel:
#define ADC_AIN0            0x00     // single ended P0_0
#define ADC_AIN1            0x01     // single ended P0_1
#define ADC_AIN2            0x02     // single ended P0_2
#define ADC_AIN3            0x03     // single ended P0_3
#define ADC_AIN4            0x04     // single ended P0_4
#define ADC_AIN5            0x05     // single ended P0_5
#define ADC_AIN6            0x06     // single ended P0_6
#define ADC_AIN7            0x07     // single ended P0_7
#define ADC_GND             0x0C     // Ground
#define ADC_TEMP_SENS       0x0E     // on-chip temperature sensor
#define ADC_VDD_3           0x0F     // (vdd/3)

// Macro for starting the ADC in continuous conversion mode
#define ADC_SAMPLE_CONTINUOUS()   \
  do {                            \
    ADCCON1 &= ~0x30;             \
    ADCCON1 |= 0x10;              \
  } while (0)

// Macro for stopping the ADC in continuous mode
#define ADC_STOP()                \
  do {                            \
    ADCCON1 |= 0x30;              \
  } while (0)

// Macro for initiating a single sample in single-conversion mode (ADCCON1.STSEL = 11).
#define ADC_SAMPLE_SINGLE()       \
  do{                             \
    ADC_STOP();                   \
    ADCCON1 |= 0x40;              \
} while (0)

// Macro for configuring the ADC to be started from T1 channel 0. (T1 ch 0 must be in compare mode!!)
#define ADC_TRIGGER_FROM_TIMER1() \
  do {                            \
    ADC_STOP();                   \
    ADCCON1 &= ~0x10;             \
  } while (0)

// Expression indicating whether a conversion is finished or not.
#define ADC_SAMPLE_READY()      (ADCCON1 & 0x80)

// Macro for setting/clearing a channel as input of the ADC
#define ADC_ENABLE_CHANNEL(ch)   APCFG |=  (0x01 << ch)
#define ADC_DISABLE_CHANNEL(ch)  APCFG &= ~(0x01 << ch)

#define ADC_GET_VALUE( v )       GET_WORD( ADCH, ADCL, v )


/******************************************************************************
* @fn  halAdcSampleSingle
*
* @brief
*      This function makes the adc sample the given channel at the given
*      resolution with the given reference.
*
* Parameters:
*
* @param BYTE reference
*          The reference to compare the channel to be sampled.
*        BYTE resolution
*          The resolution to use during the sample (8, 10, 12 or 14 bit)
*        BYTE input
*          The channel to be sampled.
*
* @return INT16
*          The conversion result
*
******************************************************************************/
INT16 halAdcSampleSingle(BYTE reference, BYTE resolution, UINT8 input);


/******************************************************************************
*******************         Common USART functions/macros   *******************
*******************************************************************************

The macros in this section are available for both SPI and UART operation.

******************************************************************************/

// Example usage:
//   USART0_FLUSH();
#define USART_FLUSH(num)              (U##num##UCR |= 0x80)
#define USART0_FLUSH()                USART_FLUSH(0)
#define USART1_FLUSH()                USART_FLUSH(1)

// Example usage:
//   if (USART0_BUSY())
//     ...
#define USART_BUSY(num)               (U##num##CSR & 0x01)
#define USART0_BUSY()                 USART_BUSY(0)
#define USART1_BUSY()                 USART_BUSY(1)

// Example usage:
//   while(!USART1_BYTE_RECEIVED())
//     ...
#define USART_BYTE_RECEIVED(num)      (U##num##CSR & 0x04)
#define USART0_BYTE_RECEIVED()        USART_BYTE_RECEIVED(0)
#define USART1_BYTE_RECEIVED()        USART_BYTE_RECEIVED(1)

// Example usage:
//   if(USART1_BYTE_TRANSMITTED())
//     ...
#define USART_BYTE_TRANSMITTED(num)   (U##num##CSR & 0x02)
#define USART0_BYTE_TRANSMITTED()     USART_BYTE_TRANSMITTED(0)
#define USART1_BYTE_TRANSMITTED()     USART_BYTE_TRANSMITTED(1)


/******************************************************************************
*******************  USART-UART specific functions/macros   *******************
******************************************************************************/
// The macros in this section simplify UART operation.
#define BAUD_E(baud, clkDivPow) (     \
    (baud==2400)   ?  6  +clkDivPow : \
    (baud==4800)   ?  7  +clkDivPow : \
    (baud==9600)   ?  8  +clkDivPow : \
    (baud==14400)  ?  8  +clkDivPow : \
    (baud==19200)  ?  9  +clkDivPow : \
    (baud==28800)  ?  9  +clkDivPow : \
    (baud==38400)  ?  10 +clkDivPow : \
    (baud==57600)  ?  10 +clkDivPow : \
    (baud==76800)  ?  11 +clkDivPow : \
    (baud==115200) ?  11 +clkDivPow : \
    (baud==153600) ?  12 +clkDivPow : \
    (baud==230400) ?  12 +clkDivPow : \
    0  )


#define BAUD_M(baud) (      \
    (baud==2400)   ?  59  : \
    (baud==4800)   ?  59  : \
    (baud==9600)   ?  59  : \
    (baud==14400)  ?  216 : \
    (baud==19200)  ?  59  : \
    (baud==28800)  ?  216 : \
    (baud==38400)  ?  59  : \
    (baud==57600)  ?  216 : \
    (baud==76800)  ?  59  : \
    (baud==115200) ?  216 : \
    (baud==153600) ?  59  : \
    (baud==230400) ?  216 : \
  0)




// Macro for setting up a UART transfer channel. The macro sets the appropriate
// pins for peripheral operation, sets the baudrate, and the desired options of
// the selected uart. _uart_ indicates which uart to configure and must be
// either 0 or 1. _baudRate_ must be one of 2400, 4800, 9600, 14400, 19200,
// 28800, 38400, 57600, 76800, 115200, 153600, 230400 or 307200. Possible
// options are defined below.
//
// Example usage:
//
//      UART_SETUP(0,115200,HIGH_STOP);
//
// This configures uart 0 for contact with "hyperTerminal", setting:
//      Baudrate:           115200
//      Data bits:          8
//      Parity:             None
//      Stop bits:          1
//      Flow control:       None
//

#define UART_SETUP(uart, baudRate, options)      \
   do {                                          \
      if ((options) & FLOW_CONTROL_ENABLE){      \
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x3C;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xF0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x3C;                    \
            }                                    \
         }                                       \
      }                                          \
      else{                    /* Flow Ctrl Dis*/\
         if((uart) == 0){      /* USART0       */\
            if(PERCFG & 0x01){ /* Alt 2        */\
               P1SEL |= 0x30;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x0C;                    \
            }                                    \
         }                                       \
         else {                /* USART1       */\
            if(PERCFG & 0x02){ /* Alt 2        */\
               P1SEL |= 0xC0;                    \
            } else {           /* Alt 1        */\
               P0SEL |= 0x30;                    \
            }                                    \
         }                                       \
      }                                          \
                                                 \
      U##uart##GCR = BAUD_E((baudRate), CLKSPD); \
      U##uart##BAUD = BAUD_M(baudRate);          \
                                                 \
      U##uart##CSR |= 0x80;                      \
                                                 \
      U##uart##UCR |= ((options) | 0x80);        \
                                                 \
      if((options) & TRANSFER_MSB_FIRST){        \
         U##uart##GCR |= 0x20;                   \
      }                                          \
   } while(0)


// Options for UART_SETUP macro
//以下是上面宏定义options的取值
#define FLOW_CONTROL_ENABLE         0x40  //流控使能
#define FLOW_CONTROL_DISABLE        0x00  //流控不是能
#define EVEN_PARITY                 0x20  //偶校验
#define ODD_PARITY                  0x00  //奇校验
#define NINE_BIT_TRANSFER           0x10  //九位传送
#define EIGHT_BIT_TRANSFER          0x00  //八位传送
#define PARITY_ENABLE               0x08  //校验使能
#define PARITY_DISABLE              0x00  //校验不使能
#define TWO_STOP_BITS               0x04  //两个停止位
#define ONE_STOP_BITS               0x00  //一个停止位
#define HIGH_STOP                   0x02  //停止位是高电平
#define LOW_STOP                    0x00  //停止位是低电平
#define HIGH_START                  0x01  //起始位是高电平
#define TRANSFER_MSB_FIRST          0x80  //首先传送高位
#define TRANSFER_MSB_LAST           0x00  //首先传送低位


// Example usage:
//   if(UART0_PARERR())
//     ...
//奇偶校验错误
#define UART_PARERR(num)      (U##num##CSR & 0x08)
#define UART0_PARERR()        UART_PARERR(0)
#define UART1_PARERR()        UART_PARERR(1)

// Example usage:
//   if(UART1_FRAMEERR())
//     ...
//接收数据帧错误
#define UART_FRAMEERR(num)    (U ##num## CSR & 0x10)
#define UART0_FRAMEERR()      UART_FRAMEERR(0)
#define UART1_FRAMEERR()      UART_FRAMEERR(1)


// Example usage:
//   char ch = 'A';
//   UART1_SEND(ch);发送字符
//   ...
//   UART1_RECEIVE(ch);接收字符
#define UART_SEND(num, x)     U##num##DBUF = x
#define UART0_SEND(x)         UART_SEND(0, x)
#define UART1_SEND(x)         UART_SEND(1, x)

#define UART_RECEIVE(num, x)  x = U##num##DBUF
#define UART0_RECEIVE(x)      UART_RECEIVE(0, x)
#define UART1_RECEIVE(x)      UART_RECEIVE(1, x)



/******************************************************************************
*******************    USART-SPI specific functions/macros  *******************
******************************************************************************/
// The macros in this section simplify SPI operation.

//*****************************************************************************
// Macro for setting up an SPI connection. The macro configures the appropriate
// pins for peripheral operation, sets the baudrate if the chip is configured
// to be SPI master, and sets the desired clock polarity and phase. Whether to
// transfer MSB or LSB first is also determined. _spi_ indicates whether
// to use spi 0 or 1. _baudRate_ must be one of 2400, 4800, 9600, 14400, 19200,
// 28800, 38400, 57600, 76800, 115200, 153600, 230400 or 307200.
// Possible options are defined below.

#define SPI_SETUP(spi, baudRate, options)           \
   do {                                             \
      U##spi##UCR = 0x80;                           \
      U##spi##CSR = 0x00;                           \
                                                    \
      if(spi == 0){                                 \
         if(PERCFG & 0x01){                         \
            P1SEL |= 0x3C;                          \
         } else {                                   \
            P0SEL |= 0x3C;                          \
         }                                          \
      }                                             \
      else {                                        \
         if(PERCFG & 0x02){                         \
            P1SEL |= 0xF0;                          \
         } else {                                   \
            P0SEL |= 0x3C;                          \
         }                                          \
      }                                             \
                                                    \
      if(options & SPI_SLAVE){                      \
         U##spi##CSR = 0x20;                        \
      }                                             \
      else {                                        \
         U##spi##GCR = BAUD_E(baudRate, CLKSPD);    \
         U##spi##BAUD = BAUD_M(baudRate);           \
      }                                             \
      U##spi##GCR |= (options & 0xE0);              \
   } while(0)


// Options for the SPI_SETUP macro.
#define SPI_SLAVE              0x01
#define SPI_MASTER             0x00
#define SPI_CLOCK_POL_LO       0x00
#define SPI_CLOCK_POL_HI       0x80
#define SPI_CLOCK_PHA_0        0x00
#define SPI_CLOCK_PHA_1        0x40
#define SPI_TRANSFER_MSB_FIRST 0x20
#define SPI_TRANSFER_MSB_LAST  0x00





/******************************************************************************
**************************   DMA structures / macros  *************************
******************************************************************************/

// The macros and structs in this section simplify setup and usage of DMA.

//******************************************************************************

#define DMA_CHANNEL_0  0x01
#define DMA_CHANNEL_1  0x02
#define DMA_CHANNEL_2  0x04
#define DMA_CHANNEL_3  0x08
#define DMA_CHANNEL_4  0x10

#define VLEN_USE_LEN            0x00 // Use LEN for transfer count
#define VLEN_FIXED              0x00 // Use LEN for transfer count
#define VLEN_1_P_VALOFFIRST     0x01 // Transfer the first byte + the number of bytes indicated by the first byte
#define VLEN_VALOFFIRST         0x02 // Transfer the number of bytes indicated by the first byte (starting with the first byte)
#define VLEN_1_P_VALOFFIRST_P_1 0x03 // Transfer the first byte + the number of bytes indicated by the first byte + 1 more byte
#define VLEN_1_P_VALOFFIRST_P_2 0x04 // Transfer the first byte + the number of bytes indicated by the first byte + 2 more bytes

#define WORDSIZE_BYTE           0x00 // Transfer a byte at a time
#define WORDSIZE_WORD           0x01 // Transfer a 16-bit word at a time

#define TMODE_SINGLE            0x00 // Transfer a single byte/word after each DMA trigger
#define TMODE_BLOCK             0x01 // Transfer block of data (length len) after each DMA trigger
#define TMODE_SINGLE_REPEATED   0x02 // Transfer single byte/word (after len transfers, rearm DMA)
#define TMODE_BLOCK_REPEATED    0x03 // Transfer block of data (after len transfers, rearm DMA)

#define DMATRIG_NONE           0   // No trigger, setting DMAREQ.DMAREQx bit starts transfer
#define DMATRIG_PREV           1   // DMA channel is triggered by completion of previous channel
#define DMATRIG_T1_CH0         2   // Timer 1, compare, channel 0
#define DMATRIG_T1_CH1         3   // Timer 1, compare, channel 1
#define DMATRIG_T1_CH2         4   // Timer 1, compare, channel 2
#define DMATRIG_T2_EVENT1      5   // Timer 2, compare
#define DMATRIG_T2_EVENT2      6   // Timer 2, overflow
#define DMATRIG_T3_CH0         7   // Timer 3, compare, channel 0
#define DMATRIG_T3_CH1         8   // Timer 3, compare, channel 1
#define DMATRIG_T4_CH0         9   // Timer 4, compare, channel 0
#define DMATRIG_T4_CH1         10   // Timer 4, compare, channel 1
#define DMATRIG_ST             11   // Sleep Timer compare
#define DMATRIG_IOC_0          12   // Port 0 I/O pin input transition
#define DMATRIG_IOC_1          13   // Port 1 I/O pin input transition
#define DMATRIG_URX0           14   // USART0 RX complete
#define DMATRIG_UTX0           15   // USART0 TX complete
#define DMATRIG_URX1           16   // USART1 RX complete
#define DMATRIG_UTX1           17   // USART1 TX complete
#define DMATRIG_FLASH          18   // Flash data write complete
#define DMATRIG_RADIO          19   // RF packet byte received/transmit
#define DMATRIG_ADC_CHALL      20   // ADC end of a conversion in a sequence, sample ready
#define DMATRIG_ADC_CH0        21   // ADC end of conversion channel 0 in sequence, sample ready
#define DMATRIG_ADC_CH1        22   // ADC end of conversion channel 1 in sequence, sample ready
#define DMATRIG_ADC_CH2        23   // ADC end of conversion channel 2 in sequence, sample ready
#define DMATRIG_ADC_CH3        24   // ADC end of conversion channel 3 in sequence, sample ready
#define DMATRIG_ADC_CH4        25   // ADC end of conversion channel 4 in sequence, sample ready
#define DMATRIG_ADC_CH5        26   // ADC end of conversion channel 5 in sequence, sample ready
#define DMATRIG_ADC_CH6        27   // ADC end of conversion channel 6 in sequence, sample ready
#define DMATRIG_ADC_CH7        28   // ADC end of conversion channel 7 in sequence, sample ready
#define DMATRIG_ENC_DW         29   // AES encryption processor requests download input data
#define DMATRIG_ENC_UP         30   // AES encryption processor requests upload output data
#define DMATRIG_DBG_BW         31   // Debug interface burst write

#define SRCINC_0         0x00 // Increment source pointer by 0 bytes/words after each transfer
#define SRCINC_1         0x01 // Increment source pointer by 1 bytes/words after each transfer
#define SRCINC_2         0x02 // Increment source pointer by 2 bytes/words after each transfer
#define SRCINC_M1        0x03 // Decrement source pointer by 1 bytes/words after each transfer

#define DESTINC_0        0x00 // Increment destination pointer by 0 bytes/words after each transfer
#define DESTINC_1        0x01 // Increment destination pointer by 1 bytes/words after each transfer
#define DESTINC_2        0x02 // Increment destination pointer by 2 bytes/words after each transfer
#define DESTINC_M1       0x03 // Decrement destination pointer by 1 bytes/words after each transfer

#define IRQMASK_DISABLE  0x00 // Disable interrupt generation
#define IRQMASK_ENABLE   0x01 // Enable interrupt generation upon DMA channel done

#define M8_USE_8_BITS    0x00 // Use all 8 bits for transfer count
#define M8_USE_7_BITS    0x01 // Use 7 LSB for transfer count

#define PRI_LOW          0x00 // Low, CPU has priority
#define PRI_GUARANTEED   0x01 // Assured, DMA at least every second try
#define PRI_HIGH         0x02 // High, DMA has priority
#define PRI_ABSOLUTE     0x03 // Highest, DMA has priority. Reserved for DMA port access.




#pragma bitfields=reversed    //为了节省空间使用的
typedef struct {
   BYTE SRCADDRH;
   BYTE SRCADDRL;
   BYTE DESTADDRH;
   BYTE DESTADDRL;
   BYTE VLEN      : 3;
   BYTE LENH      : 5;
   BYTE LENL      : 8;
   BYTE WORDSIZE  : 1;
   BYTE TMODE     : 2;
   BYTE TRIG      : 5;
   BYTE SRCINC    : 2;
   BYTE DESTINC   : 2;
   BYTE IRQMASK   : 1;
   BYTE M8        : 1;
   BYTE PRIORITY  : 2;
} DMA_DESC;
#pragma bitfields=default

//通道0配置地址，a是16位的数据

#define DMA_SET_ADDR_DESC0(dmaCH)           \
   do{                                  \
      DMA0CFGH = (unsigned char)(((unsigned short)(dmaCH)) >> 8);\
      DMA0CFGL = (unsigned char)(((unsigned short)(dmaCH)) & 0x00FF);     \
   } while(0)

//通道1-4配置地址，a是16位的数据
#define DMA_SET_ADDR_DESC1234(a)        \
   do{                                  \
      DMA1CFGH = (unsigned char)(((unsigned short)(dmaCH)) >> 8);\
      DMA1CFGL = (unsigned char)(((unsigned short)(dmaCH)) & 0x00FF);     \
   } while(0)

//使能通道，ch可以取0,1,2,3,4
#define DMA_ARM_CHANNEL(ch)           \
   do{                                \
      DMAARM = ((0x01 << ch) & 0x1F); \
   } while(0)

//停止DMA，ch可以取0,1,2,3,4
#define DMA_ABORT_CHANNEL(ch)    DMAARM = (0x80 | ((0x01 << ch) & 0x1F))

//触发通道开始传输，ch可以取0,1,2,3,4
#define DMA_MAN_TRIGGER(ch)      DMAREQ = (0x01 << (ch))

//这句就是在调用上一句宏定义
#define DMA_START_CHANNEL(ch)    DMA_MAN_TRIGGER(ch)

#define DMA_CLEAR_IRQ( ch )        DMAIRQ &= ~( 1 << (ch) )
#define DMA_CHECK_IRQ( ch )       (DMAIRQ & ( 1 << (ch) ))

// Macro for quickly setting the destination address of a DMA structure
//配置目的地址
#define SET_DMA_DEST(pDmaDesc, dest)                 \
   do{                                               \
      pDmaDesc->DESTADDRH = (BYTE) ((WORD)dest >> 8);\
      pDmaDesc->DESTADDRL = (BYTE)  (WORD)dest;      \
   } while (0);

// Macro for quickly setting the source address of a DMA structure
//配置源地址
#define SET_DMA_SOURCE(pDmaDesc, source)              \
   do{                                                \
      pDmaDesc->SRCADDRH = (BYTE) ((WORD)source >> 8);\
      pDmaDesc->SRCADDRL = (BYTE)  (WORD)source;      \
   } while (0)

// Macro for quickly setting the number of bytes to be transferred by the DMA.
// max lenght is 0x1FFF
#define SET_DMA_LENGTH(pDmaDesc, length)          \
   do{                                            \
      pDmaDesc->LENH = (BYTE) ((WORD)length >> 8);\
      pDmaDesc->LENL = (BYTE)  (WORD)length;      \
   } while (0)

// Macro for getting the destination address of a DMA channel
#define GET_DMA_DEST(pDmaDesc)   \
   ( (WORD)pDmaDesc->DESTADDRL | ( (WORD)pDmaDesc->DESTADDRH << 8 ))

// Macro for getting the source address of a DMA channel
#define GET_DMA_SOURCE(pDmaDesc) \
   ( (WORD)pDmaDesc->SRCADDRL  | ( (WORD)pDmaDesc->SRCADDRH << 8 ))



// Macro for quickly setting the source address of a DMA structure.
#define HAL_DMA_SET_SOURCE( pDesc, src ) \
  st( \
    pDesc->srcAddrH = (uint8)((uint16)(src) >> 8); \
    pDesc->srcAddrL = (uint8)(uint16)(src); \
  )

// Macro for quickly setting the destination address of a DMA structure.
#define HAL_DMA_SET_DEST( pDesc, dst ) \
  st( \
    pDesc->dstAddrH = (uint8)((uint16)(dst) >> 8); \
    pDesc->dstAddrL = (uint8)(uint16)(dst); \
  )

// Macro for quickly setting the number of bytes to be transferred by the DMA,
// max length is 0x1FFF.
#define HAL_DMA_SET_LEN( pDesc, len ) \
  st( \
    pDesc->xferLenL = (uint8)(uint16)(len); \
    pDesc->xferLenV &= ~HAL_DMA_LEN_H; \
    pDesc->xferLenV |= (uint8)((uint16)(len) >> 8); \
  )

#define HAL_DMA_GET_LEN( pDesc ) \
  (((uint16)(pDesc->xferLenV & HAL_DMA_LEN_H) << 8) | pDesc->xferLenL)

#define HAL_DMA_SET_VLEN( pDesc, vMode ) \
  st( \
    pDesc->xferLenV &= ~HAL_DMA_LEN_V; \
    pDesc->xferLenV |= (vMode << 5); \
  )

#define HAL_DMA_SET_WORD_SIZE( pDesc, xSz ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_WORD_SIZE; \
    pDesc->ctrlA |= (xSz << 7); \
  )

#define HAL_DMA_SET_TRIG_MODE( pDesc, tMode ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_TRIG_MODE; \
    pDesc->ctrlA |= (tMode << 5); \
  )

#define HAL_DMA_GET_TRIG_MODE( pDesc ) ((pDesc->ctrlA >> 5) & 0x3)

#define HAL_DMA_SET_TRIG_SRC( pDesc, tSrc ) \
  st( \
    pDesc->ctrlA &= ~HAL_DMA_TRIG_SRC; \
    pDesc->ctrlA |= tSrc; \
  )

#define HAL_DMA_SET_SRC_INC( pDesc, srcInc ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_SRC_INC; \
    pDesc->ctrlB |= (srcInc << 6); \
  )

#define HAL_DMA_SET_DST_INC( pDesc, dstInc ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_DST_INC; \
    pDesc->ctrlB |= (dstInc << 4); \
  )

#define HAL_DMA_SET_IRQ( pDesc, enable ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_IRQ_MASK; \
    pDesc->ctrlB |= (enable << 3); \
  )

#define HAL_DMA_SET_M8( pDesc, m8 ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_M8; \
    pDesc->ctrlB |= (m8 << 2); \
  )

#define HAL_DMA_SET_PRIORITY( pDesc, pri ) \
  st( \
    pDesc->ctrlB &= ~HAL_DMA_PRIORITY; \
    pDesc->ctrlB |= pri; \
  )

/*********************************************************************
 * CONSTANTS
 */

// Use LEN for transfer count
#define HAL_DMA_VLEN_USE_LEN            0x00
// Transfer the first byte + the number of bytes indicated by the first byte
#define HAL_DMA_VLEN_1_P_VALOFFIRST     0x01
// Transfer the number of bytes indicated by the first byte (starting with the first byte)
#define HAL_DMA_VLEN_VALOFFIRST         0x02
// Transfer the first byte + the number of bytes indicated by the first byte + 1 more byte
#define HAL_DMA_VLEN_1_P_VALOFFIRST_P_1 0x03
// Transfer the first byte + the number of bytes indicated by the first byte + 2 more bytes
#define HAL_DMA_VLEN_1_P_VALOFFIRST_P_2 0x04

#define HAL_DMA_WORDSIZE_BYTE           0x00 /* Transfer a byte at a time. */
#define HAL_DMA_WORDSIZE_WORD           0x01 /* Transfer a 16-bit word at a time. */

#define HAL_DMA_TMODE_SINGLE            0x00 /* Transfer a single byte/word after each DMA trigger. */
#define HAL_DMA_TMODE_BLOCK             0x01 /* Transfer block of data (length len) after each DMA trigger. */
#define HAL_DMA_TMODE_SINGLE_REPEATED   0x02 /* Transfer single byte/word (after len transfers, rearm DMA). */
#define HAL_DMA_TMODE_BLOCK_REPEATED    0x03 /* Transfer block of data (after len transfers, rearm DMA). */

#define HAL_DMA_TRIG_NONE           0   /* No trigger, setting DMAREQ.DMAREQx bit starts transfer. */
#define HAL_DMA_TRIG_PREV           1   /* DMA channel is triggered by completion of previous channel. */
#define HAL_DMA_TRIG_T1_CH0         2   /* Timer 1, compare, channel 0. */
#define HAL_DMA_TRIG_T1_CH1         3   /* Timer 1, compare, channel 1. */
#define HAL_DMA_TRIG_T1_CH2         4   /* Timer 1, compare, channel 2. */
#define HAL_DMA_TRIG_T2_COMP        5   /* Timer 2, compare. */
#define HAL_DMA_TRIG_T2_OVFL        6   /* Timer 2, overflow. */
#define HAL_DMA_TRIG_T3_CH0         7   /* Timer 3, compare, channel 0. */
#define HAL_DMA_TRIG_T3_CH1         8   /* Timer 3, compare, channel 1. */
#define HAL_DMA_TRIG_T4_CH0         9   /* Timer 4, compare, channel 0. */
#define HAL_DMA_TRIG_T4_CH1        10   /* Timer 4, compare, channel 1. */
#define HAL_DMA_TRIG_ST            11   /* Sleep Timer compare. */
#define HAL_DMA_TRIG_IOC_0         12   /* Port 0 I/O pin input transition. */
#define HAL_DMA_TRIG_IOC_1         13   /* Port 1 I/O pin input transition. */
#define HAL_DMA_TRIG_URX0          14   /* USART0 RX complete. */
#define HAL_DMA_TRIG_UTX0          15   /* USART0 TX complete. */
#define HAL_DMA_TRIG_URX1          16   /* USART1 RX complete. */
#define HAL_DMA_TRIG_UTX1          17   /* USART1 TX complete. */
#define HAL_DMA_TRIG_FLASH         18   /* Flash data write complete. */
#define HAL_DMA_TRIG_RADIO         19   /* RF packet byte received/transmit. */
#define HAL_DMA_TRIG_ADC_CHALL     20   /* ADC end of a conversion in a sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH0       21   /* ADC end of conversion channel 0 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH1       22   /* ADC end of conversion channel 1 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH2       23   /* ADC end of conversion channel 2 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH3       24   /* ADC end of conversion channel 3 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH4       25   /* ADC end of conversion channel 4 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH5       26   /* ADC end of conversion channel 5 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH6       27   /* ADC end of conversion channel 6 in sequence, sample ready. */
#define HAL_DMA_TRIG_ADC_CH7       28   /* ADC end of conversion channel 7 in sequence, sample ready. */
#define HAL_DMA_TRIG_ENC_DW        29   /* AES encryption processor requests download input data. */
#define HAL_DMA_TRIG_ENC_UP        30   /* AES encryption processor requests upload output data. */

#define HAL_DMA_SRCINC_0         0x00 /* Increment source pointer by 0 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_1         0x01 /* Increment source pointer by 1 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_2         0x02 /* Increment source pointer by 2 bytes/words after each transfer. */
#define HAL_DMA_SRCINC_M1        0x03 /* Decrement source pointer by 1 bytes/words after each transfer. */

#define HAL_DMA_DSTINC_0         0x00 /* Increment destination pointer by 0 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_1         0x01 /* Increment destination pointer by 1 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_2         0x02 /* Increment destination pointer by 2 bytes/words after each transfer. */
#define HAL_DMA_DSTINC_M1        0x03 /* Decrement destination pointer by 1 bytes/words after each transfer. */

#define HAL_DMA_IRQMASK_DISABLE  0x00 /* Disable interrupt generation. */
#define HAL_DMA_IRQMASK_ENABLE   0x01 /* Enable interrupt generation upon DMA channel done. */

#define HAL_DMA_M8_USE_8_BITS    0x00 /* Use all 8 bits for transfer count. */
#define HAL_DMA_M8_USE_7_BITS    0x01 /* Use 7 LSB for transfer count. */

#define HAL_DMA_PRI_LOW          0x00 /* Low, CPU has priority. */
#define HAL_DMA_PRI_GUARANTEED   0x01 /* Guaranteed, DMA at least every second try. */
#define HAL_DMA_PRI_HIGH         0x02 /* High, DMA has priority. */
#define HAL_DMA_PRI_ABSOLUTE     0x03 /* Highest, DMA has priority. Reserved for DMA port access.. */

#define HAL_DMA_MAX_ARM_CLOCKS   45   // Maximum number of clocks required if arming all 5 at once.

/*********************************************************************
 * TYPEDEFS
 */

// Bit fields of the 'lenModeH'
#define HAL_DMA_LEN_V     0xE0
#define HAL_DMA_LEN_H     0x1F

// Bit fields of the 'ctrlA'
#define HAL_DMA_WORD_SIZE 0x80
#define HAL_DMA_TRIG_MODE 0x60
#define HAL_DMA_TRIG_SRC  0x1F

// Bit fields of the 'ctrlB'
#define HAL_DMA_SRC_INC   0xC0
#define HAL_DMA_DST_INC   0x30
#define HAL_DMA_IRQ_MASK  0x08
#define HAL_DMA_M8        0x04
#define HAL_DMA_PRIORITY  0x03

typedef struct {
  uint8 srcAddrH;
  uint8 srcAddrL;
  uint8 dstAddrH;
  uint8 dstAddrL;
  uint8 xferLenV;
  uint8 xferLenL;
  uint8 ctrlA;
  uint8 ctrlB;
} halDMADesc_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern halDMADesc_t dmaCh0;
extern halDMADesc_t dmaCh1234[4];

#endif


