/**************************************************************************************************
  Filename:       hal_spi.c
  Revised:        $Date: 2012-10-02 15:13:51 -0700 (Tue, 02 Oct 2012) $
  Revision:       $Revision: 31675 $

  Description: This file contains the interface for the SPI driver.


  Copyright 2006-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board.h"
#if (defined HAL_SPI) && (HAL_SPI == TRUE)
#include "hal_assert.h"
#include "hal_dma.h"
#include "hal_spi.h"
#include "osal.h"

// SPI Callbacks for Client
extern uint8 *npSpiPollCallback( void );       // call client when POLL frame is received
extern void   npSpiReqCallback( uint8 type );  // call client when a AREQ or SREQ frame is received
extern bool   npSpiReadyCallback( void );      // call client to check if it has data ready to send

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  The MAC_ASSERT macro is for use during debugging.
 *  The given expression must evaluate as "true" or else fatal error occurs.
 *  At that point, the call stack feature of the debugger can pinpoint where the problem occurred.
 *
 *  To disable this feature and save code size, the project should define NP_SPI_NODEBUG to TRUE.
 */

#if !defined ( NP_SPI_NODEBUG )
  #define NP_SPI_NODEBUG              TRUE
#endif

#if ( NP_SPI_NODEBUG )
  #define NP_SPI_ASSERT( expr )
#else
  #define NP_SPI_ASSERT( expr)        HAL_ASSERT( expr )
#endif

#define DMATRIG_RX  HAL_DMA_TRIG_URX1
#define DMATRIG_TX  HAL_DMA_TRIG_UTX1
#define DMA_UDBUF   HAL_SPI_U1DBUF

#define DMA_RX() \
  st( \
    volatile uint8 ClearTheRxTrigger = *(volatile uint8 *)DMA_UDBUF; \
    \
    HAL_DMA_CLEAR_IRQ(HAL_DMA_CH_RX); \
    \
    HAL_DMA_ARM_CH(HAL_DMA_CH_RX); \
  )

#define DMA_TX( buf ) \
  st( \
    halDMADesc_t *ch = HAL_DMA_GET_DESC1234(HAL_DMA_CH_TX); \
    \
    HAL_DMA_SET_SOURCE(ch, (buf)); \
    \
    HAL_DMA_CLEAR_IRQ(HAL_DMA_CH_TX); \
    \
    HAL_DMA_ARM_CH(HAL_DMA_CH_TX); \
    \
    HAL_DMA_START_CH(HAL_DMA_CH_TX); \
  )

#define HAL_DMA_GET_SOURCE( pDesc, src ) \
  st( \
    src = (uint16)(pDesc->srcAddrH) << 8; \
    src += pDesc->srcAddrL; \
  )

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* UxDBUF - USART Receive/Transmit Data Buffer */
#define HAL_SPI_U0DBUF  0x70C1 // UART0
#define HAL_SPI_U1DBUF  0x70F9 // UART1

#define UTX0IE          0x04
#define UTX1IE          0x08

#ifdef SPI_CONFIG_ON_PORT1
// MRDY
#define NP_RDYIn      P1_3
#define NP_RDYIn_BIT  BV(3)
#define NP_RDYIn_IFG  P1IFG
// SRDY
#define NP_RDYOut     P1_2
#define NP_RDYOut_BIT BV(2)

#else
#define NP_RDYIn      P0_3
#define NP_RDYIn_BIT  BV(3)
#define NP_RDYIn_IFG  P0IFG

// SRDY
#define NP_RDYOut     P0_4
#define NP_RDYOut_BIT BV(4)

#endif

/* ------------------------------------------------------------------------------------------------
 *                                           TypeDefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  NP_SPI_SYNCH,          // new state added to synch with master; not used after startup
  NP_SPI_IDLE,           /* Idle, no transaction in progress. */
  NP_SPI_WAIT_RX,        /* Waiting for RX to complete. */
  NP_SPI_WAIT_TX,        /* Waiting for TX to complete. */
  NP_SPI_WAIT_AREQ       /* Waiting for asynchronous request to finish processing. */
} halSpiState_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

// buffer used to store the frame currently received or transmitted over SPI
static uint8 halSpiBuf[ HAL_SPI_BUF_LEN ];

// state of the current SPI transaction
static volatile halSpiState_t halSpiState;

// debug log
#if NP_SPI_NODEBUG
#define HAL_SPI_DBG_LOG(_trace)
#else
static volatile __no_init uint8 halSpiDbgLog[256] @ "PM0_XDATA";
static uint8 halSpiDbgLogIdx = 0;
#define HAL_SPI_DBG_LOG(_trace) (halSpiDbgLog[halSpiDbgLogIdx++]=(_trace))
#endif

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * -----------------------------------------------------------------------------------------------
 */

static void HalSpiDmaInit( void );
static void HalSpiUsartInit( void );
static void HalSpiGpioInit( void );

/* ------------------------------------------------------------------------------------------------
 *                              HAL SPI API
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          HalSpiInit
 *
 * @brief       This function is called to set up the SPI interface.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalSpiInit(void)
{
  // setup the MRDY and SRDY GPIO pins
  HalSpiGpioInit();

  // deassert SRDY as soon as possible; master gives 250ms after releasing the
  // slave from reset before it polls for an asserted SRDY (indicating that the
  // slave has completed startup and is ready)
  NP_RDYOut = 1;

  // setup USART1 to operate as a SPI slave
  HalSpiUsartInit();

  // setup DMA configuration tables for SPI RX and TX
  HalSpiDmaInit();

  // set the SPI FSM to SYNCH initially to wait for master sychronization
  halSpiState = NP_SPI_SYNCH;
}


/**************************************************************************************************
 * @fn          HalSpiGpioInit
 *
 * @brief       This function configures two GPIO ports for MRDY and SRDY. The
 *              MRDY input generates an interrupt on its falling edge.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void HalSpiGpioInit( void )
{
#ifdef SPI_CONFIG_ON_PORT1
  // setup GPIO pins for MRDY and SRDY
  P1SEL &= ~(NP_RDYIn_BIT | NP_RDYOut_BIT);
  P1DIR &= ~NP_RDYIn_BIT;
  P1DIR |=  NP_RDYOut_BIT;
  PICTL |= BV(1);                           // falling edge on P1 interrupt
  P1IFG &= ~NP_RDYIn_BIT;
  P1IEN |= BV(3);                           // interrupt enable on P1.3
  IEN2  |= BV(4);                           // enable P1 interrupts
#else
  // setup GPIO pins for MRDY and SRDY
  P0SEL &= ~(NP_RDYIn_BIT | NP_RDYOut_BIT);
  P0DIR &= ~NP_RDYIn_BIT;
  P0DIR |=  NP_RDYOut_BIT;
  PICTL |= BV(0);                           // falling edge on P0 interrupt
  P0IFG &= ~NP_RDYIn_BIT;
  P0IEN |= BV(3);                           // interrupt enable on P0.3
  IEN1  |= BV(5);                           // enable P0 interrupts
#endif
}


/**************************************************************************************************
 * @fn          HalSpiUsartInit
 *
 * @brief       This function configures the USART to be used for SPI slave
 *              control. Note that baud is not set as the master controls the
 *              clock.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void HalSpiUsartInit( void )
{
  // set USART1 serial bit order to MSB
  U1GCR |= BV(5);                   // ORDER: MSB first

  // set USART1 to SPI slave mode and enable RX
  U1CSR = BV(5) | BV(6);            // MODE: SPI mode, RE: enab RX, SLAVE: SPI slave

  // set USART1 I/O pins to Port 1 pins (alternative 2 location)
  PERCFG |= BV(1);                  // U1CFG: USART I/O Alt. 2 Location, P1.4-P1.7 are peripherals

  // select Port 1 pins for peripheral function for USART1
  P1SEL |= 0xF0;                    // SELP1_[7:4]

  // give USART1 priority over Timer3
  P2SEL &= ~(BV(5));                 // PRI2P1
}


/**************************************************************************************************
 * @fn          HalSpiDmaInit
 *
 * @brief       This function initializes the DMA for the SPI driver.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void HalSpiDmaInit(void)
{
  halDMADesc_t *ch;

  // Setup Tx by DMA
  ch = HAL_DMA_GET_DESC1234(HAL_DMA_CH_TX);

  // The start address of the source and destination.
  HAL_DMA_SET_SOURCE(ch, halSpiBuf);
  HAL_DMA_SET_DEST(ch, DMA_UDBUF);

  // Transfer the first byte + the number of bytes indicated by the first byte + 2 more bytes.
  HAL_DMA_SET_VLEN(ch, HAL_DMA_VLEN_1_P_VALOFFIRST_P_2);
  HAL_DMA_SET_LEN(ch, HAL_SPI_BUF_LEN);

  // One byte is transferred each time.
  HAL_DMA_SET_WORD_SIZE(ch, HAL_DMA_WORDSIZE_BYTE);

  // The bytes are transferred 1-by-1 on Tx Complete trigger.
  HAL_DMA_SET_TRIG_MODE(ch, HAL_DMA_TMODE_SINGLE);
  HAL_DMA_SET_TRIG_SRC(ch, DMATRIG_TX);

  // The source address is incremented by 1 byte after each transfer.
  HAL_DMA_SET_SRC_INC(ch, HAL_DMA_SRCINC_1);

  // The destination address is constant - the Tx Data Buffer.
  HAL_DMA_SET_DST_INC(ch, HAL_DMA_DSTINC_0);

  // The DMA shall issue an IRQ upon completion.
  HAL_DMA_SET_IRQ(ch, HAL_DMA_IRQMASK_ENABLE);

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8(ch, HAL_DMA_M8_USE_8_BITS);

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY(ch, HAL_DMA_PRI_HIGH);

  //////////////////////////////////////////////////////////////////////////////

  // Setup Rx by DMA.
  ch = HAL_DMA_GET_DESC1234(HAL_DMA_CH_RX);

  // The start address of the source and destination.
  HAL_DMA_SET_SOURCE(ch, DMA_UDBUF);
  HAL_DMA_SET_DEST(ch, halSpiBuf);

  // Transfer the first byte + the number of bytes indicated by the first byte + 2 more bytes.
  HAL_DMA_SET_VLEN(ch, HAL_DMA_VLEN_1_P_VALOFFIRST_P_2);
  HAL_DMA_SET_LEN(ch, HAL_SPI_BUF_LEN);

  HAL_DMA_SET_WORD_SIZE(ch, HAL_DMA_WORDSIZE_BYTE);

  // The bytes are transferred 1-by-1 on Rx Complete trigger.
  HAL_DMA_SET_TRIG_MODE(ch, HAL_DMA_TMODE_SINGLE);
  HAL_DMA_SET_TRIG_SRC(ch, DMATRIG_RX);

  // The source address is constant - the Rx Data Buffer.
  HAL_DMA_SET_SRC_INC(ch, HAL_DMA_SRCINC_0);

  // The destination address is incremented by 1 byte after each transfer.
  HAL_DMA_SET_DST_INC(ch, HAL_DMA_DSTINC_1);

  // The DMA shall issue an IRQ upon completion.
  HAL_DMA_SET_IRQ(ch, HAL_DMA_IRQMASK_ENABLE);

  // Xfer all 8 bits of a byte xfer.
  HAL_DMA_SET_M8(ch, HAL_DMA_M8_USE_8_BITS);

  // DMA has highest priority for memory access.
  HAL_DMA_SET_PRIORITY(ch, HAL_DMA_PRI_HIGH);
}


/**************************************************************************************************
 * @fn          npSpiIdle
 *
 * @brief       This function returns true if SPI is idle and there is no queued data.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      True if SPI is idle with no queued data.
 **************************************************************************************************
 */
bool npSpiIdle(void)
{
  return (halSpiState == NP_SPI_IDLE && !npSpiReadyCallback());
}


/**************************************************************************************************
 * @fn          npSpiAReqAlloc
 *
 * @brief       This function is called by SPI client to allocate a buffer in
 *              which to build an AREQ frame.
 *
 * input parameters
 *
 * @param       len - Length of the buffer required.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL for failure; otherwise a pointer to the data of an osal message.
 **************************************************************************************************
 */
uint8 *npSpiAReqAlloc(uint8 len)
{
  return osal_msg_allocate(len + RPC_FRAME_HDR_SZ);
}


/**************************************************************************************************
 * @fn          npSpiAReqReady
 *
 * @brief       This function is called by MT to notify the SPI driver that an AREQ frame is ready
 *              to be transmitted.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npSpiAReqReady(void)
{
  halIntState_t intState;
  HAL_ENTER_CRITICAL_SECTION(intState);

  HAL_SPI_DBG_LOG(0x01);
  if (halSpiState == NP_SPI_IDLE)
  {
    halSpiState = NP_SPI_WAIT_RX;
    DMA_RX();
    NP_RDYOut = 0; // assert SRDY to request POLL from master
  }

  HAL_EXIT_CRITICAL_SECTION(intState);
}


/**************************************************************************************************
 * @fn          npSpiAReqComplete
 *
 * @brief       This function is called by MT to notify the SPI driver that the processing of a
 *              received AREQ is complete.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npSpiAReqComplete(void)
{
  HAL_SPI_DBG_LOG(0x02);
  if (halSpiState == NP_SPI_WAIT_AREQ)
  {
    halSpiState = NP_SPI_IDLE;
  }
}


/**************************************************************************************************
 * @fn          npSpiSRspAlloc
 *
 * @brief       This function is called by MT to allocate a buffer in which to build an SRSP frame.
 *              MT must only call this function after processing a received SREQ frame.
 *
 * input parameters
 *
 * @param       len - Length of the buffer required.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL for failure; a pointer to the halSpiBuf on success. Success is determined by
 *              the correct halSpiState and H/W signals as well as a valid length request.
 **************************************************************************************************
 */
uint8 *npSpiSRspAlloc(uint8 len)
{
  // Remove unused parameter warning when NP_SPI_ASSERT macro is not enabled
  (void) len;

  if (halSpiState == NP_SPI_WAIT_TX)
  {
    NP_SPI_ASSERT(len <= HAL_SPI_BUF_LEN);
    return halSpiBuf;
  }
  else
  {
    return NULL;
  }
}


/**************************************************************************************************
 * @fn          npSpiSRspReady
 *
 * @brief       This function is called by MT to notify SPI driver that an SRSP is ready to Tx.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the buffer to transmit on the SPI.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npSpiSRspReady(uint8 *pBuf)
{
  volatile uint8 i = 1;

  HAL_SPI_DBG_LOG(0x03);
  if ((halSpiState == NP_SPI_WAIT_TX) && (NP_RDYOut == 0))
  {
    HAL_SPI_DBG_LOG(0x04);
    DMA_TX( pBuf );
    NP_RDYOut = 1;
  }
  else
  {
   while(i);
  }
}

/**************************************************************************************************
 * @fn          npSpiGetReqBuf
 *
 * @brief       This function is called by the application to get the buffer containing the
 *              currently received AREQ or SREQ.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      Pointer to the buffer containing the currently received AREQ or SREQ.
 **************************************************************************************************
 */
uint8 *npSpiGetReqBuf(void)
{
  if (halSpiState != NP_SPI_IDLE)
  {
    return halSpiBuf;
  }
  else
  {
    return NULL;
  }
}


/**************************************************************************************************
 * @fn          npSpiMrdyIsr
 *
 * @brief       This function is called when a GPIO falling-edge interrupt occurs on the MRDY.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npSpiMrdyIsr(void)
{
  HAL_SPI_DBG_LOG(0x11);
  if (halSpiState == NP_SPI_SYNCH)
  {
    // we have just been released from reset by the master, so deassert SRDY
    NP_RDYOut = 1;

    // ready SPI FSM for transactions
    halSpiState = NP_SPI_IDLE;
  }
  else if (halSpiState == NP_SPI_IDLE)
  {
    halSpiState = NP_SPI_WAIT_RX;

    // setup the DMA for receiving data
    DMA_RX();

    // assert SRDY
    NP_RDYOut = 0;
  }
}

/**************************************************************************************************
 * @fn          npSpiRxIsr
 *
 * @brief       This function handles the DMA Rx complete interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalSpiRxIsr(void)
{
  uint8 type = halSpiBuf[1] & RPC_CMD_TYPE_MASK;
  uint8 *pBuf, rdy = 1;

  NP_SPI_ASSERT(halSpiState == NP_SPI_WAIT_RX);

  switch (type)
  {
  case RPC_CMD_AREQ:
    // call client with message type so it can schedule an OSAL event
    // ISN'T THIS PROBLEMATIC? IF THE BUFFER HASN'T BEEN COPIED, OR THE DMA
    // POINTER CHANGED, WHAT'S TO STOP THE MASTER FROM SENDING ANOTHER AREQ?
    // THE STATE MIGHT CAUSE AN ASSERT, BUT THIS DOESN'T SEEM LIKE THE RIGHT
    // SOLUTION.
    HAL_SPI_DBG_LOG(0x12);
    npSpiReqCallback( RPC_CMD_AREQ );
    halSpiState = NP_SPI_WAIT_AREQ;
    break;

  case RPC_CMD_SREQ:
    // synchronous request received, so need to send back a synchronous reply
    // call client with message type so it can schedule an OSAL event
    HAL_SPI_DBG_LOG(0x13);
    npSpiReqCallback( RPC_CMD_SREQ );
    halSpiState = NP_SPI_WAIT_TX;
    rdy = 0; // keep SRDY asserted until SRSP is ready to be sent, then deassert so master can read back SRSP
    break;

  case RPC_CMD_POLL:
    // callback to assign pBuf with AREQ to send from slave
    // Note: this AREQ was already queued by the slave when it wanted to send
    //       an asynchronous command to the master by asserting SRDY.
    HAL_SPI_DBG_LOG(0x14);
    if ( (pBuf = npSpiPollCallback()) == NULL )
    {
      // nothing was queued, which is odd, so just send an empty frame?
      halSpiBuf[0] = 0;
      halSpiBuf[1] = 0;
      halSpiBuf[2] = 0;
      pBuf = halSpiBuf;
    }
    halSpiState = NP_SPI_WAIT_TX;
    DMA_TX(pBuf);
    break;

  default:
    HAL_SPI_DBG_LOG(0x15);
    halSpiState = NP_SPI_IDLE;
    break;
  }
  NP_RDYOut = rdy;
}

/**************************************************************************************************
 * @fn          npSpiTxIsr
 *
 * @brief       This function handles the DMA Tx complete interrupt.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalSpiTxIsr(void)
{
  halDMADesc_t *ch = HAL_DMA_GET_DESC1234(HAL_DMA_CH_TX);
  uint16 src;

  HAL_SPI_DBG_LOG(0x20);
  NP_SPI_ASSERT(halSpiState == NP_SPI_WAIT_TX);

  HAL_DMA_GET_SOURCE( ch, src );

  if ((uint8 *)src != halSpiBuf)
  {
    osal_msg_deallocate((uint8 *)src);
  }

  halSpiState = NP_SPI_IDLE;

  // Callback is required so that client can schedule to call npSpiMonitor
  // function.
  npSpiTxCompleteCallback();
}

/**************************************************************************************************
 * @fn          portIsr
 *
 * @brief       This function handles the GPIO port interrupt to handle the
 *              MRDY signal.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
#ifdef SPI_CONFIG_ON_PORT1
HAL_ISR_FUNCTION(port1Isr, P1INT_VECTOR)
{
  HAL_ENTER_ISR();

  if (P1IFG & NP_RDYIn_BIT)
  {
    npSpiMrdyIsr();
  }

  P1IFG = 0; // since interrupt is enabled for all four bits together clear all.
  P1IF = 0;

  HAL_EXIT_ISR();
}
#else
HAL_ISR_FUNCTION(port0Isr, P0INT_VECTOR)
{
  HAL_ENTER_ISR();

  if (P0IFG & NP_RDYIn_BIT)
  {
    npSpiMrdyIsr();
  }

  P0IFG = 0; // since interrupt is enabled for all four bits together clear all.
  P0IF = 0;

  HAL_EXIT_ISR();
}
#endif

/**************************************************************************************************
 * @fn          npSpiMonitor
 *
 * @brief       This function monitors the SPI signals for error conditions and for the end of a
 *              transaction. If an error is detected it attempts to recover.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void npSpiMonitor(void)
{
  switch (halSpiState)
  {
  case NP_SPI_SYNCH:
    break;

  case NP_SPI_IDLE:
    NP_SPI_ASSERT((NP_RDYIn_IFG & NP_RDYIn_BIT) == 0);
    break;

  case NP_SPI_WAIT_RX:
    NP_SPI_ASSERT((HAL_DMA_CHECK_IRQ(HAL_DMA_CH_RX)) == 0);
    break;

  case NP_SPI_WAIT_TX:
    NP_SPI_ASSERT((HAL_DMA_CHECK_IRQ(HAL_DMA_CH_TX)) == 0);
    break;

  case NP_SPI_WAIT_AREQ:
    break;

  default:
    NP_SPI_ASSERT(0);
    break;
  }

  if (halSpiState == NP_SPI_IDLE)
  {
    /* Poll for MRDY in case it was set before slave had setup the ISR.
     * Also, async responses may get queued, so flush them out here.
     */
    if ((NP_RDYIn == 0) || (npSpiReadyCallback()))
    {
      npSpiAReqReady();
    }
  }
}

/**************************************************************************************************
 * @fn          HalSpiAssertSrdy
 *
 * @brief       This function is called to assert the SRDY signal.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void HalSpiAssertSrdy(void)
{
  // assert SRDY to indicate to the master that the slave is ready
  NP_RDYOut = 0;
}

#endif
/**************************************************************************************************
*/
