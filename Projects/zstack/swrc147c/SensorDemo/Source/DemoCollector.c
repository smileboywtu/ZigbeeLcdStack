/**************************************************************************************************
  Filename:       DemoCollector.c

  Description:    Collector application for the Sensor Demo utilizing Simple API.

                  The collector node can be set in a state where it accepts 
                  incoming reports from the sensor nodes, and can send the reports
                  via the UART to a PC tool. The collector node in this state
                  functions as a gateway. The collector nodes that are not in the
                  gateway node function as routers in the network.  


  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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

/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "DemoApp.h"

/******************************************************************************
 * CONSTANTS
 */

#define REPORT_FAILURE_LIMIT                4
#define ACK_REQ_INTERVAL                    5 // each 5th packet is sent with ACK request

// General UART frame offsets
#define FRAME_SOF_OFFSET                    0
#define FRAME_LENGTH_OFFSET                 1 
#define FRAME_CMD0_OFFSET                   2
#define FRAME_CMD1_OFFSET                   3
#define FRAME_DATA_OFFSET                   4

// ZB_RECEIVE_DATA_INDICATION offsets
#define ZB_RECV_SRC_OFFSET                  0
#define ZB_RECV_CMD_OFFSET                  2
#define ZB_RECV_LEN_OFFSET                  4
#define ZB_RECV_DATA_OFFSET                 6
#define ZB_RECV_FCS_OFFSET                  8

// ZB_RECEIVE_DATA_INDICATION frame length
#define ZB_RECV_LENGTH                      15

// PING response frame length and offset
#define SYS_PING_RSP_LENGTH                 7 
#define SYS_PING_CMD_OFFSET                 1

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define CPT_SOP                             0xFE
#define SYS_PING_REQUEST                    0x0021
#define SYS_PING_RESPONSE                   0x0161
#define ZB_RECEIVE_DATA_INDICATION          0x8746

// Application States
#define APP_INIT                            0
#define APP_START                           2
#define APP_BINDED                          3    

// Application osal event identifiers
#define MY_START_EVT                        0x0001
#define MY_REPORT_EVT                       0x0002
#define MY_FIND_COLLECTOR_EVT               0x0004


/******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16              source;
  uint16              parent;
  uint8               temp;
  uint8               voltage;
} gtwData_t;

/******************************************************************************
 * LOCAL VARIABLES
 */

static uint8 appState =             APP_INIT;
static uint8 reportState =          FALSE;
static uint8 myStartRetryDelay =    10;          // milliseconds
static uint8 isGateWay =            FALSE;
static uint16 myBindRetryDelay =    2000;        // milliseconds
static uint16 myReportPeriod =      2000;        // milliseconds

static uint8 reportFailureNr =      0;
static uint16 parentShortAddr;
static gtwData_t gtwData;

/******************************************************************************
 * LOCAL FUNCTIONS
 */

static uint8 calcFCS(uint8 *pBuf, uint8 len);
static void sysPingReqRcvd(void);
static void sysPingRsp(void);
static void sendGtwReport(gtwData_t *gtwData);
static void sendDummyReport(void);

/******************************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Collector device
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 2

// List of output and input commands for Collector device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

const cId_t zb_OutCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

// Define SimpleDescriptor for Collector device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};

/******************************************************************************
 * FUNCTIONS
 */

/******************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{
  uint8 logicalType;
  
  if(event & SYS_EVENT_MSG)
  {
    
  }
  
  if( event & ZB_ENTRY_EVENT )
  {  
    // Initialise UART
    initUart(uartRxCB);
    
    // blind LED 1 to indicate starting/joining a network
    HalLedBlink ( HAL_LED_1, 0, 50, 500 );
    HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
    
    // Read logical device type from NV
    zb_ReadConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
   
    // Start the device 
    zb_StartRequest();
  }
  
  if ( event & MY_START_EVT )
  {
    zb_StartRequest();
  }
  
  if ( event & MY_REPORT_EVT )
  {
    if (isGateWay) 
    {
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
    else if (appState == APP_BINDED) 
    {
      sendDummyReport();
      osal_start_timerEx( sapi_TaskID, MY_REPORT_EVT, myReportPeriod );
    }
  }
  if ( event & MY_FIND_COLLECTOR_EVT )
  { 
    // Find and bind to a gateway device (if this node is not gateway)
    if (!isGateWay) 
    {
      zb_BindDevice( TRUE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );
    }
  }
  
}

/******************************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  static uint8 allowBind=FALSE;
  static uint8 allowJoin=TRUE;
  uint8 logicalType;
  
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      if ( appState == APP_INIT  )
      {
        // Key 1 starts device as a coordinator
        logicalType = ZG_DEVICETYPE_COORDINATOR;
        zb_WriteConfiguration(ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType);
                
        // Reset the device with new configuration
        zb_SystemReset();
      }
    }
    if ( keys & HAL_KEY_SW_2 )
    {
      allowBind ^= 1;
      if (allowBind) 
      {
        // Turn ON Allow Bind mode infinitly
        zb_AllowBind( 0xFF );
        HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
        //This node is the gateway node
        isGateWay = TRUE;
        
        // Update the display
        #if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "Gateway Mode", HAL_LCD_LINE_2 );
        #endif
      }
      else
      {
        // Turn OFF Allow Bind mode infinitly
        zb_AllowBind( 0x00 );
        HalLedSet( HAL_LED_2, HAL_LED_MODE_OFF );
        isGateWay = FALSE;
        
        // Update the display
        #if defined ( LCD_SUPPORTED )
        HalLcdWriteString( "Collector", HAL_LCD_LINE_2 );
        #endif
      }
    }
    if ( keys & HAL_KEY_SW_3 )
    {
      // Start reporting
      osal_set_event( sapi_TaskID, MY_REPORT_EVT );
    }
    if ( keys & HAL_KEY_SW_4 )
    {
      // Key 4 is used to control which routers 
      // that can accept join requests
      allowJoin ^= 1;
      if(allowJoin)
      {
        NLME_PermitJoiningRequest(0xFF);
      }
      else {
        NLME_PermitJoiningRequest(0);
      }
    }
  }
}

/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{ 
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS )   
  {
    // Set LED 1 to indicate that node is operational on the network
    HalLedSet( HAL_LED_1, HAL_LED_MODE_ON );
    
    // Update the display
    #if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SensorDemo", HAL_LCD_LINE_1 );
    HalLcdWriteString( "Collector", HAL_LCD_LINE_2 );
    #endif
    
    // Change application state
    appState = APP_START;
    
    // Set event to bind to a collector
    osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );
       
     // Store parent short address
    zb_GetDeviceInfo(ZB_INFO_PARENT_SHORT_ADDR, &parentShortAddr);
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee stack after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  if ( status != ZB_SUCCESS && !isGateWay ) 
  {
    if ( ++reportFailureNr>=REPORT_FAILURE_LIMIT ) 
    {   
       // Stop reporting
       osal_stop_timerEx( sapi_TaskID, MY_REPORT_EVT );
       
       // After failure reporting start automatically when the device
       // is binded to a new gateway
       reportState=TRUE;
       
       // Delete previous binding
       zb_BindDevice( FALSE, DUMMY_REPORT_CMD_ID, (uint8 *)NULL );
       
       // Try binding to a new gateway
       osal_set_event( sapi_TaskID, MY_FIND_COLLECTOR_EVT );
       reportFailureNr=0;
    }
  }
  else if ( !isGateWay ) 
  {
    reportFailureNr=0;
  }
}

/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{
  if( status == ZB_SUCCESS )
  {
    appState = APP_BINDED;
    // Set LED2 to indicate binding successful
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    
    // After failure reporting start automatically when the device
    // is binded to a new gateway
    if ( reportState ) 
    {
      // Start reporting
      osal_set_event( sapi_TaskID, MY_REPORT_EVT );
    }
  }
  else
  {
    osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
  
}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}

/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{ 
  gtwData.parent = BUILD_UINT16(pData[SENSOR_PARENT_OFFSET+ 1], pData[SENSOR_PARENT_OFFSET]);
  gtwData.source=source;
  gtwData.temp=*pData;
  gtwData.voltage=*(pData+1);
  
  // Flash LED 2 once to indicate data reception
  HalLedSet ( HAL_LED_2, HAL_LED_MODE_FLASH );
  
  // Update the display
  #if defined ( LCD_SUPPORTED )
  HalLcdWriteScreen( "Report", "rcvd" );
  #endif
  
  // Send gateway report
  sendGtwReport(&gtwData);
}

/******************************************************************************
 * @fn          uartRxCB
 *
 * @brief       Callback function for UART 
 *
 * @param       port - UART port
 *              event - UART event that caused callback 
 *
 * @return      none
 */
void uartRxCB( uint8 port, uint8 event )
{
  uint8 pBuf[RX_BUF_LEN];
  uint16 cmd;
  uint16 len;
  
  if ( event != HAL_UART_TX_EMPTY ) 
  {
  
    // Read from UART
    len = HalUARTRead( HAL_UART_PORT_0, pBuf, RX_BUF_LEN );
    
    if ( len>0 ) 
    {
      cmd = BUILD_UINT16(pBuf[SYS_PING_CMD_OFFSET+ 1], pBuf[SYS_PING_CMD_OFFSET]);
  
      if( (pBuf[FRAME_SOF_OFFSET] == CPT_SOP) && (cmd == SYS_PING_REQUEST) ) 
      {
        sysPingReqRcvd();
      }
    }
  }
}

/******************************************************************************
 * @fn          sysPingReqRcvd
 *
 * @brief       Ping request received 
 *
 * @param       none
 *              
 * @return      none
 */
static void sysPingReqRcvd(void)
{
   sysPingRsp();
}

/******************************************************************************
 * @fn          sysPingRsp
 *
 * @brief       Build and send Ping response
 *
 * @param       none
 *              
 * @return      none
 */
static void sysPingRsp(void)
{
  uint8 pBuf[SYS_PING_RSP_LENGTH];
  
  // Start of Frame Delimiter
  pBuf[FRAME_SOF_OFFSET] = CPT_SOP;
  
  // Length
  pBuf[FRAME_LENGTH_OFFSET] = 2; 
  
  // Command type
  pBuf[FRAME_CMD0_OFFSET] = LO_UINT16(SYS_PING_RESPONSE); 
  pBuf[FRAME_CMD1_OFFSET] = HI_UINT16(SYS_PING_RESPONSE);
  
  // Stack profile
  pBuf[FRAME_DATA_OFFSET] = LO_UINT16(STACK_PROFILE);
  pBuf[FRAME_DATA_OFFSET+ 1] = HI_UINT16(STACK_PROFILE);
  
  // Frame Check Sequence
  pBuf[SYS_PING_RSP_LENGTH - 1] = calcFCS(&pBuf[FRAME_LENGTH_OFFSET], (SYS_PING_RSP_LENGTH - 2));
  
  // Write frame to UART
  HalUARTWrite(HAL_UART_PORT_0,pBuf, SYS_PING_RSP_LENGTH);
}

/******************************************************************************
 * @fn          sendGtwReport
 *
 * @brief       Build and send gateway report
 *
 * @param       none
 *              
 * @return      none
 */
static void sendGtwReport(gtwData_t *gtwData)
{
  uint8 pFrame[ZB_RECV_LENGTH];
  
  // Start of Frame Delimiter
  pFrame[FRAME_SOF_OFFSET] = CPT_SOP; // Start of Frame Delimiter
  
  // Length
  pFrame[FRAME_LENGTH_OFFSET] = 10;
  
  // Command type
  pFrame[FRAME_CMD0_OFFSET] = LO_UINT16(ZB_RECEIVE_DATA_INDICATION);   
  pFrame[FRAME_CMD1_OFFSET] = HI_UINT16(ZB_RECEIVE_DATA_INDICATION); 
  
  // Source address
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_SRC_OFFSET] = LO_UINT16(gtwData->source); 
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_SRC_OFFSET+ 1] = HI_UINT16(gtwData->source);
  
  // Command ID
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_CMD_OFFSET] = LO_UINT16(SENSOR_REPORT_CMD_ID); 
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_CMD_OFFSET+ 1] = HI_UINT16(SENSOR_REPORT_CMD_ID);
  
  // Length
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_LEN_OFFSET] = LO_UINT16(4); 
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_LEN_OFFSET+ 1] = HI_UINT16(4);
  
  // Data
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_DATA_OFFSET] = gtwData->temp;
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_DATA_OFFSET+ 1] = gtwData->voltage; 
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_DATA_OFFSET+ 2] = LO_UINT16(gtwData->parent); 
  pFrame[FRAME_DATA_OFFSET+ ZB_RECV_DATA_OFFSET+ 3] = HI_UINT16(gtwData->parent);
  
  // Frame Check Sequence
  pFrame[ZB_RECV_LENGTH - 1] = calcFCS(&pFrame[FRAME_LENGTH_OFFSET], (ZB_RECV_LENGTH - 2) );
  
  // Write report to UART
  HalUARTWrite(HAL_UART_PORT_0,pFrame, ZB_RECV_LENGTH);
}

/******************************************************************************
 * @fn          sendDummyReport
 *
 * @brief       Send dummy report (used to visualize collector nodes on PC GUI)
 *
 * @param       none
 *              
 * @return      none
 */
static void sendDummyReport(void)
{
  uint8 pData[SENSOR_REPORT_LENGTH];
  static uint8 reportNr=0;
  uint8 txOptions;
  
  // dummy report data
  pData[SENSOR_TEMP_OFFSET] =  0xFF;
  pData[SENSOR_VOLTAGE_OFFSET] = 0xFF; 
    
  pData[SENSOR_PARENT_OFFSET] =  HI_UINT16(parentShortAddr);
  pData[SENSOR_PARENT_OFFSET+ 1] =  LO_UINT16(parentShortAddr);
  
  // Set ACK request on each ACK_INTERVAL report
  // If a report failed, set ACK request on next report
  if ( ++reportNr<ACK_REQ_INTERVAL && reportFailureNr==0 ) 
  {
    txOptions = AF_TX_OPTIONS_NONE;
  }
  else 
  {
    txOptions = AF_MSG_ACK_REQUEST;
    reportNr = 0;
  }
  
  // Destination address 0xFFFE: Destination address is sent to previously
  // established binding for the commandId.
  zb_SendDataRequest( 0xFFFE, DUMMY_REPORT_CMD_ID, SENSOR_REPORT_LENGTH, pData, 0, txOptions, 0 );
}

/******************************************************************************
 * @fn          calcFCS
 *
 * @brief       This function calculates the FCS checksum for the serial message 
 *
 * @param       pBuf - Pointer to the end of a buffer to calculate the FCS.
 *              len - Length of the pBuf.
 *
 * @return      The calculated FCS.
 ******************************************************************************
 */
static uint8 calcFCS(uint8 *pBuf, uint8 len)
{
  uint8 rtrn = 0;

  while (len--)
  {
    rtrn ^= *pBuf++;
  }

  return rtrn;
}

