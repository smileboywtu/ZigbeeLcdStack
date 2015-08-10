/*********************************************************************
 *
 *
 * @Reverse Time:
 *				set the number of the list in the list, so here the device
 *			only associated limited size. set "NWK_MAX_DEVICE_LIST" in the
 *			pre-compile option.
 *						--2015/4/30
 *
 */

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "WirelessMonitorSystem.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_uart.h"


/*********************************************************************
 * MACROS
 */

// Max in / out Cluster number
#define Monitor_MAX_IN_CLUSTERS			1
#define Monitor_MAX_OUT_CLUSTERS		10

// State For Monitor Data
#define Machine_SOF_STATE			0
#define Machine_LEN_STATE			1
#define Machine_CMD1_STATE	 		2
#define Machine_CMD2_STATE			3
#define Machine_DST1_STATE			4
#define Machine_DST2_STATE 			5
#define Machine_DATA_STATE			6
#define Machine_FCS_STATE			7	

// START FRAME OF MONITOR DATA
#define MONITOR_START_FRMAE			0xFE

// EVENT
#define MONITOR_SEND_DATA_EVT		0x0001
#define MONITOR_RESEND_EVT			0x0002

// Delay a time to make sure the coordinator can receive data
// if not delay the coordinator can't receive data from other node
#define MONITOR_SEND_DELAY			(RESPONSE_POLL_RATE * 2)

// resend delay
#define MONITOR_RESEND_DELAY	10

/*********************************************************************
 * CONSTANTS
 */
// ACK
const byte ACK[] = {
  0xFE,
  0x00,
  LO_UINT16(ACK_CMD),
  HI_UINT16(ACK_CMD),
  0x00, 
  0x00,
  0x00};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t Monitor_ClusterInList[Monitor_MAX_IN_CLUSTERS] =
{
	ZIGBEE_COMMON_CLUSTER
};

const cId_t Monitor_ClusterOutList[Monitor_MAX_OUT_CLUSTERS] =
{
	TOPO_REQ,
	IMAGE_REQ,
	LCD_SUBJECT_CMD,
	LCD_CLASS_CMD,
	LCD_TEACHER_CMD,
	LCD_PEOPLE_CMD,
	LCD_TIME_CMD,
	ROOM_CMD,
	DATA_ALLOW,
	DATA_REFUSE
};

const SimpleDescriptionFormat_t Monitor_SimpleDesc =
{
  Monitor_ENDPOINT,              //  int Endpoint;
  Monitor_PROFID,                //  uint16 AppProfId[2];
  Monitor_DEVICEID,              //  uint16 AppDeviceId[2];
  Monitor_DEVICE_VERSION,        //  int   AppDevVer:4;
  Monitor_FLAGS,                 //  int   AppFlags:4;
  Monitor_MAX_IN_CLUSTERS,           //  byte  AppNumInClusters;
  (cId_t *)Monitor_ClusterInList,    //  byte *pAppInClusterList;
  Monitor_MAX_OUT_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)Monitor_ClusterOutList    //  byte *pAppOutClusterList;
};

endPointDesc_t Monitor_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/**************************** OSAL **********************************/
byte Monitor_TaskID;    // Task ID for internal task/event processing

byte Monitor_TransID;   // This is the unique message ID (counter)

devStates_t Monitor_NwkState;	// record the network state

afAddrType_t Monitor_DstAddr;	// Bind endpoint address

/*************************** STATE *********************************/
static byte readStep = Machine_SOF_STATE;

static byte length;

static byte fcs;

static byte recvDataLen;

static byte* pData = NULL;

/******************************resend buffer ***********************/
static byte resendBuffer[128] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Monitor_UartInit( void );
static void Monitor_UartCallBack( uint8 port, uint8 event );
static void Monitor_MessageMSGCB( afIncomingMSGPacket_t *pckt ); 
static void Monitor_ProcessMonitorData( void );
static void Monitor_SendTopologyInformation( void );
static byte Monitor_CalcFCS( uint8 *msg_ptr, uint8 len );
static void Monitor_ProcessMonitorIncomingData(byte* pData);

/*********************************************************************
 * @fn      Monitor_Init
 *
 * @brief   Initialization function for the Sensor App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void WirelessMonitorSystem_Init( uint8 task_id )
{
  Monitor_TaskID = task_id;
  Monitor_NwkState = DEV_INIT;
  Monitor_TransID = 0;
  
  // init the uart, keep care do not use flow control
  Monitor_UartInit();
  
  // Destinations
  Monitor_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  Monitor_DstAddr.endPoint = Monitor_ENDPOINT;
  Monitor_DstAddr.addr.shortAddr = 0xFFFF;	

  // Fill out the endpoint description.
  Monitor_epDesc.endPoint = Monitor_ENDPOINT;
  Monitor_epDesc.task_id = &Monitor_TaskID;
  Monitor_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Monitor_SimpleDesc;
  Monitor_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Monitor_epDesc );
  
}

/*********************************************************************
 * @fn      Monitor_ProcessEvent
 *
 * @brief   Sensor Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 WirelessMonitorSystem_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  		  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Monitor_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

		  if( sentStatus == ZSuccess )
		  {
			// Action taken when confirmation is received.
		  	osal_start_timerEx(	Monitor_TaskID,
							 	MONITOR_SEND_DATA_EVT,
							 	MONITOR_SEND_DELAY);	
		  }
          break;

        case AF_INCOMING_MSG_CMD:
          Monitor_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          Monitor_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (Monitor_NwkState == DEV_ZB_COORD)
              || (Monitor_NwkState == DEV_ROUTER)
              || (Monitor_NwkState == DEV_END_DEVICE) )
          { 
			// start signal to the coordinator
			HalUARTWrite(UART_PORT, (byte*)ACK, 7);
			// send the topology information
			Monitor_SendTopologyInformation();
          }
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Monitor_TaskID );
    }
	
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // continue to send event
  if( events & MONITOR_SEND_DATA_EVT )
  {
	// process the data
	Monitor_ProcessMonitorData();
	
	// return unprocessed events
	return (events ^ MONITOR_SEND_DATA_EVT);
  }
  
  // resend the data
  if( events & MONITOR_RESEND_EVT )
  {
	// resend the data
	Monitor_ProcessMonitorIncomingData( resendBuffer );
	
	// return unprocessed events
	return (events ^ MONITOR_RESEND_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      Monitor_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void Monitor_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  // just Write to usart
  HalUARTWrite(UART_PORT, pkt->cmd.Data, pkt->cmd.DataLength);
}


/*********************************************************************
 *
 * @fn		Monitor_UartInit
 *
 * @brief	this method init the usart for the coordinator
 *
 *
 * @param   None
 *
 * @return  void
 */
void Monitor_UartInit()
{
  // create the uart structure
  halUARTCfg_t uartConfig;
  // set the parameters
  uartConfig.configured           = TRUE;               
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 128;   				
  uartConfig.rx.maxBufSize        = 256;  				
  uartConfig.tx.maxBufSize        = 256;  				
  uartConfig.idleTimeout          = 6;    				
  uartConfig.intEnable            = TRUE;               
  uartConfig.callBackFunc         = Monitor_UartCallBack;
  // open it
  HalUARTOpen (UART_PORT, &uartConfig);
}
 
/*********************************************************************
 *
 * @fn		Monitor_UartCallBack
 *
 * @brief	this method deal with the uart event
 *
 *
 * @param   port  -- uart port
 *			event -- event occur
 *
 * @return  void
 */
void Monitor_UartCallBack( uint8 port, uint8 event )
{
  (void)port;

  // if Rx interrupt has occur
  if ((event & (HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL)))
  {
	// deal with the coming data 
	Monitor_ProcessMonitorData();
  }
}

/*********************************************************************
 * @fn      Monitor_ProcessMonitorData()
 *
 * @brief   process the message from the pc monitor, this
 *			message is send by monitor and will send to the node.
 *
 * @return  none
 */
void Monitor_ProcessMonitorData()
{
  uint8  ch;				
  uint8  bytesInRxBuffer;
  
  // if the Rx hava data
  while ( Hal_UART_RxBufLen(UART_PORT) )
  {
    HalUARTRead (UART_PORT, &ch, 1);
    switch (readStep)
    {
	  // start of the frame
      case Machine_SOF_STATE:
        if (MONITOR_START_FRMAE == ch)
          readStep = Machine_LEN_STATE;
        break;
	  // record length
      case Machine_LEN_STATE:
        length = ch;
        recvDataLen = 0;
        pData = (byte*)osal_mem_alloc(length+5);
		osal_memset(pData, 0x00, length+5);
        if (NULL != pData)
        {
		  // record the length in pData
		  pData[0] = length;
          readStep = Machine_CMD1_STATE;
        }
        else
        {
          readStep = Machine_SOF_STATE;
          return;
        }
        break;
	  // record command
      case Machine_CMD1_STATE:
		pData[1] = ch;
        readStep = Machine_CMD2_STATE;
        break;
      case Machine_CMD2_STATE:
        pData[2] = ch;
		readStep = Machine_DST1_STATE;
		break;
	  // record dst addr
	  case Machine_DST1_STATE:
		pData[3] = ch;
		readStep = Machine_DST2_STATE;
		break;
	  case Machine_DST2_STATE:
	  	pData[4] = ch;
        if (0 != length)
        {
          readStep = Machine_DATA_STATE;
        }
        else
        {
          readStep = Machine_FCS_STATE;
        }
        break;
	  // receive data
      case Machine_DATA_STATE:

        /* Fill in the buffer the first byte of the data */
        pData[5+recvDataLen++] = ch;

        /* Check number of bytes left in the Rx buffer */
        bytesInRxBuffer = Hal_UART_RxBufLen(UART_PORT);

        /* If the remain of the data is there, read them all, otherwise, just read enough */
        if (bytesInRxBuffer <= length - recvDataLen)
        {
          HalUARTRead (UART_PORT, &pData[5+recvDataLen], bytesInRxBuffer);
          recvDataLen += bytesInRxBuffer;
        }
        else
        {
          HalUARTRead (UART_PORT, &pData[5+recvDataLen], length - recvDataLen);
          recvDataLen += (length - recvDataLen);
        }

        /* If number of bytes read is equal to data length, time to move on to FCS */
        if ( recvDataLen == length )
            readStep = Machine_FCS_STATE;

        break;
	  // check fcs
      case Machine_FCS_STATE:
        fcs = ch;
        /* Make sure it's correct */
        if (Monitor_CalcFCS ((uint8*)pData, length+5) == fcs)
        {
		  // if this is the allow start command from pc
		  // this allow the coordinator to ack start when
		  // it's already start when the application from
		  // pc is started. this is very flexiable for the 
		  // user
		  Monitor_ProcessMonitorIncomingData(pData);
		}
        else
        {
          /* deallocate the msg */
          osal_mem_free ( (uint8 *)pData );
        }
		
        /* Reset the readStep, send or discard the buffers at this point */
        readStep = Machine_SOF_STATE;
     	break;
    }
	// return to wait the send signal
	if( 0 == Hal_UART_RxBufLen(UART_PORT) )
	{
	  // all data hava read out
  	  HalUARTWrite(UART_PORT, (byte*)ACK, 7);
	}else{
	  break;	// get out the loop
	}
  }// read all data
}

/*********************************************************************
 *
 * @fn	Monitor_ProcessMonitorIncomingData
 *
 *
 */
void Monitor_ProcessMonitorIncomingData(byte* pData)
{
  // get the cluster first
  uint16 clusterID = BUILD_UINT16(pData[1], pData[2]);
  // record the AF status
  uint8 sendResult = afStatus_FAILED;
  
  // send the message
  if( START_CMD == clusterID )
  {
	// also need to send the topology information
	Monitor_SendTopologyInformation();
	// boadcast the toporequest
  	Monitor_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  	Monitor_DstAddr.addr.shortAddr = 0xFFFF;
	// broadcast
	sendResult = AF_DataRequest(	&Monitor_DstAddr, 
				    				&Monitor_epDesc,
									TOPO_REQ,
									0,
									(byte*)NULL,
									&Monitor_TransID,
									AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
	
  }
  else if((DATA_ALLOW == clusterID) || (DATA_REFUSE == clusterID))
  {
	// this is mainly for the vibrate when receiving camera data
	// boadcast the toporequest
  	Monitor_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  	Monitor_DstAddr.addr.shortAddr = 0xFFFF;
	// broadcast
	sendResult = AF_DataRequest(	&Monitor_DstAddr, 
									&Monitor_epDesc,
									clusterID,
									0,
									(byte*)NULL,
									&Monitor_TransID,
									AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  else
  {
	// set dst use point to point transmit
  	Monitor_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  	Monitor_DstAddr.addr.shortAddr = BUILD_UINT16(pData[3], pData[4]);
	
	// send to the dst node
  	sendResult = AF_DataRequest(	&Monitor_DstAddr, 
									&Monitor_epDesc,
									clusterID,
									pData[0],
									(byte*)&pData[5],
									&Monitor_TransID,
									AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  // check if need to resend the message
  if( afStatus_SUCCESS == sendResult ){
	// nothing to do
	
  }else{
	// copy the message to the resend memory
	osal_memcpy((byte*)resendBuffer, pData, pData[0]);
	// here resend
	osal_start_timerEx( Monitor_TaskID, MONITOR_RESEND_EVT, MONITOR_RESEND_DELAY );
  }
  // realease the memory
  osal_mem_free ((byte*)pData);
}

/*********************************************************************
 * @fn		Monitor_SendTopologyInformation
 *
 * @brief	send the topology information to the the coordinator
 *
 *	| SOF  | LEN  |  CMD |  CMD2  | SRC  |  SRC |  DATA  | FCS  |
 */
static void Monitor_SendTopologyInformation()
{
  // Create frame buffer
  byte pFrame[11];
  // fill SOF
  pFrame[0] = 0xFE;
  // fill len
  pFrame[1] = 0x0004;
  // file CMD
  pFrame[2] = LO_UINT16(TOPOLOGY_CMD);
  pFrame[3] = HI_UINT16(TOPOLOGY_CMD);
  // file source address
  pFrame[4] = LO_UINT16(0);
  pFrame[5] = HI_UINT16(0);
  // fill type
  pFrame[6] = LO_UINT16(COORDINATOR);
  pFrame[7] = HI_UINT16(COORDINATOR);
  // file parent
  pFrame[8] = LO_UINT16(0);
  pFrame[9] = HI_UINT16(0);
  // fill fcs
  pFrame[10] = Monitor_CalcFCS((byte*)&pFrame[1], 9);
  // send out use uart
  HalUARTWrite(UART_PORT, pFrame, 11);
}

/*********************************************************************
 * @fn      Monitor_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ********************************************************************/
byte Monitor_CalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult = 0x00;

  for ( x=0; x<len; x++ )
    xorResult ^=  msg_ptr[x];

  return ( xorResult );
}
/*********************************************************************
 */
