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
// Cluster number for LCD
#define LCD_MAX_IN_CLUSTERS 	7
#define LCD_MAX_OUT_CLUSTERS	1

// ACK command
#define LCD_ACK_CMD 	0xFFFE
#define LCD_RETRY_CMD	0xFFFD

/*********************************************************************
 * TYPEDEFS
 */
typedef struct{
  byte data[64];
  byte len;
}MessageNode;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t LCD_ClusterInList[LCD_MAX_IN_CLUSTERS] =
{
  LCD_SUBJECT_CMD,			
  LCD_CLASS_CMD,				
  LCD_TEACHER_CMD,				
  LCD_PEOPLE_CMD,				
  LCD_TIME_CMD,
  TOPO_REQ,
  ROOM_CMD
};

const cId_t LCD_ClusterOutList[LCD_MAX_OUT_CLUSTERS] =
{
  ZIGBEE_COMMON_CLUSTER
};

const SimpleDescriptionFormat_t LCD_SimpleDesc =
{
  Monitor_ENDPOINT,              //  int Endpoint;
  Monitor_PROFID,                //  uint16 AppProfId[2];
  Monitor_DEVICEID,              //  uint16 AppDeviceId[2];
  Monitor_DEVICE_VERSION,        //  int   AppDevVer:4;
  Monitor_FLAGS,                 //  int   AppFlags:4;
  LCD_MAX_IN_CLUSTERS,           //  byte  AppNumInClusters;
  (cId_t *)LCD_ClusterInList, //  byte *pAppInClusterList;
  LCD_MAX_OUT_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)LCD_ClusterOutList //  byte *pAppOutClusterList;
};

endPointDesc_t LCD_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/**************************** OSAL *********************************/
byte LCD_TaskID;    // Task ID for internal task/event processing

byte LCD_TransID;  // This is the unique message ID (counter)

devStates_t LCD_NwkState;

afAddrType_t LCD_DstAddr;	

/***************************** screen parameters ******************/
// here hold the message and send to the LCD
static byte pHeader = 0;
static byte pTail = 0;
static MessageNode messageArray[12];

// state for the ack flag
static byte ackFlag = 0;
static byte shiftToRoomInformation = 0;
// list for lcd
const uint16 roomReg = 		0x0010;
const uint16 courseReg = 	0x0020;
const uint16 teacherReg = 	0x0040;
const uint16 classReg	= 	0x0050;
const uint16 peopleReg = 	0x0070;
const uint16 timeReg = 		0x0080;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void LCD_Loading( void );
static void LCD_RoomPlate( void );
static void LCD_UartInit( void );
static void LCD_SendRetry( void );
static void LCD_SendACK( byte flag );
static void LCD_SendTopoInformation( void );
static void copyExtAddr(byte* src, byte* dst);
static byte LCD_CalcFCS( uint8 *msg_ptr, uint8 len );
static void LCD_UartCallBack( uint8 port, uint8 event );
static void LCD_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void LCD_BuildScreenFrame(uint8* gbk, uint8 len, uint16 reg);

// hold for the queue
static void LCD_InitMessageQueue( void );
static void LCD_PushMessage( byte* str, byte len );
static MessageNode* LCD_PopMessage( void );
static byte LCD_IsMessageEmpty( void );

// send one message to the lcd
static void LCD_SendOneMessage( void );

/*********************************************************************
 * @fn      LCD_Init
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
  // Init the Stack
  LCD_TaskID = task_id;
  LCD_NwkState = DEV_INIT;
  LCD_TransID = 0;
	
  // Init the LCD state
  ackFlag = 0;
  shiftToRoomInformation = 0;
  // init the queue
  LCD_InitMessageQueue();
  
  // Init the uart for the LCD
  LCD_UartInit();

  // init destination address
  LCD_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  LCD_DstAddr.endPoint = Monitor_ENDPOINT;
  LCD_DstAddr.addr.shortAddr = 0x0000;		// coordinator
	
  // Fill out the endpoint description.
  LCD_epDesc.endPoint = Monitor_ENDPOINT;
  LCD_epDesc.task_id = &LCD_TaskID;
  LCD_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&LCD_SimpleDesc;
  LCD_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &LCD_epDesc );
}

/*********************************************************************
 * @fn      LCD_ProcessEvent
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LCD_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {

        case AF_DATA_CONFIRM_CMD:
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus == ZSuccess )
          {
			// here when the ack send, just send the message to the lcd
			// send one message
	  		LCD_SendOneMessage();
		  }
          break;

        case AF_INCOMING_MSG_CMD:
          LCD_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          LCD_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (LCD_NwkState == DEV_ZB_COORD)
              || (LCD_NwkState == DEV_ROUTER)
              || (LCD_NwkState == DEV_END_DEVICE) )
          {
			  // initialize the  LCD 
			  LCD_Loading();
			  // send topology information
			  LCD_SendTopoInformation();
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LCD_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      LCD_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void LCD_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
	  // record the params
	  byte* gbk = pkt->cmd.Data;
	  byte  len = pkt->cmd.DataLength;
	  // According to the clusterID
	  switch( pkt->clusterId )
	  {
		// write subject information
		case LCD_SUBJECT_CMD:
		    // set the bit 1
		  	ackFlag += 2;
			LCD_BuildScreenFrame(gbk, len, courseReg);
		break;
		
		// write class information
		case LCD_CLASS_CMD:
		  	// this is the index of 0, so set the bit 0 to 1
		    ackFlag += 1;
			// send the data to the LCD
			LCD_BuildScreenFrame(gbk, len, classReg);
		break;
		
		// write teacher information
		case LCD_TEACHER_CMD:
		    // set the bit 2
		    ackFlag += 4;
			LCD_BuildScreenFrame(gbk, len, teacherReg);  
		break;
		
		// write people number information
		case LCD_PEOPLE_CMD:
		    // set the bit 3
		    ackFlag += 8;
			LCD_BuildScreenFrame(gbk, len, peopleReg);
		break;
		
		// write time duration infor
		case LCD_TIME_CMD:
		  	// set the bit 4
		  	ackFlag += 16;
			ackFlag |= 0x20;
			LCD_BuildScreenFrame(gbk, len, timeReg);
			// send the ack here
	  		LCD_SendACK(ackFlag);
			// send one message
	  		//LCD_SendOneMessage();
			// reset to zero
			ackFlag = 0;
		break;
		
		// here write the room number
		case ROOM_CMD:
			// here shift to the plate
		  	if(0 == shiftToRoomInformation){
			  	// shift to the room
				LCD_RoomPlate();
				// send retry
				LCD_SendRetry();
				// set the room state
				shiftToRoomInformation = 1;
		  	}else{
				// set the number
				LCD_BuildScreenFrame(gbk, len, roomReg);
				// send the ack here
	  			LCD_SendACK(0x3F);
				// send one message
	  			//LCD_SendOneMessage();
				ackFlag = 0;
			}
		break;
		
		// send the topology information
		case TOPO_REQ:
			// send the topology 
			LCD_SendTopoInformation();
		break;
	  }// end switch
}

/*******************************************************
	build screen uart frame
*******************************************************/
static void LCD_BuildScreenFrame(uint8* gbk, uint8 len, uint16 reg){
  // new byte array
  byte* frame = (byte*)osal_mem_alloc(len+6);
  // set uart header
  frame[0] = 0xE5;
  frame[1] = 0xE5;
  // set len
  frame[2] = 3 + len;
  // set wirte reg command
  frame[3] = 0x82;
  // set reg address
  frame[4] = HI_UINT16(reg);
  frame[5] = LO_UINT16(reg);
  // copy data
  osal_memcpy(&frame[6], gbk, len);
  // add to the message array
  LCD_PushMessage(frame, len+6);
  // free 
  osal_mem_free((byte*)frame);
}

/*******************************************************
		show the loading screen
*******************************************************/
static void LCD_Loading(){
  // here just shift the screen to the 1
  byte shift[] = {0xE5, 0xE5, 0x04, 0x80, 0x03, 0x00, 0x01};
  // write this to the usart
  HalUARTWrite(UART_PORT, shift, 7);
}

/******************************************************
		show the room
******************************************************/
static void LCD_RoomPlate(){
  // shift to the six screen
  byte shift[] = {0xE5, 0xE5, 0x04, 0x80, 0x03, 0x00, 0x06};
  // write to the uart
  HalUARTWrite(UART_PORT, shift, 7);
}

/*********************************************************************
 *
 * @fn		LCD_UartInit
 *
 * @brief	this method init the usart for the coordinator
 *
 *
 * @param   None
 *
 * @return  void
 */
static void LCD_UartInit()
{
  // create the uart structure
  halUARTCfg_t uartConfig;
  // set the parameters
  uartConfig.configured           = TRUE;               
  uartConfig.baudRate             = HAL_UART_BR_115200;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;   				
  uartConfig.rx.maxBufSize        = 128;  				
  uartConfig.tx.maxBufSize        = 128;  				
  uartConfig.idleTimeout          = 6;    				
  uartConfig.intEnable            = TRUE;               
  uartConfig.callBackFunc         = LCD_UartCallBack;
  // open it
  HalUARTOpen (UART_PORT, &uartConfig);
}

/*********************************************************************
 *
 * @fn		LCD_SendTopoInfo
 *
 *
 *
 */
static void LCD_SendTopoInformation()
{ 
  // Define the Frame
  unsigned char* srcExtAddr;
  srcExtAddr = NLME_GetExtAddr();
  uint16 srcAddr = NLME_GetShortAddr();
  uint16 parAddr = NLME_GetCoordShortAddr();
  byte frame[19] = {0};
  
  // Build the Frame
  // Fill SOF	0xFE
  frame[0] = 0xFE;
  // Fill len
  frame[1] = 12;
  // Fill CMD
  frame[2] = LO_UINT16(TOPOLOGY_CMD);
  frame[3] = HI_UINT16(TOPOLOGY_CMD);
  // Fill Addr
  frame[4] = LO_UINT16(srcAddr);
  frame[5] = HI_UINT16(srcAddr);
  // fill type
  frame[6] = LO_UINT16(LCD);
  frame[7] = HI_UINT16(LCD);
  // Fill Parent
  frame[8] = LO_UINT16(parAddr);
  frame[9] = HI_UINT16(parAddr);
  // fill the ieee64 address
  copyExtAddr(srcExtAddr, (byte*)&frame[10]); 
  // Cal and fill FCS
  frame[18] = LCD_CalcFCS((byte*)&frame[1], 17);
  
  // Send the data to Coordinator
  AF_DataRequest( &LCD_DstAddr, 
				  &LCD_epDesc,
				  ZIGBEE_COMMON_CLUSTER,
				  19,
				  (byte *)frame,
				  &LCD_TransID,
				  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/*********************************************************************
 *
 * @fn		LCD_UartCallBack
 *
 * @brief	this method deal with the uart event
 *
 *
 * @param   port  -- uart port
 *			event -- event occur
 *
 * @return  void
 */
static void LCD_UartCallBack( uint8 port, uint8 event )
{
  (void)port;

  // if Tx interrupt has occur
  if (event & HAL_UART_TX_EMPTY)
  {
	// hava message in the queue
	if(LCD_IsMessageEmpty() == 0x00){
	  // send another message
	  LCD_SendOneMessage();
	}
  }
}
/**************************************************************
		send the ack command to the monitor 
**************************************************************/
static void LCD_SendACK(byte flag){
  // new the frame
  byte frame[8] = {0};
  // fill the header
  frame[0] = 0xFE;
  // fill the length
  frame[1] = 1;
  // fill the command
  frame[2] = LO_UINT16(LCD_ACK_CMD);
  frame[3] = HI_UINT16(LCD_ACK_CMD);
  // fill Addr
  frame[4] = LO_UINT16(0);
  frame[5] = HI_UINT16(0);
  // fill the flag
  frame[6] = flag;
  // fill the fcs
  frame[7] = LCD_CalcFCS((byte*)&frame[1], 6);
  
  // Send the data to Coordinator
  AF_DataRequest( &LCD_DstAddr, 
				  &LCD_epDesc,
				  ZIGBEE_COMMON_CLUSTER,
				  8,
				  (byte *)frame,
				  &LCD_TransID,
				  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/***********************************************************
	LCD send retry
***********************************************************/
static void LCD_SendRetry(){
  // first get the short address
  uint16 srcAddr = NLME_GetShortAddr();
  // new the frame
  byte frame[7] = {0};
  // fill the header
  frame[0] = 0xFE;
  // fill the length
  frame[1] = 0;
  // fill the command
  frame[2] = LO_UINT16(LCD_RETRY_CMD);
  frame[3] = HI_UINT16(LCD_RETRY_CMD);
  // fill Addr
  frame[4] = LO_UINT16(srcAddr);
  frame[5] = HI_UINT16(srcAddr);
  // fill the fcs
  frame[6] = LCD_CalcFCS((byte*)&frame[1], 5);
  
  // Send the data to Coordinator
  AF_DataRequest( &LCD_DstAddr, 
				  &LCD_epDesc,
				  ZIGBEE_COMMON_CLUSTER,
				  7,
				  (byte *)frame,
				  &LCD_TransID,
				  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/*********************************************************************
 * @fn      LCD_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ********************************************************************/
static byte LCD_CalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult = 0x00;

  for ( x=0; x<len; x++ )
    xorResult ^=  msg_ptr[x];

  return ( xorResult );
}

/**************************************
	save extern address in array
**************************************/
static void copyExtAddr(byte* src, byte* dst){
	// here LSB goes first
  	int i = 0;
	while(i < 8){
	  dst[i] = src[i];
	  // update
	  i++;
	}
}

/******************************************************
		Define The Queue
******************************************************/
static void LCD_InitMessageQueue( ){
  // only set the pHeader and the pTail
  pHeader = 0;
  pTail = 0;
}

static void LCD_PushMessage( byte* str, byte len ){
  // check first
  if( pTail > 11 ){
	return;
  }
  // copy the data to the pTail
  osal_memcpy(messageArray[pTail].data, (byte*)str, len);
  messageArray[pTail].len = len;
  // then increase the tail
  pTail++; 
}

static MessageNode* LCD_PopMessage( ){
  // if the message array is null just return 
  if(0 == pTail){
	return NULL;
  }
  // counter
  byte i = 0;
  // get the header
  MessageNode* returnNode = osal_mem_alloc(sizeof(MessageNode));
  // get node
  osal_memcpy(returnNode->data, messageArray[pHeader].data, messageArray[pHeader].len);
  returnNode->len = messageArray[pHeader].len;
  // shift data forward
  for(i=0; i<=pTail-1; i++){
	osal_memcpy(messageArray[i].data, messageArray[i+1].data, messageArray[i+1].len);
	// set the len
	messageArray[i].len = messageArray[i+1].len;
  }
  // and now pTail is refered to a null node
  // just use to save new data
  pTail--;
  pHeader=0;
  
  // return
  return returnNode;
}

static byte LCD_IsMessageEmpty( ){
  if(pHeader == pTail){
	return 0x01;
  }else{
	return 0x00;
  }
}

static void LCD_SendOneMessage( ){
  // just return
  if(LCD_IsMessageEmpty() == 0x01){
	return;
  }	
  // pop a message from the message array
  MessageNode* item = LCD_PopMessage();
  // send
  HalUARTWrite(UART_PORT, item->data, item->len);
  // free it after send
  osal_mem_free((MessageNode*)item);
}
/*******************************************************************************
*******************************************************************************/