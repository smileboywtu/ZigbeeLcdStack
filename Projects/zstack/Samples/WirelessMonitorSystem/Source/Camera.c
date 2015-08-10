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
// Cluster number for Camera
#define CAMERA_MAX_IN_CLUSTERS 	2
#define CAMERA_MAX_OUT_CLUSTERS	3

// Camera Read Step
// you don't need to set the camera
#define	CAMERA_RESET		1

#define CAMERA_CLEAR		2
#define CAMERA_SHUT			3
#define CAMERA_RECEIVE_LEN	4
#define CAMERA_SEND_DATA	5

// EVENT LIST
#define CAMERA_IMAGE_READ_EVT			0x0002
#define CAMERA_INIT_EVT					0x0004


// Image Send Delay
// the coordinator needs 11 ms to send the data to the monitor
#define CAMERA_SEND_DELAY    1

/*********************************************************************
 * CONSTANTS
 */
const byte reset[] = {0x56, 0x00, 0x26, 0x00};
const byte reset_ack[] = {0x76, 0x00, 0x26, 0x00};

const byte clear[] = {0x56, 0x00, 0x36, 0x01, 0x02};
const byte clear_ack[] = {0x76, 0x00, 0x36, 0x00, 0x00};

const byte shut[] = {0x56, 0x00, 0x36, 0x01, 0x00};
const byte shut_ack[] = {0x76, 0x00, 0x36, 0x00, 0x00};

const byte length[] = {0x56, 0x00, 0x34, 0x01, 0x00};
const byte length_ack[] = {0x76, 0x00, 0x34, 0x00, 0x04, 0x00, 0x00};

const byte data_pre[] = {0x56, 0x00, 0x32, 0x0C, 0x00, 0x0A};
const byte data_end[] = {0x00, 0x00}; 

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t Camera_ClusterInList[CAMERA_MAX_IN_CLUSTERS] =
{
	TOPO_REQ,
	IMAGE_REQ
};

const cId_t Camera_ClusterOutList[CAMERA_MAX_OUT_CLUSTERS] =
{
  ZIGBEE_COMMON_CLUSTER,
  CAMERA_START_CMD,
  CAMERA_DATA_CMD
};

const SimpleDescriptionFormat_t Camera_SimpleDesc =
{
  Monitor_ENDPOINT,              //  int Endpoint;
  Monitor_PROFID,                //  uint16 AppProfId[2];
  Monitor_DEVICEID,              //  uint16 AppDeviceId[2];
  Monitor_DEVICE_VERSION,        //  int   AppDevVer:4;
  Monitor_FLAGS,                 //  int   AppFlags:4;
  CAMERA_MAX_IN_CLUSTERS,           //  byte  AppNumInClusters;
  (cId_t *)Camera_ClusterInList,    //  byte *pAppInClusterList;
  CAMERA_MAX_OUT_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)Camera_ClusterOutList    //  byte *pAppOutClusterList;
};

endPointDesc_t Camera_epDesc;

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
byte Camera_TaskID;    // Task ID for internal task/event processing

byte Camera_TransID;  // This is the unique message ID (counter)

devStates_t Camera_NwkState;

afAddrType_t Camera_DstAddr;	// Bind endpoint address

/************************** Camera *********************************/
byte cameraState;	

byte maxTransmitDataLen;

uint16	imageLength;
uint16 	currentRead;
uint16  currentAddr;

// Get the Max output
afDataReqMTU_t mtu;

/*************************** AF *******************************/
// address for the current node
uint16 srcAddr;
uint16 parAddr;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Camera_UartInit( void );
static void Camera_UartCallBack( uint8 port, uint8 event );
static void Camera_SendImageData( byte* buffer, byte len );
static void Camera_SendTopoInformation( void );
static void Camera_SendImageStartFrame( void );
static byte Camera_CalcFCS( uint8 *msg_ptr, uint8 len );
static void copyExtAddr(byte* src, byte* dst);
static void Camera_MessageMSGCB( afIncomingMSGPacket_t *pckt );

static void Camera_SendResetCMD( void );
static void Camera_SendClearCMD( void );
static void Camera_SendShutCMD( void );
static void Camera_SendLengthCMD( void );
static void Camera_SendDataCMD(uint16 startAddr, uint16 len);

static void Camera_SendCommand( void );
static void Camera_UartInterface( void );
static uint16 Camera_ToolTimesOfEight(uint16 addr);

/*********************************************************************
 * @fn      Camera_Init
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
  Camera_TaskID = task_id;
  Camera_NwkState = DEV_INIT;
  Camera_TransID = 0;
  
  // init the state and len
  cameraState = CAMERA_RESET;	// reset mode
  imageLength = 0;
  currentRead = 0;
  currentAddr = 0;

  // init the camera uart
  Camera_UartInit();
  
  // init the destination address
  Camera_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  Camera_DstAddr.endPoint = Monitor_ENDPOINT;
  Camera_DstAddr.addr.shortAddr = 0x0000;		// coordinator
	
  // Fill out the endpoint description.
  Camera_epDesc.endPoint = Monitor_ENDPOINT;
  Camera_epDesc.task_id = &Camera_TaskID;
  Camera_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Camera_SimpleDesc;
  Camera_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Camera_epDesc );

  // Set the size
  mtu.kvp        = FALSE;
  mtu.aps.secure = FALSE;
  maxTransmitDataLen = afDataReqMTU( &mtu );
}

/*********************************************************************
 * @fn      Camera_ProcessEvent
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Camera_TaskID );
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
		  }
			 
		  // delay just a little time as a during
		  // here just wait 20 ms for test
//		  if( cameraState == CAMERA_RESET )
//		  {
//			  // here you just need to reset the camera
//			  // and the incoming camera data request will
//			  // send the clear command the continious cmd
//			// delay enough time for the camera to initialized
//			int counter = 2500;
//			while(counter--);
//			Camera_SendResetCMD();
//		  }
		  if( cameraState ==  CAMERA_SEND_DATA )	
		  {
			// this will first happen when the 
			// start frame send out
			// we just know that the start frame send out successfully 
			// but we can't receive the data frame
			osal_start_timerEx(Camera_TaskID, 
							   CAMERA_IMAGE_READ_EVT,
							   CAMERA_SEND_DELAY);
		  }
          break;

        case AF_INCOMING_MSG_CMD:
          Camera_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          Camera_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (Camera_NwkState == DEV_ZB_COORD)
              || (Camera_NwkState == DEV_ROUTER)
              || (Camera_NwkState == DEV_END_DEVICE) )
          {
			  // get address
			  srcAddr = NLME_GetShortAddr();
			  parAddr = NLME_GetCoordShortAddr();
			  
			  //send topology information
			  // here we can wait the camera initialized then
			  // send topology to the monitor
			  //Camera_SendTopoInformation();  
			  osal_start_timerEx(Camera_TaskID,
								 CAMERA_INIT_EVT,
								 2500);
			  
			  // here you just need to reset the camera
			  // and the incoming camera data request will
			  // send the clear command the continious cmd
//			  Camera_SendResetCMD();
          }
          break;

        default:
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Camera_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // initialize the camera
  if ( events & CAMERA_INIT_EVT )
  {
	// init the camera 
	// send the reset command here
	Camera_SendResetCMD();
	// so will send the topology information at the 
	// ack for reset
	
	return (events ^ CAMERA_INIT_EVT);
  }
  
   // transfer the camera data
  if ( events & CAMERA_IMAGE_READ_EVT )
  {
	// you should keep care of this , that's the 
	// the last time the data request for the camera
	// maybe is more than needed, but this don't event make the 
	// the situation worse, it will always work well
	// cause will check the length in the AF send 
	// or we can chack the length in the ack
	if(currentRead < imageLength){
	  // get the mtu and sent the request
	  maxTransmitDataLen = afDataReqMTU( &mtu );
	  // here just request the data from the camera
	  // here you must read the data with a address which
	  // is the times of 8
	  // so just make
	  Camera_SendDataCMD(currentAddr, Camera_ToolTimesOfEight(maxTransmitDataLen));
	  // you  should update the data read and current addr in the 
	  // data ack 
	}
	else
	{
	  // data send ok
	  // reset all state 
//	  cameraState = CAMERA_RESET;
	  cameraState = CAMERA_CLEAR;
	  currentRead = 0;
	  currentAddr = 0;
	  imageLength = 0;
	}
    // return unprocessed events
    return (events ^ CAMERA_IMAGE_READ_EVT);
  }
  
  // Discard unknown events
  return 0;
}

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
static void Camera_MessageMSGCB( afIncomingMSGPacket_t *pckt )
{
  // According to the clusterID
  switch( pckt->clusterId )
  {
	// send the topology information
  	case TOPO_REQ:
      Camera_SendTopoInformation();
    break;
	// send image data
  	case IMAGE_REQ:
	  // set the camera pointer to clear
	  // cause the the camera is reset when the node start
	  cameraState = CAMERA_CLEAR;
	  currentRead = 0;
	  currentAddr = 0;
	  imageLength = 0;
	  // here you should send the clear command to the camera
	  Camera_SendClearCMD();
	break;
  }
}

/*********************************************************************
 *
 *@fn Camera_SendStartFrame
 *
 */
void Camera_SendImageStartFrame()
{
  // Define the Frame
  byte frame[9] = {0};
  
  // Build the Frame
  // Fill SOF	0xFE
  frame[0] = 0xFE;
  // Fill len
  frame[1] = 2;
  // Fill CMD
  frame[2] = LO_UINT16(CAMERA_START_CMD);
  frame[3] = HI_UINT16(CAMERA_START_CMD);
  // Fill Addr
  frame[4] = LO_UINT16(srcAddr);
  frame[5] = HI_UINT16(srcAddr);
  // fill the length
  frame[6] = LO_UINT16(imageLength);
  frame[7] = HI_UINT16(imageLength);
  // Cal and fill FCS
  frame[8] = Camera_CalcFCS((byte*)&frame[1], 7);
  
  // Send the data to Coordinator
  AF_DataRequest( &Camera_DstAddr, 
				  &Camera_epDesc,
				  ZIGBEE_COMMON_CLUSTER,
				  9,
				  (byte *)frame,
				  &Camera_TransID,
				  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/*********************************************************************
 *
 *@fn 	Camera_SendImageData
 *
 *@brief	A very Good way to send the data out
 *
 *@param		None
 *
 *@return	None
 */
void Camera_SendImageData( byte* buffer, byte len )
{  
  // alloc memory
  byte* imageBuffer = (byte*)osal_mem_alloc(len+7);
  osal_memset(imageBuffer, 0x00, len+7);
  
  /*
   * |	SOF 1| LEN  1| CMD 2| Addr 2| Data | FCS 1|
   *  Max length is maxTransmitDataLen
   */
  // Add SOF 
  imageBuffer[0] = 0xFE;
  // Add DataLen containing len+1
  imageBuffer[1] = len;
  // Fill cmd
  imageBuffer[2] = LO_UINT16(CAMERA_DATA_CMD);
  imageBuffer[3] = HI_UINT16(CAMERA_DATA_CMD);
  // Fill Addr
  imageBuffer[4] = LO_UINT16(srcAddr);
  imageBuffer[5] = HI_UINT16(srcAddr);
  // Add the data
  osal_memcpy(&imageBuffer[6], buffer, len);
  // Fill fcs
  imageBuffer[len+6] = Camera_CalcFCS(&imageBuffer[1], len+5);

  // send the data
  AF_DataRequest( &Camera_DstAddr, 
				&Camera_epDesc,
				ZIGBEE_COMMON_CLUSTER,
				len+7,
				(byte *)imageBuffer,
				&Camera_TransID,
				AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
 
  // free the data
  osal_mem_free((byte*)imageBuffer);
  imageBuffer = NULL;
}

/*********************************************************************
 *
 * @fn		Camera_SendTopoInfo
 *
 *
 *
 */
static void Camera_SendTopoInformation()
{
  
  unsigned char* srcExtAddr;
  srcExtAddr = NLME_GetExtAddr();
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
  frame[6] = LO_UINT16(CAMERA);
  frame[7] = HI_UINT16(CAMERA);
  // Fill Parent
  frame[8] = LO_UINT16(parAddr);
  frame[9] = HI_UINT16(parAddr);
  // fill the ieee64 address
  copyExtAddr(srcExtAddr, (byte*)&frame[10]); 
  // Cal and fill FCS
  frame[18] = Camera_CalcFCS((byte*)&frame[1], 17);
  
  // Send the data to Coordinator
  AF_DataRequest( &Camera_DstAddr, 
				  &Camera_epDesc,
				  ZIGBEE_COMMON_CLUSTER,
				  19,
				  (byte *)frame,
				  &Camera_TransID,
				  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

/*********************************************************************
 * @fn      Camera_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ********************************************************************/
byte Camera_CalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult = 0x00;

  for ( x=0; x<len; x++ )
    xorResult ^=  msg_ptr[x];

  return ( xorResult );
}

/*********************************************************************
 *
 * @fn		Camera_UartInit
 *
 * @brief	this method init the usart for the coordinator
 *
 *
 * @param   None
 *
 * @return  void
 */
static void Camera_UartInit()
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
  uartConfig.callBackFunc         = Camera_UartCallBack;
  // open it
  HalUARTOpen (UART_PORT, &uartConfig);
}

/*********************************************************************
 *
 * @fn		Camera_UartCallBack
 *
 * @brief	this method deal with the uart event
 *
 *
 * @param   port  -- uart port
 *			event -- event occur
 *
 * @return  void
 */
static void Camera_UartCallBack( uint8 port, uint8 event )
{
  (void)port;

  // if Rx interrupt has occur
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
	Camera_UartInterface();
  }
}

// Camera_Interface
static void Camera_UartInterface()
{
  byte* ack = NULL;
  byte* buffer = NULL;
  byte sendLength = 0;
  byte bytesInRxBuffer = 0;
  
  if( cameraState < CAMERA_RECEIVE_LEN )	// ignore the ack
  {
		// read all data out and send new command
		bytesInRxBuffer = Hal_UART_RxBufLen(UART_PORT);
		// memory data and read all data out
		ack = osal_mem_alloc(bytesInRxBuffer);
		// this data will be destroyed at the end of the method
		HalUARTRead(UART_PORT, (byte*)ack, bytesInRxBuffer);
		// send next command
		// keep care that if the this is the reset ack
		// you should never send next command again
		if(CAMERA_RESET != cameraState)
		{
			Camera_SendCommand();
			// update the current state
			cameraState++;
		}
		else	// this is the ack for reset command
		{
		  // here send the topology information to the monitor
		  Camera_SendTopoInformation();
		}
  }
  else
  {
	  if(CAMERA_RECEIVE_LEN == cameraState)
	  {
		  // here you will deal with the length ack with length in it
		  // here you need to read enough data out and get the length
		  bytesInRxBuffer = Hal_UART_RxBufLen(UART_PORT);
		  if( 9 == bytesInRxBuffer )
		  {
			  ack = osal_mem_alloc(9);
			  // the last two bytes are the length
			  HalUARTRead(UART_PORT, ack, 9);
			  // get the image len
			  imageLength = BUILD_UINT16(ack[8], ack[7]);
			  // here you need to send the starter command to 
			  // the monitor with the data length
			  Camera_SendImageStartFrame();
			  // update the state
			  cameraState++;
		  }
		  else
		  {
			// from the start and read again
			cameraState = CAMERA_CLEAR;
			// send clear command
			Camera_SendClearCMD();
		  }
	  }
	  else if( CAMERA_SEND_DATA == cameraState )
	  {
		  // ignore two bytes first and send the real data to the 
		  // pack and AF send mathod
		  // read all data out and send new command
		  bytesInRxBuffer = Hal_UART_RxBufLen(UART_PORT);
		  // memory data and read all data out
		  ack = osal_mem_alloc(bytesInRxBuffer);
		  buffer = osal_mem_alloc(bytesInRxBuffer-10);
		  HalUARTRead(UART_PORT, ack, bytesInRxBuffer);
		  // ignore first and last 7 bytes
		  if((bytesInRxBuffer-10) > (imageLength - currentRead))
		  {
			sendLength = (imageLength - currentRead);
		  }
		  else
		  {
			sendLength = bytesInRxBuffer-10;
		  }
		  //  pack the data and send
		  osal_memcpy(buffer, &ack[5], sendLength);
		  Camera_SendImageData(buffer, sendLength);
		  
		  // update the addr and read counter
		  currentRead += sendLength;
		  currentAddr += sendLength;
		  
		  // free the data
		  osal_mem_free((byte*)buffer);
  		  // reset
  		  buffer = NULL;
		  sendLength = 0; 
		  
		  // here you do not to update the state again
		  // cause the state then will change by the app not the ack
		  // this will actually happen in the image_read_transfer event
		  // it will be reset when the user read all imageLength data
		  // and wait the user send camera command again
		  // this will make the zigbee enddevice send clear cmd agian
		  // with ack update the state to receive length state again  
	  } // end else if CAMERA_SEND_DATA
	}// end else CAMERA_RECEIVE_LEN
  
  // just dispose the data
  osal_mem_free((byte*)ack);
  // reset
  ack = NULL;
  bytesInRxBuffer = 0;
}

// send the command
static void Camera_SendCommand()
{
  switch(cameraState)
  {
  case CAMERA_CLEAR:
	// this may be happen when the user request the camera data
	Camera_SendShutCMD();
	break;
  case CAMERA_SHUT:
	// this is after the the user receive the shut ack
	Camera_SendLengthCMD();
	break;
  // do not need more conditions 
  // cause the length cmd will return a user length for user to send start
  // frame
  // the data ack will carry the data of the camera, then send to the monitor
  }
}

// deal with the ack and know what's the ack is
// sometimes you need not the ack command

static void Camera_SendResetCMD( void ){
  HalUARTWrite(UART_PORT, (byte*)reset, 4);
}

static void Camera_SendClearCMD( void ){
  HalUARTWrite(UART_PORT, (byte*)clear, 5);
}

static void Camera_SendShutCMD( void ){
  HalUARTWrite(UART_PORT, (byte*)shut, 5);
}

static void Camera_SendLengthCMD( void ){
  HalUARTWrite(UART_PORT, (byte*)length, 5);
}

static void Camera_SendDataCMD( uint16 startAddr, uint16 len ){
  byte* buffer = osal_mem_alloc(16);
  osal_memcpy(buffer, (byte*)data_pre, 6);
  buffer[6] = 0x00;
  buffer[7] = 0x00;
  buffer[8] = HI_UINT16(startAddr);
  buffer[9] = LO_UINT16(startAddr);
  buffer[10] = 0x00;
  buffer[11] = 0x00;
  buffer[12] = HI_UINT16(len);
  buffer[13] = LO_UINT16(len);
  osal_memcpy(&buffer[14], (byte*)data_end, 2);
  // send the command
  HalUARTWrite(UART_PORT, buffer, 16);
  // free the buffer
  osal_mem_free((byte*)buffer);
  buffer = NULL;
}

static uint16 Camera_ToolTimesOfEight(uint16 addr)
{
  // here you should just read less than the bits
  if((addr/8*8+7) > addr)
  {
	return ((addr/8-1)*8);
  }
  else
  {
	return (addr/8*8);
  }
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
/*******************************************************************************
*******************************************************************************/