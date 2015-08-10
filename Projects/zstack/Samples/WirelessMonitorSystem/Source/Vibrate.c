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
//#include "hal_i2c.h"
//#include "hal_adxl345.h"
//#include "ADXL345SPI.h"


/*********************************************************************
 * MACROS
 */
// Max Cluster ID
#define VIBRATE_MAX_IN_CLUSTERS 	3
#define VIBRATE_MAX_OUT_CLUSTERS	1

// Send Message Timeout
#define Vibrate_SEND_DELAY  	(RESPONSE_POLL_RATE * 2)

// Application Events (OSAL) - These are bit weighted definitions.
#define Vibrate_SEND_MSG_EVT    0x0001

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
const cId_t Vibrate_ClusterInList[VIBRATE_MAX_IN_CLUSTERS] =
{
	TOPO_REQ,
	DATA_ALLOW,
	DATA_REFUSE
};

const cId_t Vibrate_ClusterOutList[VIBRATE_MAX_OUT_CLUSTERS] =
{
	ZIGBEE_COMMON_CLUSTER
};

const SimpleDescriptionFormat_t Vibrate_SimpleDesc =
{
  Monitor_ENDPOINT,              //  int Endpoint;
  Monitor_PROFID,                //  uint16 AppProfId[2];
  Monitor_DEVICEID,              //  uint16 AppDeviceId[2];
  Monitor_DEVICE_VERSION,        //  int   AppDevVer:4;
  Monitor_FLAGS,                 //  int   AppFlags:4;
  VIBRATE_MAX_IN_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)Vibrate_ClusterInList,   //  byte *pAppInClusterList;
  VIBRATE_MAX_OUT_CLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)Vibrate_ClusterOutList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in Vibrate_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t Vibrate_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte Vibrate_TaskID;   // Task ID for internal task/event processing

devStates_t Vibrate_NwkState;

byte Vibrate_TransID;  // This is the unique message ID (counter)

afAddrType_t Vibrate_DstAddr;	// the coordinator addr

uint16 srcAddr = 0;


/************************ User *********************************/
static byte allowSend = 1;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Vibrate_InitUart( void );
static void Vibrate_UartCallBack( uint8 port, uint8 event );
static void Vibrate_ProcessingUartData( void );
static void Vibrate_MessageMSGCB( afIncomingMSGPacket_t *pckt );
//static void Vibrate_InitSensor( void );
static void Vibrate_SendTopoInformation( void );
static void Vibrate_SendADXL345Data( byte* buffer );
static byte Vibrate_CalcFCS( uint8 *msg_ptr, uint8 len );

/*********************************************************************
 * @fn      Controller_Init
 *
 * @brief   Initialization function for the Generic App Task.
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
  Vibrate_TaskID = task_id;
  Vibrate_NwkState = DEV_INIT;
  Vibrate_TransID = 0;
  
  // allow send by default
  allowSend = 1;

  // Init Uart
  Vibrate_InitUart();
  
  // Init the ADXL345
//  Vibrate_InitSensor();
  
  // init the destination address
  Vibrate_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  Vibrate_DstAddr.endPoint = Monitor_ENDPOINT;
  Vibrate_DstAddr.addr.shortAddr = 0x0000;	

  // Fill out the endpoint description.
  Vibrate_epDesc.endPoint =Monitor_ENDPOINT;
  Vibrate_epDesc.task_id = &Vibrate_TaskID;
  Vibrate_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Vibrate_SimpleDesc;
  Vibrate_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Vibrate_epDesc );
}

/*********************************************************************
 * @fn      Controller_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
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
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Vibrate_TaskID );
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
          if ( sentStatus != ZSuccess )
          {
			
          }
		  
		  // delay a time
		  // this will make the device receive the allow or refuse command
		  osal_start_timerEx(Vibrate_TaskID, 
							 Vibrate_SEND_MSG_EVT, 
							 Vibrate_SEND_DELAY);
          break;
		  
		  case AF_INCOMING_MSG_CMD:
          	Vibrate_MessageMSGCB( MSGpkt );
          break;
		  
		  case ZDO_STATE_CHANGE:
          	Vibrate_NwkState = (devStates_t)(MSGpkt->hdr.status);
		    if ( (Vibrate_NwkState == DEV_ZB_COORD)
			  || (Vibrate_NwkState == DEV_ROUTER)
			  || (Vibrate_NwkState == DEV_END_DEVICE) )
			{
			    // record the destinator address
			  	srcAddr = NLME_GetShortAddr();
				// Start Topology
				Vibrate_SendTopoInformation();
				// Start the send task
//		  		osal_start_timerEx( Vibrate_TaskID,
//							 		Vibrate_SEND_MSG_EVT,
//									Vibrate_SEND_DELAY);
			}
			break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Vibrate_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in Vibrate_Init()).
  if ( events & Vibrate_SEND_MSG_EVT )
  {
    // Send "the" message
//    Vibrate_ReadADXL345();
	Vibrate_ProcessingUartData();
	
    // return unprocessed events
    return (events ^ Vibrate_SEND_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

// Processing Incoming Message
static void Vibrate_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  // According to the clusterID
  switch( pkt->clusterId )
  {
	// send the topology information
  	case TOPO_REQ:
      Vibrate_SendTopoInformation();
    break;
	
  	case DATA_ALLOW:
	  // sent the flag
	  allowSend = 1;
	  break;
	  
  case DATA_REFUSE:
	 // reset the flag
	allowSend = 0;
	break;
  }
}
/*********************************************************************
 * @fn      Vibrate_InitSensor
 *
 * @brief   Init the ADXL345.
 *
 * @param   none
 *
 * @return  none
 */
/*
void Vibrate_InitSensor()
{
  byte value = 0x00;
  
//  // set the SPI
//  Init_Spi();
//  // use 4 -wrie mode
//  // default is 4 wire so do nothing 
//  value = 0x00;
//  ADXL345Write(0, 1, &value, 0x31);
//  
//  // Just Open the ADXL345
//  value = 0;
//  ADXL345Write(0, 1, &value, 0x2d);
//  value = 16;
//  ADXL345Write(0, 1, &value, 0x2d);
//  value = 8;
//  ADXL345Write(0, 1, &value, 0x2d);
//  
//  // DO More Setting here
//  value = 75;
//  ADXL345Write(0, 1, &value, 0x24);
//  ADXL345Write(0, 1, &value, 0x25);
//  value = 10;
//  ADXL345Write(0, 1, &value, 0x26);
  
  ADXL345_PowerOn();
  
  ADXL345WriteByte(ADXL345_THRESH_ACT, 75);
  ADXL345WriteByte(ADXL345_THRESH_INACT, 75);
  ADXL345WriteByte(ADXL345_TIME_INACT, 10);
  
  value = ADXL345ReadByte(ADXL345_ACT_INACT_CTL);
  // set x, y, z movement
  value |= 0x77;
  ADXL345WriteByte(ADXL345_ACT_INACT_CTL, value);
  
  // set tap
  value = ADXL345ReadByte(ADXL345_TAP_AXES);
  value &= 0xF9;
  value |= 0x01;
  ADXL345WriteByte(ADXL345_TAP_AXES, value);
  
  // single double tap
  ADXL345WriteByte(ADXL345_THRESH_TAP, 50);
  ADXL345WriteByte(ADXL345_DUR, 15);
  ADXL345WriteByte(ADXL345_LATENT, 80);
  ADXL345WriteByte(ADXL345_WINDOW, 200);
  
  // set free fall 
  ADXL345WriteByte(ADXL345_THRESH_FF, 7);
  ADXL345WriteByte(ADXL345_TIME_FF, 45);
  
//  value = 0x0F;
//  ADXL345Write(0, 1, &value, 0x2c);
}
*/

/*********************************************************************
 * @fn      Vibrate_SendTopoInfo
 *
 * @brief   Send the topology information to the coordinator.
 *
 * @param   none
 *
 * @return  none
 *
 *
 *
 * |	SOF		|	LEN		| 	CMD		| 	Addr	|	DATA	| 	FCS		|
 *		1			1			2			2			--			1
 *
 */

void Vibrate_SendTopoInformation()
{
  // Define the Frame
  uint16 srcAddr = NLME_GetShortAddr();
  uint16 parAddr = NLME_GetCoordShortAddr();
  byte frame[11] = {0};
  
  // Build the Frame
  // Fill SOF	0xFE
  frame[0] = 0xFE;
  // Fill len
  frame[1] = 4;
  // Fill CMD
  frame[2] = LO_UINT16(TOPOLOGY_CMD);
  frame[3] = HI_UINT16(TOPOLOGY_CMD);
  // Fill Addr
  frame[4] = LO_UINT16(srcAddr);
  frame[5] = HI_UINT16(srcAddr);
  // Fill type
  frame[6] = LO_UINT16(VIBRATE);
  frame[7] = HI_UINT16(VIBRATE);
  // Fill parent
  frame[8] = LO_UINT16(parAddr);
  frame[9] = HI_UINT16(parAddr);
  // Cal and fill FCS
  frame[10] = Vibrate_CalcFCS((byte*)&frame[1], \
								9);
  
  // Send the data to Coordinator
  AF_DataRequest( &Vibrate_DstAddr, &
				  Vibrate_epDesc,
                  ZIGBEE_COMMON_CLUSTER,
                  11,
                  (byte *) frame,
                  &Vibrate_TransID,
                  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}

// read the uart data and send to the monitor
static void Vibrate_ProcessingUartData( void ){
  byte* buffer = (byte*)osal_mem_alloc(6);
  // if the Rx hava data
  while ( Hal_UART_RxBufLen(UART_PORT) )
  {
	// read 6 bytes to buffer
    HalUARTRead (UART_PORT, buffer, 6);
	// send the data 
	Vibrate_SendADXL345Data(buffer);
	// here just break 
	// wait the timer to wake this method again
	// or the uart callback to wake the method
	break;
  }
  // free the memory
  osal_mem_free((byte*)buffer);
}

/*********************************************************************
 * @fn      Vibrate_ReadADXL345
 *
 * @brief   read the adxl345 data
 *
 * @param   buf		-- where to save the data
 *
 * @return  none
 */
void Vibrate_SendADXL345Data(byte* buffer)
{
  // Read the ADXL345
  // Use I2C
//  byte* buffer = (byte*)osal_mem_alloc(6);
//  osal_memset(buffer, 0x00, 6);
//  // Read
//  ADXL345_readAccel(buffer);
  //ADXL345Read(0, 1, buffer, 0x2C);
  //ADXL345Read(1, 6, &buffer[0], 0x32);
  
  // Build Frame
  byte frame[13] = {0};
  
  // Fill the SOF
  frame[0] = 0xFE;
  // Fill len
  frame[1] = 6;
  // Fill CMD
  frame[2] = LO_UINT16(VIBRATE_DATA_CMD);
  frame[3] = HI_UINT16(VIBRATE_DATA_CMD);
  // Fill Addr
  frame[4] = LO_UINT16(srcAddr);
  frame[5] = HI_UINT16(srcAddr);
  // Fill Data
  frame[6] = buffer[0];
  frame[7] = buffer[1];
  frame[8] = buffer[2];
  frame[9] = buffer[3];
  frame[10] = buffer[4];
  frame[11] = buffer[5];
  // Calc FCS and Fill
  frame[12] = Vibrate_CalcFCS((byte*)&frame[1], \
								 11);
  
  if(1 == allowSend)
  {
	  // Send the sensor data to Coordinator
	  AF_DataRequest( &Vibrate_DstAddr, &
					  Vibrate_epDesc,
					  ZIGBEE_COMMON_CLUSTER,
					  13,
					  (byte *) frame,
					  &Vibrate_TransID,
					  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
  }
  else
  {
	// do nothing here
  }
  
  // reset the buffer
//  osal_mem_free((byte*)buffer);
//  buffer = NULL;
  
  // send next
//  osal_start_timerEx(Vibrate_TaskID,
//					 Vibrate_SEND_MSG_EVT,
//					 Vibrate_SEND_DELAY);
}

	
/*********************************************************************
 * @fn      Vibrate_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ********************************************************************/
byte Vibrate_CalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult = 0x00;

  for ( x=0; x<len; x++ )
    xorResult ^=  msg_ptr[x];

  return ( xorResult );
}

// uart init
static void Vibrate_InitUart( void )
{
    // create the uart structure
  halUARTCfg_t uartConfig;
  // set the parameters
  uartConfig.configured           = TRUE;               
  uartConfig.baudRate             = HAL_UART_BR_9600;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64;   				
  uartConfig.rx.maxBufSize        = 128;  				
  uartConfig.tx.maxBufSize        = 128;  				
  uartConfig.idleTimeout          = 6;    				
  uartConfig.intEnable            = TRUE;               
  uartConfig.callBackFunc         = Vibrate_UartCallBack;
  // open it
  HalUARTOpen (UART_PORT, &uartConfig);
}

// uart call back
static void Vibrate_UartCallBack( uint8 port, uint8 event )
{
  (void)port;

  // if Rx interrupt has occur
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)))
  {
	Vibrate_ProcessingUartData();
  }
}
/*********************************************************************
 */
