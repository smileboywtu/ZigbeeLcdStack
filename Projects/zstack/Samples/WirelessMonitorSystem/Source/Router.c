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

/*********************************************************************
 * MACROS
 */

// Max in / out Cluster number
#define Router_MAX_IN_CLUSTERS			1
#define Router_MAX_OUT_CLUSTERS			10

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t Router_ClusterInList[Router_MAX_IN_CLUSTERS] =
{
	ZIGBEE_COMMON_CLUSTER,
};

const cId_t Router_ClusterOutList[Router_MAX_OUT_CLUSTERS] =
{
	TOPO_REQ,
	IMAGE_REQ,
	LCD_SUBJECT_CMD,
	LCD_CLASS_CMD,
	LCD_TEACHER_CMD,
	LCD_PEOPLE_CMD,
	LCD_TIME_CMD,
	GRAPHICS_CMD,
	DATA_ALLOW,
	DATA_REFUSE
};

const SimpleDescriptionFormat_t Router_SimpleDesc =
{
  Monitor_ENDPOINT,              //  int Endpoint;
  Monitor_PROFID,                //  uint16 AppProfId[2];
  Monitor_DEVICEID,              //  uint16 AppDeviceId[2];
  Monitor_DEVICE_VERSION,        //  int   AppDevVer:4;
  Monitor_FLAGS,                 //  int   AppFlags:4;
  Router_MAX_IN_CLUSTERS,           //  byte  AppNumInClusters;
  (cId_t *)Router_ClusterInList,    //  byte *pAppInClusterList;
  Router_MAX_OUT_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)Router_ClusterOutList    //  byte *pAppOutClusterList;
};

endPointDesc_t Router_epDesc;

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
byte Router_TaskID;    // Task ID for internal task/event processing

byte Router_TransID;   // This is the unique message ID (counter)

devStates_t Router_NwkState;	// record the network state

afAddrType_t Router_DstAddr;	// Bind endpoint address


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Router_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void Router_SendTopologyInformation( void );
static byte Router_CalcFCS( uint8 *msg_ptr, uint8 len );

/*********************************************************************
 * @fn      Router_Init
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
  Router_TaskID = task_id;
  Router_NwkState = DEV_INIT;
  Router_TransID = 0;
  
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  Router_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  Router_DstAddr.endPoint = Monitor_ENDPOINT;
  Router_DstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;	// 0xFFFF

  // Fill out the endpoint description.
  Router_epDesc.endPoint = Monitor_ENDPOINT;
  Router_epDesc.task_id = &Router_TaskID;
  Router_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&Router_SimpleDesc;
  Router_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &Router_epDesc );
  
}

/*********************************************************************
 * @fn      Router_ProcessEvent
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Router_TaskID );
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
			// success do something here
		  } 
          break;

        case AF_INCOMING_MSG_CMD:
          Router_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          Router_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (Router_NwkState == DEV_ZB_COORD)
              || (Router_NwkState == DEV_ROUTER)
              || (Router_NwkState == DEV_END_DEVICE) )
          { 
			// send the topology information
			Router_SendTopologyInformation();
          }
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( Router_TaskID );
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
 * @fn      Router_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void Router_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  // According to the clusterID
  switch( pkt->clusterId )
  {
	// send the topology information
  	case TOPO_REQ:
      Router_SendTopologyInformation();
    break;
  }
}

/*********************************************************************
 * @fn      Router_SendTopoInfo
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

void Router_SendTopologyInformation()
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
  frame[6] = LO_UINT16(ROUTER);
  frame[7] = HI_UINT16(ROUTER);
  // Fill parent
  frame[8] = LO_UINT16(parAddr);
  frame[9] = HI_UINT16(parAddr);
  // Cal and fill FCS
  frame[10] = Router_CalcFCS((byte*)&frame[1], 9);
  
  // Send the data to Coordinator
  AF_DataRequest( &Router_DstAddr, &
				  Router_epDesc,
                  ZIGBEE_COMMON_CLUSTER,
                  11,
                  (byte *) frame,
                  &Router_TransID,
                  AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
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
byte Router_CalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult = 0x00;

  for ( x=0; x<len; x++ )
    xorResult ^=  msg_ptr[x];

  return ( xorResult );
}