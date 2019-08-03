/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "DebugTrace.h"
#if !defined(WIN32) || defined(ZBIT)
#include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "aps_groups.h"

/* RTOS */
#if defined(IAR_ARMCM3_LM)
#include "RTOS_App.h"
#endif

#include "Debug.h"
#include "GenericApp.h"
#include "PCApp.h"

endPointDesc_t GenericApp_epDesc;
afAddrType_t GenericApp_DstAddr;

devStates_t GenericApp_NwkState;
unsigned char device_count = 0;
byte GenericApp_TaskID;
byte GenericApp_TransID;
aps_Group_t SampleApp_Group;
EndDeviceInfo_t EndDeviceInfos[16];


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs(zdoIncomingMsg_t *inMsg);
static void GenericApp_MessageMSGCB(afIncomingMSGPacket_t *pckt);
/*********************************************************************
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 */
void GenericApp_Init(uint8 task_id)
{
    GenericApp_TaskID = task_id;
    GenericApp_NwkState = DEV_INIT;
    GenericApp_TransID = 0;
    
    // Fill out the endpoint description.
    GenericApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
    GenericApp_epDesc.task_id = &GenericApp_TaskID;
    GenericApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
    GenericApp_epDesc.latencyReq = noLatencyReqs;

    // Register the endpoint description with the AF
    afRegister(&GenericApp_epDesc);
    
      // By default, all devices start out in Group 1
    SampleApp_Group.ID = 0x0001;
    osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
    aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

    // Register for all key events - This app will handle all key events
    RegisterForKeys(GenericApp_TaskID);
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, NWK_addr_req );
    DebugInit();
    LREPMaster("Initialized debug module \n");
}

/*********************************************************************
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 */
uint16 GenericApp_ProcessEvent(uint8 task_id, uint16 events)
{
    afIncomingMSGPacket_t *MSGpkt;
    afDataConfirm_t *afDataConfirm;

    // Data Confirmation message fields
    byte sentEP;
    ZStatus_t sentStatus;
    byte sentTransID; // This should match the value sent
    (void)task_id;    // Intentionally unreferenced parameter

    if (events & SYS_EVENT_MSG)
    {
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(GenericApp_TaskID);
        while (MSGpkt)
        {
            switch (MSGpkt->hdr.event)
            {
                case ZDO_CB_MSG:
                {
                    GenericApp_ProcessZDOMsgs((zdoIncomingMsg_t *)MSGpkt);
                    break;
                }
                case AF_DATA_CONFIRM_CMD:
                {
                    // This message is received as a confirmation of a data packet sent.
                    // The status is of ZStatus_t type [defined in ZComDef.h]
                    // The message fields are defined in AF.h
                    afDataConfirm = (afDataConfirm_t *)MSGpkt;

                    sentEP = afDataConfirm->endpoint;
                    (void)sentEP; // This info not used now
                    sentTransID = afDataConfirm->transID;
                    (void)sentTransID; // This info not used now
                    sentStatus = afDataConfirm->hdr.status;
                    // Action taken when confirmation is received.
                    if (sentStatus != ZSuccess)
                    {
                        LREPMaster("AF_DATA_CONFIRM: Send lost! \n");
                    }
                    break;
                }
                case AF_INCOMING_MSG_CMD:
                {
                    GenericApp_MessageMSGCB(MSGpkt);
                    break;
                }
                case ZDO_STATE_CHANGE:
                {
                    GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                    LREP("GenericApp_NwkState 0x%x \n", GenericApp_NwkState);
                    if ((GenericApp_NwkState == DEV_ZB_COORD) ||
                        (GenericApp_NwkState == DEV_ROUTER) ||
                        (GenericApp_NwkState == DEV_END_DEVICE))
                    {
#if defined ( ROUTER ) || (ENDDEVICE)
                        // Start sending "the" message in a regular interval.
                        osal_start_timerEx(GenericApp_TaskID,
                                           GENERICAPP_SEND_MSG_EVT,
                                           GENERICAPP_SEND_MSG_TIMEOUT);
#endif
                    }
                    break;
                }
                default:
                    break;
            }

            // Release the memory
            osal_msg_deallocate((uint8 *)MSGpkt);

            // Next
            MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(GenericApp_TaskID);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    // Send a message out - This event is generated by a timer
    //  (setup in GenericApp_Init()).
    if (events & GENERICAPP_SEND_MSG_EVT)
    {
#if defined ( ROUTER )
        // // Send "the" message
        PCApp_SendTheMessage(0, 0x00, 1);
#else
        SampleApp_SendPeriodicMessage();
#endif         
        // // Setup to send message again
        osal_start_timerEx(GenericApp_TaskID,
                           GENERICAPP_SEND_MSG_EVT,
                           GENERICAPP_SEND_MSG_TIMEOUT);

        // return unprocessed events
        return (events ^ GENERICAPP_SEND_MSG_EVT);
    }
    if(events & GENERICAPP_UART_RX_EVT)
    {
        PCApp_SerialMSGCB();
    }
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
static void GenericApp_ProcessZDOMsgs(zdoIncomingMsg_t *inMsg)
{
    
    switch (inMsg->clusterID)
    {
        case Match_Desc_rsp:
        {
            ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp(inMsg);
            if (pRsp)
            {
                if (pRsp->status == ZSuccess && pRsp->cnt)
                {
                    GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
                    GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
                    // Take the first endpoint, Can be changed to search through endpoints
                    GenericApp_DstAddr.endPoint = pRsp->epList[0];
                    // Light LED
                    //HalLedSet(HAL_LED_4, HAL_LED_MODE_ON);
                    LREP("Light LED on \n");
                }
                osal_mem_free(pRsp);
            }
        }
        break;
    }
}

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 */
static void GenericApp_MessageMSGCB(afIncomingMSGPacket_t *pkt)
{
  unsigned char temp_extAddr[8];
  unsigned char i = 0;
  unsigned char existed = 0;
  unsigned char buffer[8] = "";
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      {// "the" message
        (void)APSME_LookupExtAddr(pkt->srcAddr.addr.shortAddr, temp_extAddr);
        for (i = 0; i < device_count+1; i++) {  // to see whether the device ever connected
          if (osal_memcmp(EndDeviceInfos[i].extAddr, temp_extAddr, 8)) { //if existed, break;
            existed = 1;
            //HalUARTWrite(0, "Existed!\r\n",12);
            break;
          }
        }
        if (!existed) {  //not connected before, register
          //HalUARTWrite(0, "New Device!\r\n",15);
          osal_memcpy(EndDeviceInfos[device_count].extAddr, temp_extAddr, 8);
          EndDeviceInfos[device_count].endPoint = pkt->srcAddr.endPoint;
          EndDeviceInfos[device_count].compressed_addr = GENERICAPP_ENDPOINT | EndDeviceInfos[device_count].endPoint;
          device_count++;
        }
        if (device_count == 16) { device_count = 0;}  //in case to overflow
        //TO DO: process data received, send it to slave machine
        osal_memcpy(&buffer[0], pkt->cmd.Data, 8);
        reserve_string((char*)&buffer[3], 1);
        HalUARTWrite(0, &buffer[0], 8);
        break;
      }
  }
}
