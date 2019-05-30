/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined(WIN32) || defined(ZBIT)
#include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* RTOS */
#if defined(IAR_ARMCM3_LM)
#include "RTOS_App.h"
#endif

#include "CompileOption.h"
#include "Debug.h"
#include "Config.h"
#include "SH_App.h"
/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
    {
        GENERICAPP_CLUSTERID};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
    {
        GENERICAPP_ENDPOINT,             //  int Endpoint;
        GENERICAPP_PROFID,               //  uint16 AppProfId[2];
        GENERICAPP_DEVICEID,             //  uint16 AppDeviceId[2];
        GENERICAPP_DEVICE_VERSION,       //  int   AppDevVer:4;
        GENERICAPP_FLAGS,                //  int   AppFlags:4;
        GENERICAPP_MAX_CLUSTERS,         //  byte  AppNumInClusters;
        (cId_t *)GenericApp_ClusterList, //  byte *pAppInClusterList;
        GENERICAPP_MAX_CLUSTERS,         //  byte  AppNumInClusters;
        (cId_t *)GenericApp_ClusterList  //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID; // Task ID for internal task/event processing
                        // This variable will be received when
                        // GenericApp_Init() is called.

devStates_t GenericApp_NwkState;
static int rxMsgCount = 0;
byte GenericApp_TransID; // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
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
    // Device hardware initialization can be added here or in main() (Zmain.c).
    // If the hardware is application specific - add it here.
    // If the hardware is other parts of the device add it in main().
    GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
    GenericApp_DstAddr.endPoint = 0;
    GenericApp_DstAddr.addr.shortAddr = 0;

    // Fill out the endpoint description.
    GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
    GenericApp_epDesc.task_id = &GenericApp_TaskID;
    GenericApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
    GenericApp_epDesc.latencyReq = noLatencyReqs;

    // Register the endpoint description with the AF
    afRegister(&GenericApp_epDesc);

    // Register for all key events - This app will handle all key events
    RegisterForKeys(GenericApp_TaskID);
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
    // ZDO_RegisterForZDOMsg( GenericApp_TaskID, NWK_addr_req );
    DebugInit();
    LREPMaster("Initialize UART module successfully \n");
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
            //LREP("Event: 0x%x \n", MSGpkt->hdr.event);
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
                        LREPMaster("AF_DATA_CONFIRM: Send error! \n");
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
                        // Start sending "the" message in a regular interval.
                        // osal_start_timerEx(GenericApp_TaskID,
                        //                    GENERICAPP_SEND_MSG_EVT,
                        //                    GENERICAPP_SEND_MSG_TIMEOUT);
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
        // // Send "the" message
        // GenericApp_SendTheMessage();
        // // Setup to send message again
        // osal_start_timerEx(GenericApp_TaskID,
        //                    GENERICAPP_SEND_MSG_EVT,
        //                    GENERICAPP_SEND_MSG_TIMEOUT);
        // return unprocessed events
        return (events ^ GENERICAPP_SEND_MSG_EVT);
    }
    if(events & GENERICAPP_UART_RX_EVT)
    {
        SHApp_SerialMSGCB();
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
    LREP("Received Data: %s \n", (char *)pkt->cmd.Data);
    switch (pkt->clusterId)
    {
    case GENERICAPP_CLUSTERID:
        rxMsgCount += 1;                          // Count this message
        HalLedSet(HAL_LED_4, HAL_LED_MODE_BLINK); //  an LED
        break;
    }
}
