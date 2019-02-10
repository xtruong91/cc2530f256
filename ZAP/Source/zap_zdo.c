/**************************************************************************************************
    Filename:       zap_zdo.c
    Revised:        $Date: 2013-11-13 13:14:39 -0800 (Wed, 13 Nov 2013) $
    Revision:       $Revision: 36080 $

    Description:

    This file declares the ZNP Application Processor proxy ZDO API functions.


    Copyright 2009-2011 Texas Instruments Incorporated. All rights reserved.

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

#include "mt.h"
#include "mt_rpc.h"
#include "OSAL.h"
#include "sapi.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_znp.h"
#include "ZComDef.h"
#include "ZDApp.h"
#include "ZDObject.h"

#if defined (ZAP_ZDO_FUNC)
/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef struct
{
  void *next;
  uint8 taskID;
  uint16 clusterID;
} ZDO_MsgCB_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static ZDO_MsgCB_t *zdoMsgCBs = (ZDO_MsgCB_t *)NULL;
static pfnZdoCb zdoCBFunc[MAX_ZDO_CB_FUNC];

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static void zapZDO_ConcentratorIndicationCB(uint8 *pBuf);
static void ZDO_SendMsgCBs(zdoIncomingMsg_t *inMsg);

/**************************************************************************************************
 * @fn          zapZdoProcessIncoming
 *
 * @brief       This function processes the ZDO sub-system response from the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that sent the message.
 * @param       pBuf - A pointer to the RPC response.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapZdoProcessIncoming(uint8 port, uint8 *pBuf)
{
  uint8 len = pBuf[MT_RPC_POS_LEN];
  uint8 cmd1 = pBuf[MT_RPC_POS_CMD1];

  pBuf += MT_RPC_FRAME_HDR_SZ;

  switch (cmd1)
  {
  case MT_ZDO_STATUS_ERROR_RSP:  // TODO: fill-in when/if needed.
    break;

  case MT_ZDO_STATE_CHANGE_IND:
    // Especially for UART transport, allow time for multiple got syncs before acting on it.
    if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_ZDO_STATE_CHANGE_EVT,
                                                  ZAP_APP_ZDO_STATE_CHANGE_DLY))
    {
      (void)osal_set_event(zapTaskId, ZAP_APP_ZDO_STATE_CHANGE_EVT);
    }
    break;

  case MT_ZDO_MATCH_DESC_RSP_SENT:  // TODO: fill-in when/if needed.
    break;

  case MT_ZDO_SRC_RTG_IND:  // TODO: fill-in when/if needed.
    break;

  case MT_ZDO_JOIN_CNF:  // TODO: fill-in when/if needed.
    break;

  case MT_ZDO_CONCENTRATOR_IND_CB:
    zapZDO_ConcentratorIndicationCB(pBuf);
    break;

  case MT_ZDO_MSG_CB_INCOMING:
    {
      zdoIncomingMsg_t inMsg;

      // Assuming exclusive use of network short addresses.
      inMsg.srcAddr.addrMode = Addr16Bit;
      inMsg.srcAddr.addr.shortAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
      pBuf += 2;
      inMsg.wasBroadcast = *pBuf++;
      inMsg.clusterID = BUILD_UINT16(pBuf[0], pBuf[1]);
      pBuf += 2;
      inMsg.SecurityUse = *pBuf++;
      inMsg.TransSeq = *pBuf++;
      inMsg.asduLen = len-9;
      inMsg.macDestAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
      pBuf += 2;
      inMsg.asdu = pBuf;

      ZDO_SendMsgCBs(&inMsg);
    }
    break;

  default:
    break;
  }
}

/**************************************************************************************************
 * @fn          zapZdoSync
 *
 * @brief       This function syncs the ZDO registered callbacks (which are not NV restored on ZNP.)
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
void zapZdoSync(void)
{
  ZDO_MsgCB_t *pCB = zdoMsgCBs;

  while (pCB)
  {
    if (znp_ZDO_RegisterForZDOMsg(pCB->clusterID) != ZSuccess)
    {
      // Especially for UART transport, allow time for multiple got syncs before acting on it.
      if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_SYNC_EVT, ZAP_APP_SYNC_DLY))
      {
        (void)osal_set_event(zapTaskId, ZAP_APP_SYNC_EVT);
      }
    }
    pCB = (ZDO_MsgCB_t *)pCB->next;
  }
}


/* ------------------------------------------------------------------------------------------------
 *                                     ZDApp Proxy
 * ------------------------------------------------------------------------------------------------
 */

/*********************************************************************
 * @fn      ZDOInitDevice
 *
 * @brief   Start the device in the network.  This function will read
 *   ZCD_NV_STARTUP_OPTION (NV item) to determine whether or not to
 *   restore the network state of the device.
 *
 * @param   startDelay - timeDelay to start device (in milliseconds).
 *      There is a jitter added to this delay:
 *              ((NWK_START_DELAY + startDelay)
 *              + (osal_rand() & EXTENDED_JOINING_RANDOM_MASK))
 *
 * NOTE:    If the application would like to force a "new" join, the
 *          application should set the ZCD_STARTOPT_DEFAULT_NETWORK_STATE
 *          bit in the ZCD_NV_STARTUP_OPTION NV item before calling
 *          this function. "new" join means to not restore the network
 *          state of the device. Use zgWriteStartupOptions() to set these
 *          options.
 *
 * @return
 *    ZDO_INITDEV_RESTORED_NETWORK_STATE  - The device's network state was
 *          restored.
 *    ZDO_INITDEV_NEW_NETWORK_STATE - The network state was initialized.
 *          This could mean that ZCD_NV_STARTUP_OPTION said to not restore, or
 *          it could mean that there was no network state to restore.
 *    ZDO_INITDEV_LEAVE_NOT_STARTED - Before the reset, a network leave was issued
 *          with the rejoin option set to TRUE.  So, the device was not
 *          started in the network (one time only).  The next time this
 *          function is called it will start.
 *    0xFF for failure.
 */
uint8 ZDOInitDevice(uint16 startDelay)
{
  uint8 *pBuf;
#if !ZAP_ZDO_STARTUP_AREQ
  uint8 rtrn;
#endif

  (void)startDelay;  // ZNP MT_ZDO_STARTUP_FROM_APP processing forces delay 0.

  zb_GetDeviceInfo(ZB_INFO_DEV_STATE, &devState);
  if ((DEV_HOLD != devState) && (DEV_INIT != devState) && (DEV_NWK_ORPHAN != devState))
  {
    return FAILURE;
  }

#if ZAP_ZDO_STARTUP_AREQ
  pBuf = zap_msg_allocate(0, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_AREQ,
                             (uint8)MT_ZDO_STARTUP_FROM_APP);
#else
  pBuf = zap_msg_allocate(0, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                             (uint8)MT_ZDO_STARTUP_FROM_APP);
#endif

  if (NULL == pBuf)
  {
    return 0xFF;
  }

  zapPhySend(zapAppPort, pBuf);
#if !ZAP_ZDO_STARTUP_AREQ
  if (ZSuccess == (rtrn = ZAP_SRSP_STATUS(pBuf)))
#endif
  // Need to locally enter the discovery state to holdoff calls to ZDOInitDevice() until the ZAP
  // monitoring task requests the actual ZNP state.
  devState = DEV_NWK_DISC;
  zap_msg_deallocate(&pBuf);

  // Joining can take some time - especially with > 1 scan channel.
  if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_TMR_EVT, ZAP_APP_JOIN_DLY))
  {
    (void)osal_set_event(zapTaskId, ZAP_APP_TMR_EVT);
  }

#if ZAP_ZDO_STARTUP_AREQ
  // Made into an AREQ after empirical results showed > 400 msec delay on SRSP.
#if ZAP_NV_RESTORE
  return ZDO_INITDEV_RESTORED_NETWORK_STATE;
#else
  return ZDO_INITDEV_NEW_NETWORK_STATE;
#endif
#else
  return rtrn;
#endif
}

/*********************************************************************
 * @fn          ZDO_ConcentratorIndicationCB
 *
 * @brief       This function allows the next higher layer of a
 *              device to be notified of existence of the concentrator.
 *
 * @param       nwkAddr - 16-bit NWK address of the concentrator
 * @param       extAddr - pointer to extended Address
 *                        NULL if not available
 * @param       pktCost - PktCost from RREQ
 *
 * @return      void
 */
static void zapZDO_ConcentratorIndicationCB(uint8 *pBuf)
{
  if (zdoCBFunc[ZDO_CONCENTRATOR_IND_CBID] != NULL)
  {
    zdoConcentratorInd_t conInd;

    conInd.nwkAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
    pBuf += 2;
    conInd.extAddr = pBuf;
    pBuf += Z_EXTADDR_LEN;
    conInd.pktCost = *pBuf;
    zdoCBFunc[ZDO_CONCENTRATOR_IND_CBID]((void *)&conInd);
  }
}

/*********************************************************************
 * @fn          ZDO_RegisterForZdoCB
 *
 * @brief       Call this function to register the higher layer (for
 *              example, the Application layer or MT layer) with ZDO
 *              callbacks to get notified of some ZDO indication like
 *              existence of a concentrator or receipt of a source
 *              route record.
 *
 * @param       indID - ZDO Indication ID
 * @param       pFn   - Callback function pointer
 *
 * @return      ZSuccess - successful, ZInvalidParameter if not
 */
ZStatus_t ZDO_RegisterForZdoCB( uint8 indID, pfnZdoCb pFn )
{
  // Check the range of the indication ID
  if ( indID < MAX_ZDO_CB_FUNC )
  {
    zdoCBFunc[indID] = pFn;
    return ZSuccess;
  }

  return ZInvalidParameter;
}


/* ------------------------------------------------------------------------------------------------
 *                                     ZDObject Proxy
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          ZDO_UpdateNwkStatus
 *
 * @brief       This function sends a ZDO_STATE_CHANGE message to the task of every EndPoint
 *              registered with AF (except, of course, the ZDO_EP). Even if a single task has more
 *              than one registered EndPoint, it will only receive one notification per state
 *              change. Although the device may go through a sequence of state changes, the
 *              Application task may only receive notification of the final, steady-state state
 *              because it has the lowest priority and never even runs to receive the intermediate
 *              state change notifications.
 *
 * input parameters
 *
 * @param       state - The current device state.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void ZDO_UpdateNwkStatus(devStates_t state)
{
  epList_t *pItem = epList;

  while (pItem != NULL)
  {
    if (pItem->epDesc->endPoint != ZDO_EP)
    {
      osal_event_hdr_t *pMsg = (osal_event_hdr_t *)osal_msg_find(*(pItem->epDesc->task_id),
                                                                         ZDO_STATE_CHANGE);

      if (NULL == pMsg)
      {
        if (NULL == (pMsg = (osal_event_hdr_t *)osal_msg_allocate(sizeof(osal_event_hdr_t))))
        {
          // Upon failure to notify any EndPoint of the state change, re-set the ZDO event to
          // try again later when more Heap may be available.
          osal_set_event(zapTaskId, ZAP_APP_ZDO_STATE_CHANGE_EVT);
        }
        else
        {
          pMsg->event = ZDO_STATE_CHANGE;
          pMsg->status = (uint8)state;

          (void)osal_msg_send(*(pItem->epDesc->task_id), (uint8 *)pMsg);
        }
      }
      else
      {
        // Modify in place the status of an existing ZDO_STATE_CHANGE message to the EndPoint.
        pMsg->status = (uint8)state;
      }
    }

    pItem = pItem->nextDesc;
  }
}

/*********************************************************************
 * @fn          ZDO_ParseDeviceAnnce
 *
 * @brief       Parse a Device Announce message.
 *
 * @param       inMsg - Incoming message
 * @param       pAnnce - place to put the parsed information
 *
 * @return      none
 */
void ZDO_ParseDeviceAnnce( zdoIncomingMsg_t *inMsg, ZDO_DeviceAnnce_t *pAnnce )
{
  uint8 *msg;

  // Parse incoming message
  msg = inMsg->asdu;
  pAnnce->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += 2;
  osal_cpyExtAddr( pAnnce->extAddr, msg );
  msg += Z_EXTADDR_LEN;
  pAnnce->capabilities = *msg;
}

/*********************************************************************
 * @fn          ZDO_ParseSimpleDescRsp
 *
 * @brief       This function parse the Simple_Desc_rsp message.
 *
 *   NOTE: The pAppInClusterList and pAppOutClusterList fields
 *         in the SimpleDescriptionFormat_t structure are allocated
 *         and the calling function needs to free [osal_msg_free()]
 *         these buffers.
 *
 * @param       inMsg  - incoming message
 * @param       pSimpleDescRsp - place to parse the message into
 *
 * @return      none
 */
void ZDO_ParseSimpleDescRsp( zdoIncomingMsg_t *inMsg, ZDO_SimpleDescRsp_t *pSimpleDescRsp )
{
  uint8 *msg;

  msg = inMsg->asdu;
  pSimpleDescRsp->status = *msg++;
  pSimpleDescRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
  msg += sizeof ( uint16 );
  msg++; // Skip past the length field.

  if ( pSimpleDescRsp->status == ZDP_SUCCESS )
  {
    ZDO_ParseSimpleDescBuf( msg, &(pSimpleDescRsp->simpleDesc) );
  }
}

/*********************************************************************
 * @fn          ZDO_ParseEPListRsp
 *
 * @brief       This parse the Active_EP_rsp or Match_Desc_rsp message.
 *
 * @param       inMsg  - incoming message
 *
 * @return      none
 */
ZDO_ActiveEndpointRsp_t *ZDO_ParseEPListRsp( zdoIncomingMsg_t *inMsg )
{
  ZDO_ActiveEndpointRsp_t *pRsp;
  uint8 *msg;
  uint8 Status;
  uint8 cnt;

  msg = inMsg->asdu;
  Status = *msg++;
  cnt = msg[2];

  pRsp = (ZDO_ActiveEndpointRsp_t *)osal_mem_alloc( sizeof(  ZDO_ActiveEndpointRsp_t ) + cnt );
  if ( pRsp )
  {
    pRsp->status = Status;
    pRsp->nwkAddr = BUILD_UINT16( msg[0], msg[1] );
    msg += sizeof( uint16 );
    pRsp->cnt = cnt;
    msg++; // pass cnt
    osal_memcpy( pRsp->epList, msg, cnt );
  }

  return ( pRsp );
}

/*********************************************************************
 * @fn          ZDO_ParseSimpleDescBuf
 *
 * @brief       Parse a byte sequence representation of a Simple Descriptor.
 *
 * @param       buf  - pointer to a byte array representing a Simple Desc.
 * @param       desc - SimpleDescriptionFormat_t *
 *
 *              This routine allocates storage for the cluster IDs because
 *              they are 16-bit and need to be aligned to be properly processed.
 *              This routine returns non-zero if an allocation fails.
 *
 *              NOTE: This means that the caller or user of the input structure
 *                    is responsible for freeing the memory
 *
 * @return      0: success
 *              1: failure due to malloc failure.
 */
uint8 ZDO_ParseSimpleDescBuf( uint8 *buf, SimpleDescriptionFormat_t *desc )
{
  uint8 num, i;

  desc->EndPoint = *buf++;
  desc->AppProfId = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;
  desc->AppDeviceId = BUILD_UINT16( buf[0], buf[1] );
  buf += 2;
  desc->AppDevVer = *buf >> 4;

  desc->Reserved = 0;
  buf++;

  // move in input cluster list (if any). allocate aligned memory.
  num = desc->AppNumInClusters = *buf++;
  if ( num )
  {
    if (!(desc->pAppInClusterList = (uint16 *)osal_mem_alloc(num*sizeof(uint16))))
    {
      // malloc failed. we're done.
      return 1;
    }
    for (i=0; i<num; ++i)
    {
      desc->pAppInClusterList[i] = BUILD_UINT16( buf[0], buf[1] );
      buf += 2;
    }
  }

  // move in output cluster list (if any). allocate aligned memory.
  num = desc->AppNumOutClusters = *buf++;
  if (num)
  {
    if (!(desc->pAppOutClusterList = (uint16 *)osal_mem_alloc(num*sizeof(uint16))))
    {
      // malloc failed. free input cluster list memory if there is any
      if ( desc->pAppInClusterList != NULL )
      {
        osal_mem_free(desc->pAppInClusterList);
      }
      return 1;
    }
    for (i=0; i<num; ++i)
    {
      desc->pAppOutClusterList[i] = BUILD_UINT16( buf[0], buf[1] );
      buf += 2;
    }
  }
  return 0;
}


/* ------------------------------------------------------------------------------------------------
 *                                     ZDProfile Proxy
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          ZDP_MatchDescReq
 *
 * @brief       This builds and send a Match_Desc_req message.  This
 *              function sends a broadcast or unicast message
 *              requesting the list of endpoint/interfaces that
 *              match profile ID and cluster IDs.
 *
 * input parameters
 *
 * @param       dstAddr - destination address
 * @param       ProfileID - Profile ID
 * @param       NumInClusters - number of input clusters
 * @param       InClusterList - input cluster ID list
 * @param       NumOutClusters - number of output clusters
 * @param       OutClusterList - output cluster ID list
 * @param       SecurityEnable - Security Options
 *
 * output parameters
 *
 * None.
 *
 * @return      afStatus_t
 **************************************************************************************************
 */
afStatus_t ZDP_MatchDescReq(zAddrType_t *dstAddr, uint16 nwkAddr,
                            uint16 ProfileID,
                            byte NumInClusters, cId_t *InClusterList,
                            byte NumOutClusters, cId_t *OutClusterList,
                            byte SecurityEnable)
{
  const uint8 len = 8 + ((NumInClusters + NumOutClusters) * 2);
  uint8 *pBuf, *pPtr, cnt;

  (void)SecurityEnable;

  if ((Addr16Bit != dstAddr->addrMode) && (AddrBroadcast != dstAddr->addrMode))
  {
    return afStatus_INVALID_PARAMETER;
  }

  pBuf = zap_msg_allocate(len, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                               (uint8)MT_ZDO_MATCH_DESC_REQ);

  if (NULL == pBuf)
  {
    return afStatus_MEM_FAIL;
  }

  pPtr = pBuf;
  *pPtr++ = LO_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = HI_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = LO_UINT16(nwkAddr);
  *pPtr++ = HI_UINT16(nwkAddr);
  *pPtr++ = LO_UINT16(ProfileID);
  *pPtr++ = HI_UINT16(ProfileID);
  *pPtr++ = NumInClusters;
  for (cnt = 0; cnt < NumInClusters; cnt++)
  {
    *pPtr++ = LO_UINT16(*InClusterList);
    *pPtr++ = HI_UINT16(*InClusterList);
    InClusterList++;
  }
  *pPtr++ = NumOutClusters;
  for (cnt = 0; cnt < NumOutClusters; cnt++)
  {
    *pPtr++ = LO_UINT16(*OutClusterList);
    *pPtr++ = HI_UINT16(*OutClusterList);
    OutClusterList++;
  }

  zapPhySend(zapAppPort, pBuf);
  cnt = ZAP_SRSP_STATUS(pBuf);
  zap_msg_deallocate(&pBuf);

  return (afStatus_t)cnt;
}

/*********************************************************************
 * @fn          ZDP_SimpleDescReq
 *
 * @brief       This builds and send a NWK_Simple_Desc_req
 *              message.  This function sends unicast message to the
 *              destination device.
 *
 * @param       dstAddr - destination address
 * @param       nwkAddr - 16 bit address
 * @param       epIntf - endpoint/interface
 * @param       SecurityEnable - Security Options
 *
 * @return      afStatus_t
 */
afStatus_t ZDP_SimpleDescReq( zAddrType_t *dstAddr, uint16 nwkAddr,
                                    byte endPoint, byte SecurityEnable )

{
  uint8 *pBuf, rtrn;

  (void)SecurityEnable;

  pBuf = zap_msg_allocate(5, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                             (uint8)MT_ZDO_SIMPLE_DESC_REQ);
  if (NULL == pBuf)
  {
    return afStatus_MEM_FAIL;
  }

  pBuf[0] = LO_UINT16(dstAddr->addr.shortAddr);
  pBuf[1] = HI_UINT16(dstAddr->addr.shortAddr);
  pBuf[2] = LO_UINT16(nwkAddr);
  pBuf[3] = HI_UINT16(nwkAddr);
  pBuf[4] = endPoint;

  zapPhySend(zapAppPort, pBuf);
  rtrn = ZAP_SRSP_STATUS(pBuf);
  zap_msg_deallocate(&pBuf);

  return (afStatus_t)rtrn;
}

/**************************************************************************************************
 * @fn          ZDP_EndDeviceBindReq
 *
 * @brief       This builds and sends a End_Device_Bind_req message.
 *              This function sends a unicast message.
 *
 * input parameters
 *
 * @param       dstAddr - destination address
 * @param       LocalCoordinator - short address of local coordinator
 * @param       epIntf - Endpoint/Interface of Simple Desc
 * @param       ProfileID - Profile ID
 *
 *   The Input cluster list is the opposite of what you would think.
 *   This is the output cluster list of this device
 * @param       NumInClusters - number of input clusters
 * @param       InClusterList - input cluster ID list
 *
 *   The Output cluster list is the opposite of what you would think.
 *   This is the input cluster list of this device
 * @param       NumOutClusters - number of output clusters
 * @param       OutClusterList - output cluster ID list
 *
 * @param       SecurityEnable - Security Options
 *
 * output parameters
 *
 * None.
 *
 * @return      afStatus_t
 **************************************************************************************************
 */
afStatus_t ZDP_EndDeviceBindReq(zAddrType_t *dstAddr,
                                uint16 LocalCoordinator,
                                byte endPoint,
                                uint16 ProfileID,
                                byte NumInClusters, cId_t *InClusterList,
                                byte NumOutClusters, cId_t *OutClusterList,
                                byte SecurityEnable)
{
  const uint8 len = 17 + ((NumInClusters + NumOutClusters) * 2);
  uint8 *pBuf, *pPtr, cnt;

  (void)SecurityEnable;

  if ((Addr16Bit != dstAddr->addrMode) && (AddrBroadcast != dstAddr->addrMode))
  {
    return afStatus_INVALID_PARAMETER;
  }

  pBuf = zap_msg_allocate(len, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                               (uint8)MT_ZDO_END_DEV_BIND_REQ);

  if (NULL == pBuf)
  {
    return afStatus_MEM_FAIL;
  }

  pPtr = pBuf;
  *pPtr++ = LO_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = HI_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = LO_UINT16(LocalCoordinator);
  *pPtr++ = HI_UINT16(LocalCoordinator);
  //(void)osal_memset(pPtr, 0, Z_EXTADDR_LEN);  // MT_ZDO ignores extended address.
  pPtr += Z_EXTADDR_LEN;
  *pPtr++ = endPoint;
  *pPtr++ = LO_UINT16(ProfileID);
  *pPtr++ = HI_UINT16(ProfileID);
  *pPtr++ = NumInClusters;
  for (cnt = 0; cnt < NumInClusters; cnt++)
  {
    *pPtr++ = LO_UINT16(*InClusterList);
    *pPtr++ = HI_UINT16(*InClusterList);
    InClusterList++;
  }
  *pPtr++ = NumOutClusters;
  for (cnt = 0; cnt < NumOutClusters; cnt++)
  {
    *pPtr++ = LO_UINT16(*OutClusterList);
    *pPtr++ = HI_UINT16(*OutClusterList);
    OutClusterList++;
  }

  zapPhySend(zapAppPort, pBuf);
  cnt = ZAP_SRSP_STATUS(pBuf);
  zap_msg_deallocate(&pBuf);

  return (afStatus_t)cnt;
}

/*********************************************************************
 * @fn          ZDP_MgmtPermitJoinReq
 *
 * @brief       This builds and sends a Mgmt_Permit_Join_req message.
 *
 * input parameters
 *
 * @param       dstAddr - destination address of the message
 * @param       duration - Permit duration
 * @param       TcSignificance - Trust Center Significance
 *
 * output parameters
 *
 * None.
 *
 * @return      afStatus_t
 **************************************************************************************************
 */
afStatus_t ZDP_MgmtPermitJoinReq( zAddrType_t *dstAddr, byte duration,
                                  byte TcSignificance, byte SecurityEnable )
{
  (void)SecurityEnable;  // Intentionally unreferenced parameter

  uint8 *pBuf, *pPtr, rtrn;

  if ((Addr16Bit != dstAddr->addrMode) && (AddrBroadcast != dstAddr->addrMode))
  {
    return afStatus_INVALID_PARAMETER;
  }

  pBuf = zap_msg_allocate(5, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                             (uint8)MT_ZDO_MGMT_PERMIT_JOIN_REQ);

  if (NULL == pBuf)
  {
    return afStatus_MEM_FAIL;
  }

  pPtr = pBuf;
  *pPtr++ = dstAddr->addrMode;
  *pPtr++ = LO_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = HI_UINT16(dstAddr->addr.shortAddr);
  *pPtr++ = duration;
  *pPtr++ = TcSignificance;

  zapPhySend(zapAppPort, pBuf);
  rtrn = ZAP_SRSP_STATUS(pBuf);
  zap_msg_deallocate(&pBuf);

  return (afStatus_t)rtrn;
}

/*********************************************************************
 * @fn          ZDO_RegisterForZDOMsg
 *
 * @brief       Call this function to register of an incoming over
 *              the air ZDO message - probably a response message
 *              but requests can also be received.
 *              Messages are delivered to the task with ZDO_CB_MSG
 *              as the message ID.
 *
 * @param       taskID - Where you would like the message delivered
 * @param       clusterID - What message?
 *
 * @return      ZSuccess - successful, ZMemError if not
 */
ZStatus_t ZDO_RegisterForZDOMsg(uint8 taskID, uint16 clusterID)
{
  ZDO_MsgCB_t *pList;
  ZDO_MsgCB_t *pLast;
  ZDO_MsgCB_t *pNew;
  (void)taskID;

  // Look for duplicate
  pList = pLast = zdoMsgCBs;
  while ( pList )
  {
    if ( pList->taskID == taskID && pList->clusterID == clusterID )
      return ( ZSuccess );
    pLast = pList;
    pList = (ZDO_MsgCB_t *)pList->next;
  }

  // Add to the list
  pNew = (ZDO_MsgCB_t *)osal_mem_alloc( sizeof ( ZDO_MsgCB_t ) );
  if ( pNew )
  {
    pNew->taskID = taskID;
    pNew->clusterID = clusterID;
    pNew->next = NULL;
    if ( zdoMsgCBs )
    {
      pLast->next = pNew;
    }
    else
      zdoMsgCBs = pNew;

    return znp_ZDO_RegisterForZDOMsg(clusterID);
  }

  return ZMemError;
}

/*********************************************************************
 * @fn          ZDO_RemoveRegisteredCB
 *
 * @brief       Call this function if you don't want to receive the
 *              incoming message.
 *
 * @param       taskID - Where the messages are being delivered.
 * @param       clusterID - What message?
 *
 * @return      ZSuccess - successful, ZFailure if not found
 */
ZStatus_t ZDO_RemoveRegisteredCB( uint8 taskID, uint16 clusterID )
{
  ZDO_MsgCB_t *pList;
  ZDO_MsgCB_t *pLast = NULL;

  pList = zdoMsgCBs;
  while ( pList )
  {
    if ( pList->taskID == taskID && pList->clusterID == clusterID )
    {
      uint8 *pBuf, rtrn;

      // Note that this could be sent AREQ to free up the host (but then no status.)
      if (NULL == (pBuf = zap_msg_allocate(2, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                                              (uint8)MT_ZDO_MSG_CB_REMOVE)))
      {
        return ZMemError;
      }

      pBuf[0] = LO_UINT16(clusterID);
      pBuf[1] = HI_UINT16(clusterID);

      zapPhySend(zapAppPort, pBuf);
      rtrn = ZAP_SRSP_STATUS(pBuf);
      zap_msg_deallocate(&pBuf);

      if (ZSuccess != rtrn)
      {
        return (ZStatus_t)rtrn;
      }

      if ( pLast )
      {
        // remove this one from the linked list
        pLast->next = pList->next;
      }
      else if ( pList->next )
      {
        // remove the first one from the linked list
        zdoMsgCBs = pList->next;
      }
      else
      {
        // remove the only item from the list
        zdoMsgCBs = (ZDO_MsgCB_t *)NULL;
      }
      osal_mem_free( pList );
      return ( ZSuccess );
    }
    pLast = pList;
    pList = pList->next;
  }

  return ( ZFailure );
}

/*********************************************************************
 * @fn          ZDO_SendMsgCBs
 *
 * @brief       De-mux the ZDO_CB_MSG from the ZNP.
 *
 * @param       inMsg - pointer to the incoming message buffer.
 *
 * @return      none
 */
static void ZDO_SendMsgCBs(zdoIncomingMsg_t *inMsg)
{
  ZDO_MsgCB_t *pList = zdoMsgCBs;

  while ( pList )
  {
    if ( pList->clusterID == inMsg->clusterID )
    {
      zdoIncomingMsg_t *msgPtr;

      // Send the address to the task
      msgPtr = (zdoIncomingMsg_t *)osal_msg_allocate( sizeof( zdoIncomingMsg_t ) + inMsg->asduLen );
      if ( msgPtr )
      {
        // copy struct
        osal_memcpy( msgPtr, inMsg, sizeof( zdoIncomingMsg_t ));

        if ( inMsg->asduLen )
        {
          msgPtr->asdu = (byte*)(((byte*)msgPtr) + sizeof( zdoIncomingMsg_t ));
          osal_memcpy( msgPtr->asdu, inMsg->asdu, inMsg->asduLen );
        }

        msgPtr->hdr.event = ZDO_CB_MSG;
        osal_msg_send( pList->taskID, (uint8 *)msgPtr );
      }
    }
    pList = (ZDO_MsgCB_t *)pList->next;
  }
}

#endif
/**************************************************************************************************
*/
