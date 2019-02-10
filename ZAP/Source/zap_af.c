/**************************************************************************************************
  Filename:       zap_af.c
  Revised:        $Date: 2014-05-14 13:17:12 -0700 (Wed, 14 May 2014) $
  Revision:       $Revision: 38539 $

  Description:

  This file defines the ZNP Application Processor API to the ZNP AF layer.


  Copyright 2009-2014 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#include "af.h"
#include "mt.h"
#include "mt_rpc.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_znp.h"

#if defined (ZAP_AF_FUNC)
/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

epList_t *epList;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

#if ZAP_AF_DATA_REQ_FRAG
static afStatus_t afStore(uint8 *buf, uint16 len);
#endif
static void afCnf(uint8 *pBuf);
static epList_t *afFindEndPointDescList(uint8 EndPoint);
static void afRecv(uint8 *pBuf);
static uint8 *afIncMsgPktParse(uint8 cmd1, uint8 *pBuf, afIncomingMSGPacket_t *pMsg);
#if ZAP_AF_DATA_REQ_FRAG
static void afRetrieve(uint8 taskId, afIncomingMSGPacket_t *pMsg);
#endif

/**************************************************************************************************
 * @fn          zapAfProcessIncoming
 *
 * @brief       This function processes the AF sub-system response from the ZNP.
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
void zapAfProcessIncoming(uint8 port, uint8 *pBuf)
{
  switch (pBuf[MT_RPC_POS_CMD1])
  {
  case MT_AF_DATA_CONFIRM:
    afCnf(pBuf);
    break;

  case MT_AF_INCOMING_MSG:
  case MT_AF_INCOMING_MSG_EXT:
    afRecv(pBuf);
    break;

  default:
    break;
  }
}

/**************************************************************************************************
 * @fn          zapAfSync
 *
 * @brief       This function syncs the AF registered endpoints (which are not NV restored on ZNP.)
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
void zapAfSync(void)
{
  epList_t *epSync = epList;

  while (epSync)
  {
    if ((epSync->epDesc->simpleDesc != NULL) && !znp_afRegisterExtended(epSync->epDesc))
    {
      // Especially for UART transport, allow time for multiple got syncs before acting on it.
      if (ZSuccess != osal_start_timerEx(zapTaskId, ZAP_APP_SYNC_EVT, ZAP_APP_SYNC_DLY))
      {
        (void)osal_set_event(zapTaskId, ZAP_APP_SYNC_EVT);
      }
    }
    epSync = epSync->nextDesc;
  }
#if defined INTER_PAN
  zapStubAPS_Sync();
#endif
}

/**************************************************************************************************
 * @fn      afRegisterExtended
 *
 * @brief   Register an Application's EndPoint description.
 *
 * input parameters
 *
 * @param   epDesc - pointer to the Application's endpoint descriptor.
 *                   NOTE: The memory that epDesc is pointing to must persist after this call.
 * @param   descFn - pointer to descriptor callback function - only NULL supported on ZAP.
 * @param   applFn - pointer to Application callback function - only NULL supported on ZAP.
 *
 * output parameters
 *
 * None.
 *
 * @return  Pointer to epList_t on success, NULL otherwise.
 **************************************************************************************************
 */
epList_t *afRegisterExtended( endPointDesc_t *epDesc, pDescCB descFn, pApplCB applFn )
{
  epList_t *ep;
  epList_t *epSearch;

  ep = osal_mem_alloc( sizeof ( epList_t ) );
  if ( ep )
  {
    // Fill in the new list entry
    ep->epDesc = epDesc;

    // Default to allow Match Descriptor.
    ep->flags = eEP_AllowMatch;
    ep->pfnDescCB = descFn;
    ep->nextDesc = NULL;
    ep->pfnApplCB = applFn;

    // Does a list exist?
    if ( epList == NULL )
      epList = ep;  // Make this the first entry
    else
    {
      // Look for the end of the list
      epSearch = epList;
      while( epSearch->nextDesc != NULL )
        epSearch = epSearch->nextDesc;

      // Add new entry to end of list
      epSearch->nextDesc = ep;
    }

    if ((epDesc->simpleDesc != NULL) && !znp_afRegisterExtended(epDesc))
    {
      return NULL;
    }
  }

  return ep;
}

/**************************************************************************************************
 * @fn      afRegister
 *
 * @brief   Register an Application's EndPoint description.
 *
 * input parameters
 *
 * @param   epDesc - pointer to the Application's endpoint descriptor.
 *
 * output parameters
 *
 * None.
 *
 * @return  afStatus_SUCCESS - Registered
 *          afStatus_MEM_FAIL - not enough memory to add descriptor
 *          afStatus_INVALID_PARAMETER - duplicate endpoint
 **************************************************************************************************
 */
afStatus_t afRegister(endPointDesc_t *epDesc)
{
  epList_t *ep;

  // Look for duplicate endpoint
  if ( afFindEndPointDescList( epDesc->endPoint ) )
    return ( afStatus_INVALID_PARAMETER );

  ep = afRegisterExtended( epDesc, NULL, NULL );

  return ((ep == NULL) ? afStatus_MEM_FAIL : afStatus_SUCCESS);
}

/**************************************************************************************************
 * @fn      AF_DataRequest
 *
 * @brief   API definition to invoke AF_DataRequest on the ZNP.
 *
 * input parameters
 *
 * @param  *dstAddr - Full ZB destination address: Nwk Addr + End Point.
 * @param  *srcEP - Origination (i.e. respond to or ack to) End Point Descr.
 * @param   cID - A valid cluster ID as specified by the Profile.
 * @param   len - Number of bytes of data pointed to by next param.
 * @param  *buf - A pointer to the data bytes to send.
 * @param  *transID - A pointer to a byte which can be modified and which will
 *                    be used as the transaction sequence number of the msg.
 * @param   options - Valid bit mask of Tx options.
 * @param   radius - Normally set to AF_DEFAULT_RADIUS.
 *
 * output parameters
 *
 * @param  *transID - Incremented by one if the return value is success.
 *
 * @return  afStatus_t per declaration.
 **************************************************************************************************
 */
afStatus_t AF_DataRequest(afAddrType_t *dstAddr, endPointDesc_t *srcEP,
                          uint16 cID, uint16 len, uint8 *buf, uint8 *transID,
                          uint8 options, uint8 radius)
{
  #define ZAP_AF_REQ_MSG_HDR  20
  #define ZAP_AF_REQ_DAT_MAX (MT_RPC_DATA_MAX - ZAP_AF_REQ_MSG_HDR)
  uint8 *pBuf;

#if !ZAP_AF_DATA_REQ_AREQ
  afStatus_t rtrn;

#if ZAP_AF_DATA_REQ_FRAG
  if (len > ZAP_AF_REQ_DAT_MAX)
  {
    pBuf = zap_msg_allocate(ZAP_AF_REQ_MSG_HDR, (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_SREQ,
                                                (uint8)MT_AF_DATA_REQUEST_EXT);
  }
  else
#endif
  {
    pBuf = zap_msg_allocate((uint8)(len + ZAP_AF_REQ_MSG_HDR),
                            (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_SREQ,
                            (uint8)MT_AF_DATA_REQUEST_EXT);
  }
#else
  pBuf = zap_msg_allocate((uint8)(len + ZAP_AF_REQ_MSG_HDR),
                          (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_AREQ,
                          (uint8)MT_AF_DATA_REQUEST_EXT);
#endif

  if (NULL == pBuf)
  {
    return afStatus_MEM_FAIL;
  }

  *pBuf++ = dstAddr->addrMode;

  if (dstAddr->addrMode == afAddr64Bit)
  {
    (void)osal_memcpy(pBuf, dstAddr->addr.extAddr, Z_EXTADDR_LEN);
  }
  else
  {
    pBuf[0] = LO_UINT16(dstAddr->addr.shortAddr);
    pBuf[1] = HI_UINT16(dstAddr->addr.shortAddr);
  }
  pBuf += Z_EXTADDR_LEN;

  *pBuf++ = dstAddr->endPoint;
#if defined INTER_PAN
  *pBuf++ = LO_UINT16(dstAddr->panId);
  *pBuf++ = HI_UINT16(dstAddr->panId);
#else
  *pBuf++ = 0;
  *pBuf++ = 0;
#endif
  *pBuf++ = srcEP->endPoint;
  *pBuf++ = LO_UINT16(cID);
  *pBuf++ = HI_UINT16(cID);
  *pBuf++ = *transID;
  (*transID)++;
  *pBuf++ = options;
  *pBuf++ = radius;
  *pBuf++ = LO_UINT16(len);
  *pBuf++ = HI_UINT16(len);

#if ZAP_AF_DATA_REQ_FRAG
  if (len <= ZAP_AF_REQ_DAT_MAX)
#endif
  {
    (void)osal_memcpy(pBuf, buf, len);
  }

  pBuf -= ZAP_AF_REQ_MSG_HDR;
  zapPhySend(zapAppPort, pBuf);
#if !ZAP_AF_DATA_REQ_AREQ
  rtrn = (afStatus_t)ZAP_SRSP_STATUS(pBuf);
#endif
  zap_msg_deallocate(&pBuf);

#if ZAP_AF_DATA_REQ_FRAG
  if ((len > ZAP_AF_REQ_DAT_MAX) && (rtrn == afStatus_SUCCESS))
  {
    rtrn = afStore(buf, len);
  }
#endif

#if ZAP_AF_DATA_REQ_AREQ
  return SUCCESS;
#else
  return rtrn;
#endif
}

#if ZAP_AF_DATA_REQ_FRAG
/**************************************************************************************************
 * @fn          afStore
 *
 * @brief       This function stores a huge outgoing message data buffer on the ZNP.
 *
 * input parameters
 *
 * @param       buf - Pointer to the message data buffer.
 * @param       len - The length of the message data buffer 'buf'.
 *
 * output parameters
 *
 * None.
 *
 * @return      The AF-Status of storing the message data on the ZNP.
 **************************************************************************************************
 */
static afStatus_t afStore(uint8 *buf, uint16 len)
{
  #define ZAP_AF_STO_MSG_HDR  3
  #define ZAP_AF_STO_DAT_MAX (MT_RPC_DATA_MAX - ZAP_AF_STO_MSG_HDR)
  uint8 *pBuf;
  uint16 idx = 0;
  uint8 tmpLen = 0;
  afStatus_t rtrn;

  do {
    /* This trick to pre-decrement (with zero on the first pass) allows the while() test to
     * succeed and loop to send a zero data length message which will trigger the ZNP to send the
     * accumulated data OTA in an AF_DataRequest().
     */
    len -= tmpLen;
    idx += tmpLen;

    if (len > ZAP_AF_STO_DAT_MAX)
    {
      tmpLen = ZAP_AF_STO_DAT_MAX;
    }
    else
    {
      tmpLen = len;
    }

    if ((pBuf = zap_msg_allocate((uint8)(tmpLen + ZAP_AF_STO_MSG_HDR),
                                 (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_SREQ,
                                 (uint8)MT_AF_DATA_STORE)) == NULL)
    {
      rtrn = afStatus_MEM_FAIL;
      break;
    }

    pBuf[0] = LO_UINT16(idx);
    pBuf[1] = HI_UINT16(idx);
    pBuf[2] = tmpLen;
    (void)osal_memcpy(pBuf+3, buf+idx, tmpLen);
    zapPhySend(zapAppPort, pBuf);
    rtrn = (afStatus_t)ZAP_SRSP_STATUS(pBuf);
    zap_msg_deallocate(&pBuf);
  } while ((rtrn == afStatus_SUCCESS) && len);

  return rtrn;
}
#endif

/**************************************************************************************************
 * @fn          afCnf
 *
 * @brief       This function de-muxes an incoming AF data confirm message.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC message buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void afCnf(uint8 *pBuf)
{
  pBuf += MT_RPC_FRAME_HDR_SZ;
  epList_t *pEP = afFindEndPointDescList(pBuf[1]);

  if (NULL != pEP)
  {
    afDataConfirm_t *pMsg = (afDataConfirm_t *)osal_msg_allocate(sizeof(afDataConfirm_t));

    if (NULL != pMsg)
    {
      pMsg->hdr.event = AF_DATA_CONFIRM_CMD;
      pMsg->hdr.status = *pBuf++;
      pMsg->endpoint = *pBuf++;
      pMsg->transID = *pBuf;
      osal_msg_send(*(pEP->epDesc->task_id), (uint8 *)pMsg);
    }
  }
}

/**************************************************************************************************
 * @fn      afFindEndPointDescList
 *
 * @brief   Find the endpoint description entry from the endpoint number.
 *
 * input parameters
 *
 * @param   EndPoint - Application Endpoint to look for
 *
 * output parameters
 *
 * None.
 *
 * @return  the address to the endpoint/interface description entry
 **************************************************************************************************
 */
static epList_t *afFindEndPointDescList( uint8 EndPoint )
{
  epList_t *epSearch;

  // Start at the beginning
  epSearch = epList;

  // Look through the list until the end
  while ( epSearch )
  {
    // Is there a match?
    if ( epSearch->epDesc->endPoint == EndPoint )
    {
      return ( epSearch );
    }
    else
      epSearch = epSearch->nextDesc;  // Next entry
  }

  return ( (epList_t *)NULL );
}

/**************************************************************************************************
 * @fn      afFindEndPointDesc
 *
 * @brief   Find the endpoint description entry from the endpoint number.
 *
 * input parameters
 *
 * @param   EndPoint - Application Endpoint to look for
 *
 * output parameters
 *
 * None.
 *
 * @return  the address to the endpoint/interface description entry
 **************************************************************************************************
 */
endPointDesc_t *afFindEndPointDesc(uint8 EndPoint)
{
  epList_t *epSearch;

  // Look for the endpoint
  epSearch = afFindEndPointDescList( EndPoint );

  if ( epSearch )
    return ( epSearch->epDesc );
  else
    return ( (endPointDesc_t *)NULL );
}

/**************************************************************************************************
 * @fn          afRecv
 *
 * @brief       This function de-muxes an incoming AF data message.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC message buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void afRecv(uint8 *pBuf)
{
  #define ZAP_AF_INC_MSG_HDR  27
  #define ZAP_AF_INC_DAT_MAX (MT_RPC_DATA_MAX - ZAP_AF_INC_MSG_HDR)

  afIncomingMSGPacket_t *pMsg;
  epList_t *pEP;
  uint16 tmp;
  uint8 cmd1 = pBuf[MT_RPC_POS_CMD1];

  pBuf += MT_RPC_FRAME_HDR_SZ;
  if (cmd1 == MT_AF_INCOMING_MSG)
  {
    pEP = afFindEndPointDescList(pBuf[7]);
    tmp = pBuf[16];
  }
  else
  {
    pEP = afFindEndPointDescList(pBuf[16]);
    tmp = BUILD_UINT16(pBuf[25], pBuf[26]);
  }

  if ((pEP == NULL) || (NULL ==
      (pMsg = (afIncomingMSGPacket_t *)osal_msg_allocate(sizeof(afIncomingMSGPacket_t) + tmp))))
  {
    return;
  }

  pMsg->hdr.event = AF_INCOMING_MSG_CMD;
  pBuf = afIncMsgPktParse(cmd1, pBuf, pMsg);

#if ZAP_AF_DATA_REQ_FRAG
  if (pMsg->cmd.DataLength > ZAP_AF_INC_DAT_MAX)
  {
    afRetrieve(*(pEP->epDesc->task_id), pMsg);
  }
  else
#endif
  {
    if (pMsg->cmd.DataLength)
    {
      (void)osal_memcpy(pMsg->cmd.Data, pBuf, pMsg->cmd.DataLength);
    }
    else
    {
      pMsg->cmd.Data = NULL;
    }

    (void)osal_msg_send(*(pEP->epDesc->task_id), (uint8 *)pMsg);
  }
}

/**************************************************************************************************
 * @fn          afIncMsgPktParse
 *
 * @brief       This function parses an incoming AF data buffer into an afIncomingMSGPacket_t
 *              structure.
 *
 * input parameters
 *
 * @param       cmd1 - The RPC command type: MT_AF_INCOMING_MSG or MT_AF_INCOMING_MSG_EXT.
 * @param       pMsg - Pointer to the afIncomingMSGPacket_t structure.
 * @param       pMsg - Pointer to the afIncomingMSGPacket_t structure.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static uint8 *afIncMsgPktParse(uint8 cmd1, uint8 *pBuf, afIncomingMSGPacket_t *pMsg)
{
  pMsg->groupId = BUILD_UINT16(pBuf[0], pBuf[1]);
  pBuf += 2;
  pMsg->clusterId = BUILD_UINT16(pBuf[0], pBuf[1]);
  pBuf += 2;

  if (cmd1 == MT_AF_INCOMING_MSG)
  {
    pMsg->srcAddr.addrMode = afAddr16Bit;
    pMsg->srcAddr.addr.shortAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
    pMsg->macDestAddr = pMsg->srcAddr.addr.shortAddr;
    pBuf += 2;
    pMsg->srcAddr.endPoint = *pBuf++;
    pMsg->srcAddr.panId = znpPanId;
  }
  else
  {
    pMsg->srcAddr.addrMode = (afAddrMode_t)*pBuf++;

    if (pMsg->srcAddr.addrMode == afAddr64Bit)
    {
      (void)osal_memcpy(pMsg->srcAddr.addr.extAddr, pBuf, Z_EXTADDR_LEN);
      pMsg->macDestAddr = 0xFFFF;
    }
    else
    {
      pMsg->srcAddr.addr.shortAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
      pMsg->macDestAddr = pMsg->srcAddr.addr.shortAddr;
    }
    pBuf += Z_EXTADDR_LEN;

    pMsg->srcAddr.endPoint = *pBuf++;
    pMsg->srcAddr.panId = BUILD_UINT16(pBuf[0], pBuf[1]);
    pBuf += 2;
  }
  pMsg->endPoint = *pBuf++;
  pMsg->wasBroadcast = *pBuf++;
  pMsg->LinkQuality = *pBuf++;
  pMsg->correlation = pMsg->rssi = 0;
  pMsg->SecurityUse = *pBuf++;
  pMsg->timestamp = BUILD_UINT32(pBuf[0], pBuf[1], pBuf[2], pBuf[3]);
  pBuf += 4;
  pMsg->cmd.TransSeqNumber = *pBuf++;

  if (cmd1 == MT_AF_INCOMING_MSG)
  {
    pMsg->cmd.DataLength = *pBuf++;
  }
  else
  {
    pMsg->cmd.DataLength = BUILD_UINT16(pBuf[0], pBuf[1]);
    pBuf += 2;
  }
  pMsg->cmd.Data = (uint8 *)(pMsg+1);
  return pBuf;
}

#if ZAP_AF_DATA_REQ_FRAG
/**************************************************************************************************
 * @fn          afRetrieve
 *
 * @brief       This function retrieves the data of a huge incoming message. On an failure during
 *              the retrieval, the incoming message is freed. Otherwise, the incoming message is
 *              forwarded to the corresponding task.
 *
 * input parameters
 *
 * @param       pMsg - Pointer to the incoming AF message.
 * @param       taskId - The task ID corresponding to the destination endpoint of the message.
 *
 * output parameters
 *
 * @param       pMsg->cmd.Data - The incoming message data buffer member is filled.
 *
 * @return      None.
 **************************************************************************************************
 */
static void afRetrieve(uint8 taskId, afIncomingMSGPacket_t *pMsg)
{
  #define ZAP_AF_RTV_MSG_HDR  7  // Retrieve message header length.
  #define ZAP_AF_RTV_RPY_HDR  2  // Retrieve-reply message header length.
  #define ZAP_AF_RTV_DAT_MAX (MT_RPC_DATA_MAX - ZAP_AF_RTV_RPY_HDR)

  uint16 idx = 0, len = pMsg->cmd.DataLength;
  uint8 *pBuf, rtrn, tmpLen = 0;

  do {
    /* This trick to pre-decrement (with zero on the first pass) allows the while() test to
     * succeed and loop to send a zero data length message which will trigger the ZNP to
     * de-allocate the huge incoming message being held.
     */
    len -= tmpLen;
    idx += tmpLen;

    if (len > ZAP_AF_RTV_DAT_MAX)
    {
      tmpLen = ZAP_AF_RTV_DAT_MAX;
    }
    else
    {
      tmpLen = len;
    }

    if ((pBuf = zap_msg_allocate(ZAP_AF_RTV_MSG_HDR, ((uint8)MT_RPC_SYS_AF | MT_RPC_CMD_SREQ),
                                                             MT_AF_DATA_RETRIEVE)) == NULL)
    {
      rtrn = afStatus_MEM_FAIL;
      break;
    }

    pBuf[0] = BREAK_UINT32(pMsg->timestamp, 0);
    pBuf[1] = BREAK_UINT32(pMsg->timestamp, 1);
    pBuf[2] = BREAK_UINT32(pMsg->timestamp, 2);
    pBuf[3] = BREAK_UINT32(pMsg->timestamp, 3);
    pBuf[4] = LO_UINT16(idx);
    pBuf[5] = HI_UINT16(idx);
    pBuf[6] = tmpLen;
    zapPhySend(zapAppPort, pBuf);
    rtrn = (afStatus_t)ZAP_SRSP_STATUS(pBuf);
    (void)osal_memcpy(pMsg->cmd.Data+idx, pBuf+ZAP_AF_RTV_RPY_HDR, tmpLen);
    zap_msg_deallocate(&pBuf);
  } while ((rtrn == afStatus_SUCCESS) && len);

  if (rtrn == afStatus_SUCCESS)
  {
    (void)osal_msg_send(taskId, (uint8 *)pMsg);
  }
  else
  {
    (void)osal_msg_deallocate((uint8 *)pMsg);
  }
}
#endif

#endif
/**************************************************************************************************
*/
