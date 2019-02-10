/**************************************************************************************************
    Filename:       zap_util.c
    Revised:        $Date: 2013-11-15 17:27:39 -0800 (Fri, 15 Nov 2013) $
    Revision:       $Revision: 36132 $

    Description:

    This file declares the ZNP Application Processor UTIL API functions.


    Copyright 2010-2013 Texas Instruments Incorporated. All rights reserved.

    IMPORTANT: Your use of this Software is limited to those specific rights
    granted under the terms of a software license agreement between the user
    who downloaded the software, his/her employer (which must be your employer)
    and Texas Instruments Incorporated (the "License"). You may not use this
    Software unless you agree to abide by the terms of the License. The License
    limits your use, and you acknowledge, that the Software may not be modified,
    copied or distributed unless embedded on a Texas Instruments microcontroller
    or used solely and exclusively in conjunction with a Texas Instruments radio
    frequency transceiver, which is integrated into your product. Other than for
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

#include "AddrMgr.h"
#include "hal_board.h"
#include "mt.h"
#include "mt_rpc.h"
#include "OSAL.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_znp.h"
#if defined ZCL_KEY_ESTABLISH
#include "zcl_key_establish.h"
#include "zcl_se.h"
#endif
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

const uint8 nullAddr[Z_EXTADDR_LEN] = { 0,0,0,0,0,0,0,0 };

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
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

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static associated_devices_t assocDevT;
static BindingEntry_t bindEntryT;

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static uint8 zapUtilParseAssocDevT(uint8 *pBuf);
static uint8 zapUtilParseBindEntryT(uint8 *pBuf);
#if defined ZCL_KEY_ESTABLISH
static void zapUtilParseKeyInd(uint8 *pBuf);
#endif

#if defined (ZAP_UTIL_FUNC)
/**************************************************************************************************
 * @fn          zapUtilProcessIncoming
 *
 * @brief       This function processes the UTIL sub-system response from the ZNP.
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
void zapUtilProcessIncoming(uint8 port, uint8 *pBuf)
{
  uint8 cmd1 = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  switch (cmd1)
  {
  case MT_UTIL_SYNC_REQ:
    zapGotSync(port);
    break;

#if defined ZCL_KEY_ESTABLISH
  case MT_UTIL_ZCL_KEY_ESTABLISH_IND:
    zapUtilParseKeyInd(pBuf);
    break;
#endif

  default:
    break;
  }
}
#endif

/**************************************************************************************************
 * @fn          AddrMgrEntryLookupNwk
 *
 * @brief       Lookup entry based on NWK address.
 *
 * input parameters
 *
 * @param       entry
 *                ::nwkAddr - [in] NWK address
 *
 * output parameters
 *
 * @param       entry
 *                ::extAddr - [out] EXT address
 *
 * @return      uint8 - success(TRUE:FALSE)
 **************************************************************************************************
 */
uint8 AddrMgrEntryLookupNwk(AddrMgrEntry_t* entry)
{
  uint8 args[2] = { LO_UINT16(entry->nwkAddr), HI_UINT16(entry->nwkAddr) };

  if (SUCCESS == zapUtilReq(MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP, entry->extAddr, args))
  {
    return ((osal_memcmp(nullAddr, entry->extAddr, Z_EXTADDR_LEN)) ? FALSE : TRUE);
  }
  else
  {
    (void)osal_memset(entry->extAddr, 0, Z_EXTADDR_LEN);
    return FALSE;
  }
}

/**************************************************************************************************
 * @fn          AddrMgrExtAddrLookup
 *
 * @brief       Lookup EXT address using the NWK address.
 *
 * input parameters
 *
 * @param       nwkAddr - [in] NWK address
 *
 * output parameters
 *
 * @param       extAddr - [out] EXT address
 *
 * @return      uint8 - success(TRUE:FALSE)
 **************************************************************************************************
 */
uint8 AddrMgrExtAddrLookup(uint16 nwkAddr, uint8* extAddr)
{
  uint8 args[2] = { LO_UINT16(nwkAddr), HI_UINT16(nwkAddr) };

  if (SUCCESS == zapUtilReq(MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP, extAddr, args))
  {
    return ((osal_memcmp(nullAddr, extAddr, Z_EXTADDR_LEN)) ? FALSE : TRUE);
  }
  else
  {
    (void)osal_memset(extAddr, 0, Z_EXTADDR_LEN);
    return FALSE;
  }
}

/**************************************************************************************************
 * @fn          AddrMgrExtAddrSet
 *
 * @brief       Set destination address to source address or empty{0x00}.
 *
 * input parameters
 *
 * @param       dstExtAddr - Pointer to the buffer to which to copy.
 * @param       srcExtAddr - Pointer to the buffer from which to copy.
 *
 * output parameters
 *
 * @param       dstExtAddr - Pointer to the buffer to which to copy.
 *
 * @return      None.
 **************************************************************************************************
 */
void AddrMgrExtAddrSet(uint8 *dstExtAddr, uint8 *srcExtAddr)
{
  if ( srcExtAddr != NULL )
  {
    osal_cpyExtAddr( dstExtAddr, srcExtAddr );
  }
  else
  {
    osal_memset( dstExtAddr, 0x00, Z_EXTADDR_LEN );
  }
}

/**************************************************************************************************
 * @fn          AssocCount()
 *
 * @brief       Counts the number of entries in the device list.
 *
 * input parameters
 *
 * @param       startRelation - Device relation to start counting at.
 * @param       endRelation - Device relation to end counting at.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of devices within the relation parameters.
 **************************************************************************************************
 */
uint16 AssocCount(uint8 startRelation, uint8 endRelation)
{
  uint16 count = 0;
  uint8 pBuf[2] = { startRelation, endRelation };

  if (SUCCESS == zapUtilReq(MT_UTIL_ASSOC_COUNT, pBuf, NULL))
  {
    count = BUILD_UINT16(pBuf[0], pBuf[1]);
  }

  return count;
}

/**************************************************************************************************
 * @fn          AssocFindDevice()
 *
 * @brief       Finds Nth active entry in the device list.
 *
 * input parameters
 *
 * @param       number - Device index where 0 = first.
 *
 * output parameters
 *
 * None.
 *
 * @return      A pointer to associated_devices_t if device found, NULL if operation failed.
 **************************************************************************************************
 */
associated_devices_t *AssocFindDevice(uint16 number)
{
  uint8 pBuf[sizeof(associated_devices_t)];

  /* ZNP can only handle one-byte index */
  if ( number < 256 )
  {
    uint8 index = number;

    if ((SUCCESS == zapUtilReq(MT_UTIL_ASSOC_FIND_DEVICE, pBuf, &index)) &&
        (SUCCESS == zapUtilParseAssocDevT(pBuf)))
    {
      return &assocDevT;
    }
  }

  return NULL;
}

/**************************************************************************************************
 * @fn          AssocGetWithShort()
 *
 * @brief       Search the Device list using shortAddr.
 *
 * input parameters
 *
 * @param       shortAddr - look for this short address
 *
 * output parameters
 *
 * None.
 *
 * @return      A pointer to associated_devices_t if device found, NULL if operation failed.
 **************************************************************************************************
 */
associated_devices_t *AssocGetWithShort(uint16 shortAddr)
{
  uint8 pBuf[sizeof(associated_devices_t)];
  assocDevT.shortAddr = shortAddr;

  if ((SUCCESS == zapUtilReq(MT_UTIL_ASSOC_GET_WITH_ADDRESS, pBuf, NULL)) &&
      (SUCCESS == zapUtilParseAssocDevT(pBuf)))
  {
    return &assocDevT;
  }
  else
  {
    return NULL;
  }
}

/**************************************************************************************************
 * @fn          NLME_GetCoordShortAddr
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack accessor function. The
 *              concurrency of the information is assured by the ZAP task which queries the current
 *              values on a ZDO state change notification.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The ZigBee network address of the ZNP's parent device.
 **************************************************************************************************
 */
uint16 NLME_GetCoordShortAddr(void)
{
  return znpParent;
}

/**************************************************************************************************
 * @fn          NLME_GetExtAddr
 *
 * @brief       This function will return a pointer to the ZNP's IEEE 64-bit address.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      Pointer to the ZNP 64-bit address.
 **************************************************************************************************
 */
uint8 *NLME_GetExtAddr(void)
{
  return znpIEEE;
}

/**************************************************************************************************
 * @fn          NLME_GetShortAddr
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack accessor function. The
 *              concurrency of the information is assured by the ZAP task which queries the current
 *              values on a ZDO state change notification.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The ZigBee network address of the ZNP.
 **************************************************************************************************
 */
uint16 NLME_GetShortAddr(void)
{
  return znpAddr;
}

/**************************************************************************************************
 * @fn          NLME_RemoveChild
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to remove a child
 *              device and optionally re-use its network address.
 *
 * input parameters
 *
 * @param       newRate = number of milliseconds to do next poll.
 *                        0 will turn off the polling.
 *                        1 will do a one time poll.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 *
void NLME_RemoveChild(uint8 *extAddr, uint8 dealloc)
{
  (void)zapUtilReq(MT_NLME_REMOVE_CHILD, extAddr, &dealloc);
}*/

/**************************************************************************************************
 * @fn          NLME_SetPollRate
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to immediately set
 *              the poll rate of the ZNP.
 *
 * input parameters
 *
 * @param       newRate = number of milliseconds to do next poll.
 *                        0 will turn off the polling.
 *                        1 will do a one time poll.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void NLME_SetPollRate(uint16 newRate)
{
  (void)znp_nv_write(ZCD_NV_POLL_RATE, 0, 2, (uint8 *)(&newRate));
}

/**************************************************************************************************
 * @fn          NLME_PermitJoiningRequest
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to immediately set
 *              the coordinator device to permit devices to join its network for a fixed period.
 *
 * input parameters
 *
 * @param       PermitDuration - Length of time to allow for associates
 *                               0x00 - macAssociationPermit = FALSE
 *                               0xFF - macAssociationPermit = TRUE
 *                               0x01 - 0xFE number of seconds to remain
 *                                  TRUE.
 *
 * output parameters
 *
 * None.
 *
 * @return      ZStatus_t
 **************************************************************************************************
 */
ZStatus_t NLME_PermitJoiningRequest( byte PermitDuration )
{
  uint8 rtrn;
  zAddrType_t dstAddrLocal;

  dstAddrLocal.addr.shortAddr = NLME_GetShortAddr();
  dstAddrLocal.addrMode = afAddr16Bit;

  rtrn = ZDP_MgmtPermitJoinReq( &dstAddrLocal, PermitDuration, TRUE, FALSE );

  return (ZStatus_t)rtrn;
}

/**************************************************************************************************
 * @fn      bindAddEntry()
 *
 * @brief   This function is a ZAP-side proxy for a Z-Stack function to
 *          Add an entry to the binding table
 *
 * input parameters
 *
 * @param       srcEpInt - source endpoint
 * @param       dstAddr - destination Address
 * @param       dstEpInt - destination endpoint
 * @param       numClusterIds - number of cluster Ids in the list
 * @param       clusterIds - pointer to the Object ID list
 *
 * output parameters
 *
 * None.
 *
 * @return  pointer to binding table entry, NULL if not added
 **************************************************************************************************
 */
BindingEntry_t *bindAddEntry( byte srcEpInt,
                              zAddrType_t *dstAddr, byte dstEpInt,
                              byte numClusterIds, uint16 *clusterIds )
{
  uint8 *pData = NULL;
  uint8 *pBuf;
  uint8 len;

  len = 1 + sizeof(zAddrType_t) + 1 + 1 + ( numClusterIds * sizeof(uint16) );

  pData = (uint8 *)osal_mem_alloc( len );

  if ( pData != NULL )
  {
    pBuf = pData;

    *pBuf++ = srcEpInt;
    *pBuf++ = dstAddr->addrMode;

    if ( dstAddr->addrMode == Addr64Bit )
    {
      osal_cpyExtAddr( pBuf, dstAddr->addr.extAddr );
    }
    else
    {
      *pBuf++ = LO_UINT16( dstAddr->addr.shortAddr );
      *pBuf++ = HI_UINT16( dstAddr->addr.shortAddr );
      pBuf += ( Z_EXTADDR_LEN - 2);
    }

    *pBuf++ = dstEpInt;
    *pBuf++ = numClusterIds;
    osal_memcpy( pBuf, clusterIds, numClusterIds * sizeof(uint16) );

    if ( ( SUCCESS == zapUtilReq( MT_UTIL_BIND_ADD_ENTRY, pData, &len ) ) &&
         ( SUCCESS == zapUtilParseBindEntryT( pData ) ) )
    {
      osal_mem_free( pData );

      return &bindEntryT;
    }

    osal_mem_free( pData );
  }

  return NULL;
}

#if defined ZCL_KEY_ESTABLISH
/**************************************************************************************************
 * @fn          zclGeneral_KeyEstablish_InitiateKeyEstablishment
 *
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to initiate
 *              key establishment with partner device.
 *
 * input parameters
 *
 * @param       appTaskID - task ID of the application that initates the key establish
 * @param       partnerAddr - short address and endpoint of the partner to establish key with
 * @param       seqNum - pointer to the sequence number of application (ZCL)
 *
 * output parameters
 *
 * None.
 *
 * @return      ZStatus_t ZSuccess or ZFailure
 **************************************************************************************************
 */
ZStatus_t zclGeneral_KeyEstablish_InitiateKeyEstablishment(uint8 appTaskID,
                                                           afAddrType_t *partnerAddr, uint8 seqNum)
{
  uint8 req[12], rtrn;

  req[0] = appTaskID;
  req[1] = seqNum;
  req[2] = partnerAddr->endPoint;
  req[3] = partnerAddr->addrMode;
  if (afAddr64Bit == partnerAddr->addrMode)
  {
    (void)osal_memcpy(req+4, partnerAddr->addr.extAddr, Z_EXTADDR_LEN);
  }
  else
  {
    req[4] = LO_UINT16(partnerAddr->addr.shortAddr);
    req[5] = HI_UINT16(partnerAddr->addr.shortAddr);
  }

  zapPhyWait(zapAppPort, ZCL_KEY_EST_INIT_EST_WAIT);
  rtrn = zapUtilReq(MT_UTIL_ZCL_KEY_EST_INIT_EST, req, NULL);
  zapPhyWait(zapAppPort, 0);
  return rtrn;
}

/**************************************************************************************************
 * @fn          zclGeneral_KeyEstablishment_ECDSASign
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to creates an
 *              ECDSA signature of a message digest.
 *
 * input parameters
 *
 * @param       input - input data buffer
 * @param       inputLen - byte length of the input buffer
 * @param       output - output buffer (21x2 bytes: SE_PROFILE_SIGNATURE_LENGTH).
 *
 * output parameters
 *
 * None.
 *
 * @return      ZStatus_t - success.
 **************************************************************************************************
 */
ZStatus_t zclGeneral_KeyEstablishment_ECDSASign(uint8 *input, uint8 inputLen, uint8 *output)
{
  uint8 rtrn, *pBuf;

#if defined SECURE
  if (SE_PROFILE_SIGNATURE_LENGTH > inputLen)
  {
    pBuf = output;
    (void)osal_memcpy(pBuf, input, inputLen);
  }
  else
  {
    pBuf = input;
  }

  rtrn = zapUtilReq(MT_UTIL_ZCL_KEY_EST_SIGN, pBuf, &inputLen);

  if (SE_PROFILE_SIGNATURE_LENGTH <= inputLen)
  {
    (void)osal_memcpy(output, pBuf, SE_PROFILE_SIGNATURE_LENGTH);
  }
#endif

  return rtrn;
}
#endif

#if SECURE
/**************************************************************************************************
 * @fn          APSME_LinkKeyDataGet
 *
 *
 * @brief       This function is a ZAP-side proxy for a Z-Stack function to get the
 *              APS Link Key NV ID for a specified extended address.
 *
 * input parameters
 *
 * @param       extAddr - [in] EXT address
 *
 * output parameters
 *
 * @param       data    - [out] pKeyNvId
 *
 * @return      ZStatus_t
 **************************************************************************************************
 */
ZStatus_t APSME_LinkKeyNVIdGet(uint8* extAddr, uint16 *pKeyNvId)
{
  // query for the APS Link Key NV id
  if (SUCCESS == zapUtilReq(MT_UTIL_APSME_LINK_KEY_NV_ID_GET, extAddr, (uint8 *)pKeyNvId))
  {
    return ZSuccess;
  }
  else
  {
    return ZNwkUnknownDevice;
  }
}


/******************************************************************************
 * @fn          APSME_IsLinkKeyValid
 *
 * @brief       Verifies if Link Key in NV has been set.
 *
 * @param       extAddr - [in] EXT address
 *
 * @return      TRUE - Link Key has been established
 *              FALSE - Link Key in NV has default value.
 */
uint8 APSME_IsLinkKeyValid(uint8* extAddr)
{
  APSME_LinkKeyData_t *pKeyData = NULL;
  uint8 nullKey[SEC_KEY_LEN];
  uint8 status = FALSE;
  uint8 ret;

  // initialize default vealue to compare to
  osal_memset(nullKey, 0x00, SEC_KEY_LEN);

  pKeyData = (APSME_LinkKeyData_t *)osal_mem_alloc(sizeof(APSME_LinkKeyData_t));

  if (pKeyData != NULL)
  {
    ret = zapUtilReq(MT_UTIL_APSME_LINK_KEY_DATA_GET, extAddr, (uint8 *)pKeyData);

    if (ret == SUCCESS)
    {
      // if stored key is different than default value, then a key has been established
      if (!osal_memcmp(pKeyData, nullKey, SEC_KEY_LEN))
      {
        status = TRUE;
      }
    }
  }

  return status;
}
#endif

/**************************************************************************************************
 * @fn          zapUtilReq
 *
 * @brief       This function packs and sends an RPC NWK request.
 *
 * input parameters
 *
 * @param       cmd - A valid NWK command.
 * @param       req - A buffer containing the contents of the request/response, or NULL.
 * @param       args - Valid argument(s) corresponding to the NWK command.
 *
 * output parameters
 *
 * @param       req - The buffer filled with the contents or success of a response.
 * @param       args - The buffer filled with the contents or success of a response.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 zapUtilReq(uint8 cmd, uint8 *req, uint8 *args)
{
  uint8 len, cmd0 = (uint8)MT_RPC_CMD_SREQ;
  uint8 rtrn = SUCCESS;
  uint8 *pBuf;

  if (DEV_STATE_INVALID <= devState)
  {
    return FAILURE;
  }

  switch (cmd)
  {
  // SREQ's to ZNP.

  case MT_UTIL_ASSOC_GET_WITH_ADDRESS:
    len = Z_EXTADDR_LEN + 2;
    break;

  case MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP:
  case MT_UTIL_ASSOC_COUNT:
    len = 2;
    break;

#if SECURE
  case MT_UTIL_APSME_LINK_KEY_DATA_GET:
    len = Z_EXTADDR_LEN;
    break;

  case MT_UTIL_APSME_LINK_KEY_NV_ID_GET:
    len = Z_EXTADDR_LEN;
    break;
#endif

  case MT_UTIL_ASSOC_FIND_DEVICE:
    len = 1;
    break;

#if defined ZCL_KEY_ESTABLISH
  case MT_UTIL_ZCL_KEY_EST_INIT_EST:
    len = 12;
    break;

  case MT_UTIL_ZCL_KEY_EST_SIGN:
    len = *args +1;
    break;
#endif

  case MT_UTIL_BIND_ADD_ENTRY:
    len = *args;
    break;

  // AREQ's to ZNP.

  case MT_UTIL_SYNC_REQ:
    cmd0 = (uint8)MT_RPC_CMD_AREQ;
    len = 0;
    break;

  default:
    return FAILURE;
  }
  cmd0 |= (uint8)MT_RPC_SYS_UTIL;

  if (NULL == (pBuf = zap_msg_allocate(len, cmd0, cmd)))
  {
    return FAILURE;
  }

  switch (cmd)
  {
  // SREQ's to ZNP.

  case MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP:
    pBuf[0] = *args++;
    pBuf[1] = *args;
    break;

#if SECURE
  case MT_UTIL_APSME_LINK_KEY_DATA_GET:
    (void)osal_memcpy(pBuf, req, Z_EXTADDR_LEN);
    break;

  case MT_UTIL_APSME_LINK_KEY_NV_ID_GET:
    (void)osal_memcpy(pBuf, req, Z_EXTADDR_LEN);
    break;
#endif

  case MT_UTIL_ASSOC_COUNT:
    (void)osal_memcpy(pBuf, req, 2);
    break;

  case MT_UTIL_ASSOC_FIND_DEVICE:
    pBuf[0] = *args;
    break;

  case MT_UTIL_ASSOC_GET_WITH_ADDRESS:
    if (NULL == args)
    {
      (void)osal_memset(pBuf, 0, Z_EXTADDR_LEN);
    }
    else
    {
      (void)osal_memcpy(pBuf, args, Z_EXTADDR_LEN);
    }
    pBuf[Z_EXTADDR_LEN] = LO_UINT16(assocDevT.shortAddr);
    pBuf[Z_EXTADDR_LEN+1] = HI_UINT16(assocDevT.shortAddr);
    break;

#if defined ZCL_KEY_ESTABLISH
  case MT_UTIL_ZCL_KEY_EST_INIT_EST:
    (void)osal_memcpy(pBuf, req, 12);
    break;

  case MT_UTIL_ZCL_KEY_EST_SIGN:
    *pBuf = *args;
    (void)osal_memcpy(pBuf+1, req, *args);
    break;
#endif

  case MT_UTIL_BIND_ADD_ENTRY:
    (void)osal_memcpy(pBuf, req, len);
    break;

  // AREQ's to ZNP.

  default:
    break;
  }

  if (zapPhySend(zapAppPort, pBuf) == FAILURE)
  {
    zap_msg_deallocate(&pBuf);
    return FAILURE;
  }

  switch (cmd)
  {
  // SREQ's to ZNP.

  case MT_UTIL_ADDRMGR_NWK_ADDR_LOOKUP:
    (void)osal_memcpy(req, pBuf, Z_EXTADDR_LEN);
    break;

#if SECURE
  case MT_UTIL_APSME_LINK_KEY_DATA_GET:
    if (SUCCESS == (rtrn = *pBuf))
    {
      APSME_LinkKeyData_t *pData = (APSME_LinkKeyData_t *)args;
      uint8 *ptr = pBuf+1;

      // copy key data
      (void)osal_memcpy(pData->key, ptr, SEC_KEY_LEN);
      ptr += SEC_KEY_LEN;
      pData->txFrmCntr = BUILD_UINT32(ptr[0], ptr[1], ptr[2], ptr[3]);
      ptr += 4;
      pData->rxFrmCntr = BUILD_UINT32(ptr[0], ptr[1], ptr[2], ptr[3]);
    }
    break;

  case MT_UTIL_APSME_LINK_KEY_NV_ID_GET:
    if (SUCCESS == (rtrn = *pBuf))
    {
      uint16 *pNvId = (uint16 *)args;
      uint8 *ptr = pBuf+1;

      *pNvId = BUILD_UINT16(ptr[0], ptr[1]);
    }
    break;
#endif

  case MT_UTIL_ASSOC_COUNT:
    (void)osal_memcpy(req, pBuf, 2);
    break;

  case MT_UTIL_ASSOC_FIND_DEVICE:
  case MT_UTIL_ASSOC_GET_WITH_ADDRESS:
    (void)osal_memcpy(req, pBuf, sizeof(associated_devices_t));
    break;

    case MT_UTIL_BIND_ADD_ENTRY:
    (void)osal_memcpy(req, pBuf, sizeof(BindingEntry_t));
    break;

#if defined ZCL_KEY_ESTABLISH
  case MT_UTIL_ZCL_KEY_EST_SIGN:
#if defined SECURE
    (void)osal_memcpy(req, pBuf+1, SE_PROFILE_SIGNATURE_LENGTH);
#endif
  case MT_UTIL_ZCL_KEY_EST_INIT_EST:
    rtrn = *pBuf;
    break;
#endif

  // AREQ's to ZNP.

  default:
    break;
  }

  zap_msg_deallocate(&pBuf);
  return rtrn;
}

/**************************************************************************************************
 * @fn          zapUtilParseAssocDevT
 *
 * @brief       This function parses a packed associated_devices_t.
 *
 * input parameters
 *
 * @param       pBuf - A buffer containing a packed associated_devices_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS if the parsed shortAddr is not invalid, otherwise FAILURE.
 **************************************************************************************************
 */
static uint8 zapUtilParseAssocDevT(uint8 *pBuf)
{
  assocDevT.shortAddr = BUILD_UINT16(pBuf[0], pBuf[1]);
  assocDevT.addrIdx = BUILD_UINT16(pBuf[2], pBuf[3]);
  pBuf += 4;
  assocDevT.nodeRelation = *pBuf++;
  assocDevT.devStatus = *pBuf++;
  assocDevT.assocCnt = *pBuf++;
  assocDevT.age = *pBuf++;
  assocDevT.linkInfo.txCounter = *pBuf++;
  assocDevT.linkInfo.txCost = *pBuf++;
  assocDevT.linkInfo.rxLqi = *pBuf++;
  assocDevT.linkInfo.inKeySeqNum = *pBuf++;
  assocDevT.linkInfo.inFrmCntr = BUILD_UINT32(pBuf[0], pBuf[1], pBuf[2], pBuf[3]);
  assocDevT.linkInfo.txFailure = BUILD_UINT16(pBuf[4], pBuf[5]);

  return ((INVALID_NODE_ADDR != assocDevT.shortAddr) ? SUCCESS : FAILURE);
}

/**************************************************************************************************
 * @fn          zapUtilParseBindEntryT
 *
 * @brief       This function parses a packed BindingEntry_t.
 *
 * input parameters
 *
 * @param       pBuf - A buffer containing a packed BindingEntry_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS if the parsed dstIdx is valid, otherwise FAILURE.
 **************************************************************************************************
 */
static uint8 zapUtilParseBindEntryT(uint8 *pBuf)
{
  bindEntryT.srcEP = *pBuf++;
  bindEntryT.dstGroupMode = *pBuf++;
  bindEntryT.dstIdx = BUILD_UINT16(pBuf[0], pBuf[1]);
  pBuf += 2;
  bindEntryT.dstEP = *pBuf++;
  bindEntryT.numClusterIds = *pBuf++;

  if ( bindEntryT.numClusterIds > MAX_BINDING_CLUSTER_IDS )
  {
    bindEntryT.numClusterIds = MAX_BINDING_CLUSTER_IDS;
  }

  osal_memcpy( bindEntryT.clusterIdList, pBuf, bindEntryT.numClusterIds * sizeof(uint16));

  return ((INVALID_NODE_ADDR != bindEntryT.dstIdx) ? SUCCESS : FAILURE);
}

#if defined ZCL_KEY_ESTABLISH
/**************************************************************************************************
 * @fn          zapUtilParseKeyInd
 *
 * @brief       This function parses a packed keyEstablishmentInd_t.
 *
 * input parameters
 *
 * @param       pBuf - A buffer containing a packed keyEstablishmentInd_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
static void zapUtilParseKeyInd(uint8 *pBuf)
{
  keyEstablishmentInd_t *pInd;

  // Send osal message to the application.
  if (NULL != (pInd = (keyEstablishmentInd_t *)osal_msg_allocate(sizeof(keyEstablishmentInd_t))))
  {
    pInd->hdr.event = pBuf[1];
    pInd->hdr.status = pBuf[2];
    pInd->waitTime = pBuf[3];
    pInd->keyEstablishmentSuite = BUILD_UINT16(pBuf[4], pBuf[5]);
    osal_msg_send(pBuf[0], (uint8 *)pInd);
  }
}
#endif

/**************************************************************************************************
*/
