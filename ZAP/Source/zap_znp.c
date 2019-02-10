/**************************************************************************************************
    Filename:       zap_znp.c
    Revised:        $Date: 2012-04-16 12:57:57 -0700 (Mon, 16 Apr 2012) $
    Revision:       $Revision: 30211 $

    Description:

    This file defines the ZAP proxy functionality to the ZNP.


    Copyright 2009-2012 Texas Instruments Incorporated. All rights reserved.

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
#include "zap_app.h"
#include "zap_phy.h"
#include "zap_znp.h"
#include "ZDApp.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
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

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

/*********************************************************************
 * @fn          zgSetItem
 *
 * @brief       This function is called to initialize NV items based on zgItemTable
 *
 * @param       id - NV ID
 *              len - NV item length
 *              buf - pointer to the input buffer
 *
 * @return      none
 */
void zgSetItem(uint16 id, uint16 len, void *buf)
{
  (void)znp_nv_write(id, 0, len, buf);
}

/**************************************************************************************************
 * @fn          znp_afRegisterExtended
 *
 * @brief       This function is the ZAP proxy to the ZNP afRegisterExtended() functionality.
 *
 * input parameters
 *
 * @param       epDesc - A pointer to the EndPoint descriptor to register. The 'simpleDesc' member
 *                       of the EndPoint descriptor must be a non-NULL pointer to a valid
 *                       SimpleDescriptionFormat_t. If the 'simpleDesc' member is NULL, this is a
 *                       dummy EndPoint descriptor and does not need to be registered with the AF
 *                       on the ZNP.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 znp_afRegisterExtended(endPointDesc_t *epDesc)
{
  const uint8 len = ((epDesc->simpleDesc->AppNumInClusters) * 2) +
                    ((epDesc->simpleDesc->AppNumOutClusters) * 2) + 9;
  uint8 *pBuf = zap_msg_allocate(len, (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_SREQ,
                                      (uint8)MT_AF_REGISTER);
  uint8 rtrn = FAILURE;

  if (pBuf != NULL)
  {
    uint8 idx;

    *pBuf++ = (uint8)epDesc->endPoint;
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppProfId);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppProfId >> 8);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppDeviceId);
    *pBuf++ = (uint8)(epDesc->simpleDesc->AppDeviceId >> 8);
    *pBuf++ = epDesc->simpleDesc->AppDevVer;
    *pBuf++ = epDesc->latencyReq;
    *pBuf++ = epDesc->simpleDesc->AppNumInClusters;
    for (idx = 0; idx < epDesc->simpleDesc->AppNumInClusters; idx++)
    {
      *pBuf++ = LO_UINT16(epDesc->simpleDesc->pAppInClusterList[idx]);
      *pBuf++ = HI_UINT16(epDesc->simpleDesc->pAppInClusterList[idx]);
    }
    *pBuf++ = epDesc->simpleDesc->AppNumOutClusters;
    for (idx = 0; idx < epDesc->simpleDesc->AppNumOutClusters; idx++)
    {
      *pBuf++ = LO_UINT16(epDesc->simpleDesc->pAppOutClusterList[idx]);
      *pBuf++ = HI_UINT16(epDesc->simpleDesc->pAppOutClusterList[idx]);
    }

    pBuf -= len;
    zapPhySend(zapAppPort, pBuf);
    rtrn = ZAP_SRSP_STATUS(pBuf);
    zap_msg_deallocate(&pBuf);
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          znp_nv_read
 *
 * @brief       This function is the ZAP proxy to the ZNP uint8 osal_nv_read() functionality.
 *
 * input parameters
 *
 * @param       id  - Valid NV item Id.
 * @param       ndx - Index offset into item
 * @param       len - Length of data to read.
 * @param       *buf - Data to read.
 *
 * output parameters
 *
 * None.
 *
 * @return      ZSUCCESS if NV data was copied to the parameter 'buf'.
 *              Otherwise, NV_OPER_FAILED for failure.
 **************************************************************************************************
 */
uint8 znp_nv_read(uint16 id, uint8 ndx, uint8 len, void *buf)
{
  uint8 args[4];

  args[0] = LO_UINT16(id);
  args[1] = HI_UINT16(id);
  args[2] = ndx;  // TODO - ZNP MT_SYS_OSAL_NV_READ is only supporting uint8 here.
  args[3] = len;  // TODO - ZNP MT_SYS_OSAL_NV_READ is not even supporting len.

  zapSysReq(MT_SYS_OSAL_NV_READ, (uint8 *)buf, args);
  return args[0];
}

/**************************************************************************************************
 * @fn          znp_nv_write
 *
 * @brief       This function is the ZAP proxy to the ZNP uint8 osal_nv_write() functionality.
 *
 * input parameters
 *
 * @param       id  - Valid NV item Id.
 * @param       ndx - Index offset into item
 * @param       len - Length of data to write.
 * @param       *buf - Data to write.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS if successful.
 *              NV_ITEM_UNINIT if item did not exist in NV.
 *              NV_OPER_FAILED if failure.
 **************************************************************************************************
 */
uint8 znp_nv_write(uint16 id, uint8 ndx, uint8 len, void *buf)
{
  uint8 args[4];

  args[0] = LO_UINT16(id);
  args[1] = HI_UINT16(id);
  args[2] = ndx;  // TODO - ZNP MT_SYS_OSAL_NV_WRITE is only supporting uint8 here.
  args[3] = len;  // TODO - ZNP MT_SYS_OSAL_NV_WRITE is only supporting uint8 here.

  zapSysReq(MT_SYS_OSAL_NV_WRITE, (uint8 *)buf, args);
  return args[0];
}

/*********************************************************************
 * @fn          znp_ZDO_RegisterForZDOMsg
 *
 * @brief       This function is the ZAP proxy to the ZNP ZDO_RegisterForZDOMsg() functionality.
 *
 * @param       clusterID - What message?
 *
 * @return      ZSuccess - successful, ZMemError if not
 */
ZStatus_t znp_ZDO_RegisterForZDOMsg(uint16 clusterID)
{
  uint8 *pBuf, rtrn = ZMemError;

  // Note that this could be sent AREQ to free up the host (but then no status.)
  if ((pBuf = zap_msg_allocate(2, (uint8)MT_RPC_SYS_ZDO | (uint8)MT_RPC_CMD_SREQ,
                                          (uint8)MT_ZDO_MSG_CB_REGISTER)))
  {
    pBuf[0] = LO_UINT16(clusterID);
    pBuf[1] = HI_UINT16(clusterID);
    zapPhySend(zapAppPort, pBuf);
    rtrn = ZAP_SRSP_STATUS(pBuf);
    zap_msg_deallocate(&pBuf);
  }

  return (ZStatus_t)rtrn;
}

/**************************************************************************************************
 * @fn          znpSystemReset
 *
 * @brief       This function is the ZAP proxy to the ZNP SystemReset() functionality.
 *
 * input parameters
 *
 * @param       type - Hard or soft reset type: ZNP_RESET_HARD  or ZNP_RESET_SOFT.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void znpSystemReset(uint8 type)
{
  zapSysReq(MT_SYS_RESET_REQ, NULL, &type);
}

/**************************************************************************************************
*/
