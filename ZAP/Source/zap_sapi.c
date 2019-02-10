/**************************************************************************************************
    Filename:       zap_sapi.c
    Revised:        $Date: 2011-05-20 18:49:45 -0700 (Fri, 20 May 2011) $
    Revision:       $Revision: 26058 $

    Description:

    This file declares the ZNP Application Processor SAPI API functions.


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
#include "sapi.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "ZComDef.h"

#if defined (ZAP_SAPI_FUNC)
/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

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

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static uint8 sapiReq(uint8 cmd, uint8 arg, uint8 *req, uint8 len);

/**************************************************************************************************
 * @fn          zapSapiProcessIncoming
 *
 * @brief       This function processes the SAPI sub-system response from the ZNP.
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
void zapSapiProcessIncoming(uint8 port, uint8 *pBuf)
{
  uint8 cmd1 = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  switch (cmd1)
  {
  case MT_SAPI_START_CNF:
    break;

  case MT_SAPI_BIND_CNF:
    break;

  case MT_SAPI_ALLOW_BIND_CNF:
    break;

  case MT_SAPI_SEND_DATA_CNF:
    break;

  case MT_SAPI_READ_CFG_RSP:
    break;

  case MT_SAPI_FIND_DEV_CNF:
    break;

  case MT_SAPI_DEV_INFO_RSP:
    break;

  case MT_SAPI_RCV_DATA_IND:
    break;

  default:
    break;
  }
}

/******************************************************************************
 * @fn          zb_PermitJoiningRequest
 *
 * @brief       The zb_PermitJoiningRequest function is used to control the
 *              joining permissions and thus allow or disallow new devices from
 *              joining the network.
 *
 * @param       destination - The destination parameter indicates the address
 *                            of the device for which the joining permissions
 *                            should be set. This is usually the local device
 *                            address or the special broadcast address that denotes
 *                            all routers and coordinator ( 0xFFFC ). This way
 *                            the joining permissions of a single device or the
 *                            whole network can be controlled.
 *              timeout -  Indicates the amount of time in seconds for which
 *                         the joining permissions should be turned on.
 *                         If timeout is set to 0x00, the device will turn off the
 *                         joining permissions indefinitely. If it is set to 0xFF,
 *                         the joining permissions will be turned on indefinitely.
 *
 *
 * @return      ZB_SUCCESS or a failure code
 *
 */
uint8 zb_PermitJoiningRequest(uint16 destination, uint8 timeout)
{
  uint8 rtrn = ZB_FAILURE;
  uint8 *pBuf = zap_msg_allocate(3, ((uint8)MT_RPC_SYS_SAPI | (uint8)MT_RPC_CMD_SREQ),
                                                                     MT_SAPI_PMT_JOIN_REQ);
  if (pBuf != NULL)
  {
    pBuf[0] = LO_UINT16(destination);
    pBuf[1] = HI_UINT16(destination);
    pBuf[2] = timeout;

    if (zapPhySend(zapAppPort, pBuf) == SUCCESS)
    {
      rtrn = pBuf[0];
    }

    zap_msg_deallocate(&pBuf);
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          zb_GetDeviceInfo
 *
 * @brief       The zb_GetDeviceInfo function retrieves a Device Information Property.
 *
 * input parameters
 *
 * @param       param - The identifier for the device information.
 *              pValue - A buffer to hold the device information.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zb_GetDeviceInfo(uint8 param, void *pValue)
{
  uint8 pBuf[Z_EXTADDR_LEN+1];
  osal_memset(pBuf, 0xFF, 9);

  if (sapiReq(MT_SAPI_GET_DEV_INFO_REQ, param, pBuf, (Z_EXTADDR_LEN+1)) == FAILURE)
  {
    return;
  }

  if (NULL != pValue)
  {
    uint8 len;

    switch (param)
    {
    case ZB_INFO_DEV_STATE:
    case ZB_INFO_CHANNEL:
      len = sizeof(uint8);
      break;

    case ZB_INFO_IEEE_ADDR:
    case ZB_INFO_PARENT_IEEE_ADDR:
    case ZB_INFO_EXT_PAN_ID:
      len = Z_EXTADDR_LEN;
      break;

    case ZB_INFO_SHORT_ADDR:
    case ZB_INFO_PARENT_SHORT_ADDR:
    case ZB_INFO_PAN_ID:
      len = sizeof(uint16);
      break;

    default:
      len = 0;
      break;
    }

    // The first data byte is just the 'param' echo.
    (void)osal_memcpy((uint8 *)pValue, pBuf+1, len);
  }
}

/**************************************************************************************************
 * @fn          sapiReq
 *
 * @brief       This function packs and sends an RPC SAPI request.
 *
 * input parameters
 *
 * @param       cmd - A valid SAPI command.
 * @param       arg - A valid argument corresponding to the SAPI command.
 * @param       req - A buffer containing the contents of the request, or NULL.
 * @param       len - The non-zero length of a non-NULL req buffer, or NULL.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
static uint8 sapiReq(uint8 cmd, uint8 arg, uint8 *req, uint8 len)
{
  uint8 cmd0 = (MT_SAPI_SYS_RESET == cmd) ? ((uint8)MT_RPC_SYS_SAPI | (uint8)MT_RPC_CMD_AREQ):
                                            ((uint8)MT_RPC_SYS_SAPI | (uint8)MT_RPC_CMD_SREQ);
  uint8 *pBuf = zap_msg_allocate((uint8)(len + 2), cmd0, cmd);
  uint8 rtrn = FAILURE;

  if (NULL != pBuf)
  {
    pBuf[0] = arg;
    pBuf[1] = len;
    if (NULL != req)
    {
      (void)osal_memcpy(pBuf+2, req, len);
    }

    if (((rtrn = zapPhySend(zapAppPort, pBuf)) == SUCCESS) && (NULL != req))
    {
      (void)osal_memcpy(req, pBuf, ZAP_MSG_LEN(pBuf));
    }

    zap_msg_deallocate(&pBuf);
  }

  return rtrn;
}

#endif
/**************************************************************************************************
*/
