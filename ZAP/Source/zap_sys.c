/**************************************************************************************************
    Filename:       zap_sys.c
    Revised:        $Date: 2010-12-01 15:31:18 -0800 (Wed, 01 Dec 2010) $
    Revision:       $Revision: 24529 $

    Description:

    This file declares the ZNP Application Processor SYS API functions.


    Copyright 2009-2010 Texas Instruments Incorporated. All rights reserved.

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
#include "mt_sys.h"
#include "mt_version.h"
#include "OSAL.h"
#include "zap_app.h"
#include "zap_phy.h"
#include "ZComDef.h"

#if defined (ZAP_SYS_FUNC)
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

static void zapSysResetInd(uint8 port, uint8 *pBuf);

/**************************************************************************************************
 * @fn          zapSysResponseProcessing
 *
 * @brief       This function processes the SYS sub-system response from the ZNP.
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
void zapSysProcessIncoming(uint8 port, uint8 *pBuf)
{
  uint8 cmd1 = pBuf[MT_RPC_POS_CMD1];
  pBuf += MT_RPC_FRAME_HDR_SZ;

  switch (cmd1)
  {
  case MT_SYS_RESET_IND:
    zapSysResetInd(port, pBuf);
    break;

  case MT_SYS_OSAL_TIMER_EXPIRED:
    break;

  default:
    break;
  }
}

/**************************************************************************************************
 * @fn          zapSysStackTune
 *
 * @brief       This function packs and sends an RPC SYS request.
 *
 * input parameters
 *
 * @param       cmd - A valid MT_SYS_STACK_TUNE command from STK_Tune_t.
 * @param       req - A buffer containing the contents of the request/response, or NULL.
 *
 * output parameters
 *
 * @param       req - The buffer filled with the contents or success of a response.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 zapSysStackTune(uint8 cmd, uint8 *req)
{
  uint8 args[2], rtrn = SUCCESS;

  args[0] = cmd;
  args[1] = 1;

  switch (cmd)
  {
  case STK_TX_PWR:
  case STK_RX_ON_IDLE:
    break;

  default:
    rtrn = FAILURE;
    break;
  }

  zapSysReq(MT_SYS_STACK_TUNE, req, args);
  return rtrn;
}

/**************************************************************************************************
 * @fn          zapSysReq
 *
 * @brief       This function packs and sends an RPC SYS request.
 *
 * input parameters
 *
 * @param       cmd - A valid SYS command.
 * @param       req - A buffer containing the contents of the request/response, or NULL.
 * @param       args - Valid argument(s) corresponding to the SYS command.
 *
 * output parameters
 *
 * @param       req - The buffer filled with the contents or success of a response.
 * @param       args - The buffer filled with the contents or success of a response.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 zapSysReq(uint8 cmd, uint8 *req, uint8 *args)
{
  uint8 len, cmd0 = (uint8)MT_RPC_CMD_SREQ;
  uint8 *pBuf;

  switch (cmd)
  {
  // SREQ's to ZNP.

  case MT_SYS_PING:
  case MT_SYS_RANDOM:
  case MT_SYS_ADC_READ:
    len = 2;
    break;

  case MT_SYS_VERSION:
    len = sizeof(MTVersionString);
    break;

  case MT_SYS_SET_EXTADDR:
  case MT_SYS_GET_EXTADDR:
    len = Z_EXTADDR_LEN;
    break;

  case MT_SYS_OSAL_NV_READ:
    len = 3;
    break;

  case MT_SYS_OSAL_NV_WRITE:
    len = args[3] + 4;
    break;

  case MT_SYS_OSAL_START_TIMER:
    len = 3;
    break;

  case MT_SYS_OSAL_STOP_TIMER:
    len = 1;
    break;
    
  case MT_SYS_GPIO:
    break;

  case MT_SYS_STACK_TUNE:
    len = args[1] + 1;
    break;

  // AREQ's to ZNP.

  case MT_SYS_RESET_REQ:
    cmd0 = (uint8)MT_RPC_CMD_AREQ;
    len = 1;
    break;
    
  default:
    len = 8;  // Biggest return buffer size besides NV access.
    break;
  }
  cmd0 |= (uint8)MT_RPC_SYS_SYS;

  if (NULL == (pBuf = zap_msg_allocate(len, cmd0, cmd)))
  {
    return FAILURE;
  }

  switch (cmd)
  {
  case MT_SYS_SET_EXTADDR:
    (void)osal_memcpy(pBuf, req, Z_EXTADDR_LEN);
    break;

  case MT_SYS_OSAL_NV_WRITE:
    (void)osal_memcpy(pBuf+4, req, args[3]);
    pBuf[3] = args[3];  // NV Item len.
  case MT_SYS_OSAL_NV_READ:
  case MT_SYS_OSAL_START_TIMER:
    pBuf[2] = args[2];  // NV Item offset / Timer Period MSB.
  case MT_SYS_ADC_READ:
    pBuf[1] = args[1];  // NV Item Id MSB / Timer Period LSB / ADC Resolution.
  case MT_SYS_RESET_REQ:
  case MT_SYS_OSAL_STOP_TIMER:
    pBuf[0] = args[0];  // NV Item Id LSB / Timer Idx / ADC Chan.
    break;

  case MT_SYS_GPIO:
    break;

  case MT_SYS_STACK_TUNE:
    pBuf[0] = args[0];  // The MT_SYS_STACK_TUNE command from STK_Tune_t.
    (void)osal_memcpy(pBuf+1, req, args[1]);
    break;
    
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

  case MT_SYS_PING:
  case MT_SYS_RANDOM:
  case MT_SYS_ADC_READ:
    (void)osal_memcpy(req, pBuf, 2);
    break;

  case MT_SYS_VERSION:
    (void)osal_memcpy(req, pBuf, sizeof(MTVersionString));
    break;
    
  case MT_SYS_SET_EXTADDR:
    // It is not intuitive to have non-NULL args just for return value that should always be TRUE.
    break;

  case MT_SYS_OSAL_NV_WRITE:
  case MT_SYS_OSAL_START_TIMER:
  case MT_SYS_OSAL_STOP_TIMER:
    args[0] = pBuf[0];
    break;

  case MT_SYS_GET_EXTADDR:
    (void)osal_memcpy(req, pBuf, Z_EXTADDR_LEN);
    break;

  case MT_SYS_OSAL_NV_READ:
    if ((SUCCESS == (args[0] = pBuf[0])) && (args[3] == (args[1] = pBuf[1])))
    {
      (void)osal_memcpy(req, pBuf+2, args[1]);
    }
    else
    {
      args[0] = NV_OPER_FAILED;
    }
    break;

  case MT_SYS_GPIO:
    break;

  case MT_SYS_STACK_TUNE:
    *req = *pBuf;
    break;
    
  default:
    break;
  }

  zap_msg_deallocate(&pBuf);
  return SUCCESS;
}

/**************************************************************************************************
 * @fn          zapSysResetInd
 *
 * @brief       This function processes the SYS sub-system MT_SYS_RESET_IND response.
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
static void zapSysResetInd(uint8 port, uint8 *pBuf)
{
  //pBuf[0] = ResetReason();   /* Reason */
  //pBuf[1] = 0x00;            /* Transport Revision */
  //pBuf[2] = 0x00;            /* Product */
  //pBuf[3] = 0x00;            /* Major Revision */
  //pBuf[4] = 0x00;            /* Minor Revision */
  //pBuf[5] = 0x00;            /* Hardware Revision */
  (void)pBuf;
  zapGotSync(port);
}

#endif
/**************************************************************************************************
*/
