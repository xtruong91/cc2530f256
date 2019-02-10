/**************************************************************************************************
  Filename:       zap_stub_aps.c
  Revised:        $Date: 2010-01-18 12:20:01 -0800 (Mon, 18 Jan 2010) $
  Revision:       $Revision: 21540 $

  Description:

  This file defines the ZNP Application Processor API to the ZNP ZMAC layer.

  
  Copyright 2009 Texas Instruments Incorporated. All rights reserved.

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
#include "mt_af.h"
#include "mt_rpc.h"
#include "stub_aps.h"
#include "zap_app.h"
#include "zap_phy.h"

#if defined INTER_PAN
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

/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

static uint8 appEndPoint = 0;   // Application endpoint.

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

static uint8 StubAPS_InterPanReq(uint8 len, uint8 *req);

/**************************************************************************************************
 * @fn          StubAPS_InterPanReq
 *
 * @brief       This function proxies a request to the MT_AfInterPanCtl.
 *
 * input parameters
 *
 * @param       len - Length of the request.
 * @param       req - Inter-Pan control command request according to the InterPanParam_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if the Inter-PAN control command suceeded, FALSE otherwise
 **************************************************************************************************
 */
static uint8 StubAPS_InterPanReq(uint8 len, uint8 *req)
{
  uint8 *pBuf = zap_msg_allocate(len, (uint8)MT_RPC_SYS_AF | (uint8)MT_RPC_CMD_SREQ,
                                                             (uint8)MT_AF_INTER_PAN_CTL);
  uint8 rtrn = FALSE;

  if (NULL != pBuf)
  {
    (void)osal_memcpy(pBuf, req, len);
    zapPhySend(zapAppPort, pBuf);
    rtrn = *pBuf;
    zap_msg_deallocate(&pBuf);
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          StubAPS_InterPan
 *
 * @brief       This function checks to see if a PAN is an Inter-PAN.
 *
 * input parameters
 *
 * @param       panId - PAN ID
 * @param       endPoint - endpoint
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE if PAN is Inter-PAN, FALSE otherwise
 **************************************************************************************************
 */
uint8 StubAPS_InterPan( uint16 panId, uint8 endPoint )
{
  uint8 req[4];
  req[0] = InterPanChk;
  req[1] = LO_UINT16(panId);
  req[2] = HI_UINT16(panId);
  req[3] = endPoint;
  return !StubAPS_InterPanReq(4, req);
}

/**************************************************************************************************
 * @fn          StubAPS_RegisterApp
 *
 * @brief       This function registers the Application with the Stub APS layer.
 *
 *              NOTE: Since Stub APS messages don't include the application endpoint,
 *                    the application has to register its endpoint with Stub APS.
 *
 * input parameters
 *
 * @param       epDesc - application's endpoint descriptor
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void StubAPS_RegisterApp(endPointDesc_t *epDesc)
{
  uint8 req[2];
  req[0] = InterPanReg;
  req[1] = epDesc->endPoint;
  if (StubAPS_InterPanReq(2, req) == SUCCESS)
  {
    appEndPoint = epDesc->endPoint;
  }
}

/**************************************************************************************************
 * @fn          zapStubAPS_Sync
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
void zapStubAPS_Sync(void)
{
  endPointDesc_t *pEP;

  if ((appEndPoint != 0) && ((pEP = afFindEndPointDesc(appEndPoint))))
  {
    StubAPS_RegisterApp(pEP);
  }
}
#endif

/**************************************************************************************************
*/
