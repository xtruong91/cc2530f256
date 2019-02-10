/**************************************************************************************************
  Filename:       zap_zmac.c
  Revised:        $Date: 2013-10-21 10:01:10 -0700 (Mon, 21 Oct 2013) $
  Revision:       $Revision: 35731 $

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

#include "hal_board.h"
#include "mt.h"
#include "mt_rpc.h"
#include "mt_sys.h"
#include "zap_app.h"
#include "zap_znp.h"
#include "zmac.h"

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

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          ZMacGetReq
 *
 * @brief       This function proxies the Z-Stack ZMacGetReq function across the RPC.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC message buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      Status.
 **************************************************************************************************
 */
uint8 ZMacGetReq(uint8 attr, uint8 *value)
{
  uint8 rtrn = ZMacSuccess;

  switch (attr)
  {
  case ZMacExtAddr:
    (void)osal_memcpy(value, &aExtendedAddress, Z_EXTADDR_LEN);
    break;

  case ZMacRxOnIdle:
    *value = 3;  // Any value other than TRUE or FALSE signals a Get vice Set ZMacRxOnIdle.
    (void)zapSysStackTune(STK_RX_ON_IDLE, value);
    break;

  default:
    rtrn = ZInvalidParameter;
    break;
  }

  return rtrn;
}

/**************************************************************************************************
 * @fn          ZMacSetReq
 *
 * @brief       This function proxies the Z-Stack ZMacSetReq function across the RPC.
 *
 * input parameters
 *
 * @param       pBuf - Pointer to the RPC message buffer.
 *
 * output parameters
 *
 * None.
 *
 * @return      Status.
 **************************************************************************************************
 */
uint8 ZMacSetReq(uint8 attr, uint8 *value)
{
  uint8 rtrn = ZMacSuccess;

  switch (attr)
  {
  case ZMacExtAddr:
    (void)znp_nv_write(ZCD_NV_EXTADDR, 0, Z_EXTADDR_LEN, value);
    (void)osal_memcpy(&aExtendedAddress, value, Z_EXTADDR_LEN);
    break;

  case ZMacRxOnIdle:
    zapSysStackTune(STK_RX_ON_IDLE, value);
    break;

  case ZMacPhyTransmitPowerSigned:
    zapSysStackTune(STK_TX_PWR, value);
    break;

  default:
    rtrn = ZInvalidParameter;
    break;
  }

  return rtrn;
}

/**************************************************************************************************
*/
