/**************************************************************************************************
    Filename:       zap_znp.h
    Revised:        $Date: 2010-07-28 17:08:02 -0700 (Wed, 28 Jul 2010) $
    Revision:       $Revision: 23200 $

    Description:

    This file declares the ZAP proxy functionality to the ZNP.


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
#ifndef ZAP_ZNP_H
#define ZAP_ZNP_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

// The key establishment process is expected to take no less than 20 seconds.
#if !defined ZCL_KEY_EST_INIT_EST_WAIT
#define ZCL_KEY_EST_INIT_EST_WAIT  30000
#endif

// Especially when connected to the CC2531 USB virtual COM port, a soft reset is preferred so as
// not to lock-up the default windows drivers.
#define ZNP_RESET_HARD  0
#define ZNP_RESET_SOFT  1

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

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
uint8 znp_afRegisterExtended(endPointDesc_t *epDesc);

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
uint8 znp_nv_read(uint16 id, uint8 ndx, uint8 len, void *buf);

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
uint8 znp_nv_write(uint16 id, uint8 ndx, uint8 len, void *buf);

/*********************************************************************
 * @fn          znp_ZDO_RegisterForZDOMsg
 *
 * @brief       This function is the ZAP proxy to the ZNP ZDO_RegisterForZDOMsg() functionality.
 *
 * @param       clusterID - What message?
 *
 * @return      ZSuccess - successful, ZMemError if not
 */
ZStatus_t znp_ZDO_RegisterForZDOMsg(uint16 clusterID);

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
void znpSystemReset(uint8 type);

#endif
/**************************************************************************************************
*/
