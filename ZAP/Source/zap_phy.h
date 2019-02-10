/**************************************************************************************************
    Filename:       zap_phy.h
    Revised:        $Date: 2010-12-01 15:31:18 -0800 (Wed, 01 Dec 2010) $
    Revision:       $Revision: 24529 $

    Description:

    This file declares the functionality of the ZNP Application Processor Physical Link Layer.


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
#ifndef ZAP_PHY_H
#define ZAP_PHY_H

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board_cfg.h"
#include "ZComDef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */

#if !defined ZAP_PHY_SPI
#define ZAP_PHY_SPI  FALSE
#endif
#if !defined ZAP_PHY_UART
#define ZAP_PHY_UART  FALSE
#endif

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
 * @fn          zapPhyInit
 *
 * @brief       This function is the application's task initialization.
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
void zapPhyInit(void);

/**************************************************************************************************
 * @fn          zapPhyExec
 *
 * @brief       This function is called to execute the physical link layer background work.
 *
 * input parameters
 *
 * @param       evts - Bit mask of PHY events pending.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyExec(uint16 evts);

/**************************************************************************************************
 * @fn          zapPhyReset
 *
 * @brief       This function resets the ZNP slave.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to reset.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyReset(uint8 port);

/**************************************************************************************************
 * @fn          zapPhySend
 *
 * @brief       This function sends an RPC message buffer created with zap_msg_allocate().
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP to which to send the message.
 * @param       pBuf - A the buffer pointer returned by zap_msg_allocate().
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
uint8 zapPhySend(uint8 port, uint8 *pBuf);

/**************************************************************************************************
 * @fn          zapPhySync
 *
 * @brief       This function sets sync TRUE for the ZNP specified.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that is in sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhySync(uint8 port);

/**************************************************************************************************
 * @fn          zapPhyWait
 *
 * @brief       This function sets the SRSP wait timeout as specified by the parameter or to the
 *              specific medium's default value if the parameter is zero.
 *
 * input parameters
 *
 * @param       wait - the maximum wait delay for an SRSP.
 * @param       port - Port Id corresponding to the ZNP wait.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapPhyWait(uint8 port, uint16 wait);

#endif
/**************************************************************************************************
*/
