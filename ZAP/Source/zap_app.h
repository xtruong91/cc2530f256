/**************************************************************************************************
    Filename:       zap_app.h
    Revised:        $Date: 2010-12-01 15:31:18 -0800 (Wed, 01 Dec 2010) $
    Revision:       $Revision: 24529 $

    Description:

    This file declares the functionality of the ZNP Application Processor.


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
#ifndef ZAP_APP_H
#define ZAP_APP_H

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

//#define SYS_EVENT_MSG   0x8000
#define ZAP_PHY_SPI_EVT   0x4000
#define ZAP_PHY_UART_EVT  0x2000
#define ZAP_APP_TMR_EVT   0x1000           // ZNP monitoring task.
#define ZAP_APP_SYNC_EVT  0x0800
#define ZAP_APP_ZDO_STATE_CHANGE_EVT \
                          0x0400

#define ZAP_APP_TMR_DLY   1000             // Period of the ZNP monitoring task.
#if SECURE
#define ZAP_APP_JOIN_DLY  30000            // Holdoff monitoring task longer while joining SECURE.
#else
#define ZAP_APP_JOIN_DLY  10000            // Holdoff monitoring task while joining.
#endif

#if !defined ZAP_APP_SYNC_DLY
#define ZAP_APP_SYNC_DLY  100
#endif

#if !defined ZAP_APP_ZDO_STATE_CHANGE_DLY
#define ZAP_APP_ZDO_STATE_CHANGE_DLY   ZAP_APP_SYNC_DLY
#endif

#include "ZDApp.h"
// Extended values for devStates_t.
#define DEV_STATE_INVALID      ((devStates_t)0xFC)
#define DEV_STATE_SYNC_LOST    ((devStates_t)(DEV_STATE_INVALID+1))
#define DEV_STATE_ZNP_LOST     ((devStates_t)(DEV_STATE_INVALID+2))
#define DEV_STATE_ZAP_LOST     ((devStates_t)(DEV_STATE_INVALID+3))

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

#define ZAP_MSG_LEN(PBUF)          (*((uint8 *)(PBUF) - MT_RPC_FRAME_HDR_SZ + MT_RPC_POS_LEN))
#define ZAP_SRSP_STATUS(PBUF)      (*((uint8 *)(PBUF) - MT_RPC_FRAME_HDR_SZ + MT_RPC_POS_DAT0))

/* ------------------------------------------------------------------------------------------------
 *                                           Global Variables
 * ------------------------------------------------------------------------------------------------
 */

extern uint8 zapTaskId;
extern uint8 zapAppPort;

// IEEE Address (64-bit Extended Address) of the ZNP device.
extern uint8 znpIEEE[8];
// ZigBee Network Address of the ZNP device.
extern uint16 znpAddr;
// ZigBee Network Address of the parent of the ZNP device.
extern uint16 znpParent;
// ZigBee Network PanId.
extern uint16 znpPanId;

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */

/**************************************************************************************************
 * @fn          zapInit
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
void zapInit(uint8 id);

/**************************************************************************************************
 * @fn          zapEvt
 *
 * @brief       This function is called to process the OSAL events for the task.
 *
 * input parameters
 *
 * @param       id - OSAL task Id.
 * @param       evts - OSAL events bit mask of pending events.
 *
 * output parameters
 *
 * None.
 *
 * @return      evts - OSAL events bit mask of unprocessed events.
 **************************************************************************************************
 */
uint16 zapEvt(uint8 id, uint16 evts);

/**************************************************************************************************
 * @fn          zapGotSync
 *
 * @brief       This function is invoked upon receipt of MT_SYS_RESET_IND or MT_UTIL_SYNC_REQ
 *              and would indicate that sync is (re-)gained with the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that got sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapGotSync(uint8 port);

/**************************************************************************************************
 * @fn          zapLostSync
 *
 * @brief       This function is invoked by the PHY drivers when sync is lost with the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that lost sync.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapLostSync(uint8 port);

/**************************************************************************************************
 * @fn          zapProcessFunc_t zapProcessFunctions.
 *
 * @brief       These functions process the corresponding sub-system incoming message from the ZNP.
 *
 * input parameters
 *
 * @param       port - Port Id corresponding to the ZNP that sent the message.
 * @param       pBuf - A pointer to the RPC message.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zapProcessIncoming(uint8 port, uint8 *pBuf);
#if defined (ZAP_SYS_FUNC)
void zapSysProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_MAC_FUNC)
void zapMacProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_NWK_FUNC)
void zapNwkProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_AF_FUNC)
void zapAfProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_ZDO_FUNC)
void zapZdoProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_SAPI_FUNC)
void zapSapiProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_UTIL_FUNC)
void zapUtilProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_DEBUG_FUNC)
void zapDebugProcessIncoming(uint8 port, uint8 *pBuf);
#endif
#if defined (ZAP_APP_FUNC)
void zapApsProcessIncoming(uint8 port, uint8 *pBuf);
#endif

void zapAfSync(void);
void zapCertSync(void);
#if defined INTER_PAN
void zapStubAPS_Sync(void);
#endif
void zapZdoSync(void);

/**************************************************************************************************
 * @fn          zap'Subsystem'Req
 *
 * @brief       These functions build and send the corresponding sub-system RPC request.
 *
 * input parameters
 *
 * @param       cmd - A valid Subsystem command.
 * @param       req - A buffer containing the contents of the request/response, or NULL.
 * @param       args - Valid argument(s) corresponding to the Subsystem command.
 *
 * output parameters
 *
 * None.
 *
 * @return      SUCCESS or FAILURE.
 **************************************************************************************************
 */
#if defined (ZAP_SYS_FUNC)
uint8 zapSysStackTune(uint8 cmd, uint8 *req);
uint8 zapSysReq(uint8 cmd, uint8 *req, uint8 *args);
#endif
uint8 zapUtilReq(uint8 cmd, uint8 *req, uint8 *args);

/**************************************************************************************************
 * @fn          zap_msg_allocate
 *
 * @brief       This function allocates and pre-fills a dynamic RPC message buffer.
 *
 * input parameters
 *
 * @param       len - length of the data to send via RPC (i.e. not including RPC overhead.)
 * @param       cmd0 - a valid logical OR of the mtRpcCmdType_t & mtRpcSysType_t.
 * @param       cmd1 - a valid command for the mtRpcSysType_t.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL on failure to allocate all memory necessary.
 *              A pointer to the data area (size 'len') of a valid dynamic memory buffer on success.
 **************************************************************************************************
 */
uint8 *zap_msg_allocate(uint8 len, uint8 cmd0, uint8 cmd1);

/**************************************************************************************************
 * @fn          zap_msg_deallocate
 *
 * @brief       This function deallocates an RPC message buffer created with zap_msg_allocate().
 *
 * input parameters
 *
 * @param       ppBuf - A pointer to the buffer pointer returned by zap_msg_allocate().
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void zap_msg_deallocate(uint8 **ppBuf);

#endif
/**************************************************************************************************
*/
