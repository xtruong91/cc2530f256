/**************************************************************************************************
  Filename:       mac_spi.h
  Revised:        $Date: 2007-05-01 18:23:25 -0700 (Tue, 01 May 2007) $
  Revision:       $Revision: 14169 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2007 Texas Instruments Incorporated. All rights reserved.

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

#ifndef MAC_SPI_H
#define MAC_SPI_H


/* ------------------------------------------------------------------------------------------------
 *                                         Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_types.h"
#include "mac_assert.h"


/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */
/* values used exclusively for ASSERT logic */
#define MAC_SPI_RADIO_POWER_OFF           0x00
#define MAC_SPI_RADIO_POWER_VREG_ON       0x01
#define MAC_SPI_RADIO_POWER_OSC_ON        0x02



#define MAC_SPI_READ_RX_FIFO_FLAG   0
#define MAC_SPI_WRITE_TX_FIFO_FLAG  1

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_SPI_WRITE_TX_FIFO(p,l)  macSpiFifoAccess(p, l, MAC_SPI_WRITE_TX_FIFO_FLAG)
#define MAC_SPI_READ_RX_FIFO(p,l)   macSpiFifoAccess(p, l, MAC_SPI_READ_RX_FIFO_FLAG)


void macSpiFifoAccess(uint8 * pData, uint8 len, uint8 writeFlag);


/* ------------------------------------------------------------------------------------------------
 *                                      Global Variables
 * ------------------------------------------------------------------------------------------------
 */
MAC_ASSERT_DECLARATION( extern uint8 macSpiRadioPower; )


/* ------------------------------------------------------------------------------------------------
 *                                         Prototypes
 * ------------------------------------------------------------------------------------------------
 */
void macSpiInit(void);

uint8 macSpiCmdStrobe(uint8 opCode);

uint8 macSpiReadReg(uint8 regAddr);
void macSpiWriteReg(uint8 regAddr, uint8 regValue);

void macSpiWriteRam(uint16 ramAddr, uint8 * pWriteData, uint8 len);
void macSpiWriteRamUint16(uint16 ramAddr, uint16 data);

void macSpiReadRam(uint16 ramAddr, uint8 *pReadData, uint8 len);

void macSpiWriteTxFifo(uint8 * pWriteData, uint8 len);
void macSpiReadRxFifo(uint8 * pReadData, uint8 len);

uint8 macSpiRandomByte(void);

void macSpiSendECBO(uint8 p, uint8 k, uint8 c, uint16 a);

/**************************************************************************************************
 */
#endif
