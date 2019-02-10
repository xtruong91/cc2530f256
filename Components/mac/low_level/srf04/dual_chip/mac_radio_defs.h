/**************************************************************************************************
  Filename:       mac_radio_defs.h
  Revised:        $Date: 2014-06-03 09:28:17 -0700 (Tue, 03 Jun 2014) $
  Revision:       $Revision: 38773 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2014 Texas Instruments Incorporated. All rights reserved.

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

#ifndef MAC_RADIO_DEFS_H
#define MAC_RADIO_DEFS_H


/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_mac_cfg.h"
#include "mac_dualchip.h"
#include "mac_dualchip_tx.h"
#include "mac_mcu_timer.h"
#include "mac_spi.h"
#include "mac_spec.h"


/* ------------------------------------------------------------------------------------------------
 *                                    Target Specific Defines
 * ------------------------------------------------------------------------------------------------
 */

/* strobe command registers */
#define SNOP                        0x00
#define IBUFLD                      0x02
#define SIBUFEX                     0x03
#define SSAMPLECCA                  0x04
#define SRES                        0x0F
#define MEMRD                       0x10
#define MEMWR                       0x20
#define RXBUF                       0x30
#define RXBUFCP                     0x38
#define RXBUFMOV                    0x32
#define TXBUF                       0x3A
#define TXBUFCP                     0x3E
#define RANDOM                      0x3C
#define SXOSCON                     0x40
#define STXCAL                      0x41
#define SRXON                       0x42
#define STXON                       0x43
#define STXONCCA                    0x44
#define SRFOFF                      0x45
#define SXOSCOFF                    0x46
#define SFLUSHRX                    0x47
#define SFLUSHTX                    0x48
#define SACK                        0x49
#define SACKPEND                    0x4A
#define SNACK                       0x4B
#define SRXMASKBITSET               0x4C
#define SMRMASKBITCLR               0x4D
#define RXMASKAND                   0x4E
#define RXMASKOR                    0x4F
#define MEMCP                       0x50
#define MEMCPR                      0x52
#define MEMXCP                      0x54
#define MEMXWR                      0x56
#define BCLR                        0x58
#define BSET                        0x59
#define CTR_UCTR                    0x60
#define CBCMAC                      0x64
#define UCBCMAC                     0x66
#define CCM                         0x68
#define UCCM                        0x6A
#define ECB                         0x70
#define ECBO                        0x72
#define ECBX                        0x74
#define ECBXO                       0x76
#define INC                         0x78
#define ABORT                       0x7F
#define REGRD                       0x80
#define REGWR                       0xC0

/* configuration registers */
#define FRMFILT0                    0x00
#define FRMFILT1                    0x01
#define SRCMATCH                    0x02
#define SRCSHORTEN0                 0x04
#define SRCSHORTEN1                 0x05
#define SRCSHORTEN2                 0x06
#define SRCEXTEN0                   0x08
#define SRCEXTEN1                   0x09
#define SRCEXTEN2                   0x0A
#define FRMCTRL0                    0x0C
#define FRMCTRL1                    0x0D
#define RXENABLE0                   0x0E
#define RXENABLE1                   0x0F
#define EXCFLAG0                    0x10
#define EXCFLAG1                    0x11
#define EXCFLAG2                    0x12
#define EXCMASKA0                   0x14
#define EXCMASKA1                   0x15
#define EXCMASKA2                   0x16
#define EXCMASKB0                   0x18
#define EXCMASKB1                   0x19
#define EXCMASKB2                   0x1A
#define EXCBINDX0                   0x1C
#define EXCBINDX1                   0x1D
#define EXCBINDY0                   0x1E
#define EXCBINDY1                   0x1F
#define GPIOCTRL0                   0x20
#define GPIOCTRL1                   0x21
#define GPIOCTRL2                   0x22
#define GPIOCTRL3                   0x23
#define GPIOCTRL4                   0x24
#define GPIOCTRL5                   0x25
#define GPIOPOLARITY                0x26
#define GPIOCTRL                    0x28
#define DPUCON                      0x2A
#define DPUSTAT                     0x2C
#define FREQCTRL                    0x2E
#define FREQTUNE                    0x2F
#define TXPOWER                     0x30
#define TXCTRL                      0x31
#define FSMSTAT0                    0x32
#define FSMSTAT1                    0x33
#define FIFOPCTRL                   0x34
#define FSMCTRL                     0x35
#define CCACTRL0                    0x36
#define CCACTRL1                    0x37
#define RSSI                        0x38
#define RSSISTAT                    0x39
#define TXFIFO_BUF                  0x3A
#define RXFIRST                     0x3C
#define RXFIFOCNT                   0x3E
#define TXFIFOCNT                   0x3F
#define CHIPID                      0x40
#define VERSION                     0x42
#define EXTCLOCK                    0x44
#define MDMCTRL0                    0x46
#define MDMCTRL1                    0x47
#define FREQEST                     0x48
#define RXCTRL                      0x4A
#define FSCTRL                      0x4C
#define FSCAL0                      0x4E
#define FSCAL1                      0x4F
#define FSCAL2                      0x50
#define FSCAL3                      0x51
#define AGCCTRL0                    0x52
#define AGCCTRL1                    0x53
#define AGCCTRL2                    0x54
#define AGCCTRL3                    0x55
#define ADCTEST0                    0x56
#define ADCTEST1                    0x57
#define ADCTEST2                    0x58
#define MSMTEST0                    0x5A
#define MSMTEST1                    0x5B
#define DACTEST0                    0x5C
#define DACTEST1                    0x5D
#define ATEST                       0x5E
#define DACTEST2                    0x5F
#define PTEST0                      0x60
#define PTEST1                      0x61
#define RESERVED                    0x62
#define DPUTEST                     0x7A
#define ACTTEST                     0x7C
#define RAM_BIST_CTRL               0x7E

/* RAM memory spaces */
#define RAM_PANID                   0x3F2
#define RAM_SHORTADR                0x3F4
#define RAM_IEEEADR                 0x3EA

/* Src matching table */
#define SRC_MATCH_TABLE             0x380
#define SRCSHORTPENDEN0             0x3E7
#define SRCEXTPENDEN0               0x3E4
#define SRCRESINDEX                 0x3E3

/* status byte */
#define XOSC16M_STABLE              (1 << 7)
#define RSSI_VALID                  (1 << 6)
#define EXCEPTION_A                 (1 << 5)
#define EXCEPTION_B                 (1 << 4)
#define DPU_H_ACTIVE                (1 << 3)
#define DPU_L_ACTIVE                (1 << 2)
#define TX_ACTIVE                   (1 << 1)
#define RX_ACTIVE                   (1 << 0)

/* CHIPID */
#define CHIPID_RESET_VALUE          0x84

/* CHIP VERSION */
#define REV_A                       0

/* FRMCTRL0 */
#define FRMCTRL0_RESET_VALUE        0x40
#define RX_MODE(x)                  ((x) << 2)
#define RX_MODE_INFINITE_RECEPTION  RX_MODE(2)
#define RX_MODE_NORMAL_OPERATION    RX_MODE(0)
#define AUTOACK_BV                  (1 << 5)

/* FRMCTRL1 */
#define FRMCTRL1_RESET_VALUE        0x01
#define PENDING_OR_BV               (1 << 2)

/* FIFOPCTRL */
#define FIFOP_THR_RESET_VALUE       64

/* FRMFILT0 */
#define FRMFILT0_RESET_VALUE        0x0D
#define MAX_FRAME_VERSION_MASK      (3 << 2)
#define MAX_FRAME_VERSION           (1 << 3)
#define PAN_COORDINATOR_BV          (1 << 1)
#define ADR_DECODE_BV               (1 << 0)

/* FREQCTRL */
#define FREQCTRL_BASE_VALUE         0
#define FREQCTRL_FREQ_2405MHZ       11

/* FSMSTAT1 */
#define SAMPLED_CCA_BV              (1 << 3)

/* TXPOWER */
#define TXPOWER_BASE_VALUE          0

/* FSMSTATE */
#define FSM_FFCTRL_STATE_RX_MASK    0x3F
#define FSM_FFCTRL_STATE_RX_INF     31

/* SRCMATCH */
#define SRCMATCH_RESET_VALUE        0x07
#define SRC_MATCH_EN                (1 << ( 0 ))
#define AUTOPEND                    (1 << ( 1 ))
#define PEND_DATAREQ_ONLY           (1 << ( 2 ))

/* SRCRESINDEX */
#define AUTOPEND_RES                (1 << ( 6 ))

/* FRMFILT1 */
#define FRMFILT1_RESET_VALUE        0x78

/* MDMCTRL1 */
#define MDMCTRL1_RESET_VALUE        0x2E
#define CORR_THR_MASK               0x1F

/* GPIO directional control */
#define GPIO_DIR_RADIO_INPUT        0x80
#define GPIO_DIR_RADIO_OUTPUT       0x00

/* GPIO command strobes */
#define GPIO_CMD_SIBUFEX            0x00
#define GPIO_CMD_SRXMASKBITCLR      0x01
#define GPIO_CMD_SRXMASKBITSET      0x02
#define GPIO_CMD_SRXON              0x03
#define GPIO_CMD_SSAMPLECCA         0x04
#define GPIO_CMD_SACK               0x05
#define GPIO_CMD_SACKPEND           0x06
#define GPIO_CMD_SNACK              0x07
#define GPIO_CMD_STXON              0x08
#define GPIO_CMD_STXONCCA           0x09
#define GPIO_CMD_SFLUSHRX           0x0A
#define GPIO_CMD_SFLUSHTX           0x0B
#define GPIO_CMD_SRXFIFOPOP         0x0C
#define GPIO_CMD_STXCAL             0x0D
#define GPIO_CMD_SRFOFF             0x0E
#define GPIO_CMD_SXOSCOFF           0x0F

/* GPIO exceptions */
#define EXCEPTION_ECG_EXT_CLOCK     0x00
#define EXCEPTION_RF_IDLE           0x01
#define EXCEPTION_TX_FRM_DONE       0x02
#define EXCEPTION_TX_ACK_DONE       0x03
#define EXCEPTION_TX_UNDERFLOW      0x04
#define EXCEPTION_TX_OVERFLOW       0x05
#define EXCEPTION_RX_UNDERFLOW      0x06
#define EXCEPTION_RX_OVERFLOW       0x07
#define EXCEPTION_RXENABLE_ZERO     0x08
#define EXCEPTION_RX_FRM_DONE       0x09
#define EXCEPTION_RX_FRM_ACCEPTED   0x0A
#define EXCEPTION_SRC_MATCH_DONE    0x0B
#define EXCEPTION_SRC_MATCH_FOUND   0x0C
#define EXCEPTION_FIFOP             0x0D
#define EXCEPTION_SFD               0x0E
#define EXCEPTION_DPU_DONE_L        0x0F
#define EXCEPTION_DPU_DONE_H        0x10
#define EXCEPTION_MEMADDR_ERROR     0x11
#define EXCEPTION_USAGE_ERROR       0x12
#define EXCEPTION_OPERAND_ERROR     0x13
#define EXCEPTION_SPI_ERROR         0x14
#define EXCEPTION_RF_NO_LOCK        0x15
#define EXCEPTION_RX_FRM_ABORTED    0x16
#define EXCEPTION_RXBUFMOV_TIMEOUT  0x17
#define EXCEPTION_UNUSED            0x18
#define EXCEPTION_CHANNEL_A         0x21
#define EXCEPTION_CHANNEL_B         0x22
#define EXCEPTION_CHANNEL_COMP_A    0x23
#define EXCEPTION_CHANNEL_COMP_B    0x24
#define EXCEPTION_RFC_FIFO          0x27
#define EXCEPTION_RFC_FIFOP         0x28
#define EXCEPTION_RFC_CCA           0x29
#define EXCEPTION_RFC_SFD_SYNC      0x2A
#define EXCEPTION_RFC_SNIFFER_CLK   0x31
#define EXCEPTION_RFC_SNIFFER_DATA  0x32

/* GPIO exception bit in EXCFLAG0 */
#define RF_IDLE_BIT                 (1<<0)
#define TX_FRM_DONE_BIT             (1<<1)
#define TX_ACK_DONE_BIT             (1<<2)
#define TX_UNDERFLOW_BIT            (1<<3)
#define TX_OVERFLOW_BIT             (1<<4)
#define RX_UNDERFLOW_BIT            (1<<5)
#define RX_OVERFLOW_BIT             (1<<6)
#define RXENABLE_ZERO_BIT           (1<<7)

/* GPIO exception bit in EXCFLAG1 */
#define RX_FRM_DONE_BIT             (1<<0)
#define RX_FRM_ACCEPTED_BIT         (1<<1)
#define SRC_MATCH_DONE_BIT          (1<<2)
#define SRC_MATCH_FOUND_BIT         (1<<3)
#define FIFOP_BIT                   (1<<4)
#define SFD_BIT                     (1<<5)
#define DPU_DONE_L_BIT              (1<<6)
#define DPU_DONE_H_BIT              (1<<7)

/* GPIO exception bit in EXCFLAG2 */
#define MEMADDR_ERROR_BIT           (1<<0)
#define USAGE_ERROR_BIT             (1<<1)
#define OPERAND_ERROR_BIT           (1<<2)
#define SPI_ERROR_BIT               (1<<3)
#define RF_NO_LOCK_BIT              (1<<4)
#define RX_FRM_ABORTED_BIT          (1<<5)
#define RXBUFMOV_TIMEOUT_BIT        (1<<6)
#define UNUSED_BIT                  (1<<7)

/* Clear GPIO exception in EXCFLAG0 */
#define RF_IDLE_FLAG                ((1<<0)^0xFF)
#define TX_FRM_DONE_FLAG            ((1<<1)^0xFF)
#define TX_ACK_DONE_FLAG            ((1<<2)^0xFF)
#define TX_UNDERFLOW_FLAG           ((1<<3)^0xFF)
#define TX_OVERFLOW_FLAG            ((1<<4)^0xFF)
#define RX_UNDERFLOW_FLAG           ((1<<5)^0xFF)
#define RX_OVERFLOW_FLAG            ((1<<6)^0xFF)
#define RXENABLE_ZERO_FLAG          ((1<<7)^0xFF)

/* Clear GPIO exception in EXCFLAG1 */
#define RX_FRM_DONE_FLAG            ((1<<0)^0xFF)
#define RX_FRM_ACCEPTED_FLAG        ((1<<1)^0xFF)
#define SRC_MATCH_DONE_FLAG         ((1<<2)^0xFF)
#define SRC_MATCH_FOUND_FLAG        ((1<<3)^0xFF)
#define FIFOP_FLAG                  ((1<<4)^0xFF)
#define SFD_FLAG                    ((1<<5)^0xFF)
#define DPU_DONE_L_FLAG             ((1<<6)^0xFF)
#define DPU_DONE_H_FLAG             ((1<<7)^0xFF)

/* Clear GPIO exception in EXCFLAG2 */
#define MEMADDR_ERROR_FLAG          ((1<<0)^0xFF)
#define USAGE_ERROR_FLAG            ((1<<1)^0xFF)
#define OPERAND_ERROR_FLAG          ((1<<2)^0xFF)
#define SPI_ERROR_FLAG              ((1<<3)^0xFF)
#define RF_NO_LOCK_FLAG             ((1<<4)^0xFF)
#define RX_FRM_ABORTED_FLAG         ((1<<5)^0xFF)
#define RXBUFMOV_TIMEOUT_FLAG       ((1<<6)^0xFF)
#define UNUSED_FLAG                 ((1<<7)^0xFF)


/* ------------------------------------------------------------------------------------------------
 *                                    Unique Radio Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_RADIO_CC2520


/* ------------------------------------------------------------------------------------------------
 *                                    Common Radio Defines
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_RADIO_CHANNEL_DEFAULT               11
#define MAC_RADIO_CHANNEL_INVALID               0xFF
#define MAC_RADIO_TX_POWER_INVALID              0xFF
#define MAC_RADIO_TX_POWER_DEFAULT              0x32


#define MAC_RADIO_RECEIVER_SENSITIVITY_DBM      -98 /* dBm */
#define MAC_RADIO_RECEIVER_SATURATION_DBM       6   /* dBm */

#define MAC_RADIO_RSSI_OFFSET                   HAL_MAC_RSSI_OFFSET

#if (defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590)
#define MAC_RADIO_RSSI_LNA_HGM_OFFSET           HAL_MAC_RSSI_LNA_HGM_OFFSET
#define MAC_RADIO_RSSI_LNA_LGM_OFFSET           HAL_MAC_RSSI_LNA_LGM_OFFSET
#define MAC_RADIO_RSSI_LNA_OFFSET(x)            st(  x += macRxGain ? MAC_RADIO_RSSI_LNA_HGM_OFFSET : MAC_RADIO_RSSI_LNA_LGM_OFFSET; )
#else
#define MAC_RADIO_RSSI_LNA_OFFSET(x)
#endif


#define MAC_RADIO_RX_TX_PROP_DELAY_MIN_USEC     3.076  /* usec */
#define MAC_RADIO_RX_TX_PROP_DELAY_MAX_USEC     3.284  /* usec */


/* register value table offset */
#define MAC_RADIO_DEFS_TBL_TXPWR_FIRST_ENTRY   0
#define MAC_RADIO_DEFS_TBL_TXPWR_LAST_ENTRY    1
#define MAC_RADIO_DEFS_TBL_TXPWR_ENTRIES       2


/* ------------------------------------------------------------------------------------------------
 *                                    Common Radio Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MAC_RADIO_MCU_INIT()                          macDualchipSystemInit()

#define MAC_RADIO_TURN_ON_POWER()                     macDualchipTurnOnRadioPower()
#define MAC_RADIO_TURN_OFF_POWER()                    macDualchipTurnOffRadioPower()
#define MAC_RADIO_TURN_ON_OSC()                       macDualchipTurnOnRadioOscillator()
#define MAC_RADIO_TURN_OFF_OSC()                      macDualchipTurnOffRadioOscillator()

#define MAC_RADIO_RX_FIFO_HAS_OVERFLOWED()            (HAL_MAC_READ_FIFOP_PIN() && !HAL_MAC_READ_FIFO_PIN())
#define MAC_RADIO_RX_FIFO_IS_EMPTY()                  (!HAL_MAC_READ_FIFO_PIN() && !HAL_MAC_READ_FIFOP_PIN())

#define MAC_RADIO_RSSI_IS_VALID()                     (macSpiCmdStrobe(SNOP) & RSSI_VALID)

#define MAC_RADIO_SET_RX_THRESHOLD(x)                 macSpiWriteReg(FIFOPCTRL, (x)-1)
#define MAC_RADIO_RX_IS_AT_THRESHOLD()                HAL_MAC_READ_FIFOP_PIN()
#define MAC_RADIO_ENABLE_RX_THRESHOLD_INTERRUPT()     HAL_MAC_ENABLE_FIFOP_INT()
#define MAC_RADIO_DISABLE_RX_THRESHOLD_INTERRUPT()    HAL_MAC_DISABLE_FIFOP_INT()
#define MAC_RADIO_CLEAR_RX_THRESHOLD_INTERRUPT_FLAG() HAL_MAC_CLEAR_FIFOP_INT_FLAG()

#define MAC_RADIO_TX_ACK()                            macSpiWriteReg(FRMCTRL1, FRMCTRL1_RESET_VALUE & ~PENDING_OR_BV)
#define MAC_RADIO_TX_ACK_PEND()                       macSpiWriteReg(FRMCTRL1, FRMCTRL1_RESET_VALUE | PENDING_OR_BV)
#define MAC_RADIO_RX_ON()                             macSpiCmdStrobe(SRXON)
#define MAC_RADIO_RXTX_OFF()                          macSpiCmdStrobe(SRFOFF)
#define MAC_RADIO_FLUSH_TX_FIFO()                     macSpiCmdStrobe(SFLUSHTX)
#define MAC_RADIO_FLUSH_RX_FIFO()                     st( macSpiCmdStrobe(SFLUSHRX); macSpiCmdStrobe(SFLUSHRX); )
#define MAC_RADIO_TX_IS_ACTIVE()                      (macSpiCmdStrobe(SNOP) & TX_ACTIVE)

#define MAC_RADIO_READ_RX_FIFO(pData,len)             macSpiReadRxFifo(pData, len)
#define MAC_RADIO_WRITE_TX_FIFO(pData,len)            macSpiWriteTxFifo(pData, len)

#define MAC_RADIO_SET_PAN_COORDINATOR(b)              st( if (b) { macDualchipOrFRMFILT0(PAN_COORDINATOR_BV); } else { macDualchipAndFRMFILT0(PAN_COORDINATOR_BV^0xFF); } )
#define MAC_RADIO_SET_CHANNEL(x)                      macSpiWriteReg(FREQCTRL, (FREQCTRL_BASE_VALUE + (FREQCTRL_FREQ_2405MHZ + 5 * ((x) - 11))))
#define MAC_RADIO_SET_TX_POWER(x)                     macSpiWriteReg(TXPOWER, (TXPOWER_BASE_VALUE | (x)))
#define MAC_RADIO_SELECT_BOOST_MODE(x)

#define MAC_RADIO_SELECT_PTABLE(PTABLE)               st( MAC_RADIO_SELECT_PTABLE_CC2520((PTABLE));)

#if defined MAC_RUNTIME_CC2591 || defined MAC_RUNTIME_CC2590

#define MAC_RADIO_SELECT_PTABLE_CC2520(PTABLE) \
  st( (PTABLE) = macRadioDefsTxPwrTables[macRadioDefsRefTableId >> 4]; )

#elif defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590

#define MAC_RADIO_SELECT_PTABLE_CC2520(PTABLE) \
  st( (PTABLE) = macRadioDefsTxPwrTables[0]; )

#else

#define MAC_RADIO_SELECT_PTABLE_CC2520(PTABLE) \
  st( (PTABLE) = macRadioDefsTxPwrBare; )

#endif
    
# define MAC_RADIO_SENSITIVITY_OFFSET(x)
# define MAC_RADIO_SATURATION_OFFSET(x) 

#define MAC_RADIO_SET_PAN_ID(x)                       macSpiWriteRamUint16(RAM_PANID, x)
#define MAC_RADIO_SET_SHORT_ADDR(x)                   macSpiWriteRamUint16(RAM_SHORTADR, x)
#define MAC_RADIO_SET_IEEE_ADDR(p)                    macSpiWriteRam(RAM_IEEEADR, p, 8)

#define MAC_RADIO_REQUEST_ACK_TX_DONE_CALLBACK()      /* nothing required */
#define MAC_RADIO_CANCEL_ACK_TX_DONE_CALLBACK()       /* nothing required */
    
#define MAC_RADIO_TURN_ON_AUTOPEND_DATAREQ_ONLY()     /* SRCMATCH_RESET_VALUE already includes this*/
 
#define MAC_RADIO_RANDOM_BYTE()                       macDualchipRandomByte()
#define MAC_RADIO_RANDOM_WORD()                       macDualchipRandomWord()

#define MAC_RADIO_TX_RESET()                          macDualchipTxReset()
#define MAC_RADIO_TX_PREP_CSMA_UNSLOTTED()            /* nothing required */
#define MAC_RADIO_TX_PREP_CSMA_SLOTTED()              macDualchipTxPrepCsmaSlotted()
#define MAC_RADIO_TX_PREP_SLOTTED()                   macDualchipTxPrepSlotted()
#define MAC_RADIO_TX_PREP_GREEN_POWER()               /* Green Power is not supported on CC2520 yet */
#define MAC_RADIO_TX_GO_CSMA()                        macDualchipTxGoCsma()
#define MAC_RADIO_TX_GO_SLOTTED_CSMA()                macDualchipTxGoSlottedCsma()
#define MAC_RADIO_TX_GO_SLOTTED()                     macDualchipTxGoSlotted()
#define MAC_RADIO_TX_GO_GREEN_POWER()                 /* Green Power is not supported on CC2520 yet */

#define MAC_RADIO_FORCE_TX_DONE_IF_PENDING()          /* nothing required */

#define MAC_RADIO_TX_REQUEST_ACK_TIMEOUT_CALLBACK()   macDualchipTxRequestAckTimeoutCallback()
#define MAC_RADIO_TX_CANCEL_ACK_TIMEOUT_CALLBACK()    macDualchipTxCancelAckTimeoutCallback()

#define MAC_RADIO_TIMER_TICKS_PER_USEC()              HAL_MAC_TIMER_TICKS_PER_USEC /* may be fractional */
#define MAC_RADIO_TIMER_TICKS_PER_BACKOFF()           ((uint16) (HAL_MAC_TIMER_TICKS_PER_USEC * MAC_SPEC_USECS_PER_BACKOFF + 0.5))
#define MAC_RADIO_TIMER_TICKS_PER_SYMBOL()            ((uint16) (HAL_MAC_TIMER_TICKS_PER_USEC * MAC_SPEC_USECS_PER_SYMBOL + 0.5))

#define MAC_RADIO_TIMER_CAPTURE()                     macMcuTimerTickCapture()
#define MAC_RADIO_TIMER_FORCE_DELAY(x)                macMcuTimerForceDelay(x)

#define MAC_RADIO_TIMER_SLEEP()                       st( if (macPib.beaconOrder == MAC_BO_NON_BEACON) \
                                                            HAL_MAC_TIMER_ROLLOVER_DISABLE_INTERRUPT(); )
#define MAC_RADIO_TIMER_WAKE_UP()                     st( if (macPib.beaconOrder == MAC_BO_NON_BEACON) \
                                                            HAL_MAC_TIMER_ROLLOVER_ENABLE_INTERRUPT(); )

#define MAC_RADIO_BACKOFF_COUNT()                     macMcuTimerBackoffCount
#define MAC_RADIO_BACKOFF_CAPTURE()                   macMcuTimerBackoffCapture()
#define MAC_RADIO_BACKOFF_SET_COUNT(x)                st( macMcuTimerBackoffCount = x; )
#define MAC_RADIO_BACKOFF_SET_COMPARE(x)              st( macMcuTimerBackoffCompare = x; )

#define MAC_RADIO_BACKOFF_COMPARE_CLEAR_INTERRUPT()   HAL_MAC_TIMER_ROLLOVER_CLEAR_INTERRUPT()
#define MAC_RADIO_BACKOFF_COMPARE_ENABLE_INTERRUPT()  HAL_MAC_TIMER_ROLLOVER_ENABLE_INTERRUPT()
#define MAC_RADIO_BACKOFF_COMPARE_DISABLE_INTERRUPT() HAL_MAC_TIMER_ROLLOVER_DISABLE_INTERRUPT()

#define MAC_RADIO_BACKOFF_SET_PERIOD(x)               st( macMcuTimerBackoffRollover = x; )
#define MAC_RADIO_BACKOFF_PERIOD_CLEAR_INTERRUPT()    /* nothing required */
#define MAC_RADIO_BACKOFF_PERIOD_ENABLE_INTERRUPT()   /* nothing required */
#define MAC_RADIO_BACKOFF_PERIOD_DISABLE_INTERRUPT()  /* nothing required */

#define MAC_RADIO_RECORD_MAX_RSSI_START()             macDualchipRecordMaxRssiStart()
#define MAC_RADIO_RECORD_MAX_RSSI_STOP()              macDualchipRecordMaxRssiStop()

#define MAC_RADIO_TURN_ON_RX_FRAME_FILTERING()        macDualchipOrFRMFILT0(ADR_DECODE_BV | MAX_FRAME_VERSION)
#define MAC_RADIO_TURN_OFF_RX_FRAME_FILTERING()       macDualchipAndFRMFILT0(ADR_DECODE_BV^0xFF)

#define MAC_RADIO_TURN_ON_AUTO_ACK()                  macSpiWriteReg(FRMCTRL0, (FRMCTRL0_RESET_VALUE | AUTOACK_BV));
#define MAC_RADIO_TURN_OFF_SRC_MATCH()                macSpiWriteReg(SRCMATCH, SRCMATCH_RESET_VALUE & ~SRC_MATCH_EN);
#define MAC_RADIO_TURN_ON_SRC_MATCH()                 macSpiWriteReg(SRCMATCH, SRCMATCH_RESET_VALUE);
#define MAC_RADIO_TURN_ON_AUTOPEND()                  macSpiWriteReg(SRCMATCH, SRCMATCH_RESET_VALUE);
#define MAC_RADIO_SRC_MATCH_RESULT()                  MAC_SrcMatchCheckResult()
#define MAC_RADIO_SRC_MATCH_RESINDEX(p)               macSpiReadRam(SRCRESINDEX, &(p), 1)

#define MAC_RADIO_TX_ACK_DONE_EXCEPTION(x)            ( x &  TX_ACK_DONE_BIT)
#define MAC_RADIO_TX_FRM_DONE_EXCEPTION(x)            ( x &  TX_FRM_DONE_BIT)
#define MAC_RADIO_TX_ACK_AND_TX_FRM_DONE_EXCEPTION(x) ((x & (TX_ACK_DONE_BIT | TX_FRM_DONE_BIT)) == (TX_ACK_DONE_BIT | TX_FRM_DONE_BIT))

#define MAC_RADIO_CLEAR_TX_ACK_DONE_PIN()             macSpiWriteReg(EXCFLAG0,  TX_ACK_DONE_FLAG)
#define MAC_RADIO_CLEAR_TX_FRM_DONE_PIN()             macSpiWriteReg(EXCFLAG0,  TX_FRM_DONE_FLAG)
#define MAC_RADIO_CLEAR_TX_ACK_AND_TX_FRM_DONE_PIN()  macSpiWriteReg(EXCFLAG0, (TX_ACK_DONE_FLAG & TX_FRM_DONE_FLAG))

#define MAC_RADIO_CANCEL_TX_ACK()                     macSpiCmdStrobe(SNACK)

#define MAC_RADIO_CONFIG_TRIGGER_CCA()                st( HAL_MAC_CONFIG_TRIGGER_PIN_AS_OUTPUT(); \
                                                          macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | EXCEPTION_RFC_CCA ); );
#define MAC_RADIO_SET_TRIGGER_STXON()                 st( macSpiWriteReg( GPIOCTRL5, GPIO_DIR_RADIO_INPUT  | GPIO_CMD_STXON ); )
#define MAC_RADIO_SET_TRIGGER_STXONCCA()              st( macSpiWriteReg( GPIOCTRL5, GPIO_DIR_RADIO_INPUT  | GPIO_CMD_STXONCCA ); )

#define MAC_RADIO_READ_SAMPLED_CCA()                  (SAMPLED_CCA_BV & macSpiReadReg( FSMSTAT1 ))

/* ------------Source Matching --------- */

#define MAC_RADIO_TURN_ON_PENDING_OR()                macSpiWriteReg(FRMCTRL1, FRMCTRL1_RESET_VALUE | PENDING_OR_BV)
#define MAC_RADIO_TURN_OFF_PENDING_OR()               macSpiWriteReg(FRMCTRL1, FRMCTRL1_RESET_VALUE & ~PENDING_OR_BV)
#define MAC_RADIO_READ_PENDING_OR()                   ( macSpiReadReg(FRMCTRL1) & PENDING_OR_BV )

#define MAC_RADIO_SRC_MATCH_GET_SHORTADDR_EN()        macSrcMatchGetShortAddrEnableBit()
#define MAC_RADIO_SRC_MATCH_GET_EXTADDR_EN()          macSrcMatchGetExtAddrEnableBit()
#define MAC_RADIO_SRC_MATCH_GET_SHORTADDR_PENDEN()    macSrcMatchGetShortAddrPendEnBit()
#define MAC_RADIO_SRC_MATCH_GET_EXTADDR_PENDEN()      macSrcMatchGetExtAddrPendEnBit()

#define MAC_RADIO_GET_SRC_SHORTEN(p)                  st( p[0] = macSpiReadReg(SRCSHORTEN0); \
                                                          p[1] = macSpiReadReg(SRCSHORTEN1); \
                                                          p[2] = macSpiReadReg(SRCSHORTEN2); )

#define MAC_RADIO_GET_SRC_EXTEN(p)                    st( p[0] = macSpiReadReg(SRCEXTEN0); \
                                                          p[1] = macSpiReadReg(SRCEXTEN1); \
                                                          p[2] = macSpiReadReg(SRCEXTEN2); )

#define MAC_RADIO_GET_SRC_SHORTPENDEN(p)              macSpiReadRam(SRCSHORTPENDEN0, (p), 3)
#define MAC_RADIO_GET_SRC_EXTENPEND(p)                macSpiReadRam(SRCEXTPENDEN0, (p), 3)

#define MAC_RADIO_SRC_MATCH_SET_SHORTPENDEN(p)        macSpiWriteRam(SRCSHORTPENDEN0, (p), 3)

#define MAC_RADIO_SRC_MATCH_SET_EXTPENDEN(p)          macSpiWriteRam(SRCEXTPENDEN0, (p), 3)

#define MAC_RADIO_SRC_MATCH_SET_SHORTEN(x)            st( macSpiWriteReg(SRCSHORTEN0, BREAK_UINT32((x), 0)); \
                                                          macSpiWriteReg(SRCSHORTEN1, BREAK_UINT32((x), 1)); \
                                                          macSpiWriteReg(SRCSHORTEN2, BREAK_UINT32((x), 2)); )

#define MAC_RADIO_SRC_MATCH_SET_EXTEN(x)              st( macSpiWriteReg(SRCEXTEN0, BREAK_UINT32((x), 0)); \
                                                          macSpiWriteReg(SRCEXTEN1, BREAK_UINT32((x), 1)); \
                                                          macSpiWriteReg(SRCEXTEN2, BREAK_UINT32((x), 2)); )

#define MAC_RADIO_SRC_MATCH_TABLE_WRITE(offset, p, len)  macSpiWriteRam( SRC_MATCH_TABLE + (offset), (p), (len))
#define MAC_RADIO_SRC_MATCH_TABLE_READ(offset, p, len)   macSpiReadRam( SRC_MATCH_TABLE + (offset), (p), (len))


#if defined(HAL_BOARD_F5438) || defined(HAL_BOARD_LM3S)
#define HAL_MAC_CONFIG_SNIFFER_MODE_PINS()            /* nothing required */
#else
#define HAL_MAC_CONFIG_SNIFFER_MODE_PINS()            st( HAL_MAC_CONFIG_TRIGGER_PIN_AS_INTPUT(); \
                                                          macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | EXCEPTION_RFC_SNIFFER_DATA ); \
                                                          macSpiWriteReg( GPIOCTRL5, GPIO_DIR_RADIO_OUTPUT | EXCEPTION_RFC_SNIFFER_CLK ); )
#endif


/* ----------- PA/LNA control ---------- */
#if (defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590)
#define HAL_PA_LNA_RX_HGM()                           st( macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | 0x7F ); macRxGain = 1; )
#define HAL_PA_LNA_RX_LGM()                           st( macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | 0x7E ); macRxGain = 0; )
#define HAL_PA_LNA_RX_GAIN()                          ( macRxGain )
#endif

/* Stubs for CC2591 compression workaround */
#define COMPRESSION_WORKAROUND_ON()                   /* Nothing */
#define COMPRESSION_WORKAROUND_OFF()                  /* Nothing */
#define COMPRESSION_WORKAROUND_RESET_RSSI()           /* Nothing */


/* ------------------------------------------------------------------------------------------------
 *                                    Common Radio Externs
 * ------------------------------------------------------------------------------------------------
 */
extern const uint8 CODE *const CODE macRadioDefsTxPwrTables[];
extern const uint8 CODE macRadioDefsTxPwrBare[];


/* ------------------------------------------------------------------------------------------------
 *                              Transmit Power Setting Configuration
 * ------------------------------------------------------------------------------------------------
 */

/*
 *  To use actual register values for setting power level, delete the 'x' from the following #define.
 *  In this case, the value passed to the set power function will be written directly to the
 *  LOWER BYTE of TXCTRL.  Characterized values for this register can be found in the datasheet
 *  in the "power settings" table.
 *
 *  NOTE!  The table gives the full value for TXCTRL, for example the register value
 *        for -7 dbm is 0xA0EF.  Only the lower byte is written so, in this case, use 0xEF for
 *        the regsiter value.
 */
#define xHAL_MAC_USE_REGISTER_POWER_VALUES


/**************************************************************************************************
 */
#endif
