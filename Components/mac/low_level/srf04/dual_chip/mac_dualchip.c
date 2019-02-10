/**************************************************************************************************
  Filename:       mac_dualchip.c
  Revised:        $Date: 2012-07-09 15:12:28 -0700 (Mon, 09 Jul 2012) $
  Revision:       $Revision: 30875 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2011 Texas Instruments Incorporated. All rights reserved.

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

/* hal */
#include "hal_types.h"
#include "hal_mac_cfg.h"

/* high-level */
#include "mac_pib.h"
#include "mac_spec.h"

/* exported low-level */
#include "mac_low_level.h"

/* low-level */
#include "mac_spi.h"
#include "mac_dualchip.h"
#include "mac_sleep.h"
#include "mac_rx.h"
#include "mac_tx.h"
#include "mac_rx_onoff.h"
#include "mac_radio.h"
#include "mac_backoff_timer.h"
#include "mac_mcu_timer.h"
#include "mac_dualchip_tx.h"

/* radio sepcific */
#include "mac_radio_defs.h"

/* debug */
#include "hal_board.h"
#include "mac_assert.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */
#define RANDOM_POLY             0x1021
#define RANDOM_TOP_MOST_BIT     0x8000

/* calculation of phy bytes (actual over-the-air bytes) received per backoff */
#define PHY_BYTES_PER_OCTET     1
#define PHY_OCTETS_PER_SYMBOL   MAC_SPEC_OCTETS_PER_SYMBOL
#define PHY_BYTES_PER_BACKOFF   (PHY_BYTES_PER_OCTET * PHY_OCTETS_PER_SYMBOL * MAC_A_UNIT_BACKOFF_PERIOD)


/* ------------------------------------------------------------------------------------------------
 *                                        Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void dualchipTurnOnRadioPowerVREG(void);

void MAC_SetRandomSeedCB(macRNGFcn_t pCBFcn);

/* ------------------------------------------------------------------------------------------------
 *                                         Local Variables
 * ------------------------------------------------------------------------------------------------
 */
uint8          macChipVersion = 0;
static uint16  random;
static uint8   frmfilt0 = (FRMFILT0_RESET_VALUE & ~MAX_FRAME_VERSION_MASK);
static int8    maxRssi;

#if (defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590)
uint8          macRxGain = 1;
#endif

/* Function pointer for the random seed callback */
static macRNGFcn_t pRandomSeedCB = NULL;

/**************************************************************************************************
 * @fn          MAC_SetRandomSeedCB
 *
 * @brief       Set the function pointer for the random seed callback.
 *
 * @param       pCBFcn - function pointer of the random seed callback
 *
 * @return      none
 **************************************************************************************************
 */
void MAC_SetRandomSeedCB(macRNGFcn_t pCBFcn)
{
  pRandomSeedCB = pCBFcn;
}

/**************************************************************************************************
 * @fn          macDualchipSystemInit
 *
 * @brief       Initialize all code dualchip system code.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipSystemInit(void)
{
  /* initialize modules */
  halMacInit();
  macMcuTimerInit();
  macSpiInit();

  /* minimal power-up of radio chip */
  dualchipTurnOnRadioPowerVREG();

  /* make sure correct radio chip is attached */
  MAC_ASSERT(macSpiReadReg(CHIPID) == CHIPID_RESET_VALUE); /* incorrect radio chip or SPI not operating */

  /* Save the chip version for workarounds */
  macChipVersion = macSpiReadReg(VERSION);

 /*----------------------------------------------------------------------------------------------
  *  Initialize random seed value.
  */

  /*
   *  Set radio for infinite reception.  Once radio reaches this state,
   *  it will stay in receive mode regardless of RF activity.
   */
  macSpiWriteReg(FRMCTRL0, FRMCTRL0_RESET_VALUE | RX_MODE_INFINITE_RECEPTION);

  /* turn on the receiver */
  macSpiCmdStrobe(SRXON);

  /*
   *  Wait for radio to reach infinite reception state.  Once it does,
   *  RANDOM command strobe can be used to generate random number.
   */
  while ((macSpiReadReg(FSMSTAT0) & FSM_FFCTRL_STATE_RX_MASK) != FSM_FFCTRL_STATE_RX_INF);

  /* use customized RANDOM strobe command to get random seed number */
  random = (macSpiRandomByte() << 8) + macSpiRandomByte();

  /*
   *  The seed value must not be zero or 0xF01F.  If it is zero, the pseudo random sequence will 
   *  always be zero. If it is 0xF01F, the pseudo random sequence will always be 0xF01F.
   *  There is an extremely small chance this seed could randomly be zero (more likely some type of
   *  hardware problem would cause this).  The following check makes sure this does not happen.
   */
  if (random == 0x0000 || random == 0xF01F)
  {
    random = 0xBEEF; /* completely arbitrary "random" value */
  }

  /* Read 16*8 random bits and store them in flash for future use in random
     key generation for CBKE key establishment */
  if( pRandomSeedCB )
  {
    uint8 randomSeed[MAC_RANDOM_SEED_LEN];
    uint8 i;

    for(i = 0; i < MAC_RANDOM_SEED_LEN; i++)
    {
      randomSeed[i] = macSpiRandomByte();
    }
    pRandomSeedCB( randomSeed );
  }
  /*----------------------------------------------------------------------------------------------
   */

  /*
   *  No need to turn off the receiver or to exit infinite receptin mode.
   *  The entire radio chip is about to be turned off.
   */

  /* turn radio back off */
  macDualchipTurnOffRadioPower();
}


/**************************************************************************************************
 * @fn          macDualchipRandomByte
 *
 * @brief       Returns a random byte.  Initialization for random the random generation is
 *              performed by the module initialization function.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8 macDualchipRandomByte(void)
{
  uint8 i;

  for (i=0; i<8; i++)
  {
    if (random & RANDOM_TOP_MOST_BIT)
    {
      random = (random << 1) ^ RANDOM_POLY;
    }
    else
    {
      random = (random << 1);
    }
  }

  return (random & 0xFF);
}

/**************************************************************************************************
 * @fn          macDualchipRandomWord
 *
 * @brief       Returns a random word.  Initialization for random the random generation is
 *              performed by the module initialization function.
 *
 * @param       none
 *
 * @return      a random word
 **************************************************************************************************
 */
uint16 macDualchipRandomWord(void)
{
  uint8 i;

  for (i=0; i<8; i++)
  {
    if (random & RANDOM_TOP_MOST_BIT)
    {
      random = (random << 1) ^ RANDOM_POLY;
    }
    else
    {
      random = (random << 1);
    }
  }

  return (random);
}


/**************************************************************************************************
 * @fn          macDualchipOrFRMFILT0
 *
 * @brief       Specialized function that OR's bits into the FRMFILT0 register.  A shadow register
 *              for FRMFILT0 is maintained so only a register write, instead of a costly
 *              read-modify-write, is necessary.
 *
 * @param       bit - value to 'OR' into FRMFILT0 register
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipOrFRMFILT0(uint8 bit)
{
  frmfilt0 |= bit;
  macSpiWriteReg(FRMFILT0, frmfilt0);
}


/**************************************************************************************************
 * @fn          macDualchipAndFRMFILT0
 *
 * @brief       Specialized function that AND's bits into the FRMFILT0 register.  A shadow register
 *              for FRMFILT0 is  maintained so only a register write, instead of a costly
 *              read-modify-write, is necessary.
 *
 * @param       bit - value to 'AND' into FRMFILT0 register
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipAndFRMFILT0(uint8 bit)
{
  frmfilt0 &= bit;
  macSpiWriteReg(FRMFILT0, frmfilt0);
}


/**************************************************************************************************
 * @fn          macDualchipTurnOnRadioPower
 *
 * @brief       Logic and sequence for powering up the radio.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTurnOnRadioPower(void)
{
  /* enable power to the radio chip */
  dualchipTurnOnRadioPowerVREG();

  /* configure GPIOs */

  /* There is a 2520 chip bug (#657, #1388) that causes undesirable behaviour
   * when reconfiguring a 2520 GPIO pin from output to input pin. There is a
   * software workaround for that. In the following sequence, we are just
   * changing the signals coming out on these pin but their direction are same
   * as the reset values. So the workaround is not needed.
   *
   * However if this is remapped again, see the previous version of this file
   * for an example of the workaround.
   */

  /* GPIO0 is high when TX_ACK_DONE or TX_FRM_DONE using exception channel A */
  macSpiWriteReg( EXCMASKA0,  TX_FRM_DONE_BIT       |  TX_ACK_DONE_BIT        );
  macSpiWriteReg( GPIOCTRL0,  GPIO_DIR_RADIO_OUTPUT |  EXCEPTION_CHANNEL_A    );

  macSpiWriteReg( GPIOCTRL1,  GPIO_DIR_RADIO_OUTPUT |  EXCEPTION_RFC_FIFO     );
  macSpiWriteReg( GPIOCTRL2,  GPIO_DIR_RADIO_OUTPUT |  EXCEPTION_RFC_FIFOP    );

#if (defined HAL_PA_LNA || defined HAL_PA_LNA_CC2590)
  
  /* PA/LNA register configurations. Note that the settings for GPIOCTRL4 and
   * GPIOCTRL5 are not documented in the datasheet.
   *
   * GPIOCTRL3    = 0x7F (HGM per macRxGain)
   * GPIOCTRL4    = 0x46 (EN set to lna_pd[1] inverted)
   * GPIOCTRL5    = 0x47 (PAEN set to pa_pd inverted)
   * GPIOPOLARITY = 0x0F (invert GPIO4 and GPIO5)
   */
  if (macRxGain == 1)
  {
    macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | 0x7F );

    /* Sets CCA threshold to -70dBm */
    macSpiWriteReg( CCACTRL0, 0x06 );
  }
  else
  {
    macSpiWriteReg( GPIOCTRL3, GPIO_DIR_RADIO_OUTPUT | 0x7E );
    
    /* Sets CCA threshold to -80dBm nominally */
    macSpiWriteReg( CCACTRL0, 0xFC );
  }
  macSpiWriteReg( GPIOCTRL4, GPIO_DIR_RADIO_OUTPUT | 0x46 );
  macSpiWriteReg( GPIOCTRL5, GPIO_DIR_RADIO_OUTPUT | 0x47 );
  macSpiWriteReg( GPIOPOLARITY, 0x0F );

  /* Increased AGC target gain. Fixed packet loss at some input levels - with PA/LNA */
  macSpiWriteReg( AGCCTRL1, 0x16 );
  
#else
  
  /* SFD is required for Beacon mode */
  macSpiWriteReg( GPIOCTRL4,  GPIO_DIR_RADIO_OUTPUT |  EXCEPTION_RFC_SFD_SYNC );

  /* Configure CC2520 GPIO controls (GPIOCTRL3, GPIOCTRL5) for Sniffer mode.
   * Beacon related GPIOs will be turned on after the first beacon transmission.
   */
  HAL_MAC_CONFIG_SNIFFER_MODE_PINS();

  /* Increased AGC target gain. Fixed packet loss at some input levels - without PA/LNA */
  macSpiWriteReg( AGCCTRL1, 0x11 );
  
  /* Sets CCA threshold to -80dBm nominally */
  macSpiWriteReg( CCACTRL0, 0xFC );  
  
#endif

  /* For best sensitivity and noise (false packets) immunity  */
  macSpiWriteReg( MDMCTRL0, 0x85 );
  macSpiWriteReg( MDMCTRL1, 0x14 );

  /* For best sensitivity */
  macSpiWriteReg( RXCTRL, 0x3F );

  /* For better sensitivity and improved lock stability at high temp */
  macSpiWriteReg( FSCTRL, 0x5A );

  /* For better EVM and improved lock stability at high temp */
  macSpiWriteReg( FSCAL1, 0x2B );

  /* For best sensitivity */
  macSpiWriteReg( ADCTEST0, 0x10 );

  /* For stable ADC, and best RSSI performance */
  macSpiWriteReg( ADCTEST1, 0x0E );

  /* For stable ADC */
  macSpiWriteReg( ADCTEST2, 0x03 );

  /* turn off source address pending match feature */
  MAC_RADIO_TURN_OFF_SRC_MATCH();

  /* restore radio values lost when chip is powered down */
  macRadioSetChannel(macPib.logicalChannel);
  macRadioSetTxPower(macPib.phyTransmitPower);
  macRadioSetPanCoordinator(macPanCoordinator);
  macRadioSetPanID(macPib.panId);
  macRadioSetShortAddr(macPib.shortAddress);
  macRadioSetIEEEAddr(macPib.extendedAddress.addr.extAddr);

  /* Turn on autoack. All frames that are accepted by address filtering, have the acknowledge request
   * flag set and have a valid CRC, are automatically acknowledged 12 symbol periods after being received.
   */
  MAC_RADIO_TURN_ON_AUTO_ACK();

  /* Turn on Frame Filter */
  MAC_RADIO_TURN_ON_RX_FRAME_FILTERING();

  /* Prepare TX_ACK_DONE and TX_FRM_DONE shared interrupt */
  MAC_RADIO_CLEAR_TX_ACK_AND_TX_FRM_DONE_PIN();
  HAL_MAC_CLEAR_TX_ACK_DONE_INT_FLAG();
  HAL_MAC_ENABLE_TX_ACK_DONE_INT();
  
  /* enable rx overflow interrupt on falling edge */
  HAL_MAC_ENABLE_FIFO_INT();
  HAL_MAC_CONFIG_FIFO_FALLING_EDGE_INT();
}


/*=================================================================================================
 * @fn          dualchipEnableVREG
 *
 * @brief       Sequence for enabling power to the radio chip.  Oscillator is
 *              automatically when VREG is turned on.
 *
 * @param       none
 *
 * @return      none
 *=================================================================================================
 */
static void dualchipTurnOnRadioPowerVREG(void)
{
  MAC_ASSERT(macSpiRadioPower == MAC_SPI_RADIO_POWER_OFF);
  MAC_ASSERT_STATEMENT( macSpiRadioPower = MAC_SPI_RADIO_POWER_VREG_ON | MAC_SPI_RADIO_POWER_OSC_ON; );

  /* put radio chip into reset */
  HAL_MAC_DRIVE_RESETN_PIN_LOW();

  /* enable the voltage regulator */
  HAL_MAC_DRIVE_VREG_EN_PIN_HIGH();

  /* wait for the chip to power up */
  MAC_MCU_TIMER_WAIT_USECS( HAL_MAC_VREG_SETTLE_TIME_USECS );

  /* release from reset */
  HAL_MAC_DRIVE_RESETN_PIN_HIGH();

  /* wait for oscillator to stabilize */

  /*
   * Cannot access the SO pin directly.
   * Luminary workaround.
   */
  HAL_MAC_SPI_LUMINARY_SO_AS_GPIO();

  HAL_MAC_SPI_SET_CHIP_SELECT_ON();
  while (!HAL_MAC_SPI_READ_SO_PIN());
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();

  HAL_MAC_SPI_LUMINARY_SO_RESTORE();
}


/**************************************************************************************************
 * @fn          macDualchipTurnOffRadioPower
 *
 * @brief       Sequence for disabling power to the radio chip.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTurnOffRadioPower(void)
{
  MAC_ASSERT(macSpiRadioPower != MAC_SPI_RADIO_POWER_OFF);
  MAC_ASSERT_STATEMENT( macSpiRadioPower = MAC_SPI_RADIO_POWER_OFF; );

  /*
   *  Re-initialize radio software module when powering off chip.
   *  The shadow values of certan radio parameters need to match
   *  the power-on state.
   */
  macRadioInit();

  /* put chip into reset and then turn off voltage regulator */
  HAL_MAC_DRIVE_RESETN_PIN_LOW();
  HAL_MAC_DRIVE_VREG_EN_PIN_LOW();
}


/**************************************************************************************************
 * @fn          macDualchipTurnOnRadioOscillator
 *
 * @brief       Logic and sequence for turning on the radio oscillator.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTurnOnRadioOscillator(void)
{
  MAC_ASSERT(macSpiRadioPower == MAC_SPI_RADIO_POWER_VREG_ON);
  MAC_ASSERT_STATEMENT( macSpiRadioPower = MAC_SPI_RADIO_POWER_VREG_ON | MAC_SPI_RADIO_POWER_OSC_ON; );

  /* turn on the oscillator */
  macSpiCmdStrobe(SXOSCON);

  /* wait for oscillator to stabilize */

  /*
   * Cannot access the SO pin directly.
   * Luminary workaround.
   */
  HAL_MAC_SPI_LUMINARY_SO_AS_GPIO();

  HAL_MAC_SPI_SET_CHIP_SELECT_ON();
  while (!HAL_MAC_SPI_READ_SO_PIN());
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();

  HAL_MAC_SPI_LUMINARY_SO_RESTORE();
}


/**************************************************************************************************
 * @fn          macDualchipTurnOffRadioOscillator
 *
 * @brief       Sequence for disabling the radio oscillator.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipTurnOffRadioOscillator(void)
{
  MAC_ASSERT( macSpiRadioPower & MAC_SPI_RADIO_POWER_OSC_ON);
  MAC_ASSERT_STATEMENT( macSpiRadioPower = MAC_SPI_RADIO_POWER_VREG_ON; );

  macSpiCmdStrobe(SXOSCOFF);
}


/**************************************************************************************************
 * @fn          macDualchipRecordMaxRssiStart
 *
 * @brief       Starts recording of the maximum received RSSI value.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipRecordMaxRssiStart(void)
{
  /* start maximum recorded value at the lowest possible value */
  maxRssi = -128;

  /* enable timer overflow interrupt */
  MAC_MCU_TIMER_ENABLE_ENERGY_DETECT();
}


/**************************************************************************************************
 * @fn          macDualchipRecordMaxRssiStop
 *
 * @brief       Stops recording of the maximum received RSSI.  It returns the maximum value
 *              received since starting the recording.
 *
 * @param       none
 *
 * @return      maximum received RSSI value
 **************************************************************************************************
 */
int8 macDualchipRecordMaxRssiStop(void)
{
  /* disable timer overflow interrupt */
  MAC_MCU_TIMER_DISABLE_ENERGY_DETECT();

  /* return maximum recorded RSSI value */
  return(maxRssi);
}


/**************************************************************************************************
 * @fn          macDualchipRecordMaxRssiIsr
 *
 * @brief       Interrupt service routine called during recording of max RSSI value.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macDualchipRecordMaxRssiIsr(void)
{
  int8 rssi;
  int8 rssi2;
  int8 rssi3;

  /* read three values, this is a workaround for an older chip bug. New errata does not show
   * this anymore but keep it for backward compatibility.
   */
  rssi  = macSpiReadReg(RSSI) & 0xff;
  rssi2 = macSpiReadReg(RSSI) & 0xff;
  rssi3 = macSpiReadReg(RSSI) & 0xff;

  /* if the first two values are equal, use that value, if not, use the third value */
  if (rssi != rssi2)
  {
    rssi = rssi3;
  }

  /* store maximum RSSI value */
  if (rssi > maxRssi)
  {
    maxRssi = rssi;
  }
}


/**************************************************************************************************
 */
