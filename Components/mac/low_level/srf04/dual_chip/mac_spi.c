/**************************************************************************************************
  Filename:       mac_spi.c
  Revised:        $Date: 2007-05-02 13:41:12 -0700 (Wed, 02 May 2007) $
  Revision:       $Revision: 14174 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

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

/* low-level specific */
#include "mac_spi.h"

/* SPI registers */
#include "mac_radio_defs.h"

/* debug */
#include "mac_assert.h"


/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define SPI_ACCESS_BUF_LEN          3

#define NUM_BYTES_STROBE_CMD        1
#define NUM_BYTES_REG_ACCESS        2
#define NUM_BYTES_MEM_ACCESS        3
#define NUM_BYTES_RANDOM            3

#define FIFO_ACCESS_TX_WRITE        0
#define FIFO_ACCESS_RX_READ         1


/* ------------------------------------------------------------------------------------------------
 *                                         Global Variables
 * ------------------------------------------------------------------------------------------------
 */
MAC_ASSERT_DECLARATION( uint8 macSpiRadioPower; )


/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void spiFifoAccess(uint8 * pData, uint8 len, uint8 writeFlag);
static uint8 spiSendBytes(uint8 * pBytes, uint8 numBytes);
static void spiWriteRamByte(uint16 ramAddr, uint8 byte);


/**************************************************************************************************
 * @fn          macSpiInit
 *
 * @brief       Initialize SPI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiInit(void)
{
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_INIT();
}


/**************************************************************************************************
 * @fn          macSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio.  Returns status byte read during transfer
 *              of strobe command.
 *
 * @param       opCode - op code of the strobe command
 *
 * @return      status byte of radio
 **************************************************************************************************
 */
uint8 macSpiCmdStrobe(uint8 opCode)
{
  uint8 buf[SPI_ACCESS_BUF_LEN];

  /* set first byte of array as op code */
  buf[0] = opCode;

  /* send strobe command and pass up return value */
  return( spiSendBytes(&buf[0], NUM_BYTES_STROBE_CMD) );
}


/**************************************************************************************************
 * @fn          macSpiReadReg
 *
 * @brief       Read value from radio regiser.
 *
 * @param       regAddr - address of register
 *
 * @return      register value
 **************************************************************************************************
 */
uint8 macSpiReadReg(uint8 regAddr)
{
  uint8 buf[SPI_ACCESS_BUF_LEN];
  uint8 len;

  /* fast register access is available for registers in first 0x40 bytes */
  if (regAddr <= 0x3F)
  {
    /* set up a register read command */
    buf[0] = REGRD | regAddr;
    len = NUM_BYTES_REG_ACCESS;
  }
  else
  {
    /* set up a memory read command, note memory above 0xFF not accessible via this function */
    buf[0] = MEMRD;
    buf[1] = regAddr;
    len = NUM_BYTES_MEM_ACCESS;
  }

  /* send read command and pass up return value */
  return( spiSendBytes(&buf[0], len) );
}


/**************************************************************************************************
 * @fn          macSpiWriteReg
 *
 * @brief       Write value to radio register.
 *
 * @param       regAddr  - address of register
 * @param       regValue - register value to write
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiWriteReg(uint8 regAddr, uint8 regValue)
{
  uint8 buf[SPI_ACCESS_BUF_LEN];
  uint8 len;

  /* fast register access is available for registers in first 0x40 bytes */
  if (regAddr <= 0x3F)
  {
    /* set up a register write command */
    buf[0] = REGWR | regAddr;
    buf[1] = regValue;
    len = NUM_BYTES_REG_ACCESS;
  }
  else
  {
    /* set up a memory write command, note memory above 0xFF not accessible via this function */
    buf[0] = MEMWR;
    buf[1] = regAddr;
    buf[2] = regValue;
    len = NUM_BYTES_MEM_ACCESS;
  }

  /* send write command */
  spiSendBytes(&buf[0], len);
}


/*=================================================================================================
 * @fn          spiSendBytes
 *
 * @brief       Primitive for sending byte via SPI.  Returns SPI transferred during sending
 *              of last byte.
 *
 * @param       pSendBytes   - pointer to bytes to send over SPI
 * @param       numSendBytes - number of bytes to send
 *
 * @return      byte read from SPI after last byte sent
 *=================================================================================================
 */
static uint8 spiSendBytes(uint8 * pBytes, uint8 numBytes)
{
  halMacSpiIntState_t s;
  uint8 returnValue;

  MAC_ASSERT(macSpiRadioPower & MAC_SPI_RADIO_POWER_VREG_ON);  /* radio must be powered */

  /*-------------------------------------------------------------------------------
   *  Disable interrupts that call SPI functions.
   */
  HAL_MAC_SPI_ENTER_CRITICAL_SECTION(s);

  /*-------------------------------------------------------------------------------
   *  Turn chip select "off" and then "on" to clear any current SPI access.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_SET_CHIP_SELECT_ON();

  /*-------------------------------------------------------------------------------
   *  execute based on type of access
   */
  while (numBytes)
  {
    HAL_MAC_SPI_WRITE_BYTE(*pBytes);
    pBytes++;
    numBytes--;
    HAL_MAC_SPI_WAIT_DONE();

   /*-------------------------------------------------------------------------------
    *  SPI data register now contains the status byte. The status byte is
    *  discarded.
    */
    if (numBytes > 0)
    {
      HAL_MAC_SPI_LUMINARY_READ_DUMMY_BYTE();
    }
  }

  /*-------------------------------------------------------------------------------
   *  SPI data register now contains the status byte. The status byte is
   *  discarded.
   */
  returnValue = HAL_MAC_SPI_READ_BYTE();

  /*-------------------------------------------------------------------------------
   *  Turn off chip select.  Enable interrupts that call SPI functions.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_EXIT_CRITICAL_SECTION(s);

  return(returnValue);
}

/**************************************************************************************************
 * @fn          macSpiWriteRamUint16
 *
 * @brief       Write unsigned 16-bit value to radio RAM.
 *
 * @param       ramAddr    - radio RAM address
 * @param       pWriteData - pointer to data to write
 * @param       len        - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiWriteRamUint16(uint16 ramAddr, uint16 data)
{
  spiWriteRamByte(ramAddr,   data & 0xFF);
  spiWriteRamByte(ramAddr+1, data >> 8);
}


/**************************************************************************************************
 * @fn          macSpiWriteRam
 *
 * @brief       Write data to radio RAM.
 *
 * @param       ramAddr - radio RAM address
 * @param       pData   - pointer to data to write
 * @param       len     - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiWriteRam(uint16 ramAddr, uint8 * pData, uint8 len)
{
  while (len)
  {
    spiWriteRamByte(ramAddr, *pData);
    ramAddr++;
    pData++;
    len--;
  }
}


/*=================================================================================================
 * @fn          spiWriteRamByte
 *
 * @brief       Write a byte to radio RAM.
 *
 * @param       ramAddr - radio RAM address
 * @param       byte    - data byte to write to RAM
 *
 * @return      none
 *=================================================================================================
 */
static void spiWriteRamByte(uint16 ramAddr, uint8 byte)
{
  uint8 buf[SPI_ACCESS_BUF_LEN];

  MAC_ASSERT(ramAddr <= 0xFFF); /* address out of range */

  /* setup for a memory write */
  buf[0] = MEMWR | (ramAddr >> 8);
  buf[1] = ramAddr & 0xFF;
  buf[2] = byte;

  /* send bytes out via SPI */
  spiSendBytes(&buf[0], NUM_BYTES_MEM_ACCESS);
}


/**************************************************************************************************
 * @fn          macSpiReadRam
 *
 * @brief       Read data from radio RAM.
 *
 * @param       ramAddr     - radio RAM address
 * @param       pReadData   - pointer to read data
 * @param       len         - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiReadRam(uint16 ramAddr, uint8 *pReadData, uint8 len)
{
  halMacSpiIntState_t s;
  uint8 i;

  /* Address out of range */
  MAC_ASSERT(ramAddr <= 0xFFF);

  /* Buffer must be valid */
  MAC_ASSERT(pReadData != NULL);

  /* Radio must be powered */
  MAC_ASSERT(macSpiRadioPower & MAC_SPI_RADIO_POWER_VREG_ON);

  /* Disable interrupts that call SPI functions. */
  HAL_MAC_SPI_ENTER_CRITICAL_SECTION(s);

  /*-------------------------------------------------------------------------------
   *  Turn chip select "off" and then "on" to clear any current SPI access.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_SET_CHIP_SELECT_ON();

  /* Send MEMRD via SPI */
  HAL_MAC_SPI_WRITE_BYTE(MEMRD | (ramAddr >> 8));
  HAL_MAC_SPI_WAIT_DONE();

  /* Dummy read to clear the SPI Rx buffer */
  HAL_MAC_SPI_LUMINARY_READ_DUMMY_BYTE();

  HAL_MAC_SPI_WRITE_BYTE(ramAddr & 0xFF);
  HAL_MAC_SPI_WAIT_DONE();

  /* Dummy read to clear the SPI Rx buffer */
  HAL_MAC_SPI_LUMINARY_READ_DUMMY_BYTE();

  /* Get the data */
  for (i=0; i<len; i++)
  {
    /* Write 0 to SPI to get the data */
    HAL_MAC_SPI_WRITE_BYTE(0);
    /* Wait for Done */
    HAL_MAC_SPI_WAIT_DONE();
    /* Fill up the buffer*/
    *pReadData++ = HAL_MAC_SPI_READ_BYTE();
  }

  /*-------------------------------------------------------------------------------
   *  Turn off chip select and re-enable interrupts that use SPI.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
 * @fn          macSpiWriteTxFifo
 *
 * @brief       Write data to radio FIFO.
 *
 * @param       pData - pointer for storing write data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiWriteTxFifo(uint8 * pData, uint8 len)
{
  spiFifoAccess(pData, len, FIFO_ACCESS_TX_WRITE);
}


/**************************************************************************************************
 * @fn          macSpiReadRxFifo
 *
 * @brief       Read data from radio FIFO.
 *
 * @param       pData - pointer for storing read data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiReadRxFifo(uint8 * pData, uint8 len)
{
  spiFifoAccess(pData, len, FIFO_ACCESS_RX_READ);
}


/*=================================================================================================
 * @fn          spiFifoAccess
 *
 * @brief       Read/write data to or from radio FIFO.
 *
 * @param       pData - pointer to data to read or write
 * @param       len   - length of data in bytes
 *
 * @return      none
 *=================================================================================================
 */
static void spiFifoAccess(uint8 * pData, uint8 len, uint8 accessType)
{
  halMacSpiIntState_t s;

  MAC_ASSERT(macSpiRadioPower & MAC_SPI_RADIO_POWER_OSC_ON);  /* oscillator must be on */
  MAC_ASSERT(len != 0); /* zero length is not allowed */

  /*-------------------------------------------------------------------------------
   *  Disable interrupts that call SPI functions.
   */
  HAL_MAC_SPI_ENTER_CRITICAL_SECTION(s);

  /*-------------------------------------------------------------------------------
   *  Turn chip select "off" and then "on" to clear any current SPI access.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_SET_CHIP_SELECT_ON();

  /*-------------------------------------------------------------------------------
   *  Main loop.  If the SPI access is interrupted, execution comes back to
   *  the start of this loop.  Loop exits when nothing left to transfer.
   *  (Note: previous test guarantees at least one byte to transfer.)
   */
  do
  {
    /*-------------------------------------------------------------------------------
     *  Send FIFO access command byte.  Wait for SPI access to complete.
     */
    if (accessType == FIFO_ACCESS_TX_WRITE)
    {
      HAL_MAC_SPI_WRITE_BYTE( TXBUF );
    }
    else
    {
      HAL_MAC_SPI_WRITE_BYTE( RXBUF );
    }
    HAL_MAC_SPI_WAIT_DONE();

    /* Make a Dummy read to empty the SPI Rx buffer */
    HAL_MAC_SPI_LUMINARY_READ_DUMMY_BYTE();

    /*-------------------------------------------------------------------------------
     *  Inner loop.  This loop executes as long as the SPI access is not interrupted.
     *  Loop completes when nothing left to transfer.
     *  (Note: previous test guarantees at least one byte to transfer.)
     */
    do
    {
      HAL_MAC_SPI_WRITE_BYTE(*pData);

      /*-------------------------------------------------------------------------------
       *  Use idle time.  Perform increment/decrement operations before pending on
       *  completion of SPI access.
       *
       *  Decrement the length counter.  Wait for SPI access to complete.
       */
      len--;
      HAL_MAC_SPI_WAIT_DONE();

      /*-------------------------------------------------------------------------------
       *  SPI data register holds data just read, store the value
       *  into memory.
       */
      if (accessType != FIFO_ACCESS_TX_WRITE)
      {
        *pData = HAL_MAC_SPI_READ_BYTE();
      }
      else
      {
        /* Dummy read to clear the SPI Rx buffer */
        HAL_MAC_SPI_LUMINARY_READ_DUMMY_BYTE();
      }

      /*-------------------------------------------------------------------------------
       *  At least one byte of data has transferred.  Briefly enable (and then disable)
       *  interrupts that call SPI functions.  This provides a window for any timing
       *  critical interrupts that might be pending.
       *
       *  To improve latency, take care of pointer increment within the interrupt
       *  enabled window.
       */
      HAL_MAC_SPI_EXIT_CRITICAL_SECTION(s);
      pData++;
      HAL_MAC_SPI_ENTER_CRITICAL_SECTION(s);

      /*-------------------------------------------------------------------------------
       *  If chip select is "off" the SPI access was interrupted.  In this case,
       *  turn back on chip select and break to the main loop.  The main loop will
       *  pick up where the access was interrupted.
       */
      if (HAL_MAC_SPI_CHIP_SELECT_IS_OFF())
      {
        HAL_MAC_SPI_SET_CHIP_SELECT_ON();
        break;
      }

    /*-------------------------------------------------------------------------------
     */
    } while (len); /* inner loop */
  } while (len);   /* main loop */

  /*-------------------------------------------------------------------------------
   *  Turn off chip select and re-enable interrupts that use SPI.
   */
  HAL_MAC_SPI_SET_CHIP_SELECT_OFF();
  HAL_MAC_SPI_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
 * @fn          macSpiRandomByte
 *
 * @brief       returns a random byte
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8 macSpiRandomByte(void)
{
  uint8 sendBytes[SPI_ACCESS_BUF_LEN];

  /* setup to send RANDOM command to radio*/
  sendBytes[0] = RANDOM;

  /*
   *  Send RANDOM command via SPI. Note that valid return value needs
   *  two dummy writes (i.e. NUM_BYTES_RANDOM equals 3). */
  return( spiSendBytes(&sendBytes[0], NUM_BYTES_RANDOM) );
}

/**************************************************************************************************
 * @fn          macSpiSendECBO
 *
 * @brief       Send a ECBO command to spi
 *
 * @param       p - 0/1 low/high
 *              k - address of the key at 16xk address
 *              c - 16-c bytes of plaintext
 *              a - input/output address where plaintext and converted text is stored
 *
 * @return      none
 **************************************************************************************************
 */
void macSpiSendECBO(uint8 p, uint8 k, uint8 c, uint16 a)
{
  uint8 pBuf[4];

  /* Address out of range */
  MAC_ASSERT(a <= 0xFFF);

  /* Apply Encryption */
  pBuf[0] = ECBO | (p & 0x01);
  pBuf[1] = k;
  pBuf[2] = ((c & 0x0F) << 4) | (a >> 8);
  pBuf[3] = a & 0xFF;

  /* send bytes out via SPI */
  spiSendBytes(&pBuf[0], 4);
}

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */
#if (MAC_SPI_RADIO_POWER_VREG_ON & MAC_SPI_RADIO_POWER_OSC_ON)
#error "ERROR!  Non-unique bit values for SPI radio power states."
#endif

#if (FIFO_ACCESS_TX_WRITE != 0)
#error "ERROR!  Code optimized for FIFO_ACCESS_TX_WRITE equal to zero."
#endif

/**************************************************************************************************
*/
