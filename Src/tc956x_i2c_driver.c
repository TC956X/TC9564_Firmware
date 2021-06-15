/* ============================================================================
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Toshiba Electronic Devices & Storage Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ========================================================================= */

/*! History:
 *  22 Oct 2020 : Baselined
 *  23 Feb 2021 : Macros used for magic numbers 
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x_i2c_driver.c
 *
 *********************************************************************************************************
 */

/*********************************************************************************************************
*                                             Include Files
*********************************************************************************************************/
#include "tc956x_i2c_driver.h"
#include "tc956x_intc_driver.h"

/*********************************************************************************************************
*                                             Macro Definition
*********************************************************************************************************/
#define ARM_I2C_DRV_VERSION    (ARM_DRIVER_VERSION_MAJOR_MINOR(2U, 1U)) /* driver version */

#define ADDRESS_MASK_10BIT          (0x3FFU)
#define ADDRESS_MASK_7BIT           (0x7FU)
#define TAR_10BITADDR               (((uint32_t)0x01U)<<12)
#define I2C_ADDRESS_10BIT           (0x0400U)
#define I2C_SLAVE_ADDRESS_10BITMASK (0x3FFU | I2C_ADDRESS_10BIT)
#define I2C_SLAVE_ADDRESS_7BITMASK  (0x7FU)

#define I2C_STATUS_TFNF_MASK        (0x2U)
#define I2C_STATUS_TFE_MASK         (0x4U)

#define I2C_CON_SLAVE_DISABLE       (1U<<6)
#define I2C_CON_RESTART_ENABLE      (1U<<5)
#define I2C_CON_SPEED_STANDARD      (1U<<1)
#define I2C_CON_SPEED_FAST          (2U<<1)
#define I2C_CON_MASTER_ENABLE       (1U<<0)

#define I2C_SS_SCL_HCNT             (0x2DFU)
#define I2C_SS_SCL_LCNT             (0x2EDU)
#define I2C_SS_SDA_SETUP            (0x26U)

#define I2C_FS_SCL_HCNT             (0x78U)
#define I2C_FS_SCL_LCNT             (0xEFU)
#define I2C_FS_SDA_SETUP            (0xFU)

#define I2C_FS_PLUS_SCL_HCNT        (0x2AU)
#define I2C_FS_PLUS_SCL_LCNT        (0x5CU)
#define I2C_FS_PLUS_SDA_SETUP       (0x8U)

#define TIMEOUT_PERIOD              (0x10000U)
/*********************************************************************************************************
*                                             Global Variables
*********************************************************************************************************/

typedef struct {
  uint8_t   flags;          /* Control and state flags */
  uint32_t  speed;          /* IC_CON.SPEED register fieldvalue */
} I2C_INFO;

static I2C_INFO  *info;      /* Run-Time information */

/*********************************************************************************************************
*                                             Function Declaration
*********************************************************************************************************/

static ARM_DRIVER_VERSION ARM_I2C_GetVersion (void);
/**
*brief       Get driver version.
*return      \ref ARM_DRIVER_VERSION
*/

static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities (void);
/**
*brief       Get driver capabilities.
*return      \ref ARM_I2C_CAPABILITIES
*/

static int32_t ARM_I2C_Initialize (ARM_I2C_SignalEvent_t cb_event);
/**
*brief       Initialize I2C Interface.
*param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
*return      \ref execution_status
*/

static int32_t ARM_I2C_Uninitialize (void);
/**
*brief       De-initialize I2C Interface.
*return      \ref execution_status
*/

static int32_t ARM_I2C_PowerControl (ARM_POWER_STATE state);
/**
*brief       Control I2C Interface Power.
*param[in]   state  Power state
*return      \ref execution_status
*/

static int32_t ARM_I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
/**
* brief       Start transmitting data as I2C Master.
* param[in]   addr          Slave address (7-bit or 10-bit)
* param[in]   data          Pointer to buffer with data to transmit to I2C Slave
* param[in]   num           Number of data bytes to transmit
* param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
* return      \ref execution_status
*/

static int32_t ARM_I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
/**
* brief       Start receiving data as I2C Master.
* param[in]   addr          Slave address (7-bit or 10-bit)
* param[out]  data          Pointer to buffer for data to receive from I2C Slave
* param[in]   num           Number of data bytes to receive
* param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
* return      \ref execution_status
*/

static int32_t ARM_I2C_SlaveTransmit (const uint8_t *data, uint32_t num);
/**
* brief       Start transmitting data as I2C Slave.
* param[in]   data  Pointer to buffer with data to transmit to I2C Master
* param[in]   num   Number of data bytes to transmit
* return      \ref execution_status
*/

static int32_t ARM_I2C_SlaveReceive (uint8_t *data, uint32_t num);
/**
* brief       Start receiving data as I2C Slave.
* param[out]  data  Pointer to buffer for data to receive from I2C Master
* param[in]   num   Number of data bytes to receive
* return      \ref execution_status
*/

static int32_t ARM_I2C_GetDataCount (void);
/**
* brief       Get transferred data count.
* return      number of data bytes transferred; -1 when Slave is not addressed by Master
*/

static int32_t ARM_I2C_Control (uint32_t control, uint32_t arg);
/**
*brief       Control I2C Interface.
* param[in]   control  Operation
* param[in]   arg      Argument of operation (optional)
* return      \ref execution_status
*/

static ARM_I2C_STATUS ARM_I2C_GetStatus (void);
/**
* brief       Get I2C status.
* return      I2C status \ref ARM_I2C_STATUS
*/

static int32_t tc956x_I2C_EepromWrite (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num);
/**
* brief       Transmits data as Master to the EEPROM device.
* param[in]   addr         Slave address (7-bit)
* param[in]   page_no      EEPROM page number (0-7)
* param[in]   *data        Pointer to buffer with Word Address and data to transmit to I2C Slave
* param[in]   num          Word address size and number of data bytes to transmit
* return      \ref Status Error Codes
*/

static int32_t tc956x_I2C_EepromRead (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num);
/**
* brief       Reads data as Master from the EEPROM device.
* param[in]   addr         Slave address (7-bit)
* param[in]   page_no      EEPROM page number (0-7)
* param[in]   *data        Pointer to buffer with Word Address and for data to be received from I2C Slave
* param[in]   num          Word address size and number of data bytes to receive
* return      \ref Status Error Codes
*/

static int32_t SetupBusSpeed(uint32_t speed);

/*********************************************************************************************************
*                                             Function Definition
*********************************************************************************************************/

/**
  \fn          int32_t ARM_I2C_Initialize (ARM_I2C_SignalEvent_t cb_event)
  \brief       Initialize I2C Interface.
  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent_t
  \return      \ref Status Error Codes
*/
static int32_t ARM_I2C_Initialize (ARM_I2C_SignalEvent_t cb_event)
{
  uint32_t reg_value;
  uint8_t  cnt;
  uint32_t mode_pin;

  if ((I2C_INITIALIZED == info->flags) || (cb_event != NULL))
  {
    return ARM_DRIVER_ERROR;
  }

  /* Check whether I2C mode is selected on board*/
  mode_pin = hw_reg_read32 ( TC956X_REG_BASE, CNF_REG_NMODESTS ) ;
  mode_pin = ( mode_pin >> MODE_I2C_BIT_POS ) & TC956X_I2C_ONE;
  if (mode_pin != TC956X_ZERO)
  {
    return ARM_DRIVER_ERROR_UNSUPPORTED ;
  }

  /* select I2C_SCL/SDA */
  reg_value = hw_reg_read32(CNF_REG_BASE, CNF_REG_NFUNCEN0);
  reg_value = reg_value | I2C_SCL_SDA_ENABLE;
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NFUNCEN0, reg_value);

  /* I2CSPIEN=1 */
  reg_value = hw_reg_read32(CNF_REG_BASE, CNF_REG_NFUNCEN0);
  if (((reg_value & (I2C_SPI_ENABLE_POS)) >> I2C_SPI_ENABLE_BIT) == TC956X_ZERO)
  {
    reg_value = reg_value | ((uint32_t)TC956X_I2C_ONE << I2C_SPI_ENABLE_BIT);
    hw_reg_write32(CNF_REG_BASE, CNF_REG_NFUNCEN0, reg_value);
    /* Dummy Read 10 times */
    for(cnt = TC956X_ZERO; cnt < TC956X_TEN; cnt++)
    {
      hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);
    }
  }

  /* I2CMEN=1 */
  reg_value = hw_reg_read32(CNF_REG_BASE, CNF_REG_NFUNCEN0);
  reg_value = reg_value | (((uint32_t)TC956X_I2C_ONE) << I2C_MASTER_ENABLE_BIT);
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NFUNCEN0, reg_value);
  /* Dummy Read 2 times */
  hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);
  hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);

  /* Enable Clock and De-assert Reset*/
  reg_value  = hw_reg_read32(CNF_REG_BASE, CNF_REG_NRSTCTRL);
  reg_value |=  ((uint32_t)TC956X_I2C_ONE) << I2C_CLK_RST_BIT; /* enable NCLKRST for I2C */
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NRSTCTRL, reg_value);

  reg_value = hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);
  reg_value = reg_value | I2C_CLK_ENABLE;
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NCLKCTRL, reg_value);

  reg_value  = hw_reg_read32(CNF_REG_BASE, CNF_REG_NRSTCTRL);
  reg_value &= ~((uint32_t)TC956X_I2C_ONE << I2C_CLK_RST_BIT); /* disable NCLKRST for I2C */
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NRSTCTRL, reg_value);

  info->speed = I2C_CON_SPEED_FAST;  /* set to reset value */
  info->flags = I2C_INITIALIZED;
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_I2C_Uninitialize (void)
  \brief       De-initialize I2C Interface.
  \return      \ref Status Error Codes
*/
static int32_t ARM_I2C_Uninitialize (void)
{
  uint32_t reg_value;

  if (I2C_INITIALIZED != info->flags)
  {
    return ARM_DRIVER_ERROR;
  }

  /* NVIC_DisableIRQ(INT_SRC_NBR_I2C_Master); */

  reg_value  = hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);
  reg_value |=  ((uint32_t)TC956X_I2C_ZERO) << I2C_CLK_RST_BIT; /* disable NCLKCTRL for I2C */
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NCLKCTRL, reg_value);

  reg_value  = hw_reg_read32(CNF_REG_BASE, CNF_REG_NRSTCTRL);
  reg_value |=  ((uint32_t)TC956X_I2C_ONE) << I2C_CLK_RST_BIT; /* enable NCLKRST for I2C */
  hw_reg_write32(CNF_REG_BASE, CNF_REG_NRSTCTRL, reg_value);
  /* Dummy Read*/
  hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);
  hw_reg_read32(CNF_REG_BASE, CNF_REG_NCLKCTRL);

  info->flags = I2C_UNINITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
  \brief       Transmits data as Master to the selected Slave.
  \param[in]   addr         Slave address (7-bit or 10-bit)
  \param[in]   *data        Pointer to buffer with data to transmit to I2C Slave
  \param[in]   num          Number of data bytes to transmit
  \param[in]   xfer_pending Transfer operation is pending - Stop condition will not be generated
  \return      \ref Status Error Codes
*/
static int32_t ARM_I2C_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
  uint32_t i2c_data_cmd, reg_value ;
  uint32_t i2c_10bit, stop_bit;
  uint32_t pending_bytes = num;
  uint32_t retryCount = TC956X_ZERO;

  if (I2C_INITIALIZED != info->flags)
  {
    return ARM_DRIVER_ERROR;
  }

  if ((num == TC956X_ZERO) || (data == NULL))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /*check for slave address*/
  if ((addr & ARM_I2C_ADDRESS_10BIT) != TC956X_ZERO)
  {
    if ((addr == ARM_I2C_ADDRESS_10BIT) || ((addr & ~I2C_SLAVE_ADDRESS_10BITMASK) != TC956X_ZERO)) {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
    addr &= ADDRESS_MASK_10BIT;
    i2c_10bit = TAR_10BITADDR;
  }
  else
  {
    if ((addr == TC956X_ZERO) || ((addr & ~I2C_SLAVE_ADDRESS_7BITMASK) != TC956X_ZERO)) {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
    addr &= ADDRESS_MASK_7BIT;
    i2c_10bit = TC956X_ZERO;
  }

  if (xfer_pending == true)
  {
    stop_bit = (((uint32_t)TC956X_I2C_ZERO) << IC_STOP_BIT_POS);
  }
  else
  {
    stop_bit = (((uint32_t)TC956X_I2C_ONE) << IC_STOP_BIT_POS);
  }

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_NOT_INT); /* IC_ENABLE */
  hw_reg_write32(I2C_REG_BASE, IC_INTR_MASK, I2C_INTR_MASK);

  /*set the target slave address*/
  reg_value = i2c_10bit | addr;
  hw_reg_write32(I2C_REG_BASE, IC_TAR, reg_value);

  /*set the bit addressing, speed and master mode in control register*/
  reg_value = info->speed | I2C_CON_MASTER_ENABLE
                          | I2C_CON_SLAVE_DISABLE
                          | I2C_CON_RESTART_ENABLE;
  hw_reg_write32(I2C_REG_BASE, IC_CON, reg_value);

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_IN_PROGRESS); /* IC_ENABLE */

  while (pending_bytes > TC956X_ZERO)
  {
    reg_value = hw_reg_read32(I2C_REG_BASE, IC_STATUS);
    while ((reg_value & I2C_STATUS_TFNF_MASK) != TC956X_ZERO)
    {
      if(pending_bytes > TC956X_ONE)
      {
        i2c_data_cmd = (((uint32_t)TC956X_I2C_ZERO) << IC_CMD_BIT_POS) | *data;
        data++;
      }
      else if (pending_bytes == TC956X_ONE)
      {
        i2c_data_cmd = stop_bit | (((uint32_t)TC956X_I2C_ZERO) << IC_CMD_BIT_POS) | *data;
        data++;
      }
      else
      {
        break;
      }
      hw_reg_write32(I2C_REG_BASE, IC_DATA_CMD, i2c_data_cmd);
      pending_bytes--;
      reg_value = hw_reg_read32(I2C_REG_BASE, IC_STATUS);
    }

    reg_value = hw_reg_read32(I2C_REG_BASE, IC_TXFFLR);
    if (((reg_value == TC956X_ZERO) && (pending_bytes == TC956X_ZERO)) || (retryCount > TIMEOUT_PERIOD))
    {
      break;
    }
    else
    {
      retryCount++;
    }

  }

  mdelay(1);

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
  \brief       Receive data as Master from the selected Slave.
  \param[in]   addr         Slave address (7-bit or 10-bit)
  \param[in]   *data        Pointer to buffer for data to receive from I2C Slave
  \param[in]   num          Number of data bytes to receive
  \param[in]   xfer_pending Transfer operation is pending - Stop condition will not be generated
  \return      \ref Status Error Codes
*/
static int32_t ARM_I2C_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
  uint32_t i2c_data_cmd, reg_value;
  uint32_t i2c_10bit, stop_bit;
  uint32_t command_bytes, pending_bytes;
  uint32_t retryCount = TC956X_ZERO;

  command_bytes = num;
  pending_bytes = num;

  if (I2C_INITIALIZED != info->flags)
  {
    return ARM_DRIVER_ERROR;
  }

  if ((num == TC956X_ZERO) || (data == NULL))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /*check for slave address*/
  if ((addr & ARM_I2C_ADDRESS_10BIT) != TC956X_ZERO)
  {
    if ((addr == ARM_I2C_ADDRESS_10BIT) || ((addr & ~I2C_SLAVE_ADDRESS_10BITMASK) != TC956X_ZERO)) {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
    addr &= ADDRESS_MASK_10BIT;
    i2c_10bit = TAR_10BITADDR;
  }
  else
  {
    if ((addr == TC956X_ZERO) || ((addr & ~I2C_SLAVE_ADDRESS_7BITMASK) != TC956X_ZERO)) {
      return ARM_DRIVER_ERROR_PARAMETER;
    }
    addr &= ADDRESS_MASK_7BIT;
    i2c_10bit = TC956X_ZERO;
  }

  if (xfer_pending == true)
  {
    stop_bit = (((uint32_t)TC956X_ZERO) << IC_STOP_BIT_POS);
  }
  else
  {
    stop_bit = (((uint32_t)TC956X_ONE) << IC_STOP_BIT_POS);
  }

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_NOT_INT); /* IC_ENABLE */
  hw_reg_write32(I2C_REG_BASE, IC_INTR_MASK, I2C_INTR_MASK);

  /*set the target slave address*/
  reg_value = i2c_10bit | addr;
  hw_reg_write32(I2C_REG_BASE, IC_TAR, reg_value);

  /*set the bit addressing, speed and master mode in control register*/
  reg_value = info->speed | I2C_CON_MASTER_ENABLE
                          | I2C_CON_SLAVE_DISABLE
                          | I2C_CON_RESTART_ENABLE;
  hw_reg_write32(I2C_REG_BASE, IC_CON, reg_value);

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_IN_PROGRESS); /* IC_ENABLE */

  while (pending_bytes > TC956X_ZERO)
  {
    reg_value = hw_reg_read32(I2C_REG_BASE, IC_STATUS);
    while ((reg_value & I2C_STATUS_TFNF_MASK) != TC956X_ZERO)
    {
      if (command_bytes == num)
      {
        i2c_data_cmd = (((uint32_t)TC956X_FIVE) << IC_CMD_BIT_POS);
      }
      else if (command_bytes > TC956X_ONE)
      {
        i2c_data_cmd = (((uint32_t)TC956X_ONE) << IC_CMD_BIT_POS);
      }
      else if (command_bytes == TC956X_ONE)
      {
        i2c_data_cmd = stop_bit | (((uint32_t)TC956X_ONE) << IC_CMD_BIT_POS);
      }
      else
      {
        break;
      }

      hw_reg_write32(I2C_REG_BASE, IC_DATA_CMD, i2c_data_cmd);
      command_bytes--;
      reg_value = hw_reg_read32(I2C_REG_BASE, IC_STATUS);
    }
    reg_value = hw_reg_read32(I2C_REG_BASE, IC_RXFLR);
    while((reg_value > TC956X_ZERO) && (pending_bytes > TC956X_ZERO))
    {
      i2c_data_cmd = hw_reg_read32(I2C_REG_BASE, IC_DATA_CMD);
      /* Bug Fix done to avoid over data corruption*/
      *data = (uint8_t)i2c_data_cmd;
      data++;
      pending_bytes--;
      reg_value = hw_reg_read32(I2C_REG_BASE, IC_RXFLR);
    }

    do
    {
      reg_value = hw_reg_read32(I2C_REG_BASE, IC_STATUS);
      retryCount++;
    } while((((reg_value & I2C_STATUS_TFE_MASK) != TC956X_ZERO) && (command_bytes > TC956X_ZERO)) && (retryCount <= TIMEOUT_PERIOD));

    if (retryCount > TIMEOUT_PERIOD) {
      break;
    }
    else
    {
      retryCount++;
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t ARM_I2C_Control (uint32_t control, uint32_t arg)
  \brief       Control I2C Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      \ref Status Error Codes
*/
static int32_t ARM_I2C_Control (uint32_t control, uint32_t arg)
{
  int32_t ret;
  if (I2C_INITIALIZED != info->flags)
  {
    return ARM_DRIVER_ERROR;
  }

  switch (control) {
    case ARM_I2C_OWN_ADDRESS:
    ret = ARM_DRIVER_ERROR_UNSUPPORTED;
	break;
  case ARM_I2C_BUS_SPEED:
    ret = SetupBusSpeed(arg);
	break;
  case ARM_I2C_BUS_CLEAR:
    ret = ARM_DRIVER_ERROR_UNSUPPORTED;
	break;
  case ARM_I2C_ABORT_TRANSFER:
    ret = ARM_DRIVER_ERROR_UNSUPPORTED;
	break;
  default:
    ret = ARM_DRIVER_ERROR_PARAMETER;
	break;
  }
  return ret;
}

/**
  \fn          ARM_DRIVER_VERSION ARM_I2C_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION ARM_I2C_GetVersion (void)
{
  /* Driver Version */
  static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
  };

  return DriverVersion;
}

/**
  \fn          ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities (void)
  \brief       Get driver capabilities.
  \return      \ref ARM_I2C_CAPABILITIES
*/
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities (void)
{
  ARM_I2C_CAPABILITIES DriverCapabilities = {
    1 ,    /* supports 10-bit addressing */
    0
  };
  return DriverCapabilities ;
}

static int32_t ARM_I2C_SlaveTransmit (const uint8_t *data, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;    /* This Feature is supported by hardware in TC956X */
}

static int32_t ARM_I2C_SlaveReceive (uint8_t *data, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;    /* This Feature is supported by hardware in TC956X */
}

static int32_t SetupBusSpeed(uint32_t speed)
{
  int32_t  ret = ARM_DRIVER_OK;

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_NOT_INT); /* IC_ENABLE */

  switch (speed) {
    case ARM_I2C_BUS_SPEED_STANDARD:
      info->speed = I2C_CON_SPEED_STANDARD;
      hw_reg_write32(I2C_REG_BASE, IC_SS_SCL_HCNT, I2C_SS_SCL_HCNT);
      hw_reg_write32(I2C_REG_BASE, IC_SS_SCL_LCNT, I2C_SS_SCL_LCNT);
      hw_reg_write32(I2C_REG_BASE, IC_SDA_SETUP, I2C_SS_SDA_SETUP);
      break;
    case ARM_I2C_BUS_SPEED_FAST:
      info->speed = I2C_CON_SPEED_FAST;
      hw_reg_write32(I2C_REG_BASE, IC_FS_SCL_HCNT, I2C_FS_SCL_HCNT);
      hw_reg_write32(I2C_REG_BASE, IC_FS_SCL_LCNT, I2C_FS_SCL_LCNT);
      hw_reg_write32(I2C_REG_BASE, IC_SDA_SETUP, I2C_FS_SDA_SETUP);
      break;
    case ARM_I2C_BUS_SPEED_FAST_PLUS:
      info->speed = I2C_CON_SPEED_FAST;
      hw_reg_write32(I2C_REG_BASE, IC_FS_SCL_HCNT, I2C_FS_PLUS_SCL_HCNT);
      hw_reg_write32(I2C_REG_BASE, IC_FS_SCL_LCNT, I2C_FS_PLUS_SCL_LCNT);
      hw_reg_write32(I2C_REG_BASE, IC_SDA_SETUP, I2C_FS_PLUS_SDA_SETUP);
      break;
    case ARM_I2C_BUS_SPEED_HIGH:
      ret = ARM_DRIVER_ERROR_UNSUPPORTED;
      break;
    default:
      ret = ARM_DRIVER_ERROR_PARAMETER;
      break;
  }

  hw_reg_write32(I2C_REG_BASE, IC_ENABLE, ABT_IN_PROGRESS); /* IC_ENABLE */
  return ret;
}

static int32_t ARM_I2C_PowerControl (ARM_POWER_STATE state)
{
  /* Undefined function*/
  return ARM_DRIVER_ERROR_UNSUPPORTED ;
}

static int32_t ARM_I2C_GetDataCount (void)
{
  /* Undefined function*/
  return ARM_DRIVER_ERROR_UNSUPPORTED ;
}

static ARM_I2C_STATUS ARM_I2C_GetStatus (void)
{
  ARM_I2C_STATUS status;

  status.busy             = 0;
  status.mode             = 0;
  status.direction        = 0;
  status.general_call     = 0;
  status.arbitration_lost = 0;
  status.bus_error        = 0;
  status.reserved         = 0;

  return status;
}

/**
  \fn          int32_t tc956x_I2C_EepromWrite (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num)
  \brief       Transmits data as Master to the EEPROM device.
  \param[in]   addr         Slave address (7-bit 0x50)
  \param[in]   page_no      EEPROM page number (0-7)
  \param[in]   *data        Pointer to buffer with Word Address and data to transmit to I2C Slave
  \param[in]   num          Word address size and number of data bytes to transmit
  \return      \ref Status Error Codes
*/

static int32_t tc956x_I2C_EepromWrite (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num)
{
  int32_t ret = ARM_DRIVER_OK;

  if ((num == TC956X_ZERO) || (data == NULL))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((addr != EEPROM_SLAVE_ADDRESS) || (page_no > EEPROM_PAGE_LIMIT))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /* Appending page no to EEPROM slave address */
  addr |= page_no;
  ret = ARM_I2C_MasterTransmit(addr, data, num, false);
  /* page write cycle time i.e. 8ms */
  mdelay(5U);
  return ret;
}

/**
  \fn          int32_t tc956x_I2C_EepromRead (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num)
  \brief       Reads data as Master from the EEPROM device.
  \param[in]   addr         Slave address (7-bit 0x50)
  \param[in]   page_no      EEPROM page number (0-7)
  \param[in]   *data        Pointer to buffer with Word Address and for data to be received from I2C Slave
  \param[in]   num          Word address size and number of data bytes to receive
  \return      \ref Status Error Codes
*/

static int32_t tc956x_I2C_EepromRead (uint32_t addr, uint8_t page_no, uint8_t *data, uint32_t num)
{
  int32_t ret = ARM_DRIVER_OK;

  if ((num == TC956X_ZERO) || (data == NULL))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((addr != EEPROM_SLAVE_ADDRESS) || (page_no > EEPROM_PAGE_LIMIT))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  /* Appending page info to EEPROM slave address */
  addr |= page_no;
  ret = ARM_I2C_MasterTransmit(addr, data, TC956X_ONE, true);
  if (ret != ARM_DRIVER_OK)
  {
    return ret;
  }

  /* Start EEPROM read */
  ret = ARM_I2C_MasterReceive(addr, &data[1], num - 1U, false);
  return ret; 
}

ARM_DRIVER_I2C DRIVER_I2C = {
ARM_I2C_GetVersion,
ARM_I2C_GetCapabilities,
ARM_I2C_Initialize,
ARM_I2C_Uninitialize,
ARM_I2C_PowerControl,
ARM_I2C_MasterTransmit,
ARM_I2C_MasterReceive,
ARM_I2C_SlaveTransmit,
ARM_I2C_SlaveReceive,
ARM_I2C_GetDataCount,
ARM_I2C_Control,
ARM_I2C_GetStatus,
tc956x_I2C_EepromWrite,
tc956x_I2C_EepromRead,
};
