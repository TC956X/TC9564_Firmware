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
 *  8 Jul 2020   : Initial Version
 *  23 Feb 2021  : Macros used for magic numbers
 *                 Removed code for UART interrupt enabling
                   in INTEXTMASK2 and INTINTXMASK2 registers
 *  05 Jul 2021  : vsprintf changed to vsnprintf
 *  VERSION      : 1.0.1
 */


/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x_uart_driver.c
 *
 *********************************************************************************************************
 */

#include "tc956x_uart_driver.h"
#include "tc956x_intc_driver.h"

static int32_t tc956x_UART_InterSend (uint8_t UART_id, const uint8_t *data, uint32_t length);
static int32_t tc956x_UART_Send (const void *data, uint32_t num, uint8_t UART_id);
static int32_t tc956x_UART_Receive (void *data, uint32_t num, uint8_t UART_id);
static int32_t tc956x_UART_Control (uint8_t UART_id, uint32_t control, uint32_t arg );
static ARM_USART_STATUS tc956x_UART_GetStatus  (uint8_t UART_id);
static int32_t tc956x_UART_Initialize(ARM_USART_SignalEvent_t cb_event, UART_CONFIG uart);
static int32_t tc956x_UART_Uninitialize (uint8_t UART_id);
static uint32_t tc956x_UART_GetTxCount (uint8_t UART_id);
static uint32_t tc956x_UART_GetRxCount (uint8_t UART_id);
int32_t tc956x_UART_InterReceive (uint8_t UART_id, uint8_t *data, uint32_t length);

static ARM_DRIVER_VERSION USARTX_GetVersion (void);
static ARM_USART_CAPABILITIES USART_GetCapabilities (uint8_t UART_id);
static ARM_USART_CAPABILITIES USART0_GetCapabilities (void);
static int32_t USART0_Initialize (ARM_USART_SignalEvent_t cb_event);
static int32_t USART0_Uninitialize (void);
static int32_t USART0_Send (const void *data, uint32_t num);
static int32_t USART0_Receive (void *data, uint32_t num);
static int32_t USART0_Control (uint32_t control, uint32_t arg);
static ARM_USART_STATUS USART0_GetStatus (void);
static uint32_t USART0_GetTxCount (void);
static uint32_t USART0_GetRxCount (void);


static ARM_USART_CAPABILITIES USART1_GetCapabilities (void);
static int32_t USART1_Initialize (ARM_USART_SignalEvent_t cb_event);
static int32_t USART1_Uninitialize (void);
static int32_t USART1_Send (const void *data, uint32_t num);
static int32_t USART1_Receive (void *data, uint32_t num);
static int32_t USART1_Control (uint32_t control, uint32_t arg);
static ARM_USART_STATUS USART1_GetStatus (void);
static int32_t USARTX_PowerControl (ARM_POWER_STATE state);
static int32_t USARTX_Transfer (const void *data_out, void *data_in, uint32_t num);
static int32_t USARTX_SetModemControl (ARM_USART_MODEM_CONTROL control);
static ARM_USART_MODEM_STATUS USARTX_GetModemStatus (void);
static uint32_t USART1_GetTxCount (void);
static uint32_t USART1_GetRxCount (void);


static uint8_t uart_init[2];
static ARM_USART_SignalEvent_t cb_event_uart_0;
static ARM_USART_SignalEvent_t cb_event_uart_1;

static UART_RESOURCE UART0_resource = {
  0,      /*TX-ENABLE*/
  0,      /*RX-ENABLE*/
  NULL,   /*Receive buffer*/
  NULL,   /*Send buffer*/
  0,      /*Number of bytes to be received*/
  0,      /*Number of bytes received*/
  0,      /*Number of bytes to be send*/
  0       /*Number of bytes send*/
};

static UART_RESOURCE UART1_resource = {
  0,      /*TX-ENABLE*/
  0,      /*RX-ENABLE*/
  NULL,   /*Receive buffer*/
  NULL,   /*Send buffer*/
  0,      /*Number of bytes to be received*/
  0,      /*Number of bytes received*/
  0,      /*Number of bytes to be send*/
  0       /*Number of bytes send*/
};

static ARM_USART_STATUS UART0_STATUS;
static ARM_USART_STATUS UART1_STATUS;

static ARM_USART_CAPABILITIES UART0_Cap = {
  1,  /*asynchronous       */
  0,  /*synchronous_master */
  0,  /*synchronous_slave  */
  0,  /*single_wire        */
  0,  /*irda               */
  0,  /*smart_card         */
  0,  /*smart_card_clock   */
  0,  /*flow_control_rts   */
  0,  /*flow_control_cts   */
  0,  /*event_tx_complete  */
  1,  /*event_rx_timeout   */
  0,  /*rts                */
  0,  /*cts                */
  0,  /*dtr                */
  0,  /*dsr                */
  0,  /*dcd                */
  0,  /*ri                 */
  0,  /*event_cts          */
  0,  /*event_dsr          */
  0,  /*event_dcd          */
  0,  /*event_ri           */
  0   /*reserved           */
};

static ARM_USART_CAPABILITIES UART1_Cap = {
  1,  /*asynchronous       */
  0,  /*synchronous_master */
  0,  /*synchronous_slave  */
  0,  /*single_wire        */
  0,  /*irda               */
  0,  /*smart_card         */
  0,  /*smart_card_clock   */
  0,  /*flow_control_rts   */
  0,  /*flow_control_cts   */
  0,  /*event_tx_complete  */
  1,  /*event_rx_timeout   */
  0,  /*rts                */
  0,  /*cts                */
  0,  /*dtr                */
  0,  /*dsr                */
  0,  /*dcd                */
  0,  /*ri                 */
  0,  /*event_cts          */
  0,  /*event_dsr          */
  0,  /*event_dcd          */
  0,  /*event_ri           */
  0   /*reserved           */
};

/**
  \fn          ARM_USART_CAPABILITIES USART_GetCapabilities (uint8_t UART_id)
  \brief       Get driver capabilities
  \param[in]   UART_id UART-ID
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART_GetCapabilities (uint8_t UART_id)
{
  if(UART_id == 0U)
  {
    return UART0_Cap;
  }
  else
  {
    return UART1_Cap;
  }
}

/**
  \fn          uint32_t tc956x_UART_GetTxCount (uint8_t UART_id)
  \brief       Get transmitted data count.
  \param[in]   UART_id UART-ID
  \return      number of data items transmitted
*/
static uint32_t tc956x_UART_GetTxCount (uint8_t UART_id)
{
  if (UART_id == 0U)
  {
    return UART0_resource.tx_cnt;
  }
  else
  {
    return UART1_resource.tx_cnt;
  }
}

/**
  \fn          uint32_t tc956x_UART_GetRxCount (uint8_t UART_id)
  \brief       Get received data count.
  \param[in]   UART_id UART-ID
  \return      number of data items received
*/
static uint32_t tc956x_UART_GetRxCount (uint8_t UART_id)
{
  if (UART_id == 0U)
  {
    return UART0_resource.rx_cnt;
  }
  else
  {
    return UART1_resource.rx_cnt;
  }
}

/**
  \fn          int32_t tc956x_UART_Send (const void *data, uint32_t num, uint8_t UART_id)
  \brief       Start sending data to USART transmitter.
  \param[in]   data    Pointer to buffer with data to send to USART transmitter
  \param[in]   num     Number of data items to send
  \param[in]   UART_id UART-ID
  \return      \ref execution_status
*/
static int32_t tc956x_UART_Send (const void *data, uint32_t num, uint8_t UART_id)
{
  uint32_t i, count;
  uint32_t uart_base, regVal;

  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  if ((data == NULL) || (num == 0U))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (UART_id == 0U)
  {
    uart_base = UART0_REG_BASE;
    /* Check whether TX is Enabled, if not return error */
    if (UART0_resource.txe == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
    if (UART0_STATUS.tx_busy == 0U)
    {
      UART0_Cap.event_tx_complete = 0;
      UART0_STATUS.tx_busy = 1;
      UART0_resource.tx_num = num;
      UART0_resource.tx_cnt = 0;
      UART0_resource.tx_buf = (const uint8_t *)data;
      if(num < TX_FIFO)
      {
        count = num;
      }
      else
      {
        count = TX_FIFO;
      }
      for (i=0U; i < count; i++)
      {
        while (1U == ((hw_reg_read32(uart_base, UART_FR)>> UART_FR_TXFF_BIT_POS) & 0x1U))
        {
          ;
        }
        hw_reg_write32 (uart_base, UART_DR, UART0_resource.tx_buf[i]);
        UART0_resource.tx_cnt++;
      }
      if (UART0_resource.tx_cnt == UART0_resource.tx_num)
      {
        UART0_STATUS.tx_busy = 0;
        UART0_Cap.event_tx_complete = 1;
        if(cb_event_uart_0 != NULL)
        {
          cb_event_uart_0 (ARM_USART_EVENT_SEND_COMPLETE);
        }
      }
      else
      {
        regVal = hw_reg_read32 (uart_base, UART_IMSC);
        /* Enable the TX Interrupt to copy data from UART Buffer to FIFO */
        hw_reg_write32(uart_base, UART_IMSC, (regVal | TXRIS));
      }
    }
    else
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  else
  {
    uart_base = UART1_REG_BASE;
    /* Check whether TX is Enabled, if not return error */
    if (UART1_resource.txe == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
    if (UART1_STATUS.tx_busy == 0U)
    {
      UART1_Cap.event_tx_complete = 0;
      UART1_STATUS.tx_busy = 1;
      UART1_resource.tx_num = num;
      UART1_resource.tx_cnt = 0;
      UART1_resource.tx_buf = (const uint8_t *)data;
      if(num < TX_FIFO)
      {
        count = num;
      }
      else
      {
        count = TX_FIFO;
      }
      for (i=0U; i < count; i++)
      {
        while (1U == ((hw_reg_read32(uart_base, UART_FR)>> UART_FR_TXFF_BIT_POS) & 0x1U))
        {
          ;
        }
        hw_reg_write32 (uart_base, UART_DR, UART1_resource.tx_buf[i]);
        UART1_resource.tx_cnt++;
      }
      if (UART1_resource.tx_cnt == UART1_resource.tx_num)
      {
        UART1_STATUS.tx_busy = 0;
        UART1_Cap.event_tx_complete = 1;
        if(cb_event_uart_1 != NULL)
        {
          cb_event_uart_1 (ARM_USART_EVENT_SEND_COMPLETE);
        }
      }
      else
      {
        regVal = hw_reg_read32 (uart_base, UART_IMSC);
        /* Enable the TX Interrupt to copy data from UART Buffer to FIFO */
        hw_reg_write32(uart_base, UART_IMSC, (regVal | TXRIS));
      }
    }
    else
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t tc956x_UART_Receive (void *data, uint32_t num, uint8_t UART_id)
  \brief       Start receiving data from USART receiver.
  \param[in]   data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   UART_id UART-ID
  \return      \ref execution_status
*/
static int32_t tc956x_UART_Receive (void *data, uint32_t num, uint8_t UART_id)
{
  uint32_t uart_base, regVal;

  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  /* Select base address of UART based on UART-ID */
  if(UART_id == 0U)
  {
    uart_base = UART0_REG_BASE;
  }
  else
  {
    uart_base = UART1_REG_BASE;
  }
  if (data == NULL)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (0U == num)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  /*Map Receive buffer to the Structure UART Receive*/
  if (UART_id == 0U)
  {
    if (UART0_resource.rxe == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
    if(UART0_STATUS.rx_busy == 0U)
    {
      UART0_STATUS.rx_busy = 1;
      UART0_STATUS.rx_break = 0;
      UART0_STATUS.rx_framing_error = 0;
      UART0_STATUS.rx_overflow = 0;
      UART0_STATUS.rx_parity_error = 0;
      UART0_resource.rx_buf = (uint8_t *)data;
      UART0_resource.rx_num = num;
      UART0_resource.rx_cnt = 0;
    }
    else
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  else
  {
    if (UART1_resource.rxe == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
    if(UART1_STATUS.rx_busy == 0U)
    {
      UART1_STATUS.rx_busy = 1;
      UART1_STATUS.rx_break = 0;
      UART1_STATUS.rx_framing_error = 0;
      UART1_STATUS.rx_overflow = 0;
      UART1_STATUS.rx_parity_error = 0;
      UART1_resource.rx_buf = (uint8_t *)data;
      UART1_resource.rx_num = num;
      UART1_resource.rx_cnt = 0;
    }
    else
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  /*Enable RX and RT Interrupts*/
  regVal = hw_reg_read32(uart_base, UART_IMSC);
  hw_reg_write32(uart_base, UART_IMSC, (regVal | RXRIS | RTRIS | FERIS | PERIS | BERIS | OERIS));
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t tc956x_UART_Control (uint8_t UART_id, uint32_t control, uint32_t arg)
  \brief       Control USART Interface.
  \param[in]   UART_id UART-ID
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t tc956x_UART_Control (uint8_t UART_id, uint32_t control, uint32_t arg)
{
  uint16_t line_control;
  uint32_t data_bit_option;
  uint32_t parity_option;
  uint32_t stop_bit_option;
  uint8_t  data_bits;
  uint8_t  pen, eps, sps;
  uint8_t  two_stop_bits;
  uint32_t uart_base;
  uint32_t Mclk_Freq;
  uint32_t Divider = 0;
  uint32_t Remainder;
  uint32_t Fraction = 0;
  uint32_t flow_control;
  uint32_t clock_polarity;
  uint32_t clock_phase;
  uint8_t fifo_lvl_tx;
  uint8_t fifo_lvl_rx;
  uint32_t regVal;
  uint32_t bd_rate_flag = 0;

  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  /* Select base address of UART based on UART-ID */
  if (0U == UART_id)
  {
    uart_base = UART0_REG_BASE;
    if ((control != ARM_USART_ABORT_RECEIVE) && (control != ARM_USART_ABORT_SEND) && (control != ARM_USART_CONTROL_BREAK))
    {
      if (UART0_STATUS.rx_busy == 1U)
      {
        return ARM_DRIVER_ERROR_BUSY;
      }
      if (UART0_STATUS.tx_busy == 1U)
      {
        return ARM_DRIVER_ERROR_BUSY;
      }
    }
  }
  else
  {
    uart_base = UART1_REG_BASE;
    if ((control != ARM_USART_ABORT_RECEIVE) && (control != ARM_USART_ABORT_SEND) && (control != ARM_USART_CONTROL_BREAK))
    {
      if (UART1_STATUS.rx_busy == 1U)
      {
        return ARM_DRIVER_ERROR_BUSY;
      }
      if (UART1_STATUS.tx_busy == 1U)
      {
        return ARM_DRIVER_ERROR_BUSY;
      }
    }
  }
  if (1U == ((control >> TC956X_FOUR) & 0x1U))
  {
    if ((control < TC956X_TWENTYONE) || (control > TC956X_TWENTYSIX))
    {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    else if (control > TC956X_TWENTYTHREE)
    {
      switch (control)
      {
        case ARM_USART_ABORT_RECEIVE:
          regVal = hw_reg_read32(uart_base, UART_CR);
          hw_reg_write32(uart_base, UART_CR, regVal & ~(((uint32_t)0x1U)<<UART_CR_RXE_BIT_POS));
          if(UART_id == 0U)
          {
            UART0_STATUS.rx_busy = 0;
            UART0_resource.rxe = 0;
          }
          else
          {
            UART1_STATUS.rx_busy = 0;
            UART1_resource.rxe = 0;
          }
          break;

        case ARM_USART_ABORT_SEND:
          regVal = hw_reg_read32(uart_base, UART_CR);
          hw_reg_write32(uart_base, UART_CR, regVal & ~(((uint32_t)0x1U)<<UART_CR_TXE_BIT_POS));
          if(UART_id == 0U)
          {
            UART0_STATUS.tx_busy = 0;
            UART0_resource.txe = 0;
          }
          else
          {
            UART1_STATUS.tx_busy = 0;
            UART1_resource.txe = 0;
          }
          break;

        default :
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      return ARM_DRIVER_OK;
    }
    else
    {
      if(arg > 1U)
      {
        return ARM_DRIVER_ERROR_PARAMETER;
      }
      else
      {
        switch (control)
        {
          case ARM_USART_CONTROL_BREAK:
            if(arg == 1U)
            {
              regVal = hw_reg_read32(uart_base, UART_LCR_H);
              hw_reg_write32(uart_base, UART_LCR_H, regVal | TC956X_ONE);
            }
            else
            {
              regVal = hw_reg_read32(uart_base, UART_LCR_H);
              hw_reg_write32(uart_base, UART_LCR_H, regVal & ~(0x1U));
            }
            break;

          case ARM_USART_CONTROL_RX:
            if(arg == 1U)
            {
              regVal = hw_reg_read32(uart_base, UART_CR);
              hw_reg_write32(uart_base, UART_CR, (regVal | (((uint32_t)0x1U)<<UART_CR_RXE_BIT_POS)));
              if(UART_id == 0U)
              {
                UART0_resource.rxe = 1;
              }
              else
              {
                UART1_resource.rxe = 1;
              }
            }
            else
            {
              regVal = hw_reg_read32(uart_base, UART_CR);
              hw_reg_write32(uart_base, UART_CR, regVal & ~(((uint32_t)0x1U)<<UART_CR_RXE_BIT_POS));
              if(UART_id == 0U)
              {
                UART0_resource.rxe = 0;
              }
              else
              {
                UART1_resource.rxe = 0;
              }
            }
            break;

          default: /*case ARM_USART_CONTROL_TX:*/
            if(arg == 1U)
            {
              regVal = hw_reg_read32(uart_base, UART_CR);
              hw_reg_write32(uart_base, UART_CR, (regVal | (((uint32_t)0x1U)<<UART_CR_TXE_BIT_POS)));
              if(UART_id == 0U)
              {
                UART0_resource.txe = 1;
              }
              else
              {
                UART1_resource.txe = 1;
              }
            }
            else
            {
              regVal = hw_reg_read32(uart_base, UART_CR);
              hw_reg_write32(uart_base, UART_CR, regVal & ~(((uint32_t)0x1U)<<UART_CR_TXE_BIT_POS));
              if(UART_id == 0U)
              {
                UART0_resource.txe = 0;
              }
              else
              {
                UART1_resource.txe = 0;
              }
            }
            break;
        }
        return ARM_DRIVER_OK;
      }
    }
  }
  else
  {
    if ((control & TC956X_FIFTEEN) != 0U)
    {
      if (1U != (control & TC956X_FIFTEEN))
      {
        return ARM_USART_ERROR_MODE;
      }
      else
      {
        if((arg <= UART_MAX_BAUD_RATE) && (arg >= UART_MIN_BAUD_RATE))
        {
          /* Set baurdrate */
          Mclk_Freq = MCLK_FREQ_VAL ; /* UART_CLK */
          Divider =  ( Mclk_Freq / ( TC956X_SIXTEEN * arg )); /* IBRD */
          Remainder = Mclk_Freq % ( TC956X_SIXTEEN * arg );
          Fraction = ( ( ( TC956X_EIGHT * Remainder ) / arg ) >> TC956X_ONE ) ; /*FBRD */
          bd_rate_flag = 1;
        }
        else
        {
          return ARM_USART_ERROR_BAUDRATE;
        }
      }
    }
    data_bit_option     = (uint32_t)(control & ARM_USART_DATA_BITS_Msk);
    parity_option       = (uint32_t)(control & ARM_USART_PARITY_Msk);
    stop_bit_option     = (uint32_t)(control & ARM_USART_STOP_BITS_Msk);
    flow_control        = (uint32_t)(control & ARM_USART_FLOW_CONTROL_Msk);
    clock_polarity      = (uint32_t)(control & ARM_USART_CPOL_Msk);
    clock_phase         = (uint32_t)(control & ARM_USART_CPHA_Msk);

    switch (data_bit_option)
    {
      /*Data bit configuration*/
      case ARM_USART_DATA_BITS_5:
        data_bits = 0x00;
        break;
      case ARM_USART_DATA_BITS_6:
        data_bits = 0x01;
        break;
      case ARM_USART_DATA_BITS_7:
        data_bits = 0x02;
        break;
      case ARM_USART_DATA_BITS_8:
        data_bits = 0x03;
        break;
      default :
        return ARM_USART_ERROR_DATA_BITS;
    }

    switch (parity_option)
    {
      /*Parity bit configuration*/
      case ARM_USART_PARITY_NONE:
        pen = 0;
        eps = 0;
        sps = 0;
        break;
      case ARM_USART_PARITY_EVEN:
        pen = 1;
        eps = 1;
        sps = 0;
        break;
      case ARM_USART_PARITY_ODD:
        pen = 1;
        eps = 0;
        sps = 0;
        break;
      default :
        return ARM_USART_ERROR_PARITY;
    }

    switch (stop_bit_option)
    {
      /*Stop bit configuration*/
      case ARM_USART_STOP_BITS_1:
        two_stop_bits = 0x00;
        break;
      case ARM_USART_STOP_BITS_2:
        two_stop_bits = 0x01;
        break;
      default :
        return ARM_USART_ERROR_STOP_BITS;
    }
    switch (flow_control)
    {
      case ARM_USART_FLOW_CONTROL_NONE:
        break;
      default:
        return ARM_USART_ERROR_FLOW_CONTROL;
    }
    switch (clock_polarity)
    {
      case ARM_USART_CPOL0:
        break;
      default:
        return ARM_USART_ERROR_CPOL;
    }
    switch (clock_phase)
    {
      case ARM_USART_CPHA0:
        break;
      default:
        return ARM_USART_ERROR_CPHA;
    }

  }
/* Update FIFO Level*/
  fifo_lvl_tx = (uint8_t)(((TC956X_USART_TXIFLSEL_1_2 | TC956X_USART_TXIFLSEL_1_4 | TC956X_USART_TXIFLSEL_1_8 | TC956X_USART_TXIFLSEL_3_4 | TC956X_USART_TXIFLSEL_7_8) & (control))>> TC956X_TWENTYONE);
  fifo_lvl_rx = (uint8_t)(((TC956X_USART_RXIFLSEL_1_2 | TC956X_USART_RXIFLSEL_1_4 | TC956X_USART_RXIFLSEL_1_8 | TC956X_USART_RXIFLSEL_3_4 | TC956X_USART_RXIFLSEL_7_8) & (control))>> TC956X_TWENTYFOUR);

  if (1U ==((control>>TC956X_TWENTY)& 0x1U))
  {
    /* The FIFO INT level can be changed only when this bit is 1 */
    switch (fifo_lvl_tx)
    {
      case 0:
        break;
      case 2:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, regVal |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, (regVal&~(TC956X_SEVEN)));
        break;
      case 1:
      case 3:
      case 4:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, regVal |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, (regVal&~(TC956X_SEVEN)) | fifo_lvl_tx);
        break;
      case 5:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, (regVal) |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, (regVal&~(TC956X_SEVEN)) | UART_FIFO_DEFAULT);
        break;
      default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    switch (fifo_lvl_rx)
    {
      case 0:
        break;
      case 2:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, (regVal) |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, (regVal&(TC956X_SEVEN)));
        break;
      case 1:
      case 3:
      case 4:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, (regVal) |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, ((regVal)&(TC956X_SEVEN)) | (fifo_lvl_rx<<TC956X_THREE));
        break;
      case 5:
        regVal = hw_reg_read32(uart_base, UART_LCR_H);
        hw_reg_write32 (uart_base, UART_LCR_H, (regVal) |
                        (TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
        regVal = hw_reg_read32(uart_base, UART_IFLS);
        hw_reg_write32 (uart_base, UART_IFLS, ((regVal)&(TC956X_SEVEN)) | (UART_FIFO_DEFAULT<<UART_RX_FIFO_LVL_SEL_BIT_POS));
        break;
      default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
  }
  else if (1U == ((control>>TC956X_TWENTYSEVEN)& 0x1U))
  { /* To disable FIFO*/
    regVal = hw_reg_read32(uart_base, UART_LCR_H);
    hw_reg_write32 (uart_base, UART_LCR_H, (regVal) & ~(TC956X_UART_ONE<<UART_LCR_H_FEN_BIT_POS));
  }
  else
  {
    switch(fifo_lvl_tx | fifo_lvl_rx)
    {
    case 0:
      break;
    default:
      return ARM_DRIVER_ERROR;
    }
  }
  if (1U == bd_rate_flag)
  {
    hw_reg_write32( uart_base, UART_IBRD, Divider );
    hw_reg_write32( uart_base, UART_FBRD, Fraction );
    bd_rate_flag = 0;
  }
  line_control = (uint16_t)hw_reg_read32 (uart_base, UART_LCR_H);
  line_control &= TC956X_SIXTEEN; /*FIFO bit*/
  hw_reg_write32(uart_base, UART_LCR_H, ((uint32_t)(line_control) | ((uint32_t) pen<<UART_LCR_H_PEN_BIT_POS) |
  ((uint32_t)eps<<UART_LCR_H_EPS_BIT_POS) | ((uint32_t)two_stop_bits<<UART_LCR_H_STP2_BIT_POS) |
  ((uint32_t)data_bits<<UART_LCR_H_WLEN_BIT_POS) | ((uint32_t)sps<<UART_LCR_H_SPS_BIT_POS)));
  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS tc956x_UART_GetStatus  (uint8_t UART_id)
  \brief       Get USART status.
  \param[in]   UART_id UART-ID
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS tc956x_UART_GetStatus  (uint8_t UART_id)
{

  /* Select base address of UART based on UART-ID */
  if (0U == UART_id)
  {
    return UART0_STATUS;
  }
  else
  {
    return UART1_STATUS;
  }
}

/**
  \fn          int32_t tc956x_UART_InterSend (uint8_t UART_id, const uint8_t *data, uint32_t length)
  \brief       Send data to USART transmitter.
  \param[in]   UART_id UART ID - 0/1
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t tc956x_UART_InterSend (uint8_t UART_id, const uint8_t *data, uint32_t length)
{
  uint32_t tx_enable, i;
  uint32_t uart_base, regVal;
  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  /* Select base address of UART based on UART-ID */
  if(UART_id == 0U)
  {
    uart_base = UART0_REG_BASE;
  }
  else if (UART_id == 1U)
  {
    uart_base = UART1_REG_BASE;
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(data == NULL)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((length == 0U) || (length > TC956X_TWOFIFTYFIVE))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  /*Enable TX, if not enabled*/
  tx_enable = hw_reg_read32 (uart_base, UART_CR);
  tx_enable = (tx_enable >> UART_CR_TXE_BIT_POS) & 0x1U;

  if (tx_enable == 0U)
  {
    regVal = hw_reg_read32(uart_base, UART_CR);
    hw_reg_write32 (uart_base, UART_CR, (regVal) | (TC956X_UART_ONE <<UART_CR_TXE_BIT_POS));
  }
  /*Write data in to data register*/
  for (i = 0U; i < length; i++)
  {
    while (1U == ((hw_reg_read32(uart_base, UART_FR)>> UART_FR_TXFF_BIT_POS) & 0x1U))
    {
      ;
    }
    hw_reg_write32 (uart_base, UART_DR, data[i]);
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t tc956x_UART_InterReceive (uint8_t UART_id, uint8_t *data, uint32_t length)
  \brief       Receiving data from UART receiver.
  \param[in]   UART_id UART ID - 0/1
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
int32_t tc956x_UART_InterReceive (uint8_t UART_id, uint8_t *data, uint32_t length)
{
  uint32_t rx_enable, i;
  uint32_t uart_base, regVal;
  uint8_t inp;
  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  /* Select base address of UART based on UART-ID */
  if(UART_id == 0U)
  {
    uart_base = UART0_REG_BASE;
  }
  else if (UART_id == 1U)
  {
    uart_base = UART1_REG_BASE;
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(data == NULL)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if ((length == 0U) || (length > TC956X_TWOFIFTYFIVE))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  rx_enable = hw_reg_read32 (uart_base, UART_CR);
  rx_enable = (rx_enable >> UART_CR_RXE_BIT_POS) & 0x1U;
  if (rx_enable == 0U)
  {
    regVal = hw_reg_read32(uart_base, UART_CR);
    hw_reg_write32 (uart_base, UART_CR, (regVal) | (TC956X_UART_ONE << UART_CR_RXE_BIT_POS));
  }
  /*Read data from data register*/
  for (i = 0U; i < length; i++)
  {
    while (1U == ((hw_reg_read32(uart_base, UART_FR)>> UART_FR_RXFE_BIT_POS) & 0x1U))
    {
      ;
    }
    inp = (uint8_t)hw_reg_read32(uart_base, UART_DR);
    data[i] = inp;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          iint32_t tc956x_UART_Initialize(ARM_USART_SignalEvent_t cb_event, UART_CONFIG uart)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   UART_id UART-ID
  \return      \ref execution_status
*/
static int32_t tc956x_UART_Initialize(ARM_USART_SignalEvent_t cb_event, UART_CONFIG uart)
{
  uint32_t uart_intrim, uart_base, regVal;
  uint8_t uartid;
  int32_t config;
  uartid = uart.UART_id;
  if(uart_init[uartid] == 1U)
  {
    return ARM_DRIVER_ERROR;
  }
  if(uartid == 0U)
  {
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS);
    hw_reg_write32 ( TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS, regVal | TC956X_UART0_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, regVal | TC956X_UART0_CLK_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS);
    hw_reg_write32 ( TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS, regVal & ~(TC956X_UART0_CLK_ENABLE));
    uart_base = UART0_REG_BASE;
    cb_event_uart_0 = cb_event;
  }
  else
  {
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS);
    hw_reg_write32 ( TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS, regVal | TC956X_UART1_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, regVal | TC956X_UART1_CLK_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS);
    hw_reg_write32 ( TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS, regVal & ~(TC956X_UART1_CLK_ENABLE));
    uart_base = UART1_REG_BASE;
    cb_event_uart_1 = cb_event;
  }
  uart_init[uartid] = 1;
  hw_reg_write32 (uart_base, UART_CR, 0x0);
  hw_reg_write32 (uart_base, UART_IMSC, 0x0);
  hw_reg_write32 (uart_base, UART_LCR_H, 0x0);
  hw_reg_write32 (uart_base, UART_ICR, TC956X_UART_ICR);

  /*UART configuration*/
  config = tc956x_UART_Control(uartid, ARM_USART_MODE_ASYNCHRONOUS, uart.baudrate);
  if (0 != config)
  {
    return ARM_DRIVER_ERROR;
  }

  if (1U == uart.fen)
  { /* FIFO Enable*/
    regVal = hw_reg_read32(uart_base, UART_LCR_H);
    hw_reg_write32 (uart_base, UART_LCR_H, (regVal) |
    (((uint32_t)0x1U)<<UART_LCR_H_FEN_BIT_POS));
    hw_reg_write32 (uart_base, UART_IFLS, ((uart.rxiflsel<<UART_RX_FIFO_LVL_SEL_BIT_POS) | uart.txiflsel));
  }
  /*Interrupt Enable*/
  if (uartid == 0U)
  {
    regVal = hw_reg_read32(TC956X_REG_BASE, INTMCUMASK2);
    hw_reg_write32(TC956X_REG_BASE, INTMCUMASK2, regVal & ~(1U<<TC956X_SIX));
    NVIC_EnableIRQ(INT_SRC_NBR_UART0);
  }
  else
  {
    regVal = hw_reg_read32(TC956X_REG_BASE, INTMCUMASK2);
    hw_reg_write32(TC956X_REG_BASE, INTMCUMASK2, regVal & ~(1U<<TC956X_SEVEN));
    /* UART1 not supported for TC956X, Following code is commented to avoid compilation error
    NVIC_EnableIRQ(INT_SRC_NBR_UART1);*/
  }
  uart_intrim = (uint32_t)((((uint32_t)uart.oeim)<<UART_INTR_OE_BIT_POS) | (((uint32_t)uart.beim)<<UART_INTR_BE_BIT_POS) |
                          (((uint32_t)uart.peim)<<UART_INTR_PE_BIT_POS) | (uart.feim<<UART_INTR_FE_BIT_POS) |
                            (uart.rtim<<UART_INTR_RT_BIT_POS) | (uart.rxim<<UART_INTR_RX_BIT_POS) |
                            (uart.txim<<UART_INTR_TX_BIT_POS));
  hw_reg_write32 (uart_base, UART_IMSC, uart_intrim);
  /*Enable RXE, TXE and UART*/
  (void)tc956x_UART_Control(uartid, ARM_USART_CONTROL_TX,uart.txen);
  (void)tc956x_UART_Control(uartid, ARM_USART_CONTROL_RX,uart.rxen);
  hw_reg_write32 (uart_base, UART_CR, ((((uint32_t)uart.txen)<<UART_CR_TXE_BIT_POS) |
  ((uint32_t)uart.rxen<<UART_CR_RXE_BIT_POS) |(0x1U<<UART_CR_UART_EN_BIT_POS)));
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t tc956x_UART_Uninitialize (uint8_t UART_id)
  \brief       De-initialize USART Interface.
  \param[in]   UART_id UART-ID
  \return      \ref execution_status
*/
static int32_t tc956x_UART_Uninitialize (uint8_t UART_id)
{
  uint32_t uart_base, regVal;
  if (uart_init[UART_id] == 0U)
  {
    return ARM_DRIVER_ERROR;
  }
  /* Select base address of UART based on UART-ID */
  /*Wait for UART RX function complete*/
  if(UART_id == 0U)
  {
    uart_base = UART0_REG_BASE;
    if (UART0_STATUS.rx_busy != 0U)
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
    if (UART0_STATUS.tx_busy != 0U)
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  else
  {
    uart_base = UART1_REG_BASE;
    if (UART1_STATUS.rx_busy != 0U)
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
    if (UART1_STATUS.tx_busy != 0U)
    {
      return ARM_DRIVER_ERROR_BUSY;
    }
  }
  /*Wait for UART TX function complete*/
  while (1U == ((hw_reg_read32(uart_base, UART_FR)>> UART_FR_BUSY_BIT_POS) & 0x1U))
  {
    ;
  }
  /*Reset all the registers*/
  hw_reg_write32 (uart_base, UART_CR, 0x0);
  hw_reg_write32 (uart_base, UART_IMSC, 0x0);
  hw_reg_write32 (uart_base, UART_LCR_H, 0x0);
  hw_reg_write32 (uart_base, UART_ICR, TC956X_UART_ICR);
  hw_reg_write32 (uart_base, UART_IBRD, 0x0);
  hw_reg_write32 (uart_base, UART_FBRD, 0x0);
  hw_reg_write32 (uart_base, UART_DR, 0x0);
  hw_reg_write32 (uart_base, UART_RSR, 0x0);
  hw_reg_write32 (uart_base, UART_ECR, 0x0);
  hw_reg_write32 (uart_base, UART_FR, 0x0);
  hw_reg_write32 (uart_base, UART_RIS, 0x0);
  hw_reg_write32 (uart_base, UART_ICR, 0x0);

  if (UART_id == 0U)
  {
    UART0_STATUS.tx_busy = 0;
    UART0_STATUS.tx_underflow = 0;
    UART0_STATUS.rx_break = 0;
    UART0_STATUS.rx_busy = 0;
    UART0_STATUS.rx_framing_error = 0;
    UART0_STATUS.rx_overflow = 0;
    UART0_STATUS.rx_parity_error = 0;
    UART0_Cap.event_rx_timeout = 0;
    UART0_Cap.event_tx_complete = 0;
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, regVal & ~(TC956X_UART0_CLK_ENABLE));
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS, regVal | TC956X_UART0_CLK_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS, regVal & ~(TC956X_UART0_ENABLE));
    regVal = hw_reg_read32(TC956X_REG_BASE, INTMCUMASK2);
    hw_reg_write32(TC956X_REG_BASE, INTMCUMASK2, regVal | (TC956X_UART_ONE<<TC956X_SIX));
    NVIC_DisableIRQ(INT_SRC_NBR_UART0);
  }
  else
  {
    UART1_STATUS.tx_busy = 0;
    UART1_STATUS.tx_underflow = 0;
    UART1_STATUS.rx_break = 0;
    UART1_STATUS.rx_busy = 0;
    UART1_STATUS.rx_framing_error = 0;
    UART1_STATUS.rx_overflow = 0;
    UART1_STATUS.rx_parity_error = 0;
    UART1_Cap.event_rx_timeout = 0;
    UART1_Cap.event_tx_complete = 0;
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, regVal & ~(TC956X_UART1_CLK_ENABLE));
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS, regVal | TC956X_UART1_CLK_ENABLE);
    regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS);
    hw_reg_write32 (TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS, regVal & ~(TC956X_UART1_ENABLE));
    regVal = hw_reg_read32(TC956X_REG_BASE, INTMCUMASK2);
    hw_reg_write32(TC956X_REG_BASE, INTMCUMASK2, regVal | (TC956X_UART_ONE<<TC956X_SEVEN));
    /* UART1 not supported for TC956X, Following code is commented to avoid compilation error
    NVIC_DisableIRQ(INT_SRC_NBR_UART1);*/
  }

  uart_init[UART_id] = 0;
  return ARM_DRIVER_OK;
}

/**
  \fn          void UART0_IRQHandler (void)
  \brief       IRQ handler
  \return      NULL
*/
void UART0_IRQHandler (void)
{
  uint32_t intc, regVal;
  intc = hw_reg_read32 (UART0_REG_BASE, UART_RIS);
  intc &= TC956X_UART_RIS;

  if ((intc & TXRIS) == TXRIS)
  {
    /* TX Interrupt Occured, copy the data from tx buffer*/
    /* Clear TX Interrupt */
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, TXRIS);
    /* Copy data from tx buffer till, tx_num*/
    while (UART0_resource.tx_cnt < UART0_resource.tx_num)
    {
      if (1U != ((hw_reg_read32(UART0_REG_BASE, UART_FR)>> TC956X_FIVE) & 0x1U))
      {
        hw_reg_write32 (UART0_REG_BASE, UART_DR, UART0_resource.tx_buf[UART0_resource.tx_cnt]);
        UART0_resource.tx_cnt++;
      }
      else
      {
        break;
      }
    }
    /* Check TX complete */
    if (UART0_resource.tx_cnt == UART0_resource.tx_num)
    {
      UART0_STATUS.tx_busy = 0;
      UART0_Cap.event_tx_complete = 1;
      regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(TXRIS));
      if(cb_event_uart_0 != NULL)
      {
        cb_event_uart_0 (ARM_USART_EVENT_SEND_COMPLETE);
      }
    }
    else
    {
      regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
       hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (TXRIS));
    }
  }

  if ((intc & RXRIS) == RXRIS)
  {/* RX Interrupt Occured, copy the data from FIFO to rx buffer*/
    /* Mask and Clear RX Interrupt */
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, RXRIS);
    /*Copy the data from FIFO to RX buffer */
    while (UART0_resource.rx_cnt < UART0_resource.rx_num)
    {
      if (1U != ((hw_reg_read32(UART0_REG_BASE, UART_FR)>> TC956X_FOUR) & 0x1U))
      {
        UART0_resource.rx_buf[UART0_resource.rx_cnt] = (uint8_t)hw_reg_read32(UART0_REG_BASE, UART_DR);
        UART0_resource.rx_cnt++;
      }
      else
      {
        break;
      }
    }
    /* Reset rx cnt and rx num */
    if (UART0_resource.rx_cnt == UART0_resource.rx_num)
    {
      UART0_STATUS.rx_busy = 0;
      regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(RXRIS));
      if(cb_event_uart_0 != NULL)
      {
        cb_event_uart_0 (ARM_USART_EVENT_RECEIVE_COMPLETE);
      }
    }
    else
    {
      regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (RXRIS));
    }
  }

  if ((intc & RTRIS) == RTRIS)
  { /* Receive Timeout Interrupt Occured */
    /* Mask and Clear the Interrupt */
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, RTRIS);
    /* Reset rx cnt and rx num */
    if (UART0_resource.rx_cnt == UART0_resource.rx_num)
    {
      UART0_STATUS.rx_busy = 0;
    }
    else
    {
      while (UART0_resource.rx_cnt < UART0_resource.rx_num)
      {
        if (1U != ((hw_reg_read32(UART0_REG_BASE, UART_FR)>> TC956X_FOUR) & 0x1U))
        {
          UART0_resource.rx_buf[UART0_resource.rx_cnt] = (uint8_t)hw_reg_read32(UART0_REG_BASE, UART_DR);
          UART0_resource.rx_cnt++;
        }
        else
        {
          break;
        }
      }
      if (UART0_resource.rx_cnt == UART0_resource.rx_num)
      {
        UART0_STATUS.rx_busy = 0;
        if(cb_event_uart_0 != NULL)
        {
          cb_event_uart_0 (ARM_USART_EVENT_RECEIVE_COMPLETE);
        }
      }
      else
      {
        UART0_Cap.event_rx_timeout = 1;
        if(cb_event_uart_0 != NULL)
        {
          cb_event_uart_0 (ARM_USART_EVENT_RX_TIMEOUT);
          regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
          hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (RTRIS));
        }
      }
    }
  }

  if ((intc & FERIS) == FERIS)
  { /*Framing Error*/
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(FERIS));
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, FERIS);
    UART0_STATUS.rx_framing_error = 1;
    /*Callback event*/
    if(cb_event_uart_0 != NULL)
    {
      cb_event_uart_0 (ARM_USART_EVENT_RX_FRAMING_ERROR);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (FERIS));
  }

  if ((intc & PERIS) == PERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(PERIS));
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, PERIS);
    UART0_STATUS.rx_parity_error = 1;
    /*Callback event*/
    if(cb_event_uart_0 != NULL)
    {
      cb_event_uart_0 (ARM_USART_EVENT_RX_PARITY_ERROR);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (PERIS));
  }

  if ((intc & BERIS) == BERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(BERIS));
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, BERIS);
    UART0_STATUS.rx_break = 1;
    /*Callback event*/
    if(cb_event_uart_0 != NULL)
    {
      cb_event_uart_0 (ARM_USART_EVENT_RX_BREAK);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (BERIS));
  }

  if ((intc & OERIS) == OERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) & ~(OERIS));
    hw_reg_write32 (UART0_REG_BASE, UART_ICR, OERIS);
    UART0_STATUS.rx_overflow = 1;
    /*Callback event*/
    if (cb_event_uart_0 != NULL)
    {
      cb_event_uart_0 (ARM_USART_EVENT_RX_OVERFLOW);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART0_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART0_REG_BASE, UART_IMSC, (regVal) | (OERIS));
  }
  CM3_Errata_IRQ();
  return;
}

/**
  \fn          void UART1_IRQHandler (void)
  \brief       IRQ handler
  \return      NULL
*/
void UART1_IRQHandler (void)
{
  uint32_t intc, regVal;
  intc = hw_reg_read32 (UART1_REG_BASE, UART_RIS);
  intc &= TC956X_UART_RIS;

  if ((intc & TXRIS) == TXRIS)
  {
    /* TX Interrupt Occured, copy the data from tx buffer*/
    /* Clear TX Interrupt */
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, TXRIS);
    /* Copy data from tx buffer till, tx_num*/
    while (UART1_resource.tx_cnt < UART1_resource.tx_num)
    {
      if (1U != ((hw_reg_read32(UART1_REG_BASE, UART_FR)>> TC956X_FIVE) & 0x1U))
      {
        hw_reg_write32 (UART1_REG_BASE, UART_DR, UART1_resource.tx_buf[UART1_resource.tx_cnt]);
        UART1_resource.tx_cnt++;
      }
      else
      {
        break;
      }
    }
    /* Check TX complete */
    if (UART1_resource.tx_cnt == UART1_resource.tx_num)
    {
      UART1_STATUS.tx_busy = 0;
      UART1_Cap.event_tx_complete = 1;
      regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(TXRIS));
      if(cb_event_uart_1 != NULL)
      {
        cb_event_uart_1 (ARM_USART_EVENT_SEND_COMPLETE);
      }
    }
    else
    {
      regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
       hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (TXRIS));
    }
  }

  if ((intc & RXRIS) == RXRIS)
  {
    /* RX Interrupt Occured, copy the data from FIFO to rx buffer
     Mask and Clear RX Interrupt */
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, RXRIS);
    /*Copy the data from FIFO to RX buffer */
    while (UART1_resource.rx_cnt < UART1_resource.rx_num)
    {
      if (1U != ((hw_reg_read32(UART1_REG_BASE, UART_FR)>> TC956X_FOUR) & 0x1U))
      {
        UART1_resource.rx_buf[UART1_resource.rx_cnt] = (uint8_t)hw_reg_read32(UART1_REG_BASE, UART_DR);
        UART1_resource.rx_cnt++;
      }
      else
      {
        break;
      }
    }
    /* Reset rx cnt and rx num */
    if (UART1_resource.rx_cnt == UART1_resource.rx_num)
    {
      UART1_STATUS.rx_busy = 0;
      regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(RXRIS));
      if(cb_event_uart_1 != NULL)
      {
        cb_event_uart_1 (ARM_USART_EVENT_RECEIVE_COMPLETE);
      }
    }
    else
    {
      regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
      hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (RXRIS));
    }
  }

  if ((intc & RTRIS) == RTRIS)
  {
    /* Receive Timeout Interrupt Occured */
    /* Mask and Clear the Interrupt */
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, RTRIS);
    /* Reset rx cnt and rx num */
    if (UART1_resource.rx_cnt == UART1_resource.rx_num)
    {
      UART1_STATUS.rx_busy = 0;
    }
    else
    {
      while (UART1_resource.rx_cnt < UART1_resource.rx_num)
      {
        if (1U != ((hw_reg_read32(UART1_REG_BASE, UART_FR)>> TC956X_FOUR) & 0x1U))
        {
          UART1_resource.rx_buf[UART1_resource.rx_cnt] = (uint8_t)hw_reg_read32(UART1_REG_BASE, UART_DR);
          UART1_resource.rx_cnt++;
        }
        else
        {
          break;
        }
      }
      if (UART1_resource.rx_cnt == UART1_resource.rx_num)
      {
        UART1_STATUS.rx_busy = 0;
        if(cb_event_uart_1 != NULL)
        {
          cb_event_uart_1 (ARM_USART_EVENT_RECEIVE_COMPLETE);
        }
      }
      else
      {
        UART1_Cap.event_rx_timeout = 1;
        if(cb_event_uart_1 != NULL)
        {
          cb_event_uart_1 (ARM_USART_EVENT_RX_TIMEOUT);
        }
      }
    }
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (RTRIS));
  }

  if ((intc & FERIS) == FERIS)
  {
    /*Framing Error*/
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(FERIS));
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, FERIS);
    UART1_STATUS.rx_framing_error = 1;
    /*Callback event*/
    if(cb_event_uart_1 != NULL)
    {
      cb_event_uart_1 (ARM_USART_EVENT_RX_FRAMING_ERROR);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (FERIS));
  }

  if ((intc & PERIS) == PERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(PERIS));
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, PERIS);
    UART1_STATUS.rx_parity_error = 1;
    /*Callback event*/
    if(cb_event_uart_1 != NULL)
    {
      cb_event_uart_1 (ARM_USART_EVENT_RX_PARITY_ERROR);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (PERIS));
  }

  if ((intc & BERIS) == BERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(BERIS));
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, BERIS);
    UART1_STATUS.rx_break = 1;
    /*Callback event*/
    if(cb_event_uart_1 != NULL)
    {
      cb_event_uart_1 (ARM_USART_EVENT_RX_BREAK);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (BERIS));
  }

  if ((intc & OERIS) == OERIS)
  {
    /* Mask and Clear the Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) & ~(OERIS));
    hw_reg_write32 (UART1_REG_BASE, UART_ICR, OERIS);
    UART1_STATUS.rx_overflow = 1;
    /*Callback event*/
    if (cb_event_uart_1 != NULL)
    {
      cb_event_uart_1 (ARM_USART_EVENT_RX_OVERFLOW);
    }
    /* Unmask Interrupt */
    regVal = hw_reg_read32(UART1_REG_BASE, UART_IMSC);
    hw_reg_write32 (UART1_REG_BASE, UART_IMSC, (regVal) | (OERIS));
  }
  CM3_Errata_IRQ();
  return;
}

/**
  \fn          int32_t USARTX_PowerControl (ARM_POWER_STATE state)
  \brief       Control USART Interface Power - Not Supported.
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USARTX_PowerControl (ARM_POWER_STATE state)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t USARTX_Transfer (const void *data_out, void *data_in, uint32_t num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t USARTX_Transfer (const void *data_out, void *data_in, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t USARTX_SetModemControl (ARM_USART_MODEM_CONTROL control)
  \brief       Set USART Modem Control line state.
  \param[in]   control  \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t USARTX_SetModemControl (ARM_USART_MODEM_CONTROL control)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          ARM_USART_MODEM_STATUS USARTX_GetModemStatus (void)
  \brief       Get USART Modem Status lines state.
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USARTX_GetModemStatus (void)
{
  ARM_USART_MODEM_STATUS UARTX = {
    0,  /*CTS*/
    0,  /*DSR*/
    0,  /*DCD*/
    0,  /*RI*/
    0   /*Recerved*/
  };
  return UARTX;
}

/**
  \fn          ARM_DRIVER_VERSION USARTX_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USARTX_GetVersion (void)
{
  /* Driver Version */
  static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,      /* defined in Driver_USART.h */
    ARM_USART_DRV_VERSION
  };

  return DriverVersion;
}

/**
  \fn          ARM_USART_CAPABILITIES USART0_GetCapabilities (void)
  \brief       Get driver capabilities
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART0_GetCapabilities (void)
{
  return USART_GetCapabilities (UART0);
}

/**
  \fn          int32_t USART0_Initialize (ARM_USART_SignalEvent_t cb_event)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t USART0_Initialize (ARM_USART_SignalEvent_t cb_event)
{
  UART_CONFIG UART0_Config = {
    UART0,      /* UART-ID*/
    UART_BAUDRATE,  /*Baudrate */
    1,        /*TX Enable*/
    1,        /*RX Enable*/
    1,        /*FIFO Enable*/
    0,        /*RX FIFO Level selection*/
    0,        /*TX FIFO Level selection*/
    1,        /*Overrun error interrupt mask*/
    1,        /*Break error interrupt mask*/
    1,        /*Parity error interrupt mask*/
    1,        /*Framing error interrupt mask*/
    1,        /*Receive time out interrupt mask*/
    0,        /*Transmit interrupt mask*/
    0         /*Receive interrupt mask*/
  };
  return tc956x_UART_Initialize (cb_event, UART0_Config);
}

/**
  \fn          int32_t USART0_Uninitialize (void)
  \brief       De-initialize USART Interface.
  \return      \ref execution_status
*/
static int32_t USART0_Uninitialize (void)
{
  return tc956x_UART_Uninitialize(UART0);
}

/**
  \fn          int32_t USART0_Send (const void *data, uint32_t num)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t USART0_Send (const void *data, uint32_t num)
{
  return tc956x_UART_Send (data, num, UART0);
}

/**
  \fn          int32_t USART0_Receive (void *data, uint32_t num)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t USART0_Receive (void *data, uint32_t num)
{
  return tc956x_UART_Receive (data, num, UART0);
}


/**
  \fn          uint32_t USART0_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
static uint32_t USART0_GetTxCount (void)
{
  return tc956x_UART_GetTxCount(UART0);
}

/**
  \fn          uint32_t USART0_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received
*/
static uint32_t USART0_GetRxCount (void)
{
  return tc956x_UART_GetRxCount(UART0);
}

/**
  \fn          int32_t USART0_Control (uint32_t control, uint32_t arg)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART0_Control (uint32_t control, uint32_t arg)
{
  return tc956x_UART_Control (UART0, control, arg);
}

/**
  \fn          ARM_USART_STATUS USART0_GetStatus (void)
  \brief       Get USART status.
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART0_GetStatus (void)
{
  return tc956x_UART_GetStatus(UART0);
}

/**
  \brief Access structure of the USART0 Driver.
*/
ARM_DRIVER_USART Driver_USART0 = {
  USARTX_GetVersion,
  USART0_GetCapabilities,
  USART0_Initialize,
  USART0_Uninitialize,
  USARTX_PowerControl,
  USART0_Send,
  USART0_Receive,
  USARTX_Transfer,
  USART0_GetTxCount,
  USART0_GetRxCount,
  USART0_Control,
  USART0_GetStatus,
  USARTX_SetModemControl,
  USARTX_GetModemStatus
};

/**
  \fn          ARM_USART_CAPABILITIES USART1_GetCapabilities (void)
  \brief       Get driver capabilities
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART1_GetCapabilities (void)
{
  return USART_GetCapabilities (UART1);
}

/**
  \fn          int32_t USART1_Initialize (ARM_USART_SignalEvent_t cb_event)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t USART1_Initialize (ARM_USART_SignalEvent_t cb_event)
{
  UART_CONFIG UART1_Config = {
    UART1,      /* UART-ID*/
    UART_BAUDRATE,  /*Baudrate */
    1,        /*TX Enable*/
    1,        /*RX Enable*/
    1,        /*FIFO Enable*/
    0,        /*RX FIFO Level selection*/
    0,        /*TX FIFO Level selection*/
    1,        /*Overrun error interrupt mask*/
    1,        /*Break error interrupt mask*/
    1,        /*Parity error interrupt mask*/
    1,        /*Framing error interrupt mask*/
    1,        /*Receive time out interrupt mask*/
    0,        /*Transmit interrupt mask*/
    0         /*Receive interrupt mask*/
  };
  return tc956x_UART_Initialize (cb_event, UART1_Config);
}

/**
  \fn          int32_t USART1_Uninitialize (void)
  \brief       De-initialize USART Interface.
  \return      \ref execution_status
*/
static int32_t USART1_Uninitialize (void)
{
  return tc956x_UART_Uninitialize(UART1);
}

/**
  \fn          int32_t USART1_Send (const void *data, uint32_t num)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t USART1_Send (const void *data, uint32_t num)
{
  return tc956x_UART_Send (data, num, UART1);
}

/**
  \fn          int32_t USART1_Receive (void *data, uint32_t num)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t USART1_Receive (void *data, uint32_t num)
{
  return tc956x_UART_Receive (data, num, UART1);
}

/**
  \fn          uint32_t USART1_GetTxCount (void)
  \brief       Get transmitted data count.
  \return      number of data items transmitted
*/
static uint32_t USART1_GetTxCount (void)
{
  return tc956x_UART_GetTxCount(UART1);
}

/**
  \fn          uint32_t USART1_GetRxCount (void)
  \brief       Get received data count.
  \return      number of data items received
*/
static uint32_t USART1_GetRxCount (void)
{
  return tc956x_UART_GetRxCount(UART1);
}

/**
  \fn          int32_t USART1_Control (uint32_t control, uint32_t arg)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART1_Control (uint32_t control, uint32_t arg)
{
  return tc956x_UART_Control (UART1, control, arg);
}

/**
  \fn          ARM_USART_STATUS USART1_GetStatus (void)
  \brief       Get USART status.
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART1_GetStatus (void)
{
  return tc956x_UART_GetStatus(UART1);
}

/**
  \brief Access structure of the USART1 Driver.
*/
ARM_DRIVER_USART Driver_USART1 = {
  USARTX_GetVersion,
  USART1_GetCapabilities,
  USART1_Initialize,
  USART1_Uninitialize,
  USARTX_PowerControl,
  USART1_Send,
  USART1_Receive,
  USARTX_Transfer,
  USART1_GetTxCount,
  USART1_GetRxCount,
  USART1_Control,
  USART1_GetStatus,
  USARTX_SetModemControl,
  USARTX_GetModemStatus
};

/**
  \fn          int32_t  TC956X_Ser_Printf (char *format, ...)
  \brief       Transmit the input string.
  \param[in]   format data ponter to transmit.
  \return      USART status \ref ARM_USART_STATUS
*/
int32_t TC956X_Ser_Printf (char *format, ...)
{
  int32_t retval;
  uint8_t  buffer[255u + 1u];
  va_list   vArgs;
  uint32_t len = 0;

  if(format == NULL)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  va_start(vArgs, format);
  (void)vsnprintf((char *)buffer, 256U, (const char *)format, vArgs);
  va_end(vArgs);

  while (buffer[len])
  {
    len++;
  }

  retval = tc956x_UART_InterSend(UART_DEGUG, (const uint8_t *) buffer, len);

  return retval;
}

