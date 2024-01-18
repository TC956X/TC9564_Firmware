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
 *  8 Jul 2020  : Initial Version
 *  23 Feb 2021 : Updated for error returns
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x_gpio_driver.c
 *
 *********************************************************************************************************
 */

/*====================================================================
INCLUDE FILES
==================================================================== */
#include "tc956x_gpio_driver.h"
#include "tc956x_intc_driver.h"

/*====================================================================
static function prototype declaration
==================================================================== */
static ARM_DRIVER_VERSION tc956x_GPIO_GetVersion (void);
static void tc956x_GPIO_ConfigReg (uint32_t Addr_Offs, uint32_t config, uint8_t flag);
static int8_t tc956x_GPIO_Initialize (void);
static int8_t tc956x_GPIO_ConfigOutput (uint8_t gpio_pin);
static int8_t tc956x_GPIO_ConfigInput (uint8_t gpio_pin);
static int8_t tc956x_GPIO_OutputData (uint8_t gpio_pin, uint8_t flag);
static int8_t tc956x_GPIO_InputData (uint8_t gpio_pin);
static int8_t tc956x_GPIO_Uninitialize (void);
static int8_t tc956x_GPIO_CheckArgFlag (uint8_t data);
static int8_t tc956x_GPIO_ConfigIntr (uint8_t pinNum, uint8_t config);

static int32_t gpio_init;
static int8_t gpio_dir[37];
static Enable_gpio0 gpio0_mem, *gpio0;
static Enable_gpio1 gpio1_mem, *gpio1;

static ARM_DRIVER_VERSION tc956x_GPIO_GetVersion (void)
{
  /* Driver Version */
  static ARM_DRIVER_VERSION DriverVersion = {
    ARM_GPIO_API_VERSION,
    ARM_GPIO_DRV_VERSION
  };
  return DriverVersion;
}

/**
  \fn          int8_t tc956x_GPIO_Initialize (void)
  \brief       Initialize GPIO Driver
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_Initialize (void)
{
  const reg_pinMuxCtrl0 *pinMux0;
  const reg_pinMuxCtrl1 *pinMux1;
  const reg_pinMuxCtrl2 *pinMux2;
  const reg_pinMuxCtrl3 *pinMux3;
  const reg_pinMuxCtrl4 *pinMux4;
  const reg_pinMuxCtrl5 *pinMux5;
  const reg_pinMuxCtrl6 *pinMux6;
  const reg_pinMuxCtrl7 *pinMux7;
  uint32_t pinMuxC1;
  uint32_t pinMuxC2;
  uint32_t pinMuxC3;
  uint32_t pinMuxC4;
  uint32_t pinMuxC5;
  uint32_t pinMuxC6;
  uint32_t pinMuxC7;
  uint32_t pinMuxC0;

  if (gpio_init == 1)
  {
    return ARM_DRIVER_ERROR;
  }
  /*Read the pinMux value*/
  pinMuxC1 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN1_OFFS);
  pinMuxC2 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN2_OFFS);
  pinMuxC3 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN3_OFFS);
  pinMuxC4 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN4_OFFS);
  pinMuxC5 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN5_OFFS);
  pinMuxC6 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN6_OFFS);
  pinMuxC7 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN7_OFFS);
  pinMuxC0 = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN0_OFFS);

  /* Store pinMux value to Structures */
  pinMux1 = (reg_pinMuxCtrl1 *)&(pinMuxC1);
  pinMux2 = (reg_pinMuxCtrl2 *)&(pinMuxC2);
  pinMux3 = (reg_pinMuxCtrl3 *)&(pinMuxC3);
  pinMux4 = (reg_pinMuxCtrl4 *)&(pinMuxC4);
  pinMux5 = (reg_pinMuxCtrl5 *)&(pinMuxC5);
  pinMux6 = (reg_pinMuxCtrl6 *)&(pinMuxC6);
  pinMux7 = (reg_pinMuxCtrl7 *)&(pinMuxC7);
  pinMux0 = (reg_pinMuxCtrl0 *)&(pinMuxC0);

  gpio0 = &gpio0_mem;
  gpio1 = &gpio1_mem;
  if (pinMux4->GPIO0 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_ZERO);
  }
  if (pinMux4->GPIO1 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_ONE);
  }
  if (pinMux4->GPIO2 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWO);
  }
  if (pinMux4->GPIO3 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_THREE);
  }
  if (pinMux4->GPIO4 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_FOUR);
  }
  if (pinMux4->GPIO5 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_FIVE);
  }
  if (pinMux4->GPIO6 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_SIX);
  }
  if (pinMux6->INT0 == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_NINE);
  }
  if (pinMux6->GPIO12 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWELVE);
  }
  if (pinMux7->REFCLK == TC956X_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_THIRTEEN);
  }
  if (pinMux5->SIN0 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TEN);
  }
  if (pinMux5->SOUT0 == TC956X_ZERO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_ELEVEN);
  }
  if (pinMux3->ETH_1_MDIO == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYNINE);
  }
  if (pinMux3->ETH_1_MDC == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_THIRTY);
  }
  if (pinMux3->ETH_1_INT_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_THIRTYONE);
  }
  if (pinMux3->ETH_0_MDIO == TC956X_ONE)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC956X_GPIO_ONE << GPIO_ZERO); /* 32 */
  }
  if (pinMux3->ETH_0_MDC == TC956X_ONE)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC956X_GPIO_ONE << GPIO_ONE); /* 33 */
  }
  if (pinMux3->ETH_0_INT_N == TC956X_ONE)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC956X_GPIO_ONE << GPIO_TWO); /* 34 */
  }
  if (pinMux0->JTAG == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (GPIO_PIN_SELECT);/* gpio pins 7, 8, 14, 15, 16, 17, 18 */
  }
  if (pinMux2->PCIe_A_RESET_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_NINETEEN);
  }
  if (pinMux2->PCIe_A_CLKREQ_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYTWO);
  }
  if (pinMux2->PCIe_B_CLKREQ_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYTHREE);
  }
  if (pinMux2->PCIe_C_CLKREQ_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYFOUR);
  }
  if (pinMux1->GPIO35 == TC956X_ZERO)
  {
    gpio1->gpio_pins1 = gpio1->gpio_pins1 | (TC956X_GPIO_ONE << GPIO_THREE); /* 35 */
  }
  if (pinMux1->GPIO36 == TC956X_ZERO)
  {
    gpio1->gpio_pins1 = ((gpio1->gpio_pins1) | (TC956X_GPIO_ONE << GPIO_FOUR)); /* 36 */
  }
  if (pinMux0->SPIO_CLK == TC956X_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYEIGHT);
  }
  if (pinMux0->SPIO_MOSI == TC956X_TWO)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYSEVEN);
  }
  if (pinMux0->SPIO_SS_N == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYFIVE);
  }
  if (pinMux0->SPIO_MISO == TC956X_ONE)
  {
    gpio0->gpio_pins0 = gpio0->gpio_pins0 | (TC956X_GPIO_ONE << GPIO_TWENTYSIX);
  }
  gpio_init = 1;
  return ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc956x_GPIO_ConfigOutput(uint8_t gpio_pin)
  \brief       Config the gpio pin as output pin
  \param[in]   gpio_pin [0-36]
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_ConfigOutput (uint8_t gpio_pin)
{
  uint32_t config;
  uint32_t regVal;
  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }

  if((GPIO_TWENTY == gpio_pin) || (GPIO_TWENTYONE == gpio_pin))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (gpio_pin < GPIO_THIRTYTWO)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < GPIO_THIRTYSEVEN)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - GPIO_THIRTYTWO)) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(gpio_pin < GPIO_THIRTYTWO)
  {
    config = ~(TC956X_GPIO_ONE << gpio_pin);
    regVal = hw_reg_read32(TC956X_REG_BASE, GPIO0_ENABLE);
    hw_reg_write32(TC956X_REG_BASE, GPIO0_ENABLE, regVal & config);
  }
  else
  {
    config = ~(TC956X_GPIO_ONE << (gpio_pin - GPIO_THIRTYTWO));
    regVal = hw_reg_read32(TC956X_REG_BASE, GPIO1_ENABLE);
    hw_reg_write32(TC956X_REG_BASE, GPIO1_ENABLE, regVal & config);
  }
  gpio_dir[gpio_pin] = 1;
  return (int8_t)ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc956x_GPIO_OutputData(uint8_t  gpio_pin, uint8_t flag)
  \brief       Write the output value in gpio pin
  \param [in]  gpio_pin [0-36]
  \param [in]  flag HIGH/LOW
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_OutputData (uint8_t  gpio_pin, uint8_t flag)
{
  uint8_t status;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }

  if((GPIO_TWENTY == gpio_pin) || (GPIO_TWENTYONE == gpio_pin))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (gpio_pin < GPIO_THIRTYTWO)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < GPIO_THIRTYSEVEN)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - GPIO_THIRTYTWO)) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (gpio_dir[gpio_pin] != 0x1)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  status = (uint8_t)tc956x_GPIO_CheckArgFlag(flag);
  if(status == GPIO_SUCCESS)
  {
    if(gpio_pin < GPIO_THIRTYTWO)
    {
      tc956x_GPIO_ConfigReg(GPIO0_OUT_REG, TC956X_GPIO_ONE << gpio_pin, flag);
    }
    else
    {
      tc956x_GPIO_ConfigReg(GPIO1_OUT_REG, TC956X_GPIO_ONE << (gpio_pin - GPIO_THIRTYTWO), flag);
    }
  }
  return (int8_t)status;
}

/**
  \fn          void tc956x_GPIO_ConfigReg(uint32_t Addr_Offs, uint32_t config, uint8_t flag)
  \brief       Write data into gpio register
  \param [in]  Addr_Offs GPIO register offset
  \param [in]  config Value on corresponding bit of gpio pin
  \param [in]  flag HIGH/LOW
  \return      NULL
*/
static void tc956x_GPIO_ConfigReg (uint32_t Addr_Offs, uint32_t config, uint8_t flag)
{
  uint32_t data;
  data = hw_reg_read32(TC956X_REG_BASE, Addr_Offs);
  if(flag == TC956X_ONE)
  {
    data |= config;
  }
  else
  {
    data &= ~(config);
  }
  hw_reg_write32(TC956X_REG_BASE, Addr_Offs, data);
}

/**
  \fn          int8_t tc956x_GPIO_ConfigInput(uint8_t gpio_pin)
  \brief       Config the gpio pin as input pin
  \param [in]  gpio_pin [0-36]
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_ConfigInput (uint8_t gpio_pin)
{
  uint32_t config;
  uint32_t regVal;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }

  if((GPIO_TWENTY == gpio_pin) || (GPIO_TWENTYONE == gpio_pin))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (gpio_pin < GPIO_THIRTYTWO)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < GPIO_THIRTYSEVEN)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - GPIO_THIRTYTWO)) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if(gpio_pin < GPIO_THIRTYTWO)
  {
    config = (TC956X_GPIO_ONE << gpio_pin);
    regVal = hw_reg_read32(TC956X_REG_BASE, GPIO0_ENABLE);
    hw_reg_write32(TC956X_REG_BASE, GPIO0_ENABLE, regVal | config);
  }
  else
  {
    config = (TC956X_GPIO_ONE << (gpio_pin - GPIO_THIRTYTWO));
    regVal = hw_reg_read32(TC956X_REG_BASE, GPIO1_ENABLE);
    hw_reg_write32(TC956X_REG_BASE, GPIO1_ENABLE, regVal | config);
  }
  gpio_dir[gpio_pin] = 0;
  return (int8_t)ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc956x_GPIO_InputData (uint8_t gpio_pin)
  \brief       Read the data from input pin
  \param [in]  gpio_pin [0-36]
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_InputData (uint8_t gpio_pin)
{
  uint32_t ret;

  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }

  if((GPIO_TWENTY == gpio_pin) || (GPIO_TWENTYONE == gpio_pin))
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (gpio_pin < GPIO_THIRTYTWO)
  {
    if ((((gpio0->gpio_pins0) >> gpio_pin) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else if (gpio_pin < GPIO_THIRTYSEVEN)
  {
    if ((((gpio1->gpio_pins1) >> (gpio_pin - GPIO_THIRTYTWO)) & 0x1U) == 0U)
    {
      return ARM_DRIVER_ERROR;
    }
  }
  else
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if (gpio_dir[gpio_pin] != 0x0)
  {
    return ARM_DRIVER_ERROR_PARAMETER;
  }
  if(gpio_pin < GPIO_THIRTYTWO)
  {
    ret = hw_reg_read32(TC956X_REG_BASE, GPIO0_REG);
    ret = (ret >> gpio_pin)& 0x1U;
  }
  else
  {
    ret = hw_reg_read32(TC956X_REG_BASE, GPIO1_REG);
    ret = (ret >> (gpio_pin - GPIO_THIRTYTWO))& 0x1U;
  }
  return (int8_t)ret;
}

/**
  \fn          int8_t tc956x_GPIO_CheckArgFlag(uint8_t data)
  \brief       Check the argument flag as valid
  \param [in]  data Flag
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_CheckArgFlag (uint8_t data)
{
  int8_t status;
  if ((data == GPIO_HIGH) || (data == GPIO_LOW))
  {
    status = GPIO_SUCCESS;
  }
  else
  {
    status = ARM_DRIVER_ERROR_PARAMETER;  /*Invalid argument. argument other than 0 or 1.*/
  }
  return status;
}

/**
  \fn          int8_t tc956x_GPIO_Uninitialize (void)
  \brief       De-initialize GPIO Interface.
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_Uninitialize (void)
{
  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }
  else
  {
    gpio_init = 0;
    gpio0->gpio_pins0 = 0;
    gpio1->gpio_pins1 = 0;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int8_t tc956x_GPIO_ConfigIntr(uint8_t pinNum, uint8_t config)
  \brief       Enable or Disable External Interrupts
  \param [in]  pinNum [Interrupt 0, 1, 2]
  \param [in]  config Enable\Disable
  \return      \ref execution_status
*/
static int8_t tc956x_GPIO_ConfigIntr (uint8_t pinNum, uint8_t config)
{
  uint32_t regVal, regVal2;
  if (gpio_init == 0)
  {
    return ARM_DRIVER_ERROR;
  }

  if(GPIO_IRQ_ENABLE != config)
  {
    switch (pinNum)
    {
      case INTI01:
        NVIC_DisableIRQ(INT_SRC_NBR_INTI01);
        return ARM_DRIVER_OK;
      case INTI02:
        NVIC_DisableIRQ(INT_SRC_NBR_INTI02);
        return ARM_DRIVER_OK;
      case INTI03:
        NVIC_DisableIRQ(INT_SRC_NBR_INTI03);
        return ARM_DRIVER_OK;
      default:
        return ARM_DRIVER_ERROR_PARAMETER;
    }
  }

  switch (pinNum)
  {

    case INTI01:
      /*Set input direction*/
      regVal = hw_reg_read32(TC956X_REG_BASE, GPIO1_ENABLE);
      hw_reg_write32 (TC956X_REG_BASE, GPIO1_ENABLE, (regVal | ((pinNum-GPIO_THIRTYFOUR) << TC956X_THREE)));

      /*Pin MUX register update*/
      regVal = hw_reg_read32(TC956X_REG_BASE, TC956X_NFUNCEN1_OFFS);
      /* Set field to 0b0000, and then set to required function value */
      regVal &= ~(MUXF_CLEAR << GPIO35_POS);
      regVal |= (MUXF2 << GPIO35_POS);
      hw_reg_write32 (TC956X_REG_BASE, TC956X_NFUNCEN1_OFFS, regVal);
      regVal = hw_reg_read32(TC956X_REG_BASE, EXTINTCFG);
      hw_reg_write32 (TC956X_REG_BASE, EXTINTCFG, (regVal | TC956X_TWO));
      /*INTMCUMASK register update*/
      regVal = hw_reg_read32 (TC956X_REG_BASE, INTMCUMASK0);
      regVal &= ~(0x1U << INTI01_POS);
      hw_reg_write32 (TC956X_REG_BASE, INTMCUMASK0, regVal);
      NVIC_EnableIRQ(INT_SRC_NBR_INTI01);
      break;

    case INTI02:
      /*Set input direction*/
      regVal = hw_reg_read32(TC956X_REG_BASE, GPIO1_ENABLE);
      hw_reg_write32 (TC956X_REG_BASE, GPIO1_ENABLE, (regVal | ((pinNum-GPIO_THIRTYFOUR) << TC956X_THREE)));

      /*Pin MUX register update*/
      regVal = hw_reg_read32 (TC956X_REG_BASE, TC956X_NFUNCEN1_OFFS);
      /* Set field to 0b0000, and then set to required function value */
      regVal &= ~(MUXF_CLEAR << GPIO36_POS);
      regVal |= (MUXF2 << GPIO36_POS);
      hw_reg_write32 (TC956X_REG_BASE, TC956X_NFUNCEN1_OFFS, regVal);

      regVal = hw_reg_read32(TC956X_REG_BASE, EXTINTCFG);
      hw_reg_write32 (TC956X_REG_BASE, EXTINTCFG, regVal | (TC956X_TWO << TC956X_TWO));
      /*INTMCUMASK register update*/
      regVal = hw_reg_read32 (TC956X_REG_BASE, INTMCUMASK0);
      regVal &= ~(0x1U << INTI02_POS);
      hw_reg_write32 (TC956X_REG_BASE, INTMCUMASK0, regVal);
      NVIC_EnableIRQ(INT_SRC_NBR_INTI02);
      break;

    case INTI03:
      /*Set input direction*/
      regVal = pinNum;
      regVal2 = hw_reg_read32(TC956X_REG_BASE, GPIO0_ENABLE);
      hw_reg_write32 (TC956X_REG_BASE, GPIO0_ENABLE, (regVal2 | (regVal << TC956X_THIRTEEN)));

      regVal = hw_reg_read32 (TC956X_REG_BASE, TC956X_NFUNCEN7_OFFS);
      /* Set field to 0b0000, and then set to required function value */
      regVal &= ~(MUXF_CLEAR << GPIO13_POS);
      regVal |= (MUXF0 << GPIO13_POS);
      hw_reg_write32 (TC956X_REG_BASE, TC956X_NFUNCEN7_OFFS, regVal);

      /*INTMCUMASK register update*/
      regVal = hw_reg_read32(TC956X_REG_BASE, EXTINTCFG);
      hw_reg_write32 (TC956X_REG_BASE, EXTINTCFG, regVal | (TC956X_TWO << TC956X_FOUR));

      regVal = hw_reg_read32 (TC956X_REG_BASE, INTMCUMASK0);
      regVal &= ~(0x1U << INTI03_POS);
      hw_reg_write32 (TC956X_REG_BASE, INTMCUMASK0, regVal);
      NVIC_EnableIRQ(INT_SRC_NBR_INTI03);
      break;
    default:
      return ARM_DRIVER_ERROR_PARAMETER;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          void INTI01_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI01_IRQHandler (void)
{
  tc956x_GPIO_ConfigReg(EXTINTFLG_OFFS, TC956X_ONE << INTI01_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI01);
  CM3_Errata_IRQ();
}

/**
  \fn          void INTI02_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI02_IRQHandler (void)
{
  tc956x_GPIO_ConfigReg(EXTINTFLG_OFFS, TC956X_ONE << INTI02_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI02);
  CM3_Errata_IRQ();
}

/**
  \fn          void INTI03_IRQHandler(void)
  \brief       IRQ handler
  \return      NULL
*/
void INTI03_IRQHandler (void)
{
  tc956x_GPIO_ConfigReg(EXTINTFLG_OFFS, TC956X_ONE << INTI03_POS, GPIO_HIGH);
  NVIC_ClearPendingIRQ((IRQn_Type)INT_SRC_NBR_INTI03);
  CM3_Errata_IRQ();
}

/**
    \brief Access structure of the GPIO Driver.
*/
TC956X_GPIO_DRIVER tc956x_gpio = {
  tc956x_GPIO_GetVersion,
  tc956x_GPIO_Initialize,
  tc956x_GPIO_Uninitialize,
  tc956x_GPIO_ConfigOutput,
  tc956x_GPIO_ConfigInput,
  tc956x_GPIO_ConfigIntr,
  tc956x_GPIO_OutputData,
  tc956x_GPIO_InputData
};
