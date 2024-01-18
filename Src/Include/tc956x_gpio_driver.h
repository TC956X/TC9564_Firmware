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
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x_gpio_driver.h
 *
 *********************************************************************************************************
 */

#ifndef TC956X_GPIO_DRIVER_H_
#define TC956X_GPIO_DRIVER_H_


/*======================================================================
INCLUDE FILES
===================================================================== */
#include "Driver_Common.h"
#include "tc956x_regacc.h"

/*======================================================================
MACRO DEFINITION
======================================================================*/


#define GPIO_HIGH                   1U
#define GPIO_LOW                    0U

#define GPIO_ZERO                   0
#define GPIO_ONE                    1U
#define GPIO_TWO                    2U
#define GPIO_THREE                  3U
#define GPIO_FOUR                   4U
#define GPIO_FIVE                   5U
#define GPIO_SIX                    6U
#define GPIO_SEVEN                  7U
#define GPIO_EIGHT                  8U
#define GPIO_NINE                   9U  /*GPIO pin number (GPIO09)*/
#define GPIO_TEN                    10U /*GPIO pin number (GPIO10)*/
#define GPIO_ELEVEN                 11U /*GPIO pin number (GPIO11)*/
#define GPIO_TWELVE                 12U
#define GPIO_THIRTEEN               13U
#define GPIO_FOURTEEN               14U
#define GPIO_FIFTEEN                15U
#define GPIO_SIXTEEN                16U
#define GPIO_SEVENTEEN              17U
#define GPIO_EIGHTEEN               18U
#define GPIO_NINETEEN               19U
#define GPIO_TWENTY                 20U
#define GPIO_TWENTYONE              21U
#define GPIO_TWENTYTWO              22U
#define GPIO_TWENTYTHREE            23U
#define GPIO_TWENTYFOUR             24U
#define GPIO_TWENTYFIVE             25U
#define GPIO_TWENTYSIX              26U
#define GPIO_TWENTYSEVEN            27U
#define GPIO_TWENTYEIGHT            28U
#define GPIO_TWENTYNINE             29U
#define GPIO_THIRTY                 30U
#define GPIO_THIRTYONE              31U
#define GPIO_THIRTYTWO              32U /*GPIO pin number (GPIO32)*/
#define GPIO_THIRTYTHREE            33U
#define GPIO_THIRTYFOUR             34U
#define GPIO_THIRTYFIVE             35U
#define GPIO_THIRTYSIX              36U /*GPIO pin number (GPIO36)*/

#define GPIO_THIRTYSEVEN            37U /* MAX GPIO pin number*/

#define GPIO_SUCCESS                0U   /*Successful completion*/
#define ERROR_PIN                   -1U  /*ERROR: pin number that does not exist*/
#define ERROR_FLG                   -2U  /*ERROR: flag that does not exist*/

#define GPIO_IRQ_ENABLE             1U
#define GPIO_IRQ_DISABLE            0U

#define GPIO0_REG                   (0x1200U)
#define GPIO1_REG                   (0x1204U)
#define GPIO0_ENABLE                (0x1208U)
#define GPIO1_ENABLE                (0x120CU)
#define GPIO0_OUT_REG               (0x1210U)
#define GPIO1_OUT_REG               (0x1214U)

#define GPIO_PIN_SELECT             0x7C180U

#define ARM_GPIO_DRV_VERSION         ARM_DRIVER_VERSION_MAJOR_MINOR(1U, 0U)  /* driver version */
#define ARM_GPIO_API_VERSION         ARM_DRIVER_VERSION_MAJOR_MINOR(1U, 0U)

#define MUXF0                       ((uint32_t)0x00U)
#define MUXF1                       ((uint32_t)0x01U)
#define MUXF2                       ((uint32_t)0x02U)
#define MUXF_CLEAR                  ((uint32_t)0xFU)

#define GPIO0_POS                   0U
#define GPIO1_POS                   4U
#define GPIO2_POS                   8U
#define GPIO3_POS                   12U
#define GPIO4_POS                   16U
#define GPIO5_POS                   20U
#define GPIO6_POS                   24U

#define GPIO36_POS                  20U
#define GPIO35_POS                  16U
#define GPIO13_POS                  0U

#define INTI01                      35U
#define INTI02                      36U
#define INTI03                      13U

#define INTI01_POS                  0U
#define INTI02_POS                  1U
#define INTI03_POS                  2U


#define INTMCUMASK0                 (0x8020U)
#define INTEXTMASK0                 (0x8030U)
#define INTINTXMASK0                (0x8040U)
#define EXTINTFLG_OFFS              (0x8014U)
#define EXTINTCFG                   (0x804CU)
#define PMEINTCFG                   (0x8050U)
#define TC956X_GPIO_ONE             ((uint32_t) TC956X_ONE)

typedef struct{
  uint32_t gpio_pins0;
}Enable_gpio0;

typedef struct{
  uint32_t gpio_pins1 :5;
  uint32_t reserved   :27;
}Enable_gpio1;
typedef struct{
  uint32_t res0     :4;
  uint32_t res1     :4;
  uint32_t res2     :4;
  uint32_t res3     :4;
  uint32_t GPIO35   :4;
  uint32_t GPIO36   :4;
  uint32_t res4     :4;
  uint32_t res5     :4;
}reg_pinMuxCtrl1;

typedef struct{
  uint32_t PCIe_A_RESET_N   :4;
  uint32_t res0             :4;
  uint32_t res1             :4;
  uint32_t PCIe_A_CLKREQ_N  :4;
  uint32_t PCIe_B_CLKREQ_N  :4;
  uint32_t PCIe_C_CLKREQ_N  :4;
  uint32_t res2             :4;
  uint32_t res3             :4;
}reg_pinMuxCtrl2;

typedef struct{
  uint32_t ETH_1_MDIO    :4;
  uint32_t ETH_1_MDC     :4;
  uint32_t ETH_1_INT_N   :4;
  uint32_t ETH_0_MDIO    :4;
  uint32_t ETH_0_MDC     :4;
  uint32_t ETH_0_INT_N   :4;
  uint32_t res0          :4;
  uint32_t res1          :4;
}reg_pinMuxCtrl3;

typedef struct{
  uint32_t GPIO0    :4;
  uint32_t GPIO1    :4;
  uint32_t GPIO2    :4;
  uint32_t GPIO3    :4;
  uint32_t GPIO4    :4;
  uint32_t GPIO5    :4;
  uint32_t GPIO6    :4;
  uint32_t res      :4;
}reg_pinMuxCtrl4;

typedef struct{
  uint32_t SOUT0    :4;
  uint32_t SIN0     :4;
  uint32_t res0     :4;
  uint32_t res1     :4;
  uint32_t res2     :4;
  uint32_t res3     :4;
  uint32_t res4     :4;
  uint32_t res5     :4;
}reg_pinMuxCtrl5;

typedef struct{
  uint32_t res0     :4;
  uint32_t INT0     :4;
  uint32_t res1     :4;
  uint32_t GPIO12   :4;
  uint32_t res2     :4;
  uint32_t INTI     :4;
  uint32_t res3     :4;
  uint32_t res4     :4;
}reg_pinMuxCtrl6;

typedef struct{
  uint32_t REFCLK   :4;
  uint32_t res0     :4;
  uint32_t res1     :4;
  uint32_t res2     :4;
  uint32_t res3     :4;
  uint32_t res4     :4;
  uint32_t res5     :4;
  uint32_t res6     :4;
}reg_pinMuxCtrl7;

typedef struct{
  uint32_t SPIO_CLK   :4;
  uint32_t SPIO_MOSI  :4;
  uint32_t SPIO_SS_N  :4;
  uint32_t SPIO_MISO  :4;
  uint32_t JTAG       :4;
  uint32_t res0       :4;
  uint32_t res1       :4;
  uint32_t res2       :4;
}reg_pinMuxCtrl0;

typedef struct GPIO_DRIVER {
  ARM_DRIVER_VERSION (*GetVersion) (void);
  int8_t (*Initialize)(void);
  int8_t (*Uninitialize) (void);
  int8_t (*ConfigOutput) (uint8_t gpio_pin);
  int8_t (*ConfigInput) (uint8_t gpio_pin);
  int8_t (*ConfigInterrupt) (uint8_t pinNum, uint8_t config);
  int8_t (*OutputData) (uint8_t  gpio_pin, uint8_t flag);
  int8_t (*InputData) (uint8_t gpio_pin);
}TC956X_GPIO_DRIVER;

extern TC956X_GPIO_DRIVER tc956x_gpio;

void INTI01_IRQHandler (void);
void INTI02_IRQHandler (void);
void INTI03_IRQHandler (void);

#endif /* _TC956X_GPIO_DRIVER_H_ */
