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
 * Target      : TC956X
 * Filename    : tc956x_regacc.h
 *
 *********************************************************************************************************
 */

#ifndef _TC956X_REGACC_H_
#define _TC956X_REGACC_H_

#include<stdint.h>

/* ****************************** Macro definition **************************************************/
#define TC956X_REG_BASE           (0x40000000U)

#define hw_reg_write32(addr_base, offset, val) *((volatile unsigned int*)((addr_base) + (offset)))=(val)
#define hw_reg_read32(addr_base, offset) (*((volatile unsigned int*)((addr_base) + (offset))))

#define TC956X_NFUNCEN0_OFFS  (0x0008U)
#define TC956X_NFUNCEN1_OFFS  (0x1514U)
#define TC956X_NFUNCEN2_OFFS  (0x151CU)
#define TC956X_NFUNCEN3_OFFS  (0x1524U)
#define TC956X_NFUNCEN4_OFFS  (0x1528U)
#define TC956X_NFUNCEN5_OFFS  (0x152CU)
#define TC956X_NFUNCEN6_OFFS  (0x1530U)
#define TC956X_NFUNCEN7_OFFS  (0x153CU)
#define TC956X_NMODESTS_OFFS  (0x0004U)

#define TC956X_ZERO          0U
#define TC956X_ONE           1U
#define TC956X_TWO           2U
#define TC956X_THREE         3U
#define TC956X_FOUR          4U
#define TC956X_FIVE          5U
#define TC956X_SIX           6U
#define TC956X_SEVEN         7U
#define TC956X_EIGHT         8U
#define TC956X_NINE          9U
#define TC956X_TEN           10U
#define TC956X_ELEVEN        11U
#define TC956X_TWELVE        12U
#define TC956X_THIRTEEN      13U
#define TC956X_FOURTEEN      14U
#define TC956X_FIFTEEN       15U
#define TC956X_SIXTEEN       16U
#define TC956X_SEVENTEEN     17U
#define TC956X_EIGHTEEN      18U
#define TC956X_NINETEEN      19U
#define TC956X_TWENTY        20U
#define TC956X_TWENTYONE     21U
#define TC956X_TWENTYTWO     22U
#define TC956X_TWENTYTHREE   23U
#define TC956X_TWENTYFOUR    24U
#define TC956X_TWENTYFIVE    25U
#define TC956X_TWENTYSIX     26U
#define TC956X_TWENTYSEVEN   27U
#define TC956X_TWENTYEIGHT   28U
#define TC956X_TWENTYNINE    29U
#define TC956X_THIRTY        30U
#define TC956X_THIRTYONE     31U

#define TC956X_TWOFIFTYFIVE  255U

#define TC956X_NCLKCTRL0_OFFS       0x1004U
#define TC956X_NCLKCTRL1_OFFS       0x100CU
#define TC956X_NRSTCTRL0_OFFS       0x1008U
#define TC956X_NRSTCTRL1_OFFS       0x1010U
#define TC956X_VAL_ZERO             0U
#define TC956X_CLK_CTRL0_ENABLE     0x80047291U
#define TC956X_CLK_CTRL0_DISABLE    0x3201U
#define TC956X_CLK_CTRL1_ENABLE     0x8000C080U
#define TC956X_CLK_CTRL1_DISABLE    0x0

__inline static void iowrite32(uint32_t val, volatile uint32_t *addr)
{
  *addr = val;
}

__inline static unsigned int ioread32(volatile const uint32_t *addr)
{
  volatile uint32_t val = *addr;
  return val;
}

__inline static void mdelay(uint32_t ms)
{
  unsigned int i, cnt = 0;

  cnt = 187500U*ms;  /* 187.5MHz */
  for (i = 0U; i < cnt; i++) {
  ;
  }
  return;
}

__inline static void udelay(uint32_t us)
{
  unsigned int i, cnt = 0;

  cnt = 187U*us;  /* 187.5MHz: Its not an accurate usec */
  for (i = 0U; i < cnt; i++) {
  ;
  }
  return;
}

#endif //_TC956X_REGACC_H_
