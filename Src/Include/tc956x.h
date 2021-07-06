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
 *  22 Oct 2020 : EEPROM support
 *  05 Jul 2021  : Used Systick handler instead of Driver kernel timer to process transmitted Tx descriptors.
 *  VERSION      : 1.0.1
 */

#ifndef __TC956X_H__
#define __TC956X_H__

#include "tc956x_regacc.h"

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x.h
 *
 *********************************************************************************************************
 */

/*======================================================================
MACRO DEFINITION
======================================================================*/
/* Debugging count SRAM area start address */
#define TC956X_M3_DBG_CNT_START            0x2000F800U      /* DMEM: 0x2000_0000 - 0x2001_0000 */
/* Debugging count SRAM area size */
#define TC956X_M3_DBG_CNT_SIZE             ( 18U*4U )

#define TC956X_M3_DBG_VER_START            0x2000F900

#define TC956X_M3_MAC_ADDR_START           0x20007000U      /* MAC ADDR storage */
#define TC956X_M3_EEPROM_OFFSET_ADDR       0x20007050U      /* Word address offset of EEPROM */
#define TC956X_M3_MAC_COUNT                0x20007051U      /* Number of MAC addresses to be stored */
#define TC956X_M3_FW_INIT_DONE             0x20007054U      /* Memory for FW init completed */
#define TC956X_M3_FW_EXIT                  0x20007058U      /* Memory for FW Exit when driver unloaded */

#define TC956X_SW_MSI_TRIGGER_TIME         1000      /* 1 second */

/* TC956X_M3_DBG_CNT_START + 4*0:  Reserved
* TC956X_M3_DBG_CNT_START + 4*1:   Reserved
* TC956X_M3_DBG_CNT_START + 4*2:   Reserved
* TC956X_M3_DBG_CNT_START + 4*3:   Reserved
* TC956X_M3_DBG_CNT_START + 4*4:   Reserved
* TC956X_M3_DBG_CNT_START + 4*5:   Reserved
* TC956X_M3_DBG_CNT_START + 4*6:   Reserved
*   .........................
* TC956X_M3_DBG_CNT_START + 4*11:  INTC WDT Expiry Count
* TC956X_M3_DBG_CNT_START + 4*12:  WDT Monitor count
* TC956X_M3_DBG_CNT_START + 4*13:  Reserved
* TC956X_M3_DBG_CNT_START + 4*14:  Reserved
* TC956X_M3_DBG_CNT_START + 4*15:  guiM3Ticks (64bit)
* TC956X_M3_DBG_CNT_START + 4*16:  guiM3Ticks (64 bit)
* TC956X_M3_DBG_CNT_START + 4*17:  Tx Timeout for Port0
* TC956X_M3_DBG_CNT_START + 4*18:  Tx Timeout for Port1
* TC956X_M3_DBG_CNT_START + 4*19:  Reserved
*/

/* 0x20004000 to 0x2000401C is for Port0, 0x20004020 to 0x2000403C is for Port1 */
#define SRAM_TX_PCIE_ADDR_LOC  0x20004000

/* 0x20004040 to 0x2000405C is for Port0, 0x20004060 to 0x2000407C is for Port1 */
#define SRAM_RX_PCIE_ADDR_LOC  0x20004040

/* Total bytes for SRAM location size i.e 0x20004100 - 0x20004000 */
#define SRAM_EVENT_LOC_SIZE   64


#define XGMAC_MAC_OFFSET0                0x40040000U
#define XGMAC_MAC_OFFSET1                0x40048000U
#define XGMAC_DMA_CUR_TxDESC_LADDR(x)   (0x00003144 + (0x80 * (x)))
#define XGMAC_DMA_CUR_RxDESC_LADDR(x)   (0x0000314C + (0x80 * (x)))

#define XGMAC_MTL_TxQ_DEBUG(x)          (0x00001108 + (0x80 * (x)))
#define XGMAC_MTL_TxQ_DEBUG_TXQSTS_MASK	(1 << 4)
#define XGMAC_MTL_TxQ_DEBUG_TXQSTS_SHIFT	(4)

/* M3 registers */
/* SysTick Ctrl & Status Reg.     */
#define CPU_REG_NVIC_ST_CTRL           ( *( ( uint32_t * )( 0xE000E010U ) ) )
/* SysTick Reload      Value Reg. */
#define CPU_REG_NVIC_ST_RELOAD         ( *( ( uint32_t * )( 0xE000E014U ) ) )
/* System Handlers 12 to 15 Prio. */
#define CPU_REG_NVIC_SHPRI3            ( *( ( uint32_t * )( 0xE000ED20U ) ) )

#define TC956X_MSIGEN_REG_BASE           0x4000F000U
#define SW_MSI_SET_PF0                   0x0050U
#define SW_MSI_SET_PF1                   0x0150U

/* Interrupt registers */
#define TC956X_INTC_REG_BASE           0x40008000U
#define MAC0STATUS                     0x000CU
#define MAC1STATUS                     0x0010U
#define INTMCUMASK0_OFFS               0x0020U
#define INTMCUMASK1_OFFS               0x0024U
#define INTMCUMASK2_OFFS               0x0028U
#define INTMCUMASK3_OFFS               0x002CU
#define INTEXTMASK0_OFFS               0x0030U
#define INTEXTMASK1_OFFS               0x0034U
#define INTEXTMASK2_OFFS               0x0038U
#define INTEXTMASK3_OFFS               0x003CU
#define INTINTXMASK0_OFFS              0x0040U
#define INTINTXMASK1_OFFS              0x0044U
#define INTINTXMASK2_OFFS              0x0048U
#define INTINTWDCTL_OFFS               0x0060U
#define INTINTWDEXP_OFFS               0x0064U
#define INTINTWDMON_OFFS               0x0068U
#define INTMCUMASK_TX_CH0               16U
#define INTMCUMASK_RX_CH0               24U

#define WDEXP_WDEXP_MAX_MASK           (0x0FFFFFFFU) /* 0xFFF_FFFF*16ns = 4.29sec */
#define WDCTL_WDEnable_MASK            (0x00000002U)
#define WDCTL_WDRestart_MASK           (0x00000001U)

#define MAX_DMA_TX_CH                    8U
#define MAX_DMA_RX_CH                    8U
#define MACxRXSTS_CH0                    16U
/*=====================================================================
ENUMERATION
======================================================================*/
typedef struct FW_Version_s
{
  unsigned char rel_dbg;  // "R for Release, D for debug"
  unsigned char major;
  unsigned char minor;
  unsigned char sub_minor;
}FW_Version_t;

#endif /* _TC956X_H__ */
