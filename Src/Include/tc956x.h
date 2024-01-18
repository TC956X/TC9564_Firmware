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
 *  22 Jul 2021  : Dynamic CM3 TAMAP configuration.
 *  VERSION      : 1.0.2
 *  23 Sep 2021  : Handling RBU Interrupt at Host Driver and maintaining ethtool statistics.
 *  VERSION      : 1.0.4
 *  19 Oct 2021  : Updated value of Debug Counters size as per No.of SRAM Debug counters (each counter of 4 Bytes Size).
 *  VERSION      : 1.0.5
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
//#define ENABLE_MAC2MAC_BRIDGE
//#define MAC2MAC_DEBUG_INFO

/* Debugging count SRAM area start address */
#define TC956X_M3_DBG_CNT_START            0x2000F800U      /* DMEM: 0x2000_0000 - 0x2001_0000 */
/* Debugging count SRAM area size */
#define TC956X_M3_DBG_CNT_SIZE             ( 20U*4U )

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
#define XGMAC_DMA_CUR_TxDESC_LADDR(x)   (0x00003144U + (0x80U * (x)))
#define XGMAC_DMA_CUR_RxDESC_LADDR(x)   (0x0000314CU + (0x80U * (x)))

#define XGMAC_DMA_CH_Status(x)                        (0x00003160U + (0x80U * (x)))
#define XGMAC_DMA_Int_Enable(x)                       (0x00003138U + (0x80U * (x)))
#define XGMAC_DMA_STS_TX_TI                           (((uint32_t)0x1U) << 0U)
#define XGMAC_DMA_STS_TX_TPS                          (((uint32_t)0x1U) << 1U)
#define XGMAC_DMA_STS_TX_TBU                          (((uint32_t)0x1U) << 2U)
#define XGMAC_DMA_STS_RX_RI                           (((uint32_t)0x1U) << 6U)
#define XGMAC_DMA_STS_RX_RBU                          (((uint32_t)0x1U) << 7U)
#define XGMAC_DMA_STS_RX_RPS                          (((uint32_t)0x1U) << 8U)
#define XGMAC_DMA_STS_TX_TEB                          (((uint32_t)0x7U) << 16U)
#define XGMAC_DMA_STS_RX_REB                          (((uint32_t)0x7U) << 19U)
#define XGMAC_DMA_TX_INT_STS_ALL                      ((XGMAC_DMA_STS_TX_TI) | (XGMAC_DMA_STS_TX_TPS) | \
                                                        (XGMAC_DMA_STS_TX_TBU) | XGMAC_DMA_STS_TX_TEB)
#define XGMAC_DMA_RX_INT_STS_ALL                      ((XGMAC_DMA_STS_RX_RI) | /*(XGMAC_DMA_STS_RX_RBU)| */ \
                                                       (XGMAC_DMA_STS_RX_RPS) | (XGMAC_DMA_STS_RX_REB))
#define XGMAC_MTL_TxQ_DEBUG(x)          (0x00001108U + (0x80U * (x)))
#define XGMAC_MTL_TxQ_DEBUG_TXQSTS_MASK	(1U << 4U)
#define XGMAC_MTL_TxQ_DEBUG_TXQSTS_SHIFT	(4U)

#ifdef ENABLE_MAC2MAC_BRIDGE
#define XGMAC_PCI_OFFSET                  0x40020000U
#define ATR_AXI4_SLV0_TABLE0_1            0x804

#define XGMAC_DESC_MASKADDLOW             0xFFFFFFFFU
#define XGMAC_PACKET_FILTER               (0x0008)
#define SET_DDS                           0x80U
#define XGMAC_BROADCAST_ADDLOW            0xFFFFFFFFU
#define XGMAC_BROADCAST_ADDHIGH           0x8000FFFFU
#define XGMAC_MAC_RXQ_CTRL4               (0x0094)
#define UFFQE_SHIFT                       0
#define UFFQ_SHIFT                        1
#define UFFQ_MASK                         0xFF
#define MFFQE_SHIFT                       8
#define MFFQ_SHIFT                        9
#define MFFQ_MASK                         0xFF00
#define XGMAC_MAC_RXQ_CTRL0               (0x00A0)
#define RXQEN_MASK                        0x3
#define MTL_QUEUE_DCB                     0x2
#define XGMAC_MAC_RXQ_CTRL1               (0x00A4)
#define UPQ_MASK                          0xF
#define MCBCQ_SHIFT                       8
#define MCBCQ_MASK                        0xF
#define MCBCQEN_SHIFT                     15
#define XGMAC_EXTENDED_REG                (0x0140)
#define XGMAC_ADDRx_HIGH(x)               (0x0300 + (x) * 0x8)
#define XGMAC_ADDRx_LOW(x)                (0x0304 + (x) * 0x8)
#define XGMAC_INDIR_ACCESS_CTRL           (0x0700)
#define XGMAC_INDIR_ACCESS_DATA           (0x0704)
#define XGMAC_Timestamp_Control           (0x0D00)

#define XGMAC_MTL_RXQ_DMA_MAP0            (0x1030)
#define XGMAC_MTL_RXQ_DMA_MAP1            (0x1034)
#define DDMACH_SHIFT                      7
#define XGMAC_MTL_Q_Interrupt_Status(x)   (0x1174 + (0x80 * (x)))

#define XGMAC_DMA_CH_CONTROL(x)           (0x3100 + (0x80 * (x)))
#define XGMAC_DMA_CH_TX_Control(x)        (0x3104 + (0x80 * (x)))
#define XGMAC_TxPBL_MASK                  0x1F0000
#define XGMAC_TxPBL_SHIFT                 16
#define XGMAC_OSP                         0x10
#define XGMAC_DMA_CH_RX_Control(x)        (0x3108 + (0x80 * (x)))
#define XGMAC_RX_PBL_MASK                 0x1F0000
#define XGMAC_RxPBL_SHIFT                 16
#define XGMAC_RBSZ_MASK                   0x7FFE
#define XGMAC_RBSZ_SHIFT                  1
#define XGMAC_DMA_CH_TxDESC_HADDR(x)      (0x3110 + (0x80 * (x)))
#define XGMAC_DMA_CH_TxDESC_LADDR(x)      (0x3114 + (0x80 * (x)))
#define XGMAC_DMA_CH_RxDESC_HADDR(x)      (0x3118 + (0x80 * (x)))
#define XGMAC_DMA_CH_RxDESC_LADDR(x)      (0x311c + (0x80 * (x)))
#define XGMAC_DMA_CH_TxDESC_TAIL_LPTR(x)  (0x3124 + (0x80 * (x)))
#define XGMAC_DMA_CH_RxDESC_TAIL_LPTR(x)  (0x312c + (0x80 * (x)))
#define XGMAC_DMA_CH_TX_CONTROL2(x)       (0x3130 + (0x80 * (x)))
#define XGMAC_TDRL_MASK                   0x1FFF
#define XGMAC_DMA_CH_RX_CONTROL2(x)       (0x3134 + (0x80 * (x)))
#define XGMAC_OWRQ_MASK                   0x3000000
#define XGMAC_OWRQ_SHIFT                  24
#define XGMAC_RDRL_MASK                   0x1FFF
#define XGMAC_DMA_CH_INT_EN(x)            (0x3138 + (0x80 * (x)))
/* (XGMAC_NIE | XGMAC_AIE | XGMAC_RBUE | XGMAC_RIE | XGMAC_TIE) */
#define XGMAC_DMA_INT_DEFAULT_EN          0xC0C1

// DMA_CH_Status
// [21:19] : REB, Rx DMA Error Bits (Bus Error)
// [18:16] : TEB, Tx DMA Error Bits (Bus Error)
//    [15] : NIS, Normal Interrupt Summary   ( RI | TBU | TI )
//    [14] : AIS, Abnormal Interrupt Summary (CDE | FBE | DDE | RPS | RBU | TPS)
//    [13] : CDE, Context Descriptor Error
//    [12] : FBE, Fatal Bus Error
//     [9] : DDE, Descriptor Definition Error
//     [8] : RPS, Receive Process Stopped
//     [7] : RBU, Receive Buffer Unavailable
//     [6] : RI,  Receive Interrupt
//     [2] : TBU, Transmit Buffer Unavailable
//     [1] : TPS, Transmit Process Stopped
//     [0] : TI,  Transmit Interrupt
#define EMAC_DMA_CH_STS_REB       (0x7 << 19)
#define EMAC_DMA_CH_STS_TEB       (0x7 << 16)
#define EMAC_DMA_CH_STS_NIS       (0x1 << 15)
#define EMAC_DMA_CH_STS_AIS       (0x1 << 14)
#define EMAC_DMA_CH_STS_CDE       (0x1 << 13)
#define EMAC_DMA_CH_STS_FBE       (0x1 << 12)
#define EMAC_DMA_CH_STS_DDE       (0x1 <<  9)
#define EMAC_DMA_CH_STS_RPS       (0x1 <<  8)
#define EMAC_DMA_CH_STS_RBU       (0x1 <<  7)
#define EMAC_DMA_CH_STS_RI        (0x1 <<  6)
#define EMAC_DMA_CH_STS_TBU       (0x1 <<  2)
#define EMAC_DMA_CH_STS_TPS       (0x1 <<  1)
#define EMAC_DMA_CH_STS_TI        (0x1 <<  0)

#define EMAC_DMA_CH_STS_RXERR  (EMAC_DMA_CH_STS_REB | \
                                 EMAC_DMA_CH_STS_RPS | EMAC_DMA_CH_STS_RBU)
#define EMAC_DMA_CH_STS_TXERR  (EMAC_DMA_CH_STS_TEB | \
                 EMAC_DMA_CH_STS_TPS | EMAC_DMA_CH_STS_TBU)
#define EMAC_DMA_CH_STS_ERR    (EMAC_DMA_CH_STS_RXERR | EMAC_DMA_CH_STS_TXERR | \
                                 EMAC_DMA_CH_STS_AIS | EMAC_DMA_CH_STS_DDE | \
                                 EMAC_DMA_CH_STS_CDE | EMAC_DMA_CH_STS_FBE)
#define EMAC_DMA_CH_STS_RX_ALL   (EMAC_DMA_CH_STS_RXERR | EMAC_DMA_CH_STS_RI)
#define EMAC_DMA_CH_STS_TX_ALL  (EMAC_DMA_CH_STS_TXERR | EMAC_DMA_CH_STS_TI)
#define DMA_CH_Current_App_TxDesc_L(x)        (0x3144 + (0x80 * (x)))
#define XGMAC_DMA_CH_STATUS(x)                (0x3160 + (0x80 * (x)))
#define XGMAC_RX_WR_RINGOFF(x)                (0x317C + (0x80 * (x)))
#define XGMAC_TX_WR_RINGOFF(x)                (0x3178 + (0x80 * (x)))
#define DESC_PKT_LEN_MASK                      0x3FFF
#define MASK_32BIT                             0xFF
#endif

/* M3 registers */
/* SysTick Ctrl & Status Reg.     */
#define CPU_REG_NVIC_ST_CTRL           (*((uint32_t *)(0xE000E010U)))
/* SysTick Reload      Value Reg. */
#define CPU_REG_NVIC_ST_RELOAD         (*((uint32_t *)(0xE000E014U)))
/* System Handlers 12 to 15 Prio. */
#define CPU_REG_NVIC_SHPRI3            (*((uint32_t *)(0xE000ED20U)))
#define TC956X_PCIE_GLUE_BASE                 0x4002C000U
#define TC956X_PCIE_GLUE_EP_REG_ACCESS_CTRL   0x00000034U

#define TC956X_MSIGEN_REG_BASE           0x4000F000U
#define SW_MSI_SET_PF0                   0x0050U
#define SW_MSI_SET_PF1                   0x0150U

/* Interrupt registers */
#define TC956X_INTC_REG_BASE           0x40008000U
#define MAC0STATUS                     0x000CU
#define MAC1STATUS                     0x0010U
#define PCIEL12FLG                     0x0018U
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
#define MCUFLG_OFFS                    0x0054U
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

#define TC956X_EMAC_RST					(0x00000080U)

#define RSCMNG_RSC_BASE                  0x40020000U
#define RSCMNG_RSC_CTRL_REG_OFFSET       0x00002004U
#define RSCMNG_RSC_ST_REG                0x00002008U
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
