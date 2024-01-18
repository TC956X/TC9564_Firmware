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
 * Filename      : tc956x_intc_driver.h
 *
 *********************************************************************************************************
 */

#ifndef TC956X_INTC_DRIVER_H_
#define TC956X_INTC_DRIVER_H_

#include "Driver_Common.h"
#include "tc956x_regacc.h"

#define INTMCUMASK2       0x8028U
#define INTEXTMASK2       0x8038U
#define INTINTXMASK2      0x8048U

/* ===================================
 * Inline Function Declaration
 * =================================== */

/**
 * @brief  Inline function for CM3 IRQs fix as per Errata
 * @param[in]  void
 * @return void
 */
__inline void CM3_Errata_IRQ(void)
{
  __schedule_barrier();
#if defined (__CC_ARM)
  __asm{DSB};
#endif
  __schedule_barrier();
}

/** Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Specific Interrupt Numbers ***********************************************************************/
    INT_SRC_NMI                 = -14,  /* Non Maskable                */
    INT_SRC_Hard_Fault          = -13,  /* Cortex-M3 Hard Fault        */
    INT_SRC_Memory_Management   = -12,  /* Cortex-M3 Memory Management */
    INT_SRC_Bus_Fault           = -11,  /* Cortex-M3 Bus Fault         */
    INT_SRC_Usage_Fault         = -10,  /* Cortex-M3 Usage Fault       */
    INT_SRC_SVC                 = -5,   /* Cortex-M3 SV Call           */
    INT_SRC_Debug_Monitor       = -4,   /* Cortex-M3 Debug Monitor     */
    INT_SRC_Pend_SV             = -2,   /* Cortex-M3 Pend SV           */
    SysTick_IRQn                = -1,   /* Cortex-M3 System Tick       */

/* **** TC956X Specific Interrupt Numbers *************************/
    INT_SRC_NBR_INTI01            = 0U,   /* Interrupt Pin 0. */
    INT_SRC_NBR_INTI02            = 1U,   /* Interrupt Pin 1. */
    INT_SRC_NBR_INTI03            = 2U,   /* Interrupt Pin 2. */
    INT_SRC_NBR_EXT_INTI          = 3U,   /* External Input Interrupt Pin */
    INT_SRC_NBR_I2C_SLAVE         = 4U,   /* I2C slave interrupt */
    INT_SRC_NBR_I2C_MASTER        = 5U,   /* I2C mater interrupt */
    INT_SRC_NBR_SPI_SLAVE         = 6U,   /* SPI slave interrupt */
    INT_SRC_NBR_MAC0_LPI          = 8U,   /* eMAC0 LPI exit */
    INT_SRC_NBR_MAC0_POWER        = 9U,   /* eMAC0 power management */
    INT_SRC_NBR_MAC0_EVENTS       = 10U,  /* eMAC0 event from LPI, GRMII, Management counter */
    INT_SRC_NBR_MAC0_TXDMA        = 11U,  /* eMAC0 Tx DMA Chnl  */

    INT_SRC_NBR_MAC0_RXDMA        = 14U,  /* eMAC0 Rx DMA Chnl */

    INT_SRC_NBR_MAC0_EXT          = 16U,  /* eMAC0 External interrupt */
    INT_SRC_NBR_MAC0_PPO          = 17U,  /* eMAC0 PPO */
    INT_SRC_NBR_MAC1_LPI          = 18U,  /* eMAC1 LPI exit */
    INT_SRC_NBR_MAC1_POWER        = 19U,  /* eMAC1 power management */
    INT_SRC_NBR_MAC1_EVENTS       = 20U,  /* eMAC1 event from LPI, GRMII, Management counter */
    INT_SRC_NBR_MAC1_TXDMA        = 21U,  /* eMAC1 Tx DMA Chnl  */

    INT_SRC_NBR_MAC1_RXDMA        = 24U,  /* eMAC1 Rx DMA Chnl */

    INT_SRC_NBR_MAC1_EXT          = 26U,  /* eMAC1 External interrupt */
    INT_SRC_NBR_MAC1_PPO          = 27U,  /* eMAC1 PPO */
    INT_SRC_NBR_MAC0_XPCS         = 28U,  /* eMAC0 XPCS */
    INT_SRC_NBR_MAC1_XPCS         = 29U,  /* eMAC1 XPCS */

    INT_SRC_NBR_UART0             = 38U,  /* UART0. */

    INT_SRC_NBR_MSIGEN            = 40U,  /* MSIGEN. */
    INT_SRC_NBR_MAILBOX           = 41U,  /* MAILBOX */
    INT_SRC_NBR_PCIEEP            = 42U,  /* PCIe EP Local Interrupt. */
    INT_SRC_NBR_PCIEFLR           = 43U,  /* PCIe FLR */
    INT_SRC_NBR_PCIEIDLE_ST       = 44U,  /* PCIe IDLE ST (L1p1, L1p2, IDLE) */
    INT_SRC_NBR_MCU_FLAG          = 45U,  /* MCU FLAG Int */

    INT_SRC_NBR_WDT               = 48U,  /*  INTC WatchDog timer */
} IRQn_Type;


/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __CM3_REV               0x0200  /* Cortex-M3 Core Revision */
#define __MPU_PRESENT           0       /* MPU present or not      */
#define __NVIC_PRIO_BITS        3       /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig  0       /* Set to 1 if different SysTick Config is used */

#include <core_cm3.h>                   /* Cortex-M3 processor and core peripherals */

#endif
