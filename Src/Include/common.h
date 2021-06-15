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

#ifndef _COMMON_H__
#define _COMMON_H__

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : common.h
 *
 *********************************************************************************************************
 */

/*********************************************************************
*                      INCLUDE FILES
********************************************************************* */
#include "tc956x.h"  /* Register address, Interrupt numbers,
                          Core peripheral section */
#include "tc956x_intc_driver.h"

/*=====================================================================
                    MACRO DEFINITION
==================================================================== */
#define DEF_DISABLED                      0
#define DEF_ENABLED                       1
#define NULL                              0

//#define DMA_OFFLOAD_ENABLE

#define TC956X_COMMON_ONE                 0x00000001U
#define TC956X_COMMON_TWO                 0x00000002U
#define TC956X_COMMON_FOUR                0x00000004U
#define TC956X_THOUSAND                   1000U

typedef unsigned long long uint64_t ;
typedef unsigned int uint32_t ;
typedef int int32_t ;
typedef unsigned char uint8_t ;
typedef char int08_t ;

extern volatile uint64_t guiM3Ticks ;        /* System ticks */

/*=====================================================================
FUNCTION PROTOTYPES
==================================================================== */
void SysTick_Handler( void ) ;

void Hw_Reg_Write32 ( const uint32_t uiAddr_base, const uint32_t uiOffset,
                      const uint32_t uiVal ) ;
uint32_t Hw_Reg_Read32 ( const uint32_t uiAddr_base,
                         const uint32_t uiOffset ) ;

extern void WDT_IRQHandler ( void ) ;

extern void CPU_IntEn( void ) ;

#endif /* _COMMON_H__ */
