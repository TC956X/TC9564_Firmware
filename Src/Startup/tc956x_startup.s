;/* ============================================================================
; * The MIT License (MIT)
; *
; * Copyright (c) 2020 Toshiba Electronic Devices & Storage Corporation
; * Permission is hereby granted, free of charge, to any person obtaining a copy
; * of this software and associated documentation files (the "Software"), to deal
; * in the Software without restriction, including without limitation the rights
; * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; * copies of the Software, and to permit persons to whom the Software is
; * furnished to do so, subject to the following conditions:
; *
; * The above copyright notice and this permission notice shall be included in
; * all copies or substantial portions of the Software.
; *
; * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; * THE SOFTWARE.
; * ========================================================================= */
;
;/*! History:
; *  8 Jul 2020   : Initial Version
; *  VERSION      : 1.0.0
; */
;
;/*
; *********************************************************************************************************
; *
; * Target      : TC956X
; * Filename    : tc956x_startup.s
; *
; *********************************************************************************************************
; */
;
;/*
;*********************************************************************************************************
;*                                              STACK DEFINITIONS
;*********************************************************************************************************
;*/

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


;/*
;*********************************************************************************************************
;*                                              STACK DEFINITIONS
;*********************************************************************************************************
;*/

Heap_Size        EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB
;/*
;*********************************************************************************************************
;*                           Vector Table
;*********************************************************************************************************
;*/
              AREA    RESET, DATA, READONLY
              EXPORT  __Vectors
              IMPORT  SysTick_Handler

__Vectors   DCD    __initial_sp                   ; 0:  Stack Pointer
      DCD       Reset_IRQHandler                  ; 1:  Reset
      DCD       NMI_IRQHandler                    ; 2:  NMI
      DCD       HardFault_IRQHandler              ; 3:  Hard Fault
      DCD       MemManage_IRQHandler              ; 4:  MPU Fault
      DCD       BusFault_IRQHandler               ; 5:  Bus Fault
      DCD       UsageFault_IRQHandler             ; 6:  Usage Fault
      DCD       App_Spurious_IRQHandler           ; 7:  App Spurious
      DCD       App_Spurious_IRQHandler           ; 8:  App Spurious
      DCD       App_Spurious_IRQHandler           ; 9:  App Spurious
      DCD       App_Spurious_IRQHandler           ; 10: App Spurious
      DCD       SVC_IRQHandler                    ; 11: SVCall
      DCD       DebugMon_IRQHandler               ; 12: Debug Monitor
      DCD       App_Spurious_IRQHandler           ; 13: App Spurious
      DCD       PendSV_IRQHandler                 ; 14: PendSV
      DCD       SysTick_Handler                   ; 15: SysTick
      ; Vectors for the device specific external interrupts handler
      DCD       INTI01_IRQHandler                 ; 16: External interrupt input (INTI01)
      DCD       INTI02_IRQHandler                 ; 17: External interrupt input (INTI02)
      DCD       INTI03_IRQHandler                 ; 18: External interrupt input (INTI03)
      DCD       EXT_INTI_IRQHandler               ; 19: External interrupt input (INTi)
      DCD       I2CSlave_IRQHandler               ; 20: I2C slave interrupt
      DCD       I2CMaster_IRQHandler              ; 21: I2C master interrupt
      DCD       SPISlave_IRQHandler               ; 22: SPI slave interrupt
      DCD       0                                 ; 23: Reserved
      DCD       EMAC0_LPIEXIT_IRQHandler          ; 24: MAC eMAC0 LPI exit interrupt
      DCD       EMAC0_PM_IRQHandler               ; 25: MAC eMAC0 Power management interrupt
      DCD       EMAC0_EVENT_IRQHandler            ; 26: MAC eMAC0 event interrupt
      DCD       EMAC0_TXDMA_IRQHandler            ; 27: MAC interrupt from eMAC0 Tx DMA channel
      DCD       0                                 ; 28: Reserved
      DCD       0                                 ; 29: Reserved
      DCD       EMAC0_RXDMA_IRQHandler            ; 30: MAC interrupt from eMAC0 Rx DMA channel
      DCD       0                                 ; 31: Reserved
      DCD       EMAC0_EXT_IRQHandler              ; 32: MAC interrupt from eMAC0 external interrupt
      DCD       EMAC0_PPO_IRQHandler              ; 33: MAC eMAC0 PPO
      DCD       EMAC1_LPIEXIT_IRQHandler          ; 34: MAC eMAC1 LPI exit interrupt
      DCD       EMAC1_PM_IRQHandler               ; 35: MAC eMAC1 Power management interrupt
      DCD       EMAC1_EVENT_IRQHandler            ; 36: MAC eMAC1 event interrupt
      DCD       EMAC1_TXDMA_IRQHandler            ; 37: MAC interrupt from eMAC1 Tx DMA channel
      DCD       0                                 ; 38: Reserved
      DCD       0                                 ; 39: Reserved
      DCD       EMAC1_RXDMA_IRQHandler            ; 40: MAC interrupt from eMAC1 Rx DMA channel
      DCD       0                                 ; 41: Reserved
      DCD       EMAC1_EXT_IRQHandler              ; 42: MAC interrupt from eMAC1 external interrupt.
      DCD       EMAC1_PPO_IRQHandler              ; 43: MAC eMAC1 PPO
      DCD       EMAC0_XPCS_IRQHandler             ; 44: MAC eMAC0 XPCS
      DCD       EMAC1_XPCS_IRQHandler             ; 45: MAC eMAC1 XPCS
      DCD       0                                 ; 46: Reserved
      DCD       0                                 ; 47: Reserved
      DCD       0                                 ; 48: Reserved
      DCD       0                                 ; 49: Reserved
      DCD       0                                 ; 50: Reserved
      DCD       0                                 ; 51: Reserved
      DCD       0                                 ; 52: Reserved
      DCD       0                                 ; 53: Reserved
      DCD       UART0_IRQHandler                  ; 54: UART ch0 interrupt
      DCD       0                                 ; 55: Reserved
      DCD       MSIGEN_IRQHandler                 ; 56: MSIGEN interrupt
      DCD       MAILBOX_IRQHandler                ; 57: Mail Box Inerrupt Status
      DCD       PCIeEP_IRQHandler                 ; 58: PCIe EP Local Interrupt Status
      DCD       PCIeFLR_IRQHandler                ; 59: PCIe FLR (all functional reset)
      DCD       PCIeIDLE_ST_IRQHandler            ; 60: PCIe IDLE ST (L1p1, L1p2, IDLE)
      DCD       MCUFLG_IRQHandler                 ; 61: MCU Flag interrupt
      DCD       0                                 ; 62: Reserved
      DCD       0                                 ; 63: Reserved
      DCD       WDT_IRQHandler                    ; 64: Watchdog-timer interrupt
      DCD       0                                 ; 65: Reserved
      DCD       0                                 ; 66: Reserved
      DCD       0                                 ; 67: Reserved
      DCD       0                                 ; 68: Reserved
      DCD       0                                 ; 69: Reserved
__Vectors_End
          AREA    |.text|, CODE, READONLY


Reset_IRQHandler    PROC
      EXPORT  Reset_IRQHandler
      IMPORT  __main
      LDR     R0, =__main
      BX      R0
      ENDP

NMI_IRQHandler       PROC
      EXPORT  NMI_IRQHandler
      B       .
      ENDP

HardFault_IRQHandler  PROC
      EXPORT  HardFault_IRQHandler
      B       .
      ENDP

MemManage_IRQHandler  PROC
      EXPORT  MemManage_IRQHandler
      B       .
      ENDP

BusFault_IRQHandler  PROC
      EXPORT  BusFault_IRQHandler
      B       .
      ENDP

UsageFault_IRQHandler  PROC
      EXPORT  UsageFault_IRQHandler
      B       .
      ENDP

SVC_IRQHandler       PROC
      EXPORT  SVC_IRQHandler
      B       .
      ENDP

DebugMon_IRQHandler  PROC
      EXPORT  DebugMon_IRQHandler
      B       .
      ENDP

PendSV_IRQHandler    PROC
      EXPORT  PendSV_IRQHandler
      B       .
      ENDP

App_Spurious_IRQHandler  PROC
      EXPORT  App_Spurious_IRQHandler
      B       .
      ENDP

App_Reserved_IRQHandler  PROC
      EXPORT  App_Reserved_IRQHandler
      B       .
      ENDP

CPU_IntDis    PROC
      EXPORT CPU_IntDis
      CPSID   I
      BX      LR
      ENDP

CPU_IntEn    PROC
      EXPORT CPU_IntEn
      CPSIE   I
      BX      LR
      ENDP

Default_Handler PROC
      EXPORT  INTI01_IRQHandler           [WEAK]
      EXPORT  INTI02_IRQHandler           [WEAK]
      EXPORT  INTI03_IRQHandler           [WEAK]
      EXPORT  EXT_INTI_IRQHandler         [WEAK]
      EXPORT  I2CSlave_IRQHandler         [WEAK]
      EXPORT  I2CMaster_IRQHandler        [WEAK]
      EXPORT  SPISlave_IRQHandler         [WEAK]
      EXPORT  EMAC0_LPIEXIT_IRQHandler    [WEAK]
      EXPORT  EMAC0_PM_IRQHandler         [WEAK]
      EXPORT  EMAC0_EVENT_IRQHandler      [WEAK]
      EXPORT  EMAC0_TXDMA_IRQHandler      [WEAK]
      EXPORT  EMAC0_RXDMA_IRQHandler      [WEAK]
      EXPORT  EMAC0_EXT_IRQHandler        [WEAK]
      EXPORT  EMAC0_PPO_IRQHandler        [WEAK]
      EXPORT  EMAC1_LPIEXIT_IRQHandler    [WEAK]
      EXPORT  EMAC1_PM_IRQHandler         [WEAK]
      EXPORT  EMAC1_EVENT_IRQHandler      [WEAK]
      EXPORT  EMAC1_TXDMA_IRQHandler      [WEAK]
      EXPORT  EMAC1_RXDMA_IRQHandler      [WEAK]
      EXPORT  EMAC1_EXT_IRQHandler        [WEAK]
      EXPORT  EMAC1_PPO_IRQHandler        [WEAK]
      EXPORT  EMAC0_XPCS_IRQHandler       [WEAK]
      EXPORT  EMAC1_XPCS_IRQHandler       [WEAK]
      EXPORT  UART0_IRQHandler            [WEAK]
      EXPORT  MSIGEN_IRQHandler           [WEAK]
      EXPORT  MAILBOX_IRQHandler          [WEAK]
      EXPORT  PCIeEP_IRQHandler           [WEAK]
      EXPORT  PCIeFLR_IRQHandler          [WEAK]
      EXPORT  PCIeIDLE_ST_IRQHandler      [WEAK]
      EXPORT  MCUFLG_IRQHandler           [WEAK]
      EXPORT  WDT_IRQHandler              [WEAK]
; Default exception/interrupt handler
INTI01_IRQHandler
INTI02_IRQHandler
INTI03_IRQHandler
EXT_INTI_IRQHandler
I2CSlave_IRQHandler
I2CMaster_IRQHandler
SPISlave_IRQHandler
EMAC0_LPIEXIT_IRQHandler
EMAC0_PM_IRQHandler
EMAC0_EVENT_IRQHandler
EMAC0_TXDMA_IRQHandler
EMAC0_RXDMA_IRQHandler
EMAC0_EXT_IRQHandler
EMAC0_PPO_IRQHandler
EMAC1_LPIEXIT_IRQHandler
EMAC1_PM_IRQHandler
EMAC1_EVENT_IRQHandler
EMAC1_TXDMA_IRQHandler
EMAC1_RXDMA_IRQHandler
EMAC1_EXT_IRQHandler
EMAC1_PPO_IRQHandler
EMAC0_XPCS_IRQHandler
EMAC1_XPCS_IRQHandler
UART0_IRQHandler
MSIGEN_IRQHandler
MAILBOX_IRQHandler
PCIeEP_IRQHandler
PCIeFLR_IRQHandler
PCIeIDLE_ST_IRQHandler
MCUFLG_IRQHandler
WDT_IRQHandler
    B       .
    ENDP

    ALIGN

; User Initial Stack & Heap
    IF      :DEF:__MICROLIB

    EXPORT  __initial_sp
    EXPORT  __heap_base
    EXPORT  __heap_limit

    ELSE

    IMPORT  __use_two_region_memory
    EXPORT  __user_initial_stackheap
__user_initial_stackheap

    LDR     R0, =  Heap_Mem
    LDR     R1, =(Stack_Mem + Stack_Size)
    LDR     R2, = (Heap_Mem +  Heap_Size)
    LDR     R3, = Stack_Mem
    BX      LR

    ALIGN

    ENDIF


    END
