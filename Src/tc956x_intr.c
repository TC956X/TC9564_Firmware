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
* =========================================================================== */

/*! History:
*  8 Jul 2020   : Initial Version
*  VERSION      : 1.0.0
*/

/*******************************************************************************************************
*
* Target        : TC956X
* Filename      : tc956x_MSI_Int.c
*
********************************************************************************************************/

/*=====================================================================
INCLUDE FILES
=====================================================================*/
#include "common.h"

/*=====================================================================
FUNCTION DEFINITION
======================================================================*/
/*
*  Function    :   WDT_IRQHandler(void)
*  Purpose     :   Interrupt handler for handling WatchDog Timer interrupts
*  Inputs      :   None
*  Outputs     :   None
*  Return Value:   None
*  Limitations :   None
*/
void WDT_IRQHandler (void)
{
  uint32_t uiData ;

  *( uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_ELEVEN * TC956X_FOUR ) ) += TC956X_ONE ;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS ) ;
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS,
                ( uiData | ( WDCTL_WDRestart_MASK ) ) ) ;
  
  /* Read the Watchdog-timer counter monitor */
  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTINTWDMON_OFFS ) ;
  *( uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_TWELVE * TC956X_FOUR ) ) = uiData ;
  CM3_Errata_IRQ();
}
/* end WDT_IRQHandler */

#ifdef DMA_OFFLOAD_ENABLE
/*
*  Function    :   EMAC0_TXDMA_IRQHandler(void)
*  Purpose     :   Interrupt handler for handling IPA Tx interrupts for MAC0
*  Inputs      :   None
*  Outputs     :   None
*  Return Value:   None
*  Limitations :   None
*/
void EMAC0_TXDMA_IRQHandler (void)
{
  uint32_t uiData, uiIntMask;
  uint32_t i;
  uint32_t uiCurDesc;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, MAC0STATUS );
  
  for( i = 0; i < MAX_DMA_TX_CH; i++ )
  {
    if( uiData & ( TC956X_ONE << i) )
    { 
      /* Disable Interrupt generation for the channel */
      uiIntMask = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS );
      uiIntMask  |= ( 1 << ( INTMCUMASK_TX_CH0 + i ) );
      Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS, uiIntMask ) ;
      
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET0, XGMAC_DMA_CUR_TxDESC_LADDR(i) );

      *(uint32_t*)(*( uint32_t*)( SRAM_TX_PCIE_ADDR_LOC + ( i * TC956X_FOUR ) ) ) =  uiCurDesc;
    }
  }
  
  CM3_Errata_IRQ();
}

void EMAC0_RXDMA_IRQHandler (void)
{
  uint32_t uiData, uiIntMask;
  uint32_t i;
  uint32_t uiCurDesc;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, MAC0STATUS ) ;
  
  for(i = 0; i < MAX_DMA_RX_CH; i++)
  {
    if( uiData & ( TC956X_ONE << ( i + MACxRXSTS_CH0 ) ) )
    { 
      /* Disable Interrupt generation for the channel */
      uiIntMask = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS );
      uiIntMask  |= ( 1 << ( INTMCUMASK_RX_CH0 + i ) );
      Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS, uiIntMask );
      
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET0, XGMAC_DMA_CUR_RxDESC_LADDR(i) );

      *(uint32_t*)(*( uint32_t*)( SRAM_RX_PCIE_ADDR_LOC + ( i * TC956X_FOUR ) ) ) = uiCurDesc;
    }
  }
  
  CM3_Errata_IRQ();
}

void EMAC1_TXDMA_IRQHandler (void)
{
  uint32_t uiData, uiIntMask;
  uint32_t i;
  uint32_t uiCurDesc;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, MAC1STATUS ) ;
  
  for( i = 0; i < MAX_DMA_TX_CH; i++ )
  {
    if( uiData & ( TC956X_ONE << i ) )
    {
      /* Disable Interrupt generation for the channel */
      uiIntMask = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS );
      uiIntMask  |= ( 1 << ( INTMCUMASK_TX_CH0 + i ) );
      Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS, uiIntMask );
      
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET1, XGMAC_DMA_CUR_TxDESC_LADDR(i) );

      *(uint32_t*)(*( uint32_t*)( SRAM_TX_PCIE_ADDR_LOC + ( MAX_DMA_TX_CH * TC956X_FOUR ) + ( i * TC956X_FOUR ) ) ) =  
                                  uiCurDesc;
    }
  }
  
  CM3_Errata_IRQ();
}

void EMAC1_RXDMA_IRQHandler (void)
{
  uint32_t uiData, uiIntMask;
  uint32_t i;
  uint32_t uiCurDesc;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, MAC1STATUS ) ;
  
  for( i = 0; i < MAX_DMA_RX_CH; i++ )
  {
    if( uiData & ( TC956X_ONE << ( i + MACxRXSTS_CH0 ) ) )
    { 
      /* Disable Interrupt generation for the channel */
      uiIntMask = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS );
      uiIntMask  |= ( 1 << ( INTMCUMASK_RX_CH0 + i ) );
      Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS, uiIntMask );
      
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET1, XGMAC_DMA_CUR_RxDESC_LADDR(i) );

      *(uint32_t*)(*( uint32_t*)( SRAM_RX_PCIE_ADDR_LOC + ( MAX_DMA_RX_CH * TC956X_FOUR ) + ( i * TC956X_FOUR ) ) ) =  
                                  uiCurDesc;
    }
  }
  
  CM3_Errata_IRQ();
}

#endif
