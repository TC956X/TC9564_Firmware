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
*  22 Oct 2020  : EEPROM support
*  23 Feb 2021  : Disable Unused clocks,
*                 Exit Firmware on driver unload
*  05 Jul 2021  : Used Systick handler instead of Driver kernel timer to process transmitted Tx descriptors.
*  22 Jul 2021  : For IPA channel interrupts, clear only interrupt status
*  23 Jul 2021  : Enable DMA IPA offload by default
*  VERSION      : 1.0.3
*  23 Sep 2021  : Version Update
*  VERSION      : 1.0.4
*/

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : main.c
 *
 *********************************************************************************************************
 */

/*=====================================================================
INCLUDE FILES
===================================================================== */
#include "common.h"
#include "tc956x_uart_driver.h"
#include "tc956x_gpio_driver.h"
#include "tc956x_i2c_driver.h"

/*======================================================================
GLOBAL VARIABLE
======================================================================*/
volatile uint64_t guiM3Ticks = TC956X_VAL_ZERO ;    /* System ticks */

/*======================================================================
MACRO DEFINITION
======================================================================*/
#define INT_MCUMASK_MASK_ALL        0xFFFFFFFFU
#define INT_MCUMASK2_MASK           0xFFFF7FFFU
#define CPU_FREQ                    250000000U
#define SYS_TICK_PRIO               0x00E00000U
#define TC956X_GPIO3_OUTPUT         DEF_DISABLED
#define TC956X_SW_MSI               DEF_ENABLED
#define TC956X_UART                 DEF_ENABLED
#define TEST_EEPROM_WRITE_ENABLE    DEF_DISABLED
#define USE_EEPROM                  DEF_DISABLED

static void Configure_Watchdog(void);
static void Configure_Systick(void);
static void I2c_Init(void) ;
#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
static void Eeprom_Mac_Write(void);
#endif

#if (DEF_ENABLED == USE_EEPROM)
static void Eeprom_Mac_Read(void);
#endif
static void SysInit(void);

#if ( DEF_ENABLED == TC956X_UART )
static const FW_Version_t version = {'R', 1, 0, 4};
#endif

#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
static uint8_t eeprom_mac_addr[52] = {0x00,0xEC,0x21,0xE5,0x10,0x4F,0xEA,0xEC,0x21,0xE5,0x11,0x4F,0xEA,
									  0x10,0xEC,0x21,0xE5,0x12,0x4F,0xEA,0xEC,0x21,0xE5,0x13,0x4F,0xEA,
									  0x20,0xEC,0x21,0xE5,0x14,0x4F,0xEA,0xEC,0x21,0xE5,0x15,0x4F,0xEA,
									  0x30,0xEC,0x21,0xE5,0x16,0x4F,0xEA,0xEC,0x21,0xE5,0x17,0x4F,0xEA};
#endif

#if (DEF_ENABLED == USE_EEPROM)
static uint8_t default_eeprom_mac_addr[48] = {0xEC,0x21,0xE5,0x10,0x4F,0xEA,0xEC,0x21,0xE5,0x11,0x4F,0xEA,
											  0xEC,0x21,0xE5,0x12,0x4F,0xEA,0xEC,0x21,0xE5,0x13,0x4F,0xEA,
											  0xEC,0x21,0xE5,0x14,0x4F,0xEA,0xEC,0x21,0xE5,0x15,0x4F,0xEA,
											  0xEC,0x21,0xE5,0x16,0x4F,0xEA,0xEC,0x21,0xE5,0x17,0x4F,0xEA};

static uint8_t eeprom_read_mac_addr[65];
#endif
/*=====================================================================
FUNCTION PROTOTYPES
=======================================================================*/
/*
*  Function    :    Hw_Reg_Write32( uiAddr_Base, uiOffset, uiVal )
*  Purpose     :    write data to an address
*  Inputs      :    uiAddr_Base - base address to write
*  Inputs      :    uiOffset - offset from base address
*  Inputs      :    uiVal - value to write
*  Outputs     :    None
*  Return Value:    None
*  Limitations :    None
*/
void Hw_Reg_Write32 (const uint32_t uiAddr_base, const uint32_t uiOffset,
                      const uint32_t uiVal)
{
  volatile uint32_t *puiAddr = ( volatile uint32_t * )
                               ( uint32_t * )( uiAddr_base + uiOffset ) ;

  if ( NULL != puiAddr )
  {
    *puiAddr = uiVal;
  }
}
/* End of Hw_Reg_Write32() */

/*
*  Function    :    Hw_Reg_Read32( uiAddr_Base, uiOffset )
*  Purpose     :    Read data from an address
*  Inputs      :    uiAddr_Base - base address to read from
*  Inputs      :    uiOffset - offset from base address
*  Outputs     :    None
*  Return Value:    puiAddr - Pointer to read data
*  Limitations :    None
*/
uint32_t Hw_Reg_Read32 (const uint32_t uiAddr_base,
                         const uint32_t uiOffset)
{
  uint32_t            uiVal  = 0 ;

  volatile const uint32_t *puiAddr = ( volatile uint32_t *)
                               ( uint32_t *)( uiAddr_base + uiOffset ) ;

  if ( NULL != puiAddr )
  {
    uiVal = *puiAddr ;
  }
  return ( uiVal ) ;     /* Returns address */
}
/* End of Hw_Reg_Read32() */

/*
*  Function    :    SysTick_Handler( )
*  Purpose     :    System Tick Handler
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
void SysTick_Handler (void)
{
  volatile uint32_t uiVal  = 0 ;
  volatile uint32_t uiTxsts;

  guiM3Ticks++ ;

  *(uint64_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_FIFTEEN * TC956X_FOUR ) ) = guiM3Ticks ;

#if ( DEF_ENABLED == TC956X_GPIO3_OUTPUT )
  if (guiM3Ticks & 1)
  {
    tc956x_gpio.OutputData(GPIO_THREE, GPIO_HIGH);
  }
  else
  {
    tc956x_gpio.OutputData(GPIO_THREE, GPIO_LOW);
  }
#endif
#if ( DEF_ENABLED == TC956X_SW_MSI )

  /* Systick timer is 1ms. Check for every TC956X_SW_MSI_TRIGGER_TIME ms*/
  if ((guiM3Ticks % TC956X_SW_MSI_TRIGGER_TIME) == 0)
  {
    /* Handle Port0 */
    uiVal = *(uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_SEVENTEEN * TC956X_FOUR ) );
    if (uiVal == 1)
    {
      uiTxsts = Hw_Reg_Read32( XGMAC_MAC_OFFSET0, XGMAC_MTL_TxQ_DEBUG(0) );
      uiTxsts = (uiTxsts & XGMAC_MTL_TxQ_DEBUG_TXQSTS_MASK) >> XGMAC_MTL_TxQ_DEBUG_TXQSTS_SHIFT;
      if(uiTxsts == 0)
      {
        Hw_Reg_Write32( TC956X_MSIGEN_REG_BASE, SW_MSI_SET_PF0, TC956X_ONE ) ;
      }
    }

  /* Handle Port1 */
    uiVal = *(uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_EIGHTEEN * TC956X_FOUR ) );
    if (uiVal == 1)
    {
      uiTxsts = Hw_Reg_Read32( XGMAC_MAC_OFFSET1, XGMAC_MTL_TxQ_DEBUG(0) );
      uiTxsts = (uiTxsts & XGMAC_MTL_TxQ_DEBUG_TXQSTS_MASK) >> XGMAC_MTL_TxQ_DEBUG_TXQSTS_SHIFT;
      if (uiTxsts == 0)
      {
        Hw_Reg_Write32( TC956X_MSIGEN_REG_BASE, SW_MSI_SET_PF1, TC956X_ONE ) ;
      }
    }
  }

#endif
  CM3_Errata_IRQ();
}
/* End of SysTick_Handler */

/*
*  Function    :    Configure_Systick( )
*  Purpose     :    Configure systick timer
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
static void Configure_Systick (void)
{
  uint32_t uiCount ;

  /* CPU_FREQ ticks : cpu_freq/1000 for 1ms */
  uiCount = ( CPU_FREQ / TC956X_THOUSAND ) ;
  CPU_REG_NVIC_ST_RELOAD = ( uiCount - TC956X_ONE ) ;

  /* Set SysTick handler */
  CPU_REG_NVIC_SHPRI3 = SYS_TICK_PRIO ;
  /* Enable timer.*/
  CPU_REG_NVIC_ST_CTRL |= ( TC956X_COMMON_FOUR | TC956X_COMMON_ONE ) ;
  /* Enable timer interrupt.*/
  CPU_REG_NVIC_ST_CTRL |= TC956X_COMMON_TWO ;
}
/* End of Configure_Systick */

/*
*  Function    :    Configure_Watchdog( )
*  Purpose     :    Configure watchdog timer
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
static void Configure_Watchdog (void)
{
  uint32_t uiData ;

  /* Enable WatchDog Timer */
  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTINTWDEXP_OFFS ) ;
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTWDEXP_OFFS,
                  ( uiData | ( WDEXP_WDEXP_MAX_MASK ) ) ) ;

  uiData = Hw_Reg_Read32( TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS ) ;
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS, ( uiData |
                  ( WDCTL_WDEnable_MASK ) | ( WDCTL_WDRestart_MASK ) ) ) ;

}
/* End of Configure_Watchdog */

/*
*  Function    :    SysInit( )
*  Purpose     :    System Initialization
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
static void SysInit (void)
{

  /* Enable clocks (eMAC0All, MSIGEN, MACRX/TX, INTC and default clocks  ) */
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, TC956X_CLK_CTRL0_ENABLE ) ;
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NCLKCTRL1_OFFS, TC956X_CLK_CTRL1_ENABLE ) ;

  /* Reset is de-asserted (MCU, INTC, eMAC ...) */
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NRSTCTRL0_OFFS, TC956X_VAL_ZERO ) ;
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NRSTCTRL1_OFFS, TC956X_VAL_ZERO ) ;


#if ( DEF_ENABLED == TC956X_GPIO3_OUTPUT )
  tc956x_gpio.ConfigOutput(GPIO_THREE); /* config GPIO0 for output */
#endif

  /* Mask all interrupts for MCU , Not using any interrupts for CPU */
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS, INT_MCUMASK_MASK_ALL );
  /* Enable WDT interrupt, PCIe EP Local Interrupt and mask all other interrupts */
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK2_OFFS, INT_MCUMASK2_MASK );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTMCUMASK3_OFFS, INT_MCUMASK_MASK_ALL );

  /* Mask all Externel interrupts */
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTEXTMASK0_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTEXTMASK1_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTEXTMASK2_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTEXTMASK3_OFFS, INT_MCUMASK_MASK_ALL );

  /* Mask all INTX interrupts */
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTXMASK0_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTXMASK1_OFFS, INT_MCUMASK_MASK_ALL );
  Hw_Reg_Write32( TC956X_INTC_REG_BASE, INTINTXMASK2_OFFS, INT_MCUMASK_MASK_ALL );

  Configure_Systick();
  Configure_Watchdog();
  CPU_IntEn() ;
}
/* End of SysInit */

/*
*  Function    :    I2c_Init( )
*  Purpose     :    I2C Initialization
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
static void I2c_Init(void)
{
  int32_t ret = 0;
  ARM_DRIVER_I2C * I2Cdrv = &DRIVER_I2C;
  ret = I2Cdrv->Initialize(NULL);

  if (ARM_DRIVER_OK == ret)
  {
#if ( DEF_ENABLED == TC956X_UART )
    TC956X_Ser_Printf(" I2C INIT DONE *********************\r");
#endif
  }
  else
  {
#if ( DEF_ENABLED == TC956X_UART )
    TC956X_Ser_Printf(" I2C INIT FAILED *********************\r");
#endif
    return;
  }
  ret = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);

  if (ARM_DRIVER_OK == ret)
  {
#if ( DEF_ENABLED == TC956X_UART )
    TC956X_Ser_Printf(" I2C config DONE *********************\r");
#endif
  }
  else
  {
#if ( DEF_ENABLED == TC956X_UART )
    TC956X_Ser_Printf(" I2C config FAILED *********************\r");
#endif
  }
  return;
}
/* End of I2c_Init */

#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
/*
*  Function    :    Eeprom_Mac_Write( )
*  Purpose     :    Write static MAC addr to EEPROM
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
static void Eeprom_Mac_Write(void)
{
  int32_t ret = 0;
  uint8_t i, eeprom_mac_count, counter;
  ARM_DRIVER_I2C * I2Cdrv = &DRIVER_I2C;
  uint8_t offset = *(uint8_t *) TC956X_M3_EEPROM_OFFSET_ADDR;
  uint8_t mac_addr_count = *(uint8_t *) TC956X_M3_MAC_COUNT;

  if(mac_addr_count == TC956X_ONE) {
	eeprom_mac_count = TC956X_SIX;
	counter = mac_addr_count;
  } else {
	eeprom_mac_count = TC956X_THIRTEEN;
	counter = (mac_addr_count % TC956X_TWO) ? ((mac_addr_count/TC956X_TWO)+TC956X_ONE):(mac_addr_count/TC956X_TWO);
  }

  for(i = 0; i < counter; i++) {
    eeprom_mac_addr[i * TC956X_THIRTEEN] = offset + i * TC956X_SIXTEEN;
    ret = I2Cdrv->EepromWrite(EEPROM_SLAVE_ADDRESS, TC956X_ZERO, &eeprom_mac_addr[i * TC956X_THIRTEEN],
	                          eeprom_mac_count);
    if (ret != ARM_DRIVER_OK)
    {
#if ( DEF_ENABLED == TC956X_UART )
      TC956X_Ser_Printf("MAC address write on EEPROM failed\n");
#endif
    }
    else
    {
#if ( DEF_ENABLED == TC956X_UART )
      TC956X_Ser_Printf("*** MAC address write on EEPROM successful ***\n");
#endif
    }
  }

  return;
}
/* End of Eeprom_Mac_Write */
#endif

/*
*  Function    :    Eeprom_Mac_Read( )
*  Purpose     :    Read MAC addresses from EEPROM
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
#if (DEF_ENABLED == USE_EEPROM)
static void Eeprom_Mac_Read(void)
{
  int32_t ret = 0;
  uint8_t i, eeprom_mac_count, count = 0, k = 0;
#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
  uint8_t n = 0;
#endif

  uint8_t offset_addr = *(uint8_t *) TC956X_M3_EEPROM_OFFSET_ADDR;
  uint8_t mac_addr_count = *(uint8_t *) TC956X_M3_MAC_COUNT;

  ARM_DRIVER_I2C * I2Cdrv = &DRIVER_I2C;
  eeprom_read_mac_addr[0] = offset_addr;
  if(mac_addr_count > TC956X_TWO) {
	  eeprom_mac_count = TC956X_EIGHT * mac_addr_count;
  } else if(mac_addr_count == TC956X_ONE) {
	  eeprom_mac_count = TC956X_SIX;
  } else {
	  eeprom_mac_count = TC956X_TWELVE;
  }
  ret = I2Cdrv->EepromRead(EEPROM_SLAVE_ADDRESS, TC956X_ZERO, eeprom_read_mac_addr,(eeprom_mac_count + TC956X_COMMON_ONE));
  if (ret != ARM_DRIVER_OK)
  {
    /* writing MAC address for MAC0*/
    for (i = TC956X_ZERO ; i < eeprom_mac_count; i++) {
      /* if EEPROM read fails write 0*/
      *( uint8_t* )( TC956X_M3_MAC_ADDR_START + k ) = default_eeprom_mac_addr[i];
	    if(k % TC956X_EIGHT == TC956X_FIVE)
        k += TC956X_THREE;
      else
        k++;
    }
#if ( DEF_ENABLED == TC956X_UART )
    TC956X_Ser_Printf("MAC address read from EEPROM failed\n");
#endif
  }
  else
  {
    /* writing MAC address for MAC0*/
    for (i = TC956X_ZERO ; i < eeprom_mac_count; i++) {
	    count++;

	    if(count == TC956X_SIXTEEN) {
		    count = TC956X_ZERO;
		    continue;
	    }

	    if(count > TC956X_TWELVE)
		    continue;

      *( uint8_t* )( TC956X_M3_MAC_ADDR_START + k ) = eeprom_read_mac_addr[i + 1];
	    if(k % TC956X_EIGHT == TC956X_FIVE)
        k += TC956X_THREE;
      else
        k++;
    }
#if ( DEF_ENABLED == TC956X_UART )
      TC956X_Ser_Printf("*** MAC address read from EEPROM successful ***\n");
#endif
  }
#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
  count = 0;
  for (i = TC956X_ZERO ; i < eeprom_mac_count; i++) {
   	count++;

	   if(count == TC956X_SIXTEEN) {
	    count = TC956X_ZERO;
	    continue;
	   }

	   if(count > TC956X_TWELVE)
	    continue;

    n++;

	  if((n != TC956X_ZERO) && !(n % TC956X_THIRTEEN)) {
  		n++;
  	}

    if (eeprom_mac_addr[n] == eeprom_read_mac_addr[i+1])
    {
      continue;
    }
    else
    {
#if ( DEF_ENABLED == TC956X_UART )
      TC956X_Ser_Printf("*** MAC address read from EEPROM incorrect %dth bit is 0x%X instead of 0x%X***\n", i + 1,
      eeprom_read_mac_addr[i+1], eeprom_mac_addr[n]);
#endif
    }
  }
#endif
  return;
}
/* End of Eeprom_Mac_Read */
#endif

/*
*  Function    :    main( )
*  Purpose     :    The standard entry point for C code. It is assumed
*                   that your code will call main() once you have
*                   performed all necessary initialization.
*  Inputs      :    None
*  Outputs     :    None
*  Return Value:    None
*  Limitation  :    None
*/
int32_t main (void)
{
  /* Iteration count */
  uint32_t uiCount ;
#if ( DEF_ENABLED == TC956X_UART )
  int08_t ver_str[32] ;
#endif

  *(uint8_t *) TC956X_M3_FW_INIT_DONE = TC956X_ZERO;

  for ( uiCount = TC956X_ZERO; uiCount < ( TC956X_M3_DBG_CNT_SIZE / TC956X_FOUR ); uiCount++ )
  {
    /* clear the count */
    *( uint32_t*)( TC956X_M3_DBG_CNT_START + ( uiCount * TC956X_FOUR ) ) = TC956X_ZERO ;
  }

  *(uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_SEVENTEEN * TC956X_FOUR ) ) = TC956X_ZERO;
  *(uint32_t*)( TC956X_M3_DBG_CNT_START + ( TC956X_EIGHTEEN * TC956X_FOUR ) ) = TC956X_ZERO;

  for ( uiCount = TC956X_ZERO; uiCount < ( SRAM_EVENT_LOC_SIZE / TC956X_FOUR ); uiCount++ )
  {
    /* clear the count */
    *( uint32_t*)( SRAM_TX_PCIE_ADDR_LOC + (uiCount * TC956X_FOUR ) ) = TC956X_ZERO;
    *( uint32_t*)( SRAM_RX_PCIE_ADDR_LOC + (uiCount * TC956X_FOUR ) ) = TC956X_ZERO;
  }

#if ( DEF_ENABLED == TC956X_UART )
  *( uint32_t*)(TC956X_M3_DBG_VER_START) = *( uint32_t *)(&version);
  snprintf(ver_str, 32, "Version %s_%d.%d-%d", (version.rel_dbg == 'R')?"REL":"DBG",
  version.major, version.minor, version.sub_minor);
#endif

#if ( DEF_ENABLED == TC956X_GPIO3_OUTPUT )
  /* Initialize GPIO */
  tc956x_gpio.Initialize();
#endif

  /* System Init function */
  SysInit( ) ;

#if ( DEF_ENABLED == TC956X_UART )
  /**  UART0 Init  **/
  Driver_USART0.Initialize(NULL);
#endif
  I2c_Init();
#if (DEF_ENABLED == TEST_EEPROM_WRITE_ENABLE)
  Eeprom_Mac_Write();
#endif

#if (DEF_ENABLED == USE_EEPROM)
  /* Disable EEPROM for bringup */
  Eeprom_Mac_Read();
#endif
  /* Enable WatchDog Timer Interrupt */
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_WDT ) ;

#ifdef DMA_OFFLOAD_ENABLE
  /* Enable Tx and Rx Interrupts for MAC0 and MAC1 */
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_MAC0_TXDMA ) ;
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_MAC0_RXDMA ) ;
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_MAC1_TXDMA ) ;
  NVIC_EnableIRQ( ( IRQn_Type )INT_SRC_NBR_MAC1_RXDMA ) ;
#endif

#if ( DEF_ENABLED == TC956X_UART )
  TC956X_Ser_Printf( "\r\nTC956X PCIe Firmware %s is Loaded Successfully and running on Cortex M3\r\n", ver_str ) ;
#endif

  *(uint8_t *) TC956X_M3_FW_INIT_DONE = TC956X_ONE;

  while ( true )
  {
    if ( (*(uint32_t *)TC956X_M3_FW_EXIT) == TC956X_TWO )
      break;

    __wfi();
  }

  /* Disable systick timer */
  CPU_REG_NVIC_ST_CTRL &= ~TC956X_COMMON_ONE;

  /* Disable clocks (eMAC0All, MSIGEN, MACRX/TX, INTC ) except default clocks */
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NCLKCTRL0_OFFS, TC956X_CLK_CTRL0_DISABLE ) ;
  Hw_Reg_Write32 ( TC956X_REG_BASE, TC956X_NCLKCTRL1_OFFS, TC956X_CLK_CTRL1_DISABLE ) ;


}
/* End of main */
