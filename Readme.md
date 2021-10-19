# Toshiba Electronic Devices & Storage Corporation TC956X PCIe Ethernet Bridge Firmware

Release Date: Oct 19 2021
Relase Version: V1.0.5 : Limited-tested version


# Introduction:
The folder contains a Keil project, which is the firmware for PCIe interface.

# Environment:
1. Keil uVision IDE Version 5 or Version 4 is used.  
   Evaluation licence should be OK, as the image size is much smaller than 32k. This project does not
   use any RTOS. It is a OSless firmware.
2. Keil JLink standard/Pro JTAG connector is used.

# Procedures:
1. Open the project file, compile it and run it. 
2. A binary file will be generated, and it is located at ./Bin
3. A windows tool may be used to convert the binary into header file format. If windows environment
   does not support this tool, please ignore this step.

#  Notes:
1. Some debugging counters are available:
   #define TC956X_M3_DBG_CNT_START            0x2000F800U
   #define TC956X_M3_DBG_CNT_SIZE             ( 20U*4U )

    /*
    * TC956X_M3_DBG_CNT_START + 4*0:  Reserved
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

2. Firmware Version
    #define TC956X_M3_DBG_VER_START      0x2000F900 // Firmware Version SRAM area start address

3. Disable DMA_OFFLOAD_ENABLE macro in \Src\Include\common.h to disable IPA feature.
   By default, DMA_OFFLOAD_ENABLE macro is enabled

# Release Versions:

## TC956X_Firmware_PCIeBridge_20210326_V1.0.0:
1. Initial Version

## TC956X_Firmware_PCIeBridge_20210705_V1.0.1:
1. Used Systick handler instead of Driver kernel timer to process transmitted Tx descriptors.
2. sprintf, vsprintf APIs replaced with vnsprintf or vnsprintf APIs
3. CPU Frequency corrected
4. Note that this release is only a limited test version.

## TC956X_Firmware_PCIeBridge_20210722_V1.0.2:
1. For IPA channel interrupts, clear only interrupt status

## TC956X_Firmware_PCIeBridge_20210723_V1.0.3:
1. Enable DMA IPA offload by default

## TC956X_Firmware_PCIeBridge_20210923_V1.0.4:
1. Triggering DoreBell only when Transmission/Reception completion
2. Handling RBU Interrupt at Host Driver and maintaining ethtool statistics

## TC956X_Firmware_PCIeBridge_20211019_V1.0.5:
1. Updated value of Debug Counters size as per No.of SRAM Debug counters (each counter of 4 Bytes Size).
2. Resetting all SRAM debug counters to Zeros at common place
