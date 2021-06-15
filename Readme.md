# Toshiba Electronic Devices & Storage Corporation TC956X PCIe Ethernet Bridge Firmware

Release Date: Mar 26 2021
Relase Version: V_1.0.0


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
    #define  TC956X_M3_DBG_CNT_SIZE   0x2000F800  // Debugging count SRAM area start address
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
    * TC956X_M3_DBG_CNT_START + 4*15:  guiM3Ticks
    *   .........................
    * TC956X_M3_DBG_CNT_START + 4*17:  Reserved
    */

2. Firmware Version
    #define TC956X_M3_DBG_VER_START      0x2000F900 // Firmware Version SRAM area start address

# Release Versions:

## TC956X_Firmware_PCIeBridge_20210326_V1.0.0:
1. Initial Version
