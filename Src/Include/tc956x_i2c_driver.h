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
 *  23 Feb 2021 : Baselined
 *  VERSION     : 1.0.0
 */

/*
 *********************************************************************************************************
 *
 * Target        : TC956X
 * Filename      : tc956x_i2c_driver.h
 *
 *********************************************************************************************************
 */

#ifndef TC956X_I2C_DRIVER_H_
#define TC956X_I2C_DRIVER_H_

#include "Driver_I2C.h"
#include "tc956x_regacc.h"
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#define CNF_REG_BASE            (0x40000000U)
#define I2C_REG_BASE            (0x4000D000U)

#define INT_MCUMASK0_I2C_MASK   0xFFFFFFDFU

#define CNF_REG_NMODESTS        0x0004U
#define CNF_REG_NFUNCEN0        0x0008U
#define CNF_REG_NCLKCTRL        0x1004U
#define CNF_REG_NRSTCTRL        0x1008U

#define IC_CON                  0x000U
#define IC_TAR                  0x004U
#define IC_SAR                  0x008U
#define IC_DATA_CMD             0x010U
#define IC_SS_SCL_HCNT          0x014U
#define IC_SS_SCL_LCNT          0x018U
#define IC_FS_SCL_HCNT          0x01cU
#define IC_FS_SCL_LCNT          0x020U
#define IC_INTR_START           0x02cU
#define IC_INTR_MASK            0x030U
#define IC_RAW_INTR_START       0x034U
#define IC_RX_TL                0x038U


#define IC_TX_TL                0x03cU
#define IC_CLR_INTR             0x040U
#define IC_CLR_RX_UNDER         0x044U
#define IC_CLR_RX_OVER          0x048U

#define IC_CLR_TX_OVER          0x04cU
#define IC_CLR_RD_REQ           0x050U
#define IC_CLR_TX_ABRT          0x054U
#define IC_CLR_RX_DONE          0x058U

#define IC_CLR_ACTIVITY         0x05cU
#define IC_CLR_STOP_DET         0x060U
#define IC_CLR_START_DET        0x064U
#define IC_CLR_GEN_CALL         0x068U

#define IC_ENABLE               0x06cU
#define IC_STATUS               0x070U
#define IC_TXFFLR               0x074U
#define IC_RXFLR                0x078U

#define IC_SDA_HOLD             0x07cU
#define IC_TX_ABRT_SOURCE       0x080U
#define IC_SLV_DATA_NACK_ONLY   0x084U
#define IC_DATA_CR              0x088U

#define IC_DMA_TDLR             0x08cU
#define IC_DMA_RDLR             0x090U
#define IC_SDA_SETUP            0x094U
#define IC_ACK_GENERAL_CALL     0x098U

#define IC_ENABLE_STATUS        0x09cU
#define IC_FS_SPKLEN            0x0A0U
#define IC_CLR_RESTART_DET      0x0A8U
#define IC_COMP_PARAM_1         0x0f4U
#define IC_COMP_VERSION         0x0f8U
#define IC_COMP_TYPE            0x0fcU

#define MODE_I2C_BIT_POS        8
#define I2C_MASTER_SLAVE_BIT    28
#define I2CSPIEN_BIT_POS        29
#define MODE_I2C_SLAVE_ADDR_BIT_POS 11
#define I2CCEN                  12
#define IC_SAR0                 0x77
#define IC_SAR1                 0x33
#define IC_CMD_BIT_POS          8
#define IC_STOP_BIT_POS         9

#define I2C_INITIALIZED   1U
#define I2C_UNINITIALIZED 0U

#define I2C_SCL_SDA_ENABLE 0x11U
#define I2C_SPI_ENABLE_POS 0x20000000U
#define I2C_SPI_ENABLE_BIT 29U
#define I2C_MASTER_ENABLE_BIT 28U
#define I2C_CLK_RST_BIT 12U
#define I2C_CLK_ENABLE 0x00001000U

#define ABT_NOT_INT 0x00000000U
#define ABT_IN_PROGRESS 0x00000001U

#define  I2C_INTR_MASK 0x0U

#define TC956X_I2C_ZERO 0x0U
#define TC956X_I2C_ONE  0x1U

/****** EEPROM SLAVE ADDRESS ******/

#define EEPROM_SLAVE_ADDRESS    (0x50U)      //< 7 bit slave address of EEPROM device
#define EEPROM_PAGE_LIMIT       (0x7U)

extern ARM_DRIVER_I2C DRIVER_I2C;
#endif
