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
*  22 Jul 2021  : For IPA channel interrupts, clear only interrupt status
*  VERSION      : 1.0.2
*  23 Sep 2021  : 1.Triggering DoreBell only when Transmission/Reception completion
                  2.Handling RBU Interrupt at Host Driver and maintaining ethtool statistics
*  VERSION      : 1.0.4
*  04 Feb 2022  : 1.Invoking Doorbell for IPA DMA channels only. Skipped for SW path channels based on MCU mask enabled in the driver.
*  VERSION      : 1.0.6
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

#ifdef ENABLE_MAC2MAC_BRIDGE
#include "tc956x_uart_driver.h"
#include <string.h>


#define MAC2MAC_UART_DEBUG

#ifdef MAC2MAC_UART_DEBUG
#define MAC2MAC_DEBUG_PRINT(x...) do{TC956X_Ser_Printf(x); }while(0)
#else
#define MAC2MAC_DEBUG_PRINT(x...) do{}while(0)
#endif

#define ETH_PORT0  0
#define ETH_PORT1  1
#define NUM_PORT   2

/* Tx/Rx Queue */
#define ETH0_PCI2MAC_QUEUE                (0)
#define ETH0_MAC2MAC_UNICAST_QUEUE        (1)
#define ETH0_MAC2MAC_BROAD_MULTI_QUEUE    (7)
#define ETH0_MAC2MAC_TX_QUEUE             (2)

#define ETH1_PCI2MAC_QUEUE                (ETH0_PCI2MAC_QUEUE)
#define ETH1_MAC2MAC_UNICAST_QUEUE        (ETH0_MAC2MAC_UNICAST_QUEUE)
#define ETH1_MAC2MAC_BROAD_MULTI_QUEUE    (ETH0_MAC2MAC_BROAD_MULTI_QUEUE)
#define ETH1_MAC2MAC_TX_QUEUE             (ETH0_MAC2MAC_TX_QUEUE)

#define PCI2MAC_QUEUE(n) \
((n == 0)?(ETH0_PCI2MAC_QUEUE):(ETH1_PCI2MAC_QUEUE))
#define MAC2MAC_UNICAST_QUEUE(n) \
((n == 0)?(ETH0_MAC2MAC_UNICAST_QUEUE):(ETH1_MAC2MAC_UNICAST_QUEUE))
#define MAC2MAC_BROAD_MULTI_QUEUE(n) \
((n == 0)?(ETH0_MAC2MAC_BROAD_MULTI_QUEUE):(ETH1_MAC2MAC_BROAD_MULTI_QUEUE))
#define MAC2MAC_TX_QUEUE(n) \
((n == 0)?(ETH0_MAC2MAC_TX_QUEUE):(ETH1_MAC2MAC_TX_QUEUE))

/* Tx/Rx DMA */
#define ETH0_BRIDGE_TxCH  (2)
#define ETH1_BRIDGE_TxCH  (ETH0_BRIDGE_TxCH)
#define ETH0_BRIDGE_RxCH  (2)
#define ETH1_BRIDGE_RxCH  (ETH0_BRIDGE_RxCH)
#define BRIDGE_TXCH(n) ((n == 0)?(ETH0_BRIDGE_TxCH):(ETH1_BRIDGE_TxCH))
#define BRIDGE_RXCH(n) ((n == 0)?(ETH0_BRIDGE_RxCH):(ETH1_BRIDGE_RxCH))

#define MAC2MAC_RX_PBL  (16)
#define MAC2MAC_TX_PBL  (16)
#define MAC2MAC_PBLx8    (0x10000)
#define MAC2MAC_DESCLEN_MASK    (0xFFFF)
#define MAC2MAC_DESCBUF_MASK    (0x3FFF)
#define MAC2MAC_TXQ_TIMEOUT     (10000)

/* MCU flag */
#define MAC2MAC_DRIVER_ETH0_INIT_DONE   (0x1 << 0)
#define MAC2MAC_DRIVER_ETH1_INIT_DONE   (0x1 << 1)
#define MAC2MAC_DRIVER_ETHX_EXIT        (0x1 << 2)
#define MAC2MAC_DRIVER_INIT_DONE \
(MAC2MAC_DRIVER_ETH0_INIT_DONE | MAC2MAC_DRIVER_ETH1_INIT_DONE)
#define MAC2MAC_CLEAR_SW_COUNTER        (0x1 << 7)

#define MAC2MAC_WDExpire                (0xF424)

#define ETH0_RX_POLL_WEIGHT             (128)
#define ETH1_RX_POLL_WEIGHT             (ETH0_RX_POLL_WEIGHT)

#define MAC2MAC_RX_POLL_WEIGHT(port) \
((port==ETH_PORT0)?(ETH0_RX_POLL_WEIGHT):(ETH1_RX_POLL_WEIGHT))

#define XGMAC_BROADCAST_ADDROFF         (0x0U)

#define MAC2MAC_ETH0_NUM_RXDESC         (512)
#define MAC2MAC_ETH1_NUM_TXDESC         (MAC2MAC_ETH0_NUM_RXDESC)
#define MAC2MAC_ETH1_NUM_RXDESC         (512)
#define MAC2MAC_ETH0_NUM_TXDESC         (MAC2MAC_ETH1_NUM_RXDESC)

#define MAC2MAC_RX_BUFF_SIZE            (1536)

/* Port0 -> Port1 */
#define RXDESC_BASE_ADDR0               (0x20007800) /* Port-0 Rx Descriptor */
#define TXDESC_BASE_ADDR0               (0x20005000) /* Port-1 Tx Descriptor */

/* Port1 -> Port0 */
#define RXDESC_BASE_ADDR1               (0x2000B800) /* Port-1 Rx Descriptor */
#define TXDESC_BASE_ADDR1               (0x20002000) /* Port-0 Tx Descriptor */

#ifdef MAC2MAC_DEBUG_INFO
#define MAC2MAC_SW_COUNTER_OFFS         (0x20004100)
#endif

#define MAC2MAC_NUM_TXDESC(port) \
((port==ETH_PORT0)?(MAC2MAC_ETH0_NUM_TXDESC):(MAC2MAC_ETH1_NUM_TXDESC))
#define MAC2MAC_NUM_RXDESC(port) \
((port==ETH_PORT0)?(MAC2MAC_ETH0_NUM_RXDESC):(MAC2MAC_ETH1_NUM_RXDESC))

#define INT_MCUMASK0_MASK_ALL       0xFFFF1FFFU
#define INT_MCUMASK1_MASK_ALL       0xFFFF1F80U
#define INT_MCUMASK2_MASK_ALL       0x0000F7C0U
#define INTINTWDCTL_DISABLE         0x0

typedef struct eth_desc
{
  uint32_t desc0;
  uint32_t desc1;
  uint32_t desc2;
  uint32_t desc3;
}eth_desc;

typedef struct bridge_dma_desc
{
  eth_desc *head;
  uint32_t tail;
  uint16_t ring_len;
  uint64_t buf_head;
  uint32_t buf_size;
  uint16_t offs;
  uint16_t dirty_offs;
}bridge_dma_desc;

typedef struct bridge_mac_info
{
  struct bridge_dma_desc txdesc;
  struct bridge_dma_desc rxdesc;
  uint8_t tx_wd;
  uint8_t rx_wd;
}bridge_mac_info;

#ifdef MAC2MAC_DEBUG_INFO
struct mac2mac_sw_counter{
  /* Tx packet */
  uint32_t tx_pkt_n;
  uint32_t tx_good_pkt_n;
  /* Tx DMA */
  uint32_t tx_dma_own_n;
  uint32_t tx_irq_n;
  uint32_t tx_axi_err_n;
  uint32_t tx_stop_n;

  /* Rx packet */
  uint32_t rx_pkt_n;
  uint32_t rx_pkt_len;
  uint32_t rx_good_pkt_n;
  /* Rx DMA */
  uint32_t rx_dma_own_n;

  //uint32_t rx_not_last_n;
  uint32_t rx_first_desc_offs;
  uint32_t rx_last_desc_offs;
  uint32_t rx_dirty_first_offs;
  uint32_t rx_dirty_last_offs;

  uint32_t rx_dma_own_desc;
  uint32_t rx_buf_unav_n;
  uint32_t rx_error_n;
  uint32_t rx_error_desc;

  uint32_t rx_irq_n;
  uint32_t rx_cpu_wd_irq;

  /* Rx Queue */
  uint32_t rx_overflow_n[2];

};
struct mac2mac_sw_counter *gbridge_stats
  = (struct mac2mac_sw_counter *)MAC2MAC_SW_COUNTER_OFFS;

#endif /* MAC2MAC_DEBUG_INFO */

static eth_desc *p0p1_txch_desc = (eth_desc *)TXDESC_BASE_ADDR0;
static eth_desc *p0p1_rxch_desc = (eth_desc *)RXDESC_BASE_ADDR0;

static eth_desc *p1p0_txch_desc = (eth_desc *)TXDESC_BASE_ADDR1;
static eth_desc *p1p0_rxch_desc = (eth_desc *)RXDESC_BASE_ADDR1;

static struct bridge_mac_info mac_info[NUM_PORT];
static uint8_t mac2mac_save_mcu_flag = 0;

/* Tx status */
enum tx_frame_status {
  tx_done    = TC956X_ZERO,
  tx_not_ls  = TC956X_ONE << TC956X_ZERO,
  tx_err     = TC956X_ONE << TC956X_ONE,
  tx_dma_own = TC956X_ONE << TC956X_TWO,
};

/* Rx status */
enum rx_frame_status {
  good_frame    = TC956X_ZERO,
  discard_frame = TC956X_ONE << TC956X_ZERO,
  dma_own       = TC956X_ONE << TC956X_ONE,
  rx_not_ls     = TC956X_ONE << TC956X_TWO,
  rx_context    = TC956X_ONE << TC956X_THREE,
  rx_fs_not_ls  = TC956X_ONE << TC956X_FOUR,
};
#endif /* ENABLE_MAC2MAC_BRIDGE */

#ifndef ENABLE_MAC2MAC_BRIDGE
extern int32_t  TC956X_Ser_Printf (char *format, ...);
void PCIeFLR_IRQHandler(void);
#endif
/*=====================================================================
FUNCTION DEFINITION
======================================================================*/
#ifdef ENABLE_MAC2MAC_BRIDGE
void modify_func (uint32_t uiAddr, uint32_t uiMask, uint32_t uiVal)
{
  uint32_t uiOldVal;   /* store data read from memory */

  /* read -> modify -> write*/
  uiOldVal = read32(uiAddr);
  uiOldVal &= ~(uiMask);
  uiVal &= uiMask;
  uiVal |= uiOldVal;
  write32(uiAddr, uiVal);
}

void Set_tx_desc (struct eth_desc *desc,
                  uint64_t buf_addr, bool own, bool ioc, uint32_t len)
{
  uint32_t uiVal = 0;

  desc->desc0 = buf_addr & XGMAC_DESC_MASKADDLOW;  // addr[31:0]
  desc->desc1 = buf_addr >> TC956X_THIRTYTWO;      // addr[63:32]

  if (ioc) {
    uiVal = (TC956X_ONE << TC956X_THIRTYONE);  // IOC
  }

  desc->desc2 = len;

  uiVal = ((TC956X_ONE << TC956X_TWENTYNINE) | (TC956X_ONE << TC956X_TWENTYEIGHT));  // FD,LD
  uiVal |= len;              // FL/TPL
  if (own) {
    uiVal |= (TC956X_ONE << TC956X_THIRTYONE);  // OWN
  }
  desc->desc3 = uiVal;
}

void Init_mac2mac_txdesc (void)
{
  uint32_t uiDescCount;

  for (uiDescCount = 0; uiDescCount < MAC2MAC_NUM_TXDESC(ETH_PORT0); uiDescCount++)
  {
    /* Port1->Port0 */
    Set_tx_desc(&p1p0_txch_desc[uiDescCount]
           ,mac_info[ETH_PORT0].txdesc.buf_head
            + (uiDescCount * MAC2MAC_RX_BUFF_SIZE) /* Buffer Address */
          , false    /* OWN */
          , true    /* IOC */
          , TC956X_ZERO);  /* length */
  }

  for (uiDescCount = 0; uiDescCount < MAC2MAC_NUM_TXDESC(ETH_PORT1); uiDescCount++)
  {
    /* Port0->Port1 */
    Set_tx_desc(&p0p1_txch_desc[uiDescCount]
          , mac_info[ETH_PORT1].txdesc.buf_head
            + (uiDescCount * MAC2MAC_RX_BUFF_SIZE) /* Buffer Address */
          , false    /* OWN */
          , true    /* IOC */
          , TC956X_ZERO);  /* length */
  }
}

void Set_rx_desc (struct eth_desc *desc, uint64_t buf_addr, bool own, bool ioc)
{
  uint32_t uiVal =0;

  desc->desc0 = buf_addr & XGMAC_DESC_MASKADDLOW;  // addr[31:0]
  desc->desc1 = buf_addr >> TC956X_THIRTYTWO;      // addr[63:32]
  desc->desc2 = 0;

  if (own) {
    uiVal |= (TC956X_ONE << TC956X_THIRTYONE);  // OWN
  }
  if (ioc) {
    uiVal |= (TC956X_ONE << TC956X_THIRTY);  // IOC
  }
  desc->desc3 = uiVal;
}

static void Init_mac2mac_rxdesc (void)
{
  uint32_t uiDescCount;

  for (uiDescCount = TC956X_ZERO; uiDescCount < MAC2MAC_NUM_RXDESC(ETH_PORT0); uiDescCount++)
  {
    /* Port0->Port1 */
    Set_rx_desc(&p0p1_rxch_desc[uiDescCount]
          , mac_info[ETH_PORT0].rxdesc.buf_head
            + (uiDescCount * MAC2MAC_RX_BUFF_SIZE)  /* Buffer Address */
          , true    /* OWN */
          , true);  /* IOC */
  }

  for (uiDescCount = TC956X_ZERO; uiDescCount < MAC2MAC_NUM_RXDESC(ETH_PORT1); uiDescCount++)
  {
    /* Port1->Port0 */
    Set_rx_desc(&p1p0_rxch_desc[uiDescCount]
          , mac_info[ETH_PORT1].rxdesc.buf_head
            + (uiDescCount * MAC2MAC_RX_BUFF_SIZE)  /* Buffer Address */
          , true    /* OWN */
          , true);  /* IOC */
  }
}

uint32_t tc9563_get_rx_status
(
  eth_desc *rxdesc  /* pointer of descriptor */
)
{
  uint32_t uiStatus = good_frame;

  if (rxdesc->desc3 & (TC956X_ONE << TC956X_THIRTYONE)) /* Own bit is set yet */
  {
    uiStatus |= dma_own;
  }
  if (rxdesc->desc3 & (TC956X_ONE << TC956X_THIRTY)) /* Context format */
  {
    uiStatus |= rx_context;
  }
  if (!(rxdesc->desc3 & (TC956X_ONE << TC956X_TWENTYEIGHT))) /* Not last descriptor */
  {
    if (rxdesc->desc3 & (TC956X_ONE << TC956X_TWENTYNINE)) /* First descriptor */
    {
       uiStatus |= rx_fs_not_ls;
    }
    else
    {
      uiStatus |= rx_not_ls;
    }
  }
  if (rxdesc->desc3 & (TC956X_ONE << TC956X_FIFTEEN)   /* error is occurred */
    && rxdesc->desc3 & (TC956X_ONE << TC956X_TWENTYEIGHT))
  {
    uiStatus |= discard_frame;
  }

  return uiStatus;
}

uint32_t tc9563_get_tx_status
(
  eth_desc *txdesc  /* pointer of descriptor */
)
{
  uint32_t uiStatus = tx_done;

  if (txdesc->desc3 & (TC956X_ONE << TC956X_THIRTYONE))
  {
    uiStatus |= tx_dma_own;
  }
  if (!(txdesc->desc3 & (TC956X_ONE << TC956X_TWENTYEIGHT)))
  {
    uiStatus |= tx_not_ls;
  }
  if (txdesc->desc3 & (TC956X_ONE << TC956X_TWENTYSEVEN))
  {
    uiStatus |= tx_err;
  }

  return uiStatus;
}

static void tc9563_BridgeTrasfer
(
  uint8_t rx_port,  /* Rx EthPort number (0 or 1) */
  uint8_t rx_ch,    /* RxDMA channel number (Expected=2) */
  uint8_t tx_ch      /* TxDMA channel number (Expected=2) */
)
{
  uint32_t uiRegVal,uiIntVal,uiRxBaseAddr,uiTxBaseAddr,uiRxBaseAddr1;
  eth_desc *txdesc_curr;
  uint32_t uiLength, uiStatus, uiTimeout =0;
  uint8_t tx_port = TC956X_ONE - rx_port;
  bool rx_overflow = false, rx_ioc = true;
  int32_t iVal, iDirty, iCount, iOffs;
  bridge_dma_desc *txdesc, *rxdesc;

  txdesc = &mac_info[tx_port].txdesc;
  rxdesc = &mac_info[rx_port].rxdesc;

  uiTxBaseAddr = tx_port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;
  uiRxBaseAddr = rx_port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;
  uiRxBaseAddr1 = (TC956X_ONE-rx_port) ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

 // Clear IRQ
  uiRegVal = Hw_Reg_Read32(uiRxBaseAddr, XGMAC_DMA_CH_STATUS(rx_ch));
  uiIntVal = Hw_Reg_Read32(uiRxBaseAddr, XGMAC_DMA_CH_INT_EN(rx_ch));
  Hw_Reg_Write32(uiRxBaseAddr, XGMAC_DMA_CH_STATUS(rx_ch), (uiRegVal & uiIntVal));

  /* Disable MCU interrupt */
  modify_func(TC956X_INTC_REG_BASE + (INTMCUMASK0_OFFS+TC956X_FOUR*rx_port)
        , TC956X_ONE << (TC956X_TWENTYFOUR+rx_ch) , TC956X_ONE << (TC956X_TWENTYFOUR+rx_ch));

  /* Set Receive Buffer Unavailable bit */
  if (uiRegVal & EMAC_DMA_CH_STS_RBU)
  {
#ifdef MAC2MAC_DEBUG_INFO
    gbridge_stats[rx_port].rx_buf_unav_n++;
#endif
    rx_overflow = TC956X_ONE;
  }

  /* Check MTL interrupt */
  uiRegVal = Hw_Reg_Read32(uiRxBaseAddr
    , XGMAC_MTL_Q_Interrupt_Status(MAC2MAC_UNICAST_QUEUE(rx_port)));
  if (uiRegVal & (TC956X_ONE << TC956X_SIXTEEN))
  {
    Hw_Reg_Write32(uiRxBaseAddr
      , XGMAC_MTL_Q_Interrupt_Status(MAC2MAC_UNICAST_QUEUE(rx_port)), uiRegVal);
#ifdef MAC2MAC_DEBUG_INFO
    gbridge_stats[rx_port].rx_overflow_n[TC956X_ZERO]++;
#endif
    rx_overflow = true;
  }

  /* Check dirty Rx descriptor */
  iOffs = rxdesc->dirty_offs;
  rxdesc->offs = Hw_Reg_Read32(uiRxBaseAddr, XGMAC_RX_WR_RINGOFF(rx_ch));
  if (iOffs <= rxdesc->offs)
  {
    iDirty = rxdesc->offs - iOffs + TC956X_ONE;
  }
  else
  {
    iDirty = rxdesc->ring_len - (iOffs - rxdesc->offs) + TC956X_ONE;
  }

  if (iDirty > MAC2MAC_RX_POLL_WEIGHT(rx_port))
  {
    iDirty = MAC2MAC_RX_POLL_WEIGHT(rx_port);
  }

#ifdef MAC2MAC_DEBUG_INFO
  gbridge_stats[rx_port].rx_first_desc_offs = iOffs;
  gbridge_stats[rx_port].rx_last_desc_offs = rxdesc->offs;
#endif

  /* Get Rx Data & Kick TxDMA */
  iCount = iDirty;
  iVal = iOffs;
  while (true)
  {
    /* Check Rx descriptor */
    uiStatus = tc9563_get_rx_status(rxdesc->head + iVal);
    if (uiStatus == good_frame) /* Completed Rx  */
    {
      goto Get_length;
    }
    if (uiStatus & dma_own) /* Owned by RxDMA */
    {
#ifdef MAC2MAC_DEBUG_INFO
      gbridge_stats[rx_port].rx_dma_own_n++;
      gbridge_stats[rx_port].rx_dirty_first_offs = iOffs;
      gbridge_stats[rx_port].rx_dirty_last_offs = rxdesc->offs;
      gbridge_stats[rx_port].rx_dma_own_desc = (rxdesc->head+iVal)->desc3;
#endif
      break;
    }
    if (uiStatus & rx_context) /* Current descriptor is context format */
    {
      goto Rx_clean;
    }

    if (uiStatus & discard_frame) /* Error is occurred */
    {
#ifdef MAC2MAC_DEBUG_INFO
      gbridge_stats[rx_port].rx_error_n++;
      gbridge_stats[rx_port].rx_error_desc = (rxdesc->head+iVal)->desc3;
#endif
      goto Rx_clean;
    }

Get_length:
    uiLength = ((rxdesc->head+iVal)->desc3 & DESC_PKT_LEN_MASK);
    if (uiLength == TC956X_ZERO)
    {
#ifdef MAC2MAC_DEBUG_INFO
      gbridge_stats[rx_port].rx_error_n++;
#endif
      goto Rx_clean;
    }

    txdesc_curr = (eth_desc *)txdesc->tail;
Check_Tx_Desc:
    if (txdesc_curr->desc3 & (TC956X_ONE << TC956X_THIRTYONE))
    {
      uiTimeout++;
      if (uiTimeout < MAC2MAC_TXQ_TIMEOUT)
      {
        goto Check_Tx_Desc;
      }
#ifdef MAC2MAC_DEBUG_INFO
      gbridge_stats[tx_port].tx_dma_own_n++;
#endif
      uiTimeout = 0U;
      break;
    }
    uiTimeout = 0U;
#ifdef MAC2MAC_DEBUG_INFO
    if (txdesc_curr->desc3 & (TC956X_ONE << TC956X_TWENTYSEVEN))
    {
      gbridge_stats[tx_port].tx_axi_err_n++;
    }
    gbridge_stats[rx_port].rx_good_pkt_n++;
#endif

    Set_tx_desc(txdesc_curr
        , mac_info[rx_port].rxdesc.buf_head + iVal * MAC2MAC_RX_BUFF_SIZE
        , TC956X_ONE /* OWN  */
        , TC956X_ONE /* IOC */
        , uiLength); /* length */

    if ((uint32_t)(txdesc_curr+1) >= ((uint32_t)(txdesc->head)
      + TC956X_SIXTEEN * txdesc->ring_len))
    {
      txdesc->tail = (uint32_t)(txdesc->head);
    }
    else
    {
      txdesc->tail = (uint32_t)(txdesc_curr + TC956X_ONE);
    }

    /* Kick TxDMA */
    Hw_Reg_Write32(uiTxBaseAddr, XGMAC_DMA_CH_TxDESC_TAIL_LPTR(tx_ch)
      , txdesc->tail);

Rx_clean:
    /* Refill Rx descriptor */
    Set_rx_desc(rxdesc->head + iVal  /* Pointer of descriptor */
          , mac_info[rx_port].rxdesc.buf_head + iVal * MAC2MAC_RX_BUFF_SIZE
          , true    /* OWN */
          , rx_ioc);  /* IOC */

#ifdef MAC2MAC_DEBUG_INFO
    gbridge_stats[rx_port].rx_pkt_n++;
    gbridge_stats[rx_port].rx_pkt_len += uiLength;
#endif

    iCount--;
    iVal = (iVal+TC956X_ONE) % rxdesc->ring_len;
    if (iCount <= TC956X_ZERO)
    {
      break;
    }
  }
  rxdesc->dirty_offs = iVal;

  if (rx_overflow)
  {
    Hw_Reg_Write32(uiRxBaseAddr, XGMAC_DMA_CH_RxDESC_TAIL_LPTR(rx_ch)
      , rxdesc->tail);
  }

  /* Check DMA interrupt of the other EMAC port */
  uiRegVal = Hw_Reg_Read32(uiRxBaseAddr1, XGMAC_DMA_CH_STATUS(BRIDGE_RXCH(TC956X_ONE-rx_port)));
  if (!(uiRegVal & EMAC_DMA_CH_STS_RI))
  {
    /* Enable MCU interrupt for own EMAC port */
    modify_func(TC956X_INTC_REG_BASE + (INTMCUMASK0_OFFS+TC956X_FOUR*rx_port)
      , TC956X_ONE << (TC956X_TWENTYFOUR+rx_ch) , TC956X_ZERO << (TC956X_TWENTYFOUR+rx_ch));
  }
  /* Enable MCU interrupt  */
  modify_func(TC956X_INTC_REG_BASE + (INTMCUMASK0_OFFS+TC956X_FOUR*(TC956X_ONE-rx_port))
    , TC956X_ONE << (TC956X_TWENTYFOUR+BRIDGE_RXCH(TC956X_ONE-rx_port)) , TC956X_ZERO << (TC956X_TWENTYFOUR+BRIDGE_RXCH(TC956X_ONE-rx_port)));

}

static void Start_txdma (uint8_t port, uint8_t txch)
{
  uint32_t uiRegVal;
  uint32_t uiMacOffset = (port)?(XGMAC_MAC_OFFSET1):(XGMAC_MAC_OFFSET0);

  uiRegVal = Hw_Reg_Read32(uiMacOffset, XGMAC_DMA_CH_TX_Control(txch));
  uiRegVal |= TC956X_ONE;
  Hw_Reg_Write32(uiMacOffset, XGMAC_DMA_CH_TX_Control(txch), uiRegVal);
}

static void Start_rxdma (uint8_t port, uint8_t rxch)
{
  uint32_t uiRegVal;
  uint32_t uiMacOffset = (port)?(XGMAC_MAC_OFFSET1):(XGMAC_MAC_OFFSET0);

  uiRegVal = Hw_Reg_Read32(uiMacOffset, XGMAC_DMA_CH_RX_Control(rxch));
  uiRegVal |= TC956X_ONE;
  Hw_Reg_Write32(uiMacOffset, XGMAC_DMA_CH_RX_Control(rxch), uiRegVal);
}

static void Stop_txdma (uint8_t port, uint8_t txch)
{
  uint32_t uiRegVal;
  uint32_t uiMacOffset = (port)?(XGMAC_MAC_OFFSET1):(XGMAC_MAC_OFFSET0);

  uiRegVal = Hw_Reg_Read32(uiMacOffset, XGMAC_DMA_CH_TX_Control(txch));
  uiRegVal &= ~TC956X_ONE;
  Hw_Reg_Write32(uiMacOffset, XGMAC_DMA_CH_TX_Control(txch), uiRegVal);
}

static void Stop_rxdma (uint8_t port, uint8_t rxch)
{
  uint32_t uiRegVal;
  uint32_t uiMacOffset = (port)?(XGMAC_MAC_OFFSET1):(XGMAC_MAC_OFFSET0);

  uiRegVal = Hw_Reg_Read32(uiMacOffset, XGMAC_DMA_CH_RX_Control(rxch));
  uiRegVal &= ~TC956X_ONE;
  Hw_Reg_Write32(uiMacOffset, XGMAC_DMA_CH_RX_Control(rxch), uiRegVal);
}

static void Start_port0_to_port1_dma (void)
{
  Start_rxdma(TC956X_ZERO, ETH0_BRIDGE_RxCH);
  Start_txdma(TC956X_ONE, ETH1_BRIDGE_TxCH);
}

static void Start_port1_to_port0_dma (void)
{
  Start_rxdma(TC956X_ONE, ETH1_BRIDGE_RxCH);
  Start_txdma(TC956X_ZERO, ETH0_BRIDGE_TxCH);
}

static void Stop_port0_to_port1_dma (void)
{
  Stop_rxdma(TC956X_ZERO, ETH0_BRIDGE_RxCH);
  Stop_txdma(TC956X_ONE, ETH1_BRIDGE_TxCH);
}

static void Stop_port1_to_port0_dma (void)
{
  Stop_rxdma(TC956X_ONE, ETH1_BRIDGE_RxCH);
  Stop_txdma(TC956X_ZERO, ETH0_BRIDGE_TxCH);
}

void Set_txdma_tail_ptr (uint8_t port, uint8_t txch, uint32_t addr)
{
  Hw_Reg_Write32(port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0 ,
    XGMAC_DMA_CH_TxDESC_TAIL_LPTR(txch), addr);
}

static void Init_txdma (uint8_t port, uint8_t txch,
  uint16_t pbl, uint16_t desc_len, uint32_t head_addr_H, uint32_t head_addr_L)
{
  uint32_t uiRegVal;
  uint32_t uiBaseAddr;

  uiBaseAddr = port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_CONTROL(txch));
  uiRegVal = MAC2MAC_PBLx8; //pbl8
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_CONTROL(txch), uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_TX_Control(txch));
  uiRegVal &= ~XGMAC_TxPBL_MASK;
  uiRegVal |= (pbl << XGMAC_RxPBL_SHIFT);
  uiRegVal |= XGMAC_OSP;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_TX_Control(txch), uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_TX_CONTROL2(txch));
  uiRegVal &= ~XGMAC_TDRL_MASK;
  uiRegVal |= (desc_len -1);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_TX_CONTROL2(txch), uiRegVal);

  uiRegVal = head_addr_H;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_TxDESC_HADDR(txch), uiRegVal);

  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_TxDESC_LADDR(txch), head_addr_L);

  Set_txdma_tail_ptr(port, txch, head_addr_L);

  uiRegVal = XGMAC_DMA_INT_DEFAULT_EN;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_INT_EN(txch), uiRegVal);
}

void Set_rxdma_tail_ptr (uint8_t port, uint8_t rxch, uint32_t addr)
{
  Hw_Reg_Write32(port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0,
    XGMAC_DMA_CH_RxDESC_TAIL_LPTR(rxch), addr);
}

static void Init_rxdma (uint8_t port, uint8_t rxch,
  uint16_t pbl, uint16_t buff_size, uint16_t desc_len,
  uint32_t head_addr_H, uint32_t head_addr_L)
{
  uint32_t uiRegVal;
  uint32_t uiBaseAddr;

  uiBaseAddr = port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_RX_Control(rxch));
  uiRegVal &= ~XGMAC_RX_PBL_MASK;
  uiRegVal |= (pbl << XGMAC_RxPBL_SHIFT);

  uiRegVal &= ~XGMAC_RBSZ_MASK;
  uiRegVal |= (buff_size << XGMAC_RBSZ_SHIFT);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_RX_Control(rxch), uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_RX_CONTROL2(rxch));
  uiRegVal &= ~XGMAC_OWRQ_MASK;
  uiRegVal |= (TC956X_THREE << XGMAC_OWRQ_SHIFT);
  uiRegVal &= ~XGMAC_RDRL_MASK;
  uiRegVal |= (desc_len -TC956X_ONE);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_RX_CONTROL2(rxch), uiRegVal);

  uiRegVal = head_addr_H;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_RxDESC_HADDR(rxch), uiRegVal);

  uiRegVal = head_addr_L;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_RxDESC_LADDR(rxch), uiRegVal);

  Set_rxdma_tail_ptr(port, rxch, head_addr_L + TC956X_SIXTEEN * desc_len);

  uiRegVal = XGMAC_DMA_INT_DEFAULT_EN;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_DMA_CH_INT_EN(rxch), uiRegVal);

}

static void Init_mac2mac_dma (void)
{
  uint8_t uiPort;

  for (uiPort=TC956X_ZERO; uiPort < NUM_PORT; uiPort++)
  {
    Init_rxdma(uiPort,
          BRIDGE_RXCH(uiPort),
          MAC2MAC_RX_PBL,
          MAC2MAC_RX_BUFF_SIZE,
          MAC2MAC_NUM_RXDESC(uiPort),
          TC956X_ZERO,
          (uiPort==ETH_PORT0)?(RXDESC_BASE_ADDR0):(RXDESC_BASE_ADDR1));

    Init_txdma(TC956X_ONE-uiPort,
          BRIDGE_TXCH(TC956X_ONE-uiPort),
          MAC2MAC_TX_PBL,
          MAC2MAC_NUM_TXDESC(TC956X_ONE-uiPort),
          TC956X_ZERO,
          (uiPort==ETH_PORT0)?(TXDESC_BASE_ADDR0):(TXDESC_BASE_ADDR1));
  }
}

static void Init_mac2mac_queue_routing (uint8_t port)
{
  uint32_t uiRegVal;
  uint32_t uiBaseAddr;

  uiBaseAddr = port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

  /* Enable RxQueue */
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL0);
  if (((uiRegVal >> (TC956X_TWO*PCI2MAC_QUEUE(port))) & RXQEN_MASK) != MTL_QUEUE_DCB)
  {
    /* RxQ for PCI2MAC Bridge -> DCB/General */
    uiRegVal &= ~(RXQEN_MASK << PCI2MAC_QUEUE(port));
    uiRegVal |= (MTL_QUEUE_DCB << PCI2MAC_QUEUE(port));
  }
  if (((uiRegVal >> (TC956X_TWO*MAC2MAC_UNICAST_QUEUE(port))) & RXQEN_MASK)
    != MTL_QUEUE_DCB)
  {
    /* Unicast RxQ for MAC2MAC Bridge -> DCB/General */
    uiRegVal &= ~(RXQEN_MASK << MAC2MAC_UNICAST_QUEUE(port));
    uiRegVal |= (MTL_QUEUE_DCB << MAC2MAC_UNICAST_QUEUE(port));
  }
  if (((uiRegVal >> (TC956X_TWO*MAC2MAC_BROAD_MULTI_QUEUE(port))) & RXQEN_MASK)
    != MTL_QUEUE_DCB)
  {
    /* Multicast/Broadcast RxQ -> DCB/General */
    uiRegVal &= ~(RXQEN_MASK << MAC2MAC_BROAD_MULTI_QUEUE(port));
    uiRegVal |= (MTL_QUEUE_DCB << MAC2MAC_BROAD_MULTI_QUEUE(port));
  }
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL0, uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL1);
  if ((uiRegVal & UPQ_MASK) != PCI2MAC_QUEUE(port))
  {
    /* Untagged passed packets -> RxQ for PCI2MAC Bridge */
    uiRegVal &= ~UPQ_MASK;
    uiRegVal |= PCI2MAC_QUEUE(port);
  }
  if (((uiRegVal >> MCBCQ_SHIFT) & MCBCQ_MASK) != MAC2MAC_BROAD_MULTI_QUEUE(port))
  {
    /* Multicast/Broadcast -> Multi/Broadcast RxQ */
    uiRegVal &= ~(MCBCQ_MASK << MCBCQ_SHIFT);
    uiRegVal |= (MAC2MAC_BROAD_MULTI_QUEUE(port) << MCBCQ_SHIFT)
      | (TC956X_ONE << MCBCQEN_SHIFT);
  }
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL1, uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL4);
  uiRegVal &= ~UFFQ_MASK;
  /* Unicast Failed packets -> RxQ for MAC2MAC Bridge  */
  uiRegVal |= (MAC2MAC_UNICAST_QUEUE(port) << UFFQ_SHIFT)
    | (TC956X_ONE << UFFQE_SHIFT);
  uiRegVal &= ~MFFQ_MASK;
  /* Multicast Failed packets -> RxQ for MAC2MAC Bridge */
  uiRegVal |= (MAC2MAC_UNICAST_QUEUE(port) << MFFQ_SHIFT)
    | (TC956X_ONE << MFFQE_SHIFT);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MAC_RXQ_CTRL4, uiRegVal);

  /* RxQ for PCI2MAC Bridge -> Dynamic DMA selection */
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP0);
  uiRegVal |= (TC956X_ONE << (DDMACH_SHIFT + TC956X_EIGHT * PCI2MAC_QUEUE(port)));
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP0, uiRegVal);

  /* RxQ for MAC2MAC Bridge -> RxCH2 */
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP0);
  uiRegVal &= ~(TC956X_ONE <<
    (DDMACH_SHIFT + TC956X_EIGHT * MAC2MAC_UNICAST_QUEUE(port)));
  uiRegVal |= (BRIDGE_RXCH(port) << (TC956X_EIGHT * MAC2MAC_UNICAST_QUEUE(port)));
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP0, uiRegVal);

  /* Multi/Broadcast RxQ -> Dynamic DMA selection */
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP1);
  uiRegVal |= (TC956X_ONE <<
    (DDMACH_SHIFT + TC956X_EIGHT * (MAC2MAC_BROAD_MULTI_QUEUE(port) - TC956X_FOUR)));
  Hw_Reg_Write32(uiBaseAddr, XGMAC_MTL_RXQ_DMA_MAP1, uiRegVal);

}

static void Init_mac2mac_filter (uint8_t port)
{
  uint32_t uiRegVal;
  uint32_t uiBaseAddr;

  uiBaseAddr = port ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

  /* Broadcast packets -> Duplicate Ch0/Ch2 */
  uiRegVal = XGMAC_BROADCAST_ADDHIGH;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_ADDRx_HIGH(XGMAC_BROADCAST_ADDROFF), uiRegVal);

  uiRegVal = XGMAC_BROADCAST_ADDLOW;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_ADDRx_LOW(XGMAC_BROADCAST_ADDROFF), uiRegVal);

  /* Read, modify, write XDCS setting for MAC address offset */

  while (uiRegVal & TC956X_ONE)
  {
    uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL);
  }

  uiRegVal = (XGMAC_BROADCAST_ADDROFF << TC956X_EIGHT) | (TC956X_ONE << TC956X_ONE) | (TC956X_ONE);

  Hw_Reg_Write32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL, uiRegVal);
  
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL);
  while (uiRegVal & TC956X_ONE)
  {
    uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL);
  }

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_DATA);

  uiRegVal |= (TC956X_ONE << BRIDGE_RXCH(port));
  Hw_Reg_Write32(uiBaseAddr, XGMAC_INDIR_ACCESS_DATA, uiRegVal);

  uiRegVal = (XGMAC_BROADCAST_ADDROFF << TC956X_EIGHT) | (TC956X_ONE);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL, uiRegVal);

  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL);
  while (uiRegVal & TC956X_ONE)
  {
    uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_INDIR_ACCESS_CTRL);
  }

  /* Enable Packet duplication */
  uiRegVal = Hw_Reg_Read32(uiBaseAddr, XGMAC_EXTENDED_REG);
  uiRegVal |= SET_DDS;
  Hw_Reg_Write32(uiBaseAddr, XGMAC_EXTENDED_REG, uiRegVal);

  /* Set RA mode */
  uiRegVal = (TC956X_ONE << TC956X_THIRTYONE);
  Hw_Reg_Write32(uiBaseAddr, XGMAC_PACKET_FILTER, uiRegVal);
}

static void Init_mac2mac_mtl_mac (void)
{
  uint8_t uiPort;

  for (uiPort=TC956X_ZERO; uiPort < NUM_PORT; uiPort++)
  {
    /* Init MTL queue routing */
    Init_mac2mac_queue_routing(uiPort);
    /* Init MAC filter */
    Init_mac2mac_filter(uiPort);
  }
}

void Init_mac2mac_param (void)
{
  uint32_t uiPort;
  struct bridge_dma_desc *txdesc, *rxdesc;
  uint32_t uiBaseAddr;

  for (uiPort=TC956X_ZERO; uiPort < NUM_PORT ; uiPort++)
  {
    txdesc = &mac_info[uiPort].txdesc;
    rxdesc = &mac_info[uiPort].rxdesc;

    uiBaseAddr = uiPort ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;

#ifdef MAC2MAC_DEBUG_INFO
    memset(gbridge_stats + uiPort, 0U, sizeof(struct mac2mac_sw_counter));
#endif
    /* Store EMAC(port) TxDMA descriptor information */
    /* Head pointer */
    txdesc->head
    = (eth_desc *)(Hw_Reg_Read32(uiBaseAddr
      , XGMAC_DMA_CH_TxDESC_LADDR(BRIDGE_TXCH(uiPort))));

    /* Tail pointer */
    txdesc->tail
    =  Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_TxDESC_TAIL_LPTR(BRIDGE_TXCH(uiPort)));

    /* Ring length */
    txdesc->ring_len
    = (Hw_Reg_Read32(uiBaseAddr , XGMAC_DMA_CH_TX_CONTROL2(BRIDGE_TXCH(uiPort)))
      & MAC2MAC_DESCLEN_MASK) + TC956X_ONE;

    txdesc->buf_size = TC956X_ZERO; /* Unused */
    /* Last write-backed offset -> will be updated */
    txdesc->offs = TC956X_ZERO;
    /* First write-backed offset -> will be updated */
    txdesc->dirty_offs = txdesc->offs;

    /* Store EMAC(port) RxDMA descriptor information */

    /* Head pointer */
    rxdesc->head
    = (eth_desc *)(Hw_Reg_Read32(uiBaseAddr
      , XGMAC_DMA_CH_RxDESC_LADDR(BRIDGE_RXCH(uiPort))));

    /* Tail pointer */
    rxdesc->tail
    = (Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_RxDESC_TAIL_LPTR(BRIDGE_RXCH(uiPort))));

    /* Ring length */
    rxdesc->ring_len
    = (Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_RX_CONTROL2(BRIDGE_RXCH(uiPort)))
      & MAC2MAC_DESCLEN_MASK) + TC956X_ONE;

    /* Buffer size */
    rxdesc->buf_size
    = ((Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_RX_Control(BRIDGE_RXCH(uiPort)))) >> TC956X_ONE)
      & MAC2MAC_DESCBUF_MASK;

    /* Last write-backed offset -> will be updated */
    rxdesc->offs = TC956X_ZERO;

    /* First write-backed offset -> will be updated */
    rxdesc->dirty_offs = rxdesc->offs;

    mac_info[uiPort].tx_wd = false;
    mac_info[uiPort].rx_wd = false;

  }
}
/* End of tc9563_bridge_desc_initialization */

static void Init_mac2mac_bridge (void)
{
  uint32_t uiData;
  uint64_t buff_head_addr[NUM_PORT];

  /* Get Tx/Rx buffer address */
  uiData = Hw_Reg_Read32(XGMAC_PCI_OFFSET, ATR_AXI4_SLV0_TABLE0_1); // SRC_ADDR[63:32]
  buff_head_addr[ETH_PORT0]
    = *(uint64_t *)RXDESC_BASE_ADDR0 + ((uint64_t)uiData << TC956X_THIRTYTWO);
  buff_head_addr[ETH_PORT1]
    = *(uint64_t *)RXDESC_BASE_ADDR1 + ((uint64_t)uiData << TC956X_THIRTYTWO);

  mac_info[ETH_PORT0].txdesc.buf_head = buff_head_addr[ETH_PORT0];
  mac_info[ETH_PORT0].rxdesc.buf_head = mac_info[ETH_PORT0].txdesc.buf_head;
  mac_info[ETH_PORT1].txdesc.buf_head = buff_head_addr[ETH_PORT1];
  mac_info[ETH_PORT1].rxdesc.buf_head = mac_info[ETH_PORT1].txdesc.buf_head;
  MAC2MAC_DEBUG_PRINT("Eth0 Buffer Head address: 0x%llx\n"
    , mac_info[ETH_PORT0].rxdesc.buf_head);
  MAC2MAC_DEBUG_PRINT("Eth1 Buffer Head address: 0x%llx\n"
    , mac_info[ETH_PORT1].rxdesc.buf_head);

  /* Stop Tx/RxDMA */
  Stop_port0_to_port1_dma();
  Stop_port1_to_port0_dma();
  MAC2MAC_DEBUG_PRINT("Stop Tx/RxDMA\n");

  // Disable TSEALL
  uiData = Hw_Reg_Read32(XGMAC_MAC_OFFSET0, XGMAC_Timestamp_Control);
  uiData &= ~(TC956X_ONE << TC956X_EIGHT);
  Hw_Reg_Write32(XGMAC_MAC_OFFSET0, XGMAC_Timestamp_Control, uiData);
  uiData = Hw_Reg_Read32(XGMAC_MAC_OFFSET1, XGMAC_Timestamp_Control);
  uiData &= ~(TC956X_ONE << TC956X_EIGHT);
  Hw_Reg_Write32(XGMAC_MAC_OFFSET1, XGMAC_Timestamp_Control, uiData);

  /* Init Tx/Rx Descriptor */
  Init_mac2mac_txdesc();
  Init_mac2mac_rxdesc();
  MAC2MAC_DEBUG_PRINT("Init Tx/Rx Descriptor\n");

  /* Init DMA */
  Init_mac2mac_dma();
  MAC2MAC_DEBUG_PRINT("Init DMA\n");

  /* Init MTL Queue routing & MAC filter */
  Init_mac2mac_mtl_mac();
  MAC2MAC_DEBUG_PRINT("Init MTL/MAC\n");

  /* Init parameters in firmware */
  Init_mac2mac_param();
  MAC2MAC_DEBUG_PRINT("Init parameters\n");

  /* Disable Tx/ Enable RX DMA interrupt for MCU */
  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS);
  uiData |= (TC956X_ONE << (TC956X_SIXTEEN+ETH0_BRIDGE_TxCH));
  uiData &= ~(TC956X_ONE << (TC956X_TWENTYFOUR+ETH0_BRIDGE_RxCH));
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS, uiData);

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS);
  uiData |= (TC956X_ONE << (TC956X_SIXTEEN+ETH1_BRIDGE_TxCH));
  uiData &= ~(TC956X_ONE << (TC956X_TWENTYFOUR+ETH1_BRIDGE_RxCH));
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS, uiData);
  MAC2MAC_DEBUG_PRINT("Enable Tx/RxDMA interrupt\n");

  /* Start Tx/RxDMA */
  Start_port0_to_port1_dma();
  Start_port1_to_port0_dma();
  MAC2MAC_DEBUG_PRINT("Start DMA\n");

  /* Enable Watchdog interrpt */
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTINTWDEXP_OFFS, MAC2MAC_WDExpire);

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS);
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS
    , (uiData | (WDCTL_WDEnable_MASK) | (WDCTL_WDRestart_MASK)));

}

void tc956x_bridge_watchdog_timer (void)
{
  uint32_t uiStatus,uiBaseAddr;
  uint8_t uiPort;
  for (uiPort=TC956X_ZERO; uiPort < NUM_PORT; uiPort++)
  {
    uiBaseAddr = uiPort ? XGMAC_MAC_OFFSET1 : XGMAC_MAC_OFFSET0;
    /* Check DMA interrupt status */
    uiStatus = Hw_Reg_Read32(uiBaseAddr, XGMAC_DMA_CH_STATUS(BRIDGE_RXCH(uiPort)));

    /*Check Set Receive Buffer Unavailable bit */
    if (uiStatus & EMAC_DMA_CH_STS_RBU)
    {
      mac_info[uiPort].rx_wd = true;
      /* Perform bridge transfer process */
      tc9563_BridgeTrasfer(uiPort, BRIDGE_RXCH(uiPort), BRIDGE_TXCH(TC956X_ONE-uiPort));
      mac_info[uiPort].rx_wd = false;
#ifdef MAC2MAC_DEBUG_INFO
      gbridge_stats[uiPort].rx_cpu_wd_irq++;
#endif
    }
  }
}

void MCUFLG_IRQHandler (void)
{
  uint32_t uiData;
  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MCUFLG_OFFS);
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, MCUFLG_OFFS, uiData);

  mac2mac_save_mcu_flag |= (uiData & MASK_32BIT);

  if ((mac2mac_save_mcu_flag & MAC2MAC_DRIVER_ETHX_EXIT)
    == MAC2MAC_DRIVER_ETHX_EXIT) {

      /* Stop Tx/RxDMA */
      Stop_port0_to_port1_dma();
      Stop_port1_to_port0_dma();

      /* Mask all interrupts for MCU */
      Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS, INT_MCUMASK0_MASK_ALL);
      Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS, INT_MCUMASK1_MASK_ALL);
      Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK2_OFFS, INT_MCUMASK2_MASK_ALL);
      Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS, INTINTWDCTL_DISABLE);

      mac2mac_save_mcu_flag &= ~MAC2MAC_DRIVER_ETHX_EXIT;

    }

  if ((mac2mac_save_mcu_flag & MAC2MAC_DRIVER_INIT_DONE)
    == MAC2MAC_DRIVER_INIT_DONE)
  {
    MAC2MAC_DEBUG_PRINT("\n--> Enter MCU Interrupt: Init MAC2MAC Bridge (mcu_flag=0x%x)\n"
      , mac2mac_save_mcu_flag);
    /* Init for MAC2MAC Bridge */
    Init_mac2mac_bridge();
    mac2mac_save_mcu_flag &= ~MAC2MAC_DRIVER_INIT_DONE;
    MAC2MAC_DEBUG_PRINT("<-- Exit MCU Interrupt: Done to Init MAC2MAC Bridge\n");
  }
  if (mac2mac_save_mcu_flag & MAC2MAC_CLEAR_SW_COUNTER)
  {
#ifdef MAC2MAC_DEBUG_INFO
    memset(gbridge_stats + ETH_PORT0, 0U, sizeof(struct mac2mac_sw_counter));
    memset(gbridge_stats + ETH_PORT1, 0U, sizeof(struct mac2mac_sw_counter));
#endif
    mac2mac_save_mcu_flag &= ~MAC2MAC_CLEAR_SW_COUNTER;
    MAC2MAC_DEBUG_PRINT("Cleared SW counters for MAC2MAC Bridge\n");
  }

    CM3_Errata_IRQ();
}
#endif /* ENABLE_MAC2MAC_BRIDGE */

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
  uint32_t uiData;

  *(uint32_t*)(TC956X_M3_DBG_CNT_START + (TC956X_ELEVEN * TC956X_FOUR)) += TC956X_ONE;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS);
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTINTWDCTL_OFFS,
                (uiData | (WDCTL_WDRestart_MASK)));

  /* Read the Watchdog-timer counter monitor */
  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTINTWDMON_OFFS);
  *(uint32_t*)(TC956X_M3_DBG_CNT_START + (TC956X_TWELVE * TC956X_FOUR)) = uiData;

#ifdef ENABLE_MAC2MAC_BRIDGE
  tc956x_bridge_watchdog_timer();
#endif
  CM3_Errata_IRQ();
}
/* end WDT_IRQHandler */

#if defined(TC956X_DMA_OFFLOAD_ENABLE) || defined(ENABLE_MAC2MAC_BRIDGE)
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
  uint32_t uiData, uiMask;
  uint32_t i;
  uint32_t uiCurDesc;
  uint32_t uiIntSts, uiIntEn;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC0STATUS);
  uiMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS);
  
  for (i = 0U; i < MAX_DMA_TX_CH; i++)
  {
    /* Skip the channels for which INTMCUMASK interrupt is masked*/
    if ((uiMask & (1U << (INTMCUMASK_TX_CH0 + i))))
    {
      continue;
    }

    if (uiData & (TC956X_ONE << i))
    { 
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET0, XGMAC_DMA_CUR_TxDESC_LADDR(i) );

      uiIntEn = Hw_Reg_Read32(XGMAC_MAC_OFFSET0, XGMAC_DMA_Int_Enable(i));
      uiIntSts = Hw_Reg_Read32(XGMAC_MAC_OFFSET0, XGMAC_DMA_CH_Status(i));
      uiIntSts = (uiIntEn & uiIntSts & XGMAC_DMA_TX_INT_STS_ALL);

      if (uiIntSts & XGMAC_DMA_STS_TX_TI)
      {
        *(uint32_t*)(*( uint32_t*)( SRAM_TX_PCIE_ADDR_LOC + ( i * TC956X_FOUR ) ) ) =  uiCurDesc;
      }

      Hw_Reg_Write32(XGMAC_MAC_OFFSET0, XGMAC_DMA_CH_Status(i), uiIntSts);
    }
  }

  CM3_Errata_IRQ();
}

void EMAC0_RXDMA_IRQHandler (void)
{
#ifndef ENABLE_MAC2MAC_BRIDGE
  uint32_t uiData, uiMask;
  uint32_t i;
  uint32_t uiCurDesc;
  uint32_t uiIntSts, uiIntEn;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC0STATUS);
  uiMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK0_OFFS);

  for (i = 0U; i < MAX_DMA_RX_CH; i++)
  {
    /* Skip the channels for which INTMCUMASK interrupt is masked*/
    if ((uiMask & (1U << (INTMCUMASK_RX_CH0 + i))))
    {
      continue;
    }

    if (uiData & (TC956X_ONE << (i + MACxRXSTS_CH0)))
    { 
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET0, XGMAC_DMA_CUR_RxDESC_LADDR(i) );

      uiIntEn = Hw_Reg_Read32(XGMAC_MAC_OFFSET0, XGMAC_DMA_Int_Enable(i));
      uiIntSts = Hw_Reg_Read32(XGMAC_MAC_OFFSET0, XGMAC_DMA_CH_Status(i));
      uiIntSts = (uiIntEn & uiIntSts & XGMAC_DMA_RX_INT_STS_ALL);

      if(uiIntSts & XGMAC_DMA_STS_RX_RI)
      {
        *(uint32_t*)(*( uint32_t*)( SRAM_RX_PCIE_ADDR_LOC + ( i * TC956X_FOUR ) ) ) =  uiCurDesc;
      }

      Hw_Reg_Write32(XGMAC_MAC_OFFSET0, XGMAC_DMA_CH_Status(i), uiIntSts);
    }
  }
#else
  uint32_t uiData;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC0STATUS);
  if (uiData & (TC956X_ONE << (BRIDGE_RXCH(ETH_PORT0)+MACxRXSTS_CH0)))
  {
     /* Get Rx length & Kick TxDMA */
    tc9563_BridgeTrasfer(ETH_PORT0, BRIDGE_RXCH(ETH_PORT0)
      , BRIDGE_TXCH(ETH_PORT1));
#ifdef MAC2MAC_DEBUG_INFO
    gbridge_stats[ETH_PORT0].rx_irq_n++;
#endif
  }
#endif

  CM3_Errata_IRQ();
}

void EMAC1_TXDMA_IRQHandler (void)
{
  uint32_t uiData, uiMask;
  uint32_t i;
  uint32_t uiCurDesc;
  uint32_t uiIntSts, uiIntEn;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC1STATUS);
  uiMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS);

  for (i = 0U; i < MAX_DMA_TX_CH; i++)
  {
    /* Skip the channels for which INTMCUMASK interrupt is masked*/
    if ((uiMask & (1U << (INTMCUMASK_TX_CH0 + i))))
    {
      continue;
    }

    if (uiData & (TC956X_ONE << i))
    {
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET1, XGMAC_DMA_CUR_TxDESC_LADDR(i) );

      uiIntEn = Hw_Reg_Read32(XGMAC_MAC_OFFSET1, XGMAC_DMA_Int_Enable(i));
      uiIntSts = Hw_Reg_Read32(XGMAC_MAC_OFFSET1, XGMAC_DMA_CH_Status(i));
      uiIntSts = (uiIntEn & uiIntSts & XGMAC_DMA_TX_INT_STS_ALL);

      if (uiIntSts & XGMAC_DMA_STS_TX_TI)
      {
        *(uint32_t*)(*(uint32_t*)(SRAM_TX_PCIE_ADDR_LOC + (MAX_DMA_TX_CH * TC956X_FOUR) + (i * TC956X_FOUR))) =  
                                    uiCurDesc;
      }

      Hw_Reg_Write32(XGMAC_MAC_OFFSET1, XGMAC_DMA_CH_Status(i), uiIntSts);
    }
  }

  CM3_Errata_IRQ();
}

void EMAC1_RXDMA_IRQHandler (void)
{
#ifndef ENABLE_MAC2MAC_BRIDGE
  uint32_t uiData, uiMask;
  uint32_t i;
  uint32_t uiCurDesc;
  uint32_t uiIntSts, uiIntEn;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC1STATUS);
  uiMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK1_OFFS);

  for (i = 0U; i < MAX_DMA_RX_CH; i++)
  {
    /* Skip the channels for which INTMCUMASK interrupt is masked*/
    if((uiMask & (1U << (INTMCUMASK_RX_CH0 + i))))
    {
      continue;
    }
    if (uiData & (TC956X_ONE << (i + MACxRXSTS_CH0)))
    {
      uiCurDesc = Hw_Reg_Read32( XGMAC_MAC_OFFSET1, XGMAC_DMA_CUR_RxDESC_LADDR(i) );

      uiIntEn = Hw_Reg_Read32(XGMAC_MAC_OFFSET1, XGMAC_DMA_Int_Enable(i));
      uiIntSts = Hw_Reg_Read32(XGMAC_MAC_OFFSET1, XGMAC_DMA_CH_Status(i));
      uiIntSts = (uiIntEn & uiIntSts & XGMAC_DMA_RX_INT_STS_ALL);

      if (uiIntSts & XGMAC_DMA_STS_RX_RI)
      {
        *(uint32_t *)(*(uint32_t *)(SRAM_RX_PCIE_ADDR_LOC + (MAX_DMA_RX_CH * TC956X_FOUR) + (i * TC956X_FOUR))) =  
                                    uiCurDesc;
      }

      Hw_Reg_Write32(XGMAC_MAC_OFFSET1, XGMAC_DMA_CH_Status(i), uiIntSts);
    }
  }
#else
  uint32_t uiData;

  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, MAC1STATUS);
  if (uiData & (TC956X_ONE << (BRIDGE_RXCH(ETH_PORT1)+MACxRXSTS_CH0)))
  {
    // Get Rx length & Kick TxDMA
    tc9563_BridgeTrasfer(ETH_PORT1, BRIDGE_RXCH(ETH_PORT1)
      , BRIDGE_TXCH(ETH_PORT0));

#ifdef MAC2MAC_DEBUG_INFO
    gbridge_stats[ETH_PORT1].rx_irq_n++;
#endif
  }
#endif

  CM3_Errata_IRQ();
}

#endif /* defined(TC956X_DMA_OFFLOAD_ENABLE) || defined(ENABLE_MAC2MAC_BRIDGE) */

#ifndef ENABLE_MAC2MAC_BRIDGE
/*
*  Function    :   PCIeFLR_IRQHandler(void)
*  Purpose     :   Interrupt handler for handling FLR interrupts
*  Inputs      :   None
*  Outputs     :   None
*  Return Value:   None
*  Limitations :   None
*/
void PCIeFLR_IRQHandler (void)
{
  uint32_t uiData, nrstctrl_offs;
  uint32_t i, mac_stat, regVal, allocated_dma;
  uint32_t uiIntMask, uiIntMask_reset;
  uint32_t reg_access = 0, mask_offset = 0, mac_status = 0;
  
  /* FLR interrupt mask */
  uiIntMask_reset = Hw_Reg_Read32(TC956X_INTC_REG_BASE, INTMCUMASK3_OFFS);
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK3_OFFS, 0);

  /* Identify the function ID */
  uiData = Hw_Reg_Read32(TC956X_INTC_REG_BASE, PCIEL12FLG);
  uiData &= 0x7073000U;

  switch(uiData) {
    case (TC956X_ONE << 12):
      reg_access = 0x000; /* PF0 */
      mac_status = MAC0STATUS;
      mask_offset = INTMCUMASK0_OFFS;
      nrstctrl_offs = TC956X_NRSTCTRL0_OFFS;
      break;
    case (TC956X_ONE << 13):
      reg_access = 0x200; /* PF1 */
      mac_status = MAC1STATUS;
      mask_offset = INTMCUMASK1_OFFS;
      nrstctrl_offs = TC956X_NRSTCTRL1_OFFS;
      break;
    case (TC956X_ONE << 16):
      reg_access = 0x001; /* VF0.1 */
      mac_status = MAC0STATUS;
      mask_offset = INTMCUMASK0_OFFS;
      break;
    case (TC956X_ONE << 17):
      reg_access = 0x002; /* VF0.2 */
      mac_status = MAC0STATUS;
      mask_offset = INTMCUMASK0_OFFS;
      break;
    case (TC956X_ONE << 18):
      reg_access = 0x003; /* VF0.3 */
      mac_status = MAC0STATUS;
      mask_offset = INTMCUMASK0_OFFS;
      break;
    case (TC956X_ONE << 24):
      reg_access = 0x201; /* VF1.1 */
      mac_status = MAC1STATUS;
      mask_offset = INTMCUMASK1_OFFS;
      break;
    case (TC956X_ONE << 25):
      reg_access = 0x202; /* VF1.2 */
      mac_status = MAC1STATUS;
      mask_offset = INTMCUMASK1_OFFS;
      break;
    case (TC956X_ONE << 26):
      reg_access = 0x203; /* VF1.3 */
      mac_status = MAC1STATUS;
      mask_offset = INTMCUMASK1_OFFS;
      break;
    default:
      reg_access = 0;
      mask_offset = 0;
      break;
  }

  /* Setting the GLUE register to access resource manager */
  Hw_Reg_Write32(TC956X_PCIE_GLUE_BASE, TC956X_PCIE_GLUE_EP_REG_ACCESS_CTRL, reg_access);

  if ((uiData & ((uint32_t)TC956X_ONE << 12)) || (uiData & ((uint32_t)TC956X_ONE << 13))) /* PF0 and PF1 */
  {
    /* Clear the Pending INT */
    mac_stat = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mac_status);

    allocated_dma = Hw_Reg_Read32(RSCMNG_RSC_BASE,RSCMNG_RSC_ST_REG);

    for (i = 0U; i < MAX_DMA_RX_CH; i++)
    {
      if ((allocated_dma & (TC956X_ONE << i)) != TC956X_ZERO) {    
        if ((mac_stat & ((uint32_t)TC956X_ONE << (i + MACxRXSTS_CH0))) != TC956X_ZERO)
        {
          /* Disable Interrupt generation for the RX channel */
          uiIntMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mask_offset);
          uiIntMask  |= ((uint32_t)TC956X_ONE << (INTMCUMASK_RX_CH0 + i));
          Hw_Reg_Write32(TC956X_INTC_REG_BASE, mask_offset, uiIntMask);
        }
        if ((mac_stat & (TC956X_ONE << i)) != TC956X_ZERO)
        {
          /* Disable Interrupt generation for the TX channel */
          uiIntMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mask_offset);
          uiIntMask  |= ((uint32_t)TC956X_ONE << (INTMCUMASK_TX_CH0 + i));
          Hw_Reg_Write32(TC956X_INTC_REG_BASE, mask_offset, uiIntMask);
        }
      }
    }
    
    /* Assert reset of EMAC */
    regVal = hw_reg_read32(TC956X_REG_BASE, nrstctrl_offs);
    Hw_Reg_Write32(TC956X_REG_BASE, nrstctrl_offs, regVal | (TC956X_EMAC_RST));

    mdelay(1);

    /* De-assert the reset of EMAC */
    regVal = hw_reg_read32(TC956X_REG_BASE, nrstctrl_offs);
    Hw_Reg_Write32(TC956X_REG_BASE, nrstctrl_offs, regVal & (~TC956X_EMAC_RST));

    /* Clear the resource manager control register */
    Hw_Reg_Write32(RSCMNG_RSC_BASE, RSCMNG_RSC_CTRL_REG_OFFSET, 0);
  } else {
    /* Find assigned DMA channel to reset */
    allocated_dma = Hw_Reg_Read32(RSCMNG_RSC_BASE,RSCMNG_RSC_ST_REG);

    /* Clear the Pending INT */
    mac_stat = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mac_status);

    for (i = 0U; i < MAX_DMA_RX_CH; i++)
    {
      if ((allocated_dma & (TC956X_ONE << i)) != TC956X_ZERO)
      {
        if ((mac_stat & ((uint32_t)TC956X_ONE << (i + MACxRXSTS_CH0))) != TC956X_ZERO)
        { 
          /* Disable Interrupt generation for the RX channel */
          uiIntMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mask_offset);
          uiIntMask  |= ((uint32_t)TC956X_ONE << (INTMCUMASK_RX_CH0 + i));
          Hw_Reg_Write32(TC956X_INTC_REG_BASE, mask_offset, uiIntMask);
        }
        if ((mac_stat & (TC956X_ONE << i)) != TC956X_ZERO)
        { 
          /* Disable Interrupt generation for the TX channel */
          uiIntMask = Hw_Reg_Read32(TC956X_INTC_REG_BASE, mask_offset);
          uiIntMask |= ((uint32_t)TC956X_ONE << (INTMCUMASK_TX_CH0 + i));
          Hw_Reg_Write32(TC956X_INTC_REG_BASE, mask_offset, uiIntMask);
        }
      }
    }
  }
  /* enabling flr intc */
  Hw_Reg_Write32(TC956X_INTC_REG_BASE, INTMCUMASK3_OFFS, uiIntMask_reset);

  Hw_Reg_Write32(TC956X_INTC_REG_BASE, PCIEL12FLG, uiData);

  CM3_Errata_IRQ();
}
#endif
