/*******************************************************************************

  Intel 10 Gigabit PCI Express Linux driver
  Copyright(c) 1999 - 2010 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/******************************************************************************
 Copyright (c)2006 - 2007 Myricom, Inc. for some LRO specific code
******************************************************************************/
#include <linux/types.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/pkt_sched.h>
#include <linux/ipv6.h>
#ifdef NETIF_F_TSO
#include <net/checksum.h>
#ifdef NETIF_F_TSO6
#include <net/ip6_checksum.h>
#endif
#endif
#ifdef SIOCETHTOOL
#include <linux/ethtool.h>
#endif
#ifdef NETIF_F_HW_VLAN_TX
#include <linux/if_vlan.h>
#endif


#include "ixgbe.h"

#include "ixgbe_sriov.h"

char ixgbe_driver_name[] = "ixgbe";
static const char ixgbe_driver_string[] =
	"Intel(R) 10 Gigabit PCI Express Network Driver";
#define DRV_HW_PERF

#ifndef CONFIG_IXGBE_NAPI
#define DRIVERNAPI
#else
#define DRIVERNAPI "-NAPI"
#endif

#define FPGA

#define DRV_VERSION "2.1.4" DRIVERNAPI DRV_HW_PERF FPGA
const char ixgbe_driver_version[] = DRV_VERSION;
static char ixgbe_copyright[] = "Copyright (c) 1999-2010 Intel Corporation.";
/* ixgbe_pci_tbl - PCI Device ID Table
 *
 * Wildcard entries (PCI_ANY_ID) should come last
 * Last entry must be all 0s
 *
 * { Vendor ID, Device ID, SubVendor ID, SubDevice ID,
 *   Class, Class Mask, private data (not used) }
 */
static struct pci_device_id ixgbe_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598_BX)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598AF_DUAL_PORT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598AF_SINGLE_PORT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598AT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598AT2)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598EB_CX4)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598_CX4_DUAL_PORT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598_DA_DUAL_PORT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598_SR_DUAL_PORT_EM)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598EB_XF_LR)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82598EB_SFP_LOM)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_KX4)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_XAUI_LOM)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_KR)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_SFP)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_SFP_EM)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_KX4_MEZZ)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_T3_LOM)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_CX4)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, IXGBE_DEV_ID_82599_COMBO_BACKPLANE)},
	/* required last entry */
	{0, }
};
MODULE_DEVICE_TABLE(pci, ixgbe_pci_tbl);

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
static int ixgbe_notify_dca(struct notifier_block *, unsigned long event,
                            void *p);
static struct notifier_block dca_notifier = {
	.notifier_call = ixgbe_notify_dca,
	.next          = NULL,
	.priority      = 0
};

#endif
MODULE_AUTHOR("Intel Corporation, <linux.nics@intel.com>");
MODULE_DESCRIPTION("Intel(R) 10 Gigabit PCI Express Network Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

#define DEFAULT_DEBUG_LEVEL_SHIFT 3

static inline void ixgbe_disable_sriov(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 gcr;
	u32 gpie;
	u32 vmdctl;

#ifdef CONFIG_PCI_IOV
	/* disable iov and allow time for transactions to clear */
	pci_disable_sriov(adapter->pdev);
#endif

	/* turn off device IOV mode */
	gcr = IXGBE_READ_REG(hw, IXGBE_GCR_EXT);
	gcr &= ~(IXGBE_GCR_EXT_SRIOV);
	IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT, gcr);
	gpie = IXGBE_READ_REG(hw, IXGBE_GPIE);
	gpie &= ~IXGBE_GPIE_VTMODE_MASK;
	IXGBE_WRITE_REG(hw, IXGBE_GPIE, gpie);

	/* set default pool back to 0 */
	vmdctl = IXGBE_READ_REG(hw, IXGBE_VT_CTL);
	vmdctl &= ~IXGBE_VT_CTL_POOL_MASK;
	IXGBE_WRITE_REG(hw, IXGBE_VT_CTL, vmdctl);

	/* take a breather then clean up driver data */
	msleep(100);
	if (adapter->vfinfo)
		kfree(adapter->vfinfo);
	adapter->vfinfo = NULL;

	adapter->num_vfs = 0;
	adapter->flags &= ~IXGBE_FLAG_SRIOV_ENABLED;
}

static void ixgbe_release_hw_control(struct ixgbe_adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware take over control of h/w */
	ctrl_ext = IXGBE_READ_REG(&adapter->hw, IXGBE_CTRL_EXT);
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_CTRL_EXT,
	                ctrl_ext & ~IXGBE_CTRL_EXT_DRV_LOAD);
}

static void ixgbe_get_hw_control(struct ixgbe_adapter *adapter)
{
	u32 ctrl_ext;

	/* Let firmware know the driver has taken over */
	ctrl_ext = IXGBE_READ_REG(&adapter->hw, IXGBE_CTRL_EXT);
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_CTRL_EXT,
	                ctrl_ext | IXGBE_CTRL_EXT_DRV_LOAD);
}

/*
 * ixgbe_set_ivar - set the IVAR registers, mapping interrupt causes to vectors
 * @adapter: pointer to adapter struct
 * @direction: 0 for Rx, 1 for Tx, -1 for other causes
 * @queue: queue to map the corresponding interrupt to
 * @msix_vector: the vector to map to the corresponding queue
 *
 */
static void ixgbe_set_ivar(struct ixgbe_adapter *adapter, s8 direction,
	                   u8 queue, u8 msix_vector)
{
	u32 ivar, index;
	struct ixgbe_hw *hw = &adapter->hw;
	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		msix_vector |= IXGBE_IVAR_ALLOC_VAL;
		if (direction == -1)
			direction = 0;
		index = (((direction * 64) + queue) >> 2) & 0x1F;
		ivar = IXGBE_READ_REG(hw, IXGBE_IVAR(index));
		ivar &= ~(0xFF << (8 * (queue & 0x3)));
		ivar |= (msix_vector << (8 * (queue & 0x3)));
		IXGBE_WRITE_REG(hw, IXGBE_IVAR(index), ivar);
		break;
	case ixgbe_mac_82599EB:
		if (direction == -1) {
			/* other causes */
			msix_vector |= IXGBE_IVAR_ALLOC_VAL;
			index = ((queue & 1) * 8);
			ivar = IXGBE_READ_REG(&adapter->hw, IXGBE_IVAR_MISC);
			ivar &= ~(0xFF << index);
			ivar |= (msix_vector << index);
			IXGBE_WRITE_REG(&adapter->hw, IXGBE_IVAR_MISC, ivar);
			break;
		} else {
			/* tx or rx causes */
			msix_vector |= IXGBE_IVAR_ALLOC_VAL;
			index = ((16 * (queue & 1)) + (8 * direction));
			ivar = IXGBE_READ_REG(hw, IXGBE_IVAR(queue >> 1));
			ivar &= ~(0xFF << index);
			ivar |= (msix_vector << index);
			IXGBE_WRITE_REG(hw, IXGBE_IVAR(queue >> 1), ivar);
			break;
		}
	default:
		break;
	}
}

static inline void ixgbe_irq_rearm_queues(struct ixgbe_adapter *adapter,
                                          u64 qmask)
{
	u32 mask;

	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB:
		mask = (IXGBE_EIMS_RTX_QUEUE & qmask);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EICS, mask);
		break;
	case ixgbe_mac_82599EB:
		mask = (qmask & 0xFFFFFFFF);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EICS_EX(0), mask);
		mask = (qmask >> 32);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EICS_EX(1), mask);
		break;
	default:
		break;
	}
}

void ixgbe_unmap_and_free_tx_resource(struct ixgbe_ring *tx_ring,
                                      struct ixgbe_tx_buffer *tx_buffer_info)
{
	if (tx_buffer_info->dma) {
		if (tx_buffer_info->mapped_as_page)
			dma_unmap_page(tx_ring->dev,
			               tx_buffer_info->dma,
			               tx_buffer_info->length,
			               DMA_TO_DEVICE);
		else
			dma_unmap_single(tx_ring->dev,
			                 tx_buffer_info->dma,
			                 tx_buffer_info->length,
			                 DMA_TO_DEVICE);
		tx_buffer_info->dma = 0;
	}
	if (tx_buffer_info->skb) {
		dev_kfree_skb_any(tx_buffer_info->skb);
		tx_buffer_info->skb = NULL;
	}
	tx_buffer_info->time_stamp = 0;
	/* tx_buffer_info must be completely set up in the transmit path */
}

/**
 * ixgbe_tx_xon_state - check the tx ring xon state
 * @adapter: the ixgbe adapter
 * @tx_ring: the corresponding tx_ring
 *
 * If not in DCB mode, checks TFCS.TXOFF, otherwise, find out the
 * corresponding TC of this tx_ring when checking TFCS.
 *
 * Returns : true if in xon state (currently not paused)
 */
static inline bool ixgbe_tx_xon_state(struct ixgbe_adapter *adapter,
                                      struct ixgbe_ring *tx_ring)
{
	u32 txoff = IXGBE_TFCS_TXOFF;

	if ((adapter->flags & IXGBE_FLAG_DCB_CAPABLE) &&
	     adapter->dcb_cfg.pfc_mode_enable) {
		int tc = 0;
		int dcb_i = adapter->ring_feature[RING_F_DCB].indices;
		u8 reg_idx = tx_ring->reg_idx;

		if (adapter->hw.mac.type == ixgbe_mac_82598EB) {
			tc = reg_idx >> 2;
			txoff = IXGBE_TFCS_TXOFF0;
		} else if (adapter->hw.mac.type == ixgbe_mac_82599EB) {
			tc = 0;
			txoff = IXGBE_TFCS_TXOFF;
			if (dcb_i == 8) {
				/* TC0, TC1 */
				tc = reg_idx >> 5;
				if (tc == 2) /* TC2, TC3 */
					tc += (reg_idx - 64) >> 4;
				else if (tc == 3) /* TC4, TC5, TC6, TC7 */
					tc += 1 + ((reg_idx - 96) >> 3);
			} else if (dcb_i == 4) {
				/* TC0, TC1 */
				tc = reg_idx >> 6;
				if (tc == 1) {
					tc += (reg_idx - 64) >> 5;
					if (tc == 2) /* TC2, TC3 */
						tc += (reg_idx - 96) >> 4;
				}
			}
		}
		txoff <<= tc;
	}
	return IXGBE_READ_REG(&adapter->hw, IXGBE_TFCS) & txoff;
}

static inline bool ixgbe_check_tx_hang(struct ixgbe_adapter *adapter,
                                       struct ixgbe_ring *tx_ring,
                                       unsigned int eop)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 head, tail;

	/* Detect a transmit hang in hardware, this serializes the
	 * check with the clearing of time_stamp and movement of eop */
	head = IXGBE_READ_REG(hw, IXGBE_TDH(tx_ring->reg_idx));
	tail = IXGBE_READ_REG(hw, IXGBE_TDT(tx_ring->reg_idx));
	clear_check_for_tx_hang(tx_ring);
	if ((head != tail) &&
	    tx_ring->tx_buffer_info[eop].time_stamp &&
	    time_after(jiffies, tx_ring->tx_buffer_info[eop].time_stamp + HZ) &&
	    ixgbe_tx_xon_state(adapter, tx_ring)) {
		/* detected Tx unit hang */
		union ixgbe_adv_tx_desc *tx_desc;
		tx_desc = IXGBE_TX_DESC_ADV(tx_ring, eop);
		DPRINTK(DRV, ERR, "Detected Tx Unit Hang\n"
			"  Tx Queue             <%d>\n"
			"  TDH, TDT             <%x>, <%x>\n"
			"  next_to_use          <%x>\n"
			"  next_to_clean        <%x>\n",
			tx_ring->queue_index, head, tail,
			tx_ring->next_to_use, eop);
		DPRINTK(DRV, ERR, "tx_buffer_info[next_to_clean]\n"
			"  time_stamp           <%lx>\n"
			"  jiffies              <%lx>\n",
			tx_ring->tx_buffer_info[eop].time_stamp, jiffies);

		netif_stop_subqueue(tx_ring->netdev, tx_ring->queue_index);

		return true;
	}

	return false;
}

#define IXGBE_MAX_TXD_PWR	14
#define IXGBE_MAX_DATA_PER_TXD	(1 << IXGBE_MAX_TXD_PWR)

/* Tx Descriptors needed, worst case */
#define TXD_USE_COUNT(S) (((S) >> IXGBE_MAX_TXD_PWR) + \
			 (((S) & (IXGBE_MAX_DATA_PER_TXD - 1)) ? 1 : 0))
#ifdef MAX_SKB_FRAGS
#define DESC_NEEDED ((MAX_SKB_FRAGS * TXD_USE_COUNT(PAGE_SIZE)) + 4)
#else
#define DESC_NEEDED 4
#endif

static void ixgbe_tx_timeout(struct net_device *netdev);

/**
 * ixgbe_clean_tx_irq - Reclaim resources after transmit completes
 * @q_vector: structure containing interrupt and ring information
 * @tx_ring: tx ring to clean
 **/
static bool ixgbe_clean_tx_irq(struct ixgbe_q_vector *q_vector,
                               struct ixgbe_ring *tx_ring)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	union ixgbe_adv_tx_desc *tx_desc, *eop_desc;
	struct ixgbe_tx_buffer *tx_buffer_info;
	unsigned int total_bytes = 0, total_packets = 0;
	u16 i, eop, count = 0;

	i = tx_ring->next_to_clean;
	eop = tx_ring->tx_buffer_info[i].next_to_watch;
	eop_desc = IXGBE_TX_DESC_ADV(tx_ring, eop);

	while ((eop_desc->wb.status & cpu_to_le32(IXGBE_TXD_STAT_DD)) &&
	       (count < tx_ring->work_limit)) {
		bool cleaned = false;
		rmb(); /* read buffer_info after eop_desc */
		for ( ; !cleaned; count++) {
			tx_desc = IXGBE_TX_DESC_ADV(tx_ring, i);
			tx_buffer_info = &tx_ring->tx_buffer_info[i];

			tx_desc->wb.status = 0;
			cleaned = (i == eop);

			i++;
			if (i == tx_ring->count)
				i = 0;

			if (cleaned && tx_buffer_info->skb) {
				total_bytes += tx_buffer_info->bytecount;
				total_packets += tx_buffer_info->gso_segs;
			}

			ixgbe_unmap_and_free_tx_resource(tx_ring,
			                                 tx_buffer_info);
		}

		eop = tx_ring->tx_buffer_info[i].next_to_watch;
		eop_desc = IXGBE_TX_DESC_ADV(tx_ring, eop);
	}

	tx_ring->next_to_clean = i;
	tx_ring->total_bytes += total_bytes;
	tx_ring->total_packets += total_packets;
	tx_ring->stats.packets += total_packets;
	tx_ring->stats.bytes += total_bytes;


	if (check_for_tx_hang(tx_ring) &&
	    ixgbe_check_tx_hang(adapter, tx_ring, i)) {
		/* schedule immediate reset if we believe we hung */
		DPRINTK(PROBE, INFO,
		        "tx hang %d detected, resetting adapter\n",
		        adapter->tx_timeout_count + 1);
		ixgbe_tx_timeout(tx_ring->netdev);

		/* the adapter is about to reset, no point in enabling stuff */
		return true;
	}

#define TX_WAKE_THRESHOLD (DESC_NEEDED * 2)
	if (unlikely(count && netif_carrier_ok(tx_ring->netdev) &&
	             (IXGBE_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD))) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();
#ifdef HAVE_TX_MQ
		if (__netif_subqueue_stopped(tx_ring->netdev, tx_ring->queue_index)
		    && !test_bit(__IXGBE_DOWN, &adapter->state)) {
			netif_wake_subqueue(tx_ring->netdev, tx_ring->queue_index);
			++tx_ring->tx_stats.restart_queue;
		}
#else
		if (netif_queue_stopped(tx_ring->netdev) &&
		    !test_bit(__IXGBE_DOWN, &adapter->state)) {
			netif_wake_queue(tx_ring->netdev);
			++tx_ring->tx_stats.restart_queue;
		}
#endif
	}

#ifndef CONFIG_IXGBE_NAPI
	/* re-arm the interrupt */
	if ((count >= tx_ring->work_limit) &&
	    (!test_bit(__IXGBE_DOWN, &adapter->state)))
		ixgbe_irq_rearm_queues(adapter, ((u64)1 << q_vector->v_idx));

#endif
	return (count < tx_ring->work_limit);
}

static void ixgbe_update_rx_dca(struct ixgbe_adapter *adapter,
                                struct ixgbe_ring *rx_ring,
                                int cpu)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rxctrl;
	u8 reg_idx = rx_ring->reg_idx;

	rxctrl = IXGBE_READ_REG(hw, IXGBE_DCA_RXCTRL(reg_idx));
	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		rxctrl &= ~IXGBE_DCA_RXCTRL_CPUID_MASK;
		rxctrl |= dca3_get_tag(&adapter->pdev->dev, cpu);
		break;
	case ixgbe_mac_82599EB:
		rxctrl &= ~IXGBE_DCA_RXCTRL_CPUID_MASK_82599;
		rxctrl |= (dca3_get_tag(&adapter->pdev->dev, cpu) <<
		                    IXGBE_DCA_RXCTRL_CPUID_SHIFT_82599);
		break;
	default:
		break;
	}
	rxctrl |= IXGBE_DCA_RXCTRL_DESC_DCA_EN;
	rxctrl |= IXGBE_DCA_RXCTRL_HEAD_DCA_EN;
	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED_DATA) {
		/* just do the header data when in Packet Split mode */
		if (ring_is_ps_enabled(rx_ring))
			rxctrl |= IXGBE_DCA_RXCTRL_HEAD_DCA_EN;
		else
			rxctrl |= IXGBE_DCA_RXCTRL_DATA_DCA_EN;
	}
	rxctrl &= ~(IXGBE_DCA_RXCTRL_DESC_RRO_EN);
	rxctrl &= ~(IXGBE_DCA_RXCTRL_DESC_WRO_EN |
	            IXGBE_DCA_RXCTRL_DESC_HSRO_EN);
	IXGBE_WRITE_REG(hw, IXGBE_DCA_RXCTRL(reg_idx), rxctrl);
}

static void ixgbe_update_tx_dca(struct ixgbe_adapter *adapter,
                                struct ixgbe_ring *tx_ring,
                                int cpu)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 txctrl;
	u8 reg_idx = tx_ring->reg_idx;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		txctrl = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL(reg_idx));
		txctrl &= ~IXGBE_DCA_TXCTRL_CPUID_MASK;
		txctrl |= dca3_get_tag(&adapter->pdev->dev, cpu);
		txctrl |= IXGBE_DCA_TXCTRL_DESC_DCA_EN;
		txctrl &= ~IXGBE_DCA_TXCTRL_TX_WB_RO_EN;
		IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL(reg_idx), txctrl);
		break;
	case ixgbe_mac_82599EB:
		txctrl = IXGBE_READ_REG(hw, IXGBE_DCA_TXCTRL_82599(reg_idx));
		txctrl &= ~IXGBE_DCA_TXCTRL_CPUID_MASK_82599;
		txctrl |= (dca3_get_tag(&adapter->pdev->dev, cpu) <<
		                    IXGBE_DCA_TXCTRL_CPUID_SHIFT_82599);
		txctrl |= IXGBE_DCA_TXCTRL_DESC_DCA_EN;
		txctrl &= ~IXGBE_DCA_TXCTRL_TX_WB_RO_EN;
		IXGBE_WRITE_REG(hw, IXGBE_DCA_TXCTRL_82599(reg_idx), txctrl);
		break;
	default:
		break;
	}
}

static void ixgbe_update_dca(struct ixgbe_q_vector *q_vector)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	int cpu = get_cpu();
	long r_idx;
	int i;

	if (q_vector->cpu == cpu)
		goto out_no_update;

	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	for (i = 0; i < q_vector->txr_count; i++) {
		ixgbe_update_tx_dca(adapter, adapter->tx_ring[r_idx], cpu);
		r_idx = find_next_bit(q_vector->txr_idx, adapter->num_tx_queues,
		                      r_idx + 1);
	}

	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	for (i = 0; i < q_vector->rxr_count; i++) {
		ixgbe_update_rx_dca(adapter, adapter->rx_ring[r_idx], cpu);
		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	q_vector->cpu = cpu;
out_no_update:
	put_cpu();
}

static void ixgbe_setup_dca(struct ixgbe_adapter *adapter)
{
	int num_q_vectors;
	int i;

	if (!(adapter->flags & IXGBE_FLAG_DCA_ENABLED))
		return;

	/* always use CB2 mode, difference is masked in the CB driver */
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_DCA_CTRL, 2);

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED)
		num_q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;
	else
		num_q_vectors = 1;

	for (i = 0; i < num_q_vectors; i++) {
		adapter->q_vector[i]->cpu = -1;
		ixgbe_update_dca(adapter->q_vector[i]);
	}
}

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
static int __ixgbe_notify_dca(struct device *dev, void *data)
{
	struct ixgbe_adapter *adapter = dev_get_drvdata(dev);
	unsigned long event = *(unsigned long *)data;

	if (!(adapter->flags & IXGBE_FLAG_DCA_ENABLED))
		return 0;

	switch (event) {
	case DCA_PROVIDER_ADD:
		/* if we're already enabled, don't do it again */
		if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
			break;
		if (dca_add_requester(dev) == 0) {
			adapter->flags |= IXGBE_FLAG_DCA_ENABLED;
			ixgbe_setup_dca(adapter);
			break;
		}
		/* Fall Through since DCA is disabled. */
	case DCA_PROVIDER_REMOVE:
		if (adapter->flags & IXGBE_FLAG_DCA_ENABLED) {
			dca_remove_requester(dev);
			adapter->flags &= ~IXGBE_FLAG_DCA_ENABLED;
			IXGBE_WRITE_REG(&adapter->hw, IXGBE_DCA_CTRL, 1);
		}
		break;
	}

	return 0;
}

#endif /* CONFIG_DCA or CONFIG_DCA_MODULE */
/**
 * ixgbe_receive_skb - Send a completed packet up the stack
 * @q_vector: structure containing interrupt and ring information
 * @skb: packet to send up
 * @vlan_tag: vlan tag for packet
 **/
static void ixgbe_receive_skb(struct ixgbe_q_vector *q_vector,
                              struct sk_buff *skb, u16 vlan_tag)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	int ret = NET_RX_SUCCESS;

#ifdef CONFIG_IXGBE_NAPI
		if (!(adapter->flags & IXGBE_FLAG_IN_NETPOLL)) {
#ifdef NETIF_F_HW_VLAN_TX
			if (vlan_tag & VLAN_VID_MASK) {
				if (adapter->vlgrp)
					vlan_gro_receive(&q_vector->napi,
							 adapter->vlgrp,
					                 vlan_tag, skb);
				else
					dev_kfree_skb_any(skb);
			} else {
				napi_gro_receive(&q_vector->napi, skb);
			}
#else
			napi_gro_receive(&q_vector->napi, skb);
#endif
		} else {
#endif /* CONFIG_IXGBE_NAPI */

#ifdef NETIF_F_HW_VLAN_TX
			if (vlan_tag & VLAN_VID_MASK) {
				if (adapter->vlgrp)
					ret = vlan_hwaccel_rx(skb,
					                      adapter->vlgrp,
					                      vlan_tag);
				else
					dev_kfree_skb_any(skb);
			} else {
				ret = netif_rx(skb);
			}
#else
			ret = netif_rx(skb);
#endif
#ifndef CONFIG_IXGBE_NAPI
			if (ret == NET_RX_DROP)
				adapter->rx_dropped_backlog++;
#endif
#ifdef CONFIG_IXGBE_NAPI
		}
#endif /* CONFIG_IXGBE_NAPI */
}

/**
 * ixgbe_rx_checksum - indicate in skb if hw indicated a good cksum
 * @adapter: address of board private structure
 * @rx_desc: current Rx descriptor being processed
 * @skb: skb currently being received and modified
 **/
static inline void ixgbe_rx_checksum(struct ixgbe_adapter *adapter,
                                     union ixgbe_adv_rx_desc *rx_desc,
                                     struct sk_buff *skb)
{
	u32 status_err = le32_to_cpu(rx_desc->wb.upper.status_error);
	skb->ip_summed = CHECKSUM_NONE;

	/* Rx csum disabled */
	if (!(adapter->flags & IXGBE_FLAG_RX_CSUM_ENABLED))
		return;

	/* if IP and error */
	if ((status_err & IXGBE_RXD_STAT_IPCS) &&
	    (status_err & IXGBE_RXDADV_ERR_IPE)) {
		adapter->hw_csum_rx_error++;
		return;
	}

	if (!(status_err & IXGBE_RXD_STAT_L4CS))
		return;

	if (status_err & IXGBE_RXDADV_ERR_TCPE) {
		u16 pkt_info = rx_desc->wb.lower.lo_dword.hs_rss.pkt_info;

		/*
		 * 82599 errata, UDP frames with a 0 checksum can be marked as
		 * checksum errors.
		 */
		if ((pkt_info & IXGBE_RXDADV_PKTTYPE_UDP) &&
		    (adapter->hw.mac.type == ixgbe_mac_82599EB))
			return;

		adapter->hw_csum_rx_error++;
		return;
	}

	/* It must be a TCP or UDP packet with a valid checksum */
	skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static inline void ixgbe_release_rx_desc(struct ixgbe_ring *rx_ring, u32 val)
{
	/*
	 * Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64).
	 */
	wmb();
	writel(val, rx_ring->tail);
}

/**
 * ixgbe_alloc_rx_buffers - Replace used receive buffers; packet split
 * @rx_ring: ring to place buffers on
 * @cleaned_count: number of buffers to replace
 **/
void ixgbe_alloc_rx_buffers(struct ixgbe_ring *rx_ring, u16 cleaned_count)
{
	union ixgbe_adv_rx_desc *rx_desc;
	struct ixgbe_rx_buffer *bi;
	struct sk_buff *skb;
	u16 i = rx_ring->next_to_use;

	/* do nothing if no valid netdev defined */
	if (!rx_ring->netdev)
	    return;

	while (cleaned_count--) {
		rx_desc = IXGBE_RX_DESC_ADV(rx_ring, i);
		bi = &rx_ring->rx_buffer_info[i];
		skb = bi->skb;

		if (!skb) {
			skb = netdev_alloc_skb_ip_align(rx_ring->netdev,
			                                rx_ring->rx_buf_len);
			if (!skb) {
				rx_ring->rx_stats.alloc_rx_buff_failed++;
				goto no_buffers;
			}
			/* initialize queue mapping */
			skb_record_rx_queue(skb, rx_ring->queue_index);
			bi->skb = skb;
		}

		if (!bi->dma) {
			bi->dma = dma_map_single(rx_ring->dev,
						 skb->data,
			                         rx_ring->rx_buf_len,
			                         DMA_FROM_DEVICE);
			if (dma_mapping_error(rx_ring->dev, bi->dma)) {
				rx_ring->rx_stats.alloc_rx_buff_failed++;
				bi->dma = 0;
				goto no_buffers;
			}
		}

		if (ring_is_ps_enabled(rx_ring)) {
			if (!bi->page) {
				bi->page = netdev_alloc_page(rx_ring->netdev);
				if (!bi->page) {
					rx_ring->rx_stats.alloc_rx_page_failed++;
					goto no_buffers;
				}
			}

			if (!bi->page_dma) {
				/* use a half page if we're re-using */
				bi->page_offset ^= PAGE_SIZE / 2;
				bi->page_dma = dma_map_page(rx_ring->dev,
							    bi->page,
				                            bi->page_offset,
				                            PAGE_SIZE / 2,
				                            DMA_FROM_DEVICE);
				if (dma_mapping_error(rx_ring->dev,
						      bi->page_dma)) {
					rx_ring->rx_stats.alloc_rx_page_failed++;
					bi->page_dma = 0;
					goto no_buffers;
				}
			}

			/* Refresh the desc even if buffer_addrs didn't change
			 * because each write-back erases this info. */
			rx_desc->read.pkt_addr = cpu_to_le64(bi->page_dma);
			rx_desc->read.hdr_addr = cpu_to_le64(bi->dma);
		} else {
			rx_desc->read.pkt_addr = cpu_to_le64(bi->dma);
			rx_desc->read.hdr_addr = 0;
		}

		i++;
		if (i == rx_ring->count)
			i = 0;
	}

no_buffers:
	if (rx_ring->next_to_use != i) {
		rx_ring->next_to_use = i;
		ixgbe_release_rx_desc(rx_ring, i);
	}
}

static inline u16 ixgbe_get_hlen(union ixgbe_adv_rx_desc *rx_desc)
{
	/* HW will not DMA in data larger than the given buffer, even if it
	 * parses the (NFS, of course) header to be larger.  In that case, it
	 * fills the header buffer and spills the rest into the page.
	 */
	u16 hdr_info = le16_to_cpu(rx_desc->wb.lower.lo_dword.hs_rss.hdr_info);
	u16 hlen = (hdr_info &  IXGBE_RXDADV_HDRBUFLEN_MASK) >>
		    IXGBE_RXDADV_HDRBUFLEN_SHIFT;
	if (hlen > IXGBE_RX_HDR_SIZE)
		hlen = IXGBE_RX_HDR_SIZE;
	return hlen;
}
/**
 * ixgbe_transform_rsc_queue - change rsc queue into a full packet
 * @skb: pointer to the last skb in the rsc queue
 *
 * This function changes a queue full of hw rsc buffers into a completed
 * packet.  It uses the ->prev pointers to find the first packet and then
 * turns it into the frag list owner.
 **/
static inline struct sk_buff *ixgbe_transform_rsc_queue(struct sk_buff *skb)
{
	unsigned int frag_list_size = 0;
	unsigned int skb_cnt = 1;

	while (skb->prev) {
		struct sk_buff *prev = skb->prev;
		frag_list_size += skb->len;
		skb->prev = NULL;
		skb = prev;
		skb_cnt++;
	}

	skb_shinfo(skb)->frag_list = skb->next;
	skb->next = NULL;
	skb->len += frag_list_size;
	skb->data_len += frag_list_size;
	skb->truesize += frag_list_size;
	IXGBE_RSC_CB(skb)->skb_cnt = skb_cnt;

	return skb;
}

#ifndef IXGBE_NO_LRO
/**
 * ixgbe_can_lro - returns true if packet is TCP/IPV4 and LRO is enabled
 * @rx_ring: structure containing ring specific data
 * @rx_desc: pointer to the rx descriptor
 *
 **/
static inline bool ixgbe_can_lro(struct ixgbe_ring *rx_ring,
                                 union ixgbe_adv_rx_desc *rx_desc)
{
	u16 pkt_info = le16_to_cpu(rx_desc->wb.lower.lo_dword.hs_rss.pkt_info);

	return (ring_is_lro_enabled(rx_ring) &&
		!(rx_ring->netdev->flags & IFF_PROMISC) &&
	        (pkt_info & IXGBE_RXDADV_PKTTYPE_IPV4) &&
	        (pkt_info & IXGBE_RXDADV_PKTTYPE_TCP));
}

/**
 * ixgbe_lro_flush - Indicate packets to upper layer.
 *
 * Update IP and TCP header part of head skb if more than one
 * skb's chained and indicate packets to upper layer.
 **/
static void ixgbe_lro_flush(struct ixgbe_q_vector *q_vector,
                                 struct ixgbe_lro_desc *lrod)
{
	struct ixgbe_lro_list *lrolist = q_vector->lrolist;
	struct iphdr *iph;
	struct tcphdr *th;
	struct sk_buff *skb;
	u32 *ts_ptr;

	hlist_del(&lrod->lro_node);
	lrolist->active_cnt--;

	skb = lrod->skb;
	lrod->skb = NULL;

	if (lrod->append_cnt) {
		/* take the lro queue and convert to skb format */
		skb = ixgbe_transform_rsc_queue(skb);

		/* incorporate ip header and re-calculate checksum */
		iph = (struct iphdr *)skb->data;
		iph->tot_len = ntohs(skb->len);
		iph->check = 0;
		iph->check = ip_fast_csum((unsigned char *)iph, iph->ihl);

		/* incorporate the latest ack into the tcp header */
		th = (struct tcphdr *) ((char *)skb->data + sizeof(*iph));
		th->ack_seq = lrod->ack_seq;
		th->psh = lrod->psh;
		th->window = lrod->window;
		th->check = 0;

		/* incorporate latest timestamp into the tcp header */
		if (lrod->opt_bytes) {
			ts_ptr = (u32 *)(th + 1);
			ts_ptr[1] = htonl(lrod->tsval);
			ts_ptr[2] = lrod->tsecr;
		}
	}

#ifdef NETIF_F_TSO
	skb_shinfo(skb)->gso_size = lrod->mss;
#endif
	ixgbe_receive_skb(q_vector, skb, lrod->vlan_tag);
	lrolist->stats.flushed++;


	hlist_add_head(&lrod->lro_node, &lrolist->free);
}

static void ixgbe_lro_flush_all(struct ixgbe_q_vector *q_vector)
{
	struct ixgbe_lro_desc *lrod;
	struct hlist_node *node, *node2;
	struct ixgbe_lro_list *lrolist = q_vector->lrolist;

	hlist_for_each_entry_safe(lrod, node, node2, &lrolist->active, lro_node)
		ixgbe_lro_flush(q_vector, lrod);
}

/*
 * ixgbe_lro_header_ok - Main LRO function.
 **/
static u16 ixgbe_lro_header_ok(struct sk_buff *new_skb, struct iphdr *iph,
                               struct tcphdr *th)
{
	int opt_bytes, tcp_data_len;
	u32 *ts_ptr = NULL;

	/* If we see CE codepoint in IP header, packet is not mergeable */
	if (INET_ECN_is_ce(ipv4_get_dsfield(iph)))
		return -1;

	/* ensure there are no options */
	if ((iph->ihl << 2) != sizeof(*iph))
		return -1;

	/* .. and the packet is not fragmented */
	if (iph->frag_off & htons(IP_MF|IP_OFFSET))
		return -1;

	/* ensure no bits set besides ack or psh */
	if (th->fin || th->syn || th->rst ||
	    th->urg || th->ece || th->cwr || !th->ack)
		return -1;

	/* ensure that the checksum is valid */
	if (new_skb->ip_summed != CHECKSUM_UNNECESSARY)
		return -1;

	/*
	 * check for timestamps. Since the only option we handle are timestamps,
	 * we only have to handle the simple case of aligned timestamps
	 */

	opt_bytes = (th->doff << 2) - sizeof(*th);
	if (opt_bytes != 0) {
		ts_ptr = (u32 *)(th + 1);
		if ((opt_bytes != TCPOLEN_TSTAMP_ALIGNED) ||
			(*ts_ptr != ntohl((TCPOPT_NOP << 24) |
			(TCPOPT_NOP << 16) | (TCPOPT_TIMESTAMP << 8) |
			TCPOLEN_TIMESTAMP))) {
			return -1;
		}
	}

	tcp_data_len = ntohs(iph->tot_len) - (th->doff << 2) - sizeof(*iph);

	return tcp_data_len;
}

/**
 * ixgbe_lro_queue - if able, queue skb into lro chain
 * @q_vector: structure containing interrupt and ring information
 * @new_skb: pointer to current skb being checked
 * @tag: vlan tag for skb
 *
 * Checks whether the skb given is eligible for LRO and if that's
 * fine chains it to the existing lro_skb based on flowid. If an LRO for
 * the flow doesn't exist create one.
 **/
static struct sk_buff *ixgbe_lro_queue(struct ixgbe_q_vector *q_vector,
                                       struct sk_buff *new_skb,
				       u16 tag)
{
	struct sk_buff *lro_skb;
	struct ixgbe_lro_desc *lrod;
	struct hlist_node *node;
	struct skb_shared_info *new_skb_info = skb_shinfo(new_skb);
	struct ixgbe_lro_list *lrolist = q_vector->lrolist;
	struct iphdr *iph = (struct iphdr *)new_skb->data;
	struct tcphdr *th = (struct tcphdr *)(iph + 1);
	int tcp_data_len = ixgbe_lro_header_ok(new_skb, iph, th);
	u16  opt_bytes = (th->doff << 2) - sizeof(*th);
	u32 *ts_ptr = (opt_bytes ? (u32 *)(th + 1) : NULL);
	u32 seq = ntohl(th->seq);

	/*
	 * we have a packet that might be eligible for LRO,
	 * so see if it matches anything we might expect
	 */
	hlist_for_each_entry(lrod, node, &lrolist->active, lro_node) {
		if (lrod->source_port != th->source ||
			lrod->dest_port != th->dest ||
			lrod->source_ip != iph->saddr ||
			lrod->dest_ip != iph->daddr ||
			lrod->vlan_tag != tag)
			continue;

		/* malformed header, no tcp data, resultant packet would be too large */
		if (tcp_data_len <= 0 || (tcp_data_len + lrod->len) > 65535) {
			ixgbe_lro_flush(q_vector, lrod);
			break;
		}

		/* out of order packet */
		if (seq != lrod->next_seq) {
			ixgbe_lro_flush(q_vector, lrod);
			tcp_data_len = -1;
			break;
		}

		/* packet without timestamp, or timestamp suddenly added to flow */
		if (lrod->opt_bytes != opt_bytes) {
			ixgbe_lro_flush(q_vector, lrod);
			break;
		}

		if (opt_bytes) {
			u32 tsval = ntohl(*(ts_ptr + 1));
			/* make sure timestamp values are increasing */
			if (opt_bytes != lrod->opt_bytes ||
			    lrod->tsval > tsval || *(ts_ptr + 2) == 0) {
				ixgbe_lro_flush(q_vector, lrod);
				tcp_data_len = -1;
				break;
			}

			lrod->tsval = tsval;
			lrod->tsecr = *(ts_ptr + 2);
		}

		/* remove any padding from the end of the skb */
		__pskb_trim(new_skb, ntohs(iph->tot_len));
		/* Remove IP and TCP header*/
		skb_pull(new_skb, ntohs(iph->tot_len) - tcp_data_len);

		lrod->next_seq += tcp_data_len;
		lrod->ack_seq = th->ack_seq;
		lrod->window = th->window;
		lrod->len += tcp_data_len;
		lrod->psh |= th->psh;
		lrod->append_cnt++;
		lrolist->stats.coal++;

		if (tcp_data_len > lrod->mss)
			lrod->mss = tcp_data_len;

		lro_skb = lrod->skb;

		/* if header is empty pull pages into current skb */
		if (!skb_headlen(new_skb) &&
		    ((skb_shinfo(lro_skb)->nr_frags +
		      skb_shinfo(new_skb)->nr_frags) <= MAX_SKB_FRAGS )) {
			struct skb_shared_info *lro_skb_info = skb_shinfo(lro_skb);

			/* copy frags into the last skb */
			memcpy(lro_skb_info->frags + lro_skb_info->nr_frags,
			       new_skb_info->frags,
			       new_skb_info->nr_frags * sizeof(skb_frag_t));

			lro_skb_info->nr_frags += new_skb_info->nr_frags;
			lro_skb->len += tcp_data_len;
			lro_skb->data_len += tcp_data_len;
			lro_skb->truesize += tcp_data_len;

			new_skb_info->nr_frags = 0;
			new_skb->truesize -= tcp_data_len;
			new_skb->len = new_skb->data_len = 0;
			new_skb->data = skb_mac_header(new_skb);
			__pskb_trim(new_skb, 0);
			new_skb->protocol = 0;
			lrolist->stats.recycled++;
		} else {
		/* Chain this new skb in frag_list */
			new_skb->prev = lro_skb;
			lro_skb->next = new_skb;
			lrod->skb = new_skb ;
			new_skb = NULL;
		}

		if (lrod->psh)
			ixgbe_lro_flush(q_vector, lrod);

		return new_skb;
	}

	/* start a new packet */
	if (tcp_data_len > 0 && !hlist_empty(&lrolist->free) && !th->psh) {
		lrod = hlist_entry(lrolist->free.first, struct ixgbe_lro_desc,
		                   lro_node);

		lrod->skb = new_skb;
		lrod->source_ip = iph->saddr;
		lrod->dest_ip = iph->daddr;
		lrod->source_port = th->source;
		lrod->dest_port = th->dest;
		lrod->vlan_tag = tag;
		lrod->len = new_skb->len;
		lrod->next_seq = seq + tcp_data_len;
		lrod->ack_seq = th->ack_seq;
		lrod->window = th->window;
		lrod->mss = tcp_data_len;
		lrod->opt_bytes = opt_bytes;
		lrod->psh = 0;
		lrod->append_cnt = 0;

		/* record timestamp if it is present */
		if (opt_bytes) {
			lrod->tsval = ntohl(*(ts_ptr + 1));
			lrod->tsecr = *(ts_ptr + 2);
		}
		/* remove first packet from freelist.. */
		hlist_del(&lrod->lro_node);
		/* .. and insert at the front of the active list */
		hlist_add_head(&lrod->lro_node, &lrolist->active);
		lrolist->active_cnt++;
		lrolist->stats.coal++;
		return NULL;
	}

	/* packet not handled by any of the above, pass it to the stack */
	ixgbe_receive_skb(q_vector, new_skb, tag);
	return NULL;
}

static void ixgbe_lro_ring_exit(struct ixgbe_lro_list *lrolist)
{
	struct hlist_node *node, *node2;
	struct ixgbe_lro_desc *lrod;

	hlist_for_each_entry_safe(lrod, node, node2, &lrolist->active,
	                          lro_node) {
		hlist_del(&lrod->lro_node);
		kfree(lrod);
	}

	hlist_for_each_entry_safe(lrod, node, node2, &lrolist->free,
	                          lro_node) {
		hlist_del(&lrod->lro_node);
		kfree(lrod);
	}
}

static void ixgbe_lro_ring_init(struct ixgbe_lro_list *lrolist)
{
	int j, bytes;
	struct ixgbe_lro_desc *lrod;

	bytes = sizeof(struct ixgbe_lro_desc);

	INIT_HLIST_HEAD(&lrolist->free);
	INIT_HLIST_HEAD(&lrolist->active);

	for (j = 0; j < IXGBE_LRO_MAX; j++) {
		lrod = kzalloc(bytes, GFP_KERNEL);
		if (lrod != NULL) {
			INIT_HLIST_NODE(&lrod->lro_node);
			hlist_add_head(&lrod->lro_node, &lrolist->free);
		}
	}
}

#endif /* IXGBE_NO_LRO */

static inline bool ixgbe_get_rsc_state(union ixgbe_adv_rx_desc *rx_desc)
{
	return !!(le32_to_cpu(rx_desc->wb.lower.lo_dword.data) &
	        IXGBE_RXDADV_RSCCNT_MASK);
}

static void ixgbe_rx_status_indication(u32 staterr,
                                       struct ixgbe_adapter *adapter)
{
	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82599EB:
		if (staterr & IXGBE_RXD_STAT_FLM)
			adapter->flm++;
#ifndef IXGBE_NO_LLI
		if (staterr & IXGBE_RXD_STAT_DYNINT)
			adapter->lli_int++;
#endif /* IXGBE_NO_LLI */
		break;
	case ixgbe_mac_82598EB:
#ifndef IXGBE_NO_LLI
		if (staterr & IXGBE_RXD_STAT_DYNINT)
			adapter->lli_int++;
#endif /* IXGBE_NO_LLI */
		break;
	default:
		break;
	}
}

#ifdef CONFIG_IXGBE_NAPI
static void ixgbe_clean_rx_irq(struct ixgbe_q_vector *q_vector,
                               struct ixgbe_ring *rx_ring,
                               int *work_done, int work_to_do)
#else
static bool ixgbe_clean_rx_irq(struct ixgbe_q_vector *q_vector,
                               struct ixgbe_ring *rx_ring)
#endif
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	union ixgbe_adv_rx_desc *rx_desc, *next_rxd;
	struct ixgbe_rx_buffer *rx_buffer_info, *next_buffer;
	struct sk_buff *skb;
	unsigned int total_rx_bytes = 0, total_rx_packets = 0;
	const int current_node = numa_node_id();
#ifdef IXGBE_FCOE
	int ddp_bytes = 0;
#endif /* IXGBE_FCOE */
	u32 staterr;
	u16 vlan_tag, i;
	u16  cleaned_count = 0;
#ifndef CONFIG_IXGBE_NAPI
	u16 work_to_do = rx_ring->work_limit, local_work_done = 0;
	u16 *work_done = &local_work_done;
#endif
	bool pkt_is_rsc = false;

	i = rx_ring->next_to_clean;
	rx_desc = IXGBE_RX_DESC_ADV(rx_ring, i);
	staterr = le32_to_cpu(rx_desc->wb.upper.status_error);

	while (staterr & IXGBE_RXD_STAT_DD) {
		u32 upper_len = 0;

		rx_buffer_info = &rx_ring->rx_buffer_info[i];

		skb = rx_buffer_info->skb;
		rx_buffer_info->skb = NULL;
		prefetch(skb->data - NET_IP_ALIGN);

		if (ring_is_rsc_enabled(rx_ring))
			pkt_is_rsc = ixgbe_get_rsc_state(rx_desc);

		/* if this is a skb from previous receive dma will be 0 */
		if (rx_buffer_info->dma) {
			u16 hlen;
			if (pkt_is_rsc && !(staterr & IXGBE_RXD_STAT_EOP) &&
			    !skb->prev) {
				/*
				 * When HWRSC is enabled, delay unmapping
				 * of the first packet. It carries the
				 * header information, HW may still
				 * access the header after the writeback.
				 * Only unmap it when EOP is reached
				 */
				IXGBE_RSC_CB(skb)->delay_unmap = true;
				IXGBE_RSC_CB(skb)->dma = rx_buffer_info->dma;
			} else {
				dma_unmap_single(rx_ring->dev,
						 rx_buffer_info->dma,
				                 rx_ring->rx_buf_len,
				                 DMA_FROM_DEVICE);
			}
			rx_buffer_info->dma = 0;

			if (ring_is_ps_enabled(rx_ring)) {
				hlen = ixgbe_get_hlen(rx_desc);
				upper_len = le16_to_cpu(rx_desc->wb.upper.length);
			} else {
				hlen = le16_to_cpu(rx_desc->wb.upper.length);
			}

			/* small packet padding for queue-to-queue loopback */
			if ((staterr & IXGBE_RXD_STAT_LB)
			    && hlen < 60 && upper_len == 0) {
				memset(skb->data + hlen, 0, 60 - hlen);
				hlen = 60;
			}

			skb_put(skb, hlen);
		} else {
			/* assume packet split since header is unmapped */
			upper_len = le16_to_cpu(rx_desc->wb.upper.length);
		}

		if (upper_len) {
			dma_unmap_page(rx_ring->dev,
				       rx_buffer_info->page_dma,
			               PAGE_SIZE / 2,
			               DMA_FROM_DEVICE);
			rx_buffer_info->page_dma = 0;
			skb_fill_page_desc(skb, skb_shinfo(skb)->nr_frags,
			                   rx_buffer_info->page,
			                   rx_buffer_info->page_offset,
			                   upper_len);

			if ((page_count(rx_buffer_info->page) == 1) &&
			    (page_to_nid(rx_buffer_info->page) == current_node))
				get_page(rx_buffer_info->page);
			else
				rx_buffer_info->page = NULL;

			skb->len += upper_len;
			skb->data_len += upper_len;
			skb->truesize += upper_len;
		}

		i++;
		if (i == rx_ring->count)
			i = 0;

		next_rxd = IXGBE_RX_DESC_ADV(rx_ring, i);
		prefetch(next_rxd);
		cleaned_count++;

		if (pkt_is_rsc) {
			u32 nextp = (staterr & IXGBE_RXDADV_NEXTP_MASK) >>
				     IXGBE_RXDADV_NEXTP_SHIFT;
			next_buffer = &rx_ring->rx_buffer_info[nextp];
		} else {
			next_buffer = &rx_ring->rx_buffer_info[i];
		}

		if (!(staterr & IXGBE_RXD_STAT_EOP)) {
			if (ring_is_ps_enabled(rx_ring)) {
				rx_buffer_info->skb = next_buffer->skb;
				rx_buffer_info->dma = next_buffer->dma;
				next_buffer->skb = skb;
				next_buffer->dma = 0;
			} else {
				skb->next = next_buffer->skb;
				skb->next->prev = skb;
			}
			rx_ring->rx_stats.non_eop_descs++;
			goto next_desc;
		}

		ixgbe_rx_status_indication(staterr, adapter);
		if (skb->prev) {
			skb = ixgbe_transform_rsc_queue(skb);
			/* if we got here without RSC the packet is invalid */
			if (!pkt_is_rsc) {
				__pskb_trim(skb, 0);
				rx_buffer_info->skb = skb;
				goto next_desc;
			}
		}

		if (ring_is_rsc_enabled(rx_ring)) {
			if (IXGBE_RSC_CB(skb)->delay_unmap) {
				dma_unmap_single(rx_ring->dev,
				                 IXGBE_RSC_CB(skb)->dma,
				                 rx_ring->rx_buf_len,
				                 DMA_FROM_DEVICE);
				IXGBE_RSC_CB(skb)->dma = 0;
				IXGBE_RSC_CB(skb)->delay_unmap = false;
			}
		}
		if (pkt_is_rsc) {
			if (ring_is_ps_enabled(rx_ring))
				rx_ring->rx_stats.rsc_count += skb_shinfo(skb)->nr_frags;
			else
				rx_ring->rx_stats.rsc_count += IXGBE_RSC_CB(skb)->skb_cnt;
			rx_ring->rx_stats.rsc_flush++;
		}

		/* ERR_MASK will only have valid bits if EOP set */
		if (unlikely(staterr & IXGBE_RXDADV_ERR_FRAME_ERR_MASK)) {
			/* trim packet back to size 0 and recycle it */
			__pskb_trim(skb, 0);
			rx_buffer_info->skb = skb;
			goto next_desc;
		}

		ixgbe_rx_checksum(adapter, rx_desc, skb);

		/* probably a little skewed due to removing CRC */
		total_rx_bytes += skb->len;
		total_rx_packets++;

		skb->protocol = eth_type_trans(skb, rx_ring->netdev);

#ifdef IXGBE_FCOE
		/* if ddp, not passing to ULD unless for FCP_RSP or error */
		if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED) {
			ddp_bytes = ixgbe_fcoe_ddp(adapter, rx_desc, skb);
			if (!ddp_bytes) {
				rx_ring->netdev->last_rx = jiffies;
				goto next_desc;
			}
		}

#endif /* IXGBE_FCOE */
		vlan_tag = ((staterr & IXGBE_RXD_STAT_VP) ?
		            le16_to_cpu(rx_desc->wb.upper.vlan) : 0);

#ifndef IXGBE_NO_LRO
		if (ixgbe_can_lro(rx_ring, rx_desc))
			rx_buffer_info->skb = ixgbe_lro_queue(q_vector, skb, vlan_tag);
		else
#endif
			ixgbe_receive_skb(q_vector, skb, vlan_tag);

		rx_ring->netdev->last_rx = jiffies;

next_desc:
		rx_desc->wb.upper.status_error = 0;

		(*work_done)++;
		if (*work_done >= work_to_do)
			break;

		/* return some buffers to hardware, one at a time is too slow */
		if (cleaned_count >= IXGBE_RX_BUFFER_WRITE) {
			ixgbe_alloc_rx_buffers(rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
		staterr = le32_to_cpu(rx_desc->wb.upper.status_error);
	}

#ifndef IXGBE_NO_LRO
	if (ring_is_lro_enabled(rx_ring))
		ixgbe_lro_flush_all(q_vector);

#endif /* IXGBE_NO_LRO */
	rx_ring->next_to_clean = i;
	cleaned_count = IXGBE_DESC_UNUSED(rx_ring);

	if (cleaned_count)
		ixgbe_alloc_rx_buffers(rx_ring, cleaned_count);

#ifdef IXGBE_FCOE
	/* include DDPed FCoE data */
	if (ddp_bytes > 0) {
		unsigned int mss;

		mss = rx_ring->netdev->mtu - sizeof(struct fcoe_hdr) -
			sizeof(struct fc_frame_header) -
			sizeof(struct fcoe_crc_eof);
		if (mss > 512)
			mss &= ~511;
		total_rx_bytes += ddp_bytes;
		total_rx_packets += DIV_ROUND_UP(ddp_bytes, mss);
	}
#endif /* IXGBE_FCOE */

	rx_ring->total_packets += total_rx_packets;
	rx_ring->total_bytes += total_rx_bytes;
	rx_ring->stats.packets += total_rx_packets;
	rx_ring->stats.bytes += total_rx_bytes;
#ifndef CONFIG_IXGBE_NAPI

	/* re-arm the interrupt if we had to bail early and have more work */
	if ((*work_done >= work_to_do) &&
	    (!test_bit(__IXGBE_DOWN, &adapter->state)))
		ixgbe_irq_rearm_queues(adapter, ((u64)1 << q_vector->v_idx));

	return local_work_done;
#endif
}

#ifdef CONFIG_IXGBE_NAPI
static int ixgbe_clean_rxonly(struct napi_struct *, int);
#endif
/**
 * ixgbe_configure_msix - Configure MSI-X hardware
 * @adapter: board private structure
 *
 * ixgbe_configure_msix sets up the hardware to properly generate MSI-X
 * interrupts.
 **/
static void ixgbe_configure_msix(struct ixgbe_adapter *adapter)
{
	struct ixgbe_q_vector *q_vector;
	int i, q_vectors, v_idx, r_idx;
	u32 mask;

	q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

	/*
	 * Populate the IVAR table and set the ITR values to the
	 * corresponding register.
	 */
	for (v_idx = 0; v_idx < q_vectors; v_idx++) {
		q_vector = adapter->q_vector[v_idx];
		/* XXX for_each_bit(...) */
		r_idx = find_first_bit(q_vector->rxr_idx,
		                       adapter->num_rx_queues);

		for (i = 0; i < q_vector->rxr_count; i++) {
			u8 reg_idx = adapter->rx_ring[r_idx]->reg_idx;
			ixgbe_set_ivar(adapter, 0, reg_idx, v_idx);
			r_idx = find_next_bit(q_vector->rxr_idx,
			                      adapter->num_rx_queues,
			                      r_idx + 1);
		}
		r_idx = find_first_bit(q_vector->txr_idx,
		                       adapter->num_tx_queues);

		for (i = 0; i < q_vector->txr_count; i++) {
			u8 reg_idx = adapter->tx_ring[r_idx]->reg_idx;
			ixgbe_set_ivar(adapter, 1, reg_idx, v_idx);
			r_idx = find_next_bit(q_vector->txr_idx,
			                      adapter->num_tx_queues,
			                      r_idx + 1);
		}

		if (q_vector->txr_count && !q_vector->rxr_count)
			/* tx only vector */
			q_vector->eitr = adapter->tx_eitr_param;
		else if (q_vector->rxr_count)
			/* rx or rx/tx vector */
			q_vector->eitr = adapter->rx_eitr_param;

		ixgbe_write_eitr(q_vector);
	}

	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB:
		ixgbe_set_ivar(adapter, -1, IXGBE_IVAR_OTHER_CAUSES_INDEX,
		               v_idx);
		break;
	case ixgbe_mac_82599EB:
		ixgbe_set_ivar(adapter, -1, 1, v_idx);
		break;

	default:
		break;
	}
	IXGBE_WRITE_REG(&adapter->hw, IXGBE_EITR(v_idx), 1950);
#ifdef IXGBE_TCP_TIMER
	ixgbe_set_ivar(adapter, -1, 0, ++v_idx);
#endif /* IXGBE_TCP_TIMER */

	/* set up to autoclear timer, and the vectors */
	mask = IXGBE_EIMS_ENABLE_MASK;
	mask &= ~(IXGBE_EIMS_OTHER |
		  IXGBE_EIMS_MAILBOX |
		  IXGBE_EIMS_LSC);

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIAC, mask);
}

enum latency_range {
	lowest_latency = 0,
	low_latency = 1,
	bulk_latency = 2,
	latency_invalid = 255
};

/**
 * ixgbe_update_itr - update the dynamic ITR value based on statistics
 * @adapter: pointer to adapter
 * @eitr: eitr setting (ints per sec) to give last timeslice
 * @itr_setting: current throttle rate in ints/second
 * @packets: the number of packets during this measurement interval
 * @bytes: the number of bytes during this measurement interval
 *
 *      Stores a new ITR value based on packets and byte
 *      counts during the last interrupt.  The advantage of per interrupt
 *      computation is faster updates and more accurate ITR for the current
 *      traffic pattern.  Constants in this function were computed
 *      based on theoretical maximum wire speed and thresholds were set based
 *      on testing data as well as attempting to minimize response time
 *      while increasing bulk throughput.
 *      this functionality is controlled by the InterruptThrottleRate module
 *      parameter (see ixgbe_param.c)
 **/
static u8 ixgbe_update_itr(struct ixgbe_adapter *adapter,
                           u32 eitr, u8 itr_setting,
                           int packets, int bytes)
{
	unsigned int retval = itr_setting;
	u32 timepassed_us;
	u64 bytes_perint;

	if (packets == 0)
		goto update_itr_done;


	/* simple throttlerate management
	 *    0-20MB/s lowest (100000 ints/s)
	 *   20-100MB/s low   (20000 ints/s)
	 *  100-1249MB/s bulk (8000 ints/s)
	 */
	/* what was last interrupt timeslice? */
	timepassed_us = 1000000/eitr;
	bytes_perint = bytes / timepassed_us; /* bytes/usec */

	switch (itr_setting) {
	case lowest_latency:
		if (bytes_perint > adapter->eitr_low) {
			retval = low_latency;
		}
		break;
	case low_latency:
		if (bytes_perint > adapter->eitr_high) {
			retval = bulk_latency;
		}
		else if (bytes_perint <= adapter->eitr_low) {
			retval = lowest_latency;
		}
		break;
	case bulk_latency:
		if (bytes_perint <= adapter->eitr_high) {
			retval = low_latency;
		}
		break;
	}

update_itr_done:
	return retval;
}

/**
 * ixgbe_write_eitr - write EITR register in hardware specific way
 * @q_vector: structure containing interrupt and ring information
 *
 * This function is made to be called by ethtool and by the driver
 * when it needs to update EITR registers at runtime.  Hardware
 * specific quirks/differences are taken care of here.
 */
void ixgbe_write_eitr(struct ixgbe_q_vector *q_vector)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	struct ixgbe_hw *hw = &adapter->hw;
	int v_idx = q_vector->v_idx;
	u32 itr_reg = EITR_INTS_PER_SEC_TO_REG(q_vector->eitr);

	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB:
		/* must write high and low 16 bits to reset counter */
		itr_reg |= (itr_reg << 16);
		break;
	case ixgbe_mac_82599EB:
		/*
		 * 82599 can support a value of zero, so allow it for
		 * max interrupt rate, but there is an errata where it can
		 * not be zero with RSC
		 */
		if (itr_reg == 8 &&
		    !(adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED))
			itr_reg = 0;

		/*
		 * set the WDIS bit to not clear the timer bits and cause an
		 * immediate assertion of the interrupt
		 */
		itr_reg |= IXGBE_EITR_CNT_WDIS;
		break;
	default:
		break;
	}
	IXGBE_WRITE_REG(hw, IXGBE_EITR(v_idx), itr_reg);
}

static void ixgbe_set_itr_msix(struct ixgbe_q_vector *q_vector)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	u32 new_itr;
	u8 current_itr, ret_itr;
	int i, r_idx;
	struct ixgbe_ring *rx_ring = NULL, *tx_ring = NULL;

	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	for (i = 0; i < q_vector->txr_count; i++) {
		tx_ring = adapter->tx_ring[r_idx];
		ret_itr = ixgbe_update_itr(adapter, q_vector->eitr,
		                           q_vector->tx_itr,
		                           tx_ring->total_packets,
		                           tx_ring->total_bytes);
		/* if the result for this queue would decrease interrupt
		 * rate for this vector then use that result */
		q_vector->tx_itr = ((q_vector->tx_itr > ret_itr) ?
		                    q_vector->tx_itr - 1 : ret_itr);
		r_idx = find_next_bit(q_vector->txr_idx, adapter->num_tx_queues,
		                      r_idx + 1);
	}

	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	for (i = 0; i < q_vector->rxr_count; i++) {
		rx_ring = adapter->rx_ring[r_idx];
		ret_itr = ixgbe_update_itr(adapter, q_vector->eitr,
		                           q_vector->rx_itr,
		                           rx_ring->total_packets,
		                           rx_ring->total_bytes);
		/* if the result for this queue would decrease interrupt
		 * rate for this vector then use that result */
		q_vector->rx_itr = ((q_vector->rx_itr > ret_itr) ?
		                    q_vector->rx_itr - 1 : ret_itr);
		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	current_itr = max(q_vector->rx_itr, q_vector->tx_itr);

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = 100000;
		break;
	case low_latency:
		new_itr = 20000; /* aka hwitr = ~200 */
		break;
	case bulk_latency:
	default:
		new_itr = 8000;
		break;
	}

	if (new_itr != q_vector->eitr) {
		/* do an exponential smoothing */
		new_itr = ((q_vector->eitr * 9) + new_itr)/10;

		/* save the algorithm value here, not the smoothed one */
		q_vector->eitr = new_itr;

		ixgbe_write_eitr(q_vector);
	}
}

/**
 * ixgbe_check_overtemp_task - worker thread to check over tempurature
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_check_overtemp_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             check_overtemp_task);
	struct ixgbe_hw *hw = &adapter->hw;
	u32 eicr = adapter->interrupt_event;

	if (test_bit(__IXGBE_DOWN, &adapter->state))
		return;

	if (!(adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE))
		return;

	switch (hw->device_id) {
	case IXGBE_DEV_ID_82599_T3_LOM: {
		u32 autoneg;
		bool link_up = false;

		if (hw->mac.ops.check_link)
			hw->mac.ops.check_link(hw, &autoneg, &link_up, false);

		/*
		 * Since the warning interrupt is for both ports
		 * we don't have to check if:
		 *  - This interrupt wasn't for our port.
		 *  - We may have missed the interrupt so always have to
		 *    check if we  got a LSC
		 */
		if (((eicr & IXGBE_EICR_GPI_SDP0) && link_up) &&
		    (!(eicr & IXGBE_EICR_LSC)))
			return;

		/* Check if this is not due to overtemp */
		if (hw->phy.ops.check_overtemp(hw) != IXGBE_ERR_OVERTEMP)
			return;
		}
		break;
	default:
		if (!(eicr & IXGBE_EICR_GPI_SDP0))
			return;
		break;
	}
	DPRINTK(PROBE, CRIT, "Network adapter has been stopped because it has "
	        "over heated. Restart the computer. If the problem persists, "
	        "power off the system and replace the adapter\n");
	/* write to clear the interrupt */
	IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP0);
}

static void ixgbe_check_fan_failure(struct ixgbe_adapter *adapter, u32 eicr)
{
	struct ixgbe_hw *hw = &adapter->hw;

	if ((adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE) &&
	    (eicr & IXGBE_EICR_GPI_SDP1)) {
		DPRINTK(PROBE, CRIT, "Fan has stopped, replace the adapter\n");
		/* write to clear the interrupt */
		IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP1);
	}
}

static void ixgbe_check_sfp_event(struct ixgbe_adapter *adapter, u32 eicr)
{
	struct ixgbe_hw *hw = &adapter->hw;

	if (eicr & IXGBE_EICR_GPI_SDP2) {
		/* Clear the interrupt */
		IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP2);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			schedule_work(&adapter->sfp_config_module_task);
	}

	if (eicr & IXGBE_EICR_GPI_SDP1) {
		/* Clear the interrupt */
		IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_GPI_SDP1);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			schedule_work(&adapter->multispeed_fiber_task);
	}
}

static void ixgbe_check_lsc(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;

	adapter->lsc_int++;
	adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
		IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_EIMC_LSC);
		IXGBE_WRITE_FLUSH(hw);
		schedule_work(&adapter->watchdog_task);
	}
}

static irqreturn_t ixgbe_msix_lsc(int irq, void *data)
{
	struct net_device *netdev = data;
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	u32 eicr;

	/*
	 * Workaround for Silicon errata.  Use clear-by-write instead
	 * of clear-by-read.  Reading with EICS will return the
	 * interrupt causes without clearing, which later be done
	 * with the write to EICR.
	 */
	eicr = IXGBE_READ_REG(hw, IXGBE_EICS);
	IXGBE_WRITE_REG(hw, IXGBE_EICR, eicr);

	if (eicr & IXGBE_EICR_LSC)
		ixgbe_check_lsc(adapter);

	if (eicr & IXGBE_EICR_MAILBOX)
		ixgbe_msg_task(adapter);

	switch (hw->mac.type) {
	case ixgbe_mac_82599EB:
		if (eicr & IXGBE_EICR_ECC) {
			DPRINTK(LINK, INFO, "Received unrecoverable ECC Err, "
			                    "please reboot\n");
			IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_ECC);
		}
#ifdef HAVE_TX_MQ
		/* Handle Flow Director Full threshold interrupt */
		if (eicr & IXGBE_EICR_FLOW_DIR) {
			int i;
			IXGBE_WRITE_REG(hw, IXGBE_EICR, IXGBE_EICR_FLOW_DIR);
			/* Disable transmits before FDIR Re-initialization */
			netif_tx_stop_all_queues(netdev);
			for (i = 0; i < adapter->num_tx_queues; i++) {
				struct ixgbe_ring *tx_ring =
				                            adapter->tx_ring[i];
				if (test_and_clear_bit(__IXGBE_TX_FDIR_INIT_DONE,
				                       &tx_ring->state))
					schedule_work(&adapter->fdir_reinit_task);
			}
		}
#endif
		ixgbe_check_sfp_event(adapter, eicr);
		adapter->interrupt_event = eicr;
		if ((adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE) &&
		    ((eicr & IXGBE_EICR_GPI_SDP0) || (eicr & IXGBE_EICR_LSC))) {
			adapter->interrupt_event = eicr;
			schedule_work(&adapter->check_overtemp_task);
		}
		break;
	default:
		break;
	}

	ixgbe_check_fan_failure(adapter, eicr);

	/* re-enable the original interrupt state, no lsc, no queues */
	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		IXGBE_WRITE_REG(hw, IXGBE_EIMS, eicr &
		                ~(IXGBE_EIMS_LSC | IXGBE_EIMS_RTX_QUEUE));

	return IRQ_HANDLED;
}

#ifdef IXGBE_TCP_TIMER
static irqreturn_t ixgbe_msix_pba(int irq, void *data)
{
	struct net_device *netdev = data;
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	int i;

	u32 pba = readl(adapter->msix_addr + IXGBE_MSIXPBA);
	for (i = 0; i < MAX_MSIX_COUNT; i++) {
		if (pba & (1 << i))
			adapter->msix_handlers[i](irq, data, regs);
		else
			adapter->pba_zero[i]++;
	}

	adapter->msix_pba++;
	return IRQ_HANDLED;
}

static irqreturn_t ixgbe_msix_tcp_timer(int irq, void *data)
{
	struct net_device *netdev = data;
	struct ixgbe_adapter *adapter = netdev_priv(netdev);

	adapter->msix_tcp_timer++;

	return IRQ_HANDLED;
}

#endif /* IXGBE_TCP_TIMER */
void ixgbe_irq_enable_queues(struct ixgbe_adapter *adapter, u64 qmask)
{
	u32 mask;
	struct ixgbe_hw *hw = &adapter->hw;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		mask = (IXGBE_EIMS_RTX_QUEUE & qmask);
		IXGBE_WRITE_REG(hw, IXGBE_EIMS, mask);
		break;
	case ixgbe_mac_82599EB:
		mask = (qmask & 0xFFFFFFFF);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(0), mask);
		mask = (qmask >> 32);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMS_EX(1), mask);
		break;
	default:
		break;
	}
	/* skip the flush */
}

void ixgbe_irq_disable_queues(struct ixgbe_adapter *adapter, u64 qmask)
{
	u32 mask;
	struct ixgbe_hw *hw = &adapter->hw;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		mask = (IXGBE_EIMS_RTX_QUEUE & qmask);
		IXGBE_WRITE_REG(hw, IXGBE_EIMC, mask);
		break;
	case ixgbe_mac_82599EB:
		mask = (qmask & 0xFFFFFFFF);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(0), mask);
		mask = (qmask >> 32);
		if (mask)
			IXGBE_WRITE_REG(hw, IXGBE_EIMC_EX(1), mask);
		break;
	default:
		break;
	}
	/* skip the flush */
}

static irqreturn_t ixgbe_msix_clean_tx(int irq, void *data)
{
	struct ixgbe_q_vector *q_vector = data;
	struct ixgbe_adapter  *adapter = q_vector->adapter;
	struct ixgbe_ring     *tx_ring;
	int i, r_idx;
#ifndef CONFIG_IXGBE_NAPI
	bool tx_clean_complete = false;
#endif

	if (!q_vector->txr_count)
		return IRQ_HANDLED;

#ifndef CONFIG_IXGBE_NAPI
	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);
#endif
	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	for (i = 0; i < q_vector->txr_count; i++) {
		tx_ring = adapter->tx_ring[r_idx];
		tx_ring->total_bytes = 0;
		tx_ring->total_packets = 0;
#ifndef CONFIG_IXGBE_NAPI
		tx_clean_complete = ixgbe_clean_tx_irq(q_vector, tx_ring);
#endif
		r_idx = find_next_bit(q_vector->txr_idx, adapter->num_tx_queues,
		                      r_idx + 1);
	}

#ifdef CONFIG_IXGBE_NAPI
	/* EIAM disabled interrupts (on this vector) for us */
	napi_schedule(&q_vector->napi);
#else
	if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
		u64 eics = ((u64)1 << q_vector->v_idx);
		ixgbe_irq_enable_queues(adapter, eics);
		if (!tx_clean_complete)
			ixgbe_irq_rearm_queues(adapter, eics);
	}
#endif
	/*
	 * possibly later we can enable tx auto-adjustment if necessary
	 *
	if (adapter->itr_setting & 1)
		ixgbe_set_itr_msix(q_vector);
	 */

	return IRQ_HANDLED;
}

/**
 * ixgbe_msix_clean_rx - single unshared vector rx clean (all queues)
 * @irq: unused
 * @data: pointer to our q_vector struct for this interrupt vector
 **/
static irqreturn_t ixgbe_msix_clean_rx(int irq, void *data)
{
	struct ixgbe_q_vector *q_vector = data;
	struct ixgbe_adapter  *adapter = q_vector->adapter;
	struct ixgbe_ring  *rx_ring;
	int r_idx;
	int i;
#ifndef CONFIG_IXGBE_NAPI
	bool rx_clean_complete = false;
#endif

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);

	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	for (i = 0; i < q_vector->rxr_count; i++) {
		rx_ring = adapter->rx_ring[r_idx];
		rx_ring->total_bytes = 0;
		rx_ring->total_packets = 0;
#ifndef CONFIG_IXGBE_NAPI
		rx_clean_complete = ixgbe_clean_rx_irq(q_vector, rx_ring);

		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	if (adapter->rx_itr_setting & 1)
		ixgbe_set_itr_msix(q_vector);
	if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
		u64 eics = ((u64)1 << q_vector->v_idx);
		ixgbe_irq_enable_queues(adapter, eics);
		if (!rx_clean_complete)
			ixgbe_irq_rearm_queues(adapter, eics);
	}
#else
		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	if (!q_vector->rxr_count)
		return IRQ_HANDLED;

	/* EIAM disabled interrupts (on this vector) for us */
	napi_schedule(&q_vector->napi);
#endif

	return IRQ_HANDLED;
}

static irqreturn_t ixgbe_msix_clean_many(int irq, void *data)
{
	struct ixgbe_q_vector *q_vector = data;
	struct ixgbe_adapter  *adapter = q_vector->adapter;
	struct ixgbe_ring  *ring;
	int r_idx;
	int i;
#ifndef CONFIG_IXGBE_NAPI
	bool clean_complete = true;
#endif

	if (!q_vector->txr_count && !q_vector->rxr_count)
		return IRQ_HANDLED;

#ifndef CONFIG_IXGBE_NAPI
	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);
#endif
	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	for (i = 0; i < q_vector->txr_count; i++) {
		ring = adapter->tx_ring[r_idx];
		ring->total_bytes = 0;
		ring->total_packets = 0;
#ifndef CONFIG_IXGBE_NAPI
		clean_complete = ixgbe_clean_tx_irq(q_vector, ring);
#endif
		r_idx = find_next_bit(q_vector->txr_idx, adapter->num_tx_queues,
		                      r_idx + 1);
	}

	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	for (i = 0; i < q_vector->rxr_count; i++) {
		ring = adapter->rx_ring[r_idx];
		ring->total_bytes = 0;
		ring->total_packets = 0;
#ifndef CONFIG_IXGBE_NAPI
		clean_complete &= ixgbe_clean_rx_irq(q_vector, ring);

		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	if (adapter->rx_itr_setting & 1)
		ixgbe_set_itr_msix(q_vector);
	if (!test_bit(__IXGBE_DOWN, &adapter->state)) {
		u64 eics = ((u64)1 << q_vector->v_idx);
		ixgbe_irq_enable_queues(adapter, eics);
		if (!clean_complete)
			ixgbe_irq_rearm_queues(adapter, eics);
	}
#else
		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	/* EIAM disabled interrupts (on this vector) for us */
	napi_schedule(&q_vector->napi);
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_IXGBE_NAPI
/**
 * ixgbe_clean_rxonly - msix (aka one shot) rx clean routine
 * @napi: napi struct with our devices info in it
 * @budget: amount of work driver is allowed to do this pass, in packets
 *
 * This function is optimized for cleaning one queue only on a single
 * q_vector!!!
 **/
static int ixgbe_clean_rxonly(struct napi_struct *napi, int budget)
{
	struct ixgbe_q_vector *q_vector =
	                       container_of(napi, struct ixgbe_q_vector, napi);
	struct ixgbe_adapter *adapter = q_vector->adapter;
	struct ixgbe_ring *rx_ring = NULL;
	int work_done = 0;
	long r_idx;

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);

	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	rx_ring = adapter->rx_ring[r_idx];

	ixgbe_clean_rx_irq(q_vector, rx_ring, &work_done, budget);

#ifndef HAVE_NETDEV_NAPI_LIST
	if (!netif_running(adapter->netdev))
		work_done = 0;

#endif
	/* If all Rx work done, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		if (adapter->rx_itr_setting & 1)
			ixgbe_set_itr_msix(q_vector);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			ixgbe_irq_enable_queues(adapter,
			                        ((u64)1 << q_vector->v_idx));
	}

	return work_done;
}

/**
 * ixgbe_clean_rxtx_many - msix (aka one shot) rx clean routine
 * @napi: napi struct with our devices info in it
 * @budget: amount of work driver is allowed to do this pass, in packets
 *
 * This function will clean more than one rx queue associated with a
 * q_vector.
 **/
static int ixgbe_clean_rxtx_many(struct napi_struct *napi, int budget)
{
	struct ixgbe_q_vector *q_vector =
	                       container_of(napi, struct ixgbe_q_vector, napi);
	struct ixgbe_adapter *adapter = q_vector->adapter;
	struct ixgbe_ring *ring = NULL;
	long r_idx;
	int work_done = 0, total_work = 0, i;
	bool rx_clean_complete = true, tx_clean_complete = true;

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);

	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	for (i = 0; i < q_vector->txr_count; i++) {
		ring = adapter->tx_ring[r_idx];
		tx_clean_complete &= ixgbe_clean_tx_irq(q_vector, ring);
		r_idx = find_next_bit(q_vector->txr_idx, adapter->num_tx_queues,
		                      r_idx + 1);
	}

	/* attempt to distribute budget to each queue fairly, but don't allow
	 * the budget to go below 1 because we'll exit polling */
	budget /= (q_vector->rxr_count ?: 1);
	budget = max(budget, 1);
	r_idx = find_first_bit(q_vector->rxr_idx, adapter->num_rx_queues);
	for (i = 0; i < q_vector->rxr_count; i++) {
		work_done = 0;
		ring = adapter->rx_ring[r_idx];
		ixgbe_clean_rx_irq(q_vector, ring, &work_done, budget);
		total_work += work_done;
		rx_clean_complete &= (work_done < budget);
		r_idx = find_next_bit(q_vector->rxr_idx, adapter->num_rx_queues,
		                      r_idx + 1);
	}

	if (!tx_clean_complete || !rx_clean_complete)
		work_done = budget;

#ifndef HAVE_NETDEV_NAPI_LIST
	if (!netif_running(adapter->netdev))
		work_done = 0;

#endif
	/* If all Rx work done, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		if (adapter->rx_itr_setting & 1)
			ixgbe_set_itr_msix(q_vector);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			ixgbe_irq_enable_queues(adapter, ((u64)1 << q_vector->v_idx));
	}

	return work_done;
}

/**
 * ixgbe_clean_txonly - msix (aka one shot) tx clean routine
 * @napi: napi struct with our devices info in it
 * @budget: amount of work driver is allowed to do this pass, in packets
 *
 * This function is optimized for cleaning one queue only on a single
 * q_vector!!!
 **/
static int ixgbe_clean_txonly(struct napi_struct *napi, int budget)
{
	struct ixgbe_q_vector *q_vector =
	                       container_of(napi, struct ixgbe_q_vector, napi);
	struct ixgbe_adapter *adapter = q_vector->adapter;
	struct ixgbe_ring *tx_ring = NULL;
	long r_idx;
	int work_done = 0;

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);

	r_idx = find_first_bit(q_vector->txr_idx, adapter->num_tx_queues);
	tx_ring = adapter->tx_ring[r_idx];

	if (!ixgbe_clean_tx_irq(q_vector, tx_ring))
		work_done = budget;

#ifndef HAVE_NETDEV_NAPI_LIST
	if (!netif_running(adapter->netdev))
		work_done = 0;

#endif
	/* If all Tx work done, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		if (adapter->tx_itr_setting & 1)
			ixgbe_set_itr_msix(q_vector);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			ixgbe_irq_enable_queues(adapter, ((u64)1 << q_vector->v_idx));
	}

	return work_done;
}

#endif /* CONFIG_IXGBE_NAPI */
static inline void map_vector_to_rxq(struct ixgbe_adapter *a, int v_idx,
                                     int r_idx)
{
	struct ixgbe_q_vector *q_vector = a->q_vector[v_idx];
	struct ixgbe_ring *rx_ring = a->rx_ring[r_idx];

	set_bit(r_idx, q_vector->rxr_idx);
	q_vector->rxr_count++;
	rx_ring->q_vector = q_vector;
}

static inline void map_vector_to_txq(struct ixgbe_adapter *a, int v_idx,
                                     int t_idx)
{
	struct ixgbe_q_vector *q_vector = a->q_vector[v_idx];
	struct ixgbe_ring *tx_ring = a->tx_ring[t_idx];

	set_bit(t_idx, q_vector->txr_idx);
	q_vector->txr_count++;
	tx_ring->q_vector = q_vector;
}

/**
 * ixgbe_map_rings_to_vectors - Maps descriptor rings to vectors
 * @adapter: board private structure to initialize
 *
 * This function maps descriptor rings to the queue-specific vectors
 * we were allotted through the MSI-X enabling code.  Ideally, we'd have
 * one vector per ring/queue, but on a constrained vector budget, we
 * group the rings as "efficiently" as possible.  You would add new
 * mapping configurations in here.
 **/
static int ixgbe_map_rings_to_vectors(struct ixgbe_adapter *adapter)
{
	int q_vectors;
	int q_split;
	int v_start = 0;
	int rxr_idx = 0, txr_idx = 0;
	int rxr_remaining = adapter->num_rx_queues;
	int txr_remaining = adapter->num_tx_queues;
	int i, j;
	int rqpv, tqpv;
	int err = 0;

	/* No mapping required if MSI-X is disabled. */
	if (!(adapter->flags & IXGBE_FLAG_MSIX_ENABLED))
		goto out;

	q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

	/*
	 * The ideal configuration...
	 * We have enough vectors to map one per queue.
	 */
	if (q_vectors == adapter->num_rx_queues + adapter->num_tx_queues) {
		for (; rxr_idx < rxr_remaining; v_start++, rxr_idx++)
			map_vector_to_rxq(adapter, v_start, rxr_idx);

		for (; txr_idx < txr_remaining; v_start++, txr_idx++)
			map_vector_to_txq(adapter, v_start, txr_idx);
		goto out;
	}

	/*
	 * If we don't have enough vectors for a 1-to-1
	 * mapping, we'll have to group them so there are
	 * multiple queues per vector.
	 */
	/* Re-adjusting *qpv takes care of the remainder. */

	q_split = q_vectors;

	for (i = v_start; i < q_split; i++) {
		rqpv = DIV_ROUND_UP(rxr_remaining, q_split - i);
		for (j = 0; j < rqpv; j++) {
			map_vector_to_rxq(adapter, i, rxr_idx);
			rxr_idx++;
			rxr_remaining--;
		}
	}


	for (i = v_start; i < q_vectors; i++) {
		tqpv = DIV_ROUND_UP(txr_remaining, q_vectors - i);
		for (j = 0; j < tqpv; j++) {
			map_vector_to_txq(adapter, i, txr_idx);
			txr_idx++;
			txr_remaining--;
		}
	}

out:
	return err;
}

/**
 * ixgbe_request_msix_irqs - Initialize MSI-X interrupts
 * @adapter: board private structure
 *
 * ixgbe_request_msix_irqs allocates MSI-X vectors and requests
 * interrupts from the kernel.
 **/
static int ixgbe_request_msix_irqs(struct ixgbe_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	irqreturn_t (*handler)(int, void *);
	int i, vector, q_vectors, err;
	int ri = 0, ti = 0;

	/* Decrement for Other and TCP Timer vectors */
	q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

#define SET_HANDLER(_v) (((_v)->rxr_count && (_v)->txr_count)        \
					  ? &ixgbe_msix_clean_many : \
			  (_v)->rxr_count ? &ixgbe_msix_clean_rx   : \
			  (_v)->txr_count ? &ixgbe_msix_clean_tx   : \
			  NULL)
	for (vector = 0; vector < q_vectors; vector++) {
		struct ixgbe_q_vector *q_vector = adapter->q_vector[vector];
		handler = SET_HANDLER(q_vector);

		if (handler == &ixgbe_msix_clean_rx) {
			sprintf(q_vector->name, "%s-%s-%d",
			        netdev->name, "rx", ri++);
		} else if (handler == &ixgbe_msix_clean_tx) {
			sprintf(q_vector->name, "%s-%s-%d",
			        netdev->name, "tx", ti++);
		} else if (handler == &ixgbe_msix_clean_many) {
			sprintf(q_vector->name, "%s-%s-%d",
			        netdev->name, "TxRx", ri++);
			ti++;
		} else {
			/* skip this unused q_vector */
			continue;
		}
		err = request_irq(adapter->msix_entries[vector].vector,
		                  handler, 0, q_vector->name,
		                  q_vector);
		if (err) {
			DPRINTK(PROBE, ERR,
			        "request_irq failed for MSIX interrupt "
			        "Error: %d\n", err);
			goto free_queue_irqs;
		}
	}

	sprintf(adapter->lsc_int_name, "%s:lsc", netdev->name);
	err = request_irq(adapter->msix_entries[vector].vector,
	                  &ixgbe_msix_lsc, 0, adapter->lsc_int_name, netdev);
	if (err) {
		DPRINTK(PROBE, ERR,
		        "request_irq for msix_lsc failed: %d\n", err);
		goto free_queue_irqs;
	}

#ifdef IXGBE_TCP_TIMER
	vector++;
	sprintf(adapter->tcp_timer_name, "%s:timer", netdev->name);
	err = request_irq(adapter->msix_entries[vector].vector,
	                  &ixgbe_msix_tcp_timer, 0, adapter->tcp_timer_name,
	                  netdev);
	if (err) {
		DPRINTK(PROBE, ERR,
		        "request_irq for msix_tcp_timer failed: %d\n", err);
		/* Free "Other" interrupt */
		free_irq(adapter->msix_entries[--vector].vector, netdev);
		goto free_queue_irqs;
	}

#endif
	return 0;

free_queue_irqs:
	for (i = vector - 1; i >= 0; i--)
		free_irq(adapter->msix_entries[--vector].vector,
		         adapter->q_vector[i]);
	adapter->flags &= ~IXGBE_FLAG_MSIX_ENABLED;
	pci_disable_msix(adapter->pdev);
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;
	return err;
}

static void ixgbe_set_itr(struct ixgbe_adapter *adapter)
{
	struct ixgbe_q_vector *q_vector = adapter->q_vector[0];
	u8 current_itr;
	u32 new_itr = q_vector->eitr;
	struct ixgbe_ring *rx_ring = adapter->rx_ring[0];
	struct ixgbe_ring *tx_ring = adapter->tx_ring[0];

	q_vector->tx_itr = ixgbe_update_itr(adapter, new_itr,
	                                    q_vector->tx_itr,
	                                    tx_ring->total_packets,
	                                    tx_ring->total_bytes);
	q_vector->rx_itr = ixgbe_update_itr(adapter, new_itr,
	                                    q_vector->rx_itr,
	                                    rx_ring->total_packets,
	                                    rx_ring->total_bytes);

	current_itr = max(q_vector->rx_itr, q_vector->tx_itr);

	switch (current_itr) {
	/* counts and packets in update_itr are dependent on these numbers */
	case lowest_latency:
		new_itr = 100000;
		break;
	case low_latency:
		new_itr = 20000; /* aka hwitr = ~200 */
		break;
	case bulk_latency:
		new_itr = 8000;
		break;
	default:
		break;
	}

	if (new_itr != q_vector->eitr) {

		/* do an exponential smoothing */
		new_itr = ((q_vector->eitr * 90)/100) + ((new_itr * 10)/100);

		/* save the algorithm value here */
		q_vector->eitr = new_itr;

		ixgbe_write_eitr(q_vector);
	}

	return;
}

/**
 * ixgbe_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/
static inline void ixgbe_irq_enable(struct ixgbe_adapter *adapter, bool queues, bool flush)
{
	u32 mask;
	u64 qmask;

	mask = (IXGBE_EIMS_ENABLE_MASK & ~IXGBE_EIMS_RTX_QUEUE);
	qmask = ~0;

	/* don't reenable LSC while waiting for link */
	if (adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE)
		mask &= ~IXGBE_EIMS_LSC;
	if (adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE)
		mask |= IXGBE_EIMS_GPI_SDP0;
	if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE)
		mask |= IXGBE_EIMS_GPI_SDP1;
	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82599EB:
		mask |= IXGBE_EIMS_ECC;
		mask |= IXGBE_EIMS_GPI_SDP1;
		mask |= IXGBE_EIMS_GPI_SDP2;
		mask |= IXGBE_EIMS_MAILBOX;
		break;
	default:
		break;
	}
	if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE ||
	    adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE)
		mask |= IXGBE_EIMS_FLOW_DIR;

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMS, mask);
	if (queues)
		ixgbe_irq_enable_queues(adapter, qmask);
	if (flush)
		IXGBE_WRITE_FLUSH(&adapter->hw);

	if (adapter->num_vfs > 32) {
		u32 eitrsel = (1 << (adapter->num_vfs - 32)) - 1;
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EITRSEL, eitrsel);
	}
}

/**
 * ixgbe_intr - legacy mode Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/
static irqreturn_t ixgbe_intr(int irq, void *data)
{
	struct net_device *netdev = data;
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	struct ixgbe_q_vector *q_vector = adapter->q_vector[0];
	u32 eicr;

	/*
	 * Workaround of Silicon errata on 82598.  Mask the interrupt
	 * before the read of EICR.
	 */
	IXGBE_WRITE_REG(hw, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);

	/* for NAPI, using EIAM to auto-mask tx/rx interrupt bits on read
	 * therefore no explict interrupt disable is necessary */
	eicr = IXGBE_READ_REG(hw, IXGBE_EICR);
	if (!eicr) {
		/*
		 * shared interrupt alert!
		 * make sure interrupts are enabled because the read will
		 * have disabled interrupts due to EIAM
		 * finish the workaround of silicon errata on 82598.  Unmask
		 * the interrupt that we masked before the EICR read.
		 */
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			ixgbe_irq_enable(adapter, true, true);
		return IRQ_NONE;  /* Not our interrupt */
	}

	if (eicr & IXGBE_EICR_LSC)
		ixgbe_check_lsc(adapter);

	switch (hw->mac.type) {
	case ixgbe_mac_82599EB:
		if (eicr & IXGBE_EICR_ECC)
			DPRINTK(LINK, INFO, "Received unrecoverable ECC Err, "
			                    "please reboot\n");
		ixgbe_check_sfp_event(adapter, eicr);
		if ((adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE) &&
		    ((eicr & IXGBE_EICR_GPI_SDP0) || (eicr & IXGBE_EICR_LSC))) {
			adapter->interrupt_event = eicr;
			schedule_work(&adapter->check_overtemp_task);
		}
		break;
	default:
		break;
	}

	ixgbe_check_fan_failure(adapter, eicr);

#ifdef CONFIG_IXGBE_NAPI
	if (napi_schedule_prep(&(q_vector->napi))) {
		adapter->tx_ring[0]->total_packets = 0;
		adapter->tx_ring[0]->total_bytes = 0;
		adapter->rx_ring[0]->total_packets = 0;
		adapter->rx_ring[0]->total_bytes = 0;
		/* would disable interrupts here but EIAM disabled it */
		__napi_schedule(&(q_vector->napi));
	}

	/*
	 * re-enable link(maybe) and non-queue interrupts, no flush.
	 * ixgbe_poll will re-enable the queue interrupts
	 */
	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_enable(adapter, false, false);
#else
	adapter->tx_ring[0]->total_packets = 0;
	adapter->tx_ring[0]->total_bytes = 0;
	adapter->rx_ring[0]->total_packets = 0;
	adapter->rx_ring[0]->total_bytes = 0;
	ixgbe_clean_tx_irq(q_vector, adapter->tx_ring[0]);
	ixgbe_clean_rx_irq(q_vector, adapter->rx_ring[0]);

	/* dynamically adjust throttle */
	if (adapter->rx_itr_setting & 1)
		ixgbe_set_itr(adapter);

	/*
	 * Workaround of Silicon errata on 82598.  Unmask
	 * the interrupt that we masked before the EICR read
	 * no flush of the re-enable is necessary here
	 */
	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_enable(adapter, true, false);
#endif
	return IRQ_HANDLED;
}

static inline void ixgbe_reset_q_vectors(struct ixgbe_adapter *adapter)
{
	int i, q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

	for (i = 0; i < adapter->num_rx_queues; i++)
		adapter->rx_ring[i]->q_vector = NULL;
	for (i = 0; i < adapter->num_tx_queues; i++)
		adapter->tx_ring[i]->q_vector = NULL;

	for (i = 0; i < q_vectors; i++) {
		struct ixgbe_q_vector *q_vector = adapter->q_vector[i];
		bitmap_zero(q_vector->rxr_idx, MAX_RX_QUEUES);
		bitmap_zero(q_vector->txr_idx, MAX_TX_QUEUES);
		q_vector->rxr_count = 0;
		q_vector->txr_count = 0;
		q_vector->eitr = adapter->rx_eitr_param;
	}
}

/**
 * ixgbe_request_irq - initialize interrupts
 * @adapter: board private structure
 *
 * Attempts to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 **/
static int ixgbe_request_irq(struct ixgbe_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	int err;

#ifdef HAVE_DEVICE_NUMA_NODE
	DPRINTK(TX_ERR, INFO, "numa_node before request_irq %d\n",
	        dev_to_node(&adapter->pdev->dev));
#endif /* HAVE_DEVICE_NUMA_NODE */
	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		err = ixgbe_request_msix_irqs(adapter);
	} else if (adapter->flags & IXGBE_FLAG_MSI_ENABLED) {
		err = request_irq(adapter->pdev->irq, &ixgbe_intr, 0,
		                  netdev->name, netdev);
	} else {
		err = request_irq(adapter->pdev->irq, &ixgbe_intr, IRQF_SHARED,
		                  netdev->name, netdev);
	}

	if (err)
		DPRINTK(PROBE, ERR, "request_irq failed, Error %d\n", err);

	return err;
}

static void ixgbe_free_irq(struct ixgbe_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		int i, q_vectors;

		q_vectors = adapter->num_msix_vectors;
		i = q_vectors - 1;
#ifdef IXGBE_TCP_TIMER
		free_irq(adapter->msix_entries[i].vector, netdev);
		i--;
#endif
		free_irq(adapter->msix_entries[i].vector, netdev);
		i--;

		/* free only the irqs that were actually requested */
		for (; i >= 0; i--) {
			if (adapter->q_vector[i]->rxr_count ||
			    adapter->q_vector[i]->txr_count)
				free_irq(adapter->msix_entries[i].vector,
					 adapter->q_vector[i]);
		}

		ixgbe_reset_q_vectors(adapter);
	} else {
		free_irq(adapter->pdev->irq, netdev);
	}
}

/**
 * ixgbe_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static inline void ixgbe_irq_disable(struct ixgbe_adapter *adapter)
{
	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB:
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, ~0);
		break;
	case ixgbe_mac_82599EB:
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC, 0xFFFF0000);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(0), ~0);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_EIMC_EX(1), ~0);
		if (adapter->num_vfs > 32)
			IXGBE_WRITE_REG(&adapter->hw, IXGBE_EITRSEL, 0);
		break;
	default:
		break;
	}
	IXGBE_WRITE_FLUSH(&adapter->hw);
	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		int i;
		for (i = 0; i < adapter->num_msix_vectors; i++)
			synchronize_irq(adapter->msix_entries[i].vector);
	} else {
		synchronize_irq(adapter->pdev->irq);
	}
}

/**
 * ixgbe_configure_msi_and_legacy - Initialize PIN (INTA...) and MSI interrupts
 *
 **/
static void ixgbe_configure_msi_and_legacy(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;

	IXGBE_WRITE_REG(hw, IXGBE_EITR(0),
	                EITR_INTS_PER_SEC_TO_REG(adapter->rx_eitr_param));

	ixgbe_set_ivar(adapter, 0, 0, 0);
	ixgbe_set_ivar(adapter, 1, 0, 0);

	map_vector_to_rxq(adapter, 0, 0);
	map_vector_to_txq(adapter, 0, 0);

	DPRINTK(HW, INFO, "Legacy interrupt IVAR setup done\n");
}

/**
 * ixgbe_configure_tx_ring - Configure 8259x Tx ring after Reset
 * @adapter: board private structure
 * @ring: structure containing ring specific data
 *
 * Configure the Tx descriptor ring after a reset.
 **/
void ixgbe_configure_tx_ring(struct ixgbe_adapter *adapter,
			     struct ixgbe_ring *ring)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u64 tdba = ring->dma;
	int wait_loop = 10;
	u32 txdctl;
	u8 reg_idx = ring->reg_idx;

	/* disable queue to avoid issues while updating state */
	txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(reg_idx));
	IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(reg_idx),
			txdctl & ~IXGBE_TXDCTL_ENABLE);
	IXGBE_WRITE_FLUSH(hw);

	IXGBE_WRITE_REG(hw, IXGBE_TDBAL(reg_idx),
	                (tdba & DMA_BIT_MASK(32)));
	IXGBE_WRITE_REG(hw, IXGBE_TDBAH(reg_idx), (tdba >> 32));
	IXGBE_WRITE_REG(hw, IXGBE_TDLEN(reg_idx),
			ring->count * sizeof(union ixgbe_adv_tx_desc));
	IXGBE_WRITE_REG(hw, IXGBE_TDH(reg_idx), 0);
	IXGBE_WRITE_REG(hw, IXGBE_TDT(reg_idx), 0);
	ring->tail = hw->hw_addr + IXGBE_TDT(reg_idx);

	/* configure fetching thresholds */
	if (adapter->rx_itr_setting == 0) {
		/* cannot set wthresh when itr==0 */
		txdctl &= ~0x007F0000;
	} else {
		/* enable WTHRESH=8 descriptors, to encourage burst writeback */
		txdctl |= (8 << 16);
	}
	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
		/* PThresh workaround for Tx hang with DFP enabled. */
		txdctl |= 32;
	}

	/* reinitialize flowdirector state */
	set_bit(__IXGBE_TX_FDIR_INIT_DONE, &ring->state);

	/* enable queue */
	txdctl |= IXGBE_TXDCTL_ENABLE;
	IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(reg_idx), txdctl);

	/* TXDCTL.EN will return 0 on 82598 if link is down, so skip it */
	if (hw->mac.type == ixgbe_mac_82598EB &&
	    !(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
		return;

	/* poll to verify queue is enabled */
	do {
		msleep(1);
		txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(reg_idx));
	} while (--wait_loop && !(txdctl & IXGBE_TXDCTL_ENABLE));
	if (!wait_loop)
		DPRINTK(DRV, ERR, "Could not enable Tx Queue %d\n", reg_idx);
}

static void ixgbe_setup_mtqc(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rttdcs;
	u32 mask;

	if (hw->mac.type == ixgbe_mac_82598EB)
		return;

	/* disable the arbiter while setting MTQC */
	rttdcs = IXGBE_READ_REG(hw, IXGBE_RTTDCS);
	rttdcs |= IXGBE_RTTDCS_ARBDIS;
	IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);

	/* set transmit pool layout */
	mask = IXGBE_FLAG_SRIOV_ENABLED | IXGBE_FLAG_VMDQ_ENABLED;
	mask |= IXGBE_FLAG_DCB_ENABLED;
	switch (adapter->flags & mask) {

	case (IXGBE_FLAG_VMDQ_ENABLED):
	case (IXGBE_FLAG_SRIOV_ENABLED):
	case (IXGBE_FLAG_SRIOV_ENABLED | IXGBE_FLAG_VMDQ_ENABLED):
		IXGBE_WRITE_REG(hw, IXGBE_MTQC,
				(IXGBE_MTQC_VT_ENA | IXGBE_MTQC_64VF));
		break;
	case (IXGBE_FLAG_VMDQ_ENABLED | IXGBE_FLAG_DCB_ENABLED):
	case (IXGBE_FLAG_SRIOV_ENABLED | IXGBE_FLAG_DCB_ENABLED):
	case (IXGBE_FLAG_SRIOV_ENABLED | IXGBE_FLAG_VMDQ_ENABLED
				 | IXGBE_FLAG_DCB_ENABLED):
		IXGBE_WRITE_REG(hw, IXGBE_MTQC,
				(IXGBE_MTQC_RT_ENA
					| IXGBE_MTQC_VT_ENA
					| IXGBE_MTQC_4TC_4TQ));
		break;

	case (IXGBE_FLAG_DCB_ENABLED):
		IXGBE_WRITE_REG(hw, IXGBE_MTQC,
				IXGBE_MTQC_RT_ENA | IXGBE_MTQC_8TC_8TQ);
		break;

	default:
		IXGBE_WRITE_REG(hw, IXGBE_MTQC, IXGBE_MTQC_64Q_1PB);
		break;
	}

	/* re-enable the arbiter */
	rttdcs &= ~IXGBE_RTTDCS_ARBDIS;
	IXGBE_WRITE_REG(hw, IXGBE_RTTDCS, rttdcs);
}

/**
 * ixgbe_configure_tx - Configure 8259x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/
static void ixgbe_configure_tx(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 dmatxctl;
	u32 i;

#ifdef CONFIG_NETDEVICES_MULTIQUEUE
	if (adapter->num_tx_queues > 1)
		adapter->netdev->features |= NETIF_F_MULTI_QUEUE;
	else
		adapter->netdev->features &= ~NETIF_F_MULTI_QUEUE;

#endif
	ixgbe_setup_mtqc(adapter);

	if (hw->mac.type != ixgbe_mac_82598EB) {
		/* DMATXCTL.EN must be before Tx queues are enabled */
		dmatxctl = IXGBE_READ_REG(hw, IXGBE_DMATXCTL);
		dmatxctl |= IXGBE_DMATXCTL_TE;
		IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL, dmatxctl);
	}

	/* Setup the HW Tx Head and Tail descriptor pointers */
	for (i = 0; i < adapter->num_tx_queues; i++)
		ixgbe_configure_tx_ring(adapter, adapter->tx_ring[i]);
}

#define IXGBE_SRRCTL_BSIZEHDRSIZE_SHIFT	2

static void ixgbe_configure_srrctl(struct ixgbe_adapter *adapter,
                                   struct ixgbe_ring *rx_ring)
{
	u32 srrctl;
	u8 reg_idx = rx_ring->reg_idx;

	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB: {
		struct ixgbe_ring_feature *feature = adapter->ring_feature;
		/* program one srrctl register per VMDq index */
		if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED) {
			unsigned long mask;
			long shift, len;
			mask = (unsigned long) feature[RING_F_VMDQ].mask;
			len = sizeof(feature[RING_F_VMDQ].mask) * 8;
			shift = find_first_bit(&mask, len);
			reg_idx = (reg_idx & mask) >> shift;
		} else {
			/*
			 * if VMDq is not active we must program one srrctl
			 * register per RSS queue since we have enabled
			 * RDRXCTL.MVMEN
			 */
			const int mask = feature[RING_F_RSS].mask;
			reg_idx = reg_idx & mask;
		}
	}
		break;
	case ixgbe_mac_82599EB:
	default:
		break;
	}

	srrctl = IXGBE_READ_REG(&adapter->hw, IXGBE_SRRCTL(reg_idx));

	srrctl &= ~IXGBE_SRRCTL_BSIZEHDR_MASK;
	srrctl &= ~IXGBE_SRRCTL_BSIZEPKT_MASK;
	if (adapter->num_vfs)
		srrctl |= IXGBE_SRRCTL_DROP_EN;

	srrctl |= (IXGBE_RX_HDR_SIZE << IXGBE_SRRCTL_BSIZEHDRSIZE_SHIFT) &
		   IXGBE_SRRCTL_BSIZEHDR_MASK;

	if (ring_is_ps_enabled(rx_ring)) {
#if (PAGE_SIZE / 2) > IXGBE_MAX_RXBUFFER
		srrctl |= IXGBE_MAX_RXBUFFER >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;
#else
		srrctl |= (PAGE_SIZE / 2) >> IXGBE_SRRCTL_BSIZEPKT_SHIFT;
#endif
		srrctl |= IXGBE_SRRCTL_DESCTYPE_HDR_SPLIT_ALWAYS;
	} else {
		srrctl |= ALIGN(rx_ring->rx_buf_len, 1024) >>
		          IXGBE_SRRCTL_BSIZEPKT_SHIFT;
		srrctl |= IXGBE_SRRCTL_DESCTYPE_ADV_ONEBUF;
	}

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_SRRCTL(reg_idx), srrctl);
}

static void ixgbe_setup_mrqc(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	static const u32 seed[10] = { 0xE291D73D, 0x1805EC6C, 0x2A94B30D,
	                  0xA54F2BEC, 0xEA49AF7C, 0xE214AD3D, 0xB855AABE,
	                  0x6A3E67EA, 0x14364D17, 0x3BED200D};
	u32 mrqc = 0, reta = 0;
	u32 rxcsum;
	int i, j;
	int mask;

	/* Fill out hash function seeds */
	for (i = 0; i < 10; i++)
		IXGBE_WRITE_REG(hw, IXGBE_RSSRK(i), seed[i]);

	/* Fill out redirection table */
	for (i = 0, j = 0; i < 128; i++, j++) {
		if (j == adapter->ring_feature[RING_F_RSS].indices)
			j = 0;
		/* reta = 4-byte sliding window of
		 * 0x00..(indices-1)(indices-1)00..etc. */
		reta = (reta << 8) | (j * 0x11);
		if ((i & 3) == 3)
			IXGBE_WRITE_REG(hw, IXGBE_RETA(i >> 2), reta);
	}

	/* Disable indicating checksum in descriptor, enables RSS hash */
	rxcsum = IXGBE_READ_REG(hw, IXGBE_RXCSUM);
	rxcsum |= IXGBE_RXCSUM_PCSD;
	IXGBE_WRITE_REG(hw, IXGBE_RXCSUM, rxcsum);

	if (adapter->hw.mac.type == ixgbe_mac_82598EB)
		mask = adapter->flags & IXGBE_FLAG_RSS_ENABLED;
	else
		mask = adapter->flags & (IXGBE_FLAG_RSS_ENABLED
					 | IXGBE_FLAG_DCB_ENABLED
					 | IXGBE_FLAG_VMDQ_ENABLED
					 | IXGBE_FLAG_SRIOV_ENABLED
					);

	switch (mask) {
	case (IXGBE_FLAG_RSS_ENABLED):
		mrqc = IXGBE_MRQC_RSSEN;
		break;
	case (IXGBE_FLAG_SRIOV_ENABLED):
		mrqc = IXGBE_MRQC_VMDQEN;
		break;
	case (IXGBE_FLAG_VMDQ_ENABLED):
	case (IXGBE_FLAG_VMDQ_ENABLED | IXGBE_FLAG_SRIOV_ENABLED):
		mrqc = IXGBE_MRQC_VMDQEN;
		break;
	case (IXGBE_FLAG_RSS_ENABLED | IXGBE_FLAG_VMDQ_ENABLED):
		if (adapter->ring_feature[RING_F_RSS].indices == 4)
			mrqc = IXGBE_MRQC_VMDQRSS32EN;
		else if (adapter->ring_feature[RING_F_RSS].indices == 2)
			mrqc = IXGBE_MRQC_VMDQRSS64EN;
		else
			mrqc = IXGBE_MRQC_VMDQEN;
		break;
	case (IXGBE_FLAG_DCB_ENABLED | IXGBE_FLAG_VMDQ_ENABLED):
	case (IXGBE_FLAG_DCB_ENABLED | IXGBE_FLAG_VMDQ_ENABLED
				| IXGBE_FLAG_SRIOV_ENABLED):
		mrqc = IXGBE_MRQC_VMDQRT4TCEN;	/* 4 TCs */
		break;
	case (IXGBE_FLAG_DCB_ENABLED):
		mrqc = IXGBE_MRQC_RT8TCEN;
		break;
	default:
		break;
	}

	/* Perform hash on these packet types */
	mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4
	      | IXGBE_MRQC_RSS_FIELD_IPV4_TCP
	      | IXGBE_MRQC_RSS_FIELD_IPV6
	      | IXGBE_MRQC_RSS_FIELD_IPV6_TCP;

	IXGBE_WRITE_REG(hw, IXGBE_MRQC, mrqc);
}

/**
 * ixgbe_configure_rscctl - enable RSC for the indicated ring
 * @adapter:    address of board private structure
 * @ring: structure containing ring specific data
 **/
void ixgbe_configure_rscctl(struct ixgbe_adapter *adapter,
			    struct ixgbe_ring *ring)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rscctrl;
	int rx_buf_len;
	u8 reg_idx = ring->reg_idx;

	if (!ring_is_rsc_enabled(ring))
		return;

	rx_buf_len = ring->rx_buf_len;
	rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(reg_idx));
	rscctrl |= IXGBE_RSCCTL_RSCEN;
	/*
	 * we must limit the number of descriptors so that
	 * the total size of max desc * buf_len is not greater
	 * than 65535
	 */
	if (ring_is_ps_enabled(ring)) {
#if (MAX_SKB_FRAGS > 16)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_16;
#elif (MAX_SKB_FRAGS > 8)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_8;
#elif (MAX_SKB_FRAGS > 4)
		rscctrl |= IXGBE_RSCCTL_MAXDESC_4;
#else
		rscctrl |= IXGBE_RSCCTL_MAXDESC_1;
#endif
	} else {
		if (rx_buf_len < IXGBE_RXBUFFER_4096)
			rscctrl |= IXGBE_RSCCTL_MAXDESC_16;
		else if (rx_buf_len < IXGBE_RXBUFFER_8192)
			rscctrl |= IXGBE_RSCCTL_MAXDESC_8;
		else
			rscctrl |= IXGBE_RSCCTL_MAXDESC_4;
	}

	IXGBE_WRITE_REG(hw, IXGBE_RSCCTL(reg_idx), rscctrl);
}

/**
 * ixgbe_clear_rscctl - disable RSC for the indicated ring
 * @adapter:    address of board private structure
 * @ring: structure containing ring specific data
 **/
void ixgbe_clear_rscctl(struct ixgbe_adapter *adapter,
			struct ixgbe_ring *ring)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rscctrl;
	u8 reg_idx = ring->reg_idx;

	rscctrl = IXGBE_READ_REG(hw, IXGBE_RSCCTL(reg_idx));
	rscctrl &= ~IXGBE_RSCCTL_RSCEN;
	IXGBE_WRITE_REG(hw, IXGBE_RSCCTL(reg_idx), rscctrl);

	clear_ring_rsc_enabled(ring);
}

/**
 *  ixgbe_set_uta - Set unicast filter table address
 *  @adapter: board private structure
 *
 *  The unicast table address is a register array of 32-bit registers.
 *  The table is meant to be used in a way similar to how the MTA is used
 *  however due to certain limitations in the hardware it is necessary to
 *  set all the hash bits to 1 and use the VMOLR ROPE bit as a promiscuous
 *  enable bit to allow vlan tag stripping when promiscuous mode is enabled
 **/
static void ixgbe_set_uta(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int i;

	/* The UTA table only exists on 82599 hardware and newer */
	if (hw->mac.type < ixgbe_mac_82599EB)
		return;

	/* we only need to do this if VMDq is enabled */
	if (!(adapter->flags &
	      (IXGBE_FLAG_VMDQ_ENABLED | IXGBE_FLAG_SRIOV_ENABLED)))
		return;

	for (i = 0; i < 128; i++)
		IXGBE_WRITE_REG(hw, IXGBE_UTA(i), ~0);
}

static void ixgbe_rx_desc_queue_enable(struct ixgbe_adapter *adapter,
				       struct ixgbe_ring *ring)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int wait_loop = IXGBE_MAX_RX_DESC_POLL;
	u32 rxdctl;
	u8 reg_idx = ring->reg_idx;

	/* RXDCTL.EN will return 0 on 82598 if link is down, so skip it */
	if (hw->mac.type == ixgbe_mac_82598EB &&
	    !(IXGBE_READ_REG(hw, IXGBE_LINKS) & IXGBE_LINKS_UP))
		return;

	do {
		msleep(1);
		rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(reg_idx));
	} while (--wait_loop && !(rxdctl & IXGBE_RXDCTL_ENABLE));

	if (!wait_loop) {
		DPRINTK(DRV, ERR, "RXDCTL.ENABLE on Rx queue %d "
		        "not set within the polling period\n", reg_idx);
	}
}

void ixgbe_configure_rx_ring(struct ixgbe_adapter *adapter,
			     struct ixgbe_ring *ring)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u64 rdba = ring->dma;
	u32 rxdctl;
	u8 reg_idx = ring->reg_idx;

	/* disable queue to avoid issues while updating state */
	rxdctl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(reg_idx));
	IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(reg_idx),
			rxdctl & ~IXGBE_RXDCTL_ENABLE);
	IXGBE_WRITE_FLUSH(hw);

	IXGBE_WRITE_REG(hw, IXGBE_RDBAL(reg_idx), (rdba & DMA_BIT_MASK(32)));
	IXGBE_WRITE_REG(hw, IXGBE_RDBAH(reg_idx), (rdba >> 32));
	IXGBE_WRITE_REG(hw, IXGBE_RDLEN(reg_idx),
			ring->count * sizeof(union ixgbe_adv_rx_desc));
	IXGBE_WRITE_REG(hw, IXGBE_RDH(reg_idx), 0);
	IXGBE_WRITE_REG(hw, IXGBE_RDT(reg_idx), 0);
	ring->tail = hw->hw_addr + IXGBE_RDT(reg_idx);

	ixgbe_configure_srrctl(adapter, ring);
	ixgbe_configure_rscctl(adapter, ring);

	if (hw->mac.type == ixgbe_mac_82598EB) {
		/*
		 * enable cache line friendly hardware writes:
		 * PTHRESH=32 descriptors (half the internal cache),
		 * this also removes ugly rx_no_buffer_count increment
		 * HTHRESH=4 descriptors (to minimize latency on fetch)
		 * WTHRESH=8 burst writeback up to two cache lines
		 */
		rxdctl &= ~0x3FFFFF;
		rxdctl |=  0x080420;
	}

	/* enable receive descriptor ring */
	rxdctl |= IXGBE_RXDCTL_ENABLE;
	IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(reg_idx), rxdctl);

	ixgbe_rx_desc_queue_enable(adapter, ring);
	ixgbe_alloc_rx_buffers(ring, IXGBE_DESC_UNUSED(ring));
}

static void ixgbe_setup_psrtype(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int p;

	/* PSRTYPE must be initialized in non 82598 adapters */
	u32 psrtype = IXGBE_PSRTYPE_TCPHDR |
	              IXGBE_PSRTYPE_UDPHDR |
	              IXGBE_PSRTYPE_IPV4HDR |
		      IXGBE_PSRTYPE_L2HDR |
	              IXGBE_PSRTYPE_IPV6HDR;

	if (hw->mac.type == ixgbe_mac_82598EB)
		return;

	if (adapter->flags & IXGBE_FLAG_RSS_ENABLED)
		psrtype |= (adapter->num_rx_queues_per_pool << 29);

	for (p = 0; p < adapter->num_rx_pools; p++)
		IXGBE_WRITE_REG(hw, IXGBE_PSRTYPE(VMDQ_P(p)), psrtype);
}

static void ixgbe_configure_virtualization(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 gcr_ext;
	u32 vt_reg;
	u32 vt_reg_bits;
	u32 pool;
	u32 vmdctl;

	if (!(adapter->flags & IXGBE_FLAG_VMDQ_ENABLED ||
	      adapter->flags & IXGBE_FLAG_SRIOV_ENABLED))
		return;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		vt_reg = IXGBE_VMD_CTL;
		vt_reg_bits = IXGBE_VMD_CTL_VMDQ_EN;
		vmdctl = IXGBE_READ_REG(hw, vt_reg);
		IXGBE_WRITE_REG(hw, vt_reg, vmdctl | vt_reg_bits);
		break;
	case ixgbe_mac_82599EB:
		vt_reg = IXGBE_VT_CTL;
		vt_reg_bits = IXGBE_VMD_CTL_VMDQ_EN
				| IXGBE_VT_CTL_REPLEN;
		if (adapter->num_vfs) {
			vt_reg_bits &= ~IXGBE_VT_CTL_POOL_MASK;
			vt_reg_bits |= (adapter->num_vfs <<
					IXGBE_VT_CTL_POOL_SHIFT);
		}
		vmdctl = IXGBE_READ_REG(hw, vt_reg);
		IXGBE_WRITE_REG(hw, vt_reg, vmdctl | vt_reg_bits);
		for (pool = 1; pool < adapter->num_rx_pools; pool++) {
			u32 vmolr;
			int vmdq_pool = VMDQ_P(pool);

			/*
			* accept untagged packets until a vlan tag
			* is specifically set for the VMDQ queue/pool
			*/
			vmolr = IXGBE_READ_REG(hw, IXGBE_VMOLR(vmdq_pool));
			vmolr |= IXGBE_VMOLR_AUPE;
			vmolr |= IXGBE_VMOLR_BAM;
			IXGBE_WRITE_REG(hw, IXGBE_VMOLR(vmdq_pool), vmolr);
		}
		IXGBE_WRITE_REG(hw, IXGBE_VFRE(0), 0xFFFFFFFF);
		IXGBE_WRITE_REG(hw, IXGBE_VFRE(1), 0xFFFFFFFF);
		IXGBE_WRITE_REG(hw, IXGBE_VFTE(0), 0xFFFFFFFF);
		IXGBE_WRITE_REG(hw, IXGBE_VFTE(1), 0xFFFFFFFF);
		break;
	default:
		break;
	}

	if (!(adapter->flags & IXGBE_FLAG_SRIOV_ENABLED))
		return;

	/* Map PF MAC address in RAR Entry 0 to first pool following VFs */
	hw->mac.ops.set_vmdq(hw, 0, adapter->num_vfs);

	/*
	 * Set up VF register offsets for selected VT Mode,
	 * i.e. 32 or 64 VFs for SR-IOV
	 */
	gcr_ext = IXGBE_READ_REG(hw, IXGBE_GCR_EXT);
	gcr_ext |= IXGBE_GCR_EXT_MSIX_EN;
	gcr_ext |= IXGBE_GCR_EXT_VT_MODE_64;
	IXGBE_WRITE_REG(hw, IXGBE_GCR_EXT, gcr_ext);

	/* enable Tx loopback for VF/PF communication */
	IXGBE_WRITE_REG(hw, IXGBE_PFDTXGSWC, IXGBE_PFDTXGSWC_VT_LBEN);
}

static void ixgbe_set_rx_buffer_len(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	struct net_device *netdev = adapter->netdev;
	int max_frame = netdev->mtu + ETH_HLEN + ETH_FCS_LEN;
	int rx_buf_len;
	struct ixgbe_ring *rx_ring;
	int i;
	u32 mhadd, hlreg0;

	/* Decide whether to use packet split mode or not */
	if (netdev->mtu > ETH_DATA_LEN) {
		if (adapter->flags & IXGBE_FLAG_RX_PS_CAPABLE)
			adapter->flags |= IXGBE_FLAG_RX_PS_ENABLED;
		else
			adapter->flags &= ~IXGBE_FLAG_RX_PS_ENABLED;
	} else {
		if (adapter->flags & IXGBE_FLAG_RX_1BUF_CAPABLE)
			adapter->flags &= ~IXGBE_FLAG_RX_PS_ENABLED;
		else
			adapter->flags |= IXGBE_FLAG_RX_PS_ENABLED;
	}

	/* Set the RX buffer length according to the mode */
	if (adapter->flags & IXGBE_FLAG_RX_PS_ENABLED) {
		rx_buf_len = IXGBE_RX_HDR_SIZE;
	} else {
		if (!(adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED) &&
		    (netdev->mtu <= ETH_DATA_LEN))
			rx_buf_len = MAXIMUM_ETHERNET_VLAN_SIZE;
		else
			rx_buf_len = ALIGN(max_frame + VLAN_HLEN, 1024);
	}

#ifdef IXGBE_FCOE
	/* adjust max frame to be able to do baby jumbo for FCoE */
	if ((adapter->flags & IXGBE_FLAG_FCOE_ENABLED) &&
	    (max_frame < IXGBE_FCOE_JUMBO_FRAME_SIZE))
		max_frame = IXGBE_FCOE_JUMBO_FRAME_SIZE;

#endif /* IXGBE_FCOE */
	mhadd = IXGBE_READ_REG(hw, IXGBE_MHADD);
	if (max_frame != (mhadd >> IXGBE_MHADD_MFS_SHIFT)) {
		mhadd &= ~IXGBE_MHADD_MFS_MASK;
		mhadd |= max_frame << IXGBE_MHADD_MFS_SHIFT;

		IXGBE_WRITE_REG(hw, IXGBE_MHADD, mhadd);
	}

	hlreg0 = IXGBE_READ_REG(hw, IXGBE_HLREG0);
	/* set jumbo enable since MHADD.MFS is keeping size locked at max_frame */
	hlreg0 |= IXGBE_HLREG0_JUMBOEN;
	IXGBE_WRITE_REG(hw, IXGBE_HLREG0, hlreg0);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < adapter->num_rx_queues; i++) {
		rx_ring = adapter->rx_ring[i];
		rx_ring->rx_buf_len = rx_buf_len;

		if (adapter->flags & IXGBE_FLAG_RX_PS_ENABLED)
			set_ring_ps_enabled(rx_ring);
		else
			clear_ring_ps_enabled(rx_ring);

		if (adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED) {
			set_ring_rsc_enabled(rx_ring);
#ifndef IXGBE_NO_LRO
			clear_ring_lro_enabled(rx_ring);
		} else if (adapter->flags2 & IXGBE_FLAG2_SWLRO_ENABLED) {
			set_ring_lro_enabled(rx_ring);
			clear_ring_rsc_enabled(rx_ring);
		} else {
			clear_ring_lro_enabled(rx_ring);
#endif
			clear_ring_rsc_enabled(rx_ring);
		}

#ifdef IXGBE_FCOE
		if (netdev->features & NETIF_F_FCOE_MTU)
		{
			struct ixgbe_ring_feature *f;
			f = &adapter->ring_feature[RING_F_FCOE];
			if ((i >= f->mask) && (i < f->mask + f->indices)) {
				clear_ring_ps_enabled(rx_ring);
				if (rx_buf_len < IXGBE_FCOE_JUMBO_FRAME_SIZE)
					rx_ring->rx_buf_len =
					        IXGBE_FCOE_JUMBO_FRAME_SIZE;
			} else if (!ring_is_rsc_enabled(rx_ring) &&
			           !ring_is_ps_enabled(rx_ring)) {
				rx_ring->rx_buf_len =
					        IXGBE_FCOE_JUMBO_FRAME_SIZE;
			}
		}
#endif /* IXGBE_FCOE */
	}
}

static void ixgbe_setup_rdrxctl(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rdrxctl = IXGBE_READ_REG(hw, IXGBE_RDRXCTL);

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		/*
		 * For VMDq support of different descriptor types or
		 * buffer sizes through the use of multiple SRRCTL
		 * registers, RDRXCTL.MVMEN must be set to 1
		 *
		 * also, the manual doesn't mention it clearly but DCA hints
		 * will only use queue 0's tags unless this bit is set.  Side
		 * effects of setting this bit are only that SRRCTL must be
		 * fully programmed [0..15]
		 */
		rdrxctl |= IXGBE_RDRXCTL_MVMEN;
		break;
	case ixgbe_mac_82599EB:
		/* Disable RSC for ACK packets */
		IXGBE_WRITE_REG(hw, IXGBE_RSCDBU,
		   (IXGBE_RSCDBU_RSCACKDIS | IXGBE_READ_REG(hw, IXGBE_RSCDBU)));
		rdrxctl &= ~IXGBE_RDRXCTL_RSCFRSTSIZE;
		/* hardware requires some bits to be set by default */
		rdrxctl |= (IXGBE_RDRXCTL_RSCACKC | IXGBE_RDRXCTL_FCOE_WRFIX);
		rdrxctl |= IXGBE_RDRXCTL_CRCSTRIP;
		break;
	default:
		/* We should do nothing since we don't know this hardware */
		return;
	}

	IXGBE_WRITE_REG(hw, IXGBE_RDRXCTL, rdrxctl);
}

/**
 * ixgbe_configure_rx - Configure 8259x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void ixgbe_configure_rx(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int i;
	u32 rxctrl;

	/* disable receives while setting up the descriptors */
	rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	IXGBE_WRITE_REG(hw, IXGBE_RXCTRL, rxctrl & ~IXGBE_RXCTRL_RXEN);

	ixgbe_setup_psrtype(adapter);
	ixgbe_setup_rdrxctl(adapter);

	/* Program registers for the distribution of queues */
	ixgbe_setup_mrqc(adapter);

	ixgbe_set_uta(adapter);

	/* set_rx_buffer_len must be called before ring initialization */
	ixgbe_set_rx_buffer_len(adapter);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers and
	 * the Base and Length of the Rx Descriptor Ring
	 */
	for (i = 0; i < adapter->num_rx_queues; i++)
		ixgbe_configure_rx_ring(adapter, adapter->rx_ring[i]);

	/* disable drop enable for 82598 parts */
	if (hw->mac.type == ixgbe_mac_82598EB)
		rxctrl |= IXGBE_RXCTRL_DMBYPS;

	/* enable all receives */
	rxctrl |= IXGBE_RXCTRL_RXEN;
	ixgbe_enable_rx_dma(hw, rxctrl);

}

#ifdef NETIF_F_HW_VLAN_TX
static void ixgbe_vlan_rx_add_vid(struct net_device *netdev, u16 vid)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	int pool_ndx = adapter->num_vfs;
#ifndef HAVE_NETDEV_VLAN_FEATURES
	struct net_device *v_netdev;
#endif /* HAVE_NETDEV_VLAN_FEATURES */
	int i;

	/* add VID to filter table */
	if (hw->mac.ops.set_vfta) {
		hw->mac.ops.set_vfta(hw, vid, pool_ndx, true);
		if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED) {
			switch (adapter->hw.mac.type) {
			case ixgbe_mac_82599EB:
				/* enable vlan id for all pools */
				for (i = 1; i < adapter->num_rx_pools; i++) {
					hw->mac.ops.set_vfta(hw, vid, VMDQ_P(i), true);
				}
				break;
			default:
				break;
			}
		}
	}

#ifndef HAVE_NETDEV_VLAN_FEATURES
	/*
	 * Copy feature flags from netdev to the vlan netdev for this vid.
	 * This allows things like TSO to bubble down to our vlan device.
	 * Some vlans, such as VLAN 0 for DCB will not have a v_netdev so
	 * we will not have a netdev that needs updating.
	 */
	if (adapter->vlgrp) {
		v_netdev = vlan_group_get_device(adapter->vlgrp, vid);
		if (v_netdev) {
			v_netdev->features |= adapter->netdev->features;
			vlan_group_set_device(adapter->vlgrp, vid, v_netdev);
		}
	}
#endif /* HAVE_NETDEV_VLAN_FEATURES */
}

static void ixgbe_vlan_rx_kill_vid(struct net_device *netdev, u16 vid)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	int pool_ndx = adapter->num_vfs;
	int i;

	/* User is not allowed to remove vlan ID 0 */
	if (!vid)
		return;

	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_disable(adapter);

	vlan_group_set_device(adapter->vlgrp, vid, NULL);

	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_enable(adapter, true, true);

	/* remove VID from filter table */

	if (hw->mac.ops.set_vfta) {
		hw->mac.ops.set_vfta(hw, vid, pool_ndx, false);
		if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED) {
			switch (adapter->hw.mac.type) {
			case ixgbe_mac_82599EB:
				/* remove vlan id from all pools */
				for (i = 1; i < adapter->num_rx_pools; i++) {
					hw->mac.ops.set_vfta(hw, vid, VMDQ_P(i), false);
				}
				break;
			default:
				break;
			}
		}
	}
}
#endif

/**
 * ixgbe_vlan_stripping_disable - helper to disable vlan tag stripping
 * @adapter: driver data
 */
static void ixgbe_vlan_stripping_disable(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 vlnctrl;
	int i;

	/* leave vlan tag stripping enabled for DCB */
	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED)
		return;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
		vlnctrl &= ~IXGBE_VLNCTRL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
		break;
	case ixgbe_mac_82599EB:
		for (i = 0; i < adapter->num_rx_queues; i++) {
			u8 reg_idx = adapter->rx_ring[i]->reg_idx;
			vlnctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(reg_idx));
			vlnctrl &= ~IXGBE_RXDCTL_VME;
			IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(reg_idx), vlnctrl);
		}
		break;
	default:
		break;
	}
}

/**
 * ixgbe_vlan_stripping_enable - helper to enable vlan tag stripping
 * @adapter: driver data
 */
static void ixgbe_vlan_stripping_enable(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 vlnctrl;
	int i;

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);
		vlnctrl |= IXGBE_VLNCTRL_VME;
		IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
		break;
	case ixgbe_mac_82599EB:
		for (i = 0; i < adapter->num_rx_queues; i++) {
			u8 reg_idx = adapter->rx_ring[i]->reg_idx;
			vlnctrl = IXGBE_READ_REG(hw, IXGBE_RXDCTL(reg_idx));
			vlnctrl |= IXGBE_RXDCTL_VME;
			IXGBE_WRITE_REG(hw, IXGBE_RXDCTL(reg_idx), vlnctrl);
		}
		break;
	default:
		break;
	}
}

#ifdef NETIF_F_HW_VLAN_TX
static void ixgbe_vlan_rx_register(struct net_device *netdev,
                                   struct vlan_group *grp)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);

	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_disable(adapter);
	adapter->vlgrp = grp;

	if (grp || (adapter->flags & IXGBE_FLAG_DCB_ENABLED))
		/* enable VLAN tag insert/strip */
		ixgbe_vlan_stripping_enable(adapter);
	else
		/* disable VLAN tag insert/strip */
		ixgbe_vlan_stripping_disable(adapter);

	if (!test_bit(__IXGBE_DOWN, &adapter->state))
		ixgbe_irq_enable(adapter, true, true);
}

static void ixgbe_restore_vlan(struct ixgbe_adapter *adapter)
{
	ixgbe_vlan_rx_register(adapter->netdev, adapter->vlgrp);

	/* add vlan ID 0 so we always accept priority-tagged traffic */
	ixgbe_vlan_rx_add_vid(adapter->netdev, 0);

	if (adapter->vlgrp) {
		u16 vid;
		for (vid = 0; vid < VLAN_GROUP_ARRAY_LEN; vid++) {
			if (!vlan_group_get_device(adapter->vlgrp, vid))
				continue;
			ixgbe_vlan_rx_add_vid(adapter->netdev, vid);
		}
	}
}

#endif
static u8 *ixgbe_addr_list_itr(struct ixgbe_hw *hw, u8 **mc_addr_ptr, u32 *vmdq)
{
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *mc_ptr;
#else
	struct dev_mc_list *mc_ptr;
#endif
	struct ixgbe_adapter *adapter = hw->back;
	u8 *addr = *mc_addr_ptr;

	*vmdq = adapter->num_vfs;

#ifdef NETDEV_HW_ADDR_T_MULTICAST
	mc_ptr = container_of(addr, struct netdev_hw_addr, addr[0]);
	if (mc_ptr->list.next) {
		struct netdev_hw_addr *ha;

		ha = list_entry(mc_ptr->list.next, struct netdev_hw_addr, list);
		*mc_addr_ptr = ha->addr;
	}
#else
	mc_ptr = container_of(addr, struct dev_mc_list, dmi_addr[0]);
	if (mc_ptr->next)
		*mc_addr_ptr = mc_ptr->next->dmi_addr;
#endif
	else
		*mc_addr_ptr = NULL;

	return addr;
}

/**
 * ixgbe_write_mc_addr_list - write multicast addresses to MTA
 * @netdev: network interface device structure
 *
 * Writes multicast address list to the MTA hash table.
 * Returns: -ENOMEM on failure
 *                0 on no addresses written
 *                X on writing X addresses to MTA
 **/
static int ixgbe_write_mc_addr_list(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
#ifdef NETDEV_HW_ADDR_T_MULTICAST
	struct netdev_hw_addr *ha;
#endif
	u8  *addr_list = NULL;
	int addr_count;

	if (netdev_mc_empty(netdev)) {
		/* nothing to program, so clear mc list */
		hw->mac.ops.update_mc_addr_list(hw, NULL, 0, ixgbe_addr_list_itr);
		return 0;
	}

	if (!hw->mac.ops.update_mc_addr_list)
		return -ENOMEM;

#ifdef NETDEV_HW_ADDR_T_MULTICAST
	ha = list_first_entry(&netdev->mc.list, struct netdev_hw_addr, list);
	addr_list = ha->addr;
#else
	addr_list = netdev->mc_list->dmi_addr;
#endif
	addr_count = netdev_mc_count(netdev);

	hw->mac.ops.update_mc_addr_list(hw, addr_list, addr_count, ixgbe_addr_list_itr);

	return addr_count;
}

#ifdef HAVE_SET_RX_MODE
/**
 * ixgbe_write_uc_addr_list - write unicast addresses to RAR table
 * @netdev: network interface device structure
 *
 * Writes unicast address list to the RAR table.
 * Returns: -ENOMEM on failure/insufficient address space
 *                0 on no addresses written
 *                X on writing X addresses to the RAR table
 **/
static int ixgbe_write_uc_addr_list(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	unsigned int vfn = adapter->num_vfs;
	unsigned int rar_entries = hw->mac.num_rar_entries - (vfn + 1);
	int count = 0;

	/* return ENOMEM indicating insufficient memory for addresses */
	if (netdev_uc_count(netdev) > rar_entries)
		return -ENOMEM;

	if (!netdev_uc_empty(netdev) && rar_entries) {
#ifdef NETDEV_HW_ADDR_T_UNICAST
		struct netdev_hw_addr *ha;
#else
		struct dev_mc_list *ha;
#endif
		/* return error if we do not support writing to RAR table */
		if (!hw->mac.ops.set_rar)
			return -ENOMEM;

		netdev_for_each_uc_addr(ha, netdev) {
			if (!rar_entries)
				break;
#ifdef NETDEV_HW_ADDR_T_UNICAST
			hw->mac.ops.set_rar(hw, rar_entries--, ha->addr,
					    vfn, IXGBE_RAH_AV);
#else
			hw->mac.ops.set_rar(hw, rar_entries--, ha->da_addr,
					    vfn, IXGBE_RAH_AV);
#endif
			count++;
		}
	}
	/* write the addresses in reverse order to avoid write combining */
	for (; rar_entries > 0 ; rar_entries--)
		hw->mac.ops.clear_rar(hw, rar_entries);

	return count;
}

#endif
/**
 * ixgbe_set_rx_mode - Unicast, Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_rx_method entry point is called whenever the unicast/multicast
 * address list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper unicast, multicast and
 * promiscuous mode.
 **/
void ixgbe_set_rx_mode(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	u32 fctrl, vmolr = IXGBE_VMOLR_BAM | IXGBE_VMOLR_AUPE;
	u32 vlnctrl;
	int count;

	/* Check for Promiscuous and All Multicast modes */
	fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
	vlnctrl = IXGBE_READ_REG(hw, IXGBE_VLNCTRL);

	/* set all bits that we expect to always be set */
	fctrl |= IXGBE_FCTRL_BAM;
	fctrl |= IXGBE_FCTRL_DPF; /* discard pause frames when FC enabled */
	fctrl |= IXGBE_FCTRL_PMCF;

	/* clear the bits we are changing the status of */
	fctrl &= ~(IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
	vlnctrl  &= ~(IXGBE_VLNCTRL_VFE | IXGBE_VLNCTRL_CFIEN);

	if (netdev->flags & IFF_PROMISC) {
		hw->addr_ctrl.user_set_promisc = true;
		fctrl |= (IXGBE_FCTRL_UPE | IXGBE_FCTRL_MPE);
		vmolr |= IXGBE_VMOLR_MPE;
	} else {
		if (netdev->flags & IFF_ALLMULTI) {
			fctrl |= IXGBE_FCTRL_MPE;
			vmolr |= IXGBE_VMOLR_MPE;
		} else {
			/*
			 * Write addresses to the MTA, if the attempt fails
			 * then we should just turn on promiscous mode so
			 * that we can at least receive multicast traffic
			 */
			count = ixgbe_write_mc_addr_list(netdev);
			if (count < 0) {
				fctrl |= IXGBE_FCTRL_MPE;
				vmolr |= IXGBE_VMOLR_MPE;
			} else if (count) {
				vmolr |= IXGBE_VMOLR_ROMPE;
			}
		}
#ifdef NETIF_F_HW_VLAN_TX
		/* enable hardware vlan filtering */
		vlnctrl |= IXGBE_VLNCTRL_VFE;
#endif
		hw->addr_ctrl.user_set_promisc = false;
#ifdef HAVE_SET_RX_MODE
		/*
		 * Write addresses to available RAR registers, if there is not
		 * sufficient space to store all the addresses then enable
		 * unicast promiscous mode
		 */
		count = ixgbe_write_uc_addr_list(netdev);
		if (count < 0) {
			fctrl |= IXGBE_FCTRL_UPE;
			vmolr |= IXGBE_VMOLR_ROPE;
		}
#endif
	}

#ifdef CONFIG_PCI_IOV
	if (adapter->num_vfs)
		ixgbe_restore_vf_multicasts(adapter);

#endif
	if (hw->mac.type != ixgbe_mac_82598EB) {
		vmolr |= IXGBE_READ_REG(hw, IXGBE_VMOLR(adapter->num_vfs)) &
			 ~(IXGBE_VMOLR_MPE | IXGBE_VMOLR_ROMPE |
			   IXGBE_VMOLR_ROPE);
		IXGBE_WRITE_REG(hw, IXGBE_VMOLR(adapter->num_vfs), vmolr);
	}

	IXGBE_WRITE_REG(hw, IXGBE_VLNCTRL, vlnctrl);
	IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
}

static void ixgbe_napi_enable_all(struct ixgbe_adapter *adapter)
{
#ifdef CONFIG_IXGBE_NAPI
	int q_idx;
	struct ixgbe_q_vector *q_vector;
	int q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

	/* legacy and MSI only use one vector */
	if (!(adapter->flags & IXGBE_FLAG_MSIX_ENABLED))
		q_vectors = 1;

	for (q_idx = 0; q_idx < q_vectors; q_idx++) {
		struct napi_struct *napi;
		q_vector = adapter->q_vector[q_idx];
		napi = &q_vector->napi;
		if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
			if (!q_vector->rxr_count || !q_vector->txr_count) {
				if (q_vector->txr_count == 1)
					napi->poll = &ixgbe_clean_txonly;
				else if (q_vector->rxr_count == 1)
					napi->poll = &ixgbe_clean_rxonly;
			}
		}

		napi_enable(napi);
	}
#endif /* CONFIG_IXGBE_NAPI */
}

static void ixgbe_napi_disable_all(struct ixgbe_adapter *adapter)
{
#ifdef CONFIG_IXGBE_NAPI
	int q_idx;
	struct ixgbe_q_vector *q_vector;
	int q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

	/* legacy and MSI only use one vector */
	if (!(adapter->flags & IXGBE_FLAG_MSIX_ENABLED))
		q_vectors = 1;

	for (q_idx = 0; q_idx < q_vectors; q_idx++) {
		q_vector = adapter->q_vector[q_idx];
		napi_disable(&q_vector->napi);
	}
#endif
}

/*
 * ixgbe_configure_dcb - Configure DCB hardware
 * @adapter: ixgbe adapter struct
 *
 * This is called by the driver on open to configure the DCB hardware.
 * This is also called by the gennetlink interface when reconfiguring
 * the DCB state.
 */
static void ixgbe_configure_dcb(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	s32 err;
	u32 mtu = adapter->netdev->mtu;

	if (!(adapter->flags & IXGBE_FLAG_DCB_ENABLED)) {
		if (hw->mac.type == ixgbe_mac_82598EB)
			netif_set_gso_max_size(adapter->netdev, 65536);
		return;
	}

	if (hw->mac.type == ixgbe_mac_82598EB)
		netif_set_gso_max_size(adapter->netdev, 32768);

#ifdef IXGBE_FCOE
	if (adapter->netdev->features & NETIF_F_FCOE_MTU)
		mtu = max(mtu, (unsigned) IXGBE_FCOE_JUMBO_FRAME_SIZE);

#endif
	adapter->dcb_cfg.num_tcs.pg_tcs = adapter->ring_feature[RING_F_DCB].indices;
	err = ixgbe_dcb_check_config(&adapter->dcb_cfg);
	if (err)
		DPRINTK(DRV, ERR, "err in dcb_check_config\n");
	err = ixgbe_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, mtu,
					     DCB_TX_CONFIG);
	if (err)
		DPRINTK(DRV, ERR, "err in dcb_calculate_tc_credits (TX)\n");
	err = ixgbe_dcb_calculate_tc_credits(hw, &adapter->dcb_cfg, mtu,
					     DCB_RX_CONFIG);
	if (err)
		DPRINTK(DRV, ERR, "err in dcb_calculate_tc_credits (RX)\n");

	/* reconfigure the hardware */
	ixgbe_dcb_hw_config(hw, &adapter->dcb_cfg);
}

#ifndef IXGBE_NO_LLI
static void ixgbe_configure_lli_82599(struct ixgbe_adapter *adapter)
{
	u16 port;

	if (adapter->lli_etype) {
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_L34T_IMIR(0),
		                (IXGBE_IMIR_LLI_EN_82599 | IXGBE_IMIR_SIZE_BP_82599 |
		                 IXGBE_IMIR_CTRL_BP_82599));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_ETQS(0), IXGBE_ETQS_LLI);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_ETQF(0),
		                (adapter->lli_etype | IXGBE_ETQF_FILTER_EN));
	}

	if (adapter->lli_port) {
		port = ntohs((u16)adapter->lli_port);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_L34T_IMIR(0),
		                (IXGBE_IMIR_LLI_EN_82599 | IXGBE_IMIR_SIZE_BP_82599 |
		                 IXGBE_IMIR_CTRL_BP_82599));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_FTQF(0),
		                (IXGBE_FTQF_POOL_MASK_EN |
		                 (IXGBE_FTQF_PRIORITY_MASK <<
		                  IXGBE_FTQF_PRIORITY_SHIFT) |
		                 (IXGBE_FTQF_DEST_PORT_MASK <<
		                  IXGBE_FTQF_5TUPLE_MASK_SHIFT)));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_SDPQF(0), (port << 16));
	}

	if (adapter->flags & IXGBE_FLAG_LLI_PUSH) {
		switch (adapter->hw.mac.type) {
		case ixgbe_mac_82599EB:
			IXGBE_WRITE_REG(&adapter->hw, IXGBE_L34T_IMIR(0),
			                (IXGBE_IMIR_LLI_EN_82599 | IXGBE_IMIR_SIZE_BP_82599 |
			                 IXGBE_IMIR_CTRL_PSH_82599 | IXGBE_IMIR_CTRL_SYN_82599 |
			                 IXGBE_IMIR_CTRL_URG_82599 | IXGBE_IMIR_CTRL_ACK_82599 |
			                 IXGBE_IMIR_CTRL_RST_82599 | IXGBE_IMIR_CTRL_FIN_82599));
			IXGBE_WRITE_REG(&adapter->hw, IXGBE_LLITHRESH, 0xfc000000);
			break;
		default:
			break;
		}
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_FTQF(0),
		                (IXGBE_FTQF_POOL_MASK_EN |
		                 (IXGBE_FTQF_PRIORITY_MASK <<
		                  IXGBE_FTQF_PRIORITY_SHIFT) |
		                 (IXGBE_FTQF_5TUPLE_MASK_MASK <<
		                  IXGBE_FTQF_5TUPLE_MASK_SHIFT)));

		IXGBE_WRITE_REG(&adapter->hw, IXGBE_SYNQF, 0x80000100);
	}

	if (adapter->lli_size) {
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_L34T_IMIR(0),
		                (IXGBE_IMIR_LLI_EN_82599 | IXGBE_IMIR_CTRL_BP_82599));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_LLITHRESH, adapter->lli_size);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_FTQF(0),
		                (IXGBE_FTQF_POOL_MASK_EN |
		                 (IXGBE_FTQF_PRIORITY_MASK <<
		                  IXGBE_FTQF_PRIORITY_SHIFT) |
		                 (IXGBE_FTQF_5TUPLE_MASK_MASK <<
		                  IXGBE_FTQF_5TUPLE_MASK_SHIFT)));
	}

	if (adapter->lli_vlan_pri) {
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIRVP,
		                (IXGBE_IMIRVP_PRIORITY_EN | adapter->lli_vlan_pri));
	}
}

static void ixgbe_configure_lli(struct ixgbe_adapter *adapter)
{
	u16 port;

	/* lli should only be enabled with MSI-X and MSI */
	if (!(adapter->flags & IXGBE_FLAG_MSI_ENABLED) &&
	    !(adapter->flags & IXGBE_FLAG_MSIX_ENABLED))
		return;

	if (adapter->hw.mac.type != ixgbe_mac_82598EB) {
		ixgbe_configure_lli_82599(adapter);
		return;
	}

	if (adapter->lli_port) {
		/* use filter 0 for port */
		port = ntohs((u16)adapter->lli_port);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIR(0),
		                (port | IXGBE_IMIR_PORT_IM_EN));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIREXT(0),
		                (IXGBE_IMIREXT_SIZE_BP |
		                 IXGBE_IMIREXT_CTRL_BP));
	}

	if (adapter->flags & IXGBE_FLAG_LLI_PUSH) {
		/* use filter 1 for push flag */
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIR(1),
		                (IXGBE_IMIR_PORT_BP | IXGBE_IMIR_PORT_IM_EN));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIREXT(1),
		                (IXGBE_IMIREXT_SIZE_BP |
		                 IXGBE_IMIREXT_CTRL_PSH));
	}

	if (adapter->lli_size) {
		/* use filter 2 for size */
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIR(2),
		                (IXGBE_IMIR_PORT_BP | IXGBE_IMIR_PORT_IM_EN));
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_IMIREXT(2),
		                (adapter->lli_size | IXGBE_IMIREXT_CTRL_BP));
	}
}

#endif /* IXGBE_NO_LLI */
static void ixgbe_configure(struct ixgbe_adapter *adapter)
{
	ixgbe_set_rx_mode(adapter->netdev);

#ifdef NETIF_F_HW_VLAN_TX
	ixgbe_restore_vlan(adapter);
#endif
	ixgbe_configure_dcb(adapter);

#ifdef IXGBE_FCOE
	if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED)
		ixgbe_configure_fcoe(adapter);

#endif /* IXGBE_FCOE */
	if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE)
		ixgbe_init_fdir_signature_82599(&adapter->hw,
						adapter->fdir_pballoc);
	else if (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE)
		ixgbe_init_fdir_perfect_82599(&adapter->hw,
					      adapter->fdir_pballoc);
	ixgbe_configure_virtualization(adapter);

	ixgbe_configure_tx(adapter);
	ixgbe_configure_rx(adapter);
}

static inline bool ixgbe_is_sfp(struct ixgbe_hw *hw)
{
	switch (hw->phy.type) {
	case ixgbe_phy_sfp_avago:
	case ixgbe_phy_sfp_ftl:
	case ixgbe_phy_sfp_intel:
	case ixgbe_phy_sfp_unknown:
	case ixgbe_phy_sfp_passive_tyco:
	case ixgbe_phy_sfp_passive_unknown:
	case ixgbe_phy_sfp_active_unknown:
	case ixgbe_phy_sfp_ftl_active:
		return true;
	default:
		return false;
	}
}

/**
 * ixgbe_sfp_link_config - set up SFP+ link
 * @adapter: pointer to private adapter struct
 **/
static void ixgbe_sfp_link_config(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;

		if (hw->phy.multispeed_fiber) {
			/*
			 * In multispeed fiber setups, the device may not have
			 * had a physical connection when the driver loaded.
			 * If that's the case, the initial link configuration
			 * couldn't get the MAC into 10G or 1G mode, so we'll
			 * never have a link status change interrupt fire.
			 * We need to try and force an autonegotiation
			 * session, then bring up link.
			 */
			hw->mac.ops.setup_sfp(hw);
			if (!(adapter->flags & IXGBE_FLAG_IN_SFP_LINK_TASK))
				schedule_work(&adapter->multispeed_fiber_task);
		} else {
			/*
			 * Direct Attach Cu and non-multispeed fiber modules
			 * still need to be configured properly prior to
			 * attempting link.
			 */
			if (!(adapter->flags & IXGBE_FLAG_IN_SFP_MOD_TASK))
				schedule_work(&adapter->sfp_config_module_task);
		}
}

/**
 * ixgbe_non_sfp_link_config - set up non-SFP+ link
 * @hw: pointer to private hardware struct
 *
 * Returns 0 on success, negative on failure
 **/
static int ixgbe_non_sfp_link_config(struct ixgbe_hw *hw)
{
	u32 autoneg;
	bool negotiation, link_up = false;
	u32 ret = IXGBE_ERR_LINK_SETUP;

	if (hw->mac.ops.check_link)
		ret = hw->mac.ops.check_link(hw, &autoneg, &link_up, false);

	if (ret)
		goto link_cfg_out;

	autoneg = hw->phy.autoneg_advertised;
	if ((!autoneg) && (hw->mac.ops.get_link_capabilities))
		ret = hw->mac.ops.get_link_capabilities(hw, &autoneg,
		                                        &negotiation);
	if (ret)
		goto link_cfg_out;

	if (hw->mac.ops.setup_link)
		ret = hw->mac.ops.setup_link(hw, autoneg, negotiation, link_up);
link_cfg_out:
	return ret;
}

/**
 * ixgbe_clear_vf_stats_counters - Clear out VF stats after reset
 * @adapter: board private structure
 *
 * On a reset we need to clear out the VF stats or accounting gets
 * messed up because they're not clear on read.
 **/
void ixgbe_clear_vf_stats_counters(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int i;

	for(i = 0; i < adapter->num_vfs; i++) {
		adapter->vfinfo[i].last_vfstats.gprc =
			IXGBE_READ_REG(hw, IXGBE_PVFGPRC(i));
		adapter->vfinfo[i].saved_rst_vfstats.gprc +=
			adapter->vfinfo[i].vfstats.gprc;
		adapter->vfinfo[i].vfstats.gprc = 0;
		adapter->vfinfo[i].last_vfstats.gptc =
			IXGBE_READ_REG(hw, IXGBE_PVFGPTC(i));
		adapter->vfinfo[i].saved_rst_vfstats.gptc +=
			adapter->vfinfo[i].vfstats.gptc;
		adapter->vfinfo[i].vfstats.gptc = 0;
		adapter->vfinfo[i].last_vfstats.gorc =
			IXGBE_READ_REG(hw, IXGBE_PVFGORC_LSB(i));
		adapter->vfinfo[i].saved_rst_vfstats.gorc +=
			adapter->vfinfo[i].vfstats.gorc;
		adapter->vfinfo[i].vfstats.gorc = 0;
		adapter->vfinfo[i].last_vfstats.gotc =
			IXGBE_READ_REG(hw, IXGBE_PVFGOTC_LSB(i));
		adapter->vfinfo[i].saved_rst_vfstats.gotc +=
			adapter->vfinfo[i].vfstats.gotc;
		adapter->vfinfo[i].vfstats.gotc = 0;
		adapter->vfinfo[i].last_vfstats.mprc =
			IXGBE_READ_REG(hw, IXGBE_PVFMPRC(i));
		adapter->vfinfo[i].saved_rst_vfstats.mprc +=
			adapter->vfinfo[i].vfstats.mprc;
		adapter->vfinfo[i].vfstats.mprc = 0;
	}
}

static void ixgbe_setup_gpie(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 gpie = 0;

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		gpie = IXGBE_GPIE_MSIX_MODE | IXGBE_GPIE_PBA_SUPPORT |
		       IXGBE_GPIE_OCD;
#ifdef CONFIG_IXGBE_NAPI
		gpie |= IXGBE_GPIE_EIAME;
		/*
		 * use EIAM to auto-mask when MSI-X interrupt is asserted
		 * this saves a register write for every interrupt
		 */
		switch (hw->mac.type) {
		case ixgbe_mac_82598EB:
			IXGBE_WRITE_REG(hw, IXGBE_EIAM, IXGBE_EICS_RTX_QUEUE);
			break;
		default:
		case ixgbe_mac_82599EB:
			IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(0), 0xFFFFFFFF);
			IXGBE_WRITE_REG(hw, IXGBE_EIAM_EX(1), 0xFFFFFFFF);
			break;
		}
	} else {
		/* legacy interrupts, use EIAM to auto-mask when reading EICR,
		 * specifically only auto mask tx and rx interrupts */
		IXGBE_WRITE_REG(hw, IXGBE_EIAM, IXGBE_EICS_RTX_QUEUE);
#endif
	}

	/* XXX: to interrupt immediately for EICS writes, enable this */
	/* gpie |= IXGBE_GPIE_EIMEN; */

	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED) {
		gpie &= ~IXGBE_GPIE_VTMODE_MASK;
		gpie |= IXGBE_GPIE_VTMODE_64;
	}

	/* Enable Thermal over heat sensor interrupt */
	if (adapter->flags2 & IXGBE_FLAG2_TEMP_SENSOR_CAPABLE)
		gpie |= IXGBE_SDP0_GPIEN;

	/* Enable fan failure interrupt */
	if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE)
		gpie |= IXGBE_SDP1_GPIEN;

	if (hw->mac.type == ixgbe_mac_82599EB)
		gpie |= IXGBE_SDP1_GPIEN;
		gpie |= IXGBE_SDP2_GPIEN;

	IXGBE_WRITE_REG(hw, IXGBE_GPIE, gpie);
#ifdef IXGBE_TCP_TIMER

	if ((adapter->flags & IXGBE_FLAG_MSIX_ENABLED) ||
	    (adapter->flags & IXGBE_FLAG_MSI_ENABLED)) {
		u32 tcp_timer = IXGBE_READ_REG(hw, IXGBE_TCPTIMER);
		tcp_timer |= IXGBE_TCPTIMER_DURATION_MASK;
		tcp_timer |= (IXGBE_TCPTIMER_KS |
		              IXGBE_TCPTIMER_COUNT_ENABLE |
	        	      IXGBE_TCPTIMER_LOOP);
		IXGBE_WRITE_REG(hw, IXGBE_TCPTIMER, tcp_timer);
		tcp_timer = IXGBE_READ_REG(hw, IXGBE_TCPTIMER);
	}
#endif
}

static int ixgbe_up_complete(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int err;
	u32 ctrl_ext;

	ixgbe_get_hw_control(adapter);
	ixgbe_setup_gpie(adapter);

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED)
		ixgbe_configure_msix(adapter);
	else
		ixgbe_configure_msi_and_legacy(adapter);

#ifndef IXGBE_NO_LLI
	ixgbe_configure_lli(adapter);

#endif
	/* enable the optics */
	if (hw->phy.multispeed_fiber)
		ixgbe_enable_tx_laser(hw);

	clear_bit(__IXGBE_DOWN, &adapter->state);
	ixgbe_napi_enable_all(adapter);

	if (ixgbe_is_sfp(hw)) {
		ixgbe_sfp_link_config(adapter);
	} else {
		err = ixgbe_non_sfp_link_config(hw);
		if (err)
			DPRINTK(PROBE, ERR, "link_config FAILED %d\n", err);
	}

	/* clear any pending interrupts, may auto mask */
	IXGBE_READ_REG(hw, IXGBE_EICR);
	ixgbe_irq_enable(adapter, true, true);

	/*
	 * For hot-pluggable SFP+ devices, a SFP+ module may have arrived
	 * before interrupts were enabled but after probe.  Such devices
	 * wouldn't have their type indentified yet.  We need to kick off
	 * the SFP+ module setup first, then try to bring up link.  If we're
	 * not hot-pluggable SFP+, we just need to configure link and bring
	 * it up.
	 */
	if (hw->phy.type == ixgbe_phy_none)
		schedule_work(&adapter->sfp_config_module_task);

	/* enable transmits */
	netif_tx_start_all_queues(adapter->netdev);

	/* bring the link up in the watchdog, this could race with our first
	 * link up interrupt but shouldn't be a problem */
	adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	mod_timer(&adapter->watchdog_timer, jiffies);

	ixgbe_clear_vf_stats_counters(adapter);
	/* Set PF Reset Done bit so PF/VF Mail Ops can work */
	ctrl_ext = IXGBE_READ_REG(hw, IXGBE_CTRL_EXT);
	ctrl_ext |= IXGBE_CTRL_EXT_PFRSTD;
	IXGBE_WRITE_REG(hw, IXGBE_CTRL_EXT, ctrl_ext);

	return 0;
}

void ixgbe_reinit_locked(struct ixgbe_adapter *adapter)
{
	WARN_ON(in_interrupt());
	/* put off any impending NetWatchDogTimeout */
	adapter->netdev->trans_start = jiffies;

	while (test_and_set_bit(__IXGBE_RESETTING, &adapter->state))
		msleep(1);
	ixgbe_down(adapter);
	/*
	 * If SR-IOV enabled then wait a bit before bringing the adapter
	 * back up to give the VFs time to respond to the reset.  The
	 * two second wait is based upon the watchdog timer cycle in
	 * the VF driver.
	 */
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
		msleep(2000);
	ixgbe_up(adapter);
	clear_bit(__IXGBE_RESETTING, &adapter->state);
}

int ixgbe_up(struct ixgbe_adapter *adapter)
{
	int err;

	ixgbe_configure(adapter);

	err = ixgbe_up_complete(adapter);

	return err;
}

void ixgbe_reset(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int err;

	err = hw->mac.ops.init_hw(hw);
	switch (err) {
	case 0:
	case IXGBE_ERR_SFP_NOT_PRESENT:
		break;
	case IXGBE_ERR_MASTER_REQUESTS_PENDING:
		DPRINTK(HW, INFO, "master disable timed out\n");
		break;
	case IXGBE_ERR_EEPROM_VERSION:
		/* We are running on a pre-production device, log a warning */
		DPRINTK(PROBE, INFO, "This device is a pre-production adapter/"
		        "LOM.  Please be aware there may be issues associated "
		        "with your hardware.  If you are experiencing problems "
		        "please contact your Intel or hardware representative "
		        "who provided you with this hardware.\n");
		break;
	default:
		DPRINTK(PROBE, ERR, "Hardware Error: %d\n", err);
	}

	/* reprogram the RAR[0] in case user changed it. */
	if (hw->mac.ops.set_rar)
		hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);
}

/**
 * ixgbe_clean_rx_ring - Free Rx Buffers per Queue
 * @rx_ring: ring to free buffers from
 **/
void ixgbe_clean_rx_ring(struct ixgbe_ring *rx_ring)
{
	struct device *dev = rx_ring->dev;
	unsigned long size;
	u16 i;

	/* ring already cleared, nothing to do */
	if (!rx_ring->rx_buffer_info)
		return;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		struct ixgbe_rx_buffer *rx_buffer_info;

		rx_buffer_info = &rx_ring->rx_buffer_info[i];
		if (rx_buffer_info->dma) {
			dma_unmap_single(dev,
					 rx_buffer_info->dma,
			                 rx_ring->rx_buf_len,
			                 DMA_FROM_DEVICE);
			rx_buffer_info->dma = 0;
		}
		if (rx_buffer_info->skb) {
			struct sk_buff *skb = rx_buffer_info->skb;
			rx_buffer_info->skb = NULL;
			do {
				struct sk_buff *this = skb;
				if (IXGBE_RSC_CB(this)->delay_unmap) {
					dma_unmap_single(dev,
							 IXGBE_RSC_CB(this)->dma,
					                 rx_ring->rx_buf_len,
					                 DMA_FROM_DEVICE);
					IXGBE_RSC_CB(skb)->dma = 0;
					IXGBE_RSC_CB(skb)->delay_unmap = false;
				}
				skb = skb->prev;
				dev_kfree_skb(this);
			} while (skb);
		}
		if (!rx_buffer_info->page)
			continue;
		if (rx_buffer_info->page_dma) {
			dma_unmap_page(dev,
				       rx_buffer_info->page_dma,
				       PAGE_SIZE / 2,
				       DMA_FROM_DEVICE);
		}

		rx_buffer_info->page_dma = 0;
		put_page(rx_buffer_info->page);
		rx_buffer_info->page = NULL;
		rx_buffer_info->page_offset = 0;
	}

	size = sizeof(struct ixgbe_rx_buffer) * rx_ring->count;
	memset(rx_ring->rx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
}

/**
 * ixgbe_clean_tx_ring - Free Tx Buffers
 * @tx_ring: ring to be cleaned
 **/
static void ixgbe_clean_tx_ring(struct ixgbe_ring *tx_ring)
{
	struct ixgbe_tx_buffer *tx_buffer_info;
	unsigned long size;
	u16 i;

	/* ring already cleared, nothing to do */
	if (!tx_ring->tx_buffer_info)
		return;

	/* Free all the Tx ring sk_buffs */
	for (i = 0; i < tx_ring->count; i++) {
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		ixgbe_unmap_and_free_tx_resource(tx_ring, tx_buffer_info);
	}

	size = sizeof(struct ixgbe_tx_buffer) * tx_ring->count;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
}

/**
 * ixgbe_clean_all_rx_rings - Free Rx Buffers for all queues
 * @adapter: board private structure
 **/
static void ixgbe_clean_all_rx_rings(struct ixgbe_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		ixgbe_clean_rx_ring(adapter->rx_ring[i]);
}

/**
 * ixgbe_clean_all_tx_rings - Free Tx Buffers for all queues
 * @adapter: board private structure
 **/
static void ixgbe_clean_all_tx_rings(struct ixgbe_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		ixgbe_clean_tx_ring(adapter->tx_ring[i]);
}

void ixgbe_down(struct ixgbe_adapter *adapter)
{
	struct net_device *netdev = adapter->netdev;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 rxctrl;
	u32 txdctl;
	int i;

	/* signal that we are down to the interrupt handler */
	set_bit(__IXGBE_DOWN, &adapter->state);

	/* disable receives */
	rxctrl = IXGBE_READ_REG(hw, IXGBE_RXCTRL);
	IXGBE_WRITE_REG(hw, IXGBE_RXCTRL, rxctrl & ~IXGBE_RXCTRL_RXEN);

	IXGBE_WRITE_FLUSH(hw);
	msleep(10);

	netif_tx_stop_all_queues(netdev);

	/* call carrier off first to avoid false dev_watchdog timeouts */
	netif_carrier_off(netdev);
	netif_tx_disable(netdev);

	ixgbe_irq_disable(adapter);

	ixgbe_napi_disable_all(adapter);

	clear_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state);
	del_timer_sync(&adapter->sfp_timer);
	del_timer_sync(&adapter->watchdog_timer);

	/* disable receive for all VFs and wait one second */
	if (adapter->num_vfs) {
		/* Mark all the VFs as inactive */
		for (i = 0 ; i < adapter->num_vfs; i++)
			adapter->vfinfo[i].clear_to_send = 0;

		/* ping all the active vfs to let them know we are going down */
		ixgbe_ping_all_vfs(adapter);

		/* Disable all VFTE/VFRE TX/RX */
		ixgbe_disable_tx_rx(adapter);
	}

	/* disable transmits in the hardware now that interrupts are off */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		u8 reg_idx = adapter->tx_ring[i]->reg_idx;
		txdctl = IXGBE_READ_REG(hw, IXGBE_TXDCTL(reg_idx));
		IXGBE_WRITE_REG(hw, IXGBE_TXDCTL(reg_idx),
		                (txdctl & ~IXGBE_TXDCTL_ENABLE));
	}
	/* Disable the Tx DMA engine on 82599 */
	switch (hw->mac.type) {
	case ixgbe_mac_82599EB:
		IXGBE_WRITE_REG(hw, IXGBE_DMATXCTL,
				(IXGBE_READ_REG(hw, IXGBE_DMATXCTL) &
				 ~IXGBE_DMATXCTL_TE));
		break;
	default:
		break;
	}

	/* power down the optics */
	if (hw->phy.multispeed_fiber)
		ixgbe_disable_tx_laser(hw);

#ifdef NETIF_F_NTUPLE
	ethtool_ntuple_flush(netdev);
#endif /* NETIF_F_NTUPLE */

#ifdef HAVE_PCI_ERS
	if (!pci_channel_offline(adapter->pdev))
#endif
		ixgbe_reset(adapter);
	ixgbe_clean_all_tx_rings(adapter);
	ixgbe_clean_all_rx_rings(adapter);

	/* since we reset the hardware DCA settings were cleared */
	ixgbe_setup_dca(adapter);
}

#ifdef CONFIG_IXGBE_NAPI
/**
 * ixgbe_poll - NAPI Rx polling callback
 * @napi: structure for representing this polling device
 * @budget: how many packets driver is allowed to clean
 *
 * This function is used for legacy and MSI, NAPI mode
 **/
static int ixgbe_poll(struct napi_struct *napi, int budget)
{
	struct ixgbe_q_vector *q_vector =
	                        container_of(napi, struct ixgbe_q_vector, napi);
	struct ixgbe_adapter *adapter = q_vector->adapter;
	int tx_clean_complete, work_done = 0;

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED)
		ixgbe_update_dca(q_vector);

	tx_clean_complete = ixgbe_clean_tx_irq(q_vector, adapter->tx_ring[0]);
	ixgbe_clean_rx_irq(q_vector, adapter->rx_ring[0], &work_done, budget);

	if (!tx_clean_complete)
		work_done = budget;

#ifndef HAVE_NETDEV_NAPI_LIST
	if (!netif_running(adapter->netdev))
		work_done = 0;

#endif
	/* If no Tx and not enough Rx work done, exit the polling mode */
	if (work_done < budget) {
		napi_complete(napi);
		if (adapter->rx_itr_setting & 1)
			ixgbe_set_itr(adapter);
		if (!test_bit(__IXGBE_DOWN, &adapter->state))
			ixgbe_irq_enable_queues(adapter, IXGBE_EIMS_RTX_QUEUE);
	}
	return work_done;
}

#endif /* CONFIG_IXGBE_NAPI */
/**
 * ixgbe_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/
static void ixgbe_tx_timeout(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);

	/* Do the reset outside of interrupt context */
	schedule_work(&adapter->reset_task);
}

static void ixgbe_reset_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter;
	adapter = container_of(work, struct ixgbe_adapter, reset_task);

	/* If we're already down or resetting, just bail */
	if (test_bit(__IXGBE_DOWN, &adapter->state) ||
	    test_bit(__IXGBE_RESETTING, &adapter->state))
		return;

	adapter->tx_timeout_count++;

	ixgbe_reinit_locked(adapter);
}


/**
 * ixgbe_set_dcb_queues: Allocate queues for a DCB-enabled device
 * @adapter: board private structure to initialize
 *
 * When DCB (Data Center Bridging) is enabled, allocate queues for
 * each traffic class.  If multiqueue isn't availabe, then abort DCB
 * initialization.
 *
 **/
static inline bool ixgbe_set_dcb_queues(struct ixgbe_adapter *adapter)
{
	bool ret = false;
	struct ixgbe_ring_feature *f = &adapter->ring_feature[RING_F_DCB];

	if (!(adapter->flags & IXGBE_FLAG_DCB_ENABLED))
		return ret;

#ifdef HAVE_TX_MQ
	f->mask = 0x7 << 3;
	adapter->num_rx_queues = f->indices;
	adapter->num_tx_queues = f->indices;
	ret = true;
#else
	DPRINTK(DRV, INFO, "Kernel has no multiqueue support, disabling DCB\n");
	f->mask = 0;
	f->indices = 0;
#endif

	return ret;
}

/**
 * ixgbe_set_vmdq_queues: Allocate queues for VMDq devices
 * @adapter: board private structure to initialize
 *
 * When VMDq (Virtual Machine Devices queue) is enabled, allocate queues
 * and VM pools where appropriate.  If RSS is available, then also try and
 * enable RSS and map accordingly.
 *
 **/
static inline bool ixgbe_set_vmdq_queues(struct ixgbe_adapter *adapter)
{
	int vmdq_i = adapter->ring_feature[RING_F_VMDQ].indices;
	int vmdq_m = 0;
	int rss_i = adapter->ring_feature[RING_F_RSS].indices;
	int rss_m = adapter->ring_feature[RING_F_RSS].mask;
	unsigned long i;
	int rss_shift;
	bool ret = false;

	switch (adapter->flags & (IXGBE_FLAG_RSS_ENABLED
				   | IXGBE_FLAG_DCB_ENABLED
				   | IXGBE_FLAG_VMDQ_ENABLED)) {

	case (IXGBE_FLAG_RSS_ENABLED | IXGBE_FLAG_VMDQ_ENABLED):
		switch (adapter->hw.mac.type) {
		case ixgbe_mac_82599EB:
			vmdq_i = min(IXGBE_MAX_VMDQ_INDICES, vmdq_i);
			if (vmdq_i > 32)
				rss_i = 2;
			else
				rss_i = 4;
			i = rss_i;
			rss_shift = find_first_bit(&i, sizeof(i) * 8);
			rss_m = (rss_i - 1);
			vmdq_m = ((IXGBE_MAX_VMDQ_INDICES - 1) <<
			           rss_shift) & (MAX_RX_QUEUES - 1);
			break;
		default:
			break;
		}
		adapter->num_rx_queues = vmdq_i * rss_i;
		adapter->num_tx_queues = min(MAX_TX_QUEUES, vmdq_i * rss_i);
		ret = true;
		break;

	case (IXGBE_FLAG_VMDQ_ENABLED):
		switch (adapter->hw.mac.type) {
		case ixgbe_mac_82598EB:
			vmdq_m = (IXGBE_MAX_VMDQ_INDICES - 1);
			break;
		case ixgbe_mac_82599EB:
			vmdq_m = (IXGBE_MAX_VMDQ_INDICES - 1) << 1;
			break;
		default:
			break;
		}
		adapter->num_rx_queues = vmdq_i;
		adapter->num_tx_queues = vmdq_i;
		ret = true;
		break;

	default:
		ret = false;
		goto vmdq_queues_out;
	}

	if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED) {
		adapter->num_rx_pools = vmdq_i;
		adapter->num_rx_queues_per_pool = adapter->num_rx_queues /
		                                  vmdq_i;
	} else {
		adapter->num_rx_pools = adapter->num_rx_queues;
		adapter->num_rx_queues_per_pool = 1;
	}
	/* save the mask for later use */
	adapter->ring_feature[RING_F_VMDQ].mask = vmdq_m;
vmdq_queues_out:
	return ret;
}

/**
 * ixgbe_set_rss_queues: Allocate queues for RSS
 * @adapter: board private structure to initialize
 *
 * This is our "base" multiqueue mode.  RSS (Receive Side Scaling) will try
 * to allocate one Rx queue per CPU, and if available, one Tx queue per CPU.
 *
 **/
static inline bool ixgbe_set_rss_queues(struct ixgbe_adapter *adapter)
{
	bool ret = false;
	struct ixgbe_ring_feature *f = &adapter->ring_feature[RING_F_RSS];

	if (adapter->flags & IXGBE_FLAG_RSS_ENABLED) {
		f->mask = 0xF;
		adapter->num_rx_queues = f->indices;
#ifdef HAVE_TX_MQ
		adapter->num_tx_queues = f->indices;
#endif
		ret = true;
	}

	return ret;
}

/**
 * ixgbe_set_fdir_queues: Allocate queues for Flow Director
 * @adapter: board private structure to initialize
 *
 * Flow Director is an advanced Rx filter, attempting to get Rx flows back
 * to the original CPU that initiated the Tx session.  This runs in addition
 * to RSS, so if a packet doesn't match an FDIR filter, we can still spread the
 * Rx load across CPUs using RSS.
 *
 **/
static bool inline ixgbe_set_fdir_queues(struct ixgbe_adapter *adapter)
{
	bool ret = false;
	struct ixgbe_ring_feature *f_fdir = &adapter->ring_feature[RING_F_FDIR];

	f_fdir->indices = min((int)num_online_cpus(), f_fdir->indices);
	f_fdir->mask = 0;

	/*
	 * Use RSS in addition to Flow Director to ensure the best
	 * distribution of flows across cores, even when an FDIR flow
	 * isn't matched.
	 */
	if (adapter->flags & IXGBE_FLAG_RSS_ENABLED &&
	    ((adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE ||
	     (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE)))) {
		adapter->num_rx_queues = f_fdir->indices;
#ifdef HAVE_TX_MQ
		adapter->num_tx_queues = f_fdir->indices;
#endif
		ret = true;
	} else {
		adapter->flags &= ~IXGBE_FLAG_FDIR_HASH_CAPABLE;
		adapter->flags &= ~IXGBE_FLAG_FDIR_PERFECT_CAPABLE;
	}
	return ret;
}

#ifdef IXGBE_FCOE
/**
 * ixgbe_set_fcoe_queues: Allocate queues for Fiber Channel over Ethernet (FCoE)
 * @adapter: board private structure to initialize
 *
 * FCoE RX FCRETA can use up to 8 rx queues for up to 8 different exchanges.
 * The ring feature mask is not used as a mask for FCoE, as it can take any 8
 * rx queues out of the max numberof rx queues, instead, it is used as the
 * index of the first rx queue.
 *
 **/
static inline bool ixgbe_set_fcoe_queues(struct ixgbe_adapter *adapter)
{
	bool ret = false;
	struct ixgbe_ring_feature *f = &adapter->ring_feature[RING_F_FCOE];

	f->indices = min((int)num_online_cpus(), f->indices);
	if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED) {
		adapter->num_rx_queues = 1;
		adapter->num_tx_queues = 1;
		if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
			DPRINTK(PROBE, INFO, "FCoE enabled with DCB \n");
			ixgbe_set_dcb_queues(adapter);
		}
		if (adapter->flags & IXGBE_FLAG_RSS_ENABLED) {
			DPRINTK(PROBE, INFO, "FCoE enabled with RSS \n");
			if ((adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) ||
			    (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE))
				ixgbe_set_fdir_queues(adapter);
			else
				ixgbe_set_rss_queues(adapter);
		}
		/* adding FCoE queues */
		f->mask = adapter->num_rx_queues;
		adapter->num_rx_queues += f->indices;
		adapter->num_tx_queues += f->indices;

		ret = true;
	}

	return ret;
}

#endif /* IXGBE_FCOE */

/**
 * ixgbe_set_sriov_queues: Allocate queues for IOV use
 * @adapter: board private structure to initialize
 *
 * IOV doesn't actually use anything, so just NAK the
 * request for now and let the other queue routines
 * figure out what to do.
 */
static inline bool ixgbe_set_sriov_queues(struct ixgbe_adapter *adapter)
{
	return false;
}

/*
 * ixgbe_set_num_queues: Allocate queues for device, feature dependant
 * @adapter: board private structure to initialize
 *
 * This is the top level queue allocation routine.  The order here is very
 * important, starting with the "most" number of features turned on at once,
 * and ending with the smallest set of features.  This way large combinations
 * can be allocated if they're turned on, and smaller combinations are the
 * fallthrough conditions.
 *
 **/
static void ixgbe_set_num_queues(struct ixgbe_adapter *adapter)
{
	/* Start with base case */
	adapter->num_rx_queues = 1;
	adapter->num_tx_queues = 1;
	adapter->num_rx_pools = adapter->num_rx_queues;
	adapter->num_rx_queues_per_pool = 1;

	if (ixgbe_set_sriov_queues(adapter))
		return;

	if (ixgbe_set_vmdq_queues(adapter))
		return;

#ifdef IXGBE_FCOE
	if (ixgbe_set_fcoe_queues(adapter))
		return;

#endif /* IXGBE_FCOE */
	if (ixgbe_set_dcb_queues(adapter))
		return;

	if (ixgbe_set_fdir_queues(adapter))
		return;


	if (ixgbe_set_rss_queues(adapter))
		return;
}

static void ixgbe_acquire_msix_vectors(struct ixgbe_adapter *adapter,
                                       int vectors)
{
	int err, vector_threshold;

	/* We'll want at least 3 (vector_threshold):
	 * 1) TxQ[0] Cleanup
	 * 2) RxQ[0] Cleanup
	 * 3) Other (Link Status Change, etc.)
	 * 4) TCP Timer (optional)
	 */
	vector_threshold = MIN_MSIX_COUNT;

	/* The more we get, the more we will assign to Tx/Rx Cleanup
	 * for the separate queues...where Rx Cleanup >= Tx Cleanup.
	 * Right now, we simply care about how many we'll get; we'll
	 * set them up later while requesting irq's.
	 */
	while (vectors >= vector_threshold) {
		err = pci_enable_msix(adapter->pdev, adapter->msix_entries,
		                      vectors);
		if (!err) /* Success in acquiring all requested vectors. */
			break;
		else if (err < 0)
			vectors = 0; /* Nasty failure, quit now */
		else /* err == number of vectors we should try again with */
			vectors = err;
	}

	if (vectors < vector_threshold) {
		/* Can't allocate enough MSI-X interrupts?  Oh well.
		 * This just means we'll go with either a single MSI
		 * vector or fall back to legacy interrupts.
		 */
		DPRINTK(HW, DEBUG, "Unable to allocate MSI-X interrupts\n");
		adapter->flags &= ~IXGBE_FLAG_MSIX_ENABLED;
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
	} else {
		adapter->flags |= IXGBE_FLAG_MSIX_ENABLED; /* Woot! */
		/*
		 * Adjust for only the vectors we'll use, which is minimum
		 * of max_msix_q_vectors + NON_Q_VECTORS, or the number of
		 * vectors we were allocated.
		 */
		adapter->num_msix_vectors = min(vectors,
		                   adapter->max_msix_q_vectors + NON_Q_VECTORS);
	}
}

/**
 * ixgbe_cache_ring_rss - Descriptor ring to register mapping for RSS
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for RSS to the assigned rings.
 *
 **/
static inline bool ixgbe_cache_ring_rss(struct ixgbe_adapter *adapter)
{
	int i;

	if (!(adapter->flags & IXGBE_FLAG_RSS_ENABLED))
		return false;

	for (i = 0; i < adapter->num_rx_queues; i++)
		adapter->rx_ring[i]->reg_idx = i;
	for (i = 0; i < adapter->num_tx_queues; i++)
		adapter->tx_ring[i]->reg_idx = i;

	return true;
}

/**
 * ixgbe_cache_ring_dcb - Descriptor ring to register mapping for DCB
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for DCB to the assigned rings.
 *
 **/
static inline bool ixgbe_cache_ring_dcb(struct ixgbe_adapter *adapter)
{
	int i;
	bool ret = false;
	int dcb_i = adapter->ring_feature[RING_F_DCB].indices;

	if (!(adapter->flags & IXGBE_FLAG_DCB_ENABLED))
		return false;

	/* the number of queues is assumed to be symmetric */
	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82598EB:
		for (i = 0; i < dcb_i; i++) {
			adapter->rx_ring[i]->reg_idx = i << 3;
			adapter->tx_ring[i]->reg_idx = i << 2;
		}
		ret = true;
		break;
	case ixgbe_mac_82599EB:
		if (dcb_i == 8) {
			/*
			 * Tx TC0 starts at: descriptor queue 0
			 * Tx TC1 starts at: descriptor queue 32
			 * Tx TC2 starts at: descriptor queue 64
			 * Tx TC3 starts at: descriptor queue 80
			 * Tx TC4 starts at: descriptor queue 96
			 * Tx TC5 starts at: descriptor queue 104
			 * Tx TC6 starts at: descriptor queue 112
			 * Tx TC7 starts at: descriptor queue 120
			 *
			 * Rx TC0-TC7 are offset by 16 queues each
			 */
			for (i = 0; i < 3; i++) {
				adapter->tx_ring[i]->reg_idx = i << 5;
				adapter->rx_ring[i]->reg_idx = i << 4;
			}
			for ( ; i < 5; i++) {
				adapter->tx_ring[i]->reg_idx = ((i + 2) << 4);
				adapter->rx_ring[i]->reg_idx = i << 4;
			}
			for ( ; i < dcb_i; i++) {
				adapter->tx_ring[i]->reg_idx = ((i + 8) << 3);
				adapter->rx_ring[i]->reg_idx = i << 4;
			}
			ret = true;
		} else if (dcb_i == 4) {
			/*
			 * Tx TC0 starts at: descriptor queue 0
			 * Tx TC1 starts at: descriptor queue 64
			 * Tx TC2 starts at: descriptor queue 96
			 * Tx TC3 starts at: descriptor queue 112
			 *
			 * Rx TC0-TC3 are offset by 32 queues each
			 */
			adapter->tx_ring[0]->reg_idx = 0;
			adapter->tx_ring[1]->reg_idx = 64;
			adapter->tx_ring[2]->reg_idx = 96;
			adapter->tx_ring[3]->reg_idx = 112;
			for (i = 0 ; i < dcb_i; i++)
				adapter->rx_ring[i]->reg_idx = i << 5;
			ret = true;
		}
		break;
	default:
		break;
	}
	return ret;
}

/**
 * ixgbe_cache_ring_vmdq - Descriptor ring to register mapping for VMDq
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for VMDq to the assigned rings.  It
 * will also try to cache the proper offsets if RSS is enabled along with
 * VMDq.
 *
 **/
static inline bool ixgbe_cache_ring_vmdq(struct ixgbe_adapter *adapter)
{
	int i;
	bool ret = false;
#ifdef IXGBE_FCOE
	struct ixgbe_ring_feature *f = &adapter->ring_feature[RING_F_FCOE];
#endif /* IXGBE_FCOE */
	switch (adapter->flags & (IXGBE_FLAG_RSS_ENABLED
				   | IXGBE_FLAG_DCB_ENABLED
				   | IXGBE_FLAG_VMDQ_ENABLED)) {

	case (IXGBE_FLAG_RSS_ENABLED | IXGBE_FLAG_VMDQ_ENABLED):
		switch (adapter->hw.mac.type) {
		case ixgbe_mac_82599EB:
			/* since the # of rss queues per vmdq pool is
			 * limited to either 2 or 4, there is no index
			 * skipping and we can set them up with no
			 * funky mapping
			 */
			for (i = 0; i < adapter->num_rx_queues; i++)
				adapter->rx_ring[i]->reg_idx = i;
			for (i = 0; i < adapter->num_tx_queues; i++)
				adapter->tx_ring[i]->reg_idx = i;
			ret = true;
			break;
		default:
			break;
		}
		break;

	case (IXGBE_FLAG_VMDQ_ENABLED | IXGBE_FLAG_DCB_ENABLED):
		if (adapter->hw.mac.type == ixgbe_mac_82599EB) {
			for (i = 0; i < adapter->num_rx_queues; i++) {
				adapter->rx_ring[i]->reg_idx =
					(adapter->num_vfs + i) *
					 adapter->ring_feature[RING_F_DCB].indices;
#ifdef IXGBE_FCOE
				adapter->rx_ring[i]->reg_idx +=
					(i >= f->mask ? adapter->fcoe.tc : 0);
#endif /* IXGBE_FCOE */
			}

			for (i = 0; i < adapter->num_tx_queues; i++) {
				adapter->tx_ring[i]->reg_idx =
					(adapter->num_vfs + i) *
					 adapter->ring_feature[RING_F_DCB].indices;
#ifdef IXGBE_FCOE
				adapter->tx_ring[i]->reg_idx +=
					(i >= f->mask ? adapter->fcoe.tc : 0);
#endif /* IXGBE_FCOE */
			}
			ret = true;
		}
		break;

	case (IXGBE_FLAG_VMDQ_ENABLED):
		switch (adapter->hw.mac.type) {
		case ixgbe_mac_82598EB:
			for (i = 0; i < adapter->num_rx_queues; i++)
				adapter->rx_ring[i]->reg_idx = i;
			for (i = 0; i < adapter->num_tx_queues; i++)
				adapter->tx_ring[i]->reg_idx = i;
			ret = true;
			break;
		case ixgbe_mac_82599EB:
			/* even without rss, there are 2 queues per
			 * pool, the odd numbered ones are unused.
			 */
			for (i = 0; i < adapter->num_rx_queues; i++)
				adapter->rx_ring[i]->reg_idx = VMDQ_P(i) * 2;
			for (i = 0; i < adapter->num_tx_queues; i++)
				adapter->tx_ring[i]->reg_idx = VMDQ_P(i) * 2;
			ret = true;
			break;
		default:
			break;
		}
		break;
	}

	return ret;
}

/**
 * ixgbe_cache_ring_fdir - Descriptor ring to register mapping for Flow Director
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for Flow Director to the assigned rings.
 *
 **/
static bool inline ixgbe_cache_ring_fdir(struct ixgbe_adapter *adapter)
{
	int i;
	bool ret = false;

	if (adapter->flags & IXGBE_FLAG_RSS_ENABLED &&
	    ((adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) ||
	     (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE))) {
		for (i = 0; i < adapter->num_rx_queues; i++)
			adapter->rx_ring[i]->reg_idx = i;
		for (i = 0; i < adapter->num_tx_queues; i++)
			adapter->tx_ring[i]->reg_idx = i;
		ret = true;
	}

	return ret;
}

#ifdef IXGBE_FCOE
/**
 * ixgbe_cache_ring_fcoe - Descriptor ring to register mapping for the FCoE
 * @adapter: board private structure to initialize
 *
 * Cache the descriptor ring offsets for FCoE mode to the assigned rings.
 *
 */
static inline bool ixgbe_cache_ring_fcoe(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ring_feature *f = &adapter->ring_feature[RING_F_FCOE];
	int i;
	u8 fcoe_rx_i = 0, fcoe_tx_i = 0;

	if (!(adapter->flags & IXGBE_FLAG_FCOE_ENABLED))
		return false;

	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
		struct ixgbe_fcoe *fcoe = &adapter->fcoe;

		ixgbe_cache_ring_dcb(adapter);
		/* find out queues in TC for FCoE */
		fcoe_rx_i = adapter->rx_ring[fcoe->tc]->reg_idx + 1;
		fcoe_tx_i = adapter->tx_ring[fcoe->tc]->reg_idx + 1;
		/*
		 * In 82599, the number of Tx queues for each traffic
		 * class for both 8-TC and 4-TC modes are:
		 * TCs  : TC0 TC1 TC2 TC3 TC4 TC5 TC6 TC7
		 * 8 TCs:  32  32  16  16   8   8   8   8
		 * 4 TCs:  64  64  32  32
		 * We have max 8 queues for FCoE, where 8 the is
		 * FCoE redirection table size. If TC for FCoE is
		 * less than or equal to TC3, we have enough queues
		 * to add max of 8 queues for FCoE, so we start FCoE
		 * tx queue from the next one, i.e., reg_idx + 1.
		 * If TC for FCoE is above TC3, implying 8 TC mode,
		 * and we need 8 for FCoE, we have to take all queues
		 * in that traffic class for FCoE.
		 */
		if ((f->indices == IXGBE_FCRETA_SIZE) && (fcoe->tc > 3))
			fcoe_tx_i--;
	}
	if (adapter->flags & IXGBE_FLAG_RSS_ENABLED) {
		if ((adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) ||
			(adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE))
			ixgbe_cache_ring_fdir(adapter);
		else
			ixgbe_cache_ring_rss(adapter);

		fcoe_rx_i = f->mask;
		fcoe_tx_i = f->mask;
	}
	for (i = 0; i < f->indices; i++, fcoe_rx_i++, fcoe_tx_i++) {
		adapter->rx_ring[f->mask + i]->reg_idx = fcoe_rx_i;
		adapter->tx_ring[f->mask + i]->reg_idx = fcoe_tx_i;
	}
	return true;
}

#endif /* IXGBE_FCOE */
/**
 * ixgbe_cache_ring_sriov - Descriptor ring to register mapping for sriov
 * @adapter: board private structure to initialize
 *
 * SR-IOV doesn't use any descriptor rings but changes the default if
 * no other mapping is used.
 *
 */
static inline bool ixgbe_cache_ring_sriov(struct ixgbe_adapter *adapter)
{
	adapter->rx_ring[0]->reg_idx = adapter->num_vfs * 2;
	adapter->tx_ring[0]->reg_idx = adapter->num_vfs * 2;
	return false;
}

/**
 * ixgbe_cache_ring_register - Descriptor ring to register mapping
 * @adapter: board private structure to initialize
 *
 * Once we know the feature-set enabled for the device, we'll cache
 * the register offset the descriptor ring is assigned to.
 *
 * Note, the order the various feature calls is important.  It must start with
 * the "most" features enabled at the same time, then trickle down to the
 * least amount of features turned on at once.
 **/
static void ixgbe_cache_ring_register(struct ixgbe_adapter *adapter)
{
	/* start with default case */
	adapter->rx_ring[0]->reg_idx = 0;
	adapter->tx_ring[0]->reg_idx = 0;

	if (ixgbe_cache_ring_sriov(adapter))
		return;

	if (ixgbe_cache_ring_vmdq(adapter))
		return;

#ifdef IXGBE_FCOE
	if (ixgbe_cache_ring_fcoe(adapter))
		return;

#endif /* IXGBE_FCOE */
	if (ixgbe_cache_ring_dcb(adapter))
		return;

	if (ixgbe_cache_ring_fdir(adapter))
		return;

	if (ixgbe_cache_ring_rss(adapter))
		return;

}

/**
 * ixgbe_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 * We allocate one ring per queue at run-time since we don't know the
 * number of queues at compile-time.  The polling_netdev array is
 * intended for Multiqueue, but should work fine with a single queue.
 **/
static int ixgbe_alloc_queues(struct ixgbe_adapter *adapter)
{
	int i;
	int rx_count;
#ifdef HAVE_DEVICE_NUMA_NODE
	int orig_node = adapter->node;

	WARN_ON(orig_node != -1 && !node_online(orig_node));
#endif /* HAVE_DEVICE_NUMA_NODE */

	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct ixgbe_ring *ring = adapter->tx_ring[i];
#ifdef HAVE_DEVICE_NUMA_NODE
		if (orig_node == -1) {
			int cur_node = next_online_node(adapter->node);
			if (cur_node == MAX_NUMNODES)
				cur_node = first_online_node;
			adapter->node = cur_node;
		}
#endif /* HAVE_DEVICE_NUMA_NODE */
		ring = kzalloc_node(sizeof(struct ixgbe_ring), GFP_KERNEL,
		                    adapter->node);
		if (!ring)
			ring = kzalloc(sizeof(struct ixgbe_ring), GFP_KERNEL);
		if (!ring)
			goto err_tx_ring_allocation;
		ring->count = adapter->tx_ring_count;
		ring->queue_index = i;
		ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;
		ring->numa_node = adapter->node;
		if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE)
			ring->atr_sample_rate = adapter->atr_sample_rate;
		ring->atr_count = 0;

		adapter->tx_ring[i] = ring;
	}

#ifdef HAVE_DEVICE_NUMA_NODE
	/* Restore the adapter's original node */
	adapter->node = orig_node;

#endif /* HAVE_DEVICE_NUMA_NODE */
	rx_count = adapter->rx_ring_count;
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct ixgbe_ring *ring = adapter->rx_ring[i];
#ifdef HAVE_DEVICE_NUMA_NODE
		if (orig_node == -1) {
			int cur_node = next_online_node(adapter->node);
			if (cur_node == MAX_NUMNODES)
				cur_node = first_online_node;
			adapter->node = cur_node;
		}
#endif /* HAVE_DEVICE_NUMA_NODE */
		ring = kzalloc_node(sizeof(struct ixgbe_ring), GFP_KERNEL,
		                    adapter->node);
		if (!ring)
			ring = kzalloc(sizeof(struct ixgbe_ring), GFP_KERNEL);
		if (!ring)
			goto err_rx_ring_allocation;
		ring->count = rx_count;
		ring->queue_index = i;
		ring->dev = pci_dev_to_dev(adapter->pdev);
		ring->netdev = adapter->netdev;
		ring->numa_node = adapter->node;

		adapter->rx_ring[i] = ring;
	}

#ifdef HAVE_DEVICE_NUMA_NODE
	/* Restore the adapter's original node */
	adapter->node = orig_node;

#endif /* HAVE_DEVICE_NUMA_NODE */
	ixgbe_cache_ring_register(adapter);

	return 0;

err_rx_ring_allocation:
	for (i = 0; i < adapter->num_tx_queues; i++)
		kfree(adapter->tx_ring[i]);
err_tx_ring_allocation:
	return -ENOMEM;
}

/**
 * ixgbe_set_interrupt_capability - set MSI-X or MSI if supported
 * @adapter: board private structure to initialize
 *
 * Attempt to configure the interrupts using the best available
 * capabilities of the hardware and the kernel.
 **/
static int ixgbe_set_interrupt_capability(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	int err = 0;
	int vector, v_budget;

	if (!(adapter->flags & IXGBE_FLAG_MSIX_CAPABLE))
		goto try_msi;

	/*
	 * It's easy to be greedy for MSI-X vectors, but it really
	 * doesn't do us much good if we have a lot more vectors
	 * than CPU's.  So let's be conservative and only ask for
	 * (roughly) the same number of vectors as there are CPU's.
	 */
	v_budget = min(adapter->num_rx_queues + adapter->num_tx_queues,
	               (int)num_online_cpus()) + NON_Q_VECTORS;

	/*
	 * At the same time, hardware can only support a maximum of
	 * hw.mac->max_msix_vectors vectors.  With features
	 * such as RSS and VMDq, we can easily surpass the number of Rx and Tx
	 * descriptor queues supported by our device.  Thus, we cap it off in
	 * those rare cases where the cpu count also exceeds our vector limit.
	 */
	v_budget = min(v_budget, (int)hw->mac.max_msix_vectors);

	/* A failure in MSI-X entry allocation isn't fatal, but it does
	 * mean we disable MSI-X capabilities of the adapter. */
	adapter->msix_entries = kcalloc(v_budget,
	                                sizeof(struct msix_entry), GFP_KERNEL);
	if (adapter->msix_entries) {
		for (vector = 0; vector < v_budget; vector++)
			adapter->msix_entries[vector].entry = vector;

		ixgbe_acquire_msix_vectors(adapter, v_budget);

		if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED)
			goto out;
	}

	adapter->flags &= ~IXGBE_FLAG_DCB_ENABLED;
	adapter->flags &= ~IXGBE_FLAG_DCB_CAPABLE;
	adapter->flags &= ~IXGBE_FLAG_FDIR_HASH_CAPABLE;
	adapter->flags &= ~IXGBE_FLAG_FDIR_PERFECT_CAPABLE;
	adapter->atr_sample_rate = 0;
	adapter->flags &= ~IXGBE_FLAG_VMDQ_ENABLED;
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED) {
		DPRINTK(PROBE, ERR, "MSIX interrupt not available - "
			"disabling SR-IOV\n");
		ixgbe_disable_sriov(adapter);
	}

	adapter->flags &= ~IXGBE_FLAG_RSS_ENABLED;
	ixgbe_set_num_queues(adapter);

try_msi:
	if (!(adapter->flags & IXGBE_FLAG_MSI_CAPABLE))
		goto out;

	err = pci_enable_msi(adapter->pdev);
	if (!err) {
		adapter->flags |= IXGBE_FLAG_MSI_ENABLED;
	} else {
		DPRINTK(HW, DEBUG, "Unable to allocate MSI interrupt, "
		                   "falling back to legacy.  Error: %d\n", err);
		/* reset err */
		err = 0;
	}

out:
#ifdef HAVE_TX_MQ
	/* Notify the stack of the (possibly) reduced Tx Queue count. */
#ifdef CONFIG_NETDEVICES_MULTIQUEUE
	adapter->netdev->egress_subqueue_count = adapter->num_tx_queues;
#else
	adapter->netdev->real_num_tx_queues = adapter->num_tx_queues;
#endif
#endif /* HAVE_TX_MQ */
	return err;
}

/**
 * ixgbe_alloc_q_vectors - Allocate memory for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * We allocate one q_vector per queue interrupt.  If allocation fails we
 * return -ENOMEM.
 **/
static int ixgbe_alloc_q_vectors(struct ixgbe_adapter *adapter)
{
	int v_idx, num_q_vectors;
	struct ixgbe_q_vector *q_vector;
	int rx_vectors;
#ifdef CONFIG_IXGBE_NAPI
	int (*poll)(struct napi_struct *, int);
#endif

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		num_q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;
		rx_vectors = adapter->num_rx_queues;
#ifdef CONFIG_IXGBE_NAPI
		poll = &ixgbe_clean_rxtx_many;
#endif
	} else {
		num_q_vectors = 1;
		rx_vectors = 1;
#ifdef CONFIG_IXGBE_NAPI
		poll = &ixgbe_poll;
#endif
	}

	for (v_idx = 0; v_idx < num_q_vectors; v_idx++) {
		q_vector = kzalloc_node(sizeof(struct ixgbe_q_vector),
		                        GFP_KERNEL, adapter->node);
		if (!q_vector)
			q_vector = kzalloc(sizeof(struct ixgbe_q_vector),
			                   GFP_KERNEL);
		if (!q_vector)
			goto err_out;
		q_vector->adapter = adapter;
		if (q_vector->txr_count && !q_vector->rxr_count)
			q_vector->eitr = adapter->tx_eitr_param;
		else
			q_vector->eitr = adapter->rx_eitr_param;
		q_vector->v_idx = v_idx;
#ifndef IXGBE_NO_LRO
		if (v_idx < rx_vectors) {
			int size = sizeof(struct ixgbe_lro_list);
			q_vector->lrolist = vmalloc_node(size, adapter->node);
			if (!q_vector->lrolist)
				q_vector->lrolist = vmalloc(size);
			if (!q_vector->lrolist) {
				kfree(q_vector);
				goto err_out;
			}
			memset(q_vector->lrolist, 0, size);
			ixgbe_lro_ring_init(q_vector->lrolist);
		}
#endif
#ifdef CONFIG_IXGBE_NAPI
		netif_napi_add(adapter->netdev, &q_vector->napi, (*poll), 64);
#endif
		adapter->q_vector[v_idx] = q_vector;
	}

	return 0;

err_out:
	while (v_idx) {
		v_idx--;
		q_vector = adapter->q_vector[v_idx];
#ifdef CONFIG_IXGBE_NAPI
			netif_napi_del(&q_vector->napi);
#endif
#ifndef IXGBE_NO_LRO
		if (q_vector->lrolist) {
			ixgbe_lro_ring_exit(q_vector->lrolist);
			vfree(q_vector->lrolist);
			q_vector->lrolist = NULL;
		}
#endif
		kfree(q_vector);
		adapter->q_vector[v_idx] = NULL;
	}
	return -ENOMEM;
}

/**
 * ixgbe_free_q_vectors - Free memory allocated for interrupt vectors
 * @adapter: board private structure to initialize
 *
 * This function frees the memory allocated to the q_vectors.  In addition if
 * NAPI is enabled it will delete any references to the NAPI struct prior
 * to freeing the q_vector.
 **/
static void ixgbe_free_q_vectors(struct ixgbe_adapter *adapter)
{
	int v_idx, num_q_vectors;

	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		num_q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;
	} else {
		num_q_vectors = 1;
	}

	for (v_idx = 0; v_idx < num_q_vectors; v_idx++) {
		struct ixgbe_q_vector *q_vector = adapter->q_vector[v_idx];

		adapter->q_vector[v_idx] = NULL;
#ifdef CONFIG_IXGBE_NAPI
		netif_napi_del(&q_vector->napi);
#endif
#ifndef IXGBE_NO_LRO
		if (q_vector->lrolist) {
			ixgbe_lro_ring_exit(q_vector->lrolist);
			vfree(q_vector->lrolist);
			q_vector->lrolist = NULL;
		}
#endif
		kfree(q_vector);
	}
}

static void ixgbe_reset_interrupt_capability(struct ixgbe_adapter *adapter)
{
	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		adapter->flags &= ~IXGBE_FLAG_MSIX_ENABLED;
		pci_disable_msix(adapter->pdev);
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;
	} else if (adapter->flags & IXGBE_FLAG_MSI_ENABLED) {
		adapter->flags &= ~IXGBE_FLAG_MSI_ENABLED;
		pci_disable_msi(adapter->pdev);
	}
	return;
}

/**
 * ixgbe_init_interrupt_scheme - Determine proper interrupt scheme
 * @adapter: board private structure to initialize
 *
 * We determine which interrupt scheme to use based on...
 * - Kernel support (MSI, MSI-X)
 *   - which can be user-defined (via MODULE_PARAM)
 * - Hardware queue count (num_*_queues)
 *   - defined by miscellaneous hardware support/features (RSS, etc.)
 **/
int ixgbe_init_interrupt_scheme(struct ixgbe_adapter *adapter)
{
	int err;

	/* Number of supported queues */
	ixgbe_set_num_queues(adapter);

	err = ixgbe_set_interrupt_capability(adapter);
	if (err) {
		DPRINTK(PROBE, ERR, "Unable to setup interrupt capabilities\n");
		goto err_set_interrupt;
	}

	err = ixgbe_alloc_q_vectors(adapter);
	if (err) {
		DPRINTK(PROBE, ERR, "Unable to allocate memory for queue "
		        "vectors\n");
		goto err_alloc_q_vectors;
	}

	err = ixgbe_alloc_queues(adapter);
	if (err) {
		DPRINTK(PROBE, ERR, "Unable to allocate memory for queues\n");
		goto err_alloc_queues;
	}

	DPRINTK(DRV, INFO, "Multiqueue %s: Rx Queue count = %u, "
	                   "Tx Queue count = %u\n",
	        (adapter->num_rx_queues > 1) ? "Enabled" :
	        "Disabled", adapter->num_rx_queues, adapter->num_tx_queues);

	set_bit(__IXGBE_DOWN, &adapter->state);

	return 0;
err_alloc_queues:
	ixgbe_free_q_vectors(adapter);
err_alloc_q_vectors:
	ixgbe_reset_interrupt_capability(adapter);
err_set_interrupt:
	return err;
}

/**
 * ixgbe_clear_interrupt_scheme - Clear the current interrupt scheme settings
 * @adapter: board private structure to clear interrupt scheme on
 *
 * We go through and clear interrupt specific resources and reset the structure
 * to pre-load conditions
 **/
void ixgbe_clear_interrupt_scheme(struct ixgbe_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++) {
		kfree(adapter->tx_ring[i]);
		adapter->tx_ring[i] = NULL;
	}
	for (i = 0; i < adapter->num_rx_queues; i++) {
		kfree(adapter->rx_ring[i]);
		adapter->rx_ring[i] = NULL;
	}

	ixgbe_free_q_vectors(adapter);
	ixgbe_reset_interrupt_capability(adapter);
}

/**
 * ixgbe_sfp_timer - worker thread to find a missing module
 * @data: pointer to our adapter struct
 **/
static void ixgbe_sfp_timer(unsigned long data)
{
	struct ixgbe_adapter *adapter = (struct ixgbe_adapter *)data;

	/* Do the sfp_timer outside of interrupt context due to the
	 * delays that sfp+ detection requires */
	schedule_work(&adapter->sfp_task);
}

/**
 * ixgbe_sfp_task - worker thread to find a missing module
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_sfp_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             sfp_task);
	struct ixgbe_hw *hw = &adapter->hw;

	if ((hw->phy.type == ixgbe_phy_nl) &&
	    (hw->phy.sfp_type == ixgbe_sfp_type_not_present)) {
		s32 ret = hw->phy.ops.identify_sfp(hw);
		if (ret && ret != IXGBE_ERR_SFP_NOT_SUPPORTED)
			goto reschedule;
		ret = hw->phy.ops.reset(hw);
		if (ret == IXGBE_ERR_SFP_NOT_SUPPORTED) {
			DPRINTK(PROBE, ERR, "failed to initialize because an "
			        "unsupported SFP+ module type was detected.\n"
			        "Reload the driver after installing a "
			        "supported module.\n");
			unregister_netdev(adapter->netdev);
			adapter->netdev_registered = false;
		} else {
			DPRINTK(PROBE, INFO, "detected SFP+: %d\n",
			        hw->phy.sfp_type);
		}
		/* don't need this routine any more */
		clear_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state);
	}
	return;
reschedule:
	if (test_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state))
		mod_timer(&adapter->sfp_timer,
		          round_jiffies(jiffies + (2 * HZ)));
}

/**
 * ixgbe_sw_init - Initialize general software structures (struct ixgbe_adapter)
 * @adapter: board private structure to initialize
 *
 * ixgbe_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 **/
static int __devinit ixgbe_sw_init(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	struct pci_dev *pdev = adapter->pdev;
	int err;

	/* PCI config space info */

	hw->vendor_id = pdev->vendor;
	hw->device_id = pdev->device;
	pci_read_config_byte(pdev, PCI_REVISION_ID, &hw->revision_id);
	hw->subsystem_vendor_id = pdev->subsystem_vendor;
	hw->subsystem_device_id = pdev->subsystem_device;

	err = ixgbe_init_shared_code(hw);
	if (err) {
		DPRINTK(PROBE, ERR, "init_shared_code failed: %d\n", err);
		goto out;
	}

	/* Set capability flags */
	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		if (hw->device_id == IXGBE_DEV_ID_82598AT)
			adapter->flags |= IXGBE_FLAG_FAN_FAIL_CAPABLE;
#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
		adapter->flags |= IXGBE_FLAG_DCA_CAPABLE;
#endif
		adapter->flags |= IXGBE_FLAG_MSI_CAPABLE;
		adapter->flags |= IXGBE_FLAG_MSIX_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MSIX_CAPABLE)
			adapter->flags |= IXGBE_FLAG_MQ_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_DCB_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_RSS_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_VMDQ_CAPABLE;
		adapter->flags2 &= ~IXGBE_FLAG2_RSC_CAPABLE;
		adapter->flags &= ~IXGBE_FLAG_SRIOV_CAPABLE;
		adapter->max_msix_q_vectors = IXGBE_MAX_MSIX_Q_VECTORS_82598;
		break;
	case ixgbe_mac_82599EB:
#ifndef IXGBE_NO_SMART_SPEED
		hw->phy.smart_speed = ixgbe_smart_speed_on;
#else
		hw->phy.smart_speed = ixgbe_smart_speed_off;
#endif
		adapter->flags2 |= IXGBE_FLAG2_RSC_CAPABLE;
#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
		adapter->flags |= IXGBE_FLAG_DCA_CAPABLE;
#endif
		adapter->flags |= IXGBE_FLAG_MSI_CAPABLE;
		adapter->flags |= IXGBE_FLAG_MSIX_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MSIX_CAPABLE)
			adapter->flags |= IXGBE_FLAG_MQ_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_DCB_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_RSS_CAPABLE;
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_VMDQ_CAPABLE;
#ifdef IXGBE_FCOE
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE) {
			adapter->flags |= IXGBE_FLAG_FCOE_CAPABLE;
			adapter->flags &= ~IXGBE_FLAG_FCOE_ENABLED;
			adapter->ring_feature[RING_F_FCOE].indices = 0;
#ifdef CONFIG_DCB
			/* Default traffic class to use for FCoE */
			adapter->fcoe.tc = IXGBE_FCOE_DEFTC;
			adapter->fcoe.up = IXGBE_FCOE_DEFTC;
#endif
		}
#endif
		if (adapter->flags & IXGBE_FLAG_MQ_CAPABLE)
			adapter->flags |= IXGBE_FLAG_SRIOV_CAPABLE;
		if (hw->device_id == IXGBE_DEV_ID_82599_T3_LOM)
			adapter->flags2 |= IXGBE_FLAG2_TEMP_SENSOR_CAPABLE;
#ifdef NETIF_F_NTUPLE
		/* n-tuple support exists, always init our spinlock */
		spin_lock_init(&adapter->fdir_perfect_lock);
#endif /* NETIF_F_NTUPLE */
		adapter->max_msix_q_vectors = IXGBE_MAX_MSIX_Q_VECTORS_82599;
		break;
	default:
		break;
	}

	/* Default DCB settings, if applicable */
	adapter->ring_feature[RING_F_DCB].indices = 8;

	if (adapter->flags & IXGBE_FLAG_DCB_CAPABLE) {
		int j, dcb_i;
		struct tc_configuration *tc;
		dcb_i = adapter->ring_feature[RING_F_DCB].indices;
		adapter->dcb_cfg.num_tcs.pg_tcs = dcb_i;
		adapter->dcb_cfg.num_tcs.pfc_tcs = dcb_i;
		for (j = 0; j < dcb_i; j++) {
			tc = &adapter->dcb_cfg.tc_config[j];
			tc->path[DCB_TX_CONFIG].bwg_id = 0;
			tc->path[DCB_TX_CONFIG].bwg_percent = 100 / dcb_i;
			tc->path[DCB_RX_CONFIG].bwg_id = 0;
			tc->path[DCB_RX_CONFIG].bwg_percent = 100 / dcb_i;
			tc->dcb_pfc = pfc_disabled;
			if (j == 0) {
				/* total of all TCs bandwidth needs to be 100 */
				tc->path[DCB_TX_CONFIG].bwg_percent += 100 % dcb_i;
				tc->path[DCB_RX_CONFIG].bwg_percent += 100 % dcb_i;
			}
		}
		adapter->dcb_cfg.bw_percentage[DCB_TX_CONFIG][0] = 100;
		adapter->dcb_cfg.bw_percentage[DCB_RX_CONFIG][0] = 100;
		adapter->dcb_cfg.rx_pba_cfg = pba_equal;
		adapter->dcb_cfg.pfc_mode_enable = false;

		adapter->dcb_cfg.round_robin_enable = false;
		adapter->dcb_set_bitmap = 0x00;

	}
	/* XXX does this need to be initialized even w/o DCB? */
	ixgbe_copy_dcb_cfg(&adapter->dcb_cfg, &adapter->temp_dcb_cfg,
	                   adapter->ring_feature[RING_F_DCB].indices);

	if (hw->mac.type == ixgbe_mac_82599EB)
		hw->mbx.ops.init_params(hw);

	/* default flow control settings */
	hw->fc.requested_mode = ixgbe_fc_full;
	hw->fc.current_mode = ixgbe_fc_full;	/* init for ethtool output */

	adapter->last_lfc_mode = hw->fc.current_mode;
	hw->fc.high_water = IXGBE_DEFAULT_FCRTH;
	hw->fc.low_water = IXGBE_DEFAULT_FCRTL;
	hw->fc.pause_time = IXGBE_DEFAULT_FCPAUSE;
	hw->fc.send_xon = true;
	hw->fc.disable_fc_autoneg = false;

	/* set defaults for eitr in MegaBytes */
	adapter->eitr_low = 10;
	adapter->eitr_high = 20;

	/* set default ring sizes */
	adapter->tx_ring_count = IXGBE_DEFAULT_TXD;
	adapter->rx_ring_count = IXGBE_DEFAULT_RXD;

	/* enable rx csum by default */
	adapter->flags |= IXGBE_FLAG_RX_CSUM_ENABLED;

	set_bit(__IXGBE_DOWN, &adapter->state);
out:
	return err;
}

/**
 * ixgbe_setup_tx_resources - allocate Tx resources (Descriptors)
 * @tx_ring:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
int ixgbe_setup_tx_resources(struct ixgbe_ring *tx_ring)
{
	struct device *dev = tx_ring->dev;
	int size;

	size = sizeof(struct ixgbe_tx_buffer) * tx_ring->count;
	tx_ring->tx_buffer_info = vmalloc_node(size, tx_ring->numa_node);
	if (!tx_ring->tx_buffer_info)
		tx_ring->tx_buffer_info = vmalloc(size);
	if (!tx_ring->tx_buffer_info)
		goto err;
	memset(tx_ring->tx_buffer_info, 0, size);

	/* round up to nearest 4K */
	tx_ring->size = tx_ring->count * sizeof(union ixgbe_adv_tx_desc);
	tx_ring->size = ALIGN(tx_ring->size, 4096);

	tx_ring->desc = dma_alloc_coherent(dev, tx_ring->size,
	                                   &tx_ring->dma, GFP_KERNEL);
	if (!tx_ring->desc)
		goto err;

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	tx_ring->work_limit = tx_ring->count;
	return 0;

err:
	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Tx descriptor ring\n");
	return -ENOMEM;
}

/**
 * ixgbe_setup_all_tx_resources - allocate all queues Tx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int ixgbe_setup_all_tx_resources(struct ixgbe_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_tx_queues; i++) {
#ifdef HAVE_DEVICE_NUMA_NODE
		DPRINTK(TX_ERR, INFO, "tx[%02d] bd: %d - assigning node %d\n",
		        i, adapter->bd_number, adapter->tx_ring[i]->numa_node);
#endif /* HAVE_DEVICE_NUMA_NODE */
		err = ixgbe_setup_tx_resources(adapter->tx_ring[i]);
		if (!err)
			continue;
		DPRINTK(PROBE, ERR, "Allocation for Tx Queue %u failed\n", i);
		break;
	}

	return err;
}

/**
 * ixgbe_setup_rx_resources - allocate Rx resources (Descriptors)
 * @rx_ring:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/
int ixgbe_setup_rx_resources(struct ixgbe_ring *rx_ring)
{
	struct device *dev = rx_ring->dev;
	int size;

	size = sizeof(struct ixgbe_rx_buffer) * rx_ring->count;
	rx_ring->rx_buffer_info = vmalloc_node(size, rx_ring->numa_node);
	if (!rx_ring->rx_buffer_info)
		rx_ring->rx_buffer_info = vmalloc(size);
	if (!rx_ring->rx_buffer_info)
		goto err;
	memset(rx_ring->rx_buffer_info, 0, size);

	/* Round up to nearest 4K */
	rx_ring->size = rx_ring->count * sizeof(union ixgbe_adv_rx_desc);
	rx_ring->size = ALIGN(rx_ring->size, 4096);

	rx_ring->desc = dma_alloc_coherent(dev, rx_ring->size,
					   &rx_ring->dma, GFP_KERNEL);

	if (!rx_ring->desc)
		goto err;

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;
#ifndef CONFIG_IXGBE_NAPI
	rx_ring->work_limit = rx_ring->count / 2;
#endif

	return 0;
err:
	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;
	dev_err(dev, "Unable to allocate memory for the Rx descriptor ring\n");
	return -ENOMEM;
}

/**
 * ixgbe_setup_all_rx_resources - allocate all queues Rx resources
 * @adapter: board private structure
 *
 * If this function returns with an error, then it's possible one or
 * more of the rings is populated (while the rest are not).  It is the
 * callers duty to clean those orphaned rings.
 *
 * Return 0 on success, negative on failure
 **/
static int ixgbe_setup_all_rx_resources(struct ixgbe_adapter *adapter)
{
	int i, err = 0;

	for (i = 0; i < adapter->num_rx_queues; i++) {
#ifdef HAVE_DEVICE_NUMA_NODE
		DPRINTK(RX_ERR, INFO, "rx[%02d] bd: %d - assigning node %d\n",
		        i, adapter->bd_number, adapter->rx_ring[i]->numa_node);
#endif /* HAVE_DEVICE_NUMA_NODE */
		err = ixgbe_setup_rx_resources(adapter->rx_ring[i]);
		if (!err)
			continue;
		DPRINTK(PROBE, ERR, "Allocation for Rx Queue %u failed\n", i);
		break;
	}

	return err;
}

/**
 * ixgbe_free_tx_resources - Free Tx Resources per Queue
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/
void ixgbe_free_tx_resources(struct ixgbe_ring *tx_ring)
{
	ixgbe_clean_tx_ring(tx_ring);

	vfree(tx_ring->tx_buffer_info);
	tx_ring->tx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!tx_ring->desc)
		return;

	dma_free_coherent(tx_ring->dev, tx_ring->size,
			  tx_ring->desc, tx_ring->dma);

	tx_ring->desc = NULL;
}

/**
 * ixgbe_free_all_tx_resources - Free Tx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
static void ixgbe_free_all_tx_resources(struct ixgbe_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_tx_queues; i++)
		if (adapter->tx_ring[i]->desc)
			ixgbe_free_tx_resources(adapter->tx_ring[i]);
}

/**
 * ixgbe_free_rx_resources - Free Rx Resources
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
void ixgbe_free_rx_resources(struct ixgbe_ring *rx_ring)
{
	ixgbe_clean_rx_ring(rx_ring);

	vfree(rx_ring->rx_buffer_info);
	rx_ring->rx_buffer_info = NULL;

	/* if not set, then don't free */
	if (!rx_ring->desc)
		return;

	dma_free_coherent(rx_ring->dev, rx_ring->size,
			  rx_ring->desc, rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 * ixgbe_free_all_rx_resources - Free Rx Resources for All Queues
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
static void ixgbe_free_all_rx_resources(struct ixgbe_adapter *adapter)
{
	int i;

	for (i = 0; i < adapter->num_rx_queues; i++)
		if (adapter->rx_ring[i]->desc)
			ixgbe_free_rx_resources(adapter->rx_ring[i]);
}

/**
 * ixgbe_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 **/
static int ixgbe_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN;

	/* MTU < 68 is an error and causes problems on some kernels */
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED) {
		if ((new_mtu < 68) || (max_frame > MAXIMUM_ETHERNET_VLAN_SIZE))
			return -EINVAL;
	} else {
		if ((new_mtu < 68) || (max_frame > IXGBE_MAX_JUMBO_FRAME_SIZE))
			return -EINVAL;
	}

	DPRINTK(PROBE, INFO, "changing MTU from %d to %d\n",
	        netdev->mtu, new_mtu);
	/* must set new MTU before calling down or up */
	netdev->mtu = new_mtu;

	if (netif_running(netdev))
		ixgbe_reinit_locked(adapter);

	return 0;
}

/**
 * ixgbe_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog timer is started,
 * and the stack is notified that the interface is ready.
 **/
static int ixgbe_open(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	int err;

	/* disallow open during test */
	if (test_bit(__IXGBE_TESTING, &adapter->state))
		return -EBUSY;

	netif_carrier_off(netdev);

	/* allocate transmit descriptors */
	err = ixgbe_setup_all_tx_resources(adapter);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	err = ixgbe_setup_all_rx_resources(adapter);
	if (err)
		goto err_setup_rx;

	ixgbe_configure(adapter);

	/*
	 * Map the Tx/Rx rings to the vectors we were allotted.
	 * if request_irq will be called in this function map_rings
	 * must be called *before* up_complete
	 */
	ixgbe_map_rings_to_vectors(adapter);

	err = ixgbe_up_complete(adapter);
	if (err)
		goto err_setup_rx;

	/* clear any pending interrupts, may auto mask */
	IXGBE_READ_REG(hw, IXGBE_EICR);

	err = ixgbe_request_irq(adapter);
	if (err)
		goto err_req_irq;

	ixgbe_irq_enable(adapter, true, true);

	/*
	 * If this adapter has a fan, check to see if we had a failure
	 * before we enabled the interrupt.
	 */
	if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE) {
		u32 esdp = IXGBE_READ_REG(hw, IXGBE_ESDP);
		if (esdp & IXGBE_ESDP_SDP1)
			DPRINTK(DRV, CRIT,
				"Fan has stopped, replace the adapter\n");
	}

	return 0;

err_req_irq:
	ixgbe_down(adapter);
	ixgbe_release_hw_control(adapter);
	ixgbe_free_irq(adapter);
err_setup_rx:
	ixgbe_free_all_rx_resources(adapter);
err_setup_tx:
	ixgbe_free_all_tx_resources(adapter);
	ixgbe_reset(adapter);

	return err;
}

/**
 * ixgbe_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/
static int ixgbe_close(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);

	ixgbe_down(adapter);
	ixgbe_free_irq(adapter);

	ixgbe_free_all_tx_resources(adapter);
	ixgbe_free_all_rx_resources(adapter);

	ixgbe_release_hw_control(adapter);

	return 0;
}

#ifdef CONFIG_PM
static int ixgbe_resume(struct pci_dev *pdev)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;
	u32 err;

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	/*
	 * pci_restore_state clears dev->state_saved so call
	 * pci_save_state to restore it.
	 */
	pci_save_state(pdev);

	err = pci_enable_device(pdev);
	if (err) {
		printk(KERN_ERR "ixgbe: Cannot enable PCI device from "
		       "suspend\n");
		return err;
	}
	pci_set_master(pdev);

	pci_wake_from_d3(pdev, false);

	err = ixgbe_init_interrupt_scheme(adapter);
	if (err) {
		printk(KERN_ERR "ixgbe: Cannot initialize interrupts for "
		       "device\n");
		return err;
	}

	ixgbe_reset(adapter);

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_WUS, ~0);

	if (netif_running(netdev)) {
		err = ixgbe_open(netdev);
		if (err)
			return err;
	}

	netif_device_attach(netdev);

	return 0;
}
#endif /* CONFIG_PM */

/*
 * __ixgbe_shutdown is not used when power management
 * is disabled on older kernels (<2.6.12). causes a compile
 * warning/error, because it is defined and not used.
 */
#if defined(CONFIG_PM) || !defined(USE_REBOOT_NOTIFIER)
static int __ixgbe_shutdown(struct pci_dev *pdev, bool *enable_wake)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 ctrl, fctrl;
	u32 wufc = adapter->wol;
#ifdef CONFIG_PM
	int retval = 0;
#endif

	netif_device_detach(netdev);

	if (netif_running(netdev)) {
		ixgbe_down(adapter);
		ixgbe_free_irq(adapter);
		ixgbe_free_all_tx_resources(adapter);
		ixgbe_free_all_rx_resources(adapter);
	}

	ixgbe_clear_interrupt_scheme(adapter);

#ifdef CONFIG_PM
	retval = pci_save_state(pdev);
	if (retval)
		return retval;

#endif
	if (wufc) {
		ixgbe_set_rx_mode(netdev);

		/* turn on all-multi mode if wake on multicast is enabled */
		if (wufc & IXGBE_WUFC_MC) {
			fctrl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
			fctrl |= IXGBE_FCTRL_MPE;
			IXGBE_WRITE_REG(hw, IXGBE_FCTRL, fctrl);
		}

		ctrl = IXGBE_READ_REG(hw, IXGBE_CTRL);
		ctrl |= IXGBE_CTRL_GIO_DIS;
		IXGBE_WRITE_REG(hw, IXGBE_CTRL, ctrl);

		IXGBE_WRITE_REG(hw, IXGBE_WUFC, wufc);
	} else {
		IXGBE_WRITE_REG(hw, IXGBE_WUC, 0);
		IXGBE_WRITE_REG(hw, IXGBE_WUFC, 0);
	}

	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		pci_wake_from_d3(pdev, false);
		break;
	case ixgbe_mac_82599EB:
		if (wufc)
			pci_wake_from_d3(pdev, true);
		else
			pci_wake_from_d3(pdev, false);
		break;
	default:
		break;
	}

	*enable_wake = !!wufc;

	ixgbe_release_hw_control(adapter);

	pci_disable_device(pdev);

	return 0;
}
#endif /* defined(CONFIG_PM) || !defined(USE_REBOOT_NOTIFIER) */

#ifdef CONFIG_PM
static int ixgbe_suspend(struct pci_dev *pdev, pm_message_t state)
{
	int retval;
	bool wake;

	retval = __ixgbe_shutdown(pdev, &wake);
	if (retval)
		return retval;

	if (wake) {
		pci_prepare_to_sleep(pdev);
	} else {
		pci_wake_from_d3(pdev, false);
		pci_set_power_state(pdev, PCI_D3hot);
	}

	return 0;
}
#endif /* CONFIG_PM */

#ifndef USE_REBOOT_NOTIFIER
static void ixgbe_shutdown(struct pci_dev *pdev)
{
	bool wake;

	__ixgbe_shutdown(pdev, &wake);

	if (system_state == SYSTEM_POWER_OFF) {
		pci_wake_from_d3(pdev, wake);
		pci_set_power_state(pdev, PCI_D3hot);
	}
}

#endif
/**
 * ixgbe_update_stats - Update the board statistics counters.
 * @adapter: board private structure
 **/
void ixgbe_update_stats(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u64 total_mpc = 0;
	u32 i, missed_rx = 0, mpc, bprc, lxon, lxoff, xon_off_tot;
	u64 non_eop_descs = 0, restart_queue = 0, tx_busy = 0;
	u64 alloc_rx_page_failed = 0, alloc_rx_buff_failed = 0;
	u64 bytes = 0, packets = 0;
#ifndef IXGBE_NO_LRO
	u32 flushed = 0, coal = 0, recycled = 0;
	int num_q_vectors = 1;
#endif

	if (test_bit(__IXGBE_DOWN, &adapter->state) ||
	    test_bit(__IXGBE_RESETTING, &adapter->state))
		return;

#ifndef IXGBE_NO_LRO
	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED)
		num_q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;

#endif
	if (adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED) {
		u64 rsc_count = 0;
		u64 rsc_flush = 0;
		for (i = 0; i < 16; i++)
			adapter->hw_rx_no_dma_resources += IXGBE_READ_REG(hw, IXGBE_QPRDC(i));
		for (i = 0; i < adapter->num_rx_queues; i++) {
			rsc_count += adapter->rx_ring[i]->rx_stats.rsc_count;
			rsc_flush += adapter->rx_ring[i]->rx_stats.rsc_flush;
		}
		adapter->rsc_total_count = rsc_count;
		adapter->rsc_total_flush = rsc_flush;
	}

#ifndef IXGBE_NO_LRO
	for (i = 0; i < num_q_vectors; i++) {
		struct ixgbe_q_vector *q_vector = adapter->q_vector[i];
		if (!q_vector || !q_vector->lrolist)
			continue;
		flushed += q_vector->lrolist->stats.flushed;
		coal += q_vector->lrolist->stats.coal;
		recycled += q_vector->lrolist->stats.recycled;
	}
	adapter->lro_stats.flushed = flushed;
	adapter->lro_stats.coal = coal;
	adapter->lro_stats.recycled = recycled;

#endif
	for (i = 0; i < adapter->num_rx_queues; i++) {
		struct ixgbe_ring *rx_ring = adapter->rx_ring[i];
		non_eop_descs += rx_ring->rx_stats.non_eop_descs;
		alloc_rx_page_failed += rx_ring->rx_stats.alloc_rx_page_failed;
		alloc_rx_buff_failed += rx_ring->rx_stats.alloc_rx_buff_failed;
		bytes += rx_ring->stats.bytes;
		packets += rx_ring->stats.packets;
		
	}
	adapter->non_eop_descs = non_eop_descs;
	adapter->alloc_rx_page_failed = alloc_rx_page_failed;
	adapter->alloc_rx_buff_failed = alloc_rx_buff_failed;
	adapter->net_stats.rx_bytes = bytes;
	adapter->net_stats.rx_packets = packets;

	bytes = 0;
	packets = 0;
	/* gather some stats to the adapter struct that are per queue */
	for (i = 0; i < adapter->num_tx_queues; i++) {
		struct ixgbe_ring *tx_ring = adapter->tx_ring[i];
		restart_queue += tx_ring->tx_stats.restart_queue;
		tx_busy += tx_ring->tx_stats.tx_busy;
		bytes += tx_ring->stats.bytes;
		packets += tx_ring->stats.packets;
	}
	adapter->restart_queue = restart_queue;
	adapter->tx_busy = tx_busy;
	adapter->net_stats.tx_bytes = bytes;
	adapter->net_stats.tx_packets = packets;

	adapter->stats.crcerrs += IXGBE_READ_REG(hw, IXGBE_CRCERRS);
	for (i = 0; i < 8; i++) {
		/* for packet buffers not used, the register should read 0 */
		mpc = IXGBE_READ_REG(hw, IXGBE_MPC(i));
		missed_rx += mpc;
		adapter->stats.mpc[i] += mpc;
		total_mpc += adapter->stats.mpc[i];
		if (hw->mac.type == ixgbe_mac_82598EB)
			adapter->stats.rnbc[i] += IXGBE_READ_REG(hw, IXGBE_RNBC(i));
		adapter->stats.qptc[i] += IXGBE_READ_REG(hw, IXGBE_QPTC(i));
		adapter->stats.qbtc[i] += IXGBE_READ_REG(hw, IXGBE_QBTC(i));
		adapter->stats.qprc[i] += IXGBE_READ_REG(hw, IXGBE_QPRC(i));
		adapter->stats.qbrc[i] += IXGBE_READ_REG(hw, IXGBE_QBRC(i));
		switch (hw->mac.type) {
		case ixgbe_mac_82598EB:
			adapter->stats.pxonrxc[i] += IXGBE_READ_REG(hw,
			                                      IXGBE_PXONRXC(i));
			adapter->stats.pxoffrxc[i] += IXGBE_READ_REG(hw,
			                                     IXGBE_PXOFFRXC(i));
			break;
		case ixgbe_mac_82599EB:
			adapter->stats.pxonrxc[i] += IXGBE_READ_REG(hw,
			                                    IXGBE_PXONRXCNT(i));
			adapter->stats.pxoffrxc[i] += IXGBE_READ_REG(hw,
			                                   IXGBE_PXOFFRXCNT(i));
			break;
		default:
			break;
		}
		adapter->stats.pxontxc[i] += IXGBE_READ_REG(hw,
		                                            IXGBE_PXONTXC(i));
		adapter->stats.pxofftxc[i] += IXGBE_READ_REG(hw,
		                                             IXGBE_PXOFFTXC(i));
	}
	adapter->stats.gprc += IXGBE_READ_REG(hw, IXGBE_GPRC);
	/* work around hardware counting issue */
	adapter->stats.gprc -= missed_rx;

	/* 82598 hardware only has a 32 bit counter in the high register */
	switch (hw->mac.type) {
	case ixgbe_mac_82598EB:
		adapter->stats.lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXC);
		adapter->stats.lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXC);
		adapter->stats.gorc += IXGBE_READ_REG(hw, IXGBE_GORCH);
		adapter->stats.gotc += IXGBE_READ_REG(hw, IXGBE_GOTCH);
		adapter->stats.tor += IXGBE_READ_REG(hw, IXGBE_TORH);
		break;
	case ixgbe_mac_82599EB:
		adapter->stats.gorc += IXGBE_READ_REG(hw, IXGBE_GORCL);
		IXGBE_READ_REG(hw, IXGBE_GORCH); /* to clear */
		adapter->stats.gotc += IXGBE_READ_REG(hw, IXGBE_GOTCL);
		IXGBE_READ_REG(hw, IXGBE_GOTCH); /* to clear */
		adapter->stats.tor += IXGBE_READ_REG(hw, IXGBE_TORL);
		IXGBE_READ_REG(hw, IXGBE_TORH); /* to clear */
		adapter->stats.lxonrxc += IXGBE_READ_REG(hw, IXGBE_LXONRXCNT);
		adapter->stats.lxoffrxc += IXGBE_READ_REG(hw, IXGBE_LXOFFRXCNT);
#ifdef HAVE_TX_MQ
		adapter->stats.fdirmatch += IXGBE_READ_REG(hw, IXGBE_FDIRMATCH);
		adapter->stats.fdirmiss += IXGBE_READ_REG(hw, IXGBE_FDIRMISS);
#endif /* HAVE_TX_MQ */
#ifdef IXGBE_FCOE
		adapter->stats.fccrc += IXGBE_READ_REG(hw, IXGBE_FCCRC);
		adapter->stats.fclast += IXGBE_READ_REG(hw, IXGBE_FCLAST);
		adapter->stats.fcoerpdc += IXGBE_READ_REG(hw, IXGBE_FCOERPDC);
		adapter->stats.fcoeprc += IXGBE_READ_REG(hw, IXGBE_FCOEPRC);
		adapter->stats.fcoeptc += IXGBE_READ_REG(hw, IXGBE_FCOEPTC);
		adapter->stats.fcoedwrc += IXGBE_READ_REG(hw, IXGBE_FCOEDWRC);
		adapter->stats.fcoedwtc += IXGBE_READ_REG(hw, IXGBE_FCOEDWTC);
#endif /* IXGBE_FCOE */
		break;
	default:
		break;
	}
	bprc = IXGBE_READ_REG(hw, IXGBE_BPRC);
	adapter->stats.bprc += bprc;
	adapter->stats.mprc += IXGBE_READ_REG(hw, IXGBE_MPRC);
	if (hw->mac.type == ixgbe_mac_82598EB)
		adapter->stats.mprc -= bprc;
	adapter->stats.roc += IXGBE_READ_REG(hw, IXGBE_ROC);
	adapter->stats.prc64 += IXGBE_READ_REG(hw, IXGBE_PRC64);
	adapter->stats.prc127 += IXGBE_READ_REG(hw, IXGBE_PRC127);
	adapter->stats.prc255 += IXGBE_READ_REG(hw, IXGBE_PRC255);
	adapter->stats.prc511 += IXGBE_READ_REG(hw, IXGBE_PRC511);
	adapter->stats.prc1023 += IXGBE_READ_REG(hw, IXGBE_PRC1023);
	adapter->stats.prc1522 += IXGBE_READ_REG(hw, IXGBE_PRC1522);
	adapter->stats.rlec += IXGBE_READ_REG(hw, IXGBE_RLEC);
	lxon = IXGBE_READ_REG(hw, IXGBE_LXONTXC);
	adapter->stats.lxontxc += lxon;
	lxoff = IXGBE_READ_REG(hw, IXGBE_LXOFFTXC);
	adapter->stats.lxofftxc += lxoff;
	adapter->stats.ruc += IXGBE_READ_REG(hw, IXGBE_RUC);
	adapter->stats.gptc += IXGBE_READ_REG(hw, IXGBE_GPTC);
	adapter->stats.mptc += IXGBE_READ_REG(hw, IXGBE_MPTC);
	/*
	 * 82598 errata - tx of flow control packets is included in tx counters
	 */
	xon_off_tot = lxon + lxoff;
	adapter->stats.gptc -= xon_off_tot;
	adapter->stats.mptc -= xon_off_tot;
	adapter->stats.gotc -= (xon_off_tot * (ETH_ZLEN + ETH_FCS_LEN));
	adapter->stats.ruc += IXGBE_READ_REG(hw, IXGBE_RUC);
	adapter->stats.rfc += IXGBE_READ_REG(hw, IXGBE_RFC);
	adapter->stats.rjc += IXGBE_READ_REG(hw, IXGBE_RJC);
	adapter->stats.tpr += IXGBE_READ_REG(hw, IXGBE_TPR);
	adapter->stats.ptc64 += IXGBE_READ_REG(hw, IXGBE_PTC64);
	adapter->stats.ptc64 -= xon_off_tot;
	adapter->stats.ptc127 += IXGBE_READ_REG(hw, IXGBE_PTC127);
	adapter->stats.ptc255 += IXGBE_READ_REG(hw, IXGBE_PTC255);
	adapter->stats.ptc511 += IXGBE_READ_REG(hw, IXGBE_PTC511);
	adapter->stats.ptc1023 += IXGBE_READ_REG(hw, IXGBE_PTC1023);
	adapter->stats.ptc1522 += IXGBE_READ_REG(hw, IXGBE_PTC1522);
	adapter->stats.bptc += IXGBE_READ_REG(hw, IXGBE_BPTC);

	/* Fill out the OS statistics structure */
	adapter->net_stats.multicast = adapter->stats.mprc;

	/* Rx Errors */
	adapter->net_stats.rx_errors = adapter->stats.crcerrs +
	                               adapter->stats.rlec;
	adapter->net_stats.rx_dropped = 0;
	adapter->net_stats.rx_length_errors = adapter->stats.rlec;
	adapter->net_stats.rx_crc_errors = adapter->stats.crcerrs;
	adapter->net_stats.rx_missed_errors = total_mpc;

	/*
	 * VF Stats Collection - skip while resetting because these
	 * are not clear on read and otherwise you'll sometimes get
	 * crazy values.
	 */
	if (!test_bit(__IXGBE_RESETTING, &adapter->state)) {
		for(i = 0; i < adapter->num_vfs; i++) {
			UPDATE_VF_COUNTER_32bit(IXGBE_PVFGPRC(i),	      \
					adapter->vfinfo[i].last_vfstats.gprc, \
					adapter->vfinfo[i].vfstats.gprc);
			UPDATE_VF_COUNTER_32bit(IXGBE_PVFGPTC(i),	      \
					adapter->vfinfo[i].last_vfstats.gptc, \
					adapter->vfinfo[i].vfstats.gptc);
			UPDATE_VF_COUNTER_36bit(IXGBE_PVFGORC_LSB(i),	      \
					IXGBE_PVFGORC_MSB(i),		      \
					adapter->vfinfo[i].last_vfstats.gorc, \
					adapter->vfinfo[i].vfstats.gorc);
			UPDATE_VF_COUNTER_36bit(IXGBE_PVFGOTC_LSB(i),	      \
					IXGBE_PVFGOTC_MSB(i),		      \
					adapter->vfinfo[i].last_vfstats.gotc, \
					adapter->vfinfo[i].vfstats.gotc);
			UPDATE_VF_COUNTER_32bit(IXGBE_PVFMPRC(i),	      \
					adapter->vfinfo[i].last_vfstats.mprc, \
					adapter->vfinfo[i].vfstats.mprc);
		}
	}
}

/**
 * ixgbe_watchdog - Timer Call-back
 * @data: pointer to adapter cast into an unsigned long
 **/
static void ixgbe_watchdog(unsigned long data)
{
	struct ixgbe_adapter *adapter = (struct ixgbe_adapter *)data;
	struct ixgbe_hw *hw = &adapter->hw;
	u64 eics = 0;
	int i;

	/* if interface is down do nothing */
	if (test_bit(__IXGBE_DOWN, &adapter->state))
		return;

	if (!(adapter->flags & IXGBE_FLAG_MSIX_ENABLED)) {
		/*
		 * for legacy and MSI interrupts don't set any bits
		 * that are enabled for EIAM, because this operation
		 * would set *both* EIMS and EICS for any bit in EIAM
		 */
		IXGBE_WRITE_REG(hw, IXGBE_EICS,
			(IXGBE_EICS_TCP_TIMER | IXGBE_EICS_OTHER));
	} else {
		/* get one bit for every active tx/rx interrupt vector */
		for (i = 0; i < adapter->num_msix_vectors - NON_Q_VECTORS; i++) {
			struct ixgbe_q_vector *qv = adapter->q_vector[i];
			if (qv->rxr_count || qv->txr_count)
				eics |= ((u64)1 << i);
		}
	}

	/* Cause software interrupt to ensure rings are cleaned */
	ixgbe_irq_rearm_queues(adapter, eics);

	/* Reset the timer */
	mod_timer(&adapter->watchdog_timer, round_jiffies(jiffies + 2 * HZ));

	schedule_work(&adapter->watchdog_task);
}

/**
 * ixgbe_multispeed_fiber_task - worker thread to configure multispeed fiber
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_multispeed_fiber_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             multispeed_fiber_task);
	struct ixgbe_hw *hw = &adapter->hw;
	u32 autoneg;
	bool negotiation;

	adapter->flags |= IXGBE_FLAG_IN_SFP_LINK_TASK;
	autoneg = hw->phy.autoneg_advertised;
	if ((!autoneg) && (hw->mac.ops.get_link_capabilities))
		hw->mac.ops.get_link_capabilities(hw, &autoneg, &negotiation);
	hw->mac.autotry_restart = false;
	if (hw->mac.ops.setup_link)
		hw->mac.ops.setup_link(hw, autoneg, negotiation, true);
	adapter->flags |= IXGBE_FLAG_NEED_LINK_UPDATE;
	adapter->link_check_timeout = jiffies;
	adapter->flags &= ~IXGBE_FLAG_IN_SFP_LINK_TASK;
}

/**
 * ixgbe_sfp_config_module_task - worker thread to configure a new SFP+ module
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_sfp_config_module_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             sfp_config_module_task);
	struct ixgbe_hw *hw = &adapter->hw;
	u32 err;

	adapter->flags |= IXGBE_FLAG_IN_SFP_MOD_TASK;
	err = hw->phy.ops.identify_sfp(hw);
	if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		DPRINTK(PROBE, ERR, "failed to load because an "
		        "unsupported SFP+ module type was detected.\n");
		unregister_netdev(adapter->netdev);
		adapter->netdev_registered = false;
		return;
	}
	/*
	 * A module may be identified correctly, but the EEPROM may not have
	 * support for that module.  setup_sfp() will fail in that case, so
	 * we should not allow that module to load.
	 */
	err = hw->mac.ops.setup_sfp(hw);
	if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		DPRINTK(PROBE, ERR, "failed to load because an "
		        "unsupported SFP+ module type was detected.\n");
		unregister_netdev(adapter->netdev);
		adapter->netdev_registered = false;
		return;
	}

	if (!(adapter->flags & IXGBE_FLAG_IN_SFP_LINK_TASK))
		/* This will also work for DA Twinax connections */
		schedule_work(&adapter->multispeed_fiber_task);
	adapter->flags &= ~IXGBE_FLAG_IN_SFP_MOD_TASK;
}

#ifdef HAVE_TX_MQ
/**
 * ixgbe_fdir_reinit_task - worker thread to reinit FDIR filter table
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_fdir_reinit_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             fdir_reinit_task);
	struct ixgbe_hw *hw = &adapter->hw;
	int i;

	/* if interface is down do nothing */
	if (test_bit(__IXGBE_DOWN, &adapter->state))
		return;

	if (ixgbe_reinit_fdir_tables_82599(hw) == 0) {
		for (i = 0; i < adapter->num_tx_queues; i++)
			set_bit(__IXGBE_TX_FDIR_INIT_DONE,
			        &(adapter->tx_ring[i]->state));
	} else {
		DPRINTK(PROBE, ERR, "failed to finish FDIR re-initialization, "
		        "ignored adding FDIR ATR filters \n");
	}
	/* Done FDIR Re-initialization, enable transmits */
	netif_tx_start_all_queues(adapter->netdev);
}

#endif /* HAVE_TX_MQ */
/**
 * ixgbe_watchdog_task - worker thread to bring link up
 * @work: pointer to work_struct containing our data
 **/
static void ixgbe_watchdog_task(struct work_struct *work)
{
	struct ixgbe_adapter *adapter = container_of(work,
	                                             struct ixgbe_adapter,
	                                             watchdog_task);
	struct net_device *netdev = adapter->netdev;
	struct ixgbe_hw *hw = &adapter->hw;
	u32 link_speed = adapter->link_speed;
	bool link_up = adapter->link_up;
	int i;
	struct ixgbe_ring *tx_ring;
	int some_tx_pending = 0;

	/* if interface is down do nothing */
	if (test_bit(__IXGBE_DOWN, &adapter->state))
		return;

	if (adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE) {
		if (hw->mac.ops.check_link) {
			hw->mac.ops.check_link(hw, &link_speed, &link_up, false);
		} else {
			/* always assume link is up, if no check link function */
			link_speed = IXGBE_LINK_SPEED_10GB_FULL;
			link_up = true;
		}
		if (link_up) {
			if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
				for (i = 0; i < MAX_TRAFFIC_CLASS; i++)
					hw->mac.ops.fc_enable(hw, i);
			} else {
				hw->mac.ops.fc_enable(hw, 0);
			}
		}

		if (link_up ||
		    time_after(jiffies, (adapter->link_check_timeout +
		                         IXGBE_TRY_LINK_TIMEOUT))) {
			adapter->flags &= ~IXGBE_FLAG_NEED_LINK_UPDATE;
			IXGBE_WRITE_REG(hw, IXGBE_EIMS, IXGBE_EIMC_LSC);
			IXGBE_WRITE_FLUSH(hw);
		}
		adapter->link_up = link_up;
		adapter->link_speed = link_speed;
	}

	if (link_up) {
		if (!netif_carrier_ok(netdev)) {
			bool flow_rx, flow_tx;

			switch (hw->mac.type) {
			case ixgbe_mac_82598EB: {
				u32 frctl = IXGBE_READ_REG(hw, IXGBE_FCTRL);
				u32 rmcs = IXGBE_READ_REG(hw, IXGBE_RMCS);
				flow_rx = !!(frctl & IXGBE_FCTRL_RFCE);
				flow_tx = !!(rmcs & IXGBE_RMCS_TFCE_802_3X);
			}
				break;
			case ixgbe_mac_82599EB: {
				u32 mflcn = IXGBE_READ_REG(hw, IXGBE_MFLCN);
				u32 fccfg = IXGBE_READ_REG(hw, IXGBE_FCCFG);
				flow_rx = !!(mflcn & IXGBE_MFLCN_RFCE);
				flow_tx = !!(fccfg & IXGBE_FCCFG_TFCE_802_3X);
			}
				break;
			default:
				flow_tx = false;
				flow_rx = false;
				break;
			}
			DPRINTK(LINK, INFO, "NIC Link is Up %s, "
			        "Flow Control: %s\n",
			        (link_speed == IXGBE_LINK_SPEED_10GB_FULL ?
			         "10 Gbps" :
			         (link_speed == IXGBE_LINK_SPEED_1GB_FULL ?
			          "1 Gbps" : "unknown speed")),
			        ((flow_rx && flow_tx) ? "RX/TX" :
			         (flow_rx ? "RX" :
			         (flow_tx ? "TX" : "None"))));

			netif_carrier_on(netdev);
			netif_tx_wake_all_queues(netdev);
		} else {
			/* Force detection of hung controller */
			for (i = 0; i < adapter->num_tx_queues; i++) {
				tx_ring = adapter->tx_ring[i];
				set_check_for_tx_hang(tx_ring);
			}
		}
	} else {
		adapter->link_up = false;
		adapter->link_speed = 0;
		if (netif_carrier_ok(netdev)) {
			DPRINTK(LINK, INFO, "NIC Link is Down\n");
			netif_carrier_off(netdev);
			netif_tx_stop_all_queues(netdev);
		}
	}

	if (!netif_carrier_ok(netdev)) {
		for (i = 0; i < adapter->num_tx_queues; i++) {
			tx_ring = adapter->tx_ring[i];
			if (tx_ring->next_to_use != tx_ring->next_to_clean) {
				some_tx_pending = 1;
				break;
			}
		}

		if (some_tx_pending) {
			/* We've lost link, so the controller stops DMA,
			 * but we've got queued Tx work that's never going
			 * to get done, so reset controller to flush Tx.
			 * (Do the reset outside of interrupt context).
			 */
			 schedule_work(&adapter->reset_task);
		}
	}

	ixgbe_update_stats(adapter);

	if (adapter->flags & IXGBE_FLAG_NEED_LINK_UPDATE) {
		/* poll faster when waiting for link */
		mod_timer(&adapter->watchdog_timer, jiffies + (HZ/10));
	}

}

void ixgbe_tx_ctxtdesc(struct ixgbe_ring *tx_ring, u32 vlan_macip_lens,
		       u32 fcoe_sof_eof, u32 type_tucmd, u32 mss_l4len_idx)
{
	struct ixgbe_adv_tx_context_desc *context_desc;
	struct ixgbe_tx_buffer *tx_buffer_info;
	u16 i = tx_ring->next_to_use;

	tx_buffer_info = &tx_ring->tx_buffer_info[i];
	tx_buffer_info->time_stamp = jiffies;
	tx_buffer_info->next_to_watch = i;

	context_desc = IXGBE_TX_CTXTDESC_ADV(tx_ring, i);

	i++;
	if (i == tx_ring->count)
		tx_ring->next_to_use = 0;
	else
		tx_ring->next_to_use = i;

	/* set bits to identify this as an advanced context descriptor */
	type_tucmd |= IXGBE_TXD_CMD_DEXT | IXGBE_ADVTXD_DTYP_CTXT;

	context_desc->vlan_macip_lens	= cpu_to_le32(vlan_macip_lens);
	context_desc->seqnum_seed	= cpu_to_le32(fcoe_sof_eof);
	context_desc->type_tucmd_mlhl	= cpu_to_le32(type_tucmd);
	context_desc->mss_l4len_idx	= cpu_to_le32(mss_l4len_idx);
}

static int ixgbe_tso(struct ixgbe_ring *tx_ring, struct sk_buff *skb,
                     u32 tx_flags, u8 *hdr_len)
{
#ifdef NETIF_F_TSO
	int err;
	u32 vlan_macip_lens, type_tucmd;
	u32 mss_l4len_idx, l4len;

	if (!skb_is_gso(skb))
#endif /* NETIF_F_TSO */
		return 0;
#ifdef NETIF_F_TSO

	if (skb_header_cloned(skb)) {
		err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (err)
			return err;
	}

	/* ADV DTYP TUCMD MKRLOC/ISCSIHEDLEN */
	type_tucmd = IXGBE_ADVTXD_TUCMD_L4T_TCP;

	if (skb->protocol == __constant_htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);
		iph->tot_len = 0;
		iph->check = 0;
		tcp_hdr(skb)->check = ~csum_tcpudp_magic(iph->saddr,
		                                         iph->daddr, 0,
		                                         IPPROTO_TCP,
		                                         0);
		type_tucmd |= IXGBE_ADVTXD_TUCMD_IPV4;
#ifdef NETIF_F_TSO6
	} else if (skb_is_gso_v6(skb)) {
		ipv6_hdr(skb)->payload_len = 0;
		tcp_hdr(skb)->check =
		    ~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
		                     &ipv6_hdr(skb)->daddr,
		                     0, IPPROTO_TCP, 0);
#endif
	}

	l4len = tcp_hdrlen(skb);
	*hdr_len = skb_transport_offset(skb) + l4len;

	/* mss_l4len_id: use 1 as index for TSO */
	mss_l4len_idx = l4len << IXGBE_ADVTXD_L4LEN_SHIFT;
	mss_l4len_idx |= skb_shinfo(skb)->gso_size << IXGBE_ADVTXD_MSS_SHIFT;
	mss_l4len_idx |= (1 << IXGBE_ADVTXD_IDX_SHIFT);

	/* vlan_macip_lens: HEADLEN, MACLEN, VLAN tag */
	vlan_macip_lens = skb_network_header_len(skb);
	vlan_macip_lens |= skb_network_offset(skb) << IXGBE_ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= tx_flags & IXGBE_TX_FLAGS_VLAN_MASK;

	ixgbe_tx_ctxtdesc(tx_ring, vlan_macip_lens, 0, type_tucmd,
	                  mss_l4len_idx);

	return 1;
#endif
}

static bool ixgbe_tx_csum(struct ixgbe_ring *tx_ring,
                          struct sk_buff *skb, u32 tx_flags)
{
	u32 vlan_macip_lens = 0, type_tucmd = 0;

	if (skb->ip_summed != CHECKSUM_PARTIAL) {
		if (!(tx_flags & IXGBE_TX_FLAGS_VLAN))
			return false;
	} else {
		__be16 protocol = skb->protocol;

#ifdef NETIF_F_HW_VLAN_TX
		if (skb->protocol == __constant_htons(ETH_P_8021Q)) {
			const struct vlan_ethhdr *vhdr =
				(const struct vlan_ethhdr *)skb->data;

			protocol = vhdr->h_vlan_encapsulated_proto;
		} else {
			protocol = skb->protocol;
		}

#endif
		switch (protocol) {
		case __constant_htons(ETH_P_IP):
			if (ip_hdr(skb)->protocol == IPPROTO_TCP)
				type_tucmd = IXGBE_ADVTXD_TUCMD_L4T_TCP |
					     IXGBE_ADVTXD_TUCMD_IPV4;
			else
				type_tucmd = IXGBE_ADVTXD_TUCMD_IPV4;
			break;
#ifdef NETIF_F_IPV6_CSUM
		case __constant_htons(ETH_P_IPV6):
			/* XXX what about other V6 headers?? */
			if (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP)
				type_tucmd = IXGBE_ADVTXD_TUCMD_L4T_TCP;
			break;
#endif
		default:
			if (unlikely(net_ratelimit())) {
				dev_warn(tx_ring->dev,
				 "partial checksum but proto=%x!\n",
				 skb->protocol);
			}
			break;
		}
		vlan_macip_lens = skb_network_header_len(skb);
	}

	vlan_macip_lens |= skb_network_offset(skb) << IXGBE_ADVTXD_MACLEN_SHIFT;
	vlan_macip_lens |= tx_flags & IXGBE_TX_FLAGS_VLAN_MASK;

	ixgbe_tx_ctxtdesc(tx_ring, vlan_macip_lens, 0, type_tucmd, 0);

	return (skb->ip_summed == CHECKSUM_PARTIAL);
}

static u16 ixgbe_tx_map(struct ixgbe_ring *tx_ring,
                        struct sk_buff *skb, u32 tx_flags,
                        unsigned int first, const u8 hdr_len)
{
	struct device *dev = tx_ring->dev;
	struct ixgbe_tx_buffer *tx_buffer_info;
#ifdef MAX_SKB_FRAGS
	unsigned int nr_frags = skb_shinfo(skb)->nr_frags;
	unsigned int f = 0;
	unsigned int data_len = skb->data_len;
#endif
	unsigned int len = skb_headlen(skb), bytecount = skb->len;
	u32 offset = 0, size;
	u16 gso_segs = 1;
	u16 i = tx_ring->next_to_use;
	u16 count = 0;

#ifdef IXGBE_FCOE
	if (tx_flags & IXGBE_TX_FLAGS_FCOE) {
		if (data_len >= sizeof(struct fcoe_crc_eof)) {
			data_len -= sizeof(struct fcoe_crc_eof);
		} else {
			len -= sizeof(struct fcoe_crc_eof) - data_len;
			data_len = 0;
		}
	}

#endif
	while (len) {
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		size = min_t(unsigned int, len, IXGBE_MAX_DATA_PER_TXD);

		tx_buffer_info->length = size;
		tx_buffer_info->mapped_as_page = false;
		tx_buffer_info->dma = dma_map_single(dev,
		                                     skb->data + offset,
		                                     size, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, tx_buffer_info->dma))
			goto dma_error;
		tx_buffer_info->time_stamp = jiffies;
		tx_buffer_info->next_to_watch = i;

		len -= size;
		offset += size;
		count++;
		i++;
		if (i == tx_ring->count)
			i = 0;
	}

#ifdef MAX_SKB_FRAGS
	while (data_len && (f < nr_frags)) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[f];
		len = min_t(unsigned int, data_len, frag->size);

		offset = frag->page_offset;

		while (len) {
			tx_buffer_info = &tx_ring->tx_buffer_info[i];
			size = min_t(unsigned int, len, IXGBE_MAX_DATA_PER_TXD);

			tx_buffer_info->length = size;
			tx_buffer_info->mapped_as_page = true;
			tx_buffer_info->dma = dma_map_page(dev,
							   frag->page,
							   offset, size,
							   DMA_TO_DEVICE);
			if (dma_mapping_error(dev, tx_buffer_info->dma))
				goto dma_error;
			tx_buffer_info->time_stamp = jiffies;
			tx_buffer_info->next_to_watch = i;

			len -= size;
			data_len -= size;
			offset += size;
			count++;
			i++;
			if (i == tx_ring->count)
				i = 0;
		}
		f++;
	}

#endif
	if (i == 0)
		i = tx_ring->count;
	i--;

#ifdef NETIF_F_TSO
	if (tx_flags & IXGBE_TX_FLAGS_TSO)
		gso_segs = skb_shinfo(skb)->gso_segs;
#ifdef IXGBE_FCOE
	/* adjust for FCoE Sequence Offload */
	else if (tx_flags & IXGBE_TX_FLAGS_FSO)
		gso_segs = DIV_ROUND_UP(skb->len - hdr_len,
					skb_shinfo(skb)->gso_size);
#endif /* IXGBE_FCOE */
#endif
	bytecount += ((gso_segs - 1) * hdr_len);

	/* multiply data chunks by size of headers */
	tx_ring->tx_buffer_info[i].bytecount = bytecount;
	tx_ring->tx_buffer_info[i].gso_segs = gso_segs;
	tx_ring->tx_buffer_info[i].skb = skb;
	tx_ring->tx_buffer_info[first].next_to_watch = i;

	return count;

dma_error:
	dev_err(dev, "TX DMA map failed\n");

	/* clear timestamp and dma mappings for failed tx_buffer_info map */
	tx_buffer_info->dma = 0;
	tx_buffer_info->time_stamp = 0;
	tx_buffer_info->next_to_watch = 0;

	/* clear timestamp and dma mappings for remaining portion of packet */
	for (; count > 0; count--) {
		if (i == 0)
			i = tx_ring->count;
		i--;
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		ixgbe_unmap_and_free_tx_resource(tx_ring, tx_buffer_info);
	}

	return 0;
}

#define IXGBE_TXD_CMD (IXGBE_TXD_CMD_EOP | \
		       IXGBE_TXD_CMD_RS | \
		       IXGBE_TXD_CMD_IFCS)

static void ixgbe_tx_queue(struct ixgbe_ring *tx_ring, int tx_flags,
                           int count, u32 paylen, const u8 hdr_len)
{
	union ixgbe_adv_tx_desc *tx_desc = NULL;
	struct ixgbe_tx_buffer *tx_buffer_info;
	u32 olinfo_status = 0, cmd_type_len;
	u16 i;

	cmd_type_len = IXGBE_ADVTXD_DTYP_DATA |
		       IXGBE_ADVTXD_DCMD_IFCS |
		       IXGBE_ADVTXD_DCMD_DEXT;

	if (tx_flags & IXGBE_TX_FLAGS_VLAN)
		cmd_type_len |= IXGBE_ADVTXD_DCMD_VLE;

	if (tx_flags & IXGBE_TX_FLAGS_TSO) {
		cmd_type_len |= IXGBE_ADVTXD_DCMD_TSE;

		olinfo_status |= IXGBE_TXD_POPTS_TXSM <<
		                 IXGBE_ADVTXD_POPTS_SHIFT;

		/* use index 1 context for tso */
		olinfo_status |= (1 << IXGBE_ADVTXD_IDX_SHIFT);
		if (tx_flags & IXGBE_TX_FLAGS_IPV4)
			olinfo_status |= IXGBE_TXD_POPTS_IXSM <<
			                 IXGBE_ADVTXD_POPTS_SHIFT;

	} else if (tx_flags & IXGBE_TX_FLAGS_CSUM) {
		olinfo_status |= IXGBE_TXD_POPTS_TXSM <<
		                 IXGBE_ADVTXD_POPTS_SHIFT;

#ifdef IXGBE_FCOE
	} else if (tx_flags & IXGBE_TX_FLAGS_FCOE) {
		olinfo_status |= IXGBE_ADVTXD_CC;
		olinfo_status |= (1 << IXGBE_ADVTXD_IDX_SHIFT);
		if (tx_flags & IXGBE_TX_FLAGS_FSO)
			cmd_type_len |= IXGBE_ADVTXD_DCMD_TSE;
#endif /* IXGBE_FCOE */
	}

	olinfo_status |= ((paylen - hdr_len) << IXGBE_ADVTXD_PAYLEN_SHIFT);

	i = tx_ring->next_to_use;
	while (count--) {
		tx_buffer_info = &tx_ring->tx_buffer_info[i];
		tx_desc = IXGBE_TX_DESC_ADV(tx_ring, i);
		tx_desc->read.buffer_addr = cpu_to_le64(tx_buffer_info->dma);
		tx_desc->read.cmd_type_len =
			cpu_to_le32(cmd_type_len | tx_buffer_info->length);
		tx_desc->read.olinfo_status = cpu_to_le32(olinfo_status);
		i++;
		if (i == tx_ring->count)
			i = 0;
	}
	tx_ring->next_to_use = i;

	tx_desc->read.cmd_type_len |= cpu_to_le32(IXGBE_TXD_CMD);

	/*
	 * Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64).
	 */
	wmb();

	writel(i, tx_ring->tail);
}

static void ixgbe_atr(struct ixgbe_adapter *adapter, struct sk_buff *skb,
	              u8 queue, u32 tx_flags)
{
	struct ethhdr *eth = (struct ethhdr *)skb->data;
	struct iphdr *iph = ip_hdr(skb);
	struct ixgbe_atr_input atr_input;
	u16 vlan_id, src_port, dst_port;
	u8 l4type = 0;

	/* Right now, we support IPv4 only */
	if (skb->protocol != htons(ETH_P_IP))
		return;

	/* check if we're UDP or TCP */
	if (iph->protocol == IPPROTO_TCP) {
		struct tcphdr *th = tcp_hdr(skb);
		src_port = th->source;
		dst_port = th->dest;
		l4type |= IXGBE_ATR_L4TYPE_TCP;
		/* l4type IPv4 type is 0, no need to assign */
	} else {
		/* Unsupported L4 header, just bail here */
		return;
	}

	memset(&atr_input, 0, sizeof(struct ixgbe_atr_input));

	vlan_id = (tx_flags & IXGBE_TX_FLAGS_VLAN_MASK) >>
	           IXGBE_TX_FLAGS_VLAN_SHIFT;

	ixgbe_atr_set_vlan_id_82599(&atr_input, vlan_id);
	ixgbe_atr_set_src_port_82599(&atr_input, dst_port);
	ixgbe_atr_set_dst_port_82599(&atr_input, src_port);
	ixgbe_atr_set_flex_byte_82599(&atr_input, eth->h_proto);
	ixgbe_atr_set_l4type_82599(&atr_input, l4type);
	/* src and dst are inverted, think how the receiver sees them */
	ixgbe_atr_set_src_ipv4_82599(&atr_input, iph->daddr);
	ixgbe_atr_set_dst_ipv4_82599(&atr_input, iph->saddr);

	/* This assumes the Rx queue and Tx queue are bound to the same CPU */
	ixgbe_fdir_add_signature_filter_82599(&adapter->hw, &atr_input, queue);
}

static int __ixgbe_maybe_stop_tx(struct ixgbe_ring *tx_ring, int size)
{
	netif_stop_subqueue(tx_ring->netdev, tx_ring->queue_index);
	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it. */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (likely(IXGBE_DESC_UNUSED(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! - use start_queue because it doesn't call schedule */
	netif_start_subqueue(tx_ring->netdev, tx_ring->queue_index);
	++tx_ring->tx_stats.restart_queue;
	return 0;
}

static int ixgbe_maybe_stop_tx(struct ixgbe_ring *tx_ring, int size)
{
	if (likely(IXGBE_DESC_UNUSED(tx_ring) >= size))
		return 0;
	return __ixgbe_maybe_stop_tx(tx_ring, size);
}

netdev_tx_t ixgbe_xmit_frame_ring(struct sk_buff *skb,
			  struct ixgbe_adapter *adapter,
			  struct ixgbe_ring *tx_ring)
{
	struct net_device *netdev = tx_ring->netdev;
	int tso;
	int count = 0, tx_map_count = 0;
#ifdef MAX_SKB_FRAGS
#if PAGE_SIZE > IXGBE_MAX_DATA_PER_TXD
	unsigned int f;
#endif
#endif
	u32 tx_flags = 0;
	u16 first;
	u8 hdr_len = 0;

	/*
	 * need: 1 descriptor per page * PAGE_SIZE/IXGBE_MAX_DATA_PER_TXD,
	 *       + 1 desc for skb_head_len/IXGBE_MAX_DATA_PER_TXD,
	 *       + 2 desc gap to keep tail from touching head,
	 *       + 1 desc for context descriptor,
	 * otherwise try next time
	 */
#ifdef MAX_SKB_FRAGS
#if PAGE_SIZE > IXGBE_MAX_DATA_PER_TXD
	for (f = 0; f < skb_shinfo(skb)->nr_frags; f++)
		count += TXD_USE_COUNT(skb_shinfo(skb)->frags[f].size);
#else
	count += skb_shinfo(skb)->nr_frags;
#endif
#endif
	count += TXD_USE_COUNT(skb_headlen(skb));
	if (ixgbe_maybe_stop_tx(tx_ring, count + 3)) {
		tx_ring->tx_stats.tx_busy++;
		return NETDEV_TX_BUSY;
	}

#ifdef NETIF_F_HW_VLAN_TX
	if (adapter->vlgrp && vlan_tx_tag_present(skb)) {
		tx_flags |= vlan_tx_tag_get(skb) << IXGBE_TX_FLAGS_VLAN_SHIFT;
		tx_flags |= IXGBE_TX_FLAGS_VLAN;
	}

#endif
#ifdef HAVE_TX_MQ
	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
		tx_flags &= ~IXGBE_TX_FLAGS_VLAN_PRIO_MASK;
#ifdef IXGBE_FCOE
		/* for FCoE with DCB, we force the priority to what
		 * was specified by the switch */
		if ((adapter->flags & IXGBE_FLAG_FCOE_ENABLED) &&
		    ((skb->protocol == htons(ETH_P_FCOE)) ||
		     (skb->protocol == htons(ETH_P_FIP))))
			tx_flags |= adapter->fcoe.up << 
			            IXGBE_TX_FLAGS_VLAN_PRIO_SHIFT;
		else
#endif /* IXGBE_FCOE */
			tx_flags |= skb->queue_mapping <<
				    IXGBE_TX_FLAGS_VLAN_PRIO_SHIFT;
		tx_flags |= IXGBE_TX_FLAGS_VLAN;
	}

#endif /* HAVE_TX_MQ */
	first = tx_ring->next_to_use;

	if (skb->protocol == htons(ETH_P_IP))
		tx_flags |= IXGBE_TX_FLAGS_IPV4;

#ifdef IXGBE_FCOE
	/* setup tx offload for FCoE */
	else if (skb->protocol == htons(ETH_P_FCOE)) {
		if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED) {
			tso = ixgbe_fso(tx_ring, skb, tx_flags, &hdr_len);
			if (tso < 0) {
				dev_kfree_skb_any(skb);
				return NETDEV_TX_OK;
			}
			if (tso)
				tx_flags |= IXGBE_TX_FLAGS_FSO |
					    IXGBE_TX_FLAGS_FCOE;
			else
				tx_flags |= IXGBE_TX_FLAGS_FCOE;

			goto xmit_fcoe;
		}
	}

#endif /* IXGBE_FCOE */
	if ((tso = ixgbe_tso(tx_ring, skb, tx_flags, &hdr_len)))
		tx_flags |= IXGBE_TX_FLAGS_TSO;
	else if (ixgbe_tx_csum(tx_ring, skb, tx_flags))
		tx_flags |= IXGBE_TX_FLAGS_CSUM;

	if (tso < 0) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* add the ATR filter if ATR is on */
	if (tx_ring->atr_sample_rate) {
		++tx_ring->atr_count;
		if ((tx_ring->atr_count >= tx_ring->atr_sample_rate) &&
		    test_bit(__IXGBE_TX_FDIR_INIT_DONE, &tx_ring->state)) {
			ixgbe_atr(adapter, skb, tx_ring->queue_index, tx_flags);
			tx_ring->atr_count = 0;
		}
	}

#ifdef IXGBE_FCOE
xmit_fcoe:
#endif /* IXGBE_FCOE */
	tx_map_count = ixgbe_tx_map(tx_ring, skb, tx_flags, first, hdr_len);
	if (!tx_map_count) {
		/* handle dma mapping errors in ixgbe_tx_map */
		dev_kfree_skb_any(skb);
		tx_ring->next_to_use = first;
		return NETDEV_TX_OK;
	}

	ixgbe_tx_queue(tx_ring, tx_flags, tx_map_count, skb->len, hdr_len);

	netdev->trans_start = jiffies;

	ixgbe_maybe_stop_tx(tx_ring, DESC_NEEDED);

	return NETDEV_TX_OK;
}

static netdev_tx_t ixgbe_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_ring *tx_ring;

#ifdef HAVE_TX_MQ
	tx_ring = adapter->tx_ring[skb->queue_mapping];
#else
	tx_ring = adapter->tx_ring[0];
#endif
	return ixgbe_xmit_frame_ring(skb, adapter, tx_ring);
}

/**
 * ixgbe_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the timer callback.
 **/
static struct net_device_stats *ixgbe_get_stats(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);

	/* only return the current stats */
	return &adapter->net_stats;
}

/**
 * ixgbe_set_mac - Change the Ethernet Address of the NIC
 * @netdev: network interface device structure
 * @p: pointer to an address structure
 *
 * Returns 0 on success, negative on failure
 **/
static int ixgbe_set_mac(struct net_device *netdev, void *p)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	struct ixgbe_hw *hw = &adapter->hw;
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	memcpy(hw->mac.addr, addr->sa_data, netdev->addr_len);

	if (hw->mac.ops.set_rar)
		hw->mac.ops.set_rar(hw, 0, hw->mac.addr, 0, IXGBE_RAH_AV);

	return 0;
}

#if defined(HAVE_NETDEV_STORAGE_ADDRESS) && defined(NETDEV_HW_ADDR_T_SAN)
/**
 * ixgbe_add_sanmac_netdev - Add the SAN MAC address to the corresponding
 * netdev->dev_addr_list
 * @netdev: network interface device structure
 *
 * Returns non-zero on failure
 **/
static int ixgbe_add_sanmac_netdev(struct net_device *dev)
{
	int err = 0;
	struct ixgbe_adapter *adapter = netdev_priv(dev);
	struct ixgbe_mac_info *mac = &adapter->hw.mac;

	if (is_valid_ether_addr(mac->san_addr)) {
		rtnl_lock();
		err = dev_addr_add(dev, mac->san_addr, NETDEV_HW_ADDR_T_SAN);
		rtnl_unlock();
	}
	return err;
}

/**
 * ixgbe_del_sanmac_netdev - Removes the SAN MAC address to the corresponding
 * netdev->dev_addr_list
 * @netdev: network interface device structure
 *
 * Returns non-zero on failure
 **/
static int ixgbe_del_sanmac_netdev(struct net_device *dev)
{
	int err = 0;
	struct ixgbe_adapter *adapter = netdev_priv(dev);
	struct ixgbe_mac_info *mac = &adapter->hw.mac;

	if (is_valid_ether_addr(mac->san_addr)) {
		rtnl_lock();
		err = dev_addr_del(dev, mac->san_addr, NETDEV_HW_ADDR_T_SAN);
		rtnl_unlock();
	}
	return err;
}

#endif /* (HAVE_NETDEV_STORAGE_ADDRESS) && defined(NETDEV_HW_ADDR_T_SAN) */
#ifdef ETHTOOL_OPS_COMPAT
/**
 * ixgbe_ioctl -
 * @netdev:
 * @ifreq:
 * @cmd:
 **/
static int ixgbe_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	switch (cmd) {
	case SIOCETHTOOL:
		return ethtool_ioctl(ifr);
	default:
		return -EOPNOTSUPP;
	}
}

#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void ixgbe_netpoll(struct net_device *netdev)
{
	struct ixgbe_adapter *adapter = netdev_priv(netdev);
	int i;

	/* if interface is down do nothing */
	if (test_bit(__IXGBE_DOWN, &adapter->state))
		return;

#ifndef CONFIG_IXGBE_NAPI
	ixgbe_irq_disable(adapter);
#endif
	adapter->flags |= IXGBE_FLAG_IN_NETPOLL;
	if (adapter->flags & IXGBE_FLAG_MSIX_ENABLED) {
		int num_q_vectors = adapter->num_msix_vectors - NON_Q_VECTORS;
		for (i = 0; i < num_q_vectors; i++) {
			struct ixgbe_q_vector *q_vector = adapter->q_vector[i];
			ixgbe_msix_clean_many(0, q_vector);
		}
	} else {
		ixgbe_intr(adapter->pdev->irq, netdev);
	}
	adapter->flags &= ~IXGBE_FLAG_IN_NETPOLL;
#ifndef CONFIG_IXGBE_NAPI
	ixgbe_irq_enable(adapter, true, true);
#endif
}

#endif
#ifdef HAVE_NETDEV_SELECT_QUEUE
static u16 ixgbe_select_queue(struct net_device *dev, struct sk_buff *skb)
{
	struct ixgbe_adapter *adapter = netdev_priv(dev);
	int txq = smp_processor_id();

#ifdef IXGBE_FCOE
	if ((skb->protocol == htons(ETH_P_FCOE)) ||
	    (skb->protocol == htons(ETH_P_FIP))) {
		if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED) {
			txq &= (adapter->ring_feature[RING_F_FCOE].indices - 1);
			txq += adapter->ring_feature[RING_F_FCOE].mask;
			return txq;
		} else if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
			txq = adapter->fcoe.up;
			return txq;
		}
	}

#endif /* IXGBE_FCOE */
	if (adapter->flags & IXGBE_FLAG_FDIR_HASH_CAPABLE) {
		while (unlikely(txq >= dev->real_num_tx_queues))
			txq -= dev->real_num_tx_queues;
		return txq;
	}

	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED) {
		if (skb->priority == TC_PRIO_CONTROL)
			txq = adapter->ring_feature[RING_F_DCB].indices - 1;
		else
			txq = (skb->vlan_tci & IXGBE_TX_FLAGS_VLAN_PRIO_MASK)
			       >> 13;
		return txq;
	}
	return skb_tx_hash(dev, skb);
}

#endif /* HAVE_NETDEV_SELECT_QUEUE */
#ifdef HAVE_NET_DEVICE_OPS
static const struct net_device_ops ixgbe_netdev_ops = {
	.ndo_open		= &ixgbe_open,
	.ndo_stop		= &ixgbe_close,
	.ndo_start_xmit		= &ixgbe_xmit_frame,
	.ndo_get_stats		= &ixgbe_get_stats,
	.ndo_set_rx_mode	= &ixgbe_set_rx_mode,
	.ndo_set_multicast_list	= &ixgbe_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= &ixgbe_set_mac,
	.ndo_change_mtu		= &ixgbe_change_mtu,
#ifdef ETHTOOL_OPS_COMPAT
	.ndo_do_ioctl		= &ixgbe_ioctl,
#endif
	.ndo_tx_timeout		= &ixgbe_tx_timeout,
	.ndo_vlan_rx_register	= &ixgbe_vlan_rx_register,
	.ndo_vlan_rx_add_vid	= &ixgbe_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= &ixgbe_vlan_rx_kill_vid,
#ifdef HAVE_IPLINK_VF_CONFIG
	.ndo_set_vf_mac		= ixgbe_ndo_set_vf_mac,
	.ndo_set_vf_vlan	= ixgbe_ndo_set_vf_vlan,
	.ndo_set_vf_tx_rate	= ixgbe_ndo_set_vf_bw,
	.ndo_get_vf_config	= ixgbe_ndo_get_vf_config,
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= &ixgbe_netpoll,
#endif
	.ndo_select_queue	= &ixgbe_select_queue,
#ifdef IXGBE_FCOE
	.ndo_fcoe_ddp_setup = ixgbe_fcoe_ddp_get,
	.ndo_fcoe_ddp_done = ixgbe_fcoe_ddp_put,
#ifdef HAVE_NETDEV_OPS_FCOE_ENABLE
	.ndo_fcoe_enable = ixgbe_fcoe_enable,
	.ndo_fcoe_disable = ixgbe_fcoe_disable,
#endif
#ifdef HAVE_NETDEV_OPS_FCOE_GETWWN
	.ndo_fcoe_get_wwn = ixgbe_fcoe_get_wwn,
#endif
#endif /* IXGBE_FCOE */
};

#endif /* HAVE_NET_DEVICE_OPS */



void ixgbe_assign_netdev_ops(struct net_device *dev)
{
	struct ixgbe_adapter *adapter;
	adapter = netdev_priv(dev);
#ifdef HAVE_NET_DEVICE_OPS
	dev->netdev_ops = &ixgbe_netdev_ops;
#else /* HAVE_NET_DEVICE_OPS */
	dev->open = &ixgbe_open;
	dev->stop = &ixgbe_close;
	dev->hard_start_xmit = &ixgbe_xmit_frame;
	dev->get_stats = &ixgbe_get_stats;
#ifdef HAVE_SET_RX_MODE
	dev->set_rx_mode = &ixgbe_set_rx_mode;
#endif
	dev->set_multicast_list = &ixgbe_set_rx_mode;
	dev->set_mac_address = &ixgbe_set_mac;
	dev->change_mtu = &ixgbe_change_mtu;
#ifdef ETHTOOL_OPS_COMPAT
	dev->do_ioctl = &ixgbe_ioctl;
#endif
#ifdef HAVE_TX_TIMEOUT
	dev->tx_timeout = &ixgbe_tx_timeout;
#endif
#ifdef NETIF_F_HW_VLAN_TX
	dev->vlan_rx_register = &ixgbe_vlan_rx_register;
	dev->vlan_rx_add_vid = &ixgbe_vlan_rx_add_vid;
	dev->vlan_rx_kill_vid = &ixgbe_vlan_rx_kill_vid;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	dev->poll_controller = &ixgbe_netpoll;
#endif
#ifdef HAVE_NETDEV_SELECT_QUEUE
	dev->select_queue = &ixgbe_select_queue;
#endif /* HAVE_NETDEV_SELECT_QUEUE */
#endif /* HAVE_NET_DEVICE_OPS */
	ixgbe_set_ethtool_ops(dev);
	dev->watchdog_timeo = 5 * HZ;
}

static void __devinit ixgbe_probe_vf(struct ixgbe_adapter *adapter)
{
#ifdef CONFIG_PCI_IOV
	int err;

	err = pci_enable_sriov(adapter->pdev, adapter->num_vfs);
	if (err) {
		DPRINTK(PROBE, ERR,
			"Failed to enable PCI sriov: %d\n", err);
		goto err_novfs;
	}
	/* If call to enable VFs succeeded then allocate memory
	 * for per VF control structures.
	 */
	adapter->vfinfo =
		kcalloc(adapter->num_vfs,
			sizeof(struct vf_data_storage), GFP_KERNEL);
	if (adapter->vfinfo) {
		adapter->l2switch_enable = true;
		adapter->repl_enable = true;

		/* RSS not compatible with SR-IOV operation */
		adapter->flags &= ~IXGBE_FLAG_RSS_ENABLED;

		/* Disable RSC when in SR-IOV mode */
		adapter->flags2 &= ~(IXGBE_FLAG2_RSC_CAPABLE |
				     IXGBE_FLAG2_RSC_ENABLED);

		adapter->flags &= ~(IXGBE_FLAG_RX_PS_ENABLED |
				    IXGBE_FLAG_RX_PS_CAPABLE);

		return;
	}

	/* Oh oh */
	DPRINTK(PROBE, ERR,
		"Unable to allocate memory for VF "
		"Data Storage - SRIOV disabled\n");
	pci_disable_sriov(adapter->pdev);

err_novfs:
	adapter->flags &= ~IXGBE_FLAG_SRIOV_ENABLED;
	adapter->num_vfs = 0;
#endif /* CONFIG_PCI_IOV */
}

/**
 * ixgbe_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in ixgbe_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * ixgbe_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int __devinit ixgbe_probe(struct pci_dev *pdev,
                                 const struct pci_device_id *ent)
{
	struct net_device *netdev;
	struct ixgbe_adapter *adapter = NULL;
	struct ixgbe_hw *hw = NULL;
	static int cards_found;
	int i, err, pci_using_dac;
#ifdef HAVE_TX_MQ
	unsigned int indices;
#endif
	u32 part_num;
	u8 part_str[IXGBE_PBA_LEN];
	u32 part_str_size = IXGBE_PBA_LEN;
	enum ixgbe_mac_type mac_type = ixgbe_mac_unknown;
#ifdef IXGBE_FCOE
	u16 device_caps;
#endif

	err = pci_enable_device(pdev);
	if (err)
		return err;

	if (!dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64)) &&
	    !dma_set_coherent_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(64))) {
		pci_using_dac = 1;
	} else {
		err = dma_set_mask(pci_dev_to_dev(pdev), DMA_BIT_MASK(32));
		if (err) {
			err = dma_set_coherent_mask(pci_dev_to_dev(pdev),
			                                  DMA_BIT_MASK(32));
			if (err) {
				dev_err(pci_dev_to_dev(pdev), "No usable DMA "
				        "configuration, aborting\n");
				goto err_dma;
			}
		}
		pci_using_dac = 0;
	}

	err = pci_request_regions(pdev, ixgbe_driver_name);
	if (err) {
		dev_err(pci_dev_to_dev(pdev),
			"pci_request_regions failed 0x%x\n", err);
		goto err_pci_reg;
	}

	/*
	 * The mac_type is needed before we have the adapter is  set up
	 * so rather than maintain two devID -> MAC tables we dummy up
	 * an ixgbe_hw stuct and use ixgbe_set_mac_type.
	 */
	hw = vmalloc(sizeof(struct ixgbe_hw));
	if (!hw) {
		printk(KERN_INFO "Unable to allocate memory for early mac "
			"check\n");
	} else {
		hw->vendor_id = pdev->vendor;
		hw->device_id = pdev->device;
		ixgbe_set_mac_type(hw);
		mac_type = hw->mac.type;
		vfree(hw);
	}

	/*
	 * Workaround of Silicon errata on 82598. Disable LOs in the PCI switch
	 * port to which the 82598 is connected to prevent duplicate
	 * completions caused by LOs.  We need the mac type so that we only
	 * do this on 82598 devices, ixgbe_set_mac_type does this for us if
	 * we set it's device ID.
	 */
	if (mac_type == ixgbe_mac_82598EB)
		pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S);

	pci_enable_pcie_error_reporting(pdev);

	pci_set_master(pdev);

#ifdef HAVE_TX_MQ
	indices = num_possible_cpus();
	if (mac_type == ixgbe_mac_unknown)
		indices = max_t(unsigned int, IXGBE_MAX_RSS_INDICES,
				IXGBE_MAX_FDIR_INDICES);
	else if (mac_type == ixgbe_mac_82598EB)
		indices = min_t(unsigned int, indices, IXGBE_MAX_RSS_INDICES);
	else
		indices = min_t(unsigned int, indices, IXGBE_MAX_FDIR_INDICES);
	indices = max_t(unsigned int, indices, IXGBE_MAX_DCB_INDICES);
#ifdef IXGBE_FCOE
	indices += min_t(unsigned int, num_possible_cpus(),
			  IXGBE_MAX_FCOE_INDICES);
#endif
	netdev = alloc_etherdev_mq(sizeof(struct ixgbe_adapter), indices);
#else
	netdev = alloc_etherdev(sizeof(struct ixgbe_adapter));
#endif
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	adapter = netdev_priv(netdev);
	pci_set_drvdata(pdev, adapter);

	adapter->netdev = netdev;
	adapter->pdev = pdev;
	hw = &adapter->hw;
	hw->back = adapter;
	adapter->msg_enable = (1 << DEFAULT_DEBUG_LEVEL_SHIFT) - 1;

#ifdef HAVE_DEVICE_NUMA_NODE
	DPRINTK(TX_ERR, INFO, "my (original) node was: %d\n",
	        dev_to_node(&pdev->dev));
#endif /* HAVE_DEVICE_NUMA_NODE */

#ifdef HAVE_PCI_ERS
	/*
	 * call save state here in standalone driver because it relies on
	 * adapter struct to exist, and needs to call netdev_priv
	 */
	pci_save_state(pdev);

#endif
	hw->hw_addr = ioremap(pci_resource_start(pdev, 0),
	                      pci_resource_len(pdev, 0));
	if (!hw->hw_addr) {
		err = -EIO;
		goto err_ioremap;
	}

	ixgbe_assign_netdev_ops(netdev);

	strcpy(netdev->name, pci_name(pdev));

	adapter->bd_number = cards_found;

#ifdef IXGBE_TCP_TIMER
	adapter->msix_addr = ioremap(pci_resource_start(pdev, 3),
	                             pci_resource_len(pdev, 3));
	if (!adapter->msix_addr) {
		err = -EIO;
		printk("Error in ioremap of BAR3\n");
		goto err_map_msix;
	}

#endif
	/* set up this timer and work struct before calling get_invariants
	 * which might start the timer
	 */
	setup_timer(&adapter->sfp_timer, &ixgbe_sfp_timer,
	            (unsigned long) adapter);
	INIT_WORK(&adapter->sfp_task, ixgbe_sfp_task);

	/* multispeed fiber has its own tasklet, called from GPI SDP1 context */
	INIT_WORK(&adapter->multispeed_fiber_task, ixgbe_multispeed_fiber_task);

	/* a new SFP+ module arrival, called from GPI SDP2 context */
	INIT_WORK(&adapter->sfp_config_module_task,
	          ixgbe_sfp_config_module_task);

	/* setup the private structure */
	err = ixgbe_sw_init(adapter);
	if (err)
		goto err_sw_init;

	/* Make it possible the adapter to be woken up via WOL */
	if (adapter->hw.mac.type == ixgbe_mac_82599EB)
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_WUS, ~0);

	/*
	 * If we have a fan, this is as early we know, warn if we
	 * have had a failure.
	 */
	if (adapter->flags & IXGBE_FLAG_FAN_FAIL_CAPABLE) {
		u32 esdp = IXGBE_READ_REG(hw, IXGBE_ESDP);
		if (esdp & IXGBE_ESDP_SDP1)
			DPRINTK(PROBE, CRIT,
				"Fan has stopped, replace the adapter\n");
	}

	/* reset_hw fills in the perm_addr as well */
	hw->phy.reset_if_overtemp = true;
	err = hw->mac.ops.reset_hw(hw);
	hw->phy.reset_if_overtemp = false;
	if (err == IXGBE_ERR_SFP_NOT_PRESENT &&
	    hw->mac.type == ixgbe_mac_82598EB) {
		/*
		 * Start a kernel thread to watch for a module to arrive.
		 * Only do this for 82598, since 82599 will generate interrupts
		 * on module arrival.
		 */
		set_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state);
		mod_timer(&adapter->sfp_timer,
		          round_jiffies(jiffies + (2 * HZ)));
		err = 0;
	} else if (err == IXGBE_ERR_SFP_NOT_SUPPORTED) {
		DPRINTK(PROBE, ERR, "failed to load because an "
		        "unsupported SFP+ module type was detected.\n");
		goto err_sw_init;
	} else if (err) {
		DPRINTK(PROBE, ERR, "HW Init failed: %d\n", err);
		goto err_sw_init;
	}

	/*
	 * check_options must be called before setup_link to set up
	 * hw->fc completely
	 */
	ixgbe_check_options(adapter);

	DPRINTK(TX_ERR, INFO, "my (preferred) node is: %d\n", adapter->node);

	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
		ixgbe_probe_vf(adapter);

#ifdef MAX_SKB_FRAGS
#ifdef NETIF_F_HW_VLAN_TX
	netdev->features = NETIF_F_SG |
			   NETIF_F_IP_CSUM |
			   NETIF_F_HW_VLAN_TX |
			   NETIF_F_HW_VLAN_RX |
			   NETIF_F_HW_VLAN_FILTER;

#else
	netdev->features = NETIF_F_SG | NETIF_F_IP_CSUM;

#endif
#ifdef NETIF_F_IPV6_CSUM
	netdev->features |= NETIF_F_IPV6_CSUM;
#endif
#ifdef NETIF_F_TSO
	netdev->features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
	netdev->features |= NETIF_F_TSO6;
#endif /* NETIF_F_TSO6 */
#endif /* NETIF_F_TSO */
#ifdef NETIF_F_GRO
	netdev->features |= NETIF_F_GRO;
#endif /* NETIF_F_GRO */
#ifdef NETIF_F_NTUPLE
	/*
	 * If perfect filters were enabled in check_options(), enable them
	 * on the netdevice too.
	 */
	if (adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE)
		netdev->features |= NETIF_F_NTUPLE;
#endif /* NETIF_F_NTUPLE */
	if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED)
		adapter->flags &= ~IXGBE_FLAG_RSS_ENABLED;
	if (adapter->flags & IXGBE_FLAG_DCB_ENABLED)
		adapter->flags &= ~IXGBE_FLAG_RSS_ENABLED;
	if (adapter->flags & IXGBE_FLAG_VMDQ_ENABLED) {
		adapter->flags &= ~(IXGBE_FLAG_FDIR_HASH_CAPABLE
				    | IXGBE_FLAG_FDIR_PERFECT_CAPABLE);
#ifdef NETIF_F_NTUPLE
		/* clear n-tuple support in the netdev unconditionally */
		netdev->features &= ~NETIF_F_NTUPLE;
#endif /* NETIF_F_NTUPLE */
	}
	if (adapter->flags2 & IXGBE_FLAG2_RSC_CAPABLE) {
		netdev->features |= NETIF_F_LRO;
		adapter->flags2 &= ~IXGBE_FLAG2_SWLRO_ENABLED;
#ifndef IXGBE_NO_HW_RSC
		adapter->flags2 |= IXGBE_FLAG2_RSC_ENABLED;
#else
		netdev->features |= NETIF_F_LRO;
		adapter->flags2 |= IXGBE_FLAG2_SWLRO_ENABLED;
#endif
	} else {
#ifndef IXGBE_NO_LRO
		netdev->features |= NETIF_F_LRO;
		adapter->flags2 |= IXGBE_FLAG2_SWLRO_ENABLED;
#endif
		adapter->flags2 &= ~IXGBE_FLAG2_RSC_ENABLED;
	}
#ifdef HAVE_NETDEV_VLAN_FEATURES
#ifdef NETIF_F_TSO
	netdev->vlan_features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
	netdev->vlan_features |= NETIF_F_TSO6;
#endif /* NETIF_F_TSO6 */
#endif /* NETIF_F_TSO */
	netdev->vlan_features |= NETIF_F_IP_CSUM;
#ifdef NETIF_F_IPV6_CSUM
	netdev->vlan_features |= NETIF_F_IPV6_CSUM;
#endif
	netdev->vlan_features |= NETIF_F_SG;

#endif /* HAVE_NETDEV_VLAN_FEATURES */
#ifdef CONFIG_DCB
	netdev->dcbnl_ops = &dcbnl_ops;
#endif

#ifdef IXGBE_FCOE
	switch (adapter->hw.mac.type) {
	case ixgbe_mac_82599EB:
#ifdef NETIF_F_FSO
		if (adapter->flags & IXGBE_FLAG_FCOE_CAPABLE) {
			ixgbe_get_device_caps(hw, &device_caps);
			if (device_caps & IXGBE_DEVICE_CAPS_FCOE_OFFLOADS) {
				adapter->flags &= ~IXGBE_FLAG_FCOE_ENABLED;
				adapter->flags &= ~IXGBE_FLAG_FCOE_CAPABLE;
				DPRINTK(PROBE, INFO, "FCoE offload feature "
					"is not available. Disabling FCoE "
					"offload feature\n");
			}
#ifndef HAVE_NETDEV_OPS_FCOE_ENABLE
			else {
				adapter->flags |= IXGBE_FLAG_FCOE_ENABLED;
				adapter->ring_feature[RING_F_FCOE].indices =
				        IXGBE_FCRETA_SIZE;
				netdev->features |= NETIF_F_FSO;
				netdev->features |= NETIF_F_FCOE_CRC;
				netdev->features |= NETIF_F_FCOE_MTU;
				netdev->fcoe_ddp_xid = IXGBE_FCOE_DDP_MAX - 1;
				DPRINTK(PROBE, INFO, "Enabling FCoE offload "
					"feature\n");
			}
#endif /* HAVE_NETDEV_OPS_FCOE_ENABLE */
		}
#ifdef HAVE_NETDEV_VLAN_FEATURES
		if (adapter->flags & IXGBE_FLAG_FCOE_CAPABLE) {
			netdev->vlan_features |= NETIF_F_FSO;
			netdev->vlan_features |= NETIF_F_FCOE_CRC;
			netdev->vlan_features |= NETIF_F_FCOE_MTU;
		}
#endif /* HAVE_NETDEV_VLAN_FEATURES */
#endif /* NETIF_F_FSO */
		break;
	default:
		break;
	}
#endif /* IXGBE_FCOE */
	if (pci_using_dac)
		netdev->features |= NETIF_F_HIGHDMA;

#endif /* MAX_SKB_FRAGS */
	/* make sure the EEPROM is good */
	if (hw->eeprom.ops.validate_checksum &&
	    (hw->eeprom.ops.validate_checksum(hw, NULL) < 0)) {
		DPRINTK(PROBE, ERR, "The EEPROM Checksum Is Not Valid\n");
		err = -EIO;
		goto err_sw_init;
	}

	memcpy(netdev->dev_addr, hw->mac.perm_addr, netdev->addr_len);
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, hw->mac.perm_addr, netdev->addr_len);

	if (ixgbe_validate_mac_addr(netdev->perm_addr)) {
		DPRINTK(PROBE, INFO, "invalid MAC address\n");
		err = -EIO;
		goto err_sw_init;
	}
#else
	if (ixgbe_validate_mac_addr(netdev->dev_addr)) {
		DPRINTK(PROBE, INFO, "invalid MAC address\n");
		err = -EIO;
		goto err_sw_init;
	}
#endif

	/* power down the optics */
	if (hw->phy.multispeed_fiber)
		ixgbe_disable_tx_laser(hw);

	setup_timer(&adapter->watchdog_timer, &ixgbe_watchdog,
	            (unsigned long) adapter);

	INIT_WORK(&adapter->reset_task, ixgbe_reset_task);
	INIT_WORK(&adapter->watchdog_task, ixgbe_watchdog_task);

	err = ixgbe_init_interrupt_scheme(adapter);
	if (err)
		goto err_sw_init;

	switch (pdev->device) {
	case IXGBE_DEV_ID_82599_COMBO_BACKPLANE:
		/* All except this subdevice support WOL */
		if (pdev->subsystem_device ==
		    IXGBE_SUBDEV_ID_82599_KX4_KR_MEZZ) {
			adapter->wol = 0;
			break;
		}
	case IXGBE_DEV_ID_82599_KX4:
		adapter->wol = (IXGBE_WUFC_MAG | IXGBE_WUFC_EX |
		                IXGBE_WUFC_MC | IXGBE_WUFC_BC);
		break;
	default:
		adapter->wol = 0;
		break;
	}
	device_set_wakeup_enable(&adapter->pdev->dev, adapter->wol);

	/* save off EEPROM version number */
	ixgbe_read_eeprom(hw, 0x29, &adapter->eeprom_version);

	/* reset the hardware with the new settings */
	err = hw->mac.ops.start_hw(hw);
	if (err == IXGBE_ERR_EEPROM_VERSION) {
		/* We are running on a pre-production device, log a warning */
		DPRINTK(PROBE, INFO, "This device is a pre-production adapter/"
		        "LOM.  Please be aware there may be issues associated "
		        "with your hardware.  If you are experiencing problems "
		        "please contact your Intel or hardware representative "
		        "who provided you with this hardware.\n");
	}
	/* pick up the PCI bus settings for reporting later */
	if (hw->mac.ops.get_bus_info)
		hw->mac.ops.get_bus_info(hw);


	strcpy(netdev->name, "eth%d");
	err = register_netdev(netdev);
	if (err)
		goto err_register;

	adapter->netdev_registered = true;
	/* carrier off reporting is important to ethtool even BEFORE open */
	netif_carrier_off(netdev);
	/* keep stopping all the transmit queues for older kernels */
	netif_tx_stop_all_queues(netdev);

#ifdef HAVE_TX_MQ
	INIT_WORK(&adapter->fdir_reinit_task, ixgbe_fdir_reinit_task);
#endif /* HAVE_TX_MQ */
	INIT_WORK(&adapter->check_overtemp_task, ixgbe_check_overtemp_task);
	if (adapter->flags & IXGBE_FLAG_DCA_CAPABLE) {
		err = dca_add_requester(&pdev->dev);
		switch (err) {
		case 0:
			adapter->flags |= IXGBE_FLAG_DCA_ENABLED;
			ixgbe_setup_dca(adapter);
			break;
		/* -19 is returned from the kernel when no provider is found */
		case -19:
			DPRINTK(PROBE, INFO, "No DCA provider found.  Please "
			        "start ioatdma for DCA functionality.\n");
			break;
		default:
			DPRINTK(PROBE, INFO, "DCA registration failed: %d\n",
			        err);
			break;
		}
	}

	/* print all messages at the end so that we use our eth%d name */
	/* print bus type/speed/width info */
	DPRINTK(PROBE, INFO, "(PCI Express:%s:%s) ",
		((hw->bus.speed == ixgbe_bus_speed_5000) ? "5.0Gb/s":
		 (hw->bus.speed == ixgbe_bus_speed_2500) ? "2.5Gb/s":"Unknown"),
		 (hw->bus.width == ixgbe_bus_width_pcie_x8) ? "Width x8" :
		 (hw->bus.width == ixgbe_bus_width_pcie_x4) ? "Width x4" :
		 (hw->bus.width == ixgbe_bus_width_pcie_x1) ? "Width x1" :
		 ("Unknown"));

	/* print the MAC address */
	for (i = 0; i < 6; i++)
		printk("%2.2x%c", netdev->dev_addr[i], i == 5 ? '\n' : ':');

	/* Frist try to read PBA as a string */
	err = ixgbe_read_pba_string(hw, part_str, &part_str_size);
	switch (err) {
	case 0:
		break;
	case IXGBE_NOT_IMPLEMENTED:
		/* old style PBA number */
		ixgbe_read_pba_num(hw, &part_num);
		sprintf(part_str, "%06x-%03x\n", (part_num >> 8),
		        (part_num & 0xff));
		break;
	default:
		strcpy(part_str, "Unknown");
		break;
	}
	if (ixgbe_is_sfp(hw) && hw->phy.sfp_type != ixgbe_sfp_type_not_present)
		DPRINTK(PROBE, INFO, "MAC: %d, PHY: %d, SFP+: %d, PBA No: %s\n",
		        hw->mac.type, hw->phy.type, hw->phy.sfp_type,
		        part_str);
	else
		DPRINTK(PROBE, INFO, "MAC: %d, PHY: %d, PBA No: %s\n",
		        hw->mac.type, hw->phy.type, part_str);

	if (((hw->bus.speed == ixgbe_bus_speed_2500) &&
	     (hw->bus.width <= ixgbe_bus_width_pcie_x4)) ||
	    (hw->bus.width <= ixgbe_bus_width_pcie_x2)) {
		DPRINTK(PROBE, WARNING, "PCI-Express bandwidth available for "
			"this card is not sufficient for optimal "
			"performance.\n");
		DPRINTK(PROBE, WARNING, "For optimal performance a x8 "
			"PCI-Express slot is required.\n");
	}

#ifdef NETIF_F_GRO
	if (adapter->netdev->features & NETIF_F_GRO)
		DPRINTK(PROBE, INFO, "GRO is enabled\n");
	else if (adapter->flags2 & IXGBE_FLAG2_SWLRO_ENABLED)
#else
	if (adapter->flags2 & IXGBE_FLAG2_SWLRO_ENABLED)
#endif
		DPRINTK(PROBE, INFO, "Internal LRO is enabled \n");
	else
		DPRINTK(PROBE, INFO, "LRO is disabled \n");

	if (adapter->flags2 & IXGBE_FLAG2_RSC_ENABLED)
		DPRINTK(PROBE, INFO, "HW RSC is enabled \n");
#ifdef CONFIG_PCI_IOV
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED) {
		DPRINTK(PROBE, INFO, "IOV is enabled with %d VFs\n",
			adapter->num_vfs);
		for (i = 0; i < adapter->num_vfs; i++)
			ixgbe_vf_configuration(pdev, (i | 0x10000000));
	}
#endif

#if defined(HAVE_NETDEV_STORAGE_ADDRESS) && defined(NETDEV_HW_ADDR_T_SAN)
	/* add san mac addr to netdev */
	ixgbe_add_sanmac_netdev(netdev);

#endif /* (HAVE_NETDEV_STORAGE_ADDRESS) && (NETDEV_HW_ADDR_T_SAN) */
	DPRINTK(PROBE, INFO, "Intel(R) 10 Gigabit Network Connection\n");
	cards_found++;
	return 0;

err_register:
	ixgbe_clear_interrupt_scheme(adapter);
	ixgbe_release_hw_control(adapter);
err_sw_init:
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
		ixgbe_disable_sriov(adapter);
	clear_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state);
	del_timer_sync(&adapter->sfp_timer);
	cancel_work_sync(&adapter->sfp_task);
	cancel_work_sync(&adapter->multispeed_fiber_task);
	cancel_work_sync(&adapter->sfp_config_module_task);
#ifdef IXGBE_TCP_TIMER
	iounmap(adapter->msix_addr);
err_map_msix:
#endif
	iounmap(hw->hw_addr);
err_ioremap:
	free_netdev(netdev);
err_alloc_etherdev:
	pci_release_regions(pdev);
err_pci_reg:
err_dma:

	pci_disable_device(pdev);
	return err;
}

/**
 * ixgbe_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * ixgbe_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void __devexit ixgbe_remove(struct pci_dev *pdev)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;

	set_bit(__IXGBE_DOWN, &adapter->state);
	/*
	 * clear the module not found bit to make sure the worker won't
	 * reschedule
	 */
	clear_bit(__IXGBE_SFP_MODULE_NOT_FOUND, &adapter->state);
	del_timer_sync(&adapter->watchdog_timer);
	del_timer_sync(&adapter->sfp_timer);
	cancel_work_sync(&adapter->reset_task);
	cancel_work_sync(&adapter->watchdog_task);
	cancel_work_sync(&adapter->sfp_task);
#ifdef HAVE_TX_MQ
	cancel_work_sync(&adapter->fdir_reinit_task);
#endif
	cancel_work_sync(&adapter->check_overtemp_task);
	cancel_work_sync(&adapter->multispeed_fiber_task);
	cancel_work_sync(&adapter->sfp_config_module_task);
	flush_scheduled_work();

	if (adapter->flags & IXGBE_FLAG_DCA_ENABLED) {
		adapter->flags &= ~IXGBE_FLAG_DCA_ENABLED;
		dca_remove_requester(&pdev->dev);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_DCA_CTRL, 1);
	}

#ifdef IXGBE_FCOE
	if (adapter->flags & IXGBE_FLAG_FCOE_ENABLED)
		ixgbe_cleanup_fcoe(adapter);

#endif /* IXGBE_FCOE */
#if defined(HAVE_NETDEV_STORAGE_ADDRESS) && defined(NETDEV_HW_ADDR_T_SAN)
	/* remove the added san mac */
	ixgbe_del_sanmac_netdev(netdev);

#endif /* (HAVE_NETDEV_STORAGE_ADDRESS) && (NETDEV_HW_ADDR_T_SAN) */
	if (adapter->netdev_registered) {
		unregister_netdev(netdev);
		adapter->netdev_registered = false;
	}
	if (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED)
		ixgbe_disable_sriov(adapter);

	ixgbe_clear_interrupt_scheme(adapter);
	ixgbe_release_hw_control(adapter);

#ifdef IXGBE_TCP_TIMER
	iounmap(adapter->msix_addr);
#endif
	iounmap(adapter->hw.hw_addr);
	pci_release_regions(pdev);

	DPRINTK(PROBE, INFO, "complete\n");
	free_netdev(netdev);

	pci_disable_pcie_error_reporting(pdev);

	pci_disable_device(pdev);
}

u16 ixgbe_read_pci_cfg_word(struct ixgbe_hw *hw, u32 reg)
{
	u16 value;
	struct ixgbe_adapter *adapter = hw->back;

	pci_read_config_word(adapter->pdev, reg, &value);
	return value;
}

void ixgbe_write_pci_cfg_word(struct ixgbe_hw *hw, u32 reg, u16 value)
{
	struct ixgbe_adapter *adapter = hw->back;

	pci_write_config_word(adapter->pdev, reg, value);
}

#ifdef HAVE_PCI_ERS
/**
 * ixgbe_io_error_detected - called when PCI error is detected
 * @pdev: Pointer to PCI device
 * @state: The current pci connection state
 *
 * This function is called after a PCI bus error affecting
 * this device has been detected.
 */
static pci_ers_result_t ixgbe_io_error_detected(struct pci_dev *pdev,
                                                pci_channel_state_t state)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;

	netif_device_detach(netdev);

	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;

	if (netif_running(netdev))
		ixgbe_down(adapter);
	pci_disable_device(pdev);

	/* Request a slot reset. */
	return PCI_ERS_RESULT_NEED_RESET;
}

/**
 * ixgbe_io_slot_reset - called after the pci bus has been reset.
 * @pdev: Pointer to PCI device
 *
 * Restart the card from scratch, as if from a cold-boot.
 */
static pci_ers_result_t ixgbe_io_slot_reset(struct pci_dev *pdev)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	pci_ers_result_t result;

	if (pci_enable_device(pdev)) {
		DPRINTK(PROBE, ERR,
			"Cannot re-enable PCI device after reset.\n");
		result = PCI_ERS_RESULT_DISCONNECT;
	} else {
		pci_set_master(pdev);
		pci_restore_state(pdev);
		/*
		 * After second error pci->state_saved is false, this
		 * resets it so EEH doesn't break.
		 */
		pci_save_state(pdev);

		pci_wake_from_d3(pdev, false);

		ixgbe_reset(adapter);
		IXGBE_WRITE_REG(&adapter->hw, IXGBE_WUS, ~0);
		result = PCI_ERS_RESULT_RECOVERED;
	}

	pci_cleanup_aer_uncorrect_error_status(pdev);

	return result;
}

/**
 * ixgbe_io_resume - called when traffic can start flowing again.
 * @pdev: Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation.
 */
static void ixgbe_io_resume(struct pci_dev *pdev)
{
	struct ixgbe_adapter *adapter = pci_get_drvdata(pdev);
	struct net_device *netdev = adapter->netdev;

	if (netif_running(netdev)) {
		if (ixgbe_up(adapter)) {
			DPRINTK(PROBE, INFO, "ixgbe_up failed after reset\n");
			return;
		}
	}

	netif_device_attach(netdev);
}

static struct pci_error_handlers ixgbe_err_handler = {
	.error_detected = ixgbe_io_error_detected,
	.slot_reset = ixgbe_io_slot_reset,
	.resume = ixgbe_io_resume,
};

#endif
static struct pci_driver ixgbe_driver = {
	.name     = ixgbe_driver_name,
	.id_table = ixgbe_pci_tbl,
	.probe    = ixgbe_probe,
	.remove   = __devexit_p(ixgbe_remove),
#ifdef CONFIG_PM
	.suspend  = ixgbe_suspend,
	.resume   = ixgbe_resume,
#endif
#ifndef USE_REBOOT_NOTIFIER
	.shutdown = ixgbe_shutdown,
#endif
#ifdef HAVE_PCI_ERS
	.err_handler = &ixgbe_err_handler
#endif
};

bool ixgbe_is_ixgbe(struct pci_dev *pcidev)
{
	if (pci_dev_driver(pcidev) != &ixgbe_driver)
		return false;
	else
		return true;
}

/**
 * ixgbe_init_module - Driver Registration Routine
 *
 * ixgbe_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/
static int __init ixgbe_init_module(void)
{
	int ret;
	printk(KERN_INFO "ixgbe: %s - version %s\n", ixgbe_driver_string,
	       ixgbe_driver_version);

	printk(KERN_INFO "%s\n", ixgbe_copyright);

#ifndef CONFIG_DCB
	ixgbe_dcb_netlink_register();
#endif
#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
	dca_register_notify(&dca_notifier);

#endif
	ret = pci_register_driver(&ixgbe_driver);
	return ret;
}

module_init(ixgbe_init_module);

/**
 * ixgbe_exit_module - Driver Exit Cleanup Routine
 *
 * ixgbe_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit ixgbe_exit_module(void)
{
#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
	dca_unregister_notify(&dca_notifier);
#endif
#ifndef CONFIG_DCB
	ixgbe_dcb_netlink_unregister();
#endif
	pci_unregister_driver(&ixgbe_driver);
}

#if defined(CONFIG_DCA) || defined(CONFIG_DCA_MODULE)
static int ixgbe_notify_dca(struct notifier_block *nb, unsigned long event,
                            void *p)
{
	int ret_val;

	ret_val = driver_for_each_device(&ixgbe_driver.driver, NULL, &event,
	                                 __ixgbe_notify_dca);

	return ret_val ? NOTIFY_BAD : NOTIFY_DONE;
}
#endif
module_exit(ixgbe_exit_module);

/* ixgbe_main.c */

