/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 1999 - 2026 Intel Corporation */

#include "ixgbe.h"

static int ixgbe_flowspec_to_flow_type(struct ethtool_rx_flow_spec *fsp,
				       u8 *flow_type)
{
	switch (fsp->flow_type & ~FLOW_EXT) {
	case TCP_V4_FLOW:
		*flow_type = IXGBE_ATR_FLOW_TYPE_TCPV4;
		break;
	case UDP_V4_FLOW:
		*flow_type = IXGBE_ATR_FLOW_TYPE_UDPV4;
		break;
	case TCP_V6_FLOW:
		*flow_type = IXGBE_ATR_FLOW_TYPE_TCPV6;
		break;
	case UDP_V6_FLOW:
		*flow_type = IXGBE_ATR_FLOW_TYPE_UDPV6;
		break;
	case SCTP_V4_FLOW:
		*flow_type = IXGBE_ATR_FLOW_TYPE_SCTPV4;
		break;
	case IP_USER_FLOW:
		switch (fsp->h_u.usr_ip4_spec.proto) {
		case IPPROTO_TCP:
			*flow_type = IXGBE_ATR_FLOW_TYPE_TCPV4;
			break;
		case IPPROTO_UDP:
			*flow_type = IXGBE_ATR_FLOW_TYPE_UDPV4;
			break;
		case IPPROTO_SCTP:
			*flow_type = IXGBE_ATR_FLOW_TYPE_SCTPV4;
			break;
		case 0:
			if (!fsp->m_u.usr_ip4_spec.proto) {
				*flow_type = IXGBE_ATR_FLOW_TYPE_IPV4;
				break;
			}
			fallthrough;
		default:
			return 0;
		}
		break;
	default:
		return 0;
	}

	return 1;
}

static void ixgbe_set_flowspec_flow_type(struct ixgbe_adapter *adapter,
					 u8 flow_type,
					 struct ethtool_rx_flow_spec *fsp)
{
	switch (flow_type) {
	case IXGBE_ATR_FLOW_TYPE_TCPV4:
		fsp->flow_type = TCP_V4_FLOW;
		break;
	case IXGBE_ATR_FLOW_TYPE_UDPV4:
		fsp->flow_type = UDP_V4_FLOW;
		break;
	case IXGBE_ATR_FLOW_TYPE_TCPV6:
		fsp->flow_type = TCP_V6_FLOW;
		break;
	case IXGBE_ATR_FLOW_TYPE_UDPV6:
		fsp->flow_type = UDP_V6_FLOW;
		break;
	case IXGBE_ATR_FLOW_TYPE_SCTPV4:
		fsp->flow_type = SCTP_V4_FLOW;
		break;
	case IXGBE_ATR_FLOW_TYPE_IPV4:
		fsp->flow_type = IP_USER_FLOW;
		fsp->h_u.usr_ip4_spec.ip_ver = ETH_RX_NFC_IP4;
		fsp->h_u.usr_ip4_spec.proto = 0;
		fsp->m_u.usr_ip4_spec.proto = 0;
		break;
	default:
		fsp->flow_type = 0;
		e_warn(drv, "Unsupported flow type.\n");
	}
}

static void ixgbe_get_ethtool_fdir_entry(struct ixgbe_adapter *adapter,
					 struct ixgbe_fdir_filter *rule,
					 struct ethtool_rx_flow_spec *fsp)
{
	union ixgbe_atr_input *mask;

	ixgbe_set_flowspec_flow_type(adapter, rule->filter.formatted.flow_type,
				     fsp);

	mask = &adapter->fdir_mask;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	if (rule->filter.formatted.flow_type & IXGBE_ATR_L4TYPE_IPV6_MASK) {
		fsp->h_u.tcp_ip6_spec.psrc = rule->filter.formatted.src_port;
		fsp->m_u.tcp_ip6_spec.psrc = mask->formatted.src_port;
		fsp->h_u.tcp_ip6_spec.pdst = rule->filter.formatted.dst_port;
		fsp->m_u.tcp_ip6_spec.pdst = mask->formatted.dst_port;
	} else {
#endif
		fsp->h_u.tcp_ip4_spec.psrc = rule->filter.formatted.src_port;
		fsp->m_u.tcp_ip4_spec.psrc = mask->formatted.src_port;
		fsp->h_u.tcp_ip4_spec.pdst = rule->filter.formatted.dst_port;
		fsp->m_u.tcp_ip4_spec.pdst = mask->formatted.dst_port;
		fsp->h_u.tcp_ip4_spec.ip4src = rule->filter.formatted.src_ip[0];
		fsp->m_u.tcp_ip4_spec.ip4src = mask->formatted.src_ip[0];
		fsp->h_u.tcp_ip4_spec.ip4dst = rule->filter.formatted.dst_ip[0];
		fsp->m_u.tcp_ip4_spec.ip4dst = mask->formatted.dst_ip[0];
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	}
#endif

	fsp->h_ext.vlan_tci = rule->filter.formatted.vlan_id;
	fsp->m_ext.vlan_tci = mask->formatted.vlan_id;
	fsp->h_ext.vlan_etype = rule->filter.formatted.flex_bytes;
	fsp->m_ext.vlan_etype = mask->formatted.flex_bytes;
	fsp->h_ext.data[1] = htonl(rule->filter.formatted.vm_pool);
	fsp->m_ext.data[1] = htonl(mask->formatted.vm_pool);
	fsp->flow_type |= FLOW_EXT;

	/* record action */
	if (rule->action == IXGBE_FDIR_DROP_QUEUE)
		fsp->ring_cookie = RX_CLS_FLOW_DISC;
	else
		fsp->ring_cookie = rule->action;
}

int ixgbe_get_ethtool_filter_entry(struct ixgbe_adapter *adapter,
				   struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct ixgbe_rx_filter *filter = NULL;
	struct hlist_node *node2;
	int fdir_count, err = 0;

	/* report total rule count */
	fdir_count = (1024 << adapter->fdir_pballoc) - 2;
	cmd->data = fdir_count;

	spin_lock(&adapter->fdir_perfect_lock);

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		if (fsp->location <= filter->sw_idx)
			break;
	}

	if (!filter || fsp->location != filter->sw_idx) {
		err = -EINVAL;
		goto unlock;
	}

	switch (filter->type) {
	case IXGBE_RX_FILTER_FDIR:
		ixgbe_get_ethtool_fdir_entry(adapter, &filter->fdir, fsp);
		break;
	default:
		err = -EOPNOTSUPP;
	}

unlock:
	spin_unlock(&adapter->fdir_perfect_lock);
	return err;
}

int ixgbe_get_ethtool_filter_all(struct ixgbe_adapter *adapter,
				 struct ethtool_rxnfc *cmd, u32 *rule_locs)
{
	struct ixgbe_rx_filter *filter;
	struct hlist_node *node2;
	int fdir_count, cnt = 0;
	int err = 0;

	/* report total rule count */
	fdir_count = (1024 << adapter->fdir_pballoc) - 2;
	cmd->data = fdir_count;

	spin_lock(&adapter->fdir_perfect_lock);

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		if (cnt == cmd->rule_cnt) {
			err = -EMSGSIZE;
			goto unlock;
		}
		rule_locs[cnt] = filter->sw_idx;
		cnt++;
	}

	cmd->rule_cnt = cnt;

unlock:
	spin_unlock(&adapter->fdir_perfect_lock);
	return err;
}

int ixgbe_get_rss_hash_opts(struct ixgbe_adapter *adapter,
			    struct ethtool_rxnfc *cmd)
{
	cmd->data = 0;

	/* Report default options for RSS on ixgbe */
	switch (cmd->flow_type) {
	case TCP_V4_FLOW:
		cmd->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		fallthrough;
	case UDP_V4_FLOW:
		if (adapter->flags2 & IXGBE_FLAG2_RSS_FIELD_IPV4_UDP)
			cmd->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		fallthrough;
	case SCTP_V4_FLOW:
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case IPV4_FLOW:
		cmd->data |= RXH_IP_SRC | RXH_IP_DST;
		break;
	case TCP_V6_FLOW:
		cmd->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		fallthrough;
	case UDP_V6_FLOW:
		if (adapter->flags2 & IXGBE_FLAG2_RSS_FIELD_IPV6_UDP)
			cmd->data |= RXH_L4_B_0_1 | RXH_L4_B_2_3;
		fallthrough;
	case SCTP_V6_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case IPV6_FLOW:
		cmd->data |= RXH_IP_SRC | RXH_IP_DST;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int ixgbe_update_ethtool_filter_list(struct ixgbe_adapter *adapter,
				     struct ixgbe_rx_filter *input, u16 sw_idx)
{
	struct ixgbe_rx_filter *filter = NULL, *parent = NULL;
	struct ixgbe_hw *hw = &adapter->hw;
	bool present, deleted = false;
	struct hlist_node *node2;
	s32 err;

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		/* hash found, or no matching entry */
		if (filter->sw_idx >= sw_idx)
			break;

		parent = filter;
	}

	present = filter && filter->sw_idx == sw_idx;

	/* if there is an old filter occupying our place remove it */
	if (present && filter->type == IXGBE_RX_FILTER_FDIR) {
		struct ixgbe_fdir_filter *input_rule = NULL;
		struct ixgbe_fdir_filter *rule = NULL;

		if (input)
			input_rule = &input->fdir;

		rule = &filter->fdir;

		/* hardware filters are only configured when interface is up,
		 * and we should not issue filter commands while the interface
		 * is down
		 */
		if (netif_running(adapter->netdev) &&
		    (!input || (rule->filter.formatted.bkt_hash !=
				input_rule->filter.formatted.bkt_hash))) {
			err = ixgbe_fdir_erase_perfect_filter_82599(hw,
								&rule->filter,
								rule->fdir_idx);
			if (err)
				return -EINVAL;
		}

		hlist_del(&filter->filter_node);
		kfree(filter);
		adapter->filter_count--;
		deleted = true;
	}

	/* If we weren't given an input, then this was a request to delete a
	 * filter. We should return -EINVAL if the filter wasn't found, but
	 * return 0 if the rule was successfully deleted.
	 */
	if (!input)
		return deleted ? 0 : -EINVAL;

	/* initialize node and set software index */
	INIT_HLIST_NODE(&input->filter_node);

	/* add filter to the list */
	if (parent)
		hlist_add_behind(&input->filter_node, &parent->filter_node);
	else
		hlist_add_head(&input->filter_node,
			       &adapter->filter_list);

	/* update counts */
	adapter->filter_count++;

	return 0;
}

static bool ixgbe_match_ethtool_fdir_entry(struct ixgbe_adapter *adapter,
					   struct ixgbe_rx_filter *input)
{
	struct ixgbe_fdir_filter *rule;
	struct ixgbe_rx_filter *filter;
	struct hlist_node *node2;

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		if (filter->type != IXGBE_RX_FILTER_FDIR)
			continue;

		rule = &filter->fdir;

		if (rule->filter.formatted.bkt_hash ==
		    input->fdir.filter.formatted.bkt_hash &&
		    rule->action == input->fdir.action) {
			e_info(drv, "FDIR entry already exist\n");
			return true;
		}
	}
	return false;
}

u16 ixgbe_ethtool_find_free_fdir_idx(struct ixgbe_adapter *adapter)
{
	struct ixgbe_rx_filter *filter;
	struct hlist_node *node2;
	u16 max_fdir_filters;
	bool used;

	max_fdir_filters = (1024 << adapter->fdir_pballoc) - 2;

	for (int fdir_idx = 0; fdir_idx < max_fdir_filters; fdir_idx++) {
		used = false;

		hlist_for_each_entry_safe(filter, node2, &adapter->filter_list,
					  filter_node) {
			if (filter->type != IXGBE_RX_FILTER_FDIR)
				continue;

			if (filter->fdir.fdir_idx == fdir_idx) {
				used = true;
				break;
			}
		}

		if (!used)
			return fdir_idx;
	}

	return IXGBE_FDIR_IDX_ERR;
}

static int ixgbe_add_ethtool_fdir_entry(struct ixgbe_adapter *adapter,
					struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	struct ixgbe_fdir_filter *rule = NULL;
	struct ixgbe_hw *hw = &adapter->hw;
	struct ixgbe_rx_filter *filter;
	union ixgbe_atr_input mask;
	int err = -EINVAL;
	u16 fdir_idx;
	u8 queue;

	if (!(adapter->flags & IXGBE_FLAG_FDIR_PERFECT_CAPABLE))
		return -EOPNOTSUPP;

	/* ring_cookie is a masked into a set of queues and ixgbe pools or
	 * we use drop index
	 */
	if (fsp->ring_cookie == RX_CLS_FLOW_DISC) {
		queue = IXGBE_FDIR_DROP_QUEUE;
	} else {
		u32 ring = ethtool_get_flow_spec_ring(fsp->ring_cookie);
		u8 vf = ethtool_get_flow_spec_ring_vf(fsp->ring_cookie);

		if (!vf && ring >= adapter->num_rx_queues)
			return -EINVAL;
		else if (vf &&
			 ((vf > adapter->num_vfs) ||
			   ring >= adapter->num_rx_queues_per_pool))
			return -EINVAL;

		/* Map the ring onto the absolute queue index */
		if (vf)
			queue = ((vf - 1) *
				adapter->num_rx_queues_per_pool) + ring;
		else
			queue = adapter->rx_ring[ring]->reg_idx;
	}

	/* Don't allow indexes to exist outside of available space */
	if (fsp->location >= ((1024 << adapter->fdir_pballoc) - 2)) {
		e_err(drv, "Location out of range\n");
		return -EINVAL;
	}

	filter = kzalloc(sizeof(*filter), GFP_ATOMIC);
	if (!filter)
		return -ENOMEM;

	rule = &filter->fdir;

	memset(&mask, 0, sizeof(union ixgbe_atr_input));

	filter->type = IXGBE_RX_FILTER_FDIR;

	/* set indexes */
	filter->sw_idx = fsp->location;

	spin_lock(&adapter->fdir_perfect_lock);

	fdir_idx = ixgbe_ethtool_find_free_fdir_idx(adapter);
	if (fdir_idx == IXGBE_FDIR_IDX_ERR) {
		err = -ENOSPC;
		goto err_out_w_lock;
	}

	rule->fdir_idx = fdir_idx;
	/* record flow type */
	if (!ixgbe_flowspec_to_flow_type(fsp,
					 &rule->filter.formatted.flow_type)) {
		e_err(drv, "Unrecognized flow type\n");
		goto err_out_w_lock;
	}

	mask.formatted.flow_type = IXGBE_ATR_L4TYPE_IPV6_MASK |
				   IXGBE_ATR_L4TYPE_MASK;

	if (rule->filter.formatted.flow_type == IXGBE_ATR_FLOW_TYPE_IPV4)
		mask.formatted.flow_type &= IXGBE_ATR_L4TYPE_IPV6_MASK;

#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	/* not support full IPV6 address filtering */
	if (rule->filter.formatted.flow_type & IXGBE_ATR_L4TYPE_IPV6_MASK) {
		if (fsp->m_u.tcp_ip6_spec.ip6src[0] ||
		    fsp->m_u.tcp_ip6_spec.ip6src[1] ||
		    fsp->m_u.tcp_ip6_spec.ip6src[2] ||
		    fsp->m_u.tcp_ip6_spec.ip6src[3] ||
		    fsp->m_u.tcp_ip6_spec.ip6dst[0] ||
		    fsp->m_u.tcp_ip6_spec.ip6dst[1] ||
		    fsp->m_u.tcp_ip6_spec.ip6dst[2] ||
		    fsp->m_u.tcp_ip6_spec.ip6dst[3]) {
			e_err(drv, "Error not support IPv6 address filters\n");
			goto err_out_w_lock;
		}
		rule->filter.formatted.src_port = fsp->h_u.tcp_ip6_spec.psrc;
		mask.formatted.src_port = fsp->m_u.tcp_ip6_spec.psrc;
		rule->filter.formatted.dst_port = fsp->h_u.tcp_ip6_spec.pdst;
		mask.formatted.dst_port = fsp->m_u.tcp_ip6_spec.pdst;
	} else {
#endif
		/* Copy input into formatted structures */
		rule->filter.formatted.src_ip[0] =
			fsp->h_u.tcp_ip4_spec.ip4src;
		mask.formatted.src_ip[0] =
			fsp->m_u.tcp_ip4_spec.ip4src;
		rule->filter.formatted.dst_ip[0] =
			fsp->h_u.tcp_ip4_spec.ip4dst;
		mask.formatted.dst_ip[0] =
			fsp->m_u.tcp_ip4_spec.ip4dst;
		rule->filter.formatted.src_port =
			fsp->h_u.tcp_ip4_spec.psrc;
		mask.formatted.src_port =
			fsp->m_u.tcp_ip4_spec.psrc;
		rule->filter.formatted.dst_port =
			fsp->h_u.tcp_ip4_spec.pdst;
		mask.formatted.dst_port =
			fsp->m_u.tcp_ip4_spec.pdst;
#ifdef HAVE_ETHTOOL_FLOW_UNION_IP6_SPEC
	}
#endif

	if (fsp->flow_type & FLOW_EXT) {
		rule->filter.formatted.vm_pool =
				(unsigned char)ntohl(fsp->h_ext.data[1]);
		mask.formatted.vm_pool =
				(unsigned char)ntohl(fsp->m_ext.data[1]);
		rule->filter.formatted.vlan_id = fsp->h_ext.vlan_tci;
		mask.formatted.vlan_id = fsp->m_ext.vlan_tci;
		rule->filter.formatted.flex_bytes =
						fsp->h_ext.vlan_etype;
		mask.formatted.flex_bytes = fsp->m_ext.vlan_etype;
	}

	/* determine if we need to drop or route the packet */
	if (fsp->ring_cookie == RX_CLS_FLOW_DISC)
		rule->action = IXGBE_FDIR_DROP_QUEUE;
	else
		rule->action = fsp->ring_cookie;

	if (hlist_empty(&adapter->filter_list)) {
		/* save mask and program input mask into HW */
		memcpy(&adapter->fdir_mask, &mask, sizeof(mask));
		err = ixgbe_fdir_set_input_mask_82599(hw, &mask,
						      adapter->cloud_mode);
		if (err) {
			e_err(drv, "Error writing mask\n");
			goto err_out_w_lock;
		}
	} else if (memcmp(&adapter->fdir_mask, &mask, sizeof(mask))) {
		e_err(drv, "Hardware only supports one mask per port. To change the mask you must first delete all the rules.\n");
		goto err_out_w_lock;
	}

	/* apply mask and compute/store hash */
	ixgbe_atr_compute_perfect_hash_82599(&rule->filter, &mask);

	/* check if new entry does not exist on filter list */
	if (ixgbe_match_ethtool_fdir_entry(adapter, filter))
		goto err_out_w_lock;

	/* only program filters to hardware if the net device is running, as
	 * we store the filters in the Rx buffer which is not allocated when
	 * the device is down
	 */
	if (netif_running(adapter->netdev)) {
		err = ixgbe_fdir_write_perfect_filter_82599(hw,
					&rule->filter, rule->fdir_idx, queue,
					adapter->cloud_mode);
		if (err)
			goto err_out_w_lock;
	}

	ixgbe_update_ethtool_filter_list(adapter, filter, filter->sw_idx);

	spin_unlock(&adapter->fdir_perfect_lock);

	return 0;
err_out_w_lock:
	spin_unlock(&adapter->fdir_perfect_lock);
	kfree(filter);
	return err;
}

static enum ixgbe_rx_filter_type
ixgbe_ethtool_get_filter_type(struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;

	switch (fsp->flow_type & ~FLOW_EXT) {
	case TCP_V4_FLOW:
	case UDP_V4_FLOW:
	case SCTP_V4_FLOW:
	case TCP_V6_FLOW:
	case UDP_V6_FLOW:
	case IP_USER_FLOW:
		return IXGBE_RX_FILTER_FDIR;
	default:
		return IXGBE_RX_FILTER_UNKNOWN;
	}
}

int ixgbe_add_ethtool_filter_entry(struct ixgbe_adapter *adapter,
				   struct ethtool_rxnfc *cmd)
{
	enum ixgbe_rx_filter_type type;

	type = ixgbe_ethtool_get_filter_type(cmd);

	switch (type) {
	case IXGBE_RX_FILTER_FDIR:
		return ixgbe_add_ethtool_fdir_entry(adapter, cmd);
	default:
		return -EOPNOTSUPP;
	}
}

int ixgbe_del_ethtool_filter_entry(struct ixgbe_adapter *adapter,
				   struct ethtool_rxnfc *cmd)
{
	struct ethtool_rx_flow_spec *fsp =
		(struct ethtool_rx_flow_spec *)&cmd->fs;
	int err;

	spin_lock(&adapter->fdir_perfect_lock);
	err = ixgbe_update_ethtool_filter_list(adapter, NULL, fsp->location);
	spin_unlock(&adapter->fdir_perfect_lock);

	return err;
}

#define UDP_RSS_FLAGS (IXGBE_FLAG2_RSS_FIELD_IPV4_UDP | \
		       IXGBE_FLAG2_RSS_FIELD_IPV6_UDP)

int ixgbe_set_rss_hash_opt(struct ixgbe_adapter *adapter,
			   struct ethtool_rxnfc *nfc)
{
	u32 flags2 = adapter->flags2;

	/*
	 * RSS does not support anything other than hashing
	 * to queues on src and dst IPs and ports
	 */
	if (nfc->data & ~(RXH_IP_SRC | RXH_IP_DST |
			  RXH_L4_B_0_1 | RXH_L4_B_2_3))
		return -EINVAL;

	switch (nfc->flow_type) {
	case TCP_V4_FLOW:
	case TCP_V6_FLOW:
		if (!(nfc->data & RXH_IP_SRC) ||
		    !(nfc->data & RXH_IP_DST) ||
		    !(nfc->data & RXH_L4_B_0_1) ||
		    !(nfc->data & RXH_L4_B_2_3))
			return -EINVAL;
		break;
	case UDP_V4_FLOW:
		if (!(nfc->data & RXH_IP_SRC) ||
		    !(nfc->data & RXH_IP_DST))
			return -EINVAL;
		switch (nfc->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3)) {
		case 0:
			flags2 &= ~IXGBE_FLAG2_RSS_FIELD_IPV4_UDP;
			break;
		case (RXH_L4_B_0_1 | RXH_L4_B_2_3):
			flags2 |= IXGBE_FLAG2_RSS_FIELD_IPV4_UDP;
			break;
		default:
			return -EINVAL;
		}
		break;
	case UDP_V6_FLOW:
		if (!(nfc->data & RXH_IP_SRC) ||
		    !(nfc->data & RXH_IP_DST))
			return -EINVAL;
		switch (nfc->data & (RXH_L4_B_0_1 | RXH_L4_B_2_3)) {
		case 0:
			flags2 &= ~IXGBE_FLAG2_RSS_FIELD_IPV6_UDP;
			break;
		case (RXH_L4_B_0_1 | RXH_L4_B_2_3):
			flags2 |= IXGBE_FLAG2_RSS_FIELD_IPV6_UDP;
			break;
		default:
			return -EINVAL;
		}
		break;
	case AH_ESP_V4_FLOW:
	case AH_V4_FLOW:
	case ESP_V4_FLOW:
	case SCTP_V4_FLOW:
	case AH_ESP_V6_FLOW:
	case AH_V6_FLOW:
	case ESP_V6_FLOW:
	case SCTP_V6_FLOW:
		if (!(nfc->data & RXH_IP_SRC) ||
		    !(nfc->data & RXH_IP_DST) ||
		    (nfc->data & RXH_L4_B_0_1) ||
		    (nfc->data & RXH_L4_B_2_3))
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	/* if we changed something we need to update flags */
	if (flags2 != adapter->flags2) {
		struct ixgbe_hw *hw = &adapter->hw;
		u32 mrqc;
		unsigned int pf_pool = adapter->num_vfs;

		if (hw->mac.type >= ixgbe_mac_X550 &&
		    (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED))
			mrqc = IXGBE_READ_REG(hw, IXGBE_PFVFMRQC(pf_pool));
		else
			mrqc = IXGBE_READ_REG(hw, IXGBE_MRQC);

		if ((flags2 & UDP_RSS_FLAGS) &&
		    !(adapter->flags2 & UDP_RSS_FLAGS))
			e_warn(drv, "enabling UDP RSS: fragmented packets may arrive out of order to the stack above\n");

		adapter->flags2 = flags2;

		/* Perform hash on these packet types */
		mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4
		      | IXGBE_MRQC_RSS_FIELD_IPV4_TCP
		      | IXGBE_MRQC_RSS_FIELD_IPV6
		      | IXGBE_MRQC_RSS_FIELD_IPV6_TCP;

		mrqc &= ~(IXGBE_MRQC_RSS_FIELD_IPV4_UDP |
			  IXGBE_MRQC_RSS_FIELD_IPV6_UDP);

		if (flags2 & IXGBE_FLAG2_RSS_FIELD_IPV4_UDP)
			mrqc |= IXGBE_MRQC_RSS_FIELD_IPV4_UDP;

		if (flags2 & IXGBE_FLAG2_RSS_FIELD_IPV6_UDP)
			mrqc |= IXGBE_MRQC_RSS_FIELD_IPV6_UDP;

		if (hw->mac.type >= ixgbe_mac_X550 &&
		    (adapter->flags & IXGBE_FLAG_SRIOV_ENABLED))
			IXGBE_WRITE_REG(hw, IXGBE_PFVFMRQC(pf_pool), mrqc);
		else
			IXGBE_WRITE_REG(hw, IXGBE_MRQC, mrqc);
	}

	return 0;
}

void ixgbe_fdir_filter_restore(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	struct ixgbe_fdir_filter *rule;
	struct ixgbe_rx_filter *filter;
	struct hlist_node *node2;
	u8 queue = 0;

	spin_lock(&adapter->fdir_perfect_lock);

	if (!hlist_empty(&adapter->filter_list))
		ixgbe_fdir_set_input_mask_82599(hw, &adapter->fdir_mask,
						adapter->cloud_mode);

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		if (filter->type != IXGBE_RX_FILTER_FDIR)
			continue;

		rule = &filter->fdir;

		if (rule->action == IXGBE_FDIR_DROP_QUEUE) {
			queue = IXGBE_FDIR_DROP_QUEUE;
		} else {
			u32 ring = ethtool_get_flow_spec_ring(rule->action);
			u8 vf = ethtool_get_flow_spec_ring_vf(rule->action);

			if (!vf && ring >= adapter->num_rx_queues) {
				e_err(drv,
				      "FDIR restore failed w/o vf, ring:%u\n",
				      ring);
				continue;
			} else if (vf &&
				   ((vf > adapter->num_vfs) ||
				   ring >= adapter->num_rx_queues_per_pool)) {
				e_err(drv,
				      "FDIR restore failed vf:%u, ring:%u\n",
				      vf, ring);
				continue;
			}

			/* Map the ring onto the absolute queue index */
			if (vf)
				queue = ((vf - 1) *
					adapter->num_rx_queues_per_pool) + ring;
			else
				queue = adapter->rx_ring[ring]->reg_idx;
		}

		ixgbe_fdir_write_perfect_filter_82599(hw,
						      &rule->filter,
						      rule->fdir_idx,
						      queue,
						      adapter->cloud_mode);
	}

	spin_unlock(&adapter->fdir_perfect_lock);
}

void ixgbe_fdir_filter_exit(struct ixgbe_adapter *adapter)
{
	struct ixgbe_rx_filter *filter;
	struct hlist_node *node2;

	spin_lock(&adapter->fdir_perfect_lock);

	hlist_for_each_entry_safe(filter, node2,
				  &adapter->filter_list, filter_node) {
		if (filter->type != IXGBE_RX_FILTER_FDIR)
			continue;

		hlist_del(&filter->filter_node);
		kfree(filter);
	}
	adapter->filter_count = 0;

	spin_unlock(&adapter->fdir_perfect_lock);
}
