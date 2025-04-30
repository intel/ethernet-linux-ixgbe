 /* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (C) 1999 - 2025 Intel Corporation */

#include <linux/ptp_classify.h>
#include <linux/bitfield.h>
#include "ixgbe.h"
#include "ixgbe_ptp_e600.h"

#define IXGBE_PHY_TS_VAL(_ts)	((u64)(_ts) << 1)
#define IXGBE_PHY_TS_MASK(_ts)	((_ts) & GENMASK(32, 0))
#define IXGBE_E600_BASE_PERIOD	0x333333333ULL
#define ADJ_VAL_M		GENMASK(30, 0)
#define ADJ_SIGN_M		BIT(31)
#define TSAUXC_DIS_TS_CLEAR	0x40000000

static const struct ptp_pin_desc e600_pin_cfg[] = {
	{ "SDP0", 0, 0, 0 }
};

/**
 * ixgbe_ptp_update_cached_phc_time_e600 - update cached PHC time
 * @info: Driver's PTP info structure
 *
 * Update cached PHC time needed for extending Tx/Rx timestamps to 64 bit.
 */
static void ixgbe_ptp_update_cached_phc_time_e600(struct ptp_clock_info *info)
{
	struct ixgbe_adapter *adapter = container_of(info, struct ixgbe_adapter,
						     ptp_caps);
	struct ixgbe_hw *hw = &adapter->hw;
	struct timespec64 ts;

	IXGBE_READ_REG(hw, IXGBE_SYSTIMR);
	ts.tv_nsec = IXGBE_READ_REG(hw, IXGBE_SYSTIML);
	ts.tv_sec = IXGBE_READ_REG(hw, IXGBE_SYSTIMH);

	adapter->ptp.cached_phc_time = timespec64_to_ns(&ts);
}

/**
 * ixgbe_ptp_adjfine_e600 - adjust PHC increment rate
 * @ptp: the ptp clock structure
 * @scaled_ppm: scaled parts per million adjustment from base
 *
 * Adjust the frequency of the PTP hardware clock by the indicated scaled_ppm
 * from the base frequency.
 *
 * Scaled parts per million is ppm with a 16-bit binary fractional field.
 *
 * For adapters supporting PTP by PHY, reinitialize PHY ToD when the ppb is
 * greater than maximal drift threshold.
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_adjfine_e600(struct ptp_clock_info *ptp, long scaled_ppm)
{
	struct ixgbe_adapter *adapter = container_of(ptp, struct ixgbe_adapter,
						     ptp_caps);
	long ppb = ((1 + scaled_ppm) * 125) >> 13;
	struct ixgbe_hw *hw = &adapter->hw;
	bool neg_adj;
	u64 rate;
	u32 inca;

	neg_adj = diff_by_scaled_ppm(IXGBE_E600_BASE_PERIOD, scaled_ppm, &rate);

	/* Warn if rate is too large. */
	if (rate >= ADJ_VAL_M)
		e_dev_warn("PTP ppb adjusted SYSTIME rate overflowed!\n");

	inca = rate & ADJ_VAL_M;
	if (neg_adj)
		inca |= ADJ_SIGN_M;

	IXGBE_WRITE_REG(hw, IXGBE_TIMINCA, inca);

	if (adapter->ptp.ptp_by_phy_ena &&
	    (ppb > adapter->ptp.max_drift_threshold ||
	     ppb < -adapter->ptp.max_drift_threshold)) {
		u8 request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_TOD_INIT;
		s32 status;

		status = ixgbe_set_ptp_by_phy(hw, request, 0);
		if (status)
			e_dev_err("PTP by PHY ToD reinit failed, status=%d\n",
				  status);

		ixgbe_ptp_update_cached_phc_time_e600(ptp);
	}

	return 0;
}

#ifndef HAVE_PTP_CLOCK_INFO_ADJFINE
/**
 * ixgbe_ptp_adjfreq_e600 - Adjust the frequency of the clock
 * @info: the driver's PTP info structure
 * @ppb: Parts per billion adjustment from the base
 *
 * Adjust the frequency of the clock by the indicated parts per billion from the
 * base frequency.
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_adjfreq_e600(struct ptp_clock_info *info, s32 ppb)
{
	long scaled_ppm;

	/*
	 * We want to calculate
	 *
	 *    scaled_ppm = ppb * 2^16 / 1000
	 *
	 * which simplifies to
	 *
	 *    scaled_ppm = ppb * 2^13 / 125
	 */
	scaled_ppm = ((long)ppb << 13) / 125;
	return ixgbe_ptp._adjfine_e600(info, scaled_ppm);
}

#endif
/**
 * ixgbe_ptp_gettimex_e600 - get PHC time and optional system timestamp
 * @ptp: the ptp clock structure
 * @ts: timespec to hold the PHC timestamp
 * @sts: structure to hold the system time before and after reading the PHC
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_gettimex_e600(struct ptp_clock_info *ptp,
				   struct timespec64 *ts,
				   struct ptp_system_timestamp *sts)
{
	struct ixgbe_adapter *adapter = container_of(ptp, struct ixgbe_adapter,
						     ptp_caps);
	struct ixgbe_hw *hw = &adapter->hw;

	ptp_read_system_prets(sts);
	IXGBE_READ_REG(hw, IXGBE_SYSTIMR);
	ptp_read_system_postts(sts);
	ts->tv_nsec = IXGBE_READ_REG(hw, IXGBE_SYSTIML);
	ts->tv_sec = IXGBE_READ_REG(hw, IXGBE_SYSTIMH);

	return 0;
}

/**
 * ixgbe_ptp_gettime_e600 - get PHC time and optional system timestamp
 * @ptp: the ptp clock structure
 * @ts: timespec64 structure to hold the current time value
 *
 * Return: 0 on success.
 */
static int __maybe_unused ixgbe_ptp_gettime_e600(struct ptp_clock_info *ptp,
						 struct timespec64 *ts)
{
	return ixgbe_ptp_gettimex_e600(ptp, ts, NULL);
}

/**
 * ixgbe_ptp_setup_sdp_e600
 * @adapter: private adapter structure
 * @rq: periodic output request
 * @on: true to enable SDP output, false otherwise
 *
 * Enable or disable a clock output signal on SDP0 for E600 hardware.
 *
 * Use the target time feature to align the output signal on the next full
 * second.
 *
 * Return: 0 on success, negative error code otherwise
 */
static int ixgbe_ptp_setup_sdp_e600(struct ixgbe_adapter *adapter,
				    struct ptp_perout_request *rq, bool on)
{
	struct ixgbe_hw *hw = &adapter->hw;
	u32 period = NSEC_PER_SEC;
	u32 esdp, tsauxc, tssdp;
	u64 clk, start, phase;
	struct timespec64 ts;
	int err;

#ifdef PTP_PEROUT_PHASE
	if (rq->flags & ~PTP_PEROUT_PHASE)
		return -EOPNOTSUPP;
#else /* !PTP_PEROUT_PHASE */
	if (rq->flags)
		return -EOPNOTSUPP;
#endif /* PTP_PEROUT_PHASE */

	/* Disable the pin first. */
	IXGBE_WRITE_REG(hw, IXGBE_TSAUXC, 0x0);
	IXGBE_WRITE_FLUSH(hw);

	period = rq->period.sec * NSEC_PER_SEC + rq->period.nsec;
	if (!on || !period)
		return 0;

	if (period != NSEC_PER_SEC) {
		e_dev_err("CLK Period != 1s is not supported\n");
		return -EINVAL;
	}

	start = rq->start.sec * NSEC_PER_SEC + rq->start.nsec;

#ifdef PTP_PEROUT_PHASE
	/* If PTP_PEROUT_PHASE is set, rq has phase instead of start time. */
	if (rq->flags & PTP_PEROUT_PHASE)
		phase = start;
	else
#endif /* PTP_PEROUT_PHASE */
		div64_u64_rem(start, period, &phase);

	if (phase % NSEC_PER_SEC != 0) {
		e_dev_err("Start/phase not on a second boundary is not supported\n");
		return -EINVAL;
	}

	/* If we have only phase or start time is in the past, start the timer
	 * at the next multiple of period, maintaining phase at least 0.1 second
	 * from now, so we have time to write it to HW.
	 */
	err = ixgbe_ptp_gettimex_e600(&adapter->ptp_caps, &ts, NULL);
	if (err)
		return err;
	clk = timespec64_to_ns(&ts) + NSEC_PER_MSEC * 100;
#ifdef PTP_PEROUT_PHASE
	if (rq->flags & PTP_PEROUT_PHASE || start <= clk)
#else /* !PTP_PEROUT_PHASE */
	if (start <= clk)
#endif /* PTP_PEROUT_PHASE */
		start = roundup_u64(clk, period) + phase;

	/* Write half of the period (50% duty cycle). */
	IXGBE_WRITE_REG(hw, IXGBE_FREQOUT0, NSEC_PER_SEC / 2);
	IXGBE_WRITE_REG(hw, IXGBE_TRGTTIML0, 0);
	IXGBE_WRITE_REG(hw, IXGBE_TRGTTIMH0, start / NSEC_PER_SEC);

	/* Enable the SDP0 pin as output. */
	esdp = IXGBE_READ_REG(hw, IXGBE_ESDP);
	esdp |= IXGBE_ESDP_SDP0_DIR | IXGBE_ESDP_SDP0_NATIVE;
	IXGBE_WRITE_REG(hw, IXGBE_ESDP, esdp);

	tssdp = IXGBE_TSSDP_TS_SDP0_EN | IXGBE_TSSDP_TS_SDP0_CLK0;
	IXGBE_WRITE_REG(hw, IXGBE_TSSDP, tssdp);

	/* Enable the Clock Out feature on SDP0, and use Target Time 0 to
	 * enable generation of interrupts on the clock change.
	 */
	tsauxc = IXGBE_TSAUXC_EN_CLK | IXGBE_TSAUXC_ST0 | IXGBE_TSAUXC_EN_TT0 |
		 IXGBE_TSAUXC_SDP0_INT | TSAUXC_DIS_TS_CLEAR;
	IXGBE_WRITE_REG(hw, IXGBE_TSAUXC, tsauxc);

	IXGBE_WRITE_FLUSH(hw);

	return 0;
}

/**
 * ixgbe_ptp_verify_pin_e600 - verify if pin supports requested pin function
 * @info: the driver's PTP info structure
 * @pin: pin index
 * @func: assigned function
 * @chan: assigned channel
 *
 * Return: 0 on success, -EOPNOTSUPP when function is not supported.
 */
static int ixgbe_ptp_verify_pin_e600(struct ptp_clock_info *info,
				     unsigned int pin,
				     enum ptp_pin_function func,
				     unsigned int chan)
{
	/* Is assigned function allowed? */
	switch (func) {
	case PTP_PF_PEROUT:
	case PTP_PF_NONE:
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

/**
 * ixgbe_ptp_gpio_enable_e600 - Enable/disable ancillary features of PHC
 * @info: the driver's PTP info structure
 * @rq: the requested feature to change
 * @on: enable/disable flag
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int ixgbe_ptp_gpio_enable_e600(struct ptp_clock_info *info,
				      struct ptp_clock_request *rq, int on)
{
	struct ixgbe_adapter *adapter = container_of(info, struct ixgbe_adapter,
						     ptp_caps);

	switch (rq->type) {
	case PTP_CLK_REQ_PEROUT:
	{
		int err = ixgbe_ptp_setup_sdp_e600(adapter, &rq->perout, on);

		if (!err && on)
			adapter->flags2 |= IXGBE_FLAG2_PTP_PPS_ENABLED;
		else
			adapter->flags2 &= ~IXGBE_FLAG2_PTP_PPS_ENABLED;

		return err;
	}
	default:
		return -EOPNOTSUPP;
	}
}

/**
 * ixgbe_ptp_settime_e600 - set PHC time
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_settime_e600(struct ptp_clock_info *ptp,
				  const struct timespec64 *ts)
{
	struct ixgbe_adapter *adapter = container_of(ptp, struct ixgbe_adapter,
						     ptp_caps);
	struct ptp_perout_request rq = {
		.start = {},
		.period = { .sec = NSEC_PER_SEC, .nsec = 0 }
	};
	struct ixgbe_hw *hw = &adapter->hw;

	/* Disable SDP and re-enable after time change. */
	ixgbe_ptp_setup_sdp_e600(adapter, &rq, false);

	IXGBE_WRITE_REG(hw, IXGBE_SYSTIML, ts->tv_nsec);
	IXGBE_WRITE_REG(hw, IXGBE_SYSTIMH, timespec64_to_ns(ts) / NSEC_PER_SEC);
	IXGBE_WRITE_FLUSH(hw);

	ixgbe_ptp_setup_sdp_e600(adapter, &rq, true);

	if (adapter->ptp.ptp_by_phy_ena) {
		u8 request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_TOD_INIT;
		s32 status;

		status = ixgbe_set_ptp_by_phy(hw, request, 0);
		if (status)
			e_dev_err("PTP by PHY ToD init failed, status=%d\n",
				  status);

		ixgbe_ptp_update_cached_phc_time_e600(ptp);
	}

	return 0;
}

#ifndef HAVE_PTP_CLOCK_INFO_GETTIME64
/**
 * ixgbe_ptp_gettime32_e600 - get PHC time
 * @ptp: the ptp clock structure
 * @ts: timespec64 structure to hold the current time value
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_gettime32_e600(struct ptp_clock_info *ptp,
				    struct timespec *ts)
{
	struct timespec64 ts64;
	int err;

	err = ixgbe_ptp_gettime_e600(ptp, &ts64);
	if (err)
		return err;

	*ts = timespec64_to_timespec(ts64);

	return 0;
}

/**
 * ixgbe_ptp_settime_e600 - set PHC time
 * @ptp: the ptp clock structure
 * @ts: the timespec containing the new time
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_settime32_e600(struct ptp_clock_info *ptp,
				    const struct timespec *ts)
{
	struct timespec64 ts64 = timespec_to_timespec64(*ts);

	return ixgbe_ptp_settime_e600(ptp, &ts64);
}
#endif
/**
 * ixgbe_ptp_adjtime_nonatomic_e600 - do a non-atomic clock adjustment
 * @ptp: the driver's PTP info structure
 * @delta: Offset in nanoseconds to adjust the time by
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_adjtime_nonatomic_e600(struct ptp_clock_info *ptp,
					    s64 delta)
{
	struct timespec64 now;
	int err;

	err = ixgbe_ptp_gettimex_e600(ptp, &now, NULL);
	if (err)
		return err;
	now = timespec64_add(now, ns_to_timespec64(delta));

	return ixgbe_ptp_settime_e600(ptp, (const struct timespec64 *)&now);
}

/**
 * ixgbe_ptp_adjtime_e600 - adjust time by a specified delta
 * @ptp: the ptp clock structure
 * @delta: offset to adjust the cycle counter by
 *
 * Return: 0 on success.
 */
static int ixgbe_ptp_adjtime_e600(struct ptp_clock_info *ptp, s64 delta)
{
	struct ixgbe_adapter *adapter = container_of(ptp, struct ixgbe_adapter,
						     ptp_caps);
	u32 val;

	/* Hardware only supports atomic adjustments using unsigned
	 * 31-bit integers with a sign bit. For any adjustment outside
	 * this range, perform a non-atomic get->adjust->set flow.
	 */
	if (delta > S32_MAX / 2 || delta < S32_MIN / 2 ||
	    adapter->ptp.ptp_by_phy_ena)
		return ixgbe_ptp_adjtime_nonatomic_e600(ptp, delta);

	if (delta < 0) {
		val = -delta;
		val |= ADJ_SIGN_M;
	} else {
		val = delta;
	}

	IXGBE_WRITE_REG(&adapter->hw, IXGBE_TIMADJL, val);

	return 0;
}

/**
 * ixgbe_ptp_cfg_phy_timestamping_e600 - configure PTP timestamping on the PHY
 * @adapter: pointer to the adapter structure
 */
static void ixgbe_ptp_cfg_phy_timestamping_e600(struct ixgbe_adapter *adapter)
{
	struct ixgbe_ptp_e600 *ptp = &adapter->ptp;
	struct ixgbe_hw *hw = &adapter->hw;
	u8 ptp_request, flags;
	bool enable;
	s32 status;

	/* Enable only for <= 1 GB or if one-step timestamping is enabled. */
	switch (adapter->link_speed) {
	case IXGBE_LINK_SPEED_10_FULL:
	case IXGBE_LINK_SPEED_100_FULL:
	case IXGBE_LINK_SPEED_1GB_FULL:
		enable = true;
		break;
	case IXGBE_LINK_SPEED_2_5GB_FULL:
	case IXGBE_LINK_SPEED_5GB_FULL:
	case IXGBE_LINK_SPEED_10GB_FULL:
		enable = ptp->onestep_ena;
		break;
	default:
		return;
	}

	status = ixgbe_get_ptp_by_phy(hw, &ptp_request, &flags,
				      &ptp->max_drift_threshold);
	if (status) {
		e_dev_err("PTP by PHY get failed, status=%d\n", status);
		return;
	}

	ptp->ptp_by_phy_ena = ptp_request;

	/* Don't reconfigure if already disabled. */
	if (!enable && enable == ptp_request)
		goto skip_set;

	if (adapter->ptp.vlan_ena)
		flags = FIELD_PREP(IXGBE_SET_PTP_BY_PHY_ETHERTYPE_M,
				   IXGBE_SET_PTP_BY_PHY_ETHERTYPE_VLAN_TAG);
	else
		flags = FIELD_PREP(IXGBE_SET_PTP_BY_PHY_ETHERTYPE_M,
				   IXGBE_SET_PTP_BY_PHY_ETHERTYPE_NO_VLAN_TAG);

	if (adapter->ptp.onestep_ena)
		flags |= FIELD_PREP(IXGBE_SET_PTP_BY_PHY_TX_TS_M,
				    IXGBE_SET_PTP_BY_PHY_TX_TS_1STEP);

	status = ixgbe_set_ptp_by_phy(hw, enable, flags);
	if (status) {
		e_dev_err("PTP by PHY set failed, status=%d\n", status);
		return;
	}

skip_set:
	if (!enable) {
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
		ptp_cancel_worker_sync(adapter->ptp_clock);
#else /* !HAVE_PTP_CANCEL_WORKER_SYNC */
		kthread_cancel_delayed_work_sync(&ptp->ptp_aux_work);
#endif /* HAVE_PTP_CANCEL_WORKER_SYNC */
		ptp->ptp_by_phy_ena = false;
		return;
	}

	/* Clear timestamping status register. */
	IXGBE_WRITE_REG(hw, PROXY_STS, 0);

	ptp_request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_SET_PHY_PARAMS;
	status = ixgbe_set_ptp_by_phy(hw, ptp_request, flags);
	if (status)
		e_dev_err("PTP by PHY set PHY params failed, status=%d\n",
			  status);

	ptp_request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_TOD_INIT;
	status = ixgbe_set_ptp_by_phy(hw, ptp_request, 0);
	if (status) {
		e_dev_err("PTP by PHY ToD init failed, status=%d\n", status);
	} else {
		ptp->ptp_by_phy_ena = true;
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
		ptp_schedule_worker(adapter->ptp_clock, 0);
#else /* HAVE_PTP_CANCEL_WORKER_SYNC */
		kthread_mod_delayed_work(ptp->ptp_kworker, &ptp->ptp_aux_work,
					 0);
#endif /* !HAVE_PTP_CANCEL_WORKER_SYNC */
	}
}

/**
 * ixgbe_ptp_clear_tx_timestamp_e600 - clear Tx timestamp state
 * @adapter: the private adapter structure
 *
 * This function should be called whenever the state related to a Tx timestamp
 * needs to be cleared. This helps ensure that all related bits are reset for
 * the next Tx timestamp event.
 */
static void ixgbe_ptp_clear_tx_timestamp_e600(struct ixgbe_adapter *adapter)
{
	IXGBE_READ_REG(&adapter->hw, IXGBE_TXSTMPH);
	if (adapter->ptp_tx_skb) {
		dev_kfree_skb_any(adapter->ptp_tx_skb);
		adapter->ptp_tx_skb = NULL;
	}
	clear_bit_unlock(__IXGBE_PTP_TX_IN_PROGRESS, adapter->state);
}

/**
 * ixgbe_ptp_extend_32b_ts_e600 - Convert a 32b nanoseconds timestamp to 64b
 * @adapter: the private adapter struct
 * @in_tstamp: Ingress/egress 32b nanoseconds timestamp value
 *
 * Hardware captures timestamps which contain only 32 bits of nominal
 * nanoseconds shifted one bit right, as opposed to the 64bit timestamps, that
 * the stack expects.
 *
 * Extend the 32bit nanosecond timestamp using the following algorithm and
 * assumptions:
 *
 * 1) have a recently cached copy of the PHC time
 * 2) assume that the in_tstamp was captured 2^32 nanoseconds (~4.3 seconds)
 *    before or after the PHC time was captured.
 * 3) shift the timestamp one bit left
 * 4) calculate the delta between the cached time and the timestamp
 * 5) if the delta is smaller than 2^32 nanoseconds, then the timestamp was
 *    captured after the PHC time. In this case, the full timestamp is just
 *    the cached PHC time plus the delta.
 * 6) otherwise, if the delta is larger than 2^32 nanoseconds, then the
 *    timestamp was captured *before* the PHC time, i.e. because the PHC
 *    cache was updated after the timestamp was captured by hardware. In this
 *    case, the full timestamp is the cached time minus the inverse delta.
 *
 * This algorithm works even if the PHC time was updated after a Tx timestamp
 * was requested, but before the Tx timestamp event was reported from
 * hardware.
 *
 * This calculation primarily relies on keeping the cached PHC time up to
 * date. If the timestamp was captured more than 2^32 nanoseconds after the
 * PHC time, it is possible that the lower 33bits of PHC time have
 * overflowed more than once, and we might generate an incorrect timestamp.
 *
 * This is prevented by periodically updating the cached PHC time every 1 s.
 *
 * Return: 64 bit extended timestamp value.
 */
static u64 ixgbe_ptp_extend_32b_ts_e600(struct ixgbe_adapter *adapter,
					u32 in_tstamp)
{
	u64 delta, phc_time_lo;

	/* Extract the lower 33 bits of the PHC time. */
	phc_time_lo = IXGBE_PHY_TS_MASK(adapter->ptp.cached_phc_time);

	/* Calculate the delta between the lower 33bits of the cached PHC time
	 * and the in_tstamp value.
	 */
	delta = IXGBE_PHY_TS_MASK(IXGBE_PHY_TS_VAL(in_tstamp) - phc_time_lo);

	/* Do not assume that the in_tstamp is always more recent than the
	 * cached PHC time. If the delta is large, it indicates that the
	 * in_tstamp was taken in the past, and should be converted
	 * forward.
	 */
	if (delta > U32_MAX) {
		/* Reverse the delta calculation here. */
		delta = IXGBE_PHY_TS_MASK(phc_time_lo -
					  IXGBE_PHY_TS_VAL(in_tstamp));
		return adapter->ptp.cached_phc_time - delta;
	}

	return adapter->ptp.cached_phc_time + delta;
}

/**
 * ixgbe_ptp_is_tx_ptp - check if packet is a PTP packet
 * @adapter: the private adapter struct
 * @skb: the packet
 *
 * Return: true if packet is a PTP packet, false otherwise.
 */
bool ixgbe_ptp_is_tx_ptp(struct ixgbe_adapter *adapter, struct sk_buff *skb)
{
	unsigned int ptp_class;
	struct ptp_header *hdr;

	if (likely(!adapter->ptp.ptp_by_phy_ena))
		return false;

	ptp_class = ptp_classify_raw(skb);
	if (likely(ptp_class == PTP_CLASS_NONE))
		return false;

	skb_reset_mac_header(skb);
	hdr = ptp_parse_header(skb, ptp_class);

	/* Clear Tx TS bit if it's a PTP packet with invalid header. */
	if (!hdr)
		clear_bit_unlock(__IXGBE_PTP_TX_IN_PROGRESS, adapter->state);
	else
		adapter->ptp.ptp_seq_id = be16_to_cpu(hdr->sequence_id);

	return true;
}

/**
 * ixgbe_ptp_tx_phytstamp_e600 - read the PHY Tx timestamp from the register
 * @adapter: the private adapter struct
 * @sts: value of the status register
 *
 * Return:
 * * %0       - success
 * * %-ENXIO  - Tx TS not in progress
 * * %-EINVAL - invalid sequence ID
 * * %other   - FW error code
 */
static int ixgbe_ptp_tx_phytstamp_e600(struct ixgbe_adapter *adapter, u32 sts)
{
	struct sk_buff *skb = adapter->ptp_tx_skb;
	struct skb_shared_hwtstamps shhwtstamps;
	struct ixgbe_hw *hw = &adapter->hw;
	u64 tstamp;
	int err;

	/* Return error if PTP Tx TS not in progress. */
	if (!test_bit(__IXGBE_PTP_TX_IN_PROGRESS, adapter->state)) {
		err = -ENXIO;
		goto err;
	}

	err = FIELD_GET(PROXY_STS_ERR, sts);
	if (err)
		goto err;

	/* Incorrect sequence ID, discard the timestamp. */
	if (FIELD_GET(PROXY_STS_SEQ_ID, sts) != adapter->ptp.ptp_seq_id) {
		err = -EINVAL;
		goto err;
	}

	/* The timestamp is recorded in the PHY memory as a 32 bit value.
	 * The driver needs to extend it to full 64 bit time.
	 */
	tstamp = ixgbe_ptp_extend_32b_ts_e600(adapter,
					      IXGBE_READ_REG(hw, PROXY_TX_TS));

	/* Notify the stack and then free the skb after we've unlocked. */
	shhwtstamps.hwtstamp = ns_to_ktime(tstamp);
	skb_tstamp_tx(skb, &shhwtstamps);
err:
	/* Clear status register to indicate SW read the timestamp. */
	IXGBE_WRITE_REG(hw, PROXY_STS, 0);
	adapter->ptp_tx_skb = NULL;
	clear_bit_unlock(__IXGBE_PTP_TX_IN_PROGRESS, adapter->state);
	dev_kfree_skb_any(skb);
	if (err)
		adapter->tx_hwtstamp_skipped++;

	return err;
}

/**
 * ixgbe_ptp_fw_intr_e600 - handle PTP by PHY FW interrupt
 * @adapter: the private adapter struct
 *
 * Return:
 * * %0       - success
 * * %-EINTR  - not a Tx TS interrupt cause
 * * %-ENXIO  - Tx TS not in progress
 * * %-EINVAL - invalid sequence ID
 * * %other   - FW error code
 */
int ixgbe_ptp_fw_intr_e600(struct ixgbe_adapter *adapter)
{
	u32 sts = IXGBE_READ_REG(&adapter->hw, PROXY_STS);

	/* Check if FW interrupt was a Tx TS one. */
	if (sts & PROXY_STS_TX_TS_RDY && sts & PROXY_STS_TX_TS_INT) {
		return ixgbe_ptp_tx_phytstamp_e600(adapter, sts);
	} else if (!(sts & PROXY_STS_TX_TS_RDY) && sts & PROXY_STS_TX_TS_INT) {
		atomic_set(&adapter->ptp.reinit_phy_tod, true);
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
		ptp_schedule_worker(adapter->ptp_clock, 0);
#else /* HAVE_PTP_CANCEL_WORKER_SYNC */
		kthread_mod_delayed_work(adapter->ptp.ptp_kworker,
					 &adapter->ptp.ptp_aux_work, 0);
#endif /* !HAVE_PTP_CANCEL_WORKER_SYNC */
		IXGBE_WRITE_REG(&adapter->hw, PROXY_STS, 0);
		return 0;
	}

	return -EINTR;
}

/**
 * ixgbe_ptp_rx_phytstamp_e600 - utility function to get RX tstamp PTP reserved field
 * @q_vector: structure containing interrupt and ring information
 * @skb: the packet
 * @ptp_class: class of the PTP packet
 *
 * This function will be called by the Rx routine of the timestamp for this
 * packet is stored in the buffer.
 */
void ixgbe_ptp_rx_phytstamp_e600(struct ixgbe_q_vector *q_vector,
				 struct sk_buff *skb, unsigned int ptp_class)
{
	struct ixgbe_adapter *adapter = q_vector->adapter;
	struct ptp_header *hdr;
	u64 tstamp;

	skb_reset_mac_header(skb);
	hdr = ptp_parse_header(skb, ptp_class);
	if (!hdr)
		return;

	/* The timestamp is recorded in big endian format, and is stored at
	 * reserved2 field of PTP header as a 32 bit value. The driver needs
	 * to extend it to full 64 bit time.
	 */
	tstamp = ixgbe_ptp_extend_32b_ts_e600(adapter,
					      be32_to_cpu(hdr->reserved2));

	__pskb_trim(skb, skb->len - IXGBE_TS_HDR_LEN);
	skb_hwtstamps(skb)->hwtstamp = ns_to_ktime(tstamp);
}

/**
 * ixgbe_ptp_by_phy_set_params_e600 - configure PTP by PHY parameters
 * @adapter: pointer to the adapter structure
 */
static void ixgbe_ptp_by_phy_set_params_e600(struct ixgbe_adapter *adapter)
{
	u8 ptp_request, flags;
	s32 status;

	/* Enable PTP by PHY on > 1G if one-step TS requested */
	switch (adapter->link_speed) {
	case IXGBE_LINK_SPEED_10_FULL:
	case IXGBE_LINK_SPEED_100_FULL:
	case IXGBE_LINK_SPEED_1GB_FULL:
		break;
	case IXGBE_LINK_SPEED_2_5GB_FULL:
	case IXGBE_LINK_SPEED_5GB_FULL:
	case IXGBE_LINK_SPEED_10GB_FULL:
		if (adapter->ptp.onestep_ena != adapter->ptp.ptp_by_phy_ena) {
			ixgbe_ptp_cfg_phy_timestamping_e600(adapter);
			return;
		}
		if (!adapter->ptp.onestep_ena)
			return;
	default:
		return;
	}

	ptp_request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_SET_PHY_PARAMS;
	if (adapter->ptp.vlan_ena)
		flags = FIELD_PREP(IXGBE_SET_PTP_BY_PHY_ETHERTYPE_M,
				   IXGBE_SET_PTP_BY_PHY_ETHERTYPE_VLAN_TAG);
	else
		flags = FIELD_PREP(IXGBE_SET_PTP_BY_PHY_ETHERTYPE_M,
				   IXGBE_SET_PTP_BY_PHY_ETHERTYPE_NO_VLAN_TAG);

	if (adapter->ptp.onestep_ena)
		flags |= FIELD_PREP(IXGBE_SET_PTP_BY_PHY_TX_TS_M,
				    IXGBE_SET_PTP_BY_PHY_TX_TS_1STEP);

	status = ixgbe_set_ptp_by_phy(&adapter->hw, ptp_request, flags);
	if (status)
		e_dev_err("PTP by PHY set PHY params failed, status=%d\n",
			  status);
}

/**
 * ixgbe_do_aux_work - do PTP periodic work
 * @info: Driver's PTP info structure
 *
 * Do periodic PTP work for PTP by PHY feature. This consists of:
 * - updating cached PHC time at least once per 4.3 seconds,
 * - setting PTP by PHY parameters on VLAN state change, which cannot be
 *   performed in atomic context,
 * - reinitializing PHY ToD on PHY ToD drift interrupt, which cannot be
 *   performed in atomic context.
 *
 * Reschedule work every second to have a safe margin and to ensure that cached
 * PHC time never goes stale, even with OS scheduling delays.
 *
 * Return: delay of the next auxiliary work scheduling time (>=0) or negative
 *         value in case further scheduling is not required
 */
static long ixgbe_do_aux_work(struct ptp_clock_info *info)
{
	struct ixgbe_adapter *adapter = container_of(info, struct ixgbe_adapter,
						     ptp_caps);

	ixgbe_ptp_update_cached_phc_time_e600(info);

	if (atomic_cmpxchg(&adapter->ptp.vlan_change, true, false))
		ixgbe_ptp_by_phy_set_params_e600(adapter);

	if (atomic_cmpxchg(&adapter->ptp.reinit_phy_tod, true, false)) {
		u8 request = IXGBE_SET_PTP_BY_PHY_PTP_REQ_TOD_INIT;
		s32 status;

		status = ixgbe_set_ptp_by_phy(&adapter->hw, request, 0);
		if (status)
			e_dev_err("PTP by PHY ToD reinit failed, status=%d\n",
				  status);
	}

	return msecs_to_jiffies(MSEC_PER_SEC);
}

#ifndef HAVE_PTP_CANCEL_WORKER_SYNC
static void ptp_aux_kworker(struct kthread_work *work)
{
	struct ixgbe_ptp_e600 *ptp = container_of(work, struct ixgbe_ptp_e600,
						  ptp_aux_work.work);
	struct ixgbe_adapter *adapter = container_of(ptp, struct ixgbe_adapter,
						     ptp);
	struct ptp_clock_info *info = &adapter->ptp_caps;
	long delay;

	delay = ixgbe_do_aux_work(info);

	if (delay >= 0)
		kthread_queue_delayed_work(ptp->ptp_kworker, &ptp->ptp_aux_work,
					   delay);
}

#endif /* !HAVE_PTP_CANCEL_WORKER_SYNC */
/**
 * ixgbe_ptp_cfg_phy_vlan_e600 - configure PTP vlan tag timestamping on PHY
 * @adapter: pointer to the adapter structure
 * @enabled: true to indicate to the PHY that VLAN is enabled, false otherwise
 */
void ixgbe_ptp_cfg_phy_vlan_e600(struct ixgbe_adapter *adapter, bool enabled)
{
	if (adapter->ptp.vlan_ena == enabled)
		return;

	adapter->ptp.vlan_ena = enabled;
	if (!adapter->ptp.ptp_by_phy_ena)
		return;

	/* Schedule worker to handle VLAN change. */
	atomic_set(&adapter->ptp.vlan_change, true);
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
	ptp_schedule_worker(adapter->ptp_clock, 0);
#else /* HAVE_PTP_CANCEL_WORKER_SYNC */
	kthread_mod_delayed_work(adapter->ptp.ptp_kworker,
				 &adapter->ptp.ptp_aux_work, 0);
#endif /* !HAVE_PTP_CANCEL_WORKER_SYNC */
}

/**
 * ixgbe_ptp_set_timestamp_mode_e600 - setup the hardware for the requested mode
 * @adapter: the private ixgbe adapter structure
 * @config: the hwtstamp configuration requested
 *
 * Outgoing time stamping can be enabled and disabled. Play nice and
 * disable it when requested, although it shouldn't cause any overhead
 * when no packet needs it. At most one packet in the queue may be
 * marked for time stamping, otherwise it would be impossible to tell
 * for sure to which packet the hardware time stamp belongs.
 *
 * Incoming time stamping has to be configured via the hardware
 * filters. Not all combinations are supported, in particular event
 * type has to be specified. Matching the kind of event packet is
 * not supported, with the exception of "all V2 events regardless of
 * level 2 or 4".
 *
 * Since hardware always timestamps Path delay packets when timestamping V2
 * packets, regardless of the type specified in the register, only use V2
 * Event mode. This more accurately tells the user what the hardware is going
 * to do anyways.
 *
 * Note: this may modify the hwtstamp configuration towards a more general
 * mode, if required to support the specifically requested mode.
 *
 * Return: 0 on success, -EINVAL on invalid flags, -ERANGE on invalid filter.
 */
int ixgbe_ptp_set_timestamp_mode_e600(struct ixgbe_adapter *adapter,
				      struct hwtstamp_config *config)
{
	int rx_mtrl_filter = HWTSTAMP_FILTER_NONE;
	struct ixgbe_hw *hw = &adapter->hw;
	bool onestep_ena = false;
	u32 val;

	/* Reserved for future extensions. */
	if (config->flags)
		return -EINVAL;

	/* Enable/disable TX. */
	val = IXGBE_READ_REG(hw, IXGBE_TSYNCTXCTL);
	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
		val &= ~IXGBE_TSYNCTXCTL_ENABLED;
		break;
	case HWTSTAMP_TX_ON:
		val |= IXGBE_TSYNCTXCTL_ENABLED;
		break;
	case HWTSTAMP_TX_ONESTEP_SYNC:
#ifdef HAVE_PTP_TX_ONESTEP_P2P
	case HWTSTAMP_TX_ONESTEP_P2P:
#endif /* HAVE_PTP_TX_ONESTEP_P2P */
		/* One-step timestamping is supported only on PHY. */
		if (!adapter->hw.dev_caps.common_cap.ptp_by_phy_support)
			return -EOPNOTSUPP;

		val &= ~IXGBE_TSYNCTXCTL_ENABLED;
		onestep_ena = true;
		break;
	default:
		return -ERANGE;
	}
	IXGBE_WRITE_REG(hw, IXGBE_TSYNCTXCTL, val);

	if (onestep_ena != adapter->ptp.onestep_ena) {
		adapter->ptp.onestep_ena = onestep_ena;
		ixgbe_ptp_by_phy_set_params_e600(adapter);
	}

	/* Enable/disable RX. */
	val = IXGBE_READ_REG(hw, IXGBE_TSYNCRXCTL);
	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
#ifdef HAVE_HWTSTAMP_FILTER_NTP_ALL
	case HWTSTAMP_FILTER_NTP_ALL:
#endif /* HAVE_HWTSTAMP_FILTER_NTP_ALL */
	case HWTSTAMP_FILTER_ALL:
		/* Per-packet timestamping only works if the filter is set to
		 * all packets. Since this is desired, always timestamp
		 * all packets as long as any Rx filter was configured.
		 *
		 * Enable timestamping all packets only if at least some packets
		 * were requested. Otherwise, play nice and disable
		 * timestamping.
		 */
		val = IXGBE_TSYNCRXCTL_ENABLED |
		      IXGBE_TSYNCRXCTL_TYPE_ALL |
		      IXGBE_TSYNCRXCTL_TSIP_UT_EN;

		/* Cache original Rx filter for RXMTRL. */
		rx_mtrl_filter = config->rx_filter;
		config->rx_filter = HWTSTAMP_FILTER_ALL;
		adapter->flags |= IXGBE_FLAG_RX_HWTSTAMP_ENABLED;
		adapter->flags &= ~IXGBE_FLAG_RX_HWTSTAMP_IN_REGISTER;
		break;
	case HWTSTAMP_FILTER_NONE:
	default:
		val &= ~(IXGBE_TSYNCRXCTL_ENABLED | IXGBE_TSYNCRXCTL_TYPE_MASK);
		adapter->flags &= ~(IXGBE_FLAG_RX_HWTSTAMP_ENABLED |
				    IXGBE_FLAG_RX_HWTSTAMP_IN_REGISTER);
		if (config->rx_filter != HWTSTAMP_FILTER_NONE) {
			/* Register RXMTRL must be set in order to do V1
			 * packets, therefore it is not possible to timestamp
			 * both V1 Sync and Delay_Req messages unless hardware
			 * supports timestamping all packets => return error.
			 */
			config->rx_filter = HWTSTAMP_FILTER_NONE;
			return -ERANGE;
		}
	}
	IXGBE_WRITE_REG(hw, IXGBE_TSYNCRXCTL, val);

	/* Define which PTP packets are timestamped. */
	switch (rx_mtrl_filter) {
	case HWTSTAMP_FILTER_NONE:
		val = 0;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		val = IXGBE_RXMTRL_V1_SYNC_MSG | PTP_EV_PORT << 16;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		val = IXGBE_RXMTRL_V1_DELAY_REQ_MSG | PTP_EV_PORT << 16;
		break;
	default:
		val = PTP_EV_PORT << 16;
	}
	IXGBE_WRITE_REG(hw, IXGBE_RXMTRL, val);

	/* Define ethertype filter for timestamping L2 packets. */
	if (config->rx_filter != HWTSTAMP_FILTER_NONE)
		IXGBE_WRITE_REG(hw, IXGBE_ETQF(IXGBE_ETQF_FILTER_1588),
				(IXGBE_ETQF_FILTER_EN |
				 IXGBE_ETQF_1588 |
				 ETH_P_1588));
	else
		IXGBE_WRITE_REG(hw, IXGBE_ETQF(IXGBE_ETQF_FILTER_1588), 0);

	IXGBE_WRITE_FLUSH(hw);

	ixgbe_ptp_clear_tx_timestamp_e600(adapter);
	return 0;
}

/**
 * ixgbe_ptp_link_up_e600 - perform PTP adjustments on a link up
 * @adapter: pointer to the adapter structure
 */
void ixgbe_ptp_link_up_e600(struct ixgbe_adapter *adapter)
{
	if (adapter->hw.dev_caps.common_cap.ptp_by_phy_support) {
		ixgbe_ptp_cfg_phy_timestamping_e600(adapter);
		/* Sleep for at least 2 seconds for the PHY to reset, so that
		 * the driver does not report any link flaps.
		 */
		msleep(MSEC_PER_SEC * 3);
	}
}

/**
 * ixgbe_ptp_start_clock_e600 - create the cycle counter from hw
 * @adapter: pointer to the adapter structure
 *
 * This function should be called to set the proper values for the TIMINCA
 * register and tell the cyclecounter structure what the tick rate of SYSTIME
 * is. It does not directly modify SYSTIME registers or the timecounter
 * structure. It should be called whenever a new TIMINCA value is necessary,
 * such as during initialization or when the link speed changes.
 */
static void ixgbe_ptp_start_clock_e600(struct ixgbe_adapter *adapter)
{
	struct ixgbe_hw *hw = &adapter->hw;
	struct timespec64 ts;
	u32 tsauxc;

	/* enable SYSTIME counter */
	ts = ktime_to_timespec64(ktime_get_real());
	tsauxc = IXGBE_READ_REG(hw, IXGBE_TSAUXC);
	IXGBE_WRITE_REG(hw, IXGBE_TSAUXC,
			tsauxc | IXGBE_TSAUXC_DISABLE_SYSTIME);
	IXGBE_WRITE_REG(hw, IXGBE_TIMINCA, 0);
	IXGBE_WRITE_REG(hw, IXGBE_SYSTIMR, 0);
	IXGBE_WRITE_REG(hw, IXGBE_SYSTIML, ts.tv_nsec);
	IXGBE_WRITE_REG(hw, IXGBE_SYSTIMH, timespec64_to_ns(&ts) /
					   NSEC_PER_SEC);
	IXGBE_WRITE_REG(hw, IXGBE_TSAUXC,
			tsauxc & ~IXGBE_TSAUXC_DISABLE_SYSTIME);
	IXGBE_WRITE_REG(hw, IXGBE_TSIM, IXGBE_TSIM_TXTS);
	IXGBE_WRITE_REG(hw, IXGBE_EIMS, IXGBE_EIMS_TIMESYNC);
	IXGBE_WRITE_FLUSH(hw);
}

/**
 * ixgbe_ptp_reset_e600
 * @adapter: the ixgbe private board structure
 *
 * When the MAC resets, all the hardware bits for timesync are reset. This
 * function is used to re-enable the device for PTP based on current settings.
 * We do lose the current clock time, so just reset the cyclecounter to the
 * system real clock time.
 *
 * This function will maintain hwtstamp_config settings, and resets the SDP
 * output if it was enabled.
 */
void ixgbe_ptp_reset_e600(struct ixgbe_adapter *adapter)
{
	/* reset the hardware timestamping mode */
	ixgbe_ptp_set_timestamp_mode_e600(adapter, &adapter->tstamp_config);
	ixgbe_ptp_start_clock_e600(adapter);
}

/**
 * ixgbe_ptp_create_clock - create a PTP clock device
 * @adapter: the ixgbe private adapter structure
 *
 * This function performs setup of the user entry point function table and
 * initializes the PTP clock device, which is used to access the clock-like
 * features of the PTP core. It will be called by ixgbe_ptp_init, and may
 * reuse a previously initialized clock (such as during a suspend/resume
 * cycle).
 *
 * Return: 0 on success, negative error code otherwise.
 */
static long ixgbe_ptp_create_clock_e600(struct ixgbe_adapter *adapter)
{
	struct ptp_clock_info *info = &adapter->ptp_caps;
	struct net_device *netdev = adapter->netdev;
	long err;

	/* Do nothing if we already have a clock device. */
	if (!IS_ERR_OR_NULL(adapter->ptp_clock))
		return 0;

	snprintf(adapter->ptp_caps.name, 16, "%s", netdev->name);
	info->owner = THIS_MODULE;
	info->max_adj = 156249999;
#ifdef HAVE_PTP_CLOCK_INFO_ADJFINE
	info->adjfine = ixgbe_ptp_adjfine_e600;
#else
	info->adjfreq = ixgbe_ptp_adjfreq_e600;
#endif
	info->adjtime = ixgbe_ptp_adjtime_e600;
#ifdef HAVE_PTP_CLOCK_INFO_GETTIME64
#ifdef HAVE_PTP_SYS_OFFSET_EXTENDED_IOCTL
	info->gettimex64 = ixgbe_ptp_gettimex_e600;
#else
	info->gettime64 = ixgbe_ptp_gettime_e600;
#endif /* HAVE_PTP_SYS_OFFSET_EXTENDED_IOCTL */
	info->settime64 = ixgbe_ptp_settime_e600;
#else /* HAVE_PTP_CLOCK_INFO_GETTIME64 */
	info->gettime = ixgbe_ptp_gettime32_e600;
	info->settime = ixgbe_ptp_settime32_e600;
#endif /* !HAVE_PTP_CLOCK_INFO_GETTIME64 */
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
	if (adapter->hw.dev_caps.common_cap.ptp_by_phy_support)
		info->do_aux_work = ixgbe_do_aux_work;
#endif /* HAVE_PTP_CANCEL_WORKER_SYNC */
	info->n_pins = 1;
	info->n_per_out = 1;
	info->pin_config = adapter->ptp.pin_config;
	info->enable = ixgbe_ptp_gpio_enable_e600;
	info->verify = ixgbe_ptp_verify_pin_e600;

	adapter->ptp_clock = ptp_clock_register(&adapter->ptp_caps,
						ixgbe_pf_to_dev(adapter));
	if (IS_ERR(adapter->ptp_clock)) {
		err = PTR_ERR(adapter->ptp_clock);
		adapter->ptp_clock = NULL;
		e_dev_err("ptp_clock_register failed\n");
		return err;
	} else if (adapter->ptp_clock) {
		e_dev_info("Registered PHC device on %s\n", netdev->name);
	}

	return 0;
}

/**
 * ixgbe_ptp_init_e600 - initialize PTP support on HW and SW
 * @adapter: the ixgbe private adapter structure
 *
 * This function performs the required steps for enabling PTP support.
 */
void ixgbe_ptp_init_e600(struct ixgbe_adapter *adapter)
{
	adapter->tstamp_config.rx_filter = HWTSTAMP_FILTER_NONE;
	adapter->tstamp_config.tx_type = HWTSTAMP_TX_OFF;

	adapter->ptp.pin_config[0] = e600_pin_cfg[0];
	/* Obtain a PTP device, or re-use an existing device. */
	if (ixgbe_ptp_create_clock_e600(adapter))
		return;

	INIT_WORK(&adapter->ptp_tx_work, ixgbe_ptp_tx_hwtstamp_work);

#ifndef HAVE_PTP_CANCEL_WORKER_SYNC
	if (adapter->hw.dev_caps.common_cap.ptp_by_phy_support) {
		kthread_init_delayed_work(&adapter->ptp.ptp_aux_work,
					  ptp_aux_kworker);
		adapter->ptp.ptp_kworker = kthread_create_worker(0,
								 "ixgbe_ptp");
		if (IS_ERR(adapter->ptp.ptp_kworker)) {
			e_dev_err("Failed to create ptp aux_worker err=%li\n",
				  PTR_ERR(adapter->ptp.ptp_kworker));
			return;
		}
	}
#endif /* !HAVE_PTP_CANCEL_WORKER_SYNC */
	ixgbe_ptp_reset_e600(adapter);

	set_bit(__IXGBE_PTP_RUNNING, adapter->state);
}

/**
 * ixgbe_ptp_release_e600 - release PTP resources
 * @adapter: pointer to adapter struct
 *
 * Completely destroy the PTP device, should only be called when the device is
 * being fully closed.
 */
void ixgbe_ptp_release_e600(struct ixgbe_adapter *adapter)
{
	struct ptp_perout_request rq = {};

	if (!test_and_clear_bit(__IXGBE_PTP_RUNNING, adapter->state))
		return;

	/* Disable SDP. */
	ixgbe_ptp_setup_sdp_e600(adapter, &rq, false);

	/* Ensure that we cancel any pending PTP Tx work item in progress. */
	cancel_work_sync(&adapter->ptp_tx_work);
	if (adapter->ptp.ptp_by_phy_ena)
#ifdef HAVE_PTP_CANCEL_WORKER_SYNC
		ptp_cancel_worker_sync(adapter->ptp_clock);
#else /* !HAVE_PTP_CANCEL_WORKER_SYNC */
		kthread_cancel_delayed_work_sync(&adapter->ptp.ptp_aux_work);
#endif /* HAVE_PTP_CANCEL_WORKER_SYNC */

	ixgbe_ptp_clear_tx_timestamp_e600(adapter);

	if (adapter->ptp_clock) {
		ptp_clock_unregister(adapter->ptp_clock);
		adapter->ptp_clock = NULL;
		e_dev_info("Removed PHC device on %s\n", adapter->netdev->name);
	}
}
