#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 1999 - 2023 Intel Corporation

set -Eeuo pipefail

# This file generates HAVE_ and NEED_ defines for current kernel
# (or KSRC if provided).
#
# It does so by 'gen' function calls (see body of 'gen-devlink' for examples).
# 'gen' could look for various kinds of declarations in provided kernel headers,
# eg look for an enum in one of files specified and check if given enumeration
# (single value) is present. See 'Documentation' or comment above the 'gen' fun
# in the kcompat-lib.sh.

# Why using bash/awk instead of an old/legacy approach?
#
# The aim is to replicate all the defines provided by human developers
# in the past. Additional bonus is the fact, that we no longer need to care
# about backports done by OS vendors (RHEL, SLES, ORACLE, UBUNTU, more to come).
# We will even work (compile) with only part of backports provided.
#
# To enable smooth transition, especially in time of late fixes, "old" method
# of providing flags should still work as usual.

# End of intro.
# Find info about coding style/rules at the end of file.
# Most of the implementation is in kcompat-lib.sh, here are actual 'gen' calls.

export LC_ALL=C
ORIG_CWD="$(pwd)"
trap 'rc=$?; echo >&2 "$(realpath "$ORIG_CWD/${BASH_SOURCE[0]}"):$LINENO: failed with rc: $rc"' ERR

# shellcheck source=kcompat-lib.sh
source "$ORIG_CWD"/kcompat-lib.sh

# DO NOT break gen calls below (via \), to make our compat code more grep-able,
# keep them also grouped, first by feature (like DEVLINK), then by .h filename
# finally, keep them sorted within a group (sort by flag name)

# handy line of DOC copy-pasted form kcompat-lib.sh:
#   gen DEFINE if (KIND [METHOD of]) NAME [(matches|lacks) PATTERN|absent] in <list-of-files>

function gen-devlink() {
	dh='include/net/devlink.h'
	gen HAVE_DEVLINK_FLASH_UPDATE_BEGIN_END_NOTIFY if fun devlink_flash_update_begin_notify in "$dh"
	gen HAVE_DEVLINK_FLASH_UPDATE_PARAMS    if struct devlink_flash_update_params in "$dh"
	gen HAVE_DEVLINK_FLASH_UPDATE_PARAMS_FW if struct devlink_flash_update_params matches 'struct firmware \\*fw' in "$dh"
	gen HAVE_DEVLINK_HEALTH if enum devlink_health_reporter_state in "$dh"
	gen HAVE_DEVLINK_HEALTH_DEFAULT_AUTO_RECOVER if fun devlink_health_reporter_create lacks auto_recover in "$dh"
	gen HAVE_DEVLINK_HEALTH_OPS_EXTACK if method dump of devlink_health_reporter_ops matches ext_ack in "$dh"
	gen HAVE_DEVLINK_INFO_DRIVER_NAME_PUT if fun devlink_info_driver_name_put in "$dh"
	gen HAVE_DEVLINK_NOTIFY_REGISTER if fun devlink_notify_register in "$dh"
	gen HAVE_DEVLINK_PARAMS if method validate of devlink_param matches ext_ack in "$dh"
	gen HAVE_DEVLINK_PARAMS_PUBLISH if fun devlink_params_publish in "$dh"
	gen HAVE_DEVLINK_PORT_SPLIT_PORT_STRUCT if fun port_split matches devlink_port in "$dh"
	gen HAVE_DEVLINK_RATE_NODE_CREATE if fun devl_rate_node_create in "$dh"
	# keep devlink_region_ops body in variable, to not look 4 times for
	# exactly the same thing in big file
	# please consider it as an example of "how to speed up if needed"
	REGION_OPS="$(find-struct-decl devlink_region_ops "$dh")"
	gen HAVE_DEVLINK_REGIONS if struct devlink_region_ops in - <<< "$REGION_OPS"
	gen HAVE_DEVLINK_REGION_OPS_SNAPSHOT if fun snapshot in - <<< "$REGION_OPS"
	gen HAVE_DEVLINK_REGION_OPS_SNAPSHOT_OPS if fun snapshot matches devlink_region_ops in - <<< "$REGION_OPS"
	gen HAVE_DEVLINK_REGISTER_SETS_DEV if fun devlink_register matches 'struct device' in "$dh"
	gen HAVE_DEVLINK_RELOAD_ENABLE_DISABLE if fun devlink_reload_enable in "$dh"
	gen HAVE_DEVLINK_SET_FEATURES  if fun devlink_set_features in "$dh"
	gen HAVE_DEVL_PORT_REGISTER if fun devl_port_register in "$dh"

	gen HAVE_DEVLINK_RELOAD_ACTION_AND_LIMIT if enum devlink_reload_action matches DEVLINK_RELOAD_ACTION_FW_ACTIVATE in include/uapi/linux/devlink.h
}

function gen-ethtool() {
	eth='include/linux/ethtool.h'
	gen HAVE_ETHTOOL_EXTENDED_RINGPARAMS if method get_ringparam of ethtool_ops matches 'struct kernel_ethtool_ringparam \\*' in "$eth"
}

function gen-filter() {
	fh='include/linux/filter.h'
	gen HAVE_XDP_DO_FLUSH if fun xdp_do_flush_map in "$fh"
	gen NEED_NO_NETDEV_PROG_XDP_WARN_ACTION if fun bpf_warn_invalid_xdp_action lacks 'struct net_device \\*' in "$fh"
}

function gen-flow-dissector() {
	gen HAVE_FLOW_DISSECTOR_KEY_PPPOE if enum flow_dissector_key_id matches FLOW_DISSECTOR_KEY_PPPOE in include/net/flow_dissector.h include/net/flow_keys.h
	# following HAVE ... CVLAN flag is mistakenly named after an enum key,
	# but guards code around function call that was introduced later
	gen HAVE_FLOW_DISSECTOR_KEY_CVLAN if fun flow_rule_match_cvlan in include/net/flow_offload.h
}

function gen-gnss() {
	cdh=include/linux/cdev.h
	clh=include/linux/device/class.h
	gh=include/linux/gnss.h
	th=include/uapi/linux/types.h
	fh=include/linux/fs.h
	if [ -f "$cdh" ]; then
		gen HAVE_CDEV_DEVICE if fun cdev_device_add in "$cdh"
	fi
	if [ -f "$clh" ]; then
		gen HAVE_DEV_UEVENT_CONST if method dev_uevent of class matches 'const struct device' in "$clh"
	fi
	if [ -f "$gh" ]; then
		gen HAVE_GNSS_MODULE if struct gnss_device in "$gh"
	fi
	gen HAVE_POLL_T if macro __poll_t in "$th"
	gen HAVE_STREAM_OPEN if fun stream_open in "$fh"
}

function gen-netdevice() {
	ndh='include/linux/netdevice.h'
	gen HAVE_NDO_FDB_ADD_VID    if method ndo_fdb_del of net_device_ops matches 'u16 vid' in "$ndh"
	gen HAVE_NDO_FDB_DEL_EXTACK if method ndo_fdb_del of net_device_ops matches ext_ack   in "$ndh"
	gen HAVE_NDO_GET_DEVLINK_PORT if method ndo_get_devlink_port of net_device_ops in "$ndh"
	gen HAVE_NDO_UDP_TUNNEL_CALLBACK if method ndo_udp_tunnel_add of net_device_ops in "$ndh"
	gen HAVE_NETIF_SET_TSO_MAX if fun netif_set_tso_max_size in "$ndh"
	gen HAVE_SET_NETDEV_DEVLINK_PORT if macro SET_NETDEV_DEVLINK_PORT in "$ndh"
	gen NEED_NETIF_NAPI_ADD_NO_WEIGHT if fun netif_napi_add matches 'int weight' in "$ndh"
	gen NEED_NET_PREFETCH if fun net_prefetch absent in "$ndh"
}


function gen-other() {
	gen NEED_PCI_AER_CLEAR_NONFATAL_STATUS if fun pci_aer_clear_nonfatal_status absent in include/linux/aer.h
	gen NEED_BITMAP_COPY_CLEAR_TAIL if fun bitmap_copy_clear_tail absent in include/linux/bitmap.h
	gen NEED_BITMAP_FROM_ARR32 if fun bitmap_from_arr32 absent in include/linux/bitmap.h
	gen NEED_BITMAP_TO_ARR32 if fun bitmap_to_arr32 absent in include/linux/bitmap.h
	gen NEED_ETH_HW_ADDR_SET if fun eth_hw_addr_set absent in include/linux/etherdevice.h
	gen NEED_MUL_U64_U64_DIV_U64 if fun mul_u64_u64_div_u64 absent in include/linux/math64.h
	gen HAVE_PCI_MSIX_ALLOC_IRQ_AT if fun pci_msix_alloc_irq_at in include/linux/pci.h
	gen HAVE_PCI_MSIX_CAN_ALLOC_DYN if fun pci_msix_can_alloc_dyn in include/linux/pci.h
	gen HAVE_PCI_MSIX_FREE_IRQ if fun pci_msix_free_irq in include/linux/pci.h
	gen NEED_DIFF_BY_SCALED_PPM if fun diff_by_scaled_ppm absent in include/linux/ptp_clock_kernel.h
	gen NEED_PTP_SYSTEM_TIMESTAMP if fun ptp_read_system_prets absent in include/linux/ptp_clock_kernel.h
	gen HAVE_U64_STATS_FETCH_BEGIN_IRQ if fun u64_stats_fetch_begin_irq in include/linux/u64_stats_sync.h
	gen HAVE_U64_STATS_FETCH_RETRY_IRQ if fun u64_stats_fetch_retry_irq in include/linux/u64_stats_sync.h
}

# all the generations, extracted from main() to keep normal code and various
# prep separated
function gen-all() {
	if grep -qcE CONFIG_NET_DEVLINK.+1 "$CONFFILE"; then
		gen-devlink
	fi
	gen-netdevice
	# code above is covered by unit_tests/test_gold.sh
	if [ -n "${JUST_UNIT_TESTING-}" ]; then
		return
	fi
	gen-ethtool
	gen-filter
	gen-flow-dissector
	gen-gnss
	gen-other
}

function main() {
	# note that this is likely called from makefile and stdout is by slurped default
	if [ -n "${OUT-}" ]; then
		exec > "$OUT"
		# all stdout goes to OUT since now
		echo "/* Autogenerated for KSRC=${KSRC-} via $(basename "$0") */"
	fi
	if [ -d "${KSRC-}" ]; then
		cd "${KSRC}"
	fi

	# check if KSRC was ok/if we are in proper place to look for headers
	if [ -z "$(filter-out-bad-files include/linux/kernel.h)" ]; then
		echo >&2 "seems that there are no kernel includes placed in KSRC=${KSRC}
			pwd=$(pwd); ls -l:"
		ls -l >&2
		exit 8
	fi

	# we need just CONFIG_NET_DEVLINK so far, but it's in .config, required
	if [ ! -f "${CONFFILE-}" ]; then
		echo >&2 ".config should be passed as env CONFFILE
			(and it's not set or not a file)"
		exit 9
	fi

	gen-all

	# dump output, will be visible in CI
	if [ -n "${OUT-}" ]; then
		cd "$ORIG_CWD"
		if [ -n "${JUST_UNIT_TESTING-}${QUIET_COMPAT-}" ]; then
			return
		fi
		cat -n "$OUT" >&2
	fi
}

main

# Coding style:
# - rely on `set -e` handling as much as possible, so:
#  - do not use <(bash process substitution) - it breaks error handling;
#  - do not put substantial logic in `if`-like statement - it disables error
#    handling inside of the conditional (`if big-fun call; then` is substantial)
# - make shellcheck happy - https://www.shellcheck.net
#
# That enables us to move processing out of `if` or `... && ...` statements,
# what finally means that bash error handling (`set -e`) would break on errors.
