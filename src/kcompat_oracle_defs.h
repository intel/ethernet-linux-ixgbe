/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright(c) 1999 - 2022 Intel Corporation. */

#ifndef _KCOMPAT_ORACLE_DEFS_H_
#define _KCOMPAT_ORACLE_DEFS_H_

/* UEK kernel versions are a combination of the LINUX_VERSION_CODE along with
 * an extra 3 digits. This values are part of BUILD_KERNEL string, and first
 * number extracted by common.mk and placed into UEK_RELEASE_NUMBER.
 *
 * We combine the value of UEK_RELEASE_NUMBER along with the LINUX_VERSION code
 * to generate the useful value that determines what specific kernel we're
 * dealing with.
 *
 * Just in case the UEK_RELEASE_NUMBER ever goes above 255, we reserve 16 bits
 * instead of 8 for this value.
 */
#if !defined(UEK_RELEASE_NUMBER)
#error "UEK_RELEASE_NUMBER is undefined"
#endif

#if !defined(UEK_MINOR_RELEASE_NUMBER)
#error "UEK_MINOR_RELEASE_NUMBER is undefined"
#endif

#if UEK_RELEASE_NUMBER > 65535
#error "UEK_RELEASE_NUMBER is unexpectedly large"
#endif

#define UEK_KERNEL_CODE ((LINUX_VERSION_CODE << 16) + UEK_RELEASE_NUMBER)
#define UEK_KERNEL_VERSION(a, b, c, d) ((KERNEL_VERSION(a, b, c) << 16) + (d))

#if UEK_KERNEL_VERSION(5, 4, 17, 2136) > UEK_KERNEL_CODE
#define NEED_ORCL_LIN_PCI_AER_CLEAR_NONFATAL_STATUS
#endif

#if UEK_KERNEL_VERSION(5, 4, 17, 2136) == UEK_KERNEL_CODE
#if UEK_MINOR_RELEASE_NUMBER >= 305
#undef NEED_NET_PREFETCH
#endif
#endif

#endif /* _KCOMPAT_ORACLE_DEFS_H_ */
