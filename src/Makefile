################################################################################
#
# Intel 10 Gigabit PCI Express Linux driver
# Copyright (c) 1999 - 2014 Intel Corporation.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Linux NICS <linux.nics@intel.com>
# e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
# Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497
#
################################################################################

ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
#
# Makefile for the Intel(R) 10GbE PCI Express ethernet adapter driver
#

obj-$(CONFIG_IXGBE) += ixgbe.o

define ixgbe-y
	ixgbe_main.o
	ixgbe_api.o
	ixgbe_common.o
	ixgbe_dcb.o
	ixgbe_dcb_82598.o
	ixgbe_dcb_82599.o
	ixgbe_ethtool.o
	ixgbe_lib.o
	ixgbe_mbx.o
	ixgbe_sriov.o
	ixgbe_param.o
	ixgbe_phy.o
	ixgbe_procfs.o
	ixgbe_82598.o
	ixgbe_82599.o
	ixgbe_x540.o
	ixgbe_x550.o
endef
ixgbe-y := $(strip ${ixgbe-y})

ixgbe-${CONFIG_DCB} += ixgbe_dcb_nl.o

ixgbe-${CONFIG_DEBUG_FS} += ixgbe_debugfs.o

ixgbe-${CONFIG_FCOE:m=y} += ixgbe_fcoe.o

ixgbe-$(CONFIG_PTP_1588_CLOCK:m=y) += ixgbe_ptp.o

ixgbe-${CONFIG_SYSFS} += ixgbe_sysfs.o

ixgbe-y += kcompat.o

else	# ifneq($(KERNELRELEASE),)
# normal makefile

#####################
# Helpful functions #
#####################

readlink = $(shell readlink -f ${1})

# helper functions for converting kernel version to version codes
get_kver = $(or $(word ${2},$(subst ., ,${1})),0)
get_kvercode = $(shell [ "${1}" -ge 0 -a "${1}" -le 255 2>/dev/null ] && \
                       [ "${2}" -ge 0 -a "${2}" -le 255 2>/dev/null ] && \
                       [ "${3}" -ge 0 -a "${3}" -le 255 2>/dev/null ] && \
                       printf %d $$(( ( ${1} << 16 ) + ( ${2} << 8 ) + ( ${3} ) )) )

#####################
# Environment setup #
#####################

SOURCES := $(shell find . -name "*.[ch]" | grep -v "\.mod\.c")
DRIVER := ixgbe
DRIVER_UPPERCASE := $(shell echo ${DRIVER} | tr "[:lower:]" "[:upper:]")

#####################
# Environment tests #
#####################

ifeq (,${BUILD_KERNEL})
BUILD_KERNEL=$(shell uname -r)
endif

# Kernel Search Path
# All the places we look for kernel source
KSP :=  /lib/modules/${BUILD_KERNEL}/build \
        /lib/modules/${BUILD_KERNEL}/source \
        /usr/src/linux-${BUILD_KERNEL} \
        /usr/src/linux-$(${BUILD_KERNEL} | sed 's/-.*//') \
        /usr/src/kernel-headers-${BUILD_KERNEL} \
        /usr/src/kernel-source-${BUILD_KERNEL} \
        /usr/src/linux-$(${BUILD_KERNEL} | sed 's/\([0-9]*\.[0-9]*\)\..*/\1/') \
        /usr/src/linux \
        /usr/src/kernels/${BUILD_KERNEL} \
        /usr/src/kernels

# prune the list down to only values that exist and have an include/linux
# sub-directory. We can't use include/config because some older kernels don't
# have this.
test_dir = $(shell [ -e ${dir}/include/linux ] && echo ${dir})
KSP := $(foreach dir, ${KSP}, ${test_dir})

# we will use this first valid entry in the search path
ifeq (,${KSRC})
  KSRC := $(firstword ${KSP})
endif

ifeq (,${KSRC})
  $(warning *** Kernel header files not in any of the expected locations.)
  $(warning *** Install the appropriate kernel development package, e.g.)
  $(error *** kernel-devel, for building kernel modules and try again)
else
ifeq (/lib/modules/${BUILD_KERNEL}/source, ${KSRC})
  KOBJ :=  /lib/modules/${BUILD_KERNEL}/build
else
  KOBJ :=  ${KSRC}
endif
endif

# Version file Search Path
VSP :=  ${KOBJ}/include/generated/utsrelease.h \
        ${KOBJ}/include/linux/utsrelease.h \
        ${KOBJ}/include/linux/version.h \
        ${KOBJ}/include/generated/uapi/linux/version.h \
        /boot/vmlinuz.version.h

# Config file Search Path
CSP :=  ${KOBJ}/include/generated/autoconf.h \
        ${KOBJ}/include/linux/autoconf.h \
        /boot/vmlinuz.autoconf.h

# System.map Search Path (for depmod)
MSP := ${KSRC}/System.map \
       /boot/System.map-${BUILD_KERNEL}

# prune the lists down to only files that exist
test_file = $(shell [ -f ${file} ] && echo ${file})
VSP := $(foreach file, ${VSP}, ${test_file})
CSP := $(foreach file, ${CSP}, ${test_file})
MSP := $(foreach file, ${MSP}, ${test_file})

# and use the first valid entry in the Search Paths
ifeq (,${VERSION_FILE})
  VERSION_FILE := $(firstword ${VSP})
endif

ifeq (,${CONFIG_FILE})
  CONFIG_FILE := $(firstword ${CSP})
endif

ifeq (,${SYSTEM_MAP_FILE})
  SYSTEM_MAP_FILE := $(firstword ${MSP})
endif

ifeq (,$(wildcard ${VERSION_FILE}))
  $(error Linux kernel source not configured - missing version header file)
endif

ifeq (,$(wildcard ${CONFIG_FILE}))
  $(error Linux kernel source not configured - missing autoconf.h)
endif

ifeq (,$(wildcard ${SYSTEM_MAP_FILE}))
  $(warning Missing System.map file - depmod will not check for missing symbols)
endif

#######################
# Linux Version Setup #
#######################

# The following command line parameter is intended for development of KCOMPAT
# against upstream kernels such as net-next which have broken or non-updated
# version codes in their Makefile. They are intended for debugging and
# development purpose only so that we can easily test new KCOMPAT early. If you
# don't know what this means, you do not need to set this flag. There is no
# arcane magic here.

# Convert LINUX_VERSION into LINUX_VERSION_CODE
ifneq (${LINUX_VERSION},)
  LINUX_VERSION_CODE=$(call get_kvercode,$(call get_kver,${LINUX_VERSION},1),$(call get_kver,${LINUX_VERSION},2),$(call get_kver,${LINUX_VERSION},3))
endif

# Honor LINUX_VERSION_CODE
ifneq (${LINUX_VERSION_CODE},)
  $(warning Forcing target kernel to build with LINUX_VERSION_CODE of ${LINUX_VERSION_CODE}$(if ${LINUX_VERSION}, from LINUX_VERSION=${LINUX_VERSION}). Do this at your own risk.)
  KVER_CODE := ${LINUX_VERSION_CODE}
  EXTRA_CFLAGS += -DLINUX_VERSION_CODE=${LINUX_VERSION_CODE}
endif

EXTRA_CFLAGS += ${CFLAGS_EXTRA}


# get the kernel version - we use this to find the correct install path
KVER := $(shell ${CC} ${EXTRA_CFLAGS} -E -dM ${VERSION_FILE} | grep UTS_RELEASE | \
        awk '{ print $$3 }' | sed 's/\"//g')

# assume source symlink is the same as build, otherwise adjust KOBJ
ifneq (,$(wildcard /lib/modules/${KVER}/build))
ifneq (${KSRC},$(call readlink,/lib/modules/${KVER}/build))
  KOBJ=/lib/modules/${KVER}/build
endif
endif

ifeq (${KVER_CODE},)
  KVER_CODE := $(shell ${CC} ${EXTRA_CFLAGS} -E -dM ${VSP} 2> /dev/null |\
                 grep -m 1 LINUX_VERSION_CODE | awk '{ print $$3 }' | sed 's/\"//g')
endif

# set correct CCFLAGS variable for kernels older than 2.6.24
ifeq (0,$(shell [ ${KVER_CODE} -lt $(call get_kvercode,2,6,24) ]; echo $$?))
CCFLAGS_VAR := EXTRA_CFLAGS
else
CCFLAGS_VAR := ccflags-y
endif

# abort the build on kernels older than 2.6.0
ifeq (0,$(shell [ ${KVER_CODE} -lt $(call get_kvercode,2,6,0) ]; echo $$? ))
  $(warning *** Aborting the build.)
  $(error This driver is not supported on kernel versions older than 2.6.0)
endif

################
# Manual Pages #
################

MANSECTION = 7

ifeq (,${MANDIR})
  # find the best place to install the man page
  MANPATH := $(shell (manpath 2>/dev/null || echo $MANPATH) | sed 's/:/ /g')
  ifneq (,${MANPATH})
    # test based on inclusion in MANPATH
    test_dir = $(findstring ${dir}, ${MANPATH})
  else
    # no MANPATH, test based on directory existence
    test_dir = $(shell [ -e ${dir} ] && echo ${dir})
  endif
  # our preferred install path
  # should /usr/local/man be in here ?
  MANDIR := /usr/share/man /usr/man
  MANDIR := $(foreach dir, ${MANDIR}, ${test_dir})
  MANDIR := $(firstword ${MANDIR})
endif
ifeq (,${MANDIR})
  # fallback to /usr/man
  MANDIR := /usr/man
endif

############################
# Module Install Directory #
############################

# Default to using updates/drivers/net/ethernet/intel/ path, since depmod since
# v3.1 defaults to checking updates folder first, and only checking kernels/
# and extra afterwards. We use updates instead of kernel/* due to desire to
# prevent over-writing built-in modules files.
INSTALL_MOD_DIR ?= updates/drivers/net/ethernet/intel/${DRIVER}

######################
# Kernel Build Macro #
######################

# kernel build function
# ${1} is the kernel build target
# ${2] may contain any extra rules to pass directly to the sub-make process
kernelbuild = ${MAKE} $(if ${GCC_I_SYS},CC:="${GCC_I_SYS}") \
                      ${CCFLAGS_VAR}:="${EXTRA_CFLAGS}" \
                      -C ${KSRC} \
                      $(if ${KOBJ},O:=${KOBJ}) \
                      CONFIG_${DRIVER_UPPERCASE}=m \
                      $(if $(filter -DNO_PTP_SUPPORT,${EXTRA_CFLAGS}),CONFIG_PTP_1588_CLOCK=n) \
                      M:=$(call readlink,.) \
                      $(if ${INSTALL_MOD_PATH},INSTALL_MOD_PATH:=${INSTALL_MOD_PATH}) \
                      INSTALL_MOD_DIR:=${INSTALL_MOD_DIR} \
                      ${2} ${1};

################
# depmod Macro #
################

cmd_depmod = /sbin/depmod $(if ${SYSTEM_MAP_FILE},-e -F ${SYSTEM_MAP_FILE}) \
                          $(if $(strip ${INSTALL_MOD_PATH}),-b ${INSTALL_MOD_PATH}) \
                          -a ${KVER}

###############
# Build rules #
###############

# Standard compilation, with regular output
default: ${SOURCES}
	@+$(call kernelbuild,modules)

# Noisy output, for extra debugging
noisy: ${SOURCES}
	@+$(call kernelbuild,modules,V=1)

# Silence any output generated
silent: ${SOURCES}
	@+$(call kernelbuild,modules,>/dev/null)

# Enable higher warning level
checkwarnings: clean ${SOURCES}
	@+$(call kernelbuild,modules,W=1)

# Run sparse static analyzer
sparse: clean ${SOURCES}
	@+$(call kernelbuild,modules,C=2 CF="-D__CHECK_ENDIAN__ -Wbitwise -Wcontext")

# Run coccicheck static analyzer
ccc: clean ${SOURCES}
	@+$(call kernelbuild,modules,coccicheck MODE=report))

# Build manfiles
manfile:
	@gzip -c ../${DRIVER}.${MANSECTION} > ${DRIVER}.${MANSECTION}.gz

# Clean the module subdirectories
clean:
	@+$(call kernelbuild,clean)
	@-rm -rf *.${MANSECTION}.gz *.ko

# Install the modules and manpage
install: default manfile
# remove all old versions of the driver
	install -D -m 644 ${DRIVER}.${MANSECTION}.gz ${INSTALL_MOD_PATH}${MANDIR}/man${MANSECTION}/${DRIVER}.${MANSECTION}.gz
	@$(call kernelbuild,modules_install)
	$(call cmd_depmod)

uninstall:
	rm -f ${INSTALL_MOD_PATH}/lib/modules/${KVER}/${INSTALL_MOD_DIR}/${DRIVER}.ko;
	$(call cmd_depmod)
	if [ -e ${INSTALL_MOD_PATH}${MANDIR}/man${MANSECTION}/${DRIVER}.${MANSECTION}.gz ] ; then \
		rm -f ${INSTALL_MOD_PATH}${MANDIR}/man${MANSECTION}/${DRIVER}.${MANSECTION}.gz ; \
	fi;

########
# Help #
########
help:
	@echo 'Cleaning targets:'
	@echo '  clean               - Clean files generated by kernel module build'
	@echo 'Build targets:'
	@echo '  default             - Build module(s) with standard verbosity'
	@echo '  noisy               - Build module(s) with V=1 verbosity -- very noisy'
	@echo '  silent              - Build module(s), squelching all output'
	@echo 'Static Analysis:'
	@echo '  checkwarnings       - Clean, then build module(s) with W=1 warnings enabled'
	@echo '  sparse              - Clean, then check module(s) using sparse'
	@echo '  ccc                 - Clean, then check module(s) using coccicheck'
	@echo 'Other targets:'
	@echo '  manfile             - Generate a gzipped manpage'
	@echo '  install             - Build then install the module(s) and manpage'
	@echo '  uninstall           - Uninstall the module(s) and manpage'
	@echo '  help                - Display this help message'
	@echo 'Variables:'
	@echo '  LINUX_VERSION       - Debug tool to force kernel LINUX_VERSION_CODE. Use at your own risk.'
	@echo '  W=N                 - Kernel variable for setting warning levels'
	@echo '  V=N                 - Kernel variable for setting output verbosity'
	@echo '  INSTALL_MOD_PATH    - Add prefix for the module and manpage installation path'
	@echo '  INSTALL_MOD_DIR     - Use module directory other than updates/drivers/net/ethernet/intel/${DRIVER}'
	@echo ' Other variables may be available for tuning make process, see'
	@echo ' Kernel Kbuild documentation for more information'

.PHONY: default noisy clean manfile silent sparse ccc install uninstall help

endif	# ifneq($(KERNELRELEASE),)
