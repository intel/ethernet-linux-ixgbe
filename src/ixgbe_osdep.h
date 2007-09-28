/*******************************************************************************

  Intel 10 Gigabit PCI Express Linux driver
  Copyright(c) 1999 - 2007 Intel Corporation.

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
  Linux NICS <linux.nics@intel.com>
  e1000-devel Mailing List <e1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* glue for the OS independent part of ixgbe
 * includes register access macros
 */

#ifndef _IXGBE_OSDEP_H_
#define _IXGBE_OSDEP_H_

#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/if_ether.h>

#include <linux/sched.h>

#include "kcompat.h"

#ifdef DBG
#define DEBUGOUT(S)		printk(KERN_DEBUG S)
#define DEBUGOUT1(S, A...)	printk(KERN_DEBUG S, A)
#else
#define DEBUGOUT(S)             do {} while (0)
#define DEBUGOUT1(S, A...)      do {} while (0)
#endif

#define DEBUGFUNC(F) DEBUGOUT(F)
#define DEBUGOUT2 DEBUGOUT1
#define DEBUGOUT3 DEBUGOUT2
#define DEBUGOUT6 DEBUGOUT3
#define DEBUGOUT7 DEBUGOUT6

#ifdef DBG
#define IXGBE_WRITE_REG(a, reg, value) do {\
	switch (reg) { \
	case IXGBE_EIMS: \
	case IXGBE_EIMC: \
	case IXGBE_EIAM: \
	case IXGBE_EIAC: \
	case IXGBE_EICR: \
	case IXGBE_EICS: \
		printk("%s: Reg - 0x%05X, value - 0x%08X\n", __FUNCTION__, \
			reg, (u32)(value)); \
	default: \
		break; \
	} \
	writel((value), ((a)->hw_addr + (reg))); \
} while (0)
#else
#define IXGBE_WRITE_REG(a, reg, value) writel((value), ((a)->hw_addr + (reg)))
#endif

#define IXGBE_READ_REG(a, reg) readl((a)->hw_addr + (reg))

#define IXGBE_WRITE_REG_ARRAY(a, reg, offset, value) ( \
    writel((value), ((a)->hw_addr + (reg) + ((offset) << 2))))

#define IXGBE_READ_REG_ARRAY(a, reg, offset) ( \
    readl((a)->hw_addr + (reg) + ((offset) << 2)))

#define IXGBE_WRITE_FLUSH(a) IXGBE_READ_REG(a, IXGBE_STATUS)
struct ixgbe_hw;
extern u16 ixgbe_read_pci_cfg_word(struct ixgbe_hw *hw, u32 reg);
#define IXGBE_READ_PCIE_WORD ixgbe_read_pci_cfg_word
#define IXGBE_EEPROM_GRANT_ATTEMPS 100

#endif /* _IXGBE_OSDEP_H_ */
