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

#include <linux/types.h>
#include <linux/module.h>

#include "ixgbe.h"

/* This is the only thing that needs to be changed to adjust the
 * maximum number of ports that the driver can manage.
 */

#define IXGBE_MAX_NIC 8

#define OPTION_UNSET    -1
#define OPTION_DISABLED 0
#define OPTION_ENABLED  1

/* All parameters are treated the same, as an integer array of values.
 * This macro just reduces the need to repeat the same declaration code
 * over and over (plus this helps to avoid typo bugs).
 */

#define IXGBE_PARAM_INIT { [0 ... IXGBE_MAX_NIC] = OPTION_UNSET }
#ifndef module_param_array
/* Module Parameters are always initialized to -1, so that the driver
 * can tell the difference between no user specified value or the
 * user asking for the default value.
 * The true default values are loaded in when ixgbe_check_options is called.
 *
 * This is a GCC extension to ANSI C.
 * See the item "Labeled Elements in Initializers" in the section
 * "Extensions to the C Language Family" of the GCC documentation.
 */

#define IXGBE_PARAM(X, desc) \
	static const int __devinitdata X[IXGBE_MAX_NIC+1] = IXGBE_PARAM_INIT; \
	MODULE_PARM(X, "1-" __MODULE_STRING(IXGBE_MAX_NIC) "i"); \
	MODULE_PARM_DESC(X, desc);
#else
#define IXGBE_PARAM(X, desc) \
	static int __devinitdata X[IXGBE_MAX_NIC+1] = IXGBE_PARAM_INIT; \
	static int num_##X; \
	module_param_array_named(X, X, int, &num_##X, 0); \
	MODULE_PARM_DESC(X, desc);
#endif

/* Interrupt Type
 *
 * Valid Range: 0-2
 *  - 0 - Legacy Interrupt
 *  - 1 - MSI Interrupt
 *  - 2 - MSI-X Interrupt(s)
 *
 * Default Value: 2
 */
IXGBE_PARAM(InterruptType, "Change Interrupt Mode (0=Legacy, 1=MSI, 2=MSI-X)");
#define IXGBE_INT_LEGACY		      0
#define IXGBE_INT_MSI			      1
#define IXGBE_INT_MSIX			      2
#define IXGBE_DEFAULT_INT	 IXGBE_INT_MSIX

/* Rx Queues count
 *
 * Valid Range: 1, 2, 4, 8
 *
 * Default Value: 8 (1 in NAPI mode)
 */
IXGBE_PARAM(RxQueues, "Number of RX queues");

/* Receive Flow control high threshold (when we send a pause frame)
 * (FCRTH)
 *
 * Valid Range: 1,536 - 524,280 (0x600 - 0x7FFF8, 8 byte granularity)
 *
 * Default Value: 196,608 (0x30000)
 */
IXGBE_PARAM(RxFCHighThresh, "Receive Flow Control High Threshold");
#define DEFAULT_FCRTH            0x20000
#define MIN_FCRTH                      0
#define MAX_FCRTH                0x7FFF0

/* Receive Flow control low threshold (when we send a resume frame)
 * (FCRTL)
 *
 * Valid Range: 64 - 524,160 (0x40 - 0x7FF80, 8 byte granularity)
 *              must be less than high threshold by at least 8 bytes
 *
 * Default Value:  163,840 (0x28000)
 */
IXGBE_PARAM(RxFCLowThresh, "Receive Flow Control Low Threshold");
#define DEFAULT_FCRTL            0x10000
#define MIN_FCRTL                      0
#define MAX_FCRTL                0x7FF80

/* Flow control request timeout (how long to pause the link partner's tx)
 * (PAP 15:0)
 *
 * Valid Range: 1 - 65535
 *
 * Default Value:  65535 (0xffff) (we'll send an xon if we recover)
 */
IXGBE_PARAM(FCReqTimeout, "Flow Control Request Timeout");
#define DEFAULT_FCPAUSE           0x6800  /* this may be too long */
#define MIN_FCPAUSE                    0
#define MAX_FCPAUSE               0xFFFF

/* Interrupt Throttle Rate (interrupts/sec)
 *
 * Valid Range: 100-500000 (0=off)
 *
 * Default Value: 8000
 */
IXGBE_PARAM(InterruptThrottleRate, "Maximum interrupts per second, per vector");
#define DEFAULT_ITR                 8000
#define MAX_ITR                   500000
#define MIN_ITR                      100

#ifndef IXGBE_NO_LLI
/* LLIPort (Low Latency Interrupt TCP Port)
 *
 * Valid Range: 0 - 65535
 *
 * Default Value: 0 (disabled)
 */
IXGBE_PARAM(LLIPort, "Low Latency Interrupt TCP Port");

#define DEFAULT_LLIPORT                0
#define MAX_LLIPORT               0xFFFF
#define MIN_LLIPORT                    0

/* LLIPush (Low Latency Interrupt on TCP Push flag)
 *
 * Valid Range: 0,1
 *
 * Default Value: 0 (disabled)
 */
IXGBE_PARAM(LLIPush, "Low Latency Interrupt on TCP Push flag");

#define DEFAULT_LLIPUSH                0
#define MAX_LLIPUSH                    1
#define MIN_LLIPUSH                    0

/* LLISize (Low Latency Interrupt on Packet Size)
 *
 * Valid Range: 0 - 1500
 *
 * Default Value: 0 (disabled)
 */
IXGBE_PARAM(LLISize, "Low Latency Interrupt on Packet Size");

#define DEFAULT_LLISIZE                0
#define MAX_LLISIZE                 1500
#define MIN_LLISIZE                    0
#endif /* IXGBE_NO_LLI */

/* Rx buffer mode
 *
 * Valid Range: 0-2 0 = 1buf_mode_always, 1 = ps_mode_always and 2 = optimal
 *
 * Default Value: 2
 */
IXGBE_PARAM(RxBufferMode, "Rx Buffer Mode - Packet split or one buffer in rx");

#define IXGBE_RXBUFMODE_1BUF_ALWAYS     0
#define IXGBE_RXBUFMODE_PS_ALWAYS       1
#define IXGBE_RXBUFMODE_OPTIMAL         2
#define IXGBE_DEFAULT_RXBUFMODE         IXGBE_RXBUFMODE_OPTIMAL



struct ixgbe_option {
	enum { enable_option, range_option, list_option } type;
	char *name;
	char *err;
	int  def;
	union {
		struct { /* range_option info */
			int min;
			int max;
		} r;
		struct { /* list_option info */
			int nr;
			struct ixgbe_opt_list {
				int i;
				char *str;
			} *p;
		} l;
	} arg;
};

static int __devinit ixgbe_validate_option(int *value, struct ixgbe_option *opt)
{
	if (*value == OPTION_UNSET) {
		*value = opt->def;
		return 0;
	}

	switch (opt->type) {
	case enable_option:
		switch (*value) {
		case OPTION_ENABLED:
			printk(KERN_INFO "ixgbe: %s Enabled\n", opt->name);
			return 0;
		case OPTION_DISABLED:
			printk(KERN_INFO "ixgbe: %s Disabled\n", opt->name);
			return 0;
		}
		break;
	case range_option:
		if (*value >= opt->arg.r.min && *value <= opt->arg.r.max) {
			printk(KERN_INFO "ixgbe: %s set to %i\n", opt->name, *value);
			return 0;
		}
		break;
	case list_option: {
		int i;
		struct ixgbe_opt_list *ent;

		for (i = 0; i < opt->arg.l.nr; i++) {
			ent = &opt->arg.l.p[i];
			if (*value == ent->i) {
				if (ent->str[0] != '\0')
					printk(KERN_INFO "%s\n", ent->str);
				return 0;
			}
		}
	}
		break;
	default:
		BUG();
	}

	printk(KERN_INFO "Invalid %s specified (%i) %s\n",
	       opt->name, *value, opt->err);
	*value = opt->def;
	return -1;
}

#define LIST_LEN(l) (sizeof(l) / sizeof(l[0]))

/**
 * ixgbe_check_options - Range Checking for Command Line Parameters
 * @adapter: board private structure
 *
 * This routine checks all command line parameters for valid user
 * input.  If an invalid value is given, or if no user specified
 * value exists, a default value is used.  The final value is stored
 * in a variable in the adapter structure.
 **/
void __devinit ixgbe_check_options(struct ixgbe_adapter *adapter)
{
	int bd = adapter->bd_number;
	struct ixgbe_hw *hw = &adapter->hw;

	if (bd >= IXGBE_MAX_NIC) {
		printk(KERN_NOTICE
		       "Warning: no configuration for board #%i\n", bd);
		printk(KERN_NOTICE "Using defaults for all values\n");
#ifndef module_param_array
		bd = IXGBE_MAX_NIC;
#endif
	}

	{ /* Interrupt Type */
		int int_type;
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Interrupt Type",
			.err = "using default of "
					__MODULE_STRING(IXGBE_DEFAULT_INT),
			.def = IXGBE_DEFAULT_INT,
			.arg = {.r = {.min = IXGBE_INT_LEGACY,
				      .max = IXGBE_INT_MSIX}}
		};

#ifdef module_param_array
		if (num_InterruptType > bd) {
#endif
			int_type = InterruptType[bd];
			ixgbe_validate_option(&int_type, &opt);
			switch (int_type) {
			case IXGBE_INT_MSIX:
				adapter->flags |= IXGBE_FLAG_MSIX_CAPABLE;
				adapter->flags |= IXGBE_FLAG_MSI_CAPABLE;
				break;
			case IXGBE_INT_MSI:
				adapter->flags |= IXGBE_FLAG_MSI_CAPABLE;
				break;
			case IXGBE_INT_LEGACY:
			default:
				break;
			}
#ifdef module_param_array
		} else {
			adapter->flags |= IXGBE_FLAG_MSIX_CAPABLE;
			adapter->flags |= IXGBE_FLAG_MSI_CAPABLE;
		}
#endif
	}
	{ /* Receive Queues Count */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Receive Queues",
			.err = "using default of "
					__MODULE_STRING(IXGBE_DEFAULT_RXQ),
			.def = IXGBE_DEFAULT_RXQ,
			.arg = {.r = {.min = IXGBE_MIN_RXQ,
				      .max = IXGBE_MAX_RXQ}}
		};

#ifdef module_param_array
		if (num_RxQueues > bd) {
#endif
			adapter->num_rx_queues = RxQueues[bd];

#ifndef CONFIG_IXGBE_NAPI
			if (adapter->num_rx_queues > 4)
				adapter->num_rx_queues = 8;
			else if (adapter->num_rx_queues > 2)
				adapter->num_rx_queues = 4;
#endif

			ixgbe_validate_option(&adapter->num_rx_queues, &opt);

#ifdef module_param_array
		} else {
			adapter->num_rx_queues = opt.def;
		}
#endif
	}
	{ /* Receive Flow Control High Threshold */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Rx Flow Control High Threshold",
			.err  =
			     "using default of " __MODULE_STRING(DEFAULT_FCRTH),
			.def  = DEFAULT_FCRTH,
			.arg  = { .r = { .min = MIN_FCRTH,
					 .max = MAX_FCRTH}}
		};

#ifdef module_param_array
		if (num_RxFCHighThresh > bd) {
#endif
			hw->fc.high_water = RxFCHighThresh[bd];
			ixgbe_validate_option(&hw->fc.high_water, &opt);
#ifdef module_param_array
		} else {
			hw->fc.high_water = opt.def;
		}
#endif
		if (!(hw->fc.type & ixgbe_fc_tx_pause))
			printk(KERN_INFO
			       "Ignoring RxFCHighThresh when no RxFC\n");
	}
	{ /* Receive Flow Control Low Threshold */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Rx Flow Control Low Threshold",
			.err  =
			     "using default of " __MODULE_STRING(DEFAULT_FCRTL),
			.def  = DEFAULT_FCRTL,
			.arg  = { .r = { .min = MIN_FCRTL,
					 .max = MAX_FCRTL}}
		};

#ifdef module_param_array
		if (num_RxFCLowThresh > bd) {
#endif
			hw->fc.low_water = RxFCLowThresh[bd];
			ixgbe_validate_option(&hw->fc.low_water, &opt);
#ifdef module_param_array
		} else {
			hw->fc.low_water = opt.def;
		}
#endif
		if (!(hw->fc.type & ixgbe_fc_tx_pause))
			printk(KERN_INFO
			       "Ignoring RxFCLowThresh when no RxFC\n");
	}
	{ /* Flow Control Pause Time Request */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Flow Control Pause Time Request",
			.err  =
			    "using default of "__MODULE_STRING(DEFAULT_FCPAUSE),
			.def  = DEFAULT_FCPAUSE,
			.arg = { .r = { .min = MIN_FCPAUSE,
					.max = MAX_FCPAUSE}}
		};

#ifdef module_param_array
		if (num_FCReqTimeout > bd) {
#endif
			int pause_time = FCReqTimeout[bd];
			ixgbe_validate_option(&pause_time, &opt);
			hw->fc.pause_time = pause_time;
#ifdef module_param_array
		} else {
			hw->fc.pause_time = opt.def;
		}
#endif
		if (!(hw->fc.type & ixgbe_fc_tx_pause))
			printk(KERN_INFO
			       "Ignoring FCReqTimeout when no RxFC\n");
	}
	/* high low and spacing check for rx flow control thresholds */
	if (hw->fc.type & ixgbe_fc_tx_pause) {
		/* high must be greater than low */
		if (hw->fc.high_water < (hw->fc.low_water + 8)) {
			/* set defaults */
			printk(KERN_INFO
			       "RxFCHighThresh must be >= (RxFCLowThresh + 8),"
			       " Using Defaults\n");
			hw->fc.high_water = DEFAULT_FCRTH;
			hw->fc.low_water  = DEFAULT_FCRTL;
		}
		/* enable the sending of XON frames, otherwise the long pause
		 * timeout is not good */
		hw->fc.send_xon = 1;
	}
	{ /* Interrupt Throttling Rate */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Interrupt Throttling Rate (ints/sec)",
			.err  = "using default of "__MODULE_STRING(DEFAULT_ITR),
			.def  = DEFAULT_ITR,
			.arg  = { .r = { .min = MIN_ITR,
					 .max = MAX_ITR }}
		};

#ifdef module_param_array
		if (num_InterruptThrottleRate > bd) {
#endif
			adapter->eitr = InterruptThrottleRate[bd];
			switch (adapter->eitr) {
			case 0:
				DPRINTK(PROBE, INFO, "%s turned off\n",
					opt.name);
				/* zero is a special value, we don't want to
				 * turn off ITR completely, just set it to an
				 * insane interrupt rate (like 3.5 Million
				 * ints/s */
				adapter->eitr = EITR_REG_TO_INTS_PER_SEC(1);
				break;
			default:
				ixgbe_validate_option(&adapter->eitr, &opt);
				break;
			}
#ifdef module_param_array
		} else {
			adapter->eitr = DEFAULT_ITR;
		}
#endif
	}
#ifndef IXGBE_NO_LLI
	{ /* Low Latency Interrupt TCP Port*/
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Low Latency Interrupt TCP Port",
			.err  = "using default of "
					__MODULE_STRING(DEFAULT_LLIPORT),
			.def  = DEFAULT_LLIPORT,
			.arg  = { .r = { .min = MIN_LLIPORT,
					 .max = MAX_LLIPORT }}
		};

#ifdef module_param_array
		if (num_LLIPort > bd) {
#endif
			adapter->lli_port = LLIPort[bd];
			if (adapter->lli_port) {
				ixgbe_validate_option(&adapter->lli_port, &opt);
			} else {
				DPRINTK(PROBE, INFO, "%s turned off\n",
					opt.name);
			}
#ifdef module_param_array
		} else {
			adapter->lli_port = opt.def;
		}
#endif
	}
	{ /* Low Latency Interrupt on Packet Size */
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Low Latency Interrupt on Packet Size",
			.err  = "using default of "
					__MODULE_STRING(DEFAULT_LLISIZE),
			.def  = DEFAULT_LLISIZE,
			.arg  = { .r = { .min = MIN_LLISIZE,
					 .max = MAX_LLISIZE }}
		};

#ifdef module_param_array
		if (num_LLISize > bd) {
#endif
			adapter->lli_size = LLISize[bd];
			if (adapter->lli_size) {
				ixgbe_validate_option(&adapter->lli_size, &opt);
			} else {
				DPRINTK(PROBE, INFO, "%s turned off\n",
					opt.name);
			}
#ifdef module_param_array
		} else {
			adapter->lli_size = opt.def;
		}
#endif
	}
	{ /*Low Latency Interrupt on TCP Push flag*/
		struct ixgbe_option opt = {
			.type = enable_option,
			.name = "Low Latency Interrupt on TCP Push flag",
			.err  = "defaulting to Disabled",
			.def  = OPTION_DISABLED
		};

#ifdef module_param_array
		if (num_LLIPush > bd) {
#endif
			int lli_push = LLIPush[bd];
			ixgbe_validate_option(&lli_push, &opt);
			if (lli_push)
				adapter->flags |= IXGBE_FLAG_LLI_PUSH;
			else
				adapter->flags &= ~IXGBE_FLAG_LLI_PUSH;
#ifdef module_param_array
		} else {
			if (opt.def == OPTION_ENABLED)
				adapter->flags |= IXGBE_FLAG_LLI_PUSH;
			else
				adapter->flags &= ~IXGBE_FLAG_LLI_PUSH;
		}
#endif
	}
#endif /* IXGBE_NO_LLI */
	{ /* Rx buffer mode */
		int rx_buf_mode;
		struct ixgbe_option opt = {
			.type = range_option,
			.name = "Rx buffer mode",
			.err = "using default of "
				__MODULE_STRING(IXGBE_DEFAULT_RXBUFMODE),
			.def = IXGBE_DEFAULT_RXBUFMODE,
			.arg = {.r = {.min = IXGBE_RXBUFMODE_1BUF_ALWAYS,
				      .max = IXGBE_RXBUFMODE_OPTIMAL}}
		};

#ifdef module_param_array
		if (num_RxBufferMode > bd) {
#endif
			rx_buf_mode = RxBufferMode[bd];
			ixgbe_validate_option(&rx_buf_mode, &opt);
			switch (rx_buf_mode) {
			case IXGBE_RXBUFMODE_OPTIMAL:
				adapter->flags |= IXGBE_FLAG_RX_1BUF_CAPABLE;
				adapter->flags |= IXGBE_FLAG_RX_PS_CAPABLE;
				break;
			case IXGBE_RXBUFMODE_PS_ALWAYS:
				adapter->flags |= IXGBE_FLAG_RX_PS_CAPABLE;
				break;
			case IXGBE_RXBUFMODE_1BUF_ALWAYS:
				adapter->flags |= IXGBE_FLAG_RX_1BUF_CAPABLE;
			default:
				break;
			}
#ifdef module_param_array
		} else {
			adapter->flags |= IXGBE_FLAG_RX_1BUF_CAPABLE;
			adapter->flags |= IXGBE_FLAG_RX_PS_CAPABLE;
		}
#endif
	}
}
