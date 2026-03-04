# Fix: 10GBase-T SFP+ Module Support for Intel ixgbe (82599/X520/X540)

## The Problem

10GBase-T SFP+ modules (copper RJ45 transceivers) are rejected by the Intel ixgbe driver with:

```
ixgbe: SFP+ module not supported
```

This happens even with `allow_unsupported_sfp=1` set.

### Root Cause

The SFF-8472 standard defines compliance code bits in byte 0x03 (`comp_codes_10g`) only for fiber optic types:

| Bit | Type |
|-----|------|
| 0x10 | 10GBASE-SR |
| 0x20 | 10GBASE-LR |
| 0x40 | 10GBASE-LRM |
| 0x80 | 10GBASE-ER |

**10GBase-T has no bit defined.** Copper RJ45 SFP+ modules report `comp_codes_10g = 0x00`.

The ixgbe driver checks `comp_codes_10g` early in `ixgbe_identify_sfp_module_generic()` and rejects modules with `comp_codes_10g == 0` **before** the `allow_unsupported_sfp` flag is ever consulted. This makes the kernel parameter useless for 10GBase-T modules.

## The Fix

A 4-line patch (+ comments) in `src/ixgbe_phy.c` that detects 10GBase-T modules and forces SR classification when `allow_unsupported_sfp` is explicitly enabled:

```c
if (comp_codes_10g == 0 && cable_tech == 0 &&
    hw->allow_unsupported_sfp) {
    comp_codes_10g = IXGBE_SFF_10GBASESR_CAPABLE;
}
```

The conditions `comp_codes_10g == 0 && cable_tech == 0` specifically identify modules that are neither fiber nor DAC (direct attach copper), which matches 10GBase-T transceivers. The fix only activates when `allow_unsupported_sfp` is explicitly enabled by the administrator.

## Tested Hardware

| Component | Model |
|-----------|-------|
| Server | Dell PowerEdge R630 |
| NIC | Dell C63DV rNDC (Intel X520 / 82599 SFI, 2x SFP+) |
| SFP+ Module | FS.com SFP-10GM-T-30 (10GBase-T, Dell-coded) |
| OS | Proxmox Backup Server 3.3-1, kernel 6.17.2-1-pve |

**Result:** Link established at 10 Gbps Full Duplex, 0.13ms latency.

This fix should work with any 10GBase-T SFP+ module in any Intel 82599-based NIC (X520, X540 with SFP+ slots, etc.).

## Prerequisites

### 1. EEPROM Vendor Lock Bypass (Dell/HP/Lenovo servers only)

Brand-locked servers reject non-OEM SFP+ modules at the EEPROM level. The SFP+ module's EEPROM byte 0x58 (Enhanced Options) must include the ALLOW_ANY_SFP bit.

If your module is already vendor-coded for your server brand, skip this step.

To check and patch (requires `ethtool` or `i2cset`):
```bash
# Read current value (via a working NIC that can see the module)
ethtool -m <interface> offset 0x58 length 1

# Patch to 0xfd (sets ALLOW_ANY_SFP bit)
# WARNING: This modifies the SFP+ module's EEPROM permanently
i2cset -y <bus> 0x50 0x58 0xfd
```

### 2. Enable allow_unsupported_sfp

```bash
# Persistent via modprobe
echo 'options ixgbe allow_unsupported_sfp=1' > /etc/modprobe.d/ixgbe.conf

# Also in GRUB (belt and suspenders)
# Add to GRUB_CMDLINE_LINUX in /etc/default/grub:
#   ixgbe.allow_unsupported_sfp=1
# Then: update-grub
```

## Build & Install

### Debian/Ubuntu/Proxmox

```bash
# Install dependencies
apt update
apt install -y build-essential dkms proxmox-headers-$(uname -r) libdw-dev
# For non-Proxmox: apt install -y linux-headers-$(uname -r)

# Clone this repo
git clone https://github.com/FilleMang/ethernet-linux-ixgbe.git
cd ethernet-linux-ixgbe

# Build
cd src
make clean
make CFLAGS_EXTRA="-DDISABLE_PCI_AER" install

# Update initramfs so the patched driver loads at boot
update-initramfs -u

# Load the new driver
modprobe -r ixgbe && modprobe ixgbe allow_unsupported_sfp=1
```

### Apply Patch Only (if you already have Intel OOT source)

```bash
cd ethernet-linux-ixgbe
git apply patches/0001-ixgbe-add-10GBase-T-SFP-support.patch
```

Or using `patch`:
```bash
patch -p1 < patches/0001-ixgbe-add-10GBase-T-SFP-support.patch
```

## Verification

```bash
# Check driver version (should show 6.3.4)
modinfo ixgbe | grep version

# Check module is from updates/ (not kernel built-in)
modinfo ixgbe | grep filename
# Expected: /lib/modules/.../updates/drivers/net/ethernet/intel/ixgbe/ixgbe.ko

# Check link status
ethtool eno1
# Expected: Speed: 10000Mb/s, Link detected: yes

# Check driver loaded with allow_unsupported_sfp
dmesg | grep -i "allow_unsupported_sfp"
# Expected: ixgbe ...: allow_unsupported_sfp Enabled
```

## DKMS (Survive Kernel Upgrades)

To automatically rebuild the driver when the kernel is upgraded:

```bash
DRIVER_VERSION=6.3.4
cp -r /path/to/ethernet-linux-ixgbe /usr/src/ixgbe-${DRIVER_VERSION}

cat > /usr/src/ixgbe-${DRIVER_VERSION}/dkms.conf << EOF
PACKAGE_NAME="ixgbe"
PACKAGE_VERSION="${DRIVER_VERSION}"
BUILT_MODULE_NAME[0]="ixgbe"
BUILT_MODULE_LOCATION[0]="src/"
DEST_MODULE_LOCATION[0]="/updates/drivers/net/ethernet/intel/ixgbe/"
MAKE[0]="cd src && make CFLAGS_EXTRA='-DDISABLE_PCI_AER'"
CLEAN="cd src && make clean"
AUTOINSTALL="yes"
EOF

dkms add -m ixgbe -v ${DRIVER_VERSION}
dkms build -m ixgbe -v ${DRIVER_VERSION}
dkms install -m ixgbe -v ${DRIVER_VERSION}
```

## Background

This issue affects everyone using 10GBase-T (copper RJ45) SFP+ transceivers in Intel 82599-based NICs. Affected users include those with:

- FS.com SFP-10GM-T-30 / SFP-10GM-T-80
- 10Gtek 10GBase-T SFP+ modules
- Various other copper SFP+ transceivers

Related discussions:
- [vfreex/ixgbe-fix gist](https://gist.github.com/vfreex/) — partial fix for vendor lock only
- [DPDK Bug 263](https://bugs.dpdk.org/show_bug.cgi?id=263) — same root cause in DPDK
- Proxmox forum threads about unsupported SFP+ modules

## License

This patch is provided under the same license as the original Intel ixgbe driver (GPL v2).
