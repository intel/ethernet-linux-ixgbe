# DKMS Installation Guide for ixgbe Driver

## What is DKMS?

Dynamic Kernel Module Support (DKMS) is a framework that enables automatic rebuilding of kernel modules when the kernel is updated. This ensures the ixgbe driver continues to work after kernel upgrades without manual intervention.

## Prerequisites

1. DKMS must be installed on your system:
   - Debian/Ubuntu: `sudo apt-get install dkms`
   - RHEL/CentOS/Fedora: `sudo dnf install dkms` or `sudo yum install dkms`
   - openSUSE: `sudo zypper install dkms`

2. Kernel headers for your running kernel must be installed:
   - Debian/Ubuntu: `sudo apt-get install linux-headers-$(uname -r)`
   - RHEL/CentOS/Fedora: `sudo dnf install kernel-devel-$(uname -r)`
   - openSUSE: `sudo zypper install kernel-devel`

## Installation via DKMS

### Method 1: Using DKMS directly

1. Copy the source to /usr/src:
   ```bash
   sudo cp -r /path/to/ethernet-linux-ixgbe /usr/src/ixgbe-6.2.5
   ```

2. Add the module to DKMS:
   ```bash
   sudo dkms add -m ixgbe -v 6.2.5
   ```

3. Build the module:
   ```bash
   sudo dkms build -m ixgbe -v 6.2.5
   ```

4. Install the module:
   ```bash
   sudo dkms install -m ixgbe -v 6.2.5
   ```

### Method 2: One-step installation

If the source is already in `/usr/src/ixgbe-6.2.5/`:
```bash
sudo dkms install -m ixgbe -v 6.2.5
```

This will automatically add, build, and install the module.

## Verification

1. Check DKMS status:
   ```bash
   dkms status ixgbe
   ```

   You should see output like:
   ```
   ixgbe/6.2.5, 5.15.0-generic, x86_64: installed
   ```

2. Verify the module is loaded:
   ```bash
   modinfo ixgbe
   lsmod | grep ixgbe
   ```

3. Load the module if needed:
   ```bash
   sudo modprobe ixgbe
   ```

## Uninstallation

To remove the DKMS module:

1. Uninstall the module:
   ```bash
   sudo dkms uninstall -m ixgbe -v 6.2.5
   ```

2. Remove from DKMS:
   ```bash
   sudo dkms remove -m ixgbe -v 6.2.5 --all
   ```

3. Optionally, remove the source:
   ```bash
   sudo rm -rf /usr/src/ixgbe-6.2.5
   ```

## Automatic Rebuilds

Once installed via DKMS, the ixgbe module will automatically rebuild when:
- The kernel is updated
- New kernel versions are installed

You can manually trigger a rebuild for a specific kernel:
```bash
sudo dkms build -m ixgbe -v 6.2.5 -k <kernel-version>
sudo dkms install -m ixgbe -v 6.2.5 -k <kernel-version>
```

## Troubleshooting

### Build fails with "Kernel headers not found"

Ensure kernel headers are installed for the target kernel:
```bash
sudo apt-get install linux-headers-$(uname -r)  # Debian/Ubuntu
sudo dnf install kernel-devel-$(uname -r)       # RHEL/Fedora
```

### Module doesn't load after installation

1. Check dmesg for errors:
   ```bash
   sudo dmesg | grep ixgbe
   ```

2. Ensure the module is properly installed:
   ```bash
   sudo depmod -a
   sudo modprobe ixgbe
   ```

### Checking build logs

If build fails, check the DKMS build log:
```bash
cat /var/lib/dkms/ixgbe/6.2.5/build/make.log
```

## Additional Notes

- The DKMS configuration is stored in `dkms.conf` in the root of this package
- The module will be installed to `/lib/modules/<kernel>/updates/dkms/`
- DKMS modules take precedence over in-tree kernel modules
- After installation, you may need to update initramfs if the module is required at boot:
  ```bash
  sudo update-initramfs -u     # Debian/Ubuntu
  sudo dracut --force          # RHEL/Fedora
  ```
