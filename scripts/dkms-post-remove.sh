#!/bin/bash
# DKMS POST_REMOVE hook: remove DDP firmware installed by POST_INSTALL.
# Called by DKMS after 'dkms remove', before the source tree is cleaned up.
# Derive paths from $0 because DKMS does not export source_tree/name/version
# to the POST_REMOVE environment.  Mirrors what 'make modules_uninstall' does.
set -e
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}

src_dir="$(realpath "$(dirname "$0")/..")"
pkg_name="$(basename "${src_dir}")"
driver="${pkg_name%%-[0-9]*}"
kver="${kernelver:-$(uname -r)}"

# DKMS runs POST_REMOVE before its own depmod; update modules.dep first.
depmod -a "${kver}" 2>/dev/null || true
if command -v update-initramfs >/dev/null 2>&1; then
    _itramfs_modules=/etc/initramfs-tools/modules
    [ -f "${_itramfs_modules}" ] && \
        sed -i "/^${driver}$/d" "${_itramfs_modules}" 2>/dev/null || true
    update-initramfs -u -k "${kver}" 2>/dev/null || true
elif command -v dracut >/dev/null 2>&1; then
    dracut --force --kver "${kver}" 2>/dev/null || true
fi

ddp_dst="/lib/firmware/updates/intel/${driver}/ddp"
link="${ddp_dst}/${driver}.pkg"

[ -d "${ddp_dst}" ] || exit 0

if [ -L "${link}" ]; then
    pkg_file="${ddp_dst}/$(readlink "${link}")"
    rm -f "${link}"
    rm -f "${pkg_file}"
fi

rm -f "${ddp_dst}/LICENSE"
