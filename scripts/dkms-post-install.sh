#!/bin/bash
# DKMS POST_INSTALL hook.
# Called by DKMS after 'dkms install'.  Derive paths from $0 because DKMS does
# not export source_tree/name/version to the POST_INSTALL environment.
set -e
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}

src_dir="$(realpath "$(dirname "$0")/..")"
pkg_name="$(basename "${src_dir}")"
driver="${pkg_name%%-[0-9]*}"
kver="${kernelver:-$(uname -r)}"

# Remove stale non-DKMS copies that would shadow updates/dkms/ in modules.dep.
find "/lib/modules/${kver}/updates" \
     -path "*/updates/dkms/*" -prune \
     -o -name "${driver}.ko" -print \
     -o -name "${driver}.ko.gz" -print \
     -o -name "${driver}.ko.zst" -print \
     -o -name "${driver}.ko.xz" -print \
  | grep -v "updates/dkms" \
  | xargs -r rm -f
depmod -a "${kver}" 2>/dev/null || true
if command -v update-initramfs >/dev/null 2>&1; then
    # MODULES=most walks kernel/ by path; explicit listing forces modules.dep lookup.
    _itramfs_modules=/etc/initramfs-tools/modules
    if [ -f "${_itramfs_modules}" ] && \
       ! grep -qx "${driver}" "${_itramfs_modules}" 2>/dev/null; then
        echo "${driver}" >> "${_itramfs_modules}"
    fi
    update-initramfs -u -k "${kver}" 2>/dev/null || true
elif command -v dracut >/dev/null 2>&1; then
    dracut --force --kver "${kver}" 2>/dev/null || true
fi

ddp_src="${src_dir}/ddp"
ddp_dst="/lib/firmware/updates/intel/${driver}/ddp"

[ -d "${ddp_src}" ] || exit 0

mkdir -p "${ddp_dst}"

for pkg in "${ddp_src}"/*.pkg; do
    install -m 644 "${pkg}" "${ddp_dst}/"
    ln -sf "${pkg##*/}" "${ddp_dst}/${driver}.pkg"
done

install -m 644 "${ddp_src}/LICENSE" "${ddp_dst}/"
