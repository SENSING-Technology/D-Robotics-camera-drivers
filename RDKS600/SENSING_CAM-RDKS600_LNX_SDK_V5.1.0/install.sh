#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEB_DIR="$SCRIPT_DIR/deb"
OVERLAY_DIR="$SCRIPT_DIR/overlay"
CONFIG_FILE=/boot/config.txt
OVERLAY_BASE=s600_sensing_cam_rx0123_portABCD_yuv_generated
DTBO="$OVERLAY_DIR/${OVERLAY_BASE}.dtbo"
DTSO="$OVERLAY_DIR/${OVERLAY_BASE}.dtso"
DTBO_PATH="/overlays/${OVERLAY_BASE}.dtbo"
BACKUP_DIR=""
FORCE_DEB=0

usage()
{
	cat <<EOF
Usage: sudo $0 [--force-deb]

  --force-deb  Reinstall DEB packages even when the same versions are installed.
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }
on_error()
{
	local rc=$?
	echo "ERROR: installation failed at line ${BASH_LINENO[0]} (exit ${rc})." >&2
	[[ -z "$BACKUP_DIR" ]] || echo "Boot backup: $BACKUP_DIR" >&2
	exit "$rc"
}
trap on_error ERR

while (( $# )); do
	case $1 in
		--force-deb) FORCE_DEB=1 ;;
		-h|--help) usage; exit 0 ;;
		*) usage >&2; die "unknown option: $1" ;;
	esac
	shift
done

[[ $EUID -eq 0 ]] || die "run with sudo: sudo $0"
[[ $(dpkg --print-architecture) == arm64 ]] || die "this SDK requires arm64"
[[ -d "$DEB_DIR" ]] || die "missing directory: $DEB_DIR"
[[ -d "$OVERLAY_DIR" ]] || die "missing directory: $OVERLAY_DIR"
if [[ -e "$CONFIG_FILE" ]]; then
	[[ -f "$CONFIG_FILE" && -w "$CONFIG_FILE" ]] || die "$CONFIG_FILE is not a writable regular file"
else
	[[ -w /boot ]] || die "/boot is not writable; cannot create $CONFIG_FILE"
fi
[[ -s "$DTBO" ]] || die "missing generated DTBO: $DTBO"
[[ -s "$DTSO" ]] || die "missing generated DTSO: $DTSO"
command -v dpkg-deb >/dev/null || die "dpkg-deb is required"
command -v dtc >/dev/null || die "dtc is required"
dtc -I dtb -O dts -o /dev/null "$DTBO" >/dev/null 2>&1 || die "invalid DTBO: $DTBO"

one_deb()
{
	local pattern=$1 files=()
	shopt -s nullglob
	files=("$DEB_DIR"/$pattern)
	shopt -u nullglob
	(( ${#files[@]} == 1 )) || die "expected exactly one $pattern in $DEB_DIR; found ${#files[@]}"
	printf '%s' "${files[0]}"
}

IMAGE_DEB=$(one_deb 'linux-image-rdk-s600_*.deb')
HEADERS_DEB=$(one_deb 'linux-headers-*.deb')
CAMERA_DEB=$(one_deb 'hobot-camera_*.deb')
SPDEV_DEB=$(one_deb 'hobot-spdev_*.deb')
MULTIMEDIA_SAMPLES_DEB=$(one_deb 'hobot-multimedia-samples_*.deb')

for deb in "$IMAGE_DEB" "$HEADERS_DEB" "$CAMERA_DEB" "$SPDEV_DEB" "$MULTIMEDIA_SAMPLES_DEB"; do
	[[ $(dpkg-deb -f "$deb" Architecture) == arm64 ]] || die "non-arm64 package: $deb"
	dpkg-deb --info "$deb" >/dev/null || die "invalid DEB: $deb"
done

deb_needs_install()
{
	local deb=$1 package version installed
	package=$(dpkg-deb -f "$deb" Package)
	version=$(dpkg-deb -f "$deb" Version)
	(( FORCE_DEB )) && return 0
	installed=$(dpkg-query -W -f='${Status}\t${Version}' "$package" 2>/dev/null || true)
	[[ "$installed" != $'install ok installed\t'"$version" ]]
}

DEBS_TO_INSTALL=()
for deb in "$IMAGE_DEB" "$HEADERS_DEB" "$CAMERA_DEB" "$SPDEV_DEB" "$MULTIMEDIA_SAMPLES_DEB"; do
	if deb_needs_install "$deb"; then
		DEBS_TO_INSTALL+=("$deb")
	else
		echo "DEB already installed: $(dpkg-deb -f "$deb" Package) $(dpkg-deb -f "$deb" Version)"
	fi
done

IMAGE_VERSION=$(dpkg-deb -f "$IMAGE_DEB" Version)
HEADERS_VERSION=$(dpkg-deb -f "$HEADERS_DEB" Version)
[[ "$IMAGE_VERSION" == "$HEADERS_VERSION" ]] || die "kernel image/headers version mismatch"

mapfile -t releases < <(dpkg-deb -c "$IMAGE_DEB" | sed -n 's#.*\./lib/modules/\([^/]*\)/.*#\1#p' | sort -u)
(( ${#releases[@]} == 1 )) || die "cannot determine one kernel release from image package"
TARGET_RELEASE=${releases[0]}

if (( ${#DEBS_TO_INSTALL[@]} )); then
	avail_kb=$(df -Pk /boot | awk 'NR==2 {print $4}')
	(( avail_kb >= 30000 )) || die "/boot has less than 30 MiB free"
fi

stamp=$(date +%Y%m%d-%H%M%S)
BACKUP_DIR="/boot/sensing-camera-backup-$stamp"
mkdir -m 0700 "$BACKUP_DIR"
if [[ -f "$CONFIG_FILE" ]]; then
	cp -a "$CONFIG_FILE" "$BACKUP_DIR/config.txt"
else
	printf 'config.txt did not exist before installation\n' > "$BACKUP_DIR/config.txt.absent"
fi

current_path=""
if [[ -f "$CONFIG_FILE" ]]; then
	current_path=$(sed -n 's/^[[:space:]]*dtbo_file_path=//p' "$CONFIG_FILE" | tail -n 1)
fi
if [[ "$current_path" == /overlays/* && -f "/boot$current_path" ]]; then
	cp -a "/boot$current_path" "$BACKUP_DIR/"
fi
for file in "/boot/overlays/${OVERLAY_BASE}.dtbo" "/boot/overlays/${OVERLAY_BASE}.dtso"; do
	[[ ! -f "$file" ]] || cp -a "$file" "$BACKUP_DIR/"
done

if (( ${#DEBS_TO_INSTALL[@]} )); then
	echo "Installing ${#DEBS_TO_INSTALL[@]} changed DEB package(s)..."
	for deb in "${DEBS_TO_INSTALL[@]}"; do
		dpkg -i "$deb"
	done
else
	echo "DEB packages unchanged; skipping dpkg installation."
fi

install -m 0644 "$DTBO" "/boot/overlays/${OVERLAY_BASE}.dtbo"
install -m 0644 "$DTSO" "/boot/overlays/${OVERLAY_BASE}.dtso"

tmp_config=$(mktemp /boot/config.txt.sensing.XXXXXX)
if [[ -f "$CONFIG_FILE" ]] && grep -q '^[[:space:]]*dtbo_file_path=' "$CONFIG_FILE"; then
	sed "s|^[[:space:]]*dtbo_file_path=.*|dtbo_file_path=$DTBO_PATH|" "$CONFIG_FILE" > "$tmp_config"
else
	if [[ -f "$CONFIG_FILE" ]]; then cat "$CONFIG_FILE" > "$tmp_config"; fi
	printf '\ndtbo_file_path=%s\n' "$DTBO_PATH" >> "$tmp_config"
fi
if [[ -f "$CONFIG_FILE" ]]; then
	chmod --reference="$CONFIG_FILE" "$tmp_config"
	chown --reference="$CONFIG_FILE" "$tmp_config"
else
	chmod 0644 "$tmp_config"
fi
mv -f "$tmp_config" "$CONFIG_FILE"

[[ -s "/boot/overlays/${OVERLAY_BASE}.dtbo" ]] || die "installed DTBO is missing"
grep -qx "dtbo_file_path=$DTBO_PATH" "$CONFIG_FILE" || die "boot configuration verification failed"
[[ -d "/lib/modules/$TARGET_RELEASE" ]] || die "installed module directory is missing"
dpkg-query -W -f='${Status}' linux-image-rdk-s600 2>/dev/null | grep -q 'install ok installed' || die "kernel package is not installed"

echo "OK: installation complete"
echo "Backup: $BACKUP_DIR"
echo "Target kernel: $TARGET_RELEASE"
echo "REBOOT REQUIRED: sudo reboot"
echo "After reboot verify: uname -r"
