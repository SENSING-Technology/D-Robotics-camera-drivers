#!/usr/bin/env bash

set -Eeuo pipefail

die() { echo "ERROR: $*" >&2; exit 1; }
loaded() { [[ -d "/sys/module/${1//-/_}" ]]; }
unload() { if loaded "$1"; then modprobe -r "$1"; fi; }

mode=auto-raw-isp
case ${1:-} in
	"") ;;
	--direct) mode=direct ;;
	--rx3-s56-isp) mode=rx3-s56-isp ;;
	--rx2-s56-isp-rx3-direct) mode=rx2-s56-isp-rx3-direct ;;
	--auto-raw-isp) mode=auto-raw-isp ;;
	-h|--help)
		echo "Usage: sudo $0 [--direct|--rx3-s56-isp|--rx2-s56-isp-rx3-direct|--auto-raw-isp]"
		echo "  default          DT-based routing: S56 RAW -> ISP, known YUV -> CIM direct"
		echo "  --direct         RX0..RX3 CIM-to-DDR capture nodes (scene 0)"
		echo "  --rx3-s56-isp   RX3 RAW10 -> ISP3/YNR3/PYM3 -> NV12 (scene 9)"
		echo "  --rx2-s56-isp-rx3-direct  RX2 RAW10 -> ISP2; RX3 CIM direct (mixed scene_table)"
		echo "  --auto-raw-isp  Use DT camera compatible: S56 RAW -> ISP, known YUV -> CIM direct"
		exit 0
		;;
	*) die "unknown option: $1" ;;
esac
(( $# <= 1 )) || die "too many arguments"

build_rx2_isp_scene_table()
{
	local table='' rx vc slot
	# Keep the scene 0 video ordering: CIM4, CIM5, RX0..RX3 VC0..VC3.
	for rx in 4 5; do
		table+="{cim${rx}-0,1,video,0,1}"
	done
	for rx in 0 1 2 3; do
		for vc in 0 1 2 3; do
			if (( rx == 2 )); then
				slot=$((4 + vc))
				table+="{cim2-${vc},0,isp2-${slot},0,1}"
				table+="{isp2-${slot},0,ynr2-${slot},0,1}"
				table+="{ynr2-${slot},0,pym2-${vc},0,1}"
				table+="{pym2-${vc},0,video,0,1}"
			else
				table+="{cim${rx}-${vc},1,video,0,1}"
			fi
		done
	done
	printf '%s' "$table"
}

build_auto_scene_table()
{
	local table='' rx vc video slot pym hw
	for video in 0 1; do
		# CIM4/5 are outside the RX0..RX3 camera-client mapping.
		table+="{cim$((video + 4))-0,1,video,0,1}"
	done
	for rx in 0 1 2 3; do
		for vc in 0 1 2 3; do
			video=$((2 + rx * 4 + vc))
			if [[ ${route_isp[$video]:-0} == 1 ]]; then
				hw=$rx; slot=$((4 + vc)); pym=$vc
				if (( rx == 3 )); then slot=$((6 + vc)); pym=$((2 + vc)); fi
				table+="{cim${rx}-${vc},0,isp${hw}-${slot},0,1}"
				table+="{isp${hw}-${slot},0,ynr${hw}-${slot},0,1}"
				table+="{ynr${hw}-${slot},0,pym${hw}-${pym},0,1}"
				table+="{pym${hw}-${pym},0,video,0,1}"
			else
				table+="{cim${rx}-${vc},1,video,0,1}"
			fi
		done
	done
	printf '%s' "$table"
}

prepare_s56_isp()
{
	local sensor_dir=/usr/hobot/lib/sensor
	local tuning_lib=$sensor_dir/lib_VB1940_linear.so
	local service_lib=$sensor_dir/lib_sensing-stereocam_linear.so
	local param
	[[ -f "$tuning_lib" ]] || die "missing ISP tuning library: $tuning_lib"
	if [[ -L "$service_lib" ]]; then
		[[ $(readlink "$service_lib") == lib_VB1940_linear.so ]] ||
			die "$service_lib points to a different tuning library"
	elif [[ -e "$service_lib" ]]; then
		die "$service_lib already exists as a regular file; refusing to replace it"
	else
		ln -s lib_VB1940_linear.so "$service_lib"
	fi
	for param in nr2d_en nr3d_en; do
		[[ -w /sys/module/hobot_ynr_v4l2/parameters/$param ]] || die "$param is unavailable"
		printf '0\n' > "/sys/module/hobot_ynr_v4l2/parameters/$param"
	done
	command -v isp_service >/dev/null || die "isp_service is required for RAW ISP processing"
}

dt_u32()
{
	local file=$1 hex
	[[ -r "$file" ]] || return 1
	hex=$(od -An -N4 -tx1 "$file" | tr -d ' \n')
	[[ ${#hex} -eq 8 ]] || return 1
	printf '%u' "$((16#$hex))"
}

dt_u32_at()
{
	local file=$1 index=$2 hex value offset
	[[ -r "$file" ]] || return 1
	hex=$(od -An -v -tx1 "$file" | tr -d ' \n')
	offset=$((index * 8))
	value=${hex:offset:8}
	[[ ${#value} -eq 8 ]] || return 1
	printf '%u' "$((16#$value))"
}

print_camera_nodes()
{
	local dev bus address vc video port phy model width height node name index
	local printed=0

	echo "Camera capture nodes:"
	for dev in /sys/bus/i2c/devices/*-*; do
		[[ -r "$dev/name" && -L "$dev/driver" ]] || continue
		name=$(<"$dev/name")
		case "$name" in
			sensing_cam|sensing-cam|s36-gmsl2|s36_gmsl2|s56-gmsl2|s56_gmsl2|sensing-stereocam|sensing_stereocam) ;;
			*) continue ;;
		esac

		bus=${dev##*/}; bus=${bus%%-*}
		address=${dev##*-}; address=$((16#$address))
		if (( address >= 0x11 && address <= 0x14 )); then
			vc=$((address - 0x11))
		elif (( address >= 0x22 && address <= 0x25 )); then
			vc=$((address - 0x22))
		else
			continue
		fi
		(( bus >= 0 && bus <= 3 && vc >= 0 && vc <= 3 )) || continue

		video=$((2 + bus * 4 + vc))
		port=${port:-}; port=$(printf "\\$(printf '%03o' $((65 + vc)))")
		if (( bus % 2 == 0 )); then phy="D-PHY"; else phy="C-PHY"; fi
		node=$(readlink -f "$dev/of_node" 2>/dev/null || true)
		model=$name
		if [[ -r "$node/sensing,model" ]]; then
			model=$(tr -d '\0' < "$node/sensing,model")
		fi
		width=$(dt_u32 "$node/width" 2>/dev/null || echo '?')
		height=$(dt_u32 "$node/height" 2>/dev/null || echo '?')
		case "$name" in
			s36-gmsl2|s36_gmsl2|s56-gmsl2|s56_gmsl2|sensing-stereocam|sensing_stereocam)
				port_index=$(dt_u32 "$node/link-port" 2>/dev/null || echo "$vc")
				port=$(printf "\\$(printf '%03o' $((65 + port_index)))")
				for index in 0 1; do
					vc=$(dt_u32_at "$node/sensing,stream-dst-vcs" "$index" 2>/dev/null) || continue
					video=$((2 + bus * 4 + vc))
					printf '  RX%d Port %s VC%d (%s, %s, %sx%s) -> /dev/video%d\n' \
						"$bus" "$port" "$vc" "$phy" "$model" "$width" "$height" "$video"
					printed=$((printed + 1))
				done
				continue
				;;
		esac
		printf '  RX%d Port %s VC%d (%s, %s, %sx%s) -> /dev/video%d\n' \
			"$bus" "$port" "$vc" "$phy" "$model" "$width" "$height" "$video"
		printed=$((printed + 1))
	done
	(( printed > 0 )) || echo "  (no bound camera clients)"
}

route_active_cameras_from_dt()
{
	local dev name bus address node vc video compatible route index
	local -a vcs=()
	route_isp=()
	for dev in /sys/bus/i2c/devices/*-*; do
		[[ -r "$dev/name" && -L "$dev/driver" ]] || continue
		name=$(<"$dev/name")
		case "$name" in
			sensing_cam|sensing-cam|s36-gmsl2|s36_gmsl2|s56-gmsl2|s56_gmsl2|sensing-stereocam|sensing_stereocam|ar0820) ;;
			*) continue ;;
		esac
		bus=${dev##*/}; bus=${bus%%-*}
		[[ $bus =~ ^[0-3]$ ]] || continue
		address=${dev##*-}; address=$((16#$address))
		node=$(readlink -f "$dev/of_node" 2>/dev/null || true)
		[[ -r "$node/compatible" ]] || die "missing DT compatible for $dev"
		compatible=$(tr '\0' '\n' < "$node/compatible" | head -n 1)
		case "$compatible" in
			sensing,s56-gmsl2) route=1 ;;
			sensing,s36-gmsl2|sensing,sensing-cam) route=0 ;;
			*) die "unknown camera compatible '$compatible' for $dev; add an explicit RAW/YUV classification" ;;
		esac
		vcs=()
		if [[ -r "$node/sensing,stream-dst-vcs" ]]; then
			for index in 0 1; do
				vc=$(dt_u32_at "$node/sensing,stream-dst-vcs" "$index") || die "invalid VC list for $dev"
				vcs+=("$vc")
			done
		elif (( address >= 0x11 && address <= 0x14 )); then
			vcs+=("$((address - 0x11))")
		elif (( address >= 0x22 && address <= 0x25 )); then
			vcs+=("$((address - 0x22))")
		else
			die "cannot determine VC for $dev ($name)"
		fi
		for vc in "${vcs[@]}"; do
			(( vc >= 0 && vc <= 3 )) || die "invalid VC$vc for $dev"
			video=$((2 + bus * 4 + vc))
			route_isp[$video]=$route
			if (( route )); then
				echo "RX$bus VC$vc $compatible -> ISP (video$video)"
			else
				echo "RX$bus VC$vc $compatible -> CIM direct (video$video)"
			fi
		done
	done
}

on_error()
{
	local rc=$?
	echo "ERROR: camera load failed at line ${BASH_LINENO[0]} (exit ${rc})" >&2
	dmesg | grep -Ei 'sensing|s36|s56|max9295|max96712|v4l|mipi|cim|error|fail|timeout' |
		tail -n 40 >&2 || true
	exit "$rc"
}
trap on_error ERR

[[ $EUID -eq 0 ]] || die "run with sudo: sudo $0"

# The vendor SerDes/CSI remove paths are unsafe on this RT kernel.  This
# loader is for a cold boot only; never tear down an existing media graph.
for module in vid_v4l2 v4l_mipicsi v4l2_cim hobot_isp_v4l2; do
	loaded "$module" && die "$module is already loaded; reboot before changing camera mode"
done

# Stop userspace and make sure no process owns a camera node.
pkill -TERM -f '(^|/)isp_service([[:space:]]|$)' 2>/dev/null || true
for _ in {1..20}; do
	pgrep -f '(^|/)isp_service([[:space:]]|$)' >/dev/null || break
	sleep 0.1
done
pkill -KILL -f '(^|/)isp_service([[:space:]]|$)' 2>/dev/null || true
if fuser /dev/video* /dev/media* /dev/v4l-subdev* >/dev/null 2>&1; then
	fuser -v /dev/video* /dev/media* /dev/v4l-subdev* 2>/dev/null >&2 || true
	die "camera devices are busy"
fi

# The stock image auto-loads the HBN processing graph. Its drivers own the
# same hardware nodes needed by V4L2, so release the idle HBN consumers first.
# Keep SerDes and sensor provider modules resident.
for module in hobot_gdc hobot_ynr hobot_pym_jplus hobot_isp \
	              hobot_cim hobot_mipidbg hobot_mipicsi; do
	if loaded "$module"; then
		modprobe -r "$module" || die "$module is busy; stop its users and reboot"
	fi
done

# Load providers in order.  If a DT-autoloaded sensor failed to bind, the
# probe checks below stop rather than hot-unloading max96712.
modprobe max9295a
modprobe max96712
modprobe sensing_stereocam

echo "Waiting for S56 stereo probes..."
for _ in {1..90}; do
	stereo_found=0
	stereo_unbound=0
	for dev in /sys/bus/i2c/devices/*-*; do
		[[ -r "$dev/name" ]] || continue
		name=$(<"$dev/name")
		case "$name" in
			s56-gmsl2|s56_gmsl2|s36-gmsl2|s36_gmsl2|sensing-stereocam|sensing_stereocam)
				stereo_found=$((stereo_found + 1))
				# The driver symlink can appear before the probe completes.
				# Wait for the sensor driver's final registration message so
				# a mono camera cannot reconfigure the shared MAX96712 mid-probe.
				if [[ ! -L "$dev/driver" ]] || ! dmesg | grep -F "sensing-stereocam ${dev##*/}:" | grep 'dual-stream registered on link' >/dev/null; then
					stereo_unbound=$((stereo_unbound + 1))
				fi
				;;
		esac
	done
	(( stereo_found == 0 || stereo_unbound == 0 )) && break
	sleep 1
done
(( stereo_unbound == 0 )) || die "S56 stereo probe timeout: $((stereo_found - stereo_unbound))/${stereo_found} clients bound"

modprobe sensing_cam

# sensing_cam may have auto-probed before MAX9295/MAX96712 registered in
# serdes_core. That early probe fails with -ENOSYS and modprobe does not retry.
# Once the SerDes providers are bound, explicitly bind only failed mono clients.
for _ in {1..90}; do
	serdes_unbound=0
	for dev in /sys/bus/i2c/devices/*-*; do
		[[ -r "$dev/name" ]] || continue
		name=$(<"$dev/name")
		case "$name" in
			max9295|max9295a|max96712)
				[[ -L "$dev/driver" ]] || serdes_unbound=$((serdes_unbound + 1)) ;;
		esac
	done
	(( serdes_unbound == 0 )) && break
	sleep 1
done
(( serdes_unbound == 0 )) || die "$serdes_unbound SerDes clients remain unbound"
for dev in /sys/bus/i2c/devices/*-*; do
	[[ -r "$dev/name" && ! -L "$dev/driver" ]] || continue
	name=$(<"$dev/name")
	case "$name" in sensing_cam|sensing-cam) ;;
		*) continue ;;
	esac
	echo "Retrying $name probe for ${dev##*/} after SerDes initialization..."
	printf '%s\n' "${dev##*/}" > /sys/bus/i2c/drivers/sensing_cam/bind
done

# S56 initialization is serialized and four stereo pairs can take about one
# minute. Do not construct the V4L2 graph until every DT camera client binds.
echo "Waiting for camera probes..."
for _ in {1..90}; do
	found=0
	bound=0
	unbound=0
	for dev in /sys/bus/i2c/devices/*-*; do
		[[ -r "$dev/name" ]] || continue
		name=$(<"$dev/name")
		case "$name" in
			ar0820|sensing_cam|sensing-cam|s36-gmsl2|s36_gmsl2|s56-gmsl2|s56_gmsl2|sensing-stereocam|sensing_stereocam)
				found=$((found + 1))
				[[ -L "$dev/driver" ]] && bound=$((bound + 1)) || unbound=$((unbound + 1))
				;;
		esac
	done
	(( found > 0 && unbound == 0 )) && break
	sleep 1
done
(( found > 0 )) || die "no supported camera clients found; check the active DTBO"
(( unbound == 0 )) || die "camera probe timeout: ${bound}/${found} clients bound"

# Build the direct, RX3 ISP, or mixed RX2 ISP/RX3 direct graph.
modprobe hobot_camsys_adapter
[[ -w /sys/module/hobot_camsys_adapter/parameters/mops ]] || die "camsys mops is unavailable"
printf '%s' ion > /sys/module/hobot_camsys_adapter/parameters/mops

for module in videobuf2-common videobuf2-v4l2 videobuf2-memops \
	              videobuf2-dma-contig v4l2-mem2mem v4l_mipicsi v4l2_cim \
	              hobot_isp_v4l2 pym_v4l_drv gdc_v4l_drv hobot_ynr_v4l2; do
	modprobe "$module"
done

if [[ $mode == rx3-s56-isp ]]; then
	# S600 built-in scene 9 is the supported SerDes graph:
	# CIM3 DDR -> ISP3 -> YNR3 -> PYM3 -> video14..17.
	# RX3 VC0/VC1 therefore remain video14/video15 after ISP processing.
	modprobe vid_v4l2 scene=9
	prepare_s56_isp
	nohup isp_service >/tmp/isp_service.log 2>&1 &
elif [[ $mode == rx2-s56-isp-rx3-direct ]]; then
	# RX2 VC0..3 uses the scene 9 ISP/YNR/PYM path; RX3 and other RXs
	# retain scene 0's direct CIM capture. Video node numbering stays stable.
	modprobe vid_v4l2 scene_table="$(build_rx2_isp_scene_table)"
	prepare_s56_isp
	nohup isp_service >/tmp/isp_service.log 2>&1 &
elif [[ $mode == auto-raw-isp ]]; then
	# This package's DT generator identifies S56 RAW and known YUV cameras
	# by compatible; there is no need to tear down a temporary scene 0 graph.
	route_active_cameras_from_dt
	modprobe vid_v4l2 scene_table="$(build_auto_scene_table)"
	prepare_s56_isp
	nohup isp_service >/tmp/isp_service.log 2>&1 &
else
	modprobe vid_v4l2 scene=0
fi

for _ in {1..900}; do
	video_nodes=(/dev/video*)
	[[ -e /dev/media0 && -e "${video_nodes[0]}" ]] && break
	sleep 0.1
done
video_nodes=(/dev/video*)
[[ -e /dev/media0 && -e "${video_nodes[0]}" ]] || die "V4L2 nodes were not created within 90 seconds"

echo "OK: cameras ${bound}/${found} bound; /dev/media0; ${#video_nodes[@]} video nodes"
if [[ $mode == rx3-s56-isp ]]; then
	echo "RX3 S56 ISP capture nodes:"
	echo "  RX3 Port A VC0 RAW10 -> ISP3/YNR3/PYM3 -> /dev/video14 (expected NV12)"
	echo "  RX3 Port A VC1 RAW10 -> ISP3/YNR3/PYM3 -> /dev/video15 (expected NV12)"
	echo "  ISP service log: /tmp/isp_service.log"
elif [[ $mode == rx2-s56-isp-rx3-direct ]]; then
	echo "RX2 VC0..3 -> ISP2/YNR2/PYM2 -> /dev/video10..13 (expected NV12)"
	echo "RX3 VC0..3 -> CIM3 DDR -> /dev/video14..17 (direct sensor format)"
	echo "Set the capture format explicitly before streaming; ISP service log: /tmp/isp_service.log"
elif [[ $mode == auto-raw-isp ]]; then
	echo "Automatic RAW->ISP / YUV->CIM routing active; /dev/video2..17 retain RX/VC numbering"
	echo "Set the capture format explicitly before streaming; ISP service log: /tmp/isp_service.log"
else
	print_camera_nodes
fi
