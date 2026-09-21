# SENSING CAM RDK S600 Linux SDK V5.1.0

This SDK generates, installs, and loads SENSING GMSL2 camera configurations on the D-Robotics RDK S600 platform.
This driver package currently only supports the v4l2 framework.

## Package Layout

```text
.
├── Readme.md
├── generate_camera_overlay.py
├── install.sh
├── load.sh
├── deb/
│   ├── hobot-camera_*.deb
│   ├── hobot-multimedia-samples_*.deb
│   ├── hobot-spdev_*.deb
│   ├── linux-headers-*.deb
│   └── linux-image-rdk-s600_*.deb
└── overlay/
    ├── s600_sensing_cam_rx0123_portABCD_yuv.dtso
    ├── s600_sensing_cam_rx0123_portABCD_yuv_generated.dtso
    └── s600_sensing_cam_rx0123_portABCD_yuv_generated.dtbo
```

## V4L2

### Supported Cameras

| Model                      | Type          |    Resolution | GMSL Link Rate |
| -------------------------- | ------------- | ------------: | -------------: |
| SN2M4EFGD                  | Monocular YUV |     1920×1080 |             3G |
| SG3S11AFSK                 | Monocular YUV |     1920×1536 |             3G |
| SG8-OX08DC-5300-G2G-Hxxx   | Monocular YUV |     3840×2160 |             6G |
| SG8S-AR0820C-5300-G2G-Hxxx | Monocular YUV |     3840×2160 |             6G |
| S36                        | Stereo        | 1920×1536 × 2 |             6G |
| S56                        | Stereo        | 2560×1984 × 2 |             6G |

A stereo camera may start only on Link A or Link C. A stereo camera on Link A consumes Links A+B and VC0/VC1. A stereo camera on Link C consumes Links C+D and VC2/VC3. Each RX supports up to two stereo camera pairs.

### Connector Modes

J12 and J14 use the same camera-expansion-board topology. The first RX on
each connector is a four-lane D-PHY interface; the second RX is a three-trio
C-PHY interface.

| Argument | Enabled RX             | First interface     | Second interface    |
| -------- | ---------------------- | ------------------- | ------------------- |
| `j12`    | RX0 and RX1            | RX0: D-PHY, 4 lanes | RX1: C-PHY, 3 trios |
| `j14`    | RX2 and RX3            | RX2: D-PHY, 4 lanes | RX3: C-PHY, 3 trios |
| `all`    | RX0, RX1, RX2, and RX3 | RX0/RX2: D-PHY      | RX1/RX3: C-PHY      |

### MIPI PHY Rate and Bandwidth Configuration

The current overlay uses the following matched MAX96712 and S600 MIPI RX
settings:

| RX  | PHY            | MAX96712 `phy_speed` | S600 `lane-rate` | Per-lane/trio rate | Equivalent bandwidth per RX |
| --- | -------------- | -------------------: | ---------------: | -----------------: | --------------------------: |
| RX0 | D-PHY, 4 lanes |                 2500 |            10000 |      2.5 Gbps/lane |                     10 Gbps |
| RX1 | C-PHY, 3 trios |                 3648 |             4800 |      1.6 Gsps/trio |                 10.944 Gbps |
| RX2 | D-PHY, 4 lanes |                 2500 |            10000 |      2.5 Gbps/lane |                     10 Gbps |
| RX3 | C-PHY, 3 trios |                 3648 |             4800 |      1.6 Gsps/trio |                 10.944 Gbps |

For D-PHY, `lane-rate` is the total bit rate of all data lanes:

```text
2.5 Gbps/lane × 4 lanes = 10 Gbps per RX
```

For C-PHY, `lane-rate` is the total symbol rate of all trios. C-PHY carries
approximately 2.28 bits per symbol:

```text
1.6 Gsps/trio × 3 trios = 4.8 Gsps
4.8 Gsps × 2.28 bits/symbol = 10.944 Gbps equivalent bandwidth per RX
```

The theoretical aggregate physical-layer bandwidth of RX0 through RX3 is:

```text
(2 × 10 Gbps) + (2 × 10.944 Gbps) = 41.888 Gbps
```

Protocol headers, CRC, blanking, and other CSI-2 overhead reduce the usable
video payload bandwidth.

For C-PHY, the MAX96712 driver converts `phy_speed` to the predefined DPLL
frequency code using `phy_speed / 228`. Therefore `phy_speed = <3648>` selects
DPLL code `0x10`, corresponding to 1600 Msps/trio. Because the driver updates
only register bits `[4:0]` and preserves bit 5 as `1`, the complete MAX96712
rate-register value is `0x30`:

```text
3648 / 228 = 16 = 0x10
bit 5 = 1, bits [4:0] = 0x10 -> complete register value = 0x30
```

Do not configure C-PHY as `phy_speed = <1600>` with `lane-rate = <4800>`.
With the current driver conversion, `1600 / 228` selects DPLL code `0x07`,
which does not match the S600 receiver setting of 1600 Msps/trio.

### 1. Generate the Device Tree Overlay

```bash
cd /home/sunrise/fxh/SENSING_CAM-RDKS600_LNX_SDK_V5.1.0
./generate_camera_overlay.py j12 --fps 25
```

The generator asks about each physical Port on the selected RXs. The number
shown next to a camera is the number to type at `Select ...:`; `0` disables
that Port. A stereo S36/S56 on Port A reserves Port B and VC0/VC1; one on
Port C reserves Port D and VC2/VC3. The available numbers can change when
fewer than two VCs remain, so use the menu currently printed on screen.

For example, to configure **J12** with S56 on RX0 Port A, an
SG8-OX08DC-5300-G2G-Hxxx YUV camera on RX0 Port C, and all other Ports
disabled, type `6` for RX0 Port A, `3` for RX0 Port C, then `0` for RX0
Port D and each of RX1 Ports A-D. Port B is skipped automatically:

```text
Mode: j12 (enabled RX: RX0, RX1)
S36/S56 external trigger: 25 Hz (period 40000000 ns, duty 10000 ns)

RX0 Port A:
  0: DISABLED (unused physical link)
  1: SN2M4EFGD (mono YUV, one VC)
  2: SG3S11AFSK (mono YUV, one VC)
  3: SG8-OX08DC-5300-G2G-Hxxx (mono YUV, one VC)
  4: SG8S-AR0820C-5300-G2G-Hxxx (mono YUV, one VC)
  5: S36 (stereo, one physical link and two VCs)
  6: S56 (stereo, one physical link and two VCs)
Select 0-6: 6
  S56 uses RX0 Port A+B, reserves output VC0+VC1, and skips Port B.

RX0 Port C:
  0: DISABLED (unused physical link)
  1: SN2M4EFGD (mono YUV, one VC)
  2: SG3S11AFSK (mono YUV, one VC)
  3: SG8-OX08DC-5300-G2G-Hxxx (mono YUV, one VC)
  4: SG8S-AR0820C-5300-G2G-Hxxx (mono YUV, one VC)
  5: S36 (stereo, one physical link and two VCs)
  6: S56 (stereo, one physical link and two VCs)
Select 0-6: 3
  Fixed configuration: 3840x2160, 6G

RX0 Port D: Select 0-4: 0
RX1 Port A: Select 0-6: 0
RX1 Port B: Select 0-4: 0
RX1 Port C: Select 0-6: 0
RX1 Port D: Select 0-4: 0

Final configuration:
  RX0 Port A VC0+VC1: S56 2560x1984
  RX0 Port B: reserved by previous stereo camera
  RX0 Port C VC2: SG8-OX08DC-5300-G2G-Hxxx 3840x2160
  RX0 Port D: disabled
  RX1 Port A: disabled
  RX1 Port B: disabled
  RX1 Port C: disabled
  RX1 Port D: disabled
  RX2: disabled (J14 not selected)
  RX3: disabled (J14 not selected)
```

This is an example of the actual menu and final summary; the repeated
disabled-Port menus are shortened above. Choose the cameras that match your
physical wiring. `j12` configures RX0/RX1; for RX2/RX3 use `j14` below.

The external trigger supports 20, 25, and 30 Hz:

```bash
./generate_camera_overlay.py j12 --fps 20
./generate_camera_overlay.py j12 --fps 25
./generate_camera_overlay.py j12 --fps 30
```

If the connector argument is omitted, the generator defaults to `all`. If `--fps` is omitted, it defaults to 25 Hz.

The generated files are written to `overlay/`:

```text
s600_sensing_cam_rx0123_portABCD_yuv_generated.dtso
s600_sensing_cam_rx0123_portABCD_yuv_generated.dtbo
```

Before running `install.sh`, verify that the final configuration summary printed by the generator matches the physical camera connections.

Generate J14 with the same selection workflow:

```bash
./generate_camera_overlay.py j14 --fps 25
```

Generate both connectors:

```bash
./generate_camera_overlay.py all --fps 25
```

### 2. Install the SDK

The installer updates the kernel, kernel headers, `hobot-camera`, `hobot-spdev`,
`hobot-multimedia-samples`, the boot overlay, and `/boot/config.txt`. Root
privileges are required:

```bash
sudo ./install.sh
```

The installer completes all preflight checks before modifying the system. It backs up the boot configuration and relevant overlays to:

```text
/boot/sensing-camera-backup-YYYYmmdd-HHMMSS/
```

A reboot is required after a successful installation. Do not attempt to load camera modules built for the new kernel while the old kernel is still running:

```bash
sudo reboot
```

### 3. Verify the Installation After Reboot

Confirm that the running kernel matches the target kernel printed by `install.sh`:

```bash
uname -r
grep '^dtbo_file_path=' /boot/config.txt
```

Load the camera graph:

```bash
cd ~/SENSING_CAM-RDKS600_LNX_SDK_V5.1.0
sudo ./load.sh
```

Check the V4L2 devices and relevant kernel messages:

```bash
v4l2-ctl --list-devices
ls -l /dev/media* /dev/video*
dmesg | grep -Ei 'sensing|s36|s56|max9295|max96712|error|fail'
```

### Capture Node Mapping

By default, `load.sh` routes S56 RAW streams through ISP/YNR/PYM and routes
known YUV cameras directly from CIM. `sudo ./load.sh --direct` selects scene 0
for direct capture. Both modes preserve four video node numbers per RX, one
per virtual channel:

| RX  | PHY   | Port A / VC0   | Port B / VC1   | Port C / VC2   | Port D / VC3   |
| --- | ----- | -------------- | -------------- | -------------- | -------------- |
| RX0 | D-PHY | `/dev/video2`  | `/dev/video3`  | `/dev/video4`  | `/dev/video5`  |
| RX1 | C-PHY | `/dev/video6`  | `/dev/video7`  | `/dev/video8`  | `/dev/video9`  |
| RX2 | D-PHY | `/dev/video10` | `/dev/video11` | `/dev/video12` | `/dev/video13` |
| RX3 | C-PHY | `/dev/video14` | `/dev/video15` | `/dev/video16` | `/dev/video17` |

`/dev/video0` and `/dev/video1` are not part of the RX0-RX3 VC capture
mapping. A stereo camera on Port A occupies VC0/VC1; one on Port C occupies
VC2/VC3. A node may exist even when its Port is disabled. Use the camera
mapping printed by `load.sh`, then verify the node's current format:

```bash
v4l2-ctl -d /dev/video10 --get-fmt-video
media-ctl -d /dev/media0 -p
```

### Resolution and Output Format by Camera Model

The generated overlay fixes each model's size. With the default automatic
RAW-to-ISP / YUV-direct route, use these capture settings for each active VC:

| Model | Streams | Width | Height | Expected V4L2 format |
| --- | ---: | ---: | ---: | --- |
| SN2M4EFGD | 1 | 1920 | 1080 | NV16 |
| SG3S11AFSK | 1 | 1920 | 1536 | NV16 |
| SG8-OX08DC-5300-G2G-Hxxx | 1 | 3840 | 2160 | NV16 |
| SG8S-AR0820C-5300-G2G-Hxxx | 1 | 3840 | 2160 | NV16 |
| S36 | 2 | 1920 | 1536 | NV16 on each VC |
| S56 | 2 | 2560 | 1984 | NV12 on each VC, after ISP |

Check `--get-fmt-video` on the actual node before streaming. In `--direct`
mode S56 outputs RAW10, so the NV12 commands below do not apply.

### Print the Frame Rate

Use the video node printed by `load.sh` and the size for its camera model.
For example, RX2 Port C with an SG8-OX08DC YUV camera uses `/dev/video12`:

```bash
v4l2-ctl -d /dev/video12 \
  --set-fmt-video=width=3840,height=2160,pixelformat=NV16 \
  --stream-mmap=4 \
  --stream-count=300 \
  --stream-to=/dev/null
```

For S56 on RX2 Port A, capture VC0 from `/dev/video10`. Use `/dev/video11`
for VC1:

```bash
v4l2-ctl -d /dev/video10 \
  --set-fmt-video=width=2560,height=1984,pixelformat=NV12 \
  --stream-mmap=4 --stream-count=300 --stream-to=/dev/null
```

The progress output includes the measured frame rate, for example
`25.00 fps`. The S36/S56 trigger rate may be 20, 25, or 30 Hz depending on
the generated overlay. Stop every process using a node before opening it in
another streaming test.

### Preview a Camera Stream with GStreamer

Use the actual video node and format for that camera. For example, the RX2
Port C YUV camera uses 3840x2160 NV16:

```bash
gst-launch-1.0 -v v4l2src device=/dev/video12 io-mode=mmap ! \
  'video/x-raw,format=NV16,width=3840,height=2160' ! \
  videoconvert ! glimagesink sync=false
```

For S56 on RX2 Port A, preview VC0 from `/dev/video10`. Use `/dev/video11`
for VC1:

```bash
gst-launch-1.0 -v v4l2src device=/dev/video10 io-mode=mmap ! \
  'video/x-raw,format=NV12,width=2560,height=1984' ! \
  videoconvert ! glimagesink sync=false
```

To preview S56 and YUV at the same time, run their preview commands in
separate terminals with different video nodes. One `v4l2src` reads only one
video node, and a node already open in another process may report `busy`.

### 4. Restore the Previous Boot Configuration

If the new overlay does not initialize the cameras correctly, restore `/boot/config.txt` from the backup directory printed by `install.sh`:

```bash
sudo cp /boot/sensing-camera-backup-YYYYmmdd-HHMMSS/config.txt /boot/config.txt
sudo reboot
```

The backup directory also contains the previously active overlay and any older copy of the target overlay, when those files existed before installation.

Rolling back the kernel requires installing the original linux-image, linux-headers, and hobot-camera DEB packages. Do not copy a single `.ko` file between different kernel releases.

### Important Notes

- Use a cold boot after replacing Sensor or SerDes drivers. Do not repeatedly hot-unload MAX96712 or MAX9295 drivers.
- `load.sh` does not hot-unload Sensor or SerDes modules. It rebuilds only the V4L2/HBN graph.
- Before running `load.sh`, stop GStreamer, `v4l2-ctl`, and any other application using camera device nodes.
- Two S56 stereo pairs connected to the same RX share one LPWM trigger source and therefore use the same trigger frequency.
- Equal trigger frequencies do not guarantee phase alignment between different LPWM channels. Validate multi-RX synchronization with an oscilloscope and frame timestamps.
- A successfully compiled device tree confirms structural validity only. Complete real-hardware probe, streaming, bandwidth, and synchronization validation before release.
