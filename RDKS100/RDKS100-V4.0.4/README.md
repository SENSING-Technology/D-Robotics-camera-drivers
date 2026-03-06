#### RDK version

* 4.0.4

#### Supported SENSING Camera Modules
* SHW3G
  * Supports up to 4 cameras streaming simultaneously.

* S56
  * Supports up to 1 camera streaming simultaneously.
  * Can only be connected to linkA of MAX96712.

#### Quick Bring-Up (Take S56 and SHW3G as an example)

## Wiring Diagram

There are 4 connectors on the right side (arranged as a 2×2 block). Connect as follows:

- Bottom-left connector: `s56g`
- Top-right connector: `shw3g`

```text
Right-side 4 connectors (illustration)

┌────────────────┬────────────────┐
│ Top-left: N/A  │Top-right: shw3g│
├────────────────┼────────────────┤
│Bottom-left:s56g│Bottom-right:N/A│
└────────────────┴────────────────┘
```

## hobot-camera_4.0.4-20260305145416_arm64.deb

This directory provides the package: `hobot-camera_4.0.4-20260305145416_arm64.deb`.

It is a Debian (arm64) package that includes drivers and related components for:

- `shw3g`
- `s56g`

This directory also includes sample tools for data capture:

- `get_multi_vin_data`: capture RAW from multiple VIN pipelines
- `get_isp_data`: ISP-related data capture (if needed)

## Install

Run on S100:

```bash
sudo dpkg -i hobot-camera_4.0.4-20260304152400_arm64.deb
```

If dependencies are missing, run (depending on your system):

```bash
sudo apt-get -f install
```

## Capture RAW Data

### Multi-stream capture

Use `get_multi_vin_data` to capture RAW data from multiple streams.

Example (two `s56g` + one `shw3g`):

```bash
cd hobot-multimedia-samples/debian/app/multimedia_samples/sample_vin/get_multi_vin_data
make
./get_multi_vin_data  -c "sensor=0 link=0" -c "sensor=1 link=1" -c "sensor=2 link=2"
```

### Example output

```text
Using index:0  sensor_name:s56std  config_file:linear_2560x1984_30fps_1lane_right.c
Using index:1  sensor_name:s56std  config_file:linear_2560x1984_30fps_1lane_left.c
Using index:2  sensor_name:shw3gstd  config_file:linear_2064x1552_30fps_1lane.c
Pipeline index 0:
        Sensor index: 0
        Sensor name: s56std
        Active mipi host: 4
Pipeline index 1:
        Sensor index: 0
        Sensor name: s56std
        Active mipi host: 4
Pipeline index 2:
        Sensor index: 0
        Sensor name: shw3gstd
        Active mipi host: 4
Verbose: 1
vc_index:0
vc_index:1
vc_index:2
All deserial link info:
        [link_port:0] s56std:0@512
        [link_port:1] s56std:0@512
        [link_port:2] shw3gstd:0@0
        [link_port:3] sc1336_gmsl:0@256
deserial_config:29_max96712, des_handle:213810
Dumping RAW data: handle 34661, resolution: 2560x1984 (stride: 3840), size: 7618560, frame id: 1, timestamp: 66628391700
Dumping RAW data: handle 100197, resolution: 2560x1984 (stride: 3840), size: 7618560, frame id: 1, timestamp: 66628414125
Dump image to file(handle_34661_chn0_2560x1984_stride_3840_frameid_1_ts_66628391700.raw), size(7618560) succeeded
Pipeline 0 FPS: 0
Dump image to file(handle_100197_chn1_2560x1984_stride_3840_frameid_1_ts_66628414125.raw), size(7618560) succeeded
Pipeline 1 FPS: 0
Dumping RAW data: handle 165733, resolution: 2064x1552 (stride: 3104), size: 4817408, frame id: 1, timestamp: 66702398425
Dump image to file(handle_165733_chn2_2064x1552_stride_3104_frameid_1_ts_66702398425.raw), size(4817408) succeeded
Pipeline 2 FPS: 0
Dumping RAW data: handle 100197, resolution: 2560x1984 (stride: 3840), size: 7618560, frame id: 31, timestamp: 67628421450
Dumping RAW data: handle 34661, resolution: 2560x1984 (stride: 3840), size: 7618560, frame id: 31, timestamp: 67628414175
Dump image to file(handle_34661_chn0_2560x1984_stride_3840_frameid_31_ts_67628414175.raw), size(7618560) succeeded
Pipeline 0 FPS: 29
Dump image to file(handle_100197_chn1_2560x1984_stride_3840_frameid_31_ts_67628421450.raw), size(7618560) succeeded
Pipeline 1 FPS: 29
Dumping RAW data: handle 165733, resolution: 2064x1552 (stride: 3104), size: 4817408, frame id: 31, timestamp: 67702086875
Dump image to file(handle_165733_chn2_2064x1552_stride_3104_frameid_31_ts_67702086875.raw), size(4817408) succeeded
Pipeline 2 FPS: 30
```

## Capture YUV Data

Example (right `s56g`):

```bash
cd /root/hobot-multimedia-samples/debian/app/multimedia_samples/sample_isp/get_isp_data
make
./get_isp_data -s 0 -l 0
```

### Example output

```text
Using index:0  sensor_name:s56std  config_file:linear_2560x1984_30fps_1lane_right.c
All deserial link info:
        [link_port:0] s56std:0@512
        [link_port:1] sc1336_gmsl:0@256
        [link_port:2] sc1336_gmsl:0@256
        [link_port:3] sc1336_gmsl:0@256

INFO: ISP channel info:
        input info: [mipi_rx: 4] [is_online: 0]
        isp channel info: [hw_id: 0] [slot_id: 4]
deserial_config:,29,max96712, des_handle:82738

***************  Command Lists  ***************
 g      -- get single frame
 l      -- get a set frames
 q      -- quit
 h      -- print help message

Command: g
handle 100197 isp dump yuv 2560x1984(stride:2560), buffer size: 5079040 + 2539520 frame id: 26, timestamp: 198363261625
```

Example (left `s56g`):

Note: You must start the right camera first. In terminal 1:

```bash
./get_multi_vin_data -c "sensor=0 link=0"
```

Then capture the left YUV image in terminal 2:

```bash
./get_isp_data -s 1 -l 1
```

```text
Using index:1  sensor_name:s56std  config_file:linear_2560x1984_30fps_1lane_left.c
All deserial link info:
        [link_port:0] sc1336_gmsl:0@256
        [link_port:1] s56std:0@512
        [link_port:2] sc1336_gmsl:0@256
        [link_port:3] sc1336_gmsl:0@256

INFO: ISP channel info:
        input info: [mipi_rx: 4] [is_online: 0]
        isp channel info: [hw_id: 0] [slot_id: 4]
deserial_config:,29,max96712, des_handle:82738

***************  Command Lists  ***************
 g      -- get single frame
 l      -- get a set frames
 q      -- quit
 h      -- print help message

Command: g
handle 100197 isp dump yuv 2560x1984(stride:2560), buffer size: 5079040 + 2539520 frame id: 26, timestamp: 305464533500
```