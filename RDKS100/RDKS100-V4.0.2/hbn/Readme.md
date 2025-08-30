#### RDK version

* 4.0.2

#### Supported SENSING Camera Modules
* SG1S-OX01F10C-G1G-Hxxx

  * support max 4 cameras to light up at the same time
* SG2-AR0231C-0202-GMSL-Hxxx

  * support max 4 cameras to light up at the same time
* SG2-AR0233-5200-G2A-Hxxx

  * support max 4 cameras to light up at the same time
* SG3-ISX031C-GMSL2F-Hxxx

  * support max 4 cameras to light up at the same time
* SHW3H-Hxxx

  * support max 4 cameras to light up at the same time
* SHF3L-Hxxx

  * support max 2 cameras to light up at the same time
* SG8S-AR0820C-5300-G2A-Hxxx

  * support max 2 cameras to light up at the same time
* SG3S-ISX031-DUAL-MIPI-Hxxx

  * support max 1 cameras to light up at the same time


#### Quick Bring Up(Take SG2-AR0233C-5200-G2A as an example)

1. Copy the hbn driver package to the working directory of the Jetson device, such as “/home/sunrise”

   ```
   /home/nvidia/hbn
   ```
2. Enter the driver directory

   ```
   cd hbn
   ```
3. Install camera driver
  
    ```
    chmod a+x install.sh
    ./install
    This package is use for RDK S100 on V4.0.2
    1:SG1S-OX01F10C-G1G
    2:SG2-AR0231C-0202-GMSL
    3:SG2-AR0233C-5200-G2A
    4:SG3S-ISX031C-GMSL2F
    5:SHW3H
    6:SHF3L
    7:SG8S-AR0820C-5300-G2A
    8:SG3S-ISX031-DUAL-MIPI
    Press select your camera type:
    3
    (Reading database ... 212484 files and directories currently installed.)
    Preparing to unpack .../hobot-camera_4.0.2-20250829164010_arm64.deb ...
    Unpacking hobot-camera (4.0.2-20250829164010) over (4.0.2-20250829164010) ...
    Setting up hobot-camera (4.0.2-20250829164010) ...
    Processing triggers for libc-bin (2.35-0ubuntu3.10) ...
    
    ```
  
6. Save single-channel camera images

   For example:

   ```
   cd /home/sunrise/hbn/app/camsys_demo_capture/sample_vin/get_vin_data/
   make
   ./get_vin_data (root)
   No sensors specified.
   Usage: get_vin_data [OPTIONS]
   Options:
     -s <sensor_index>      Specify sensor index
     -o <online>            Specify vin mode of camera_config_t
     -t <settle_value>      Specify settle time for debug
     -m <sensor_mode>       Specify sensor mode of camera_config_t
     -l <link_port>         Specify gmsl sensor link_port
     -h                     Show this help message
   index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
   index: 1  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
   index: 2  sensor_name: ovx3cstd-30fps           config_file:linear_1920x1280_yuv_30fps_1lane.c
   index: 3  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
   index: 4  sensor_name: ox01fstd-30fps           config_file:linear_1280x960_30fps_1lane.c
   index: 5  sensor_name: ar0231std-30fps          config_file:linear_1920x1080_30fps_1lane.c
   index: 6  sensor_name: ar0233std-30fps          config_file:linear_1920x1080_30fps_1lane.c
   index: 7  sensor_name: isx031std                config_file:linear_1920x1536_30fps_1lane.c
   index: 8  sensor_name: shw3h_shf3l_std-30fps    config_file:linear_1920x1536_30fps_1lane.c
   index: 9  sensor_name: shw3h_shf3l_std-60fps    config_file:linear_1920x1536_60fps_1lane.c
   index: 10  sensor_name: ar0820std-30fps         config_file:linear_3840x2160_30fps_1lane.c
   index: 11  sensor_name: isx031_dual             config_file:linear_1920x1536_yuv_30fps_1lane_left.c
   index: 12  sensor_name: isx031_dual             config_file:linear_1920x1536_yuv_30fps_1lane_right.c

   ./get_vin_data -s 6 -l 0
   Using index:6  sensor_name:ar0233std-30fps  config_file:linear_1920x1080_30fps_1lane.c
   addr: 0x11 ,serial_addr: 0x41 ,eeprom_addr : 0x51
   All deserial link info:
        [link_port:0] ar0233std:2@512
        [link_port:1] sc1336_gmsl:0@256
        [link_port:2] sc1336_gmsl:0@256
        [link_port:3] sc1336_gmsl:0@256
   vc_index:0

   ***************  Command Lists  ***************
    g      -- get single frame 
    l      -- get a set frames 
    q      -- quit  
    h      -- print help message

   Command: 

   Enter "g" or "l" to start taking pictures

   ```
7. Save multiple camera images
   
   For example:

   ```
   cd /home/sunrise/hbn/app/camsys_demo_capture/sample_vin/get_multi_vin_data/
   make 
   ./get_multi_vin_data (root)
   Usage: get_multi_vin_data [Options]
   Options:
   -c, --config="sensor=id"
                   Configure parameters for each video pipeline, can be repeated up to 6 times.
                   sensor   --  Sensor index,can have multiple parameters, reference sensor list.
                   mode     --  Sensor mode of camera_config_t
                   link     --  Sensor link port number, gsml sensor must be configured according to the hardware connection, can be set to [0-3].
   -h, --help      Show help message
   Support sensor list:
   index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
   index: 1  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
   index: 2  sensor_name: ovx3cstd-30fps           config_file:linear_1920x1280_yuv_30fps_1lane.c
   index: 3  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
   index: 4  sensor_name: ox01fstd-30fps           config_file:linear_1280x960_30fps_1lane.c
   index: 5  sensor_name: ar0231std-30fps          config_file:linear_1920x1080_30fps_1lane.c
   index: 6  sensor_name: ar0233std-30fps          config_file:linear_1920x1080_30fps_1lane.c
   index: 7  sensor_name: isx031std                config_file:linear_1920x1536_30fps_1lane.c
   index: 8  sensor_name: shw3h_shf3l_std-30fps    config_file:linear_1920x1536_30fps_1lane.c
   index: 9  sensor_name: shw3h_shf3l_std-60fps    config_file:linear_1920x1536_60fps_1lane.c
   index: 10  sensor_name: ar0820std-30fps         config_file:linear_3840x2160_30fps_1lane.c
   index: 11  sensor_name: isx031_dual             config_file:linear_1920x1536_yuv_30fps_1lane_left.c
   index: 12  sensor_name: isx031_dual             config_file:linear_1920x1536_yuv_30fps_1lane_right.c

   ./get_multi_vin_data --config="sensor=6 link=0" --config="sensor=6 link=1" --config="sensor=6 link=2" --config="sensor=6 link=3"
   ```

   note:This command is used to simultaneously capture the streams from four cameras. If the corresponding link-port is not connected to a camera, simply remove the corresponding "--config" option.
   
