#### RDK version

* 4.0.5-20260507

#### DRIVER version

* V1.0.0 

  ​	First commit

* V1.0.1

  ​	Add S56C OV50Q40 camera

* 

#### Supported SENSING Camera Modules
* SG2-AR0233C-5200-G2A-Hxxx

  * support max 4 cameras to light up at the same time
  
* SN2M4EFGD

  * support max 4 cameras to light up at the same time
  
* SG3S11ALFK

  * support max 4 cameras to light up at the same time
  
* SP2504M2M-A

  * support max 4 cameras to light up at the same time
  
* SP2504M2M-A(mipi)

  * support max 2 cameras to light up at the same time
  
* SG8S-AR0820C-5300-G2A-Hxxx

  * support max 2 cameras to light up at the same time
  
* S36

  * support max 2 cameras to light up at the same time
  (Can only be connected to linkA and linkC of MAX96712 for use)
  
* S56C

  * support max 1 cameras to light up at the same time

    (Can only be connected to linkA or linkC of MAX96712 for use)

* OV50Q40(mipi)

  * support max 2 cameras to light up at the same time

#### Quick Bring Up(Take SG2-AR0233C-5200-G2A as an example)

1. Copy the hbn driver package to the working directory of the Jetson device, such as “/home/sunrise”

   ```
   /home/sunrise/hbn
   ```
2. Enter the driver directory

   ```
   cd hbn
   ```
3. Install camera driver
  
    ```
    sudo dpkg -i hobot-camera_4.0.5-20260930093605_arm64.deb
    sudo dpkg -i hobot-multimedia-samples_4.0.5-20260930093640_arm64.deb
    ```
  
4. Save single-channel camera images

   For example:

   ```
   cd /app/multimedia_samples/sample_vin/get_vin_data/
   make
   ./get_vin_data (root)
   No sensors specified.
   Usage: get_vin_data [OPTIONS]
   Options:
     -s <sensor_index>      Specify sensor index
     -l <link_port>         Specify gmsl sensor link_port
     -h                     Show this help message
   index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
   index: 1  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
   index: 2  sensor_name: ar0820std-30fps          config_file:linear_3840x2160_30fps_1lane.c 
   index: 3  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
   index: 4  sensor_name: ovx3cstd-30fps           config_file:linear_1920x1280_yuv_30fps_1lane.c
   index: 5  sensor_name: dummy                    config_file:dummy_sensor.c
   index: 6  sensor_name: sc230ai-30fps            config_file:linear_1920x1080_raw10_30fps_1lane.c
   index: 7  sensor_name: sc132gs-30fps            config_file:linear_1088x1280_raw10_30fps_2lane.c
   index: 8  sensor_name: ar0233std-30fps          config_file:linear_1920x1080_30fps_4lane.c
   index: 9  sensor_name: SG3S11ALFK               config_file:linear_1920x1536_30fps_4lane.c
   index: 10  sensor_name: SP2504M2M-A             config_file:linear_1600x1200_30fps_1lane.c
   index: 11  sensor_name: SN2M4EFGD               config_file:linear_1920x1080_30fps_4lane.c
   index: 12  sensor_name: S36                     config_file:linear_1920x1536_yuv_30fps_4lane.c
   index: 13  sensor_name: SP2504M2M-A_mipi_rx0    config_file:linear_1600x1200_raw10_30fps_1lane_rx0.c
   index: 14  sensor_name: SP2504M2M-A_mipi_rx1    config_file:linear_1600x1200_raw10_30fps_1lane_rx1.c
   ./get_vin_data -s 8 -l 0
   Using index:8  sensor_name:ar0233std-30fps  config_file:linear_1920x1080_30fps_4lane.c
    All deserial link info:
            [link_port:0] ar0233std:5@512
            [link_port:1] sc1336_gmsl:0@256
            [link_port:2] sc1336_gmsl:0@256
            [link_port:3] sc1336_gmsl:0@256
    vc_index:0
    deserial_config:,29,max96712, des_handle:82738

    ***************  Command Lists  ***************
    f    -- print VIN fps every second for 30 seconds
    g      -- get single frame
    l      -- get a set frames
    q      -- quit
    h      -- print help message

    Command:

   Enter "g" or "l" to start taking pictures Enter "f" to print frame rate

   In particular, for the S36 dual-camera, you can obtain the data from the two sensors separately by using 
   ./get_vin_data -s 12 -l 0 and 
   ```
5. Save multiple camera images
   
   For example: 

   ```
   cd /app/multimedia_samples/sample_vin/get_multi_vin_data
   make 
   ./get_multi_vin_data (root)
   Usage: get_multi_vin_data [Options]
    Options:
    -c, --config="sensor=id"
                    Configure parameters for each video pipeline, can be repeated up to 6 times.
                    sensor   --  Sensor index,can have multiple parameters, reference sensor list.
                    link     --  Sensor link port number, serdes sensor must be configured according to the hardware connection, can be set to [0-3] 0:A 1:B 2:C 3:D.
    -h, --help      Show help message
    Support sensor list:
    index: 0  sensor_name: imx219-30fps             config_file:linear_1920x1080_raw10_30fps_1lane.c
    index: 1  sensor_name: sc1336_gmsl-30fps        config_file:linear_1280x720_raw10_30fps_2lane.c
    index: 2  sensor_name: SG8S-AR0820-5300-G2A     config_file:linear_3840x2160_30fps_1lane.c
    index: 3  sensor_name: ar0820std-1080p30        config_file:linear_1920x1080_yuv_30fps_1lane.c
    index: 4  sensor_name: ovx3cstd-30fps           config_file:linear_1920x1280_yuv_30fps_1lane.c
    index: 5  sensor_name: dummy                    config_file:dummy_sensor.c
    index: 6  sensor_name: sc230ai-30fps            config_file:linear_1920x1080_raw10_30fps_1lane.c
    index: 7  sensor_name: sc132gs-30fps            config_file:linear_1088x1280_raw10_30fps_2lane.c
    index: 8  sensor_name: SG2-AR0233C-5200-G2A     config_file:linear_1920x1080_30fps_4lane.c
    index: 9  sensor_name: SG3S11ALFK               config_file:linear_1920x1536_30fps_4lane.c
    index: 10  sensor_name: SP2504M2M-A             config_file:linear_1600x1200_30fps_1lane.c
    index: 11  sensor_name: SN2M4EFGD               config_file:linear_1920x1080_30fps_4lane.c
    index: 12  sensor_name: S36                     config_file:linear_1920x1536_yuv_30fps_4lane.c
    index: 13  sensor_name: SP2504M2M-A_mipi_rx0    config_file:linear_1600x1200_raw10_30fps_1lane_rx0.c
    index: 14  sensor_name: SP2504M2M-A_mipi_rx1    config_file:linear_1600x1200_raw10_30fps_1lane_rx1.c

    Four SG2-AR0233C-5200-G2A-Hxxx cameras:
   ./get_multi_vin_data --config="sensor=8 link=0" --config="sensor=8 link=1" --config="sensor=8 link=2" --config="sensor=8 link=3"

    One SN2M4EFGD camera and three SG3S11ALFK cameras
    ./get_multi_vin_data --config="sensor=11 link=0" --config="sensor=9 link=1" --config="sensor=9 link=2" --config="sensor=9 link=3"

   Specifically, for the s36 dual-camera, you can use the commands 
   ./get_multi_vin_data --config="sensor=12 link=0" --config="sensor=12 link=2" 
   to simultaneously obtain the data from both sensors.
   ```

   note:This command is used to simultaneously capture the streams from four cameras. If the corresponding link-port is not connected to a camera, simply remove the corresponding "--config" option.

  6. Trigger mode

      Nowadays, all cameras adopt the default 30Hz trigger configuration. (Except for the SP2504M2M-A camera).

      Change trigger frame rate

      For example: SG2-AR0233C-5200-G2A-Hxxx camera

      ```
      cd /app/multimedia_samples/vp_sensors/ar0233c
      vi linear_1920x1080_yuv_30fps_4lane.c

      Modify this structure to change the frame rate.
      .lpwm_attr = {
                    .lpwm_chn_attr = {
                            /* VCON4 lpwm_chn[0] -> LPWM0 CH2 -> GPIO_CAM_2_1V8 -> MAX96712 MFP5 */
                            [0] = {
                                    .enable = 1,
                                    .trigger_source = 0,
                                    .trigger_mode = 1,
                                    .period = 33333,
                                    .offset = 10,
                                    .duty_time = 20,
                                    .threshold = 0,
                                    .adjust_step = 0,
                            },
                    },
      };
      
      eg:
      .period = 33333,  //30hz
      .period = 40000,  //25hz
      .period = 50000,  //20hz
      
      ```

  7. Mipi camera

      Now support two SP2504M2M-A MIPI camera and two OV50Q40 camera

      Currently, this camera does not support triggering. Please switch the switch SW2200 on the camera expansion board to the LPWM mode; otherwise, the lighting may fail.

      

