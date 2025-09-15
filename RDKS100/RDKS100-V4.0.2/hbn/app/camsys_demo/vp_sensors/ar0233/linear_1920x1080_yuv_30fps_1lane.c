#include "../vp_sensors.h"
// #include "vp_pym.h"

#define SENSOR_FPS 30

#define SENSOR_WIDTH 1920
#define SENSOR_HEIGHT 1080


//0820c 的sample 配置
static mipi_config_t ar0233std_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 4,
		.datatype = 30,
		.fps = 30,
		.mclk = 24,
		.mipiclk = 810,
		.width = 1920,
		.height = 1080,
		.linelenth = 2149,
		.framelenth = 1125 * 2,
		.settle = 22,
		.channel_num = 2,
		.channel_sel[0] = 0,
		.channel_sel[1] = 1,
	},
	.end_flag = MIPI_CONFIG_END_FLAG,
};

static camera_config_t ar0233std_camera_config = {
		/* 0 */
		.name = "ar0233std",
		.addr = 0x13,
		.eeprom_addr = 0x53,
		.serial_addr = 0x43,
		.sensor_mode = 0x5,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.extra_mode = 0,
		.config_index = 512,
		// .mipi_cfg = &ar0233std_mipi_config, // MIPI配置,NULL自动获取
		.end_flag = CAMERA_CONFIG_END_FLAG,
		.calib_lname = "disable",
};

static poc_config_t g_poc_cfg[] = {
	{
		/* 0 */
		.addr = 0x28,
		.poc_map = 0x2013,
		.end_flag = POC_CONFIG_END_FLAG,
	},
};

static deserial_config_t ar0233std_deserial_config = {
	.name = "max96712",
	.link_desp[0] = "ar0233std:0@512",
	.link_desp[1] = "ar0233std:0@512",
	.link_desp[2] = "ar0233std:0@512",
	.link_desp[3] = "ar0233std:0@512",
	.addr = 0x29,
	.poc_cfg = &g_poc_cfg[0],
	.end_flag = DESERIAL_CONFIG_END_FLAG,
};


static vin_ichn_attr_t ar0233std_vin_ichn_attr = {
	.width =  1920,
	.height = 1080,
	.format = 30,
};

static vin_attr_ex_t ar0233std_vin_attr_ex = {
	.cim_static_attr = {
		.water_level_mark = 0,
	},
};

static vin_ochn_attr_t ar0233std_vin_ochn_attr = {
	.ddr_en = 1,
	.vin_basic_attr = {
		.format = 30,
		.wstride = 0,
		.vstride = 0,
		.pack_mode = 1,
	},
	.pingpong_ring = 1,
	.roi_en = 0,
	.roi_attr = {
		.roi_x = 1280,
		.roi_y = 720,
		.roi_width = 64,
		.roi_height = 64,
	},
	.rawds_en = 0,
	.rawds_attr = {
		.rawds_mode = 0,
	},
	.magicNumber = 0x12345678,
};

static vin_node_attr_t ar0233std_vin_node_attr = {
	.vcon_attr = {
		.bus_main = 3,
		.bus_second = 3,
	},

	.cim_attr = {
		.mipi_en = 1,
		.cim_isp_flyby = 0,
		.cim_pym_flyby = 0,
		.mipi_rx = 4,
		.vc_index = 3,
		.ipi_channels = 1,
		.y_uv_swap = 0,
		.func = {
			.enable_frame_id = 1,
			.set_init_frame_id = 1,
			.enable_pattern = 0,
		},
		.rdma_input = {
			.rdma_en = 0,
			.stride = 0,
			.pack_mode = 1,
			.buff_num = 6,
		},
	}
};


vp_sensor_config_t ar0233std_linear_1920x1080_yuv_30fps_1lane = {
	.chip_id_reg = 0,
	.chip_id = 0x0231,
	.sensor_i2c_addr_list = {0x10},
	.sensor_type = SENSOR_TYPE_GMSL_YUV,
	.sensor_name = "ar0233std-1080p30",
	.config_file = "linear_1920x1080_yuv_30fps_1lane.c",
	.camera_config = &ar0233std_camera_config,
	.deserial_node_attr = &ar0233std_deserial_config,
	.vin_ichn_attr = &ar0233std_vin_ichn_attr,
	.vin_node_attr = &ar0233std_vin_node_attr,
	.vin_attr_ex = &ar0233std_vin_attr_ex,
	.vin_ochn_attr = &ar0233std_vin_ochn_attr,

	// .isp_attr      = &ar0233std_isp_attr,
	// .isp_ichn_attr = &ar0233std_isp_ichn_attr,
	// .isp_ochn_attr = &ar0233std_isp_ochn_attr,
	// .ynr_attr = &ar0233std_ynr_attr,
};