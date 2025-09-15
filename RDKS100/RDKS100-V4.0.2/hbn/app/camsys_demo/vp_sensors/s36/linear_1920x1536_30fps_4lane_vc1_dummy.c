#include "vp_sensors.h"
// #include "vp_pym.h"


#define SENSOR_FPS 30

#define SENSOR_WIDTH 1920
#define SENSOR_HEIGHT 1536

static mipi_config_t dummystd_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 1,
		.datatype = 30,
		.fps = 30,
		.mclk = 24,
		.mipiclk = 810,
		.width = 1920,
		.height = 1536,
		.linelenth = 2149,
		.framelenth = 1125 * 2,
		.settle = 22,
		.channel_num = 1,
		.channel_sel = {0},
	},
};

static camera_config_t dummystd_camera_config = {
		/* 0 */
		.name = "dummystd",
		.addr = 0xff,
		.eeprom_addr = 0xff,
		.serial_addr = 0xff,
		.sensor_mode = 0x1,
		.fps = 30,
		.width = 1920,
		.height = 1536,
		.extra_mode = 4,
		.config_index = 0,
		// .mipi_cfg = &dummystd_mipi_config, // MIPI配置,NULL自动获取
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

static deserial_config_t dummystd_deserial_config = {
	.name = "max96712",
	.link_desp[0] = "dummystd:4@0",
	.link_desp[1] = "dummystd:4@0",
	.link_desp[2] = "dummystd:4@0",
	.link_desp[3] = "dummystd:4@0",
	.addr = 0x29,
	.poc_cfg = &g_poc_cfg[0],
	.end_flag = DESERIAL_CONFIG_END_FLAG,
};

static vin_ichn_attr_t dummystd_vin_ichn_attr = {
	.width =  1920,
	.height = 1536,
	.format = 30,
};

static vin_attr_ex_t dummystd_vin_attr_ex = {
	.cim_static_attr = {
		.water_level_mark = 0,
	},
};

static vin_ochn_attr_t dummystd_vin_ochn_attr = {
	.ddr_en = 1,
	.vin_basic_attr = {
		.format = 30,
		.wstride = 0,
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
};

static vin_node_attr_t dummystd_vin_node_attr = {
	.vcon_attr = {
		.bus_main = 3,
		.bus_second = 3,
	},

	.cim_attr = {
		.mipi_en = 1,
		.cim_isp_flyby = 0,
		.cim_pym_flyby = 0,
		.mipi_rx = 4,
		.vc_index = 1,
		.ipi_channels = 1,
		.y_uv_swap = 0,
		.func = {
			.enable_frame_id = 1,
			.set_init_frame_id = 1,
			.enable_pattern = 0,
			.lpwm_trig_sel = 2,
		},
		.rdma_input = {
			.rdma_en = 0,
			.stride = 0,
			.pack_mode = 1,
			.buff_num = 6,
		},
	},
	.lpwm_attr = {
		.lpwm_chn_attr = {
			{	.enable = 1,
				.trigger_source = 0,
				.trigger_mode = 1,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.enable = 1,
				.trigger_source = 0,
				.trigger_mode = 1,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.enable = 1,
				.trigger_source = 0,
				.trigger_mode = 1,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
			{	.enable = 1,
				.trigger_source = 0,
				.trigger_mode = 1,
				.period = 33333,
				.offset = 10,
				.duty_time = 100,
				.threshold = 0,
				.adjust_step = 0,
			},
		},
	},
};

static isp_attr_t dummystd_isp_attr = {
	.channel = {
		.hw_id = 0,
		.slot_id = 4,
		.ctx_id = -1, //#define AUTO_ALLOC_ID -1
	},
	.work_mode = 0,
	.hdr_mode = 1,
	.size = {
		.width = SENSOR_WIDTH,
		.height = SENSOR_HEIGHT,
	},
	.frame_rate = 10,
	.sched_mode = 1,
	.algo_state = 1,
	.isp_combine = {
		.isp_channel_mode = 0, //ISP_CHANNEL_MODE_NORMAL
		.bind_channel = {
			.bind_hw_id = 0,
			.bind_slot_id = 0,
		},
	},
	.clear_record = 0, //json和代码中未拿到，设置为0
	.isp_sw_ctrl = {
		.ae_stat_buf_en = 1,
		.awb_stat_buf_en = 1,
		.ae5bin_stat_buf_en = 1,
		.ctx_buf_en = 0,
		.pixel_consistency_en = 0,
	},
};

static isp_ichn_attr_t dummystd_isp_ichn_attr = {
	.input_crop_cfg = {
		.enable = 0,
		.rect = {
			.x = 0,
			.y = 0,
			.width = 0,
			.height = 0,
		},
	},
	.in_buf_noclean = 1,
	.in_buf_noncached = 0,
};

static isp_ochn_attr_t dummystd_isp_ochn_attr = {
	.output_crop_cfg = {
		.enable = 0,
		.rect = {
			.x = 0,
			.y = 0,
			.width = 0,
			.height = 0,
		},
	},
	.out_buf_noinvalid = 1,
	.out_buf_noncached = 0,
	.output_raw_level = 0, //ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA
	.stream_output_mode = 0, //convert_isp_stream_output(1),
	.axi_output_mode = 9, //convert_isp_axi_output(0),
	.buf_num = 3,
};

static struct ynr_init_attr dummystd_ynr_attr = {
	.work_mode = 1,
	.slot_id = 4,

	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.nr_static_switch = 0b11, // (nr3d_en << 1) | (nr2d_en);
	.in_stride = {
		SENSOR_WIDTH, SENSOR_HEIGHT
	},
	.nr2d_en = 1,
	.nr3d_en = 1,

	.dma_output_en = 1, // nr3d_en

	.debug_en = 0,
};

vp_sensor_config_t dummystd_linear_1920x1536_30fps_4lane_vc1 = {
	.chip_id_reg = 0,
	.chip_id = 0x0231,
	.sensor_i2c_addr_list = {0x10}, 
	.sensor_type = SENSOR_TYPE_GMSL_YUV,
	// .sensor_i2c_addr_list = {0, 0, 0, 0, 0, 0, 0, 0}, // don't know chipid, so set addr 0
	.sensor_name = "dummystd",
	.config_file = "linear_1920x1536_30fps_4lane_vc1_dummy.c",
	.camera_config = &dummystd_camera_config,
	.deserial_node_attr = &dummystd_deserial_config,
	// .vin_attr = &dummystd_vin_attr,

	.vin_ichn_attr = &dummystd_vin_ichn_attr,
	.vin_node_attr = &dummystd_vin_node_attr,
	.vin_attr_ex = &dummystd_vin_attr_ex,
	.vin_ochn_attr = &dummystd_vin_ochn_attr,

	.isp_attr      = &dummystd_isp_attr,
	.isp_ichn_attr = &dummystd_isp_ichn_attr,
	.isp_ochn_attr = &dummystd_isp_ochn_attr,
	.ynr_attr = &dummystd_ynr_attr,
};
