#include "vp_sensors.h"

static mipi_config_t imx219_mipi_config = {
	.rx_enable = 1,
	.rx_attr = {
		.phy = 0,
		.lane = 2,
		.datatype = 0x12b,
		.fps = 30,
		.mclk = 24,
		.mipiclk = 1728,
		.width = 0,
		.height = 0,
		.linelenth = 0,
		.framelenth = 0,
		.settle = 0,
		.channel_num = 0,
		.channel_sel = {0},
	},

	.rx_ex_mask = 0x40,
	.rx_attr_ex = {
		.stop_check_instart = 1,
	},

	.end_flag = MIPI_CONFIG_END_FLAG,
};

static camera_config_t imx219_camera_config = {
		/* 0 */
		.name = "imx219",
		.addr = 0x10,
		.eeprom_addr = 0x51,
		.serial_addr = 0x40,
		.sensor_mode = 1,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.extra_mode = 0,
		.config_index = 0,
		.mipi_cfg = &imx219_mipi_config, // MIPI配置,NULL自动获取
		.end_flag = CAMERA_CONFIG_END_FLAG,
		.calib_lname = "lib_imx219_linear.so",
};


static isp_attr_t imx219_isp_attr = {
	.channel = {
		.hw_id = 0,
		.slot_id = 4,
		.ctx_id = -1, //#define AUTO_ALLOC_ID -1
	},
	.work_mode = ISP_WORK_MODE_NOMAL,
	.hdr_mode = HDR_MODE_LINEAR,
	.size = {
		.width = 1920,
		.height = 1080,
	},
	.frame_rate = 30,
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

static isp_ichn_attr_t imx219_isp_ichn_attr = {
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

static isp_ochn_attr_t imx219_isp_ochn_attr = {
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

static vin_node_attr_t imx219_vin_node_attr = {
	.vcon_attr = {
		.bus_main = 2,
		.bus_second = 2,
	},

	.cim_attr = {
		.mipi_en = 1,
		.cim_isp_flyby = 0,
		.cim_pym_flyby = 0,
		.mipi_rx = 0,
		.vc_index = 0,
		.ipi_channels = 1,
		.y_uv_swap = 0, //(uint32_t)vpf_get_json_value(p_node_mipi, "y_uv_swap");
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

static vin_ichn_attr_t imx219_vin_ichn_attr = {
	.width =  1920,
	.height = 1080,
	.format = 43,
};

static vin_attr_ex_t imx219_vin_attr_ex = {
	.cim_static_attr = {
		.water_level_mark = 0,
	},
};

static vin_ochn_attr_t imx219_vin_ochn_attr = {
	.ddr_en = 1,
	.vin_basic_attr = {
		.format = 43,
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

static struct ynr_init_attr imx219_ynr_attr = {
	.work_mode = 1,
	.slot_id = 4,

	.width = 1920,
	.height = 1080,
	.nr_static_switch = 0b11, // (nr3d_en << 1) | (nr2d_en);
	.in_stride = {
		1920, 1080
	},
	.nr2d_en = 1,
	.nr3d_en = 1,

	.dma_output_en = 1, // nr3d_en

	.debug_en = 0,
};

vp_sensor_config_t imx219_linear_1920x1080_raw10_30fps_1lane = {
	.chip_id_reg = 0,
	.chip_id = 0x0219,
	.sensor_i2c_addr_list = {0x10},
	.sensor_name = "imx219-30fps",
	.config_file = "linear_1920x1080_raw10_30fps_1lane.c",
	.camera_config = &imx219_camera_config,
	.deserial_node_attr = NULL,
	// .vin_attr = &imx219_vin_attr,
	.vin_ichn_attr = &imx219_vin_ichn_attr,
	.vin_node_attr = &imx219_vin_node_attr,
	.vin_attr_ex = &imx219_vin_attr_ex,
	.vin_ochn_attr = &imx219_vin_ochn_attr,

	.isp_attr      = &imx219_isp_attr,
	.isp_ichn_attr = &imx219_isp_ichn_attr,
	.isp_ochn_attr = &imx219_isp_ochn_attr,
	.ynr_attr = &imx219_ynr_attr
};
