#include "vp_sensors.h"

// 本文件定义了一个 1920x1080@30fps 的虚拟 Camera Sensor
// 在实际的示例代码中，用户可以根据需求进行变更来符合实际的开发需求

#define SENSOR_WIDTH  1920
#define SENSOR_HEIGHT  1080
#define SENSOE_FPS 30
#define RAW10 0x2B

static mipi_config_t mipi_config = {
	.rx_enable = 0,
};

static camera_config_t camera_config = {
	.name = "dummy",
	.addr = 0xFF,
	.sensor_mode = NORMAL_M,
	.fps = SENSOE_FPS,
	.format = RAW10,
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.mipi_cfg = &mipi_config,
	.calib_lname = "disable",
};

static vin_node_attr_t vin_node_attr = {
	.cim_attr = {
		.mipi_rx = 1,
		.vc_index = 0,
		.ipi_channels = 1,
		.cim_isp_flyby = 1,
		.func = {
			.enable_frame_id = 0,
			.set_init_frame_id = 0,
		},
	},
};

static vin_ichn_attr_t vin_ichn_attr = {
	.width = SENSOR_WIDTH,
	.height = SENSOR_HEIGHT,
	.format = RAW10,
};

static vin_ochn_attr_t vin_ochn_attr = {
	.ddr_en = 1,
	.ochn_attr_type = VIN_BASIC_ATTR,
	.vin_basic_attr = {
		.format = RAW10,
		// 硬件 stride 跟格式匹配，通过行像素根据raw数据bit位数计算得来
		// 8bit：x1, 10bit: x2 12bit: x2 16bit: x2,例raw10，1920 x 2 = 3840
		.wstride = (SENSOR_WIDTH) * 2,
	},
};

static vin_attr_ex_t vin_attr_ex = {
	.cim_static_attr = {
		.water_level_mark = 0,
	},
};

static isp_attr_t isp_attr = {
	.sched_mode = 1,
};

static isp_ichn_attr_t isp_ichn_attr = {
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

static isp_ochn_attr_t isp_ochn_attr = {
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

vp_sensor_config_t dummy_sensor_config = {
	.sensor_name = "dummy",
	.config_file = "dummy_sensor.c",
	.camera_config = &camera_config,
	.vin_ichn_attr = &vin_ichn_attr,
	.vin_node_attr = &vin_node_attr,
	.vin_attr_ex   = &vin_attr_ex,
	.vin_ochn_attr = &vin_ochn_attr,
	.isp_attr      = &isp_attr,
	.isp_ichn_attr = &isp_ichn_attr,
	.isp_ochn_attr = &isp_ochn_attr,
};
