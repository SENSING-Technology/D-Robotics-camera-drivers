#include <stdio.h>

#include "hb_vin_data_info.h"
#include "hb_vpm_common.h"
#include "hb_vin_data_info.h"
#include "hb_vpm_data_info.h"
#include "hb_vio_interface.h"
#include "hb_camera_data_config.h"
#include "hbn_vpf_data_info.h"
#include "hbn_vpf_interface.h"
#include "hb_deserial_interface.h"
#include "hb_camera_interface.h"
#include "hbn_isp_cfg.h"
#include "hbn_sth_cfg.h"
#include "hbn_pym_cfg.h"
#include "hbn_ynr_cfg.h"
#include "hb_mem_mgr.h"
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include <signal.h>

#define H264_FNAME "cim-isp-pym-stitch.h264"
#define MAGIC_NUMBER 0x12345678
#define TIMEOUT 10
#define PIPELINE_NUM 2
#ifndef TRUE
#define TRUE			1
#endif /* TRUE */
#ifndef FALSE
#define FALSE		   0
#endif /* FALSE */
#define ERR_CON_EQ(ret, a) do {\
         if ((ret) != (a)) {\
             printf("%s(%d) failed, ret %d\n", __func__, __LINE__, (int32_t)(ret));\
             return (ret);\
         }\
     } while(0)\

#define ERR_CON_NE(ret, a) do {\
         if ((ret) == (a)) {\
             printf("%s(%d) failed, ret %ld\n", __func__, __LINE__, (ret));\
             return (ret);\
         }\
     } while(0)\

typedef struct stitch_resource_s {
	char alpha_file[100];
	char beta_file[100];
	uint32_t alpha_file_size;
	uint32_t beta_file_size;
} stitch_resource_t;

typedef struct image_frame_s {
	int32_t cnt;
	hbn_vnode_image_t vnode_buffer;
	media_codec_buffer_t in_buffer;
	media_codec_buffer_t out_buffer;
	media_codec_output_buffer_info_t info;
}imgframe_t;

struct ynr_init_attr {
    uint32_t work_mode;
    uint32_t slot_id;
    uint32_t width;
    uint32_t height;
    uint32_t nr_static_switch;
    uint32_t in_stride[2];  //just for hbn y\uv
    uint32_t nr2d_en;
    uint32_t nr3d_en;
    uint32_t dma_output_en;
    uint32_t debug_en;
};

stitch_resource_t stitch_res;
bool is_loop;

vin_attr_t vin_attr[] = {
	[0] = {
		.vin_node_attr = {
			.vcon_attr = {
				.bus_main = 2,
				.bus_second = 2,
			},

			.cim_attr = {
				.mipi_en = 1,
				.mipi_rx = 0,
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
					.lpwm_trig_sel = 0xffff,
				},
				.rdma_input = {
					.rdma_en = 0,
					.stride = 0,
					.pack_mode = 1,
					.buff_num = 6,
				},
			},
            .magicNumber = MAGIC_NUMBER,
		},

		.vin_ichn_attr = {
			.width =  1920,
			.height = 1080,
			.format = 43,
		},

		.vin_attr_ex = {
			.cim_static_attr = {
				.water_level_mark = 0,
			},
		},

		.vin_ochn_attr = {
			[VIN_MAIN_FRAME] = { //vin_ochn0_attr
				.ddr_en = 1,
				.roi_en = 0,
				.rawds_en = 0,
				.pingpong_ring = 1,
				.vin_basic_attr = {
					.format = 43,
					.wstride = 0,
					.pack_mode = 1,
				},
				.roi_attr = {
					.roi_x = 0,
					.roi_y = 0,
					.roi_width = 1920,
					.roi_height = 1080,
				},
				.rawds_attr = {
					.rawds_mode = 0,
				},
		        .magicNumber = 0x12345678,
			},
		},
		.vin_ochn_buff_attr = {
			[VIN_MAIN_FRAME] = { //vin_ochn0_buff_attr
				.buffers_num = 6,
			},
			[VIN_EMB] = { //vin_ochn3_buff_attr
				.buffers_num = 6,
			},
			[VIN_ROI] = { //vin_ochn4_buff_attr
				.buffers_num = 6,
			},
		},
		.magicNumber = 0x12345678,
	},
	[1] = {
		.vin_node_attr = {
			.vcon_attr = {
				.bus_main = 3,
				.bus_second = 3,
			},

			.cim_attr = {
				.mipi_en = 1,
				.mipi_rx = 1,
				.cim_isp_flyby = 0,
				.cim_pym_flyby = 0,
				.vc_index = 0,
				.ipi_channels = 1,
				.y_uv_swap = 0, //(uint32_t)vpf_get_json_value(p_node_mipi, "y_uv_swap");
				.func = {
					.enable_frame_id = 1,
					.set_init_frame_id = 1,
					.enable_pattern = 0,
					.lpwm_trig_sel = 0xffff,
				},
				.rdma_input = {
					.rdma_en = 0,
					.stride = 0,
					.pack_mode = 1,
					.buff_num = 6,
				},
			},
            .magicNumber = 0x12345678,
		},

		.vin_ichn_attr = {
			.width =  1920,
			.height = 1080,
			.format = 43,
		},

		.vin_attr_ex = {
			.cim_static_attr = {
				.water_level_mark = 0,
			},
		},

		.vin_ochn_attr = {
			[VIN_MAIN_FRAME] = { //vin_ochn0_attr
				.ddr_en = 1,
				.rawds_en = 0,
				.roi_en = 0,
				.pingpong_ring = 1,
				.ochn_attr_type = VIN_BASIC_ATTR,
				.vin_basic_attr = {
					.format = 43,
					.wstride = 0,
					.pack_mode = 1,
				},
				.roi_attr = {
					.roi_x = 0,
					.roi_y = 0,
					.roi_width = 1920,
					.roi_height = 1080,
				},
				.rawds_attr = {
					.rawds_mode = 0,
				},
                .magicNumber = 0x12345678,
			},
		},
		.vin_ochn_buff_attr = {
			[VIN_MAIN_FRAME] = { //vin_ochn0_buff_attr
				.buffers_num = 6,
			},
			[VIN_EMB] = { //vin_ochn3_buff_attr
				.buffers_num = 6,
			},
			[VIN_ROI] = { //vin_ochn4_buff_attr
				.buffers_num = 6,
			},
		},
		.magicNumber = 0x12345678,
	},
};

isp_cfg_t isp_cfg[] = {
	[0] = {
		.isp_attr = {
	    	.channel = {
	    		.hw_id = 1,
	    		.slot_id = 4,
	    		.ctx_id = -1,
	    	},
	    	.sched_mode = 1,
	    	.work_mode = ISP_WORK_MODE_NOMAL,
	    	.hdr_mode = HDR_MODE_NATIVE,
	    	.size = {
	    		.width = 1920,
	    		.height = 1080,
	    	},
	    	.frame_rate = 30,
	    	.isp_combine = {
	    		.isp_channel_mode = ISP_CHANNEL_MODE_NORMAL,
	    		.bind_channel = {
	    			.bind_hw_id = 0,
	    			.bind_slot_id = 0,
	    		},
	    	},
	    	.isp_sw_ctrl = {
	    		.ae_stat_buf_en = 1,
	    		.awb_stat_buf_en = 1,
	    		.ae5bin_stat_buf_en = 1,
	    		.ctx_buf_en = 0,
	    		.pixel_consistency_en = 0
	    	},
	    	.algo_state = 1,
	    	.clear_record = 0,
	    },
	    .ochn_attr = {
	    	.stream_output_mode = STREAM_OUTPUT_MODE_ENABLE,
	    	.axi_output_mode = AXI_OUTPUT_MODE_YUV420,
	    	.output_crop_cfg = {
	    		.enable = HB_FALSE,
	    		.rect = {
	    			.x = 0,
	    			.y = 0,
	    			.width = 0,
	    			.height = 0,
	    		},
	    	},
	    	.output_raw_level = ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA,
	    	.out_buf_noinvalid = 1,
	    	.out_buf_noncached = 0,
	    	.buf_num = 3,
	    },
	    .ichn_attr = {
	    	.input_crop_cfg = {
	    		.enable = HB_FALSE,
	    		.rect = {
	    			.x = 0,
	    			.y = 0,
	    			.width = 0,
	    			.height = 0,
	    		},
	    	},
	    	.in_buf_noclean = 1,
	    	.in_buf_noncached = 0,
	    },
	},
	[1] = {
		.isp_attr = {
			.channel = {
			  .hw_id = 1,
			  .slot_id = 5,
			  .ctx_id = -1,
			},
			.sched_mode = 1,
			.work_mode = ISP_WORK_MODE_NOMAL,
			.hdr_mode = HDR_MODE_NATIVE,
			.size = {
			  .width = 1920,
			  .height = 1080,
			},
			.frame_rate = 30,
			.isp_combine = {
			  .isp_channel_mode = ISP_CHANNEL_MODE_NORMAL,
			  .bind_channel = {
				.bind_hw_id = 0,
				.bind_slot_id = 0,
			  }
			},
			.isp_sw_ctrl = {
			  .ae_stat_buf_en = 1,
			  .awb_stat_buf_en = 1,
			  .ae5bin_stat_buf_en = 1,
			  .ctx_buf_en = 0,
			  .pixel_consistency_en = 0,
			},
			.algo_state = 1,
			.clear_record = 0,
		  },
		  .module_ctrl = {
			.isp_module_ctrl_reg1 = {
			  .u32Key = 0,
			},
			.isp_module_ctrl_reg2 = {
			  .u32Key = 0,
			},
		  },
		  .ochn_attr = {
			.stream_output_mode = STREAM_OUTPUT_MODE_ENABLE,
			.axi_output_mode = AXI_OUTPUT_MODE_YUV420,
			.output_crop_cfg = {
			  .enable = HB_FALSE,
			  .rect = {
				.x = 0,
				.y = 0,
				.width = 0,
				.height = 0,
			  },
			},
			.output_raw_level = ISP_OUTPUT_RAW_LEVEL_SENSOR_DATA,
			.out_buf_noinvalid = 1,
			.out_buf_noncached = 0,
			.buf_num = 3,
		  },
		  .ichn_attr = {
			.input_crop_cfg = {
			  .enable = HB_FALSE,
			  .rect = {
				.x = 0,
				.y = 0,
				.width = 0,
				.height = 0,
			  }
			},
			.in_buf_noclean = 1,
			.in_buf_noncached = 0,
		  }
	}
};

ynr_info_t ynr_info[] = {
	[0] = {
		.hw_id = 1,
		.link_mode = 1,
	    .slot_id = 4,
	    .ch_img_width = 1920,
	    .ch_img_height = 1080,
	    .nr2d_en = 1,
	    .nr3d_en = 1,
	    .debug_en = 0,
	},
	[1] = {
		.hw_id = 1,
		.link_mode = 1,
		.slot_id = 5,
		.ch_img_width = 1920,
		.ch_img_height = 1080,
		.nr2d_en = 1,
		.nr3d_en = 1,
		.debug_en = 0,
	},
};

pym_cfg_t pym_cfg[] = {
	[0] = {
		.hw_id = 1,
	    .pym_mode = 1,
	    .slot_id = 4,
	    .axi_burst_len = 0,
	    .in_linebuff_watermark = 0,
	    .out_buf_noinvalid = 1,
	    .out_buf_noncached = 0,
	    .in_buf_noclean = 1,
	    .in_buf_noncached = 0,
	    .buf_consecutive = 0,
	    .pingpong_ring = 0,
	    .output_buf_num = 6,
	    .timeout = 0,
	    .threshold_time = 0,
	    .layer_num_trans_next = 0,
	    .layer_num_share_prev = -1,
	    .chn_ctrl = {
	    	.pixel_num_before_sol = 2,
	    	.invalid_head_lines = 0,
	    	.src_in_width = 1920,
	    	.src_in_height = 1080,
	    	.src_in_stride_y = 1920,
	    	.src_in_stride_uv = 1920,
	    	.suffix_hb_val = 100,
	    	.prefix_hb_val = 2,
	    	.suffix_vb_val = 10,
	    	.prefix_vb_val = 0,
	    	.bl_max_layer_en = 5,
	    	.ds_roi_en = 1,
	    	.ds_roi_uv_bypass = 0,
	    	//.ds_roi_sel = "\000\000\000\000\000",
	    	//.ds_roi_layer = "\000\000\000\000\000",
	    	.ds_roi_info = {
	    		[0] = {
	    			.start_top = 0,
	    			.start_left = 0,
	    			.region_width = 1920,
	    			.region_height = 1080,
	    			.wstride_uv = 1920,
	    			.wstride_y = 1920,
	    			.vstride = 1080,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 1920,
	    			.out_height = 1080,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		},
	    		[1] = {
	    			.start_top = 20,
	    			.start_left = 16,
	    			.region_width = 900,
	    			.region_height = 600,
	    			.wstride_uv = 700,
	    			.wstride_y = 700,
	    			.vstride = 500,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 700,
	    			.out_height = 500,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		},
	    		[2] = {
	    			.start_top = 8,
	    			.start_left = 48,
	    			.region_width = 720,
	    			.region_height = 480,
	    			.wstride_uv = 640,
	    			.wstride_y = 640,
	    			.vstride = 320,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 640,
	    			.out_height = 320,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		},
	    		[3] = {
	    			.start_top = 18,
	    			.start_left = 24,
	    			.region_width = 520,
	    			.region_height = 450,
	    			.wstride_uv = 320,
	    			.wstride_y = 320,
	    			.vstride = 160,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 320,
	    			.out_height = 160,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		},
	    		[4] = {
	    			.start_top = 0,
	    			.start_left = 0,
	    			.region_width = 520,
	    			.region_height = 450,
	    			.wstride_uv = 512,
	    			.wstride_y = 512,
	    			.vstride = 240,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 512,
	    			.out_height = 240,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		},
	    		[5] = {
	    			.start_top = 0,
	    			.start_left = 10,
	    			.region_width = 520,
	    			.region_height = 450,
	    			.wstride_uv = 122,
	    			.wstride_y = 122,
	    			.vstride = 32,
	    			.step_v = 0,
	    			.step_h = 0,
	    			.out_width = 122,
	    			.out_height = 32,
	    			.phase_y_v = 0,
	    			.phase_y_h = 0,
	    		}
	    	},
	    	.pre_int_set_y = {
	    		[0] = 0,
	    		[1] = 0,
	    		[2] = 0,
	    		[3] = 0,
	    		[4] = 0,
	    		[5] = 0,
	    		[6] = 0,
	    		[7] = 0,
	    	},
	    	.pre_int_set_uv = {
	    		[0] = 0,
	    		[1] = 0,
	    		[2] = 0,
	    		[3] = 0,
	    		[4] = 0,
	    		[5] = 0,
	    		[6] = 0,
	    		[7] = 0,
	    	}
	    },
	    .fb_buf_num = 2,
	    //.reserved = {
	    //	0, 0, 0, 0, 0, 0
	    //},
	    .magicNumber = 0x12345678,
	},
	[1] = {
		.hw_id = 1,
		.pym_mode = 1,
		.slot_id = 5,
		.axi_burst_len = 0,
		.in_linebuff_watermark = 0,
		.out_buf_noinvalid = 1,
		.out_buf_noncached = 0,
		.in_buf_noclean = 1,
		.in_buf_noncached = 0,
		.buf_consecutive = 0,
		.pingpong_ring = 0,
		.output_buf_num = 6,
		.timeout = 0,
		.threshold_time = 0,
		.layer_num_trans_next = 0,
		.layer_num_share_prev = -1,
		.chn_ctrl = {
		  .pixel_num_before_sol = 2,
		  .invalid_head_lines = 0,
		  .src_in_width = 1920,
		  .src_in_height = 1080,
		  .src_in_stride_y = 1920,
		  .src_in_stride_uv = 1920,
		  .suffix_hb_val = 100,
		  .prefix_hb_val = 2,
		  .suffix_vb_val = 10,
		  .prefix_vb_val = 0,
		  .bl_max_layer_en = 5,
		  .ds_roi_en = 1,
		  .ds_roi_uv_bypass = 0,
		 //ds_roi_sel = "\000\000\000\000\000",
		 //ds_roi_layer = "\000\000\000\000\000",
		  .ds_roi_info = {{
			  .start_top = 0,
			  .start_left = 0,
			  .region_width = 1920,
			  .region_height = 1080,
			  .wstride_uv = 1920,
			  .wstride_y = 1920,
			  .vstride = 1080,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 1920,
			  .out_height = 1080,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}, {
			  .start_top = 20,
			  .start_left = 16,
			  .region_width = 900,
			  .region_height = 600,
			  .wstride_uv = 700,
			  .wstride_y = 700,
			  .vstride = 500,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 700,
			  .out_height = 500,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}, {
			  .start_top = 8,
			  .start_left = 48,
			  .region_width = 720,
			  .region_height = 480,
			  .wstride_uv = 640,
			  .wstride_y = 640,
			  .vstride = 320,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 640,
			  .out_height = 320,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}, {
			  .start_top = 18,
			  .start_left = 24,
			  .region_width = 520,
			  .region_height = 450,
			  .wstride_uv = 320,
			  .wstride_y = 320,
			  .vstride = 160,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 320,
			  .out_height = 160,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}, {
			  .start_top = 0,
			  .start_left = 0,
			  .region_width = 520,
			  .region_height = 450,
			  .wstride_uv = 512,
			  .wstride_y = 512,
			  .vstride = 240,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 512,
			  .out_height = 240,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}, {
			  .start_top = 0,
			  .start_left = 10,
			  .region_width = 520,
			  .region_height = 450,
			  .wstride_uv = 122,
			  .wstride_y = 122,
			  .vstride = 32,
			  .step_v = 0,
			  .step_h = 0,
			  .out_width = 122,
			  .out_height = 32,
			  .phase_y_v = 0,
			  .phase_y_h = 0,
			}},
		  .pre_int_set_y = {0, 0, 0, 0, 0, 0, 0, 0},
		  .pre_int_set_uv = {0, 0, 0, 0, 0, 0, 0, 0},
		},
		.fb_buf_num = 2,
		.reserved = {0, 0, 0, 0, 0, 0},
		.magicNumber = 0x12345678,
	},
};

struct stitch_base_attr base_attr;
struct stitch_base_attr sth_base_attr = {
		  .mode = 2,
		  .roi_nums = 2,
		  .img_nums = 2,
		  .alpha_lut = {
			.share_id = 0,
			.vaddr = 0,
			.offset = 0,
			.size = 0
		  },
		  .beta_lut = {
			.share_id = 0,
			.vaddr = 0,
			.offset = 0,
			.size = 0
		  },
		  .blending = {{
			  .roi_index = 0,
			  .blending_mode = 3,
			  .direct = 0,
			  .uv_en = 1,
			  .src0_index = 0,
			  .src1_index = 1,
			  .margin = 0,
			  .margin_inv = 128,
			  .gain_src0_yuv = {256, 256, 256},
			  .gain_src1_yuv = {256, 256, 256}
			}, {
			  .roi_index = 1,
			  .blending_mode = 3,
			  .direct = 0,
			  .uv_en = 1,
			  .src0_index = 1,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 128,
			  .gain_src0_yuv = {256, 256, 256},
			  .gain_src1_yuv = {256, 256, 256}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}, {
			  .roi_index = 0,
			  .blending_mode = 0,
			  .direct = 0,
			  .uv_en = 0,
			  .src0_index = 0,
			  .src1_index = 0,
			  .margin = 0,
			  .margin_inv = 0,
			  .gain_src0_yuv = {0, 0, 0},
			  .gain_src1_yuv = {0, 0, 0}
			}}
};
struct stitch_ch_attr sth_inch_attr[] = {
           {
			.width = 1920,
			.height = 1080,
			.strid = {1920, 1920},
			.rois = {{
				.roi_index = 0,
				.roi_x = 0,
				.roi_y = 0,
				.roi_w = 1920,
				.roi_h = 1080
			  }, {
				.roi_index = 1,
				.roi_x = 0,
				.roi_y = 0,
				.roi_w = 1920,
				.roi_h = 1080
			  },
			}
		  }, {
			.width = 1920,
			.height = 1080,
			.strid = {1920, 1920},
			.rois = {{
				.roi_index = 0,
				.roi_x = 0,
				.roi_y = 0,
				.roi_w = 1920,
				.roi_h = 1080
			  }, {
				.roi_index = 1,
				.roi_x = 0,
				.roi_y = 0,
				.roi_w = 1920,
				.roi_h = 1080
			  },
			}
		  }, {
			.width = 0,
			.height = 0,
			.strid = {0, 0},
		  }, {
			.width = 0,
			.height = 0,
			.strid = {0, 0},
		  }
};
struct stitch_ch_attr sth_och_attr = {
		  .width = 1920,
		  .height = 2160,
		  .strid = {1920, 1920},
		  .rois = {{
			  .roi_index = 0,
			  .roi_x = 0,
			  .roi_y = 0,
			  .roi_w = 1920,
			  .roi_h = 1080
			}, {
			  .roi_index = 1,
			  .roi_x = 0,
			  .roi_y = 1080,
			  .roi_w = 1920,
			  .roi_h = 1080
			},
		  }
};

mipi_config_t mipi_config[] = {
	[0] = {
		.rx_enable = 1,
		.rx_attr = {
		  .phy = 0,
		  .lane = 2,
		  .datatype = 299,
		  .fps = 0,
		  .mclk = 24,
		  .mipiclk = 1728,
		  .width = 0,
		  .height = 0,
		  .linelenth = 0,
		  .framelenth = 0,
		  .settle = 0,
		  .ppi_pg = 0,
		  .hsaTime = 0,
		  .hbpTime = 0,
		  .hsdTime = 0,
		  .channel_num = 0,
		  .channel_sel = {0, 0, 0, 0}
		},
		.rx_ex_mask = 64,
		.rx_attr_ex = {
		  .nocheck = 0,
		  .notimeout = 0,
		  .wait_ms = 0,
		  .dbg_value = 0,
		  .adv_value = 0,
		  .need_stop_check = 0,
		  .stop_check_instart = 1,
		  .cut_through = 0,
		  .mem_flush = 0,
		  .data_ids_1 = 0,
		  .data_ids_2 = 0,
		  .data_ids_vc1 = 0,
		  .data_ids_vc2 = 0,
		  .ipi_16bit = 0,
		  .ipi_force = 0,
		  .ipi_limit = 0,
		  .ipi_overst = 0,
		  .pkt2pkt_time = 0,
		  .snrclk_en = 0,
		  .snrclk_freq = 0,
		  .vcext_en = 0,
		  .error_diag = 0,
		  .ipi1_dt = 0,
		  .ipi2_dt = 0,
		  .ipi3_dt = 0,
		  .ipi4_dt = 0,
		  .cfg_nocheck = 0,
		  .drop_func = 0,
		  .drop_mask = 0,
		  .irq_cnt = 0,
		  .irq_debug = 0,
		  .fatal_ap = 0
		},
		.bypass = 0x0,
		.end_flag = 0x433600c8,
	},
	[1] = {
		.rx_enable = 1,
		.rx_attr = {
		  .phy = 0,
		  .lane = 2,
		  .datatype = 299,
		  .fps = 0,
		  .mclk = 24,
		  .mipiclk = 1728,
		  .width = 0,
		  .height = 0,
		  .linelenth = 0,
		  .framelenth = 0,
		  .settle = 0,
		  .ppi_pg = 0,
		  .hsaTime = 0,
		  .hbpTime = 0,
		  .hsdTime = 0,
		  .channel_num = 0,
		  .channel_sel = {0, 0, 0, 0}
		},
		.rx_ex_mask = 64,
		.rx_attr_ex = {
		  .nocheck = 0,
		  .notimeout = 0,
		  .wait_ms = 0,
		  .dbg_value = 0,
		  .adv_value = 0,
		  .need_stop_check = 0,
		  .stop_check_instart = 1,
		  .cut_through = 0,
		  .mem_flush = 0,
		  .data_ids_1 = 0,
		  .data_ids_2 = 0,
		  .data_ids_vc1 = 0,
		  .data_ids_vc2 = 0,
		  .ipi_16bit = 0,
		  .ipi_force = 0,
		  .ipi_limit = 0,
		  .ipi_overst = 0,
		  .pkt2pkt_time = 0,
		  .snrclk_en = 0,
		  .snrclk_freq = 0,
		  .vcext_en = 0,
		  .error_diag = 0,
		  .ipi1_dt = 0,
		  .ipi2_dt = 0,
		  .ipi3_dt = 0,
		  .ipi4_dt = 0,
		  .cfg_nocheck = 0,
		  .drop_func = 0,
		  .drop_mask = 0,
		  .irq_cnt = 0,
		  .irq_debug = 0,
		  .fatal_ap = 0
		},
		.bypass = 0x0,
		.end_flag = 0x433600c8,
	}
};
camera_config_t cam_cfg[] = {
	[0] = {
		.name = "imx219",
		.addr = 0x10,
		.isp_addr = 0x0,
		.eeprom_addr = 0x51,
		.serial_addr = 0x40,
		.sensor_mode = 0x1,
		.sensor_clk = 0,
		.gpio_enable = 0,
		.gpio_level = 0,
		.bus_select = 0,
		.bus_timeout = 0,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.format = 0,
		.flags = 0,
		.extra_mode = 0,
		.config_index = 0,
		.ts_compensate = 0,
		.mipi_cfg = &mipi_config[0],
		.calib_lname = "lib_imx219_linear.so",
		.sensor_param = 0x0,
		.iparam_mode = 0,
		.end_flag = 0x43310140,
	},
	[1] = {
		.name = "imx219",
		.addr = 0x10,
		.isp_addr = 0x0,
		.eeprom_addr = 0x51,
		.serial_addr = 0x40,
		.sensor_mode = 0x1,
		.sensor_clk = 0,
		.gpio_enable = 0,
		.gpio_level = 0,
		.bus_select = 0,
		.bus_timeout = 0,
		.fps = 30,
		.width = 1920,
		.height = 1080,
		.format = 0,
		.flags = 0,
		.extra_mode = 0,
		.config_index = 0,
		.ts_compensate = 0,
		.mipi_cfg = &mipi_config[1],
		.calib_lname = "lib_imx219_linear.so",
		.sensor_param = 0x0,
		.iparam_mode = 0,
		.end_flag = 0x43310140,
	}
};

media_codec_context_t context;
hbn_vflow_handle_t vflow_fd[PIPELINE_NUM] = {0};
hbn_vnode_handle_t vin_vnode_fd[PIPELINE_NUM] = {0};
hbn_vnode_handle_t isp_vnode_fd[PIPELINE_NUM] = {0};
hbn_vnode_handle_t ynr_vnode_fd[PIPELINE_NUM] = {0};
hbn_vnode_handle_t pym_vnode_fd[PIPELINE_NUM] = {0};
hbn_vnode_handle_t sth_vnode_fd;
camera_handle_t cam_vnode_fd[PIPELINE_NUM] = {0};
hb_mem_common_buf_t alpha_buffer = {0};
hb_mem_common_buf_t beta_buffer = {0};
imgframe_t imgframe;
char name[64] = { 0 };
int32_t sth_fd;

hbn_vnode_handle_t vin_vnode_create(vin_attr_t *vin_attr)
{
    int32_t ret = 0;
    uint32_t hw_id;
    vin_ichn_attr_t *vin_ichn_attr;
    vin_ochn_attr_t *vin_ochn_attr;
    hbn_vnode_handle_t vnode_magic_id;
    hbn_buf_alloc_attr_t alloc_attr;

    hw_id = vin_attr->vin_node_attr.cim_attr.mipi_rx;
    vin_ochn_attr = &vin_attr->vin_ochn_attr[VIN_MAIN_FRAME];
    vin_ichn_attr = &vin_attr->vin_ichn_attr;

    ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, &vnode_magic_id);
    ERR_CON_EQ(ret, 0);

    ret = hbn_vnode_set_attr(vnode_magic_id, vin_attr);
    ERR_CON_EQ(ret, 0);

    ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, vin_ichn_attr);
    ERR_CON_EQ(ret, 0);

    ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, vin_ochn_attr);
    ERR_CON_EQ(ret, 0);

    memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
    alloc_attr.buffers_num = vin_attr->vin_ochn_buff_attr[VIN_MAIN_FRAME].buffers_num;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED;
    ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
    ERR_CON_EQ(ret, 0);

    return vnode_magic_id;
}

hbn_vnode_handle_t isp_vnode_create(isp_cfg_t *isp_config)
{
	int32_t ret;
	hbn_vnode_handle_t vnode_magic_id;
	uint32_t hw_id = isp_cfg->isp_attr.channel.hw_id;
	uint32_t slot_id = isp_cfg->isp_attr.channel.slot_id;
	int32_t ctx_id = isp_cfg->isp_attr.channel.ctx_id;
    uint32_t buf_num;
    hbn_buf_alloc_attr_t alloc_attr = { 0 };

	if ((hw_id > ISP_IP_MAX) || (slot_id >= ISP_IP_SENSOR_SLOTS)) {
		printf("hw_id(%d) or slot_id(%d) invaild.\n", hw_id, slot_id);
		return -HBN_STATUS_ISP_INVALID_PARAMETER;
    }

	ret = hbn_vnode_open(HB_ISP, hw_id, ctx_id, &vnode_magic_id);
	if (ret < 0) {
	    printf("[hw%d][slot%d]HB_ISP open failed ret = %d\n", hw_id, slot_id, ret);
	    return -HBN_STATUS_ISP_NODE_UNEXIST;
	}

	ret = hbn_vnode_set_attr(vnode_magic_id, &isp_config->isp_attr);
	if (ret < 0) {
	    printf("[hw%d][slot%d]HB_ISP set_attr failed ret = %d\n", hw_id, slot_id, ret);
	    hbn_vnode_close(vnode_magic_id);
	    return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}

	ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, &isp_config->ichn_attr);
	if (ret < 0) {
	    printf("[hw%d][slot%d]HB_ISP set_ichn_attr failed ret = %d\n", hw_id, slot_id, ret);
	    hbn_vnode_close(vnode_magic_id);
	    return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}

	ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, &isp_config->ochn_attr);
	if (ret < 0) {
	    printf("[hw%d][slot%d]HB_ISP set_ochn_attr failed ret = %d\n", hw_id, slot_id, ret);
	    hbn_vnode_close(vnode_magic_id);
	    return -HBN_STATUS_ISP_ILLEGAL_ATTR;
	}

	buf_num = isp_cfg->ochn_attr.buf_num;
	if (buf_num > 0u) {
	    alloc_attr.buffers_num = buf_num;
	    alloc_attr.is_contig = 1;
	    if (isp_cfg->ochn_attr.out_buf_noncached == 0u)
	        alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
	                     (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);
	    else
	        alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
	                     (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN);
	    ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
	    if (ret < 0) {
	        printf("[hw%d][slot%d]HB_ISP set_ochn_buf_attr failed ret = %d\n", hw_id, slot_id, ret);
	        hbn_vnode_close(vnode_magic_id);
	        return -HBN_STATUS_ISP_INVALID_PARAMETER;
	    }
	}

    printf("[hw%d][slot%d]done alloc_attr.flags = %ld, cfg size %ld\n", hw_id, slot_id, alloc_attr.flags, sizeof(isp_cfg_t));
    return vnode_magic_id;
}

hbn_vnode_handle_t ynr_vnode_create(ynr_info_t *info)
{
    int32_t ret = 0;
    hobot_status status;
    hbn_vnode_handle_t vnode_magic_id;

    struct ynr_init_attr attr;
    struct hobot_ynr_channel_output_config channel_output_cfg = {0};
    struct hobot_ynr_channel_input_config channel_input_cfg = {0};
    hbn_buf_alloc_attr_t alloc_attr;

    if (info  == NULL) {
        printf("%s YNR configuration is empty.", __func__);
        return -1;
    }
    memset(&attr, 0, sizeof(struct ynr_init_attr));

    attr.work_mode = info->link_mode;
    attr.height = info->ch_img_height;
    attr.width = info->ch_img_width;
    attr.slot_id = info->slot_id;
    attr.debug_en = info->debug_en;
    attr.nr3d_en = info->nr3d_en;
    attr.nr2d_en = info->nr2d_en;
    attr.nr_static_switch = (attr.nr3d_en << 1) | (attr.nr2d_en);

    attr.in_stride[0] = info->ch_img_width;
    attr.in_stride[1] = info->ch_img_width;

    attr.dma_output_en = attr.nr3d_en;

    ret = hbn_vnode_open(HB_YNR, info->hw_id, AUTO_ALLOC_ID, &vnode_magic_id);
    if (ret < 0) {
        printf("YNR hbn_vnode_open failed.\n");
        return -1;
    } else {
        printf("YNR hbn_vnode_open succeed.\n");
    }

    status = hbn_vnode_set_attr(vnode_magic_id, &attr);
    if (status < 0) {
        printf("YNR hbn_vnode_set_attr failed.\n");
        goto error;
    } else {
        printf("YNR hbn_vnode_set_attr succeed.\n");
    }

    ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, &channel_input_cfg);
    if (ret < 0) {
        printf("YNR hbn_vnode_set_ichn_attr failed.\n");
        goto error;
    } else {
        printf("YNR hbn_vnode_set_ichn_attr succeed.\n");
    }
    ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 1, &channel_input_cfg);
    if (ret < 0) {
        printf("YNR hbn_vnode_set_ichn_attr failed.\n");
        goto error;
    } else {
        printf("YNR hbn_vnode_set_ichn_attr succeed.\n");
    }

    ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, &channel_output_cfg);
    if (ret < 0) {
        printf("YNR hbn_vnode_set_ochn_attr failed.\n");
        goto error;
    } else {
        printf("YNR hbn_vnode_set_ochn_attr succeed.\n");
    }

    if (attr.nr3d_en == 1u) {
        /* Apply for comparison frame buffer for YNR */
        alloc_attr.buffers_num = YNR_ALLOC_BUFFER_NUM;
        alloc_attr.is_contig = 1;
        alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
            (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_CACHED);
        ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
        if (ret < 0) {
            printf("YNR hbn_vnode_set_ochn_buf_attr failed.\n");
            return ret;
        }
    }

    return vnode_magic_id;
error:
    hbn_vnode_close(vnode_magic_id);
    return -1;
}
hbn_vnode_handle_t pym_vnode_create(pym_cfg_t *pym_config)
{
    int32_t ret = 0;
    hbn_vnode_handle_t vnode_magic_id;
    hbn_buf_alloc_attr_t alloc_attr;

    ret = hbn_vnode_open(HB_PYM, pym_config->hw_id, AUTO_ALLOC_ID, &vnode_magic_id);
    if (ret < 0) {
        goto err;
    }

    ret = hbn_vnode_set_attr(vnode_magic_id, pym_config);
    if (ret < 0) {
        goto err1;
    }

    ret = hbn_vnode_set_ichn_attr(vnode_magic_id, 0, pym_config);
    if (ret < 0) {
        goto err1;
    }

    ret = hbn_vnode_set_ochn_attr(vnode_magic_id, 0, pym_config);
    if (ret < 0) {
        goto err1;
    }

    if (pym_config->output_buf_num > 0u) {
        memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
        alloc_attr.buffers_num = pym_config->output_buf_num;
        alloc_attr.is_contig = 1;
        alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN | (uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN);
        if (pym_config->out_buf_noncached == 0u) {
            alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;
        }
        ret = hbn_vnode_set_ochn_buf_attr(vnode_magic_id, 0, &alloc_attr);
        if (ret < 0) {
            goto err1;
        }
    }
    printf("done cfg size %ld\n", sizeof(pym_cfg_t));

    return vnode_magic_id;
err1:
    hbn_vnode_close(vnode_magic_id);
err:
    return ret;
}

hbn_vnode_handle_t sth_vnode_create(void)
{
    int32_t ret = 0, i;
    hbn_buf_alloc_attr_t alloc_attr = {0};
	hbn_vnode_handle_t sth_handle;

    ret = hbn_vnode_open(HB_STITCH, 0, -1, &sth_handle);
    if (ret < 0) {
    	printf("STH vnode open fail\n");
    	return -1;
    }

    ret = hbn_vnode_set_attr(sth_handle, &sth_base_attr);
    if (ret < 0) {
    	printf("STH vnode set attr fail\n");
    	return -1;
    }

    for (i = 0; i < 2; i++) {
    	ret = hbn_vnode_set_ichn_attr(sth_handle, i, &sth_inch_attr[i]);
    	if (ret < 0) {
    		printf("STH vnode set ichn attr fail\n");
    		return -1;
    	}
    }

    ret = hbn_vnode_set_ochn_attr(sth_handle, 0, &sth_och_attr);
    if (ret < 0) {
    	printf("STH vnode set ochn attr fail\n");
    	return -1;
    }

    memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
    alloc_attr.buffers_num = 3;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
    		(uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_MAP_INITIALIZED);
    alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;

    ret = hbn_vnode_set_ochn_buf_attr(sth_handle, 0, &alloc_attr);
    if (ret < 0) {
    	printf("STH vnode set ochn buf attr fail\n");
    	return -1;
    }

	return sth_handle;
}

int32_t codec_set_input(media_codec_context_t *context, imgframe_t *frame)
{
	int32_t ret = 0;
	media_codec_buffer_t *in_buffer = NULL;
	hbn_vnode_image_t *vnode_buffer = NULL;

	if ((context == NULL) || (frame == NULL))
	{
		printf("codec param is NULL!\n");
		return -1;
	}

	in_buffer = &(frame->in_buffer);
	vnode_buffer = &(frame->vnode_buffer);

	memset(in_buffer, 0x00, sizeof(media_codec_buffer_t));
	ret = hb_mm_mc_dequeue_input_buffer(context, in_buffer, 3000);
	if (ret == (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
		printf("%s: Timeout to dequeue input buffer.\n", __FUNCTION__);
		return -1;
	} else if (ret < 0) {
		printf("%s: dequeue input buffer fail(%d).\n", __FUNCTION__, ret);
		return -1;
	}

	in_buffer->vframe_buf.vir_ptr[0] = (hb_u8 *)vnode_buffer->buffer.virt_addr[0];
	in_buffer->vframe_buf.vir_ptr[1] = (hb_u8 *)vnode_buffer->buffer.virt_addr[1];
	in_buffer->vframe_buf.phy_ptr[0] = vnode_buffer->buffer.phys_addr[0];
	in_buffer->vframe_buf.phy_ptr[1] = vnode_buffer->buffer.phys_addr[1];
	in_buffer->vframe_buf.fd[0] = vnode_buffer->buffer.fd[0];
	in_buffer->vframe_buf.stride = vnode_buffer->buffer.stride;

	ret = hb_mm_mc_queue_input_buffer(context, in_buffer, 100);
	if (ret != 0)
	{
		printf("hb_mm_mc_queue_input_buffer failed, ret = 0x%x\n", ret);
		return -1;
	}

	printf("%s idx: %d, %s successful\n", context->encoder ? "Encode" : "Decode", context->instance_index, __func__);
	return 0;
}

int32_t codec_get_output(media_codec_context_t *context, imgframe_t *frame)
{
	int32_t ret = 0;
	media_codec_output_buffer_info_t *info = NULL;
	media_codec_buffer_t *out_buffer = NULL;

	if ((context == NULL) || (frame == NULL))
	{
		printf("codec param is NULL");
		return -1;
	}
	out_buffer = &(frame->out_buffer);
	info = &(frame->info);

	memset(out_buffer, 0x00, sizeof(media_codec_buffer_t));
	memset(info, 0x00, sizeof(media_codec_output_buffer_info_t));

	ret = hb_mm_mc_dequeue_output_buffer(context, out_buffer, info, 3000);
	if (ret != 0) {
        if (ret == (int32_t)HB_MEDIA_ERR_WAIT_TIMEOUT) {
            printf("dequeue input buffer timeout(%d).\n", ret);
		}
		printf("dequeue input buffer fail(%d).\n", ret);
		return -1;
	}

	return 0;
}

int32_t codec_release_output(media_codec_context_t *context, imgframe_t *frame)
{
	int32_t ret = 0;
	media_codec_buffer_t *out_buffer = NULL;

	if ((context == NULL) || (frame == NULL))
	{
		printf("codec param is NULL!\n");
		return -1;
	}
	out_buffer = &(frame->out_buffer);

    ret = hb_mm_mc_queue_output_buffer(context, out_buffer, 100);
	if (out_buffer->vstream_buf.stream_end) {
		printf("There is no more output data!\n");
	} else {
		printf("There is more output data!\n");
	}

	if (ret) {
		printf("hb_mm_mc_queue_output_buffer error!!!\n");
		return -1;
	}

	printf("%s idx: %d, %s successful\n", context->encoder ? "Encode" : "Decode", context->instance_index, __func__);
	return 0;
}


int32_t codec_deinit(media_codec_context_t *context)
{
	int32_t ret = 0;

	ret = hb_mm_mc_release(context);
	if (ret != 0)
	{
		printf("Failed to hb_mm_mc_release ret = %d \n", ret);
		return -1;
	}

	printf("%s idx: %d, %s successful\n", context->encoder ? "Encode" : "Decode", context->instance_index, __func__);
	return 0;
}

int32_t codec_stop(media_codec_context_t *context)
{
	int32_t ret = 0;
	ret = hb_mm_mc_pause(context);
	if (ret != 0)
	{
		printf("Failed to hb_mm_mc_pause ret = %d \n", ret);
		return -1;
	}

	printf("%s idx: %d, %s successful\n", context->encoder ? "Encode" : "Decode", context->instance_index, __func__);
	return 0;
}

int32_t codec_start(media_codec_context_t *context)
{
	int32_t ret = 0;
	mc_av_codec_startup_params_t startup_params = {0};

	ret = hb_mm_mc_start(context, &startup_params);
	if (ret != 0)
	{
		printf("%s:%d hb_mm_mc_start failed.\n", __FUNCTION__, __LINE__);
		return -1;
	}

	printf("%s idx: %d, %s successful\n", context->encoder ? "Encode" : "Decode", context->instance_index, __func__);
	return ret;
}

int32_t codec_init(media_codec_context_t *context)
{
	int32_t ret = 0;

	ret = hb_mm_mc_initialize(context);
	if (0 != ret)
	{
		printf("hb_mm_mc_initialize failed.\n");
		return -1;
	}

	ret = hb_mm_mc_configure(context);
	if (0 != ret)
	{
		printf("hb_mm_mc_configure failed.\n");
		hb_mm_mc_release(context);
		return -1;
	}

	return 0;
}

int get_rc_params(media_codec_context_t *context, mc_rate_control_params_t *rc_params)
{
	int ret = 0;
	ret = hb_mm_mc_get_rate_control_config(context, rc_params);
	if (ret != 0) {
		printf("Failed to get rc params ret=0x%x\n", ret);
		return -1;
	}
	switch (rc_params->mode) {
	case MC_AV_RC_MODE_H264CBR:
		rc_params->h264_cbr_params.intra_period = 30;
		rc_params->h264_cbr_params.intra_qp = 30;
		rc_params->h264_cbr_params.bit_rate = 5000;
		rc_params->h264_cbr_params.frame_rate = 30;
		rc_params->h264_cbr_params.initial_rc_qp = 20;
		rc_params->h264_cbr_params.vbv_buffer_size = 20;
		rc_params->h264_cbr_params.mb_level_rc_enalbe = 1;
		rc_params->h264_cbr_params.min_qp_I = 8;
		rc_params->h264_cbr_params.max_qp_I = 50;
		rc_params->h264_cbr_params.min_qp_P = 8;
		rc_params->h264_cbr_params.max_qp_P = 50;
		rc_params->h264_cbr_params.min_qp_B = 8;
		rc_params->h264_cbr_params.max_qp_B = 50;
		rc_params->h264_cbr_params.hvs_qp_enable = 1;
		rc_params->h264_cbr_params.hvs_qp_scale = 2;
		rc_params->h264_cbr_params.max_delta_qp = 10;
		rc_params->h264_cbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H264VBR:
		rc_params->h264_vbr_params.intra_qp = 20;
		rc_params->h264_vbr_params.intra_period = 30;
		rc_params->h264_vbr_params.intra_qp = 35;
		break;
	case MC_AV_RC_MODE_H264AVBR:
		rc_params->h264_avbr_params.intra_period = 15;
		rc_params->h264_avbr_params.intra_qp = 25;
		rc_params->h264_avbr_params.bit_rate = 2000;
		rc_params->h264_avbr_params.vbv_buffer_size = 3000;
		rc_params->h264_avbr_params.min_qp_I = 15;
		rc_params->h264_avbr_params.max_qp_I = 50;
		rc_params->h264_avbr_params.min_qp_P = 15;
		rc_params->h264_avbr_params.max_qp_P = 45;
		rc_params->h264_avbr_params.min_qp_B = 15;
		rc_params->h264_avbr_params.max_qp_B = 48;
		rc_params->h264_avbr_params.hvs_qp_enable = 0;
		rc_params->h264_avbr_params.hvs_qp_scale = 2;
		rc_params->h264_avbr_params.max_delta_qp = 5;
		rc_params->h264_avbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H264FIXQP:
		rc_params->h264_fixqp_params.force_qp_I = 23;
		rc_params->h264_fixqp_params.force_qp_P = 23;
		rc_params->h264_fixqp_params.force_qp_B = 23;
		rc_params->h264_fixqp_params.intra_period = 23;
		break;
	case MC_AV_RC_MODE_H264QPMAP:
		break;
	case MC_AV_RC_MODE_H265CBR:
		rc_params->h265_cbr_params.intra_period = 20;
		rc_params->h265_cbr_params.intra_qp = 30;
		rc_params->h265_cbr_params.bit_rate = 5000;
		rc_params->h265_cbr_params.frame_rate = 30;
		if (context->video_enc_params.width >= 480 ||
			context->video_enc_params.height >= 480) {
			rc_params->h265_cbr_params.initial_rc_qp = 30;
			rc_params->h265_cbr_params.vbv_buffer_size = 3000;
			rc_params->h265_cbr_params.ctu_level_rc_enalbe = 1;
		} else {
			rc_params->h265_cbr_params.initial_rc_qp = 20;
			rc_params->h265_cbr_params.vbv_buffer_size = 20;
			rc_params->h265_cbr_params.ctu_level_rc_enalbe = 1;
		}
		rc_params->h265_cbr_params.min_qp_I = 8;
		rc_params->h265_cbr_params.max_qp_I = 50;
		rc_params->h265_cbr_params.min_qp_P = 8;
		rc_params->h265_cbr_params.max_qp_P = 50;
		rc_params->h265_cbr_params.min_qp_B = 8;
		rc_params->h265_cbr_params.max_qp_B = 50;
		rc_params->h265_cbr_params.hvs_qp_enable = 1;
		rc_params->h265_cbr_params.hvs_qp_scale = 2;
		rc_params->h265_cbr_params.max_delta_qp = 10;
		rc_params->h265_cbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H265VBR:
		rc_params->h265_vbr_params.intra_qp = 20;
		rc_params->h265_vbr_params.intra_period = 30;
		rc_params->h265_vbr_params.intra_qp = 35;
		break;
	case MC_AV_RC_MODE_H265AVBR:
		rc_params->h265_avbr_params.intra_period = 15;
		rc_params->h265_avbr_params.intra_qp = 25;
		rc_params->h265_avbr_params.bit_rate = 2000;
		rc_params->h265_avbr_params.vbv_buffer_size = 3000;
		rc_params->h265_avbr_params.min_qp_I = 15;
		rc_params->h265_avbr_params.max_qp_I = 50;
		rc_params->h265_avbr_params.min_qp_P = 15;
		rc_params->h265_avbr_params.max_qp_P = 45;
		rc_params->h265_avbr_params.min_qp_B = 15;
		rc_params->h265_avbr_params.max_qp_B = 48;
		rc_params->h265_avbr_params.hvs_qp_enable = 0;
		rc_params->h265_avbr_params.hvs_qp_scale = 2;
		rc_params->h265_avbr_params.max_delta_qp = 5;
		rc_params->h265_avbr_params.qp_map_enable = 0;
		break;
	case MC_AV_RC_MODE_H265FIXQP:
		rc_params->h265_fixqp_params.force_qp_I = 23;
		rc_params->h265_fixqp_params.force_qp_P = 23;
		rc_params->h265_fixqp_params.force_qp_B = 23;
		rc_params->h265_fixqp_params.intra_period = 23;
		break;
	case MC_AV_RC_MODE_H265QPMAP:
		break;
	default:
		ret = HB_MEDIA_ERR_INVALID_PARAMS;
		break;
	}
	return ret;
}

int32_t codec_config_param(media_codec_context_t *context, media_codec_id_t codec_type, int32_t width, int32_t height)
{
	mc_video_codec_enc_params_t *params;

	memset(context, 0x00, sizeof(media_codec_context_t));

	context->codec_id = codec_type;
	context->encoder = TRUE;
	params = &context->video_enc_params;
	params->width = width;
	params->height = height;
	params->pix_fmt = MC_PIXEL_FORMAT_NV12;
	params->frame_buf_count = 5;
	params->external_frame_buf = TRUE;
	params->bitstream_buf_count = 5;
	if (context->codec_id == MEDIA_CODEC_ID_H264) {
		params->rc_params.mode = MC_AV_RC_MODE_H264CBR;
	} else if (context->codec_id == MEDIA_CODEC_ID_H265) {
		params->rc_params.mode = MC_AV_RC_MODE_H265CBR;
	} else if (context->codec_id == MEDIA_CODEC_ID_MJPEG) {
		params->rc_params.mode = MC_AV_RC_MODE_MJPEGFIXQP;
	}
	get_rc_params(context, &params->rc_params);

	params->gop_params.decoding_refresh_type = 2;
	params->gop_params.gop_preset_idx = 2;
	params->rot_degree = MC_CCW_0;
	params->mir_direction = MC_DIRECTION_NONE;
	params->frame_cropping_flag = FALSE;

    return 0;
}

int32_t dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1, uint32_t size, uint32_t size1) {
    FILE *yuvFd = NULL;
    char *buffer = NULL;

    yuvFd = fopen(filename, "w+");

    if (yuvFd == NULL) {
        printf("open(%s) fail", filename);
        return -1;
    }

    buffer = (char *)malloc(size + size1);

    if (buffer == NULL) {
        printf("ERR:malloc file");
        fclose(yuvFd);
        return -1;
    }

    memcpy(buffer, srcBuf, size);

	if (srcBuf1 != NULL && size1 != 0) {
    	memcpy(buffer + size, srcBuf1, size1);
	}

    fflush(stdout);

    fwrite(buffer, 1, size + size1, yuvFd);

    fflush(yuvFd);

    if (yuvFd != NULL)
        fclose(yuvFd);
    if (buffer != NULL)
        free(buffer);

    printf("filedump(%s, size(%d) is successed!!\n", filename, size + size1);

    return 0;
}

int write_output_h264(imgframe_t *frame, FILE *h264fd)
{
	size_t count;
	if(!h264fd || !frame) {
		printf("args is null\n");
		return -1;
    }

	media_codec_buffer_t *out_buffer = out_buffer = &(frame->out_buffer);

    fflush(stdout);

    count = fwrite(out_buffer->vstream_buf.vir_ptr, 1, out_buffer->vstream_buf.size, h264fd);
	if (count != out_buffer->vstream_buf.size) {
		printf("ferror error!!!\n");
	}

    fflush(h264fd);

	return 0;
}

int main() {
	int32_t ret = 0;
	int i = 0;
	FILE *h264fd = NULL;
	ret = hbn_vflow_create(&vflow_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_create[%d]:%d error\n", i, __LINE__);
		goto err;
	}

	ret = hbn_camera_create(&cam_cfg[i], &cam_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_camera_create[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	vin_vnode_fd[i] = vin_vnode_create(&vin_attr[i]);
	if (vin_vnode_fd[i] < 0) {
		ret = (int32_t)vin_vnode_fd[i];
		printf("vin_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], vin_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	isp_vnode_fd[i] = isp_vnode_create(&isp_cfg[i]);
	if (isp_vnode_fd[i] < 0) {
		ret = (int32_t)isp_vnode_fd[i];
		printf("isp_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], isp_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ynr_vnode_fd[i] = ynr_vnode_create(&ynr_info[i]);
	if (ynr_vnode_fd[i] < 0) {
		ret = (int32_t)ynr_vnode_fd[i];
		printf("ynr_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], ynr_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	pym_vnode_fd[i] = pym_vnode_create(&pym_cfg[i]);
	if (pym_vnode_fd[i] < 0) {
		ret = (int32_t)pym_vnode_fd[i];
		printf("pym_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], pym_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	sth_vnode_fd = sth_vnode_create();
	if (sth_vnode_fd < 0) {
		ret = (int32_t)sth_vnode_fd;
		printf("sth_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], sth_vnode_fd);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_camera_attach_to_vin(cam_vnode_fd[i], vin_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], vin_vnode_fd[i], 0, isp_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], isp_vnode_fd[i], 1, ynr_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], ynr_vnode_fd[i], 1, pym_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], pym_vnode_fd[i], 0, sth_vnode_fd, 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	i = 1;
	ret = hbn_vflow_create(&vflow_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_create[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_camera_create(&cam_cfg[i], &cam_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_camera_create[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	vin_vnode_fd[i] = vin_vnode_create(&vin_attr[i]);
	if (vin_vnode_fd[i] < 0) {
		ret = (int32_t)vin_vnode_fd[i];
		printf("vin_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], vin_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	isp_vnode_fd[i] = isp_vnode_create(&isp_cfg[i]);
	if (isp_vnode_fd[i] < 0) {
		ret = (int32_t)isp_vnode_fd[i];
		printf("isp_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], isp_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ynr_vnode_fd[i] = ynr_vnode_create(&ynr_info[i]);
	if (ynr_vnode_fd[i] < 0) {
		ret = (int32_t)ynr_vnode_fd[i];
		printf("ynr_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], ynr_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}
	pym_vnode_fd[i] = pym_vnode_create(&pym_cfg[i]);;
	if (pym_vnode_fd[i] < 0) {
		ret = (int32_t)pym_vnode_fd[i];
		printf("pym_vnode_init[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], pym_vnode_fd[i]);
	if (ret < 0) {
		printf("bn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_add_vnode(vflow_fd[i], sth_vnode_fd);
	if (ret < 0) {
		printf("hbn_vflow_add_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_camera_attach_to_vin(cam_vnode_fd[i], vin_vnode_fd[i]);
	if (ret < 0) {
		printf("hbn_camera_attach_to_vin[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], vin_vnode_fd[i], 0, isp_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], isp_vnode_fd[i], 1, ynr_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], ynr_vnode_fd[i], 1, pym_vnode_fd[i], 0);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	ret = hbn_vflow_bind_vnode(vflow_fd[i], pym_vnode_fd[i], 0, sth_vnode_fd, 1);
	if (ret < 0) {
		printf("hbn_vflow_bind_vnode[%d]:%d error\n", i, __LINE__);
		goto err1;
	}

	//config codec
	ret = codec_config_param(&context, MEDIA_CODEC_ID_H264, sth_och_attr.width, sth_och_attr.height);
	if (ret < 0) {
		printf("codec_config_param error!!!\n");
		goto err1;
	}

	ret = codec_init(&context);
	if (ret < 0) {
		printf("codec_init error!!!\n");
		goto err1;
	}

	ret = codec_start(&context);
	if (ret < 0) {
		printf("codec_init error!!!\n");
		goto err2;
	}

	h264fd = fopen(H264_FNAME, "w+");
    if (h264fd == NULL) {
        printf("open(%s) fail", H264_FNAME);
		ret = -1;
        goto err3;
    }

	ret = hbn_vflow_start(vflow_fd[0]);
	ret |= hbn_vflow_start(vflow_fd[1]);
	if (ret < 0) {
		printf("codec_init error!!!\n");
		goto err3;
	}

	while (imgframe.cnt < 30 * TIMEOUT) {
		ret = hbn_vnode_getframe(sth_vnode_fd, 0, 1000, &imgframe.vnode_buffer);
		printf("sth_worker, ret = %d\n", ret);
		if (ret == 0) {
			ret = codec_set_input(&context, &imgframe);
			if (ret < 0) {
				printf("codec_set_input error!!!\n");
				goto err4;
			}

			ret = codec_get_output(&context, &imgframe);
			if (ret < 0) {
				printf("codec_get_output error!!!\n");
				goto err4;
			}

            ret = write_output_h264(&imgframe, h264fd);
			if (ret < 0) {
				printf("write_output_h264 error!!!\n");
				goto err4;
			}

			ret = codec_release_output(&context, &imgframe);
			if (ret < 0) {
				printf("codec_release_output error!!!\n");
				goto err4;
			}

			hbn_vnode_releaseframe(sth_vnode_fd, 0, &imgframe.vnode_buffer);
		} else {
			printf("hbn_vnode_getframe fail, ret = %d\n", ret);
			goto err4;
		}

		imgframe.cnt++;
	}

	fflush(h264fd);
	fclose(h264fd);
err4:
	hbn_vflow_stop(vflow_fd[0]);
	hbn_vflow_stop(vflow_fd[1]);
err3:
	codec_stop(&context);
err2:
	codec_deinit(&context);
err1:
	for (; i >= 0; i--) {
		hbn_vflow_destroy(vflow_fd[i]);
	}
err:
	return ret;
}
