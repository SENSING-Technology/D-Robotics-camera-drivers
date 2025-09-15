#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <dirent.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include "utils.h"

#define  SENSOR_NUMS  4
uint32_t g_loop = 5;
uint32_t g_need_dump = 0;
uint32_t g_debug = 0;
const char *g_res_path = "/app/multimedia_demo/camsys_demo/sample_gdc_stitch";
hb_mem_common_buf_t gdc_config_buf[SENSOR_NUMS];
hb_mem_graphic_buf_t stitch_dst_buf;
hb_mem_common_buf_t alpha_buffer;
hb_mem_common_buf_t beta_buffer;

#define MAGIC_NUM	0x12345678
#define AUTO_ALLOC_ID   -1  // TODO

struct stitch_base_attr base_attr = {
	.mode = 0,
	.roi_nums = 8,
	.img_nums = 4,
	.alpha_lut = { .share_id = 52, .vaddr = 0, .offset = 0, .size = 373052  },
	.beta_lut  = { .share_id = 0,  .vaddr = 0, .offset = 0, .size = 0  },
	.blending = {
		{ .roi_index = 0, .blending_mode = 3, .direct = 0, .uv_en = 1, .src0_index = 2, .src1_index = 2, .margin = 0,  .margin_inv = 128, .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 1, .blending_mode = 3, .direct = 0, .uv_en = 1, .src0_index = 3, .src1_index = 3, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 2, .blending_mode = 3, .direct = 0, .uv_en = 1, .src0_index = 0, .src1_index = 0, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 3, .blending_mode = 3, .direct = 0, .uv_en = 1, .src0_index = 1, .src1_index = 1, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 4, .blending_mode = 1, .direct = 0, .uv_en = 1, .src0_index = 2, .src1_index = 1, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 5, .blending_mode = 1, .direct = 3, .uv_en = 1, .src0_index = 3, .src1_index = 1, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 6, .blending_mode = 1, .direct = 2, .uv_en = 1, .src0_index = 2, .src1_index = 0, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 7, .blending_mode = 1, .direct = 1, .uv_en = 1, .src0_index = 3, .src1_index = 0, .margin = 10, .margin_inv = 0,   .gain_src0_yuv = {256, 256, 256}, .gain_src1_yuv = {256, 256, 256}  },
		{ .roi_index = 0, .blending_mode = 0, .direct = 0, .uv_en = 0, .src0_index = 0, .src1_index = 0, .margin = 0,  .margin_inv = 0,   .gain_src0_yuv = {0, 0, 0},       .gain_src1_yuv = {0, 0, 0}  },
		{ .roi_index = 0, .blending_mode = 0, .direct = 0, .uv_en = 0, .src0_index = 0, .src1_index = 0, .margin = 0,  .margin_inv = 0,   .gain_src0_yuv = {0, 0, 0},       .gain_src1_yuv = {0, 0, 0}  },
		{ .roi_index = 0, .blending_mode = 0, .direct = 0, .uv_en = 0, .src0_index = 0, .src1_index = 0, .margin = 0,  .margin_inv = 0,   .gain_src0_yuv = {0, 0, 0},       .gain_src1_yuv = {0, 0, 0}  },
		{ .roi_index = 0, .blending_mode = 0, .direct = 0, .uv_en = 0, .src0_index = 0, .src1_index = 0, .margin = 0,  .margin_inv = 0,   .gain_src0_yuv = {0, 0, 0},       .gain_src1_yuv = {0, 0, 0}  },
	}
};

struct stitch_ch_attr inch_attr[4] = {
	{
		.width = 896,
		.height = 298,
		.strid = {896, 896},
		.rois = {
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 2, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 6, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 7, .roi_x = 506, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },

		}
	},
	{
		.width = 896,
		.height = 298,
		.strid = {896, 896},
		.rois = {
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 3, .roi_x = 4, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 4, .roi_x = 2, .roi_y = 16, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 5, .roi_x = 508, .roi_y = 14, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },

		}
	},
	{
		.width = 400,
		.height = 778,
		.strid = {400, 400},
		.rois = {
			{ .roi_index = 0, .roi_x = 10, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 4, .roi_x = 10, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 6, .roi_x = 10, .roi_y = 582, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0, .roi_y = 0, .roi_w = 0, .roi_h = 0  },

		}

	},
	{
		.width = 400,
		.height = 780,
		.strid = {400, 400},
		.rois = {
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 1, .roi_x = 10, .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 5, .roi_x = 10, .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 7, .roi_x = 10, .roi_y = 584, .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },
			{ .roi_index = 0, .roi_x = 0,  .roi_y = 0,   .roi_w = 0, .roi_h = 0  },

		}
	}
};

struct stitch_ch_attr och_attr = {
	.width = 896,
	.height = 896,
	.strid = {896, 896},
	.rois = {
		{ .roi_index = 0, .roi_x =   0, .roi_y =  16, .roi_w = 390, .roi_h = 778  },
		{ .roi_index = 1, .roi_x = 506, .roi_y =  14, .roi_w = 390, .roi_h = 780  },
		{ .roi_index = 2, .roi_x =   0, .roi_y = 598, .roi_w = 896, .roi_h = 298  },
		{ .roi_index = 3, .roi_x =   0, .roi_y =   0, .roi_w = 892, .roi_h = 298  },
		{ .roi_index = 4, .roi_x =   0, .roi_y =  16, .roi_w = 390, .roi_h = 282  },
		{ .roi_index = 5, .roi_x = 506, .roi_y =  14, .roi_w = 388, .roi_h = 284  },
		{ .roi_index = 6, .roi_x =   0, .roi_y = 598, .roi_w = 390, .roi_h = 196  },
		{ .roi_index = 7, .roi_x = 506, .roi_y = 598, .roi_w = 390, .roi_h = 196  },
		{ .roi_index = 0, .roi_x =   0, .roi_y =   0, .roi_w =   0, .roi_h =   0  },
		{ .roi_index = 0, .roi_x =   0, .roi_y =   0, .roi_w =   0, .roi_h =   0  },
		{ .roi_index = 0, .roi_x =   0, .roi_y =   0, .roi_w =   0, .roi_h =   0  },
		{ .roi_index = 0, .roi_x =   0, .roi_y =   0, .roi_w =   0, .roi_h =   0  },

	}
};

gdc_settings_t gdc_setting[] = {
	{
		.gdc_config = {
			.input_width = 1920,
			.input_height = 1376,
			.output_width = 896,
			.output_height = 298,
			.total_planes = 2,
		}

	},
	{
		.gdc_config = {
			.input_width = 1920,
			.input_height = 1376,
			.output_width = 896,
			.output_height = 298,
			.total_planes = 2,
		}

	},
	{
		.gdc_config = {
			.input_width = 1920,
			.input_height = 1376,
			.output_width = 400,
			.output_height = 778,
			.total_planes = 2,
		}

	},
	{
		.gdc_config = {
			.input_width = 1920,
			.input_height = 1376,
			.output_width = 400,
			.output_height = 780,
			.total_planes = 2,
		}

	}
};

void print_usage(const char *prog)
{
	printf("Usage: %s \n", prog);
	puts("  -l --loop           need while/loop\n"
		 "  -d --dump           dump img, 1: dump stitch output 2: dump gdc output 3: dump all\n"
		 "  -r --res_path       res file path\n"
		 "  -D --debug          print stitch cfg\n");
	exit(1);
}

void parse_opts(int argc, char *argv[])
{
	int cmd_ret;
	static const char short_options[] =
			"l:d:s:v:D:r:h";
	static const struct option long_options[] = {
		{ "loop", 1, 0, 'l' },
		{ "dump", 1, 0, 'd' },
		{ "debug", 1, 0, 'D' },
		{ "res", 1, 0, 'r' },
		{ "help", 1, 0, 'h' },
		{ NULL, 0, 0, 0 },
	};

	while (1) {
		cmd_ret = getopt_long(argc, argv, short_options, long_options, NULL);
		if (cmd_ret == -1)
			break;

		switch (cmd_ret) {
			case 'l':
				g_loop = atoi(optarg);
				printf("loop = %d\n", g_loop);
				break;
			case 'd':
				g_need_dump = atoi(optarg);
				printf("need_dump = %d\n", g_need_dump);
				break;
			case 'r':
				g_res_path = optarg;
				printf("g_res_path = %s\n", g_res_path);
				break;
			case 'D':
				g_debug = atoi(optarg);
				printf("debug = %d\n", g_debug);
				break;
			default:
				print_usage(argv[0]);
				break;
		}
	}
}

int32_t init_gdc(test_ctx_t *test_ctx)
{
	int32_t ret, i;
	char res_file_name[128] = {0};
	/* gdc_settings_t gdc_setting[4] = {0}; */
	hbn_buf_alloc_attr_t alloc_attr = {0};
	struct stat fileStat;

	char* gdc_config[SENSOR_NUMS] = {
		"camera_0_gdc_cfg.bin",
		"camera_1_gdc_cfg.bin",
		"camera_2_gdc_cfg.bin",
		"camera_3_gdc_cfg.bin"
	};

	for (i = 0; i < SENSOR_NUMS; i++) {
		memset(res_file_name, 0, sizeof(res_file_name));
		sprintf(res_file_name, "%s/%s", g_res_path, gdc_config[i]);
		if(stat(res_file_name, &fileStat) != 0) {
			printf("Failed to get file stats. cfg file = %s\n", res_file_name);
			return -1;
		}

		ret = hb_mem_alloc_com_buf(fileStat.st_size, HB_MEM_USAGE_MAP_INITIALIZED |
			HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
			HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED, &gdc_config_buf[i]);
		if (ret < 0) {
			printf("hb_mem_alloc_com_buf alpha_lut faild, ret = %d\n", ret);
			return -1;
		}

		load_file_2_buff(res_file_name, (char *)gdc_config_buf[i].virt_addr, fileStat.st_size);
		hb_mem_flush_buf_with_vaddr((uint64_t)gdc_config_buf[i].virt_addr, fileStat.st_size);

		gdc_setting[i].magicNumber = MAGIC_NUM;
		gdc_setting[i].binary_ion_id = gdc_config_buf[i].share_id;
		gdc_setting[i].gdc_config.config_size = fileStat.st_size;
		gdc_setting[i].gdc_config.total_planes = 2;
		/* gdc_setting[i].buf_num = 3; */
		gdc_setting[i].gdc_config.input_stride = ALIGN_UP(gdc_setting[i].gdc_config.input_width, 16u);
		gdc_setting[i].gdc_config.output_stride = ALIGN_UP(gdc_setting[i].gdc_config.output_width, 16u);

		ret = hbn_vnode_open(HB_GDC, 0, AUTO_ALLOC_ID, &test_ctx->gdc_handle[i]);
		if (ret < 0) {
			printf("GDC vnode open fail\n");
			return -1;
		}

		ret = hbn_vnode_set_attr(test_ctx->gdc_handle[i], &gdc_setting[i]);
		if (ret < 0) {
			printf("GDC vnode set attr fail\n");
			return -1;
		}
		ret = hbn_vnode_set_ichn_attr(test_ctx->gdc_handle[i], 0, &gdc_setting[i]);
		if (ret < 0) {
			printf("GDC vnode set ichn attr fail\n");
			return -1;
		}
		ret = hbn_vnode_set_ochn_attr(test_ctx->gdc_handle[i], 0, &gdc_setting[i]);
		if (ret < 0) {
			printf("GDC vnode set ochn attr fail\n");
			return -1;
		}

		memset(&alloc_attr, 0, sizeof(hbn_buf_alloc_attr_t));
		alloc_attr.buffers_num = 3;
		alloc_attr.is_contig = 1;
		alloc_attr.flags = (int64_t)((uint64_t)HB_MEM_USAGE_CPU_READ_OFTEN |
				(uint64_t)HB_MEM_USAGE_CPU_WRITE_OFTEN | (uint64_t)HB_MEM_USAGE_MAP_INITIALIZED);
		alloc_attr.flags |= (uint64_t)HB_MEM_USAGE_CACHED;

		ret = hbn_vnode_set_ochn_buf_attr(test_ctx->gdc_handle[i], 0, &alloc_attr);
		if (ret < 0) {
			printf("GDC vnode set ochn buf attr fail\n");
			return -1;
		}

		ret = hbn_vnode_start(test_ctx->gdc_handle[i]);
		if (ret < 0) {
			printf("GDC vnode start fail\n");
			return -1;
		}
	}

	return 0;
}

int32_t deinit_gdc(test_ctx_t *test_ctx)
{
	int32_t i;

	for (i = 0; i < SENSOR_NUMS; i++) {
		hbn_vnode_stop(test_ctx->gdc_handle[i]);
		hbn_vnode_close(test_ctx->gdc_handle[i]);
	}

	return 0;
}

int32_t init_stitch(test_ctx_t *test_ctx)
{
	int32_t ret = 0, i;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	char res_file_name[128] = {0};
	struct stat fileStat;

	ret = hbn_vnode_open(HB_STITCH, 0, -1, &test_ctx->sth_handle);
	if (ret < 0) {
		printf("STH vnode open fail\n");
		return -1;
	}

	memset(res_file_name, 0, sizeof(res_file_name));
	sprintf(res_file_name, "%s/%s", g_res_path, "alpha_lut_apa.bin");
	if(stat(res_file_name, &fileStat) != 0) {
		printf("Failed to get file stats. cfg file = %s\n", res_file_name);
		return -1;
	}

	ret = hb_mem_alloc_com_buf(fileStat.st_size, HB_MEM_USAGE_MAP_INITIALIZED |
							HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD | HB_MEM_USAGE_CPU_READ_OFTEN |
							HB_MEM_USAGE_CPU_WRITE_OFTEN | HB_MEM_USAGE_CACHED, &alpha_buffer);
	if (ret < 0) {
		printf("hb_mem_alloc_com_buf alpha_lut faild, ret = %d\n", ret);
		return -1;
	}

	load_file_2_buff(res_file_name, (char *)alpha_buffer.virt_addr, fileStat.st_size);
	hb_mem_flush_buf_with_vaddr((uint64_t)alpha_buffer.virt_addr, fileStat.st_size);

	base_attr.alpha_lut.share_id = alpha_buffer.share_id;
	base_attr.alpha_lut.vaddr = (uint64_t)alpha_buffer.virt_addr;
	base_attr.alpha_lut.size = fileStat.st_size;

	ret = hbn_vnode_set_attr(test_ctx->sth_handle, &base_attr);
	if (ret < 0) {
		printf("STH vnode set attr fail\n");
		return -1;
	}

	for (i = 0; i < SENSOR_NUMS; i++) {
		ret = hbn_vnode_set_ichn_attr(test_ctx->sth_handle, i, &inch_attr[i]);
		if (ret < 0) {
			printf("STH vnode set ichn attr fail\n");
			return -1;
		}
	}

	ret = hbn_vnode_set_ochn_attr(test_ctx->sth_handle, 0, &och_attr);
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

	ret = hbn_vnode_set_ochn_buf_attr(test_ctx->sth_handle, 0, &alloc_attr);
	if (ret < 0) {
		printf("STH vnode set ochn buf attr fail\n");
		return -1;
	}

	ret = hbn_vnode_start(test_ctx->sth_handle);
	if (ret < 0) {
		printf("STH vnode start fail\n");
		return -1;
	}

	return 0;
}

int32_t deinit_stitch(test_ctx_t *test_ctx)
{
	int32_t ret = 0;

	hbn_vnode_stop(test_ctx->sth_handle);
	hbn_vnode_close(test_ctx->sth_handle);

	return ret;
}

int main(int argc, char *argv[])
{

	int32_t i = 0, j = 0;
	int32_t ret = 0;
	uint32_t loop = 0;
	char name[64] = { 0 };
	char res_file_name[128] = {0};

	test_ctx_t test_ctx = {0};
	hb_vio_buffer_t gdc_src_buf = {0};
	hb_vio_buffer_t gdc_dst_buf[SENSOR_NUMS] = {0};
	char* gdc_config[SENSOR_NUMS] = {
		"camera_0_gdc_cfg.bin",
		"camera_1_gdc_cfg.bin",
		"camera_2_gdc_cfg.bin",
		"camera_3_gdc_cfg.bin"
	};
	char* gdc_yuv_file[SENSOR_NUMS] = {
		"camera_0.yuv",
		"camera_1.yuv",
		"camera_2.yuv",
		"camera_3.yuv"
	};

	hb_mem_module_open();
	parse_opts(argc, argv);

	for (i = 0; i < SENSOR_NUMS; i++) {
		ret = hb_mem_alloc_graph_buf(1920, 1376,
			MEM_PIX_FMT_NV12, HB_MEM_USAGE_MAP_INITIALIZED | HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
			HB_MEM_USAGE_CPU_READ_OFTEN | HB_MEM_USAGE_CPU_WRITE_OFTEN, 1920, 1376,
			&test_ctx.src_img[i].buffer);
		if (ret != 0) {
			printf("alloc src buf fail\n");
			return -1;
		}

		// push yuv data to src_buf
		int img_size = 1920 * 1376;
		memset(res_file_name, 0, sizeof(res_file_name));
		sprintf(res_file_name, "%s/%s", g_res_path, gdc_yuv_file[i]);
		ret = read_yuv420_file(res_file_name,
						test_ctx.src_img[i].buffer.virt_addr[0],
						test_ctx.src_img[i].buffer.virt_addr[1],
						img_size);
	}

	// init gdc
	ret = init_gdc(&test_ctx);
	if (ret != 0) {
		printf("init gdc fail\n");
		return -1;
	}

	// init stitch
	ret = init_stitch(&test_ctx);
	if (ret < 0) {
		goto err1;
	}

	for (loop = 0; loop < g_loop; loop++) {
		// gdc process
		for (i = 0; i < SENSOR_NUMS; i++) {
			// send to gdc
			ret = hbn_vnode_sendframe(test_ctx.gdc_handle[i], 0, &test_ctx.src_img[i]);
			if (ret != 0) {
				printf("send frame fail sensor=%d\n", i);
				goto err;
			}
			// get gdc frame
			ret = hbn_vnode_getframe(test_ctx.gdc_handle[i], 0, 1000, &test_ctx.gdc_out_img[i]);
			if (ret != 0) {
				printf("get frame fail sensor=%d\n", i);
				goto err;
			}

			printf("vio get gdc_buffer success,sensor=%d\n", i);
			if (g_need_dump & 0x2) {
				sprintf(name, "gdc_%d_dst_w%d_h%d_cnt%d.yuv", i,
						test_ctx.gdc_out_img[i].buffer.width,
						test_ctx.gdc_out_img[i].buffer.height, loop);

				dumpToFile2plane(name, test_ctx.gdc_out_img[i].buffer.virt_addr[0],
								test_ctx.gdc_out_img[i].buffer.virt_addr[1],
								test_ctx.gdc_out_img[i].buffer.size[0],
								test_ctx.gdc_out_img[i].buffer.size[1]);
			}
		}

		// send to stitch
		ret = hbn_vnode_sendframe_async(test_ctx.sth_handle, 3, &test_ctx.gdc_out_img[3]);
		ret |= hbn_vnode_sendframe_async(test_ctx.sth_handle, 2, &test_ctx.gdc_out_img[2]);
		ret |= hbn_vnode_sendframe_async(test_ctx.sth_handle, 1, &test_ctx.gdc_out_img[1]);
		ret |= hbn_vnode_sendframe(test_ctx.sth_handle, 0, &test_ctx.gdc_out_img[0]);
		if (ret != 0) {
			printf("send frame to sth fail\n");
			goto err;
		}
		// get stitch frame
		ret = hbn_vnode_getframe(test_ctx.sth_handle, 0, 1000, &test_ctx.sth_out_img);
		if (g_need_dump & 0x1) {
			sprintf(name, "stitch_dst_w%d_h%d_cnt%d.yuv",
					test_ctx.gdc_out_img[i].buffer.width,
					test_ctx.gdc_out_img[i].buffer.height, loop);

			dumpToFile2plane(name, test_ctx.sth_out_img.buffer.virt_addr[0],
							test_ctx.sth_out_img.buffer.virt_addr[1],
							test_ctx.sth_out_img.buffer.size[0],
							test_ctx.sth_out_img.buffer.size[1]);
		}
		ret = hbn_vnode_releaseframe(test_ctx.sth_handle, 0, &test_ctx.sth_out_img);

		for (i = 0; i < SENSOR_NUMS; i++)
			hbn_vnode_releaseframe(test_ctx.gdc_handle[i], 0, &test_ctx.gdc_out_img[i]);
	}

err:
	deinit_stitch(&test_ctx);
err1:
	deinit_gdc(&test_ctx);

	printf("GDC & STITCH TEST PASSED\n");
	return 0;
}



