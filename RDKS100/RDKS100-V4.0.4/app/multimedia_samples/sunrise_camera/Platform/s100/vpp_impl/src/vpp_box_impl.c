#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include "communicate/sdk_common_cmd.h"
#include "communicate/sdk_common_struct.h"
#include "communicate/sdk_communicate.h"

#include "utils/utils_log.h"
#include "utils/cqueue.h"
#include "utils/common_utils.h"
#include "utils/stream_define.h"
#include "utils/stream_manager.h"
#include "utils/mthread.h"
#include "utils/mqueue.h"

#include "model_info.h"
#include "bpu_wrap.h"
#include "vp_wrap.h"
#include "vp_codec.h"

#include "solution_handle.h"
#include "solution_config.h"

#include "vpp_preparam.h"
#include "vpp_box_impl.h"

#define VPP_BOX_MAX_CHANNELS 8

typedef struct
{
	int pipline_id;
	char			m_stream_path[128];
	media_codec_context_t m_decode_context;
	vp_decode_param_t m_decode_param;

	media_codec_user_config_t m_encode_user_config;
	media_codec_context_t m_encode_context;

	vp_vflow_contex_t vp_vflow_contex;

	bpu_handle_t	m_bpu_handle;

	tsThread 		m_venc_thread; /* 图像编码、输出给vo、算法图像前处理 */
	tsThread 		m_vdec_thread; /* 读取h264视频文件解码 */
	tsThread		m_bpu_thread;

	void *media;
	uint64_t first_frame_timestamp;
	const char *media_type;

} vpp_box_t;

static vpp_box_t g_vpp_box[VPP_BOX_MAX_CHANNELS];

static int32_t send_video_frame_info(int pipeline_id, int frame_id, int64_t timestamp)
{
	int32_t ret = 0;
	char *ws_msg = NULL;

	ws_msg = malloc(200);
	if (NULL == ws_msg) {
		SC_LOGE("Failed to allocate memory for ws_msg");
		return -1;
	}
	sprintf(ws_msg, "{\"kind\":11, \"pipeline\":%d, \"frame_id\":%d, \"timestamp\":%ld}", pipeline_id + 1, frame_id, timestamp);
	ret = SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_SEND_MSG, (void*)ws_msg);
	free(ws_msg);
	return ret;
}

static void vpp_box_push_stream(vpp_box_t *vpp_box, ImageFrame *stream, int pipline_id)
{
	if(stream == NULL) {
		SC_LOGE("Param is NULL");
		return;
	}
	media_codec_buffer_t *buffer = (media_codec_buffer_t *)(stream->frame_buffer);

	send_video_frame_info(pipline_id, buffer->vstream_buf.src_idx, buffer->vstream_buf.pts);

	T_SDK_MEDIA_SRV_PUSH_PARAM push_param = {
		.media = vpp_box->media,
		.data = (const char*)buffer->vstream_buf.vir_ptr,
		.data_length = buffer->vstream_buf.size,
		.pts = buffer->vstream_buf.pts /1000,
		.dts = buffer->vstream_buf.pts /1000,
		.codec_name = vpp_box->media_type
	};

	SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_PUSH_DATA, &push_param);
}
static void *get_decode_output_thread(void *ptr) {
	int32_t ret = 0;
	tsThread *privThread = (tsThread*)ptr;

	ImageFrame decode_frame = {0};
	ImageFrame encode_frame = {0};
	ImageFrame encode_stream = {0};

	hbn_vnode_image_t src_img = {0};
	bpu_buffer_info_t bpu_input_buffer = {0};
	media_codec_buffer_t *decode_frame_buffer = NULL;

	vpp_box_t *vpp_box = (vpp_box_t *)privThread->pvThreadData;

	if (vp_allocate_image_frame(&decode_frame) == NULL) {
		SC_LOGE("vp_allocate_image_frame for decode_frame failed, so exit program.");
		exit(-1);
	}
	if (vp_allocate_image_frame(&encode_frame) == NULL) {
		SC_LOGE("vp_allocate_image_frame for encode_frame failed, so exit program.");
		exit(-1);
	}
	if (vp_allocate_image_frame(&encode_stream) == NULL) {
		SC_LOGE("vp_allocate_image_frame for encode_stream failed, so exit program.");
		exit(-1);
	}

	mThreadSetName(privThread, __func__);

	while (privThread->eState == E_THREAD_RUNNING) {
		ret = vp_codec_get_output(&vpp_box->m_decode_context, &decode_frame, VP_DECODER_GET_FRAME_TIMEOUT);
		if (ret != 0) {
			usleep(30 * 1000);
			continue;
		}

		decode_frame_buffer = (media_codec_buffer_t *)decode_frame.frame_buffer;
		if (strlen(vpp_box->m_bpu_handle.m_model_name) > 0) {
			memset(&bpu_input_buffer, 0, sizeof(bpu_buffer_info_t));
			vpp_codec_buf_to_bpu_buffer_info(decode_frame_buffer,
				&bpu_input_buffer);

			bpu_wrap_send_frame(&vpp_box->m_bpu_handle, &bpu_input_buffer);
		}

		for(int i = 0; i< decode_frame.plane_count; i++){
			encode_frame.data[i] = decode_frame.data[i];
			encode_frame.data_size[i] = decode_frame.data_size[i];
		}
		encode_frame.plane_count = decode_frame.plane_count;
		encode_frame.image_timestamp = decode_frame.image_timestamp;

		ret = vp_codec_set_input(&vpp_box->m_encode_context, &encode_frame, 0);
		if (ret != 0) {
			SC_LOGE("vp_codec_set_input send encode frame failed(%d)", ret);
			continue;
		}

		// 从编码器获取码流
		ret = vp_codec_get_output(&vpp_box->m_encode_context, &encode_stream, VP_GET_FRAME_TIMEOUT);
		if (ret != 0) {
			SC_LOGE("vp_codec_get_output get encode stream failed(%d)", ret);
			continue;
		}
		// 推流
		vpp_box_push_stream(vpp_box, &encode_stream, vpp_box->pipline_id);

		// 把轮转 buffer queue 进队列
		vp_codec_release_output(&vpp_box->m_encode_context, &encode_stream);
		vp_codec_release_output(&vpp_box->m_decode_context, &decode_frame);
	}
	vp_free_image_frame(&decode_frame);
	vp_free_image_frame(&encode_frame);
	vp_free_image_frame(&encode_stream);

	hb_mem_free_buf(src_img.buffer.fd[0]);

	mThreadFinish(privThread);
	return NULL;
}

int32_t vpp_box_init_param_full(solution_cfg_t *solution_config)
{
	int i, ret = 0;

	vpp_box_t *vpp_box = NULL;
	solution_cfg_box_vpp_t *cfg_box_vpp = NULL;

	memset(&g_vpp_box, 0, sizeof(g_vpp_box));

	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		g_vpp_box[i].m_encode_context.codec_id = MEDIA_CODEC_ID_NONE;
		g_vpp_box[i].m_decode_context.codec_id = MEDIA_CODEC_ID_NONE;
	}

	for (i = 0; i < solution_config->box_solution.pipeline_count; i++) {
		vpp_box = &g_vpp_box[i];
		cfg_box_vpp = &solution_config->box_solution.box_vpp[i];
		strncpy(vpp_box->m_stream_path, cfg_box_vpp->stream,
				sizeof(vpp_box->m_stream_path) - 1);

		// 配置算法模型
		if (strlen(cfg_box_vpp->model) > 1 && strcmp(cfg_box_vpp->model, "null") != 0) {
			vpp_box->m_bpu_handle.m_vpp_id = i;
			strncpy(vpp_box->m_bpu_handle.m_model_name,
				cfg_box_vpp->model,
				sizeof(vpp_box->m_bpu_handle.m_model_name) - 1);
			vpp_box->m_bpu_handle.m_model_name[sizeof(vpp_box->m_bpu_handle.m_model_name) - 1] = '\0';
		}

		// 配置编码通道
		media_codec_user_config_t *codec_user_config = &g_vpp_box[i].m_encode_user_config;
		codec_user_config->bit_rate = cfg_box_vpp->encode_bitrate;
		codec_user_config->codec_type =VP_GET_MD_CODEC_TYPE(cfg_box_vpp->encode_type);
		codec_user_config->frame_rate = cfg_box_vpp->encode_frame_rate;
		codec_user_config->width = cfg_box_vpp->encode_width;
		codec_user_config->height = cfg_box_vpp->encode_height;

		codec_user_config->input_buffer_is_extrenal = false;
		codec_user_config->input_buffer_count = 5;
		codec_user_config->output_buffer_count = 5;

		ret = vp_encode_config_param(&vpp_box->m_encode_context, codec_user_config);
		if (ret != 0) {
			SC_LOGE("Encode config param error, type:%d width:%d height:%d"
				" frame_rate: %d bit_rate:%d\n",
				VP_GET_MD_CODEC_TYPE(cfg_box_vpp->encode_type),
				cfg_box_vpp->encode_width,
				cfg_box_vpp->encode_height,
				cfg_box_vpp->encode_frame_rate,
				cfg_box_vpp->encode_bitrate);
		}

		// 配置解码通道
		ret = vp_decode_config_param(&vpp_box->m_decode_context,
			VP_GET_MD_CODEC_TYPE(cfg_box_vpp->decode_type),
			cfg_box_vpp->decode_width,
			ALIGN_16(cfg_box_vpp->decode_height));
		if (ret != 0)
		{
			SC_LOGE("Decode config param error, type:%d width:%d height:%d\n",
				VP_GET_MD_CODEC_TYPE(cfg_box_vpp->decode_type),
				cfg_box_vpp->decode_width,
				ALIGN_16(cfg_box_vpp->decode_height));
		}

	}

	return ret;
}
int32_t vpp_box_init_param(void)
{
	return vpp_box_init_param_full(&g_solution_config);
}

int32_t vpp_box_ion_param_get(solution_cfg_t* solution_cfg, solution_ion_param_info_t *solution_param_info){
	return 0;
}

int32_t vpp_box_vpu_param_get(solution_cfg_t* solution_cfg, solution_vpu_param_info_t *solution_param_info){
	solution_cfg_box_vpp_t *cfg_box_vpp = NULL;
	solution_param_info->valid_count = 0;
	for (int i = 0; i < solution_cfg->box_solution.pipeline_count; i++) {
		cfg_box_vpp = &solution_cfg->box_solution.box_vpp[i];
		vp_codec_usr_param_single_t *param_single = &solution_param_info->params[solution_param_info->valid_count];
		param_single->encode.width = cfg_box_vpp->encode_width;
		param_single->encode.height = cfg_box_vpp->encode_height;
		param_single->encode.fps = cfg_box_vpp->encode_frame_rate;

		param_single->decode.width = cfg_box_vpp->decode_width;
		param_single->decode.height = cfg_box_vpp->decode_height;
		param_single->decode.fps = cfg_box_vpp->decode_frame_rate;
		solution_param_info->valid_count++;

		SC_LOGI("vpp_box_vpu_param_get [%d] [encode:%d %d %d] [decode:%d %d %d]",
			solution_param_info->valid_count,
			param_single->encode.width, param_single->encode.height, param_single->encode.fps,
			param_single->decode.width, param_single->decode.height, param_single->decode.fps);
	}
	return 0;
}
int32_t vpp_box_init(void)
{
	int32_t i = 0, ret = 0;

	hb_mem_module_open();

	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (strlen(g_vpp_box[i].m_stream_path) == 0)
			continue;

		// 初始化编码器
		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_init(&g_vpp_box[i].m_encode_context);
			if (ret != 0)
			{
				SC_LOGE("Encode vp_codec_init error(%d)", i);
				return -1;
			}
			SC_LOGI("Init video encode instance %d successful", g_vpp_box[i].m_encode_context.instance_index);
		}

		// 初始化解码器
		if (g_vpp_box[i].m_decode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_init(&g_vpp_box[i].m_decode_context);
			if (ret != 0)
			{
				SC_LOGE("Decode vp_codec_init error(%d)", i);
				return -1;
			}
			SC_LOGI("Init video decode instance %d successful", g_vpp_box[i].m_decode_context.instance_index);
		}

		// 初始化算法模块，初始化bpu
		if (strlen(g_vpp_box[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_model_init(&g_vpp_box[i].m_bpu_handle, g_vpp_box[i].m_bpu_handle.m_model_name);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_model_init failed");
			return -1;
		}
		// 注册算法结果回调函数
		bpu_wrap_callback_register(&g_vpp_box[i].m_bpu_handle,
			bpu_wrap_general_result_handle, &g_vpp_box[i].m_bpu_handle.m_vpp_id);
	}

	return 0;
}

int32_t vpp_box_uninit(void)
{
	int32_t i = 0, ret = 0;

	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (strlen(g_vpp_box[i].m_stream_path) == 0)
			continue;
		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_deinit(&g_vpp_box[i].m_encode_context);
			if (ret != 0)
			{
				SC_LOGE("Encode vp_codec_deinit error(%d)", i);
				return -1;
			}
			SC_LOGI("Deinit video encode instance %d successful", g_vpp_box[i].m_encode_context.instance_index);
		}

		if (g_vpp_box[i].m_decode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_deinit(&g_vpp_box[i].m_decode_context);
			if (ret != 0)
			{
				SC_LOGE("Decode vp_codec_deinit error(%d)", i);
				return -1;
			}
			SC_LOGI("Deinit video decode instance %d successful", g_vpp_box[i].m_decode_context.instance_index);
		}

		if (strlen(g_vpp_box[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_deinit(&g_vpp_box[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_model_init failed");
			return -1;
		}
	}

	hb_mem_module_close();

	vp_print_debug_infos();
	return 0;
}

int32_t vpp_box_start(void)
{
	int32_t i = 0, ret = 0;

	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (strlen(g_vpp_box[i].m_stream_path) == 0)
			continue;
		g_vpp_box[i].pipline_id = i;

		//media server
		char meida_name[64];
		sprintf(meida_name, "ch%d", g_vpp_box[i].pipline_id);
		g_vpp_box[i].media_type = vp_codec_get_codec_type_string(g_vpp_box[i].m_encode_context.codec_id);

		T_SDK_MEDIA_SRV_CREATE_PARAM create_param = {
			.media_name = meida_name,
			.stream_name = "main",
			.codec_type_name = g_vpp_box[i].media_type,
			.media = NULL,
		};
		SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_CREATE, &create_param);
		 	g_vpp_box[i].media = create_param.media;

		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_start(&g_vpp_box[i].m_encode_context);
			if (ret != 0)
			{
				SC_LOGE("Encode vp_codec_start error(%d)", i);
				return -1;
			}
			SC_LOGI("Start video encode instance %d successful", g_vpp_box[i].m_encode_context.instance_index);

			g_vpp_box[i].m_venc_thread.pvThreadData = (void *)&g_vpp_box[i];
			mThreadStart(get_decode_output_thread, &g_vpp_box[i].m_venc_thread, E_THREAD_JOINABLE);
		}

		// 启动解码线程
		if (g_vpp_box[i].m_decode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_start(&g_vpp_box[i].m_decode_context);
			if (ret != 0)
			{
				SC_LOGE("Decode vp_codec_start error(%d)", i);
					return -1;
			}
			SC_LOGI("Start video decode instance %d successful", g_vpp_box[i].m_decode_context.instance_index);
			g_vpp_box[i].m_decode_param.context = &g_vpp_box[i].m_decode_context;
			strcpy(g_vpp_box[i].m_decode_param.stream_path, g_vpp_box[i].m_stream_path);
			g_vpp_box[i].m_vdec_thread.pvThreadData = (void*)&g_vpp_box[i].m_decode_param;
			mThreadStart(vp_decode_work_func, &g_vpp_box[i].m_vdec_thread, E_THREAD_JOINABLE);
		}

		if (strlen(g_vpp_box[i].m_bpu_handle.m_model_name) == 0)
			continue;
		ret = bpu_wrap_start(&g_vpp_box[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_start failed");
			return -1;
		}
	}

	vp_print_debug_infos();
	return 0;
}

static int32_t get_pipeline_id_by_video_id(int32_t video_id)
{
	int32_t i = 0;
	int32_t enable_pipeline_count = 0;
	// 遍历所有 pipeline
	// 用 enable_pipeline_count 记录使能的pipeline的编号，这个编号理论上与 web 上的video编号相等
	// 当 enable_pipeline_count == video_id时就说明找到了对应的pipeline
	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			enable_pipeline_count++;
			if (enable_pipeline_count == video_id) {
				return i;
			}
		}
	}
	return 0;
}

int32_t vpp_box_stop(void)
{
	int32_t i = 0, ret = 0;

	// 先把所有线程停掉
	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (strlen(g_vpp_box[i].m_stream_path) == 0)
			continue;
		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			// 结束编码线程
			mThreadStop(&g_vpp_box[i].m_venc_thread);
		}
		if (g_vpp_box[i].m_decode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			mThreadStop(&g_vpp_box[i].m_vdec_thread);
		}
	}

	for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
		if (strlen(g_vpp_box[i].m_stream_path) == 0)
			continue;

		if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_stop(&g_vpp_box[i].m_encode_context);
			if (ret != 0)
			{
				SC_LOGE("Encode vp_codec_stop error(%d)", i);
				return -1;
			}
			SC_LOGI("Stop video encode instance %d successful", g_vpp_box[i].m_encode_context.instance_index);
		}

		if (g_vpp_box[i].m_decode_context.codec_id != MEDIA_CODEC_ID_NONE) {
			ret = vp_codec_stop(&g_vpp_box[i].m_decode_context);
			if (ret != 0)
			{
				SC_LOGE("Decode vp_codec_stop error(%d)", i);
				return -1;
			}
			SC_LOGI("Stop video decode instance %d successful", g_vpp_box[i].m_decode_context.instance_index);
		}

		SDK_Cmd_Impl(SDK_CMD_MEDIA_SERVER_DESTROY, g_vpp_box[i].media);
		// media_server_destroy_media(g_vpp_camera[i].media);
		g_vpp_box[i].media = NULL;
		g_vpp_box[i].media_type = NULL;

		if (strlen(g_vpp_box[i].m_bpu_handle.m_model_name) == 0)
			continue;
		// mThreadStop(&g_vpp_box[i].m_bpu_thread);
		ret = bpu_wrap_stop(&g_vpp_box[i].m_bpu_handle);
		if (ret != 0) {
			SC_LOGE("bpu_wrap_start failed");
			return -1;
		}
	}

	return 0;
}

int32_t vpp_box_param_set(SOLUTION_PARAM_E type, char* val, uint32_t length)
{
	switch(type)
	{
	case SOLUTION_VENC_BITRATE_SET:
		{
			break;
		}
	default:
		break;
	}
	return 0;
}

int32_t vpp_box_param_get(SOLUTION_PARAM_E type, char* val, uint32_t* length)
{
	int32_t i= 0, ret = 0;
	mc_video_codec_enc_params_t *enc_params;
	ImageFrame image_frame = {0};
	char file_name[256] = {0};

	switch(type)
	{
	case SOLUTION_VENC_CHN_PARAM_GET: // 获取某个编码通道的配置
		{
			venc_info_t* param = (venc_info_t*)val;
			param->enable = 0;
			SC_LOGI("param->channel: %d", param->channel);
			if(i >= VPP_BOX_MAX_CHANNELS){
				SC_LOGE("box solutions max channel is %d, but get channel index is %d .", VPP_BOX_MAX_CHANNELS, i);
				return -1;
			}
			if(g_vpp_box[param->channel].m_encode_context.codec_id == MEDIA_CODEC_ID_NONE){
				SC_LOGE("box solutions channel %d is not enable, can't get encode param .", param->channel);
				return -1;
			}

			enc_params = &g_vpp_box[param->channel].m_encode_context.video_enc_params;
			param->enable = 1;
			param->width = enc_params->width;
			param->height = enc_params->height;
			param->stream_buf_size = enc_params->bitstream_buf_size;
			if (g_vpp_box[param->channel].m_encode_context.codec_id == MEDIA_CODEC_ID_H264) {
				param->type = 96;
				param->bitrate = enc_params->rc_params.h264_cbr_params.bit_rate;
				param->framerate = enc_params->rc_params.h264_cbr_params.frame_rate;
			} else if (g_vpp_box[param->channel].m_encode_context.codec_id == MEDIA_CODEC_ID_H265) {
				param->type = 265;
				param->bitrate = enc_params->rc_params.h265_cbr_params.bit_rate;
				param->framerate = enc_params->rc_params.h265_cbr_params.frame_rate;
			} else {
				SC_LOGE("unsupport codec_id %d, so exit.", g_vpp_box[param->channel].m_encode_context.codec_id);
				exit(-1);
			}
			vp_codec_get_user_buffer_param(enc_params, &param->suggest_buffer_region_size,
					&param->suggest_buffer_item_count);
			SC_LOGI("Codec_id: %d", g_vpp_box[param->channel].m_encode_context.codec_id);
			SC_LOGI("Instance Index: %d", g_vpp_box[param->channel].m_encode_context.instance_index);
			SC_LOGI("Param Channel: %d", param->channel);
			SC_LOGI("Param Enable: %d", param->enable);
			SC_LOGI("Param Width: %d", param->width);
			SC_LOGI("Param Height: %d", param->height);
			SC_LOGI("Param Stream Buffer Size: %d", param->stream_buf_size);
			SC_LOGI("Param Type: %d", param->type);
			SC_LOGI("Param Bitrate: %d", param->bitrate);
			SC_LOGI("Param Framerate: %d", param->framerate);
			break;
		}
	case SOLUTION_GET_VENC_CHN_STATUS: // 获取哪些编码通道被使能了
		{
			// 32位的整形，每个通道的状态占其中一个bit
			// 注： 64bit的值位与会有异常，待查
			unsigned int *status = (unsigned int *)val;
			*status = 0;
			int valid_index = 0;
			for (i = 0; i < VPP_BOX_MAX_CHANNELS; i++) {
				if (g_vpp_box[i].m_encode_context.codec_id != MEDIA_CODEC_ID_NONE) {
					*status |= (1 << valid_index);
					valid_index++;
				}
			}
			SC_LOGI("Box Solution current enabled status is [0x%x]\n", *status);
			break;
		}
	case SOLUTION_GET_PYM_FRAME:
{
			// video_id 代表web上的第几个 video 控件，从1开始计数
			// 需要结合当前使能了多少路pipeline来获取到对应的 pipeline id
			int32_t video_id = *(int32_t *)val;
			int32_t pipeline_id = get_pipeline_id_by_video_id(video_id);
			if (vp_allocate_image_frame(&image_frame) == NULL) {
				SC_LOGE("vp_allocate_image_frame failed");
				return -1;
			}
			ret = vp_pym_get_frame(&g_vpp_box[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_pym_get_frame failed (%d)", ret);
				vp_free_image_frame(&image_frame);
				return -1;
			}

			hbn_vnode_image_t *hbn_vnode_image = (hbn_vnode_image_t *)image_frame.hbn_vnode_image;

			snprintf(file_name, sizeof(file_name),
				"/tmp/pipeline_%d_pym_ochn0_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
				pipeline_id,
				hbn_vnode_image->buffer.width,
				hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			SC_LOGI("pipeline %d pym dump yuv %dx%d(stride:%d), buffer size: %ld frame id: %d,"
				" timestamp: %ld",
				pipeline_id,
				hbn_vnode_image->buffer.width, hbn_vnode_image->buffer.height,
				hbn_vnode_image->buffer.stride,
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->info.frame_id,
				hbn_vnode_image->info.timestamps);

			delete_files_with_extension("/tmp", ".yuv");
			vp_dump_2plane_yuv_to_file(file_name,
				hbn_vnode_image->buffer.virt_addr[0],
				hbn_vnode_image->buffer.virt_addr[1],
				hbn_vnode_image->buffer.size[0],
				hbn_vnode_image->buffer.size[1]);

			vp_pym_release_frame(&g_vpp_box[pipeline_id].vp_vflow_contex, &image_frame);
			if (ret != 0) {
				SC_LOGE("vp_pym_release_frame failed.");
				vp_free_image_frame(&image_frame);
				return -1;
			}
			vp_free_image_frame(&image_frame);
			// 通知浏览器下载文件
			SDK_Cmd_Impl(SDK_CMD_WEBSOCKET_UPLOAD_FILE, (void*)file_name);
			break;
		}
	default:
		{
			ret= -1;
			break;
		}
	}
	return ret;
}
