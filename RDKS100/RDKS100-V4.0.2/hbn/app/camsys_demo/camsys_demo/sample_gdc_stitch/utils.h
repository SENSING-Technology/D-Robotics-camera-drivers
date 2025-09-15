#ifndef __TEST_STITCH_PROCESS__
#define __TEST_STITCH_PROCESS__

#include "hb_vpm_data_info.h"
#include "hb_vio_interface.h"
#include "hbn_vin_cfg.h"
#include "hb_comm_isp.h"
#include "hbn_pym_cfg.h"
#include "hb_camera_data_config.h"
#include "hbn_vpf_data_info.h"
#include "hb_camera_interface.h"
#include "hbn_vpf_interface.h"
#include "hbn_sth_cfg.h"

#define MAX_FILE_NAME_LENGTH 100

#define ALIGN_UP(a, size) (((a) + (size)-1u) & (~((size)-1u)))

typedef struct test_ctx_s {
	hbn_vnode_handle_t sth_handle;
	hbn_vnode_handle_t gdc_handle[4];
	hbn_vnode_image_t src_img[4];
	hbn_vnode_image_t gdc_out_img[4];
	hbn_vnode_image_t sth_out_img;
} test_ctx_t;

int read_yuv420_file(const char *filename, char *addr0, char *addr1, uint32_t y_size);
int32_t load_file_2_buff(const char *path, char *filebuff, int32_t size);
int dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1, unsigned int size, unsigned int size1);

void show_cfg(test_ctx_t *res_cfg);
#endif
