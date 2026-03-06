#ifndef VP_YNR_H_
#define VP_YNR_H_

#include "vp_common.h"
#ifdef __cplusplus
extern "C" {
#endif

int32_t vp_ynr_init(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_ynr_deinit(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_ynr_start(vp_vflow_contex_t *vp_vflow_contex);
int32_t vp_ynr_stop(vp_vflow_contex_t *vp_vflow_contex);
#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_YNR_H_