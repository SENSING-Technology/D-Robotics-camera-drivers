#ifndef NEON_NV12_RESIZE_H
#define NEON_NV12_RESIZE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NV12图像缩放函数（NEON优化）
 *
 * @param src 输入图像数据（NV12格式）
 * @param dst 输出图像数据（预分配内存）
 * @param src_w 源图像宽度
 * @param src_h 源图像高度
 * @param dst_w 目标图像宽度
 * @param dst_h 目标图像高度
 */
void neon_nv12_resize(uint8_t* src, int src_w, int src_h,
                      uint8_t* dst, int dst_w, int dst_h);

void neon_nv12_resize_2plane(uint8_t* src_y, uint8_t* src_uv,int src_w, int src_h,
                      uint8_t* dst_y, uint8_t* dst_uv, int dst_w, int dst_h);
#ifdef __cplusplus
}
#endif

#endif // NEON_NV12_RESIZE_H
