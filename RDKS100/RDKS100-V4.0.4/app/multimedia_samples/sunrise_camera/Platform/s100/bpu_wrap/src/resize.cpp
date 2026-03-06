#include "resize.h"
#include <iostream>
#include <fstream>
#include <arm_neon.h>

struct NV12ScaleParams {
    int src_w;
    int src_h;
    int dst_w;
    int dst_h;
    float x_ratio;
    float y_ratio;
    int32_t* x0_array;
    int32_t* x1_array;
    uint8_t* a_array;
};


void init_scale_params(NV12ScaleParams& params, int src_w, int src_h, int dst_w, int dst_h) {
    params.src_w = src_w;
    params.src_h = src_h;
    params.dst_w = dst_w;
    params.dst_h = dst_h;
    params.x_ratio = static_cast<float>(src_w) / dst_w;
    params.y_ratio = static_cast<float>(src_h) / dst_h;

    // 使用posix_memalign替代memalign
    int ret = posix_memalign((void**)&params.x0_array, 64, dst_w * sizeof(int32_t) * 2);
	if(ret != 0){
		exit(-1);
	}
    params.x1_array = params.x0_array + dst_w;
    ret = posix_memalign((void**)&params.a_array, 64, dst_w * sizeof(uint8_t));
	if(ret != 0){
		exit(-1);
	}
    #pragma omp parallel for
    for (int dx = 0; dx < dst_w; ++dx) {
        float sx = dx * params.x_ratio;
        params.x0_array[dx] = static_cast<int>(sx);
        params.x1_array[dx] = std::min(params.x0_array[dx] + 1, src_w - 1);
        params.a_array[dx] = static_cast<uint8_t>((sx - params.x0_array[dx]) * 255);
    }
}

// 释放缩放参数
void free_scale_params(NV12ScaleParams& params) {
    free(params.x0_array);
    free(params.a_array);
}

// 像素收集函数实现
inline void gather_pixels_16(const uint8_t* base,
                            int32x4_t x0, int32x4_t x1,
                            int32x4_t x2, int32x4_t x3,
                            uint8x16_t* result) {
    uint8x8_t low = vld1_u8(base + vgetq_lane_s32(x0, 0));
    low = vld1_lane_u8(base + vgetq_lane_s32(x0, 1), low, 1);
    low = vld1_lane_u8(base + vgetq_lane_s32(x0, 2), low, 2);
    low = vld1_lane_u8(base + vgetq_lane_s32(x0, 3), low, 3);
    low = vld1_lane_u8(base + vgetq_lane_s32(x1, 0), low, 4);
    low = vld1_lane_u8(base + vgetq_lane_s32(x1, 1), low, 5);
    low = vld1_lane_u8(base + vgetq_lane_s32(x1, 2), low, 6);
    low = vld1_lane_u8(base + vgetq_lane_s32(x1, 3), low, 7);

    uint8x8_t high = vld1_u8(base + vgetq_lane_s32(x2, 0));
    high = vld1_lane_u8(base + vgetq_lane_s32(x2, 1), high, 1);
    high = vld1_lane_u8(base + vgetq_lane_s32(x2, 2), high, 2);
    high = vld1_lane_u8(base + vgetq_lane_s32(x2, 3), high, 3);
    high = vld1_lane_u8(base + vgetq_lane_s32(x3, 0), high, 4);
    high = vld1_lane_u8(base + vgetq_lane_s32(x3, 1), high, 5);
    high = vld1_lane_u8(base + vgetq_lane_s32(x3, 2), high, 6);
    high = vld1_lane_u8(base + vgetq_lane_s32(x3, 3), high, 7);

    *result = vcombine_u8(low, high);
}


// UV解交织示例
inline void load_and_deinterleave(const uint8_t* src, int32x4_t x0, int32x4_t x1,
                                 uint8x8x2_t* uv) {
    uint8x8_t data0 = vld1_u8(src + vgetq_lane_s32(x0, 0));
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 1), data0, 1);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 2), data0, 2);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x0, 3), data0, 3);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 0), data0, 4);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 1), data0, 5);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 2), data0, 6);
    data0 = vld1_lane_u8(src + vgetq_lane_s32(x1, 3), data0, 7);

    uint8x8_t data1 = vld1_u8(src + vgetq_lane_s32(x0, 0) + 1);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 1) + 1, data1, 1);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 2) + 1, data1, 2);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x0, 3) + 1, data1, 3);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 0) + 1, data1, 4);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 1) + 1, data1, 5);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 2) + 1, data1, 6);
    data1 = vld1_lane_u8(src + vgetq_lane_s32(x1, 3) + 1, data1, 7);

    (*uv).val[0] = data0;
    (*uv).val[1] = data1;

}

// 双线性插值函数实现（16像素版本）
inline uint8x16_t bilinear_interp_16(uint8x16_t p00, uint8x16_t p01,
                                   uint8x16_t p10, uint8x16_t p11,
                                   uint8x16_t a_coeff, uint8x8_t vec_a, uint8x8_t vec_b) {
    uint8x16_t inv_a_coeff = vsubq_u8(vdupq_n_u8(255), a_coeff);

    // 水平插值
    uint16x8_t h0_low = vmull_u8(vget_low_u8(p00), vget_low_u8(inv_a_coeff));
    h0_low = vmlal_u8(h0_low, vget_low_u8(p01), vget_low_u8(a_coeff));
    uint16x8_t h0_high = vmull_u8(vget_high_u8(p00), vget_high_u8(inv_a_coeff));
    h0_high = vmlal_u8(h0_high, vget_high_u8(p01), vget_high_u8(a_coeff));

    uint16x8_t h1_low = vmull_u8(vget_low_u8(p10), vget_low_u8(inv_a_coeff));
    h1_low = vmlal_u8(h1_low, vget_low_u8(p11), vget_low_u8(a_coeff));
    uint16x8_t h1_high = vmull_u8(vget_high_u8(p10), vget_high_u8(inv_a_coeff));
    h1_high = vmlal_u8(h1_high, vget_high_u8(p11), vget_high_u8(a_coeff));

    uint16x8_t v_low = vmull_u8(vshrn_n_u16(h0_low, 8), vec_a);
    v_low = vmlal_u8(v_low, vshrn_n_u16(h1_low, 8), vec_b);

    uint16x8_t v_high = vmull_u8(vshrn_n_u16(h0_high, 8), vec_a);
    v_high = vmlal_u8(v_high, vshrn_n_u16(h1_high, 8), vec_b);

    // 归一化
    uint8x8_t res_low = vshrn_n_u16(v_low, 8);
    uint8x8_t res_high = vshrn_n_u16(v_high, 8);
    return vcombine_u8(res_low, res_high);
}

// UV插值函数
inline uint8x8_t bilinear_interp_uv(uint8x8_t p00, uint8x8_t p01,
                                   uint8x8_t p10, uint8x8_t p11,
                                   uint8x8_t a_coeff, uint8x8_t vec_a, uint8x8_t vec_b) {
    uint8x8_t inv_a_coeff = vsub_u8(vdup_n_u8(255), a_coeff);

    // 水平插值
    uint16x8_t h0 = vmlal_u8(vmull_u8(p00, inv_a_coeff), p01, a_coeff);
    uint16x8_t h1 = vmlal_u8(vmull_u8(p10, inv_a_coeff), p11, a_coeff);

    // 垂直插值
    uint16x8_t v = vmull_u8(vshrn_n_u16(h0, 8), vec_a);
    v = vmlal_u8(v, vshrn_n_u16(h1, 8), vec_b);

    return vshrn_n_u16(v, 8);
}



// 修正后的Y平面缩放函数
void neon_scale_y(const uint8_t* src_y, uint8_t* dst_y, const NV12ScaleParams& params) {
    const int src_stride = params.src_w;
    const int dst_stride = params.dst_w;

    #pragma omp parallel for
    for (int dy = 0; dy < params.dst_h; ++dy) {
        const float sy = dy * params.y_ratio;
        const int y0 = static_cast<int>(sy);
        const int y1 = std::min(y0 + 1, params.src_h - 1);
        const uint8_t b = static_cast<uint8_t>((sy - y0) * 255);
        const uint8_t a = 255 - b;
        uint8x8_t vec_b = vdup_n_u8(b);
        uint8x8_t vec_a = vsub_u8(vdup_n_u8(255), vec_b);

        const uint8_t* src_row0 = src_y + y0 * src_stride;
        const uint8_t* src_row1 = src_y + y1 * src_stride;
        uint8_t* dst_row = dst_y + dy * dst_stride;
        int dx = 0;
        for (; dx <= params.dst_w - 16; dx += 16) {

            int32x4_t x0_0 = vld1q_s32(params.x0_array + dx);
            int32x4_t x0_1 = vld1q_s32(params.x0_array + dx + 4);
            int32x4_t x0_2 = vld1q_s32(params.x0_array + dx + 8);
            int32x4_t x0_3 = vld1q_s32(params.x0_array + dx + 12);
            int32x4_t x1_0 = vld1q_s32(params.x1_array + dx);
            int32x4_t x1_1 = vld1q_s32(params.x1_array + dx + 4);
            int32x4_t x1_2 = vld1q_s32(params.x1_array + dx + 8);
            int32x4_t x1_3 = vld1q_s32(params.x1_array + dx + 12);

            uint8x16_t a_coeff = vld1q_u8(params.a_array + dx);

            uint8x16_t p00, p01, p10, p11;
            gather_pixels_16(src_row0, x0_0, x0_1, x0_2, x0_3, &p00);
            gather_pixels_16(src_row0, x1_0, x1_1, x1_2, x1_3, &p01);
            gather_pixels_16(src_row1, x0_0, x0_1, x0_2, x0_3, &p10);
            gather_pixels_16(src_row1, x1_0, x1_1, x1_2, x1_3, &p11);


            // ...类似处理其他像素...

            uint8x16_t result = bilinear_interp_16(p00, p01, p10, p11,
                                                 a_coeff, vec_a, vec_b);
            vst1q_u8(dst_row + dx, result);

        }
        // 处理剩余像素（标量代码）
        for (; dx <  params.dst_w; ++dx) {
            const int x0 = params.x0_array[dx];
            const int x1 = params.x1_array[dx];
            const uint8_t a1 = params.a_array[dx];
            const uint8_t inv_a1 = 255 - a1;

            const uint8_t p00 = src_row0[x0];
            const uint8_t p01 = src_row0[x1];
            const uint8_t p10 = src_row1[x0];
            const uint8_t p11 = src_row1[x1];

            // 水平插值
            const uint16_t h0 = p00 * inv_a1 + p01 * a1;
            const uint16_t h1 = p10 * inv_a1 + p11 * a1;

            // 垂直插值
            const uint16_t v = (h0 * a + h1 * b + 32512) / 65025;

            dst_row[dx] = static_cast<uint8_t>(v);
        }
    }
}

// Neon优化UV平面缩放（处理交错UV）
void neon_scale_uv(const uint8_t* src_uv, uint8_t* dst_uv, const NV12ScaleParams& params) {
    const int uv_src_w = params.src_w / 2;
    const int uv_src_h = params.src_h / 2;
    const int uv_dst_w = params.dst_w / 2;
    const int uv_dst_h = params.dst_h / 2;

    NV12ScaleParams uv_params;
    init_scale_params(uv_params, uv_src_w, uv_src_h, uv_dst_w, uv_dst_h);

    #pragma omp parallel for
    for (int dy = 0; dy < uv_dst_h; ++dy) {
        const float sy = dy * uv_params.y_ratio;
        const int y0 = static_cast<int>(sy);
        const int y1 = std::min(y0 + 1, uv_src_h - 1);
        const uint8_t b = static_cast<uint8_t>((sy - y0) * 255);
        const uint8_t a = 255 - b;
        uint8x8_t vec_b = vdup_n_u8(b);
        uint8x8_t vec_a = vsub_u8(vdup_n_u8(255), vec_b);

        const uint8_t* src_row0 = src_uv + y0 * params.src_w;
        const uint8_t* src_row1 = src_uv + y1 * params.src_w;
        uint8_t* dst_row = dst_uv + dy * params.dst_w;

        int dx = 0;
        for (; dx <= uv_dst_w - 8; dx += 8) {
            // 加载UV坐标参数（每个UV对应两个Y像素）
            int32x4_t x0_0 = vshlq_n_s32(vld1q_s32(uv_params.x0_array + dx), 1);
            int32x4_t x0_1 = vshlq_n_s32(vld1q_s32(uv_params.x0_array + dx + 4), 1);
            int32x4_t x1_0 = vshlq_n_s32(vld1q_s32(uv_params.x1_array + dx), 1);
            int32x4_t x1_1 = vshlq_n_s32(vld1q_s32(uv_params.x1_array + dx + 4), 1);

            uint8x8_t a_coeff = vld1_u8(uv_params.a_array + dx);

            // 加载并解交织UV分量
            uint8x8x2_t uv00, uv01, uv10, uv11;
            load_and_deinterleave(src_row0, x0_0, x0_1, &uv00);
            load_and_deinterleave(src_row0, x1_0, x1_1, &uv01);
            load_and_deinterleave(src_row1, x0_0, x0_1, &uv10);
            load_and_deinterleave(src_row1, x1_0, x1_1, &uv11);

            // 分别处理U和V分量
            uint8x8x2_t result;
            result.val[0] = bilinear_interp_uv(uv00.val[0], uv01.val[0],
                                             uv10.val[0], uv11.val[0], a_coeff, vec_a, vec_b);
            result.val[1] = bilinear_interp_uv(uv00.val[1], uv01.val[1],
                                             uv10.val[1], uv11.val[1], a_coeff, vec_a, vec_b);

            vst2_u8(dst_row + dx*2, result);
        }
        // 处理剩余像素（标量代码）
        for (; dx <  uv_dst_w; ++dx) {
            const int x0 = (params.x0_array[dx]) * 2;
            const int x1 = (params.x1_array[dx]) * 2;
            const uint8_t a1 = params.a_array[dx];
            const uint8_t inv_a1 = 255 - a1;

            uint8_t p00 = src_row0[x0];
            uint8_t p01 = src_row0[x1];
            uint8_t p10 = src_row1[x0];
            uint8_t p11 = src_row1[x1];

            // 水平插值
            uint16_t h0 = p00 * inv_a1 + p01 * a1;
            uint16_t h1 = p10 * inv_a1 + p11 * a1;

            // 垂直插值
            uint16_t v = (h0 * a + h1 * b + 32512) / 65025;

            dst_row[dx*2] = static_cast<uint8_t>(v);

            p00 = src_row0[x0+1];
            p01 = src_row0[x1+1];
            p10 = src_row1[x0+1];
            p11 = src_row1[x1+1];

            // 水平插值
            h0 = p00 * inv_a1 + p01 * a1;
            h1 = p10 * inv_a1 + p11 * a1;

            // 垂直插值
            v = (h0 * a + h1 * b + 32512) / 65025;

            dst_row[dx*2+1] = static_cast<uint8_t>(v);
        }
    }
    free_scale_params(uv_params);
}

// NV12缩放主函数
void neon_nv12_resize(uint8_t* src, int src_w, int src_h,
                      uint8_t* dst, int dst_w, int dst_h) {
    NV12ScaleParams params;
    init_scale_params(params, src_w, src_h, dst_w, dst_h);

    const int y_size = src_w * src_h;
    neon_scale_y(src, dst, params);                   // 处理Y平面
    neon_scale_uv(src + y_size, dst + dst_w*dst_h, params); // 处理UV平面
    free_scale_params(params);
}

void neon_nv12_resize_2plane(uint8_t* src_y, uint8_t* src_uv,int src_w, int src_h,
                      uint8_t* dst_y, uint8_t* dst_uv, int dst_w, int dst_h) {
    NV12ScaleParams params;
    init_scale_params(params, src_w, src_h, dst_w, dst_h);

    neon_scale_y(src_y, dst_y, params);                   // 处理Y平面
    neon_scale_uv(src_uv, dst_uv, params); // 处理UV平面
    free_scale_params(params);
}