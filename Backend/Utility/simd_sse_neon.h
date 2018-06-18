/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef _UTILITY_SIMD_NEON_H_
#define _UTILITY_SIMD_NEON_H_
#if defined __SSE__ || defined _MSC_VER
#define __IBA_SSE__
#endif

#ifdef __IBA_SSE__
#include <xmmintrin.h>
#endif // __IBA_SSE__

#ifdef __SSE2__
#include <emmintrin.h>
#endif // __SSE2__

#ifdef __SSE4_1__
#include <smmintrin.h>
#endif // __SSE4_1__

#include <string.h>     // for memcpy
#include <cmath>

#include "../IBA/IBA_config.h"

#include <climits>      // for USHRT_MAX

#ifdef __ARM_NEON__
#include <arm_neon.h>
#endif
//BYTE-ALIGNMENT for data allocation (16 required for SSE, 32 required for AVX)
//PREVIOUS version uses only SSE. The new version will include AVX.
//SO the alignment is increased from 16 to 32
#define SIMD_ALIGN 16
#ifdef _WIN32
// _CRT_ALIGN(X) is __declspec(align(X))
#  define SIMD_ALIGN_DECLSPEC _CRT_ALIGN(SIMD_ALIGN)
#else
// This requires c++11 to work
#  define SIMD_ALIGN_DECLSPEC __attribute__((aligned(SIMD_ALIGN)))
#endif

struct SIMD_ALIGN_DECLSPEC xp128f {
 public:
  //***************************Scaler Operation******************/
  // non-inplace operation
  static inline xp128f get(const float s) {
    xp128f tmp;
    tmp.vdup_all_lane(s);
    return tmp;
  }
  static inline xp128f get(const float x0, const float x1, const float x2, const float x3) {
    xp128f tmp;
    tmp.vset_all_lane(x0, x1, x2, x3);
    return tmp;
  }
  static inline xp128f get(const float *src) {
    xp128f tmp;
    tmp.vload_unalign(src);
    return tmp;
  }
  inline xp128f operator+(const float& s) const {
    xp128f tmp;
    tmp.vdup_all_lane(s);
#ifdef __IBA_SSE__
    tmp.vec = _mm_add_ps(vec, tmp.vec);
#elif __ARM_NEON__
    tmp.vec = vaddq_f32(vec, tmp.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] += val[i];
    }
#endif
    return tmp;
  }
  inline xp128f operator-(const float& s) const {
    xp128f tmp;
    tmp.vdup_all_lane(s);
#ifdef __IBA_SSE__
    tmp.vec = _mm_sub_ps(vec, tmp.vec);
#elif __ARM_NEON__
    tmp.vec = vsubq_f32(vec, tmp.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] = val[i] - tmp.val[i];
    }
#endif
    return tmp;
  }
  inline xp128f operator*(const float& s) const {
    xp128f tmp;
    tmp.vdup_all_lane(s);
#ifdef __IBA_SSE__
    tmp.vec = _mm_mul_ps(vec, tmp.vec);
#elif __ARM_NEON__
    tmp.vec = vmulq_f32(vec, tmp.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] *= val[i];
    }
#endif
    return tmp;
  }

  // inplace operation
  inline void operator+=(const float& s) {
#ifdef __IBA_SSE__
    __m128 _s = _mm_set1_ps(s);
    vec = _mm_add_ps(vec, _s);
#elif __ARM_NEON__
    float32x4_t _s = vdupq_n_f32(s);
    vec = vaddq_f32(vec, _s);
#else
    for(int i = 0; i < 4; ++i) {
      val[i] += s;
    }
#endif
  }
  inline void operator-=(const float& s) {
#ifdef __IBA_SSE__
    __m128 _s = _mm_set1_ps(s);
    vec = _mm_sub_ps(vec, _s);
#elif __ARM_NEON__
    float32x4_t _s = vdupq_n_f32(s);
    vec = vsubq_f32(vec, _s);
#else
    for(int i = 0; i < 4; ++i) {
      val[i] -= s;
    }
#endif
  }
  inline void operator*=(const float& s) {
#ifdef __IBA_SSE__
    __m128 _s = _mm_set1_ps(s);
    vec = _mm_mul_ps(vec, _s);
#elif __ARM_NEON__
    vec = vmulq_n_f32(vec, s);
#else
    for(int i = 0; i < 4; ++i) {
      val[i] *= s;
    }
#endif
  }

  //***************************vector Operation******************/
  // element-wise non-inplace operation
  inline const xp128f operator+(const xp128f& other) const {
    xp128f tmp;
#ifdef __IBA_SSE__
    tmp.vec = _mm_add_ps(vec, other.vec);
#elif __ARM_NEON__
    tmp.vec = vaddq_f32(vec, other.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] = val[i] + other.val[i];
    }
#endif
    return tmp;
  }
  // element-wise subtraction
  inline const xp128f operator-(const xp128f& other) const {
    xp128f tmp;
#ifdef __IBA_SSE__
    tmp.vec = _mm_sub_ps(vec, other.vec);
#elif __ARM_NEON__
    tmp.vec = vsubq_f32(vec, other.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] = val[i] - other.val[i];
    }
#endif
    return tmp;
  }
  // element-wise multiplication
  inline const xp128f operator*(const xp128f& other) const {
    xp128f tmp;
#ifdef __IBA_SSE__
    tmp.vec = _mm_mul_ps(vec, other.vec);
#elif __ARM_NEON__
    tmp.vec = vmulq_f32(vec, other.vec);
#else
    for (int i = 0; i < 4; ++i) {
      tmp.val[i] = val[i] * other.val[i];
    }
#endif
    return tmp;
  }
  // element-wise inplace operation
  inline void operator+=(const xp128f& other) {
#ifdef __IBA_SSE__
    vec = _mm_add_ps(vec, other.vec);
#elif __ARM_NEON__
    vec = vaddq_f32(vec, other.vec);
#else
    val[0] += other.val[0];
    val[1] += other.val[1];
    val[2] += other.val[2];
    val[3] += other.val[3];
#endif
  }

  inline void operator-=(const xp128f& other) {
#ifdef __IBA_SSE__
    vec = _mm_sub_ps(vec, other.vec);
#elif __ARM_NEON__
    vec = vsubq_f32(vec, other.vec);
#else
    val[0] -= other.val[0];
    val[1] -= other.val[1];
    val[2] -= other.val[2];
    val[3] -= other.val[3];
#endif
  }

  inline void operator*=(const xp128f& other) {
#ifdef __IBA_SSE__
    vec = _mm_mul_ps(vec, other.vec);
#elif __ARM_NEON__
    vec = vmulq_f32(vec, other.vec);
#else
    val[0] *= other.val[0];
    val[1] *= other.val[1];
    val[2] *= other.val[2];
    val[3] *= other.val[3];
#endif
  }

  inline float& operator[](int lane) {
    return val[lane];
  }

  inline const float& operator[](int lane) const {
    return val[lane];
  }

  inline void vset_all_lane(const float x0, const float x1 = 0.f,
                            const float x2 = 0.f, const float x3 = 0.f) {
    val[0] = x0;
    val[1] = x1;
    val[2] = x2;
    val[3] = x3;
  }
  // broadcast a to all lanes of xp128f
  inline void vdup_all_lane(const float a) {
#ifdef __IBA_SSE__
    vec = _mm_set1_ps(a);
#elif __ARM_NEON__
    vec = vdupq_n_f32(a);
#else
    val[0] = a;
    val[1] = a;
    val[2] = a;
    val[3] = a;
#endif
  }
  inline float vsum_all(void) const {
    return val[0] + val[1] + val[2] + val[3];
  }
  inline float vsum_all(float* s012) const {
    *s012 = val[0] + val[1] + val[2];
    return *s012 + val[3];
  }

  inline float vsum_012(void) const {
    return val[0] + val[1] + val[2];
  }
  inline void normalize012() {
#ifdef __IBA_SSE__
    __m128 vtmp = vec;
    vec = _mm_mul_ps(vec, vec);
    vec = _mm_mul_ps(_mm_set1_ps(1.0f / sqrtf(vsum_012())), vtmp);
#elif __ARM_NEON__
    float32x4_t vtmp = vec;
    vec = vmulq_f32(vec, vec);
    vec = vmulq_n_f32(vec, 1.0f / sqrtf(vsum_012()));
#else
    float div = 1.f / sqrtf(val[0] * val[0] + val[1] * val[1] + val[2] * val[2]);
    val[0] *= div;
    val[1] *= div;
    val[2] *= div;
    val[3] *= div;
#endif
  }

  inline void vmax_with_other(const xp128f& other) {
#ifdef __IBA_SSE__
    vec = _mm_max_ps(other.vec, vec);
#elif __ARM_NEON__
    vec = vmaxq_f32(other.vec, vec);
#else
    val[0] = val[0] > other.val[0] ? val[0] : other.val[0];
    val[1] = val[1] > other.val[1] ? val[1] : other.val[1];
    val[2] = val[2] > other.val[2] ? val[2] : other.val[2];
    val[3] = val[3] > other.val[3] ? val[3] : other.val[3];
#endif
  }

  inline void vmin_with_other(const xp128f& other) {
#ifdef __IBA_SSE__
    vec = _mm_min_ps(other.vec, vec);
#elif __ARM_NEON__
    vec = vminq_f32(other.vec, vec);
#else
    val[0] = val[0] < other.val[0] ? val[0] : other.val[0];
    val[1] = val[1] < other.val[1] ? val[1] : other.val[1];
    val[2] = val[2] < other.val[2] ? val[2] : other.val[2];
    val[3] = val[3] < other.val[3] ? val[3] : other.val[3];
#endif
  }

  inline xp128f vinverse_multiply(const xp128f& other) const {
    xp128f res;
    for (int i = 0; i < 4; ++i) {
      res.val[i] = other.val[i] / val[i];
    }
    return res;
  }

  // return the maximal element of xp128f
  inline float vmaximal() const {
#ifndef __ARM_NEON__
    return std::max(std::max(val[0], val[1]), std::max(val[2], val[3]));
#else
    float32x2_t tmp = vpmax_f32(vget_low_f32(vec), vget_high_f32(vec));
    return std::max(vget_lane_f32(tmp, 0), vget_lane_f32(tmp, 1));
#endif
  }
  // return the minimal element of xp128f
  inline float vminimal() const {
#ifndef __ARM_NEON__
    return std::min(std::min(val[0], val[1]), std::min(val[2], val[3]));
#else
    float32x2_t tmp = vpmin_f32(vget_low_f32(vec), vget_high_f32(vec));
    return std::min(vget_lane_f32(tmp, 0), vget_lane_f32(tmp, 1));
#endif
  }
  // loading data from an aligned address, 16 bytes.
  inline void vload_align(const float* src) {
#ifdef __IBA_SSE__
    vec = _mm_load_ps(src);
#elif __ARM_NEON__
    vec = vld1q_f32(src);
#else
    memcpy(val, src, sizeof(float) * 4);
#endif
  }
  // loading data from address, NO alignment required.
  inline void vload_unalign(const float* src) {
#ifdef __IBA_SSE__
    vec = _mm_loadu_ps(src);
#elif __ARM_NEON__
    vec = vld1q_f32(src);
#else
    memcpy(val, src, sizeof(float) * 4);
#endif
  }

  // store data to address, NO alignment required.
  inline void vstore_unalign(float* dst) const {
#ifdef __IBA_SSE__
    _mm_storeu_ps(dst, vec);
#elif __ARM_NEON__
    vst1q_f32(dst, vec);
#else
    memcpy(dst, val, sizeof(float) * 4);
#endif
  }
  // operation: v = -v;
  inline void vmake_minus() {
#ifdef __IBA_SSE__
    vec = _mm_sub_ps(_mm_set1_ps(0.f), vec);
#elif __ARM_NEON__
    // TODO (yanghongtian) :
    vec = vmulq_n_f32(vec, -1.f);
#else
    val[0] = -val[0];
    val[1] = -val[1];
    val[2] = -val[2];
    val[3] = -val[3];
#endif
  }
  inline void vmake_sqrt() {
#ifdef __IBA_SSE__
    vec = _mm_sqrt_ps(vec);
#elif __ARM_NEON__
    vec = vrsqrteq_f32(vec);
#else
    val[0] = sqrtf(val[0]);
    val[1] = sqrtf(val[1]);
    val[2] = sqrtf(val[2]);
    val[3] = sqrtf(val[3]);
#endif
  }
  // operation: v = sqrtf(v);
  inline void vset_sqrt_from_vec(const xp128f& v) {
#ifdef __IBA_SSE__
    vec = _mm_sqrt_ps(v.vec);
#elif __ARM_NEON__
    vec = vrsqrteq_f32(v.vec);
#else
    val[0] = sqrtf(v.val[0]);
    val[1] = sqrtf(v.val[1]);
    val[2] = sqrtf(v.val[2]);
    val[3] = sqrtf(v.val[3]);
#endif
  }

  inline float vdot012(const xp128f& v2) const {
    return (*this * v2).vsum_012();
  }

  xp128f vcross012(const xp128f& v2) const {
    xp128f vxv2;
    vxv2[0] = val[1] * v2[2] - val[2] * v2[1];
    vxv2[1] = val[2] * v2[0] - val[0] * v2[2];
    vxv2[2] = val[0] * v2[1] - val[1] * v2[0];
    return vxv2;
  }

  // data member
  union {
#ifdef __IBA_SSE__

    __m128 vec;
#elif __ARM_NEON__
    float32x4_t vec;
#endif
    float val[4];
  };
};
#endif  // _UTILITY_SIMD_NEON_H_
