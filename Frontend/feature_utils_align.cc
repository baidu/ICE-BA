/******************************************************************************
 * Copyright 2017 Baidu Robotic Vision Authors. All Rights Reserved.
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
#include "feature_utils.h"
#include <Eigen/Dense>

#define SUBPIX_VERBOSE 0

namespace XP {
namespace align {

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix3i;
const int W_BITS = 14;
const int half_patch_size = 4;
const int patch_size = 8;
const int patch_area = 64;
bool align1D(const cv::Mat& cur_img,
             const Vector2f& dir,  // direction in which the patch is allowed to move
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             Vector2f* cur_px_estimate,
             float* h_inv) {
  bool converged = false;

  // compute derivative of template and prepare inverse compositional
  float __attribute__((__aligned__(16))) ref_patch_dv[patch_area];
  Matrix2f H; H.setZero();

  // compute gradient and hessian
  const int ref_step = patch_size + 2;
  float* it_dv = ref_patch_dv;
  for (int y=0; y < patch_size; ++y) {
    uint8_t* it = ref_patch_with_border + (y + 1) * ref_step + 1;
    for (int x = 0; x < patch_size; ++x, ++it, ++it_dv) {
      Vector2f J;
      J[0] = 0.5*(dir[0] * (it[1] - it[-1]) + dir[1] * (it[ref_step] - it[-ref_step]));
      J[1] = 1;
      *it_dv = J[0];
      H += J * J.transpose();
    }
  }
  *h_inv = 1.0 / H(0, 0) * patch_size * patch_size;
  Matrix2f Hinv = H.inverse();
  float mean_diff = 0;

  // Compute pixel location in new image:
  float u = (*cur_px_estimate)(0);
  float v = (*cur_px_estimate)(1);

  // termination condition
  const float min_update_squared = 0.03 * 0.03;
  const int cur_step = cur_img.step.p[0];
  float chi2 = 0;
  Vector2f update; update.setZero();
  for (int iter = 0; iter < n_iter; ++iter) {
    int u_r = floor(u);
    int v_r = floor(v);
    if (u_r < half_patch_size || v_r < half_patch_size ||
        u_r >= cur_img.cols-half_patch_size || v_r >= cur_img.rows - half_patch_size) {
      break;
    }
    if (std::isnan(u) || std::isnan(v)) {
      // TODO(SVO): very rarely this can happen, maybe H is singular? should not be at corner.
      return false;
    }

    // compute interpolation weights
    float subpix_x = u - u_r;
    float subpix_y = v - v_r;
    float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
    float wTR = subpix_x * (1.0 - subpix_y);
    float wBL = (1.0 - subpix_x) * subpix_y;
    float wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = ref_patch;
    float* it_ref_dv = ref_patch_dv;
    float new_chi2 = 0.0;
    Vector2f Jres; Jres.setZero();
    for (int y = 0; y < patch_size; ++y) {
      uint8_t* it = static_cast<uint8_t*>(cur_img.data) +
          (v_r + y - half_patch_size) * cur_step + u_r - half_patch_size;
      for (int x = 0; x < patch_size; ++x, ++it, ++it_ref, ++it_ref_dv) {
        float search_pixel = wTL * it[0] + wTR * it[1] + wBL * it[cur_step] + wBR * it[cur_step+1];
        float res = search_pixel - *it_ref + mean_diff;
        Jres[0] -= res * (*it_ref_dv);
        Jres[1] -= res;
        new_chi2 += res * res;
      }
    }

    if (iter > 0 && new_chi2 > chi2) {
#if SUBPIX_VERBOSE
      cout << "error increased." << endl;
#endif
      u -= update[0];
      v -= update[1];
      break;
    }

    chi2 = new_chi2;
    update = Hinv * Jres;
    u += update[0] * dir[0];
    v += update[0] * dir[1];
    mean_diff += update[1];

#if SUBPIX_VERBOSE
    VLOG(2) << "Iter " << iter << ":"
            << "\t u=" << u << ", v=" << v
            << "\t update = " << update[0] << ", " << update[1]
            << "\t new chi2 = " << new_chi2;
#endif

    if (update[0] * update[0] + update[1] * update[1] < min_update_squared) {
#if SUBPIX_VERBOSE
      VLOG(2) << "converged.";
#endif
      converged = true;
      break;
    }
  }

  *cur_px_estimate << u, v;
  return converged;
}



bool align2D(const cv::Mat& cur_img,
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             Vector2f* cur_px_estimate,
             bool no_simd) {
/*
#ifdef __ARM_NEON__
  Vector2f cur_px_estimate_neon = *cur_px_estimate;
  if (!no_simd) {
    align2D_NEON(cur_img, ref_patch_with_border, ref_patch, n_iter, &cur_px_estimate_neon);
  }
#endif
*/
  bool converged = false;

  // compute derivative of template and prepare inverse compositional
  float __attribute__((__aligned__(16))) ref_patch_dx[patch_area];
  float __attribute__((__aligned__(16))) ref_patch_dy[patch_area];
  Matrix3f H; H.setZero();

  // compute gradient and hessian
  const int ref_step = patch_size+2;
  float* it_dx = ref_patch_dx;
  float* it_dy = ref_patch_dy;
  for (int y = 0; y < patch_size; ++y) {
    uint8_t* it = ref_patch_with_border + (y + 1) * ref_step + 1;
    for (int x = 0; x < patch_size; ++x, ++it, ++it_dx, ++it_dy) {
      Vector3f J;
      J[0] = 0.5 * (it[1] - it[-1]);
      J[1] = 0.5 * (it[ref_step] - it[-ref_step]);
      J[2] = 1;
      *it_dx = J[0];
      *it_dy = J[1];
      H += J*J.transpose();
    }
  }
  Matrix3f Hinv = H.inverse();
  float mean_diff = 0;

  // Compute pixel location in new image:
  float u = (*cur_px_estimate)(0);
  float v = (*cur_px_estimate)(1);

  // termination condition
  const float min_update_squared = 0.03 * 0.03;
  const int cur_step = cur_img.step.p[0];
  float chi2 = std::numeric_limits<int>::max();
  Vector3f update; update.setZero();
  for (int iter = 0; iter < n_iter; ++iter) {
    int u_r = floor(u);
    int v_r = floor(v);
    if (u_r < half_patch_size || v_r < half_patch_size ||
        u_r >= cur_img.cols-half_patch_size || v_r >= cur_img.rows-half_patch_size) {
      break;
    }
    if (std::isnan(u) || std::isnan(v)) {
      // TODO(SVO): very rarely this can happen, maybe H is singular? should not be at corner
      return false;
    }

    // compute interpolation weights
    float subpix_x = u - u_r;
    float subpix_y = v - v_r;
    float wTL = (1.0 - subpix_x) * (1.0 - subpix_y);
    float wTR = subpix_x * (1.0 - subpix_y);
    float wBL = (1.0 - subpix_x) * subpix_y;
    float wBR = subpix_x * subpix_y;

    // loop through search_patch, interpolate
    uint8_t* it_ref = ref_patch;
    float* it_ref_dx = ref_patch_dx;
    float* it_ref_dy = ref_patch_dy;
    float new_chi2 = 0.0;
    Vector3f Jres; Jres.setZero();
    for (int y = 0; y < patch_size; ++y) {
      uint8_t* it = static_cast<uint8_t*>(cur_img.data) +
          (v_r + y - half_patch_size) * cur_step + u_r - half_patch_size;
      for (int x = 0; x < patch_size; ++x, ++it, ++it_ref, ++it_ref_dx, ++it_ref_dy) {
        float search_pixel = wTL*it[0] + wTR*it[1] + wBL*it[cur_step] + wBR*it[cur_step+1];
        float res = search_pixel - *it_ref + mean_diff;
        Jres[0] -= res*(*it_ref_dx);
        Jres[1] -= res*(*it_ref_dy);
        Jres[2] -= res;
        new_chi2 += res*res;
      }
    }

    /*
    if(iter > 0 && new_chi2 > chi2) {
#if SUBPIX_VERBOSE
      VLOG(2) << "error increased.";
#endif
      u -= update[0];
      v -= update[1];
      break;
    }
    */
    chi2 = new_chi2;

    update = Hinv * Jres;
    u += update[0];
    v += update[1];
    mean_diff += update[2];

#if SUBPIX_VERBOSE
    VLOG(2) << "Iter " << iter << ":"
            << "\t u=" << u << ", v=" << v
            << "\t update = " << update[0] << ", " << update[1]
            << "\t chi2 = " << chi2;
#endif

    if (update[0] * update[0] + update[1] * update[1] < min_update_squared) {
#if SUBPIX_VERBOSE
      VLOG(2) << "converged.";
#endif
      converged = true;
      break;
    }
  }
  *cur_px_estimate << u, v;
  return converged;
}

// TODO(mingyu): Add the SSE and NEON version
bool align2D_SSE2(const cv::Mat& cur_img,
                  uint8_t* ref_patch_with_border,
                  uint8_t* ref_patch,
                  const int n_iter,
                  Vector2f* cur_px_estimate) {
  LOG(FATAL) << "align2D_SSE2 is not implemented";
  return false;
}
#ifdef __ARM_NEON__
bool align2D_NEON(const cv::Mat& cur_img,
                  const uint8_t* ref_patch_with_border,
                  const uint8_t* ref_patch,
                  const int n_iter,
                  Vector2f* cur_px_estimate) {
  bool converged = false;

  // compute derivative of template and prepare inverse compositional
  /*************************************
   * memory layout
   * dx dy 1 dx dy 1 dx dy 1 ...
   */
  float __attribute__((__aligned__(16))) ref_patch_dx_dy[patch_area * 3];
  Matrix3f H = Matrix3f::Zero();

  // compute gradient
  const int ref_step = patch_size + 2;
  for (int n = 0; n < patch_size; ++n) {
    const uint8_t* it = ref_patch_with_border + (n + 1) * ref_step + 1;
    int16x8_t horizontal_diff, vertical_diff;
    {
      int16x8_t raw_data_mid_left = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it - 1)));
      int16x8_t raw_data_mid_right = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it + 1)));
      int16x8_t raw_data_top = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it - ref_step)));
      int16x8_t raw_data_bot = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it + ref_step)));
      horizontal_diff = vsubq_s16(raw_data_mid_right, raw_data_mid_left);
      vertical_diff = vsubq_s16(raw_data_bot, raw_data_top);
    }
    float32x4x3_t left4 = {
      vmulq_n_f32(vcvtq_f32_s32(vmovl_s16(vget_low_s16(horizontal_diff))), 0.5),
      vmulq_n_f32(vcvtq_f32_s32(vmovl_s16(vget_low_s16(vertical_diff))), 0.5),
      vdupq_n_f32(1.f)
    };
    vst3q_f32(ref_patch_dx_dy + n * 24, left4);
    float32x4x3_t right4 = {
      vmulq_n_f32(vcvtq_f32_s32(vmovl_s16(vget_high_s16(horizontal_diff))), 0.5),
      vmulq_n_f32(vcvtq_f32_s32(vmovl_s16(vget_high_s16(vertical_diff))), 0.5),
      vdupq_n_f32(1.f)
    };
    vst3q_f32(ref_patch_dx_dy + n * 24 + 12, right4);
  }
  // compute hessian
  float* ptr_dxy = ref_patch_dx_dy;
  for (int cnt = patch_size * patch_size; cnt > 0; --cnt) {
    Eigen::Map<Vector3f> J(ptr_dxy);
    H += J * J.transpose();
    ptr_dxy += 3;
  }

  Matrix3f Hinv = H.inverse();
  float mean_diff = 0.f;

  // Compute pixel location in new image:
  float u = (*cur_px_estimate)(0);
  float v = (*cur_px_estimate)(1);

  // termination condition
  const float min_update_squared = 0.03 * 0.03;
  const int cur_step = cur_img.step.p[0];
  float chi2 = std::numeric_limits<int>::max();
  Vector3f update = Vector3f::Zero();
  for (int iter = 0; iter < n_iter; ++iter) {
    int u_r = floor(u);
    int v_r = floor(v);
    if (u_r < half_patch_size ||
        v_r < half_patch_size ||
        u_r >= cur_img.cols - half_patch_size ||
        v_r >= cur_img.rows - half_patch_size) {
      break;
    }
    if (std::isnan(u) || std::isnan(v)) {
      // TODO(SVO): very rarely this can happen, maybe H is singular? should not be at corner
      return false;
    }

    // compute interpolation weights
    float subpix_x = u - u_r;
    float subpix_y = v - v_r;

    int iw00 = cvRound((1.f - subpix_x) * (1.f - subpix_y) * (1 << W_BITS));
    int iw01 = cvRound(subpix_x * (1.f - subpix_y) * (1 << W_BITS));
    int iw10 = cvRound((1.f - subpix_x) * subpix_y * (1 << W_BITS));
    int iw11 = (1 << W_BITS) - iw00 - iw01 - iw10;
    const int16x4_t viw00 = vdup_n_s16(static_cast<int16_t>(iw00));
    const int16x4_t viw01 = vdup_n_s16(static_cast<int16_t>(iw01));
    const int16x4_t viw10 = vdup_n_s16(static_cast<int16_t>(iw10));
    const int16x4_t viw11 = vdup_n_s16(static_cast<int16_t>(iw11));
    const int32x4_t shift = vdupq_n_s32(-W_BITS);
    const float32x4_t vmean_diff = vdupq_n_f32(mean_diff);

    // loop through search_patch, interpolate
    const uint8_t* it_ref = ref_patch;
    float new_chi2 = 0.f;
    Vector3f Jres = Vector3f::Zero();
    for (int n = 0; n < patch_size; ++n, it_ref += 8) {
      uint8_t* it = static_cast<uint8_t*>(cur_img.data) +
          (v_r + n - half_patch_size) * cur_step + u_r - half_patch_size;
      int16x8_t topleft = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it)));
      int16x8_t topright = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it + 1)));
      int16x8_t botleft = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it + cur_step)));
      int16x8_t botright = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it + cur_step + 1)));

      int32x4_t left_half1 = vaddq_s32(
                  vmull_s16(vget_low_s16(topleft), viw00),
                  vmull_s16(vget_low_s16(topright), viw01));
      int32x4_t left_half2 = vaddq_s32(
                  vmull_s16(vget_low_s16(botleft), viw10),
                  vmull_s16(vget_low_s16(botright), viw11));

      int32x4_t right_half1 = vaddq_s32(
                  vmull_s16(vget_high_s16(topleft), viw00),
                  vmull_s16(vget_high_s16(topright), viw01));
      int32x4_t right_half2 = vaddq_s32(
                  vmull_s16(vget_high_s16(botleft), viw10),
                  vmull_s16(vget_high_s16(botright), viw11));

      int16x8_t v_it_ref8 = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(it_ref)));
      float32x4x2_t search_pixel = {
        vcvtq_f32_s32(vqrshlq_s32(vaddq_s32(left_half1, left_half2), shift)),
        vcvtq_f32_s32(vqrshlq_s32(vaddq_s32(right_half1, right_half2), shift))
      };

      search_pixel.val[0] = vsubq_f32(search_pixel.val[0],
                           vcvtq_f32_s32(vmovl_s16(vget_low_s16(v_it_ref8))));
      search_pixel.val[1] = vsubq_f32(search_pixel.val[1],
                           vcvtq_f32_s32(vmovl_s16(vget_high_s16(v_it_ref8))));
      search_pixel.val[0] = vaddq_f32(search_pixel.val[0], vmean_diff);
      search_pixel.val[1] = vaddq_f32(search_pixel.val[1], vmean_diff);

      float32x4x3_t dxy_left = vld3q_f32(ref_patch_dx_dy + n * 24);
      float32x4x3_t dxy_right = vld3q_f32(ref_patch_dx_dy + n * 24 + 12);

      // dx
      dxy_left.val[0]  = vmulq_f32(dxy_left.val[0], search_pixel.val[0]);
      dxy_right.val[0] = vmulq_f32(dxy_right.val[0], search_pixel.val[1]);
      // dy
      dxy_left.val[1]  = vmulq_f32(dxy_left.val[1], search_pixel.val[0]);
      dxy_right.val[1] = vmulq_f32(dxy_right.val[1], search_pixel.val[1]);

      Jres[2] -= (vgetq_lane_f32(search_pixel.val[0], 0) + vgetq_lane_f32(search_pixel.val[0], 1) +
                  vgetq_lane_f32(search_pixel.val[0], 2) + vgetq_lane_f32(search_pixel.val[0], 3) +
                  vgetq_lane_f32(search_pixel.val[1], 0) + vgetq_lane_f32(search_pixel.val[1], 1) +
                  vgetq_lane_f32(search_pixel.val[1], 2) + vgetq_lane_f32(search_pixel.val[1], 3));

      search_pixel.val[0] = vmulq_f32(search_pixel.val[0], search_pixel.val[0]);
      search_pixel.val[1] = vmulq_f32(search_pixel.val[1], search_pixel.val[1]);
      dxy_left.val[0] = vaddq_f32(dxy_left.val[0], dxy_right.val[0]);
      dxy_left.val[1] = vaddq_f32(dxy_left.val[1], dxy_right.val[1]);
      Jres[0] -= (vgetq_lane_f32(dxy_left.val[0], 0)  + vgetq_lane_f32(dxy_left.val[0], 1) +
                  vgetq_lane_f32(dxy_left.val[0], 2)  + vgetq_lane_f32(dxy_left.val[0], 3));
      Jres[1] -= (vgetq_lane_f32(dxy_left.val[1], 0)  + vgetq_lane_f32(dxy_left.val[1], 1) +
                  vgetq_lane_f32(dxy_left.val[1], 2)  + vgetq_lane_f32(dxy_left.val[1], 3));
      search_pixel.val[0] = vaddq_f32(search_pixel.val[0], search_pixel.val[1]);
      new_chi2 += vgetq_lane_f32(search_pixel.val[0], 0) + vgetq_lane_f32(search_pixel.val[0], 1) +
                  vgetq_lane_f32(search_pixel.val[0], 2) + vgetq_lane_f32(search_pixel.val[0], 3);
    }
    /*
    if(iter > 0 && new_chi2 > chi2) {
#if SUBPIX_VERBOSE
      VLOG(2) << "error increased.";
#endif
      u -= update[0];
      v -= update[1];
      break;
    }
    */
    chi2 = new_chi2;

    update = Hinv * Jres;
    u += update[0];
    v += update[1];
    mean_diff += update[2];

#if SUBPIX_VERBOSE
    VLOG(2) << "Iter " << iter << ":"
            << "\t u=" << u << ", v=" << v
            << "\t update = " << update[0] << ", " << update[1]
            << "\t chi2 = " << chi2;
#endif

    if (update[0] * update[0] + update[1] * update[1] < min_update_squared) {
#if SUBPIX_VERBOSE
      VLOG(2) << "converged.";
#endif
      converged = true;
      break;
    }
  }

  *cur_px_estimate << u, v;
  return converged;
}
#endif
}  // namespace align
}  // namespace XP
