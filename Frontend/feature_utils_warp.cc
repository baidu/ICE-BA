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

namespace XP {
namespace warp {

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;

// Compute affine warp matrix A_ref_cur
// The warping matrix is warping the ref patch (at level_ref) to the current frame (at pyr 0)
bool getWarpMatrixAffine(const vio::cameras::CameraBase& cam_ref,
                         const vio::cameras::CameraBase& cam_cur,
                         const Vector2f& px_ref,  // distorted pixel at pyr0
                         const Vector3f& f_ref,   // undist ray in unit plane
                         const float depth_ref,
                         const Matrix3f& R_cur_ref,
                         const Vector3f& t_cur_ref,
                         const int level_ref,
                         Eigen::Matrix2f* A_cur_ref) {
  CHECK_NOTNULL(A_cur_ref);
  // TODO(mingyu): tune the *d_unit* size in pixel for different 1st order approximation
  const int halfpatch_size = 5;
  const Vector3f xyz_ref(f_ref * depth_ref);
  float d_unit = halfpatch_size * (1 << level_ref);
  Vector2f du_ref(px_ref + Vector2f(d_unit, 0));
  Vector2f dv_ref(px_ref + Vector2f(0, d_unit));
  Vector3f xyz_du_ref, xyz_dv_ref;
  if (cam_ref.backProject(du_ref, &xyz_du_ref) && cam_ref.backProject(dv_ref, &xyz_dv_ref)) {
    // Make sure the back project succeed for both du_ref & dv_ref
    xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
    xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
    Vector2f px_cur, du_cur, dv_cur;
    if (vio::cameras::CameraBase::ProjectionStatus::Successful ==
        cam_cur.project(R_cur_ref * xyz_ref + t_cur_ref, &px_cur) &&
        vio::cameras::CameraBase::ProjectionStatus::Successful ==
            cam_cur.project(R_cur_ref * xyz_du_ref + t_cur_ref, &du_cur) &&
        vio::cameras::CameraBase::ProjectionStatus::Successful ==
            cam_cur.project(R_cur_ref * xyz_dv_ref + t_cur_ref, &dv_cur)) {
      A_cur_ref->col(0) = (du_cur - px_cur) / halfpatch_size;
      A_cur_ref->col(1) = (dv_cur - px_cur) / halfpatch_size;
      return true;
    }
  }
  A_cur_ref->setIdentity();  // No warping
  return false;
}

// Compute patch level in other image (based on pyramid level 0)
int getBestSearchLevel(const Eigen::Matrix2f& A_cur_ref,
                       const int max_level) {
  int search_level = 0;
  float D = A_cur_ref.determinant();
  while (D > 3.f && search_level < max_level) {
    ++search_level;
    D *= 0.25;
  }
  return search_level;
}

namespace {
// Return value between 0 and 255
// [NOTE] Does not check whether the x/y is within the border
inline float interpolateMat_8u(const cv::Mat& mat, float u, float v) {
  CHECK_EQ(mat.type(), CV_8U);
  int x = floor(u);
  int y = floor(v);
  float subpix_x = u - x;
  float subpix_y = v - y;

  float w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
  float w01 = (1.0f - subpix_x) * subpix_y;
  float w10 = subpix_x * (1.0f - subpix_y);
  float w11 = 1.0f - w00 - w01 - w10;

  const int stride = mat.step.p[0];
  uint8_t* ptr = mat.data + y * stride + x;
  return w00 * ptr[0] + w01 * ptr[stride] + w10 * ptr[1] + w11 * ptr[stride+1];
}
}  // namespace

// Compute a squared patch that is *warperd* from img_ref with A_cur_ref.
bool warpAffine(const Eigen::Matrix2f& A_cur_ref,
                const cv::Mat& img_ref,         // at pyramid level_ref
                const Eigen::Vector2f& px_ref,  // at pyramid 0
                const int level_ref,
                const int level_cur,
                const int halfpatch_size,
                uint8_t* patch) {
  const int patch_size = halfpatch_size * 2;
  const Matrix2f A_ref_cur = A_cur_ref.inverse();
  if (std::isnan(A_ref_cur(0, 0))) {
    // TODO(mingyu): Use looser criteria for invalid affine warp?
    //               I suspect A_ref_cur can barely hit NaN.
    LOG(ERROR) << "Invalid affine warp matrix (NaN)";
    return false;
  }

  // px_ref is at pyr0, img_ref is at level_ref pyr already
  CHECK_NOTNULL(patch);
  uint8_t* patch_ptr = patch;
  const Vector2f px_ref_pyr = px_ref / (1<< level_ref);  // pixel at pyramid level_ref
  for (int y = 0; y < patch_size; ++y) {
    for (int x = 0; x < patch_size; ++x, ++patch_ptr) {
      Vector2f px_patch(x - halfpatch_size, y - halfpatch_size);
      px_patch *= (1 << level_cur);
      const Vector2f px(A_ref_cur * px_patch + px_ref_pyr);  // pixel at pyramid level_ref
      if (px[0] < 0 || px[1] < 0 || px[0] >= img_ref.cols - 1 || px[1] >= img_ref.rows - 1) {
        *patch_ptr = 0;
      } else {
        *patch_ptr = interpolateMat_8u(img_ref, px[0], px[1]);
      }
    }
  }
  return true;
}

}  // namespace warp
}  // namespace XP
