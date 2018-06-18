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
#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __FEATURE_UTILS_NO_DEBUG__
#endif
#include "feature_utils.h"
#include "patch_score.h"

#include "cameras/PinholeCamera.hpp"
#include "cameras/RadialTangentialDistortion8.hpp"
#include "cameras/RadialTangentialDistortion.hpp"

#include <boost/lexical_cast.hpp>
#ifndef __FEATURE_UTILS_NO_DEBUG__
#include <opencv2/highgui.hpp>
#endif

// define this MACROS to enable profing and verifying align2D_NEON
#undef _ALIGN2D_TIMING_AND_VERIFYING
#ifdef _ALIGN2D_TIMING_AND_VERIFYING
#include <vio/timing/ProfilingTimer.hpp>
using vio::timing::MicrosecondStopwatch;
MicrosecondStopwatch timer_align2d("align2d", 0);
MicrosecondStopwatch timer_align2d_neon("align2d neon", 0);
float align2d_total_time = 0.f;
float align2d_total_time_neon = 0.f;
#endif

namespace XP {

using Eigen::Vector2i;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

// The triangulated point at the current frame coordinate can be represented as:
// R_cur_ref * f_ref * d_ref + t_cur_ref =  f_cur * d_cur,
// where d_ref and d_cur are scales, respetively
// We can organize the equation into a linear system to solve d in the form of Ax = b:
// [R_cur_ref * f_ref  f_cur] * [d_ref; - d_cur] = - t_cur_ref
//   A = [R_cur_ref * f_ref f_cur],  3 x 2
//   x = [d_ref; -d_cur],  2 x 1
//   b = - t_cur_ref,  3 x 1
// x = (AtA)^-1 * At * b
//
// The returned depth is based on the reference frame
bool depthFromTriangulation(const Matrix3f& R_cur_ref,
                            const Vector3f& t_cur_ref,
                            const Vector3f& f_ref,
                            const Vector3f& f_cur,
                            float* depth) {
  Eigen::Matrix<float, 3, 2> A;
  A << R_cur_ref * f_ref, f_cur;
  const Matrix2f AtA = A.transpose() * A;
  if (AtA.determinant() < 1e-6) {
    // TODO(mingyu): figure the right threshold for float
    *depth = 1000;  // a very far point
    return false;
  }
  const Vector2f depth2 = - AtA.inverse() * A.transpose() * t_cur_ref;
  *depth = fabs(depth2[0]);
  return true;
}

void DirectMatcher::createPatchFromPatchWithBorder() {
  uint8_t* ref_patch_ptr = patch_;
  for (int y = 1; y < patch_size_ + 1; ++y, ref_patch_ptr += patch_size_) {
    uint8_t* ref_patch_border_ptr = patch_with_border_ + y * (patch_size_ + 2) + 1;
    memcpy(ref_patch_ptr, ref_patch_border_ptr, patch_size_);
    /*
    for (int x = 0; x < patch_size_; ++x) {
      ref_patch_ptr[x] = ref_patch_border_ptr[x];
    }
    */
  }
}

bool DirectMatcher::findMatchDirect(const vio::cameras::CameraBase& cam_ref,
                                    const vio::cameras::CameraBase& cam_cur,
                                    const Vector2f& px_ref,
                                    const Vector3f& f_ref,
                                    const Matrix3f& R_cur_ref,
                                    const Vector3f& t_cur_ref,
                                    const int level_ref,
                                    const float depth_ref,
                                    const std::vector<cv::Mat>& pyrs_ref,
                                    const std::vector<cv::Mat>& pyrs_cur,
                                    const bool edgelet_feature,
                                    Vector2f* px_cur) {
  CHECK_NEAR(f_ref[2], 1.f, 1e-6);
  CHECK_EQ(pyrs_ref.size(), pyrs_cur.size());

  // TODO(mingyu): check return boolean of getWarpMatrixAffine
  // warp affine
  warp::getWarpMatrixAffine(cam_ref,
                            cam_cur,
                            px_ref,
                            f_ref,
                            depth_ref,
                            R_cur_ref,
                            t_cur_ref,
                            level_ref,
                            &A_cur_ref_);
  const int max_level = pyrs_ref.size() - 1;
  const int search_level = warp::getBestSearchLevel(A_cur_ref_, max_level);

  // TODO(mingyu): check return boolean of warpAffine
  warp::warpAffine(A_cur_ref_,
                   pyrs_ref[level_ref],
                   px_ref,
                   level_ref,
                   search_level,
                   halfpatch_size_ + 1,
                   patch_with_border_);
  createPatchFromPatchWithBorder();

  // px_cur should be set
  Vector2f px_scaled = *px_cur / (1 << search_level);

  bool success = false;
  if (edgelet_feature) {
    // TODO(mingyu): currently not used until we further refine the feature type
    //               with gradient direction.
    //               Fast features do contain edgelet features.
    /*
    Vector2f dir_cur(A_cur_ref_ * ref_ftr_->grad);
    dir_cur.normalize();
    success = align::align1D(pyrs_cur[search_level],
                             dir_cur,
                             patch_with_border_,
                             patch_,
                             options_.max_iter,
                             &px_scaled,
                             &h_inv_);
    */
    LOG(ERROR) << "findMatchDirect for edgelet feature is NOT rimplemented yet";
    success = false;
  } else {
#ifndef _ALIGN2D_TIMING_AND_VERIFYING
#ifndef __ARM_NEON__
    success = align::align2D(pyrs_cur[search_level],
                             patch_with_border_,
                             patch_,
                             options_.max_iter,
                             &px_scaled);
#else
    success = align::align2D_NEON(pyrs_cur[search_level],
                                  patch_with_border_,
                                  patch_,
                                  options_.max_iter,
                                  &px_scaled);
#endif  // __ARM_NEON__
#else
    // timing and verifying code.
#ifdef __ARM_NEON__
    Vector2f px_scaled_neon = px_scaled;
#endif  // __ARM_NEON__
    timer_align2d.start();
    success = align::align2D(pyrs_cur[search_level],
                             patch_with_border_,
                             patch_,
                             options_.max_iter,
                             &px_scaled);
    timer_align2d.stop();
    align2d_total_time = timer_align2d.stop();

#ifdef __ARM_NEON__
    timer_align2d_neon.start();
    bool success_neon = align::align2D_NEON(pyrs_cur[search_level],
                                            patch_with_border_,
                                            patch_,
                                            options_.max_iter,
                                            &px_scaled_neon);
    timer_align2d_neon.stop();
    align2d_total_time_neon = timer_align2d_neon.elapse_ms();
    CHECK_EQ(success, success_neon);
    if (success_neon) {
      CHECK_NEAR(px_scaled[0], px_scaled_neon[0], 0.05);
      CHECK_NEAR(px_scaled[1], px_scaled_neon[1], 0.05);
      std::cout << "[NEON : NON-NEON --->" << "[" << align2d_total_time_neon
                << " : " << align2d_total_time << "]" << std::endl;
    }
#endif  // __ARM_NEON__
#endif  // _ALIGN2D_TIMING_AND_VERIFYING
  }
  *px_cur = px_scaled * (1 << search_level);
  return success;
}

bool DirectMatcher::findEpipolarMatchDirect(const vio::cameras::CameraBase& cam_ref,
                                            const vio::cameras::CameraBase& cam_cur,
                                            const Vector2f& px_ref,
                                            const Vector3f& f_ref,
                                            const Matrix3f& R_cur_ref,
                                            const Vector3f& t_cur_ref,
                                            const int level_ref,
                                            const float d_estimate,
                                            const float d_min,
                                            const float d_max,
                                            const std::vector<cv::Mat>& pyrs_ref,
                                            const std::vector<cv::Mat>& pyrs_cur,
                                            const cv::Mat_<uchar>& mask_cur,
                                            const bool edgelet_feature,
                                            Vector2f* px_cur,
                                            float* depth,
                                            int* level_cur,
                                            cv::Mat* dbg_cur) {
  CHECK_NEAR(f_ref[2], 1.f, 1e-6);
  CHECK_EQ(pyrs_ref.size(), pyrs_cur.size());

  // Compute start and end of epipolar line in old_kf for match search, on unit plane!
  // i.e., A & B are the first two elements of unit rays.
  // We will search from far to near
  Vector3f ray_A, ray_B;
  Vector2f px_A, px_B;
  ray_B = R_cur_ref * (f_ref * d_max) + t_cur_ref;  // far
  ray_B /= ray_B(2);
  if (vio::cameras::CameraBase::ProjectionStatus::Successful !=
      cam_cur.project(ray_B, &px_B)) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << "ray_A (far) cannot be reprojected in cam_cur";
#endif
    return false;
  }

  bool invalid_ray_A = true;
  for (float d = d_min; d < d_estimate; d *= 10) {
    ray_A = R_cur_ref * (f_ref * d) + t_cur_ref;  // near
    ray_A /= ray_A(2);
    if (vio::cameras::CameraBase::ProjectionStatus::Successful ==
        cam_cur.project(ray_A, &px_A)) {
      invalid_ray_A = false;
      break;
    }
  }
  if (invalid_ray_A) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << "ray_B (near) cannot be reprojected in cam_cur "
            << " d_min = " << d_min
            << " d_estimate = " << d_estimate;
#endif
    return false;
  }

  // Compute warp affine matrix
  if (!warp::getWarpMatrixAffine(cam_ref,
                                 cam_cur,
                                 px_ref,
                                 f_ref,
                                 d_estimate,
                                 R_cur_ref,
                                 t_cur_ref,
                                 level_ref,
                                 &A_cur_ref_)) {
    LOG(WARNING) << "warp::getWarpMatrixAffine fails";
    return false;
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << "A_cur_ref_ =\n" << A_cur_ref_;
#endif

  const int max_level = pyrs_ref.size() - 1;
  const int search_level = warp::getBestSearchLevel(A_cur_ref_, max_level);
  epi_dir_ = ray_A.head<2>() - ray_B.head<2>();  // far to near, B to A
  epi_length_ = (px_A - px_B).norm() / (1 << search_level);

  // feature pre-selection
  if (edgelet_feature) {
    /*
    const Vector2f grad_cur = (A_cur_ref_ * ref_ftr.grad).normalized();
    const float cosangle = fabs(grad_cur.dot(epi_dir_.normalized()));
    if (cosangle < options_.epi_search_edgelet_max_angle) {
      return false;
    }
    */
    LOG(ERROR) << "findEpipolarMatchDirect for edgelet feature is NOT implemented yet";
    return false;
  }

  if (!warp::warpAffine(A_cur_ref_,
                        pyrs_ref[level_ref],
                        px_ref,
                        level_ref,
                        search_level,
                        halfpatch_size_ + 1,
                        patch_with_border_)) {
    return false;
  }
  createPatchFromPatchWithBorder();
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << " search_level = " << search_level << " epi_length_ = " << epi_length_;
#endif
  if (epi_length_ < options_.max_epi_length_optim) {
    // The epipolar search line is short enough (< 2 pixels)
    // to perform direct alignment
    *px_cur = (px_A + px_B) * 0.5f;
    Vector2f px_scaled(*px_cur / (1 << search_level));
    bool success;
    if (options_.align_1d) {
      Vector2f direction = (px_A - px_B).normalized();
      success = align::align1D(pyrs_cur[search_level],
                               direction,
                               patch_with_border_,
                               patch_,
                               options_.max_iter,
                               &px_scaled,
                               &h_inv_);
    } else {
#ifndef _ALIGN2D_TIMING_AND_VERIFYING
#ifndef __ARM_NEON__
      success = align::align2D(pyrs_cur[search_level],
                               patch_with_border_,
                               patch_,
                               options_.max_iter,
                               &px_scaled);
#else
      success = align::align2D_NEON(pyrs_cur[search_level],
                                    patch_with_border_,
                                    patch_,
                                    options_.max_iter,
                                    &px_scaled);
#endif  // __ARM_NEON__
#else
      // verifying and timing code
#ifdef __ARM_NEON__
      Vector2f px_scaled_neon = px_scaled;
#endif  // __ARM_NEON__
      timer_align2d.start();
      success = align::align2D(pyrs_cur[search_level],
                               patch_with_border_,
                               patch_,
                               options_.max_iter,
                               &px_scaled);
      timer_align2d.stop();
      align2d_total_time = timer_align2d.elapse_ms();
#ifdef __ARM_NEON__
      timer_align2d_neon.start();
      bool success_neon = align::align2D_NEON(pyrs_cur[search_level],
                                              patch_with_border_,
                                              patch_,
                                              options_.max_iter,
                                              &px_scaled_neon);
      timer_align2d_neon.stop();
      align2d_total_time_neon = timer_align2d_neon.elapse_ms();
      CHECK_EQ(success, success_neon);
      if (success_neon) {
        CHECK_NEAR(px_scaled[0], px_scaled_neon[0], 0.05);
        CHECK_NEAR(px_scaled[1], px_scaled_neon[1], 0.05);
        std::cout << "[NEON : NON-NEON --->" << "[" << align2d_total_time_neon
                  << " : " << align2d_total_time << "]" << std::endl;
      }
#endif  // __ARM_NEON__
#endif  // _ALIGN2D_TIMING_AND_VERIFYING
    }

    if (success) {
      *px_cur = px_scaled * (1 << search_level);
      Vector3f f_cur;
      if (cam_cur.backProject(*px_cur, &f_cur)) {
        CHECK_NEAR(f_cur[2], 1.f, 1e-6);
        if (!depthFromTriangulation(R_cur_ref,
                                    t_cur_ref,
                                    f_ref,
                                    f_cur,
                                    depth)) {
          LOG(WARNING) << "depthFromTriangulation fails, set depth to d_max";
          *depth = d_max;
        }
      }
    }

    if (dbg_cur != nullptr) {
      if (success) {
        // green: subpix alignment is good
        cv::circle(*dbg_cur, cv::Point2f((*px_cur)(0), (*px_cur)(1)), 2, cv::Scalar(0, 255, 0));
      } else {
        // red: subpix alignment fails
        cv::circle(*dbg_cur, cv::Point2f((*px_cur)(0), (*px_cur)(1)), 2, cv::Scalar(0, 0, 255));
      }
    }
    return success;
  }

  // Determine the steps to Search along the epipolar line
  // [NOTE] The epipolar line can be curvy, so we slightly increase it
  //        to roughly have one step per pixel (heuristically).
  size_t n_steps = epi_length_ / 0.7;
  Vector3f step;
  step << epi_dir_ / n_steps, 0;
  if (n_steps > options_.max_epi_search_steps) {
    LOG(ERROR) << "Skip epipolar search: evaluations = " << n_steps
               << "epi length (px) = " << epi_length_;
    return false;
  }

  // Search along the epipolar line (on unit plane) with patch matching
  // for matching, precompute sum and sum2 of warped reference patch
  // [heuristic] The ssd from patch mean difference can be up to 50% of the resulting ssd.
  //  ssd = zmssd + N * (a_bar - b_bar)^2
  typedef patch_score::ZMSSD<halfpatch_size_> PatchScore;
  PatchScore patch_score(patch_);
  int zmssd_best = PatchScore::threshold();
  int ssd_corr = PatchScore::threshold() * 2;
  Vector3f ray_best;
  Vector3f ray = ray_B;
  Eigen::Vector2i last_checked_pxi(0, 0);
  const int search_img_rows = pyrs_cur[search_level].rows;
  const int search_img_cols = pyrs_cur[search_level].cols;
  ++n_steps;
  for (size_t i = 0; i < n_steps; ++i, ray += step) {
    Vector2f px;
    if (vio::cameras::CameraBase::ProjectionStatus::Successful != cam_cur.project(ray, &px)) {
      // We have already checked the valid projection of starting and ending rays.  However,
      // under very rare circumstance, cam_cur.project may still fail:
      // close to zero denominator for radial tangential 8 distortion:
      // 1 + k4 * r^2 + k5 * r^4 + k6 * r^6 < 1e-6
      continue;
    }
    Vector2i pxi(px[0] / (1 << search_level) + 0.5,
                 px[1] / (1 << search_level) + 0.5);  // round to closest int
    if (pxi == last_checked_pxi) {
      continue;
    }
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if (pxi[0] >= halfpatch_size_ && pxi[0] < search_img_cols - halfpatch_size_ &&
        pxi[1] >= halfpatch_size_ && pxi[1] < search_img_rows - halfpatch_size_ &&
        mask_cur(pxi(1), pxi(0)) > 0) {
      // TODO(mingyu): Interpolation instead?
      uint8_t* cur_patch_ptr = pyrs_cur[search_level].data
          + (pxi[1] - halfpatch_size_) * search_img_cols
          + (pxi[0] - halfpatch_size_);
      int ssd, zmssd;
      patch_score.computeScore(cur_patch_ptr, search_img_cols, &zmssd, &ssd);
      if (zmssd < zmssd_best) {
        // We store the best zmssd and its corresponding ssd score.  Usually,
        // zmssd and ssd have good correlation if the *matching* is reasonable.
        zmssd_best = zmssd;
        ssd_corr = ssd;
        ray_best = ray;
      }
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(2) << "search pxi=[" << pxi[0] << ", " << pxi[1] << "] zmssd = " << zmssd
              << " ssd = " << ssd;
#endif
      if (dbg_cur != nullptr) {
        dbg_cur->at<cv::Vec3b>(pxi[1], pxi[0]) = cv::Vec3b(255, 0, 0);
      }
    } else {
      // The patch contains out of bound pixels
      continue;
    }
  }

  VLOG(2) << "zmssd_best = " << zmssd_best << " ssd_corr = " << ssd_corr
          << " zmssd / ssd = " << static_cast<float>(zmssd_best) / ssd_corr;
  if (zmssd_best < PatchScore::threshold()) {
    cam_cur.project(ray_best, px_cur);
    if (options_.subpix_refinement) {
      Vector2f px_scaled(*px_cur / (1 << search_level));
      bool success;
      if (options_.align_1d) {
        Vector2f direction = (px_A - px_B).normalized();
        success = align::align1D(pyrs_cur[search_level],
                                 direction,
                                 patch_with_border_,
                                 patch_,
                                 options_.max_iter,
                                 &px_scaled,
                                 &h_inv_);
      } else {
#ifndef _ALIGN2D_TIMING_AND_VERIFYING
#ifndef __ARM_NEON__
        success = align::align2D(pyrs_cur[search_level],
                                 patch_with_border_,
                                 patch_,
                                 options_.max_iter,
                                 &px_scaled);
#else
        success = align::align2D_NEON(pyrs_cur[search_level],
                              patch_with_border_,
                              patch_,
                              options_.max_iter,
                              &px_scaled);
#endif  // __ARM_NEON__
#else
        #ifdef __ARM_NEON__
        Vector2f px_scaled_neon = px_scaled;
#endif  // __ARM_NEON__
        timer_align2d.start();
        success = align::align2D(pyrs_cur[search_level],
                             patch_with_border_,
                             patch_,
                             options_.max_iter,
                             &px_scaled);
        timer_align2d.stop();
        align2d_total_time = timer_align2d.elapse_ms();
#ifdef __ARM_NEON__
        timer_align2d_neon.start();
        bool success_neon = align::align2D_NEON(pyrs_cur[search_level],
                                            patch_with_border_,
                                            patch_,
                                            options_.max_iter,
                                            &px_scaled_neon);
        timer_align2d_neon.stop();
        align2d_total_time_neon = timer_align2d_neon.elapse_ms();
        CHECK_EQ(success, success_neon);
        if (success_neon) {
          CHECK_NEAR(px_scaled[0], px_scaled_neon[0], 0.05);
          CHECK_NEAR(px_scaled[1], px_scaled_neon[1], 0.05);
          std::cout << "[NEON : NON-NEON --->" << "[" << align2d_total_time_neon
                  << " : " << align2d_total_time << "]" << std::endl;
        }
#endif  // __ARM_NEON__
#endif  // _ALIGN2D_TIMING_AND_VERIFYING
      }

      if (success) {
        *px_cur = px_scaled * (1 << search_level);
        Vector3f f_cur;
        if (cam_cur.backProject(*px_cur, &f_cur)) {
          CHECK_NEAR(f_cur[2], 1.f, 1e-6);
          if (!depthFromTriangulation(R_cur_ref,
                                      t_cur_ref,
                                      f_ref,
                                      f_cur,
                                      depth)) {
            LOG(WARNING) << "depthFromTriangulation fails, set depth to d_max";
            *depth = d_max;
          }
        }
      }

      if (dbg_cur != nullptr) {
        if (success) {
          // green: subpix alignment is good
          cv::circle(*dbg_cur, cv::Point2f((*px_cur)(0), (*px_cur)(1)), 2, cv::Scalar(0, 255, 0));
        } else {
          // red: subpix alignment fails
          cv::circle(*dbg_cur, cv::Point2f((*px_cur)(0), (*px_cur)(1)), 2, cv::Scalar(0, 0, 255));
        }
      }
      return success;
    } else {
      // No subpix refinement
      CHECK_NEAR(ray_best[2], 1.f, 1e-6);
      if (!depthFromTriangulation(R_cur_ref,
                                  t_cur_ref,
                                  f_ref,
                                  ray_best,
                                  depth)) {
        LOG(WARNING) << "depthFromTriangulation fails, set depth to d_max";
        *depth = d_max;
      }
      return true;
    }
  }

  // No patch qualifiess a match
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(1) << "No matching patch found for this feature";
#endif
  return false;
}

// Prepare all the shared variabls for the whole tracking pipeline
ImgFeaturePropagator::ImgFeaturePropagator(
    const Eigen::Matrix3f& cur_camK,
    const Eigen::Matrix3f& ref_camK,
    const cv::Mat_<float>& cur_cv_dist_coeff,
    const cv::Mat_<float>& ref_cv_dist_coeff,
    const cv::Mat_<uchar>& cur_mask,
    int feat_det_pyramid_level,
    float min_feature_distance_over_baseline_ratio,
    float max_feature_distance_over_baseline_ratio) :
    mask_cur_(cur_mask),
    min_feature_distance_over_baseline_ratio_(min_feature_distance_over_baseline_ratio),
    max_feature_distance_over_baseline_ratio_(max_feature_distance_over_baseline_ratio),
    feat_det_pyramid_level_(feat_det_pyramid_level) {
  CHECK_GT(mask_cur_.rows, 0);
  CHECK_GT(mask_cur_.cols, 0);
  if (cur_cv_dist_coeff.rows == 8) {
    cam_cur_.reset(new vio::cameras::PinholeCamera<
        vio::cameras::RadialTangentialDistortion8>(
        mask_cur_.cols,
        mask_cur_.rows,
        cur_camK(0, 0),  // focalLength[0],
        cur_camK(1, 1),  // focalLength[1],
        cur_camK(0, 2),  // principalPoint[0],
        cur_camK(1, 2),  // principalPoint[1],
        vio::cameras::RadialTangentialDistortion8(
            cur_cv_dist_coeff(0),
            cur_cv_dist_coeff(1),
            cur_cv_dist_coeff(2),
            cur_cv_dist_coeff(3),
            cur_cv_dist_coeff(4),
            cur_cv_dist_coeff(5),
            cur_cv_dist_coeff(6),
            cur_cv_dist_coeff(7))));
  } else if (cur_cv_dist_coeff.rows == 4) {
    cam_cur_.reset(new vio::cameras::PinholeCamera<
        vio::cameras::RadialTangentialDistortion>(
        mask_cur_.cols,
        mask_cur_.rows,
        cur_camK(0, 0),  // focalLength[0],
        cur_camK(1, 1),  // focalLength[1],
        cur_camK(0, 2),  // principalPoint[0],
        cur_camK(1, 2),  // principalPoint[1],
        vio::cameras::RadialTangentialDistortion(
            cur_cv_dist_coeff(0),
            cur_cv_dist_coeff(1),
            cur_cv_dist_coeff(2),
            cur_cv_dist_coeff(3))));
  } else {
    LOG(FATAL) << "Dist model unsupported for cam_cur_";
  }
  if (ref_cv_dist_coeff.rows == 8) {
    cam_ref_.reset(new vio::cameras::PinholeCamera<
        vio::cameras::RadialTangentialDistortion8>(
        mask_cur_.cols,
        mask_cur_.rows,
        ref_camK(0, 0),  // focalLength[0],
        ref_camK(1, 1),  // focalLength[1],
        ref_camK(0, 2),  // principalPoint[0],
        ref_camK(1, 2),  // principalPoint[1],
        vio::cameras::RadialTangentialDistortion8(
            ref_cv_dist_coeff(0),
            ref_cv_dist_coeff(1),
            ref_cv_dist_coeff(2),
            ref_cv_dist_coeff(3),
            ref_cv_dist_coeff(4),
            ref_cv_dist_coeff(5),
            ref_cv_dist_coeff(6),
            ref_cv_dist_coeff(7))));
  } else if (ref_cv_dist_coeff.rows == 4) {
    cam_ref_.reset(new vio::cameras::PinholeCamera<
        vio::cameras::RadialTangentialDistortion>(
        mask_cur_.cols,
        mask_cur_.rows,
        ref_camK(0, 0),  // focalLength[0],
        ref_camK(1, 1),  // focalLength[1],
        ref_camK(0, 2),  // principalPoint[0],
        ref_camK(1, 2),  // principalPoint[1],
        vio::cameras::RadialTangentialDistortion(
            ref_cv_dist_coeff(0),
            ref_cv_dist_coeff(1),
            ref_cv_dist_coeff(2),
            ref_cv_dist_coeff(3))));
  } else {
    LOG(FATAL) << "Dist model unsupported for cam_ref_";
  }
}

bool ImgFeaturePropagator::PropagateFeatures(
    const cv::Mat& cur_img,
    const cv::Mat& ref_img,  // TODO(mingyu): store image pyramids
    const std::vector<cv::KeyPoint>& ref_keypoints,
    const Matrix4f& T_ref_cur,
    std::vector<cv::KeyPoint>* cur_keypoints,
    cv::Mat* cur_orb_features,
    const bool draw_debug) {
  cur_keypoints->clear();
  cur_keypoints->reserve(ref_keypoints.size());

  R_cur_ref_ = T_ref_cur.topLeftCorner<3, 3>().transpose();
  t_cur_ref_ = - R_cur_ref_ * T_ref_cur.topRightCorner<3, 1>();

  // TODO(mingyu): Make shared variables of DirectMatcher into member variables
  //               instead of passing as input arguments

  // Heuristically determine the d_max, d_min, and d_estimate based on the baseline,
  // d_min = baseline x 3
  // d_max = baseline x 3000
  // inv_d_estimate is the average of inv_d_min and inv_d_max
  const float baseline = t_cur_ref_.norm();
  const float d_min = baseline * min_feature_distance_over_baseline_ratio_;
  const float d_max = baseline * max_feature_distance_over_baseline_ratio_;
  const float d_estimate = 2.f / (1.f / d_min + 1.f / d_max);

  // TODO(mingyu): feed the backend results back for d_estimate if available
  // TODO(mingyu): feed the pyramids in directly
  std::vector<cv::Mat> pyrs_cur(feat_det_pyramid_level_);
  std::vector<cv::Mat> pyrs_ref(feat_det_pyramid_level_);
  pyrs_cur[0] = cur_img;
  pyrs_ref[0] = ref_img;
  for (int pyr_lv = 1; pyr_lv < feat_det_pyramid_level_; ++pyr_lv) {
    pyrs_cur[pyr_lv] = fast_pyra_down(pyrs_cur[pyr_lv - 1]);
    pyrs_ref[pyr_lv] = fast_pyra_down(pyrs_ref[pyr_lv - 1]);
  }

  for (const cv::KeyPoint& ref_kp : ref_keypoints) {
    int level_cur = 0;
    float depth = -1.f;
    Vector2f px_cur, px_ref(ref_kp.pt.x, ref_kp.pt.y);
    Vector3f f_ref;
    if (!cam_ref_->backProject(px_ref, &f_ref)) {
      continue;
    }

    // Visualization (re-draw for every keypoint)
    if (draw_debug) {
      dbg_img_.create(mask_cur_.rows * 2, mask_cur_.cols, CV_8UC3);
      dbg_ref_ = dbg_img_(cv::Rect(0, 0, mask_cur_.cols, mask_cur_.rows));
      dbg_cur_ = dbg_img_(cv::Rect(0, mask_cur_.rows, mask_cur_.cols, mask_cur_.rows));
      dbg_cur_ptr_ = &dbg_cur_;
      cv::cvtColor(ref_img, dbg_ref_, CV_GRAY2RGB);
      cv::cvtColor(cur_img, dbg_cur_, CV_GRAY2RGB);
      cv::circle(dbg_ref_, ref_kp.pt, 2, cv::Scalar(0, 255, 0));
      cv::putText(dbg_ref_, boost::lexical_cast<std::string>(ref_kp.class_id),
                  ref_kp.pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      for (int i = 0; i < mask_cur_.rows; ++i) {
        for (int j = 0; j < mask_cur_.cols; ++j) {
          if (mask_cur_(i, j) == 0x00) {
            dbg_cur_.at<cv::Vec3b>(i, j)[0] = 0;
            dbg_cur_.at<cv::Vec3b>(i, j)[1] = 0;
          }
        }
      }
    }
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(1) << "findEpipolarMatchDirect for of_id = " << ref_kp.class_id;
#endif
    if (!direct_matcher_.findEpipolarMatchDirect(*cam_ref_,
                                                 *cam_cur_,
                                                 px_ref,
                                                 f_ref,
                                                 R_cur_ref_,
                                                 t_cur_ref_,
                                                 ref_kp.octave,  // double check here
                                                 d_estimate,
                                                 d_min,
                                                 d_max,
                                                 pyrs_ref,
                                                 pyrs_cur,
                                                 mask_cur_,
                                                 false, /*edgelet_feature, not supported yet*/
                                                 &px_cur,
                                                 &depth,
                                                 &level_cur,
                                                 dbg_cur_ptr_)) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      if (draw_debug) {
        cv::imwrite("/tmp/per_feat_dbg/det_" + boost::lexical_cast<std::string>(ref_kp.class_id)
                    + ".png", dbg_img_);
      }
      VLOG(1) << "findEpipolarMatchDirect fails for of_id: " << ref_kp.class_id;
#endif
      continue;
    }
#ifndef __FEATURE_UTILS_NO_DEBUG__
    if (draw_debug) {
      cv::imwrite("/tmp/per_feat_dbg/det_" + boost::lexical_cast<std::string>(ref_kp.class_id)
                  + ".png", dbg_img_);
      VLOG(1) << "findEpipolarMatchDirect succeeds for of_id: " << ref_kp.class_id;
    }
#endif

    // check boundary condition (set to 20 pixels for computing ORB features)
    // [NOTE] Returned px_cur should be within mask_cur_ already
    const int orb_desc_margin = 20;
    if (px_cur(0) < orb_desc_margin || px_cur(0) > mask_cur_.cols - orb_desc_margin ||
        px_cur(1) < orb_desc_margin || px_cur(1) > mask_cur_.rows - orb_desc_margin) {
      continue;
    }

    cv::KeyPoint cur_kp = ref_kp;
    cur_kp.octave = level_cur;
    cur_kp.pt.x = px_cur(0);
    cur_kp.pt.y = px_cur(1);
    cur_keypoints->push_back(cur_kp);
  }

  if (cur_orb_features != nullptr && cur_keypoints->size() > 0) {
    cur_orb_features->create(cur_keypoints->size(), 32, CV_8U);
#ifdef __ARM_NEON__
    ORBextractor::computeDescriptorsN512(cur_img, *cur_keypoints, cur_orb_features);
#else
    ORBextractor::computeDescriptors(cur_img, *cur_keypoints, cur_orb_features);
#endif
  }
  return true;
}

}  // namespace XP
