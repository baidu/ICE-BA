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
#include "timer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/lexical_cast.hpp>
#include <brisk/scale-space-feature-detector.h>
#include <brisk/internal/uniformity-enforcement.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
using std::vector;

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __FEATURE_UTILS_NO_DEBUG__
#endif
// #define VERIFY_NEON

namespace XP {
void build_pyramids(const cv::Mat& img, int max_level,
                    std::vector<cv::Mat>* _pyramids,
                    uchar* const pyra_buf_ptr) {
  CHECK_NOTNULL(_pyramids);
  std::vector<cv::Mat>& pyramids = *_pyramids;
  pyramids.resize(max_level + 1);
  const int size = img.rows * img.cols;
  // no fixed buffer specified, will allocate memory dynamically
  if (pyra_buf_ptr == nullptr) {
    pyramids[0] = img.clone();
    for (int i = 1; i <= max_level; ++i) {
      pyramids[i] = fast_pyra_down(pyramids[i - 1]);
    }
  } else {
    // fixed buffer provided
    // the pyramids kept in the fixed buffer in this way:
    // level n |  level n-1 |  level 1  |  level 0
    // so we can use them in a very cache-friendly way
    // and no need to call malloc
    for (int lvl = 0; lvl <= max_level; ++lvl) {
      int offset = 0;
      // compute pyramid start address
      for (int i = lvl + 1; i <= max_level; ++i) {
        offset += size >> (2 * i);
      }
      if (lvl != 0) {
        pyramids[lvl] = fast_pyra_down(pyramids[lvl - 1], pyra_buf_ptr + offset);
      } else {
        cv::Mat tmp(img.rows, img.cols, img.type(), pyra_buf_ptr + offset);
        img.copyTo(tmp);
        pyramids[lvl] = tmp;
      }
    }
  }
}

// make sure 0x00 keeps 0x00 in the next level
inline cv::Mat fast_mask_pyra_down(const cv::Mat& mask) {
  constexpr int compress_ratio = 2;
  cv::Mat mask_small(mask.rows / compress_ratio,
                     mask.cols / compress_ratio,
                     CV_8U);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(mask.type(), CV_8U);
#endif
  // use our own pyra down for faster performance
  const int width_step_in = mask.step1();
  const int width_step_small = mask_small.step1();
  for (int y = 0; y < mask_small.rows; y++) {
    for (int x = 0; x < mask_small.cols; x++) {
      // do not use .at<char> which is slow
      const int shift0 = (y * compress_ratio) * width_step_in + x * compress_ratio;
      const int shift1 = shift0 + width_step_in;
      if (*(mask.data + shift0) == 0x00 ||
          *(mask.data + shift1) == 0x00 ||
          *(mask.data + shift0 + 1) == 0x00 ||
          *(mask.data + shift1 + 1) == 0x00) {
        *(mask_small.data + y * width_step_small + x) = 0x00;
      } else {
        *(mask_small.data + y * width_step_small + x) = 0xff;
      }
    }
  }
  return mask_small;
}

// 1. refine_kp_in_larger_img takes the keypoints in the small image (higher pyramid level),
//    and search in a local region in the large image (lower pyramid level).  The refined
//    keypoint location is computed as the weighted average of local points by harris response.
// 2. The response of the refined keypoint is passed from the response from the original detection.
//    May NOT be harris response though.
inline bool refine_kp_in_larger_img(const cv::Mat& img_in_smooth,
                                    const std::vector<cv::KeyPoint>& kp_in_small_img,
                                    std::vector<cv::KeyPoint>* refined_kp_in_large_img_ptr) {
  CHECK_NOTNULL(refined_kp_in_large_img_ptr);
  std::vector<cv::KeyPoint>& refined_kp_in_large_img = *refined_kp_in_large_img_ptr;
  std::vector<cv::KeyPoint> local_kps;
  constexpr int harris_block_size = 7;  // 5 does not give correct results
  refined_kp_in_large_img.clear();
  refined_kp_in_large_img.reserve(kp_in_small_img.size());
  constexpr int compress_ratio = 2;

  for (int kp_small_idx = 0 ; kp_small_idx < kp_in_small_img.size(); ++kp_small_idx) {
    const cv::KeyPoint& key_pnt_small = kp_in_small_img[kp_small_idx];
    local_kps.clear();
    local_kps.reserve(compress_ratio * 2 * compress_ratio * 2);
    for (int y = key_pnt_small.pt.y * compress_ratio - compress_ratio + 1;
         y < key_pnt_small.pt.y * compress_ratio + compress_ratio; y++) {
      for (int x = key_pnt_small.pt.x * compress_ratio - compress_ratio + 1;
           x < key_pnt_small.pt.x * compress_ratio + compress_ratio; x++) {
        if (y > harris_block_size / 2 && x > harris_block_size / 2
            && y < img_in_smooth.rows - harris_block_size / 2
            && x < img_in_smooth.cols - harris_block_size / 2) {
          cv::KeyPoint local_kp = key_pnt_small;
          local_kp.pt.x = x;
          local_kp.pt.y = y;
          local_kps.push_back(local_kp);
        }
      }
    }

    // If the local keypoints (in large image) are NOT empty, we compute the weighted average
    // of the point location and response.
    if (local_kps.size() > 0) {
      ORBextractor::HarrisResponses(img_in_smooth, harris_block_size, 0.04f, &local_kps);
      float score_total = 0;
      float x_weighted_sum = 0;
      float y_weighted_sum = 0;
      float highest_score = - std::numeric_limits<float>::max();
      int best_local_kp_id = -1;
      for (size_t i = 0; i < local_kps.size(); i++) {
        if (local_kps[i].response > 0) {
          // ignore points whose harris response is less than 0
          score_total += local_kps[i].response;
          x_weighted_sum += local_kps[i].pt.x * local_kps[i].response;
          y_weighted_sum += local_kps[i].pt.y * local_kps[i].response;
          if (local_kps[i].response > highest_score) {
            highest_score = local_kps[i].response;
            best_local_kp_id = i;
          }
        }
#ifndef __FEATURE_UTILS_NO_DEBUG__
        VLOG(3) << "local_kp.response " << local_kps[i].response
                << " local_kp.pt " << local_kps[i].pt
                << " score_total " << score_total;
#endif
      }
      if (best_local_kp_id < 0) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
        VLOG(3) << "refine best_local_kp for kp_small[" << kp_small_idx
                << "] fails: no positive harris response";
#endif
        continue;
      }
      cv::KeyPoint best_local_kp = local_kps[best_local_kp_id];  // copy scale score etc.
      // use weighted average
      if (highest_score > 1e-9) {
        best_local_kp.pt.x = round(x_weighted_sum / score_total);
        best_local_kp.pt.y = round(y_weighted_sum / score_total);
#ifndef __FEATURE_UTILS_NO_DEBUG__
        VLOG(3) << "refine best_local_kp for kp_small[" << kp_small_idx
                << "]: pt = " << best_local_kp.pt;
#endif
        refined_kp_in_large_img.push_back(best_local_kp);
      } else {
#ifndef __FEATURE_UTILS_NO_DEBUG__
        VLOG(3) << "refine best_local_kp for kp_small[" << kp_small_idx
                << "] fails: weak harris response = " << highest_score;
#endif
      }
    } else {
      auto kp = key_pnt_small;
      kp.pt.x *= compress_ratio;
      kp.pt.y *= compress_ratio;
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(3) << "refine best_local_kp for kp_small[" << kp_small_idx
                << "] direct pass through (local_kps is empty)";
#endif
      refined_kp_in_large_img.push_back(kp);
    }
  }
  return true;
}

// The design concept is to trace rays along the diagonal of FOV, say
// bottom left to top right passing through the pinhole center.
// Then take the average of the farthest ray that is still projected inside the image
// as the cropping FOV.
// However, we limit the FOV upper bound to 160 degrees (for fisheye case).
bool generate_cam_mask(const cv::Matx33f& K,
                       const cv::Mat_<float>& dist_coeffs,
                       const cv::Size& mask_size,
                       cv::Mat_<uchar>* cam_mask,
                       float* fov_deg) {
  CHECK_GT(mask_size.width, 0);
  CHECK_GT(mask_size.height, 0);
  CHECK_EQ(cam_mask->rows, 0);
  cam_mask->create(mask_size);
  cam_mask->setTo(0xff);

  const float theta = std::atan2(mask_size.height, mask_size.width);
  std::vector<int> viewable_degs = {0, 0};
  for (int i = 0; i < 2; ++i) {
    int direction = i * 2 - 1;
    for (int deg = 30; deg < 90; deg += 5) {
      float d = std::tan(deg * M_PI / 180.f);
      float a = direction * d * cos(theta);
      float b = direction * d * sin(theta);
      std::vector<cv::Vec3f> ray(1);
      std::vector<cv::Point2f> dist_pt;
      ray[0] = cv::Vec3f(a, -b, 1);
      cv::projectPoints(ray, cv::Vec3f(), cv::Vec3f(), K, dist_coeffs, dist_pt);
      if (dist_pt[0].x > 0 &&
          dist_pt[0].y > 0 &&
          dist_pt[0].x < mask_size.width &&
          dist_pt[0].y < mask_size.height) {
        viewable_degs[i] = deg;
      } else {
        break;
      }
    }
  }

  if (viewable_degs[0] == 0 || viewable_degs[1] == 0) {
    // FOV cannot be estimated.  Return an empty mask.
    LOG(ERROR) << "Cannot find proper FOV to generate camera mask";
    return false;
  }

  *fov_deg = viewable_degs[0] + viewable_degs[1];
  if (*fov_deg > 160) {
    *fov_deg = 160;  // cap FOV to 160 degree
  }
  float half_fov = *fov_deg / 2;
  float d = std::tan(half_fov * M_PI / 180.f);
  std::vector<cv::Vec3f> ray(1);
  std::vector<cv::Point2f> dist_pt;
  ray[0] = cv::Vec3f(d, 0, 1);
  cv::projectPoints(ray, cv::Vec3f(), cv::Vec3f(), K, dist_coeffs, dist_pt);
  cam_mask->create(mask_size);
  cam_mask->setTo(0xff);
  const int r = static_cast<int>(dist_pt[0].x - K(0, 2));
  const int cx = static_cast<int>(K(0, 2));
  const int cy = static_cast<int>(K(1, 2));
  for (int i = 0; i < mask_size.height; ++i) {
    for (int j = 0; j < mask_size.width; ++j) {
      int rx = j - cx;
      int ry = i - cy;
      if (rx * rx + ry * ry > r * r) {
        (*cam_mask)(i, j) = 0x00;
      }
    }
  }
  return true;
}

namespace internal {
struct {
  bool operator()(const cv::KeyPoint& a, const cv::KeyPoint& b) {
    return a.response > b.response;
  }
} kp_compare;
}  // namespace internal

bool detect_orb_features(const cv::Mat& img_in_smooth,
                         const cv::Mat_<uchar>& mask,
                         int request_feat_num,
                         int pyra_level,  // Total pyramid levels, including the base image
                         int fast_thresh,
                         bool use_fast,  // or TomasShi
                         int enforce_uniformity_radius,  // less than 0 means no enforcement
                         std::vector<cv::KeyPoint>* key_pnts_ptr,
                         cv::Mat* orb_feat_ptr,
                         FeatureTrackDetector* feat_track_detector,
                         float refine_harris_threshold) {
  CHECK_GT(pyra_level, 0);
  // Only detect feature at det_pyra_level, and then look for refined corner position at level 0.
  // For now, all the features are *refined* to pyramid0 as octave is 0 for all detected features
  // ORBextractor runs detection at det_pyra_level with levels orb_pyra_levels = 1, which means
  // no multi-pyramids within ORBextractor.
  // TODO(mingyu): Pre-compute the pyramids if needed to avoid duplicate computation when calling
  //               multiple times in vio_mapper
  const int det_pyra_level = pyra_level - 1;
  const int compress_ratio = 1 << det_pyra_level;
  vector<cv::Mat> img_pyramids(pyra_level);
  vector<cv::Mat_<uchar>> mask_pyramids(pyra_level);
  img_pyramids[0] = img_in_smooth;
  mask_pyramids[0] = mask;
  for (int i = 1; i < pyra_level; i++) {
    img_pyramids[i] = fast_pyra_down(img_pyramids[i - 1]);
    mask_pyramids[i] = fast_mask_pyra_down(mask_pyramids[i - 1]);
  }
  // TODO(mingyu): Reduce more_points_ratio as it seems too conservative.
  float more_points_ratio = 1.5;  // because refinement may reduce points number
  enforce_uniformity_radius = enforce_uniformity_radius >> det_pyra_level;
  if (enforce_uniformity_radius > 5) {
    more_points_ratio = 3;  // hueristic
    fast_thresh /= 2;  // usually its set to be 10 - 20
  }

  std::vector<std::vector<cv::KeyPoint>> kp_in_pyramids(pyra_level);
  constexpr int orb_pyra_levels = 1;  // The pyramid levels used in ORBextractor
  // [NOTE] We need the score (keypoint.response) computed as harris score instead
  //        of FAST.  It's slower but more discriminative to sort / refine keypoints.
  ORBextractor orb(request_feat_num * more_points_ratio, 2, orb_pyra_levels,
                   ORBextractor::HARRIS_SCORE,
                   fast_thresh,
                   use_fast);
  orb.detect(img_pyramids[det_pyra_level],
             mask_pyramids[det_pyra_level],
             &kp_in_pyramids[det_pyra_level]);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(1) << "Before uniformaty check ORBextractor gets "
          << kp_in_pyramids[det_pyra_level].size() << " pnts from "
          << " level = " << det_pyra_level
          << " mask_pyramids[" << det_pyra_level << "].size "
          << mask_pyramids[det_pyra_level].size();
  if (mask_pyramids[det_pyra_level].rows > 0) {
    for (auto& kp : kp_in_pyramids[det_pyra_level]) {
      if (mask_pyramids[det_pyra_level](kp.pt.y, kp.pt.x) == 0x00) {
        LOG(FATAL) << "mask_pyramids[" << det_pyra_level << "]"
                   << kp.pt << " = 0x00";
      }
    }
  }
#endif
  if (enforce_uniformity_radius > 5 &&
      kp_in_pyramids[det_pyra_level].size() > 1) {
    // Copied from scale-space-layer-inl.h
    // Basically, this weight_LUT can suppress at most a radius of 15 pixels.  In pracice,
    // it is possible to see features that are 2 to 4 pixels apart without being suppressed.
    cv::Mat weight_LUT = cv::Mat::zeros(2 * 16 - 1, 2 * 16 - 1, CV_32F);
    for (int x = 0; x < 2 * 16 - 1; ++x) {
      for (int y = 0; y < 2 * 16 - 1; ++y) {
        weight_LUT.at<float>(y, x) =
            std::max(1 - static_cast<float>((15 - x) * (15 - x) + (15 - y) * (15 - y))
                / static_cast<float>(15 * 15), 0.f);
      }
    }
    vector<brisk::ScoreCalculator<float>::PointWithScore> points;
    points.resize(kp_in_pyramids[det_pyra_level].size());
    for (size_t i = 0; i < kp_in_pyramids[det_pyra_level].size(); i++) {
      points[i].x = kp_in_pyramids[det_pyra_level][i].pt.x;
      points[i].y = kp_in_pyramids[det_pyra_level][i].pt.y;
      points[i].score = kp_in_pyramids[det_pyra_level][i].response;
    }
    // TODO(mingyu): Implement a simple minded binary mask instead of using weighted mask
    // TODO(mingyu): Rewrite XpEnforceKeyPointUniformity to take vector of cv::KeyPoint directly
    XpEnforceKeyPointUniformity(weight_LUT, enforce_uniformity_radius,
                                img_pyramids[det_pyra_level].rows,
                                img_pyramids[det_pyra_level].cols,
                                request_feat_num, points);
    kp_in_pyramids[det_pyra_level].clear();
    kp_in_pyramids[det_pyra_level].reserve(points.size());
    for (const auto& pnt_and_score : points) {
      cv::KeyPoint kp;
      kp.pt.x = pnt_and_score.x;
      kp.pt.y = pnt_and_score.y;
      kp.response = pnt_and_score.score;  // brisk score here
      // hueristics: 12, 18, 24, 36, etc.
      // We choose 12 here (for octave 0) to match the brisk detector.
      kp.size = 12;
      kp_in_pyramids[det_pyra_level].push_back(kp);
    }
    // Compute orientation only when orb descriptors are requested.
    // Copied from ORBextractor.cc
    if (orb_feat_ptr != nullptr) {
      vector<int> umax;
      constexpr int HALF_PATCH_SIZE = 15;
      umax.resize(HALF_PATCH_SIZE + 1);
      int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * std::sqrt(2.f) / 2 + 1);
      int vmin = cvCeil(HALF_PATCH_SIZE * std::sqrt(2.f) / 2);
      const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
      for (v = 0; v <= vmax; ++v) {
        umax[v] = cvRound(std::sqrt(hp2 - v * v));
      }
      // Make sure we are symmetric
      for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
        while (umax[v0] == umax[v0 + 1]) ++v0;
        umax[v] = v0;
        ++v0;
      }
      ORBextractor::computeOrientation(img_pyramids[det_pyra_level],
                                       umax, &kp_in_pyramids[det_pyra_level]);
    }
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(1) << "After uniformity ORBextractor gets "
          << kp_in_pyramids[det_pyra_level].size() << " pnts from "
          << " level = " << det_pyra_level;
  if (VLOG_IS_ON(4)) {
    cv::Mat small_debug = img_pyramids[det_pyra_level].clone();
    for (const auto& kp : kp_in_pyramids[det_pyra_level]) {
      small_debug.at<uchar>(kp.pt.y, kp.pt.x) = 0xff;
      VLOG(2) << "kp " << kp.pt << " at level " << det_pyra_level << " score " << kp.response;
    }
    cv::imwrite("/tmp/img_level_" + boost::lexical_cast<std::string>(det_pyra_level) + ".png",
                small_debug);
  }
#endif
  // Refine the corner response.  Push keypoints from higher pyramids to pyramid 0.
  // Look for the corner with the highest harris response.
  // If a kp in small img has weak response in large img, it will be dumped.
  // [NOTE] This operation IGNORES and OVERWRITES existing keypoints in lower pyramids if any!
  for (int it_pyra = det_pyra_level; it_pyra > 0; it_pyra--) {
    refine_kp_in_larger_img(img_pyramids[it_pyra - 1],
                            kp_in_pyramids[it_pyra],
                            &kp_in_pyramids[it_pyra - 1]);
    CHECK_GE(kp_in_pyramids[it_pyra].size(), kp_in_pyramids[it_pyra - 1].size());
#ifndef __FEATURE_UTILS_NO_DEBUG__
    if (VLOG_IS_ON(4)) {
      cv::Mat small_debug = img_pyramids[it_pyra - 1].clone();
      for (const auto& kp : kp_in_pyramids[it_pyra - 1]) {
        small_debug.at<uchar>(kp.pt.y, kp.pt.x) = 0xff;
        VLOG(2) << "kp " << kp.pt << " at level " << it_pyra - 1;
      }
      cv::imwrite("/tmp/img_level_" + boost::lexical_cast<std::string>(it_pyra - 1) + ".png",
                  small_debug);
    }
#endif
  }

  // The original pattern bit after rotation can exceed half-window size(16), 17, or 18.
  // We set feat_half_size to 20 here to keep the KeyPoint away from
  // the possible dangerous place (see the code below) before extracting ORB descriptors.
  // The feat_half_size is large enough to satisfy the harris margin even after moving up
  // one pyramid level to within_bound_kps_small.
  const int feat_half_size = 20;  // 20 pixels at pyramid 0
  std::vector<cv::KeyPoint> within_bound_kps;
  std::vector<cv::KeyPoint> within_bound_kps_small;
  within_bound_kps.reserve(kp_in_pyramids[0].size());
  within_bound_kps.reserve(kp_in_pyramids[0].size());
  for (const auto& kp : kp_in_pyramids[0]) {
    if (kp.pt.x > feat_half_size &&
        kp.pt.y > feat_half_size &&
        kp.pt.x < img_in_smooth.cols - feat_half_size &&
        kp.pt.y < img_in_smooth.rows - feat_half_size) {
      within_bound_kps.push_back(kp);
      within_bound_kps_small.push_back(cv::KeyPoint(kp.pt / 2, kp.size));  // default response is 0
    }
  }
  CHECK_EQ(within_bound_kps.size(), within_bound_kps_small.size());

  // Check the harris response at pyramid1 (match the behavior in propagate_with_optical_flow,
  // and remove the corners with weak responses.
  // Note that the default response value in within_bound_kps_small is 0.
  // TODO(mingyu): Refactor the code to re-use the harris response computed earlier.
  if (refine_harris_threshold > 0) {
    cv::Mat img_in_smooth_small =
        (pyra_level == 1) ? fast_pyra_down(img_pyramids[0]) : img_pyramids[1];
    ORBextractor::HarrisResponses(img_in_smooth_small, 7, 0.04f, &within_bound_kps_small);
  } else {
    // Keep the default 0 response in within_bound_kps_small.
  }

  // Fill in key_pnts_ptr
  CHECK_NOTNULL(key_pnts_ptr);
  key_pnts_ptr->clear();
  if (request_feat_num > within_bound_kps.size()) {
    key_pnts_ptr->reserve(within_bound_kps.size());
  } else {
    key_pnts_ptr->reserve(request_feat_num);
    // TODO(mingyu): This sort is dummy as XpEnforceUniformity has already sorted.
    std::sort(within_bound_kps.begin(), within_bound_kps.end(), internal::kp_compare);
  }

#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << "request_feat_num = " << request_feat_num
          << " within_bound_kps.size() = " << within_bound_kps.size();
#endif
  for (int i = 0, count = 0; count < request_feat_num && i < within_bound_kps.size(); ++i) {
    if (within_bound_kps_small[i].response < refine_harris_threshold) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(2) << "within_bound[" << i << "].response = " << within_bound_kps_small[i].response
              << " < thres = " << refine_harris_threshold;
#endif
      continue;
    }

    cv::KeyPoint& kp = within_bound_kps[i];
    if (feat_track_detector) {
      kp.class_id = feat_track_detector->add_new_feature_track(kp.pt);
    } else {
      kp.class_id = -1;  // Feature track is NOT used.
    }
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << "kps[" << count << "] = within_bound[" << i << "] id = " << kp.class_id
            << " response: " << within_bound_kps_small[i].response;
#endif

    // TODO(mingyu): use 0 as we have refined to pyra0 or use the real octave at detection
    kp.octave = 0;
    key_pnts_ptr->push_back(kp);
    ++count;
  }

  // get orb
  if (orb_feat_ptr != nullptr) {
    // detect orb at pyra 0
    // since orb radius is 15, which is alraedy pretty big
#ifdef __ARM_NEON__
    ORBextractor::computeDescriptorsN512(img_in_smooth, *key_pnts_ptr, orb_feat_ptr);
#else
    ORBextractor::computeDescriptors(img_in_smooth, *key_pnts_ptr, orb_feat_ptr);
#endif
  }
  return true;
}

bool detect_harris_features(
    const cv::Mat& img_in_smooth,
    const cv::Mat_<uchar> mask,
    int request_feat_num,
    int pyra_level,
    int fast_thresh,
    std::vector<cv::KeyPoint>* key_pnts_ptr,
    cv::Mat* orb_feat_ptr
) {
  CHECK_NOTNULL(key_pnts_ptr);
  CHECK_NOTNULL(orb_feat_ptr);
  ORBextractor orb(request_feat_num, 2, pyra_level, 1, fast_thresh);
  orb.detect(img_in_smooth, mask, key_pnts_ptr, orb_feat_ptr);
  return true;
}

SlaveImgFeatureDetector::SlaveImgFeatureDetector(int block_size,
                                                 DetectSlaveFeatureType method,
                                                 float min_feature_distance_over_baseline_ratio,
                                                 float max_feature_distance_over_baseline_ratio) :
    block_size_(block_size),
    half_block_size_(block_size / 2),
    method_(method),
    min_feature_distance_over_baseline_ratio_(min_feature_distance_over_baseline_ratio),
    max_feature_distance_over_baseline_ratio_(max_feature_distance_over_baseline_ratio) {
  if (block_size_ % 4 != 0) {
    LOG(FATAL) << "block_size = " << block_size;
  }
  gaussion_weights_ = new float[block_size * block_size];
  gaussion_weight_sum_ = 0;
  for (int v = - half_block_size_; v < half_block_size_; ++v) {
    for (int u = - half_block_size_; u < half_block_size_; ++u) {
      const float w = std::exp(- static_cast<float>(u * u + v * v) / 9.f);
      gaussion_weights_[(v + half_block_size_) * block_size + u + half_block_size_] = w;
      gaussion_weight_sum_ += w;
    }
  }
}
SlaveImgFeatureDetector::~SlaveImgFeatureDetector() {
  delete [] gaussion_weights_;
}
bool SlaveImgFeatureDetector::detect_features_on_slave_img_helper(
    const cv::Mat& master_image,
    const cv::Mat& slave_image,
    const DuoCalibParam& duo_calib_param,
    int max_pixel_val_diff,
    int master_x,
    int master_y,
    const cv::Mat_<float>& s_R_m,
    const cv::Mat_<float>& s_t_m,
    const cv::Mat_<float>& pnt_master,
    const cv::Mat_<float>& search_range,
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
    cv::Mat* slave_image_debug_ptr,
#endif
    int* min_patch_diff2_ptr,
    int* second_min_patch_diff2_ptr,
    int* best_slave_x_ptr,
    int* best_slave_y_ptr,
    int* best_search_dist_id_ptr,
    int* second_best_search_dist_id_ptr) {
  const int scale = 2;
  // since we compute diff in pyr level 1, we use block_size_ rather than half_block_size_ as radius
  // orb desc needs 16 pixels margin
  const int img_margin = (half_block_size_ * scale) > 16 ? (half_block_size_ * scale) : 16;
  // since we don't know where this point is, try to move it to the furthest dis
  cv::Mat pnts_slave = search_range * pnt_master.t() * s_R_m.t()
      + search_range.ones(search_range.size()) * s_t_m.t();
  cv::Mat pixels_slave;
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(pnts_slave.type(), CV_32F);
  CHECK_GT(pnts_slave.checkVector(3), 0);
  CHECK_EQ(pnts_slave.depth(), CV_32F);
  CHECK_GT(pnts_slave.rows, search_range.cols);
  int avg_diff_count = 0;
  int early_break_count = 0;
  int candidate_count = 0;
  int find_count = 0;
  int skip_count = 0;
#endif
  cv::projectPoints(pnts_slave, cv::Matx31d(), cv::Matx31d(),
                    duo_calib_param.Camera.cv_camK_lr[1],
                    duo_calib_param.Camera.cv_dist_coeff_lr[1],
                    pixels_slave);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(pixels_slave.type(), CV_32FC2);
  CHECK_EQ(pixels_slave.cols, 1);
  CHECK_EQ(pixels_slave.rows, search_range.rows);
#endif
  const int max_patch_val_diff = gaussion_weight_sum_ * max_pixel_val_diff;
  // so second_min_patch_val_diff2 will be assigned to this val for the first time
  int min_patch_val_diff = max_patch_val_diff * 10;
  int second_min_patch_val_diff = min_patch_val_diff * 10;
  const int slave_col = slave_image.step1();
  const int master_col = master_image.step1();
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(slave_col, master_col);
#endif
  // cache
  float master_sum = 0;
  for (int v = - half_block_size_; v < half_block_size_; ++v) {
    for (int u = - half_block_size_; u < half_block_size_; ++u) {
      const float w = gaussion_weights_[(v + half_block_size_)* block_size_ + u + half_block_size_];
      master_sum +=
          w * static_cast<float>(master_image.data[(master_y + v * scale)
              * master_col + master_x + u * scale]);
    }
  }
  float master_avg = master_sum / gaussion_weight_sum_;
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << "master_avg " << master_avg << " max_pixel_val_diff " << max_pixel_val_diff;
#endif
  int best_slave_x = 0;
  int best_slave_y = 0;
  int best_search_dist_id = -1;
  int second_best_search_dist_id = -1;
  // during fine search stage, there may be sample points having the same xy. Quickly jump over
  int pre_slave_x = -1, pre_slave_y = -1;
  for (int it_slave = 0; it_slave < pixels_slave.rows; ++it_slave) {
    int x = std::roundf(pixels_slave.at<cv::Vec2f>(it_slave)[0] + 0.5);
    int y = std::roundf(pixels_slave.at<cv::Vec2f>(it_slave)[1] + 0.5);
    if (x == pre_slave_x && y == pre_slave_y) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      ++skip_count;
#endif
      continue;
    }
    pre_slave_x = x;
    pre_slave_y = y;
    if (x >= img_margin && y >= img_margin
        && x < slave_image.cols - img_margin
        && y < slave_image.rows - img_margin) {
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
      slave_image_debug_ptr->at<uchar>(y, x) = 0xff;
      ++candidate_count;
#endif
      // compute patch avg
      float slave_sum = 0;
      for (int v = - half_block_size_; v < half_block_size_; ++v) {
        for (int u = - half_block_size_; u < half_block_size_; ++u) {
          const float w = gaussion_weights_[(v + half_block_size_)
              * block_size_ + u + half_block_size_];
          slave_sum += w * static_cast<int>(slave_image.data[(y + v * scale)
              * slave_col + x + u * scale]);
        }
      }
      float slave_avg = slave_sum / gaussion_weight_sum_;
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(2) << "it_slave " << it_slave << " slave_avg " << slave_avg
                << " slave x y [" << x << ", " << y << "]";
#endif
      // allow avg value diff 2x max pixel diff since the exposure of left and right cam
      // may be very different
      if (master_avg - slave_avg > max_pixel_val_diff * 2
          || master_avg - slave_avg < - max_pixel_val_diff * 2) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
        avg_diff_count++;
#endif
        continue;
      }
      // compute patch value diff
      float patch_diff = 0;
      for (int v = - half_block_size_; v < half_block_size_; ++v) {
        for (int u = - half_block_size_; u < half_block_size_; ++u) {
          int slave_val =
              static_cast<int>(slave_image.data[(y + v * scale) * slave_col + x + u * scale]);
          int master_val =
              static_cast<int>(master_image.data[(master_y + v * scale) * master_col
                  + master_x + u * scale]);
          int diff = (master_val - master_avg) - (slave_val - slave_avg);
          const float w = gaussion_weights_[(v + half_block_size_)* block_size_
              + u + half_block_size_];
          patch_diff += std::abs(diff) * w;
        }
        if (patch_diff > min_patch_val_diff) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
          early_break_count++;
#endif
          break;
        }
      }
      if (patch_diff < min_patch_val_diff) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
        find_count++;
#endif
        second_min_patch_val_diff = min_patch_val_diff;
        min_patch_val_diff = patch_diff;
        best_slave_x = x;
        best_slave_y = y;
        second_best_search_dist_id = best_search_dist_id;
        best_search_dist_id = it_slave;
      }
    }
  }
  if (min_patch_diff2_ptr != nullptr) {
    *min_patch_diff2_ptr = min_patch_val_diff;
  }
  if (second_min_patch_diff2_ptr != nullptr) {
    *second_min_patch_diff2_ptr = second_min_patch_val_diff;
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << "  avg_diff_count " << avg_diff_count
          << " skip_count " << skip_count
          << " early_break_count " << early_break_count
          << " candidate_count " << candidate_count
          << " find_count " << find_count;
#endif
  if (min_patch_val_diff > max_patch_val_diff) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << " min_patch_val_diff = " << min_patch_val_diff
            << " max_patch_val_diff " << max_patch_val_diff;
#endif
    return false;
  }
  if (best_slave_x_ptr != nullptr) {
    *best_slave_x_ptr = best_slave_x;
  }
  if (best_slave_y_ptr != nullptr) {
    *best_slave_y_ptr = best_slave_y;
  }
  if (best_search_dist_id_ptr != nullptr) {
    *best_search_dist_id_ptr = best_search_dist_id;
  }
  if (second_best_search_dist_id_ptr != nullptr) {
    *second_best_search_dist_id_ptr = second_best_search_dist_id;
  }
  return true;
}
bool SlaveImgFeatureDetector::detect_features_on_slave_img(
    const cv::Mat& master_image,
    const cv::Mat& slave_image,
    const std::vector<cv::KeyPoint>& master_kps,
    const DuoCalibParam& duo_calib_param,
    std::vector<cv::KeyPoint>* slave_kps_ptr,
    cv::Mat* slave_orb_feat_ptr,
    int max_pixel_val_diff) {
  if (master_kps.empty()) {
    // nothing bad happen
    return true;
  }
  if (master_image.type() != CV_8U) {
    LOG(ERROR) << "master_image.type() " << master_image.type();
    return false;
  }
  if (slave_image.type() != CV_8U) {
    LOG(ERROR) << "slave_image.type() " << slave_image.type();
    return false;
  }
#ifdef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(master_image.cols, master_image.step1());
  CHECK_EQ(slave_image.cols, slave_image.step1());
#endif
  std::vector<cv::Point2f> master_pnts(master_kps.size());
  for (size_t i = 0; i < master_kps.size(); ++i) {
    master_pnts[i] = master_kps[i].pt;
  }
  std::vector<cv::Point2f> master_pnts_undistorted;
  cv::undistortPoints(master_pnts, master_pnts_undistorted,
                      duo_calib_param.Camera.cv_camK_lr[0],
                      duo_calib_param.Camera.cv_dist_coeff_lr[0]);
  Eigen::Matrix4f s_T_m =
      duo_calib_param.Camera.D_T_C_lr[1].inverse() * duo_calib_param.Camera.D_T_C_lr[0];
  cv::Mat_<float> s_R_m(3, 3);
  cv::Mat_<float> s_t_m(3, 1);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      s_R_m(i, j) = s_T_m(i, j);
    }
    s_t_m(i) = s_T_m(i, 3);
  }
  const float cam_baseline_dis = s_T_m.topRightCorner<3, 1>().norm();
  // coarse search min_search_range x cam_baseline_dis -> max_search_range x cam_baseline_dis
  // Note: if min_search_range < 3, it greatly increases the search range in image
  // which reduces speed and increases the likelyhood of false alarm
  constexpr int range_bin_num = 25;
  cv::Mat_<float> coarse_search_range(range_bin_num, 1);
  // in the inverse depth space, uniformally generate [min_range, max_range] search range
  // i = 0 -> min_range
  // i = (range_bin_num - 1) -> max_range
  // ask sid for the math
  const float tmp_x =
      (max_feature_distance_over_baseline_ratio_ / min_feature_distance_over_baseline_ratio_ - 1.f)
          / (range_bin_num - 1);
  const float tmp_y = 1 - tmp_x;
  for (int i = 0; i < coarse_search_range.rows; ++i) {
    coarse_search_range(i) =
        max_feature_distance_over_baseline_ratio_ * cam_baseline_dis
            / ((range_bin_num - i) * tmp_x + tmp_y);
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(2) << "coarse_search_range " << coarse_search_range.t();
#endif
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
  cv::Mat slave_image_debug = slave_image.clone();
#endif
  CHECK_NOTNULL(slave_kps_ptr);
  slave_kps_ptr->clear();
  slave_kps_ptr->reserve(master_pnts_undistorted.size());
  for (size_t it_master = 0; it_master < master_pnts_undistorted.size(); ++it_master) {
    const int master_x = std::roundf(master_pnts[it_master].x + 0.5);
    const int master_y = std::roundf(master_pnts[it_master].y + 0.5);
    if (master_x < half_block_size_
        || master_y < half_block_size_
        || master_x >= master_image.cols - half_block_size_
        || master_y >= master_image.rows - half_block_size_) {
      continue;
    }
    const auto& kp_master_undist = master_pnts_undistorted[it_master];
    cv::Mat_<float> pnt_master(3, 1);
    // coarse search
    pnt_master(0) = kp_master_undist.x;
    pnt_master(1) = kp_master_undist.y;
    pnt_master(2) = 1;
    int min_patch_diff2 = 99999;
    int second_min_patch_diff2 = 99999;
    int best_search_dist_id = -1;
    int second_best_search_dist_id = -1;
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << "before coarse det it_master " << it_master
            << " kp id " << master_kps[it_master].class_id
            << " master pixel [" << master_x << ", " << master_y << "]";
#endif
    if (!detect_features_on_slave_img_helper(master_image, slave_image,
                                             duo_calib_param, max_pixel_val_diff * 2,  // coarse
                                             master_x, master_y,
                                             s_R_m, s_t_m, pnt_master, coarse_search_range,
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
        &slave_image_debug,
#endif
                                             &min_patch_diff2,
                                             &second_min_patch_diff2,
                                             nullptr,  // &best_slave_x,
                                             nullptr,  // &best_slave_y,
                                             &best_search_dist_id,
                                             &second_best_search_dist_id)) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(2) << "coarse det failed min_patch_diff2 " << min_patch_diff2;
#endif
      continue;
    }
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
    CHECK_GE(best_search_dist_id, 0);
    // cv::circle(slave_image_debug, cv::Point2i(best_slave_x, best_slave_y), 8, 255);
#endif
    // threshold test
    if (second_min_patch_diff2 * 2 < min_patch_diff2 * 3) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      CHECK_GE(second_best_search_dist_id, 0);
#endif
      // if the sampled pos are dense, a couple of samples may get close to the true position
      // So they all have low pixel diff values, which should not be discouraged.
      if (second_best_search_dist_id > best_search_dist_id + 1
          || second_best_search_dist_id < best_search_dist_id - 1) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
        VLOG(2) << "thresh test failed min_patch_diff2 / second_min_patch_diff2 "
                << min_patch_diff2 << " / " << second_min_patch_diff2
                << " = " << static_cast<float>(min_patch_diff2) / second_min_patch_diff2
                << " best id / second id " << best_search_dist_id
                << " / " << second_best_search_dist_id;
#endif
        continue;
      }
    }
    // fine search
    float min_search_dis = coarse_search_range(0);
    if (best_search_dist_id > 0) {
      min_search_dis = coarse_search_range(best_search_dist_id - 1);
    }
    float max_search_dis = coarse_search_range(coarse_search_range.rows - 1);
    if (best_search_dist_id < coarse_search_range.rows - 1) {
      max_search_dis = coarse_search_range(best_search_dist_id + 1);
    }
    cv::Mat_<float> search_range_fine(10, 1);
    // Do not use uniform distance
    // Solve the following equation
    // min_search_dis = X / (Y + search_range_fine.rows)
    // max_search_dis = X / (Y + 1)
    // ->
    // X = max_search_dis * Y + max_search_dis
    // ->
    // min_search_dis * Y + min_search_dis * search_range_fine.rows
    // = max_search_dis * Y + max_search_dis
    // ->
    const float Y =
        (max_search_dis - min_search_dis * search_range_fine.rows)
            / (min_search_dis - max_search_dis);
    const float X = max_search_dis * Y + max_search_dis;
    for (int i = 0; i < search_range_fine.rows; ++i) {
      search_range_fine(i) = X / (Y + search_range_fine.rows - i);
    }
#ifndef __FEATURE_UTILS_NO_DEBUG__
    VLOG(2) << "fine det it_master " << it_master
            << " coarse det min_patch_diff2 " << min_patch_diff2
            << " best_search_dist_id " << best_search_dist_id
            << " search range " << search_range_fine(0)
            << " " << search_range_fine(1)
            << " .. " << search_range_fine(8)
            << " " << search_range_fine(9);
#endif
    int best_slave_x, best_slave_y;
    if (!detect_features_on_slave_img_helper(master_image, slave_image,
                                             duo_calib_param, max_pixel_val_diff,
                                             master_x, master_y,
                                             s_R_m, s_t_m, pnt_master, search_range_fine,
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
        &slave_image_debug,
#endif
                                             &min_patch_diff2,
                                             nullptr,  // second_min_patch_diff2
                                             &best_slave_x,
                                             &best_slave_y,
                                             nullptr /* best_search_dist_id */,
                                             nullptr /* second_best_search_dist_id */)) {
#ifndef __FEATURE_UTILS_NO_DEBUG__
      VLOG(2) << "fine det failed.  min_patch_diff2 " << min_patch_diff2
            << " > " << gaussion_weight_sum_ * max_pixel_val_diff;
#endif
      continue;
    }
    CHECK_GE(best_search_dist_id, 0);
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
    cv::circle(slave_image_debug, cv::Point2i(best_slave_x, best_slave_y), 8, 255);
    cv::putText(slave_image_debug,
                boost::lexical_cast<std::string>(master_kps[it_master].class_id),
                cv::Point2i(best_slave_x, best_slave_y),
                cv::FONT_HERSHEY_PLAIN, 1, 255, 1);
#endif
    // push this good point
    cv::KeyPoint kp_slave = master_kps[it_master];
    kp_slave.pt.x = best_slave_x;
    kp_slave.pt.y = best_slave_y;
    slave_kps_ptr->push_back(kp_slave);
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(1) << "master kp # " << master_kps.size() << " slave kp # " << slave_kps_ptr->size();
#ifdef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
  cout << "write to /tmp/slave_image_debug.png" << endl;
  cv::imwrite("/tmp/slave_image_debug.png", slave_image_debug);
#endif
#endif
  if (slave_kps_ptr->empty()) {
    return false;
  }
  // compute desc
  if (slave_orb_feat_ptr != nullptr) {
    slave_orb_feat_ptr->create(slave_kps_ptr->size(), 32, CV_8U);
#ifdef __ARM_NEON__
    ORBextractor::computeDescriptorsN512(
          slave_image, *slave_kps_ptr, slave_orb_feat_ptr);
#else
    ORBextractor::computeDescriptors(slave_image, *slave_kps_ptr, slave_orb_feat_ptr);
#endif
  }
  return true;
}

vector<vector<cv::DMatch> > neon_orb_match(
    const cv::Mat& desc_query,
    const cv::Mat& matching_mask,
    const cv::Mat& orb_desc_training) {
  vector<vector<cv::DMatch>> matches;
  matches.resize(desc_query.rows);
  CHECK_EQ(orb_desc_training.step1(), 32);
  for (int it_query = 0; it_query < desc_query.rows; ++it_query) {
    int d1 = 256;
    int d2 = 256;
    int trainIdx1 = -1;
    int trainIdx2 = -1;
#ifdef __ARM_NEON__
    // const unsigned char *a = desc_query.ptr<unsigned char>();
    const unsigned char *a = desc_query.data + it_query * 32;
    for (int it_orb_this_rig = 0; it_orb_this_rig < orb_desc_training.rows; ++it_orb_this_rig) {
      if (matching_mask.at<uchar>(it_query, it_orb_this_rig) == 0x00) {
        continue;
      }
      // copied from OpenCV
      // .row is very slow
      // const unsigned char *b = orb_desc_training.row(it_orb_this_rig).ptr<unsigned char>();
      const unsigned char *b = orb_desc_training.data + it_orb_this_rig * 32;
      uint32x4_t bits = vmovq_n_u32(0);
      for (size_t i = 0; i < 32; i += 16) {
          uint8x16_t A_vec = vld1q_u8(a + i);
          uint8x16_t B_vec = vld1q_u8(b + i);
          uint8x16_t AxorB = veorq_u8(A_vec, B_vec);
          uint8x16_t bitsSet = vcntq_u8(AxorB);
          uint16x8_t bitSet8 = vpaddlq_u8(bitsSet);
          uint32x4_t bitSet4 = vpaddlq_u16(bitSet8);
          bits = vaddq_u32(bits, bitSet4);
      }
      uint64x2_t bitSet2 = vpaddlq_u32(bits);
      int dist = vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 0);
      dist += vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 2);
      // float dist_float = float(dist) / 255.f;
      if (d1 > dist) {
        trainIdx2 = trainIdx1;
        d2 = d1;
        trainIdx1 = it_orb_this_rig;
        d1 = dist;
      } else if (d2 > dist) {
        trainIdx2 = it_orb_this_rig;
        d2 = dist;
      }
    }
#else
    LOG(FATAL) << "neon_orb_match is called without __ARM_NEON__";
#endif

    matches[it_query].resize(2);
    matches[it_query][0].distance = d1;
    matches[it_query][0].trainIdx = trainIdx1;
    matches[it_query][1].distance = d2;
    matches[it_query][1].trainIdx = trainIdx2;
  }
  return matches;
}
// only match once (knn=1)
vector<vector<cv::DMatch> > neon_orb_match_nn(const cv::Mat& desc_query,
                                              const cv::Mat& matching_mask,
                                              const cv::Mat& orb_desc_training) {
  vector<vector<cv::DMatch>> matches;
  matches.resize(desc_query.rows);
  CHECK_EQ(desc_query.step1(), 32);
  CHECK_EQ(orb_desc_training.step1(), 32);
  for (int it_query = 0; it_query < desc_query.rows; ++it_query) {
    int d1 = 256;
    int trainIdx1 = -1;
#ifdef __ARM_NEON__
    // const unsigned char *a = desc_query.ptr<unsigned char>();
    const unsigned char *a = desc_query.data + it_query * 32;
    for (int it_orb_this_rig = 0; it_orb_this_rig < orb_desc_training.rows; ++it_orb_this_rig) {
      if (matching_mask.at<uchar>(it_query, it_orb_this_rig) == 0x00) {
        continue;
      }
      // copied from OpenCV
      // .row is very slow
      // const unsigned char *b = orb_desc_training.row(it_orb_this_rig).ptr<unsigned char>();
      const unsigned char *b = orb_desc_training.data + it_orb_this_rig * 32;
      uint32x4_t bits = vmovq_n_u32(0);
      for (size_t i = 0; i < 32; i += 16) {
        uint8x16_t A_vec = vld1q_u8(a + i);
        uint8x16_t B_vec = vld1q_u8(b + i);
        uint8x16_t AxorB = veorq_u8(A_vec, B_vec);
        uint8x16_t bitsSet = vcntq_u8(AxorB);
        uint16x8_t bitSet8 = vpaddlq_u8(bitsSet);
        uint32x4_t bitSet4 = vpaddlq_u16(bitSet8);
        bits = vaddq_u32(bits, bitSet4);
      }
      uint64x2_t bitSet2 = vpaddlq_u32(bits);
      int dist = vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 0);
      dist += vgetq_lane_s32(vreinterpretq_s32_u64(bitSet2), 2);
      // float dist_float = float(dist) / 255.f;
      if (d1 > dist) {
        trainIdx1 = it_orb_this_rig;
        d1 = dist;
      }
    }
#else
    LOG(FATAL) << "neon_orb_match_nn is called without __ARM_NEON__";
#endif
    if (trainIdx1 >= 0) {
      matches[it_query].resize(1);
      matches[it_query][0].distance = d1;
      matches[it_query][0].trainIdx = trainIdx1;
    }
  }
  return matches;
}
int neon_find_close_points(
    float query_u,
    float query_v,
    int training_num,
    float * training_u,
    float * training_v,
    float range_sq,
    cv::Mat * within_range_mask_ptr) {
  int in_range_count = 0;
#ifdef __ARM_NEON__
  float32x4_t query_u_32x4 = {query_u, query_u, query_u, query_u};
  float32x4_t query_v_32x4 = {query_v, query_v, query_v, query_v};
  float32x4_t range2_32x4 = {range_sq, range_sq, range_sq, range_sq};
  int processed_num = 0;
  for (size_t i = 0; i <= training_num - 4; i += 4) {
    float32x4_t vecU = vld1q_f32(training_u + i);
    float32x4_t vecV = vld1q_f32(training_v + i);
    float32x4_t vecUres = vsubq_f32(vecU, query_u_32x4);
    float32x4_t vecVres = vsubq_f32(vecV, query_v_32x4);
    float32x4_t vecUres2Vres2 = vmlaq_f32(vmulq_f32(vecUres, vecUres), vecVres, vecVres);
    uint32x4_t within_range = vcleq_f32(vecUres2Vres2, range2_32x4);
    within_range_mask_ptr->at<uchar>(0, i + 0) = within_range[0] != 0;
    within_range_mask_ptr->at<uchar>(0, i + 1) = within_range[1] != 0;
    within_range_mask_ptr->at<uchar>(0, i + 2) = within_range[2] != 0;
    within_range_mask_ptr->at<uchar>(0, i + 3) = within_range[3] != 0;
    /*
    for (int j = 0; j < 4; j++) {
      LOG(ERROR)
          << "vecU " << float(vecU[j])
          << " vecV " << float(vecV[j])
          << " vecUres "  << float(vecUres[j])
          << " vecVres "  << float(vecVres[j])
          << " vecUres2Vres2 " << float(vecUres2Vres2[j])
          << " query_u_32x4 "  << float(query_u_32x4[j])
          << " query_v_32x4 "  << float(vecUres[j])
          << " within_range " << (unsigned int)(within_range[j])
          << " reprojDis2 "  << float(reprojDis2[j])
          << " matching_mask.at<uchar>(0, i + j) " << (int)(matching_mask.at<uchar>(0, i + j));
    }*/
    in_range_count += within_range[0] != 0;
    in_range_count += within_range[1] != 0;
    in_range_count += within_range[2] != 0;
    in_range_count += within_range[3] != 0;
    processed_num += 4;
  }
  for (size_t i = processed_num; i < training_num; i++) {
    const float distance2 =
        (query_u - training_u[i])
        * (query_u - training_u[i])
       + (query_v - training_v[i])
       * (query_v - training_v[i]);
    if (distance2 < range_sq) {
      within_range_mask_ptr->at<uchar>(0, i) = 0x01;
      in_range_count++;
    } else {
      within_range_mask_ptr->at<uchar>(0, i) = 0x00;
    }
  }
#else
  LOG(FATAL) << "neon_find_close_points is called without __ARM_NEON__";
#endif
  return in_range_count;
}
// copied from HarrisScoreCalculatorFloat
void OpencvHarrisScoreCalculator::Get2dMaxima(std::vector<PointWithScore>& points,  // NOLINT
                                              Score_t absoluteThreshold) const {
  // Do the 8-neighbor nonmax suppression.
  const int stride = _scores.step1();
  const int rows_end = _scores.rows - 2;
  points.reserve(4000);
  for (int j = 2; j < rows_end; ++j) {
    const float* p = &_scores.at<float>(j, 2);
    const float* const p_begin = p;
    const float* const p_end = &_scores.at<float>(j, stride - 2);
    bool last = false;
    while (p < p_end) {
      const float* const center = p;
      ++p;
      if (last) {
        last = false;
        continue;
      }
      if (*center < absoluteThreshold)
        continue;
      if (*(center + 1) > *center)
        continue;
      if (*(center - 1) >= *center)
        continue;
      const float* const p1 = (center + stride);
      const float* const p2 = (center - stride);
      if (*p1 > *center)
        continue;
      if (*p2 >= *center)
        continue;
      if (*(p1 + 1) > *center)
        continue;
      if (*(p1 - 1) >= *center)
        continue;
      if (*(p2 + 1) > *center)
        continue;
      if (*(p2 - 1) >= *center)
        continue;
      const int i = p - p_begin;
#ifdef USE_SIMPLE_POINT_WITH_SCORE
      points.push_back(PointWithScore(*center, i, j - 2));
#else
      #error
      points.push_back(PointWithScore(cv::Point2i(i, j - 2), *center));
#endif
    }
  }
#ifndef __FEATURE_UTILS_NO_DEBUG__
  VLOG(1) << "OpencvHarrisScoreCalculator::Get2dMaxima points.size() " << points.size();
#endif
}
void OpencvHarrisScoreCalculator::InitializeScores() {
  // kappa = 1 / 16 is used by brisk harris
  _scores.create(_img.size(), CV_32F);
  cv::cornerHarris(_img, _scores, 5, 3, 0.04f);  // double k, int borderType )
#ifndef __FEATURE_UTILS_NO_DEBUG__
  if (VLOG_IS_ON(2)) {
    cv::imwrite("/tmp/harris_score.png", _scores * 10000);
    std::cout << "_img" << std::endl;
    for (int i = 95; i < 105; i++) {
      for (int j = 95; j < 105; j++) {
        std::cout << int(_img.at<uchar>(i, j)) << " ";
      }
      std::cout << std::endl;
    }
    std::cout << "harris" << std::endl;
    for (int i = 95; i < 105; i++) {
      for (int j = 95; j < 105; j++) {
        std::cout << _scores.at<float>(i, j) << " ";
      }
      std::cout << std::endl;
    }
  }
#endif
}

}  // namespace XP
