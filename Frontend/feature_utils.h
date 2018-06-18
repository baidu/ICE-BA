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
#ifndef XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
#define XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_

#include "ORBextractor.h"
#include "param.h"
#include "cameras/PinholeCamera.hpp"  // for vio::cameras::CameraBase
#include <glog/logging.h>
#include <opencv2/video/tracking.hpp>
#include <brisk/internal/score-calculator.h>

#include <map>
#include <random>
#include <vector>
#include <mutex>
namespace XP {
#ifndef __ARM_NEON__
/******************************************************************************
 * fast_pyra_down_internal       This interface should not used directly
 * @param[in]   img_in_smooth    input image
 * @param[out]  _img_in_small    output image, the memory of _img_in_small must
 *                               be pre-allocated before calling this function
 * TODO(yanghongtian) : use SSE intrinsics here
 */
inline void fast_pyra_down_internal(const cv::Mat& img_in_smooth, cv::Mat* _img_in_small) {
  CHECK_NOTNULL(_img_in_small);
  CHECK_NOTNULL(_img_in_small->data);
  CHECK_EQ(img_in_smooth.type(), CV_8U);
  CHECK_EQ(img_in_smooth.rows & 1, 0);
  CHECK_EQ(img_in_smooth.cols & 3, 0);
  cv::Mat& img_in_small = *_img_in_small;

  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  int sum0, sum1, sum2, sum3;
  const uchar* data0_ptr = img_in_smooth.data;
  const uchar* data1_ptr = img_in_smooth.data + width_step_in;
  uchar* target_data_ptr = img_in_small.data;

  for (int rows = img_in_small.rows, y = 0; y < rows; ++y) {
    for (int cols = img_in_small.cols & ~3, x = 0; x < cols; x += 4, target_data_ptr += 4) {
      sum0 = ((static_cast<int>(*data0_ptr) + static_cast<int>(*(data0_ptr + 1))) +
          (static_cast<int>(*data1_ptr) + static_cast<int>(*(data1_ptr + 1)))) / 4;
      sum1 = ((static_cast<int>(*(data0_ptr + 2)) + static_cast<int>(*(data0_ptr + 3))) +
          (static_cast<int>(*(data1_ptr + 2)) + static_cast<int>(*(data1_ptr + 3)))) / 4;
      sum2 = ((static_cast<int>(*(data0_ptr + 4)) + static_cast<int>(*(data0_ptr + 5))) +
          (static_cast<int>(*(data1_ptr + 4)) + static_cast<int>(*(data1_ptr + 5)))) / 4;
      sum3 = ((static_cast<int>(*(data0_ptr + 6)) + static_cast<int>(*(data0_ptr + 7))) +
          (static_cast<int>(*(data1_ptr + 6)) + static_cast<int>(*(data1_ptr + 7)))) / 4;

      *(target_data_ptr) = static_cast<uchar>(sum0);
      *(target_data_ptr + 1) = static_cast<uchar>(sum1);
      *(target_data_ptr + 2) = static_cast<uchar>(sum2);
      *(target_data_ptr + 3) = static_cast<uchar>(sum3);
      data0_ptr += 8;
      data1_ptr += 8;
    }
    data0_ptr += width_step_in;
    data1_ptr += width_step_in;
  }
}
#else
inline void fast_pyra_down_internal(const cv::Mat& img_in_smooth, cv::Mat* _img_in_small) {
  CHECK_NOTNULL(_img_in_small);
  CHECK_NOTNULL(_img_in_small->data);
  CHECK_EQ(img_in_smooth.type(), CV_8U);
  CHECK_EQ(img_in_smooth.rows & 1, 0);

  cv::Mat& img_in_small = *_img_in_small;

  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  const int width_step_small = img_in_small.step1();
  uchar* target_data_ptr = img_in_small.data;
  uchar* data0_ptr = img_in_smooth.data;
  uchar* data1_ptr = img_in_smooth.data + width_step_in;

  // provide hits to gcc for optimization
  const int cols = img_in_small.cols - 8;
  for (int rows = img_in_small.rows, y = 0; y < rows; ++y) {
    int x = 0;
    for (; x <= cols; x += 8, target_data_ptr += 8) {
      // loading 32 pixels per row from source image
      uint8x8x2_t row0 = vld2_u8(data0_ptr);
      uint8x8x2_t row1 = vld2_u8(data1_ptr);
      // compute 16 target pixels per loop
      uint16x8_t sum = vaddq_u16(vaddl_u8(row0.val[0], row0.val[1]),
                                 vaddl_u8(row1.val[0], row1.val[1]));
      vst1_u8(target_data_ptr, vmovn_u16(vshrq_n_u16(sum, 2)));
      data0_ptr += 16;
      data1_ptr += 16;
    }
    for (; x < img_in_small.cols; ++x) {
      int sum = ((static_cast<int>(*data0_ptr) + static_cast<int>(*(data0_ptr + 1))) +
                 (static_cast<int>(*data1_ptr) + static_cast<int>(*(data1_ptr + 1)))) / 4;
      *target_data_ptr++ = static_cast<uchar>(sum);
      data0_ptr += 2;
      data1_ptr += 2;
    }
    data0_ptr += width_step_in;
    data1_ptr += width_step_in;
  }
}
#endif  // __ARM_NEON__

/**************************************************************************************
 * fast_pyra_down : used this function for half-down sample image
 * @param[in]  img_in_smooth  input image
 * @param[in]  data           existence storage for the downsample images
 *                            default is nullptr, don't use existence
 * @return   half-down sample image.
 */
inline cv::Mat fast_pyra_down(const cv::Mat& img_in_smooth, uchar* const data = nullptr) {
  constexpr int compress_ratio = 2;
  if (data == nullptr) {
    cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                         img_in_smooth.cols / compress_ratio,
                         CV_8U);
    fast_pyra_down_internal(img_in_smooth, &img_in_small);
    return img_in_small;
  } else {
    cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                         img_in_smooth.cols / compress_ratio,
                         CV_8U, data);
    fast_pyra_down_internal(img_in_smooth, &img_in_small);
    return img_in_small;
  }
}
/**************************************************************
 * @param[in] img          original image, level 0
 * @param[in] max_level    pyramid level, including max_level
 * @param[in] pyra_buf_ptr buffer address of pyramid buffer
 * @param[out] pyramids    pyramids
 * [NOTE] pyramids include: level 0   -> original image size
 *                          level 1   -> original image size / 2
 *                          level 2   -> original image size / 4
 *                          level 3   -> original image size / 8
 */
void build_pyramids(const cv::Mat& img, int max_level,
                    std::vector<cv::Mat>* pyramids,
                    uchar* const pyra_buf_ptr = NULL);


class IdGenerator {
 public:
  IdGenerator() : id_(0) {}
  int get() {
    std::lock_guard<std::mutex> lock(id_mutex_);
    ++id_;
    // wrap around
    if (id_ <= 0) id_ = 1;  // 0 indicates an invalid id!
    return id_;
  }
  void reset(int id) { id_ = id; }
 private:
  std::mutex id_mutex_;
  int id_;
};

// This class performs feature (re)detection + feature propagation with optical flow by
// only considering the master view
class FeatureTrackDetector {
 public:
  struct FeatureTrack {
    explicit FeatureTrack(const cv::Point2f pt) : length(1), isActive(true), point(pt) {}
    int length;
    bool isActive;
    cv::Point2f point;
  };

  FeatureTrackDetector(const int length_thres,
                       const float drop_rate,
                       const bool use_fast,  // True: fast; False: ShiTomasi
                       const int uniform_radius,  // <= 5 means no uniformity suppression
                       const cv::Size& img_size);
  bool optical_flow_and_detect(const cv::Mat_<uchar>& mask,
                               const cv::Mat& pre_image_orb_feature,
                               const std::vector<cv::KeyPoint>& prev_img_kpts,
                               int request_feat_num,
                               int pyra_level_det,
                               int fast_thresh,
                               std::vector<cv::KeyPoint>* key_pnts_ptr,
                               cv::Mat* orb_feat_ptr = nullptr,
      // Before OF process, add init_pixel_shift to each (x, y) of
      // pre_image_keypoints as the initial value
                               const cv::Vec2f& init_pixel_shift = cv::Vec2f(0, 0),
      // provide the following info enables feature location prediction
                               const cv::Matx33f* K_ptr = nullptr,
                               const cv::Mat_<float>* dist_ptr = nullptr,
                               const cv::Matx33f* old_R_new_ptr = nullptr,
                               const bool absolute_static = false);
  void update_img_pyramids() {
    curr_img_pyramids_.swap(prev_img_pyramids_);
    curr_pyramids_buffer_.swap(prev_pyramids_buffer_);
  }
  bool detect(const cv::Mat& img_in_smooth,
              const cv::Mat_<uchar>& mask,
              int request_feat_num,
              int pyra_level,  // Total pyramid levels, including the base image
              int fast_thresh,
              std::vector<cv::KeyPoint>* key_pnts_ptr,
              cv::Mat* orb_feat_ptr);

  inline size_t feature_tracks_number() const { return feature_tracks_map_.size(); }
  int add_new_feature_track(const cv::Point2f pt);  // Return the added feature track id
  void mark_all_feature_tracks_dead();
  void filter_static_features(std::vector<cv::KeyPoint>* key_pnts);
  void update_feature_tracks(std::vector<cv::KeyPoint>* key_pnts);
  void flush_feature_tracks(const std::vector<cv::KeyPoint>& key_pnts);

 protected:
  // Once feature track exceeds this threshold, we break this feature track randomly
  // with drop_rate.
  // [NOTE] Use a negative drop rate for no drop out
  int length_threshold_;
  float drop_rate_;
  bool use_fast_;
  int uniform_radius_;
  IdGenerator id_generator_;

  // A very simple-minded data structure to bookkeep feature track ids and lengths
  // We will have at most request_feat_num active feature tracks
  std::map<int,  FeatureTrack> feature_tracks_map_;

  // A mask to hold the mask of the union of input mask and optical flow feats mask
  cv::Mat_<uchar> mask_with_of_out_;

 private:
  // Random generator
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_;

 private:
  // For XP optical flow
  std::shared_ptr<uchar> prev_pyramids_buffer_;  // buffer for storing previous pyramids
  std::shared_ptr<uchar> curr_pyramids_buffer_;  // buffer for storing current pyramids
  std::vector<cv::Mat> prev_img_pyramids_;
  std::vector<cv::Mat> curr_img_pyramids_;

 public:
  static constexpr int kMaxPyraLevelOF = 3;
  uchar* const get_prev_pyramids_buffer() const {
    return prev_pyramids_buffer_.get();
  }

  uchar* const get_curr_pyramids_buffer() const {
    return curr_pyramids_buffer_.get();
  }
  std::vector<cv::Mat>& get_prev_img_pyramids() {
    return prev_img_pyramids_;
  }
  std::vector<cv::Mat>& get_curr_img_pyramids() {
    return curr_img_pyramids_;
  }
  enum {
    BUILD_TO_CURR = 0,
    BUILD_TO_PREV = 1
  };

  void build_img_pyramids(const cv::Mat& img_in_smooth, int build_type = BUILD_TO_CURR) {
    if (build_type == BUILD_TO_CURR) {
      build_pyramids(img_in_smooth, kMaxPyraLevelOF,
                     &curr_img_pyramids_, curr_pyramids_buffer_.get());
    } else {
      build_pyramids(img_in_smooth, kMaxPyraLevelOF,
                     &prev_img_pyramids_, prev_pyramids_buffer_.get());
    }
  }
};

// detect features on slave img
// assume slave image is lr = 1, master is lr = 0
// copy the kp class_id from master to slave points
#ifndef __FEATURE_UTILS_NO_DEBUG__
#undef DEBUG_DETECT_FEATURES_ON_SLAVE_IMG
#endif
class SlaveImgFeatureDetector {
 public:
  enum DetectSlaveFeatureType {
    EPIPOLAR_LINE_SEARCH = 0,
  };
  explicit SlaveImgFeatureDetector(int block_size = 8,  // block matching size. Multiple of 4
                                   DetectSlaveFeatureType method =
                                   DetectSlaveFeatureType::EPIPOLAR_LINE_SEARCH,
                                   float min_feature_distance_over_baseline_ratio = 3,
                                   float max_feature_distance_over_baseline_ratio = 3000);
  ~SlaveImgFeatureDetector();
  bool detect_features_on_slave_img(const cv::Mat& master_image,
                                    const cv::Mat& slave_image,
                                    const std::vector<cv::KeyPoint>& master_kps,
                                    const DuoCalibParam& duo_calib_param,
                                    std::vector<cv::KeyPoint>* slave_kps_ptr,
                                    cv::Mat* slave_orb_feat_ptr = nullptr,
                                    int max_pixel_val_diff = 15);

 private:
  bool detect_features_on_slave_img_helper(const cv::Mat& master_image,
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
                                           int* second_best_search_dist_id_ptr);

  // the most light data structure
  float* gaussion_weights_;
  float gaussion_weight_sum_;
  const int block_size_;
  const int half_block_size_;
  const DetectSlaveFeatureType method_;
  const float min_feature_distance_over_baseline_ratio_;
  const float max_feature_distance_over_baseline_ratio_;
};

std::vector<std::vector<cv::DMatch> > neon_orb_match(
    const cv::Mat& desc_query,
    const cv::Mat& matching_mask,
    const cv::Mat& orb_desc_training);
std::vector<std::vector<cv::DMatch> > neon_orb_match_nn(const cv::Mat& desc_query,
                                                        const cv::Mat& matching_mask,
                                                        const cv::Mat& orb_desc_training);

// return in_range_count
int neon_find_close_points(
    float query_u,
    float query_v,
    int training_num,
    float * training_u,
    float * training_v,
    float range_sq,  // square value
    cv::Mat * within_range_mask_ptr);

// This is class is created to use brisk scale-space-feature-detector
// This is still under experiments
// There is localization error and speed issue
class OpencvHarrisScoreCalculator : public brisk::ScoreCalculator<float> {
 public:
  // copied from HarrisScoreCalculator
  inline double Score(double u, double v) const override {
    // Simple bilinear interpolation - no checking (for speed).
    const int u_int = static_cast<int>(u);
    const int v_int = static_cast<int>(v);
    if (u_int + 1 >= _scores.cols || v_int + 1 >= _scores.rows || u_int < 0
        || v_int < 0)
      return 0.0;
    const float ru = u - static_cast<float>(u_int);
    const float rv = v - static_cast<float>(v_int);
    const float oneMinus_ru = 1.f - ru;
    const float oneMinus_rv = 1.f - rv;
    return oneMinus_rv
        * (oneMinus_ru * _scores.at<float>(v_int, u_int)
            + ru * _scores.at<float>(v_int, u_int + 1))
        + rv
            * (oneMinus_ru * _scores.at<float>(v_int + 1, u_int)
                + ru * _scores.at<float>(v_int + 1, u_int + 1));
  }
  // copied from HarrisScoreCalculator
  inline Score_t Score(int u, int v) const override {
    return _scores.at<float>(v, u);
  }
  void Get2dMaxima(std::vector<PointWithScore>& points,  // NOLINT
                   Score_t absoluteThreshold = 0) const override;

 protected:
  void InitializeScores() override;
};

// Subpixel feature alignment (1D & 2D version, borrowed from SVO implementation)
namespace align {
bool align1D(const cv::Mat& cur_img,
             const Eigen::Vector2f& dir,  // direction in which the patch is allowed to move
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             Eigen::Vector2f* cur_px_estimate,
             float* h_inv);

bool align2D(const cv::Mat& cur_img,
             uint8_t* ref_patch_with_border,
             uint8_t* ref_patch,
             const int n_iter,
             Eigen::Vector2f* cur_px_estimate,
             bool no_simd = false);

bool align2D_SSE2(const cv::Mat& cur_img,
                  uint8_t* ref_patch_with_border,
                  uint8_t* ref_patch,
                  const int n_iter,
                  Eigen::Vector2f* cur_px_estimate);
#ifdef __ARM_NEON__
bool align2D_NEON(const cv::Mat& cur_img,
                  const uint8_t* ref_patch_with_border,
                  const uint8_t* ref_patch,
                  const int n_iter,
                  Eigen::Vector2f* cur_px_estimate);
#endif
}  // namespace align

// Warp a patch from the reference view to the current view (borrowed from SVO implementation)
namespace warp {
bool getWarpMatrixAffine(const vio::cameras::CameraBase& cam_ref,
                         const vio::cameras::CameraBase& cam_cur,
                         const Eigen::Vector2f& px_ref,
                         const Eigen::Vector3f& f_ref,
                         const float depth_ref,
                         const Eigen::Matrix3f& R_cur_ref,
                         const Eigen::Vector3f& t_cur_ref,
                         const int level_ref,
                         Eigen::Matrix2f* A_cur_ref);

int getBestSearchLevel(const Eigen::Matrix2f& A_cur_ref,
                       const int max_level);

bool warpAffine(const Eigen::Matrix2f& A_cur_ref,
                const cv::Mat& img_ref,
                const Eigen::Vector2f& px_ref,
                const int level_ref,
                const int level_cur,
                const int halfpatch_size,
                uint8_t* patch);
}  // namespace warp

// Patch-matcher for reprojection-matching and epipolar search in triangulation. (Borrowed from SVO)
class DirectMatcher {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int halfpatch_size_ = 4;
  static const int patch_size_ = 8;

  struct Options {
    bool align_1d;   //!< in epipolar search: align patch 1D along epipolar line
    int max_iter;    //!< number of iterations for aligning the feature patches in gauss newton
    float max_epi_length_optim;   //!< max length of epipolar line to skip epipolar search and directly go to img align  NOLINT
    size_t max_epi_search_steps;  //!< max number of evaluations along epipolar line
    bool subpix_refinement;       //!< do gauss newton feature patch alignment after epipolar search
    bool epi_search_edgelet_filtering;
    float epi_search_edgelet_max_angle;
    Options() :
        align_1d(false),
        max_iter(10),
        max_epi_length_optim(2.0),
        max_epi_search_steps(1000),
        subpix_refinement(true),
        epi_search_edgelet_filtering(true),
        epi_search_edgelet_max_angle(0.7f)
    {}
  } options_;

  // The patch_ and patch_with_border_ have to be aligned for SSE
  uint8_t patch_[patch_size_ * patch_size_] __attribute__((aligned(16)));
  uint8_t patch_with_border_[(patch_size_ + 2) * (patch_size_ + 2)] __attribute__((aligned(16)));
  Eigen::Matrix2f A_cur_ref_;     //!< affine warp matrix
  Eigen::Vector2f epi_dir_;
  float epi_length_;  //!< length of epipolar line segment in pixels (only used for epipolar search)
  float h_inv_;       //!< hessian of 1d image alignment along epipolar line
  Eigen::Vector2f px_cur_;

  DirectMatcher() {}
  ~DirectMatcher() = default;

  // Find a match by directly applying subpix refinement.
  // [NOTE] This function assumes that px_cur is already set to an estimate that is within
  // ~2-3 pixel of the final result!
  bool findMatchDirect(const vio::cameras::CameraBase& cam_ref,
                       const vio::cameras::CameraBase& cam_cur,
                       const Eigen::Vector2f& px_ref,
                       const Eigen::Vector3f& f_ref,
                       const Eigen::Matrix3f& R_cur_ref,
                       const Eigen::Vector3f& t_cur_ref,
                       const int level_ref,
                       const float depth_ref,
                       const std::vector<cv::Mat>& pyrs_ref,
                       const std::vector<cv::Mat>& pyrs_cur,
                       const bool edgelet_feature,
                       Eigen::Vector2f* px_cur);

  /// Find a match by searching along the epipolar line without using any features.
  bool findEpipolarMatchDirect(const vio::cameras::CameraBase& cam_ref,
                               const vio::cameras::CameraBase& cam_cur,
                               const Eigen::Vector2f& px_ref,
                               const Eigen::Vector3f& f_ref,
                               const Eigen::Matrix3f& R_cur_ref,
                               const Eigen::Vector3f& t_cur_ref,
                               const int level_ref,
                               const float d_estimate,
                               const float d_min,
                               const float d_max,
                               const std::vector<cv::Mat>& pyrs_ref,
                               const std::vector<cv::Mat>& pyrs_cur,
                               const cv::Mat_<uchar>& mask_cur,
                               const bool edgelet_feature,
                               Eigen::Vector2f* px_cur,
                               float* depth,
                               int* level_cur,
                               cv::Mat* dbg_cur = nullptr);

  void createPatchFromPatchWithBorder();
};

class ImgFeaturePropagator {
 public:
  ImgFeaturePropagator(
      const Eigen::Matrix3f& cur_camK,
      const Eigen::Matrix3f& ref_camK,
      const cv::Mat_<float>& cur_cv_dist_coeff,
      const cv::Mat_<float>& ref_cv_dist_coeff,
      const cv::Mat_<uchar>& cur_mask,
      int feat_det_pyramid_level,
      float min_feature_distance_over_baseline_ratio,
      float max_feature_distance_over_baseline_ratio);
  ~ImgFeaturePropagator() {}

  bool PropagateFeatures(
      const cv::Mat& cur_img,
      const cv::Mat& ref_img,  // TODO(mingyu): store image pyramids
      const std::vector<cv::KeyPoint>& ref_keypoints,
      const Eigen::Matrix4f& T_ref_cur,
      std::vector<cv::KeyPoint>* cur_keypoints,
      cv::Mat* cur_orb_features = nullptr,
      const bool draw_debug = false);

 protected:
  // options
  const float min_feature_distance_over_baseline_ratio_;
  const float max_feature_distance_over_baseline_ratio_;
  const int feat_det_pyramid_level_;  // e.g. 2

 protected:
  DirectMatcher direct_matcher_;
  std::shared_ptr<vio::cameras::CameraBase> cam_ref_;
  std::shared_ptr<vio::cameras::CameraBase> cam_cur_;
  Eigen::Matrix3f R_cur_ref_;
  Eigen::Vector3f t_cur_ref_;
  cv::Mat_<uchar> mask_cur_;  // TODO(mingyu): may not be used

  // debug visualization
  cv::Mat dbg_img_;
  cv::Mat dbg_ref_;
  cv::Mat dbg_cur_;
  cv::Mat* dbg_cur_ptr_ = nullptr;
};

// Utility functions related to feature detections
bool generate_cam_mask(const cv::Matx33f& K,
                       const cv::Mat_<float>& dist_coeffs,
                       const cv::Size& mask_size,
                       cv::Mat_<uchar>* cam_mask,
                       float* fov_deg);

bool detect_orb_features(const cv::Mat& img_in_raw,
                         const cv::Mat_<uchar>& mask,
                         int request_feat_num,
                         int pyra_level,
                         int fast_thresh,
                         bool use_fast,  // or TomasShi
                         int enforce_uniformity_radius,  // less than 5 means no enforcement
                         std::vector<cv::KeyPoint>* key_pnts_ptr,
                         cv::Mat* orb_feat_ptr,
                         FeatureTrackDetector* feat_track_detector = nullptr,
                         float refine_harris_threshold = -1.f);

// [NOTE] We keep this NON-pyramid general interface to support slave_det_mode = OF
// This function is a wrapper of the pyramid version below.
void propagate_with_optical_flow(const cv::Mat& img_in_smooth,
                                 const cv::Mat_<uchar>& mask,
                                 const cv::Mat& pre_image,
                                 const cv::Mat& pre_image_orb_feature,
                                 const std::vector<cv::KeyPoint>& pre_image_keypoints,
                                 FeatureTrackDetector* feat_tracker_detector,
                                 std::vector<cv::KeyPoint>* key_pnts_ptr,
                                 cv::Mat_<uchar>* mask_with_of_out_ptr,
                                 cv::Mat* orb_feat_OF_ptr = nullptr,
                                 const cv::Vec2f& init_pixel_shift = cv::Vec2f(0, 0),
                                 const cv::Matx33f* K_ptr = nullptr,
                                 const cv::Mat_<float>* dist_ptr = nullptr,
                                 const cv::Matx33f* old_R_new_ptr = nullptr,
                                 const bool absolute_static = false);

// [NOTE] This is pyramid interface is used by FeatureTrackDetctor
void propagate_with_optical_flow(const std::vector<cv::Mat>& img_in_smooth_pyramids,
                                 const cv::Mat_<uchar>& mask,
                                 const std::vector<cv::Mat>& pre_image_pyramids,
                                 const cv::Mat& pre_image_orb_feature,
                                 const std::vector<cv::KeyPoint>& pre_image_keypoints,
                                 FeatureTrackDetector* feat_track_detector,
                                 std::vector<cv::KeyPoint>* key_pnts_ptr,
                                 cv::Mat_<uchar>* mask_with_of_out_ptr,
                                 cv::Mat* orb_feat_OF_ptr,
                                 const cv::Vec2f& init_pixel_shift,
                                 const cv::Matx33f* K_ptr,
                                 const cv::Mat_<float>* dist_ptr,
                                 const cv::Matx33f* old_R_new_ptr,
                                 const bool absolute_static);


void XpEnforceKeyPointUniformity(const cv::Mat& LUT, double radius,
                                 int imgrows, int imgcols, size_t maxNumKpt,
                                 std::vector<brisk::ScoreCalculator<float>::PointWithScore>& points);  // NOLINT

inline cv::Mat fast_pyra_down_original(const cv::Mat& img_in_smooth) {
  constexpr int compress_ratio = 2;
  cv::Mat img_in_small(img_in_smooth.rows / compress_ratio,
                       img_in_smooth.cols / compress_ratio,
                       CV_8U);
#ifndef __FEATURE_UTILS_NO_DEBUG__
  CHECK_EQ(img_in_smooth.type(), CV_8U);
#endif
  // use our own pyra down for faster performance
  const int width_step_in = img_in_smooth.step1();
  const int width_step_small = img_in_small.step1();
  for (int y = 0; y < img_in_small.rows; y++) {
    for (int x = 0; x < img_in_small.cols; x++) {
      // do not use .at<char> which is slow
      const int shift0 = (y * compress_ratio) * width_step_in + x * compress_ratio;
      const int shift1 = shift0 + width_step_in;
      int sum =
          static_cast<int>(*(img_in_smooth.data + shift0)) +
              static_cast<int>(*(img_in_smooth.data + shift1)) +
              static_cast<int>(*(img_in_smooth.data + shift0 + 1)) +
              static_cast<int>(*(img_in_smooth.data + shift1 + 1));
      *(img_in_small.data + y * width_step_small + x) = static_cast<uchar>(sum / 4);
    }
  }
  return img_in_small;
}
}  // namespace XP
#endif  // XP_INCLUDE_XP_UTIL_FEATURE_UTILS_H_
