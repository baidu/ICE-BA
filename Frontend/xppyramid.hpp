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
#ifndef XP_INCLUDE_XP_UTILXPPYRAMID_HPP_
#define XP_INCLUDE_XP_UTILXPPYRAMID_HPP_
#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <glog/logging.h>
#include <boost/pool/object_pool.hpp>
#include <vector>
#include <memory>
using namespace cv;  //NOLINT

namespace XP {
namespace XP_OPTICAL_FLOW {
#define _XP_OPTICAL_FLOW_DEBUG_MODE_

typedef int16_t deriv_type;

typedef struct _interpolation_param {
  _interpolation_param(int _iw00, int _iw01, int _iw10, int _iw11, int _w_bits = 14) :
      iw00(_iw00), iw01(_iw01), iw10(_iw10), iw11(_iw11), w_bits(_w_bits) {}
  int iw00;
  int iw01;
  int iw10;
  int iw11;
  int w_bits;
} InterpolationParam;

// only works for win_size(7, 7)
typedef struct _XPKeyPointRepo {
  int16_t patch[52];  // image patch storage
  float covariance_maxtrix[6];  // covariance matrix
  int16_t xy_gradient[128];  // xy gradient
} XPKeyPointRepo;

typedef struct _XPKeyPointPyramidRepo {
  XPKeyPointRepo pyramids[4];
} XPKeyPointPyramidRepo;


static boost::object_pool<XPKeyPointPyramidRepo> g_xp_of_tracker_pool;
struct XPKeyPoint : public cv::KeyPoint {
  XPKeyPoint() {
    this->pt.x = 0.f;
    this->pt.y = 0.f;
    this->angle = 0.f;
    this->response = 0.f;
    this->need_to_update_repo = true;
  }
  XPKeyPoint(float _x, float _y) {
    this->pt.x = _x;
    this->pt.y = _y;
    this->response = 0.f;
    this->angle = 0.f;
    this->need_to_update_repo = true;
  }
  explicit XPKeyPoint(const cv::Point2f& pt) {
    this->pt = pt;
    this->response = 0.f;
    this->angle = 0.f;
    this->need_to_update_repo = true;
  }

  explicit XPKeyPoint(const cv::KeyPoint& kp) {
    this->pt = kp.pt;
    this->response = kp.response;
    this->angle = kp.angle;
    this->class_id = kp.class_id;
    this->need_to_update_repo = true;
  }
  // guarantee the patch memory is released
  // when doing XPKeyPoint assignment
  XPKeyPoint& operator=(const XPKeyPoint& other) {
    if (this != &other) {
      this->pt = other.pt;
      this->response = other.response;
      this->angle = other.angle;
      this->keypoint_repo = other.keypoint_repo;
      this->need_to_update_repo = other.need_to_update_repo;
    }
    return *this;
  }
  // call this function to allocate memory for the patch of this keypoint,
  // if the patch of keypoint has already allocated, this function will not
  // reallocate memory.
  void allocate() {
    if (keypoint_repo) {
      return;
    }
    keypoint_repo = std::shared_ptr<XPKeyPointPyramidRepo>(
        g_xp_of_tracker_pool.malloc(),
        [](XPKeyPointPyramidRepo* ptr) {
          g_xp_of_tracker_pool.free(ptr);});
  }
  // repository of keypoint, the memory here is automaticcally handled
  std::shared_ptr<XPKeyPointPyramidRepo> keypoint_repo;
  bool need_to_update_repo;
};
/*****************************************************************************
 * @param[in]  _prevPyramids   the pyramids of previous image
 * @param[out] _nextPyramids   the pyramids of current image
 * @param[in]  _prevPts        input keypoints
 * @param[out] _nextPts        output keypoints
 * @param[out] _status         the vector of track status
 * @param[out] _err            the vector of errors
 * @param[in]  win_size        window size, only support 7 x 7 for now
 * @param[in]  max_level       the maximum pyramid level, only surpport 4 levels
 *                             pyramid level = [0, max_level], max_level <= 3
 * @param[in]  start_level     you can do optical flow start from this level
 * @param[in]  criteria        termination condition for iteration
 * @param[in]  flags           not used yet
 * @param[in]  minEigThreshold
 */
void XPcalcOpticalFlowPyrLK(const std::vector<cv::Mat>& _prevPyramids,
                            const std::vector<cv::Mat>& _nextPyramids,
                            std::vector<XPKeyPoint>* _prevPts,
                            std::vector<Point2f>* _nextPts,
                            std::vector<bool>* _status,
                            std::vector<float>* _err,
                            const cv::Size _win_size = cv::Size(7, 7),
                            int _max_level = 3,
                            int _start_level = 0,
                            TermCriteria _criteria =
                            TermCriteria(TermCriteria::COUNT +
                                TermCriteria::EPS, 30, 0.01),
                            int _flags = 0, double _minEigThreshold = 1e-4);



typedef struct XPTrackerInvoker {
  XPTrackerInvoker(const Mat& _prevImg,
                   const Mat& _prevDeriv,
                   const Mat& _nextImg,
                   std::vector<XPKeyPoint>* _prevPts,
                   std::vector<Point2f>* _nextPts,
                   std::vector<bool>* _status,
                   std::vector<float>* _err,
                   Size _winSize,
                   TermCriteria _criteria,
                   int _level,
                   int _maxLevel,
                   int _start_level,
                   int _flags, float _minEigThreshold);

  /*************************************************************
   * To decide whether a image point is in a given region
   * @param[in]    pt        a given image point
   * @param[out]   region    a region descriped by cv::Rect
   * @return       bool      true if not in range
   */
  static inline bool is_keypoint_not_in_valid_range(
      const cv::Point2i& pt,
      const cv::Rect& region) {
    return (pt.x < region.x || pt.x > region.width + region.x ||
        pt.y < region.y || pt.y > region.height + region.y);
  }

  // Invoke optical flow by operator()
  void operator()(const Range& range) const;
  /*********************************************************************
   * compute covariance matrix and update patch to corresponding keypoint
   * @param[in]  iprevPt      keypoint position
   * @param[in]  inter_param  bilinear interpolation parameter
   * @param[out] IWinBuf      patch after interpolation
   * @param[out] derivIWinBuf gradient patch after interpolation
   * @param[in]  A_ptr        start address of covariance matrix in patch
   */
  void compute_covariance_matrix_and_update_patch(
      const Point2i& iprevPt,
      const InterpolationParam& inter_param,
      cv::Mat* IWinBuf,
      cv::Mat* derivIWinBuf,
      float* const A_ptr) const;

  const Mat* prevImg;
  const Mat* nextImg;
  const Mat* prevDeriv;
  std::vector<XPKeyPoint>* m_prevPts;
  std::vector<Point2f>* m_nextPts;
  std::vector<bool>* m_status;
  std::vector<float>* m_err;

  Size winSize;
  TermCriteria criteria;
  int level;
  int maxLevel;
  int start_level;
  int flags;
  float minEigThreshold;
} XPTrackerInvoker;

/***********************************************************************
 * @param[in]  src     source image
 * @param[out] dst     target deriv image, 2 channels, x and y gradient
 *                     the storage of dst must be ready before calling
 *                     this function
 * [NOTE] this function will not verify any condition of dst in release mode.
 */
extern void calcSharrDeriv(const cv::Mat& src, cv::Mat* dst);
}  // namespace XP_OPTICAL_FLOW
}  // namespace XP
#endif  // XP_INCLUDE_XP_UTILXPPYRAMID_HPP_
