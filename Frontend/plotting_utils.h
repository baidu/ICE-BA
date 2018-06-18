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
/// \file
#ifndef __PLOTTING_UTILS_H__  // NOLINT
#define __PLOTTING_UTILS_H__
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#ifdef HAS_OPENCV_VIZ  // defined in CMakeLists
#include <opencv2/viz.hpp>
#endif
// Stl used in this sample
#include <list>
#include <vector>
#include <atomic>
#include <string>
#include <algorithm>
#include <mutex>
#include <memory>  // unique_ptr
class PoseDrawer2D {
 public:
  constexpr static const int imageSize = 480;  // the same as camera image height
  PoseDrawer2D();
  /**
   * Add a new pose for drawing
   */
  void addPose(float x, float y, float alpha);
  /** Drawing function called by external thread
   * Draw all the positions in history.
   */
  bool drawTo(cv::Mat* img_ptr);

 private:
  cv::Point2f convertToImageCoordinates(const cv::Point2f & pointInMeters);
  void drawPath(cv::Mat* img_ptr);
  std::mutex data_io_mutex_;
  std::list<cv::Point2f> paths_;
  struct {
    float x;
    float y;
    float alpha;
  } latest_pose_;
  std::atomic<float> scale_;
  std::atomic<float> min_x_;
  std::atomic<float> min_y_;
  std::atomic<float> max_x_;
  std::atomic<float> max_y_;
  const float frameScale_ = 0.2;  // the scale of the axis in plot[m]
};
#ifdef HAS_OPENCV_VIZ
class PoseDrawer3D {
 public:
  /** \brief Initialize viz3d visualizer
   *
   * Create a bunch of widgets
   * \param viz_window pointer of the Viz3d window
   */
  explicit PoseDrawer3D(float viz_cam_height,
                        const cv::Matx33f& cam_K);
  /** \brief Render 3d widgets once
   *
   * \param viz_window pointer of the Viz3d window
   * \param viz_cam_height the height of the observing camera.
   *                       The larger the scene, the bigger this value ideally is.
   *                       This value is re-computed by this function.
   */
  void viz3d_once(const cv::Affine3f& W_T_D,
                  const cv::Mat& img = cv::Mat(),
                  const cv::Mat& rig_xyz_mat = cv::Mat(),
                  const cv::Mat_<cv::Vec3f>& depth_result_img = cv::Mat_<cv::Vec3f>());
  uchar key_pressed() const;

 protected:
  std::shared_ptr<cv::viz::Viz3d> viz_window_ptr_;
  cv::viz::WImage3D img3d_;
  cv::viz::WText viz3d_text_;
  enum {
    FREE_VIEW = 0,
    TOP_VIEW_FOLLOW = 1,
    CAM_FOLLOW = 2,
  } viz3d_view_mode_;
  uchar key_pressed_;
  float viz_cam_height_;
};
#endif  // HAS_OPENCV_VIZ
#endif  // __PLOTTING_UTILS_H__ // NOLINT
