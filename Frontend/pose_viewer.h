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
#ifndef XP_INCLUDE_XP_HELPER_POSE_VIEWER_H_
#define XP_INCLUDE_XP_HELPER_POSE_VIEWER_H_

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <atomic>
#include <mutex>
#include <list>
#include <vector>
#include <deque>
namespace XP {
class PoseViewer {
 public:
  typedef Eigen::Matrix<float, 9, 1> SpeedAndBias;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  constexpr static const int imageSize = 400;
  PoseViewer();  // this we can register as a callback
  void set_clear_canvas_before_draw(bool clear_canvas_before_draw);
  void addPose(const Eigen::Matrix4f & T_WS,
               const Eigen::Matrix<float, 9, 1> & speedAndBiases,
               const float travel_dist);
  void addPose(const Eigen::Matrix4f & T_WS);
  void addPose(const Eigen::Matrix4f & T_WS, int path_id);
  void displayTo(const std::string& win_name);
  bool drawTo(cv::Mat* img);

 private:
  cv::Point2f convertToImageCoordinates(const cv::Point2f & pointInMeters);
  void drawPath(cv::Mat* img);
  bool clear_canvas_before_draw_;
  cv::Mat _image;  // for display to
  struct XYH {
    cv::Point2f xy;
    float h;
  };
  std::deque<std::mutex> data_io_mutices_;  // vector<mutex> doesn't work
  std::vector<std::list<XYH>> paths_;
  std::vector<Eigen::Matrix4f> latest_T_WS_;
  std::mutex scale_mutex_;
  SpeedAndBias last_speedAndBiases_;
  float last_travel_dist_ = 0.f;
  std::atomic<float> _scale;
  std::atomic<float> _min_x;
  std::atomic<float> _min_y;
  std::atomic<float> _min_z;
  std::atomic<float> _max_x;
  std::atomic<float> _max_y;
  std::atomic<float> _max_z;
  const float _frameScale = 0.2;  // the scale of the axis in plot[m]
  std::atomic_bool pose_changed_after_last_display_;
};
}  // namespace XP
#endif  // XP_INCLUDE_XP_HELPER_POSE_VIEWER_H_
