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
#include "pose_viewer.h"
#include <glog/logging.h>
using std::vector;
using Eigen::Matrix4f;
namespace XP {
PoseViewer::PoseViewer() :
  clear_canvas_before_draw_(false),
  _scale(1.0),
  _min_x(-0.5),
  _min_y(-0.5),
  _min_z(-0.5),
  _max_x(0.5),
  _max_y(0.5),
  _max_z(0.5),
  pose_changed_after_last_display_(true) {
  constexpr int kMaxNumPaths = 10;  // in case there are a lot of paths
  paths_.resize(kMaxNumPaths);
  data_io_mutices_.resize(kMaxNumPaths);
  latest_T_WS_.resize(kMaxNumPaths, Matrix4f::Identity());
  // we will always see a cross in the center of the canvas which are unused  latest_T_WS_
  _image.create(imageSize, imageSize, CV_8UC3);
}
void PoseViewer::set_clear_canvas_before_draw(bool clear_canvas_before_draw) {
  clear_canvas_before_draw_ = clear_canvas_before_draw;
}
void PoseViewer::addPose(const Matrix4f & T_WS,
                         const SpeedAndBias & speedAndBiases,
                         const float travel_dist) {
  {
    std::lock_guard<std::mutex> lock(data_io_mutices_[0]);
    last_speedAndBiases_ = speedAndBiases;
    last_travel_dist_ = travel_dist;
  }
  this->addPose(T_WS);
}
void PoseViewer::addPose(const Matrix4f & T_WS) {
  // 0 is default path_id
  this->addPose(T_WS, 0);
}

void PoseViewer::addPose(const Eigen::Matrix4f & T_WS, int path_id) {
  // just append the path
  Eigen::Vector3f r = T_WS.topRightCorner<3, 1>();
  {
    std::lock_guard<std::mutex> lock(data_io_mutices_[path_id]);
    latest_T_WS_[path_id] = T_WS;
    XYH xyh;
    xyh.xy = cv::Point2f(r[0], r[1]);
    xyh.h = r[2];
    paths_[path_id].push_back(xyh);
  }
  std::lock_guard<std::mutex> lock(scale_mutex_);
  // maintain scaling
  constexpr float screen_margin = 1;  // m
  if (r[0] - _frameScale - screen_margin < _min_x)
    _min_x = r[0] - _frameScale - screen_margin;
  if (r[1] - _frameScale - screen_margin < _min_y)
    _min_y = r[1] - _frameScale - screen_margin;
  if (r[2] < _min_z)
    _min_z = r[2];
  if (r[0] + _frameScale + screen_margin > _max_x)
    _max_x = r[0] + _frameScale + screen_margin;
  if (r[1] + _frameScale + screen_margin> _max_y)
    _max_y = r[1] + _frameScale + screen_margin;
  if (r[2] > _max_z)
    _max_z = r[2];
  _scale = std::min(imageSize / (_max_x - _min_x), imageSize / (_max_y - _min_y));
  pose_changed_after_last_display_ = true;
}

void PoseViewer::displayTo(const std::string& win_name) {
  if (pose_changed_after_last_display_) {
    this->drawTo(&_image);
    pose_changed_after_last_display_ = false;
  }
  cv::imshow(win_name, _image);
}
bool PoseViewer::drawTo(cv::Mat* img_ptr) {
  CHECK_NOTNULL(img_ptr);
  CHECK_EQ(img_ptr->type(), _image.type());
  CHECK_LE(imageSize, img_ptr->rows);
  CHECK_LE(imageSize, img_ptr->cols);
  cv::Mat& img = *img_ptr;
  // erase
  if (clear_canvas_before_draw_) {
    img.setTo(cv::Scalar(0, 0, 0));
  }
  this->drawPath(&img);
  // draw axes
  for (int it_cam = 0; it_cam < latest_T_WS_.size(); ++it_cam) {
    std::lock_guard<std::mutex> lock(data_io_mutices_[it_cam]);
    Eigen::Matrix3f R = latest_T_WS_[it_cam].topLeftCorner<3, 3>();
    Eigen::Vector3f e_x = R.col(0);
    Eigen::Vector3f e_y = R.col(1);
    Eigen::Vector3f e_z = R.col(2);
    cv::Point2f cam_xy(latest_T_WS_[it_cam](0, 3), latest_T_WS_[it_cam](1, 3));
    // scale_mutex_ is locked in convertToImageCoordinates
    cv::line(img,
             convertToImageCoordinates(cam_xy),
             convertToImageCoordinates(cam_xy + cv::Point2f(e_x[0], e_x[1]) * _frameScale),
             cv::Scalar(0, 0, 255), 1, CV_AA);
    cv::line(img,
             convertToImageCoordinates(cam_xy),
             convertToImageCoordinates(cam_xy + cv::Point2f(e_y[0], e_y[1]) * _frameScale),
             cv::Scalar(0, 255, 0), 1, CV_AA);
    cv::line(img,
             convertToImageCoordinates(cam_xy),
             convertToImageCoordinates(cam_xy + cv::Point2f(e_z[0], e_z[1]) * _frameScale),
             cv::Scalar(255, 0, 0), 1, CV_AA);
  }

  // Display text for trajectory 0
  {
    SpeedAndBias speedAndBiases = last_speedAndBiases_;  // cache it in case of change
    // some text:
    std::stringstream postext;
    postext.setf(std::ios::fixed, std::ios::floatfield);
    postext.precision(3);
    {
      std::lock_guard<std::mutex> lock(data_io_mutices_[0]);
      postext << "position = [" << latest_T_WS_[0](0, 3)
              << ", " << latest_T_WS_[0](1, 3)
              << ", " << latest_T_WS_[0](2, 3) << "]";
    }
    cv::putText(img, postext.str(), cv::Point(15, 15),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    std::stringstream veltext;
    veltext.setf(std::ios::fixed, std::ios::floatfield);
    veltext.precision(3);
    veltext << "velocity = [" << speedAndBiases[0] << ", "
            << speedAndBiases[1] << ", " << speedAndBiases[2] << "]";
    cv::putText(img, veltext.str(), cv::Point(15, 35),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    std::stringstream bgtext;
    bgtext.setf(std::ios::fixed, std::ios::floatfield);
    bgtext.precision(4);
    bgtext << "Bg = [" << speedAndBiases[3] << ", " << speedAndBiases[4] << ", "
           << speedAndBiases[5] << "]";
    cv::putText(img, bgtext.str(), cv::Point(15, 55),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    std::stringstream batext;
    batext.setf(std::ios::fixed, std::ios::floatfield);
    batext.precision(4);
    batext << "Ba = [" << speedAndBiases[6] << ", "
           << speedAndBiases[7] << ", " << speedAndBiases[8] << "]";
    cv::putText(img, batext.str(), cv::Point(15, 75),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    std::stringstream disttext;
    disttext.setf(std::ios::fixed, std::ios::floatfield);
    disttext.precision(2);
    disttext << "travel dist = " << last_travel_dist_;
    cv::putText(img, disttext.str(), cv::Point(15, 95),
                cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  }

  // Display text for trajectory 2 if exists
  {
    const int k = 2;
    std::lock_guard<std::mutex> lock(data_io_mutices_[k]);
    if (!paths_[k].empty()) {
      std::stringstream postext;
      postext.setf(std::ios::fixed, std::ios::floatfield);
      postext.precision(3);
      postext << "iba pos  = [" << latest_T_WS_[k](0, 3)
              << ", " << latest_T_WS_[k](1, 3)
              << ", " << latest_T_WS_[k](2, 3) << "]";
      cv::putText(img, postext.str(), cv::Point(15, 115),
                  cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
  }
  return true;
}
cv::Point2f PoseViewer::convertToImageCoordinates(const cv::Point2f & pointInMeters) {
  std::lock_guard<std::mutex> lock(scale_mutex_);
  cv::Point2f pt = (pointInMeters - cv::Point2f(_min_x, _min_y)) * _scale;
  return cv::Point2f(pt.x, imageSize - pt.y);  // reverse y for more intuitive top-down plot
}
void PoseViewer::drawPath(cv::Mat* img_ptr) {
  // this is an internal call.
  // assume img_ptr has been validated
  cv::Mat img = *img_ptr;
  bool show_h_in_color = true;
  if (!paths_[1].empty()) {
    // if there are more than 1 path, show path color to distinguish path id
    show_h_in_color = false;
  }
  for (int path_id = 0; path_id < paths_.size(); ++path_id) {
    std::lock_guard<std::mutex> lock(data_io_mutices_[path_id]);
    if (paths_[path_id].empty()) {
      continue;
    }
    auto it_path = paths_[path_id].begin();
    auto it_path_next = it_path;
    it_path_next++;
    for (; it_path_next != paths_[path_id].end();) {
      cv::Point2f p0 = convertToImageCoordinates(it_path->xy);
      cv::Point2f p1 = convertToImageCoordinates(it_path_next->xy);
      cv::Point2f diff = p1-p0;
      if (diff.dot(diff) < 2.0) {
        auto it_path_bk = it_path_next;
        ++it_path_next;
        paths_[path_id].erase(it_path_bk);  // clean short segment
      } else {
        cv::Scalar color;
        if (show_h_in_color) {
          float rel_height = (it_path->h - _min_z + it_path_next->h - _min_z)
          * 0.5 / (_max_z - _min_z);
          color = rel_height * cv::Scalar(0, 255, 0) + (1.0 - rel_height) * cv::Scalar(0, 0, 255);
        } else {
          uchar B = path_id * 50;
          uchar G = (1 - path_id) * 200;
          uchar R = path_id * 200;
          color = cv::Scalar(B, G, R);
        }
        cv::line(img, p0, p1, color, 1, CV_AA);
        ++it_path;
        ++it_path_next;
      }
    }
  }
}

}  // namespace XP
