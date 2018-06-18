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
#include "plotting_utils.h"
#include <chrono>
#include <memory>
#include <algorithm>
#include <vector>
PoseDrawer2D::PoseDrawer2D() :
  scale_(1.0),
  min_x_(-0.5),
  min_y_(-0.5),
  max_x_(0.5),
  max_y_(0.5) {}
void PoseDrawer2D::addPose(float x, float y, float alpha) {
  std::lock_guard<std::mutex> lock(data_io_mutex_);
  latest_pose_.x = x;
  latest_pose_.y = y;
  latest_pose_.alpha = alpha;
  paths_.push_back(cv::Point2f(x, y));
  // maintain scaling
  constexpr float screen_margin = 1;  // m
  if (x - frameScale_ - screen_margin < min_x_)
    min_x_ = x - frameScale_ - screen_margin;
  if (y - frameScale_ - screen_margin < min_y_)
    min_y_ = y - frameScale_ - screen_margin;
  if (x + frameScale_ + screen_margin > max_x_)
    max_x_ = x + frameScale_ + screen_margin;
  if (y + frameScale_ + screen_margin> max_y_)
    max_y_ = y + frameScale_ + screen_margin;
  scale_ = std::min(imageSize / (max_x_ - min_x_), imageSize / (max_x_ - min_y_));
}
bool PoseDrawer2D::drawTo(cv::Mat* img_ptr) {
  if (img_ptr == nullptr) return false;
  cv::Mat& img = *img_ptr;
  // Clear image
  img.setTo(cv::Scalar(0, 0, 0));
  this->drawPath(&img);
  // draw axes
  std::lock_guard<std::mutex> lock(data_io_mutex_);
  const float e_x = sinf(latest_pose_.alpha);
  const float e_y = cosf(latest_pose_.alpha);
  // scale_mutex_ is locked in convertToImageCoordinates
  cv::line(img,
           convertToImageCoordinates(cv::Point2f(latest_pose_.x, latest_pose_.y)),
           convertToImageCoordinates(cv::Point2f(latest_pose_.x, latest_pose_.y)
                                     + cv::Point2f(e_x, e_y) * frameScale_),
           cv::Scalar(0, 0, 255), 1, CV_AA);
  return true;
}

cv::Point2f PoseDrawer2D::convertToImageCoordinates(const cv::Point2f & pointInMeters) {
  cv::Point2f pt = (pointInMeters - cv::Point2f(min_x_, min_y_)) * scale_;
  return cv::Point2f(pt.x, imageSize - pt.y);  // reverse y for more intuitive top-down plot
}
void PoseDrawer2D::drawPath(cv::Mat* img_ptr) {
  if (img_ptr == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(data_io_mutex_);
  if (paths_.empty()) {
    return;
  }
  auto it_path = paths_.begin();
  auto it_path_next = it_path;
  it_path_next++;
  for (; it_path_next !=paths_.end();) {
    cv::Point2f p0 = convertToImageCoordinates(*it_path);
    cv::Point2f p1 = convertToImageCoordinates(*it_path_next);
    cv::Point2f diff = p1-p0;
    if (diff.dot(diff) < 2.0) {
      auto it_path_bk = it_path_next;
      ++it_path_next;
      paths_.erase(it_path_bk);  // clean short segment
    } else {
      cv::line(*img_ptr, p0, p1, cv::Scalar(0, 255, 0), 1, CV_AA);
      ++it_path;
      ++it_path_next;
    }
  }
}
#ifdef HAS_OPENCV_VIZ
using cv::viz::Viz3d;
void viz3d_keyboard_cb(const cv::viz::KeyboardEvent& callback,
                       void* cookie) {
  if (callback.action == cv::viz::KeyboardEvent::Action::KEY_DOWN) {
    if (cookie != nullptr) {
      *(reinterpret_cast<uchar*>(cookie)) = callback.code;
    }
  }
}
/** \brief Initialize viz3d visualizer
 * Create a bunch of widgets
 * \param viz_window pointer of the Viz3d window
 */
PoseDrawer3D::PoseDrawer3D(float viz_cam_height,
                           const cv::Matx33f& cam_K) :
  img3d_{cv::viz::WImage3D(cv::Mat(1, 1, CV_8UC1), cv::Size2f(0.16, 0.12))},  // placeholder
  viz3d_text_(cv::viz::WText("xPerception", cv::Point2i(10, 10), 40)),
  viz3d_view_mode_(TOP_VIEW_FOLLOW),
  key_pressed_(0),
  viz_cam_height_(viz_cam_height) {
  cv::viz::WCameraPosition viz_cam(cam_K, 0.1);
  viz_window_ptr_.reset(new cv::viz::Viz3d("Tracking"));
  viz_window_ptr_->showWidget("cam mapper", viz_cam);
  cv::viz::WCoordinateSystem coord(0.5);
  viz_window_ptr_->showWidget("W system", coord);
  viz_window_ptr_->setViewerPose(
      cv::viz::makeCameraPose(cv::Vec3f(0, 0, 3),
                              cv::Vec3f(0, 0, 0),
                              cv::Vec3f(0, -1, 0)));
  // Show grid in world 3d
  cv::viz::WGrid grid(cv::Vec2i(20, 20), cv::Vec2i(1, 1), cv::viz::Color::white());
  viz_window_ptr_->showWidget("world grid", grid);
  // Show textvio/xp_ceres-solver/internal/xpceres/CMakeFiles/xpceres.dir/cxsparse.cc.o
  viz_window_ptr_->showWidget("text", viz3d_text_);
  // overlay images
  viz_window_ptr_->showWidget("img3d", img3d_);
  // register UI call back
  viz_window_ptr_->registerKeyboardCallback(viz3d_keyboard_cb, &key_pressed_);
  // Set full screen
  viz_window_ptr_->setFullScreen(true);
  viz_window_ptr_->setBackgroundColor(cv::viz::Color::bluberry());
}
void PoseDrawer3D::viz3d_once(const cv::Affine3f& W_T_D,
                              const cv::Mat& img,
                              const cv::Mat& rig_xyz_mat,
                              const cv::Mat_<cv::Vec3f>& depth_img) {
  // set the latest image as background
  if (viz3d_view_mode_ != CAM_FOLLOW) {
    if (!img.empty()) {
      img3d_.setImage(img);
    }
  } else {
    // o.w. img3d is not visible
  }
  // consume key pressed
  if (key_pressed_ == 'V' || key_pressed_ == 'v') {
    if (viz3d_view_mode_ == FREE_VIEW) {
      viz3d_view_mode_ = CAM_FOLLOW;
    } else if (viz3d_view_mode_ == TOP_VIEW_FOLLOW) {
      viz3d_view_mode_ = FREE_VIEW;
    } else if (viz3d_view_mode_ == CAM_FOLLOW) {
      viz3d_view_mode_ = TOP_VIEW_FOLLOW;
    }
    // reset
    key_pressed_ = 0;
  }

  // To compute viz3d_once frame rate
  // These var are static
  static std::chrono::time_point<std::chrono::steady_clock>
      frame_rate_last_time = std::chrono::steady_clock::now();
  static int frame_rate_counter = 0;
  static int frame_rate = 0;
  static int frame_counter = 0;
  // Get the latest camera pose
  // This is based on the latest IMU
  viz_window_ptr_->setWidgetPose("cam mapper", W_T_D);
  viz_window_ptr_->setWidgetPose("img3d", W_T_D);
  viz_window_ptr_->spinOnce(1, false);
  const cv::Vec3f cam_xyz(W_T_D.translation()[0],
                          W_T_D.translation()[1],
                          W_T_D.translation()[2]);
  // get mapper trajectory
  ++frame_counter;
  if (viz3d_view_mode_ != CAM_FOLLOW) {
    // re-draw traj every 10 frames (~0.5 sec)
    if (!rig_xyz_mat.empty()) {
      std::vector<cv::Affine3f> rig_xyz_affine3f(rig_xyz_mat.cols + 1);
      // always link the the latest pose
      rig_xyz_affine3f[0].translation(cam_xyz);
      for (size_t i = 0; i < rig_xyz_mat.cols; ++i) {
        rig_xyz_affine3f[i + 1].translation(rig_xyz_mat.at<cv::Vec3f>(i));
      }
      cv::viz::WTrajectory traj(rig_xyz_affine3f,
                                cv::viz::WTrajectory::PATH,
                                0.1, cv::viz::Color::red());
      viz_window_ptr_->showWidget("path", traj);
      viz_window_ptr_->spinOnce(1, false);
    }
  }
  // Compute frame rate
  ++frame_rate_counter;
  if (frame_rate_counter > 10) {
    const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - frame_rate_last_time).count();
    frame_rate_last_time = std::chrono::steady_clock::now();
    frame_rate = frame_rate_counter * 1000 / ms;
    frame_rate_counter = 0;
  }
  // Display frame rate info and cam pose
  // Do this every 3 frames
  if (frame_counter % 3 == 0) {
    // display frame rate info and cam pose
    std::stringstream cam_pose_ss;
    cam_pose_ss.setf(std::ios::fixed, std::ios::floatfield);
    cam_pose_ss.precision(3);
    cam_pose_ss << "xyz=" << W_T_D.translation().t();
    cam_pose_ss << " Viz Hz " << frame_rate;
    viz3d_text_.setText(cam_pose_ss.str());
  }
  // re-draw depth every 3 frames (different than text frame)
  if (frame_counter % 3 == 1) {
    if (!depth_img.empty()) {
      // compute color based on Z val
      cv::Mat_<cv::Vec3b> color(depth_img.size());
      constexpr float far_cut = 5.f;
      for (int x = 0; x < depth_img.cols; ++x) {
        for (int y = 0; y < depth_img.rows; ++y) {
          const float z_val = depth_img(y, x)[2];
          if (z_val > far_cut) {
            color(y, x) = cv::Vec3b(0, 0xff, 0);
          } else if (z_val < 0) {
            color(y, x) = cv::Vec3b(0, 0, 0xff);
          } else {
            color(y, x)[2] =
                static_cast<uint8_t>(255 * (far_cut - z_val) / far_cut);
            color(y, x)[1] =
                static_cast<uint8_t>(255 * z_val / far_cut);
            color(y, x)[0] = 0;
          }
        }
      }
      cv::viz::WCloud pnt_cloud(depth_img, color);
      viz_window_ptr_->showWidget("depth img", pnt_cloud, W_T_D);
      // A long spin time to make sure the pnt clound is displayed
      viz_window_ptr_->spinOnce(5, false);
    }
  }
  // Change view point based on viewing mode
  if (viz3d_view_mode_ == TOP_VIEW_FOLLOW) {
    // set new view point if the cam moves away
    const float cam_t_norm =
      sqrtf(W_T_D.translation()[0] * W_T_D.translation()[0] +
            W_T_D.translation()[1] * W_T_D.translation()[1] +
            W_T_D.translation()[2] * W_T_D.translation()[2]);
    if (2 * cam_t_norm > viz_cam_height_) {
      viz_cam_height_ *= 1.2;
    }
    viz_window_ptr_->setViewerPose(
        cv::viz::makeCameraPose(cam_xyz * 0.5 + cv::Vec3f(0, 0, viz_cam_height_ + 0.5),
                                cam_xyz * 0.5, cv::Vec3f(0, -1, 0)));
  } else if (viz3d_view_mode_ == CAM_FOLLOW) {
    // let the view camera follow the estimated pose
    viz_window_ptr_->setViewerPose(W_T_D);
  }
  // No need to wait, since main loop already includes waiting
  viz_window_ptr_->spinOnce(1, false);
}
uchar PoseDrawer3D::key_pressed() const {
  return key_pressed_;
}
#endif  // HAS_OPENCV_VIZ
