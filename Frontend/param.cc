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
#include "param.h"
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <fstream>
#include <list>
#include <algorithm>

using std::vector;
using Eigen::Matrix4f;
namespace YAML {
// Define templated helper functions for YAML to encode/decode from custom data type.
// We need special care for emitters of Vector and Matrix!
template<>
struct convert<Eigen::Vector3f> {
  static Node encode(const Eigen::Vector3f& rhs) {
    Node node;
    node.push_back(rhs(0));
    node.push_back(rhs(1));
    node.push_back(rhs(2));
    return node;
  }

  static bool decode(const Node& node, Eigen::Vector3f& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }
    rhs(0) = node[0].as<float>();
    rhs(1) = node[1].as<float>();
    rhs(2) = node[2].as<float>();
    return true;
  }
};

Emitter& operator << (Emitter& out, const Eigen::Vector3f& v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v(0) << v(1) << v(2) << YAML::EndSeq;
  return out;
}
template<>
struct convert<cv::Size> {
  static Node encode(const cv::Size& rhs) {
    Node node;
    node.push_back(rhs.width);
    node.push_back(rhs.height);
    return node;
  }

  static bool decode(const Node& node, cv::Size& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.width = node[0].as<int>();
    rhs.height = node[1].as<int>();
    return true;
  }
};
Emitter& operator << (Emitter& out, const cv::Size& v) {
  out << YAML::Flow;
  out << YAML::BeginSeq << v.width << v.height << YAML::EndSeq;
  return out;
}
template<int M, int N>
struct convert<Eigen::Matrix<float, M, N>> {
// Encode Matrix3f as three sequences of 3-element sequence (row-major)
static Node encode(const Eigen::Matrix<float, M, N>& rhs) {
  Node node;
  for (int r = 0; r < M; r++) {
    Node row;
    for (int c = 0; c < N; c++) {
      row.push_back(rhs(r, c));
    }
    node.push_back(row);
  }
  return node;
}

static bool decode(const Node& node, Eigen::Matrix<float, M, N>& rhs) {  // NOLINT
  if (!node.IsSequence() || node.size() != M) {
    return false;
  }
  for (int r = 0; r < M; r++) {
    if (!node[r].IsSequence() || node[r].size() != N) {
      return false;
    }
    for (int c = 0 ; c < N; c++) {
      rhs(r, c) = node[r][c].as<float>();
    }
  }
  return true;
}
};
template<int M, int N>
Emitter& operator << (Emitter& out, const Eigen::Matrix<float, M, N>& v) {
  out << YAML::BeginSeq;
  for (int r = 0; r < M; r++) {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int c = 0; c < N; c++) {
      out << v(r, c);
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq;
  return out;
}
template<int M, int N>
struct convert<cv::Matx<float, M, N>> {
  static Node encode(const cv::Matx<float, M, N>& rhs) {
    Node node;
    for (int r = 0; r < M; r++) {
      Node row;
      for (int c = 0; c < N; c++) {
        row.push_back(rhs(r, c));
      }
      node.push_back(row);
    }
    return node;
  }
  static bool decode(const Node& node, cv::Matx<float, M, N>& rhs) {  // NOLINT
    if (!node.IsSequence() || node.size() != M) {
      return false;
    }
    for (int r = 0; r < M; r++) {
      if (!node[r].IsSequence()) {
        return false;
      }
      if (node[r].size() != N) {
        return false;
      }
      for (int c = 0 ; c < N; c++) {
        rhs(r, c) = node[r][c].as<float>();
      }
    }
    return true;
  }
};
template<>
struct convert<cv::Mat_<float>> {
  static Node encode(const cv::Mat_<float>& rhs) {
    Node node;
    for (int r = 0; r < rhs.rows; r++) {
      Node row;
      for (int c = 0; c < rhs.cols; c++) {
        row.push_back(rhs(r, c));
      }
      node.push_back(row);
    }
    return node;
  }
  static bool decode(const Node& node, cv::Mat_<float>& rhs) {  // NOLINT
    if (!node.IsSequence()) {
      return false;
    }
    const int M = node.size();
    if (!node[0].IsSequence()) {
      return false;
    }
    const int N = node[0].size();
    rhs.create(M, N);
    for (int r = 0; r < M; r++) {
      if (!node[r].IsSequence()) {
        return false;
      }
      if (node[r].size() != N) {
        return false;
      }
      for (int c = 0 ; c < N; c++) {
        rhs(r, c) = node[r][c].as<float>();
      }
    }
    return true;
  }
};

template<int M, int N>
Emitter& operator << (Emitter& out, const cv::Matx<float, M, N>& v) {
  out << YAML::BeginSeq;
  for (int r = 0; r < M; r++) {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int c = 0; c < N; c++) {
      out << v(r, c);
    }
    out << YAML::EndSeq;
  }
  out << YAML::EndSeq;
  return out;
}

template<>
struct convert<XP::AlgorithmParam::FeatDetParam_t> {
  static Node encode(const XP::AlgorithmParam::FeatDetParam_t& rhs) {
    Node node;
    node["request_feat_num"] = rhs.request_feat_num;
    node["pyra_level"] = rhs.pyra_level;
    node["uniform_radius"] = rhs.uniform_radius;
    node["fast_det_thresh"] = rhs.fast_det_thresh;
    node["min_feature_distance_over_baseline_ratio"] = rhs.min_feature_distance_over_baseline_ratio;
    node["max_feature_distance_over_baseline_ratio"] = rhs.max_feature_distance_over_baseline_ratio;
    node["feature_track_length_thresh"] = rhs.feature_track_length_thresh;
    node["feature_track_dropout_rate"] = rhs.feature_track_dropout_rate;
    return node;
  }
  static bool decode(const Node& node, XP::AlgorithmParam::FeatDetParam_t& rhs) {  // NOLINT
    if (!node.IsMap()) {
      return false;
    }
    rhs.request_feat_num = node["request_feat_num"].as<int>();
    rhs.pyra_level = node["pyra_level"].as<int>();
    rhs.uniform_radius = node["uniform_radius"].as<int>();
    rhs.fast_det_thresh = node["fast_det_thresh"].as<int>();
    rhs.min_feature_distance_over_baseline_ratio =
        node["min_feature_distance_over_baseline_ratio"].as<float>();
    rhs.max_feature_distance_over_baseline_ratio =
        node["max_feature_distance_over_baseline_ratio"].as<float>();
    rhs.feature_track_length_thresh = node["feature_track_length_thresh"].as<int>();
    rhs.feature_track_dropout_rate = node["feature_track_dropout_rate"].as<float>();
    return true;
  }
};

Emitter& operator << (Emitter& out, const XP::AlgorithmParam::FeatDetParam_t& v) {
  Node node;
  node = v;
  out << node;
  return out;
}
template<>
struct convert<XP::AlgorithmParam::Tracking_t> {
  static Node encode(const XP::AlgorithmParam::Tracking_t& rhs) {
    Node node;
    node["feature_uncertainty"] = rhs.feature_uncertainty;
    node["use_of_id"] = rhs.use_of_id;
    node["slave_det"] = rhs.slave_det;
    node["undistort_before_vio"] = rhs.undistort_before_vio;
    return node;
  }

  static bool decode(const Node& node, XP::AlgorithmParam::Tracking_t& rhs) {  // NOLINT
    if (!node.IsMap()) {
      return false;
    }
    rhs.feature_uncertainty = node["feature_uncertainty"].as<float>();
    rhs.use_of_id = node["use_of_id"].as<bool>();
    rhs.slave_det = node["slave_det"].as<std::string>();
    if (rhs.slave_det != "epi" && rhs.slave_det != "OF" && rhs.slave_det != "direct") {
      LOG(ERROR) << "slave_det mode " << rhs.slave_det << " not supported";
      return false;
    }
    rhs.undistort_before_vio = node["undistort_before_vio"].as<bool>();
    if (rhs.undistort_before_vio && !rhs.use_of_id) {
      // if we feed undistorted pnts to vio, we have to let vio use the OF id
      LOG(ERROR) << "undistort_before_vio && !use_of_id";
      return false;
    }
    return true;
  }
};

Emitter& operator << (Emitter& out, const XP::AlgorithmParam::Tracking_t& v) {
  Node node;
  node = v;
  out << node;
  return out;
}


template<>
struct convert<XP::DuoCalibParam::Imu_t> {
  static Node encode(const XP::DuoCalibParam::Imu_t& rhs) {
    Node node;
    node["accel_TK"] = rhs.accel_TK;
    node["accel_bias"] = rhs.accel_bias;
    node["gyro_TK"] = rhs.gyro_TK;
    node["gyro_bias"] = rhs.gyro_bias;
    node["accel_noise_var"] = rhs.accel_noise_var;
    node["angv_noise_var"] = rhs.angv_noise_var;
    node["D_T_I"] = rhs.D_T_I;
    return node;
  }
  static bool decode(const Node& node, XP::DuoCalibParam::Imu_t& rhs) {  // NOLINT
    if (!node.IsMap()) {
      return false;
    }
    rhs.accel_TK = node["accel_TK"].as<Eigen::Matrix3f>();
    rhs.accel_bias = node["accel_bias"].as<Eigen::Vector3f>();
    rhs.gyro_TK = node["gyro_TK"].as<Eigen::Matrix3f>();
    rhs.gyro_bias = node["gyro_bias"].as<Eigen::Vector3f>();
    rhs.accel_noise_var = node["accel_noise_var"].as<Eigen::Vector3f>();
    rhs.angv_noise_var = node["angv_noise_var"].as<Eigen::Vector3f>();
    if (node["D_T_I"]) {
      rhs.D_T_I = node["D_T_I"].as<Eigen::Matrix4f>();
    } else {
      VLOG(1) << "Set D_T_I as default val (I)";
      rhs.D_T_I = Eigen::Matrix4f::Identity();
    }
    return true;
  }
};

// Imu_t need special care because of the formatting of Matrix and Vector
Emitter& operator << (Emitter& out, const XP::DuoCalibParam::Imu_t& v) {
  out << YAML::BeginMap;
  out << YAML::Key << "accel_TK"
      << YAML::Value << v.accel_TK;
  out << YAML::Key << "accel_bias"
      << YAML::Value << v.accel_bias;
  out << YAML::Key << "gyro_TK"
      << YAML::Value << v.gyro_TK;
  out << YAML::Key << "gyro_bias"
      << YAML::Value << v.gyro_bias;
  out << YAML::Key << "accel_noise_var"
      << YAML::Value << v.accel_noise_var;
  out << YAML::Key << "angv_noise_var"
      << YAML::Value << v.angv_noise_var;
  out << YAML::Key << "D_T_I"
      << YAML::Value << v.D_T_I;
  out << YAML::EndMap;
  return out;
}

template<>
struct convert<XP::DuoCalibParam::Camera_t> {
  static Node encode(const XP::DuoCalibParam::Camera_t& rhs) {
    Node node;
    node["D_T_C_l"] = rhs.D_T_C_lr[0];
    node["D_T_C_r"] = rhs.D_T_C_lr[1];
    node["cameraK_l"] = rhs.cameraK_lr[0];
    node["cameraK_r"] = rhs.cameraK_lr[1];
    node["dist_coeff_l"] = rhs.cv_dist_coeff_lr[0];
    node["dist_coeff_r"] = rhs.cv_dist_coeff_lr[1];
    node["img_size"] = rhs.img_size;
    return node;
  }

  static bool decode(const Node& node, XP::DuoCalibParam::Camera_t& rhs) {  // NOLINT
    if (!node.IsMap()) {
      return false;
    }
    rhs.D_T_C_lr.resize(2);
    rhs.D_T_C_lr[0] = node["D_T_C_l"].as<Eigen::Matrix4f>();
    rhs.D_T_C_lr[1] = node["D_T_C_r"].as<Eigen::Matrix4f>();
    rhs.cameraK_lr.resize(2);
    rhs.cameraK_lr[0] = node["cameraK_l"].as<Eigen::Matrix3f>();
    rhs.cameraK_lr[1] = node["cameraK_r"].as<Eigen::Matrix3f>();
    rhs.cv_camK_lr.resize(2);
    rhs.cv_camK_lr[0] = node["cameraK_l"].as<cv::Matx33f>();
    rhs.cv_camK_lr[1] = node["cameraK_r"].as<cv::Matx33f>();
    rhs.cv_dist_coeff_lr.resize(2);
    cv::Matx<float, 8, 1> dist_l =
        node["dist_coeff_l"].as<cv::Matx<float, 8, 1>>();
    if (dist_l(7) < 1e-7 && dist_l(7) > -1e-7 &&
        dist_l(6) < 1e-7 && dist_l(6) > -1e-7 &&
        dist_l(5) < 1e-7 && dist_l(5) > -1e-7) {
      if (dist_l(4) < 1e-7 && dist_l(4) > -1e-7) {
        rhs.cv_dist_coeff_lr[0].create(4, 1);
      } else {
        rhs.cv_dist_coeff_lr[0].create(5, 1);
      }
    } else {
      rhs.cv_dist_coeff_lr[0].create(8, 1);
    }
    for (int i = 0; i < rhs.cv_dist_coeff_lr[0].rows; ++i) {
      rhs.cv_dist_coeff_lr[0](i) = dist_l(i);
    }
    cv::Matx<float, 8, 1> dist_r =
        node["dist_coeff_r"].as<cv::Matx<float, 8, 1>>();
    if (dist_r(7) < 1e-7 && dist_r(7) > -1e-7 &&
        dist_r(6) < 1e-7 && dist_r(6) > -1e-7 &&
        dist_r(5) < 1e-7 && dist_r(5) > -1e-7) {
      if (dist_r(4) < 1e-7 && dist_r(4) > -1e-7) {
        rhs.cv_dist_coeff_lr[1].create(4, 1);
      } else {
        rhs.cv_dist_coeff_lr[1].create(5, 1);
      }
    } else {
      rhs.cv_dist_coeff_lr[1].create(8, 1);
    }
    for (int i = 0; i < rhs.cv_dist_coeff_lr[1].rows; ++i) {
      rhs.cv_dist_coeff_lr[1](i) = dist_r(i);
    }
    if (node["img_size"]) {
      rhs.img_size = node["img_size"].as<cv::Size>();
    } else {
      rhs.img_size.width = 752;  // default param
      rhs.img_size.height = 480;  // default param
      std::cout << "no img_size is found in calib file. Using default value "
                << rhs.img_size << std::endl;
    }
    return true;
  }
};
// Imu_t need special care because of the formatting of Matrix and Vector
Emitter& operator << (Emitter& out, const XP::DuoCalibParam::Camera_t& v) {
  out << YAML::BeginMap;
  out << YAML::Key << "D_T_C_l"
      << YAML::Value << v.D_T_C_lr[0];
  out << YAML::Key << "D_T_C_r"
      << YAML::Value << v.D_T_C_lr[1];
  out << YAML::Key << "cameraK_l" << YAML::Value << v.cameraK_lr[0];
  out << YAML::Key << "cameraK_r" << YAML::Value << v.cameraK_lr[1];
  cv::Matx<float, 8, 1> dist_l = cv::Matx<float, 8, 1>::zeros();
  CHECK_EQ(v.cv_dist_coeff_lr.size(), 2);
  // CHECK_LE(v.cv_dist_coeff_lr[0].rows, dist_l.rows);
  for (int i = 0; i < std::min(8, v.cv_dist_coeff_lr[0].rows); ++i) {
    dist_l(i) = v.cv_dist_coeff_lr[0](i);
  }
  out << YAML::Key << "dist_coeff_l" << YAML::Value << dist_l;
  cv::Matx<float, 8, 1> dist_r = cv::Matx<float, 8, 1>::zeros();
  // CHECK_LE(v.cv_dist_coeff_lr[0].rows, dist_r.rows);
  for (int i = 0; i < std::min(8, v.cv_dist_coeff_lr[1].rows); ++i) {
    dist_r(i) = v.cv_dist_coeff_lr[1](i);
  }
  out << YAML::Key << "dist_coeff_r" << YAML::Value << dist_r;
  out << YAML::Key << "img_size" << YAML::Value << v.img_size;
  out << YAML::EndMap;
  return out;
}

}  // namespace YAML


namespace XP {

YAML::Node ParamBase::deserialize(const std::string& filename) {
  std::ifstream in(filename);
  CHECK(in.is_open()) << "Cannot load file " << filename;
  in.close();
  YAML::Node node = YAML::LoadFile(filename);
  return node;
}

void ParamBase::serialize(const std::string& filename, const YAML::Emitter& emitter) {
  std::ofstream fout(filename);
  fout << emitter.c_str();
}

bool AlgorithmParam::LoadFromYaml(const std::string& filename) {
  YAML::Node node = deserialize(filename);
  FeatDetParam = node["FeatDetParam"].as<FeatDetParam_t>();
  Tracking = node["Tracking"].as<Tracking_t>();
  return true;
}

bool AlgorithmParam::LoadFromCvYaml(const std::string& filename) {
  LOG(FATAL) << "Not Implemented.";
  return false;
}

bool AlgorithmParam::WriteToYaml(const std::string& filename) {
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "FeatDetParam"
          << YAML::Value << FeatDetParam;
  emitter << YAML::Key << "Tracking"
          << YAML::Value << Tracking;
  emitter << YAML::EndMap;

  serialize(filename, emitter);
  return true;
}

bool AlgorithmParam::WriteToCvYaml(const std::string& filename) {
  LOG(FATAL) << "Not Implemented.";
  return false;
}

DuoCalibParam::DuoCalibParam() :
    C_R_B(Eigen::Matrix3f::Identity()),
    C_p_B(Eigen::Vector3f::Zero()),
    device_id("n/a"),
    sensor_type(SensorType::UNKNOWN) {
  this->Camera.D_T_C_lr.resize(2, Eigen::Matrix4f::Identity());
  this->Camera.cameraK_lr.resize(2, Eigen::Matrix3f::Identity());
  this->Camera.cv_camK_lr.resize(2, cv::Matx33f::eye());
  this->Camera.lurd_lr.resize(2, Eigen::Vector4f::Zero());
  this->Camera.cv_dist_coeff_lr.resize(2);
  this->Imu.accel_TK = Eigen::Matrix3f::Identity();
  this->Imu.accel_bias.setZero();
  this->Imu.gyro_TK = Eigen::Matrix3f::Identity();
  this->Imu.gyro_bias.setZero();
  this->Imu.accel_noise_var.setConstant(0.04 * 0.04);
  this->Imu.angv_noise_var.setConstant(0.01 * 0.01);
  this->Imu.D_T_I = Eigen::Matrix4f::Identity();;
}

bool DuoCalibParam::LoadFromCvYaml(const std::string& filename) {
  cv::FileStorage cvfs(filename, cv::FileStorage::READ);
  if (!cvfs.isOpened()) {
    LOG(ERROR) << "Can't open file: " << filename;
    return false;
  }
  // loading device id
  cvfs["DeviceId"] >> device_id;
  // loading sensor type
  std::string str_sensor_type = cvfs["SensorType"];
  if (str_sensor_type == "UNKNOWN") {
    sensor_type = XP::DuoCalibParam::SensorType::UNKNOWN;
  } else if (str_sensor_type == "LI") {
    sensor_type = XP::DuoCalibParam::SensorType::LI;
  } else if (str_sensor_type == "XP") {
    sensor_type = XP::DuoCalibParam::SensorType::XP;
  } else if (str_sensor_type == "XP2") {
    sensor_type = XP::DuoCalibParam::SensorType::XP2;
  } else if (str_sensor_type == "XP3") {
    sensor_type = XP::DuoCalibParam::SensorType::XP3;
  } else {
    LOG(ERROR) << "Unknown sensor type";
    cvfs.release();
    return false;
  }

  // IMU
  cv::Mat accel_TK;
  cvfs["accel_TK"] >> accel_TK;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Imu.accel_TK(i, j) = accel_TK.at<double>(i, j);
    }
  }

  cv::Mat accel_bias;
  cvfs["accel_bias"] >> accel_bias;
  for (int i = 0; i < 3; ++i) {
    Imu.accel_bias(i, 0) = accel_bias.at<double>(i, 0);
  }

  cv::Mat gyro_TK;
  cvfs["gyro_TK"] >> gyro_TK;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Imu.gyro_TK(i, j) = gyro_TK.at<double>(i, j);
    }
  }

  cv::Mat gyro_bias;
  cvfs["gyro_bias"] >> gyro_bias;
  for (int i = 0; i < 3; ++i) {
    Imu.gyro_bias(i, 0) = gyro_bias.at<double>(i, 0);
  }

  cv::Mat accel_noise_var;
  cvfs["accel_noise_var"] >> accel_noise_var;
  for (int i = 0; i < 3; ++i) {
    Imu.accel_noise_var(i, 0) = accel_noise_var.at<double>(i, 0);
  }

  cv::Mat angv_noise_var;
  cvfs["angv_noise_var"] >> angv_noise_var;
  for (int i = 0; i < 3; ++i) {
    Imu.angv_noise_var(i, 0) = angv_noise_var.at<double>(i, 0);
  }

  cv::Mat D_T_I;
  cvfs["D_T_I"] >> D_T_I;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Imu.D_T_I(i, j) = D_T_I.at<double>(i, j);
    }
  }

  // CAMERA
  cv::Mat d_t_c[2];
  cvfs["D_T_C_l"] >> d_t_c[0];
  cvfs["D_T_C_r"] >> d_t_c[1];
  // COLUMN MAJOR ORDER
  Eigen::Matrix4f& dtcl = Camera.D_T_C_lr[0];
  Eigen::Matrix4f& dtcr = Camera.D_T_C_lr[1];
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      dtcl(i, j) = d_t_c[0].at<double>(i, j);
      dtcr(i, j) = d_t_c[1].at<double>(i, j);
    }
  }

  cv::Mat cameraK[2];
  cvfs["cameraK_l"] >> cameraK[0];
  cvfs["cameraK_r"] >> cameraK[1];
  // COLUMN MAJOR ORDER
  Eigen::Matrix3f& camera_k_l = Camera.cameraK_lr[0];
  Eigen::Matrix3f& camera_k_r = Camera.cameraK_lr[1];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      camera_k_l(i, j) = cameraK[0].at<double>(i, j);
      camera_k_r(i, j) = cameraK[1].at<double>(i, j);
    }
  }

  cv::Mat dist_coeff[2];
  cvfs["dist_coeff_l"] >> dist_coeff[0];
  cvfs["dist_coeff_r"] >> dist_coeff[1];
  dist_coeff[0].convertTo(Camera.cv_dist_coeff_lr[0], CV_32FC1);
  dist_coeff[1].convertTo(Camera.cv_dist_coeff_lr[1], CV_32FC1);

  cvfs["img_size"] >> Camera.img_size;
  cvfs.release();
  return true;
}

bool DuoCalibParam::LoadFromYaml(const std::string& filename) {
  return (this->LoadImuCalibFromYaml(filename) &&
      this->LoadCamCalibFromYaml(filename));
}

bool DuoCalibParam::LoadImuCalibFromYaml(const std::string& filename) {
  YAML::Node node = deserialize(filename);
  if (!node["Imu"].IsDefined()) {
    return false;
  }
  this->Imu = node["Imu"].as<Imu_t>();
  return true;
}

bool DuoCalibParam::LoadCamCalibFromYaml(const std::string& filename) {
  YAML::Node node = deserialize(filename);
  this->device_id = "";
  if (node["DeviceId"]) {
    this->device_id = node["DeviceId"].as<std::string>();
  }
  this->sensor_type = SensorType::UNKNOWN;
  if (node["SensorType"]) {
    std::string type = node["SensorType"].as<std::string>();
    if (type == "LI") {
      this->sensor_type = SensorType::LI;
    } else if (type == "XP") {
      this->sensor_type = SensorType::XP;
    } else if (type == "XP2") {
      this->sensor_type = SensorType::XP2;
    } else if (type == "XP3") {
      this->sensor_type = SensorType::XP3;
    } else if (type == "UNKNOWN") {
      this->sensor_type = SensorType::UNKNOWN;
    } else {
      LOG(ERROR) << "sensor_type in yaml " << type << " is illegal";
      return false;
    }
  }
  if (!node["Camera"].IsDefined()) {
    return false;
  }
  this->Camera = node["Camera"].as<Camera_t>();
  // generate undistort map
  CHECK(this->initUndistortMap(this->Camera.img_size));
  return true;
}

bool DuoCalibParam::WriteToCvYaml(const std::string& filename) {
  cv::FileStorage cvfs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
  if (!cvfs.isOpened()) {
    LOG(ERROR) << "Can't open file: " << filename;
    return false;
  }
  // device id
  cvfs << "DeviceId" << device_id;
  // sensor type
  switch (sensor_type) {
    case XP::DuoCalibParam::SensorType::UNKNOWN:
      cvfs << "SensorType" << "UNKNOWN"; break;
    case XP::DuoCalibParam::SensorType::LI:
      cvfs << "SensorType" << "LI"; break;
    case XP::DuoCalibParam::SensorType::XP:
      cvfs << "SensorType" << "XP"; break;
    case XP::DuoCalibParam::SensorType::XP2:
      cvfs << "SensorType" << "XP2"; break;
    case XP::DuoCalibParam::SensorType::XP3:
      cvfs << "SensorType" << "XP3"; break;
    default:
      LOG(ERROR) << "sensor type invalid";
      return -1;
  }
  double param_buffer[4 * 4 * sizeof(double)];
  // IMU
  // accel_TK
  cv::Mat accel_TK(3, 3, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  float* tmp_ptr = Imu.accel_TK.data();
  for (int i = 0; i < 9; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "accel_TK" << accel_TK;

  // accel_bias
  cv::Mat accel_bias(3, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = Imu.accel_bias.data();
  for (int i = 0; i < 3; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "accel_bias" << accel_bias;

  // gyro_TK
  cv::Mat gyro_TK(3, 3, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = Imu.gyro_TK.data();
  for (int i = 0; i < 9; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "gyro_TK" << gyro_TK;

  // gyro_bias
  cv::Mat gyro_bias(3, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = Imu.gyro_bias.data();
  for (int i = 0; i < 3; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "gyro_bias" << gyro_bias;

  // accel_noise_var
  cv::Mat accel_noise_var(3, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = Imu.accel_noise_var.data();
  for (int i = 0; i < 3; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "accel_noise_var" << accel_noise_var;

  // angv_noise_var
  cv::Mat angv_noise_var(3, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = Imu.angv_noise_var.data();
  for (int i = 0; i < 3; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "angv_noise_var" << angv_noise_var;

  // D_T_I
  cv::Mat D_T_I(4, 4, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      param_buffer[i * 4 + j] = static_cast<double>(Imu.D_T_I(i, j));
    }
  }
  cvfs << "D_T_I" << D_T_I;

  // CAMERA
  cv::Mat d_t_c_l(4, 4, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      param_buffer[i * 4 + j] = static_cast<double>(Camera.D_T_C_lr[0](i, j));
    }
  }
  cvfs << "D_T_C_l" << d_t_c_l;

  cv::Mat d_t_c_r(4, 4, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      param_buffer[i * 4 + j] = static_cast<double>(Camera.D_T_C_lr[1](i, j));
    }
  }
  cvfs << "D_T_C_r" << d_t_c_r;

  cv::Mat camera_l(3, 3, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      param_buffer[i * 3 + j] = static_cast<double>(Camera.cameraK_lr[0](i, j));
    }
  }
  cvfs << "cameraK_l" << camera_l;

  cv::Mat camera_r(3, 3, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      param_buffer[i * 3 + j] = static_cast<double>(Camera.cameraK_lr[1](i, j));
    }
  }
  cvfs << "cameraK_r" << camera_r;

  cv::Mat dist_coeff_l(8, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = reinterpret_cast<float*>(Camera.cv_dist_coeff_lr[0].data);
  for (int i = 0; i < 8; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "dist_coeff_l" << dist_coeff_l;

  cv::Mat dist_coeff_r(8, 1, CV_64FC1, reinterpret_cast<void*>(param_buffer));
  tmp_ptr = reinterpret_cast<float*>(Camera.cv_dist_coeff_lr[1].data);
  for (int i = 0; i < 8; ++i) {
    param_buffer[i] = static_cast<double>(*tmp_ptr++);
  }
  cvfs << "dist_coeff_r" << dist_coeff_r;
  cvfs << "img_size" << Camera.img_size;
  cvfs.release();
  return true;
}

bool DuoCalibParam::WriteToYaml(const std::string& filename) {
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "DeviceId"
          << YAML::Value << this->device_id;
  if (this->sensor_type == SensorType::UNKNOWN) {
    emitter << YAML::Key << "SensorType"
            << YAML::Value << "UNKNOWN";
  } else if (this->sensor_type == SensorType::LI) {
    emitter << YAML::Key << "SensorType"
            << YAML::Value << "LI";
  } else if (this->sensor_type == SensorType::XP) {
    emitter << YAML::Key << "SensorType"
            << YAML::Value << "XP";
  } else if (this->sensor_type == SensorType::XP2) {
    emitter << YAML::Key << "SensorType"
            << YAML::Value << "XP2";
  } else if (this->sensor_type == SensorType::XP3) {
    emitter << YAML::Key << "SensorType"
            << YAML::Value << "XP3";
  } else {
    LOG(ERROR) << "this->sensor_type == "
               << this->sensor_type << " not supported";
    return false;
  }
  emitter << YAML::Key << "Imu"
          << YAML::Value << this->Imu;
  emitter << YAML::Key << "Camera"
          << YAML::Value << this->Camera;
  emitter << YAML::EndMap;
  serialize(filename, emitter);
  return true;
}

// init undistort map from camera K and distort
bool DuoCalibParam::initUndistortMap(const cv::Size& new_img_size) {
  // compute the boundary of images in uv coordinate
  this->Camera.lurd_lr.resize(2);
  for (int lr = 0; lr < 2; ++lr) {
    std::vector<cv::Point2f> lurd(4);
    lurd[0].x = 0;
    lurd[0].y = this->Camera.img_size.height / 2;
    lurd[1].x = this->Camera.img_size.width / 2;
    lurd[1].y = 0;
    lurd[2].x = this->Camera.img_size.width;
    lurd[2].y = this->Camera.img_size.height / 2;
    lurd[3].x = this->Camera.img_size.width / 2;
    lurd[3].y = this->Camera.img_size.height;
    std::vector<cv::Point2f> lurd_undistort(4);
    cv::undistortPoints(lurd, lurd_undistort,
                        this->Camera.cv_camK_lr[lr],
                        this->Camera.cv_dist_coeff_lr[lr]);
    this->Camera.lurd_lr[lr][0] = lurd_undistort[0].x;
    this->Camera.lurd_lr[lr][1] = lurd_undistort[1].y;
    this->Camera.lurd_lr[lr][2] = lurd_undistort[2].x;
    this->Camera.lurd_lr[lr][3] = lurd_undistort[3].y;
  }
  this->Camera.undistort_map_op1_lr.resize(2);
  this->Camera.undistort_map_op2_lr.resize(2);
  vector<cv::Mat> R(2);
  vector<cv::Mat> P(2);
  Matrix4f C0_T_C1 = this->Camera.D_T_C_lr[0].inverse() * this->Camera.D_T_C_lr[1];
  Matrix4f C1_T_C0 = C0_T_C1.inverse();
  cv::Mat C0_R_C1(3, 3, CV_64F);
  cv::Mat C1_R_C0(3, 3, CV_64F);
  cv::Mat C1_t_C0(3, 1, CV_64F);
  cv::Mat C0_t_C1(3, 1, CV_64F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C0_R_C1.at<double>(i, j) = C0_T_C1(i, j);
      C1_R_C0.at<double>(i, j) = C1_T_C0(i, j);
    }
    C0_t_C1.at<double>(i, 0) = C0_T_C1(i, 3);
    C1_t_C0.at<double>(i, 0) = C1_T_C0(i, 3);
  }
  cv::Matx44d Q;
  cv::stereoRectify(this->Camera.cv_camK_lr[0],
                    this->Camera.cv_dist_coeff_lr[0],
                    this->Camera.cv_camK_lr[1],
                    this->Camera.cv_dist_coeff_lr[1],
                    this->Camera.img_size,
                    C1_R_C0, C1_t_C0,
                    R[0], R[1],
                    P[0], P[1],
                    Q, cv::CALIB_ZERO_DISPARITY, 0, new_img_size);
  this->Camera.Q = cv::Matx44f(Q);
  this->Camera.cv_undist_K_lr.resize(2);
  this->Camera.undist_D_T_C_lr.resize(2);
  for (int lr = 0; lr < 2; lr++) {
    cv::Mat newK(3, 3, CV_32F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        // note P[lr] doesn't work
        newK.at<float>(i, j) = P[lr].at<double>(i, j);
      }
    }
    cv::initUndistortRectifyMap(this->Camera.cv_camK_lr[lr],
                                this->Camera.cv_dist_coeff_lr[lr],
                                R[lr],
                                newK,  // this->Camera.cv_camK_lr[lr],  // newK
                                new_img_size,
                                CV_32FC1,
                                this->Camera.undistort_map_op1_lr[lr],
                                this->Camera.undistort_map_op2_lr[lr]);
    this->Camera.cv_undist_K_lr[lr] = cv::Matx33f(newK);
    this->Camera.undist_D_T_C_lr[lr].setIdentity();
  }

  // Compute the new *rectified* extrinsics
  // In undist_D_T_C_lr, we assume D_T_C_l is identity, which means,
  // the device {D} is *moved* to the new C_l (R[0] is actually Cl_new_R_Cl)
  // Therefore, D_T_I needs to be adjusted as well to undist_D_T_I.
  cv::Mat new_C0_t_C1 = R[0] * C0_t_C1;
  this->Camera.undist_D_T_C_lr[1](0, 3) = new_C0_t_C1.at<double>(0);
  this->Camera.undist_D_T_C_lr[1](1, 3) = new_C0_t_C1.at<double>(1);
  this->Camera.undist_D_T_C_lr[1](2, 3) = new_C0_t_C1.at<double>(2);
  Eigen::Matrix4f new_D_T_D = Eigen::Matrix4f::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      new_D_T_D(i, j) = static_cast<float>(R[0].at<double>(i, j));
    }
  }
  this->Imu.undist_D_T_I = new_D_T_D * this->Imu.D_T_I;
  return true;
}
bool get_calib_file_from_device_id(const std::string& device_id,
                                   std::string* calib_file_ptr) {
  CHECK_NOTNULL(calib_file_ptr);
  // todo: auto-parse device_id from string...
  std::list<std::string> known_device_ids;
  known_device_ids.push_back(std::string("93312E057D22"));  // this one has an IR pass filter
  known_device_ids.push_back(std::string("8219C930E5FC"));
  known_device_ids.push_back(std::string("DFCDC2D10CD9"));
  known_device_ids.push_back(std::string("BAAFF337329A"));
  known_device_ids.push_back(std::string("BDC4FA571539"));
  known_device_ids.push_back(std::string("7E92FAF26363"));
  known_device_ids.push_back(std::string("8E1BC83F0421"));
  known_device_ids.push_back(std::string("4924E006C82A"));
  known_device_ids.push_back(std::string("054AC6A9044F"));
  known_device_ids.push_back(std::string("408C392FF83F"));
  known_device_ids.push_back(std::string("505FC7C0497A"));
  known_device_ids.push_back(std::string("A72F4DA6A92D"));
  known_device_ids.push_back(std::string("FC329CAD406E"));
  known_device_ids.push_back(std::string("XP0000001"));
  char* p_path;
  p_path = getenv("MASTER_DIR");
  CHECK_NOTNULL(p_path);
  std::string dir_path(p_path);
  dir_path += "/calib_params/";
  for (const auto& known_device_id : known_device_ids) {
    if (device_id.find(known_device_id) != std::string::npos) {
      *calib_file_ptr = dir_path + known_device_id + ".calib.yaml";
      return true;
    }
  }
  return false;
}

bool load_imu_calib_param(const std::string& device_id,
                          DuoCalibParam* duo_calib_param_ptr) {
  CHECK_NOTNULL(duo_calib_param_ptr);
  std::string calib_file;
  if (!get_calib_file_from_device_id(device_id, &calib_file)) {
    LOG(ERROR) << "unknown device id: " << device_id;
    return false;
  }
  return duo_calib_param_ptr->LoadFromYaml(calib_file);
}
bool load_camera_calib_param(const std::string& calib_file,
                             DuoCalibParam* duo_calib_param_ptr) {
  cv::FileStorage calib_fs(calib_file, cv::FileStorage::READ);
  if (!calib_fs.isOpened()) {
    return false;
  }
  CHECK_NOTNULL(duo_calib_param_ptr);
  XP::DuoCalibParam& duo_calib_param = *duo_calib_param_ptr;
  cv::Mat K_l, K_r, dist_l, dist_r, R_l, t_l;
  calib_fs["K_l"] >> K_l;
  calib_fs["K_r"] >> K_r;
  CHECK_EQ(K_l.type(), CV_32F);
  CHECK_EQ(K_r.type(), CV_32F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      duo_calib_param.Camera.cameraK_lr[0](i, j) = K_l.at<float>(i, j);
      duo_calib_param.Camera.cameraK_lr[1](i, j) = K_r.at<float>(i, j);
      duo_calib_param.Camera.cv_camK_lr[0](i, j) = K_l.at<float>(i, j);
      duo_calib_param.Camera.cv_camK_lr[1](i, j) = K_r.at<float>(i, j);
    }
  }
  calib_fs["dist_l"] >> dist_l;
  calib_fs["dist_r"] >> dist_r;
  CHECK_EQ(dist_l.type(), CV_32F);
  CHECK_EQ(dist_l.type(), CV_32F);
  duo_calib_param.Camera.cv_dist_coeff_lr[0].create(dist_l.rows, 1);
  for (int i = 0; i < dist_l.rows; i++) {
    duo_calib_param.Camera.cv_dist_coeff_lr[0](i) = dist_l.at<float>(i);
  }
  duo_calib_param.Camera.cv_dist_coeff_lr[1].create(dist_r.rows, 1);
  for (int i = 0; i < dist_r.rows; i++) {
    duo_calib_param.Camera.cv_dist_coeff_lr[1](i) = dist_r.at<float>(i);
  }
  cv::Mat Cl_T_Cr;
  calib_fs["Cl_T_Cr"] >> Cl_T_Cr;
  if (Cl_T_Cr.rows != 0) {
    CHECK_EQ(Cl_T_Cr.type(), CV_32F);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        duo_calib_param.Camera.D_T_C_lr[1](i, j) = Cl_T_Cr.at<float>(i, j);
      }
    }
  } else {
    // calib file is saved by reading duo mlx
    calib_fs["R_l"] >> R_l;
    calib_fs["t_l"] >> t_l;
    CHECK_EQ(R_l.type(), CV_32F);
    CHECK_EQ(R_l.rows, 3);
    CHECK_EQ(R_l.cols, 3);
    CHECK_EQ(t_l.type(), CV_32F);
    // This is needed by 1.0.50 driver
    // For new log recorded by 1.0.80 driver, Rl and tl is no longer used
    R_l = R_l.t();
    Eigen::Matrix4f Cr_T_D = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        Cr_T_D(i, j) = R_l.at<float>(i, j);
      }
      Cr_T_D(i, 3) = t_l.at<float>(i, 0) / 1e3;
    }
    duo_calib_param.Camera.D_T_C_lr[1] = Cr_T_D.inverse();
  }
  CHECK_NEAR(duo_calib_param.Camera.D_T_C_lr[1](3, 0), 0, 1e-5);
  CHECK_NEAR(duo_calib_param.Camera.D_T_C_lr[1](3, 1), 0, 1e-5);
  CHECK_NEAR(duo_calib_param.Camera.D_T_C_lr[1](3, 2), 0, 1e-5);
  CHECK_NEAR(duo_calib_param.Camera.D_T_C_lr[1](3, 3), 1, 1e-5);
  VLOG(1) << "duo_calib_param.Camera.D_T_C_lr[1] " << duo_calib_param.Camera.D_T_C_lr[1];
  // load size
  if (calib_fs["img_size"].isNone()) {
    duo_calib_param.Camera.img_size = cv::Size(752, 480);  // default param
    std::cout << "No img size info is found. Using default "
              << duo_calib_param.Camera.img_size << std::endl;
  } else {
    calib_fs["img_size"] >> duo_calib_param.Camera.img_size;
  }
  // generate undistort map
  CHECK(duo_calib_param.initUndistortMap(duo_calib_param.Camera.img_size));
  return true;
}
bool save_camera_calib_param(const std::string& calib_file,
                             const DuoCalibParam& duo_calib_param) {
  cv::FileStorage calib_fs(calib_file, cv::FileStorage::WRITE);
  if (!calib_fs.isOpened()) {
    return false;
  }
  cv::Mat K_l, K_r, dist_l, dist_r, R_l, t_l;
  calib_fs << "K_l" << cv::Mat(duo_calib_param.Camera.cv_camK_lr[0]);
  calib_fs << "dist_l" << cv::Mat(duo_calib_param.Camera.cv_dist_coeff_lr[0]);
  calib_fs << "K_r" << cv::Mat(duo_calib_param.Camera.cv_camK_lr[1]);
  calib_fs << "dist_r" << cv::Mat(duo_calib_param.Camera.cv_dist_coeff_lr[0]);
  cv::Mat Cl_T_Cr(4, 4, CV_32F);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      Cl_T_Cr.at<float>(i, j) = duo_calib_param.Camera.D_T_C_lr[1](i, j);
    }
  }
  calib_fs << "Cl_T_Cr" << Cl_T_Cr;
  calib_fs << "img_size" << duo_calib_param.Camera.img_size;
  return true;
}

}  // namespace XP
