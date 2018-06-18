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
#include "iba_helper.h"
#include <IBA_config.h>  // for CFG_STEREO
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../Geometry/Rotation.h"

namespace XP {
IBA::Intrinsic to_iba_intrinsic(const DuoCalibParam::Camera_t& cam, const int lr) {
  IBA::Intrinsic K;
  K.fx = cam.cameraK_lr[lr](0, 0);
  K.fy = cam.cameraK_lr[lr](1, 1);
  K.cx = cam.cameraK_lr[lr](0, 2);
  K.cy = cam.cameraK_lr[lr](1, 2);
  for (int i = 0; i < 8; ++i) {
    if (i < cam.cv_dist_coeff_lr[lr].rows) {
      K.ds[i] = cam.cv_dist_coeff_lr[lr].at<float>(i);
    } else {
      K.ds[i] = 0.f;
    }
  }
  return K;
}

IBA::Calibration to_iba_calibration(const DuoCalibParam& calib) {
  IBA::Calibration iba_calib;
  iba_calib.w = calib.Camera.img_size.width;
  iba_calib.h = calib.Camera.img_size.height;
  iba_calib.fishEye = false;
  Eigen::Matrix4f Cl_T_I = calib.Camera.D_T_C_lr[0].inverse() * calib.Imu.D_T_I;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      iba_calib.Tu[i][j] = Cl_T_I(i, j);
    }
  }
  Rotation3D R;
  R.Set(iba_calib.Tu[0], iba_calib.Tu[1], iba_calib.Tu[2]);
  R.MakeOrthogonal();
  R.Get(iba_calib.Tu[0], iba_calib.Tu[1], iba_calib.Tu[2]);
  iba_calib.K = to_iba_intrinsic(calib.Camera, 0);

#ifdef CFG_STEREO
  Eigen::Matrix4f Cl_T_Cr = calib.Camera.D_T_C_lr[0].inverse() * calib.Camera.D_T_C_lr[1];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      iba_calib.Tr[i][j] = Cl_T_Cr(i, j);
    }
  }
  R.Set(iba_calib.Tr[0], iba_calib.Tr[1], iba_calib.Tr[2]);
  R.MakeOrthogonal();
  R.Get(iba_calib.Tr[0], iba_calib.Tr[1], iba_calib.Tr[2]);
  iba_calib.Kr = to_iba_intrinsic(calib.Camera, 1);
#endif

  for (int i = 0; i < 3; ++i) {
    iba_calib.ba[i] = calib.Imu.accel_bias(i);
    iba_calib.bw[i] = calib.Imu.gyro_bias(i);
  }
  return iba_calib;
}

IBA::IMUMeasurement to_iba_imu(const ImuData& imu_in) {
  IBA::IMUMeasurement imu_out;
  imu_out.t = imu_in.time_stamp;
  for (size_t i = 0; i < 3; ++i) {
    imu_out.a[i] = imu_in.accel(i);
    imu_out.w[i] = imu_in.ang_v(i);
  }
  return imu_out;
}


/*

IBA::CameraPose vio_to_iba_pose(const Eigen::Matrix4f& T) {
  IBA::CameraPose pose;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      pose.R[i][j] = T(i, j);
    }
    pose.p[i] = T(i, 3);
  }
  return pose;
}



IBA::CameraIMUState vio_to_iba_state(const Eigen::Matrix4f& W_T_C,
                                     const VIO::CalibParam& calib) {
  // TODO(mingyu): Need to save ground-truth velocity from synthetic data
  IBA::CameraIMUState state;
  for (int i = 0; i < 3; ++i) {
    // Check IBA_datatype.h for the CameraPose notation:
    // R is R_CW, p is the camera position in W
    state.C.p[i] = W_T_C(i, 3);
    for (int j = 0; j < 3; ++j) {
      state.C.R[i][j] = W_T_C(j, i);  // Transpose here!
    }
    state.v[i] = 0.f;  // WRONG velocity for now
    state.ba[i] = calib.Imu.accel_bias(i);
    state.bw[i] = calib.Imu.gyro_bias(i);
  }
  return state;
}
*/

}  // namespace XP
