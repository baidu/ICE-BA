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

#include "rotation.h"
#include "xp_quaternion.h"
#include <glog/logging.h>

namespace XP {

// angle2dcm 'ZYX'
// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
void RotationMatrixToEulerRadians(const Eigen::Matrix3f &R, Eigen::Vector3f *euler_rad) {
  CHECK_NOTNULL(euler_rad);
  (*euler_rad)(0) = std::atan2(R(1, 2), R(2, 2));
  (*euler_rad)(1) =-std::atan2(R(0, 2), std::sqrt(1 - R(0, 2) * R(0, 2)));
  (*euler_rad)(2) = std::atan2(R(0, 1), R(0, 0));
}

// dcm2angle 'ZYX'
void EulerRadiansToRotationMatrix(const Eigen::Vector3f& euler_rad, Eigen::Matrix3f *R) {
  CHECK_NOTNULL(R);
  const float &phi   = euler_rad(0);
  const float &theta = euler_rad(1);
  const float &psi   = euler_rad(2);
  (*R)(0, 0) = cos(psi)*cos(theta);
  (*R)(0, 1) = sin(psi)*cos(theta);
  (*R)(0, 2) = -sin(theta);
  (*R)(1, 0) = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
  (*R)(1, 1) = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
  (*R)(1, 2) = cos(theta)*sin(phi);
  (*R)(2, 0) = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);
  (*R)(2, 1) = -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);
  (*R)(2, 2) = cos(theta)*cos(phi);
}

// Runge–Kutta 4th method for quaternion integration
void IntegrateQuaternion(const Eigen::Vector3f &omega_begin,
                         const Eigen::Vector3f &omega_end,
                         const Eigen::Quaternionf &q_begin,
                         const float dt,
                         Eigen::Quaternionf *q_end) {
  XpQuaternion xp_q_begin(q_begin);
  XpQuaternion xp_q_end;
  IntegrateQuaternion(omega_begin,
                      omega_end,
                      xp_q_begin,
                      dt,
                      &xp_q_end);
  *q_end = xp_q_end.ToEigenQuaternionf();
}

}  // namespace XP
