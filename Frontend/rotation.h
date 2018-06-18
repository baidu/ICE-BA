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

#ifndef XP_INCLUDE_XP_MATH_ROTATION_H_
#define XP_INCLUDE_XP_MATH_ROTATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace XP {

/*
 * Common rotation-related utility functions
 */
// angle2dcm 'ZYX'
// https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
void RotationMatrixToEulerRadians(const Eigen::Matrix3f &R, Eigen::Vector3f *euler_rad);

// dcm2angle 'ZYX'
void EulerRadiansToRotationMatrix(const Eigen::Vector3f& euler_rad, Eigen::Matrix3f *R);

// Runge–Kutta 4th method for quaternion integration
// q_begin and q_end should be in the notation of L_q_W, where L is local coordinate,
// and W is world coordinate.  R(L_q_W) will be the top left 3x3 corner of T_LW, which
// is the transformation from W to L: p_L = T_LW * p_W
void IntegrateQuaternion(const Eigen::Vector3f &omega_begin,
                         const Eigen::Vector3f &omega_end,
                         const Eigen::Quaternionf &q_begin,
                         const float dt,
                         Eigen::Quaternionf *q_end);

}  // namespace XP
#endif  // XP_INCLUDE_XP_MATH_ROTATION_H_
