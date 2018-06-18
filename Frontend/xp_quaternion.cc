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
#include <iostream>
namespace XP {

// Quaternion multiplication
XpQuaternion quat_multiply(const XpQuaternion& lhs,
                           const XpQuaternion& rhs) {
  XpQuaternion result;
  result << lhs(0) * rhs(3) + lhs(1) * rhs(2) - lhs(2) * rhs(1) + lhs(3) * rhs(0),
           -lhs(0) * rhs(2) + lhs(1) * rhs(3) + lhs(2) * rhs(0) + lhs(3) * rhs(1),
            lhs(0) * rhs(1) - lhs(1) * rhs(0) + lhs(2) * rhs(3) + lhs(3) * rhs(2),
           -lhs(0) * rhs(0) - lhs(1) * rhs(1) - lhs(2) * rhs(2) + lhs(3) * rhs(3);
  result.normalize();

  // keep scalar value non-negative
  if (result(3) < 0) {
    result = -result;
  }
  return result;
}

/*
 * XpQuaternion
 */
XpQuaternion::XpQuaternion() {
  *this = Eigen::Vector4f(0, 0, 0, 1);
}

// Hamiltonian representaiton
Eigen::Quaternionf XpQuaternion::ToEigenQuaternionf() const {
  return Eigen::Quaternionf{(*this)(3),
                            (*this)(0),
                            (*this)(1),
                            (*this)(2)};
}

Eigen::Matrix3f XpQuaternion::ToRotationMatrix() const {
  const XpQuaternion &q = *this;
  Eigen::Matrix3f R;
  R << 1 - 2 * (q(1)*q(1) + q(2)*q(2)),
       2 * (q(0)*q(1) - q(2)*q(3)),
       2 * (q(0)*q(2) + q(1)*q(3)),
       2 * (q(0)*q(1) + q(2)*q(3)),
       1 - 2 * (q(0)*q(0) + q(2)*q(2)),
       2 * (q(1)*q(2) - q(0)*q(3)),
       2 * (q(0)*q(2) - q(1)*q(3)),
       2 * (q(1)*q(2) + q(0)*q(3)),
       1 - 2 * (q(0)*q(0) + q(1)*q(1));
  return R;
}

Eigen::Vector3f XpQuaternion::ToEulerRadians() const {
  // pitch roll yaw
  Eigen::Vector3f euler_rad;
  RotationMatrixToEulerRadians(this->ToRotationMatrix(), &euler_rad);
  return euler_rad;
}

// set from delta theta during update
// see page 8 on ref[1]
void XpQuaternion::SetFromDeltaTheta(Eigen::Vector3f dtheta) {
  (*this).head<3>() = 0.5 * dtheta;
  (*this)(3) = std::sqrt(1 - (*this).head<3>().squaredNorm());
}

void XpQuaternion::SetFromRotationMatrix(const Eigen::Matrix3f &R) {
  float trace = R(0, 0) + R(1, 1) + R(2, 2);
  if ( trace > 0 ) {
    float s = 2.0 * std::sqrt(trace + 1.0);
    (*this)(0) = ( R(2, 1) - R(1, 2) ) / s;
    (*this)(1) = ( R(0, 2) - R(2, 0) ) / s;
    (*this)(2) = ( R(1, 0) - R(0, 1) ) / s;
    (*this)(3) = 0.25 * s;
  } else {
    // find the biggest diagonal element
    if ( R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2) ) {
      float s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
      (*this)(0) = 0.25 * s;
      (*this)(1) = (R(1, 0) + R(0, 1)) / s;
      (*this)(2) = (R(2, 0) + R(0, 2)) / s;
      (*this)(3) = (R(2, 1) - R(1, 2)) / s;
    } else if (R(1, 1) >= R(0, 0) && R(1, 1) >= R(2, 2)) {
      float s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
      (*this)(0) = (R(1, 0) + R(0, 1)) / s;
      (*this)(1) = 0.25 * s;
      (*this)(2) = (R(2, 1) + R(1, 2)) / s;
      (*this)(3) = (R(0, 2) - R(2, 0)) / s;
    } else {  // in this case, R(2,2) is the biggest element
      float s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
      (*this)(0) = (R(2, 0) + R(0, 2)) / s;
      (*this)(1) = (R(2, 1) + R(1, 2)) / s;
      (*this)(2) = 0.25 * s;
      (*this)(3) = (R(1, 0) - R(0, 1)) / s;
    }
  }
  // keep scalar value non-negative
  if ((*this)(3) < 0) {
    (*this) = -(*this);
  }
}

void XpQuaternion::SetFromEulerRadians(const Eigen::Vector3f &euler_rad) {
  Eigen::Matrix3f R;
  EulerRadiansToRotationMatrix(euler_rad, &R);
  SetFromRotationMatrix(R);
}

// v_lhs = R(q) * v_rhs
void XpQuaternion::SetFromTwoVectors(
    const Eigen::Vector3f& v_rhs, const Eigen::Vector3f& v_lhs) {
  Eigen::Vector3f v_rhs_unit = v_rhs / v_rhs.norm();
  Eigen::Vector3f v_lhs_unit = v_lhs / v_lhs.norm();
  this->topRows<3>() = v_lhs_unit.cross(v_rhs_unit);
  this->topRows<3>().normalize();
  float cos_theta = v_rhs_unit.dot(v_lhs_unit);
  float theta = -std::acos(cos_theta);
  (*this)[3] = std::cos(theta / 2);
  this->topRows<3>() = this->topRows<3>() * std::sin(theta / 2);
  // keep scalar value non-negative
  if ((*this)(3) < 0) {
    (*this) = -(*this);
  }
}

// Hides Eigen::MatrixBase<>::inverse
XpQuaternion XpQuaternion::inverse() const {
  return XpQuaternion{-(*this)(0), -(*this)(1), -(*this)(2), (*this)(3)};
}
XpQuaternion XpQuaternion::mul(const XpQuaternion& rhs) const {
  return quat_multiply(*this, rhs);
}

void IntegrateQuaternion(
    const Eigen::Vector3f &omega_begin, const Eigen::Vector3f &omega_end,
    const XpQuaternion &q_begin, const float dt,
    XpQuaternion *q_end,
    XpQuaternion *q_mid,
    XpQuaternion *q_fourth,
    XpQuaternion *q_eighth) {
  CHECK_NOTNULL(q_end);
  XpQuaternion q_0 = q_begin;

  // divide dt time interval into num_segment segments
  // TODO(mingyu): Reduce to 2 or 4 segments as 8 segments may overkill
  const int num_segment = 8;

  // the time duration in each segment
  const float inv_num_segment = 1.0 / num_segment;
  const float dt_seg = dt * inv_num_segment;

  Eigen::Vector3f delta_omega = omega_end - omega_begin;
  for (int i = 0; i < num_segment; ++i) {
    // integrate in the region: [i/num_segment, (i+1)/num_segment]

    Eigen::Vector3f omega_tmp = omega_begin + (i * inv_num_segment) * delta_omega;
    Eigen::Vector4f k1 = XpQuaternionDerivative(q_0, omega_tmp);
    omega_tmp = omega_begin + 0.5 * (2 * i + 1) * inv_num_segment * delta_omega;
    Eigen::Vector4f k2 = XpQuaternionDerivative(q_0 + 0.5 * k1 * dt_seg, omega_tmp);
    Eigen::Vector4f k3 = XpQuaternionDerivative(q_0 + 0.5 * k2 * dt_seg, omega_tmp);
    omega_tmp = omega_begin + (i + 1) * inv_num_segment * delta_omega;
    Eigen::Vector4f k4 = XpQuaternionDerivative(q_0 + k3 * dt_seg, omega_tmp);
    (*q_end) = q_0 + (dt_seg / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);

    // store the start point the next region of integration
    q_0 = (*q_end);

    // store the intermediate attitude as output
    if (i == 0 && q_eighth) {
      (*q_eighth) = (*q_end);
    } else if (i == 1 && q_fourth) {
      (*q_fourth) = (*q_end);
    } else if (i == 3 && q_mid) {
      (*q_mid) = (*q_end);
    }
  }
}
}  // namespace XP
