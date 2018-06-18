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

#ifndef XP_INCLUDE_XP_MATH_XP_QUATERNION_H_
#define XP_INCLUDE_XP_MATH_XP_QUATERNION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace XP {

class XpQuaternion : public Eigen::Vector4f {
  // "Indirect Kalman Filter for 3D Attitude Estimation"
  // x y z w
 public:
  // REQUIRED BY EIGEN
  typedef Eigen::Vector4f Base;

  // constructors
  // XpQuaternion is Hamilton quaternion in the order of xyzw
  // Eigen::Quaternion is also Hamiton quaternion in the order of wxyz
  XpQuaternion();
  template <typename Scalar>
  XpQuaternion(Scalar q0, Scalar q1, Scalar q2, Scalar q3)
      : Base{static_cast<float>(q0), static_cast<float>(q1),
        static_cast<float>(q2), static_cast<float>(q3)} {
    Eigen::Vector4f& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
  }

  template <typename OtherDerived>
  explicit XpQuaternion(
      const Eigen::QuaternionBase<OtherDerived> &eigen_quat) {
    SetFromEigenQuaternionf(eigen_quat);
  }

  // from MatrixBase - REQUIRED BY EIGEN
  template <typename OtherDerived>
  XpQuaternion(const MatrixBase<OtherDerived> &other)  // NOLINT
    : Base{other} {
    Eigen::Vector4f& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
  }

  // assignment operator - REQUIRED BY EIGEN
  template <typename OtherDerived>
  XpQuaternion &operator=(const MatrixBase<OtherDerived> &other) {
    Base::operator=(other);
    Eigen::Vector4f& q = *this;
    q = q * (1.f / q.norm());
    if (q(3) < 0) {
      q = -q;
    }
    return *this;
  }

  inline float x() const { return (*this)[0]; }
  inline float y() const { return (*this)[1]; }
  inline float z() const { return (*this)[2]; }
  inline float w() const { return (*this)[3]; }

  // For more details on conversion to and from Hamiltonian quaternions
  // see ref [2] and [3]
  Eigen::Quaternionf ToEigenQuaternionf() const;

  Eigen::Matrix3f ToRotationMatrix() const;

  Eigen::Vector3f ToEulerRadians() const;

  // set from Eigen Quaternion
  template <typename OtherDerived>
  void SetFromEigenQuaternionf(
      const Eigen::QuaternionBase<OtherDerived> &eigen_quat) {
    (*this)(0) = eigen_quat.x();
    (*this)(1) = eigen_quat.y();
    (*this)(2) = eigen_quat.z();
    (*this)(3) = eigen_quat.w();
    // keep scalar value non-negative
    if ((*this)(3) < 0) {
      (*this) = -(*this);
    }
  }

  // set from RotationMatrix.
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/
  // matrixToQuaternion/
  // Normalize the quaternion
  void SetFromRotationMatrix(const Eigen::Matrix3f &R);

  void SetFromEulerRadians(const Eigen::Vector3f& euler_rad);

  // set from delta theta
  // see page 8 on ref
  // Not const & because dtheta is changed inside the function
  void SetFromDeltaTheta(Eigen::Vector3f dtheta);

  // v_lhs = R(q) * v_rhs
  void SetFromTwoVectors(const Eigen::Vector3f& v_rhs, const Eigen::Vector3f& v_lhs);

  // Hides Eigen::MatrixBase<>::inverse
  XpQuaternion inverse() const;
  XpQuaternion mul(const XpQuaternion& rhs) const;
};

inline Eigen::Matrix4f XpComposeOmega(const Eigen::Vector3f& w) {
  Eigen::Matrix4f Ohm;
  Ohm(0, 0) = 0;     Ohm(0, 1) = -w(2);  Ohm(0, 2) = w(1);   Ohm(0, 3) = w(0);
  Ohm(1, 0) = w(2);  Ohm(1, 1) = 0;      Ohm(1, 2) = -w(0);  Ohm(1, 3) = w(1);
  Ohm(2, 0) = -w(1); Ohm(2, 1) = w(0);   Ohm(2, 2) = 0;      Ohm(2, 3) = w(2);
  Ohm(3, 0) = -w(0); Ohm(3, 1) = -w(1);  Ohm(3, 2) = -w(2);  Ohm(3, 3) = 0;
  return Ohm;
}

// page 11
// Note we use Vector4f to represent quaternion instead of quaternion
// because it is better do not normalize q during integration
inline Eigen::Vector4f XpQuaternionDerivative(
    const Eigen::Vector4f &q,
    const Eigen::Vector3f &omega) {
  return -0.5 * XpComposeOmega(omega) * q;
}

XpQuaternion quat_multiply(const XpQuaternion& lhs, const XpQuaternion& rhs);

// page 11
void IntegrateQuaternion(const Eigen::Vector3f &omega_begin,
                         const Eigen::Vector3f &omega_end,
                         const XpQuaternion &q_begin,
                         const float dt,
                         XpQuaternion *q_end,
                         XpQuaternion *q_mid = nullptr,
                         XpQuaternion *q_fourth = nullptr,
                         XpQuaternion *q_eighth = nullptr);

}  // namespace XP
#endif  // XP_INCLUDE_XP_MATH_XP_QUATERNION_H_
