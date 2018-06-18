/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jul 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/RadialTangentialDistortion.hpp
 * @brief Header implementation file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */

#include <Eigen/LU>
#include <iostream>

/// \brief vio Main namespace of this package.
namespace vio {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

// This constant defines the valid region of the distortion model
// R = 2,    atan2(R, 1) is 63.4 deg, FOV is 126.8 deg
// R = 8.14, atan2(R, 1) is 83.0 deg, FOV is 166.0 deg
// TODO(mingyu): Should automatically compute R_squared instead of hardcoded
// constexpr float R_squared = 2 * 2;
constexpr float R_squared = 8.14 * 8.14;

// The default constructor with all zero ki
RadialTangentialDistortion8::RadialTangentialDistortion8()
    : k1_(0.0),
      k2_(0.0),
      p1_(0.0),
      p2_(0.0),
      k3_(0.0),
      k4_(0.0),
      k5_(0.0),
      k6_(0.0) {
  parameters_.setZero();
}

// Constructor initialising ki
RadialTangentialDistortion8::RadialTangentialDistortion8(float k1, float k2,
                                                         float p1, float p2,
                                                         float k3, float k4,
                                                         float k5, float k6) {
  parameters_[0] = k1;
  parameters_[1] = k2;
  parameters_[2] = p1;
  parameters_[3] = p2;
  parameters_[4] = k3;
  parameters_[5] = k4;
  parameters_[6] = k5;
  parameters_[7] = k6;
  k1_ = k1;
  k2_ = k2;
  p1_ = p1;
  p2_ = p2;
  k3_ = k3;
  k4_ = k4;
  k5_ = k5;
  k6_ = k6;
}
bool RadialTangentialDistortion8::setParameters(
    const Eigen::VectorXd & parameters) {
  if (parameters.cols() != NumDistortionIntrinsics) {
    return false;
  }
  parameters_ = parameters.cast<float>();
  k1_ = parameters[0];
  k2_ = parameters[1];
  p1_ = parameters[2];
  p2_ = parameters[3];
  k3_ = parameters[4];
  k4_ = parameters[5];
  k5_ = parameters[6];
  k6_ = parameters[7];
  return true;
}

#ifdef __ARM_NEON__
/***********************************************************************************
* [In]:    pointUndistorted -> the coordinates of point undistorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   pointDistorted -> the coordinates of point distorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   status, 0 if distorted successfully, or 0xffffffff
* NO RETURNS
*/
void RadialTangentialDistortion8::distortT4f(
    const float* pointUndistorted, float* pointDistorted, unsigned int status[4]) const {
  float p1 = 2.f * p1_, p2 = 2.f * p2_;
  float32x4_t u = vld1q_f32(pointUndistorted);  // u = (u0, u1, u2, u3)
  float32x4_t v = vld1q_f32(pointUndistorted + 4);  // v = (v0, v1, v2, v3)
  float32x4_t vp1 = vmovq_n_f32(p1), vp2 = vmovq_n_f32(p2);
  float32x4_t mx = vmulq_f32(u, u);     // mx = u * u
  float32x4_t my = vmulq_f32(v, v);     // my = v * v
  float32x4_t mxy = vmulq_f32(u, v);    // mxy = u * v
  const float32x4_t rho = vaddq_f32(mx, my);  // rho = mx + my
  // rho > r_squared, ret = all ones for true, or all zeros
  const uint32x4_t ret = vcgtq_f32(rho, vmovq_n_f32(R_squared));

  const float32x4_t rho_power2 = vmulq_f32(rho, rho);  // rho_power2 = rho * rho
  float32x4_t tmp = vld1q_dup_f32(&k5_);
  float32x4_t f00 = vmlaq_n_f32(tmp, rho, k6_);  // f00 = vk5 + rho * k6_;
  tmp = vld1q_dup_f32(&k2_);
  float32x4_t f10 = vmlaq_n_f32(tmp, rho, k3_);  // f10 = vk2 + rho * k6_;
  tmp = vmovq_n_f32(1.f);  // tmp = (1.f, 1.f, 1.f, 1.f)
  const float32x4_t f01 = vmlaq_n_f32(tmp, rho, k4_);  // f01 = 1.f + rho * k4_;
  const float32x4_t f11 = vmlaq_n_f32(tmp, rho, k1_);  // f11 = 1.f + rho * k1_
  float32x4_t m0 = vmulq_n_f32(rho, p2_);
  float32x4_t m1 = vmulq_n_f32(rho, p1_);

  f00 = vmlaq_f32(f01, f00, rho_power2);
  f10 = vmlaq_f32(f11, f10, rho_power2);

  m0 = vaddq_f32(m0, vmulq_f32(mx, vp2));
  m1 = vaddq_f32(m1, vmulq_f32(my, vp1));

  // TODO(hongtian): handle the corner case of divided by 0 (f00 = 0)
  float32x4_t rad_dist_u = vrecpeq_f32(f00);
  rad_dist_u = vmulq_f32(vrecpsq_f32(f00, rad_dist_u), rad_dist_u);  // rad_dist_u = 1.f / f00
  rad_dist_u = vmulq_f32(vrecpsq_f32(f00, rad_dist_u), rad_dist_u);
  rad_dist_u = vmulq_f32(rad_dist_u, f10);  // rad_dist_u = f10 / f00;

  m0 = vaddq_f32(m0, vmulq_f32(mxy, vp1));
  m1 = vaddq_f32(m1, vmulq_f32(mxy, vp2));

  const float32x4_t pd0 = vmlaq_f32(m0, u, rad_dist_u);
  const float32x4_t pd1 = vmlaq_f32(m1, v, rad_dist_u);

  vst1q_f32(pointDistorted, pd0);
  vst1q_f32(pointDistorted + 4, pd1);
  vst1q_u32(status, ret);
}

/***********************************************************************************
* [In]:    pointUndistorted -> the coordinates of point undistorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   pointDistorted -> the coordinates of point distorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   pointJacobian -> the jacobian of 4 points, size: 16 floats
*          [p0     p1     p2     p3]
*          j00    j00    j00    j00
*          j01    j01    j01    j01
*          j10    j10    j10    j10
*          j11    j11    j11    j11
*          for point i[0, 1, 2, 3], JACOBIAN is:
*          | pointJacobian + 0 + i  pointJacobian + 4 + i |
*          | pointJacobian + 8 + i  pointJacobian + 12 + i|
* [Out]:   status, 0 if distorted successfully, or 0xffffffff
* NO RETURNS
*/
void RadialTangentialDistortion8::distortT4f(
    const float* pointUndistorted, float* pointDistorted,
    float* pointJacobian, unsigned int status[4]) const {
  const float32x4_t u = vld1q_f32(pointUndistorted);  // u = (u0, u1, u2, u3)
  const float32x4_t v = vld1q_f32(pointUndistorted + 4);  // v = (v0, v1, v2, v3)
  const float32x4_t v3f = vmovq_n_f32(3.f);
  const float32x4_t v1f = vmovq_n_f32(1.f);
  const float32x4_t x0 = vmulq_f32(u, u);  // x0 = u * u;
  const float32x4_t x1 = vmulq_f32(v, v);  // x1 = v * v;
  const float32x4_t v2f = vmovq_n_f32(2.f);
  const float32x4_t x2 = vaddq_f32(x0, x1);  // x2 = x0 + x1;
  const float32x4_t x2_power2 = vmulq_f32(x2, x2);  // x2_power2 = x2 * x2;
  const uint32x4_t ret = vcgtq_f32(x2, vmovq_n_f32(R_squared));

  float32x4_t kl = vld1q_dup_f32(&k3_), kh = vld1q_dup_f32(&k6_);
  const float32x4_t x2_power3 = vmulq_f32(x2, x2_power2);
  float32x4_t uv_coeff123 = vmulq_f32(kl, x2_power2);  // uv_coeff123 = k3_ * x2_power2;
  float32x4_t uv_coeff456 = vmulq_f32(kh, x2_power2);  // uv_coeff456 = k6_ * x2_power2;
  float32x4_t m0 = vmulq_f32(kl, x2_power3);  // m0 = k3_ * x2_power3;
  float32x4_t m1 = vmulq_f32(kh, x2_power3);  // m1 = k6_ * x2_power3;

  kl = vld1q_dup_f32(&k2_), kh = vld1q_dup_f32(&k5_);

  uv_coeff123 = vmulq_f32(v3f, uv_coeff123);  // uv_coeff123 = uv_coeff123 * 3.f;
  uv_coeff456 = vmulq_f32(v3f, uv_coeff456);  // uv_coeff456 = uv_coeff456 * 3.f;
  m0 = vaddq_f32(m0, vmulq_f32(kl, x2_power2));  // m0 = m0 + k2_ * x2_power2;
  m1 = vaddq_f32(m1, vmulq_f32(kh, x2_power2));  // m1 = m1 + k5_ * x2_power2;
  // uv_coeff123 = uv_coeff123 + x2 * k2_ * 2.f
  uv_coeff123 = vaddq_f32(uv_coeff123, vmulq_f32(v2f, vmulq_f32(kl, x2)));
  // uv_coeff456 = uv_coeff456 + x2 * k5_ * 2.f
  uv_coeff456 = vaddq_f32(uv_coeff456, vmulq_f32(v2f, vmulq_f32(kh, x2)));

  kl = vld1q_dup_f32(&k1_), kh = vld1q_dup_f32(&k4_);
  m0 = vaddq_f32(m0, vmulq_f32(kl, x2));  // m0 = m0 + x2 * k1_;
  m1 = vaddq_f32(m1, vmulq_f32(kh, x2));  // m1 = m1 + x2 * k4_;
  uv_coeff123 = vaddq_f32(uv_coeff123, kl);  // uv_coeff123 = uv_coeff123 + k1_
  uv_coeff456 = vaddq_f32(uv_coeff456, kh);  // uv_coeff456 = uv_coeff456 + k4_
  m0 = vaddq_f32(m0, v1f);  // m0 = m0 + 1.f
  m1 = vaddq_f32(m1, v1f);  // m1 = m1 + 1.f
  float32x4_t pd0 = vmlaq_f32(x1, v3f, x0);
  float32x4_t pd1 = vmlaq_f32(x0, v3f, x1);

  float32x4_t x13 = vrecpeq_f32(m1);
  x13 = vmulq_f32(vrecpsq_f32(m1, x13), x13);  // x13 = 1.f / m1
  x13 = vmulq_f32(vrecpsq_f32(m1, x13), x13);  // x13 = 1.f / m1
  float32x4_t x14  = vmulq_f32(m0, x13);  // x14 = m0 * x13;
  float32x4_t vp1_ = vmulq_n_f32(v, p1_);
  float32x4_t up2_ = vmulq_n_f32(u, p2_);
  x13 = vaddq_f32(x13, x13);
  const float32x4_t jd0 = vaddq_f32(vmulq_n_f32(up2_, 6.f), vmulq_n_f32(vp1_, 2.f));
  const float32x4_t jd1 = vaddq_f32(vmulq_n_f32(up2_, 2.f), vmulq_n_f32(vp1_, 6.f));

  const float32x4_t x13_x14 = vmulq_f32(x14, x13);
  const float32x4_t uv = vmulq_f32(u, v);

  const float32x4_t j_coeff = vsubq_f32(vmulq_f32(x13, uv_coeff123),
                                        vmulq_f32(x13_x14, uv_coeff456));

  pd0 = vmulq_n_f32(pd0, p2_);
  pd1 = vmulq_n_f32(pd1, p1_);
  pd0 = vaddq_f32(pd0, vmulq_f32(u, x14));
  pd1 = vaddq_f32(pd1, vmulq_f32(v, x14));

  pd0 = vaddq_f32(pd0, vmulq_n_f32(vmulq_f32(u, vp1_), 2.f));
  pd1 = vaddq_f32(pd1, vmulq_n_f32(vmulq_f32(v, up2_), 2.f));

  float32x4_t j01 = vaddq_f32(vmulq_n_f32(v, p2_), vmulq_n_f32(u, p1_));
  float32x4_t j00 = vmlaq_f32(x14, x0, j_coeff);
  j01 = vmulq_n_f32(j01, 2.f);
  float32x4_t j11 = vmlaq_f32(x14, x1, j_coeff);
  j00 = vaddq_f32(j00, jd0);
  j11 = vaddq_f32(j11, jd1);
  j01 = vmlaq_f32(j01, uv, j_coeff);

  vst1q_f32(pointDistorted, pd0);
  vst1q_f32(pointDistorted + 4, pd1);

  vst1q_f32(pointJacobian, j00);
  vst1q_f32(pointJacobian + 4, j01);
  vst1q_f32(pointJacobian + 8, j01);
  vst1q_f32(pointJacobian + 12, j11);
  vst1q_u32(status, ret);
}
#endif  // __ARM_NEON__

template<typename T>
bool RadialTangentialDistortion8::distortT(
    const Eigen::Matrix<T, 2, 1> & pointUndistorted,
    Eigen::Matrix<T, 2, 1> * pointDistorted) const {
  // use float for speed
  register float mx_u = pointUndistorted[0] * pointUndistorted[0];
  register float my_u = pointUndistorted[1] * pointUndistorted[1];
  register float mxy_u = pointUndistorted[0] * pointUndistorted[1];
  float u0 = pointUndistorted[0];
  float u1 = pointUndistorted[1];
  register float rho_u = mx_u + my_u;
  float p1 = 2.f * p1_;
  float p2 = 2.f * p2_;
  if (rho_u > R_squared) {
    return false;
  }
  float rho_u2 = rho_u * rho_u;
  float f00 = k6_ * rho_u + k5_;
  float f01 = k4_ * rho_u + 1.f;
  float f10 = k3_ * rho_u + k2_;
  float f11 = k1_ * rho_u + 1.f;
  f00 = f00 * rho_u2 + f01;
  f10 = f10 * rho_u2 + f11;
  // [NOTE] Depending on calibration, it's actually possible that f00 crosses zero as
  // r (rho_u) increases. We need to protect distortT from the singular case of division by zero.
  if (fabs(f00) < 1e-6) {
    LOG(WARNING) << "|f00| is smaller than 1e-6 at rho = " << rho_u;
    return false;
  }
  float rad_dist_u = f10 / f00;
  float m0 = p2 * mx_u + p2_ * rho_u + p1 * mxy_u;
  float m1 = p1 * my_u + p1_ * rho_u + p2 * mxy_u;

  (*pointDistorted)[0] = u0 * rad_dist_u + m0;
  (*pointDistorted)[1] = u1 * rad_dist_u + m1;
  return true;
}

template<typename T>
bool RadialTangentialDistortion8::distortT(
    const Eigen::Matrix<T, 2, 1> & pointUndistorted,
    Eigen::Matrix<T, 2, 1> * pointDistorted,
    Eigen::Matrix<T, 2, 2> * pointJacobian,
    Eigen::Matrix<T, 2, Eigen::Dynamic> * parameterJacobian) const {
  const T x0 = pointUndistorted[0] * pointUndistorted[0];
  const T x1 = pointUndistorted[1] * pointUndistorted[1];
  const T u = pointUndistorted[0];
  const T v = pointUndistorted[1];
  const T x2 = x0 + x1;
  const T pd0 = 3.f * x0 + x1;
  const T pd1 = x0 + 3.f * x1;
  const T x2_power2 = x2 * x2;
  if (x2 > R_squared) {
    return false;  // to avoid confusion of this model
  }
  const T x2_power3 = x2 * x2_power2;

  const T m1 = 1.f + k4_ * x2 + x2_power2 * k5_ + k6_ * x2_power3;
  const T m0 = 1.f + k1_ * x2 + x2_power2 * k2_ + k3_ * x2_power3;

  if (fabs(m1) < 1e-6) {
    LOG(WARNING) << "|f00| is smaller than 1e-6 at rho = " << x2;
    return false;
  }

  const T uv_coeff456 = k4_ + (k5_ + k5_) * x2 + 3.f * k6_ * x2_power2;
  const T uv_coeff123 = k1_ + (k2_ + k2_) * x2 + 3.f * k3_ * x2_power2;

  T x13 = 1.f / m1;
  const T vp1_ = v * p1_;
  const T up2_ = u * p2_;
  const T x14 = m0 * x13;
  x13 = x13 + x13;
  const T jd0 = 6.f * up2_ + 2.f * vp1_;
  const T jd1 = 2.f * up2_ + 6.f * vp1_;
  const T x13_x14 = x13 * x14;
  const T uv = u * v;
  const T j_coeff = x13 * uv_coeff123 - x13_x14 * uv_coeff456;

  (*pointDistorted)[0] = p2_ * pd0 + u * (x14 + 2.f * vp1_);
  (*pointDistorted)[1] = p1_ * pd1 + v * (x14 + 2.f * up2_ );

  Eigen::Matrix<T, 2, 2>& J = *pointJacobian;

  J(0, 0) = (jd0 + x14) + x0 * j_coeff;
  J(0, 1) = J(1, 0) = uv * j_coeff + 2.f * (p2_ * v + u * p1_);
  J(1, 1) = (jd1 + x14) + x1 * j_coeff;

  if (parameterJacobian) {
    LOG(FATAL) << "parameterjacobian is NOT supported in RadialTangentialDistortion8";
  }

  return true;
}


template<typename T>
bool RadialTangentialDistortion8::distortT_slow(
    const Eigen::Matrix<T, 2, 1> & pointUndistorted,
    Eigen::Matrix<T, 2, 1> * pointDistorted,
    Eigen::Matrix<T, 2, 2> * pointJacobian,
    Eigen::Matrix<T, 2, Eigen::Dynamic> * parameterJacobian) const {
  // use float for speed
  // first compute the distorted point
  const T u0 = pointUndistorted[0];
  const T u1 = pointUndistorted[1];
  const T mx_u = u0 * u0;
  const T my_u = u1 * u1;
  const T mxy_u = u0 * u1;
  const T rho_u = mx_u + my_u;

  if (rho_u > R_squared) {
    return false;  // to avoid confusion of this model
  }

  const T c = rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0;
  const T c2 = c*c;

  // TODO(mingyu): It's possible to have denominator equals zero!
  const T rad_dist_u = (1.f + ((k3_ * rho_u + k2_) * rho_u + k1_) * rho_u)
      / (1.f + ((k6_ * rho_u + k5_) * rho_u + k4_) * rho_u);
  (*pointDistorted)[0] = u0 * rad_dist_u + 2.f * p1_ * mxy_u + p2_ * (rho_u + 2.f * mx_u);
  (*pointDistorted)[1] = u1 * rad_dist_u + 2.f * p2_ * mxy_u + p1_ * (rho_u + 2.f * my_u);

  // next the Jacobian w.r.t. changes on the undistorted point
  Eigen::Matrix<T, 2, 2> & J = *pointJacobian;
  J(0,0) = p1_*u1*2.0+p2_*u0*6.0+(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)+(u0*(rho_u*(u0*(k2_+k3_*rho_u)*2.0+k3_*u0*rho_u*2.0)+u0*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u0*(rho_u*(u0*(k5_+k6_*rho_u)*2.0+k6_*u0*rho_u*2.0)+u0*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(0,1) = p1_*u0*2.0+p2_*u1*2.0+(u0*(rho_u*(u1*(k2_+k3_*rho_u)*2.0+k3_*u1*rho_u*2.0)+u1*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u0*(rho_u*(u1*(k5_+k6_*rho_u)*2.0+k6_*u1*rho_u*2.0)+u1*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(1,0) = p1_*u0*2.0+p2_*u1*2.0+(u1*(rho_u*(u0*(k2_+k3_*rho_u)*2.0+k3_*u0*rho_u*2.0)+u0*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u1*(rho_u*(u0*(k5_+k6_*rho_u)*2.0+k6_*u0*rho_u*2.0)+u0*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(1,1) = p1_*u1*6.0+p2_*u0*2.0+(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)+(u1*(rho_u*(u1*(k2_+k3_*rho_u)*2.0+k3_*u1*rho_u*2.0)+u1*(k1_+rho_u*(k2_+k3_*rho_u))*2.0))/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0)-u1*(rho_u*(u1*(k5_+k6_*rho_u)*2.0+k6_*u1*rho_u*2.0)+u1*(k4_+rho_u*(k5_+k6_*rho_u))*2.0)*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT

  if (parameterJacobian) {
    // the Jacobian w.r.t. intrinsics parameters
    const T rho_u2 = rho_u*rho_u;
    const T rho_u3 = rho_u*rho_u2;
    Eigen::Matrix<T, 2, Eigen::Dynamic> & Jp = *parameterJacobian;
    Jp.resize(2, NumDistortionIntrinsics);
    Jp(0,0) = (u0*rho_u)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);         // NOLINT
    Jp(0,1) = (u0*rho_u2)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);        // NOLINT
    Jp(0,2) = u0*u1*2.0;                                                  // NOLINT
    Jp(0,3) = (u0*u0)*3.0+u1*u1;                                          // NOLINT
    Jp(0,4) = (u0*rho_u3)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);        // NOLINT
    Jp(0,5) = -u0*rho_u*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;   // NOLINT
    Jp(0,6) = -u0*rho_u2*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(0,7) = -u0*rho_u3*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(1,0) = (u1*rho_u)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);         // NOLINT
    Jp(1,1) = (u1*rho_u2)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);        // NOLINT
    Jp(1,2) = u0*u0+(u1*u1)*3.0;                                          // NOLINT
    Jp(1,3) = u0*u1*2.0;                                                  // NOLINT
    Jp(1,4) = (u1*rho_u3)/(rho_u*(k4_+rho_u*(k5_+k6_*rho_u))+1.0);        // NOLINT
    Jp(1,5) = -u1*rho_u*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;   // NOLINT
    Jp(1,6) = -u1*rho_u2*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(1,7) = -u1*rho_u3*(rho_u*(k1_+rho_u*(k2_+k3_*rho_u))+1.0)*1.0/c2;  // NOLINT
  }

  return true;
}

bool RadialTangentialDistortion8::distortWithExternalParameters(
    const Eigen::Vector2d & pointUndistorted,
    const Eigen::VectorXd & parameters, Eigen::Vector2d * pointDistorted,
    Eigen::Matrix2d * pointJacobian, Eigen::Matrix2Xd * parameterJacobian) const {
  // use float for speed
  // calibration is not accurate anyway
  const float k1 = parameters[0];
  const float k2 = parameters[1];
  const float p1 = parameters[2];
  const float p2 = parameters[3];
  const float k3 = parameters[4];
  const float k4 = parameters[5];
  const float k5 = parameters[6];
  const float k6 = parameters[7];
  // first compute the distorted point
  const float u0 = pointUndistorted[0];
  const float u1 = pointUndistorted[1];
  const float mx_u = u0 * u0;
  const float my_u = u1 * u1;
  const float mxy_u = u0 * u1;
  const float rho_u = mx_u + my_u;

  if (rho_u > R_squared) {
    return false;  // to avoid confusion of this model
  }

  const float c = rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0;
  const float c2 = c*c;

  const float rad_dist_u = (1.0 + ((k3 * rho_u + k2) * rho_u + k1) * rho_u)
        / (1.0 + ((k6 * rho_u + k5) * rho_u + k4) * rho_u);
    (*pointDistorted)[0] = u0 * rad_dist_u + 2.0 * p1 * mxy_u
        + p2 * (rho_u + 2.0 * mx_u);
    (*pointDistorted)[1] = u1 * rad_dist_u + 2.0 * p2 * mxy_u
        + p1 * (rho_u + 2.0 * my_u);

  // next the Jacobian w.r.t. changes on the undistorted point
  Eigen::Matrix2d & J = *pointJacobian;
  J(0,0) = p1*u1*2.0+p2*u0*6.0+(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)+(u0*(rho_u*(u0*(k2+k3*rho_u)*2.0+k3*u0*rho_u*2.0)+u0*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u0*(rho_u*(u0*(k5+k6*rho_u)*2.0+k6*u0*rho_u*2.0)+u0*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(0,1) = p1*u0*2.0+p2*u1*2.0+(u0*(rho_u*(u1*(k2+k3*rho_u)*2.0+k3*u1*rho_u*2.0)+u1*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u0*(rho_u*(u1*(k5+k6*rho_u)*2.0+k6*u1*rho_u*2.0)+u1*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(1,0) = p1*u0*2.0+p2*u1*2.0+(u1*(rho_u*(u0*(k2+k3*rho_u)*2.0+k3*u0*rho_u*2.0)+u0*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u1*(rho_u*(u0*(k5+k6*rho_u)*2.0+k6*u0*rho_u*2.0)+u0*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
  J(1,1) = p1*u1*6.0+p2*u0*2.0+(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)+(u1*(rho_u*(u1*(k2+k3*rho_u)*2.0+k3*u1*rho_u*2.0)+u1*(k1+rho_u*(k2+k3*rho_u))*2.0))/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0)-u1*(rho_u*(u1*(k5+k6*rho_u)*2.0+k6*u1*rho_u*2.0)+u1*(k4+rho_u*(k5+k6*rho_u))*2.0)*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT

  if (parameterJacobian) {
    // the Jacobian w.r.t. intrinsics parameters
    const float rho_u2 = rho_u*rho_u;
    const float rho_u3 = rho_u*rho_u2;
    Eigen::Matrix2Xd & Jp = *parameterJacobian;
    Jp.resize(2, NumDistortionIntrinsics);
    Jp(0,0) = (u0*rho_u)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);         // NOLINT
    Jp(0,1) = (u0*rho_u2)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);        // NOLINT
    Jp(0,2) = u0*u1*2.0;                                                  // NOLINT
    Jp(0,3) = (u0*u0)*3.0+u1*u1;                                          // NOLINT
    Jp(0,4) = (u0*rho_u3)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);        // NOLINT
    Jp(0,5) = -u0*rho_u*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;   // NOLINT
    Jp(0,6) = -u0*rho_u2*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(0,7) = -u0*rho_u3*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(1,0) = (u1*rho_u)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);         // NOLINT
    Jp(1,1) = (u1*rho_u2)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);        // NOLINT
    Jp(1,2) = u0*u0+(u1*u1)*3.0;                                          // NOLINT
    Jp(1,3) = u0*u1*2.0;                                                  // NOLINT
    Jp(1,4) = (u1*rho_u3)/(rho_u*(k4+rho_u*(k5+k6*rho_u))+1.0);        // NOLINT
    Jp(1,5) = -u1*rho_u*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;   // NOLINT
    Jp(1,6) = -u1*rho_u2*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
    Jp(1,7) = -u1*rho_u3*(rho_u*(k1+rho_u*(k2+k3*rho_u))+1.0)*1.0/c2;  // NOLINT
  }
  return true;
}


#ifdef __ARM_NEON__
/***********************************************************************************
* [In]:    uv_dist4f -> the coordinates of distorted points, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   uv_undist4f -> the coordinates of point undistorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   status, 0 if distorted successfully, or 0xffffffff
* NO RETURNS
*/
void RadialTangentialDistortion8::undistort4f(const float* uv_dist4f,
                          float* uv_undist4f, unsigned int status[4]) const {
  // uv_dist4f has high degreee of temporal locality
  __builtin_prefetch(uv_dist4f, 0, 3);

  const int iter = 7;
  union {
    unsigned int success[4] = {0};
    uint32x4_t vsuccess;
  };

  union {
    unsigned int s[4] = {0};
    uint32x4_t vs;
  };

  union {
    float uv[8];
    struct {
      float32x4_t u;
      float32x4_t v;
    };
  } src, in, tmp, e;

  union {
    float pointJacobians[16];
    float32x4x4_t E;  // E.val[0] = j00, E.val[1] = j01, E.val[2] = j10, E.val[3] = j11
  };

  union {
    unsigned int sts[4] = {0};
    uint32x4_t vsts;
  };

  float32x4_t det_1, chi2;
  src.u = in.u = vld1q_f32(uv_dist4f);
  src.v = in.v = vld1q_f32(uv_dist4f + 4);

  for (int i = iter; i > 0; --i) {
    this->distortT4f(reinterpret_cast<float*>(in.uv),
        reinterpret_cast<float*>(tmp.uv), reinterpret_cast<float*>(pointJacobians), s);
    vsuccess = vorrq_u32(vsuccess, vs);
    e.u = vsubq_f32(src.u, tmp.u);
    e.v = vsubq_f32(src.v, tmp.v);
    // calculate E.inverse() * e
    {
      float32x4_t det = vsubq_f32(vmulq_f32(E.val[0], E.val[3]), vmulq_f32(E.val[1], E.val[2]));
      // det_1 = 1/det
      det_1 = vrecpeq_f32(det);
      det_1 = vmulq_f32(vrecpsq_f32(det, det_1), det_1);
      det_1 = vmulq_f32(vrecpsq_f32(det, det_1), det_1);
    }
    float32x4_t duu = vsubq_f32(vmulq_f32(E.val[3], e.u),
                                vmulq_f32(E.val[1], e.v));
    float32x4_t duv = vsubq_f32(vmulq_f32(E.val[0], e.v),
                                vmulq_f32(E.val[2], e.u));
    // update the input of distortT4f
    in.u = vaddq_f32(in.u, vmulq_f32(duu, det_1));
    in.v = vaddq_f32(in.v, vmulq_f32(duv, det_1));

    chi2 = vaddq_f32(vmulq_f32(e.u, e.u), vmulq_f32(e.v, e.v));
    vsts = vcgtq_f32(chi2, vmovq_n_f32(1e-8));
    if ((sts[0] | sts[1] | sts[2] | sts[3]) == 0) {
      break;  // if accuracy is all met, jump out from the Gauss-Newton iteration loop
    }
  }

  chi2 = vaddq_f32(vmulq_f32(e.u, e.u), vmulq_f32(e.v, e.v));
  // we set 1e-4 accuracy requirement here, if this accuracy is met, we also use the results
  // if e is less than 1e-8, e is also less than 1e-4
  vsts = vcgtq_f32(chi2, vmovq_n_f32(1e-4));

  chi2 = vaddq_f32(vmulq_f32(in.u, in.u), vmulq_f32(in.v, in.v));
  // check for R_squared and merge status with distortT4f
  vsuccess = vorrq_u32(vsuccess, vcgtq_f32(chi2, vmovq_n_f32(R_squared)));
  // merge status with accuracy requirement and store status
  vst1q_u32(status, vorrq_u32(vsuccess, vsts));
  // store undistorted points
  vst1q_f32(uv_undist4f, in.u);
  vst1q_f32(uv_undist4f + 4, in.v);
}

/***********************************************************************************
* [In]:    uv_dist4f -> the coordinates of point distorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   uv_undist4f -> the coordinates of point undistorted, size: 8 floats
*          u0, u1, u2, u3, v0, v1, v2, v3
* [Out]:   jacobians_out -> the jacobian of 4 points, size: 16 floats
*          [p0     p1     p2     p3]
*          j00    j00    j00    j00
*          j01    j01    j01    j01
*          j10    j10    j10    j10
*          j11    j11    j11    j11
*          for point i[0, 1, 2, 3], JACOBIAN is:
*          | jacobians_out + 0 + i  jacobians_out + 4 + i |
*          | jacobians_out + 8 + i  jacobians_out + 12 + i|
* [Out]:   status, 0 if distorted successfully, or 0xffffffff
* NO RETURNS
*/
void RadialTangentialDistortion8::undistort4f(const float* uv_dist4f, float* uv_undist4f,
           float* jacobians_out, unsigned int status[4]) const {
  // uv_dist4f has high degreee of temporal locality
  __builtin_prefetch(uv_dist4f, 0, 3);
  __builtin_prefetch(jacobians_out, 1, 0);

  const int iter = 7;
  union {
    unsigned int success[4] = {0};
    uint32x4_t vsuccess;
  };

  union {
    unsigned int s[4] = {0};
    uint32x4_t vs;
  };

  union {
    float uv[8];
    struct {
      float32x4_t u;
      float32x4_t v;
    };
  } src, in, tmp, e;

  union {
    float pointJacobians[16];
    float32x4x4_t E;  // E.val[0] = j00, E.val[1] = j01, E.val[2] = j10, E.val[3] = j11
  };

  union {
    unsigned int sts[4] = {0};
    uint32x4_t vsts;
  };

  float32x4_t det_1, chi2;
  src.u = in.u = vld1q_f32(uv_dist4f);
  src.v = in.v = vld1q_f32(uv_dist4f + 4);

  for (int i = iter; i > 0; --i) {
    this->distortT4f(reinterpret_cast<float*>(in.uv),
        reinterpret_cast<float*>(tmp.uv), reinterpret_cast<float*>(pointJacobians), s);
    vsuccess = vorrq_u32(vsuccess, vs);
    e.u = vsubq_f32(src.u, tmp.u);
    e.v = vsubq_f32(src.v, tmp.v);
    // calculate E.inverse() * e
    {
      float32x4_t det = vsubq_f32(vmulq_f32(E.val[0], E.val[3]), vmulq_f32(E.val[1], E.val[2]));
      // det_1 = 1/det
      det_1 = vrecpeq_f32(det);
      det_1 = vmulq_f32(vrecpsq_f32(det, det_1), det_1);
      det_1 = vmulq_f32(vrecpsq_f32(det, det_1), det_1);
    }
    float32x4_t duu = vsubq_f32(vmulq_f32(E.val[3], e.u),
                                vmulq_f32(E.val[1], e.v));
    float32x4_t duv = vsubq_f32(vmulq_f32(E.val[0], e.v),
                                vmulq_f32(E.val[2], e.u));
    // update the input of distortT4f
    in.u = vaddq_f32(in.u, vmulq_f32(duu, det_1));
    in.v = vaddq_f32(in.v, vmulq_f32(duv, det_1));

    chi2 = vaddq_f32(vmulq_f32(e.u, e.u), vmulq_f32(e.v, e.v));
    vsts = vcgtq_f32(chi2, vmovq_n_f32(1e-8));
    if ((sts[0] | sts[1] | sts[2] | sts[3]) == 0) {
      break;  // if accuracy is all met, jump out from the Gauss-Newton iteration loop
    }
  }

  chi2 = vaddq_f32(vmulq_f32(e.u, e.u), vmulq_f32(e.v, e.v));
  // we set 1e-4 accuracy requirement here, if this accuracy is met, we also use the results
  // if e is less than 1e-8, e is also less than 1e-4
  vsts = vcgtq_f32(chi2, vmovq_n_f32(1e-4));

  chi2 = vaddq_f32(vmulq_f32(in.u, in.u), vmulq_f32(in.v, in.v));
  // check for R_squared and merge status with distortT4f
  vsuccess = vorrq_u32(vsuccess, vcgtq_f32(chi2, vmovq_n_f32(R_squared)));
  // merge status with accuracy requirement and store status
  vst1q_u32(status, vorrq_u32(vsuccess, vsts));
  // store undistorted points
  vst1q_f32(uv_undist4f, in.u);
  vst1q_f32(uv_undist4f + 4, in.v);

  // if needed, calculate jacobian and store it
  if (__builtin_expect(jacobians_out != NULL, 1)) {
    // simply equal to E.inverse() here
    float32x4_t tmp = E.val[3];
    E.val[3] = vmulq_f32(E.val[0], det_1);
    E.val[0] = vmulq_f32(tmp, det_1);
    E.val[2] = vmulq_n_f32(E.val[2], -1.f);
    E.val[1] = vmulq_f32(E.val[2], det_1);
    E.val[2] = E.val[1];
    // we don't change the order of jacobian data, cause it needs further NEON calculation
    vst1q_f32(jacobians_out, E.val[0]);
    vst1q_f32(jacobians_out + 4, E.val[1]);
    vst1q_f32(jacobians_out + 8, E.val[2]);
    vst1q_f32(jacobians_out + 12, E.val[3]);
  }
}
#endif

bool RadialTangentialDistortion8::undistort_GN(float u_dist,
                                               float v_dist,
                                               float* u_undist,
                                               float* v_undist,
                                               Eigen::Matrix2f * pointJacobian) const {
  // this is expensive: we solve with Gauss-Newton
  const Eigen::Vector2f uv_dist_in(u_dist, v_dist);
  Eigen::Vector2f x_bar = uv_dist_in;  // initialise at distorted point
  const int n = 7;  // just 5 iterations max.
  Eigen::Matrix2f E = Eigen::Matrix2f::Zero();  // error Jacobian
  bool success = false;
  for (int i = 0; i < n; i++) {
    Eigen::Vector2f x_tmp;
    // do not use distort() since it uses lookup table
    if (!this->distortT<float>(x_bar, &x_tmp, &E)) {
      break;
    }
    Eigen::Vector2f e(uv_dist_in - x_tmp);
#if 0
    // GOOGLE TEST accuracy 1e-6
    Eigen::Matrix2f E2 = (E.transpose() * E);
    Eigen::Vector2f du = E2.inverse() * E.transpose() * e;
#else
    // GOOGLE TEST accuracy 1e-5
    Eigen::Vector2f du = E.inverse() * e;
#endif
    float chi2 = e.dot(e);
    x_bar += du;

    if (chi2 < 1e-4) {
      success = true;
    }

    if (chi2 < 1e-8) {
      success = true;
      break;
    }
  }

  *u_undist = x_bar[0];
  *v_undist = x_bar[1];

  if (x_bar.squaredNorm() > R_squared) {
    success = false;
  }
  /*
  if (!success) {
    VLOG(0) << "RadialTangentialDistortion8::undistort failed "
            << (E.transpose() * E);
  }
  */
  if (success && pointJacobian != nullptr) {
    // the Jacobian of the inverse map is simply the inverse Jacobian.
    *pointJacobian = E.inverse();
  }
  return success;
}

bool RadialTangentialDistortion8::undistort(const Eigen::Vector2d & pointDistorted,
                                            Eigen::Vector2d * pointUndistorted) const {
  bool success = false;
  /*
  int u_lt = std::round(pointDistorted[0] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_u_);
  int v_lt = std::round(pointDistorted[1] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_v_);
  if (u_lt >= 0 &&
      u_lt < undistort_lookup_table_.u_.cols() &&
      v_lt >= 0 &&
      v_lt < undistort_lookup_table_.u_.rows()) {
    if (undistort_lookup_table_.valid_(v_lt, u_lt) != 0) {
      (*pointUndistorted)[0] = undistort_lookup_table_.u_(v_lt, u_lt);
      (*pointUndistorted)[1] = undistort_lookup_table_.v_(v_lt, u_lt);
      success = true;
    } else {
      // LOG(ERROR) << "a point at the margin of the image cannot be undistorted by lookup table"
      //            << " (double) v_lt " << v_lt << " u_lt " << u_lt;
    }
  } else {
    // undistort at the margin is very likely to fail,
    // and the result is inaccurate. Don't bother to try
    // float undistort_u, undistort_v;
    // success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
    //                             &undistort_u, &undistort_v);
    // (*pointUndistorted)[0] = undistort_u;
    // (*pointUndistorted)[1] = undistort_v;
  }
  */
  float undistort_u, undistort_v;
  success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                               &undistort_u, &undistort_v);
  (*pointUndistorted)[0] = undistort_u;
  (*pointUndistorted)[1] = undistort_v;
  return success;
}
bool RadialTangentialDistortion8::undistort(const Eigen::Vector2f & pointDistorted,
                                            Eigen::Vector2f * pointUndistorted) const {
  // TODO(mingyu): LUT may result in nan values. Need to futher debug
  // TODO(mingyu): We'll directly compute as in undistort for Eigen::Vector2d
  /*
  int u_lt = std::round(pointDistorted[0] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_u_);
  int v_lt = std::round(pointDistorted[1] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_v_);
  bool success = false;
  if (u_lt >= 0 &&
      u_lt < undistort_lookup_table_.u_.cols() &&
      v_lt >= 0 &&
      v_lt < undistort_lookup_table_.u_.rows()) {
    if (undistort_lookup_table_.valid_(v_lt, u_lt) != 0) {
      (*pointUndistorted)[0] = undistort_lookup_table_.u_(v_lt, u_lt);
      (*pointUndistorted)[1] = undistort_lookup_table_.v_(v_lt, u_lt);
      success = true;
    } else {
      LOG(ERROR) << "a point at the margin of the image cannot be undistorted by lookup table"
                 << " (float) v_lt " << v_lt << " u_lt " << u_lt;
    }
  } else {
    // TODO(mingyu): undistort at the margin is very likely to fail, and the result is inaccurate.
    // Don't bother to try?
    success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                                 &((*pointUndistorted)[0]), &((*pointUndistorted)[1]));
  }
  */
  float undistort_u, undistort_v;
  bool success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                                    &undistort_u, &undistort_v);
  (*pointUndistorted)[0] = undistort_u;
  (*pointUndistorted)[1] = undistort_v;
  return success;
}

bool RadialTangentialDistortion8::undistort(const Eigen::Vector2d & pointDistorted,
                                            Eigen::Vector2d * pointUndistorted,
                                            Eigen::Matrix2d * pointJacobian) const {
  Eigen::Matrix2f J;
  float undistort_u, undistort_v;
  bool success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                                    &undistort_u, &undistort_v, &J);
  (*pointUndistorted)[0] = undistort_u;
  (*pointUndistorted)[1] = undistort_v;
  if (success && pointJacobian != nullptr) {
    *pointJacobian = J.cast<double>();
  }
  return success;
}

bool RadialTangentialDistortion8::undistort(const Eigen::Vector2f& pointDistorted,
                                            Eigen::Vector2f* pointUndistorted,
                                            Eigen::Matrix2f* pointJacobian) const {
  /*
  int u_lt = std::round(pointDistorted[0] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_u_);
  int v_lt = std::round(pointDistorted[1] * undistort_lookup_table_.multiple_
                        + undistort_lookup_table_.shift_v_);
  bool success = false;
  if (u_lt >= 0 &&
      u_lt < undistort_lookup_table_.u_.cols() &&
      v_lt >= 0 &&
      v_lt < undistort_lookup_table_.u_.rows()) {
    if (undistort_lookup_table_.valid_(v_lt, u_lt) != 0) {
      (*pointUndistorted)[0] = undistort_lookup_table_.u_(v_lt, u_lt);
      (*pointUndistorted)[1] = undistort_lookup_table_.v_(v_lt, u_lt);
      (*pointJacobian)(0, 0) = undistort_lookup_table_.J00_(v_lt, u_lt);
      (*pointJacobian)(0, 1) = undistort_lookup_table_.J01_(v_lt, u_lt);
      (*pointJacobian)(1, 0) = undistort_lookup_table_.J10_(v_lt, u_lt);
      (*pointJacobian)(1, 1) = undistort_lookup_table_.J11_(v_lt, u_lt);
      success = true;
    } else {
      // LOG(ERROR) << "a point at the margin of the image cannot be undistorted by lookup table"
      // << " (float) v_lt " << v_lt << " u_lt " << u_lt;
    }
  } else {
    // TODO(mingyu): undistort at the margin is very likely to fail, and the result is inaccurate.
    // Don't bother to try?
    success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                                 &((*pointUndistorted)[0]), &((*pointUndistorted)[1]),
                                 pointJacobian);
  }
  */
  bool success = this->undistort_GN(pointDistorted[0], pointDistorted[1],
                                    &((*pointUndistorted)[0]), &((*pointUndistorted)[1]),
                                    pointJacobian);
  return success;
}

}  // namespace cameras
}  // namespace vio
