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
 *  Created on: Jan 28, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file implementation/PinholeCamera.hpp
 * @brief Header implementation file for the PinholeCamera class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */
#include <vector>
#include <cassert>

// \brief vio Main namespace of this package.
namespace vio {
// \brief cameras Namespace for camera-related functionality.
namespace cameras {

template<class DISTORTION_T>
PinholeCamera<DISTORTION_T>::PinholeCamera(int imageWidth,
                                           int imageHeight,
                                           double focalLengthU,
                                           double focalLengthV,
                                           double imageCenterU,
                                           double imageCenterV,
                                           const distortion_t & distortion,
                                           uint64_t id)
    : CameraBase(imageWidth, imageHeight, id),
    distortion_(distortion),
    fu_(focalLengthU),
    fv_(focalLengthV),
    cu_(imageCenterU),
    cv_(imageCenterV) {
  intrinsics_[0] = fu_;  //< focalLengthU
  intrinsics_[1] = fv_;  //< focalLengthV
  intrinsics_[2] = cu_;  //< imageCenterU
  intrinsics_[3] = cv_;  //< imageCenterV
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
}

// overwrite all intrinsics - use with caution !
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::setIntrinsics(
    const Eigen::VectorXd & intrinsics) {
  if (intrinsics.cols() != NumIntrinsics) {
    return false;
  }
  intrinsics_ = intrinsics;
  fu_ = intrinsics[0];  //< focalLengthU
  fv_ = intrinsics[1];  //< focalLengthV
  cu_ = intrinsics[2];  //< imageCenterU
  cv_ = intrinsics[3];  //< imageCenterV
  distortion_.setParameters(
      intrinsics.tail<distortion_t::NumDistortionIntrinsics>());
  one_over_fu_ = 1.0 / fu_;  //< 1.0 / fu_
  one_over_fv_ = 1.0 / fv_;  //< 1.0 / fv_
  fu_over_fv_ = fu_ / fv_;  //< fu_ / fv_
  return true;
}

template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::getIntrinsics(Eigen::VectorXd* intrinsics) const {
  *intrinsics = intrinsics_;
  Eigen::VectorXd distortionIntrinsics;
  if (distortion_t::NumDistortionIntrinsics > 0) {
    distortion_.getParameters(distortionIntrinsics);
    intrinsics->tail<distortion_t::NumDistortionIntrinsics>() = distortionIntrinsics;
  }
}

//////////////////////////////////////////
// Methods to project points

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint) const {
  // handle singularity
  if (fabs(point[2]) < 1e-6) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2]; imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  // distortion
  Eigen::Vector2d imagePoint2;
  if (!distortion_.distort(imagePointUndistorted, &imagePoint2)) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    return CameraBase::ProjectionStatus::Invalid;
  }

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if (point[2] > 0.0) {
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3f & point, Eigen::Vector2f * imagePoint) const {
  // handle singularity
  if (fabs(point[2]) < 1e-6) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2f imagePointUndistorted;
  const float rz = 1.0 / point[2];
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  // distortion
  Eigen::Vector2f imagePoint2;
  if (!distortion_.distort(imagePointUndistorted, &imagePoint2)) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    return CameraBase::ProjectionStatus::Invalid;
  }

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if (point[2] > 0.0) {
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

#ifdef __ARM_NEON__
/*******************************************************************************
* project 4 3d points into image plane simultaneously.
* [In]:    points -> the coordinates of 4 3d points, size: 12 floats
*          x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3
* [Out]:   image points
*          u0, v0, u1, v1, u2, v2, u3, v3
* [Out]:   status[i] -> 0 if point i is projected succesffuly, or 0xffffffff
* NO RETURNS
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::project4f(
    const float* points, float* imagePoints, ProjectionStatus status[4]) const {
  float32x4x3_t p4f = vld3q_f32(points);
  union {
    uint32x4_t vres0;
    uint32_t res0[4];
  };
  union {
    uint32x4_t vres1;
    uint32_t res1[4];
  };
  union {
    uint32x4_t vgtz;
    uint32_t gtz[4];
  };

  vres0 = vcaltq_f32(p4f.val[2], vmovq_n_f32(1e-6));
  vgtz = vcgtq_f32(p4f.val[2], vmovq_n_f32(0.f));
  // singularity checking
  if ((res0[0] & res0[1] & res0[2] & res0[3]) != 0) {
    float32x4x2_t tmp = {
      vmovq_n_f32(cu_),
      vmovq_n_f32(cv_)
    };
    vst2q_f32(imagePoints, tmp);
    status[0] = CameraBase::ProjectionStatus::Invalid;
    status[1] = CameraBase::ProjectionStatus::Invalid;
    status[2] = CameraBase::ProjectionStatus::Invalid;
    status[3] = CameraBase::ProjectionStatus::Invalid;
    return;
  }

  // projection
  float32x4_t vrz = vrecpeq_f32(p4f.val[2]);
  vrz = vmulq_f32(vrecpsq_f32(p4f.val[2], vrz), vrz);  // vrz = 1.f / p4f.val[2]
  vrz = vmulq_f32(vrecpsq_f32(p4f.val[2], vrz), vrz);
  p4f.val[0] = vmulq_f32(p4f.val[0], vrz);
  p4f.val[1] = vmulq_f32(p4f.val[1], vrz);

  union {
    float32x4x2_t v_point_distorted;
    float point_distorted[8];
  };
  distortion_.distort4f(reinterpret_cast<const float*>(&p4f),
                         point_distorted, res1);
  vres0 = vorrq_u32(vres0, vres1);
  if ((res0[0] & res0[1] & res0[2] & res0[3]) != 0) {
    float32x4x2_t tmp = {
      vmovq_n_f32(cu_),
      vmovq_n_f32(cv_)
    };
    vst2q_f32(imagePoints, tmp);
    status[0] = CameraBase::ProjectionStatus::Invalid;
    status[1] = CameraBase::ProjectionStatus::Invalid;
    status[2] = CameraBase::ProjectionStatus::Invalid;
    status[3] = CameraBase::ProjectionStatus::Invalid;
    return;
  }

  // scale and offset.
  v_point_distorted.val[0] = vaddq_f32(vmulq_n_f32(
                               v_point_distorted.val[0], fu_), vmovq_n_f32(cu_));
  v_point_distorted.val[1] = vaddq_f32(vmulq_n_f32(
                               v_point_distorted.val[1], fv_), vmovq_n_f32(cv_));

  __builtin_prefetch(imagePoints, 1, 1);
  isInImage(point_distorted, res1);
  for (int i = 0; i < 4; ++i) {
    if (res0[i] != 0) {
      point_distorted[i] = cu_;
      point_distorted[i + 4] = cv_;
      status[i] = CameraBase::ProjectionStatus::Invalid;
      continue;
    }

    if (res1[i] != 0) {
      status[i] = CameraBase::ProjectionStatus::OutsideImage;
      continue;
    }
    if (CameraBase::isMasked(Eigen::Vector2f(point_distorted[i], point_distorted[i + 4]))) {
      status[i] = CameraBase::ProjectionStatus::Masked;
      continue;
    }
    if (gtz[i] != 0) {
      status[i] = CameraBase::ProjectionStatus::Successful;
    } else {
      status[i] = CameraBase::ProjectionStatus::Behind;
    }
  }
  vst2q_f32(imagePoints, v_point_distorted);
}
#endif  // __ARM_NEON__

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::project(
    const Eigen::Vector3d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 3> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian,
    Eigen::Matrix2d* distortJacobianPtr) const {
  // handle singularity
  if (fabs(point[2]) < 1e-6) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    pointJacobian->setZero();
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2];
  double rz2 = rz * rz;
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  Eigen::Matrix<double, 2, 3> pointJacobianProjection;
  Eigen::Matrix2Xd intrinsicsJacobianProjection;
  Eigen::Matrix2d distortionJacobian(Eigen::Matrix2d::Zero());
  Eigen::Matrix2Xd intrinsicsJacobianDistortion;
  Eigen::Vector2d imagePoint2(0.0, 0.0);
  bool distortionSuccess;
  if (intrinsicsJacobian) {
    // get both Jacobians
    intrinsicsJacobian->resize(2, NumIntrinsics);

    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian,
                                            &intrinsicsJacobianDistortion);
    // compute the intrinsics Jacobian
    intrinsicsJacobian->template topLeftCorner<2, 2>() = imagePoint2.asDiagonal();
    intrinsicsJacobian->template block<2, 2>(0, 2) = Eigen::Matrix2d::Identity();

    if (distortion_t::NumDistortionIntrinsics > 0) {
      intrinsicsJacobian
          ->template bottomRightCorner<2, distortion_t::NumDistortionIntrinsics>() =
          Eigen::Vector2d(fu_, fv_).asDiagonal() * intrinsicsJacobianDistortion;  // chain rule
    }
  } else {
    // only get point Jacobian
    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian);
  }

  if (distortJacobianPtr) {
    *distortJacobianPtr = distortionJacobian;
  }

  if (!distortionSuccess) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    pointJacobian->setZero();
    return CameraBase::ProjectionStatus::Invalid;
  }

  // compute the point Jacobian in any case
  // pointJacobian J = J_scale_offset * J_dist * J_proj
  Eigen::Matrix<double, 2, 3> & J = *pointJacobian;
  J(0, 0) = fu_ * distortionJacobian(0, 0) * rz;
  J(0, 1) = fu_ * distortionJacobian(0, 1) * rz;
  J(0, 2) = -fu_
      * (point[0] * distortionJacobian(0, 0)
          + point[1] * distortionJacobian(0, 1)) * rz2;
  J(1, 0) = fv_ * distortionJacobian(1, 0) * rz;
  J(1, 1) = fv_ * distortionJacobian(1, 1) * rz;
  J(1, 2) = -fv_
      * (point[0] * distortionJacobian(1, 0)
          + point[1] * distortionJacobian(1, 1)) * rz2;

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if (point[2] > 0.0) {
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}
template<class DISTORTION_T>
CameraBase::ProjectionStatus
  PinholeCamera<DISTORTION_T>::project(const Eigen::Vector3f & point,
                                       Eigen::Vector2f * imagePoint,
                                       Eigen::Matrix<float, 2, 3> * pointJacobian,
                                       Eigen::Matrix2Xf * intrinsicsJacobian,
                                       Eigen::Matrix2f* distortJacobianPtr) const {
  // handle singularity
  if (fabs(point[2]) < 1e-6) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    pointJacobian->setZero();
    return CameraBase::ProjectionStatus::Invalid;
  }

  // projection
  Eigen::Vector2f imagePointUndistorted;
  const float rz = 1.0 / point[2];
  float rz2 = rz * rz;
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;
  Eigen::Matrix<float, 2, 3> pointJacobianProjection;
  Eigen::Matrix2Xf intrinsicsJacobianProjection;
  Eigen::Matrix2f distortionJacobian = Eigen::Matrix2f::Zero();
  Eigen::Matrix2Xf intrinsicsJacobianDistortion;
  Eigen::Vector2f imagePoint2(0.f, 0.f);

  bool distortionSuccess;
  if (intrinsicsJacobian) {
    // get both Jacobians
    intrinsicsJacobian->resize(2, NumIntrinsics);
    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian,
                                            &intrinsicsJacobianDistortion);
    // compute the intrinsics Jacobian
    intrinsicsJacobian->template topLeftCorner<2, 2>() = imagePoint2.asDiagonal();
    intrinsicsJacobian->template block<2, 2>(0, 2) = Eigen::Matrix2f::Identity();

    if (distortion_t::NumDistortionIntrinsics > 0) {
      intrinsicsJacobian
          ->template bottomRightCorner<2, distortion_t::NumDistortionIntrinsics>() =
          Eigen::Vector2f(fu_, fv_).asDiagonal() * intrinsicsJacobianDistortion;  // chain rule
    }
  } else {
    // only get point Jacobian
    distortionSuccess = distortion_.distort(imagePointUndistorted, &imagePoint2,
                                            &distortionJacobian);
  }

  if (distortJacobianPtr) {
    *distortJacobianPtr = distortionJacobian;
  }

  if (!distortionSuccess) {
    (*imagePoint)[0] = cu_;
    (*imagePoint)[1] = cv_;
    pointJacobian->setZero();
    return CameraBase::ProjectionStatus::Invalid;
  }

  // compute the point Jacobian in any case
  // pointJacobian J = J_scale_offset * J_dist * J_proj
  Eigen::Matrix<float, 2, 3> & J = *pointJacobian;
  J(0, 0) = fu_ * distortionJacobian(0, 0) * rz;
  J(0, 1) = fu_ * distortionJacobian(0, 1) * rz;
  J(0, 2) = -fu_
      * (point[0] * distortionJacobian(0, 0)
         + point[1] * distortionJacobian(0, 1)) * rz2;
  J(1, 0) = fv_ * distortionJacobian(1, 0) * rz;
  J(1, 1) = fv_ * distortionJacobian(1, 1) * rz;
  J(1, 2) = -fv_
      * (point[0] * distortionJacobian(1, 0)
         + point[1] * distortionJacobian(1, 1)) * rz2;

  // scale and offset
  (*imagePoint)[0] = fu_ * imagePoint2[0] + cu_;
  (*imagePoint)[1] = fv_ * imagePoint2[1] + cv_;

  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if (point[2] > 0.0) {
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

// Projects a Euclidean point to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectWithExternalParameters(
    const Eigen::Vector3d & point, const Eigen::VectorXd & parameters,
    Eigen::Vector2d * imagePoint, Eigen::Matrix<double, 2, 3> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const {
  // handle singularity
  if (fabs(point[2]) < 1e-6) {
    return CameraBase::ProjectionStatus::Invalid;
  }

  // parse parameters into human readable form
  const double fu = parameters[0];
  const double fv = parameters[1];
  const double cu = parameters[2];
  const double cv = parameters[3];

  Eigen::VectorXd distortionParameters;
  if (distortion_t::NumDistortionIntrinsics > 0) {
    distortionParameters = parameters
        .template tail<distortion_t::NumDistortionIntrinsics>();
  }

  // projection
  Eigen::Vector2d imagePointUndistorted;
  const double rz = 1.0 / point[2];
  double rz2 = rz * rz;
  imagePointUndistorted[0] = point[0] * rz;
  imagePointUndistorted[1] = point[1] * rz;

  Eigen::Matrix<double, 2, 3> pointJacobianProjection;
  Eigen::Matrix2Xd intrinsicsJacobianProjection;
  Eigen::Matrix2d distortionJacobian;
  Eigen::Matrix2Xd intrinsicsJacobianDistortion;
  Eigen::Vector2d imagePoint2;

  bool distortionSuccess;
  if (intrinsicsJacobian) {
    // get both Jacobians
    intrinsicsJacobian->resize(2, NumIntrinsics);

    distortionSuccess = distortion_.distortWithExternalParameters(imagePointUndistorted,
                                            distortionParameters, &imagePoint2,
                                            &distortionJacobian,
                                            &intrinsicsJacobianDistortion);
    // compute the intrinsics Jacobian
    intrinsicsJacobian->template topLeftCorner<2, 2>() =
        imagePoint2.asDiagonal();
    intrinsicsJacobian->template block<2, 2>(0, 2) = Eigen::Matrix2d::Identity();

    if (distortion_t::NumDistortionIntrinsics > 0) {
      intrinsicsJacobian
          ->template bottomRightCorner<2, distortion_t::NumDistortionIntrinsics>() =
          Eigen::Vector2d(fu, fv).asDiagonal() * intrinsicsJacobianDistortion;  // chain rule
    }
  } else {
    // only get point Jacobian
    distortionSuccess = distortion_.distortWithExternalParameters(imagePointUndistorted,
                                            distortionParameters, &imagePoint2,
                                            &distortionJacobian);
  }

  // compute the point Jacobian, if requested
  if (pointJacobian) {
    Eigen::Matrix<double, 2, 3> & J = *pointJacobian;
    J(0, 0) = fu * distortionJacobian(0, 0) * rz;
    J(0, 1) = fu * distortionJacobian(0, 1) * rz;
    J(0, 2) = -fu
        * (point[0] * distortionJacobian(0, 0)
            + point[1] * distortionJacobian(0, 1)) * rz2;
    J(1, 0) = fv * distortionJacobian(1, 0) * rz;
    J(1, 1) = fv * distortionJacobian(1, 1) * rz;
    J(1, 2) = -fv
        * (point[0] * distortionJacobian(1, 0)
            + point[1] * distortionJacobian(1, 1)) * rz2;
  }

  // scale and offset
  (*imagePoint)[0] = fu * imagePoint2[0] + cu;
  (*imagePoint)[1] = fv * imagePoint2[1] + cv;

  if (!distortionSuccess) {
    return CameraBase::ProjectionStatus::Invalid;
  }
  if (!CameraBase::isInImage(*imagePoint)) {
    return CameraBase::ProjectionStatus::OutsideImage;
  }
  if (CameraBase::isMasked(*imagePoint)) {
    return CameraBase::ProjectionStatus::Masked;
  }
  if (point[2] > 0.0) {
    return CameraBase::ProjectionStatus::Successful;
  } else {
    return CameraBase::ProjectionStatus::Behind;
  }
}

// Projects Euclidean points to 2d image points (projection) in a batch.
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::projectBatch(
    const Eigen::Matrix3Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<CameraBase::ProjectionStatus> * stati) const {
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector3d point = points.col(i);
    Eigen::Vector2d imagePoint;
    CameraBase::ProjectionStatus status = project(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if (stati) {
      stati->push_back(status);
    }
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint) const {
  if (point[3] < 0) {
    return this->project(-point.head<3>(), imagePoint);
  } else {
    return this->project(point.head<3>(), imagePoint);
  }
}
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
  const Eigen::Vector4f & point, Eigen::Vector2f * imagePoint) const {
  if (point[3] < 0) {
    return this->project(-point.head<3>(), imagePoint);
  } else {
    return this->project(point.head<3>(), imagePoint);
  }
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
    const Eigen::Vector4d & point, Eigen::Vector2d * imagePoint,
    Eigen::Matrix<double, 2, 4> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const {
  Eigen::Vector3d head = point.head<3>();
  Eigen::Matrix<double, 2, 3> pointJacobian3;
  CameraBase::ProjectionStatus status;
  if (point[3] < 0) {
    status = project(-head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  } else {
    status = project(head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneous(
    const Eigen::Vector4f & point, Eigen::Vector2f * imagePoint,
    Eigen::Matrix<float, 2, 4> * pointJacobian,
    Eigen::Matrix2Xf * intrinsicsJacobian) const {
  Eigen::Vector3f head = point.head<3>();
  Eigen::Matrix<float, 2, 3> pointJacobian3;
  CameraBase::ProjectionStatus status;
  if (point[3] < 0) {
    status = project(-head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  } else {
    status = project(head, imagePoint, &pointJacobian3, intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2f::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}

// Projects a point in homogenous coordinates to a 2d image point (projection).
template<class DISTORTION_T>
CameraBase::ProjectionStatus PinholeCamera<DISTORTION_T>::projectHomogeneousWithExternalParameters(
    const Eigen::Vector4d & point, const Eigen::VectorXd & parameters,
    Eigen::Vector2d * imagePoint, Eigen::Matrix<double, 2, 4> * pointJacobian,
    Eigen::Matrix2Xd * intrinsicsJacobian) const {
  Eigen::Vector3d head = point.head<3>();
  Eigen::Matrix<double, 2, 3> pointJacobian3;
  CameraBase::ProjectionStatus status;
  if (point[3] < 0) {
    status = projectWithExternalParameters(-head, parameters, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  } else {
    status = projectWithExternalParameters(head, parameters, imagePoint,
                                                  &pointJacobian3,
                                                  intrinsicsJacobian);
  }
  pointJacobian->template bottomRightCorner<2, 1>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<2, 3>() = pointJacobian3;
  return status;
}

// Projects points in homogenous coordinates to 2d image points (projection) in a batch.
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::projectHomogeneousBatch(
    const Eigen::Matrix4Xd & points, Eigen::Matrix2Xd * imagePoints,
    std::vector<ProjectionStatus> * stati) const {
  const int numPoints = points.cols();
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector4d point = points.col(i);
    Eigen::Vector2d imagePoint;
    CameraBase::ProjectionStatus status = projectHomogeneous(point, &imagePoint);
    imagePoints->col(i) = imagePoint;
    if (stati) {
      stati->push_back(status);
    }
  }
}

//////////////////////////////////////////
// Methods to backproject points

// Back-project a 2d image point into Euclidean space (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction) const {
  // unscale and center
  Eigen::Vector2d imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;

  // undistort
  Eigen::Vector2d undistortedImagePoint = Eigen::Vector2d::Zero();
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  return success;
}

#ifdef __ARM_NEON__
/*******************************************************************************
* Back-project 4 2d image points into Euclidean space simultaneously.
* [In]:    imagePoints -> the coordinates of 4 image points, size: 8 floats
*          u0, v0, u1, v1, u2, v2, u3, v3
* [Out]:   direction -> the ray direction of 4 image points, size: 12 floats
*          x0, y0, 1, x1, y1, 1, x2, y2, 1, x3, y3, 1
* [Out]:   status[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
* NO RETURNS
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProject4f(
    const float* imagePoints, float* direction, unsigned int * status) const {
  union {
    float dist_pts[8] = {0.f};  // u0, u1, u2, u3, v0, v1, v2, v3
    float32x4x2_t imagePoint_uv;
  };

  imagePoint_uv = vld2q_f32(imagePoints);

  // unscale and center
  imagePoint_uv.val[0] = vsubq_f32(imagePoint_uv.val[0], vmovq_n_f32(static_cast<float>(cu_)));
  imagePoint_uv.val[1] = vsubq_f32(imagePoint_uv.val[1], vmovq_n_f32(static_cast<float>(cv_)));
  imagePoint_uv.val[0] = vmulq_n_f32(imagePoint_uv.val[0], static_cast<float>(one_over_fu_));
  imagePoint_uv.val[1] = vmulq_n_f32(imagePoint_uv.val[1], static_cast<float>(one_over_fv_));

  union {
    float undist_pts[12] = {
      0.f, 0.f, 0.f, 0.f,
      0.f, 0.f, 0.f, 0.f,
      1.f, 1.f, 1.f, 1.f
    };
    float32x4x3_t uv1;
  };

  // undistort
  distortion_.undistort4f(reinterpret_cast<float*>(dist_pts),
                          reinterpret_cast<float*>(undist_pts), status);

  vst3q_f32(direction, uv1);
}
#endif


template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProject(const Eigen::Vector2f & imagePoint,
                                              Eigen::Vector3f * direction) const {
  // unscale and center
  Eigen::Vector2f imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;
  // undistort
  Eigen::Vector2f undistortedImagePoint = Eigen::Vector2f::Zero();
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  return success;
}

// Back-project a 2d image point into Euclidean space (direction vector).
template<class DISTORTION_T>
inline bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2d & imagePoint, Eigen::Vector3d * direction,
    Eigen::Matrix<double, 3, 2> * pointJacobian) const {
  // unscale and center
  Eigen::Vector2d imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;

  // undistort
  Eigen::Vector2d undistortedImagePoint = Eigen::Vector2d::Zero();
  Eigen::Matrix2d pointJacobianUndistortion = Eigen::Matrix2d::Zero();
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint,
                                       &pointJacobianUndistortion);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  // TODO(mingyu): The math here seems wrong (see the float version below)
  // Jacobian w.r.t. imagePoint
  Eigen::Matrix<double, 3, 2> outProjectJacobian =
      Eigen::Matrix<double, 3, 2>::Zero();
  outProjectJacobian(0, 0) = one_over_fu_;
  outProjectJacobian(1, 1) = one_over_fv_;

  (*pointJacobian) = outProjectJacobian * pointJacobianUndistortion;  // chain rule

  return success;
}

#ifdef __ARM_NEON__
/*******************************************************************************
* Back-project 4 2d image points into Euclidean space simultaneously.
* [In]:    imagePoints -> the coordinates of 4 image points, size: 8 floats
*          u0, v0, u1, v1, u2, v2, u3, v3
* [Out]:   direction -> the ray direction of 4 image points, size: 12 floats
*          x0, y0, 1, x1, y1, 1, x2, y2, 1, x3, y3, 1
* [Out]:   pixelJacobian, the jacobian of 4 points, size: 16 floats
*          j0(0, 0) j0(0, 1) j0(1, 0) j0(1, 1)
*          j1(0, 0) j1(0, 1) j1(1, 0) j1(1, 1)
*          j2(0, 0) j2(0, 1) j2(1, 0) j2(1, 1)
*          j3(0, 0) j3(0, 1) j3(1, 0) j3(1, 1)
*          This can be intrepreted as a Eigen::Matrix<float, 8, 2, RowMajor> as
*          [ J0(0,0) J0(0,1)
*            J0(1,0) J0(1,1)
*            J1(0,0) J1(0,1)
*            J1(1,0) J1(1,1)
*            J2(0,0) J2(0,1)
*            J2(1,0) J2(1,1)
*            J3(0,0) J3(0,1)
*            J3(1,0) J3(1,1) ]
* [Out]:   status[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
* NO RETURNS.
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProject4f(const float* imagePoints,
  float* direction, float* pixelJacobian, unsigned int status[4]) const {
  union {
    float dist_pts[8] = {0.f};  // u0, u1, u2, u3, v0, v1, v2, v3
    float32x4x2_t imagePoint_uv;
  };

  imagePoint_uv = vld2q_f32(imagePoints);

  // unscale and center
  imagePoint_uv.val[0] = vsubq_f32(imagePoint_uv.val[0], vmovq_n_f32(static_cast<float>(cu_)));
  imagePoint_uv.val[1] = vsubq_f32(imagePoint_uv.val[1], vmovq_n_f32(static_cast<float>(cv_)));
  imagePoint_uv.val[0] = vmulq_n_f32(imagePoint_uv.val[0], static_cast<float>(one_over_fu_));
  imagePoint_uv.val[1] = vmulq_n_f32(imagePoint_uv.val[1], static_cast<float>(one_over_fv_));

  // u0, u1, u2, u3, v0, v1, v2, v3
  union {
    float undist_pts[12] = {
      0.f, 0.f, 0.f, 0.f,
      0.f, 0.f, 0.f, 0.f,
      1.f, 1.f, 1.f, 1.f
    };
    float32x4x3_t uv1;
  };
  /* ray_J data layout (row-major)
    j0(0, 0) j1(0, 0) j2(0, 0) j3(0, 0)
    j0(0, 1) j1(0, 1) j2(0, 1) j3(0, 1)
    j0(1, 0) j1(1, 0) j2(1, 0) j3(1, 0)
    j0(1, 1) j1(1, 1) j2(1, 1) j3(1, 1)
  */
  union {
    float ray_J[16] = {0.f};
    float32x4x4_t vray_J;
  };

  // undistort
  distortion_.undistort4f(reinterpret_cast<float*>(dist_pts),
                          reinterpret_cast<float*>(undist_pts),
                          reinterpret_cast<float*>(ray_J), status);

  vst3q_f32(direction, uv1);

  // gcc branch prediction helper is added here
  // if backProject4f is not asked to ouput the Jacobian
  // Please call the other version of backProject4f
  if (__builtin_expect(pixelJacobian != NULL, 1)) {
    vray_J.val[0] = vmulq_n_f32(vray_J.val[0], one_over_fu_);
    vray_J.val[1] = vmulq_n_f32(vray_J.val[1], one_over_fv_);
    vray_J.val[2] = vmulq_n_f32(vray_J.val[2], one_over_fu_);
    vray_J.val[3] = vmulq_n_f32(vray_J.val[3], one_over_fv_);
    vst4q_f32(pixelJacobian, vray_J);
  }
}
#endif  // __ARM_NEON__

// Back-project a 2D image point into Euclidean space (direction vector)
template<class DISTORTION_T>
inline bool PinholeCamera<DISTORTION_T>::backProject(
    const Eigen::Vector2f& imagePoint, Eigen::Vector3f* direction,
    Eigen::Matrix<float, 3, 2>* pixelJacobian) const {
  // unscale and center
  Eigen::Vector2f imagePoint2;
  imagePoint2[0] = (imagePoint[0] - cu_) * one_over_fu_;
  imagePoint2[1] = (imagePoint[1] - cv_) * one_over_fv_;

  // undistort
  Eigen::Vector2f undistortedImagePoint = Eigen::Vector2f::Zero();
  Eigen::Matrix2f ray_J = Eigen::Matrix2f::Zero();
  bool success = distortion_.undistort(imagePoint2, &undistortedImagePoint, &ray_J);

  // project 1 into z direction
  (*direction)[0] = undistortedImagePoint[0];
  (*direction)[1] = undistortedImagePoint[1];
  (*direction)[2] = 1.0;

  // Jacobian w.r.t. imagePoint (distorted pixel)
  if (pixelJacobian != nullptr) {
    Eigen::Matrix<float, 3, 2>& pixel_J = *pixelJacobian;
    pixel_J(0, 0) = ray_J(0, 0) * one_over_fu_;
    pixel_J(0, 1) = ray_J(0, 1) * one_over_fv_;
    pixel_J(1, 0) = ray_J(1, 0) * one_over_fu_;
    pixel_J(1, 1) = ray_J(1, 1) * one_over_fv_;
    pixel_J(2, 0) = 0;
    pixel_J(2, 1) = 0;
  }
  return success;
}

#ifdef __ARM_NEON__
/************************************************************************************
* Back-project any 2d image points into Euclidean space at one call.
* [In]:    imagePoints -> the coordinate of image points, MUST BE COLUMN-MAJOR
           for safer, imagePoints.cols() % 4 MUST BE EQUAL to 0
* [Out]:   direction -> the ray direction of the input image points, COLUMN-MAJOR
           one column is a ray direction of a point
* [Out]:   success[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
* NO RETURNS
* NOTE:    directions->cols() % 4 MUST BE equal to 0, or this function at last will
           store data to an invalid memory location, which may be a problem.
           The same reason is also applied to success output.
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProjectBatchNeon(
  const Eigen::Matrix2Xf& imagePoints,
  Eigen::Matrix3Xf* directions,
  unsigned int* success) const {
  int points = imagePoints.cols();

  if ((points == 0)
      || (directions == NULL)
      || (success == NULL)) {
    return;
  }
  CHECK(directions->cols() == (((points & 3) != 0) ? ((points / 4 + 1) * 4) : points));

  float* src_data_ptr = const_cast<float*>(imagePoints.data());
  float* dst_data_ptr = directions->data();
  unsigned int* sts_ptr = success;
  points = points >> 2;

  do {
    // gcc built-in intrinsics for decreasing cache-miss latency
    // the second parameter: 0 for read, 1, for write
    // the third parameter: 0 ~ 3
    // 3 indicates the data prefetched has high degree of temporal locality
    // 0 indicates the data prefetched has no temporal locality
    __builtin_prefetch(dst_data_ptr, 1, 0);
    // back project 4 image points, no Jacobian output is required
    this->backProject4f(reinterpret_cast<float*>(src_data_ptr),
                        reinterpret_cast<float*>(dst_data_ptr), sts_ptr);

    src_data_ptr += 8;
    dst_data_ptr += 12;
    sts_ptr += 4;
  } while (--points);
}
#endif  // __ARM_NEON__

// Back-project 2d image points into Euclidean space (direction vectors).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix3Xd * directions,
    std::vector<bool> * success) const {
  const int numPoints = imagePoints.cols();
  CHECK_NOTNULL(success);
  success->clear();
  success->reserve(numPoints);

  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    success->push_back(suc);
    directions->col(i) = point;
  }
  return true;
}

template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectBatch(
    const Eigen::Matrix2Xf& imagePoints, Eigen::Matrix3Xf* directions,
    std::vector<bool>* success) const {
  const size_t numPoints = imagePoints.cols();
  CHECK_NOTNULL(success);
  success->clear();
  success->reserve(numPoints);
  size_t finished_counter = 0;
  // Do not use iterator as the counter since iterator may run over numPoints
#ifdef __ARM_NEON__
  // Leverage NEON for the first 4*n image points
  // Eigen::Matrix default is ColMajor
  size_t neon_loops = numPoints >> 2;
  unsigned int status[4];  // 0 means success!
  float* dist_pts4_ptr = const_cast<float*>(imagePoints.data());
  float* undist_rays4_ptr = directions->data();

  while (neon_loops--) {
    // gcc built-in intrinsics for decreasing cache-miss latency
    // the second parameter: 0 for read, 1, for write
    // the third parameter: 0 ~ 3
    // 3 indicates the data prefetched has high degree of temporal locality
    // 0 indicates the data prefetched has no temporal locality
    __builtin_prefetch(undist_rays4_ptr, 1, 0);

    backProject4f(reinterpret_cast<float*>(dist_pts4_ptr),
                  reinterpret_cast<float*>(undist_rays4_ptr), status);
    success->push_back(status[0] == 0);
    success->push_back(status[1] == 0);
    success->push_back(status[2] == 0);
    success->push_back(status[3] == 0);
    dist_pts4_ptr += 8;
    undist_rays4_ptr += 12;
    finished_counter += 4;
  }
#endif  // __ARM_NEON__
  for (; finished_counter < numPoints; ++finished_counter) {
    Eigen::Vector3f undist_ray = Eigen::Vector3f::Zero();
    bool succ = backProject(imagePoints.col(finished_counter), &undist_ray);
    success->push_back(succ);
    directions->col(finished_counter) = undist_ray;
  }
  CHECK_EQ(success->size(), numPoints);
  return true;
}

template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectBatch(
    const Eigen::Matrix2Xf & imagePoints,
    Eigen::Matrix3Xf* directions,
    Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>* pixelJacobians,
    std::vector<bool> * success) const {
  const size_t numPoints = imagePoints.cols();
  CHECK_NOTNULL(success);
  success->clear();
  success->reserve(numPoints);
  size_t finished_counter = 0;
  // Do not use iterator as the counter since iterator may run over numPoints
#ifdef __ARM_NEON__
  // Leverage NEON for the first 4*n image points
  // Eigen::Matrix default is ColMajor
  size_t neon_loops = numPoints >> 2;
  unsigned int status[4];  // 0 means success!
  float* dist_pts4_ptr = const_cast<float*>(imagePoints.data());
  float* undist_rays4_ptr = directions->data();
  float* pixel_jac4_ptr = pixelJacobians->data();

  while (neon_loops--) {
    // gcc built-in intrinsics for decreasing cache-miss latency
    // the second parameter: 0 for read, 1 for write
    // the third parameter: 0 ~ 3
    // 3 indicates the data prefetched has high degree of temporal locality
    // 0 indicates the data prefetched has no temporal locality
    __builtin_prefetch(undist_rays4_ptr, 1, 0);
    __builtin_prefetch(pixel_jac4_ptr, 1, 0);

    backProject4f(reinterpret_cast<float*>(dist_pts4_ptr),
                  reinterpret_cast<float*>(undist_rays4_ptr),
                  reinterpret_cast<float*>(pixel_jac4_ptr), status);
    success->push_back(status[0] == 0);
    success->push_back(status[1] == 0);
    success->push_back(status[2] == 0);
    success->push_back(status[3] == 0);
    dist_pts4_ptr += 8;
    undist_rays4_ptr += 12;
    pixel_jac4_ptr += 16;
    finished_counter += 4;
  }
#endif  // __ARM_NEON__
  for (; finished_counter < numPoints; ++finished_counter) {
    Eigen::Vector3f undist_ray = Eigen::Vector3f::Zero();
    Eigen::Matrix<float, 3, 2> pixel_jac;
    bool succ = backProject(imagePoints.col(finished_counter), &undist_ray, &pixel_jac);
    success->push_back(succ);
    directions->col(finished_counter) = undist_ray;
    pixelJacobians->block<2, 2>(finished_counter * 2, 0) = pixel_jac.topLeftCorner<2, 2>();
  }
  CHECK_EQ(success->size(), numPoints);
  return true;
}

// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction) const {
  Eigen::Vector3d ray;
  bool success = backProject(imagePoint, &ray);
  direction->template head<3>() = ray;
  (*direction)[3] = 1.0;  // arbitrary
  return success;
}

// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2f & imagePoint, Eigen::Vector4f * direction) const {
  Eigen::Vector3f ray;
  bool success = backProject(imagePoint, &ray);
  direction->template head<3>() = ray;
  (*direction)[3] = 1.f;  // arbitrary
  return success;
}

#ifdef __ARM_NEON__
/*******************************************************************************
* Back-project 4 2d image points into Euclidean space simultaneously.
* [In]:    imagePoints -> the coordinates of 4 image points, size: 8 floats
*          u0, v0, u1, v1, u2, v2, u3, v3
* [Out]:   directions -> the ray direction of 4 image points, size: 16 floats
*          x0, y0, z0, 1, x1, y1, z1, 1, x2, y2, z2, 1, x3, y3, z3, 1
* [Out]:   status[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
* NO RETURNS
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProjectHomogeneous4f(
  const float* imagePoints, float* directions, unsigned int status[4]) const {
  union {
    float dist_pts[8] = {0.f};
    float32x4x2_t imagePoint_uv;
  };

  imagePoint_uv = vld2q_f32(imagePoints);

  // unscale and center
  imagePoint_uv.val[0] = vsubq_f32(imagePoint_uv.val[0], vmovq_n_f32(static_cast<float>(cu_)));
  imagePoint_uv.val[1] = vsubq_f32(imagePoint_uv.val[1], vmovq_n_f32(static_cast<float>(cv_)));
  imagePoint_uv.val[0] = vmulq_n_f32(imagePoint_uv.val[0], static_cast<float>(one_over_fu_));
  imagePoint_uv.val[1] = vmulq_n_f32(imagePoint_uv.val[1], static_cast<float>(one_over_fv_));

  union {
    float undist_pts[16] = {
      0.f, 0.f, 0.f, 0.f,
      0.f, 0.f, 0.f, 0.f,
      1.f, 1.f, 1.f, 1.f,
      1.f, 1.f, 1.f, 1.f
    };
    float32x4x4_t xyz1;
  };

  // undistort
  distortion_.undistort4f(reinterpret_cast<float*>(dist_pts),
                          reinterpret_cast<float*>(undist_pts), status);

  vst4q_f32(directions, xyz1);
}
#endif


// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2d & imagePoint, Eigen::Vector4d * direction,
    Eigen::Matrix<double, 4, 2> * pointJacobian) const {
  Eigen::Vector3d ray;
  Eigen::Matrix<double, 3, 2> pointJacobian3;
  bool success = backProject(imagePoint, &ray, &pointJacobian3);
  direction->template head<3>() = ray;
  (*direction)[3] = 1.0;  // arbitrary
  pointJacobian->template bottomRightCorner<1, 2>() = Eigen::Vector2d::Zero();
  pointJacobian->template topLeftCorner<3, 2>() = pointJacobian3;
  return success;
}

// Back-project a 2d image point into homogeneous point (direction vector).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneous(
    const Eigen::Vector2f & imagePoint, Eigen::Vector4f * direction,
    Eigen::Matrix<float, 4, 2> * pointJacobian) const {
  Eigen::Vector3f ray;
  Eigen::Matrix<float, 3, 2> pointJacobian3;
  bool success = backProject(imagePoint, &ray, &pointJacobian3);
  direction->template head<3>() = ray;
  (*direction)[3] = 1.f;  // arbitrary
  pointJacobian->template bottomRightCorner<1, 2>() = Eigen::Vector2f::Zero();
  pointJacobian->template topLeftCorner<3, 2>() = pointJacobian3;
  return success;
}

#ifdef __ARM_NEON__
/*******************************************************************************
* Back-project 4 2d image points into Euclidean space simultaneously.
* [In]:    imagePoints -> the coordinates of 4 image points, size: 8 floats
*          u0, v0, u1, v1, u2, v2, u3, v3
* [Out]:   directions -> the ray direction of 4 image points, size: 16 floats
*          x0, y0, z0, 1, x1, y1, z1, 1, x2, y2, z2, 1, x3, y3, z3, 1
* [Out]:   pointJacobian -> the points of jacobian, 16 floats
*          j0(0, 0), j0(0, 1), j0(1, 0), j0(1, 1)
*          j1(0, 0), j1(0, 1), j1(1, 0), j1(1, 1)
*          j2(0, 0), j2(0, 1), j2(1, 0), j2(1, 1)
*          j3(0, 0), j3(0, 1), j3(1, 0), j3(1, 1)
           j0, j1, j2, j3 is the corresponding Jacobian matrix of point 0, 1, 2, 3
* [Out]:   status[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
* NO RETURNS
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProjectHomogeneous4f(
  const float* imagePoints, float* directions,
  float* pointJacobian, unsigned int status[4]) const {
  union {
    float dist_pts[8] = {0.f};
    float32x4x2_t imagePoint_uv;
  };

  imagePoint_uv = vld2q_f32(imagePoints);

  // unscale and center
  imagePoint_uv.val[0] = vsubq_f32(imagePoint_uv.val[0], vmovq_n_f32(static_cast<float>(cu_)));
  imagePoint_uv.val[1] = vsubq_f32(imagePoint_uv.val[1], vmovq_n_f32(static_cast<float>(cv_)));
  imagePoint_uv.val[0] = vmulq_n_f32(imagePoint_uv.val[0], static_cast<float>(one_over_fu_));
  imagePoint_uv.val[1] = vmulq_n_f32(imagePoint_uv.val[1], static_cast<float>(one_over_fv_));

  union {
    float undist_pts[16] = {
      0.f, 0.f, 0.f, 0.f,
      0.f, 0.f, 0.f, 0.f,
      1.f, 1.f, 1.f, 1.f,
      1.f, 1.f, 1.f, 1.f
    };
    float32x4x4_t xyz1;
  };

  /* ray_J data layout (row-major)
    j0(0, 0) j1(0, 0) j2(0, 0) j3(0, 0)
    j0(0, 1) j1(0, 1) j2(0, 1) j3(0, 1)
    j0(1, 0) j1(1, 0) j2(1, 0) j3(1, 0)
    j0(1, 1) j1(1, 1) j2(1, 1) j3(1, 1)
  */
  union {
    float ray_J[16] = {0.f};
    float32x4x4_t vray_J;
  };

  // undistort
  distortion_.undistort4f(reinterpret_cast<float*>(dist_pts),
                          reinterpret_cast<float*>(undist_pts),
                          reinterpret_cast<float*>(ray_J), status);

  vst4q_f32(directions, xyz1);

  // gcc branch prediction helper is added here
  // if backProject4f is not asked to ouput the Jacobian
  // Please call the other version of backProject4f
  if (__builtin_expect(pointJacobian != NULL, 1)) {
    vray_J.val[0] = vmulq_n_f32(vray_J.val[0], one_over_fu_);
    vray_J.val[1] = vmulq_n_f32(vray_J.val[1], one_over_fv_);
    vray_J.val[2] = vmulq_n_f32(vray_J.val[2], one_over_fu_);
    vray_J.val[3] = vmulq_n_f32(vray_J.val[3], one_over_fv_);
    // BUT!! the order will not be the same as E, here is the order of jacobians_out
    // j0(0, 0), j0(0, 1), j0(1, 0), j0(1, 1), j1(0, 0), j1(0, 1), j1(1, 0), j1(1, 1)
    // j2(0, 0), j2(0, 1), j2(1, 0), j2(1, 1), j3(0, 0), j3(0, 1), j3(1, 0), j3(1, 1)
    // j0, j1, j2, j3 is the corresponding Jacobian matrix of point 0, 1, 2, 3
    vst4q_f32(pointJacobian, vray_J);
  }
}
#endif

// Back-project 2d image points into homogeneous points (direction vectors).
template<class DISTORTION_T>
bool PinholeCamera<DISTORTION_T>::backProjectHomogeneousBatch(
    const Eigen::Matrix2Xd & imagePoints, Eigen::Matrix4Xd * directions,
    std::vector<bool> * success) const {
  const int numPoints = imagePoints.cols();
  directions->row(3) = Eigen::VectorXd::Ones(numPoints);
  for (int i = 0; i < numPoints; ++i) {
    Eigen::Vector2d imagePoint = imagePoints.col(i);
    Eigen::Vector3d point;
    bool suc = backProject(imagePoint, &point);
    if (success) success->push_back(suc);
    directions->template block<3, 1>(0, i) = point;
  }
  return true;
}

#ifdef __ARM_NEON__
/************************************************************************************
* Back-project any 2d image points into Euclidean space at one call.
* [In]:    imagePoints -> the coordinate of image points, MUST BE COLUMN-MAJOR
           for safer, imagePoints.cols() % 4 MUST BE EQUAL to 0
* [Out]:   direction -> the ray direction of the input image points, COLUMN-MAJOR
           one column is a ray direction of a point
* [Out]:   success[i] -> 0 if point i is back-projected succesffuly, or 0xffffffff
           MAKE SURE the size of success % 4 == 0
* NO RETURNS
* NOTE:    directions->cols() % 4 MUST BE equal to 0, or this function at last will
           store data to an invalid memory location, which may be a problem.
           The same reason is also applied to success output.
*/
template<class DISTORTION_T>
void PinholeCamera<DISTORTION_T>::backProjectHomogeneousBatchNeon(
    const Eigen::Matrix2Xf & imagePoints, Eigen::Matrix4Xf * directions,
    unsigned int* success) const {
  int points = imagePoints.cols();

  if ((points == 0)
     || (directions == NULL)
     || (success == NULL)) {
    return;
  }
  CHECK(directions->cols() == (((points & 3) != 0) ? ((points / 4 + 1) * 4) : points));
  points = points >> 2;
  float* src_data_ptr = const_cast<float*>(imagePoints.data());
  float* dst_data_ptr = directions->data();
  unsigned int* sts_ptr = success;
  do {
    // gcc built-in intrinsics for decreasing cache-miss latency
    // the second parameter: 0 for read, 1, for write
    // the third parameter: 0 ~ 3
    // 3 indicates the data prefetched has high degree of temporal locality
    // 0 indicates the data prefetched has no temporal locality
    __builtin_prefetch(dst_data_ptr, 1, 0);
    // back project 4 image points, no Jacobian output is required
    this->backProjectHomogeneous4f(reinterpret_cast<float*>(src_data_ptr),
                        reinterpret_cast<float*>(dst_data_ptr), sts_ptr);
    src_data_ptr += 8;
    dst_data_ptr += 16;
    sts_ptr += 4;
  } while (--points);
}
#endif

}  // namespace cameras
}  // namespace vio
