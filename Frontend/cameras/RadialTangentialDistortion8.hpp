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
 * @file cameras/RadialTangentialDistortion8.hpp
 * @brief Header file for the RadialTangentialDistortion class.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_VIO_CAMERAS_RADIALTANGENTIALDISTORTION8_HPP_
#define INCLUDE_VIO_CAMERAS_RADIALTANGENTIALDISTORTION8_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>
#include "DistortionBase.hpp"

/// \brief vio Main namespace of this package.
namespace vio {
/// \brief cameras Namespace for camera-related functionality.
namespace cameras {

class RadialTangentialDistortion8 : public DistortionBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief The default constructor with all zero ki
  inline RadialTangentialDistortion8();

  /// \brief Constructor initialising ki
  /// @param[in] k1 radial parameter 1
  /// @param[in] k2 radial parameter 2
  /// @param[in] p1 tangential parameter 1
  /// @param[in] p2 tangential parameter 2
  /// @param[in] k3 radial parameter 3
  /// @param[in] k4 radial parameter 4
  /// @param[in] k5 radial parameter 5
  /// @param[in] k6 radial parameter 6
  inline RadialTangentialDistortion8(float k1, float k2, float p1, float p2,
                                     float k3, float k4, float k5,
                                     float k6);

  //////////////////////////////////////////////////////////////
  /// \name Methods related to generic parameters
  /// @{

  /// \brief set the generic parameters
  /// @param[in] parameters Parameter vector -- length must correspond numDistortionIntrinsics().
  /// @return    True if the requirements were followed.
  inline bool setParameters(const Eigen::VectorXd & parameters) override;

  /// \brief Obtain the generic parameters.
  inline bool getParameters(Eigen::VectorXd & parameters) const override {
    parameters = parameters_.cast<double>();
    return true;
  }

  /// \brief The class type.
  inline std::string type() const override {
    return "RadialTangentialDistortion8";
  }

  /// \brief Number of distortion parameters
  inline int numDistortionIntrinsics() const override {
    return NumDistortionIntrinsics;
  }

  static const int NumDistortionIntrinsics = 8;  ///< The Number of distortion parameters.
  /// @}

  /// \brief Unit test support -- create a test distortion object
  /// The distortion coefficients are copied from DUO MLX
  static std::shared_ptr<DistortionBase> createTestObject() {
    return std::shared_ptr<DistortionBase>(
        new RadialTangentialDistortion8(2.710e+01, 5.254e+01, -1.800e-03, -4.265e-04,
                                        5.976e+00, 2.751e+01, 6.474e+01, 2.519e+01));
  }
  /// \brief Unit test support -- create a test distortion object
  /// The distortion coefficients are copied from DUO MLX
  static RadialTangentialDistortion8 testObject() {
    return RadialTangentialDistortion8(2.710e+01, 5.254e+01, -1.800e-03, -4.265e-04,
                                       5.976e+00, 2.751e+01, 6.474e+01, 2.519e+01);
  }

  //////////////////////////////////////////////////////////////
  /// \name Distortion functions
  /// @{

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  template<typename T> inline bool distortT(const Eigen::Matrix<T, 2, 1> & pointUndistorted,
                                            Eigen::Matrix<T, 2, 1> * pointDistorted) const;
#ifdef __ARM_NEON__
  inline void distortT4f(const float* pointUndistorted,
    float* pointDistorted, unsigned int status[4]) const;

  inline void distortT4f(const float* pointUndistorted, float* pointDistorted,
    float* pointJacobian, unsigned int status[4]) const;
  void distort4f(const float* uv_undist4f, float* uv_dist4f,
                 unsigned int status[4]) const override {
    this->distortT4f(uv_undist4f, uv_dist4f, status);
  }
#endif  // __ARM_NEON__
  inline bool distort(const Eigen::Vector2d & pointUndistorted,
                      Eigen::Vector2d * pointDistorted) const override {
    return this->distortT<double>(pointUndistorted, pointDistorted);
  };

  /// \brief Distortion only
  /// @param[in]  pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointDistorted   The distorted normalised (!) image point.
  /// @return     True on success (no singularity)
  inline bool distort(const Eigen::Vector2f & pointUndistorted,
                      Eigen::Vector2f * pointDistorted) const override {
    // this shows much better result than using lookup table
    return this->distortT<float>(pointUndistorted, pointDistorted);
  };

  /// \brief Distortion and Jacobians (the fast version)
  ///        Computing parameterJacobian is currently NOT supported!
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the intrinsics vector.
  /// @return     True on success (no singularity)
  template<typename T> inline bool distortT(
      const Eigen::Matrix<T, 2, 1> & pointUndistorted,
      Eigen::Matrix<T, 2, 1>  * pointDistorted,
      Eigen::Matrix<T, 2, 2> * pointJacobian,
      Eigen::Matrix<T, 2, Eigen::Dynamic> * parameterJacobian = NULL) const;

  /// \brief Distortion and Jacobians (the slow version)
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the intrinsics vector.
  /// @return     True on success (no singularity)
  template<typename T> inline bool distortT_slow(
      const Eigen::Matrix<T, 2, 1> & pointUndistorted,
      Eigen::Matrix<T, 2, 1>  * pointDistorted,
      Eigen::Matrix<T, 2, 2> * pointJacobian,
      Eigen::Matrix<T, 2, Eigen::Dynamic> * parameterJacobian = NULL) const;

  inline bool distort(const Eigen::Vector2d& pointUndistorted,
                      Eigen::Vector2d* pointDistorted,
                      Eigen::Matrix2d* pointJacobian,
                      Eigen::Matrix2Xd* parameterJacobian = NULL) const override {
    return this->distortT<double>(pointUndistorted, pointDistorted,
                                  pointJacobian, parameterJacobian);
  }

  /// \brief Distortion and Jacobians.
  inline bool distort(const Eigen::Vector2f& pointUndistorted,
                      Eigen::Vector2f* pointDistorted,
                      Eigen::Matrix2f* pointJacobian,
                      Eigen::Matrix2Xf* parameterJacobian = NULL) const override {
    return this->distortT<float>(pointUndistorted, pointDistorted,
                                 pointJacobian, parameterJacobian);
  }

  /// \brief Distortion and Jacobians using external distortion intrinsics parameters.
  /// @param[in]  pointUndistorted  The undistorted normalised (!) image point.
  /// @param[in]  parameters        The distortion intrinsics vector.
  /// @param[out] pointDistorted    The distorted normalised (!) image point.
  /// @param[out] pointJacobian     The Jacobian w.r.t. changes on the image point.
  /// @param[out] parameterJacobian The Jacobian w.r.t. changes on the intrinsics vector.
  /// @return     True on success (no singularity)
  inline bool distortWithExternalParameters(
      const Eigen::Vector2d & pointUndistorted,
      const Eigen::VectorXd & parameters, Eigen::Vector2d * pointDistorted,
      Eigen::Matrix2d * pointJacobian = NULL,
      Eigen::Matrix2Xd * parameterJacobian = NULL) const override;
  /// @}

  //////////////////////////////////////////////////////////////
  /// \name Undistortion functions
  /// @{

  /// \brief Undistortion only (use look up table)
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @return     True on success (no singularity)
  inline bool undistort(const Eigen::Vector2d & pointDistorted,
                        Eigen::Vector2d * pointUndistorted) const override;
  inline bool undistort(const Eigen::Vector2f & pointDistorted,
                        Eigen::Vector2f * pointUndistorted) const override;

  /// \brief Undistortion only (use undistort Gauss Newton)
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointJacobian    The Jacobian w.r.t. changes on the image point.
  /// @return     True on success (no singularity)
  inline bool undistort(const Eigen::Vector2d & pointDistorted,
                        Eigen::Vector2d * pointUndistorted,
                        Eigen::Matrix2d * pointJacobian) const override;

  /// \brief Undistortion: use look up table for pointUndistorted & pointJacobian
  /// @param[in]  pointDistorted   The distorted normalised (!) image point.
  /// @param[out] pointUndistorted The undistorted normalised (!) image point.
  /// @param[out] pointJacobian    The Jacobian w.r.t. changes on the image point.
  /// @return     True on success (no singularity)
  inline bool undistort(const Eigen::Vector2f & pointDistorted,
                        Eigen::Vector2f * pointUndistorted,
                        Eigen::Matrix2f * pointJacobian) const override;
  /// @}
#ifdef __ARM_NEON__
  inline void undistort4f(const float* uv_dist4f, float* uv_undist4f,
                          unsigned int status[4]) const override;
  inline void undistort4f(const float* uv_dist4f, float* uv_undist4f,
                          float* jacobians_out, unsigned int status[4]) const override;
#endif

 protected:
  // Gauss-Newton iterative solution for undistortion
  inline bool undistort_GN(float u_dist, float v_dist, float* u_undist, float* v_undist,
                           Eigen::Matrix2f * pointJacobian = nullptr) const;
  Eigen::Matrix<float, NumDistortionIntrinsics, 1> parameters_;  ///< all distortion parameters

  float k1_;  ///< radial parameter 1
  float k2_;  ///< radial parameter 2
  float p1_;  ///< tangential parameter 1
  float p2_;  ///< tangential parameter 2
  float k3_;  ///< radial parameter 3
  float k4_;  ///< radial parameter 4
  float k5_;  ///< radial parameter 5
  float k6_;  ///< radial parameter 6
};

}  // namespace cameras
}  // namespace vio

#include "implementation/RadialTangentialDistortion8.hpp"

#endif /* INCLUDE_VIO_CAMERAS_RADIALTANGENTIALDISTORTION8_HPP_ */
