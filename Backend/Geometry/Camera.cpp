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
#include "stdafx.h"
#include "Camera.h"

#ifdef CFG_DEBUG_EIGEN
#if 0
#define CAMERA_PRIOR_RIGID_EIGEN_DEBUG_JACOBIAN
Camera::Prior::Rigid::EigenErrorJacobian Camera::Prior::Rigid::EigenGetErrorJacobian(
  const Rigid3D &T1, const Rigid3D &T2) const {
  const EigenRotation3D e_Rp = EigenMatrix3x3f(m_R), e_RpT = EigenRotation3D(e_Rp.transpose());
  const EigenPoint3D e_pp = EigenPoint3D(m_p);
  const EigenRotation3D e_R1 = EigenRotation3D(T1), e_R2 = EigenRotation3D(T2);
  const EigenRotation3D e_R1T = EigenRotation3D(e_R1.transpose());
  const EigenPoint3D e_p1 = EigenVector3f(T1.GetPosition()), e_p2 = EigenVector3f(T2.GetPosition());
  const EigenRotation3D e_R12 = EigenRotation3D(e_R2 * e_R1T);
  const EigenRotation3D e_eR = EigenRotation3D(e_R12 * e_RpT);
  const EigenVector3f e_er = e_eR.GetRodrigues();
  const EigenMatrix3x3f e_JrI = EigenRotation3D::GetRodriguesJacobianInverse(e_er);
  const EigenMatrix3x3f e_Jrr2 = EigenMatrix3x3f(e_JrI * e_R2);
  const EigenMatrix3x3f e_Jrr1 = EigenMatrix3x3f(-e_Jrr2);
  EigenVector3f e_p12 = EigenVector3f(e_p2 - e_p1);
  const EigenMatrix3x3f e_Jpr1 = EigenMatrix3x3f(e_R1 * EigenSkewSymmetricMatrix(e_p12));
  e_p12 = EigenVector3f(e_R1 * e_p12);
  const EigenVector3f e_ep = EigenVector3f(e_p12 - e_pp);
  const EigenMatrix3x3f e_Jpr1 = EigenSkewSymmetricMatrix(e_p12);
  const EigenMatrix3x3f e_Jpp1 = EigenMatrix3x3f(-e_R1);
  const EigenMatrix3x3f e_Jpp2 = e_R1;
#ifdef CAMERA_PRIOR_RIGID_EIGEN_DEBUG_JACOBIAN
  //const float e_dr1Max = 0.0f;
  const float e_dr1Max = 1.0f;
  //const float e_dr2Max = 0.0f;
  const float e_dr2Max = 1.0f;
  //const float e_dp1Max = 0.0f;
  const float e_dp1Max = 0.1f;
  //const float e_dp2Max = 0.0f;
  const float e_dp2Max = 0.1f;
  //const EigenVector3f e_dr1 = EigenAxisAngle::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD).GetRodrigues();
  //const EigenVector3f e_dr2 = EigenAxisAngle::GetRandom(e_dr2Max * UT_FACTOR_DEG_TO_RAD).GetRodrigues();
  //const EigenRotation3D e_R1GT = EigenMatrix3x3f(EigenRotation3D(e_dr1) * e_R1);
  //const EigenRotation3D e_R2GT = EigenMatrix3x3f(EigenRotation3D(e_dr2) * e_R2);
#if 0
  EigenVector3f e_dr1, e_dr2;
  EigenRotation3D e_R1GT, e_R2GT;
  if (e_dr1Max == 0.0f && e_dr2Max == 0.0f) {
    e_dr1.setZero();
    e_R1GT = e_R1;
    e_dr2.setZero();
    e_R2GT = e_R2;
  } else if (e_dr1Max == 0.0f) {
    e_dr1.setZero();
    e_R1GT = e_R1;
    e_R2GT = e_Rd * e_R1GT;
    e_dr2 = EigenRotation3D(e_R2GT * e_R2.transpose()).GetRodrigues();
  } else if (e_dr2Max == 0.0f) {
    e_dr2.setZero();
    e_R2GT = e_R2;
    e_R1GT = e_RdT * e_R2GT;
    e_dr1 = EigenRotation3D(e_R1GT * e_R1T).GetRodrigues();
  } else {
    e_dr1 = EigenAxisAngle::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD).GetRodrigues();
    e_R1GT = EigenMatrix3x3f(EigenRotation3D(e_dr1) * e_R1);
    e_R2GT = e_Rd * e_R1GT;
    e_dr2 = EigenRotation3D(e_R2GT * e_R2.transpose()).GetRodrigues();
  }
#else
  const EigenVector3f e_dr1 = EigenVector3f::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD);
  const EigenVector3f e_dr2 = EigenVector3f::GetRandom(e_dr2Max * UT_FACTOR_DEG_TO_RAD);
  const EigenRotation3D e_R1GT = EigenRotation3D(e_R1 * EigenRotation3D(e_dr1));
  const EigenRotation3D e_R2GT = EigenRotation3D(e_R2 * EigenRotation3D(e_dr2));
#endif
  const EigenVector3f e_dp1 = EigenVector3f::GetRandom(e_dp1Max);
  const EigenVector3f e_dp2 = EigenVector3f::GetRandom(e_dp2Max);
  const EigenPoint3D e_p1GT = EigenVector3f(e_p1 + e_dp1);
  const EigenPoint3D e_p2GT = EigenVector3f(e_p2 + e_dp2);
  const EigenRotation3D e_R12GT = EigenRotation3D(e_R2GT * e_R1GT.transpose());
  const EigenVector3f e_p12GT = EigenVector3f(e_R1GT * (e_p2GT - e_p1GT));
  const EigenRotation3D e_eRGT = EigenRotation3D(e_R12 * e_R12GT.transpose());
  const EigenVector3f e_er1 = EigenVector3f(e_eRGT.GetRodrigues());
  const EigenVector3f e_er2 = EigenVector3f(e_er1 + e_Jrr1 * e_dr1 + e_Jrr2 * e_dr2);
  const EigenVector3f e_ep1 = EigenVector3f(e_p12 - e_p12GT);
  const EigenVector3f e_ep2 = EigenVector3f(e_ep1 + e_Jpr1 * e_dr1 + e_Jpp1 * e_dp1 +
                                                    e_Jpp2 * e_dp2);
  UT::AssertReduction(e_er1, e_er2);
  UT::AssertReduction(e_ep1, e_ep2);
#endif
  EigenErrorJacobian e_Je;
  e_Je.m_J1 = EigenMatrix6x6f(EigenMatrix3x3f::Zero(), e_Jrr1, e_Jpp1, e_Jpr1);
  e_Je.m_J2 = EigenMatrix6x6f(EigenMatrix3x3f::Zero(), e_Jrr2, e_Jpp2, EigenMatrix3x3f::Zero());
  e_Je.m_e = EigenVector6f(e_er, e_ep);
  return e_Je;
}

Camera::Prior::Rigid::EigenFactor Camera::Prior::Rigid::EigenGetFactor(const float w,
                                                                       const Rigid3D &T1, const Rigid3D &T2) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(T1, T2);
  const EigenMatrix6x12f e_J12 = EigenMatrix6x12f(e_Je.m_J1, e_Je.m_J2);
  const EigenMatrix6x13f e_Je12 = EigenMatrix6x13f(e_J12, e_Je.m_e);
  const EigenMatrix6x6f e_W = EigenMatrix6x6f(w * EigenMatrix6x6f(m_Wrr, m_Wrp, m_Wpr, m_Wpp));
  const EigenMatrix6x12f e_WJ12 = EigenMatrix6x12f(e_W * e_J12);
  const float e_F = (e_W * e_Je.m_e).dot(e_Je.m_e);
  return Camera::Prior::Rigid::EigenFactor(e_WJ12.transpose() * e_Je12, e_F);
}

float Camera::Prior::Rigid::EigenGetCost(const float w, const Rigid3D &T1, const Rigid3D &T2,
                                         const EigenVector6f &e_x1, const EigenVector6f &e_x2) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(T1, T2);
  const EigenMatrix6x6f e_W = EigenMatrix6x6f(w * EigenMatrix6x6f(m_Wrr, m_Wrp, m_Wpr, m_Wpp));
  const EigenVector6f e_e = EigenVector6f(e_Je.m_e + e_Je.m_J1 * e_x1 + e_Je.m_J2 * e_x2);
  return (e_W * e_e).dot(e_e);
}
#endif

//#define CAMERA_FIX_POSE_EIGEN_DEBUG_JACOBIAN
Camera::Fix::Origin::EigenErrorJacobian
Camera::Fix::Origin::EigenGetErrorJacobian(const Rigid3D &T, const float eps) const {
//#ifdef CFG_DEBUG
#if 0
  ((Rigid3D *) &T)->MakeIdentity();
#endif
  LA::AlignedVector3f g;
  //Rotation3D RT;
  //T.GetGravity(g);
  //RT.MakeIdentity(&g);
  //RT.Transpose();
  //const EigenRotation3D e_RzT = RT;
  const EigenRotation3D e_RzT = m_RT;
  const EigenRotation3D e_R = T;
  const EigenRotation3D e_eR = EigenRotation3D(e_RzT * e_R);
  const EigenVector3f e_er = e_eR.GetRodrigues(eps);
  const EigenMatrix3x3f e_JrI = EigenRotation3D::GetRodriguesJacobianInverse(e_er, eps);
  const EigenMatrix3x3f e_Jr = EigenMatrix3x3f(e_JrI * e_eR);
  EigenErrorJacobian e_Je;
  e_Je.m_Jr = e_Jr;
  e_Je.m_er = e_er;
  e_Je.m_ep = T.GetPosition();
#ifdef CAMERA_FIX_POSE_EIGEN_DEBUG_JACOBIAN
  //const float e_drMax = 1.0f;
  const float e_drMax = 10.0f;
  const EigenVector3f e_dr = EigenVector3f::GetRandom(e_drMax * UT_FACTOR_DEG_TO_RAD);
  const EigenRotation3D e_RGT = EigenMatrix3x3f(e_R * EigenRotation3D(e_dr));
  const EigenVector3f e_er1 = EigenVector3f(e_er - EigenRotation3D(e_RzT * e_RGT).GetRodrigues());
  const EigenVector3f e_er2 = EigenVector3f(e_er1 + e_Jr * e_dr);
  UT::AssertReduction(e_er1, e_er2);
#endif
  return e_Je;
}

Camera::Fix::Origin::EigenFactor
Camera::Fix::Origin::EigenGetFactor(const Rigid3D &T, const float eps) const {
  EigenFactor e_A;
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(T, eps);
  e_A.m_A.setZero();
  const xp128f &wr = m_wr;
  const float wp = m_wp[0];
  const EigenMatrix3x3f e_Wr = EigenMatrix3x3f(wr[0], wr[1], wr[2]);
  const EigenMatrix3x3f e_JTWr = EigenMatrix3x3f(e_Je.m_Jr.transpose() * e_Wr);
  e_A.m_A.block<3, 3>(3, 3) = e_JTWr * e_Je.m_Jr;
  e_A.m_b.block<3, 1>(3, 0) = e_JTWr * e_Je.m_er;
  e_A.m_A(0, 0) = e_A.m_A(1, 1) = e_A.m_A(2, 2) = wp;
  e_A.m_b.block<3, 1>(0, 0) = e_Je.m_ep * wp;
  e_A.m_F = (e_Wr * e_Je.m_er).dot(e_Je.m_er) + wp * e_Je.m_ep.squaredNorm();
  return e_A;
}

float Camera::Fix::Origin::EigenGetCost(const Rigid3D &T, const EigenVector6f &e_x,
                                        const float eps) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(T, eps);
  const EigenVector3f e_er = EigenVector3f(e_Je.m_er + e_Je.m_Jr * e_x.block<3, 1>(3, 0));
  const EigenVector3f e_ep = EigenVector3f(e_Je.m_ep + e_x.block<3, 1>(0, 0));
  const xp128f &wr = m_wr;
  const float wp = m_wp[0];
  const EigenMatrix3x3f e_Wr = EigenMatrix3x3f(wr[0], wr[1], wr[2]);
  return (e_Wr * e_er).dot(e_er) + wp * e_ep.squaredNorm();
}
#endif
