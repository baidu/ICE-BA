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
#include "Feature.h"

namespace FTR {

#ifdef CFG_DEBUG_EIGEN
//#define FTR_EIGEN_DEBUG_JACOBIAN
EigenErrorJacobian EigenGetErrorJacobian(const Rigid3D &C1, const Source &x1, const Depth::InverseGaussian &d1, 
                                         const Rigid3D &C2, const Point2D &z2, const bool cx, const bool cz
#ifdef CFG_STEREO
                                       , const Point3D *br
#endif
                                       ) {
  Rigid3D T12 = C2 / C1;
#ifdef CFG_STEREO
  if (br) {
    T12.SetTranslation(*br + T12.GetTranslation());
  }
#endif

  const EigenRotation3D e_R1 = EigenRotation3D(C1), e_R2 = EigenRotation3D(C2);
  const EigenPoint3D e_p1 = EigenVector3f(C1.GetPosition()), e_p2 = EigenVector3f(C2.GetPosition());
  const EigenRotation3D e_R12 = EigenRotation3D(T12);
  const EigenVector3f e_t12 = EigenVector3f(T12.GetTranslation());
#ifdef CFG_STEREO
  const EigenVector3f e_br = br ? EigenVector3f(*br) : EigenVector3f(0.0f, 0.0f, 0.0f);
#endif

  const EigenPoint3D e_x1 = EigenPoint2D(x1.m_x);
  const float e_d1 = d1.u(), e_z1 = 1.0f / e_d1;
  const EigenPoint3D e_X1 = EigenVector3f(e_x1 * e_z1);
  const EigenPoint3D e_X2 = EigenVector3f(e_R12 * e_X1 + e_t12);
  const EigenPoint2D e_x2 = e_X2.Project();
  const float e_d2 = 1.0f / e_X2.z();
  const EigenVector2f e_e = EigenVector2f(e_x2 - EigenVector2f(z2));
    0.0f;
  //const EigenVector3f e_JXd = EigenVector3f(-e_R12 * e_x1 * e_z1 * e_z1);
  const EigenVector3f e_JXd_d1 = EigenVector3f(-e_R12 * e_x1 * e_z1);
  const EigenVector3f e_w1 = EigenVector3f(e_R1.transpose() * e_X1);
  const EigenMatrix3x3f e_JXr1 = EigenMatrix3x3f(-e_R2 * EigenSkewSymmetricMatrix(e_w1));
  const EigenMatrix3x3f e_JXp1 = e_R2;
  const EigenVector3f e_w2 = EigenVector3f(e_w1 + e_p1 - e_p2);
  const EigenMatrix3x3f e_JXr2 = EigenMatrix3x3f(e_R2 * EigenSkewSymmetricMatrix(e_w2));
  const EigenMatrix3x3f e_JXp2 = EigenMatrix3x3f(-e_R2);
  const EigenMatrix2x3f e_JxX = e_X2.GetJacobianProjection();
  //const EigenVector2f e_Jxd = EigenVector2f(e_JxX * e_JXd);
  //const EigenVector2f e_Jxd = EigenVector2f(e_JxX * e_z1 * e_JXd_d1);
  EigenMatrix2x3f e_JxX_z2;
  e_JxX_z2(0, 0) = 1.0f;    e_JxX_z2(0, 1) = 0.0f;    e_JxX_z2(0, 2) = -e_x2.x();
  e_JxX_z2(1, 0) = 0.0f;    e_JxX_z2(1, 1) = 1.0f;    e_JxX_z2(1, 2) = -e_x2.y();
  const EigenVector2f e_Jxd = EigenVector2f(e_JxX_z2 * e_JXd_d1 * (e_d2 * e_z1));
  const EigenMatrix2x3f e_Jxr1 = EigenMatrix2x3f(e_JxX * e_JXr1);
  const EigenMatrix2x3f e_Jxp1 = EigenMatrix2x3f(e_JxX * e_JXp1);
  const EigenMatrix2x3f e_Jxr2 = EigenMatrix2x3f(e_JxX * e_JXr2);
  const EigenMatrix2x3f e_Jxp2 = EigenMatrix2x3f(e_JxX * e_JXp2);
  const float eps = 1.0e-3f;
#ifdef FTR_EIGEN_DEBUG_JACOBIAN
  //const float e_ddMax = 0.0f;
  const float e_ddMax = 0.001f;
  //const float e_dr1Max = 0.0f;
  const float e_dr1Max = 0.1f;
  //const float e_dp1Max = 0.0f;
  const float e_dp1Max = 0.01f;
  //const float e_dr2Max = 0.0f;
  const float e_dr2Max = 0.1f;
  //const float e_dp2Max = 0.0f;
  const float e_dp2Max = 0.01f;
  const float e_dd = UT::Random<float>(-e_ddMax, e_ddMax);
  const EigenVector3f e_dr1 = EigenAxisAngle::GetRandom(e_dr1Max *
                                                        UT_FACTOR_DEG_TO_RAD).GetRodrigues();
  const EigenVector3f e_dp1 = EigenVector3f::GetRandom(e_dp1Max);
  const EigenVector3f e_dr2 = EigenAxisAngle::GetRandom(e_dr2Max *
                                                        UT_FACTOR_DEG_TO_RAD).GetRodrigues();
  const EigenVector3f e_dp2 = EigenVector3f::GetRandom(e_dp2Max);
  const float e_d1GT = e_d1 + e_dd, e_z1GT = 1.0f / e_d1GT;
  const EigenRotation3D e_R1GT = EigenMatrix3x3f(e_R1 * EigenRotation3D(e_dr1));
  const EigenPoint3D e_p1GT = EigenVector3f(e_p1 + e_dp1);
  const EigenRotation3D e_R2GT = EigenMatrix3x3f(e_R2 * EigenRotation3D(e_dr2));
  const EigenPoint3D e_p2GT = EigenVector3f(e_p2 + e_dp2);
  const EigenPoint3D e_X1GT = EigenVector3f(e_x1 * e_z1GT);
  const EigenPoint3D e_X2GT = EigenVector3f(e_R2GT *
                              EigenVector3f(e_R1GT.GetTranspose() * e_X1GT + e_p1GT - e_p2GT)
#ifdef CFG_STEREO
                                          + e_br
#endif
                                          );
  const EigenPoint2D e_x2GT = e_X2GT.Project();
  const float e_d2GT = 1.0f / e_X2GT.z();
  const EigenVector2f e_ex1 = EigenVector2f(e_x2 - e_x2GT);
  const EigenVector2f e_ex2 = EigenVector2f(e_ex1 + e_Jxd * e_dd + e_Jxr1 * e_dr1 +
                                            e_Jxp1 * e_dp1 + e_Jxr2 * e_dr2 + e_Jxp2 * e_dp2);
  UT::AssertReduction(e_ex1, e_ex2, 1, "ex", eps);
#endif
  EigenErrorJacobian e_Je;
  //const bool vp = fabs(e_d2) > DEPTH_PROJECTION_MIN;
  const bool vp = e_d2 > DEPTH_PROJECTION_MIN && e_d2 < DEPTH_PROJECTION_MAX;
  //if (vp) {
  if (true) {
    e_Je.m_Jd = e_Jxd;
  } else {
    e_Je.m_Jd.setZero();
  }
  if (vp && cx) {
    e_Je.m_Jcx = EigenMatrix2x6f(e_Jxp1, e_Jxr1);
  } else {
    e_Je.m_Jcx.setZero();
  }
  if (vp && cz) {
    e_Je.m_Jcz = EigenMatrix2x6f(e_Jxp2, e_Jxr2);
  } else {
    e_Je.m_Jcz.setZero();
  }
  e_Je.m_e = e_e;
  if (!cx && !cz) {
    FTR::ErrorJacobian::D Je;
    GetErrorJacobian(T12, x1, d1, C2, z2, Je
#ifdef CFG_STEREO
                   , br
#endif
                   );
    e_Je.AssertEqual(Je, 1, "", eps);
    e_Je.Set(Je);
  } else if (!cx && cz) {
    FTR::ErrorJacobian::DCZ Je;
    GetErrorJacobian(T12, x1, d1, C2, z2, Je
#ifdef CFG_STEREO
                   , br
#endif
                   );
    e_Je.AssertEqual(Je, 1, "", eps);
    e_Je.Set(Je);
  } else if (cx && cz) {
    FTR::ErrorJacobian::DCXZ Je;
    GetErrorJacobian(T12, x1, d1, C2, z2, Je
#ifdef CFG_STEREO
                   , br
#endif
                   );
    e_Je.AssertEqual(Je, 1, "", eps);
    e_Je.Set(Je);
  } else {
    UT::Error("TODO (haomin)\n");
  }
  return e_Je;
}

#ifdef CFG_STEREO
EigenErrorJacobian::Stereo EigenGetErrorJacobian(const Point3D &br,
                                                 const Depth::InverseGaussian &d,
                                                 const Source &x) {
  const EigenPoint3D e_x = EigenPoint2D(x.m_x);
  const float e_d = d.u(), e_z = 1.0f / e_d;
  const EigenPoint3D e_X = EigenVector3f(e_x * e_z);
  const EigenPoint3D e_Xr = EigenVector3f(e_X + EigenPoint3D(br));
  const EigenPoint2D e_xr = e_Xr.Project();
  const float e_dr = 1.0f / e_Xr.z();
  const EigenVector2f e_ex = EigenVector2f(e_xr - EigenVector2f(x.m_xr));
  //const EigenVector3f e_JXd = EigenVector3f(-e_x * e_z * e_z);
  //const EigenMatrix2x3f e_JxX = e_Xr.GetJacobianProjection();
  //const EigenVector2f e_Jxd = EigenVector2f(e_JxX * e_JXd);
  const EigenVector3f e_JXd_d = EigenVector3f(-e_x * e_z);
  EigenMatrix2x3f e_JxX_zr;
  e_JxX_zr(0, 0) = 1.0f;    e_JxX_zr(0, 1) = 0.0f;    e_JxX_zr(0, 2) = -e_xr.x();
  e_JxX_zr(1, 0) = 0.0f;    e_JxX_zr(1, 1) = 1.0f;    e_JxX_zr(1, 2) = -e_xr.y();
  const EigenVector2f e_Jxd = EigenVector2f(e_JxX_zr * e_JXd_d * (e_dr * e_z));

  EigenErrorJacobian::Stereo e_Je;
  e_Je.m_Jd = e_Jxd;
  e_Je.m_e = e_ex;

  const float eps = 1.0e-4f;
  FTR::ErrorJacobian::D Je;
  GetErrorJacobian(br, d, x, Je);
  e_Je.AssertEqual(Je, 1, "", eps);
  e_Je = Je;
  return e_Je;
}
#endif
#endif
}
