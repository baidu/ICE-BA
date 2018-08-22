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
//#ifndef CFG_DEBUG
//#define CFG_DEBUG
//#endif
#include "GlobalBundleAdjustor.h"

#ifdef CFG_DEBUG
//#define GBA_DEBUG_ACTUAL_COST
#endif

void GlobalBundleAdjustor::SaveB(FILE *fp) {
  MT::Thread::SaveB(fp);
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::ListSaveB(m_ITs1, fp);
  UT::ListSaveB(m_ITs2, fp);
  FRM::ListSaveB(m_IKFs1, fp);
  FRM::ListSaveB(m_IKFs2, fp);
  UT::ListSaveB(m_IDKFs1, fp);
  UT::ListSaveB(m_IDKFs2, fp);
  UT::VectorsSaveB(m_IDMPs1, fp);
  UT::VectorsSaveB(m_IDMPs2, fp);
  UT::VectorsSaveB(m_IUCs1, fp);
  UT::VectorsSaveB(m_IUCs2, fp);
  FRM::ListSaveB(m_IZps1, fp);
  FRM::ListSaveB(m_IZps2, fp);
  m_IZpLM1.SaveB(fp);
  m_IZpLM2.SaveB(fp);
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
#ifdef CFG_HISTORY
  UT::VectorSaveB(m_hists, fp);
#endif
  UT::VectorSaveB(m_CsDel, fp);
  //UT::SaveB(m_delta2, fp);
  UT::SaveB(m_Zo, fp);
  UT::SaveB(m_Ao, fp);
  FRM::VectorSaveB(m_Zps, fp);
  FRM::VectorSaveB(m_Aps, fp);
  m_ZpLM.SaveB(fp);
  m_ApLM.SaveB(fp);
  FRM::VectorSaveB(m_KFs, fp);
  UT::VectorSaveB(m_iFrms, fp);
  m_Cs.SaveB(fp);
  m_CsLM.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.SaveB(fp);
  m_CsLMGT.SaveB(fp);
#endif
  UT::VectorSaveB(m_ucs, fp);
  UT::VectorSaveB(m_ucmsLM, fp);
#ifdef CFG_HANDLE_SCALE_JUMP
  UT::VectorSaveB(m_dsKF, fp);
#endif
#ifdef CFG_INCREMENTAL_PCG
  m_xcs.SaveB(fp);
  m_xmsLM.SaveB(fp);
#endif
  m_DsLM.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
  m_DsLMGT.SaveB(fp);
#endif
  m_AdsLM.SaveB(fp);
  m_Afps.SaveB(fp);
  m_AfmsLM.SaveB(fp);
  UT::VectorSaveB(m_iKF2d, fp);
  UT::VectorSaveB(m_ds, fp);
  UT::VectorSaveB(m_uds, fp);
  m_SAcus.SaveB(fp);
  m_SMcus.SaveB(fp);
  m_SAcmsLM.SaveB(fp);
  //UT::SaveB(m_F, fp);
  UT::VectorSaveB(m_iKF2cb, fp);
}

void GlobalBundleAdjustor::LoadB(FILE *fp) {
  MT::Thread::LoadB(fp);
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::ListLoadB(m_ITs1, fp);
  UT::ListLoadB(m_ITs2, fp);
  FRM::ListLoadB(m_IKFs1, fp);
  FRM::ListLoadB(m_IKFs2, fp);
  UT::ListLoadB(m_IDKFs1, fp);
  UT::ListLoadB(m_IDKFs2, fp);
  UT::VectorsLoadB(m_IDMPs1, fp);
  UT::VectorsLoadB(m_IDMPs2, fp);
  UT::VectorsLoadB(m_IUCs1, fp);
  UT::VectorsLoadB(m_IUCs2, fp);
  FRM::ListLoadB(m_IZps1, fp);
  FRM::ListLoadB(m_IZps2, fp);
  m_IZpLM1.LoadB(fp);
  m_IZpLM2.LoadB(fp);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
#ifdef CFG_HISTORY
  UT::VectorLoadB(m_hists, fp);
#endif
  UT::VectorLoadB(m_CsDel, fp);
  //UT::LoadB(m_delta2, fp);
  UT::LoadB(m_Zo, fp);
  UT::LoadB(m_Ao, fp);
  FRM::VectorLoadB(m_Zps, fp);
  FRM::VectorLoadB(m_Aps, fp);
  m_ZpLM.LoadB(fp);
  m_ApLM.LoadB(fp);
  FRM::VectorLoadB(m_KFs, fp);
  UT::VectorLoadB(m_iFrms, fp);
  m_Cs.LoadB(fp);
  m_CsLM.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.LoadB(fp);
  m_CsLMGT.LoadB(fp);
  if (m_CsGT && m_CsKFGT.Size() != m_Cs.Size()) {
    const int nKFs = m_Cs.Size();
    m_CsKFGT.Resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      m_CsKFGT[iKF] = m_CsGT[m_iFrms[iKF]].m_T;
    }
  }
  if (m_CsGT && m_CsLMGT.Size() != m_CsLM.Size()) {
    const int Nm = m_CsLM.Size();
    m_CsLMGT.Resize(Nm);
    for (int im = 0, ic = m_Cs.Size() - Nm; im < Nm; ++im, ++ic) {
      m_CsLMGT[im] = m_CsGT[m_iFrms[ic]];
    }
  }
#endif
  UT::VectorLoadB(m_ucs, fp);
  UT::VectorLoadB(m_ucmsLM, fp);
#ifdef CFG_HANDLE_SCALE_JUMP
  UT::VectorLoadB(m_dsKF, fp);
#endif
#ifdef CFG_INCREMENTAL_PCG
  m_xcs.LoadB(fp);
  m_xmsLM.LoadB(fp);
#endif
  m_DsLM.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
  m_DsLMGT.LoadB(fp);
  if (m_CsGT && m_DsLMGT.Size() != m_DsLM.Size()) {
    const int Nm = m_DsLM.Size();
    m_DsLMGT.Resize(Nm);
    m_DsLMGT[0] = m_DsLM[0];
    for (int im1 = 0, im2 = 1, ic1 = m_Cs.Size() - Nm, ic2 = ic1 + 1; im2 < Nm;
         im1 = im2++, ic1 = ic2++) {
      const IMU::Delta &D = m_DsLM[im2];
      IMU::PreIntegrate(m_KFs[ic2].m_us, m_KFs[ic1].m_T.m_t, m_KFs[ic2].m_T.m_t, m_CsLMGT[im1],
                        &m_DsLMGT[im2], &m_work, true, &D.m_u1, &D.m_u2, BA_ANGLE_EPSILON);
    }
  }
#endif
  m_AdsLM.LoadB(fp);
  m_Afps.LoadB(fp);
  m_AfmsLM.LoadB(fp);
  UT::VectorLoadB(m_iKF2d, fp);
  UT::VectorLoadB(m_ds, fp);
  UT::VectorLoadB(m_uds, fp);
  m_SAcus.LoadB(fp);
  m_SMcus.LoadB(fp);
  m_SAcmsLM.LoadB(fp);
  //UT::LoadB(m_F, fp);
  UT::VectorLoadB(m_iKF2cb, fp);
}

float GlobalBundleAdjustor::PrintErrorStatistic(const std::string str,
                                                const AlignedVector<Rigid3D> &Cs,
                                                const AlignedVector<Camera> &CsLM,
                                                const std::vector<Depth::InverseGaussian> &ds,
                                                const AlignedVector<IMU::Delta> &DsLM,
                                                const bool detail) {
  const ES _ES = ComputeErrorStatistic(Cs, CsLM, ds, DsLM);
  const float F = _ES.Total();
  if (detail) {
    const std::string _str(str.size(), ' ');
    _ES.m_ESx.Print( str);
    _ES.m_ESc.Print(_str);
    _ES.m_ESm.Print(_str);
    _ES.m_ESd.Print(_str);
    _ES.m_ESo.Print(_str);
    UT::Print("%se  = %e\n", _str.c_str(), F);
  } else {
    _ES.m_ESx.m_ESx.Print(str, false, false, false, 0, false);
    UT::Print(" --> %e", F);
  }
  return F;
}

GlobalBundleAdjustor::ES GlobalBundleAdjustor::ComputeErrorStatistic(const AlignedVector<Rigid3D>
                                                                     &Cs, const AlignedVector<Camera> &CsLM,
                                                                     const std::vector<Depth::InverseGaussian> &ds,
                                                                     const AlignedVector<IMU::Delta> &DsLM) {
  AlignedVector<Depth::InverseGaussian> _ds;
  const int Nd = static_cast<int>(m_ds.size());
  if (static_cast<int>(ds.size()) < Nd) {
    m_work.Resize(_ds.BindSize(Nd));
    _ds.Bind(m_work.Data(), Nd);
    _ds.Set(m_ds.data(), Nd);
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const int iX = m_iKF2X[iKF];
      if (iX == -1) {
        continue;
      }
      const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
      memcpy(_ds.Data() + m_iKF2d[iKF], ds.data() + iX, Nx * sizeof(Depth::InverseGaussian));
    }
  } else {
    _ds.Bind((Depth::InverseGaussian *) ds.data(), Nd);
  }
  ES _ES;
  _ES.m_ESx = ComputeErrorStatisticFeaturePriorDepth(Cs, _ds.Data());
  _ES.m_ESc = ComputeErrorStatisticPriorCameraPose(Cs);
  _ES.m_ESm = ComputeErrorStatisticPriorCameraMotion(CsLM);
  _ES.m_ESd = ComputeErrorStatisticIMU(CsLM, DsLM);
  _ES.m_ESo = ComputeErrorStatisticFixOrigin(Cs);
  _ES.m_ESfp = ComputeErrorStatisticFixPositionZ(Cs);
  _ES.m_ESfm = ComputeErrorStatisticFixMotion(CsLM);
  return _ES;
}

FTR::ES GlobalBundleAdjustor::ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Rigid3D>
                                                                     &Cs, const Depth::InverseGaussian *ds) {
  FTR::ES ES;
  ES.Initialize();
  Rigid3D Tr[2];
  FTR::Error e;
  const float r2Max = ME::Variance<GBA_ME_FUNCTION>();
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const Rigid3D &C = Cs[iKF];
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      *Tr = C / Cs[Z.m_iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const Depth::InverseGaussian *_ds = ds + m_iKF2d[Z.m_iKF];
      const KeyFrame &_KF = m_KFs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        FTR::GetError(Tr, _KF.m_xs[ix], _ds[ix], z, e);
        const FTR::ESIndex idx(_KF.m_T.m_iFrm, ix, KF.m_T.m_iFrm, iz);
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_e);
          const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
          const bool r = r2 < r2Max;
          ES.Accumulate(m_K.m_K, e.m_e, F, idx, r);
//#ifdef CFG_DEBUG
#if 0
          //if (iKF == 12 && iz == 368) {
          //  Tr[0].Print(true);
          //  _KF.m_xs[ix].m_x.Print(true);
          //  _ds[ix].Print(true);
          //  z.m_z.Print(true);
          //  z.m_W.Print(true);
          //  UT::Print("%e %e\n", r2, F);
          //}
          UT::Print("%d %d %e %e\n", iKF, iz, F, ES.Total());
#endif
        }
        if (z.m_zr.Valid()) {
          const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, e.m_er);
          const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
          const bool r = r2 < r2Max;
          ES.Accumulate(m_K.m_Kr, e.m_er, F, idx, r);
//#ifdef CFG_DEBUG
#if 0
          UT::Print("%d %d %e %e\n", iKF, iz, F, ES.Total());
#endif
        }
#else
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_e);
        //const float F = BA_WEIGHT_FEATURE * ME::Cost<GBA_ME_FUNCTION>(r2);
        const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
        const bool r = r2 < r2Max;
        ES.Accumulate(m_K.m_K, e.m_e, F, idx, r);
#endif
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Depth::InverseGaussian *_ds = ds + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const FTR::Source &x = KF.m_xs[ix];
#ifdef CFG_STEREO
      if (x.m_xr.Valid()) {
        FTR::GetError(m_K.m_br, _ds[ix], x, e.m_er);
        const float rx2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, e.m_er);
        const float Fx = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(rx2) * rx2;
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        const bool rx = rx2 < r2Max;
        ES.Accumulate(m_K.m_Kr, e.m_er, Fx, idx, rx);
      } else
#endif
      {
        const float ed = _ds[ix].u() - zp.m_d;
        const float r2d = zp.m_w * ed * ed;
        //const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Cost<GBA_ME_FUNCTION>(r2d);
        const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<GBA_ME_FUNCTION>(r2d) * r2d;
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        const bool rd = r2d < r2Max;
        ES.Accumulate(ed, Fd, idx, rd);
      }
    }
  }
  return ES;
}

CameraPrior::Pose::ES GlobalBundleAdjustor::ComputeErrorStatisticPriorCameraPose(const
                                                                                 AlignedVector<Rigid3D> &Cs) {
  CameraPrior::Pose::ES ES;
  ES.Initialize();

//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator();
#endif
  CameraPrior::Pose::Error e;
  const int NZ = static_cast<int>(m_Zps.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const CameraPrior::Pose &Z = m_Zps[iZ];
    Z.GetError(Cs, &e, BA_ANGLE_EPSILON);
    //const float F = Z.GetCost(BA_WEIGHT_PRIOR_CAMERA_POSE, e);
    //const float F = Z.GetCost(BA_WEIGHT_PRIOR_CAMERA_POSE, e) -
    //                          BA_WEIGHT_PRIOR_CAMERA_POSE * Z.m_xTb;
    const float w = m_Aps[iZ].m_w;
    const float F = Z.GetCost(w, e) - w * Z.m_xTb;
    ES.Accumulate(e, F, m_iFrms[Z.m_iKFr]);
//#ifdef CFG_DEBUG
#if 0
    UT::Print("%d %f\t", iZ, w / BA_WEIGHT_PRIOR_CAMERA_POSE);
#endif
  }
//#ifdef CFG_DEBUG
#if 0
  UT::Print("\n");
#endif
  return ES;
}

CameraPrior::Motion::ES GlobalBundleAdjustor::ComputeErrorStatisticPriorCameraMotion(const
                                                                                     AlignedVector<Camera> &CsLM) {
  CameraPrior::Motion::ES ES;
  ES.Initialize();
  if (m_ZpLM.Valid()) {
    CameraPrior::Motion::Error e;
    const int ic = m_ZpLM.m_iKF, im = ic - m_Cs.Size() + m_CsLM.Size();
    m_ZpLM.GetError(CsLM[im], &e);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    const float F = m_ZpLM.GetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, e);
#else
    CameraPrior::Element::MM Amm = m_ZpLM.m_Amm;
    CameraPrior::Element::M em = m_ZpLM.m_bm;
    Amm.SolveLDL(em);
    CameraPrior::Element::M _e, Ae;
    e.Get(&_e);
    _e += em;
    e.Set(_e);
    LA::AlignedMatrix9x9f::Ab(m_ZpLM.m_Amm, _e, (float *) &Ae);
    const float F = BA_WEIGHT_PRIOR_CAMERA_MOTION * _e.Dot(Ae);
#endif
    ES.Accumulate(e, F, m_iFrms[ic]);
  }
  return ES;
}

IMU::Delta::ES GlobalBundleAdjustor::ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLM,
                                                              const AlignedVector<IMU::Delta> &DsLM) {
  IMU::Delta::ES ES;
  ES.Initialize();
  const int Nm = CsLM.Size(), iKF0 = m_Cs.Size() - Nm;
  for (int im1 = 0, im2 = 1; im2 < Nm; im1 = im2++) {
    const KeyFrame &KF = m_KFs[iKF0 + im2];
    if (KF.m_us.Empty()) {
      continue;
    }
    const IMU::Delta &D = DsLM[im2];
    const IMU::Delta::Error e = D.GetError(CsLM[im1], CsLM[im2], m_K.m_pu, BA_ANGLE_EPSILON);
    const float F = D.GetCost(BA_WEIGHT_IMU, e);
    ES.Accumulate(e, F, KF.m_T.m_iFrm);
  }
  return ES;
}

Camera::Fix::Origin::ES GlobalBundleAdjustor::ComputeErrorStatisticFixOrigin(const
                                                                             AlignedVector<Rigid3D> &Cs) {
  Camera::Fix::Origin::ES ES;
  ES.Initialize();
  const int iKF = 0, iFrm = m_iFrms[iKF];
  if (iFrm == 0) {
    Camera::Fix::Origin::Error e;
    m_Zo.GetError(Cs[iKF], e, BA_ANGLE_EPSILON);
    const float F = m_Zo.GetCost(e);
    ES.Accumulate(e, F, iFrm);
//#ifdef CFG_DEBUG
#if 0
    if (m_Cs.Size() == 2) {
      const LA::AlignedVector3f g = Cs[iKF].GetColumn2();
      const LA::AlignedVector3f gGT = m_CsKFGT[iKF].GetColumn2();
      const float d = g.Dot(gGT);
      const float a = UT_DOT_TO_ANGLE(d) * UT_FACTOR_RAD_TO_DEG;
      UT::Print("%f\n", a);
    }
#endif
  }
  return ES;
}

Camera::Fix::PositionZ::ES GlobalBundleAdjustor::ComputeErrorStatisticFixPositionZ(const
                                                                                   AlignedVector<Rigid3D> &CsLF) {
  Camera::Fix::PositionZ::ES ES;
  ES.Initialize();
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const float pz = CsLF[iKF].GetPositionZ();
    ES.Accumulate(pz, z.GetCost(pz), m_iFrms[iKF]);
  }
  return ES;
}

Camera::Fix::Motion::ES GlobalBundleAdjustor::ComputeErrorStatisticFixMotion(const
                                                                             AlignedVector<Camera> &CsLM) {
  Camera::Fix::Motion::ES ES;
  ES.Initialize();
  const Camera::Fix::Zero zv[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INVALID)};
  const Camera::Fix::Zero zba[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID)};
  const Camera::Fix::Zero zbw[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID)};
  const int Nm = CsLM.Size(), iKF0 = m_Cs.Size() - Nm;
  for (int im = 0; im < Nm; ++im) {
    const int iFrm = m_iFrms[iKF0 + im];
    const int i = m_ucmsLM[im] >> 5;
    const Camera &C = CsLM[im];
    ES.AccumulateVelocity(C.m_v, zv[i].GetCost(C.m_v), iFrm);
    ES.AccumulateBiasAcceleration(C.m_ba, zba[i].GetCost(C.m_ba), iFrm);
    ES.AccumulateBiasGyroscope(C.m_bw, zbw[i].GetCost(C.m_bw), iFrm);
  }
  return ES;
}

float GlobalBundleAdjustor::PrintErrorStatistic(const std::string str,
                                                const AlignedVector<Rigid3D> &Cs,
                                                const AlignedVector<Camera> &CsLM,
                                                const std::vector<Depth::InverseGaussian> &ds,
                                                const AlignedVector<IMU::Delta> &DsLM,
                                                const LA::AlignedVectorXf &xs,
                                                const bool detail) {
  const ES _ES = ComputeErrorStatistic(Cs, CsLM, ds, DsLM, xs);
  const float F = _ES.Total();
  if (detail) {
    const std::string _str(str.size(), ' ');
    _ES.m_ESx.Print( str);
    _ES.m_ESc.Print(_str);
    _ES.m_ESm.Print(_str);
    _ES.m_ESd.Print(_str);
    _ES.m_ESo.Print(_str);
    UT::Print("%se  = %e", _str.c_str(), F);
  } else {
    UT::Print("%s%e", str.c_str(), F);
  }
  return F;
}

GlobalBundleAdjustor::ES GlobalBundleAdjustor::ComputeErrorStatistic(const AlignedVector<Rigid3D>
                                                                     &Cs, const AlignedVector<Camera> &CsLM,
                                                                     const std::vector<Depth::InverseGaussian> &ds,
                                                                     const AlignedVector<IMU::Delta> &DsLM,
                                                                     const LA::AlignedVectorXf &xs,
                                                                     const bool updateOnly) {
  const int pc = 6, pm = 9;
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size(), Nd = static_cast<int>(m_ds.size());
  const LA::Vector6f *xcs = (LA::Vector6f *) xs.Data();
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  const float *xds = (float *) (xms + Nm);
  AlignedVector<Depth::InverseGaussian> _ds;
  LA::AlignedVectorXf _xds;
  ConvertCameraUpdates(xcs, &m_xcsP);
  //if (static_cast<int>(ds.size()) < Nd) {
  if (xs.Size() < Nc * pc + Nm * pm + Nd) {
    m_work.Resize(_ds.BindSize(Nd) + _xds.BindSize(Nd));
    _ds.Bind(m_work.Data(), Nd);
    _ds.Set(m_ds.data(), Nd);
    _xds.Bind(_ds.BindNext(), Nd);
    _xds.MakeZero();
    for (int iKF = 0, i = 0; iKF < Nc; ++iKF) {
      const int iX = m_iKF2X[iKF];
      if (iX == -1) {
        continue;
      }
      const KeyFrame &KF = m_KFs[iKF];
      const int id = m_iKF2d[iKF], Nx = static_cast<int>(KF.m_xs.size());
      memcpy(_ds.Data() + id, ds.data() + iX, Nx * sizeof(Depth::InverseGaussian));
      const ubyte *uds = m_uds.data() + id;
      float *__xds = _xds.Data() + id;
      for (int ix = 0; ix < Nx; ++ix) {
        if (uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
          __xds[ix] = xds[i++];
        }
      }
    }
  } else {
    _ds.Bind((Depth::InverseGaussian *) ds.data(), Nd);
    _xds.Bind((float *) xds, Nd);
  }
  const LA::ProductVector6f *xcsP = m_xcsP.Data() + Nc - Nm;
  ES _ES;
  _ES.m_ESx = ComputeErrorStatisticFeaturePriorDepth(Cs, _ds.Data(), m_xcsP, _xds, updateOnly);
  _ES.m_ESc = ComputeErrorStatisticPriorCameraPose(Cs, m_xcsP, updateOnly);
  _ES.m_ESm = ComputeErrorStatisticPriorCameraMotion(CsLM, xcsP, xms, updateOnly);
  _ES.m_ESd = ComputeErrorStatisticIMU(CsLM, DsLM, xcsP, xms, updateOnly);
  _ES.m_ESo = ComputeErrorStatisticFixOrigin(Cs, m_xcsP, updateOnly);
  _ES.m_ESfp = ComputeErrorStatisticFixPositionZ(Cs, m_xcsP, updateOnly);
  _ES.m_ESfm = ComputeErrorStatisticFixMotion(CsLM, xms, updateOnly);
  return _ES;
}

FTR::ES GlobalBundleAdjustor::ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Rigid3D>
                                                                     &Cs, const Depth::InverseGaussian *ds,
                                                                     const AlignedVector<LA::ProductVector6f> &xcs,
                                                                     const LA::AlignedVectorXf &xds,
                                                                     const bool updateOnly) {
  FTR::ES ES;
  ES.Initialize();
#ifdef GBA_DEBUG_ACTUAL_COST
  FTR::ES _ES;
  _ES.Initialize();
#endif
  FTR::Error ex;
  float Fx;
#if 0
//#if 1
  float FMax = 0.0f, eMax = 0.0f;
#endif
#ifdef GBA_DEBUG_ACTUAL_COST
  const float r2Max = ME::Variance<GBA_ME_FUNCTION>();
#endif
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const LA::ProductVector6f *xc = !updateOnly || (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA) ?
                                    &xcs[iKF] : NULL;
#ifdef GBA_DEBUG_ACTUAL_COST
    const Rigid3D C = xc ? Rigid3D(Cs[iKF], &xc->Get012(), &xc->Get345()) : Cs[iKF];
#endif
    const KeyFrame &KF = m_KFs[iKF];
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF, id = m_iKF2d[_iKF];
      const LA::ProductVector6f *_xc = !updateOnly || (m_ucs[_iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA) ?
                                       &xcs[_iKF] : NULL;
#ifdef GBA_DEBUG_ACTUAL_COST
      const Rigid3D _C = _xc ? Rigid3D(Cs[_iKF], &_xc->Get012(), &_xc->Get345()) : Cs[_iKF];
      Rigid3D Tr[2];
      *Tr = C / _C;
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const Depth::InverseGaussian *_ds = ds + id;
#endif
      const ubyte *_uds = m_uds.data() + id;
      const float *_xds = xds.Data() + id;
      const KeyFrame &_KF = m_KFs[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        const float *xd = !updateOnly || (_uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH) ? &_xds[ix] : NULL;
        const FTR::Factor::Full::L &L = KF.m_Lzs[iz];
        if (_xc || xc || xd) {
          Fx = FTR::GetCost(L, z, _xc, xc, xd, ex);
        } else {
          Fx = L.m_F;
#ifdef CFG_STEREO
          if (z.m_z.Valid()) {
            ex.m_e = L.m_Je.m_e;
          }
          if (z.m_zr.Valid()) {
            ex.m_er = L.m_Jer.m_e;
          }
#else
          ex.m_e = L.m_Je.m_e;
#endif
        }
        const FTR::ESIndex idx(_KF.m_T.m_iFrm, ix, KF.m_T.m_iFrm, iz);
#if 0
//#if 1
        if (iKF == 1 && iz == 41) {
          FTR::ErrorJacobian::DCXZ Je;
          Je = L.m_Je;
          const Rigid3D Tr = m_CsBkp[iKF] / m_CsBkp[_iKF];
          FTR::GetErrorJacobian(Tr, _KF.m_xs[ix], m_dsBkp[id + ix], m_CsBkp[iKF], z.m_z, Je);
          UT::PrintSeparator();
          UT::Print("%f\n", sqrtf(Je.m_e.SquaredLength() * m_K.m_K.fxy()));
          FTR::GetError(Je, _xc, NULL, NULL, ex.m_e);
          UT::Print("%f\n", sqrtf(ex.m_e.SquaredLength() * m_K.m_K.fxy()));
          FTR::GetError(Je, NULL, xc, NULL, ex.m_e);
          UT::Print("%f\n", sqrtf(ex.m_e.SquaredLength() * m_K.m_K.fxy()));
          FTR::GetError(Je, NULL, NULL, xd, ex.m_e);
          UT::Print("%f\n", sqrtf(ex.m_e.SquaredLength() * m_K.m_K.fxy()));
          FTR::GetError(Je, _xc, xc, xd, ex.m_e);
          UT::Print("%f\n", sqrtf(ex.m_e.SquaredLength() * m_K.m_K.fxy()));
        }
#endif
#if 0
//#if 1
        if (UT::Debugging()) {
          UT::Print("%e + %e = %e\n", Fx, ES.m_ESx.m_SF, F + ES.m_ESx.m_SF);
          if (F > FMax) {
            FMax = F;
            UT::Print("iKF %d ix %d --> iKF %d iz %d %e\n", _iKF, ix, iKF, iz, F);
          }
          if (z.m_z.Valid()) {
            const float e = sqrtf(ex.m_e.SquaredLength() * m_K.m_K.fxy());
            if (_e > eMax) {
              eMax = _e;
              UT::Print("iKF %d ix %d --> iKF %d iz %d %f\n", _iKF, ix, iKF, iz, e);
            }
          }
          if (z.m_zr.Valid()) {
            const float e = sqrtf(ex.m_er.SquaredLength() * m_K.m_Kr.fxy());
            if (e > eMax) {
              eMax = e;
              UT::Print("iKF %d ix %d --> iKF %d iz %d %f\n", _iKF, ix, iKF, iz, e);
            }
          }
        }
#endif
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          ES.Accumulate(m_K.m_K, ex.m_e, Fx, idx);
        }
        if (z.m_zr.Valid()) {
          ES.Accumulate(m_K.m_Kr, ex.m_er, 0.0f, idx);
        }
#else
        ES.Accumulate(m_K.m_K, ex.m_e, Fx, idx);
#endif
#ifdef GBA_DEBUG_ACTUAL_COST
        Depth::InverseGaussian d = _ds[ix];
        if (xd) {
          d.u() += *xd;
        }
        FTR::GetError(Tr, _KF.m_xs[ix], d, z, ex);
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ex.m_e);
          const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
          const bool r = r2 < r2Max;
          _ES.Accumulate(m_K.m_K, ex.m_e, F, idx, r);
        }
        if (z.m_zr.Valid()) {
          const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, ex.m_er);
          const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
          const bool r = r2 < r2Max;
          _ES.Accumulate(m_K.m_Kr, ex.m_er, F, idx, r);
        }
#else
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_e);
        //const float F = BA_WEIGHT_FEATURE * ME::Cost<GBA_ME_FUNCTION>(r2);
        const float F = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(r2) * r2;
        const bool r = r2 < r2Max;
        _ES.Accumulate(m_K.m_K, ex.m_e, F, idx, r);
#endif
#endif
      }
    }
  }
  float ep, Fp;
#ifdef CFG_STEREO
  LA::Vector2f er;
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id = m_iKF2d[iKF];
#ifdef GBA_DEBUG_ACTUAL_COST
    const Depth::InverseGaussian *_ds = ds + id;
#endif
    const ubyte *uds = m_uds.data() + id;
    const float *_xds = xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef GBA_DEBUG_ACTUAL_COST
      Depth::InverseGaussian d = _ds[ix];
      d.u() += _xds[ix];
#endif
      const FTR::Source &x = KF.m_xs[ix];
      const bool ud = !updateOnly || (uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH);
#ifdef CFG_STEREO
      if (x.m_xr.Valid()) {
        const FTR::Factor::Stereo &a = KF.m_Ards[ix];
        if (ud) {
          Fp = FTR::GetCost(a, x, _xds[ix], er);
        } else {
          er = a.m_Je.m_e;
          Fp = a.m_F;
        }
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        ES.Accumulate(m_K.m_Kr, er, Fp, idx);
#if 0
//#if 1
        if (UT::Debugging()) {
          UT::Print("%e + %e = %e\n", Fp, ES.m_ESx.m_SF, Fp + ES.m_ESx.m_SF);
          if (Fp > FMax) {
            FMax = Fp;
            UT::Print("iKF %d ix %d %e\n", iKF, ix, Fx);
          }
          const float e = sqrtf(er.SquaredLength() * m_K.m_Kr.fxy());
          if (e > eMax) {
            eMax = e;
            UT::Print("iKF %d ix %d %f\n", iKF, ix, e);
          }
        }
#endif
#ifdef GBA_DEBUG_ACTUAL_COST
        FTR::GetError(m_K.m_br, d, x, er);
        const float rx2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, er);
        const float Fx = BA_WEIGHT_FEATURE * ME::Weight<GBA_ME_FUNCTION>(rx2) * rx2;
        const bool rx = rx2 < r2Max;
        _ES.Accumulate(m_K.m_Kr, er, Fx, idx, rx);
#endif
      } else
#endif
      {
        const Depth::Prior::Factor &a = KF.m_Apds[ix];
        if (ud) {
          Fp = zp.GetCost(a, _xds[ix], ep);
        } else {
          ep = a.m_e;
          Fp = a.m_F;
        }
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        ES.Accumulate(ep, Fp, idx);
#ifdef GBA_DEBUG_ACTUAL_COST
        const float ed = _ds[ix].u() - _zp.m_d;
        const float r2d = _zp.m_w * ed * ed;
        //const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Cost<GBA_ME_FUNCTION>(r2d);
        const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<GBA_ME_FUNCTION>(r2d) * r2d;
        const bool rd = r2d < r2Max;
        _ES.Accumulate(ed, Fd, idx, rd);
#endif
      }
    }
  }
#ifdef GBA_DEBUG_ACTUAL_COST
  if (m_iIter == 3) {
    UT::DebugStart();
    _ES.Print();
    UT::DebugStop();
  }
#endif
  return ES;
}

CameraPrior::Pose::ES GlobalBundleAdjustor::ComputeErrorStatisticPriorCameraPose(const
                                                                                 AlignedVector<Rigid3D> &Cs,
                                                                                 const AlignedVector<LA::ProductVector6f> &xcs,
                                                                                 const bool updateOnly) {
  CameraPrior::Pose::ES ES;
  ES.Initialize();
  CameraPrior::Pose::Error e;

  const int Nc = xcs.Size();
  m_xps.Resize(Nc);   m_xpsP.resize(Nc);
  m_xrs.Resize(Nc);   m_xrsP.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    const bool uc = !updateOnly || (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA);
    if (uc) {
      m_xcsP[ic].Get(m_xps[ic], m_xrs[ic]);
      m_xpsP[ic] = &m_xps[ic];
      m_xrsP[ic] = &m_xrs[ic];
    } else {
      m_xpsP[ic] = NULL;
      m_xrsP[ic] = NULL;
    }
  }
  const int NZ = static_cast<int>(m_Zps.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const CameraPrior::Pose &Z = m_Zps[iZ];
    //const float F = Z.GetCost(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Aps[iZ].m_Je, m_xpsP, m_xrsP, &e);
    const float F =  Z.GetCost(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Aps[iZ].m_Je, m_xpsP, m_xrsP, &e) -
                               BA_WEIGHT_PRIOR_CAMERA_POSE * Z.m_xTb;
    ES.Accumulate(e, F, m_iFrms[Z.m_iKFr]);
  }
  return ES;
}

CameraPrior::Motion::ES GlobalBundleAdjustor::ComputeErrorStatisticPriorCameraMotion(const
                                                                                     AlignedVector<Camera> &CsLM,
                                                                                     const LA::ProductVector6f *xcs,
                                                                                     const LA::Vector9f *xms,
                                                                                     const bool updateOnly) {
  CameraPrior::Motion::ES ES;
  ES.Initialize();
  if (m_ZpLM.Valid()) {
    CameraPrior::Motion::Error e;
    const int ic = m_ZpLM.m_iKF, im = ic - m_Cs.Size() + m_CsLM.Size();
    const int iFrm = m_iFrms[ic];
    const ubyte ucm = m_ucmsLM[im];
    const bool ur = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION) != 0;
    const bool uv = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) != 0;
    const bool uba = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) != 0;
    const bool ubw = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) != 0;
    if (ur || uv || uba || ubw) {
      LA::AlignedVector3f xr, xv, xba, xbw;
      const LA::ProductVector6f &xc = xcs[im];
      const LA::Vector9f &xm = xms[im];
      const LA::AlignedVector3f *_xr = ur ? &(xr = LA::AlignedVector3f(&xc.v3())) : NULL;
      const LA::AlignedVector3f *_xv = uv ? &(xv = LA::AlignedVector3f(&xm.v0())) : NULL;
      const LA::AlignedVector3f *_xba = uba ? &(xba = LA::AlignedVector3f(&xm.v3())) : NULL;
      const LA::AlignedVector3f *_xbw = ubw ? &(xbw = LA::AlignedVector3f(&xm.v6())) : NULL;
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
      const float F = m_ZpLM.GetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_ApLM.m_Je,
                                     _xr, _xv, _xba, _xbw, &e);
#else
      m_ZpLM.GetError(m_ApLM.m_Je, _xr, _xv, _xba, _xbw, &e);
      CameraPrior::Element::MM Amm = m_ZpLM.m_Amm;
      CameraPrior::Element::M em = m_ZpLM.m_bm;
      Amm.SolveLDL(em);
      CameraPrior::Element::M _e, Ae;
      e.Get(&_e);
      _e += em;
      e.Set(_e);
      LA::AlignedMatrix9x9f::Ab(m_ZpLM.m_Amm, _e, (float *) &Ae);
      const float F = BA_WEIGHT_PRIOR_CAMERA_MOTION * _e.Dot(Ae);
#endif
      ES.Accumulate(e, F, iFrm);
    } else {
      ES.Accumulate(m_ApLM.m_Je.m_e, m_ApLM.m_F, iFrm);
    }
  }
  return ES;
}

IMU::Delta::ES GlobalBundleAdjustor::ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLM,
                                                              const AlignedVector<IMU::Delta> &DsLM,
                                                              const LA::ProductVector6f *xcs,
                                                              const LA::Vector9f *xms,
                                                              const bool updateOnly) {
  IMU::Delta::ES ES;
  ES.Initialize();
#ifdef GBA_DEBUG_ACTUAL_COST
  IMU::Delta::ES _ES;
  _ES.Initialize();
#endif
//#ifdef CFG_DEBUG
#if 0
  float dF = 0.0f;
#endif
  IMU::Delta::Error e;
  float F;
  LA::AlignedVector3f xp[2], xr[2], xv[2], xba[2], xbw[2];
  LA::AlignedVector3f *_xp[2], *_xr[2], *_xv[2], *_xba[2], *_xbw[2];
  int r1 = 0, r2 = 1;
  
  const ubyte ucFlag = GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                       GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  const ubyte ucmFlag = ucFlag |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
  const int Nm = m_CsLM.Size(), iKF0 = m_Cs.Size() - Nm;
  for (int im1 = 0, im2 = 1; im2 < Nm; im1 = im2++) {
    UT_SWAP(r1, r2);
    if (im1 == 0) {
      const ubyte ucm = m_ucmsLM[im1], uc = ucm & ucFlag;
      _xp[r1] = !updateOnly || uc ? &(xp[r1] = LA::AlignedVector3f(&xcs[im1].v0())) : NULL;
      _xr[r1] = !updateOnly || uc ? &(xr[r1] = LA::AlignedVector3f(&xcs[im1].v3())) : NULL;
      _xv[r1] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
                &(xv[r1] = LA::AlignedVector3f(&xms[im1].v0())) : NULL;
      _xba[r1] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
                 &(xba[r1] = LA::AlignedVector3f(&xms[im1].v3())) : NULL;
      _xbw[r1] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
                 &(xbw[r1] = LA::AlignedVector3f(&xms[im1].v6())) : NULL;
    }
    const ubyte ucm = m_ucmsLM[im2], uc = ucm & ucFlag;
    _xp[r2] = !updateOnly || uc ? &(xp[r2] = LA::AlignedVector3f(&xcs[im2].v0())) : NULL;
    _xr[r2] = !updateOnly || uc ? &(xr[r2] = LA::AlignedVector3f(&xcs[im2].v3())) : NULL;
    _xv[r2] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
              &(xv[r2] = LA::AlignedVector3f(&xms[im2].v0())) : NULL;
    _xba[r2] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
               &(xba[r2] = LA::AlignedVector3f(&xms[im2].v3())) : NULL;
    _xbw[r2] = !updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
               &(xbw[r2] = LA::AlignedVector3f(&xms[im2].v6())) : NULL;
    const KeyFrame &KF = m_KFs[iKF0 + im2];
    if (KF.m_us.Empty()) {
      continue;
    }
    const IMU::Delta &D = DsLM[im2];
    const IMU::Delta::Factor &A = m_AdsLM[im2];
    if (!updateOnly || (ucm & ucmFlag) || (m_ucmsLM[im1] & ucmFlag)) {
      F = D.GetCost(BA_WEIGHT_IMU, A.m_Je, _xp[r1], _xr[r1], _xv[r1], _xba[r1], _xbw[r1], 
                                           _xp[r2], _xr[r2], _xv[r2], _xba[r2], _xbw[r2], e);
    } else {
      e = A.m_Je.m_e;
      F = A.m_F;
    }
    ES.Accumulate(e, F, KF.m_T.m_iFrm);
#ifdef GBA_DEBUG_ACTUAL_COST
    if (m_iIter==3 && KF.m_T.m_iFrm == 17) {
      UT::DebugStart();
      e.Print(false, true);
    }
    const Camera C1(CsLM[im1], _xp[r1], _xr[r1], _xv[r1], _xba[r1], _xbw[r1]);
    const Camera C2(CsLM[im2], _xp[r2], _xr[r2], _xv[r2], _xba[r2], _xbw[r2]);
    D.GetError(C1, C2, m_K.m_pu, e);
    F = D.GetCost(BA_WEIGHT_IMU, e);
    if (UT::Debugging()) {
      e.Print(false, true);
      UT::DebugStop();
    }
    _ES.Accumulate(e, F, KF.m_T.m_iFrm);
#endif
//#ifdef CFG_DEBUG
#if 0
    dF = A.m_F - F + dF;
#endif
  }
#ifdef GBA_DEBUG_ACTUAL_COST
  if (m_iIter == 3) {
    UT::DebugStart();
    _ES.Print();
    UT::DebugStop();
  }
#endif
  return ES;
}

Camera::Fix::Origin::ES GlobalBundleAdjustor::ComputeErrorStatisticFixOrigin(const
                                                                             AlignedVector<Rigid3D> &Cs,
                                                                             const AlignedVector<LA::ProductVector6f> &xcs,
                                                                             const bool updateOnly) {
  Camera::Fix::Origin::ES ES;
  ES.Initialize();
  const int iKF = 0, iFrm = m_iFrms[iKF];
  if (iFrm == 0) {
    Camera::Fix::Origin::Error e;
    LA::AlignedVector3f xp, xr;
    xcs[iKF].Get(xp, xr);
    m_Zo.GetError(m_Ao.m_Je, xp, xr, e);
    const float F = m_Zo.GetCost(e);
    ES.Accumulate(e, F, iFrm);
  }
  return ES;
}

Camera::Fix::PositionZ::ES GlobalBundleAdjustor::ComputeErrorStatisticFixPositionZ(const
                                                                                   AlignedVector<Rigid3D> &Cs,
                                                                                   const AlignedVector<LA::ProductVector6f> &xcs,
                                                                                   const bool updateOnly) {
  Camera::Fix::PositionZ::ES ES;
  ES.Initialize();
  float F;
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const float pz = Cs[iKF].GetPositionZ();
    if (!updateOnly || (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA)) {
      F = z.GetCost(pz, xcs[iKF].v2());
    }
    else {
      F = m_Afps[iKF].m_F;
    }
    ES.Accumulate(pz, F, m_iFrms[iKF]);
  }
  return ES;
}

Camera::Fix::Motion::ES GlobalBundleAdjustor::ComputeErrorStatisticFixMotion(const
                                                                             AlignedVector<Camera> &CsLM,
                                                                             const LA::Vector9f *xms,
                                                                             const bool updateOnly) {
  Camera::Fix::Motion::ES ES;
  ES.Initialize();
  LA::AlignedVector3f e;
  float F;
  const Camera::Fix::Zero zv[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INVALID)};
  const Camera::Fix::Zero zba[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID)};
  const Camera::Fix::Zero zbw[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID)};
  const int Nm = m_CsLM.Size(), iKF0 = m_Cs.Size() - Nm;
  for (int im = 0; im < Nm; ++im) {
    const Camera::Fix::Motion::Factor &A = m_AfmsLM[im];
    const Camera &C = CsLM[im];
    const LA::Vector9f &xm = xms[im];
    const int iFrm = m_iFrms[iKF0 + im];
    const ubyte ucm = m_ucmsLM[im];
    const int i = ucm >> 5;
    if (!updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY)) {
      F = zv[i].GetCost(C.m_v, &xm.v0(), e);
    } else {
      e = C.m_v;
      F = A.m_Av.F();
    }
    ES.AccumulateVelocity(e, F, iFrm);
    if (!updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION)) {
      F = zba[i].GetCost(C.m_ba, &xm.v3(), e);
    } else {
      e = C.m_ba;
      F = A.m_Aba.F();
    }
    ES.AccumulateBiasAcceleration(e, F, iFrm);
    if (!updateOnly || (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
      F = zbw[i].GetCost(C.m_bw, &xm.v6(), e);
    } else {
      e = C.m_bw;
      F = A.m_Abw.F();
    }
    ES.AccumulateBiasGyroscope(e, F, iFrm);
  }
  return ES;
}

void GlobalBundleAdjustor::AssertConsistency(const bool chkFlag, const bool chkSchur) {
  const int iFrm = m_iFrms.back();
  const std::string str = UT::String("GBA [%d]", iFrm);
  const int nKFs = static_cast<int>(m_KFs.size()), Nm = m_CsLM.Size();
  UT_ASSERT(static_cast<int>(m_iFrms.size()) == nKFs);
  UT_ASSERT(m_Cs.Size() == nKFs && static_cast<int>(m_ucs.size()) == nKFs);
#ifdef CFG_HANDLE_SCALE_JUMP
  UT_ASSERT(static_cast<float>(m_dsKF.size()) == nKFs);
#endif
  UT_ASSERT(Nm <= nKFs && static_cast<int>(m_ucmsLM.size()) == Nm);
#ifdef CFG_INCREMENTAL_PCG
  UT_ASSERT(m_xcs.Size() == nKFs && m_xmsLM.Size() == Nm);
#endif
  UT_ASSERT(m_DsLM.Size() == Nm && m_AdsLM.Size() == Nm);
  UT_ASSERT(m_Afps.Size() == nKFs && m_AfmsLM.Size() == Nm);
  const int Nd = static_cast<int>(m_ds.size());
  UT_ASSERT(static_cast<int>(m_uds.size()) == Nd);
  UT_ASSERT(m_SAcus.Size() == nKFs && m_SMcus.Size() == nKFs && m_SAcmsLM.Size() == Nm);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    UT_ASSERT(m_CsKFGT.Size() == nKFs && m_CsLMGT.Size() == Nm && m_DsLMGT.Size() == Nm);
  } else {
    UT_ASSERT(m_CsKFGT.Empty() && m_CsLMGT.Empty() && m_DsLMGT.Empty());
  }
  //if (m_dsGT) {
  //  UT_ASSERT(m_dsGT->size() == Nd);
  //}
#endif

  const int NZp = static_cast<int>(m_Zps.size());
  for (int iZp = 0; iZp < NZp; ++iZp) {
    const CameraPrior::Pose &Zp = m_Zps[iZp];
    //if (iZp > 0) {
    //  UT_ASSERT(Zp.m_iKFr > m_Zps[iZp - 1].m_iKFr);
    //}
    Zp.AssertConsistency();
    m_Aps[iZp].AssertConsistency();
  }
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  for (std::list<CameraPrior::Pose>::const_iterator IZp = m_IZps1.begin(); IZp != m_IZps1.end();
       ++IZp) {
    IZp->AssertConsistency();
  }
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);

  int SNd = 0, SNcb = 0;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    UT_ASSERT(m_iKF2d[iKF] == SNd && m_iKF2cb[iKF] == SNcb);
    const KeyFrame &KF = m_KFs[iKF];
    SNd += static_cast<int>(KF.m_xs.size());
    SNcb += static_cast<int>(KF.m_ikp2KF.size());
  }
  UT_ASSERT(SNd == Nd && static_cast<int>(m_iKF2d.size()) == nKFs + 1 && m_iKF2d.back() == Nd);
  const int Ncb = CountSchurComplementsOffDiagonal();
  UT_ASSERT(Ncb == SNcb);
  UT_ASSERT(static_cast<int>(m_iKF2cb.size()) == nKFs + 1 && m_iKF2cb.back() == Ncb);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    m_Cs[iKF].AssertOrthogonal(1, str + UT::String(" R[%d]", iKF));
#ifdef CFG_HANDLE_SCALE_JUMP
    const int id = m_iKF2d[iKF];
    const float u = AverageDepths(m_ds.data() + id, m_iKF2d[iKF + 1] - id);
    UT::AssertEqual(u, m_dsKF[iKF], 1, str + UT::String(" dk[%d]", iKF));
#endif
  }
  const ubyte ucmFlag = GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  for (int im = Nm - 1, ic = nKFs - 1; im >= 0; --im, --ic) {
    m_CsLM[im].AssertConsistency(1, str + UT::String(" C[%d]", ic));
    UT_ASSERT(m_CsLM[im].m_T == m_Cs[ic]);
    const ubyte ucm = m_ucmsLM[im];
    if (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA) {
      UT_ASSERT((ucm & ucmFlag) != 0);
    } else {
      UT_ASSERT((ucm & ucmFlag) == 0);
    }
    if (ic == 0) {
      UT_ASSERT((ucm & GBA_FLAG_CAMERA_MOTION_INVALID) == 0);
    } else {
      const int im1 = im - 1, im2 = im + 1;
      if ((im1 >= 0 && !m_KFs[ic].m_us.Empty()) ||
          (im2 < Nm && !m_KFs[ic + 1].m_us.Empty())) {
        UT_ASSERT((ucm & GBA_FLAG_CAMERA_MOTION_INVALID) == 0);
      } else {
        UT_ASSERT((ucm & GBA_FLAG_CAMERA_MOTION_INVALID) != 0);
      }
    }
  }
#ifdef CFG_INCREMENTAL_PCG
  if (chkFlag) {
    for (int ic = 0; ic < nKFs; ++ic) {
      if (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA) {
        UT_ASSERT(m_xcs[ic].SquaredLength() == 0.0f);
      }
    }
    for (int im = 0; im < Nm; ++im) {
      const ubyte ucm = m_ucmsLM[im];
      const LA::Vector9f &xm = m_xmsLM[im];
      if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
        UT_ASSERT(xm.Get012().SquaredLength() == 0.0f);
      }
      if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
        UT_ASSERT(xm.Get345().SquaredLength() == 0.0f);
      }
      if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
        UT_ASSERT(xm.Get678().SquaredLength() == 0.0f);
      }
    }
  }
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    UT_ASSERT(m_iFrms[iKF] == KF.m_T.m_iFrm);
    if (iKF > 0) {
      const int _iKF = iKF - 1;
      const FRM::Tag &_T = m_KFs[_iKF].m_T;
      UT_ASSERT(KF.m_T > _T);
      if (!KF.m_us.Empty()) {
        UT_ASSERT(KF.m_us.Front().t() >= _T.m_t);
        //UT_ASSERT(KF.m_iKFsMatch.back() == _iKF);
        UT_ASSERT(KF.m_iKFsMatch[KF.m_Zm.m_SMczms.Size() - 1] == _iKF);
      }
    }
    if (!KF.m_us.Empty()) {
      UT_ASSERT(KF.m_us.Back().t() <= KF.m_T.m_t);
    }
    if (KF.m_iKFNearest != -1) {
      UT_ASSERT(m_KFs[KF.m_iKFNearest].m_T < KF.m_T);
    }
    KF.AssertConsistency(iKF);
    m_marksTmp1.assign(nKFs, 0);
    const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int _iKF = KF.m_iKFsMatch[ik];
      m_marksTmp1[_iKF] = 1;
      const int _ik = m_KFs[_iKF].SearchMatchKeyFrame(iKF);
      UT_ASSERT(_ik != -1);
      if (_iKF < iKF) {
        KF.m_Zm.AssertConsistency(ik, m_KFs[_iKF], KF, m_izmsTmp);
      }
    }
    const int _iKF = iKF - 1;
    if (iKF > 0) {
      UT_ASSERT(m_marksTmp1[_iKF] != 0);
    }
    for (int jKF = 0; jKF < _iKF; ++jKF) {
      if (KF.SearchFrameMeasurement(jKF) != -1 ||
          FRM::Frame::HasFeatureMeasurementMatch(m_KFs[jKF], KF)) {
        UT_ASSERT(m_marksTmp1[jKF] != 0);
      } else {
        //UT_ASSERT(m_marksTmp[jKF] == 0);
      }
    }
    for (int jKF = iKF + 1; jKF < nKFs; ++jKF) {
      if (m_KFs[jKF].SearchFrameMeasurement(iKF) != -1 ||
          FRM::Frame::HasFeatureMeasurementMatch(m_KFs[jKF], KF)) {
        UT_ASSERT(m_marksTmp1[jKF] != 0);
      } else {
        //UT_ASSERT(m_marksTmp1[jKF] == 0);
      }
    }
  }
  if (!m_xsGN.Empty()) {
    ConvertCameraUpdates(m_xsGN.Data(), &m_xp2s, &m_xr2s);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const ubyte dc = m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DELTA;
      if (m_xr2s[iKF] > BA_BACK_SUBSTITUTE_ROTATION || m_xp2s[iKF] > BA_BACK_SUBSTITUTE_POSITION) {
        UT_ASSERT(dc != 0);
      } else {
        UT_ASSERT(dc == 0);
      }
    }
  }
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const ubyte uc = m_ucs[iKF];
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    UT_ASSERT(/*!(uc & GBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION) &&*/
              !(uc & GBA_FLAG_FRAME_UPDATE_SCHUR_COMPLEMENT) &&
              !(uc & GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION));
    UT_ASSERT(((uc & GBA_FLAG_FRAME_UPDATE_DEPTH) != 0) ==
              UT::VectorExistFlag<ubyte>(uds, Nx, GBA_FLAG_TRACK_UPDATE_DEPTH));
    UT_ASSERT(((uc & GBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION) != 0) ==
              UT::VectorExistFlag<ubyte>(uds, Nx, GBA_FLAG_TRACK_UPDATE_INFORMATION));
    UT_ASSERT(((uc & GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION) != 0) ==
              UT::VectorExistFlag<ubyte>(uds, Nx, GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION));
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const ubyte *_uds = m_uds.data() + m_iKF2d[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        UT_ASSERT((_uds[KF.m_zs[iz].m_ix] & GBA_FLAG_TRACK_INVALID) == 0);
      }
    }
    if (!chkSchur) {
      continue;
    }
    for (int ix = 0; ix < Nx; ++ix) {
      const ubyte ud = uds[ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
      const float a = KF.m_Axs[ix].m_Sadx.m_add.m_a;
      if (a > epsd) {
        UT_ASSERT(ud == 0);
      } else if (uds[ix] & GBA_FLAG_TRACK_INVALID) {
        UT_ASSERT(a == 0.0f);
      } else {
        UT_ASSERT(ud != 0);
      }
    }
  }
  
  ////const float s2dMax = BA_VARIANCE_MAX_DEPTH == 0.0f ? FLT_MAX :
  //const float s2dMax = BA_VARIANCE_MAX_DEPTH == 0.0f ? 0.0f :
  //                     BA_VARIANCE_MAX_DEPTH + DEPTH_VARIANCE_EPSILON + FLT_EPSILON;
  //for (int id = 0; id < Nd; ++id) {
  //  UT_ASSERT(m_ds[id].s2() <= s2dMax);
  //}

  const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  m_work.Resize(Nd * sizeof(FTR::Factor::DDC) / sizeof(float));
  AlignedVector<FTR::Factor::DDC> Sadxs((FTR::Factor::DDC *) m_work.Data(), Nd, false);
  Sadxs.MakeZero();
  for (int id = 0; id < Nd; ++id) {
    Sadxs[id].m_add.m_a = add;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      FTR::Factor::DDC *_Sadxs = Sadxs.Data() + m_iKF2d[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        _Sadxs[KF.m_zs[iz].m_ix] += KF.m_Azs2[iz].m_adx;
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    FTR::Factor::DDC *_Sadxs = Sadxs.Data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      if (KF.m_xs[ix].m_xr.Valid()) {
        _Sadxs[ix].m_add += KF.m_Ards[ix].m_add;
      } else
#endif
      {
        _Sadxs[ix].m_add += KF.m_Apds[ix];
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const FTR::Factor::DDC *_Sadxs = Sadxs.Data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = KF.m_Axs.Size();
    for (int ix = 0; ix < Nx; ++ix) {
      _Sadxs[ix].AssertEqual(KF.m_Axs[ix].m_Sadx, 1,
                             str + UT::String(" iKF %d ix %d Sadx", iKF, ix));
    }
  }

  const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
  AlignedVector<Camera::Factor::Unitary::CC> SAcus, SMcus;
  AlignedVector<Camera::Factor> SAcmsLM;
  m_work.Resize((SAcus.BindSize(nKFs) + SMcus.BindSize(nKFs) + SAcmsLM.BindSize(Nm)) /
                sizeof(float));
  SAcus.Bind(m_work.Data(), nKFs);      SAcus.MakeZero();
  SMcus.Bind(SAcus.BindNext(), nKFs);   SMcus.MakeZero();
  SAcmsLM.Bind(SMcus.BindNext(), nKFs); SAcmsLM.MakeZero();
//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator('*');
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    Camera::Factor::Unitary::CC &SAczz = SAcus[iKF], &SMczz = SMcus[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      const ubyte *_uds = m_uds.data() + m_iKF2d[_iKF];
      Camera::Factor::Unitary::CC &SAcxx = SAcus[_iKF];
//#ifdef CFG_DEBUG
#if 0
      Camera::Factor::Unitary::CC AcxxChk;
      AcxxChk.MakeZero();
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        AcxxChk += KF.m_Azs2[iz].m_Acxx;
      }
      if (m_iFrms[_iKF] == 24) {
        UT::PrintSeparator();
        UT::Print("Visual %d [%d]\n", iKF, KF.m_T.m_iFrm);
        UT::Print("%f + %f = %f\n", AcxxChk.m_A.m00(), SAcxx.m_A.m00(),
                                    AcxxChk.m_A.m00() + SAcxx.m_A.m00());
      }
#endif
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Factor::Full::A2 &Az = KF.m_Azs2[iz];
        SAcxx += Az.m_Acxx;
        SAczz += Az.m_Aczz;
        if (!(_uds[KF.m_zs[iz].m_ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO)) {
          SMczz += KF.m_Mzs2[iz].m_Mczz;
        }
//#ifdef CFG_DEBUG
#if 0
        if (iKF == 87) {
          UT::Print("%d %f\n", iz, SAczz.m_A.m00());
        }
#endif
      }
    }
    Camera::Factor::Unitary::CC &SMcxx = SMcus[iKF];
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = KF.m_Axs.Size();
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO)) {
        SMcxx += KF.m_Mxs2[ix].m_Mcxx;
      }
    }
  }

  Camera::Factor::Unitary::CC Acu;
  for (int iZp = 0; iZp < NZp; ++iZp) {
    const CameraPrior::Pose &Zp = m_Zps[iZp];
    const CameraPrior::Pose::Factor &Ap = m_Aps[iZp];
    const int N = static_cast<int>(Zp.m_iKFs.size());
    for (int i = -1, _i = 0; i < N; ++i, ++_i) {
      const int iKF = i == -1 ? Zp.m_iKFr : Zp.m_iKFs[i];
      Acu.Set(Ap.m_A[_i][_i], Ap.m_b[_i]);
//#ifdef CFG_DEBUG
#if 0
      if (m_iFrms[iKF] == 24) {
        UT::PrintSeparator();
        UT::Print("Prior %d\n", iZp);
        UT::Print("%f + %f = %f\n", Acu.m_A.m00(), SAcus[iKF].m_A.m00(),
                                    Acu.m_A.m00() + SAcus[iKF].m_A.m00());
      }
#endif
      SAcus[iKF] += Acu;
    }
  }
  if (m_ZpLM.Valid()) {
    const int im = m_ZpLM.m_iKF - m_Cs.Size() + m_CsLM.Size();
    SAcus[m_ZpLM.m_iKF].Increase3(m_ApLM.m_Arr.m_A, m_ApLM.m_Arr.m_b);
    Camera::Factor::Unitary &SAcm = SAcmsLM[im].m_Au;
    SAcm.m_Acm.Increase3(m_ApLM.m_Arm);
    SAcm.m_Amm += m_ApLM.m_Amm;
  }
  for (int ic1 = nKFs - m_CsLM.Size(), ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < nKFs;
       ic1 = ic2++, im1 = im2++) {
    const IMU::Delta::Factor &Ad = m_AdsLM[im2];
//#ifdef CFG_DEBUG
#if 0
    if (m_iFrms[ic1] == 24) {
      UT::PrintSeparator();
      UT::Print("IMU (%d, %d)\n", ic1, ic2);
      UT::Print("%f + %f = %f\n", Ad.m_A11.m_Acc.m_A.m00(), SAcus[ic1].m_A.m00(),
                                  Ad.m_A11.m_Acc.m_A.m00() + SAcus[ic1].m_A.m00());
    }
    if (m_iFrms[ic2] == 24) {
      UT::PrintSeparator();
      UT::Print("IMU (%d, %d)\n", ic1, ic2);
      UT::Print("%f + %f = %f\n", Ad.m_A22.m_Acc.m_A.m00(), SAcus[ic2].m_A.m00(),
                                  Ad.m_A22.m_Acc.m_A.m00() + SAcus[ic2].m_A.m00());
    }
#endif
    Camera::Factor::Unitary &SAcm1 = SAcmsLM[im1].m_Au;
    Camera::Factor::Unitary &SAcm2 = SAcmsLM[im2].m_Au;
    SAcus[ic1] += Ad.m_A11.m_Acc;
    SAcm1.m_Acm += Ad.m_A11.m_Acm;
    SAcm1.m_Amm += Ad.m_A11.m_Amm;
    SAcus[ic2] += Ad.m_A22.m_Acc;
    SAcm2.m_Acm += Ad.m_A22.m_Acm;
    SAcm2.m_Amm += Ad.m_A22.m_Amm;
    SAcmsLM[im2].m_Ab = m_SAcmsLM[im2].m_Ab;
  }
  const Camera::Fix::Zero zv[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INVALID)};
  const Camera::Fix::Zero zba[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID)};
  const Camera::Fix::Zero zbw[3] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID)};
  for (int im = 0; im < Nm; ++im) {
    const int i = m_ucmsLM[im] >> 5;
    Camera::Fix::Motion::Factor &A = m_AfmsLM[im];
    Camera::Factor::Unitary::MM &SA = SAcmsLM[im].m_Au.m_Amm;
    SA.m_A.IncreaseDiagonal012(zv[i].w());   SA.m_b.Increase(0, A.m_Av.m_b);
    SA.m_A.IncreaseDiagonal345(zba[i].w());  SA.m_b.Increase(3, A.m_Aba.m_b);
    SA.m_A.IncreaseDiagonal678(zbw[i].w());  SA.m_b.Increase(6, A.m_Abw.m_b);
  }
  const int iKFo = 0;
  if (m_iFrms[iKFo] == 0) {
//#ifdef CFG_DEBUG
#if 0
    UT::PrintSeparator();
    UT::Print("Origin\n");
    UT::Print("%f + %f = %f\n", m_Ao.m_A.m_A.m00(), SAcus[iKFo].m_A.m00(),
                                m_Ao.m_A.m_A.m00() + SAcus[iKFo].m_A.m00());
#endif
    SAcus[iKFo] += m_Ao.m_A;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SAcus[iKF].AssertEqual(m_SAcus[iKF], 1, str + UT::String(" SAcc[%d][%d]", iKF, iKF),
                           epsAbs, epsRel);
    SMcus[iKF].AssertEqual(m_SMcus[iKF], 1, str + UT::String(" SMcc[%d][%d]", iKF, iKF),
                           epsAbs, epsRel);
  }
  for (int im = 0; im < Nm; ++im) {
    SAcmsLM[im].AssertEqual(m_SAcmsLM[im], 1, str + UT::String(" SAcmLM[%d]", im), epsAbs, epsRel);
  }
  
  Camera::Factor::Binary::CC Acb;
  AlignedVector<Camera::Factor::Binary::CC> SAcbs;
  m_work.Resize(SAcbs.BindSize(Ncb) / sizeof(float));
  SAcbs.Bind(m_work.Data(), Ncb);
  SAcbs.MakeZero();
  for (int iZp = 0; iZp < NZp; ++iZp) {
    const CameraPrior::Pose &Zp = m_Zps[iZp];
    const CameraPrior::Pose::Factor &Ap = m_Aps[iZp];
    const int N = static_cast<int>(Zp.m_iKFs.size());
    for (int i = 0, _i = 1; i < N; ++i, ++_i) {
      const int iKF = Zp.m_iKFs[i];
      const int iKF1 = std::min(Zp.m_iKFr, iKF), iKF2 = std::max(Zp.m_iKFr, iKF);
      const int ip = m_KFs[iKF2].SearchPriorKeyFrame(iKF1);
      Ap.m_A[0][_i].Get(Acb, Zp.m_iKFr > iKF);
      SAcbs[m_iKF2cb[iKF2] + ip] += Acb;
    }
    for (int i1 = 0, _i1 = 1; i1 < N; ++i1, ++_i1) {
      const int iKF1 = Zp.m_iKFs[i1];
      for (int i2 = i1 + 1, _i2 = i2 + 1; i2 < N; ++i2, ++_i2) {
        const int iKF2 = Zp.m_iKFs[i2];
        const int ip = m_KFs[iKF2].SearchPriorKeyFrame(iKF1);
        SAcbs[m_iKF2cb[iKF2] + ip] += Ap.m_A[_i1][_i2];
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Camera::Factor::Binary::CC *SAps = SAcbs.Data() + m_iKF2cb[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Np = static_cast<int>(KF.m_iKFsPrior.size());
    for (int ip = 0; ip < Np; ++ip) {
      const int _iKF = KF.m_iKFsPrior[ip];
      SAps[ip].AssertEqual(KF.m_SAps[ip], 1, str + UT::String(" SAp[%d][%d]", _iKF, iKF),
                           epsAbs, epsRel);
    }
  }

  Camera::Factor::Binary::CC SAcxz;
  AlignedVector<Camera::Factor::Binary::CC> SMcbs;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    //const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
    const int Nk = KF.m_Zm.m_SMczms.Size();
    m_work.Resize(SMcbs.BindSize(Nk) / sizeof(float));
    SMcbs.Bind(m_work.Data(), Nk);
    SMcbs.MakeZero();
    m_marksTmp1.assign(KF.m_zs.size(), 0);
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      SAcxz.MakeZero();
      Camera::Factor::Binary::CC &SMcxz = SMcbs[Z.m_ik];
      const ubyte *_uds = m_uds.data() + m_iKF2d[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        SAcxz += KF.m_Azs2[iz].m_Acxz;
        if (_uds[KF.m_zs[iz].m_ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) {
          continue;
        }
        SMcxz += KF.m_Mzs2[iz].m_Mcxz;
        m_marksTmp1[iz] = 1;
      }
      SAcxz.AssertEqual(KF.m_SAcxzs[iZ], 1, str + UT::String(" SAcc[%d][%d]", _iKF, iKF),
                        epsAbs, epsRel);
    }
    for (int ik = 0; ik < Nk; ++ik) {
      Camera::Factor::Binary::CC &SMczm = SMcbs[ik];
      const int i1 = KF.m_Zm.m_ik2zm[ik], i2 = KF.m_Zm.m_ik2zm[ik + 1];
      for (int i = i1; i < i2; ++i) {
        if (m_marksTmp1[KF.m_Zm.m_izms[i].m_iz2]) {
          SMczm += KF.m_Zm.m_Mczms[i];
        }
      }
      const int _iKF = KF.m_iKFsMatch[ik];
      SMczm.AssertEqual(KF.m_Zm.m_SMczms[ik], 1, str + UT::String(" SMcc[%d][%d]", _iKF, iKF),
                        epsAbs, epsRel);
    }
  }
}
