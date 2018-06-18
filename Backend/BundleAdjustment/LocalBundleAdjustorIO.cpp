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
#ifndef CFG_DEBUG
//#define CFG_DEBUG
#endif
#include "LocalBundleAdjustor.h"
#include "IBA_internal.h"

#ifdef CFG_DEBUG
//#define LBA_DEBUG_ACTUAL_COST
#endif

//#ifdef CFG_DEBUG
#if 0
#ifdef LBA_ME_FUNCTION
#undef LBA_ME_FUNCTION
#define LBA_ME_FUNCTION ME::FUNCTION_NONE
#endif
#endif

#ifdef CFG_DEBUG
//#define LBA_DEBUG_CORRECT_ERROR 1
//#define LBA_DEBUG_CORRECT_ERROR 2
#endif

void LocalBundleAdjustor::SaveB(FILE *fp) {
  MT::Thread::SaveB(fp);
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  FRM::ListSaveB(m_ILFs1, fp);
  FRM::ListSaveB(m_ILFs2, fp);
  FRM::ListSaveB(m_IKFs1, fp);
  FRM::ListSaveB(m_IKFs2, fp);
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::VectorSaveB(m_hists, fp);
  //UT::SaveB(m_delta2, fp);
  UT::VectorSaveB(m_ic2LF, fp);
  UT::SaveB(m_Zo, fp);
  UT::SaveB(m_Ao, fp);
  FRM::VectorSaveB(m_LFs, fp);
  m_CsLF.SaveB(fp);
//#ifdef CFG_DEBUG
#if 0
  m_CsLF[10].Print(true);
#endif
#ifdef CFG_GROUND_TRUTH
  m_CsLFGT.SaveB(fp);
#endif
  UT::VectorSaveB(m_ucsLF, fp);
  UT::VectorSaveB(m_ucmsLF, fp);
#ifdef CFG_INCREMENTAL_PCG
  m_xcsLF.SaveB(fp);
  m_xmsLF.SaveB(fp);
#endif
  m_DsLF.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
  m_DsLFGT.SaveB(fp);
#endif
  m_AdsLF.SaveB(fp);
  m_AfpsLF.SaveB(fp);
  m_AfmsLF.SaveB(fp);
  FRM::VectorSaveB(m_KFs, fp);
  m_CsKF.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.SaveB(fp);
#endif
  UT::VectorSaveB(m_ucsKF, fp);
  m_usKF.SaveB(fp);
  UT::VectorSaveB(m_iKF2d, fp);
  UT::VectorSaveB(m_ds, fp);
  UT::VectorSaveB(m_uds, fp);
  m_Zp.SaveB(fp);
  m_ZpLF.SaveB(fp);
  m_ZpKF.SaveB(fp);
  m_ApLF.SaveB(fp);
  m_SAcusLF.SaveB(fp);
  m_SMcusLF.SaveB(fp);
  m_SAcmsLF.SaveB(fp);
  //UT::SaveB(m_F, fp);
}

void LocalBundleAdjustor::LoadB(FILE *fp) {
  MT::Thread::LoadB(fp);
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  FRM::ListLoadB(m_ILFs1, fp);
  FRM::ListLoadB(m_ILFs2, fp);
  FRM::ListLoadB(m_IKFs1, fp);
  FRM::ListLoadB(m_IKFs2, fp);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::VectorLoadB(m_hists, fp);
  //UT::LoadB(m_delta2, fp);
  UT::VectorLoadB(m_ic2LF, fp);
  UT::LoadB(m_Zo, fp);
  UT::LoadB(m_Ao, fp);
  FRM::VectorLoadB(m_LFs, fp);
  m_CsLF.LoadB(fp);
//#ifdef CFG_DEBUG
#if 0
  m_CsLF[10].Print(true);
#endif
#ifdef CFG_GROUND_TRUTH
  m_CsLFGT.LoadB(fp);
  if (m_CsGT && m_CsLFGT.Size() != m_CsLF.Size()) {
    const int nLFs = m_CsLF.Size();
    m_CsLFGT.Resize(nLFs);
    for (int iLF = 0; iLF < nLFs; ++iLF) {
      m_CsLFGT[iLF] = m_CsGT[m_LFs[iLF].m_T.m_iFrm];
    }
  }
#endif
  UT::VectorLoadB(m_ucsLF, fp);
  UT::VectorLoadB(m_ucmsLF, fp);
#ifdef CFG_INCREMENTAL_PCG
  m_xcsLF.LoadB(fp);
  m_xmsLF.LoadB(fp);
#endif
  m_DsLF.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
  m_DsLFGT.LoadB(fp);
  if (m_CsGT && m_DsLFGT.Size() != m_DsLF.Size()) {
    const int Nc = m_DsLF.Size();
    m_DsLFGT.Resize(Nc);
    m_DsLFGT[m_ic2LF[0]] = m_DsLF[m_ic2LF[0]];
    for (int ic1 = 0, ic2 = 1; ic2 < Nc; ic1 = ic2++) {
      const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
      const IMU::Delta &D = m_DsLF[iLF2];
      IMU::PreIntegrate(m_LFs[iLF2].m_us, m_LFs[iLF1].m_T.m_t, m_LFs[iLF2].m_T.m_t, m_CsLFGT[iLF1],
                        &m_DsLFGT[iLF2], &m_work, true, &D.m_u1, &D.m_u2);
    }
  }
#endif
  m_AdsLF.LoadB(fp);
  m_AfpsLF.LoadB(fp);
  m_AfmsLF.LoadB(fp);
  FRM::VectorLoadB(m_KFs, fp);
  m_CsKF.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.LoadB(fp);
  if (m_CsGT && m_CsKFGT.Size() != m_CsKF.Size()) {
    const int nKFs = m_CsKF.Size();
    m_CsKFGT.Resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      m_CsKFGT[iKF] = m_CsGT[m_KFs[iKF].m_T.m_iFrm].m_T;
    }
  }
#endif
  UT::VectorLoadB(m_ucsKF, fp);
  m_usKF.LoadB(fp);
  UT::VectorLoadB(m_iKF2d, fp);
  UT::VectorLoadB(m_ds, fp);
  UT::VectorLoadB(m_uds, fp);
  m_Zp.LoadB(fp);
  m_ZpLF.LoadB(fp);
  m_ZpKF.LoadB(fp);
  m_ApLF.LoadB(fp);
  m_SAcusLF.LoadB(fp);
  m_SMcusLF.LoadB(fp);
  m_SAcmsLF.LoadB(fp);
  //UT::LoadB(m_F, fp);
}

float LocalBundleAdjustor::PrintErrorStatistic(const std::string str,
                                               const AlignedVector<Camera> &CsLF,
                                               const AlignedVector<Rigid3D> &CsKF,
                                               const std::vector<Depth::InverseGaussian> &ds,
                                               const AlignedVector<IMU::Delta> &DsLF, bool detail) {
  ES _ES = ComputeErrorStatistic(CsLF, CsKF, ds, DsLF);
  const float F = _ES.Total();
  if (detail) {
    const std::string _str(str.size(), ' ');
    _ES.m_ESx.Print( str);
    _ES.m_ESm.Print(_str);
    _ES.m_ESd.Print(_str);
    UT::Print("%se  = %e\n", _str.c_str(), F);
  } else {
    _ES.m_ESx.m_ESx.Print(str, false, false, false, 0, false);
    UT::Print(" --> %e", F);
  }
  return F;
}

LocalBundleAdjustor::ES LocalBundleAdjustor::ComputeErrorStatistic(const AlignedVector<Camera>
                                                                   &CsLF, const AlignedVector<Rigid3D> &CsKF,
                                                                   const std::vector<Depth::InverseGaussian> &ds,
                                                                   const AlignedVector<IMU::Delta> &DsLF) {
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
  _ES.m_ESx = ComputeErrorStatisticFeaturePriorDepth(CsLF, CsKF, _ds.Data());
  _ES.m_ESm = ComputeErrorStatisticPriorCameraMotion(CsLF);
  _ES.m_ESd = ComputeErrorStatisticIMU(CsLF, DsLF);
  _ES.m_ESo = ComputeErrorStatisticFixOrigin(CsLF);
  _ES.m_ESfp = ComputeErrorStatisticFixPositionZ(CsLF);
  _ES.m_ESfm = ComputeErrorStatisticFixMotion(CsLF);
  return _ES;
}

FTR::ES LocalBundleAdjustor::ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Camera>
                                                                    &CsLF, const AlignedVector<Rigid3D> &CsKF,
                                                                    const Depth::InverseGaussian *ds) {
  FTR::ES ES;
  ES.Initialize();
  const int nLFs = int(m_LFs.size());
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    AccumulateErrorStatisticFeature(&m_LFs[iLF], CsLF[iLF].m_T, CsKF, ds, &ES, true);
  }
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    AccumulateErrorStatisticFeature(&m_KFs[iKF], CsKF[iKF], CsKF, ds, &ES, false);
  }
  AccumulateErrorStatisticPriorDepth(ds, &ES);
  return ES;
}

void LocalBundleAdjustor::AccumulateErrorStatisticFeature(const FRM::Frame *F, const Rigid3D &C,
                                                          const AlignedVector<Rigid3D> &CsKF,
                                                          const Depth::InverseGaussian *ds,
                                                          FTR::ES *ES, const bool localFrm) {
  //float SFx = 0.0f;
  Rigid3D Tr[2];
  FTR::Error e;
  float r2x, Fx;
  bool rx;
  const float wx = localFrm ? BA_WEIGHT_FEATURE : BA_WEIGHT_FEATURE_KEY_FRAME;
  const float r2Max = ME::Variance<LBA_ME_FUNCTION>();
  const int NZ = int(F->m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F->m_Zs[iZ];
    *Tr = C / CsKF[Z.m_iKF];
#ifdef CFG_STEREO
    Tr[1] = Tr[0];
    Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
    const KeyFrame &KF = m_KFs[Z.m_iKF];
    const Depth::InverseGaussian *_ds = ds + m_iKF2d[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
//#ifdef CFG_DEBUG
#if 0
      if (F->m_T.m_iFrm == 1010 && iz == 29) {
        UT::DebugStart();
        UT::DebugStop();
      }
#endif
      const FTR::Measurement &z = F->m_zs[iz];
      const int ix = z.m_ix;
      FTR::GetError(Tr, KF.m_xs[ix], _ds[ix], z, e);
      const FTR::ESIndex idx(KF.m_T.m_iFrm, ix, F->m_T.m_iFrm, iz);
#ifdef CFG_STEREO
      if (z.m_z.Valid()) {
        r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
        Fx = wx * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
        rx = r2x < r2Max;
//#ifdef CFG_DEBUG
#if 0
        UT::Print("%d: %e + %e = %e\n", iz, Fx, ES->m_ESx.m_SF, Fx + ES->m_ESx.m_SF);
#endif
        ES->Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
        //SFx += Fx;
      }
      if (z.m_zr.Valid()) {
        r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, e.m_exr);
        Fx = wx * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
        rx = r2x < r2Max;
//#ifdef CFG_DEBUG
#if 0
        UT::Print("%d: %e + %e = %e\n", iz, Fx, ES->m_ESx.m_SF, Fx + ES->m_ESx.m_SF);
#endif
        ES->Accumulate(m_K.m_Kr, e.m_exr, Fx, idx, rx);
        //SFx += Fx;
      }
#else
      r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
      //const float Fx = wx * ME::Cost<LBA_ME_FUNCTION>(r2x);
      Fx = wx * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
      rx = r2x < r2Max;
//#ifdef CFG_DEBUG
#if 0
      if (F->m_T.m_iFrm == 1)
        UT::Print("%d: %e + %e = %e\n", iz, Fx, ES->m_ESx.m_SF, Fx + ES->m_ESx.m_SF);
#endif
      ES->Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
#ifdef CFG_DEPTH_MAP
      if (z.m_d != 0.0f) {
        const float r2d = DEPTH_MAP_WEIGHT * e.m_ed * e.m_ed;
        //const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Cost<LBA_ME_FUNCTION>(r2d);
        const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<LBA_ME_FUNCTION>(r2d) * r2d;
        const bool rd = r2d < r2Max;
        ES->Accumulate(e.m_ed, Fd, idx, rd);
      }
#endif
#endif
//#ifdef CFG_DEBUG
#if 0
      if (F->m_T.m_iFrm == 117 && iz == 4) {
        Tr.Print("Tr = ", false);
        UT::Print("d  = %f\n", _ds[ix].u());
        (e.m_ex * LA::Vector2f(m_K.fx(), m_K.fy())).Print("ex = ", false);
      }
#endif
    }
  }
//#ifdef CFG_DEBUG
#if 0
  UT::Print("[%d] %e\n", F->m_T.m_iFrm, ES->Total());
#endif
}

void LocalBundleAdjustor::AccumulateErrorStatisticPriorDepth(const Depth::InverseGaussian *ds,
                                                             FTR::ES *ES) {
#ifdef CFG_STEREO
  LA::Vector2f e;
#endif
  const float r2Max = ME::Variance<LBA_ME_FUNCTION>();
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Depth::InverseGaussian *_ds = ds + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      const FTR::Source &x = KF.m_xs[ix];
      if (x.m_xr.Valid()) {
        FTR::GetError(m_K.m_br, _ds[ix], x, e);
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, e);
        const float F = BA_WEIGHT_FEATURE_KEY_FRAME * ME::Weight<LBA_ME_FUNCTION>(r2) * r2;
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        const bool r = r2 < r2Max;
        ES->Accumulate(m_K.m_Kr, e, F, idx, r);
      } else
#endif
      {
        const Depth::Prior _zp =
#ifdef CFG_DEPTH_MAP
          KF.m_xs[ix].m_d != 0.0f ? Depth::Prior(KF.m_xs[ix].m_d, DEPTH_MAP_WEIGHT) :
#endif
          zp;
        const float ed = _ds[ix].u() - _zp.m_d;
        const float r2d = _zp.m_w * ed * ed;
        //const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Cost<LBA_ME_FUNCTION>(r2d);
        const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<LBA_ME_FUNCTION>(r2d) * r2d;
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        const bool r = r2d < r2Max;
        ES->Accumulate(ed, Fd, idx, r);
      }
    }
  }
}

CameraPrior::Motion::ES LocalBundleAdjustor::ComputeErrorStatisticPriorCameraMotion(const
                                                                                    AlignedVector<Camera> &CsLF) {
  CameraPrior::Motion::ES ES;
  ES.Initialize();
  CameraPrior::Motion::Error e;
  const int iLF = m_ic2LF.front();
  m_ZpLF.GetError(CsLF[iLF], &e);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  const float F = m_ZpLF.GetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, e);
#else
  CameraPrior::Block::MM Amm = m_ZpLF.m_Amm;
  CameraPrior::Block::M em = m_ZpLF.m_bm;
  Amm.SolveLDL(em);
  CameraPrior::Block::M _e, Ae;
  e.Get(&_e);
  _e += em;
  e.Set(_e);
  LA::AlignedMatrix9x9f::Ab(m_ZpLF.m_Amm, _e, (float *) &Ae);
  const float F = BA_WEIGHT_PRIOR_CAMERA_MOTION * _e.Dot(Ae);
#endif
  ES.Accumulate(e, F, m_LFs[iLF].m_T.m_iFrm);
  return ES;
}

IMU::Delta::ES LocalBundleAdjustor::ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLF,
                                                             const AlignedVector<IMU::Delta> &DsLF) {
  IMU::Delta::ES ES;
  ES.Initialize();
  const int nLFs = int(m_LFs.size());
  for (int ic1 = 0, ic2 = 1; ic2 < nLFs; ic1 = ic2++) {
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    const IMU::Delta &D = DsLF[iLF2];
    const IMU::Delta::Error e = D.GetError(CsLF[iLF1], CsLF[iLF2], m_K.m_pu);
    const float F = D.GetCost(BA_WEIGHT_IMU, e);
    ES.Accumulate(e, F, m_LFs[iLF2].m_T.m_iFrm);
  }
  return ES;
}

Camera::Fix::Origin::ES LocalBundleAdjustor::ComputeErrorStatisticFixOrigin(const
                                                                            AlignedVector<Camera> &CsLF) {
  Camera::Fix::Origin::ES ES;
  ES.Initialize();
  const int iLF = m_ic2LF[0], iFrm = m_LFs[iLF].m_T.m_iFrm;
  if (iFrm == 0) {
    Camera::Fix::Origin::Error e;
    m_Zo.GetError(CsLF[iLF].m_T, e);
    const float F = m_Zo.GetCost(e);
    ES.Accumulate(e, F, iFrm);
  }
  return ES;
}

Camera::Fix::PositionZ::ES LocalBundleAdjustor::ComputeErrorStatisticFixPositionZ(const
                                                                                  AlignedVector<Camera> &CsLF) {
  Camera::Fix::PositionZ::ES ES;
  ES.Initialize();
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const int nLFs = static_cast<int>(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    const float pz = CsLF[iLF].m_p.z();
    ES.Accumulate(pz, z.GetCost(pz), m_LFs[iLF].m_T.m_iFrm);
  }
  return ES;
}

Camera::Fix::Motion::ES LocalBundleAdjustor::ComputeErrorStatisticFixMotion(const
                                                                            AlignedVector<Camera> &CsLF) {
  Camera::Fix::Motion::ES ES;
  ES.Initialize();
  const Camera::Fix::Zero zv[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL)};
  const Camera::Fix::Zero zba[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL)};
  const Camera::Fix::Zero zbw[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL)};
  const int nLFs = static_cast<int>(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    const int iFrm = m_LFs[iLF].m_T.m_iFrm;
    const int i = iFrm == 0 ? 1 : 0;
    const Camera &C = CsLF[iLF];
    ES.AccumulateVelocity(C.m_v, zv[i].GetCost(C.m_v), iFrm);
    ES.AccumulateBiasAcceleration(C.m_ba, zba[i].GetCost(C.m_ba), iFrm);
    ES.AccumulateBiasGyroscope(C.m_bw, zbw[i].GetCost(C.m_bw), iFrm);
  }
  return ES;
}

float LocalBundleAdjustor::PrintErrorStatistic(const std::string str,
                                               const AlignedVector<Camera> &CsLF,
                                               const AlignedVector<Rigid3D> &CsKF,
                                               const std::vector<Depth::InverseGaussian> &ds,
                                               const LA::AlignedVectorXf &xs,
                                               const bool detail) {
  const ES _ES = ComputeErrorStatistic(CsLF, CsKF, ds, xs, false);
  const float F = _ES.Total();
  if (detail) {
    const std::string _str(str.size(), ' ');
    _ES.m_ESx.Print( str/*, 0*/);
    _ES.m_ESm.Print(_str);
    _ES.m_ESd.Print(_str);
    UT::Print("%se  = %e", _str.c_str(), F);
  } else {
    UT::Print("%s%e", str.c_str(), F);
  }
  return F;
}

LocalBundleAdjustor::ES LocalBundleAdjustor::ComputeErrorStatistic(const AlignedVector<Camera>
                                                                   &CsLF, const AlignedVector<Rigid3D> &CsKF,
                                                                   const std::vector<Depth::InverseGaussian> &ds,
                                                                   const LA::AlignedVectorXf &xs,
                                                                   const bool updateOnly) {
  const int pc = 6, pm = 9;
  const int Nc = static_cast<int>(m_LFs.size()), Nd = static_cast<int>(m_ds.size());
  const LA::Vector6f *xcs = (LA::Vector6f *) xs.Data();
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  const float *xds = (float *) (xms + Nc);
  AlignedVector<Depth::InverseGaussian> _ds;
  LA::AlignedVectorXf _xds;
  ConvertCameraUpdates(xcs, &m_xcsP);
  //if (static_cast<int>(ds.size()) < Nd) {
  if (xs.Size() < Nc * (pc + pm) + Nd) {
    m_work.Resize(_ds.BindSize(Nd) + _xds.BindSize(Nd));
    _ds.Bind(m_work.Data(), Nd);
    _ds.Set(m_ds.data(), Nd);
    _xds.Bind(_ds.BindNext(), Nd);
    _xds.MakeZero();
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0, i = 0; iKF < nKFs; ++iKF) {
      const int iX = m_iKF2X[iKF];
      if (iX == -1) {
        continue;
      }
      const int id = m_iKF2d[iKF], Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
      memcpy(_ds.Data() + id, ds.data() + iX, Nx * sizeof(Depth::InverseGaussian));
      const ubyte *uds = m_uds.data() + id;
      float *__xds = _xds.Data() + id;
      for (int ix = 0; ix < Nx; ++ix) {
        if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
          __xds[ix] = xds[i++];
        }
      }
    }
  } else {
    _ds.Bind((Depth::InverseGaussian *) ds.data(), Nd);
    _xds.Bind((float *) xds, Nd);
  }
  ES _ES;
  _ES.m_ESx = ComputeErrorStatisticFeaturePriorDepth(CsLF, CsKF, _ds.Data(), m_xcsP, _xds, updateOnly);
  _ES.m_ESm = ComputeErrorStatisticPriorCameraMotion(m_xcsP.Data(), xms, updateOnly);
  _ES.m_ESd = ComputeErrorStatisticIMU(m_xcsP.Data(), xms, updateOnly);
  _ES.m_ESo = ComputeErrorStatisticFixOrigin(CsLF, m_xcsP, updateOnly);
  _ES.m_ESfp = ComputeErrorStatisticFixPositionZ(CsLF, m_xcsP.Data(), updateOnly);
  _ES.m_ESfm = ComputeErrorStatisticFixMotion(CsLF, xms, updateOnly);
  return _ES;
}

#ifdef LBA_DEBUG_ACTUAL_COST
static FTR::ES g_ESx;
#endif

FTR::ES LocalBundleAdjustor::ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Camera>
                                                                    &CsLF, const AlignedVector<Rigid3D> &CsKF,
                                                                    const Depth::InverseGaussian *ds,
                                                                    const AlignedVector<LA::ProductVector6f> &xcs,
                                                                    const LA::AlignedVectorXf &xds,
                                                                    const bool updateOnly) {
  FTR::ES ES;
  ES.Initialize();
#ifdef LBA_DEBUG_ACTUAL_COST
  g_ESx.Initialize();
#endif
  AccumulateErrorStatisticFeatureLF(CsLF, CsKF, ds, xcs, xds, &ES, updateOnly);
  AccumulateErrorStatisticFeatureKF(CsKF, ds, xds, &ES, updateOnly);
  AccumulateErrorStatisticPriorDepth(ds, xds, &ES, updateOnly);
  return ES;
}

void LocalBundleAdjustor::AccumulateErrorStatisticFeatureLF(const AlignedVector<Camera> &CsLF,
                                                            const AlignedVector<Rigid3D> &CsKF,
                                                            const Depth::InverseGaussian *ds,
                                                            const AlignedVector<LA::ProductVector6f> &xcs,
                                                            const LA::AlignedVectorXf &xds,
                                                            FTR::ES *ES, const bool updateOnly) {
//#ifdef CFG_DEBUG
#if 0
  float dF = 0.0f;
#endif
  FTR::Error e;
  float Fx;
#ifdef CFG_DEPTH_MAP
  float Fd;
#endif
  const float r2Max = ME::Variance<LBA_ME_FUNCTION>();
  const int Nc = int(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const LA::ProductVector6f *xc = !updateOnly || (m_ucsLF[iLF] & LBA_FLAG_FRAME_UPDATE_CAMERA) ?
                                    &xcs[ic] : NULL;
#ifdef LBA_DEBUG_ACTUAL_COST
    const Rigid3D C = xc ? Rigid3D(CsLF[iLF].m_T, &xc->Get012(), &xc->Get345()) : CsLF[iLF].m_T;
#endif
    //float SFx = 0.0f;
    const LocalFrame &LF = m_LFs[iLF];
    const int NZ = int(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
#ifdef LBA_DEBUG_ACTUAL_COST
      Rigid3D Tr[2];
      *Tr = C / CsKF[Z.m_iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
#endif
      const KeyFrame &KF = m_KFs[Z.m_iKF];
#ifdef LBA_DEBUG_ACTUAL_COST
      const Depth::InverseGaussian *_ds = ds + KF.m_id;
#endif
      const int id = m_iKF2d[Z.m_iKF];
      const ubyte *uds = m_uds.data() + id;
      const float *_xds = xds.Data() + id;
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
//#ifdef CFG_DEBUG
#if 0
        if (LF.m_T.m_iFrm == 1010 && iz == 29 ||
            LF.m_T.m_iFrm == 1037 && iz == 3) {
          UT::DebugStart();
          UT::DebugStop();
        }
#endif
        const FTR::Measurement &z = LF.m_zs[iz];
        const int ix = z.m_ix;
        const float *xd = !updateOnly || (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) ?
                          &_xds[ix] : NULL;
        const FTR::Factor::FixSource::L &L = LF.m_Lzs[iz];
        if (xc || xd) {
          Fx = FTR::GetCost(L, z, xc, xd, e);
        } else {
          Fx = L.m_F;
#ifdef CFG_STEREO
          if (z.m_z.Valid()) {
            e.m_ex = L.m_Je.m_ex;
          }
          if (z.m_zr.Valid()) {
            e.m_exr = L.m_Jer.m_ex;
          }
#else
          ex.m_ex = L.m_Je.m_ex;
#endif
        }
        //SFx += Fx;
//#ifdef CFG_DEBUG
#if 0
        dF = L.m_F - Fx + dF;
#endif
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix, LF.m_T.m_iFrm, iz);
#ifdef CFG_DEPTH_MAP
        if (z.m_d != 0.0f) {
          if (!xc && !xd) {
            e.m_ed = A.m_Je.m_ed;
          }
          Fd = A.m_Je.m_wd * DEPTH_MAP_WEIGHT * e.m_ed * e.m_ed;
          Fx -= Fd;
          ES->Accumulate(e.m_ed, Fd, idx);
        }
#endif
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          ES->Accumulate(m_K.m_K, e.m_ex, Fx, idx);
        }
        if (z.m_zr.Valid()) {
          ES->Accumulate(m_K.m_Kr, e.m_exr, 0.0f, idx);
        }
#else
        ES->Accumulate(m_K.m_K, ex.m_ex, Fx, idx);
#endif
#ifdef LBA_DEBUG_ACTUAL_COST
        Depth::InverseGaussian d = _ds[ix];
        if (xd) {
          d.u() += *xd;
        }
        FTR::GetError(Tr, KF.m_xs[ix], d, z, e);
//        Fx = FTR::GetCost(L, z, e);
//#ifdef CFG_STEREO
//        if (z.m_z.Valid()) {
//          g_ESx.Accumulate(m_K.m_K, e.m_ex, Fx, idx);
//        }
//        if (z.m_zr.Valid()) {
//          g_ESx.Accumulate(m_K.m_Kr, e.m_exr, 0.0f, idx);
//        }
//#else
//        g_ESx.Accumulate(m_K.m_K, ex.m_ex, Fx, idx);
//#endif
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
          const float Fx = BA_WEIGHT_FEATURE * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
          const bool rx = r2x < r2Max;
          g_ESx.Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
        }
        if (z.m_zr.Valid()) {
          const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, e.m_exr);
          const float Fx = BA_WEIGHT_FEATURE * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
          const bool rx = r2x < r2Max;
          g_ESx.Accumulate(m_K.m_Kr, e.m_exr, Fx, idx, rx);
        }
#else
        const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
        //const float Fx = BA_WEIGHT_FEATURE * ME::Cost<LBA_ME_FUNCTION>(r2x);
        const float Fx = BA_WEIGHT_FEATURE * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
        const bool rx = r2x < r2Max;
        g_ESx.Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
#ifdef CFG_DEPTH_MAP
        if (z.m_d != 0.0f) {
          const float r2d = DEPTH_MAP_WEIGHT * e.m_ed * e.m_ed;
          //const float Fd = GBA_WEIGHT_PRIOR_DEPTH * ME::Cost<LBA_ME_FUNCTION>(r2d);
          const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<LBA_ME_FUNCTION>(r2d) * r2d;
          const bool rd = r2d < r2Max;
          g_ESx.Accumulate(e.m_ed, Fd, idx, rd);
        }
#endif
#endif
#endif
      }
    }
    //UT::Print("[%d] %e\n", LF.m_T.m_iFrm, g_ESx.Total());
  }
}

void LocalBundleAdjustor::AccumulateErrorStatisticFeatureKF(const AlignedVector<Rigid3D> &CsKF,
                                                            const Depth::InverseGaussian *ds,
                                                            const LA::AlignedVectorXf &xds,
                                                            FTR::ES *ES, const bool updateOnly) {
//#ifdef CFG_DEBUG
#if 0
  float dF = 0.0f;
#endif
  FTR::Error e;
  float Fx;
#ifdef CFG_DEPTH_MAP
  float Fd;
#endif
  const float r2Max = ME::Variance<LBA_ME_FUNCTION>();
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef LBA_DEBUG_ACTUAL_COST
    const Rigid3D &C = CsKF[iKF];
#endif
    const KeyFrame &KF = m_KFs[iKF];
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF, id = m_iKF2d[_iKF];
      const KeyFrame &_KF = m_KFs[_iKF];
#ifdef LBA_DEBUG_ACTUAL_COST
      const Rigid3D _C = CsKF[_iKF];
      const Depth::InverseGaussian *_ds = ds + id;
      Rigid3D Tr[2];
      *Tr = C / _C;
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
#endif
      const ubyte *uds = m_uds.data() + id;
      const float *_xds = xds.Data() + id;
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        const float *xd = !updateOnly || (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) ?
                          &_xds[ix] : NULL;
        const FTR::Factor::Depth &A = KF.m_Azs[iz];
        if (xd) {
          Fx = FTR::GetCost(A, z, *xd, e);
        } else {
          Fx = A.m_F;
#ifdef CFG_STEREO
          if (z.m_z.Valid()) {
            e.m_ex = A.m_Je.m_ex;
          }
          if (z.m_zr.Valid()) {
            e.m_exr = A.m_Jer.m_ex;
          }
#else
          e.m_ex = A.m_Je.m_ex;
#endif
        }
//#ifdef CFG_DEBUG
#if 0
        dF = A.m_F - Fx + dF;
#endif
        const FTR::ESIndex idx(_KF.m_T.m_iFrm, ix, KF.m_T.m_iFrm, iz);
#ifdef CFG_DEPTH_MAP
        if (z.m_d != 0.0f) {
          if (!ud) {
            e.m_ed = A.m_Je.m_ed;
          }
          Fd = A.m_Je.m_wd * DEPTH_MAP_WEIGHT * e.m_ed * e.m_ed;
          Fx -= Fd;
          ES->Accumulate(e.m_ed, Fd, idx);
        }
#endif
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          ES->Accumulate(m_K.m_K, e.m_ex, Fx, idx);
        }
        if (z.m_zr.Valid()) {
          ES->Accumulate(m_K.m_Kr, e.m_exr, 0.0f, idx);
        }
#else
        ES.Accumulate(m_K.m_K, ex.m_ex, Fx, idx);
#endif
#ifdef LBA_DEBUG_ACTUAL_COST
        Depth::InverseGaussian d = _ds[ix];
        if (xd) {
          d.u() += *xd;
        }
        FTR::GetError(Tr, _KF.m_xs[ix], d, z, e);
//        Fx = FTR::GetCost(A, z, e);
//#ifdef CFG_STEREO
//        if (z.m_z.Valid()) {
//          g_ESx.Accumulate(m_K.m_K, e.m_ex, Fx, idx);
//        }
//        if (z.m_zr.Valid()) {
//          g_ESx.Accumulate(m_K.m_Kr, e.m_exr, 0.0f, idx);
//        }
//#else
//        g_ESx.Accumulate(m_K.m_K, ex.m_ex, Fx, idx);
//#endif
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
          const float Fx = BA_WEIGHT_FEATURE_KEY_FRAME * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
          const bool rx = r2x < r2Max;
          g_ESx.Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
        }
        if (z.m_zr.Valid()) {
          const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, e.m_exr);
          const float Fx = BA_WEIGHT_FEATURE_KEY_FRAME * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
          const bool rx = r2x < r2Max;
          g_ESx.Accumulate(m_K.m_Kr, e.m_exr, Fx, idx, rx);
        }
#else
        const float r2x = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_ex);
        //const float Fx = BA_WEIGHT_FEATURE * ME::Cost<LBA_ME_FUNCTION>(r2x);
        const float Fx = BA_WEIGHT_FEATURE_KEY_FRAME * ME::Weight<LBA_ME_FUNCTION>(r2x) * r2x;
        const bool rx = r2x < r2Max;
        _ES.Accumulate(m_K.m_K, e.m_ex, Fx, idx, rx);
#ifdef CFG_DEPTH_MAP
        if (z.m_d != 0.0f) {
          const float r2d = DEPTH_MAP_WEIGHT * e.m_ed * e.m_ed;
          //const float Fd = GBA_WEIGHT_PRIOR_DEPTH * ME::Cost<LBA_ME_FUNCTION>(r2d);
          const float Fd = BA_WEIGHT_PRIOR_DEPTH * ME::Weight<LBA_ME_FUNCTION>(r2d) * r2d;
          const bool rd = r2d < r2Max;
          _ES.Accumulate(e.m_ed, Fd, idx, rd);
        }
#endif
#endif
#endif
      }
    }
    //UT::Print("[%d] %e\n", KF.m_T.m_iFrm, g_ESx.Total());
  }
}

void LocalBundleAdjustor::AccumulateErrorStatisticPriorDepth(const Depth::InverseGaussian *ds,
                                                             const LA::AlignedVectorXf &xds,
                                                             FTR::ES *ES, const bool updateOnly) {
//#ifdef CFG_DEBUG
#if 0
  float dF = 0.0f;
#endif
  float e, F;
#ifdef CFG_STEREO
  LA::Vector2f er;
#endif
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id = m_iKF2d[iKF];
    const ubyte *uds = m_uds.data() + id;
    const float *_xds = xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const bool ud = !updateOnly || (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) != 0;
#ifdef CFG_STEREO
      if (KF.m_xs[ix].m_xr.Valid()) {
        const FTR::Factor::Stereo &a = KF.m_Ards[ix];
        if (ud) {
          F = FTR::GetCost(a, KF.m_xs[ix], _xds[ix], er);
        } else {
          er = a.m_Je.m_ex;
          F = a.m_F;
        }
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        ES->Accumulate(m_K.m_Kr, er, F, idx);
//#ifdef CFG_DEBUG
#if 0
        dF = a.m_F - F + dF;
#endif
      } else
#endif
      {
        const Depth::Prior::Factor &a = KF.m_Apds[ix];
        const Depth::Prior _zp =
#ifdef CFG_DEPTH_MAP
          KF.m_xs[ix].m_d != 0.0f ? Depth::Prior(KF.m_xs[ix].m_d, DEPTH_MAP_WEIGHT) :
#endif
          zp;
        if (ud) {
          F = _zp.GetCost(a, _xds[ix], e);
        } else {
          e = a.m_e;
          F = a.m_F;
        }
        const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
        ES->Accumulate(e, F, idx);
//#ifdef CFG_DEBUG
#if 0
        dF = a.m_F - F + dF;
#endif
      }
    }
  }
}

CameraPrior::Motion::ES LocalBundleAdjustor::ComputeErrorStatisticPriorCameraMotion(const
                                                                                    LA::ProductVector6f *xcs,
                                                                                    const LA::Vector9f *xms,
                                                                                    const bool updateOnly) {
  CameraPrior::Motion::ES ES;
  ES.Initialize();
  CameraPrior::Motion::Error e;
  const int iLF = m_ic2LF.front();
  const int iFrm = m_LFs[iLF].m_T.m_iFrm;
  const ubyte ucm = m_ucmsLF[iLF];
  const bool ur = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION) != 0;
  const bool uv = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) != 0;
  const bool uba = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) != 0;
  const bool ubw = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) != 0;
  if (ur || uv || uba || ubw) {
    LA::AlignedVector3f xr, xv, xba, xbw;
    const LA::ProductVector6f &xc = xcs[0];
    const LA::Vector9f &xm = xms[0];
    const LA::AlignedVector3f *_xr = ur ? &(xr = LA::AlignedVector3f(&xc.v3())) : NULL;
    const LA::AlignedVector3f *_xv = uv ? &(xv = LA::AlignedVector3f(&xm.v0())) : NULL;
    const LA::AlignedVector3f *_xba = uba ? &(xba = LA::AlignedVector3f(&xm.v3())) : NULL;
    const LA::AlignedVector3f *_xbw = ubw ? &(xbw = LA::AlignedVector3f(&xm.v6())) : NULL;
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    const float F = m_ZpLF.GetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_ApLF.m_Je, _xr, _xv, _xba, _xbw, &e);
#else
    m_ZpLF.GetError(m_ApLF.m_Je, _xr, _xv, _xba, _xbw, &e);
    CameraPrior::Block::MM Amm = m_ZpLF.m_Amm;
    CameraPrior::Block::M em = m_ZpLF.m_bm;
    Amm.SolveLDL(em);
    CameraPrior::Block::M _e, Ae;
    e.Get(&_e);
    _e += em;
    e.Set(_e);
    LA::AlignedMatrix9x9f::Ab(m_ZpLF.m_Amm, _e, (float *) &Ae);
    const float F = BA_WEIGHT_PRIOR_CAMERA_MOTION * _e.Dot(Ae);
#endif
    ES.Accumulate(e, F, iFrm);
  } else {
    ES.Accumulate(m_ApLF.m_Je.m_e, m_ApLF.m_F, iFrm);
  }
  return ES;
}

IMU::Delta::ES LocalBundleAdjustor::ComputeErrorStatisticIMU(const LA::ProductVector6f *xcs,
                                                             const LA::Vector9f *xms, const bool updateOnly) {
  IMU::Delta::ES ES;
  ES.Initialize();
//#ifdef CFG_DEBUG
#if 0
  float dF = 0.0f;
#endif
  IMU::Delta::Error e;
  float F;
  LA::AlignedVector3f xp[2], xr[2], xv[2], xba[2], xbw[2];
  LA::AlignedVector3f *_xp[2], *_xr[2], *_xv[2], *_xba[2], *_xbw[2];
  int r1 = 0, r2 = 1;
  
  const ubyte ucmFlag = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  const int nLFs = int(m_LFs.size());
  for (int ic1 = 0, ic2 = 1; ic2 < nLFs; ic1 = ic2++) {
    UT_SWAP(r1, r2);
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    if (ic1 == 0) {
      const ubyte ucm = m_ucmsLF[iLF1], uc = ucm & ucmFlag;
      _xp[r1] = !updateOnly || uc ? &(xp[r1] = LA::AlignedVector3f(&xcs[ic1].v0())) : NULL;
      _xr[r1] = !updateOnly || uc ? &(xr[r1] = LA::AlignedVector3f(&xcs[ic1].v3())) : NULL;
      _xv[r1] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
                &(xv[r1] = LA::AlignedVector3f(&xms[ic1].v0())) : NULL;
      _xba[r1] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
                 &(xba[r1] = LA::AlignedVector3f(&xms[ic1].v3())) : NULL;
      _xbw[r1] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
                 &(xbw[r1] = LA::AlignedVector3f(&xms[ic1].v6())) : NULL;
    }
    const ubyte ucm = m_ucmsLF[iLF2], uc = ucm & ucmFlag;
    _xp[r2] = !updateOnly || uc ? &(xp[r2] = LA::AlignedVector3f(&xcs[ic2].v0())) : NULL;
    _xr[r2] = !updateOnly || uc ? &(xr[r2] = LA::AlignedVector3f(&xcs[ic2].v3())) : NULL;
    _xv[r2] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
              &(xv[r2] = LA::AlignedVector3f(&xms[ic2].v0())) : NULL;
    _xba[r2] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
               &(xba[r2] = LA::AlignedVector3f(&xms[ic2].v3())) : NULL;
    _xbw[r2] = !updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
               &(xbw[r2] = LA::AlignedVector3f(&xms[ic2].v6())) : NULL;
    const IMU::Delta &D = m_DsLF[iLF2];
    const IMU::Delta::Factor &A = m_AdsLF[iLF2];
    if (!updateOnly || ucm || m_ucmsLF[iLF1]) {
      F = D.GetCost(BA_WEIGHT_IMU, A.m_Je, _xp[r1], _xr[r1], _xv[r1], _xba[r1], _xbw[r1],
                    _xp[r2], _xr[r2], _xv[r2], _xba[r2], _xbw[r2], e);
    } else {
      e = A.m_Je.m_e;
      F = A.m_F;
    }
    ES.Accumulate(e, F, m_LFs[iLF2].m_T.m_iFrm);
    //UT::Print("%d %e\n", ic2, F);
//#ifdef CFG_DEBUG
#if 0
    dF = A.m_F - F + dF;
#endif
  }
  return ES;
}

Camera::Fix::Origin::ES LocalBundleAdjustor::ComputeErrorStatisticFixOrigin(const
                                                                            AlignedVector<Camera> &CsLF,
                                                                            const AlignedVector<LA::ProductVector6f> &xcs,
                                                                            const bool updateOnly) {
  Camera::Fix::Origin::ES ES;
  ES.Initialize();
  const int iLF = m_ic2LF[0], iFrm = m_LFs[iLF].m_T.m_iFrm;
  if (iFrm == 0) {
    Camera::Fix::Origin::Error e;
    LA::AlignedVector3f xp, xr;
    xcs[0].Get(xp, xr);
    m_Zo.GetError(m_Ao.m_Je, xp, xr, e);
    const float F = m_Zo.GetCost(e);
    ES.Accumulate(e, F, iFrm);
  }
  return ES;
}

Camera::Fix::PositionZ::ES LocalBundleAdjustor::ComputeErrorStatisticFixPositionZ(const AlignedVector<Camera>
                                                                                  &CsLF, const LA::ProductVector6f *xcs,
                                                                                  const bool updateOnly) {
  Camera::Fix::PositionZ::ES ES;
  ES.Initialize();
  float F;
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const int Nc = static_cast<int>(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const float pz = CsLF[iLF].m_p.z();
    if (!updateOnly || (m_ucmsLF[iLF] & LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION)) {
      F = z.GetCost(pz, xcs[ic].v2());
    } else {
      F = m_AfpsLF[iLF].m_F;
    }
    ES.Accumulate(pz, F, m_LFs[iLF].m_T.m_iFrm);
  }
  return ES;
}

Camera::Fix::Motion::ES LocalBundleAdjustor::ComputeErrorStatisticFixMotion(const AlignedVector<Camera>
                                                                            &CsLF, const LA::Vector9f *xms,
                                                                            const bool updateOnly) {
  Camera::Fix::Motion::ES ES;
  ES.Initialize();
  LA::AlignedVector3f e;
  float F;
  const Camera::Fix::Zero zv[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL)};
  const Camera::Fix::Zero zba[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL)};
  const Camera::Fix::Zero zbw[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL)};
  const int Nc = static_cast<int>(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const Camera::Fix::Motion::Factor &A = m_AfmsLF[iLF];
    const Camera &C = CsLF[iLF];
    const LA::Vector9f &xm = xms[ic];
    const int iFrm = m_LFs[iLF].m_T.m_iFrm;
    const int i = iFrm == 0 ? 1 : 0;
    const ubyte ucm = m_ucmsLF[iLF];
    if (!updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY)) {
      F = zv[i].GetCost(C.m_v, &xm.v0(), e);
    } else {
      e = C.m_v;
      F = A.m_Av.F();
    }
    ES.AccumulateVelocity(e, F, iFrm);
    if (!updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION)) {
      F = zba[i].GetCost(C.m_ba, &xm.v3(), e);
    } else {
      e = C.m_ba;
      F = A.m_Aba.F();
    }
    ES.AccumulateBiasAcceleration(e, F, iFrm);
    if (!updateOnly || (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
      F = zbw[i].GetCost(C.m_bw, &xm.v6(), e);
    } else {
      e = C.m_bw;
      F = A.m_Abw.F();
    }
    ES.AccumulateBiasGyroscope(e, F, iFrm);
  }
  return ES;
}

void LocalBundleAdjustor::AssertConsistency(const bool chkFlag, const bool chkSchur) {
  m_solver->m_internal->AssertConsistency();

  //const float epsAbs = 1.0e-3f;
  const float epsAbs = 1.0e-2f;
  const float epsRel = 1.0e-3f;
  //const float epsRel = 1.0e-2f;
  //const float epsRel = 5.0e-2f;

  const FRM::Tag &T = m_LFs[m_ic2LF.back()].m_T;
  const int iFrm = T.m_iFrm;
  const std::string str = UT::String("LBA [%d]", iFrm);
  const int nLFs = static_cast<int>(m_LFs.size()), nKFs = static_cast<int>(m_KFs.size());
  const int STL = std::min(nLFs, LBA_MAX_SLIDING_TRACK_LENGTH);
  const int Nd = static_cast<int>(m_ds.size());
  UT_ASSERT(static_cast<int>(m_ic2LF.size()) == nLFs && m_CsLF.Size() == nLFs);
  UT_ASSERT(static_cast<int>(m_ucsLF.size()) == nLFs && static_cast<int>(m_ucmsLF.size()) == nLFs);
#ifdef CFG_INCREMENTAL_PCG
  UT_ASSERT(m_xcsLF.Size() == nLFs && m_xmsLF.Size() == nLFs);
#endif
  UT_ASSERT(m_DsLF.Size() == nLFs && m_AdsLF.Size() == nLFs);
  UT_ASSERT(m_AfpsLF.Size() == nLFs && m_AfmsLF.Size() == nLFs);
  UT_ASSERT(int(m_uds.size()) == Nd);
  UT_ASSERT(m_SAcusLF.Size() == nLFs && m_SMcusLF.Size() == nLFs && m_SAcmsLF.Size() == nLFs);
  UT_ASSERT(m_CsKF.Size() == nKFs && static_cast<int>(m_ucsKF.size()) == nKFs);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    UT_ASSERT(m_CsLFGT.Size() == nLFs && m_CsKFGT.Size() == nKFs && m_DsLFGT.Size() == nLFs);
  } else {
    UT_ASSERT(m_CsLFGT.Empty() && m_CsKFGT.Empty() && m_DsLFGT.Empty());
  }
  //if (m_dsGT) {
  //  UT_ASSERT(m_dsGT->size() == Nd);
  //}
#endif
  for (int ic = 0; ic < nLFs; ++ic) {
    m_CsLF[m_ic2LF[ic]].AssertConsistency(1, str + UT::String(" Cc[%d]", ic));
  }
  m_Zp.AssertConsistency();
  if (m_Zp.Pose::Valid()) {
    const LocalFrame &LF = m_LFs[m_ic2LF.front()];
    if (LF.m_T == m_KFs[LF.m_iKFNearest].m_T) {
      UT_ASSERT(m_Zp.m_iKFr != LF.m_iKFNearest);
    }
  }
  //m_ZpKF.AssertConsistency();

  if (!m_xsGN.Empty()) {
    ConvertCameraUpdates((float *) m_xsGN.Data(), (LA::AlignedVectorXf *) &m_xp2s, (LA::AlignedVectorXf *) &m_xr2s);
  }
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  const float epsdST = UT::Inverse(BA_VARIANCE_MAX_DEPTH_SLIDING_TRACK, BA_WEIGHT_FEATURE, eps);
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    const LocalFrame &LF = m_LFs[iLF];
    UT_ASSERT(ic == (iLF - m_ic2LF.front() + nLFs) % nLFs);
    if (ic > 0) {
      const FRM::Tag &_T = m_LFs[m_ic2LF[ic - 1]].m_T;
      UT_ASSERT(LF.m_T > _T);
      if (!LF.m_us.Empty()) {
        UT_ASSERT(LF.m_us.Front().t() >= _T.m_t);
      }
    }
    if (!LF.m_us.Empty()) {
      UT_ASSERT(LF.m_us.Back().t() <= LF.m_T.m_t);
    }
    UT_ASSERT(LF.m_iKFNearest != -1);
    UT_ASSERT(m_KFs[LF.m_iKFNearest].m_T <= LF.m_T);
    LF.AssertConsistency();
    if (!m_xsGN.Empty()) {
      const ubyte dc = m_ucsLF[iLF] & LBA_FLAG_FRAME_UPDATE_DELTA;
      if (m_xr2s[ic] > BA_BACK_SUBSTITUTE_ROTATION || m_xp2s[ic] > BA_BACK_SUBSTITUTE_POSITION) {
        UT_ASSERT(dc != 0);
      } else {
        UT_ASSERT(dc == 0);
      }
    }
    const int NZ = static_cast<int>(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const KeyFrame &KF = m_KFs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        UT_ASSERT(KF.m_Nsts[LF.m_zs[iz].m_ix] > 0);
      }
    }
    const int NkLF = std::min(STL, nLFs - ic) - 1;
    UT_ASSERT(static_cast<int>(LF.m_iLFsMatch.size()) == NkLF);
    for (int ik = 0; ik < NkLF; ++ik) {
      const int _iLF = m_ic2LF[ic + ik + 1];
      UT_ASSERT(LF.m_iLFsMatch[ik] == _iLF);
      const LocalFrame &_LF = m_LFs[_iLF];
      LF.m_Zm.AssertConsistency(ik, LF, _LF, m_izmsTmp);
    }
    m_marksTmp.assign(nKFs, 0);
    const int NkKF = static_cast<int>(LF.m_iKFsMatch.size());
    for (int ik = 0; ik < NkKF; ++ik) {
      const int iKF = LF.m_iKFsMatch[ik];
      if (m_KFs[iKF].m_T == LF.m_T) {
        UT_ASSERT(LF.m_iKFNearest == iKF);
      }
      m_marksTmp[iKF] = 1;
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (LF.SearchFrameMeasurement(iKF) != -1 ||
          FRM::Frame::HasFeatureMeasurementMatch(m_KFs[iKF], LF)) {
        UT_ASSERT(m_marksTmp[iKF] != 0);
      } else {
        //UT_ASSERT(m_marksTmp[iKF] == 0);
      }
    }
  }
  const ubyte ucmFlag = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  if (chkFlag) {
    for (int iLF = 0; iLF < nLFs; ++iLF) {
      const ubyte uc = m_ucsLF[iLF], ucm = m_ucmsLF[iLF];
      UT_ASSERT(!(uc & LBA_FLAG_FRAME_PUSH_TRACK) && !(uc & LBA_FLAG_FRAME_POP_TRACK) &&
                !(uc & LBA_FLAG_FRAME_UPDATE_DEPTH) &&
                !(uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION) &&
                !(uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION_KF) &&
                !(uc & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION));
      if (uc & LBA_FLAG_FRAME_UPDATE_CAMERA) {
        UT_ASSERT((ucm & ucmFlag) != 0);
      } else {
        UT_ASSERT((ucm & ucmFlag) == 0);
      }
      const LocalFrame &LF = m_LFs[iLF];
      const int NZ = int(LF.m_Zs.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const FRM::Measurement &Z = LF.m_Zs[iZ];
        const ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
        const KeyFrame &KF = m_KFs[Z.m_iKF];
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          UT_ASSERT((LF.m_ms[iz] & LBA_FLAG_MARGINALIZATION_UPDATE) == 0);
          const LocalFrame::SlidingTrack &ST = LF.m_STs[iz];
          const int iST0 = KF.m_ix2ST[LF.m_zs[iz].m_ix], iST1 = iST0 + ST.m_ist1, iST2 = iST0 + ST.m_ist2;
          const bool nonZero1 = UT::VectorExistFlagNot<ubyte>(KF.m_usST.data() + iST1, iST2 - iST1,
                                                              LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO);
          const bool nonZero2 = (LF.m_ms[iz] & LBA_FLAG_MARGINALIZATION_NON_ZERO) != 0;
          UT_ASSERT(nonZero1 == nonZero2);
        }
      }
      const int NI = int(LF.m_Zm.m_Is.size());
      for (int iI = 0; iI < NI; ++iI) {
        const MeasurementMatchLF::Index &I = LF.m_Zm.m_Is[iI];
        const KeyFrame &KF = m_KFs[I.m_iKF];
        const LocalFrame &_LF = m_LFs[LF.m_iLFsMatch[I.m_ik]];
        const int i1 = LF.m_Zm.m_iI2zm[iI], i2 = LF.m_Zm.m_iI2zm[iI + 1];
        for (int i = i1; i < i2; ++i) {
          UT_ASSERT((LF.m_Zm.m_ms[i] & LBA_FLAG_MARGINALIZATION_UPDATE) == 0);
          const FTR::Measurement::Match &izm = LF.m_Zm.m_izms[i];
          const LocalFrame::SlidingTrack &ST1 = LF.m_STs[izm.m_iz1], &ST2 = _LF.m_STs[izm.m_iz2];
          UT_ASSERT(ST1.m_ist1 <= ST2.m_ist1 && ST1.m_ist2 <= ST2.m_ist2 && ST2.m_ist1 < ST1.m_ist2);
          const int ix = LF.m_zs[izm.m_iz1].m_ix, iST0 = KF.m_ix2ST[ix];
          const int iST1 = iST0 + ST2.m_ist1, iST2 = iST0 + ST1.m_ist2;
          const bool nonZero1 = UT::VectorExistFlagNot<ubyte>(KF.m_usST.data() + iST1, iST2 - iST1,
                                                              LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO);
          const bool nonZero2 = (LF.m_Zm.m_ms[i] & LBA_FLAG_MARGINALIZATION_NON_ZERO) != 0;
          UT_ASSERT(nonZero1 == nonZero2);
        }
      }
    }
  }
#ifdef CFG_INCREMENTAL_PCG
  if (chkFlag) {
    for (int iLF = 0; iLF < nLFs; ++iLF) {
      if (m_ucsLF[iLF] & LBA_FLAG_FRAME_UPDATE_CAMERA) {
        UT_ASSERT(m_xcsLF[iLF].SquaredLength() == 0.0f);
      }
      const ubyte ucm = m_ucmsLF[iLF];
      const LA::Vector9f &xm = m_xmsLF[iLF];
      if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
        UT_ASSERT(xm.Get012().SquaredLength() == 0.0f);
      }
      if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
        UT_ASSERT(xm.Get345().SquaredLength() == 0.0f);
      }
      if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
        UT_ASSERT(xm.Get678().SquaredLength() == 0.0f);
      }
    }
  }
#endif

  for (int iKF = 0; iKF < nKFs; ++iKF) {
    m_CsKF[iKF].AssertOrthogonal(1, str + UT::String(" Rk[%d]", iKF));
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    if (iKF > 0) {
      UT_ASSERT(KF.m_T > m_KFs[iKF - 1].m_T);
      if (KF.m_iKFNearest != -1) {
        UT_ASSERT(m_KFs[KF.m_iKFNearest].m_T < KF.m_T);
      }
    }
    KF.AssertConsistency(iKF);
    m_marksTmp.assign(nKFs, 0);
    const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int _iKF = KF.m_iKFsMatch[ik];
      m_marksTmp[_iKF] = 1;
      const KeyFrame &_KF = m_KFs[_iKF];
      const std::vector<int>::const_iterator _ik = std::lower_bound(_KF.m_iKFsMatch.begin(),
                                                                    _KF.m_iKFsMatch.end(), iKF);
      UT_ASSERT(_ik != _KF.m_iKFsMatch.end() && *_ik == iKF);
    }
    for (int _iKF = 0; _iKF < iKF; ++_iKF) {
      if (KF.SearchFrameMeasurement(_iKF) != -1 ||
          FRM::Frame::HasFeatureMeasurementMatch(m_KFs[_iKF], KF)) {
        UT_ASSERT(m_marksTmp[_iKF] != 0);
      } else {
        //UT_ASSERT(m_marksTmp[_iKF] == 0);
      }
    }
    for (int _iKF = iKF + 1; _iKF < nKFs; ++_iKF) {
      if (m_KFs[_iKF].SearchFrameMeasurement(iKF) != -1 ||
          FRM::Frame::HasFeatureMeasurementMatch(m_KFs[_iKF], KF)) {
        UT_ASSERT(m_marksTmp[_iKF] != 0);
      } else {
        //UT_ASSERT(m_marksTmp[_iKF] == 0);
      }
    }
  }
  if (chkFlag) {
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const ubyte uc = m_ucsKF[iKF];
      const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
      const KeyFrame &KF = m_KFs[iKF];
      const int Nx = static_cast<int>(KF.m_xs.size());
      UT_ASSERT(!(uc & LBA_FLAG_FRAME_PUSH_TRACK) && !(uc & LBA_FLAG_FRAME_POP_TRACK) &&
                /*!(uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION) &&
                !(uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION_KF) &&*/
                !(uc & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_PUSH_TRACK) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_PUSH));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_POP_TRACK) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_POP));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_UPDATE_DEPTH) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_UPDATE_DEPTH));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_UPDATE_INFORMATION));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION_KF) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_UPDATE_INFORMATION_KF));
      UT_ASSERT(((uc & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION) != 0) ==
                UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION));
      if (!chkSchur) {
        continue;
      }
      for (int ix = 0; ix < Nx; ++ix) {
        const ubyte ud = uds[ix], zero = ud & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
        if (KF.m_Axs[ix].m_Sadd.m_a > epsd) {
          UT_ASSERT(zero == 0);
        } else {
          UT_ASSERT(zero != 0);
        }
        UT_ASSERT((KF.m_ms[ix] & LBA_FLAG_MARGINALIZATION_UPDATE) == 0);
        const int iST1 = KF.m_ix2ST[ix], iST2 = KF.m_ix2ST[ix + 1];
        const bool nonZero1 = UT::VectorExistFlagNot<ubyte>(KF.m_usST.data() + iST1, iST2 - iST1,
                                                            LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO);
        const bool nonZero2 = (KF.m_ms[ix] & LBA_FLAG_MARGINALIZATION_NON_ZERO) != 0;
        UT_ASSERT(nonZero1 == nonZero2);
        for (int iST = iST1; iST < iST2; ++iST) {
          const ubyte udST = KF.m_usST[iST];
          UT_ASSERT(!(udST & LBA_FLAG_TRACK_PUSH) && !(udST & LBA_FLAG_TRACK_POP) &&
                    !(udST & LBA_FLAG_TRACK_UPDATE_DEPTH));
          UT_ASSERT(/*!(udST & LBA_FLAG_TRACK_UPDATE_INFORMATION) &&*/
                    !(udST & LBA_FLAG_TRACK_UPDATE_INFORMATION_KF));
          const ubyte _udST = udST & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
          if (KF.m_AxsST[iST].m_Sadd.m_a > epsdST) {
            UT_ASSERT(_udST == 0);
          } else {
            UT_ASSERT(_udST != 0);
          }
        }
      }
    }
  }

  if (!m_usKF.Empty()) {
    const float t1 = m_usKF.Front().t(), t2 = m_usKF.Back().t();
    UT_ASSERT(t1 >= m_KFs.back().m_T.m_t && t2 <= T.m_t);

    int ic;
    for (ic = nLFs - 1; ic >= 0; --ic) {
      const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
      if (LF.m_T == m_KFs[LF.m_iKFNearest].m_T) {
        break;
      }
    }
    const int ick = ic;
    AlignedVector<float> &ts = m_work;
    ts.Resize(0);
    for (ic = ick + 1; ic < nLFs; ++ic) {
      const AlignedVector<IMU::Measurement> &us = m_LFs[m_ic2LF[ic]].m_us;
      const int N = us.Size();
      for (int i = 0; i < N; ++i) {
        ts.Push(us[i].t());
      }
    }
    const int N1 = m_usKF.Size(), N2 = ts.Size();
    if (ick < 0) {
      UT_ASSERT(N1 >= N2 && t1 <= ts.Front());
    } else {
      UT_ASSERT(N1 == N2);
    }
    for (int i1 = N1 - 1, i2 = N2 - 1; i1 >= 0 && i2 >= 0; --i1, --i2) {
      UT_ASSERT(m_usKF[i1].t() == ts[i2]);
    }
  }

  
  int SNd = 0;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    UT_ASSERT(m_iKF2d[iKF] == SNd);
    SNd += int(KF.m_xs.size());
  }
  UT_ASSERT(SNd == Nd && static_cast<int>(m_iKF2d.size()) == nKFs + 1 && m_iKF2d.back() == Nd);

  ////const float s2dMax = BA_VARIANCE_MAX_DEPTH == 0.0f ? FLT_MAX :
  //const float s2dMax = BA_VARIANCE_MAX_DEPTH == 0.0f ? 0.0f :
  //                     BA_VARIANCE_MAX_DEPTH + DEPTH_VARIANCE_EPSILON + FLT_EPSILON;
  //for (int id = 0; id < Nd; ++id) {
  //  UT_ASSERT(m_ds[id].s2() <= s2dMax);
  //}

  m_work.Resize(Nd * sizeof(FTR::Factor::DD) / sizeof(float));
  AlignedVector<FTR::Factor::DD> Sadds((FTR::Factor::DD *) m_work.Data(), Nd, false);
  Sadds.MakeZero();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    AccumulateFactorFeatureDD(&m_KFs[iKF], &Sadds, false);
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    FTR::Factor::DD *_Sadds = Sadds.Data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      if (KF.m_xs[ix].m_xr.Valid()) {
        _Sadds[ix] += KF.m_Ards[ix].m_add;
      } else
#endif
      {
        _Sadds[ix] += KF.m_Apds[ix];
      }
      if (_Sadds[ix].AssertEqual(KF.m_Axps[ix].m_Sadd, 1,
                                 str + UT::String(" iKF %d ix %d Axp", iKF, ix), epsAbs, epsRel)) {
#if !defined LBA_DEBUG_CORRECT_ERROR || LBA_DEBUG_CORRECT_ERROR <= 1
        continue;
#endif
      }
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR >= 1
      *((FTR::Factor::DD *) &KF.m_Axps[ix].m_Sadd) = _Sadds[ix];
      *((FTR::Factor::DD *) &KF.m_AxpsST[ix].m_Sadd) = KF.m_Axps[ix].m_Sadd *
                                                       (1.0f / std::max(KF.m_Nsts[ix], 1));
#endif
    }
  }
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    AccumulateFactorFeatureDD(&m_LFs[iLF], &Sadds, true);
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const FTR::Factor::DD *_Sadds = Sadds.Data() + m_iKF2d[iKF];
    KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (_Sadds[ix].AssertEqual(KF.m_Axs[ix].m_Sadd, 1,
                                 str + UT::String(" iKF %d ix %d Sadd", iKF, ix), epsAbs, epsRel)) {
#if !defined LBA_DEBUG_CORRECT_ERROR || LBA_DEBUG_CORRECT_ERROR <= 1
        continue;
#endif
      }
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR >= 1
      *((FTR::Factor::DD *) &KF.m_Axs[ix].m_Sadd) = _Sadds[ix];
#endif
    }
  }
  std::vector<int> &iKF2XST = m_idxsTmp1;
  iKF2XST.resize(nKFs + 1);
  iKF2XST[0] = 0;
  for (int iKF = 0, idST = 0; iKF < nKFs; ++iKF) {
    iKF2XST[iKF + 1] = iKF2XST[iKF] + int(m_KFs[iKF].m_STs.size());
  }
  const int NXST = iKF2XST.back();
  m_work.Resize(NXST * sizeof(FTR::Factor::DD) / sizeof(float));
  AlignedVector<FTR::Factor::DD> SaddsST((FTR::Factor::DD *) m_work.Data(), NXST, false);
  SaddsST.MakeZero();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    FTR::Factor::DD *_SaddsST = SaddsST.Data() + iKF2XST[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const FTR::Factor::DD SaddST = KF.m_AxpsST[ix].m_Sadd;
      const int iST1 = KF.m_ix2ST[ix], iST2 = KF.m_ix2ST[ix + 1];
      for (int iST = iST1; iST < iST2; ++iST) {
        _SaddsST[iST] = SaddST;
#if 0
//#if 1
        //if (m_iIter == 0 && KF.m_T.m_iFrm == 103 && ix == 8 && iST == iST1) {
        if (iKF == 0 && ix == 39 && iST == iST1 + 1) {
          UT::PrintSeparator();
          UT::Print("  SaddST = %e\n", SaddST.m_a);
        }
#endif
      }
    }
  }
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    const LocalFrame &LF = m_LFs[iLF];
    const int NZ = int(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      FTR::Factor::DD *_SaddsST = SaddsST.Data() + iKF2XST[Z.m_iKF];
      const KeyFrame &KF = m_KFs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Factor::DD &addST = LF.m_AzsST[iz].m_add;
        const LocalFrame::SlidingTrack &ST = LF.m_STs[iz];
        const int ix = LF.m_zs[iz].m_ix;
        const int iST0 = KF.m_ix2ST[ix], iST1 = iST0 + ST.m_ist1, iST2 = iST0 + ST.m_ist2;
        for (int iST = iST1; iST < iST2; ++iST) {
#if 0
//#if 1
          //if (m_iIter == 0 && KF.m_T.m_iFrm == 103 && ix == 8 && iST == iST0) {
          if (Z.m_iKF == 0 && ix == 4 && iST == iST0 + 1) {
            UT::Print("  SaddST = %e + %e = %e [%d]\n", addST.m_a, _SaddsST[iST].m_a,
                                                        addST.m_a + _SaddsST[iST].m_a, LF.m_T.m_iFrm);
          }
#endif
          _SaddsST[iST] += addST;
        }
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const FTR::Factor::DD *_SaddsST = SaddsST.Data() + iKF2XST[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const std::string _str = str + UT::String(" iKF %d ix %d", iKF, ix);
      const int iST1 = KF.m_ix2ST[ix], iST2 = KF.m_ix2ST[ix + 1];
      for (int iST = iST1; iST < iST2; ++iST) {
        if (_SaddsST[iST].AssertEqual(KF.m_AxsST[iST].m_Sadd, 1, 
                                      _str + UT::String(" ist %d SaddST", iST - iST1), epsAbs, epsRel)) {
#if !defined LBA_DEBUG_CORRECT_ERROR || LBA_DEBUG_CORRECT_ERROR <= 1
          continue;
#endif
        }
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR >= 1
        *((FTR::Factor::DD *) &KF.m_AxsST[iST].m_Sadd) = _SaddsST[iST];
#endif
      }
    }
  }
  std::vector<int> &iLF2c = m_idxsTmp1;
  iLF2c.resize(nLFs);
  for (int ic = 0; ic < nLFs; ++ic) {
    iLF2c[m_ic2LF[ic]] = ic;
  }
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    const int ic = iLF2c[iLF];
    const LocalFrame &LF = m_LFs[iLF];
    const int NZ = int(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const int iKF = Z.m_iKF;
      const KeyFrame &KF = m_KFs[iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = LF.m_zs[iz].m_ix;
        const FTR::Factor::DD &SmddST = LF.m_SmddsST[iz];
        const std::string _str = str + UT::String(" iKF %d ix %d ic %d iz %d SmddST", iKF, ix, ic, iz);
        if (LF.m_ms[iz] & LBA_FLAG_MARGINALIZATION_NON_ZERO) {
          FTR::Factor::DD _SmddST;
          _SmddST.MakeZero();
          const LocalFrame::SlidingTrack &ST = LF.m_STs[iz];
          const int iST0 = KF.m_ix2ST[ix], iST1 = iST0 + ST.m_ist1, iST2 = iST0 + ST.m_ist2;
          for (int iST = iST1; iST < iST2; ++iST) {
            if (KF.m_usST[iST] & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) {
              continue;
            }
            if (chkSchur) {
              const float a = KF.m_AxsST[iST].m_Sadd.m_a;
              const float m = KF.m_MxsST[iST].m_mdd.m_a;
              UT_ASSERT(m == 1.0f / a);
            }
#if 0
//#if 1
            //if (m_iIter == 0 && iLF == 5 && iz == 52) {
            if (m_iIter == 0 && LF.m_T.m_iFrm == 105 && iz == 52) {
              if (iST == iST1) {
                UT::PrintSeparator();
                UT::Print("iST = [%d, %d)\n", iST1, iST2);
              }
              UT::Print("  addST = %e, SmddST = %e + %e = %e\n", a, m, _SmddST.m_a, m + _SmddST.m_a);
            }
#endif
            _SmddST += KF.m_MxsST[iST].m_mdd;
          }
          if (_SmddST.AssertEqual(SmddST, 1, _str, epsAbs, epsRel)) {
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR <= 1
            continue;
#endif
          }
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR >= 1
          *((FTR::Factor::DD *) &SmddST) = _SmddST;
#endif
        } else {
          SmddST.AssertZero(1, _str);
        }
      }
    }
    const int NI = static_cast<int>(LF.m_Zm.m_Is.size());
    for (int iI = 0; iI < NI; ++iI) {
      const MeasurementMatchLF::Index &I = LF.m_Zm.m_Is[iI];
      const int iKF = I.m_iKF;
      const KeyFrame &KF = m_KFs[iKF];
      const int _iLF = LF.m_iLFsMatch[I.m_ik];
      const int _ic = iLF2c[_iLF];
      const LocalFrame &_LF = m_LFs[_iLF];
      const int i1 = LF.m_Zm.m_iI2zm[iI], i2 = LF.m_Zm.m_iI2zm[iI + 1];
      for (int i = i1; i < i2; ++i) {
        const FTR::Measurement::Match &izm = LF.m_Zm.m_izms[i];
        const float &SmddST = LF.m_Zm.m_SmddsST[i];
        const int ix = LF.m_zs[izm.m_iz1].m_ix;
        const std::string _str = str + UT::String(" iKF %d ix %d ic (%d, %d) iz (%d, %d) SmddST",
                                                  iKF, ix, ic, _ic, izm.m_iz1, izm.m_iz2);
        if (LF.m_Zm.m_ms[i] & LBA_FLAG_MARGINALIZATION_NON_ZERO) {
          float _SmddST = 0.0f;
          const LocalFrame::SlidingTrack &ST1 = LF.m_STs[izm.m_iz1], &ST2 = _LF.m_STs[izm.m_iz2];
          const int iST0 = KF.m_ix2ST[ix], iST1 = iST0 + ST2.m_ist1, iST2 = iST0 + ST1.m_ist2;
          for (int iST = iST1; iST < iST2; ++iST) {
            if (!(KF.m_usST[iST] & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO)) {
              _SmddST = KF.m_MxsST[iST].m_mdd.m_a + _SmddST;
            }
          }
          if (UT::AssertEqual(_SmddST, SmddST, 1, _str, epsAbs, epsRel)) {
#if !defined LBA_DEBUG_CORRECT_ERROR || LBA_DEBUG_CORRECT_ERROR <= 1
            continue;
#endif
          }
#if !defined LBA_DEBUG_CORRECT_ERROR && LBA_DEBUG_CORRECT_ERROR >= 1
          *((float *) &SmddST) = _SmddST;
#endif
        } else {
          UT::AssertZero(SmddST, 1, _str, -1.0f, -1.0f);
        }
      }
    }
  }

  Camera::Factor::Unitary::CC SAczz, SMczz;
  Camera::Factor::Binary::CC SMczm;
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    const LocalFrame &LF = m_LFs[iLF];
    SAczz.MakeZero();
    SMczz.MakeZero();
    const int Nz = int(LF.m_zs.size());
    for (int iz = 0; iz < Nz; ++iz) {
      SAczz += LF.m_Azs2[iz].m_Aczz;
      if (LF.m_ms[iz] & LBA_FLAG_MARGINALIZATION_NON_ZERO) {
        SMczz += LF.m_Mzs2[iz].m_Mczz;
      }
    }
    const int ic1 = ic - 1;
    if (ic1 >= 0) {
      SAczz += m_AdsLF[iLF].m_A22.m_Acc;
    }
    const int ic2 = ic + 1;
    if (ic2 < nLFs) {
      SAczz += m_AdsLF[m_ic2LF[ic2]].m_A11.m_Acc;
    }
    if (ic == 0) {
      SAczz.Increase3(m_ApLF.m_Arr.m_A, m_ApLF.m_Arr.m_b);
    }
    if (LF.m_T.m_iFrm == 0) {
      SAczz += m_Ao.m_A;
    }
    SAczz.AssertEqual(m_SAcusLF[iLF], 1, str + UT::String(" SAcc[%d][%d]", ic, ic),
                      epsAbs, epsRel);
    SMczz.AssertEqual(m_SMcusLF[iLF], 1, str + UT::String(" SMcc[%d][%d]", ic, ic),
                      epsAbs, epsRel);
    const int Nk = int(LF.m_iLFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      SMczm.MakeZero();
      const int _ic = iLF2c[LF.m_iLFsMatch[ik]];
      const int i1 = LF.m_Zm.m_ik2zm[ik], i2 = LF.m_Zm.m_ik2zm[ik + 1];
      for (int i = i1; i < i2; ++i) {
        if (LF.m_Zm.m_ms[i] & LBA_FLAG_MARGINALIZATION_NON_ZERO) {
          SMczm += LF.m_Zm.m_Mczms[i];
        }
      }
      SMczm.AssertEqual(LF.m_Zm.m_SMczms[ik], 1, str + UT::String(" SMcc[%d][%d]", ic, _ic),
                        epsAbs, epsRel);
    }
  }
}

void LocalBundleAdjustor::AccumulateFactorFeatureDD(const FRM::Frame *F,
                                                    AlignedVector<FTR::Factor::DD> *Sadds,
                                                    const bool localFrm) {
  const LocalFrame *LF = localFrm ? (LocalFrame *) F : NULL;
  const KeyFrame *KF = localFrm ? NULL : (KeyFrame *) F;
  const int NZ = int(F->m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F->m_Zs[iZ];
    FTR::Factor::DD *_Sadds = Sadds->Data() + m_iKF2d[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const int ix = F->m_zs[iz].m_ix;
      const FTR::Factor::DD &a = LF ? LF->m_Azs2[iz].m_add : KF->m_Azs[iz].m_add;
      FTR::Factor::DD &Sa = _Sadds[ix];
#if 0
//#if 1
      if (Z.m_iKF == 2 && ix == 68) {
        if (Sa.m_a == 0.0f) {
          UT::PrintSeparator();
        }
        UT::Print("[%d] %e + %e = %e\n", F->m_T.m_iFrm, a.m_a, Sa.m_a, a.m_a + Sa.m_a);
      }
#endif
      Sa += a;
    }
  }
}
