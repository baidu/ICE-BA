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
#include "GlobalBundleAdjustor.h"
#include "IBA_internal.h"

#ifdef CFG_DEBUG
#ifdef CFG_DEBUG_EIGEN
#define GBA_DEBUG_EIGEN
#endif
//#define GBA_DEBUG_CHECK
//#define GBA_DEBUG_PRINT
#ifdef CFG_GROUND_TRUTH
//#define GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
#endif
#endif
#if WIN32
#define GBA_DEBUG_VIEW
#endif
#ifdef GBA_DEBUG_VIEW
#include "ViewerIBA.h"
#endif

#ifdef GBA_DEBUG_CHECK
//static const int g_iKF = 87;
static const int g_iKF = INT_MAX;
static const int g_ix = 44;
static const Rigid3D *g_C = NULL;
static const Camera *g_CLM = NULL;
static const GlobalBundleAdjustor::KeyFrame *g_KF = NULL;
static const Camera::Factor::Unitary::CC *g_SAcu = NULL;
static const Camera::Factor *g_SAcmLM = NULL;
static const Depth::InverseGaussian *g_d = NULL;
static const FTR::Factor::Full::Source::A *g_Ax = NULL;
static const FTR::Factor::Full::Source::M1 *g_Mx1 = NULL;
static const FTR::Factor::Full::Source::M2 *g_Mx2 = NULL;
#endif

void GlobalBundleAdjustor::Initialize(IBA::Solver *solver, const int serial, const int verbose,
                                      const int debug, const int history) {
  MT::Thread::Initialize(serial, 5, "GBA");
  m_solver = solver;
  m_LM = &solver->m_internal->m_LM;
  m_GM = &solver->m_internal->m_GM;
  m_LBA = &solver->m_internal->m_LBA;
  m_K = solver->m_internal->m_K;
  m_verbose = verbose;
  m_debug = debug;
  m_history = history;
  m_dir = solver->m_internal->m_dir;
#ifdef CFG_GROUND_TRUTH
  m_CsGT = solver->m_internal->m_CsGT.Data();
  m_dsGT = solver->m_internal->m_dsGT.empty() ? NULL : &solver->m_internal->m_dsGT;
  if (m_history >= 3 && (!m_CsGT || !m_dsGT)) {
    m_history = 2;
  }
#endif
}

void GlobalBundleAdjustor::Reset() {
  MT::Thread::Reset();
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_GBA_Reset);
  m_IKFs1.resize(0);
  m_IKFs2.resize(0);
  m_ICs1.resize(0);
  m_ICs2.resize(0);
  m_IZps1.resize(0);
  m_IZps2.resize(0);
  m_IZpLM1.Invalidate();
  m_IZpLM2.Invalidate();
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_GBA_Reset);

  for (int i = 0; i < TM_TYPES; ++i) {
    m_ts[i].Reset(TIME_AVERAGING_COUNT);
  }
  m_hists.resize(0);
  m_CsDel.resize(0);

  m_delta2 = BA_DL_RADIUS_INITIAL;

  m_Zps.resize(0);
  m_Aps.resize(0);
  //m_ZpLM.Invalidate();
  m_ZpLM.m_iKF = 0;
  m_ZpLM.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY,
                    BA_VARIANCE_PRIOR_BIAS_ACCELERATION,
                    BA_VARIANCE_PRIOR_BIAS_GYROSCOPE);
#ifdef GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
  if (m_CsGT) {
    m_ZpLM.DebugSetMeasurement(m_CsGT[0]);
  }
#endif
  m_ApLM.MakeZero();

  m_KFs.resize(0);
  m_Cs.Resize(0);
  m_CsLM.Resize(0);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.Resize(0);
  m_CsLMGT.Resize(0);
#endif
  m_ucs.resize(0);
  m_ucmsLM.resize(0);
#ifdef CFG_INCREMENTAL_PCG
  m_xcs.Resize(0);
  m_xmsLM.Resize(0);
#endif
  m_DsLM.Resize(0);
#ifdef CFG_GROUND_TRUTH
  m_DsLMGT.Resize(0);
#endif
  m_AdsLM.Resize(0);

  m_Afps.Resize(0);
  m_AfmsLM.Resize(0);

  m_iKF2d.assign(1, 0);
  m_ds.resize(0);
  m_uds.resize(0);

  m_SAcus.Resize(0);
  m_SMcus.Resize(0);
  m_SAcmsLM.Resize(0);

  m_iKF2cb.assign(1, 0);

  //m_F = 0.0f;
}

void GlobalBundleAdjustor::PushKeyFrame(const GlobalMap::InputKeyFrame &IKF,
                                        const AlignedVector<IMU::Measurement> &us,
                                        const std::vector<Depth::InverseGaussian> &dzs) {
  MT_WRITE_LOCK_BEGIN(m_MT, IKF.m_T.m_iFrm, MT_TASK_GBA_PushKeyFrame);
#ifdef CFG_DEBUG
  UT_ASSERT(dzs.size() >= IKF.m_zs.size());
#endif
  //printf("%d -> ", m_IKFs1.size());
  m_IKFs1.push_back(InputKeyFrame(IKF, us, dzs));
  //printf("%d\n", m_IKFs1.size());
  MT_WRITE_LOCK_END(m_MT, IKF.m_T.m_iFrm, MT_TASK_GBA_PushKeyFrame);
}

void GlobalBundleAdjustor::DeleteKeyFrame(const int iKF, AlignedVector<IMU::Measurement> *us) {
  Synchronize();
#ifdef CFG_DEBUG
  UT_ASSERT(BufferDataEmpty());
#endif
  const int nKFs = static_cast<int>(m_KFs.size());
  if (iKF == nKFs - 1) {
    us->Set(m_KFs[iKF].m_us);
  } else {
    m_KFs[iKF + 1].m_us.Insert(0, m_KFs[iKF].m_us, &m_work);
  }
  const int Nd = static_cast<int>(m_KFs[iKF].m_xs.size());
  const int id1 = m_iKF2d[iKF], id2 = id1 + Nd;
  const ubyte *uds = m_uds.data() + id1;
  m_iKF2Z.resize(nKFs);
  for (int jKF2 = iKF + 1; jKF2 < nKFs; ++jKF2) {
    KeyFrame &KF2 = m_KFs[jKF2];
    const std::vector<FRM::Measurement>::iterator iZ2 = std::lower_bound(KF2.m_Zs.begin(),
                                                                         KF2.m_Zs.end(), iKF);
    m_iKF2Z[jKF2] = iZ2;
    const bool z2 = iZ2 != KF2.m_Zs.end() && iZ2->m_iKF == iKF;
    const int Nk = static_cast<int>(KF2.m_iKFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int jKF1 = KF2.m_iKFsMatch[ik];
      const std::vector<FRM::Measurement>::iterator iZ1 = m_iKF2Z[jKF1];
      const KeyFrame &KF1 = m_KFs[jKF1];
      const bool z1 = jKF1 > iKF && iZ1 != KF1.m_Zs.end() && iZ1->m_iKF == iKF;
      if (!z1 && !z2) {
        continue;
      }
      std::vector<FTR::Measurement::Match> &izms = KF2.m_Zm.m_izms;
      const int i1 = KF2.m_Zm.m_ik2zm[ik], i2 = KF2.m_Zm.m_ik2zm[ik + 1];
      if (z1) {
        const int Nz1 = iZ1->CountFeatureMeasurements();
        for (int i = i2 - 1; i >= i1 && izms[i].m_iz1 >= iZ1->m_iz2; --i) {
          izms[i].m_iz1 -= Nz1;
        }
      }
      if (z2) {
        const int Nz2 = iZ2->CountFeatureMeasurements();
        int i;
        for (i = i2 - 1; i >= i1 && izms[i].m_iz2 >= iZ2->m_iz2; --i) {
          izms[i].m_iz2 -= Nz2;
        }
        if (!z1) {
          continue;
        }
        const int j2 = i + 1;
        const int j1 = static_cast<int>(std::lower_bound(izms.begin() + i1,
                                                         izms.begin() + j2, iZ2->m_iz1) -
                                                         izms.begin());
        const int Nzm = j2 - j1;
        m_marksTmp.assign(Nzm, 0);
        ubyte *ms = m_marksTmp.data() - j1;
        for (int j = j1; j < j2; ++j) {
          const int ix = KF2.m_zs[izms[j].m_iz2].m_ix;
          ms[j] = (uds[ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) ? 0 : 1;
        }
        KF2.m_Zm.DeleteFeatureMeasurementMatches(ik, j1, j2, ms);
      }
    }
  }
  {
    KeyFrame &KF = m_KFs[iKF];
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      Camera::Factor::Unitary::CC &SAcxx = m_SAcus[_iKF];
      ubyte *_uds = m_uds.data() + m_iKF2d[_iKF];
      KeyFrame &_KF = m_KFs[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = KF.m_zs[iz].m_ix;
        FTR::Factor::Full::A2 &A = KF.m_Azs2[iz];
        A.m_Acxx.MakeMinus();
        SAcxx += A.m_Acxx;
        A.m_adx.MakeMinus();
        _KF.m_Axs[ix].m_Sadx += A.m_adx;
        _uds[ix] |= GBA_FLAG_TRACK_UPDATE_INFORMATION;
      }
      if (Z.m_iz1 < Z.m_iz2) {
        m_ucs[_iKF] |= GBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION;
      }
    }
  }
  int icb = m_iKF2cb[iKF];
  for (int jKF = iKF + 1; jKF < nKFs; ++jKF) {
    const std::vector<FRM::Measurement>::iterator iZ = m_iKF2Z[jKF];
    KeyFrame &KF = m_KFs[jKF];
    const int _iZ = iZ != KF.m_Zs.end() && iZ->m_iKF == iKF ?
                      static_cast<int>(iZ - KF.m_Zs.begin()) : -1;
    if (_iZ != -1) {
      Camera::Factor::Unitary::CC &SAczz = m_SAcus[jKF], &SMczz = m_SMcus[jKF];
      for (int iz = iZ->m_iz1; iz < iZ->m_iz2; ++iz) {
        Camera::Factor::Unitary::CC &Aczz = KF.m_Azs2[iz].m_Aczz;
        Aczz.MakeMinus();
        SAczz += Aczz;
        if (uds[KF.m_zs[iz].m_ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) {
          continue;
        }
        Camera::Factor::Unitary::CC &Mczz = KF.m_Mzs2[iz].m_Mczz;
        Mczz.MakeMinus();
        SMczz += Mczz;
      }
    }
    KF.DeleteKeyFrame(iKF, &iZ);
    const int _jKF = jKF - 2;
    if (_jKF >= 0 && !KF.m_us.Empty() &&
       (KF.m_iKFsMatch.empty() || KF.m_iKFsMatch.back() != _jKF)) {
      KF.InsertMatchKeyFrame(_jKF);
    }
    m_iKF2d[jKF] -= Nd;
    m_iKF2cb[jKF] = icb;
    icb += static_cast<int>(KF.m_ikp2KF.size());
  }
  m_CsDel.push_back(HistoryCamera(m_KFs[iKF].m_T, m_Cs[iKF]));
  m_KFs.erase(m_KFs.begin() + iKF);
  m_iKF2d.back() -= Nd;
  m_iKF2d.erase(m_iKF2d.begin() + iKF);
  m_iKF2cb.back() = icb;
  m_iKF2cb.erase(m_iKF2cb.begin() + iKF);
  m_Cs.Erase(iKF);
  const int Nm = m_CsLM.Size(), im = iKF - nKFs + Nm;
  if (im >= 0) {
    m_CsLM.Erase(im);
  }
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsKFGT.Erase(iKF);
    if (im >= 0) {
      m_CsLMGT.Erase(im);
    }
  }
#endif
#ifdef CFG_INCREMENTAL_PCG
  m_xcs.Erase(iKF);
  if (im >= 0) {
    m_xmsLM.Erase(im);
  }
#endif
  m_ucs.erase(m_ucs.begin() + iKF);
  if (im >= 0) {
    m_ucmsLM.erase(m_ucmsLM.begin() + im);
    const int im1 = im - 1, im2 = im + 1;
    if (im1 >= 0 && im2 < Nm) {
      KeyFrame &KF1 = m_KFs[iKF - 1], &KF2 = m_KFs[iKF];
      IMU::Delta &D = m_DsLM[im2];
      if ((KF2.m_T.m_t - KF1.m_T.m_t) > GBA_MAX_IMU_TIME_INTERVAL) {
        KF2.m_us.Clear();
        D.Invalidate();
        m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_INVALID;
      } else {
        IMU::PreIntegrate(KF2.m_us, KF1.m_T.m_t, KF2.m_T.m_t, m_CsLM[im1], &D, &m_work,
                          false, KF1.m_us.Empty() ? NULL : &KF1.m_us.Back());
      }
    }
    m_DsLM.Erase(im);
  }
  //m_Ucs.erase(m_Ucs.begin() + iKF);
  m_ds.erase(m_ds.begin() + id1, m_ds.begin() + id2);
  m_uds.erase(m_uds.begin() + id1, m_uds.begin() + id2);
  //m_Uds.erase(m_Uds.begin() + id1, m_Uds.begin() + id2);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT && im >= 0) {
    const int im1 = im - 1, im2 = im + 1;
    if (im1 >= 0 && im2 < Nm) {
      KeyFrame &KF1 = m_KFs[iKF - 1], &KF2 = m_KFs[iKF];
      IMU::Delta &D = m_DsLMGT[im2];
      if (KF2.m_us.Empty()) {
        D.Invalidate();
      } else {
        IMU::PreIntegrate(KF2.m_us, KF1.m_T.m_t, KF2.m_T.m_t, m_CsLMGT[im1], &D, &m_work,
                          false, KF1.m_us.Empty() ? NULL : &KF1.m_us.Back());
#ifdef GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
        D.DebugSetMeasurement(m_CsLMGT[im1], m_CsLMGT[im], m_K.m_pu);
        m_DsLM[im].DebugSetMeasurement(m_CsLMGT[im1], m_CsLMGT[im], m_K.m_pu);
#endif
      }
    }
    m_DsLMGT.Erase(im);
  }
  if (m_history >= 3) {
    m_ucsGT.erase(m_ucsGT.begin() + iKF);
    m_udsGT.erase(m_udsGT.begin() + id1, m_udsGT.begin() + id2);
  }
#endif
  m_Afps.Erase(iKF);
  m_SAcus.Erase(iKF);
  m_SMcus.Erase(iKF);
  if (im >= 0) {
    const int im1 = im - 1, im2 = im + 1;
    //Camera::Factor &SAcm = m_SAcmsLM[im];
    if (im1 >= 0) {
      IMU::Delta::Factor &A = m_AdsLM[im];
      A.m_A11.MakeMinus();
      m_SAcus[iKF - 1] += A.m_A11.m_Acc;
      Camera::Factor::Unitary &SAcm1 = m_SAcmsLM[im1].m_Au;
      SAcm1.m_Acm += A.m_A11.m_Acm;
      SAcm1.m_Amm += A.m_A11.m_Amm;
    }
    if (im2 < Nm) {
      IMU::Delta::Factor &A = m_AdsLM[im2];
      A.m_A22.MakeMinus();
      m_SAcus[iKF] += A.m_A22.m_Acc;
      Camera::Factor::Unitary &SAcm2 = m_SAcmsLM[im2].m_Au;
      SAcm2.m_Acm += A.m_A22.m_Acm;
      SAcm2.m_Amm += A.m_A22.m_Amm;
    }
    if (im1 >= 0 && im2 < Nm && m_DsLM[im].Valid()) {
      IMU::Delta::Factor::Auxiliary::Global U;
      Camera::Factor::Unitary &SAcm1 = m_SAcmsLM[im1].m_Au;
      Camera::Factor &SAcm2 = m_SAcmsLM[im2];
      IMU::Delta::Factor &A = m_AdsLM[im2];
      m_DsLM[im].GetFactor(BA_WEIGHT_IMU, m_CsLM[im1], m_CsLM[im], m_K.m_pu, &A, &SAcm2.m_Ab, &U);
      m_SAcus[iKF - 1] += A.m_A11.m_Acc;
      SAcm1.m_Acm += A.m_A11.m_Acm;
      SAcm1.m_Amm += A.m_A11.m_Amm;
      m_SAcus[iKF] += A.m_A22.m_Acc;
      SAcm2.m_Au.m_Acm += A.m_A22.m_Acm;
      SAcm2.m_Au.m_Amm += A.m_A22.m_Amm;
    } else if (im1 == -1) {
      m_SAcmsLM[im2].m_Ab.MakeZero();
    }
    m_AdsLM.Erase(im);
    m_AfmsLM.Erase(im);
    m_SAcmsLM.Erase(im);
  }
  int iZp, jZp;
  Camera::Factor::Unitary::CC Au, dAu;
  Camera::Factor::Binary::CC Ab, dAb;
  CameraPrior::Pose::Factor::Auxiliary Up;
  const int NZp1 = static_cast<int>(m_Zps.size());
  for (iZp = 0; iZp < NZp1; ++iZp) {
    CameraPrior::Pose &Zp = m_Zps[iZp];
    const std::vector<int>::iterator i = std::lower_bound(Zp.m_iKFs.begin(),
                                                          Zp.m_iKFs.end(), iKF);
    const int _i = static_cast<int>(i - Zp.m_iKFs.begin()) + 1;
    const bool del = Zp.m_iKFr == iKF || (i != Zp.m_iKFs.end() && *i == iKF);
    Zp.DeleteKeyFrame(iKF, &i);
    const int N = static_cast<int>(Zp.m_iKFs.size());
    if (!del && N != 0) {
      continue;
    }
    CameraPrior::Pose::Factor &Ap = m_Aps[iZp];
    Ap.Swap(m_ApTmp, m_bpTmp);
    const bool nonZero = Zp.Valid() && N != 0;
    if (nonZero) {
      m_ApTmp.Erase(_i);
      m_bpTmp.Erase(_i);
      m_work.Resize(Up.BindSize(N) / sizeof(float));
      Up.Bind(m_work.Data(), N);
      Zp.GetFactor(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Cs, &Ap, &Up);
    }
    for (int j = -1, _j = 0; j < N; ++j, ++_j) {
      const int jKF = j == -1 ? Zp.m_iKFr : Zp.m_iKFs[j];
      if (jKF != -1) {
        dAu.Set(m_ApTmp[_j][_j], m_bpTmp[_j]);
        if (nonZero) {
          Au.Set(Ap.m_A[_j][_j], Ap.m_b[_j]);
          Camera::Factor::Unitary::CC::AmB(Au, dAu, dAu);
        } else {
          dAu.MakeMinus();
        }
        m_SAcus[jKF] += dAu;
      }
      if (j == -1) {
        continue;
      }
      if (Zp.Valid()) {
        const int iKF1 = std::min(Zp.m_iKFr, jKF), iKF2 = std::max(Zp.m_iKFr, jKF);
        KeyFrame &KF2 = m_KFs[iKF2];
        m_ApTmp[0][_j].Get(dAb, Zp.m_iKFr > jKF);
        if (nonZero) {
          Ap.m_A[0][_j].Get(Ab, Zp.m_iKFr > jKF);
          Camera::Factor::Binary::CC::AmB(Ab, dAb, dAb);
        } else {
          dAb.MakeMinus();
        }
        const int ip = KF2.SearchPriorKeyFrame(iKF1);
        KF2.m_SAps[ip] += dAb;
      }
    }
    for (int i1 = 0, _i1 = 1; i1 < N; ++i1, ++_i1) {
      const int iKF1 = Zp.m_iKFs[i1];
      for (int i2 = i1 + 1, _i2 = i2 + 1; i2 < N; ++i2, ++_i2) {
        const int iKF2 = Zp.m_iKFs[i2];
        KeyFrame &KF = m_KFs[iKF2];
        const int ip = KF.SearchPriorKeyFrame(iKF1);
        if (nonZero) {
          Camera::Factor::Binary::CC::AmB(Ap.m_A[_i1][_i2], m_ApTmp[_i1][_i2], dAb);
        } else {
          m_ApTmp[_i1][_i2].GetMinus(dAb);
        }
        KF.m_SAps[ip] += dAb;
      }
    }
  }
  for (iZp = jZp = 0; iZp < NZp1; ++iZp) {
    const CameraPrior::Pose &Zp = m_Zps[iZp];
    if (Zp.Invalid() || Zp.m_iKFs.empty()) {
      continue;
    } else if (jZp < iZp) {
      m_Zps[jZp] = Zp;
      m_Aps[jZp] = m_Aps[iZp];
    }
    ++jZp;
  }
  const int NZp2 = jZp;
  m_Zps.resize(NZp2);
  m_Aps.resize(NZp2);
  if (m_ZpLM.DeleteKeyFrame(iKF)) {
    m_ApLM.MakeZero();
  }
  std::list<CameraPrior::Pose>::iterator IZp = m_IZps1.begin(), IZpNext;
  while (IZp != m_IZps1.end()) {
    IZpNext = IZp;
    IZpNext++;
    IZp->DeleteKeyFrame(iKF);
    if (IZp->Invalid()) {
      m_IZps1.erase(IZp);
    }
    IZp = IZpNext;
  }
  if (m_IZpLM1.m_iKF == iKF) {
    m_IZpLM1.Invalidate();
  }
#ifdef CFG_DEBUG
  if (m_debug) {
    m_xsGN.Resize(0);
    AssertConsistency(true, false);
  }
#endif
}

void GlobalBundleAdjustor::UpdateCameras(const int iFrm, const std::vector<InputCamera> &Cs) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GBA_UpdateCameras);
  m_ICs1.insert(m_ICs1.end(), Cs.begin(), Cs.end());
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GBA_UpdateCameras);
}

void GlobalBundleAdjustor::PushCameraPriorPose(const int iFrm, const CameraPrior::Pose &Zp) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GBA_PushCameraPriorPose);
  m_IZps1.push_back(Zp);
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GBA_PushCameraPriorPose);
}

void GlobalBundleAdjustor::PushCameraPriorMotion(const int iFrm, const int iKF, const CameraPrior::Motion &Zp) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GBA_PushCameraPriorMotion);
  m_IZpLM1.Set(iKF, Zp);
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GBA_PushCameraPriorMotion);
}

void GlobalBundleAdjustor::Run() {
#if 0
//#if 1
  AssertConsistency();
#endif
  m_delta2 = BA_DL_RADIUS_INITIAL;
  m_ts[TM_TOTAL].Start();
  SynchronizeData();
  m_ts[TM_TOTAL].Stop();
#ifdef CFG_DEBUG
  if (m_debug < 0) {
    return;
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  {
    std::vector<float> ts;
    const int Nm = m_CsLM.Size();
    for (int im1 = 0, im2 = 1; im2 < Nm; im1 = im2++)
      ts.push_back(m_DsLM[im2].m_Tpv);
    UT::SaveValues("D:/tmp/test.txt", ts);
    exit(0);
  }
#endif
#ifdef GBA_DEBUG_EIGEN
  DebugGenerateTracks();
#endif
#ifdef GBA_DEBUG_CHECK
  {
    const int nKFs = int(m_KFs.size());
    if (nKFs != 0) {
      const int iKF = g_iKF >= 0 && g_iKF < nKFs ? g_iKF : nKFs - 1;
      g_C = &m_Cs[iKF];
      g_KF = &m_KFs[iKF];
      g_SAcu = &m_SAcus[iKF];
      const int im = iKF - (nKFs - m_CsLM.Size());
      if (im >= 0) {
        g_CLM = &m_CsLM[im];
        g_SAcmLM = &m_SAcmsLM[im];
      }
      if (g_ix >= 0 && g_ix < static_cast<int>(g_KF->m_xs.size())) {
        g_d = m_ds.data() + m_iKF2d[iKF] + g_ix;
        g_Ax = &g_KF->m_Axs[g_ix];
        g_Mx1 = &g_KF->m_Mxs1[g_ix];
        g_Mx2 = &g_KF->m_Mxs2[g_ix];
      }
    }
  }
#endif
  const int iFrm = m_KFs.back().m_T.m_iFrm;
#ifdef CFG_VERBOSE
  if (m_verbose >= 2) {
    UT::PrintSeparator('*');
    UT::Print("[%d] [GlobalBundleAdjustor::Run]\n", iFrm);
    UT::Print("  Frame = %d\tMeasurement = %d + %d\tSchur = %d\tTrack = %d\n", m_Cs.Size(),
              CountMeasurementsFeature(), CountMeasurementsPriorCameraPose(),
              CountSchurComplements(), m_ds.size());
#ifdef CFG_GROUND_TRUTH
    if (!m_CsKFGT.Empty() && m_dsGT) {
      UT::PrintSeparator();
      PrintErrorStatistic("*GT: ", m_CsKFGT, m_CsLMGT, *m_dsGT, m_DsLMGT, true);
    }
#endif
  }
#endif
#if 0
//#if 1
  const int Nc = m_Cs.Size();
  //if (Nc == 2) {
  if (Nc >= 2) {
    UT::Check("Noise\n");
    //const float erMax1 = 0.0f;
    const float erMax1 = 0.5f;
    //const float erMax2 = 0.0f;
    const float erMax2 = 0.5f;
    //const float erMax2 = 10.0f;
    //const float epMax1 = 0.0f;
    const float epMax1 = 0.05f;
    //const float epMax2 = 0.0f;
    const float epMax2 = 0.05f;
    //const float epMax2 = 1.0f;
    const float evMax = 0.0f;
    //const float evMax = 1.0f;
    const float ebaMax = 0.0f;
    //const float ebaMax = 0.001f;
    const float ebwMax = 0.0f;
    const float edMax = 0.0f;
    //const float edMax = 0.1f;
    //const float ebwMax = 1.0f * UT_FACTOR_DEG_TO_RAD;
    Rotation3D dR;
    LA::AlignedVector3f dp;
    dR.MakeIdentity();
    dp.MakeZero();
    for (int ic = 0, im = m_CsLM.Size() - Nc; ic < Nc; ++ic, ++im) {
      Rigid3D &T = m_Cs[ic];
      Point3D p = T.GetPosition();
//#if 0
#if 1
      if (ic > 0)
#endif
      {
        dR = Rotation3D::GetRandom(erMax1 * UT_FACTOR_DEG_TO_RAD) * dR;
        T = dR * Rotation3D::GetRandom(erMax1 * UT_FACTOR_DEG_TO_RAD) * T;
      }
      //dp.Random(epMax);
      dp += LA::AlignedVector3f::GetRandom(epMax1);
      p += dp + LA::AlignedVector3f::GetRandom(epMax2);
      T.SetPosition(p);
      m_ucs[ic] |= GBA_FLAG_FRAME_UPDATE_CAMERA | GBA_FLAG_FRAME_UPDATE_DEPTH;
      if (im == -1) {
        continue;
      }
      Camera &C = m_CsLM[im];
      C.m_T = T;
      C.m_p = p;
      C.m_v += LA::AlignedVector3f::GetRandom(evMax);
      C.m_ba += LA::AlignedVector3f::GetRandom(ebaMax);
      C.m_bw += LA::AlignedVector3f::GetRandom(ebwMax);
#ifdef CFG_DEBUG
      C.m_v.MakeZero();
      C.m_ba.MakeZero();
      C.m_bw.MakeZero();
#endif
      m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION | GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
    }
    const int Nd = int(m_ds.size());
    for (int id = 0; id < Nd; ++id) {
      m_ds[id].u() += UT::Random<float>(edMax);
      m_uds[id] |= GBA_FLAG_TRACK_UPDATE_DEPTH;
    }
  }
#endif
#ifdef GBA_DEBUG_VIEW
  ViewerIBA *viewer = NULL;
  if (m_verbose >= 2) {
    viewer = new ViewerIBA();
    viewer->Create(m_solver/*, "D:/tmp/backend.png", 2*/);
    //viewer->Start("D:/tmp/view.txt");
    viewer->ActivateFrame(viewer->GetKeyFrames() + viewer->GetLocalFrames() - 1);
    viewer->ActivateFrame(viewer->GetKeyFrames() - 1);
    viewer->m_keyPause = true;
    viewer->m_keyDrawCamTypeKF = ViewerIBA::DRAW_CAM_KF_GBA;
    viewer->m_keyDrawDepType = ViewerIBA::DRAW_DEP_GBA;
  }
#endif
#if 0
//#if 1
  {
    UT::Check("Noise\n");
    UpdateFactors();
    UT::PrintSeparator();
    PrintErrorStatistic("    ", m_Cs, m_CsLM, m_ds, m_DsLM, false);
    UT::Print("\n");
    const float edMax = 1.0f;
    const int Nd = int(m_ds.size());
    for (int id = 0; id < Nd; ++id) {
      if (m_uds[id] & GBA_FLAG_TRACK_UPDATE_DEPTH) {
        m_ds[id].u() += UT::Random<float>(edMax);
      }
    }
    UT::PrintSeparator();
    PrintErrorStatistic("--> ", m_Cs, m_CsLM, m_ds, m_DsLM, false);
    UT::Print("\n");
#ifdef GBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run();
    }
#endif
    EmbeddedPointIteration();
    UT::PrintSeparator();
    PrintErrorStatistic("--> ", m_Cs, m_CsLM, m_ds, m_DsLM, false);
    UT::Print("\n");
#ifdef GBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run();
    }
#endif
  }
#endif
  m_ts[TM_TOTAL].Start();
  for (m_iIter = 0; m_iIter < BA_MAX_ITERATIONS; ++m_iIter) {
//#ifdef CFG_DEBUG
#if 0
//#if 1
    if (m_iIter == 0) {
      UT::Check("Update\n");
      m_Ao.MakeZero();
      const int NZp = static_cast<int>(m_Aps.size());
      for (int iZp = 0; iZp < NZp; ++iZp) {
        m_Aps[iZp].MakeZero();
      }
      if (m_ZpLM.Valid()) {
        m_ApLM.MakeZero();
      }
      const int nKFs = static_cast<int>(m_KFs.size());
      for (int iKF = 0; iKF < nKFs; ++iKF) {
        m_ucs[iKF] |= GBA_FLAG_FRAME_UPDATE_CAMERA | GBA_FLAG_FRAME_UPDATE_DEPTH;
        m_Ucs[iKF] |= GM_FLAG_FRAME_UPDATE_CAMERA/* | GM_FLAG_FRAME_UPDATE_DEPTH*/;
        m_KFs[iKF].MakeZero();
      }
      for (int ic = nKFs - m_CsLM.Size(), im = 0; ic < nKFs; ++ic, ++im) {
        m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION | GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION | GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
        if (im == 0) {
          continue;
        }
        IMU::Delta &D = m_DsLM[im];
        const KeyFrame &KF = m_KFs[ic];
        IMU::PreIntegrate(KF.m_us, m_KFs[ic - 1].m_T.m_t, KF.m_T.m_t, m_CsLM[im - 1], &D,
                          &m_work, true, D.m_u1.Valid() ? &D.m_u1 : NULL, D.m_u2.Valid() ? &D.m_u2 : NULL);
      }
      m_AdsLM.MakeZero();
      m_Afps.MakeZero();
      m_AfmsLM.MakeZero();
      const int Nd = static_cast<int>(m_ds.size());
      for (int id = 0; id < Nd; ++id) {
        m_uds[id] |= GBA_FLAG_TRACK_UPDATE_DEPTH | GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
        //m_Uds[id] |= GM_FLAG_TRACK_UPDATE_DEPTH;
      }
      m_SAcus.MakeZero();
      m_SMcus.MakeZero();
      m_SAcmsLM.MakeZero();
//#ifdef CFG_GROUND_TRUTH
#if 0
      m_Cs.Set(m_CsKFGT);
      m_CsLM.Set(m_CsLMGT);
      //const int ic = nKFs - 1;
      const int ic = nKFs - 2;
      Rigid3D &T = m_Cs[ic];
      const LA::AlignedVector3f dr(0.0f, 0.0f, 0.0f);
      //const LA::AlignedVector3f dr(0.1f, 0.2f, 0.3f);
      const LA::AlignedVector3f dp(0.0f, 0.0f, 0.0f);
      //const LA::AlignedVector3f dp(0.1f, 0.2f, 0.3f);
      //const LA::AlignedVector3f dv(0.0f, 0.0f, 0.0f);
      const LA::AlignedVector3f dv(0.1f, 0.2f, 0.3f);
      const LA::AlignedVector3f dba(0.0f, 0.0f, 0.0f);
      //const LA::AlignedVector3f dba(0.1f, 0.2f, 0.3f);
      const LA::AlignedVector3f dbw(0.0f, 0.0f, 0.0f);
      //const LA::AlignedVector3f dbw(0.1f, 0.2f, 0.3f);
      const Point3D p = T.GetPosition() + dp;
      const Rotation3D dR(dr * UT_FACTOR_DEG_TO_RAD);
      T = Rotation3D(T) * dR;
      T.SetPosition(p);
      const int im = m_CsLM.Size() - nKFs + ic;
      if (im >= 0) {
        Camera &C = m_CsLM[im];
        C.m_T = T;
        C.m_p = p;
        C.m_v += dv;
        C.m_ba += dba;
        C.m_bw += dbw * UT_FACTOR_DEG_TO_RAD;
      }
#endif
    }
#endif
#ifdef CFG_VERBOSE
    if (m_verbose) {
      if (m_verbose >= 2) {
        UT::PrintSeparator('*');
      } else if (m_iIter == 0) {
        UT::PrintSeparator();
      }
      PrintErrorStatistic(UT::String("*%2d: ", m_iIter), m_Cs, m_CsLM, m_ds, m_DsLM,
                          m_verbose >= 2);
    }
#endif
#ifdef GBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run(true, false);
    }
#endif
    m_ts[TM_FACTOR].Start();
    UpdateFactors();
    m_ts[TM_FACTOR].Stop();
#ifdef GBA_DEBUG_EIGEN
    DebugUpdateFactors();
#endif
    m_ts[TM_SCHUR_COMPLEMENT].Start();
    UpdateSchurComplement();
    m_ts[TM_SCHUR_COMPLEMENT].Stop();
#ifdef GBA_DEBUG_EIGEN
    DebugUpdateSchurComplement();
#endif
    m_ts[TM_CAMERA].Start();
    const bool scc = SolveSchurComplement();
    m_ts[TM_CAMERA].Stop();
#ifdef GBA_DEBUG_EIGEN
    DebugSolveSchurComplement();
#endif
    //if (!scc) {
    //  m_update = false;
    //  m_converge = false;
    //  break;
    //}
    m_ts[TM_DEPTH].Start();
    SolveBackSubstitution();
    m_ts[TM_DEPTH].Stop();
#ifdef GBA_DEBUG_EIGEN
    DebugSolveBackSubstitution();
#endif

#ifdef CFG_VERBOSE
    const int N = UT::PrintStringWidth();
#endif
    m_xsGD.Resize(0);   m_x2GD = 0.0f;
    m_xsDL.Resize(0);   m_x2DL = 0.0f;
    m_rho = FLT_MAX;
    const int nItersDL = std::max(BA_DL_MAX_ITERATIONS, 1);
    for (m_iIterDL = 0; m_iIterDL < nItersDL; ++m_iIterDL) {
      if (m_x2GN > m_delta2 && m_xsGD.Empty() && BA_DL_MAX_ITERATIONS > 0) {
        m_ts[TM_UPDATE].Start();
        SolveGradientDescent();
        m_ts[TM_UPDATE].Stop();
#ifdef GBA_DEBUG_EIGEN
        DebugSolveGradientDescent();
#endif
      }
      m_ts[TM_UPDATE].Start();
      SolveDogLeg();
      UpdateStatesPropose();
      m_ts[TM_UPDATE].Stop();
#ifdef CFG_VERBOSE
      if (m_verbose) {
        if (m_verbose >= 3) {
          UT::PrintSeparator();
        }
        const std::string str = m_verbose >= 2 ? " --> " :
                                (m_iIterDL == 0 ? " > " : std::string(N + 3, ' '));
        PrintErrorStatistic(str, m_CsBkp, m_CsLMBkp, m_dsBkp, m_DsLM, m_xsDL, m_verbose >= 2);
      }
#endif
      if (BA_DL_MAX_ITERATIONS == 0) {
#ifdef CFG_VERBOSE
        if (m_verbose) {
          UT::Print("\n");
        }
#endif
        break;
      }
      m_ts[TM_UPDATE].Start();
      ComputeReduction();
      m_ts[TM_UPDATE].Stop();
#ifdef CFG_VERBOSE
      if (m_verbose) {
        if (m_verbose >= 2) {
          UT::Print(" %f * (%e %e) <= %e %f\n", m_beta, sqrtf(m_x2GN), sqrtf(m_x2GD), sqrtf(m_delta2), m_rho);
        } else {
          UT::Print(" %.3f %.2e <= %.2e %.1f\n", m_beta, sqrtf(m_x2DL), sqrtf(m_delta2), m_rho);
        }
      }
#endif
#ifdef GBA_DEBUG_EIGEN
      DebugComputeReduction();
#endif
      m_ts[TM_UPDATE].Start();
      const bool accept = UpdateStatesDecide();
      m_ts[TM_UPDATE].Stop();
      if (accept) {
        break;
      }
    }
    if (m_iIterDL == BA_DL_MAX_ITERATIONS) {
      m_ts[TM_UPDATE].Start();
      UpdateStatesDecide();
      m_ts[TM_UPDATE].Stop();
    }
#ifdef CFG_DEBUG
    if (m_debug >= 2) {
      AssertConsistency();
    }
#endif
    if (!m_update || m_converge) {
      break;
    }
    if (GBA_EMBEDDED_POINT_ITERATION) {
      m_ts[TM_UPDATE].Start();
      EmbeddedPointIteration(m_Cs, m_ucs, m_uds, &m_ds);
      m_ts[TM_UPDATE].Stop();
    }
    MT_READ_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GBA_BufferDataEmpty);
    m_empty = BufferDataEmpty();
    MT_READ_LOCK_END(m_MT, iFrm, MT_TASK_GBA_BufferDataEmpty);
    if (!m_empty) {
      break;
    }
  }
#ifdef CFG_VERBOSE
  if (m_verbose) {
    UT::PrintSeparator('*');
    PrintErrorStatistic(UT::String("*%2d: ", m_iIter), m_Cs, m_CsLM, m_ds, m_DsLM, m_verbose >= 2);
  }
#endif
  m_ts[TM_TOTAL].Stop();
  for (int i = 0; i < TM_TYPES; ++i) {
    m_ts[i].Finish();
  }
  if (m_history >= 1) {
    History hist;
    hist.MakeZero();
    for (int i = 0; i < TM_TYPES; ++i) {
      hist.m_ts[i] = m_ts[i].GetAverageSeconds() * 1000.0;
    }
    //hist.m_Nd = static_cast<int>(m_ds.size());
    if (m_history >= 2) {
      hist.m_ESa = ComputeErrorStatistic(m_Cs, m_CsLM, m_ds, m_DsLM);
      hist.m_ESb = ComputeErrorStatistic(m_CsBkp, m_CsLMBkp, m_dsBkp, m_DsLM);
      hist.m_ESp = ComputeErrorStatistic(m_CsBkp, m_CsLMBkp, m_dsBkp, m_DsLM, m_xsDL, false);
#ifdef CFG_GROUND_TRUTH
      if (m_CsGT) {
        SolveSchurComplementGT(m_CsBkp, m_CsLMBkp, &m_xsGT);
        if (m_dsGT) {
          if (m_history >= 3) {
            EmbeddedPointIteration(m_CsKFGT, m_ucsGT, m_udsGT, m_dsGT);
          }
          SolveBackSubstitutionGT(m_dsBkp, &m_xsGT);
          hist.m_ESaGT = ComputeErrorStatistic(m_CsKFGT, m_CsLMGT, *m_dsGT, m_DsLMGT);
          hist.m_ESpGT = ComputeErrorStatistic(m_CsBkp, m_CsLMBkp, m_dsBkp, m_DsLM, m_xsGT, false);
        }
      }
#endif
      const int N = m_bs.Size();
      m_xsGN.Resize(N);
      hist.m_R = ComputeResidual(m_xsGN);
#ifdef CFG_GROUND_TRUTH
      if (m_CsGT) {
        m_xsGT.Resize(N);
        hist.m_RGT = ComputeResidual(m_xsGT);
      }
#endif
    }
    m_hists.push_back(hist);
  }
#ifdef GBA_DEBUG_VIEW
  if (viewer) {
    viewer->m_keyPause = true;
    viewer->Run();
    //viewer->Stop("D:/tmp/view.txt");
    delete viewer;
  }
#endif
  UpdateData();
#ifdef CFG_DEBUG
  //if (!m_serial && m_debug)
  if (m_debug) {
    AssertConsistency();
  }
#endif
#ifdef GBA_DEBUG_PRINT
  {
    Rigid3D C;
    Camera::Factor::Unitary::CC A;
    Camera::Factor::Binary::CC M;
    double Su, Ss2;
    UT::PrintSeparator('*');
    for (int i = 0; i < 2; ++i) {
      const int nKFs = static_cast<int>(m_KFs.size());
      if (i == 0) {
        const int iKF = nKFs - 1;
        const KeyFrame &KF = m_KFs[iKF];
        UT::Print("[%d]\n", KF.m_T.m_iFrm);
        C = m_Cs[iKF];
        A = m_SAcus[iKF] - m_SMcus[iKF];
        M.MakeZero();
        const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
        const xp128f sm = xp128f::get(1.0f / Nk);
        for (int ik = 0; ik < Nk; ++ik) {
          M += KF.m_Zm.m_SMczms[ik] * sm;
        }
        const int NZ = static_cast<int>(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
          M -= KF.m_SAcxzs[iZ] * sm;
        }

        Su = Ss2 = 0.0;
        for (int iZ = 0; iZ < NZ; ++iZ) {
          const FRM::Measurement &Z = KF.m_Zs[iZ];
          const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[Z.m_iKF];
          for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
            const Depth::InverseGaussian &d = ds[KF.m_zs[iz].m_ix];
            Su = d.u() + Su;
            Ss2 = d.s2() + Ss2;
          }
        }
      } else {
        C.MakeIdentity();
        A.MakeZero();
        M.MakeZero();
        Su = Ss2 = 0.0;
        int SNZ = 0;
        for (int iKF = 0; iKF < nKFs; ++iKF) {
          SNZ += static_cast<int>(m_KFs[iKF].m_Zs.size());
        }
        const xp128f sa = xp128f::get(1.0f / nKFs);
        const xp128f sm = xp128f::get(1.0f / (CountSchurComplements() - nKFs));
        for (int iKF = 0; iKF < nKFs; ++iKF) {
          C = m_Cs[iKF] * C;
          A += (m_SAcus[iKF] - m_SMcus[iKF]) * sa;
          const KeyFrame &KF = m_KFs[iKF];
          const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
          for (int ik = 0; ik < Nk; ++ik) {
            M += KF.m_Zm.m_SMczms[ik] * sm;
          }
          const int NZ = static_cast<int>(KF.m_Zs.size());
          for (int iZ = 0; iZ < NZ; ++iZ) {
            M -= KF.m_SAcxzs[iZ] * sm;
          }
        }
        const int Nd = m_iKF2d[nKFs - 1];
        for (int id = 0; id < Nd; ++id) {
          const Depth::InverseGaussian &d = m_ds[id];
          Su = d.u() + Su;
          Ss2 = d.s2() + Ss2;
        }
      }
      UT::Print("C:\n");
      C.Print(true);
      UT::Print("A:\n");
      A.Print(true);
      M.Print(true);
      UT::Print("d: %.10e %.10e\n", Su, Ss2);
    }
    UT::PrintSeparator('*');
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  const std::string fileName = UT::String("D:/tmp/xsGN/x_%03d_%03d.txt", m_KFs.size(), m_Zps.size());
  LA::AlignedVectorXf xsGN;
  xsGN.LoadB(fileName);
  const bool scc = m_xsGN.AssertEqual(xsGN, 1, "", -1.0f, -1.0f);
  UT_ASSERT(scc);
#endif
//#ifdef CFG_DEBUG
#if 0
  //const int iKF = 100;
  //const int ix = 2;
  //if (iKF < static_cast<int>(m_KFs.size()))
  //  UT::Print("%.10e\n", m_ds[m_KFs[iKF].m_id].u());
  const int im = 83;
  const int ipc = 5, ipm = 8;
  if (im < m_SAcmsLM.Size())
    UT::Print("%.10e\n", m_SAcmsLM[im].m_Au.m_Acm[ipc][ipm]);
#endif

  // Trigger GBA callback function if set
  if (m_callback) {
    const FRM::Tag &T = m_KFs.back().m_T;
    m_callback(T.m_iFrm, T.m_t);
  }
}

void GlobalBundleAdjustor::SetCallback(const IBA::Solver::IbaCallback& iba_callback) {
  m_callback = iba_callback;
}

//int GlobalBundleAdjustor::GetTotalPoints(int *N) {
//  int SN = 0;
//  const int _N = static_cast<int>(m_hists.size());
//  for (int i = 0; i < _N; ++i) {
//    SN += m_hists[i].m_Nd;
//  }
//  if (N) {
//    *N = _N;
//  }
//  return SN;
//}

float GlobalBundleAdjustor::GetTotalTime(int *N) {
  const int _N = static_cast<int>(m_hists.size());
  m_work.Resize(_N);
  LA::AlignedVectorXf ts(m_work.Data(), _N, false);
  for (int i = 0; i < _N; ++i) {
    ts[i] = float(m_hists[i].m_ts[TM_TOTAL]);
  }
  if (N) {
    *N = _N;
  }
  return ts.Sum();
}

bool GlobalBundleAdjustor::SaveTimes(const std::string fileName) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const double *ts = m_hists[i].m_ts;
    for (int j = 0; j < TM_TYPES; ++j) {
      fprintf(fp, "%f ", ts[j]);
    }
    fprintf(fp, "\n");
  }
  UT::PrintSaved(fileName);
  fclose(fp);
  return true;
}

bool GlobalBundleAdjustor::SaveCameras(const std::string fileName, const bool poseOnly) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int Nc1 = static_cast<int>(m_CsDel.size()), Nc2 = m_Cs.Size(), Nc = Nc1 + Nc2;
  AlignedVector<HistoryCamera> Cs;
  m_work.Resize(Cs.BindSize(Nc) / sizeof(float));
  Cs.Bind(m_work.Data(), Nc);
  Cs.Set(m_CsDel.data(), Nc1);
  for (int ic = 0; ic < Nc2; ++ic) {
    Cs.Push(HistoryCamera(m_KFs[ic].m_T, m_Cs[ic]));
  }
  std::sort(Cs.Data(), Cs.Data() + Nc);
  std::vector<int> &iFrm2m = m_idxsTmp1;
  if (!poseOnly) {
    iFrm2m.assign(Cs.Back().m_iFrm + 1, -1);
    for (int ic = Nc2 - m_CsLM.Size(), im = 0; ic < Nc2; ++ic, ++im) {
      iFrm2m[m_KFs[ic].m_T.m_iFrm] = im;
    }
  }

  Point3D p;
  Quaternion q;
  Rotation3D R;
  LA::AlignedVector3f ba, bw;
  const Rotation3D RuT = m_K.m_Ru.GetTranspose();
  for (int ic = 0; ic < Nc2; ++ic) {
    const HistoryCamera &C = Cs[ic];
    const Rigid3D &T = C.m_C;
    T.GetPosition(p);
    p += T.GetAppliedRotationInversely(m_K.m_pu);
    Rotation3D::AB(RuT, T, R);
    R.GetQuaternion(q);
    fprintf(fp, "%f %f %f %f %f %f %f %f", C.m_t, p.x(), p.y(), p.z(),
                                           q.x(), q.y(), q.z(), q.w());
    if (!poseOnly && iFrm2m[C.m_iFrm] != -1) {
      const Camera &_C = m_CsLM[iFrm2m[C.m_iFrm]];
      RuT.Apply(_C.m_ba, ba);
      RuT.Apply(_C.m_bw, bw);
      ba += m_K.m_ba;
      bw += m_K.m_bw;
      fprintf(fp, " %f %f %f %f %f %f %f %f %f", _C.m_v.x(), _C.m_v.y(), _C.m_v.z(),
                                                 ba.x(), ba.y(), ba.z(),
                                                 bw.x(), bw.y(), bw.z());
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

bool GlobalBundleAdjustor::SaveCosts(const std::string fileName, const int type) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const History &hist = m_hists[i];
    switch (type) {
    case 0: hist.m_ESa.Save(fp);     break;
    case 1: hist.m_ESb.Save(fp);     break;
    case 2: hist.m_ESp.Save(fp);     break;
#ifdef CFG_GROUND_TRUTH
    case 3: hist.m_ESaGT.Save(fp);  break;
    case 4: hist.m_ESpGT.Save(fp);  break;
#endif
    }
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

bool GlobalBundleAdjustor::SaveResiduals(const std::string fileName, const int type) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const History &hist = m_hists[i];
    switch (type) {
    case 0: hist.m_R.Save(fp);    break;
#ifdef CFG_GROUND_TRUTH
    case 1: hist.m_RGT.Save(fp);  break;
#endif
    }
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

void GlobalBundleAdjustor::ComputeErrorFeature(float *ex) {
  Rigid3D Tr[2];
  FTR::Error e;
  float exi;
#if 0
  float Se2;
#ifdef CFG_STEREO
  float Ser2;
#endif
  int SN;
#endif
  *ex = 0.0f;
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
#if 0
    Se2 = 0.0f;
#ifdef CFG_STEREO
    Ser2 = 0.0f;
#endif
    SN = 0;
#else
    const KeyFrame &KF = m_KFs[iKF];
    const int Nz = static_cast<int>(KF.m_zs.size());
#ifdef CFG_STEREO
    m_work.Resize(Nz * 2);
    LA::AlignedVectorXf e2s(m_work.Data(), Nz, false);
    LA::AlignedVectorXf e2rs(e2s.BindNext(), Nz, false);
    e2s.Resize(0);
    e2rs.Resize(0);
#else
    m_work.Resize(Nz);
    LA::AlignedVectorXf e2s(m_work.Data(), Nz, false);
    e2s.Resize(0);
#endif
#endif
    const Rigid3D C = m_Cs[iKF];
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      *Tr = C / m_Cs[Z.m_iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const Depth::InverseGaussian *_ds = m_ds.data() + m_iKF2d[Z.m_iKF];
      const KeyFrame &_KF = m_KFs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        FTR::GetError(Tr, _KF.m_xs[ix], _ds[ix], z, e);
#ifdef CFG_STEREO
        if (z.m_z.Valid()) {
          const float e2 = e.m_ex.SquaredLength();
#if 0
          Se2 = e2 + Se2;
          ++SN;
#else
          e2s.Push(e2);
#endif
        }
        if (z.m_zr.Valid()) {
          const float e2r = e.m_exr.SquaredLength();
#if 0
          Ser2 = e2r + Ser2;
          ++SN;
#else
          e2rs.Push(e2r);
#endif
        }
#else
        const float e2 = e.m_ex.SquaredLength();
#if 0
        Se2 = e2 * m_K.m_K.fxy() + Se2;
        ++SN;
#else
        e2s.Push(e2);
#endif
#endif
      }
    }
#ifdef CFG_STEREO
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (KF.m_xs[ix].m_xr.Invalid()) {
        continue;
      }
      FTR::GetError(m_K.m_br, ds[ix], KF.m_xs[ix], e.m_exr);
      const float e2r = e.m_exr.SquaredLength();
#if 0
      Ser2 = e2r + Ser2;
      ++SN;
#else
      e2rs.Push(e2r);
#endif
    }
#endif
#if 0
#ifdef CFG_STEREO
    exi = sqrtf((Se2 * m_K.m_K.fxy() + Ser2 * m_K.m_Kr.fxy()) / SN);
#else
    exi = sqrtf(Se2 * m_K.m_K.fxy() / SN);
#endif
#else
    if (!e2s.Empty()) {
      const int ith = e2s.Size() >> 1;
      std::nth_element(e2s.Data(), e2s.Data() + ith, e2s.End());
      exi = sqrtf(e2s[ith] * m_K.m_K.fxy());
    } else
      exi = 0.0f;
#ifdef CFG_STEREO
    if (!e2rs.Empty()) {
      const int ith = e2rs.Size() >> 1;
      std::nth_element(e2rs.Data(), e2rs.Data() + ith, e2rs.End());
      exi = std::max(sqrtf(e2rs[ith] * m_K.m_K.fxy()), exi);
    }
#endif
#endif
    *ex = std::max(exi, *ex);
  }
}

void GlobalBundleAdjustor::ComputeErrorIMU(float *er, float *ep, float *ev,
                                           float *eba, float *ebw) {
  *er = *ep = *ev = *eba = *ebw = 0.0f;
  const int Nc = m_Cs.Size();
  for (int ic1 = Nc - m_CsLM.Size(), ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc; 
       ic1 = ic2++, im1 = im2++) {
    if (m_KFs[ic2].m_us.Empty()) {
      continue;
    }
    const IMU::Delta::Error e = m_DsLM[im2].GetError(m_CsLM[im1], m_CsLM[im2], m_K.m_pu);
    *er = std::max(e.m_er.SquaredLength(), *er);
    *ep = std::max(e.m_ep.SquaredLength(), *ep);
    *ev = std::max(e.m_ev.SquaredLength(), *ev);
    *eba = std::max(e.m_eba.SquaredLength(), *eba);
    *ebw = std::max(e.m_ebw.SquaredLength(), *ebw);
  }
  *er *= UT_FACTOR_RAD_TO_DEG;
  *ebw *= UT_FACTOR_RAD_TO_DEG;
}

void GlobalBundleAdjustor::ComputeErrorDrift(float *er, float *ep) {
  *er = 0.0f;
  *ep = 0.0f;
#ifdef CFG_GROUND_TRUTH
  if (m_CsKFGT.Empty())
    return;
  Rigid3D Tr, TrGT, Te;
  LA::AlignedVector3f _er, _ep;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iKFNearest = m_KFs[iKF].m_iKFNearest;
    if (iKFNearest == -1)
      continue;
    Tr = m_Cs[iKF] / m_Cs[iKFNearest];
    TrGT = m_CsKFGT[iKF] / m_CsKFGT[iKFNearest];
    Te = Tr / TrGT;
    Te.GetRodrigues(_er);
    Te.GetPosition(_ep);
    *er = std::max(_er.SquaredLength(), *er);
    *ep = std::max(_ep.SquaredLength(), *ep);
  }
  *er *= UT_FACTOR_RAD_TO_DEG;
#endif
}

float GlobalBundleAdjustor::ComputeRMSE() {
  float Se2 = 0.0f;
  const int Nc1 = static_cast<int>(m_CsDel.size()), Nc2 = m_Cs.Size(), Nc = Nc1 + Nc2;
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    for (int ic = 0; ic < Nc1; ++ic) {
      const HistoryCamera &C = m_CsDel[ic];
      const Point3D p1 = C.m_C.GetPosition(), p2 = m_CsGT[C.m_iFrm].m_p;
      Se2 += (p1 - p2).SquaredLength();
    }
  }
  if (!m_CsKFGT.Empty()) {
    for (int ic = 0; ic < Nc2; ++ic) {
      const Point3D p1 = m_Cs[ic].GetPosition(), p2 = m_CsKFGT[ic].GetPosition();
      Se2 += (p1 - p2).SquaredLength();
    }
  }
#endif
  return sqrtf(Se2 / Nc2);
}

void GlobalBundleAdjustor::SynchronizeData() {
  const int iFrm = m_KFs.empty() ? MT_TASK_NONE : m_KFs.back().m_T.m_iFrm;
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GBA_SynchronizeData);
  m_IKFs1.swap(m_IKFs2);
  m_ICs1.swap(m_ICs2);
  m_IZps1.swap(m_IZps2);
  if (m_IZpLM1.Valid()) {
    m_IZpLM2 = m_IZpLM1;
    m_IZpLM1.Invalidate();
  }
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GBA_SynchronizeData);

  const int nKFs1 = static_cast<int>(m_KFs.size());
  const int nKFs2 = nKFs1 + static_cast<int>(m_IKFs2.size());
  const int Nm1 = m_CsLM.Size();
  const int Nm2 = Nm1 + static_cast<int>(m_IKFs2.size());
  m_KFs.resize(nKFs2);
  m_Cs.Resize(nKFs2, true);
  m_CsLM.Resize(Nm2, true);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsKFGT.Resize(nKFs2, true);
    m_CsLMGT.Resize(Nm2, true);
  }
#endif
#ifdef CFG_INCREMENTAL_PCG
  m_xcs.Resize(nKFs2, true);
  m_xmsLM.Resize(Nm2, true);
#endif
  m_ucs.resize(nKFs2, GBA_FLAG_FRAME_UPDATE_CAMERA);
  //m_Ucs.assign(nKFs2, GM_FLAG_FRAME_UPDATE_CAMERA);
  m_Ucs.assign(nKFs2, GM_FLAG_FRAME_DEFAULT);
  const ubyte ucmFlag1 = GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                         GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  const ubyte ucmFlag2 = GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                         GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                         GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
  m_ucmsLM.resize(Nm2, ucmFlag1 | ucmFlag2);
  m_DsLM.Resize(Nm2, true);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_DsLMGT.Resize(Nm2, true);
  }
  if (m_history >= 3) {
    m_ucsGT.resize(m_KFs.size(), LBA_FLAG_FRAME_DEFAULT);
  }
#endif
  std::list<InputKeyFrame>::iterator IKF = m_IKFs2.begin();
  const ubyte udFlag1 = GBA_FLAG_TRACK_UPDATE_DEPTH | GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
  const ubyte udFlag2 = GBA_FLAG_TRACK_UPDATE_DEPTH
#ifdef GBA_FLAG_TRACK_MEASURE
                      | GBA_FLAG_TRACK_MEASURE
#endif
                      ;
  std::vector<int> &iKF2X = m_idxsTmp1, &iX2z = m_idxsTmp2, &izs = m_idxsTmp3;
  for (int iKF = nKFs1, im = Nm1; iKF < nKFs2; ++iKF, ++IKF, ++im) {
    KeyFrame &KF = m_KFs[iKF];
    KF.Initialize(*IKF);
//#ifdef CFG_DEBUG
#if 0
    IKF->m_C.Print(true);
#endif
    PushFeatureMeasurementMatchesFirst(KF, iKF2X, iX2z);
    const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      PushFeatureMeasurementMatchesNext(m_KFs[KF.m_iKFsMatch[ik]], KF, iKF2X, iX2z, KF.m_Zm);
    }
    if (iKF > 0) {
      const int _iKF = iKF - 1;
      if (KF.m_us.Empty()) {
        m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_INVALID;
      } else {
        if ((KF.m_T.m_t - m_KFs[_iKF].m_T.m_t) > GBA_MAX_IMU_TIME_INTERVAL) {
          KF.m_us.Clear();
          m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_INVALID;
        } else if (KF.m_iKFsMatch.empty() || KF.m_iKFsMatch.back() != _iKF) {
          KF.InsertMatchKeyFrame(_iKF);
        }
        if (!KF.m_us.Empty()) {
          const int _im = im - 1;
          if (_im >= 0 && (m_ucmsLM[_im] & GBA_FLAG_CAMERA_MOTION_INVALID)) {
            m_ucmsLM[_im] &= ~GBA_FLAG_CAMERA_MOTION_INVALID;
            m_ucmsLM[_im] |= ucmFlag2;
          }
        }
      }
    }
    const bool v1 = IKF->m_C.m_T.Valid(), v2 = IKF->m_C.m_v.Valid();
    if (im > 0 && !KF.m_us.Empty() &&
       (GBA_PROPAGATE_CAMERA || !v1 || !v2)) {
      IMU::Delta D;
      Camera C;
      const KeyFrame &_KF = m_KFs[iKF - 1];
      const Camera &_C = m_CsLM[im - 1];
      IMU::PreIntegrate(KF.m_us, _KF.m_T.m_t, KF.m_T.m_t, _C, &D, &m_work,
                        false, _KF.m_us.Empty() ? NULL : &_KF.m_us.Back());
      IMU::Propagate(m_K.m_pu, D, _C, C);
      if (GBA_PROPAGATE_CAMERA) {
        m_CsLM[im] = C;
        m_Cs[iKF] = m_CsLM[im].m_T;
      } else {
        if (!v1) {
          m_Cs[iKF] = C.m_T;
          m_CsLM[im].m_T = C.m_T;
          m_CsLM[im].m_p = C.m_p;
        }
        if (!v2) {
          m_CsLM[im].m_v = C.m_v;
          m_CsLM[im].m_ba = C.m_ba;
          m_CsLM[im].m_bw = C.m_bw;
        }
      }
    } else {
      m_Cs[iKF] = IKF->m_C.m_T;
      m_CsLM[im] = IKF->m_C;
    }
#ifdef CFG_GROUND_TRUTH
    if (m_CsGT) {
      m_CsKFGT[iKF] = m_CsGT[KF.m_T.m_iFrm].m_T;
      m_CsLMGT[im] = m_CsGT[KF.m_T.m_iFrm];
    }
#endif
#ifdef CFG_INCREMENTAL_PCG
    m_xcs[iKF].MakeZero();
    m_xmsLM[im].MakeZero();
#endif
    IMU::Delta &D = m_DsLM[im];
    if (KF.m_us.Empty() || im == 0) {
      D.Invalidate();
    } else {
      const int _iKF = iKF - 1;
      const int _im = im - 1;
      const KeyFrame &_KF = m_KFs[_iKF];
      const float _t = _KF.m_T.m_t;
      IMU::PreIntegrate(KF.m_us, _t, KF.m_T.m_t, m_CsLM[_im], &D, &m_work,
                        true, _KF.m_us.Empty() ? NULL : &_KF.m_us.Back());
      if (_im > 0 && !_KF.m_us.Empty()) {
        IMU::Delta &_D = m_DsLM[_im];
        IMU::PreIntegrate(_KF.m_us, m_KFs[_iKF - 1].m_T.m_t, _t, m_CsLM[_im - 1], &_D, &m_work,
                          true, _D.m_u1.Valid() ? &_D.m_u1 : NULL, &KF.m_us.Front());
        m_ucs[_iKF] |= GBA_FLAG_FRAME_UPDATE_CAMERA;
        m_ucmsLM[_im] |= ucmFlag1 | ucmFlag2;
      }
    }
    if (KF.m_T.m_iFrm == 0) {
      const LA::Vector3f s2r = LA::Vector3f::Get(BA_VARIANCE_FIX_ORIGIN_ROTATION_X,
                                                 BA_VARIANCE_FIX_ORIGIN_ROTATION_Y,
                                                 BA_VARIANCE_FIX_ORIGIN_ROTATION_Z);
      const float s2p = BA_VARIANCE_FIX_ORIGIN_POSITION;
      m_Zo.Set(BA_WEIGHT_FIX_ORIGIN, s2r, s2p, m_Cs[iKF]);
      m_Ao.MakeZero();
    }
#ifdef CFG_GROUND_TRUTH
    if (m_CsGT) {
      if (KF.m_us.Empty() || im == 0) {
        m_DsLMGT[im] = D;
      } else {
        const int _iKF = iKF - 1, _im = im - 1;
        const KeyFrame &_KF = m_KFs[_iKF];
        const float _t = _KF.m_T.m_t;
        const Camera &_C = m_CsLMGT[_im];
        IMU::PreIntegrate(KF.m_us, _t, KF.m_T.m_t, _C, &m_DsLMGT[im], &m_work,
                          true, _KF.m_us.Empty() ? NULL : &_KF.m_us.Back());
#ifdef GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
        D.DebugSetMeasurement(_C, m_CsLMGT[im], m_K.m_pu);
        m_DsLMGT[im].DebugSetMeasurement(_C, m_CsLMGT[im], m_K.m_pu);
#endif
        if (_im > 0 && !_KF.m_us.Empty()) {
          IMU::Delta &_D = m_DsLMGT[_im];
          IMU::PreIntegrate(_KF.m_us, m_KFs[_iKF - 1].m_T.m_t, _t, m_CsLMGT[_im - 1], &_D, &m_work,
                            true, _D.m_u1.Valid() ? &_D.m_u1 : NULL, &KF.m_us.Front());
#ifdef GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
          m_DsLM[_im].DebugSetMeasurement(m_CsLMGT[_im - 1], _C, m_K.m_pu);
          _D.DebugSetMeasurement(m_CsLMGT[_im - 1], _C, m_K.m_pu);
#endif
        }
      }
    }
#endif
#ifdef CFG_DEBUG
    UT_ASSERT(KF.m_iKFsPrior.empty());
#endif
    m_iKF2d.push_back(m_iKF2d.back());
    m_iKF2cb.push_back(m_iKF2cb.back() + static_cast<int>(KF.m_ikp2KF.size()));
    const int NZ = static_cast<int>(IKF->m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = IKF->m_Zs[iZ];
      const int id = m_iKF2d[Z.m_iKF];
      Depth::InverseGaussian *ds = m_ds.data() + id;
      ubyte *uds = m_uds.data() + id/*, *Uds = m_Uds.data() + id*/;
#ifdef CFG_GROUND_TRUTH
      ubyte *udsGT = m_udsGT.data() + id;
#endif
      const KeyFrame &_KF = m_KFs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = IKF->m_zs[iz].m_ix;
//#ifdef CFG_DEBUG 
#if 0
        if (iz == 0) {
          UT::PrintSeparator();
        }
        UT::Print("iz = %d, ix = %d, d = %f\n", iz, ix, IKF->m_dzs[iz].u());
#endif
        if (IKF->m_dzs[iz].Valid() && fabs(IKF->m_dzs[iz].u() - ds[ix].u()) >= BA_UPDATE_DEPTH) {
          ds[ix] = IKF->m_dzs[iz];
          m_ucs[Z.m_iKF] |= GBA_FLAG_FRAME_UPDATE_DEPTH;
          uds[ix] |= udFlag2;
          //m_Ucs[Z.m_iKF] |= GM_FLAG_FRAME_UPDATE_DEPTH;
          //Uds[ix] |= GM_FLAG_TRACK_UPDATE_DEPTH;
        }
#ifdef GBA_FLAG_TRACK_MEASURE
        else {
          uds[ix] |= GBA_FLAG_TRACK_MEASURE;
        }
#endif
#ifdef CFG_GROUND_TRUTH
        if (m_history >= 3) {
          m_ucsGT[Z.m_iKF] |= GBA_FLAG_FRAME_UPDATE_DEPTH;
          udsGT[ix] |= GBA_FLAG_TRACK_UPDATE_DEPTH;
        }
#endif
      }
    }
    const int NX = static_cast<int>(IKF->m_Xs.size());
    for (int iX1 = 0, iX2 = 0; iX1 < NX; iX1 = iX2) {
      const int _iKF = IKF->m_Xs[iX1].m_iKF;
      for (iX2 = iX1 + 1; iX2 < NX && IKF->m_Xs[iX2].m_iKF == _iKF; ++iX2) {}
      const int id = m_iKF2d[_iKF + 1], Nx = iX2 - iX1;
#ifdef CFG_DEBUG
      UT_ASSERT(Nx != 0);
#endif
      for (int jKF = _iKF; jKF <= iKF; ++jKF) {
        m_iKF2d[jKF + 1] += Nx;
      }
      m_ds.insert(m_ds.begin() + id, Nx, Depth::InverseGaussian());
      m_uds.insert(m_uds.begin() + id, Nx, udFlag1);
      m_ucs[_iKF] |= GBA_FLAG_FRAME_UPDATE_DEPTH;
#ifdef CFG_GROUND_TRUTH
      if (m_history >= 3) {
        m_ucsGT[_iKF] |= GBA_FLAG_FRAME_UPDATE_DEPTH;
        m_udsGT.insert(m_udsGT.begin() + id, Nx, GBA_FLAG_TRACK_UPDATE_DEPTH);
      }
#endif
      const GlobalMap::Point *Xs = IKF->m_Xs.data() + iX1;
      Depth::InverseGaussian *ds = m_ds.data() + id;
      ubyte *uds = m_uds.data() + id;
      m_xsTmp.resize(Nx);
      KeyFrame &_KF = m_KFs[_iKF];
      for (int i = 0, ix = static_cast<int>(_KF.m_xs.size()); i < Nx; ++i, ++ix) {
        const GlobalMap::Point &X = Xs[i];
        ds[i] = X.m_d;
        m_xsTmp[i] = X.m_x;
        const int Nz = static_cast<int>(X.m_zs.size());
        if (Nz == 0) {
          continue;
        }
#ifdef GBA_FLAG_TRACK_MEASURE
        uds[i] |= GBA_FLAG_TRACK_MEASURE;
#endif
        izs.resize(Nz);
        for (int i2 = 0; i2 < Nz; ++i2) {
          const FTR::Measurement &z2 = X.m_zs[i2];
          const int iKF2 = z2.m_iKF;
          KeyFrame &KF2 = m_KFs[iKF2];
          int ik2, &iz2 = izs[i2];
          const int Nk2 = static_cast<int>(KF2.m_iKFsMatch.size());
          KF2.PushFeatureMeasurement(_iKF, ix, z2, &ik2, &iz2);
          for (int iKF3 = iKF2 + 1; iKF3 < nKFs2; ++iKF3) {
            KeyFrame &KF3 = m_KFs[iKF3];
            if (KF3.m_Zs.empty() || KF3.m_Zs.back().m_iKF <= _iKF) {
              continue;
            }
            const int ik3 = KF3.SearchMatchKeyFrame(iKF2);
            if (ik3 >= 0) {
              KF3.m_Zm.InsertFeatureMeasurement1(ik3, iz2);
            }
          }
          for (int i1 = 0; i1 < i2; ++i1) {
            const int iKF1 = X.m_zs[i1].m_iKF;
            const std::vector<int>::iterator _ik2 = std::lower_bound(KF2.m_iKFsMatch.begin() + ik2,
                                                                     KF2.m_iKFsMatch.end(), iKF1);
            ik2 = static_cast<int>(_ik2 - KF2.m_iKFsMatch.begin());
            if (_ik2 == KF2.m_iKFsMatch.end() || *_ik2 != iKF1) {
              KF2.InsertMatchKeyFrame(iKF1, &_ik2);
            }
            KF2.m_Zm.InsertFeatureMeasurementMatch(ik2, izs[i1], iz2);
//#ifdef CFG_DEBUG
#if 0
            const KeyFrame &KF1 = m_KFs[iKF1];
            KF2.m_Zm.AssertConsistency(static_cast<int>(KF2.m_iKFsMatch.size()));
            KF2.m_Zm.AssertConsistency(ik2, KF1, KF2, m_izmsTmp);
#endif
          }
//#ifdef CFG_DEBUG
#if 0
          const int _Nk2 = static_cast<int>(KF2.m_iKFsMatch.size());
          for (int ik = 0; ik < _Nk2; ++ik) {
            const KeyFrame &KF1 = m_KFs[KF2.m_iKFsMatch[ik]];
            KF2.m_Zm.AssertConsistency(ik, KF1, KF2, m_izmsTmp);
          }
#endif
          if (static_cast<int>(KF2.m_iKFsMatch.size()) == Nk2) {
            continue;
          }
          const int Ncb = static_cast<int>(KF2.m_iKFsMatch.size()) - Nk2;
          for (int jKF = z2.m_iKF; jKF <= iKF; ++jKF) {
            m_iKF2cb[jKF + 1] += Ncb;
          }
        }
      }
      _KF.PushFeatures(m_xsTmp);
    }
  }
  m_AdsLM.Resize(Nm2, true);    m_AdsLM.MakeZero(Nm1, Nm2);
  m_Afps.Resize(nKFs2, true);   m_Afps.MakeZero(nKFs1, nKFs2);
  m_AfmsLM.Resize(Nm2, true);   m_AfmsLM.MakeZero(Nm1, Nm2);
  m_SAcus.Resize(nKFs2, true);  m_SAcus.MakeZero(nKFs1, nKFs2);
  m_SMcus.Resize(nKFs2, true);  m_SMcus.MakeZero(nKFs1, nKFs2);
  m_SAcmsLM.Resize(Nm2, true);  m_SAcmsLM.MakeZero(Nm1, Nm2);
  m_IKFs2.resize(0);
  const int N = static_cast<int>(m_ICs2.size());
  for (int i = 0; i < N; ++i) {
    const InputCamera &C = m_ICs2[i];
    m_Cs[C.m_iKF] = C.m_C;
    m_ucs[C.m_iKF] |= GBA_FLAG_FRAME_UPDATE_CAMERA;
    m_Ucs[C.m_iKF] |= GM_FLAG_FRAME_UPDATE_CAMERA;
    const int im = C.m_iKF - (nKFs2 - Nm2);
    if (im < 0) {
      continue;
    }
    Camera &_C = m_CsLM[im];
    _C.m_T = C.m_C;
    _C.m_T.GetPosition(_C.m_p);
    m_ucmsLM[im] |= GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                    GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  }
  m_ICs2.resize(0);
  for (std::list<CameraPrior::Pose>::const_iterator IZp = m_IZps2.begin();
       IZp != m_IZps2.end(); ++IZp) {
    const int iZp = static_cast<int>(m_Zps.size());
    m_Zps.push_back(*IZp);
    m_Aps.resize(iZp + 1);
    //m_Aps.push_back(CameraPrior::Pose::Factor());
    m_Aps[iZp].MakeZero();
    m_ucs[IZp->m_iKFr] |= GBA_FLAG_FRAME_UPDATE_CAMERA;
    const int imr = IZp->m_iKFr - (nKFs2 - Nm2);
    if (imr >= 0) {
      m_ucmsLM[imr] |= ucmFlag1;
    }
    const int N = static_cast<int>(IZp->m_iKFs.size());
    for (int i = 0; i < N; ++i) {
      const int _iKF = IZp->m_iKFs[i];
      m_ucs[_iKF] |= GBA_FLAG_FRAME_UPDATE_CAMERA;
      const int im = _iKF - (nKFs2 - Nm2);
      if (im >= 0) {
        m_ucmsLM[im] |= ucmFlag1;
      }
    }
    for (int i1 = -1; i1 < N; ++i1) {
      const int iKF1 = i1 == -1 ? IZp->m_iKFr : IZp->m_iKFs[i1];
      for (int i2 = i1 + 1; i2 < N; ++i2) {
        const int iKF2 = IZp->m_iKFs[i2];
        const int _iKF1 = std::min(iKF1, iKF2), _iKF2 = std::max(iKF1, iKF2);
        KeyFrame &KF = m_KFs[_iKF2];
        const int Nkp = static_cast<int>(KF.m_ikp2KF.size());
        const int ip = KF.PushCameraPrior(_iKF1, &m_work);
        if (ip == -1 || static_cast<int>(KF.m_ikp2KF.size()) == Nkp) {
          continue;
        }
        for (int jKF = _iKF2; jKF < nKFs2; ++jKF) {
          ++m_iKF2cb[jKF + 1];
        }
      }
    }
  }
  m_IZps2.resize(0);
  if (m_IZpLM2.Valid()) {
    if (m_ZpLM.Valid()) {
      m_ApLM.MakeMinus();
      m_SAcus[m_ZpLM.m_iKF].Increase3(m_ApLM.m_Arr.m_A, m_ApLM.m_Arr.m_b);
      const int im = m_ZpLM.m_iKF - (nKFs2 - Nm2);
      Camera::Factor::Unitary &SAcm = m_SAcmsLM[im].m_Au;
      SAcm.m_Acm.Increase3(m_ApLM.m_Arm);
      SAcm.m_Amm += m_ApLM.m_Amm;
    }
    m_ZpLM = m_IZpLM2;
    m_ucmsLM[m_ZpLM.m_iKF - (nKFs2 - Nm2)] |= ucmFlag2;

    m_ApLM.MakeZero();
    m_IZpLM2.Invalidate();
  }
}

void GlobalBundleAdjustor::UpdateData() {
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  m_GM->GBA_Update(iFrm, m_Cs/*, m_ds*/, m_Ucs);
#ifdef CFG_VERBOSE
  if (m_verbose >= 2) {
    const int Nc = int(m_KFs.size()), Nd = int(m_ds.size());
    const int Ncu = UT::VectorCountFlag<ubyte>(m_Ucs, GM_FLAG_FRAME_UPDATE_CAMERA);
    //const int Ndu = UT::VectorCountFlag<ubyte>(m_Uds, GM_FLAG_TRACK_UPDATE_DEPTH);
    UT::PrintSeparator();
    UT::Print("[%d] [GlobalBundleAdjustor::UpdateData]\n", iFrm);
    UT::Print("  Camera = %d / %d = %.2f%%\n", Ncu, Nc, UT::Percentage(Ncu, Nc));
    //UT::Print("  Depth  = %d / %d = %.2f%%\n", Ndu, Nd, UT::Percentage(Ndu, Nd));
  }
#endif
}

bool GlobalBundleAdjustor::BufferDataEmpty() {
  return m_IKFs1.empty() && m_ICs1.empty()/* && m_IZps1.empty() && m_IZpLM1.Invalid()*/;
}

void GlobalBundleAdjustor::PushFeatureMeasurementMatchesFirst(const FRM::Frame &F,
                                                              std::vector<int> &iKF2X,
                                                              std::vector<int> &iX2z) {
  int SNx = 0;
  const int NZ = int(F.m_Zs.size());
  iKF2X.assign(m_KFs.size(), -1);
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const int iKF = F.m_Zs[iZ].m_iKF;
    iKF2X[iKF] = SNx;
    SNx += int(m_KFs[iKF].m_xs.size());
  }
  iX2z.assign(SNx, -1);
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    int *ix2z = iX2z.data() + iKF2X[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz)
      ix2z[F.m_zs[iz].m_ix] = iz;
  }
}

void GlobalBundleAdjustor::PushFeatureMeasurementMatchesNext(const FRM::Frame &F1,
                                                             const FRM::Frame &F2, const std::vector<int> &iKF2X, const std::vector<int> &iX2z2,
                                                             FRM::MeasurementMatch &Zm) {
  m_izmsTmp.resize(0);
  const int NZ1 = int(F1.m_Zs.size());
  for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
    const FRM::Measurement &Z1 = F1.m_Zs[iZ1];
    const int iX = iKF2X[Z1.m_iKF];
    if (iX == -1) {
      continue;
    }
    const int *ix2z2 = iX2z2.data() + iX;
    const int iz11 = Z1.m_iz1, iz12 = Z1.m_iz2;
    for (int iz1 = iz11; iz1 < iz12; ++iz1) {
      const int iz2 = ix2z2[F1.m_zs[iz1].m_ix];
      if (iz2 != -1) {
        m_izmsTmp.push_back(FTR::Measurement::Match(iz1, iz2));
      }
    }
  }
  Zm.PushFeatureMeasurementMatches(m_izmsTmp);
}

int GlobalBundleAdjustor::CountMeasurementsFrame() {
  int SN = 0;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SN += static_cast<int>(m_KFs[iKF].m_Zs.size());
  }
  return SN;
}

int GlobalBundleAdjustor::CountMeasurementsFeature() {
  int SN = 0;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SN += static_cast<int>(m_KFs[iKF].m_zs.size());
  }
  return SN;
}

int GlobalBundleAdjustor::CountMeasurementsPriorCameraPose() {
  int SN = 0;
#if 0
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF)
    SN += m_KFs[iKF].m_Zps.Size();
#endif
  return SN;
}

int GlobalBundleAdjustor::CountSchurComplements() {
  return m_SAcus.Size() + CountSchurComplementsOffDiagonal();
}

int GlobalBundleAdjustor::CountSchurComplementsOffDiagonal() {
  return m_iKF2cb.back();
}
