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
#include "GlobalBundleAdjustor.h"
#include "IBA_internal.h"
#include "Vector12.h"

#ifdef CFG_DEBUG
#ifdef CFG_DEBUG_EIGEN
#define LBA_DEBUG_EIGEN
#endif
//#define LBA_DEBUG_CHECK
//#define LBA_DEBUG_PRINT
//#define LBA_DEBUG_PRINT_STEP
#ifdef CFG_GROUND_TRUTH
//#define LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
#endif
#endif
#if WIN32
#define LBA_DEBUG_VIEW
#endif
#ifdef LBA_DEBUG_VIEW
#include "ViewerIBA.h"
#endif

#ifdef LBA_DEBUG_CHECK
static const int g_ic = 0;
//static const int g_ic = INT_MAX;
static const int g_iKF = 0;
//static const int g_iKF = INT_MAX;
static const int g_ix = 9;
static const int g_ist = 41;
static const int g_izKF = 0;
static const int g_izLF = 22;
static const Rigid3D *g_CKF = NULL;
static const LocalBundleAdjustor::KeyFrame *g_KF = NULL;
static const Depth::InverseGaussian *g_d = NULL;
static const FTR::Factor::FixSource::Source::A *g_Ax = NULL;
static const FTR::Factor::FixSource::Source::A *g_AxST = NULL;
static const FTR::Factor::FixSource::Source::M *g_MxST = NULL;
static const FTR::Factor::FixSource::Source::M *g_Mx = NULL;
static const FTR::Factor::Depth *g_AzKF = NULL;
static const Camera *g_CLF = NULL;
static const LocalBundleAdjustor::LocalFrame *g_LF = NULL;
static const Camera::Factor::Unitary::CC *g_SAcuLF = NULL;
static const Camera::Factor::Unitary::CC *g_SMcuLF = NULL;
static const Camera::Factor *g_SAcmLF = NULL;
static const FTR::Factor::FixSource::A1 *g_Az1LF = NULL;
static const FTR::Factor::FixSource::A2 *g_Az2LF = NULL;
static const FTR::Factor::FixSource::M1 *g_Mz1LF = NULL;
static const FTR::Factor::FixSource::M2 *g_Mz2LF = NULL;
static const FTR::Factor::DD *g_SmddST = NULL;
#endif

void LocalBundleAdjustor::Initialize(IBA::Solver *solver, const int serial, const int verbose,
                                     const int debug, const int history) {
  MT::Thread::Initialize(serial, 4, "LBA");
  m_solver = solver;
  m_LM = &solver->m_internal->m_LM;
  m_GM = &solver->m_internal->m_GM;
  m_GBA = &solver->m_internal->m_GBA;
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

void LocalBundleAdjustor::Reset() {
  MT::Thread::Reset();
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_LBA_Reset);
  m_ILFs1.resize(0);
  m_ILFs2.resize(0);
  m_IKFs1.resize(0);
  m_IKFs2.resize(0);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_LBA_Reset);

  m_GM->LBA_Reset();

  for (int i = 0; i < TM_TYPES; ++i) {
    m_ts[i].Reset(TIME_AVERAGING_COUNT);
  }
  m_hists.resize(0);

  m_delta2 = BA_DL_RADIUS_INITIAL;

  m_ic2LF.reserve(LBA_MAX_LOCAL_FRAMES);    m_ic2LF.resize(0);
  m_LFs.reserve(LBA_MAX_LOCAL_FRAMES);      m_LFs.resize(0);
  m_CsLF.Reserve(LBA_MAX_LOCAL_FRAMES);     m_CsLF.Resize(0);
#ifdef CFG_GROUND_TRUTH
  m_CsLFGT.Reserve(LBA_MAX_LOCAL_FRAMES);   m_CsLFGT.Resize(0);
#endif
  m_ucsLF.reserve(LBA_MAX_LOCAL_FRAMES);    m_ucsLF.resize(0);
  m_ucmsLF.reserve(LBA_MAX_LOCAL_FRAMES);   m_ucmsLF.resize(0);
#ifdef CFG_INCREMENTAL_PCG
  m_xcsLF.Reserve(LBA_MAX_LOCAL_FRAMES);    m_xcsLF.Resize(0);
  m_xmsLF.Reserve(LBA_MAX_LOCAL_FRAMES);    m_xmsLF.Resize(0);
#endif
  m_DsLF.Reserve(LBA_MAX_LOCAL_FRAMES);     m_DsLF.Resize(0);
#ifdef CFG_GROUND_TRUTH
  m_DsLFGT.Reserve(LBA_MAX_LOCAL_FRAMES);   m_DsLFGT.Resize(0);
#endif
  m_AdsLF.Reserve(LBA_MAX_LOCAL_FRAMES);    m_AdsLF.Resize(0);
  m_AfpsLF.Reserve(LBA_MAX_LOCAL_FRAMES);   m_AfpsLF.Resize(0);
  m_AfmsLF.Reserve(LBA_MAX_LOCAL_FRAMES);   m_AfmsLF.Resize(0);
  m_SAcusLF.Reserve(LBA_MAX_LOCAL_FRAMES);  m_SAcusLF.Resize(0);
  m_SMcusLF.Reserve(LBA_MAX_LOCAL_FRAMES);  m_SMcusLF.Resize(0);
  m_SAcmsLF.Reserve(LBA_MAX_LOCAL_FRAMES);  m_SAcmsLF.Resize(0);

  m_KFs.resize(0);
  m_CsKF.Resize(0);
#ifdef CFG_GROUND_TRUTH
  m_CsKFGT.Resize(0);
#endif
  m_ucsKF.resize(0);
  m_usKF.Resize(0);

  m_iKF2d.assign(1, 0);
  m_ds.resize(0);
  m_uds.resize(0);

  m_ZpLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY,
                    BA_VARIANCE_PRIOR_BIAS_ACCELERATION,
                    BA_VARIANCE_PRIOR_BIAS_GYROSCOPE);
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
  if (m_CsGT) {
    m_ZpLF.DebugSetMeasurement(m_CsGT[0]);
  }
#endif
  m_Zp.Initialize(m_ZpLF);
  m_ApLF.MakeZero();
  m_ZpKF.Invalidate();
  //m_F = 0.0f;
}

void LocalBundleAdjustor::PushCurrentFrame(const InputLocalFrame &ILF) {
  MT_WRITE_LOCK_BEGIN(m_MT, ILF.m_T.m_iFrm, MT_TASK_LBA_PushCurrentFrame);
  m_ILFs1.push_back(ILF);
  MT_WRITE_LOCK_END(m_MT, ILF.m_T.m_iFrm, MT_TASK_LBA_PushCurrentFrame);
}

void LocalBundleAdjustor::PushCurrentFrame(const InputLocalFrame &ILF,
                                           const GlobalMap::InputKeyFrame &IKF) {
  MT_WRITE_LOCK_BEGIN(m_MT, ILF.m_T.m_iFrm, MT_TASK_LBA_PushCurrentFrame);
  m_ILFs1.push_back(ILF);
  m_IKFs1.push_back(IKF);
  MT_WRITE_LOCK_END(m_MT, ILF.m_T.m_iFrm, MT_TASK_LBA_PushCurrentFrame);
}

void LocalBundleAdjustor::DeleteKeyFrame(const int iKF) {
  Synchronize();
#ifdef CFG_DEBUG
  UT_ASSERT(BufferDataEmpty());
#endif
  const int nKFs1 = static_cast<int>(m_KFs.size()), nKFs2 = nKFs1 - 1;
  const int Nd = static_cast<int>(m_KFs[iKF].m_xs.size());
  const int id1 = m_iKF2d[iKF], id2 = id1 + Nd;
  {
    FTR::Factor::DD daddST;
    KeyFrame &KF = m_KFs[iKF];
    const ubyte ucFlag = LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION |
                         LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION_KF;
    const ubyte udFlag = LBA_FLAG_TRACK_UPDATE_INFORMATION | LBA_FLAG_TRACK_UPDATE_INFORMATION_KF;
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      ubyte *_uds = m_uds.data() + m_iKF2d[_iKF];
      KeyFrame &_KF = m_KFs[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = KF.m_zs[iz].m_ix;
        FTR::Factor::Depth &A = KF.m_Azs[iz];
        A.m_add.MakeMinus();
        _KF.m_Axps[ix].m_Sadd += A.m_add;
        _KF.m_Axs[ix].m_Sadd += A.m_add;
        _uds[ix] |= udFlag;
        FTR::Factor::FixSource::Source::A &AST = _KF.m_AxpsST[ix];
        daddST = AST.m_Sadd;
        AST = _KF.m_Axps[ix];
        const int iST1 = _KF.m_ix2ST[ix], iST2 = _KF.m_ix2ST[ix + 1], Nst = iST2 - iST1;
        if (Nst > 1) {
          _KF.m_AxpsST[ix] *= 1.0f / Nst;
        }
        FTR::Factor::DD::amb(AST.m_Sadd, daddST, daddST);
        for (int iST = iST1; iST < iST2; ++iST) {
          _KF.m_AxsST[iST].m_Sadd += daddST;
          _KF.m_usST[iST] |= LBA_FLAG_TRACK_UPDATE_INFORMATION;
        }
      }
      if (Z.m_iz1 < Z.m_iz2) {
        m_ucsKF[_iKF] |= ucFlag;
      }
    }
    const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int jKF = KF.m_iKFsMatch[ik];
      if (jKF > iKF) {
        break;
      }
      m_KFs[jKF].DeleteMatchKeyFrame(iKF);
    }
  }
  for (int jKF = iKF + 1; jKF < nKFs1; ++jKF) {
    m_KFs[jKF].DeleteKeyFrame(iKF);
  }
  m_KFs.erase(m_KFs.begin() + iKF);
  for (int jKF = iKF + 1; jKF <= nKFs1; ++jKF) {
    m_iKF2d[jKF] -= Nd;
  }
  m_iKF2d.erase(m_iKF2d.begin() + iKF);
  m_CsKF.Erase(iKF);
  m_ucsKF.erase(m_ucsKF.begin() + iKF);
  //m_UcsKF.erase(m_UcsKF.begin() + iKF);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsKFGT.Erase(iKF);
  }
  if (m_history >= 3) {
    m_ucsKFGT.erase(m_ucsKFGT.begin() + iKF);
  }
#endif
  m_ds.erase(m_ds.begin() + id1, m_ds.begin() + id2);
  m_uds.erase(m_uds.begin() + id1, m_uds.begin() + id2);
  //m_Uds.erase(m_Uds.begin() + id1, m_Uds.begin() + id2);
  const int nLFs = static_cast<int>(m_LFs.size());
  m_iLF2Z.resize(nLFs);
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    LocalFrame &LF = m_LFs[iLF];
    m_iLF2Z[iLF] = std::lower_bound(LF.m_Zs.begin(), LF.m_Zs.end(), iKF);
  }
  for (int iLF1 = 0; iLF1 < nLFs; ++iLF1) {
    LocalFrame &LF1 = m_LFs[iLF1];
    const std::vector<FRM::Measurement>::iterator iZ1 = m_iLF2Z[iLF1];
    const bool z1 = iZ1 != LF1.m_Zs.end() && iZ1->m_iKF == iKF;
    const int Nk = static_cast<int>(LF1.m_iLFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int iLF2 = LF1.m_iLFsMatch[ik];
      const std::vector<FRM::Measurement>::iterator iZ2 = m_iLF2Z[iLF2];
      const bool z2 = iZ2 != m_LFs[iLF2].m_Zs.end() && iZ2->m_iKF == iKF;
      if (!z1 && !z2) {
        continue;
      }
      std::vector<FTR::Measurement::Match> &izms = LF1.m_Zm.m_izms;
      const int i1 = LF1.m_Zm.m_ik2zm[ik], i2 = LF1.m_Zm.m_ik2zm[ik + 1];
      if (z1) {
        const int Nz1 = iZ1->CountFeatureMeasurements();
        for (int i = i2 - 1; i >= i1 && izms[i].m_iz1 >= iZ1->m_iz2; --i) {
          izms[i].m_iz1 -= Nz1;
        }
      }
      if (z2) {
        const int Nz2 = iZ2->CountFeatureMeasurements();
        for (int i = i2 - 1; i >= i1 && izms[i].m_iz2 >= iZ2->m_iz2; --i) {
          izms[i].m_iz2 -= Nz2;
        }
      }
    }
  }
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    LocalFrame &LF = m_LFs[iLF];
    const std::vector<FRM::Measurement>::iterator iZ = m_iLF2Z[iLF];
    if (iZ != LF.m_Zs.end() && iZ->m_iKF == iKF) {
      Camera::Factor::Unitary::CC &SAczz = m_SAcusLF[iLF], &SMczz = m_SMcusLF[iLF];
      for (int iz = iZ->m_iz1; iz < iZ->m_iz2; ++iz) {
        Camera::Factor::Unitary::CC &Aczz = LF.m_Azs2[iz].m_Aczz;
        Aczz.MakeMinus();
        SAczz += Aczz;
        if (!(LF.m_ms[iz] & LBA_FLAG_MARGINALIZATION_NON_ZERO)) {
          continue;
        }
        Camera::Factor::Unitary::CC &Mczz = LF.m_Mzs2[iz].m_Mczz;
        Mczz.MakeMinus();
        SMczz += Mczz;
      }
    }
    LF.DeleteKeyFrame(iKF, &iZ);
    if (LF.m_iKFNearest != -1) {
      continue;
    }
    if (LBA_MARGINALIZATION_REFERENCE_NEAREST) {
      ubyte first = 1;
      int iKFNearest = -1;
      float imgMotionNearest = FLT_MAX;
      const Rigid3D &C = m_CsLF[iLF].m_T;
      const float z = 1.0f / LF.m_d.u();
      m_marksTmp.assign(nKFs2, 0);
      const int Nk = static_cast<int>(LF.m_iKFsMatch.size());
      for (int i = 0; i < Nk; ++i) {
        m_marksTmp[LF.m_iKFsMatch[i]] = 1;
      }
      for (int jKF = 0; jKF < nKFs2 && m_KFs[jKF].m_T < LF.m_T; ++jKF) {
        if (Nk > 0 && !m_marksTmp[jKF]) {
          continue;
        }
        const Rigid3D _C = m_CsKF[jKF];
        const float imgMotion = ComputeImageMotion(z, C, _C, &first);
        if (imgMotion > imgMotionNearest) {
          continue;
        }
        imgMotionNearest = imgMotion;
        iKFNearest = jKF;
      }
      LF.m_iKFNearest = iKFNearest;
    } else {
      LF.m_iKFNearest = iKF - 1;
    }
  }
  m_Zp.DeleteKeyFrame(iKF);
  if (m_Zp.Pose::Invalid()) {
    const int iLF = m_ic2LF.front(), iKFr = m_LFs[iLF].m_iKFNearest;
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
    if (m_CsGT) {
      m_ZpLF.DebugSetMeasurement(m_CsLFGT[iLF]);
    }
#endif
    if (m_LFs[iLF].m_T.m_iFrm == m_KFs[iKFr].m_T.m_iFrm) {
      m_Zp.Initialize(m_ZpLF);
    } else {
      m_Zp.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, iKFr, m_CsKF[iKFr],
                      BA_VARIANCE_PRIOR_ROTATION_INITIAL, m_ZpLF, false, &m_CsLF[iLF].m_T);
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
      if (m_CsGT) {
        m_Zp.Pose::DebugSetMeasurement(m_CsKFGT[iKFr], false, &m_CsLFGT[iLF].m_T);
      }
#endif
    }
  }
#ifdef CFG_GROUND_TRUTH
  if (m_history >= 3) {
    m_udsGT.erase(m_udsGT.begin() + id1, m_udsGT.begin() + id2);
  }
#endif
  m_GBA->DeleteKeyFrame(iKF, &m_usBkp);
  m_GM->LBA_Delete(iKF);
  if (iKF == nKFs2) {
    m_usKF.Insert(0, m_usBkp, &m_work);
  }
#ifdef CFG_DEBUG
  if (m_debug) {
    AssertConsistency(true, false);
  }
#endif
}

void LocalBundleAdjustor::GetCamera(FRM::Tag &T, Camera &C) {
  MT_READ_LOCK_BEGIN(m_MTC, MT_TASK_NONE, MT_TASK_LBA_GetCamera);
  T = m_C.m_T;
  C = m_C.m_C;
  MT_READ_LOCK_END(m_MTC, MT_TASK_NONE, MT_TASK_LBA_GetCamera);
}

void LocalBundleAdjustor::Run() {
//#ifdef CFG_DEBUG
#if 0
//#if 1
  if (m_debug >= 2) {
    AssertConsistency();
  }
#endif
  m_delta2 = BA_DL_RADIUS_INITIAL;
  m_ts[TM_TOTAL].Start();
  SynchronizeData();
#ifdef LBA_DEBUG_EIGEN
  DebugGenerateTracks();
#endif
  m_ts[TM_TOTAL].Stop();
#ifdef CFG_DEBUG
  if (m_debug < 0) {
    return;
  }
#endif
//#ifdef CFG_DEBUG
#if 0
//#if 1
  if (m_debug >= 2) {
    AssertConsistency(false);
  }
#endif
#ifdef LBA_DEBUG_CHECK
  {
    const int nKFs = int(m_KFs.size());
    if (nKFs != 0) {
      const int iKF = g_iKF < nKFs ? g_iKF : nKFs - 1;
      g_CKF = &m_CsKF[iKF];
      g_KF = &m_KFs[iKF];
      if (g_ix >= 0 && g_ix < int(g_KF->m_xs.size())) {
        g_d = m_ds.data() + m_iKF2d[iKF] + g_ix;
        g_Ax = &g_KF->m_Axs[g_ix];
        g_Mx = &g_KF->m_Mxs[g_ix];
        if (g_ist >= 0 && g_ist < g_KF->m_ix2ST[g_ix + 1] - g_KF->m_ix2ST[g_ix]) {
          const int iST = g_KF->m_ix2ST[g_ix] + g_ist;
          g_AxST = &g_KF->m_AxsST[iST];
          g_MxST = &g_KF->m_MxsST[iST];
        }
      }
      if (g_izKF >= 0 && g_izKF < g_KF->m_Azs.Size())
        g_AzKF = &g_KF->m_Azs[g_izKF];
    }
    const int nLFs = int(m_LFs.size());
    if (nLFs != 0) {
      const int ic = g_ic >= 0 && g_ic < nLFs ? g_ic : nLFs - 1;
      const int iLF = m_ic2LF[ic];
      g_CLF = m_CsLF.Data() + iLF;
      g_LF = &m_LFs[iLF];
      g_SAcuLF = m_SAcusLF.Data() + iLF;
      g_SMcuLF = m_SMcusLF.Data() + iLF;
      g_SAcmLF = m_SAcmsLF.Data() + iLF;
      if (g_izLF >= 0 && g_izLF < int(g_LF->m_zs.size())) {
        g_Az1LF = &g_LF->m_Azs1[g_izLF];
        g_Az2LF = &g_LF->m_Azs2[g_izLF];
        g_Mz1LF = &g_LF->m_Mzs1[g_izLF];
        g_Mz2LF = &g_LF->m_Mzs2[g_izLF];
        g_SmddST = &g_LF->m_SmddsST[g_izLF];
      }
    }
  }
#endif
#ifdef CFG_VERBOSE
  if (m_verbose >= 2) {
    const int NzLF = CountMeasurementsFeatureLF(), NzKF = CountMeasurementsFeatureKF();
    const int NsLF = CountSchurComplements();
    const int NST = CountSlidingTracks(), Nd = CountLocalTracks();
    UT::PrintSeparator('*');
    UT::Print("[%d] [LocalBundleAdjustor::Run]\n", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
    UT::Print("  FrameLF = %d\t\t\tMeasurement = %d\tSchur = %d\n", m_LFs.size(), NzLF, NsLF);
    UT::Print("  FrameKF = %d\t\t\tMeasurement = %d\n", m_KFs.size(), NzKF);
    UT::Print("  TrackST = %d * %.2f = %d\n", Nd, UT::Percentage(NST, Nd), NST);
#ifdef CFG_GROUND_TRUTH
    if (!m_CsLFGT.Empty() && !m_CsKFGT.Empty() && !m_DsLFGT.Empty() && m_dsGT) {
      UT::PrintSeparator();
      PrintErrorStatistic("*GT: ", m_CsLFGT, m_CsKFGT, *m_dsGT, m_DsLFGT, true);
    }
#endif
  }
#endif
//#ifdef CFG_DEBUG
#if 0
//#if 1
  {
    UT::Check("Noise\n");
    //const float erMax = 1.0f;
    const float erMax = 10.0f;
    //const float epMax = 0.0f;
    const float epMax = 0.1f;
    //const float epMax = 1.0f;
    //const float evMax = 0.0f;
    const float evMax = 0.01f;
    //const float ebaMax = 0.0f;
    const float ebaMax = 0.001f;
    //const float ebwMax = 0.0f;
    const float ebwMax = 1.0f * UT_FACTOR_DEG_TO_RAD;
    //const float edMax = 0.0f;
    const float edMax = 0.1f;
    const ubyte ucmFlag = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
    const int nLFs = int(m_LFs.size());
    for (int iLF = 0; iLF < nLFs; ++iLF) {
      Camera &C = m_CsLF[iLF];
      C.m_T = Rotation3D::GetRandom(erMax * UT_FACTOR_DEG_TO_RAD) * C.m_T;
      C.m_p += LA::AlignedVector3f::GetRandom(epMax);
      C.m_T.SetPosition(C.m_p);
      m_ucsLF[iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
      C.m_v += LA::AlignedVector3f::GetRandom(evMax);
      C.m_ba += LA::AlignedVector3f::GetRandom(ebaMax);
      C.m_bw += LA::AlignedVector3f::GetRandom(ebwMax);
      m_ucmsLF[iLF] |= ucmFlag;
      m_UcsLF[iLF] |= LM_FLAG_FRAME_UPDATE_CAMERA_LF;
    }
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const KeyFrame &KF = m_KFs[iKF];
      Depth::InverseGaussian *ds = m_ds.data() + KF.m_id;
      ubyte *uds = m_uds.data() + KF.m_id, *Uds = m_Uds.data() + KF.m_id;
      const int Nx = int(KF.m_xs.size());
      for (int ix = 0; ix < Nx; ++ix) {
        ds[ix].u() += UT::Random<float>(edMax);
        uds[ix] |= LBA_FLAG_TRACK_UPDATE_DEPTH;
        Uds[ix] |= LM_FLAG_TRACK_UPDATE_DEPTH;
      }
      m_ucsKF[iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
      m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_DEPTH;
    }
  }
#endif
#ifdef LBA_DEBUG_VIEW
  ViewerIBA *viewer = NULL;
  if (m_verbose >= 2) {
    viewer = new ViewerIBA();
    viewer->Create(m_solver);
    viewer->ActivateFrame(viewer->GetKeyFrames() + viewer->GetLocalFrames() - 1);
    viewer->m_keyPause = true;
    viewer->m_keyDrawCamTypeLF = ViewerIBA::DRAW_CAM_LF_LBA;
    viewer->m_keyDrawCamTypeKF = ViewerIBA::DRAW_CAM_KF_LBA;
    viewer->m_keyDrawDepType = ViewerIBA::DRAW_DEP_LBA;
  }
#endif
#if 0
//#if 1
  {
    UT::Check("Noise\n");
    UpdateFactors();
    UT::PrintSeparator();
    PrintErrorStatistic("    ", m_CsLF, m_CsKF, m_ds, m_DsLF, false);
    UT::Print("\n");
    const float edMax = 1.0f;
    const int Nd = int(m_ds.size());
    for (int id = 0; id < Nd; ++id) {
      if (m_uds[id] & LBA_FLAG_TRACK_UPDATE_DEPTH) {
        m_ds[id].u() += UT::Random<float>(edMax);
      }
    }
    UT::PrintSeparator();
    PrintErrorStatistic("--> ", m_CsLF, m_CsKF, m_ds, m_DsLF, false);
    UT::Print("\n");
#ifdef LBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run();
    }
#endif
    EmbeddedPointIteration();
    UT::PrintSeparator();
    PrintErrorStatistic("--> ", m_CsLF, m_CsKF, m_ds, m_DsLF, false);
    UT::Print("\n");
#ifdef LBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run();
    }
#endif
  }
#endif
  m_ts[TM_TOTAL].Start();
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  for (m_iIter = 0; m_iIter < BA_MAX_ITERATIONS; ++m_iIter) {
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 2) {
      m_debug = 3;
    } else {
      m_debug = 0;
    }
#endif
//#ifdef CFG_DEBUG
#if 0
//#if 1
    if (m_iIter == 0) {
      UT::Check("Update\n");
//#ifdef CFG_DEBUG
#if 0
      ((Depth::InverseGaussian *) g_d)->u() = 0.0f;
#endif
      const int Nc = int(m_LFs.size());
      for (int ic = 0; ic < Nc; ++ic) {
        const int iLF = m_ic2LF[ic];
        m_ucsLF[iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
        m_ucmsLF[iLF] |= LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION | LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                         LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                         LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION | LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
        m_UcsLF[iLF] |= LM_FLAG_FRAME_UPDATE_CAMERA_LF;
        m_LFs[iLF].MakeZero();
        if (ic == 0) {
          continue;
        }
        IMU::Delta &D = m_DsLF[iLF];
        const LocalFrame &LF = m_LFs[iLF];
        const int _iLF = m_ic2LF[ic - 1];
        IMU::PreIntegrate(LF.m_us, m_LFs[_iLF].m_T.m_t, LF.m_T.m_t, m_CsLF[_iLF], &D, &m_work,
                          true, D.m_u1.Valid() ? &D.m_u1 : NULL, D.m_u2.Valid() ? &D.m_u2 : NULL);
      }
      m_AdsLF.MakeZero();
      m_AfpsLF.MakeZero();
      m_AfmsLF.MakeZero();
      const int nKFs = int(m_KFs.size());
      for (int iKF = 0; iKF < nKFs; ++iKF) {
        m_ucsKF[iKF] |= LBA_FLAG_FRAME_UPDATE_CAMERA | LBA_FLAG_FRAME_UPDATE_DEPTH;
        m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_CAMERA_KF | LM_FLAG_FRAME_UPDATE_DEPTH;
        m_KFs[iKF].MakeZero();
      }
      const int Nd = int(m_ds.size());
      for (int id = 0; id < Nd; ++id) {
        m_uds[id] |= LBA_FLAG_TRACK_UPDATE_DEPTH | LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
        m_Uds[id] |= LM_FLAG_TRACK_UPDATE_DEPTH;
      }
      m_SAcusLF.MakeZero();
      m_SMcusLF.MakeZero();
      m_SAcmsLF.MakeZero();
    }
#endif
#ifdef CFG_VERBOSE
    if (m_verbose) {
      if (m_verbose >= 2) {
        UT::PrintSeparator('*');
      } else if (m_iIter == 0) {
        UT::PrintSeparator();
      }
      PrintErrorStatistic(UT::String("*%2d: ", m_iIter), m_CsLF, m_CsKF, m_ds, m_DsLF,
                          m_verbose >= 2);
    }
#endif
#ifdef LBA_DEBUG_VIEW
    if (viewer) {
      viewer->Run(true, false);
    }
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] UpdateFactors Start\t\t\t", iFrm);
#endif
    m_ts[TM_FACTOR].Start();
    UpdateFactors();
    m_ts[TM_FACTOR].Stop();
#ifdef LBA_DEBUG_EIGEN
    DebugUpdateFactors();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] UpdateFactors Stop\t\t\t", iFrm);
#endif
#if 0
//#if 1
    UT::DebugStart();
    const ES ES1 = ComputeErrorStatistic(m_CsLF, m_CsKF, m_ds, m_DsLF);
    SolveSchurComplementGT(&m_xs);
    SolveBackSubstitutionGT(&m_xs);
    const ES ES2 = ComputeErrorStatistic(m_CsLF, m_xs, false);
    UT::DebugStop();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] UpdateSchurComplement Start\t\t\t", iFrm);
#endif
    m_ts[TM_SCHUR_COMPLEMENT].Start();
    UpdateSchurComplement();
    m_ts[TM_SCHUR_COMPLEMENT].Stop();
#ifdef LBA_DEBUG_EIGEN
    DebugUpdateSchurComplement();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] UpdateSchurComplement Stop\t\t\t", iFrm);
#endif

#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] SolveSchurComplement Start\t\t\t", iFrm);
#endif
    m_ts[TM_CAMERA].Start();
    const bool scc = SolveSchurComplement();
    m_ts[TM_CAMERA].Stop();
#ifdef LBA_DEBUG_EIGEN
    DebugSolveSchurComplement();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] SolveSchurComplement Stop\t\t\t", iFrm);
#endif
    //if (!scc) {
    //  m_update = false;
    //  m_converge = false;
    //  break;
    //}
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] SolveBackSubstitution Start\t\t\t", iFrm);
#endif
    m_ts[TM_DEPTH].Start();
    SolveBackSubstitution();
    m_ts[TM_DEPTH].Stop();
#ifdef LBA_DEBUG_EIGEN
    DebugSolveBackSubstitution();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
    UT::Print("\r[%d] SolveBackSubstitution Stop\t\t\t", iFrm);
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
#ifdef LBA_DEBUG_PRINT_STEP
        UT::Print("\r[%d] SolveGradientDescent Start\t\t\t", iFrm);
#endif
        m_ts[TM_UPDATE].Start();
        SolveGradientDescent();
        m_ts[TM_UPDATE].Stop();
#ifdef LBA_DEBUG_EIGEN
        DebugSolveGradientDescent();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
        UT::Print("\r[%d] SolveGradientDescent Stop\t\t\t", iFrm);
#endif
      }
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] SolveDogLeg Start\t\t\t", iFrm);
#endif
      m_ts[TM_UPDATE].Start();
      SolveDogLeg();
      UpdateStatesPropose();
      m_ts[TM_UPDATE].Stop();
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] SolveDogLeg Stop\t\t\t", iFrm);
#endif
#ifdef CFG_VERBOSE
      if (m_verbose) {
        if (m_verbose >= 3) {
          UT::PrintSeparator();
        }
        const std::string str = m_verbose >= 2 ? " --> " :
                                (m_iIterDL == 0 ? " > " : std::string(N + 3, ' '));
        PrintErrorStatistic(str, m_CsLFBkp, m_CsKF, m_dsBkp, m_xsDL, m_verbose >= 2);
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
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] ComputeReduction Start\t\t\t", iFrm);
#endif
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
#ifdef LBA_DEBUG_EIGEN
      DebugComputeReduction();
#endif
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] ComputeReduction Stop\t\t\t", iFrm);
#endif
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] UpdateStatesDecide Start\t\t\t", iFrm);
#endif
      m_ts[TM_UPDATE].Start();
      const bool accept = UpdateStatesDecide();
      m_ts[TM_UPDATE].Stop();
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] UpdateStatesDecide Stop\t\t\t", iFrm);
#endif
      if (accept) {
        break;
      }
    }
    if (m_iIterDL == BA_DL_MAX_ITERATIONS) {
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] UpdateStatesDecide Start\t\t\t", iFrm);
#endif
      m_ts[TM_UPDATE].Start();
      UpdateStatesDecide();
      m_ts[TM_UPDATE].Stop();
#ifdef LBA_DEBUG_PRINT_STEP
      UT::Print("\r[%d] UpdateStatesDecide Stop\t\t\t", iFrm);
#endif
    }
//#ifdef CFG_DEBUG
#if 1
    if (m_debug >= 2) {
      AssertConsistency();
    }
#endif
    if (!m_update || m_converge) {
      break;
    }
    if (LBA_EMBEDDED_POINT_ITERATION) {
      m_ts[TM_UPDATE].Start();
      EmbeddedPointIteration(m_CsLF, m_CsKF, m_ucsKF, m_uds, &m_ds);
      m_ts[TM_UPDATE].Stop();
    }
    MT_READ_LOCK_BEGIN(m_MT, iFrm, MT_TASK_LBA_BufferDataEmpty);
    m_empty = BufferDataEmpty();
    MT_READ_LOCK_END(m_MT, iFrm, MT_TASK_LBA_BufferDataEmpty);
    if (!m_empty) {
      break;
    }
  }
#ifdef CFG_VERBOSE
  if (m_verbose) {
    UT::PrintSeparator('*');
    PrintErrorStatistic(UT::String("*%2d: ", m_iIter), m_CsLF, m_CsKF, m_ds, m_DsLF,
                        m_verbose >= 2);
  }
#endif
  m_ts[TM_TOTAL].Stop();
  for (int i = 0; i < TM_TYPES; ++i) {
    m_ts[i].Finish();
  }
  if (m_history >= 1) {
    History hist;
    hist.MakeZero();
    const int iLF = m_ic2LF.back();
    hist.m_C = m_CsLF[iLF];
    //hist.m_C = m_CsLFGT[iLF];
    hist.m_t = m_LFs[iLF].m_T.m_t;
    for (int i = 0; i < TM_TYPES; ++i) {
      hist.m_ts[i] = m_ts[i].GetAverageSeconds() * 1000.0;
    }
    //hist.m_Nd = CountLocalTracks();
    if (m_history >= 2) {
      hist.m_ESa = ComputeErrorStatistic(m_CsLF, m_CsKF, m_ds, m_DsLF);
      hist.m_ESb = ComputeErrorStatistic(m_CsLFBkp, m_CsKF, m_dsBkp, m_DsLF);
      hist.m_ESp = ComputeErrorStatistic(m_CsLFBkp, m_CsKF, m_dsBkp, m_xsDL, false);
#ifdef CFG_GROUND_TRUTH
      if (m_CsGT) {
        SolveSchurComplementGT(m_CsLFBkp, &m_xsGT);
        if (m_dsGT) {
          if (m_history >= 3) {
            EmbeddedPointIteration(m_CsLFGT, m_CsKFGT, m_ucsKFGT, m_udsGT, m_dsGT);
          }
          SolveBackSubstitutionGT(m_dsBkp, &m_xsGT);
          hist.m_ESaGT = ComputeErrorStatistic(m_CsLFGT, m_CsKFGT, *m_dsGT, m_DsLFGT);
          hist.m_ESpGT = ComputeErrorStatistic(m_CsLFBkp, m_CsKFBkp, m_dsBkp, m_xsGT, false);
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
      if (m_Zp.Pose::Valid()) {
        //hist.m_PR.m_r2c = m_Zp.m_br.SquaredLength() + m_Zp.m_bc.SquaredLength();
        //hist.m_PR.m_r2m = m_Zp.m_bm.SquaredLength();
        //hist.m_PR.m_r2 = hist.m_PR.m_r2c + hist.m_PR.m_r2m;
#ifdef CFG_DEBUG
        UT_ASSERT(m_Zp.m_iKFs.back() == INT_MAX);
#endif
        const float eps = 0.0f;
        const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
        const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
        const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
        const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
        const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
        const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                              epsba, epsba, epsba, epsbw, epsbw, epsbw};
        m_Zp.GetPriorPose(INT_MAX, &m_ZpKF, &m_work, _eps);
        m_Zp.GetPriorMotion(&m_ZpLF, &m_work, _eps);
        hist.m_PR.m_r2 = m_Zp.m_br.SquaredLength() + m_Zp.m_bc.SquaredLength() +
                         m_Zp.m_bm.SquaredLength();
        hist.m_PR.m_r2c = m_ZpKF.m_br.SquaredLength() + m_ZpKF.m_bc.SquaredLength();
        hist.m_PR.m_r2m = m_ZpLF.m_bm.SquaredLength();
#ifdef CFG_GROUND_TRUTH
        if (m_CsGT) {
          CameraPrior::Joint::Error e;
          CameraPrior::Vector::C ec;
          CameraPrior::Block::M em;
          CameraPrior::Block::R rr;
          CameraPrior::Vector::C rc;
          CameraPrior::Block::M rm;
          const int iLF = m_ic2LF.front();
          const Camera &CGT = m_CsLFGT[iLF];
          m_CsKFGT.Push(CGT.m_T);
          m_Zp.GetResidual(m_CsKFGT, CGT, &e, &ec, &em, &rr, &rc, &rm);
          hist.m_PRGT.m_r2 = rr.SquaredLength() + rc.SquaredLength() + rm.SquaredLength();
          m_ZpKF.GetResidual(m_CsKFGT, &e.m_ec, &ec, &rr, &rc);
          hist.m_PRGT.m_r2c = rr.SquaredLength() + rc.SquaredLength();
          m_ZpLF.GetResidual(CGT, &e.m_em, &em, &rm);
          hist.m_PRGT.m_r2m = rm.SquaredLength();
          m_CsKFGT.Resize(m_CsKFGT.Size() - 1);
        }
#endif
        LA::AlignedVectorXf x;
        LA::AlignedMatrixXf S;
        if (m_Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &x, &S, &m_work)) {
          const float *_x = x.Data();
          const Rigid3D &Cr = m_CsKF[m_Zp.m_iKFr];
          hist.m_PE.m_er = sqrtf(m_Zp.GetReferenceRotationError(Cr, _x).SquaredLength()) *
                           UT_FACTOR_RAD_TO_DEG;
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            const LA::AlignedVector3f er = m_Zp.GetReferenceRotationError(m_CsKFGT[m_Zp.m_iKFr], _x);
            hist.m_PEGT.m_er = sqrtf(er.SquaredLength()) * UT_FACTOR_RAD_TO_DEG;
          }
#endif
          _x += 2;
          const Camera &C = m_CsLF[iLF];
          const int Nk = static_cast<int>(m_Zp.m_iKFs.size());
          for (int i = 0; i < Nk; ++i) {
            const int iKF = m_Zp.m_iKFs[i];
            const Rigid3D &Ci = iKF == INT_MAX ? C.m_T : m_CsKF[iKF];
            const float ep = sqrtf(m_Zp.GetPositionError(Cr, Ci, i, _x).SquaredLength());
            hist.m_PE.m_ep = std::max(hist.m_PE.m_ep, ep);
#ifdef CFG_GROUND_TRUTH
            if (m_CsGT) {
              const Rigid3D &CrGT = m_CsKFGT[m_Zp.m_iKFr];
              const Rigid3D &CiGT = iKF == INT_MAX ? m_CsLFGT[iLF].m_T : m_CsKFGT[iKF];
              const float epGT = sqrtf(m_Zp.GetPositionError(CrGT, CiGT, i, _x).SquaredLength());
              hist.m_PEGT.m_ep = std::max(hist.m_PEGT.m_ep, epGT);
            }
#endif
            _x += 3;
            const float er = sqrtf(m_Zp.GetRotationError(Cr, Ci, i, _x).SquaredLength()) *
                             UT_FACTOR_RAD_TO_DEG;
            hist.m_PE.m_er = std::max(hist.m_PE.m_er, er);
#ifdef CFG_GROUND_TRUTH
            if (m_CsGT) {
              const Rigid3D &CrGT = m_CsKFGT[m_Zp.m_iKFr];
              const Rigid3D &CiGT = iKF == INT_MAX ? m_CsLFGT[iLF].m_T : m_CsKFGT[iKF];
              const float erGT = sqrtf(m_Zp.GetRotationError(CrGT, CiGT, i, _x).SquaredLength()) *
                                 UT_FACTOR_RAD_TO_DEG;
              hist.m_PEGT.m_er = std::max(hist.m_PEGT.m_er, erGT);
            }
#endif
            _x += 3;
          }
          hist.m_PE.m_ev = sqrtf(m_Zp.GetVelocityError(C, _x).SquaredLength());
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            const LA::AlignedVector3f evGT = m_Zp.GetVelocityError(m_CsLFGT[iLF], _x);
            hist.m_PEGT.m_ev = sqrtf(evGT.SquaredLength());
          }
#endif
          _x += 3;
          hist.m_PE.m_eba = sqrtf(m_Zp.GetBiasAccelerationError(C, _x).SquaredLength());
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            const LA::AlignedVector3f ebaGT = m_Zp.GetBiasAccelerationError(m_CsLFGT[iLF], _x);
            hist.m_PEGT.m_eba = sqrtf(ebaGT.SquaredLength());
          }
#endif
          _x += 3;
          hist.m_PE.m_ebw = sqrtf(m_Zp.GetBiasGyroscopeError(C, _x).SquaredLength()) *
                            UT_FACTOR_RAD_TO_DEG;
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            const LA::AlignedVector3f ebwGT = m_Zp.GetBiasGyroscopeError(m_CsLFGT[iLF], _x);
            hist.m_PEGT.m_ebw = sqrtf(ebwGT.SquaredLength()) * UT_FACTOR_RAD_TO_DEG;
          }
#endif
        }
      }
    }
    m_hists.push_back(hist);
  }
#if 0
//#if 1
  UT::PrintSeparator();
  UT::Print("Time = %f\n", m_ts[TM_TOTAL].GetAverageMilliseconds() * 1000.0);
#endif
#ifdef LBA_DEBUG_VIEW
  if (viewer) {
    viewer->m_keyPause = true;
    viewer->Run();
    delete viewer;
  }
#endif
#if 0
//#if 1
  {
    FILE *fp = fopen(UT::String("D:/tmp/%04d.txt", iFrm).c_str(), "rb");
    m_CsLF.LoadB(fp);
    m_CsLFLP.LoadB(fp);
    UT::VectorLoadB(m_ucsLF, fp);
    UT::VectorLoadB(m_ucmsLF, fp);
    UT::VectorLoadB(m_UcsLF, fp);
    m_CsKF.LoadB(fp);
    m_CsKFLP.LoadB(fp);
    UT::VectorLoadB(m_ucsKF, fp);
    UT::VectorLoadB(m_UcsKF, fp);
    UT::VectorLoadB(m_ds, fp);
    UT::VectorLoadB(m_dsLP, fp);
    UT::VectorLoadB(m_uds, fp);
    UT::VectorLoadB(m_Uds, fp);
    fclose(fp);
  }
#endif
  UpdateData();
//#ifdef CFG_DEBUG
#if 1
  //if (!m_serial && m_debug)
  if (m_debug) {
    AssertConsistency();
  }
#endif
#if 0
//#if 1
  {
    FILE *fp = fopen(UT::String("D:/tmp/%04d.txt", iFrm).c_str(), "wb");
    m_CsLF.SaveB(fp);
    m_CsLFLP.SaveB(fp);
    UT::VectorSaveB(m_ucsLF, fp);
    UT::VectorSaveB(m_ucmsLF, fp);
    UT::VectorSaveB(m_UcsLF, fp);
    m_CsKF.SaveB(fp);
    m_CsKFLP.SaveB(fp);
    UT::VectorSaveB(m_ucsKF, fp);
    UT::VectorSaveB(m_UcsKF, fp);
    UT::VectorSaveB(m_ds, fp);
    UT::VectorSaveB(m_dsLP, fp);
    UT::VectorSaveB(m_uds, fp);
    UT::VectorSaveB(m_Uds, fp);
    fclose(fp);
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  if (m_KFs.size() == 2) {
    UT::PrintSeparator();
    m_CsKF[0].Print(true);
    UT::PrintSeparator();
    m_CsKF[1].Print(true);
  }
#endif
#ifdef LBA_DEBUG_PRINT
  {
    Camera Cc;
    Rigid3D Ck;
    Camera::Factor::Unitary::CC A;
    Camera::Factor::Binary::CC M;
    double Su, Ss2, Sadd, SmddST;
    UT::PrintSeparator('*');
    for (int i = 0; i < 2; ++i) {
      if (i == 0) {
        const int Nc = static_cast<int>(m_LFs.size());
        const int ic = Nc - 1, iLF = m_ic2LF[ic];
        const LocalFrame &LF = m_LFs[iLF];
        const int iKFNearest = LF.m_iKFNearest == -1 ? LF.m_iKFsMatch.front() : LF.m_iKFNearest;
        UT::Print("[%d] <-- [%d]\n", LF.m_T.m_iFrm, m_KFs[iKFNearest].m_T.m_iFrm);
        Cc = m_CsLF[iLF];
        Ck.MakeIdentity();
        const int NkKF = static_cast<int>(LF.m_iKFsMatch.size());
        for (int ik = 0; ik < NkKF; ++ik)
          Ck = m_CsKF[LF.m_iKFsMatch[ik]] * Ck;
        A = m_SAcusLF[iLF] - m_SMcusLF[iLF];
        M.MakeZero();
        const int NkLF = std::min(Nc, LBA_MAX_SLIDING_TRACK_LENGTH) - 1;
        xp128f sm; sm.vdup_all_lane(1.0f / NkLF);
        for (int ik = 0, _ic = ic - 1; ik < NkLF; ++ik, --_ic) {
          const int _iLF = m_ic2LF[_ic];
          const LocalFrame &_LF = m_LFs[_iLF];
#ifdef CFG_DEBUG
          UT_ASSERT(_LF.m_iLFsMatch.back() == iLF);
#endif
          M += _LF.m_Zm.m_SMczms.Back() * sm;
        }
        Su = Ss2 = Sadd = SmddST = 0.0;
        const int NZ = static_cast<int>(LF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
          const FRM::Measurement &Z = LF.m_Zs[iZ];
          const KeyFrame &KF = m_KFs[Z.m_iKF];
          const Depth::InverseGaussian *ds = m_ds.data() + KF.m_id;
          for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
            const int ix = LF.m_zs[iz].m_ix;
            const Depth::InverseGaussian &d = ds[ix];
            Su = d.u() + Su;
            Ss2 = d.s2() + Ss2;
            Sadd = KF.m_Axps[ix].m_Sadd.m_a + Sadd;
            SmddST = LF.m_SmddsST[iz].m_a + SmddST;
#ifdef CFG_DEBUG
            UT::Print("%d %.10e %.10e\n", iz, d.u(), Su);
#endif
//#ifdef CFG_DEBUG
#if 0
            UT::Print("%d %.10e %.10e\n", iz, SmddST, LF.m_SmddsST[iz].m_a);
#endif
          }
        }
      } else {
        Cc.MakeIdentity();
        Ck.MakeIdentity();
        A.MakeZero();
        M.MakeZero();
        Su = Ss2 = Sadd = SmddST = 0.0;
        const xp128f sc = xp128f::get(1.0f / nLFs);
        const xp128f sm = xp128f::get(1.0f / (CountSchurComplements() - nLFs));
        for (int iLF = 0; iLF < nLFs; ++iLF) {
          const Camera &C = m_CsLF[iLF];
          Cc.m_T = C.m_T * Cc.m_T;
          Cc.m_v += C.m_v * sc;
          Cc.m_ba += C.m_ba * sc;
          Cc.m_bw += C.m_bw * sc;
          A += (m_SAcusLF[iLF] - m_SMcusLF[iLF]) * sc;
//#ifdef CFG_DEBUG
#if 0
          UT::Print("%d %.10e\n", iLF, A.m_A.m00());
#endif
          const LocalFrame &LF = m_LFs[iLF];
          const int Nk = static_cast<int>(LF.m_iLFsMatch.size());
          for (int ik = 0; ik < Nk; ++ik) {
            M += LF.m_Zm.m_SMczms[ik] * sm;
          }
        }
        const int nKFs = static_cast<int>(m_KFs.size());
        for (int iKF = 0; iKF < nKFs; ++iKF) {
          Ck = m_CsKF[iKF] * Ck;
          const KeyFrame &KF = m_KFs[iKF];
          const Depth::InverseGaussian *ds = m_ds.data() + KF.m_id;
          const int Nx = static_cast<int>(KF.m_xs.size());
          for (int ix = 0; ix < Nx; ++ix) {
            const Depth::InverseGaussian &d = ds[ix];
            Su = d.u() + Su;
            Ss2 = d.s2() + Ss2;
            Sadd = KF.m_Axps[ix].m_Sadd.m_a + Sadd;
          }
        }
        for (int iLF = 0; iLF < nLFs; ++iLF) {
          const LocalFrame &LF = m_LFs[iLF];
          const int Nz = static_cast<int>(LF.m_zs.size());
          for (int iz = 0; iz < Nz; ++iz) {
            SmddST = LF.m_SmddsST[iz].m_a + SmddST;
          }
        }
      }
      UT::Print("C:\n");
      Cc.m_T.Print(true);
      Cc.m_v.Print("", true, false);
      Cc.m_ba.Print(" ", true, false);
      Cc.m_bw.Print(" ", true, true);
      Ck.Print(true);
      UT::Print("A:\n");
      A.Print(true);
      M.Print(true);
      UT::Print("d: %.10e %.10e %.10e %.10e\n", Su, Ss2, Sadd, SmddST);
    }
    UT::PrintSeparator('*');
  }
  const float xGN = m_xsGN.Mean(), s2GN = m_xsGN.Variance(xGN);
  const float xGD = m_xsGD.Mean(), s2GD = m_xsGD.Variance(xGD);
  const float xDL = m_xsDL.Mean(), s2DL = m_xsDL.Variance(xDL);
  UT::Print("x: %e %e %e %e %e %e\n", xGN, xGD, xDL, s2GN, s2GD, s2DL);
#endif
//#ifndef WIN32
#if 0
  UT::Print("[%d]\n", iFrm);
#endif
//#ifdef CFG_DEBUG
#if 0
  UT::Print("%.10e\n", m_SAcusLF[m_ic2LF.back()].m_b.v0());
#endif
//#ifdef CFG_DEBUG
#if 0
  {
    const LocalFrame &LF = m_LFs[m_ic2LF.back()];
    const int NZ = static_cast<int>(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const Depth::InverseGaussian *ds = m_ds.data() + m_KFs[Z.m_iKF].m_id;
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = LF.m_zs[iz].m_ix;
        UT::Print("%d (%d %d) %.10e\n", iz, Z.m_iKF, ix, ds[ix].u());
      }
    }
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  const LA::AlignedVector3f &v = m_CsLF[m_ic2LF.back()].m_v;
  //const LA::AlignedVector3f &v = m_CsLF[m_ic2LF.back()].m_bw;
  UT::Print("[%d] %.10e %.10e %.10e\n", iFrm, v.x(), v.y(), v.z());
#endif
//#ifdef CFG_DEBUG
#if 0
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    const int _iFrm = m_LFs[iLF].m_T.m_iFrm;
    //if (_iFrm == 1500) {
    //  UT::Print("[%d] %.10e\n", iFrm, m_SAcusLF[iLF].m_b.v4());
    if (_iFrm == 1506)
      UT::Print("[%d] %.10e\n", iFrm, m_DsLF[iLF].m_Jpbw.m00());
  }
#endif

  // Trigger LBA callback function if set
  if (m_callback) {
    //UT::Print("Trigger LBA callback [iFrm, ts]  = [%d, %f]\n", iFrm,
    //          m_LFs[m_ic2LF.back()].m_T.m_t);
    m_callback(iFrm, m_LFs[m_ic2LF.back()].m_T.m_t);
  }
}

void LocalBundleAdjustor::SetCallback(const IBA::Solver::IbaCallback& iba_callback) {
  m_callback = iba_callback;
}

//int LocalBundleAdjustor::GetTotalPoints(int *N) {
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

float LocalBundleAdjustor::GetTotalTime(int *N) {
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

bool LocalBundleAdjustor::SaveTimes(const std::string fileName) {
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

bool LocalBundleAdjustor::SaveCameras(const std::string fileName, const bool poseOnly) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  Point3D p;
  Quaternion q;
  Rotation3D R;
  LA::AlignedVector3f ba, bw;
  const Rotation3D RuT = m_K.m_Ru.GetTranspose();
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const History &hist = m_hists[i];
    const Rigid3D &T = hist.m_C.m_T;
    p = hist.m_C.m_p + T.GetAppliedRotationInversely(m_K.m_pu);
    Rotation3D::AB(RuT, T, R);
    R.GetQuaternion(q);
    fprintf(fp, "%f %f %f %f %f %f %f %f", hist.m_t, p.x(), p.y(), p.z(),
                                           q.x(), q.y(), q.z(), q.w());
    if (!poseOnly) {
      RuT.Apply(hist.m_C.m_ba, ba);
      RuT.Apply(hist.m_C.m_bw, bw);
      ba += m_K.m_ba;
      bw += m_K.m_bw;
      const LA::AlignedVector3f &v = hist.m_C.m_v;
      fprintf(fp, " %f %f %f %f %f %f %f %f %f", v.x(), v.y(), v.z(),
                                                 ba.x(), ba.y(), ba.z(),
                                                 bw.x(), bw.y(), bw.z());
    }
    fprintf(fp, "\n");
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

bool LocalBundleAdjustor::SaveCosts(const std::string fileName, const int type) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const History &hist = m_hists[i];
    switch (type) {
    case 0: hist.m_ESa.Save(fp);    break;
    case 1: hist.m_ESb.Save(fp);    break;
    case 2: hist.m_ESp.Save(fp);    break;
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

bool LocalBundleAdjustor::SaveResiduals(const std::string fileName, const int type) {
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

bool LocalBundleAdjustor::SavePriors(const std::string fileName, const int type) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  const int N = static_cast<int>(m_hists.size());
  for (int i = 0; i < N; ++i) {
    const History &hist = m_hists[i];
    switch (type) {
    case 0: hist.m_PR.Save(fp);   break;
#ifdef CFG_GROUND_TRUTH
    case 1: hist.m_PRGT.Save(fp); break;
#endif
    case 2: hist.m_PE.Save(fp);   break;
#ifdef CFG_GROUND_TRUTH
    case 3: hist.m_PEGT.Save(fp); break;
#endif
    }
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

void LocalBundleAdjustor::ComputeErrorFeature(float *ex) {
  float exi;
  *ex = 0.0f;
  const int nLFs = int(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    ComputeErrorFeature(&m_LFs[iLF], m_CsLF[iLF].m_T, m_CsKF, m_ds, &exi);
    *ex = std::max(exi, *ex);
  }
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    ComputeErrorFeature(&m_KFs[iKF], m_CsKF[iKF], m_CsKF, m_ds, &exi, iKF);
    *ex = std::max(exi, *ex);
  }
}

void LocalBundleAdjustor::ComputeErrorFeature(const FRM::Frame *F, const Rigid3D &C,
                                              const AlignedVector<Rigid3D> &CsKF,
                                              const std::vector<Depth::InverseGaussian> &ds,
                                              float *ex, const int iKF) {
  Rigid3D Tr[2];
  FTR::Error e;
#if 0
  float Se2 = 0.0f;
#ifdef CFG_STEREO
  float Se2r = 0.0f;
#endif
  int SN = 0;
#else
  const int Nz = int(F->m_zs.size());
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
  const int NZ = int(F->m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F->m_Zs[iZ];
    *Tr = C / CsKF[Z.m_iKF];
#ifdef CFG_STEREO
    Tr[1] = Tr[0];
    Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
    const Depth::InverseGaussian *_ds = ds.data() + m_iKF2d[Z.m_iKF];
    const KeyFrame &KF = m_KFs[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const FTR::Measurement &z = F->m_zs[iz];
      const int ix = z.m_ix;
      FTR::GetError(Tr, KF.m_xs[ix], _ds[ix], z, e);
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
        Se2r = e2r + Se2r;
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
  if (iKF != -1) {
    const Depth::InverseGaussian *_ds = ds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = *((KeyFrame *) F);
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (KF.m_xs[ix].m_xr.Invalid())
        continue;
      FTR::GetError(m_K.m_br, _ds[ix], KF.m_xs[ix], e.m_exr);
      const float e2r = e.m_exr.SquaredLength();
#if 0
      Se2r = e2r + Se2r;
      ++SN;
#else
      e2rs.Push(e2r);
#endif
    }
  }
#endif
#if 0
#ifdef CFG_STEREO
  *ex = sqrtf((Se2 * m_K.m_K.fxy() + Se2r * m_K.m_Kr.fxy()) / SN);
#else
  *ex = sqrtf(Se2 * m_K.m_K.fxy() / SN);
#endif
#else
  if (!e2s.Empty()) {
    const int ith = e2s.Size() >> 1;
    std::nth_element(e2s.Data(), e2s.Data() + ith, e2s.End());
    *ex = sqrtf(e2s[ith] * m_K.m_K.fxy());
  } else
    *ex = 0.0f;
#ifdef CFG_STEREO
  if (!e2rs.Empty()) {
    const int ith = e2rs.Size() >> 1;
    std::nth_element(e2rs.Data(), e2rs.Data() + ith, e2rs.End());
    *ex = std::max(sqrtf(e2rs[ith] * m_K.m_K.fxy()), *ex);
  }
#endif
#endif
}

void LocalBundleAdjustor::ComputeErrorIMU(float *er, float *ep, float *ev,
                                          float *eba, float *ebw) {
  *er = *ep = *ev = *eba = *ebw = 0.0f;
  const int nLFs = int(m_LFs.size());
  for (int ic1 = 0, ic2 = 1; ic2 < nLFs; ic1 = ic2++) {
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    const IMU::Delta::Error e = m_DsLF[iLF2].GetError(m_CsLF[iLF1], m_CsLF[iLF2], m_K.m_pu);
    *er = std::max(e.m_er.SquaredLength(), *er);
    *ep = std::max(e.m_ep.SquaredLength(), *ep);
    *ev = std::max(e.m_ev.SquaredLength(), *ev);
    *eba = std::max(e.m_eba.SquaredLength(), *eba);
    *ebw = std::max(e.m_ebw.SquaredLength(), *ebw);
  }
  *er *= UT_FACTOR_RAD_TO_DEG;
  *ebw *= UT_FACTOR_RAD_TO_DEG;
}

void LocalBundleAdjustor::ComputeErrorDrift(float *er, float *ep) {
  *er = 0.0f;
  *ep = 0.0f;
#ifdef CFG_GROUND_TRUTH
  if (m_CsLFGT.Empty() || m_CsKFGT.Empty()) {
    return;
  }
  Rigid3D Tr, TrGT, Te;
  LA::AlignedVector3f _er, _ep;
  const int nLFs = int(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    const int iKFNearest = m_LFs[iLF].m_iKFNearest;
    Tr = m_CsLF[iLF].m_T / m_CsKF[iKFNearest];
    TrGT = m_CsLFGT[iLF].m_T / m_CsKFGT[iKFNearest];
    Te = Tr / TrGT;
    Te.GetRodrigues(_er);
    Te.GetPosition(_ep);
    *er = std::max(_er.SquaredLength(), *er);
    *ep = std::max(_ep.SquaredLength(), *ep);
  }
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iKFNearest = m_KFs[iKF].m_iKFNearest;
    if (iKFNearest == -1) {
      continue;
    }
    Tr = m_CsKF[iKF] / m_CsKF[iKFNearest];
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

float LocalBundleAdjustor::GetTotalDistance() {
  float S = 0.0f;
  const int N = static_cast<int>(m_hists.size());
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    for (int i1 = 0, i2 = 1; i2 < N; i1 = i2++) {
      S += sqrtf((m_CsGT[i1].m_p - m_CsGT[i2].m_p).SquaredLength());
    }
  } else
#endif
  {
    for (int i1 = 0, i2 = 1; i2 < N; i1 = i2++) {
      S += sqrtf((m_hists[i1].m_C.m_p - m_hists[i2].m_C.m_p).SquaredLength());
    }
  }
  return S;
}

void LocalBundleAdjustor::SynchronizeData() {
#ifdef CFG_DEBUG
  UT_ASSERT(m_ILFs2.empty() && m_IKFs2.empty());
#endif
  const int iFrm = m_ic2LF.empty() ? MT_TASK_NONE : m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_LBA_SynchronizeData);
  m_ILFs1.swap(m_ILFs2);
  m_IKFs1.swap(m_IKFs2);
  //m_ILFs1.resize(m_ILFs2.size());
  //m_IKFs1.resize(m_IKFs2.size());
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_LBA_SynchronizeData);
  m_UcsLF.assign(m_LFs.size(), LM_FLAG_FRAME_DEFAULT);
  m_UcsKF.assign(m_KFs.size(), LM_FLAG_FRAME_DEFAULT);
  m_Uds.assign(m_ds.size(), LM_FLAG_TRACK_DEFAULT);
#ifdef CFG_GROUND_TRUTH
  if (m_history >= 3) {
    m_ucsKFGT.assign(m_KFs.size(), LBA_FLAG_FRAME_DEFAULT);
    m_udsGT.assign(m_ds.size(), LBA_FLAG_TRACK_DEFAULT);
  }
#endif
  for (std::list<InputLocalFrame>::iterator ILF = m_ILFs2.begin(); ILF != m_ILFs2.end(); ++ILF) {
    if (ILF->m_C.Valid()) {
      continue;
    }
    Camera C;
    if (m_LFs.empty()) {
      IMU::InitializeCamera(ILF->m_us, C);
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
//#if 0
      if (m_CsGT) {
        const LA::AlignedVector3f g = m_CsGT[ILF->m_T.m_iFrm].m_T.GetGravity();
        C.MakeIdentity(&g);
      }
#endif
    } else {
      const int iLF = m_ic2LF.back();
      if (LBA_PROPAGATE_CAMERA) {
        IMU::Delta D;
        const LocalFrame &LF = m_LFs[iLF];
        IMU::PreIntegrate(ILF->m_us, LF.m_T.m_t, ILF->m_T.m_t, m_CsLF[iLF], &D, &m_work,
                          false, &LF.m_us.Back());
        IMU::Propagate(m_K.m_pu, D, m_CsLF[iLF], C);
      } else {
        C = m_CsLF[iLF];
      }
//#ifdef CFG_DEBUG
#if 0
      const LA::AlignedVector3f &bw = m_CsLF[iLF].m_bw;
      UT::Print("%.10e %.10e %.10e\n", bw.x(), bw.y(), bw.z());
#endif
    }
    ILF->m_C = C;
  }
#ifdef CFG_STEREO
  LA::SymmetricMatrix2x2f Wr;
  const float w = 1.0f;
  //const float w = BA_WEIGHT_FEATURE;
  //////////////////////////////////////////////////////////////////////////
  const bool robust = false;
  //const bool robust = true;
  //////////////////////////////////////////////////////////////////////////
#endif
  const int nKFsBkp = m_CsKF.Size();
  for (std::list<GlobalMap::InputKeyFrame>::iterator IKF = m_IKFs2.begin();
       IKF != m_IKFs2.end(); ++IKF) {
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
    const std::vector<Depth::InverseGaussian> &dsGT = m_solver->m_internal->m_dsGT;
    if (m_CsGT && !dsGT.empty()) {
      const Depth::InverseGaussian *ds = dsGT.data() + IKF->m_id;
      const int Nx = static_cast<int>(IKF->m_xs.size());
      for (int ix = 0; ix < Nx; ++ix) {
        FTR::Source &x = IKF->m_xs[ix];
        if (x.m_xr.Valid()) {
          FTR::DebugSetMeasurement(m_K.m_br, ds[ix], x);
        }
      }
      DebugSetFeatureMeasurements(m_CsGT[IKF->m_T.m_iFrm].m_T, m_CsKFGT, dsGT, *IKF);
    }
#endif
    LA::Vector3f Rx;
    LA::SymmetricMatrix2x2f W;
    const int NX = static_cast<int>(IKF->m_Xs.size());
#ifdef CFG_DEBUG
    for (int iX = 0; iX < NX; ++iX) {
      const GlobalMap::Point &X = IKF->m_Xs[iX];
      if (iX > 0) {
        UT_ASSERT(X.m_iKF >= IKF->m_Xs[iX - 1].m_iKF);
      }
      X.AssertConsistency();
    }
#endif
    m_CsKF.Push(IKF->m_C.m_T);
    const int nKFs = m_CsKF.Size();
    m_R12s.Resize(nKFs);
#ifdef CFG_STEREO
    m_t12s.Resize(nKFs << 1);
#else
    m_t12s.Resize(nKFs);
#endif
    for (int iX1 = 0, iX2 = 0; iX1 < NX; iX1 = iX2) {
      const int iKF = IKF->m_Xs[iX1].m_iKF;
      const Rigid3D &C = m_CsKF[iKF];
      m_marksTmp.assign(nKFs, 0);
      for (iX2 = iX1 + 1; iX2 < NX && IKF->m_Xs[iX2].m_iKF == iKF; ++iX2) {}
      for (int iX = iX1; iX < iX2; ++iX) {
//#ifdef CFG_DEBUG
#if 0
        if (iX == 17) {
          UT::DebugStart();
          UT::DebugStop();
        }
#endif
        GlobalMap::Point &X = IKF->m_Xs[iX];
        m_zds.resize(0);
#ifdef CFG_STEREO
        if (X.m_x.m_xr.Valid()) {
          Rx.Set(X.m_x.m_x.x(), X.m_x.m_x.y(), 1.0f);
          X.m_x.m_Wr.GetScaled(w, W);
          m_zds.push_back(Depth::Measurement(m_K.m_br, Rx, X.m_x.m_xr, W));
        }
#endif
        const int Nz = static_cast<int>(X.m_zs.size());
        for (int i = 0; i < Nz; ++i) {
          const FTR::Measurement &z = X.m_zs[i];
          Rotation3D &R = m_R12s[z.m_iKF];
#ifdef CFG_STEREO
          LA::AlignedVector3f *t = m_t12s.Data() + (z.m_iKF << 1);
#else
          LA::AlignedVector3f *t = m_t12s.Data() + z.m_iKF;
#endif
          if (!m_marksTmp[z.m_iKF]) {
            const Rigid3D &_C = m_CsKF[z.m_iKF];
            if (_C.Valid()) {
              const Rigid3D T = _C / C;
              R = T;
              T.GetTranslation(*t);
#ifdef CFG_STEREO
              LA::AlignedVector3f::apb(t[0], m_K.m_br, t[1]);
#endif
            } else {
              R.Invalidate();
              t->Invalidate();
#ifdef CFG_STEREO
              t[1].Invalidate();
#endif
            }
            m_marksTmp[z.m_iKF] = 1;
          }
          if (R.Valid()) {
            R.Apply(X.m_x.m_x, Rx);
#ifdef CFG_STEREO
            if (z.m_z.Valid()) {
              z.m_W.GetScaled(w, W);
              m_zds.push_back(Depth::Measurement(t[0], Rx, z.m_z, W));
            }
            if (z.m_zr.Valid()) {
              z.m_Wr.GetScaled(w, W);
              m_zds.push_back(Depth::Measurement(t[1], Rx, z.m_zr, W));
            }
#else
            z.m_W.GetScaled(w, W);
            m_zds.push_back(Depth::Measurement(*t, Rx, z.m_z, W));
#endif
          }
        }
        if (!Depth::Triangulate(w, static_cast<int>(m_zds.size()), m_zds.data(),
                                &X.m_d, &m_work, X.m_d.Valid(), robust)) {
          X.m_d.Invalidate();
        }
#if 0
//#ifdef CFG_DEBUG
        if (X.m_d.Valid()) {
          X.m_d.Print(UT::String("%d: ", iX), true, true);
        }
#endif
      }
    }
  }
  m_CsKF.Resize(nKFsBkp);
  while (!m_ILFs2.empty() || !m_IKFs2.empty()) {
//#ifdef LBA_DEBUG_EIGEN
#if 0
    DebugGenerateTracks();
#endif
    if (m_IKFs2.empty() ||
        (!m_ILFs2.empty() && m_ILFs2.front().FRM::Frame::m_T <= m_IKFs2.front().FRM::Frame::m_T)) {
      InputLocalFrame &ILF = m_ILFs2.front();
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
      const std::vector<Depth::InverseGaussian> &dsGT = m_solver->m_internal->m_dsGT;
      if (m_CsGT && !dsGT.empty()) {
        DebugSetFeatureMeasurements(m_CsGT[ILF.m_T.m_iFrm].m_T, m_CsKFGT, dsGT, ILF);
      }
#endif
      if (ILF.m_d.Invalid()) {
        if (m_LFs.empty()) {
          ILF.m_d.Initialize();
        } else {
          const LocalFrame &LF = m_LFs[m_ic2LF.back()];
          ILF.m_d = LF.m_d;
          ILF.m_d.Propagate(ILF.m_T.m_t - LF.m_T.m_t);
        }
        const int Nz = static_cast<int>(ILF.m_zs.size()), NzC = SIMD_FLOAT_CEIL(Nz);
        m_work.Resize(NzC * 3);
        LA::AlignedVectorXf us(m_work.Data(), Nz, false);     us.Resize(0);
        LA::AlignedVectorXf ws(us.Data() + NzC, Nz, false);   ws.Resize(0);
        LA::AlignedVectorXf wus(ws.Data() + NzC, Nz, false);
        const int nKFs = static_cast<int>(m_KFs.size());
        if (!m_IKFs2.empty() && m_IKFs2.front().m_T.m_iFrm == ILF.m_T.m_iFrm) {
          const GlobalMap::InputKeyFrame &IKF = m_IKFs2.front();
          const int NX = static_cast<int>(IKF.m_Xs.size()), N = NX + Nz, NC = SIMD_FLOAT_CEIL(N);
          m_work.Resize(NC * 3);
          us.Bind(m_work.Data(), N);     us.Resize(0);
          ws.Bind(us.Data() + NC, N);    ws.Resize(0);
          wus.Bind(ws.Data() + NC, N);
          for (int iX = 0; iX < NX; ++iX) {
            const GlobalMap::Point &X = IKF.m_Xs[iX];
            if (X.m_iKF != nKFs || !X.m_d.Valid()) {
              continue;
            }
            us.Push(X.m_d.u());
            ws.Push(DEPTH_VARIANCE_EPSILON + X.m_d.s2());
          }
        }
        Depth::InverseGaussian dz;
        Rigid3D::Row Crz;
        const Rigid3D C = ILF.m_C.m_T;
        const Rigid3D::Row Cz = C.GetRowZ();
        const int NZ = static_cast<int>(ILF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
          const FRM::Measurement &Z = ILF.m_Zs[iZ];
          if (Z.m_iKF == nKFs) {
            continue;
          }
          const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[Z.m_iKF];
          Rigid3D::ABI(Cz, m_CsKF[Z.m_iKF], Crz);
          const GlobalMap::KeyFrame &KF = m_KFs[Z.m_iKF];
          for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
            const int ix = ILF.m_zs[iz].m_ix;
            const Depth::InverseGaussian &d = ds[ix];
            if (!d.Valid() || !d.ProjectD(Crz, KF.m_xs[ix].m_x, dz)) {
              continue;
            }
            us.Push(dz.u());
            ws.Push(DEPTH_VARIANCE_EPSILON + dz.s2());
          }
        }
        ws.MakeInverse();
        const float Sw = ws.Sum();
        if (Sw < FLT_EPSILON) {
          ws.Set(1.0f / Nz);
        } else {
          ws *= 1.0f / Sw;
        }
        LA::AlignedVectorXf::AB(ws, us, wus);
        const float u = wus.Sum();
        us -= u;
        us.MakeSquared();
        LA::AlignedVectorXf::AB(ws, us, wus);
        const float s2 = wus.Sum();
        if (s2 > 0.0f) {
          ILF.m_d.Update(Depth::InverseGaussian(u, s2));
        }
      }
      //if (!ILF.m_Zs.empty() && ILF.m_iKFsMatch.empty()) {
        SearchMatchingKeyFrames(ILF);
      //}
      if (ILF.m_iKFNearest == -1 && !ILF.m_iKFsMatch.empty()) {
        if (LBA_MARGINALIZATION_REFERENCE_NEAREST) {
          ubyte first = 1;
          int iKFNearest = m_Zp.m_iKFr;
          float imgMotionNearest = FLT_MAX;
          const Rigid3D C = ILF.m_C.m_T;
          const float z = 1.0f / ILF.m_d.u();
          const int nKFs1 = static_cast<int>(m_KFs.size());
          const int nKFs2 = ILF.m_Zs.back().m_iKF < nKFs1 ? nKFs1 : nKFs1 + 1;
          m_marksTmp.assign(nKFs2, 0);
          const int nKFsMatch = static_cast<int>(ILF.m_iKFsMatch.size());
          for (int i = 0; i < nKFsMatch; ++i) {
            m_marksTmp[ILF.m_iKFsMatch[i]] = 1;
          }
          for (int iKF = 0; iKF < nKFs2; ++iKF) {
            if (!m_marksTmp[iKF] || iKF == nKFs1) {
              continue;
            }
            const Rigid3D _C = m_CsKF[iKF];
            const float imgMotion = ComputeImageMotion(z, C, _C, &first);
            if (imgMotion > imgMotionNearest) {
              continue;
            }
            imgMotionNearest = imgMotion;
            iKFNearest = iKF;
          }
          ILF.m_iKFNearest = iKFNearest;
        } else {
          ILF.m_iKFNearest = static_cast<int>(m_KFs.size()) - 1;
        }
      } else if (ILF.m_iKFNearest == -1 && ILF.m_iKFsMatch.empty()) {
        ILF.m_iKFNearest = m_Zp.m_iKFr;
      }
      PushLocalFrame(ILF);
      m_ILFs2.pop_front();
    } else {
      GlobalMap::InputKeyFrame &IKF = m_IKFs2.front();
      const bool v1 = IKF.m_C.m_T.Valid(), v2 = IKF.m_C.m_v.Valid();
      if (!v1 || !v2) {
        const int nLFs = static_cast<int>(m_LFs.size());
        for (int ic = nLFs - 1; ic >= 0; --ic) {
          const int iLF = m_ic2LF[ic];
          if (m_LFs[iLF].m_T != IKF.m_T) {
            continue;
          }
          const Camera &C = m_CsLF[iLF];
          if (!v1) {
            IKF.m_C.m_T = C.m_T;
            IKF.m_C.m_p = C.m_p;
          }
          if (!v2) {
            IKF.m_C.m_v = C.m_v;
            IKF.m_C.m_ba = C.m_ba;
            IKF.m_C.m_bw = C.m_bw;
          }
          break;
        }
      }
      if (IKF.m_d.Invalid()) {
        const int nLFs = static_cast<int>(m_LFs.size());
        for (int iLF = 0; iLF < nLFs; ++iLF) {
          const LocalFrame &LF = m_LFs[iLF];
          if (LF.m_T != IKF.m_T) {
            continue;
          }
          IKF.m_d = LF.m_d;
          break;
        }
      }
      //if (!IKF.m_Zs.empty() && IKF.m_iKFsMatch.empty()) {
        SearchMatchingKeyFrames(IKF);
      //}
      const int nKFs = static_cast<int>(m_KFs.size());
      const int NX = static_cast<int>(IKF.m_Xs.size());
      for (int iX = 0; iX < NX; ++iX) {
        GlobalMap::Point &X = IKF.m_Xs[iX];
        if (X.m_d.Invalid()) {
          X.m_d.Initialize(X.m_iKF == nKFs ? IKF.m_d.u() : m_KFs[X.m_iKF].m_d.u());
          //////////////////////////////////////////////////////////////////////////
          //d.Initialize();
          //////////////////////////////////////////////////////////////////////////
        }
      }
      PushKeyFrame(IKF);
      m_IKFs2.pop_front();
    }
  }
  if (m_CsKF.Size() > nKFsBkp) {
    m_ts[TM_TOTAL].Stop();
    m_GBA->WakeUp();
    m_ts[TM_TOTAL].Start();
  }
#if 0
//#if 1
  m_IT->RunView();
#endif
  if (!m_KFs.empty() && m_GM->LBA_Synchronize(m_KFs.back().m_T.m_iFrm, m_CsKFBkp, m_marksTmp)) {
    Rigid3D TI;
    int iKFNearest = -1;
#ifdef CFG_DEBUG
    UT_ASSERT(m_CsKFBkp.Size() == m_CsKF.Size() && m_marksTmp.size() == m_ucsKF.size());
#endif
    m_CsKF.Swap(m_CsKFBkp);
    const ubyte ucmFlag = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                          LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY;
    const int nLFs = static_cast<int>(m_LFs.size());
    for (int ic = 0; ic < nLFs; ++ic) {
      const int iLF = m_ic2LF[ic];
      const int _iKFNearest = m_LFs[iLF].m_iKFNearest;
      if (/*_iKFNearest == -1 || */!(m_marksTmp[_iKFNearest] & GM_FLAG_FRAME_UPDATE_CAMERA)) {
        continue;
      }
      if (_iKFNearest != iKFNearest) {
        iKFNearest = _iKFNearest;
        TI = m_CsKF[iKFNearest].GetInverse() * m_CsKFBkp[iKFNearest];
      }
      Camera &C = m_CsLF[iLF];
//#ifdef CFG_DEBUG
#if 0
      if (iLF == m_ic2LF.back()) {
        UT::PrintSeparator();
        TI.Print(true);
        UT::PrintSeparator();
        C.Print(true);
      }
#endif
      C.m_T = C.m_T / TI;
      C.m_T.GetPosition(C.m_p);
      C.m_v = TI.GetAppliedRotation(C.m_v);
//#ifdef CFG_DEBUG
#if 0
      if (iLF == m_ic2LF.back()) {
        UT::PrintSeparator();
        C.Print(true);
      }
#endif
      m_ucsLF[iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
      m_ucmsLF[iLF] |= ucmFlag;
      m_UcsLF[iLF] = LM_FLAG_FRAME_UPDATE_CAMERA_LF;
    }
#ifdef CFG_DEBUG
    UT_ASSERT(GM_FLAG_FRAME_DEFAULT == LM_FLAG_FRAME_DEFAULT);
#endif
    const bool ud = LBA_RESET_DEPTH_INFORMATION;
    const ubyte ucFlag = LBA_FLAG_FRAME_UPDATE_CAMERA |
                         (ud ? LBA_FLAG_FRAME_UPDATE_DEPTH : LBA_FLAG_FRAME_DEFAULT);
    const int nKFs = static_cast<int>(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const ubyte uc = m_marksTmp[iKF];
#ifdef CFG_DEBUG
      UT_ASSERT(uc == GM_FLAG_FRAME_DEFAULT || uc == GM_FLAG_FRAME_UPDATE_CAMERA);
#endif
      if (!uc) {
        continue;
      }
      m_ucsKF[iKF] |= ucFlag;
      m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_CAMERA_KF;
      if (!ud) {
        continue;
      }
      const KeyFrame &KF = m_KFs[iKF];
      const int NZ = static_cast<int>(KF.m_Zs.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const FRM::Measurement &Z = KF.m_Zs[iZ];
        m_ucsKF[Z.m_iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
        ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          uds[KF.m_zs[iz].m_ix] |= LBA_FLAG_TRACK_UPDATE_DEPTH;
        }
      }
    }
  }
//#ifdef CFG_DEBUG
#if 0
  if (m_KFs.size() == 2) {
    UT::PrintSeparator();
    m_CsKF[0].Print(true);
    UT::PrintSeparator();
    m_CsKF[1].Print(true);
  }
#endif
}

void LocalBundleAdjustor::UpdateData() {
  const int iFrm1 = m_LFs[m_ic2LF.front()].m_T.m_iFrm;
  const int iFrm2 = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  m_LM->LBA_Update(iFrm1, iFrm2, m_ic2LF, m_CsLF, m_UcsLF, m_CsKF, m_UcsKF, m_iKF2d, m_ds, m_Uds);
#ifdef CFG_VERBOSE
  if (m_verbose >= 2) {
    const int Ncu = UT::VectorCountFlag<ubyte>(m_UcsLF, LM_FLAG_FRAME_UPDATE_CAMERA_LF);
    const int Nc = static_cast<int>(m_LFs.size());
    const int Nku = UT::VectorCountFlag<ubyte>(m_UcsKF, LM_FLAG_FRAME_UPDATE_CAMERA_KF);
    const int Nk = static_cast<int>(m_KFs.size());
    const int Ndu = UT::VectorCountFlag<ubyte>(m_Uds, LM_FLAG_TRACK_UPDATE_DEPTH);
    const int Nd = static_cast<int>(m_ds.size());
    UT::PrintSeparator();
    UT::Print("[%d] [LocalBundleAdjustor::UpdateData]\n", iFrm2);
    UT::Print("  CameraLF = %d / %d = %.2f%%\n", Ncu, Nc, UT::Percentage(Ncu, Nc));
    UT::Print("  CameraKF = %d / %d = %.2f%%\n", Nku, Nk, UT::Percentage(Nku, Nk));
    UT::Print("  Depth    = %d / %d = %.2f%%\n", Ndu, Nd, UT::Percentage(Ndu, Nd));
  }
#endif
  const int iLF = m_ic2LF.back(), iFrm = m_LFs[iLF].m_T.m_iFrm;
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_LBA_UpdateData);
  m_C.m_T = m_LFs[iLF].m_T;
  m_C.m_C = m_CsLF[iLF];
  //m_ILFs1.resize(0);
  //m_IKFs1.resize(0);
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_LBA_UpdateData);
}

bool LocalBundleAdjustor::BufferDataEmpty() {
  return m_ILFs1.empty() && m_IKFs1.empty();
}

void LocalBundleAdjustor::MarginalizeLocalFrame() {
  if (static_cast<int>(m_LFs.size()) < LBA_MAX_LOCAL_FRAMES) {
    return;
  }
  //const float sx = 1.0f;
  //const float sx = 1.0e3f;
  //const float sd = 1.0f;
  //const float sd = 1.0e3f;
//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator();
  //m_Zp.Print();
  m_Zp.PrintDiagonal();
#endif
  const float eps = 0.0f;
  const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
  const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
  const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
  const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
  const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
  const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                        epsba, epsba, epsba, epsbw, epsbw, epsbw};
  const int iLF1 = m_ic2LF[0], iLF2 = m_ic2LF[1];
  const LocalFrame &LF1 = m_LFs[iLF1];
  const int iKFr = LF1.m_iKFNearest == -1 ? m_Zp.m_iKFr : LF1.m_iKFNearest;
//#ifdef CFG_DEBUG
#if 0
  UT_ASSERT(m_KFs[iKFr].m_T.m_iFrm <= LF1.m_T.m_iFrm);
#endif
  const bool newKF = LF1.m_T.m_iFrm == m_KFs[iKFr].m_T.m_iFrm;
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  if (newKF) {
    if (m_Zp.Pose::Valid()) {
      m_ts[TM_TEST_2].Start();
      if (m_Zp.GetPriorMotion(&m_ZpLF, &m_work, _eps) ||
          !LBA_MARGINALIZATION_CHECK_INVERTIBLE) {
        //m_GBA->PushCameraPriorMotion(iFrm, m_Zp.m_iKFr, m_ZpLF);
        m_GBA->PushCameraPriorMotion(iFrm, iKFr, m_ZpLF);
      } else {
        //m_ZpLF.MakeZero();
        m_ZpLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY,
                          BA_VARIANCE_PRIOR_BIAS_ACCELERATION,
                          BA_VARIANCE_PRIOR_BIAS_GYROSCOPE, &m_CsLF[iLF1]);
      }
      m_ts[TM_TEST_2].Stop();
      if (m_Zp.GetPriorPose(iKFr, &m_ZpKF, &m_work, _eps) ||
          !LBA_MARGINALIZATION_CHECK_INVERTIBLE) {
        m_GBA->PushCameraPriorPose(iFrm, m_ZpKF);
        //////////////////////////////////////////////////////////////////////////
        //m_GBA->WakeUp();
        //////////////////////////////////////////////////////////////////////////
      } else {
        m_ZpKF.Invalidate();
      }
    }
    const Rigid3D &Tr = m_CsKF[iKFr];
    const float s2r = m_Zp.Pose::Valid() ? BA_VARIANCE_PRIOR_ROTATION : BA_VARIANCE_PRIOR_ROTATION_INITIAL;
    m_Zp.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, iKFr, Tr, s2r, m_ZpLF, true);
    
    Camera C1;
    C1.m_T = Tr;
    Tr.GetPosition(C1.m_p);
    m_Zp.GetMotion(Tr, &C1.m_v, &C1.m_ba, &C1.m_bw);

    IMU::Delta::Error e;
    IMU::Delta::Jacobian::RelativeKF J;
    IMU::Delta::Factor::Auxiliary::RelativeKF A;
    const Camera &C2 = m_CsLF[iLF2];
//#ifdef CFG_DEBUG
#if 0
    C1.Print();
    C2.Print();
#endif
    m_DsLF[iLF2].GetFactor(BA_WEIGHT_IMU/* * sd*/, C1, C2, m_K.m_pu, &e, &J, &A);
#if 0
//#ifdef CFG_DEBUG
    for (int i = 0; i < 8; ++i) {
      A.m_bc[i].MakeZero();
    }
    A.m_bg.MakeZero();
#endif
    m_Zp.PropagateKF(Tr, C2, A, &m_work);
//#ifdef CFG_DEBUG
#if 0
    const Point3D pr2 = m_Zp.m_Zps[0].GetInverse().GetPosition();
    const Point3D pr2GT = (m_CsLFGT[iLF2].m_T / m_CsKFGT[iKFr]).GetPosition();
    UT::PrintSeparator();
    pr2.Print();
    pr2GT.Print();
#endif
  } else {
    Camera C1;
    Rigid3D Tr, TrI, Tr1, Trk, Tk1[2];
    /*const */int Nk = static_cast<int>(m_Zp.m_iKFs.size()) - 1;
#ifdef CFG_DEBUG
    UT_ASSERT(m_Zp.m_iKFs[Nk] == INT_MAX);
#endif
    const Rigid3D &Cr = m_CsKF[m_Zp.m_iKFr];
    m_Zp.GetReferencePose(Cr, &Tr, &TrI);
    m_Zp.GetPose(TrI, Nk, &Tr1, &C1.m_T);
    const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
    const float epsc[12] = {epsp, epsp, epsp, epsr, epsr, epsr,
                            epsp, epsp, epsp, epsr, epsr, epsr};
    const int NZ = static_cast<int>(LF1.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF1.m_Zs[iZ];
      const int iKF = Z.m_iKF;
      const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
      const KeyFrame &KF = m_KFs[iKF];
      if (iKF == m_Zp.m_iKFr) {
        Tk1[0] = Tr1;
#ifdef CFG_STEREO
        Tk1[1] = Tk1[0];
        Tk1[1].SetTranslation(m_K.m_br + Tk1[0].GetTranslation());
#endif
        FTR::Factor::FixSource::L L;
        FTR::Factor::FixSource::A1 A1;
        FTR::Factor::FixSource::A2 A2;
        FTR::Factor::FixSource::U U;
        FTR::Factor::FixSource::M2 M;
        FTR::Factor::DD mdd;
        LA::AlignedVector6f mdcz;
        Camera::Factor::Unitary::CC SAczz;
        SAczz.MakeZero();
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          const FTR::Measurement &z = LF1.m_zs[iz];
          const int ix = z.m_ix;
          FTR::GetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE/* * sx*/, BA_WEIGHT_PRIOR_DEPTH, Tk1, KF.m_xs[ix],
                                          ds[ix], Tr1, z, &L, &A1, &A2, &U
#ifdef CFG_STEREO
                                        , m_K.m_br
#endif
                                        );
          SAczz += A2.m_Aczz;
          if (A2.m_add.m_a < epsd) {
            continue;
          }
          mdd.m_a = 1.0f / A2.m_add.m_a;
          mdd.m_b = mdd.m_a * A2.m_add.m_b;
          FTR::Factor::FixSource::Marginalize(mdd, A1.m_adczA, &mdcz, &M);
          SAczz -= M.m_Mczz;
        }
        m_Zp.Update(Nk, SAczz);

        if (m_history >= 2) {
          History &hist = m_hists.back();
          LA::AlignedVector6f b;
          SAczz.m_b.Get(b);
          hist.m_PR.m_r2xr = b.SquaredLength();
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            LA::AlignedMatrix6x6f A;
            LA::AlignedVector6f e, r;
            CameraPrior::Block::EC ez;
            SAczz.m_A.GetAlignedMatrix6x6f(A);
            m_Zp.Pose::GetError(m_CsKFGT[m_Zp.m_iKFr], m_CsLFGT[iLF1].m_T, Nk, &ez);
            e.Set(ez.m_ep, ez.m_er);
            LA::AlignedMatrix6x6f::Ab(A, e, (float *) &r);
            r += b;
            hist.m_PRGT.m_r2xr += r.SquaredLength();
          }
#endif
        }
      } else {
        const std::vector<int>::const_iterator it = std::lower_bound(m_Zp.m_iKFs.begin(),
                                                                     m_Zp.m_iKFs.end(), iKF);
        const int ik = static_cast<int>(it - m_Zp.m_iKFs.begin());
        if (it == m_Zp.m_iKFs.end() || *it != iKF) {
          const Rigid3D &Ck = m_CsKF[iKF];
          Rigid3D::ABI(Ck, Tr, Trk);
        } else {
          m_Zp.m_Zps[ik].GetInverse(Trk);
        }
        Rigid3D::ABI(Tr1, Trk, Tk1[0]);
#ifdef CFG_STEREO
        Tk1[1] = Tk1[0];
        Tk1[1].SetTranslation(m_K.m_br + Tk1[0].GetTranslation());
#endif
        xp128f mdd;
        FTR::Factor::Full::L L;
        FTR::Factor::Full::A1 A1;
        FTR::Factor::Full::A2 A2;
        FTR::Factor::Full::U U;
        FTR::Factor::Full::Source::M1 M1;
        FTR::Factor::Full::Source::M2 M2;
        FTR::Factor::Full::M1 M3;
        FTR::Factor::Full::M2 M4;
        LA::ProductVector6f adcz;
        Camera::Factor::Unitary::CC SAcxx, SAczz;
        Camera::Factor::Binary::CC SAcxz;
        SAcxx.MakeZero();
        SAcxz.MakeZero();
        SAczz.MakeZero();
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          const FTR::Measurement &z = LF1.m_zs[iz];
          const int ix = z.m_ix;
          FTR::GetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE/* * sx*/, BA_WEIGHT_PRIOR_DEPTH, Tk1, KF.m_xs[ix],
                                          ds[ix], Tr1, z, &L, &A1, &A2, &U
#ifdef CFG_STEREO
                                        , m_K.m_br
#endif
                                        );
          SAcxx += A2.m_Acxx;
          SAcxz += A2.m_Acxz;
          SAczz += A2.m_Aczz;
          if (A2.m_adx.m_add.m_a < epsd) {
            continue;
          }
          mdd.vdup_all_lane(1.0f / A2.m_adx.m_add.m_a);
          FTR::Factor::Full::Source::Marginalize(mdd, A2.m_adx, &M1, &M2);
          FTR::Factor::Full::Marginalize(mdd, M1, A1, &M3, &M4, &adcz);
          SAcxx -= M2.m_Mcxx;
          SAcxz -= M4.m_Mcxz;
          SAczz -= M4.m_Mczz;
        }
        if (it == m_Zp.m_iKFs.end() || *it != iKF) {
          LA::AlignedMatrix12x12f A;
          A.Set(SAcxx.m_A, SAcxz, SAczz.m_A);
          if (LBA_MARGINALIZATION_CHECK_RANK) {
            //if (!A.DecomposeLDL(epsc)) {
            //  continue;
            //}
            const int rank = A.RankLDL(epsc);
            //UT::Print("[%d] --> [%d] %d\n", m_KFs[Z.m_iKF].m_T.m_iFrm, LF1.m_T.m_iFrm, rank);
            //if (rank < 6) {
            //  continue;
            //}
          }
          m_Zp.Insert(BA_WEIGHT_PRIOR_CAMERA_INITIAL, ik, iKF, Trk,
                      BA_VARIANCE_PRIOR_POSITION, BA_VARIANCE_PRIOR_ROTATION, &m_work);
          ++Nk;
        }
        m_Zp.Update(ik, Nk, SAcxx, SAcxz, SAczz);
        if (m_history >= 2) {
          History &hist = m_hists.back();
          LA::AlignedVector12f b;
          b.Set(SAcxx.m_b, SAczz.m_b);
          hist.m_PR.m_r2xk += b.SquaredLength();
#ifdef CFG_GROUND_TRUTH
          if (m_CsGT) {
            LA::AlignedMatrix12x12f A;
            LA::AlignedVector12f e, r;
            CameraPrior::Block::EC ex, ez;
            A.Set(SAcxx.m_A, SAcxz, SAczz.m_A);
            m_Zp.Pose::GetError(m_CsKFGT[m_Zp.m_iKFr], m_CsKFGT[Z.m_iKF], ik, &ex);
            m_Zp.Pose::GetError(m_CsKFGT[m_Zp.m_iKFr], m_CsLFGT[iLF1].m_T, Nk, &ez);
            e.Set(ex.m_ep, ex.m_er, ez.m_ep, ez.m_er);
            LA::AlignedMatrix12x12f::Ab(A, e, r);
            r += b;
            hist.m_PRGT.m_r2xk += r.SquaredLength();
          }
#endif
        }
      }
    }
    if (iKFr == m_Zp.m_iKFr) {
      C1.m_T.GetPosition(C1.m_p);
    } else {
      Tr = m_CsKF[iKFr];
      C1 = m_CsLF[iLF1];
      m_ts[TM_TEST_2].Start();
      if (!m_Zp.GetPriorMotion(&m_ZpLF, &m_work, _eps) && LBA_MARGINALIZATION_CHECK_INVERTIBLE) {
        m_ZpLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY,
                          BA_VARIANCE_PRIOR_BIAS_ACCELERATION,
                          BA_VARIANCE_PRIOR_BIAS_GYROSCOPE, &C1);
      }
      m_ts[TM_TEST_2].Stop();
      if (m_Zp.GetPriorPose(INT_MAX, &m_ZpKF, &m_work, _eps) ||
          !LBA_MARGINALIZATION_CHECK_INVERTIBLE) {
        m_GBA->PushCameraPriorPose(iFrm, m_ZpKF);
        //////////////////////////////////////////////////////////////////////////
        //m_GBA->WakeUp();
        //////////////////////////////////////////////////////////////////////////
      } else {
        m_ZpKF.Invalidate();
      }
      m_Zp.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, iKFr, Tr, BA_VARIANCE_PRIOR_ROTATION, m_ZpLF, false, &C1.m_T);
    }
    m_Zp.GetMotion(C1.m_T, &C1.m_v, &C1.m_ba, &C1.m_bw);
//#ifdef CFG_DEBUG
#if 0
    {
      const Camera &_C1 = m_CsLF[iLF1];
      const Rotation3D &R1 = C1.m_T, &_R1 = _C1.m_T;
      UT::PrintSeparator();
      R1.Print();
      UT::PrintSeparator();
      _R1.Print();
      UT::PrintSeparator();
      ((R1 / _R1).GetRodrigues() * UT_FACTOR_RAD_TO_DEG).Print();
      (C1.m_p - _C1.m_p).Print();
      (C1.m_v - _C1.m_v).Print();
      (C1.m_ba - _C1.m_ba).Print();
      (C1.m_bw - _C1.m_bw).Print();
    }
#endif

    IMU::Delta::Error e;
    IMU::Delta::Jacobian::RelativeLF J;
    IMU::Delta::Factor::Auxiliary::RelativeLF A;
    const Camera &C2 = m_CsLF[iLF2];
    m_DsLF[iLF2].GetFactor(BA_WEIGHT_IMU/* * sd*/, C1, C2, m_K.m_pu, Tr, &e, &J, &A);
    m_Zp.PropagateLF(Tr, C2, A, &m_work);
//#ifdef CFG_DEBUG
#if 0
    const Point3D pr2 = m_Zp.m_Zps[Nk - 1].GetInverse().GetPosition();
    const Point3D pr2GT = (m_CsLFGT[iLF2].m_T / m_CsKFGT[iKFr]).GetPosition();
    UT::PrintSeparator();
    pr2.Print();
    pr2GT.Print();
#endif
  }
  m_ts[TM_TEST_2].Start();
  if (LBA_MARGINALIZATION_PARTIAL) {
    if (!m_Zp.GetPriorMotion(&m_ZpLF, &m_work) && LBA_MARGINALIZATION_CHECK_INVERTIBLE) {
      m_ZpLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY,
                        BA_VARIANCE_PRIOR_BIAS_ACCELERATION,
                        BA_VARIANCE_PRIOR_BIAS_GYROSCOPE, &m_CsLF[iLF2]);
    }
  } else {
    m_ZpLF = m_Zp;
  }
  m_ts[TM_TEST_2].Stop();
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  CameraPrior::Block::MM Amm = m_ZpLF.m_Amm;
  CameraPrior::Block::M bm = m_ZpLF.m_bm;
  if (Amm.SolveLDL(bm, _eps + 6)) {
    m_ZpLF.m_em.Set(bm);
  } else {
    m_ZpLF.m_Amm.MakeZero();
    m_ZpLF.m_em.MakeZero();
  }
#endif
  //////////////////////////////////////////////////////////////////////////
  m_ApLF.MakeZero();
  m_ucmsLF[iLF2] |= LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                    LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                    LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
//#ifdef CFG_DEBUG
#if 0
  UT::Print("[%d] %d\n", m_LFs[m_ic2LF.back()].m_T.m_iFrm, m_Zp.m_iKFr);
#endif
}

void LocalBundleAdjustor::PushLocalFrame(const InputLocalFrame &ILF) {
#if 0
//#if 1
  if (!m_KFs.empty()) {
    const int iKF = 0;
    const KeyFrame &KF = m_KFs[iKF];
    if (!KF.m_xs.empty()) {
      UT::PrintSeparator('*');
      const int ix = 206;
      const int iST1 = KF.m_ix2ST[ix], iST2 = KF.m_ix2ST[ix + 1];
      for (int iST = iST1; iST < iST2; ++iST) {
        const KeyFrame::SlidingTrack &ST = KF.m_STs[iST];
        const int iFrmMin = m_LFs[m_ic2LF[ST.m_icMin]].m_T.m_iFrm;
        const int iFrmMax = m_LFs[m_ic2LF[ST.m_icMax]].m_T.m_iFrm;
        UT::Print("[%d, %d]\n", iFrmMin, iFrmMax);
      }
    }
  }
#endif
  const int nLFs1 = static_cast<int>(m_LFs.size());
  if (static_cast<int>(m_LFs.size()) < LBA_MAX_LOCAL_FRAMES) {
    const int nLFs2 = nLFs1 + 1;
    m_ic2LF.push_back(nLFs1);
    m_LFs.resize(nLFs2);
    m_CsLF.Resize(nLFs2, true);
#ifdef CFG_GROUND_TRUTH
    if (m_CsGT) {
      m_CsLFGT.Resize(nLFs2, true);
    }
#endif
    m_ucsLF.resize(nLFs2);
    m_ucmsLF.resize(nLFs2);
#ifdef CFG_INCREMENTAL_PCG
    m_xcsLF.Resize(nLFs2, true);
    m_xmsLF.Resize(nLFs2, true);
#endif
    m_DsLF.Resize(nLFs2, true);
#ifdef CFG_GROUND_TRUTH
    if (m_CsGT) {
      m_DsLFGT.Resize(nLFs2, true);
    }
#endif
    m_AdsLF.Resize(nLFs2, true);
    m_AfpsLF.Resize(nLFs2, true);
    m_AfmsLF.Resize(nLFs2, true);
    m_SAcusLF.Resize(nLFs2, true);
    m_SMcusLF.Resize(nLFs2, true);
    m_SAcmsLF.Resize(nLFs2, true);
    m_UcsLF.resize(nLFs2, LM_FLAG_FRAME_DEFAULT);
  } else {
    PopLocalFrame();
    const int iLF = m_ic2LF.front();
    m_ic2LF.erase(m_ic2LF.begin());
    m_ic2LF.push_back(iLF);
  }

  const int iLF = m_ic2LF.back();
  LocalFrame &LF = m_LFs[iLF];
#if 0
//#if 1
  UT::Print("+ [%d]\n", LF.m_T.m_iFrm);
#endif
  LF.Initialize(ILF);
  while (!LF.m_Zs.empty() && LF.m_Zs.back().m_iKF >= static_cast<int>(m_KFs.size())) {
    LF.PopFrameMeasurement();
  }
#ifdef CFG_DEBUG
  if (!LF.m_Zs.empty() && LF.m_Zs.back().m_iKF == static_cast<int>(m_KFs.size())) {
    UT_ASSERT(!m_IKFs2.empty() && m_IKFs2.front().m_T == LF.m_T);
  }
#endif
  std::vector<int> &iKF2X = m_idxsTmp1, &iX2z = m_idxsTmp2;
  PushFeatureMeasurementMatchesFirst(LF, iKF2X, iX2z);
  const int nLFs2 = int(m_LFs.size()), STL = std::min(nLFs2, LBA_MAX_SLIDING_TRACK_LENGTH);
  const int ic1 = nLFs2 - STL, ic2 = nLFs2 - 1;
  for (int _ic = ic1; _ic < ic2; ++_ic) {
    LocalFrame &_LF = m_LFs[m_ic2LF[_ic]];
    _LF.m_iLFsMatch.push_back(iLF);
    PushFeatureMeasurementMatchesNext(_LF, LF, iKF2X, iX2z, _LF.m_Zm);
  }
  m_ucsLF[iLF] = LBA_FLAG_FRAME_UPDATE_CAMERA;
  m_ucmsLF[iLF] = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION | LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                  LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                  LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION | LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
  m_UcsLF[iLF] = LM_FLAG_FRAME_UPDATE_CAMERA_LF;
#if 0
  if (nLFs2 > 1) {
    m_ucsLF[m_ic2LF[ic2 - 1]] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
  }
#endif
  const ubyte udFlagST = LBA_FLAG_TRACK_PUSH | LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
  const int NZ = static_cast<int>(LF.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = LF.m_Zs[iZ];
    bool pushST = false;
    ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
    KeyFrame &KF = m_KFs[Z.m_iKF];
    m_marksTmp.assign(KF.m_xs.size(), 0);
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const int ix = LF.m_zs[iz].m_ix, Nst = KF.CountSlidingTracks(ix), iSTMax = KF.m_ix2ST[ix + 1] - 1;
      if (Nst > 0 && KF.m_STs[iSTMax].m_icMin >= ic1) {
        LF.m_STs[iz].Set(Nst - 1, Nst);
        KF.m_STs[iSTMax].m_icMax = ic2;
      } else {
        LF.m_STs[iz].Set(Nst, Nst + 1);
        pushST = true;
        uds[ix] |= LBA_FLAG_TRACK_PUSH;
        //uds[ix] |= LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
        m_marksTmp[ix] = 1;
      }
    }
    if (!pushST) {
      continue;
    }
    m_ucsKF[Z.m_iKF] |= LBA_FLAG_FRAME_PUSH_TRACK;
    m_idxsListTmp.resize(STL);
    for (int i = 0; i < STL; ++i) {
      m_idxsListTmp[i].resize(0);
    }
    const int Nx = int(KF.m_xs.size());
    m_ix2STTmp.swap(KF.m_ix2ST);  KF.m_ix2ST.resize(Nx + 1);
    m_STsTmp.swap(KF.m_STs);      KF.m_STs.resize(0);
    m_usSTTmp.swap(KF.m_usST);    KF.m_usST.resize(0);
    m_AxsTmp.Swap(KF.m_AxsST);    KF.m_AxsST.Resize(0);
    m_MxsTmp.Swap(KF.m_MxsST);    KF.m_MxsST.Resize(0);
    for (int ix = 0; ix < Nx; ++ix) {
      const int iST1 = m_ix2STTmp[ix], iST2 = m_ix2STTmp[ix + 1], Nst = iST2 - iST1;
      KF.m_ix2ST[ix] = int(KF.m_STs.size());
      KF.m_STs.insert(KF.m_STs.end(), m_STsTmp.begin() + iST1, m_STsTmp.begin() + iST2);
      KF.m_usST.insert(KF.m_usST.end(), m_usSTTmp.begin() + iST1, m_usSTTmp.begin() + iST2);
      KF.m_AxsST.Push(m_AxsTmp.Data() + iST1, Nst);
      KF.m_MxsST.Push(m_MxsTmp.Data() + iST1, Nst);
      //if (!(uds[ix] & LBA_FLAG_TRACK_PUSH))
      if (!m_marksTmp[ix]) {
        continue;
      }
      const int NST1 = int(KF.m_STs.size()), NST2 = NST1 + 1;
      KF.m_STs.resize(NST2);
      KF.m_usST.push_back(udFlagST);
      KF.m_AxsST.Resize(NST2, true);  KF.m_AxsST[NST1].MakeZero();
      KF.m_MxsST.Resize(NST2, true);  KF.m_MxsST[NST1].MakeZero();
#ifdef CFG_DEBUG
      KF.m_MxsST[NST1].m_mdd.Invalidate();
#endif
      KeyFrame::SlidingTrack &ST = KF.m_STs[NST1];
      ST.Set(ic2);
      for (int _ic = ic1; _ic < ic2; ++_ic) {
        const int _iLF = m_ic2LF[_ic];
        LocalFrame &_LF = m_LFs[_iLF];
        std::vector<int> &_ix2z = m_idxsListTmp[_ic - ic1];
        if (_ix2z.empty()) {
          MarkFeatureMeasurements(_LF, Z.m_iKF, _ix2z);
        }
        const int _iz = _ix2z[ix];
        if (_iz == -1) {
          continue;
        }
        ST.m_icMin = std::min(ST.m_icMin, _ic);
#ifdef CFG_DEBUG
        const LocalFrame::SlidingTrack &_ST = _LF.m_STs[_iz];
        UT_ASSERT(_ST.m_ist2 == Nst);
#endif
        ++_LF.m_STs[_iz].m_ist2;
      }
    }
    KF.m_ix2ST[Nx] = static_cast<int>(KF.m_STs.size());
  }
  m_CsLF[iLF] = ILF.m_C;
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsLFGT[iLF] = m_CsGT[LF.m_T.m_iFrm];
  }
#endif
#ifdef CFG_INCREMENTAL_PCG
  m_xcsLF[iLF].MakeZero();
  m_xmsLF[iLF].MakeZero();
#endif
  IMU::Delta &D = m_DsLF[iLF];
  if (nLFs2 > 1) {
    const int _iLF = m_ic2LF[nLFs2 - 2];
    const LocalFrame &_LF = m_LFs[_iLF];
    const float _t = _LF.m_T.m_t;
    //UT::DebugStart();
    IMU::PreIntegrate(LF.m_us, _t, LF.m_T.m_t, m_CsLF[_iLF], &D, &m_work, true, &_LF.m_us.Back());
    //UT::DebugStop();
    if (nLFs2 > 2) {
      const int __iLF = m_ic2LF[nLFs2 - 3];
      IMU::Delta &_D = m_DsLF[_iLF];
      IMU::PreIntegrate(_LF.m_us, m_LFs[__iLF].m_T.m_t, _t, m_CsLF[__iLF], &_D, &m_work, true,
                        &_D.m_u1, &LF.m_us.Front());
      m_ucsLF[_iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
      m_ucmsLF[_iLF] |= LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION | LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION | LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
    }
  } else {
    D.Invalidate();
  }
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    if (nLFs2 > 1) {
      const int _iLF = m_ic2LF[nLFs2 - 2];
      const LocalFrame &_LF = m_LFs[_iLF];
      const float _t = _LF.m_T.m_t;
      const Camera &_C = m_CsLFGT[_iLF];
      IMU::PreIntegrate(LF.m_us, _t, LF.m_T.m_t, _C, &m_DsLFGT[iLF],
                        &m_work, true, &_LF.m_us.Back());
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
      D.DebugSetMeasurement(_C, m_CsLFGT[iLF], m_K.m_pu);
      m_DsLFGT[iLF].DebugSetMeasurement(_C, m_CsLFGT[iLF], m_K.m_pu);
#endif
      if (nLFs2 > 2) {
        const int __iLF = m_ic2LF[nLFs2 - 3];
        IMU::Delta &_D = m_DsLFGT[_iLF];
        IMU::PreIntegrate(_LF.m_us, m_LFs[__iLF].m_T.m_t, _t, m_CsLFGT[__iLF], &_D,
                          &m_work, true, &_D.m_u1, &LF.m_us.Front());
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
        m_DsLF[_iLF].DebugSetMeasurement(m_CsLFGT[__iLF], _C, m_K.m_pu);
        _D.DebugSetMeasurement(m_CsLFGT[__iLF], _C, m_K.m_pu);
#endif
      }
    } else {
      m_DsLFGT[iLF] = D;
    }
  }
  if (m_history >= 3) {
    MarkFeatureMeasurementsUpdateDepth(LF, m_ucsKFGT, m_udsGT);
  }
#endif
  if (LF.m_T.m_iFrm == 0) {    
    const LA::Vector3f s2r = LA::Vector3f::Get(BA_VARIANCE_FIX_ORIGIN_ROTATION_X,
                                               BA_VARIANCE_FIX_ORIGIN_ROTATION_Y,
                                               BA_VARIANCE_FIX_ORIGIN_ROTATION_Z);
    const float s2p = BA_VARIANCE_FIX_ORIGIN_POSITION;
    m_Zo.Set(BA_WEIGHT_FIX_ORIGIN, s2r, s2p, m_CsLF[iLF].m_T);
    m_Ao.MakeZero();
  }
  m_AdsLF[iLF].MakeZero();
  m_AfpsLF[iLF].MakeZero();
  m_AfmsLF[iLF].MakeZero();
  m_SAcusLF[iLF].MakeZero();
  m_SMcusLF[iLF].MakeZero();
  m_SAcmsLF[iLF].MakeZero();
  m_usKF.Push(ILF.m_us);
}

void LocalBundleAdjustor::PopLocalFrame() {
#ifdef LBA_DEBUG_EIGEN
  m_ZpBkp = m_Zp;
#endif
  m_ts[TM_TEST_1].Start();
  MarginalizeLocalFrame();
  m_ts[TM_TEST_1].Stop();
#ifdef LBA_DEBUG_EIGEN
  DebugMarginalizeLocalFrame();
#endif
  const int nLFs = static_cast<int>(m_LFs.size());
  const int STL = std::min(nLFs, LBA_MAX_SLIDING_TRACK_LENGTH);
  const int iLF = m_ic2LF.front();
  LocalFrame &LF = m_LFs[iLF];
  const int NZ = static_cast<int>(LF.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = LF.m_Zs[iZ];
    m_idxsListTmp.resize(STL);
    for (int i = 0; i < STL; ++i) {
      m_idxsListTmp[i].resize(0);
    }
    bool popST = false;
    ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
    const ubyte pushFrm = m_ucsKF[Z.m_iKF] & LBA_FLAG_FRAME_PUSH_TRACK;
    KeyFrame &KF = m_KFs[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const int ix = LF.m_zs[iz].m_ix, iSTMin = KF.m_ix2ST[ix];
      int icMin = -1;
      for (int _ic = 1; _ic < STL && icMin == -1; ++_ic) {
        std::vector<int> &_iz2x = m_idxsListTmp[_ic];
        if (_iz2x.empty()) {
          MarkFeatureMeasurements(m_LFs[m_ic2LF[_ic]], Z.m_iKF, _iz2x);
        }
        if (_iz2x[ix] != -1) {
          icMin = _ic;
        }
      }
      FTR::Factor::FixSource::A2 &Az = LF.m_Azs2[iz];
      Az.m_add.MakeMinus();
      FTR::Factor::FixSource::A3 &AzST = LF.m_AzsST[iz];
      AzST.m_add.MakeMinus();
#ifdef CFG_DEBUG
      //UT_ASSERT(LF.m_Nsts[iz] == 1);
      UT_ASSERT(LF.m_STs[iz].m_ist1 == 0 && LF.m_STs[iz].m_ist2 == 1);
#endif
      if (icMin == -1 ||
          (KF.m_ix2ST[ix + 1] - iSTMin > 1 && KF.m_STs[iSTMin + 1].m_icMin == icMin)) {
        popST = true;
        uds[ix] |= LBA_FLAG_TRACK_POP;
        KF.m_usST[iSTMin] |= LBA_FLAG_TRACK_POP;
        FTR::Factor::DD &mddST = KF.m_MxsST[iSTMin].m_mdd;
        mddST.MakeMinus();
        const bool nonZero = !(KF.m_usST[iSTMin] & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO);
        if (icMin != -1) {
          for (int _ic = 1; _ic < STL; ++_ic) {
            const int _iLF = m_ic2LF[_ic];
            LocalFrame &_LF = m_LFs[_iLF];
            std::vector<int> &_ix2z = m_idxsListTmp[_ic];
            if (_ix2z.empty()) {
              MarkFeatureMeasurements(_LF, Z.m_iKF, _ix2z);
            }
            const int _iz = _ix2z[ix];
            if (_iz == -1) {
              continue;
            }
#ifdef CFG_DEBUG
            UT_ASSERT(_LF.m_STs[_iz].m_ist1 == 0 && _LF.m_STs[_iz].m_ist2 > 1);
#endif
            --_LF.m_STs[_iz].m_ist2;
            if (nonZero) {
              _LF.m_SmddsST[_iz] += mddST;
              _LF.m_ms[_iz] |= LBA_FLAG_MARGINALIZATION_UPDATE;
            }
          }
        }
        if (nonZero) {
//#ifdef CFG_DEBUG
#if 0
          if (Z.m_iKF == 187 && ix == 1) {
            UT::PrintSeparator();
            const float dm = mdxST.m_add.m_a, Sm = KF.m_SmdxsST[ix].m_add.m_a;
            UT::Print("[-] %f + %f = %f\n", dm, Sm, dm + Sm);
          }
#endif
          KF.m_ms[ix] |= LBA_FLAG_MARGINALIZATION_UPDATE;
        }
      } else {
        KF.m_STs[iSTMin].m_icMin = icMin;
        if (!pushFrm || !(KF.m_usST[iSTMin] & LBA_FLAG_TRACK_PUSH)) {
          KF.m_usST[iSTMin] |= LBA_FLAG_TRACK_UPDATE_INFORMATION;
//#ifdef CFG_DEBUG
#if 0
          if (Z.m_iKF == 0 && ix == 4) {
            UT::Print("  SaddST = %e + %e = %e [%d]\n", AzST.m_add.m_a, KF.m_AxsST[iSTMin].m_Sadd.m_a,
                                                        AzST.m_add.m_a + KF.m_AxsST[iSTMin].m_Sadd.m_a, LF.m_T.m_iFrm);
          }
#endif
          KF.m_AxsST[iSTMin].m_Sadd += AzST.m_add;
        }
      }
      KF.m_Axs[ix].m_Sadd += Az.m_add;
      //m_F = -Az.m_F + m_F;
      uds[ix] |= LBA_FLAG_TRACK_UPDATE_INFORMATION;
    }
    if (Z.m_iz1 < Z.m_iz2) {
      m_ucsKF[Z.m_iKF] |= LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION;
    }
    if (popST) {
      m_ucsKF[Z.m_iKF] |= LBA_FLAG_FRAME_POP_TRACK;
    }
  }
  const int icMax = STL - 1;
  for (int _ic = 1; _ic < icMax; ++_ic) {
    const int iLF1 = m_ic2LF[_ic];
    LocalFrame &LF1 = m_LFs[iLF1];
    const int NI = int(LF1.m_Zm.m_Is.size());
    for (int iI = 0; iI < NI; ++iI) {
      const MeasurementMatchLF::Index &I = LF1.m_Zm.m_Is[iI];
      if (!(m_ucsKF[I.m_iKF] & LBA_FLAG_FRAME_POP_TRACK)) {
        continue;
      }
      const KeyFrame &KF = m_KFs[I.m_iKF];
      const int iLF2 = LF1.m_iLFsMatch[I.m_ik];
      const LocalFrame &LF2 = m_LFs[iLF2];
      const int i1 = LF1.m_Zm.m_iI2zm[iI], i2 = LF1.m_Zm.m_iI2zm[iI + 1];
      for (int i = i1; i < i2; ++i) {
        const int iz2 = LF1.m_Zm.m_izms[i].m_iz2, ix = LF2.m_zs[iz2].m_ix, iST = KF.m_ix2ST[ix];
        if (LF2.m_STs[iz2].m_ist1 != 0 || !(KF.m_usST[iST] & LBA_FLAG_TRACK_POP) ||
            (KF.m_usST[iST] & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO)) {
          continue;
        }
        //LF1.m_Zm.m_SmddsST[i] = -KF.m_MxsST[iST].m_mdd.m_a + LF1.m_ZmLF.m_SmddsST[i];
        LF1.m_Zm.m_SmddsST[i] = KF.m_MxsST[iST].m_mdd.m_a + LF1.m_Zm.m_SmddsST[i];
        LF1.m_Zm.m_ms[i] |= LBA_FLAG_MARGINALIZATION_UPDATE;
      }
    }
  }
  for (int _ic = STL; _ic < nLFs; ++_ic) {
    LocalFrame &_LF = m_LFs[m_ic2LF[_ic]];
    const int _NZ = int(_LF.m_Zs.size());
    for (int _iZ = 0; _iZ < _NZ; ++_iZ) {
      const FRM::Measurement &_Z = _LF.m_Zs[_iZ];
      if (!(m_ucsKF[_Z.m_iKF] & LBA_FLAG_FRAME_POP_TRACK)) {
        continue;
      }
      const KeyFrame &KF = m_KFs[_Z.m_iKF];
      const int _iz1 = _Z.m_iz1, _iz2 = _Z.m_iz2;
      for (int _iz = _iz1; _iz < _iz2; ++_iz) {
        if (KF.m_usST[KF.m_ix2ST[_LF.m_zs[_iz].m_ix]] & LBA_FLAG_TRACK_POP) {
          _LF.m_STs[_iz].Step();
        }
      }
    }
  }
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = LF.m_Zs[iZ];
    if (!(m_ucsKF[Z.m_iKF] & LBA_FLAG_FRAME_POP_TRACK)) {
      continue;
    }
    KeyFrame &KF = m_KFs[Z.m_iKF];
    const int Nx = int(KF.m_xs.size());
    m_ix2STTmp.swap(KF.m_ix2ST);
    KF.m_ix2ST.resize(Nx + 1);
    KF.m_ix2ST[0] = 0;
    for (int ix = 0, iST = 0; ix < Nx; ++ix) {
      const int iST1 = m_ix2STTmp[ix], iST2 = m_ix2STTmp[ix + 1];
      const bool popST = iST1 < iST2 && (KF.m_usST[iST1] & LBA_FLAG_TRACK_POP);
      for (int _iST = popST ? iST1 + 1 : iST1; _iST < iST2; ++iST, ++_iST) {
        KF.m_STs[iST] = KF.m_STs[_iST];
        KF.m_usST[iST] = KF.m_usST[_iST];
        KF.m_AxsST[iST] = KF.m_AxsST[_iST];
        KF.m_MxsST[iST] = KF.m_MxsST[_iST];
      }
      KF.m_ix2ST[ix + 1] = iST;
      if (popST && iST == KF.m_ix2ST[ix]) {
        KF.m_Axs[ix] = KF.m_Axps[ix];
        KF.m_ms[ix] &= ~LBA_FLAG_MARGINALIZATION_NON_ZERO;
      }
    }
    const int NST = KF.m_ix2ST.back();
    if (NST == 0) {
      KF.m_STs.clear();
      KF.m_usST.clear();
      KF.m_AxsST.Clear();
      KF.m_MxsST.Clear();
    } else {
      KF.m_STs.resize(NST);
      KF.m_usST.resize(NST);
      KF.m_AxsST.Resize(NST);
      KF.m_MxsST.Resize(NST);
    }
  }
  const int _iLF = m_ic2LF[1];
  IMU::Delta::Factor &Ad = m_AdsLF[_iLF];
  Ad.m_A22.MakeMinus();
  m_SAcusLF[_iLF] += Ad.m_A22.m_Acc;
  Camera::Factor &SAcm = m_SAcmsLF[_iLF];
  SAcm.m_Au.m_Acm += Ad.m_A22.m_Acm;
  SAcm.m_Au.m_Amm += Ad.m_A22.m_Amm;
  SAcm.m_Ab.MakeZero();
  Ad.MakeZero();
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    KeyFrame &KF = m_KFs[iKF];
    const int NST = int(KF.m_STs.size());
    for (int iST = 0; iST < NST; ++iST) {
      KF.m_STs[iST].Step();
    }
  }
#ifdef CFG_GROUND_TRUTH
  if (m_history >= 3) {
    MarkFeatureMeasurementsUpdateDepth(LF, m_ucsKFGT, m_udsGT);
  }
#endif
}

void LocalBundleAdjustor::PushKeyFrame(const GlobalMap::InputKeyFrame &IKF) {
  const int nKFs1 = static_cast<int>(m_KFs.size()), nKFs2 = nKFs1 + 1;
  m_KFs.resize(nKFs2);
  m_CsKF.Resize(nKFs2, true);
  m_ucsKF.resize(nKFs2, LBA_FLAG_FRAME_UPDATE_CAMERA);
  m_UcsKF.resize(nKFs2, LM_FLAG_FRAME_UPDATE_CAMERA_KF);
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsKFGT.Resize(nKFs2, true);
  }
  if (m_history >= 3) {
    m_ucsKFGT.resize(nKFs2, LBA_FLAG_FRAME_DEFAULT);
  }
#endif
  std::vector<int> &iks = m_idxsTmp1;
  KeyFrame &KF = m_KFs[nKFs1];
  KF.Initialize(IKF);
  const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
#ifdef CFG_DEBUG
  for (int ik = 0; ik < Nk; ++ik) {
    UT_ASSERT(KF.m_iKFsMatch[ik] < nKFs1);
  }
#endif
  for (int ik = 0; ik < Nk; ++ik) {
    m_KFs[KF.m_iKFsMatch[ik]].InsertMatchKeyFrame(nKFs1);
  }
  m_iKF2d.push_back(m_iKF2d.back());
  m_dsBkp.resize(KF.m_zs.size());
  const int NZ1 = static_cast<int>(KF.m_Zs.size());
  for (int iZ = 0; iZ < NZ1; ++iZ) {
    const FRM::Measurement &Z = KF.m_Zs[iZ];
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      m_dsBkp[iz] = ds[KF.m_zs[iz].m_ix];
//#ifdef CFG_DEBUG
#if 0
      if (iz == 0) {
        UT::PrintSeparator();
      }
      UT::Print("iz = %d, ix = %d, d = %f\n", iz, KF.m_zs[iz].m_ix, m_dsBkp[iz].u());
#endif
    }
  }
  const ubyte udFlag1 = LBA_FLAG_TRACK_UPDATE_DEPTH | LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
  const int NX = static_cast<int>(IKF.m_Xs.size());
  for (int iX1 = 0, iX2 = 0; iX1 < NX; iX1 = iX2) {
    const int iKF = IKF.m_Xs[iX1].m_iKF;
    for (iX2 = iX1 + 1; iX2 < NX && IKF.m_Xs[iX2].m_iKF == iKF; ++iX2) {}
    const int id = m_iKF2d[iKF + 1], Nx = iX2 - iX1;
    for (int jKF = iKF; jKF < nKFs2; ++jKF) {
      m_iKF2d[jKF + 1] += Nx;
    }
    m_ds.insert(m_ds.begin() + id, Nx, Depth::InverseGaussian());
    m_uds.insert(m_uds.begin() + id, Nx, udFlag1);
    m_Uds.insert(m_Uds.begin() + id, Nx, LM_FLAG_TRACK_UPDATE_DEPTH);
    m_ucsKF[iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
    m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_DEPTH;
#ifdef CFG_GROUND_TRUTH
    if (m_history >= 3) {
      m_ucsKFGT[iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
      m_udsGT.insert(m_udsGT.begin() + id, Nx, LBA_FLAG_TRACK_UPDATE_DEPTH);
    }
#endif
    const GlobalMap::Point *Xs = IKF.m_Xs.data() + iX1;
    Depth::InverseGaussian *ds = m_ds.data() + id;
    ubyte *uds = m_uds.data() + id;
    m_xsTmp.resize(Nx);
    KeyFrame &_KF = m_KFs[iKF];
    for (int i = 0, ix = static_cast<int>(_KF.m_xs.size()); i < Nx; ++i, ++ix) {
      const GlobalMap::Point &X = Xs[i];
      ds[i] = X.m_d;
      m_xsTmp[i] = X.m_x;
      const int Nz = static_cast<int>(X.m_zs.size());
#ifdef LBA_FLAG_TRACK_MEASURE_KF
      if (Nz > 0) {
        uds[i] |= LBA_FLAG_TRACK_MEASURE_KF;
      }
#endif
      iks.resize(Nz);
      for (int i2 = 0; i2 < Nz; ++i2) {
        const FTR::Measurement &z2 = X.m_zs[i2];
        KeyFrame &KF2 = m_KFs[z2.m_iKF];
        int &ik2 = iks[i2];
        const int Nk2 = static_cast<int>(KF2.m_iKFsMatch.size());
        KF2.PushFeatureMeasurement(iKF, ix, z2, &ik2);
        if (static_cast<int>(KF2.m_iKFsMatch.size()) > Nk2) {
          _KF.InsertMatchKeyFrame(z2.m_iKF);
        }
        for (int i1 = 0; i1 < i2; ++i1) {
          const int iKF1 = X.m_zs[i1].m_iKF;
          const std::vector<int>::iterator _ik2 = std::lower_bound(KF2.m_iKFsMatch.begin() + ik2,
                                                                   KF2.m_iKFsMatch.end(), iKF1);
          ik2 = static_cast<int>(_ik2 - KF2.m_iKFsMatch.begin());
          if (_ik2 != KF2.m_iKFsMatch.end() && *_ik2 == iKF1) {
            continue;
          }
          KF2.InsertMatchKeyFrame(iKF1, &_ik2);
          KeyFrame &KF1 = m_KFs[iKF1];
          int &ik1 = iks[i1];
          const std::vector<int>::iterator _ik1 = std::lower_bound(KF1.m_iKFsMatch.begin() + ik1,
                                                                   KF1.m_iKFsMatch.end(), z2.m_iKF);
          ik1 = static_cast<int>(_ik1 - KF1.m_iKFsMatch.begin());
          KF1.InsertMatchKeyFrame(z2.m_iKF, &_ik1);
        }
      }
    }
    _KF.PushFeatures(m_xsTmp);
  }
  std::vector<int> &iKF2X = m_idxsTmp1, &iX2z = m_idxsTmp2;
  PushFeatureMeasurementMatchesFirst(KF, iKF2X, iX2z);
  m_CsKF[nKFs1] = IKF.m_C.m_T;
#ifdef CFG_GROUND_TRUTH
  if (m_CsGT) {
    m_CsKFGT[nKFs1] = m_CsGT[KF.m_T.m_iFrm].m_T;
  }
#endif

  const ubyte udFlag2 = LBA_FLAG_TRACK_PUSH | LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO;
  const int nLFs = static_cast<int>(m_ic2LF.size());
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    LocalFrame &LF = m_LFs[iLF];
#ifdef CFG_DEBUG
    UT_ASSERT(LF.m_Zs.empty() || LF.m_Zs.back().m_iKF < nKFs1);
#endif
    if (LF.m_T == IKF.m_T) {
      const int Nx = static_cast<int>(KF.m_xs.size()), Nz = static_cast<int>(LF.m_zs.size());
//#ifdef CFG_DEBUG
#if 0
      UT_ASSERT(static_cast<int>(KF.m_zs.size()) >= Nz);
#endif
      if (Nx > 0) {
        LF.PushFrameMeasurement(nKFs1, Nx);
      } else {
        bool found = false;
        const int NZ = static_cast<int>(LF.m_Zs.size());
        for (int iZ = 0; iZ < NZ && !found; ++iZ) {
          const FRM::Measurement &Z = LF.m_Zs[iZ];
          const int iX = iKF2X[Z.m_iKF];
          if (iX == -1) {
            continue;
          }
          const int *_ix2z = iX2z.data() + iX;
          for (int iz = Z.m_iz1; iz < Z.m_iz2 && !found; ++iz) {
            found = _ix2z[LF.m_zs[iz].m_ix] != -1;
          }
        }
        if (found) {
          LF.m_iKFsMatch.push_back(nKFs1);
        }
      }
      KF.m_iKFNearest = LF.m_T.m_iFrm == 0 ? 0 : LF.m_iKFNearest;
      LF.m_iKFNearest = nKFs1;
      m_ucsKF[nKFs1] |= LBA_FLAG_FRAME_PUSH_TRACK;
      ubyte *uds = m_uds.data() + m_iKF2d[nKFs1];
      const GlobalMap::Point *Xs = IKF.m_Xs.data() + IKF.m_Xs.size() - Nx;
#ifdef CFG_DEBUG
      for (int ix = 0; ix < Nx; ++ix) {
        UT_ASSERT(Xs[ix].m_iKF == nKFs1);
      }
      if (static_cast<int>(IKF.m_Xs.size()) > Nx) {
        UT_ASSERT(Xs[-1].m_iKF < nKFs1);
      }
#endif
      for (int ix = 0, iz = Nz; ix < Nx; ++ix, ++iz) {
        const FTR::Source &x = KF.m_xs[ix];
        LF.m_zs[iz].Set(ix, x.m_x, Xs[ix].m_W
#ifdef CFG_STEREO
                      , x.m_xr, x.m_Wr
#endif
#ifdef CFG_DEPTH_MAP
                      , x.m_d
#endif
                        );
        LF.m_STs[iz].Set(0, 1);
        KF.m_ix2ST[ix] = ix;
        uds[ix] |= udFlag2;
      }
      KF.m_ix2ST[Nx] = Nx;
      KF.m_STs.assign(Nx, KeyFrame::SlidingTrack(ic));
      KF.m_usST.assign(Nx, udFlag2);
      KF.m_AxsST.Resize(Nx);  KF.m_AxsST.MakeZero();
      KF.m_MxsST.Resize(Nx);  KF.m_MxsST.MakeZero();
#ifdef CFG_DEBUG
      for (int ix = 0; ix < Nx; ++ix) {
        KF.m_MxsST[ix].m_mdd.Invalidate();
      }
#endif
    } else {
      bool found = false;
      const int NZ = static_cast<int>(LF.m_Zs.size());
      for (int iZ = 0; iZ < NZ && !found; ++iZ) {
        const FRM::Measurement &Z = LF.m_Zs[iZ];
        const int iX = iKF2X[Z.m_iKF];
        if (iX == -1) {
          continue;
        }
        const int *_ix2z = iX2z.data() + iX;
        for (int iz = Z.m_iz1; iz < Z.m_iz2 && !found; ++iz) {
          found = _ix2z[LF.m_zs[iz].m_ix] != -1;
        }
      }
      if (found) {
        LF.m_iKFsMatch.push_back(nKFs1);
      }
    }
  }
  //const ubyte udFlag3 = LBA_FLAG_TRACK_MEASURE_KF;
  const bool ud = LBA_RESET_DEPTH_INFORMATION;
  const ubyte udFlag3 = (ud ? LBA_FLAG_TRACK_UPDATE_DEPTH : LBA_FLAG_TRACK_DEFAULT)
#ifdef LBA_FLAG_TRACK_MEASURE_KF
                       | LBA_FLAG_TRACK_MEASURE_KF
#endif
                       ;
  const int NZ2 = static_cast<int>(KF.m_Zs.size());
  for (int iZ = 0; iZ < NZ2; ++iZ) {
    const FRM::Measurement &Z = KF.m_Zs[iZ];
    if (ud) {
      m_ucsKF[Z.m_iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
    }
    ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      uds[KF.m_zs[iz].m_ix] |= udFlag3;
    }
  }
  m_GM->LBA_Push(IKF.m_T.m_iFrm, IKF.m_C.m_T);
  const int N = static_cast<int>(std::lower_bound(m_usKF.Data(), m_usKF.End(), KF.m_T.m_t) -
                                                  m_usKF.Data());
  m_usKF.Erase(N, m_usBkp);
  m_GBA->PushKeyFrame(IKF, m_usBkp, m_dsBkp);
#ifdef CFG_GROUND_TRUTH
  if (m_history >= 3) {
    MarkFeatureMeasurementsUpdateDepth(KF, m_ucsKFGT, m_udsGT);
  }
#endif
}

void LocalBundleAdjustor::SearchMatchingKeyFrames(FRM::Frame &F) {
  std::vector<int> &iKF2Z = m_idxsTmp1;
  const int nKFs1 = static_cast<int>(m_KFs.size());
  const int nKFs2 = !F.m_Zs.empty() && F.m_Zs.back().m_iKF < nKFs1 ? nKFs1 : nKFs1 + 1;
  iKF2Z.assign(nKFs2, -1);
  const int NZ = static_cast<int>(F.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    iKF2Z[F.m_Zs[iZ].m_iKF] = iZ;
  }
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const GlobalMap::KeyFrame &KF = *(Z.m_iKF == nKFs1 ? (GlobalMap::KeyFrame *) &m_IKFs2.front()
       : (GlobalMap::KeyFrame *) &m_KFs[Z.m_iKF]);
    m_marksTmp.assign(KF.m_xs.size(), 0);
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      m_marksTmp[F.m_zs[iz].m_ix] = 1;
    }
    const int nKFsMatch = static_cast<int>(KF.m_iKFsMatch.size());
    for (int i = 0; i < nKFsMatch; ++i) {
      const int _iKF = KF.m_iKFsMatch[i];
      if (iKF2Z[_iKF] != -1) {
        continue;
      }
      const KeyFrame &_KF = m_KFs[_iKF];
      const int _iZ = _KF.SearchFrameMeasurement(Z.m_iKF);
      if (_iZ == -1) {
        continue;
      }
      const FRM::Measurement &_Z = _KF.m_Zs[_iZ];
      const int _iz1 = _Z.m_iz1, _iz2 = _Z.m_iz2;
      int _iz;
      for (_iz = _iz1; _iz < _iz2 && !m_marksTmp[_KF.m_zs[_iz].m_ix]; ++_iz);
      if (_iz < _iz2) {
        iKF2Z[_iKF] = -2;
      }
    }
  }
  F.m_iKFsMatch.resize(0);
  for (int iKF = 0; iKF < nKFs2; ++iKF) {
    const int iZ = iKF2Z[iKF];
    if (iZ == -1) {
      continue;
    } else if (iZ >= 0) {
      F.m_Zs[iZ].m_ik = static_cast<int>(F.m_iKFsMatch.size());
    }
    F.m_iKFsMatch.push_back(iKF);
  }
}

void LocalBundleAdjustor::PushFeatureMeasurementMatchesFirst(const FRM::Frame &F,
                                                             std::vector<int> &iKF2X, std::vector<int> &iX2z) {
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
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      ix2z[F.m_zs[iz].m_ix] = iz;
    }
  }
}

void LocalBundleAdjustor::PushFeatureMeasurementMatchesNext(const FRM::Frame &F1,
                                                            const FRM::Frame &F2, const std::vector<int> &iKF2X, const std::vector<int> &iX2z2,
                                                            MeasurementMatchLF &Zm) {
  ubyte firstKF = 1;
  const int NZ1 = int(F1.m_Zs.size());
  for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
    const FRM::Measurement &Z1 = F1.m_Zs[iZ1];
    const int iX = iKF2X[Z1.m_iKF];
    if (iX == -1) {
      continue;
    }
    m_izmsTmp.resize(0);
    const int *ix2z2 = iX2z2.data() + iX;
    const int iz11 = Z1.m_iz1, iz12 = Z1.m_iz2;
    for (int iz1 = iz11; iz1 < iz12; ++iz1) {
      const int iz2 = ix2z2[F1.m_zs[iz1].m_ix];
      if (iz2 != -1) {
        m_izmsTmp.push_back(FTR::Measurement::Match(iz1, iz2));
      }
    }
    if (!m_izmsTmp.empty()) {
      Zm.PushFeatureMeasurementMatches(m_izmsTmp, Z1.m_iKF, &firstKF);
    }
  }
  if (firstKF) {
    m_izmsTmp.resize(0);
    Zm.FRM::MeasurementMatch::PushFeatureMeasurementMatches(m_izmsTmp);
  }
}

void LocalBundleAdjustor::MarkFeatureMeasurements(const LocalFrame &LF, const int iKF,
                                                  std::vector<int> &ix2z) {
  ix2z.assign(m_KFs[iKF].m_xs.size(), -1);
  const int iZ = LF.SearchFrameMeasurement(iKF);
  if (iZ == -1) {
    return;
  }
  const FRM::Measurement &Z = LF.m_Zs[iZ];
  for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
    ix2z[LF.m_zs[iz].m_ix] = iz;
  }
}

void LocalBundleAdjustor::MarkFeatureMeasurementsUpdateDepth(const FRM::Frame &F,
                                                             std::vector<ubyte> &ucsKF,
                                                             std::vector<ubyte> &uds) {
  const int NZ = static_cast<int>(F.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const int iKF = Z.m_iKF;
    ucsKF[iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
    ubyte *uds = m_udsGT.data() + m_iKF2d[iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      uds[F.m_zs[iz].m_ix] |= LBA_FLAG_TRACK_UPDATE_DEPTH;
    }
  }
}

#ifdef CFG_DEBUG
void LocalBundleAdjustor::DebugSetFeatureMeasurements(const Rigid3D &C,
                                                      const AlignedVector<Rigid3D> &CsKF,
                                                      const std::vector<Depth::InverseGaussian> &ds,
                                                      FRM::Frame &F) {
  Rigid3D Tr[2];
  const int NZ = static_cast<int>(F.m_Zs.size()), nKFs = static_cast<int>(m_KFs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const int iKF = Z.m_iKF;
    if (iKF >= nKFs) {
      continue;
    }
    *Tr = C / CsKF[iKF];
#ifdef CFG_STEREO
    Tr[1] = Tr[0];
    Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
    const Depth::InverseGaussian *_ds = ds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      FTR::Measurement &z = F.m_zs[iz];
      FTR::DebugSetMeasurement(Tr, KF.m_xs[z.m_ix], _ds[z.m_ix], z);
    }
  }
}
#endif

int LocalBundleAdjustor::CountMeasurementsFrameLF() {
  int SN = 0;
  const int nLFs = int(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    SN += int(m_LFs[iLF].m_Zs.size());
  }
  return SN;
}

int LocalBundleAdjustor::CountMeasurementsFrameKF() {
  int SN = 0;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SN += int(m_KFs[iKF].m_Zs.size());
  }
  return SN;
}

int LocalBundleAdjustor::CountMeasurementsFeatureLF() {
  int SN = 0;
  const int nLFs = int(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    SN += int(m_LFs[iLF].m_zs.size());
  }
  return SN;
}

int LocalBundleAdjustor::CountMeasurementsFeatureKF() {
  int SN = 0;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SN += int(m_KFs[iKF].m_zs.size());
  }
  return SN;
}

int LocalBundleAdjustor::CountLocalTracks() {
  int SN = 0;
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (KF.m_ix2ST[ix] != KF.m_ix2ST[ix + 1]) {
        ++SN;
      }
    }
  }
  return SN;
}

int LocalBundleAdjustor::CountSlidingTracks() {
  int SN = 0;
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    SN += static_cast<int>(m_KFs[iKF].m_STs.size());
  }
  return SN;
}

int LocalBundleAdjustor::CountSchurComplements() {
  return m_SAcusLF.Size() + CountSchurComplementsOffDiagonal();
}

int LocalBundleAdjustor::CountSchurComplementsOffDiagonal() {
  int SN = 0;
  const int nLFs = int(m_LFs.size());
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    SN += int(m_LFs[iLF].m_iLFsMatch.size());
  }
  return SN;
}

float LocalBundleAdjustor::ComputeImageMotion(const float z1, const Rigid3D &C1, const Rigid3D &C2,
                                              ubyte *first /* = NULL */) {
  Rigid3D C12;
  C1.ApplyRotation(C2.r_20_21_22_x(), C12.r_20_21_22_x());
  if (fabs(C12.r20()) < fabs(C12.r21())) {
    C12.r_10_11_12_x().vset_all_lane(0.0f, C12.r22(), -C12.r21(), 0.0f);
    C12.r_10_11_12_x().normalize012();
    SIMD::Cross012(C12.r_10_11_12_x(), C12.r_20_21_22_x(), C12.r_00_01_02_x());
  } else {
    C12.r_00_01_02_x().vset_all_lane(C12.r22(), 0.0f, -C12.r20(), 0.0f);
    C12.r_00_01_02_x().normalize012();
    SIMD::Cross012(C12.r_20_21_22_x(), C12.r_00_01_02_x(), C12.r_10_11_12_x());
  }
  C12.SetPosition(C1.GetApplied(C2.GetPosition()));
  m_work.Resize(4 * (sizeof(Point3D) + 2 * sizeof(Point2D)) / sizeof(float));
  AlignedVector<Point3D> X1s((Point3D *) m_work.Data(), 4, false);
  AlignedVector<Point2D> x1s((Point2D *) (X1s.Data() + 4), 4, false);
  AlignedVector<Point2D> x2s(x1s.Data() + 4, 4, false);
  if (!first || *first) {
    if (first) {
      *first = 0;
    }
    const Intrinsic &K = m_K.m_K;
    x1s[0].Set(-K.fxIcx(), -K.fyIcy());
    x1s[1].Set(-K.fxIcx(), K.fyIcy());
    x1s[2].Set(K.fxIcx(), K.fyIcy());
    x1s[3].Set(K.fxIcx(), -K.fyIcy());
    for (int i = 0; i < 4; ++i) {
      X1s[i].Set(x1s[i], z1);
    }
  }
  for (int i = 0; i < 4; ++i) {
    C12.GetApplied(X1s[i]).Project(x2s[i]);
  }
  LA::AlignedVectorXf e2s((float *) x2s.Data(), 4 * 2, false);
  e2s -= (float *) x1s.Data();
  return sqrtf(e2s.SquaredLength() * m_K.m_K.fxy() * 0.25f);
}
