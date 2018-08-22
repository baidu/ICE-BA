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
#include "LocalBundleAdjustor.h"

#ifdef CFG_DEBUG_EIGEN
//#ifdef LBA_ME_FUNCTION
#if 0
#undef LBA_ME_FUNCTION
#define LBA_ME_FUNCTION ME::FUNCTION_NONE
#endif

//#define LBA_DEBUG_PRINT_STEP

const int pc = 6, pm = 9;
const int pcm = pc + pm;

void LocalBundleAdjustor::DebugMarginalizeLocalFrame() {
  //// TODO (haomin): Bad condition!!!
  //return;
  //if (m_debug <= 0) {
  if (m_debug <= 1) {
    return;
  }
  if (static_cast<int>(m_LFs.size()) < LBA_MAX_LOCAL_FRAMES) {
    return;
  }
  CameraPrior::Joint::EigenPrior e_Ap;
  CameraPrior::Motion::EigenPrior e_ApLF;
  CameraPrior::Pose::EigenPrior e_ApKF;
  e_Ap.Set(m_ZpBkp, true);

  const int iLF1 = m_ic2LF[0], iLF2 = m_ic2LF[1];
  const LocalFrame &LF1 = m_LFs[iLF1];
  const int iKFr = LF1.m_iKFNearest;
  const std::string str = UT::String("LBA [%d] <-- [%d]", m_LFs[m_ic2LF.back()].m_T.m_iFrm + 1,
                                     LF1.m_T.m_iFrm);
  if (LF1.m_T.m_iFrm == m_KFs[iKFr].m_T.m_iFrm) {
    const bool v = m_ZpBkp.Pose::Valid();
    if (v) {
      e_Ap.GetPriorPose(iKFr, &e_ApKF);
      e_Ap.GetPriorMotion(&e_ApLF);
    } else {
      CameraPrior::Motion ZpLF;
      ZpLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY_FIRST,
                      BA_VARIANCE_PRIOR_BIAS_ACCELERATION_FIRST,
                      BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST);
      m_ZpBkp.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, iKFr, m_CsKF[iKFr],
                         BA_VARIANCE_PRIOR_GRAVITY_FIRST, ZpLF, true);
      e_ApLF.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, BA_VARIANCE_PRIOR_VELOCITY_FIRST,
                        BA_VARIANCE_PRIOR_BIAS_ACCELERATION_FIRST,
                        BA_VARIANCE_PRIOR_BIAS_GYROSCOPE_FIRST);
    }
    const float s2r = v ? BA_VARIANCE_PRIOR_GRAVITY_NEW
                        : BA_VARIANCE_PRIOR_GRAVITY_FIRST;
    e_Ap.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, s2r, e_ApLF);

    Camera C1;
    C1.m_T = m_CsKF[iKFr];
    C1.m_T.GetPosition(C1.m_p);
    m_ZpBkp.GetMotion(C1.m_T, &C1.m_v, &C1.m_ba, &C1.m_bw);

    IMU::Delta::Error e;
    IMU::Delta::Jacobian::RelativeKF J;
    IMU::Delta::Factor::Auxiliary::RelativeKF A;
    IMU::Delta::EigenFactor::RelativeKF e_A;
    const Camera &C2 = m_CsLF[iLF2];
    const IMU::Delta &D = m_DsLF[iLF2];
    D.GetFactor(BA_WEIGHT_IMU, C1, C2, m_K.m_pu, &e, &J, &A, BA_ANGLE_EPSILON);
    D.EigenGetFactor(BA_WEIGHT_IMU, C1, C2, m_K.m_pu, &e_A, BA_ANGLE_EPSILON);
    e_A.AssertEqual(A);
    e_A.Set(A, e_A.m_F);
    e_Ap.PropagateKF(e_A);
  } else {
    //Camera C1;
    Rigid3D /*Tr, TrI, */Trr, Tr1, Trk, Tk;
    EigenMatrix6x6f e_SAxx, e_SAxz, e_SAzz;
    EigenVector6f e_Sbx, e_Sbz;
    /*const */int Nk = static_cast<int>(m_ZpBkp.m_iKFs.size()) - 1;
#ifdef CFG_DEBUG
    UT_ASSERT(m_ZpBkp.m_iKFs[Nk] == INT_MAX);
#endif
    //m_ZpBkp.GetReferencePose(m_CsKF[m_ZpBkp.m_iKFr], &Tr, &TrI);
    //m_ZpBkp.GetPose(TrI, Nk, &Tr1, &C1.m_T);
    m_ZpBkp.m_Zps[Nk].GetInverse(Tr1);
    const float eps = FLT_EPSILON;
    const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
    const int NZ = static_cast<int>(LF1.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF1.m_Zs[iZ];
      const int iKF = Z.m_iKF;
      const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
      const KeyFrame &KF = m_KFs[iKF];
      if (iKF == m_ZpBkp.m_iKFr) {
        e_SAzz.setZero();
        e_Sbz.setZero();
        Trr.MakeIdentity();
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          const FTR::Measurement &z = LF1.m_zs[iz];
          const int ix = z.m_ix;
          const FTR::EigenFactor e_A = FTR::EigenGetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE,
                                       Trr, KF.m_xs[ix], ds[ix], Tr1, z, false, true
#ifdef CFG_STEREO
                                     , m_K.m_br
#endif
                                     );
          e_SAzz += e_A.m_Aczz;
          e_Sbz += e_A.m_bcz;
          if (e_A.m_add < epsd) {
            continue;
          }
          const EigenVector6f mdczT = EigenVector6f((e_A.m_adcz / e_A.m_add).transpose());
          e_SAzz -= EigenMatrix6x6f(mdczT * e_A.m_adcz);
          e_Sbz -= EigenVector6f(mdczT * e_A.m_bd);
        }
        e_Ap.Update(Nk, e_SAzz, e_Sbz);
      } else {
        const std::vector<int>::const_iterator it = std::lower_bound(m_ZpBkp.m_iKFs.begin(),
                                                                     m_ZpBkp.m_iKFs.end(), iKF);
        const int ik = static_cast<int>(it - m_ZpBkp.m_iKFs.begin());
        if (it == m_ZpBkp.m_iKFs.end() || *it != iKF) {
          //Tk = m_CsKF[iKF];
          //Rigid3D::ABI(Tk, Tr, Trk);
          Rigid3D::ABI(m_CsKF[iKF], m_CsKF[m_ZpBkp.m_iKFr], Trk);
          m_ZpBkp.Insert(BA_WEIGHT_PRIOR_CAMERA_INITIAL, ik, iKF, Trk,
                         BA_VARIANCE_PRIOR_POSITION_NEW, BA_VARIANCE_PRIOR_ROTATION_NEW,
                         &m_work);
          e_Ap.Insert(BA_WEIGHT_PRIOR_CAMERA_INITIAL, ik,
                      BA_VARIANCE_PRIOR_POSITION_NEW, BA_VARIANCE_PRIOR_ROTATION_NEW);
          ++Nk;
        } else {
          //m_ZpBkp.GetPose(TrI, ik, &Trk, &Tk);
          m_ZpBkp.m_Zps[ik].GetInverse(Trk);
        }
        e_SAxx.setZero();
        e_SAxz.setZero();
        e_SAzz.setZero();
        e_Sbx.setZero();
        e_Sbz.setZero();
        for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
          const FTR::Measurement &z = LF1.m_zs[iz];
          const int ix = z.m_ix;
          const FTR::EigenFactor e_A = FTR::EigenGetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE,
                                       Trk, KF.m_xs[ix], ds[ix], Tr1, z, true, true
#ifdef CFG_STEREO
                                     , m_K.m_br
#endif
                                     );
          e_SAxx += e_A.m_Acxx;
          e_SAxz += e_A.m_Acxz;
          e_SAzz += e_A.m_Aczz;
          e_Sbx += e_A.m_bcx;
          e_Sbz += e_A.m_bcz;
          if (e_A.m_add < epsd) {
            continue;
          }
          const EigenVector6f mdcxT = EigenVector6f((e_A.m_adcx / e_A.m_add).transpose());
          const EigenVector6f mdczT = EigenVector6f((e_A.m_adcz / e_A.m_add).transpose());
          e_SAxx -= EigenMatrix6x6f(mdcxT * e_A.m_adcx);
          e_SAxz -= EigenMatrix6x6f(mdcxT * e_A.m_adcz);
          e_SAzz -= EigenMatrix6x6f(mdczT * e_A.m_adcz);
          e_Sbx -= EigenVector6f(mdcxT * e_A.m_bd);
          e_Sbz -= EigenVector6f(mdczT * e_A.m_bd);
        }
        e_Ap.Update(ik, Nk, e_SAxx, e_SAxz, e_SAzz, e_Sbx, e_Sbz);
      }
    }
    if (iKFr == m_ZpBkp.m_iKFr) {
      //C1.m_T.GetPosition(C1.m_p);
    } else {
      //Tr = m_CsKF[iKFr];
      //C1 = m_CsLF[iLF1];
      e_Ap.GetPriorPose(INT_MAX, &e_ApKF);
      e_Ap.GetPriorMotion(&e_ApLF);
      e_Ap.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL,
                      BA_VARIANCE_PRIOR_GRAVITY_NEW, e_ApLF,
                      BA_VARIANCE_PRIOR_POSITION_NEW, BA_VARIANCE_PRIOR_ROTATION_NEW);
      m_ZpBkp.Initialize(BA_WEIGHT_PRIOR_CAMERA_INITIAL, iKFr, m_CsKF[iKFr],
                         BA_VARIANCE_PRIOR_GRAVITY_NEW, m_ZpLF, false,
                         &m_CsLF[iLF1].m_T, BA_VARIANCE_PRIOR_POSITION_NEW,
                         BA_VARIANCE_PRIOR_ROTATION_NEW);
    }
    //m_ZpBkp.GetMotion(C1.m_T, &C1.m_v, &C1.m_ba, &C1.m_bw);
    //const Camera &C2 = m_CsLF[iLF2];
    const Rigid3D &Tr = m_CsKF[iKFr];
    Rigid3D _Tr, TrI, Tr2;
    Camera C1, C2;
    LA::AlignedVector3f v2;
    m_ZpBkp.GetReferencePose(Tr, &_Tr, &TrI);
    Rigid3D::ABI(Tr1, TrI, C1.m_T);
    C1.m_T.GetPosition(C1.m_p);
    m_ZpBkp.GetMotion(C1.m_T, &C1.m_v, &C1.m_ba, &C1.m_bw);
    C2 = m_CsLF[iLF2];
    Rigid3D::ABI(C2.m_T, Tr, Tr2);
    C2.m_T.ApplyRotation(C2.m_v, v2);
    Rigid3D::ABI(Tr2, TrI, C2.m_T);
    C2.m_T.GetPosition(C2.m_p);
    C2.m_T.ApplyRotationInversely(v2, C2.m_v);

    IMU::Delta::Error e;
    IMU::Delta::Jacobian::RelativeLF J;
    IMU::Delta::Factor::Auxiliary::RelativeLF A;
    IMU::Delta::EigenFactor::RelativeLF e_A;
    const IMU::Delta &D = m_DsLF[iLF2];
    D.GetFactor(BA_WEIGHT_IMU, C1, C2, m_K.m_pu, _Tr, &e, &J, &A, BA_ANGLE_EPSILON);
    D.EigenGetFactor(BA_WEIGHT_IMU, C1, C2, m_K.m_pu, Tr, &e_A, BA_ANGLE_EPSILON);
    e_A.AssertEqual(A);
    e_A.Set(A, e_A.m_F);
    e_Ap.PropagateLF(e_A);
  }
  e_Ap.AssertEqual(m_Zp, 1, str + " Ap");
//#ifdef CFG_DEBUG
#if 0
  e_Ap.Set(m_Zp);
  //e_Ap.m_A.Print(true);
  UT::DebugStart();
  m_Zp.GetPriorMotion(&m_ZpLF, &m_work);
  UT::DebugStop();
#endif
  e_Ap.GetPriorMotion(&e_ApLF);
  e_ApLF.AssertEqual(m_ZpLF, 1, str + " ApLF");
  if (m_ZpBkp.Pose::Valid() && iKFr != m_ZpBkp.m_iKFr) {
    e_ApKF.AssertEqual(m_ZpKF, 1, str + " ApKF");
  }
}

void LocalBundleAdjustor::DebugGenerateTracks() {
  if (m_debug <= 0) {
    return;
  }
  const int nKFs = static_cast<int>(m_KFs.size());
  e_Xs.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    std::vector<Track> &Xs = e_Xs[iKF];
    Xs.resize(Nx);
    for (int ix = 0; ix < Nx; ++ix) {
      Xs[ix].Initialize();
    }
  }
  const int Nc = static_cast<int>(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
    const int NZ = static_cast<int>(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      std::vector<Track> &Xs = e_Xs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        Xs[LF.m_zs[iz].m_ix].m_zsLF.push_back(Track::MeasurementLF(ic, iz));
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      std::vector<Track> &Xs = e_Xs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        Xs[KF.m_zs[iz].m_ix].m_zsKF.push_back(Track::MeasurementKF(iKF, iz));
      }
    }
  }
  const int STL = std::min(Nc, LBA_MAX_SLIDING_TRACK_LENGTH);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      const int NzLF = static_cast<int>(X.m_zsLF.size());
      if (NzLF == 0) {
        X.m_STsLF.resize(1);
        X.m_STsLF[0].resize(0);
      } else {
        for (int i = 0; i < NzLF; ++i) {
          const int ic = X.m_zsLF[i].m_ic, ic2 = std::min(ic + STL, Nc);
          m_idxsTmp1.resize(0);
          for (int j = i; j < NzLF && X.m_zsLF[j].m_ic < ic2; ++j) {
            m_idxsTmp1.push_back(j);
          }
          if (X.m_STsLF.empty() || X.m_STsLF.back().back() < m_idxsTmp1.back()) {
            X.m_STsLF.push_back(m_idxsTmp1);
          }
        }
        const int Nst = static_cast<int>(X.m_STsLF.size());
        for (int ist = 0; ist < Nst; ++ist) {
          const std::vector<int> &ST = X.m_STsLF[ist];
          const int NzST = static_cast<int>(ST.size());
          for (int i = 0; i < NzST; ++i) {
            ++X.m_zsLF[ST[i]].m_Nst;
          }
        }
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      if (X.m_zsLF.empty()) {
        continue;
      }
      const int Nst = static_cast<int>(X.m_STsLF.size());
      UT_ASSERT(Nst == KF.CountSlidingTracks(ix));
      for (int ist = 0, iST = KF.m_ix2ST[ix]; ist < Nst; ++ist, ++iST) {
        const std::vector<int> &ST = X.m_STsLF[ist];
        const KeyFrame::SlidingTrack &_ST = KF.m_STs[iST];
        UT_ASSERT(_ST.m_icMin == X.m_zsLF[ST.front()].m_ic && _ST.m_icMax == X.m_zsLF[ST.back()].m_ic);
        const int NzST = static_cast<int>(ST.size());
        for (int i = 0; i < NzST; ++i) {
          const Track::MeasurementLF &z = X.m_zsLF[ST[i]];
          const LocalFrame::SlidingTrack &STz = m_LFs[m_ic2LF[z.m_ic]].m_STs[z.m_iz];
          UT_ASSERT(STz.m_ist1 <= ist && STz.m_ist2 > ist);
          UT_ASSERT(STz.Count() == z.m_Nst);
        }
      }
    }
  }
  for (int ic = 0; ic < Nc; ++ic) {
    const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
    const int NZ = static_cast<int>(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const std::vector<Track> &Xs = e_Xs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = LF.m_zs[iz].m_ix;
        const Track &X = Xs[ix];
        const int i = static_cast<int>(std::lower_bound(X.m_zsLF.begin(), X.m_zsLF.end(), ic) -
                                                        X.m_zsLF.begin());
        UT_ASSERT(X.m_zsLF[i].m_ic == ic);
        UT_ASSERT(X.m_zsLF[i].m_iz == iz);
        const LocalFrame::SlidingTrack &STz = LF.m_STs[iz];
        for (int ist = STz.m_ist1; ist < STz.m_ist2; ++ist) {
          const std::vector<int> &ST = X.m_STsLF[ist];
          const std::vector<int>::const_iterator j = std::lower_bound(ST.begin(), ST.end(), i);
          UT_ASSERT(j != ST.end() && *j == i);
        }
      }
    }
  }

  e_M.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_M[ic].assign(Nc, 0);
    e_M[ic][ic] = 1;
    //if (ic > 0)
    //  e_M[ic - 1][ic] = 1;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const int Nst = int(X.m_STsLF.size());
      for (int ist = 0; ist < Nst; ++ist) {
        const std::vector<int> &ST = X.m_STsLF[ist];
        const int NzST = static_cast<int>(ST.size());
        for (int i = 0; i < NzST; ++i) {
          const int ic = X.m_zsLF[ST[i]].m_ic;
          for (int j = i + 1; j < NzST; ++j) {
            e_M[ic][X.m_zsLF[ST[j]].m_ic] = 1;
          }
        }
      }
    }
  }
#ifdef LBA_FLAG_TRACK_MEASURE_KF
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const std::vector<Track> &Xs = e_Xs[iKF];
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const ubyte ud = uds[ix];
      UT_ASSERT(X.m_zsKF.empty() && !(ud & LBA_FLAG_TRACK_MEASURE_KF) ||
               !X.m_zsKF.empty() &&  (ud & LBA_FLAG_TRACK_MEASURE_KF));
    }
  }
#endif
  for (int ic1 = 0, ic2 = 1; ic2 < Nc; ic1 = ic2++) {
    e_M[ic1][ic2] = 1;
  }

  e_I.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_I[ic].assign(Nc, -1);
  }
  int i = 0;
  for (int ic = 0; ic < Nc; ++ic) {
    for (int jc = ic; jc < Nc; ++jc) {
      if (e_M[ic][jc]) {
        e_I[ic][jc] = i++;
      }
      UT_ASSERT(ic == jc || !e_M[jc][ic]);
    }
    for (int jc = ic + STL; jc < Nc; ++jc) {
      UT_ASSERT(!e_M[ic][jc]);
    }
  }
}

void LocalBundleAdjustor::DebugUpdateFactors() {
  if (m_debug <= 0) {
    return;
  }
  const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      X.m_Sadd.MakeZero();
      X.m_Sadd.m_a = add;
      const int Nst = int(X.m_STsLF.size());
      X.m_SaddsST.resize(Nst);
      for (int ist = 0; ist < Nst; ++ist) {
        X.m_SaddsST[ist].MakeZero();
        X.m_SaddsST[ist].m_a = add;
      }
    }
  }
  const int Nc = static_cast<int>(m_LFs.size()), Ncc = e_I[Nc - 1][Nc - 1] + 1;
  e_SAccs.resize(Ncc);
  for (int icc = 0; icc < Ncc; ++icc) {
    e_SAccs[icc].setZero();
  }
  e_Sbcs.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_Sbcs[ic].setZero();
  }
  e_SAcms.resize(Nc);
  e_Sbms.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_SAcms[ic].MakeZero();
    e_Sbms[ic].setZero();
  }

#if 0
  if (m_debug >= 4) {
    const int Ncmp = nLFs * pcm;
    e_A.Resize(Ncmp, Ncmp);
    e_A.MakeZero();
    e_b.Resize(Ncmp);
    e_b.MakeZero();
  }
#endif
  //e_F = 0.0f;

  DebugUpdateFactorsFeature();
  DebugUpdateFactorsPriorDepth();
  DebugUpdateFactorsPriorCameraMotion();
  DebugUpdateFactorsIMU();
  //DebugUpdateFactorsFixOrigin();
  DebugUpdateFactorsFixPositionZ();
  DebugUpdateFactorsFixMotion();

  //const float epsAbs = 1.0e-3f;
  const float epsAbs = 5.0e-3f;
  //const float epsRel = 1.0e-3f;
  //const float epsRel = 1.0e-2f;
  const float epsRel = 5.0e-2f;
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      const std::string _str = str + UT::String(" iKF %d ix %d", iKF, ix);
      X.m_Sadd.AssertEqual(KF.m_Axs[ix].m_Sadd, 1, _str + " Sadd", epsAbs, epsRel);
      X.m_Sadd = KF.m_Axs[ix].m_Sadd;
      if (X.m_zsLF.empty()) {
        continue;
      }
      const int Nst = static_cast<int>(X.m_STsLF.size());
      for (int ist = 0, iST = KF.m_ix2ST[ix]; ist < Nst; ++ist, ++iST) {
        X.m_SaddsST[ist].AssertEqual(KF.m_AxsST[iST].m_Sadd, 1,
                                     _str + UT::String(" ist %d SaddST", ist));
        X.m_SaddsST[ist] = KF.m_AxsST[iST].m_Sadd;
      }
    }
  }

#if 0
  if (m_debug >= 4) {
    e_A.SetLowerFromUpper();
  }
#endif

  for (int ic = 0; ic < Nc; ++ic) {
    const int icu = e_I[ic][ic];
    const auto e_SAcu = Camera::Factor::Unitary::CC::Get(e_SAccs[icu].GetSymmetricMatrix6x6f(),
                                                         e_Sbcs[ic].GetVector6f());
    const Camera::Factor::Unitary::CC &SAcu = m_SAcusLF[m_ic2LF[ic]];
    e_SAcu.AssertEqual(SAcu, 1, str + UT::String(" SAcc[%d][%d]", ic, ic), epsAbs, epsRel);
    e_SAccs[icu] = SAcu.m_A;
    e_Sbcs[ic] = SAcu.m_b;
    const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
    const int Nk = int(LF.m_iLFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int _iLF = LF.m_iLFsMatch[ik], _ic = (_iLF + Nc - m_ic2LF[0]) % Nc;
      UT_ASSERT(m_ic2LF[_ic] == _iLF);
      const int icb = e_I[ic][_ic];
      if (icb != -1) {
        const auto e_Acb = Camera::Factor::Binary::CC::Get(e_SAccs[icb].GetAlignedMatrix6x6f());
        e_Acb.AssertZero(1, str + UT::String(" SAcc[%d][%d]", ic, _ic));
      }
    }
  }
  for (int ic = 0; ic < Nc; ++ic) {
    const Camera::Factor &SAcm = m_SAcmsLF[m_ic2LF[ic]];
    e_SAcms[ic].AssertEqual(SAcm, 1, str + UT::String(" SAcm[%d]", ic));
    e_SAcms[ic] = SAcm;
    e_Sbms[ic].AssertEqual(SAcm.m_Au.m_Amm.m_b, 1, str + UT::String(" Sbm[%d]", ic));
    e_Sbms[ic] = SAcm.m_Au.m_Amm.m_b;
  }
  //UT::AssertEqual(e_F, m_F);
}

void LocalBundleAdjustor::DebugUpdateFactorsFeature() {
#ifdef CFG_STEREO
  const Point3D *br[2] = {NULL, &m_K.m_br};
#endif
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  std::vector<FTR::EigenFactor> e_AsST;
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D &C = m_CsKF[iKF];
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    //const int Nx = 1;
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      const FTR::Source &x = KF.m_xs[ix];
      const Depth::InverseGaussian &d = ds[ix];
      const std::string _str = str + UT::String(" iKF = %d ix = %d", iKF, ix);
      const int NzLF = static_cast<int>(X.m_zsLF.size()), NzKF = static_cast<int>(X.m_zsKF.size());
      e_AsST.resize(NzLF + NzKF);
      for (int i = 0; i < NzLF; ++i) {
        Track::MeasurementLF &z = X.m_zsLF[i];
        const int iLF = m_ic2LF[z.m_ic];
        const LocalFrame &LF = m_LFs[iLF];
        FTR::EigenFactor &A = e_AsST[i];
        const FTR::Measurement &_z = LF.m_zs[z.m_iz];
#if 0
        if (m_iIter == 2 && z.m_ic == 21 && z.m_iz == 7) {
          UT::DebugStart();
        }
#endif
        A = FTR::EigenGetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE, C, x, d, m_CsLF[iLF].m_T, _z,
                                                 false, true
#ifdef CFG_STEREO
                                               , m_K.m_br
#endif
                                               );
        A.AssertEqual(LF.m_Lzs[z.m_iz], LF.m_Azs1[z.m_iz], LF.m_Azs2[z.m_iz], 1,
                      _str + UT::String(" --> ic %d iz %d Az", z.m_ic, z.m_iz));
        A.Set(LF.m_Lzs[z.m_iz], LF.m_Azs1[z.m_iz], LF.m_Azs2[z.m_iz]);
        z.m_adcz = A.m_adcz;
        A *= (1.0f / z.m_Nst);
        z.m_adczST = A.m_adcz;
        //e_F = A.m_F + e_F;
      }
      const int Nst = int(X.m_STsLF.size());
      for (int i = 0, j = NzLF; i < NzKF; ++i, ++j) {
        Track::MeasurementKF &z = X.m_zsKF[i];
        const KeyFrame &_KF = m_KFs[z.m_iKF];
        FTR::EigenFactor &A = e_AsST[j];
        const FTR::Measurement &_z = _KF.m_zs[z.m_iz];
        A = FTR::EigenGetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE_KEY_FRAME, C, x, d,
                                                 m_CsKF[z.m_iKF], _z, false, false
#ifdef CFG_STEREO
                                               , m_K.m_br
#endif
                                               );
        A.AssertEqual(_KF.m_Azs[z.m_iz], 1,
                      _str + UT::String(" --> iKF %d iz %d Az", z.m_iKF, z.m_iz));
        A.Set(_KF.m_Azs[z.m_iz]);
        A *= (1.0f / Nst);
        //e_F = A.m_F + e_F;
      }
      for (int ist = 0; ist < Nst; ++ist) {
        FTR::EigenFactor::DD &SaddST = X.m_SaddsST[ist];
        const std::vector<int> &ST = X.m_STsLF[ist];
        const int NzST = int(ST.size());
        for (int i = 0; i < NzST; ++i) {
          const Track::MeasurementLF &z = X.m_zsLF[ST[i]];
          const FTR::EigenFactor &AST = e_AsST[ST[i]];
          const FTR::EigenFactor::DD e_addST = FTR::EigenFactor::DD::Get(AST.m_add, AST.m_bd);
//#ifdef CFG_DEBUG
#if 0
          if (iKF == 0 && ix == 0) {
            if (i == 0) {
              UT::PrintSeparator();
            }
            UT::Print("[%d] %f + %f = %f\n", m_LFs[m_ic2LF[z.m_ic]].m_T.m_iFrm,
                                             e_addST.m_a, X.m_Sadd.m_a,
                                             e_addST.m_a + X.m_Sadd.m_a);
          }
#endif
          X.m_Sadd += e_addST;
          SaddST += e_addST;
          const int iczz = e_I[z.m_ic][z.m_ic];
          e_SAccs[iczz] += AST.m_Aczz;
          e_Sbcs[z.m_ic] += AST.m_bcz;
#if 0
          if (m_debug >= 4) {
            const int icp = z.m_ic * pcm;
            e_A.block<pc, pc>(icp, icp) += AST.m_Aczz;
            e_b.block<pc, 1>(icp, 0) += AST.m_bcz;
          }
#endif
        }
        for (int i = 0, j = NzLF; i < NzKF; ++i, ++j) {
          const FTR::EigenFactor &AST = e_AsST[j];
          const FTR::EigenFactor::DD e_addST = FTR::EigenFactor::DD::Get(AST.m_add, AST.m_bd);
//#ifdef CFG_DEBUG
#if 0
          if (iKF == 0 && ix == 0) {
            UT::Print("[%d] %f + %f = %f\n", m_KFs[X.m_zsKF[i].m_iKF].m_T.m_iFrm, 
                                             e_addST.m_a, X.m_Sadd.m_a,
                                             e_addST.m_a + X.m_Sadd.m_a);
          }
#endif
          X.m_Sadd += e_addST;
          SaddST += e_addST;
        }
      }
    }
  }
}

void LocalBundleAdjustor::DebugUpdateFactorsPriorDepth() {
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
#endif
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
#ifdef CFG_STEREO
      const FTR::Source &x = KF.m_xs[ix];
      const bool xr = x.m_xr.Valid();
      if (xr) {
        const FTR::EigenFactor::Stereo e_Ard =
              FTR::EigenGetFactor<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE_KEY_FRAME, m_K.m_br, ds[ix], x);
        e_Ard.AssertEqual(KF.m_Ards[ix]);
      }
#endif
      const FTR::Factor::DD a = 
#ifdef CFG_STEREO
        xr ? KF.m_Ards[ix].m_add : 
#endif
        FTR::Factor::DD::Get(KF.m_Apds[ix]);
//#ifdef CFG_DEBUG
#if 0
      if (iKF == 0 && ix == 0)
        UT::Print("  %f + %f = %f\n", a.m_a, X.m_Sadd.m_a, a.m_a + X.m_Sadd.m_a);
#endif
      X.m_Sadd += a;
      //e_F = a.m_F + e_F;
      const int Nst = static_cast<int>(X.m_STsLF.size());
      const FTR::Factor::DD aST = a * (1.0f / Nst);
      for (int ist = 0; ist < Nst; ++ist) {
        X.m_SaddsST[ist] += aST;
      }
    }
  }
}

void LocalBundleAdjustor::DebugUpdateFactorsPriorCameraMotion() {
  CameraPrior::Motion::EigenFactor A;
  const int iLF = m_ic2LF.front();
  A = m_ZpLF.EigenGetFactor(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_CsLF[iLF]);
  A.AssertEqual(m_ApLF, 1, UT::String("LBA [%d] iIter %d Am",
                m_LFs[m_ic2LF.back()].m_T.m_iFrm, m_iIter));
  A.Set(m_ApLF);
  e_SAccs[0].block<3, 3>(3, 3) += A.m_A.block<3, 3>(0, 0);
  e_Sbcs[0].block<3, 1>(3, 0) += A.m_b.block<3, 1>(0, 0);
  Camera::EigenFactor::Unitary &SA = e_SAcms[0].m_Au;
  SA.m_Acm.block<3, 9>(3, 0) += A.m_A.block<3, 9>(0, 3);
  SA.m_Amm += A.m_A.block<9, 9>(3, 3);
  e_Sbms[0] += A.m_b.block<9, 1>(3, 0);
}

void LocalBundleAdjustor::DebugUpdateFactorsIMU() {
  IMU::Delta::EigenFactor::Global A;
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  const int nLFs = static_cast<int>(m_LFs.size());
  for (int ic1 = 0, ic2 = 1, icp1 = 0, imp1 = pc, icp2 = pcm, imp2 = pcm + pc; ic2 < nLFs;
       ic1 = ic2++, icp1 = icp2, imp1 = imp2, icp2 += pcm, imp2 += pcm) {
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
//#ifdef CFG_DEBUG
#if 0
    if (ic2 == 2) {
      UT::DebugStart();
    }
#endif
     m_DsLF[iLF2].EigenGetFactor(BA_WEIGHT_IMU, m_CsLF[iLF1], m_CsLF[iLF2], m_K.m_pu, &A,
                                 BA_ANGLE_EPSILON);
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging()) {
      UT::DebugStop();
    }
#endif
    A.AssertEqual(m_AdsLF[iLF2], m_SAcmsLF[iLF2].m_Ab, 1,
                  str + UT::String(" Ad[%d][%d]", ic1, ic2));
    A.Set(m_AdsLF[iLF2], m_SAcmsLF[iLF2].m_Ab);
    Camera::EigenFactor::Unitary &SA11 = e_SAcms[ic1].m_Au, &SA22 = e_SAcms[ic2].m_Au;
    Camera::EigenFactor::Binary &SA12 = e_SAcms[ic2].m_Ab;
    e_SAccs[e_I[ic1][ic1]] += A.m_Ac1c1;
    SA11.m_Acm += A.m_Ac1m1;
    SA12.m_Acc += A.m_Ac1c2;
    SA12.m_Acm += A.m_Ac1m2;
    e_Sbcs[ic1] += A.m_bc1;
    SA11.m_Amm += A.m_Am1m1;
    SA12.m_Amc += A.m_Am1c2;
    SA12.m_Amm += A.m_Am1m2;
    e_Sbms[ic1] += A.m_bm1;
    e_SAccs[e_I[ic2][ic2]] += A.m_Ac2c2;
    SA22.m_Acm += A.m_Ac2m2;
    e_Sbcs[ic2] += A.m_bc2;
    SA22.m_Amm += A.m_Am2m2;
    e_Sbms[ic2] += A.m_bm2;
    //e_F = A.m_F + e_F;
#if 0
    if (m_debug >= 4) {
      e_A.block<pc, pc>(icp1, icp1) += A.m_Ac1c1;
      e_A.block<pc, pm>(icp1, imp1) += A.m_Ac1m1;
      e_A.block<pc, pc>(icp1, icp2) += A.m_Ac1c2;
      e_A.block<pc, pm>(icp1, imp2) += A.m_Ac1m2;
      e_b.block<pc, 1>(icp1, 0) += A.m_bc1;
      e_A.block<pm, pm>(imp1, imp1) += A.m_Am1m1;
      e_A.block<pm, pc>(imp1, icp2) += A.m_Am1c2;
      e_A.block<pm, pm>(imp1, imp2) += A.m_Am1m2;
      e_b.block<pm, 1>(imp1, 0) += A.m_bm1;
      e_A.block<pc, pc>(icp2, icp2) += A.m_Ac2c2;
      e_A.block<pc, pm>(icp2, imp2) += A.m_Ac2m2;
      e_b.block<pc, 1>(icp2, 0) += A.m_bc2;
      e_A.block<pm, pm>(imp2, imp2) += A.m_Am2m2;
      e_b.block<pm, 1>(imp2, 0) += A.m_bm2;
    }
#endif
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 1) {
      UT::PrintSeparator();
      A.m_Ac1m2.Print(true);
    }
#endif
  }
}

void LocalBundleAdjustor::DebugUpdateFactorsFixOrigin() {
  const int iLF = m_ic2LF[0];
  if (m_LFs[iLF].m_T.m_iFrm != 0) {
    return;
  }
  Camera::Fix::Origin::EigenFactor e_A = m_Zo.EigenGetFactor(m_CsLF[iLF].m_T, BA_ANGLE_EPSILON);
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  e_A.AssertEqual(m_Ao, 1, str + "Ao");
  e_A = m_Ao;
  e_SAccs[0] += e_A.m_A;
  e_Sbcs[0] += e_A.m_b;
}

void LocalBundleAdjustor::DebugUpdateFactorsFixPositionZ() {
  const float w = UT::Inverse(BA_VARIANCE_FIX_POSITION_Z, BA_WEIGHT_FIX_POSITION_Z);
  const int Nc = static_cast<int>(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int icc = e_I[ic][ic];
    e_SAccs[icc](2, 2) += w;
    e_Sbcs[ic](2, 0) += w * m_CsLF[m_ic2LF[ic]].m_p.z();
  }
}

void LocalBundleAdjustor::DebugUpdateFactorsFixMotion() {
  const float wv = UT::Inverse(BA_VARIANCE_FIX_VELOCITY);
  const float wba = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION);
  const float wbw = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE);
  const float wv0 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INITIAL);
  const float wba0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL);
  const float wbw0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL);
  const int nLFs = int(m_LFs.size()), Ncmp = nLFs * pcm;
  for (int ic = 0, imp = pc; ic < nLFs; ++ic, imp += pcm) {
    const int iLF = m_ic2LF[ic];
    const Camera &C = m_CsLF[iLF];
    EigenMatrix9x9f &SA = e_SAcms[ic].m_Au.m_Amm;
    EigenVector9f &Sb = e_Sbms[ic];
    const float v2 = C.m_v.SquaredLength();
    const float ba2 = C.m_ba.SquaredLength();
    const float bw2 = C.m_bw.SquaredLength();
    float _wv, _wba, _wbw;
    if (m_LFs[iLF].m_T.m_iFrm == 0) {
      _wv = wv0;
      _wba = wba0;
      _wbw = wbw0;
    } else {
      _wv = ME::Weight<ME::FUNCTION_NONE>(v2 * wv) * wv;
      _wba = ME::Weight<ME::FUNCTION_NONE>(ba2 * wba) * wba;
      _wbw = ME::Weight<ME::FUNCTION_NONE>(bw2 * wbw) * wbw;
    }
    _wv *= BA_WEIGHT_FIX_MOTION;
    _wba *= BA_WEIGHT_FIX_MOTION;
    _wbw *= BA_WEIGHT_FIX_MOTION;
    const EigenMatrix3x3f Av = EigenMatrix3x3f(_wv);
    const EigenMatrix3x3f Aba = EigenMatrix3x3f(_wba);
    const EigenMatrix3x3f Abw = EigenMatrix3x3f(_wbw);
    const EigenVector3f bv = EigenVector3f(C.m_v * _wv);
    const EigenVector3f bba = EigenVector3f(C.m_ba * _wba);
    const EigenVector3f bbw = EigenVector3f(C.m_bw * _wbw);
    SA.block<3, 3>(0, 0) += Av;
    Sb.block<3, 1>(0, 0) += bv;
    SA.block<3, 3>(3, 3) += Aba;
    Sb.block<3, 1>(3, 0) += bba;
    SA.block<3, 3>(6, 6) += Abw;
    Sb.block<3, 1>(6, 0) += bbw;
    //e_F = _wv * C.m_v.SquaredLength() + _wba * C.m_ba.SquaredLength() + _wbw * C.m_bw.SquaredLength() + e_F;
#if 0
    if (m_debug >= 4) {
      e_A.block<3, 3>(imp, imp) += Av;
      e_b.block<3, 1>(imp, 0) += bv;
      e_A.block<3, 3>(imp + 3, imp + 3) += Aba;
      e_b.block<3, 1>(imp + 3, 0) += bba;
      e_A.block<3, 3>(imp + 6, imp + 6) += Abw;
      e_b.block<3, 1>(imp + 6, 0) += bbw;
    }
#endif
  }
}

void LocalBundleAdjustor::DebugUpdateSchurComplement() {
  if (m_debug <= 0) {
    return;
  }
  const int Nc = static_cast<int>(m_LFs.size()), Ncc = e_I[Nc - 1][Nc - 1] + 1;
  e_SMccs.resize(Ncc);
  for (int icc = 0; icc < Ncc; ++icc) {
    e_SMccs[icc].setZero();
  }
  e_Smcs.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_Smcs[ic].setZero();
  }
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  const float epsdST = UT::Inverse(BA_VARIANCE_MAX_DEPTH_SLIDING_TRACK, BA_WEIGHT_FEATURE, eps);
  //const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    //const int Nx = 1;
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const int Nst = static_cast<int>(X.m_STsLF.size());
      const float _epsd = KF.m_Nsts[ix] == 0 ? epsd : epsdST;
      for (int ist = 0; ist < Nst; ++ist) {
        const FTR::EigenFactor::DD &SaddST = X.m_SaddsST[ist];
        const float Sadd = SaddST.m_a/* + add*/;
        const float mdd = Sadd > _epsd ? 1.0f / Sadd : 0.0f;
        const std::vector<int> &ST = X.m_STsLF[ist];
        const int NzST = static_cast<int>(ST.size());
        for (int i1 = 0; i1 < NzST; ++i1) {
          const Track::MeasurementLF &z1 = X.m_zsLF[ST[i1]];
          const int icp1 = z1.m_ic * pcm;
          const EigenVector6f mdczT = EigenVector6f(z1.m_adczST.transpose() * mdd);
          for (int i2 = i1; i2 < NzST; ++i2) {
            const Track::MeasurementLF &z2 = X.m_zsLF[ST[i2]];
            const int iczz = e_I[z1.m_ic][z2.m_ic];
            const EigenMatrix6x6f Mczz = EigenMatrix6x6f(mdczT * z2.m_adczST);
            e_SMccs[iczz] += Mczz;
#if 0
            if (m_debug >= 4) {
              const int icp2 = z2.m_ic * pcm;
              e_A.block<pc, pc>(icp1, icp2) -= Mczz;
            }
#endif
//#ifdef CFG_DEBUG
#if 0
            if (ix == 0 && i1 == 0 && i2 == 0) {
              UT::PrintSeparator();
            }
            if (m_iIter == 0 && z1.m_ic == 1 && z2.m_ic == 1) {
              UT::Print("%d %.10e\n", z1.m_iz, e_Mccs[iczz](0, 0));
            }
#endif
          }
          const EigenVector6f mcz = EigenVector6f(mdczT * SaddST.m_b);
          e_Smcs[z1.m_ic] += mcz;
#if 0
          if (m_debug >= 4) {
            e_b.block<pc, 1>(icp1, 0) -= mcz;
          }
#endif
        }
      }
    }
  }
#if 0
  if (m_debug >= 4) {
    e_A.SetLowerFromUpper();
  }
#endif

  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  //const float epsAbs = 1.0e-3f;
  const float epsAbs = 5.0e-3f;
  //const float epsRel = 1.0e-3f;
  //const float epsRel = 1.0e-2f;
  const float epsRel = 5.0e-2f;
  for (int ic = 0; ic < Nc; ++ic) {
    const int icu = e_I[ic][ic];
    const auto e_SMcu = Camera::Factor::Unitary::CC::Get(e_SMccs[icu].GetSymmetricMatrix6x6f(),
                                                         e_Smcs[ic].GetVector6f());
    const Camera::Factor::Unitary::CC &SMcu = m_SMcusLF[m_ic2LF[ic]];
    e_SMcu.AssertEqual(SMcu, 1, str + UT::String(" SMcc[%d][%d]", ic, ic), epsAbs, epsRel);
    e_SMccs[icu] = SMcu.m_A;
    e_Smcs[ic] = SMcu.m_b;
    const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
    const int Nk = static_cast<int>(LF.m_iLFsMatch.size());
    for (int ik = 0; ik < Nk; ++ik) {
      const int _iLF = LF.m_iLFsMatch[ik], _ic = (_iLF + Nc - m_ic2LF[0]) % Nc;
      UT_ASSERT(m_ic2LF[_ic] == _iLF);
      const int icb = e_I[ic][_ic];
      const Camera::Factor::Binary::CC &SMcb = LF.m_Zm.m_SMczms[ik];
      const std::string _str = str + UT::String(" SMcc[%d][%d]", ic, _ic);
      if (icb == -1) {
        SMcb.AssertZero(1, _str);
        continue;
      }
      const auto e_SMcb = Camera::Factor::Binary::CC::Get(e_SMccs[icb].GetAlignedMatrix6x6f());
      e_SMcb.AssertEqual(SMcb, 1, _str, epsAbs, epsRel);
      e_SMccs[icb] = SMcb;
    }
  }
#if 0
//#if 1
  AssertConsistency(false);
#endif
}

#if 0
static inline void SolveLDL(const int N, float **A, float *b1, float *b2) {
  LA::LS::DecomposeLDL(N, A);
  for (int i = 0; i < N; ++i) {
    const float *ai = A[i];
    for (int j = i + 1; j < N; ++j) {
      b1[j] -= ai[j] * b1[i];
      b2[j] -= ai[j] * b2[i];
      UT::AssertEqual(b1[j], b2[j]);
    }
    b1[i] *= ai[i];
    b2[i] *= ai[i];
    UT::AssertEqual(b1[i], b2[i]);
  }
  for (int i = N - 2; i >= 0; --i) {
    const float *ai = A[i];
    for (int j = i + 1; j < N; ++j) {
      b1[i] -= ai[j] * b1[j];
      b2[i] -= ai[j] * b2[j];
      UT::AssertEqual(b1[i], b2[i]);
    }
  }
}
#endif

void LocalBundleAdjustor::DebugSolveSchurComplement() {
  if (m_debug <= 0) {
    return;
  }
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  const int Nc = int(m_LFs.size());
  e_xcs.resize(Nc);
  e_xms.resize(Nc);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsGN.Data();
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_xcs[ic] = EigenVector6f(xcs[ic]);
    e_xms[ic] = EigenVector9f(xms[ic]);
    //e_xcs[ic] = EigenVector6f(-e_xcs[ic]);
    //e_xms[ic] = EigenVector9f(-e_xms[ic]);
  }
  if (m_debug < 2) {
    return;
  }
  EigenVector6f e_Sbc;
  EigenVector9f e_Sbm;
  const float e2Max = ME::ChiSquareDistance<3>(BA_PCG_MIN_CONVERGE_PROBABILITY, BA_WEIGHT_FEATURE);
  for (int ic1 = 0; ic1 < Nc; ++ic1) {
    e_Sbc.setZero();
    e_Sbm.setZero();
    {
      const int ic2 = ic1;
      const int icc = e_I[ic1][ic2];
      e_Sbc = (e_SAccs[icc] - e_SMccs[icc]) * e_xcs[ic2];
    }
    for (int ic0 = 0; ic0 < ic1; ++ic0) {
      const int icc = e_I[ic0][ic1];
      if (icc == -1) {
        continue;
      }
      EigenMatrix6x6f e_A = EigenMatrix6x6f(e_SAccs[icc] - e_SMccs[icc]);
      if (ic0 == ic1 - 1) {
        e_A += e_SAcms[ic1].m_Ab.m_Acc;
      }
      e_Sbc = EigenVector6f(e_A.transpose() * e_xcs[ic0] + e_Sbc);
    }
    //for (int ic2 = ic1; ic2 < Nc; ++ic2) {
    for (int ic2 = ic1 + 1; ic2 < Nc; ++ic2) {
      const int icc = e_I[ic1][ic2];
      if (icc == -1) {
        continue;
      }
      EigenMatrix6x6f e_A = EigenMatrix6x6f(e_SAccs[icc] - e_SMccs[icc]);
      if (ic2 == ic1 + 1) {
        e_A += e_SAcms[ic2].m_Ab.m_Acc;
      }
      e_Sbc = EigenVector6f(e_A * e_xcs[ic2] + e_Sbc);
      //LA::AlignedVector6f b;
      //LA::AlignedMatrix6x6f::Ab(e_A.GetAlignedMatrix6x6f(),
      //                          LA::ProductVector6f::Get(e_xcs[ic2].GetAlignedVector6f()),
      //                          b);
      //e_Sbc = EigenVector6f(b) + e_Sbc;
//#ifdef CFG_DEBUG
#if 0
      if (ic1 == 1) {
        UT::PrintSeparator();
        e_SAcms[ic2].m_Ab.m_Acc.Print();
        UT::PrintSeparator();
        EigenMatrix6x6f(e_SAccs[icc] - e_SMccs[icc]).Print();
        UT::PrintSeparator();
        e_A.Print();
        UT::PrintSeparator();
        e_xcs[ic2].Print();
        UT::PrintSeparator();
        //EigenVector6f(e_A * e_xcs[ic2]).Print();
        b.Print();
      }
#endif
    }
    const int ic0 = ic1 - 1;
    if (ic0 >= 0) {
      const Camera::EigenFactor::Binary &SAcmb = e_SAcms[ic1].m_Ab;
      //e_Sbc = EigenVector6f(SAcmb.m_Acc.transpose() * e_xcs[ic0] + e_Sbc);
      e_Sbc = EigenVector6f(SAcmb.m_Amc.transpose() * e_xms[ic0] + e_Sbc);
      e_Sbm = EigenVector9f(SAcmb.m_Acm.transpose() * e_xcs[ic0] + e_Sbm);
      e_Sbm = EigenVector9f(SAcmb.m_Amm.transpose() * e_xms[ic0] + e_Sbm);
    }
    const Camera::EigenFactor::Unitary &SAcmu = e_SAcms[ic1].m_Au;
    e_Sbc = EigenVector6f(SAcmu.m_Acm * e_xms[ic1] + e_Sbc);
    e_Sbm = EigenVector9f(SAcmu.m_Acm.transpose() * e_xcs[ic1] + e_Sbm);
    e_Sbm = EigenVector9f(SAcmu.m_Amm * e_xms[ic1] + e_Sbm);
    const int ic2 = ic1 + 1;
    if (ic2 < Nc) {
      const Camera::EigenFactor::Binary &SAcmb = e_SAcms[ic2].m_Ab;
      //e_Sbc = EigenVector6f(SAcmb.m_Acc * e_xcs[ic2] + e_Sbc);
      e_Sbc = EigenVector6f(SAcmb.m_Acm * e_xms[ic2] + e_Sbc);
      e_Sbm = EigenVector9f(SAcmb.m_Amc * e_xcs[ic2] + e_Sbm);
      e_Sbm = EigenVector9f(SAcmb.m_Amm * e_xms[ic2] + e_Sbm);
    }
    e_Sbc = -e_Sbc;
    const int iLF1 = m_ic2LF[ic1];
    const EigenMatrix6x6f e_Mc = m_Mcs[ic1].GetAlignedMatrix6x6f();
#ifdef CFG_DEBUG
    const EigenVector6f e_Sbc_ = EigenVector6f(e_Sbcs[ic1] - e_Smcs[ic1]);
#endif
    const EigenVector6f e_rc = EigenVector6f((e_Sbcs[ic1] - e_Smcs[ic1]) - e_Sbc);
    const EigenVector6f e_zc = EigenVector6f(e_Mc * e_rc);
    const float e_e2p = e_rc.block<3, 1>(0, 0).dot(e_zc.block<3, 1>(0, 0));
    const float e_e2r = e_rc.block<3, 1>(3, 0).dot(e_zc.block<3, 1>(3, 0));
    if (e2Max == 0.0f) {
      UT_ASSERT(UT::AssertZero(e_e2p) && UT::AssertZero(e_e2r));
    } else {
      UT_ASSERT(e_e2p < e2Max && e_e2r < e2Max);
    }
    e_Sbm = -e_Sbm;
    const EigenMatrix9x9f e_Mm = m_Mms[ic1].GetAlignedMatrix9x9f().GetMatrix9x9f();
    const EigenVector9f e_rm = EigenVector9f(e_Sbms[ic1] - e_Sbm);
    const EigenVector9f e_zm = EigenVector9f(e_Mm * e_rm);
    const float e_e2v = e_rm.block<3, 1>(0, 0).dot(e_zm.block<3, 1>(0, 0));
    const float e_e2ba = e_rm.block<3, 1>(3, 0).dot(e_zm.block<3, 1>(3, 0));
    const float e_e2bw = e_rm.block<3, 1>(6, 0).dot(e_zm.block<3, 1>(6, 0));
    if (e2Max == 0.0f) {
      UT_ASSERT(UT::AssertZero(e_e2v) && UT::AssertZero(e_e2ba) && UT::AssertZero(e_e2bw));
    } else {
      UT_ASSERT(e_e2v < e2Max && e_e2ba < e2Max && e_e2bw < e2Max);
    }
  }
}

#ifdef CFG_DEBUG
static inline void PrintBackSubstitution(const int iFrm, const LA::Vector6f &mdc,
                                         const LA::Vector6f &x1, const float xd1, const float Sxd1,
                                         const LA::Vector6f &x2, const float xd2, const float Sxd2
                                         ) {
  UT::PrintSeparator();
  UT::Print("[%d]\n", iFrm);
  mdc.Print("mdc = ", false, true);
  x1.Print("xc1 = ", false, false); UT::Print(" --> %f\n", mdc.Dot(x1));
  x2.Print("xc2 = ", false, false); UT::Print(" --> %f\n", mdc.Dot(x2));
  const LA::Vector6f dx = x1 - x2;
  dx.Print("dxc = ", false, false); UT::Print(" --> %f\n", mdc.Dot(dx));
  const LA::Vector3f dp = dx.Get012(), dr = dx.Get345();
  const float dp2 = dp.SquaredLength(), dr2 = dr.SquaredLength();
  UT_ASSERT(dp2 <= BA_BACK_SUBSTITUTE_POSITION && dr2 <= BA_BACK_SUBSTITUTE_ROTATION);
  UT::Print("  --> %f %f --> %f %f\n", sqrtf(dp2), sqrtf(dr2) * UT_FACTOR_RAD_TO_DEG,
                                       mdc.Get012().Dot(dp), mdc.Get345().Dot(dr));
  UT::Print(" xd = %f - %f = %f\n", xd1, xd2, xd1 - xd2);
  UT::Print("Sxd = %f - %f = %f\n", Sxd1, Sxd2, Sxd1 - Sxd2);
}
#endif

void LocalBundleAdjustor::DebugSolveBackSubstitution() {
  if (m_debug <= 0) {
    return;
  }
  //const float epsAbs = 1.0e-5f;
  const float epsAbs = BA_UPDATE_DEPTH;
  //const float epsRel = 1.0e-3f;
  const float epsRel = 1.0e-2f;
  const int nLFs = static_cast<int>(m_LFs.size()), nKFs = static_cast<int>(m_KFs.size());
#if 0
  e_xcs.resize(nLFs);
  e_xks.resize(nKFs);
  //for(int ic = 0; ic < nLFs; ++ic)
  //  e_xcs[ic] = EigenVector6f(m_SxcsLF[m_ic2LF[ic]]);
  //for(int iKF = 0; iKF < nKFs; ++iKF)
  //  e_xks[iKF] = EigenVector6f(m_SxcsKF[iKF]);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xcs.Data(), *xks = (LA::Vector6f *) m_xks.Data();
  for (int ic = 0; ic < nLFs; ++ic) {
    if (m_ucsLF[m_ic2LF[ic]] & LBA_FLAG_FRAME_UPDATE_DELTA) {
      e_xcs[ic] = EigenVector6f(xcs[ic]);
    } else {
      e_xcs[ic].setZero();
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int ik = m_iKF2k[iKF];
    if (ik != -1 && (m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DELTA)) {
      e_xks[iKF] = EigenVector6f(xks[ik]);
    } else {
      e_xks[iKF].setZero();
    }
  }
#endif

  e_xds.Resize(0);
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  //const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  for (int iKF = 0, iX = 0; iKF < nKFs; ++iKF) {
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    float *xds = NULL;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const bool exist = UT::VectorExistFlag<ubyte>(uds, Nx, LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION);
    if (m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION) {
      UT_ASSERT(exist);
      xds = m_xds.Data() + iX;
      iX += Nx;
    } else {
      UT_ASSERT(!exist);
      continue;
    }
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        e_xds.Push(0.0f);
        continue;
      }
      const Track &X = Xs[ix];
      const float Sadd = X.m_Sadd.m_a/* + add*/, mdd = Sadd > epsd ? 1.0f / Sadd : 0.0f;
      float xd = X.m_Sadd.m_b * mdd, Sxd = -xd;
      const int NzLF = static_cast<int>(X.m_zsLF.size());
      for (int i = 0; i < NzLF; ++i) {
        const Track::MeasurementLF &z = X.m_zsLF[i];
        if (m_ucsLF[m_ic2LF[z.m_ic]] & LBA_FLAG_FRAME_UPDATE_DELTA) {
          xd = z.m_adcz.dot(e_xcs[z.m_ic]) * mdd;
          Sxd = -xd + Sxd;
        }
      }
      UT::AssertEqual(Sxd, xds ? xds[ix] : 0.0f, 1, str + UT::String(" iKF %d ix %d xd", iKF, ix),
                      epsAbs, epsRel);
#ifdef CFG_DEBUG
      if (UT::Debugging()) {
        UT::DebugStop();
      }
#endif
      e_xds.Push(xds[ix]);
    }
  }
}

void LocalBundleAdjustor::DebugSolveGradientDescent() {
  if (m_debug <= 0) {
    return;
  }
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  const float s = m_bl == 0.0f ? 0.0f : 1.0f / m_bl;
  const int Nc = int(m_LFs.size());
  e_gcs.resize(Nc);
  e_gms.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_gcs[ic] = e_Sbcs[ic] * s;
    e_gms[ic] = e_Sbms[ic] * s;
  }
  e_Agcs.resize(Nc);
  e_Agms.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    const EigenVector6f &e_gc = e_gcs[ic];
    const EigenVector9f &e_gm = e_gms[ic];
    const Camera::EigenFactor::Unitary &e_A = e_SAcms[ic].m_Au;
    e_Agcs[ic] = EigenVector6f(e_A.m_Acm * e_gm + e_SAccs[e_I[ic][ic]] * e_gc);
    e_Agms[ic] = EigenVector9f(e_A.m_Amm * e_gm + e_A.m_Acm.transpose() * e_gc);
  }
  for (int ic1 = 0, ic2 = 1; ic2 < Nc; ic1 = ic2++) {
    const Camera::EigenFactor::Binary &e_A = e_SAcms[ic2].m_Ab;
    const EigenVector6f &e_gc1 = e_gcs[ic1], &e_gc2 = e_gcs[ic2];
    const EigenVector9f &e_gm1 = e_gms[ic1], &e_gm2 = e_gms[ic2];
    EigenVector6f &e_Agc1 = e_Agcs[ic1], &e_Agc2 = e_Agcs[ic2];
    EigenVector9f &e_Agm1 = e_Agms[ic1], &e_Agm2 = e_Agms[ic2];
    e_Agc1 = EigenVector6f(e_A.m_Acc * e_gc2 + e_Agc1);
    e_Agc2 = EigenVector6f(e_A.m_Acc.transpose() * e_gc1 + e_Agc2);
    e_Agc1 = EigenVector6f(e_A.m_Acm * e_gm2 + e_Agc1);
    e_Agm1 = EigenVector9f(e_A.m_Amm.block<9, 3>(0, 0) * e_gm2.block<3, 1>(0, 0) + e_Agm1);
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 1 && ic1 == 10) {
      e_Agm1.Print();
    }
#endif
    e_Agm2 = EigenVector9f(e_A.m_Acm.transpose() * e_gc1 + e_Agm2);
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 1 && ic2 == 10) {
      e_Agm2.Print();
    }
#endif
    e_Agm1 = EigenVector9f(e_A.m_Amc * e_gc2 + e_Agm1);
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 1 && ic1 == 10) {
      e_Agm1.Print();
    }
#endif
    e_Agc2 = EigenVector6f(e_A.m_Amc.transpose() * e_gm1 + e_Agc2);
    //e_Agm1 = EigenVector9f(e_A.m_Amm * e_gm2 + e_Agm1);
    e_Agm1 = EigenVector9f(e_A.m_Amm.block<9, 6>(0, 3) * e_gm2.block<6, 1>(3, 0) + e_Agm1);
    e_Agm2 = EigenVector9f(e_A.m_Amm.transpose() * e_gm1 + e_Agm2);
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 1) {
      if (ic1 == 10) {
        e_Agm1.Print();
      }
      if (ic2 == 10) {
        e_Agm2.Print();
      }
    }
#endif
  }
  e_gds.Resize(0);
  e_Agds.Resize(0);
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const FTR::EigenFactor::DD &e_Sadd = Xs[ix].m_Sadd;
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        const float e_gd = e_Sadd.m_b * s;
        const float e_Agd = e_Sadd.m_a * e_gd;
        e_gds.Push(e_gd);
        e_Agds.Push(e_Agd);
      } else {
        e_gds.Push(0.0f);
        e_Agds.Push(0.0f);
      }
    }
  }
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agd Start\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    float *gds1 = e_gds.Data() + iX, *Agds1 = e_Agds.Data() + iX;
    const float *gds2 = m_gds.Data() + iX, *Agds2 = m_Agds.Data() + iX;
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        continue;
      }
      UT::AssertEqual(gds1[ix], gds2[ix], 1, str + UT::String(" iKF %d ix %d gd", iKF, ix));
      gds1[ix] = gds2[ix];
      const float e_gd = gds1[ix];
      float &e_Agd = Agds1[ix];
      const Track &X = Xs[ix];
      const int NzLF = static_cast<int>(X.m_zsLF.size());
      for (int i = 0; i < NzLF; ++i) {
        const Track::MeasurementLF &z = X.m_zsLF[i];
        const EigenVector6f e_adcz = EigenVector6f(z.m_adcz.transpose());
        e_Agd = e_adcz.dot(e_gcs[z.m_ic]) + e_Agd;
        e_Agcs[z.m_ic] = EigenVector6f(e_adcz * e_gd + e_Agcs[z.m_ic]);
      }
      UT::AssertEqual(e_Agd, Agds2[ix], 1, str + UT::String(" iKF %d ix %d Agd", iKF, ix));
      e_Agd = Agds2[ix];
    }
  }
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agd Stop\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agc Start\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
  const LA::Vector6f *Agcs = (LA::Vector6f *) m_Ags.Data();
  const LA::Vector9f *Agms = (LA::Vector9f *) (Agcs + Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    e_Agcs[ic].GetVector6f().AssertEqual(Agcs[ic], 1, str + UT::String(" Agc[%d]", ic));
    e_Agcs[ic] = Agcs[ic];
  }
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agc Stop\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agm Start\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
  for (int ic = 0; ic < Nc; ++ic) {
    e_Agms[ic].GetVector9f().AssertEqual(Agms[ic], 1, str + UT::String(" Agm[%d]", ic));
    e_Agms[ic] = Agms[ic];
  }
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert Agm Stop\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif

  float gTgc = 0.0f, gTAgc = 0.0f;
  float gTgm = 0.0f, gTAgm = 0.0f;
  for (int ic = 0; ic < Nc; ++ic) {
    gTgc = e_gcs[ic].SquaredLength() + gTgc;
    gTAgc = e_gcs[ic].dot(e_Agcs[ic]) + gTAgc;
  }
  for (int ic = 0; ic < Nc; ++ic) {
    gTgm = e_gms[ic].SquaredLength() + gTgm;
    gTAgm = e_gms[ic].dot(e_Agms[ic]) + gTAgm;
  }
  const float gTgd = e_gds.SquaredLength(), gTAgd = e_gds.Dot(e_Agds);
  const float gTg = gTgc + gTgm + gTgd;
  const float gTAg = gTAgc + gTAgm + gTAgd;
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert gTg Start\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
  UT::AssertEqual(gTg, 1.0f, 1, str + " gTg");
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert gTg Stop\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert m_gTAg Start\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
  //UT::Print("\r[%d] %e %e %e %e %e\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm,
  //  gTAgc, gTAgm, gTAgk, gTAgd, gTAg);
  UT::Print("\r[%d] %e %e\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm, gTgd, gTAgd);
#endif
  UT::AssertEqual(gTAg, m_gTAg, 1, str + " gTAg");
#ifdef LBA_DEBUG_PRINT_STEP
  UT::Print("\r[%d] Assert m_gTAg Stop\t\t\t", m_LFs[m_ic2LF.back()].m_T.m_iFrm);
#endif
}

void LocalBundleAdjustor::DebugComputeReduction() {
  if (m_debug <= 0) {
    return;
  }
  const int Nc = int(m_LFs.size());
  e_xcs.resize(Nc);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data();
  for (int ic = 0; ic < Nc; ++ic) {
    if (m_ucsLF[m_ic2LF[ic]] & LBA_FLAG_FRAME_UPDATE_CAMERA) {
      e_xcs[ic] = EigenVector6f(xcs[ic]);
    } else {
      e_xcs[ic].setZero();
    }
  }
  e_xms.resize(Nc);
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    const LA::Vector9f &xm = xms[ic];
    EigenVector9f &e_xm = e_xms[ic];
    const ubyte ucm = m_ucmsLF[m_ic2LF[ic]];
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
      e_xm.block<3, 1>(0, 0) = EigenVector3f(&xm.v0());
    } else {
      e_xm.block<3, 1>(0, 0).setZero();
    }
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
      e_xm.block<3, 1>(3, 0) = EigenVector3f(&xm.v3());
    } else {
      e_xm.block<3, 1>(3, 0).setZero();
    }
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
      e_xm.block<3, 1>(6, 0) = EigenVector3f(&xm.v6());
    } else {
      e_xm.block<3, 1>(6, 0).setZero();
    }
  }
  e_xds.Resize(int(m_ds.size()));
  e_xds.MakeZero();
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const int id = m_iKF2d[iKF];
    const ubyte *uds = m_uds.data() + id;
    const float *xds1 = m_xds.Data() + iX;
    float *xds2 = e_xds.Data() + id;
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) {
        xds2[ix] = xds1[ix];
      }
    }
  }
  e_dFp = 0.0f;
  DebugComputeReductionFeature();
  DebugComputeReductionPriorDepth();
  DebugComputeReductionPriorCameraMotion();
  DebugComputeReductionIMU();
  //DebugComputeReductionFixOrigin();
  DebugComputeReductionFixPositionZ();
  DebugComputeReductionFixMotion();
  const int iFrm = m_LFs[m_ic2LF.back()].m_T.m_iFrm;
  const std::string str = UT::String("LBA [%d] iIter %d", iFrm, m_iIter);
  const float eps = 1.0e-3f;
  UT::AssertEqual(e_dFp, m_dFp, 1, str + UT::String(" iIterDL %d dFp", m_iIterDL), eps);
}

void LocalBundleAdjustor::DebugComputeReductionFeature() {
  const int nLFs = int(m_LFs.size()), nKFs = int(m_KFs.size());
  m_work.Resize(nLFs + nKFs);
  LA::AlignedVectorXf dFpsLF(m_work.Data(), nLFs, false);
  LA::AlignedVectorXf dFpsKF(dFpsLF.BindNext(), nKFs, false);
  dFpsLF.MakeZero();
  dFpsKF.MakeZero();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D &C = m_CsKF[iKF];
    const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
    const Depth::InverseGaussian *ds = iX == -1 ? m_ds.data() + id : m_dsBkp.data() + iX;
    const float *xds = e_xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const FTR::Source &x = KF.m_xs[ix];
      const Depth::InverseGaussian &d = ds[ix];
      const float xd = xds[ix];
      const int NzLF = static_cast<int>(X.m_zsLF.size()), NzKF = static_cast<int>(X.m_zsKF.size());
      for (int i = 0; i < NzLF; ++i) {
        const Track::MeasurementLF &z = X.m_zsLF[i];
        const int iLF = m_ic2LF[z.m_ic], iz = z.m_iz;
        const LocalFrame &LF = m_LFs[iLF];
        const float F = FTR::EigenGetCost<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE, C, x, d,
                                                           m_CsLFBkp[iLF].m_T, LF.m_zs[iz],
                                                           NULL, &e_xcs[z.m_ic], xd
#ifdef CFG_STEREO
                                                         , m_K.m_br
#endif
                                                         );
        const float dF = LF.m_Lzs[iz].m_F - F;
        dFpsLF[z.m_ic] = dF + dFpsLF[z.m_ic];
//#ifdef CFG_DEBUG
#if 0
        if (m_iIter == 2 && z.m_ic == 12 && dFpsLF[z.m_ic] != 0.0f) {
          UT::Print("iKF = %d, ix = %d, iz = %d, dFp = %f\n", iKF, ix, z.m_iz, dFpsLF[z.m_ic]);
        }
#endif
      }
      for (int i = 0, j = NzLF; i < NzKF; ++i, ++j) {
        const Track::MeasurementKF &z = X.m_zsKF[i];
        const int _iKF = z.m_iKF, iz = z.m_iz;
        const KeyFrame &_KF = m_KFs[_iKF];
        const float F = FTR::EigenGetCost<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE_KEY_FRAME, C, x, d,
                                                           m_CsKF[_iKF], _KF.m_zs[iz], NULL, NULL, xd
#ifdef CFG_STEREO
                                                         , m_K.m_br
#endif
                                                         );
        const float dF = _KF.m_Azs[iz].m_F - F;
        dFpsKF[_iKF] = dF + dFpsKF[_iKF];
      }
    }
  }
  for (int ic = 0; ic < nLFs; ++ic) {
    e_dFp = dFpsLF[ic] + e_dFp;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    e_dFp = dFpsKF[iKF] + e_dFp;
  }
//#ifdef CFG_DEBUG
#if 0
  if (m_iIter == 2) {
    for (int ic = 0; ic < nLFs; ++ic) {
      UT::Print("[%d] %f\n", m_LFs[m_ic2LF[ic]].m_T.m_iFrm, dFpsLF[ic]);
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      UT::Print("[%d] %f\n", m_KFs[iKF].m_T.m_iFrm, dFpsKF[iKF]);
    }
  }
#endif
}

void LocalBundleAdjustor::DebugComputeReductionPriorDepth() {
  Depth::Prior::Reduction Ra, Rp;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
    const Depth::InverseGaussian *ds = iX == -1 ? m_ds.data() + id : m_dsBkp.data() + iX;
    const float *xds = e_xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      const FTR::Source &x = KF.m_xs[ix];
      if (x.m_xr.Valid()) {
        const float F = FTR::EigenGetCost<LBA_ME_FUNCTION>(BA_WEIGHT_FEATURE_KEY_FRAME, m_K.m_br,
                                                           ds[ix], x, xds[ix]);
        e_dFp = KF.m_Ards[ix].m_F - F + e_dFp;
      } else
#endif
      {
        zp.GetReduction(KF.m_Apds[ix], ds[ix].u(), xds[ix], Ra, Rp);
        e_dFp = Rp.m_dF + e_dFp;
      }
    }
  }
}

void LocalBundleAdjustor::DebugComputeReductionPriorCameraMotion() {
  const EigenVector3f e_xr(e_xcs[0].block<3, 1>(3, 0));
  const float F = m_ZpLF.EigenGetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_CsLFBkp[m_ic2LF.front()],
                                      e_xr, e_xms[0]);
  const float dF = m_ApLF.m_F - F;
  e_dFp = dF + e_dFp;
}

void LocalBundleAdjustor::DebugComputeReductionIMU() {
  const int nLFs = int(m_LFs.size());
  for (int ic1 = 0, ic2 = 1; ic2 < nLFs; ic1 = ic2++) {
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    const float F = m_DsLF[iLF2].EigenGetCost(BA_WEIGHT_IMU, m_CsLFBkp[iLF1], m_CsLFBkp[iLF2],
                                              m_K.m_pu, e_xcs[ic1], e_xms[ic1], e_xcs[ic2],
                                              e_xms[ic2], BA_ANGLE_EPSILON);
    const float dF = m_AdsLF[iLF2].m_F - F;
//#ifdef CFG_DEBUG
#if 0
    UT::Print("[%d][%d]: %f + %f = %f\n", ic1, ic2, dF, e_dFp, dF + e_dFp);
#endif
    e_dFp = dF + e_dFp;
  }
}

void LocalBundleAdjustor::DebugComputeReductionFixOrigin() {
  const int iLF = m_ic2LF[0];
  if (m_LFs[iLF].m_T.m_iFrm != 0) {
    return;
  }
  const float F = m_Zo.EigenGetCost(m_CsLFBkp[iLF].m_T, e_xcs[0], BA_ANGLE_EPSILON);
  const float dF = m_Ao.m_F - F;
  e_dFp = dF + e_dFp;
}

void LocalBundleAdjustor::DebugComputeReductionFixPositionZ() {
  const float w = UT::Inverse(BA_VARIANCE_FIX_POSITION_Z, BA_WEIGHT_FIX_POSITION_Z);
  const int nKFs = static_cast<int>(m_KFs.size());
  const int Nc = int(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const float pz1 = m_CsLFBkp[iLF].m_p.z();
    const float pz2 = pz1 + e_xcs[ic](2, 0);
    const float F = w * pz2 * pz2;
    const float dF = F - m_AfpsLF[iLF].m_F;
    e_dFp = dF + e_dFp;
  }
}

void LocalBundleAdjustor::DebugComputeReductionFixMotion() {
  const float wv = UT::Inverse(BA_VARIANCE_FIX_VELOCITY);
  const float wba = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION);
  const float wbw = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE);
  const float wv0 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INITIAL);
  const float wba0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL);
  const float wbw0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL);
  const int Nc = int(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const Camera &C = m_CsLFBkp[iLF];
    const float v2 = C.m_v.SquaredLength();
    const float ba2 = C.m_ba.SquaredLength();
    const float bw2 = C.m_bw.SquaredLength();
    float _wv, _wba, _wbw;
    if (m_LFs[iLF].m_T.m_iFrm == 0) {
      _wv = wv0;
      _wba = wba0;
      _wbw = wbw0;
    } else {
      _wv = ME::Weight<LBA_ME_FUNCTION>(v2 * wv) * wv;
      _wba = ME::Weight<LBA_ME_FUNCTION>(ba2 * wba) * wba;
      _wbw = ME::Weight<LBA_ME_FUNCTION>(bw2 * wbw) * wbw;
    }
    _wv *= BA_WEIGHT_FIX_MOTION;
    _wba *= BA_WEIGHT_FIX_MOTION;
    _wbw *= BA_WEIGHT_FIX_MOTION;
    const EigenVector9f &e_xm = e_xms[ic];
    const Camera::Fix::Motion::Factor &A = m_AfmsLF[iLF];
    const EigenVector3f ev = EigenVector3f(EigenVector3f(C.m_v) + e_xm.block<3, 1>(0, 0));
    const EigenVector3f eba = EigenVector3f(EigenVector3f(C.m_ba) + e_xm.block<3, 1>(3, 0));
    const EigenVector3f ebw = EigenVector3f(EigenVector3f(C.m_bw) + e_xm.block<3, 1>(6, 0));
    const float dFpv = A.m_Av.F() - _wv * ev.squaredNorm();
    const float dFpba = A.m_Aba.F() - _wba * eba.squaredNorm();
    const float dFpbw = A.m_Abw.F() - _wbw * ebw.squaredNorm();
    e_dFp = dFpv + e_dFp;
    e_dFp = dFpba + e_dFp;
    e_dFp = dFpbw + e_dFp;
  }
}
#endif
