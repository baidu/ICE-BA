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

#ifdef CFG_DEBUG_EIGEN
void GlobalBundleAdjustor::DebugGenerateTracks() {
  if (m_debug <= 0) {
    return;
  }
  const int nKFs = static_cast<int>(m_KFs.size());
  e_Xs.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    std::vector<Track> &Xs = e_Xs[iKF];
    Xs.resize(Nx);
    for (int ix = 0; ix < Nx; ++ix) {
      Xs[ix].Initialize();
    }
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      UT_ASSERT(Z.m_iKF < iKF);
      std::vector<Track> &Xs = e_Xs[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        Xs[KF.m_zs[iz].m_ix].m_zs.push_back(Track::Measurement(iKF, iz));
      }
    }
  }
#ifdef GBA_FLAG_TRACK_MEASURE
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const std::vector<Track> &Xs = e_Xs[iKF];
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const ubyte ud = uds[ix];
      UT_ASSERT((X.m_zs.empty() && !(ud & GBA_FLAG_TRACK_MEASURE)) ||
                (!X.m_zs.empty() && (ud & GBA_FLAG_TRACK_MEASURE)));
    }
  }
#endif

  e_M.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    e_M[iKF].assign(nKFs, 0);
    e_M[iKF][iKF] = 1;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const int Nz = static_cast<int>(X.m_zs.size());
      for (int i = 0; i < Nz; ++i) {
        e_M[iKF][X.m_zs[i].m_iKF] = 1;
      }
      for (int i = 0; i < Nz; ++i) {
        const int _iKF = X.m_zs[i].m_iKF;
        for (int j = i + 1; j < Nz; ++j) {
          e_M[_iKF][X.m_zs[j].m_iKF] = 1;
        }
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int Np = static_cast<int>(KF.m_iKFsPrior.size());
    for (int ip = 0; ip < Np; ++ip) {
      e_M[KF.m_iKFsPrior[ip]][iKF] = 1;
    }
  }
  for (int iKF = nKFs - m_CsLM.Size() + 1; iKF < nKFs; ++iKF) {
    if (!m_KFs[iKF].m_us.Empty()) {
      e_M[iKF - 1][iKF] = 1;
    }
  }
  e_I.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    e_I[iKF].assign(nKFs, -1);
  }
  for (int iKF = 0/*, i = 0*/; iKF < nKFs; ++iKF) {
    for (int jKF = iKF; jKF < nKFs; ++jKF) {
      //if (e_M[iKF][jKF]) {
      //  e_I[iKF][jKF] = i++;
      //}
      UT_ASSERT(iKF == jKF || !e_M[jKF][iKF]);
    }
  }
  for (int iKF = 0, i = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const int Nkp = static_cast<int>(KF.m_ikp2KF.size());
    for (int ikp = 0; ikp < Nkp; ++ikp) {
      const int _iKF = KF.m_ikp2KF[ikp];
      //UT_ASSERT(e_M[_iKF][iKF] != 0);
      e_M[_iKF][iKF] = 0;
      e_I[_iKF][iKF] = i++;
    }
    e_M[iKF][iKF] = 0;
    e_I[iKF][iKF] = i++;
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    for (int jKF = 0; jKF < nKFs; ++jKF) {
      UT_ASSERT(e_M[iKF][jKF] == 0);
    }
  }
}

void GlobalBundleAdjustor::DebugUpdateFactors() {
  if (m_debug <= 0) {
    return;
  }
  const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Xs[ix].m_Sadx.MakeZero();
      Xs[ix].m_Sadx.m_add.m_a = add;
    }
  }
  const int Ncc = e_I[nKFs - 1][nKFs - 1] + 1;
  e_SAccs.resize(Ncc);
  for (int icc = 0; icc < Ncc; ++icc) {
    e_SAccs[icc].setZero();
  }
  e_Sbcs.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    e_Sbcs[iKF].setZero();
  }
  const int Nm = m_CsLM.Size();
  e_SAcmsLM.resize(Nm);
  e_Sbms.resize(Nm);
  for (int im = 0; im < Nm; ++im) {
    e_SAcmsLM[im].MakeZero();
    e_Sbms[im].setZero();
  }
  DebugUpdateFactorsFeature();
  DebugUpdateFactorsPriorCameraPose();
  DebugUpdateFactorsPriorCameraMotion();
  DebugUpdateFactorsPriorDepth();
  DebugUpdateFactorsIMU();
  DebugUpdateFactorsFixOrigin();
  DebugUpdateFactorsFixPositionZ();
  DebugUpdateFactorsFixMotion();

  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      X.m_Sadx.AssertEqual(KF.m_Axs[ix].m_Sadx, 1, str + UT::String(" iKF %d ix %d Sadx",
                                                                    iKF, ix));
      X.m_Sadx = KF.m_Axs[ix].m_Sadx;
    }
  }

  const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int icu = e_I[iKF][iKF];
    const auto e_SAcu = Camera::Factor::Unitary::CC::Get(e_SAccs[icu].GetSymmetricMatrix6x6f(),
                                                         e_Sbcs[iKF].GetVector6f());
    const Camera::Factor::Unitary::CC &SAcu = m_SAcus[iKF];
    e_SAcu.AssertEqual(SAcu, 1, str + UT::String(" SAcc[%d][%d]", iKF, iKF), epsAbs, epsRel);
    e_SAccs[icu] = SAcu.m_A;
    e_Sbcs[iKF] = SAcu.m_b;
    const KeyFrame &KF = m_KFs[iKF];
    const int Nkp = int(KF.m_ikp2KF.size());
    m_work.Resize(Nkp * sizeof(Camera::Factor::Binary::CC) / sizeof(float));
    AlignedVector<Camera::Factor::Binary::CC> SAcbs((Camera::Factor::Binary::CC *) m_work.Data(),
                                                    Nkp, false);
    SAcbs.MakeZero();
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      SAcbs[KF.m_Zs[iZ].m_ik] = KF.m_SAcxzs[iZ];
    }
    const int Np = int(KF.m_iKFsPrior.size());
    for (int ip = 0; ip < Np; ++ip) {
      SAcbs[KF.m_iAp2kp[ip]] += KF.m_SAps[ip];
    }
    for (int ikp = 0; ikp < Nkp; ++ikp) {
      const int _iKF = KF.m_ikp2KF[ikp];
      const int icb = e_I[_iKF][iKF];
      const auto e_SAcb = Camera::Factor::Binary::CC::Get(e_SAccs[icb].GetAlignedMatrix6x6f());
      const Camera::Factor::Binary::CC &SAcb = SAcbs[ikp];
      e_SAcb.AssertEqual(SAcb, 1, str + UT::String(" SAcc[%d][%d]", _iKF, iKF), epsAbs, epsRel);
      e_SAccs[icb] = SAcb;
    }
  }
  for (int im = 0; im < Nm; ++im) {
    const Camera::Factor &SAcm = m_SAcmsLM[im];
    e_SAcmsLM[im].AssertEqual(SAcm, 1, str + UT::String(" SAcm[%d]", im));
    e_SAcmsLM[im] = SAcm;
    e_Sbms[im].AssertEqual(SAcm.m_Au.m_Amm.m_b, 1, str + UT::String(" Sbm[%d]", im));
    e_Sbms[im] = SAcm.m_Au.m_Amm.m_b;
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsFeature() {
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D &C = m_Cs[iKF];
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Track &X = Xs[ix];
      const FTR::Source &x = KF.m_xs[ix];
      const Depth::InverseGaussian &d = ds[ix];
      const std::string _str = str + UT::String(" iKF = %d ix = %d", iKF, ix);
      const int Nz = static_cast<int>(X.m_zs.size());
      for (int i = 0; i < Nz; ++i) {
        Track::Measurement &z = X.m_zs[i];
        const KeyFrame &_KF = m_KFs[z.m_iKF];
        const int iz = z.m_iz;
        const FTR::Measurement &_z = _KF.m_zs[iz];
        FTR::EigenFactor A = FTR::EigenGetFactor<GBA_ME_FUNCTION>(BA_WEIGHT_FEATURE, C, x, d,
                                                                  m_Cs[z.m_iKF], _z, true, true
#ifdef CFG_STEREO
                                                                , m_K.m_br
#endif
                                                                );
        //A.AssertEqual(_KF.m_Lzs[iz], _KF.m_Azs1[iz], _KF.m_Azs2[iz], 1,
        //              _str + UT::String(" --> iKF %d iz %d Az", z.m_iKF, z.m_iz));
        A.Set(_KF.m_Lzs[iz], _KF.m_Azs1[iz], _KF.m_Azs2[iz]);
        X.m_Sadx += FTR::EigenFactor::DDC(A.m_add, A.m_adcx, A.m_bd);
        z.m_adcz = A.m_adcz;
        const int icxx = e_I[iKF][iKF], icxz = e_I[iKF][z.m_iKF], iczz = e_I[z.m_iKF][z.m_iKF];
        e_SAccs[icxx] += A.m_Acxx;
        e_SAccs[icxz] += A.m_Acxz;
        e_SAccs[iczz] += A.m_Aczz;
        e_Sbcs[iKF] += A.m_bcx;
        e_Sbcs[z.m_iKF] += A.m_bcz;
//#ifdef CFG_DEBUG
#if 0
        if (iKF == 44) {
          UT::Print("%d %d %f\n", iKF, ix, e_SAccs[icxx](0, 0));
        }
        if (z.m_iKF == 44) {
          UT::Print("%d %d %f\n", iKF, ix, e_SAccs[iczz](0, 0));
        }
#endif
      }
    }
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsPriorCameraPose() {
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const int NZ = static_cast<int>(m_Zps.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const CameraPrior::Pose &Z = m_Zps[iZ];
    CameraPrior::Pose::EigenFactor A = Z.EigenGetFactor(m_Aps[iZ].m_w, m_Cs,
                                                        BA_ANGLE_EPSILON);
    A.AssertEqual(m_Aps[iZ], 1, str + UT::String(" Ap[%d]", iZ));
    A.Set(m_Aps[iZ]);
    const int N = static_cast<int>(Z.m_iKFs.size());
    for (int i1 = -1, j1 = 0; i1 < N; ++i1, j1 += 6) {
      const int iKF1 = i1 == -1 ? Z.m_iKFr : Z.m_iKFs[i1];
      for (int i2 = i1, j2 = j1; i2 < N; ++i2, j2 += 6) {
        const int iKF2 = i2 == -1 ? Z.m_iKFr : Z.m_iKFs[i2];
        if (iKF1 <= iKF2) {
          const int icc = e_I[iKF1][iKF2];
          e_SAccs[icc] += A.m_A.block<6, 6>(j1, j2);
        } else {
          const int icc = e_I[iKF2][iKF1];
          e_SAccs[icc] += A.m_A.block<6, 6>(j2, j1);
        }
      }
    }
    for (int i = -1, j = 0; i < N; ++i, j += 6) {
      const int iKF = i == -1 ? Z.m_iKFr : Z.m_iKFs[i];
      e_Sbcs[iKF] += A.m_b.block<6, 1>(j, 0);
    }
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsPriorCameraMotion() {
  if (m_ZpLM.Invalid()) {
    return;
  }
  CameraPrior::Motion::EigenFactor A;
  const int ic = m_ZpLM.m_iKF, im = ic - m_Cs.Size() + m_CsLM.Size();
  A = m_ZpLM.EigenGetFactor(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_CsLM[im]);
  A.AssertEqual(m_ApLM, 1, UT::String("GBA [%d] iIter %d Am", m_KFs.back().m_T.m_iFrm, m_iIter));
  A.Set(m_ApLM);
  e_SAccs[e_I[ic][ic]].block<3, 3>(3, 3) += A.m_A.block<3, 3>(0, 0);
  e_Sbcs[ic].block<3, 1>(3, 0) += A.m_b.block<3, 1>(0, 0);
  Camera::EigenFactor::Unitary &SA = e_SAcmsLM[im].m_Au;
  SA.m_Acm.block<3, 9>(3, 0) += A.m_A.block<3, 9>(0, 3);
  SA.m_Amm += A.m_A.block<9, 9>(3, 3);
  e_Sbms[im] += A.m_b.block<9, 1>(3, 0);
}

void GlobalBundleAdjustor::DebugUpdateFactorsPriorDepth() {
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
    const Depth::InverseGaussian *ds = m_ds.data() + m_iKF2d[iKF];
#endif
    const KeyFrame &KF = m_KFs[iKF];
    std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      const FTR::Source &x = KF.m_xs[ix];
      const bool xr = x.m_xr.Valid();
      if (xr) {
        FTR::EigenFactor::Stereo e_Ard = FTR::EigenGetFactor<GBA_ME_FUNCTION>(BA_WEIGHT_FEATURE,
                                                                              m_K.m_br, ds[ix], x);
        //e_Ard.AssertEqual(KF.m_Ards[ix]);
        e_Ard = KF.m_Ards[ix];
      }
#endif
      FTR::Factor::DD &Sadd = Xs[ix].m_Sadx.m_add;
#ifdef CFG_STEREO
      if (xr) {
        Sadd += KF.m_Ards[ix].m_add;
      } else
#endif
      {
        Sadd += KF.m_Apds[ix];
      }
    }
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsIMU() {
  IMU::Delta::EigenFactor::Global A;
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const int Nc = int(m_KFs.size());
  for (int ic1 = Nc - m_CsLM.Size(), ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc;
       ic1 = ic2++, im1 = im2++) {
    if (m_KFs[ic2].m_us.Empty()) {
      continue;
    }
//#ifdef CFG_DEBUG
#if 0
    if (ic2 == 191 && m_iIter >= 7) {
      //IMU::Delta::Factor::Auxiliary::Global U;
      //m_DsLM[im2].GetFactor(BA_WEIGHT_IMU, m_CsLM[im1], m_CsLM[im2], m_K.m_pu,
      //                      &m_AdsLM[im2], &m_SAcmsLM[im2].m_Ab, &U);
      UT::DebugStart();
    }
#endif
    m_DsLM[im2].EigenGetFactor(BA_WEIGHT_IMU, m_CsLM[im1], m_CsLM[im2], m_K.m_pu, &A,
                               BA_ANGLE_EPSILON);
    A.AssertEqual(m_AdsLM[im2], m_SAcmsLM[im2].m_Ab, 1, str + UT::String(" Ad[%d][%d]", im1, im2));
    A.Set(m_AdsLM[im2], m_SAcmsLM[im2].m_Ab);
    Camera::EigenFactor::Unitary &SA11 = e_SAcmsLM[im1].m_Au, &SA22 = e_SAcmsLM[im2].m_Au;
    Camera::EigenFactor::Binary &SA12 = e_SAcmsLM[im2].m_Ab;
    e_SAccs[e_I[ic1][ic1]] += A.m_Ac1c1;
    SA11.m_Acm += A.m_Ac1m1;
    SA12.m_Acc += A.m_Ac1c2;
    SA12.m_Acm += A.m_Ac1m2;
    e_Sbcs[ic1] += A.m_bc1;
    SA11.m_Amm += A.m_Am1m1;
    SA12.m_Amc += A.m_Am1c2;
    SA12.m_Amm += A.m_Am1m2;
    e_Sbms[im1] += A.m_bm1;
    e_SAccs[e_I[ic2][ic2]] += A.m_Ac2c2;
    SA22.m_Acm += A.m_Ac2m2;
    e_Sbcs[ic2] += A.m_bc2;
    SA22.m_Amm += A.m_Am2m2;
    e_Sbms[im2] += A.m_bm2;
    //e_F = A.m_F + e_F;
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsFixOrigin() {
  const int iKF = 0;
  if (m_KFs[iKF].m_T.m_iFrm != 0) {
    return;
  }
  Camera::Fix::Origin::EigenFactor e_A = m_Zo.EigenGetFactor(m_Cs[iKF], BA_ANGLE_EPSILON);
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  e_A.AssertEqual(m_Ao, 1, str + "Ao");
  e_A = m_Ao;
  const int icc = e_I[iKF][iKF];
  e_SAccs[icc] += e_A.m_A;
  e_Sbcs[iKF] += e_A.m_b;
}

void GlobalBundleAdjustor::DebugUpdateFactorsFixPositionZ() {
  const float w = UT::Inverse(BA_VARIANCE_FIX_POSITION_Z, BA_WEIGHT_FIX_POSITION_Z);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int icc = e_I[iKF][iKF];
    e_SAccs[icc](2, 2) += w;
    e_Sbcs[iKF](2, 0) += w * m_Cs[iKF].GetPositionZ();
  }
}

void GlobalBundleAdjustor::DebugUpdateFactorsFixMotion() {
  const float wv = UT::Inverse(BA_VARIANCE_FIX_VELOCITY);
  const float wba = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION);
  const float wbw = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE);
  const float wv0 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INITIAL);
  const float wba0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL);
  const float wbw0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL);
  const float wv1 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INVALID);
  const float wba1 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID);
  const float wbw1 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID);
  const int Nc = m_Cs.Size();
  for (int ic1 = Nc - m_CsLM.Size(), ic2 = ic1 + 1, im = 0; ic1 < Nc; ic1 = ic2++, ++im) {
    const Camera &C = m_CsLM[im];
    EigenMatrix9x9f &SA = e_SAcmsLM[im].m_Au.m_Amm;
    EigenVector9f &Sb = e_Sbms[im];
    const float v2 = C.m_v.SquaredLength();
    const float ba2 = C.m_ba.SquaredLength();
    const float bw2 = C.m_bw.SquaredLength();
    float _wv, _wba, _wbw;
    const KeyFrame &KF = m_KFs[ic1];
    if (KF.m_T.m_iFrm == 0) {
      _wv = wv0;
      _wba = wba0;
      _wbw = wbw0;
    } else if (KF.m_us.Empty() && (ic2 == Nc || m_KFs[ic2].m_us.Empty())) {
      _wv = wv1;
      _wba = wba1;
      _wbw = wbw1;
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
  }
}

void GlobalBundleAdjustor::DebugUpdateSchurComplement() {
  if (m_debug <= 0) {
    return;
  }
  const int nKFs = int(m_KFs.size()), Ncc = e_I[nKFs - 1][nKFs - 1] + 1;
  e_SMccs.resize(Ncc);
  for (int icc = 0; icc < Ncc; ++icc) {
    e_SMccs[icc].setZero();
  }
  e_Smcs.resize(nKFs);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    e_Smcs[iKF].setZero();
  }
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  //const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int icxx = e_I[iKF][iKF];
    EigenMatrix6x6f &SMcxx = e_SMccs[icxx];
    EigenVector6f &Smcx = e_Smcs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const float Sadd = X.m_Sadx.m_add.m_a/* + add*/, Sbd = X.m_Sadx.m_add.m_b;
      const float mdd = Sadd > epsd ? 1.0f / Sadd : 0.0f;
      const EigenVector6f mdcxT = EigenVector6f(X.m_Sadx.m_adc.transpose() * mdd);
      SMcxx += mdcxT * X.m_Sadx.m_adc;
      Smcx += mdcxT * Sbd;
      const int Nz = int(X.m_zs.size());
      for (int i = 0; i < Nz; ++i) {
        const Track::Measurement &z = X.m_zs[i];
        const int icxz = e_I[iKF][z.m_iKF];
        e_SMccs[icxz] += mdcxT * z.m_adcz;
      }
      for (int i1 = 0; i1 < Nz; ++i1) {
        const Track::Measurement &z1 = X.m_zs[i1];
        const EigenVector6f mdczT = EigenVector6f(z1.m_adcz.transpose() * mdd);
        for (int i2 = i1; i2 < Nz; ++i2) {
          const Track::Measurement &z2 = X.m_zs[i2];
          const int iczz = e_I[z1.m_iKF][z2.m_iKF];
          e_SMccs[iczz] += mdczT * z2.m_adcz;
        }
        e_Smcs[z1.m_iKF] += mdczT * Sbd;
      }
    }
  }

  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int icu = e_I[iKF][iKF];
    const auto e_SMcu = Camera::Factor::Unitary::CC::Get(e_SMccs[icu].GetSymmetricMatrix6x6f(),
                                                         e_Smcs[iKF].GetVector6f());
    const Camera::Factor::Unitary::CC &SMcu = m_SMcus[iKF];
    e_SMcu.AssertEqual(SMcu, 1, str + UT::String(" SMcc[%d][%d]", iKF, iKF), epsAbs, epsRel);
    e_SMccs[icu] = SMcu.m_A;
    e_Smcs[iKF] = SMcu.m_b;
    const KeyFrame &KF = m_KFs[iKF];
    const int Nk = KF.m_Zm.m_SMczms.Size();
    for (int ik = 0; ik < Nk; ++ik) {
      const int _iKF = KF.m_ikp2KF[ik];
      const int icb = e_I[_iKF][iKF];
      const auto e_SMcb = Camera::Factor::Binary::CC::Get(e_SMccs[icb].GetAlignedMatrix6x6f());
      const Camera::Factor::Binary::CC &SMcb = KF.m_Zm.m_SMczms[ik];
      e_SMcb.AssertEqual(SMcb, 1, str + UT::String(" SMcc[%d][%d]", _iKF, iKF), epsAbs, epsRel);
      e_SMccs[icb] = SMcb;
    }
  }
}

void GlobalBundleAdjustor::DebugSolveSchurComplement() {
  if (m_debug <= 0) {
    return;
  }
  const int Nc = m_Cs.Size();
  e_xcs.resize(Nc);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsGN.Data();
  for (int ic = 0; ic < Nc; ++ic) {
    e_xcs[ic] = EigenVector6f(xcs[ic]);
  }
  const int Nm = m_CsLM.Size();
  e_xms.resize(Nm);
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  for (int im = 0; im < Nm; ++im) {
    e_xms[im] = EigenVector9f(xms[im]);
  }
  if (m_debug < 2) {
    return;
  }
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  EigenVector6f e_Sbc;
  EigenVector9f e_Sbm;
  const float e2Max = ME::ChiSquareDistance<3>(BA_PCG_MIN_CONVERGE_PROBABILITY, BA_WEIGHT_FEATURE);
  for (int ic1 = 0, im1 = Nm - Nc; ic1 < Nc; ++ic1, ++im1) {
    e_Sbc.setZero();
    e_Sbm.setZero();
    for (int ic0 = 0; ic0 < ic1; ++ic0) {
      const int icc = e_I[ic0][ic1];
      if (icc != -1) {
        e_Sbc += (e_SAccs[icc] - e_SMccs[icc]).transpose() * e_xcs[ic0];
      }
    }
    const int ic0 = ic1 - 1, im0 = im1 - 1;
    if (im0 >= 0) {
      const Camera::EigenFactor::Binary &SAcmb = e_SAcmsLM[im1].m_Ab;
      e_Sbc = EigenVector6f(SAcmb.m_Acc.transpose() * e_xcs[ic0] + e_Sbc);
      e_Sbc = EigenVector6f(SAcmb.m_Amc.transpose() * e_xms[im0] + e_Sbc);
      e_Sbm = EigenVector9f(SAcmb.m_Acm.transpose() * e_xcs[ic0] + e_Sbm);
      e_Sbm = EigenVector9f(SAcmb.m_Amm.transpose() * e_xms[im0] + e_Sbm);
    }
    for (int ic2 = ic1; ic2 < Nc; ++ic2) {
      const int icc = e_I[ic1][ic2];
      if (icc != -1) {
        e_Sbc += (e_SAccs[icc] - e_SMccs[icc]) * e_xcs[ic2];
      }
    }
    if (im1 >= 0) {
      const Camera::EigenFactor::Unitary &SAcmu = e_SAcmsLM[im1].m_Au;
      e_Sbc = EigenVector6f(SAcmu.m_Acm * e_xms[im1] + e_Sbc);
      e_Sbm = EigenVector9f(SAcmu.m_Acm.transpose() * e_xcs[ic1] + e_Sbm);
      e_Sbm = EigenVector9f(SAcmu.m_Amm * e_xms[im1] + e_Sbm);
      const int ic2 = ic1 + 1, im2 = im1 + 1;
      if (im2 < Nm) {
        const Camera::EigenFactor::Binary &e_SAcmb = e_SAcmsLM[im2].m_Ab;
        e_Sbc = EigenVector6f(e_SAcmb.m_Acc * e_xcs[ic2] + e_Sbc);
        e_Sbc = EigenVector6f(e_SAcmb.m_Acm * e_xms[im2] + e_Sbc);
        e_Sbm = EigenVector9f(e_SAcmb.m_Amc * e_xcs[ic2] + e_Sbm);
        e_Sbm = EigenVector9f(e_SAcmb.m_Amm * e_xms[im2] + e_Sbm);
      }
    }
    e_Sbc = -e_Sbc;
    const EigenMatrix6x6f e_Mc = m_Mcs[ic1].GetAlignedMatrix6x6f();
    const EigenVector6f e_rc = EigenVector6f((e_Sbcs[ic1] - e_Smcs[ic1]) - e_Sbc);
    const EigenVector6f e_zc = EigenVector6f(e_Mc * e_rc);
    const float e_e2p = e_rc.block<3, 1>(0, 0).dot(e_zc.block<3, 1>(0, 0));
    const float e_e2r = e_rc.block<3, 1>(3, 0).dot(e_zc.block<3, 1>(3, 0));
    if (e2Max == 0.0f) {
      UT_ASSERT(UT::AssertZero(e_e2p) && UT::AssertZero(e_e2r));
    } else {
      UT_ASSERT(e_e2p < e2Max && e_e2r < e2Max);
    }
    if (im1 < 0) {
      continue;
    }
    e_Sbm = -e_Sbm;
    const EigenMatrix9x9f e_Mm = m_MmsLM[im1].GetAlignedMatrix9x9f().GetMatrix9x9f();
    const EigenVector9f e_rm = EigenVector9f(e_Sbms[im1] - e_Sbm);
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

void GlobalBundleAdjustor::DebugSolveBackSubstitution() {
  if (m_debug <= 0) {
    return;
  }
  //const float epsAbs = 1.0e-5f;
  const float epsAbs = BA_UPDATE_DEPTH;
  //const float epsRel = 1.0e-3f;
  const float epsRel = 1.0e-2f;
  const int nKFs = static_cast<int>(m_KFs.size());
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsGN.Data();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DELTA) {
      e_xcs[iKF] = EigenVector6f(xcs[iKF]);
    } else {
      e_xcs[iKF].setZero();
    }
  }
  
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const float eps = FLT_EPSILON;
  const float epsd = UT::Inverse(BA_VARIANCE_MAX_DEPTH, BA_WEIGHT_FEATURE, eps);
  //const float add = UT::Inverse(BA_VARIANCE_REGULARIZATION_DEPTH, BA_WEIGHT_FEATURE);
  for (int iKF = 0, iX = 0; iKF < nKFs; ++iKF) {
    const EigenVector6f &xc = e_xcs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
#if 0
    const float *Sxds = m_Sxds.Data() + m_iKF2d[iKF];
#else
    float *Sxds = NULL;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const bool exist = UT::VectorExistFlag<ubyte>(uds, Nx,
                                                  GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION);
    if (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION) {
      UT_ASSERT(exist);
      Sxds = m_xds.Data() + iX;
      iX += Nx;
    } else {
      UT_ASSERT(!exist);
      continue;
    }
#endif
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        continue;
      }
      const Track &X = Xs[ix];
      const float Sadd = X.m_Sadx.m_add.m_a/* + add*/;
      const float mdd = Sadd > epsd ? 1.0f / Sadd : 0.0f;
      float xd = (X.m_Sadx.m_add.m_b + X.m_Sadx.m_adc.dot(xc)) * mdd, Sxd = -xd;
#if 0
      float _xd = KF.m_Axs[ix].m_xdx, _Sxd = -_xd;
#endif
      const int Nz = static_cast<int>(X.m_zs.size());
      for (int i = 0; i < Nz; ++i) {
        const Track::Measurement &z = X.m_zs[i];
        xd = z.m_adcz.dot(e_xcs[z.m_iKF]) * mdd;
        Sxd = -xd + Sxd;
#if 0
        _xd = m_KFs[z.m_iKF].m_Azs[z.m_iz].m_xdz;
        _Sxd = -_xd + _Sxd;
#endif
      }
      UT::AssertEqual(Sxd, Sxds ? Sxds[ix] : 0.0f, 1, str + UT::String(" iKF %d ix %d xd", iKF, ix),
                      epsAbs, epsRel);
    }
  }
}

void GlobalBundleAdjustor::DebugSolveGradientDescent() {
  if (m_debug <= 0) {
    return;
  }
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const float s = m_bl == 0.0f ? 0.0f : 1.0f / m_bl;
  const int Nc = m_Cs.Size();
  e_gcs.resize(Nc);
  e_Agcs.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    EigenVector6f &e_gc = e_gcs[ic];
    e_gc = EigenVector6f(e_Sbcs[ic] * s);
    e_Agcs[ic] = EigenVector6f(e_SAccs[e_I[ic][ic]] * e_gc);
  }
  const int Nm = m_CsLM.Size();
  e_gms.resize(Nm);
  e_Agms.resize(Nm);
  for (int ic = Nc - Nm, im = 0; ic < Nc; ++ic, ++im) {
    EigenVector9f &e_gm = e_gms[im];
    e_gm = EigenVector9f(e_Sbms[im] * s);
    const Camera::EigenFactor::Unitary &e_A = e_SAcmsLM[im].m_Au;
    e_Agcs[ic] = EigenVector6f(e_A.m_Acm * e_gm + e_Agcs[ic]);
    e_Agms[im] = EigenVector9f(e_A.m_Amm * e_gm + e_A.m_Acm.transpose() * e_gcs[ic]);
  }
  for (int ic1 = Nc - Nm, ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc; ic1 = ic2++, im1 = im2++) {
    const Camera::EigenFactor::Binary &e_A = e_SAcmsLM[im2].m_Ab;
    const EigenVector6f &e_gc1 = e_gcs[ic1], &e_gc2 = e_gcs[ic2];
    const EigenVector9f &e_gm1 = e_gms[im1], &e_gm2 = e_gms[im2];
    EigenVector6f &e_Agc1 = e_Agcs[ic1], &e_Agc2 = e_Agcs[ic2];
    EigenVector9f &e_Agm1 = e_Agms[im1], &e_Agm2 = e_Agms[im2];
    e_Agc1 = EigenVector6f(e_A.m_Acc * e_gc2 + e_Agc1);
    e_Agc2 = EigenVector6f(e_A.m_Acc.transpose() * e_gc1 + e_Agc2);
    e_Agc1 = EigenVector6f(e_A.m_Acm * e_gm2 + e_Agc1);
    e_Agm1 = EigenVector9f(e_A.m_Amm.block<9, 3>(0, 0) * e_gm2.block<3, 1>(0, 0) + e_Agm1);
    e_Agm2 = EigenVector9f(e_A.m_Acm.transpose() * e_gc1 + e_Agm2);
    e_Agm1 = EigenVector9f(e_A.m_Amc * e_gc2 + e_Agm1);
    e_Agc2 = EigenVector6f(e_A.m_Amc.transpose() * e_gm1 + e_Agc2);
    //e_Agm1 = EigenVector9f(e_A.m_Amm * e_gm2 + e_Agm1);
    e_Agm1 = EigenVector9f(e_A.m_Amm.block<9, 6>(0, 3) * e_gm2.block<6, 1>(3, 0) + e_Agm1);
    e_Agm2 = EigenVector9f(e_A.m_Amm.transpose() * e_gm1 + e_Agm2);
  }
  for (int ic1 = 0; ic1 < Nc; ++ic1) {
    const EigenVector6f &e_gc1 = e_gcs[ic1];
    EigenVector6f &e_Agc1 = e_Agcs[ic1];
    for (int ic2 = ic1 + 1; ic2 < Nc; ++ic2) {
      const int icc = e_I[ic1][ic2];
      if (icc == -1) {
        continue;
      }
      const EigenMatrix6x6f &e_A = e_SAccs[icc];
      e_Agc1 = EigenVector6f(e_A * e_gcs[ic2] + e_Agc1);
      e_Agcs[ic2] = EigenVector6f(e_A.transpose() * e_gc1 + e_Agcs[ic2]);
    }
  }
  e_gds.Resize(0);
  e_Agds.Resize(0);
  for (int ic = 0; ic < Nc; ++ic) {
    const EigenVector6f &e_gc = e_gcs[ic];
    EigenVector6f &e_Agc = e_Agcs[ic];
    const int iX = m_iKF2X[ic];
    if (iX == -1) {
      continue;
    }
    const ubyte *uds = m_uds.data() + m_iKF2d[ic];
    const float *gds = m_gds.Data() + iX, *Agds = m_Agds.Data() + iX;
    const std::vector<Track> &Xs = e_Xs[ic];
    const int Nx = int(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const FTR::EigenFactor::DDC &e_Sadx = X.m_Sadx;
      float &e_gd = e_gds.Push(), &e_Agd = e_Agds.Push();
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        e_gd = e_Sadx.m_add.m_b * s;
        e_Agd = e_Sadx.m_add.m_a * e_gd + e_Sadx.m_adc * e_gc;
//#ifdef CFG_DEBUG
#if 0
        if (m_iIter == 1 && ic == 0)
          UT::Print("%d: %e * %e + %e = %e\n", ix, e_gd, e_Sadx.m_adc(0, 0), e_Agc(0, 0), e_Sadx.m_adc(0, 0) * e_gd + e_Agc(0, 0));
#endif
        e_Agc = EigenVector6f(e_Sadx.m_adc.transpose() * e_gd + e_Agc);
        const int Nz = int(X.m_zs.size());
        for (int i = 0; i < Nz; ++i) {
          const Track::Measurement &z = X.m_zs[i];
          const int _iKF = z.m_iKF;
#ifdef CFG_DEBUG
          UT_ASSERT(_iKF > ic);
#endif
          const EigenVector6f e_adcz = EigenVector6f(z.m_adcz.transpose());
          e_Agd = e_adcz.dot(e_gcs[_iKF]) + e_Agd;
          e_Agcs[_iKF] = EigenVector6f(e_adcz * e_gd + e_Agcs[_iKF]);
        }
        UT::AssertEqual(e_gd, gds[ix], 1, str + UT::String(" ic %d ix %d gd", ic, ix));
        UT::AssertEqual(e_Agd, Agds[ix], 1, str + UT::String(" ic %d ix %d Agd", ic, ix));
        e_gd = gds[ix];
        e_Agd = Agds[ix];
      } else {
        e_gd = 0.0f;
        e_Agd = 0.0f;
      }
    }
  }
  const LA::Vector6f *Agcs = (LA::Vector6f *) m_Ags.Data();
  for (int ic = 0; ic < Nc; ++ic) {
    e_Agcs[ic].GetVector6f().AssertEqual(Agcs[ic], 1, str + UT::String(" Agc[%d]", ic));
    e_Agcs[ic] = Agcs[ic];
  }
  const LA::Vector9f *Agms = (LA::Vector9f *) (Agcs + Nc);
  for (int im = 0; im < Nm; ++im) {
    e_Agms[im].GetVector9f().AssertEqual(Agms[im], 1, str + UT::String(" Agm[%d]", im));
    e_Agms[im] = Agms[im];
  }
  float gTgc = 0.0f, gTAgc = 0.0f;
  for (int ic = 0; ic < Nc; ++ic) {
    gTgc = e_gcs[ic].SquaredLength() + gTgc;
    gTAgc = e_gcs[ic].dot(e_Agcs[ic]) + gTAgc;
  }
  float gTgm = 0.0f, gTAgm = 0.0f;
  for (int im = 0; im < Nm; ++im) {
    gTgm = e_gms[im].SquaredLength() + gTgm;
    gTAgm = e_gms[im].dot(e_Agms[im]) + gTAgm;
  }
  const float gTgd = e_gds.SquaredLength(), gTAgd = e_gds.Dot(e_Agds);
  const float gTg = gTgc + gTgm + gTgd;
  const float gTAg = gTAgc + gTAgm + gTAgd;
  UT::AssertEqual(gTg, 1.0f, 1, str + " gTg");
  UT::AssertEqual(gTAg, m_gTAg, 1, str + " gTAg");
}

void GlobalBundleAdjustor::DebugComputeReduction() {
  if (m_debug <= 0) {
    return;
  }
  const int Nc = m_Cs.Size();
  e_xcs.resize(Nc);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data();
  for (int ic = 0; ic < Nc; ++ic) {
    if (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA) {
      e_xcs[ic] = EigenVector6f(xcs[ic]);
    } else {
      e_xcs[ic].setZero();
    }
  }
  const int Nm = m_CsLM.Size();
  e_xms.resize(Nm);
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  for (int im = 0; im < Nm; ++im) {
    const LA::Vector9f &xm = xms[im];
    EigenVector9f &e_xm = e_xms[im];
    const ubyte ucm = m_ucmsLM[im];
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
      e_xm.block<3, 1>(0, 0) = EigenVector3f(&xm.v0());
    } else {
      e_xm.block<3, 1>(0, 0).setZero();
    }
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
      e_xm.block<3, 1>(3, 0) = EigenVector3f(&xm.v3());
    } else {
      e_xm.block<3, 1>(3, 0).setZero();
    }
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
      e_xm.block<3, 1>(6, 0) = EigenVector3f(&xm.v6());
    } else {
      e_xm.block<3, 1>(6, 0).setZero();
    }
  }
  e_xds.Resize(int(m_ds.size()));
  e_xds.MakeZero();
  for (int iKF = 0; iKF < Nc; ++iKF) {
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
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH) {
        xds2[ix] = xds1[ix];
      }
    }
  }
  e_dFp = 0.0f;
  DebugComputeReductionFeature();
  DebugComputeReductionPriorCameraPose();
  DebugComputeReductionPriorCameraMotion();
  DebugComputeReductionPriorDepth();
  DebugComputeReductionIMU();
  DebugComputeReductionFixOrigin();
  DebugComputeReductionFixPositionZ();
  DebugComputeReductionFixMotion();
  const int iFrm = m_KFs.back().m_T.m_iFrm;
  const std::string str = UT::String("GBA [%d] iIter %d", iFrm, m_iIter);
  const float eps = 1.0e-3f;
  UT::AssertEqual(e_dFp, m_dFp, 1, str + UT::String(" iIterDL %d dFp", m_iIterDL), eps);
}

void GlobalBundleAdjustor::DebugComputeReductionFeature() {
  const int nKFs = int(m_KFs.size());
  m_work.Resize(nKFs);
  LA::AlignedVectorXf dFps(m_work.Data(), nKFs, false);
  dFps.MakeZero();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D &C = m_CsBkp[iKF];
    const EigenVector6f *e_xc = &e_xcs[iKF];
    const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
    const Depth::InverseGaussian *ds = iX == -1 ? m_ds.data() + id : m_dsBkp.data() + iX;
    const float *xds = e_xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const std::vector<Track> &Xs = e_Xs[iKF];
    const int Nx = static_cast<int>(Xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Track &X = Xs[ix];
      const FTR::Source &x = KF.m_xs[ix];
      const Depth::InverseGaussian &d = ds[ix];
      const float xd = xds[ix];
      const int Nz = int(X.m_zs.size());
      for (int i = 0; i < Nz; ++i) {
        const Track::Measurement &z = X.m_zs[i];
        const int _iKF = z.m_iKF, iz = z.m_iz;
        const KeyFrame &_KF = m_KFs[_iKF];
        const float F = FTR::EigenGetCost<GBA_ME_FUNCTION>(BA_WEIGHT_FEATURE, C, x, d,
                                                           m_CsBkp[_iKF], _KF.m_zs[iz],
                                                           e_xc, &e_xcs[_iKF], xd
#ifdef CFG_STEREO
                                                         , m_K.m_br
#endif
                                                         );
        const float dF = _KF.m_Lzs[iz].m_F - F;
        dFps[_iKF] = dF + dFps[_iKF];
      }
    }
  }
  for (int iKF = 0; iKF < nKFs; ++iKF)
    e_dFp = dFps[iKF] + e_dFp;
//#ifdef CFG_DEBUG
#if 0
  if (UT::Debugging() && m_iIter == 0) {
    UT::PrintSeparator();
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      UT::Print("%d %e\n", iKF, dFps[iKF]);
    }
  }
#endif
}

void GlobalBundleAdjustor::DebugComputeReductionPriorCameraPose() {
  const int NZ = static_cast<int>(m_Zps.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const float F = m_Zps[iZ].EigenGetCost(m_Aps[iZ].m_w, m_CsBkp, e_xcs,
                                           BA_ANGLE_EPSILON);
    const float dF = m_Aps[iZ].m_F - F;
    e_dFp = dF + e_dFp;
  }
}

void GlobalBundleAdjustor::DebugComputeReductionPriorCameraMotion() {
  if (m_ZpLM.Invalid()) {
    return;
  }
  const int ic = m_ZpLM.m_iKF, im = ic - m_Cs.Size() + m_CsLM.Size();
  const EigenVector3f e_xr(e_xcs[ic].block<3, 1>(3, 0));
  const float F = m_ZpLM.EigenGetCost(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_CsLMBkp[im], e_xr, e_xms[im]);
  const float dF = m_ApLM.m_F - F;
  e_dFp = dF + e_dFp;
}

void GlobalBundleAdjustor::DebugComputeReductionPriorDepth() {
  Depth::Prior::Reduction Ra, Rp;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
    const Depth::InverseGaussian *ds = iX == -1 ? m_ds.data() + id : m_dsBkp.data() + iX;
    const float *xds = e_xds.Data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
#ifdef CFG_STEREO
      const FTR::Source &x = KF.m_xs[ix];
      if (x.m_xr.Valid()) {
        const float F = FTR::EigenGetCost<GBA_ME_FUNCTION>(BA_WEIGHT_FEATURE, m_K.m_br,
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

void GlobalBundleAdjustor::DebugComputeReductionIMU() {
  const int Nc = m_Cs.Size();
  for (int ic1 = Nc - m_CsLM.Size(), ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc;
       ic1 = ic2++, im1 = im2++) {
    if (m_KFs[ic2].m_us.Empty()) {
      continue;
    }
    const float F = m_DsLM[im2].EigenGetCost(BA_WEIGHT_IMU, m_CsLMBkp[im1], m_CsLMBkp[im2],
                                             m_K.m_pu, e_xcs[ic1], e_xms[im1], e_xcs[ic2],
                                             e_xms[im2], BA_ANGLE_EPSILON);
    const float dF = m_AdsLM[im2].m_F - F;
    e_dFp = dF + e_dFp;
  }
}

void GlobalBundleAdjustor::DebugComputeReductionFixOrigin() {
  const int iKF = 0;
  if (m_KFs[iKF].m_T.m_iFrm != 0) {
    return;
  }
  const float F = m_Zo.EigenGetCost(m_CsBkp[iKF], e_xcs[iKF], BA_ANGLE_EPSILON);
  const float dF = m_Ao.m_F - F;
  e_dFp = dF + e_dFp;
}

void GlobalBundleAdjustor::DebugComputeReductionFixPositionZ() {
  const float w = UT::Inverse(BA_VARIANCE_FIX_POSITION_Z, BA_WEIGHT_FIX_POSITION_Z);
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const float pz1 = m_CsBkp[iKF].GetPositionZ();
    const float pz2 = pz1 + e_xcs[iKF](2, 0);
    const float F = w * pz2 * pz2;
    const float dF = F - m_Afps[iKF].m_F;
    e_dFp = dF + e_dFp;
  }
}

void GlobalBundleAdjustor::DebugComputeReductionFixMotion() {
  const float wv = UT::Inverse(BA_VARIANCE_FIX_VELOCITY);
  const float wba = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION);
  const float wbw = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE);
  const float wv0 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INITIAL);
  const float wba0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL);
  const float wbw0 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL);
  const float wv1 = UT::Inverse(BA_VARIANCE_FIX_VELOCITY_INVALID);
  const float wba1 = UT::Inverse(BA_VARIANCE_FIX_BIAS_ACCELERATION_INVALID);
  const float wbw1 = UT::Inverse(BA_VARIANCE_FIX_BIAS_GYROSCOPE_INVALID);
  const int Nc = m_Cs.Size();
  for (int ic = Nc - m_CsLM.Size(), im = 0; ic < Nc; ++ic, ++im) {
    const Camera &C = m_CsLMBkp[im];
    const float v2 = C.m_v.SquaredLength();
    const float ba2 = C.m_ba.SquaredLength();
    const float bw2 = C.m_bw.SquaredLength();
    float _wv, _wba, _wbw;
    const KeyFrame &KF = m_KFs[ic];
    if (KF.m_T.m_iFrm == 0) {
      _wv = wv0;
      _wba = wba0;
      _wbw = wbw0;
    } else if (KF.m_us.Empty() && (ic + 1 == Nc || m_KFs[ic + 1].m_us.Empty())) {
      _wv = wv1;
      _wba = wba1;
      _wbw = wbw1;
    } else {
      _wv = ME::Weight<GBA_ME_FUNCTION>(v2 * wv) * wv;
      _wba = ME::Weight<GBA_ME_FUNCTION>(ba2 * wba) * wba;
      _wbw = ME::Weight<GBA_ME_FUNCTION>(bw2 * wbw) * wbw;
    }
    _wv *= BA_WEIGHT_FIX_MOTION;
    _wba *= BA_WEIGHT_FIX_MOTION;
    _wbw *= BA_WEIGHT_FIX_MOTION;
    const EigenVector9f &e_xm = e_xms[im];
    const Camera::Fix::Motion::Factor &A = m_AfmsLM[im];
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
