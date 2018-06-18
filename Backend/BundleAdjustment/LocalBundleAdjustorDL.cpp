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

#if defined CFG_DEBUG && defined CFG_GROUND_TRUTH
//#define LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
#endif

void LocalBundleAdjustor::SolveDogLeg() {
  if (m_x2GN <= m_delta2 || BA_DL_MAX_ITERATIONS == 0) {
    m_xsDL.Set(m_xsGN);
    m_x2DL = m_x2GN;
    m_beta = 1.0f;
  } else if (m_x2GD >= m_delta2) {
    if (m_delta2 == 0.0f) {
      m_xsDL.Set(m_xsGD);
      m_x2DL = m_x2GD;
    } else {
      m_xsGD.GetScaled(sqrtf(m_delta2 / m_x2GD), m_xsDL);
      m_x2DL = m_delta2;
    }
    m_beta = 0.0f;
  } else {
    LA::AlignedVectorXf::AmB(m_xsGN, m_xsGD, m_dxs);
    const float d = m_xsGD.Dot(m_dxs), dx2 = m_dxs.SquaredLength();
    //m_beta = static_cast<float>((-d + sqrt(static_cast<double>(d) * d +
    //                            (m_delta2 - m_x2GD) * static_cast<double>(dx2))) / dx2);
    m_beta = (-d + sqrtf(d * d + (m_delta2 - m_x2GD) * dx2)) / dx2;
    m_dxs *= m_beta;
    m_xsDL.Set(m_xsGD);
    m_xsDL += m_dxs;
    m_x2DL = m_delta2;
  }
  const int pc = 6, pm = 9;
  const int Nc = int(m_LFs.size()), Ncp = Nc * pc, Nmp = Nc * pm, Ncmp = Ncp + Nmp;
  ConvertCameraUpdates(m_xsDL.Data(), &m_xp2s, &m_xr2s);
  ConvertMotionUpdates(m_xsDL.Data() + Ncp, &m_xv2s, &m_xba2s, &m_xbw2s);
  ConvertDepthUpdates(m_xsDL.Data() + Ncmp, &m_xds);
}

void LocalBundleAdjustor::SolveGradientDescent() {
  const int pc = 6, pm = 9;
  const int Nc = int(m_LFs.size()), Nd = m_xds.Size();
  const int Ncp = Nc * pc, Nmp = Nc * pm, Ncmp = Ncp + Nmp;
  m_xsGD.Resize(0);
  for (int ic = 0; ic < Nc; ++ic) {
    m_xsGD.Push(m_SAcusLF[m_ic2LF[ic]].m_b, 6);
  }
  for (int ic = 0; ic < Nc; ++ic) {
    m_xsGD.Push(m_SAcmsLF[m_ic2LF[ic]].m_Au.m_Amm.m_b, 9);
  }
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        m_xsGD.Push(KF.m_Axs[ix].m_Sadd.m_b);
      }
    }
  }
  m_bl = sqrtf(m_xsGD.SquaredLength());
  if (m_bl != 0.0f) {
    m_xsGD /= m_bl;
  }
  ConvertCameraUpdates((LA::Vector6f *) m_xsGD.Data(), &m_gcs);
  ConvertDepthUpdates(m_xsGD.Data() + Ncmp, &m_gds);

  LA::AlignedMatrix6x6f A66;
  m_Ags.Resize(Ncmp);
  LA::Vector6f *Agcs = (LA::Vector6f *) m_Ags.Data();
  for (int ic = 0; ic < Nc; ++ic) {
    m_SAcusLF[m_ic2LF[ic]].m_A.GetAlignedMatrix6x6f(A66);
    LA::AlignedMatrix6x6f::Ab(A66, m_gcs[ic], (float *) &Agcs[ic]);
  }

  ApplyAcm(m_gcs.Data(), (LA::Vector9f *) (m_xsGD.Data() + Ncp), Agcs,
           (LA::Vector9f *) (m_Ags.Data() + Ncp));

  m_Agds.Resize(Nd);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const float *gds = m_gds.Data() + iX;
    float *Agds = m_Agds.Data() + iX;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        Agds[ix] = KF.m_Axs[ix].m_Sadd.m_a * gds[ix];
      }
    }
  }

  LA::AlignedVector6f Agc;
  for (int ic = 0; ic < Nc; ++ic) {
    const LocalFrame &LF = m_LFs[m_ic2LF[ic]];
    const LA::AlignedVector6f &gc = m_gcs[ic];
    Agc.Set(Agcs[ic]);
    const int NZ = int(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const int iX = m_iKF2X[Z.m_iKF];
      if (iX == -1) {
        continue;
      }
      const float *gds = m_gds.Data() + iX;
      float *Agds = m_Agds.Data() + iX;
      const ubyte *uds = m_uds.data() + m_iKF2d[Z.m_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = LF.m_zs[iz].m_ix;
        if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
          continue;
        }
        const LA::AlignedVector6f &adcz = LF.m_Azs1[iz].m_adczA;
        Agds[ix] = adcz.Dot(gc) + Agds[ix];
        LA::AlignedVector6f::AddsaTo(gds[ix], adcz, Agc);
      }
    }
    Agc.Get(Agcs[ic]);
  }
  PushDepthUpdates(m_Agds, &m_Ags);
  m_gTAg = m_xsGD.Dot(m_Ags);
  const float xl = m_bl / m_gTAg;
  m_xsGD *= -xl;
  m_x2GD = xl * xl;
//#ifdef CFG_DEBUG
#if 0
  if (m_debug) {
    const float x2GD = m_xsGD.SquaredLength();
    UT::AssertEqual(x2GD, m_x2GD);
  }
#endif
}

void LocalBundleAdjustor::ComputeReduction() {
  m_dFa = m_dFp = 0.0f;
  if (m_update) {
    ConvertCameraUpdates((LA::Vector6f *) m_xsDL.Data(), &m_xcsP);
    ComputeReductionFeaturePriorDepth();
    ComputeReductionPriorCameraMotion();
    ComputeReductionIMU();
    //ComputeReductionFixOrigin();
    ComputeReductionFixPositionZ();
    ComputeReductionFixMotion();
    m_rho = m_dFa > 0.0f && m_dFp > 0.0f ? m_dFa / m_dFp : -1.0f;
  } else {
    m_rho = 0.0f;
  }
}

void LocalBundleAdjustor::ComputeReductionFeaturePriorDepth() {
  ComputeReductionFeatureLF();
  ComputeReductionFeatureKF();
  ComputeReductionPriorDepth();
}

void LocalBundleAdjustor::ComputeReductionFeatureLF() {
  float dFa, dFp;
  Rigid3D Tr[2];
  FTR::Reduction Ra, Rp;
  const int nLFs = int(m_LFs.size());
  for (int ic = 0; ic < nLFs; ++ic) {
    const int iLF = m_ic2LF[ic];
    const LocalFrame &LF = m_LFs[iLF];
    const Rigid3D &C = m_CsLF[iLF].m_T;
    const LA::ProductVector6f *xc = (m_ucsLF[iLF] & LBA_FLAG_FRAME_UPDATE_CAMERA) ? &m_xcsP[ic] : NULL;
    dFa = dFp = 0.0f;
    const int NZ = int(LF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = LF.m_Zs[iZ];
      const int iKF = Z.m_iKF;
      if (!xc && !(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH)) {
        continue;
      }
      *Tr = C / m_CsKF[iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
      const Depth::InverseGaussian *ds = m_ds.data() + id;
      const ubyte *uds = m_uds.data() + id;
      const float *xds = iX == -1 ? NULL : m_xds.Data() + iX;
      const KeyFrame &KF = m_KFs[iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = LF.m_zs[iz];
        const int ix = z.m_ix;
        const float *xd = xds && (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) ? &xds[ix] : NULL;
        if (!xc && !xd) {
          continue;
        }
        FTR::GetReduction(LF.m_Lzs[iz], Tr, KF.m_xs[ix], ds[ix], z, xc, xd, Ra, Rp);
        dFa = Ra.m_dF + dFa;
        dFp = Rp.m_dF + dFp;
//#ifdef CFG_DEBUG
#if 0
        if (m_iIter == 1 && ic == 1) {
          UT::Print("%d %e %e\n", iz, Ra.m_dF, Rp.m_dF);
        }
#endif
      }
    }
    m_dFa = dFa + m_dFa;
    m_dFp = dFp + m_dFp;
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 2)
      UT::Print("[%d] %f\n", F->m_T.m_iFrm, dFp);
#endif
  }
}

void LocalBundleAdjustor::ComputeReductionFeatureKF() {
  float dFa, dFp;
  Rigid3D Tr[2];
  FTR::Reduction Ra, Rp;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const Rigid3D &C = m_CsKF[iKF];
    dFa = dFp = 0.0f;
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      if (!(m_ucsKF[_iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH)) {
        continue;
      }
      *Tr = C / m_CsKF[_iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const int id = m_iKF2d[_iKF], iX = m_iKF2X[_iKF];
      const Depth::InverseGaussian *ds = m_ds.data() + id;
#ifdef CFG_DEBUG
      UT_ASSERT(iX != -1);
#endif
      const ubyte *uds = m_uds.data() + id;
      const float *xds = m_xds.Data() + iX;
      const KeyFrame &_KF = m_KFs[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH))
          continue;
        FTR::GetReduction(KF.m_Azs[iz], Tr, _KF.m_xs[ix], ds[ix], z, xds[ix], Ra, Rp);
        dFa = Ra.m_dF + dFa;
        dFp = Rp.m_dF + dFp;
      }
    }
    m_dFa = dFa + m_dFa;
    m_dFp = dFp + m_dFp;
//#ifdef CFG_DEBUG
#if 0
    if (m_iIter == 2)
      UT::Print("[%d] %f\n", F->m_T.m_iFrm, dFp);
#endif
  }
}

void LocalBundleAdjustor::ComputeReductionPriorDepth() {
  Depth::Prior::Reduction Ra, Rp;
#ifdef CFG_STEREO
  FTR::Reduction Rar, Rpr;
#endif
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (!(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH)) {
      continue;
    }
    const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
    const ubyte *uds = m_uds.data() + id;
    const Depth::InverseGaussian *ds = m_ds.data() + id;
#ifdef CFG_DEBUG
    UT_ASSERT(iX != -1);
#endif
    const float *xds = m_xds.Data() + iX;
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH))
        continue;
#ifdef CFG_STEREO
      if (KF.m_xs[ix].m_xr.Valid()) {
        FTR::GetReduction(KF.m_Ards[ix], m_K.m_br, ds[ix], KF.m_xs[ix], xds[ix], Rar, Rpr);
        m_dFa = Rar.m_dF + m_dFa;
        m_dFp = Rpr.m_dF + m_dFp;
      }
      else
#endif
      {
        const Depth::Prior _zp =
#ifdef CFG_DEPTH_MAP
          KF.m_xs[ix].m_d != 0.0f ? Depth::Prior(KF.m_xs[ix].m_d, DEPTH_MAP_WEIGHT) :
#endif
          zp;
        _zp.GetReduction(KF.m_Apds[ix], ds[ix].u(), xds[ix], Ra, Rp);
        m_dFa = Ra.m_dF + m_dFa;
        m_dFp = Rp.m_dF + m_dFp;
      }
    }
  }
}

void LocalBundleAdjustor::ComputeReductionPriorCameraMotion() {
  const int iLF = m_ic2LF.front();
  const ubyte ucm = m_ucmsLF[iLF];
  const bool ur = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION) != 0;
  const bool uv = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) != 0;
  const bool uba = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) != 0;
  const bool ubw = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) != 0;
  if (!ur && !uv && !uba && !ubw)
    return;
  CameraPrior::Motion::Reduction Ra, Rp;
  LA::AlignedVector3f xr, xv, xba, xbw;
  const int Nc = int(m_LFs.size());
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data(), &xc = xcs[0];
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc), &xm = xms[0];
  const LA::AlignedVector3f *_xr = ur ? &(xr = LA::AlignedVector3f(&xc.v3())) : NULL;
  const LA::AlignedVector3f *_xv = uv ? &(xv = LA::AlignedVector3f(&xm.v0())) : NULL;
  const LA::AlignedVector3f *_xba = uba ? &(xba = LA::AlignedVector3f(&xm.v3())) : NULL;
  const LA::AlignedVector3f *_xbw = ubw ? &(xbw = LA::AlignedVector3f(&xm.v6())) : NULL;
  m_ZpLF.GetReduction(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_ApLF, m_CsLF[iLF],
                      _xr, _xv, _xba, _xbw, &Ra, &Rp);
  m_dFa = Ra.m_dF + m_dFa;
  m_dFp = Rp.m_dF + m_dFp;
}

void LocalBundleAdjustor::ComputeReductionIMU() {
  IMU::Delta::Reduction Ra, Rp;
  LA::AlignedVector3f xp[2], xr[2], xv[2], xba[2], xbw[2];
  LA::AlignedVector3f *_xp[2], *_xr[2], *_xv[2], *_xba[2], *_xbw[2];
  int r1 = 0, r2 = 1;
  const ubyte ucFlag = LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                       LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  const int Nc = int(m_LFs.size());
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data();
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nc);
  for (int ic1 = 0, ic2 = 1; ic2 < Nc; ic1 = ic2++) {
    UT_SWAP(r1, r2);
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    if (ic1 == 0) {
      const ubyte ucm = m_ucmsLF[iLF1], uc = ucm & ucFlag;
      _xp[r1] = uc ? &(xp[r1] = LA::AlignedVector3f(&xcs[ic1].v0())) : NULL;
      _xr[r1] = uc ? &(xr[r1] = LA::AlignedVector3f(&xcs[ic1].v3())) : NULL;
      _xv[r1] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
                &(xv[r1] = LA::AlignedVector3f(&xms[ic1].v0())) : NULL;
      _xba[r1] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
                 &(xba[r1] = LA::AlignedVector3f(&xms[ic1].v3())) : NULL;
      _xbw[r1] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
                 &(xbw[r1] = LA::AlignedVector3f(&xms[ic1].v6())) : NULL;
    }
    const ubyte ucm = m_ucmsLF[iLF2], uc = ucm & ucFlag;
    _xp[r2] = uc ? &(xp[r2] = LA::AlignedVector3f(&xcs[ic2].v0())) : NULL;
    _xr[r2] = uc ? &(xr[r2] = LA::AlignedVector3f(&xcs[ic2].v3())) : NULL;
    _xv[r2] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
              &(xv[r2] = LA::AlignedVector3f(&xms[ic2].v0())) : NULL;
    _xba[r2] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
               &(xba[r2] = LA::AlignedVector3f(&xms[ic2].v3())) : NULL;
    _xbw[r2] = (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
               &(xbw[r2] = LA::AlignedVector3f(&xms[ic2].v6())) : NULL;
    if (!ucm && !m_ucmsLF[iLF1])
      continue;
//#ifdef CFG_DEBUG
#if 0
    m_DsLF[iLF2].m_W.m_Wr.MakeZero();
    m_DsLF[iLF2].m_W.m_Wv.MakeZero();
    //m_DsLF[iLF2].m_W.m_Wp.MakeZero();
#endif
    m_DsLF[iLF2].GetReduction(BA_WEIGHT_IMU, m_AdsLF[iLF2], m_CsLF[iLF1], m_CsLF[iLF2], m_K.m_pu,
                              _xp[r1], _xr[r1], _xv[r1], _xba[r1], _xbw[r1],
                              _xp[r2], _xr[r2], _xv[r2], _xba[r2], _xbw[r2], Ra, Rp);
//#ifdef CFG_DEBUG
#if 0
    UT::PrintSeparator();
    Rp.m_e.m_er.Print();
    Rp.m_e.m_ev.Print();
    Rp.m_e.m_ep.Print();
    const float F = m_DsLF[iLF2].GetCost(BA_WEIGHT_IMU, Rp.m_e);
#endif
//#ifdef CFG_DEBUG
#if 0
    UT::Print("(%d, %d): %f + %f = %f\n", ic1, ic2, Rp.m_dF, m_dFp, Rp.m_dF + m_dFp);
#endif
    m_dFa = Ra.m_dF + m_dFa;
    m_dFp = Rp.m_dF + m_dFp;
  }
}

void LocalBundleAdjustor::ComputeReductionFixOrigin() {
  const int iLF = m_ic2LF[0];
  if (m_LFs[iLF].m_T.m_iFrm != 0 || !(m_ucsLF[iLF] & LBA_FLAG_FRAME_UPDATE_CAMERA)) {
    return;
  }
  LA::AlignedVector3f xp, xr;
  Camera::Fix::Origin::Reduction Ra, Rp;
  m_xcsP[0].Get(xp, xr);
  m_Zo.GetReduction(m_Ao, m_CsLF[iLF].m_T, xp, xr, Ra, Rp);
  m_dFa = Ra.m_dF + m_dFa;
  m_dFp = Rp.m_dF + m_dFp;
}

void LocalBundleAdjustor::ComputeReductionFixPositionZ() {
  Camera::Fix::PositionZ::Reduction Ra, Rp;
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data();
  const int Nc = static_cast<int>(m_LFs.size());
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    if (!(m_ucmsLF[iLF] & LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION)) {
      continue;
    }
    const Camera::Fix::PositionZ::Factor &A = m_AfpsLF[iLF];
    z.GetReduction(A, m_CsLFBkp[iLF].m_p.z(), m_CsLF[iLF].m_p.z(), xcs[ic].v2(), Ra, Rp);
    m_dFa = Ra.m_dF + m_dFa;
    m_dFp = Rp.m_dF + m_dFp;
  }
}

void LocalBundleAdjustor::ComputeReductionFixMotion() {
  Camera::Fix::Zero::Reduction Ra, Rp;
  const Camera::Fix::Zero zv[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_VELOCITY_INITIAL)};
  const Camera::Fix::Zero zba[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_ACCELERATION_INITIAL)};
  const Camera::Fix::Zero zbw[2] = {
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE),
        Camera::Fix::Zero(BA_WEIGHT_FIX_MOTION, BA_VARIANCE_FIX_BIAS_GYROSCOPE_INITIAL)};
  const int pc = 6;
  const int Nc = static_cast<int>(m_LFs.size()), Ncp = Nc * pc;
  const LA::Vector9f *xms = (LA::Vector9f *) (m_xsDL.Data() + Ncp);
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    const Camera::Fix::Motion::Factor &A = m_AfmsLF[iLF];
    const Camera &C1 = m_CsLFBkp[iLF], &C2 = m_CsLF[iLF];
    const LA::Vector9f &xm = xms[ic];
    const int i = m_LFs[iLF].m_T.m_iFrm == 0 ? 1 : 0;
    const ubyte ucm = m_ucmsLF[iLF];
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
      zv[i].GetReduction(A.m_Av, C1.m_v, C2.m_v, &xm.v0(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
      zba[i].GetReduction(A.m_Aba, C1.m_ba, C2.m_ba, &xm.v3(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
    if (ucm & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
      zbw[i].GetReduction(A.m_Abw, C1.m_bw, C2.m_bw, &xm.v6(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
  }
}

bool LocalBundleAdjustor::UpdateStatesPropose() {
#ifdef CFG_VERBOSE
  int SNc = 0, SNm = 0, SNk = 0;
#endif
  LA::AlignedVector3f dp, dr, dv, dba, dbw, p;
  Rotation3D dR;
  m_CsLFBkp.Set(m_CsLF);
#ifdef CFG_INCREMENTAL_PCG
  m_xcsLFBkp.Set(m_xcsLF);
  m_xmsLFBkp.Set(m_xmsLF);
#endif
  m_update = false;
  m_converge = false;
  const int Nc = int(m_LFs.size());
  const LA::Vector6f *xcsDL = (LA::Vector6f *) m_xsDL.Data();
  const LA::Vector9f *xmsDL = (LA::Vector9f *) (xcsDL + Nc);
#ifdef CFG_INCREMENTAL_PCG
  const LA::Vector6f *xcsGN = (LA::Vector6f *) m_xsGN.Data();
  const LA::Vector9f *xmsGN = (LA::Vector9f *) (xcsGN + Nc);
#endif
  for (int ic = 0; ic < Nc; ++ic) {
    const int iLF = m_ic2LF[ic];
    Camera &C = m_CsLF[iLF];
    const bool ur = m_xr2s[ic] >= BA_UPDATE_ROTATION, up = m_xp2s[ic] >= BA_UPDATE_POSITION;
    if (ur || up) {
      xcsDL[ic].Get(dp, dr);
      dR.SetRodrigues(dr);
      C.m_T = Rotation3D(C.m_T) * dR;
      C.m_p += dp;
      C.m_T.SetPosition(C.m_p);
      m_ucsLF[iLF] |= LBA_FLAG_FRAME_UPDATE_CAMERA;
#ifdef CFG_INCREMENTAL_PCG
      m_xcsLF[iLF].MakeZero();
#endif
      m_update = true;
    } else {
      m_ucsLF[iLF] &= ~LBA_FLAG_FRAME_UPDATE_CAMERA;
#ifdef CFG_INCREMENTAL_PCG
      m_xcsLF[iLF] = xcsGN[ic];
#endif
    }
    ubyte &ucm = m_ucmsLF[iLF];
    if (ur) {
      ucm |= LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION;
    } else {
      ucm &= ~LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION;
    }
    if (up) {
      ucm |= LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
    } else {
      ucm &= ~LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
    }
    const LA::Vector9f &xmDL = xmsDL[ic];
#ifdef CFG_INCREMENTAL_PCG
    const LA::Vector9f &xmGN = xmsGN[ic];
    LA::Vector9f &xm = m_xmsLF[iLF];
#endif
    if (m_xv2s[ic] >= BA_UPDATE_VELOCITY) {
      xmDL.Get012(dv);
      C.m_v += dv;
      ucm |= LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero012();
#endif
      m_update = true;
    } else {
      ucm &= ~LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set012(&xmGN.v0());
#endif
    }
    if (m_xba2s[ic] >= BA_UPDATE_BIAS_ACCELERATION) {
      xmDL.Get345(dba);
      C.m_ba += dba;
      ucm |= LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero345();
#endif
      m_update = true;
    } else {
      ucm &= ~LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set345(&xmGN.v3());
#endif
    }
    if (m_xbw2s[ic] >= BA_UPDATE_BIAS_GYROSCOPE) {
      xmDL.Get678(dbw);
      C.m_bw += dbw;
      ucm |= LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero678();
#endif
      m_update = true;
    } else {
      ucm &= ~LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set678(&xmGN.v6());
#endif
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 3) {
      if (ur || up) {
        ++SNc;
      } else if (ucm & (LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                        LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
        ++SNm;
      }
    }
#endif
  }

#ifdef CFG_VERBOSE
  int SNx = 0, SNX = 0, SNu = 0, SNU = 0;
#endif
  m_axds.Resize(0);
  m_dsBkp.resize(m_xds.Size());
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    m_ucsKF[iKF] &= ~LBA_FLAG_FRAME_UPDATE_CAMERA;
    const int id = m_iKF2d[iKF];
    ubyte *uds = m_uds.data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    if (m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH) {
      m_ucsKF[iKF] &= ~LBA_FLAG_FRAME_UPDATE_DEPTH;
      for (int ix = 0; ix < Nx; ++ix) {
        uds[ix] &= ~LBA_FLAG_TRACK_UPDATE_DEPTH;
      }
    }
    if (!(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION)) {
      continue;
    }
    //m_ucsKF[iKF] &= ~LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION;
    const int iX = m_iKF2X[iKF];
    Depth::InverseGaussian *ds = m_ds.data() + id, *dsBkp = m_dsBkp.data() + iX;
    memcpy(dsBkp, ds, Nx * sizeof(Depth::InverseGaussian));
    const float *xds = m_xds.Data() + iX;
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        continue;
      }
      //uds[ix] &= ~LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION;
#ifdef CFG_VERBOSE
      if (m_verbose >= 3) {
        ++SNx;
      }
#endif
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) {
        continue;
      }
      const float axd = fabs(xds[ix]);
//#ifdef CFG_DEBUG
#if 0
      if (axd > 40.0f) {
        UT::DebugStart();
        UT::Print("[%d] %d %f\n", KF.m_T.m_iFrm, ix, xds[ix]);
        UT::DebugStop();
      }
#endif
      if (axd < BA_UPDATE_DEPTH) {
        continue;
      }
      Depth::InverseGaussian &d = ds[ix];
      d.u() += xds[ix];
      d.s2() = DEPTH_VARIANCE_EPSILON + KF.m_Mxs[ix].m_mdd.m_a * BA_WEIGHT_FEATURE;
      if (!d.Valid()) {
        d = dsBkp[ix];
        continue;
      }
      m_axds.Push(axd);
      m_ucsKF[iKF] |= LBA_FLAG_FRAME_UPDATE_DEPTH;
      uds[ix] |= LBA_FLAG_TRACK_UPDATE_DEPTH;
      m_update = true;
#ifdef CFG_VERBOSE
      if (m_verbose >= 3) {
        ++SNu;
      }
#endif
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 3) {
      ++SNX;
      if (m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH) {
        ++SNU;
      }
    }
#endif
  }
#ifdef CFG_VERBOSE
  if (m_verbose >= 3) {
    const int Nd = int(m_ds.size());
    UT::PrintSeparator();
    UT::Print("*%2d: [LocalBundleAdjustor::UpdateStates]\n", m_iIter);
    UT::Print("  Camera = %d / %d = %.2f%%\n", SNc, Nc, UT::Percentage(SNc, Nc));
    UT::Print("         + %d / %d = %.2f%%\n", SNm, Nc, UT::Percentage(SNm, Nc));
    UT::Print("  Depth = %3d / %5d = %.2f%% (%d / %d = %.2f%%)\n", SNx, Nd, UT::Percentage(SNx, Nd),
              SNX, nKFs, UT::Percentage(SNX, nKFs));
    UT::Print("      --> %3d / %5d = %.2f%% (%d / %d = %.2f%%)\n", SNu, SNx, UT::Percentage(SNu, SNx),
              SNU, SNX, UT::Percentage(SNU, SNX));
  }
#endif
  return m_update;
}

bool LocalBundleAdjustor::UpdateStatesDecide() {
  const int nLFs = int(m_LFs.size()), nKFs = int(m_KFs.size());
  if (m_update && m_rho < BA_DL_GAIN_RATIO_MIN) {
    m_delta2 *= BA_DL_RADIUS_FACTOR_DECREASE;
    if (m_delta2 < BA_DL_RADIUS_MIN) {
      m_delta2 = BA_DL_RADIUS_MIN;
    }
    m_CsLF.Swap(m_CsLFBkp);
#ifdef CFG_INCREMENTAL_PCG
    m_xcsLF.Swap(m_xcsLFBkp);
    m_xmsLF.Swap(m_xmsLFBkp);
#endif
    for (int iLF = 0; iLF < nLFs; ++iLF) {
      m_ucsLF[iLF] &= ~LBA_FLAG_FRAME_UPDATE_CAMERA;
      m_ucmsLF[iLF] = LBA_FLAG_CAMERA_MOTION_DEFAULT;
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (!(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH)) {
        continue;
      }
      m_ucsKF[iKF] &= ~LBA_FLAG_FRAME_UPDATE_DEPTH;
      const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
      const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
      memcpy(m_ds.data() + id, m_dsBkp.data() + iX, Nx * sizeof(Depth::InverseGaussian));
      ubyte *uds = m_uds.data() + id;
      for (int ix = 0; ix < Nx; ++ix) {
        uds[ix] &= ~LBA_FLAG_TRACK_UPDATE_DEPTH;
      }
    }
    m_update = false;
    m_converge = false;
    return false;
  } else if (m_rho > BA_DL_GAIN_RATIO_MAX) {
    //m_delta2 *= BA_DL_RADIUS_FACTOR_INCREASE;
    m_delta2 = std::max(m_delta2, BA_DL_RADIUS_FACTOR_INCREASE * m_x2DL);
    if (m_delta2 > BA_DL_RADIUS_MAX) {
      m_delta2 = BA_DL_RADIUS_MAX;
    }
  }
  for (int iLF = 0; iLF < nLFs; ++iLF) {
    if (m_ucmsLF[iLF]) {
      m_UcsLF[iLF] |= LM_FLAG_FRAME_UPDATE_CAMERA_LF;
    }
  }
  for (int ic1 = 0, ic2 = 1; ic2 < nLFs; ic1 = ic2++) {
    const int iLF1 = m_ic2LF[ic1], iLF2 = m_ic2LF[ic2];
    if (!(m_ucmsLF[iLF1] & LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
      continue;
    }
    IMU::Delta &D = m_DsLF[iLF2];
    //if (ic1 == 5) {
    //  UT::DebugStart();
    //}
    IMU::PreIntegrate(m_LFs[iLF2].m_us, m_LFs[iLF1].m_T.m_t, m_LFs[iLF2].m_T.m_t, m_CsLF[iLF1], &D,
                      &m_work, true, D.m_u1.Valid() ? &D.m_u1 : NULL, D.m_u2.Valid() ? &D.m_u2 : NULL);
#ifdef LBA_DEBUG_GROUND_TRUTH_MEASUREMENT
    if (!m_CsLFGT.Empty()) {
      D.DebugSetMeasurement(m_CsLFGT[iLF1], m_CsLFGT[iLF2], m_K.m_pu);
    }
#endif
    //if (UT::Debugging()) {
    //  UT::DebugStop();
    //}
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_CAMERA) {
      m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_CAMERA_KF;
    }
    if (!(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION)) {
      continue;
    }
    m_ucsKF[iKF] &= ~LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION;
    const int id = m_iKF2d[iKF];
    ubyte *uds = m_uds.data() + id;
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      uds[ix] &= ~LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION;
    }
    if (!(m_ucsKF[iKF] & LBA_FLAG_FRAME_UPDATE_DEPTH)) {
      continue;
    }
    m_UcsKF[iKF] |= LM_FLAG_FRAME_UPDATE_DEPTH;
    ubyte *Uds = m_Uds.data() + id;
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_DEPTH) {
        Uds[ix] |= LM_FLAG_TRACK_UPDATE_DEPTH;
      }
    }
  }
  m_converge = m_xr2s.Maximal() < BA_CONVERGE_ROTATION && m_xp2s.Maximal() < BA_CONVERGE_POSITION &&
               m_xv2s.Maximal() < BA_CONVERGE_VELOCITY && m_xba2s.Maximal() < BA_CONVERGE_BIAS_ACCELERATION &&
               m_xbw2s.Maximal() < BA_CONVERGE_BIAS_GYROSCOPE &&
               m_axds.Maximal() < BA_CONVERGE_DEPTH;
  return true;
}

void LocalBundleAdjustor::ConvertCameraUpdates(const float *xcs, LA::AlignedVectorXf *xp2s,
                                               LA::AlignedVectorXf *xr2s) {
  const int pc = 6;
  const int Nc = int(m_LFs.size()), Ncp = Nc * pc;
  m_x2s.Set(xcs, Ncp);
  m_x2s.MakeSquared();
  const LA::Vector6f *xc2s = (LA::Vector6f *) m_x2s.Data();
  xp2s->Resize(Nc);
  xr2s->Resize(Nc);
  float *_xp2s = xp2s->Data(), *_xr2s = xr2s->Data();
  for (int ic = 0; ic < Nc; ++ic) {
    const LA::Vector6f &xc2 = xc2s[ic];
    _xp2s[ic] = xc2.v0() + xc2.v1() + xc2.v2();
    _xr2s[ic] = xc2.v3() + xc2.v4() + xc2.v5();
  }
}

void LocalBundleAdjustor::ConvertCameraUpdates(const LA::Vector6f *xcs,
                                               AlignedVector<LA::ProductVector6f> *xcsP) {
  const int Nc = int(m_LFs.size());
  xcsP->Resize(Nc);
  LA::ProductVector6f *_xcsP = xcsP->Data();
  for (int ic = 0; ic < Nc; ++ic) {
    _xcsP[ic].Set(xcs[ic]);
  }
}

void LocalBundleAdjustor::ConvertCameraUpdates(const AlignedVector<LA::AlignedVector6f> &xcsA,
                                               LA::Vector6f *xcs) {
  const int pc = 6;
  const int Nc = xcsA.Size();
  for (int ic = 0; ic < Nc; ++ic) {
    xcsA[ic].Get(xcs[ic]);
  }
}

void LocalBundleAdjustor::ConvertMotionUpdates(const float *xms, LA::AlignedVectorXf *xv2s,
                                               LA::AlignedVectorXf *xba2s,
                                               LA::AlignedVectorXf *xbw2s) {
  const int pm = 9;
  const int Nc = static_cast<int>(m_LFs.size()), Nmp = Nc * pm;
  m_x2s.Set(xms, Nmp);
  m_x2s.MakeSquared();
  const LA::Vector9f *xm2s = (LA::Vector9f *) m_x2s.Data();
  xv2s->Resize(Nc);
  xba2s->Resize(Nc);
  xbw2s->Resize(Nc);
  float *_xv2s = xv2s->Data(), *_xba2s = xba2s->Data(), *_xbw2s = xbw2s->Data();
  for (int ic = 0; ic < Nc; ++ic) {
    const LA::Vector9f &xm2 = xm2s[ic];
    _xv2s[ic] = xm2.v0() + xm2.v1() + xm2.v2();
    _xba2s[ic] = xm2.v3() + xm2.v4() + xm2.v5();
    _xbw2s[ic] = xm2.v6() + xm2.v7() + xm2.v8();
  }
}

void LocalBundleAdjustor::ConvertCameraMotionResiduals(const LA::AlignedVectorXf &rs,
                                                       const LA::AlignedVectorXf &zs,
                                                       float *Se2, float *e2Max) {
  const int N = rs.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(zs.Size() == N);
#endif
  m_x2s.Resize(N);
  SIMD::Multiply(N, rs.Data(), zs.Data(), m_x2s.Data());
  if (LBA_PCG_CONDITIONER_BAND <= 1) {
    *Se2 = 0.0f;
    *e2Max = 0.0f;
#ifdef CFG_PCG_FULL_BLOCK
    const int Nc = static_cast<int>(m_LFs.size());
    const LA::Vector6f *e2cs = (LA::Vector6f *) m_x2s.Data();
    for (int ic = 0; ic < Nc; ++ic) {
      const float e2 = e2cs[ic].Sum();
#ifdef CFG_DEBUG
      UT_ASSERT(e2 >= 0.0f);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
    const LA::Vector9f *e2ms = (LA::Vector9f *) (e2cs + Nc);
    for (int ic = 0; ic < Nc; ++ic) {
      const float e2 = e2ms[ic].Sum();
#ifdef CFG_DEBUG
      UT_ASSERT(e2 >= 0.0f);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
#else
    const LA::Vector3f *e2s = (LA::Vector3f *) m_x2s.Data();
    const int _N = N / 3;
#ifdef CFG_DEBUG
    UT_ASSERT(N == _N * 3);
#endif
    for (int i = 0; i < _N; ++i) {
      const float e2 = e2s[i].Sum();
      // TODO (haomin): Bad condition!!!
#ifdef CFG_DEBUG
//#if 0
      UT_ASSERT(e2 >= 0.0f);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
#endif
  } else {
    *Se2 = *e2Max = m_x2s.Sum();
#ifdef CFG_DEBUG
    UT_ASSERT(*Se2 >= 0.0f);
#endif
  }
}

void LocalBundleAdjustor::ConvertDepthUpdates(const float *xs, LA::AlignedVectorXf *xds) {
  int i = 0;
  const int Nd = m_xds.Size();
  xds->Resize(Nd);
  const int nKFs = int(m_KFs.size());
#ifdef CFG_DEBUG
  int SN = 0;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (m_iKF2X[iKF] != -1) {
      SN += int(m_KFs[iKF].m_xs.size());
    }
  }
  UT_ASSERT(Nd == SN);
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1)
      continue;
    float *_xds = xds->Data() + iX;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        _xds[ix] = xs[i++];
      }
    }
  }
}

void LocalBundleAdjustor::PushDepthUpdates(const LA::AlignedVectorXf &xds,
                                           LA::AlignedVectorXf *xs) {
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const float *_xds = xds.Data() + iX;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        xs->Push(_xds[ix]);
      }
    }
  }
}
