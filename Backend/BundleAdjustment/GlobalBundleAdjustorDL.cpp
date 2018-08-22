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

#if defined WIN32 && defined CFG_DEBUG && defined CFG_GROUND_TRUTH
//#define GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
#endif

void GlobalBundleAdjustor::SolveDogLeg() {
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
    LA::AlignedVectorXf::AmB(m_xsGN, m_xsGD, m_xsDL);
    const float d = m_xsGD.Dot(m_xsDL), dx2 = m_xsDL.SquaredLength();
    //m_beta = static_cast<float>((-d + sqrt(static_cast<double>(d) * d +
    //                            (m_delta2 - m_x2GD) * static_cast<double>(dx2))) / dx2);
    m_beta = (-d + sqrtf(d * d + (m_delta2 - m_x2GD) * dx2)) / dx2;
    m_xsDL *= m_beta;
    m_xsDL += m_xsGD;
    m_x2DL = m_delta2;
  }
  const int pc = 6, pm = 9;
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size(), Ncp = Nc * pc, Nmp = Nm * pm, Ncmp = Ncp + Nmp;
  ConvertCameraUpdates(m_xsDL.Data(), &m_xp2s, &m_xr2s);
  ConvertMotionUpdates(m_xsDL.Data() + Ncp, &m_xv2s, &m_xba2s, &m_xbw2s);
  ConvertDepthUpdates(m_xsDL.Data() + Ncmp, &m_xds);
}

void GlobalBundleAdjustor::SolveGradientDescent() {
  m_xsGD.Resize(0);
  const int pc = 6, pm = 9;
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size(), Nd = m_xds.Size();
  const int Ncp = Nc * pc, Nmp = Nm * pm, Ncmp = Ncp + Nmp;
  for (int ic = 0; ic < Nc; ++ic) {
    m_xsGD.Push(m_SAcus[ic].m_b, 6);
  }
  for (int im = 0; im < Nm; ++im) {
    m_xsGD.Push(m_SAcmsLM[im].m_Au.m_Amm.m_b, 9);
  }
  for (int iKF = 0; iKF < Nc; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        m_xsGD.Push(KF.m_Axs[ix].m_Sadx.m_add.m_b);
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
    const LA::ProductVector6f &gc = m_gcs[ic];
    float *Agc = Agcs[ic];
    m_SAcus[ic].m_A.GetAlignedMatrix6x6f(A66);
    LA::AlignedMatrix6x6f::Ab(A66, gc, Agc);
    const KeyFrame &KF = m_KFs[ic];
    const int Np = KF.m_SAps.Size();
    for (int ip = 0; ip < Np; ++ip) {
      const int _ic = KF.m_iKFsPrior[ip];
      LA::AlignedMatrix6x6f::AddAbTo(KF.m_SAps[ip], gc, (float *) &Agcs[_ic]);
      KF.m_SAps[ip].GetTranspose(A66);
      LA::AlignedMatrix6x6f::AddAbTo(A66, m_gcs[_ic], Agc);
    }
  }
  const int ic0 = Nc - Nm;
  ApplyAcm(m_gcs.Data() + ic0, (LA::Vector9f *) (m_xsGD.Data() + Ncp), Agcs + ic0,
           (LA::Vector9f *) (m_Ags.Data() + Ncp));

  LA::AlignedVector6f Agc;
  LA::AlignedVector7f gcd;
  gcd.MakeZero();
  m_Agds.Resize(Nd);
  for (int ic = 0; ic < Nc; ++ic) {
    const int iX = m_iKF2X[ic];
    if (iX == -1) {
      continue;
    }
    gcd.Set(m_gcs[ic]);
    Agc.Set(Agcs[ic]);
    const float *gds = m_gds.Data() + iX;
    float *Agds = m_Agds.Data() + iX;
    const ubyte *uds = m_uds.data() + m_iKF2d[ic];
    const KeyFrame &KF = m_KFs[ic];
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        continue;
      }
      const FTR::Factor::DDC &Sadx = KF.m_Axs[ix].m_Sadx;
      gcd.v6() = gds[ix];
      Agds[ix] = Sadx.m_adcd.Dot(gcd);
//#ifdef CFG_DEBUG
#if 0
      if (m_iIter == 1 && ic == 0)
        UT::Print("%d: %e * %e + %e = %e\n", ix, gds[ix], Sadx.m_adc.v0(), Agc.v0(), gds[ix] * Sadx.m_adc.v0() + Agc.v0());
#endif
      LA::AlignedVector6f::AddsaTo(gds[ix], Sadx.m_adcA, Agc);
    }
    Agc.Get(Agcs[ic]);
  }
  for (int ic = 0; ic < Nc; ++ic) {
    const LA::ProductVector6f &gc = m_gcs[ic];
    Agc.Set(Agcs[ic]);
    const KeyFrame &KF = m_KFs[ic];
    const int NZ = static_cast<int>(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
#ifdef CFG_DEBUG
      UT_ASSERT(_iKF < ic);
#endif
      LA::AlignedMatrix6x6f::AddAbTo(KF.m_SAcxzs[iZ], gc, (float *) &Agcs[_iKF]);
      KF.m_SAcxzs[iZ].GetTranspose(A66);
      LA::AlignedMatrix6x6f::AddAbTo(A66, m_gcs[_iKF], (float *) &Agc);
      const int _iX = m_iKF2X[_iKF];
      if (_iX == -1) {
        continue;
      }
      const float *_gds = m_gds.Data() + _iX;
      float *_Agds = m_Agds.Data() + _iX;
      const ubyte *_uds = m_uds.data() + m_iKF2d[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const int ix = KF.m_zs[iz].m_ix;
        if (!(_uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION))
          continue;
        const LA::AlignedVector6f &adcz = KF.m_Azs1[iz].m_adczA;
        _Agds[ix] = adcz.Dot(gc) + _Agds[ix];
        LA::AlignedVector6f::AddsaTo(_gds[ix], adcz, Agc);
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

void GlobalBundleAdjustor::ComputeReduction() {
  m_dFa = m_dFp = 0.0f;
  if (m_update) {
    ConvertCameraUpdates((LA::Vector6f *) m_xsDL.Data(), &m_xcsP);
    ComputeReductionFeature();
    ComputeReductionPriorCameraPose();
    ComputeReductionPriorCameraMotion();
    ComputeReductionPriorDepth();
    ComputeReductionIMU();
    ComputeReductionFixOrigin();
    ComputeReductionFixPositionZ();
    ComputeReductionFixMotion();
    m_rho = m_dFa > 0.0f && m_dFp > 0.0f ? m_dFa / m_dFp : -1.0f;
  } else {
    m_rho = 0.0f;
  }
}

void GlobalBundleAdjustor::ComputeReductionFeature() {
//#ifdef CFG_DEBUG
#if 0
  UT::DebugStart();
  UT::DebugStop();
  return;
#endif
  float dFa, dFp;
  Rigid3D Tr[2];
  FTR::Reduction Ra, Rp;
  const ubyte ucFlag = GBA_FLAG_FRAME_UPDATE_CAMERA | GBA_FLAG_FRAME_UPDATE_DEPTH;
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const KeyFrame &KF = m_KFs[iKF];
    const Rigid3D &C = m_Cs[iKF];
    const LA::ProductVector6f *xc = (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA) ?
                                    &m_xcsP[iKF] : NULL;
    dFa = dFp = 0.0f;
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const FRM::Measurement &Z = KF.m_Zs[iZ];
      const int _iKF = Z.m_iKF;
      if (!xc && !(m_ucs[_iKF] & ucFlag)) {
        continue;
      }
      *Tr = C / m_Cs[_iKF];
#ifdef CFG_STEREO
      Tr[1] = Tr[0];
      Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
      const int id = m_iKF2d[_iKF];
      const Depth::InverseGaussian *ds = m_ds.data() + id;
      const ubyte *uds = m_uds.data() + id;
      const LA::ProductVector6f *_xc = (m_ucs[_iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA) ?
                                       &m_xcsP[_iKF] : NULL;
      const bool ucr = xc || _xc;
      const float *xds = m_iKF2X[_iKF] == -1 ? NULL : m_xds.Data() + m_iKF2X[_iKF];
      const KeyFrame &_KF = m_KFs[_iKF];
      for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
        const FTR::Measurement &z = KF.m_zs[iz];
        const int ix = z.m_ix;
        const float *xd = xds && (uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH) ? &xds[ix] : NULL;
        if (!ucr && !xd) {
          continue;
        }
        FTR::GetReduction(KF.m_Lzs[iz], Tr, _KF.m_xs[ix], ds[ix], z, _xc, xc, xd, Ra, Rp);
        dFa = Ra.m_dF + dFa;
        dFp = Rp.m_dF + dFp;
      }
    }
    m_dFa = dFa + m_dFa;
    m_dFp = dFp + m_dFp;
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging() && m_iIter == 0) {
      if (iKF == 0)
        UT::PrintSeparator();
      UT::Print("%d %e\n", iKF, dFa);
    }
#endif
  }
}

void GlobalBundleAdjustor::ComputeReductionPriorCameraPose() {
  const int Nc = m_xcsP.Size();
  m_xps.Resize(Nc);   m_xpsP.resize(Nc);
  m_xrs.Resize(Nc);   m_xrsP.resize(Nc);
  for (int ic = 0; ic < Nc; ++ic) {
    if (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA) {
      m_xcsP[ic].Get(m_xps[ic], m_xrs[ic]);
      m_xpsP[ic] = &m_xps[ic];
      m_xrsP[ic] = &m_xrs[ic];
    } else {
      m_xpsP[ic] = NULL;
      m_xrsP[ic] = NULL;
    }
  }
  CameraPrior::Pose::Reduction Ra, Rp;
  const int NZ = static_cast<int>(m_Zps.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const CameraPrior::Pose &Z = m_Zps[iZ];
    bool uc = (m_ucs[Z.m_iKFr] & GBA_FLAG_FRAME_UPDATE_CAMERA) != 0;
    const int N = static_cast<int>(Z.m_iKFs.size());
    for (int i = 0; i < N && !uc; ++i) {
      uc = (m_ucs[Z.m_iKFs[i]] & GBA_FLAG_FRAME_UPDATE_CAMERA) != 0;
    }
    if (!uc) {
      continue;
    }
    m_work.Resize((Ra.BindSize(N) + Rp.BindSize(N)) / sizeof(float));
    Ra.Bind(m_work.Data(), N);
    Rp.Bind(Ra.BindNext(), N);
    //Z.GetReduction(BA_WEIGHT_PRIOR_CAMERA_POSE, m_Aps[iZ], m_Cs, m_xpsP, m_xrsP, &Ra, &Rp,
    //               BA_ANGLE_EPSILON);
    Z.GetReduction(m_Aps[iZ].m_w, m_Aps[iZ], m_Cs, m_xpsP, m_xrsP, &Ra, &Rp,
                   BA_ANGLE_EPSILON);
    m_dFa = Ra.m_dF + m_dFa;
    m_dFp = Rp.m_dF + m_dFp;
  }
}

void GlobalBundleAdjustor::ComputeReductionPriorCameraMotion() {
  if (m_ZpLM.Invalid()) {
    return;
  }
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size(), im = m_ZpLM.m_iKF - Nc + Nm;
  const ubyte ucm = m_ucmsLM[im];
  const bool ur = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION) != 0;
  const bool uv = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) != 0;
  const bool uba = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) != 0;
  const bool ubw = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) != 0;
  if (!ur && !uv && !uba && !ubw) {
    return;
  }
  CameraPrior::Motion::Reduction Ra, Rp;
  LA::AlignedVector3f xr, xv, xba, xbw;
  const LA::Vector6f *xcs = ((LA::Vector6f *) m_xsDL.Data()) + Nc - Nm, &xc = xcs[im];
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nm), &xm = xms[im];
  const LA::AlignedVector3f *_xr = ur ? &(xr = LA::AlignedVector3f(&xc.v3())) : NULL;
  const LA::AlignedVector3f *_xv = uv ? &(xv = LA::AlignedVector3f(&xm.v0())) : NULL;
  const LA::AlignedVector3f *_xba = uba ? &(xba = LA::AlignedVector3f(&xm.v3())) : NULL;
  const LA::AlignedVector3f *_xbw = ubw ? &(xbw = LA::AlignedVector3f(&xm.v6())) : NULL;
  m_ZpLM.GetReduction(BA_WEIGHT_PRIOR_CAMERA_MOTION, m_ApLM, m_CsLM[im], _xr, _xv, _xba, _xbw, &Ra, &Rp);
  m_dFa = Ra.m_dF + m_dFa;
  m_dFp = Rp.m_dF + m_dFp;
}

void GlobalBundleAdjustor::ComputeReductionPriorDepth() {
//#ifdef CFG_DEBUG
#if 0
  UT::DebugStart();
  UT::DebugStop();
  return;
#endif
  Depth::Prior::Reduction Ra, Rp;
#ifdef CFG_STEREO
  FTR::Reduction Rar, Rpr;
#endif
  const int nKFs = int(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (!(m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DEPTH)) {
      continue;
    }
    const int id = m_iKF2d[iKF];
    const ubyte *uds = m_uds.data() + id;
    const Depth::InverseGaussian *ds = m_ds.data() + id;
#ifdef CFG_DEBUG
    UT_ASSERT(m_iKF2X[iKF] != -1);
#endif
    const float *xds = m_xds.Data() + m_iKF2X[iKF];
    const KeyFrame &KF = m_KFs[iKF];
    const Depth::Prior zp(KF.m_d.u(), 1.0f / (BA_VARIANCE_PRIOR_FRAME_DEPTH + KF.m_d.s2()));
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH)) {
        continue;
      }
#ifdef CFG_STEREO
      if (KF.m_xs[ix].m_xr.Valid()) {
        FTR::GetReduction(KF.m_Ards[ix], m_K.m_br, ds[ix], KF.m_xs[ix], xds[ix], Rar, Rpr);
        m_dFa = Rar.m_dF + m_dFa;
        m_dFp = Rpr.m_dF + m_dFp;
      }
      else
#endif
      {
        zp.GetReduction(KF.m_Apds[ix], ds[ix].u(), xds[ix], Ra, Rp);
        m_dFa = Ra.m_dF + m_dFa;
        m_dFp = Rp.m_dF + m_dFp;
      }
    }
  }
}

void GlobalBundleAdjustor::ComputeReductionIMU() {
  IMU::Delta::Reduction Ra, Rp;
  LA::AlignedVector3f xp[2], xr[2], xv[2], xba[2], xbw[2];
  LA::AlignedVector3f *_xp[2], *_xr[2], *_xv[2], *_xba[2], *_xbw[2];
  int r1 = 0, r2 = 1;
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size();
  const LA::Vector6f *xcs = ((LA::Vector6f *) m_xsDL.Data()) + Nc - Nm;
  const LA::Vector9f *xms = (LA::Vector9f *) (xcs + Nm);
  const ubyte ucFlag = GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                       GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
  const ubyte ucmFlag = ucFlag |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                        GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
  for (int ic1 = Nc - Nm, ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc;
       ic1 = ic2++, im1 = im2++) {
    UT_SWAP(r1, r2);
    if (im1 == 0) {
      const ubyte ucm = m_ucmsLM[im1], uc = ucm & ucFlag;
      _xp[r1] = uc ? &(xp[r1] = LA::AlignedVector3f(&xcs[im1].v0())) : NULL;
      _xr[r1] = uc ? &(xr[r1] = LA::AlignedVector3f(&xcs[im1].v3())) : NULL;
      _xv[r1] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
                &(xv[r1] = LA::AlignedVector3f(&xms[im1].v0())) : NULL;
      _xba[r1] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
                 &(xba[r1] = LA::AlignedVector3f(&xms[im1].v3())) : NULL;
      _xbw[r1] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
                 &(xbw[r1] = LA::AlignedVector3f(&xms[im1].v6())) : NULL;
    }
    const ubyte ucm = m_ucmsLM[im2], uc = ucm & ucFlag;
    _xp[r2] = uc ? &(xp[r2] = LA::AlignedVector3f(&xcs[im2].v0())) : NULL;
    _xr[r2] = uc ? &(xr[r2] = LA::AlignedVector3f(&xcs[im2].v3())) : NULL;
    _xv[r2] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) ?
              &(xv[r2] = LA::AlignedVector3f(&xms[im2].v0())) : NULL;
    _xba[r2] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) ?
               &(xba[r2] = LA::AlignedVector3f(&xms[im2].v3())) : NULL;
    _xbw[r2] = (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) ?
               &(xbw[r2] = LA::AlignedVector3f(&xms[im2].v6())) : NULL;
    if (m_KFs[ic2].m_us.Empty() || (!(ucm & ucmFlag) && !(m_ucmsLM[im1] & ucmFlag))) {
      continue;
    }
    m_DsLM[im2].GetReduction(BA_WEIGHT_IMU, m_AdsLM[im2], m_CsLM[im1], m_CsLM[im2], m_K.m_pu,
                             _xp[r1], _xr[r1], _xv[r1], _xba[r1], _xbw[r1],
                             _xp[r2], _xr[r2], _xv[r2], _xba[r2], _xbw[r2], Ra, Rp,
                             BA_ANGLE_EPSILON);
    m_dFa = Ra.m_dF + m_dFa;
    m_dFp = Rp.m_dF + m_dFp;
  }
}

void GlobalBundleAdjustor::ComputeReductionFixOrigin() {
  const int iKF = 0;
  if (m_KFs[iKF].m_T.m_iFrm != 0 || !(m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA)) {
    return;
  }
  LA::AlignedVector3f xp, xr;
  Camera::Fix::Origin::Reduction Ra, Rp;
  m_xcsP[iKF].Get(xp, xr);
  m_Zo.GetReduction(m_Ao, m_Cs[iKF], xp, xr, Ra, Rp, BA_ANGLE_EPSILON);
  m_dFa = Ra.m_dF + m_dFa;
  m_dFp = Rp.m_dF + m_dFp;
}

void GlobalBundleAdjustor::ComputeReductionFixPositionZ() {
  Camera::Fix::PositionZ::Reduction Ra, Rp;
  const Camera::Fix::PositionZ z(BA_WEIGHT_FIX_POSITION_Z, BA_VARIANCE_FIX_POSITION_Z);
  const LA::Vector6f *xcs = (LA::Vector6f *) m_xsDL.Data();
  const int nKFs = static_cast<int>(m_KFs.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (!(m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_CAMERA)) {
      continue;
    }
    const Camera::Fix::PositionZ::Factor &A = m_Afps[iKF];
    z.GetReduction(A, m_CsBkp[iKF].GetPositionZ(), m_Cs[iKF].GetPositionZ(), xcs[iKF].v2(),
                   Ra, Rp);
    m_dFa = Ra.m_dF + m_dFa;
    m_dFp = Rp.m_dF + m_dFp;
  }
}

void GlobalBundleAdjustor::ComputeReductionFixMotion() {
  Camera::Fix::Zero::Reduction Ra, Rp;
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
  const int pc = 6;
  const int Nc = m_Cs.Size(), Ncp = Nc * pc;
  const LA::Vector9f *xms = (LA::Vector9f *) (m_xsDL.Data() + Ncp);
  for (int ic = Nc - m_CsLM.Size(), im = 0; ic < Nc; ++ic, ++im) {
    const Camera::Fix::Motion::Factor &A = m_AfmsLM[im];
    const Camera &C1 = m_CsLMBkp[im], &C2 = m_CsLM[im];
    const LA::Vector9f &xm = xms[im];
    const ubyte ucm = m_ucmsLM[im];
    const int i = ucm >> 5;
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY) {
      zv[i].GetReduction(A.m_Av, C1.m_v, C2.m_v, &xm.v0(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION) {
      zba[i].GetReduction(A.m_Aba, C1.m_ba, C2.m_ba, &xm.v3(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
    if (ucm & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE) {
      zbw[i].GetReduction(A.m_Abw, C1.m_bw, C2.m_bw, &xm.v6(), Ra, Rp);
      m_dFa = Ra.m_dF + m_dFa;
      m_dFp = Rp.m_dF + m_dFp;
    }
  }
}

bool GlobalBundleAdjustor::UpdateStatesPropose() {
#ifdef CFG_VERBOSE
  int SNc = 0, SNm = 0;
#endif
  LA::AlignedVector3f dp, dr, dv, dba, dbw, p;
  Rotation3D dR;
  m_CsBkp.Set(m_Cs);
  m_CsLMBkp.Set(m_CsLM);
#ifdef CFG_INCREMENTAL_PCG
  m_xcsBkp.Set(m_xcs);
  m_xmsLMBkp.Set(m_xmsLM);
#endif
  m_update = false;
  m_converge = false;
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size();
  const LA::Vector6f *xcsDL = (LA::Vector6f *) m_xsDL.Data();
  const LA::Vector9f *xmsDL = (LA::Vector9f *) (xcsDL + Nc);
#ifdef CFG_INCREMENTAL_PCG
  const LA::Vector6f *xcsGN = (LA::Vector6f *) m_xsGN.Data();
  const LA::Vector9f *xmsGN = (LA::Vector9f *) (xcsGN + Nc);
#endif
  for (int ic = 0, im = Nm - Nc; ic < Nc; ++ic, ++im) {
    Rigid3D &C = m_Cs[ic];
    const bool ur = m_xr2s[ic] >= BA_UPDATE_ROTATION, up = m_xp2s[ic] >= BA_UPDATE_POSITION;
    if (ur || up) {
      xcsDL[ic].Get(dp, dr);
      //dp.z() = 0.0f;
      dR.SetRodrigues(dr, BA_ANGLE_EPSILON);
      C.GetPosition(p);
      C = Rotation3D(C) * dR;
      p += dp;
      C.SetPosition(p);
      m_ucs[ic] |= GBA_FLAG_FRAME_UPDATE_CAMERA;
#ifdef CFG_INCREMENTAL_PCG
      m_xcs[ic].MakeZero();
#endif
      m_update = true;
#ifdef CFG_VERBOSE
      if (m_verbose >= 3)
        ++SNc;
#endif
    } else {
      m_ucs[ic] &= ~GBA_FLAG_FRAME_UPDATE_CAMERA;
#ifdef CFG_INCREMENTAL_PCG
      m_xcs[ic] = xcsGN[ic];
#endif
    }
    if (im < 0) {
      continue;
    }
    Camera &_C = m_CsLM[im];
    if (ur || up) {
      _C.m_T = C;
      _C.m_p = p;
    }
    ubyte &ucm = m_ucmsLM[im];
    if (ur) {
      ucm |= GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION;
    } else {
      ucm &= ~GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION;
    }
    if (up) {
      ucm |= GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
    } else {
      ucm &= ~GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION;
    }
    const LA::Vector9f &xmDL = xmsDL[im];
#ifdef CFG_INCREMENTAL_PCG
    const LA::Vector9f &xmGN = xmsGN[im];
    LA::Vector9f &xm = m_xmsLM[im];
#endif
    if (m_xv2s[im] >= BA_UPDATE_VELOCITY) {
      xmDL.Get012(dv);
      _C.m_v += dv;
      ucm |= GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero012();
#endif
      m_update = true;
    } else {
      ucm &= ~GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set012(&xmGN.v0());
#endif
    }
    if (m_xba2s[im] >= BA_UPDATE_BIAS_ACCELERATION) {
      xmDL.Get345(dba);
      _C.m_ba += dba;
      ucm |= GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero345();
#endif
      m_update = true;
    } else {
      ucm &= ~GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set345(&xmGN.v3());
#endif
    }
    if (m_xbw2s[im] >= BA_UPDATE_BIAS_GYROSCOPE) {
      xmDL.Get678(dbw);
      _C.m_bw += dbw;
      ucm |= GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
#ifdef CFG_INCREMENTAL_PCG
      xm.MakeZero678();
#endif
      m_update = true;
    } else {
      ucm &= ~GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
#ifdef CFG_INCREMENTAL_PCG
      xm.Set678(&xmGN.v6());
#endif
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 3) {
      if (ucm & (GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                 GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                 GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
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
  for (int iKF = 0; iKF < Nc; ++iKF) {
    const int id = m_iKF2d[iKF];
    ubyte *uds = m_uds.data() + id;
    const KeyFrame &KF = m_KFs[iKF];
    const int Nx = static_cast<int>(KF.m_xs.size());
    if (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DEPTH) {
      m_ucs[iKF] &= ~GBA_FLAG_FRAME_UPDATE_DEPTH;
      for (int ix = 0; ix < Nx; ++ix) {
        uds[ix] &= ~GBA_FLAG_TRACK_UPDATE_DEPTH;
      }
    }
    if (!(m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION)) {
      continue;
    }
    //m_ucs[iKF] &= ~GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION;
    const int iX = m_iKF2X[iKF];
    Depth::InverseGaussian *ds = m_ds.data() + id, *dsBkp = m_dsBkp.data() + iX;
    memcpy(dsBkp, ds, Nx * sizeof(Depth::InverseGaussian));
    const float *xds = m_xds.Data() + iX;
    for (int ix = 0; ix < Nx; ++ix) {
      if (!(uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION)) {
        continue;
      }
      //uds[ix] &= ~GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION;
#ifdef CFG_VERBOSE
      if (m_verbose >= 3) {
        ++SNx;
      }
#endif
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO) {
        continue;
      }
      const float axd = fabs(xds[ix]);
      if (axd < BA_UPDATE_DEPTH) {
        continue;
      }
      Depth::InverseGaussian &d = ds[ix];
      d.u() = xds[ix] + d.u();
      d.s2() = DEPTH_VARIANCE_EPSILON + KF.m_Mxs1[ix].m_mdx.m_add.m_a * BA_WEIGHT_FEATURE;
      //if (!d.Valid()) {
      //  d = dsBkp[ix];
      //  continue;
      //}
      //d.u() = UT_CLAMP(d.u(), DEPTH_MIN, DEPTH_MAX);
      //d.u() = UT_CLAMP(d.u(), DEPTH_EPSILON, DEPTH_MAX);
      //if (d.u() < DEPTH_EPSILON) {
      //  d.u() = DEPTH_EPSILON;
      //} else if (d.u() > DEPTH_MAX) {
      //  d.u() = KF.m_d.u();
      //}
      //if (!d.Valid()) {
      //  d.u() = KF.m_d.u();
      //}
      m_axds.Push(axd);
      m_ucs[iKF] |= GBA_FLAG_FRAME_UPDATE_DEPTH;
      uds[ix] |= GBA_FLAG_TRACK_UPDATE_DEPTH;
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
      if (m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DEPTH) {
        ++SNU;
      }
    }
#endif
  }
#ifdef CFG_VERBOSE
  if (m_verbose >= 3) {
    const int Nd = int(m_ds.size());
    UT::PrintSeparator();
    UT::Print("*%2d: [GlobalBundleAdjustor::UpdateStates]\n", m_iIter);
    UT::Print("  Camera = %d / %d = %.2f%%\n", SNc, Nc, UT::Percentage(SNc, Nc));
    UT::Print("         + %d / %d = %.2f%%\n", SNm, Nm, UT::Percentage(SNm, Nm));
    UT::Print("  Depth = %3d / %5d = %.2f%% (%d / %d = %.2f%%)\n", SNx, Nd, UT::Percentage(SNx, Nd),
              SNX, Nc, UT::Percentage(SNX, Nc));
    UT::Print("      --> %3d / %5d = %.2f%% (%d / %d = %.2f%%)\n", SNu, SNx, UT::Percentage(SNu, SNx),
              SNU, SNX, UT::Percentage(SNU, SNX));
  }
#endif
  return m_update;
}

bool GlobalBundleAdjustor::UpdateStatesDecide() {
  const int Nc = m_Cs.Size(), Nm = m_CsLM.Size();
  if (m_update && m_rho < BA_DL_GAIN_RATIO_MIN) {
    m_delta2 *= BA_DL_RADIUS_FACTOR_DECREASE;
    if (m_delta2 < BA_DL_RADIUS_MIN) {
      m_delta2 = BA_DL_RADIUS_MIN;
    }
    m_Cs.Swap(m_CsBkp);
    m_CsLM.Swap(m_CsLMBkp);
#ifdef CFG_INCREMENTAL_PCG
    m_xcs.Swap(m_xcsBkp);
    m_xmsLM.Swap(m_xmsLMBkp);
#endif
    for (int ic = 0; ic < Nc; ++ic) {
      m_ucs[ic] &= ~GBA_FLAG_FRAME_UPDATE_CAMERA;
    }
    //m_ucmsLM.assign(Nm, GBA_FLAG_CAMERA_MOTION_DEFAULT);
    const ubyte ucmFlag = GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION |
                          GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION |
                          GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY |
                          GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION |
                          GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE;
    for (int im = 0; im < Nm; ++im) {
      m_ucmsLM[im] &= ~ucmFlag;
    }
    for (int iKF = 0; iKF < Nc; ++iKF) {
      if (!(m_ucs[iKF] & GBA_FLAG_FRAME_UPDATE_DEPTH)) {
        continue;
      }
      m_ucs[iKF] &= ~GBA_FLAG_FRAME_UPDATE_DEPTH;
      const KeyFrame &KF = m_KFs[iKF];
      const int id = m_iKF2d[iKF], iX = m_iKF2X[iKF];
      const int Nx = static_cast<int>(KF.m_xs.size());
      memcpy(m_ds.data() + id, m_dsBkp.data() + iX, Nx * sizeof(Depth::InverseGaussian));
      ubyte *uds = m_uds.data() + id;
      for (int ix = 0; ix < Nx; ++ix) {
        uds[ix] &= ~GBA_FLAG_TRACK_UPDATE_DEPTH;
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
  for (int ic = 0; ic < Nc; ++ic) {
    if (m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_CAMERA) {
      m_Ucs[ic] |= GM_FLAG_FRAME_UPDATE_CAMERA;
    }
    if (!(m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION)) {
      continue;
    }
    m_ucs[ic] &= ~GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION;
    const int id = m_iKF2d[ic];
    ubyte *uds = m_uds.data() + id;
    const int Nx = static_cast<int>(m_KFs[ic].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      uds[ix] &= ~GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION;
    }
#ifdef CFG_HANDLE_SCALE_JUMP
    if (!(m_ucs[ic] & GBA_FLAG_FRAME_UPDATE_DEPTH)) {
      continue;
    }
    m_Ucs[ic] |= GM_FLAG_FRAME_UPDATE_DEPTH;
    //ubyte *Uds = m_Uds.data() + id;
    //for (int ix = 0; ix < Nx; ++ix) {
    //  if (uds[ix] & GBA_FLAG_TRACK_UPDATE_DEPTH) {
    //    Uds[ix] |= GM_FLAG_TRACK_UPDATE_DEPTH;
    //  }
    //}
    m_dsKF[ic] = AverageDepths(m_ds.data() + id, Nx);
#endif
  }
  for (int ic1 = Nc - Nm, ic2 = ic1 + 1, im1 = 0, im2 = 1; ic2 < Nc;
       ic1 = ic2++, im1 = im2++) {
    const KeyFrame &KF2 = m_KFs[ic2];
    if (!KF2.m_us.Empty() && (m_ucmsLM[im1] & GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE)) {
//#ifdef CFG_DEBUG
#if 0
      if (im1 == 83)
        UT::DebugStart();
#endif
      IMU::Delta &D = m_DsLM[im2];
      IMU::PreIntegrate(m_KFs[ic2].m_us, m_KFs[ic1].m_T.m_t, m_KFs[ic2].m_T.m_t, m_CsLM[im1], &D,
                        &m_work, true, D.m_u1.Valid() ? &D.m_u1 : NULL,
                        D.m_u2.Valid() ? &D.m_u2 : NULL, BA_ANGLE_EPSILON);
#ifdef GBA_DEBUG_GROUND_TRUTH_MEASUREMENT
      if (!m_CsLMGT.Empty()) {
        D.DebugSetMeasurement(m_CsLMGT[im1], m_CsLMGT[im2], m_K.m_pu, BA_ANGLE_EPSILON);
      }
#endif
//#ifdef CFG_DEBUG
#if 0
      if (UT::Debugging())
        UT::DebugStop();
#endif
    }
  }
  m_converge = m_xr2s.Maximal() < BA_CONVERGE_ROTATION && m_xp2s.Maximal() < BA_CONVERGE_POSITION &&
               m_xv2s.Maximal() < BA_CONVERGE_VELOCITY && m_xba2s.Maximal() < BA_CONVERGE_BIAS_ACCELERATION &&
               m_xbw2s.Maximal() < BA_CONVERGE_BIAS_GYROSCOPE &&
               m_axds.Maximal() < BA_CONVERGE_DEPTH;
  return true;
}

void GlobalBundleAdjustor::ConvertCameraUpdates(const float *xcs, LA::AlignedVectorXf *xp2s,
                                                LA::AlignedVectorXf *xr2s) {
  const int pc = 6;
  const int Nc = m_Cs.Size(), Ncp = Nc * pc;
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

void GlobalBundleAdjustor::ConvertCameraUpdates(const LA::Vector6f *xcs,
                                                AlignedVector<LA::ProductVector6f> *xcsP) {
  const int Nc = m_Cs.Size();
  xcsP->Resize(Nc);
  LA::ProductVector6f *_xcsP = xcsP->Data();
  for (int ic = 0; ic < Nc; ++ic) {
    _xcsP[ic].Set(xcs[ic]);
  }
}

void GlobalBundleAdjustor::ConvertCameraUpdates(const LA::Vector6d *xcs,
                                                AlignedVector<LA::ProductVector6f> *xcsP) {
  const int Nc = m_Cs.Size();
  xcsP->Resize(Nc);
  LA::ProductVector6f *_xcsP = xcsP->Data();
  for (int ic = 0; ic < Nc; ++ic) {
    _xcsP[ic].Set(xcs[ic]);
  }
}

void GlobalBundleAdjustor::ConvertCameraUpdates(const AlignedVector<LA::AlignedVector6f> &xcsA,
                                                LA::Vector6f *xcs) {
  const int Nc = xcsA.Size();
  for (int ic = 0; ic < Nc; ++ic) {
    xcsA[ic].Get(xcs[ic]);
  }
}

void GlobalBundleAdjustor::ConvertMotionUpdates(const float *xms, LA::AlignedVectorXf *xv2s,
                                                LA::AlignedVectorXf *xba2s,
                                                LA::AlignedVectorXf *xbw2s) {
  const int pm = 9;
  const int Nm = m_CsLM.Size(), Nmp = Nm * pm;
  m_x2s.Set(xms, Nmp);
  m_x2s.MakeSquared();
  const LA::Vector9f *xm2s = (LA::Vector9f *) m_x2s.Data();
  xv2s->Resize(Nm);
  xba2s->Resize(Nm);
  xbw2s->Resize(Nm);
  float *_xv2s = xv2s->Data(), *_xba2s = xba2s->Data(), *_xbw2s = xbw2s->Data();
  for (int im = 0; im < Nm; ++im) {
    const LA::Vector9f &xm2 = xm2s[im];
    _xv2s[im] = xm2.v0() + xm2.v1() + xm2.v2();
    _xba2s[im] = xm2.v3() + xm2.v4() + xm2.v5();
    _xbw2s[im] = xm2.v6() + xm2.v7() + xm2.v8();
  }
}

void GlobalBundleAdjustor::ConvertCameraMotionResiduals(const LA::AlignedVectorX<PCG_TYPE> &rs,
                                                        const LA::AlignedVectorX<PCG_TYPE> &zs,
                                                        PCG_TYPE *Se2, PCG_TYPE *e2Max) {
  const int N = rs.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(zs.Size() == N);
#endif
  m_e2s.Resize(N);
  SIMD::Multiply(N, rs.Data(), zs.Data(), m_e2s.Data());
  *Se2 = 0;
  *e2Max = 0;
  if (GBA_PCG_CONDITIONER_BAND <= 1) {
#ifdef CFG_PCG_FULL_BLOCK
    const int Nc = m_Cs.Size();
    const LA::Vector6<PCG_TYPE> *e2cs = (LA::Vector6<PCG_TYPE> *) m_e2s.Data();
    for (int ic = 0; ic < Nc; ++ic) {
      const PCG_TYPE e2 = e2cs[ic].Sum();
#ifdef CFG_DEBUG
      UT_ASSERT(e2 >= 0.0f);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
    const LA::Vector9<PCG_TYPE> *e2ms = (LA::Vector9<PCG_TYPE> *) (e2cs + Nc);
    const int Nm = m_CsLM.Size();
    for (int im = 0; im < Nm; ++im) {
      const PCG_TYPE e2 = e2ms[im].Sum();
#ifdef CFG_DEBUG
      UT_ASSERT(e2 >= 0.0f);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
#else
    const LA::Vector3<PCG_TYPE> *e2s = (LA::Vector3<PCG_TYPE> *) m_e2s.Data();
    const int _N = m_Cs.Size() * 2 + m_CsLM.Size() * 3;
#ifdef CFG_DEBUG
    UT_ASSERT(N == _N * 3);
#endif
    for (int i = 0; i < _N; ++i) {
      const PCG_TYPE e2 = e2s[i].Sum();
#ifdef CFG_DEBUG
//#if 0
      UT_ASSERT(e2 >= 0);
#endif
      *Se2 = e2 + *Se2;
      *e2Max = std::max(e2, *e2Max);
    }
#endif
  } else {
    *Se2 = *e2Max = m_e2s.Sum();
#ifdef CFG_DEBUG
    UT_ASSERT(*Se2 >= 0.0f);
#endif
  }
}

void GlobalBundleAdjustor::ConvertDepthUpdates(const float *xs, LA::AlignedVectorXf *xds) {
  int i = 0;
  const int Nd = m_xds.Size();
  xds->Resize(Nd);
  const int nKFs = int(m_KFs.size());
#ifdef CFG_DEBUG
  int SN = 0;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (m_iKF2X[iKF] != -1)
      SN += int(m_KFs[iKF].m_xs.size());
  }
  UT_ASSERT(Nd == SN);
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int iX = m_iKF2X[iKF];
    if (iX == -1) {
      continue;
    }
    float *_xds = xds->Data() + iX;
    const ubyte *uds = m_uds.data() + m_iKF2d[iKF];
    const int Nx = static_cast<int>(m_KFs[iKF].m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        _xds[ix] = xs[i++];
      }
    }
  }
}

void GlobalBundleAdjustor::PushDepthUpdates(const LA::AlignedVectorXf &xds,
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
      if (uds[ix] & GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION) {
        xs->Push(_xds[ix]);
      }
    }
  }
}

float GlobalBundleAdjustor::AverageDepths(const Depth::InverseGaussian *ds, const int N) {
  const int NC = SIMD_FLOAT_CEIL(N);
  m_work.Resize(NC + N);
  LA::AlignedVectorXf us(m_work.Data(), N, false);
  LA::AlignedVectorXf ws(us.Data() + NC, N, false);
  for (int i = 0; i < N; ++i) {
    const Depth::InverseGaussian &d = ds[i];
    us[i] = d.u();
    ws[i] = d.s2();
  }
  ws += DEPTH_VARIANCE_EPSILON;
  ws.MakeInverse();
  const float Sw = ws.Sum();
  if (Sw < FLT_EPSILON) {
    ws.Set(1.0f / N);
  } else {
    ws *= 1.0f / Sw;
  }
  us *= ws;
  return us.Sum();
}
