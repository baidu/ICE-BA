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
#include "IBA.h"
#include "IBA_internal.h"

#ifndef _WIN32
#include <boost/filesystem.hpp>
#endif

namespace IBA {

static inline void ConvertCamera(const CameraIMUState &Ci, Camera *Co) {
  if (Ci.C.R[0][0] == FLT_MAX) {
    Co->Invalidate();
    return;
  }
  Co->m_T.LA::AlignedMatrix3x3f::Set(Ci.C.R);
  Co->m_p.Set(Ci.C.p);
  Co->m_T.SetPosition(Co->m_p);
  Co->m_v.Set(Ci.v);
  Co->m_ba.Set(Ci.ba);
  Co->m_bw.Set(Ci.bw);
}

static inline void ConvertCamera(const CameraPose &Ci, Rigid3D *Co) {
  if (Ci.R[0][0] == FLT_MAX) {
    Co->Invalidate();
    return;
  }
  Co->LA::AlignedMatrix3x3f::Set(Ci.R);
  const ::Point3D p = Ci.p;
  Co->SetPosition(p);
}

static inline void ConvertCamera(const Camera &Ci, CameraIMUState *Co) {
  Ci.m_T.LA::AlignedMatrix3x3f::Get(Co->C.R);
  Ci.m_p.Get(Co->C.p);
  Ci.m_v.Get(Co->v);
  Ci.m_ba.Get(Co->ba);
  Ci.m_bw.Get(Co->bw);
}

static inline void ConvertDepth(const Depth &di, ::Depth::InverseGaussian *_do) {
  _do->u() = di.d;
  _do->s2() = di.s2;
}

static inline void ConvertDepth(const ::Depth::InverseGaussian &di, Depth *_do) {
  _do->d = di.u();
  _do->s2 = di.s2();
}

Solver::Solver() {
  m_internal = NULL;
}

Solver::~Solver() {
#ifdef CFG_DEBUG
  UT_ASSERT(m_internal == NULL);
#endif
}

void Solver::Create(const Calibration &K, const int serial, const int verbose, const int debug,
                    const int history, const std::string param, const std::string dir,
                    const std::vector<CameraIMUState> *XsGT, const std::vector<Depth> *dsGT) {
#ifdef CFG_DEBUG
  UT_ASSERT(m_internal == NULL);
#endif
  m_internal = new Internal();
#ifdef CFG_DEBUG
  UT_ASSERT(m_internal != NULL);
#ifndef WIN32
  UT_ASSERT((reinterpret_cast<std::uintptr_t>(m_internal) & (SIMD_ALIGN - 1)) == 0);
#endif
#endif
  Camera::Calibration &_K = m_internal->m_K;
  _K.m_K.Set(K.w, K.h, K.K.fx, K.K.fy, K.K.cx, K.K.cy, K.K.ds, K.fishEye);
  const Rigid3D Tu(K.Tu);
  _K.m_Ru = Tu;
  _K.m_pu = Tu.GetTranslation();
#ifdef CFG_STEREO
  const Rigid3D Tr(K.Tr);
  _K.m_Kr.Set(K.w, K.h, K.Kr.fx, K.Kr.fy, K.Kr.cx, K.Kr.cy, K.Kr.ds, K.fishEye,
              //K.K.fx, K.K.fy,
              _K.m_K.fx(), _K.m_K.fy(),
              &Tr);
  _K.m_Rr = Tr;
  _K.m_br = Tr.GetTranslation().GetMinus();
#endif
  _K.m_ba.Set(K.ba);
  _K.m_bw.Set(K.bw);
  //_K.m_sa.Set(K.sa);
  LoadParameters(param);
  AlignedVector<Camera> &CsGT = m_internal->m_CsGT;
  if (XsGT) {
    const int N = static_cast<int>(XsGT->size());
    CsGT.Resize(N);
    for (int i = 0; i < N; ++i) {
      Camera &C = CsGT[i];
      ConvertCamera(XsGT->at(i), &C);
      C.m_ba -= _K.m_ba;
      C.m_bw -= _K.m_bw;
    }
  } else {
    CsGT.Clear();
  }
  std::vector<::Depth::InverseGaussian> &DsGT = m_internal->m_DsGT;
  if (dsGT) {
    const int N = static_cast<int>(dsGT->size());
    DsGT.resize(N);
    for (int i = 0; i < N; ++i) {
      ConvertDepth(dsGT->at(i), &DsGT[i]);
    }
  } else {
    DsGT.clear();
  }
  m_internal->m_dir = dir;
  m_internal->m_LBA.Initialize(this, serial & 255, verbose & 255,
                               static_cast<int>(static_cast<char>(debug & 255)),
                               static_cast<int>(static_cast<char>(history & 255)));
  m_internal->m_GBA.Initialize(this, (serial >> 8) & 255, (verbose >> 8) & 255,
                               static_cast<int>(static_cast<char>((debug >> 8) & 255)),
                               static_cast<int>(static_cast<char>((history >> 8) & 255)));
}

void Solver::Destroy() {
  delete m_internal;
  m_internal = NULL;
}

void Solver::Start() {
  m_internal->m_LBA.Start();
  m_internal->m_GBA.Start();
  m_internal->m_nFrms = 0;
  m_internal->m_Ts.resize(0);
  m_internal->m_LM.IBA_Reset();
  m_internal->m_iFrmsKF.resize(0);
  m_internal->m_iKF2d.assign(1, 0);
  m_internal->m_id2X.resize(0);
  m_internal->m_iX2d.resize(0);
  m_internal->m_id2idx.resize(0);
  m_internal->m_idx2iX.resize(0);
  m_internal->m_CsKF.Resize(0);
  m_internal->m_xs.resize(0);
  m_internal->m_ds.resize(0);
#ifdef CFG_GROUND_TRUTH
  m_internal->m_dsGT.resize(0);
  m_internal->m_tsGT.Resize(0);
  m_internal->m_zsGT.resize(0);
#endif
}

void Solver::Stop() {
  m_internal->m_LBA.Stop();
  m_internal->m_GBA.Stop();
}

void Solver::PushCurrentFrame(const CurrentFrame &CF, const KeyFrame *KF) {
  if (KF) {
    //UT::Print("[%d]\n", KF->iFrm);
    if (KF->iFrm == CF.iFrm) {
      m_internal->PushCurrentFrame(CF);
      m_internal->PushKeyFrame(*KF, &m_internal->m_ILF.m_C);
    } else {
      m_internal->PushKeyFrame(*KF);
      m_internal->PushCurrentFrame(CF);
    }
    m_internal->m_LM.IBA_PushCurrentFrame(LocalMap::CameraLF(m_internal->m_ILF.m_C, CF.iFrm),
                                          m_internal->m_IKF);
    m_internal->m_LBA.PushCurrentFrame(m_internal->m_ILF, m_internal->m_IKF);
  } else {
    const LocalBundleAdjustor::InputLocalFrame &ILF = m_internal->PushCurrentFrame(CF);
    m_internal->m_LM.IBA_PushCurrentFrame(LocalMap::CameraLF(ILF.m_C, CF.iFrm));
    m_internal->m_LBA.PushCurrentFrame(ILF);
  }
  m_internal->m_LBA.WakeUp();
}

bool Solver::PushRelativeConstraint(const RelativeConstraint &Z) {
  if (Z.iFrm1 == -1 || Z.iFrm2 == -1) {
    return false;
  }
#ifdef CFG_DEBUG
  UT_ASSERT(Z.iFrm1 < Z.iFrm2);
#endif
  const std::vector<int> &iFrmsKF = m_internal->m_iFrmsKF;
  const std::vector<int>::const_iterator i1 = std::lower_bound(iFrmsKF.begin(), iFrmsKF.end(),
                                                               Z.iFrm1);
  if (i1 == iFrmsKF.end() || *i1 != Z.iFrm1) {
    return false;
  }
  const std::vector<int>::const_iterator i2 = std::lower_bound(i1, iFrmsKF.end(), Z.iFrm2);
  if (i2 == iFrmsKF.end() || *i2 != Z.iFrm2) {
    return false;
  }
  Rigid3D T;
  LA::AlignedVector3f p;
  T.Rotation3D::Set(Z.T.R);
  p.Set(Z.T.p);
  T.SetPosition(p);
  LA::AlignedMatrix6x6f S;
  S.Set(Z.S);
  CameraPrior::Pose _Z;
  if (!_Z.Initialize(BA_WEIGHT_PRIOR_CAMERA_RELATIVE_CONSTRAINT,
                     static_cast<int>(i1 - iFrmsKF.begin()),
                     static_cast<int>(i2 - iFrmsKF.begin()), T, S)) {
    return false;
  }
  //////////////////////////////////////////////////////////////////////////
  //const int verboseBkp = m_internal->m_GBA.m_verbose;
  //m_internal->m_GBA.m_verbose = 2;
  //////////////////////////////////////////////////////////////////////////
  m_internal->m_GBA.PushCameraPriorPose(Z.iFrm2, _Z);
  m_internal->m_GBA.WakeUp();
  //////////////////////////////////////////////////////////////////////////
  //m_internal->m_GBA.m_verbose = verboseBkp;
  //////////////////////////////////////////////////////////////////////////
  return true;
}

#ifdef CFG_GROUND_TRUTH
static inline void OffsetPointer(const LA::AlignedVector3f *t0,
                                 const AlignedVector<LA::AlignedVector3f> &ts,
                                 std::vector<std::vector<::Depth::Measurement> > *zs) {
  if (ts.Data() == t0) {
    return;
  }
  const int NX = static_cast<int>(zs->size());
  for (int iX = 0; iX < NX; ++iX) {
    std::vector<::Depth::Measurement> &_zs = zs->at(iX);
    const int N = static_cast<int>(_zs.size());
    for (int i = 0; i < N; ++i) {
      ::Depth::Measurement &z = _zs[i];
      //const int it1 = static_cast<int>(z.m_t - t0);
      z.m_t = ts.Data() + static_cast<int>(z.m_t - t0);
      //const int it2 = static_cast<int>(z.m_t - ts.Data());
#ifdef CFG_DEBUG
      UT_ASSERT(z.m_t >= ts.Data() && z.m_t < ts.End());
#endif
    }
  }
}
static inline int PushTranslation(const LA::AlignedVector3f &t,
                                  AlignedVector<LA::AlignedVector3f> *ts,
                                  std::vector<std::vector<::Depth::Measurement> > *zs) {
  const int it = ts->Size();
  if (it + 1 > ts->Capacity()) {
    const LA::AlignedVector3f *t0 = ts->Data();
    ts->Push(t);
    OffsetPointer(t0, *ts, zs);
  } else {
    ts->Push(t);
  }
  return it;
}

void Solver::PushDepthMeasurementsGT(const CurrentFrame &CF, const KeyFrame *KF,
                                     const bool keyframeOnly) {
  const AlignedVector<Camera> &CsGT = m_internal->m_CsGT;
  if (CsGT.Empty()) {
    return;
  }
  const LocalBundleAdjustor::InputLocalFrame &ILF = m_internal->PushCurrentFrame(CF);
  if (KF) {
    //UT::Print("[%d]\n", KF->iFrm);
    const GlobalMap::InputKeyFrame &IKF = m_internal->PushKeyFrame(*KF);

    //float e;
    int it;
    LA::Vector3f Rx;
    const std::vector<int> &iFrmsKF = m_internal->m_iFrmsKF, &iKF2d = m_internal->m_iKF2d;
    std::vector<::Depth::InverseGaussian> &dsGT = m_internal->m_dsGT;
    AlignedVector<LA::AlignedVector3f> &tsGT = m_internal->m_tsGT;
    std::vector<std::vector<::Depth::Measurement> > &zsGT = m_internal->m_zsGT;
    AlignedVector<float> &work = m_internal->m_work;
    AlignedVector<Rotation3D> &Rs = m_internal->m_RsGT;
    std::vector<int> &its = m_internal->m_idxsTmp;
#ifdef CFG_STEREO
    const LA::AlignedVector3f &br = m_internal->m_K.m_br;
    if (tsGT.Empty()) {
      tsGT.Push(br);
    }
#endif
    const int N = static_cast<int>(IKF.m_Xs.size());
    for (int i1 = 0, i2 = 0; i1 < N; i1 = i2) {
      const int iKF = IKF.m_Xs[i1].m_iKF;
      const Rigid3D &C = CsGT[iFrmsKF[iKF]].m_T;
      const int it0 = tsGT.Size();
      Rs.Resize(0);
      const int nKFs = static_cast<int>(iFrmsKF.size());
      its.assign(nKFs, -1);
      for (i2 = i1 + 1; i2 < N && IKF.m_Xs[i2].m_iKF == iKF; ++i2) {}
      const GlobalMap::Point *Xs = IKF.m_Xs.data() + i1;
      const int Nx = i2 - i1, id = iKF2d[iKF + 1] - Nx;
      dsGT.insert(dsGT.begin() + id, Nx, ::Depth::InverseGaussian());
      ::Depth::InverseGaussian *ds = dsGT.data() + id;
      zsGT.insert(zsGT.begin() + id, Nx, std::vector<::Depth::Measurement>());
      std::vector<::Depth::Measurement> *zs = zsGT.data() + id;
      for (int i = 0; i < Nx; ++i) {
        const GlobalMap::Point &X = Xs[i];
        ::Depth::InverseGaussian &d = ds[i];
        std::vector<::Depth::Measurement> &_zs = zs[i];
        d.Initialize();
#ifdef CFG_STEREO
        const FTR::Source &x = X.m_x;
        if (x.m_xr.Valid()) {
          Rx.Set(x.m_x.x(), x.m_x.y(), 1.0f);
          _zs.push_back(::Depth::Measurement(tsGT[0], Rx, x.m_xr, x.m_Wr));
          ::Depth::Triangulate(1.0f, static_cast<int>(_zs.size()), _zs.data(), &d, &work);
        }
#endif
        const int Nz = static_cast<int>(X.m_zs.size());
        for (int j = 0; j < Nz; ++j) {
          const FTR::Measurement &z = X.m_zs[j];
          if ((it = its[z.m_iKF]) == -1) {
            const Rigid3D T = CsGT[iFrmsKF[z.m_iKF]].m_T / C;
            Rs.Push(T);
            const LA::AlignedVector3f t = T.GetTranslation();
            it = its[z.m_iKF] = PushTranslation(t, &tsGT, &zsGT);
#ifdef CFG_STEREO
            const LA::AlignedVector3f tr = t + br;
            PushTranslation(tr, &tsGT, &zsGT);
#endif
          }
          const int iR = (it - it0)
#ifdef CFG_STEREO
                       >> 1;
#endif
#ifdef CFG_DEBUG
//#if 0
          const Rigid3D T = CsGT[iFrmsKF[z.m_iKF]].m_T / C;
          const LA::AlignedVector3f t = T.GetTranslation();
          const LA::AlignedVector3f tr = t + br;
          Rs[iR].AssertEqual(T);
          tsGT[it].AssertEqual(t);
          tsGT[it + 1].AssertEqual(tr);
#endif
          Rs[iR].Apply(x.m_x, Rx);
#ifdef CFG_STEREO
          if (z.m_z.Valid()) {
            _zs.push_back(::Depth::Measurement(tsGT[it], Rx, z.m_z, z.m_W));
          }
          if (z.m_zr.Valid()) {
            _zs.push_back(::Depth::Measurement(tsGT[it + 1], Rx, z.m_zr, z.m_Wr));
          }
#else
          _zs.push_back(::Depth::Measurement(tsGT[it], Rx, z.m_z, z.m_W));
#endif
          ::Depth::Triangulate(1.0f, static_cast<int>(_zs.size()), _zs.data(),
                               &d, &work, true, true/*, &e*/);
          //const int iX = DsGT.size() - 1;
          //UT::Print("iX = %d d = %f e = %f [%d]\n", iX, d.u(),
          //          e * sqrtf(m_internal->m_K.m_K.fxy()), iFrmsKF[z.m_iKF]);
        }
      }
    }
    m_internal->PushDepthMeasurementsGT(IKF);
  }
  if (!keyframeOnly) {
    m_internal->PushDepthMeasurementsGT(ILF);
  }
}

void Internal::PushDepthMeasurementsGT(const FRM::Frame &F) {
  //float e;
  LA::Vector3f Rx;
  const int NZ = static_cast<int>(F.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const int iFrmKF = m_iFrmsKF[Z.m_iKF];
#ifdef CFG_DEBUG
    UT_ASSERT(iFrmKF < F.m_T.m_iFrm);
#endif
    const Rigid3D T = m_CsGT[F.m_T.m_iFrm].m_T / m_CsGT[iFrmKF].m_T;
    const LA::AlignedVector3f t = T.GetTranslation();
    const int it = PushTranslation(t, &m_tsGT, &m_zsGT);
#ifdef CFG_STEREO
    const LA::AlignedVector3f tr = t + m_K.m_br;
    PushTranslation(tr, &m_tsGT, &m_zsGT);
#endif
    const int id = m_iKF2d[Z.m_iKF];
    const ::Point2D *xs = m_xs.data() + id;
    ::Depth::InverseGaussian *ds = m_dsGT.data() + id;
    std::vector<::Depth::Measurement> *zs = m_zsGT.data() + id;
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const FTR::Measurement &z = F.m_zs[iz];
      const int ix = z.m_ix;
      std::vector<::Depth::Measurement> &_zs = zs[ix];
      T.ApplyRotation(xs[ix], Rx);
#ifdef CFG_STEREO
      if (z.m_z.Valid()) {
        _zs.push_back(::Depth::Measurement(m_tsGT[it], Rx, z.m_z, z.m_W));
      }
      if (z.m_zr.Valid()) {
        _zs.push_back(::Depth::Measurement(m_tsGT[it + 1], Rx, z.m_zr, z.m_Wr));
      }
#else
      _zs.push_back(::Depth::Measurement(m_tsGT[it], Rx, z.m_z, z.m_W));
#endif
      ::Depth::Triangulate(1.0f, static_cast<int>(_zs.size()), _zs.data(),
                           &ds[ix], &m_work, true, true/*, &e*/);
      //UT::Print("iX = %d d = %f e = %f [%d]\n", iX, m_DsGT[iX].u(),
      //          e * sqrtf(m_K.m_K.fxy()), F.m_T.m_iFrm);
    }
  }
}

void Solver::TriangulateDepthsGT(std::vector<Depth> *dsGT) {
  float e;
  LA::AlignedVectorXf es;
  const int N = static_cast<int>(m_internal->m_idx2iX.size());
  dsGT->resize(N);
  es.Reserve(N);
  int SNd = 0;
  for (int idx = 0; idx < N; ++idx) {
    Depth &d = dsGT->at(idx);
    const int iX = m_internal->m_idx2iX[idx];
    if (iX == -1) {
      d.d = d.s2 = 0.0f;
    } else {
      const int id = m_internal->m_iX2d[iX];
      ::Depth::InverseGaussian &_d = m_internal->m_dsGT[id];
      const std::vector<::Depth::Measurement> &zs = m_internal->m_zsGT[id];
      const int Nz = static_cast<int>(zs.size());
      if (::Depth::Triangulate(1.0f, Nz, zs.data(), &_d, &m_internal->m_work, true, true, &e)) {
        es.Push(e);
        ConvertDepth(_d, &d);
      } else {
        d.d = d.s2 = 0.0f;
      }
      if (Nz > 0) {
        ++SNd;
      }
    }
  }
  es *= sqrtf(m_internal->m_K.m_K.fxy());
  const int Nv = es.Size();
  UT::Print("Success %d / %d = %f%%\n", Nv, SNd, UT::Percentage(Nv, SNd));
  UT::Print("Error = %f +- %f <= %f\n", es.Mean(), sqrtf(es.Variance()), es.Maximal());
  //#ifdef CFG_DEBUG
#if 0
  es.Save("D:/tmp/es.txt");
  //std::sort(es.Data(), es.End());
#endif
}
#endif

void Solver::SetCallbackLBA(const IbaCallback& iba_callback) {
  m_internal->m_LBA.SetCallback(iba_callback);
}

void Solver::SetCallbackGBA(const IbaCallback& iba_callback) {
  m_internal->m_GBA.SetCallback(iba_callback);
}

void Solver::GetSlidingWindow(SlidingWindow *SW) {
  const ubyte Uc = m_internal->m_LM.IBA_Synchronize(m_internal->m_nFrms,
                                                    m_internal->m_CsLF, m_internal->m_CsKF,
                                                    m_internal->m_ucsKF, m_internal->m_ds,
                                                    m_internal->m_uds);
  SW->iFrms.resize(0);
  SW->CsLF.resize(0);
  if (Uc & LM_FLAG_FRAME_UPDATE_CAMERA_LF) {
    CameraIMUState C;
    for (std::list<LocalMap::CameraLF>::const_iterator i = m_internal->m_CsLF.begin();
         i != m_internal->m_CsLF.end(); ++i) {
      if (!i->m_uc) {
        continue;
      }
      SW->iFrms.push_back(i->m_iFrm);
      ConvertCamera(i->m_C, &C);
      SW->CsLF.push_back(C);
    }
  }
  SW->iFrmsKF.resize(0);
  SW->CsKF.resize(0);
  if (Uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF) {
    CameraPose C;
    const int nKFs = m_internal->m_CsKF.Size();
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (!(m_internal->m_ucsKF[iKF] & LM_FLAG_FRAME_UPDATE_CAMERA_KF)) {
        continue;
      }
      const Rigid3D &_C = m_internal->m_CsKF[iKF];
      _C.LA::AlignedMatrix3x3f::Get(C.R);
      _C.GetPosition().Get(C.p);
      SW->iFrmsKF.push_back(m_internal->m_iFrmsKF[iKF]);
      SW->CsKF.push_back(C);
    }
  }
  SW->Xs.resize(0);
  if (Uc & LM_FLAG_FRAME_UPDATE_DEPTH) {
    ::Point3D X1;
    Point3D X2;
    const int nKFs = static_cast<int>(m_internal->m_ucsKF.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (!(m_internal->m_ucsKF[iKF] & LM_FLAG_FRAME_UPDATE_DEPTH)) {
        continue;
      }
      const int id1 = m_internal->m_iKF2d[iKF], id2 = m_internal->m_iKF2d[iKF + 1];
      const ::Point2D *xs = m_internal->m_xs.data() + id1;
      const ::Depth::InverseGaussian *ds = m_internal->m_ds.data() + id1;
      const ubyte *uds = m_internal->m_uds.data() + id1;
      const int *idxs = m_internal->m_id2idx.data() + id1;
      const int Nx = id2 - id1;
      const Rigid3D &C = m_internal->m_CsKF[iKF];
      for (int ix = 0; ix < Nx; ++ix) {
        if (!(uds[ix] & LM_FLAG_TRACK_UPDATE_DEPTH)) {
          continue;
        }
        C.ApplyInversely(xs[ix], 1.0f / ds[ix].u(), X1);
        X2.idx = idxs[ix];
        X1.Get(X2.X);
        SW->Xs.push_back(X2);
      }
    }
  }
}

const std::vector<int>& Solver::GetKeyFrameIndexes() {
  return m_internal->m_iFrmsKF;
}

int Solver::SearchKeyFrame(const int iFrm) {
  const std::vector<int> &iFrmsKF = m_internal->m_iFrmsKF;
  const std::vector<int>::const_iterator it = std::lower_bound(iFrmsKF.begin(),
                                                               iFrmsKF.end(), iFrm);
  if (it == iFrmsKF.end() || *it != iFrm) {
    return -1;
  }
  return static_cast<int>(it - iFrmsKF.begin());
}

bool Solver::DeleteKeyFrame(const int iFrm) {
  const int iKF = SearchKeyFrame(iFrm);
  if (iKF == -1) {
    return false;
  }
#ifdef CFG_DEBUG
//#if 0
  UT::Print("\nDeleting KF%d [%d]...", iKF, iFrm);
#endif
  std::vector<int> &iFrmsKF = m_internal->m_iFrmsKF;
  const int nKFs = static_cast<int>(iFrmsKF.size());
  iFrmsKF.erase(iFrmsKF.begin() + iKF);

  std::vector<int> &iKF2d = m_internal->m_iKF2d, &id2X = m_internal->m_id2X;
  std::vector<int> &iX2d = m_internal->m_iX2d, &id2idx = m_internal->m_id2idx;
  const int id1 = iKF2d[iKF], id2 = iKF2d[iKF + 1], Nd = id2 - id1;
  for (int jKF = iKF + 1; jKF <= nKFs; ++jKF) {
    iKF2d[jKF] -= Nd;
  }
  iKF2d.erase(iKF2d.begin() + iKF);
  for (int id = id1; id < id2; ++id) {
    iX2d[id2X[id]] = -1;
  }
  const int ND = static_cast<int>(id2X.size());
  for (int id = id2; id < ND; ++id) {
    iX2d[id2X[id]] -= Nd;
  }
  id2X.erase(id2X.begin() + id1, id2X.begin() + id2);
  id2idx.erase(id2idx.begin() + id1, id2idx.begin() + id2);
  m_internal->m_CsKF.Erase(iKF);
  std::vector<::Point2D> &xs = m_internal->m_xs;
  xs.erase(xs.begin() + id1, xs.begin() + id2);
  std::vector<::Depth::InverseGaussian> &ds = m_internal->m_ds;
  ds.erase(ds.begin() + id1, ds.begin() + id2);
#ifdef CFG_GROUND_TRUTH
  std::vector<::Depth::InverseGaussian> &dsGT = m_internal->m_dsGT;
  if (!dsGT.empty()) {
    dsGT.erase(dsGT.begin() + id1, dsGT.begin() + id2);
  }
#endif
  //m_internal->m_LBA.Synchronize();
  //m_internal->m_GBA.Synchronize();
  m_internal->m_LBA.DeleteKeyFrame(iKF);
  m_internal->m_LM.IBA_DeleteKeyFrame(iKF);
#ifdef CFG_DEBUG
//#if 0
  UT::Print("done!\n");
#endif
  return true;
}

void Solver::UpdateCameras(const std::vector<int> &iFrms, const std::vector<CameraPose> &Cs) {
  Rigid3D C;
  std::vector<GlobalBundleAdjustor::InputCamera> &ICs = m_internal->m_ICs;
  ICs.resize(0);
  const std::vector<int> &iFrmsKF = m_internal->m_iFrmsKF;
  std::vector<int>::const_iterator i = iFrmsKF.begin();
  const int N = static_cast<int>(iFrms.size());
#ifdef CFG_DEBUG
  UT_ASSERT(static_cast<int>(Cs.size()) == N);
#endif
  for (int j = 0; j < N; ++j) {
    const int iFrm = iFrms[j];
#ifdef CFG_DEBUG
    if (j > 0) {
      UT_ASSERT(iFrm > iFrms[j - 1]);
    }
#endif
    i = std::lower_bound(i, iFrmsKF.end(), iFrm);
    if (i == iFrmsKF.end()) {
      break;
    } else if (*i != iFrm) {
      continue;
    }
    const int iKF = static_cast<int>(i - iFrmsKF.begin());
    ConvertCamera(Cs[j], &C);
    ICs.push_back(GlobalBundleAdjustor::InputCamera(iKF, C));
  }
#ifdef CFG_DEBUG
  const int _N = static_cast<int>(ICs.size());
  UT::Print("\nUpdating %d / %d cameras {", _N, iFrmsKF.size());
  for (int j = 0; j < _N; ++j) {
    UT::Print(" %d", ICs[j].m_iKF);
  }
  UT::Print(" }\n");
#endif
  m_internal->m_GBA.UpdateCameras(m_internal->m_nFrms, ICs);
  m_internal->m_GBA.WakeUp();
}

void Solver::PropagateState(const std::vector<IMUMeasurement> &us, const float t1, const float t2,
                            const CameraIMUState &X1, CameraIMUState *X2) {
  AlignedVector<IMU::Measurement> _us;
  const int N = static_cast<int>(us.size());
  _us.Resize(N);
  const Rotation3D Ru = m_internal->m_K.m_Ru;
  const ::Point3D pu = m_internal->m_K.m_pu;
  const LA::AlignedVector3f ba = m_internal->m_K.m_ba;
  const LA::AlignedVector3f bw = m_internal->m_K.m_bw;
  for (int i = 0; i < N; ++i) {
    const IMUMeasurement &u = us[i];
    IMU::Measurement &_u = _us[i];
    _u.m_a.Set(u.a);
    _u.m_w.Set(u.w);
    if (Ru.Valid()) {
      _u.m_a = Ru.GetApplied(_u.m_a);
      _u.m_w = Ru.GetApplied(_u.m_w);
    }
    _u.m_a -= ba;
    _u.m_w -= bw;
    _u.t() = u.t;
  }
  Camera C1, C2;
  IMU::Delta D;
  AlignedVector<float> work;
  ConvertCamera(X1, &C1);
  IMU::PreIntegrate(_us, t1, t2, C1, &D, &work, false);
  IMU::Propagate(pu, D, C1, C2);
  ConvertCamera(C2, X2);
}

bool Solver::SaveFeatures(const std::string fileName, const std::vector<MapPointMeasurement> &zs,
                          const std::vector<MapPoint> *Xs) {
  FILE *fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  fprintf(fp, "key_points: [ ");
  const int Nz = static_cast<int>(zs.size());
  for (int iz = 0; iz < Nz; ++iz) {
    const MapPointMeasurement &z = zs[iz];
    //const int id = m_internal->m_iX2d[z.iX];
    const int iX = m_internal->m_idx2iX[z.idx];
    const int id = m_internal->m_iX2d[iX];
    if (id != -1) {
      fprintf(fp, "%f, %f, 0, 0, 0, 0, %d,\n", z.x.x[0], z.x.x[1], m_internal->m_id2idx[id]);
    }
  }
  fprintf(fp, "]\n");
  if (Xs) {
    fprintf(fp, "map_points: [\n");
    const int Nx = static_cast<int>(Xs->size());
    fprintf(fp, "%d\n", Nx);
    for (int ix = 0; ix < Nx; ++ix) {
      const MapPoint &X = Xs->at(ix);
      const int Nz = static_cast<int>(X.zs.size());
      fprintf(fp, "%d %d\n", X.X.idx, Nz);
      for (int i = 0; i < Nz; ++i) {
        const MapPointMeasurement &z = X.zs[i];
        fprintf(fp, "%d %f %f\n", z.iFrm, z.x.x[0], z.x.x[1]);
      }
    }
    fprintf(fp, "]\n");
  }
  fclose(fp);
  return true;
}

bool Solver::LoadFeatures(const std::string fileName, const int iFrm,
                          std::vector<MapPointMeasurement> *zs,
                          std::vector<MapPoint> *Xs,
                          const std::vector<int> *iFrms
#ifdef CFG_STEREO
                        , const ubyte right
#endif
                        ) {
#ifdef CFG_STEREO
  if (!right)
#endif
  {
    zs->resize(0);
    if (Xs) {
      Xs->resize(0);
    }
  }
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    return false;
  }
  char line[UT_STRING_WIDTH_MAX];
  MapPointMeasurement z;
#ifdef CFG_STEREO
  z.right = right;
  std::vector<int> &idx2iXTmp = m_internal->m_idxsTmp;
  if (!right) {
    idx2iXTmp.resize(0);
  }
#endif
  int i = 0/*, idx*/;
  char *num;
  float v;
  const bool cov = FTR_VARIANCE == 0.0f;
  if (!cov) {
    z.x.S[0][0] = z.x.S[1][1] = FTR_VARIANCE;
    z.x.S[0][1] = z.x.S[1][0] = 0.0f;
  }
  const std::vector<int> &idx2iX = m_internal->m_idx2iX;
  const int idxMax = static_cast<int>(idx2iX.size()) - 1;
  while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
    const int len = int(std::remove(line, std::remove(line, line + strlen(line), 10), ' ') - line);
    if (len == 0 || line[0] == '%' || line[len - 1] == ':') {
      continue;
    }
    line[len] = 0;
    num = strtok(line, " ,");
    while (num && (sscanf(num, "%f", &v) == 1 || sscanf(num, "key_points:[%f", &v) == 1)) {
      switch (i) {
      case 0:
        z.x.x[0] = v;
        break;
      case 1:
        z.x.x[1] = v;
        break;
      case 2:
        if (cov) {
          z.x.S[0][0] = v;
        }
        break;
      case 3:
        if (cov) {
          z.x.S[0][1] = z.x.S[0][1] = v;
        }
        break;
      case 4:
        if (cov) {
          z.x.S[1][1] = v;
        }
        break;
      case 5:
        break;
      case 6:
        //idx = static_cast<int>(v);
        //z.iX = idx > idxMax ? -1 : idx2iX[idx];
        z.idx = static_cast<int>(v);
        break;
      }
      ++i;
      if (i == 7) {
        i = 0;
        if (z.idx > idxMax || idx2iX[z.idx] == -1) {
        //if (z.iX == -1) {
          if (Xs) {
#ifdef CFG_STEREO
            if (right) {
              const int iX = z.idx < static_cast<int>(idx2iXTmp.size()) ? idx2iXTmp[z.idx] : -1;
              if (iX != -1) {
                MapPoint &X = Xs->at(iX);
#ifdef CFG_DEBUG
                UT_ASSERT(X.X.idx == z.idx);
#endif
                z.iFrm = iFrm;
                X.zs.insert(std::lower_bound(X.zs.begin(), X.zs.end(), z), z);
              }
            } else
#endif
            {
              const int iX = static_cast<int>(Xs->size());
              Xs->resize(iX + 1);
              MapPoint &X = Xs->back();
              X.X.idx = z.idx;
              X.X.X[0] = FLT_MAX;
#ifdef CFG_STEREO
              if (z.idx >= static_cast<int>(idx2iXTmp.size())) {
                idx2iXTmp.resize(z.idx + 1, -1);
              }
              idx2iXTmp[z.idx] = iX;
#endif
              z.iFrm = iFrm;
              X.zs.assign(1, z);
            }
          }
        } else {
          zs->push_back(z);
        }
      }
      num = strtok(NULL, " ,");
    }
    if (line[len - 1] == ']') {
      break;
    }
  }
  if (Xs) {
    int Nx, Nz, iX, idx;
    float tmp[2];
    while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
      const int len = int(std::remove(line, std::remove(line, line + strlen(line), 10), ' ') - line);
      if (len == 0 || line[0] == '%' || line[len - 1] == ':') {
        continue;
      }
      line[len] = 0;
      if (strcmp(line, "map_points:[") != 0) {
        continue;
      }
      fscanf(fp, "%d", &Nx);
#ifdef CFG_STEREO
      if (!right)
#endif
      {
        Xs->reserve(Xs->size() + Nx);
      }
      for (int ix = 0; ix < Nx; ++ix) {
        fscanf(fp, "%d", &idx);
#ifdef CFG_STEREO
        if (right) {
          iX = idx < static_cast<int>(idx2iXTmp.size()) ? idx2iXTmp[idx] : -1;
          if (iX == -1) {
            fscanf(fp, "%d", &Nz);
            for (int i = 0; i < Nz; ++i) {
              fscanf(fp, "%d %f %f", &z.iFrm, &tmp[0], &tmp[1]);
            }
            continue;
          }
        } else
#endif
        {
          iX = static_cast<int>(Xs->size());
          Xs->resize(iX + 1);
          Xs->back().zs.resize(0);
#ifdef CFG_STEREO
          if (idx >= static_cast<int>(idx2iXTmp.size())) {
            idx2iXTmp.resize(idx + 1, -1);
          }
          idx2iXTmp[idx] = iX;
#endif
        }
        MapPoint &X = Xs->at(iX);
        X.X.idx = idx;
        fscanf(fp, "%d", &Nz);
        X.X.X[0] = FLT_MAX;
        for (int i = 0; i < Nz; ++i) {
          fscanf(fp, "%d %f %f", &z.iFrm, &z.x.x[0], &z.x.x[1]);
          z.x.S[0][0] = z.x.S[1][1] = FTR_VARIANCE;
          z.x.S[0][1] = z.x.S[1][0] = 0.0f;
          if (iFrms) {
            z.iFrm = iFrms->at(z.iFrm);
            if (z.iFrm == -1) {
              continue;
            }
          }
          X.zs.insert(std::lower_bound(X.zs.begin(), X.zs.end(), z), z);
        }
      }
      break;
    }
  }
  fclose(fp);
  return true;
}

void Solver::SaveB(FILE *fp) {
  m_internal->m_LM.SaveB(fp);
  m_internal->m_GM.SaveB(fp);
  m_internal->m_LBA.SaveB(fp);
  m_internal->m_GBA.SaveB(fp);
  UT::SaveB(m_internal->m_nFrms, fp);
  FRM::VectorSaveB(m_internal->m_Ts, fp);
  UT::VectorSaveB(m_internal->m_iFrmsKF, fp);
  UT::VectorSaveB(m_internal->m_iKF2d, fp);
  UT::VectorSaveB(m_internal->m_id2X, fp);
  UT::VectorSaveB(m_internal->m_iX2d, fp);
  UT::VectorSaveB(m_internal->m_id2idx, fp);
  UT::VectorSaveB(m_internal->m_idx2iX, fp);
  m_internal->m_CsKF.SaveB(fp);
  UT::VectorSaveB(m_internal->m_xs, fp);
  UT::VectorSaveB(m_internal->m_ds, fp);
#ifdef CFG_GROUND_TRUTH
  UT::VectorSaveB(m_internal->m_dsGT, fp);
  m_internal->m_tsGT.SaveB(fp);
  UT::VectorsSaveB(m_internal->m_zsGT, fp);
  UT::SaveB(m_internal->m_tsGT.Data(), fp);
#endif
#ifdef CFG_DEBUG_MT
  MT::SaveB(fp);
#endif
}

void Solver::LoadB(FILE *fp) {
  m_internal->m_LM.LoadB(fp);
  m_internal->m_GM.LoadB(fp);
  m_internal->m_LBA.LoadB(fp);
  m_internal->m_GBA.LoadB(fp);
  UT::LoadB(m_internal->m_nFrms, fp);
  FRM::VectorLoadB(m_internal->m_Ts, fp);
  UT::VectorLoadB(m_internal->m_iFrmsKF, fp);
  UT::VectorLoadB(m_internal->m_iKF2d, fp);
  UT::VectorLoadB(m_internal->m_id2X, fp);
  UT::VectorLoadB(m_internal->m_iX2d, fp);
  UT::VectorLoadB(m_internal->m_id2idx, fp);
  UT::VectorLoadB(m_internal->m_idx2iX, fp);
  m_internal->m_CsKF.LoadB(fp);
  UT::VectorLoadB(m_internal->m_xs, fp);
  UT::VectorLoadB(m_internal->m_ds, fp);
#ifdef CFG_GROUND_TRUTH
  UT::VectorLoadB(m_internal->m_dsGT, fp);
  m_internal->m_tsGT.LoadB(fp);
  UT::VectorsLoadB(m_internal->m_zsGT, fp);
  const LA::AlignedVector3f *t0 = UT::LoadB<LA::AlignedVector3f *>(fp);
  OffsetPointer(t0, m_internal->m_tsGT, &m_internal->m_zsGT);
#endif
#ifdef CFG_DEBUG_MT
  MT::LoadB(fp);
#endif
}

void Solver::ComputeErrorLBA(Error *e) {
  m_internal->m_LBA.ComputeErrorFeature(&e->ex);
  m_internal->m_LBA.ComputeErrorIMU(&e->eur, &e->eup, &e->euv, &e->euba, &e->eubw);
  m_internal->m_LBA.ComputeErrorDrift(&e->edr, &e->edp);
}

void Solver::ComputeErrorGBA(Error *e) {
  m_internal->m_GBA.ComputeErrorFeature(&e->ex);
  m_internal->m_GBA.ComputeErrorIMU(&e->eur, &e->eup, &e->euv, &e->euba, &e->eubw);
  m_internal->m_GBA.ComputeErrorDrift(&e->edr, &e->edp);
}

void Solver::PrintRMSE() {
  const float e = m_internal->m_GBA.ComputeRMSE();
  const float Sdist = m_internal->m_LBA.GetTotalDistance();
  UT::PrintSeparator();
  UT::Print("RMSE: %f / %f = %f%%\n", e, Sdist, UT::Percentage(e, Sdist));
}

static inline std::string ConvertFileName(const std::string fileName, const std::string dir, const bool append = true) {
  if (fileName == "") {
    return "";
  }
  std::string _fileName;
  if (UT::FileNameRemoveDirectoryExtension(fileName) == "*") {
    _fileName = dir.substr(0, dir.length() - 1) + "." + UT::FileNameExtractExtension(fileName);
    _fileName = UT::FileNameReplaceDirectory(_fileName, UT::FileNameExtractDirectory(_fileName),
                                             UT::FileNameExtractDirectory(fileName));
  } else {
    _fileName = fileName;
  }
  _fileName = UT::FileNameReplaceDirectory(_fileName, ".", dir);
  if (append) {
    _fileName = UT::FileNameAppendSuffix(_fileName);
  }
  const std::string _dir = UT::FileNameExtractDirectory(_fileName);
  if (!UT::FileExists(_dir) && _dir != "") {
#ifdef WIN32
    CreateDirectory(_dir.c_str(), 0);
#else
    boost::filesystem::create_directories(_dir);
#endif
  }
  return _fileName;
}

bool Solver::SaveCamerasLBA(const std::string fileName, const bool append, const bool poseOnly) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_LBA.SaveCameras(_fileName, poseOnly);
}

bool Solver::SaveCamerasGBA(const std::string fileName, const bool append, const bool poseOnly) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_GBA.SaveCameras(_fileName, poseOnly);
}

bool Solver::SaveCostsLBA(const std::string fileName, const bool append, const int type) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_LBA.SaveCosts(_fileName, type);
}

bool Solver::SaveCostsGBA(const std::string fileName, const bool append, const int type) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_GBA.SaveCosts(_fileName, type);
}

bool Solver::SaveResidualsLBA(const std::string fileName, const bool append, const int type) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_LBA.SaveResiduals(_fileName, type);
}

bool Solver::SaveResidualsGBA(const std::string fileName, const bool append, const int type) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_GBA.SaveResiduals(_fileName, type);
}

bool Solver::SavePriors(const std::string fileName, const bool append, const int type) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_LBA.SavePriors(_fileName, type);
}

bool Solver::SavePoints(const std::string fileName, const bool append) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  FILE *fp = fopen(_fileName.c_str(), "w");
  if (!fp) {
    return false;
  }
  ::Point3D X;
  const std::vector<::Depth::InverseGaussian> &ds = m_internal->m_GBA.m_ds;
  const int nKFs = static_cast<int>(GetKeyFrameIndexes().size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D &C = m_internal->m_GBA.m_Cs[iKF];
    const int id1 = m_internal->m_iKF2d[iKF], id2 = m_internal->m_iKF2d[iKF + 1];
    const ::Point2D *xs = m_internal->m_xs.data() + id1;
    const ::Depth::InverseGaussian *ds = m_internal->m_GBA.m_ds.data() + id1;
    const int *idxs = m_internal->m_id2idx.data() + id1;
    const int Nx = id2 - id1;
    for (int ix = 0; ix < Nx; ++ix) {
      C.ApplyInversely(xs[ix], 1.0f / ds[ix].u(), X);
      fprintf(fp, "%d %f %f %f\n", idxs[ix], X.x(), X.y(), X.z());
    }
  }
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

void Solver::GetTimeLBA(Time *t) {
  t->t = m_internal->m_LBA.GetTotalTime(&t->n);
  //t->nd = m_internal->m_LBA.GetTotalPoints();
}

void Solver::GetTimeGBA(Time *t) {
  t->t = m_internal->m_GBA.GetTotalTime(&t->n);
  //t->nd = m_internal->m_GBA.GetTotalPoints();
}

bool Solver::SaveTimesLBA(const std::string fileName, const bool append) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_LBA.SaveTimes(_fileName);
}

bool Solver::SaveTimesGBA(const std::string fileName, const bool append) {
  const std::string _fileName = ConvertFileName(fileName, m_internal->m_dir, append);
  return m_internal->m_GBA.SaveTimes(_fileName);
}

static inline void Rectify(const ::Intrinsic &K, const Point2D &x,
                           ::Point2D *xn, LA::SymmetricMatrix2x2f *Wn,
                           ::Intrinsic::UndistortionMap *UM = NULL,
                           const float eps = 0.0f
#ifdef CFG_STEREO
                         , Rotation3D *Rr = NULL
#endif
                         ) {
  ::Point2D xd;
  Point2DCovariance Sd, Sn;
  LA::AlignedMatrix2x2f J, JS;
  LA::AlignedMatrix2x2f Wd;
  K.k().ImageToNormalized(x.x, xd);
  K.Undistort(xd, xn, &J, UM);
//#ifdef CFG_DEBUG
#if 0
  UT::Print("fx = %f %f\n", K.k().m_fx, K.fx());
  UT::Print("cx = %f %f\n", K.k().m_cx, K.cx());
  UT::Print("x = %f %f\n", x.x[0], x.x[1]);
  UT::Print("xn = "); xn->Print();
  UT::Print("J = ");  J.Print();
#endif
  K.k().ImageToNormalized(x.S, Sd);
  Sd.GetInverse(Wd);
  LA::AlignedMatrix2x2f::ABT(J, Wd, JS);
  LA::SymmetricMatrix2x2f::ABT(JS, J, *Wn);
  Wn->GetInverse(Sn);
#ifdef CFG_STEREO
  if (Rr) {
    *xn = Rr->GetApplied(*xn, J);
    LA::SymmetricMatrix2x2f::AB(J, Sn, JS);
    LA::SymmetricMatrix2x2f::ABT(JS, J, Sn);
  }
#endif
  Sn.IncreaseDiagonal(eps);
  Sn.GetInverse(*Wn);
  //////////////////////////////////////////////////////////////////////////
  //Wn->Set(Wd.m00(), Wd.m01(), Wd.m11());
  //////////////////////////////////////////////////////////////////////////
}

const LocalBundleAdjustor::InputLocalFrame& Internal::PushCurrentFrame(const CurrentFrame &CF) {
  m_ILF.m_T.m_iFrm = CF.iFrm;
  m_ILF.m_T.m_t = CF.t;
  m_ILF.m_T.m_fileName = CF.fileName;
#ifdef CFG_STEREO
  m_ILF.m_T.m_fileNameRight = CF.fileNameRight;
#endif
#ifdef CFG_DEBUG
  UT_ASSERT(m_Ts.empty() || m_Ts.back() < m_ILF.m_T);
#endif
  m_Ts.push_back(m_ILF.m_T);
  ConvertCamera(CF.C, &m_ILF.m_C);
  ConvertDepth(CF.d, &m_ILF.m_d);
  ConvertFeatureMeasurements(CF.zs, &m_ILF);
  const int Nu = static_cast<int>(CF.us.size());
  m_ILF.m_us.Resize(Nu);
  for (int i = 0; i < Nu; ++i) {
    const IMUMeasurement &u = CF.us[i];
    IMU::Measurement &_u = m_ILF.m_us[i];
    _u.m_a.Set(u.a);
    _u.m_w.Set(u.w);
    if (m_K.m_Ru.Valid()) {
      _u.m_a = m_K.m_Ru.GetApplied(_u.m_a);
      _u.m_w = m_K.m_Ru.GetApplied(_u.m_w);
    }
    _u.m_a -= m_K.m_ba;
    _u.m_w -= m_K.m_bw;
    _u.t() = u.t;
  }
  ++m_nFrms;
  return m_ILF;
}

const GlobalMap::InputKeyFrame& Internal::PushKeyFrame(const KeyFrame &KF, const Camera *C) {
  const std::vector<FRM::Tag>::iterator T = std::lower_bound(m_Ts.begin(), m_Ts.end(), KF.iFrm);
#ifdef CFG_DEBUG
  UT_ASSERT(T != m_Ts.end() && T->m_iFrm == KF.iFrm);
#endif
  m_IKF.m_T = *T;
  m_Ts.erase(m_Ts.begin(), T);
  ConvertCamera(KF.C, &m_IKF.m_C.m_T);
  m_IKF.m_C.m_p.Set(KF.C.p);
  if (C) {
    m_IKF.m_C.m_v = C->m_v;
    m_IKF.m_C.m_ba = C->m_ba;
    m_IKF.m_C.m_bw = C->m_bw;
  } else {
    m_IKF.m_C.m_v.Invalidate();
    m_IKF.m_C.m_ba.Invalidate();
    m_IKF.m_C.m_bw.Invalidate();
  }
  ConvertDepth(KF.d, &m_IKF.m_d);
  ConvertFeatureMeasurements(KF.zs, &m_IKF);
  m_CsKF.Push(m_IKF.m_C.m_T);

  const int Nx1 = static_cast<int>(KF.Xs.size());
#ifdef CFG_DEBUG
  for (int ix = 0; ix < Nx1; ++ix) {
    const MapPoint &X = KF.Xs[ix];
    const int Nz = static_cast<int>(X.zs.size());
    for (int i = 1; i < Nz; ++i) {
      const MapPointMeasurement &z1 = X.zs[i - 1], &z2 = X.zs[i];
#ifdef CFG_STEREO
      UT_ASSERT(z1.iFrm < z2.iFrm || z1.iFrm == z2.iFrm && !z1.right && z2.right);
#else
      UT_ASSERT(z1.iFrm < z2.iFrm);
#endif
    }
  }
#endif
  m_iFrmsKF.push_back(KF.iFrm);
  int idxMax = static_cast<int>(m_idx2iX.size()) - 1;
  std::vector<MapPointIndex> &ixsSort = m_idxsSortTmp;
  ixsSort.resize(Nx1);
  for (int ix = 0; ix < Nx1; ++ix) {
    const MapPoint &X = KF.Xs[ix];
    idxMax = std::max(idxMax, X.X.idx);
    ixsSort[ix].Set(X.zs.front().iFrm, X.X.idx, ix);
  }
  std::sort(ixsSort.begin(), ixsSort.end());
  int iKF0, iKF1, iKF2;
  if (ixsSort.empty()) {
    iKF0 = iKF2 = -1;
  } else {
    iKF0 = static_cast<int>(std::upper_bound(m_iFrmsKF.begin(), m_iFrmsKF.end(),
                   ixsSort.front().m_iFrm) - m_iFrmsKF.begin()) - 1;
    iKF2 = static_cast<int>(std::upper_bound(m_iFrmsKF.begin() + iKF0, m_iFrmsKF.end(),
                    ixsSort.back().m_iFrm) - m_iFrmsKF.begin());
  }
  int i1, i2;
  FTR::Measurement z;
  ::Point3D X;
  m_IKF.m_Xs.resize(Nx1);
  std::vector<int> &ixs = m_idxsTmp;
  ixs.resize(Nx1);
  const float eps = FTR_VARIANCE_EPSILON * m_K.m_K.fxyI();
  const int nKFs = static_cast<int>(m_iFrmsKF.size());
  for (i1 = i2 = 0, iKF1 = iKF0; i1 < Nx1; ++i1) {
    const MapPointIndex &ix = ixsSort[i1];
//#ifdef CFG_DEBUG
#if 0
    if (ix.m_idx == 2001) {
      UT::DebugStart();
      UT::DebugStop();
    }
#endif
    if (ix.m_iFrm > m_iFrmsKF[iKF1]) {
      iKF1 = static_cast<int>(std::lower_bound(m_iFrmsKF.begin() + iKF1,
                                               m_iFrmsKF.begin() + iKF2, ix.m_iFrm) -
                                               m_iFrmsKF.begin());
      if (iKF1 == nKFs || m_iFrmsKF[iKF1] != ix.m_iFrm) {
        --iKF1;
        continue;
      }
    }
    z.m_iKF = iKF1;
    const MapPoint &X1 = KF.Xs[ix.m_ix];
    GlobalMap::Point &X2 = m_IKF.m_Xs[i2];
    X2.m_zs.resize(0);
    const int Nz1 = static_cast<int>(X1.zs.size());
    for (int i = 0; i < Nz1; ++i) {
      const MapPointMeasurement &_z = X1.zs[i];
#ifdef CFG_STEREO
      if (_z.right) {
        if (i == 0) {
          continue;
        }
        Rectify(m_K.m_Kr, _z.x, &z.m_zr, &z.m_Wr, &m_UMr, eps, &m_K.m_Rr);
        if (X1.zs[i - 1].iFrm != _z.iFrm) {
          z.m_z.Invalidate();
        }
        z.m_iKF = static_cast<int>(std::lower_bound(m_iFrmsKF.begin() + z.m_iKF,
                                                    m_iFrmsKF.begin() + iKF2, _z.iFrm) -
                                                    m_iFrmsKF.begin());
        if (z.m_iKF != nKFs && m_iFrmsKF[z.m_iKF] == _z.iFrm) {
          X2.m_zs.push_back(z);
        } else {
          --z.m_iKF;
        }
      } else
#endif
      {
        Rectify(m_K.m_K, _z.x, &z.m_z, &z.m_W, &m_UM, eps);
#ifdef CFG_STEREO
        const int j = i + 1;
        if (j == Nz1 || X1.zs[j].iFrm != _z.iFrm)
#endif
        {
#ifdef CFG_STEREO
          z.m_zr.Invalidate();
#endif
          z.m_iKF = static_cast<int>(std::lower_bound(m_iFrmsKF.begin() + z.m_iKF,
                                                      m_iFrmsKF.begin() + iKF2, _z.iFrm) -
                                                      m_iFrmsKF.begin());
          if (z.m_iKF != nKFs && m_iFrmsKF[z.m_iKF] == _z.iFrm) {
            X2.m_zs.push_back(z);
          } else {
            --z.m_iKF;
          }
        }
      }
    }
    const int Nz2 = static_cast<int>(X2.m_zs.size());
    if (Nz2 == 0) {
      continue;
    }
    const FTR::Measurement &_z = X2.m_zs.front();
    X2.m_iKF = _z.m_iKF;
    X2.m_x.m_x = _z.m_z;
    X2.m_W = _z.m_W;
#ifdef CFG_STEREO
    X2.m_x.m_xr = _z.m_zr;
    X2.m_x.m_Wr = _z.m_Wr;
#endif
    X2.m_zs.erase(X2.m_zs.begin());
    X.Set(X1.X.X);
    if (X.Valid()) {
      X2.m_d.Initialize(1.0f / m_CsKF[X2.m_iKF].GetAppliedZ(X));
    } else {
      X2.m_d.Invalidate();
    }
    ixs[i2] = ix.m_ix;
    ++i2;
  }
  const int Nx2 = i2;
  m_IKF.m_Xs.resize(Nx2);
  ixs.resize(Nx2);

  m_iKF2d.push_back(m_iKF2d.back());
  const int NX1 = static_cast<int>(m_iX2d.size()), NX2 = NX1 + Nx1;
  m_iX2d.resize(NX2, -1);
  m_idx2iX.resize(idxMax + 1, -1);
  for (int i1 = 0, i2 = 0; i1 < Nx2; i1 = i2) {
    const int iKF = m_IKF.m_Xs[i1].m_iKF;
    for (i2 = i1 + 1; i2 < Nx2 && m_IKF.m_Xs[i2].m_iKF == iKF; ++i2) {}
    const int id = m_iKF2d[iKF + 1], Nx = i2 - i1;
    for (int jKF = iKF; jKF < nKFs; ++jKF) {
      m_iKF2d[jKF + 1] += Nx;
    }
    m_id2X.insert(m_id2X.begin() + id, Nx, -1);
    const int Nd = static_cast<int>(m_id2X.size());
    for (int jd = id + Nx; jd < Nd; ++jd) {
      const int iX = m_id2X[jd];
      m_iX2d[iX] += Nx;
    }
    m_id2idx.insert(m_id2idx.begin() + id, Nx, -1);
    m_xs.insert(m_xs.begin() + id, Nx, ::Point2D());
    m_ds.insert(m_ds.begin() + id, Nx, ::Depth::InverseGaussian());
    const int *_ixs = ixs.data() + i1;
    const GlobalMap::Point *Xs = m_IKF.m_Xs.data() + i1;
    ::Point2D *xs = m_xs.data() + id;
    ::Depth::InverseGaussian *ds = m_ds.data() + id;
#ifdef CFG_GROUND_TRUTH
    ::Depth::InverseGaussian *dsGT = NULL;
    if (static_cast<int>(m_DsGT.size()) > idxMax) {
      m_dsGT.insert(m_dsGT.begin() + id, Nx, ::Depth::InverseGaussian());
      dsGT = m_dsGT.data() + id;
    }
#endif
    for (int i = 0, jd = id; i < Nx; ++i, ++jd) {
      const int ix = _ixs[i], iX = NX1 + ix, idx = KF.Xs[ix].X.idx;
      m_id2X[jd] = iX;
      m_iX2d[iX] = jd;
      m_id2idx[jd] = idx;
      const int _iX = m_idx2iX[idx];
      if (_iX != -1) {
        const int _id = m_iX2d[_iX];
        if (_id != -1) {
#ifdef CFG_DEBUG
          UT_ASSERT(m_id2X[_id] == _iX);
#endif
          //m_id2X[_id] = -1;
          //m_iX2d[_iX] = -1;
          m_id2idx[_id] = -1;
        }
      }
      m_idx2iX[idx] = iX;
      const GlobalMap::Point &X = Xs[i];
      xs[i] = X.m_x.m_x;
      ds[i] = X.m_d;
#ifdef CFG_GROUND_TRUTH
      if (dsGT) {
        dsGT[i] = m_DsGT[idx];
      }
#endif
    }
  }
  return m_IKF;
}

void Internal::ConvertFeatureMeasurements(const std::vector<MapPointMeasurement> &zs,
                                          FRM::Frame *F) {
  F->ClearMeasurements();

  int i, j;
  const float eps = FTR_VARIANCE_EPSILON * m_K.m_K.fxyI();
  const int Nz1 = static_cast<int>(zs.size());
  m_zsSortTmp.resize(Nz1);
  for (i = j = 0; i < Nz1; ++i) {
    const MapPointMeasurement &z1 = zs[i];
    FeatureMeasurement &z2 = m_zsSortTmp[j];
    const int iX = m_idx2iX[z1.idx];
    //if (iX == -1) {
    //  continue;
    //}
#ifdef CFG_DEBUG
    UT_ASSERT(iX != -1);
#endif
    if ((z2.m_id = m_iX2d[iX]) == -1) {
      continue;
    }
#ifdef CFG_STEREO
    z2.m_right = z1.right;
    if (z1.right) {
      Rectify(m_K.m_Kr, z1.x, &z2.m_z, &z2.m_W, &m_UMr, eps, &m_K.m_Rr);
    } else
#endif
    {
      Rectify(m_K.m_K, z1.x, &z2.m_z, &z2.m_W, &m_UM, eps);
    }
    ++j;
  }
  const int Nz2 = j;
  m_zsSortTmp.resize(Nz2);
  std::sort(m_zsSortTmp.begin(), m_zsSortTmp.end());
#ifdef CFG_DEBUG
  UT_ASSERT(m_zsSortTmp.empty() || m_zsSortTmp.front().m_id != -1);
#endif
  FTR::Measurement z;
  for (int i1 = 0, i2 = 0, iKF = 0; i1 < Nz2; i1 = i2) {
    const int id1 = m_zsSortTmp[i1].m_id;
    iKF = static_cast<int>(std::upper_bound(m_iKF2d.begin() + iKF, m_iKF2d.end(), id1) -
                                            m_iKF2d.begin()) - 1;
    const int id0 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1];
#ifdef CFG_DEBUG
    UT_ASSERT(id1 >= id0 && id1 < id2);
#endif
    i2 = static_cast<int>(std::lower_bound(m_zsSortTmp.begin() + i1, m_zsSortTmp.end(), id2) -
                                           m_zsSortTmp.begin());
#ifdef CFG_DEBUG
    UT_ASSERT(i2 == Nz2 || m_zsSortTmp[i2].m_id >= id2);
    for (i = i1; i < i2; ++i) {
      const int id = m_zsSortTmp[i].m_id;
      UT_ASSERT(id >= id0 && id < id2);
    }
#endif
    m_zsTmp.resize(0);
    for (i = i1; i < i2; ++i) {
//#ifdef CFG_DEBUG
#if 0
      if (m_zsTmp.size() == 58) {
        UT::DebugStart();
        UT::DebugStop();
      }
#endif
      const FeatureMeasurement &_z = m_zsSortTmp[i];
      z.m_ix = _z.m_id - id0;
#ifdef CFG_STEREO
      if (_z.m_right) {
        z.m_zr = _z.m_z;
        z.m_Wr = _z.m_W;
        if (i == i1 || m_zsSortTmp[i - 1].m_id != _z.m_id) {
          z.m_z.Invalidate();
        }
        m_zsTmp.push_back(z);
      } else
#endif
      {
        z.m_z = _z.m_z;
        z.m_W = _z.m_W;
#ifdef CFG_STEREO
        j = i + 1;
        if (j == i2 || m_zsSortTmp[j].m_id != _z.m_id)
#endif
        {
          z.m_ix = _z.m_id - id0;
#ifdef CFG_STEREO
          z.m_zr.Invalidate();
#endif
          m_zsTmp.push_back(z);
        }
      }
    }
    F->PushFrameMeasurement(iKF, m_zsTmp);
  }
}

void Internal::AssertConsistency() {
  const int N = static_cast<int>(m_Ts.size()), nKFs = static_cast<int>(m_iFrmsKF.size());
  if (N > 0) {
    for (int i = 1; i < N; ++i) {
      UT_ASSERT(m_Ts[i - 1] < m_Ts[i]);
    }
    if (nKFs > 0) {
      UT_ASSERT(m_iFrmsKF.back() <= m_Ts.front().m_iFrm);
    }
  }
  UT_ASSERT(m_CsKF.Size() == nKFs);
  UT_ASSERT(static_cast<int>(m_iKF2d.size()) == nKFs + 1);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    UT_ASSERT(m_iKF2d[iKF] <= m_iKF2d[iKF + 1]);
  }
  const int Nd = m_iKF2d.back();
  int SNd = 0;
  const int NX = static_cast<int>(m_iX2d.size());
  for (int iX = 0; iX < NX; ++iX) {
    const int id = m_iX2d[iX];
    if (id == -1) {
      continue;
    }
    UT_ASSERT(m_id2X[id] == iX);
    ++SNd;
  }
  UT_ASSERT(SNd == Nd);
  UT_ASSERT(static_cast<int>(m_id2X.size()) == Nd && static_cast<int>(m_id2idx.size()) == Nd);
  for (int id = 0; id < Nd; ++id) {
    const int iX = m_id2X[id], idx = m_id2idx[id];
    UT_ASSERT(m_iX2d[iX] == id);
    if (idx != -1) {
      UT_ASSERT(m_idx2iX[idx] == iX);
    }
  }
  const int idxMax = static_cast<int>(m_idx2iX.size() - 1);
  for (int idx = 0; idx <= idxMax; ++idx) {
    const int iX = m_idx2iX[idx];
    if (iX == -1) {
      continue;
    }
    const int id = m_iX2d[iX];
    if (id != -1) {
      UT_ASSERT(m_id2idx[id] == idx);
    }
  }
  UT_ASSERT(static_cast<int>(m_xs.size()) == Nd);
  UT_ASSERT(static_cast<int>(m_ds.size()) == Nd);
}

void LoadParameters(const std::string fileName) {
  if (!UT::FileExists(fileName)) {
    return;
  }
  const Configurator cfgor(fileName.c_str());
  //cfgor.Print();
  ::LoadParameters(cfgor);
}

bool SaveCalibration(const std::string fileName, const Calibration &K) {
  FILE *fp = fopen(fileName.c_str(), "wb");
  if (!fp) {
    return false;
  }
  UT::SaveB(K, fp);
  fclose(fp);
  UT::PrintSaved(fileName);
  return true;
}

bool LoadCalibration(const std::string fileName, Calibration *K) {
  FILE *fp = fopen(fileName.c_str(), "rb");
  if (!fp) {
    return false;
  }
  UT::LoadB(*K, fp);
  fclose(fp);
  UT::PrintLoaded(fileName);
//#ifdef CFG_DEBUG
#if 0
  K->ba[0] = 1.0f;
  K->ba[1] = -1.0f;
  K->ba[2] = 0.5f;
  UT::Check("Noise\n");
#endif
  return true;
}

bool LoadCalibration(const std::string fileName, float T[3][4], Intrinsic *K,
                     float *ba, float *bw/*, float *sa*/) {
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    return false;
  }
  if (ba) {
    ba[0] = ba[1] = ba[2] = 0.0f;
  }
  if (bw) {
    bw[0] = bw[1] = bw[2] = 0.0f;
  }
  //if (sa) {
  //  sa[0] = sa[1] = sa[2] = 1.0f;
  //}
/*  const std::string ext = UT::FileNameExtractExtension(fileName);
  if (ext == "txt") {
    memset(T, 0, sizeof(float) * 12);
    T[0][0] = T[1][1] = T[2][2] = 1.0f;
    if (fscanf(fp, "%f %f %f %f", &K->fx, &K->fy, &K->cx, &K->cy) != 4)
      return false;
    memset(K->ds, 0, sizeof(K->ds));
    for (int i = 0; i < 8 && UT::Load<float>(K->ds[i], fp); ++i) {}
  }
  else if (ext == "yaml") */{
    int i = 0, j = 0;
    float k[3];
    char line[UT_STRING_WIDTH_MAX];
    memset(K->ds, 0, sizeof(K->ds));
    while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
      const int len = int(std::remove(line, std::remove(line, line + strlen(line), 10), ' ') - line);
      if (len == 0 || line[len - 1] == ':')
        continue;
      line[len] = 0;
      switch (i) {
      case 0: {
        if (j == 3 ||
            sscanf(line, "- [%f, %f, %f, %f]", &T[j][0], &T[j][1], &T[j][2], &T[j][3]) == 4) {
          ++j;
        }
        break; }
      case 1: {
        if (sscanf(line, "- [%f, %f, %f]", &k[0], &k[1], &k[2]) == 3) {
          if (j == 0) {
            K->fx = k[0];
            K->cx = k[2];
          } else if (j == 1) {
            K->fy = k[1];
            K->cy = k[2];
          }
          ++j;
        }
        break; }
      case 2: {
        if (sscanf(line, "- [%f]", &K->ds[j]) == 1)
          ++j;
        break; }
      case 3: {
        if (j == 0 && ba) {
          sscanf(line, "accel_bias: [%f, %f, %f]", &ba[0], &ba[1], &ba[2]);
        }
        if (j == 1 && bw) {
          sscanf(line, "gyro_bias: [%f, %f, %f]", &bw[0], &bw[1], &bw[2]);
        }
        //if (j == 2 && sa) {
        //  sscanf(line, "accel_scale: [%f, %f, %f]", &sa[0], &sa[1], &sa[2]);
        //}
        ++j;
        break; }
      }
      if ((i == 0 && j == 4) || (i == 1 && j == 3) || (i == 2 && j == 8) || (i == 3 && j == 2)) {
        j = 0;
        ++i;
      }
    }
    if (i < 3) {
      return false;
    }
    Rotation3D R;
    R.Set(T[0], T[1], T[2]);
    R.MakeOrthogonal();
    R.Get(T[0], T[1], T[2]);
  }
  UT::PrintLoaded(fileName);
  return true;
}

void PrintCalibration(const Calibration &K) {
  Rigid3D T;
  T.Set(&K.Tu[0][0]);
  T.Print("Tcu = ", false);
#ifdef CFG_STEREO
  T.Set(&K.Tr[0][0]);
  T.Print("Tlr = ", false);
#endif
  UT::Print("K   = %f %f %f %f\n", K.K.fx, K.K.fy, K.K.cx, K.K.cy);
#ifdef CFG_STEREO
  UT::Print("Kr  = %f %f %f %f\n", K.Kr.fx, K.Kr.fy, K.Kr.cx, K.Kr.cy);
#endif
}

bool SaveGroundTruth(const std::string fileName, const std::vector<CameraIMUState> &XsGT) {
  return UT::VectorSaveB(XsGT, fileName.c_str());
}

bool LoadGroundTruth(const std::string fileName, std::vector<CameraIMUState> *XsGT) {
  return UT::VectorLoadB(*XsGT, fileName.c_str());
}

bool SaveGroundTruth(const std::string fileName, const std::vector<Depth> &dsGT) {
  return UT::VectorSaveB(dsGT, fileName.c_str());
}

bool LoadGroundTruth(const std::string fileName, std::vector<Depth> *dsGT) {
  if (!UT::VectorLoadB(*dsGT, fileName.c_str())) {
    return false;
  }
  const int N = static_cast<int>(dsGT->size());
  for (int i = 0; i < N; ++i) {
    Depth &d = dsGT->at(i);
    if (d.d != 0.0f) {
      continue;
    }
    d.d = DEPTH_INITIAL;
    d.s2 = DEPTH_VARIANCE_INITIAL;
  }
  return true;
}

bool LoadKeyFrames(const std::string fileName, const std::vector<float> &ts,
                   std::vector<ubyte> *kfs, const float dtMax) {
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    kfs->resize(0);
    return false;
  }
  char line[UT_STRING_WIDTH_MAX];
  int kf, iFrm;
  float t, dt1, dt2;
  std::vector<ubyte> &_kfs = *kfs;
  _kfs.assign(ts.size(), 0);
  std::vector<float>::const_iterator it = ts.begin();
  while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
    if (line[0] == '#') {
      continue;
    }
    sscanf(line, "%f %d", &t, &kf);
    it = std::lower_bound(it, ts.end(), t);
    dt1 = it == ts.begin() ? FLT_MAX : (t - *(it - 1));
    dt2 = it == ts.end() ? FLT_MAX : (*it - t);
    if (dt1 > dtMax && dt2 > dtMax) {
      continue;
    } else if (dt1 < dt2) {
      iFrm = static_cast<int>(it - ts.begin()) - 1;
    } else {
      iFrm = static_cast<int>(it - ts.begin());
    }
    _kfs[iFrm] = static_cast<ubyte>(kf);
  }
  if (!_kfs.empty()) {
    _kfs.front() = 1;
  }
  fclose(fp);
  UT::PrintLoaded(fileName);
  return true;
}

bool LoadKeyFrames(const std::string fileName, std::vector<int> *iFrms) {
  iFrms->resize(0);
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    return false;
  }
  int iFrm;
  while (fscanf(fp, "%d", &iFrm) == 1) {
    iFrms->push_back(iFrm);
  }
  fclose(fp);
  return true;
}

bool SaveCurrentFrame(const std::string fileName, const CurrentFrame &CF, const KeyFrame &KF) {
  FILE *fp = fopen(fileName.c_str(), "wb");
  if (!fp) {
    return false;
  }
  UT::SaveB(CF.iFrm, fp);
  UT::SaveB(CF.C, fp);
  UT::VectorSaveB(CF.zs, fp);
  UT::VectorSaveB(CF.us, fp);
  UT::SaveB(CF.t, fp);
  UT::SaveB(CF.d, fp);
  UT::StringSaveB(CF.fileName, fp);
#ifdef CFG_STEREO
  UT::StringSaveB(CF.fileNameRight, fp);
#endif
  UT::SaveB(KF.iFrm, fp);
  UT::SaveB(KF.C, fp);
  UT::VectorSaveB(KF.zs, fp);
  const int NX = static_cast<int>(KF.Xs.size());
  UT::SaveB(NX, fp);
  for (int iX = 0; iX < NX; ++iX) {
    const MapPoint &X = KF.Xs[iX];
    UT::VectorSaveB(X.zs, fp);
    UT::SaveB(X.X, fp);
  }
  UT::SaveB(KF.d, fp);
  fclose(fp);
  return true;
}

bool LoadCurrentFrame(const std::string fileName, CurrentFrame *CF, KeyFrame *KF) {
  FILE *fp = fopen(fileName.c_str(), "rb");
  if (!fp) {
    return false;
  }
  UT::LoadB(CF->iFrm, fp);
  UT::LoadB(CF->C, fp);
  UT::VectorLoadB(CF->zs, fp);
  UT::VectorLoadB(CF->us, fp);
  UT::LoadB(CF->t, fp);
  UT::LoadB(CF->d, fp);
  UT::StringLoadB(CF->fileName, fp);
#ifdef CFG_STEREO
  UT::StringLoadB(CF->fileNameRight, fp);
#endif
  UT::LoadB(KF->iFrm, fp);
  UT::LoadB(KF->C, fp);
  UT::VectorLoadB(KF->zs, fp);
  const int NX = UT::LoadB<int>(fp);
  KF->Xs.resize(NX);
  for (int iX = 0; iX < NX; ++iX) {
    MapPoint &X = KF->Xs[iX];
    UT::VectorLoadB(X.zs, fp);
    UT::LoadB(X.X, fp);
  }
  UT::LoadB(KF->d, fp);
  fclose(fp);
#ifdef CFG_DEBUG
//#if 1
  for (int iX = 0; iX < NX; ++iX) {
    const MapPoint &X = KF->Xs[iX];
    UT_ASSERT(X.zs.back().iFrm == KF->iFrm);
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  const int Nz = static_cast<int>(CF->zs.size());
  for (int iz = 0; iz < Nz; ++iz) {
    const int idx = CF->zs[iz].idx;
    if (idx == 845) {
      UT::DebugStart();
      UT::DebugStop();
      UT::Print("[%d] %s iz = %d idx = %d\n", CF->iFrm, fileName.c_str(), iz, idx);
      exit(0);
    }
  }
#endif
//#ifdef CFG_DEBUG
#if 0
  if (KF->iFrm != -1) {
    for (int iX = 0; iX < NX; ++iX) {
      const int idx = KF->Xs[iX].X.idx;
      if (idx == 845) {
        UT::DebugStart();
        UT::DebugStop();
        UT::Print("[%d] %s iX = %d idx = %d\n", CF->iFrm, fileName.c_str(), iX, idx);
        exit(0);
      }
    }
  }
#endif
  return true;
}

bool LoadCurrentFrameTime(const std::string fileName, double *t) {
  FILE *fp = fopen(fileName.c_str(), "rb");
  if (!fp) {
    return false;
  }
  fseek(fp, sizeof(int) + sizeof(CameraIMUState), SEEK_CUR);
  const int Nz = UT::LoadB<int>(fp);
  fseek(fp, sizeof(MapPointMeasurement) * Nz, SEEK_CUR);
  const int Nu = UT::LoadB<int>(fp);
  fseek(fp, sizeof(IMUMeasurement) * Nu, SEEK_CUR);
  *t = static_cast<double>(UT::LoadB<float>(fp));
  fclose(fp);
  return true;
}

bool LoadRelativeConstraints(const std::string fileName, const std::vector<float> &ts,
                             std::vector<RelativeConstraint> *Zs, const float dtMax) {
  Zs->resize(0);
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    return false;
  }
  char line[UT_STRING_WIDTH_MAX];
  float t1, t2;
  RelativeConstraint Z;
  //const float sr = 1.0f * UT_FACTOR_DEG_TO_RAD;
  //const float sr2 = sr * sr;
  std::string format = "%f";
  for (int i = 0; i < 40; ++i) {
    format = format + " %f";
  }
  memset(&Z.S[0][0], 0, sizeof(Z.S));
  while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
    if (line[0] == '#') {
      continue;
    }
    sscanf(line, format.c_str(),
           &t1, &t2,
           &Z.T.p[0], &Z.T.p[1], &Z.T.p[2],
           &Z.T.R[0][0], &Z.T.R[0][1], &Z.T.R[0][2],
           &Z.T.R[1][0], &Z.T.R[1][1], &Z.T.R[1][2],
           &Z.T.R[2][0], &Z.T.R[2][1], &Z.T.R[2][2],
           &Z.S[0][0], &Z.S[0][1], &Z.S[0][2],
           &Z.S[1][0], &Z.S[1][1], &Z.S[1][2],
           &Z.S[2][0], &Z.S[2][1], &Z.S[2][2],
           &Z.S[0][3], &Z.S[0][4], &Z.S[0][5],
           &Z.S[1][3], &Z.S[1][4], &Z.S[1][5],
           &Z.S[2][3], &Z.S[2][4], &Z.S[2][5],
           &Z.S[3][3], &Z.S[3][4], &Z.S[3][5],
           &Z.S[4][3], &Z.S[4][4], &Z.S[4][5],
           &Z.S[5][3], &Z.S[5][4], &Z.S[5][5]);
    //Z.S[3][3] = Z.S[4][4] = Z.S[5][5] = sr2;
    for (int i = 0; i < 3; ++i) {
      for (int j = 3; j < 6; ++j) {
        Z.S[j][i] = Z.S[i][j];
      }
    }
    const std::vector<float>::const_iterator it1 = std::lower_bound(ts.begin(), ts.end(), t1);
    const std::vector<float>::const_iterator it2 = std::lower_bound(ts.begin(), ts.end(), t2);
    if (it1 != ts.end() && fabs(t1 - *it1) <= dtMax && it2 != ts.end() && fabs(t2 - *it2) <= dtMax) {
      Z.iFrm1 = static_cast<int>(it1 - ts.begin());
      Z.iFrm2 = static_cast<int>(it2 - ts.begin());
      Zs->push_back(Z);
    }
  }
  fclose(fp);
  UT::PrintLoaded(fileName);
  return true;
}

void PrintCameraPose(const int iFrm, const CameraPose &C, const bool n) {
  const ::Point3D p = C.p;
  const Quaternion q = Rotation3D(C.R).GetQuaternion();
  if (n) {
    UT::Print("[%d]   %e %e %e   %e %e %e %e\n", iFrm, p.x(), p.y(), p.z(),
              q.x(), q.y(), q.z(), q.w());
  } else {
    UT::Print("\r[%d]   %f %f %f   %f %f %f %f", iFrm, p.x(), p.y(), p.z(),
              q.x(), q.y(), q.z(), q.w());
  }
}

void SaveCameraPose(const int iFrm, const CameraPose &C, FILE *fp) {
  const ::Point3D p = C.p;
  const Quaternion q = Rotation3D(C.R).GetQuaternion();
  fprintf(fp, "[%d]   %e %e %e   %e %e %e %e\n", iFrm, p.x(), p.y(), p.z(),
          q.x(), q.y(), q.z(), q.w());
}

bool LoadCameraPoses(const std::string fileName, std::vector<int> *iFrms,
                     std::vector<CameraPose> *Cs) {
  iFrms->resize(0);
  FILE *fp = fopen(fileName.c_str(), "r");
  if (!fp) {
    return false;
  }
  int iFrm;
  CameraPose C;
  while (fscanf(fp, "%d", &iFrm) == 1 &&
         fscanf(fp, "%e %e %e", &C.R[0][0], &C.R[0][1], &C.R[0][2]) == 3 &&
         fscanf(fp, "%e %e %e", &C.R[1][0], &C.R[1][1], &C.R[1][2]) == 3 &&
         fscanf(fp, "%e %e %e", &C.R[2][0], &C.R[2][1], &C.R[2][2]) == 3 &&
         fscanf(fp, "%e %e %e", &C.p[0], &C.p[1], &C.p[2]) == 3) {
    iFrms->push_back(iFrm);
    Cs->push_back(C);
  }
  fclose(fp);
  return true;
}

static inline bool AssertError(const int iFrm, const float e, const float eMax, const std::string str) {
  if (e <= eMax) {
    return true;
  }
  UT::Print("[%d] %s = %f > %f\n", iFrm, str.c_str(), e, eMax);
  return false;
}

bool AssertError(const int iFrm, const Error &e) {
  const bool ex = AssertError(iFrm, e.ex, MAX_ERROR_FEATURE, "ex");
  const bool eur = AssertError(iFrm, e.eur, MAX_ERROR_IMU_ROTATION, "eur");
  const bool eup = AssertError(iFrm, e.eup, MAX_ERROR_IMU_POSITION, "eup");
  const bool euv = AssertError(iFrm, e.euv, MAX_ERROR_IMU_VELOCITY, "euv");
  const bool euba = AssertError(iFrm, e.euba, MAX_ERROR_IMU_BIAS_ACCELERATION, "euba");
  const bool eubw = AssertError(iFrm, e.eubw, MAX_ERROR_IMU_BIAS_GYROSCOPE, "eubw");
  const bool edr = AssertError(iFrm, e.edr, MAX_ERROR_DRIFT_ROTATION, "edr");
  const bool edp = AssertError(iFrm, e.edp, MAX_ERROR_DRIFT_POSITION, "edp");
  return ex && eur && eup && euv && euba && eubw && edr && edp;
}

void PrintError(const std::string str, const Error &e) {
  const std::string _str(str.size(), ' ');
  UT::PrintSeparator();
  UT::Print("%sex = %f\n", str.c_str(), e.ex);
  UT::Print("%seu = %f %f %f %f %f\n", _str.c_str(), e.eur, e.eup, e.euv, e.euba, e.eubw);
  UT::Print("%sed = %f %f\n", _str.c_str(), e.edr, e.edp);
}

}  // namespace IBA
