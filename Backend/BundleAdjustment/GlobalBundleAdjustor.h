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
#ifndef _GLOBAL_BUNDLE_ADJUSTOR_H_
#define _GLOBAL_BUNDLE_ADJUSTOR_H_

#include "IBA.h"
#include "LocalMap.h"
#include "GlobalMap.h"
#include "CameraPrior.h"
#include "Timer.h"

#define GBA_FLAG_FRAME_DEFAULT                  0
#define GBA_FLAG_FRAME_UPDATE_CAMERA            1
#define GBA_FLAG_FRAME_UPDATE_DEPTH             2
#define GBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION 4
#define GBA_FLAG_FRAME_UPDATE_DELTA             8
#define GBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION 16
#define GBA_FLAG_FRAME_UPDATE_SCHUR_COMPLEMENT  32

#define GBA_FLAG_TRACK_DEFAULT                  0
//#define GBA_FLAG_TRACK_MEASURE                1
#define GBA_FLAG_TRACK_INVALID                  1
#define GBA_FLAG_TRACK_UPDATE_DEPTH             2
#define GBA_FLAG_TRACK_UPDATE_INFORMATION       4
#define GBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO  8
#define GBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION 16

#define GBA_FLAG_CAMERA_MOTION_DEFAULT                  0
#define GBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION          1
#define GBA_FLAG_CAMERA_MOTION_UPDATE_POSITION          2
#define GBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY          4
#define GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION 8
#define GBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE    16
#define GBA_FLAG_CAMERA_MOTION_INVALID                  32

#define GBA_ME_FUNCTION   ME::FUNCTION_HUBER
//#define GBA_ME_FUNCTION ME::FUNCTION_NONE
class GlobalBundleAdjustor : public MT::Thread {

 public:

  class InputKeyFrame : public GlobalMap::InputKeyFrame {
   public:
    inline InputKeyFrame() : GlobalMap::InputKeyFrame() {}
    inline InputKeyFrame(const InputKeyFrame &IKF) { *this = IKF; }
    inline InputKeyFrame(const GlobalMap::InputKeyFrame &IKF,
                         const AlignedVector<IMU::Measurement> &us,
                         const std::vector<Depth::InverseGaussian> &dzs
#ifdef CFG_HANDLE_SCALE_JUMP
                       , const float d
#endif
                       ) : GlobalMap::InputKeyFrame(IKF) {
      m_us.Set(us);
      m_dzs = dzs;
#ifdef CFG_HANDLE_SCALE_JUMP
      m_d = d;
#endif
    }
    inline void operator = (const InputKeyFrame &IKF) {
      *((GlobalMap::InputKeyFrame *) this) = IKF;
      m_us.Set(IKF.m_us);
      m_dzs = IKF.m_dzs;
#ifdef CFG_HANDLE_SCALE_JUMP
      m_d = IKF.m_d;
#endif
    }
    inline void SaveB(FILE *fp) const {
      GlobalMap::InputKeyFrame::SaveB(fp);
      m_us.SaveB(fp);
      UT::VectorSaveB(m_dzs, fp);
#ifdef CFG_HANDLE_SCALE_JUMP
      UT::SaveB(m_d, fp);
#endif
    }
    inline void LoadB(FILE *fp) {
      GlobalMap::InputKeyFrame::LoadB(fp);
      m_us.LoadB(fp);
      UT::VectorLoadB(m_dzs, fp);
#ifdef CFG_HANDLE_SCALE_JUMP
      UT::LoadB(m_d, fp);
#endif
    }
   public:
    AlignedVector<IMU::Measurement> m_us;
    std::vector<Depth::InverseGaussian> m_dzs;
#ifdef CFG_HANDLE_SCALE_JUMP
    float m_d;
#endif
  };
  
 public:

  virtual void Initialize(IBA::Solver *solver, const int serial = 0, const int verbose = 0,
                          const int debug = 0, const int history = 0);
  virtual void Reset();
  virtual void PushKeyFrame(const GlobalMap::InputKeyFrame &IKF,
                            const AlignedVector<IMU::Measurement> &us,
                            const std::vector<Depth::InverseGaussian> &dzs
#ifdef CFG_HANDLE_SCALE_JUMP
                          , const float d
#endif
                          );
  virtual void PushDeleteKeyFrame(const int iFrm, const int iKF);
  virtual void PushDeleteMapPoints(const int iFrm, const std::vector<int> &ids);
  virtual void PushUpdateCameras(const int iFrm, const std::vector<GlobalMap::InputCamera> &Cs);
  virtual void PushCameraPriorPose(const int iFrm, const CameraPrior::Pose &IZp);
  virtual void PushCameraPriorMotion(const int iFrm, const int iKF, const CameraPrior::Motion &IZp);
  virtual void Run();
  //virtual int GetTotalPoints(int *N = NULL);
  virtual float GetTotalTime(int *N = NULL);
  virtual bool SaveTimes(const std::string fileName);
  virtual bool SaveCameras(const std::string fileName, const bool poseOnly = true);
  virtual bool SaveCosts(const std::string fileName, const int type = 0);
  virtual bool SaveResiduals(const std::string fileName, const int type = 0);
  virtual void ComputeErrorFeature(float *ex);
  virtual void ComputeErrorIMU(float *er, float *ep, float *ev, float *eba, float *ebw);
  virtual void ComputeErrorDrift(float *er, float *ep);
  virtual float ComputeRMSE();

  virtual void SaveB(FILE *fp);
  virtual void LoadB(FILE *fp);

  virtual void SetCallback(const IBA::Solver::IbaCallback& callback_iba);

 public:

  class KeyFrame : public GlobalMap::KeyFrameBA {
   public:
    inline KeyFrame() : KeyFrameBA() {}
    inline KeyFrame(const KeyFrame &KF) { *this = KF; }
    inline void operator = (const KeyFrame &KF) {
      *((KeyFrameBA *) this) = KF;
      m_us.Set(KF.m_us);
      m_Axs.Set(KF.m_Axs);
      m_Mxs1.Set(KF.m_Mxs1);
      m_Mxs2.Set(KF.m_Mxs2);
      m_Lzs.Set(KF.m_Lzs);
      m_Azs1.Set(KF.m_Azs1);
      m_Azs2.Set(KF.m_Azs2);
      m_Mzs1.Set(KF.m_Mzs1);
      m_Mzs2.Set(KF.m_Mzs2);
      m_SAcxzs.Set(KF.m_SAcxzs);
      m_Zm = KF.m_Zm;

      m_iKFsPrior = KF.m_iKFsPrior;
      m_iAp2kp = KF.m_iAp2kp;
      m_ikp2KF = KF.m_ikp2KF;
      m_SAps.Set(KF.m_SAps);
    }
    inline void Initialize(const InputKeyFrame &IKF) {
      KeyFrameBA::Initialize(IKF);
      m_us.Set(IKF.m_us);
      const int Nz = static_cast<int>(m_zs.size()), NZ = static_cast<int>(m_Zs.size());
      m_Lzs.Resize(Nz);     m_Lzs.MakeZero();
      m_Azs1.Resize(Nz);    m_Azs1.MakeZero();
      m_Azs2.Resize(Nz);    m_Azs2.MakeZero();
      m_Mzs1.Resize(Nz);    m_Mzs1.MakeZero();
      m_Mzs2.Resize(Nz);    m_Mzs2.MakeZero();
      m_SAcxzs.Resize(NZ);  m_SAcxzs.MakeZero();
      m_Zm.Initialize();

      m_iKFsPrior.resize(0);
      m_iAp2kp.resize(0);
      m_ikp2KF = m_iKFsMatch;
      m_SAps.Resize(0);
    }
    inline void PushFeatures(const std::vector<FTR::Source> &xs) {
      KeyFrameBA::PushFeatures(xs);
      const int ix = m_Axs.Size(), Nx = static_cast<int>(xs.size());
      m_Axs.InsertZero(ix, Nx, NULL);
      m_Mxs1.InsertZero(ix, Nx, NULL);
      m_Mxs2.InsertZero(ix, Nx, NULL);
#ifdef CFG_DEBUG
      for (int jx = ix; jx < Nx; ++jx) {
        m_Mxs1[jx].m_mdx.Invalidate();
        m_Mxs2[jx].m_Mcxx.Invalidate();
      }
#endif
    }
    inline void PushFeatureMeasurements(const int iKF, const std::vector<FTR::Measurement> &zs,
                                        int *ik, int *iz, AlignedVector<float> *work) {
      int iZ;
      const int Nk = static_cast<int>(m_iKFsMatch.size());
      KeyFrameBA::PushFeatureMeasurements(iKF, zs, &iZ, iz);
      *ik = m_Zs[iZ].m_ik;
      const int Nz = static_cast<int>(zs.size());
      m_Lzs.InsertZero(*iz, Nz, work);
      m_Azs1.InsertZero(*iz, Nz, work);
      m_Azs2.InsertZero(*iz, Nz, work);
      m_Mzs1.InsertZero(*iz, Nz, work);
      m_Mzs2.InsertZero(*iz, Nz, work);
      if (static_cast<int>(m_Zs.size()) > m_SAcxzs.Size()) {
        m_SAcxzs.InsertZero(iZ);
        if (static_cast<int>(m_iKFsMatch.size()) > Nk) {
          m_Zm.InsertKeyFrame(*ik);
          m_ikp2KF.insert(m_ikp2KF.begin() + *ik, iKF);
          const int Np = static_cast<int>(m_iKFsPrior.size());
          for (int ip = 0; ip < Np; ++ip) {
            if (m_iAp2kp[ip] >= *ik) {
              ++m_iAp2kp[ip];
            }
          }
        }
      }
    }
    inline void DeleteKeyFrame(const int iKF,
                               const std::vector<FRM::Measurement>::iterator *iZ = NULL) {
      const std::vector<FRM::Measurement>::iterator _iZ = iZ ? *iZ : std::lower_bound(m_Zs.begin(),
                                                                                      m_Zs.end(), iKF);
      if (_iZ != m_Zs.end() && _iZ->m_iKF == iKF) {
        const int Nz = _iZ->CountFeatureMeasurements();
        m_Lzs.Erase(_iZ->m_iz1, Nz);
        m_Azs1.Erase(_iZ->m_iz1, Nz);
        m_Azs2.Erase(_iZ->m_iz1, Nz);
        m_Mzs1.Erase(_iZ->m_iz1, Nz);
        m_Mzs2.Erase(_iZ->m_iz1, Nz);
        m_SAcxzs.Erase(static_cast<int>(_iZ - m_Zs.begin()));
      }
      const std::vector<int>::iterator ik = std::lower_bound(m_iKFsMatch.begin(),
                                                             m_iKFsMatch.end(), iKF);
      const int _ik = (ik != m_iKFsMatch.end() && *ik == iKF) ?
                      static_cast<int>(ik - m_iKFsMatch.begin()) : -1;
      if (_ik != -1) {
        m_Zm.DeleteKeyFrame(_ik);
      }
      const std::vector<int>::iterator ip = std::lower_bound(m_iKFsPrior.begin(),
                                                             m_iKFsPrior.end(), iKF);
      const int _ip = (ip != m_iKFsPrior.end() && *ip == iKF) ?
                      static_cast<int>(ip - m_iKFsPrior.begin()) : -1;
      if (ip != m_iKFsPrior.end()) {
        for (std::vector<int>::iterator jp = *ip == iKF ? ip + 1 : ip; jp != m_iKFsPrior.end(); ++jp) {
          --*jp;
        }
      }
      if (ik != m_iKFsMatch.end() || ip != m_iKFsPrior.end()) {
        const int Nkp = static_cast<int>(m_ikp2KF.size());
        for (int jkp = 0; jkp < Nkp; ++jkp) {
          if (m_ikp2KF[jkp] > iKF) {
            --m_ikp2KF[jkp];
          }
        }
      }
      if (_ik != -1 || _ip != -1) {
        const int Nk = static_cast<int>(m_iKFsMatch.size());
        const int ikp = _ik != -1 ? _ik : m_iAp2kp[_ip];
        const int Np = static_cast<int>(m_iAp2kp.size());
        for (int jp = 0; jp < Np; ++jp) {
          if (m_iAp2kp[jp] > ikp) {
            --m_iAp2kp[jp];
          }
        }
        if (_ip != -1) {
          m_iKFsPrior.erase(ip);
          m_iAp2kp.erase(m_iAp2kp.begin() + _ip);
          m_SAps.Erase(_ip);
        }
        m_ikp2KF.erase(m_ikp2KF.begin() + ikp);
      }
      KeyFrameBA::DeleteKeyFrame(iKF, &_iZ, &ik);
    }
    inline void DeleteFeatureMeasurements(const std::vector<int> &izs) {
      const int Nz1 = static_cast<int>(m_zs.size());
      KeyFrameBA::DeleteFeatureMeasurements(izs);
      for (int iz1 = 0; iz1 < Nz1; ++iz1) {
        const int iz2 = izs[iz1];
        if (iz2 < 0) {
          continue;
        }
        m_Lzs[iz2] = m_Lzs[iz1];
        m_Azs1[iz2] = m_Azs1[iz1];
        m_Azs2[iz2] = m_Azs2[iz1];
        m_Mzs1[iz2] = m_Mzs1[iz1];
        m_Mzs2[iz2] = m_Mzs2[iz1];
      }
      const int Nz2 = static_cast<int>(m_zs.size());
      m_Lzs.Resize(Nz2);
      m_Azs1.Resize(Nz2);
      m_Azs2.Resize(Nz2);
      m_Mzs1.Resize(Nz2);
      m_Mzs2.Resize(Nz2);
    }
    inline void InvalidateFeatures(const ubyte *mxs) {
      KeyFrameBA::InvalidateFeatures(mxs);
      const int Nx = static_cast<int>(m_xs.size());
      for (int ix = 0, cnt = 0; ix < Nx; ++ix) {
        if (!mxs[ix]) {
          continue;
        }
        m_Axs[ix].MakeZero();
        m_Mxs1[ix].MakeZero();
        m_Mxs2[ix].MakeZero();
      }
    }
    inline void MakeZero() {
      KeyFrameBA::MakeZero();
      m_Axs.MakeZero();
      m_Mxs1.MakeZero();
      m_Mxs2.MakeZero();
      m_Lzs.MakeZero();
      m_Azs1.MakeZero();
      m_Azs2.MakeZero();
      m_Mzs1.MakeZero();
      m_Mzs2.MakeZero();
      m_SAcxzs.MakeZero();
      m_Zm.MakeZero();
      m_SAps.MakeZero();
#ifdef CFG_DEBUG
      const int Nx = int(m_xs.size());
      for (int ix = 0; ix < Nx; ++ix) {
        m_Mxs1[ix].m_mdx.Invalidate();
        m_Mxs2[ix].m_Mcxx.Invalidate();
      }
#endif
    }
    inline void InsertMatchKeyFrame(const int iKF, const std::vector<int>::iterator *ik = NULL) {
      std::vector<int>::iterator _ik;
      if (m_iKFsMatch.empty() || iKF > m_iKFsMatch.back()) {
        _ik = m_iKFsMatch.end();
      } else {
        _ik = ik ? *ik : std::lower_bound(m_iKFsMatch.begin(), m_iKFsMatch.end(), iKF);
        if (_ik != m_iKFsMatch.end() && *_ik == iKF) {
          return;
        }
      }
      const int jk = static_cast<int>(_ik - m_iKFsMatch.begin());
      KeyFrameBA::InsertMatchKeyFrame(iKF, &_ik);
      m_Zm.InsertKeyFrame(jk);
      m_ikp2KF.insert(m_ikp2KF.begin() + jk, iKF);
      const int Np = static_cast<int>(m_iKFsPrior.size());
      for (int ip = 0; ip < Np; ++ip) {
        if (m_iAp2kp[ip] >= jk) {
          ++m_iAp2kp[ip];
        }
      }
    }
    inline int PushCameraPrior(const int iKF, AlignedVector<float> *work) {
      const std::vector<int>::iterator ip = std::lower_bound(m_iKFsPrior.begin(),
                                                             m_iKFsPrior.end(), iKF);
      if (ip != m_iKFsPrior.end() && *ip == iKF) {
        return -1;
      }
      int _ip;
      const int ik = SearchMatchKeyFrame(iKF);
      const int ikp = ik == -1 ? static_cast<int>(m_ikp2KF.size()) : ik;
      if (ip == m_iKFsPrior.end()) {
        _ip = int(m_iKFsPrior.size());
        m_iKFsPrior.push_back(iKF);
        m_iAp2kp.push_back(ikp);
        m_SAps.Resize(_ip + 1, true);
      } else {
        _ip = int(ip - m_iKFsPrior.begin());
        m_iKFsPrior.insert(ip, iKF);
        m_iAp2kp.insert(m_iAp2kp.begin() + _ip, ikp);
        m_SAps.Insert(_ip, 1, work);
      }
      m_SAps[_ip].MakeZero();
      if (ik == -1) {
        m_ikp2KF.push_back(iKF);
      }
      return _ip;
    }
    inline int SearchPriorKeyFrame(const int iKF) const {
      const std::vector<int>::const_iterator ip = std::lower_bound(m_iKFsPrior.begin(),
                                                                   m_iKFsPrior.end(), iKF);
      return (ip == m_iKFsPrior.end() || *ip != iKF) ? -1 : int(ip - m_iKFsPrior.begin());
    }
    inline void SaveB(FILE *fp) const {
      KeyFrameBA::SaveB(fp);
      m_us.SaveB(fp);
      m_Axs.SaveB(fp);
      m_Mxs1.SaveB(fp);
      m_Mxs2.SaveB(fp);
      m_Lzs.SaveB(fp);
      m_Azs1.SaveB(fp);
      m_Azs2.SaveB(fp);
      m_Mzs1.SaveB(fp);
      m_Mzs2.SaveB(fp);
      m_SAcxzs.SaveB(fp);
      m_Zm.SaveB(fp);
      UT::VectorSaveB(m_iKFsPrior, fp);
      UT::VectorSaveB(m_iAp2kp, fp);
      UT::VectorSaveB(m_ikp2KF, fp);
      m_SAps.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
      KeyFrameBA::LoadB(fp);
      m_us.LoadB(fp);
      m_Axs.LoadB(fp);
      m_Mxs1.LoadB(fp);
      m_Mxs2.LoadB(fp);
      m_Lzs.LoadB(fp);
      m_Azs1.LoadB(fp);
      m_Azs2.LoadB(fp);
      m_Mzs1.LoadB(fp);
      m_Mzs2.LoadB(fp);
      m_SAcxzs.LoadB(fp);
      m_Zm.LoadB(fp);
      UT::VectorLoadB(m_iKFsPrior, fp);
      UT::VectorLoadB(m_iAp2kp, fp);
      UT::VectorLoadB(m_ikp2KF, fp);
      m_SAps.LoadB(fp);
    }
    inline void AssertConsistency(const int iKF) const {
      KeyFrameBA::AssertConsistency(iKF);
      const int Nx = static_cast<int>(m_xs.size());
      UT_ASSERT(m_Axs.Size() == Nx && m_Mxs1.Size() == Nx && m_Mxs2.Size() == Nx);
      const int Nz = static_cast<int>(m_zs.size());
      UT_ASSERT(m_Lzs.Size() == Nz);
      UT_ASSERT(m_Azs1.Size() == Nz && m_Azs2.Size() == Nz);
      UT_ASSERT(m_Mzs1.Size() == Nz && m_Mzs2.Size() == Nz);
#ifdef CFG_STEREO
      for (int iz = 0; iz < Nz; ++iz) {
        UT_ASSERT(m_Lzs[iz].m_Je.Valid() || m_Lzs[iz].m_Jer.Valid());
      }
#endif
      UT_ASSERT(m_SAcxzs.Size() == static_cast<int>(m_Zs.size()));
      //UT_ASSERT(m_iKFsMatch.empty() || m_iKFsMatch.back() < iKF);
      //UT_ASSERT(m_iKFsPrior.empty() || m_iKFsPrior.back() < iKF);
      const int Nk = static_cast<int>(std::lower_bound(m_iKFsMatch.begin(),
                                                       m_iKFsMatch.end(), iKF) -
                                                       m_iKFsMatch.begin());
      const int Np = static_cast<int>(std::lower_bound(m_iKFsPrior.begin(),
                                                       m_iKFsPrior.end(), iKF) -
                                                       m_iKFsPrior.begin());
      //m_Zm.AssertConsistency(static_cast<int>(m_iKFsMatch.size()));
      m_Zm.AssertConsistency(Nk);
      UT_ASSERT(static_cast<int>(m_iAp2kp.size()) == Np);
      UT_ASSERT(m_SAps.Size() == Np);
      const int _Np = static_cast<int>(m_iKFsPrior.size());
      for (int ip = 1; ip < _Np; ++ip) {
        UT_ASSERT(m_iKFsPrior[ip - 1] < m_iKFsPrior[ip]);
      }
      for (int ik = 0; ik < Nk; ++ik) {
        UT_ASSERT(m_iKFsMatch[ik] == m_ikp2KF[ik]);
      }
      int SNkp = Nk;
      for (int ip = 0; ip < Np; ++ip) {
        const int _iKF = m_iKFsPrior[ip], ikp = m_iAp2kp[ip];
        UT_ASSERT(m_ikp2KF[ikp] == _iKF);
        if (ikp < Nk) {
          UT_ASSERT(m_iKFsMatch[ikp] == _iKF);
        } else {
          ++SNkp;
        }
      }
      UT_ASSERT(static_cast<int>(m_ikp2KF.size()) == SNkp);
    }
   public:
    AlignedVector<IMU::Measurement> m_us;
    AlignedVector<FTR::Factor::Full::Source::A> m_Axs;
    AlignedVector<FTR::Factor::Full::Source::M1> m_Mxs1;
    AlignedVector<FTR::Factor::Full::Source::M2> m_Mxs2;
    AlignedVector<FTR::Factor::Full::L> m_Lzs;
    AlignedVector<FTR::Factor::Full::A1> m_Azs1;
    AlignedVector<FTR::Factor::Full::A2> m_Azs2;
    AlignedVector<FTR::Factor::Full::M1> m_Mzs1;
    AlignedVector<FTR::Factor::Full::M2> m_Mzs2;
    AlignedVector<Camera::Factor::Binary::CC> m_SAcxzs;
    FRM::MeasurementMatch m_Zm;
    std::vector<int> m_iKFsPrior, m_iAp2kp, m_ikp2KF;
    AlignedVector<Camera::Factor::Binary::CC> m_SAps;
  };
  
  class CameraPriorMotion : public CameraPrior::Motion {
   public:
    inline void Set(const int iKF, const CameraPrior::Motion &Zp) {
      m_iKF = iKF;
      *((CameraPrior::Motion *) this) = Zp;
    }
    inline void operator = (const CameraPriorMotion &Zp) {
      m_iKF = Zp.m_iKF;
      *((CameraPrior::Motion *) this) = Zp;
    }
    inline bool DeleteKeyFrame(const int iKF) {
      if (m_iKF == iKF) {
        Invalidate();
        return true;
      } else if (m_iKF > iKF) {
        --m_iKF;
      }
      return false;
    }
    inline bool Valid() const { return m_iKF != -1; }
    inline bool Invalid() const { return m_iKF == -1; }
    inline void Invalidate() { m_iKF = -1; }
    inline void SaveB(FILE *fp) const { CameraPrior::Motion::SaveB(fp); UT::SaveB(m_iKF, fp); }
    inline void LoadB(FILE *fp) { CameraPrior::Motion::LoadB(fp); UT::LoadB(m_iKF, fp); }
   public:
    int m_iKF;
  };

  enum TimerType { TM_TOTAL, TM_SYNCHRONIZE, TM_FACTOR, TM_SCHUR_COMPLEMENT, TM_CAMERA, TM_DEPTH,
                   TM_UPDATE, TM_TYPES };
  class ES {
   public:
    inline float Total() const {
      return m_ESx.Total() + m_ESc.Total() + m_ESm.Total() + m_ESd.Total() +
             m_ESo.Total() + m_ESfp.Total() + m_ESfm.Total();
    }
    inline void Save(FILE *fp) const {
      fprintf(fp, "%e %e %e %e %e %e\n", Total(), m_ESx.Total(), m_ESc.Total(), m_ESm.Total(),
                                                  m_ESd.Total(), m_ESo.Total());
    }
   public:
    FTR::ES m_ESx;
    CameraPrior::Pose::ES m_ESc;
    CameraPrior::Motion::ES m_ESm;
    IMU::Delta::ES m_ESd;
    Camera::Fix::Origin::ES m_ESo;
    Camera::Fix::PositionZ::ES m_ESfp;
    Camera::Fix::Motion::ES m_ESfm;
  };
  class Residual {
   public:
    inline void Save(FILE *fp) const {
      fprintf(fp, "%e %e\n", m_r2, m_F);
    }
   public:
    float m_r2, m_F;
  };
  class History {
   public:
    inline void MakeZero() { memset(this, 0, sizeof(History)); }
   public:
    // m_history >= 1
    double m_ts[TM_TYPES];
    //int m_Nd;
    // m_history >= 2
    ES m_ESa, m_ESb, m_ESp;
#ifdef CFG_GROUND_TRUTH
    // m_history >= 3 using only available depth measurements
    ES m_ESaGT, m_ESpGT;
#endif
    // m_history >= 2
    Residual m_R;
#ifdef CFG_GROUND_TRUTH
    Residual m_RGT;
#endif
  };
  class HistoryCamera {
   public:
    inline HistoryCamera() {}
    inline HistoryCamera(const FRM::Tag &T, const Rigid3D &C) : m_iFrm(T.m_iFrm), m_t(T.m_t),
                                                                m_C(C) {}
    inline bool operator < (const HistoryCamera &C) const { return m_iFrm < C.m_iFrm; }
   public:
    int m_iFrm;
    float m_t;
    Rigid3D m_C;
  };

 protected:

  virtual void SynchronizeData();
  virtual void UpdateData();
  virtual bool BufferDataEmpty();
  
  virtual void PushKeyFrame(const InputKeyFrame &IKF);
  virtual void DeleteKeyFrame(const int iKF);
  virtual void DeleteMapPoints(const std::vector<int> &ids);
  virtual void UpdateCameras(const std::vector<GlobalMap::InputCamera> &Cs);
  virtual void PushCameraPriorPose(const CameraPrior::Pose &Zp);
  virtual void SetCameraPriorMotion(const CameraPriorMotion &Zp);
  virtual void PushFeatureMeasurementMatchesFirst(const FRM::Frame &F, std::vector<int> &iKF2X,
                                                  std::vector<int> &iX2z);
  virtual void PushFeatureMeasurementMatchesNext(const FRM::Frame &F1, const FRM::Frame &F2,
                                                 const std::vector<int> &iKF2X, const std::vector<int> &iX2z2,
                                                 FRM::MeasurementMatch &Zm);
  virtual int CountMeasurementsFrame();
  virtual int CountMeasurementsFeature();
  virtual int CountMeasurementsPriorCameraPose();
  virtual int CountSchurComplements();
  virtual int CountSchurComplementsOffDiagonal();

  virtual void UpdateFactors();
  virtual void UpdateFactorsFeature();
  virtual void UpdateFactorsPriorCameraPose();
  virtual void UpdateFactorsPriorCameraMotion();
  virtual void UpdateFactorsPriorDepth();
  virtual void UpdateFactorsIMU();
  virtual void UpdateFactorsFixOrigin();
  virtual void UpdateFactorsFixPositionZ();
  virtual void UpdateFactorsFixMotion();
  virtual void UpdateSchurComplement();
  virtual bool SolveSchurComplement();
  virtual bool SolveSchurComplementPCG();
#ifdef CFG_GROUND_TRUTH
  virtual void SolveSchurComplementGT(const AlignedVector<Rigid3D> &Cs,
                                      const AlignedVector<Camera> &CsLM,
                                      LA::AlignedVectorXf *xs,
                                      const bool motion = true);
#endif
  virtual void PrepareConditioner();
  virtual void ApplyM(const LA::AlignedVectorX<PCG_TYPE> &xs, LA::AlignedVectorX<PCG_TYPE> *Mxs);
  virtual void ApplyA(const LA::AlignedVectorX<PCG_TYPE> &xs, LA::AlignedVectorX<PCG_TYPE> *Axs);
  virtual void ApplyAcm(const LA::ProductVector6f *xcs, const LA::Vector9d *xms,
                        LA::Vector6d *Axcs, LA::Vector9d *Axms, const bool Acc = true,
                        const LA::AlignedMatrix9x9f *Amus = NULL);
  virtual void ApplyAcm(const LA::ProductVector6f *xcs, const LA::Vector9f *xms,
                        LA::Vector6f *Axcs, LA::Vector9f *Axms, const bool Acc = true,
                        const LA::AlignedMatrix9x9f *Amus = NULL);
  virtual Residual ComputeResidual(const LA::AlignedVectorX<PCG_TYPE> &xs,
                                   const bool minmus = false);
  virtual void SolveBackSubstitution();
#ifdef CFG_GROUND_TRUTH
  virtual void SolveBackSubstitutionGT(const std::vector<Depth::InverseGaussian> &ds,
                                       LA::AlignedVectorXf *xs);
#endif
  virtual bool EmbeddedMotionIteration();
  virtual void EmbeddedPointIteration(const AlignedVector<Rigid3D> &Cs,
                                      const std::vector<ubyte> &ucs,
                                      const std::vector<ubyte> &uds,
                                      std::vector<Depth::InverseGaussian> *ds);

  virtual void SolveDogLeg();
  virtual void SolveGradientDescent();
  virtual void ComputeReduction();
  virtual void ComputeReductionFeature();
  virtual void ComputeReductionPriorCameraPose();
  virtual void ComputeReductionPriorCameraMotion();
  virtual void ComputeReductionPriorDepth();
  virtual void ComputeReductionIMU();
  virtual void ComputeReductionFixOrigin();
  virtual void ComputeReductionFixPositionZ();
  virtual void ComputeReductionFixMotion();
  virtual bool UpdateStatesPropose();
  virtual bool UpdateStatesDecide();
  virtual void ConvertCameraUpdates(const float *xcs, LA::AlignedVectorXf *xp2s,
                                    LA::AlignedVectorXf *xr2s);
  virtual void ConvertCameraUpdates(const LA::Vector6f *xcs,
                                    AlignedVector<LA::ProductVector6f> *xcsP);
  virtual void ConvertCameraUpdates(const LA::Vector6d *xcs,
                                    AlignedVector<LA::ProductVector6f> *xcsP);
  virtual void ConvertCameraUpdates(const AlignedVector<LA::AlignedVector6f> &xcsA,
                                    LA::Vector6f *xcs);
  virtual void ConvertMotionUpdates(const float *xms, LA::AlignedVectorXf *xv2s,
                                    LA::AlignedVectorXf *xba2s, LA::AlignedVectorXf *xbw2s);
  virtual void ConvertCameraMotionResiduals(const LA::AlignedVectorX<PCG_TYPE> &rs,
                                            const LA::AlignedVectorX<PCG_TYPE> &zs,
                                            PCG_TYPE *Se2, PCG_TYPE *e2Max);
  virtual void ConvertDepthUpdates(const float *xs, LA::AlignedVectorXf *xds);
  virtual void PushDepthUpdates(const LA::AlignedVectorXf &xds, LA::AlignedVectorXf *xs);
  virtual float AverageDepths(const Depth::InverseGaussian *ds, const int N);

  virtual float PrintErrorStatistic(const std::string str, const AlignedVector<Rigid3D> &Cs,
                                    const AlignedVector<Camera> &CsLM,
                                    const std::vector<Depth::InverseGaussian> &ds,
                                    const AlignedVector<IMU::Delta> &DsLM, const bool detail);
  virtual ES ComputeErrorStatistic(const AlignedVector<Rigid3D> &Cs,
                                   const AlignedVector<Camera> &CsLM,
                                   const std::vector<Depth::InverseGaussian> &ds,
                                   const AlignedVector<IMU::Delta> &DsLM);
  virtual FTR::ES ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Rigid3D> &Cs,
                                                         const Depth::InverseGaussian *ds);
  virtual CameraPrior::Pose::ES ComputeErrorStatisticPriorCameraPose(const AlignedVector<Rigid3D>
                                                                     &Cs);
  virtual CameraPrior::Motion::ES ComputeErrorStatisticPriorCameraMotion(const
                                                                         AlignedVector<Camera> &CsLM);
  virtual IMU::Delta::ES ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLM,
                                                  const AlignedVector<IMU::Delta> &DsLM);
  virtual Camera::Fix::Origin::ES ComputeErrorStatisticFixOrigin(const AlignedVector<Rigid3D> &Cs);
  virtual Camera::Fix::PositionZ::ES ComputeErrorStatisticFixPositionZ(const
                                                                       AlignedVector<Rigid3D> &Cs);
  virtual Camera::Fix::Motion::ES ComputeErrorStatisticFixMotion(const
                                                                 AlignedVector<Camera> &CsLM);
  virtual float PrintErrorStatistic(const std::string str, const AlignedVector<Rigid3D> &Cs,
                                    const AlignedVector<Camera> &CsLM,
                                    const std::vector<Depth::InverseGaussian> &ds,
                                    const AlignedVector<IMU::Delta> &DsLM,
                                    const LA::AlignedVectorXf &xs, const bool detail);
  virtual ES ComputeErrorStatistic(const AlignedVector<Rigid3D> &Cs,
                                   const AlignedVector<Camera> &CsLM,
                                   const std::vector<Depth::InverseGaussian> &ds,
                                   const AlignedVector<IMU::Delta> &DsLM,
                                   const LA::AlignedVectorXf &xs,
                                   const bool updateOnly = true);
  virtual FTR::ES ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Rigid3D> &Cs,
                                                         const Depth::InverseGaussian *ds,
                                                         const AlignedVector<LA::ProductVector6f> &xcs,
                                                         const LA::AlignedVectorXf &xds,
                                                         const bool updateOnly = true);
  virtual CameraPrior::Pose::ES ComputeErrorStatisticPriorCameraPose(const AlignedVector<Rigid3D> &Cs,
                                                                     const AlignedVector<LA::ProductVector6f> &xcs,
                                                                     const bool updateOnly = true);
  virtual CameraPrior::Motion::ES ComputeErrorStatisticPriorCameraMotion(const
                                                                         AlignedVector<Camera> &CsLM,
                                                                         const LA::ProductVector6f *xcs,
                                                                         const LA::Vector9f *xms,
                                                                         const bool updateOnly = true);
  virtual IMU::Delta::ES ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLM,
                                                  const AlignedVector<IMU::Delta> &DsLM,
                                                  const LA::ProductVector6f *xcs,
                                                  const LA::Vector9f *xms,
                                                  const bool updateOnly = true);
  virtual Camera::Fix::Origin::ES ComputeErrorStatisticFixOrigin(const AlignedVector<Rigid3D> &Cs,
                                                                 const AlignedVector<LA::ProductVector6f> &xcs,
                                                                 const bool updateOnly = true);
  virtual Camera::Fix::PositionZ::ES ComputeErrorStatisticFixPositionZ(const AlignedVector<Rigid3D>
                                                                       &Cs, const AlignedVector<LA::ProductVector6f> &xcs,
                                                                       const bool updateOnly = true);
  virtual Camera::Fix::Motion::ES ComputeErrorStatisticFixMotion(const AlignedVector<Camera> &CsLM,
                                                                 const LA::Vector9f *xms,
                                                                 const bool updateOnly = true);
  virtual void AssertConsistency(const bool chkFlag = true, const bool chkSchur = true);

 protected:
   
  friend class IBA::Solver;
  friend class IBA::Internal;
  friend class LocalBundleAdjustor;
  friend class ViewerIBA;

  IBA::Solver *m_solver;
  LocalMap *m_LM;
  GlobalMap *m_GM;
  class LocalBundleAdjustor *m_LBA;
  Camera::Calibration m_K;
  int m_verbose, m_debug, m_history;
  const Camera *m_CsGT;
  std::string m_dir;

  enum InputType { IT_KEY_FRAME, IT_DELETE_KEY_FRAME, IT_DELETE_MAP_POINTS, IT_UPDATE_CAMERAS,
                   IT_CAMERA_PRIOR_POSE, IT_CAMERA_PRIOR_MOTION };
  std::list<InputType> m_ITs1, m_ITs2;
  std::list<InputKeyFrame> m_IKFs1, m_IKFs2;
  std::list<int> m_IDKFs1, m_IDKFs2;
  std::list<std::vector<int> > m_IDMPs1, m_IDMPs2;
  std::list<std::vector<GlobalMap::InputCamera> > m_IUCs1, m_IUCs2;
  std::list<CameraPrior::Pose> m_IZps1, m_IZps2;
  CameraPriorMotion m_IZpLM1, m_IZpLM2;

  Timer m_ts[TM_TYPES];
#ifdef CFG_HISTORY
  std::vector<History> m_hists;
#endif
  std::vector<HistoryCamera> m_CsDel;

  int m_iIter, m_iIterPCG, m_iIterDL;
  float m_delta2;

  Camera::Fix::Origin m_Zo;
  Camera::Fix::Origin::Factor m_Ao;
  std::vector<CameraPrior::Pose> m_Zps;
  SIMD::vector<CameraPrior::Pose::Factor> m_Aps;
  CameraPriorMotion m_ZpLM;
  CameraPrior::Motion::Factor m_ApLM;

  std::vector<KeyFrame> m_KFs;
  std::vector<int> m_iFrms;
  AlignedVector<Rigid3D> m_Cs;
  AlignedVector<Camera> m_CsLM;
#ifdef CFG_GROUND_TRUTH
  AlignedVector<Rigid3D> m_CsKFGT;
  AlignedVector<Camera> m_CsLMGT;
#endif
  std::vector<ubyte> m_ucs, m_ucmsLM;
#ifdef CFG_GROUND_TRUTH
  std::vector<ubyte> m_ucsGT;
#endif
#ifdef CFG_HANDLE_SCALE_JUMP
  std::vector<float> m_dsKF;
#endif
#ifdef CFG_INCREMENTAL_PCG
  AlignedVector<LA::Vector6f> m_xcs;
  AlignedVector<LA::Vector9f> m_xmsLM;
#endif
  AlignedVector<IMU::Delta> m_DsLM;
#ifdef CFG_GROUND_TRUTH
  AlignedVector<IMU::Delta> m_DsLMGT;
#endif
  AlignedVector<IMU::Delta::Factor> m_AdsLM;

  AlignedVector<Camera::Fix::PositionZ::Factor> m_Afps;
  AlignedVector<Camera::Fix::Motion::Factor> m_AfmsLM;

  std::vector<int> m_iKF2d;
  std::vector<Depth::InverseGaussian> m_ds;
#ifdef CFG_GROUND_TRUTH
  std::vector<Depth::InverseGaussian> *m_dsGT;
#endif
  std::vector<ubyte> m_uds;
#ifdef CFG_GROUND_TRUTH
  std::vector<ubyte> m_udsGT;
#endif

  AlignedVector<Camera::Factor::Unitary::CC> m_SAcus, m_SMcus;
  AlignedVector<Camera::Factor> m_SAcmsLM;

  std::vector<ubyte> m_Ucs/*, m_Uds*/;

  std::vector<int> m_iKF2cb;
  AlignedVector<LA::AlignedMatrix6x6f> m_Acus, m_Acbs, m_AcbTs;
  AlignedVector<LA::AlignedMatrix9x9f> m_AmusLM;
  //LA::AlignedVectorXf m_ss;
  AlignedVector<Camera::Conditioner::C> m_Mcs;
  AlignedVector<Camera::Conditioner::M> m_MmsLM;
  AlignedMatrixX<LA::AlignedMatrix6x6f> m_Mcc, m_MccT;
  AlignedMatrixX<LA::AlignedMatrix6x9f> m_McmLM, m_MmcTLM;
  AlignedMatrixX<LA::AlignedMatrix9x6f> m_MmcLM, m_McmTLM;
  AlignedMatrixX<LA::AlignedMatrix9x9f> m_MmmLM, m_MmmTLM;
  AlignedVector<LA::ProductVector6f> m_bcs;
  AlignedVector<LA::AlignedVector9f> m_bmsLM;
  LA::AlignedVectorXf m_bs;
#ifdef CFG_PCG_DOUBLE
  LA::AlignedVectorXd m_xs, m_rs, m_ps, m_zs, m_drs, m_dxs, m_e2s;
#else
  LA::AlignedVectorXf m_xs, m_rs, m_ps, m_zs, m_drs, m_dxs, m_e2s;
#endif
  LA::AlignedVectorXf m_xp2s, m_xr2s, m_xv2s, m_xba2s, m_xbw2s, m_xds, m_x2s, m_axds;
  AlignedVector<LA::ProductVector6f> m_xcsP;
  AlignedVector<LA::AlignedVector6f> m_Axcs;
  AlignedVector<LA::AlignedVector3f> m_xps, m_xrs;
  std::vector<const LA::AlignedVector3f *> m_xpsP, m_xrsP;
  std::vector<int> m_iKF2X;

  AlignedVector<Rigid3D> m_CsBkp;
  AlignedVector<Camera> m_CsLMBkp;
  std::vector<Depth::InverseGaussian> m_dsBkp;
  AlignedVector<LA::Vector6f> m_xcsBkp;
  AlignedVector<LA::Vector9f> m_xmsLMBkp;

  LA::AlignedVectorXf m_xsGN, m_xsGD, m_xsDL, m_xsGT;
  AlignedVector<LA::ProductVector6f> m_gcs;
  LA::AlignedVectorXf m_gds, m_Agds, m_Ags;
  float m_x2GN, m_x2GD, m_x2DL, m_bl, m_gTAg, m_beta, m_dFa, m_dFp, m_rho;
  bool m_update, m_converge, m_empty;

  AlignedVector<LA::AlignedVector3f> m_t12s;
  std::vector<Depth::Measurement> m_zds;

  std::vector<ubyte> m_marksTmp1, m_marksTmp2, m_marksTmp3;
  std::vector<int> m_idxsTmp1, m_idxsTmp2, m_idxsTmp3;
  std::vector<std::vector<int> > m_idxsListTmp;
  std::vector<std::vector<FRM::Measurement>::iterator> m_iKF2Z;
  std::vector<FTR::Source> m_xsTmp;
  std::vector<FTR::Measurement> m_zsTmp;
  std::vector<std::vector<FTR::Measurement> > m_zsListTmp;
  std::vector<FTR::Measurement::Match> m_izmsTmp;
  AlignedVector<LA::ProductVector6f> m_adczsTmp;
  CameraPrior::Matrix::CC m_ApTmp;
  CameraPrior::Vector::C m_bpTmp;
  AlignedVector<float> m_work;

  // Callback function that will be triggered after GBA::run() finishes
  IBA::Solver::IbaCallback m_callback;

#ifdef CFG_DEBUG_EIGEN
 protected:
  class Track {
   public:
    class Measurement {
     public:
      inline Measurement() {}
      inline Measurement(const int iKF, const int iz) : m_iKF(iKF), m_iz(iz) {}
      inline bool operator < (const Measurement &z) const { return m_iKF < z.m_iKF; }
     public:
      int m_iKF, m_iz;
      FTR::EigenFactor::DC m_adcz;
    };
   public:
    inline void Initialize() { m_zs.resize(0); }
   public:
    std::vector<Measurement> m_zs;
    FTR::EigenFactor::DDC m_Sadx;
  };
 protected:
  virtual void DebugGenerateTracks();
  virtual void DebugUpdateFactors();
  virtual void DebugUpdateFactorsFeature();
  virtual void DebugUpdateFactorsPriorCameraPose();
  virtual void DebugUpdateFactorsPriorCameraMotion();
  virtual void DebugUpdateFactorsPriorDepth();
  virtual void DebugUpdateFactorsIMU();
  virtual void DebugUpdateFactorsFixOrigin();
  virtual void DebugUpdateFactorsFixPositionZ();
  virtual void DebugUpdateFactorsFixMotion();
  virtual void DebugUpdateSchurComplement();
  virtual void DebugSolveSchurComplement();
  virtual void DebugSolveBackSubstitution();
  virtual void DebugSolveGradientDescent();
  virtual void DebugComputeReduction();
  virtual void DebugComputeReductionFeature();
  virtual void DebugComputeReductionPriorCameraPose();
  virtual void DebugComputeReductionPriorCameraMotion();
  virtual void DebugComputeReductionPriorDepth();
  virtual void DebugComputeReductionIMU();
  virtual void DebugComputeReductionFixOrigin();
  virtual void DebugComputeReductionFixPositionZ();
  virtual void DebugComputeReductionFixMotion();
 protected:
  std::vector<std::vector<Track> > e_Xs;
  std::vector<std::vector<ubyte> > e_M;
  std::vector<std::vector<int> > e_I;
  std::vector<EigenMatrix6x6f> e_SAccs, e_SMccs;
  std::vector<EigenVector6f> e_Sbcs, e_Smcs, e_xcs, e_gcs, e_Agcs;
  LA::AlignedVectorXf e_xds, e_gds, e_Agds;
  std::vector<Camera::EigenFactor> e_SAcmsLM;
  std::vector<EigenVector9f> e_Sbms, e_xms, e_gms, e_Agms;
  float e_dFp;
#endif
};

#endif
