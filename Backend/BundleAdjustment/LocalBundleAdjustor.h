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
#ifndef _LOCAL_BUNDLE_ADJUSTOR_H_
#define _LOCAL_BUNDLE_ADJUSTOR_H_

#include "IBA.h"
#include "LocalMap.h"
#include "GlobalMap.h"
#include "CameraPrior.h"
#include "Candidate.h"
#include "Timer.h"

#define LBA_FLAG_FRAME_DEFAULT                      0
#define LBA_FLAG_FRAME_PUSH_TRACK                   1
#define LBA_FLAG_FRAME_POP_TRACK                    2
#define LBA_FLAG_FRAME_UPDATE_CAMERA                4
#define LBA_FLAG_FRAME_UPDATE_DEPTH                 8
#define LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION     16
#define LBA_FLAG_FRAME_UPDATE_TRACK_INFORMATION_KF  32
#define LBA_FLAG_FRAME_UPDATE_DELTA                 64
#define LBA_FLAG_FRAME_UPDATE_BACK_SUBSTITUTION     128

#define LBA_FLAG_TRACK_DEFAULT                  0
#define LBA_FLAG_TRACK_PUSH                     1
#define LBA_FLAG_TRACK_POP                      2
//#define LBA_FLAG_TRACK_MEASURE_KF               4
#define LBA_FLAG_TRACK_UPDATE_DEPTH             8
#define LBA_FLAG_TRACK_UPDATE_INFORMATION       16
#define LBA_FLAG_TRACK_UPDATE_INFORMATION_KF    32
#define LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO  64
#define LBA_FLAG_TRACK_UPDATE_BACK_SUBSTITUTION 128

#define LBA_FLAG_MARGINALIZATION_DEFAULT  0
#define LBA_FLAG_MARGINALIZATION_UPDATE   1
#define LBA_FLAG_MARGINALIZATION_NON_ZERO 2

#define LBA_FLAG_CAMERA_MOTION_DEFAULT                  0
#define LBA_FLAG_CAMERA_MOTION_UPDATE_ROTATION          1
#define LBA_FLAG_CAMERA_MOTION_UPDATE_POSITION          2
#define LBA_FLAG_CAMERA_MOTION_UPDATE_VELOCITY          4
#define LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_ACCELERATION 8
#define LBA_FLAG_CAMERA_MOTION_UPDATE_BIAS_GYROSCOPE    16

#define LBA_ME_FUNCTION   ME::FUNCTION_HUBER
//#define LBA_ME_FUNCTION ME::FUNCTION_NONE
class LocalBundleAdjustor : public MT::Thread {

 public:

  class InputLocalFrame : public FRM::Frame {
   public:
    inline InputLocalFrame() : FRM::Frame() {}
    inline InputLocalFrame(const InputLocalFrame &ILF) { *this = ILF; }
    inline InputLocalFrame(const FRM::Frame &LF, const AlignedVector<IMU::Measurement> &us,
                           const Camera &C) : FRM::Frame(LF) {
      m_us.Set(us);
      m_C = C;
    }
    inline void operator = (const InputLocalFrame &ILF) {
      *((FRM::Frame *) this) = ILF;
      m_us.Set(ILF.m_us);
      m_C = ILF.m_C;
    }
    inline void SaveB(FILE *fp) const {
      FRM::Frame::SaveB(fp);
      m_us.SaveB(fp);
      UT::SaveB(m_C, fp);
    }
    inline void LoadB(FILE *fp) {
      FRM::Frame::LoadB(fp);
      m_us.LoadB(fp);
      UT::LoadB(m_C, fp);
    }

   public:

    AlignedVector<IMU::Measurement> m_us;
    Camera m_C;
  };

 public:

  virtual void Initialize(IBA::Solver *solver, const int serial = 0, const int verbose = 0,
                          const int debug = 0, const int history = 0);
  virtual void Reset();
  virtual void PushCurrentFrame(const InputLocalFrame &ILF);
  virtual void PushCurrentFrame(const InputLocalFrame &ILF, const GlobalMap::InputKeyFrame &IKF);
  virtual void DeleteKeyFrame(const int iKF);
  virtual void GetCamera(FRM::Tag &T, Camera &C);
  virtual void Run();
  //virtual int GetTotalPoints(int *N = NULL);
  virtual float GetTotalTime(int *N = NULL);
  virtual bool SaveTimes(const std::string fileName);
  virtual bool SaveCameras(const std::string fileName, const bool poseOnly = true);
  virtual bool SaveCosts(const std::string fileName, const int type = 0);
  virtual bool SaveResiduals(const std::string fileName, const int type = 0);
  virtual bool SavePriors(const std::string fileName, const int type = 0);
  virtual void ComputeErrorFeature(float *ex);
  virtual void ComputeErrorFeature(const FRM::Frame *F, const Rigid3D &C,
                                   const AlignedVector<Rigid3D> &CsKF,
                                   const std::vector<Depth::InverseGaussian> &ds,
                                   float *ex, const int iKF = -1);
  virtual void ComputeErrorIMU(float *er, float *ep, float *ev, float *eba, float *ebw);
  virtual void ComputeErrorDrift(float *er, float *ep);
  virtual float GetTotalDistance();

  virtual void SaveB(FILE *fp);
  virtual void LoadB(FILE *fp);

  virtual void SetCallback(const IBA::Solver::IbaCallback& callback_iba);

 public:

  class CameraLF {
   public:
    Camera m_C;
    FRM::Tag m_T;
  };
  
  class MeasurementMatchLF : public FRM::MeasurementMatch {
   public:
    class Index {
      public:
      inline Index() {}
      inline Index(const int ik, const int iKF) : m_ik(ik), m_iKF(iKF) {}
      inline bool operator < (const int ik) const { return m_ik < ik; }
     public:
      int m_ik, m_iKF;
    };
   public:
    inline void operator = (const MeasurementMatchLF &Zm) {
      *((FRM::MeasurementMatch *) this) = Zm;
      m_Is = Zm.m_Is;
      m_iI2zm = Zm.m_iI2zm;
      m_ms = Zm.m_ms;
      m_SmddsST.Set(Zm.m_SmddsST);
    }
    inline void Initialize() {
      FRM::MeasurementMatch::Initialize();
      m_Is.resize(0);
      m_iI2zm.assign(1, 0);
      m_ms.resize(0);
      m_SmddsST.Resize(0);
    }
    inline void DeleteKeyFrame(const int iKF, const bool Z) {
      if (Z) {
        int iI, jI;
        const int NI1 = static_cast<int>(m_Is.size());
        for (iI = 0; iI < NI1; ++iI) {
          Index &I = m_Is[iI];
          if (I.m_iKF == iKF) {
            DeleteFeatureMeasurementMatches(iI, m_iI2zm[iI], m_iI2zm[iI + 1]);
          }
        }
        for (iI = jI = 0; iI < NI1; ++iI) {
          Index &I = m_Is[iI];
          if (I.m_iKF == iKF) {
            continue;
          } else if (I.m_iKF > iKF) {
            --I.m_iKF;
          }
          m_Is[jI] = I;
          m_iI2zm[++jI] = m_iI2zm[iI + 1];
        }
        const int NI2 = jI;
        m_Is.resize(NI2);
        m_iI2zm.resize(NI2 + 1);
      } else {
        const int NI = static_cast<int>(m_Is.size());
        for (int iI = 0; iI < NI; ++iI) {
          Index &I = m_Is[iI];
          if (I.m_iKF > iKF) {
            --I.m_iKF;
          }
        }
      }
    }
    inline void DeleteFeatureMeasurementMatches(const int iI, const int i1, const int i2) {
      const int Nzm = i2 - i1;
      if (Nzm == 0) {
        return;
      }
      const Index &I = m_Is[iI];
      const int NI = static_cast<int>(m_Is.size());
      for (int jI = iI + 1; jI <= NI; ++jI) {
        m_iI2zm[jI] -= Nzm;
      }
      for (int i = i1; i < i2; ++i) {
        m_ms[i] &= LBA_FLAG_MARGINALIZATION_NON_ZERO;
      }
      MeasurementMatch::DeleteFeatureMeasurementMatches(I.m_ik, i1, i2, m_ms.data());
      m_ms.erase(m_ms.begin() + i1, m_ms.begin() + i2);
      m_SmddsST.Erase(i1, Nzm);
    }
    inline void MakeZero() { FRM::MeasurementMatch::MakeZero(); m_SmddsST.MakeZero(); }
    inline void PushFeatureMeasurementMatches(const std::vector<FTR::Measurement::Match> &izms,
                                              const int iKF, ubyte *firstKF = NULL) {
      const int Nzm1 = static_cast<int>(m_izms.size());
      const int Nzm2 = Nzm1 + static_cast<int>(izms.size());
      FRM::MeasurementMatch::PushFeatureMeasurementMatches(izms, firstKF);
      const int ik = static_cast<int>(m_ik2zm.size()) - 2;
      m_Is.push_back(Index(ik, iKF));
      m_iI2zm.push_back(Nzm2);
      //m_ms.resize(Nzm2, LBA_FLAG_MARGINALIZATION_ZERO);
      m_ms.resize(Nzm2, LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_SmddsST.Resize(Nzm2, true);
      m_SmddsST.MakeZero(Nzm1, Nzm2);
    }
    inline void SaveB(FILE *fp) const {
      FRM::MeasurementMatch::SaveB(fp);
      UT::VectorSaveB(m_Is, fp);
      UT::VectorSaveB(m_iI2zm, fp);
      UT::VectorSaveB(m_ms, fp);
      m_SmddsST.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
      FRM::MeasurementMatch::LoadB(fp);
      UT::VectorLoadB(m_Is, fp);
      UT::VectorLoadB(m_iI2zm, fp);
      UT::VectorLoadB(m_ms, fp);
      m_SmddsST.LoadB(fp);
    }
    inline void AssertConsistency(const int Nk) const {
      FRM::MeasurementMatch::AssertConsistency(Nk);
      const int NI = static_cast<int>(m_Is.size());
      UT_ASSERT(static_cast<int>(m_iI2zm.size()) == NI + 1);
      int iI = 0;
      while (iI < NI) {
        const int ik = m_Is[iI].m_ik;
        for (++iI; iI < NI && m_Is[iI].m_ik == ik; ++iI)
          UT_ASSERT(m_Is[iI - 1].m_iKF < m_Is[iI].m_iKF);
        if (iI == NI)
          break;
        //UT_ASSERT(m_Is[iI].m_ik == ik + 1);
        UT_ASSERT(m_Is[iI].m_ik >= ik + 1);
      }
      const int Nzm = static_cast<int>(m_izms.size());
      UT_ASSERT(static_cast<int>(m_ms.size()) == Nzm && m_SmddsST.Size() == Nzm);
    }
    inline void AssertConsistency(const int ik, const FRM::Frame &F1, const FRM::Frame &F2,
                                  std::vector<FTR::Measurement::Match> &izmsTmp) const {
      FRM::MeasurementMatch::AssertConsistency(ik, F1, F2, izmsTmp);
      const int NI = static_cast<int>(m_Is.size());
      for (int iI = static_cast<int>(std::lower_bound(m_Is.begin(), m_Is.end(), ik) -
           m_Is.begin()); iI < NI && m_Is[iI].m_ik == ik; ++iI) {
        FRM::Frame::SearchFeatureMeasurementMatches(F1, F2, izmsTmp, m_Is[iI].m_iKF);
        const int i1 = m_iI2zm[iI], i2 = m_iI2zm[iI + 1], Nzm = i2 - i1;
        UT_ASSERT(Nzm == static_cast<int>(izmsTmp.size()));
        const FTR::Measurement::Match *izms = m_izms.data() + i1;
        for (int i = 0; i < Nzm; ++i) {
          const FTR::Measurement::Match &izm = izms[i];
          UT_ASSERT(izm == izmsTmp[i]);
          UT_ASSERT(F1.m_zs[izm.m_iz1].m_ix == F2.m_zs[izm.m_iz2].m_ix);
        }
      }
    }
   public:
    std::vector<Index> m_Is;
    std::vector<int> m_iI2zm;
    std::vector<ubyte> m_ms;
    AlignedVector<float> m_SmddsST;
  };

  class LocalFrame : public FRM::Frame {
   public:
    class SlidingTrack {
    public:
      inline bool operator == (const SlidingTrack &ST) const {
        return m_ist1 == ST.m_ist1 && m_ist2 == ST.m_ist2;
      }
      inline bool Valid() const { return m_ist1 != -1; }
      inline bool Invalid() const { return m_ist1 == -1; }
      inline void Invalidate() { m_ist1 = -1; }
      inline void Set(const int ist1, const int ist2) { m_ist1 = ist1; m_ist2 = ist2; }
      inline int Count() const { return m_ist2 - m_ist1; }
      inline void Step() { --m_ist1; --m_ist2; }
     public:
      int m_ist1, m_ist2;
    };
   public:
    inline LocalFrame() {}
    inline LocalFrame(const LocalFrame &LF) { *this = LF; }
    inline void operator = (const LocalFrame &LF) {
      *((FRM::Frame *) this) = LF;
      m_us.Set(LF.m_us);
      m_STs = LF.m_STs;
      m_Nsts = LF.m_Nsts;
      m_ms = LF.m_ms;
      m_Lzs.Set(LF.m_Lzs);
      m_Azs1.Set(LF.m_Azs1);
      m_Azs2.Set(LF.m_Azs2);
      m_Mzs1.Set(LF.m_Mzs1);
      m_Mzs2.Set(LF.m_Mzs2);
      m_AzsST.Set(LF.m_AzsST);
      m_SmddsST.Set(LF.m_SmddsST);
      m_iLFsMatch = LF.m_iLFsMatch;
      m_Zm = LF.m_Zm;
    }
    inline void Initialize(const InputLocalFrame &ILF) {
      FRM::Frame::Initialize(ILF);
      SortFeatureMeasurements();
      const int Nz = static_cast<int>(m_zs.size()), NZ = static_cast<int>(m_Zs.size());
      const int Nk = static_cast<int>(m_iKFsMatch.size());
      m_us.Set(ILF.m_us);
      m_STs.resize(Nz);
      m_Nsts.assign(Nz, 0);
      //m_ms.assign(Nz, LBA_FLAG_MARGINALIZATION_ZERO);
      m_ms.assign(Nz, LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_Lzs.Resize(Nz);     m_Lzs.MakeZero();
      m_Azs1.Resize(Nz);    m_Azs1.MakeZero();
      m_Azs2.Resize(Nz);    m_Azs2.MakeZero();
      m_Mzs1.Resize(Nz);    m_Mzs1.MakeZero();
      m_Mzs2.Resize(Nz);    m_Mzs2.MakeZero();
      m_AzsST.Resize(Nz);   m_AzsST.MakeZero();
      m_SmddsST.Resize(Nz); m_SmddsST.MakeZero();
      m_iLFsMatch.resize(0);
      m_Zm.Initialize();
#ifdef CFG_DEBUG
      for (int iz = 0; iz < Nz; ++iz) {
        m_STs[iz].Invalidate();
        m_Azs1[iz].m_adcz.Invalidate();
        m_Mzs1[iz].m_adcz.Invalidate();
        m_AzsST[iz].m_adc.Invalidate();
      }
#endif
    }
    inline void DeleteKeyFrame(const int iKF,
                               const std::vector<FRM::Measurement>::iterator *iZ = NULL) {
      const std::vector<FRM::Measurement>::iterator _iZ = iZ ? *iZ : std::lower_bound(m_Zs.begin(),
                                                                                      m_Zs.end(), iKF);
      if (_iZ != m_Zs.end()) {
        const bool Z = _iZ->m_iKF == iKF;
        if (Z) {
          m_STs.erase(m_STs.begin() + _iZ->m_iz1, m_STs.begin() + _iZ->m_iz2);
          m_Nsts.erase(m_Nsts.begin() + _iZ->m_iz1, m_Nsts.begin() + _iZ->m_iz2);
          m_ms.erase(m_ms.begin() + _iZ->m_iz1, m_ms.begin() + _iZ->m_iz2);
          const int Nz = _iZ->CountFeatureMeasurements();
          m_Lzs.Erase(_iZ->m_iz1, Nz);
          m_Azs1.Erase(_iZ->m_iz1, Nz);
          m_Azs2.Erase(_iZ->m_iz1, Nz);
          m_Mzs1.Erase(_iZ->m_iz1, Nz);
          m_Mzs2.Erase(_iZ->m_iz1, Nz);
          m_AzsST.Erase(_iZ->m_iz1, Nz);
          m_SmddsST.Erase(_iZ->m_iz1, Nz);
        }
        m_Zm.DeleteKeyFrame(iKF, Z);
      }
      FRM::Frame::DeleteKeyFrame(iKF, &_iZ);
    }
    inline void MakeZero() {
      //m_ms.assign(m_ms.size(), LBA_FLAG_MARGINALIZATION_ZERO);
      m_ms.assign(m_ms.size(), LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_Lzs.MakeZero();
      m_Azs1.MakeZero();
      m_Azs2.MakeZero();
      m_Mzs1.MakeZero();
      m_Mzs2.MakeZero();
      m_AzsST.MakeZero();
      m_SmddsST.MakeZero();
      m_Zm.MakeZero();
#ifdef CFG_DEBUG
      const int Nz = static_cast<int>(m_zs.size());
      for (int iz = 0; iz < Nz; ++iz) {
        m_Azs1[iz].m_adcz.Invalidate();
        m_Mzs1[iz].m_adcz.Invalidate();
        m_AzsST[iz].m_adc.Invalidate();
      }
#endif
    }
    inline void PushFrameMeasurement(const int iKF, const int Nz) {
      FRM::Frame::PushFrameMeasurement(iKF, Nz);
      const int Nz1 = static_cast<int>(m_STs.size()), Nz2 = Nz1 + Nz;
      m_STs.resize(Nz2);
      m_Nsts.resize(Nz2, 0);
      //m_ms.resize(Nz2, LBA_FLAG_MARGINALIZATION_ZERO);
      m_ms.resize(Nz2, LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_Lzs.Resize(Nz2, true);      m_Lzs.MakeZero(Nz1, Nz2);
      m_Azs1.Resize(Nz2, true);     m_Azs1.MakeZero(Nz1, Nz2);
      m_Azs2.Resize(Nz2, true);     m_Azs2.MakeZero(Nz1, Nz2);
      m_Mzs1.Resize(Nz2, true);     m_Mzs1.MakeZero(Nz1, Nz2);
      m_Mzs2.Resize(Nz2, true);     m_Mzs2.MakeZero(Nz1, Nz2);
      m_AzsST.Resize(Nz2, true);    m_AzsST.MakeZero(Nz1, Nz2);
      m_SmddsST.Resize(Nz2, true);  m_SmddsST.MakeZero(Nz1, Nz2);
#ifdef CFG_DEBUG
      for (int iz = Nz1; iz < Nz2; ++iz) {
        m_STs[iz].Invalidate();
        m_Azs1[iz].m_adcz.Invalidate();
        m_Mzs1[iz].m_adcz.Invalidate();
        m_AzsST[iz].m_adc.Invalidate();
      }
#endif
    }
    inline void PopFrameMeasurement() {
      FRM::Frame::PopFrameMeasurement();
      const int Nz = static_cast<int>(m_zs.size());
#ifdef CFG_DEBUG
      UT_ASSERT(Nz < static_cast<int>(m_STs.size()));
#endif
      m_STs.resize(Nz);
      m_Nsts.resize(Nz);
      m_ms.resize(Nz);
      m_Lzs.Resize(Nz);
      m_Azs1.Resize(Nz);
      m_Azs2.Resize(Nz);
      m_Mzs1.Resize(Nz);
      m_Mzs2.Resize(Nz);
      m_AzsST.Resize(Nz);
      m_SmddsST.Resize(Nz);
    }
    inline void SaveB(FILE *fp) const {
      FRM::Frame::SaveB(fp);
      m_us.SaveB(fp);
      UT::VectorSaveB(m_STs, fp);
      UT::VectorSaveB(m_Nsts, fp);
      UT::VectorSaveB(m_ms, fp);
      m_Lzs.SaveB(fp);
      m_Azs1.SaveB(fp);
      m_Azs2.SaveB(fp);
      m_Mzs1.SaveB(fp);
      m_Mzs2.SaveB(fp);
      m_AzsST.SaveB(fp);
      m_SmddsST.SaveB(fp);
      UT::VectorSaveB(m_iLFsMatch, fp);
      m_Zm.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
      FRM::Frame::LoadB(fp);
      m_us.LoadB(fp);
      UT::VectorLoadB(m_STs, fp);
      UT::VectorLoadB(m_Nsts, fp);
      UT::VectorLoadB(m_ms, fp);
      m_Lzs.LoadB(fp);
      m_Azs1.LoadB(fp);
      m_Azs2.LoadB(fp);
      m_Mzs1.LoadB(fp);
      m_Mzs2.LoadB(fp);
      m_AzsST.LoadB(fp);
      m_SmddsST.LoadB(fp);
      UT::VectorLoadB(m_iLFsMatch, fp);
      m_Zm.LoadB(fp);
    }
    inline void AssertConsistency() const {
      FRM::Frame::AssertConsistency();
      const int NZ = static_cast<int>(m_Zs.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const FRM::Measurement &Z = m_Zs[iZ];
        for (int iz = Z.m_iz1 + 1; iz < Z.m_iz2; ++iz)
          UT_ASSERT(m_zs[iz - 1] < m_zs[iz]);
      }
      const int Nz = static_cast<int>(m_zs.size());
      UT_ASSERT(static_cast<int>(m_STs.size()) == Nz);
      UT_ASSERT(static_cast<int>(m_Nsts.size()) == Nz);
      UT_ASSERT(static_cast<int>(m_ms.size()) == Nz);
      UT_ASSERT(m_Lzs.Size() == Nz);
      UT_ASSERT(m_Azs1.Size() == Nz && m_Azs2.Size() == Nz);
      UT_ASSERT(m_Mzs1.Size() == Nz && m_Mzs2.Size() == Nz);
      UT_ASSERT(m_AzsST.Size() == Nz && m_SmddsST.Size() == Nz);
      for (int iz = 0; iz < Nz; ++iz) {
        const int Nst = m_STs[iz].Count();
        UT_ASSERT(Nst > 0 && Nst == m_Nsts[iz]);
#ifdef CFG_STEREO
        const FTR::Factor::FixSource::L &L = m_Lzs[iz];
        UT_ASSERT(L.m_Je.Valid() || L.m_Jer.Valid());
#endif
        const FTR::Factor::FixSource::A3 Az(m_Azs2[iz].m_add, m_Azs1[iz].m_adcz);
        const FTR::Factor::FixSource::A3 AzST = Az * (1.0f / Nst);
        UT_ASSERT(AzST.AssertEqual(m_AzsST[iz], 1, "", -1.0f, -1.0f));
      }
      m_Zm.AssertConsistency(static_cast<int>(m_iLFsMatch.size()));
    }
   public:
    AlignedVector<IMU::Measurement> m_us;
    std::vector<SlidingTrack> m_STs;
    std::vector<int> m_Nsts;
    std::vector<ubyte> m_ms;
    AlignedVector<FTR::Factor::FixSource::L> m_Lzs;
    AlignedVector<FTR::Factor::FixSource::A1> m_Azs1;
    AlignedVector<FTR::Factor::FixSource::A2> m_Azs2;
    AlignedVector<FTR::Factor::FixSource::M1> m_Mzs1;
    AlignedVector<FTR::Factor::FixSource::M2> m_Mzs2;
    AlignedVector<FTR::Factor::FixSource::A3> m_AzsST;
    AlignedVector<FTR::Factor::DD> m_SmddsST;
    std::vector<int> m_iLFsMatch;
    MeasurementMatchLF m_Zm;
  };

  class KeyFrame : public GlobalMap::KeyFrameBA {
   public:
    class SlidingTrack {
     public:
      inline SlidingTrack() {}
      inline SlidingTrack(const int ic) : m_icMin(ic), m_icMax(ic) {}
      inline bool operator < (const SlidingTrack &ST) const { return m_icMin < ST.m_icMin && m_icMax < ST.m_icMax; }
      inline void Set(const int ic) { m_icMin = m_icMax = ic; }
      inline void Step() { --m_icMin; --m_icMax; }
      inline bool Valid() const { return m_icMin != -1; }
      inline bool Invalid() const { return m_icMin == -1; }
      inline void Invalidate() { m_icMin = -1; }
     public:
      int m_icMin, m_icMax;
    };
   public:
    inline KeyFrame() {}
    inline KeyFrame(const KeyFrame &KF) { *this = KF; }
    inline void operator = (const KeyFrame &KF) {
      *((GlobalMap::KeyFrameBA *) this) = KF;
      m_ix2ST = KF.m_ix2ST;
      m_STs = KF.m_STs;
      m_Nsts = KF.m_Nsts;
      m_ms = KF.m_ms;
      m_usST = KF.m_usST;
      m_Axps.Set(KF.m_Axps);
      m_AxpsST.Set(KF.m_AxpsST);
      m_Axs.Set(KF.m_Axs);
      m_AxsST.Set(KF.m_AxsST);
      m_Mxs.Set(KF.m_Mxs);
      m_MxsST.Set(KF.m_MxsST);
      m_Azs.Set(KF.m_Azs);
    }
    inline void Initialize(const FRM::Frame &F) {
      GlobalMap::KeyFrameBA::Initialize(F);
      m_ix2ST.assign(1, 0);
      m_STs.resize(0);
      m_Nsts.resize(0);
      m_ms.resize(0);
      m_usST.resize(0);
      m_Axps.Resize(0);
      m_AxpsST.Resize(0);
      m_Axs.Resize(0);
      m_AxsST.Resize(0);
      m_Mxs.Resize(0);
      m_MxsST.Resize(0);
      m_Azs.Resize(static_cast<int>(m_zs.size()));
      m_Azs.MakeZero();
    }
    inline void PushFeatures(const std::vector<FTR::Source> &xs) {
      GlobalMap::KeyFrameBA::PushFeatures(xs);
      const int ix = static_cast<int>(m_ms.size()), Nx = static_cast<int>(m_xs.size());
      m_ix2ST.resize(Nx + 1, static_cast<int>(m_STs.size()));
      m_Nsts.resize(Nx, 0);
      m_ms.resize(Nx, LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_Axps.Resize(Nx, true);    m_Axps.MakeZero(ix, Nx);
      m_AxpsST.Resize(Nx, true);  m_AxpsST.MakeZero(ix, Nx);
      m_Axs.Resize(Nx, true);     m_Axs.MakeZero(ix, Nx);
      m_Mxs.Resize(Nx, true);     m_Mxs.MakeZero(ix, Nx);
#ifdef CFG_DEBUG
      for (int jx = ix; jx < Nx; ++jx) {
        m_Mxs[jx].m_mdd.Invalidate();
      }
#endif
    }
    inline void PushFeatureMeasurement(const int iKF, const int ix, const FTR::Measurement &z,
                                       int *ik) {
      int iZ, iz;
      GlobalMap::KeyFrameBA::PushFeatureMeasurement(iKF, ix, z, &iZ, &iz);
      m_Azs.InsertZero(iz);
      *ik = m_Zs[iZ].m_ik;
    }
    inline void DeleteKeyFrame(const int iKF) {
      const std::vector<FRM::Measurement>::iterator iZ = std::lower_bound(m_Zs.begin(),
                                                                          m_Zs.end(), iKF);
      if (iZ != m_Zs.end() && iZ->m_iKF == iKF) {
        m_Azs.Erase(iZ->m_iz1, iZ->CountFeatureMeasurements());
      }
      GlobalMap::KeyFrameBA::DeleteKeyFrame(iKF, &iZ);
    }
    inline void MakeZero() {
      GlobalMap::KeyFrameBA::MakeZero();
      //m_ms.assign(m_ms.size(), LBA_FLAG_MARGINALIZATION_ZERO);
      m_ms.assign(m_ms.size(), LBA_FLAG_MARGINALIZATION_DEFAULT);
      m_usST.assign(m_usST.size(), LBA_FLAG_TRACK_UPDATE_INFORMATION_ZERO);
      m_Axps.MakeZero();  m_AxpsST.MakeZero();
      m_Axs.MakeZero();   m_AxsST.MakeZero();
      m_Mxs.MakeZero();   m_MxsST.MakeZero();
      m_Azs.MakeZero();
#ifdef CFG_DEBUG
      const int Nx = m_Mxs.Size();
      for (int ix = 0; ix < Nx; ++ix) {
        m_Mxs[ix].m_mdd.Invalidate();
      }
      const int NST = m_MxsST.Size();
      for (int iST = 0; iST < NST; ++iST) {
        m_MxsST[iST].m_mdd.Invalidate();
      }
#endif
    }
    inline int CountSlidingTracks(const int ix) const { return m_ix2ST[ix + 1] - m_ix2ST[ix]; }
    inline void SaveB(FILE *fp) const {
      GlobalMap::KeyFrameBA::SaveB(fp);
      UT::VectorSaveB(m_ix2ST, fp);
      UT::VectorSaveB(m_STs, fp);
      UT::VectorSaveB(m_Nsts, fp);
      UT::VectorSaveB(m_ms, fp);
      UT::VectorSaveB(m_usST, fp);
      m_Axps.SaveB(fp);   m_AxpsST.SaveB(fp);
      m_Axs.SaveB(fp);    m_AxsST.SaveB(fp);
      m_Mxs.SaveB(fp);    m_MxsST.SaveB(fp);
      m_Azs.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
      GlobalMap::KeyFrameBA::LoadB(fp);
      UT::VectorLoadB(m_ix2ST, fp);
      UT::VectorLoadB(m_STs, fp);
      UT::VectorLoadB(m_Nsts, fp);
      UT::VectorLoadB(m_ms, fp);
      UT::VectorLoadB(m_usST, fp);
      m_Axps.LoadB(fp);   m_AxpsST.LoadB(fp);
      m_Axs.LoadB(fp);    m_AxsST.LoadB(fp);
      m_Mxs.LoadB(fp);    m_MxsST.LoadB(fp);
      m_Azs.LoadB(fp);
    }
    inline void AssertConsistency(const int iKF) const {
      GlobalMap::KeyFrameBA::AssertConsistency(iKF);
      const int Nx = static_cast<int>(m_xs.size()), NST = static_cast<int>(m_STs.size());
      UT_ASSERT(static_cast<int>(m_ix2ST.size()) == Nx + 1);
      UT_ASSERT(m_ix2ST[0] == 0 && m_ix2ST[Nx] == NST);
      UT_ASSERT(static_cast<int>(m_Nsts.size()) == Nx && static_cast<int>(m_ms.size()) == Nx);
      UT_ASSERT(static_cast<int>(m_usST.size()) == NST);
      UT_ASSERT(m_Axps.Size() == Nx && m_AxpsST.Size() == Nx);
      UT_ASSERT(m_Axs.Size() == Nx && m_AxsST.Size() == NST);
      UT_ASSERT(m_Mxs.Size() == Nx && m_MxsST.Size() == NST);
      for (int ix = 0; ix < Nx; ++ix) {
        const int iST1 = m_ix2ST[ix], iST2 = m_ix2ST[ix + 1];
        UT_ASSERT(iST1 <= iST2);
        for (int iST = iST1; iST < iST2; ++iST) {
          if (iST > iST1)
            UT_ASSERT(m_STs[iST - 1] < m_STs[iST]);
        }
        const int Nst = m_Nsts[ix];
        UT_ASSERT(Nst == iST2 - iST1);
        if (Nst == 0) {
          UT_ASSERT(m_Axps[ix] == m_AxpsST[ix]);
          UT_ASSERT(m_Axs[ix] == m_Axps[ix]);
        } else {
          const FTR::Factor::FixSource::Source::A AxpST = m_Axps[ix] * (1.0f / Nst);
          UT_ASSERT(AxpST == m_AxpsST[ix]);
        }
      }
      UT_ASSERT(m_Azs.Size() == static_cast<int>(m_zs.size()));
    }
   public:
    std::vector<int> m_ix2ST;
    std::vector<SlidingTrack> m_STs;
    std::vector<int> m_Nsts;
    std::vector<ubyte> m_ms, m_usST;
    AlignedVector<FTR::Factor::FixSource::Source::A> m_Axps, m_AxpsST;
    AlignedVector<FTR::Factor::FixSource::Source::A> m_Axs, m_AxsST;
    AlignedVector<FTR::Factor::FixSource::Source::M> m_Mxs, m_MxsST;
    AlignedVector<FTR::Factor::Depth> m_Azs;
  };

  enum TimerType { TM_TOTAL, TM_FACTOR, TM_SCHUR_COMPLEMENT, TM_CAMERA, TM_DEPTH, TM_UPDATE,
                   TM_TEST_1, TM_TEST_2, TM_TEST_3, TM_TEST_4, TM_TEST_5, TM_TYPES };
  Timer m_ts[TM_TYPES];
  class ES {
   public:
    inline float Total() const {
      return m_ESx.Total() + m_ESm.Total() + m_ESd.Total() + m_ESo.Total() +
             m_ESfp.Total() + m_ESfm.Total();
    }
    inline void Save(FILE *fp) const {
      fprintf(fp, "%e %e %e %e %e\n", Total(), m_ESx.Total(), m_ESm.Total(),
                                               m_ESd.Total(), m_ESo.Total());
    }
   public:      
    FTR::ES m_ESx;
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
  class PriorResidual {
   public:
    inline void Save(FILE *fp) const {
      fprintf(fp, "%e %e %e %e %e\n", m_r2, m_r2c, m_r2m, m_r2xr, m_r2xk);
    }
   public:
    float m_r2, m_r2c, m_r2m, m_r2xr, m_r2xk;
  };
  class PriorError {
   public:
    inline void Save(FILE *fp) const {
      fprintf(fp, "%e %e %e %e %e\n", m_ep, m_er, m_ev, m_eba, m_ebw);
    }
   public:
    float m_ep, m_er, m_ev, m_eba, m_ebw;
  };
  class History {
   public:
    inline void MakeZero() { memset(this, 0, sizeof(History)); }
   public:
    // m_history >= 1
    Camera m_C;
    float m_t;
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
    //m_history >= 2
    PriorResidual m_PR;
#ifdef CFG_GROUND_TRUTH
    PriorResidual m_PRGT;
#endif
    PriorError m_PE;
#ifdef CFG_GROUND_TRUTH
    PriorError m_PEGT;
#endif
  };

 protected:

  virtual void SynchronizeData();
  virtual void UpdateData();
  virtual bool BufferDataEmpty();

  virtual void MarginalizeLocalFrame();
  virtual void PushLocalFrame(const InputLocalFrame &ILF);
  virtual void PopLocalFrame();
  virtual void PushKeyFrame(const GlobalMap::InputKeyFrame &IKF);
  virtual void SearchMatchingKeyFrames(FRM::Frame &F);
  virtual void PushFeatureMeasurementMatchesFirst(const FRM::Frame &F, std::vector<int> &iKF2X,
                                                  std::vector<int> &iX2z);
  virtual void PushFeatureMeasurementMatchesNext(const FRM::Frame &F1, const FRM::Frame &F2,
                                                 const std::vector<int> &iKF2X, const std::vector<int> &iX2z2,
                                                 MeasurementMatchLF &Zm);
  virtual void MarkFeatureMeasurements(const LocalFrame &LF, const int iKF, std::vector<int> &ix2z);
  virtual void MarkFeatureMeasurementsUpdateDepth(const FRM::Frame &F, std::vector<ubyte> &ucsKF,
                                                  std::vector<ubyte> &uds);
#ifdef CFG_DEBUG
  virtual void DebugSetFeatureMeasurements(const Rigid3D &C, const AlignedVector<Rigid3D> &CsKF,
                                           const std::vector<Depth::InverseGaussian> &ds,
                                           FRM::Frame &F);
#endif
  virtual int CountMeasurementsFrameLF();
  virtual int CountMeasurementsFrameKF();
  virtual int CountMeasurementsFeatureLF();
  virtual int CountMeasurementsFeatureKF();
  virtual int CountLocalTracks();
  virtual int CountSlidingTracks();
  virtual int CountSchurComplements();
  virtual int CountSchurComplementsOffDiagonal();
  virtual float ComputeImageMotion(const float z1, const Rigid3D &C1, const Rigid3D &C2,
                                   ubyte *first = NULL);

  virtual void UpdateFactors();
  virtual void UpdateFactorsFeaturePriorDepth();
  virtual void UpdateFactorsFeatureLF();
  virtual void UpdateFactorsFeatureKF();
  virtual void UpdateFactorsPriorDepth();
  virtual void UpdateFactorsPriorCameraMotion();
  virtual void UpdateFactorsIMU();
  virtual void UpdateFactorsFixOrigin();
  virtual void UpdateFactorsFixPositionZ();
  virtual void UpdateFactorsFixMotion();
  virtual void UpdateSchurComplement();
  virtual bool SolveSchurComplement();
  virtual bool SolveSchurComplementPCG();
#ifdef CFG_GROUND_TRUTH
  virtual void SolveSchurComplementGT(const AlignedVector<Camera> &CsLF,
                                      LA::AlignedVectorXf *xs,
                                      const bool motion = true);
#endif
  virtual bool SolveSchurComplementLast();
  virtual void PrepareConditioner();
  virtual void ApplyM(const LA::AlignedVectorXf &xs, LA::AlignedVectorXf *Mxs);
  virtual void ApplyA(const LA::AlignedVectorXf &xs, LA::AlignedVectorXf *Axs);
  virtual void ApplyAcm(const LA::ProductVector6f *xcs, const LA::Vector9f *xms,
                        LA::Vector6f *Axcs, LA::Vector9f *Axms, const bool Acc = true,
                        const LA::AlignedMatrix9x9f *Amus = NULL);
  virtual Residual ComputeResidual(const LA::AlignedVectorXf &xs, const bool minus = false);
  virtual void SolveBackSubstitution();
#ifdef CFG_GROUND_TRUTH
  virtual void SolveBackSubstitutionGT(const std::vector<Depth::InverseGaussian> &ds,
                                       LA::AlignedVectorXf *xs);
#endif
  virtual bool EmbeddedMotionIteration();
  virtual void EmbeddedPointIteration(const AlignedVector<Camera> &CsLF,
                                      const AlignedVector<Rigid3D> &CsKF,
                                      const std::vector<ubyte> &ucsKF,
                                      const std::vector<ubyte> &uds,
                                      std::vector<Depth::InverseGaussian> *ds);

  virtual void SolveDogLeg();
  virtual void SolveGradientDescent();
  virtual void ComputeReduction();
  virtual void ComputeReductionFeaturePriorDepth();
  virtual void ComputeReductionFeatureLF();
  virtual void ComputeReductionFeatureKF();
  virtual void ComputeReductionPriorDepth();
  virtual void ComputeReductionPriorCameraMotion();
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
  virtual void ConvertCameraUpdates(const AlignedVector<LA::AlignedVector6f> &xcsA,
                                    LA::Vector6f *xcs);
  virtual void ConvertMotionUpdates(const float *xms, LA::AlignedVectorXf *xv2s,
                                    LA::AlignedVectorXf *xba2s, LA::AlignedVectorXf *xbw2s);
  virtual void ConvertCameraMotionResiduals(const LA::AlignedVectorXf &rs,
                                            const LA::AlignedVectorXf &zs,
                                            float *Se2, float *e2Max);
  virtual void ConvertDepthUpdates(const float *xs, LA::AlignedVectorXf *xds);
  virtual void PushDepthUpdates(const LA::AlignedVectorXf &xds, LA::AlignedVectorXf *xs);

  virtual float PrintErrorStatistic(const std::string str, const AlignedVector<Camera> &CsLF,
                                    const AlignedVector<Rigid3D> &CsKF,
                                    const std::vector<Depth::InverseGaussian> &ds,
                                    const AlignedVector<IMU::Delta> &DsLF, const bool detail);
  virtual ES ComputeErrorStatistic(const AlignedVector<Camera> &CsLF, const AlignedVector<Rigid3D> &CsKF,
                                   const std::vector<Depth::InverseGaussian> &ds,
                                   const AlignedVector<IMU::Delta> &DsLF);
  virtual FTR::ES ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Camera> &CsLF,
                                                         const AlignedVector<Rigid3D> &CsKF,
                                                         const Depth::InverseGaussian *ds);
  virtual void AccumulateErrorStatisticFeature(const FRM::Frame *F, const Rigid3D &C,
                                               const AlignedVector<Rigid3D> &CsKF,
                                               const Depth::InverseGaussian *ds,
                                               FTR::ES *ES, const bool localFrm);
  virtual void AccumulateErrorStatisticPriorDepth(const Depth::InverseGaussian *ds, FTR::ES *ES);
  virtual CameraPrior::Motion::ES ComputeErrorStatisticPriorCameraMotion(const
                                                                         AlignedVector<Camera> &CsLF);
  virtual IMU::Delta::ES ComputeErrorStatisticIMU(const AlignedVector<Camera> &CsLF,
                                                  const AlignedVector<IMU::Delta> &DsLF);
  virtual Camera::Fix::Origin::ES ComputeErrorStatisticFixOrigin(const AlignedVector<Camera> &CsLF);
  virtual Camera::Fix::PositionZ::ES ComputeErrorStatisticFixPositionZ(const
                                                                       AlignedVector<Camera> &CsLF);
  virtual Camera::Fix::Motion::ES ComputeErrorStatisticFixMotion(const
                                                                 AlignedVector<Camera> &CsLF);
  virtual float PrintErrorStatistic(const std::string str, const AlignedVector<Camera> &CsLF,
                                    const AlignedVector<Rigid3D> &CsKF,
                                    const std::vector<Depth::InverseGaussian> &ds,
                                    const LA::AlignedVectorXf &xs, const bool detail);
  virtual ES ComputeErrorStatistic(const AlignedVector<Camera> &CsLF,
                                   const AlignedVector<Rigid3D> &CsKF,
                                   const std::vector<Depth::InverseGaussian> &ds,
                                   const LA::AlignedVectorXf &xs,
                                   const bool updateOnly = true);
  virtual FTR::ES ComputeErrorStatisticFeaturePriorDepth(const AlignedVector<Camera> &CsLF,
                                                         const AlignedVector<Rigid3D> &CsKF,
                                                         const Depth::InverseGaussian *ds,
                                                         const AlignedVector<LA::ProductVector6f> &xcs,
                                                         const LA::AlignedVectorXf &xds,
                                                         const bool updateOnly = true);
  virtual void AccumulateErrorStatisticFeatureLF(const AlignedVector<Camera> &CsLF,
                                                 const AlignedVector<Rigid3D> &CsKF,
                                                 const Depth::InverseGaussian *ds,
                                                 const AlignedVector<LA::ProductVector6f> &xcs,
                                                 const LA::AlignedVectorXf &xds, FTR::ES *ES,
                                                 const bool updateOnly = true);
  virtual void AccumulateErrorStatisticFeatureKF(const AlignedVector<Rigid3D> &CsKF,
                                                 const Depth::InverseGaussian *ds,
                                                 const LA::AlignedVectorXf &xds, FTR::ES *ES,
                                                 const bool updateOnly = true);
  virtual void AccumulateErrorStatisticPriorDepth(const Depth::InverseGaussian *ds,
                                                  const LA::AlignedVectorXf &xds, FTR::ES *ES,
                                                  const bool updateOnly = true);
  virtual CameraPrior::Motion::ES ComputeErrorStatisticPriorCameraMotion(const
                                                                         LA::ProductVector6f *xcs,
                                                                         const LA::Vector9f *xms,
                                                                         const bool updateOnly = true);
  virtual IMU::Delta::ES ComputeErrorStatisticIMU(const LA::ProductVector6f *xcs,
                                                  const LA::Vector9f *xms, const bool updateOnly = true);
  virtual Camera::Fix::Origin::ES ComputeErrorStatisticFixOrigin(const AlignedVector<Camera> &CsLF,
                                                                 const AlignedVector<LA::ProductVector6f> &xcs,
                                                                 const bool updateOnly = true);
  virtual Camera::Fix::PositionZ::ES ComputeErrorStatisticFixPositionZ(const AlignedVector<Camera>
                                                                       &CsLF, const LA::ProductVector6f *xcs,
                                                                       const bool updateOnly = true);
  virtual Camera::Fix::Motion::ES ComputeErrorStatisticFixMotion(const AlignedVector<Camera> &CsLF,
                                                                 const LA::Vector9f *xms,
                                                                 const bool updateOnly = true);
  virtual void AssertConsistency(const bool chkFlag = true, const bool chkSchur = true);
  virtual void AccumulateFactorFeatureDD(const FRM::Frame *F,
                                         AlignedVector<FTR::Factor::DD> *Sadds,
                                         const bool localFrm);

 protected:

  friend class GlobalBundleAdjustor;
  friend class ViewerIBA;

  IBA::Solver *m_solver;
  LocalMap *m_LM;
  GlobalMap *m_GM;
  class GlobalBundleAdjustor *m_GBA;
  Camera::Calibration m_K;
  int m_verbose, m_debug, m_history;
#ifdef CFG_GROUND_TRUTH
  const Camera *m_CsGT;
#endif
  std::string m_dir;

  std::list<InputLocalFrame> m_ILFs1, m_ILFs2;
  std::list<GlobalMap::InputKeyFrame> m_IKFs1, m_IKFs2;

  CameraLF m_C;
  boost::shared_mutex m_MTC;

  std::vector<History> m_hists;
  int m_iIter, m_iIterPCG, m_iIterDL;
  float m_delta2;

  Camera::Fix::Origin m_Zo;
  Camera::Fix::Origin::Factor m_Ao;

  std::vector<int> m_ic2LF;
  std::vector<LocalFrame> m_LFs;
  AlignedVector<Camera> m_CsLF;
#ifdef CFG_GROUND_TRUTH
  AlignedVector<Camera> m_CsLFGT;
#endif
  std::vector<ubyte> m_ucsLF, m_ucmsLF;
#ifdef CFG_INCREMENTAL_PCG
  AlignedVector<LA::Vector6f> m_xcsLF;
  AlignedVector<LA::Vector9f> m_xmsLF;
#endif
  AlignedVector<IMU::Delta> m_DsLF;
#ifdef CFG_GROUND_TRUTH
  AlignedVector<IMU::Delta> m_DsLFGT;
#endif
  AlignedVector<IMU::Delta::Factor> m_AdsLF;
  AlignedVector<Camera::Fix::PositionZ::Factor> m_AfpsLF;
  AlignedVector<Camera::Fix::Motion::Factor> m_AfmsLF;

  std::vector<KeyFrame> m_KFs;
  AlignedVector<Rigid3D> m_CsKF;
#ifdef CFG_GROUND_TRUTH
  AlignedVector<Rigid3D> m_CsKFGT;
#endif
  std::vector<ubyte> m_ucsKF;
#ifdef CFG_GROUND_TRUTH
  std::vector<ubyte> m_ucsKFGT;
#endif
  AlignedVector<IMU::Measurement> m_usKF;

  std::vector<int> m_iKF2d;
  std::vector<Depth::InverseGaussian> m_ds;
  std::vector<Depth::InverseGaussian> *m_dsGT;
  std::vector<ubyte> m_uds;
#ifdef CFG_GROUND_TRUTH
  std::vector<ubyte> m_udsGT;
#endif

  CameraPrior::Joint m_Zp, m_ZpBkp;
  CameraPrior::Motion m_ZpLF;
  CameraPrior::Pose m_ZpKF;
  CameraPrior::Motion::Factor m_ApLF;

  AlignedVector<Camera::Factor::Unitary::CC> m_SAcusLF, m_SMcusLF;
  AlignedVector<Camera::Factor> m_SAcmsLF;

  std::vector<ubyte> m_UcsLF, m_UcsKF, m_Uds;

  AlignedVector<LA::AlignedMatrix6x6f> m_Acus, m_Acbs, m_AcbTs;
  AlignedVector<LA::AlignedMatrix9x9f> m_Amus;
  AlignedVector<Camera::Conditioner::C> m_Mcs;
  AlignedVector<Camera::Conditioner::M> m_Mms;
  AlignedMatrixX<LA::AlignedMatrix6x6f> m_Mcc, m_MccT;
  AlignedMatrixX<LA::AlignedMatrix6x9f> m_Mcm, m_MmcT;
  AlignedMatrixX<LA::AlignedMatrix9x6f> m_Mmc, m_McmT;
  AlignedMatrixX<LA::AlignedMatrix9x9f> m_Mmm, m_MmmT;
  AlignedVector<LA::ProductVector6f> m_bcs;
  AlignedVector<LA::AlignedVector9f> m_bms;
  LA::AlignedVectorXf m_xs, m_bs, m_rs, m_ps, m_zs, m_drs;
  std::vector<int> m_ic2b;

  LA::AlignedVectorXf m_xp2s, m_xr2s, m_xv2s, m_xba2s, m_xbw2s, m_xds, m_x2s, m_axds;
  AlignedVector<LA::ProductVector6f> m_xcsP;
  AlignedVector<LA::AlignedVector6f> m_Axcs;
  std::vector<int> m_iKF2X;

  AlignedVector<Camera> m_CsLFBkp;
  AlignedVector<Rigid3D> m_CsKFBkp;
  AlignedVector<LA::Vector6f> m_xcsLFBkp;
  AlignedVector<LA::Vector9f> m_xmsLFBkp;
  std::vector<Depth::InverseGaussian> m_dsBkp;
  AlignedVector<IMU::Measurement> m_usBkp;

  LA::AlignedVectorXf m_xsGN, m_xsGD, m_xsDL, m_xsGT, m_dxs;
  AlignedVector<LA::ProductVector6f> m_gcs;
  LA::AlignedVectorXf m_gds, m_Agds, m_Ags;
  float m_x2GN, m_x2GD, m_x2DL, m_bl, m_gTAg, m_beta, m_dFa, m_dFp, m_rho;
  bool m_update, m_converge, m_empty;

  AlignedVector<Rotation3D> m_R12s;
  AlignedVector<LA::AlignedVector3f> m_t12s;
  std::vector<Depth::Measurement> m_zds;

  CandidateVector<int> m_idxsSortTmp;
  std::vector<ubyte> m_marksTmp;
  std::vector<int> m_cntsTmp, m_idxsTmp1, m_idxsTmp2, m_idxsTmp3, m_idxsTmp4;
  std::vector<std::vector<FRM::Measurement>::iterator> m_iLF2Z;
  std::vector<std::vector<int> > m_idxsListTmp;
  std::vector<FTR::Source> m_xsTmp;
  std::vector<FTR::Measurement::Match> m_izmsTmp;
  std::vector<KeyFrame::SlidingTrack> m_STsTmp;
  std::vector<int> m_ix2STTmp;
  std::vector<ubyte> m_usSTTmp;
  AlignedVector<FTR::Factor::FixSource::Source::A> m_AxsTmp;
  AlignedVector<FTR::Factor::FixSource::Source::M> m_MxsTmp;
  AlignedVector<float> m_work;

  // Callback function that will be triggered after LBA::run() finishes
  IBA::Solver::IbaCallback m_callback;

#ifdef CFG_DEBUG_EIGEN
 protected:
  class Track {
   public:
    class MeasurementLF {
     public:
      inline MeasurementLF() {}
      inline MeasurementLF(const int ic, const int iz) : m_ic(ic), m_iz(iz), m_Nst(0) {}
      inline bool operator < (const MeasurementLF &z) const { return m_ic < z.m_ic; }
      inline bool operator < (const int ic) const { return m_ic < ic; }
     public:
      int m_ic, m_iz, m_Nst;
      FTR::EigenFactor::DC m_adcz, m_adczST;
    };
    class MeasurementKF {
     public:
      inline MeasurementKF() {}
      inline MeasurementKF(const int iKF, const int iz) : m_iKF(iKF), m_iz(iz) {}
      inline bool operator < (const MeasurementKF &z) const { return m_iKF < z.m_iKF; }
     public:
      int m_iKF, m_iz;
    };
   public:
    inline void Initialize() { m_zsLF.resize(0); m_zsKF.resize(0); m_STsLF.resize(0); m_SaddsST.resize(0); }
   public:
    std::vector<MeasurementLF> m_zsLF;
    std::vector<MeasurementKF> m_zsKF;
    std::vector<std::vector<int> > m_STsLF;
    FTR::EigenFactor::DD m_Sadd;
    std::vector<FTR::EigenFactor::DD> m_SaddsST;
  };
 protected:
  virtual void DebugMarginalizeLocalFrame();
  virtual void DebugGenerateTracks();
  virtual void DebugUpdateFactors();
  virtual void DebugUpdateFactorsFeature();
  virtual void DebugUpdateFactorsPriorDepth();
  virtual void DebugUpdateFactorsPriorCameraMotion();
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
  virtual void DebugComputeReductionPriorDepth();
  virtual void DebugComputeReductionPriorCameraMotion();
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
  std::vector<Camera::EigenFactor> e_SAcms;
  std::vector<EigenVector9f> e_Sbms, e_xms, e_gms, e_Agms;
  LA::AlignedMatrixXf m_A, m_LT, m_A1, m_A2;
  LA::AlignedVectorXf m_b, m_x, m_x1, m_x2;
  EigenMatrixXd e_A, e_LT, e_LD;
  EigenVectorXd e_b, e_x, e_LTx;
  float e_dFp;
#endif

};

#endif
