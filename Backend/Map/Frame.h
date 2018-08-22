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
#ifndef _FRAME_H_
#define _FRAME_H_

#include "IMU.h"
#include "Feature.h"

namespace FRM {

template<class TYPE> inline void VectorSaveB(const std::vector<TYPE> &V, FILE *fp) {
  const int N = static_cast<int>(V.size());
  UT::SaveB(N, fp);
  for (int i = 0; i < N; ++i) {
    V[i].SaveB(fp);
  }
}
template<class TYPE> inline void VectorLoadB(std::vector<TYPE> &V, FILE *fp) {
  const int N = UT::LoadB<int>(fp);
  V.resize(N);
  for (int i = 0; i < N; ++i) {
    V[i].LoadB(fp);
  }
}

template<class TYPE> inline void VectorSaveB(const SIMD::vector<TYPE> &V, FILE *fp) {
  const int N = static_cast<int>(V.size());
  UT::SaveB(N, fp);
  for (int i = 0; i < N; ++i) {
    V[i].SaveB(fp);
  }
}
template<class TYPE> inline void VectorLoadB(SIMD::vector<TYPE> &V, FILE *fp) {
  const int N = UT::LoadB<int>(fp);
  V.resize(N);
  for (int i = 0; i < N; ++i) {
    V[i].LoadB(fp);
  }
}

template<class TYPE> inline void ListSaveB(const std::list<TYPE> &L, FILE *fp) {
  const int N = int(L.size());
  UT::SaveB(N, fp);
  for (typename std::list<TYPE>::const_iterator it = L.begin(); it != L.end(); ++it) {
    it->SaveB(fp);
  }
}
template<class TYPE> inline void ListLoadB(std::list<TYPE> &L, FILE *fp) {
  const int N = UT::LoadB<int>(fp);
  L.resize(N);
  for (typename std::list<TYPE>::iterator it = L.begin(); it != L.end(); ++it) {
    it->LoadB(fp);
  }
}

class Tag {
 public:
  inline Tag() {}
  inline Tag(const int iFrm, const float t) : m_iFrm(iFrm), m_t(t) {}
  inline bool operator == (const Tag &T) const { return m_iFrm == T.m_iFrm && m_t == T.m_t; }
  inline bool operator != (const Tag &T) const { return m_iFrm != T.m_iFrm || m_t != T.m_t; }
  inline bool operator < (const Tag &T) const { return m_iFrm < T.m_iFrm && m_t < T.m_t; }
  inline bool operator < (const int iFrm) const { return m_iFrm < iFrm; }
  inline bool operator <= (const Tag &T) const { return m_iFrm <= T.m_iFrm && m_t <= T.m_t; }
  inline bool operator > (const Tag &T) const { return m_iFrm > T.m_iFrm && m_t > T.m_t; }
  inline bool Valid() const { return m_iFrm >= 0; }
  inline bool Invalid() const { return m_iFrm == -1; }
  inline void Invalidate() { m_iFrm = -1; }
  inline void SaveB(FILE *fp) const {
    UT::SaveB(m_iFrm, fp);
    UT::SaveB(m_t, fp);
    UT::StringSaveB(m_fileName, fp);
#ifdef CFG_STEREO
    UT::StringSaveB(m_fileNameRight, fp);
#endif
  }
  inline void LoadB(FILE *fp) {
    UT::LoadB(m_iFrm, fp);
    UT::LoadB(m_t, fp);
    UT::StringLoadB(m_fileName, fp);
#ifdef CFG_STEREO
    UT::StringLoadB(m_fileNameRight, fp);
#endif
  }
 public:
  int m_iFrm;
  float m_t;
  std::string m_fileName;
#ifdef CFG_STEREO
  std::string m_fileNameRight;
#endif
};

class Measurement {
 public:
  inline Measurement() {}
  inline Measurement(const int iKF, const int ik, const int iz1, const int iz2) :
                     m_iKF(iKF), m_ik(ik), m_iz1(iz1), m_iz2(iz2) {}
  inline bool operator == (const Measurement &Z) const {
    return m_iKF == Z.m_iKF && m_ik == Z.m_ik && m_iz1 == Z.m_iz1 && m_iz2 == Z.m_iz2;
  }
  inline bool operator < (const int iKF) const { return m_iKF < iKF; }
  inline int CountFeatureMeasurements() const { return m_iz2 - m_iz1; }
  inline bool Inside(const int iz) const { return iz >= m_iz1 && iz < m_iz2; }
  inline void AssertConsistency() const {
    UT_ASSERT(m_iKF >= 0 && m_ik >= 0 && m_iz1 <= m_iz2);
  }
 public:
  int m_iKF, m_ik, m_iz1, m_iz2;
};

class Frame {
 public:
  inline void Initialize(const Tag &T) { m_T = T; m_d.Initialize(); ClearMeasurements(); }
  inline void Initialize(const Frame &F) { *this = F; }
  inline bool Valid() const { return m_T.Valid(); }
  inline bool Invalid() const { return m_T.Invalid(); }
  inline void Invalidate() { m_T.Invalidate(); }
  inline void DeleteKeyFrame(const int iKF, const std::vector<Measurement>::iterator *iZ = NULL,
                             const std::vector<int>::iterator *ik = NULL) {
    const std::vector<Measurement>::iterator _iZ = iZ ? *iZ : std::lower_bound(m_Zs.begin(),
                                                                               m_Zs.end(), iKF);
    const std::vector<int>::iterator _ik = ik ? *ik : std::lower_bound(m_iKFsMatch.begin(),
                                                                       m_iKFsMatch.end(), iKF);
    if (_iZ != m_Zs.end()) {
      const bool Z = _iZ->m_iKF == iKF;
      if (Z) {
        const int Nz = _iZ->CountFeatureMeasurements();
        for (std::vector<Measurement>::iterator jZ = _iZ + 1; jZ != m_Zs.end(); ++jZ) {
          jZ->m_iz1 -= Nz;
          jZ->m_iz2 -= Nz;
        }
        m_zs.erase(m_zs.begin() + _iZ->m_iz1, m_zs.begin() + _iZ->m_iz2);
      }
      const bool k = _ik != m_iKFsMatch.end() && *_ik == iKF;
      for (std::vector<Measurement>::iterator jZ = Z ? _iZ + 1 : _iZ; jZ != m_Zs.end(); ++jZ) {
        --jZ->m_iKF;
        if (k) {
          --jZ->m_ik;
        }
      }
      if (Z) {
        m_Zs.erase(_iZ);
      }
    }
    DeleteMatchKeyFrame(iKF, &_ik);
    if (m_iKFNearest == iKF) {
      m_iKFNearest = -1;
    } else if (m_iKFNearest > iKF) {
      --m_iKFNearest;
    }
  }
  inline void InsertMatchKeyFrame(const int iKF, const std::vector<int>::iterator *ik = NULL) {
    if (m_iKFsMatch.empty() || iKF > m_iKFsMatch.back()) {
      m_iKFsMatch.push_back(iKF);
      return;
    }
    const std::vector<int>::iterator _ik = ik ? *ik : std::lower_bound(m_iKFsMatch.begin(),
                                                                       m_iKFsMatch.end(), iKF);
    if (_ik != m_iKFsMatch.end() && *_ik == iKF) {
      return;
    }
    m_iKFsMatch.insert(_ik, iKF);
    const std::vector<Measurement>::iterator iZ = std::lower_bound(m_Zs.begin(), m_Zs.end(), iKF);
    for (std::vector<Measurement>::iterator jZ = iZ; jZ != m_Zs.end(); ++jZ) {
      ++jZ->m_ik;
    }
  }
  inline void DeleteMatchKeyFrame(const int iKF, const std::vector<int>::iterator *ik = NULL) {
    const std::vector<int>::iterator _ik = ik ? *ik : std::lower_bound(m_iKFsMatch.begin(),
                                                                       m_iKFsMatch.end(), iKF);
    if (_ik != m_iKFsMatch.end()) {
      const bool k = *_ik == iKF;
      for (std::vector<int>::iterator jk = k ? _ik + 1 : _ik;
           jk != m_iKFsMatch.end(); ++jk) {
        --*jk;
      }
      if (k) {
        m_iKFsMatch.erase(_ik);
      }
    }
  }
  inline void DeleteFeatureMeasurementsPrepare(const std::vector<int> &izsDel,
                                               std::vector<int> *izs) const {
    const int N = static_cast<int>(izsDel.size());
#ifdef CFG_DEBUG
    UT_ASSERT(N > 0);
#endif
    const int Nz = static_cast<int>(m_zs.size());
    izs->resize(Nz + 1);
    for (int iz1 = 0, iz2 = 0, i = 0, jz = izsDel[i]; iz1 < Nz; ++iz1) {
#ifdef CFG_DEBUG
      UT_ASSERT(jz >= iz1);
#endif
      if (iz1 == jz) {
        izs->at(iz1) = -1;
        jz = ++i == N ? Nz : izsDel[i];
      } else {
        izs->at(iz1) = iz2++;
      }
    }
    izs->at(Nz) = Nz - N;
  }
  inline void DeleteFeatureMeasurements(const std::vector<int> &izs) {
    const int Nz1 = static_cast<int>(m_zs.size());
#ifdef CFG_DEBUG
    UT_ASSERT(static_cast<int>(izs.size()) == Nz1 + 1);
    int zCnt = 0;
    for (int iz1 = 0, iz2 = 0; iz1 < Nz1; ++iz1) {
      if (izs[iz1] >= 0) {
        UT_ASSERT(izs[iz1] == iz2++);
      } else {
        ++zCnt;
      }
    }
    UT_ASSERT(izs.back() == Nz1 - zCnt);
#endif
    for (int iz1 = 0; iz1 < Nz1; ++iz1) {
      const int iz2 = izs[iz1];
      if (iz2 >= 0) {
        m_zs[iz2] = m_zs[iz1];
      }
    }
    const int Nz2 = izs.back();
    m_zs.resize(Nz2);

    const int NZ = static_cast<int>(m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      Measurement &Z = m_Zs[iZ];
      for (int iz = Z.m_iz1; iz <= Nz1 && (Z.m_iz1 = izs[iz]) < 0; ++iz);
      for (int iz = Z.m_iz2; iz <= Nz1 && (Z.m_iz2 = izs[iz]) < 0; ++iz);
    }
  }
  inline void ClearMeasurements() {
    m_Zs.resize(0);
    m_zs.resize(0);
    m_iKFsMatch.resize(0);
    m_iKFNearest = -1;
  }
  inline void PushFrameMeasurement(const int iKF, const int Nz) {
    const int iZ = int(m_Zs.size());
    m_Zs.resize(iZ + 1);
    Measurement &Z = m_Zs[iZ];
    Z.m_iKF = iKF;
    if (m_iKFsMatch.empty() || m_iKFsMatch.back() < iKF) {
      Z.m_ik = static_cast<int>(m_iKFsMatch.size());
      m_iKFsMatch.push_back(iKF);
    } else {
      const std::vector<int>::iterator ik = std::lower_bound(m_iKFsMatch.begin(),
                                                             m_iKFsMatch.end(), iKF);
      Z.m_ik = static_cast<int>(ik - m_iKFsMatch.begin());
      if (*ik != iKF) {
        m_iKFsMatch.insert(ik, iKF);
      }
    }
    Z.m_iz1 = static_cast<int>(m_zs.size());
    Z.m_iz2 = Z.m_iz1 + Nz;
    m_zs.resize(Z.m_iz2);
  }
  inline void PushFrameMeasurement(const int iKF, const std::vector<FTR::Measurement> &zs) {
    const int iZ = static_cast<int>(m_Zs.size());
    m_Zs.resize(iZ + 1);
    Measurement &Z = m_Zs[iZ];
    Z.m_iKF = iKF;
    if (m_iKFsMatch.empty() || m_iKFsMatch.back() < iKF) {
      Z.m_ik = static_cast<int>(m_iKFsMatch.size());
      m_iKFsMatch.push_back(iKF);
    } else {
      const std::vector<int>::iterator ik = std::lower_bound(m_iKFsMatch.begin(),
                                                             m_iKFsMatch.end(), iKF);
      Z.m_ik = static_cast<int>(ik - m_iKFsMatch.begin());
      if (*ik != iKF) {
        m_iKFsMatch.insert(ik, iKF);
      }
    }
    const int Nz = static_cast<int>(zs.size());
    Z.m_iz1 = static_cast<int>(m_zs.size());
    Z.m_iz2 = Z.m_iz1 + Nz;
    m_zs.insert(m_zs.end(), zs.begin(), zs.end());
  }
  inline void PushFeatureMeasurements(const int iKF, const std::vector<FTR::Measurement> &zs,
                                      int *iZ, int *iz) {
    if (m_Zs.empty() || iKF > m_Zs.back().m_iKF) {
      *iZ = static_cast<int>(m_Zs.size());
      *iz = static_cast<int>(m_zs.size());
      PushFrameMeasurement(iKF, zs);
    } else {
      const int Nz = static_cast<int>(zs.size());
      const std::vector<Measurement>::iterator _iZ = std::lower_bound(m_Zs.begin(),
                                                                      m_Zs.end(), iKF);
      *iZ = static_cast<int>(_iZ - m_Zs.begin());
      if (_iZ->m_iKF == iKF) {
        *iz = _iZ->m_iz2;
        _iZ->m_iz2 += Nz;
        for (std::vector<Measurement>::iterator jZ = _iZ + 1; jZ != m_Zs.end(); ++jZ) {
          jZ->m_iz1 += Nz;
          jZ->m_iz2 += Nz;
        }
      } else {
        const std::vector<int>::iterator ik = std::lower_bound(m_iKFsMatch.begin(),
                                                               m_iKFsMatch.begin() + _iZ->m_ik, iKF);
        const int _ik = static_cast<int>(ik - m_iKFsMatch.begin());
        if (ik == m_iKFsMatch.end() || *ik != iKF) {
          m_iKFsMatch.insert(ik, iKF);
          for (std::vector<Measurement>::iterator jZ = _iZ; jZ != m_Zs.end(); ++jZ) {
            ++jZ->m_ik;
          }
        }
        *iz = _iZ->m_iz1;
        for (std::vector<Measurement>::iterator jZ = _iZ; jZ != m_Zs.end(); ++jZ) {
          jZ->m_iz1 += Nz;
          jZ->m_iz2 += Nz;
        }
        m_Zs.insert(_iZ, Measurement(iKF, _ik, *iz, *iz + Nz));
      }
      m_zs.insert(m_zs.begin() + *iz, zs.begin(), zs.end());
    }
  }
  inline void PopFrameMeasurement() {
#ifdef CFG_DEBUG
    UT_ASSERT(!m_Zs.empty() && !m_iKFsMatch.empty() && m_Zs.back().m_iKF == m_iKFsMatch.back() &&
              m_Zs.back().m_ik == static_cast<int>(m_iKFsMatch.size()) - 1);
#endif
    m_Zs.resize(m_Zs.size() - 1);
    if (m_Zs.empty()) {
      m_zs.resize(0);
    } else {
      m_zs.resize(m_Zs.back().m_iz2);
    }
    m_iKFsMatch.resize(m_iKFsMatch.size() - 1);
  }
  inline int SearchFrameMeasurement(const int iKF) const {
    const std::vector<Measurement>::const_iterator iZ = std::lower_bound(m_Zs.begin(), m_Zs.end(),
                                                                         iKF);
    if (iZ == m_Zs.end() || iZ->m_iKF != iKF) {
      return -1;
    } else {
      return static_cast<int>(iZ - m_Zs.begin());
    }
  }
  inline int SearchFeatureMeasurement(const int iKF, const int ix) const {
    const int iZ = SearchFrameMeasurement(iKF);
    if (iZ == -1) {
      return -1;
    }
    const Measurement &Z = m_Zs[iZ];
    const int iz = static_cast<int>(std::lower_bound(m_zs.begin() + Z.m_iz1,
                                                     m_zs.begin() + Z.m_iz2, ix) - m_zs.begin());
    if (iz != Z.m_iz2 && m_zs[iz].m_ix == ix) {
      return iz;
    } else {
      return -1;
    }
  }
  inline int SearchFeatureMeasurementKeyFrame(const int iz) const {
    const int NZ = int(m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const Measurement &Z = m_Zs[iZ];
      if (Z.m_iz2 > iz) {
        return Z.m_iKF;
      }
    }
    return -1;
  }
  static inline void SearchFeatureMeasurementMatches(const Frame &F1, const Frame &F2,
                                                     std::vector<FTR::Measurement::Match> &izms,
                                                     const int iKF = -1) {
    izms.resize(0);
    const int NZ1 = static_cast<int>(F1.m_Zs.size());
    for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
      const Measurement &Z1 = F1.m_Zs[iZ1];
      if (iKF != -1 && Z1.m_iKF != iKF) {
        continue;
      }
      const int iZ2 = F2.SearchFrameMeasurement(Z1.m_iKF);
      if (iZ2 == -1) {
        continue;
      }
      const Measurement &Z2 = F2.m_Zs[iZ2];
      const std::vector<FTR::Measurement>::const_iterator iz21 = F2.m_zs.begin() + Z2.m_iz1,
                                                          iz22 = F2.m_zs.begin() + Z2.m_iz2;
      const int iz11 = Z1.m_iz1, iz12 = Z1.m_iz2;
      for (int iz1 = iz11; iz1 < iz12; ++iz1) {
        const int ix = F1.m_zs[iz1].m_ix;
        const std::vector<FTR::Measurement>::const_iterator iz2 = std::lower_bound(iz21, iz22, ix);
        if (iz2 != iz22 && iz2->m_ix == ix) {
          izms.push_back(FTR::Measurement::Match(iz1, int(iz2 - F2.m_zs.begin())));
        }
      }
    }
  }
  static inline bool HasFeatureMeasurementMatch(const Frame &F1, const Frame &F2) {
    const int NZ1 = static_cast<int>(F1.m_Zs.size());
    for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
      const Measurement &Z1 = F1.m_Zs[iZ1];
      const int iZ2 = F2.SearchFrameMeasurement(Z1.m_iKF);
      if (iZ2 == -1) {
        continue;
      }
      const Measurement &Z2 = F2.m_Zs[iZ2];
      const std::vector<FTR::Measurement>::const_iterator iz21 = F2.m_zs.begin() + Z2.m_iz1,
                                                          iz22 = F2.m_zs.begin() + Z2.m_iz2;
      const int iz11 = Z1.m_iz1, iz12 = Z1.m_iz2;
      for (int iz1 = iz11; iz1 < iz12; ++iz1) {
        const int ix = F1.m_zs[iz1].m_ix;
        const std::vector<FTR::Measurement>::const_iterator iz2 = std::lower_bound(iz21, iz22, ix);
        if (iz2 != iz22 && iz2->m_ix == ix) {
          return true;
        }
      }
    }
    return false;
  }
  inline int SearchMatchKeyFrame(const int iKF) const {
    const std::vector<int>::const_iterator ik = std::lower_bound(m_iKFsMatch.begin(),
                                                                 m_iKFsMatch.end(), iKF);
    return (ik == m_iKFsMatch.end() || *ik != iKF) ? -1 : int(ik - m_iKFsMatch.begin());
  }
  inline void SortFeatureMeasurements() {
    const int NZ = int(m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
      Measurement &Z = m_Zs[iZ];
      std::sort(m_zs.begin() + Z.m_iz1, m_zs.begin() + Z.m_iz2);
    }
  }
  inline void SaveB(FILE *fp) const {
    m_T.SaveB(fp);
    UT::SaveB(m_d, fp);
    UT::VectorSaveB(m_Zs, fp);
    UT::VectorSaveB(m_zs, fp);
    UT::VectorSaveB(m_iKFsMatch, fp);
    UT::SaveB(m_iKFNearest, fp);
  }
  inline void LoadB(FILE *fp) {
    m_T.LoadB(fp);
    UT::LoadB(m_d, fp);
    UT::VectorLoadB(m_Zs, fp);
    UT::VectorLoadB(m_zs, fp);
    UT::VectorLoadB(m_iKFsMatch, fp);
    UT::LoadB(m_iKFNearest, fp);
  }
  inline void AssertConsistency() const {
    const int NZ = static_cast<int>(m_Zs.size());
    if (NZ == 0) {
      return;
    }
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const Measurement &Z = m_Zs[iZ];
      Z.AssertConsistency();
      UT_ASSERT(Z.m_iKF == m_iKFsMatch[Z.m_ik]);
    }
    UT_ASSERT(m_Zs.front().m_iz1 == 0);
    for (int iZ = 1; iZ < NZ; ++iZ) {
      const Measurement &Z1 = m_Zs[iZ - 1], &Z2 = m_Zs[iZ];
      UT_ASSERT(Z1.m_iKF < Z2.m_iKF && Z1.m_ik < Z2.m_ik);
      UT_ASSERT(Z1.m_iz2 == Z2.m_iz1);
    }
    UT_ASSERT(m_Zs.back().m_iz2 == static_cast<int>(m_zs.size()));
    const int Nk = static_cast<int>(m_iKFsMatch.size());
    for (int ik = 1; ik < Nk; ++ik) {
      UT_ASSERT(m_iKFsMatch[ik - 1] < m_iKFsMatch[ik]);
    }
    std::vector<int>::const_iterator ik = m_iKFsMatch.begin();
    for (int iZ = 0; iZ < NZ; ++iZ) {
      const Measurement &Z = m_Zs[iZ];
      for (int iz = Z.m_iz1 + 1; iz < Z.m_iz2; ++iz) {
        UT_ASSERT(m_zs[iz - 1].m_ix < m_zs[iz].m_ix);
      }
      ik = std::lower_bound(ik, m_iKFsMatch.end(), Z.m_iKF);
      UT_ASSERT(ik != m_iKFsMatch.end() && *ik == Z.m_iKF);
    }
#ifdef CFG_STEREO
    const int Nz = static_cast<int>(m_zs.size());
    for (int iz = 0; iz < Nz; ++iz) {
      const FTR::Measurement &z = m_zs[iz];
      UT_ASSERT(z.m_z.Valid() || z.m_zr.Valid());
    }
#endif
  }
 public:
  Tag m_T;
  Depth::InverseGaussian m_d;
  std::vector<Measurement> m_Zs;
  std::vector<FTR::Measurement> m_zs;
  std::vector<int> m_iKFsMatch;
  int m_iKFNearest;
};

class MeasurementMatch {
 public:
  inline void operator = (const MeasurementMatch &Zm) {
    m_ik2zm = Zm.m_ik2zm;
    m_izms = Zm.m_izms;
    m_Mczms.Set(Zm.m_Mczms);
    m_SMczms.Set(Zm.m_SMczms);
  }
  inline void Initialize() {
    m_ik2zm.assign(1, 0);
    m_izms.resize(0);
    m_Mczms.Resize(0);
    m_SMczms.Resize(0);
  }
  inline void InsertKeyFrame(const int ik) {
    const int izm = m_ik2zm[ik];
    m_ik2zm.insert(m_ik2zm.begin() + ik, izm);
    m_SMczms.InsertZero(ik);
  }
  inline void DeleteKeyFrame(const int ik) {
    DeleteFeatureMeasurementMatches(ik, m_ik2zm[ik], m_ik2zm[ik + 1]);
    m_ik2zm.erase(m_ik2zm.begin() + ik);
    m_SMczms.Erase(ik);
  }
  inline void InsertFeatureMeasurement1(const int ik, const int iz1, const int Nz1) {
    const int i1 = m_ik2zm[ik], i2 = m_ik2zm[ik + 1];
    for (int i = i2 - 1; i >= i1 && m_izms[i].m_iz1 >= iz1; --i) {
      m_izms[i].m_iz1 += Nz1;
    }
  }
  inline void InsertFeatureMeasurement2(const int ik, const int iz2, const int Nz2) {
    const int Nk = static_cast<int>(m_ik2zm.size()) - 1;
    for (int jk = ik + 1; jk < Nk; ++jk) {
      const int i1 = m_ik2zm[jk], i2 = m_ik2zm[jk + 1];
      for (int i = i2 - 1; i >= i1 && m_izms[i].m_iz2 >= iz2; --i) {
        m_izms[i].m_iz2 += Nz2;
      }
    }
  }
  inline void InsertFeatureMeasurementMatches(const int ik,
                                              const std::vector<FTR::Measurement::Match> &izms,
                                              AlignedVector<float> *work) {
    const std::vector<FTR::Measurement::Match>::iterator i =
      std::lower_bound(m_izms.begin() + m_ik2zm[ik],
                       m_izms.begin() + m_ik2zm[ik + 1], izms.front().m_iz2);
    const int Nk = static_cast<int>(m_ik2zm.size()) - 1, Nzm = static_cast<int>(izms.size());
    for (int jk = ik; jk < Nk; ++jk) {
      m_ik2zm[jk + 1] += Nzm;
    }
    m_Mczms.InsertZero(static_cast<int>(i - m_izms.begin()), Nzm, work);
    m_izms.insert(i, izms.begin(), izms.end());
  }
  inline void PushFeatureMeasurementMatches(const std::vector<FTR::Measurement::Match> &izms,
                                            ubyte *first = NULL) {
    const int Nzm1 = m_ik2zm.back(), Nzm = static_cast<int>(izms.size()), Nzm2 = Nzm1 + Nzm;
    if (!first || *first) {
      if (first) {
        *first = 0;
      }
      m_ik2zm.push_back(Nzm2);
      m_SMczms.Push().MakeZero();
    } else {
      m_ik2zm.back() = Nzm2;
    }
    m_izms.insert(m_izms.end(), izms.begin(), izms.end());
    m_Mczms.InsertZero(Nzm1, Nzm, NULL);
//#ifdef CFG_DEBUG
#if 0
    for (int i = Nzm1; i < Nzm2; ++i) {
      m_Mczms[i].Invalidate();
    }
#endif
  }
  inline void DeleteFeatureMeasurementMatches(const int ik, const int i1, const int i2,
                                              const ubyte *ms = NULL) {
    const int Nzm = i2 - i1;
    if (Nzm == 0) {
      return;
    }
    const int Nk = m_SMczms.Size();
    for (int jk = ik + 1; jk <= Nk; ++jk) {
      m_ik2zm[jk] -= Nzm;
    }
    m_izms.erase(m_izms.begin() + i1, m_izms.begin() + i2);
    if (ms) {
      Camera::Factor::Binary::CC &SMczm = m_SMczms[ik];
      for (int i = i1; i < i2; ++i) {
        if (!ms[i]) {
          continue;
        }
        Camera::Factor::Binary::CC &Mczm = m_Mczms[i];
        Mczm.MakeMinus();
        SMczm += Mczm;
      }
    }
    m_Mczms.Erase(i1, Nzm);
  }
  inline void DeleteFeatureMeasurementMatches(const int ik, const int i1, const int i2,
                                              const std::vector<int> &izs1,
                                              const std::vector<int> &izs2,
                                              std::vector<ubyte> *ms = NULL) {
    if (!izs1.empty() && !izs2.empty()) {
      int i, j;
      Camera::Factor::Binary::CC &SMczm = m_SMczms[ik];
      for (i = j = i1; i < i2; ++i) {
        const FTR::Measurement::Match &izm = m_izms[i];
        Camera::Factor::Binary::CC &Mczm = m_Mczms[i];
        const int iz1 = izs1[izm.m_iz1], iz2 = izs2[izm.m_iz2];
#ifdef CFG_DEBUG
        UT_ASSERT(iz1 >= 0 && iz2 >= 0 || iz1 == iz2);
#endif
        if (iz1 < 0) {
          //if (iz1 == -1) {
          if (ms && ms->at(i) || !ms && iz1 == -1) {
            Mczm.MakeMinus();
            SMczm += Mczm;
          }
          continue;
        }
        m_izms[j].Set(iz1, iz2);
        m_Mczms[j] = Mczm;
        if (ms) {
          ms->at(j) = ms->at(i);
        }
        ++j;
      }
      const int Nzm = i - j;
      m_izms.erase(m_izms.begin() + j, m_izms.begin() + i);
      m_Mczms.Erase(j, Nzm);
      if (ms) {
        ms->erase(ms->begin() + j, ms->begin() + i);
      }
      const int Nk = m_SMczms.Size();
      for (int jk = ik + 1; jk <= Nk; ++jk) {
        m_ik2zm[jk] -= Nzm;
      }
    } else if (!izs1.empty()) {
      const int Nzm = static_cast<int>(m_izms.size());
      for (int i = i1; i < i2; ++i) {
        m_izms[i].m_iz1 = izs1[m_izms[i].m_iz1];
      }
    } else if (!izs2.empty()) {
      const int Nzm = static_cast<int>(m_izms.size());
      for (int i = i1; i < i2; ++i) {
        m_izms[i].m_iz2 = izs2[m_izms[i].m_iz2];
      }
    }
  }
  inline void MakeZero() { m_Mczms.MakeZero(); m_SMczms.MakeZero(); }
  inline void SaveB(FILE *fp) const {
    UT::VectorSaveB(m_ik2zm, fp);
    UT::VectorSaveB(m_izms, fp);
    m_Mczms.SaveB(fp);
    m_SMczms.SaveB(fp);
  }
  inline void LoadB(FILE *fp) {
    UT::VectorLoadB(m_ik2zm, fp);
    UT::VectorLoadB(m_izms, fp);
    m_Mczms.LoadB(fp);
    m_SMczms.LoadB(fp);
  }
  inline void AssertConsistency(const int Nk) const {
    const int Nzm = static_cast<int>(m_izms.size());
    UT_ASSERT(static_cast<int>(m_ik2zm.size()) == Nk + 1);
    UT_ASSERT(m_ik2zm[0] == 0 && m_ik2zm[Nk] == Nzm);
    for (int ik = 0; ik < Nk; ++ik) {
      const int i1 = m_ik2zm[ik], i2 = m_ik2zm[ik + 1];
      for (int i = i1 + 1; i < i2; ++i) {
        const FTR::Measurement::Match &z1 = m_izms[i - 1], &z2 = m_izms[i];
        UT_ASSERT(z1.m_iz1 < z2.m_iz1);
        UT_ASSERT(z1.m_iz2 < z2.m_iz2);
      }
    }
    UT_ASSERT(m_Mczms.Size() == Nzm);
    UT_ASSERT(m_SMczms.Size() == Nk);
  }
  inline void AssertConsistency(const int ik, const Frame &F1, const Frame &F2,
                                std::vector<FTR::Measurement::Match> &izmsTmp) const {
    Frame::SearchFeatureMeasurementMatches(F1, F2, izmsTmp);
    const int i1 = m_ik2zm[ik], i2 = m_ik2zm[ik + 1], Nzm = i2 - i1;
    UT_ASSERT(Nzm == static_cast<int>(izmsTmp.size()));
    const FTR::Measurement::Match *izms = m_izms.data() + i1;
    for (int i = 0; i < Nzm; ++i) {
      const FTR::Measurement::Match &izm = izms[i];
      UT_ASSERT(izm == izmsTmp[i]);
      UT_ASSERT(F1.m_zs[izm.m_iz1].m_ix == F2.m_zs[izm.m_iz2].m_ix);
    }
  }
 public:
  std::vector<int> m_ik2zm;
  std::vector<FTR::Measurement::Match> m_izms;
  AlignedVector<Camera::Factor::Binary::CC> m_Mczms, m_SMczms;
};

}
#endif
