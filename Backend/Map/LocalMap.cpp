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
#include "LocalMap.h"

void LocalMap::IBA_Reset() {
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_LM_IBA_Reset);
  m_CsLF.resize(0);
  m_CsKF.Resize(0);
  m_ds.resize(0);
  m_iKF2d.assign(1, 0);
  m_Uc = LM_FLAG_FRAME_DEFAULT;
  m_ucsKF.resize(0);
  m_uds.resize(0);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_LM_IBA_Reset);
}

void LocalMap::IBA_PushCurrentFrame(const CameraLF &C) {
  MT_WRITE_LOCK_BEGIN(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushCurrentFrame);
  m_CsLF.push_back(C);
  MT_WRITE_LOCK_END(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushCurrentFrame);
}

void LocalMap::IBA_PushCurrentFrame(const CameraLF &C, const GlobalMap::InputKeyFrame &KF) {
  MT_WRITE_LOCK_BEGIN(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushCurrentFrame);
  m_CsLF.push_back(C);
  m_CsKF.Push(KF.m_C.m_T);
  const int nKFs = static_cast<int>(m_iKF2d.size());
  m_iKF2d.push_back(m_iKF2d.back());
  const int NX = static_cast<int>(KF.m_Xs.size());
  for (int iX1 = 0, iX2 = 0; iX1 < NX; iX1 = iX2) {
    const int iKF = KF.m_Xs[iX1].m_iKF;
    for (iX2 = iX1 + 1; iX2 < NX && KF.m_Xs[iX2].m_iKF == iKF; ++iX2) {}
    const int id = m_iKF2d[iKF + 1], Nx = iX2 - iX1;
    for (int jKF = iKF + 1; jKF <= nKFs; ++jKF) {
      m_iKF2d[jKF] += Nx;
    }
    m_ds.insert(m_ds.begin() + id, Nx, Depth::InverseGaussian());
    const GlobalMap::Point *Xs = KF.m_Xs.data() + iX1;
    Depth::InverseGaussian *ds = m_ds.data() + id;
    for (int ix = 0; ix < Nx; ++ix) {
      ds[ix] = Xs[ix].m_d;
    }
    m_uds.insert(m_uds.begin() + id, Nx, LM_FLAG_TRACK_DEFAULT);
  }
  m_ucsKF.push_back(LM_FLAG_FRAME_DEFAULT);
  MT_WRITE_LOCK_END(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushCurrentFrame);
}

void LocalMap::IBA_DeleteKeyFrame(const int iKF) {
  const int nKFs = m_CsKF.Size();
  m_CsKF.Erase(iKF);
  const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1], Nd = id2 - id1;
  for (int jKF = iKF + 1; jKF <= nKFs; ++jKF) {
    m_iKF2d[jKF] -= Nd;
  }
  m_iKF2d.erase(m_iKF2d.begin() + iKF);
  m_ds.erase(m_ds.begin() + id1, m_ds.begin() + id2);
  m_ucsKF.erase(m_ucsKF.begin() + iKF);
  m_uds.erase(m_uds.begin() + id1, m_uds.begin() + id2);
}

ubyte LocalMap::IBA_Synchronize(const int iFrm, std::list<CameraLF> &CsLF,
                                AlignedVector<Rigid3D> &CsKF, std::vector<ubyte> &ucsKF,
                                std::vector<Depth::InverseGaussian> &ds, std::vector<ubyte> &uds) {
  ubyte Uc = LM_FLAG_FRAME_DEFAULT;
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_LM_IBA_Synchronize);
  if (m_Uc) {
    if (m_Uc & LM_FLAG_FRAME_UPDATE_CAMERA_LF) {
      CsLF = m_CsLF;
      for (std::list<CameraLF>::iterator C = m_CsLF.begin(); C != m_CsLF.end(); ++C) {
        C->m_uc = LM_FLAG_FRAME_DEFAULT;
      }
    }
    Uc = m_Uc;
    m_Uc = LM_FLAG_FRAME_DEFAULT;
    const ubyte uc = Uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF, ud = Uc & LM_FLAG_FRAME_UPDATE_DEPTH;
    if (uc || ud) {
      const int nKFs = m_CsKF.Size();
      m_ucsKF.swap(ucsKF);
      m_ucsKF.assign(nKFs, LM_FLAG_FRAME_DEFAULT);
      if (uc) {
        CsKF.Set(m_CsKF);
      }
      m_uds.swap(uds);
      m_uds.resize(uds.size());
      if (ud) {
#ifdef CFG_DEBUG
        UT_ASSERT(ds.size() == m_ds.size());
#endif
        for (int iKF = 0; iKF < nKFs; ++iKF) {
          if (!(ucsKF[iKF] & LM_FLAG_FRAME_UPDATE_DEPTH)) {
            continue;
          }
          const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1], Nx = id2 - id1;
          memset(m_uds.data() + id1, LM_FLAG_TRACK_DEFAULT, Nx);
          memcpy(ds.data() + id1, m_ds.data() + id1, sizeof(Depth::InverseGaussian) * Nx);
        }
      }
    }
  }
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_LM_IBA_Synchronize);
  return Uc;
}

void LocalMap::LBA_Update(const int iFrm1, const int iFrm2, const std::vector<int> &ic2LF,
                          const AlignedVector<Camera> &CsLF, const std::vector<ubyte> &ucsLF,
                          const AlignedVector<Rigid3D> &CsKF, const std::vector<ubyte> &ucsKF,
                          const std::vector<int> &iKF2d, const std::vector<Depth::InverseGaussian> &ds,
                          const std::vector<ubyte> &uds) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm2, MT_TASK_LM_LBA_Update);
  const int nLFs = CsLF.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(static_cast<int>(m_CsLF.size()) >= nLFs);
#endif
  while (m_CsLF.front().m_iFrm != iFrm1) {
    m_CsLF.pop_front();
  }
#ifdef CFG_DEBUG
  UT_ASSERT(static_cast<int>(m_CsLF.size()) >= nLFs);
#endif
  std::list<CameraLF>::iterator C = m_CsLF.begin();
  for (int ic = 0; ic < nLFs; ++ic, ++C) {
    const int iLF = ic2LF[ic];
    if (!ucsLF[iLF]) {
      continue;
    }
    C->m_C = CsLF[iLF];
    C->m_uc = LM_FLAG_FRAME_UPDATE_CAMERA_LF;
    m_Uc |= LM_FLAG_FRAME_UPDATE_CAMERA_LF;
  }
  const int nKFs = CsKF.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(nKFs <= m_CsKF.Size() && ds.size() <= m_ds.size());
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const ubyte uc = ucsKF[iKF];
    if (!uc) {
      continue;
    } else if (uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF) {
      m_CsKF[iKF] = CsKF[iKF];
    }
    if (uc & LM_FLAG_FRAME_UPDATE_DEPTH) {
      const int Nx = iKF2d[iKF + 1] - iKF2d[iKF];
#ifdef CFG_DEBUG
      UT_ASSERT(Nx <= m_iKF2d[iKF + 1] - m_iKF2d[iKF]);
#endif
      memcpy(m_ds.data() + m_iKF2d[iKF], ds.data() + iKF2d[iKF],
             sizeof(Depth::InverseGaussian) * Nx);
      const ubyte *uds1 = uds.data() + iKF2d[iKF];
      ubyte *uds2 = m_uds.data() + m_iKF2d[iKF];
      for (int ix = 0; ix < Nx; ++ix) {
        uds2[ix] |= uds1[ix];
      }
    }
    m_Uc |= uc;
    m_ucsKF[iKF] |= uc;
  }
  MT_WRITE_LOCK_END(m_MT, iFrm2, MT_TASK_LM_LBA_Update);
}

void LocalMap::SaveB(FILE *fp) {
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  FRM::ListSaveB(m_CsLF, fp);
  m_CsKF.SaveB(fp);
  UT::VectorSaveB(m_ds, fp);
  UT::VectorSaveB(m_iKF2d, fp);
  UT::SaveB(m_Uc, fp);
  UT::VectorSaveB(m_ucsKF, fp);
  UT::VectorSaveB(m_uds, fp);
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void LocalMap::LoadB(FILE *fp) {
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  FRM::ListLoadB(m_CsLF, fp);
  m_CsKF.LoadB(fp);
  UT::VectorLoadB(m_ds, fp);
  UT::VectorLoadB(m_iKF2d, fp);
  UT::LoadB(m_Uc, fp);
  UT::VectorLoadB(m_ucsKF, fp);
  UT::VectorLoadB(m_uds, fp);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void LocalMap::AssertConsistency() {
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  for (std::list<CameraLF>::const_iterator C = m_CsLF.begin(); C != m_CsLF.end(); ++C) {
    C->m_C.AssertConsistency();
  }
  for (std::list<CameraLF>::const_iterator C1 = m_CsLF.begin(), C2 = m_CsLF.begin();
       ++C2 != m_CsLF.end(); C1 = C2) {
    UT_ASSERT(C1->m_iFrm < C2->m_iFrm);
  }
  const int nKFs = m_CsKF.Size();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    m_CsKF[iKF].AssertOrthogonal();
  }
  UT_ASSERT(static_cast<int>(m_iKF2d.size()) == nKFs + 1);
  UT_ASSERT(m_iKF2d[0] == 0 && m_iKF2d[nKFs] == static_cast<int>(m_ds.size()));
  if (m_Uc & LM_FLAG_FRAME_UPDATE_CAMERA_LF) {
    bool exist = false;
    for (std::list<CameraLF>::const_iterator C = m_CsLF.begin(); C != m_CsLF.end() && !exist; ++C) {
      exist = C->m_uc == LM_FLAG_FRAME_UPDATE_CAMERA_LF;
    }
    UT_ASSERT(exist);
  } else {
    for (std::list<CameraLF>::const_iterator C = m_CsLF.begin(); C != m_CsLF.end(); ++C) {
      UT_ASSERT(C->m_uc == LM_FLAG_FRAME_DEFAULT);
    }
  }
  UT_ASSERT((m_Uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF) != 0 == UT::VectorExistFlag<ubyte>
            (m_ucsKF.data(), nKFs, LM_FLAG_FRAME_UPDATE_CAMERA_KF));
  UT_ASSERT((m_Uc & LM_FLAG_FRAME_UPDATE_DEPTH) != 0 ==
            UT::VectorExistFlag<ubyte>(m_ucsKF.data(), nKFs, LM_FLAG_FRAME_UPDATE_DEPTH));
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1];
    UT_ASSERT((m_ucsKF[iKF] & LM_FLAG_FRAME_UPDATE_DEPTH) != 0 ==
              UT::VectorExistFlag<ubyte>(m_uds.data() + id1, id2 - id1,
                                         LM_FLAG_TRACK_UPDATE_DEPTH));
  }
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}
