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
  m_CsKF.resize(0);
  m_ds.resize(0);
  m_iKF2d.assign(1, 0);
  m_Uc = LM_FLAG_FRAME_DEFAULT;
  m_uds.resize(0);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_LM_IBA_Reset);
}

void LocalMap::IBA_PushLocalFrame(const CameraLF &C) {
  MT_WRITE_LOCK_BEGIN(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushLocalFrame);
  m_CsLF.push_back(C);
  MT_WRITE_LOCK_END(m_MT, C.m_iFrm, MT_TASK_LM_IBA_PushLocalFrame);
}

void LocalMap::IBA_PushKeyFrame(const GlobalMap::InputKeyFrame &KF) {
  MT_WRITE_LOCK_BEGIN(m_MT, KF.m_T.m_iFrm, MT_TASK_LM_IBA_PushKeyFrame);
  m_CsKF.push_back(CameraKF(KF.m_C.m_T, KF.m_T.m_iFrm));
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
  MT_WRITE_LOCK_END(m_MT, KF.m_T.m_iFrm, MT_TASK_LM_IBA_PushKeyFrame);
}

void LocalMap::IBA_DeleteKeyFrame(const int iFrm, const int iKF) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_LM_IBA_DeleteKeyFrame);
  const int nKFs = static_cast<int>(m_CsKF.size());
  m_CsKF.erase(m_CsKF.begin() + iKF);
  const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1], Nd = id2 - id1;
  for (int jKF = iKF + 1; jKF <= nKFs; ++jKF) {
    m_iKF2d[jKF] -= Nd;
  }
  m_iKF2d.erase(m_iKF2d.begin() + iKF);
  m_ds.erase(m_ds.begin() + id1, m_ds.begin() + id2);
  m_uds.erase(m_uds.begin() + id1, m_uds.begin() + id2);
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_LM_IBA_DeleteKeyFrame);
}

ubyte LocalMap::IBA_Synchronize(const int iFrm, std::list<CameraLF> &CsLF,
                                std::vector<CameraKF> &CsKF,
                                std::vector<Depth::InverseGaussian> &ds,
                                std::vector<ubyte> &uds) {
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
      const int nKFs = static_cast<int>(m_CsKF.size());
#ifdef CFG_DEBUG
      UT_ASSERT(static_cast<int>(CsKF.size()) == nKFs && ds.size() == m_ds.size());
#endif
      if (ud) {
        m_uds.swap(uds);
        m_uds.assign(uds.size(), LM_FLAG_TRACK_DEFAULT);
      }
      for (int iKF = 0; iKF < nKFs; ++iKF) {
        CameraKF &C1 = m_CsKF[iKF];
        if (C1.m_uc == LM_FLAG_FRAME_DEFAULT) {
          continue;
        }
        CameraKF &C2 = CsKF[iKF];
#ifdef CFG_DEBUG
        UT_ASSERT(C1.m_iFrm == C2.m_iFrm);
#endif
        if (C1.m_uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF) {
          C2.m_C = C1.m_C;
#ifdef CFG_CHECK_REPROJECTION
          C2.m_e = C1.m_e;
#endif
        }
        if (C1.m_uc & LM_FLAG_FRAME_UPDATE_DEPTH) {
          const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1], Nx = id2 - id1;
          memcpy(ds.data() + id1, m_ds.data() + id1, sizeof(Depth::InverseGaussian) * Nx);
        }
        C2.m_uc = C1.m_uc;
        C1.m_uc = LM_FLAG_FRAME_DEFAULT;
      }
    }
  }
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_LM_IBA_Synchronize);
  return Uc;
}

void LocalMap::LBA_Update(const int iFrm1, const int iFrm2, const std::vector<int> &ic2LF,
                          const AlignedVector<Camera> &CsLF, const std::vector<ubyte> &ucsLF,
                          const std::vector<int> &iFrmsKF, const AlignedVector<Rigid3D> &CsKF,
                          const std::vector<ubyte> &ucsKF, const std::vector<int> &iKF2d,
                          const std::vector<Depth::InverseGaussian> &ds,
                          const std::vector<ubyte> &uds
#ifdef CFG_CHECK_REPROJECTION
                        , const std::vector<std::pair<float, float> > &esLF,
                          const std::vector<std::pair<float, float> > &esKF
#endif
                        ) {
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
#ifdef CFG_CHECK_REPROJECTION
    C->m_e = esLF[iLF];
#endif
  }
  std::vector<CameraKF>::iterator i = m_CsKF.begin();
  const int nKFs = CsKF.Size();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const ubyte uc = ucsKF[iKF];
    if (!uc) {
      continue;
    }
    const int iFrm = iFrmsKF[iKF];
    i = std::lower_bound(i, m_CsKF.end(), iFrm);
    if (i == m_CsKF.end()) {
      break;
    } else if (i->m_iFrm != iFrm) {
      continue;
    }
    if (uc & LM_FLAG_FRAME_UPDATE_CAMERA_KF) {
      i->m_C = CsKF[iKF];
    }
    if (uc & LM_FLAG_FRAME_UPDATE_DEPTH) {
      const int _iKF = static_cast<int>(i - m_CsKF.begin());
      const int id1 = iKF2d[iKF], id2 = iKF2d[iKF + 1], Nx = id2 - id1;
      const int _id1 = m_iKF2d[_iKF], _id2 = m_iKF2d[_iKF + 1];
#ifdef CFG_DEBUG
      UT_ASSERT(Nx <= _id2 - _id1);
#endif
      memcpy(m_ds.data() + _id1, ds.data() + id1, sizeof(Depth::InverseGaussian) * Nx);
      const ubyte *uds1 = uds.data() + id1;
      ubyte *uds2 = m_uds.data() + _id1;
      for (int ix = 0; ix < Nx; ++ix) {
        uds2[ix] |= uds1[ix];
      }
    }
    i->m_uc |= uc;
    m_Uc |= uc;
#ifdef CFG_CHECK_REPROJECTION
    i->m_e = esKF[iKF];
#endif
  }
  MT_WRITE_LOCK_END(m_MT, iFrm2, MT_TASK_LM_LBA_Update);
}

void LocalMap::SaveB(FILE *fp) {
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::ListSaveB(m_CsLF, fp);
  UT::VectorSaveB(m_CsKF, fp);
  UT::VectorSaveB(m_ds, fp);
  UT::VectorSaveB(m_iKF2d, fp);
  UT::SaveB(m_Uc, fp);
  UT::VectorSaveB(m_uds, fp);
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void LocalMap::LoadB(FILE *fp) {
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  UT::ListLoadB(m_CsLF, fp);
  UT::VectorLoadB(m_CsKF, fp);
  UT::VectorLoadB(m_ds, fp);
  UT::VectorLoadB(m_iKF2d, fp);
  UT::LoadB(m_Uc, fp);
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
  ubyte Uc = GM_FLAG_FRAME_DEFAULT;
  for (std::list<CameraLF>::const_iterator C = m_CsLF.begin(); C != m_CsLF.end(); ++C) {
    C->m_C.AssertConsistency();
    Uc |= C->m_uc;
  }
  const int nKFs = static_cast<int>(m_CsKF.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const CameraKF &C = m_CsKF[iKF];
    C.m_C.AssertOrthogonal();
    Uc |= C.m_uc;
  }
  UT_ASSERT(m_Uc == Uc);
  UT_ASSERT(static_cast<int>(m_iKF2d.size()) == nKFs + 1);
  UT_ASSERT(m_iKF2d[0] == 0 && m_iKF2d[nKFs] == static_cast<int>(m_ds.size()));
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1];
    UT_ASSERT((m_CsKF[iKF].m_uc & LM_FLAG_FRAME_UPDATE_DEPTH) != 0 ==
              UT::VectorExistFlag<ubyte>(m_uds.data() + id1, id2 - id1,
                                         LM_FLAG_TRACK_UPDATE_DEPTH));
  }
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}
