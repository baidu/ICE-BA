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
#include "GlobalMap.h"

void GlobalMap::LBA_Reset() {
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_GM_LBA_Reset);
  m_Cs.Resize(0);
  m_Uc = GM_FLAG_FRAME_DEFAULT;
  m_ucs.resize(0);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_GM_LBA_Reset);
}

void GlobalMap::LBA_Push(const int iFrm, const Rigid3D &C) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GM_LBA_Push);
  m_Cs.Push(C);
  m_ucs.resize(m_Cs.Size(), GM_FLAG_FRAME_DEFAULT);
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GM_LBA_Push);
}

void GlobalMap::LBA_Delete(const int iKF) {
  const int nKFs = m_Cs.Size();
  m_Cs.Erase(iKF);
  const ubyte uc = m_ucs[iKF];
  m_ucs.erase(m_ucs.begin() + iKF);
  if ((uc & GM_FLAG_FRAME_UPDATE_CAMERA) &&
      !UT::VectorExistFlag<ubyte>(m_ucs.data(), nKFs - 1, GM_FLAG_FRAME_UPDATE_CAMERA)) {
    m_Uc &= ~GM_FLAG_FRAME_UPDATE_CAMERA;
  }
}

ubyte GlobalMap::LBA_Synchronize(const int iFrm, AlignedVector<Rigid3D> &Cs, std::vector<ubyte> &ucs) {
  ubyte ret;
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GM_LBA_Synchronize);
  if (!(m_Uc & GM_FLAG_FRAME_UPDATE_CAMERA)) {
    ret = GM_FLAG_FRAME_DEFAULT;
  } else {
    m_Uc &= ~GM_FLAG_FRAME_UPDATE_CAMERA;
    Cs.Set(m_Cs);
    const int nKFs = m_Cs.Size();
    ucs.assign(nKFs, GM_FLAG_FRAME_DEFAULT);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (!(m_ucs[iKF] & GM_FLAG_FRAME_UPDATE_CAMERA)) {
        continue;
      }
      ucs[iKF] = GM_FLAG_FRAME_UPDATE_CAMERA;
      m_ucs[iKF] &= ~GM_FLAG_FRAME_UPDATE_CAMERA;
    }
    ret = GM_FLAG_FRAME_UPDATE_CAMERA;
  }
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GM_LBA_Synchronize);
  return ret;
}

void GlobalMap::GBA_Update(const int iFrm, const AlignedVector<Rigid3D> &Cs,
                           const std::vector<ubyte> &ucs) {
  MT_WRITE_LOCK_BEGIN(m_MT, iFrm, MT_TASK_GM_GBA_Update);
  const int nKFs = Cs.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(m_Cs.Size() >= nKFs && static_cast<int>(ucs.size()) == nKFs);
#endif
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const ubyte uc = ucs[iKF];
    if (!(uc & GM_FLAG_FRAME_UPDATE_CAMERA)) {
      continue;
    }
    m_Cs[iKF] = Cs[iKF];
    m_Uc |= uc;
    m_ucs[iKF] |= uc;
  }
  MT_WRITE_LOCK_END(m_MT, iFrm, MT_TASK_GM_GBA_Update);
}

void GlobalMap::SaveB(FILE *fp) {
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  m_Cs.SaveB(fp);
  UT::SaveB(m_Uc, fp);
  UT::VectorSaveB(m_ucs, fp);
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void GlobalMap::LoadB(FILE *fp) {
  MT_WRITE_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  m_Cs.LoadB(fp);
  UT::LoadB(m_Uc, fp);
  UT::VectorLoadB(m_ucs, fp);
  MT_WRITE_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}

void GlobalMap::AssertConsistency() {
  MT_READ_LOCK_BEGIN(m_MT, MT_TASK_NONE, MT_TASK_NONE);
  const int nKFs = m_Cs.Size();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    m_Cs[iKF].AssertOrthogonal();
  }
  UT_ASSERT(static_cast<int>(m_ucs.size()) == nKFs);
  if (UT::VectorExistFlag<ubyte>(m_ucs.data(), nKFs, GM_FLAG_FRAME_UPDATE_CAMERA)) {
    UT_ASSERT((m_Uc & GM_FLAG_FRAME_UPDATE_CAMERA) != 0);
  } else {
    UT_ASSERT(!(m_Uc & GM_FLAG_FRAME_UPDATE_CAMERA));
  }
  MT_READ_LOCK_END(m_MT, MT_TASK_NONE, MT_TASK_NONE);
}
