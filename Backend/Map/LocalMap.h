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
#ifndef _LOCAL_MAP_H_
#define _LOCAL_MAP_H_

#include "GlobalMap.h"
#include "MultiThread.h"

#define LM_FLAG_FRAME_DEFAULT           0
#define LM_FLAG_FRAME_UPDATE_CAMERA_LF  1
#define LM_FLAG_FRAME_UPDATE_CAMERA_KF  2
#define LM_FLAG_FRAME_UPDATE_DEPTH      4

#define LM_FLAG_TRACK_DEFAULT       0
#define LM_FLAG_TRACK_UPDATE_DEPTH  1

class LocalMap {

 public:

  class CameraLF {
   public:
    inline CameraLF() {}
    inline CameraLF(const Camera &C, const int iFrm, const ubyte uc = LM_FLAG_FRAME_DEFAULT) :
                    m_C(C), m_iFrm(iFrm), m_uc(uc) {}
    inline bool operator < (const int iFrm) const { return m_iFrm < iFrm; }
    inline void SaveB(FILE *fp) const {
      UT::SaveB(m_C, fp);
      UT::SaveB(m_iFrm, fp);
      UT::SaveB(m_uc, fp);
    }
    inline void LoadB(FILE *fp) {
      UT::LoadB(m_C, fp);
      UT::LoadB(m_iFrm, fp);
      UT::LoadB(m_uc, fp);
    }
   public:
    Camera m_C;
    int m_iFrm;
    ubyte m_uc;
  };

 public:

  void IBA_Reset();
  void IBA_PushCurrentFrame(const CameraLF &C);
  void IBA_PushCurrentFrame(const CameraLF &C, const GlobalMap::InputKeyFrame &KF);
  void IBA_DeleteKeyFrame(const int iKF);
  ubyte IBA_Synchronize(const int iFrm, std::list<CameraLF> &CsLF, AlignedVector<Rigid3D> &CsKF,
                        std::vector<ubyte> &ucsKF, std::vector<Depth::InverseGaussian> &ds,
                        std::vector<ubyte> &uds);
  void LBA_Update(const int iFrm1, const int iFrm2, const std::vector<int> &ic2LF,
                  const AlignedVector<Camera> &CsLF, const std::vector<ubyte> &ucsLF,
                  const AlignedVector<Rigid3D> &CsKF, const std::vector<ubyte> &ucsKF,
                  const std::vector<int> &iKF2d, const std::vector<Depth::InverseGaussian> &ds,
                  const std::vector<ubyte> &uds);

  void SaveB(FILE *fp);
  void LoadB(FILE *fp);
  void AssertConsistency();

 protected:

  std::list<CameraLF> m_CsLF;
  AlignedVector<Rigid3D> m_CsKF;
  std::vector<Depth::InverseGaussian> m_ds;
  std::vector<int> m_iKF2d;
  ubyte m_Uc;
  std::vector<ubyte> m_ucsKF, m_uds;
  boost::shared_mutex m_MT;

};

#endif
