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
                    m_C(C), m_iFrm(iFrm), m_uc(uc) {
#ifdef CFG_CHECK_REPROJECTION
      m_e.first = m_e.second = FLT_MAX;
#endif
    }
    inline bool operator < (const int iFrm) const { return m_iFrm < iFrm; }
   public:
    Camera m_C;
    int m_iFrm;
    ubyte m_uc;
#ifdef CFG_CHECK_REPROJECTION
    std::pair<float, float> m_e;
#endif
  };

  class CameraKF : public GlobalMap::Camera {
   public:
    inline CameraKF() : GlobalMap::Camera() {}
    inline CameraKF(const Rigid3D &C, const int iFrm, const ubyte uc = GM_FLAG_FRAME_DEFAULT) :
                    GlobalMap::Camera(C, iFrm, uc) {
#ifdef CFG_CHECK_REPROJECTION
      m_e.first = m_e.second = FLT_MAX;
#endif
    }
   public:
#ifdef CFG_CHECK_REPROJECTION
    std::pair<float, float> m_e;
#endif
  };

 public:

  void IBA_Reset();
  void IBA_PushLocalFrame(const CameraLF &C);
  void IBA_PushKeyFrame(const GlobalMap::InputKeyFrame &KF);
  void IBA_DeleteKeyFrame(const int iFrm, const int iKF);
  ubyte IBA_Synchronize(const int iFrm, std::list<CameraLF> &CsLF, std::vector<CameraKF> &CsKF,
                        std::vector<Depth::InverseGaussian> &ds, std::vector<ubyte> &uds);
  void LBA_Update(const int iFrm1, const int iFrm2, const std::vector<int> &ic2LF,
                  const AlignedVector<Camera> &CsLF, const std::vector<ubyte> &ucsLF,
                  const std::vector<int> &iFrmsKF, const AlignedVector<Rigid3D> &CsKF,
                  const std::vector<ubyte> &ucsKF, const std::vector<int> &iKF2d,
                  const std::vector<Depth::InverseGaussian> &ds, const std::vector<ubyte> &uds
#ifdef CFG_CHECK_REPROJECTION
                , const std::vector<std::pair<float, float> > &esLF,
                  const std::vector<std::pair<float, float> > &esKF
#endif
                );

  void SaveB(FILE *fp);
  void LoadB(FILE *fp);
  void AssertConsistency();

 protected:

  std::list<CameraLF> m_CsLF;
  std::vector<CameraKF> m_CsKF;
  std::vector<Depth::InverseGaussian> m_ds;
  std::vector<int> m_iKF2d;
  ubyte m_Uc;
  std::vector<ubyte> m_uds;
  boost::shared_mutex m_MT;

};

#endif
