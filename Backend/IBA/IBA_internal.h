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
#ifndef _IBA_INTERNAL_H_
#define _IBA_INTERNAL_H_

#include "IBA.h"
#include "LocalBundleAdjustor.h"
#include "GlobalBundleAdjustor.h"

class ViewerIBA;

namespace IBA {

class Internal {

 public:

  class FeatureMeasurement {
   public:
    inline bool operator < (const FeatureMeasurement &z) const {
      return m_id < z.m_id
#ifdef CFG_STEREO
          || (m_id == z.m_id && !m_right && z.m_right)
#endif
        ;
    }
    inline bool operator < (const int id) const {
      return m_id < id;
    }
   public:
    int m_id;
    ::Point2D m_z;
    LA::SymmetricMatrix2x2f m_W;
    ubyte m_right;
  };

  class MapPointIndex {
   public:
    inline MapPointIndex() {}
    inline MapPointIndex(const int iFrm, const int idx, const int ix) :
                         m_iFrm(iFrm), m_idx(idx), m_ix(ix) {}
    inline bool operator < (const MapPointIndex &idx) const {
      return m_iFrm < idx.m_iFrm || (m_iFrm == idx.m_iFrm && m_idx < idx.m_idx);
    }
    inline void Set(const int iFrm, const int idx, const int ix) {
      m_iFrm = iFrm;
      m_idx = idx;
      m_ix = ix;
    }
   public:
    int m_iFrm, m_idx, m_ix;
  };

 public:

  void* operator new(std::size_t count) {
    // [NOTE] : we don't use count here, count is equal to sizeof(Internal)
    return SIMD::Malloc<Internal>();
  }

  void operator delete(void* p) {
    SIMD::Free<Internal>(static_cast<Internal*>(p));
  }

 protected:

  const LocalBundleAdjustor::InputLocalFrame& PushCurrentFrame(const CurrentFrame &CF);
  const GlobalMap::InputKeyFrame& PushKeyFrame(const KeyFrame &KF, const Camera *C = NULL);
  void ConvertFeatureMeasurements(const std::vector<MapPointMeasurement> &zs, FRM::Frame *F);
#ifdef CFG_GROUND_TRUTH
  void PushDepthMeasurementsGT(const FRM::Frame &F);
#endif

  void AssertConsistency();

 protected:

  friend Solver;
  friend LocalBundleAdjustor;
  friend GlobalBundleAdjustor;
  friend ViewerIBA;

  LocalMap m_LM;
  GlobalMap m_GM;
  LocalBundleAdjustor m_LBA;
  GlobalBundleAdjustor m_GBA;
  int m_nFrms;
  std::vector<FRM::Tag> m_Ts;
  std::vector<int> m_iFrmsKF, m_iKF2d, m_id2X, m_iX2d, m_id2idx, m_idx2iX;
  AlignedVector<Rigid3D> m_CsKF;
  std::vector<::Point2D> m_xs;
  std::vector<::Depth::InverseGaussian> m_ds;
#ifdef CFG_GROUND_TRUTH
  std::vector<::Depth::InverseGaussian> m_dsGT;
  AlignedVector<LA::AlignedVector3f> m_tsGT;
  std::vector<std::vector<::Depth::Measurement> > m_zsGT;
  AlignedVector<Rotation3D> m_RsGT;
  AlignedVector<float> m_work;
#endif

  Camera::Calibration m_K;
  ::Intrinsic::UndistortionMap m_UM;
#ifdef CFG_STEREO
  ::Intrinsic::UndistortionMap m_UMr;
#endif
  AlignedVector<Camera> m_CsGT;
  std::vector<::Depth::InverseGaussian> m_DsGT;
  std::string m_dir;

  std::vector<FeatureMeasurement> m_zsSortTmp;
  std::vector<FTR::Measurement> m_zsTmp;
  std::vector<MapPointIndex> m_idxsSortTmp;
  std::vector<int> m_idxsTmp;

  LocalBundleAdjustor::InputLocalFrame m_ILF;
  GlobalMap::InputKeyFrame m_IKF;

  std::list<LocalMap::CameraLF> m_CsLF;
  std::vector<ubyte> m_ucsKF, m_uds;

  std::vector<GlobalBundleAdjustor::InputCamera> m_ICs;
};

}

#endif
