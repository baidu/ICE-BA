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
#ifndef _IBA_H_
#define _IBA_H_

#include "IBA_datatype.h"
#include <functional>  // for function
#include <string>
#include <vector>

class LocalBundleAdjustor;
class GlobalBundleAdjustor;
class ViewerIBA;

namespace IBA {

class Solver {
 public:
  typedef std::function<void(const int, const float)> IbaCallback;

  Solver();
  ~Solver();

  // The following four functions must be called by main thread
  void Create(const Calibration &K, const int serial = IBA_SERIAL_NONE,
              const int verbose = IBA_VERBOSE_NONE, const int debug = IBA_DEBUG_NONE,
              const int history = IBA_HISTORY_NONE, const std::string param = "",
              const std::string dir = "", const std::vector<CameraIMUState> *XsGT = NULL,
              const std::vector<Depth> *dsGT = NULL);
  void Destroy();
  void Start();
  void Stop();

  /*
   * @brief Call for each current frame.
   * @param[in] CF current frame
   * @param[in] KF the key frame we want to push (optional)
   */
  void PushCurrentFrame(const CurrentFrame &CF, const KeyFrame *KF = NULL,
                        const bool serial = false);
  void PushKeyFrame(const KeyFrame &KF, const bool serial = false);

  /*
   * @brief Add relative constraint between two keyframe poses
   * @param[in] Z relative constraint
   */
  bool PushRelativeConstraint(const RelativeConstraint &Z);

#ifdef CFG_GROUND_TRUTH
  void PushIMUMeasurementsGT(const CurrentFrame &CF);
  void EstimateMotionGT(std::vector<CameraIMUState> *XsGT);
  void PushDepthMeasurementsGT(const CurrentFrame &CF, const KeyFrame *KF = NULL,
                               const bool keyframeOnly = false);
  void TriangulateDepthsGT(std::vector<Depth> *dsGT);
#endif

  /*
   * @brief Set callback function that will be triggered after LBA/GBA finishes
   */
  void SetCallbackLBA(const IbaCallback& iba_callback);
  void SetCallbackGBA(const IbaCallback& iba_callback);

  /*
   * @brief Call for each current frame to synchronize the optimization results
   * @param[out] SW The sliding window of interested
   */
  bool GetSlidingWindow(SlidingWindow *SW);
  void PrintSlidingWindow(const SlidingWindow &SW);

  int GetKeyFrames();
  int GetKeyFrameIndex(const int iKF);
  int SearchKeyFrame(const int iFrm);
  bool DeleteKeyFrame(const int iFrm);
  void GetMapPointIndexes(std::vector<int> *idxs);
  void DeleteMapPoints(const std::vector<int> &idxs);
  void UpdateCameras(const std::vector<int> &iFrms, const std::vector<CameraPose> &Cs,
                     const bool serial = false);

  bool PropagateState(const std::vector<IMUMeasurement> &us, const float t1, const float t2,
                      const CameraIMUState &X1, CameraIMUState *X2,
                      CameraPoseCovariance *S = NULL);

  bool SaveFeatures(const std::string fileName, const std::vector<MapPointMeasurement> &zs,
                    const std::vector<MapPoint> *Xs = NULL);
  bool LoadFeatures(const std::string fileName, const int iFrm,
                    std::vector<MapPointMeasurement> *zs,
                    std::vector<MapPoint> *Xs = NULL,
                    const std::vector<int> *iFrms = NULL
#ifdef CFG_STEREO
                  , const ubyte right = 0
#endif
                  );
  void SaveB(FILE *fp);
  void LoadB(FILE *fp);

  void ComputeErrorLBA(Error *e);
  void ComputeErrorGBA(Error *e);
  float ComputeRMSELBA();
  float ComputeRMSEGBA();
  float GetTotalDistance();
  bool SaveCamerasLBA(const std::string fileName, const bool append = true, const bool poseOnly = true);
  bool SaveCamerasGBA(const std::string fileName, const bool append = true, const bool poseOnly = true);
  bool SaveCostsLBA(const std::string fileName, const bool append = true, const int type = 0);
  bool SaveCostsGBA(const std::string fileName, const bool append = true, const int type = 0);
  bool SaveResidualsLBA(const std::string fileName, const bool append = true, const int type = 0);
  bool SaveResidualsGBA(const std::string fileName, const bool append = true, const int type = 0);
  bool SavePriors(const std::string fileName, const bool append = true, const int type = 0);
  bool SaveMarginalizations(const std::string fileName, const bool append = true, const int type = 0);
  bool SavePointsLBA(const std::string fileName, const bool append = true);
  bool SavePointsGBA(const std::string fileName, const bool append = true);
  void GetTimeLBA(Time *t);
  void GetTimeGBA(Time *t);
  bool SaveTimesLBA(const std::string fileName, const bool append = true);
  bool SaveTimesGBA(const std::string fileName, const bool append = true);

 protected:

  friend LocalBundleAdjustor;
  friend GlobalBundleAdjustor;
  friend ViewerIBA;
  class Internal *m_internal;
};

extern void LoadParameters(const std::string fileName);
extern bool SaveCalibration(const std::string fileName, const Calibration &K);
extern bool LoadCalibration(const std::string fileName, Calibration *K);
extern bool LoadCalibration(const std::string fileName, float T[3][4], Intrinsic *K,
                            float *ba = NULL, float *bw = NULL/*, float *sa = NULL*/);
extern void PrintCalibration(const Calibration &K);
extern bool SaveGroundTruth(const std::string fileName, const std::vector<CameraIMUState> &XsGT);
extern bool LoadGroundTruth(const std::string fileName, std::vector<CameraIMUState> *XsGT);
extern bool SaveGroundTruth(const std::string fileName, const std::vector<Depth> &dsGT);
extern bool LoadGroundTruth(const std::string fileName, std::vector<Depth> *dsGT);
extern bool SaveKeyFrames(const std::string fileName, const std::vector<int> &iFrms);
extern bool LoadKeyFrames(const std::string fileName, std::vector<int> *iFrms);
extern bool LoadKeyFrames(const std::string fileName, const std::vector<float> &ts,
                          std::vector<ubyte> *kfs, const float dtMax = 0.0f);
extern bool SaveMapPoints(const std::string fileName, const std::vector<int> &idxs);
extern bool LoadMapPoints(const std::string fileName, std::vector<int> *idxs);
extern bool SaveFeatureTracks(const std::string fileName, const std::vector<FeatureTrack> &xs);
extern bool LoadFeatureTracks(const std::string fileName, std::vector<FeatureTrack> *xs);
extern bool SaveCurrentFrame(const std::string fileName, const CurrentFrame &CF,
                             const KeyFrame &KF);
extern bool LoadCurrentFrame(const std::string fileName, CurrentFrame *CF, KeyFrame *KF);
extern bool LoadCurrentFrameTime(const std::string fileName, double *t);
extern bool LoadRelativeConstraints(const std::string fileName, const std::vector<float> &ts,
                                    std::vector<RelativeConstraint> *Zs, const float dtMax = 0.0);

extern void PrintCameraPose(const int iFrm, const CameraPose &C, const bool n = false);
extern void SaveCameraPose(const int iFrm, const CameraPose &C, FILE *fp);
extern bool SaveCameraPoses(const std::string fileName, const std::vector<int> &iFrms,
                            const std::vector<CameraPose> &Cs);
extern bool LoadCameraPoses(const std::string fileName, std::vector<int> *iFrms,
                            std::vector<CameraPose> *Cs);
extern void PrintPoints(const std::vector<Point3D> &Xs);
extern bool AssertError(const int iFrm, const Error &e);
extern void PrintError(const std::string str, const Error &e);

}  // namespace IBA

#endif  // _IBA_H_
