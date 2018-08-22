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
#ifndef _VIEWER_IBA_H_
#define _VIEWER_IBA_H_

#include "Viewer.h"
#include "Frustrum.h"
#include "Arcball.h"
#include "Keyboard.h"
#include "IBA.h"
#include "LocalBundleAdjustor.h"
#include "GlobalBundleAdjustor.h"
#include "UtilityWM.h"

class ViewerIBA : public Viewer {

 public:

  virtual void Create(IBA::Solver *solver, const std::string screenFile = "",
                      const int screenCombine = false, const std::string saveFile = "",
                      const int iFrmSave = -1, const bool wnd = true);
  virtual void Reset();
  virtual bool Run(const bool visualize = true, const bool step = true, const int iFrm = -1);
  virtual int Start(const std::string viewFile = "", const bool pause = false);
  virtual void Stop(const std::string viewFile = "", const bool pause = false);
  virtual void DeleteKeyFrame(const int iFrm);

 protected:

  enum DrawViewType { DRAW_VIEW_2D, DRAW_VIEW_3D, DRAW_VIEW_PROFILE, DRAW_VIEW_TYPES };
  enum DrawTimeLineType { DRAW_TLN_NONE, DRAW_TLN_FEATURE_MATCH, DRAW_TLN_PRIOR };
  enum DrawCameraTypeKF { DRAW_CAM_KF_LBA, DRAW_CAM_KF_GBA,
#ifdef CFG_GROUND_TRUTH
                          DRAW_CAM_KF_GT,
#endif
                          DRAW_CAM_KF_TYPES
                        };
  enum DrawCameraTypeLF { DRAW_CAM_LF_LBA,
#ifdef CFG_GROUND_TRUTH
                          DRAW_CAM_LF_GT,
#endif
                          DRAW_CAM_LF_TYPES
                        };
  enum DrawDepthType { DRAW_DEP_LBA, DRAW_DEP_GBA,
#ifdef CFG_GROUND_TRUTH
                       DRAW_DEP_GT,
#endif
                       DRAW_DEP_TYPES
                     };
  enum DrawFeatureType { DRAW_FTR_NONE, DRAW_FTR_SOURCE_MEASUREMENT, DRAW_FTR_MATCH,
                         DRAW_FTR_TYPES };
  enum DrawProjectionType { DRAW_PRJ_NONE, DRAW_PRJ_TRACKED, DRAW_PRJ_UNTRACKED,
                            DRAW_PRJ_ALL, DRAW_PRJ_TYPES };
  enum DrawMotionTypeLF { DRAW_MOT_LF_NONE, DRAW_MOT_LF_TRAJECTORY,
                          DRAW_MOT_LF_TRAJECTORY_NEIGHBOR, DRAW_MOT_LF_TYPES };
  enum DrawMotionTypeKF { DRAW_MOT_KF_NONE, DRAW_MOT_KF_TRAJECTORY, DRAW_MOT_KF_POSE,
                          DRAW_MOT_KF_POSE_NEIGHBOR, DRAW_MOT_KF_TYPES };
  enum DrawStructureType { DRAW_STR_NONE, DRAW_STR_ACTIVE, DRAW_STR_CONVERGED, DRAW_STR_NOT_CONVERGED, DRAW_STR_ALL, DRAW_STR_TYPES };
  enum DrawErrorType { DRAW_ERR_NONE, DRAW_ERR_MEAN, DRAW_ERR_COVARIANCE, DRAW_ERR_ALL, DRAW_ERR_TYPES };
  enum DrawBackgroundColor { DRAW_BG_BLACK, DRAW_BG_WHITE, DRAW_BG_COLORS };
  enum DrawProfileType { DRAW_PRF_ACCELERATION, DRAW_PRF_ACCELERATION_DEVICE,
                         DRAW_PRF_GYROSCOPE, DRAW_PRF_GYROSCOPE_DEVICE,
                         DRAW_PRF_IMU_DELTA_ROTATION_STATE,
                         DRAW_PRF_IMU_DELTA_ROTATION_MEASUREMENT,
                         DRAW_PRF_IMU_DELTA_ROTATION_ERROR,
                         DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE,
                         DRAW_PRF_IMU_DELTA_POSITION_STATE,
                         DRAW_PRF_IMU_DELTA_POSITION_MEASUREMENT,
                         DRAW_PRF_IMU_DELTA_POSITION_ERROR,
                         DRAW_PRF_IMU_DELTA_POSITION_COVARIANCE,
                         DRAW_PRF_IMU_DELTA_VELOCITY_STATE,
                         DRAW_PRF_IMU_DELTA_VELOCITY_MEASUREMENT,
                         DRAW_PRF_IMU_DELTA_VELOCITY_ERROR,
                         DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE,
                         DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE,
                         DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_COVARIANCE,
                         DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE,
                         DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE,
                         DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE,
                         DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT,
                         DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR,
                         DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE,
                         DRAW_PRF_CAMERA_PRIOR_POSITION_STATE,
                         DRAW_PRF_CAMERA_PRIOR_POSITION_MEASUREMENT,
                         DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR,
                         DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE,
                         DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE,
                         DRAW_PRF_CAMERA_PRIOR_VELOCITY_MEASUREMENT,
                         DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR,
                         DRAW_PRF_CAMERA_PRIOR_VELOCITY_COVARIANCE,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_MEASUREMENT,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_ERROR,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_MEASUREMENT,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_ERROR,
                         DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE,
                         DRAW_PRF_REPROJECTION_ERROR,
                         DRAW_PRF_STATE_ROTATION_ABSOLUTE,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_ROTATION_ABSOLUTE_ERROR,
#endif
                         DRAW_PRF_STATE_ROTATION_RELATIVE,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR,
#endif
                         DRAW_PRF_STATE_POSITION_ABSOLUTE,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_POSITION_ABSOLUTE_ERROR,
#endif
                         DRAW_PRF_STATE_POSITION_RELATIVE,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_POSITION_RELATIVE_ERROR,
#endif
                         DRAW_PRF_STATE_VELOCITY,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_VELOCITY_ERROR,
#endif
                         DRAW_PRF_STATE_BIAS_ACCELERATION,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_BIAS_ACCELERATION_ERROR,
#endif
                         DRAW_PRF_STATE_BIAS_GYROSCOPE,
#ifdef CFG_GROUND_TRUTH
                         DRAW_PRF_STATE_BIAS_GYROSCOPE_ERROR,
#endif
                         DRAW_PRF_TYPES
                       };
  enum DrawAxisType { DRAW_AXIS_NONE, DRAW_AXIS_WORLD, DRAW_AXIS_WORLD_AND_CAMERA, DRAW_AXIS_TYPES };
  class FeatureIndex {
   public:
    inline bool Valid() const { return m_ix.Valid(); }
    inline bool Invalid() const { return m_ix.Invalid(); }
    inline void Invalidate() { m_ix.Invalidate(); m_iz.Invalidate(); m_ic.Invalidate(); }
    inline bool operator == (const FeatureIndex &iFtr) const { return m_ix == iFtr.m_ix && m_iz == iFtr.m_iz && m_ic == iFtr.m_ic; }
   public:
    class Source {
     public:
      inline Source() {}
      inline Source(const int iKF, const int ix) : m_iKF(iKF), m_ix(ix) {}
      inline bool operator == (const Source &ix) const { return m_iKF == ix.m_iKF && m_ix == ix.m_ix; }
      inline bool operator != (const Source &ix) const { return m_iKF != ix.m_iKF || m_ix != ix.m_ix; }
      inline void Set(const int iKF, const int ix) { m_iKF = iKF; m_ix = ix; }
      inline bool Equal(const int iKF, const int ix) const { return m_iKF == iKF && m_ix == ix; }
      inline bool Valid() const { return m_iKF != -1; }
      inline bool Invalid() const { return m_iKF == -1; }
      inline void Invalidate() { m_iKF = m_ix = -1; }
     public:
      int m_iKF, m_ix;
    } m_ix;
    class Measurement {
     public:
      inline bool operator == (const Measurement &iz) const { return m_iFrm == iz.m_iFrm && m_iKF == iz.m_iKF && m_iLF == iz.m_iLF && m_iz == iz.m_iz; }
      inline bool operator != (const Measurement &iz) const { return m_iFrm != iz.m_iFrm || m_iKF != iz.m_iKF || m_iLF != iz.m_iLF || m_iz != iz.m_iz; }
      inline void SetKF(const int iFrm, const int iKF, const int iz) { m_iFrm = iFrm; m_iKF = iKF; m_iLF = -1; m_iz = iz; }
      inline void SetLF(const int iFrm, const int iLF, const int iz) { m_iFrm = iFrm; m_iKF = -1; m_iLF = iLF; m_iz = iz; }
      inline bool EqualKF(const int iKF, const int iz) const { return m_iKF == iKF && m_iz == iz; }
      inline bool EqualLF(const int iLF, const int iz) const { return m_iLF == iLF && m_iz == iz; }
      inline bool Valid() const { return m_iFrm != -1 && m_iz != -1; }
      inline bool Invalid() const { return m_iFrm == -1 || m_iz == -1; }
      inline void Invalidate() { m_iFrm = m_iKF = m_iLF = m_iz = -1; }
      inline void AssertConsistency() const { UT_ASSERT(Invalid() || m_iKF != -1 && m_iLF == -1 || m_iKF == -1 && m_iLF != -1); }
     public:
      int m_iFrm, m_iKF, m_iLF, m_iz;
    } m_iz;
    class Current {
     public:
      inline bool operator == (const Current &ic) const { return m_iFrm == ic.m_iFrm && m_iz == ic.m_iz; }
      inline bool operator != (const Current &ic) const { return m_iFrm != ic.m_iFrm || m_iz != ic.m_iz; }
      inline void operator = (const Measurement &iz) { m_iFrm = iz.m_iFrm; m_iz = iz.m_iz; }
      inline void Set(const int iFrm, const int iz) { m_iFrm = iFrm; m_iz = iz; }
      inline bool Equal(const int iFrm, const int iz) const { return m_iFrm == iFrm && m_iz == iz; }
      inline bool Valid() const { return m_iFrm != -1; }
      inline bool Invalid() const { return m_iFrm == -1; }
      inline void Invalidate() { m_iFrm = m_iz = -1; }
     public:
      int m_iFrm, m_iz;
    } m_ic;
  };

 protected:

  virtual int GetKeyFrames();
  virtual int GetLocalFrames();
  virtual const FRM::Frame* GetKeyFrame(const int iKF);
  virtual const FRM::Frame* GetLocalFrame(int iLF);
  virtual Rigid3D GetCameraKF(const int iKF, const int type = -1
#ifdef CFG_STEREO
                            , const bool right = false
#endif
                            );
  virtual Camera GetCameraLF(const int iLF, const int type = -1
#ifdef CFG_STEREO
                           , const bool right = false
#endif
                           );
  virtual Camera GetCameraGBALM(const int iKF, const int type = -1);
  virtual IMU::Delta GetIMUDeltaLF(const int iLF, const int type = -1);
  virtual IMU::Delta GetIMUDeltaGBALM(const int iKF, const int type = -1);
  virtual Depth::InverseGaussian GetFrameDepthKF(const int iKF, const int type = -1);
  virtual Depth::InverseGaussian GetFrameDepthLF(const int iLF, const int type = -1);
  virtual Depth::InverseGaussian GetFeatureDepth(const int iKF, const int ix, const int type = -1);

  virtual void UpdateCurrentFrame();
  virtual void ActivateFrame(const int iFrmActive);
  virtual void ActivateLocalFrame(const int iLFActive);
  virtual bool DragActiveFrame(const CVD::ImageRef &from, const CVD::ImageRef &to);
  virtual void ActivateFeature(const FeatureIndex::Source &ix, const int iz = -1);
  virtual bool SearchActiveFeature(const CVD::ImageRef &x);
  virtual void SearchActiveFeature(const Point2D &x, const int iKF = -1, int ix = -1,
                                   const int iz = -1);
  virtual void SearchActiveFeatureReset();
  virtual void SearchActiveFeatureStart();
  virtual void SearchActiveFeatureStop();
  virtual void PrintActiveFeatureDepth();
  virtual void PrintActiveFeatureTracks();
  virtual void PrintFeatureTrack(const int iFrm, const int iz, const Intrinsic &K,
                                 const Point2D &z, const Point2D &zp,
                                 const LA::SymmetricMatrix2x2f &W,
                                 const std::string str = " ");
  virtual void PrintFeatureTrackKF(const int iKFx, const int ix, const int iKFz,
                                   const std::string str = " ");
  virtual void PrintFeatureTrackLF(const int iKFx, const int ix, const int iLFz,
                                   const std::string str = " ");
#ifdef CFG_STEREO
  virtual void ActivateCamera(const bool rightActive);
#endif

  virtual int CountFeatureMatchesSource(const int iKF1, const FRM::Frame &F2);
  virtual int CountFeatureMatchesMeasurement(const FRM::Frame &F1, const FRM::Frame &F2);
  virtual int CountFeatureMatchesKFKF(const int iKF1, const int iKF2);
  virtual int CountFeatureMatchesKFLF(const int iKF1, const int iLF2);
  virtual int CountFeatureMatchesLFLF(const int iLF1, const int iLF2);

  virtual void Draw(const bool swapBuffers = true);
  virtual void DrawString();
  virtual void DrawString(const std::string str, const int t, const bool r,
                          LA::AlignedVector3f &v, LA::AlignedMatrix3x3f &S,
                          int *rowStart = NULL);
  virtual void DrawString(const CameraPrior::Pose &Zp, const LA::AlignedMatrixXf &S,
                          const LA::AlignedVectorXf &x, int *rowStart = NULL);
  virtual void DrawTimeLine();
  virtual int DrawTimeLine(std::vector<float> *alphas, const float alphaMin = 0.0f,
                           const float alphaMax = 1.0f, const bool localFrm = true);
  virtual void DrawTimeLine(const CameraPrior::Pose &Zp, const LA::AlignedMatrixXf &S,
                            std::vector<float> *alphas, const float alphaMin = 0.0f,
                            const float alphaMax = 1.0f);

  virtual void Draw2DImage();
  virtual void Draw2DFeatures();
  virtual void Draw2DFeature(const FeatureIndex::Source &ix, const int iz = -1);
  virtual void Draw2DProjections();
  virtual void Draw2DProjection(const int iKF, const int ix, const int iz = -1);
  virtual void Get2DProjectionRange(const Point2D &x, const LA::Vector2f &Jx, const float s2,
                                    Point2D *xs);
  virtual void Update2DImageTexture();

  virtual void Draw3DCameras();
  virtual void Draw3DCameras(const std::vector<float> &alphas, const float alphaMin = 0.0f,
                             const int iKFr = -1);
  virtual void Draw3DPoints();
  virtual void Draw3DWorldAxis();
  virtual void Draw3DDepthPlane();
  virtual void Draw3DDepthPlane(const Rigid3D &C, const float d, Point3D *Xs = NULL);
  virtual void Draw3DObjects();
  virtual void Reset3DProjectionMatrix();
  virtual void Reset3DModelViewMatrix();
  virtual void Reset3DViewPoint(const bool active = true);
  virtual void Update3DViewPoint();
  virtual void Update3DBackgroundColor();
  virtual float ComputeMeanDepth(const Rigid3D &C);
  virtual void UnProjectWindowPoint(const CVD::ImageRef &x, Point3D &X);
  virtual void UnProjectWindowPoint(const CVD::ImageRef &x, Point3D &X, const float z);
  virtual float UnProjectWindowSize(const float z);

  virtual void DrawProfile();
  virtual void DrawProfile(const CameraPrior::Pose &Zp, const float dx, const float ratio,
                           const float yb, const float b);
  virtual void DrawProfile(const CameraPrior::Pose &Zp, const float dx, const float ratio,
                           const float yb, const float sv, const LA::Vector2f &size,
                           const LA::AlignedVectorXf &x, const LA::AlignedMatrixXf &S);
  virtual bool DrawProfileTypeValid(const int iFrm = -1);
  virtual void DrawProfileTypeStep(const int *types, const int N, const bool next);
  virtual int GetDrawProfileScaleType();
  virtual UT::ES<FTR::ESError, FTR::ESIndex> ComputeReprojectionError(const int iKF, const int iLF);

  virtual bool OnKeyDown(const int key);
  virtual void OnMouseMove(const CVD::ImageRef &where);
  virtual void OnMouseDown(const CVD::ImageRef &where, const int button);
  virtual void OnMouseUp(const CVD::ImageRef &where, const int button);
  virtual bool OnMouseDraging(const CVD::ImageRef &from, const CVD::ImageRef &to, const int button);
  virtual bool OnMouseDoubleClicked(const CVD::ImageRef &where, const int button);
  virtual void OnResize(const CVD::ImageRef &size);
  virtual void SaveB(FILE *fp);
  virtual void LoadB(FILE *fp);
  virtual void SaveScreen(const std::string fileName, const bool sizeImg = true);

 protected:

  friend LocalBundleAdjustor;
  friend GlobalBundleAdjustor;

  typedef LocalBundleAdjustor::LocalFrame LocalFrame;
  typedef LocalBundleAdjustor::KeyFrame KeyFrame;
  //typedef GlobalBundleAdjustor::KeyFrame KeyFrame;

  IBA::Solver *m_solver;
  LocalBundleAdjustor *m_LBA;
  GlobalBundleAdjustor *m_GBA;
  Camera::Calibration m_K;
  BoundingBox2D m_B, m_Bn;
  Intrinsic::RectificationMap m_RM;
#ifdef CFG_STEREO
  Intrinsic::RectificationMap m_RMr;
#endif
  std::string m_fileNameDir, m_fileNameScreen, m_fileNameSave;
  int m_screenCombine, m_iFrmSave;

  int m_iLF, m_iFrm, m_iKFActive, m_iLFActive, m_iLFActiveLast, m_iFrmActive, m_iFrmActiveLast;
#ifdef CFG_STEREO
  bool m_rightActive, m_rightActiveLast;
#endif
  FeatureIndex m_iFtrActive;
  std::vector<int> m_iFrmsKF, m_iKF2d;
  std::vector<LA::Vector3ub> m_clrsKF;

  LA::Vector2f m_factorWinToImg, m_factorImgToWin, m_sizeCross, m_sizeBox;

  bool m_dragingActiveFrm, m_dragingActiveFtr, m_drawPch;
  CVD::Image<ubyte> m_imgIT;
  CVD::Image<CVD::Rgb<ubyte> > m_imgRGB, m_imgRGBScreen, m_imgRGBTmp;
#ifdef CFG_STEREO
  CVD::Image<CVD::Rgb<ubyte> > m_imgRGBr;
#endif
  TextureGL3 m_texRGB;
#ifdef CFG_STEREO
  TextureGL3 m_texRGBr;
#endif
  int m_iFrmTexRGB;
#ifdef CFG_STEREO
  int m_iFrmTexRGBr;
#endif

  std::vector<float> m_alphas;
  std::vector<std::vector<int> > m_iX2z;
  LA::AlignedVectorXf m_work;
  std::vector<bool> m_marks;

  Frustrum m_frustrum, m_frustrumActive;
  double m_projMatrix[16], m_projMatrixPano[16], m_modelMatrix[16], m_modelMatrixScale[16];
  int m_viewport[4];
  LA::AlignedVector3f m_translation, m_translationStart;
  Point3D m_whereMouseDown3DStart;
  Arcball m_arcball;
  Rigid3D m_Cv;
  float m_factorWinToTranslationZ;
  LA::AlignedMatrix4x4f m_CT;

  Point2D m_xSearch;
  FeatureIndex::Source m_ixSearch, m_ixSearchStart;
  int m_izSearch;
  float m_d2MinSearch;

  Keyboard::Switch m_keyDrawViewType, m_keyDrawViewTypeBkp;
  Keyboard::Binary m_keyPause, m_keyStep;
  Keyboard::Switch m_keyDrawCamTypeKF, m_keyDrawCamTypeLF, m_keyDrawDepType, m_keyDrawDepTypeCMP;
  Keyboard::Binary m_keyDrawString;
  Keyboard::Switch m_keyDrawTlnType;
  Keyboard::Scalar m_keyDrawTlnMaxFtrMatches, m_keyDrawTlnPriorVarPos, m_keyDrawTlnPriorVarRot;

  Keyboard::Switch m_keyDrawPrfType;
  Keyboard::Scalar m_keyDrawPrfScale[DRAW_PRF_TYPES];

  Keyboard::Switch m_keyDrawAxis;
  Keyboard::Scalar m_keyDrawAxisLen;
  Keyboard::Binary m_keyDrawDepPlane;
  Keyboard::Scalar m_keyDrawDepVar;
  Keyboard::Scalar m_keyDrawCovProb, m_keyDrawCovScale;

  Keyboard::Switch m_keyDrawFtrType;
  Keyboard::Scalar m_keyDrawPchSize;
  Keyboard::Switch m_keyDrawPrjType;
  Keyboard::Switch m_keyDrawErrType2D;

  Keyboard::Switch m_keyDrawMotTypeLF, m_keyDrawMotTypeKF, m_keyDrawStrType;
  Keyboard::Scalar m_keyDrawCamSize;
  Keyboard::Binary m_keyDrawCamVelocity, m_keyDrawCamTex;
#ifdef CFG_GROUND_TRUTH
  Keyboard::Binary m_keyDrawCamGT;
#endif
  Keyboard::Switch m_keyDrawErrType3D, m_keyDrawBgClr;

};

#endif
