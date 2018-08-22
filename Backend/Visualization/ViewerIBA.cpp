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
#include "ViewerIBA.h"
#include "IBA_internal.h"
//#ifndef CFG_DEBUG
//#define CFG_DEBUG
//#endif

void ViewerIBA::Create(IBA::Solver *solver, const std::string screenFile, const int screenCombine,
                       const std::string saveFile, const int iFrmSave, const bool wnd) {
  m_solver = solver;
  m_LBA = &solver->m_internal->m_LBA;
  m_GBA = &solver->m_internal->m_GBA;
  m_K = m_LBA->m_K;
  const int w = m_K.m_K.w(), h = m_K.m_K.h();
  m_B.m_xMin.Set(0.0f, 0.0f);
  m_B.m_xMax.Set(float(w - 1), float(h - 1));
  m_K.m_K.ImageToNormalized(m_B.m_xMin, m_Bn.m_xMin);
  m_K.m_K.ImageToNormalized(m_B.m_xMax, m_Bn.m_xMax);
  m_fileNameDir = solver->m_internal->m_dir;
  m_fileNameScreen = UT::FileNameReplaceDirectory(screenFile, ".", m_fileNameDir);
  if (m_fileNameScreen != "" && UT::FileNameExtractSuffix<int>(m_fileNameScreen) != -1) {
    UT::FilesStartSaving(m_fileNameScreen);
  }
  m_screenCombine = screenCombine;
  m_fileNameSave = UT::FileNameReplaceDirectory(saveFile, ".", m_fileNameDir);
  m_iFrmSave = iFrmSave;
  if (wnd) {
    const float aspect = VM_WINDOW_ASPECT_RATIO == 0.0f ? float(w) / h : VM_WINDOW_ASPECT_RATIO;
    if (VW_WINDOW_WIDTH != 0 && VW_WINDOW_HEIGHT != 0) {
      Viewer::Initialize(VW_WINDOW_WIDTH, VW_WINDOW_HEIGHT);
    } else if (VW_WINDOW_WIDTH != 0 && VW_WINDOW_HEIGHT == 0) {
      Viewer::Initialize(VW_WINDOW_WIDTH, int(VW_WINDOW_WIDTH / aspect + 0.5f));
    } else if (VW_WINDOW_WIDTH == 0 && VW_WINDOW_HEIGHT != 0) {
      Viewer::Initialize(int(VW_WINDOW_HEIGHT * aspect + 0.5f), VW_WINDOW_HEIGHT);
    } else if (VW_WINDOW_WIDTH == 0 && VW_WINDOW_HEIGHT == 0) {
      Viewer::Initialize(w, h);
    }
  }

  m_keyDrawViewType.Set(DRAW_VIEW_2D, UT::Strings("DRAW_VIEW_2D",
                                                  "DRAW_VIEW_3D"/*, "DRAW_VIEW_PROFILE"*/), false);
  m_keyDrawViewTypeBkp = m_keyDrawViewType;
  m_keyPause = false;
  m_keyStep = false;
  m_keyDrawCamTypeKF.Set(DRAW_CAM_KF_GBA, UT::Strings("DRAW_CAM_KF_LBA / DRAW_CAM_LF_LBA / DRAW_DEP_LBA",
                                                      "DRAW_CAM_KF_GBA / DRAW_DEP_GBA"
#ifdef CFG_GROUND_TRUTH
                                                    , "DRAW_CAM_KF_GT / DRAW_CAM_LF_GT / DRAW_DEP_GT"
#endif
                                                    ));
  m_keyDrawCamTypeLF.Set(DRAW_CAM_KF_LBA, UT::Strings("DRAW_CAM_LF_LBA / DRAW_CAM_KF_LBA / DRAW_DEP_LBA"
#ifdef CFG_GROUND_TRUTH
                                                    , "DRAW_CAM_LF_GT / DRAW_CAM_KF_GT / DRAW_DEP_GT"
#endif
                                                    ));
  m_keyDrawDepType.Set(DRAW_DEP_LBA, UT::Strings("DRAW_DEP_LBA", "DRAW_DEP_GBA"
#ifdef CFG_GROUND_TRUTH
                                               , "DRAW_DEP_GT"
#endif
                                               ));
  m_keyDrawDepTypeCMP.Set(DRAW_DEP_LBA, UT::Strings("DRAW_DEP_CMP_LBA", "DRAW_DEP_CMP_GBA"
#ifdef CFG_GROUND_TRUTH
                                                  , "DRAW_DEP_CMP_GT"
#endif
                                                  ));
  m_keyDrawString = true;
  m_keyDrawTlnType.Set(DRAW_TLN_PRIOR, UT::Strings("DRAW_TLN_NONE", "DRAW_TLN_FEATURE_MATCH",
                                                   "DRAW_TLN_PRIOR"));
  m_keyDrawTlnMaxFtrMatches.Set(300, "DRAW_TLN_MAX_FEATURE_MATCHES", true, 50);
  m_keyDrawTlnPriorVarPos.Set(1.0e-5f, "DRAW_TLN_PRIOR_VARIANCE_POSITION", true/*, 1.0e-7f*/);
  m_keyDrawTlnPriorVarRot.Set(1.0e-2f, "DRAW_TLN_PRIOR_VARIANCE_ROTATION", true/*, 1.0e-5f*/);
  m_keyDrawPrfScale[DRAW_PRF_ACCELERATION].Set(10.0f, "DRAW_PRF_ACCELERATION");
  m_keyDrawPrfScale[DRAW_PRF_GYROSCOPE].Set(30.0f, "DRAW_PRF_GYROSCOPE");
  m_keyDrawPrfScale[DRAW_PRF_IMU_DELTA_ROTATION_STATE].Set(5.0f, "DRAW_PRF_IMU_DELTA_ROTATION");
  m_keyDrawPrfScale[DRAW_PRF_IMU_DELTA_POSITION_STATE].Set(0.05f, "DRAW_PRF_IMU_DELTA_POSITION");
  m_keyDrawPrfScale[DRAW_PRF_IMU_DELTA_VELOCITY_STATE].Set(0.5f, "DRAW_PRF_IMU_DELTA_VELOCITY");
  m_keyDrawPrfScale[DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE].Set(0.1f, "DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE");
  m_keyDrawPrfScale[DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE].Set(1.0f, "DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE");
  m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE].Set(5.0f, "DRAW_PRF_CAMERA_PRIOR_ROTATION");
  m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_POSITION_STATE].Set(0.05f, "DRAW_PRF_CAMERA_PRIOR_POSITION");
  m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE].Set(0.1f, "DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE");
  //m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE].Set(1.0f, "DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE");
  m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE].Set(1.0f, "DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE");
  m_keyDrawPrfScale[DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE].Set(10.0f, "DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE");
  m_keyDrawPrfScale[DRAW_PRF_REPROJECTION_ERROR].Set(10.0f, "DRAW_PRF_REPROJECTION_ERROR");
  m_keyDrawPrfScale[DRAW_PRF_STATE_ROTATION_ABSOLUTE].Set(30.0f, "DRAW_PRF_STATE_ROTATION");
  m_keyDrawPrfScale[DRAW_PRF_STATE_POSITION_ABSOLUTE].Set(1.0f, "DRAW_PRF_STATE_POSITION");
  m_keyDrawPrfScale[DRAW_PRF_STATE_VELOCITY].Set(1.0f, "DRAW_PRF_ERROR_VELOCITY");
  m_keyDrawPrfScale[DRAW_PRF_STATE_BIAS_ACCELERATION].Set(1.0f, "DRAW_PRF_STATE_BIAS_ACCELERATION");
  m_keyDrawPrfScale[DRAW_PRF_STATE_BIAS_GYROSCOPE].Set(10.0f, "DRAW_PRF_STATE_BIAS_GYROSCOPE");

  const std::string info[DRAW_PRF_TYPES] = {
    "DRAW_PRF_ACCELERATION", "DRAW_PRF_ACCELERATION_DEVICE",
    "DRAW_PRF_GYROSCOPE", "DRAW_PRF_GYROSCOPE_DEVICE",
    "DRAW_PRF_IMU_DELTA_ROTATION_STATE", "DRAW_PRF_IMU_DELTA_ROTATION_MEASUREMENT", "DRAW_PRF_IMU_DELTA_ROTATION_ERROR", "DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE",
    "DRAW_PRF_IMU_DELTA_POSITION_STATE", "DRAW_PRF_IMU_DELTA_POSITION_MEASUREMENT", "DRAW_PRF_IMU_DELTA_POSITION_ERROR", "DRAW_PRF_IMU_DELTA_POSITION_COVARIANCE",
    "DRAW_PRF_IMU_DELTA_VELOCITY_STATE", "DRAW_PRF_IMU_DELTA_VELOCITY_MEASUREMENT", "DRAW_PRF_IMU_DELTA_VELOCITY_ERROR", "DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE",
    "DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE", "DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_COVARIANCE",
    "DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE", "DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE",
    "DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE", "DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT", "DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR", "DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE",
    "DRAW_PRF_CAMERA_PRIOR_POSITION_STATE", "DRAW_PRF_CAMERA_PRIOR_POSITION_MEASUREMENT", "DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR", "DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE",
    "DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE", "DRAW_PRF_CAMERA_PRIOR_VELOCITY_MEASUREMENT", "DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR", "DRAW_PRF_CAMERA_PRIOR_VELOCITY_COVARIANCE",
    "DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE", "DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_MEASUREMENT",
    "DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_ERROR", "DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE",
    "DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE", "DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_MEASUREMENT",
    "DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_ERROR", "DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE",
    "DRAW_PRF_REPROJECTION_ERROR",
    "DRAW_PRF_STATE_ROTATION_ABSOLUTE"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_ROTATION_ABSOLUTE_ERROR"
#endif
  , "DRAW_PRF_STATE_ROTATION_RELATIVE"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR"
#endif
  , "DRAW_PRF_STATE_POSITION_ABSOLUTE"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_POSITION_ABSOLUTE_ERROR"
#endif
  , "DRAW_PRF_STATE_POSITION_RELATIVE"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_POSITION_RELATIVE_ERROR"
#endif
  , "DRAW_PRF_STATE_VELOCITY"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_VELOCITY_ERROR"
#endif
  , "DRAW_PRF_STATE_BIAS_ACCELERATION"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_BIAS_ACCELERATION_ERROR"
#endif
  , "DRAW_PRF_STATE_BIAS_GYROSCOPE"
#ifdef CFG_GROUND_TRUTH
  , "DRAW_PRF_STATE_BIAS_GYROSCOPE_ERROR"
#endif
  };
  m_keyDrawPrfType.Set(DRAW_PRF_ACCELERATION, UT::Strings(info, DRAW_PRF_TYPES));
  m_keyDrawAxis.Set(DRAW_AXIS_NONE, UT::Strings("DRAW_AXIS_NONE", "DRAW_AXIS_WORLD", "DRAW_AXIS_WORLD_AND_CAMERA"));
  m_keyDrawAxisLen.Set(1.0f, "DRAW_AXIS_LENGTH");
  m_keyDrawDepPlane = false;
  m_keyDrawDepVar.Set(DEPTH_VARIANCE_CONVERGE, "DRAW_DEPTH_VARIANCE", 1/*, 0.1f*/);
  m_keyDrawCovProb.Set(0.5f, "DRAW_COVARIANCE_PROBABILITY", true, 0.1f);
  m_keyDrawCovScale.Set(1.0f, "DRAW_COVARIANCE_SCALE");

  m_keyDrawFtrType.Set(DRAW_FTR_SOURCE_MEASUREMENT, UT::Strings("DRAW_FTR_NONE",
                                                                "DRAW_FTR_SOURCE_MEASUREMENT",
                                                                "DRAW_FTR_MATCH"));
  m_keyDrawPchSize.Set(1.0f, "DRAW_PATCH_SIZE");
  m_keyDrawPrjType.Set(DRAW_PRJ_TRACKED, UT::Strings("DRAW_PRJ_NONE", "DRAW_PRJ_TRACKED",
                                                     "DRAW_PRJ_UNTRACKED", "DRAW_PRJ_ALL"));
  m_keyDrawErrType2D.Set(DRAW_ERR_ALL, UT::Strings("DRAW_ERR_NONE", "DRAW_ERR_MEAN",
                                                   "DRAW_ERR_COVARIANCE", "DRAW_ERR_ALL"));

  m_keyDrawMotTypeLF.Set(DRAW_MOT_LF_TRAJECTORY, UT::Strings("DRAW_MOT_LF_NONE",
                                                             "DRAW_MOT_LF_TRAJECTORY",
                                                             "DRAW_MOT_LF_TRAJECTORY_NEIGHBOR"));
  m_keyDrawMotTypeKF.Set(DRAW_MOT_KF_TRAJECTORY, UT::Strings("DRAW_MOT_KF_NONE",
                                                             "DRAW_MOT_KF_TRAJECTORY",
                                                             "DRAW_MOT_KF_POSE",
                                                             "DRAW_MOT_KF_POSE_NEIGHBOR"));
  m_keyDrawStrType.Set(DRAW_STR_ACTIVE, UT::Strings("DRAW_STR_NONE", "DRAW_STR_ACTIVE",
                                                    "DRAW_STR_CONVERGED", "DRAW_STR_NOT_CONVERGED", "DRAW_STR_ALL"));
  m_keyDrawCamVelocity = false;
  m_keyDrawCamTex = false;
#ifdef CFG_GROUND_TRUTH
  m_keyDrawCamGT = true;
#endif
  m_keyDrawCamSize.Set(1.0f, "DRAW_CAM_SIZE");
  m_keyDrawErrType3D.Set(DRAW_ERR_ALL, UT::Strings("DRAW_ERR_NONE", "DRAW_ERR_MEAN",
                                                   "DRAW_ERR_COVARIANCE", "DRAW_ERR_ALL"));
  m_keyDrawBgClr.Set(DRAW_BG_BLACK, UT::Strings("DRAW_BG_BLACK", "DRAW_BG_WHITE"), false);

//#ifndef WIN32
#if 1
  if (m_pWnd)
#endif
  {
    m_texRGB.Generate(w, h, GL_LINEAR);
#ifdef CFG_STEREO
    m_texRGBr.Generate(w, h, GL_LINEAR);
#endif
    glEnable(GL_CULL_FACE);
  }
  Reset();
}

void ViewerIBA::Reset() {
  //m_iLF = -1;
  m_iLF = m_LBA->m_ic2LF.empty() ? -1 : m_LBA->m_ic2LF.back();
  m_iFrm = m_iLF == -1 ? -1 : GetLocalFrame(m_iLF)->m_T.m_iFrm;
  m_iKFActive = m_iLFActive = m_iLFActiveLast = m_iFrmActive = m_iFrmActiveLast = -1;
#ifdef CFG_STEREO
  m_rightActive = m_rightActiveLast = false;
#endif
  m_iFtrActive.Invalidate();

  //const std::vector<LocalMap::CameraKF> &CsKF = m_solver->m_internal->m_CsKF;
  //const int nKFs = static_cast<int>(CsKF.size());
  //m_iFrmsKF.resize(nKFs);
  //for (int iKF = 0; iKF < nKFs; ++iKF) {
  //  m_iFrmsKF[iKF] = CsKF[iKF].m_iFrm;
  //}
  //m_iKF2d = m_solver->m_internal->m_iKF2d;
  //m_clrsKF.assign(m_solver->m_internal->m_ds.size(), LA::Vector3ub::Get(0, 255, 255));
  m_iFrmsKF = m_LBA->m_iFrmsKF;
  m_iKF2d = m_LBA->m_iKF2d;
  m_clrsKF.assign(m_LBA->m_ds.size(), LA::Vector3ub::Get(0, 255, 255));

  m_dragingActiveFrm = false;
  m_dragingActiveFtr = false;
  SearchActiveFeatureReset();

  m_iFrmTexRGB = -1;
#ifdef CFG_STEREO
  m_iFrmTexRGBr = -1;
#endif
  Reset3DProjectionMatrix();
  Reset3DModelViewMatrix();
  //Reset3DViewPoint();
  Update3DBackgroundColor();
  m_Cv.Invalidate();

//#ifdef CFG_DEBUG
#if 1
  //m_keyDrawPrfType = DRAW_PRF_IMU_DELTA_ROTATION_STATE;
  //m_keyDrawPrfType = DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR;
  m_keyDrawPrfType = DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR;
//#ifdef CFG_GROUND_TRUTH
#if 0
  if (!m_solver->m_internal->m_CsGT.Empty()) {
    m_keyDrawCamTypeKF = DRAW_CAM_KF_GT;
    m_keyDrawCamTypeLF = DRAW_CAM_LF_GT;
  }
  if (!m_solver->m_internal->m_DsGT.empty()) {
    m_keyDrawDepType = DRAW_DEP_GT;
  }
#endif
#endif
}

bool ViewerIBA::Run(const bool visualize, const bool step, const int iFrm) {
  if (visualize) {
    //m_LBA->Synchronize();
    //m_GBA->Synchronize();
    const int iLFBkp = m_iLF;
    const bool _step = step && m_iLF != m_LBA->m_ic2LF.back();
    if (_step) {
      m_iLF = m_LBA->m_ic2LF.back();
    }
    const LocalFrame &LF = *(LocalFrame *) GetLocalFrame(m_iLF);
    m_iFrm = LF.m_T.m_iFrm;
    if (_step &&
        //GetKeyFrame(LF.m_iKFNearest)->m_T.m_iFrm == iFrm &&
        //m_iFrmActive >= GetKeyFrames() - 1
        LF.m_iKFNearest != -1 && m_LBA->m_KFs[LF.m_iKFNearest].m_T.m_iFrm == m_iFrm &&
        m_iFrmActive >= static_cast<int>(m_LBA->m_KFs.size()) - 1) {
      ++m_iFrmActive;
    }
    UpdateCurrentFrame();
    //if (GetLocalFrames() == 1) {
    if (m_iLFActive == -1) {
      if (LF.m_T.m_fileName == "") {
        m_keyDrawViewType = DRAW_VIEW_3D;
      }
      ActivateFrame(LF.m_iKFNearest);
      ActivateLocalFrame(m_iLF);
    } else if (_step) {
      const int nKFs = GetKeyFrames();
      const bool activeKF = m_iFrmActive < nKFs;
#if 0
      if (LF.m_iKFNearest != -1) {
        ActivateFrame(LF.m_iKFNearest);
      }
#endif
      if (activeKF) {
        if (m_keyDrawCamTypeKF != DRAW_CAM_KF_LBA) {
          ActivateFrame(nKFs - 1);
        } else if (LF.m_iKFNearest != -1) {
          ActivateFrame(LF.m_iKFNearest);
        }
      } else if (!activeKF) {
        if (m_iFrmActive == nKFs && m_iLFActive != iLFBkp) {
          ActivateLocalFrame(m_LBA->m_ic2LF.front());
        } else {
          ActivateLocalFrame(m_iLF);
        }
      }
      //Update3DViewPoint();
    }
    if (m_Cv.Invalid()) {
      Reset3DViewPoint();
    }
    if (m_keyPause) {
      m_keyStep = false;
    }
    if (m_pWnd) {
      m_pWnd->make_current();
    }
    Draw();
    //if (m_keyPause) {
    //  m_keyStep = false;
    //  while (m_keyPause && !m_keyStep && !m_handler.Quitted())
    //    Draw();
    //}
    while (m_keyPause && !m_keyStep && !m_handler.Quitted()) {
      Draw();
    }
    if (m_fileNameScreen != "" && UT::FileNameExtractSuffix<int>(m_fileNameScreen) != -1) {
      const int iFrmActive = m_iFrmActive;
      //const bool keyDrawString = m_keyDrawString;
      //const bool keyDrawTlnType = m_keyDrawTlnType;
      //const int keyFtrType = m_keyDrawFtrType;
      //const int keyPrjType = m_keyDrawPrjType;
      //const int keyErrType3D = m_keyDrawErrType3D;
      ActivateLocalFrame(m_iLF);
      //m_keyDrawString = false;
      //m_keyDrawTlnType = DRAW_TLN_NONE;
      //m_keyDrawFtrType = DRAW_FTR_NONE;
      //m_keyDrawPrjType = DRAW_PRJ_NONE;
      //m_keyDrawErrType3D = DRAW_ERR_NONE;
      SaveScreen(UT::FileNameIncreaseSuffix(m_fileNameScreen, m_iFrm));
      ActivateFrame(iFrmActive);
      //m_keyDrawString = keyDrawString;
      //m_keyDrawTlnType = keyDrawTlnType;
      //m_keyDrawFtrType = keyFtrType;
      //m_keyDrawPrjType = keyPrjType;
      //m_keyDrawErrType3D = keyErrType3D;
      Draw();
    }
  } else {
    if (iFrm != -1) {
      m_iFrm = iFrm;
    } else if (step) {
      ++m_iFrm;
    }
  }
  if (m_iFrm == m_iFrmSave || (m_iFrm > m_iFrmSave &&
      !UT::FileExists(UT::FileNameAppendSuffix(m_fileNameSave, m_iFrmSave)))) {
    FILE *fp = fopen(m_fileNameSave.c_str(), "wb");
    if (fp) {
      m_solver->Stop();
      m_solver->SaveB(fp);
      SaveB(fp);
      fclose(fp);
      UT::PrintSaved(m_fileNameSave);
      UT::FileCopy(m_fileNameSave, UT::FileNameAppendSuffix(m_fileNameSave, m_iFrm));
      m_handler.Quit();
    }
  }
  return !m_handler.Quitted();
}

int ViewerIBA::Start(const std::string viewFile, const bool pause) {
  int iFrmStart = -1;
  FILE *fp = fopen(m_fileNameSave.c_str(), "rb");
  if (fp) {
    m_solver->LoadB(fp);
    LoadB(fp);
    fclose(fp);
    UT::PrintLoaded(m_fileNameSave);
//#ifdef CFG_DEBUG
#if 0
    fp = fopen(m_fileNameSave.c_str(), "wb");
    m_solver->SaveB(fp);
    SaveB(fp);
    fclose(fp);
    UT::PrintSaved(m_fileNameSave);
#endif
    //iFrmStart = GetLocalFrame(m_LBA->m_ic2LF.back())->m_T.m_iFrm;
    iFrmStart = m_iFrm;
  }
  const std::string _viewFile = UT::FileNameReplaceDirectory(viewFile, ".", m_fileNameDir);
  fp = fopen(_viewFile.c_str(), "rb");
  if (fp) {
    LoadB(fp);
    fclose(fp);
    UT::PrintLoaded(_viewFile);
  }
  m_keyPause = pause;
  return iFrmStart;
}

void ViewerIBA::Stop(const std::string viewFile, const bool pause) {
  if (pause && !m_handler.Quitted()) {
    m_keyPause = true;
    Run();
  }
  const std::string _viewFile = UT::FileNameAppendSuffix(UT::FileNameReplaceDirectory(
                                viewFile, ".", m_fileNameDir));
  FILE *fp = fopen(_viewFile.c_str(), "wb");
  if (fp) {
    SaveB(fp);
    fclose(fp);
    UT::PrintSaved(_viewFile);
  }
}

void ViewerIBA::DeleteKeyFrame(const int iFrm) {
  const std::vector<int>::iterator i = std::lower_bound(m_iFrmsKF.begin(), m_iFrmsKF.end(), iFrm);
  if (i == m_iFrmsKF.end() || *i != iFrm) {
    return;
  }
  const int iKF = static_cast<int>(i - m_iFrmsKF.begin());
  if (m_iKFActive == iKF) {
    m_iKFActive = -1;
  } else if (m_iKFActive > iKF) {
    --m_iKFActive;
  }
  if (m_iFrmActiveLast == iKF) {
    m_iFrmActiveLast = -1;
  } else if (m_iFrmActiveLast > iKF) {
    --m_iFrmActiveLast;
  }
  if (m_iFrmActive == iKF) {
    m_iFrmActive = m_iFrmActiveLast;
  } else if (m_iFrmActive > iKF) {
    --m_iFrmActive;
  }
  m_iFtrActive.Invalidate();
  const int nKFs = static_cast<int>(m_iFrmsKF.size());
  m_iFrmsKF.erase(i);
  const int id1 = m_iKF2d[iKF], id2 = m_iKF2d[iKF + 1], Nd = id2 - id1;
  for (int jKF = iKF + 1; jKF <= nKFs; ++jKF) {
    m_iKF2d[jKF] -= Nd;
  }
  m_iKF2d.erase(m_iKF2d.begin() + iKF);
  m_clrsKF.erase(m_clrsKF.begin() + id1, m_clrsKF.begin() + id2);
  //UpdateCurrentFrame();
}

int ViewerIBA::GetKeyFrames() {
  return static_cast<int>(m_GBA->m_KFs.size());
}


int ViewerIBA::GetLocalFrames() {
  return static_cast<int>(m_LBA->m_LFs.size());
}

const FRM::Frame* ViewerIBA::GetKeyFrame(int iKF) {
  //return &m_GBA->m_KFs[iKF];
  return iKF < GetKeyFrames() ? (FRM::Frame* ) &m_GBA->m_KFs[iKF] : &m_LBA->m_KFs[iKF];
}

const FRM::Frame* ViewerIBA::GetLocalFrame(int iLF) {
  return &m_LBA->m_LFs[iLF];
}

Rigid3D ViewerIBA::GetCameraKF(const int iKF, const int type
#ifdef CFG_STEREO
                             , const bool right
#endif
                             ) {
  Rigid3D C;
  C.Invalidate();
  if (iKF == -1) {
    return C;
  }
  const int _type = type == -1 ? m_keyDrawCamTypeKF : type;
  switch (_type) {
  case DRAW_CAM_KF_LBA:
    //m_LBA->Synchronize();
    if (iKF < m_LBA->m_CsKF.Size()) {
      C = m_LBA->m_CsKF[iKF];
    }
    break;
  case DRAW_CAM_KF_GBA:
    //m_GBA->Synchronize();
    if (iKF < m_GBA->m_Cs.Size()) {
      C = m_GBA->m_Cs[iKF];
    }
    break;
#ifdef CFG_GROUND_TRUTH
  case DRAW_CAM_KF_GT:
    if (iKF < m_GBA->m_CsKFGT.Size()) {
      C = m_GBA->m_CsKFGT[iKF];
    }
    break;
#endif
  }
#ifdef CFG_STEREO
  if (right) {
    C.SetTranslation(m_K.m_br + C.GetTranslation());
  }
#endif
  return C;
}

Camera ViewerIBA::GetCameraLF(const int iLF, const int type
#ifdef CFG_STEREO
                            , const bool right
#endif
                            ) {
  Camera C;
  C.Invalidate();
  const int _type = type == -1 ? m_keyDrawCamTypeLF : type;
  switch (_type) {
  case DRAW_CAM_LF_LBA: {
    //m_LBA->Synchronize();
    C = m_LBA->m_CsLF[iLF];
    break;
  }
#ifdef CFG_GROUND_TRUTH
  case DRAW_CAM_LF_GT:
    if (!m_LBA->m_CsLFGT.Empty()) {
      C = m_LBA->m_CsLFGT[iLF];
    }
#endif
  }
#ifdef CFG_STEREO
  if (right) {
    C.m_T.SetTranslation(m_K.m_br + C.m_T.GetTranslation());
    C.m_T.GetPosition(C.m_p);
  }
#endif
  return C;
}

Camera ViewerIBA::GetCameraGBALM(const int iKF, const int type) {
  Camera C;
  C.Invalidate();
  if (iKF == -1) {
    return C;
  }
  const int _type = type == -1 ? m_keyDrawCamTypeKF : type;
#ifdef CFG_GROUND_TRUTH
  if (_type == DRAW_CAM_KF_GT && m_GBA->m_CsLMGT.Empty()) {
    return C;
  }
#endif
  //m_GBA->Synchronize();
  const int im = iKF - (m_GBA->m_Cs.Size() - m_GBA->m_CsLM.Size());
  if (im < 0) {
    return C;
  }
#ifdef CFG_GROUND_TRUTH
  else if (_type == DRAW_CAM_KF_GT) {
    C = m_GBA->m_CsLMGT[im];
  }
#endif
  else {
    C = m_GBA->m_CsLM[im];
  }
  if (_type == DRAW_CAM_KF_LBA) {
    C.m_T = m_LBA->m_CsKF[iKF];
    C.m_T.GetPosition(C.m_p);
  }
  return C;
}

IMU::Delta ViewerIBA::GetIMUDeltaLF(const int iLF, const int type) {
  IMU::Delta D;
  D.Invalidate();
  const int _type = type == -1 ? m_keyDrawCamTypeLF : type;
  switch (_type) {
  case DRAW_CAM_LF_LBA: {
    //m_LBA->Synchronize();
    D = m_LBA->m_DsLF[iLF];
    break;
  }
#ifdef CFG_GROUND_TRUTH
  case DRAW_CAM_LF_GT:
    if (!m_LBA->m_DsLFGT.Empty()) {
      D = m_LBA->m_DsLFGT[iLF];
    }
#endif
  }
  return D;
}

IMU::Delta ViewerIBA::GetIMUDeltaGBALM(const int iKF, const int type) {
  IMU::Delta D;
  D.Invalidate();
  const int _type = type == -1 ? m_keyDrawCamTypeKF : type;
#ifdef CFG_GROUND_TRUTH
  if (_type == DRAW_CAM_KF_GT && m_GBA->m_DsLMGT.Empty()) {
    return D;
  }
#endif
  //m_GBA->Synchronize();
  const int im = iKF - (m_GBA->m_Cs.Size() - m_GBA->m_CsLM.Size());
  if (im < 0) {
    return D;
  }
#ifdef CFG_GROUND_TRUTH
  else if (_type == DRAW_CAM_KF_GT) {
    D = m_GBA->m_DsLMGT[im];
  }
#endif
  else {
    D = m_GBA->m_DsLM[im];
  }
  return D;
}

Depth::InverseGaussian ViewerIBA::GetFrameDepthKF(const int iKF, const int type) {
  return GetKeyFrame(iKF)->m_d;
}

Depth::InverseGaussian ViewerIBA::GetFrameDepthLF(const int iLF, const int type) {
  return GetLocalFrame(iLF)->m_d;
}

Depth::InverseGaussian ViewerIBA::GetFeatureDepth(const int iKF, const int ix, const int type) {
  const int _type = type == -1 ? m_keyDrawDepType : type;
  const int id = m_iKF2d[iKF] + ix;
  switch (_type) {
  case DRAW_DEP_LBA:
    //m_LBA->Synchronize();
    if (id < int(m_LBA->m_ds.size())) {
      return m_LBA->m_ds[id];
    }
  case DRAW_DEP_GBA:
    //m_GBA->Synchronize();
    if (id < static_cast<int>(m_GBA->m_ds.size())) {
      return m_GBA->m_ds[id];
    }
#ifdef CFG_GROUND_TRUTH
  case DRAW_DEP_GT:
    //if (id < static_cast<int>(m_solver->m_internal->m_dsGT.size())) {
    //  return m_solver->m_internal->m_dsGT[id];
    //}
    if (m_LBA->m_dsGT && id < static_cast<int>(m_LBA->m_dsGT->size())) {
      return m_LBA->m_dsGT->at(id);
    }
#endif
  }
  return Depth::InverseGaussian(0.0f, FLT_MAX);
}

void ViewerIBA::UpdateCurrentFrame() {
  m_iFrmTexRGB = -1;
#ifdef CFG_STEREO
  m_iFrmTexRGBr = -1;
#endif
  if (m_K.m_K.NeedRectification() && m_RM.Empty()) {
    m_RM.Set(m_K.m_K);
  }
#ifdef CFG_STEREO
  if (m_K.m_Kr.NeedRectification() && m_RMr.Empty()) {
    m_RMr.Set(m_K.m_Kr, &m_K.m_Rr);
  }
#endif
  //const std::vector<LocalMap::CameraKF> &CsKF = m_solver->m_internal->m_CsKF;
  //const int nKFs = static_cast<int>(CsKF.size());
  const std::vector<int> &iFrmsKF = m_LBA->m_iFrmsKF;
  const int nKFs = static_cast<int>(iFrmsKF.size());
  if (static_cast<int>(m_iFrmsKF.size()) < nKFs) {
    //m_iFrmsKF.resize(nKFs);
    //for (int iKF = 0; iKF < nKFs; ++iKF) {
    //  m_iFrmsKF[iKF] = CsKF[iKF].m_iFrm;
    //}
    m_iFrmsKF = iFrmsKF;
    //const std::vector<int> &iKF2d = m_solver->m_internal->m_iKF2d;
    const std::vector<int> &iKF2d = m_LBA->m_iKF2d;
    while (m_iKF2d.size() < iKF2d.size()) {
      m_iKF2d.push_back(m_iKF2d.back());
    }
    Point2D x;
    CVD::Rgb<ubyte> clr;
    const LA::Vector3ub clrDFT = LA::Vector3ub::Get(0, 255, 255);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      const int Nx = (iKF2d[iKF + 1] - iKF2d[iKF]) - (m_iKF2d[iKF + 1] - m_iKF2d[iKF]);
      if (Nx == 0) {
        continue;
      }
      const int id = m_iKF2d[iKF + 1];
      m_clrsKF.insert(m_clrsKF.begin() + id, Nx, clrDFT);
      for (int jKF = iKF; jKF < nKFs; ++jKF) {
        m_iKF2d[jKF + 1] += Nx;
      }
#if 0
      const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(iKF);
      const std::string ext = UT::FileNameExtractExtension(KF.m_T.m_fileName);
      if (ext == "txt" || ext == "dat" || !UT::ImageLoad(KF.m_T.m_fileName, m_imgRGB, m_imgRGBTmp)
#ifdef CFG_STEREO
                                       || !UT::ImageLoad(KF.m_T.m_fileNameRight, m_imgRGBr, m_imgRGBTmp)
#endif
                                       ) {
        continue;
      }
      //const Point2D *xs = m_solver->m_internal->m_xs.data() + iKF2d[iKF + 1] - Nx;
      LA::Vector3ub *clrs = m_clrsKF.data() + id;
      for (int ix = 0; ix < Nx; ++ix) {
        //x = xs[ix];
        x = KF.m_xs[ix].m_x;
        if (m_K.m_K.NeedRectification()) {
          x = m_K.m_K.GetDistorted(x);
        }
        m_K.m_K.k().NormalizedToImage(x);
        const int _ix = static_cast<int>(x.x()), _iy = static_cast<int>(x.y());
        clr = m_imgRGB[_iy][_ix];
        clrs[ix].Set(clr.red, clr.green, clr.blue);
      }
#endif
    }
  }
#ifdef CFG_DEBUG
  //UT_ASSERT(m_clrsKF.size() == m_solver->m_internal->m_ds.size());
  //UT_ASSERT(static_cast<int>(m_iFrmsKF.size()) == nKFs);
  //for (int iKF = 0; iKF < nKFs; ++iKF) {
  //  UT_ASSERT(m_iFrmsKF[iKF] == CsKF[iKF].m_iFrm);
  //}
  //UT_ASSERT(UT::VectorEqual(m_iKF2d, m_solver->m_internal->m_iKF2d));
  UT_ASSERT(UT::VectorEqual(m_iFrmsKF, iFrmsKF));
#endif
  if (m_iFtrActive.Invalid()) {
#ifdef CFG_DEBUG
    UT_ASSERT(m_iFtrActive.m_ix.Invalid() && m_iFtrActive.m_iz.Invalid() &&
              m_iFtrActive.m_ic.Invalid());
#endif
    return;
  } else if (m_iFtrActive.m_iz.Valid()) {
#ifdef CFG_DEBUG
    m_iFtrActive.m_iz.AssertConsistency();
#endif
    if (m_iFtrActive.m_iz.m_iKF == -1 &&
        GetLocalFrame(m_iFtrActive.m_iz.m_iLF)->m_T.m_iFrm != m_iFtrActive.m_iz.m_iFrm) {
      m_iFtrActive.m_iz.Invalidate();
    }
#ifdef CFG_DEBUG
    else {
      const FRM::Frame &F = *(m_iFtrActive.m_iz.m_iKF != -1 ? GetKeyFrame(m_iFtrActive.m_iz.m_iKF) :
                              GetLocalFrame(m_iFtrActive.m_iz.m_iLF));
      UT_ASSERT(F.m_T.m_iFrm == m_iFtrActive.m_iz.m_iFrm);
      const int iKF = m_iFtrActive.m_ix.m_iKF, ix = m_iFtrActive.m_ix.m_ix;
      const int iz1 = m_iFtrActive.m_iz.m_iz;
      const int iz2 = F.SearchFeatureMeasurement(iKF, ix);
      UT_ASSERT(iz1 == iz2 && F.m_zs[iz1].m_ix == ix && F.SearchFeatureMeasurementKeyFrame(iz1) == iKF);
    }
#endif
  }
  const FRM::Frame &LF = *GetLocalFrame(m_iLF);
  if (m_iFtrActive.m_ic.Valid() && m_iFtrActive.m_ic.m_iFrm != LF.m_T.m_iFrm) {
    m_iFtrActive.m_ic.Invalidate();
  }
  if (m_iFtrActive.m_ic.Invalid()) {
    const int iz = LF.SearchFeatureMeasurement(m_iFtrActive.m_ix.m_iKF, m_iFtrActive.m_ix.m_ix);
    if (iz != -1
#ifdef CFG_STEREO
    && (!m_rightActive && LF.m_zs[iz].m_z.Valid() ||
         m_rightActive && LF.m_zs[iz].m_zr.Valid())
#endif
     ) {
      m_iFtrActive.m_ic.Set(LF.m_T.m_iFrm, iz);
    }
  }
  if (m_iFtrActive.Valid()) {
    if (m_keyDrawViewType == DRAW_VIEW_2D && m_iFtrActive.m_ic.Valid()) {
      PrintFeatureTrackLF(m_iFtrActive.m_ix.m_iKF, m_iFtrActive.m_ix.m_ix, m_iLF, "+");
    } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
      PrintActiveFeatureDepth();
    }
  }
}

void ViewerIBA::ActivateFrame(const int iFrmActive) {
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames();
  if (iFrmActive < 0 || iFrmActive >= nKFs + nLFs) {
    return;
  }
  if (iFrmActive != m_iFrmActive) {
    m_iFrmActiveLast = m_iFrmActive;
#ifdef CFG_STEREO
    m_rightActiveLast = m_rightActive;
#endif
    m_iFrmActive = iFrmActive;
  }
  if (iFrmActive < nKFs) {
    m_iKFActive = iFrmActive;
#ifdef CFG_STEREO
    GetCameraKF(m_iKFActive, -1, m_rightActive).GetTranspose(m_CT);
#else
    GetCameraKF(m_iKFActive).GetTranspose(m_CT);
#endif
  } else {
    const int iLFActive = (iFrmActive - nKFs + m_iLF + 1) % nLFs;
    if (iLFActive != m_iLFActive) {
      m_iLFActiveLast = m_iLFActive;
      m_iLFActive = iLFActive;
    }
#ifdef CFG_STEREO
    GetCameraLF(m_iLFActive, -1, m_rightActive).m_T.GetTranspose(m_CT);
#else
    GetCameraLF(m_iLFActive).m_T.GetTranspose(m_CT);
#endif
  }
}

void ViewerIBA::ActivateLocalFrame(const int iLFActive) {
  const int nLFs = GetLocalFrames();
  ActivateFrame(GetKeyFrames() + (iLFActive + nLFs - m_iLF - 1) % nLFs);
}

bool ViewerIBA::DragActiveFrame(const CVD::ImageRef &from, const CVD::ImageRef &to) {
  if (m_keyDrawViewType != DRAW_VIEW_PROFILE && m_keyDrawTlnType == DRAW_TLN_NONE) {
    return false;
  }
  const int W = m_pWnd->size().x, H = m_pWnd->size().y;
  const int w = VW_STRING_BORDER_X + VW_STRING_BORDER_X, h = VW_STRING_BORDER_Y + VW_STRING_GAP_Y;
  int y1, y2;
  if (m_keyDrawViewType == DRAW_VIEW_PROFILE) {
    y1 = Viewer::GetStringYTopLeft(2, GLUT_BITMAP_HELVETICA_10) + VW_STRING_GAP_Y;
    y2 = H - VW_STRING_BORDER_Y - VW_STRING_HEIGHT - VW_STRING_GAP_Y - H * 0.2f - VW_STRING_GAP_Y;
  } else {
    y2 = H - 1 - (VW_STRING_BORDER_Y + VW_STRING_GAP_Y + VW_STRING_HEIGHT);
    y1 = y2 - (h << 1);
  }

  if (from.y < y1 || from.y > y2) {
    return false;
  }
  m_dragingActiveFrm = true;
  const int wh = (w >> 1);
  int x = to.x;
  if (x < 0) {
    x = 0;
  } else if (x > W - 1) {
    x = W - 1;
  }
  const int nFrms = GetKeyFrames() + GetLocalFrames();
  const float dx = float(nFrms - 1) / (W - w);
  x += int(dx * 0.5f - wh + 0.5f);
  const int iFrmActive = int(x * dx + 0.5f);
  ActivateFrame(iFrmActive);
  return true;
}

void ViewerIBA::ActivateFeature(const FeatureIndex::Source &ix, const int iz) {
  if (ix.Invalid()) {
    m_iFtrActive.Invalidate();
    return;
  }
  const FeatureIndex iFtrActiveBkp = m_iFtrActive;
  m_iFtrActive.m_ix = ix;
  const bool activeKF = m_iFrmActive < GetKeyFrames();
  const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive));
  const int iFrm = F.m_T.m_iFrm;
  if (iz == -1) {
    m_iFtrActive.m_iz.Invalidate();
    m_iFtrActive.m_ic.Invalidate();
  } else if (!activeKF && m_iLFActive == m_iLF) {
    if (F.SearchFeatureMeasurement(ix.m_iKF, ix.m_ix) == iz
#ifdef CFG_STEREO
    && (!m_rightActive && F.m_zs[iz].m_z.Valid() ||
         m_rightActive && F.m_zs[iz].m_zr.Valid())
#endif
     ) {
      m_iFtrActive.m_iz.SetLF(iFrm, m_iLF, iz);
    } else {
      m_iFtrActive.m_iz.Invalidate();
    }
    m_iFtrActive.m_ic.Set(iFrm, iz);
  } else {
#ifdef CFG_DEBUG
    const int _iz = F.SearchFeatureMeasurement(ix.m_iKF, ix.m_ix);
    UT_ASSERT(_iz == iz);
#endif
    if (activeKF) {
      m_iFtrActive.m_iz.SetKF(iFrm, m_iKFActive, iz);
    } else {
      m_iFtrActive.m_iz.SetLF(iFrm, m_iLFActive, iz);
    }
  }
  if (iFtrActiveBkp == m_iFtrActive) {
    return;
  }
  PrintActiveFeatureDepth();
  PrintActiveFeatureTracks();
}

bool ViewerIBA::SearchActiveFeature(const CVD::ImageRef &x) {
  m_xSearch.x() = x.x * m_factorWinToImg.x();
  m_xSearch.y() = x.y * m_factorWinToImg.y();
  //const int dMin = 5;
  const int dMin = 20;
  m_d2MinSearch = dMin * dMin * m_factorWinToImg.x() * m_factorWinToImg.y() * m_K.m_K.fxyI();
  m_K.m_K.ImageToNormalized(m_xSearch);
  Draw(false);
  if (m_dragingActiveFtr) {
    ActivateFeature(m_ixSearchStart, m_izSearch);
  } else {
    ActivateFeature(m_ixSearch, m_izSearch);
  }
  return m_iFtrActive.Valid();
}

void ViewerIBA::SearchActiveFeature(const Point2D &x, const int iKF, const int ix,
                                    const int iz) {
  if (!m_xSearch.Valid()) {
    return;
  }
  const float d2 = (x - m_xSearch).SquaredLength();
  if (d2 > m_d2MinSearch) {
    return;
  }
  if (iKF != -1 && ix != -1) {
    m_ixSearch.Set(iKF, ix);
  }
  //if (iz != -1)
  m_izSearch = iz;
  m_d2MinSearch = d2;
}

void ViewerIBA::SearchActiveFeatureReset() {
  m_xSearch.Invalidate();
  m_ixSearch.Invalidate();
  m_izSearch = -1;
  m_d2MinSearch = FLT_MAX;
}

void ViewerIBA::SearchActiveFeatureStart() {
  SearchActiveFeatureReset();
  m_iFtrActive.Invalidate();
}

void ViewerIBA::SearchActiveFeatureStop() {
  SearchActiveFeatureReset();
}

void ViewerIBA::PrintActiveFeatureDepth() {
#ifdef CFG_DEBUG
  UT_ASSERT(m_iFtrActive.Valid());
#endif
  UT::PrintSeparator();
  const int iKF = m_iFtrActive.m_ix.m_iKF, ix = m_iFtrActive.m_ix.m_ix;
  GetFeatureDepth(iKF, ix, DRAW_DEP_LBA).Print("LBA = ", false, true);
  GetFeatureDepth(iKF, ix, DRAW_DEP_GBA).Print("GBA = ", false, true);
#ifdef CFG_GROUND_TRUTH
  GetFeatureDepth(iKF, ix, DRAW_DEP_GT).Print(" GT = ", false, true);
#endif
}

void ViewerIBA::PrintActiveFeatureTracks() {
#ifdef CFG_DEBUG
  UT_ASSERT(m_iFtrActive.Valid());
#endif
  UT::PrintSeparator();
  const int iKFx = m_iFtrActive.m_ix.m_iKF, ix = m_iFtrActive.m_ix.m_ix;
  const int nKFs = GetKeyFrames();
  for (int iKFz = 0; iKFz < nKFs; ++iKFz) {
    PrintFeatureTrackKF(iKFx, ix, iKFz, iKFx == iKFx || iKFz == m_iFtrActive.m_iz.m_iKF ? "*" : " ");
  }
  if (m_iFrmActive >= nKFs) {
    UT::PrintSeparator();
    const int nLFs = GetLocalFrames();
    for (int iLFz = (m_iLF + 1) % nLFs; iLFz != m_iLF; iLFz = (iLFz + 1) % nLFs) {
      PrintFeatureTrackLF(iKFx, ix, iLFz, iLFz == m_iFtrActive.m_iz.m_iLF ? "*" : " ");
    }
    if (m_iFtrActive.m_ic.Valid()) {
      PrintFeatureTrackLF(iKFx, ix, m_iLF, "+");
    }
  }
}

void ViewerIBA::PrintFeatureTrack(const int iFrm, const int iz, const Intrinsic &K,
                                  const Point2D &z, const Point2D &zp,
                                  const LA::SymmetricMatrix2x2f &W, const std::string str) {
  const Point2D z1 = K.GetNormalizedToImage(z), z2 = K.GetNormalizedToImage(zp);
  const Point2DCovariance Sz = W.GetInverse();
  UT::Print("%s[%d] iz = %3d  z = (%.1f %.1f) +- (%.1f %.1f)  e = %.1f\n", str.c_str(), iFrm, iz,
            z1.x(), z1.y(), K.fx() * sqrtf(Sz.sxx()), K.fy() * sqrtf(Sz.syy()),
            sqrtf((z1 - z2).SquaredLength()));
}

void ViewerIBA::PrintFeatureTrackKF(const int iKFx, const int ix, const int iKFz,
                                    const std::string str) {
  const KeyFrame &KFz = *(KeyFrame *) GetKeyFrame(iKFz);
  if (iKFx == iKFz) {
    const FTR::Source &x = KFz.m_xs[ix];
    const Point2D _x = m_K.m_K.GetNormalizedToImage(x.m_x);
    const int id = m_iKF2d[iKFx] + ix;
    UT::Print("%s[%d] ix = %3d  x = (%.1f %.1f)\n", str.c_str(), KFz.m_T.m_iFrm, ix,
              _x.x(), _x.y());
#ifdef CFG_STEREO
    if (x.m_xr.Valid()) {
      const Depth::InverseGaussian d = GetFeatureDepth(iKFx, ix);
      if (d.Valid()) {
        const Point2D zp = d.GetProjected(m_K.m_br, x.m_x);
        PrintFeatureTrack(KFz.m_T.m_iFrm, ix, m_K.m_Kr, x.m_xr, zp, x.m_Wr, str);
      }
    }
#endif
  } else {
    const Depth::InverseGaussian d = GetFeatureDepth(iKFx, ix);
    if (d.Invalid()) {
      return;
    }
    const int iz = KFz.SearchFeatureMeasurement(iKFx, ix);
    if (iz == -1) {
      return;
    }
    const int iFrm = KFz.m_T.m_iFrm;
    const Rigid3D Cx = GetCameraKF(iKFx);
    const Point2D &x = ((KeyFrame *) GetKeyFrame(iKFx))->m_xs[ix].m_x;
    const FTR::Measurement &z = KFz.m_zs[iz];
#ifdef CFG_STEREO
    if (z.m_z.Valid())
#endif
    {
      const Rigid3D Tr = GetCameraKF(iKFz) / Cx;
      const Point2D zp = d.GetProjected(Tr, x);
      PrintFeatureTrack(iFrm, iz, m_K.m_K, z.m_z, zp, z.m_W, str);
    }
#ifdef CFG_STEREO
    if (z.m_zr.Valid()) {
      const Rigid3D Tr = GetCameraKF(iKFz, -1, true) / Cx;
      const Point2D zp = d.GetProjected(Tr, x);
      PrintFeatureTrack(iFrm, iz, m_K.m_Kr, z.m_zr, zp, z.m_Wr, str);
    }
#endif
  }
}

void ViewerIBA::PrintFeatureTrackLF(const int iKFx, const int ix, const int iLFz,
                                       const std::string str) {
  const Depth::InverseGaussian d = GetFeatureDepth(iKFx, ix);
  if (d.Invalid()) {
    return;
  }
  const LocalFrame &LFz = *(LocalFrame *) GetLocalFrame(iLFz);
  const int iz = LFz.SearchFeatureMeasurement(iKFx, ix);
  if (iz == -1) {
    return;
  }
  const int iFrm = LFz.m_T.m_iFrm;
  const Rigid3D Cx = GetCameraKF(iKFx);
  const Point2D &x = ((KeyFrame *) GetKeyFrame(iKFx))->m_xs[ix].m_x;
  const FTR::Measurement &z = LFz.m_zs[iz];
#ifdef CFG_STEREO
  if (z.m_z.Valid())
#endif
  {
    const Rigid3D Tr = GetCameraLF(iLFz).m_T / Cx;
    const Point2D zp = d.GetProjected(Tr, x);
    PrintFeatureTrack(iFrm, iz, m_K.m_K, z.m_z, zp, z.m_W, str);
  }
#ifdef CFG_STEREO
  if (z.m_zr.Valid()) {
    const Rigid3D Tr = GetCameraLF(iLFz, -1, true).m_T / Cx;
    const Point2D zp = d.GetProjected(Tr, x);
    PrintFeatureTrack(iFrm, iz, m_K.m_Kr, z.m_zr, zp, z.m_Wr, str);
  }
#endif
}

#ifdef CFG_STEREO
void ViewerIBA::ActivateCamera(const bool rightActive) {
  if (rightActive == m_rightActive) {
    return;
  }
  m_iFrmActiveLast = m_iFrmActive;
  m_rightActiveLast = m_rightActive;
  m_rightActive = rightActive;
  UT::Print("\r[");
  if (m_rightActive) {
    UT::Print("IP_LEVEL_RIGHT");
  } else {
    UT::Print("IP_LEVEL_LEFT");
  }
  UT::Print("]\t\t\t");
}
#endif

int ViewerIBA::CountFeatureMatchesSource(const int iKF1, const FRM::Frame &F2) {
  const int iZ2 = F2.SearchFrameMeasurement(iKF1);
  return iZ2 == -1 ? 0 : F2.m_Zs[iZ2].CountFeatureMeasurements();
}

int ViewerIBA::CountFeatureMatchesMeasurement(const FRM::Frame &F1, const FRM::Frame &F2) {
  int N = 0;
  std::vector<FRM::Measurement>::const_iterator iZ21 = F2.m_Zs.begin(), iZ22 = F2.m_Zs.end();
  const int NZ1 = int(F1.m_Zs.size());
  for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
    const FRM::Measurement &Z1 = F1.m_Zs[iZ1];
    const int iKF = Z1.m_iKF;
    const std::vector<FRM::Measurement>::const_iterator iZ2 = std::lower_bound(iZ21, iZ22, iKF);
    if (iZ2 == F2.m_Zs.end() || iZ2->m_iKF != iKF) {
      continue;
    }
    iZ21 = iZ2;
    m_marks.assign(m_LBA->m_KFs[iKF].m_xs.size(), 0);
    for (int iz1 = Z1.m_iz1; iz1 < Z1.m_iz2; ++iz1) {
      m_marks[F1.m_zs[iz1].m_ix] = true;
    }
    for (int iz2 = iZ2->m_iz1; iz2 < iZ2->m_iz2; ++iz2) {
      if (m_marks[F2.m_zs[iz2].m_ix]) {
        ++N;
      }
    }
  }
  return N;
}

int ViewerIBA::CountFeatureMatchesKFKF(const int iKF1, const int iKF2) {
  const FRM::Frame &KF1 = *GetKeyFrame(iKF1), &KF2 = *GetKeyFrame(iKF2);
  return CountFeatureMatchesSource(iKF1, KF2) +
         CountFeatureMatchesSource(iKF2, KF1) +
         CountFeatureMatchesMeasurement(KF1, KF2);
}

int ViewerIBA::CountFeatureMatchesKFLF(const int iKF1, const int iLF2) {
  const FRM::Frame &KF1 = *GetKeyFrame(iKF1), &LF2 = *GetLocalFrame(iLF2);
  return CountFeatureMatchesSource(iKF1, LF2) +
         CountFeatureMatchesMeasurement(KF1, LF2);
}

int ViewerIBA::CountFeatureMatchesLFLF(const int iLF1, const int iLF2) {
  return CountFeatureMatchesMeasurement(m_LBA->m_LFs[iLF1], m_LBA->m_LFs[iLF2]);
}
