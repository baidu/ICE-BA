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
#include "IBA_internal.h"
#include "ViewerIBA.h"
#include "ViewerIBAKey.hpp"
#include "Parameter.h"
#include <cvd/vision.h>

bool ViewerIBA::OnKeyDown(int key) {
#ifdef __linux__
  if (key == VW_KEY_CTRL_KEY_VALUE) {
    m_ctrlDown = true;
    return true;
  }
  if (m_ctrlDown) {
    key = key - 'a' + 1;
  }
#endif
  if (Viewer::OnKeyDown(key)) {
    return true;
  }
//#ifdef CFG_DEBUG
#if 0
//#if 1
  UT::Print("%d\n", key);
#endif
//#ifdef CFG_DEBUG
#if 0
  if (key == '+' || key == '-') {
    Camera &C = m_LBA->m_CsLF[m_iLFActive];
    if (key == '+') {
      C.m_p.y() += 0.1f;
    } else {
      C.m_p.y() -= 0.1f;
    }
    C.m_T.SetPosition(C.m_p);
    return true;
  }
#endif
#ifdef CFG_STEREO
  if (key == '1' || key == '2') {
     ActivateCamera(key == '2');
    return true;
  }
#endif
  const int nKFs = GetKeyFrames();
  const bool activeKF = m_iFrmActive < nKFs;
  switch (key) {
  case VW_KEY_XD_DRAW_VIEW_TYPE:
    if (m_keyDrawViewType == DRAW_VIEW_PROFILE) {
      m_keyDrawViewType = m_keyDrawViewTypeBkp;
    } else {
      m_keyDrawViewType.Press();
    }
    return true;
  case VW_KEY_XD_PAUSE:   m_keyPause.Press();   return true;
  case VW_KEY_XD_STEP:    m_keyStep.Press();    return true;
  case VW_KEY_XD_SAVE:
    if (m_fileNameSave == "") {
      m_fileNameSave = UT::Input<std::string>("save_file");
    }
    m_iFrmSave = GetLocalFrame(m_iLF)->m_T.m_iFrm;
    m_keyStep.Press();
    //m_handler.Quit();
    return true;
  case VW_KEY_XD_SCREEN:
    if (m_fileNameScreen == "") {
      m_fileNameScreen = UT::Input<std::string>("screen_file");
    }
    if (m_fileNameScreen != "" && UT::FileNameExtractSuffix<int>(m_fileNameScreen) == -1) {
      bool resize = m_pWnd->size().x != m_K.m_K.w() || m_pWnd->size().y != m_K.m_K.h();
      if (resize) {
        const char inp = UT::Input<char>("resize (y/n)");
        if (inp != 'y' && inp != 'Y') {
          resize = false;
        }
      }
      SaveScreen(UT::FileNameAppendSuffix(m_fileNameScreen), resize);
    }
    return true;
  //case VW_KEY_XD_ACTIVATE_NEXT_FRAME:   ActivateFrame(m_iFrmActive + 1);  return true;
  //case VW_KEY_XD_ACTIVATE_LAST_FRAME:   ActivateFrame(m_iFrmActive - 1);  return true;
  case VW_KEY_XD_ACTIVATE_NEXT_FRAME:
  case VW_KEY_XD_ACTIVATE_LAST_FRAME: {
    const bool next = key == VW_KEY_XD_ACTIVATE_NEXT_FRAME;
    int iFrm = next ? m_iFrmActive + 1 : m_iFrmActive - 1;
    if (m_iFtrActive.Valid()) {
      const int nLFs = GetLocalFrames(), nFrms = nKFs + nLFs;
      const int iKF = m_iFtrActive.m_ix.m_iKF, ix = m_iFtrActive.m_ix.m_ix;
      while (iFrm >= 0 && iFrm < nFrms && iFrm != iKF) {
        const FRM::Frame &F = *(iFrm < nKFs ? GetKeyFrame(iFrm) :
                                              GetLocalFrame((iFrm - nKFs + m_iLF + 1) % nLFs));
        const int iz = F.SearchFeatureMeasurement(iKF, ix);
        if (iz != -1
#ifdef CFG_STEREO
        && (!m_rightActive && F.m_zs[iz].m_z.Valid() || m_rightActive && F.m_zs[iz].m_zr.Valid())
#endif
        ) {
          break;
        }
        iFrm = next ? iFrm + 1 : iFrm - 1;
      }
    }
    ActivateFrame(iFrm);
    return true;
  }
  case VW_KEY_XD_ACTIVATE_KEY_FRAME:
    if (m_iFtrActive.m_ix.Valid()) {
      ActivateFrame(m_iFtrActive.m_ix.m_iKF);
    } else {
      ActivateFrame(m_iKFActive);
    }
    return true;
  case VW_KEY_XD_ACTIVATE_LOCAL_FRAME:
    if (m_iFtrActive.m_iz.Valid() && m_iFtrActive.m_iz.m_iLF != -1) {
      ActivateLocalFrame(m_iFtrActive.m_iz.m_iLF);
    } else if (activeKF) {
      ActivateLocalFrame(m_iLFActive);
    } else {
      ActivateLocalFrame(m_iLFActiveLast);
    }
    return true;
  case VW_KEY_XD_ACTIVATE_CURRENT_FRAME:
    ActivateLocalFrame(m_iLF);
    return true;
  case VW_KEY_XD_DRAW_CAMERA_TYPE_NEXT:
  case VW_KEY_XD_DRAW_CAMERA_TYPE_LAST:
    if (activeKF) {
      m_keyDrawCamTypeKF.Press(key == VW_KEY_XD_DRAW_CAMERA_TYPE_NEXT, false);
      switch (m_keyDrawCamTypeKF) {
      case DRAW_CAM_KF_LBA:
        m_keyDrawCamTypeLF = DRAW_CAM_LF_LBA;
        m_keyDrawDepType = DRAW_DEP_LBA;
        break;
      case DRAW_CAM_KF_GBA:
        m_keyDrawDepType = DRAW_DEP_GBA;
        break;
#ifdef CFG_GROUND_TRUTH
      case DRAW_CAM_KF_GT:
        m_keyDrawCamTypeLF = DRAW_CAM_LF_GT;
        //if (!m_solver->m_internal->m_DsGT.empty()) {
        if (m_LBA->m_dsGT) {
          m_keyDrawDepType = DRAW_DEP_GT;
        }
        break;
#endif
      }
    } else {
      m_keyDrawCamTypeLF.Press(key == VW_KEY_XD_DRAW_CAMERA_TYPE_NEXT, false);
      switch (m_keyDrawCamTypeLF) {
      case DRAW_CAM_LF_LBA:
        m_keyDrawCamTypeKF = DRAW_CAM_KF_LBA;
        m_keyDrawDepType = DRAW_DEP_LBA;
        break;
#ifdef CFG_GROUND_TRUTH
      case DRAW_CAM_LF_GT:
        m_keyDrawCamTypeKF = DRAW_CAM_KF_GT;
        //if (!m_solver->m_internal->m_DsGT.empty()) {
        if (m_LBA->m_dsGT) {
          m_keyDrawDepType = DRAW_DEP_GT;
        }
        break;
#endif
      }
    }
    ActivateFrame(m_iFrmActive);
    return true;
#ifdef CFG_GROUND_TRUTH
  case VW_KEY_XD_DRAW_CAMERA_TYPE_GROUND_TRUTH:
    if (activeKF && m_keyDrawCamTypeKF == DRAW_CAM_KF_GT) {
      m_keyDrawCamTypeKF.Set(DRAW_CAM_KF_GBA);
      m_keyDrawDepType = DRAW_DEP_GBA;
    } else if (!activeKF && m_keyDrawCamTypeLF == DRAW_CAM_LF_GT) {
      m_keyDrawCamTypeKF.Set(DRAW_CAM_KF_LBA);
      m_keyDrawCamTypeLF = DRAW_CAM_LF_LBA;
      m_keyDrawDepType = DRAW_DEP_LBA;
    } else if (activeKF && GetCameraKF(m_iKFActive, DRAW_CAM_KF_GT).Valid() ||
              !activeKF && GetCameraLF(m_iLFActive, DRAW_CAM_LF_GT).Valid()) {
      m_keyDrawCamTypeKF.Set(DRAW_CAM_KF_GT);
      m_keyDrawCamTypeLF = DRAW_CAM_LF_GT;
      //if (!m_solver->m_internal->m_DsGT.empty()) {
      if (m_LBA->m_dsGT) {
        m_keyDrawDepType = DRAW_DEP_GT;
      }
    }
    ActivateFrame(m_iFrmActive);
    return true;
#endif
  case VW_KEY_XD_DRAW_DEPTH_TYPE_NEXT:
  case VW_KEY_XD_DRAW_DEPTH_TYPE_LAST:
    m_keyDrawDepType.Press(key == VW_KEY_XD_DRAW_DEPTH_TYPE_NEXT, false);
    return true;
  case VW_KEY_XD_DRAW_STRING:
    m_keyDrawString.Press();
    return true;
  case VW_KEY_XD_DRAW_TIME_LINE_NEXT:
  case VW_KEY_XD_DRAW_TIME_LINE_LAST:
    m_keyDrawTlnType.Press(key == VW_KEY_XD_DRAW_TIME_LINE_NEXT);
    return true;
  case VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_1:
  case VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_1:
  case VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_2:
  case VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_2:
    if (m_keyDrawTlnType == DRAW_TLN_FEATURE_MATCH) {
      m_keyDrawTlnMaxFtrMatches.Press(key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_1 ||
                                      key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_2);
    } else if (key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_1 ||
               key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_DECREASE_1) {
      m_keyDrawTlnPriorVarPos.Press(key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_1);
    } else {
      m_keyDrawTlnPriorVarRot.Press(key == VW_KEY_XD_DRAW_TIME_LINE_BRIGHTNESS_INCREASE_2);
    }
    return true;
  case VM_KEY_XD_PRINT_CALIBRATION:
    UT::PrintSeparator();
    m_K.m_K.Print();
#ifdef CFG_STEREO
    m_K.m_Kr.Print();
#endif
    return true;
  case VW_KEY_XD_INPUT_ACTIVE_FEATURE: {
    const int iFrm = UT::Input<int>(" Frame");
    const std::vector<GlobalBundleAdjustor::KeyFrame>::const_iterator iKF = std::lower_bound(
      m_GBA->m_KFs.begin(), m_GBA->m_KFs.end(), iFrm);
    if (iKF != m_GBA->m_KFs.end() && *iKF == iFrm) {
      const int ix = UT::Input<int>("Source");
      if (ix < 0 || ix >= static_cast<int>(iKF->m_xs.size())) {
        return false;
      }
      const int _iKF = static_cast<int>(iKF - m_GBA->m_KFs.begin());
      const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive));
      const int iz = F.SearchFeatureMeasurement(_iKF, ix);
      ActivateFeature(FeatureIndex::Source(_iKF, ix), iz);
    }
    return false; }
  }
  if (m_keyDrawViewType == DRAW_VIEW_2D || m_keyDrawViewType == DRAW_VIEW_3D) {
    switch (key) {
    case VW_KEY_XD_DRAW_AXIS:
      m_keyDrawAxis.Press();
      return true;
    case VW_KEY_XD_DRAW_AXIS_LENGTH_INCREASE:
    case VW_KEY_XD_DRAW_AXIS_LENGTH_DECREASE:
      if (m_keyDrawAxis != DRAW_AXIS_NONE) {
        m_keyDrawAxisLen.Press(key == VW_KEY_XD_DRAW_AXIS_LENGTH_INCREASE);
        m_frustrum.SetAxisLength(m_keyDrawAxisLen);
        m_frustrumActive.SetAxisLength(m_keyDrawAxisLen);
        return true;
      }
      break;
    case VW_KEY_XD_DRAW_DEPTH_PLANE:
      m_keyDrawDepPlane.Press();
      return true;
    case VW_KEY_XD_DRAW_DEPTH_VARIANCE_INCREASE:
    case VW_KEY_XD_DRAW_DEPTH_VARIANCE_DECREASE:
      m_keyDrawDepVar.Press(key == VW_KEY_XD_DRAW_DEPTH_VARIANCE_INCREASE);
      return true;
    case VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_INCREASE:
    case VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_DECREASE:
      if (m_keyDrawTlnType != DRAW_TLN_NONE ||
         (m_keyDrawViewType == DRAW_VIEW_2D && (m_keyDrawErrType2D == DRAW_ERR_ALL ||
                                                m_keyDrawErrType2D == DRAW_ERR_COVARIANCE)) ||
         (m_keyDrawViewType == DRAW_VIEW_3D && (m_keyDrawErrType3D == DRAW_ERR_ALL ||
                                                m_keyDrawErrType3D == DRAW_ERR_COVARIANCE))) {
        m_keyDrawCovProb.Press(key == VW_KEY_XD_DRAW_COVARIANCE_PROBABILITY_INCREASE);
        return true;
      }
      break;
    case VW_KEY_XD_DRAW_COVARIANCE_SCALE_INCREASE:
    case VW_KEY_XD_DRAW_COVARIANCE_SCALE_DECREASE:
      if (m_keyDrawTlnType != DRAW_TLN_NONE ||
         (m_keyDrawViewType == DRAW_VIEW_2D && (m_keyDrawErrType2D == DRAW_ERR_ALL ||
                                                m_keyDrawErrType2D == DRAW_ERR_COVARIANCE)
       || m_keyDrawViewType == DRAW_VIEW_3D && (m_keyDrawErrType3D == DRAW_ERR_ALL ||
                                                m_keyDrawErrType3D == DRAW_ERR_COVARIANCE))) {
        m_keyDrawCovScale.Press(key == VW_KEY_XD_DRAW_COVARIANCE_SCALE_INCREASE);
        return true;
      }
      break;
    case VW_KEY_PROFILE_ACTIVATE:
      if (m_keyDrawViewType != DRAW_VIEW_PROFILE) {
        m_keyDrawViewTypeBkp = m_keyDrawViewType;
        m_keyDrawViewType = DRAW_VIEW_PROFILE;
      }
      return true;
    }
  } else if (m_keyDrawViewType == DRAW_VIEW_PROFILE) {
    switch (key) {
    case VW_KEY_PROFILE_ACTIVATE:
      m_keyDrawViewType = m_keyDrawViewTypeBkp;
      return true;
    case VM_KEY_PROFILE_LEVEL_1_NEXT:
    case VM_KEY_PROFILE_LEVEL_1_LAST: {
      const int types[] = {DRAW_PRF_ACCELERATION, DRAW_PRF_IMU_DELTA_ROTATION_STATE,
                           DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE, DRAW_PRF_REPROJECTION_ERROR,
                           DRAW_PRF_STATE_ROTATION_ABSOLUTE,
                           DRAW_PRF_TYPES};
      const int N = sizeof(types) / sizeof(int);
      DrawProfileTypeStep(types, N, key == VM_KEY_PROFILE_LEVEL_1_NEXT);
      return true; }
    case VM_KEY_PROFILE_LEVEL_2_NEXT:
    case VM_KEY_PROFILE_LEVEL_2_LAST: {
      const int types[] = {DRAW_PRF_ACCELERATION, DRAW_PRF_GYROSCOPE,
                           DRAW_PRF_IMU_DELTA_ROTATION_STATE,
                           DRAW_PRF_IMU_DELTA_POSITION_STATE,
                           DRAW_PRF_IMU_DELTA_VELOCITY_STATE,
                           DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE,
                           DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE,
                           DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE,
                           DRAW_PRF_CAMERA_PRIOR_POSITION_STATE,
                           DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE,
                           DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE,
                           DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE,
                           DRAW_PRF_REPROJECTION_ERROR,
                           DRAW_PRF_STATE_ROTATION_ABSOLUTE,
                           DRAW_PRF_STATE_POSITION_ABSOLUTE,
                           DRAW_PRF_STATE_VELOCITY,
                           DRAW_PRF_STATE_BIAS_ACCELERATION,
                           DRAW_PRF_STATE_BIAS_GYROSCOPE,
                           DRAW_PRF_TYPES};
      const int N = sizeof(types) / sizeof(int);
      DrawProfileTypeStep(types, N, key == VM_KEY_PROFILE_LEVEL_2_NEXT);
      return true; }
    case VM_KEY_PROFILE_LEVEL_3_NEXT:
    case VM_KEY_PROFILE_LEVEL_3_LAST: {
      const int typeBkp = m_keyDrawPrfType;
      while (m_keyDrawPrfType.Press(key == VM_KEY_PROFILE_LEVEL_3_NEXT, false) &&
             !DrawProfileTypeValid()) {}
      if (!DrawProfileTypeValid()) {
        m_keyDrawPrfType = typeBkp;
      }
      return true; }
    }
  }
  if (m_keyDrawViewType == DRAW_VIEW_2D) {
    switch (key) {
    case VW_KEY_2D_DRAW_FEATURE_TYPE:     m_keyDrawFtrType.Press();   return true;
    case VW_KEY_2D_DRAW_PROJECTION_TYPE:  m_keyDrawPrjType.Press();   return true;
    case VW_KEY_2D_DRAW_ERROR_TYPE:       m_keyDrawErrType2D.Press(); return true;
    }
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    switch (key) {
    case VW_KEY_3D_DRAW_MOTION_TYPE:
      if (activeKF) {
        m_keyDrawMotTypeKF.Press();
      } else {
        m_keyDrawMotTypeLF.Press();
      }
      return true;
    case VW_KEY_3D_DRAW_STRUCTURE_TYPE:
      m_keyDrawStrType.Press();
      return true;
    case VW_KEY_3D_DRAW_DEPTH_COMPARE_TYPE:
      if (m_keyDrawErrType3D == DRAW_ERR_ALL || m_keyDrawErrType3D == DRAW_ERR_MEAN) {
        m_keyDrawDepTypeCMP.Press();
        //while(m_keyDrawDepTypeCMP == m_keyDrawDepType)
        //  m_keyDrawDepTypeCMP.Press();
      }
      return true;
    case VW_KEY_3D_DRAW_CAMERA_SIZE_INCREASE:
    case VW_KEY_3D_DRAW_CAMERA_SIZE_DECREASE:
      m_keyDrawCamSize.Press(key == VW_KEY_3D_DRAW_CAMERA_SIZE_INCREASE);
      m_frustrum.SetSize(m_keyDrawCamSize);
      m_frustrumActive.SetSize(m_keyDrawCamSize * VW_CAMERA_SIZE_ACTIVE_RATIO);
      return true;
    case VW_KEY_3D_DRAW_CAMERA_VELOCITY:      m_keyDrawCamVelocity.Press(); break;
    case VW_KEY_3D_DRAW_CAMERA_TEXTURE:       m_keyDrawCamTex.Press();      break;
#ifdef CFG_GROUND_TRUTH
    case VW_KEY_3D_DRAW_CAMERA_GROUND_TRUTH:  m_keyDrawCamGT.Press();       break;
#endif
    case VW_KEY_3D_DRAW_ERROR_TYPE:
      m_keyDrawErrType3D.Press();
      return true;
    case VW_KEY_3D_DRAW_BACKGROUND_COLOR:
      m_keyDrawBgClr.Press();
      Update3DBackgroundColor();
      return true;
    case VW_KEY_3D_RESET_VIEW_POINT_ACTIVE:
    case VW_KEY_3D_RESET_VIEW_POINT_VIRTUAL:
      Reset3DViewPoint(key == VW_KEY_3D_RESET_VIEW_POINT_ACTIVE);
      return true;
    }
  }
  return false;
}

void ViewerIBA::OnMouseMove(const CVD::ImageRef &where) {
  //double Xx, Xy, Xz;
  //gluUnProject(where.x, m_pWnd->size().y - where.y, 0, m_modelMatrix, m_projMatrix, m_viewport, &Xx, &Xy, &Xz);
  //UT::Print("\r%e %e %e\t\t\t", Xx, Xy, Xz);
}

void ViewerIBA::OnMouseDown(const CVD::ImageRef &where, const int button) {
  if (m_keyDrawViewType == DRAW_VIEW_2D) {
    if (m_iFrmActive >= GetKeyFrames() && m_iLFActive == m_iLF && m_iFtrActive.Valid()) {
      SearchActiveFeatureReset();
      m_dragingActiveFtr = true;
      m_ixSearchStart = m_iFtrActive.m_ix;
    }
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    if (button == VW_MOUSE_BUTTON_LEFT) {
      UnProjectWindowPoint(where, m_whereMouseDown3DStart);
      m_arcball.StartRotation(m_whereMouseDown3DStart);
    } else if (button == VW_MOUSE_BUTTON_RIGHT) {
      m_translationStart = m_translation;
      UnProjectWindowPoint(where, m_whereMouseDown3DStart, m_arcball.GetCenterZ());
    }
  }
}

void ViewerIBA::OnMouseUp(const CVD::ImageRef &where, const int button) {
  if (m_dragingActiveFrm) {
    m_dragingActiveFrm = false;
  } else if (m_dragingActiveFtr) {
    m_dragingActiveFtr = false;
    SearchActiveFeatureReset();
  } else if (m_keyDrawViewType == DRAW_VIEW_2D && (button == VW_MOUSE_BUTTON_MIDDLE_UP ||
                                                   button == VW_MOUSE_BUTTON_MIDDLE_DOWN)) {
    m_drawPch = false;
    Draw2DFeatures();
    Draw2DProjections();
    if (m_drawPch) {
      if (button == VW_MOUSE_BUTTON_MIDDLE_UP) {
        m_keyDrawPchSize.Increase();
      } else {
        m_keyDrawPchSize.Decrease();
      }
    }
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    const float z = ComputeMeanDepth(m_Cv);
    if (button == VW_MOUSE_BUTTON_MIDDLE_UP || button == VW_MOUSE_BUTTON_MIDDLE_DOWN) {
      m_factorWinToTranslationZ = z / m_pWnd->size().y * VW_SCROLL_TRANSLATTON_Z_RATE;
      const float factor = button == VW_MOUSE_BUTTON_MIDDLE_DOWN ? m_factorWinToTranslationZ :
                                                                  -m_factorWinToTranslationZ;
      m_translation.xyzr() = m_arcball.GetRotation().r_20_21_22_x() * factor + m_translation.xyzr();
    }
    Point3D t = m_arcball.GetCenter();
    t.z() -= z;
    m_translation += m_arcball.GetRotationTranspose().GetApplied(t) - t;
    m_arcball.Set(z, UnProjectWindowSize(z) * VW_ARCBALL_RADIUS_TO_WINDOW_SIZE_RATIO);
    Update3DViewPoint();
  } else if (m_keyDrawViewType == DRAW_VIEW_PROFILE && (button == VW_MOUSE_BUTTON_MIDDLE_UP ||
                                                        button == VW_MOUSE_BUTTON_MIDDLE_DOWN)) {
    const int type = GetDrawProfileScaleType();
    m_keyDrawPrfScale[type].Press(button == VW_MOUSE_BUTTON_MIDDLE_DOWN);
  }
}

bool ViewerIBA::OnMouseDraging(const CVD::ImageRef &from, const CVD::ImageRef &to,
                                  const int button) {
  if ((m_keyDrawViewType == DRAW_VIEW_PROFILE || m_keyDrawTlnType != DRAW_TLN_NONE) &&
      DragActiveFrame(from, to)) {
    return true;
  } else if (m_keyDrawViewType == DRAW_VIEW_2D) {
    if (m_iFrmActive >= GetKeyFrames() && m_iLFActive == m_iLF && m_iFtrActive.Valid() &&
        SearchActiveFeature(to)) {
      return true;
    } else {
      return false;
    }
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    if (button == VW_MOUSE_BUTTON_LEFT) {
      Point3D whereMouseDown3D;
      UnProjectWindowPoint(to, whereMouseDown3D);
      m_arcball.ComputeRotation(whereMouseDown3D);
    } else if (button == VW_MOUSE_BUTTON_RIGHT) {
      Point3D whereMouseDown3D;
      UnProjectWindowPoint(to, whereMouseDown3D, m_arcball.GetCenterZ());
      const Rotation3D &R = m_arcball.GetRotation();
      m_translation.xyzr() = R.r_00_01_02_x() * (whereMouseDown3D.x() - m_whereMouseDown3DStart.x()) +
                             R.r_10_11_12_x() * (whereMouseDown3D.y() - m_whereMouseDown3DStart.y()) +
                             m_translationStart.xyzr();
    }
    Update3DViewPoint();
    return true;
  }
  return false;
}

bool ViewerIBA::OnMouseDoubleClicked(const CVD::ImageRef &where, const int button) {
  if (button == VW_MOUSE_BUTTON_MIDDLE_UP || button == VW_MOUSE_BUTTON_MIDDLE_DOWN) {
    return false;
  } else if (button == VW_MOUSE_BUTTON_RIGHT) {
    const Point2D x = Point2D(where.x * m_factorWinToImg.x(), where.y * m_factorWinToImg.y());
    x.Print("x = ", false, true);
    return true;
  } else {
    SearchActiveFeatureStart();
    const bool found = SearchActiveFeature(where);
    SearchActiveFeatureStop();
    return found;
  }
}

void ViewerIBA::OnResize(const CVD::ImageRef &size) {
  Viewer::OnResize(size);
  m_viewport[0] = 0;
  m_viewport[1] = 0;
  m_viewport[2] = size.x;
  m_viewport[3] = size.y;
  m_factorImgToWin.Set(float(size.x) / m_K.m_K.w(), float(size.y) / m_K.m_K.h());
  m_factorWinToImg.Set(1.0f / m_factorImgToWin.x(), 1.0f / m_factorImgToWin.y());
  const int sizeCross = 8;
  const int sizeBox = 10;
  m_sizeCross.Set(sizeCross * m_factorWinToImg.x() * m_K.m_K.fxI(),
                  sizeCross * m_factorWinToImg.y() * m_K.m_K.fyI());
  m_sizeBox.Set(sizeBox * m_factorWinToImg.x() * m_K.m_K.fxI(),
                sizeBox * m_factorWinToImg.y() * m_K.m_K.fyI());
}

void ViewerIBA::SaveB(FILE *fp) {
  UT::SaveB(m_iLF, fp);
  UT::SaveB(m_iFrm, fp);
  UT::SaveB(m_iKFActive, fp);
  UT::SaveB(m_iLFActive, fp);
  UT::SaveB(m_iLFActiveLast, fp);
  UT::SaveB(m_iFrmActive, fp);
  UT::SaveB(m_iFrmActiveLast, fp);
#ifdef CFG_STEREO
  UT::SaveB(m_rightActive, fp);
  UT::SaveB(m_rightActiveLast, fp);
#else
  const bool rightActive = false;
  const bool rightActiveLast = false;
  UT::SaveB(rightActive, fp);
  UT::SaveB(rightActiveLast, fp);
#endif
  UT::SaveB(m_iFtrActive, fp);
  UT::VectorSaveB(m_iFrmsKF, fp);
  UT::VectorSaveB(m_iKF2d, fp);
  UT::VectorSaveB(m_clrsKF, fp);
  UT::SaveB(m_frustrum, fp);
  UT::SaveB(m_frustrumActive, fp);
  UT::SaveB(m_translation, fp);
  UT::SaveB(m_arcball, fp);
  UT::SaveB(m_Cv, fp);
  UT::SaveB(m_factorWinToTranslationZ, fp);
  m_keyDrawViewType.SaveB(fp);
  m_keyDrawViewTypeBkp.SaveB(fp);
  m_keyPause.SaveB(fp);
  m_keyStep.SaveB(fp);
  m_keyDrawCamTypeKF.SaveB(fp);
  m_keyDrawCamTypeLF.SaveB(fp);
  m_keyDrawDepType.SaveB(fp);
  m_keyDrawDepTypeCMP.SaveB(fp);
  m_keyDrawString.SaveB(fp);
  m_keyDrawTlnType.SaveB(fp);
  m_keyDrawTlnMaxFtrMatches.SaveB(fp);
  m_keyDrawTlnPriorVarPos.SaveB(fp);
  m_keyDrawTlnPriorVarRot.SaveB(fp);
  m_keyDrawPrfType.SaveB(fp);
  for (int t = 0; t < DRAW_PRF_TYPES; ++t) {
    m_keyDrawPrfScale[t].SaveB(fp);
  }
  m_keyDrawAxis.SaveB(fp);
  m_keyDrawAxisLen.SaveB(fp);
  m_keyDrawDepPlane.SaveB(fp);
  m_keyDrawDepVar.SaveB(fp);
  m_keyDrawCovProb.SaveB(fp);
  m_keyDrawCovScale.SaveB(fp);
  m_keyDrawFtrType.SaveB(fp);
  m_keyDrawPrjType.SaveB(fp);
  m_keyDrawErrType2D.SaveB(fp);
  m_keyDrawMotTypeLF.SaveB(fp);
  m_keyDrawMotTypeKF.SaveB(fp);
  m_keyDrawStrType.SaveB(fp);
  m_keyDrawCamSize.SaveB(fp);
  m_keyDrawCamVelocity.SaveB(fp);
  m_keyDrawCamTex.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
  m_keyDrawCamGT.SaveB(fp);
#endif
  m_keyDrawErrType3D.SaveB(fp);
  m_keyDrawBgClr.SaveB(fp);
}

void ViewerIBA::LoadB(FILE *fp) {
  UT::LoadB(m_iLF, fp);
  UT::LoadB(m_iFrm, fp);
  UT::LoadB(m_iKFActive, fp);
  UT::LoadB(m_iLFActive, fp);
  UT::LoadB(m_iLFActiveLast, fp);
  UT::LoadB(m_iFrmActive, fp);
  UT::LoadB(m_iFrmActiveLast, fp);
#ifdef CFG_STEREO
  UT::LoadB(m_rightActive, fp);
  UT::LoadB(m_rightActiveLast, fp);
#else
  const bool rightActive = UT::LoadB<bool>(fp);
  const bool m_rightActiveLast = UT::LoadB<bool>(fp);
#endif
  UT::LoadB(m_iFtrActive, fp);
  UT::VectorLoadB(m_iFrmsKF, fp);
  UT::VectorLoadB(m_iKF2d, fp);
  UT::VectorLoadB(m_clrsKF, fp);
  UT::LoadB(m_frustrum, fp);
  UT::LoadB(m_frustrumActive, fp);
  UT::LoadB(m_translation, fp);
  UT::LoadB(m_arcball, fp);
  UT::LoadB(m_Cv, fp);
  UT::LoadB(m_factorWinToTranslationZ, fp);
  m_keyDrawViewType.LoadB(fp);
  m_keyDrawViewTypeBkp.LoadB(fp);
  m_keyPause.LoadB(fp);
  m_keyStep.LoadB(fp);
  m_keyDrawCamTypeKF.LoadB(fp);
  m_keyDrawCamTypeLF.LoadB(fp);
  m_keyDrawDepType.LoadB(fp);
  m_keyDrawDepTypeCMP.LoadB(fp);
  m_keyDrawString.LoadB(fp);
  m_keyDrawTlnType.LoadB(fp);
  m_keyDrawTlnMaxFtrMatches.LoadB(fp);
  m_keyDrawTlnPriorVarPos.LoadB(fp);
  m_keyDrawTlnPriorVarRot.LoadB(fp);
  m_keyDrawPrfType.LoadB(fp);
  for (int t = 0; t < DRAW_PRF_TYPES; ++t) {
    m_keyDrawPrfScale[t].LoadB(fp);
  }
  m_keyDrawAxis.LoadB(fp);
  m_keyDrawAxisLen.LoadB(fp);
  m_keyDrawDepPlane.LoadB(fp);
  m_keyDrawDepVar.LoadB(fp);
  m_keyDrawCovProb.LoadB(fp);
  m_keyDrawCovScale.LoadB(fp);
  m_keyDrawFtrType.LoadB(fp);
  m_keyDrawPrjType.LoadB(fp);
  m_keyDrawErrType2D.LoadB(fp);
  m_keyDrawMotTypeLF.LoadB(fp);
  m_keyDrawMotTypeKF.LoadB(fp);
  m_keyDrawStrType.LoadB(fp);
  m_keyDrawCamSize.LoadB(fp);
  m_keyDrawCamVelocity.LoadB(fp);
  m_keyDrawCamTex.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
  m_keyDrawCamGT.LoadB(fp);
#endif
  m_keyDrawErrType3D.LoadB(fp);
  m_keyDrawBgClr.LoadB(fp);
}

void ViewerIBA::SaveScreen(const std::string fileName, const bool sizeImg) {
  const CVD::ImageRef size1 = m_pWnd->size();
  if (sizeImg) {
    OnResize(CVD::ImageRef(m_K.m_K.w(), m_K.m_K.h()));
  }
  const CVD::ImageRef size2 = m_pWnd->size();
  if (m_screenCombine == 1) {
    const int keyDrawViewTypeBkp = m_keyDrawViewType;
    m_keyDrawViewType = DRAW_VIEW_3D;
    Draw(false);
    m_imgRGBScreen.resize(CVD::ImageRef(size2.x * 2, size2.y));
    m_imgRGBTmp.resize(size2);
    glReadPixels(0, 0, size2.x, size2.y, GL_RGB, GL_UNSIGNED_BYTE, m_imgRGBTmp.data());
    CVD::copy(m_imgRGBTmp, m_imgRGBScreen);
    m_keyDrawViewType = DRAW_VIEW_2D;
    Draw(false);
    glReadPixels(0, 0, size2.x, size2.y, GL_RGB, GL_UNSIGNED_BYTE, m_imgRGBTmp.data());
    CVD::copy(m_imgRGBTmp, m_imgRGBScreen, size2, CVD::ImageRef(0, 0), CVD::ImageRef(size2.x, 0));
    m_keyDrawViewType = keyDrawViewTypeBkp;
  } else if (m_screenCombine == 2) {
    const int keyDrawViewTypeBkp = m_keyDrawViewType;
    m_keyDrawViewType = DRAW_VIEW_2D;
    Draw(false);
    m_imgRGBScreen.resize(CVD::ImageRef(size2.x, size2.y * 2));
    m_imgRGBTmp.resize(size2);
    glReadPixels(0, 0, size2.x, size2.y, GL_RGB, GL_UNSIGNED_BYTE, m_imgRGBTmp.data());
    CVD::copy(m_imgRGBTmp, m_imgRGBScreen);
    m_keyDrawViewType = DRAW_VIEW_3D;
    Draw(false);
    glReadPixels(0, 0, size2.x, size2.y, GL_RGB, GL_UNSIGNED_BYTE, m_imgRGBTmp.data());
    CVD::copy(m_imgRGBTmp, m_imgRGBScreen, size2, CVD::ImageRef(0, 0), CVD::ImageRef(0, size2.y));
    m_keyDrawViewType = keyDrawViewTypeBkp;
  } else {
    Draw(false);
    m_imgRGBScreen.resize(size2);
    glReadPixels(0, 0, size2.x, size2.y, GL_RGB, GL_UNSIGNED_BYTE, m_imgRGBScreen.data());
  }
  CVD::flipVertical(m_imgRGBScreen);
  UT::ImageSave(fileName, m_imgRGBScreen);
  if (sizeImg) {
    OnResize(size1);
  }
}
