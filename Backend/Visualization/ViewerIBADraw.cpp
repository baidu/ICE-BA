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
#include "ViewerIBA.h"
#include "ViewerIBAKey.hpp"
#include "IBA_internal.h"

void ViewerIBA::Draw(const bool swapBuffers) {
//#ifndef WIN32
//#if 0
#if 1
  if (!m_pWnd) {
    if (m_keyPause) {
      //const int inp = UT::Input<int>();
      const int inp = UT::Input<int>("[%d] [quit %d] [pause %d] [step %d] [save %d]",
                                     GetLocalFrame(m_iLF)->m_T.m_iFrm, VW_KEY_QUIT,
                                     VW_KEY_XD_PAUSE, VW_KEY_XD_STEP, VW_KEY_XD_SAVE);
      if (inp == VW_KEY_QUIT) {
        m_handler.Quit();
      } else if (inp == VW_KEY_XD_PAUSE) {
        m_keyPause.Press();
      } else if (inp == VW_KEY_XD_STEP) {
        m_keyStep.Press();
      } else if (inp == VW_KEY_XD_SAVE) {
        m_iFrmSave = GetLocalFrame(m_iLF)->m_T.m_iFrm;
        m_keyPause.Press();
        m_keyStep.Press();
      }
    }
    return;
  }
#endif
#ifdef CFG_DEBUG
  if (m_iFrmActive < GetKeyFrames()) {
    UT_ASSERT(m_iKFActive == m_iFrmActive);
  }
#endif

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (m_keyDrawViewType == DRAW_VIEW_2D) {
    Draw2DImage();

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glScalef(m_factorImgToWin.x(), m_factorImgToWin.y(), 1.0f);
    const Intrinsic &K = m_K.m_K;
    glTranslatef(K.cx(), K.cy(), 0.0f);
    glScalef(K.fx(), K.fy(), 1.0f);
    Draw2DFeatures();
    Draw2DProjections();
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(m_projMatrix);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(m_modelMatrixScale);
    glMultMatrixf(m_CT);
    Draw3DWorldAxis();
    Draw3DDepthPlane();
    Draw3DObjects();
    //Draw3DCameras();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadMatrixd(m_projMatrix);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(m_modelMatrixScale);
    glPushMatrix();
    //m_arcball.Draw(10, 10);
    m_arcball.ApplyTransformation();
    glTranslatef(m_translation.x(), m_translation.y(), m_translation.z());
    Draw3DDepthPlane();
    Draw3DObjects();
    Draw3DPoints();
    Draw3DCameras();
    Draw3DWorldAxis();
    glPopMatrix();
//#if _DEBUG
#if 0
//#if 1
    glColor3ub(255, 255, 255);
    m_arcball.Draw(20, 20);
#endif
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
  } else if (m_keyDrawViewType == DRAW_VIEW_PROFILE) {
    DrawProfile();
  }

  DrawString();
  if (m_keyDrawViewType != DRAW_VIEW_PROFILE) {
    DrawTimeLine();
  }
  Viewer::Draw(swapBuffers);
}

void ViewerIBA::DrawString() {
  if (!m_keyDrawString) {
    return;
  }
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames();
  const GlobalBundleAdjustor::KeyFrame *KF = m_iFrmActive < nKFs ? &m_GBA->m_KFs[m_iKFActive] :
                                                                   NULL;
  const LocalFrame *LF = KF == NULL ? (LocalFrame *) GetLocalFrame(m_iLFActive) : NULL;
    if (KF) {
      glColor3ub(0, 255, 255);
    } else {
      glColor3ub(0, 255, 0);
    }
  const FRM::Tag &T = KF ? KF->m_T : LF->m_T;
  if (KF && m_keyDrawCamTypeKF == DRAW_CAM_KF_LBA || LF && m_keyDrawCamTypeLF == DRAW_CAM_LF_LBA) {
    Viewer::DrawStringTopLeft("LBA [%d] t %.3f", T.m_iFrm, T.m_t);
    Viewer::DrawStringCurrent("  fps %.2f  sync %.0f  norm %.0f  schur %.0f  cam %.0f  dep %.0f  dl %.0f",
                              m_LBA->m_ts[LocalBundleAdjustor::TM_TOTAL].GetFPS(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_SYNCHRONIZE].GetAverageMilliseconds(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_FACTOR].GetAverageMilliseconds(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_SCHUR_COMPLEMENT].GetAverageMilliseconds(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_CAMERA].GetAverageMilliseconds(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_DEPTH].GetAverageMilliseconds(),
                              m_LBA->m_ts[LocalBundleAdjustor::TM_UPDATE].GetAverageMilliseconds());
  } else if (KF && m_keyDrawCamTypeKF == DRAW_CAM_KF_GBA) {
    Viewer::DrawStringTopLeft("GBA [%d] t %.3f", T.m_iFrm, T.m_t);
    Viewer::DrawStringCurrent("  fps %.2f  sync %.0f  norm %.0f  schur %.0f  cam %.0f  dep %.0f  dl %.0f",
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_TOTAL].GetFPS(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_SYNCHRONIZE].GetFPS(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_FACTOR].GetAverageMilliseconds(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_SCHUR_COMPLEMENT].GetAverageMilliseconds(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_CAMERA].GetAverageMilliseconds(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_DEPTH].GetAverageMilliseconds(),
                              m_GBA->m_ts[GlobalBundleAdjustor::TM_UPDATE].GetAverageMilliseconds());
  }
#ifdef CFG_GROUND_TRUTH
  else if (KF && m_keyDrawCamTypeKF == DRAW_CAM_KF_GT || LF && m_keyDrawCamTypeLF == DRAW_CAM_LF_GT) {
    glColor3ub(255, 255, 0);
    Viewer::DrawStringTopLeft("GT [%d] t %.3f", T.m_iFrm, T.m_t);
  }
#endif
  if (m_keyDrawViewType == DRAW_VIEW_2D) {
    const std::string fileName = 
#ifdef CFG_STEREO
      m_rightActive ? T.m_fileNameRight : 
#endif
      T.m_fileName;
    Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_18, "%s", fileName.c_str());
  } else if (m_keyDrawViewType == DRAW_VIEW_3D) {
    glPushAttrib(GL_CURRENT_BIT);
    switch (m_keyDrawBgClr) {
    case DRAW_BG_BLACK: glColor3ub(255, 255, 255);  break;
    case DRAW_BG_WHITE: glColor3ub(0, 0, 0);        break;
    }
    const Camera C = KF ? GetCameraGBALM(m_iKFActive) : GetCameraLF(m_iLFActive);
    if (KF && C.Invalid()) {
      const Rigid3D T = GetCameraKF(m_iKFActive);
      const Point3D p = T.GetPosition();
      if (T.Valid()) {
        Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, " T = %f %f %f %f",
                                  T.r00(), T.r01(), T.r02(), T.tx());
        Viewer::DrawStringTopLeft(2, GLUT_BITMAP_HELVETICA_10, "     %f %f %f %f",
                                  T.r10(), T.r11(), T.r12(), T.ty());
        Viewer::DrawStringTopLeft(3, GLUT_BITMAP_HELVETICA_10, "     %f %f %f %f",
                                  T.r20(), T.r21(), T.r22(), T.tz());
        Viewer::DrawStringTopLeft(4, GLUT_BITMAP_HELVETICA_10, " p = %f %f %f",
                                  p.x(), p.y(), p.z());
      }
    } else if (C.Valid()) {
      Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, " T = %f %f %f %f",
                                C.m_T.r00(), C.m_T.r01(), C.m_T.r02(), C.m_T.tx());
      Viewer::DrawStringTopLeft(2, GLUT_BITMAP_HELVETICA_10, "     %f %f %f %f",
                                C.m_T.r10(), C.m_T.r11(), C.m_T.r12(), C.m_T.ty());
      Viewer::DrawStringTopLeft(3, GLUT_BITMAP_HELVETICA_10, "     %f %f %f %f",
                                C.m_T.r20(), C.m_T.r21(), C.m_T.r22(), C.m_T.tz());
      Viewer::DrawStringTopLeft(4, GLUT_BITMAP_HELVETICA_10, " p = %f %f %f",
                                C.m_p.x(), C.m_p.y(), C.m_p.z());
      if (C.m_v.Valid()) {
        Viewer::DrawStringTopLeft(5, GLUT_BITMAP_HELVETICA_10, " v = %f %f %f",
                                  C.m_v.x(), C.m_v.y(), C.m_v.z());
      }
      if (C.m_ba.Valid()) {
        Viewer::DrawStringTopLeft(6, GLUT_BITMAP_HELVETICA_10, "ba = %f %f %f",
                                  C.m_ba.x(), C.m_ba.y(), C.m_ba.z());
      }
      if (C.m_bw.Valid()) {
        Viewer::DrawStringTopLeft(7, GLUT_BITMAP_HELVETICA_10, "bw = %f %f %f",
                                  C.m_bw.x(), C.m_bw.y(), C.m_bw.z());
      }
    }
    glPopAttrib();
  } else if (m_keyDrawViewType == DRAW_VIEW_PROFILE && DrawProfileTypeValid()) {
    const Camera C1 = KF ? GetCameraGBALM(m_iKFActive - 1) :
                           GetCameraLF((m_iLFActive - 1 + nLFs) % nLFs);
    const Camera C2 = KF ? GetCameraGBALM(m_iKFActive) : GetCameraLF(m_iLFActive);
    glPushAttrib(GL_CURRENT_BIT);
    glColor3ub(255, 255, 255);
    if (m_keyDrawPrfType >= DRAW_PRF_ACCELERATION &&
        m_keyDrawPrfType <= DRAW_PRF_GYROSCOPE_DEVICE) {
      const bool a = m_keyDrawPrfType <= DRAW_PRF_ACCELERATION_DEVICE;
      const bool d = m_keyDrawPrfType == DRAW_PRF_ACCELERATION_DEVICE ||
                      m_keyDrawPrfType == DRAW_PRF_GYROSCOPE_DEVICE;
      std::string str = a ? "a" : "w";
      if (d) {
        str += "'";
      }
      LA::AlignedVector3f v;
      const Rotation3D RT = C1.m_T.Rotation3D::GetTranspose();
      const LA::AlignedVector3f b = a ? C1.m_ba : C1.m_bw;
      const AlignedVector<IMU::Measurement> &us = KF ? KF->m_us : LF->m_us;
      const int Nu = us.Size();
      for (int i = 0, r = 0; i < Nu; ++i) {
        const IMU::Measurement &u = us[i];
        v = a ? u.m_a : u.m_w;
        if (d) {
          v -= b;
          if (a) {
            v = RT.GetApplied(v);
            if (!IMU_GRAVITY_EXCLUDED) {
              v.z() -= IMU_GRAVITY_MAGNITUDE;
            }
          }
        }
        if (!a) {
          v *= UT_FACTOR_RAD_TO_DEG;
        }
        Viewer::DrawStringTopLeft(++r, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f  t = %f",
                                  str.c_str(), v.x(), v.y(), v.z(), u.t());
      }
    } else if (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_ROTATION_STATE &&
               m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) {
      std::string str;
      LA::AlignedVector3f v;
      LA::AlignedMatrix3x3f S;
      const IMU::Delta D = KF ? GetIMUDeltaGBALM(m_iKFActive) : GetIMUDeltaLF(m_iLFActive);
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_IMU_DELTA_ROTATION_STATE:               str = "xdr";
        v = (D.GetRotationState(C1, C2)).GetRodrigues(BA_ANGLE_EPSILON);    break;
      case DRAW_PRF_IMU_DELTA_ROTATION_MEASUREMENT:         str = "zdr";
        v = (D.GetRotationMeasurement(C1, BA_ANGLE_EPSILON)).GetRodrigues(BA_ANGLE_EPSILON);  break;
      case DRAW_PRF_IMU_DELTA_ROTATION_ERROR:               str = "edr";
        v = D.GetRotationError(C1, C2, BA_ANGLE_EPSILON);                   break;
      case DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE:          str = "Sdr";
#ifdef CFG_IMU_FULL_COVARIANCE
        S = D.m_W[0][0].GetInverseLDL();
#else
        S = D.m_W.m_Wr.GetInverseLDL();
#endif
        break;
      case DRAW_PRF_IMU_DELTA_POSITION_STATE:       str = "xdp";
        v = D.GetPositionState(C1, C2, m_K.m_pu);   break;
      case DRAW_PRF_IMU_DELTA_POSITION_MEASUREMENT: str = "zdp";
        v = D.GetPositionMeasurement(C1);           break;
      case DRAW_PRF_IMU_DELTA_POSITION_ERROR:       str = "edp";
        v = D.GetPositionError(C1, C2, m_K.m_pu);   break;
      case DRAW_PRF_IMU_DELTA_POSITION_COVARIANCE:  str = "Sdp";
#ifdef CFG_IMU_FULL_COVARIANCE
        S = D.m_W[2][2].GetInverseLDL();
#else
        S = D.m_W.m_Wp.GetInverseLDL();
#endif
        break;
      case DRAW_PRF_IMU_DELTA_VELOCITY_STATE:       str = "xdv";
        v = D.GetVelocityState(C1, C2);             break;
      case DRAW_PRF_IMU_DELTA_VELOCITY_MEASUREMENT: str = "zdv";
        v = D.GetVelocityMeasurement(C1);           break;
      case DRAW_PRF_IMU_DELTA_VELOCITY_ERROR:       str = "edv";
        v = D.GetVelocityError(C1, C2);             break;
      case DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE:  str = "Sdv";
#ifdef CFG_IMU_FULL_COVARIANCE
        S = D.m_W[1][1].GetInverseLDL();
#else
        S = D.m_W.m_Wv.GetInverseLDL();
#endif
        break;
      case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE:      str = "xdba";
        v = C1.m_ba - C2.m_ba;                              break;
      case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_COVARIANCE: str = "Sdba";
#ifdef CFG_IMU_FULL_COVARIANCE
        S = D.m_W[3][3].GetInverseLDL();
#else
        S.MakeDiagonal(D.m_W.m_wba);
#endif
        break;
      case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE:       str = "xdbw";
        v = C1.m_ba - C2.m_ba;                            break;
      case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE:  str = "Sdbw";
#ifdef CFG_IMU_FULL_COVARIANCE
        S = D.m_W[4][4].GetInverseLDL();
#else
        S.MakeDiagonal(D.m_W.m_wbw);
#endif
        break;
      }
      const int t = m_keyDrawPrfType < DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE ?
                   (m_keyDrawPrfType - DRAW_PRF_IMU_DELTA_ROTATION_STATE) % 4 :
                  ((m_keyDrawPrfType - DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE) % 2 == 0 ? 0 : 3);
      const bool r = m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE ||
                    (m_keyDrawPrfType == DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE ||
                     m_keyDrawPrfType == DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE);
      DrawString(str, t, r, v, S);
    } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
               m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE) {
      LA::AlignedVectorXf x;
      LA::AlignedMatrixXf S;
      const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE) % 4;
      const float eps = 0.0f;
      const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
      const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
      const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
      const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
      const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
      const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                            epsba, epsba, epsba, epsbw, epsbw, epsbw};
      if (KF) {
        const int NZ = static_cast<int>(m_GBA->m_Zps.size());
        for (int iZ = 0, rowStart = 0; iZ < NZ; ++iZ) {
          const CameraPrior::Pose &Zp = m_GBA->m_Zps[iZ];
          if (Zp.m_iKFr != m_iKFActive) {
            continue;
          } else if (t == 0 || Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &x, NULL,
                                                      &m_work, _eps)) {
            DrawString(Zp, S, x, &rowStart);
          } else {
            Viewer::DrawStringTopLeft(++rowStart, GLUT_BITMAP_HELVETICA_10, "INVALID");
          }
        }
      } else if (m_LBA->m_Zp.Pose::Valid()) {
        if (t == 0 || m_LBA->m_Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &x, NULL,
                                                      &m_work, _eps)) {
          DrawString(m_LBA->m_Zp, S, x);
        } else {
          Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, "INVALID");
        }
      }
    } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE &&
               m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) {
      std::string str;
      LA::AlignedVectorXf x;
      LA::AlignedMatrixXf S;
      LA::AlignedVector3f v;
      LA::AlignedMatrix3x3f _S;
      const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE) % 4;
      const bool r = m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE;
      const float eps = 0.0f;
      const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
      const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
      const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
      const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
      const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
      const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                            epsba, epsba, epsba, epsbw, epsbw, epsbw};
      if (t == 0 || m_LBA->m_Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &x, NULL,
                                                    &m_work, _eps)) {
        const Camera C = GetCameraLF(m_LBA->m_ic2LF.front());
        const int ip = x.Size() - 9;
        switch (m_keyDrawPrfType) {
        case DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE:                            str = "xpv";
          v = m_LBA->m_Zp.GetVelocityState(C);                                break;
        case DRAW_PRF_CAMERA_PRIOR_VELOCITY_MEASUREMENT:                      str = "zpv";
          v = m_LBA->m_Zp.GetVelocityMeasurement(x.Data() + ip);              break;
        case DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR:                            str = "epv";
          v = m_LBA->m_Zp.GetVelocityError(C, x.Data() + ip);                 break;
        case DRAW_PRF_CAMERA_PRIOR_VELOCITY_COVARIANCE:                       str = "Spv";
          S.GetBlock(ip, ip, _S);                                             break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE:                   str = "xpba";
          v = m_LBA->m_Zp.GetBiasAccelerationState(C);                        break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_MEASUREMENT:             str = "zpba";
          v = m_LBA->m_Zp.GetBiasAccelerationMeasurement(x.Data() + ip + 3);  break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_ERROR:                   str = "epba";
          v = m_LBA->m_Zp.GetBiasAccelerationError(C, x.Data() + ip + 3);     break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE:              str = "Spba";
          S.GetBlock(ip + 3, ip + 3, _S);                                     break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE:                      str = "xpbw";
          v = m_LBA->m_Zp.GetBiasGyroscopeState(C);                           break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_MEASUREMENT:                str = "zpbw";
          v = m_LBA->m_Zp.GetBiasGyroscopeMeasurement(x.Data() + ip + 6);     break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_ERROR:                      str = "epbw";
          v = m_LBA->m_Zp.GetBiasGyroscopeError(C, x.Data() + ip + 6);        break;
        case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE:                 str = "Spbw";
          S.GetBlock(ip + 6, ip + 6, _S);                                     break;
        }
        DrawString(str, t, r, v, _S);
      } else {
        Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, "INVALID");
      }
    } else if (m_keyDrawPrfType == DRAW_PRF_REPROJECTION_ERROR) {
      const UT::ES<FTR::ESError, FTR::ESIndex> ES = ComputeReprojectionError(KF ? m_iKFActive : -1,
                                                                             LF ? m_iLFActive : -1);
      Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, "reprojection error = %f <= %f [%d] %d\n",
                                ES.Mean(), ES.Maximal(), ES.m_idxMax.m_ixFrm, ES.m_idxMax.m_ix);
    }
    else if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_ABSOLUTE) {
      std::string str;
      LA::AlignedVector3f v;
      if (m_keyDrawPrfType < DRAW_PRF_STATE_POSITION_ABSOLUTE) {
        Rotation3D R = KF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
        str = "r";
        const int iKFNearest = KF ? KF->m_iKFNearest : LF->m_iKFNearest;
        if (iKFNearest == -1) {
          v.Invalidate();
        } else {
          if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_RELATIVE) {
            R = R / GetCameraKF(iKFNearest);
            str = UT::String("[%d] --> [%d] ", GetKeyFrame(iKFNearest)->m_T.m_iFrm, T.m_iFrm) + str;
          }
#ifdef CFG_GROUND_TRUTH
          if (m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR) {
            R = R * GetCameraKF(iKFNearest, DRAW_CAM_KF_GT);
          }
          if (m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR ||
              m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_ABSOLUTE_ERROR) {
            R = R / (KF ? GetCameraKF(m_iKFActive, DRAW_CAM_KF_GT) :
                          GetCameraLF(m_iLFActive, DRAW_CAM_LF_GT).m_T);
            str += " - r'";
          }
#endif
          R.GetRodrigues(v, BA_ANGLE_EPSILON);
          v *= UT_FACTOR_RAD_TO_DEG;
        }
      } else if (m_keyDrawPrfType < DRAW_PRF_STATE_VELOCITY) {
        v = KF ? GetCameraKF(m_iKFActive).GetPosition() : GetCameraLF(m_iLFActive).m_p;
        str = "p";
        const int iKFNearest = KF ? KF->m_iKFNearest : LF->m_iKFNearest;
        if (iKFNearest == -1) {
          v.Invalidate();
        } else {
          if (m_keyDrawPrfType >= DRAW_PRF_STATE_POSITION_RELATIVE) {
            v -= GetCameraKF(iKFNearest).GetPosition();
            str = UT::String("[%d] --> [%d] ", GetKeyFrame(iKFNearest)->m_T.m_iFrm, T.m_iFrm) + str;
          }
#ifdef CFG_GROUND_TRUTH
          if (m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_RELATIVE_ERROR) {
            v += GetCameraKF(iKFNearest, DRAW_CAM_KF_GT).GetPosition();
          }
          if (m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_RELATIVE_ERROR ||
              m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_ABSOLUTE_ERROR) {
            v -= KF ? GetCameraKF(m_iKFActive, DRAW_CAM_KF_GT).GetPosition() :
                      GetCameraLF(m_iLFActive, DRAW_CAM_LF_GT).m_p;
            str += " - p'";
          }
#endif
        }
      } else {
        const Camera C = KF ? GetCameraGBALM(m_iKFActive) : GetCameraLF(m_iLFActive);
        if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_ACCELERATION) {
          v = C.m_v;
          str = "v";
        } else if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_GYROSCOPE) {
          v = C.m_ba;
          str = "ba";
        } else {
          v = C.m_bw;
          str = "bw";
        }
#ifdef CFG_GROUND_TRUTH
        if ((m_keyDrawPrfType - DRAW_PRF_STATE_BIAS_ACCELERATION) & 1) {
          const Camera CGT = KF ? GetCameraGBALM(m_iKFActive, DRAW_CAM_KF_GT) :
                                  GetCameraLF(m_iLFActive, DRAW_CAM_LF_GT);
          if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_ACCELERATION) {
            v -= CGT.m_v;
            str += " - v'";
          } else if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_GYROSCOPE) {
            v -= CGT.m_ba;
            str += " - ba'";
          } else {
            v -= CGT.m_bw;
            str += " - bw'";
          }
        }
#endif
        if (m_keyDrawPrfType >= DRAW_PRF_STATE_BIAS_GYROSCOPE) {
          v *= UT_FACTOR_RAD_TO_DEG;
        }
      }
      if (v.Valid()) {
        Viewer::DrawStringTopLeft(1, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f", 
                                  str.c_str(), v.x(), v.y(), v.z());
      }
    }
    glPopAttrib();
  }
  if (KF) {
    Viewer::DrawStringBottomLeft("KF = %d / %d  x = %d / %d  z = %d", m_iKFActive, nKFs, 
                                 KF->m_xs.size(), m_iKF2d.back(), KF->m_zs.size());
  }
  else {
    Viewer::DrawStringBottomLeft("KF = %d  LF = %d / %d  z = %d", nKFs,
                                 (m_iLFActive + nLFs - (m_iLF + 1)) % nLFs, nLFs, LF->m_zs.size());
  }
}

void ViewerIBA::DrawString(const std::string str, const int t, const bool r,
                           LA::AlignedVector3f &v, LA::AlignedMatrix3x3f &S,
                           int *rowStart) {
  const int row = rowStart ? *rowStart : 0;
  if (t < 3) {
    if (r) {
      v *= UT_FACTOR_RAD_TO_DEG;
    }
    Viewer::DrawStringTopLeft(row + 1, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f", str.c_str(),
                              v.x(), v.y(), v.z());
    if (t == 2) {
      Viewer::DrawStringCurrent(GLUT_BITMAP_HELVETICA_10, " = %f", sqrtf(v.SquaredLength()));
    }
    if (rowStart) {
      ++*rowStart;
    }
  } else if (S.Valid()) {
    if (r) {
      S *= UT_FACTOR_RAD_TO_DEG * UT_FACTOR_RAD_TO_DEG;
    }
    S.GetDiagonal(v);
    v.MakeSquareRoot();
#if 0
    const std::string _str(Viewer::GetStringWidth(GLUT_BITMAP_HELVETICA_10, str.c_str()) /
                           Viewer::GetCharacterWidth(GLUT_BITMAP_HELVETICA_10, ' '), ' ');
    Viewer::DrawStringTopLeft(row + 1, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f (%f)",  str.c_str(),
                              S.m00(), S.m01(), S.m02(), v.x());
    Viewer::DrawStringTopLeft(row + 2, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f (%f)", _str.c_str(),
                              S.m10(), S.m11(), S.m12(), v.y());
    Viewer::DrawStringTopLeft(row + 3, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f (%f)", _str.c_str(),
                              S.m20(), S.m21(), S.m22(), v.z());
    if (rowStart) {
      *rowStart += 3;
    }
#else
    Viewer::DrawStringTopLeft(row + 1, GLUT_BITMAP_HELVETICA_10, "%s = %f %f %f = %f", str.c_str(),
                              v.x(), v.y(), v.z(), sqrtf(v.SquaredLength()));
    if (rowStart) {
      ++*rowStart;
    }
#endif
  }
}

void ViewerIBA::DrawString(const CameraPrior::Pose &Zp, const LA::AlignedMatrixXf &S,
                           const LA::AlignedVectorXf &x, int *rowStart) {
#ifdef CFG_DEBUG
  UT_ASSERT(Zp.Valid());
  UT_ASSERT(m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
            m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE);
#endif
  std::string str;
  LA::AlignedVector3f v;
  LA::AlignedMatrix3x3f _S;
  const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE) % 4;
  const bool r = m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE;
  const int iFrm1 = GetKeyFrame(Zp.m_iKFr)->m_T.m_iFrm;
  const Rigid3D C1 = GetCameraKF(Zp.m_iKFr);
  const int Nk = static_cast<int>(Zp.m_iKFs.size());
  int row = rowStart ? *rowStart : 0;
  for (int i = -1, ip = 0; i < Nk; ip = ip + (i == -1 ? 2 : 6), ++i) {
    if (i == -1) {
      if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_POSITION_STATE) {
        continue;
      }
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE:                              str = "xr";
        v = Zp.GetReferenceRotationState(C1).GetRodrigues(BA_ANGLE_EPSILON);  break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT:                        str = "zr"; {
        const Rotation3D eR = Zp.GetReferenceRotationMeasurement(x.Data(), BA_ANGLE_EPSILON);
        v = eR.GetRodrigues(BA_ANGLE_EPSILON);                                break; }
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR:                              str = "er";
        v = Zp.GetReferenceRotationError(C1, x.Data(), BA_ANGLE_EPSILON);     break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE:                         str = "Sr"; {
        LA::SymmetricMatrix2x2f Srr;
        S.GetBlockDiagonal(ip, Srr);
        _S.MakeDiagonal(Srr, 0.0f);
        break; }
      }
      str = UT::String("[%d] ", iFrm1) + str;
    } else {
      const int iKF2 = Zp.m_iKFs[i];
      const int iLF = iKF2 == INT_MAX ? m_LBA->m_ic2LF.front() : -1;
      const Rigid3D C2 = iKF2 == INT_MAX ? GetCameraLF(iLF).m_T : GetCameraKF(iKF2);
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE:                            str = "xpr";
        v = Zp.GetRotationState(C1, C2).GetRodrigues(BA_ANGLE_EPSILON);     break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT:                      str = "zpr"; {
        const Rotation3D R = Zp.GetRotationMeasurement(i, x.Data() + ip + 3, BA_ANGLE_EPSILON);
        v = R.GetRodrigues(BA_ANGLE_EPSILON);                               break; }
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR:                            str = "epr"; {
        v = Zp.GetRotationError(C1, C2, i, x.Data() + ip + 3,
                                BA_ANGLE_EPSILON);                          break; }
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE:                       str = "Spr";
        S.GetBlock(ip + 3, ip + 3, _S);                                     break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_STATE:                            str = "xpp";
        v = Zp.GetPositionState(C1, C2);                                    break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_MEASUREMENT:                      str = "zpp";
        v = Zp.GetPositionMeasurement(i, x.Data() + ip);                    break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR:                            str = "epp";
        v = Zp.GetPositionError(C1, C2, i, x.Data() + ip);                  break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE:                       str = "Spp";
        S.GetBlock(ip, ip, _S);                                             break;
      }
      str = UT::String("[%d] --> [%d] ", iFrm1, (iKF2 == INT_MAX ? GetLocalFrame(iLF) :
                                                                   GetKeyFrame(iKF2))->m_T.m_iFrm) + str;
    }
    DrawString(str, t, r, v, _S, &row);
  }
  if (rowStart) {
    *rowStart = row;
  }
}

void ViewerIBA::DrawTimeLine() {
  if (m_keyDrawTlnType == DRAW_TLN_NONE) {
    return;
  }
  const float alphaMin = 0.1f, alphaMax = 0.8f;
  const int iKFr = DrawTimeLine(&m_alphas, alphaMin, alphaMax);
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames(), nFrms = nKFs + nLFs;
  const int W = m_pWnd->size().x, H = m_pWnd->size().y;
  const int wh = VW_STRING_BORDER_X, h = VW_STRING_BORDER_Y + VW_STRING_GAP_Y, w = wh + wh;
  const float ratio = static_cast<float>(W - w) / (nFrms - 1);
  const Point2D sizeQuad(ratio * 0.5f, h);
  float x = static_cast<float>(wh);
  float y = H - 1 - (VW_STRING_BORDER_Y + VW_STRING_GAP_Y + VW_STRING_HEIGHT + h);
  glLineWidth(1.0f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (int iFrm = 0; iFrm < nFrms; ++iFrm, x += ratio) {
    if (m_alphas[iFrm] == 0.0f) {
      continue;
    } else if (m_alphas[iFrm] < 0.0f) {
      glColor4f(1.0f, 0.0f, 0.0f, alphaMax);
    } else {
      glColor4f(1.0f, 0.0f, 1.0f, m_alphas[iFrm]);
    }
    Viewer::DrawQuad(x, y, sizeQuad);
  }
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glColor3ub(255, 255, 0);
  if (iKFr != -1) {
    x = wh + iKFr * ratio;
    glVertex2f(x, y - h);
    glVertex2f(x, y + h);
  }
  glEnd();
  if (m_iFtrActive.Valid()) {
    //glColor3ub(0, 0, 255);
    glColor4ub(0, 0, 255, 63);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
      if (iKF != m_iFtrActive.m_ix.m_iKF &&
          GetKeyFrame(iKF)->SearchFeatureMeasurement(m_iFtrActive.m_ix.m_iKF,
                                                     m_iFtrActive.m_ix.m_ix) == -1) {
        continue;
      }
      x = wh + iKF * ratio;
      Viewer::DrawQuad(x, y, sizeQuad);
    }
    for (int iFrm = nKFs, iLF = nLFs == 0 ? 0 : (m_iLF + 1) % nLFs; iFrm < nFrms;
         ++iFrm, iLF = (iLF + 1) % nLFs) {
      if (GetLocalFrame(iLF)->SearchFeatureMeasurement(m_iFtrActive.m_ix.m_iKF,
                                                       m_iFtrActive.m_ix.m_ix) == -1) {
        continue;
      }
      x = wh + iFrm * ratio;
      Viewer::DrawQuad(x, y, sizeQuad);
    }

    glColor3ub(255, 255, 255);
    x = wh + m_iFtrActive.m_ix.m_iKF * ratio;
    Viewer::DrawBox(x, y, sizeQuad);
    if (m_iFtrActive.m_iz.Valid()) {
      if (m_iFtrActive.m_iz.m_iKF != -1) {
        x = wh + m_iFtrActive.m_iz.m_iKF * ratio;
      } else {
        x = wh + (nKFs + (m_iFtrActive.m_iz.m_iLF + nLFs - m_iLF - 1) % nLFs) * ratio;
      }
      Viewer::DrawBox(x, y, sizeQuad);
    }
    if (m_iFtrActive.m_ic.Valid()) {
      x = wh + (nFrms - 1) * ratio;
      Viewer::DrawBox(x, y, sizeQuad);
    }
  }
  glDisable(GL_BLEND);

  glColor3ub(0, 255, 0);
  glBegin(GL_LINES);
  glVertex2i(0, y);
  glVertex2i(W, y);
  x = wh + (nKFs - 0.5f) * ratio;
  glVertex2f(x, y - h);
  glVertex2f(x, y + h);
  glEnd();

  if (m_dragingActiveFrm) {
    glColor3ub(0, 0, 255);
  }
  x = wh + m_iFrmActive * ratio;
  glBegin(GL_TRIANGLES);
  glVertex2f(x, y);
  y -= h;
  x += wh;
  glVertex2f(x, y);
  x -= w;
  glVertex2f(x, y);
  glEnd();
}

int ViewerIBA::DrawTimeLine(std::vector<float> *alphas, const float alphaMin,
                            const float alphaMax, const bool localFrm) {
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames(), nFrms = nKFs + (localFrm ? nLFs : 0);
  alphas->assign(nFrms, 0.0f);
  const bool activeKF = m_iFrmActive < nKFs;
  if (m_keyDrawTlnType == DRAW_TLN_FEATURE_MATCH) {
    if (activeKF) {
      const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(m_iKFActive);
      const int Nk = static_cast<int>(KF.m_iKFsMatch.size());
      for (int ik = 0; ik < Nk; ++ik) {
        const int iKF = KF.m_iKFsMatch[ik];
        const int nMatches = CountFeatureMatchesKFKF(iKF, m_iKFActive);
        if (nMatches == 0) {
          continue;
        }
        const float alpha = nMatches / m_keyDrawTlnMaxFtrMatches;
        alphas->at(iKF) = UT_CLAMP(alpha, alphaMin, alphaMax);
      }
      for (int iFrm = nKFs, iLF = nLFs == 0 ? 0 : (m_iLF + 1) % nLFs; iFrm < nFrms;
           ++iFrm, iLF = (iLF + 1) % nLFs) {
        const int nMatches = CountFeatureMatchesKFLF(m_iKFActive, iLF);
        if (nMatches == 0) {
          continue;
        }
        const float alpha = nMatches / m_keyDrawTlnMaxFtrMatches;
        alphas->at(iFrm) = UT_CLAMP(alpha, alphaMin, alphaMax);
      }
      return KF.m_iKFNearest;
    } else {
      const LocalFrame &LF = *(LocalFrame *) GetLocalFrame(m_iLFActive);
      const int Nk = static_cast<int>(LF.m_iKFsMatch.size());
      for (int ik = 0; ik < Nk; ++ik) {
        const int iKF = LF.m_iKFsMatch[ik];
        const int nMatches = CountFeatureMatchesKFLF(iKF, m_iLFActive);
        if (nMatches == 0) {
          continue;
        }
        const float alpha = nMatches / m_keyDrawTlnMaxFtrMatches;
        alphas->at(iKF) = UT_CLAMP(alpha, alphaMin, alphaMax);
      }
      for (int iFrm = nKFs, iLF = nLFs == 0 ? 0 : (m_iLF + 1) % nLFs; iFrm < nFrms;
           ++iFrm, iLF = (iLF + 1) % nLFs) {
        const int nMatches = CountFeatureMatchesLFLF(iLF, m_iLFActive);
        if (nMatches == 0) {
          continue;
        }
        const float alpha = nMatches / m_keyDrawTlnMaxFtrMatches;
        alphas->at(iFrm) = UT_CLAMP(alpha, alphaMin, alphaMax);
      }
      return LF.m_iKFNearest;
    }
  } else {
    LA::AlignedMatrixXf S;
    const float eps = 0.0f;
    const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
    const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
    const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
    const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
    const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
    const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                          epsba, epsba, epsba, epsbw, epsbw, epsbw};
    if (activeKF) {
      const int NZ = static_cast<int>(m_GBA->m_Zps.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const CameraPrior::Pose &Zp = m_GBA->m_Zps[iZ];
        if (Zp.m_iKFr == m_iKFActive) {
          Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, NULL, NULL, &m_work, _eps);
          DrawTimeLine(Zp, S, alphas, alphaMin, alphaMax);
        }
      }
      return -1;
    } else {
      alphas->push_back(0.0f);
      const CameraPrior::Joint &Zp = m_LBA->m_Zp;
      if (Zp.Pose::Valid()) {
        Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, NULL, NULL, &m_work, _eps);
        DrawTimeLine(Zp, S, alphas, alphaMin, alphaMax);
      }
      return Zp.m_iKFr;
    }
  }
}

void ViewerIBA::DrawTimeLine(const CameraPrior::Pose &Zp, const LA::AlignedMatrixXf &S,
                             std::vector<float> *alphas, const float alphaMin,
                             const float alphaMax) {
  float alpha;
  LA::SymmetricMatrix2x2f Sr, Wr;
  LA::AlignedMatrix6x6f Sc, Wc;
  const float dp = m_keyDrawTlnPriorVarPos;
  const float dr = m_keyDrawTlnPriorVarRot * UT_FACTOR_DEG_TO_RAD;
  const float X2 = -logf(m_keyDrawCovProb) * m_keyDrawCovScale;
  const int nKFs = GetKeyFrames(), Nk = static_cast<int>(Zp.m_iKFs.size());
  if (!S.Empty()) {
    if (alphas->at(Zp.m_iKFr) >= 0.0f) {
      S.GetBlockDiagonal(0, Sr);
      Sr *= X2;
      if (Sr.GetInverse(Wr)) {
        alpha = LA::SymmetricMatrix2x2f::MahalanobisDistance(Wr, dr);
        alphas->at(Zp.m_iKFr) += UT_CLAMP(alpha, alphaMin, alphaMax);
      } else {
        alphas->at(Zp.m_iKFr) = -1.0f;
      }
    }
    for (int i = 0, ip = 2; i < Nk; ++i, ip += 6) {
      const int iKF = Zp.m_iKFs[i], iFrm = iKF == INT_MAX ? nKFs : iKF;
      if (alphas->at(iFrm) < 0.0f) {
        continue;
      }
      S.GetBlock(ip, ip, Sc);
      Sc *= X2;
//#ifdef CFG_DEBUG
#if 0
//#if 1
      UT::PrintSeparator();
      S.Print(true);
      UT::PrintSeparator();
      Sc.Print(true);
#endif
      if (Sc.GetInverseLDL(Wc)) {
        alpha = LA::AlignedMatrix6x6f::MahalanobisDistance(Wc, dp, dr);
        alphas->at(iFrm) += UT_CLAMP(alpha, alphaMin, alphaMax);
      } else {
        alphas->at(iFrm) = -1.0f;
      }
    }
  } else {
    alphas->at(Zp.m_iKFr) = -1.0f;
    for (int i = 0; i < Nk; ++i) {
      const int iKF = Zp.m_iKFs[i], iFrm = iKF == INT_MAX ? nKFs : iKF;
      alphas->at(iFrm) = -1.0f;
    }
  }
}

void ViewerIBA::Draw2DImage() {
#ifdef CFG_DEBUG
  UT_ASSERT(m_keyDrawViewType == DRAW_VIEW_2D);
#endif
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  Update2DImageTexture();
#ifdef CFG_STEREO
  if (m_rightActive) {
    Viewer::DrawTexture(m_texRGBr);
  } else
#endif
  {
    Viewer::DrawTexture(m_texRGB);
  }
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
}

void ViewerIBA::Draw2DFeatures() {
  if (m_keyDrawFtrType == DRAW_FTR_NONE) {
    return;
  }
  const bool activeKF = m_iFrmActive < GetKeyFrames();
  const LocalFrame &LF = *(LocalFrame *) GetLocalFrame(m_iLFActive);
  glLineWidth(1.0f);
  if (activeKF) {
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(m_iKFActive);
    //glColor3ub(0, 0, 255);
    glColor3ub(0, 255, 255);
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      Draw2DFeature(FeatureIndex::Source(m_iKFActive, ix));
    }
  }
  const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : &LF);
  const int NZ = int(F.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const bool _activeKF = !activeKF && iZ == NZ - 1 && GetKeyFrame(Z.m_iKF)->m_T == LF.m_T;
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const FTR::Measurement &z = F.m_zs[iz];
      if (_activeKF) {
        glColor3ub(0, 255, 255);
      } else {
        glColor3ub(0, 255, 0);
      }
      Draw2DFeature(FeatureIndex::Source(Z.m_iKF, z.m_ix), iz);
    }
  }
}

void ViewerIBA::Draw2DFeature(const FeatureIndex::Source &ix, const int iz) {
#ifdef CFG_DEBUG
  UT_ASSERT(m_keyDrawViewType == DRAW_VIEW_2D);
#endif
  const int nKFs = GetKeyFrames();
  const bool activeKF = m_iFrmActive < nKFs, activeCF = !activeKF && m_iLFActive == m_iLF;
  const FRM::Frame *F = activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive);
  const KeyFrame *KF = activeKF ? (KeyFrame *) F : NULL;
  const LocalFrame *LF = activeKF ? NULL : (LocalFrame *) F;
#ifdef CFG_STEREO
  if (iz == -1 && m_rightActive && KF->m_xs[ix.m_ix].m_xr.Invalid()
   || iz != -1 && (!m_rightActive && F->m_zs[iz].m_z.Invalid()
                 || m_rightActive && F->m_zs[iz].m_zr.Invalid())) {
    return;
  }
#endif
  const bool activePch = /*!activeCF && */m_iFtrActive.m_ix == ix || activeCF &&
                                          m_iFtrActive.m_ic.m_iz == iz;
  Point2D x;
  if (iz == -1) {
    const FTR::Source &_x = KF->m_xs[ix.m_ix];
#ifdef CFG_STEREO
    if (m_rightActive) {
      x = _x.m_xr;
    } else
#endif
    {
      x = _x.m_x;
    }
  }
  else {
#ifdef CFG_STEREO
    if (m_rightActive) {
      x = F->m_zs[iz].m_zr;
    } else
#endif
    {
      x = F->m_zs[iz].m_z;
    }
  }
  if (m_keyDrawFtrType == DRAW_FTR_MATCH && m_iFrmActiveLast != -1) {
    Point2D _x;
    _x.Invalidate();
    const bool _activeKF = m_iFrmActiveLast < nKFs;
    const int _iLFActive = (m_iFrmActiveLast - nKFs + m_iLF + 1) % GetLocalFrames();
    if (_activeKF && m_iFrmActiveLast == ix.m_iKF) {
      const FTR::Source &__x = ((KeyFrame *) GetKeyFrame(ix.m_iKF))->m_xs[ix.m_ix];
#ifdef CFG_STEREO
      if (m_rightActiveLast) {
        _x = __x.m_xr;
      } else
#endif
      {
        _x = __x.m_x;
      }
    }
    else {
      const FRM::Frame &_F = *(_activeKF ? GetKeyFrame(m_iFrmActiveLast) : GetLocalFrame(_iLFActive));
      const int _iz = _F.SearchFeatureMeasurement(ix.m_iKF, ix.m_ix);
      if (_iz != -1) {
#ifdef CFG_STEREO
        if (m_rightActiveLast) {
          _x = _F.m_zs[_iz].m_zr;
        } else
#endif
        {
          _x = _F.m_zs[_iz].m_z;
        }
      }
    }
    if (_x.Invalid()) {
      return;
    }
    glPushAttrib(GL_CURRENT_BIT);
    glColor3ub(255, 255, 255);
    glBegin(GL_LINES);
    glVertex2fv(x);
    glVertex2fv(_x);
    glEnd();
    glPopAttrib();
  }
  Viewer::DrawCross(x, m_sizeCross);
  if (m_keyDrawViewType == DRAW_VIEW_2D
  && (m_keyDrawErrType2D == DRAW_ERR_ALL || m_keyDrawErrType2D == DRAW_ERR_COVARIANCE)
  && (iz != -1
#ifdef CFG_STEREO
   || iz == -1 && m_rightActive
#endif
   )) {
    Point2DCovariance S;
#ifdef CFG_STEREO
    if (iz == -1) {
      KF->m_xs[ix.m_ix].m_Wr.GetInverse(S);
    } else if (m_rightActive) {
      F->m_zs[iz].m_Wr.GetInverse(S);
    } else
#endif
    {
      F->m_zs[iz].m_W.GetInverse(S);
    }
    Viewer::DrawCovariance(x, S, -logf(m_keyDrawCovProb) * m_keyDrawCovScale);
  }
  if (activePch) {
    const Point2D size = m_sizeBox * m_keyDrawPchSize;
    Viewer::DrawBox(x.x(), x.y(), size);
    m_drawPch = true;
  }
  SearchActiveFeature(x, ix.m_iKF, ix.m_ix, iz);
}

void ViewerIBA::Draw2DProjections() {
  if (m_keyDrawPrjType == DRAW_PRJ_NONE) {
    return;
  }
  glPointSize(3.0f);
  const int nKFs = GetKeyFrames();
  const bool activeKF = m_iFrmActive < nKFs, activeCF = !activeKF && m_iLFActive == m_iLF;
  const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive));
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
  const float dotMin = cosf(VW_PROJECTION_MAX_VIEW_ANGLE);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (iKF == m_iFrmActive
#ifdef CFG_STEREO
     && !m_rightActive
#endif
     ) {
      continue;
    }
    const bool prj = C.DotOrientation(GetCameraKF(iKF)) > dotMin;
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(iKF);
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const int iz = F.SearchFeatureMeasurement(iKF, ix);
      const bool trk = iz != -1
#ifdef CFG_STEREO
        && (!m_rightActive && F.m_zs[iz].m_z.Valid() || m_rightActive && F.m_zs[iz].m_zr.Valid())
        || activeKF && iKF == m_iKFActive && m_rightActive && KF.m_xs[ix].m_xr.Valid()
#endif
        ;
      if ((m_keyDrawPrjType == DRAW_PRJ_ALL || m_keyDrawPrjType == DRAW_PRJ_TRACKED) && trk) {
        glColor3ub(255, 255, 0);
        Draw2DProjection(iKF, ix, iz);
      }
      if ((m_keyDrawPrjType == DRAW_PRJ_ALL || m_keyDrawPrjType == DRAW_PRJ_UNTRACKED) &&
          !trk && prj) {
        glColor3ub(0, 255, 255);
        Draw2DProjection(iKF, ix, iz);
      }
    }
  }
}

void ViewerIBA::Draw2DProjection(const int iKF, const int ix, const int iz) {
  const Depth::InverseGaussian d = GetFeatureDepth(iKF, ix);
  if (d.Invalid()) {
    return;
  }
  const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(iKF);
  Point2D x, xs[4];
  LA::Vector2f Jx;
  const bool activeKF = m_iFrmActive < GetKeyFrames();
#ifdef CFG_STEREO
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive, -1, m_rightActive) :
                               GetCameraLF(m_iLFActive, -1, m_rightActive).m_T;
#else
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
#endif
  const Rigid3D _C = GetCameraKF(iKF);
  if (!d.Project(C / _C, KF.m_xs[ix].m_x, m_Bn, x, Jx)) {
    return;
  }
  const bool range = m_iFtrActive.m_ix.Equal(iKF, ix) ||
                    (m_keyDrawErrType2D == DRAW_ERR_ALL ||
                     m_keyDrawErrType2D == DRAW_ERR_COVARIANCE) &&
                     d.s2() >= m_keyDrawDepVar * m_keyDrawDepVar;
  if (range) {
    Get2DProjectionRange(x, Jx, d.s2(), xs);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2fv(xs[0]);
    glVertex2fv(xs[1]);
    glVertex2fv(xs[2]);
    glVertex2fv(xs[3]);
    glEnd();
  }
  const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive));
#ifdef CFG_STEREO
  const bool vxr = activeKF && iKF == m_iKFActive && m_rightActive && KF.m_xs[ix].m_xr.Valid();
#endif
  const bool vz = iz != -1
#ifdef CFG_STEREO
         && (!m_rightActive && F.m_zs[iz].m_z.Valid() || m_rightActive && F.m_zs[iz].m_zr.Valid())
         || vxr
#endif
         ;
  if (m_iFtrActive.m_ix.Equal(iKF, ix)) {
    //glLineWidth(2.0f);
    const Point2D size = m_sizeBox * m_keyDrawPchSize;
    Viewer::DrawBox(x.x(), x.y(), size);
    m_drawPch = true;
  }
  if ((m_keyDrawErrType2D == DRAW_ERR_ALL || m_keyDrawErrType2D == DRAW_ERR_MEAN ||
       m_iFtrActive.m_ix.Equal(iKF, ix)) && vz) {
    int clr[4];
    glGetIntegerv(GL_CURRENT_COLOR, clr);
    //glPushAttrib(GL_CURRENT_BIT);
    glColor3ub(255, 0, 0);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glVertex2fv(x);
    Point2D z;
#ifdef CFG_STEREO
    if (vxr) {
      z = KF.m_xs[ix].m_xr;
    } else if (vz && m_rightActive) {
      z = F.m_zs[iz].m_zr;
    } else
#endif
    {
      z = F.m_zs[iz].m_z;
    }
    glVertex2fv(z);
    glEnd();
    //glPopAttrib();
    glColor4iv(clr);
  }
  //if (!range)
  {
    glBegin(GL_POINTS);
    glVertex2fv(x);
    glEnd();
  }
  SearchActiveFeature(x, iKF, ix, iz);
}

void ViewerIBA::Get2DProjectionRange(const Point2D &x, const LA::Vector2f &Jx, const float s2,
                                        Point2D *xs) {
  const float X2 = -logf(m_keyDrawCovProb) * m_keyDrawCovScale, sx2 = X2 * s2;
  const LA::Vector2f dx = Jx * sqrtf(sx2);
  const float sy2Max = m_sizeCross.SquaredLength() / Jx.SquaredLength();
  const float sy2 = std::min(sx2, sy2Max);
  const LA::Vector2f dy = LA::Vector2f(-Jx.y(), Jx.x()) * sqrtf(sy2);
  xs[0] = x - dx;
  xs[1] = x + dx;
  xs[2] = x - dy;
  xs[3] = x + dy;
}

template<typename TYPE_SRC, typename TYPE_DST>
static inline void Rectify(const Intrinsic::RectificationMap &RM, const CVD::Image<TYPE_SRC> &Id,
                           CVD::Image<TYPE_DST> &Ir) {
  if (UT::ImageInvalid(Id)) {
    UT::ImageInvalidate(Ir);
    return;
  }
  const int w = Id.size().x, h = Id.size().y;
  const TYPE_DST iInv = UT::Invalid<TYPE_DST>();
  Ir.resize(Id.size());
  for (int yr = 0, i = 0; yr < h; ++yr)
  for (int xr = 0; xr < w; ++xr, ++i) {
    const int ix1 = RM.m_xs[i].x();
    if (ix1 == -1) {
      Ir[yr][xr] = iInv;
    } else {
      const int ix2 = ix1 + 1, iy1 = RM.m_xs[i].y(), iy2 = iy1 + 1;
      UT::ImageInterpolate(Id[iy1][ix1], Id[iy2][ix1], Id[iy1][ix2], Id[iy2][ix2], RM.m_ws[i], Ir[yr][xr]);
    }
  }
}

void ViewerIBA::Update2DImageTexture() {
  const bool activeKF = m_iFrmActive < GetKeyFrames();
  const LocalFrame &LF = *(LocalFrame *) GetLocalFrame(m_iLFActive);
  const FRM::Tag &T = activeKF ? GetKeyFrame(m_iKFActive)->m_T : LF.m_T;
#ifdef CFG_STEREO
  if (!m_rightActive && m_iFrmTexRGB != T.m_iFrm &&
      UT::ImageLoad(T.m_fileName, m_imgRGB, m_imgRGBTmp)) {
    if (m_K.m_K.NeedRectification()) {
      Rectify(m_RM, m_imgRGB, m_imgRGBTmp);
      UT::ImageSwap(m_imgRGB, m_imgRGBTmp);
    }
    m_texRGB.Bind();
    m_texRGB.UploadFromCPU((ubyte *) m_imgRGB.data());
    m_iFrmTexRGB = T.m_iFrm;
  } else if (m_rightActive && m_iFrmTexRGBr != T.m_iFrm &&
             UT::ImageLoad(T.m_fileNameRight, m_imgRGBr, m_imgRGBTmp)) {
    if (m_K.m_Kr.NeedRectification()) {
      Rectify(m_RMr, m_imgRGBr, m_imgRGBTmp);
      UT::ImageSwap(m_imgRGBr, m_imgRGBTmp);
    }
    m_texRGBr.Bind();
    m_texRGBr.UploadFromCPU((ubyte *) m_imgRGBr.data());
    m_iFrmTexRGBr = T.m_iFrm;
  }
#else
  if (m_iFrmTexRGB != T.m_iFrm && UT::ImageLoad(T.m_fileName, m_imgRGB, m_imgRGBTmp)) {
    if (m_K.m_K.NeedRectification()) {
      Rectify(m_RM, m_imgRGB, m_imgRGBTmp);
      UT::ImageSwap(m_imgRGB, m_imgRGBTmp);
    }
    m_texRGB.Bind();
    m_texRGB.UploadFromCPU((ubyte *) m_imgRGB.data());
    m_iFrmTexRGB = T.m_iFrm;
  }
#endif
}

void ViewerIBA::Draw3DCameras() {
  glPushAttrib(GL_DEPTH_BUFFER_BIT);
  //glDisable(GL_DEPTH_TEST);
  glDepthFunc(GL_ALWAYS);
  glLineWidth(1.0f);
  const int nKFs = GetKeyFrames();
  const bool activeKF = m_iFrmActive < nKFs;
  const float alphaMin = 0.3f, alphaMax = 1.0f;
  if (activeKF) {
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(m_iKFActive);
    if (m_keyDrawMotTypeKF == DRAW_MOT_KF_TRAJECTORY) {
      glColor3ub(0, 0, 255);
      glPushAttrib(GL_LINE_BIT);
      glLineWidth(2.0f);
      glBegin(GL_LINE_STRIP);
      for (int iKF = 0; iKF <= m_iKFActive; ++iKF) {
#ifdef CFG_STEREO
        const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
        const Rigid3D C = GetCameraKF(iKF);
#endif
        if (C.Invalid()) {
          continue;
        }
        const Point3D p = C.GetPosition();
        glVertex3fv(p);
      }
      glEnd();
      glPopAttrib();
#ifdef CFG_GROUND_TRUTH
      if (m_keyDrawCamGT) {
        glColor3ub(255, 255, 0);
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(1.0f);
        //glLineWidth(2.0f);
        glBegin(GL_LINE_STRIP);
        for (int iKF = 0; iKF <= m_iKFActive; ++iKF) {
#ifdef CFG_STEREO
          const Rigid3D C = GetCameraKF(iKF, DRAW_CAM_KF_GT, m_rightActive);
#else
          const Rigid3D C = GetCameraKF(iKF, DRAW_CAM_KF_GT);
#endif
          if (C.Invalid()) {
            continue;
          }
          const Point3D p = C.GetPosition();
          glVertex3fv(p);
        }
        glEnd();
        glPopAttrib();
      }
#endif
    } else if (m_keyDrawMotTypeKF == DRAW_MOT_KF_POSE) {
      glColor3ub(0, 0, 255);
      for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
        const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
        const Rigid3D C = GetCameraKF(iKF);
#endif
        if (C.Invalid()) {
          continue;
        }
        m_frustrum.SetPose(C, false);
        m_frustrum.Draw(false);
      }
    } else if (m_keyDrawMotTypeKF == DRAW_MOT_KF_POSE_NEIGHBOR) {
      const int iKFr = DrawTimeLine(&m_alphas, alphaMin, alphaMax, false);
      Draw3DCameras(m_alphas, alphaMin, iKFr);
    }
  } else {
    const int nLFs = GetLocalFrames();
    if (m_keyDrawMotTypeLF == DRAW_MOT_LF_TRAJECTORY) {
      glColor3ub(0, 0, 255);
#if 0
      for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
        const Rigid3D C = GetCameraKF(iKF, -1, right);
#else
        const Rigid3D C = GetCameraKF(iKF);
#endif
        if (C.Invalid()) {
          continue;
        }
        m_frustrum.SetPose(C, false);
        m_frustrum.Draw(false);
      }
#else
      glPushAttrib(GL_LINE_BIT);
      glLineWidth(2.0f);
      glBegin(GL_LINE_STRIP);
      for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
        const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
        const Rigid3D C = GetCameraKF(iKF);
#endif
        if (C.Invalid()) {
          continue;
        }
        const Point3D p = C.GetPosition();
        glVertex3fv(p);
      }
      glEnd();
      glPopAttrib();
#ifdef CFG_GROUND_TRUTH
      if (m_keyDrawCamGT) {
        glColor3ub(255, 255, 0);
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(1.0f);
        //glLineWidth(2.0f);
        glBegin(GL_LINE_STRIP);
        for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
          const Rigid3D C = GetCameraKF(iKF, DRAW_CAM_KF_GT, m_rightActive);
#else
          const Rigid3D C = GetCameraKF(iKF, DRAW_CAM_KF_GT);
#endif
          if (C.Invalid()) {
            continue;
          }
          const Point3D p = C.GetPosition();
          glVertex3fv(p);
        }
        glEnd();
        glPopAttrib();
      }
#endif
#endif
    }
    if (m_keyDrawMotTypeLF != DRAW_MOT_LF_NONE) {
      glPushAttrib(GL_LINE_BIT);
      glLineWidth(2.0f);
      //glColor3ub(255, 127, 0);
      glColor3ub(205, 102, 0);
      glBegin(GL_LINE_STRIP);
      for (int iLF = (m_iLF + 1) % nLFs; ; iLF = (iLF + 1) % nLFs) {
#ifdef CFG_STEREO
        const Camera C = GetCameraLF(iLF, -1, m_rightActive);
#else
        const Camera C = GetCameraLF(iLF);
#endif
        if (C.Valid()) {
          glVertex3fv(C.m_p);
        }
        if (iLF == m_iLFActive) {
          break;
        }
      }
      glEnd();
      glPopAttrib();
#ifdef CFG_GROUND_TRUTH
      if (m_keyDrawCamGT) {
        glColor3ub(255, 255, 0);
        glPushAttrib(GL_LINE_BIT);
        glLineWidth(1.0f);
        //glLineWidth(2.0f);
        glBegin(GL_LINE_STRIP);
        for (int iLF = (m_iLF + 1) % nLFs; ; iLF = (iLF + 1) % nLFs) {
#ifdef CFG_STEREO
          const Camera C = GetCameraLF(iLF, DRAW_CAM_LF_GT, m_rightActive);
#else
          const Camera C = GetCameraLF(iLF, DRAW_CAM_LF_GT);
#endif
          if (C.Valid()) {
            glVertex3fv(C.m_p);
          }
          if (iLF == m_iLFActive) {
            break;
          }
        }
        glEnd();
        glPopAttrib();
      }
#endif
    }
    if (m_keyDrawCamVelocity) {
      glColor3ub(0, 255, 255);
      glBegin(GL_LINES);
      float dt;
      Point3D p1, p2;
      for (int iLF1 = (m_iLF + 1) % nLFs, iLF2 = int(iLF1 + 1) % nLFs; ;
           iLF1 = iLF2, iLF2 = (iLF2 + 1) % nLFs) {
        const Camera C = GetCameraLF(iLF1);
        if (C.Invalid() || C.m_v.Invalid()) {
          continue;
        }
        C.m_T.ApplyRotationInversely(m_K.m_pu, p1);
        p1 += C.m_p;
        const float t1 = GetLocalFrame(iLF1)->m_T.m_t;
        if (iLF1 == m_iLF) {
          dt = t1 - GetLocalFrame((iLF1 - 1 + nLFs) % nLFs)->m_T.m_t;
        } else {
          dt = GetLocalFrame(iLF2)->m_T.m_t - t1;
        }
        C.m_v.GetScaled(dt, p2);
        p2 += p1;
        glVertex3fv(p1);
        glVertex3fv(p2);
        if (iLF1 == m_iLFActive) {
          break;
        }
      }
      glEnd();
    }
    if (m_keyDrawMotTypeLF == DRAW_MOT_LF_TRAJECTORY_NEIGHBOR) {
      const int iKFr = DrawTimeLine(&m_alphas, alphaMin, alphaMax, false);
      Draw3DCameras(m_alphas, alphaMin, iKFr);
    }
  }
#ifdef CFG_STEREO
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive, -1, m_rightActive) :
                               GetCameraLF(m_iLFActive, -1, m_rightActive).m_T;
#else
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
#endif
  if (C.Valid() && (activeKF && m_keyDrawMotTypeKF != DRAW_MOT_KF_NONE || !activeKF &&
                    m_keyDrawMotTypeLF != DRAW_MOT_LF_NONE)) {
    m_frustrumActive.SetPose(C, m_keyDrawAxis == DRAW_AXIS_WORLD_AND_CAMERA);
    if (m_keyDrawCamTex && !m_keyDrawDepPlane) {
      glEnable(GL_TEXTURE_RECTANGLE_ARB);
      Update2DImageTexture();
#ifdef CFG_STEREO
      if (m_rightActive) {
        m_frustrumActive.DrawTexture(m_texRGBr);
      } else
#endif
      {
        m_frustrumActive.DrawTexture(m_texRGB);
      }
      glDisable(GL_TEXTURE_RECTANGLE_ARB);
    }
#ifdef CFG_GROUND_TRUTH
    if (activeKF && m_keyDrawCamTypeKF == DRAW_CAM_KF_GT ||
       !activeKF && m_keyDrawCamTypeLF == DRAW_CAM_LF_GT) {
      glColor3ub(255, 255, 0);
    } else
#endif
    {
      //glColor3ub(135, 204, 234);
      glColor3ub(0, 255, 0);
    }
    m_frustrumActive.Draw(m_keyDrawAxis == DRAW_AXIS_WORLD_AND_CAMERA);
  }

//#ifdef CFG_GROUND_TRUTH
#if 0
  if (m_keyDrawCamGT) {
    const int N = m_CTGT.Size();
    glPushAttrib(GL_LINE_BIT);
    glLineWidth(2.0f);
    //glColor3ub(255, 127, 0);
    //glColor3ub(205, 102, 0);
    glColor3ub(255, 255, 0);
    glBegin(GL_LINE_STRIP);
#ifdef CFG_STEREO
    Point3D p;
#endif
    for (int i = 0; i < N; ++i) {
      const Camera &C = m_CTGT.m_Cs[i];
      if (C.Invalid()) {
        continue;
      }
#ifdef CFG_STEREO
      if (right) {
        C.m_T.ApplyRotationInversely(m_K.m_br, p);
        p.xyzw() = _mm_sub_ps(C.m_p.xyzw(), p.xyzw());
        glVertex3fv(p);
      } else
#endif
      {
        glVertex3fv(C.m_p);
      }
    }
    glEnd();
    if (m_keyDrawCamVelocity && N > 1) {
      glLineWidth(1.0f);
      glColor3ub(0, 255, 255);
      glBegin(GL_LINES);
      float dt;
      Point3D p1, p2;
      for (int i1 = 0, i2 = 1; i1 < N; i1 = i2++) {
        const Camera &C = m_CTGT.m_Cs[i1];
        if (C.Invalid() || C.m_v.Invalid()) {
          continue;
        }
        C.m_T.ApplyRotationInversely(m_K.m_pu, p1);
        p1 += C.m_p;
        const float t1 = m_CTGT.m_ts[i1];
        if (i2 == N) {
          dt = t1 - m_CTGT.m_ts[i1 - 1];
        } else {
          dt = m_CTGT.m_ts[i2] - t1;
        }
        C.m_v.GetScaled(dt, p2);
        p2 += p1;
        glVertex3fv(p1);
        glVertex3fv(p2);
      }
      glEnd();
    }
    glPopAttrib();
  }
#endif  // CFG_GROUND_TRUTH
  glPopAttrib();
}

void ViewerIBA::Draw3DCameras(const std::vector<float> &alphas, const float alphaMin,
                              const int iKFr) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColor4f(0.5f, 0.5f, 0.5f, alphaMin);
  const int nKFs = static_cast<int>(alphas.size());
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (alphas[iKF] != 0.0f) {
      continue;
    }
#ifdef CFG_STEREO
    const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
    const Rigid3D C = GetCameraKF(iKF);
#endif
    if (C.Invalid()) {
      continue;
    }
    m_frustrum.SetPose(C, false);
    m_frustrum.Draw(false);
  }
  glColor4f(1.0f, 0.0f, 0.0f, alphaMin);
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (alphas[iKF] >= 0.0f) {
      continue;
    }
#ifdef CFG_STEREO
    const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
    const Rigid3D C = GetCameraKF(iKF);
#endif
    if (C.Invalid()) {
      continue;
    }
    m_frustrum.SetPose(C, false);
    m_frustrum.Draw(false);
  }
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    if (alphas[iKF] <= 0.0f) {
      continue;
    }
#ifdef CFG_STEREO
    const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
    const Rigid3D C = GetCameraKF(iKF);
#endif
    if (C.Invalid()) {
      continue;
    }
    glColor4f(1.0f, 0.0f, 1.0f, alphas[iKF]);
    m_frustrum.SetPose(C, false);
    m_frustrum.Draw(false);
  }
  glDisable(GL_BLEND);
  if (iKFr == -1) {
    return;
  }
#ifdef CFG_STEREO
  const Rigid3D C = GetCameraKF(iKFr, -1, m_rightActive);
#else
  const Rigid3D C = GetCameraKF(iKFr);
#endif
  if (C.Invalid()) {
    return;
  }
  glColor3ub(255, 255, 0);
  m_frustrum.SetPose(C, false);
  m_frustrum.Draw(false);
}

void ViewerIBA::Draw3DPoints() {
  //if (m_keyDrawStrType == DRAW_STR_NONE)
  //  return;
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, m_pWnd->size().x, m_pWnd->size().y, 0);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glScalef(m_factorImgToWin.x(), m_factorImgToWin.y(), 1.0f);
  glTranslatef(m_K.m_K.cx(), m_K.m_K.cy(), 0.0f);
  glScalef(m_K.m_K.fx(), m_K.m_K.fy(), 1.0f);

  Point2D x, xc[2], xps[4];
  LA::Vector2f Jx;
  glPointSize(1.0f);
  glLineWidth(1.0f);
  const int nKFs = GetKeyFrames();
  const bool activeKF = m_iFrmActive < nKFs;
  const FRM::Frame &F = *(activeKF ? GetKeyFrame(m_iKFActive) : GetLocalFrame(m_iLFActive));
  const int NZ = int(F.m_Zs.size());
  //m_marksListTmp.resize(NZ);
  m_iX2z.resize(NZ);
  for (int iZ = 0; iZ < NZ; ++iZ) {
    std::vector<int> &ix2z = m_iX2z[iZ];
    const FRM::Measurement &Z = F.m_Zs[iZ];
    ix2z.assign(((KeyFrame *) GetKeyFrame(Z.m_iKF))->m_xs.size(), -1);
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      ix2z[F.m_zs[iz].m_ix] = iz;
    }
  }
  float d;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D _C = GetCameraKF(iKF);
    if (_C.Invalid()) {
      continue;
    }
    const int iZ = F.SearchFrameMeasurement(iKF);
    const Rigid3D _Tr = m_Cv / _C;
    const LA::Vector3ub *clrs = m_clrsKF.data() + m_iKF2d[iKF];
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(iKF);
    const int Nx = static_cast<int>(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const bool activeFtr = m_iFtrActive.m_ix.Equal(iKF, ix);
      if (m_keyDrawStrType == DRAW_STR_NONE && !activeFtr) {
        continue;
      }
      //const bool activeFrm = iKF == m_iFrmActive || iZ != -1 && m_marksListTmp[iZ][ix];
      const int iz = iZ == -1 ? -1 : m_iX2z[iZ][ix];
      const bool activeFrm = iKF == m_iFrmActive || iz != -1;
      if (m_keyDrawStrType == DRAW_STR_ACTIVE && !activeFrm && !activeFtr) {
        continue;
      }
      const Depth::InverseGaussian _d = GetFeatureDepth(iKF, ix);
      if (_d.Invalid()) {
        continue;
      }
      const bool converge = _d.s2() < m_keyDrawDepVar * m_keyDrawDepVar;
      if ((m_keyDrawStrType == DRAW_STR_CONVERGED && !converge ||
           m_keyDrawStrType == DRAW_STR_NOT_CONVERGED && converge) && !activeFtr) {
        continue;
      }
      if (!_d.Project(_Tr, KF.m_xs[ix].m_x, m_Bn, x, d, Jx)) {
        continue;
      }
      if (d <= 0.0f) {
        glColor3ub(255, 0, 0);
      } else if (activeFtr) {
        glColor3ub(0, 0, 255);
      } else if (activeFrm) {
        glColor3ub(255, 255, 0);
      } else {
        glColor3ubv(clrs[ix]);
      }
      if (m_keyDrawErrType3D == DRAW_ERR_ALL || m_keyDrawErrType3D == DRAW_ERR_COVARIANCE || activeFtr) {
        if (converge) {
          glBegin(GL_POINTS);
          glVertex2fv(x);
          glEnd();
        } else {
          Get2DProjectionRange(x, Jx, _d.s2(), xps);
          glBegin(GL_LINES);
          glVertex2fv(xps[0]);
          glVertex2fv(xps[1]);
          glVertex2fv(xps[2]);
          glVertex2fv(xps[3]);
          glEnd();
        }
#if 0
//#if 1
        const LocalFrame &LF = *(LocalFrame *) GetLocalFrame(m_iLFActive);
        const int iz = LF.SearchFeatureMeasurement(iKF, ix);
        if (iz != -1) {
          Point2DCovariance Sx;
          LA::SymmetricMatrix2x2f::aaT(Jx, Sx);
          Sx *= _d.s2();
          Sx += LF.m_zs[iz].m_SI.GetInverse(0.0f);
          Viewer::DrawCovariance(x, Sx, X2);
        }
#endif
      }
      //glPushAttrib(GL_DEPTH_BUFFER_BIT);
      //glDisable(GL_DEPTH_TEST);
      //glDepthFunc(GL_ALWAYS);
      if (m_keyDrawErrType3D == DRAW_ERR_ALL || m_keyDrawErrType3D == DRAW_ERR_MEAN || activeFtr) {
        xc[0].Invalidate();
        if (m_keyDrawDepTypeCMP != m_keyDrawDepType) {
          const Depth::InverseGaussian dc = GetFeatureDepth(iKF, ix, m_keyDrawDepTypeCMP);
          if (!dc.Invalid()) {
            dc.Project(_Tr, KF.m_xs[ix].m_x, xc[0]);
          }
        }
        xc[1].Invalidate();
        glPushAttrib(GL_CURRENT_BIT);
        for (int i = 0; i < 2; ++i) {
          const Point2D _xc = xc[i];
          if (_xc.Invalid()) {
            continue;
          }
          glColor3ub(255, 0, 0);
          glBegin(GL_LINES);
          glVertex2fv(x);
          glVertex2fv(_xc);
          glEnd();
          SearchActiveFeature(_xc, iKF, ix);
        }
        glPopAttrib();
      }
      glBegin(GL_POINTS);
      glVertex2fv(x);
      glEnd();
      if (activeFtr) {
        glColor3ub(0, 0, 255);
        //Viewer::DrawBox(x.x(), x.y(), 0.05f * _d.GetProjectedD(_Cr, KF.m_xs[ix]));
        Viewer::DrawBox(x.x(), x.y(), m_sizeCross * m_keyDrawPchSize);
      }
      //glPopAttrib();
      SearchActiveFeature(x, iKF, ix);
    }
  }
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void ViewerIBA::Draw3DWorldAxis() {
  if (m_keyDrawAxis == DRAW_AXIS_NONE) {
    return;
  }
  glLineWidth(1.0f);
  m_frustrum.MakeOrigin(true);
  m_frustrum.DrawAxis();
  //m_frustrumActive.MakeOrigin(true);
  //m_frustrumActive.DrawAxis();
#if 0
//#if 1
  glLineWidth(2.0f);
  glColor3ub(0, 255, 0);
  const float x = 0.105f, y = 0.1485f;
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < 4; ++i) {
    switch (i) {
    case 0: glVertex3f( x,  y, 0.0f); break;
    case 1: glVertex3f(-x,  y, 0.0f); break;
    case 2: glVertex3f(-x, -y, 0.0f); break;
    case 3: glVertex3f( x, -y, 0.0f); break;
    }
  }
  glEnd();
#endif
}

void ViewerIBA::Draw3DDepthPlane() {
  if (!m_keyDrawDepPlane) {
    return;
  }
  const bool activeKF = m_iFrmActive < GetKeyFrames();
  const Rigid3D C = activeKF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
  const Depth::InverseGaussian d = activeKF ? GetFrameDepthKF(m_iKFActive)
                                            : GetFrameDepthLF(m_iLFActive);
  const float s = sqrtf(d.s2());
  glColor3ub(128, 128, 128);
  Draw3DDepthPlane(C, std::max(d.u() - s, 0.0f));
  Draw3DDepthPlane(C, d.u() + s);

  Point3D *Xs = NULL;
  if (m_keyDrawCamTex && m_keyDrawViewType == DRAW_VIEW_3D) {
    m_work.Resize(4 * sizeof(Point3D));
    Xs = (Point3D *) m_work.Data();
  }
  glColor3ub(135, 204, 234);
  Draw3DDepthPlane(C, d.u(), Xs);
  if (Xs) {
    glEnable(GL_TEXTURE_RECTANGLE_ARB);
    Update2DImageTexture();
#ifdef CFG_STEREO
    if (m_rightActive) {
      Viewer::DrawTexture(m_texRGBr, Xs);
    } else
#endif
    {
      Viewer::DrawTexture(m_texRGB, Xs);
    }
    glDisable(GL_TEXTURE_RECTANGLE_ARB);
  }
}

void ViewerIBA::Draw3DDepthPlane(const Rigid3D &C, const float d, Point3D *Xs) {
  Point2D x;
  Point3D X;
  const float z = 1.0f / d;
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < 4; ++i) {
    switch (i) {
    case 0: x.Set(-m_K.m_K.fxIcx(), -m_K.m_K.fyIcy());  break;
    case 1: x.Set(-m_K.m_K.fxIcx(),  m_K.m_K.fyIcy());  break;
    case 2: x.Set( m_K.m_K.fxIcx(),  m_K.m_K.fyIcy());  break;
    case 3: x.Set( m_K.m_K.fxIcx(), -m_K.m_K.fyIcy());  break;
    }
    X.Set(x, z);
    C.ApplyInversely(X);
    glVertex3fv(X);
    if (Xs) {
      Xs[i] = X;
    }
  }
  glEnd();
}

void ViewerIBA::Draw3DObjects() {
}

void ViewerIBA::Reset3DProjectionMatrix() {
  const double aspect = m_K.m_K.w() / double(m_K.m_K.h());
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef((m_K.m_K.cx() * 2.0f) / m_K.m_K.w() - 1.0f,
               1.0f - (m_K.m_K.cy() * 2.0f) / m_K.m_K.h(), 0.0f);
  gluPerspective(double(m_K.m_K.GetFovY()), aspect, VW_WINDOW_DEPTH, DBL_MAX);
  glGetDoublev(GL_PROJECTION_MATRIX, m_projMatrix);
  glPopMatrix();
}

void ViewerIBA::Reset3DModelViewMatrix() {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  gluLookAt(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
  glGetDoublev(GL_MODELVIEW_MATRIX, m_modelMatrix);
  //glScalef(VW_SCENE_SCALE, VW_SCENE_SCALE, VW_SCENE_SCALE);
  glGetDoublev(GL_MODELVIEW_MATRIX, m_modelMatrixScale);
  glPopMatrix();
}

void ViewerIBA::Reset3DViewPoint(const bool active) {
  if (active) {
    const bool activeKF = m_iFrmActive < GetKeyFrames();
#ifdef CFG_STEREO
    m_Cv = activeKF ? GetCameraKF(m_iKFActive, -1, m_rightActive) :
                      GetCameraLF(m_iLFActive, -1, m_rightActive).m_T;
#else
    m_Cv = activeKF ? GetCameraKF(m_iKFActive) : GetCameraLF(m_iLFActive).m_T;
#endif
  }
  const float z = ComputeMeanDepth(m_Cv);
  const float xzRatio = m_K.m_K.w() * 0.5f * m_K.m_K.fxI();
  const float yzRatio = m_K.m_K.h() * 0.5f * m_K.m_K.fyI();
  m_frustrum.SetShape(z * VW_CAMERA_DEPTH_TO_SCENE_DEPTH_RATIO, xzRatio, yzRatio, m_keyDrawCamSize,
                      m_keyDrawAxisLen);
  m_frustrumActive.SetShape(z * VW_CAMERA_DEPTH_TO_SCENE_DEPTH_RATIO, xzRatio, yzRatio,
                            m_keyDrawCamSize * VW_CAMERA_SIZE_ACTIVE_RATIO, m_keyDrawAxisLen);
  //m_translation.MakeZero();
  m_arcball.Initialize(m_Cv.LA::AlignedMatrix3x3f::GetTranspose(), z,
                       UnProjectWindowSize(z) * VW_ARCBALL_RADIUS_TO_WINDOW_SIZE_RATIO);
  m_arcball.GetRotationTranspose().Apply(m_Cv.GetTranslation() - m_arcball.GetCenter(),
                                         m_translation);
  m_translation += m_arcball.GetCenter();
  m_factorWinToTranslationZ = z / (m_pWnd ? m_pWnd->size().y : m_K.m_K.h()) *
                              VW_SCROLL_TRANSLATTON_Z_RATE;
}

void ViewerIBA::Update3DViewPoint() {
  m_Cv = m_arcball.GetRotation();
  m_Cv.SetTranslation(m_arcball.GetRotation().GetApplied(m_translation - m_arcball.GetCenter()) +
                      m_arcball.GetCenter());
}

void ViewerIBA::Update3DBackgroundColor() {
  switch (m_keyDrawBgClr) {
  case DRAW_BG_BLACK: glClearColor(0.0f, 0.0f, 0.0f, 0.0f); break;
  case DRAW_BG_WHITE: glClearColor(1.0f, 1.0f, 1.0f, 1.0f); break;
  }
}

float ViewerIBA::ComputeMeanDepth(const Rigid3D &Cv) {
  bool pt = false;
  Rigid3D::Row Trz;
  Depth::InverseGaussian d, di;
  d.Initialize(DEPTH_INITIAL);
  const Rigid3D::Row CvZ = Cv.GetRowZ();
  const int nKFs = GetKeyFrames();
  for (int iKF = 0; iKF < nKFs; ++iKF) {
    const Rigid3D C = GetCameraKF(iKF);
    Rigid3D::ABI(CvZ, C, Trz);
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(iKF);
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const Depth::InverseGaussian _d = GetFeatureDepth(iKF, ix);
      if (_d.Invalid() || !_d.ProjectD(Trz, KF.m_xs[ix].m_x, di)) {
        continue;
      }
      d.Update(di);
      pt = true;
#if 0
      UT::Print("%d %d %f %f %f %f %f %f\n", iKF, ix, _d.u(), _d.s2(), di.u(), di.s2(), d.u(), d.s2());
#endif
    }
  }
  if (pt) {
    return 1.0f / d.u();
  }
  float Sd = 0.0f;
  int SN = 0;
  for (int iKF = 0; iKF < nKFs; ++iKF) {
#ifdef CFG_STEREO
    const Rigid3D C = GetCameraKF(iKF, -1, m_rightActive);
#else
    const Rigid3D C = GetCameraKF(iKF);
#endif
    di.u() = 1.0f / Cv.GetAppliedZ(C.GetPosition());
    if (!di.Valid()) {
      continue;
    }
    Sd = di.u() + Sd;
    ++SN;
  }
  const int nLFs = GetLocalFrames();
  for (int iLF = 0; iLF < nLFs; ++iLF) {
#ifdef CFG_STEREO
    const Camera C = GetCameraLF(iLF, -1, m_rightActive);
#else
    const Camera C = GetCameraLF(iLF);
#endif
    di.u() = 1.0f / Cv.GetAppliedZ(C.m_p);
    if (!di.Valid()) {
      continue;
    }
    Sd = di.u() + Sd;
    ++SN;
  }
  if (SN > 0) {
    return SN / Sd;
  } else {
    return 1.0f / DEPTH_INITIAL;
  }
}

void ViewerIBA::UnProjectWindowPoint(const CVD::ImageRef &x, Point3D &X) {
  double Xx, Xy, Xz;
  gluUnProject(x.x, m_pWnd->size().y - x.y, 0, m_modelMatrix, m_projMatrix, m_viewport,
               &Xx, &Xy, &Xz);
  X.Set(float(Xx), float(Xy), float(Xz));
#ifdef CFG_DEBUG
  //UT_ASSERT(Xz == VW_WINDOW_DEPTH);
  UT::AssertEqual(Xz, VW_WINDOW_DEPTH);
#endif
}

void ViewerIBA::UnProjectWindowPoint(const CVD::ImageRef &x, Point3D &X, const float z) {
  double Xx, Xy, Xz;
  gluUnProject(x.x, m_pWnd->size().y - x.y, 0, m_modelMatrix, m_projMatrix, m_viewport,
               &Xx, &Xy, &Xz);
#ifdef CFG_DEBUG
  //UT_ASSERT(Xz == VW_WINDOW_DEPTH);
  UT::AssertEqual(Xz, VW_WINDOW_DEPTH);
#endif
  Xz = z / Xz;
  X.Set(float(Xx * Xz), float(Xy * Xz), z);
}

float ViewerIBA::UnProjectWindowSize(const float z) {
  //const float xWinSize = m_K.m_K.w() * m_K.m_K.fxI();
  //const float yWinSize = m_K.m_K.h() * m_K.m_K.fyI();
  //if (xWinSize < yWinSize) {
  //  return z * xWinSize;
  //} else {
  //  return z * yWinSize;
  //}
  return std::min(m_K.m_K.fxIcx(), m_K.m_K.fyIcy()) * z;
}

void ViewerIBA::DrawProfile() {
  const int W = m_pWnd->size().x, H = m_pWnd->size().y;
  const int dx = VW_STRING_BORDER_X;
  const float hi = H * 0.2f, hhi = hi * 0.5f, whi = m_K.m_K.w() * hhi / m_K.m_K.h();
  const Point2D xi(dx + whi, H - VW_STRING_BORDER_Y - VW_STRING_HEIGHT - VW_STRING_GAP_Y - hhi);
  glEnable(GL_TEXTURE_RECTANGLE_ARB);
  Update2DImageTexture();
  const LA::Vector2f size(whi, hhi);
#ifdef CFG_STEREO
  if (m_rightActive) {
    Viewer::DrawTexture(m_texRGBr, xi, size);
  } else
#endif
  {
    Viewer::DrawTexture(m_texRGB, xi, size);
  }
  glDisable(GL_TEXTURE_RECTANGLE_ARB);

  //const float y1 = yi + hhi + VW_STRING_GAP_Y + Viewer::GetStringHeight(GLUT_BITMAP_HELVETICA_10) + VW_STRING_GAP_Y;
  //const float y2 = H - VW_STRING_BORDER_Y - VW_STRING_HEIGHT - VW_STRING_GAP_Y;
  const float y1 = Viewer::GetStringYTopLeft(2, GLUT_BITMAP_HELVETICA_10) + VW_STRING_GAP_Y;
  const float y2 = xi.y() - hhi - VW_STRING_GAP_Y;
  const float B = (y2 - y1) * 0.5f, b = B * 0.1f, yb = y1 + B;
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames(), nFrms = nKFs + nLFs;
  const float ratio = float(W - dx - dx) / (nFrms - 1);
  const bool activeKF = m_iFrmActive < nKFs;

  glColor3ub(255, 255, 255);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  glVertex2i(0, yb);
  glVertex2i(W, yb);
  const float x1 = dx + (nKFs - 0.5f) * ratio;
  float x = x1;
  //glColor3ub(0, 0, 255);
  glColor3ub(0, 255, 255);
  glVertex2f(x, yb - b);
  glVertex2f(x, yb + b);
  if (m_dragingActiveFrm) {
    glColor3ub(0, 0, 255);
  } else {
    glColor3ub(255, 255, 255);
  }
  glEnd();
  if (m_keyDrawPrfType >= DRAW_PRF_ACCELERATION &&
      m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) {
    x = dx + (m_iFrmActive - 1) * ratio;
    glBegin(GL_LINES);
    glVertex2f(x, yb - b);
    glVertex2f(x, yb + b);
    glEnd();
  } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) {
    if (activeKF) {
      const int NZ = static_cast<int>(m_GBA->m_Zps.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const CameraPrior::Pose &Zp = m_GBA->m_Zps[iZ];
        if (Zp.m_iKFr == m_iKFActive) {
          DrawProfile(Zp, dx, ratio, yb, b);
        }
      }
    } else if (m_LBA->m_Zp.Pose::Valid()) {
      DrawProfile(m_LBA->m_Zp, dx, ratio, yb, b);
    }
  }
  x = dx + m_iFrmActive * ratio;
  glBegin(GL_LINES);
  glVertex2f(x, yb - B);
  glVertex2f(x, yb + B);
  glEnd();
  //if (!DrawProfileTypeValid())
  //  return;
  const int type = GetDrawProfileScaleType();
  const float vMax = m_keyDrawPrfScale[type];
  const bool r = (m_keyDrawPrfType >= DRAW_PRF_GYROSCOPE &&
                  m_keyDrawPrfType <= DRAW_PRF_GYROSCOPE_DEVICE) ||
                 (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_ROTATION_STATE &&
                  m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE) ||
                 (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE &&
                  m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) ||
                 (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
                  m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE) ||
                 (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE &&
                  m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) ||
                 (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_ABSOLUTE &&
                  m_keyDrawPrfType < DRAW_PRF_STATE_POSITION_ABSOLUTE) ||
                  m_keyDrawPrfType >= DRAW_PRF_STATE_BIAS_GYROSCOPE;
  std::string str;
  switch (m_keyDrawPrfType) {
  case DRAW_PRF_ACCELERATION:         str = "a";    break;
  case DRAW_PRF_ACCELERATION_DEVICE:  str = "a'";   break;
  case DRAW_PRF_GYROSCOPE:            str = "w";    break;
  case DRAW_PRF_GYROSCOPE_DEVICE:     str = "w'";   break;
  case DRAW_PRF_IMU_DELTA_ROTATION_STATE:               str = "xdr";  break;
  case DRAW_PRF_IMU_DELTA_ROTATION_MEASUREMENT:         str = "zdr";  break;
  case DRAW_PRF_IMU_DELTA_ROTATION_ERROR:               str = "edr";  break;
  case DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE:          str = "Sdr";  break;
  case DRAW_PRF_IMU_DELTA_POSITION_STATE:               str = "xdp";  break;
  case DRAW_PRF_IMU_DELTA_POSITION_MEASUREMENT:         str = "zdp";  break;
  case DRAW_PRF_IMU_DELTA_POSITION_ERROR:               str = "edp";  break;
  case DRAW_PRF_IMU_DELTA_POSITION_COVARIANCE:          str = "Sdp";  break;
  case DRAW_PRF_IMU_DELTA_VELOCITY_STATE:               str = "xdv";  break;
  case DRAW_PRF_IMU_DELTA_VELOCITY_MEASUREMENT:         str = "zdv";  break;
  case DRAW_PRF_IMU_DELTA_VELOCITY_ERROR:               str = "edv";  break;
  case DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE:          str = "Sdv";  break;
  case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE:      str = "xdba"; break;
  case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_COVARIANCE: str = "Sdba"; break;
  case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE:         str = "xdbw"; break;
  case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE:    str = "Sdbw"; break;
  case DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE:                str = "xpr";  break;
  case DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT:          str = "zpr";  break;
  case DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR:                str = "epr";  break;
  case DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE:           str = "Spr";  break;
  case DRAW_PRF_CAMERA_PRIOR_POSITION_STATE:                str = "xpp";  break;
  case DRAW_PRF_CAMERA_PRIOR_POSITION_MEASUREMENT:          str = "zpp";  break;
  case DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR:                str = "epp";  break;
  case DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE:           str = "Spp";  break;
  case DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE:                str = "xpv";  break;
  case DRAW_PRF_CAMERA_PRIOR_VELOCITY_MEASUREMENT:          str = "zpv";  break;
  case DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR:                str = "epv";  break;
  case DRAW_PRF_CAMERA_PRIOR_VELOCITY_COVARIANCE:           str = "Spv";  break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE:       str = "xpba"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_MEASUREMENT: str = "zpba"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_ERROR:       str = "epba"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE:  str = "Spba"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE:          str = "xpbw"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_MEASUREMENT:    str = "zpbw"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_ERROR:          str = "epbw"; break;
  case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE:     str = "Spbw"; break;
  case DRAW_PRF_REPROJECTION_ERROR:       str = "reprojection error"; break;
  case DRAW_PRF_STATE_ROTATION_ABSOLUTE:
  case DRAW_PRF_STATE_ROTATION_RELATIVE:  str = "r";  break;
  case DRAW_PRF_STATE_POSITION_ABSOLUTE:
  case DRAW_PRF_STATE_POSITION_RELATIVE:  str = "p";  break;
  case DRAW_PRF_STATE_VELOCITY:           str = "v";  break;
  case DRAW_PRF_STATE_BIAS_ACCELERATION:  str = "ba"; break;
  case DRAW_PRF_STATE_BIAS_GYROSCOPE:     str = "bw"; break;
#ifdef CFG_GROUND_TRUTH
  case DRAW_PRF_STATE_ROTATION_ABSOLUTE_ERROR:
  case DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR:  str = "r - r'";   break;
  case DRAW_PRF_STATE_POSITION_ABSOLUTE_ERROR:
  case DRAW_PRF_STATE_POSITION_RELATIVE_ERROR:  str = "p - p'";   break;
  case DRAW_PRF_STATE_VELOCITY_ERROR:           str = "v - v'";   break;
  case DRAW_PRF_STATE_BIAS_ACCELERATION_ERROR:  str = "ba - ba'"; break;
  case DRAW_PRF_STATE_BIAS_GYROSCOPE_ERROR:     str = "bw - bw'"; break;
#endif
  }
  str = UT::String("%s = %f", str.c_str(), vMax);
  const int ws = Viewer::GetStringWidth(GLUT_BITMAP_HELVETICA_10, str.c_str());
  x -= ws * 0.5f;
  if (x < dx) {
    x = dx;
  } else if (x + ws + dx >= W) {
    x = W - ws - dx;
  }
  Viewer::DrawStringAt(x, yb - B - Viewer::GetStringHeight(GLUT_BITMAP_HELVETICA_10),
                       GLUT_BITMAP_HELVETICA_10, "%s", str.c_str());
  glLineWidth(1.0f);
  m_work.Resize(0);
  LA::AlignedVector3f v;
  if (m_keyDrawPrfType >= DRAW_PRF_ACCELERATION &&
      m_keyDrawPrfType <= DRAW_PRF_GYROSCOPE_DEVICE) {
    const bool a = m_keyDrawPrfType <= DRAW_PRF_ACCELERATION_DEVICE;
    const bool d = m_keyDrawPrfType == DRAW_PRF_ACCELERATION_DEVICE ||
                   m_keyDrawPrfType == DRAW_PRF_GYROSCOPE_DEVICE;
    x = dx + ratio;
    for (int iFrm1 = 0, iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2++, x = ratio + x) {
      if (iFrm1 == nKFs) {
        continue;
      }
      const int iKF1 = iFrm1 < nKFs ? iFrm1 : -1;
      const int iLF1 = iKF1 == -1 ? (iFrm1 - nKFs + m_iLF + 1) % nLFs : -1;
      const int iKF2 = iFrm2 < nKFs ? iFrm2 : -1;
      const int iLF2 = iKF2 == -1 ? (iFrm2 - nKFs + m_iLF + 1) % nLFs : -1;
      const Camera C1 = iKF1 != -1 ? GetCameraGBALM(iKF1) : GetCameraLF(iLF1);
      if (d && C1.Invalid()) {
        continue;
      }
      const Rotation3D RT = C1.m_T.Rotation3D::GetTranspose();
      const LA::AlignedVector3f b = a ? C1.m_ba : C1.m_bw;
      const float t1 = (iKF1 != -1 ? GetKeyFrame(iKF1) : GetLocalFrame(iLF1))->m_T.m_t;
      const GlobalBundleAdjustor::KeyFrame *KF2 = iKF2 != -1 ? &m_GBA->m_KFs[iKF2] : NULL;
      const LocalFrame *LF2 = !KF2 ? (LocalFrame *) GetLocalFrame(iLF2) : NULL;
      const float _ratio = ratio / ((KF2 ? KF2->m_T.m_t : LF2->m_T.m_t) - t1);
      const AlignedVector<IMU::Measurement> &us = KF2 ? KF2->m_us : LF2->m_us;
      const int Nu = us.Size();
      for (int i = 0; i < Nu; ++i) {
        const IMU::Measurement &u = us[i];
        v = a ? u.m_a : u.m_w;
        if (d) {
          v -= b;
          if (a) {
            v = RT.GetApplied(v);
            if (!IMU_GRAVITY_EXCLUDED) {
              v.z() -= IMU_GRAVITY_MAGNITUDE;
            }
          }
        }
        m_work.Push(x - ratio + (u.t() - t1) * _ratio);
        m_work.Push(v, 3);
      }
    }
  } else if (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) {
    LA::AlignedMatrix3x3f S;
    const bool c = m_keyDrawPrfType < DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE ?
                  (m_keyDrawPrfType - DRAW_PRF_IMU_DELTA_ROTATION_STATE) % 4 == 3 :
                   m_keyDrawPrfType == DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE ||
                   m_keyDrawPrfType == DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE;
    x = dx + ratio;
    for (int iFrm1 = 0, iFrm2 = 1; iFrm2 < nFrms; iFrm1 = iFrm2++, x = ratio + x) {
      if (iFrm2 == nKFs) {
        continue;
      }
      const int iKF1 = iFrm1 < nKFs ? iFrm1 : -1;
      const int iLF1 = iKF1 == -1 ? (iFrm1 - nKFs + m_iLF + 1) % nLFs : -1;
      const int iKF2 = iFrm2 < nKFs ? iFrm2 : -1;
      const int iLF2 = iKF2 == -1 ? (iFrm2 - nKFs + m_iLF + 1) % nLFs : -1;
      const Camera C1 = iKF1 != -1 ? GetCameraGBALM(iKF1) : GetCameraLF(iLF1);
      const Camera C2 = iKF2 != -1 ? GetCameraGBALM(iKF2) : GetCameraLF(iLF2);
      const IMU::Delta D = iKF2 != -1 ? GetIMUDeltaGBALM(iKF2) : GetIMUDeltaLF(iLF2);
      if (C1.Invalid() || C2.Invalid() || D.Invalid()) {
        continue;
        //v.Invalidate();
        //v.x() = FLT_MAX;
        //v.y() = FLT_MAX;
        //v.z() = FLT_MAX;
      } else {
        switch (m_keyDrawPrfType) {
        case DRAW_PRF_IMU_DELTA_ROTATION_STATE:
          v = (D.GetRotationState(C1, C2)).GetRodrigues(BA_ANGLE_EPSILON);
          break;
        case DRAW_PRF_IMU_DELTA_ROTATION_MEASUREMENT:
          v = (D.GetRotationMeasurement(C1, BA_ANGLE_EPSILON)).GetRodrigues(BA_ANGLE_EPSILON);
          break;
        case DRAW_PRF_IMU_DELTA_ROTATION_ERROR:
          v = D.GetRotationError(C1, C2, BA_ANGLE_EPSILON);
          break;
        case DRAW_PRF_IMU_DELTA_ROTATION_COVARIANCE:
#ifdef CFG_IMU_FULL_COVARIANCE
          S = D.m_W[0][0].GetInverseLDL();
#else
          S = D.m_W.m_Wr.GetInverseLDL();
#endif
          break;
        case DRAW_PRF_IMU_DELTA_POSITION_STATE:
          v = D.GetPositionState(C1, C2, m_K.m_pu);
          break;
        case DRAW_PRF_IMU_DELTA_POSITION_MEASUREMENT:
          v = D.GetPositionMeasurement(C1);
          break;
        case DRAW_PRF_IMU_DELTA_POSITION_ERROR:
          v = D.GetPositionError(C1, C2, m_K.m_pu);
          break;
        case DRAW_PRF_IMU_DELTA_POSITION_COVARIANCE:
#ifdef CFG_IMU_FULL_COVARIANCE
          S = D.m_W[2][2].GetInverseLDL();
#else
          S = D.m_W.m_Wp.GetInverseLDL();
#endif
          break;
        case DRAW_PRF_IMU_DELTA_VELOCITY_STATE:
          v = D.GetVelocityState(C1, C2);
          break;
        case DRAW_PRF_IMU_DELTA_VELOCITY_MEASUREMENT:
          v = D.GetVelocityMeasurement(C1);
          break;
        case DRAW_PRF_IMU_DELTA_VELOCITY_ERROR:
          v = D.GetVelocityError(C1, C2);
          break;
        case DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE:
#ifdef CFG_IMU_FULL_COVARIANCE
          S = D.m_W[1][1].GetInverseLDL();
#else
          S = D.m_W.m_Wv.GetInverseLDL();
#endif
          break;
        case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE:
          v = C1.m_ba - C2.m_ba;
          break;
        case DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_COVARIANCE:
#ifdef CFG_IMU_FULL_COVARIANCE
          S = D.m_W[3][3].GetInverseLDL();
#else
          S.MakeDiagonal(D.m_W.m_wba);
#endif
          break;
        case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_STATE:
          v = C1.m_bw - C2.m_bw;
          break;
        case DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE:
#ifdef CFG_IMU_FULL_COVARIANCE
          S = D.m_W[4][4].GetInverseLDL();
#else
          S.MakeDiagonal(D.m_W.m_wbw);
#endif
          break;
        }
        if (c) {
          if (S.Valid()) {
            v = S.GetDiagonal().GetSquareRoot();
          } else {
            v.Invalidate();
          }
        }
      }
      m_work.Push(x);
      m_work.Push(v, 3);
    }
  } else if (m_keyDrawPrfType == DRAW_PRF_REPROJECTION_ERROR) {
    x = dx;
    for (int iFrm = 0; iFrm < nFrms; ++iFrm, x = ratio + x) {
      const int iKF = iFrm < nKFs ? iFrm : -1;
      const int iLF = iKF == -1 ? (iFrm - nKFs + m_iLF + 1) % nLFs : -1;
      const UT::ES<FTR::ESError, FTR::ESIndex> ES = ComputeReprojectionError(iKF, iLF);
      m_work.Push(x);
      m_work.Push(ES.Mean());
      m_work.Push(ES.Maximal());
    }
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_ABSOLUTE) {
    Rotation3D R;
    x = dx;
    for (int iFrm = 0; iFrm < nFrms; ++iFrm, x = ratio + x) {
      const int iKF = iFrm < nKFs ? iFrm : -1;
      const int iLF = iKF == -1 ? (iFrm - nKFs + m_iLF + 1) % nLFs : -1;
      const int iKFNearest = (iKF != -1 ? GetKeyFrame(iKF) : GetLocalFrame(iLF))->m_iKFNearest;
      if (m_keyDrawPrfType < DRAW_PRF_STATE_POSITION_ABSOLUTE) {
        if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_RELATIVE && iKFNearest == -1) {
          v.Invalidate();
        } else {
          R = iKF != -1 ? GetCameraKF(iKF) : GetCameraLF(iLF).m_T;
          if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_RELATIVE) {
            R = R / GetCameraKF(iKFNearest);
          }
#ifdef CFG_GROUND_TRUTH
          if (m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR) {
            R = R * GetCameraKF(iKFNearest, DRAW_CAM_KF_GT);
          }
          if (m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_RELATIVE_ERROR ||
              m_keyDrawPrfType == DRAW_PRF_STATE_ROTATION_ABSOLUTE_ERROR) {
            R = R / (iKF != -1 ? GetCameraKF(iKF, DRAW_CAM_KF_GT) :
            GetCameraLF(iLF, DRAW_CAM_LF_GT).m_T);
          }
#endif
          R.GetRodrigues(v, BA_ANGLE_EPSILON);
        }
      } else if (m_keyDrawPrfType < DRAW_PRF_STATE_VELOCITY) {
        if (m_keyDrawPrfType >= DRAW_PRF_STATE_POSITION_RELATIVE && iKFNearest == -1) {
          v.Invalidate();
        } else {
          v = iKF != -1 ? GetCameraKF(iKF).GetPosition() : GetCameraLF(iLF).m_p;
          if (m_keyDrawPrfType >= DRAW_PRF_STATE_POSITION_RELATIVE) {
            v -= GetCameraKF(iKFNearest).GetPosition();
          }
#ifdef CFG_GROUND_TRUTH
          if (m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_RELATIVE_ERROR) {
            v += GetCameraKF(iKFNearest, DRAW_CAM_KF_GT).GetPosition();
          }
          if (m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_RELATIVE_ERROR ||
              m_keyDrawPrfType == DRAW_PRF_STATE_POSITION_ABSOLUTE_ERROR) {
            v -= iKF != -1 ? GetCameraKF(iKF, DRAW_CAM_KF_GT).GetPosition() :
            GetCameraLF(iLF, DRAW_CAM_LF_GT).m_p;
          }
#endif
        }
      } else {
        const Camera C = iKF != -1 ? GetCameraGBALM(iKF) : GetCameraLF(iLF);
        if (C.Invalid()) {
          continue;
        }
        if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_ACCELERATION) {
          v = C.m_v;
        } else if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_GYROSCOPE) {
          v = C.m_ba;
        } else {
          v = C.m_bw;
        }
#ifdef CFG_GROUND_TRUTH
        if (((m_keyDrawPrfType - DRAW_PRF_STATE_BIAS_ACCELERATION) & 1)) {
          const Camera CGT = iKF != -1 ? GetCameraGBALM(iKF, DRAW_CAM_KF_GT) :
            GetCameraLF(iLF, DRAW_CAM_LF_GT);
          if (CGT.Invalid()) {
            continue;
          }
          if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_ACCELERATION) {
            v -= CGT.m_v;
          } else if (m_keyDrawPrfType < DRAW_PRF_STATE_BIAS_GYROSCOPE) {
            v -= CGT.m_ba;
          } else {
            v -= CGT.m_bw;
          }
        }
#endif
      }
      m_work.Push(x);
      m_work.Push(v, 3);
    }
  }
  float sv = -B / vMax;
  if (r) {
    sv /= UT_FACTOR_DEG_TO_RAD;
  }
  const int n = m_keyDrawPrfType == DRAW_PRF_REPROJECTION_ERROR ? 3 : 4, N = m_work.Size() / n;
  int N1 = 0;
  for (int i = 0; N1 < N && m_work[i] < x1; i += n, ++N1) {}
  const int N2 = N - N1;
  const LA::Vector2f _size(10.0f, 10.0f);
  for (int i = 1; i < n; ++i) {
    switch (i) {
    case 1: glColor3ub(255, 0, 0);  break;
    case 2: glColor3ub(0, 255, 0);  break;
    case 3: glColor3ub(0, 0, 255);  break;
    }
    const float *v = m_work.Data();
    if (N1 == 1) {
      if (v[i] != FLT_MAX) {
        Viewer::DrawCross(Point2D(v[0], v[i] * sv + yb), _size);
      }
      v += n;
    } else {
      glBegin(GL_LINE_STRIP);
      for (int j = 0; j < N1; ++j, v += n) {
        if (v[i] == FLT_MAX) {
          glVertex2f(v[0], B + yb);
        } else {
          glVertex2f(v[0], v[i] * sv + yb);
        }
      }
      glEnd();
    }
    if (N2 == 1) {
      if (v[i] != FLT_MAX) {
        Viewer::DrawCross(Point2D(v[0], v[i] * sv + yb), _size);
      }
      v += n;
    } else {
      glBegin(GL_LINE_STRIP);
      for (int j = 0; j < N2; ++j, v += n) {
        if (v[i] == FLT_MAX) {
          glVertex2f(v[0], B + yb);
        } else {
          glVertex2f(v[0], v[i] * sv + yb);
        }
      }
      glEnd();
    }
  }
  const float eps = 0.0f;
  const float epsr = UT::Inverse(BA_VARIANCE_MAX_ROTATION, BA_WEIGHT_FEATURE, eps);
  const float epsp = UT::Inverse(BA_VARIANCE_MAX_POSITION, BA_WEIGHT_FEATURE, eps);
  const float epsv = UT::Inverse(BA_VARIANCE_MAX_VELOCITY, BA_WEIGHT_FEATURE, eps);
  const float epsba = UT::Inverse(BA_VARIANCE_MAX_BIAS_ACCELERATION, BA_WEIGHT_FEATURE, eps);
  const float epsbw = UT::Inverse(BA_VARIANCE_MAX_BIAS_GYROSCOPE, BA_WEIGHT_FEATURE, eps);
  const float _eps[] = {epsp, epsp, epsp, epsr, epsr, epsr, epsv, epsv, epsv,
                        epsba, epsba, epsba, epsbw, epsbw, epsbw};
  if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
      m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE) {
    LA::AlignedVectorXf x;
    LA::AlignedMatrixXf S;
    const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE) % 4;
    if (activeKF) {
      const int NZ = static_cast<int>(m_GBA->m_Zps.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        const CameraPrior::Pose &Zp = m_GBA->m_Zps[iZ];
        if (Zp.m_iKFr == m_iKFActive &&
           (t == 0 || Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &x, NULL,
                                             &m_work, _eps))) {
          DrawProfile(Zp, dx, ratio, yb, sv, _size, x, S);
        }
      }
    } else if (m_LBA->m_Zp.Pose::Valid()) {
      if (t == 0 || m_LBA->m_Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &x, NULL,
                                                    &m_work, _eps)) {
        DrawProfile(m_LBA->m_Zp, dx, ratio, yb, sv, _size, x, S);
      }
    }
  } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) {
    LA::AlignedVectorXf _x;
    LA::AlignedMatrixXf S;
    LA::AlignedVector3f v;
    const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE) % 4;
    if (t == 0 || m_LBA->m_Zp.GetPriorMeasurement(BA_WEIGHT_FEATURE, &S, &_x, NULL,
                                                  &m_work, _eps)) {
      const Camera C = GetCameraLF(m_LBA->m_ic2LF.front());
      const int ip = _x.Size() - 9;
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE:
        v = m_LBA->m_Zp.GetVelocityState(C);
        break;
      case DRAW_PRF_CAMERA_PRIOR_VELOCITY_MEASUREMENT:
        v = m_LBA->m_Zp.GetVelocityMeasurement(_x.Data() + ip);
        break;
      case DRAW_PRF_CAMERA_PRIOR_VELOCITY_ERROR:
        v = m_LBA->m_Zp.GetVelocityError(C, _x.Data() + ip);
        break;
      case DRAW_PRF_CAMERA_PRIOR_VELOCITY_COVARIANCE:
        S.GetDiagonal(ip, v);
        v.MakeSquareRoot();
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_STATE:
        v = m_LBA->m_Zp.GetBiasAccelerationState(C);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_MEASUREMENT:
        v = m_LBA->m_Zp.GetBiasAccelerationMeasurement(_x.Data() + ip + 3);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_ERROR:
        v = m_LBA->m_Zp.GetBiasAccelerationError(C, _x.Data() + ip + 3);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_ACCELERATION_COVARIANCE:
        S.GetDiagonal(ip + 3, v);
        v.MakeSquareRoot();
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_STATE:
        v = m_LBA->m_Zp.GetBiasGyroscopeState(C);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_MEASUREMENT:
        v = m_LBA->m_Zp.GetBiasGyroscopeMeasurement(_x.Data() + ip + 6);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_ERROR:
        v = m_LBA->m_Zp.GetBiasGyroscopeError(C, _x.Data() + ip + 6);
        break;
      case DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE:
        S.GetDiagonal(ip + 6, v);
        v.MakeSquareRoot();
        break;
      }
      x = dx + nKFs * ratio;
      glColor3ub(255, 0, 0);  Viewer::DrawCross(Point2D(x, v.x() * sv + yb), _size);
      glColor3ub(0, 255, 0);  Viewer::DrawCross(Point2D(x, v.y() * sv + yb), _size);
      glColor3ub(0, 0, 255);  Viewer::DrawCross(Point2D(x, v.z() * sv + yb), _size);
    }
  }
}

void ViewerIBA::DrawProfile(const CameraPrior::Pose &Zp, const float dx, const float ratio,
                            const float yb, const float b) {
#ifdef CFG_DEBUG
  UT_ASSERT(Zp.Valid());
#endif
  glBegin(GL_LINES);
  float x;
  const float B = b + b;
  x = dx + Zp.m_iKFr * ratio;
  glColor3ub(255, 255, 0);
  glVertex2f(x, yb - B);
  glVertex2f(x, yb + B);

  glColor3ub(255, 255, 255);
  const int Nk = static_cast<int>(Zp.m_iKFs.size());
  for (int i = 0; i < Nk; ++i) {
    const int iKF = Zp.m_iKFs[i];
    if (iKF == INT_MAX) {
      x = dx + GetKeyFrames() * ratio;
      glVertex2f(x, yb - B);
      glVertex2f(x, yb + B);
    } else {
      x = dx + iKF * ratio;
      glVertex2f(x, yb - b);
      glVertex2f(x, yb + b);
    }
  }
  glEnd();
}

void ViewerIBA::DrawProfile(const CameraPrior::Pose &Zp, const float dx, const float ratio,
                            const float yb, const float sv, const LA::Vector2f &size,
                            const LA::AlignedVectorXf &x, const LA::AlignedMatrixXf &S) {
#ifdef CFG_DEBUG
  UT_ASSERT(Zp.Valid());
  UT_ASSERT(m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
            m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE);
#endif
  int iFrm;
  LA::AlignedVector3f v;
  const int t = (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE) % 4;
  const Rigid3D C1 = GetCameraKF(Zp.m_iKFr);
  const int Nk = static_cast<int>(Zp.m_iKFs.size());
  for (int i = -1, ip = 0; i < Nk; ip = ip + (i == -1 ? 2 : 6), ++i) {
    if (i == -1) {
      if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_POSITION_STATE) {
        continue;
      }
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE:
        v = Zp.GetReferenceRotationState(C1).GetRodrigues(BA_ANGLE_EPSILON);
        break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT:
        v = Zp.GetReferenceRotationMeasurement(x.Data(), BA_ANGLE_EPSILON).GetRodrigues(BA_ANGLE_EPSILON);
        break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR:
        v = Zp.GetReferenceRotationError(C1, x.Data() + ip, BA_ANGLE_EPSILON);
        break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE: {
        LA::Vector2f vr;
        S.GetDiagonal(ip, vr);
        vr.MakeSquareRoot();
        v.Set(vr.x(), vr.y(), 0.0f);
        break; }
      }
      iFrm = Zp.m_iKFr;
    } else {
      const int iKF2 = Zp.m_iKFs[i];
      const int iLF = iKF2 == INT_MAX ? m_LBA->m_ic2LF.front() : -1;
      const Rigid3D C2 = iKF2 == INT_MAX ? GetCameraLF(iLF).m_T : GetCameraKF(iKF2);
      switch (m_keyDrawPrfType) {
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE:
        v = Zp.GetRotationState(C1, C2).GetRodrigues(BA_ANGLE_EPSILON);
        break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_MEASUREMENT: {
        const Rotation3D R = Zp.GetRotationMeasurement(i, x.Data() + ip + 3, BA_ANGLE_EPSILON);
        v = R.GetRodrigues(BA_ANGLE_EPSILON);
        break; }
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_ERROR:
        v = Zp.GetRotationError(C1, C2, i, x.Data() + ip + 3, BA_ANGLE_EPSILON);
        break;
      case DRAW_PRF_CAMERA_PRIOR_ROTATION_COVARIANCE:
        S.GetDiagonal(ip + 3, v);
        v.MakeSquareRoot();
        break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_STATE:
        v = Zp.GetPositionState(C1, C2);
        break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_MEASUREMENT:
        v = Zp.GetPositionMeasurement(i, x.Data() + ip);
        break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_ERROR:
        v = Zp.GetPositionError(C1, C2, i, x.Data() + ip);
        break;
      case DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE:
        S.GetDiagonal(ip, v);
        v.MakeSquareRoot();
        break;
      }
      iFrm = iKF2 == INT_MAX ? GetKeyFrames() : iKF2;
    }
    const float _x = dx + iFrm * ratio;
    glColor3ub(255, 0, 0);  Viewer::DrawCross(Point2D(_x, v.x() * sv + yb), size);
    glColor3ub(0, 255, 0);  Viewer::DrawCross(Point2D(_x, v.y() * sv + yb), size);
    glColor3ub(0, 0, 255);  Viewer::DrawCross(Point2D(_x, v.z() * sv + yb), size);
  }
}

bool ViewerIBA::DrawProfileTypeValid(const int iFrm) {
  const int _iFrm = iFrm == -1 ? m_iFrmActive : iFrm;
  const int nKFs = GetKeyFrames(), nLFs = GetLocalFrames();
  const int iKF = _iFrm < nKFs ? _iFrm : -1;
  const int iLF = iKF == -1 ? (_iFrm - nKFs + m_iLF + 1) % nLFs : -1;
  if (m_keyDrawPrfType == DRAW_PRF_ACCELERATION || m_keyDrawPrfType == DRAW_PRF_GYROSCOPE) {
    return iKF != -1 ? !m_GBA->m_KFs[iKF].m_us.Empty() :
                       !((LocalFrame *) GetLocalFrame(iLF))->m_us.Empty();
  } else if (m_keyDrawPrfType == DRAW_PRF_ACCELERATION_DEVICE ||
             m_keyDrawPrfType == DRAW_PRF_GYROSCOPE_DEVICE) {
    return iKF != -1 ? GetCameraGBALM(iKF - 1).Valid() :
                       GetCameraLF((iLF - 1 + nLFs) % nLFs).Valid();
  } else if (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) {
    return iKF != -1 ? GetCameraGBALM(iKF - 1).Valid() && GetIMUDeltaGBALM(iKF).Valid() :
                       _iFrm != nKFs && GetCameraLF((iLF - 1 + nLFs) % nLFs).Valid() &&
                                        GetIMUDeltaLF(iLF).Valid();
  } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_POSITION_COVARIANCE) {
    if (iKF != -1) {
      const int NZ = static_cast<int>(m_GBA->m_Zps.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        if (m_GBA->m_Zps[iZ].m_iKFr == iKF) {
          return true;
        }
      }
      return false;
    } else {
      return m_LBA->m_Zp.Pose::Valid();
    }
  } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_VELOCITY_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) {
    return true;
  } else if (m_keyDrawPrfType == DRAW_PRF_REPROJECTION_ERROR) {
    return true;
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_ABSOLUTE &&
             m_keyDrawPrfType < DRAW_PRF_STATE_VELOCITY) {
    return iKF != -1 ? GetCameraKF(iKF).Valid() : GetCameraLF(iLF).Valid()
#ifdef CFG_GROUND_TRUTH
        && ((m_keyDrawPrfType - DRAW_PRF_STATE_ROTATION_ABSOLUTE) % 2 == 0
        //|| !m_solver->m_internal->m_CsGT.Empty())
        || m_LBA->m_CsGT)
#endif
        ;
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_VELOCITY) {
    return iKF != -1 ? GetCameraGBALM(iKF).Valid() : GetCameraLF(iLF).Valid()
#ifdef CFG_GROUND_TRUTH
        && (((m_keyDrawPrfType - DRAW_PRF_STATE_VELOCITY) & 1) == 0
        //|| !m_solver->m_internal->m_CsGT.Empty())
        || m_LBA->m_CsGT)
#endif
        ;
  } else {
    return false;
  }
}

void ViewerIBA::DrawProfileTypeStep(const int *types, const int N, const bool next) {
  const int typeBkp = m_keyDrawPrfType;
  int i = static_cast<int>(std::upper_bound(types, types + N, typeBkp) - types) - 1;
  const int typeOffset = typeBkp - types[i];
  while (1) {
    if (next) {
      if (++i == N) {
        break;
      }
    } else {
      if (--i < 0) {
        break;
      }
    }
    const int type = types[i] + typeOffset;
    if (type < types[i + 1]) {
      m_keyDrawPrfType.Set(type);
    } else {
      m_keyDrawPrfType.Set(types[i]);
    }
    if (DrawProfileTypeValid()) {
      break;
    }
  }
  if (!DrawProfileTypeValid()) {
    m_keyDrawPrfType.Set(typeBkp);
  }
}

int ViewerIBA::GetDrawProfileScaleType() {
  if (m_keyDrawPrfType >= DRAW_PRF_ACCELERATION && m_keyDrawPrfType <= DRAW_PRF_GYROSCOPE_DEVICE) {
    return m_keyDrawPrfType / 2 * 2;
  } else if (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_VELOCITY_COVARIANCE) {
    return DRAW_PRF_IMU_DELTA_ROTATION_STATE +
          (m_keyDrawPrfType - DRAW_PRF_IMU_DELTA_ROTATION_STATE) / 4 * 4;
  } else if (m_keyDrawPrfType >= DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_IMU_DELTA_BIAS_GYROSCOPE_COVARIANCE) {
    return DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE +
          (m_keyDrawPrfType - DRAW_PRF_IMU_DELTA_BIAS_ACCELERATION_STATE) / 2 * 2;
  } else if (m_keyDrawPrfType >= DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE &&
             m_keyDrawPrfType <= DRAW_PRF_CAMERA_PRIOR_BIAS_GYROSCOPE_COVARIANCE) {
    return DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE +
          (m_keyDrawPrfType - DRAW_PRF_CAMERA_PRIOR_ROTATION_STATE) / 4 * 4;
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_ROTATION_ABSOLUTE &&
             m_keyDrawPrfType < DRAW_PRF_STATE_POSITION_ABSOLUTE) {
    return DRAW_PRF_STATE_ROTATION_ABSOLUTE;
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_POSITION_ABSOLUTE &&
             m_keyDrawPrfType < DRAW_PRF_STATE_VELOCITY) {
    return DRAW_PRF_STATE_POSITION_ABSOLUTE;
  } else if (m_keyDrawPrfType >= DRAW_PRF_STATE_VELOCITY) {
    return DRAW_PRF_STATE_VELOCITY + (m_keyDrawPrfType - DRAW_PRF_STATE_VELOCITY) / 2 * 2;
  } else {
    return m_keyDrawPrfType;
  }
}

UT::ES<FTR::ESError, FTR::ESIndex> ViewerIBA::ComputeReprojectionError(const int iKF,
                                                                       const int iLF) {
#ifdef CFG_DEBUG
  UT_ASSERT(iKF != -1 && iLF == -1 || iKF == -1 && iLF != -1);
#endif
  const FRM::Frame &F = *(iKF != -1 ? GetKeyFrame(iKF) : GetLocalFrame(iLF));
  UT::ES<FTR::ESError, FTR::ESIndex> ES;
  ES.Initialize(true);

  Rigid3D Tr[2];
  FTR::Error e;
  LA::Vector2f _e;
  LA::SymmetricMatrix2x2f W;
  const float r2Max = ME::Variance<ME::FUNCTION_HUBER>();
  const Rigid3D C = iKF != -1 ? GetCameraKF(iKF) : GetCameraLF(iLF).m_T;
  const int NZ = int(F.m_Zs.size());
  for (int iZ = 0; iZ < NZ; ++iZ) {
    const FRM::Measurement &Z = F.m_Zs[iZ];
    const Rigid3D _C = GetCameraKF(Z.m_iKF);
    *Tr = C / _C;
#ifdef CFG_STEREO
    Tr[1] = Tr[0];
    Tr[1].SetTranslation(m_K.m_br + Tr[0].GetTranslation());
#endif
    const KeyFrame &KF = *(KeyFrame *) GetKeyFrame(Z.m_iKF);
    for (int iz = Z.m_iz1; iz < Z.m_iz2; ++iz) {
      const FTR::Measurement &z = F.m_zs[iz];
      const Depth::InverseGaussian d = GetFeatureDepth(Z.m_iKF, z.m_ix);
      FTR::GetError(Tr, KF.m_xs[z.m_ix], d, z, e);
      const FTR::ESIndex idx(KF.m_T.m_iFrm, z.m_ix/*, F.m_T.m_iFrm, iz*/);
#ifdef CFG_STEREO
      for (int i = 0; i < 2; ++i)
#endif
      {
#ifdef CFG_STEREO
        if (i == 0 && z.m_z.Invalid() || i == 1 && z.m_zr.Invalid()) {
          continue;
        }
        if (i == 1) {
          _e = e.m_er;
          W = z.m_Wr;
        }
        else
#endif
        {
          _e = e.m_e;
          W = z.m_W;
        }
        const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(W, _e);
        //const float F = ME::Weight<ME::FUNCTION_HUBER>(r2) * r2;
        const float F = ME::Cost<ME::FUNCTION_HUBER>(r2);
        const FTR::ESError __e(m_K.m_K, _e);
        const bool r = r2 < r2Max;
        ES.Accumulate(__e, F, idx, r);
      }
    }
  }
#ifdef CFG_STEREO
  if (iKF != -1) {
    const KeyFrame &KF = *((KeyFrame *) GetKeyFrame(iKF));
    const int Nx = int(KF.m_xs.size());
    for (int ix = 0; ix < Nx; ++ix) {
      const FTR::Source &x = KF.m_xs[ix];
      if (x.m_xr.Invalid()) {
        continue;
      }
      const Depth::InverseGaussian d = GetFeatureDepth(iKF, ix);
      FTR::GetError(m_K.m_br, d, x, e.m_er);
      const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, e.m_er);
      const float F = ME::Cost<LBA_ME_FUNCTION>(r2) * r2;
      const FTR::ESIndex idx(KF.m_T.m_iFrm, ix);
      const FTR::ESError _e(m_K.m_Kr, e.m_er);
      const bool r = r2 < r2Max;
      ES.Accumulate(_e, F, idx, r);
    }
  }
#endif
  return ES;
}
