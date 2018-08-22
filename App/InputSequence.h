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
#ifndef _INPUT_SEQUENCE_H_
#define _INPUT_SEQUENCE_H_
//#ifndef CFG_DEBUG
//#define CFG_DEBUG
//#endif

#include "IBA.h"
#include "Configurator.h"
#include "IMU.h"
#include "CameraTrajectory.h"
#include "Depth.h"

// IBA_WITH_CVD is automatically detected and defined by the top level CMakeLists.
// For Windows, we simply assume IBA runs with CVD and manually define IBA_WITH_CVD
#ifdef WIN32
#ifndef IBA_WITH_CVD
#define IBA_WITH_CVD
#endif
#endif

#ifdef IBA_WITH_CVD
#include "UtilityWM.h"
#endif
class InputSequence {
 public:
  inline InputSequence(const Configurator &cfgor) {
    m_dir = cfgor.GetArgument("input_directory");
    const std::string fileImg = m_dir + cfgor.GetArgument("input_image", "l/*.png");
    const std::string fileFtr = m_dir + cfgor.GetArgument("input_feature", "l_det/*.yml");
    const std::string fileDKF = m_dir + cfgor.GetArgument("input_delete_keyframe", "kf_del/*.txt");
    const std::string fileDMP = m_dir + cfgor.GetArgument("input_delete_map_point", "mp_del/*.txt");
    const std::string fileUpd = m_dir + cfgor.GetArgument("input_update_camera", "kf_upd/*.txt");
    const int iStart = cfgor.GetArgument("input_start", 0);
    const int iStep = cfgor.GetArgument("input_step", 1);
    const int iEnd = cfgor.GetArgument("input_end", INT_MAX);
    const std::string dirImg = UT::FileNameExtractDirectory(fileImg);
    const std::string dirFtr = UT::FileNameExtractDirectory(fileFtr);
    const std::string dirDKF = UT::FileNameExtractDirectory(fileDKF);
    const std::string dirDMP = UT::FileNameExtractDirectory(fileDMP);
    const std::string dirUpd = UT::FileNameExtractDirectory(fileUpd);
    const std::string extImg = UT::FileNameExtractExtension(fileImg);
    const std::string extFtr = UT::FileNameExtractExtension(fileFtr);
    const std::string extDKF = UT::FileNameExtractExtension(fileDKF);
    const std::string extDMP = UT::FileNameExtractExtension(fileDMP);
    const std::string extUpd = UT::FileNameExtractExtension(fileUpd);
    m_filesImg = UT::FilesSearch(fileImg, iStart, iStep, iEnd);
    m_filesFtr = UT::FilesSearch(fileFtr, iStart, iStep, iEnd);
    m_filesDKF = UT::FilesSearch(fileDKF, iStart, iStep, iEnd);
    m_filesDMP = UT::FilesSearch(fileDMP, iStart, iStep, iEnd);
    m_filesUpd = UT::FilesSearch(fileUpd, iStart, iStep, iEnd);
    if (m_filesImg.size() != m_filesFtr.size()) {
      m_filesImg = UT::FilesSearch(m_filesFtr, dirImg, extImg);
    }
    if (m_filesDKF.size() != m_filesFtr.size()) {
      m_filesDKF = UT::FilesSearch(m_filesFtr, dirDKF, extDKF);
    }
    if (m_filesDMP.size() != m_filesFtr.size()) {
      m_filesDMP = UT::FilesSearch(m_filesFtr, dirDMP, extDMP);
    }
    if (m_filesUpd.size() != m_filesFtr.size()) {
      m_filesUpd = UT::FilesSearch(m_filesFtr, dirUpd, extUpd);
    }
    m_binary = extFtr == "dat";
#ifdef CFG_STEREO
    const std::string fileImgRight = m_dir + cfgor.GetArgument("input_image_right", "r/*.png");
    m_filesImgRight = UT::FilesSearch(fileImgRight, iStart, iStep, iEnd);
    if (m_binary) {
      m_filesFtrRight.assign(m_filesFtr.size(), "");
      if (m_filesImgRight.size() != m_filesFtr.size()) {
        m_filesImgRight = UT::FilesSearch(m_filesFtr, UT::FileNameExtractDirectory(fileImgRight),
                                          extImg);
      }
    } else {
      const std::string fileFtrRight = m_dir + cfgor.GetArgument("input_feature_right", "r_det/*.yml");
      m_filesFtrRight = UT::FilesSearch(fileFtrRight, iStart, iStep, iEnd);
      if (m_filesFtrRight.size() != m_filesFtr.size()) {
        m_filesFtrRight = UT::FilesSearch(m_filesFtr, UT::FileNameExtractDirectory(fileFtrRight),
                                          extFtr);
      }
      if (m_filesImgRight.size() != m_filesFtrRight.size()) {
        m_filesImgRight = UT::FilesSearch(m_filesFtrRight, UT::FileNameExtractDirectory(fileImgRight),
                                          extImg);
      }
#ifdef CFG_DEBUG
      UT_ASSERT(m_filesFtrRight.size() == m_filesFtr.size());
#endif
    }
#endif
    const double fps = cfgor.GetArgument("input_fps", 30.0);
    const bool tReset = cfgor.GetArgument("input_time_reset", 0) != 0;
    const double tFactor = cfgor.GetArgument("input_time_factor", 1.0e-4);
    //m_tFirst = cfgor.GetArgument("input_time_first", DBL_MAX);
    m_tFirst = cfgor.GetArgument("input_time_first", 0.0);
    m_dt = 1.0 / fps;

    double t;
    const int nImgs = static_cast<int>(m_filesFtr.size());
    m_ts.resize(nImgs);
    for (int iImg = 0; iImg < nImgs; ++iImg) {
      if (tReset) {
        t = iImg * m_dt;
      } else if (m_binary) {
        IBA::LoadCurrentFrameTime(m_filesFtr[iImg], &t);
      } else {
        t = UT::FileNameExtractSuffix<double>(m_filesFtr[iImg]) * tFactor;
      }
      if (m_tFirst == DBL_MAX && iImg == 0) {
        m_tFirst = t;
      }
      m_ts[iImg] = float(t - m_tFirst);
#ifdef CFG_DEBUG
      if (iImg > 0) {
        UT_ASSERT(m_ts[iImg] > m_ts[iImg - 1]);
      }
#endif
    }
    if (iStart != 0 || iStep != 1) {
      const int nFrms2 = static_cast<int>(m_ts.size()), nFrms1 = iStart + iStep * nFrms2;
      m_iFrms.assign(nFrms1, -1);
      for (int iFrm2 = 0; iFrm2 < nFrms2; ++iFrm2) {
        const int iFrm1 = iStart + iStep * iFrm2;
        m_iFrms[iFrm1] = iFrm2;
      }
    } else {
      m_iFrms.resize(0);
    }

    m_us.Resize(0);
    m_ius.resize(0);
    const std::string fileIMU = m_dir + cfgor.GetArgument("input_imu", "imu_data.txt");
    const double tFactorIMU = cfgor.GetArgument("input_imu_time_factor", tFactor);
    const double tFirstIMU = cfgor.GetArgument("input_imu_time_first", m_tFirst);
    FILE *fp = fopen(fileIMU.c_str(), "r");
    if (fp) {
      int iFrm = -1;
      IMU::Measurement u;
      char line[UT_STRING_WIDTH_MAX];
      const int nFrms = static_cast<int>(m_ts.size());
      std::vector<int> Nus(nFrms, 0);
      const bool csv = UT::FileNameExtractExtension(fileIMU) == "csv";
      const bool na = cfgor.GetArgument("input_imu_minus_acceleration", 0) != 0;
      //while (UT::Load<double>(t, fp) && UT::Load<float>(u.m_a, 3, fp) && UT::Load<float>(u.m_w, 4, fp)) {
      while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
        if (csv && sscanf(line, "%lf,%f,%f,%f,%f,%f,%f", &t, &u.m_w[0], &u.m_w[1], &u.m_w[2],
                                                             &u.m_a[0], &u.m_a[1], &u.m_a[2]) != 7 ||
           !csv && sscanf(line, "%lf %f %f %f %f %f %f", &t, &u.m_a[0], &u.m_a[1], &u.m_a[2],
                                                             &u.m_w[0], &u.m_w[1], &u.m_w[2]) != 7) {
          break;
        }
        u.t() = float(t * tFactorIMU - tFirstIMU);
        if ((iFrm == -1 || u.t() >= m_ts[iFrm]) && ++iFrm >= nFrms) {
          break;
        }
        if (na) {
          u.m_a.MakeMinus();
        }
        ++Nus[iFrm];
        m_us.Push(u);
        if (iFrm == 0 && u.t() == m_ts[iFrm]) {
          m_us.Front().t() -= FLT_EPSILON;
          ++Nus[++iFrm];
          m_us.Push(u);
        }
      }
      fclose(fp);
      m_ius.resize(nFrms + 1);
      m_ius[0] = 0;
      for (int iFrm = 0; iFrm < nFrms; ++iFrm) {
        m_ius[iFrm + 1] = m_ius[iFrm] + Nus[iFrm];
      }
      UT::PrintLoaded(fileIMU);
    }
    const std::string fileCalib = m_dir + cfgor.GetArgument("input_calibration_file",
                                                            "calibration.txt");
    if (UT::FileNameExtractExtension(fileCalib) == "dat") {
      IBA::LoadCalibration(fileCalib, &m_K);
    } else {
      if ((m_K.w = cfgor.GetArgument("input_width", 0)) == 0 ||
          (m_K.h = cfgor.GetArgument("input_height", 0)) == 0) {
#ifdef IBA_WITH_CVD
        CVD::Image<ubyte> I, ITmp;
        UT::ImageLoad(m_filesImg[0], I, ITmp);
        m_K.w = I.size().x;
        m_K.h = I.size().y;
#endif
      }
      if (m_K.w == 0 || m_K.h == 0) {
        m_K.w = 640;
        m_K.h = 480;
      }
      m_K.fishEye = cfgor.GetArgument("input_calibration_fish_eye", 0) != 0;
      if (!IBA::LoadCalibration(fileCalib, m_K.Tu, &m_K.K, m_K.ba, m_K.bw/*, m_K.sa*/)) {
        if ((m_K.K.fx = cfgor.GetArgument("input_camera_focal_x", 0.0f)) == 0.0f &&
            (m_K.K.fx = cfgor.GetArgument("input_camera_fov_x", 0.0f)) != 0.0f) {
          m_K.K.fx = Intrinsic::FovToFocal(m_K.w, m_K.K.fx);
        }
        if ((m_K.K.fy = cfgor.GetArgument("input_camera_focal_y", 0.0f)) == 0.0f &&
            (m_K.K.fy = cfgor.GetArgument("input_camera_fov_y", 0.0f)) != 0.0f) {
          m_K.K.fy = Intrinsic::FovToFocal(m_K.h, m_K.K.fy);
        }
        if (m_K.K.fx == 0.0f && m_K.K.fy == 0.0f) {
          UT::Error("No Intrinsic!\n");
        } else if (m_K.K.fx == 0.0f && m_K.K.fy != 0.0f) {
          m_K.K.fx = m_K.K.fy;
        } else if (m_K.K.fx != 0.0f && m_K.K.fy == 0.0f) {
          m_K.K.fy = m_K.K.fx;
        }
        if ((m_K.K.cx = cfgor.GetArgument("input_camera_center_x", 0.0f)) == 0.0f) {
          m_K.K.cx = (m_K.w - 1) * 0.5f;
        }
        if ((m_K.K.cy = cfgor.GetArgument("input_camera_center_y", 0.0f)) == 0.0f) {
          m_K.K.cy = (m_K.h - 1) * 0.5f;
        }
        memset(m_K.K.ds, 0, sizeof(m_K.K.ds));
        const std::string r[3] = {cfgor.GetArgument("input_imu_rotation_x"),
                                  cfgor.GetArgument("input_imu_rotation_y"),
                                  cfgor.GetArgument("input_imu_rotation_z")};
        memset(m_K.Tu, 0, sizeof(m_K.Tu));
        for (int i = 0; i < 3; ++i) {
          if (r[i] == "x") {
            m_K.Tu[0][i] = 1.0f;
          } else if (r[i] == "-x") {
            m_K.Tu[0][i] = -1.0f;
          } else if (r[i] == "y") {
            m_K.Tu[1][i] = 1.0f;
          } else if (r[i] == "-y") {
            m_K.Tu[1][i] = -1.0f;
          } else if (r[i] == "z") {
            m_K.Tu[2][i] = 1.0f;
          } else if (r[i] == "-z") {
            m_K.Tu[2][i] = -1.0f;
          } else {
            m_K.Tu[0][0] = FLT_MAX;
            break;
          }
        }
        m_K.Tu[0][3] = cfgor.GetArgument("input_imu_position_x", 0.0f);
        m_K.Tu[1][3] = cfgor.GetArgument("input_imu_position_y", 0.0f);
        m_K.Tu[2][3] = cfgor.GetArgument("input_imu_position_z", 0.0f);
      }
#ifdef CFG_STEREO
      const std::string fileCalibRight = m_dir + cfgor.GetArgument("input_calibration_file_right",
                                                                   "calibration_right.txt");
      if (!IBA::LoadCalibration(fileCalibRight, m_K.Tr, &m_K.Kr)) {
        m_K.Tr[0][0] = FLT_MAX;
        m_K.Tr[0][3] = cfgor.GetArgument("input_calibration_right_position_x", 0.0f);
        m_K.Tr[1][3] = cfgor.GetArgument("input_calibration_right_position_y", 0.0f);
        m_K.Tr[2][3] = cfgor.GetArgument("input_calibration_right_position_z", 0.0f);
      }
#endif
    }
//#if 0
#if 1
    IBA::PrintCalibration(m_K);
#endif
    Rigid3D Tu;
    Tu.Set(m_K.Tu);
    if (cfgor.GetArgument("tune_imu_calibration", 0)) {
      const float rx = cfgor.GetArgument("tune_imu_calibration_rotation_x", 0.0f);
      const float ry = cfgor.GetArgument("tune_imu_calibration_rotation_y", 0.0f);
      const float rz = cfgor.GetArgument("tune_imu_calibration_rotation_z", 0.0f);
      const float px = cfgor.GetArgument("tune_imu_calibration_position_x", 0.0f);
      const float py = cfgor.GetArgument("tune_imu_calibration_position_y", 0.0f);
      const float pz = cfgor.GetArgument("tune_imu_calibration_position_z", 0.0f);
      const float bax = cfgor.GetArgument("tune_imu_calibration_bias_acceleration_x", 0.0f);
      const float bay = cfgor.GetArgument("tune_imu_calibration_bias_acceleration_y", 0.0f);
      const float baz = cfgor.GetArgument("tune_imu_calibration_bias_acceleration_z", 0.0f);
      const float bwx = cfgor.GetArgument("tune_imu_calibration_bias_gyroscope_x", 0.0f);
      const float bwy = cfgor.GetArgument("tune_imu_calibration_bias_gyroscope_y", 0.0f);
      const float bwz = cfgor.GetArgument("tune_imu_calibration_bias_gyroscope_z", 0.0f);
      Rotation3D dR;
      const LA::AlignedVector3f dr = LA::AlignedVector3f(rx, ry, rz) * UT_FACTOR_DEG_TO_RAD;
      dR.SetRodrigues(dr, BA_ANGLE_EPSILON);
      const Rotation3D R = Rotation3D(Tu) / dR;
      const LA::AlignedVector3f t = Tu.GetTranslation() + LA::AlignedVector3f(px, py, pz);
      Tu.Set(R, t);
      Tu.Get(m_K.Tu);
      m_K.ba[0] += bax;
      m_K.ba[1] += bay;
      m_K.ba[2] += baz;
      m_K.bw[0] += bwx * UT_FACTOR_DEG_TO_RAD;
      m_K.bw[1] += bwy * UT_FACTOR_DEG_TO_RAD;
      m_K.bw[2] += bwz * UT_FACTOR_DEG_TO_RAD;
    }
    
    const Rotation3D *Ru = Tu.Valid() ? &Tu : NULL;
    const std::string fileGT = m_dir + cfgor.GetArgument("input_ground_truth", "est_gt.csv");
    const std::string fileGTMot = m_dir + cfgor.GetArgument("input_ground_truth_motion");
    const std::string fileGTDep = m_dir + cfgor.GetArgument("input_ground_truth_depth");
    const double tFactorGT = cfgor.GetArgument("input_ground_truth_time_factor", tFactor);
    const double tFirstGT = cfgor.GetArgument("input_ground_truth_time_first", m_tFirst);
    //const float dtMaxGT = cfgor.GetArgument("input_ground_truth_max_time_difference", FLT_MAX);
    const float dtMaxGT = cfgor.GetArgument("input_ground_truth_max_time_difference", 1.0e-2f);
    const bool inverse = cfgor.GetArgument("input_ground_truth_inverse", 0) != 0;
    const bool left = cfgor.GetArgument("input_ground_truth_left_hand", 0) != 0;
    const bool xyzw = cfgor.GetArgument("input_ground_truth_xyzw", 0) != 0;
    const bool babw = cfgor.GetArgument("input_ground_truth_babw", 0) != 0;
    const bool origin = cfgor.GetArgument("input_ground_truth_origin", 1) != 0;
    const bool gravity = cfgor.GetArgument("input_ground_truth_origin_gravity", 1) != 0;
    const int flag = (inverse ? CT_FLAG_INVERSE : CT_FLAG_DEFAULT)
                   | (left ? CT_FLAG_LEFT_HAND : CT_FLAG_DEFAULT)
                   | (xyzw ? CT_FLAG_XYZW : CT_FLAG_DEFAULT)
                   | (babw ? CT_FLAG_MOTION_BABW : CT_FLAG_DEFAULT)
                   | (origin ? CT_FLAG_ORIGIN : CT_FLAG_DEFAULT)
                   | (gravity? CT_FLAG_ORIGIN_GRAVITY: CT_FLAG_DEFAULT)
                   | (tReset ? CT_FLAG_TIME_RESET : CT_FLAG_DEFAULT);
    Rigid3D Ts;
    const std::string fileSensor = m_dir + cfgor.GetArgument("input_ground_truth_sensor",
                                                             "calibration_sensor.txt");
    if (!Ts.Load(fileSensor)) {
      Ts.MakeIdentity();
    }
    if (Tu.Valid() && cfgor.GetArgument("input_ground_truth_sensor_imu", 1)) {
      Ts = Tu * Ts;
    }
    if (UT::FileNameExtractExtension(fileGT) == "dat") {
      IBA::LoadGroundTruth(fileGT, &m_XsGT);
#ifdef CFG_DEBUG
      UT_ASSERT(m_XsGT.size() == m_ts.size());
#endif
      ConvertCameras(m_XsGT, &m_CTGT.m_Cs, &m_CTGT.m_ba, &m_CTGT.m_bw);
      //CameraTrajectory::TransformPose(Ts, m_CTGT.m_Cs);
      if (Ru) {
        CameraTrajectory::TransformBias(*Ru, m_CTGT.m_Cs, m_CTGT.m_ba, m_CTGT.m_bw);
      }
      if (origin) {
        CameraTrajectory::AlignToOrigin(m_CTGT.m_Cs, gravity);
      }
      ConvertCameras(m_CTGT.m_Cs, &m_XsGT);
      m_CTGT.m_ts = m_ts;
    } else if (m_CTGT.Load(fileGT, tFactorGT, tFirstGT, &m_ts, dtMaxGT, flag, &Ts, Ru)) {
      //if (dtMaxGT != FLT_MAX) {
      if (dtMaxGT >= 0.0f && dtMaxGT != FLT_MAX) {
        CameraTrajectory CTGT;
        std::vector<int> iFrms;
        const int nFrms1 = static_cast<int>(m_ts.size());
        iFrms.assign(nFrms1, -1);
        for (int iFrm1 = 0, iFrm2 = 0; iFrm1 < nFrms1; ++iFrm1) {
          const int i = m_CTGT.Search(m_ts[iFrm1], dtMaxGT);
          if (i == -1) {
            continue;
          }
          iFrms[iFrm1] = iFrm2;
          m_ts[iFrm2] = m_ts[iFrm1];
          m_filesImg[iFrm2] = m_filesImg[iFrm1];
          m_filesFtr[iFrm2] = m_filesFtr[iFrm1];
          m_filesDKF[iFrm2] = m_filesDKF[iFrm1];
          m_filesDMP[iFrm2] = m_filesDMP[iFrm1];
          m_filesUpd[iFrm2] = m_filesUpd[iFrm1];
#ifdef CFG_STEREO
          m_filesImgRight[iFrm2] = m_filesImgRight[iFrm1];
          m_filesFtrRight[iFrm2] = m_filesFtrRight[iFrm1];
#endif
          if (!m_us.Empty()) {
            m_ius[iFrm2 + 1] = m_ius[iFrm1 + 1];
          }
          CTGT.Push(m_CTGT, i);
          ++iFrm2;
        }
        if (m_iFrms.empty()) {
          m_iFrms.swap(iFrms);
        } else {
          const int nFrms = static_cast<int>(m_iFrms.size());
          for (int iFrm = 0; iFrm < nFrms; ++iFrm) {
            const int jFrm = m_iFrms[iFrm];
            if (jFrm != -1) {
              m_iFrms[iFrm] = iFrms[jFrm];
            }
          }
        }
        m_CTGT.Swap(CTGT);
        const int nFrms2 = m_CTGT.Size();
        m_ts.resize(nFrms2);
        m_filesImg.resize(nFrms2);
        m_filesFtr.resize(nFrms2);
        m_filesDKF.resize(nFrms2);
        m_filesDMP.resize(nFrms2);
        m_filesUpd.resize(nFrms2);
#ifdef CFG_STEREO
        m_filesImgRight.resize(nFrms2);
        m_filesFtrRight.resize(nFrms2);
#endif
        if (!m_us.Empty()) {
          m_ius.resize(nFrms2 + 1);
        }
      }
      ConvertCameras(m_CTGT.m_Cs, &m_XsGT);
    }
    if (!m_CTGT.Empty() && cfgor.GetArgument("input_ground_truth_bias", 0)) {
      m_CTGT.m_ba.Get(m_K.ba);
      m_CTGT.m_bw.Get(m_K.bw);
    }
    if (fileGTMot != m_dir && !IBA::LoadGroundTruth(fileGTMot, &m_XsGT)) {
#ifdef CFG_GROUND_TRUTH
      IBA::Solver solver;
      IBA::CurrentFrame CF;
      IBA::KeyFrame KF;
      LoadParameters(cfgor);
      solver.Create(m_K, 0, 0, (-1 << 8) | -1, 0, "", m_dir, &m_XsGT);
      solver.Start();
      const int nFrms = Size();
      for (int iFrm = 0; iFrm < nFrms; ++iFrm) {
        LoadCurrentFrame(iFrm, &CF, &KF, &solver);
        solver.PushIMUMeasurementsGT(CF);
        UT::Print("\r%d / %d = %f%%", iFrm + 1, nFrms, UT::Percentage(iFrm + 1, nFrms));
      }
      UT::Print("\n");
      solver.EstimateMotionGT(&m_XsGT);
      solver.Stop();
      solver.Destroy();
      IBA::SaveGroundTruth(fileGTMot, m_XsGT);
#endif
    }
    const std::string fileKF = m_dir + cfgor.GetArgument("input_keyframe",
                                                         "keyframe_decisions.txt");
    const float dtMaxKF = cfgor.GetArgument("input_keyframe_max_time_difference", 1.0e-2f);
    IBA::LoadKeyFrames(fileKF, m_ts, &m_kfs, dtMaxKF);

    if (fileGTDep != m_dir && !IBA::LoadGroundTruth(fileGTDep, &m_dsGT) && !m_XsGT.empty()) {
      const bool keyframeOnly = cfgor.GetArgument("input_ground_truth_depth_keyframe_only",
                                                  0) != 0;
#ifdef CFG_GROUND_TRUTH
      IBA::Solver solver;
      IBA::CurrentFrame CF;
      IBA::KeyFrame KF;
      LoadParameters(cfgor);
      solver.Create(m_K, 0, 0, (-1 << 8) | -1, 0, "", m_dir, &m_XsGT);
      solver.Start();
      const int nFrms = Size();
      for (int iFrm = 0; iFrm < nFrms; ++iFrm) {
        LoadCurrentFrame(iFrm, &CF, &KF, &solver);
        solver.PushDepthMeasurementsGT(CF, KF.iFrm == -1 ? NULL : &KF, keyframeOnly);
        UT::Print("\r%d / %d = %f%%", iFrm + 1, nFrms, UT::Percentage(iFrm + 1, nFrms));
      }
      UT::Print("\n");
      solver.TriangulateDepthsGT(&m_dsGT);
      solver.Stop();
      solver.Destroy();
      IBA::SaveGroundTruth(fileGTDep, m_dsGT);
#endif
    }

    const std::string fileZp = m_dir + cfgor.GetArgument("input_relative_constraint",
                                                          "relative_constraints.txt");
    if (IBA::LoadRelativeConstraints(fileZp, m_ts, &m_Zs, dtMaxKF)) {
      if (m_kfs.empty()) {
        m_kfs.assign(m_ts.size(), 0);
      }
      m_iFrm2Z.assign(m_ts.size(), -1);
      const int NZ = static_cast<int>(m_Zs.size());
      for (int iZ = 0; iZ < NZ; ++iZ) {
        IBA::RelativeConstraint &Z = m_Zs[iZ];
        if (Z.iFrm1 == -1 || Z.iFrm2 == -1) {
          Z.iFrm1 = Z.iFrm2 = -1;
        } else {
          m_iFrm2Z[Z.iFrm2] = iZ;
#ifdef CFG_DEBUG
//#if 0
          UT_ASSERT(m_kfs[Z.iFrm1] != 0 && m_kfs[Z.iFrm2] != 0);
#endif
          m_kfs[Z.iFrm1] = m_kfs[Z.iFrm2] = 2;
        }
      }
    }
    //return true;
  }

  inline int Size() const { return static_cast<int>(m_filesImg.size()); }

  inline int Search(const float t, const float dtMax = 0.0f) const {
    const int N = static_cast<int>(m_ts.size());
    const int i2 = static_cast<int>(std::lower_bound(m_ts.begin(), m_ts.end(), t) - m_ts.begin());
    const int i1 = i2 - 1;
    const float dt1 = i1 >= 0 && i1 < N ? fabs(m_ts[i1] - t) : FLT_MAX;
    const float dt2 = i2 >= 0 && i2 < N ? fabs(m_ts[i2] - t) : FLT_MAX;
    if (dt1 < dt2 && dt1 <= dtMax) {
      return i1;
    } else if (dt2 < dt1 && dt2 <= dtMax) {
      return i2;
    } else {
      return -1;
    }
  }

  inline bool LoadCurrentFrame(const int iFrm, IBA::CurrentFrame *CF, IBA::KeyFrame *KF,
                               IBA::Solver *solver) const {
    if (m_binary) {
      if (!IBA::LoadCurrentFrame(m_filesFtr[iFrm], CF, KF)) {
        return false;
      }
#ifdef CFG_DEBUG
      UT_ASSERT(CF->iFrm == iFrm);
#endif
      CF->fileName = m_filesImg[CF->iFrm];
#ifdef CFG_STEREO
      CF->fileNameRight = m_filesImgRight[CF->iFrm];
#endif
      if (KF->iFrm == -1) {
        if (iFrm < KF_FIRST_LOCAL_FRAMES && KF_MIN_FRAME_STEP > 0 &&
            iFrm - solver->GetKeyFrameIndex(solver->GetKeyFrames() - 1) == KF_MIN_FRAME_STEP) {
          KF->iFrm = CF->iFrm;
          KF->C = CF->C.C;
          KF->zs = CF->zs;
          KF->Xs.resize(0);
          KF->d = CF->d;
        }
      } else {
        if (!m_iFrms.empty()) {
          int i, j;
          const int NX = static_cast<int>(KF->Xs.size());
          for (int iX = 0; iX < NX; ++iX) {
            IBA::MapPoint &X = KF->Xs[iX];
            const int Nz = static_cast<int>(X.zs.size());
            for (i = j = 0; i < Nz; ++i) {
              IBA::MapPointMeasurement &z = X.zs[i];
              z.iFrm = m_iFrms[z.iFrm];
              if (z.iFrm != -1) {
                X.zs[j++] = z;
              }
            }
            X.zs.resize(j);
          }
        }
#if 0
        if (!KF->Xs.empty()) {
          std::vector<ubyte> kfs(iFrm + 1, 0);
          const std::vector<int> &iFrmsKF = solver->GetKeyFrameIndexes();
          const int nKFs = static_cast<int>(iFrmsKF.size());
          for (int iKF = 0; iKF < nKFs; ++iKF) {
            kfs[iFrmsKF[iKF]] = 1;
          }
          kfs[KF->iFrm] = 1;
          int iX, jX, i, j;
          const int NX = static_cast<int>(KF->Xs.size());
          for (iX = jX = 0; iX < NX; ++iX) {
            IBA::MapPoint &X = KF->Xs[iX];
            const int Nz = static_cast<int>(X.zs.size());
            for (i = j = 0; i < Nz; ++i) {
              const IBA::MapPointMeasurement &z = X.zs[i];
              if (kfs[z.iFrm]) {
                X.zs[j++] = z;
              }
            }
            if (j == 1 && X.zs.front().iFrm != KF->iFrm) {
              j = 0;
            }
            X.zs.resize(j);
            if (!X.zs.empty()) {
              KF->Xs[jX++] = X;
            }
          }
          KF->Xs.resize(jX);
        }
        if (KF->Xs.size() < KF_MIN_FEATURE_SROUCES && KF->zs.size() >= KF_MIN_FEATURE_MEASUREMENTS &&
            iFrm >= KF_FIRST_LOCAL_FRAMES &&
            (KF_MIN_FRAME_STEP <= 0 || iFrm - solver->GetKeyFrameIndex(solver->GetKeyFrames() - 1) != KF_MIN_FRAME_STEP) &&
            (m_kfs.empty() || (m_kfs[iFrm] != 2 && (m_kfs[iFrm] != 1 || KF_MIN_FRAME_STEP != -1)))) {
          KF->iFrm = -1;
          KF->zs.resize(0);
          KF->Xs.resize(0);
        }
#endif
      }
#ifdef CFG_DEBUG
      if (!m_ius.empty()) {
        LA::AlignedVector3f a, w;
        const int i1 = m_ius[iFrm], i2 = m_ius[iFrm + 1], N = i2 - i1;
        UT_ASSERT(static_cast<int>(CF->us.size()) == N);
        const IMU::Measurement *us = m_us.Data() + i1;
        for (int i = 0; i < N; ++i) {
          const IMU::Measurement &u1 = us[i];
          const IBA::IMUMeasurement &u2 = CF->us[i];
          a.Set(u2.a);
          w.Set(u2.w);
          u1.m_a.AssertEqual(a);
          u1.m_w.AssertEqual(w);
          UT::AssertEqual(u1.t(), u2.t);
        }
      }
#endif
      return true;
    }
    CF->iFrm = iFrm;
    CF->C.C.R[0][0] = FLT_MAX;
    std::vector<IBA::MapPoint> *Xs = KF_MIN_FRAME_STEP == -1 && !m_kfs.empty() && !m_kfs[iFrm] ?
                                     NULL : &KF->Xs;
    const std::vector<int> *iFrms = m_iFrms.empty() ? NULL : &m_iFrms;
    solver->LoadFeatures(m_filesFtr[iFrm], iFrm, &CF->zs, Xs, iFrms);
    //////////////////////////////////////////////////////////////////////////
    if (Xs && (Xs->empty() || (Xs->size() < KF_MIN_FEATURE_SROUCES &&
                               CF->zs.size() >= KF_MIN_FEATURE_MEASUREMENTS))) {
      Xs = NULL;
    }
    //////////////////////////////////////////////////////////////////////////
#ifdef CFG_STEREO
    solver->LoadFeatures(m_filesFtrRight[iFrm], iFrm, &CF->zs, Xs, iFrms, 1);
#endif
    const int i1 = m_ius[iFrm], i2 = m_ius[iFrm + 1], N = i2 - i1;
    const IMU::Measurement *us = m_us.Data() + i1;
    CF->us.resize(N);
    for (int i = 0; i < N; ++i) {
      const IMU::Measurement &u1 = us[i];
      IBA::IMUMeasurement &u2 = CF->us[i];
      u1.m_a.Get(u2.a);
      u1.m_w.Get(u2.w);
      u2.t = u1.t();
    }
    CF->t = m_ts[iFrm];
    CF->d.d = 0.0f;
    CF->d.s2 = 0.0f;
    CF->fileName = m_filesImg[iFrm];
#ifdef CFG_STEREO
    CF->fileNameRight = m_filesImgRight[iFrm];
#endif
    if (Xs || iFrm < KF_FIRST_LOCAL_FRAMES ||
       (KF_MIN_FRAME_STEP > 0 && iFrm - solver->GetKeyFrameIndex(solver->GetKeyFrames() - 1) == KF_MIN_FRAME_STEP) ||
       (!m_kfs.empty() && (m_kfs[iFrm] == 2 || m_kfs[iFrm] == 1 && KF_MIN_FRAME_STEP == -1))) {
      KF->iFrm = CF->iFrm;
      KF->C = CF->C.C;
      KF->zs = CF->zs;
      if (!Xs) {
        KF->Xs.resize(0);
      }
      KF->d = CF->d;
    } else {
      KF->iFrm = -1;
      KF->zs.resize(0);
      KF->Xs.resize(0);
    }
    return true;
  }
  inline bool LoadRelativeConstraint(const int iFrm, IBA::RelativeConstraint *Z) const {
    if (m_iFrm2Z.empty() || m_iFrm2Z[iFrm] == -1) {
      Z->iFrm1 = Z->iFrm2 = -1;
      return false;
    } else {
      *Z = m_Zs[m_iFrm2Z[iFrm]];
#ifdef CFG_DEBUG
      UT_ASSERT(m_kfs[Z->iFrm1] == 2 && Z->iFrm2 == iFrm);
#endif
      return true;
    }
  }
  inline bool LoadDeleteKeyFrames(const int iFrm, std::vector<int> *iFrms) const {
    return IBA::LoadKeyFrames(m_filesDKF[iFrm], iFrms);
  }
  inline bool LoadDeleteMapPoints(const int iFrm, std::vector<int> *idxs) const {
    return IBA::LoadMapPoints(m_filesDMP[iFrm], idxs);
  }
  inline bool LoadUpdateCameras(const int iFrm, std::vector<int> *iFrms,
                                std::vector<IBA::CameraPose> *Cs) const {
    return IBA::LoadCameraPoses(m_filesUpd[iFrm], iFrms, Cs);
  }

  static inline void ConvertCameras(const AlignedVector<Camera> &Cs,
                                    std::vector<IBA::CameraIMUState> *Xs) {
    const int N = Cs.Size();
    Xs->resize(N);
    for (int i = 0; i < N; ++i) {
      const Camera &C = Cs[i];
      IBA::CameraIMUState &X = Xs->at(i);
      C.m_T.LA::AlignedMatrix3x3f::Get(X.C.R);
      C.m_p.Get(X.C.p);
      C.m_v.Get(X.v);
      C.m_ba.Get(X.ba);
      C.m_bw.Get(X.bw);
    }
  }
  static inline void ConvertCameras(const std::vector<IBA::CameraIMUState> &Xs,
                                    AlignedVector<Camera> *Cs,
                                    LA::AlignedVector3f *ba = NULL,
                                    LA::AlignedVector3f *bw = NULL) {
    const int N = static_cast<int>(Xs.size());
    Cs->Resize(N);
    Camera *_Cs = Cs->Data();
    for (int i = 0; i < N; ++i) {
      const IBA::CameraIMUState &X = Xs[i];
      Camera &C = _Cs[i];
      C.m_T.LA::AlignedMatrix3x3f::Set(X.C.R);
      C.m_p.Set(X.C.p);
      C.m_T.SetPosition(C.m_p);
      C.m_v.Set(X.v);
      C.m_ba.Set(X.ba);
      C.m_bw.Set(X.bw);
    }
    if (ba) {
      ba->MakeZero();
      for (int i = 0; i < N; ++i) {
        *ba += _Cs[i].m_ba;
      }
      *ba *= 1.0f / N;
    }
    if (bw) {
      bw->MakeZero();
      for (int i = 0; i < N; ++i) {
        *bw += _Cs[i].m_bw;
      }
      *bw *= 1.0f / N;
    }
  }

 public:

  IBA::Calibration m_K;

  ubyte m_binary;

  double m_dt, m_tFirst;
  std::vector<float> m_ts;
  std::string m_dir;
  std::vector<std::string> m_filesImg, m_filesFtr, m_filesDKF, m_filesDMP, m_filesUpd;
#ifdef CFG_STEREO
  std::vector<std::string> m_filesImgRight, m_filesFtrRight;
#endif
  std::vector<int> m_iFrms;
  AlignedVector<IMU::Measurement> m_us;
  std::vector<int> m_ius;
  CameraTrajectory m_CTGT;
  std::vector<IBA::CameraIMUState> m_XsGT;
  std::vector<IBA::Depth> m_dsGT;
  std::vector<ubyte> m_kfs;
  std::vector<int> m_iFrm2Z;
  std::vector<IBA::RelativeConstraint> m_Zs;

};
#endif
