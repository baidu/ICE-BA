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


// solver.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "InputSequence.h"
#include "Parameter.h"
#include "Timer.h"
#include "MultiThread.h"

#ifdef IBA_WITH_CVD
#include "ViewerIBA.h"
#endif

#include <gtest/gtest.h>
#ifndef WIN32
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <chrono>
#include <thread>
#elif defined IBA_WITH_CVD
#include "Z:/CVD/lib/link.h"
#include "Z:/Graphics/glut/lib/link.h"
#include "Z:/Graphics/glew/lib/link.h"
#include "Z:/GoogleTest/lib/link.h"
#endif
#include <string>
#include <vector>

bool RunSolver(const Configurator &cfgor, const InputSequence &IS, const std::string param = "",
               IBA::Time *tLBA = NULL, IBA::Time *tGBA = NULL
#ifdef CFG_GROUND_TRUTH
             , LA::AlignedVectorXf *eps = NULL, LA::AlignedVectorXf *ers = NULL
#endif
             ) {
  const int nFrms = IS.Size();
  if (nFrms == 0) {
    return false;
  }

  std::vector<int> seeds;
  const int seed = cfgor.GetArgument("random_seed", 0);
  if (seed != 0) {
    srand(seed);
    seeds.resize(nFrms);
    for (int iFrm = 0; iFrm < nFrms; ++iFrm) {
      seeds[iFrm] = UT::Random<int>();
    }
  }

  IBA::Solver solver;
  const int serial = (cfgor.GetArgument("serial_lba", IBA_SERIAL_NONE) & 255) |
                    ((cfgor.GetArgument("serial_gba", IBA_SERIAL_NONE) & 255) << 8);
  const int verbose = (cfgor.GetArgument("verbose_lba", IBA_VERBOSE_NONE) & 255) |
                     ((cfgor.GetArgument("verbose_gba", IBA_VERBOSE_NONE) & 255) << 8);
  const int debug = (cfgor.GetArgument("debug_lba", IBA_DEBUG_NONE) & 255) |
                   ((cfgor.GetArgument("debug_gba", IBA_DEBUG_NONE) & 255) << 8);
  const int history = (cfgor.GetArgument("history_lba", IBA_HISTORY_NONE) & 255) |
                     ((cfgor.GetArgument("history_gba", IBA_HISTORY_NONE) & 255) << 8);
  solver.Create(IS.m_K, serial, verbose, debug, history, param, IS.m_dir
#ifdef CFG_GROUND_TRUTH
              , IS.m_XsGT.empty() ? NULL : &IS.m_XsGT
              , IS.m_dsGT.empty() ? NULL : &IS.m_dsGT
#endif
              );
#ifndef WIN32
    if (cfgor.GetArgument("serial_lba", IBA_SERIAL_NONE) > 0) {
      LOG(INFO) << "LBA is working serial";
    } else {
      LOG(INFO) << "LBA is working parallel";
    }
    if (cfgor.GetArgument("serial_gba", IBA_SERIAL_NONE) > 0) {
      LOG(INFO) << "GBA is working serial";
    } else {
      LOG(INFO) << "GBA is working parallel";
    }
#endif
  solver.Start();
#ifdef IBA_WITH_CVD
  ViewerIBA viewer;
  const int visualize = cfgor.GetArgument("visualize", 1);
  viewer.Create(&solver, cfgor.GetArgument("screen_file"),
                         cfgor.GetArgument("screen_combine", 0),
                         cfgor.GetArgument("save_file"),
                         cfgor.GetArgument("save_frame", INT_MAX), visualize != 0);
  const int pause = cfgor.GetArgument("pause", 0);
  const int iFrmStart = viewer.Start(cfgor.GetArgument("view_file_load"), pause >= 2);
#else
  const int iFrmStart = -1;
#endif
#ifdef CFG_DEBUG_MT
  MT::Start(UT::FileNameReplaceDirectory(cfgor.GetArgument("multi_thread_file"), ".", IS.m_dir),
            (cfgor.GetArgument("multi_thread_save", 0) != 0 ? MT_FLAG_SAVE : MT_FLAG_DEFAULT) |
            (cfgor.GetArgument("multi_thread_load", 0) != 0 ? MT_FLAG_LOAD : MT_FLAG_DEFAULT));
#endif

  IBA::CurrentFrame CF;
  IBA::KeyFrame KF;
  IBA::RelativeConstraint Z;
  std::vector<int> iFrms;
  std::vector<IBA::CameraPose> Cs;
  IBA::SlidingWindow SW;
  std::vector<int> idxs;
#if 0
//#if 1
  std::vector<IBA::Point3D> Xs;
#endif
  // IBA::Error eLBA, eGBA;
  const int print = cfgor.GetArgument("print_camera", 0);
#ifdef CFG_GROUND_TRUTH
  Rotation3D eR;
  LA::AlignedVector3f ep, er;
  LA::AlignedMatrix6x6f S;
  float epl, erl;
  if (eps) {
    eps->Resize(0);
  }
  if (ers) {
    ers->Resize(0);
  }
  const AlignedVector<Camera> &CsGT = IS.m_CTGT.m_Cs;
  const float loopRatio = cfgor.GetArgument("loop_ratio", 0.0f);
  const bool zGT = !CsGT.Empty() && cfgor.GetArgument("loop_mean_gt", 0) != 0;
  const bool eGT = !CsGT.Empty() && cfgor.GetArgument("loop_variance_gt", 0) != 0;
  const float s2p = cfgor.GetArgumentSquared("loop_variance_position", 0.0f);
  const float s2r = cfgor.GetArgumentRadianSquared("loop_variance_rotation", 0.0f);
  const float epMax = cfgor.GetArgument("loop_max_error_position", 1000.0f);
  const float erMax = cfgor.GetArgumentRadian("loop_max_error_rotation", 1000.0f);
  const float updRatioFrq = cfgor.GetArgument("update_camera_ratio_frequency", 0.0f);
  const float updRatioNum = cfgor.GetArgument("update_camera_ratio_number", 0.0f);
  const float npMax = cfgor.GetArgument("update_camera_noise_position", 0.0f);
  const float nrMax = cfgor.GetArgumentRadian("update_camera_noise_rotation", 0.0f);
#endif
  const float dkfRatio = cfgor.GetArgument("delete_keyframe_ratio", 0.0f);
  const float dmpRatioFrq = cfgor.GetArgument("delete_map_point_ratio_frequency", 0.0f);
  const float dmpRatioNum = cfgor.GetArgument("delete_map_point_ratio_number", 0.0f);
  //UT::PrintStart(UT::FileNameAppendSuffix(UT::FileNameReplaceDirectory(
  //               cfgor.GetArgument("print_file"), ".", IS.m_dir)));
  UT::PrintStart(UT::FileNameReplaceDirectory(cfgor.GetArgument("print_file"),
                 ".", IS.m_dir));
  for (int iFrm = iFrmStart + 1, iFrmLast = -1; iFrm < nFrms; ++iFrm) {
    const double t1 = Timer::GetTime();
    if (!seeds.empty()) {
      srand(seeds[iFrm]);
    }
//#ifdef IBA_WITH_CVD
#if 0
    if (!viewer.Run(visualize >= 2)) {
      break;
    }
#endif
// #ifdef CFG_DEBUG
#if 0
    UT::Print("[%d] %d\n", iFrm, UT::Random<int>(RAND_MAX));
#endif
    IS.LoadCurrentFrame(iFrm, &CF, &KF, &solver);
#ifdef CFG_DEBUG
    UT_ASSERT(CF.iFrm == iFrm);
#endif
    solver.PushCurrentFrame(CF, KF.iFrm == -1 ? NULL : &KF);
    //UT::Print("[%d] %f\n", iFrm, Timer::GetTime() - t1);
// #ifdef CFG_DEBUG
#if 0
// #if 1
    const std::string fileName = UT::FileNameRemoveDirectory(IS.m_filesFtr[iFrm]);
    solver.SaveFeatures(IS.m_dir + "l_det/" + fileName + ".yml",
                        IS.m_dir + "r_det/" + fileName + ".yml", CF, KF);
#endif
    if (IS.LoadRelativeConstraint(iFrm, &Z)) {
#ifdef CFG_GROUND_TRUTH
      if ((zGT || eGT) && !CsGT.Empty()) {
        const Rigid3D TGT = CsGT[Z.iFrm2].m_T / CsGT[Z.iFrm1].m_T;
        const Rotation3D RGT = TGT;
        const Point3D pGT = TGT.GetPosition();
        if (zGT) {
          RGT.Get(Z.T.R);
          pGT.Get(Z.T.p);
        }
        if (eGT) {
          ep = pGT - Point3D(Z.T.p);
          er = (RGT / Rotation3D(Z.T.R)).GetRodrigues(BA_ANGLE_EPSILON);
          epl = sqrtf(ep.SquaredLength());
          erl = sqrtf(er.SquaredLength());
          if (epl < epMax && erl < erMax) {
            if (eps) {
              eps->Push(epl);
            }
            if (ers) {
              ers->Push(erl);
            }
          } else {
             //UT::Print("Removed [%d, %d] ep = %f, er = %f\n", Z.iFrm1, Z.iFrm2, epl, erl);
            Z.iFrm1 = Z.iFrm2 = -1;
          }
        }
      }
#endif
      S.MakeDiagonal(s2p, s2r);
      S.Get(Z.S.S);
      solver.PushRelativeConstraint(Z);
    }
    if (IS.LoadDeleteKeyFrames(iFrm, &iFrms)) {
      const int N = static_cast<int>(iFrms.size());
      for (int i = 0; i < N; ++i) {
        const int jFrm = iFrms[i];
        solver.DeleteKeyFrame(jFrm);
#ifdef IBA_WITH_CVD
        viewer.DeleteKeyFrame(jFrm);
#endif
      }
    }
    if (IS.LoadDeleteMapPoints(iFrm, &idxs)) {
      solver.DeleteMapPoints(idxs);
    }
    //UT::Print("[%d] %f\n", iFrm, Timer::GetTime() - t1);
    if (IS.LoadUpdateCameras(iFrm, &iFrms, &Cs)) {
      solver.UpdateCameras(iFrms, Cs);
    }
    const bool scc = solver.GetSlidingWindow(&SW);
    if (scc) {
      //IBA::PrintPoints(SW.Xs);
      if (print > 0) {
        std::vector<int>::const_iterator i;
        for (i = std::upper_bound(SW.iFrms.begin(), SW.iFrms.end(), iFrmLast);
             i != SW.iFrms.end(); ++i) {
          iFrmLast = *i;
          const int j = static_cast<int>(i - SW.iFrms.begin());
          IBA::PrintCameraPose(iFrmLast, SW.CsLF[j].C, print > 1);
        }
      }
#if 0
//#if 1
      const int nLFs = static_cast<int>(SW.iFrms.size());
      for (int i = 0; i < nLFs; ++i) {
        const std::pair<float, float> &e = SW.esLF[i];
        if (i == 0) {
          UT::Print("LF\n");
        }
        UT::Print("  [%d] %f %f\n", SW.iFrms[i], e.first, e.second);
      }
      const int nKFs = static_cast<int>(SW.iFrmsKF.size());
      for (int i = 0; i < nKFs; ++i) {
        const std::pair<float, float> &e = SW.esKF[i];
        if (i == 0) {
          UT::Print("KF\n");
        }
        UT::Print("  [%d] %f %f\n", SW.iFrmsKF[i], e.first, e.second);
      }
#endif
#if 0
//#if 1
      const int N = static_cast<int>(SW.Xs.size());
      for (int i = 0; i < N; ++i) {
        const IBA::Point3D &X = SW.Xs[i];
        const int _N = static_cast<int>(Xs.size());
        if (X.idx >= _N) {
          Xs.resize(X.idx + 1);
          for (int idx = _N; idx < X.idx; ++idx) {
            Xs[idx].idx = -1;
          }
        }
        Xs[X.idx] = X;
      }
      IBA::PrintPoints(Xs);
#endif
    }
#ifdef CFG_GROUND_TRUTH
    if (!CsGT.Empty()) {
      if (loopRatio > 0.0f && KF.iFrm != -1) {
        const int nKFs = solver.GetKeyFrames();
        if (nKFs >= 2 && (loopRatio < 0.0f || UT::Random<float>() < loopRatio)) {
          const int iKF2 = nKFs - 1;
          while (1) {
            //Z.iFrm2 = UT::Random<int>(iFrm + 1);
            Z.iFrm2 = iFrm;
            if (loopRatio < 0.0f) {
              Z.iFrm1 = solver.GetKeyFrameIndex(iKF2 - 1);
            } else {
              const int iKF1 = UT::Random<int>(iKF2);
              Z.iFrm1 = solver.GetKeyFrameIndex(iKF1);
            }
            const Rigid3D T = CsGT[Z.iFrm2].m_T / CsGT[Z.iFrm1].m_T;
            T.Rotation3D::Get(Z.T.R);
            T.GetPosition().Get(Z.T.p);
            S.MakeDiagonal(s2p, s2r);
            S.Get(Z.S.S);
            if (solver.PushRelativeConstraint(Z)) {
              break;
            }
          }
        }
      }
      if (updRatioFrq > 0.0f && UT::Random<float>() < updRatioFrq) {
        iFrms.resize(0);
        const int nKFs = solver.GetKeyFrames();
        for (int iKF = 0; iKF < nKFs; ++iKF) {
          if (UT::Random<float>() < updRatioNum) {
            iFrms.push_back(solver.GetKeyFrameIndex(iKF));
          }
        }
        const int N = static_cast<int>(iFrms.size());
        Cs.resize(N);
        for (int i = 0; i < N; ++i) {
          const int jFrm = iFrms[i];
          const Camera &CGT = CsGT[jFrm];
          eR.Random(nrMax);
          const Rotation3D R = Rotation3D(CGT.m_T) / eR;
          ep.Random(npMax);
          const LA::AlignedVector3f p = CGT.m_p - ep;
          R.Get(Cs[i].R);
          p.Get(Cs[i].p);
        }
        if (N > 0) {
          solver.UpdateCameras(iFrms, Cs);
        }
      }
    }
#endif
    if (dkfRatio > 0.0f && UT::Random<float>() < dkfRatio) {
      //const int iFrm0 = SW.iFrms.front();
      const int iFrm0 = iFrm - LBA_MAX_LOCAL_FRAMES + 1;
      const int nKFs = solver.SearchKeyFrame(iFrm0);
      if (nKFs > 1) {
        const int i = UT::Random<int>(nKFs), jFrm = solver.GetKeyFrameIndex(i);
        solver.DeleteKeyFrame(jFrm);
#ifdef IBA_WITH_CVD
        viewer.DeleteKeyFrame(jFrm);
#endif
      }
    }
    if (dmpRatioFrq > 0.0f && UT::Random<float>() < dmpRatioFrq) {
      solver.GetMapPointIndexes(&idxs);
      int i, j;
      const int N = static_cast<int>(idxs.size());
      for (i = j = 0; i < N; ++i) {
        if (UT::Random<float>() < dmpRatioNum) {
          idxs[j++] = idxs[i];
        }
      }
      idxs.resize(j);
      solver.DeleteMapPoints(idxs);
    }
    if (print == 0) {
      printf("\r%d / %d = %f%%", iFrm + 1, nFrms, UT::Percentage(iFrm + 1, nFrms));
    }
#ifdef IBA_WITH_CVD
    if (scc) {
      if (!viewer.Run(visualize >= 2, true, iFrm)) {
        break;
      }
    }
#endif
    const double t2 = Timer::GetTime(), dt = t2 - t1;

    if (dt < IS.m_dt) {
      const int ms = static_cast<int>((IS.m_dt - dt) * 1000.0 + 0.5);
#ifdef WIN32
      Sleep(ms);  // Windows Sleep (ms)
#else
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
    }
  }
  if (print == 0 || print == 1) {
    printf("\n");
  }
  solver.Stop();
#ifdef CFG_VERBOSE
  const float eLBA = solver.ComputeRMSELBA(), eGBA = solver.ComputeRMSEGBA();
  const float Sdist = solver.GetTotalDistance();
  UT::PrintSeparator();
  UT::Print("RMSE\n");
  UT::Print("  LBA = %f / %f = %f%%\n", eLBA, Sdist, UT::Percentage(eLBA, Sdist));
  UT::Print("  GBA = %f / %f = %f%%\n", eGBA, Sdist, UT::Percentage(eGBA, Sdist));
#else
  UT::Print("%.2f\t%.2f * %d\n", tLBA->t / tLBA->n, tGBA->t / tGBA->n, tGBA->n);
#endif
//#ifdef CFG_DEBUG
#if 0
//#if 1
  IBA::PrintPoints(Xs);
#endif
  const bool append = cfgor.GetArgument("output_append", 1) != 0;
  const bool poseOnly = cfgor.GetArgument("output_camera_pose_only", 1) != 0;
  solver.SaveCamerasLBA(cfgor.GetArgument("output_camera_file_lba"), append, poseOnly);
  solver.SaveCamerasGBA(cfgor.GetArgument("output_camera_file_gba"), append, poseOnly);
  solver.SaveCostsLBA(cfgor.GetArgument("output_cost_file_lba_a"), append, 0);
  solver.SaveCostsGBA(cfgor.GetArgument("output_cost_file_gba_a"), append, 0);
  solver.SaveCostsLBA(cfgor.GetArgument("output_cost_file_lba_b"), append, 1);
  solver.SaveCostsGBA(cfgor.GetArgument("output_cost_file_gba_b"), append, 1);
  solver.SaveCostsLBA(cfgor.GetArgument("output_cost_file_lba_p"), append, 2);
  solver.SaveCostsGBA(cfgor.GetArgument("output_cost_file_gba_p"), append, 2);
#ifdef CFG_GROUND_TRUTH
  solver.SaveCostsLBA(cfgor.GetArgument("output_cost_file_lba_a_gt"), append, 3);
  solver.SaveCostsGBA(cfgor.GetArgument("output_cost_file_gba_a_gt"), append, 3);
  solver.SaveCostsLBA(cfgor.GetArgument("output_cost_file_lba_p_gt"), append, 4);
  solver.SaveCostsGBA(cfgor.GetArgument("output_cost_file_gba_p_gt"), append, 4);
#endif
  solver.SaveResidualsLBA(cfgor.GetArgument("output_residual_file_lba"), append);
  solver.SaveResidualsGBA(cfgor.GetArgument("output_residual_file_gba"), append);
#ifdef CFG_GROUND_TRUTH
  solver.SaveResidualsLBA(cfgor.GetArgument("output_residual_file_lba_gt"), append, 1);
  solver.SaveResidualsGBA(cfgor.GetArgument("output_residual_file_gba_gt"), append, 1);
  solver.SavePriors(cfgor.GetArgument("output_prior_file"), append, 0);
  solver.SavePriors(cfgor.GetArgument("output_prior_file_kf"), append, 1);
  solver.SavePriors(cfgor.GetArgument("output_prior_file_lf"), append, 2);
#endif
  solver.SaveMarginalizations(cfgor.GetArgument("output_marg_file_lp"), append, 0);
  solver.SaveMarginalizations(cfgor.GetArgument("output_marg_file_em"), append, 1);
#ifdef CFG_GROUND_TRUTH
  solver.SaveMarginalizations(cfgor.GetArgument("output_marg_file_gt"), append, 2);
#endif
  solver.SavePointsLBA(cfgor.GetArgument("output_point_file_lba"), append);
  solver.SavePointsGBA(cfgor.GetArgument("output_point_file_gba"), append);
  if (tLBA) {
    solver.GetTimeLBA(tLBA);
  }
  if (tGBA) {
    solver.GetTimeGBA(tGBA);
  }
  solver.SaveTimesLBA(cfgor.GetArgument("output_time_file_lba"), append);
  solver.SaveTimesGBA(cfgor.GetArgument("output_time_file_gba"), append);
#if defined CFG_VERBOSE && defined CFG_GROUND_TRUTH
  if (eps && ers && !eps->Empty() && !ers->Empty()) {
    const float ep = eps->Mean(), s2p = eps->Variance(ep), epMax = eps->Maximal();
    const float er = ers->Mean(), s2r = ers->Variance(er), erMax = ers->Maximal();
    UT::PrintSeparator();
    UT::Print("Relative constraint\n");
    UT::Print("  ep = %f +- %f <= %f (%d)\n", ep, sqrtf(s2p), epMax, eps->Size());
    UT::Print("  er = %f +- %f <= %f (%d)\n", er * UT_FACTOR_RAD_TO_DEG,
                                              sqrtf(s2r) * UT_FACTOR_RAD_TO_DEG,
                                              erMax * UT_FACTOR_RAD_TO_DEG, ers->Size());
  }
#endif
  if (tLBA && tGBA) {
#ifdef CFG_VERBOSE
    UT::PrintSeparator();
    UT::Print("Time\n");
    UT::Print("  LBA = %f / %d = %f ms\n", tLBA->t, tLBA->n, tLBA->t / tLBA->n);
    UT::Print("  GBA = %f / %d = %f ms\n", tGBA->t, tGBA->n, tGBA->t / tGBA->n);
#else
    UT::Print("%.2f\t%.2f * %d\n", tLBA->t / tLBA->n, tGBA->t / tGBA->n, tGBA->n);
#endif
  }
  UT::PrintStop();
#ifdef IBA_WITH_CVD
  viewer.Stop(cfgor.GetArgument("view_file_save"), visualize == 1 || pause >= 1);
#endif
#ifdef CFG_DEBUG_MT
  MT::Stop();
#endif
  solver.Destroy();
  return true;
}

bool RunConfig(const std::string &fileName) {
  Configurator cfgor(fileName);
  IBA::Time tLBA, tGBA;
  const std::string dir = cfgor.GetArgument("input_directory_all");
  if (dir == "") {
    const InputSequence IS(cfgor);
    return RunSolver(cfgor, IS, fileName, &tLBA, &tGBA);
  } else {
    IBA::Time StLBA, StGBA;
    StLBA.t = StGBA.t = 0.0f;
    StLBA.n = StGBA.n = 0;
    // StLBA.nd = StGBA.nd = 0;
    std::vector<std::string> seqName;
#if 0
    const std::string dataName[2] = {"ASL-EuRoC", "PennCOSYVIO"};
    if (dir.find(dataName[0]) != std::string::npos) {
      seqName.push_back("MH_01_easy");
      seqName.push_back("MH_02_easy");
      seqName.push_back("MH_03_medium");
      seqName.push_back("MH_04_difficult");
      seqName.push_back("MH_05_difficult");
      seqName.push_back("V1_01_easy");
      seqName.push_back("V1_02_medium");
      seqName.push_back("V1_03_difficult");
      seqName.push_back("V2_01_easy");
      seqName.push_back("V2_02_medium");
      seqName.push_back("V2_03_difficult");
    } else if (dir.find(dataName[1]) != std::string::npos) {
      seqName.push_back("af");
      seqName.push_back("as");
      seqName.push_back("bf");
      seqName.push_back("bs");
    } else {
      return false;
    }
#endif
    const std::string listName = dir + cfgor.GetArgument("input_sequence_list", "list.txt");
    FILE *fp = fopen(listName.c_str(), "r");
    if (!fp) {
      return false;
    }
    char line[UT_STRING_WIDTH_MAX];
    while (fgets(line, UT_STRING_WIDTH_MAX, fp)) {
      if (line[0] == '#') {
        continue;
      }
      const int len = static_cast<int>(strlen(line));
      if (line[len - 1] == 10) {
        line[len - 1] = 0;
      }
      seqName.push_back(line);
    }
    fclose(fp);
    UT::PrintLoaded(listName);
#ifdef CFG_GROUND_TRUTH
    LA::AlignedVectorXf eps, ers, Eps, Ers;
#endif
    cfgor.SetArgument("save_file", "");
    cfgor.SetArgument("visualize", 0);
    cfgor.SetArgument("pause", 0);
    cfgor.SetArgument("verbose_lba", 0);
    cfgor.SetArgument("verbose_gba", 0);
    const int N = static_cast<int>(seqName.size());
    for (int i = 0; i < N; ++i) {
//#ifdef CFG_DEBUG
#if 0
//#if 1
      if (i == 0) {
        cfgor.SetArgument("save_file", "D:/tmp/ost.txt");
        cfgor.SetArgument("save_frame", 3681);
      } else {
        cfgor.SetArgument("save_file", "");
        //cfgor.SetArgument("visualize", 2);
        //cfgor.SetArgument("pause", 2);
        //cfgor.SetArgument("verbose_lba", 3);
      }
#endif
#ifdef CFG_VERBOSE
      UT::PrintSeparator('*');
      UT::Print("%s\n", seqName[i].c_str());
#else
      UT::Print("%s\t", seqName[i].c_str());
#endif
      cfgor.SetArgument("input_directory", dir + seqName[i] + "/");
// #ifdef CFG_GROUND_TRUTH
//    cfgor.SetArgument("input_ground_truth_time_first", tFirstGT[i]);
// #endif
      const InputSequence IS(cfgor);
#ifdef CFG_GROUND_TRUTH
      if (!RunSolver(cfgor, IS, fileName, &tLBA, &tGBA, &eps, &ers)) {
#else
      if (!RunSolver(cfgor, IS, fileName, &tLBA, &tGBA)) {
#endif
        return false;
      }
#ifdef CFG_GROUND_TRUTH
      if (!eps.Empty() && !ers.Empty()) {
        Eps.Push(eps);
        Ers.Push(ers);
      }
#endif
      StLBA.t += tLBA.t;  StLBA.n += tLBA.n;
      StGBA.t += tGBA.t;  StGBA.n += tGBA.n;
      // StLBA.nd += tLBA.nd;
      // StGBA.nd += tGBA.nd;
    }
#ifdef CFG_VERBOSE
    if (!Eps.Empty() && !Ers.Empty()) {
      UT::PrintSeparator('*');
      UT::Print("Relative constraint\n");
      const float Ep = Eps.Mean(), S2p = Eps.Variance(Ep), EpMax = Eps.Maximal();
      const float Er = Ers.Mean(), S2r = Ers.Variance(Er), ErMax = Ers.Maximal();
      UT::Print("  Ep = %f +- %f <= %f (%d)\n", Ep, sqrtf(S2p), EpMax, Eps.Size());
      UT::Print("  Er = %f +- %f <= %f (%d)\n", Er * UT_FACTOR_RAD_TO_DEG,
                                                sqrtf(S2r) * UT_FACTOR_RAD_TO_DEG,
                                                ErMax * UT_FACTOR_RAD_TO_DEG, Ers.Size());
    }
#endif
#ifdef CFG_VERBOSE
    UT::PrintSeparator('*');
    UT::Print("Time\n");
    UT::Print("  LBA = %f / %d = %f ms\n", StLBA.t, StLBA.n, StLBA.t / StLBA.n);
    UT::Print("  GBA = %f / %d = %f ms\n", StGBA.t, StGBA.n, StGBA.t / StGBA.n);
    // UT::Print("  LBA = %d / %d = %f points\n", StLBA.nd, StLBA.n, float(StLBA.nd) / StLBA.n);
    // UT::Print("  GBA = %d / %d = %f points\n", StGBA.nd, StGBA.n, float(StGBA.nd) / StGBA.n);
#else
    UT::Print("Total\t\t%.2f\t%.2f * %d\n", StLBA.t / StLBA.n, StGBA.t / StGBA.n, StGBA.n);
#endif
    return true;
  }
}

#ifdef WIN32
int _tmain(int argc, _TCHAR* argv[]) {
#else
int main(int argc, char** argv) {
#endif  // _WIND32
// #ifdef CFG_DEBUG
#ifndef WIN32
  google::InitGoogleLogging(argv[0]);
  // FLAGS_stderrthreshold = 0;
#endif
#ifndef WIN32
  google::ParseCommandLineFlags(&argc, &argv, false);
#endif
  // TODO(mingyu): Upgrade to use gflag once it's ready on Windows
  if (argc == 2) {
    // Playback according to the input config file
    const bool scc = RunConfig(std::string(argv[1]));
    if (scc) {
      return 0;
    } else {
      return 1;
    }
  } else {
    std::cout << " Incorrect input arguments\n";
    return 0;
  }
}
