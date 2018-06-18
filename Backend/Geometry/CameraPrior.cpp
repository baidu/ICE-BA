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
//#define CFG_DEBUG_EIGEN
//#endif
#include "CameraPrior.h"

namespace CameraPrior {

bool Pose::GetPriorMeasurement(const float w, LA::AlignedVectorXf *x,
                               LA::AlignedMatrixXf *S) const {
  const int N = static_cast<int>(m_iKFs.size());
  const bool r = m_Zps.Size() != N;
  const int Np = (r ? 2 : 0) + N * 6;
  S->Resize(Np, Np, false, false);
  x->Resize(Np);

  int ip = 0;
  if (r) {
    S->SetBlockDiagonal(ip, m_Arr);
    for (int i = 0, jp = 2; i < N; ++i, jp += 6) {
      S->SetBlock(ip, jp, m_Arc[i]);
    }
    x->SetBlock(ip, m_br);
    ip += 2;
  }
  for (int i = 0; i < N; ++i, ip += 6) {
    for (int j = i, jp = ip; j < N; ++j, jp += 6) {
      S->SetBlock(ip, jp, m_Acc[i][j]);
    }
    x->SetBlock(ip, m_bc[i]);
  }
//#ifdef CFG_DEBUG_EIGEN
#if 0
  const EigenMatrixXd e_A = EigenMatrixXf(*S).cast<double>();
  const EigenVectorXd e_b = EigenVectorXf(*x).cast<double>();
  //const EigenMatrixXd e_S = EigenMatrixXd(e_A.ldlt().solve(EigenMatrixXd::Identity(Np)));
  const EigenMatrixXd e_S = EigenMatrixXd(e_A.inverse());
  const EigenVectorXd e_x = -e_S * e_b;
  *S = e_S.GetAlignedMatrixXf();
  *x = e_x.GetAlignedVectorXf();
  return true;
#endif
  if (!S->SolveLDL(*x, NULL)) {
    return false;
  }
  x->MakeMinus();
  S->InverseLDL(NULL, true);
  *S *= w;
  return true;
}

bool Pose::GetPriorMeasurement(const float w, LA::AlignedVectorXf *x, LA::AlignedMatrixXf *S,
                               AlignedVector<float> *work) const {
  const int N = static_cast<int>(m_iKFs.size());
  const int Np = (m_Zps.Size() == N ? 0 : 2) + N * 6;
  work->Resize((S->BindSize(Np, Np, false) + x->BindSize(Np)) / sizeof(float));
  S->Bind(work->Data(), Np, Np, false);
  x->Bind(S->BindNext(), Np);
  return GetPriorMeasurement(w, x, S);
}

bool Joint::GetPriorMeasurement(const float w, LA::AlignedVectorXf *x,
                                LA::AlignedMatrixXf *S) const {
  const int N = static_cast<int>(m_iKFs.size());
  const bool r = m_Zps.Size() != N;
  const int Nprc = (r ? 2 : 0) + N * 6, Np = Nprc + 9;
  S->Resize(Np, Np, false, false);
  x->Resize(Np);

  int ip = 0;
  if (r) {
    S->SetBlockDiagonal(ip, m_Arr);
    for (int i = 0, jp = 2; i < N; ++i, jp += 6) {
      S->SetBlock(ip, jp, m_Arc[i]);
    }
    S->SetBlock(ip, Nprc, m_Arm);
    x->SetBlock(ip, m_br);
    ip += 2;
  }
  for (int i = 0; i < N; ++i, ip += 6) {
    for (int j = i, jp = ip; j < N; ++j, jp += 6) {
      S->SetBlock(ip, jp, m_Acc[i][j]);
    }
    S->SetBlock(ip, Nprc, m_Acm[i]);
    x->SetBlock(ip, m_bc[i]);
  }
  S->SetBlock(Nprc, Nprc, m_Amm);
  x->SetBlock(Nprc, m_bm);
//#ifdef CFG_DEBUG_EIGEN
#if 0
  const EigenMatrixXd e_A = EigenMatrixXf(*S).cast<double>();
  const EigenVectorXd e_b = EigenVectorXf(*x).cast<double>();
  //const EigenMatrixXd e_S = EigenMatrixXd(e_A.ldlt().solve(EigenMatrixXd::Identity(Np)));
  const EigenMatrixXd e_S = EigenMatrixXd(e_A.inverse());
  const EigenVectorXd e_x = -e_S * e_b;
  *S = e_S.GetAlignedMatrixXf();
  *x = e_x.GetAlignedVectorXf();
  return true;
#endif
  if (!S->SolveLDL(*x, NULL)) {
    return false;
  }
  x->MakeMinus();
  S->InverseLDL(NULL, true);
  *S *= w;
  return true;
}

bool Joint::GetPriorMeasurement(const float w, LA::AlignedVectorXf *x, LA::AlignedMatrixXf *S,
                                AlignedVector<float> *work) const {
  const int N = static_cast<int>(m_iKFs.size());
  const int Np = (m_Zps.Size() == N ? 0 : 2) + N * 6 + 9;
  work->Resize((S->BindSize(Np, Np, false) + x->BindSize(Np)) / sizeof(float));
  S->Bind(work->Data(), Np, Np, false);
  x->Bind(S->BindNext(), Np);
  return GetPriorMeasurement(w, x, S);
}

void Joint::PropagateLF(const Rigid3D &Tr, const Camera &C,
                        const IMU::Delta::Factor::Auxiliary::RelativeLF &A,
                        AlignedVector<float> *work, const float *eps) {
//#ifdef CFG_DEBUG
#if 0
  m_Arr.Print();
  m_Acc[0][0].Print();
#endif
  const int N = static_cast<int>(m_iKFs.size()), Nk = N - 1, Nx2 = N * 2;
#ifdef CFG_DEBUG
  UT_ASSERT(m_iKFs[Nk] == INT_MAX);
#endif
  SetPose(Tr, Nk, C.m_T);
  SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);

  m_Acc.InsertZero(N, work);
  m_Acm.InsertZero(N, Nx2 + 1, work);
  m_Arc.InsertZero(N, 2, work);
  m_bc.InsertZero(N, 1, work);

  Vector::CM Acm1(m_Acm.Data(), N, false);
  Vector::CM Acm2(Acm1.End(), N, false);
  Vector::CM Mcm(Acm2.End() + 1, N, false);
  work->Resize((sizeof(Block::MC) * 3 + sizeof(Block::MM) * 3 + sizeof(Block::RM) * 2 +
                sizeof(Block::CC) * N) / sizeof(float));
  Vector::MC Amc(work->Data(), 3, false);
  Vector::MM Amm(Amc.End(), 3, false);
  Vector::RM Arm(Amm.End(), 2, false);
  Vector::CC Acc(Arm.End(), N, false);

#ifdef CFG_IMU_FULL_COVARIANCE
  Block::CC &Ac1c1 = m_Acc[Nk][Nk];
  Ac1c1.Increase(A.m_A, A.m_A + 9);
  Block::CM &Ac1m1 = Acm1[Nk], &dAc1m1 = m_Acm[Nx2];
  dAc1m1.Set(A.m_A + 2, A.m_A + 11);
  Ac1m1 += dAc1m1;
  Block::CC &Ac1c2 = m_Acc[Nk][N];
  Ac1c2.Set(A.m_A + 5, A.m_A + 14);
  Block::CM &Ac1m2 = m_Acm[Nx2];
  Ac1m2.Set(A.m_A + 7, A.m_A + 16);
  Block::MC &Am2c1 = Amc[0];
  Ac1m2.GetTranspose(Am2c1);
  Block::C &bc1 = m_bc[Nk];
  bc1 += Block::C(A.m_b[0], A.m_b[1]);

  Block::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_A + 19, A.m_A + 26, A.m_A + 32, true);
  Block::MC &Am1c2 = Amc[1];
  Am1c2.Set(A.m_A + 22, A.m_A + 29, A.m_A + 35);
  Block::CM &Ac2m1 = m_Acm[Nx2];
  Am1c2.GetTranspose(Ac2m1);
  // TODO (haomin) : speedup
  Block::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_A + 24, A.m_A + 31, A.m_A + 37);
  Am2m1.Transpose();
  Block::M &bm1 = m_bm;
  bm1 += Block::M(A.m_b[2], A.m_b[3], A.m_b[4]);

  Block::CC &Ac2c2 = m_Acc[N][N];
  Ac2c2.Set(A.m_A + 40, A.m_A + 44);
  Block::CM &Ac2m2 = Acm2[Nk];
  Ac2m2.Set(A.m_A + 42, A.m_A + 46);
  Block::C &bc2 = m_bc[N];
  bc2.Set(A.m_b[5], A.m_b[6]);

  Block::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_A + 49, A.m_A + 51, A.m_A + 52);
  Block::M bm2(A.m_b[7], A.m_b[8], A.m_b[9]);

  m_Arr += A.m_Agg;
  Vector::RC Arck = m_Arc.GetBlock(Nk);
  Block::RC &Arc1 = m_Arc[Nk], &dArc1 = m_Arc[N];
  dArc1.Set(A.m_Agc[0], A.m_Agc[1]);
  Arc1 += dArc1;
  Block::RM &Arm1 = m_Arm, &dArm1 = Arm[0];
  dArm1.Set(A.m_Agc[2], A.m_Agc[3], A.m_Agc[4]);
  Arm1 += dArm1;
  Block::RC &Arc2 = m_Arc[N];
  Arc2.Set(A.m_Agc[5], A.m_Agc[6]);
  Block::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agc[7], A.m_Agc[8], A.m_Agc[9]);
  m_br += A.m_bg;
#else
  Block::CC &Ac1c1 = m_Acc[Nk][Nk];
  Ac1c1.Increase(A.m_Ap1p1, A.m_Ap1r1, A.m_Ar1r1);
  Block::CM &Ac1m1 = Acm1[Nk], &dAc1m1 = m_Acm[Nx2];
  dAc1m1.Set(A.m_Ap1v1, A.m_Ap1ba1, A.m_Ap1bw1, A.m_Ar1v1, A.m_Ar1ba1, A.m_Ar1bw1);
  Ac1m1 += dAc1m1;
  Block::CC &Ac1c2 = m_Acc[Nk][N];
  Ac1c2.Set(A.m_Ap1p2, A.m_Ap1r2, A.m_Ar1p2, A.m_Ar1r2);
  //Block::CM &Ac1m2 = m_Acm[Nx2];
  //Ac1m2.Set(A.m_Ar1v2);
  Block::MC &Am2c1 = Amc[0];
  Am2c1.Set(A.m_Ar1v2.GetTranspose());
  Block::C &bc1 = m_bc[Nk];
  bc1 += Block::C(A.m_bp1, A.m_br1);

  Block::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1, A.m_Abw1bw1);
  Block::MC &Am1c2 = Amc[1];
  Am1c2.Set(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2, A.m_Abw1r2);
  Block::CM &Ac2m1 = m_Acm[Nx2];
  Am1c2.GetTranspose(Ac2m1);
  // TODO (haomin) : speedup
  Block::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Av1v2, A.m_Aba1v2, A.m_Aba1ba2, A.m_Abw1v2, A.m_Abw1bw2);
  Am2m1.Transpose();
  Block::M &bm1 = m_bm;
  bm1 += Block::M(A.m_bv1, A.m_bba1, A.m_bbw1);

  Block::CC &Ac2c2 = m_Acc[N][N];
  Ac2c2.Set(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
  Block::CM &Ac2m2 = Acm2[Nk];
  Ac2m2.Set(A.m_Ar2v2);
  Block::C &bc2 = m_bc[N];
  bc2.Set(A.m_bp2, A.m_br2);

  Block::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Av2v2, A.m_Aba2ba2, A.m_Abw2bw2);
  Block::M bm2(A.m_bv2, A.m_bba2, A.m_bbw2);

  m_Arr += A.m_Agg;
  Vector::RC Arck = m_Arc.GetBlock(Nk);
  Block::RC &Arc1 = m_Arc[Nk], &dArc1 = m_Arc[N];
  dArc1.Set(A.m_Agp1, A.m_Agr1);
  Arc1 += dArc1;
  Block::RM &Arm1 = m_Arm, &dArm1 = Arm[0];
  dArm1.Set(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
  Arm1 += dArm1;
  Block::RC &Arc2 = m_Arc[N];
  Arc2.Set(A.m_Agp2, A.m_Agr2);
  Block::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agv2);
  m_br += A.m_bg;
#endif

  Matrix::CC Ackck = m_Acc.GetBlock(Nk, Nk);
  Matrix::CC Ackc1 = m_Acc.GetColumn(Nk, Nk);
  Vector::CM Ackm1 = Acm1.GetBlock(Nk);
  Matrix::CC Ackc2 = m_Acc.GetColumn(N, Nk);
  Vector::CM Ackm2 = Acm2.GetBlock(Nk);
  Vector::C bck = m_bc.GetBlock(Nk);

  Block::MM &Mm1m1 = Am1m1;
  if (Mm1m1.InverseLDL(eps ? eps + 6 : NULL)) {
    Mm1m1.MakeMinus();
    Block::RM &Mrm1 = Arm[1];
    Block::ABT(Arm1, Mm1m1, Mrm1);
    Block::AddABTToUpper(Mrm1, Arm1, m_Arr);
    Vector::AddABTTo(Mrm1, Ackm1, Arck);
    Block::AddABTTo(Mrm1, Ac1m1, Arc1);
    Block::AddABTTo(Mrm1, Ac2m1, Arc2);
    Block::AddABTTo(Mrm1, Am2m1, Arm2);         // TODO (haomin) : speedup
    Block::AddAbTo(Mrm1, bm1, m_br);
    Vector::CM Mckm1 = Mcm.GetBlock(Nk);
    Vector::ABT(Ackm1, Mm1m1, Mckm1);
    Matrix::AddABTToUpper(Mckm1, Ackm1, Ackck);
    Vector::AddABTTo(Mckm1, Ac1m1, Ackc1);
    Vector::AddABTTo(Mckm1, Ac2m1, Ackc2);
    Vector::AddABTTo(Mckm1, Am2m1, Ackm2);      // TODO (haomin) : speedup
    Vector::AddAbTo(Mckm1, bm1, bck);
    Block::CM &Mc1m1 = Mcm[Nk];
    Block::ABT(Ac1m1, Mm1m1, Mc1m1);
    Block::AddABTToUpper(Mc1m1, Ac1m1, Ac1c1);
    Block::AddABTTo(Mc1m1, Ac2m1, Ac1c2);
    //Block::AddABTTo(Mc1m1, Am2m1, Ac1m2);
    Block::AddABTTo(Am2m1, Mc1m1, Am2c1);       // TODO (haomin) : speedup
    Block::AddAbTo(Mc1m1, bm1, bc1);
    Block::CM &Mc2m1 = Mcm[Nk];
    Block::ABT(Ac2m1, Mm1m1, Mc2m1);
    Block::AddABTToUpper(Mc2m1, Ac2m1, Ac2c2);
    Block::AddABTTo(Mc2m1, Am2m1, Ac2m2);       // TODO (haomin) : speedup
    Block::AddAbTo(Mc2m1, bm1, bc2);
    Block::MM &Mm2m1 = Amm[2];
    Block::ABT(Am2m1, Mm1m1, Mm2m1);            // TODO (haomin) : speedup
    Block::AddABTToUpper(Mm2m1, Am2m1, Am2m2);  // TODO (haomin) : speedup
    Block::AddAbTo(Mm2m1, bm1, bm2);
  }

  Block::CC &Mc1c1 = Ac1c1;
  if (Mc1c1.InverseLDL(eps)) {
    Mc1c1.MakeMinus();
    Block::CC &Ac2c1 = Ac1c2;
    Ac2c1.Transpose();
    Block::RC &Mrc1 = m_Arc[N + 1];
    Block::ABT(Arc1, Mc1c1, Mrc1);
    Block::AddABTToUpper(Mrc1, Arc1, m_Arr);
    Vector::AddABTTo(Mrc1, Ackc1, Arck);
    Block::AddABTTo(Mrc1, Ac2c1, Arc2);
    Block::AddABTTo(Mrc1, Am2c1, Arm2);
    Block::AddAbTo(Mrc1, bc1, m_br);
    Vector::CC Mckc1 = Acc.GetBlock(Nk);
    Vector::ABT(Ackc1, Mc1c1, Mckc1);
    Matrix::AddABTToUpper(Mckc1, Ackc1, Ackck);
    Vector::AddABTTo(Mckc1, Ac2c1, Ackc2);
    Vector::AddABTTo(Mckc1, Am2c1, Ackm2);
    Vector::AddAbTo(Mckc1, bc1, bck);
    Block::CC &Mc2c1 = Acc[Nk];
    Block::ABT(Ac2c1, Mc1c1, Mc2c1);
    Block::AddABTToUpper(Mc2c1, Ac2c1, Ac2c2);
    Block::AddABTTo(Mc2c1, Am2c1, Ac2m2);
    Block::AddAbTo(Mc2c1, bc1, bc2);
    Block::MC &Mm2c1 = Amc[2];
    Block::ABT(Am2c1, Mc1c1, Mm2c1);
    Block::AddABTToUpper(Mm2c1, Am2c1, Am2m2);
    Block::AddAbTo(Mm2c1, bc1, bm2);
  }

  Arc1 = Arc2;
  Arm1 = Arm2;
  Ackck.SetLowerFromUpper();
  Ackc1.Copy(Ackc2);
  Ackm1.Copy(Ackm2);
  Ac1c1 = Ac2c2;
  Ac1c1.SetLowerFromUpper();
  Ac1m1 = Ac2m2;
  Am1m1 = Am2m2;
  Am1m1.SetLowerFromUpper();
  bc1 = bc2;
  bm1 = bm2;
  m_Acc.Resize(N, N, true, true);
  m_Acm.Resize(N);
  m_Arc.Resize(N);
  m_bc.Resize(N);
}

void Joint::PropagateKF(const Rigid3D &Tr, const Camera &C,
                        const IMU::Delta::Factor::Auxiliary::RelativeKF &A,
                        AlignedVector<float> *work, const float *eps) {
#ifdef CFG_DEBUG
  UT_ASSERT(m_iKFs.empty());
  //UT_ASSERT(m_Zps.Empty());
  UT_ASSERT(m_Zps.Size() == 1);
#endif
  m_iKFs.resize(1, INT_MAX);
  //m_Zps.Resize(1);
  const Rotation3D RrT = m_Zps.Back();
  m_Zps.Resize(2);
  m_Zps.Back() = RrT;
  SetPose(Tr, 0, C.m_T);
  SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);

  m_Acc.Resize(1, 1, false, true);
  m_Acm.Resize(3);
  m_Arc.Resize(1);
  m_bc.Resize(1);

  work->Resize((sizeof(Block::MC) + sizeof(Block::MM) * 3 + sizeof(Block::RM) * 2) /
                sizeof(float));
  Vector::MC Amc(work->Data(), 1, false);
  Vector::MM Amm(Amc.End(), 3, false);
  Vector::RM Arm(Amm.End(), 2, false);

#ifdef CFG_IMU_FULL_COVARIANCE
  Block::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Ac, A.m_Ac + 7, A.m_Ac + 13, true);
  Block::MC &Am1c2 = Amc[0];
  Am1c2.Set(A.m_Ac + 3, A.m_Ac + 10, A.m_Ac + 16);
  Block::CM &Ac2m1 = m_Acm[1];
  Am1c2.GetTranspose(Ac2m1);
  // TODO (haomin) : speedup
  Block::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Ac + 5, A.m_Ac + 12, A.m_Ac + 18);
  Am2m1.Transpose();
  Block::M &bm1 = m_bm;
  bm1 += Block::M(A.m_bc[0], A.m_bc[1], A.m_bc[2]);

  Block::CC &Ac2c2 = m_Acc[0][0];
  Ac2c2.Set(A.m_Ac + 21, A.m_Ac + 25);
  Block::CM &Ac2m2 = m_Acm[0];
  Ac2m2.Set(A.m_Ac + 23, A.m_Ac + 27);
  Block::C &bc2 = m_bc[0];
  bc2.Set(A.m_bc[3], A.m_bc[4]);

  Block::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Ac + 30, A.m_Ac + 32, A.m_Ac + 33);
  Block::M bm2(A.m_bc[5], A.m_bc[6], A.m_bc[7]);

  //m_Arr = A.m_Agg;
  m_Arr += A.m_Agg;
  Block::RM &Arm1 = m_Arm;
  Arm1.Set(A.m_Agc[0], A.m_Agc[1], A.m_Agc[2]);
  Block::RC &Arc2 = m_Arc[0];
  Arc2.Set(A.m_Agc[3], A.m_Agc[4]);
  Block::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agc[5], A.m_Agc[6], A.m_Agc[7]);
  //m_br = A.m_bg;
  m_br += A.m_bg;
#else
  Block::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1, A.m_Abw1bw1);
  Block::MC &Am1c2 = Amc[0];
  Am1c2.Set(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2, A.m_Abw1r2);
  Block::CM &Ac2m1 = m_Acm[1];
  Am1c2.GetTranspose(Ac2m1);
  // TODO (haomin) : speedup
  Block::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Av1v2, A.m_Aba1v2, A.m_Aba1ba2, A.m_Abw1v2, A.m_Abw1bw2);
  Am2m1.Transpose();
  Block::M &bm1 = m_bm;
  bm1 += Block::M(A.m_bv1, A.m_bba1, A.m_bbw1);

  Block::CC &Ac2c2 = m_Acc[0][0];
  Ac2c2.Set(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
  Block::CM &Ac2m2 = m_Acm[0];
  Ac2m2.Set(A.m_Ar2v2);
  Block::C &bc2 = m_bc[0];
  bc2.Set(A.m_bp2, A.m_br2);

  Block::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Av2v2, A.m_Aba2ba2, A.m_Abw2bw2);
  Block::M bm2(A.m_bv2, A.m_bba2, A.m_bbw2);

  //m_Arr = A.m_Agg;
  m_Arr += A.m_Agg;
  Block::RM &Arm1 = m_Arm;
  Arm1.Set(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
  Block::RC &Arc2 = m_Arc[0];
  Arc2.Set(A.m_Agp2, A.m_Agr2);
  Block::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agv2);
  //m_br = A.m_bg;
  m_br += A.m_bg;
#endif

  Block::MM &Mm1m1 = Am1m1;
  if (Mm1m1.InverseLDL(eps ? eps + 6 : NULL)) {
    Mm1m1.MakeMinus();
    Block::RM &Mrm1 = Arm[1];
    Block::ABT(Arm1, Mm1m1, Mrm1);
    Block::AddABTToUpper(Mrm1, Arm1, m_Arr);
    Block::AddABTTo(Mrm1, Ac2m1, Arc2);
    Block::AddABTTo(Mrm1, Am2m1, Arm2);         // TODO (haomin) : speedup
    Block::AddAbTo(Mrm1, bm1, m_br);
    Block::CM &Mc2m1 = m_Acm[2];
    Block::ABT(Ac2m1, Mm1m1, Mc2m1);
    Block::AddABTToUpper(Mc2m1, Ac2m1, Ac2c2);
    Block::AddABTTo(Mc2m1, Am2m1, Ac2m2);       // TODO (haomin) : speedup
    Block::AddAbTo(Mc2m1, bm1, bc2);
    Block::MM &Mm2m1 = Amm[2];
    Block::ABT(Am2m1, Mm1m1, Mm2m1);            // TODO (haomin) : speedup
    Block::AddABTToUpper(Mm2m1, Am2m1, Am2m2);  // TODO (haomin) : speedup
    Block::AddAbTo(Mm2m1, bm1, bm2);
  }

  Arm1 = Arm2;
  Ac2c2.SetLowerFromUpper();
  Am1m1 = Am2m2;
  Am1m1.SetLowerFromUpper();
  bm1 = bm2;
  m_Acm.Resize(1);
}

bool Joint::GetPriorPose(const int iKF, Pose *Zp, AlignedVector<float> *work, const float *eps) const {
  *Zp = *this;
  Zp->m_iKFs.back() = iKF;
  Block::MM Mmm;
  Block::RM Mrm;
  Vector::CM Mcm;
  const int N = m_Acm.Size();
  work->Resize(Mcm.BindSize(N) / sizeof(float));
  Mcm.Bind(work->Data(), N);
  Mmm = m_Amm;
  bool scc = true;
  if (Mmm.InverseLDL(eps ? eps + 6 : NULL)) {
    Mmm.MakeMinus();
    Block::ABT(m_Arm, Mmm, Mrm);
    Block::AddABTToUpper(Mrm, m_Arm, Zp->m_Arr);
    Vector::AddABTTo(Mrm, m_Acm, Zp->m_Arc);
    Block::AddAbTo(Mrm, m_bm, Zp->m_br);
    Vector::ABT(m_Acm, Mmm, Mcm);
    Matrix::AddABTToUpper(Mcm, m_Acm, Zp->m_Acc);
    Vector::AddAbTo(Mcm, m_bm, Zp->m_bc);
  } else {
    scc = false;
  }
  if (iKF == INT_MAX) {
    const int N = static_cast<int>(Zp->m_iKFs.size()), Nk = N - 1;
#ifdef CFG_DEBUG
    UT_ASSERT(N >= 1 && Zp->m_iKFs.back() == INT_MAX);
#endif
    Block::CC &Mcici = Zp->m_Acc[Nk][Nk];
    if (Mcici.InverseLDL(eps)) {
      Mcici.MakeMinus();
      Zp->m_Arc.Resize(N + 1, true);
      const Block::RC &Arci = Zp->m_Arc[Nk];
      Block::RC &Mrci = Zp->m_Arc[N];
      Block::ABT(Arci, Mcici, Mrci);
      Block::AddABTToUpper(Mrci, Arci, Zp->m_Arr);
      const Matrix::CC Ackci = Zp->m_Acc.GetColumn(Nk, Nk);
      Vector::RC Arck = Zp->m_Arc.GetBlock(Nk);
      Vector::AddABTTo(Mrci, Ackci, Arck);
      const Block::C &bci = Zp->m_bc[Nk];
      Block::AddAbTo(Mrci, bci, Zp->m_br);
      work->Resize(sizeof(Block::CC) * Nk / sizeof(float));
      Vector::CC Mckci(work->Data(), Nk, false);
      Vector::ABT(Ackci, Mcici, Mckci);
      Matrix::CC Ackck = Zp->m_Acc.GetBlock(Nk, Nk);
      Matrix::AddABTToUpper(Mckci, Ackci, Ackck);
      Vector::C bck = Zp->m_bc.GetBlock(Nk);
      Vector::AddAbTo(Mckci, bci, bck);
    } else {
      scc = false;
    }
    Zp->m_iKFs.resize(Nk);
    //Zp->m_Zps.Resize(Nk);
    const Rotation3D RrT = Zp->m_Zps.Back();
    Zp->m_Zps.Resize(Nk + 1);
    Zp->m_Zps.Back() = RrT;
    Zp->m_Arc.Resize(Nk);
    Zp->m_Acc.Resize(Nk, Nk, true, true);
    Zp->m_bc.Resize(Nk);
  }
  Zp->m_Acc.SetLowerFromUpper();
//#ifdef CFG_DEBUG_EIGEN
#if 0
  EigenPrior e_Ap1;
  Pose::EigenPrior e_Ap2;
  e_Ap1.Set(*this);
  e_Ap1.GetPriorPose(iKF, &e_Ap2);
  e_Ap2.m_A.SetLowerFromUpper();
  e_Ap2.Get(Zp->m_Arr, Zp->m_Arc, Zp->m_Acc, Zp->m_br, Zp->m_bc);
#endif
  return scc;
}

bool Joint::GetPriorMotion(Motion *Zp, AlignedVector<float> *work, const float *eps) const {
  *Zp = *this;
  Vector::RC Arc;
  Matrix::CC Acc;
  Vector::CM Acm;
  Vector::C bc;
  const int N = static_cast<int>(m_iKFs.size());
  work->Resize((Arc.BindSize(N) + Acc.BindSize(N, N, true) + Acm.BindSize(N) + bc.BindSize(N)) /
               sizeof(float));
  Arc.Bind(work->Data(), N);            Arc.Copy(m_Arc);
  Acc.Bind(Arc.BindNext(), N, N, true); Acc.Copy(m_Acc);
  Acm.Bind(Acc.BindNext(), N);          Acm.Copy(m_Acm);
  bc.Bind(Acm.BindNext(), N);           bc.Copy(m_bc);

  Block::RR Mrr;
  Block::RC Mrc;
  Block::RM Mrm;
  bool scc = true;
  //if (m_Arr.GetInverseLDL(Mrr, eps ? eps + 3 : NULL)) {
  if (m_Arr.GetInverse(Mrr)) {
    Mrr.MakeMinus();
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging()) {
      //UT::PrintSeparator();
      //Acc[0][0].SetLowerFromUpper();
      //Acc[0][0].Print();
      Zp->m_bm.Print();
    }
#endif
    for (int i = 0; i < N; ++i) {
      LA::AlignedMatrix2x6f::ATB(Mrr, m_Arc[i], Mrc);
      Block::CC *Ai = Acc[i];
      LA::AlignedMatrix6x6f::AddATBToUpper(Mrc, m_Arc[i], Ai[i]);
      //Ai[i].SetLowerFromUpper();
      for (int j = i + 1; j < N; ++j) {
        LA::AlignedMatrix6x6f::AddATBTo(Mrc, m_Arc[j], Ai[j]);
      }
      LA::AlignedMatrix6x9f::AddATBTo(Mrc, m_Arm, Acm[i]);
      LA::AlignedMatrix2x6f::AddATbTo(Mrc, m_br, bc[i]);
    }
    LA::AlignedMatrix2x9f::ATB(Mrr, m_Arm, Mrm);
    LA::AlignedMatrix9x9f::AddATBToUpper(Mrm, m_Arm, Zp->m_Amm);
    LA::AlignedMatrix2x9f::AddATbTo(Mrm, m_br, Zp->m_bm);
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging()) {
      //UT::PrintSeparator();
      //Acc[0][0].SetLowerFromUpper();
      //Acc[0][0].Print();
      Zp->m_bm.Print();
    }
#endif
  } else {
    scc = false;
  }

  Block::CC Mij;
  Block::CM Mim;
  for (int i = 0; i < N; ++i) {
    Block::CC *Ai = Acc[i], &Mii = Ai[i];
//#ifdef CFG_DEBUG
#if 0
    //if (UT::Debugging()/* && i == 7*/)
    {
      UT::PrintSeparator();
      UT::Print("%d\n", i);
      Mii.SetLowerFromUpper();
      Mii.Print();
    }
#endif
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging() && i == 9) {
      Block::CC &Aii = Ai[i];
      Aii.SetLowerFromUpper();
      UT::PrintSeparator();
      Aii.Print();
      const Block::CC _Mii = Aii.GetInverseLDL(-1);
      Block::CC I;
      LA::AlignedMatrix6x6f::ABT(Aii, _Mii, I);
      UT::PrintSeparator();
      I.Print();
    }
#endif
    if (Mii.InverseLDL(eps)) {
      Mii.MakeMinus();
      const Block::CM &Aim = Acm[i];
      const Block::C &bi = bc[i];
      for (int j = i + 1; j < N; ++j) {
        LA::AlignedMatrix6x6f::ATB(Mii, Ai[j], Mij);
        LA::AlignedMatrix6x6f *Aj = Acc[j];
        LA::AlignedMatrix6x6f::AddATBToUpper(Mij, Ai[j], Aj[j]);
//#ifdef CFG_DEBUG
#if 0
        if (UT::Debugging() && i == 9 && j == 10) {
          UT::PrintSeparator();
          Mii.Print();
          UT::PrintSeparator();
          Ai[j].Print();
          UT::PrintSeparator();
          Mij.Print();
        }
#endif
//#ifdef CFG_DEBUG
#if 0
        if (UT::Debugging() && j == 10) {
          UT::PrintSeparator();
          UT::Print("%d\n", i);
          Aj[j].SetLowerFromUpper();
          Aj[j].Print();
        }
#endif
        for (int k = j + 1; k < N; ++k) {
          LA::AlignedMatrix6x6f::AddATBTo(Mij, Ai[k], Aj[k]);
        }
        LA::AlignedMatrix6x9f::AddATBTo(Mij, Aim, Acm[j]);
        LA::AlignedMatrix6x6f::AddATbTo(Mij, bi, bc[j]);
      }
      LA::AlignedMatrix6x9f::ATB(Mii, Aim, Mim);
      LA::AlignedMatrix9x9f::AddATBToUpper(Mim, Aim, Zp->m_Amm);
      LA::AlignedMatrix9x9f::AddATbTo(Mim, bi, Zp->m_bm);
//#ifdef CFG_DEBUG
#if 0
      if (UT::Debugging()) {
        Zp->m_bm.Print();
        if (i == 7) {
          UT::PrintSeparator();
          Mii.Print();
          UT::PrintSeparator();
          Mim.Print();
          UT::PrintSeparator();
          bi.Print();
        }
      }
#endif
    } else {
      scc = false;
    }
  }
  Zp->m_Amm.SetLowerFromUpper();
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  Block::MM Amm = Zp->m_Amm;
  Block::M bm = Zp->m_bm;
  if (Amm.SolveLDL(bm, eps ? eps + 6 : NULL)) {
    Zp->m_em.Set(bm);
  } else {
    Zp->m_Amm.MakeZero();
    Zp->m_em.MakeZero();
  }
#endif
//#ifdef CFG_DEBUG_EIGEN
#if 0
  EigenPrior e_Ap1;
  Motion::EigenPrior e_Ap2;
  e_Ap1.Set(*this);
  //e_Ap1.m_A.PrintDiagonal(true);
  e_Ap1.GetPriorMotion(&e_Ap2);
  e_Ap2.m_A.SetLowerFromUpper();
  LA::Convert(e_Ap2.m_A.GetAlignedMatrixXf(), Zp->m_Amm);
  LA::Convert<9>(e_Ap2.m_b.GetAlignedVectorXf(), Zp->m_bm);
#endif
  return scc;
}

#ifdef CFG_DEBUG_EIGEN
//#define CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
Pose::EigenErrorJacobian Pose::EigenGetErrorJacobian(const AlignedVector<Rigid3D> &Cs) const {
  EigenErrorJacobian e_Je;
  const int N = static_cast<int>(m_iKFs.size()), Nx6 = N * 6;
  e_Je.m_e.Resize(2 + Nx6);
  e_Je.m_e.MakeZero();
  e_Je.m_J.Resize(2 + Nx6, 6 + Nx6);
  e_Je.m_J.MakeZero();

  const Rigid3D &T1 = Cs[m_iKFr];
  const EigenRotation3D e_R1 = T1;
  //const bool r = !m_Arc.Empty();
  const bool r = m_Zps.Size() > N;
  if (r) {
    //const Rotation3D &RrT = m_RrT;
    const Rotation3D &RrT = m_Zps.Back();
    const EigenRotation3D e_RrT = RrT;
    //const EigenRotation3D e_eR = EigenRotation3D(e_RrT * e_R1);
    const EigenRotation3D e_eR = EigenRotation3D(RrT * T1);
    const EigenVector3f e_er1 = e_eR.GetRodrigues();
    const EigenMatrix3x3f e_Jr1 = EigenMatrix3x3f(EigenRotation3D::GetRodriguesJacobianInverse(
                                                  e_er1) * e_eR);
    e_Je.m_e.block<2, 1>(0, 0) = e_er1.block<2, 1>(0, 0);
    e_Je.m_J.block<2, 3>(0, 3) = e_Jr1.block<2, 3>(0, 0);
#ifdef CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
    const float e_dr1Max = 1.0f;
    const EigenVector3f e_dr1 = EigenVector3f::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD);
    const EigenRotation3D e_R1GT = EigenMatrix3x3f(e_R1 * EigenRotation3D(e_dr1));
    const EigenVector3f e_er1GT = EigenRotation3D(e_RrT * e_R1GT).GetRodrigues();
    const EigenVector3f e_er11 = EigenVector3f(e_er1 - e_er1GT);
    const EigenVector3f e_er12 = EigenVector3f(e_er11 + e_Jr1 * e_dr1);
    //UT::AssertReduction(e_er11, e_er12);
    UT::AssertReduction(EigenVector2f(e_er11.block<2, 1>(0, 0)),
                        EigenVector2f(e_er12.block<2, 1>(0, 0)));
#endif
  }

  EigenVector6f e_ec;
  EigenMatrix6x6f e_Jc1, e_Jc2;
  e_Jc1.setZero();
  e_Jc2.setZero();
  const EigenPoint3D e_p1 = EigenPoint3D(T1.GetPosition());
  for (int i = 0, j = 2, k = 6; i < N; ++i, j += 6, k += 6) {
    const int iKF = m_iKFs[i];
    const Rigid3D &T2 = iKF == INT_MAX ? Cs.Back() : Cs[iKF], &T21 = m_Zps[i];
    const EigenRotation3D e_R21 = T21;
    const EigenPoint3D e_p12 = EigenPoint3D(T21.GetTranslation());
    const EigenRotation3D e_R2 = T2;
    const EigenPoint3D e_p2 = EigenPoint3D(T2.GetPosition());
    const EigenVector3f e_ep = EigenVector3f(e_R1 * EigenVector3f(e_p2 - e_p1) - e_p12);
    //const EigenVector3f e_er = EigenRotation3D(e_R21 * e_R2 * e_R1.GetTranspose()).GetRodrigues();
    //const EigenVector3f e_er = (Rotation3D(T21) / (Rotation3D(T1) / T2)).GetRodrigues();
    const EigenVector3f e_er = ((Rotation3D(T21) * Rotation3D(T2)) / Rotation3D(T1)).GetRodrigues();
    e_ec.block<3, 1>(0, 0) = e_ep;
    e_ec.block<3, 1>(3, 0) = e_er;
    e_Je.m_e.block<6, 1>(j, 0) = e_ec;

    const EigenMatrix3x3f e_Jpp1 = EigenMatrix3x3f(-e_R1);
    const EigenMatrix3x3f e_Jpp2 = EigenMatrix3x3f(e_R1);
    const EigenMatrix3x3f e_Jpr1 = EigenMatrix3x3f(e_R1 * EigenSkewSymmetricMatrix(
                                                   EigenVector3f(e_p2 - e_p1)));
    const EigenMatrix3x3f e_Jrr2 = EigenMatrix3x3f(EigenRotation3D::GetRodriguesJacobianInverse(e_er) *
                                                   (e_R21 * e_R2));
    const EigenMatrix3x3f e_Jrr1 = EigenMatrix3x3f(-e_Jrr2);
    e_Jc1.block<3, 3>(0, 0) = e_Jpp1;
    e_Jc1.block<3, 3>(0, 3) = e_Jpr1;
    e_Jc2.block<3, 3>(0, 0) = e_Jpp2;
    e_Jc1.block<3, 3>(3, 3) = e_Jrr1;
    e_Jc2.block<3, 3>(3, 3) = e_Jrr2;
    e_Je.m_J.block<6, 6>(j, 0) = e_Jc1;
    e_Je.m_J.block<6, 6>(j, k) = e_Jc2;
#ifdef CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
    //const float e_dp1Max = 0.0f;
    const float e_dp1Max = 0.1f;
    //const float e_dp2Max = 0.0f;
    const float e_dp2Max = 0.1f;
    //const float e_dr1Max = 0.0f;
    const float e_dr1Max = 0.1f;
    //const float e_dr2Max = 0.0f;
    const float e_dr2Max = 0.1f;
    const EigenVector3f e_dp1 = EigenVector3f::GetRandom(e_dp1Max);
    const EigenVector3f e_dp2 = EigenVector3f::GetRandom(e_dp2Max);
    const EigenVector3f e_dr1 = EigenVector3f::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD);
    const EigenVector3f e_dr2 = EigenVector3f::GetRandom(e_dr2Max * UT_FACTOR_DEG_TO_RAD);
    const EigenPoint3D e_p1GT = EigenVector3f(e_p1 + e_dp1);
    const EigenPoint3D e_p2GT = EigenVector3f(e_p2 + e_dp2);
    const EigenRotation3D e_R1GT = EigenRotation3D(e_R1 * EigenRotation3D(e_dr1));
    const EigenRotation3D e_R2GT = EigenRotation3D(e_R2 * EigenRotation3D(e_dr2));
    const EigenVector3f e_epGT = EigenVector3f(e_R1GT * EigenVector3f(e_p2GT - e_p1GT) - e_p12);
    const EigenVector3f e_erGT = EigenRotation3D(e_R21 * e_R2GT *
                                                 e_R1GT.GetTranspose()).GetRodrigues();
    const EigenVector3f e_ep1 = EigenVector3f(e_ep - e_epGT);
    const EigenVector3f e_ep2 = EigenVector3f(e_ep1 + e_Jpr1 * e_dr1 + e_Jpp1 * e_dp1 +
                                              e_Jpp2 * e_dp2);
    const EigenVector3f e_er1 = EigenVector3f(e_er - e_erGT);
    const EigenVector3f e_er2 = EigenVector3f(e_er1 + e_Jrr1 * e_dr1 + e_Jrr2 * e_dr2);
    UT::AssertReduction(e_er1, e_er2, 1, UT::String("er[%d --> %d]", m_iKFr, iKF));
    UT::AssertReduction(e_ep1, e_ep2, 1, UT::String("ep[%d --> %d]", m_iKFr, iKF));
#endif
  }
#if 1
  ErrorJacobian Je;
  GetErrorJacobian(Cs, &Je);
  e_Je.AssertEqual(Je);
  e_Je.Set(Je);
  if (!r) {
    e_Je.m_e.block(0, 0, 2, 1).setZero();
    e_Je.m_J.block(0, 0, 2, 6 + Nx6).setZero();
  }
#endif
  return e_Je;
}

Pose::EigenFactor Pose::EigenGetFactor(const float w, const AlignedVector<Rigid3D> &Cs) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs);
  const EigenPrior e_Ap = EigenPrior(m_Arr, m_Arc, m_Acc, m_br, m_bc, w);
  const EigenMatrixXf e_JT = EigenMatrixXf(e_Je.m_J.transpose());
  const EigenVectorXf e_Ae = EigenVectorXf(e_Ap.m_A * e_Je.m_e);
  EigenFactor e_A;
  //e_A.m_F = e_Je.m_e.dot(e_Ae * 0.5f + e_Ap.m_b);
  e_A.m_F = e_Je.m_e.dot(e_Ae + e_Ap.m_b * 2.0f);
  e_A.m_b = EigenVectorXf(e_JT * (e_Ap.m_b + e_Ae));
  e_A.m_A = EigenMatrixXf(e_JT * e_Ap.m_A * e_Je.m_J);
  return e_A;
}

float Pose::EigenGetCost(const float w, const AlignedVector<Rigid3D> &Cs,
                         const std::vector<EigenVector6f> &e_xs) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs);
  EigenVectorXf e_x;
  const int N = static_cast<int>(m_iKFs.size());
  e_x.Resize(6 + N * 6);
  e_x.block<6, 1>(0, 0) = e_xs[m_iKFr];
  for (int i = 0, j = 6; i < N; ++i, j += 6) {
    e_x.block<6, 1>(j, 0) = e_xs[m_iKFs[i]];
  }
  const EigenVectorXf e_e = EigenVectorXf(e_Je.m_e + e_Je.m_J * e_x);
  const EigenPrior e_Ap = EigenPrior(m_Arr, m_Arc, m_Acc, m_br, m_bc, w);
  //return e_e.dot(e_Ap.m_A * e_e * 0.5f + e_Ap.m_b);
  return e_e.dot(e_Ap.m_A * e_e + e_Ap.m_b * 2.0f);
}

void Pose::EigenGetResidual(const AlignedVector<Rigid3D> &Cs, EigenVectorXf *e_r) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs);
  const EigenPrior e_Ap = EigenPrior(m_Arr, m_Arc, m_Acc, m_br, m_bc);
  *e_r = EigenVectorXf(e_Ap.m_A * e_Je.m_e + e_Ap.m_b);
}

Motion::EigenErrorJacobian Motion::EigenGetErrorJacobian(const Camera &C) const {
  const EigenRotation3D e_R = EigenRotation3D(C.m_T);
  const EigenVector3f e_v = EigenVector3f(C.m_v);
  const EigenMatrix3x3f e_Jvr = EigenMatrix3x3f(e_R * EigenSkewSymmetricMatrix(e_v));
  const EigenVector3f e_zv = EigenVector3f(m_v);
  const EigenVector3f e_ev = EigenVector3f(e_R * e_v - e_zv);
  const EigenMatrix3x3f e_Jvv = e_R;
  EigenErrorJacobian e_Je;
  e_Je.m_J.setZero();
  e_Je.m_J.block<3, 3>(0, 0) = e_Jvr;
  e_Je.m_J.block<3, 3>(0, 3) = e_Jvv;
  e_Je.m_J.block<3, 3>(3, 6) = EigenMatrix3x3f::Identity();
  e_Je.m_J.block<3, 3>(6, 9) = EigenMatrix3x3f::Identity();
  //e_Je.m_J.Print();
  e_Je.m_e.block<3, 1>(0, 0) = e_ev;
  e_Je.m_e.block<3, 1>(3, 0) = EigenVector3f(C.m_ba - m_ba);
  e_Je.m_e.block<3, 1>(6, 0) = EigenVector3f(C.m_bw - m_bw);
#ifdef CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
  const float e_drMax = 0.1f;
  const float e_dvMax = 1.0f;
  const EigenVector3f e_dr = EigenVector3f::GetRandom(e_drMax * UT_FACTOR_DEG_TO_RAD);
  const EigenVector3f e_dv = EigenVector3f::GetRandom(e_dvMax);
  const EigenRotation3D e_RGT = EigenRotation3D(e_R * EigenRotation3D(e_dr));
  const EigenPoint3D e_vGT = EigenVector3f(e_v + e_dv);
  const EigenVector3f e_evGT = EigenVector3f(e_RGT * e_vGT - e_zv);
  const EigenVector3f e_ev1 = EigenVector3f(e_ev - e_evGT);
  const EigenVector3f e_ev2 = EigenVector3f(e_ev1 + e_Jvr * e_dr + e_Jvv * e_dv);
  UT::AssertReduction(e_ev1, e_ev2);
#endif
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  const EigenVector9f e_em = EigenVector9f(EigenMatrix9x9f(m_Amm).inverse() * EigenVector9f(m_bm));
  //const EigenVector9f e_em = EigenVector9f(m_em.m_ev, m_em.m_eba, m_em.m_ebw);
  e_Je.m_e += e_em;
#endif
#if 1
  ErrorJacobian Je;
  GetErrorJacobian(C, &Je);
  e_Je.AssertEqual(Je);
  e_Je.Set(Je);
#endif
  return e_Je;
}

Motion::EigenFactor Motion::EigenGetFactor(const float w, const Camera &C) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C);
  const EigenMatrix9x9f e_Amm = EigenMatrix9x9f(EigenMatrix9x9f(m_Amm) * w);
  const EigenVector9f e_bm = EigenVector9f(EigenVector9f(m_bm) * w);
  const Eigen::Matrix<float, 12, 9> e_JT = e_Je.m_J.transpose();
  const EigenVector9f e_Ae = EigenVector9f(e_Amm * e_Je.m_e);
  EigenFactor e_A;
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  const Eigen::Matrix<float, 12, 9> e_JTA = e_JT * e_Amm;
  e_A.m_A = EigenMatrix12x12f(e_JTA * e_Je.m_J);
  e_A.m_b = EigenVector12f(e_JTA * e_Je.m_e);
  e_A.m_F = e_Je.m_e.dot(e_Amm * e_Je.m_e);
#else
  //e_A.m_F = e_Je.m_e.dot(e_Ae * 0.5f + e_bm);
  e_A.m_F = e_Je.m_e.dot(e_Ae + e_bm * 2.0f);
  e_A.m_b = EigenVector12f(e_JT * (e_bm + e_Ae));
  e_A.m_A = EigenMatrix12x12f(e_JT * e_Amm * e_Je.m_J);
#endif
  return e_A;
}

float Motion::EigenGetCost(const float w, const Camera &C, const EigenVector3f &e_xr,
                           const EigenVector9f &e_xm) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C);
  const EigenMatrix9x9f e_Amm = EigenMatrix9x9f(EigenMatrix9x9f(m_Amm) * w);
  EigenVector12f e_x;
  e_x.block<3, 1>(0, 0) = e_xr;
  e_x.block<9, 1>(3, 0) = e_xm;
  const EigenVector9f e_e = EigenVector9f(e_Je.m_e + e_Je.m_J * e_x);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  return e_e.dot(e_Amm * e_e);
#else
  const EigenVector9f e_bm = EigenVector9f(EigenVector9f(m_bm) * w);
  //return e_e.dot(e_Amm * e_e * 0.5f + e_bm);
  return e_e.dot(e_Amm * e_e + e_bm * 2.0f);
#endif
}

void Motion::EigenGetResidual(const Camera &C, EigenVector9f *e_r) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C);
  const EigenMatrix9x9f e_A = EigenMatrix9x9f(m_Amm);
  *e_r = EigenVector9f(e_A * e_Je.m_e);
  if (m_bm.Valid()) {
    *e_r += EigenVector9f(m_bm);
  }
}

void Joint::EigenPrior::PropagateLF(const IMU::Delta::EigenFactor::RelativeLF &e_A) {
  EigenMatrix2x2f e_dArr;
  EigenMatrix2x30f e_dArc;
  EigenMatrix30x30f e_dAcc;
  EigenVector2f e_dbr;
  EigenVector30f e_dbc;
  e_A.Get(e_dArr, e_dArc, e_dAcc, e_dbr, e_dbc);

//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator();
  //m_A.Print();
  m_A.PrintDiagonal();
#endif

  const int Nrck = m_A.GetRows() - 15;
  m_A.PushZero(15, 15);
  m_A.block<2, 2>(0, 0) += e_dArr;
  m_A.block(0, Nrck, 2, 30) += e_dArc;
  m_A.block(Nrck, 0, 30, 2) += e_dArc.transpose();
  m_A.block(Nrck, Nrck, 30, 30) += e_dAcc;
  m_b.PushZero(15);
  m_b.block<2, 1>(0, 0) += e_dbr;
  m_b.block(Nrck, 0, 30, 1) += e_dbc;

//#ifdef CFG_DEBUG
#if 0
  //UT::PrintSeparator();
  //m_A.Print();
  UT::Print("-->\n");
  m_A.PrintDiagonal();
#endif

  EigenMatrixXf e_Ab;
  e_Ab.Set(m_A, m_b);
  e_Ab.Marginalize(Nrck, 15);
  e_Ab.Get(m_A, m_b);

//#ifdef CFG_DEBUG
#if 0
  //UT::PrintSeparator();
  //m_A.Print();
  UT::Print("-->\n");
  m_A.PrintDiagonal();

  UT::PrintSeparator();
  //const EigenMatrixXf e_AI = EigenMatrixXf(m_A.ldlt().solve(EigenMatrixXf::Identity(m_A.GetRows())));
  const EigenMatrixXf e_AI = m_A.GetInverseLDL(0.0f);
  e_AI.PrintDiagonal();
  UT::PrintSeparator();
  const EigenMatrixXf e_I1 = EigenMatrixXf(m_A * e_AI);
  const EigenMatrixXf e_I2 = EigenMatrixXf(e_AI * m_A);
  e_I1.PrintDiagonal();
  e_I2.PrintDiagonal();
#endif
}

void Joint::EigenPrior::PropagateKF(const IMU::Delta::EigenFactor::RelativeKF &e_A) {
  EigenMatrix2x2f e_dArr;
  EigenMatrix2x30f e_dArc;
  EigenMatrix30x30f e_dAcc;
  EigenVector2f e_dbr;
  EigenVector30f e_dbc;
  e_A.Get(e_dArr, e_dArc, e_dAcc, e_dbr, e_dbc);
  
//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator();
  //m_A.Print();
  m_A.PrintDiagonal();
#endif

#ifdef CFG_DEBUG
  UT_ASSERT(m_A.GetRows() == 17 && m_A.GetColumns() == 17 && m_b.Size() == 17);
#endif
  m_A.PushZero(15, 15);
  m_A.block<2, 2>(0, 0) += e_dArr;
  m_A.block(0, 2, 2, 30) += e_dArc;
  m_A.block(2, 0, 30, 2) += e_dArc.transpose();
  m_A.block(2, 2, 30, 30) += e_dAcc;
  m_b.PushZero(15);
  m_b.block<2, 1>(0, 0) += e_dbr;
  m_b.block(2, 0, 30, 1) += e_dbc;

//#ifdef CFG_DEBUG
#if 0
  //UT::PrintSeparator();
  //m_A.Print();
  UT::Print("-->\n");
  m_A.PrintDiagonal();
#endif

  EigenMatrixXf e_Ab;
  e_Ab.Set(m_A, m_b);
#if 0
  e_Ab.Marginalize(8, 9);
  e_Ab.Erase(2, 6);
#else
#ifdef CFG_DEBUG
  EigenMatrixXf(e_Ab.block<6, 32>(2, 0)).AssertZero();
#endif
  e_Ab.Erase(2, 6);
  e_Ab.Marginalize(2, 9);
#endif
  e_Ab.Get(m_A, m_b);

//#ifdef CFG_DEBUG
#if 0
  //UT::PrintSeparator();
  //m_A.Print();
  UT::Print("-->\n");
  m_A.PrintDiagonal();

  UT::PrintSeparator();
  const EigenMatrixXf e_AI = EigenMatrixXf(m_A.ldlt().solve(EigenMatrixXf::Identity(17)));
  e_AI.PrintDiagonal();
  UT::PrintSeparator();
  const EigenMatrixXf e_I1 = EigenMatrixXf(m_A * e_AI);
  const EigenMatrixXf e_I2 = EigenMatrixXf(e_AI * m_A);
  e_I1.PrintDiagonal();
  e_I2.PrintDiagonal();
#endif
}

void Joint::EigenPrior::GetPriorPose(const int iKF, Pose::EigenPrior *e_Ap) const {
  //EigenMatrixXf e_Ab;
  //e_Ab.Set(m_A, m_b);
  EigenMatrixXd e_Ab;
  e_Ab.Set(EigenMatrixXd(m_A.cast<double>()), EigenVectorXd(m_b.cast<double>()));
    
  const int Nrc = m_A.GetRows() - 9;
  e_Ab.Marginalize(Nrc, 9);
  if (iKF == INT_MAX) {
    e_Ab.Marginalize(Nrc - 6, 6);
  }
  //e_Ab.Get(e_Ap->m_A, e_Ap->m_b);
  EigenMatrixXf(e_Ab.cast<float>()).Get(e_Ap->m_A, e_Ap->m_b);
}

template<typename TYPE>
static inline void TestMarginalize(const int i, const int Ni, EigenMatrixX<TYPE> *e_A,
                                   const TYPE *eps, const bool erase = true) {
  const int Nr = e_A->GetRows(), Nc = e_A->GetColumns();
  const int j = i + Ni, N1 = i, N2r = Nr - i - Ni, N2c = Nc - j;
  const EigenMatrixX<TYPE> e_Aii = EigenMatrixX<TYPE>(e_A->block(i, i, Ni, Ni));
  //const EigenMatrixX<TYPE> e_Mii = EigenMatrixX<TYPE>(e_Aii.inverse());
  EigenMatrixX<TYPE> e_Mii = e_Aii;
  if (e_Mii.InverseLDL(eps)) {
    const EigenMatrixX<TYPE> e_Ai1 = EigenMatrixX<TYPE>(e_A->block(i, 0, Ni, N1));
    const EigenMatrixX<TYPE> e_Ai2 = EigenMatrixX<TYPE>(e_A->block(i, j, Ni, N2c));
    const EigenMatrixX<TYPE> e_A1i = EigenMatrixX<TYPE>(e_A->block(0, i, N1, Ni));
    const EigenMatrixX<TYPE> e_A2i = EigenMatrixX<TYPE>(e_A->block(j, i, N2r, Ni));
    const EigenMatrixX<TYPE> e_M1i = EigenMatrixX<TYPE>(e_A1i * e_Mii);
    const EigenMatrixX<TYPE> e_M2i = EigenMatrixX<TYPE>(e_A2i * e_Mii);
    e_A->block(0, 0, N1, N1) -= e_M1i * e_Ai1;
    e_A->block(0, j, N1, N2c) -= e_M1i * e_Ai2;
    e_A->block(j, 0, N2r, N1) -= e_M2i * e_Ai1;
    e_A->block(j, j, N2r, N2c) -= e_M2i * e_Ai2;
//#ifdef CFG_DEBUG
#if 0
    if (UT::Debugging()) {
      UT::PrintSeparator();
      EigenMatrix6x6f(e_Aii).Print();
      UT::PrintSeparator();
      EigenMatrix6x6f(e_Aii * e_Mii).Print();

      UT::PrintSeparator();
      EigenMatrix6x6f(-e_Mii).Print();
      UT::PrintSeparator();
      //EigenVector6f(e_Ai2.block<6, 1>(0, e_Ai2.GetColumns() - 1)).Print();
      EigenMatrix6x6f(e_Ai2.block<6, 6>(0, 0)).Print();
      UT::PrintSeparator();
      EigenMatrix6x6f(-e_M2i.block<6, 6>(0, 0).transpose()).Print();
    }
#endif
  }
  if (erase) {
    e_A->Erase(i, Ni);
  } else {
    e_A->block(i, 0, Ni, Nc).setZero();
    e_A->block(0, i, Nr, Ni).setZero();
  }
}

void Joint::EigenPrior::GetPriorMotion(Motion::EigenPrior *e_Ap) const {
  //EigenMatrixXf e_Ab;
  //e_Ab.Set(m_A, m_b);
  EigenMatrixXd e_Ab;
  e_Ab.Set(EigenMatrixXd(m_A.cast<double>()), EigenVectorXd(m_b.cast<double>()));

  const int Nrc = m_A.GetRows() - 9;
//#if 0
#if 1
  e_Ab.Marginalize(0, Nrc);
#else
  //UT::PrintSeparator();
  //EigenMatrix6x6f(e_Ab.block<6, 6>(2, 2)).Print();
  //EigenVector9f(e_Ab.block<9, 1>(e_Ab.GetRows() - 9, e_Ab.GetColumns() - 1)).Print();
  e_Ab.Marginalize(0, 2);
  //UT::PrintSeparator();
  //EigenMatrix6x6f(e_Ab.block<6, 6>(0, 0)).Print();
  //EigenVector9f(e_Ab.block<9, 1>(e_Ab.GetRows() - 9, e_Ab.GetColumns() - 1)).Print();
  const int N = (Nrc - 2) / 6;
  for (int i = 0; i < N; ++i) {
    //e_Ab.Marginalize(0, 6);
//#ifdef CFG_DEBUG
#if 0
    UT::PrintSeparator();
    UT::Print("%d\n", i);
    EigenMatrix6x6f(e_Ab.block<6, 6>(0, 0)).Print();
#endif
#if 0
//#if 1
    if (i == 9) {
      UT::DebugStart();
    }
#endif
    TestMarginalize(0, 6, &e_Ab, eps);
    e_Ab.SetLowerFromUpper();
#if 0
//#if 1
    if (UT::Debugging()) {
      UT::DebugStop();
    }
#endif
#if 0
    EigenVector9f(e_Ab.block<9, 1>(e_Ab.GetRows() - 9, e_Ab.GetColumns() - 1)).Print();
#endif
#if 0
//#if 1
    const int j = 10;
    const int k = (j - 1 - i) * 6;
    if (k >= 0) {
      UT::PrintSeparator();
      UT::Print("%d\n", i);
      EigenMatrix6x6f(e_Ab.block<6, 6>(k, k)).Print();
    }
#endif
  }
#endif
  //e_Ab.Get(e_Ap->m_A, e_Ap->m_b);
  EigenMatrixXf(e_Ab.cast<float>()).Get(e_Ap->m_A, e_Ap->m_b);
}

void Joint::EigenGetResidual(const AlignedVector<Rigid3D> &Cs, const Camera &C,
                             EigenVectorXf *e_r) const {
  const Pose::EigenErrorJacobian e_Jec = Pose::EigenGetErrorJacobian(Cs);
  const Motion::EigenErrorJacobian e_Jem = Motion::EigenGetErrorJacobian(C);
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  EigenVectorXf e_e;
  const int Npc = e_Jec.m_e.Size();
  e_e.Resize(Npc + 9);
  e_e.block(0, 0, Npc, 1) = e_Jec.m_e;
  e_e.block<9, 1>(Npc, 0) = e_Jem.m_e;
  *e_r = EigenVectorXf(e_Ap.m_A * e_e + e_Ap.m_b);
}
#endif

}  // namespace CameraPrior
