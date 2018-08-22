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
//#define CAMERA_PRIOR_DEBUG_EIGEN
#if defined CAMERA_PRIOR_DEBUG_EIGEN && !defined CFG_DEBUG_EIGEN
#define CFG_DEBUG_EIGEN
#endif
#include "CameraPrior.h"

namespace CameraPrior {

bool Pose::Marginalize(const int i, AlignedVector<float> *work, const float *eps) {
  const int N = static_cast<int>(m_iKFs.size());
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
  Matrix::X A;
  Vector::X b, _eps, _work;
  const int Npg = m_Zps.Size() == N ? 0 : 2, Np = Npg + N * 6;
  work->Resize((A.BindSize(Np, Np, true) + b.BindSize(Np) + _work.BindSize(Np) +
               (eps ? _eps.BindSize(6) : 0)) / sizeof(float));
  A.Bind(work->Data(), Np, Np, true);
  b.Bind(A.BindNext(), Np);
  GetPriorEquation(&A, &b);
  _work.Bind(b.BindNext(), Np);
  if (eps) {
    _eps.Bind(_work.BindNext(), 6);
    _eps.Set(eps);
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int ipr = i * 3, ipp = ipr + N * 3;
  bool scc = true;
  scc = A.MarginalizeLDL(ipr, 3, b, &_work, eps ? _eps.Data() + 3 : NULL) && scc;
  scc = A.MarginalizeLDL(ipp - 3, 3, b, &_work, eps ? _eps.Data() : NULL) && scc;
#else
  const bool scc = A.MarginalizeLDL(Npg + i * 6, 6, b, &_work, eps ? _eps.Data() : NULL);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  e_Ap.Marginalize(Npg + i * 6);
#endif
  Erase(i);
  SetPriorEquation(A, b);
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  e_Ap.AssertEqual(*this, 1, "[Pose::Marginalize]");
#endif
  return scc;
#else
  Element::CC &Mcici = m_Acc[i][i];
  const bool scc = Mcici.InverseLDL(eps);
  if (scc) {
    Mcici.MakeMinus();
    const int i1 = 0, i2 = i + 1, N1 = i, N2 = N - i2;
    const Element::C &bci = m_bc[i];
    const Element::RC &Arci = m_Arc[i];
    const Matrix::CC Ac1ci = m_Acc.GetBlock(i1, i, N1, 1);
    const Matrix::CC Acic2 = m_Acc.GetBlock(i, i2, 1, N2);
    Vector::CC Ac2ci, Mc1ci, Mc2ci;
    work->Resize((Ac2ci.BindSize(N2) + Mc1ci.BindSize(N1) + Mc2ci.BindSize(N2)) / sizeof(float));
    Ac2ci.Bind(work->Data(), N2);
    Mc1ci.Bind(Ac2ci.BindNext(), N1);
    Mc2ci.Bind(Mc1ci.BindNext(), N2);
    for (int j = 0; j < N2; ++j) {
      Acic2[0][j].GetTranspose(Ac2ci[j]);
    }
    Element::RC Mrci;
    Vector::RC Arc1 = m_Arc.GetBlock(i1, N1), Arc2 = m_Arc.GetBlock(i2, N2);
    Element::ABT(Arci, Mcici, Mrci);
    Element::AddABTToUpper(Mrci, Arci, m_Arr);
    Vector::AddABTTo(Mrci, Ac1ci, Arc1);
    Vector::AddABTTo(Mrci, Ac2ci, Arc2);
    Element::AddAbTo(Mrci, bci, m_br);
    Matrix::CC Ac1c1 = m_Acc.GetBlock(i1, i1, N1, N1), Ac1c2 = m_Acc.GetBlock(i1, i2, N1, N2);
    Vector::C bc1 = m_bc.GetBlock(i1, N1);
    Vector::ABT(Ac1ci, Mcici, Mc1ci);
    Matrix::AddABTToUpper(Mc1ci, Ac1ci, Ac1c1);
    Matrix::AddABTTo(Mc1ci, Ac2ci, Ac1c2);
    Vector::AddAbTo(Mc1ci, bci, bc1);
    Matrix::CC Ac2c2 = m_Acc.GetBlock(i2, i2, N2, N2);
    Vector::C bc2 = m_bc.GetBlock(i2, N2);
    Vector::ABT(Ac2ci, Mcici, Mc2ci);
    Matrix::AddABTToUpper(Mc2ci, Ac2ci, Ac2c2);
    Vector::AddAbTo(Mc2ci, bci, bc2);
  }
  Erase(i);
  return scc;
#endif
}

bool Pose::MarginalizeUninformative(const float w, const float s2p, const float s2r,
                                    std::vector<int> *iks, AlignedVector<float> *work,
                                    const float *eps) {
  LA::AlignedMatrixXf S;
  if (!GetPriorMeasurement(1.0f, &S, NULL, NULL, work, eps)) {
    return false;
  }
  LA::Vector3f s2pi, s2ri;
  const float _s2p = s2p / w, _s2r = s2r / w;
  const int N = static_cast<int>(m_iKFs.size());
  iks->resize(N);
  for (int i = 0, ip = m_Zps.Size() == N ? 0 : 2; i < N; ++i) {
    S.GetDiagonal(ip, s2pi);  ip += 3;
    S.GetDiagonal(ip, s2ri);  ip += 3;
    if (s2pi.Maximal() > _s2p || s2ri.Maximal() > _s2r) {
      iks->at(i) = i;
    } else {
      iks->at(i) = -1;
    }
  }
  for (int i = 0; i < N; ++i) {
    const int _i = iks->at(i);
    if (_i == -1) {
      continue;
    }
    Marginalize(_i, work, eps);
    iks->at(i) = -1;
    for (int j = i + 1; j < N; ++j) {
      if (iks->at(j) != -1) {
        --iks->at(j);
      }
    }
  }
  //if (m_iKFs.empty()) {
  //  return false;
  //}
  LA::AlignedVectorXf x;
  return GetPriorMeasurement(1.0f, &S, &x, &m_xTb, work, eps);
}

void Pose::SetPriorEquation(const Matrix::X &A, const Vector::X &b, const bool g) {
  const int N = static_cast<int>(m_iKFs.size());
  const bool _g = m_Zps.Size() != N;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr;
  const int ipr = 0, ipp = Npr, ipg = Npr + Npp;
#else
  const int ipg = 0, ipc = _g ? 2 : 0;
#endif
  if (g && _g) {
    A.GetBlockDiagonal(ipg, m_Arr);
    m_Arc.Resize(N);
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Matrix3x2f Acg;
    LA::Matrix2x3f Agp, Agr;
    for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
      A.GetBlock(_ipr, ipg, Acg);   Agr.SetTranspose(Acg);
      A.GetBlock(_ipp, ipg, Acg);   Agp.SetTranspose(Acg);
      m_Arc[i].Set(Agp, Agr);
    }
#else
    for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
      A.GetBlock(ipg, ip, m_Arc[i]);
    }
#endif
    b.GetBlock(ipg, m_br);
  } else {
    m_Arc.Resize(0);
  }
  m_Acc.Resize(N, N, true);
  m_bc.Resize(N);
#ifdef CFG_CAMERA_PRIOR_REORDER
  LA::Matrix3x3f App, Apr, Arp, Arr;
  LA::Vector3f bp, br;
  for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
    for (int j = i, _jpr = _ipr, _jpp = _ipp; j < N; ++j, _jpr += 3, _jpp += 3) {
      A.GetBlock(_ipr, _jpr, Arr);
      A.GetBlock(_ipr, _jpp, Arp);
      if (i == j) {
        Arp.GetTranspose(Apr);
      } else {
        A.GetBlock(_jpr, _ipp, Apr);
        Apr.Transpose();
      }
      A.GetBlock(_ipp, _jpp, App);
      m_Acc[i][j].Set(App, Apr, Arp, Arr);
    }
    b.GetBlock(_ipr, br);
    b.GetBlock(_ipp, bp);
    m_bc[i].Set(bp, br);
  }
#else
  for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
    for (int j = i, jp = ip; j < N; ++j, jp += 6) {
      A.GetBlock(ip, jp, m_Acc[i][j]);
    }
    b.GetBlock(ip, m_bc[i]);
  }
#endif
}

void Pose::GetPriorEquation(Matrix::X *A, Vector::X *b, const bool symmetric, const bool g) const {
  const int N = static_cast<int>(m_iKFs.size());
  const bool _g = m_Zps.Size() != N;
  const int Npg = _g ? 2 : 0;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Np = Npg + Npc;
  const int ipr = 0, ipp = Npr, ipg = Npc;
#else
  const int Np = Npg + N * 6;
  const int ipg = 0, ipc = Npg;
#endif
  A->Resize(Np, Np, symmetric);
  if (b) {
    b->Resize(Np);
  }

  if (g && _g) {
    A->SetBlockDiagonal(ipg, m_Arr);
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Matrix2x3f Agp, Agr;
    LA::Matrix3x2f Acg;
    for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
      m_Arc[i].Get(Agp, Agr);
      Agr.GetTranspose(Acg);  A->SetBlock(_ipr, ipg, Acg);
      Agp.GetTranspose(Acg);  A->SetBlock(_ipp, ipg, Acg);
    }
#else
    for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
      A->SetBlock(ipg, ip, m_Arc[i]);
    }
#endif
    if (b) {
      b->SetBlock(ipg, m_br);
    }
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  LA::Matrix3x3f App, Apr, Arp, Arr;
  LA::Vector3f bp, br;
  for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
    for (int j = i, _jpr = _ipr, _jpp = _ipp; j < N; ++j, _jpr += 3, _jpp += 3) {
      m_Acc[i][j].Get(App, Apr, Arp, Arr);
      A->SetBlock(_ipr, _jpr, Arr);
      A->SetBlock(_ipr, _jpp, Arp);
      if (i != j) {
        Apr.Transpose();
        A->SetBlock(_jpr, _ipp, Apr);
      }
      A->SetBlock(_ipp, _jpp, App);
    }
    if (b) {
      m_bc[i].Get(bp, br);
      b->SetBlock(_ipr, br);
      b->SetBlock(_ipp, bp);
    }
  }
#else
  for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
    for (int j = i, jp = ip; j < N; ++j, jp += 6) {
      A->SetBlock(ip, jp, m_Acc[i][j]);
    }
    if (b) {
      b->SetBlock(ip, m_bc[i]);
    }
  }
#endif
}

bool Pose::GetPriorMeasurement(const Element::T w, Matrix::X *S, Vector::X *x,
                               Element::T *xTb, const Element::T *eps) const {
  GetPriorEquation(S, x, false);
  if (x) {
    Vector::X b;
    if (xTb) {
      const int Np = x->Size(), NpC = SIMD::Ceil<Element::T>(Np);
#ifdef CFG_DEBUG
      UT_ASSERT(x->Capacity() >= NpC + Np);
#endif
      b.Bind(x->Data() + NpC, Np);
      b.Copy(*x);
    }
    if (!S->SolveLDL(*x, eps)) {
      return false;
    }
    x->MakeMinus();
    S->InverseLDL(eps, true);
    if (xTb) {
      *xTb = x->Dot(b);
    }
  } else {
    if (!S->InverseLDL(eps)) {
      return false;
    }
  }
  if (w != 1) {
    *S *= w;
    if (xTb) {
      *xTb /= w;
    }
  }
  return true;
}

bool Pose::GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                               float *xTb, AlignedVector<float> *work, const float *eps) const {
  if (Invalid()) {
    return false;
  }
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Np = Npg + Npc;
  const int ipr = 0, ipp = Npr, ipg = Npc;
#else
  const int Np = Npg + N * 6;
  const int ipg = 0;
#endif
  Matrix::X _S;
  Vector::X _x, _eps;
  Element::T _xTb;
  const int _Nx = xTb ? (SIMD::Ceil<Element::T>(Np) + Np) : Np;
  work->Resize((S->BindSize(Np, Np, true) + (x ? x->BindSize(Np) : 0) +
                _S.BindSize(Np, Np) + (x ? _x.BindSize(_Nx) : 0) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  S->Bind(work->Data(), Np, Np, true);
  if (x) {
    x->Bind(S->BindNext(), Np);
  }
  _S.Bind(x ? x->BindNext() : S->BindNext(), Np, Np);
  if (x) {
    _x.Bind(_S.BindNext(), _Nx);
  }
  if (eps) {
    _eps.Bind(x ? _x.BindNext() : _S.BindNext(), Np);
    if (g) {
      LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg);
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *ecrs = (LA::Vector3<Element::T> *) (_eps.Data() + ipr);
    LA::Vector3<Element::T> *ecps = (LA::Vector3<Element::T> *) (_eps.Data() + ipp);
    for (int i = 0; i < N; ++i) {
      ecrs[i].Set(eps + 3);
      ecps[i].Set(eps);
    }
#else
    LA::Vector6<Element::T> *ecs = (LA::Vector6<Element::T> *) (_eps.Data() + Npg);
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
#endif
  }
  if (!GetPriorMeasurement(static_cast<Element::T>(w), &_S, x ? &_x : NULL,
                           xTb ? &_xTb : NULL, eps ? _eps.Data() : NULL)) {
    S->Resize(0, 0, true);
    if (x) {
      x->Resize(0);
      if (xTb) {
        *xTb = 0.0f;
      }
    }
    return false;
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  if (x) {
    if (g) {
      LA::Vector2f xg;
      _x.GetBlock(ipg, xg);
      x->SetBlock(0, xg);
    }
    LA::Vector3f xc;
    for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
         ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
      _x.GetBlock(ipr1, xc);  x->SetBlock(ipr2, xc);
      _x.GetBlock(ipp1, xc);  x->SetBlock(ipp2, xc);
    }
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
  if (g) {
    LA::SymmetricMatrix2x2f Sgg;
    _S.GetBlockDiagonal(ipg, Sgg);
    S->SetBlockDiagonal(0, Sgg);
    LA::Matrix2x3f Sgc;
    for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
         ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
      _S.GetBlock(ipg, ipr1, Sgc);  S->SetBlock(0, ipr2, Sgc);
      _S.GetBlock(ipg, ipp1, Sgc);  S->SetBlock(0, ipp2, Sgc);
    }
  }
  LA::Matrix3x3f Scc;
  for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
       ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
    for (int j = i, jpr1 = ipr1, jpp1 = ipp1, jpp2 = ipp2, jpr2 = ipr2; j < N;
         ++j, jpr1 += 3, jpp1 += 3, jpp2 += 6, jpr2 += 6) {
      _S.GetBlock(ipr1, jpr1, Scc);   S->SetBlock(ipr2, jpr2, Scc);
      _S.GetBlock(ipp1, jpp1, Scc);   S->SetBlock(ipp2, jpp2, Scc);
      _S.GetBlock(ipp1, jpr1, Scc);   S->SetBlock(ipp2, jpr2, Scc);
      if (j == i) {
        continue;
      }
      _S.GetBlock(ipr1, jpp1, Scc);   S->SetBlock(ipr2, jpp2, Scc);
    }
  }
#else
  if (x) {
    _x.Get(*x);
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
  _S.Get(*S);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  const std::string str = "[Pose::GetPriorMeasurement]";
  EigenMatrixXf e_S;
  EigenVectorXf e_x;
#ifdef CFG_CAMERA_PRIOR_DOUBLE
  EigenGetPriorMeasurement(w, &e_S, x ? &e_x : NULL);
  e_S.AssertEqual(*S, 1, str);
  if (x) {
    e_x.AssertEqual(*x, 1, str);
  }
#else
  EigenPrior e_Zp;
  e_Zp.Set(*this);
  //e_S = *S;
  //if (w != 1) {
  //  e_S /= w;
  //}
  //const EigenMatrixXf e_I = EigenMatrixXf::Identity(Np);
  //const EigenMatrixXf e_E1 = e_S * e_Zp.m_A - e_I;
  //const EigenMatrixXf e_E2 = e_Zp.m_A * e_S - e_I;
  ////const float e_eps = 1.0e-3f;
  //const float e_eps = 1.0e-1f;
  //e_E1.AssertZero(1, str, e_eps);
  //e_E2.AssertZero(1, str, e_eps);
  if (x) {
    e_x = *x;
    const EigenVectorXf e_e = e_Zp.m_A * e_x + e_Zp.m_b;
    e_e.AssertZero(1, str);
  }
#endif
#endif
  return true;
#else
  const int Np = Npg + N * 6;
  const int Nx = xTb ? (SIMD::Ceil<Element::T>(Np) + Np) : Np;
  LA::AlignedVectorXf _eps;
  work->Resize((S->BindSize(Np, Np) + (x ? x->BindSize(Nx) : 0) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  S->Bind(work->Data(), Np, Np);
  if (x) {
    x->Bind(S->BindNext(), Nx);
  }
  if (eps) {
    _eps.Bind(x ? x->BindNext() : S->BindNext(), Np);
    LA::Vector2f *eg = (LA::Vector2f *) _eps.Data();
    if (g) {
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
    LA::Vector6f *ecs = (LA::Vector6f *) (eg + (g ? 1 : 0));
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
  }
  return GetPriorMeasurement(w, S, x, xTb, eps ? _eps.Data() : NULL);
#endif
}

bool Motion::GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                                 float *xTb, AlignedVector<float> *work, const float *eps) const {
  if (Invalid()) {
    return false;
  }
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
  Matrix::X _S;
  Vector::X _x, _eps;
  Element::T _xTb;
  const int _Nx = xTb ? (SIMD::Ceil<Element::T>(9) + 9) : 9;
  work->Resize((S->BindSize(9, 9) + (x ? x->BindSize(9) : 0) +
                _S.BindSize(9, 9) + (x ? _x.BindSize(_Nx) : 0) +
               (eps ? _eps.BindSize(9) : 0)) / sizeof(float));
  S->Bind(work->Data(), 9, 9);
  if (x) {
    x->Bind(S->BindNext(), 9);
  }
  _S.Bind(x ? x->BindNext() : S->BindNext(), 9, 9);
  if (x) {
    _x.Bind(_S.BindNext(), _Nx);
  }
  if (eps) {
    _eps.Bind(x ? _x.BindNext() : _S.BindNext(), 9);
    _eps.Set(eps);
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  LA::Matrix3x3f A;
  LA::Vector3f b;
  m_Amm.GetBlock(6, 6, A);  _S.SetBlock(0, 0, A);
  m_Amm.GetBlock(6, 0, A);  _S.SetBlock(0, 3, A);
  m_Amm.GetBlock(6, 3, A);  _S.SetBlock(0, 6, A);
  m_Amm.GetBlock(0, 0, A);  _S.SetBlock(3, 3, A);
  m_Amm.GetBlock(0, 3, A);  _S.SetBlock(3, 6, A);
  m_Amm.GetBlock(3, 3, A);  _S.SetBlock(6, 6, A);
  if (x) {
    m_bm.Get678(b);   _x.SetBlock(0, b);
    m_bm.Get012(b);   _x.SetBlock(3, b);
    m_bm.Get345(b);   _x.SetBlock(6, b);
  }
  if (eps) {
    const LA::Vector3f *ev = (LA::Vector3f *) eps, *eba = ev + 1, *ebw = eba + 1;
    _eps.SetBlock(0, *ebw);
    _eps.SetBlock(3, *ev);
    _eps.SetBlock(6, *eba);
  }
#else
  _S.SetBlock(0, 0, m_Amm);
  if (x) {
    _x.SetBlock(0, m_bm);
  }
#endif
  if (x) {
    Vector::X _b;
    if (xTb) {
      _x.Resize(9);
      _b.Bind(_x.Data() + 9, 9);
      _b.Copy(_x);
    }
    if (!_S.SolveLDL(_x, eps ? _eps.Data() : NULL)) {
      S->Resize(0, 0);
      if (x) {
        x->Resize(0);
        if (xTb) {
          *xTb = 0.0f;
        }
      }
      return false;
    }
    _x.MakeMinus();
    _S.InverseLDL(eps ? _eps.Data() : NULL, true);
    if (xTb) {
      _xTb = _x.Dot(_b);
    }
  } else {
    if (!_S.InverseLDL(eps ? _eps.Data() : NULL)) {
      return false;
    }
  }
  if (w != 1.0) {
    const Element::T _w = static_cast<Element::T>(w);
    _S *= _w;
    if (xTb) {
      _xTb /= _w;
    }
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  _S.GetBlock(0, 0, A);   S->SetBlock(6, 6, A);
  _S.GetBlock(3, 0, A);   S->SetBlock(0, 6, A);
  _S.GetBlock(6, 0, A);   S->SetBlock(3, 6, A);
  _S.GetBlock(3, 3, A);   S->SetBlock(0, 0, A);
  _S.GetBlock(3, 6, A);   S->SetBlock(0, 3, A);
  _S.GetBlock(6, 6, A);   S->SetBlock(3, 3, A);
  if (x) {
    _x.GetBlock(0, b);  x->SetBlock(6, b);
    _x.GetBlock(3, b);  x->SetBlock(0, b);
    _x.GetBlock(6, b);  x->SetBlock(3, b);
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
  S->SetLowerFromUpper();
#else
  _S.GetBlock(0, 0, *S);
  if (x) {
    _x.GetBlock(0, *x);
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  const std::string str = "[Motion::GetPriorMeasurement]";
  EigenMatrixXf e_S;
  EigenVectorXf e_x;
#ifdef CFG_CAMERA_PRIOR_DOUBLE
  EigenGetPriorMeasurement(w, &e_S, x ? &e_x : NULL);
  e_S.AssertEqual(*S, 1, str);
  if (x) {
    e_x.AssertEqual(*x, 1, str);
  }
#else
  EigenPrior e_Zp;
  e_Zp.Set(*this);
//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator('*');
  e_Zp.m_A.Print();
  e_Zp.m_b.Print();
  UT::PrintSeparator();
  S->Print();
  if (x) {
    x->Print();
  }
#endif
  //e_S = *S;
  //if (w != 1) {
  //  e_S /= w;
  //}
  //const EigenMatrix9x9f e_I = EigenMatrix9x9f::Identity();
  //const EigenMatrix9x9f e_E1 = e_S * e_Zp.m_A - e_I;
  //const EigenMatrix9x9f e_E2 = e_Zp.m_A * e_S - e_I;
  ////const float e_eps = 1.0e-3f;
  //const float e_eps = 1.0e-1f;
  //e_E1.AssertZero(1, str, e_eps);
  //e_E2.AssertZero(1, str, e_eps);
  if (x) {
    e_x = *x;
    const EigenVector9f e_e = e_Zp.m_A * e_x + e_Zp.m_b;
    e_e.AssertZero(1, str);
  }
#endif
#endif
#else
  const int Nx = xTb ? (SIMD::Ceil<float>(9) + 9) : 9;
  work->Resize((S->BindSize(9, 9) + (x ? x->BindSize(Nx) : 0)) / sizeof(float));
  S->Bind(work->Data(), 9, 9);
  if (x) {
    x->Bind(S->BindNext(), Nx);
  }
  S->SetBlock(0, 0, m_Amm);
  if (x) {
    x->SetBlock(0, m_bm);
    Vector::X b;
    if (xTb) {
      x->Resize(9);
      b.Bind(x->Data() + SIMD::Ceil<float>(9), 9);
      b.Copy(*x);
    }
    if (!S->SolveLDL(*x, eps)) {
      return false;
    }
    x->MakeMinus();
    S->InverseLDL(eps, true);
    if (xTb) {
      *xTb = x->Dot(b);
    }
  } else {
    if (!S->InverseLDL(eps)) {
      return false;
    }
  }
  if (w != 1.0f) {
    *S *= w;
    if (xTb) {
      *xTb /= w;
    }
  }
#endif
  return true;
}

void Joint::SetPriorEquation(const Matrix::X &A, const Vector::X &b) {
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr;
#ifdef CFG_DEBUG
  const int Np = Npg + Npr + Npp + 9;
  UT_ASSERT(A.GetRows() == Np && A.GetColumns() == Np && b.Size() == Np);
#endif
  const int ipr = 0, ipp = Npr;
  const int ipbw = Npr + Npp, ipv = ipbw + 3, ipg = ipv + 3, ipba = ipg + Npg;
  Pose::SetPriorEquation(A, b, false);
  if (g) {
    A.GetBlockDiagonal(ipg, m_Arr);
    m_Arc.Resize(N);
    LA::Matrix3x2f Acg, Amg;
    LA::Matrix2x3f Agp, Agr, Agm;
    for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
      A.GetBlock(_ipr, ipg, Acg);   Agr.SetTranspose(Acg);
      A.GetBlock(_ipp, ipg, Acg);   Agp.SetTranspose(Acg);
      m_Arc[i].Set(Agp, Agr);
    }
    A.GetBlock(ipbw, ipg, Amg); Agm.SetTranspose(Amg);  m_Arm.SetBlock(0, 6, Agm);
    A.GetBlock(ipv, ipg, Amg);  Agm.SetTranspose(Amg);  m_Arm.SetBlock(0, 0, Agm);
    A.GetBlock(ipg, ipba, Agm);                         m_Arm.SetBlock(0, 3, Agm);
    b.GetBlock(ipg, m_br);
  }
  m_Acm.Resize(N);
  LA::Matrix3x3f Acm;
  for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
    LA::AlignedMatrix6x9f &_Acm = m_Acm[i];
    A.GetBlock(_ipp, ipv, Acm);   _Acm.SetBlock(0, 0, Acm);
    A.GetBlock(_ipp, ipba, Acm);  _Acm.SetBlock(0, 3, Acm);
    A.GetBlock(_ipp, ipbw, Acm);  _Acm.SetBlock(0, 6, Acm);
    A.GetBlock(_ipr, ipv, Acm);   _Acm.SetBlock(3, 0, Acm);
    A.GetBlock(_ipr, ipba, Acm);  _Acm.SetBlock(3, 3, Acm);
    A.GetBlock(_ipr, ipbw, Acm);  _Acm.SetBlock(3, 6, Acm);
  }
  LA::Matrix3x3f Amm;
  LA::Vector3f bm;
  A.GetBlock(ipbw, ipbw, Amm);                    m_Amm.SetBlock(6, 6, Amm);
  A.GetBlock(ipbw, ipv, Amm);   Amm.Transpose();  m_Amm.SetBlock(0, 6, Amm);
  A.GetBlock(ipbw, ipba, Amm);  Amm.Transpose();  m_Amm.SetBlock(3, 6, Amm);
  A.GetBlock(ipv, ipv, Amm);                      m_Amm.SetBlock(0, 0, Amm);
  A.GetBlock(ipv, ipba, Amm);                     m_Amm.SetBlock(0, 3, Amm);
  A.GetBlock(ipba, ipba, Amm);                    m_Amm.SetBlock(3, 3, Amm);
  m_Amm.SetLowerFromUpper();
  b.GetBlock(ipv, bm);          m_bm.Set012(bm);
  b.GetBlock(ipba, bm);         m_bm.Set345(bm);
  b.GetBlock(ipbw, bm);         m_bm.Set678(bm);
#else
  const int Npc = N * 6;
#ifdef CFG_DEBUG
  const int Np = Npg + Npc + 9;
  UT_ASSERT(A.GetRows() == Np && A.GetColumns() == Np && b.Size() == Np);
#endif
  const int ipg = 0, ipc = Npg, ipm = ipc + Npc;
  Pose::SetPriorEquation(A, b);
  if (g) {
    A.GetBlock(ipg, ipm, m_Arm);
  }
  m_Acm.Resize(N);
  for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
    A.GetBlock(ip, ipm, m_Acm[i]);
  }
  A.GetBlock(ipm, ipm, m_Amm);
  b.GetBlock(ipm, m_bm);
#endif
}

void Joint::GetPriorEquation(Matrix::X *A, Vector::X *b, const bool symmetric) const {
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Np = Npg + Npc + 9;
  const int ipr = 0, ipp = Npr;
  const int ipbw = Npc, ipv = ipbw + 3, ipg = ipv + 3, ipba = ipg + Npg;
  Pose::GetPriorEquation(A, b, symmetric, false);
  A->Resize(Np, Np, symmetric, true);
  if (b) {
    b->Resize(Np, true);
  }
  if (g) {
    A->SetBlockDiagonal(ipg, m_Arr);
    LA::Matrix2x3f Agp, Agr, Agm;
    LA::Matrix3x2f Acg, Amg;
    for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
      m_Arc[i].Get(Agp, Agr);
      Agr.GetTranspose(Acg);  A->SetBlock(_ipr, ipg, Acg);
      Agp.GetTranspose(Acg);  A->SetBlock(_ipp, ipg, Acg);
    }
    m_Arm.GetBlock(0, 0, Agm);  Agm.GetTranspose(Amg);  A->SetBlock(ipv, ipg, Amg);
    m_Arm.GetBlock(0, 6, Agm);  Agm.GetTranspose(Amg);  A->SetBlock(ipbw, ipg, Amg);
    m_Arm.GetBlock(0, 3, Agm);                          A->SetBlock(ipg, ipba, Agm);
    if (b) {
      b->SetBlock(ipg, m_br);
    }
  }
  LA::Matrix3x3f Acm;
  for (int i = 0, _ipr = ipr, _ipp = ipp; i < N; ++i, _ipr += 3, _ipp += 3) {
    const LA::AlignedMatrix6x9f &_Acm = m_Acm[i];
    _Acm.GetBlock(0, 0, Acm);   A->SetBlock(_ipp, ipv, Acm);
    _Acm.GetBlock(0, 3, Acm);   A->SetBlock(_ipp, ipba, Acm);
    _Acm.GetBlock(0, 6, Acm);   A->SetBlock(_ipp, ipbw, Acm);
    _Acm.GetBlock(3, 0, Acm);   A->SetBlock(_ipr, ipv, Acm);
    _Acm.GetBlock(3, 3, Acm);   A->SetBlock(_ipr, ipba, Acm);
    _Acm.GetBlock(3, 6, Acm);   A->SetBlock(_ipr, ipbw, Acm);
  }
  LA::Matrix3x3f Amm;
  LA::Vector3f bm;
  m_Amm.GetBlock(6, 6, Amm);  A->SetBlock(ipbw, ipbw, Amm);
  m_Amm.GetBlock(6, 0, Amm);  A->SetBlock(ipbw, ipv, Amm);
  m_Amm.GetBlock(6, 3, Amm);  A->SetBlock(ipbw, ipba, Amm);
  m_Amm.GetBlock(0, 0, Amm);  A->SetBlock(ipv, ipv, Amm);
  m_Amm.GetBlock(0, 3, Amm);  A->SetBlock(ipv, ipba, Amm);
  m_Amm.GetBlock(3, 3, Amm);  A->SetBlock(ipba, ipba, Amm);
  if (b) {
    m_bm.Get012(bm);            b->SetBlock(ipv, bm);
    m_bm.Get345(bm);            b->SetBlock(ipba, bm);
    m_bm.Get678(bm);            b->SetBlock(ipbw, bm);
  }
#else
  const int Npc = N * 6, Np = Npg + Npc + 9;
  const int ipg = 0, ipc = Npg, ipm = ipc + Npc;
  Pose::GetPriorEquation(A, b, symmetric);
  A->Resize(Np, Np, symmetric, true);
  if (b) {
    b->Resize(Np, true);
  }
  if (g) {
    A->SetBlock(ipg, ipm, m_Arm);
  }
  for (int i = 0, ip = ipc; i < N; ++i, ip += 6) {
    A->SetBlock(ip, ipm, m_Acm[i]);
  }
  A->SetBlock(ipm, ipm, m_Amm);
  if (b) {
    b->SetBlock(ipm, m_bm);
  }
#endif
}

bool Joint::GetPriorMeasurement(const Element::T w, Matrix::X *S, Vector::X *x,
                                Element::T *xTb, const Element::T *eps) const {
  GetPriorEquation(S, x, false);
//#ifdef CAMERA_PRIOR_DEBUG_EIGEN
#if 0
  S->SetLowerFromUpper();
  S->Print();
  const EigenMatrixX<Element::T> e_A = *S;
  EigenVectorX<Element::T> e_b;
  if (x) {
    e_b = *x;
  }
#endif
  if (x) {
    Vector::X b;
    if (xTb) {
      const int Np = x->Size(), NpC = SIMD::Ceil<Element::T>(Np);
#ifdef CFG_DEBUG
      UT_ASSERT(x->Capacity() >= NpC + Np);
#endif
      b.Bind(x->Data() + NpC, Np);
      b.Copy(*x);
    }
    if (!S->SolveLDL(*x, eps)) {
      return false;
    }
    x->MakeMinus();
    S->InverseLDL(eps, true);
    if (xTb) {
      *xTb = x->Dot(b);
    }
  } else {
    if (!S->InverseLDL(eps)) {
      return false;
    }
  }
//#ifdef CAMERA_PRIOR_DEBUG_EIGEN
#if 0
  EigenMatrixX<Element::T> e_S[2];
  const auto e_ldlt = e_A.ldlt();
  const int N = static_cast<int>(e_A.rows());
  const EigenMatrixX<Element::T> e_I = EigenMatrixX<Element::T>::Identity(N);
  e_S[0] = e_ldlt.solve(e_I);
  e_S[0].AssertEqual(*S);
  e_S[1] = *S;
  for (int i = 0; i < 2; ++i) {
    const EigenMatrixX<Element::T> e_E1 = e_A * e_S[i] - e_I;
    UT::PrintSeparator('*');
    e_E1.Print();
    const EigenMatrixX<Element::T> e_I2 = e_S[i] * e_A - e_I;
    UT::PrintSeparator();
    e_E1.Print();
  }
  if (x) {
    EigenVectorX<Element::T> e_x[2];
    e_x[0] = e_ldlt.solve(-e_b);
    e_x[0].AssertEqual(*x);
    e_x[1] = *x;
    for (int i = 0; i < 2; ++i) {
      const EigenVectorX<Element::T> e_e = e_A * e_x[i] + e_b;
      UT::PrintSeparator('*');
      e_e.Print();
    }
  }
#endif
  if (w != 1) {
    *S *= w;
    if (xTb) {
      *xTb /= w;
    }
  }
  return true;
}

bool Joint::GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                                float *xTb, AlignedVector<float> *work, const float *eps) const {
  if (Pose::Invalid() || Motion::Invalid()) {
    return false;
  }
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Npgc = Npg + Npc, Np = Npgc + 9;
  const int ipr = 0, ipp = Npr;
  const int ipbw1 = Npc, ipv1 = ipbw1 + 3, ipg1 = ipv1 + 3, ipba1 = ipg1 + Npg;
  const int ipg2 = 0, ipv2 = Npgc, ipba2 = ipv2 + 3, ipbw2 = ipba2 + 3;
#else
  const int Npgc = Npg + N * 6, Np = Npgc + 9;
  const int ipg1 = 0, ipc = Npg, ipm = Npgc;
#endif
  Matrix::X _S;
  Vector::X _x, _eps;
  Element::T _xTb;
  const int _Nx = xTb ? (SIMD::Ceil<Element::T>(Np) + Np) : Np;
  work->Resize((S->BindSize(Np, Np, true) + (x ? x->BindSize(Np) : 0) +
                _S.BindSize(Np, Np) + (x ? _x.BindSize(_Nx) : 0) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  S->Bind(work->Data(), Np, Np, true);
  if (x) {
    x->Bind(S->BindNext(), Np);
  }
  _S.Bind(x ? x->BindNext() : S->BindNext(), Np, Np);
  if (x) {
    _x.Bind(_S.BindNext(), _Nx);
  }
  if (eps) {
    _eps.Bind(x ? _x.BindNext() : _S.BindNext(), Np);
    if (g) {
      LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg1);
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *ecrs = (LA::Vector3<Element::T> *) (_eps.Data() + ipr);
    LA::Vector3<Element::T> *ecps = (LA::Vector3<Element::T> *) (_eps.Data() + ipp);
    for (int i = 0; i < N; ++i) {
      ecrs[i].Set(eps + 3);
      ecps[i].Set(eps);
    }
    LA::Vector3<Element::T> *ev = (LA::Vector3<Element::T> *) (_eps.Data() + ipv1);
    LA::Vector3<Element::T> *eba = (LA::Vector3<Element::T> *) (_eps.Data() + ipba1);
    LA::Vector3<Element::T> *ebw = (LA::Vector3<Element::T> *) (_eps.Data() + ipbw1);
    ev->Set(eps + 6);
    eba->Set(eps + 9);
    ebw->Set(eps + 12);
#else
    LA::Vector6<Element::T> *ecs = (LA::Vector6<Element::T> *) (_eps.Data() + ipc);
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
    LA::Vector9<Element::T> *em = (LA::Vector9<Element::T> *) (_eps.Data() + ipm);
    em->Set(eps + 6);
#endif
  }
  if (!GetPriorMeasurement(static_cast<Element::T>(w), &_S, x ? &_x : NULL,
                           xTb ? &_xTb : NULL, eps ? _eps.Data() : NULL)) {
    S->Resize(0, 0, true);
    if (x) {
      x->Resize(0);
      if (xTb) {
        *xTb = 0.0f;
      }
    }
    return false;
  }
#ifdef CFG_CAMERA_PRIOR_REORDER
  if (x) {
    if (g) {
      LA::Vector2f xg;
      _x.GetBlock(ipg1, xg);
      x->SetBlock(ipg2, xg);
    }
    LA::Vector3f xc;
    for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
         ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
      _x.GetBlock(ipr1, xc);  x->SetBlock(ipr2, xc);
      _x.GetBlock(ipp1, xc);  x->SetBlock(ipp2, xc);
    }
    LA::Vector3f xm;
    _x.GetBlock(ipv1, xm);    x->SetBlock(ipv2, xm);
    _x.GetBlock(ipba1, xm);   x->SetBlock(ipba2, xm);
    _x.GetBlock(ipbw1, xm);   x->SetBlock(ipbw2, xm);
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
  if (g) {
    LA::SymmetricMatrix2x2f Sgg;
    _S.GetBlockDiagonal(ipg1, Sgg);
    S->SetBlockDiagonal(ipg2, Sgg);
    LA::Matrix2x3f Sgc;
    for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
         ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
      _S.GetBlock(ipg1, ipr1, Sgc);  S->SetBlock(ipg2, ipr2, Sgc);
      _S.GetBlock(ipg1, ipp1, Sgc);  S->SetBlock(ipg2, ipp2, Sgc);
    }
    LA::Matrix2x3f Sgm;
    _S.GetBlock(ipg1, ipv1, Sgm);   S->SetBlock(ipg2, ipv2, Sgm);
    _S.GetBlock(ipg1, ipba1, Sgm);  S->SetBlock(ipg2, ipba2, Sgm);
    _S.GetBlock(ipg1, ipbw1, Sgm);  S->SetBlock(ipg2, ipbw2, Sgm);
  }
  LA::Matrix3x3f Scc, Scm;
  for (int i = 0, ipr1 = ipr, ipp1 = ipp, ipp2 = Npg, ipr2 = ipp2 + 3; i < N;
       ++i, ipr1 += 3, ipp1 += 3, ipp2 += 6, ipr2 += 6) {
    for (int j = i, jpr1 = ipr1, jpp1 = ipp1, jpp2 = ipp2, jpr2 = ipr2; j < N;
         ++j, jpr1 += 3, jpp1 += 3, jpp2 += 6, jpr2 += 6) {
      _S.GetBlock(ipr1, jpr1, Scc);   S->SetBlock(ipr2, jpr2, Scc);
      _S.GetBlock(ipp1, jpp1, Scc);   S->SetBlock(ipp2, jpp2, Scc);
      _S.GetBlock(ipp1, jpr1, Scc);   S->SetBlock(ipp2, jpr2, Scc);
      if (j == i) {
        continue;
      }
      _S.GetBlock(ipr1, jpp1, Scc);   S->SetBlock(ipr2, jpp2, Scc);
    }
    _S.GetBlock(ipr1, ipv1, Scm);   S->SetBlock(ipr2, ipv2, Scm);
    _S.GetBlock(ipr1, ipba1, Scm);  S->SetBlock(ipr2, ipba2, Scm);
    _S.GetBlock(ipr1, ipbw1, Scm);  S->SetBlock(ipr2, ipbw2, Scm);
    _S.GetBlock(ipp1, ipv1, Scm);   S->SetBlock(ipp2, ipv2, Scm);
    _S.GetBlock(ipp1, ipba1, Scm);  S->SetBlock(ipp2, ipba2, Scm);
    _S.GetBlock(ipp1, ipbw1, Scm);  S->SetBlock(ipp2, ipbw2, Scm);
  }
  LA::Matrix3x3f Smm;
  _S.GetBlock(ipv1, ipv1, Smm);   S->SetBlock(ipv2, ipv2, Smm);
  _S.GetBlock(ipv1, ipba1, Smm);  S->SetBlock(ipv2, ipba2, Smm);
  _S.GetBlock(ipv1, ipbw1, Smm);  S->SetBlock(ipv2, ipbw2, Smm);
  _S.GetBlock(ipba1, ipba1, Smm); S->SetBlock(ipba2, ipba2, Smm);
  _S.GetBlock(ipba1, ipbw1, Smm); S->SetBlock(ipba2, ipbw2, Smm);
  _S.GetBlock(ipbw1, ipbw1, Smm); S->SetBlock(ipbw2, ipbw2, Smm);
#else
  if (x) {
    _x.Get(*x);
    if (xTb) {
      *xTb = static_cast<float>(_xTb);
    }
  }
  _S.Get(*S);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  const std::string str = "[Joint::GetPriorMeasurement]";
  EigenMatrixXf e_S;
  EigenVectorXf e_x;
#ifdef CFG_CAMERA_PRIOR_DOUBLE
  EigenGetPriorMeasurement(w, &e_S, x ? &e_x : NULL);
  e_S.AssertEqual(*S, 1, str);
  if (x) {
    e_x.AssertEqual(*x, 1, str);
  }
#else
  EigenPrior e_Zp;
  e_Zp.Set(*this);
//#ifdef CFG_DEBUG
#if 0
  UT::PrintSeparator('*');
  e_Zp.m_A.Print();
  e_Zp.m_b.Print();
  UT::PrintSeparator();
  S->Print();
  if (x) {
    x->Print();
  }
#endif
//  e_S = *S;
//  if (w != 1) {
//    e_S /= w;
//  }
//  const EigenMatrixXf e_I = EigenMatrixXf::Identity(Np);
//  const EigenMatrixXf e_E1 = e_S * e_Zp.m_A - e_I;
//  const EigenMatrixXf e_E2 = e_Zp.m_A * e_S - e_I;
////#ifdef CFG_DEBUG
//#if 0
//  UT::PrintSeparator();
//  e_Zp.m_A.PrintDiagonal(true);
//  UT::PrintSeparator();
//  e_S.PrintDiagonal(true);
//#endif
//  //const float e_eps = 1.0e-3f;
//  const float e_eps = 1.0e-1f;
//  e_E1.AssertZero(1, str, e_eps);
//  e_E2.AssertZero(1, str, e_eps);
  if (x) {
    e_x = *x;
    const EigenVectorXf e_e = e_Zp.m_A * e_x + e_Zp.m_b;
    e_e.AssertZero(1, str);
  }
#endif
#endif
  return true;
#else
  const int Np = Npg + N * 6 + 9;
  const int Nx = xTb ? (SIMD::Ceil<float>(Np) + Np) : Np;
  LA::AlignedVectorXf _eps;
  work->Resize((S->BindSize(Np, Np) + (x ? x->BindSize(Nx) : 0) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  S->Bind(work->Data(), Np, Np);
  if (x) {
    x->Bind(S->BindNext(), Nx);
  }
  if (eps) {
    _eps.Bind(x ? x->BindNext() : S->BindNext(), Np);
    LA::Vector2f *eg = (LA::Vector2f *) _eps.Data();
    if (g) {
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
    LA::Vector6f *ecs = (LA::Vector6f *) (eg + (g ? 1 : 0));
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
    LA::Vector9f *em = (LA::Vector9f *) (ecs + N);
    em->Set(eps + 6);
  }
  return GetPriorMeasurement(w, S, x, xTb, eps ? _eps.Data() : NULL);
#endif
}

bool Joint::Invertible(AlignedVector<float> *work, const float *eps) const {
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Npgc = Npg + Npc, Np = Npgc + 9;
  const int ipr = 0, ipp = Npr;
  const int ipbw = Npc, ipv = ipbw + 3, ipg = ipv + 3, ipba = ipg + Npg;
#else
  const int Npgc = Npg + N * 6, Np = Npgc + 9;
  const int ipg = 0, ipc = Npg, ipm = Npgc;
#endif
  Matrix::X S;
  Vector::X _work, _eps;
  work->Resize((S.BindSize(Np, Np, true) + _work.BindSize(Np) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  S.Bind(work->Data(), Np, Np, true);
  _work.Bind(S.BindNext(), Np);
  if (eps) {
    _eps.Bind(_work.BindNext(), Np);
    if (g) {
      LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg);
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *ecrs = (LA::Vector3<Element::T> *) (_eps.Data() + ipr);
    LA::Vector3<Element::T> *ecps = (LA::Vector3<Element::T> *) (_eps.Data() + ipp);
    for (int i = 0; i < N; ++i) {
      ecrs[i].Set(eps + 3);
      ecps[i].Set(eps);
    }
    LA::Vector3<Element::T> *ev = (LA::Vector3<Element::T> *) (_eps.Data() + ipv);
    LA::Vector3<Element::T> *eba = (LA::Vector3<Element::T> *) (_eps.Data() + ipba);
    LA::Vector3<Element::T> *ebw = (LA::Vector3<Element::T> *) (_eps.Data() + ipbw);
    ev->Set(eps + 6);
    eba->Set(eps + 9);
    ebw->Set(eps + 12);
#else
    LA::Vector6<Element::T> *ecs = (LA::Vector6<Element::T> *) (_eps.Data() + ipc);
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
    LA::Vector9<Element::T> *em = (LA::Vector9<Element::T> *) (_eps.Data() + ipm);
    em->Set(eps + 6);
#endif
  }
  GetPriorEquation(&S);
  const int rank = S.RankLDL(&_work, eps ? _eps.Data() : NULL);
//#ifdef CAMERA_PRIOR_DEBUG_EIGEN
#if 0
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  const int e_rank = EigenRankLU(e_Ap.m_A);
  UT_ASSERT(e_rank == rank);
#endif
  return rank == Np;
}

bool Joint::PropagateLF(const Rigid3D &Tr, const Camera &C,
                        const IMU::Delta::Factor::Auxiliary::RelativeLF &A,
                        AlignedVector<float> *work, const float *eps) {
  const int N = static_cast<int>(m_iKFs.size()), Nk = N - 1;
#ifdef CFG_DEBUG
  UT_ASSERT(m_Zps.Size() != N);
  UT_ASSERT(m_iKFs[Nk] == INT_MAX);
#endif
  SetPose(Tr, Nk, C.m_T);
  SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);

#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  EigenPrior e_Ap;
  e_Ap.Set(*this, true);
  IMU::Delta::EigenFactor::RelativeLF e_A;
  e_A.Set(A, 0.0f);
  e_Ap.PropagateLF(e_A);
#endif
  Matrix::X _A;
  Vector::X _b, _work, _eps;
  const int Npgk = 2 + Nk * 6, Npcm = 15, Np1 = Npgk + Npcm, Np2 = Np1 + Npcm;
  work->Resize((_work.BindSize(Np2) + _A.BindSize(Np2, Np2, true) + _b.BindSize(Np2) +
               (eps ? _eps.BindSize(Npcm) : 0)) / sizeof(float));
  _work.Bind(work->Data(), Np2);
  _A.Bind(_work.BindNext(), Np2, Np2, true);
  _b.Bind(_A.BindNext(), Np2);
  if (eps) {
    _eps.Bind(_b.BindNext(), Npcm);
    _eps.Set(eps);
  }
  GetPriorEquation(&_A, &_b);
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr1 = N * 3, Npr2 = Npr1 + 3, ipr2 = Npr1, ipr1 = ipr2 - 3;
  _A.InsertZero(ipr2, 3, work);
  _b.InsertZero(ipr2, 3, work);
  const int Npp1 = Npr1, Npp2 = Npr2, ipp2 = Npr2 + Npp1, ipp1 = ipp2 - 3;
  _A.InsertZero(ipp2, 3, work);
  _b.InsertZero(ipp2, 3, work);
  const int ipbw1 = Npr2 + Npp2, ipbw2 = ipbw1 + 3;
  _A.InsertZero(ipbw2, 3, work);
  _b.InsertZero(ipbw2, 3, work);
  const int ipv1 = ipbw2 + 3, ipv2 = ipv1 + 3;
  _A.InsertZero(ipv2, 3, work);
  _b.InsertZero(ipv2, 3, work);
  const int ipg = ipv2 + 3, ipba1 = ipg + 2, ipba2 = ipba1 + 3;
  _A.InsertZero(ipba2, 3, work);
  _b.InsertZero(ipba2, 3, work);
  const int ipcs[10] = {ipp1, ipr1, ipv1, ipba1, ipbw1, ipp2, ipr2, ipv2, ipba2, ipbw2};
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  LA::Matrix3x2f Acg;
  for (int i = 0; i < 10; ++i) {
    const int ipc = ipcs[i];
    if (ipg < ipc) {
      _A.IncreaseBlock(ipg, ipc, A.m_Agc[i]);
    } else {
      A.m_Agc[i].GetTranspose(Acg);
      _A.IncreaseBlock(ipc, ipg, Acg);
    }
    _b.IncreaseBlock(ipc, A.m_b[i]);
  }
  LA::AlignedMatrix3x3f Acc;
  for (int i = 0, k = 0; i < 10; ++i) {
    const int ip = ipcs[i];
    for (int j = i; j < 10; ++j, ++k) {
      const int jp = ipcs[j];
      if (ip <= jp) {
        _A.IncreaseBlock(ip, jp, A.m_A[k]);
      } else {
        A.m_A[k].GetTranspose(Acc);
        _A.IncreaseBlock(jp, ip, Acc);
      }
    }
  }
  bool scc = true;
  scc = _A.MarginalizeLDL(ipr1, 3, _b, &_work, eps ? _eps.Data() + 3 : NULL) && scc;
  scc = _A.MarginalizeLDL(ipp1 - 3, 3, _b, &_work, eps ? _eps.Data() : NULL) && scc;
  scc = _A.MarginalizeLDL(ipbw1 - 6, 3, _b, &_work, eps ? _eps.Data() + 12 : NULL) && scc;
  scc = _A.MarginalizeLDL(ipv1 - 9, 3, _b, &_work, eps ? _eps.Data() + 6 : NULL) && scc;
  scc = _A.MarginalizeLDL(ipba1 - 12, 3, _b, &_work, eps ? _eps.Data() + 9 : NULL) && scc;
  //UT::DebugStart();
  //UT::Print("%.10e\n", _A[ipr1 + 1][ipr1 + 1]);
  ////UT::Print("%.10e\n", _A[ipr2 + 1][ipr2 + 1]);
  ////const bool scc1 = _A.MarginalizeLDL(ipr1, 3, _b, &_work, eps ? _eps.Data() + 3 : NULL);
  ////UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc11 = _A.MarginalizeLDL(ipr1, 1, _b, &_work, eps ? _eps.Data() + 3 : NULL);
  //UT::Print("%.10e\n", _A[ipr1][ipr1]);
  ////UT::Print("%.10e\n", _A[ipr2][ipr2]);
  //const bool scc12 = _A.MarginalizeLDL(ipr1, 1, _b, &_work, eps ? _eps.Data() + 3 : NULL);
  //UT::Print("%.10e\n", _A[ipr2 - 1][ipr2 - 1]);
  //const bool scc13 = _A.MarginalizeLDL(ipr1, 1, _b, &_work, eps ? _eps.Data() + 3 : NULL);
  //UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc1 = scc11 && scc12 && scc13;
  //const bool scc2 = _A.MarginalizeLDL(ipp1 - 3, 3, _b, &_work, eps ? _eps.Data() : NULL);
  ////UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc3 = _A.MarginalizeLDL(ipbw1 - 6, 3, _b, &_work, eps ? _eps.Data() + 12 : NULL);
  ////UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc4 = _A.MarginalizeLDL(ipv1 - 9, 3, _b, &_work, eps ? _eps.Data() + 6 : NULL);
  ////UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc5 = _A.MarginalizeLDL(ipba1 - 12, 3, _b, &_work, eps ? _eps.Data() + 9 : NULL);
  ////UT::Print("%.10e\n", _A[ipr2 - 2][ipr2 - 2]);
  //const bool scc = scc1 && scc2 && scc3 && scc4 && scc5;
  //UT::DebugStop();
#else
  const int ipg = 0, ipc1 = Npgk, ipc2 = ipc1 + Npcm;
#ifdef CFG_DEBUG
  UT_ASSERT(ipc2 == Np1);
#endif
  _A.InsertZero(ipc2, Npcm, NULL);
  _b.InsertZero(ipc2, Npcm, NULL);
  //_A.Resize(Np2, Np2, true, true);
  //_b.Resize(Np2, true);
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  for (int i = 0, ip = ipc1; i < 10; ++i, ip += 3) {
    _A.IncreaseBlock(0, ip, A.m_Agc[i]);
    _b.IncreaseBlock(ip, A.m_b[i]);
  }
  for (int i = 0, ip = ipc1, k = 0; i < 10; ++i, ip += 3) {
    for (int j = i, jp = ip; j < 10; ++j, jp += 3, ++k) {
      _A.IncreaseBlock(ip, jp, A.m_A[k]);
    }
  }
  //UT::PrintSeparator();
  //_b.Print(true);
  const bool scc = _A.MarginalizeLDL(Npgk, Npcm, _b, &_work, eps ? _eps.Data() : NULL);
  //UT::PrintSeparator();
  //_b.Print(true);
#endif
  SetPriorEquation(_A, _b);
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  e_Ap.AssertEqual(*this, 2, "[Joint::PropagateLF]");
#endif
  return scc;
#else
  const int Nx2 = N * 2;
  m_Acc.InsertZero(N, 1, work);
  m_Acm.InsertZero(N, Nx2 + 1, work);
  m_Arc.InsertZero(N, 2, work);
  m_bc.InsertZero(N, 1, work);

  Vector::CM Acm1(m_Acm.Data(), N, false);
  Vector::CM Acm2(Acm1.End(), N, false);
  Vector::CM Mcm(Acm2.End() + 1, N, false);
  work->Resize((sizeof(Element::MC) * 3 + sizeof(Element::MM) * 3 + sizeof(Element::RM) * 2 +
                sizeof(Element::CC) * N) / sizeof(float));
  Vector::MC Amc(work->Data(), 3, false);
  Vector::MM Amm(Amc.End(), 3, false);
  Vector::RM Arm(Amm.End(), 2, false);
  Vector::CC Acc(Arm.End(), N, false);

#ifdef CFG_IMU_FULL_COVARIANCE
  Element::CC &Ac1c1 = m_Acc[Nk][Nk];
  Ac1c1.Increase(A.m_A, A.m_A + 9);
  Element::CM &Ac1m1 = Acm1[Nk], &dAc1m1 = m_Acm[Nx2];
  dAc1m1.Set(A.m_A + 2, A.m_A + 11);
  Ac1m1 += dAc1m1;
  Element::CC &Ac1c2 = m_Acc[Nk][N];
  Ac1c2.Set(A.m_A + 5, A.m_A + 14);
  Element::CM &Ac1m2 = m_Acm[Nx2];
  Ac1m2.Set(A.m_A + 7, A.m_A + 16);
  Element::MC &Am2c1 = Amc[0];
  Ac1m2.GetTranspose(Am2c1);
  Element::C &bc1 = m_bc[Nk];
  bc1 += Element::C(A.m_b[0], A.m_b[1]);

  Element::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_A + 19, A.m_A + 26, A.m_A + 32, true);
  Element::MC &Am1c2 = Amc[1];
  Am1c2.Set(A.m_A + 22, A.m_A + 29, A.m_A + 35);
  Element::CM &Ac2m1 = m_Acm[Nx2];
  Am1c2.GetTranspose(Ac2m1);
  Element::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_A + 24, A.m_A + 31, A.m_A + 37);
  Am2m1.Transpose();
  Element::M &bm1 = m_bm;
  bm1 += Element::M(A.m_b[2], A.m_b[3], A.m_b[4]);

  Element::CC &Ac2c2 = m_Acc[N][N];
  Ac2c2.Set(A.m_A + 40, A.m_A + 44);
  Element::CM &Ac2m2 = Acm2[Nk];
  Ac2m2.Set(A.m_A + 42, A.m_A + 46);
  Element::C &bc2 = m_bc[N];
  bc2.Set(A.m_b[5], A.m_b[6]);

  Element::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_A + 49, A.m_A + 51, A.m_A + 52);
  Element::M bm2(A.m_b[7], A.m_b[8], A.m_b[9]);

  m_Arr += A.m_Agg;
  Vector::RC Arck = m_Arc.GetBlock(Nk);
  Element::RC &Arc1 = m_Arc[Nk], &dArc1 = m_Arc[N];
  dArc1.Set(A.m_Agc[0], A.m_Agc[1]);
  Arc1 += dArc1;
  Element::RM &Arm1 = m_Arm, &dArm1 = Arm[0];
  dArm1.Set(A.m_Agc[2], A.m_Agc[3], A.m_Agc[4]);
  Arm1 += dArm1;
  Element::RC &Arc2 = m_Arc[N];
  Arc2.Set(A.m_Agc[5], A.m_Agc[6]);
  Element::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agc[7], A.m_Agc[8], A.m_Agc[9]);
  m_br += A.m_bg;
#else
  Element::CC &Ac1c1 = m_Acc[Nk][Nk];
  Ac1c1.Increase(A.m_Ap1p1, A.m_Ap1r1, A.m_Ar1r1);
  Element::CM &Ac1m1 = Acm1[Nk], &dAc1m1 = m_Acm[Nx2];
  dAc1m1.Set(A.m_Ap1v1, A.m_Ap1ba1, A.m_Ap1bw1, A.m_Ar1v1, A.m_Ar1ba1, A.m_Ar1bw1);
  Ac1m1 += dAc1m1;
  Element::CC &Ac1c2 = m_Acc[Nk][N];
  Ac1c2.Set(A.m_Ap1p2, A.m_Ap1r2, A.m_Ar1p2, A.m_Ar1r2);
  //Element::CM &Ac1m2 = m_Acm[Nx2];
  //Ac1m2.Set(A.m_Ar1v2);
  Element::MC &Am2c1 = Amc[0];
  Am2c1.Set(A.m_Ar1v2.GetTranspose());
  Element::C &bc1 = m_bc[Nk];
  bc1 += Element::C(A.m_bp1, A.m_br1);

  Element::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1, A.m_Abw1bw1);
  Element::MC &Am1c2 = Amc[1];
  Am1c2.Set(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2, A.m_Abw1r2);
  Element::CM &Ac2m1 = m_Acm[Nx2];
  Am1c2.GetTranspose(Ac2m1);
  Element::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Av1v2, A.m_Aba1v2, A.m_Aba1ba2, A.m_Abw1v2, A.m_Abw1bw2);
  Am2m1.Transpose();
  Element::M &bm1 = m_bm;
  bm1 += Element::M(A.m_bv1, A.m_bba1, A.m_bbw1);

  Element::CC &Ac2c2 = m_Acc[N][N];
  Ac2c2.Set(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
  Element::CM &Ac2m2 = Acm2[Nk];
  Ac2m2.Set(A.m_Ar2v2);
  Element::C &bc2 = m_bc[N];
  bc2.Set(A.m_bp2, A.m_br2);

  Element::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Av2v2, A.m_Aba2ba2, A.m_Abw2bw2);
  Element::M bm2(A.m_bv2, A.m_bba2, A.m_bbw2);

  m_Arr += A.m_Agg;
  Vector::RC Arck = m_Arc.GetBlock(Nk);
  Element::RC &Arc1 = m_Arc[Nk], &dArc1 = m_Arc[N];
  dArc1.Set(A.m_Agp1, A.m_Agr1);
  Arc1 += dArc1;
  Element::RM &Arm1 = m_Arm, &dArm1 = Arm[0];
  dArm1.Set(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
  Arm1 += dArm1;
  Element::RC &Arc2 = m_Arc[N];
  Arc2.Set(A.m_Agp2, A.m_Agr2);
  Element::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agv2);
  m_br += A.m_bg;
#endif

  Matrix::CC Ackck = m_Acc.GetBlock(Nk, Nk);
  Matrix::CC Ackc1 = m_Acc.GetColumn(Nk, Nk);
  Vector::CM Ackm1 = Acm1.GetBlock(Nk);
  Matrix::CC Ackc2 = m_Acc.GetColumn(N, Nk);
  Vector::CM Ackm2 = Acm2.GetBlock(Nk);
  Vector::C bck = m_bc.GetBlock(Nk);

//#ifdef CFG_DEBUG
#if 0
  UT::Print("%.10e\n", Ac2c2[3][3]);
#endif
  bool scc = true;
  Element::MM &Mm1m1 = Am1m1;
  if (Mm1m1.InverseLDL(eps ? eps + 6 : NULL)) {
    Mm1m1.MakeMinus();
    Element::RM &Mrm1 = Arm[1];
    Element::ABT(Arm1, Mm1m1, Mrm1);
    Element::AddABTToUpper(Mrm1, Arm1, m_Arr);
    Vector::AddABTTo(Mrm1, Ackm1, Arck);
    Element::AddABTTo(Mrm1, Ac1m1, Arc1);
    Element::AddABTTo(Mrm1, Ac2m1, Arc2);
    Element::AddABTTo(Mrm1, Am2m1, Arm2);
    Element::AddAbTo(Mrm1, bm1, m_br);
    Vector::CM Mckm1 = Mcm.GetBlock(Nk);
    Vector::ABT(Ackm1, Mm1m1, Mckm1);
    Matrix::AddABTToUpper(Mckm1, Ackm1, Ackck);
    Vector::AddABTTo(Mckm1, Ac1m1, Ackc1);
    Vector::AddABTTo(Mckm1, Ac2m1, Ackc2);
    Vector::AddABTTo(Mckm1, Am2m1, Ackm2);
    Vector::AddAbTo(Mckm1, bm1, bck);
    Element::CM &Mc1m1 = Mcm[Nk];
    Element::ABT(Ac1m1, Mm1m1, Mc1m1);
    Element::AddABTToUpper(Mc1m1, Ac1m1, Ac1c1);
    Element::AddABTTo(Mc1m1, Ac2m1, Ac1c2);
    //Element::AddABTTo(Mc1m1, Am2m1, Ac1m2);
    Element::AddABTTo(Am2m1, Mc1m1, Am2c1);
    Element::AddAbTo(Mc1m1, bm1, bc1);
    Element::CM &Mc2m1 = Mcm[Nk];
    Element::ABT(Ac2m1, Mm1m1, Mc2m1);
    Element::AddABTToUpper(Mc2m1, Ac2m1, Ac2c2);
//#ifdef CFG_DEBUG
#if 0
    UT::Print("%.10e\n", Ac2c2[3][3]);
#endif
    Element::AddABTTo(Mc2m1, Am2m1, Ac2m2);
    Element::AddAbTo(Mc2m1, bm1, bc2);
    Element::MM &Mm2m1 = Amm[2];
    Element::ABT(Am2m1, Mm1m1, Mm2m1);
    Element::AddABTToUpper(Mm2m1, Am2m1, Am2m2);
    Element::AddAbTo(Mm2m1, bm1, bm2);
  } else {
    scc = false;
  }

  Element::CC &Mc1c1 = Ac1c1;
  if (Mc1c1.InverseLDL(eps)) {
    Mc1c1.MakeMinus();
    Element::CC &Ac2c1 = Ac1c2;
    Ac2c1.Transpose();
    Element::RC &Mrc1 = m_Arc[N + 1];
    Element::ABT(Arc1, Mc1c1, Mrc1);
    Element::AddABTToUpper(Mrc1, Arc1, m_Arr);
    Vector::AddABTTo(Mrc1, Ackc1, Arck);
    Element::AddABTTo(Mrc1, Ac2c1, Arc2);
    Element::AddABTTo(Mrc1, Am2c1, Arm2);
    Element::AddAbTo(Mrc1, bc1, m_br);
    Vector::CC Mckc1 = Acc.GetBlock(Nk);
    Vector::ABT(Ackc1, Mc1c1, Mckc1);
    Matrix::AddABTToUpper(Mckc1, Ackc1, Ackck);
    Vector::AddABTTo(Mckc1, Ac2c1, Ackc2);
    Vector::AddABTTo(Mckc1, Am2c1, Ackm2);
    Vector::AddAbTo(Mckc1, bc1, bck);
    Element::CC &Mc2c1 = Acc[Nk];
    Element::ABT(Ac2c1, Mc1c1, Mc2c1);
    Element::AddABTToUpper(Mc2c1, Ac2c1, Ac2c2);
//#ifdef CFG_DEBUG
#if 0
    UT::Print("%.10e\n", Ac2c2[3][3]);
#endif
    Element::AddABTTo(Mc2c1, Am2c1, Ac2m2);
    Element::AddAbTo(Mc2c1, bc1, bc2);
    Element::MC &Mm2c1 = Amc[2];
    Element::ABT(Am2c1, Mc1c1, Mm2c1);
    Element::AddABTToUpper(Mm2c1, Am2c1, Am2m2);
    Element::AddAbTo(Mm2c1, bc1, bm2);
  } else {
    scc = false;
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

  return scc;
#endif
}

bool Joint::PropagateLF(const IMU::Delta::Factor::Auxiliary::RelativeLF &A, LA::AlignedVectorXf *x,
                        AlignedVector<float> *work, const float *eps) const {
  const int N1 = static_cast<int>(m_iKFs.size()), N2 = N1 + 1, Nk = N1 - 1;
#ifdef CFG_DEBUG
  UT_ASSERT(m_Zps.Size() != N1);
  UT_ASSERT(m_iKFs[Nk] == INT_MAX);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  EigenPrior e_Ap;
  e_Ap.Set(*this, true);
  IMU::Delta::EigenFactor::RelativeLF e_A;
  e_A.Set(A, 0.0f);
  EigenVectorXf e_x;
  e_Ap.PropagateLF(e_A, &e_x);
#endif
  Matrix::X _A;
  Vector::X _b, _eps;
  const int Npgk = 2 + Nk * 6, Npcm = 15, Np1 = Npgk + Npcm, Np2 = Np1 + Npcm;
  work->Resize((x->BindSize(Np2) + _A.BindSize(Np2, Np2) + _b.BindSize(Np2) +
               (eps ? _eps.BindSize(Np2) : 0)) / sizeof(float));
  x->Bind(work->Data(), Np2);
  _A.Bind(x->BindNext(), Np2, Np2);
  _b.Bind(_A.BindNext(), Np2);
  if (eps) {
    _eps.Bind(_b.BindNext(), Np2);
  }
  GetPriorEquation(&_A, &_b, false);
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr1 = N1 * 3, Npr2 = Npr1 + 3, ipr2 = Npr1, ipr1 = ipr2 - 3;
  _A.InsertZero(ipr2, 3, work);
  _b.InsertZero(ipr2, 3, work);
  const int Npp1 = Npr1, Npp2 = Npr2, ipp2 = Npr2 + Npp1, ipp1 = ipp2 - 3;
  _A.InsertZero(ipp2, 3, work);
  _b.InsertZero(ipp2, 3, work);
  const int ipbw1 = Npr2 + Npp2, ipbw2 = ipbw1 + 3;
  _A.InsertZero(ipbw2, 3, work);
  _b.InsertZero(ipbw2, 3, work);
  const int ipv1 = ipbw2 + 3, ipv2 = ipv1 + 3;
  _A.InsertZero(ipv2, 3, work);
  _b.InsertZero(ipv2, 3, work);
  const int ipg = ipv2 + 3, ipba1 = ipg + 2, ipba2 = ipba1 + 3;
  _A.InsertZero(ipba2, 3, work);
  _b.InsertZero(ipba2, 3, work);
  const int ipcs[10] = {ipp1, ipr1, ipv1, ipba1, ipbw1, ipp2, ipr2, ipv2, ipba2, ipbw2};
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  LA::Matrix3x2f Acg;
  for (int i = 0; i < 10; ++i) {
    const int ipc = ipcs[i];
    if (ipg < ipc) {
      _A.IncreaseBlock(ipg, ipc, A.m_Agc[i]);
    } else {
      A.m_Agc[i].GetTranspose(Acg);
      _A.IncreaseBlock(ipc, ipg, Acg);
    }
    _b.IncreaseBlock(ipc, A.m_b[i]);
  }
  LA::AlignedMatrix3x3f Acc;
  for (int i = 0, k = 0; i < 10; ++i) {
    const int ip = ipcs[i];
    for (int j = i; j < 10; ++j, ++k) {
      const int jp = ipcs[j];
      if (ip <= jp) {
        _A.IncreaseBlock(ip, jp, A.m_A[k]);
      } else {
        A.m_A[k].GetTranspose(Acc);
        _A.IncreaseBlock(jp, ip, Acc);
      }
    }
  }
#else
  const int ipg = 0, ipc1 = Npgk, ipc2 = ipc1 + Npcm;
#ifdef CFG_DEBUG
  UT_ASSERT(ipc2 == Np1);
#endif
  _A.InsertZero(ipc2, Npcm, NULL);
  _b.InsertZero(ipc2, Npcm, NULL);
  //_A.Resize(Np2, Np2, true, true);
  //_b.Resize(Np2, true);
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  for (int i = 0, ip = ipc1; i < 10; ++i, ip += 3) {
    _A.IncreaseBlock(0, ip, A.m_Agc[i]);
    _b.IncreaseBlock(ip, A.m_b[i]);
  }
  for (int i = 0, ip = ipc1, k = 0; i < 10; ++i, ip += 3) {
    for (int j = i, jp = ip; j < 10; ++j, jp += 3, ++k) {
      _A.IncreaseBlock(ip, jp, A.m_A[k]);
    }
  }
#endif
  if (eps) {
    LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg);
    //eg->MakeZero();
    eg->Set(eps + 3);
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *ecrs = (LA::Vector3<Element::T> *) _eps.Data();
    LA::Vector3<Element::T> *ecps = (LA::Vector3<Element::T> *) (_eps.Data() + Npr2);
    for (int i = 0; i < N2; ++i) {
      ecrs[i].Set(eps + 3);
      ecps[i].Set(eps);
    }
    LA::Vector3<Element::T> *ev = (LA::Vector3<Element::T> *) (_eps.Data() + ipv1);
    LA::Vector3<Element::T> *eba = (LA::Vector3<Element::T> *) (_eps.Data() + ipba1);
    LA::Vector3<Element::T> *ebw = (LA::Vector3<Element::T> *) (_eps.Data() + ipbw1);
    for (int i = 0; i < 2; ++i) {
      ev[i].Set(eps + 6);
      eba[i].Set(eps + 9);
      ebw[i].Set(eps + 12);
    }
#else
    LA::Vector6<Element::T> *eks = (LA::Vector6<Element::T> *) (eg + 1);
    for (int i = 0; i < Nk; ++i) {
      eks[i].Set(eps);
    }
    LA::Vector6<Element::T> *ec1 = eks + Nk;
    LA::Vector9<Element::T> *em1 = (LA::Vector9<Element::T> *) (ec1 + 1);
    LA::Vector6<Element::T> *ec2 = (LA::Vector6<Element::T> *) (em1 + 1);
    LA::Vector9<Element::T> *em2 = (LA::Vector9<Element::T> *) (ec2 + 1);
    ec1->Set(eps);
    ec2->Set(eps);
    em1->Set(eps + 6);
    em2->Set(eps + 6);
#endif
  }
//#ifdef CFG_DEBUG
#if 0
  Matrix::X AA = _A;
  Vector::X r = _b;
#endif
  if (!_A.SolveLDL(_b, eps ? _eps.Data() : NULL)) {
    x->Resize(0);
    return false;
  }
  _b.MakeMinus();
//#ifdef CFG_DEBUG
#if 0
  AA.SetLowerFromUpper();
  Matrix::X::AddAbTo(AA, _b, r.Data());
  r.Print();
#endif
#ifdef CFG_CAMERA_PRIOR_REORDER
  LA::Vector2f xg;
  _b.GetBlock(ipg, xg);
  x->SetBlock(0, xg);
  LA::Vector3f xk;
  for (int i = 0, jpr1 = 0, jpp1 = Npr2, jpp2 = 2, jpr2 = jpp2 + 3; i < Nk;
    ++i, jpr1 += 3, jpp1 += 3, jpp2 += 6, jpr2 += 6) {
    _b.GetBlock(jpr1, xk);  x->SetBlock(jpr2, xk);
    _b.GetBlock(jpp1, xk);  x->SetBlock(jpp2, xk);
  }
  LA::Vector3f *xcs = (LA::Vector3f *) (x->Data() + Npgk);
  for (int i = 0; i < 10; ++i) {
    _b.GetBlock(ipcs[i], xcs[i]);
  }
#else
  //x->Copy(_b);
  _b.GetBlock(0, *x);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  e_x.AssertEqual(*x);
#endif
  return true;
}

bool Joint::PropagateKF(const Rigid3D &Tr, const Camera &C,
                        const IMU::Delta::Factor::Auxiliary::RelativeKF &A,
                        AlignedVector<float> *work, const float *eps) {
#ifdef CFG_DEBUG
  UT_ASSERT(m_iKFs.empty());
  //UT_ASSERT(m_Zps.Empty());
  UT_ASSERT(m_Zps.Size() == 1);
#endif
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  EigenPrior e_Ap;
  e_Ap.Set(*this, true);
  IMU::Delta::EigenFactor::RelativeKF e_A;
  e_A.Set(A, 0.0f);
  e_Ap.PropagateKF(e_A);
#endif
  Matrix::X _A;
  Vector::X _b, _work, _eps;
  const int Npg = 2, Npc = 6, Npm = 9, Np1 = Npg + Npm, Npcm = Npc + Npm, Np2 = Np1 + Npcm;
  work->Resize((_work.BindSize(Np2) + _A.BindSize(Np2, Np2, true) + _b.BindSize(Np2) +
               (eps ? _eps.BindSize(Npm) : 0)) / sizeof(float));
  _work.Bind(work->Data(), Np2);
  _A.Bind(_work.BindNext(), Np2, Np2, true);
  _b.Bind(_A.BindNext(), Np2);
  if (eps) {
    _eps.Bind(_b.BindNext(), Npm);
    _eps.Set(eps + Npc);
  }
  GetPriorEquation(&_A, &_b);
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int ipr2 = 0;
  _A.InsertZero(ipr2, 3, work);
  _b.InsertZero(ipr2, 3, work);
  const int ipp2 = 3;
  _A.InsertZero(ipp2, 3, work);
  _b.InsertZero(ipp2, 3, work);
  const int ipbw1 = 6, ipbw2 = ipbw1 + 3;
  _A.InsertZero(ipbw2, 3, work);
  _b.InsertZero(ipbw2, 3, work);
  const int ipv1 = ipbw2 + 3, ipv2 = ipv1 + 3;
  _A.InsertZero(ipv2, 3, work);
  _b.InsertZero(ipv2, 3, work);
  const int ipg = ipv2 + 3, ipba1 = ipg + 2, ipba2 = ipba1 + 3;
  _A.InsertZero(ipba2, 3, work);
  _b.InsertZero(ipba2, 3, work);
  const int ipcs[8] = {ipv1, ipba1, ipbw1, ipp2, ipr2, ipv2, ipba2, ipbw2};
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  LA::Matrix3x2f Acg;
  for (int i = 0; i < 8; ++i) {
    const int ipc = ipcs[i];
    if (ipg < ipc) {
      _A.IncreaseBlock(ipg, ipc, A.m_Agc[i]);
    } else {
      A.m_Agc[i].GetTranspose(Acg);
      _A.IncreaseBlock(ipc, ipg, Acg);
    }
    _b.IncreaseBlock(ipc, A.m_bc[i]);
  }
  LA::AlignedMatrix3x3f Acc;
  for (int i = 0, k = 0; i < 8; ++i) {
    const int ip = ipcs[i];
    for (int j = i; j < 8; ++j, ++k) {
      const int jp = ipcs[j];
      if (ip <= jp) {
        _A.IncreaseBlock(ip, jp, A.m_Ac[k]);
      } else {
        A.m_Ac[k].GetTranspose(Acc);
        _A.IncreaseBlock(jp, ip, Acc);
      }
    }
  }
  bool scc = true;
  scc = _A.MarginalizeLDL(ipbw1, 3, _b, &_work, eps ? _eps.Data() + 6 : NULL) && scc;
  scc = _A.MarginalizeLDL(ipv1 - 3, 3, _b, &_work, eps ? _eps.Data() : NULL) && scc;
  scc = _A.MarginalizeLDL(ipba1 - 6, 3, _b, &_work, eps ? _eps.Data() + 3 : NULL) && scc;
  //UT::DebugStart();
  //UT::Print("%f\n", _A[ipr2][ipg]);
  //const bool scc1 = _A.MarginalizeLDL(ipbw1, 3, _b, &_work, eps ? _eps.Data() + 6 : NULL);
  //UT::Print("%f\n", _A[ipr2][ipg - 3]);
  //const bool scc2 = _A.MarginalizeLDL(ipv1 - 3, 3, _b, &_work, eps ? _eps.Data() : NULL);
  //UT::Print("%f\n", _A[ipr2][ipg - 6]);
  //const bool scc3 = _A.MarginalizeLDL(ipba1 - 6, 3, _b, &_work, eps ? _eps.Data() + 3 : NULL);
  //UT::Print("%f\n", _A[ipr2][ipg - 6]);
  //const bool scc = scc1 && scc2 && scc3;
  //UT::DebugStop();
#else
  const int ipg = 0, ipc1 = Npg, ipc2 = ipc1 + Npm;
#ifdef CFG_DEBUG
  UT_ASSERT(ipc2 == Np1);
#endif
  _A.InsertZero(ipc2, Npcm, NULL);
  _b.InsertZero(ipc2, Npcm, NULL);
  //_A.Resize(Np2, Np2, true, true);
  //_b.Resize(Np2, true);
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  for (int i = 0, ip = ipc1; i < 8; ++i, ip += 3) {
    _A.IncreaseBlock(0, ip, A.m_Agc[i]);
    _b.IncreaseBlock(ip, A.m_bc[i]);
  }
  for (int i = 0, ip = ipc1, k = 0; i < 8; ++i, ip += 3) {
    for (int j = i, jp = ip; j < 8; ++j, jp += 3, ++k) {
      _A.IncreaseBlock(ip, jp, A.m_Ac[k]);
    }
  }
  const bool scc = _A.MarginalizeLDL(Npg, Npm, _b, &_work, eps ? _eps.Data() : NULL);
#endif
  m_iKFs.resize(1, INT_MAX);
  //m_Zps.Resize(1);
  const Rotation3D RrT = m_Zps.Back();
  m_Zps.Resize(2);
  m_Zps.Back() = RrT;
  SetPose(Tr, 0, C.m_T);
  SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);
  SetPriorEquation(_A, _b);
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  e_Ap.AssertEqual(*this, 1, "[Joint::PropagateKF]");
#endif
  return scc;
#else
  m_iKFs.resize(1, INT_MAX);
  //m_Zps.Resize(1);
  const Rotation3D RrT = m_Zps.Back();
  m_Zps.Resize(2);
  m_Zps.Back() = RrT;
  SetPose(Tr, 0, C.m_T);
  SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);
  m_Acc.Resize(1, 1, true);
  m_Acm.Resize(3);
  m_Arc.Resize(1);
  m_bc.Resize(1);

  work->Resize((sizeof(Element::MC) + sizeof(Element::MM) * 3 + sizeof(Element::RM) * 2) /
                sizeof(float));
  Vector::MC Amc(work->Data(), 1, false);
  Vector::MM Amm(Amc.End(), 3, false);
  Vector::RM Arm(Amm.End(), 2, false);

#ifdef CFG_IMU_FULL_COVARIANCE
  Element::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Ac, A.m_Ac + 7, A.m_Ac + 13, true);
  Element::MC &Am1c2 = Amc[0];
  Am1c2.Set(A.m_Ac + 3, A.m_Ac + 10, A.m_Ac + 16);
  Element::CM &Ac2m1 = m_Acm[1];
  Am1c2.GetTranspose(Ac2m1);
  Element::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Ac + 5, A.m_Ac + 12, A.m_Ac + 18);
  Am2m1.Transpose();
  Element::M &bm1 = m_bm;
  bm1 += Element::M(A.m_bc[0], A.m_bc[1], A.m_bc[2]);

  Element::CC &Ac2c2 = m_Acc[0][0];
  Ac2c2.Set(A.m_Ac + 21, A.m_Ac + 25);
  Element::CM &Ac2m2 = m_Acm[0];
  Ac2m2.Set(A.m_Ac + 23, A.m_Ac + 27);
  Element::C &bc2 = m_bc[0];
  bc2.Set(A.m_bc[3], A.m_bc[4]);

  Element::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Ac + 30, A.m_Ac + 32, A.m_Ac + 33);
  Element::M bm2(A.m_bc[5], A.m_bc[6], A.m_bc[7]);

  //m_Arr = A.m_Agg;
  m_Arr += A.m_Agg;
  Element::RM &Arm1 = m_Arm;
  Arm1.Set(A.m_Agc[0], A.m_Agc[1], A.m_Agc[2]);
  Element::RC &Arc2 = m_Arc[0];
  Arc2.Set(A.m_Agc[3], A.m_Agc[4]);
  Element::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agc[5], A.m_Agc[6], A.m_Agc[7]);
  //m_br = A.m_bg;
  m_br += A.m_bg;
#else
  Element::MM &Am1m1 = m_Amm;
  Am1m1.Increase(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1, A.m_Abw1bw1);
  Element::MC &Am1c2 = Amc[0];
  Am1c2.Set(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2, A.m_Abw1r2);
  Element::CM &Ac2m1 = m_Acm[1];
  Am1c2.GetTranspose(Ac2m1);
  Element::MM &Am2m1 = Amm[0];
  Am2m1.Set(A.m_Av1v2, A.m_Aba1v2, A.m_Aba1ba2, A.m_Abw1v2, A.m_Abw1bw2);
  Am2m1.Transpose();
  Element::M &bm1 = m_bm;
  bm1 += Element::M(A.m_bv1, A.m_bba1, A.m_bbw1);

  Element::CC &Ac2c2 = m_Acc[0][0];
  Ac2c2.Set(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
  Element::CM &Ac2m2 = m_Acm[0];
  Ac2m2.Set(A.m_Ar2v2);
  Element::C &bc2 = m_bc[0];
  bc2.Set(A.m_bp2, A.m_br2);

  Element::MM &Am2m2 = Amm[1];
  Am2m2.Set(A.m_Av2v2, A.m_Aba2ba2, A.m_Abw2bw2);
  Element::M bm2(A.m_bv2, A.m_bba2, A.m_bbw2);

  //m_Arr = A.m_Agg;
  m_Arr += A.m_Agg;
  Element::RM &Arm1 = m_Arm;
  Arm1.Set(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
  Element::RC &Arc2 = m_Arc[0];
  Arc2.Set(A.m_Agp2, A.m_Agr2);
  Element::RM &Arm2 = Arm[0];
  Arm2.Set(A.m_Agv2);
  //m_br = A.m_bg;
  m_br += A.m_bg;
#endif

  bool scc = true;
  Element::MM &Mm1m1 = Am1m1;
  if (Mm1m1.InverseLDL(eps ? eps + 6 : NULL)) {
    Mm1m1.MakeMinus();
    Element::RM &Mrm1 = Arm[1];
    Element::ABT(Arm1, Mm1m1, Mrm1);
    Element::AddABTToUpper(Mrm1, Arm1, m_Arr);
    Element::AddABTTo(Mrm1, Ac2m1, Arc2);
    Element::AddABTTo(Mrm1, Am2m1, Arm2);
    Element::AddAbTo(Mrm1, bm1, m_br);
    Element::CM &Mc2m1 = m_Acm[2];
    Element::ABT(Ac2m1, Mm1m1, Mc2m1);
    Element::AddABTToUpper(Mc2m1, Ac2m1, Ac2c2);
    Element::AddABTTo(Mc2m1, Am2m1, Ac2m2);
    Element::AddAbTo(Mc2m1, bm1, bc2);
    Element::MM &Mm2m1 = Amm[2];
    Element::ABT(Am2m1, Mm1m1, Mm2m1);
    Element::AddABTToUpper(Mm2m1, Am2m1, Am2m2);
    Element::AddAbTo(Mm2m1, bm1, bm2);
  } else {
    scc = false;
  }

  Arm1 = Arm2;
  Ac2c2.SetLowerFromUpper();
  Am1m1 = Am2m2;
  Am1m1.SetLowerFromUpper();
  bm1 = bm2;
  m_Acm.Resize(1);
  return scc;
#endif
}

bool Joint::PropagateKF(const IMU::Delta::Factor::Auxiliary::RelativeKF &A, LA::AlignedVectorXf *x,
                        AlignedVector<float> *work, const float *eps) const {
#ifdef CFG_DEBUG
  UT_ASSERT(m_iKFs.empty());
  //UT_ASSERT(m_Zps.Empty());
  UT_ASSERT(m_Zps.Size() == 1);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  EigenPrior e_Ap;
  e_Ap.Set(*this, true);
  IMU::Delta::EigenFactor::RelativeKF e_A;
  e_A.Set(A, 0.0f);
  EigenVectorXf e_x;
  e_Ap.PropagateKF(e_A, &e_x);
#endif
  Matrix::X _A;
  Vector::X _b, _eps;
  const int Npg = 2, Npc = 6, Npm = 9, Np1 = Npg + Npm, Npcm = Npc + Npm, Np2 = Np1 + Npcm;
  work->Resize((x->BindSize(Np2) + _A.BindSize(Np2, Np2) + _b.BindSize(Np2) +
               (eps ? _eps.BindSize(Np2) : 0)) / sizeof(float));
  x->Bind(work->Data(), Np2);
  _A.Bind(x->BindNext(), Np2, Np2);
  _b.Bind(_A.BindNext(), Np2);
  if (eps) {
    _eps.Bind(_b.BindNext(), Np2);
  }
  GetPriorEquation(&_A, &_b, false);
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int ipr2 = 0;
  _A.InsertZero(ipr2, 3, work);
  _b.InsertZero(ipr2, 3, work);
  const int ipp2 = 3;
  _A.InsertZero(ipp2, 3, work);
  _b.InsertZero(ipp2, 3, work);
  const int ipbw1 = 6, ipbw2 = ipbw1 + 3;
  _A.InsertZero(ipbw2, 3, work);
  _b.InsertZero(ipbw2, 3, work);
  const int ipv1 = ipbw2 + 3, ipv2 = ipv1 + 3;
  _A.InsertZero(ipv2, 3, work);
  _b.InsertZero(ipv2, 3, work);
  const int ipg = ipv2 + 3, ipba1 = ipg + 2, ipba2 = ipba1 + 3;
  _A.InsertZero(ipba2, 3, work);
  _b.InsertZero(ipba2, 3, work);
  const int ipcs[8] = {ipv1, ipba1, ipbw1, ipp2, ipr2, ipv2, ipba2, ipbw2};
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  LA::Matrix3x2f Acg;
  for (int i = 0; i < 8; ++i) {
    const int ipc = ipcs[i];
    if (ipg < ipc) {
      _A.IncreaseBlock(ipg, ipc, A.m_Agc[i]);
    } else {
      A.m_Agc[i].GetTranspose(Acg);
      _A.IncreaseBlock(ipc, ipg, Acg);
    }
    _b.IncreaseBlock(ipc, A.m_bc[i]);
  }
  LA::AlignedMatrix3x3f Acc;
  for (int i = 0, k = 0; i < 8; ++i) {
    const int ip = ipcs[i];
    for (int j = i; j < 8; ++j, ++k) {
      const int jp = ipcs[j];
      if (ip <= jp) {
        _A.IncreaseBlock(ip, jp, A.m_Ac[k]);
      } else {
        A.m_Ac[k].GetTranspose(Acc);
        _A.IncreaseBlock(jp, ip, Acc);
      }
    }
  }
#else
  const int ipg = 0, ipc1 = Npg, ipc2 = ipc1 + Npm;
#ifdef CFG_DEBUG
  UT_ASSERT(ipc2 == Np1);
#endif
  _A.InsertZero(ipc2, Npcm, NULL);
  _b.InsertZero(ipc2, Npcm, NULL);
  //_A.Resize(Np2, Np2, true, true);
  //_b.Resize(Np2, true);
  _A.IncreaseBlockDiagonal(ipg, A.m_Agg);
  _b.IncreaseBlock(ipg, A.m_bg);
  for (int i = 0, ip = ipc1; i < 8; ++i, ip += 3) {
    _A.IncreaseBlock(0, ip, A.m_Agc[i]);
    _b.IncreaseBlock(ip, A.m_bc[i]);
  }
  for (int i = 0, ip = ipc1, k = 0; i < 8; ++i, ip += 3) {
    for (int j = i, jp = ip; j < 8; ++j, jp += 3, ++k) {
      _A.IncreaseBlock(ip, jp, A.m_Ac[k]);
    }
  }
#endif
  if (eps) {
    LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg);
    //eg->MakeZero();
    eg->Set(eps + 3);
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *er = (LA::Vector3<Element::T> *) _eps.Data();
    LA::Vector3<Element::T> *ep = (LA::Vector3<Element::T> *) (_eps.Data() + ipp2);
    er->Set(eps + 3);
    ep->Set(eps);
    LA::Vector3<Element::T> *ev = (LA::Vector3<Element::T> *) (_eps.Data() + ipv1);
    LA::Vector3<Element::T> *eba = (LA::Vector3<Element::T> *) (_eps.Data() + ipba1);
    LA::Vector3<Element::T> *ebw = (LA::Vector3<Element::T> *) (_eps.Data() + ipbw1);
    for (int i = 0; i < 2; ++i) {
      ev[i].Set(eps + 6);
      eba[i].Set(eps + 9);
      ebw[i].Set(eps + 12);
    }
#else
    LA::Vector9<Element::T> *em1 = (LA::Vector9<Element::T> *) (_eps.Data() + ipc1);
    LA::Vector6<Element::T> *ec2 = (LA::Vector6<Element::T> *) (em1 + 1);
    LA::Vector9<Element::T> *em2 = (LA::Vector9<Element::T> *) (ec2 + 1);
    ec2->Set(eps);
    em1->Set(eps + 6);
    em2->Set(eps + 6);
#endif
  }
  if (!_A.SolveLDL(_b, eps ? _eps.Data() : NULL)) {
    x->Resize(0);
    return false;
  }
  _b.MakeMinus();
#ifdef CFG_CAMERA_PRIOR_REORDER
  LA::Vector2f xg;
  _b.GetBlock(ipg, xg);
  x->SetBlock(0, xg);
  LA::Vector3f *xcs = (LA::Vector3f *) (x->Data() + Npg);
  for (int i = 0; i < 8; ++i) {
    _b.GetBlock(ipcs[i], xcs[i]);
  }
#else
  //x->Copy(_b);
  _b.GetBlock(0, *x);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  e_x.AssertEqual(*x);
#endif
  return true;
}

bool Joint::GetPriorPose(const int iKF, Pose *Zp, AlignedVector<float> *work, const float *eps) const {
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
  Zp->m_iKFr = m_iKFr;
  Zp->m_iKFs = m_iKFs;
  Zp->m_iKFs.back() = iKF;
  Zp->m_Zps.Set(m_Zps);

  Matrix::X A;
  Vector::X b, _work, _eps;
  const int N = static_cast<int>(m_iKFs.size());
  const int Npg = m_Zps.Size() == N ? 0 : 2, Npc = N * 6, Np = Npg + Npc + 9;
  work->Resize((A.BindSize(Np, Np, true) + b.BindSize(Np) + _work.BindSize(Np) +
               (eps ? _eps.BindSize(15) : 0)) / sizeof(float));
  A.Bind(work->Data(), Np, Np, true);
  b.Bind(A.BindNext(), Np);
  _work.Bind(b.BindNext(), Np);
  if (eps) {
    _eps.Bind(_work.BindNext(), 15);
    _eps.Set(eps);
  }
  bool scc = true;
  GetPriorEquation(&A, &b);
  if (iKF == INT_MAX) {
    const int Nk = N - 1;
    Zp->m_iKFs.resize(Nk);
    //Zp->m_Zps.Resize(Nk);
    const Rotation3D RrT = Zp->m_Zps.Back();
    Zp->m_Zps.Resize(Nk + 1);
    Zp->m_Zps.Back() = RrT;
#ifdef CFG_CAMERA_PRIOR_REORDER
    const int ipr = Nk * 3, ipp = Npc - 3, ipbw = Npc, ipv = ipbw + 3, ipba = ipv + 3 + Npg;
    scc = A.MarginalizeLDL(ipr, 3, b, &_work, _eps.Data() + 3) && scc;
    scc = A.MarginalizeLDL(ipp - 3, 3, b, &_work, _eps.Data()) && scc;
    scc = A.MarginalizeLDL(ipbw - 6, 3, b, &_work, _eps.Data() + 12) && scc;
    scc = A.MarginalizeLDL(ipv - 9, 3, b, &_work, _eps.Data() + 6) && scc;
    scc = A.MarginalizeLDL(ipba - 12, 3, b, &_work, _eps.Data() + 9) && scc;
#else
    scc = A.MarginalizeLDL(Np - 15, 15, b, &_work, eps ? _eps.Data() : NULL);
#endif
  } else {
#ifdef CFG_CAMERA_PRIOR_REORDER
    const int ipbw = Npc, ipv = ipbw + 3, ipba = ipv + 3 + Npg;
    scc = A.MarginalizeLDL(ipbw, 3, b, &_work, _eps.Data() + 12) && scc;
    scc = A.MarginalizeLDL(ipv - 3, 3, b, &_work, _eps.Data() + 6) && scc;
    scc = A.MarginalizeLDL(ipba - 6, 3, b, &_work, _eps.Data() + 9) && scc;
#else
    scc = A.MarginalizeLDL(Np - 9, 9, b, &_work, eps ? _eps.Data() + 6 : NULL);
#endif
  }
  Zp->SetPriorEquation(A, b);
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  if (scc) {
    EigenPrior e_Ap1;
    Pose::EigenPrior e_Ap2;
    e_Ap1.Set(*this);
    e_Ap1.GetPriorPose(iKF, &e_Ap2);
    e_Ap2.AssertEqual(*Zp, 1, "[Joint::GetPriorPose]");
  }
#endif
  return scc;
#else
  *Zp = *this;
  Zp->m_iKFs.back() = iKF;
  Element::MM Mmm;
  Element::RM Mrm;
  Vector::CM Mcm;
  const int N = m_Acm.Size();
  work->Resize(Mcm.BindSize(N) / sizeof(float));
  Mcm.Bind(work->Data(), N);
  Mmm = m_Amm;
  bool scc = true;
  if (Mmm.InverseLDL(eps ? eps + 6 : NULL)) {
    Mmm.MakeMinus();
    Element::ABT(m_Arm, Mmm, Mrm);
    Element::AddABTToUpper(Mrm, m_Arm, Zp->m_Arr);
    Vector::AddABTTo(Mrm, m_Acm, Zp->m_Arc);
    Element::AddAbTo(Mrm, m_bm, Zp->m_br);
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
    Element::CC &Mcici = Zp->m_Acc[Nk][Nk];
    if (Mcici.InverseLDL(eps)) {
      Mcici.MakeMinus();
      Zp->m_Arc.Resize(N + 1, true);
      const Element::RC &Arci = Zp->m_Arc[Nk];
      Element::RC &Mrci = Zp->m_Arc[N];
      Element::ABT(Arci, Mcici, Mrci);
      Element::AddABTToUpper(Mrci, Arci, Zp->m_Arr);
      const Matrix::CC Ackci = Zp->m_Acc.GetColumn(Nk, Nk);
      Vector::RC Arck = Zp->m_Arc.GetBlock(Nk);
      Vector::AddABTTo(Mrci, Ackci, Arck);
      const Element::C &bci = Zp->m_bc[Nk];
      Element::AddAbTo(Mrci, bci, Zp->m_br);
      work->Resize(sizeof(Element::CC) * Nk / sizeof(float));
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
  return scc;
#endif
}

bool Joint::GetPriorMotion(Motion *Zp, AlignedVector<float> *work, const float *eps) const {
#if defined CFG_CAMERA_PRIOR_DOUBLE || defined CFG_CAMERA_PRIOR_REORDER
  Zp->m_v = m_v;
  Zp->m_ba = m_ba;
  Zp->m_bw = m_bw;
  Matrix::X A;
  Vector::X b, _work, _eps;
  const int N = static_cast<int>(m_iKFs.size());
  const bool g = m_Zps.Size() != N;
  const int Npg = g ? 2 : 0;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npr = N * 3, Npp = Npr, Npc = Npr + Npp, Npgc = Npg + Npc, Np = Npgc + 9;
  const int ipr = 0, ipp = Npr, ipg = Npc + 6;
#else
  const int Npgc = Npg + N * 6, Np = Npgc + 9;
  const int ipg = 0;
#endif
  work->Resize((A.BindSize(Np, Np, true) + b.BindSize(Np) + _work.BindSize(Np) +
               (eps ? _eps.BindSize(Np) : 0)) / sizeof(float));
  A.Bind(work->Data(), Np, Np, true);
  b.Bind(A.BindNext(), Np);
  _work.Bind(b.BindNext(), Np);
  if (eps) {
    _eps.Bind(_work.BindNext(), Np);
    if (g) {
      LA::Vector2<Element::T> *eg = (LA::Vector2<Element::T> *) (_eps.Data() + ipg);
      //eg->MakeZero();
      eg->Set(eps + 3);
    }
#ifdef CFG_CAMERA_PRIOR_REORDER
    LA::Vector3<Element::T> *ecrs = (LA::Vector3<Element::T> *) (_eps.Data() + ipr);
    LA::Vector3<Element::T> *ecps = (LA::Vector3<Element::T> *) (_eps.Data() + ipp);
    for (int i = 0; i < N; ++i) {
      ecrs[i].Set(eps + 3);
      ecps[i].Set(eps);
    }
#else
    LA::Vector6<Element::T> *ecs = (LA::Vector6<Element::T> *) (_eps.Data() + Npg);
    for (int i = 0; i < N; ++i) {
      ecs[i].Set(eps);
    }
#endif
  }
  GetPriorEquation(&A, &b);
#ifdef CFG_CAMERA_PRIOR_REORDER
//#if 0
#if 1
  bool scc = true;
  scc = A.MarginalizeLDL(0, Npc, b, &_work, eps ? _eps.Data() : NULL) && scc;
  scc = A.MarginalizeLDL(ipg - Npc, 2, b, &_work, eps ? _eps.Data() + ipg : NULL) && scc;
#else
  //UT::DebugStart();
  int ip = 4;
  //int ip = 32;
  //int ip = Npc + 2;
  //const int ipd = 0;
  const int ipd = N * 3 + 1;
  if (UT::Debugging()) {
    UT::PrintSeparator();
    UT::Print("%.10e\n", A[ip][ip + ipd]);
  }
  //const bool scc1 = A.MarginalizeLDL(0, Npc, b, &_work, eps ? _eps.Data() : NULL);
  //if (UT::Debugging()) {
  //  //UT::Print("%.10e\n", b[ipbw - Npc]);
  //  UT::Print("%.10e\n", b[ipg - Npgc + 1]);
  //}
  bool scc1 = true;
  for (int jp = 0; jp < Npc; ++jp) {
    scc1 = A.MarginalizeLDL(0, 1, b, &_work, eps ? _eps.Data() : NULL) && scc1;
    if (UT::Debugging()) {
      if (ip > 0) {
        --ip;
      } else if (ip == 0) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < A.GetRows()) {
        UT::Print("%.10e\n", A[ip][ip + ipd]);
      }
    }
  }
  const int jpg = ipg - Npc;
  //const bool scc2 = A.MarginalizeLDL(jpg, 2, b, &_work, eps ? _eps.Data() + ipg : NULL);
  const bool scc21 = A.MarginalizeLDL(jpg, 1, b, &_work, eps ? _eps.Data() + ipg : NULL);
  if (UT::Debugging()) {
    if (ip > jpg) {
      --ip;
    } else if (ip == jpg) {
      ip = -1;
    }
    if (ip >= 0 && ip + ipd < A.GetRows()) {
      UT::Print("%.10e\n", A[ip][ip + ipd]);
    }
  }
  const bool scc22 = A.MarginalizeLDL(jpg, 1, b, &_work, eps ? _eps.Data() + ipg : NULL);
  const bool scc2 = scc21 && scc22;
  if (UT::Debugging()) {
    if (ip > jpg) {
      --ip;
    } else if (ip == jpg) {
      ip = -1;
    }
    if (ip >= 0 && ip + ipd < A.GetRows()) {
      UT::Print("%.10e\n", A[ip][ip + ipd]);
    }
  }
  const bool scc = scc1 && scc2;
  //UT::DebugStop();
#endif
  LA::Matrix3x3f _A;
  LA::Vector3f _b;
  A.GetBlock(0, 0, _A);                 Zp->m_Amm.SetBlock(6, 6, _A);
  A.GetBlock(0, 3, _A); _A.Transpose(); Zp->m_Amm.SetBlock(0, 6, _A);
  A.GetBlock(0, 6, _A); _A.Transpose(); Zp->m_Amm.SetBlock(3, 6, _A);
  A.GetBlock(3, 3, _A);                 Zp->m_Amm.SetBlock(0, 0, _A);
  A.GetBlock(3, 6, _A);                 Zp->m_Amm.SetBlock(0, 3, _A);
  A.GetBlock(6, 6, _A);                 Zp->m_Amm.SetBlock(3, 3, _A);
  Zp->m_Amm.SetLowerFromUpper();
  b.GetBlock(0, _b);  Zp->m_bm.Set678(_b);
  b.GetBlock(3, _b);  Zp->m_bm.Set012(_b);
  b.GetBlock(6, _b);  Zp->m_bm.Set345(_b);
#else
  const bool scc = A.MarginalizeLDL(0, Npgc, b, &_work, eps ? _eps.Data() : NULL);
  A.GetBlock(0, 0, Zp->m_Amm);
  b.GetBlock(0, Zp->m_bm);
#endif
#ifdef CAMERA_PRIOR_DEBUG_EIGEN
  if (scc) {
    EigenPrior e_Ap1;
    Motion::EigenPrior e_Ap2;
    e_Ap1.Set(*this);
    e_Ap1.GetPriorMotion(&e_Ap2);
    e_Ap2.AssertEqual(*Zp, 1, "[Joint::GetPriorMotion]");
  }
#endif
#else
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

  Element::RR Mrr;
  Element::RC Mrc;
  Element::RM Mrm;
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
      Element::CC *Ai = Acc[i];
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

  Element::CC Mij;
  Element::CM Mim;
  for (int i = 0; i < N; ++i) {
    Element::CC *Ai = Acc[i], &Mii = Ai[i];
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
      Element::CC &Aii = Ai[i];
      Aii.SetLowerFromUpper();
      UT::PrintSeparator();
      Aii.Print();
      const Element::CC _Mii = Aii.GetInverseLDL(-1);
      Element::CC I;
      LA::AlignedMatrix6x6f::ABT(Aii, _Mii, I);
      UT::PrintSeparator();
      I.Print();
    }
#endif
    if (Mii.InverseLDL(eps)) {
      Mii.MakeMinus();
      const Element::CM &Aim = Acm[i];
      const Element::C &bi = bc[i];
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
  Element::MM Amm = Zp->m_Amm;
  Element::M bm = Zp->m_bm;
  if (Amm.SolveLDL(bm, eps ? eps + 6 : NULL)) {
    Zp->m_em.Set(bm);
  } else {
    Zp->m_Amm.MakeZero();
    Zp->m_em.MakeZero();
  }
#endif
#endif
  if (!scc) {
    return false;
  }
  LA::AlignedMatrix9x9f Amm = Zp->m_Amm;
  const int rank = Amm.RankLDL(eps ? eps + 6 : NULL);
  return rank == 9;
}

#ifdef CFG_DEBUG_EIGEN
//#define CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
Pose::EigenErrorJacobian Pose::EigenGetErrorJacobian(const AlignedVector<Rigid3D> &Cs,
                                                     const float eps) const {
  EigenErrorJacobian e_Je;
  const int N = static_cast<int>(m_iKFs.size()), Nx6 = N * 6;
  e_Je.m_e.Resize(2 + Nx6);
  e_Je.m_e.MakeZero();
  e_Je.m_J.Resize(2 + Nx6, 6 + Nx6);
  e_Je.m_J.MakeZero();

  const Rigid3D &T1 = Cs[m_iKFr];
  const EigenRotation3D e_R1 = T1;
  //const bool g = !m_Arc.Empty();
  const bool g = m_Zps.Size() != N;
  if (g) {
    //const Rotation3D &RrT = m_RrT;
    const Rotation3D &RrT = m_Zps.Back();
    const EigenRotation3D e_RrT = RrT;
    //const EigenRotation3D e_eR = EigenRotation3D(e_RrT * e_R1);
    const EigenRotation3D e_eR = EigenRotation3D(RrT * T1);
    const EigenVector3f e_er1 = e_eR.GetRodrigues(eps);
    const EigenMatrix3x3f e_Jr1 = EigenMatrix3x3f(EigenRotation3D::GetRodriguesJacobianInverse(
                                                  e_er1, eps) * e_eR);
    e_Je.m_e.block<2, 1>(0, 0) = e_er1.block<2, 1>(0, 0);
    e_Je.m_J.block<2, 3>(0, 3) = e_Jr1.block<2, 3>(0, 0);
#ifdef CAMERA_PRIOR_POSE_EIGEN_DEBUG_JACOBIAN
    const float e_dr1Max = 1.0f;
    const EigenVector3f e_dr1 = EigenVector3f::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD);
    const EigenRotation3D e_R1GT = EigenMatrix3x3f(e_R1 * EigenRotation3D(e_dr1));
    const EigenVector3f e_er1GT = EigenRotation3D(e_RrT * e_R1GT).GetRodrigues(eps);
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
    //const EigenVector3f e_er = EigenRotation3D(e_R21 * e_R2
    //                                         * e_R1.GetTranspose()).GetRodrigues(eps);
    //const EigenVector3f e_er = (Rotation3D(T21) / (Rotation3D(T1) / T2)).GetRodrigues(eps);
    const EigenVector3f e_er = ((Rotation3D(T21) * Rotation3D(T2))
                               / Rotation3D(T1)).GetRodrigues(eps);
    e_ec.block<3, 1>(0, 0) = e_ep;
    e_ec.block<3, 1>(3, 0) = e_er;
    e_Je.m_e.block<6, 1>(j, 0) = e_ec;

    const EigenMatrix3x3f e_Jpp1 = EigenMatrix3x3f(-e_R1);
    const EigenMatrix3x3f e_Jpp2 = EigenMatrix3x3f(e_R1);
    const EigenMatrix3x3f e_Jpr1 = EigenMatrix3x3f(e_R1 * EigenSkewSymmetricMatrix(
                                                   EigenVector3f(e_p2 - e_p1)));
    const EigenMatrix3x3f e_Jrr2 = EigenMatrix3x3f(
                                   EigenRotation3D::GetRodriguesJacobianInverse(e_er, eps)
                                 * (e_R21 * e_R2));
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
    const EigenRotation3D e_R1GT = EigenRotation3D(e_R1 * EigenRotation3D(e_dr1, eps));
    const EigenRotation3D e_R2GT = EigenRotation3D(e_R2 * EigenRotation3D(e_dr2, eps));
    const EigenVector3f e_epGT = EigenVector3f(e_R1GT * EigenVector3f(e_p2GT - e_p1GT) - e_p12);
    const EigenVector3f e_erGT = EigenRotation3D(e_R21 * e_R2GT *
                                                 e_R1GT.GetTranspose()).GetRodrigues(eps);
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
  GetErrorJacobian(Cs, &Je, eps);
  e_Je.AssertEqual(Je);
  e_Je.Set(Je);
  if (!g) {
    e_Je.m_e.block(0, 0, 2, 1).setZero();
    e_Je.m_J.block(0, 0, 2, 6 + Nx6).setZero();
  }
#endif
  return e_Je;
}

Pose::EigenFactor Pose::EigenGetFactor(const float w, const AlignedVector<Rigid3D> &Cs,
                                       const float eps) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs, eps);
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
                         const std::vector<EigenVector6f> &e_xs, const float eps) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs, eps);
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

void Pose::EigenGetResidual(const AlignedVector<Rigid3D> &Cs, EigenVectorXf *e_r,
                            const float eps) const {
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(Cs, eps);
  const EigenPrior e_Ap = EigenPrior(m_Arr, m_Arc, m_Acc, m_br, m_bc);
  *e_r = EigenVectorXf(e_Ap.m_A * e_Je.m_e + e_Ap.m_b);
}

void Pose::EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S, EigenVectorXf *e_x) const {
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  const auto e_ldlt = e_Ap.m_A.cast<Element::T>().ldlt();
  const int N = static_cast<int>(e_Ap.m_A.rows());
  *e_S = e_ldlt.solve(EigenMatrixX<Element::T>::Identity(N)).cast<float>() * w;
  if (e_x) {
    *e_x = e_ldlt.solve(-e_Ap.m_b.cast<Element::T>()).cast<float>();
  }
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

void Motion::EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S,
                                      EigenVectorXf *e_x) const {
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  const auto e_ldlt = e_Ap.m_A.cast<Element::T>().ldlt();
  const int N = static_cast<int>(e_Ap.m_A.rows());
  *e_S = e_ldlt.solve(EigenMatrixX<Element::T>::Identity(N)).cast<float>() * w;
  if (e_x) {
    *e_x = e_ldlt.solve(-e_Ap.m_b.cast<Element::T>()).cast<float>();
  }
}

template<typename TYPE>
class DebugEigenMatrixX : public EigenMatrixX<TYPE> {
 public:
  inline void Marginalize(const int i, const int Ni, const TYPE *eps = NULL, const bool erase = true,
                          const bool upperLeft = true, const bool zero = false) {
    const int Nr = this->GetRows(), Nc = this->GetColumns();
    const int j = i + Ni, N1 = i, N2r = Nr - i - Ni, N2c = Nc - j;
    const EigenMatrixX<TYPE> e_Aii = EigenMatrixX<TYPE>(this->block(i, i, Ni, Ni));
    //const EigenMatrixX<TYPE> e_Mii = EigenMatrixX<TYPE>(e_Aii.inverse()); {
    EigenMatrixX<TYPE> e_Mii = e_Aii;
    if (e_Mii.InverseLDL(eps)) {
      const EigenMatrixX<TYPE> e_Ai1 = EigenMatrixX<TYPE>(this->block(i, 0, Ni, N1));
      const EigenMatrixX<TYPE> e_Ai2 = EigenMatrixX<TYPE>(this->block(i, j, Ni, N2c));
      const EigenMatrixX<TYPE> e_Mi1 = EigenMatrixX<TYPE>(e_Mii * e_Ai1);
      const EigenMatrixX<TYPE> e_Mi2 = EigenMatrixX<TYPE>(e_Mii * e_Ai2);
      const EigenMatrixX<TYPE> e_A1i = EigenMatrixX<TYPE>(this->block(0, i, N1, Ni));
      //const EigenMatrixX<TYPE> e_A2i = EigenMatrixX<TYPE>(this->block(j, i, N2r, Ni));
      const EigenMatrixX<TYPE> e_A2i = EigenMatrixX<TYPE>(this->block(i, j, Ni, N2r).transpose());
      const EigenMatrixX<TYPE> e_M1i = EigenMatrixX<TYPE>(e_A1i * e_Mii);
      const EigenMatrixX<TYPE> e_M2i = EigenMatrixX<TYPE>(e_A2i * e_Mii);
      if (upperLeft) {
        this->block(0, 0, N1, N1) -= e_A1i * e_Mi1;
        this->block(0, j, N1, N2c) -= e_A1i * e_Mi2;
        //this->block(j, 0, N2r, N1) -= e_A2i * e_Mi1;
        this->block(j, 0, N2r, N1) = this->block(0, j, N1, N2r).transpose();
      }
      //if (UT::Debugging()) {
      //  UT::Print("%e - %e * %e = %e", (*this)(20, 20), e_A2i(20 - i - 1, 0), e_Mi2(0, 20 - i - 1),
      //                                 (*this)(20, 20) - e_A2i(20 - i - 1, 0) * e_Mi2(0, 20 - i - 1));
      //}
      this->block(j, j, N2r, N2c) -= e_A2i * e_Mi2;
      //if (UT::Debugging()) {
      //  UT::Print(" --> %e\n", (*this)(20, 20));
      //}
      this->block(i, i, Ni, Ni) = -e_Mii;
      if (upperLeft) {
        this->block(i, 0, Ni, N1) = -e_Mi1;
        this->block(0, i, N1, Ni) = -e_M1i;
      }
      this->block(i, j, Ni, N2c) = -e_Mi2;
      //this->block(j, i, N2r, Ni) = -e_M2i;
      this->block(j, i, N2r, Ni) = this->block(i, j, Ni, N2r).transpose();
    } else {
      this->block(i, i, Ni, Ni).setZero();
      if (upperLeft) {
        this->block(i, 0, Ni, N1).setZero();
        this->block(0, i, N1, Ni).setZero();
      }
      this->block(i, j, Ni, N2c).setZero();
      this->block(j, i, N2r, Ni).setZero();
    }
    if (erase) {
      this->Erase(i, Ni);
    } else if (zero) {
      this->block(i, 0, Ni, Nc).setZero();
      this->block(0, i, Nr, Ni).setZero();
    }
  }
};

void Joint::EigenPrior::PropagateLF(const IMU::Delta::EigenFactor::RelativeLF &e_Ad,
                                    EigenVectorXf *e_x) {
  EigenMatrix2x2f e_dArr;
  EigenMatrix2x30f e_dArc;
  EigenMatrix30x30f e_dAcc;
  EigenVector2f e_dbr;
  EigenVector30f e_dbc;
  e_Ad.Get(e_dArr, e_dArc, e_dAcc, e_dbr, e_dbc);

  EigenMatrixX<Element::T> e_A = EigenMatrixX<Element::T>(m_A.cast<Element::T>());
  const int Nrck = e_A.GetRows() - 15;
  e_A.PushZero(15, 15);
  e_A.block<2, 2>(0, 0) += e_dArr.cast<Element::T>();
  e_A.block(0, Nrck, 2, 30) += e_dArc.cast<Element::T>();
  e_A.block(Nrck, 0, 30, 2) += e_dArc.cast<Element::T>().transpose();
  e_A.block(Nrck, Nrck, 30, 30) += e_dAcc.cast<Element::T>();
  EigenVectorX<Element::T> e_b = EigenVectorX<Element::T>(m_b.cast<Element::T>());
  e_b.PushZero(15);
  e_b.block<2, 1>(0, 0) += e_dbr.cast<Element::T>();
  e_b.block(Nrck, 0, 30, 1) += e_dbc.cast<Element::T>();
  if (e_x) {
    *e_x = e_A.ldlt().solve(-e_b).cast<float>();
  }

  EigenMatrixX<Element::T> e_Ab;
  //DebugEigenMatrixX<Element::T> e_Ab;
  e_Ab.Set(e_A, e_b);
  //e_Ab.Marginalize(Nrck, 15);
  const int ipp = Nrck, ipr = ipp + 3, ipv = ipr + 3, ipba = ipv + 3, ipbw = ipba + 3;
  //e_Ab.Marginalize(ipr, 3);
  //e_Ab.Marginalize(ipp, 3);
  //e_Ab.Marginalize(ipbw - 6, 3);
  //e_Ab.Marginalize(ipv - 6, 3);
  //e_Ab.Marginalize(ipba - 9, 3);
  e_Ab.Marginalize(ipr, 1);
  e_Ab.Marginalize(ipr, 1);
  e_Ab.Marginalize(ipr, 1);
  e_Ab.Marginalize(ipp, 1);
  e_Ab.Marginalize(ipp, 1);
  e_Ab.Marginalize(ipp, 1);
  e_Ab.Marginalize(ipbw - 6, 1);
  e_Ab.Marginalize(ipbw - 6, 1);
  e_Ab.Marginalize(ipbw - 6, 1);
  e_Ab.Marginalize(ipv - 6, 1);
  e_Ab.Marginalize(ipv - 6, 1);
  e_Ab.Marginalize(ipv - 6, 1);
  e_Ab.Marginalize(ipba - 9, 1);
  e_Ab.Marginalize(ipba - 9, 1);
  e_Ab.Marginalize(ipba - 9, 1);
  EigenMatrixXf(e_Ab.cast<float>()).Get(m_A, m_b);
}

void Joint::EigenPrior::PropagateKF(const IMU::Delta::EigenFactor::RelativeKF &e_A, EigenVectorXf *e_x) {
  EigenMatrix2x2f e_dArr;
  EigenMatrix2x30f e_dArc;
  EigenMatrix30x30f e_dAcc;
  EigenVector2f e_dbr;
  EigenVector30f e_dbc;
  e_A.Get(e_dArr, e_dArc, e_dAcc, e_dbr, e_dbc);

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

#ifdef CFG_DEBUG
  EigenMatrixXf(m_A.block<6, 32>(2, 0)).AssertZero(1, "", -1.0f, -1.0f);
  EigenVectorXf(m_b.block<6, 1>(2, 0)).AssertZero(1, "", -1.0f, -1.0f);
#endif
  m_A.Erase(2, 6);
  m_b.Erase(2, 6);
  if (e_x) {
    *e_x = m_A.cast<Element::T>().ldlt().solve(-m_b.cast<Element::T>()).cast<float>();
  }

  EigenMatrixX<Element::T> e_Ab;
  e_Ab.Set(EigenMatrixX<Element::T>(m_A.cast<Element::T>()),
           EigenVectorX<Element::T>(m_b.cast<Element::T>()));
  e_Ab.Marginalize(2, 9);
  //const int ipv = 2, ipba = ipv + 3, ipbw = ipba + 3;
  //e_Ab.Marginalize(ipbw, 3);
  //e_Ab.Marginalize(ipv, 3);
  //e_Ab.Marginalize(ipba - 3, 3);
  //e_Ab.Marginalize(ipbw, 1);
  //e_Ab.Marginalize(ipbw, 1);
  //e_Ab.Marginalize(ipbw, 1);
  //e_Ab.Marginalize(ipv, 1);
  //e_Ab.Marginalize(ipv, 1);
  //e_Ab.Marginalize(ipv, 1);
  //e_Ab.Marginalize(ipba - 3, 1);
  //e_Ab.Marginalize(ipba - 3, 1);
  //e_Ab.Marginalize(ipba - 3, 1);
  EigenMatrixXf(e_Ab.cast<float>()).Get(m_A, m_b);
}

void Joint::EigenPrior::GetPriorPose(const int iKF, Pose::EigenPrior *e_Ap) const {
  EigenMatrixX<Element::T> e_Ab;
  e_Ab.Set(EigenMatrixX<Element::T>(m_A.cast<Element::T>()),
           EigenVectorX<Element::T>(m_b.cast<Element::T>()));
  /*const */int Npgc = m_A.GetRows() - 9;
#ifdef CFG_CAMERA_PRIOR_REORDER
  if (iKF == INT_MAX) {
    Npgc -= 6;
    const int ipr = Npgc + 3;
    //e_Ab.Marginalize(ipr, 3);
    e_Ab.Marginalize(ipr, 1);
    e_Ab.Marginalize(ipr, 1);
    e_Ab.Marginalize(ipr, 1);
    //e_Ab.Marginalize(Npgc, 3);
    e_Ab.Marginalize(Npgc, 1);
    e_Ab.Marginalize(Npgc, 1);
    e_Ab.Marginalize(Npgc, 1);
  }
  const int ipv = Npgc, ipba = ipv + 3, ipbw = ipba + 3;
  //e_Ab.Marginalize(ipbw, 3);
  e_Ab.Marginalize(ipbw, 1);
  e_Ab.Marginalize(ipbw, 1);
  e_Ab.Marginalize(ipbw, 1);
  //e_Ab.Marginalize(ipv, 3);
  e_Ab.Marginalize(ipv, 1);
  e_Ab.Marginalize(ipv, 1);
  e_Ab.Marginalize(ipv, 1);
  //e_Ab.Marginalize(ipba - 3, 3);
  e_Ab.Marginalize(ipba - 3, 1);
  e_Ab.Marginalize(ipba - 3, 1);
  e_Ab.Marginalize(ipba - 3, 1);
#else
  e_Ab.Marginalize(Npgc, 9);
  if (iKF == INT_MAX) {
    e_Ab.Marginalize(Npgc - 6, 6);
  }
#endif
  EigenMatrixXf(e_Ab.cast<float>()).Get(e_Ap->m_A, e_Ap->m_b);
}

void Joint::EigenPrior::GetPriorMotion(Motion::EigenPrior *e_Ap) const {
  EigenMatrixX<Element::T> e_Ab;
  //DebugEigenMatrixX<Element::T> e_Ab;
  e_Ab.Set(EigenMatrixX<Element::T>(m_A.cast<Element::T>()),
           EigenVectorX<Element::T>(m_b.cast<Element::T>()));
  const int Npgc = m_A.GetRows() - 9;
#ifdef CFG_CAMERA_PRIOR_REORDER
  const int Npg = 2, Npc = Npgc - Npg, N = Npc / 6;
#ifdef CFG_DEBUG
  UT_ASSERT(Npc == N * 6);
#endif
//#if 0
#if 1
  for (int j = 0, jpr = Npg + 3; j < N; ++j, jpr += 3) {
    //e_Ab.Marginalize(jpr, 3);
    e_Ab.Marginalize(jpr, 1);
    e_Ab.Marginalize(jpr, 1);
    e_Ab.Marginalize(jpr, 1);
  }
  for (int j = 0; j < N; ++j) {
    //e_Ab.Marginalize(Npg, 3);
    e_Ab.Marginalize(Npg, 1);
    e_Ab.Marginalize(Npg, 1);
    e_Ab.Marginalize(Npg, 1);
  }
  //e_Ab.Marginalize(0, 2);
  e_Ab.Marginalize(0, 1);
  e_Ab.Marginalize(0, 1);
#else
  //UT::DebugStart();
  //int ip = Npgc + 8;
  //int ip = Npg + 1 * 6 + 2;
  int ip = Npg + 1 * 6 + 4;
  //const int ipd = 0;
  const int ipd = -2;
  if (UT::Debugging()) {
    UT::PrintSeparator();
    UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
  }
  for (int j = 0, jpr = Npg + 3; j < N; ++j, jpr += 3) {
    //e_Ab.Marginalize(jpr, 3);
    e_Ab.Marginalize(jpr, 1);
    if (UT::Debugging()) {
      if (ip > jpr) {
        --ip;
      } else if (ip == jpr) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
    e_Ab.Marginalize(jpr, 1);
    if (UT::Debugging()) {
      if (ip > jpr) {
        --ip;
      } else if (ip == jpr) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
    e_Ab.Marginalize(jpr, 1);
    if (UT::Debugging()) {
      if (ip > jpr) {
        --ip;
      } else if (ip == jpr) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
  }
  for (int j = 0; j < N; ++j) {
    //e_Ab.Marginalize(Npg, 3);
    e_Ab.Marginalize(Npg, 1);
    if (UT::Debugging()) {
      if (ip > Npg) {
        --ip;
      } else if (ip == Npg) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
    e_Ab.Marginalize(Npg, 1);
    if (UT::Debugging()) {
      if (ip > Npg) {
        --ip;
      } else if (ip == Npg) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
    e_Ab.Marginalize(Npg, 1);
    if (UT::Debugging()) {
      if (ip > Npg) {
        --ip;
      } else if (ip == Npg) {
        ip = -1;
      }
      if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
        UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
      }
    }
  }
  //if (UT::Debugging()) {
  //  //UT::Print("%.10e\n", e_Ab[e_Ab.GetRows() - 3][e_Ab.GetRows()]);
  //  UT::Print("%.10e\n", e_Ab[1][e_Ab.GetRows()]);
  //}
  //e_Ab.Marginalize(0, 2);
  e_Ab.Marginalize(0, 1);
  if (UT::Debugging()) {
    if (ip > 0) {
      --ip;
    } else if (ip == 0) {
      ip = -1;
    }
    if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
      UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
    }
  }
  e_Ab.Marginalize(0, 1);
  if (UT::Debugging()) {
    if (ip > 0) {
      --ip;
    } else if (ip == 0) {
      ip = -1;
    }
    if (ip >= 0 && ip + ipd < e_Ab.GetRows()) {
      UT::Print("%.10e\n", e_Ab[ip][ip + ipd]);
    }
  }
#endif
  //UT::DebugStop();
#else
  e_Ab.Marginalize(0, Npgc);
#endif
  EigenMatrixXf(e_Ab.cast<float>()).Get(e_Ap->m_A, e_Ap->m_b);
}

void Joint::EigenGetResidual(const AlignedVector<Rigid3D> &Cs, const Camera &C,
                             EigenVectorXf *e_r, const float eps) const {
  const Pose::EigenErrorJacobian e_Jec = Pose::EigenGetErrorJacobian(Cs, eps);
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

void Joint::EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S, EigenVectorXf *e_x) const {
  EigenPrior e_Ap;
  e_Ap.Set(*this);
  const auto e_ldlt = e_Ap.m_A.cast<Element::T>().ldlt();
  const int N = static_cast<int>(e_Ap.m_A.rows());
  *e_S = e_ldlt.solve(EigenMatrixX<Element::T>::Identity(N)).cast<float>() * w;
  if (e_x) {
    *e_x = e_ldlt.solve(-e_Ap.m_b.cast<Element::T>()).cast<float>();
  }
}
#endif

}  // namespace CameraPrior
