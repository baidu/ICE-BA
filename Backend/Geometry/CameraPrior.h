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
#ifndef _CAMERA_PRIOR_H_
#define _CAMERA_PRIOR_H_

#include "IMU.h"

namespace CameraPrior {

namespace Element{
#ifdef CFG_CAMERA_PRIOR_DOUBLE
typedef double T;
#else
typedef float T;
#endif
class R : public LA::Vector2f {
 public:
  inline R() {}
  inline R(const LA::Vector2f &b) { *((LA::Vector2f *) this) = b; }
};
class C : public LA::AlignedVector6f {
 public:
  inline C() {}
  inline C(const LA::AlignedVector6f &b) { *((LA::AlignedVector6f *) this) = b; }
  inline C(const LA::AlignedVector3f &bp, const LA::AlignedVector3f &br) {
    Set(bp, br);
  }
  inline void operator += (const LA::AlignedVector6f &dA) {
    *((LA::AlignedVector6f *) this) += dA;
  }
  inline void operator += (const LA::Vector6f &b) {
    v0123() += xp128f::get(b);
    v4() += b.v4();
    v5() += b.v5();
  }
  inline C operator + (const C &b) const {
    C apb;
    LA::AlignedVector6f::apb(*this, b, apb);
    return apb;
  }
};
class M : public LA::AlignedVector9f {
 public:
  inline M() {}
  inline M(const LA::AlignedVector9f &b) { *((LA::AlignedVector9f *) this) = b; }
  inline M(const LA::AlignedVector3f &bv, const LA::AlignedVector3f &bba,
           const LA::AlignedVector3f &bbw) {
    Set(bv, bba, bbw);
  }
  inline M operator + (const M &b) const {
    M apb;
    LA::AlignedVector9f::apb(*this, b, apb);
    return apb;
  }
};
class JC {
 public:
  inline void GetTranspose(JC &J) const {
    m_Jpp1.GetTranspose(J.m_Jpp1);
    m_Jpr1.GetTranspose(J.m_Jpr1);
    m_Jrr1.GetTranspose(J.m_Jrr1);
    m_Jpp2.GetTranspose(J.m_Jpp2);
    m_Jrr2.GetTranspose(J.m_Jrr2);
  }
 public:
  LA::AlignedMatrix3x3f m_Jpp1, m_Jpr1, m_Jrr1, m_Jpp2, m_Jrr2;
};
class EC {
 public:
  inline void Set(const float *e) { m_ep.Set(e); m_er.Set(e + 3); }
  inline void Set(const double *e) { m_ep.Set(e); m_er.Set(e + 3); }
  inline void Get(float *e) const { m_ep.Get(e); m_er.Get(e + 3); }
  inline void Get(double *e) const { m_ep.Get(e); m_er.Get(e + 3); }
  inline void Get(C *e) const { e->Set(m_ep, m_er); } 
 public:
  LA::AlignedVector3f m_ep, m_er;
};
class EM {
 public:
  inline void Set(const float *e) { m_ev.Set(e); m_eba.Set(e + 3); m_ebw.Set(e + 6); }
  inline void Set(const double *e) { m_ev.Set(e); m_eba.Set(e + 3); m_ebw.Set(e + 6); }
  inline void Get(float *e) const { m_ev.Get(e); m_eba.Get(e + 3); m_ebw.Get(e + 6); }
  inline void Get(double *e) const { m_ev.Get(e); m_eba.Get(e + 3); m_ebw.Get(e + 6); }
 public:  
  inline void operator += (const EM &e) {
    m_ev += e.m_ev;
    m_eba += e.m_eba;
    m_ebw += e.m_ebw;
  }
 public:
  LA::AlignedVector3f m_ev, m_eba, m_ebw;
};
class RR : public LA::SymmetricMatrix2x2f {
 public:
  inline RR() {}
  inline RR(const LA::SymmetricMatrix2x2f &A) { *((LA::SymmetricMatrix2x2f *) this) = A; }
};
class RC : public LA::AlignedMatrix2x6f {
 public:
  inline RC() {}
  inline RC(const LA::AlignedMatrix2x6f &A) { *((LA::AlignedMatrix2x6f *) this) = A; }
  inline RC operator + (const RC &B) const {
    RC ApB;
    LA::AlignedMatrix2x6f::ApB(*this, B, ApB);
    return ApB;
  }
};
class RM : public LA::AlignedMatrix2x9f {
 public:
  inline RM() {}
  inline RM(const LA::AlignedMatrixMxNf<2, 9> &A) { *((LA::AlignedMatrixMxNf<2, 9> *) this) = A; }
  inline void Set(const LA::AlignedMatrix2x3f &Arv, const LA::AlignedMatrix2x3f &Arba,
                  const LA::AlignedMatrix2x3f &Arbw) {
    LA::AlignedMatrix2x9f::Set(Arv, Arba, Arbw);
  }
  inline void Set(const LA::AlignedMatrix2x3f &Arv) {
    MakeZero();
    SetBlock(0, 0, Arv);
  }
  //inline void SetBlock(const LA::AlignedMatrix2x3f &Arv) {
  //  LA::AlignedMatrix2x9f::SetBlock(0, 0, Arv);
  //}
  inline RM operator + (const RM &B) const {
    RM ApB;
    LA::AlignedMatrix2x9f::ApB(*this, B, ApB);
    return ApB;
  }
};
class CC : public LA::AlignedMatrix6x6f {
 public:
  inline CC() {}
  inline CC(const LA::AlignedMatrix6x6f &A) { *((LA::AlignedMatrix6x6f *) this) = A; }
  inline void Get(LA::AlignedMatrix6x6f &A, const bool trans = false) const {
    if (trans) {
      GetTranspose(A);
    } else {
      A = *this;
    }
  }
  inline void Get(LA::AlignedMatrix3x3f &App, LA::AlignedMatrix3x3f &Apr,
                  LA::AlignedMatrix3x3f &Arp, LA::AlignedMatrix3x3f &Arr) const {
    LA::AlignedMatrix6x6f::Get(App, Apr, Arp, Arr);
  }
  inline void Get(LA::Matrix3x3f &App, LA::Matrix3x3f &Apr,
                  LA::Matrix3x3f &Arp, LA::Matrix3x3f &Arr) const {
    LA::AlignedMatrix6x6f::Get(App, Apr, Arp, Arr);
  }
  inline void Increase(const LA::SymmetricMatrix3x3f &App, const LA::AlignedMatrix3x3f &Apr,
                       const LA::SymmetricMatrix3x3f &Arr) {
    Increase00(App);
    Increase03(Apr);
    Increase33(Arr);
    SetLowerFromUpper();
  }
  inline void Increase(const LA::AlignedMatrix3x3f *Ap, const LA::AlignedMatrix3x3f *Ar) {
    LA::SymmetricMatrix3x3f A;
    A.Set(Ap[0]);
    Increase00(A);
    Increase03(Ap[1]);
    A.Set(Ar[1]);
    Increase33(A);
    SetLowerFromUpper();
  }
  inline void operator += (const LA::AlignedMatrix6x6f &A) {
    *((LA::AlignedMatrix6x6f *) this) += A;
  }
  inline void operator += (const LA::SymmetricMatrix6x6f &A) {
    LA::AlignedMatrix6x6f &_A = *this;
    _A.m_00_01_02_03() += xp128f::get(&A.m00());  _A[0][4] += A.m04();  _A[0][5] += A.m05();
    _A[1][1] += A.m11();  _A.m_12_13_14_15() += xp128f::get(&A.m12());
    _A[2][2] += A.m22();  _A[2][3] += A.m23();    _A[2][4] += A.m24();  _A[2][5] += A.m25();
                          _A[3][3] += A.m33();    _A[3][4] += A.m34();  _A[3][5] += A.m35();
                                                  _A[4][4] += A.m44();  _A[4][5] += A.m45();
                                                                        _A[5][5] += A.m55();
  }
};
class CM : public LA::AlignedMatrix6x9f {
 public:
  inline CM() {}
  inline CM(const LA::AlignedMatrixMxNf<6, 9> &A) { *((LA::AlignedMatrixMxNf<6, 9> *) this) = A; }
  inline void Set(const LA::AlignedMatrix3x3f &Apv, const LA::AlignedMatrix3x3f &Apba,
                  const LA::AlignedMatrix3x3f &Apbw, const LA::AlignedMatrix3x3f &Arv,
                  const LA::AlignedMatrix3x3f &Arba, const LA::AlignedMatrix3x3f &Arbw) {
    LA::AlignedMatrix6x9f::Set(Apv, Apba, Apbw, Arv, Arba, Arbw);
  }
  inline void Set(const LA::AlignedMatrix3x3f *Ap, const LA::AlignedMatrix3x3f *Ar) {
    Set(Ap[0], Ap[1], Ap[2], Ar[0], Ar[1], Ar[2]);
  }
  inline void Set(const LA::AlignedMatrix3x3f &Arv) { MakeZero(); SetBlock(Arv); }
  inline void SetBlock(const LA::AlignedMatrix3x3f &Arv) {
    LA::AlignedMatrix6x9f::SetBlock(3, 0, Arv);
  }
  inline CM operator + (const CM &B) const {
    CM ApB;
    LA::AlignedMatrix6x9f::ApB(*this, B, ApB);
    return ApB;
  }
};
class MC : public LA::AlignedMatrix9x6f {
 public:
  inline MC() {}
  inline void Set(const LA::AlignedMatrix3x3f &Avp, const LA::AlignedMatrix3x3f &Avr,
                  const LA::AlignedMatrix3x3f &Abap, const LA::AlignedMatrix3x3f &Abar,
                  const LA::AlignedMatrix3x3f &Abwp, const LA::AlignedMatrix3x3f &Abwr) {
    LA::AlignedMatrix9x6f::Set(Avp, Avr, Abap, Abar, Abwp, Abwr);
  }
  inline void Set(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                  const LA::AlignedMatrix3x3f *Abw) {
    Set(Av[0], Av[1], Aba[0], Aba[1], Abw[0], Abw[1]);
  }
  inline void Set(const LA::AlignedMatrix3x3f &Avr) { MakeZero(); SetBlock(Avr); }
  inline void SetBlock(const LA::AlignedMatrix3x3f &Avr) {
    LA::AlignedMatrix9x6f::SetBlock(0, 3, Avr);
  }
};
class MM : public LA::AlignedMatrix9x9f {
 public:
  inline MM() {}
  inline MM(const LA::AlignedMatrixMxNf<9, 9> &A) { *((LA::AlignedMatrixMxNf<9, 9> *) this) = A; }
  inline MM(const LA::AlignedMatrix9x9f &A) { *((LA::AlignedMatrix9x9f *) this) = A; }
  inline void Set(const LA::SymmetricMatrix3x3f &Avv, const float Ababa, const float Abwbw) {
    MakeZero();
    SetBlockDiagonal(0, Avv);
    LA::AlignedMatrix9x9f &A = *this;
    A[3][3] = A[4][4] = A[5][5] = Ababa;
    A[6][6] = A[7][7] = A[8][8] = Abwbw;
    SetLowerFromUpper();
  }
  inline void Set(const LA::AlignedMatrix3x3f &Avv, const LA::AlignedMatrix3x3f &Abav,
                  const float Ababa, const LA::AlignedMatrix3x3f &Abwv, const float Abwbw) {
    MakeZero();
    LA::AlignedMatrix9x9f::SetBlock(0, 0, Avv);
    LA::AlignedMatrix9x9f::SetBlock(3, 0, Abav);
    LA::AlignedMatrix9x9f::SetBlock(6, 0, Abwv);
    LA::AlignedMatrix9x9f &A = *this;
    A[3][3] = A[4][4] = A[5][5] = Ababa;
    A[6][6] = A[7][7] = A[8][8] = Abwbw;
  }
  inline void Set(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                  const LA::AlignedMatrix3x3f *Abw) {
    LA::AlignedMatrix9x9f::Set(Av, Aba, Abw);
  }
  inline void Increase(const LA::SymmetricMatrix3x3f &Avv, const LA::AlignedMatrix3x3f &Avba,
                       const LA::AlignedMatrix3x3f &Avbw, const LA::SymmetricMatrix3x3f &Ababa,
                       const LA::AlignedMatrix3x3f &Ababw, const LA::SymmetricMatrix3x3f &Abwbw) {
    LA::AlignedMatrix9x9f::IncreaseDiagonal(0, Avv);
    LA::AlignedMatrix9x9f::Increase(0, 3, Avba);
    LA::AlignedMatrix9x9f::Increase(0, 6, Avbw);
    LA::AlignedMatrix9x9f::IncreaseDiagonal(3, Ababa);
    LA::AlignedMatrix9x9f::Increase(3, 6, Ababw);
    LA::AlignedMatrix9x9f::IncreaseDiagonal(6, Abwbw);
    LA::AlignedMatrix9x9f::SetLowerFromUpper();
  }
  inline void Increase(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                       const LA::AlignedMatrix3x3f *Abw, const bool symmetric) {
    if (symmetric) {
      LA::SymmetricMatrix3x3f A;
      A.Set(Av[0]);
      LA::AlignedMatrix9x9f::IncreaseDiagonal(0, A);
      LA::AlignedMatrix9x9f::Increase(0, 3, Av[1]);
      LA::AlignedMatrix9x9f::Increase(0, 6, Av[2]);
      A.Set(Aba[1]);
      LA::AlignedMatrix9x9f::IncreaseDiagonal(3, A);
      LA::AlignedMatrix9x9f::Increase(3, 6, Aba[2]);
      A.Set(Abw[2]);
      LA::AlignedMatrix9x9f::IncreaseDiagonal(6, A);
      LA::AlignedMatrix9x9f::SetLowerFromUpper();
    } else {
      LA::AlignedMatrix9x9f::Increase(0, 0, Av[0]);
      LA::AlignedMatrix9x9f::Increase(0, 3, Av[1]);
      LA::AlignedMatrix9x9f::Increase(0, 6, Av[2]);
      LA::AlignedMatrix9x9f::Increase(3, 0, Aba[0]);
      LA::AlignedMatrix9x9f::Increase(3, 3, Aba[1]);
      LA::AlignedMatrix9x9f::Increase(3, 6, Aba[2]);
      LA::AlignedMatrix9x9f::Increase(6, 0, Abw[0]);
      LA::AlignedMatrix9x9f::Increase(6, 3, Abw[1]);
      LA::AlignedMatrix9x9f::Increase(6, 6, Abw[2]);
    }
  }
};
template<int M, int N, int K, class BLOCK_ABT>
inline void ABT(const LA::AlignedMatrixMxNf<M, N> &A, const LA::AlignedMatrixMxNf<K, N> &B,
                BLOCK_ABT &ABT) {
  for (int i = 0; i < M; ++i) {
    const LA::AlignedVectorNf<N> &Ai = A(i);
    float *ABTi = ABT[i];
    for (int j = 0; j < K; ++j) {
      ABTi[j] = Ai.Dot(B(j));
    }
  }
}
inline void ABT(const RC &A, const CC &B, RC &ABT) {
  LA::AlignedMatrix6x6f::ABT(A, B, ABT);
}
inline void ABT(const CC &A, const CC &B, CC &ABT) {
  LA::AlignedMatrix6x6f::ABT(A, B, ABT);
}
inline void ABT(const MC &A, const CC &B, MC &ABT) {
  LA::AlignedMatrix9x6f::ABT(A, B, ABT);
}
template<int M, int N, int K, class BLOCK_ABT>
inline void AddABTTo(const LA::AlignedMatrixMxNf<M, N> &A, const LA::AlignedMatrixMxNf<K, N> &B,
                     BLOCK_ABT &ABT) {
  for (int i = 0; i < M; ++i) {
    const LA::AlignedVectorNf<N> &Ai = A(i);
    float *ABTi = ABT[i];
    for (int j = 0; j < K; ++j) {
      ABTi[j] += Ai.Dot(B(j));
    }
  }
}
inline void AddABTTo(const RC &A, const CC &B, RC &ABT) {
  LA::AlignedMatrix6x6f::AddABTTo(A, B, ABT);
}
inline void AddABTTo(const RC &A, const MC &B, RM &ABT) {
  LA::AlignedMatrix9x6f::AddABTTo(A, B, ABT);
}
inline void AddABTTo(const CC &A, const CC &B, CC &ABT) {
  LA::AlignedMatrix6x6f::AddABTTo(A, B, ABT);
}
inline void AddABTTo(const CC &A, const MC &B, CM &ABT) {
  LA::AlignedMatrix9x6f::AddABTTo(A, B, ABT);
}
template<int M, int N, int K, class BLOCK_ABT>
inline void AddABTToUpper(const LA::AlignedMatrixMxNf<M, N> &A,
                          const LA::AlignedMatrixMxNf<K, N> &B, BLOCK_ABT &ABT) {
  for (int i = 0; i < M; ++i) {
    const LA::AlignedVectorNf<N> &Ai = A(i);
    float *ABTi = ABT[i];
    for (int j = i; j < K; ++j) {
      ABTi[j] += Ai.Dot(B(j));
    }
  }
}
inline void AddABTToUpper(const RC &A, const RC &B, RR &ABT) {
  LA::AlignedMatrix2x6f::AddABTTo(A, B, ABT);
}
inline void AddABTToUpper(const CC &A, const CC &B, CC &ABT) {
  LA::AlignedMatrix6x6f::AddABTTo(A, B, ABT);
}
inline void AddABTToUpper(const RM &A, const RM &B, RR &ABT) {
  LA::AlignedMatrix2x9f::AddABTTo(A, B, ABT);
}
template<int M, const int N, class BLOCK_B, class BLOCK_ABT>
inline void AddAbTo(const LA::AlignedMatrixMxNf<M, N> &A, const BLOCK_B &b, BLOCK_ABT &ABT) {
  for (int i = 0; i < M; ++i) {
    ABT[i] += A(i).Dot(b);
  }
}
inline void AddAbTo(const RC &A, const C &b, R &Ab) {
  LA::AlignedMatrix2x6f::AddAbTo(A, b, Ab);
}
inline void AddAbTo(const RM &A, const M &b, R &Ab) {
  LA::AlignedMatrix2x9f::AddAbTo(A, b, Ab);
}
inline void AddAbTo(const CC &A, const C &b, C &Ab) {
  LA::AlignedMatrix6x6f::AddAbTo(A, b, (float *) &Ab);
}
}  // namespace Element

namespace Vector {
#ifdef CFG_CAMERA_PRIOR_DOUBLE
typedef LA::AlignedVectorXd X;
#else
typedef LA::AlignedVectorXf X;
#endif
class C : public AlignedVector<Element::C> {
 public:
  inline C() {}
  inline C(const AlignedVector<Element::C> &V) { *((AlignedVector<Element::C> *) this) = V; }
  inline C(void *data, const int N, const bool own = true) :
           AlignedVector<Element::C>(data, N, own) {}
  inline void operator += (const C &V) {
    const int N = Size();
#ifdef CFG_DEBUG
    UT_ASSERT(V.Size() == N);
#endif
    for (int i = 0; i < N; ++i) {
      m_data[i] += V[i];
    }
  }
  inline float SquaredLength() const {
    float S = 0.0f;
    const int N = Size();
    for (int i = 0; i < N; ++i) {
      S += m_data[i].SquaredLength();
    }
    return S;
  }
};
class EC : public AlignedVector<Element::EC> {
};
class JC : public AlignedVector<Element::JC> {
 public:
  inline void GetTranspose(JC &JT) const {
    const int N = Size();
    JT.Resize(N);
    for (int i = 0; i < N; ++i) {
      m_data[i].GetTranspose(JT[i]);
    }
  }
};
class RC : public AlignedVector<Element::RC> {
 public:
  inline RC() {}
  inline RC(const AlignedVector<Element::RC> &V) { *((AlignedVector<Element::RC> *) this) = V; }
  inline RC(void *data, const int N, const bool own = true) :
            AlignedVector<Element::RC>(data, N, own) {}
};
class RM : public AlignedVector<Element::RM> {
 public:
  inline RM() {}
  inline RM(const AlignedVector<Element::RM> &V) { *((AlignedVector<Element::RM> *) this) = V; }
  inline RM(void *data, const int N, const bool own = true) :
            AlignedVector<Element::RM>(data, N, own) {}
};
class CC : public AlignedVector<Element::CC> {
 public:
  inline CC() {}
  inline CC(const AlignedVector<Element::CC> &V) { *((AlignedVector<Element::CC> *) this) = V; }
  inline CC(void *data, const int N, const bool own = true) :
            AlignedVector<Element::CC>(data, N, own) {}
};
class CM : public AlignedVector<Element::CM> {
 public:
  inline CM() {}
  inline CM(const AlignedVector<Element::CM> &V) { *((AlignedVector<Element::CM> *) this) = V; }
  inline CM(void *data, const int N, const bool own = true) :
            AlignedVector<Element::CM>(data, N, own) {}
};
class MC : public AlignedVector<Element::MC> {
 public:
  inline MC() {}
  inline MC(const AlignedVector<Element::MC> &V) { *((AlignedVector<Element::MC> *) this) = V; }
  inline MC(void *data, const int N, const bool own = true) :
            AlignedVector<Element::MC>(data, N, own) {}
};
class MM : public AlignedVector<Element::MM> {
 public:
  inline MM() {}
  inline MM(const AlignedVector<Element::MM> &V) { *((AlignedVector<Element::MM> *) this) = V; }
  inline MM(void *data, const int N, const bool own = true) :
            AlignedVector<Element::MM>(data, N, own) {}
};
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void ABT(const AlignedVector<BLOCK_A> &A, const BLOCK_B &B,
                AlignedVector<BLOCK_ABT> &ABT) {
  const int N = A.Size();
  ABT.Resize(N);
  for (int i = 0; i < N; ++i) {
    Element::ABT(A[i], B, ABT[i]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void ABT(const AlignedMatrixX<BLOCK_A> &A, const BLOCK_B &B,
                AlignedVector<BLOCK_ABT> &ABT) {
  const int N = A.GetRows();
#ifdef CFG_DEBUG
  UT_ASSERT(A.GetColumns() == 1);
#endif
  ABT.Resize(N);
  for (int i = 0; i < N; ++i) {
    Element::ABT(A[i][0], B, ABT[i]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTTo(const AlignedVector<BLOCK_A> &A, const BLOCK_B &B,
                     AlignedVector<BLOCK_ABT> &ABT) {
  const int N = A.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(ABT.Size() == N);
#endif
  for (int i = 0; i < N; ++i) {
    Element::AddABTTo(A[i], B, ABT[i]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTTo(const AlignedVector<BLOCK_A> &A, const BLOCK_B &B,
                     AlignedMatrixX<BLOCK_ABT> &ABT) {
  const int N = A.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(ABT.GetRows() == N && ABT.GetColumns() == 1);
#endif
  for (int i = 0; i < N; ++i) {
    Element::AddABTTo(A[i], B, ABT[i][0]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTTo(const BLOCK_A &A, const AlignedVector<BLOCK_B> &B,
                     AlignedVector<BLOCK_ABT> &ABT) {
  const int N = B.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(ABT.Size() == N);
#endif
  for (int i = 0; i < N; ++i) {
    Element::AddABTTo(A, B[i], ABT[i]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTTo(const BLOCK_A &A, const AlignedMatrixX<BLOCK_B> &B,
                     AlignedVector<BLOCK_ABT> &ABT) {
  const int N = B.GetRows();
#ifdef CFG_DEBUG
  UT_ASSERT(B.GetColumns() == 1 && ABT.Size() == N);
#endif
  for (int i = 0; i < N; ++i) {
    Element::AddABTTo(A, B[i][0], ABT[i]);
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddAbTo(const AlignedVector<BLOCK_A> &A, const BLOCK_B &b,
                    AlignedVector<BLOCK_ABT> &Ab) {
  const int N = A.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(Ab.Size() == N);
#endif
  for (int i = 0; i < N; ++i) {
    Element::AddAbTo(A[i], b, Ab[i]);
  }
}

#ifdef CFG_DEBUG_EIGEN
inline EigenVectorXf EigenConvert(const C &b) {
  EigenVectorXf e_b;
  const int N = b.Size();
  e_b.Resize(N * 6);
  e_b.MakeZero();
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    if (b[i].Valid()) {
      e_b.block<6, 1>(j, 0) = EigenVector6f(b[i]);
    }
  }
  return e_b;
}
inline void EigenConvert(const EigenVectorXf &e_b, C &b) {
  const int Nx6 = e_b.Size(), N = Nx6 / 6;
#ifdef CFG_DEBUG
  UT_ASSERT(Nx6 % 6 == 0);
#endif
  b.Resize(N);
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    b[i] = EigenVector6f(e_b.block<6, 1>(j, 0)).GetAlignedVector6f();
  }
}
inline EigenMatrixXf EigenConvert(const RC &V) {
  const int N = V.Size();
  EigenMatrixXf e_V;
  e_V.Resize(2, N * 6);
  e_V.MakeZero();
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    if (V[i].Valid()) {
      e_V.block<2, 6>(0, j) = EigenMatrix2x6f(V[i]);
    }
  }
  return e_V;
}
inline void EigenConvert(const EigenMatrixXf &e_V, RC &V) {
  const int Nx6 = e_V.GetColumns(), N = Nx6 / 6;
#ifdef CFG_DEBUG
  UT_ASSERT(Nx6 % 6 == 0 && e_V.GetRows() == 2);
#endif
  V.Resize(N);
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    V[i] = EigenMatrix2x6f(e_V.block<2, 6>(0, j)).GetAlignedMatrix2x6f();
  }
}
inline EigenMatrixXf EigenConvert(const CM &V) {
  const int N = V.Size();
  EigenMatrixXf e_V;
  e_V.Resize(N * 6, 9);
  e_V.MakeZero();
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    if (V[i].Valid()) {
      e_V.block<6, 9>(j, 0) = EigenMatrix6x9f(V[i]);
    }
  }
  return e_V;
}
inline void EigenConvert(const EigenMatrixXf &e_V, CM &V) {
  const int Nx6 = e_V.GetRows(), N = Nx6 / 6;
#ifdef CFG_DEBUG
  UT_ASSERT(Nx6 % 6 == 0 && e_V.GetColumns() == 9);
#endif
  V.Resize(N);
  for (int i = 0, j = 0; i < N; ++i, j += 6) {
    V[i] = EigenMatrix6x9f(e_V.block<6, 9>(j, 0)).GetAlignedMatrixMxNf();
  }
}
inline bool EigenAssertEqual(const EigenVectorXf &e_b, const C &b,
                             const int verbose = 1, const std::string str = "") {
  const int Nx6 = e_b.Size(), N = b.Size();
  UT_ASSERT(N == 0 || Nx6 == N * 6);
  bool scc = true;
  for (int i = 0, j = 0; j < Nx6; ++i, j += 6) {
    const EigenVector6f e_b1 = EigenVector6f(e_b.block<6, 1>(j, 0));
    const std::string _str = str + UT::String("[%d]", i);
    if (N != 0 && b[i].Valid()) {
      const EigenVector6f e_b2 = EigenVector6f(b[i]);
      scc = e_b1.AssertEqual(e_b2, verbose, _str) && scc;
    } else {
      scc = e_b1.AssertZero(verbose, _str) && scc;
    }
  }
  return scc;
}
inline bool EigenAssertEqual(const EigenMatrixXf &e_V, const RC &V,
                             const int verbose = 1, const std::string str = "") {
  const int Nx6 = e_V.GetColumns(), N = V.Size();
  UT_ASSERT(e_V.GetRows() == 2 && (N == 0 || Nx6 == N * 6));
  bool scc = true;
  for (int i = 0, j = 0; j < Nx6; ++i, j += 6) {
    const EigenMatrix2x6f e_A1 = EigenMatrix2x6f(e_V.block<2, 6>(0, j));
    const std::string _str = str + UT::String("[%d]", i);
    if (N != 0 && V[i].Valid()) {
      const EigenMatrix2x6f e_A2 = EigenMatrix2x6f(V[i]);
      scc = e_A1.AssertEqual(e_A2, verbose, _str) && scc;
    } else {
      scc = e_A1.AssertZero(verbose, _str) && scc;
    }
  }
  return scc;
}
inline bool EigenAssertEqual(const EigenMatrixXf &e_V, const CM &V,
                             const int verbose = 1, const std::string str = "") {
  const int Nx6 = e_V.GetRows(), N = V.Size();
  UT_ASSERT((N == 0 || Nx6 == N * 6) && e_V.GetColumns() == 9);
  bool scc = true;
  for (int i = 0, j = 0; j < Nx6; ++i, j += 6) {
    const EigenMatrix6x9f e_A1 = EigenMatrix6x9f(e_V.block<6, 9>(j, 0));
    const std::string _str = str + UT::String("[%d]", i);
    if (N != 0 && V[i].Valid()) {
      const EigenMatrix6x9f e_A2 = EigenMatrix6x9f(V[i]);
      scc = e_A1.AssertEqual(e_A2, verbose, _str) && scc;
    } else {
      scc = e_A1.AssertZero(verbose, _str) && scc;
    }
  }
  return scc;
}
#endif
}  // namespace Vector

namespace Matrix {
#ifdef CFG_CAMERA_PRIOR_DOUBLE
typedef LA::AlignedMatrixXd X;
#else
typedef LA::AlignedMatrixXf X;
#endif
class CC : public AlignedMatrixX<Element::CC> {
 public:
  inline CC() {}
  inline CC(const AlignedMatrixX<Element::CC> &A) { *((AlignedMatrixX<Element::CC> *) this) = A; }
  inline void MakeIdentity() {
    MakeZero();
    const int N = GetRows();
#ifdef CFG_DEBUG
    UT_ASSERT(GetColumns() == N);
#endif
    for (int i = 0; i < N; ++i) {
      m_rows[i][i].MakeIdentity();
    }
  }
  inline void SetLowerFromUpper() {
    const int N = GetRows();
#ifdef CFG_DEBUG
    UT_ASSERT(GetColumns() == N);
#endif
    if (m_symmetric) {
      for (int i = 0; i < N; ++i) {
        m_rows[i][i].SetLowerFromUpper();
      }
    } else {
      for (int i = 0; i < N; ++i) {
        m_rows[i][i].SetLowerFromUpper();
        for (int j = i + 1; i < N; ++i) {
          m_rows[i][j].GetTranspose(m_rows[j][i]);
        }
      }
    }
  }
  inline void AssertSymmetric() const {
    const int N = GetRows();
#ifdef CFG_DEBUG
    UT_ASSERT(GetColumns() == N);
#endif
    if (m_symmetric) {
      for (int i = 0; i < N; ++i) {
        m_rows[i][i].AssertSymmetric();
      }
    } else {
      for (int i = 0; i < N; ++i) {
        m_rows[i][i].AssertSymmetric();
        for (int j = i + 1; j < N; ++j) {
          m_rows[i][j].AssertSymmetric(m_rows[j][i]);
        }
      }
    }
  }
};
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTTo(const AlignedVector<BLOCK_A> &A, const AlignedVector<BLOCK_B> &B,
                     AlignedMatrixX<BLOCK_ABT> &ABT) {
  const int M = A.Size(), N = B.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(ABT.GetRows() == M && ABT.GetColumns() == N);
#endif
  for (int i = 0; i < M; ++i) {
    const BLOCK_A &Ai = A[i];
    BLOCK_ABT *ABTi = ABT[i];
    for (int j = 0; j < N; ++j) {
      Element::AddABTTo(Ai, B[j], ABTi[j]);
    }
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTToUpper(const AlignedVector<BLOCK_A> &A, const AlignedVector<BLOCK_B> &B,
                          AlignedMatrixX<BLOCK_ABT> &ABT) {
  const int N = A.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(B.Size() == N && ABT.GetRows() == N && ABT.GetColumns() == N);
#endif
  for (int i = 0; i < N; ++i) {
    const BLOCK_A &Ai = A[i];
    BLOCK_ABT *ABTi = ABT[i];
    Element::AddABTToUpper(Ai, B[i], ABTi[i]);
    for (int j = i + 1; j < N; ++j) {
      Element::AddABTTo(Ai, B[j], ABTi[j]);
    }
  }
}
template<class BLOCK_A, class BLOCK_B, class BLOCK_ABT>
inline void AddABTToUpper(const AlignedVector<BLOCK_A> &A, const AlignedMatrixX<BLOCK_B> &B,
                          AlignedMatrixX<BLOCK_ABT> &ABT) {
  const int N = A.Size();
#ifdef CFG_DEBUG
  UT_ASSERT(B.GetRows() == N && B.GetColumns() == 1);
  UT_ASSERT(ABT.GetRows() == N && ABT.GetColumns() == N);
#endif
  for (int i = 0; i < N; ++i) {
    const BLOCK_A &Ai = A[i];
    BLOCK_ABT *ABTi = ABT[i];
    Element::AddABTToUpper(Ai, B[i][0], ABTi[i]);
    for (int j = i + 1; j < N; ++j) {
      Element::AddABTTo(Ai, B[j][0], ABTi[j]);
    }
  }
}

#ifdef CFG_DEBUG_EIGEN
inline EigenMatrixXf EigenConvert(const CC &A) {
  EigenMatrixXf e_A;
  const int N = A.GetRows(), Nx6 = N * 6;
#ifdef CFG_DEBUG
  UT_ASSERT(A.GetColumns() == N);
#endif
  e_A.Resize(Nx6, Nx6);
  e_A.MakeZero();
  if (A.Symmetric()) {
    for (int i1 = 0, j1 = 0; i1 < N; ++i1, j1 += 6) {
      for (int i2 = i1, j2 = j1; i2 < N; ++i2, j2 += 6) {
        if (A[i1][i2].Invalid()) {
          continue;
        }
        e_A.block<6, 6>(j1, j2) = EigenMatrix6x6f(A[i1][i2]);
        if (i2 != i1) {
          e_A.block<6, 6>(j2, j1) = e_A.block<6, 6>(j1, j2).transpose();
        }
      }
    }
  } else {
    for (int i1 = 0, j1 = 0; i1 < N; ++i1, j1 += 6) {
      for (int i2 = 0, j2 = 0; i2 < N; ++i2, j2 += 6) {
        if (A[i1][i2].Valid()) {
          e_A.block<6, 6>(j1, j2) = EigenMatrix6x6f(A[i1][i2]);
        }
      }
    }
  }
  return e_A;
}
inline void EigenConvert(const EigenMatrixXf &e_A, CC &A, const bool symmetric = true) {
  const int Nrx6 = e_A.GetRows(), Nr = Nrx6 / 6;
  const int Ncx6 = e_A.GetColumns(), Nc = Ncx6 / 6;
#ifdef CFG_DEBUG
  UT_ASSERT(Nrx6 % 6 == 0 && Ncx6 % 6 == 0);
#endif
  A.Resize(Nr, Nc, symmetric);
  for (int i1 = 0, j1 = 0; i1 < Nr; ++i1, j1 += 6) {
    for (int i2 = symmetric ? i1 : 0, j2 = symmetric ? j1 : 0; i2 < Nc; ++i2, j2 += 6) {
      A[i1][i2] = EigenMatrix6x6f(e_A.block<6, 6>(j1, j2)).GetAlignedMatrix6x6f();
    }
  }
}
inline bool EigenAssertEqual(const EigenMatrixXf &e_A, const CC &A,
                             const int verbose = 1, const std::string str = "") {
  const int Nx6 = e_A.GetRows(), N = A.GetRows();
  UT_ASSERT(N == 0 || Nx6 == N * 6);
  UT_ASSERT(e_A.GetColumns() == Nx6 && A.GetColumns() == N);
  bool scc = true;
  if (A.Symmetric()) {
    A.AssertSymmetric();
    for (int i1 = 0, j1 = 0; j1 < Nx6; ++i1, j1 += 6) {
      for (int i2 = i1, j2 = j1; j2 < Nx6; ++i2, j2 += 6) {
        const EigenMatrix6x6f e_A1 = EigenMatrix6x6f(e_A.block<6, 6>(j1, j2));
        const std::string _str = str + UT::String("[%d][%d]", i1, i2);
        if (N != 0 && A[i1][i2].Valid()) {
          const EigenMatrix6x6f e_A2 = EigenMatrix6x6f(A[i1][i2]);
          scc = e_A1.AssertEqual(e_A2, verbose, _str) && scc;
        } else {
          scc = e_A1.AssertZero(verbose, _str) && scc;
        }
        if (i2 == i1) {
          continue;
        }
        const EigenMatrix6x6f e_A3 = EigenMatrix6x6f(e_A.block<6, 6>(j2, j1).transpose());
        scc = e_A1.AssertEqual(e_A3, verbose, str + UT::String("[%d][%d]", i2, i1)) && scc;
      }
    }
  } else {
    for (int i1 = 0, j1 = 0; j1 < Nx6; ++i1, j1 += 6) {
      for (int i2 = 0, j2 = 0; j2 < Nx6; ++i2, j2 += 6) {
        const EigenMatrix6x6f e_A1 = EigenMatrix6x6f(e_A.block<6, 6>(j1, j2));
        const std::string _str = str + UT::String("[%d][%d]", i1, i2);
        if (N != 0 && A[i1][i2].Valid()) {
          const EigenMatrix6x6f e_A2 = EigenMatrix6x6f(A[i1][i2]);
          scc = e_A1.AssertEqual(e_A2, verbose, _str) && scc;
        } else {
          scc = e_A1.AssertZero(verbose, _str) && scc;
        }
      }
    }
  }
  return scc;
}
#endif
}  // namespace Matrix

class Pose {

 public:

  class Error {
   public:
    inline Error() {}
    inline Error(const Error &e) { *this = e; }
    inline void operator = (const Error &e) { m_er = e.m_er; m_ec.Set(e.m_ec); }
    inline void Resize(const int N) { m_ec.Resize(N); }
    inline int Size() const { return m_ec.Size(); }
    inline void MakeZero() { m_er.MakeZero(); m_ec.MakeZero(); }
    inline void Bind(void *data, const int N) { m_ec.Bind(data, N); }
    inline void* BindNext() { return m_ec.BindNext(); }
    inline int BindSize(const int N) const { return m_ec.BindSize(N); }
    inline void SaveB(FILE *fp) const { UT::SaveB(m_er, fp); m_ec.SaveB(fp); }
    inline void LoadB(FILE *fp) { UT::LoadB(m_er, fp); m_ec.LoadB(fp); }
   public:
    LA::Vector2f m_er;
    Vector::EC m_ec;
  };
  class Jacobian {
   public:
    inline Jacobian() {}
    inline Jacobian(const Jacobian &J) { *this = J; }
    inline void operator = (const Jacobian &J) { m_Jr = J.m_Jr; m_Jc.Set(J.m_Jc); }
    inline void Resize(const int N) { m_Jc.Resize(N); }
    inline int Size() const { return m_Jc.Size(); }
    inline void MakeZero() { m_Jr.MakeZero(); m_Jc.MakeZero(); }
    inline void Bind(void *data, const int N) { m_Jc.Bind(data, N); }
    inline void* BindNext() { return m_Jc.BindNext(); }
    inline int BindSize(const int N) const { return m_Jc.BindSize(N); }
    inline void SaveB(FILE *fp) const { UT::SaveB(m_Jr, fp); m_Jc.SaveB(fp); }
    inline void LoadB(FILE *fp) { UT::LoadB(m_Jr, fp); m_Jc.LoadB(fp); }
   public:
    //LA::Matrix2x3f m_Jr;
    LA::AlignedMatrix2x3f m_Jr;
    Vector::JC m_Jc;
  };
  class ErrorJacobian {
   public:
    inline ErrorJacobian() {}
    inline ErrorJacobian(const ErrorJacobian &Je) { *this = Je; }
    inline void operator = (const ErrorJacobian &Je) { m_e = Je.m_e; m_J = Je.m_J; }
    inline void Resize(const int N) { m_e.Resize(N); m_J.Resize(N); }
    inline int Size() const { return m_e.Size(); }
    inline void MakeZero() { m_e.MakeZero(); m_J.MakeZero(); }
    inline void Bind(void *data, const int N) { m_e.Bind(data, N); m_J.Bind(m_e.BindNext(), N); }
    inline void* BindNext() { return m_J.BindNext(); }
    inline int BindSize(const int N) const { return m_e.BindSize(N) + m_J.BindSize(N); }
    inline void SaveB(FILE *fp) const { m_e.SaveB(fp); m_J.SaveB(fp); }
    inline void LoadB(FILE *fp) { m_e.LoadB(fp); m_J.LoadB(fp); }
    inline void AssertConsistency() const { UT_ASSERT(m_e.Size() == m_J.Size()); }
   public:
    Jacobian m_J;
    Error m_e;
  };
  class Factor {
   public:
    class Auxiliary {
     public:
      inline void Bind(void *data, const int N) {
        m_JcT.Bind(data, N);
        m_JTAc0.Bind(m_JcT.BindNext(), N);
        m_JTAc.Bind(m_JTAc0.BindNext(), N, N, true);
        m_Aec.Bind(m_JTAc.BindNext(), N);
      }
      inline void* BindNext() { return m_Aec.BindNext(); }
      inline int BindSize(const int N) const {
        return m_JcT.BindSize(N) + m_JTAc0.BindSize(N) + m_JTAc.BindSize(N, N, true) +
               m_Aec.BindSize(N);
      }
      inline void Set(const bool g, const ErrorJacobian &Je/*, const LA::AlignedMatrix2x3f &Jr*/,
                      const xp128f &w, const Element::RR &Arr, const Vector::RC &Arc,
                      const Matrix::CC &Acc) {
        //m_g = !Arc.Empty();
        m_g = g;
        Je.m_J.m_Jc.GetTranspose(m_JcT);
        const int N = m_JcT.Size(), _N = N + 1;
#ifdef CFG_DEBUG
        if (m_g) {
          UT_ASSERT(Arc.Size() == N);
        }
        UT_ASSERT(Acc.GetRows() == N && Acc.GetColumns() == N);
#endif
        m_JTAc0.Resize(N);
        m_Aec.Resize(N);
        if (m_g) {
          Element::RR wArr;
          Element::RC wArc;
          LA::AlignedMatrix2x3f SArp, SArr, _Arp, _Arr;
          LA::AlignedMatrix3x3f _JTAc;
          LA::AlignedVector3f _Aec;
          SArp.MakeZero();
          Arr.GetScaled(w[0], wArr);
          const LA::AlignedMatrix2x3f &Jr = Je.m_J.m_Jr;
          LA::AlignedMatrix2x3f::AB(wArr, Jr, SArr);
          const LA::Vector2f &er = Je.m_e.m_er;
          LA::SymmetricMatrix2x2f::Ab(wArr, er, m_Aer);
          const xp128f erx = xp128f::get(er.x()), ery = xp128f::get(er.y());
          for (int i = 0; i < N; ++i) {
            Arc[i].GetScaled(w, wArc);
            wArc.Get(_Arp, _Arr);
            const Element::JC &_JcT = m_JcT[i];
            LA::AlignedMatrix2x3f::AddABTTo(_Arp, _JcT.m_Jpp1, SArp);
            LA::AlignedMatrix2x3f::AddABTTo(_Arp, _JcT.m_Jpr1, SArr);
            LA::AlignedMatrix2x3f::AddABTTo(_Arr, _JcT.m_Jrr1, SArr);
            const Element::EC &_ec = Je.m_e.m_ec[i];
            LA::AlignedMatrix2x3f::AddAbTo(_Arp, _ec.m_ep, m_Aer);
            LA::AlignedMatrix2x3f::AddAbTo(_Arr, _ec.m_er, m_Aer);

            Element::CC &JTAc = m_JTAc0[i];
            JTAc.MakeZero3x6();
            LA::AlignedMatrix2x3f::ATB(Jr, _Arp, _JTAc);  JTAc.Set30(_JTAc);
            LA::AlignedMatrix2x3f::ATB(Jr, _Arr, _JTAc);  JTAc.Set33(_JTAc);
            Element::C &Aec = m_Aec[i];
            LA::AlignedMatrix2x3f::ATb(_Arp, erx, ery, _Aec);   Aec.Set012(_Aec);
            LA::AlignedMatrix2x3f::ATb(_Arr, erx, ery, _Aec);   Aec.Set345(_Aec);
          }
          m_JTArT0.Set(SArp, SArr);
        } else {
          m_JTAc0.MakeZero();
          m_Aec.MakeZero();
        }
        Element::CC wAcc;
        LA::AlignedMatrix3x3f _App, _Apr, _Arp, _Arr;
        m_JTAc.Resize(N, N, true);
        for (int i = 0; i < N; ++i) {
          Element::CC &JTAc1 = m_JTAc0[i];
          Element::C &Aec = m_Aec[i];
          for (int j = 0; j < N; ++j) {
            const Element::JC &_JcT = m_JcT[j];
            if (i <= j) {
              Acc[i][j].GetScaled(w, wAcc);
            } else {
              Acc[j][i].GetScaled(w, wAcc);
              wAcc.Transpose();
            }
            wAcc.Get(_App, _Apr, _Arp, _Arr);
            LA::AlignedMatrix6x6f::AddABTTo00(_JcT.m_Jpp1, _App, JTAc1);
            LA::AlignedMatrix6x6f::AddABTTo03(_JcT.m_Jpp1, _Arp, JTAc1);
            LA::AlignedMatrix6x6f::AddABTTo30(_JcT.m_Jpr1, _App, JTAc1);
            LA::AlignedMatrix6x6f::AddABTTo30(_JcT.m_Jrr1, _Apr, JTAc1);
            LA::AlignedMatrix6x6f::AddABTTo33(_JcT.m_Jpr1, _Arp, JTAc1);
            LA::AlignedMatrix6x6f::AddABTTo33(_JcT.m_Jrr1, _Arr, JTAc1);
            if (j <= i) {
              Element::CC &JTAc2 = m_JTAc[j][i];
              LA::AlignedMatrix6x6f::ABTTo00(_JcT.m_Jpp2, _App, JTAc2);
              LA::AlignedMatrix6x6f::ABTTo03(_JcT.m_Jpp2, _Arp, JTAc2);
              LA::AlignedMatrix6x6f::ABTTo30(_JcT.m_Jrr2, _Apr, JTAc2);
              LA::AlignedMatrix6x6f::ABTTo33(_JcT.m_Jrr2, _Arr, JTAc2);
            }
            const Element::EC &_ec = Je.m_e.m_ec[j];
            LA::AlignedMatrix3x3f::AddAbTo(_App, _ec.m_ep, &Aec.v0());
            LA::AlignedMatrix3x3f::AddAbTo(_Apr, _ec.m_er, &Aec.v0());
            LA::AlignedMatrix3x3f::AddAbTo(_Arp, _ec.m_ep, &Aec.v3());
            LA::AlignedMatrix3x3f::AddAbTo(_Arr, _ec.m_er, &Aec.v3());
          }
        }
      }
      inline void Get(const LA::AlignedMatrix2x3f &Jr, const xp128f &w, const Element::R &br,
                      const Vector::C &bc, Matrix::CC *A, Vector::C *b) const {
        const int N = m_JcT.Size(), _N = N + 1;
#ifdef CFG_DEBUG
        UT_ASSERT(bc.Size() == N);
#endif
        A->Resize(_N, _N, true);
        b->Resize(_N);
        Matrix::CC &_A = *A;
        Vector::C &_b = *b;

        Element::CC &A00 = _A[0][0];
        Element::C &b0 = _b[0];
        if (m_g) {
          Element::R wbr;
          LA::AlignedMatrix2x3f _AJrp, _AJrr;
          LA::AlignedMatrix3x3f _A;
          LA::AlignedVector3f _b;
          m_JTArT0.Get(_AJrp, _AJrr);
          A00.MakeZero3x3();
          LA::AlignedMatrix2x3f::ATB(_AJrp, Jr, _A);        A00.Set03(_A);
          LA::AlignedMatrix2x3f::ATBToUpper(_AJrr, Jr, _A); A00.Set33(_A);
          Element::R Aepbr;
          br.GetScaled(w[0], wbr);
          LA::Vector2f::apb(wbr, m_Aer, Aepbr);
          b0.MakeZero012();
          LA::AlignedMatrix2x3f::ATb(Jr, Aepbr, _b);
          b0.Set345(_b);
        } else {
          A00.MakeZero();
          b0.MakeZero();
        }
        Element::C wbc;
        LA::AlignedMatrix3x3f _JTApp, _JTApr, _JTArp, _JTArr;
        Element::C Aepbc;
        LA::AlignedVector3f Aepbp, Aepbr;
        for (int i = 0, _i = 1; i < N; ++i, ++_i) {
          const Element::JC &JTi = m_JcT[i];
          m_JTAc0[i].Get(_JTApp, _JTApr, _JTArp, _JTArr);
          LA::SymmetricMatrix6x6f::AddABTTo00(_JTApp, JTi.m_Jpp1, A00);
          LA::AlignedMatrix6x6f::AddABTTo03(_JTApp, JTi.m_Jpr1, A00);
          LA::AlignedMatrix6x6f::AddABTTo03(_JTApr, JTi.m_Jrr1, A00);
          LA::SymmetricMatrix6x6f::AddABTTo33(_JTArp, JTi.m_Jpr1, A00);
          LA::SymmetricMatrix6x6f::AddABTTo33(_JTArr, JTi.m_Jrr1, A00);
          Element::CC &A0i = _A[0][_i];
          LA::AlignedMatrix6x6f::ABTTo00(_JTApp, JTi.m_Jpp2, A0i);
          LA::AlignedMatrix6x6f::ABTTo03(_JTApr, JTi.m_Jrr2, A0i);
          LA::AlignedMatrix6x6f::ABTTo30(_JTArp, JTi.m_Jpp2, A0i);
          LA::AlignedMatrix6x6f::ABTTo33(_JTArr, JTi.m_Jrr2, A0i);
          bc[i].GetScaled(w, wbc);
          LA::AlignedVector6f::apb(wbc, m_Aec[i], Aepbc);
          Aepbc.Get(Aepbp, Aepbr);
          LA::AlignedMatrix3x3f::AddAbTo(JTi.m_Jpp1, Aepbp, &b0.v0());
          LA::AlignedMatrix3x3f::AddAbTo(JTi.m_Jpr1, Aepbp, &b0.v3());
          LA::AlignedMatrix3x3f::AddAbTo(JTi.m_Jrr1, Aepbr, &b0.v3());
          Element::C &bi = _b[_i];
          LA::AlignedMatrix3x3f::Ab(JTi.m_Jpp2, Aepbp, &bi.v0());
          LA::AlignedMatrix3x3f::Ab(JTi.m_Jrr2, Aepbr, &bi.v3());
        }
        for (int i = 0, _i = 1; i < N; ++i, ++_i) {
          const Element::CC *JTAi = m_JTAc[i];
          JTAi[i].Get(_JTApp, _JTApr, _JTArp, _JTArr);
          const Element::JC &JTi = m_JcT[i];
          Element::CC *Ai = _A[_i], &Aii = Ai[_i];
          LA::SymmetricMatrix6x6f::ABTTo00(_JTApp, JTi.m_Jpp2, Aii);
          LA::AlignedMatrix6x6f::ABTTo03(_JTApr, JTi.m_Jrr2, Aii);
          LA::AlignedMatrix6x6f::ABTTo30(_JTArp, JTi.m_Jpp2, Aii);
          LA::SymmetricMatrix6x6f::ABTTo33(_JTArr, JTi.m_Jrr2, Aii);
          for (int j = i + 1, _j = j + 1; j < N; ++j, ++_j) {
            const Element::JC &JTj = m_JcT[j];
            Element::CC &Aij = Ai[_j];
            JTAi[j].Get(_JTApp, _JTApr, _JTArp, _JTArr);
            LA::AlignedMatrix6x6f::ABTTo00(_JTApp, JTj.m_Jpp2, Aij);
            LA::AlignedMatrix6x6f::ABTTo03(_JTApr, JTj.m_Jrr2, Aij);
            LA::AlignedMatrix6x6f::ABTTo30(_JTArp, JTj.m_Jpp2, Aij);
            LA::AlignedMatrix6x6f::ABTTo33(_JTArr, JTj.m_Jrr2, Aij);
          }
        }
        A->SetLowerFromUpper();
      }
     public:
      bool m_g;
      Vector::JC m_JcT;
      Element::RC m_JTArT0;
      Vector::CC m_JTAc0;
      Matrix::CC m_JTAc;
      Element::R m_Aer;
      Vector::C m_Aec;
    };
   public:
    inline Factor() {}
    inline Factor(const Factor &A) { *this = A; }
    inline void operator = (const Factor &A) {
      m_Je = A.m_Je;
      m_w = A.m_w;
      m_F = A.m_F;
      m_A.Set(A.m_A);
      m_b.Set(A.m_b);
    }
    //inline void MakeZero() {
    //  m_Je.MakeZero();
    //  m_w = 0.0f;
    //  m_F = 0.0f;
    //  m_A.MakeZero();
    //  m_b.MakeZero();
    //}
    inline void MakeZero(const int N/* = -1*/) {
      m_Je.MakeZero();
      m_w = 0.0f;
      m_F = 0.0f;
      if (N >= 0) {
        const int _N = N + 1;
        m_A.Resize(_N, _N, true, true);
        m_b.Resize(_N);
      }
      m_A.MakeZero();
      m_b.MakeZero();
    }
    inline void Bind(void *data, const int N) {
      const int _N = N + 1;
      m_Je.Bind(data, N);
      m_A.Bind(m_Je.BindNext(), _N, _N, true);
      m_b.Bind(m_A.BindNext(), _N);
    }
    inline void* BindNext() { return m_b.BindNext(); }
    inline int BindSize(const int N) const {
      const int _N = N + 1;
      return m_Je.BindSize(N) + m_A.BindSize(_N, _N, true) + m_b.BindSize(_N);
    }
    inline void Swap(Matrix::CC &A, Vector::C &b) {
      m_A.Swap(A);
      m_b.Swap(b);
    }
    inline void Get(Matrix::CC &A, Vector::C &b) const {
      A.Set(m_A);
      b.Set(m_b);
    }
    inline void SaveB(FILE *fp) const {
      m_Je.SaveB(fp);
      UT::SaveB(m_w, fp);
      UT::SaveB(m_F, fp);
      m_A.SaveB(fp);
      m_b.SaveB(fp);
    }
    inline void LoadB(FILE *fp) {
      m_Je.LoadB(fp);
      UT::LoadB(m_w, fp);
      UT::LoadB(m_F, fp);
      m_A.LoadB(fp);
      m_b.LoadB(fp);
    }
    inline void AssertConsistency() const {
      m_Je.AssertConsistency();
      const int N = m_Je.Size(), _N = N + 1;
      UT_ASSERT(m_A.GetRows() == _N && m_A.GetColumns() == _N);
      m_A.AssertSymmetric();
    }
   public:
    ErrorJacobian m_Je;
    float m_w, m_F;
    Matrix::CC m_A;
    Vector::C m_b;
  };
  class Reduction {
   public:
    inline void Bind(void *data, const int N) { m_e.Bind(data, N); }
    inline void* BindNext() { return m_e.BindNext(); }
    inline int BindSize(const int N) const { return m_e.BindSize(N); }
   public:
    Error m_e;
    float m_F, m_dF;
  };
  class ESError : public LA::AlignedVector3f {
   public:
    inline ESError() {}
    inline ESError(const LA::AlignedVector3f &e, const float s = 1.0f) {
      if (s == 1.0f) {
        *((LA::AlignedVector3f *) this) = e;
      } else {
        e.GetScaled(s, *this);
      }
    }
    inline void Print(const bool l = true) const {
      if (l) {
        UT::Print("%f %f %f", x(), y(), z());
      } else {
        UT::Print("%.2f %.2f %.2f", x(), y(), z());
      }
    }
  };
  class ES : public UT::ES<float, int> {
   public:
    inline void Initialize() {
      UT::ES<float, int>::Initialize();
      m_ESp.Initialize();
      m_ESr.Initialize();
    }
    inline void Accumulate(const Error &e, const float F, const int iFrm = -1) {
      UT::ES<float, int>::Accumulate(F, F, iFrm);
      if (e.m_er.Valid())
        m_ESr.Accumulate(ESError(LA::AlignedVector3f(e.m_er, 0.0f), UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
      const int N = e.Size();
      for (int i = 0; i < N; ++i) {
        const Element::EC &ec = e.m_ec[i];
        m_ESp.Accumulate(ESError(ec.m_ep), -1.0f, iFrm);
        m_ESr.Accumulate(ESError(ec.m_er, UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
      }
    }
    inline void Print(const std::string str = "", const bool l = true) const {
      if (!Valid()) {
        return;
      }
      UT::ES<float, int>::Print(str + "ec = ", true, l);
      const std::string _str(str.size() + 18, ' ');
      if (m_ESp.Valid()) {
        m_ESp.Print(_str + "ep = ", false, l);
      }
      if (m_ESr.Valid()) {
        m_ESr.Print(_str + "er = ", false, l);
      }
    }
   public:
    UT::ES<ESError, int> m_ESp, m_ESr;
  };

 public:

  inline Pose() {}
  inline Pose(const Pose &Z) { *this = Z; }
  inline void operator = (const Pose &Z) {
    m_iKFr = Z.m_iKFr;
    //m_RrT = Z.m_RrT;
    m_iKFs = Z.m_iKFs;
    m_Zps.Set(Z.m_Zps);
    m_Arr = Z.m_Arr;
    m_Arc.Set(Z.m_Arc);
    m_Acc.Set(Z.m_Acc);
    m_br = Z.m_br;
    m_bc.Set(Z.m_bc);
    m_xTb = Z.m_xTb;
  }
  inline bool operator < (const int iKFr) const { return m_iKFr < iKFr; }

  inline bool Valid() const { return m_iKFr != -1; }
  inline bool Invalid() const { return m_iKFr == -1; }
  inline void Invalidate() {
    m_iKFr = -1;
    m_iKFs.resize(0);
    m_Zps.Resize(0);
    m_Arr.Invalidate();
    m_Arc.Resize(0);
    m_Acc.Resize(0, 0, true);
    m_br.Invalidate();
    m_bc.Resize(0);
  }

  inline void Initialize(const float w, const int iKFr, const Rigid3D &Tr, const float s2r,
                         const bool newKF, const Rigid3D *T0 = NULL,
                         const float s2cp = 0.0f, const float s2cr = 0.0f) {
    m_iKFr = iKFr;
    //Rr.GetTranspose(m_RrT);
    const float arr = UT::Inverse(s2r, w);
    m_Arr.Set(arr, 0.0f, arr);
    m_br.MakeZero();
    if (newKF) {
      m_iKFs.resize(0);
      m_Zps.Resize(0);
      m_Arc.Resize(0);
      m_Acc.Resize(0, 0, true);
      m_bc.Resize(0);
    } else {
      m_iKFs.resize(1);         m_iKFs[0] = INT_MAX;
      m_Zps.Resize(1);          m_Zps[0].Invalidate();
      m_Arc.Resize(1);          m_Arc[0].MakeZero();
      m_Acc.Resize(1, 1, true); m_Acc[0][0].MakeZero();
      m_bc.Resize(1);           m_bc[0].MakeZero();
      if (T0) {
        Rigid3D::ABI(Tr, *T0, m_Zps[0]);
      }
      m_Acc[0][0].SetDiagonal(UT::Inverse(s2cp, w), UT::Inverse(s2cr, w));
    }
    Tr.Rotation3D::GetTranspose(m_Zps.Push());
  }
  inline bool Initialize(const float w, const int iKF1, const int iKF2, const Rigid3D &T,
                         const LA::AlignedMatrix6x6f &S) {
    m_Acc.Resize(1, 1, true);
    Element::CC &A = m_Acc[0][0];
    if (!S.GetInverseLDL(A, NULL)) {
      return false;
    }
    A *= w;
    m_iKFr = iKF1;
    //m_RrT.Invalidate();
    m_iKFs.resize(1);
    m_iKFs[0] = iKF2;
    m_Zps.Resize(1);
    T.GetInverse(m_Zps[0]);
    m_Arr.Invalidate();
    m_Arc.Resize(0);
    m_br.Invalidate();
    m_bc.Resize(1);
    m_bc.MakeZero();
    return true;
  }

#ifdef CFG_DEBUG
  inline void DebugSetMeasurement(const Rigid3D &Tr, const bool newKF, const Rigid3D *T0 = NULL) {
    Tr.Rotation3D::GetTranspose(m_Zps.Back());
    if (!newKF && T0) {
      Rigid3D::ABI(Tr, *T0, m_Zps[0]);
    }
  }
#endif
  
  inline void Insert(const float w, const int i, const int iKF, const Rigid3D &T, const float s2p,
                     const float s2r, AlignedVector<float> *work) {
#ifdef CFG_DEBUG
    UT_ASSERT(i < static_cast<int>(m_iKFs.size()));
#endif
    m_iKFs.insert(m_iKFs.begin() + i, iKF);
    m_Zps.Insert(i);
    T.GetInverse(m_Zps[i]);
    m_Arc.InsertZero(i);
    m_Acc.InsertZero(i, 1, work);
    m_Acc[i][i].SetDiagonal(UT::Inverse(s2p, w), UT::Inverse(s2r, w));
    m_bc.InsertZero(i);
  }
  inline void Erase(const int i) {
    m_iKFs.erase(m_iKFs.begin() + i);
    m_Zps.Erase(i);
    m_Arc.Erase(i);
    m_Acc.Erase(i);
    m_bc.Erase(i);
  }
  inline void DeleteKeyFrame(const int iKF, const std::vector<int>::iterator *i = NULL) {
    if (m_iKFr == iKF) {
      //Invalidate();
      m_iKFr = -1;
    } else if (m_iKFr > iKF) {
      --m_iKFr;
    }
    const std::vector<int>::iterator _i = i ? *i : std::lower_bound(m_iKFs.begin(),
                                                                    m_iKFs.end(), iKF);
    if (_i == m_iKFs.end()) {
      return;
    }
    const bool k = *_i == iKF;
    for (std::vector<int>::iterator jt = k ? _i + 1 : _i;
         jt != m_iKFs.end() && *jt != INT_MAX; ++jt) {
      --*jt;
    }
    if (Invalid() || !k) {
      return;
    }
    const int j = static_cast<int>(_i - m_iKFs.begin());
    m_iKFs.erase(_i);
    m_Zps.Erase(j);
    m_Arc.Erase(j);
    m_Acc.Erase(j);
    m_bc.Erase(j);
  }
  inline void SetPose(const Rigid3D &Tr, const int i, const Rigid3D &Ti) {
#ifdef CFG_DEBUG
    UT_ASSERT(i < static_cast<int>(m_iKFs.size()));
#endif
    Rigid3D::ABI(Tr, Ti, m_Zps[i]);
  }
//  inline void GetReferencePose(const Rigid3D &Tr, Rigid3D *_Tr, Rigid3D *TrI) const {
//#ifdef CFG_DEBUG
//    UT_ASSERT(m_Zps.Size() == static_cast<int>(m_iKFs.size()) + 1);
//#endif
//    //const Rotation3D &RrT = m_RrT;
//    const Rotation3D RrT = m_Zps.Back();
//    Rotation3D dR = RrT * Tr;
//    dR.SetEulerAngleZ(-dR.GetRodriguesZ());
//    Rotation3D::AB(dR, RrT, *TrI);
//    //*TrI = RrT;
//    const Point3D pr = Tr.GetPosition();
//    TrI->SetTranslation(pr);
//    TrI->GetInverse(*_Tr);
//  }
  inline void GetReferencePose(const Rigid3D &Tr, Rigid3D *_Tr, Rigid3D *TrI) const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_Zps.Size() == static_cast<int>(m_iKFs.size()) + 1);
#endif
    *TrI = m_Zps.Back();
    const Point3D pr = Tr.GetPosition();
    TrI->SetTranslation(pr);
    TrI->GetInverse(*_Tr);
  }
//  inline void GetPose(const Rigid3D &TrI, const int i, Rigid3D *Tri, Rigid3D *Ti) const {
//#ifdef CFG_DEBUG
//    UT_ASSERT(i < static_cast<int>(m_iKFs.size()));
//#endif
//    m_Zps[i].GetInverse(*Tri);
//    Rigid3D::ABI(*Tri, TrI, *Ti);
//  }

  inline Rotation3D GetReferenceRotationState(const Rotation3D &R) const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_Zps.Size() == static_cast<int>(m_iKFs.size()) + 1);
#endif
    return R.GetTranspose();
  }
  inline Rotation3D GetReferenceRotationMeasurement(const float *x/* = NULL*/,
                                                    const float eps) const {
#ifdef CFG_DEBUG
    UT_ASSERT(m_Zps.Size() == static_cast<int>(m_iKFs.size()) + 1);
#endif
    if (x) {
      Rotation3D dR;
      const LA::Vector2f _x(x);
      dR.SetRodriguesXY(_x, eps);
      dR.Transpose();
      return dR * m_Zps.Back();
    } else {
      return m_Zps.Back();
    }
  }
  inline LA::AlignedVector3f GetReferenceRotationError(const Rotation3D &R, const float *x/* = NULL*/,
                                                       const float eps) const {
    const Rotation3D eR = GetReferenceRotationMeasurement(x, eps) / GetReferenceRotationState(R);
    const LA::Vector2f e = eR.GetRodriguesXY(eps);
    return LA::AlignedVector3f(e.x(), e.y(), 0.0f);
  }
  inline Rotation3D GetRotationState(const Rotation3D &R1, const Rotation3D &R2) const {
    return R1 / R2;
  }
  inline const Rotation3D GetRotationMeasurement(const int i, const float *x/* = NULL*/,
                                                 const float eps) const {
    if (x) {
      Rotation3D dR;
      const LA::AlignedVector3f _x(x);
      dR.SetRodrigues(_x, eps);
      dR.Transpose();
      return dR * m_Zps[i];
    } else {
      return m_Zps[i];
    }
  }
  inline LA::AlignedVector3f GetRotationError(const Rotation3D &R1, const Rotation3D &R2,
                                              const int i, const float *x/* = NULL*/,
                                              const float eps) const {
    const Rotation3D eR = GetRotationMeasurement(i, x, eps) / GetRotationState(R1, R2);
    return eR.GetRodrigues(eps);
  }
  inline LA::AlignedVector3f GetPositionState(const Rigid3D &C1, const Rigid3D &C2) const {
    const Point3D p2 = C2.GetPosition();
    return C1.GetTranslation() + C1.GetAppliedRotation(p2);
  }
  inline LA::AlignedVector3f GetPositionMeasurement(const int i, const float *x = NULL) const {
    const LA::AlignedVector3f p12 = m_Zps[i].GetTranslation();
    if (x) {
      return p12 + LA::AlignedVector3f(x);
    } else {
      return p12;
    }
  }
  inline LA::AlignedVector3f GetPositionError(const Rigid3D &C1, const Rigid3D &C2,
                                              const int i, const float *x = NULL) const {
    return GetPositionState(C1, C2) - GetPositionMeasurement(i, x);
  }
  inline void GetError(const Rigid3D &C1, const Rigid3D &C2, const int i, Element::EC *e,
                       const float eps) const {
    GetError(C1, C2, m_Zps[i], e, eps);
  }
  static inline void GetError(const Rigid3D &C1, const Rigid3D &C2, const Rigid3D &Z21,
                              Element::EC *e, const float eps) {
    const LA::AlignedVector3f t1 = C1.GetTranslation();
    const LA::AlignedVector3f t2 = C2.GetTranslation();
    const Rotation3D R21 = Rotation3D(C1 / C2);
    R21.Apply(t2, e->m_ep);
    LA::AlignedVector3f::amb(t1, e->m_ep, e->m_ep);
    const LA::AlignedVector3f p12 = Z21.GetTranslation();
    e->m_ep -= p12;
    const Rotation3D eR = Rotation3D(Z21) / R21;
    eR.GetRodrigues(e->m_er, eps);
  }
  inline void GetError(const AlignedVector<Rigid3D> &Cs, Error *e, const float eps) const {
    Rotation3D eR, Rt;
    LA::AlignedVector3f p2, p12;
    const Rigid3D &C1 = Cs[m_iKFr];
    const int N = static_cast<int>(m_iKFs.size());
    //if (m_Zps.Size() == N) {
    //  e->m_er.Invalidate();
    //} else {
    //  Rotation3D::AB(m_RrT, C1, eR);
    //  eR.GetRodriguesXY(e->m_er);
    //}
    if (m_Zps.Size() == N) {
      e->m_er.Invalidate();
    } else {
#ifdef CFG_DEBUG
      UT_ASSERT(m_Zps.Size() == N + 1);
#endif
      Rotation3D::AB(m_Zps.Back(), C1, eR);
      eR.GetRodriguesXY(e->m_er, eps);
    }
    const LA::AlignedVector3f t1 = C1.GetTranslation();
    e->Resize(N);
    for (int i = 0; i < N; ++i) {
      const Rigid3D &T21 = m_Zps[i];
      const int iKF = m_iKFs[i];
      const Rigid3D &C2 = iKF == INT_MAX ? Cs.Back() : Cs[iKF];
      Element::EC &ec = e->m_ec[i];
#if 0
      Rotation3D::ABT(C1, C2, R21);
      C2.GetTranslation(t2);
      R21.Apply(t2, ec.m_ep);
      LA::AlignedVector3f::amb(t1, ec.m_ep, ec.m_ep);
      T21.GetTranslation(p12);
      ec.m_ep -= p12;
      Rotation3D::ABT(T21, R21, eR);
      eR.GetRodrigues(ec.m_er);
#else
      C2.GetPosition(p2);
      C1.ApplyRotation(p2, ec.m_ep);
      ec.m_ep += t1;
      T21.GetTranslation(p12);
      ec.m_ep -= p12;
      Rotation3D::AB(T21, C2, Rt);
      Rotation3D::ABT(Rt, C1, eR);
      eR.GetRodrigues(ec.m_er, eps);
#endif
    }
  }
  inline void GetError(const ErrorJacobian &Je, const std::vector<const LA::AlignedVector3f *> &xps,
                       const std::vector<const LA::AlignedVector3f *> &xrs, Error *e) const {
    const LA::AlignedVector3f *xp1 = xps[m_iKFr], *xr1 = xrs[m_iKFr];
    e->m_er = Je.m_e.m_er;
    const int N = static_cast<int>(m_iKFs.size());
    //if (xr1 && !m_Arc.Empty()) {
    if (xr1 && m_Zps.Size() != N) {
      LA::AlignedMatrix2x3f::AddAbTo(Je.m_J.m_Jr, *xr1, e->m_er);
      //LA::Matrix2x3f::AddAbTo(Je.m_J.m_Jr, *xr1, e->m_er);
    }
    e->Resize(N);
    for (int i = 0; i < N; ++i) {
      const int iKF2 = m_iKFs[i];
      const LA::AlignedVector3f *xp2 = xps[iKF2], *xr2 = xrs[iKF2];
      Element::EC &ec = e->m_ec[i];
      ec = Je.m_e.m_ec[i];
      const Element::JC &Jc = Je.m_J.m_Jc[i];
      if (xp1) {
        LA::AlignedMatrix3x3f::AddAbTo(Jc.m_Jpp1, *xp1, (float *) &ec.m_ep);
      }
      if (xr1) {
        LA::AlignedMatrix3x3f::AddAbTo(Jc.m_Jrr1, *xr1, (float *) &ec.m_er);
        LA::AlignedMatrix3x3f::AddAbTo(Jc.m_Jpr1, *xr1, (float *) &ec.m_ep);
      }
      if (xp2) {
        LA::AlignedMatrix3x3f::AddAbTo(Jc.m_Jpp2, *xp2, (float *) &ec.m_ep);
      }
      if (xr2) {
        LA::AlignedMatrix3x3f::AddAbTo(Jc.m_Jrr2, *xr2, (float *) &ec.m_er);
      }
    }
  }
  inline void GetErrorJacobian(const AlignedVector<Rigid3D> &Cs, ErrorJacobian *Je
                               /*, LA::AlignedMatrix2x3f *Jr*/, const float eps) const {
    Rotation3D eR, Rt;
    LA::AlignedVector3f er, t1, p2, p12;
    const Rigid3D &C1 = Cs[m_iKFr];
    C1.GetTranslation(t1);
    const int N = static_cast<int>(m_iKFs.size());
    //if (N == m_Zps.Size()) {
    //  Je->m_e.m_er.Invalidate();
    //  Je->m_J.m_Jr.Invalidate();
    //} else {
    //  Rotation3D::AB(m_RrT, C1, eR);
    //  eR.GetRodrigues(er);
    //  Je->m_e.m_er.Set(er);
    //  Rotation3D::GetRodriguesJacobianInverseXY(er, Je->m_J.m_Jr);
    //  Je->m_J.m_Jr = Je->m_J.m_Jr * eR;
    //}
    if (N == m_Zps.Size()) {
      Je->m_e.m_er.Invalidate();
      Je->m_J.m_Jr.Invalidate();
    } else {
      Rotation3D::AB(m_Zps.Back(), C1, eR);
      eR.GetRodrigues(er, eps);
      Je->m_e.m_er.Set(er);
      LA::AlignedMatrix2x3f &Jr = Je->m_J.m_Jr;
      Rotation3D::GetRodriguesJacobianInverseXY(er, Jr, eps);
      Jr = Jr * eR;
      //Je->m_J.m_Jr.Set(*Jr);
    }
    Je->m_e.Resize(N);
    Je->m_J.Resize(N);
    for (int i = 0; i < N; ++i) {
      const int iKF = m_iKFs[i];
      const Rigid3D &C2 = iKF == INT_MAX ? Cs.Back() : Cs[iKF];
      const Rigid3D &T21 = m_Zps[i];
      Element::EC &ec = Je->m_e.m_ec[i];
      Element::JC &Jc = Je->m_J.m_Jc[i];
      C2.GetPosition(p2);
      C1.ApplyRotation(p2, ec.m_ep);
      ec.m_ep += t1;
      SkewSymmetricMatrix::AB(ec.m_ep, C1, Jc.m_Jpr1);
      T21.GetTranslation(p12);
      ec.m_ep -= p12;
      Jc.m_Jpp2 = C1;
      Jc.m_Jpp2.GetMinus(Jc.m_Jpp1);
      Rotation3D::AB(T21, C2, Rt);
      Rotation3D::ABT(Rt, C1, eR);
      eR.GetRodrigues(ec.m_er, eps);
      Rotation3D::GetRodriguesJacobianInverse(ec.m_er, Jc.m_Jrr1, eps);
      LA::AlignedMatrix3x3f::AB(Jc.m_Jrr1, Rt, Jc.m_Jrr2);
      Jc.m_Jrr2.GetMinus(Jc.m_Jrr1);
    }
  }
  inline void GetFactor(const float w, const AlignedVector<Rigid3D> &Cs, Factor *A,
                        Factor::Auxiliary *U, const float eps) const {
    //LA::AlignedMatrix2x3f Jr;
    GetErrorJacobian(Cs, &A->m_Je/*, &Jr*/, eps);
    const xp128f _w = xp128f::get(w);
    U->Set(m_Zps.Size() != static_cast<int>(m_iKFs.size()), A->m_Je/*, Jr*/,
           _w, m_Arr, m_Arc, m_Acc);
    A->m_w = w;
    A->m_F = GetCost(w, A->m_Je.m_e, U->m_Aer, U->m_Aec);
    U->Get(A->m_Je.m_J.m_Jr, _w, m_br, m_bc, &A->m_A, &A->m_b);
  }
  inline float GetCost(const float w, const Error &e, const Element::R &Aer,
                       const Vector::C &Aec) const {
    float F = 0.0f;
    const float s = w + w;
    const int N = e.Size();
    //if (!m_Arc.Empty()) {
    if (m_Zps.Size() != N) {
      Element::R br;
      m_br.GetScaled(s, br);
      br += Aer;
      F += e.m_er.Dot(br);
    }
    const xp128f _s = xp128f::get(s);
#ifdef CFG_DEBUG
    UT_ASSERT(m_bc.Size() == N && Aec.Size() == N);
#endif
    Element::C bc;
    LA::AlignedVector6f ec;
    for (int i = 0; i < N; ++i) {
      m_bc[i].GetScaled(_s, bc);
      bc += Aec[i];
      const Element::EC &_ec = e.m_ec[i];
      ec.Set(_ec.m_ep, _ec.m_er);
      F += ec.Dot(bc);
    }
    //return 0.5f * F;
    return F;
  }
  inline float GetCost(const float w, const Error &e) const {
    float F = 0.0f;
    //const float s = 0.5f;
    const int N = e.Size();
#ifdef CFG_DEBUG
    UT_ASSERT(m_bc.Size() == N);
#endif
    //const bool g = !m_Arc.Empty();
    const bool g = m_Zps.Size() != N;
    Element::R Sbr;
    if (g) {
      LA::SymmetricMatrix2x2f::Ab(m_Arr, e.m_er, Sbr);
    }
    LA::AlignedMatrix2x3f Agp, Agr;
    LA::AlignedMatrix3x3f App, Apr, Arp, Arr;
    LA::AlignedVector3f bp, br;
    LA::AlignedVector6f _ec;
    Element::C bc;
    const xp128f erx = xp128f::get(e.m_er.x()), ery = xp128f::get(e.m_er.y());
    //const xp128f _s = xp128f::get(s);
    for (int i = 0; i < N; ++i) {
      if (g) {
        m_Arc[i].Get(Agp, Agr);
        const Element::EC &ec = e.m_ec[i];
        LA::AlignedMatrix2x3f::AddAbTo(Agp, ec.m_ep, Sbr);
        LA::AlignedMatrix2x3f::AddAbTo(Agr, ec.m_er, Sbr);
        LA::AlignedMatrix2x3f::ATb(Agp, erx, ery, bp);
        LA::AlignedMatrix2x3f::ATb(Agr, erx, ery, br);
        bc.Set(bp, br);
      } else {
        bc.MakeZero();
      }
      for (int j = 0; j < N; ++j) {
        if (i <= j) {
          m_Acc[i][j].Get(App, Apr, Arp, Arr);
        } else {
          m_Acc[j][i].Get(App, Arp, Apr, Arr);
          App.Transpose();
          Apr.Transpose();
          Arp.Transpose();
          Arr.Transpose();
        }
        const Element::EC &ec = e.m_ec[j];
        LA::AlignedMatrix3x3f::AddAbTo(App, ec.m_ep, &bc.v0());
        LA::AlignedMatrix3x3f::AddAbTo(Apr, ec.m_er, &bc.v0());
        LA::AlignedMatrix3x3f::AddAbTo(Arp, ec.m_ep, &bc.v3());
        LA::AlignedMatrix3x3f::AddAbTo(Arr, ec.m_er, &bc.v3());
      }
      //bc *= _s;
      bc += m_bc[i];
      bc += m_bc[i];
      const Element::EC &ec = e.m_ec[i];
      _ec.Set(ec.m_ep, ec.m_er);
      F += _ec.Dot(bc);
    }
    if (g) {
      //Sbr *= s;
      Sbr += m_br;
      Sbr += m_br;
      F += e.m_er.Dot(Sbr);
    }
    return w * F;
  }
  inline float GetCost(const float w, const ErrorJacobian &Je,
                       const std::vector<const LA::AlignedVector3f *> &xps,
                       const std::vector<const LA::AlignedVector3f *> &xrs, Error *e) const {
    GetError(Je, xps, xrs, e);
    return GetCost(w, *e);
  }
  inline void GetReduction(const float w, const Factor &A, const AlignedVector<Rigid3D> &Cs,
                           const std::vector<const LA::AlignedVector3f *> &xps,
                           const std::vector<const LA::AlignedVector3f *> &xrs,
                           Reduction *Ra, Reduction *Rp, const float eps) const {
    GetError(Cs, &Ra->m_e, eps);
    GetError(A.m_Je, xps, xrs, &Rp->m_e);
    Ra->m_dF = A.m_F - (Ra->m_F = GetCost(w, Ra->m_e));
    Rp->m_dF = A.m_F - (Rp->m_F = GetCost(w, Rp->m_e));
  }
  inline void GetResidual(const AlignedVector<Rigid3D> &Cs, Error *e, Vector::C *ec, Element::R *rr,
                          Vector::C *rc, const float eps) const {
    GetError(Cs, e, eps);
    const int N = static_cast<int>(m_iKFs.size());
    Vector::C &_ec = *ec;
    _ec.Resize(N);
    for (int i = 0; i < N; ++i) {
      Element::EC &eci = e->m_ec[i];
      _ec[i].Set(eci.m_ep, eci.m_er);
    }
    Vector::C &_rc = *rc;
    _rc.Set(m_bc);
    if (m_Zps.Size() == N) {
      rr->MakeZero();
    } else {
      LA::SymmetricMatrix2x2f::Ab(m_Arr, e->m_er, *rr);
      for (int i = 0; i < N; ++i) {
        LA::AlignedMatrix2x6f::AddAbTo(m_Arc[i], _ec[i], *rr);
        LA::AlignedMatrix2x6f::AddATbTo(m_Arc[i], e->m_er, _rc[i]);
      }
      *rr += m_br;
    }
    LA::AlignedMatrix6x6f AT;
    for (int i = 0; i < N; ++i) {
      const LA::AlignedVector6f &eci = _ec[i];
      Element::C &rci = _rc[i];
      for (int j = i; j < N; ++j) {
        LA::AlignedMatrix6x6f::AddAbTo(m_Acc[i][j], _ec[j], (float *) &rci);
        if (j == i) {
          continue;
        }
        m_Acc[i][j].GetTranspose(AT);
        LA::AlignedMatrix6x6f::AddAbTo(AT, eci, (float *) &_rc[j]);
      }
    }
  }

  inline void Print(const bool e = false) const {
    m_Arr.Print(UT::String("Arr(%d,%d) = ", m_iKFr, m_iKFr), e);
    const int Nk = static_cast<int>(m_iKFs.size());
    for (int i = 0; i < Nk; ++i) {
      const int iKF = m_iKFs[i] == INT_MAX ? -1 : m_iKFs[i];
      m_Arc[i].Print(UT::String("Arc(%d,%d) = ", m_iKFr, iKF), e);
    }
    m_br.Print(UT::String("br(%d) = ", m_iKFr), e, true);
    for (int i = 0; i < Nk; ++i) {
      const int iKF = m_iKFs[i] == INT_MAX ? -1 : m_iKFs[i];
      for (int j = i; j < Nk; ++j) {
        const int jKF = m_iKFs[j] == INT_MAX ? -1 : m_iKFs[j];
        m_Acc[i][j].Print(UT::String("Acc(%d,%d) = ", iKF, jKF), e);
      }
      m_bc[i].Print(UT::String("bc(%d) = ", iKF), e);
    }
  }
  inline void PrintDiagonal(const bool e = false) const {
    m_Arr.PrintDiagonal(UT::String("Arr(%d,%d) = ", m_iKFr, m_iKFr), e);
    const int Nk = static_cast<int>(m_iKFs.size());
    for (int i = 0; i < Nk; ++i) {
      const int iKF = m_iKFs[i] == INT_MAX ? -1 : m_iKFs[i];
      m_Acc[i][i].PrintDiagonal(UT::String("Acc(%d,%d) = ", iKF, iKF), e);
    }
  }

  inline void SaveB(FILE *fp) const {
    UT::SaveB(m_iKFr, fp);
    //UT::SaveB(m_RrT, fp);
    UT::VectorSaveB(m_iKFs, fp);
    m_Zps.SaveB(fp);
    m_Arr.SaveB(fp);
    m_Arc.SaveB(fp);
    m_Acc.SaveB(fp);
    m_br.SaveB(fp);
    m_bc.SaveB(fp);
    UT::SaveB(m_xTb, fp);
  }
  inline void LoadB(FILE *fp) {
    UT::LoadB(m_iKFr, fp);
    //UT::LoadB(m_RrT, fp);
    UT::VectorLoadB(m_iKFs, fp);
    m_Zps.LoadB(fp);
    m_Arr.LoadB(fp);
    m_Arc.LoadB(fp);
    m_Acc.LoadB(fp);
    m_br.LoadB(fp);
    m_bc.LoadB(fp);
    UT::LoadB(m_xTb, fp);
  }
  inline void AssertConsistency() const {
    const int N = static_cast<int>(m_iKFs.size());
    //UT_ASSERT(Valid() && N > 0 || Invalid() && N == 0);
    for (int i = 1; i < N; ++i) {
      const int iKF = m_iKFs[i];
      UT_ASSERT(m_iKFs[i - 1] < iKF);
      UT_ASSERT(iKF != m_iKFr);
    }
    //UT_ASSERT(m_Zps.Size() == N);
    //UT_ASSERT(m_Arc.Empty() || m_Arc.Size() == N);
    if (m_Zps.Size() == N) {
      UT_ASSERT(m_Arc.Empty());
    } else {
      UT_ASSERT(m_Zps.Size() == N + 1 && m_Arc.Size() == N);
    }
    UT_ASSERT(m_Acc.GetRows() == N && m_bc.Size() == N);
    m_Acc.AssertSymmetric();
  }

  bool Marginalize(const int i, AlignedVector<float> *work, const float *eps = NULL);
  bool MarginalizeUninformative(const float w, const float s2p, const float s2r,
                                std::vector<int> *iks, AlignedVector<float> *work,
                                const float *eps = NULL);
  void SetPriorEquation(const Matrix::X &A, const Vector::X &b, const bool g = true);
  void GetPriorEquation(Matrix::X *A, Vector::X *b = NULL, const bool symmetric = true,
                        const bool g = true) const;
  bool GetPriorMeasurement(const Element::T w, Matrix::X *S, Vector::X *x/* = NULL*/,
                           Element::T *xTb/* = NULL*/, const Element::T *eps/* = NULL*/) const;
  bool GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                           float *xTb, AlignedVector<float> *work, const float *eps = NULL) const;

 public:

  int m_iKFr;
  //Rotation3D m_RrT;
  std::vector<int> m_iKFs;
  AlignedVector<Rigid3D> m_Zps;

  Element::RR m_Arr;
  Vector::RC m_Arc;
  Matrix::CC m_Acc;
  Element::R m_br;
  Vector::C m_bc;
  float m_xTb;

#ifdef CFG_DEBUG_EIGEN
 public:
  class EigenErrorJacobian {
   public:
    inline void Set(const Error &e) {
      const int N = e.Size();
      m_e.Resize(2 + N * 6);
      m_e.block<2, 1>(0, 0) = EigenVector2f(e.m_er);
      for (int i = 0, j = 2; i < N; ++i, j += 6) {
        const Element::EC &ec = e.m_ec[i];
        m_e.block<6, 1>(j, 0) = EigenVector6f(ec.m_ep, ec.m_er);
      }
    }
    inline void Set(const Jacobian &J) {
      const int N = J.Size(), Nx6 = N * 6;
      m_J.Resize(2 + Nx6, 6 + Nx6);
      m_J.setZero();
      m_J.block<2, 3>(0, 3) = EigenMatrix2x3f(J.m_Jr);

      EigenMatrix6x6f e_Jc1, e_Jc2;
      e_Jc1.setZero();
      e_Jc2.setZero();
      for (int i = 0, j = 2, k = 6; i < N; ++i, j += 6, k += 6) {
        const Element::JC &Jc = J.m_Jc[i];
        e_Jc1.block<3, 3>(0, 0) = EigenMatrix3x3f(Jc.m_Jpp1);
        e_Jc1.block<3, 3>(0, 3) = EigenMatrix3x3f(Jc.m_Jpr1);
        e_Jc1.block<3, 3>(3, 3) = EigenMatrix3x3f(Jc.m_Jrr1);
        e_Jc2.block<3, 3>(0, 0) = EigenMatrix3x3f(Jc.m_Jpp2);
        e_Jc2.block<3, 3>(3, 3) = EigenMatrix3x3f(Jc.m_Jrr2);
        m_J.block<6, 6>(j, 0) = e_Jc1;
        m_J.block<6, 6>(j, k) = e_Jc2;
      }
    }
    inline void Set(const ErrorJacobian &J) { Set(J.m_e); Set(J.m_J); }
    inline bool AssertEqual(const Error &e, const int verbose = 1,
                            const std::string str = "") const {
      const int N = e.Size();
      UT_ASSERT(m_e.Size() == 2 + N * 6);
      bool scc = true;
      if (e.m_er.Valid()) {
        const EigenVector2f e_er(m_e.block<2, 1>(0, 0));
        scc = e_er.AssertEqual(e.m_er, verbose, str + ".m_er") && scc;
      }
      for (int i = 0, j = 2; i < N; ++i, j += 6) {
        const Element::EC &ec = e.m_ec[i];
        const EigenVector6f e_ec1 = EigenVector6f(m_e.block<6, 1>(j, 0));
        const EigenVector6f e_ec2 = EigenVector6f(ec.m_ep, ec.m_er);
        scc = e_ec1.AssertEqual(e_ec2, verbose, UT::String(".m_ec[%d]", i)) && scc;
      }
      return scc;
    }
    inline bool AssertEqual(const Jacobian &J, const int verbose = 1,
                            const std::string str = "") const {
      const int N = J.Size(), Nx6 = N * 6;
      UT_ASSERT(m_J.GetRows() == 2 + Nx6 && m_J.GetColumns() == 6 + Nx6);
      bool scc = true;
      if (J.m_Jr.Valid()) {
        const EigenMatrix2x3f e_Jr(m_J.block<2, 3>(0, 3));
        scc = e_Jr.AssertEqual(J.m_Jr, verbose, str + ".m_Jr") && scc;
      }
      scc = EigenMatrixXf(m_J.block(0, 0, 2, 3)).AssertZero(verbose, str + ".Zero[r]",
                                                            -1.0f, -1.0f) && scc;
      scc = EigenMatrixXf(m_J.block(0, 6, 2, Nx6)).AssertZero(verbose, str + ".Zero[r]",
                                                              -1.0f, -1.0f) && scc;

      EigenMatrix6x12f e_Jc1, e_Jc2;
      e_Jc2.setZero();
      for (int i = 0, j = 2, k = 6; i < N; ++i, j += 6, k += 6) {
        const Element::JC &Jc = J.m_Jc[i];
        e_Jc1.block<6, 6>(0, 0) = m_J.block<6, 6>(j, 0);
        e_Jc1.block<6, 6>(0, 6) = m_J.block<6, 6>(j, k);
        e_Jc2.block<3, 3>(0, 0) = EigenMatrix3x3f(Jc.m_Jpp1);
        e_Jc2.block<3, 3>(0, 3) = EigenMatrix3x3f(Jc.m_Jpr1);
        e_Jc2.block<3, 3>(0, 6) = EigenMatrix3x3f(Jc.m_Jpp2);
        e_Jc2.block<3, 3>(3, 3) = EigenMatrix3x3f(Jc.m_Jrr1);
        e_Jc2.block<3, 3>(3, 9) = EigenMatrix3x3f(Jc.m_Jrr2);
        scc = e_Jc1.AssertEqual(e_Jc2, verbose, str + UT::String(".m_Jc[%d]", i)) && scc;
        const std::string _str = str + UT::String(".Zero[%d]", i);
        scc = EigenMatrixXf(m_J.block(j, 6, 6, k - 6)).AssertZero(verbose, _str,
                                                                  -1.0f, -1.0f) && scc;
        scc = EigenMatrixXf(m_J.block(j, k + 6, 6, Nx6 - k)).AssertZero(verbose, _str,
                                                                        -1.0f, -1.0f) && scc;
      }
      return scc;
    }
    inline bool AssertEqual(const ErrorJacobian &Je, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = AssertEqual(Je.m_e, verbose, str) && scc;
      scc = AssertEqual(Je.m_J, verbose, str) && scc;
      return scc;
    }
   public:
    EigenVectorXf m_e;
    EigenMatrixXf m_J;
  };
  class EigenFactor {
   public:
    inline void Set(const Factor &A) {
      m_A = EigenConvert(A.m_A);
      m_b = EigenConvert(A.m_b);
      m_F = A.m_F;
    }
    inline bool AssertEqual(const Factor &A, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = EigenAssertEqual(m_A, A.m_A, verbose, str + ".m_A") && scc;
      scc = EigenAssertEqual(m_b, A.m_b, verbose, str + ".m_b") && scc;
      scc = UT::AssertEqual(m_F, A.m_F, verbose, str + ".m_F") && scc;
      return scc;
    }
   public:
    EigenMatrixXf m_A;
    EigenVectorXf m_b;
    float m_F;
  };
  class EigenPrior {
   public:
    inline EigenPrior() {}
    inline EigenPrior(const EigenMatrixXf &e_A, const EigenVectorXf &e_b) : m_A(e_A), m_b(e_b) {}
    inline EigenPrior(const Element::RR &Arr, const Vector::RC &Arc, const Matrix::CC &Acc,
                      const Element::R &br, const Vector::C &bc, const float w = 1.0f,
                      const bool pad = false) {
      Set(Arr, Arc, Acc, br, bc, w, pad);
    }
    inline void Set(const Pose &Z, const bool pad = false) {
      Set(Z.m_Arr, Z.m_Arc, Z.m_Acc, Z.m_br, Z.m_bc, 1.0f, pad);
    }
    inline void Set(const Element::RR &Arr, const Vector::RC &Arc, const Matrix::CC &Acc,
                    const Element::R &br, const Vector::C &bc, const float w = 1.0f,
                    const bool pad = false) {
      const int N = bc.Size();
#ifdef CFG_DEBUG
      UT_ASSERT(Acc.GetRows() == N && Acc.GetColumns() == N);
      if (!Arc.Empty()) {
        UT_ASSERT(Arc.Size() == N);
      }
#endif
      const int Nx6 = N == 0 && pad ? 6 : N * 6;
      m_A.Resize(2 + Nx6, 2 + Nx6);
      m_A.MakeZero();
      m_b.Resize(2 + Nx6);
      m_b.MakeZero();
      if (Arr.Valid()) {
        m_A.block<2, 2>(0, 0) = EigenMatrix2x2f(Arr);
      }
      if (!Arc.Empty()) {
        const EigenMatrixXf e_Arc = EigenConvert(Arc);
        m_A.block(0, 2, 2, Nx6) = e_Arc;
        m_A.block(2, 0, Nx6, 2) = e_Arc.transpose();
      }
      if (N != 0) {
        m_A.block(2, 2, Nx6, Nx6) = EigenConvert(Acc);
        m_b.block(2, 0, Nx6, 1) = EigenConvert(bc);
      }
      if (br.Valid()) {
        m_b.block<2, 1>(0, 0) = EigenVector2f(br);
      }
      if (w != 1.0f) {
        m_A *= w;
        m_b *= w;
      }
    }
    inline void Get(Element::RR &Arr, Vector::RC &Arc, Matrix::CC &Acc,
                    Element::R &br, Vector::C &bc) const {
      const int Nrc = m_b.Size(), Nx6 = Nrc - 2, N = Nx6 / 6;
#ifdef CFG_DEBUG
      UT_ASSERT(Nx6 % 6 == 0);
      UT_ASSERT(m_A.GetRows() == Nrc && m_A.GetColumns() == Nrc);
#endif
      Arr = EigenMatrix2x2f(m_A.block<2, 2>(0, 0)).GetSymmetricMatrix2x2f();
      EigenConvert(EigenMatrixXf(m_A.block(0, 2, 2, Nx6)), Arc);
      EigenConvert(EigenMatrixXf(m_A.block(2, 2, Nx6, Nx6)), Acc);
      br = EigenVector2f(m_b.block<2, 1>(0, 0)).GetVector2f();
      EigenConvert(EigenVectorXf(m_b.block(2, 0, Nx6, 1)), bc);
    }
    inline void Marginalize(const int ipc) {
      EigenMatrixX<Element::T> e_Ab;
      e_Ab.Set(EigenMatrixX<Element::T>(m_A.cast<Element::T>()),
               EigenVectorX<Element::T>(m_b.cast<Element::T>()));
#ifdef CFG_CAMERA_PRIOR_REORDER
      e_Ab.Marginalize(ipc + 3, 1);
      e_Ab.Marginalize(ipc + 3, 1);
      e_Ab.Marginalize(ipc + 3, 1);
      e_Ab.Marginalize(ipc, 1);
      e_Ab.Marginalize(ipc, 1);
      e_Ab.Marginalize(ipc, 1);
#else
      e_Ab.Marginalize(ipc, 6);
#endif
      EigenMatrixXf(e_Ab.cast<float>()).Get(m_A, m_b);
    }
    inline bool AssertEqual(const Pose &Z, const int verbose = 1,
                            const std::string str = "") const {
      return AssertEqual(Z.m_Arr, Z.m_Arc, Z.m_Acc, Z.m_br, Z.m_bc, verbose, str);
    }
    inline bool AssertEqual(const Element::RR &Arr, const Vector::RC &Arc, const Matrix::CC &Acc,
                            const Element::R &br, const Vector::C &bc, const int verbose = 1,
                            const std::string str = "") const {
      const int Nx6 = m_A.GetRows() - 2, N = bc.Size();
#ifdef CFG_DEBUG
      UT_ASSERT(m_A.GetColumns() == Nx6 + 2 && m_b.Size() == Nx6 + 2);
      UT_ASSERT(Acc.GetRows() == N && Acc.GetColumns() == N);
      if (!Arc.Empty()) {
        UT_ASSERT(Arc.Size() == N);
      }
      UT_ASSERT(N == 0 || Nx6 == N * 6);
#endif
      bool scc = true;
      const EigenMatrix2x2f e_Arr(m_A.block<2, 2>(0, 0));
      if (Arr.Valid()) {
        scc = e_Arr.AssertEqual(Arr, verbose, str + ".Arr") && scc;
      } else {
        scc = e_Arr.AssertZero(verbose, str + ".Arr") && scc;
      }
      const EigenMatrixXf e_Arc(m_A.block(0, 2, 2, Nx6));
      scc = EigenAssertEqual(e_Arc, Arc, verbose, str + ".Arc") && scc;
      const EigenMatrixXf e_Acc(m_A.block(2, 2, Nx6, Nx6));
      scc = EigenAssertEqual(e_Acc, Acc, verbose, str + ".Acc") && scc;
      const EigenVector2f e_br(m_b.block<2, 1>(0, 0));
      if (br.Valid()) {
        scc = e_br.AssertEqual(br, verbose, str + ".br") && scc;
      } else {
        scc = e_br.AssertZero(verbose, str + ".br") && scc;
      }
      const EigenVectorXf e_bc(m_b.block(2, 0, Nx6, 1));
      scc = EigenAssertEqual(e_bc, bc, verbose, str + ".bc") && scc;
      return scc;
    }
   public:
    EigenMatrixXf m_A;
    EigenVectorXf m_b;
  };
 public:
  EigenErrorJacobian EigenGetErrorJacobian(const AlignedVector<Rigid3D> &Cs,
                                           const float eps) const;
  EigenFactor EigenGetFactor(const float w, const AlignedVector<Rigid3D> &Cs,
                             const float eps) const;
  float EigenGetCost(const float w, const AlignedVector<Rigid3D> &Cs,
                     const std::vector<EigenVector6f> &e_xs, const float eps) const;
  void EigenGetResidual(const AlignedVector<Rigid3D> &Cs, EigenVectorXf *e_r,
                        const float eps) const;
  void EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S,
                                EigenVectorXf *e_x = NULL) const;
#endif
};

class Motion {

 public:

  class Error : public Element::EM {
   public:
    inline void Set(const Element::M &e) { e.Get(m_ev, m_eba, m_ebw); }
    inline void Get(Element::M *e) const { e->Set(m_ev, m_eba, m_ebw); }
    inline void MakeZero() { memset(this, 0, sizeof(Error)); }
  };
  class Jacobian {
   public:
    inline void GetTranspose(Jacobian *J) const {
      m_Jvr.GetTranspose(J->m_Jvr);
      m_Jvv.GetTranspose(J->m_Jvv);
    }
   public:
    LA::AlignedMatrix3x3f m_Jvr, m_Jvv;
  };
  class ErrorJacobian {
   public:
    Error m_e;
    Jacobian m_J;
  };
  class Factor {
   public:
    class RR {
     public:
      inline void MakeMinus() {
        const xp128f zero = xp128f::get(0.0f);
        m_data[0] = zero - m_data[0];
        m_data[1] = zero - m_data[1];
        m_b.v2() = -m_b.v2();
      }
      static inline void AmB(const RR &A, const RR &B, RR &AmB) {
        AmB.m_data[0] = A.m_data[0] - B.m_data[0];
        AmB.m_data[1] = A.m_data[1] - B.m_data[1];
        AmB.m_b.v2() = A.m_b.v2() - B.m_b.v2();
      }
     public:
      union {
        struct { LA::SymmetricMatrix3x3f m_A; LA::Vector3f m_b; };
        xp128f m_data[3];
      };
    };
    class RM : public LA::AlignedMatrix3x9f {};
    class MM : public Camera::Factor::Unitary::MM {};
    class Auxiliary {
     public:
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
      inline void Set(const ErrorJacobian &Je, const xp128f &w, const Element::MM &A) {
        Je.m_J.GetTranspose(&m_JT);
        Je.m_e.Get(&m_e);
        
        A.GetScaled(w, m_wA);
        LA::AlignedMatrix3x3f _A;
        m_wA.GetBlock(0, 0, _A);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArv);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvv);
        m_wA.GetBlock(0, 3, _A);
        _A.Transpose();
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArba);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvba);
        m_wA.GetBlock(0, 6, _A);
        _A.Transpose();
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArbw);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvbw);
      }
      inline void Get(const Error &e, LA::SymmetricMatrix3x3f *Arr, LA::AlignedMatrix3x9f *Arm,
                      LA::SymmetricMatrix9x9f *Amm, LA::Vector3f *br, LA::Vector9f *bm) const {
        LA::SymmetricMatrix3x3f::ABT(m_JTArv, m_JT.m_Jvr, *Arr);
        LA::AlignedMatrix3x9f &_Arm = *Arm;
        LA::AlignedMatrix3x3f::ABT(m_JTArv, m_JT.m_Jvv, _Arm[0], _Arm[1], _Arm[2]);
        float *_br = &br->v0();
        LA::AlignedMatrix3x3f::Ab(m_JTArv, e.m_ev, _br);
        Arm->SetBlock(0, 3, m_JTArba);
        LA::AlignedMatrix3x3f::AddAbTo(m_JTArba, e.m_eba, _br);
        Arm->SetBlock(0, 6, m_JTArbw);
        LA::AlignedMatrix3x3f::AddAbTo(m_JTArbw, e.m_ebw, _br);
        LA::SymmetricMatrix9x9f::ABTTo00(m_JTAvv, m_JT.m_Jvv, *Amm);
        float *_bm = &bm->v0();
        LA::AlignedMatrix3x3f::Ab(m_JTAvv, e.m_ev, _bm);
        Amm->Set03(m_JTAvba);
        LA::AlignedMatrix3x3f::AddAbTo(m_JTAvba, e.m_eba, _bm);
        Amm->Set06(m_JTAvbw);
        LA::AlignedMatrix3x3f::AddAbTo(m_JTAvbw, e.m_ebw, _bm);
        Amm->Set33(m_wA[3] + 3, m_wA[4] + 3, m_wA[5] + 3, m_wA[6] + 3, m_wA[7] + 3, m_wA[8] + 3);
        LA::AlignedMatrix9x9f::Ab(m_wA, m_e, _bm, 3);
      }
#else
      inline void Set(const ErrorJacobian &Je, const xp128f &w, const Element::MM &A) {
        Je.m_J.GetTranspose(&m_JT);
        
        A.GetScaled(w, m_wA);
        LA::AlignedMatrix3x3f _A;
        m_wA.GetBlock(0, 0, _A);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArv);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvv);
        float *Aev = &m_Ae.v0(), *Aeba = &m_Ae.v3(), *Aebw = &m_Ae.v6();
        LA::AlignedMatrix3x3f::Ab(_A, Je.m_e.m_ev, Aev);
        m_wA.GetBlock(0, 3, _A);
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_eba, Aev);
        _A.Transpose();
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArba);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvba);
        LA::AlignedMatrix3x3f::Ab(_A, Je.m_e.m_ev, Aeba);
        m_wA.GetBlock(0, 6, _A);
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_ebw, Aev);
        _A.Transpose();
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvr, _A, m_JTArbw);
        LA::AlignedMatrix3x3f::ABT(m_JT.m_Jvv, _A, m_JTAvbw);
        LA::AlignedMatrix3x3f::Ab(_A, Je.m_e.m_ev, Aebw);
        m_wA.GetBlock(3, 3, _A);
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_eba, Aeba);
        m_wA.GetBlock(3, 6, _A);
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_ebw, Aeba);
        _A.Transpose();
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_eba, Aebw);
        m_wA.GetBlock(6, 6, _A);
        LA::AlignedMatrix3x3f::AddAbTo(_A, Je.m_e.m_ebw, Aebw);
      }
      inline void Get(const xp128f &w, const Element::M &b, LA::SymmetricMatrix3x3f *Arr,
                      LA::AlignedMatrix3x9f *Arm, LA::SymmetricMatrix9x9f *Amm, LA::Vector3f *br,
                      LA::Vector9f *bm) const {
        LA::SymmetricMatrix3x3f::ABT(m_JTArv, m_JT.m_Jvr, *Arr);
        LA::AlignedMatrix3x9f &_Arm = *Arm;
        LA::AlignedMatrix3x3f::ABT(m_JTArv, m_JT.m_Jvv, _Arm[0], _Arm[1], _Arm[2]);
        Arm->SetBlock(0, 3, m_JTArba);
        Arm->SetBlock(0, 6, m_JTArbw);
        LA::SymmetricMatrix9x9f::ABTTo00(m_JTAvv, m_JT.m_Jvv, *Amm);
        Amm->Set03(m_JTAvba);
        Amm->Set06(m_JTAvbw);
        Amm->Set33(m_wA[3] + 3, m_wA[4] + 3, m_wA[5] + 3, m_wA[6] + 3, m_wA[7] + 3, m_wA[8] + 3);

        Element::M Aepb;
        b.GetScaled(w, Aepb);
        Aepb += m_Ae;
        const LA::AlignedVector3f Aepbv(&Aepb.v0());
        LA::AlignedMatrix3x3f::Ab(m_JT.m_Jvr, Aepbv, &br->v0());
        LA::AlignedMatrix3x3f::Ab(m_JT.m_Jvv, Aepbv, &bm->v0());
        memcpy(&bm->v3(), &Aepb.v3(), 24);
      }
#endif
     public:
      Jacobian m_JT;
      Element::MM m_wA;
      LA::AlignedMatrix3x3f m_JTArv, m_JTArba, m_JTArbw;
      LA::AlignedMatrix3x3f m_JTAvv, m_JTAvba, m_JTAvbw;
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
      Element::M m_e;
#else
      Element::M m_Ae;
#endif
    };
   public:
    inline void MakeZero() { memset(this, 0, sizeof(Factor)); }
    inline void MakeMinus() {
      m_Arr.MakeMinus();
      m_Arm.MakeMinus();
      m_Amm.MakeMinus();
    }
    inline void Print() const {
      m_Arr.m_A.Print("Arr = ", true);
      m_Arm.Print("Arm = ", true);
      m_Amm.m_A.Print("Amm = ", true);
      m_Arr.m_b.Print("br = ", true);
      m_Amm.m_b.Print("bm = ", true);
    }
    inline void SaveB(FILE *fp) const { UT::SaveB(*this, fp); }
    inline void LoadB(FILE *fp) { UT::LoadB(*this, fp); }
   public:
    ErrorJacobian m_Je;
    union {
      RR m_Arr;
      struct { float m_data[11], m_F; };
    };
    RM m_Arm;
    MM m_Amm;
  };
  class Reduction {
   public:
    Error m_e;
    float m_F, m_dF;
  };
  typedef Pose::ESError ESError;
  class ES : public UT::ES<float, int> {
   public:
    inline void Initialize() {
      UT::ES<float, int>::Initialize();
      m_ESv.Initialize();
      m_ESba.Initialize();
      m_ESbw.Initialize();
    }
    inline void Accumulate(const Error &e, const float F, const int iFrm = -1) {
      UT::ES<float, int>::Accumulate(F, F, iFrm);
      m_ESv.Accumulate(ESError(e.m_ev), -1.0f, iFrm);
      m_ESba.Accumulate(ESError(e.m_eba), -1.0f, iFrm);
      m_ESbw.Accumulate(ESError(e.m_ebw, UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
    }
    inline void Print(const std::string str = "", const bool l = true) const {
      if (!Valid()) {
        return;
      }
      UT::ES<float, int>::Print(str + "em = ", true, l);
      const std::string _str(str.size() + 17, ' ');
      if (m_ESv.Valid()) {
        m_ESv.Print(_str + "ev  = ", false, l);
      }
      if (m_ESba.Valid()) {
        m_ESba.Print(_str + "eba = ", false, l);
      }
      if (m_ESbw.Valid()) {
        m_ESbw.Print(_str + "ebw = ", false, l);
      }
    }
   public:
    UT::ES<ESError, int> m_ESv, m_ESba, m_ESbw;
  };

  inline void Initialize(const float w, const float s2v, const float s2ba, const float s2bw,
                         const Camera *C = NULL) {
    if (C) {
      SetMotion(C->m_T, C->m_v, C->m_ba, C->m_bw);
    } else {
      m_v.MakeZero();
      m_ba.MakeZero();
      m_bw.MakeZero();
    }
    m_Amm.MakeZero();
    m_Amm[0][0] = m_Amm[1][1] = m_Amm[2][2] = UT::Inverse(s2v, w);
    m_Amm[3][3] = m_Amm[4][4] = m_Amm[5][5] = UT::Inverse(s2ba, w);
    m_Amm[6][6] = m_Amm[7][7] = m_Amm[8][8] = UT::Inverse(s2bw, w);
    m_bm.MakeZero();
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    m_em.MakeZero();
#else
    m_xTb = 0.0f;
#endif
  }
  inline void Initialize(const Motion &Zp) { *((Motion *) this) = Zp; }

#ifdef CFG_DEBUG
  inline void DebugSetMeasurement(const Camera &C) {
    SetMotion(C.m_T, C.m_v, C.m_ba, C.m_bw);
    m_bm.MakeZero();
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    m_em.MakeZero();
#else
    m_xTb = 0.0f;
#endif
  }
#endif

  inline bool Valid() const { return m_v.Valid(); }
  inline bool Invalid() const { return m_v.Invalid(); }
  inline void Invalidate() { m_v.Invalidate(); }

  //inline void MakeZero() { memset(this, 0, sizeof(Motion)); }

  inline void SetMotion(const Rotation3D &R, const LA::AlignedVector3f &v,
                        const LA::AlignedVector3f &ba, const LA::AlignedVector3f &bw) {
    R.Apply(v, m_v);
    m_ba = ba;
    m_bw = bw;
  }
  inline void GetMotion(const Rotation3D &R, LA::AlignedVector3f *v, LA::AlignedVector3f *ba,
                        LA::AlignedVector3f *bw) const {
    R.ApplyInversely(m_v, *v);
    *ba = m_ba;
    *bw = m_bw;
  }
  inline LA::AlignedVector3f GetVelocityState(const Camera &C) const {
    return C.m_T.GetAppliedRotation(C.m_v);
  }
  inline LA::AlignedVector3f GetVelocityMeasurement(const float *x = NULL) const {
    if (x) {
      return m_v + LA::AlignedVector3f(x);
    } else {
      return m_v;
    }
  }
  inline LA::AlignedVector3f GetVelocityError(const Camera &C, const float *x) const {
    return GetVelocityState(C) - GetVelocityMeasurement(x);
  }
  inline LA::AlignedVector3f GetBiasAccelerationState(const Camera &C) const {
    return C.m_ba;
  }
  inline LA::AlignedVector3f GetBiasAccelerationMeasurement(const float *x = NULL) const {
    if (x) {
      return m_ba + LA::AlignedVector3f(x);
    } else {
      return m_ba;
    }
  }
  inline LA::AlignedVector3f GetBiasAccelerationError(const Camera &C,
                                                      const float *x = NULL) const {
    return GetBiasAccelerationState(C) - GetBiasAccelerationMeasurement(x);
  }
  inline LA::AlignedVector3f GetBiasGyroscopeState(const Camera &C) const {
    return C.m_bw;
  }
  inline LA::AlignedVector3f GetBiasGyroscopeMeasurement(const float *x = NULL) const {
    if (x) {
      return m_bw + LA::AlignedVector3f(x);
    } else {
      return m_bw;
    }
  }
  inline LA::AlignedVector3f GetBiasGyroscopeError(const Camera &C, const float *x = NULL) const {
    return GetBiasGyroscopeState(C) - GetBiasGyroscopeMeasurement(x);
  }
  inline void GetError(const Camera &C, Element::EM *e) const {
    const LA::AlignedVector3f v = C.m_T.GetAppliedRotation(C.m_v);
    LA::AlignedVector3f::amb(v, m_v, e->m_ev);
    LA::AlignedVector3f::amb(C.m_ba, m_ba, e->m_eba);
    LA::AlignedVector3f::amb(C.m_bw, m_bw, e->m_ebw);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    *e += m_em;
#endif
  }
  inline void GetError(const ErrorJacobian &Je, const LA::AlignedVector3f *xr,
                       const LA::AlignedVector3f *xv, const LA::AlignedVector3f *xba,
                       const LA::AlignedVector3f *xbw, Error *e) const {
#ifdef CFG_DEBUG
    UT_ASSERT(xr || xv || xba || xbw);
#endif
    *e = Je.m_e;
    if (xr) {
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvr, *xr, (float *) &e->m_ev);
    }
    if (xv) {
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvv, *xv, (float *) &e->m_ev);
    }
    if (xba) {
      e->m_eba += *xba;
    }
    if (xbw) {
      e->m_ebw += *xbw;
    }
  }
  inline void GetErrorJacobian(const Camera &C, ErrorJacobian *Je) const {
    GetError(C, &Je->m_e);
    SkewSymmetricMatrix::AB(C.m_T, C.m_v, Je->m_J.m_Jvr);
    Je->m_J.m_Jvv = C.m_T;
  }
  inline void GetFactor(const float w, const Camera &C, Factor *A, Factor::Auxiliary *U) const {
    GetErrorJacobian(C, &A->m_Je);
    const xp128f _w = xp128f::get(w);
    U->Set(A->m_Je, _w, m_Amm);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    U->Get(A->m_Je.m_e, &A->m_Arr.m_A, &A->m_Arm, &A->m_Amm.m_A, &A->m_Arr.m_b, &A->m_Amm.m_b);
    A->m_F = GetCost(w, A->m_Je.m_e);
#else
    A->m_F = GetCost(w, A->m_Je.m_e, U->m_Ae);
    U->Get(_w, m_bm, &A->m_Arr.m_A, &A->m_Arm, &A->m_Amm.m_A, &A->m_Arr.m_b, &A->m_Amm.m_b);
#endif
  }
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  inline float GetCost(const float w, const Error &e) const {
    Element::M _e, v;
    _e.Set(e.m_ev, e.m_eba, e.m_ebw);
    LA::AlignedMatrix9x9f::Ab(m_Amm, _e, (float *) &v);
    return w * _e.Dot(v);
  }
#else
  inline float GetCost(const float w, const Error &e, const Element::M &Ae) const {
    Element::M _e, b;
    e.Get(&_e);
    m_bm.GetScaled(w + w, b);
    b += Ae;
    //return 0.5f * _e.Dot(b);
    return _e.Dot(b);
  }
  inline float GetCost(const float w, const Error &e) const {
    Element::M _e, v;
    e.Get(&_e);
    LA::AlignedMatrix9x9f::Ab(m_Amm, _e, (float *) &v);
    //v *= 0.5f;
    v += m_bm;
    v += m_bm;
    return w * _e.Dot(v);
  }
#endif
  inline float GetCost(const float w, const ErrorJacobian &Je, const LA::AlignedVector3f *xr,
                       const LA::AlignedVector3f *xv, const LA::AlignedVector3f *xba,
                       const LA::AlignedVector3f *xbw, Error *e) const {
    GetError(Je, xr, xv, xba, xbw, e);
    return GetCost(w, *e);
  }
  inline void GetReduction(const float w, const Factor &A, const Camera &C,
                           const LA::AlignedVector3f *xr, const LA::AlignedVector3f *xv,
                           const LA::AlignedVector3f *xba, const LA::AlignedVector3f *xbw,
                           Reduction *Ra, Reduction *Rp) const {
    GetError(C, &Ra->m_e);
    GetError(A.m_Je, xr, xv, xba, xbw, &Rp->m_e);
    Ra->m_dF = A.m_F - (Ra->m_F = GetCost(w, Ra->m_e));
    Rp->m_dF = A.m_F - (Rp->m_F = GetCost(w, Rp->m_e));
  }
  inline void GetResidual(const Camera &C, Error *e, Element::M *em, Element::M *r) const {
    GetError(C, e);
    em->Set(e->m_ev, e->m_eba, e->m_ebw);
    LA::AlignedMatrix9x9f::Ab(m_Amm, *em, (float *) r);
    if (m_bm.Valid()) {
      *r += m_bm;
    }
  }

  inline void Print(const bool e = false) const {
    m_Amm.Print("Amm = ", e);
    m_bm.Print("bm = ", e);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    //m_em.Print("em = ", e);
#endif
  }
  inline void PrintDiagonal(const bool e = false) const {
    m_Amm.PrintDiagonal("Amm = ", e);
  }
  inline void SaveB(FILE *fp) const {
    UT::SaveB(m_v, fp);
    UT::SaveB(m_ba, fp);
    UT::SaveB(m_bw, fp);
    UT::SaveB(m_Amm, fp);
    UT::SaveB(m_bm, fp);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    UT::SaveB(m_em, fp);
#else
    UT::SaveB(m_xTb, fp);
#endif
  }
  inline void LoadB(FILE *fp) {
    UT::LoadB(m_v, fp);
    UT::LoadB(m_ba, fp);
    UT::LoadB(m_bw, fp);
    UT::LoadB(m_Amm, fp);
    UT::LoadB(m_bm, fp);
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
    UT::LoadB(m_em, fp);
#else
    UT::LoadB(m_xTb, fp);
#endif
  }

  bool GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                           float *xTb, AlignedVector<float> *work, const float *eps = NULL) const;

 public:

  LA::AlignedVector3f m_v, m_ba, m_bw;
  Element::MM m_Amm;
  Element::M m_bm;
#ifdef CFG_CAMERA_PRIOR_SQUARE_FORM
  Error m_em;
#else
  float m_xTb;
#endif

#ifdef CFG_DEBUG_EIGEN
 public:
  class EigenErrorJacobian {
   public:
    inline void Set(const Error &e) {
      m_e.block<3, 1>(0, 0) = EigenVector3f(e.m_ev);
      m_e.block<3, 1>(3, 0) = EigenVector3f(e.m_eba);
      m_e.block<3, 1>(6, 0) = EigenVector3f(e.m_ebw);
    }
    inline void Set(const Jacobian &J) {
      m_J.setZero();
      m_J.block<3, 3>(0, 0) = EigenMatrix3x3f(J.m_Jvr);
      m_J.block<3, 3>(0, 3) = EigenMatrix3x3f(J.m_Jvv);
      m_J.block<3, 3>(3, 6) = EigenMatrix3x3f::Identity();
      m_J.block<3, 3>(6, 9) = EigenMatrix3x3f::Identity();
    }
    inline void Set(const ErrorJacobian &Je) { Set(Je.m_e); Set(Je.m_J); }
    inline bool AssertEqual(const Error &e, const int verbose = 1,
                            const std::string str = "") const {
      const EigenVector3f e_ev(m_e.block<3, 1>(0, 0));
      const EigenVector3f e_eba(m_e.block<3, 1>(3, 0));
      const EigenVector3f e_ebw(m_e.block<3, 1>(6, 0));
      bool scc = true;
      scc = e_ev.AssertEqual(e.m_ev, verbose, str + ".m_ev") && scc;
      scc = e_eba.AssertEqual(e.m_eba, verbose, str + ".m_eba") && scc;
      scc = e_ebw.AssertEqual(e.m_ebw, verbose, str + ".m_ebw") && scc;
      return scc;
    }
    inline bool AssertEqual(const Jacobian &J, const int verbose = 1,
                            const std::string str = "") const {
      const EigenMatrix3x3f e_Jvr(m_J.block<3, 3>(0, 0));
      const EigenMatrix3x3f e_Jvv(m_J.block<3, 3>(0, 3));
      bool scc = true;
      scc = e_Jvr.AssertEqual(J.m_Jvr, verbose, str + ".m_Jvr") && scc;
      scc = e_Jvv.AssertEqual(J.m_Jvv, verbose, str + ".m_Jvv") && scc;
      return scc;
    }
    inline bool AssertEqual(const ErrorJacobian &Je, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = AssertEqual(Je.m_e, verbose, str) && scc;
      scc = AssertEqual(Je.m_J, verbose, str) && scc;
      return scc;
    }
   public:
    EigenMatrix9x12f m_J;
    EigenVector9f m_e;
  };
  class EigenFactor {
   public:
    inline void Set(const Factor &A) {
      m_A.block<3, 3>(0, 0) = EigenMatrix3x3f(A.m_Arr.m_A);
      const EigenMatrix3x9f e_Arm(A.m_Arm);
      m_A.block<3, 9>(0, 3) = e_Arm;
      m_A.block<9, 3>(3, 0) = e_Arm.transpose();
      m_A.block<9, 9>(3, 3) = EigenMatrix9x9f(A.m_Amm.m_A);
      m_b.block<3, 1>(0, 0) = EigenVector3f(A.m_Arr.m_b);
      m_b.block<9, 1>(3, 0) = EigenVector9f(A.m_Amm.m_b);
      m_F = A.m_F;
    }
    inline bool AssertEqual(const Factor &A, const int verbose = 1,
                            const std::string str = "") const {
      const EigenMatrix3x3f e_Arr(m_A.block<3, 3>(0, 0));
      const EigenMatrix3x9f e_Arm(m_A.block<3, 9>(0, 3));
      const EigenMatrix9x9f e_Amm(m_A.block<9, 9>(3, 3));
      const EigenVector3f e_br(m_b.block<3, 1>(0, 0));
      const EigenVector9f e_bm(m_b.block<9, 1>(3, 0));
      bool scc = true;
      scc = e_Arr.AssertEqual(A.m_Arr.m_A, verbose, str + ".m_Arr") && scc;
      scc = e_Arm.AssertEqual(A.m_Arm, verbose, str + ".m_Arm") && scc;
      scc = e_Amm.AssertEqual(A.m_Amm.m_A, verbose, str + ".m_Amm") && scc;
      scc = e_br.AssertEqual(A.m_Arr.m_b, verbose, str + ".m_br") && scc;
      scc = e_bm.AssertEqual(A.m_Amm.m_b, verbose, str + ".m_bm") && scc;
      scc = UT::AssertEqual(m_F, A.m_F, 1, str + ".m_F") && scc;
      return scc;
    }
   public:
    EigenMatrix12x12f m_A;
    EigenVector12f m_b;
    float m_F;
  };
  class EigenPrior : public Pose::EigenPrior {
   public:
    inline void Initialize(const float w, const float s2v, const float s2ba, const float s2bw) {
      m_A.Resize(9, 9);
      m_A.MakeZero();
      m_b.Resize(9);
      m_b.MakeZero();
      m_A(0, 0) = m_A(1, 1) = m_A(2, 2) = UT::Inverse(s2v, w);
      m_A(3, 3) = m_A(4, 4) = m_A(5, 5) = UT::Inverse(s2ba, w);
      m_A(6, 6) = m_A(7, 7) = m_A(8, 8) = UT::Inverse(s2bw, w);
    }
    inline void Set(const Motion &Z) {
      m_A = EigenMatrix9x9f(Z.m_Amm);
      m_b = EigenVector9f(Z.m_bm);
    }
    inline bool AssertEqual(const Motion &Z, const int verbose = 1,
                            const std::string str = "") const {
      UT_ASSERT(m_A.GetRows() == 9 && m_A.GetColumns() == 9 && m_b.Size() == 9);
      bool scc = true;
      const EigenMatrix9x9f e_Amm(m_A.block<9, 9>(0, 0));
      scc = e_Amm.AssertEqual(Z.m_Amm, verbose, str + ".Amm") && scc;
      const EigenVector9f e_bm(m_b.block<9, 1>(0, 0));
      scc = e_bm.AssertEqual(Z.m_bm, verbose, str + ".bm") && scc;
      //const EigenVector9f e_em = EigenVector9f(e_Amm.inverse() * e_bm);
      //Element::M em;
      //Z.m_em.Get(&em);
      //scc = e_em.AssertEqual(em, verbose, str + "em") && scc;
      return scc;
    }
  };
 public:
  EigenErrorJacobian EigenGetErrorJacobian(const Camera &C) const;
  EigenFactor EigenGetFactor(const float w, const Camera &C) const;
  float EigenGetCost(const float w, const Camera &C, const EigenVector3f &e_xr,
                     const EigenVector9f &e_xm) const;
  void EigenGetResidual(const Camera &C, EigenVector9f *e_r) const;
  void EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S,
                                EigenVectorXf *e_x = NULL) const;
#endif
};

class Joint : public Pose, public Motion {

 public:

  class Error {
   public:
    Pose::Error m_ec;
    Motion::Error m_em;
  };

 public:

  inline Joint() {}
  inline Joint(const Joint &Zp) { *this = Zp; }
  inline void operator = (const Joint &Zp) {
    *((Pose *) this) = Zp;
    *((Motion *) this) = Zp;
    m_Arm = Zp.m_Arm;
    m_Acm.Set(Zp.m_Acm);
  }

  //inline bool Valid() const { return Pose::Valid() && Motion::Valid(); }
  //inline bool Invalid() const { return Pose::Invalid() || Motion::Invalid(); }
  //inline void Invalidate() { Pose::Invalidate(); Motion::Invalidate(); }

  inline void Initialize(const Motion &Zp) {
    Pose::Invalidate();
    Motion::Initialize(Zp);
    m_Arm.MakeZero();
    m_Acm.Resize(0);
  }
  inline void Initialize(const float w, const int iKFr, const Rigid3D &Tr, const float s2r,
                         const Motion &Zp, const bool newKF, const Rigid3D *T0 = NULL,
                         const float s2cp = 0.0f, const float s2cr = 0.0f) {
    Pose::Initialize(w, iKFr, Tr, s2r, newKF, T0, s2cp, s2cr);
    Motion::Initialize(Zp);
    m_Arm.MakeZero();
    if (newKF) {
      m_Acm.Resize(0);
    } else {
      m_Acm.Resize(1);
      m_Acm[0].MakeZero();
    }
  }
  inline void Insert(const float w, const int i, const int iKF, const Rigid3D &T, const float s2p,
                     const float s2r, AlignedVector<float> *work) {
    Pose::Insert(w, i, iKF, T, s2p, s2r, work);
    m_Acm.InsertZero(i);
  }
  inline void Erase(const int i) {
    Pose::Erase(i);
    m_Acm.Erase(i);
  }
  inline void DeleteKeyFrame(const int iKF, const std::vector<int>::iterator *i = NULL) {
    const std::vector<int>::iterator _i = i ? *i : std::lower_bound(m_iKFs.begin(),
                                                                    m_iKFs.end(), iKF);
    if (iKF != m_iKFr && _i != m_iKFs.end() && *_i == iKF) {
      const int j = static_cast<int>(_i - m_iKFs.begin());
      m_Acm.Erase(j);
    }
    Pose::DeleteKeyFrame(iKF, &_i);
  }
  inline void Update(const int i, const Camera::Factor::Unitary::CC &A) {
    m_Acc[i][i] += A.m_A;
    m_Acc[i][i].SetLowerFromUpper();
    m_bc[i] += A.m_b;
  }
  inline void Update(const int i1, const int i2, const Camera::Factor::Unitary::CC &A11,
                     const Camera::Factor::Binary::CC &A12, const Camera::Factor::Unitary::CC &A22) {
    m_Acc[i1][i1] += A11.m_A;
    m_Acc[i1][i1].SetLowerFromUpper();
    m_Acc[i1][i2] += A12;
    m_Acc[i2][i2] += A22.m_A;
    m_Acc[i2][i2].SetLowerFromUpper();
    m_bc[i1] += A11.m_b;
    m_bc[i2] += A22.m_b;
  }
  inline void GetError(const Rigid3D &Tr, const Camera &C, LA::Vector2f *er/* = NULL*/,
                       Element::EC *ec/* = NULL*/, Element::EM *em/* = NULL*/,
                       const float eps) const {
    const int N = static_cast<int>(m_iKFs.size());
    if (er) {
      if (m_Zps.Size() == N) {
        er->Invalidate();
      } else {
#ifdef CFG_DEBUG
        UT_ASSERT(m_Zps.Size() == N + 1);
#endif
        Rotation3D eR;
        Rotation3D::AB(m_Zps.Back(), Tr, eR);
        eR.GetRodriguesXY(*er, eps);
      }
    }
    if (ec) {
      const int ik = N - 1;
#ifdef CFG_DEBUG
      UT_ASSERT(m_iKFs[ik] == INT_MAX);
#endif
      Pose::GetError(Tr, C.m_T, ik, ec, eps);
    }
    if (em) {
      Motion::GetError(C, em);
    }
  }
  inline void GetError(const AlignedVector<Rigid3D> &Cs, const Camera &C, Error *e,
                       const float eps) const {
    Pose::GetError(Cs, &e->m_ec, eps);
    Motion::GetError(C, &e->m_em);
  }
  inline float GetCost(const float w, const Error &e) const {
    const float Fcc = Pose::GetCost(w, e.m_ec);
    float Fmm = Motion::GetCost(w, e.m_em);
    Element::R er;
    Element::C ec1, ec2;
    Element::M em;
    e.m_em.Get(&em);
    Element::RM::Ab(m_Arm, em, er);
    float Fcm = er.Dot(e.m_ec.m_er);
    const int N = e.m_ec.Size();
    for (int i = 0; i < N; ++i) {
      Element::CM::Ab(m_Acm[i], em, (float *) &ec1);
      e.m_ec.m_ec[i].Get(&ec2);
      Fcm += ec1.Dot(ec2);
    }
    Fcm *= w + w;
    return Fcc + Fmm + Fcm;
  }
  inline void GetResidual(const AlignedVector<Rigid3D> &Cs, const Camera &C, Error *e,
                          Vector::C *ec, Element::M *em, Element::R *rr, Vector::C *rc,
                          Element::M *rm, const float eps) const {
    Pose::GetResidual(Cs, &e->m_ec, ec, rr, rc, eps);
    Motion::GetResidual(C, &e->m_em, em, rm);
    const int N = static_cast<int>(m_iKFs.size());
    if (m_Zps.Size() != N) {
      LA::AlignedMatrix2x9f::AddAbTo(m_Arm, *em, *rr);
      LA::AlignedMatrix2x9f::AddATbTo(m_Arm, e->m_ec.m_er, *rm);
    }
    LA::AlignedMatrix9x6f Amc;
    const Vector::C &_ec = *ec;
    Vector::C &_rc = *rc;
    for (int i = 0; i < N; ++i) {
      LA::AlignedMatrix6x9f::AddAbTo(m_Acm[i], *em, (float *) &_rc[i]);
      m_Acm[i].GetTranspose(Amc);
      LA::AlignedMatrix9x6f::AddAbTo(Amc, _ec[i], (float *) rm);
    }
  }

  inline void Print(const bool e = false) const {
    Pose::Print(e);
    Motion::Print(e);
    m_Arm.Print("Arm = ", e);
    const int Nk = static_cast<int>(m_iKFs.size());
    for (int i = 0; i < Nk; ++i) {
      const int iKF = m_iKFs[i] == INT_MAX ? -1 : m_iKFs[i];
      m_Arc[i].Print(UT::String("Arc(%d) = ", iKF), e);
    }
  }
  inline void PrintDiagonal(const bool e = false) const {
    Pose::PrintDiagonal(e);
    Motion::PrintDiagonal(e);
  }
  
  inline void SaveB(FILE *fp) const {
    Pose::SaveB(fp);
    Motion::SaveB(fp);
    UT::SaveB(m_Arm, fp);
    m_Acm.SaveB(fp);
  }
  inline void LoadB(FILE *fp) {
    Pose::LoadB(fp);
    Motion::LoadB(fp);
    UT::LoadB(m_Arm, fp);
    m_Acm.LoadB(fp);
  }
  inline void AssertConsistency() {
    Pose::AssertConsistency();
    if (Pose::Valid()) {
      UT_ASSERT(m_iKFs.back() == INT_MAX);
    }
    UT_ASSERT(m_Acm.Size() == static_cast<int>(m_iKFs.size()));
  }

  void SetPriorEquation(const Matrix::X &A, const Vector::X &b);
  void GetPriorEquation(Matrix::X *A, Vector::X *b = NULL, const bool symmetric = true) const;
  bool GetPriorMeasurement(const Element::T w, Matrix::X *S, Vector::X *x/* = NULL*/,
                           Element::T *xTb/* = NULL*/, const Element::T *eps/* = NULL*/) const;
  bool GetPriorMeasurement(const float w, LA::AlignedMatrixXf *S, LA::AlignedVectorXf *x,
                           float *xTb, AlignedVector<float> *work, const float *eps = NULL) const;
  bool Invertible(AlignedVector<float> *work, const float *eps = NULL) const;
  bool PropagateLF(const Rigid3D &Tr, const Camera &C,
                   const IMU::Delta::Factor::Auxiliary::RelativeLF &A,
                   AlignedVector<float> *work, const float *eps = NULL);
  bool PropagateLF(const IMU::Delta::Factor::Auxiliary::RelativeLF &A, LA::AlignedVectorXf *x,
                   AlignedVector<float> *work, const float *eps = NULL) const;
  bool PropagateKF(const Rigid3D &Tr, const Camera &C,
                   const IMU::Delta::Factor::Auxiliary::RelativeKF &A,
                   AlignedVector<float> *work, const float *eps = NULL);
  bool PropagateKF(const IMU::Delta::Factor::Auxiliary::RelativeKF &A, LA::AlignedVectorXf *x,
                   AlignedVector<float> *work, const float *eps = NULL) const;
  bool GetPriorPose(const int iKF, Pose *Zp, AlignedVector<float> *work, const float *eps = NULL) const;
  bool GetPriorMotion(Motion *Zp, AlignedVector<float> *work, const float *eps = NULL) const;

 public:

  Element::RM m_Arm;
  Vector::CM m_Acm;

#ifdef CFG_DEBUG_EIGEN
 public:
  class EigenPrior : public Pose::EigenPrior {
   public:
    inline void Initialize(const float w, const float s2r, const Motion::EigenPrior &e_Ap,
                           const float s2cp = 0.0f, const float s2cr = 0.0f) {
      m_A.Resize(17, 17);
      m_A.MakeZero();
      m_b.Resize(17);
      m_b.MakeZero();
      m_A(0, 0) = m_A(1, 1) = UT::Inverse(s2r, w);
      m_A(2, 2) = m_A(3, 3) = m_A(4, 4) = UT::Inverse(s2cp, w);
      m_A(5, 5) = m_A(6, 6) = m_A(7, 7) = UT::Inverse(s2cr, w);
#ifdef CFG_DEBUG
      UT_ASSERT(e_Ap.m_A.GetRows() == 9 && e_Ap.m_A.GetColumns() == 9 && e_Ap.m_b.Size() == 9);
#endif
      m_A.block(8, 8, 9, 9) = e_Ap.m_A;
      m_b.block(8, 0, 9, 1) = e_Ap.m_b;
    }
    inline void Set(const Joint &Z, const bool pad = false) {
      Set(Z.m_Arr, Z.m_Arc, Z.m_Arm, Z.m_Acc, Z.m_Acm, Z.m_Amm, Z.m_br, Z.m_bc, Z.m_bm, pad);
    }
    inline void Set(const Element::RR &Arr, const Vector::RC &Arc, const Element::RM &Arm,
                    const Matrix::CC &Acc, const Vector::CM &Acm, const Element::MM &Amm,
                    const Element::R &br, const Vector::C &bc, const Element::M &bm,
                    const bool pad = false) {
      const Pose::EigenPrior e_Arc = Pose::EigenPrior(Arr, Arc, Acc, br, bc, 1.0f, pad);
      const int Nx6 = bc.Size() * 6, Nrc = e_Arc.m_b.Size(), Nrcm = Nrc + 9;
      m_A.Resize(Nrcm, Nrcm);
      m_A.MakeZero();
      m_A.block(0, 0, Nrc, Nrc) = e_Arc.m_A;
      m_b.Resize(Nrcm);
      m_b.block(0, 0, Nrc, 1) = e_Arc.m_b;
      if (!Arc.Empty()) {
        const EigenMatrix2x9f e_Arm = EigenMatrix2x9f(Arm);
        m_A.block<2, 9>(0, Nrc) = e_Arm;
        m_A.block<9, 2>(Nrc, 0) = e_Arm.transpose();
      }
      if (!Acm.Empty()) {
        const EigenMatrixXf e_Acm = EigenConvert(Acm);
        m_A.block(2, Nrc, Nx6, 9) = e_Acm;
        m_A.block(Nrc, 2, 9, Nx6) = e_Acm.transpose();
      }
      m_A.block<9, 9>(Nrc, Nrc) = EigenMatrix9x9f(Amm);
      m_b.block<9, 1>(Nrc, 0) = EigenVector9f(bm);
    }
    inline void Get(Joint &Z) const {
      Get(Z.m_Arr, Z.m_Arc, Z.m_Arm, Z.m_Acc, Z.m_Acm, Z.m_Amm, Z.m_br, Z.m_bc, Z.m_bm);
    }
    inline void Get(Element::RR &Arr, Vector::RC &Arc, Element::RM &Arm,
                    Matrix::CC &Acc, Vector::CM &Acm, Element::MM &Amm,
                    Element::R &br, Vector::C &bc, Element::M &bm) const {
      const int Nrcm = m_b.Size(), Nrc = Nrcm - 9, Nx6 = Nrc - 2, N = Nx6 / 6;
#ifdef CFG_DEBUG
      UT_ASSERT(Nx6 % 6 == 0);
      UT_ASSERT(m_A.GetRows() == Nrcm && m_A.GetColumns() == Nrcm);
#endif
      const Pose::EigenPrior e_Arc(EigenMatrixXf(m_A.block(0, 0, Nrc, Nrc)),
                                   EigenVectorXf(m_b.block(0, 0, Nrc, 1)));
      e_Arc.Get(Arr, Arc, Acc, br, bc);
      Arm = EigenMatrix2x9f(m_A.block<2, 9>(0, Nrc)).GetAlignedMatrixMxNf();
      EigenConvert(EigenMatrixXf(m_A.block(2, Nrc, Nx6, 9)), Acm);
      Amm = EigenMatrix9x9f(m_A.block<9, 9>(Nrc, Nrc)).GetAlignedMatrixMxNf();
      bm = EigenVector9f(m_b.block<9, 1>(Nrc, 0)).GetAlignedVector9f();
    }
    inline void Insert(const float w, const int i, const float s2p, const float s2r) {
      const int j = 2 + i * 6;
      m_A.InsertZero(j, 6);
      m_A(j, j) = m_A(j + 1, j + 1) = m_A(j + 2, j + 2) = UT::Inverse(s2p, w);
      m_A(j + 3, j + 3) = m_A(j + 4, j + 4) = m_A(j + 5, j + 5) = UT::Inverse(s2r, w);
      m_b.InsertZero(j, 6);
    }
    inline void Update(const int i, const EigenMatrix6x6f &e_A, const EigenVector6f &e_b) {
      const int j = 2 + i * 6;
      m_A.block<6, 6>(j, j) += e_A;
      m_b.block<6, 1>(j, 0) += e_b;
    }
    inline void Update(const int i1, const int i2, const EigenMatrix6x6f &e_A11,
                       const EigenMatrix6x6f &e_A12, const EigenMatrix6x6f &e_A22,
                       const EigenVector6f &e_b1, const EigenVector6f &e_b2) {
      const int j1 = 2 + i1 * 6, j2 = 2 + i2 * 6;
      m_A.block<6, 6>(j1, j1) += e_A11;
      m_A.block<6, 6>(j1, j2) += e_A12;
      m_A.block<6, 6>(j2, j1) = m_A.block<6, 6>(j1, j2).transpose();
      m_A.block<6, 6>(j2, j2) += e_A22;
      m_b.block<6, 1>(j1, 0) += e_b1;
      m_b.block<6, 1>(j2, 0) += e_b2;
    }
    inline bool AssertEqual(const Joint &Z, const int verbose = 1,
                            const std::string str = "") const {
      return AssertEqual(Z.m_Arr, Z.m_Arc, Z.m_Arm, Z.m_Acc, Z.m_Acm, Z.m_Amm,
                         Z.m_br, Z.m_bc, Z.m_bm, verbose, str);
    }
    inline bool AssertEqual(const Element::RR &Arr, const Vector::RC &Arc, const Element::RM &Arm,
                            const Matrix::CC &Acc, const Vector::CM &Acm, const Element::MM &Amm,
                            const Element::R &br, const Vector::C &bc, const Element::M &bm,
                            const int verbose = 1, const std::string str = "") const {
      const int Nrcm = m_b.Size(), Nrc = Nrcm - 9, Nx6 = Nrc - 2;
      bool scc = true;
      const Pose::EigenPrior e_Arc(EigenMatrixXf(m_A.block(0, 0, Nrc, Nrc)),
                                   EigenVectorXf(m_b.block(0, 0, Nrc, 1)));
      scc = e_Arc.AssertEqual(Arr, Arc, Acc, br, bc, verbose, str) && scc;
      const EigenMatrix2x9f e_Arm(m_A.block<2, 9>(0, Nrc));
      if (Arm.Valid()) {
        scc = e_Arm.AssertEqual(Arm, verbose, str + ".Arm") && scc;
      } else {
        scc = e_Arm.AssertZero() && scc;
      }
      const EigenMatrixXf e_Acm(m_A.block(2, Nrc, Nx6, 9));
      scc = EigenAssertEqual(e_Acm, Acm, verbose, str + ".Acm") && scc;
      const EigenMatrix9x9f e_Amm(m_A.block<9, 9>(Nrc, Nrc));
      scc = e_Amm.AssertEqual(Amm, verbose, str + ".Amm") && scc;
      const EigenVector9f e_bm(m_b.block<9, 1>(Nrc, 0));
      scc = e_bm.AssertEqual(bm, verbose, str + ".bm") && scc;
      return scc;
    }
    
    void PropagateLF(const IMU::Delta::EigenFactor::RelativeLF &e_A, EigenVectorXf *e_x = NULL);
    void PropagateKF(const IMU::Delta::EigenFactor::RelativeKF &e_A, EigenVectorXf *e_x = NULL);
    void GetPriorPose(const int iKF, Pose::EigenPrior *e_Ap) const;
    void GetPriorMotion(Motion::EigenPrior *e_Ap) const;
  };
  void EigenGetResidual(const AlignedVector<Rigid3D> &Cs, const Camera &C,
                        EigenVectorXf *e_r, const float eps) const;
  void EigenGetPriorMeasurement(const float w, EigenMatrixXf *e_S,
                                EigenVectorXf *e_x = NULL) const;
#endif

};



}  // namespace CameraPrior

#endif
