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
#ifndef _IMU_H_
#define _IMU_H_

#include "Camera.h"
#include "Parameter.h"
#include "AlignedVector.h"

namespace IMU {
class Measurement {
 public:
  inline const float& t() const { return m_w.r(); }
  inline       float& t()       { return m_w.r(); }
  inline bool operator < (const float t) const { return this->t() < t; }
  inline bool Valid() const { return m_a.Valid(); }
  inline bool Invalid() const { return m_a.Invalid(); }
  inline void Invalidate() { m_a.Invalidate(); }
  static inline void Interpolate(const Measurement &u1, const Measurement &u2, const float t,
                                 LA::AlignedVector3f &a, LA::AlignedVector3f &w) {
    const xp128f w1 = xp128f::get((u2.t() - t) / (u2.t() - u1.t()));
    const xp128f w2 = xp128f::get(1.0f - w1[0]);
    a.xyzr() = (u1.m_a.xyzr() * w1) + (u2.m_a.xyzr() * w2);
    w.xyzr() = (u1.m_w.xyzr() * w1) + (u2.m_w.xyzr() * w2);
  }
  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e  %e %e %e  %e %e %e\n", t(), m_a.x(), m_a.y(), m_a.z(),
                                                 m_w.x(), m_w.y(), m_w.z());
    } else {
      UT::Print("%f  %f %f %f  %f %f %f\n", t(), m_a.x(), m_a.y(), m_a.z(),
                                                 m_w.x(), m_w.y(), m_w.z());
    }
  }
 public:
  LA::AlignedVector3f m_a, m_w;
};

class Delta {
 public:
  class Transition {
   public:
    class DD {
     public:
      //LA::AlignedMatrix3x3f m_Fvr, m_Fpr, m_Fpv;
      SkewSymmetricMatrix m_Fvr, m_Fpr;
      xp128f m_Fpv;
    };
    class DB {
     public:
      LA::AlignedMatrix3x3f m_Frbw, m_Fvba, m_Fvbw, m_Fpba, m_Fpbw;
    };
   public:
    DD m_Fdd;
    DB m_Fdb;
  };
  class Covariance {
   public:
    class DD {
     public:
      inline void GetUpper(float **P) const {
        memcpy(&P[0][0], &m_Prr.m00(), 12);
        memcpy(&P[0][3], &m_Prv.m00(), 12);
        memcpy(&P[0][6], &m_Prp.m00(), 12);
        memcpy(&P[1][1], &m_Prr.m11(), 8);
        memcpy(&P[1][3], &m_Prv.m10(), 12);
        memcpy(&P[1][6], &m_Prp.m10(), 12);
        P[2][2] = m_Prr.m22();
        memcpy(&P[2][3], &m_Prv.m20(), 12);
        memcpy(&P[2][6], &m_Prp.m20(), 12);
        memcpy(&P[3][3], &m_Pvv.m00(), 12);
        memcpy(&P[3][6], &m_Pvp.m00(), 12);
        memcpy(&P[4][4], &m_Pvv.m11(), 8);
        memcpy(&P[4][6], &m_Pvp.m10(), 12);
        P[5][5] = m_Pvv.m22();
        memcpy(&P[5][6], &m_Pvp.m20(), 12);
        memcpy(&P[6][6], &m_Ppp.m00(), 12);
        memcpy(&P[7][7], &m_Ppp.m11(), 8);
        P[8][8] = m_Ppp.m22();
      }
      inline void IncreaseDiagonal(const float s2r, const float s2v, const float s2p) {
        m_Prr.IncreaseDiagonal(s2r);
        m_Pvv.IncreaseDiagonal(s2v);
        m_Ppp.IncreaseDiagonal(s2p);
      }
      inline void SetLowerFromUpper() {
        m_Prr.SetLowerFromUpper();
        m_Prv.GetTranspose(m_Pvr);
        m_Prp.GetTranspose(m_Ppr);
        m_Pvv.SetLowerFromUpper();
        m_Pvp.GetTranspose(m_Ppv);
        m_Ppp.SetLowerFromUpper();
      }
     public:
      LA::AlignedMatrix3x3f m_Prr, m_Prv, m_Prp;
      LA::AlignedMatrix3x3f m_Pvr, m_Pvv, m_Pvp;
      LA::AlignedMatrix3x3f m_Ppr, m_Ppv, m_Ppp;
    };
    class BD {
     public:
      LA::AlignedMatrix3x3f m_Pbav, m_Pbap;
      LA::AlignedMatrix3x3f m_Pbwr, m_Pbwv, m_Pbwp;
    };
    class DB {
     public:
      inline void Get(float **P, const int j) const {
        const int jbw = j + 3;
        memset(P[0] + j, 0, 12);              memcpy(P[0] + jbw, &m_Prbw.m00(), 12);
        memset(P[1] + j, 0, 12);              memcpy(P[1] + jbw, &m_Prbw.m10(), 12);
        memset(P[2] + j, 0, 12);              memcpy(P[2] + jbw, &m_Prbw.m20(), 12);
        memcpy(P[3] + j, &m_Pvba.m00(), 12);  memcpy(P[3] + jbw, &m_Pvbw.m00(), 12);
        memcpy(P[4] + j, &m_Pvba.m10(), 12);  memcpy(P[4] + jbw, &m_Pvbw.m10(), 12);
        memcpy(P[5] + j, &m_Pvba.m20(), 12);  memcpy(P[5] + jbw, &m_Pvbw.m20(), 12);
        memcpy(P[6] + j, &m_Ppba.m00(), 12);  memcpy(P[6] + jbw, &m_Ppbw.m00(), 12);
        memcpy(P[7] + j, &m_Ppba.m10(), 12);  memcpy(P[7] + jbw, &m_Ppbw.m10(), 12);
        memcpy(P[8] + j, &m_Ppba.m20(), 12);  memcpy(P[8] + jbw, &m_Ppbw.m20(), 12);
      }
      inline void GetTranspose(BD *P) const {
        m_Prbw.GetTranspose(P->m_Pbwr);
        m_Pvba.GetTranspose(P->m_Pbav);
        m_Pvbw.GetTranspose(P->m_Pbwv);
        m_Ppba.GetTranspose(P->m_Pbap);
        m_Ppbw.GetTranspose(P->m_Pbwp);
      }
     public:
      LA::AlignedMatrix3x3f m_Prbw;
      LA::AlignedMatrix3x3f m_Pvba, m_Pvbw;
      LA::AlignedMatrix3x3f m_Ppba, m_Ppbw;
    };
    class BB {
     public:
      inline void GetUpper(float **P, const int i, const int j) const {
        P[i][j] = P[i + 1][j + 1] = P[i + 2][j + 2] = m_Pbaba;
        P[i + 3][j + 3] = P[i + 4][j + 4] = P[i + 5][j + 5] = m_Pbwbw;
        memset(P[i] + j + 1, 0, 20);
        memset(P[i + 1] + j + 2, 0, 16);
        memset(P[i + 2] + j + 3, 0, 12);
        memset(P[i + 3] + j + 4, 0, 8);
        P[i + 4][j + 5] = 0.0f;
      }
      inline void IncreaseDiagonal(const float s2ba, const float s2bw) {
        m_Pbaba += s2ba;
        m_Pbwbw += s2bw;
      }
     public:
      float m_Pbaba, m_Pbwbw;
    };
   public:
    inline void MakeZero() { memset(this, 0, sizeof(Covariance)); }
    inline void IncreaseDiagonal(const float s2r, const float s2v, const float s2p,
                                 const float s2ba, const float s2bw) {
      m_Pdd.IncreaseDiagonal(s2r, s2v, s2p);
      m_Pbb.IncreaseDiagonal(s2ba, s2bw);
    }
    inline void SetLowerFromUpper() {
      m_Pdd.SetLowerFromUpper();
      //m_Pdb.GetTranspose(&m_Pbd);
    }
   public:
    static inline void ABT(const Transition::DD &A, const DD &B, DD *ABT) {
      ABT->m_Prr = B.m_Prr;
      ABT->m_Prv = B.m_Prv;
      ABT->m_Prp = B.m_Prp;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fvr, B.m_Prr, ABT->m_Pvr);
      SkewSymmetricMatrix::AB(A.m_Fvr, B.m_Prr, ABT->m_Pvr);
      ABT->m_Pvr += B.m_Pvr;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fvr, B.m_Pvr, ABT->m_Pvv);
      SkewSymmetricMatrix::AB(A.m_Fvr, B.m_Prv, ABT->m_Pvv);
      ABT->m_Pvv += B.m_Pvv;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fvr, B.m_Ppr, ABT->m_Pvp);
      SkewSymmetricMatrix::AB(A.m_Fvr, B.m_Prp, ABT->m_Pvp);
      ABT->m_Pvp += B.m_Pvp;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fpr, B.m_Prr, ABT->m_Ppr);
      SkewSymmetricMatrix::AB(A.m_Fpr, B.m_Prr, ABT->m_Ppr);
      //LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpv, B.m_Prv, ABT->m_Ppr);
      LA::AlignedMatrix3x3f::AddsATo(A.m_Fpv, B.m_Pvr, ABT->m_Ppr);
      ABT->m_Ppr += B.m_Ppr;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fpr, B.m_Pvr, ABT->m_Ppv);
      SkewSymmetricMatrix::AB(A.m_Fpr, B.m_Prv, ABT->m_Ppv);
      //LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpv, B.m_Pvv, ABT->m_Ppv);
      LA::AlignedMatrix3x3f::AddsATo(A.m_Fpv, B.m_Pvv, ABT->m_Ppv);
      ABT->m_Ppv += B.m_Ppv;
      //LA::AlignedMatrix3x3f::ABT(A.m_Fpr, B.m_Ppr, ABT->m_Ppp);
      SkewSymmetricMatrix::AB(A.m_Fpr, B.m_Prp, ABT->m_Ppp);
      //LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpv, B.m_Ppv, ABT->m_Ppp);
      LA::AlignedMatrix3x3f::AddsATo(A.m_Fpv, B.m_Pvp, ABT->m_Ppp);
      ABT->m_Ppp += B.m_Ppp;
    }
    //static inline void ABT(const Transition::DD &A, const BD &B, DB *ABT) {
    //  B.m_Pbwr.GetTranspose(ABT->m_Prbw);
    //  B.m_Pbav.GetTranspose(ABT->m_Pvba);
    //  B.m_Pbwv.GetTranspose(ABT->m_Pvbw);
    //  LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvr, B.m_Pbwr, ABT->m_Pvbw);
    //  B.m_Pbap.GetTranspose(ABT->m_Ppba);
    //  LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpv, B.m_Pbav, ABT->m_Ppba);
    //  B.m_Pbwp.GetTranspose(ABT->m_Ppbw);
    //  LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpr, B.m_Pbwr, ABT->m_Ppbw);
    //  LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpv, B.m_Pbwv, ABT->m_Ppbw);
    //}
    static inline void AB(const Transition::DD &A, const DB &B, DB *AB) {
      AB->m_Prbw = B.m_Prbw;
      AB->m_Pvba = B.m_Pvba;
      AB->m_Pvbw = B.m_Pvbw;
      SkewSymmetricMatrix::AddABTo(A.m_Fvr, B.m_Prbw, AB->m_Pvbw);
      AB->m_Ppba = B.m_Ppba;
      LA::AlignedMatrix3x3f::AddsATo(A.m_Fpv, B.m_Pvba, AB->m_Ppba);
      AB->m_Ppbw = B.m_Ppbw;
      SkewSymmetricMatrix::AddABTo(A.m_Fpr, B.m_Prbw, AB->m_Ppbw);
      LA::AlignedMatrix3x3f::AddsATo(A.m_Fpv, B.m_Pvbw, AB->m_Ppbw);
    }
    static inline void AddABTTo(const Transition::DB &A, const DB &B, DD *ABT) {
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Frbw, B.m_Prbw, ABT->m_Prr);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Frbw, B.m_Pvbw, ABT->m_Prv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Frbw, B.m_Ppbw, ABT->m_Prp);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvbw, B.m_Prbw, ABT->m_Pvr);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvba, B.m_Pvba, ABT->m_Pvv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvbw, B.m_Pvbw, ABT->m_Pvv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvba, B.m_Ppba, ABT->m_Pvp);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fvbw, B.m_Ppbw, ABT->m_Pvp);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpbw, B.m_Prbw, ABT->m_Ppr);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpba, B.m_Pvba, ABT->m_Ppv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpbw, B.m_Pvbw, ABT->m_Ppv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpba, B.m_Ppba, ABT->m_Ppp);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Fpbw, B.m_Ppbw, ABT->m_Ppp);
    }
    static inline void AddABTTo(const Transition::DB &A, const BB &B, DB *ABT) {
      const xp128f Bbaba = xp128f::get(B.m_Pbaba);
      const xp128f Bbwbw = xp128f::get(B.m_Pbwbw);
      LA::AlignedMatrix3x3f::AddsATo(Bbwbw, A.m_Frbw, ABT->m_Prbw);
      LA::AlignedMatrix3x3f::AddsATo(Bbaba, A.m_Fvba, ABT->m_Pvba);
      LA::AlignedMatrix3x3f::AddsATo(Bbwbw, A.m_Fvbw, ABT->m_Pvbw);
      LA::AlignedMatrix3x3f::AddsATo(Bbaba, A.m_Fpba, ABT->m_Ppba);
      LA::AlignedMatrix3x3f::AddsATo(Bbwbw, A.m_Fpbw, ABT->m_Ppbw);
    }
    static inline void ABT(const Transition &A, const Covariance &B, DD *ABTdd, DB *ABTdb) {
      ABT(A.m_Fdd, B.m_Pdd, ABTdd);
      //ABT(A.m_Fdd, B.m_Pbd, ABTdb);
      AB(A.m_Fdd, B.m_Pdb, ABTdb);
      AddABTTo(A.m_Fdb, B.m_Pdb, ABTdd);
      AddABTTo(A.m_Fdb, B.m_Pbb, ABTdb);
    }
    static inline void ABTToUpper(const DD &A, const Transition::DD &B, DD *ABT) {
      ABT->m_Prr = A.m_Prr;
      //LA::AlignedMatrix3x3f::ABT(A.m_Prr, B.m_Fvr, ABT->m_Prv);
      SkewSymmetricMatrix::ABT(A.m_Prr, B.m_Fvr, ABT->m_Prv);
      ABT->m_Prv += A.m_Prv;
      //LA::AlignedMatrix3x3f::ABT(A.m_Prr, B.m_Fpr, ABT->m_Prp);
      SkewSymmetricMatrix::ABT(A.m_Prr, B.m_Fpr, ABT->m_Prp);
      //LA::AlignedMatrix3x3f::AddABTTo(A.m_Prv, B.m_Fpv, ABT->m_Prp);
      LA::AlignedMatrix3x3f::AddsATo(B.m_Fpv, A.m_Prv, ABT->m_Prp);
      ABT->m_Prp += A.m_Prp;
      ABT->m_Pvv = A.m_Pvv;
      //LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Pvr, B.m_Fvr, ABT->m_Pvv);
      SkewSymmetricMatrix::AddABTToUpper(A.m_Pvr, B.m_Fvr, ABT->m_Pvv);
      //LA::AlignedMatrix3x3f::ABT(A.m_Pvr, B.m_Fpr, ABT->m_Pvp);
      SkewSymmetricMatrix::ABT(A.m_Pvr, B.m_Fpr, ABT->m_Pvp);
      //LA::AlignedMatrix3x3f::AddABTTo(A.m_Pvv, B.m_Fpv, ABT->m_Pvp);
      LA::AlignedMatrix3x3f::AddsATo(B.m_Fpv, A.m_Pvv, ABT->m_Pvp);
      ABT->m_Pvp += A.m_Pvp;
      ABT->m_Ppp = A.m_Ppp;
      //LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Ppr, B.m_Fpr, ABT->m_Ppp);
      SkewSymmetricMatrix::AddABTToUpper(A.m_Ppr, B.m_Fpr, ABT->m_Ppp);
      //LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Ppv, B.m_Fpv, ABT->m_Ppp);
      LA::AlignedMatrix3x3f::AddsAToUpper(B.m_Fpv, A.m_Ppv, ABT->m_Ppp);
    }
    static inline void AddABTToUpper(const DB &A, const Transition::DB &B, DD *ABT) {
      LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Prbw, B.m_Frbw, ABT->m_Prr);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Prbw, B.m_Fvbw, ABT->m_Prv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Prbw, B.m_Fpbw, ABT->m_Prp);
      LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Pvba, B.m_Fvba, ABT->m_Pvv);
      LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Pvbw, B.m_Fvbw, ABT->m_Pvv);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Pvba, B.m_Fpba, ABT->m_Pvp);
      LA::AlignedMatrix3x3f::AddABTTo(A.m_Pvbw, B.m_Fpbw, ABT->m_Pvp);
      LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Ppba, B.m_Fpba, ABT->m_Ppp);
      LA::AlignedMatrix3x3f::AddABTToUpper(A.m_Ppbw, B.m_Fpbw, ABT->m_Ppp);
    }
    static inline void ABTToUpper(const DD &Add, const DB &Adb, const Transition &B, DD *ABT) {
      ABTToUpper(Add, B.m_Fdd, ABT);
      AddABTToUpper(Adb, B.m_Fdb, ABT);
    }
    static inline void FPFT(const Transition &F, const Covariance &P, DD *U, Covariance *FPFT) {
      ABT(F, P, U, &FPFT->m_Pdb);
      ABTToUpper(*U, FPFT->m_Pdb, F, &FPFT->m_Pdd);
      FPFT->m_Pbb = P.m_Pbb;
      FPFT->SetLowerFromUpper();
    }
   public:
    DD m_Pdd;
    DB m_Pdb;
    //BD m_Pbd;
    BB m_Pbb;
  };
#ifdef CFG_IMU_FULL_COVARIANCE
  class Weight {
   public:
    inline const LA::AlignedMatrix3x3f* operator [] (const int i) const { return m_W[i]; }
    inline       LA::AlignedMatrix3x3f* operator [] (const int i)       { return m_W[i]; }
    inline void Set(const Covariance &P, AlignedVector<float> *work) {
      work->Resize(15 * 15);
      float *_P[15];
      _P[0] = work->Data();
      for (int i = 1; i < 15; ++i) {
        _P[i] = _P[i - 1] + 15;
      }
      P.m_Pdd.GetUpper(_P);
      P.m_Pdb.Get(_P, 9);
      P.m_Pbb.GetUpper(_P, 9, 9);
      if (LA::LS::InverseLDL<float>(15, _P)) {
        for (int i = 0, _i = 0; i < 5; ++i) {
          float *W0 = _P[_i++], *W1 = _P[_i++], *W2 = _P[_i++];
          for (int j = 0, _j = 0; j < 5; ++j, _j += 3) {
            m_W[i][j].Set(W0 + _j, W1 + _j, W2 + _j);
          }
        }
      } else {
        MakeZero();
      }
    }
    inline void GetScaled(const float w, Weight *W) const {
      const xp128f _w = xp128f::get(w);
      GetScaled(_w, W);
    }
    inline void GetScaled(const xp128f &w, Weight *W) const {
      for (int i = 0; i < 5; ++i) {
        m_W[i][i].GetScaledToUpper(w, W->m_W[i][i]);
        W->m_W[i][i].SetLowerFromUpper();
        for (int j = i + 1; j < 5; ++j) {
          m_W[i][j].GetScaled(w, W->m_W[i][j]);
          W->m_W[i][j].GetTranspose(W->m_W[j][i]);
        }
      }
    }
    inline void MakeZero() { memset(this, 0, sizeof(Weight)); }
    inline void SetLowerFromUpper() {
      for (int i = 0; i < 5; ++i) {
        m_W[i][i].SetLowerFromUpper();
        for (int j = i + 1; j < 5; ++j) {
          m_W[i][j].GetTranspose(m_W[j][i]);
        }
      }
    }
    inline bool AssertEqual(const Weight &W, const int verbose = 1,
                            const std::string str = "", const bool norm = true) const {
      bool scc = true;
      LA::AlignedMatrix3x3f W1, W2;
      for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
          W1 = m_W[i][j];
          W2 = W[i][j];
          if (norm) {
            const float n1 = sqrt(W1.SquaredFrobeniusNorm());
            const float n2 = sqrt(W2.SquaredFrobeniusNorm());
            const float n = std::max(n1, n2);
            if (n == 0.0f)
              continue;
            const float s = 1.0f / n;
            W1 *= s;
            W2 *= s;
          }
          scc = W1.AssertEqual(W2, verbose, str + UT::String(".W[%d][%d]", i, j)) && scc;
        }
      }
      return scc;
    }
   public:
    LA::AlignedMatrix3x3f m_W[5][5];
  };
#else
  class Weight {
   public:
    inline void GetScaled(const float s, Weight *W) const {
      const xp128f _s = xp128f::get(s);
      GetScaled(_s, W);
    }
    inline void GetScaled(const xp128f &s, Weight *W) const {
      m_Wr.GetScaled(s, W->m_Wr);
      m_Wv.GetScaled(s, W->m_Wv);
      m_Wp.GetScaled(s, W->m_Wp);
      W->m_wba = s[0] * m_wba;
      W->m_wbw = s[0] * m_wbw;
    }
    inline void SetLowerFromUpper() {
      m_Wr.SetLowerFromUpper();
      m_Wv.SetLowerFromUpper();
      m_Wp.SetLowerFromUpper();
    }
    inline bool AssertEqual(const Weight &W, const int verobse = 1,
                            const std::string str = "", const bool norm = true) const {
      bool scc = true;
      scc = AssertEqual(m_Wr, W.m_Wr, verobse, str + ".Wr", norm) && scc;
      scc = AssertEqual(m_Wv, W.m_Wv, verobse, str + ".Wv", norm) && scc;
      scc = AssertEqual(m_Wp, W.m_Wp, verobse, str + ".Wp", norm) && scc;
      scc = UT::AssertEqual(m_wba, W.m_wba, verobse, str + ".wba") && scc;
      scc = UT::AssertEqual(m_wbw, W.m_wbw, verobse, str + ".wbw") && scc;
      return scc;
    }
    static inline bool AssertEqual(const LA::AlignedMatrix3x3f &W1,
                                   const LA::AlignedMatrix3x3f &W2,
                                   const int verbose = 1, const std::string str = "",
                                   const bool norm = true) {
      if (norm) {
        const float n1 = sqrt(W1.SquaredFrobeniusNorm());
        const float n2 = sqrt(W2.SquaredFrobeniusNorm());
        const float n = std::max(n1, n2);
        if (n == 0.0f)
          return true;
        const float s = 1.0f / n;
        const LA::AlignedMatrix3x3f W1_ = W1 * s;
        const LA::AlignedMatrix3x3f W2_ = W2 * s;
        return AssertEqual(W1_, W2_, verbose, str, false);
      } else {
        return W1.AssertEqual(W2, verbose, str);
      }
    }
   public:
    union {
      struct { LA::AlignedMatrix3x3f m_Wr, m_Wv, m_Wp; };
      struct { float m_data1[3], m_wba, m_data2[7], m_wbw; };
    };
  };
#endif

  class Error {
   public:
    inline bool Valid() const { return m_er.Valid(); }
    inline bool Invalid() const { return m_er.Invalid(); }
    inline void Invalidate() { m_er.Invalidate(); }
    inline void Print(const bool e = false, const bool l = false) const {
      UT::PrintSeparator();
      m_er.Print(" er = ", e, l, true);
      m_ev.Print(" ev = ", e, l, true);
      m_ep.Print(" ep = ", e, l, true);
      m_eba.Print("eba = ", e, l, true);
      m_ebw.Print("ebw = ", e, l, true);
    }
    inline void Print(const std::string str, const bool e, const bool l) const {
      UT::PrintSeparator();
      const std::string _str(str.size(), ' ');
      m_er.Print( str + " er = ", e, l, true);
      m_ev.Print(_str + " ev = ", e, l, true);
      m_ep.Print(_str + " ep = ", e, l, true);
      m_eba.Print(_str + "eba = ", e, l, true);
      m_ebw.Print(_str + "ebw = ", e, l, true);
    }
   public:
    LA::AlignedVector3f m_er, m_ev, m_ep, m_eba, m_ebw;
  };
  class Jacobian {
   public:
    class Gravity {
    public:
      LA::AlignedMatrix2x3f m_JvgT, m_JpgT;
    };
    class FirstMotion {
     public:
      inline void GetTranspose(FirstMotion &JT) const {
        m_Jrbw1.GetTranspose(JT.m_Jrbw1);
        m_Jvv1.GetTranspose(JT.m_Jvv1);
        m_Jvba1.GetTranspose(JT.m_Jvba1);
        m_Jvbw1.GetTranspose(JT.m_Jvbw1);
        m_Jpv1.GetTranspose(JT.m_Jpv1);
        m_Jpba1.GetTranspose(JT.m_Jpba1);
        m_Jpbw1.GetTranspose(JT.m_Jpbw1);
      }
     public:
      LA::AlignedMatrix3x3f m_Jrbw1;
      LA::AlignedMatrix3x3f m_Jvv1, m_Jvba1, m_Jvbw1;
      LA::AlignedMatrix3x3f m_Jpv1, m_Jpba1, m_Jpbw1;
    };
    class Global : public FirstMotion {
     public:
      inline void GetTranspose(Global &JT) const {
        FirstMotion::GetTranspose(JT);
        m_Jrr1.GetTranspose(JT.m_Jrr1);
        m_Jvr1.GetTranspose(JT.m_Jvr1);
        m_Jpp1.GetTranspose(JT.m_Jpp1);
        m_Jpr1.GetTranspose(JT.m_Jpr1);
        m_Jpr2.GetTranspose(JT.m_Jpr2);
      }
     public:
      LA::AlignedMatrix3x3f m_Jrr1;
      LA::AlignedMatrix3x3f m_Jvr1;
      LA::AlignedMatrix3x3f m_Jpp1;
      LA::AlignedMatrix3x3f m_Jpr1;
      LA::AlignedMatrix3x3f m_Jpr2;
    };
    class RelativeLF : public Gravity, public Global {
     public:
      LA::AlignedMatrix3x3f m_Jvr2, m_Jvv2;
    };
    class RelativeKF : public Gravity, public FirstMotion {
     public:
      inline void GetTranspose(RelativeKF &JT) const {
        FirstMotion::GetTranspose(JT);
        m_Jrr2.GetTranspose(JT.m_Jrr2);
        m_Jvr2.GetTranspose(JT.m_Jvr2);
        m_Jvv2.GetTranspose(JT.m_Jvv2);
        m_Jpp2.GetTranspose(JT.m_Jpp2);
        m_Jpr2.GetTranspose(JT.m_Jpr2);
      }
     public:
      LA::AlignedMatrix3x3f m_Jrr2;
      LA::AlignedMatrix3x3f m_Jvr2, m_Jvv2;
      LA::AlignedMatrix3x3f m_Jpp2;
      LA::AlignedMatrix3x3f m_Jpr2;
    };
  };
  class ErrorJacobian {
   public:
    Error m_e;
    Jacobian::Global m_J;
  };
  class Factor {
   public:
    class Unitary {
     public:
      inline void Set(const LA::AlignedMatrix3x3f *Ap, const LA::AlignedMatrix3x3f *Ar,
                      const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                      const LA::AlignedMatrix3x3f *Abw, const LA::AlignedVector3f *b) {
        m_Acc.Set(Ap, Ar, b);
        m_Acm.Set(Ap + 2, Ar + 2);
        m_Amm.Set(Av + 2, Aba + 2, Abw + 2, b + 2);
      }
      inline void Set(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                      const LA::AlignedMatrix3x3f *Abw, const LA::AlignedVector3f *b) {
        m_Acc.MakeZero();
        m_Acm.MakeZero();
        m_Amm.Set(Av, Aba, Abw, b);
      }
      inline void MakeMinus() {
        m_Acc.MakeMinus();
        m_Acm.MakeMinus();
        m_Amm.MakeMinus();
      }
      static inline void AmB(const Unitary &A, const Unitary &B, Unitary &AmB) {
        Camera::Factor::Unitary::CC::AmB(A.m_Acc, B.m_Acc, AmB.m_Acc);
        Camera::Factor::Unitary::CM::AmB(A.m_Acm, B.m_Acm, AmB.m_Acm);
        Camera::Factor::Unitary::MM::AmB(A.m_Amm, B.m_Amm, AmB.m_Amm);
      }
     public:
      Camera::Factor::Unitary::CC m_Acc;
      Camera::Factor::Unitary::CM m_Acm;
      Camera::Factor::Unitary::MM m_Amm;
    };
    class Auxiliary {
     public:
      class Global {
       public:
        void Set(const Jacobian::Global &J, const Error &e, const float w, const Weight &W,
                 const float Tpv);
        inline void Get(Unitary *A11, Unitary *A22, Camera::Factor::Binary *A12) const {
#ifdef CFG_IMU_FULL_COVARIANCE
          //const LA::AlignedMatrix3x3f *A[10] = {&m_A[ 0], &m_A[ 9], &m_A[17], &m_A[24], &m_A[30],
          //                                      &m_A[35], &m_A[39], &m_A[42], &m_A[44], &m_A[45]};
          A11->Set(m_A, m_A + 9, m_A + 17, m_A + 24, m_A + 30, m_b);
          A22->Set(m_A + 40, m_A + 44, m_A + 47, m_A + 49, m_A + 50, m_b + 5);
          A12->Set(m_A + 5, m_A + 14, m_A + 22, m_A + 29, m_A + 35);
#else
          A11->m_Acc.m_A.Set(m_Ap1p1, m_Ap1r1, m_Ar1r1);
          A11->m_Acm.Set(m_Ap1v1, m_Ap1ba1, m_Ap1bw1, m_Ar1v1, m_Ar1ba1, m_Ar1bw1);
          A12->m_Acc.Set(m_Ap1p2, m_Ap1r2, m_Ar1p2, m_Ar1r2);
          A12->m_Acm.m_Arv = m_Ar1v2;
          A11->m_Acc.m_b.Set(m_bp1, m_br1);

          A11->m_Amm.m_A.Set(m_Av1v1, m_Av1ba1, m_Av1bw1, m_Aba1ba1, m_Aba1bw1, m_Abw1bw1);
          A12->m_Amc.Set(m_Av1p2, m_Av1r2, m_Aba1p2, m_Aba1r2, m_Abw1p2, m_Abw1r2);
          A12->m_Amm.m_Amv.Set(m_Av1v2, m_Aba1v2, m_Abw1v2);
          A12->m_Amm.m_Ababa = m_Aba1ba2;
          A12->m_Amm.m_Abwbw = m_Abw1bw2;
          A11->m_Amm.m_b.Set(m_bv1, m_bba1, m_bbw1);

          A22->m_Acc.m_A.Set(m_Ap2p2, m_Ap2r2, m_Ar2r2);
          A22->m_Acm.MakeZero();
          A22->m_Acc.m_b.Set(m_bp2, m_br2);
          A22->m_Amm.m_A.Set(m_Av2v2, m_Aba2ba2, m_Abw2bw2);
          A22->m_Amm.m_b.Set(m_bv2, m_bba2, m_bbw2);
#endif
        }
       public:
        Jacobian::Global m_JT;
        Weight m_W;
#ifdef CFG_IMU_FULL_COVARIANCE
        LA::AlignedMatrix3x3f m_JTW[10][5], m_A[55];
        LA::AlignedVector3f m_b[10];
#else
        LA::AlignedMatrix3x3f m_JTWr1r, m_JTWbw1r;
        LA::AlignedMatrix3x3f m_JTWr1v, m_JTWv1v, m_JTWba1v, m_JTWbw1v;
        LA::AlignedMatrix3x3f m_JTWp1p, m_JTWr1p, m_JTWv1p, m_JTWba1p, m_JTWbw1p, m_JTWr2p;
        LA::SymmetricMatrix3x3f m_Ap1p1, m_Ar1r1, m_Av1v1, m_Aba1ba1, m_Abw1bw1;
        LA::SymmetricMatrix3x3f m_Ap2p2, m_Ar2r2, m_Av2v2;
        LA::AlignedVector3f m_bp1, m_br1, m_bv1, m_bba1, m_bbw1, m_bp2, m_br2, m_bv2, m_bba2, m_bbw2;
        LA::AlignedMatrix3x3f m_Ap1r1, m_Ap1v1, m_Ap1ba1,  m_Ap1bw1,  m_Ap1p2,  m_Ap1r2;
        LA::AlignedMatrix3x3f          m_Ar1v1, m_Ar1ba1,  m_Ar1bw1,  m_Ar1p2,  m_Ar1r2,  m_Ar1v2;
        LA::AlignedMatrix3x3f                   m_Av1ba1,  m_Av1bw1,  m_Av1p2,  m_Av1r2,  m_Av1v2;
        LA::AlignedMatrix3x3f                             m_Aba1bw1, m_Aba1p2, m_Aba1r2, m_Aba1v2;
        LA::AlignedMatrix3x3f                                        m_Abw1p2, m_Abw1r2, m_Abw1v2;
        LA::AlignedMatrix3x3f                                                   m_Ap2r2;
        float m_Aba1ba2, m_Abw1bw2, m_Aba2ba2, m_Abw2bw2;
#endif
      };
      class RelativeLF : public Global {
       public:
        void Set(const Jacobian::RelativeLF &J, const Error &e, const float w, const Weight &W,
                 const float Tpv);
       public:
        LA::AlignedMatrix3x3f m_Jvr2T, m_Jvv2T;
#ifdef CFG_IMU_FULL_COVARIANCE
        LA::AlignedMatrix2x3f m_JTWg[5];
#else
        LA::AlignedMatrix3x3f m_JTWr2v, m_JTWv2v;
        LA::AlignedMatrix2x3f m_JTWgv, m_JTWgp;
        LA::AlignedMatrix3x3f m_Ar2v2;
#endif
        LA::SymmetricMatrix2x2f m_Agg;
#ifdef CFG_IMU_FULL_COVARIANCE
        LA::AlignedMatrix2x3f m_Agc[10];
#else
        LA::AlignedMatrix2x3f m_Agp1, m_Agr1, m_Agv1, m_Agba1, m_Agbw1;
        LA::AlignedMatrix2x3f m_Agp2, m_Agr2, m_Agv2;
#endif
        LA::Vector2f m_bg;
      };
      class RelativeKF {
       public:
        void Set(const Jacobian::RelativeKF &J, const Error &e, const float w, const Weight &W,
                 const float Tpv);
#ifdef CFG_IMU_FULL_COVARIANCE
        inline void Get(Unitary *A11, Unitary *A22, Camera::Factor::Binary *A12) const {
          A11->Set(m_Ac, m_Ac + 7, m_Ac + 13, m_bc);
          A22->Set(m_Ac + 21, m_Ac + 25, m_Ac + 28, m_Ac + 30, m_Ac + 31, m_bc + 3);
          A12->Set(m_Ac + 3, m_Ac + 10, m_Ac + 16);
        }
#endif
       public:
        Jacobian::RelativeKF m_JT;
        Weight m_W;
#ifdef CFG_IMU_FULL_COVARIANCE
        LA::AlignedMatrix3x3f m_JTWc[8][5], m_Ac[36];
        LA::AlignedVector3f m_bc[8];
        LA::AlignedMatrix2x3f m_JTWg[5];
        LA::SymmetricMatrix2x2f m_Agg;
        LA::AlignedMatrix2x3f m_Agc[8];
        LA::Vector2f m_bg;
#else
        LA::AlignedMatrix3x3f m_JTWbw1r, m_JTWr2r;
        LA::AlignedMatrix3x3f m_JTWv1v, m_JTWba1v, m_JTWbw1v, m_JTWr2v, m_JTWv2v;
        LA::AlignedMatrix3x3f m_JTWv1p, m_JTWba1p, m_JTWbw1p, m_JTWp2p, m_JTWr2p;
        LA::AlignedMatrix2x3f m_JTWgv, m_JTWgp;
        LA::SymmetricMatrix3x3f m_Av1v1, m_Aba1ba1, m_Abw1bw1, m_Ap2p2, m_Ar2r2, m_Av2v2;
        LA::AlignedVector3f m_bv1, m_bba1, m_bbw1, m_bp2, m_br2, m_bv2, m_bba2, m_bbw2;
        LA::AlignedMatrix3x3f m_Av1ba1,  m_Av1bw1,  m_Av1p2,  m_Av1r2,  m_Av1v2;
        LA::AlignedMatrix3x3f           m_Aba1bw1, m_Aba1p2, m_Aba1r2, m_Aba1v2;
        LA::AlignedMatrix3x3f                      m_Abw1p2, m_Abw1r2, m_Abw1v2;
        LA::AlignedMatrix3x3f                                 m_Ap2r2;
        LA::AlignedMatrix3x3f                                          m_Ar2v2;
        float m_Aba1ba2, m_Abw1bw2, m_Aba2ba2, m_Abw2bw2;
        LA::SymmetricMatrix2x2f m_Agg;
        LA::AlignedMatrix2x3f m_Agv1, m_Agba1, m_Agbw1, m_Agp2, m_Agr2, m_Agv2;
        LA::Vector2f m_bg;
#endif
      };
    };
   public:
    inline void MakeZero() { memset(this, 0, sizeof(Factor)); }
   public:
    ErrorJacobian m_Je;
    union {
      struct { float m_data[21], m_F; };
      struct { Unitary m_A11, m_A22; };
    };
  };
  class Reduction {
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
      m_ESr.Initialize();
      m_ESp.Initialize();
      m_ESv.Initialize();
      m_ESba.Initialize();
      m_ESbw.Initialize();
    }
    inline void Accumulate(const Error &e, const float F, const int iFrm = -1) {
      UT::ES<float, int>::Accumulate(F, F, iFrm);
      m_ESr.Accumulate(ESError(e.m_er, UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
      m_ESp.Accumulate(ESError(e.m_ep), -1.0f, iFrm);
      m_ESv.Accumulate(ESError(e.m_ev), -1.0f, iFrm);
      m_ESba.Accumulate(ESError(e.m_eba), -1.0f, iFrm);
      m_ESbw.Accumulate(ESError(e.m_ebw, UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
    }
    inline void Print(const std::string str = "", const bool l = true) const {
      if (!Valid()) {
        return;
      }
      UT::ES<float, int>::Print(str + "ed = ", true, l);
      const std::string _str(str.size() + 17, ' ');
      if (m_ESr.Valid()) {
        m_ESr.Print(_str + "er  = ", false, l);
      }
      if (m_ESp.Valid()) {
        m_ESp.Print(_str + "ep  = ", false, l);
      }
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
    UT::ES<ESError, int> m_ESr, m_ESp, m_ESv, m_ESba, m_ESbw;
  };
 public:

  inline bool Valid() const { return m_RT.Valid(); }
  inline bool Invalid() const { return m_RT.Invalid(); }
  inline void Invalidate() { m_RT.Invalidate(); }

#ifdef CFG_IMU_FULL_COVARIANCE
  inline Rotation3D GetRotationState(const Camera &C1, const Camera &C2) const {
    return Rotation3D(C1.m_T) / C2.m_T;
  }
  inline Rotation3D GetRotationMeasurement(const Camera &C1, const float eps) const {
    return m_RT / Rotation3D(m_Jrbw * (C1.m_bw - m_bw), eps);
  }
  inline Rotation3D GetRotationMeasurement(const LA::AlignedVector3f &dbw, const float eps) const {
    return m_RT / Rotation3D(m_Jrbw * dbw, eps);
  }
  inline LA::AlignedVector3f GetRotationError(const Camera &C1, const Camera &C2,
                                              const float eps) const {
    const Rotation3D eR = GetRotationMeasurement(C1, eps) / GetRotationState(C1, C2);
    return eR.GetRodrigues(eps);
  }
#else
  inline Rotation3D GetRotationState(const Camera &C1, const Camera &C2) const {
    return Rotation3D(C2.m_T) / C1.m_T;
  }
  inline Rotation3D GetRotationMeasurement(const Camera &C1, const float eps) const {
    return Rotation3D(m_Jrbw * (C1.m_bw - m_bw), eps) / m_RT;
  }
  inline LA::AlignedVector3f GetRotationError(const Camera &C1, const Camera &C2,
                                              const float eps) const {
    const Rotation3D eR = GetRotationState(C1, C2) / GetRotationMeasurement(C1, eps);
    return eR.GetRodrigues();
  }
#endif

  inline LA::AlignedVector3f GetVelocityMeasurement(const Camera &C1) const {
    return m_v + m_Jvba * (C1.m_ba - m_ba) + m_Jvbw * (C1.m_bw - m_bw);
  }
  inline LA::AlignedVector3f GetVelocityState(const Camera &C1, const Camera &C2) const {
    LA::AlignedVector3f dv = C2.m_v - C1.m_v;
    if (!IMU_GRAVITY_EXCLUDED) {
      dv.z() += IMU_GRAVITY_MAGNITUDE * m_Tvg;
    }
    return C1.m_T.GetAppliedRotation(dv);
  }
  inline LA::AlignedVector3f GetVelocityError(const Camera &C1, const Camera &C2) const {
    return GetVelocityState(C1, C2) - GetVelocityMeasurement(C1);
  }

  inline LA::AlignedVector3f GetPositionState(const Camera &C1, const Camera &C2,
                                              const Point3D &pu) const {
    LA::AlignedVector3f dp = C2.m_p - C1.m_p - C1.m_v * m_Tpv;
    if (!IMU_GRAVITY_EXCLUDED) {
      dp.z() += IMU_GRAVITY_MAGNITUDE * m_Tpg;
    }
    dp += C2.m_T.GetAppliedRotationInversely(pu);
    dp = C1.m_T.GetAppliedRotation(dp);
    dp -= pu;
    return dp;
  }
  inline LA::AlignedVector3f GetPositionMeasurement(const Camera &C1) const {
    return m_p + m_Jpba * (C1.m_ba - m_ba) + m_Jpbw * (C1.m_bw - m_bw);
  }
  inline LA::AlignedVector3f GetPositionError(const Camera &C1, const Camera &C2,
                                              const Point3D &pu) const {
    return GetPositionState(C1, C2, pu) - GetPositionMeasurement(C1);
  }
  
#ifdef CFG_DEBUG
  inline void DebugSetMeasurement(const Camera &C1, const Camera &C2, const Point3D &pu, const float eps) {
    const LA::AlignedVector3f dba = C1.m_ba - m_ba;
    const LA::AlignedVector3f dbw = C1.m_bw - m_bw;
    m_RT = GetRotationState(C1, C2) * Rotation3D(m_Jrbw * dbw, eps);
    m_v = GetVelocityState(C1, C2) - (m_Jvba * dba + m_Jvbw * dbw);
    m_p = GetPositionState(C1, C2, pu) - (m_Jpba * dba + m_Jpbw * dbw);
  }
#endif
  inline void GetError(const Camera &C1, const Camera &C2, const Point3D &pu, Error &e,
                       const float eps) const {
    e.m_er = GetRotationError(C1, C2, eps);
    e.m_ev = GetVelocityError(C1, C2);
    e.m_ep = GetPositionError(C1, C2, pu);
    e.m_eba = C1.m_ba - C2.m_ba;
    e.m_ebw = C1.m_bw - C2.m_bw;
  }
  inline Error GetError(const Camera &C1, const Camera &C2, const Point3D &pu,
                        const float eps) const {
    Error e;
    GetError(C1, C2, pu, e, eps);
    return e;
  }
  static inline void GetError(const ErrorJacobian &Je, const LA::AlignedVector3f *xp1,
                              const LA::AlignedVector3f *xr1, const LA::AlignedVector3f *xv1,
                              const LA::AlignedVector3f *xba1, const LA::AlignedVector3f *xbw1, 
                              const LA::AlignedVector3f *xp2, const LA::AlignedVector3f *xr2,
                              const LA::AlignedVector3f *xv2, const LA::AlignedVector3f *xba2,
                              const LA::AlignedVector3f *xbw2, Error &e) {
#ifdef CFG_DEBUG
    UT_ASSERT(xp1 || xr1 || xv1 || xba1 || xbw1 || xp2 || xr2 || xv2 || xba2 || xbw2);
#endif
    e = Je.m_e;
    if (xp1) {
      if (xp2) {
        const LA::AlignedVector3f dxp = *xp1 - *xp2;
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpp1, dxp, (float *) &e.m_ep);
      } else {
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpp1, *xp1, (float *) &e.m_ep);
      }
    } else if (xp2) {
      LA::AlignedMatrix3x3f::SubtractAbFrom(Je.m_J.m_Jpp1, *xp2, (float *) &e.m_ep);
    }
    if (xr1) {
      if (xr2) {
        const LA::AlignedVector3f dxr = *xr1 - *xr2;
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jrr1, dxr, (float *) &e.m_er);
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpr2, *xr2, (float *) &e.m_ep);
      } else {
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jrr1, *xr1, (float *) &e.m_er);
      }
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvr1, *xr1, (float *) &e.m_ev);
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpr1, *xr1, (float *) &e.m_ep);
    } else if (xr2) {
      LA::AlignedMatrix3x3f::SubtractAbFrom(Je.m_J.m_Jrr1, *xr2, (float *) &e.m_er);
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpr2, *xr2, (float *) &e.m_ep);
    }
    if (xv1) {
      if (xv2) {
        const LA::AlignedVector3f dxv = *xv1 - *xv2;
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvv1, dxv, (float *) &e.m_ev);
      } else {
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvv1, *xv1, (float *) &e.m_ev);
      }
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpv1, *xv1, (float *) &e.m_ep);
    } else if (xv2) {
      LA::AlignedMatrix3x3f::SubtractAbFrom(Je.m_J.m_Jvv1, *xv2, (float *) &e.m_ev);
    }
    if (xba1) {
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvba1, *xba1, (float *) &e.m_ev);
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpba1, *xba1, (float *) &e.m_ep);
      e.m_eba += *xba1;
    }
    if (xbw1) {
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jrbw1, *xbw1, (float *) &e.m_er);
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jvbw1, *xbw1, (float *) &e.m_ev);
      LA::AlignedMatrix3x3f::AddAbTo(Je.m_J.m_Jpbw1, *xbw1, (float *) &e.m_ep);
      e.m_ebw += *xbw1;
    }
    if (xba2) {
      e.m_eba -= *xba2;
    }
    if (xbw2) {
      e.m_ebw -= *xbw2;
    }
  }
  inline void GetError(const Jacobian::RelativeLF &J, const LA::Vector2f &xg,
                       const LA::AlignedVector3f &xp1, const LA::AlignedVector3f &xr1,
                       const LA::AlignedVector3f &xv1, const LA::AlignedVector3f &xba1,
                       const LA::AlignedVector3f &xbw1, const LA::AlignedVector3f &xp2,
                       const LA::AlignedVector3f &xr2, const LA::AlignedVector3f &xv2,
                       const LA::AlignedVector3f &xba2, const LA::AlignedVector3f &xbw2,
                       Error &e) const {
    GetError(J, xg, xp1, xr1, xv1, xba1, xbw1, xp2, xr2, xv2, xba2, xbw2, m_Tpv, e);
  }
  static inline void GetError(const Jacobian::RelativeLF &J, const LA::Vector2f &xg,
                              const LA::AlignedVector3f &xp1, const LA::AlignedVector3f &xr1,
                              const LA::AlignedVector3f &xv1, const LA::AlignedVector3f &xba1,
                              const LA::AlignedVector3f &xbw1, const LA::AlignedVector3f &xp2,
                              const LA::AlignedVector3f &xr2, const LA::AlignedVector3f &xv2,
                              const LA::AlignedVector3f &xba2, const LA::AlignedVector3f &xbw2,
                              const float Tpv, Error &e) {
    const LA::AlignedVector3f dxr = xr1 - xr2;
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jrr1, dxr, (float *) &e.m_er);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jrbw1, xbw1, (float *) &e.m_er);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvr1, xr1, (float *) &e.m_ev);
    //LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvv1, xv1, (float *) &e.m_ev);
    e.m_ev -= xv1;
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvba1, xba1, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvbw1, xbw1, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvr2, xr2, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvv2, xv2, (float *) &e.m_ev);
    LA::AlignedMatrix2x3f::AddATbTo(J.m_JvgT, xg, e.m_ev);
    const LA::AlignedVector3f dxp = xp1 - xp2;
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpp1, dxp, (float *) &e.m_ep);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpr1, xr1, (float *) &e.m_ep);
    //LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpv1, xv1, e.m_ep);
    e.m_ep -= (xv1 * Tpv);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpba1, xba1, (float *) &e.m_ep);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpbw1, xbw1, (float *) &e.m_ep);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpr2, xr2, (float *) &e.m_ep);
    LA::AlignedMatrix2x3f::AddATbTo(J.m_JpgT, xg, e.m_ep);
    e.m_eba += xba1;
    e.m_eba -= xba2;
    e.m_ebw += xbw1;
    e.m_ebw -= xbw2;
  }
  inline void GetError(const Jacobian::RelativeKF &J, const LA::Vector2f &xg,
                       const LA::AlignedVector3f &xv1, const LA::AlignedVector3f &xba1,
                       const LA::AlignedVector3f &xbw1, const LA::AlignedVector3f &xp2,
                       const LA::AlignedVector3f &xr2, const LA::AlignedVector3f &xv2,
                       const LA::AlignedVector3f &xba2, const LA::AlignedVector3f &xbw2,
                       Error &e) const {
    GetError(J, xg, xv1, xba1, xbw1, xp2, xr2, xv2, xba2, xbw2, m_Tpv, e);
  }

  static inline void GetError(const Jacobian::RelativeKF &J, const LA::Vector2f &xg,
                              const LA::AlignedVector3f &xv1, const LA::AlignedVector3f &xba1,
                              const LA::AlignedVector3f &xbw1, const LA::AlignedVector3f &xp2,
                              const LA::AlignedVector3f &xr2, const LA::AlignedVector3f &xv2,
                              const LA::AlignedVector3f &xba2, const LA::AlignedVector3f &xbw2,
                              const float Tpv, Error &e) {
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jrbw1, xbw1, (float *) &e.m_er);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jrr2, xr2, (float *) &e.m_er);
    //LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvv1, xv1, (float *) &e.m_ev);
    e.m_ev -= xv1;
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvba1, xba1, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvbw1, xbw1, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvr2, xr2, (float *) &e.m_ev);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jvv2, xv2, (float *) &e.m_ev);
    LA::AlignedMatrix2x3f::AddATbTo(J.m_JvgT, xg, e.m_ev);
    //LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpv1, xv1, e.m_ep);
    e.m_ep -= (xv1 * Tpv);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpba1, xba1, (float *) &e.m_ep);
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpbw1, xbw1, (float *) &e.m_ep);
    //LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpp2, xp2, (float *) &e.m_ep);
    e.m_ep += xp2;
    LA::AlignedMatrix3x3f::AddAbTo(J.m_Jpr2, xr2, (float *) &e.m_ep);
    LA::AlignedMatrix2x3f::AddATbTo(J.m_JpgT, xg, e.m_ep);
    e.m_eba += xba1;
    e.m_eba -= xba2;
    e.m_ebw += xbw1;
    e.m_ebw -= xbw2;
  }
  inline void GetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                               Error *e, Jacobian::Global *J, const float eps) const {
#ifdef CFG_IMU_FULL_COVARIANCE
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = m_RT / Rotation3D(drbw, eps) / dR;
    eR.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr1, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrr1.MakeMinus();
    J->m_Jrbw1 = J->m_Jrr1 * m_RT * J->m_Jrbw1 * m_Jrbw;
    J->m_Jrr1 = J->m_Jrr1 * eR * C1.m_T;
#else
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = dR * m_RT, eRub = eR / Rotation3D(drbw);
    eRub.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr1, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrr1.MakeMinus();
    J->m_Jrbw1 = J->m_Jrr1 * eR * J->m_Jrbw1 * m_Jrbw;
    J->m_Jrr1 = J->m_Jrr1 * C2.m_T;
#endif

    e->m_ev = GetVelocityState(C1, C2);
    SkewSymmetricMatrix::AB(e->m_ev, C1.m_T, J->m_Jvr1);
    e->m_ev -= GetVelocityMeasurement(C1);
    C1.m_T.GetMinus(J->m_Jvv1);
    m_Jvba.GetMinus(J->m_Jvba1);
    m_Jvbw.GetMinus(J->m_Jvbw1);

    e->m_ep = C2.m_p - C1.m_p - C1.m_v * m_Tpv;
    if (!IMU_GRAVITY_EXCLUDED) {
      e->m_ep.z() += IMU_GRAVITY_MAGNITUDE * m_Tpg;
    }
    e->m_ep = C1.m_T.GetAppliedRotation(e->m_ep);
#ifdef CFG_IMU_FULL_COVARIANCE
    const LA::AlignedVector3f R21pu = dR.GetApplied(pu);
#else
    const LA::AlignedVector3f R21pu = dR.GetAppliedInversely(pu);
#endif
    e->m_ep += R21pu;
    SkewSymmetricMatrix::AB(e->m_ep, C1.m_T, J->m_Jpr1);
    e->m_ep -= pu;
    e->m_ep -= GetPositionMeasurement(C1);
    C1.m_T.GetMinus(J->m_Jpp1);
    C1.m_T.GetScaled(-m_Tpv, J->m_Jpv1);
    m_Jpba.GetMinus(J->m_Jpba1);
    m_Jpbw.GetMinus(J->m_Jpbw1);
    SkewSymmetricMatrix::ATB(R21pu, C1.m_T, J->m_Jpr2);

    e->m_eba = C1.m_ba - C2.m_ba;
    e->m_ebw = C1.m_bw - C2.m_bw;

    //J->m_Jpp1.MakeZero();
    //J->m_Jpr1.MakeZero();
    //J->m_Jpba1.MakeZero();
    //J->m_Jpbw1.MakeZero();
    //J->m_Jpr2.MakeZero();
  }
  inline void GetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                               const Rotation3D &Rg, Error *e, Jacobian::RelativeLF *J,
                               const float eps) const {
#ifdef CFG_IMU_FULL_COVARIANCE
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = m_RT / Rotation3D(drbw, eps) / dR;
    eR.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr1, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrr1.MakeMinus();
    J->m_Jrbw1 = J->m_Jrr1 * m_RT * J->m_Jrbw1 * m_Jrbw;
    J->m_Jrr1 = J->m_Jrr1 * eR;
    const Rotation3D R1T = Rg / C1.m_T;
    J->m_Jrr1 = LA::AlignedMatrix3x3f::GetABT(J->m_Jrr1, R1T);
#else
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = dR * m_RT, eRub = eR / Rotation3D(drbw, eps);
    eRub.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr1, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrr1.MakeMinus();
    J->m_Jrbw1 = J->m_Jrr1 * eR * J->m_Jrbw1 * m_Jrbw;
    const Rotation3D R2T = Rg / C2.m_T;
    J->m_Jrr1 = LA::AlignedMatrix3x3f::GetABT(J->m_Jrr1, R2T);
#endif

    C1.m_T.ApplyRotation(C2.m_v, e->m_ev);
#ifdef CFG_IMU_FULL_COVARIANCE
    J->m_Jvv2 = dR;
#else
    dR.LA::AlignedMatrix3x3f::GetTranspose(J->m_Jvv2);
#endif
    LA::AlignedMatrix3x3f::Ab(J->m_Jvv2, pu, (float *) &e->m_ep);
    //const Rotation3D R1 = Rotation3D(C1.m_T) / Rg;
    const Rotation3D R1 = R1T.GetTranspose();
    SkewSymmetricMatrix::ATB(e->m_ev, R1, J->m_Jvr2);
    SkewSymmetricMatrix::ATB(e->m_ep, R1, J->m_Jpr2);
    if (IMU_GRAVITY_EXCLUDED) {
      J->m_JvgT.Invalidate();
      J->m_JpgT.Invalidate();
    } else {
      const LA::AlignedVector3f g1 = C1.m_T.GetColumn2();
      const LA::AlignedVector3f dv = g1 * (m_Tvg * IMU_GRAVITY_MAGNITUDE);
      e->m_ev += dv;
      SkewSymmetricMatrix::ATBT(C1.m_T, dv, J->m_JvgT);
      const LA::AlignedVector3f dp = g1 * (m_Tpg * IMU_GRAVITY_MAGNITUDE);
      e->m_ep += dp;
      SkewSymmetricMatrix::ATBT(C1.m_T, dp, J->m_JpgT);
    }
    SkewSymmetricMatrix::AB(e->m_ev, R1, J->m_Jvr1);
    const LA::AlignedVector3f v1 = C1.m_T.GetAppliedRotation(C1.m_v);
    e->m_ev -= v1;
    e->m_ev -= GetVelocityMeasurement(C1);
    //J->m_Jvv1.MakeDiagonal(-1.0);
#ifdef CFG_DEBUG
    J->m_Jvv1.Invalidate();
#endif
    m_Jvba.GetMinus(J->m_Jvba1);
    m_Jvbw.GetMinus(J->m_Jvbw1);

    e->m_ep += C1.m_T.GetAppliedRotation(C2.m_p - C1.m_p);
    SkewSymmetricMatrix::AB(e->m_ep, R1, J->m_Jpr1);
    e->m_ep -= v1 * m_Tpv;
    e->m_ep -= pu;
    e->m_ep -= GetPositionMeasurement(C1);
    R1.GetMinus(J->m_Jpp1);
    //J->m_Jpv1.MakeDiagonal(-m_Tpv);
#ifdef CFG_DEBUG
    J->m_Jpv1.Invalidate();
#endif
    m_Jpba.GetMinus(J->m_Jpba1);
    m_Jpbw.GetMinus(J->m_Jpbw1);

    e->m_eba = C1.m_ba - C2.m_ba;
    e->m_ebw = C1.m_bw - C2.m_bw;
  }
  inline void GetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                               Error *e, Jacobian::RelativeKF *J, const float eps) const {
#ifdef CFG_IMU_FULL_COVARIANCE
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = m_RT / Rotation3D(drbw, eps) / dR;
    eR.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr2, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrbw1 = J->m_Jrr2 * m_RT * J->m_Jrbw1 * m_Jrbw;
    J->m_Jrbw1.MakeMinus();
    J->m_Jrr2 = J->m_Jrr2 * eR;
#else
    const Rotation3D dR = GetRotationState(C1, C2);
    const LA::AlignedVector3f drbw = m_Jrbw * (C1.m_bw - m_bw);
    const Rotation3D eR = dR * m_RT, eRub = eR / Rotation3D(drbw, eps);
    eRub.GetRodrigues(e->m_er, eps);
    Rotation3D::GetRodriguesJacobianInverse(e->m_er, J->m_Jrr2, eps);
    Rotation3D::GetRodriguesJacobian(drbw.GetMinus(), J->m_Jrbw1, eps);
    J->m_Jrbw1 = J->m_Jrr2 * eR * J->m_Jrbw1 * m_Jrbw;
    J->m_Jrbw1.MakeMinus();
    const Rotation3D R2T = Rotation3D(C1.m_T) / C2.m_T;
    J->m_Jrr2 = LA::AlignedMatrix3x3f::GetABT(J->m_Jrr2, R2T);
#endif

    C1.m_T.ApplyRotation(C2.m_v, e->m_ev);
#ifdef CFG_IMU_FULL_COVARIANCE
    J->m_Jvv2 = dR;
#else
    dR.LA::AlignedMatrix3x3f::GetTranspose(J->m_Jvv2);
#endif
    LA::AlignedMatrix3x3f::Ab(J->m_Jvv2, pu, (float *) &e->m_ep);
    SkewSymmetricMatrix::GetTranspose(e->m_ev, J->m_Jvr2);
    SkewSymmetricMatrix::GetTranspose(e->m_ep, J->m_Jpr2);
    if (IMU_GRAVITY_EXCLUDED) {
      J->m_JvgT.Invalidate();
      J->m_JpgT.Invalidate();
    } else {
      const LA::AlignedVector3f g1 = C1.m_T.GetColumn2();
      const LA::AlignedVector3f dv = g1 * (m_Tvg * IMU_GRAVITY_MAGNITUDE);
      e->m_ev += dv;
      SkewSymmetricMatrix::ATBT(C1.m_T, dv, J->m_JvgT);
      const LA::AlignedVector3f dp = g1 * (m_Tpg * IMU_GRAVITY_MAGNITUDE);
      e->m_ep += dp;
      SkewSymmetricMatrix::ATBT(C1.m_T, dp, J->m_JpgT);
    }
    const LA::AlignedVector3f v1 = C1.m_T.GetAppliedRotation(C1.m_v);
    e->m_ev -= v1;
    e->m_ev -= GetVelocityMeasurement(C1);
    //J->m_Jvv1.MakeDiagonal(-1.0);
#ifdef CFG_DEBUG
    J->m_Jvv1.Invalidate();
#endif
    m_Jvba.GetMinus(J->m_Jvba1);
    m_Jvbw.GetMinus(J->m_Jvbw1);

    e->m_ep += C1.m_T.GetAppliedRotation(C2.m_p - C1.m_p);
    e->m_ep -= v1 * m_Tpv;
    e->m_ep -= pu;
    e->m_ep -= GetPositionMeasurement(C1);
    //J->m_Jpv1.MakeDiagonal(-m_Tpv);
    //J->m_Jpp2.MakeIdentity();
#ifdef CFG_DEBUG
    J->m_Jpv1.Invalidate();
    J->m_Jpp2.Invalidate();
#endif
    m_Jpba.GetMinus(J->m_Jpba1);
    m_Jpbw.GetMinus(J->m_Jpbw1);

    e->m_eba = C1.m_ba - C2.m_ba;
    e->m_ebw = C1.m_bw - C2.m_bw;
  }
  inline void GetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                        Factor *A, Camera::Factor::Binary *A12, Factor::Auxiliary::Global *U,
                        const float eps) const {
    GetErrorJacobian(C1, C2, pu, &A->m_Je.m_e, &A->m_Je.m_J, eps);
    A->m_F = GetCost(w, A->m_Je.m_e);
    U->Set(A->m_Je.m_J, A->m_Je.m_e, w, m_W, m_Tpv);
    U->Get(&A->m_A11, &A->m_A22, A12);
  }
  inline void GetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                        const Rotation3D &Rg, Error *e, Jacobian::RelativeLF *J,
                        Factor::Auxiliary::RelativeLF *U, const float eps) const {
    GetErrorJacobian(C1, C2, pu, Rg, e, J, eps);
    U->Set(*J, *e, w, m_W, m_Tpv);
  }
  inline void GetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                        Error *e, Jacobian::RelativeKF *J, Factor::Auxiliary::RelativeKF *U,
                        const float eps) const {
    GetErrorJacobian(C1, C2, pu, e, J, eps);
    U->Set(*J, *e, w, m_W, m_Tpv);
  }
  inline float GetCost(const float w, const Error &e) const {
    return GetCost(w, m_W, e);
  }
  static inline float GetCost(const float w, const Weight &W, const Error &e) {
#ifdef CFG_IMU_FULL_COVARIANCE
    LA::AlignedVector3f We;
    float F = 0.0f;
    const LA::AlignedVector3f *_e = (LA::AlignedVector3f *) &e;
    for (int i = 0; i < 5; ++i) {
      We.MakeZero();
      const LA::AlignedMatrix3x3f *Wi = W[i];
      for (int j = 0; j < 5; ++j) {
        LA::AlignedMatrix3x3f::AddAbTo(Wi[j], _e[j], (float *) &We);
      }
      F += _e[i].Dot(We);
    }
    return w * F;
#else
    return w * (LA::SymmetricMatrix3x3f::MahalanobisDistance(W.m_Wr, e.m_er) +
                LA::SymmetricMatrix3x3f::MahalanobisDistance(W.m_Wv, e.m_ev) +
                LA::SymmetricMatrix3x3f::MahalanobisDistance(W.m_Wp, e.m_ep) +
                W.m_wba * e.m_eba.SquaredLength() +
                W.m_wbw * e.m_ebw.SquaredLength());
#endif
  }
  inline float GetCost(const float w, const ErrorJacobian &Je, const LA::AlignedVector3f *xp1,
                       const LA::AlignedVector3f *xr1, const LA::AlignedVector3f *xv1,
                       const LA::AlignedVector3f *xba1, const LA::AlignedVector3f *xbw1,
                       const LA::AlignedVector3f *xp2, const LA::AlignedVector3f *xr2,
                       const LA::AlignedVector3f *xv2, const LA::AlignedVector3f *xba2,
                       const LA::AlignedVector3f *xbw2, Error &e) const {
    GetError(Je, xp1, xr1, xv1, xba1, xbw1, xp2, xr2, xv2, xba2, xbw2, e);
    return GetCost(w, e);
  }
  inline void GetReduction(const float w, const Factor &A, const Camera &C1, const Camera &C2,
                           const Point3D &pu, const LA::AlignedVector3f *xp1,
                           const LA::AlignedVector3f *xr1, const LA::AlignedVector3f *xv1,
                           const LA::AlignedVector3f *xba1, const LA::AlignedVector3f *xbw1,
                           const LA::AlignedVector3f *xp2, const LA::AlignedVector3f *xr2, 
                           const LA::AlignedVector3f *xv2, const LA::AlignedVector3f *xba2,
                           const LA::AlignedVector3f *xbw2, Reduction &Ra, Reduction &Rp,
                           const float eps) const {
    GetError(C1, C2, pu, Ra.m_e, eps);
    GetError(A.m_Je, xp1, xr1, xv1, xba1, xbw1, xp2, xr2, xv2, xba2, xbw2, Rp.m_e);
    Ra.m_dF = A.m_F - (Ra.m_F = GetCost(w, Ra.m_e));
    Rp.m_dF = A.m_F - (Rp.m_F = GetCost(w, Rp.m_e));
  }

  inline bool AssertEqual(const Delta &D, const int verbose = 1, const std::string str = "",
    const bool normWeight = true) const {
    bool scc = true;
    scc = m_ba.AssertEqual(D.m_ba, verbose, str + ".m_ba") && scc;
    scc = m_bw.AssertEqual(D.m_bw, verbose, str + ".m_bw") && scc;
    scc = m_RT.AssertEqual(D.m_RT, verbose, str + ".m_R") && scc;
    scc = m_v.AssertEqual(D.m_v, verbose, str + ".m_v") && scc;
    scc = m_p.AssertEqual(D.m_p, verbose, str + ".m_p") && scc;
    scc = m_Jrbw.AssertEqual(D.m_Jrbw, verbose, str + ".m_Jrbw") && scc;
    scc = m_Jvba.AssertEqual(D.m_Jvba, verbose, str + ".m_Jvba") && scc;
    scc = m_Jvbw.AssertEqual(D.m_Jvbw, verbose, str + ".m_Jvbw") && scc;
    scc = m_Jpba.AssertEqual(D.m_Jpba, verbose, str + ".m_Jpba") && scc;
    scc = m_Jpbw.AssertEqual(D.m_Jpbw, verbose, str + ".m_Jpbw") && scc;
    scc = m_W.AssertEqual(D.m_W, verbose, str + ".m_W", normWeight) && scc;
    scc = UT::AssertEqual(m_Tvg, D.m_Tvg, verbose, str + ".m_Tvg") && scc;
    scc = UT::AssertEqual(m_Tpv, D.m_Tpv, verbose, str + ".m_Tpv") && scc;
    scc = UT::AssertEqual(m_Tpg, D.m_Tpg, verbose, str + ".m_Tpg") && scc;
    return scc;
  }
  inline void Print(const bool e = false) const {
    UT::PrintSeparator();
    m_ba.Print("  ba = ", e, false, true);
    m_bw.Print("  bw = ", e, false, true);
    m_RT.Print("  RT = ", e);
    m_v.Print("   v = ", e, false, true);
    m_p.Print("   p = ", e, false, true);
    m_Jrbw.Print("Jrbw = ", e);
    m_Jvba.Print("Jvba = ", e);
    m_Jvbw.Print("Jvbw = ", e);
    m_Jpba.Print("Jpba = ", e);
    m_Jpbw.Print("Jpbw = ", e);
    if (e) {
      UT::Print(" Tvg = %e\n", m_Tvg);
    } else {
      UT::Print(" Tvg = %f\n", m_Tvg);
    }
  }
 public:
  Measurement m_u1, m_u2;
  LA::AlignedVector3f m_ba, m_bw;
  Rotation3D m_RT;
  LA::AlignedVector3f m_v, m_p;
  LA::AlignedMatrix3x3f m_Jrbw, m_Jvba, m_Jvbw, m_Jpba, m_Jpbw;
  Weight m_W;
  float m_Tvg, m_Tpv, m_Tpg, m_r;
#ifdef CFG_DEBUG_EIGEN
 public:
  class EigenTransition : public EigenMatrix15x15f {
   public:
    class DD : public EigenMatrix9x9f {
     public:
      inline DD() {}
      inline DD(const Eigen::Matrix<float, 9, 9> &e_P) {
        *((EigenMatrix9x9f *) this) = e_P;
      }
      inline DD(const Transition::DD &F) { Set(F); }
      inline void Set(const Transition::DD &F) {
        setIdentity();
        //block<3, 3>(3, 0) = EigenMatrix3x3f(F.m_Fvr);
        //block<3, 3>(6, 0) = EigenMatrix3x3f(F.m_Fpr);
        //block<3, 3>(6, 3) = EigenMatrix3x3f(F.m_Fpv);
        block<3, 3>(3, 0) = EigenMatrix3x3f(F.m_Fvr.GetAlignedMatrix3x3f());
        block<3, 3>(6, 0) = EigenMatrix3x3f(F.m_Fpr.GetAlignedMatrix3x3f());
        block<3, 3>(6, 3) = EigenMatrix3x3f(F.m_Fpv[0]);
      }
      inline bool AssertEqual(const Transition::DD &F, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        const EigenMatrix3x3f e_I = EigenMatrix3x3f::Identity();
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertEqual(e_I, verbose, str + ".Frr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertZero(verbose, str + ".Frv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 6)).AssertZero(verbose, str + ".Frp") && scc;
        //scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(F.m_Fvr, verbose, str + ".Fvr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(F.m_Fvr.GetAlignedMatrix3x3f(), verbose, str + ".Fvr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(e_I, verbose, str + ".Fvv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 6)).AssertZero(verbose, str + ".Fvp") && scc;
        //scc = EigenMatrix3x3f(block<3, 3>(6, 0)).AssertEqual(F.m_Fpr, verbose, str + ".Fpr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 0)).AssertEqual(F.m_Fpr.GetAlignedMatrix3x3f(), verbose, str + ".Fpr") && scc;
        //scc = EigenMatrix3x3f(block<3, 3>(6, 3)).AssertEqual(F.m_Fpv, verbose, str + ".Fpv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 3)).AssertEqual(EigenMatrix3x3f(F.m_Fpv[0]), verbose, str + ".Fpv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 6)).AssertEqual(e_I, verbose, str + ".Fpp") && scc;
        return scc;
      }
    };
    class DB : public EigenMatrix9x6f {
     public:
      inline DB() {}
      inline DB(const Eigen::Matrix<float, 9, 6> &e_P) {
        *((EigenMatrix9x6f *) this) = e_P;
      }
      inline DB(const Transition::DB &F) { Set(F); }
      inline void Set(const Transition::DB &F) {
        block<3, 3>(0, 0) = EigenMatrix3x3f::Zero();
        block<3, 3>(0, 3) = EigenMatrix3x3f(F.m_Frbw);
        block<3, 3>(3, 0) = EigenMatrix3x3f(F.m_Fvba);
        block<3, 3>(3, 3) = EigenMatrix3x3f(F.m_Fvbw);
        block<3, 3>(6, 0) = EigenMatrix3x3f(F.m_Fpba);
        block<3, 3>(6, 3) = EigenMatrix3x3f(F.m_Fpbw);
      }
      inline bool AssertEqual(const Transition::DB &F, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertZero(verbose, str + ".Frba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertEqual(F.m_Frbw, verbose, str + ".Frbw") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(F.m_Fvba, verbose, str + ".Fvba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(F.m_Fvbw, verbose, str + ".Fvbw") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 0)).AssertEqual(F.m_Fpba, verbose, str + ".Fpba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 3)).AssertEqual(F.m_Fpbw, verbose, str + ".Fpbw") && scc;
        return scc;
      }
    };
   public:
    inline EigenTransition() {}
    inline EigenTransition(const Eigen::Matrix<float, 15, 15, Eigen::RowMajor> &e_F) {
      *((EigenMatrix15x15f *) this) = e_F;
    }
    inline void Set(const Transition &F) {
      block<9, 9>(0, 0) = DD(F.m_Fdd);
      block<9, 6>(0, 9) = DB(F.m_Fdb);
      block<6, 9>(9, 0).setZero();
      block<6, 6>(9, 9).setIdentity();
    }
    inline bool AssertEqual(const Transition &F, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = DD(block<9, 9>(0, 0)).AssertEqual(F.m_Fdd, verbose, str) && scc;
      scc = DB(block<9, 6>(0, 9)).AssertEqual(F.m_Fdb, verbose, str) && scc;
      scc = EigenMatrix6x9f(block<6, 9>(9, 0)).AssertZero(verbose, str) && scc;
      scc = EigenMatrix6x6f(block<6, 6>(9, 9)).AssertEqual(EigenMatrix6x6f::Identity(), verbose, str) && scc;
      return scc;
    }
  };
  class EigenCovariance : public EigenMatrix15x15f {
   public:
    class DD : public EigenMatrix9x9f {
     public:
      inline DD() {}
      inline DD(const Eigen::Matrix<float, 9, 9> &e_P) {
        *((EigenMatrix9x9f *) this) = e_P;
      }
      inline DD(const Covariance::DD &P) { Set(P); }
      inline void Set(const Covariance::DD &P) {
        block<3, 3>(0, 0) = EigenMatrix3x3f(P.m_Prr);
        block<3, 3>(0, 3) = EigenMatrix3x3f(P.m_Prv);
        block<3, 3>(0, 6) = EigenMatrix3x3f(P.m_Prp);
        block<3, 3>(3, 0) = EigenMatrix3x3f(P.m_Pvr);
        block<3, 3>(3, 3) = EigenMatrix3x3f(P.m_Pvv);
        block<3, 3>(3, 6) = EigenMatrix3x3f(P.m_Pvp);
        block<3, 3>(6, 0) = EigenMatrix3x3f(P.m_Ppr);
        block<3, 3>(6, 3) = EigenMatrix3x3f(P.m_Ppv);
        block<3, 3>(6, 6) = EigenMatrix3x3f(P.m_Ppp);
      }
      inline void Get(Covariance::DD *P) const {
        P->m_Prr = EigenMatrix3x3f(block<3, 3>(0, 0)).GetAlignedMatrix3x3f();
        P->m_Prv = EigenMatrix3x3f(block<3, 3>(0, 3)).GetAlignedMatrix3x3f();
        P->m_Prp = EigenMatrix3x3f(block<3, 3>(0, 6)).GetAlignedMatrix3x3f();
        P->m_Pvr = EigenMatrix3x3f(block<3, 3>(3, 0)).GetAlignedMatrix3x3f();
        P->m_Pvv = EigenMatrix3x3f(block<3, 3>(3, 3)).GetAlignedMatrix3x3f();
        P->m_Pvp = EigenMatrix3x3f(block<3, 3>(3, 6)).GetAlignedMatrix3x3f();
        P->m_Ppr = EigenMatrix3x3f(block<3, 3>(6, 0)).GetAlignedMatrix3x3f();
        P->m_Ppv = EigenMatrix3x3f(block<3, 3>(6, 3)).GetAlignedMatrix3x3f();
        P->m_Ppp = EigenMatrix3x3f(block<3, 3>(6, 6)).GetAlignedMatrix3x3f();
      }
      inline bool AssertEqual(const Covariance::DD &P, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertEqual(P.m_Prr, verbose, str + ".Prr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertEqual(P.m_Prv, verbose, str + ".Prv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 6)).AssertEqual(P.m_Prp, verbose, str + ".Prp") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(P.m_Pvr, verbose, str + ".Pvr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(P.m_Pvv, verbose, str + ".Pvv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 6)).AssertEqual(P.m_Pvp, verbose, str + ".Pvp") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 0)).AssertEqual(P.m_Ppr, verbose, str + ".Ppr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 3)).AssertEqual(P.m_Ppv, verbose, str + ".Ppv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 6)).AssertEqual(P.m_Ppp, verbose, str + ".Ppp") && scc;
        return scc;
      }
    };
    class DB : public EigenMatrix9x6f {
     public:
      inline DB() {}
      inline DB(const Eigen::Matrix<float, 9, 6> &e_P) {
        *((EigenMatrix9x6f *) this) = e_P;
      }
      inline DB(const Covariance::DB &P) { Set(P); }
      inline void Set(const Covariance::DB &P) {
        block<3, 3>(0, 0) = EigenMatrix3x3f::Zero();
        block<3, 3>(0, 3) = EigenMatrix3x3f(P.m_Prbw);
        block<3, 3>(3, 0) = EigenMatrix3x3f(P.m_Pvba);
        block<3, 3>(3, 3) = EigenMatrix3x3f(P.m_Pvbw);
        block<3, 3>(6, 0) = EigenMatrix3x3f(P.m_Ppba);
        block<3, 3>(6, 3) = EigenMatrix3x3f(P.m_Ppbw);
      }
      inline void Get(Covariance::DB *P) const {
        P->m_Prbw = EigenMatrix3x3f(block<3, 3>(0, 3)).GetAlignedMatrix3x3f();
        P->m_Pvba = EigenMatrix3x3f(block<3, 3>(3, 0)).GetAlignedMatrix3x3f();
        P->m_Pvbw = EigenMatrix3x3f(block<3, 3>(3, 3)).GetAlignedMatrix3x3f();
        P->m_Ppba = EigenMatrix3x3f(block<3, 3>(6, 0)).GetAlignedMatrix3x3f();
        P->m_Ppbw = EigenMatrix3x3f(block<3, 3>(6, 3)).GetAlignedMatrix3x3f();
      }
      inline bool AssertEqual(const Covariance::DB &P, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertZero(verbose, str + ".Prba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertEqual(P.m_Prbw, verbose, str + ".Prbw") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(P.m_Pvba, verbose, str + ".Pvba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(P.m_Pvbw, verbose, str + ".Pvbw") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 0)).AssertEqual(P.m_Ppba, verbose, str + ".Ppba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(6, 3)).AssertEqual(P.m_Ppbw, verbose, str + ".Ppbw") && scc;
        return scc;
      }
    };
    class BD : public EigenMatrix6x9f {
     public:
      inline BD() {}
      inline BD(const Eigen::Matrix<float, 6, 9> &e_P) {
        *((EigenMatrix6x9f *) this) = e_P;
      }
      inline BD(const Covariance::BD &P) { Set(P); }
      inline void Set(const Covariance::BD &P) {
        block<3, 3>(0, 0) = EigenMatrix3x3f::Zero();
        block<3, 3>(0, 3) = EigenMatrix3x3f(P.m_Pbav);
        block<3, 3>(0, 6) = EigenMatrix3x3f(P.m_Pbap);
        block<3, 3>(3, 0) = EigenMatrix3x3f(P.m_Pbwr);
        block<3, 3>(3, 3) = EigenMatrix3x3f(P.m_Pbwv);
        block<3, 3>(3, 6) = EigenMatrix3x3f(P.m_Pbwp);
      }
      inline void Get(Covariance::BD *P) const {
        P->m_Pbav = EigenMatrix3x3f(block<3, 3>(0, 3)).GetAlignedMatrix3x3f();
        P->m_Pbap = EigenMatrix3x3f(block<3, 3>(0, 6)).GetAlignedMatrix3x3f();
        P->m_Pbwr = EigenMatrix3x3f(block<3, 3>(3, 0)).GetAlignedMatrix3x3f();
        P->m_Pbwv = EigenMatrix3x3f(block<3, 3>(3, 3)).GetAlignedMatrix3x3f();
        P->m_Pbwp = EigenMatrix3x3f(block<3, 3>(3, 6)).GetAlignedMatrix3x3f();
      }
      inline bool AssertEqual(const Covariance::BD &P, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertZero(verbose, str + ".Pbar") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertEqual(P.m_Pbav, verbose, str + ".Pbav") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 6)).AssertEqual(P.m_Pbap, verbose, str + ".Pbap") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertEqual(P.m_Pbwr, verbose, str + ".Pbwr") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(P.m_Pbwv, verbose, str + ".Pbwv") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 6)).AssertEqual(P.m_Pbwp, verbose, str + ".Pbwp") && scc;
        return scc;
      }
    };
    class BB : public EigenMatrix6x6f {
     public:
      inline BB() {}
      inline BB(const Eigen::Matrix<float, 6, 6> &e_P) {
        *((EigenMatrix6x6f *) this) = e_P;
      }
      inline BB(const Covariance::BB &P) { Set(P); }
      inline void Set(const Covariance::BB &P) {
        EigenMatrix6x6f &e_P = *this;
        e_P.setZero();
        e_P(0, 0) = e_P(1, 1) = e_P(2, 2) = P.m_Pbaba;
        e_P(3, 3) = e_P(4, 4) = e_P(5, 5) = P.m_Pbwbw;
      }
      inline void Get(Covariance::BB *P) const {
        const EigenMatrix6x6f &e_P = *this;
        P->m_Pbaba = e_P(0, 0);
        P->m_Pbwbw = e_P(3, 3);
      }
      inline bool AssertEqual(const Covariance::BB &P, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = EigenMatrix3x3f(block<3, 3>(0, 0)).AssertEqual(EigenMatrix3x3f(P.m_Pbaba),
                                                             verbose, str + ".Pbaba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(0, 3)).AssertZero(verbose, str + ".Pbabw") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 0)).AssertZero(verbose, str + ".Pbwba") && scc;
        scc = EigenMatrix3x3f(block<3, 3>(3, 3)).AssertEqual(EigenMatrix3x3f(P.m_Pbwbw),
                                                             verbose, str + ".Pbwbw") && scc;
        return scc;
      }
    };
   public:
    inline EigenCovariance() {}
    inline EigenCovariance(const Eigen::Matrix<float, 15, 15, Eigen::RowMajor> &e_P) {
      *((EigenMatrix15x15f *) this) = e_P;
    }
    inline void Set(const Covariance &P) {
      block<9, 9>(0, 0) = DD(P.m_Pdd);
      block<9, 6>(0, 9) = DB(P.m_Pdb);
      //block<6, 9>(9, 0) = BD(P.m_Pbd);
      block<6, 9>(9, 0) = block<9, 6>(0, 9).transpose();
      block<6, 6>(9, 9) = BB(P.m_Pbb);
    }
    inline void Get(Covariance *P) const {
      DD(block<9, 9>(0, 0)).Get(&P->m_Pdd);
      DB(block<9, 6>(0, 9)).Get(&P->m_Pdb);
      BB(block<6, 6>(9, 9)).Get(&P->m_Pbb);
    }
    inline bool AssertEqual(const Covariance &P, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = DD(block<9, 9>(0, 0)).AssertEqual(P.m_Pdd, verbose, str) && scc;
      scc = DB(block<9, 6>(0, 9)).AssertEqual(P.m_Pdb, verbose, str) && scc;
      //scc = BD(block<6, 9>(9, 0)).AssertEqual(P.m_Pbd, verbose, str) && scc;
      scc = BB(block<6, 6>(9, 9)).AssertEqual(P.m_Pbb, verbose, str) && scc;
      return scc;
    }
  };
  class EigenWeight : public EigenMatrix15x15f {
   public:
    inline EigenWeight() {}
    inline EigenWeight(const Eigen::Matrix<float, 15, 15, Eigen::RowMajor> &e_W) { *this = e_W; }
    inline EigenWeight(const Weight &W) { Set(W); }
    inline EigenWeight(const float w, const Weight &W) { Set(w, W); }
    inline void operator = (const Eigen::Matrix<float, 15, 15, Eigen::RowMajor> &e_W) {
      *((Eigen::Matrix<float, 15, 15, Eigen::RowMajor> *) this) = e_W;
    }
    inline void Set(const EigenCovariance &e_P) {
      *this = e_P.ldlt().solve(EigenMatrix15x15f::Identity());
    }
    inline void Set(const Weight &W) {
      setZero();
#ifdef CFG_IMU_FULL_COVARIANCE
      for (int i = 0, _i = 0; i < 5; ++i, _i += 3) {
        const LA::AlignedMatrix3x3f *Wi = W[i];
        for (int j = 0, _j = 0; j < 5; ++j, _j += 3) {
          block<3, 3>(_i, _j) = EigenMatrix3x3f(Wi[j]);
        }
      }
#else
      EigenMatrix15x15f &e_W = *this;
      e_W.block<3, 3>(0, 0) = EigenMatrix3x3f(W.m_Wr);
      e_W.block<3, 3>(3, 3) = EigenMatrix3x3f(W.m_Wv);
      e_W.block<3, 3>(6, 6) = EigenMatrix3x3f(W.m_Wp);
      e_W(9, 9) = e_W(10, 10) = e_W(11, 11) = W.m_wba;
      e_W(12, 12) = e_W(13, 13) = e_W(14, 14) = W.m_wbw;
#endif
    }
    inline void Set(const float w, const Weight &W) { Set(W); *this *= w; }
    inline bool AssertEqual(const Weight &W, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      LA::AlignedMatrix3x3f Wij;
      for (int i = 0, _i = 0; i < 5; ++i, _i += 3) {
        for (int j = 0, _j = 0; j < 5; ++j, _j += 3) {
#ifdef CFG_IMU_FULL_COVARIANCE
          Wij = W[i][j];
#else
          if (i == j) {
            switch (i) {
            case 0: Wij = W.m_Wr; break;
            case 1: Wij = W.m_Wv; break;
            case 2: Wij = W.m_Wp; break;
            case 3: Wij.SetDiagonal(W.m_wba); break;
            case 4: Wij.SetDiagonal(W.m_wbw); break;
            }
          } else {
            Wij.MakeZero();
          }
#endif
          scc = EigenMatrix3x3f(block<3, 3>(_i, _j)).AssertEqual(Wij, verbose,
            str + UT::String(".W[%d][%d]", i, j)) && scc;
        }
      }
      return scc;
    }
  };
  class EigenError {
   public:
    inline EigenError() {}
    inline EigenError(const Error &e) { Set(e); }
    inline void Set(const Error &e) {
      m_er = EigenVector3f(e.m_er);
      m_ev = EigenVector3f(e.m_ev);
      m_ep = EigenVector3f(e.m_ep);
      m_eba = EigenVector3f(e.m_eba);
      m_ebw = EigenVector3f(e.m_ebw);
    }
    inline void Get(EigenVector15f *e_e) const {
      e_e->block<3, 1>(0, 0) = m_er;
      e_e->block<3, 1>(3, 0) = m_ev;
      e_e->block<3, 1>(6, 0) = m_ep;
      e_e->block<3, 1>(9, 0) = m_eba;
      e_e->block<3, 1>(12, 0) = m_ebw;
    }
    inline bool AssertEqual(const Error &e, const int verbose = 1,
                            const std::string str = "",
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      EigenError e_e;
      e_e.Set(e);
      return AssertEqual(e_e, verbose, str, epsAbs, epsRel);
    }
    inline bool AssertEqual(const EigenError &e_e, const int verbose = 1,
                            const std::string str = "",
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      bool scc = true;
      scc = m_er.AssertEqual(e_e.m_er, verbose, str + ".m_er", epsAbs, epsRel) && scc;
      scc = m_ev.AssertEqual(e_e.m_ev, verbose, str + ".m_ev", epsAbs, epsRel) && scc;
      scc = m_ep.AssertEqual(e_e.m_ep, verbose, str + ".m_ep", epsAbs, epsRel) && scc;
      scc = m_eba.AssertEqual(e_e.m_eba, verbose, str + ".m_eba", epsAbs, epsRel) && scc;
      scc = m_ebw.AssertEqual(e_e.m_ebw, verbose, str + ".m_ebw", epsAbs, epsRel) && scc;
      return scc;
    }
   public:
    EigenVector3f m_er, m_ev, m_ep, m_eba, m_ebw;
  };
  class EigenJacobian {
   public:
    class Gravity {
     public:
      inline void Set(const Jacobian::Gravity &J) {
        m_JvgT = J.m_JvgT;
        m_JpgT = J.m_JpgT;
      }
      inline void Get(EigenMatrix15x2f *e_J) const {
        e_J->setZero();
        e_J->block<3, 2>(3, 0) = m_JvgT.transpose();
        e_J->block<3, 2>(6, 0) = m_JpgT.transpose();
      }
      inline bool AssertEqual(const Jacobian::Gravity &J, const int verbose = 1,
                              const std::string str = "") const {
        EigenJacobian::Gravity e_J;
        e_J.Set(J);
        return AssertEqual(e_J, verbose, str);
      }
      inline bool AssertEqual(const EigenJacobian::Gravity &e_J, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = m_JvgT.AssertEqual(e_J.m_JvgT, verbose, str + ".m_JvgT") && scc;
        scc = m_JpgT.AssertEqual(e_J.m_JpgT, verbose, str + ".m_JpgT") && scc;
        return scc;
      }
     public:
      EigenMatrix2x3f m_JvgT, m_JpgT;
    };
    class Global {
     public:
      inline void Set(const Jacobian::FirstMotion &J) {
        m_Jr.setZero();
        m_Jr.block<3, 3>(0, 12) = EigenMatrix3x3f(J.m_Jrbw1);
        m_Jv.setZero();
        m_Jv.block<3, 3>(0, 6) = EigenMatrix3x3f(J.m_Jvv1);
        m_Jv.block<3, 3>(0, 9) = EigenMatrix3x3f(J.m_Jvba1);
        m_Jv.block<3, 3>(0, 12) = EigenMatrix3x3f(J.m_Jvbw1);
        m_Jp.setZero();
        m_Jp.block<3, 3>(0, 6) = EigenMatrix3x3f(J.m_Jpv1);
        m_Jp.block<3, 3>(0, 9) = EigenMatrix3x3f(J.m_Jpba1);
        m_Jp.block<3, 3>(0, 12) = EigenMatrix3x3f(J.m_Jpbw1);
        m_Jba.setZero();
        m_Jba.block<3, 3>(0, 9) = EigenMatrix3x3f::Identity();
        m_Jbw.setZero();
        m_Jbw.block<3, 3>(0, 12) = EigenMatrix3x3f::Identity();
      }
      inline void Set(const Jacobian::Global &J) {
        Set(Jacobian::FirstMotion(J));
        m_Jr.block<3, 3>(0, 3) = EigenMatrix3x3f(J.m_Jrr1);
        m_Jr.block<3, 3>(0, 18) = -EigenMatrix3x3f(J.m_Jrr1);
        m_Jv.block<3, 3>(0, 3) = EigenMatrix3x3f(J.m_Jvr1);
        m_Jv.block<3, 3>(0, 21) = EigenMatrix3x3f(J.m_Jvv1.GetMinus());
        m_Jp.block<3, 3>(0, 0) = EigenMatrix3x3f(J.m_Jpp1);
        m_Jp.block<3, 3>(0, 3) = EigenMatrix3x3f(J.m_Jpr1);
        m_Jp.block<3, 3>(0, 15) = EigenMatrix3x3f(J.m_Jpp1.GetMinus());
        m_Jp.block<3, 3>(0, 18) = EigenMatrix3x3f(J.m_Jpr2);
        m_Jba.block<3, 3>(0, 24) = -EigenMatrix3x3f::Identity();
        m_Jbw.block<3, 3>(0, 27) = -EigenMatrix3x3f::Identity();
      }
      inline void Get(EigenMatrix15x30f *e_J) const {
        e_J->block<3, 30>(0, 0) = m_Jr;
        e_J->block<3, 30>(3, 0) = m_Jv;
        e_J->block<3, 30>(6, 0) = m_Jp;
        e_J->block<3, 30>(9, 0) = m_Jba;
        e_J->block<3, 30>(12, 0) = m_Jbw;
      }
      inline bool AssertEqual(const Jacobian::Global &J, const int verbose = 1,
                              const std::string str = "") const {
        EigenJacobian::Global e_J;
        e_J.Set(J);
        return AssertEqual(e_J, verbose, str);
      }
      inline bool AssertEqual(const EigenJacobian::Global &e_J, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = m_Jr.AssertEqual(e_J.m_Jr, verbose, str + ".m_Jr") && scc;
        scc = m_Jv.AssertEqual(e_J.m_Jv, verbose, str + ".m_Jv") && scc;
        scc = m_Jp.AssertEqual(e_J.m_Jp, verbose, str + ".m_Jp") && scc;
        scc = m_Jba.AssertEqual(e_J.m_Jba, verbose, str + ".m_Jba") && scc;
        scc = m_Jbw.AssertEqual(e_J.m_Jbw, verbose, str + ".m_Jbw") && scc;
        return scc;
      }
     public:
      EigenMatrix3x30f m_Jr, m_Jv, m_Jp, m_Jba, m_Jbw;
    };
    class RelativeLF : public Gravity, public Global {
     public:
      inline RelativeLF() {}
      inline RelativeLF(const Jacobian::RelativeLF &J, const float Tpv) { Set(J, Tpv); }
      inline void Set(const Jacobian::RelativeLF &J, const float Tpv) {
        Gravity::Set(J);
        Global::Set(Jacobian::Global(J));
        m_Jv.block<3, 3>(0, 6) = EigenMatrix3x3f(-EigenMatrix3x3f::Identity());
        m_Jv.block<3, 3>(0, 18) = EigenMatrix3x3f(J.m_Jvr2);
        m_Jv.block<3, 3>(0, 21) = EigenMatrix3x3f(J.m_Jvv2);
        m_Jp.block<3, 3>(0, 6) = EigenMatrix3x3f(-EigenMatrix3x3f::Identity() * Tpv);
      }
      inline void Get(EigenMatrix15x2f *e_Jg, EigenMatrix15x30f *e_Jc) const {
        Gravity::Get(e_Jg);
        Global::Get(e_Jc);
      }
      inline bool AssertEqual(const Jacobian::RelativeLF &J, const float Tpv,
                              const int verbose = 1, const std::string str = "") const {
        EigenJacobian::RelativeLF e_J;
        e_J.Set(J, Tpv);
        return AssertEqual(e_J, verbose, str);
      }
      inline bool AssertEqual(const EigenJacobian::RelativeLF &e_J, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = Gravity::AssertEqual(e_J, verbose, str) && scc;
        scc = Global::AssertEqual(e_J, verbose, str) && scc;
        return scc;
      }
    };
    class RelativeKF : public RelativeLF {
     public:
      inline RelativeKF() {}
      inline RelativeKF(const Jacobian::RelativeKF &J, const float Tpv) { Set(J, Tpv); }
      inline void Set(const Jacobian::RelativeKF &J, const float Tpv) {
        Gravity::Set(Jacobian::Gravity(J));
        Global::Set(Jacobian::FirstMotion(J));
        m_Jr.block<3, 3>(0, 18) = EigenMatrix3x3f(J.m_Jrr2);
        m_Jv.block<3, 3>(0, 6) = EigenMatrix3x3f(-EigenMatrix3x3f::Identity());
        m_Jv.block<3, 3>(0, 18) = EigenMatrix3x3f(J.m_Jvr2);
        m_Jv.block<3, 3>(0, 21) = EigenMatrix3x3f(J.m_Jvv2);
        m_Jp.block<3, 3>(0, 6) = EigenMatrix3x3f(-EigenMatrix3x3f::Identity() * Tpv);
        m_Jp.block<3, 3>(0, 15) = EigenMatrix3x3f::Identity();
        m_Jp.block<3, 3>(0, 18) = EigenMatrix3x3f(J.m_Jpr2);
        m_Jba.block<3, 3>(0, 24) = -EigenMatrix3x3f::Identity();
        m_Jbw.block<3, 3>(0, 27) = -EigenMatrix3x3f::Identity();
      }
      inline bool AssertEqual(const Jacobian::RelativeKF &J, const float Tpv,
                              const int verbose = 1, const std::string str = "") const {
        EigenJacobian::RelativeKF e_J;
        e_J.Set(J, Tpv);
        return RelativeLF::AssertEqual(e_J, verbose, str);
      }
    };
  };
  class EigenErrorJacobian {
   public:
    inline void Set(const ErrorJacobian &Je) {
      m_e.Set(Je.m_e);
      m_J.Set(Je.m_J);
    }
    inline bool AssertEqual(const ErrorJacobian &Je, const int verbose = 1,
                            const std::string str = "") const {
      bool scc = true;
      scc = m_e.AssertEqual(Je.m_e, verbose, str) && scc;
      scc = m_J.AssertEqual(Je.m_J, verbose, str) && scc;
      return scc;
    }
   public:
    EigenError m_e;
    EigenJacobian::Global m_J;
  };
  class EigenFactor {
   public:
    class Global {
     public:
      inline void operator *= (const float w) {
        m_Ac1c1 *= w; m_Ac1m1 *= w; m_Ac1c2 *= w; m_Ac1m2 *= w; m_bc1 *= w;
        m_Am1m1 *= w; m_Am1c2 *= w; m_Am1m2 *= w; m_bm1 *= w;
        m_Ac2c2 *= w; m_Ac2m2 *= w; m_bc2 *= w;
        m_Am2m2 *= w; m_bm2 *= w;
      }
      inline void Set(const EigenMatrix30x31f &A, const float F) {
        Set(EigenMatrix30x30f(A.block<30, 30>(0, 0)), EigenVector30f(A.block<30, 1>(0, 30)), F);
      }
      inline void Set(const EigenMatrix30x30f &A, const EigenVector30f &b, const float F) {
        m_Ac1c1 = A.block<6, 6>(0, 0);
        m_Ac1m1 = A.block<6, 9>(0, 6);
        m_Ac1c2 = A.block<6, 6>(0, 15);
        m_Ac1m2 = A.block<6, 9>(0, 21);
        m_bc1 = b.block<6, 1>(0, 0);
        m_Am1m1 = A.block<9, 9>(6, 6);
        m_Am1c2 = A.block<9, 6>(6, 15);
        m_Am1m2 = A.block<9, 9>(6, 21);
        m_bm1 = b.block<9, 1>(6, 0);
        m_Ac2c2 = A.block<6, 6>(15, 15);
        m_Ac2m2 = A.block<6, 9>(15, 21);
        m_bc2 = b.block<6, 1>(15, 0);
        m_Am2m2 = A.block<9, 9>(21, 21);
        m_bm2 = b.block<9, 1>(21, 0);
        m_F = F;
      }
      inline void Set(const Factor::Unitary &A11, const Factor::Unitary &A22,
                      const Camera::Factor::Binary &A12, const float F) {
        m_Ac1c1 = A11.m_Acc.m_A;
        m_Ac1m1 = A11.m_Acm;
        m_Ac1c2 = A12.m_Acc;
#ifdef CFG_IMU_FULL_COVARIANCE
        m_Ac1m2 = A12.m_Acm;
#else
        m_Ac1m2.setZero();
        m_Ac1m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A12.m_Acm.m_Arv);
#endif
        m_bc1 = A11.m_Acc.m_b;

        m_Am1m1 = A11.m_Amm.m_A;
        m_Am1c2 = A12.m_Amc;
#ifdef CFG_IMU_FULL_COVARIANCE
        m_Am1m2 = A12.m_Amm;
#else
        m_Am1m2.setZero();
        m_Am1m2.block<9, 3>(0, 0) = EigenMatrix9x3f(A12.m_Amm.m_Amv);
        m_Am1m2.block<3, 3>(3, 3) = EigenMatrix3x3f(A12.m_Amm.m_Ababa);
        m_Am1m2.block<3, 3>(6, 6) = EigenMatrix3x3f(A12.m_Amm.m_Abwbw);
#endif
        m_bm1 = A11.m_Amm.m_b;

        m_Ac2c2 = A22.m_Acc.m_A;
        m_Ac2m2 = A22.m_Acm;
        m_bc2 = A22.m_Acc.m_b;

        m_Am2m2 = A22.m_Amm.m_A;
        m_bm2 = A22.m_Amm.m_b;

        m_F = F;
      }
      inline void Set(const Factor &A, const Camera::Factor::Binary &A12) {
        Set(A.m_A11, A.m_A22, A12, A.m_F);
      }
      inline void Get(EigenMatrix30x30f &e_A, EigenVector30f &e_b) const {
        e_A.block<6, 6>(0, 0) = m_Ac1c1;
        e_A.block<6, 9>(0, 6) = m_Ac1m1;
        e_A.block<6, 6>(0, 15) = m_Ac1c2;
        e_A.block<6, 9>(0, 21) = m_Ac1m2;
        e_b.block<6, 1>(0, 0) = m_bc1;
        e_A.block<9, 9>(6, 6) = m_Am1m1;
        e_A.block<9, 6>(6, 15) = m_Am1c2;
        e_A.block<9, 9>(6, 21) = m_Am1m2;
        e_b.block<9, 1>(6, 0) = m_bm1;
        e_A.block<6, 6>(15, 15) = m_Ac2c2;
        e_A.block<6, 9>(15, 21) = m_Ac2m2;
        e_b.block<6, 1>(15, 0) = m_bc2;
        e_A.block<9, 9>(21, 21) = m_Am2m2;
        e_b.block<9, 1>(21, 0) = m_bm2;
        e_A.SetLowerFromUpper();
      }
      inline bool AssertEqual(const Factor &A, const Camera::Factor::Binary &A12,
                              const int verbose = 1, const std::string str = "") const {
        Global e_A;
        e_A.Set(A, A12);
        return AssertEqual(e_A, verbose, str);
      }
      inline bool AssertEqual(const Factor::Unitary &A11, const Factor::Unitary &A22,
                              const Camera::Factor::Binary &A12, const float F,
                              const int verbose = 1, const std::string str = "") const {
        Global e_A;
        e_A.Set(A11, A22, A12, F);
        return AssertEqual(e_A, verbose, str);
      }
      inline bool AssertEqual(const Global &A, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = m_Ac1c1.AssertEqual(A.m_Ac1c1, verbose, str + ".m_Ac1c1") && scc;
        scc = m_Ac1m1.AssertEqual(A.m_Ac1m1, verbose, str + ".m_Ac1m1") && scc;
        scc = m_Ac1c2.AssertEqual(A.m_Ac1c2, verbose, str + ".m_Ac1c2") && scc;
        scc = m_Ac1m2.AssertEqual(A.m_Ac1m2, verbose, str + ".m_Ac1m2") && scc;
        scc = m_bc1.AssertEqual(A.m_bc1, verbose, str + ".m_bc1") && scc;

        scc = m_Am1m1.AssertEqual(A.m_Am1m1, verbose, str + ".m_Am1m1") && scc;
        scc = m_Am1c2.AssertEqual(A.m_Am1c2, verbose, str + ".m_Am1c2") && scc;
        scc = m_Am1m2.AssertEqual(A.m_Am1m2, verbose, str + ".m_Am1m2") && scc;
        scc = m_bm1.AssertEqual(A.m_bm1, verbose, str + ".m_bm1") && scc;

        scc = m_Ac2c2.AssertEqual(A.m_Ac2c2, verbose, str + ".m_Ac2c2") && scc;
        scc = m_Ac2m2.AssertEqual(A.m_Ac2m2, verbose, str + ".m_Ac2m2") && scc;
        scc = m_bc2.AssertEqual(A.m_bc2, verbose, str + ".m_bc2") && scc;

        scc = m_Am2m2.AssertEqual(A.m_Am2m2, verbose, str + ".m_Am2m2") && scc;
        scc = m_bm2.AssertEqual(A.m_bm2, verbose, str + ".m_bm2") && scc;

        scc = UT::AssertEqual(m_F, A.m_F, verbose, str + ".m_F") && scc;
        return scc;
      }
     public:
      EigenMatrix6x6f m_Ac1c1;
      EigenMatrix6x9f m_Ac1m1;
      EigenMatrix6x6f m_Ac1c2;
      EigenMatrix6x9f m_Ac1m2;
      EigenVector6f m_bc1;
      EigenMatrix9x9f m_Am1m1;
      EigenMatrix9x6f m_Am1c2;
      EigenMatrix9x9f m_Am1m2;
      EigenVector9f m_bm1;
      EigenMatrix6x6f m_Ac2c2;
      EigenMatrix6x9f m_Ac2m2;
      EigenVector6f m_bc2;
      EigenMatrix9x9f m_Am2m2;
      EigenVector9f m_bm2;
      float m_F;
    };
    class RelativeLF : public Global {
     public:
      inline void Set(const Factor::Auxiliary::RelativeLF &A, const float F) {
#ifdef CFG_IMU_FULL_COVARIANCE
        Factor::Unitary A11, A22;
        Camera::Factor::Binary A12;
        A.Get(&A11, &A22, &A12);
        Global::Set(A11, A22, A12, F);
        m_Agg = EigenMatrix2x2f(A.m_Agg);
        m_Agc1 = EigenMatrix2x6f(A.m_Agc[0], A.m_Agc[1]);
        m_Agm1 = EigenMatrix2x9f(A.m_Agc[2], A.m_Agc[3], A.m_Agc[4]);
        m_Agc2 = EigenMatrix2x6f(A.m_Agc[5], A.m_Agc[6]);
        m_Agm2 = EigenMatrix2x9f(A.m_Agc[7], A.m_Agc[8], A.m_Agc[9]);
        m_bg = EigenVector2f(A.m_bg);
#else
        m_Ac1c1 = EigenMatrix6x6f(A.m_Ap1p1, A.m_Ap1r1, A.m_Ar1r1);
        m_Ac1m1 = EigenMatrix6x9f(A.m_Ap1v1, A.m_Ap1ba1, A.m_Ap1bw1,
                                  A.m_Ar1v1, A.m_Ar1ba1, A.m_Ar1bw1);
        m_Ac1c2 = EigenMatrix6x6f(A.m_Ap1p2, A.m_Ap1r2, A.m_Ar1p2, A.m_Ar1r2);
        m_Ac1m2.setZero();
        m_Ac1m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A.m_Ar1v2);
        m_bc1 = EigenVector6f(A.m_bp1, A.m_br1);
        m_Am1m1 = EigenMatrix9x9f(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1,
                                  A.m_Abw1bw1);
        m_Am1c2 = EigenMatrix9x6f(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2,
                                  A.m_Abw1r2);
        m_Am1m2.setZero();
        m_Am1m2.block<3, 3>(0, 0) = EigenMatrix3x3f(A.m_Av1v2);
        m_Am1m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A.m_Aba1v2);
        m_Am1m2.block<3, 3>(3, 3) = EigenMatrix3x3f(A.m_Aba1ba2);
        m_Am1m2.block<3, 3>(6, 0) = EigenMatrix3x3f(A.m_Abw1v2);
        m_Am1m2.block<3, 3>(6, 6) = EigenMatrix3x3f(A.m_Abw1bw2);
        m_bm1 = EigenVector9f(A.m_bv1, A.m_bba1, A.m_bbw1);
        m_Ac2c2 = EigenMatrix6x6f(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
        m_Ac2m2.setZero();
        m_Ac2m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A.m_Ar2v2);
        m_bc2 = EigenVector6f(A.m_bp2, A.m_br2);
        m_Am2m2.setZero();
        m_Am2m2.block<3, 3>(0, 0) = EigenMatrix3x3f(A.m_Av2v2);
        m_Am2m2.block<3, 3>(3, 3) = EigenMatrix3x3f(A.m_Aba2ba2);
        m_Am2m2.block<3, 3>(6, 6) = EigenMatrix3x3f(A.m_Abw2bw2);
        m_bm2 = EigenVector9f(A.m_bv2, A.m_bba2, A.m_bbw2);
        m_F = F;
        m_Agg = EigenMatrix2x2f(A.m_Agg);
        m_Agc1 = EigenMatrix2x6f(A.m_Agp1, A.m_Agr1);
        m_Agm1 = EigenMatrix2x9f(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
        m_Agc2 = EigenMatrix2x6f(A.m_Agp2, A.m_Agr2);
        m_Agm2.setZero();
        m_Agm2.block<2, 3>(0, 0) = EigenMatrix2x3f(A.m_Agv2);
        m_bg = EigenVector2f(A.m_bg);
#endif
      }
      inline void Get(EigenMatrix2x2f &e_Agg, EigenMatrix2x30f &e_Agc, EigenMatrix30x30f &e_Acc,
                      EigenVector2f &e_bg, EigenVector30f &e_bc) const {
        Global::Get(e_Acc, e_bc);
        e_Agg = m_Agg;
        e_Agc.block<2, 6>(0, 0) = m_Agc1;
        e_Agc.block<2, 9>(0, 6) = m_Agm1;
        e_Agc.block<2, 6>(0, 15) = m_Agc2;
        e_Agc.block<2, 9>(0, 21) = m_Agm2;
        e_bg = m_bg;
      }
      inline bool AssertEqual(const Factor::Auxiliary::RelativeLF &A, const int verbose = 1,
                              const std::string str = "") const {
        RelativeLF e_A;
        e_A.Set(A, m_F);
        return AssertEqual(e_A, verbose, str);
      }
      inline bool AssertEqual(const RelativeLF &e_A, const int verbose = 1,
                              const std::string str = "") const {
        bool scc = true;
        scc = Global::AssertEqual(e_A, verbose, str) && scc;
        scc = m_Agg.AssertEqual(e_A.m_Agg, verbose, str + ".m_Agg") && scc;
        scc = m_Agc1.AssertEqual(e_A.m_Agc1, verbose, str + ".m_Agc1") && scc;
        scc = m_Agm1.AssertEqual(e_A.m_Agm1, verbose, str + ".m_Agm1") && scc;
        scc = m_Agc2.AssertEqual(e_A.m_Agc2, verbose, str + ".m_Agc2") && scc;
        scc = m_Agm2.AssertEqual(e_A.m_Agm2, verbose, str + ".m_Agm2") && scc;
        scc = m_bg.AssertEqual(e_A.m_bg, verbose, str + ".m_bg") && scc;
        return scc;
      }
     public:
      EigenMatrix2x2f m_Agg;
      EigenMatrix2x6f m_Agc1;
      EigenMatrix2x9f m_Agm1;
      EigenMatrix2x6f m_Agc2;
      EigenMatrix2x9f m_Agm2;
      EigenVector2f m_bg;
    };
    class RelativeKF : public RelativeLF {
     public:
      inline void Set(const Factor::Auxiliary::RelativeKF &A, const float F) {
        m_Ac1c1.setZero();
        m_Ac1m1.setZero();
        m_Ac1c2.setZero();
        m_Ac1m2.setZero();
        m_bc1.setZero();
#ifdef CFG_IMU_FULL_COVARIANCE
        Factor::Unitary A11, A22;
        Camera::Factor::Binary A12;
        A.Get(&A11, &A22, &A12);
        Global::Set(A11, A22, A12, F);
        m_Agg = EigenMatrix2x2f(A.m_Agg);
        m_Agc1.setZero();
        m_Agm1 = EigenMatrix2x9f(A.m_Agc[0], A.m_Agc[1], A.m_Agc[2]);
        m_Agc2 = EigenMatrix2x6f(A.m_Agc[3], A.m_Agc[4]);
        m_Agm2 = EigenMatrix2x9f(A.m_Agc[5], A.m_Agc[6], A.m_Agc[7]);
        m_bg = EigenVector2f(A.m_bg);
#else
        m_Am1m1 = EigenMatrix9x9f(A.m_Av1v1, A.m_Av1ba1, A.m_Av1bw1, A.m_Aba1ba1, A.m_Aba1bw1,
                                  A.m_Abw1bw1);
        m_Am1c2 = EigenMatrix9x6f(A.m_Av1p2, A.m_Av1r2, A.m_Aba1p2, A.m_Aba1r2, A.m_Abw1p2,
                                  A.m_Abw1r2);
        m_Am1m2.setZero();
        m_Am1m2.block<3, 3>(0, 0) = EigenMatrix3x3f(A.m_Av1v2);
        m_Am1m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A.m_Aba1v2);
        m_Am1m2.block<3, 3>(3, 3) = EigenMatrix3x3f(A.m_Aba1ba2);
        m_Am1m2.block<3, 3>(6, 0) = EigenMatrix3x3f(A.m_Abw1v2);
        m_Am1m2.block<3, 3>(6, 6) = EigenMatrix3x3f(A.m_Abw1bw2);
        m_bm1 = EigenVector9f(A.m_bv1, A.m_bba1, A.m_bbw1);
        m_Ac2c2 = EigenMatrix6x6f(A.m_Ap2p2, A.m_Ap2r2, A.m_Ar2r2);
        m_Ac2m2.setZero();
        m_Ac2m2.block<3, 3>(3, 0) = EigenMatrix3x3f(A.m_Ar2v2);
        m_bc2 = EigenVector6f(A.m_bp2, A.m_br2);
        m_Am2m2.setZero();
        m_Am2m2.block<3, 3>(0, 0) = EigenMatrix3x3f(A.m_Av2v2);
        m_Am2m2.block<3, 3>(3, 3) = EigenMatrix3x3f(A.m_Aba2ba2);
        m_Am2m2.block<3, 3>(6, 6) = EigenMatrix3x3f(A.m_Abw2bw2);
        m_bm2 = EigenVector9f(A.m_bv2, A.m_bba2, A.m_bbw2);
        m_F = F;
        m_Agg = EigenMatrix2x2f(A.m_Agg);
        m_Agc1.setZero();
        m_Agm1 = EigenMatrix2x9f(A.m_Agv1, A.m_Agba1, A.m_Agbw1);
        m_Agc2 = EigenMatrix2x6f(A.m_Agp2, A.m_Agr2);
        m_Agm2.setZero();
        m_Agm2.block<2, 3>(0, 0) = EigenMatrix2x3f(A.m_Agv2);
        m_bg = EigenVector2f(A.m_bg);
#endif
      }
      inline bool AssertEqual(const Factor::Auxiliary::RelativeKF &A, const int verbose = 1,
                              const std::string str = "") const {
        RelativeKF e_A;
        e_A.Set(A, m_F);
        return RelativeLF::AssertEqual(e_A, verbose, str);
      }
    };
  };
  void EigenGetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                             EigenError *e_e, EigenJacobian::Global *e_J, const float eps) const;
  void EigenGetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                             const Rotation3D &Rg, EigenError *e_e, EigenJacobian::RelativeLF *e_J,
                             const float eps) const;
  void EigenGetErrorJacobian(const Camera &C1, const Camera &C2, const Point3D &pu,
                             EigenError *e_e, EigenJacobian::RelativeKF *e_J,
                             const float eps) const;
  void EigenGetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                      EigenFactor::Global *e_A, const float eps) const;
  void EigenGetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                      const Rotation3D &Rg, EigenFactor::RelativeLF *e_A, const float eps) const;
  void EigenGetFactor(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                      EigenFactor::RelativeKF *e_A, const float eps) const;
  static void EigenGetFactor(const float w, const Weight &W, const EigenJacobian::Global &e_J,
                             const EigenError &e_e, EigenFactor::Global *e_A);
  static void EigenGetFactor(const float w, const Weight &W, const EigenJacobian::RelativeLF &e_J,
                             const EigenError &e_e, EigenFactor::RelativeLF *e_A);
  static EigenError EigenGetError(const EigenErrorJacobian &e_Je, const EigenVector30f e_x);
  float EigenGetCost(const float w, const Camera &C1, const Camera &C2, const Point3D &pu,
                     const EigenVector6f &e_xc1, const EigenVector9f &e_xm1,
                     const EigenVector6f &e_xc2, const EigenVector9f &e_xm2,
                     const float eps) const;
#endif
};

void InitializeCamera(const AlignedVector<Measurement> &us, Camera &C);
void PreIntegrate(const AlignedVector<Measurement> &us, const float t1, const float t2,
                  const Camera &C1, Delta *D, AlignedVector<float> *work, const bool jac/* = true*/,
                  const Measurement *u1/* = NULL*/, const Measurement *u2/* = NULL*/,
                  const float eps);
void PreIntegrate(const Measurement *us, const int N, const float t1, const float t2,
                  const Camera &C1, Delta *D, AlignedVector<float> *work, const bool jac/* = true*/,
                  const Measurement *u1/* = NULL*/, const Measurement *u2/* = NULL*/,
                  const float eps);
void Propagate(const Point3D &pu, const Delta &D, const Camera &C1, Camera &C2,
               const float eps);

};

#endif
