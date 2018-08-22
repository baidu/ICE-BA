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
#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "Intrinsic.h"
#include "Rigid.h"
#include "M-Estimator.h"
#include "MatrixMxN.h"

class Camera {
 public:
  class Calibration {
   public:
    Intrinsic m_K;
    Rotation3D m_Ru;
    Point3D m_pu;
    LA::AlignedVector3f m_ba, m_bw/*, m_sa*/;
#ifdef CFG_STEREO
    Intrinsic m_Kr;
    Rotation3D m_Rr;
    Point3D m_br;
#endif
  };
  
  class Pose {
   public:
    inline void operator = (const Rigid3D &T) { T.Get(m_q, m_p); }
    inline void MakeIdentity(const LA::AlignedVector3f *g = NULL) {
      if (g) {
        Rotation3D R;
        R.MakeIdentity(g);
        R.GetQuaternion(m_q);
      }
      else
        m_q.MakeIdentity();
      m_p.MakeZero();
    }
    inline bool Invalid() const { return m_q.Invalid() || m_p.Invalid(); }
    inline void Invalidate() { m_q.Invalidate(); m_p.Invalidate(); }
    inline void Interpolate(const float w1, const Pose &c1, const Pose &c2) {
      m_q.Slerp(w1, c1.m_q, c2.m_q);
      m_p.Interpolate(w1, c1.m_p, c2.m_p);
    }
    inline void DividedBy (const Pose &c, const Rotation3D &R) {
      Quaternion q;
      Quaternion::ABI(m_q, c.m_q, q);
      m_q = q;
      const LA::AlignedVector3f dp = m_p - c.m_p;
      R.Apply(dp, m_p);
    }
   public:
    Quaternion m_q;
    Point3D m_p;
  };

  class Factor {
   public:
    class Unitary {
     public:
      class CC {
       public:
        static inline CC Get(const LA::SymmetricMatrix6x6f &A, const LA::Vector6f &b) {
          CC _A;
          _A.Set(A, b);
          return _A;
        }
        inline void Set(const LA::SymmetricMatrix6x6f &A, const LA::Vector6f &b) {
          m_A = A;
          m_b = b;
        }
        inline void Set(const LA::AlignedMatrix6x6f &A, const LA::AlignedVector6f &b) {
          m_A.Set(A);
          m_b.Set(b);
        }
        inline void Set(const LA::AlignedMatrix3x3f &App, const LA::AlignedMatrix3x3f &Apr,
                        const LA::AlignedMatrix3x3f &Arr, const LA::AlignedVector3f &bp,
                        const LA::AlignedVector3f &br) {
          m_A.Set(App, Apr, Arr);
          m_b.Set(bp, br);
        }
        inline void Set(const LA::AlignedMatrix3x3f *Ap, const LA::AlignedMatrix3x3f *Ar,
                        const LA::AlignedVector3f *b) {
          m_A.Set(Ap[0], Ap[1], Ar[1]);
          m_b.Set(b[0], b[1]);
        }
        inline void Increase3(const LA::SymmetricMatrix3x3f &A, const LA::Vector3f &b) {
          m_A.Increase33(A);
          m_b.Increase345(b);
        }
        inline void operator += (const CC &A) {
          for (int i = 0; i < 7; ++i) {
            m_data[i] += A.m_data[i];
          }
        }
        inline void operator -= (const CC &A) {
          for (int i = 0; i < 7; ++i) {
            m_data[i] -= A.m_data[i];
          }
        }
        inline CC operator - (const CC &B) const {
          CC _AmB;
          AmB(*this, B, _AmB);
          return _AmB;
        }
        inline CC operator * (const xp128f &s) const {
          CC A; GetScaled(s, A); return A;
        }
        inline bool operator == (const CC &A) const {
          return m_A == A.m_A && m_b == A.m_b;
        }
        inline void MakeZero() { memset(m_data, 0, sizeof(CC)); }
        //inline void MakeZero() { memset(this, 0, 108); }
        inline void MakeMinus() {
          for (int i = 0; i < 7; ++i) {
            m_data[i].vmake_minus();
          }
        }
        inline void GetMinus(CC &A) const {
          const xp128f zero = xp128f::get(0.0f);
          for (int i = 0; i < 7; ++i) {
            A.m_data[i] = zero - m_data[i];
          }
        }
        inline void GetScaled(const xp128f &s, CC &A) const {
          for (int i = 0; i < 7; ++i) {
            A.m_data[i] = m_data[i] * s;
          }
        }
        inline bool Valid() const { return m_A.Valid(); }
        inline bool Invalid() const { return m_A.Invalid(); }
        inline void Invalidate() { m_A.Invalidate(); }
        inline void Print(const bool e = false, const bool f = false) const {
          m_A.Print(e, f);
          m_b.Print(e);
        }

        inline void PrintError(const CC &A, const bool e = false, const bool f = false) const {
          UT::PrintSeparator();
          Print(e, f);
          UT::PrintSeparator();
          A.Print(e, f);
          const CC E = *this - A;
          UT::PrintSeparator();
          E.Print(e, f);
        }
        inline bool AssertEqual(const CC &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f App1, Apr1, Arr1, App2, Apr2, Arr2;
          const LA::AlignedMatrix6x6f A1 = m_A.GetAlignedMatrix6x6f();
          const LA::AlignedMatrix6x6f A2 = A.m_A.GetAlignedMatrix6x6f();
          A1.Get00(App1); A1.Get03(Apr1); A1.Get33(Arr1);
          A2.Get00(App2); A2.Get03(Apr2); A2.Get33(Arr2);
          LA::Vector3f bp1, br1, bp2, br2;
          const LA::Vector6f b1 = m_b, b2 = A.m_b;
          b1.Get012(bp1);    b1.Get345(br1);
          b2.Get012(bp2);    b2.Get345(br2);
          bool scc = true;
          scc = App1.AssertEqual(App2, verbose, str + ".m_App", epsAbs, epsRel) && scc;
          scc = Apr1.AssertEqual(Apr2, verbose, str + ".m_Apr", epsAbs, epsRel) && scc;
          scc = Arr1.AssertEqual(Arr2, verbose, str + ".m_Arr", epsAbs, epsRel) && scc;
          scc = bp1.AssertEqual(bp2, verbose, str + ".m_bp", epsAbs, epsRel) && scc;
          scc = br1.AssertEqual(br2, verbose, str + ".m_br", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
        inline bool AssertZero(const int verbose = 1, const std::string str = "") const {
          return m_A.AssertZero(verbose, str + ".m_A", -1.0f, -1.0f) &&
                 m_b.AssertZero(verbose, str + ".m_b", -1.0f, -1.0f);
        }
        static inline void AmB(const CC &A, const CC &B, CC &AmB) {
          for (int i = 0; i < 7; ++i) {
            AmB.m_data[i] = A.m_data[i] - B.m_data[i];
          }
        }
       public:
        union {
          struct { LA::SymmetricMatrix6x6f m_A; float m_r; LA::Vector6f m_b; };
          xp128f m_data[7];
        };
      };
      class CM : public LA::AlignedMatrix6x9f {
       public:
        inline CM operator - (const CM &B) const {
          CM _AmB;
          AmB(*this, B, _AmB);
          return _AmB;
        }
        inline void PrintError(const CM &A, const bool e = false) const {
          UT::PrintSeparator();
          Print(e);
          UT::PrintSeparator();
          A.Print(e);
          const CM E = *this - A;
          UT::PrintSeparator();
          E.Print(e);
        }
        inline bool AssertEqual(const CM &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f Apv1, Apba1, Apbw1, Arv1, Arba1, Arbw1;
          LA::AlignedMatrix3x3f Apv2, Apba2, Apbw2, Arv2, Arba2, Arbw2;
          const LA::AlignedMatrix6x9f &A1 = *this;
          const LA::AlignedMatrix6x9f &A2 = A;
          A1.GetBlock(0, 0, Apv1);  A2.GetBlock(0, 0, Apv2);
          A1.GetBlock(0, 3, Apba1); A2.GetBlock(0, 3, Apba2);
          A1.GetBlock(0, 6, Apbw1); A2.GetBlock(0, 6, Apbw2);
          A1.GetBlock(3, 0, Arv1);  A2.GetBlock(3, 0, Arv2);
          A1.GetBlock(3, 3, Arba1); A2.GetBlock(3, 3, Arba2);
          A1.GetBlock(3, 6, Arbw1); A2.GetBlock(3, 6, Arbw2);
          bool scc = true;
          scc = Apv1.AssertEqual(Apv2, verbose, str + ".m_Apv", epsAbs, epsRel) && scc;
          scc = Apba1.AssertEqual(Apba2, verbose, str + ".m_Apba", epsAbs, epsRel) && scc;
          scc = Apbw1.AssertEqual(Apbw2, verbose, str + ".m_Apbw", epsAbs, epsRel) && scc;
          scc = Arv1.AssertEqual(Arv2, verbose, str + ".m_Arv", epsAbs, epsRel) && scc;
          scc = Arba1.AssertEqual(Arba2, verbose, str + ".m_Arba", epsAbs, epsRel) && scc;
          scc = Arbw1.AssertEqual(Arbw2, verbose, str + ".m_Arbw", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
      };
      class MM {
       public:
        inline void Set(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                        const LA::AlignedMatrix3x3f *Abw, const LA::AlignedVector3f *b) {
          m_A.Set(Av[0], Av[1], Av[2], Aba[1], Aba[2], Abw[2]);
          m_b.Set(b[0], b[1], b[2]);
        }
        inline void Get(LA::AlignedMatrix9x9f *A, LA::AlignedVector9f *b) const {
          A->Set(m_A);
          b->Set(m_b);
        }
        inline void operator += (const MM &A) {
          for (int i = 0; i < 14; ++i) {
            m_data[i] += A.m_data[i];
          }
        }
        inline void operator -= (const MM &A) {
          for (int i = 0; i < 14; ++i) {
            m_data[i] -= A.m_data[i];
          }
        }
        inline MM operator - (const MM &B) const {
          MM _AmB;
          AmB(*this, B, _AmB);
          return _AmB;
        }
        inline void MakeZero() { memset(this, 0, sizeof(MM)); }
        inline void MakeMinus() {
          for (int i = 0; i < 14; ++i) {
            m_data[i].vmake_minus();
          }
        }
        inline void Print(const bool e = false, const bool f = false) const {
          m_A.Print(e, f);
          m_b.Print(e);
        }
        inline void PrintError(const MM &A, const bool e = false) const {
          UT::PrintSeparator();
          Print(e);
          UT::PrintSeparator();
          A.Print(e);
          const MM E = *this - A;
          UT::PrintSeparator();
          E.Print(e);
        }
        inline bool AssertEqual(const MM &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f Avv1, Avba1, Avbw1, Ababa1, Ababw1, Abwbw1;
          LA::AlignedMatrix3x3f Avv2, Avba2, Avbw2, Ababa2, Ababw2, Abwbw2;
          const LA::SymmetricMatrix9x9f &A1 = m_A;
          const LA::SymmetricMatrix9x9f &A2 = A.m_A;
          A1.Get00(&Avv1);    A2.Get00(&Avv2);
          A1.Get03(&Avba1);   A2.Get03(&Avba2);
          A1.Get06(&Avbw1);   A2.Get06(&Avbw2);
          A1.Get33(&Ababa1);  A2.Get33(&Ababa2);
          A1.Get36(&Ababw1);  A2.Get36(&Ababw2);
          A1.Get66(&Abwbw1);  A2.Get66(&Abwbw2);
          LA::Vector3f bv1, bba1, bbw1, bv2, bba2, bbw2;
          const LA::Vector9f &b1 = m_b, b2 = A.m_b;
          b1.Get012(bv1);     b2.Get012(bv2);
          b1.Get345(bba1);    b2.Get345(bba2);
          b1.Get678(bbw1);    b2.Get678(bbw2);
          bool scc = true;
          scc = Avv1.AssertEqual(Avv2, verbose, str + ".m_Avv", epsAbs, epsRel) && scc;
          scc = Avba1.AssertEqual(Avba2, verbose, str + ".m_Avba", epsAbs, epsRel) && scc;
          scc = Avbw1.AssertEqual(Avbw2, verbose, str + ".m_Avbw", epsAbs, epsRel) && scc;
          scc = Ababa1.AssertEqual(Ababa2, verbose, str + ".m_Ababa", epsAbs, epsRel) && scc;
          scc = Ababw1.AssertEqual(Ababw2, verbose, str + ".m_Ababw", epsAbs, epsRel) && scc;
          scc = Abwbw1.AssertEqual(Abwbw2, verbose, str + ".m_Abwbw", epsAbs, epsRel) && scc;
          scc = bv1.AssertEqual(bv1, verbose, str + ".bv", epsAbs, epsRel) && scc;
          scc = bba1.AssertEqual(bba1, verbose, str + ".bba", epsAbs, epsRel) && scc;
          scc = bbw1.AssertEqual(bbw1, verbose, str + ".bbw", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
        static inline void AmB(const MM &A, const MM &B, MM &AmB) {
          for (int i = 0; i < 14; ++i) {
            AmB.m_data[i] = A.m_data[i] - B.m_data[i];
          }
        }
       public:
        union {
          struct { LA::SymmetricMatrix9x9f m_A; LA::Vector9f m_b; };
          xp128f m_data[14];
        };
      };
     public:
      inline void MakeZero() { memset(this, 0, sizeof(Unitary)); }
      inline void MakeMinus() {
        //m_Acc.MakeMinus();
        m_Acm.MakeMinus();
        m_Amm.MakeMinus();
      }
      inline void operator += (const Unitary &A) {
        //m_Acc += A.m_Acc;
        m_Acm += A.m_Acm;
        m_Amm += A.m_Amm;
      }
      inline void operator -= (const Unitary &A) {
        //m_Acc -= A.m_Acc;
        m_Acm -= A.m_Acm;
        m_Amm -= A.m_Amm;
      }
      inline void Print(const bool e = false, const bool f = false) const {
        //UT::PrintSeparator();
        //m_Acc.Print(e, f);
        UT::PrintSeparator();
        m_Acm.Print(e);
        UT::PrintSeparator();
        m_Amm.Print(e, f);
      }
      inline bool AssertEqual(const Unitary &A, const int verbose = 1, const std::string str = "",
                              const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
        bool scc = true;
        scc = m_Acm.AssertEqual(A.m_Acm, verbose, str + ".m_Acm", epsAbs, epsRel) && scc;
        scc = m_Amm.AssertEqual(A.m_Amm, verbose, str + ".m_Amm", epsAbs, epsRel) && scc;
        return scc;
      }
      static inline void AmB(const Unitary &A, const Unitary &B, Unitary &AmB) {
        //CC::AmB(A.m_Acc, B.m_Acc, AmB.m_Acc);
        CM::AmB(A.m_Acm, B.m_Acm, AmB.m_Acm);
        MM::AmB(A.m_Amm, B.m_Amm, AmB.m_Amm);
      }
     public:
      //CC m_Acc;
      CM m_Acm;
      MM m_Amm;
    };
    class Binary {
     public:
      class CC : public LA::AlignedMatrix6x6f {
       public:
        inline CC() {}
        inline CC(const LA::AlignedMatrix6x6f &A) : LA::AlignedMatrix6x6f(A) {}
        static inline CC Get(const LA::AlignedMatrix6x6f &A) { return CC(A); }
        inline void PrintError(const CC &A, const bool e = false) const {
          UT::PrintSeparator();
          LA::AlignedMatrix6x6f::Print(e);
          UT::PrintSeparator();
          A.LA::AlignedMatrix6x6f::Print(e);
          const CC E = *this - A;
          UT::PrintSeparator();
          E.LA::AlignedMatrix6x6f::Print(e);
        }
        inline bool AssertEqual(const CC &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f App1, Apr1, Arp1, Arr1, App2, Apr2, Arp2, Arr2;
          const LA::AlignedMatrix6x6f &A1 = *this, &A2 = A;
          A1.Get00(App1); A1.Get03(Apr1); A1.Get30(Arp1); A1.Get33(Arr1);
          A2.Get00(App2); A2.Get03(Apr2); A2.Get30(Arp2); A2.Get33(Arr2);
          bool scc = true;
          scc = App1.AssertEqual(App2, verbose, str + ".m_App", epsAbs, epsRel) && scc;
          scc = Apr1.AssertEqual(Apr2, verbose, str + ".m_Apr", epsAbs, epsRel) && scc;
          scc = Arp1.AssertEqual(Arp2, verbose, str + ".m_Arp", epsAbs, epsRel) && scc;
          scc = Arr1.AssertEqual(Arr2, verbose, str + ".m_Arr", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
        inline bool AssertZero(const int verbose = 1, const std::string str = "") const {
          return LA::AlignedMatrix6x6f::AssertZero(verbose, str, -1.0f, -1.0f);
        }
      };
#ifdef CFG_IMU_FULL_COVARIANCE
      class CM : public Unitary::CM {
      };
#else
      class CM {
       public:
        inline void operator += (const CM &A) { m_Arv += A.m_Arv; }
        inline void operator -= (const CM &A) { m_Arv -= A.m_Arv; }
        inline void MakeMinus() { m_Arv.MakeMinus(); }
        inline LA::AlignedMatrix6x9f GetAlignedMatrix6x9f() const {
          LA::AlignedMatrix6x9f A;
          A.MakeZero();
          A.SetBlock(3, 0, m_Arv);
          return A;
        }
        inline void Print(const bool e = false) const { m_Arv.Print(e); }
        static inline void AmB(const CM &A, const CM &B, CM &AmB) {
          LA::AlignedMatrix3x3f::AmB(A.m_Arv, B.m_Arv, AmB.m_Arv);
        }
       public:
        LA::AlignedMatrix3x3f m_Arv;
      };
#endif
      class MC : public LA::AlignedMatrix9x6f {
       public:
        inline MC operator - (const MC &B) const {
          MC _AmB;
          AmB(*this, B, _AmB);
          return _AmB;
        }
        inline void PrintError(const MC &A, const bool e = false) const {
          UT::PrintSeparator();
          LA::AlignedMatrix9x6f::Print(e);
          UT::PrintSeparator();
          A.LA::AlignedMatrix9x6f::Print(e);
          const MC E = *this - A;
          UT::PrintSeparator();
          E.LA::AlignedMatrix9x6f::Print(e);
        }
        inline bool AssertEqual(const MC &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f Avp1, Avr1, Abap1, Abar1, Abwp1, Abwr1;
          LA::AlignedMatrix3x3f Avp2, Avr2, Abap2, Abar2, Abwp2, Abwr2;
          const LA::AlignedMatrix9x6f &A1 = *this, &A2 = A;
          A1.GetBlock(0, 0, Avp1);  A2.GetBlock(0, 0, Avp2);
          A1.GetBlock(0, 3, Avr1);  A2.GetBlock(0, 3, Avr2);
          A1.GetBlock(3, 0, Abap1); A2.GetBlock(3, 0, Abap2);
          A1.GetBlock(3, 3, Abar1); A2.GetBlock(3, 3, Abar2);
          A1.GetBlock(6, 0, Abwp1); A2.GetBlock(6, 0, Abwp2);
          A1.GetBlock(6, 3, Abwr1); A2.GetBlock(6, 3, Abwr2);
          bool scc = true;
          scc = Avp1.AssertEqual(Avp2, verbose, str + ".m_Avp", epsAbs, epsRel) && scc;
          scc = Abap1.AssertEqual(Abap2, verbose, str + ".m_Abap", epsAbs, epsRel) && scc;
          scc = Abwp1.AssertEqual(Abwp2, verbose, str + ".m_Abwp", epsAbs, epsRel) && scc;
          scc = Avr1.AssertEqual(Avr2, verbose, str + ".m_Avr", epsAbs, epsRel) && scc;
          scc = Abar1.AssertEqual(Abar2, verbose, str + ".m_Abar", epsAbs, epsRel) && scc;
          scc = Abwr1.AssertEqual(Abwr2, verbose, str + ".m_Abwr", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
      };
#ifdef CFG_IMU_FULL_COVARIANCE
      class MM : public LA::AlignedMatrix9x9f {
       public:
        inline MM operator - (const MM &B) const {
          MM _AmB;
          AmB(*this, B, _AmB);
          return _AmB;
        }
        inline void PrintError(const MM &A, const bool e = false) const {
          UT::PrintSeparator();
          Print(e);
          UT::PrintSeparator();
          A.Print(e);
          const MM E = *this - A;
          UT::PrintSeparator();
          E.Print(e);
        }
        inline bool AssertEqual(const MM &A, const int verbose = 1, const std::string str = "",
                                const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
          LA::AlignedMatrix3x3f Avv1, Avba1, Avbw1, Abav1, Ababa1, Ababw1, Abwv1, Abwba1, Abwbw1;
          LA::AlignedMatrix3x3f Avv2, Avba2, Avbw2, Abav2, Ababa2, Ababw2, Abwv2, Abwba2, Abwbw2;
          const LA::AlignedMatrix9x9f &A1 = *this, A2 = A;
          A1.GetBlock(0, 0, Avv1);    A2.GetBlock(0, 0, Avv2);
          A1.GetBlock(0, 3, Avba1);   A2.GetBlock(0, 3, Avba2);
          A1.GetBlock(0, 6, Avbw1);   A2.GetBlock(0, 6, Avbw2);
          A1.GetBlock(3, 0, Abav1);   A2.GetBlock(3, 0, Abav2);
          A1.GetBlock(3, 3, Ababa1);  A2.GetBlock(3, 3, Ababa2);
          A1.GetBlock(3, 6, Ababw1);  A2.GetBlock(3, 6, Ababw2);
          A1.GetBlock(6, 0, Abwv1);   A2.GetBlock(6, 0, Abwv2);
          A1.GetBlock(6, 3, Abwba1);  A2.GetBlock(6, 3, Abwba2);
          A1.GetBlock(6, 6, Abwbw1);  A2.GetBlock(6, 6, Abwbw2);
          bool scc = true;
          scc = Avv1.AssertEqual(Avv2, verbose, str + ".m_Avv", epsAbs, epsRel) && scc;
          scc = Avba1.AssertEqual(Avba2, verbose, str + ".m_Avba", epsAbs, epsRel) && scc;
          scc = Avbw1.AssertEqual(Avbw2, verbose, str + ".m_Avbw", epsAbs, epsRel) && scc;
          scc = Abav1.AssertEqual(Abav2, verbose, str + ".m_Abav", epsAbs, epsRel) && scc;
          scc = Ababa1.AssertEqual(Ababa2, verbose, str + ".m_Ababa", epsAbs, epsRel) && scc;
          scc = Ababw1.AssertEqual(Ababw2, verbose, str + ".m_Ababw", epsAbs, epsRel) && scc;
          scc = Abwv1.AssertEqual(Abwv2, verbose, str + ".m_Abwv", epsAbs, epsRel) && scc;
          scc = Abwba1.AssertEqual(Abwba2, verbose, str + ".m_Abwba", epsAbs, epsRel) && scc;
          scc = Abwbw1.AssertEqual(Abwbw2, verbose, str + ".m_Abwbw", epsAbs, epsRel) && scc;
          if (scc) {
            return true;
          } else if (verbose) {
            PrintError(A, verbose > 1);
          }
          return false;
        }
      };
#else
      class MM {
       public:
        inline void operator += (const MM &A) {
          for (int i = 0; i < 9; ++i) {
            m_data[i] += A.m_data[i];
          }
          m_Ababa += A.m_Ababa;
          m_Abwbw += A.m_Abwbw;
        }
        inline void operator -= (const MM &A) {
          for (int i = 0; i < 9; ++i) {
            m_data[i] -= A.m_data[i];
          }
          m_Ababa -= A.m_Ababa;
          m_Abwbw -= A.m_Abwbw;
        }
        inline void MakeMinus() {
          for (int i = 0; i < 9; ++i) {
            m_data[i].vmake_minus();
          }
          m_Ababa = -m_Ababa;
          m_Abwbw = -m_Abwbw;
        }
        inline LA::Matrix9x9f GetMatrix9x9f() const {
          LA::Matrix9x9f A;
          A.MakeZero();
          for (int i = 0; i < 9; ++i) {
            memcpy(A[i], m_Amv[i], 12);
          }
          A[3][3] = A[4][4] = A[5][5] = m_Ababa;
          A[6][6] = A[7][7] = A[8][8] = m_Abwbw;
          return A;
        }
        inline void Print(const bool e = false) const {
          m_Amv.Print(e);
          if (e) {
            UT::Print("%e %e\n", m_Ababa, m_Abwbw);
          } else {
            UT::Print("%f %f\n", m_Ababa, m_Abwbw);
          }
        }
        static inline void AmB(const MM &A, const MM &B, MM &AmB) {
          for (int i = 0; i < 9; ++i) {
            AmB.m_data[i] = A.m_data[i] - B.m_data[i];
          }
          AmB.m_Ababa = A.m_Ababa - B.m_Ababa;
          AmB.m_Abwbw = A.m_Abwbw - B.m_Abwbw;
        }
       public:
        union {
          struct {
            LA::AlignedMatrix9x3f m_Amv;
            float m_Ababa, m_Abwbw;
          };
          xp128f m_data[10];
        };
      };
#endif
     public:
      inline void operator += (const Binary &A) {
        m_Acc += A.m_Acc;
        m_Acm += A.m_Acm;
        m_Amc += A.m_Amc;
        m_Amm += A.m_Amm;
      }
      inline void operator -= (const Binary &A) {
        m_Acc -= A.m_Acc;
        m_Acm -= A.m_Acm;
        m_Amc -= A.m_Amc;
        m_Amm -= A.m_Amm;
      }
#ifdef CFG_IMU_FULL_COVARIANCE
      inline void Set(const LA::AlignedMatrix3x3f *Ap, const LA::AlignedMatrix3x3f *Ar,
                      const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                      const LA::AlignedMatrix3x3f *Abw) {
        m_Acc.Set(Ap, Ar);
        m_Acm.Set(Ap + 2, Ar + 2);
        m_Amc.Set(Av, Aba, Abw);
        m_Amm.Set(Av + 2, Aba + 2, Abw + 2);
      }
      inline void Set(const LA::AlignedMatrix3x3f *Av, const LA::AlignedMatrix3x3f *Aba,
                      const LA::AlignedMatrix3x3f *Abw) {
        m_Acc.MakeZero();
        m_Acm.MakeZero();
        m_Amc.Set(Av, Aba, Abw);
        m_Amm.Set(Av + 2, Aba + 2, Abw + 2);
      }
#endif
      inline void MakeZero() { memset(this, 0, sizeof(Binary)); }
      inline void MakeMinus() {
        m_Acc.MakeMinus();
        m_Acm.MakeMinus();
        m_Amc.MakeMinus();
        m_Amm.MakeMinus();
      }
      inline void Print(const bool e = false) const {
        UT::PrintSeparator();
        m_Acc.Print(e);
        UT::PrintSeparator();
        m_Acm.Print(e);
        UT::PrintSeparator();
        m_Amc.Print(e);
        UT::PrintSeparator();
        m_Amm.Print(e);
      }
      inline bool AssertEqual(const Binary &A, const int verbose = 1, const std::string str = "",
                              const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
        bool scc = true;
        scc = m_Acc.AssertEqual(A.m_Acc, verbose, str + ".m_Acc", epsAbs, epsRel) && scc;
        scc = m_Acm.AssertEqual(A.m_Acm, verbose, str + ".m_Acm", epsAbs, epsRel) && scc;
        scc = m_Amc.AssertEqual(A.m_Amc, verbose, str + ".m_Amc", epsAbs, epsRel) && scc;
        scc = m_Amm.AssertEqual(A.m_Amm, verbose, str + ".m_Amm", epsAbs, epsRel) && scc;
        return scc;
      }
      static inline void AmB(const Binary &A, const Binary &B, Binary &AmB) {
        CC::AmB(A.m_Acc, B.m_Acc, AmB.m_Acc);
        CM::AmB(A.m_Acm, B.m_Acm, AmB.m_Acm);
        MC::AmB(A.m_Amc, B.m_Amc, AmB.m_Amc);
        MM::AmB(A.m_Amm, B.m_Amm, AmB.m_Amm);
      }
     public:
      CC m_Acc;
      CM m_Acm;
      MC m_Amc;
      MM m_Amm;
    };
   public:
    inline void MakeZero() { memset(this, 0, sizeof(Factor)); }
    inline void Print(const bool e = false, const bool f = false) const {
      UT::PrintSeparator('*');
      m_Au.Print(e, f);
      m_Ab.Print(e);
    }
    inline void AssertEqual(const Factor &A, const int verbose = 1, const std::string str = "",
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      m_Au.AssertEqual(A.m_Au, verbose, str + ".m_Au", epsAbs, epsRel);
      m_Ab.AssertEqual(A.m_Ab, verbose, str + ".m_Ab", epsAbs, epsRel);
    }
    static inline void AmB(const Factor &A, const Factor &B, Factor &AmB) {
      Unitary::AmB(A.m_Au, B.m_Au, AmB.m_Au);
      Binary::AmB(A.m_Ab, B.m_Ab, AmB.m_Ab);
    }
   public:
    Unitary m_Au;
    Binary m_Ab;
  };

  class Conditioner {
   public:
#ifdef CFG_PCG_FULL_BLOCK
    class C : public LA::AlignedMatrix6x6f {
     public:
      inline void Set(const Factor::Unitary::CC &A, const float *aMax = NULL,
                      const float *eps = NULL) {
        A.m_A.GetAlignedMatrix6x6f(*this);
        if (!InverseLDL(eps)) {
          MakeZero();
        }
      }
      template<typename TYPE>
      inline void Apply(const LA::ProductVector6f &x, TYPE *Mx) const {
        LA::AlignedMatrix6x6f::Ab(*this, x, Mx);
      }
    };
    class M : public LA::AlignedMatrix9x9f {
     public:
      inline void Set(const LA::AlignedMatrix9x9f &A, const float *aMax = NULL,
                      const float *eps = NULL) {
        if (!A.GetInverseLDL(*this, eps)) {
          MakeZero();
        }
      }
      template<typename TYPE>
      inline void Apply(const LA::AlignedVector9f &x, TYPE *Mx) const {
        LA::AlignedMatrix9x9f::Ab(*this, x, Mx);
      }
    };
#else
    class C {
     public:
      inline void operator *= (const xp128f &s) { m_Mp *= s; m_Mr *= s; }
      inline void MakeZero(float *b) { memset(this, 0, sizeof(C)); memset(b, 0, 24); }
      inline void MakeIdentity() { m_Mp.MakeIdentity(); m_Mr.MakeIdentity(); }
      inline void Set(const Factor::Unitary::CC &A, const float aMax = 0.0f,
                      const float epsI = 0.0f, const float *epsLDL = NULL) {
        LA::SymmetricMatrix3x3f _A;
        A.m_A.Get00(&_A);
        if (!_A.GetInverse(m_Mp, aMax, epsI) &&
            !_A.GetInverseLDL(m_Mp, epsLDL)) {
          m_Mp.MakeZero();
        }
        A.m_A.Get33(&_A);
        if (!_A.GetInverse(m_Mr, aMax, epsI) &&
            !_A.GetInverseLDL(m_Mr, epsLDL ? epsLDL + 3 : NULL)) {
          m_Mr.MakeZero();
        }
      }
      inline LA::AlignedMatrix6x6f GetAlignedMatrix6x6f() const {
        LA::AlignedMatrix6x6f M;
        M.MakeIdentity();
        M.Set00(m_Mp);
        M.Set33(m_Mr);
        return M;
      }
      template<typename TYPE>
      inline void Apply(const LA::AlignedVector3f &xp, const LA::AlignedVector3f &xr,
                        TYPE *Mx) const {
        LA::AlignedMatrix3x3f::Ab<TYPE>(m_Mp, xp, Mx);
        LA::AlignedMatrix3x3f::Ab<TYPE>(m_Mr, xr, Mx + 3);
      }
     public:
      LA::AlignedMatrix3x3f m_Mp, m_Mr;
    };
    class M {
     public:
      inline void operator *= (const xp128f &s) {
        m_Mv *= s;
        m_Mba *= s;
        m_Mbw *= s;
      }
      inline void MakeZero(float *b) { memset(this, 0, sizeof(M)); memset(b, 0, 36); }
      inline void MakeIdentity() {
        m_Mv.MakeIdentity();
        m_Mba.MakeIdentity();
        m_Mbw.MakeIdentity();
      }
      inline void Set(const LA::AlignedMatrix9x9f &A, const float aMax = 0.0f,
                      const float epsI = 0.0f, const float *epsLDL = NULL) {
        LA::SymmetricMatrix3x3f _A;
        A.GetBlockDiagonal(0, _A);
        if (!_A.GetInverse(m_Mv, aMax, epsI) &&
            !_A.GetInverseLDL(m_Mv, epsLDL)) {
          m_Mv.MakeZero();
        }
        A.GetBlockDiagonal(3, _A);
        if (!_A.GetInverse(m_Mba, aMax, epsI) &&
            !_A.GetInverseLDL(m_Mba, epsLDL ? epsLDL + 3 : NULL)) {
          m_Mba.MakeZero();
        }
        A.GetBlockDiagonal(6, _A);
        if (!_A.GetInverse(m_Mbw, aMax, epsI) &&
            !_A.GetInverseLDL(m_Mbw, epsLDL ? epsLDL + 6 : NULL)) {
          m_Mbw.MakeZero();
        }
      }
      inline LA::AlignedMatrix9x9f GetAlignedMatrix9x9f() const {
        LA::AlignedMatrix9x9f M;
        M.MakeIdentity();
        M.SetBlock(0, 0, m_Mv);
        M.SetBlock(3, 3, m_Mba);
        M.SetBlock(6, 6, m_Mbw);
        return M;
      }
      template<typename TYPE>
      inline void Apply(const LA::AlignedVector3f &xv, const LA::AlignedVector3f &xba,
                        const LA::AlignedVector3f &xbw, TYPE *Mx) const {
        LA::AlignedMatrix3x3f::Ab<TYPE>(m_Mv, xv, Mx);
        LA::AlignedMatrix3x3f::Ab<TYPE>(m_Mba, xba, Mx + 3);
        LA::AlignedMatrix3x3f::Ab<TYPE>(m_Mbw, xbw, Mx + 6);
      }
     public:
      LA::AlignedMatrix3x3f m_Mv, m_Mba, m_Mbw;
    };
#endif
  };

#ifdef CFG_DEBUG_EIGEN
  class EigenFactor {
   public:
    class Unitary {
     public:
      inline void MakeZero() {
        m_Acm.setZero();
        m_Amm.setZero();
      }
      inline void operator = (const Factor::Unitary &A) {
        m_Acm = A.m_Acm;
        m_Amm = A.m_Amm.m_A;
      }
      inline bool AssertEqual(const Factor::Unitary &A, const int verobse = 1,
                              const std::string str = "") const {
        return m_Acm.AssertEqual(A.m_Acm, verobse, str + ".m_Acm") &&
               m_Amm.AssertEqual(A.m_Amm.m_A, verobse, str + ".m_Amm");
      }
     public:
      EigenMatrix6x9f m_Acm;
      EigenMatrix9x9f m_Amm;
    };
    class Binary {
     public:
      inline void MakeZero() {
        m_Acc.setZero();
        m_Acm.setZero();
        m_Amc.setZero();
        m_Amm.setZero();
      }
      inline void operator = (const Factor::Binary &A) {
        m_Acc = A.m_Acc;
        m_Amc = A.m_Amc;
#ifdef CFG_IMU_FULL_COVARIANCE
        m_Acm = A.m_Acm;
        m_Amm = A.m_Amm;
#else
        m_Acm = A.m_Acm.GetAlignedMatrix6x9f();
        m_Amm = A.m_Amm.GetMatrix9x9f();
#endif
      }
      inline void AssertEqual(const Factor::Binary &A, const int verbose = 1,
                              const std::string str = "") const {
        m_Acc.AssertEqual(A.m_Acc, verbose, str + ".m_Acc");
        m_Amc.AssertEqual(A.m_Amc, verbose, str + ".m_Amc");
#ifdef CFG_IMU_FULL_COVARIANCE
        m_Acm.AssertEqual(A.m_Acm, verbose, str + ".m_Acm");
        m_Amm.AssertEqual(A.m_Amm, verbose, str + ".m_Amm");
#else
        m_Acm.AssertEqual(A.m_Acm.GetAlignedMatrix6x9f(), verbose, str + ".m_Acm");
        m_Amm.AssertEqual(A.m_Amm.GetMatrix9x9f(), verbose, str + ".m_Amm");
#endif
      }
     public:
      EigenMatrix6x6f m_Acc;
      EigenMatrix6x9f m_Acm;
      EigenMatrix9x6f m_Amc;
      EigenMatrix9x9f m_Amm;
    };
   public:
    inline EigenFactor() {}
    inline EigenFactor(const Factor &A) { *this = A; }
    inline void MakeZero() {
      m_Au.MakeZero();
      m_Ab.MakeZero();
    }
    inline void operator = (const Factor &A) {
      m_Au = A.m_Au;
      m_Ab = A.m_Ab;
    }
    inline void AssertEqual(const Factor &A, const int verbose = 1,
                            const std::string str = "") const {
      m_Au.AssertEqual(A.m_Au, verbose, str + ".m_Au");
      m_Ab.AssertEqual(A.m_Ab, verbose, str + ".m_Ab");
    }
   public:
    Unitary m_Au;
    Binary m_Ab;
  };
#endif

  class Fix {
   public:
    class ESError : public LA::AlignedVector3f {
      public:
      inline ESError() {}
      inline ESError(const LA::AlignedVector3f &e, const float s = 1.0f) {
        if (s == 1.0f)
          *((LA::AlignedVector3f *) this) = e;
        else
          e.GetScaled(s, *this);
      }
      inline void Print(const bool l = true) const {
        if (l)
          UT::Print("%f %f %f", x(), y(), z());
        else
          UT::Print("%.2f %.2f %.2f", x(), y(), z());
      }
    };
    class Origin {
     public:
      class Error {
       public:
        LA::AlignedVector3f m_er, m_ep;
      };
      class ErrorJacobian {
       public:
        Error m_e;
        LA::AlignedMatrix3x3f m_Jr;
      };
      class Factor {
       public:
        inline void MakeZero() { memset(this, 0, sizeof(Factor)); }
        ErrorJacobian m_Je;
        union {
          struct { float m_data[21], m_F; };
          Camera::Factor::Unitary::CC m_A;
        };
      };
      class Reduction {
       public:
        Error m_e;
        float m_F, m_dF;
      };
      class ES : public UT::ES<float, int> {
       public:
        inline void Initialize() {
          UT::ES<float, int>::Initialize();
          m_ESr.Initialize();
          m_ESp.Initialize();
        }
        inline void Accumulate(const Error &e, const float F, const int iFrm = -1) {
          UT::ES<float, int>::Accumulate(F, F, iFrm);
          m_ESr.Accumulate(ESError(e.m_er, UT_FACTOR_RAD_TO_DEG), -1.0f, iFrm);
          m_ESp.Accumulate(ESError(e.m_ep), -1.0f, iFrm);
        }
        inline void Print(const std::string str = "", const bool l = true) const {
          if (!Valid()) {
            return;
          }
          UT::ES<float, int>::Print(str + "eo = ", true, l);
          const std::string _str(str.size() + 18, ' ');
          if (m_ESr.Valid()) {
            m_ESr.Print(_str + "er = ", false, l);
          }
          if (m_ESp.Valid()) {
            m_ESp.Print(_str + "ep = ", false, l);
          }
        }
       public:
        UT::ES<ESError, int> m_ESr, m_ESp;
      };
      inline void Set(const float w, const LA::Vector3f &s2r, const float s2p, const Rotation3D &R) {
        m_wr[0] = UT::Inverse(s2r.x(), w);
        m_wr[1] = UT::Inverse(s2r.y(), w);
        m_wr[2] = UT::Inverse(s2r.z(), w);
        m_wp.vdup_all_lane(UT::Inverse(s2p, w));
        R.GetTranspose(m_RT);
      }
      inline void GetError(const Rigid3D &T, Error &e, const float eps) const {
        Rotation3D eR;
        //Rotation3D RT;
        //LA::AlignedVector3f g;
        //T.GetGravity(g);
        //RT.MakeIdentity(&g);
        //RT.Transpose();
        //Rotation3D::AB(RT, T, eR);
        Rotation3D::AB(m_RT, T, eR);
        eR.GetRodrigues(e.m_er, eps);
        T.GetPosition(e.m_ep);
      }
      inline void GetError(const ErrorJacobian &Je, const LA::AlignedVector3f &xp,
                           const LA::AlignedVector3f &xr, Error &e) const {
        e = Je.m_e;
        e.m_ep += xp;
        LA::AlignedMatrix3x3f::AddAbTo(Je.m_Jr, xr, (float *) &e.m_er);
      }
      inline void GetErrorJacobian(const Rigid3D &T, ErrorJacobian &Je, const float eps) const {
        Rotation3D eR;
        //Rotation3D RT;
        //LA::AlignedVector3f g;
        //T.GetGravity(g);
        //RT.MakeIdentity(&g);
        //RT.Transpose();
        //Rotation3D::AB(RT, T, eR);
        Rotation3D::AB(m_RT, T, eR);
        eR.GetRodrigues(Je.m_e.m_er, eps);
        T.GetPosition(Je.m_e.m_ep);
        Rotation3D::GetRodriguesJacobianInverse(Je.m_e.m_er, Je.m_Jr, eps);
        Je.m_Jr = Je.m_Jr * eR;
      }
      inline void GetFactor(const Rigid3D &T, Factor &A, const float eps) const {
        GetErrorJacobian(T, A.m_Je, eps);
        const LA::AlignedVector3f eTWr = A.m_Je.m_e.m_er * m_wr;
        const float r2p = A.m_Je.m_e.m_ep.SquaredLength();
        A.m_F = eTWr.Dot(A.m_Je.m_e.m_er) + m_wp[0] * r2p;
        A.m_A.m_A.MakeDiagonal(m_wp[0]);
        const LA::AlignedVector3f bp = A.m_Je.m_e.m_ep * m_wp;
        A.m_A.m_b.v0() = bp.x();
        A.m_A.m_b.v1() = bp.y();
        A.m_A.m_b.v2() = bp.z();
        const LA::AlignedMatrix3x3f JrT = A.m_Je.m_Jr.GetTranspose();
        const LA::AlignedMatrix3x3f JTWr = JrT * m_wr;
        LA::SymmetricMatrix6x6f::ABTTo33(JTWr, JrT, A.m_A.m_A);
        LA::Vector6f::AbTo3(JTWr, A.m_Je.m_e.m_er, A.m_A.m_b);
      }
      inline float GetCost(const Error &e) const {
        const LA::AlignedVector3f eTW = e.m_er * m_wr;
        return eTW.Dot(e.m_er) + m_wp[0] * e.m_ep.SquaredLength();
      }
      inline void GetReduction(const Factor &A, const Rigid3D &T, const LA::AlignedVector3f &xp,
                               const LA::AlignedVector3f &xr, Reduction &Ra, Reduction &Rp,
                               const float eps) const {
        GetError(T, Ra.m_e, eps);
        GetError(A.m_Je, xp, xr, Rp.m_e);
        Ra.m_dF = A.m_F - (Ra.m_F = GetCost(Ra.m_e));
        Rp.m_dF = A.m_F - (Rp.m_F = GetCost(Rp.m_e));
      }
     public:
      xp128f m_wr, m_wp;
      Rotation3D m_RT;
#ifdef CFG_DEBUG_EIGEN
     public:
      class EigenErrorJacobian {
       public:
        EigenVector3f m_er, m_ep;
        EigenMatrix3x3f m_Jr;
      };
      class EigenFactor {
       public:
        inline void operator = (const Factor &A) {
          m_A = A.m_A.m_A;
          m_b = A.m_A.m_b;
          m_F = A.m_F;
        }
        inline void AssertEqual(const Factor &A, const int verbose = 1,
                                const std::string str = "") const {
          m_A.AssertEqual(A.m_A.m_A, verbose, str + ".m_A");
          m_b.AssertEqual(A.m_A.m_b, verbose, str + ".m_b");
          UT::AssertEqual(m_F, A.m_F, verbose, str + ".m_F");
        }
       public:
        EigenMatrix6x6f m_A;
        EigenVector6f m_b;
        float m_F;
      };
     public:
      EigenErrorJacobian EigenGetErrorJacobian(const Rigid3D &T, const float eps) const;
      EigenFactor EigenGetFactor(const Rigid3D &T, const float eps) const;
      float EigenGetCost(const Rigid3D &T, const EigenVector6f &e_x, const float eps) const;
#endif
    };
    class Zero {
     public:
      class Factor {
       public:
        inline const float& F() const { return m_b.r(); }
        inline       float& F()       { return m_b.r(); }
       public:
        LA::AlignedVector3f m_b;
      };
      class Reduction {
       public:
        LA::AlignedVector3f m_e;
        float m_F, m_dF;
      };
     public:
      inline const float& w() const { return m_w[0]; }
      inline Zero(const float w, const float s2) {
        m_w.vdup_all_lane(UT::Inverse(s2, w));
      }
      inline void GetFactor(const LA::AlignedVector3f &v, Factor &A) const {
        v.GetScaled(m_w, A.m_b);
        A.F() = GetCost(v);
      }
      inline float GetCost(const LA::AlignedVector3f &e) const {
        return w() * e.SquaredLength();
      }
      inline float GetCost(const LA::AlignedVector3f &v, const float *x,
                           LA::AlignedVector3f &e) const {
        e.Set(x);
        e += v;
        return GetCost(e);
      }
      inline void GetReduction(const Factor &A, const LA::AlignedVector3f &v1,
                               const LA::AlignedVector3f &v2, const float *x,
                               Reduction &Ra, Reduction &Rp) const {
        Ra.m_e = v2;
        Rp.m_e.Set(x);
        Rp.m_e += v1;
        Ra.m_dF = A.F() - (Ra.m_F = GetCost(Ra.m_e));
        Rp.m_dF = A.F() - (Rp.m_F = GetCost(Rp.m_e));
      }
     public:
      xp128f m_w;
    };
    class PositionZ {
     public:
      class Factor {
       public:
        inline void MakeZero() { m_b = m_F = 0.0f; }
       public:
        float m_b, m_F;
      };
      class Reduction {
       public:
        float m_F, m_dF;
      };
      class ES : public UT::ES<float, int> {
      };
     public:
      inline PositionZ(const float w, const float s2) {
        m_w = UT::Inverse(s2, w);
      }
      inline void GetFactor(const float pz, Factor &A) const {
        A.m_b = m_w * pz;
        A.m_F = GetCost(pz);
      }
      inline float GetCost(const float e) const {
        return m_w * e * e;
      }
      inline float GetCost(const float pz, const float xpz) const {
        return GetCost(pz + xpz);
      }
      inline void GetReduction(const Factor &A, const float pz1, const float pz2,
                               const float xpz, Reduction &Ra, Reduction &Rp) const {
        Ra.m_dF = A.m_F - (Ra.m_F = GetCost(pz2));
        Rp.m_dF = A.m_F - (Rp.m_F = GetCost(pz1, xpz));
      }
     public:
      float m_w;
    };
    class Motion {
     public:
      class Factor {
       public:
        inline void MakeZero() { memset(this, 0, sizeof(Factor)); }
       public:
        Zero::Factor m_Av, m_Aba, m_Abw;
      };
      class ES {
       public:
        inline void Initialize() {
          m_ESv.Initialize();
          m_ESba.Initialize();
          m_ESbw.Initialize();
        }
        inline void AccumulateVelocity(const LA::AlignedVector3f &e, const float F,
          const int iFrm = -1) {
            m_ESv.Accumulate(ESError(e), F, iFrm);
        }
        inline void AccumulateBiasAcceleration(const LA::AlignedVector3f &e, const float F,
          const int iFrm = -1) {
            m_ESba.Accumulate(ESError(e), F, iFrm);
        }
        inline void AccumulateBiasGyroscope(const LA::AlignedVector3f &e, const float F,
          const int iFrm = -1) {
            m_ESbw.Accumulate(ESError(e, UT_FACTOR_RAD_TO_DEG), F, iFrm);
        }
        inline float Total() const { return m_ESv.m_SF + m_ESba.m_SF + m_ESbw.m_SF; }
      public:
        UT::ES<ESError, int> m_ESv, m_ESba, m_ESbw;
      };
    };
  };

 public:

  inline Camera() {}
  inline Camera(const Camera &C) {
    memcpy(this, &C, sizeof(Camera));
  }
  inline Camera(const Camera &C, const LA::AlignedVector3f *dp, const LA::AlignedVector3f *dr,
                const LA::AlignedVector3f *dv, const LA::AlignedVector3f *dba,
                const LA::AlignedVector3f *dbw, const float eps) : m_T(C.m_T, dp, dr, eps) {
    m_T.GetPosition(m_p);
    m_v = C.m_v;
    if (dv) {
      m_v += *dv;
    }
    m_ba = C.m_ba;
    if (dba) {
      m_ba += *dba;
    }
    m_bw = C.m_bw;
    if (dbw) {
      m_bw += *dbw;
    }
  }

  inline void operator = (const Camera &C) {
    memcpy(this, &C, sizeof(Camera));
  }
  inline bool operator == (const Camera &C) const {
    return m_T == C.m_T &&
           m_p == C.m_p &&
           m_v == C.m_v &&
           m_ba == C.m_ba &&
           m_bw == C.m_bw;
  }

  inline void MakeIdentity(const LA::AlignedVector3f *g = NULL) {
    m_T.MakeIdentity(g);
    m_p.MakeZero();
    m_v.MakeZero();
    m_ba.MakeZero();
    m_bw.MakeZero();
  }

  inline void Get(float *q, float *p, float *v) const {
    m_T.GetQuaternion().Get(q);
    m_p.Get(p);
    m_v.Get(v);

  }

  inline bool Valid() const { return m_T.Valid(); }
  inline bool Invalid() const { return m_T.Invalid(); }
  inline void Invalidate() { m_T.Invalidate(); m_v.Invalidate(); }

  inline void Print(const bool e = false, const bool l = false) const {
    UT::PrintSeparator();
    m_T.Print(" T = ", e);
    m_p.Print(" p = ", e, l, true);
    m_v.Print(" v = ", e, l, true);
    m_ba.Print("ba = ", e, l, true);
    m_bw.Print("bw = ", e, l, true);
  }
  inline void Print(const std::string str, const bool e, const bool l) const {
    UT::PrintSeparator();
    const std::string _str(str.size(), ' ');
    m_T.Print( str + " T = ", e);
    m_p.Print(_str + " p = ", e, l, true);
    m_v.Print(_str + " v = ", e, l, true);
    m_ba.Print(_str + "ba = ", e, l, true);
    m_bw.Print(_str + "bw = ", e, l, true);
  }

  inline void AssertConsistency(const int verbose = 1, const std::string str = "") const {
    m_T.AssertOrthogonal(verbose, str + ".m_R");
    UT_ASSERT(m_p.Valid());
    m_p.AssertEqual(m_T.GetPosition(), verbose, str + ".m_p");
  }

  inline bool AssertEqual(const Camera &C, const int verbose = 1, const std::string str = "",
                          const float rEps = 0.001745329252f, const float pEps = 0.0f,
                          const float vEps = 0.0f) const {
    if (m_T.AssertEqual(C.m_T, verbose, str + ".m_T", rEps, pEps) &&
        m_p.AssertEqual(C.m_p, verbose, str + ".m_p", pEps) &&
        m_v.AssertEqual(C.m_v, verbose, str + ".m_v", vEps) &&
        m_ba.AssertEqual(C.m_ba, verbose, str + ".m_ba") &&
        m_bw.AssertEqual(C.m_bw, verbose, str + ".m_bw")) {
      return true;
    } else if (verbose) {
      Print(verbose > 1);
      C.Print(verbose > 1);
    }
    return false;
  }

 public:

  Rigid3D m_T;
  Point3D m_p;
  LA::AlignedVector3f m_v, m_ba, m_bw;

};

#endif
