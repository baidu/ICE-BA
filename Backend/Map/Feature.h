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
#ifndef _FEATURE_H_
#define _FEATURE_H_

#include "Camera.h"
#include "Depth.h"

namespace FTR {

class Source {
 public:
  inline void Set(const float *x) {
    m_x.Set(x);
#ifdef CFG_STEREO
    m_xr.Invalidate();
#endif
  }
  inline bool operator == (const Source &x) const {
    return m_x == x.m_x
#ifdef CFG_STEREO
        && m_xr == x.m_xr
        && m_Wr == x.m_Wr
#endif
        ;
  }
 public:
  Point2D m_x;
#ifdef CFG_STEREO
  Point2D m_xr;
  LA::SymmetricMatrix2x2f m_Wr;
#endif
};

class Measurement {
 public:
  class Match {
   public:
    inline Match() {}
    inline Match(const int iz1, const int iz2) : m_iz1(iz1), m_iz2(iz2) {}
    inline bool operator < (const int iz2) const { return m_iz2 < iz2; }
    inline bool operator == (const Match &izm) const { return m_iz1 == izm.m_iz1 && m_iz2 == izm.m_iz2; }
    inline void Set(const int iz1, const int iz2) { m_iz1 = iz1; m_iz2 = iz2; }
   public:
    int m_iz1, m_iz2;
  };
 public:
  inline Measurement() {}
  inline bool operator == (const Source &x) const {
    return m_z == x.m_x
#ifdef CFG_STEREO
        && m_zr == x.m_xr && m_Wr == x.m_Wr
#endif
        ;
  }
  inline bool operator == (const Measurement &z) const {
    return m_ix == z.m_ix
        && m_z == z.m_z && m_W == z.m_W
#ifdef CFG_STEREO
        && m_zr == z.m_zr && m_Wr ==z.m_Wr
#endif
        ;
  }
  inline bool operator < (const int ix) const { return m_ix < ix; }
  inline bool operator < (const Measurement &z) const { return m_ix < z.m_ix; }
  inline void Set(const int ix, const Point2D &z, const LA::SymmetricMatrix2x2f &W
#ifdef CFG_STEREO
                , const Point2D &zr, const LA::SymmetricMatrix2x2f &Wr
#endif
                ) {
    m_ix = ix;
    m_z = z;
    m_W = W;
#ifdef CFG_STEREO
    m_zr = zr;
    m_Wr = Wr;
#endif
  }
  inline bool Valid() const { return m_ix >= 0; }
  inline bool Invalid() const { return m_ix == -1; }
  inline void Invalidate() { m_ix = -1; }
 public:
  union { int m_iKF, m_id, m_ix; };
  Point2D m_z;
  LA::SymmetricMatrix2x2f m_W;
#ifdef CFG_STEREO
  Point2D m_zr;
  LA::SymmetricMatrix2x2f m_Wr;
#endif
};

class ESError : public LA::Vector2f {
 public:
  inline ESError() {}
  inline ESError(const Intrinsic &K, const LA::Vector2f &ex) : LA::Vector2f(K.fx() * ex.x(),
                                                                            K.fy() * ex.y()) {}
  inline void Print(const bool l = true) const {
    const float ex = sqrtf(SquaredLength());
    if (l) {
      UT::Print("%f", ex);
    } else {
      UT::Print("%.2f", ex);
    }
  }
};
class ESIndex {
 public:
  inline ESIndex() : m_ixFrm(-1), m_ix(-1), m_izFrm(-1), m_iz(-1) {}
  inline ESIndex(const int ixFrm, const int ix, const int izFrm = -1,
                 const int iz = -1) : m_ixFrm(ixFrm), m_ix(ix), m_izFrm(izFrm), m_iz(iz) {}
  inline operator int() const { return m_iz; }
  inline void Print() const {
    if (m_ixFrm == -1 || m_ix == -1) {
      return;
    }
    UT::Print(" [%d] %d", m_ixFrm, m_ix);
    if (m_izFrm != -1) {
      UT::Print(" [%d]", m_izFrm);
    }
    if (m_iz != -1) {
      UT::Print(" %d", m_iz);
    }
  }
  inline void Save(FILE *fp) const {
    if (m_ixFrm == -1 || m_ix == -1) {
      return;
    }
    fprintf(fp, " [%d] %d", m_ixFrm, m_ix);
    if (m_izFrm != -1) {
      fprintf(fp, " [%d]", m_izFrm);
    }
    if (m_iz != -1) {
      fprintf(fp, " %d", m_iz);
    }
  }
 public:
  int m_ixFrm, m_ix, m_izFrm, m_iz;
};
class ES {
 public:
  inline void Initialize(const bool r = true) { m_ESx.Initialize(r); m_ESd.Initialize(r); }
  inline void Accumulate(const Intrinsic &K, const LA::Vector2f &ex, const float F,
                         const ESIndex idx, const bool r = true) {
    m_ESx.Accumulate(ESError(K, ex), F, idx, r);
  }
  inline void Accumulate(const float ed, const float F, const ESIndex idx, const bool r = true) {
    m_ESd.Accumulate(ed, F, idx, r);
  }
  inline float Total() const {
    return m_ESx.m_SF + m_ESd.m_SF;
  }
  inline void Print(const std::string str = "", const bool l = true, const int r = 1) const {
    if (m_ESx.Valid()) {
      m_ESx.Print(str + "ex = ", true, l, true, r);
    }
    if (m_ESd.Valid()) {
      m_ESd.Print(m_ESx.Valid() ? std::string(str.size(), ' ') + "   + " : str + "ex = ",
                  true, l, true, r);
    }
  }
 public:
  UT::ES<ESError, ESIndex> m_ESx;
  UT::ES<float, ESIndex> m_ESd;
};

class Error {
 public:
  inline void Invalidate() {
    m_e.Invalidate();
#ifdef CFG_STEREO
    m_er.Invalidate();
#endif
  }
  inline bool Invalid() const {
    return m_e.Invalid()
#ifdef CFG_STEREO
        && m_er.Invalid()
#endif
        ;
  }
  inline bool Valid() const {
    return m_e.Valid()
#ifdef CFG_STEREO
        || m_er.Valid()
#endif
        ;
  }
 public:
  LA::Vector2f m_e;
#ifdef CFG_STEREO
  LA::Vector2f m_er;
#endif
};
namespace ErrorJacobian {
class D {
 public:
  inline bool Valid() const { return m_e.Valid(); }
  inline bool Invalid() const { return m_e.Invalid(); }
  inline void Invalidate() { m_e.Invalidate(); }
 public:
  LA::Vector2f m_Jd, m_e;
};
class X {
 public:
  inline bool Valid() const { return m_e.Valid(); }
  inline bool Invalid() const { return m_e.Invalid(); }
  inline void Invalidate() { m_e.Invalidate(); }
 public:
  LA::Matrix2x3f m_Jx;
  LA::Vector2f m_e;
};
class DCZ : public D {
 public:
  inline void MakeZero() { memset(this, 0, sizeof(DCZ)); }
 public:
  LA::AlignedMatrix2x6f m_Jcz;
};
class DCXZ : public DCZ {
 public:
  inline void MakeZero() { memset(this, 0, sizeof(DCXZ)); }
 public:
  LA::AlignedMatrix2x6f m_Jcx;
};
class XC : public X {
 public:
  inline void MakeZero() { memset(this, 0, sizeof(XC)); }
 public:
  LA::AlignedMatrix2x6f m_Jc;
};
}  // namespace ErrorJacobian
class Reduction {
 public:
  Error m_e;
  float m_F, m_dF;
};
namespace Factor {
class DD {
 public:
  static inline DD Get(const float a, const float b) { DD _a; _a.Set(a, b); return _a; }
  static inline DD Get(const Depth::Prior::Factor &A) { DD _a; _a.Set(A.m_a, A.m_b); return _a; }
  inline void Set(const float a, const float b) { m_a = a; m_b = b; }
  inline void operator = (const Depth::Prior::Factor &A) { m_a = A.m_a; m_b = A.m_b; }
  inline void operator += (const DD &a) { m_a = a.m_a + m_a; m_b = a.m_b + m_b; }
  inline void operator += (const Depth::Prior::Factor &A) {
    m_a = A.m_a + m_a;
    m_b = A.m_b + m_b;
  }
  inline void operator -= (const Depth::Prior::Factor &A) {
    m_a = -A.m_a + m_a;
    m_b = -A.m_b + m_b;
  }
  inline void operator *= (const float s) { m_a *= s; m_b *= s; }
  inline DD operator - (const DD &b) const { DD _amb; amb(*this, b, _amb); return _amb; }
  inline DD operator * (const float s) const { DD sa; sa.Set(s * m_a, s * m_b); return sa; }
  inline bool operator == (const DD &a) const { return m_a == a.m_a && m_b == a.m_b; }
  inline void MakeZero() { memset(this, 0, sizeof(DD)); }
  inline void MakeMinus() { m_a = -m_a; m_b = -m_b; }
  inline void GetMinus(DD &a) const { a.m_a = -m_a; a.m_b = -m_b; }

  inline bool Valid() const { return m_a != FLT_MAX; }
  inline bool Invalid() const { return m_a == FLT_MAX; }
  inline void Invalidate() { m_a = FLT_MAX; }
  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e\n", m_a, m_b);
    } else {
      UT::Print("%f %f\n", m_a, m_b);
    }
  }
  inline bool AssertEqual(const DD &a, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::AssertEqual(m_a, a.m_a, verbose, str + ".m_a", epsAbs, epsRel) &&
        UT::AssertEqual(m_b, a.m_b, verbose, str + ".m_b", epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      a.Print(verbose > 1);
      const DD e = *this - a;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "") const {
    return UT::AssertZero(m_a, verbose, str + ".m_a", -1.0f, -1.0f) &&
           UT::AssertZero(m_b, verbose, str + ".m_b", -1.0f, -1.0f);
  }
  static inline void amb(const DD &a, const DD &b, DD &amb) {
    amb.m_a = a.m_a - b.m_a;
    amb.m_b = a.m_b - b.m_b;
  }
  static inline void amb(const Depth::Prior::Factor &A, const DD &b, DD &amb) {
    amb.m_a = A.m_a - b.m_a;
    amb.m_b = A.m_b - b.m_b;
  }
 public:
  float m_a, m_b;
};
class XX {
 public:
  union {
    struct { LA::Matrix3x3f m_A; LA::Vector3f m_b; };
    xp128f m_data[3];
  };
};
class DC : public LA::Vector6f {
 public:
  static inline DC Get(const float *a) { DC _a; _a.Set(a); return _a; }
  inline bool AssertEqual(const DC &a, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    LA::Vector3f ap1, ar1, ap2, ar2;
    Get012(ap1);  a.Get012(ap2);
    Get345(ar1);  a.Get345(ar2);
    if (ap1.AssertEqual(ap2, verbose, str + ".m_ap", epsAbs, epsRel) &&
        ar1.AssertEqual(ar2, verbose, str + ".m_ar", epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      a.Print(verbose > 1);
      const LA::Vector6f e = *this - a;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "") const {
    return LA::Vector6f::AssertZero(verbose, str, -1.0f, -1.0f);
  }
};
class DDC {
 public:
  inline DDC() {}
  inline ~DDC() {}
  inline bool operator == (const DDC &a) const {
    return m_adc == a.m_adc &&
           m_add == a.m_add;
  }
  inline void operator += (const DDC &a) {
    m_data[0] += a.m_data[0];
    m_data[1] += a.m_data[1];
  }
  inline void operator -= (const DDC &a) {
    m_data[0] -= a.m_data[0];
    m_data[1] -= a.m_data[1];
  }
  inline void operator *= (const float s) {
    const xp128f _s = xp128f::get(s);
    Scale(_s);
  }
  inline DDC operator + (const DD &add) const {
    DDC _a = *this;
    _a.m_add += add;
    return _a;
  }
  inline DDC operator + (const Depth::Prior::Factor &A) const {
    DDC a = *this;
    a.m_add += A;
    return a;
  }
  inline DDC operator * (const float s) const {
    DDC _a;
    GetScaled(s, _a);
    return _a;
  }
  inline void Set(const DD &add, const LA::AlignedVector6f &adc) {
    m_adcA = adc;
    m_add = add;
  }
  inline void MakeZero() { memset(this, 0, sizeof(DDC)); }
  inline void MakeMinus() {
    m_data[0].vmake_minus();
    m_data[1].vmake_minus();
  }
  inline void GetMinus(DDC &a) const {
    const xp128f zero = xp128f::get(0.0f);
    a.m_data[0] = zero - m_data[0];
    a.m_data[1] = zero - m_data[1];
  }
  inline void Scale(const xp128f &s) {
    m_data[0] *= s;
    m_data[1] *= s;
  }
  inline void GetScaled(const float s, DDC &a) const {
    const xp128f _s = xp128f::get(s);
    GetScaled(_s, a);
  }
  inline void GetScaled(const xp128f &s, DDC &a) const {
    a.m_data[0] = m_data[0] * s;
    a.m_data[1] = m_data[1] * s;
  }
  inline DDC GetScaled(const xp128f &s) const {
    DDC a;
    GetScaled(s, a);
    return a;
  }
  inline bool Valid() const { return m_adc.Valid() && m_add.Valid(); }
  inline bool Invalid() const { return m_adc.Invalid() && m_add.Invalid(); }
  inline void Invalidate() { m_adc.Invalidate(); m_add.Invalidate(); }
  inline bool AssertEqual(const DDC &a, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return m_adc.AssertEqual(a.m_adc, verbose, str + ".m_adc", epsAbs, epsRel) &&
           m_add.AssertEqual(a.m_add, verbose, str + ".m_add", epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "") const {
    return m_adc.AssertZero(verbose, str + ".m_adc") &&
           m_add.AssertZero(verbose, str + ".m_add");
  }
  static inline void aTb(const float *a, const xp128f &b0, const xp128f &b1,
                         Camera::Factor::Unitary::CC &aTb) {
    xp128f t1, t2;
    t1.vdup_all_lane(a[0]);
    aTb.m_data[0] = t1 * b0;
    t2 = t1 * b1;

    memcpy(&aTb.m_A.m04(), &t2[0], 8);
    aTb.m_b.v0() = t2[3];
    t1.vdup_all_lane(a[1]);
    t2 = t1 * b0;
    memcpy(&aTb.m_A.m11(), &t2[1], 12);
    t2 = t1 * b1;
    memcpy(&aTb.m_A.m14(), &t2[0], 8);
    aTb.m_b.v1() = t2[3];
    aTb.m_A.m22() = a[2] * b0[2];
    aTb.m_A.m23() = a[2] * b0[3];
    t2 = b1 * a[2];
    memcpy(&aTb.m_A.m24(), &t2[0], 8);
    aTb.m_b.v2() = t2[3];
    aTb.m_A.m33() = a[3] * b0[3];
    t2 = b1 * a[3];
    memcpy(&aTb.m_A.m34(), &t2[0], 8);
    aTb.m_b.v3() = t2[3];
    t2 = b1 * a[4];
    memcpy(&aTb.m_A.m44(), &t2[0], 8);
    aTb.m_b.v4() = t2[3];
    aTb.m_A.m55() = a[5] * b1[1];
    aTb.m_b.v5() = a[5] * b1[3];
  }
  static inline void aTb(const float *a, const DDC &b, Camera::Factor::Unitary::CC &aTb) {
    DDC::aTb(a, b.m_data[0], b.m_data[1], aTb);
  }
  static inline void amb(const DDC &a, const DDC &b, DDC &amb) {
    amb.m_data[0] = a.m_data[0] - b.m_data[0];
    amb.m_data[1] = a.m_data[1] - b.m_data[1];
  }
 public:
  union {
    struct { DC m_adc; DD m_add; };
    LA::AlignedVector6f m_adcA;
    LA::AlignedVector7f m_adcd;
    xp128f m_data[2];
  };
};
class Stereo {
 public:
  class U {
   public:
    inline void Initialize() { m_A.MakeZero(); }
    inline void Accumulate(const ErrorJacobian::D &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
      LA::SymmetricMatrix2x2f::Ab(W, Je.m_Jd, m_WJ);
      m_WJ *= w;
      m_A.m_a = m_WJ.Dot(Je.m_Jd) + m_A.m_a;
      m_A.m_b = m_WJ.Dot(Je.m_e) + m_A.m_b;
    }
    inline void Set(const ErrorJacobian::D &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
      LA::SymmetricMatrix2x2f::Ab(W, Je.m_Jd, m_WJ);
      m_WJ *= w;
      m_A.m_a = m_WJ.Dot(Je.m_Jd);
      m_A.m_b = m_WJ.Dot(Je.m_e);
    }
   public:
    LA::Vector2f m_WJ;
    DD m_A;
  };
 public:
  inline void MakeZero() { memset(this, 0, sizeof(Stereo)); }
 public:
  ErrorJacobian::D m_Je;
  float m_w, m_F;
  DD m_add;
};
class Depth : public Stereo {
 public:
#ifdef CFG_STEREO
  ErrorJacobian::D m_Jer;
  float m_wr;
#endif
};
namespace FixSource {
namespace Source {
class A {
 public:
  inline void operator *= (const float s) { m_Sadd *= s; }
  inline A operator * (const float s) const { A _A; _A.m_Sadd = m_Sadd * s; return _A; }
  inline bool operator == (const A &_A) const { return m_Sadd == _A.m_Sadd; }
  inline void MakeZero() { m_Sadd.MakeZero(); }
 public:
  DD m_Sadd;
};
class M {
 public:
  inline void MakeZero() { m_mdd.MakeZero(); }
  inline void operator += (const M &_M) { m_mdd += _M.m_mdd; }
  inline float BackSubstitute() const { return m_mdd.m_b; }
 public:
  DD m_mdd;
};
}  // namespace Source
class L {
 public:
  ErrorJacobian::DCZ m_Je;
#ifdef CFG_STEREO
  ErrorJacobian::DCZ m_Jer;
#endif
  union {
    struct {
      float m_w;
#ifdef CFG_STEREO
      float m_wr;
#endif
      float m_F;
    };
    xp128f m_data;
  };
};
class A1 {
 public:
  inline void operator *= (const float s) { m_adczA *= s; }
  inline A1 operator * (const float s) const {
    A1 A = *this;
    A *= s;
    return A;
  }
  inline bool AssertEqual(const A1 &A, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return m_adcz.AssertEqual(A.m_adcz, verbose, str + ".m_adcz", epsAbs, epsRel);
  }
 public:
  union {
    DC m_adcz;
    LA::AlignedVector6f m_adczA;
  };
};
class A2 {
 public:
  union {
    struct {
      DD m_add;
      Camera::Factor::Unitary::CC m_Aczz;
    };
    xp128f m_data[8];
  };
};
class A3 : public DDC {
 public:
  inline A3() {}
  inline A3(const DD &add, const DC &adc) {
    m_add = add;
    m_adc = adc;
  }
  inline A3(const DDC &A) : DDC(A) {}
};
class M1 : public A1 {
 public:
  inline float BackSubstitute(const LA::AlignedVector6f &xc) const {
    return m_adczA.Dot(xc);
  }
};
class M2 {
 public:
  Camera::Factor::Unitary::CC m_Mczz;
};
class U {
 public:
  inline void Initialize() {
    m_A.MakeZero();
  }
  inline void Accumulate(const ErrorJacobian::DCZ &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
    m_J.Set(Je.m_Jd, Je.m_Jcz);
    m_Je.Set(m_J, Je.m_e);
    W.GetScaled(w, m_W);
    LA::AlignedMatrix2x7f::AB(m_W, m_J, m_WJ);
    LA::AlignedMatrix7x8f::AddATBToUpper(m_WJ, m_Je, m_A);
  }
  inline void Set(const ErrorJacobian::DCZ &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
    m_J.Set(Je.m_Jd, Je.m_Jcz);
    m_Je.Set(m_J, Je.m_e);
    W.GetScaled(w, m_W);
    LA::AlignedMatrix2x7f::AB(m_W, m_J, m_WJ);
    LA::AlignedMatrix7x8f::ATBToUpper(m_WJ, m_Je, m_A);
  }
 public:
  LA::AlignedMatrix2x7f m_J, m_WJ;
  LA::AlignedMatrix2x8f m_Je;
  LA::AlignedMatrix7x8f m_A;
  LA::SymmetricMatrix2x2f m_W;
};
inline void Marginalize(const xp128f &mdd, const A1 &A, M1 *M) {
  A.m_adczA.GetScaled(mdd, M->m_adczA);
}
inline void Marginalize(const DD &Smdd, const LA::AlignedVector6f &adcz,
                        LA::AlignedVector6f *Smdcz, M2 *M) {
  adcz.GetScaled(Smdd.m_a, *Smdcz);
  Smdcz->v45xx()[3] = Smdd.m_b;
  DDC::aTb(adcz, Smdcz->v0123(), Smdcz->v45xx(), M->m_Mczz);
}
inline void Marginalize(const float Smdd, const A3 &A1, const A3 &A2,
                        LA::ProductVector6f *Smdcz2, Camera::Factor::Binary::CC *Mczm) {
  A2.m_adcA.GetScaled(Smdd, *Smdcz2);
  Smdcz2->Update();
  LA::AlignedMatrix6x6f::abT(A1.m_adc, *Smdcz2, *Mczm);
}
}  // namespace FixSource
namespace Full {
namespace Source {
class A {
 public:
  inline void MakeZero() { m_Sadx.MakeZero(); }
 public:
  DDC m_Sadx;
};
class M1 {
 public:
  inline void MakeZero() { memset(this, 0, sizeof(M1)); }
  inline float BackSubstitute(const LA::AlignedVector6f *xc = NULL) const {
    const float bd = m_mdx.m_add.m_b;
    if (xc) {
      return bd + m_mdx.m_adcA.Dot(*xc);
    } else {
      return bd;
    }
  }
 public:
  DDC m_mdx;
};
class M2 {
 public:
  inline void MakeZero() { memset(this, 0, sizeof(M2)); }
 public:
  Camera::Factor::Unitary::CC m_Mcxx;
};
inline void Marginalize(const xp128f &mdd, const DDC &Sadx, M1 *_M1, M2 *_M2) {
  Sadx.GetScaled(mdd, _M1->m_mdx);
#ifdef CFG_DEBUG
//#if 0
  UT_ASSERT(mdd[0] == 1.0f / Sadx.m_add.m_a);
#endif
  _M1->m_mdx.m_add.m_a = mdd[0];
  DDC::aTb(_M1->m_mdx.m_adc, Sadx, _M2->m_Mcxx);
}
}  // namespace Source
class L {
 public:
  ErrorJacobian::DCXZ m_Je;
#ifdef CFG_STEREO
  ErrorJacobian::DCXZ m_Jer;
#endif
  union {
    struct {
      float m_w;
#ifdef CFG_STEREO
      float m_wr;
#endif
      float m_F;
    };
    xp128f m_data;
  };
};
class A1 : public FixSource::A1 {};
class A2 {
 public:
  DDC m_adx;
  Camera::Factor::Unitary::CC m_Acxx;
  Camera::Factor::Binary::CC m_Acxz;
  Camera::Factor::Unitary::CC m_Aczz;
};
class M1 : public FixSource::M1 {};
class M2 : public FixSource::M2 {
 public:
  Camera::Factor::Binary::CC m_Mcxz;
};
class U {
 public:
  inline void Initialize() { m_A.MakeZero(); }
  inline void Accumulate(const ErrorJacobian::DCXZ &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
    m_J.Set(Je.m_Jd, Je.m_Jcx, Je.m_Jcz);
    m_Je.Set(m_J, Je.m_e);
    W.GetScaled(w, m_W);
    LA::AlignedMatrix2x13f::AB(m_W, m_J, m_WJ);
    LA::AlignedMatrix13x14f::AddATBToUpper(m_WJ, m_Je, m_A);
  }
  inline void Set(const ErrorJacobian::DCXZ &Je, const float w, const LA::SymmetricMatrix2x2f &W) {
    m_J.Set(Je.m_Jd, Je.m_Jcx, Je.m_Jcz);
    m_Je.Set(m_J, Je.m_e);
    W.GetScaled(w, m_W);
    LA::AlignedMatrix2x13f::AB(m_W, m_J, m_WJ);
    LA::AlignedMatrix13x14f::ATBToUpper(m_WJ, m_Je, m_A);
  }
 public:
  LA::AlignedMatrix2x13f m_J, m_WJ;
  LA::AlignedMatrix2x14f m_Je;
  LA::AlignedMatrix13x14f m_A;
  LA::SymmetricMatrix2x2f m_W;
};
inline void Marginalize(const xp128f &mdd, const Source::M1 &Mx, const A1 &Az, M1 *Mz1,
                        M2 *Mz2, LA::ProductVector6f *adcz) {
#ifdef CFG_DEBUG
  UT_ASSERT(mdd[0] == Mx.m_mdx.m_add.m_a);
#endif
  Az.m_adczA.GetScaled(mdd, Mz1->m_adczA);
  adcz->Set(Az.m_adczA);
  LA::AlignedMatrix6x6f::abT(Mx.m_mdx.m_adc, *adcz, Mz2->m_Mcxz);

  const xp128f t = xp128f::get(Mz1->m_adcz.v4(), Mz1->m_adcz.v5(),
                               Mx.m_mdx.m_add.m_a, Mx.m_mdx.m_add.m_b);
  DDC::aTb(Az.m_adcz, Mz1->m_adczA.v0123(), t, Mz2->m_Mczz);
}
static inline void Marginalize(const M1 &Mz, const LA::ProductVector6f &adcz,
                               Camera::Factor::Binary::CC &Mczm) {
  LA::AlignedMatrix6x6f::abT(Mz.m_adcz, adcz, Mczm);
}
}  // namespace Full
}  // namespace Factor

#ifdef CFG_DEBUG
inline void DebugSetMeasurement(const Rigid3D &T12, const Source &x1,
                                const Depth::InverseGaussian &d1, Point2D &z2) {
  UT_ASSERT(z2.Valid());
  d1.Project(T12, x1.m_x, z2);
}
inline void DebugSetMeasurement(const Rigid3D *T12, const Source &x1,
                                const Depth::InverseGaussian &d1, Measurement &z2) {
#ifdef CFG_STEREO
  if (z2.m_z.Valid()) {
    DebugSetMeasurement(T12[0], x1, d1, z2.m_z);
  }
  if (z2.m_zr.Valid()) {
    DebugSetMeasurement(T12[1], x1, d1, z2.m_zr);
  }
#else
  DebugSetMeasurement(*T12, x1, d1, z2.m_z);
#endif
}
#endif
inline void GetError(const Rigid3D &T12, const Source &x1, const Depth::InverseGaussian &d1,
                     const Point2D &z2, LA::Vector2f &e2) {
#ifdef CFG_DEBUG
  UT_ASSERT(z2.Valid());
#endif
  d1.Project(T12, x1.m_x, e2);
  e2 -= z2;
}
inline void GetError(const Rigid3D *T12, const Source &x1,
                     const Depth::InverseGaussian &d1, const Measurement &z2,
                     Error &e2) {
#ifdef CFG_STEREO
  if (z2.m_z.Valid()) {
    GetError(T12[0], x1, d1, z2.m_z, e2.m_e);
  }
  if (z2.m_zr.Valid()) {
    GetError(T12[1], x1, d1, z2.m_zr, e2.m_er);
  }
#else
  GetError(*T12, x1, d1, z2.m_z, e2.m_e);
#endif
}
inline Error GetError(const Rigid3D *T12, const Source &x1,
                      const Depth::InverseGaussian &d1, const Measurement &z2) {
  Error e2;
  GetError(T12, x1, d1, z2, e2);
  return e2;
}
inline void GetError(const ErrorJacobian::D &Je, const float xd, LA::Vector2f &e) {
  e = Je.m_e;
  e += Je.m_Jd * xd;
}
inline void GetError(const ErrorJacobian::DCZ &Je, const LA::ProductVector6f *xcz,
                     const float *xd, LA::Vector2f &e) {
#ifdef CFG_DEBUG
  UT_ASSERT(xcz || xd);
  UT_ASSERT(Je.Valid());
#endif
  e = Je.m_e;
  if (xcz) {
    LA::AlignedMatrix2x6f::AddAbTo(Je.m_Jcz, *xcz, e);
  }
  if (xd) {
    e += Je.m_Jd * *xd;
  }
}
inline void GetError(const ErrorJacobian::DCXZ &Je, const LA::ProductVector6f *xcx,
                     const LA::ProductVector6f *xcz, const float *xd, LA::Vector2f &e) {
#ifdef CFG_DEBUG
  UT_ASSERT(xcx || xcz || xd);
  UT_ASSERT(Je.Valid());
#endif
  if (xcz || xd) {
    GetError(Je, xcz, xd, e);
  } else {
    e = Je.m_e;
  }
  if (xcx) {
    LA::AlignedMatrix2x6f::AddAbTo(Je.m_Jcx, *xcx, e);
  }
}
inline void GetError(const Factor::Depth &A, const float xd, Error &e) {
#ifdef CFG_STEREO
  if (A.m_Je.Valid()) {
    GetError(A.m_Je, xd, e.m_e);
  }
  if (A.m_Jer.Valid()) {
    GetError(A.m_Jer, xd, e.m_er);
  }
#else
  GetError(A.m_Je, xd, e.m_e);
#endif
}
inline void GetError(const Factor::FixSource::L &L, const LA::ProductVector6f *xcz,
                     const float *xd, Error &e) {
#ifdef CFG_DEBUG
  UT_ASSERT(xcz || xd);
#endif
#ifdef CFG_STEREO
  if (L.m_Je.Valid()) {
    GetError(L.m_Je, xcz, xd, e.m_e);
  } else {
    e.m_e.Invalidate();
  }
  if (L.m_Jer.Valid()) {
    GetError(L.m_Jer, xcz, xd, e.m_er);
  } else {
    e.m_er.Invalidate();
  }
#else
  GetError(L.m_Je, xcz, xd, e.m_e);
#endif
}
inline void GetError(const Factor::Full::L &L, const LA::ProductVector6f *xcx,
                     const LA::ProductVector6f *xcz, const float *xd, Error &e) {
#ifdef CFG_DEBUG
  UT_ASSERT(xcx || xcz || xd);
#endif
#ifdef CFG_STEREO
  if (L.m_Je.Valid()) {
    GetError(L.m_Je, xcx, xcz, xd, e.m_e);
  }
  if (L.m_Jer.Valid()) {
    GetError(L.m_Jer, xcx, xcz, xd, e.m_er);
  }
#else
  GetError(L.m_Je, xcx, xcz, xd, e.m_e);
#endif
}

inline void GetErrorJacobian(const Rigid3D &T12, const Source &x1, const Depth::InverseGaussian &d1,
                             const Rigid3D &T2, const Point2D &z2, ErrorJacobian::D &Je2
#ifdef CFG_STEREO
                           , const Point3D *br = NULL
#endif
                           ) {
#ifdef CFG_DEBUG
  UT_ASSERT(z2.Valid());
#endif
  d1.Project(T12, x1.m_x, Je2.m_e, Je2.m_Jd);
  Je2.m_e -= z2;
}
inline void GetErrorJacobian(const Rigid3D &T12, const Source &x1, const Depth::InverseGaussian &d1,
                             const Rigid3D &T2, const Point2D &z2, ErrorJacobian::DCZ &Je2
#ifdef CFG_STEREO
                           , const Point3D *br = NULL
#endif
                           ) {
#ifdef CFG_DEBUG
  UT_ASSERT(z2.Valid());
#endif
  float d2;
  d1.Project(T12, x1.m_x, Je2.m_e, d2, Je2.m_Jd);
  //const bool vp = fabs(d2) > DEPTH_EPSILON;
  //const bool vp = fabs(d2) > DEPTH_PROJECTION_MIN;
  //const bool vp = d2 > DEPTH_PROJECTION_MIN;
  const bool vp = d2 > DEPTH_PROJECTION_MIN && d2 < DEPTH_PROJECTION_MAX;
  if (vp) {
    const xp128f _d2 = xp128f::get(d2);
    const xp128f _x2 = xp128f::get(Je2.m_e.x());
    const xp128f _y2 = xp128f::get(Je2.m_e.y());
    Je2.m_Jcz.m_00_01_02_03() = _d2 * (_y2 * T2.r_20_21_22_x() - T2.r_10_11_12_x());
    Je2.m_Jcz.m_00_01_02_03().vstore_unalign(Je2.m_Jcz[1]);
    Je2.m_Jcz.m_00_01_02_03() = _d2 * (_x2 * T2.r_20_21_22_x() - T2.r_00_01_02_x());
#ifdef CFG_STEREO
    if (br) {
      const LA::AlignedVector3f bd2 = *br * _d2;
      const float x = Je2.m_e.x() - bd2.x(), y = Je2.m_e.y() - bd2.y(), z = 1.0f - bd2.z();
      Je2.m_Jcz[0][3] = Je2.m_e.x() * y;
      Je2.m_Jcz[0][4] = -(Je2.m_e.x() * x + z);
      Je2.m_Jcz[0][5] = y;
      Je2.m_Jcz[1][3] = Je2.m_e.y() * y + z;
      Je2.m_Jcz[1][4] = -Je2.m_e.y() * x;
      Je2.m_Jcz[1][5] = -x;
    } else
#endif
    {
      Je2.m_Jcz[0][3] = Je2.m_e.x() * Je2.m_e.y();
      Je2.m_Jcz[0][4] = -(Je2.m_e.x() * Je2.m_e.x() + 1.0f);
      Je2.m_Jcz[0][5] = Je2.m_e.y();
      Je2.m_Jcz[1][3] = Je2.m_e.y() * Je2.m_e.y() + 1.0f;
      Je2.m_Jcz[1][4] = -Je2.m_Jcz[0][3];
      Je2.m_Jcz[1][5] = -Je2.m_e.x();
    }
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcz[0][3], T2);
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcz[1][3], T2);
  } else {
    //Je2.m_Jd.MakeZero();
    Je2.m_Jcz.MakeZero();
  }
  Je2.m_e -= z2;
}
inline void GetErrorJacobian(const Rigid3D &T12, const Source &x1, const Depth::InverseGaussian &d1,
                             const Rigid3D &T2, const Point2D &z2, ErrorJacobian::DCXZ &Je2
#ifdef CFG_STEREO
                           , const Point3D *br = NULL
#endif
                           ) {
#ifdef CFG_DEBUG
  UT_ASSERT(z2.Valid());
#endif
  float d12, d2;
  LA::AlignedVector3f t;
  d1.Project(T12, x1.m_x, Je2.m_e, d12, d2, Je2.m_Jd, t);
  //const bool vp = fabs(d2) > DEPTH_EPSILON;
  //const bool vp = fabs(d2) > DEPTH_PROJECTION_MIN;
  //const bool vp = d2 > DEPTH_PROJECTION_MIN;
  const bool vp = d2 > DEPTH_PROJECTION_MIN && d2 < DEPTH_PROJECTION_MAX;
  if (vp) {
    const xp128f _d12 = xp128f::get(d12);
    const xp128f _d2 = xp128f::get(d2);
    const xp128f _x2 = xp128f::get(Je2.m_e.x());
    const xp128f _y2 = xp128f::get(Je2.m_e.y());

    Je2.m_Jcx.m_00_01_02_03() = _d2 * (T2.r_00_01_02_x() - _x2 * T2.r_20_21_22_x());
    Je2.m_Jcz.m_00_01_02_03() = _d2 * (T2.r_10_11_12_x() - _y2 * T2.r_20_21_22_x());
    Je2.m_Jcz.m_00_01_02_03().vstore_unalign(Je2.m_Jcx[1]);
#if 0
    Je2.m_Jcz.m_04_05_10_11() = _d12 * (_x2 * T12.r_20_21_22_x() - T12.r_00_01_02_x());
    Je2.m_Jcz.m_12_13_14_15() = _d12 * (_y2 * T12.r_20_21_22_x() - T12.r_10_11_12_x());

    Je2.m_Jcx[0][3] = Je2.m_Jcz[0][5] - Je2.m_Jcz[1][0] * x1.m_x.y();
    Je2.m_Jcx[0][4] = Je2.m_Jcz[1][0] * x1.m_x.x() - Je2.m_Jcz[0][4];
    Je2.m_Jcx[0][5] = Je2.m_Jcz[0][4] * x1.m_x.y() - Je2.m_Jcz[0][5] * x1.m_x.x();
    Je2.m_Jcx[1][3] = Je2.m_Jcz[1][3] - Je2.m_Jcz[1][4] * x1.m_x.y();
    Je2.m_Jcx[1][4] = Je2.m_Jcz[1][4] * x1.m_x.x() - Je2.m_Jcz[1][2];
    Je2.m_Jcx[1][5] = Je2.m_Jcz[1][2] * x1.m_x.y() - Je2.m_Jcz[1][3] * x1.m_x.x();
#else
    t *= _d12;
    Je2.m_Jcx[0][3] = -Je2.m_e.x() * t.y();
    Je2.m_Jcx[0][4] = Je2.m_e.x() * t.x() + t.z();
    Je2.m_Jcx[0][5] = -t.y();
    Je2.m_Jcx[1][3] = -(Je2.m_e.y() * t.y() + t.z());
    Je2.m_Jcx[1][4] = Je2.m_e.y() * t.x();
    Je2.m_Jcx[1][5] = t.x();
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcx[0][3], T2);
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcx[1][3], T2);
#endif
    const xp128f zero = xp128f::get(0.0f);
    (zero - Je2.m_Jcz.m_00_01_02_03()).vstore_unalign(Je2.m_Jcz[1]);
    Je2.m_Jcz.m_00_01_02_03() = zero - Je2.m_Jcx.m_00_01_02_03();
#ifdef CFG_STEREO
    if (br) {
      const LA::AlignedVector3f bd2 = *br * _d2;
      const float x = Je2.m_e.x() - bd2.x(), y = Je2.m_e.y() - bd2.y(), z = 1.0f - bd2.z();
      Je2.m_Jcz[0][3] = Je2.m_e.x() * y;
      Je2.m_Jcz[0][4] = -(Je2.m_e.x() * x + z);
      Je2.m_Jcz[0][5] = y;
      Je2.m_Jcz[1][3] = Je2.m_e.y() * y + z;
      Je2.m_Jcz[1][4] = -Je2.m_e.y() * x;
      Je2.m_Jcz[1][5] = -x;
    } else
#endif
    {
      Je2.m_Jcz[0][3] = Je2.m_e.x() * Je2.m_e.y();
      Je2.m_Jcz[0][4] = -(Je2.m_e.x() * Je2.m_e.x() + 1.0f);
      Je2.m_Jcz[0][5] = Je2.m_e.y();
      Je2.m_Jcz[1][3] = Je2.m_e.y() * Je2.m_e.y() + 1.0f;
      Je2.m_Jcz[1][4] = -Je2.m_Jcz[0][3];
      Je2.m_Jcz[1][5] = -Je2.m_e.x();
    }
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcz[0][3], T2);
    LA::AlignedMatrix3x3f::aTB(&Je2.m_Jcz[1][3], T2);
  } else {
    //Je2.m_Jd.MakeZero();
    Je2.m_Jcx.MakeZero();
    Je2.m_Jcz.MakeZero();
  }
  Je2.m_e -= z2;
}

template<int ME_FUNCTION, class LINEARIZATION, class FACTOR>
inline void GetFactor(const float w, const Rigid3D *T12, const Source &x1,
                      const Depth::InverseGaussian &d1, const Rigid3D &T2,
                      const Measurement &z2, LINEARIZATION *L, FACTOR *A,
#ifdef CFG_STEREO
                      const Point3D &br,
#endif
                      const float r2Max = FLT_MAX) {
#ifdef CFG_STEREO
  L->m_F = 0.0f;
  A->Initialize();
  if (z2.m_z.Valid()) {
    GetErrorJacobian(T12[0], x1, d1, T2, z2.m_z, L->m_Je);
    const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z2.m_W, L->m_Je.m_e);
    if (r2 > r2Max) {
      L->m_w = 0.0f;
    } else {
      L->m_w = w * ME::Weight<ME_FUNCTION>(r2);
    }
    L->m_F += L->m_w * r2;
    A->Accumulate(L->m_Je, L->m_w, z2.m_W);
  } else {
    L->m_Je.Invalidate();
  }
  if (z2.m_zr.Valid()) {
    GetErrorJacobian(T12[1], x1, d1, T2, z2.m_zr, L->m_Jer, &br);
    const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z2.m_Wr, L->m_Jer.m_e);
    if (r2 > r2Max) {
      L->m_wr = 0.0f;
    } else {
      L->m_wr = w * ME::Weight<ME_FUNCTION>(r2);
    }
    L->m_F += L->m_wr * r2;
    A->Accumulate(L->m_Jer, L->m_wr, z2.m_Wr);
  } else {
    L->m_Jer.Invalidate();
  }
#else
  GetErrorJacobian(*T12, x1, d1, T2, z2.m_z, L->m_Je);
  const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z2.m_W, L->m_Je.m_e);
  if (r2 > r2Max) {
    L->m_w = 0.0f;
  } else {
    L->m_w = w * ME::Weight<ME_FUNCTION>(r2);
  }
  L->m_F = L->m_w * r2;
  A->Set(L->m_Je, L->m_w, z2.m_W);
#endif
}
template<int ME_FUNCTION>
inline void GetFactor(const float w, const Rigid3D *T12, const Source &x1,
                      const Depth::InverseGaussian &d1, const Rigid3D &T2,
                      const Measurement &z2, Factor::Depth *A, Factor::Depth::U *U,
#ifdef CFG_STEREO
                      const Point3D &br,
#endif
                      const float r2Max = FLT_MAX) {
  GetFactor<ME_FUNCTION, Factor::Depth, Factor::Depth::U>(w, T12, x1, d1, T2, z2, A, U,
#ifdef CFG_STEREO
                                                          br,
#endif
                                                          r2Max);
  A->m_add = U->m_A;
}
template<int ME_FUNCTION>
inline void GetFactor(const float w, const Rigid3D *T12, const Source &x1,
                      const Depth::InverseGaussian &d1, const Rigid3D &T2,
                      const Measurement &z2, Factor::FixSource::L *L,
                      Factor::FixSource::A1 *A1, Factor::FixSource::A2 *A2,
                      Factor::FixSource::U *U,
#ifdef CFG_STEREO
                      const Point3D &br,
#endif
                      const float r2Max = FLT_MAX) {
  GetFactor<ME_FUNCTION, Factor::FixSource::L, Factor::FixSource::U>(w, T12, x1, d1, T2, z2, L, U,
#ifdef CFG_STEREO
                                                                     br,
#endif
                                                                     r2Max);
  U->m_A.Get(A2->m_add.m_a, A1->m_adcz, A2->m_add.m_b, A2->m_Aczz.m_A, A2->m_Aczz.m_b);
}
template<int ME_FUNCTION>
inline void GetFactor(const float w, const Rigid3D *T12, const Source &x1,
                      const Depth::InverseGaussian &d1, const Rigid3D &T2,
                      const Measurement &z2, Factor::Full::L *L, Factor::Full::A1 *A1,
                      Factor::Full::A2 *A2, Factor::Full::U *U,
#ifdef CFG_STEREO
                      const Point3D &br,
#endif
                      const float r2Max = FLT_MAX) {
  GetFactor<ME_FUNCTION, Factor::Full::L, Factor::Full::U>(w, T12, x1, d1, T2, z2, L, U,
#ifdef CFG_STEREO
                                                           br,
#endif
                                                           r2Max);
  U->m_A.Get(A2->m_adx.m_add.m_a, A2->m_adx.m_adc, A1->m_adcz, A2->m_adx.m_add.m_b,
             A2->m_Acxx.m_A, A2->m_Acxz, A2->m_Acxx.m_b, A2->m_Aczz.m_A, A2->m_Aczz.m_b);
}

template<class LINEARIZATION>
inline float GetCost(const LINEARIZATION &L, const Measurement &z, const Error &e) {
#ifdef CFG_STEREO
  float F = 0.0f;
  if (z.m_z.Valid()) {
    const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_e);
    F += L.m_w * r2;
  }
  if (z.m_zr.Valid()) {
    const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_Wr, e.m_er);
    F += L.m_wr * r2;
  }
#else
  const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, e.m_e);
  const float F = L.m_w * r2;
#endif
  return F;
}
inline float GetCost(const Factor::Depth &A, const Measurement &z, const float xd, Error &e) {
  GetError(A, xd, e);
  return GetCost(A, z, e);
}
inline float GetCost(const Factor::FixSource::L &L, const Measurement &z, 
                     const LA::ProductVector6f *xcz, const float *xd, Error &e) {
  GetError(L, xcz, xd, e);
  return GetCost(L, z, e);
}
inline float GetCost(const Factor::Full::L &L, const Measurement &z,
                     const LA::ProductVector6f *xcx, const LA::ProductVector6f *xcz,
                     const float *xd, Error &e) {
  GetError(L, xcx, xcz, xd, e);
  return GetCost(L, z, e);
}

inline void GetReduction(const Factor::Depth &A, const Rigid3D *T12, const Source &x1,
                         const Depth::InverseGaussian &d1, const Measurement &z2, const float xd,
                         Reduction &Ra, Reduction &Rp) {
  GetError(T12, x1, d1, z2, Ra.m_e);
  GetError(A, xd, Rp.m_e);
  Ra.m_dF = A.m_F - (Ra.m_F = GetCost(A, z2, Ra.m_e));
  Rp.m_dF = A.m_F - (Rp.m_F = GetCost(A, z2, Rp.m_e));
}
inline void GetReduction(const Factor::FixSource::L &L, const Rigid3D *T12, const Source &x1,
                         const Depth::InverseGaussian &d1, const Measurement &z2,
                         const LA::ProductVector6f *xcz, const float *xd,
                         Reduction &Ra, Reduction &Rp) {
  GetError(T12, x1, d1, z2, Ra.m_e);
  GetError(L, xcz, xd, Rp.m_e);
  Ra.m_dF = L.m_F - (Ra.m_F = GetCost(L, z2, Ra.m_e));
  Rp.m_dF = L.m_F - (Rp.m_F = GetCost(L, z2, Rp.m_e));
}
inline void GetReduction(const Factor::Full::L &L, const Rigid3D *T12, const Source &x1,
                         const Depth::InverseGaussian &d1, const Measurement &z2,
                         const LA::ProductVector6f *xcx, const LA::ProductVector6f *xcz, const float *xd,
                         Reduction &Ra, Reduction &Rp) {
  GetError(T12, x1, d1, z2, Ra.m_e);
  GetError(L, xcx, xcz, xd, Rp.m_e);
  Ra.m_dF = L.m_F - (Ra.m_F = GetCost(L, z2, Ra.m_e));
  Rp.m_dF = L.m_F - (Rp.m_F = GetCost(L, z2, Rp.m_e));
}

#ifdef CFG_STEREO
#ifdef CFG_DEBUG
inline void DebugSetMeasurement(const Point3D &br, const Depth::InverseGaussian &d, Source &x) {
  UT_ASSERT(x.m_xr.Valid());
  d.Project(br, x.m_x, x.m_xr);
}
#endif
inline void GetError(const Point3D &br, const Depth::InverseGaussian &d, const Source &x,
                     LA::Vector2f &e) {
  d.Project(br, x.m_x, e);
  e -= x.m_xr;
}
inline void GetError(const Factor::Stereo &A, const float xd, LA::Vector2f &e) {
  GetError(A.m_Je, xd, e);
}
inline void GetErrorJacobian(const Point3D &br, const Depth::InverseGaussian &d, const Source &x,
                             ErrorJacobian::D &Je) {
  d.Project(br, x.m_x, Je.m_e, Je.m_Jd);
  Je.m_e -= x.m_xr;
}
template<int ME_FUNCTION>
inline void GetFactor(const float w, const Point3D &br, const Depth::InverseGaussian &d, const Source &x,
                      Factor::Stereo *A, Factor::Stereo::U *U) {
  GetErrorJacobian(br, d, x, A->m_Je);
  const float r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, A->m_Je.m_e);
  A->m_w = w * ME::Weight<ME_FUNCTION>(r2);
  A->m_F = A->m_w * r2;
  U->Initialize();
  U->Accumulate(A->m_Je, A->m_w, x.m_Wr);
  A->m_add = U->m_A;
}
inline float GetCost(const Factor::Stereo &A, const Source &x, const LA::Vector2f &e) {
  return A.m_w * LA::SymmetricMatrix2x2f::MahalanobisDistance(x.m_Wr, e);
}
inline float GetCost(const Factor::Stereo &A, const Source &x, const float xd, LA::Vector2f &e) {
  GetError(A, xd, e);
  return GetCost(A, x, e);
}
inline void GetReduction(const Factor::Stereo &A, const Point3D &br, 
                         const Depth::InverseGaussian &d, const Source &x,
                         const float xd, Reduction &Ra, Reduction &Rp) {
  GetError(br, d, x, Ra.m_e.m_er);
  GetError(A, xd, Rp.m_e.m_er);
  Ra.m_dF = A.m_F - (Ra.m_F = GetCost(A, x, Ra.m_e.m_er));
  Rp.m_dF = A.m_F - (Rp.m_F = GetCost(A, x, Rp.m_e.m_er));
}
#endif


#ifdef CFG_DEBUG_EIGEN
class EigenErrorJacobian {
#ifdef CFG_STEREO
 public:
  class Stereo {
   public:
    inline void operator = (const ErrorJacobian::D &Je) {
      m_Jd = Je.m_Jd;
      m_e = Je.m_e;
    }
    inline void AssertEqual(const ErrorJacobian::D &Je,
                            const int verbose = 1, const std::string str = "",
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      m_Jd.AssertEqual(Je.m_Jd, verbose, str + ".m_Jd", epsAbs, epsRel);
      m_e.AssertEqual(Je.m_e, verbose, str + ".m_e", epsAbs, epsRel);
    }
   public:
    EigenVector2f m_Jd, m_e;
  };
#endif
 public:
  inline void Set(const ErrorJacobian::D &Je) {
    m_Jd = Je.m_Jd;
    m_Jcx.setZero();
    m_Jcz.setZero();
    m_e = Je.m_e;
  }
  inline void Set(const ErrorJacobian::DCZ &Je) {
    m_Jd = Je.m_Jd;
    m_Jcx.setZero();
    m_Jcz = Je.m_Jcz;
    m_e = Je.m_e;
  }
  inline void Set(const ErrorJacobian::DCXZ &Je) {
    m_Jd = Je.m_Jd;
    m_Jcx = Je.m_Jcx;
    m_Jcz = Je.m_Jcz;
    m_e = Je.m_e;
  }
  inline void AssertEqual(const ErrorJacobian::D &Je,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    m_Jd.AssertEqual(Je.m_Jd, verbose, str + ".m_Jd", epsAbs, epsRel);
    m_Jcx.AssertZero(verbose, str + ".m_Jcx", -1.0f, -1.0f);
    m_Jcz.AssertZero(verbose, str + ".m_Jcz", -1.0f, -1.0f);
    m_e.AssertEqual(Je.m_e, verbose, str + ".m_e", epsAbs, epsRel);
  }
  inline void AssertEqual(const ErrorJacobian::DCZ &Je,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    m_Jd.AssertEqual(Je.m_Jd, verbose, str + ".m_Jd", epsAbs, epsRel);
    m_Jcx.AssertZero(verbose, str + ".m_Jcx", -1.0f, -1.0f);
    m_Jcz.AssertEqual(Je.m_Jcz, verbose, str + ".m_Jcz", epsAbs, epsRel);
    m_e.AssertEqual(Je.m_e, verbose, str + ".m_e", epsAbs, epsRel);
  }
  inline void AssertEqual(const ErrorJacobian::DCXZ &Je,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    m_Jd.AssertEqual(Je.m_Jd, verbose, str + ".m_Jd", epsAbs, epsRel);
    m_Jcx.AssertEqual(Je.m_Jcx, verbose, str + ".m_Jcx", epsAbs, epsRel);
    m_Jcz.AssertEqual(Je.m_Jcz, verbose, str + ".m_Jcz", epsAbs, epsRel);
    m_e.AssertEqual(Je.m_e, verbose, str + ".m_e", epsAbs, epsRel);
  }
 public:
  EigenVector2f m_Jd;
  EigenMatrix2x6f m_Jcx, m_Jcz;
  EigenVector2f m_e;
};
class EigenFactor {
 public:
  typedef Factor::DD DD;
  class DC : public Eigen::Matrix<float, 1, 6> {
   public:
    inline DC() : Eigen::Matrix<float, 1, 6>() {}
    inline DC(const Eigen::Matrix<float, 1, 6> &a) : Eigen::Matrix<float, 1, 6>(a) {}
    inline DC(const Factor::DC &a) : Eigen::Matrix<float, 1, 6>(EigenVector6f(a).transpose()) {}
    inline void operator = (const Eigen::Matrix<float, 1, 6> &a) { *((Eigen::Matrix<float, 1, 6> *) this) = a; }
    inline void operator = (const Factor::DC &a) { *this = EigenVector6f(a).transpose(); }
    inline void operator += (const Factor::DC &a) { *((Eigen::Matrix<float, 1, 6> *) this) += DC(a); }
    inline void operator += (const DC &a) { *((Eigen::Matrix<float, 1, 6> *) this) += a; }
    EigenVector6f GetTranspose() const { return EigenVector6f(*this); }
    LA::Vector6f GetVector6f() const { return GetTranspose().GetVector6f(); }
  };
  class DDC {
   public:
    inline DDC() { }
    inline ~DDC() { }
    inline DDC(const float add, const DC &adc, const float bd) {
      m_add.Set(add, bd);
      m_adc = adc;
    }
    inline void operator = (const Factor::DDC &a) {
      m_add = a.m_add;
      m_adc = a.m_adc;
    }
    inline void operator += (const DDC &a) {
      m_add += a.m_add;
      m_adc += a.m_adc;
    }
    inline void MakeZero() { m_add.MakeZero(); m_adc.setZero(); }
    inline bool AssertEqual(const Factor::DDC &a,
                            const int verbose = 1, const std::string str = "", 
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      return m_add.AssertEqual(a.m_add, verbose, str + ".m_add", epsAbs, epsRel) &&
             Factor::DC::Get(m_adc.GetVector6f()).AssertEqual(a.m_adc, verbose, str + ".m_adc",
                                                              epsAbs, epsRel);
    }
   public:
    DD m_add;
    DC m_adc;
  };
#ifdef CFG_STEREO
  class Stereo {
   public:
    inline Stereo() {}
    inline Stereo(const float F, const float add, const float bd) {
      m_F = F;
      m_add = add;
      m_bd = bd;
    }
    inline void operator = (const Factor::Stereo &A) {
      m_F = A.m_F;
      m_add = A.m_add.m_a;
      m_bd = A.m_add.m_b;
    }
    inline void AssertEqual(const Factor::Stereo &A,
                            const int verbose = 1, const std::string str = "", 
                            const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
      UT::AssertEqual(m_F, A.m_F, verbose, str + ".m_F", epsAbs, epsRel);
      UT::AssertEqual(m_add, A.m_add.m_a, verbose, str + ".m_add", epsAbs, epsRel);
      UT::AssertEqual(m_bd, A.m_add.m_b, verbose, str + ".m_bd", epsAbs, epsRel);
    }
   public:
    float m_F, m_add, m_bd;
  };
#endif
 public:
  inline EigenFactor() {}
  inline EigenFactor(const float F, const Eigen::Matrix<float, 13, 14, Eigen::RowMajor> &A) {
    m_F = F;
    m_add = A(0, 0);
    m_adcx = A.block<1, 6>(0, 1);
    m_adcz = A.block<1, 6>(0, 7);
    m_bd = A(0, 13);
    m_Acxx = A.block<6, 6>(1, 1);
    m_Acxz = A.block<6, 6>(1, 7);
    m_bcx = A.block<6, 1>(1, 13);
    m_Aczz = A.block<6, 6>(7, 7);
    m_bcz = A.block<6, 1>(7, 13);
  }
  inline void Set(const Factor::Depth &A) {
    m_F = A.m_F;
    m_add = A.m_add.m_a;
    m_adcx.setZero();
    m_adcz.setZero();
    m_bd = A.m_add.m_b;
    m_Acxx.setZero();
    m_Acxz.setZero();
    m_bcx.setZero();
    m_Aczz.setZero();
    m_bcz.setZero();
  }
  inline void Set(const Factor::FixSource::L &L, const Factor::FixSource::A1 &A1,
                  const Factor::FixSource::A2 &A2) {
    m_F = L.m_F;
    m_add = A2.m_add.m_a;
    m_adcx.setZero();
    m_adcz = A1.m_adcz;
    m_bd = A2.m_add.m_b;
    m_Acxx.setZero();
    m_Acxz.setZero();
    m_bcx.setZero();
    m_Aczz = A2.m_Aczz.m_A;
    m_bcz = A2.m_Aczz.m_b;
  }
  inline void Set(const Factor::Full::L &L, const Factor::Full::A1 &A1,
                  const Factor::Full::A2 &A2) {
    m_F = L.m_F;
    m_add = A2.m_adx.m_add.m_a;
    m_adcx = A2.m_adx.m_adc;
    m_adcz = A1.m_adcz;
    m_bd = A2.m_adx.m_add.m_b;
    m_Acxx = A2.m_Acxx.m_A;
    m_Acxz = A2.m_Acxz;
    m_bcx = A2.m_Acxx.m_b;
    m_Aczz = A2.m_Aczz.m_A;
    m_bcz = A2.m_Aczz.m_b;
  }
  inline void operator *= (const float s) {
    m_F *= s;
    m_add *= s;
    m_adcx *= s;
    m_adcz *= s;
    m_bd *= s;
    m_Acxx *= s;
    m_Acxz *= s;
    m_bcx *= s;
    m_Aczz *= s;
    m_bcz *= s;
  }
  inline void AssertEqual(const Factor::Depth &A,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    UT::AssertEqual(m_F, A.m_F, verbose, str + ".m_F");
    Factor::DD::Get(m_add, m_bd).AssertEqual(A.m_add, verbose, str + ".m_add", epsAbs, epsRel);
    Factor::DC::Get(m_adcx.GetVector6f()).AssertZero(verbose, str + ".m_adcx");
    Factor::DC::Get(m_adcz.GetVector6f()).AssertZero(verbose, str + ".m_adcz");
    Camera::Factor::Unitary::CC::Get(m_Acxx.GetSymmetricMatrix6x6f(),
                                     m_bcx.GetVector6f()).AssertZero(verbose, str + ".m_Acxx");
    Camera::Factor::Binary::CC(m_Acxz.GetAlignedMatrix6x6f()).AssertZero(verbose, str + ".m_Acxz");
    Camera::Factor::Unitary::CC::Get(m_Aczz.GetSymmetricMatrix6x6f(),
                                     m_bcz.GetVector6f()).AssertZero(verbose, str + ".m_Aczz");
  }
  inline void AssertEqual(const Factor::FixSource::L &L, const Factor::FixSource::A1 &A1,
                          const Factor::FixSource::A2 &A2,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    UT::AssertEqual(m_F, L.m_F, verbose, str + ".m_F");
    Factor::DD::Get(m_add, m_bd).AssertEqual(A2.m_add, verbose, str + ".m_add", epsAbs, epsRel);
    Factor::DC::Get(m_adcx.GetVector6f()).AssertZero(verbose, str + ".m_adcx");
    Factor::DC::Get(m_adcz.GetVector6f()).AssertEqual(A1.m_adcz, verbose, str + ".m_adcz",
                                                      epsAbs, epsRel);
    Camera::Factor::Unitary::CC::Get(m_Acxx.GetSymmetricMatrix6x6f(),
                                     m_bcx.GetVector6f()).AssertZero(verbose, str + ".m_Acxx");
    Camera::Factor::Binary::CC(m_Acxz.GetAlignedMatrix6x6f()).AssertZero(verbose, str + ".m_Acxz");
    const LA::SymmetricMatrix6x6f Aczz = m_Aczz.GetSymmetricMatrix6x6f();
    const LA::Vector6f bcz = m_bcz.GetVector6f();
    Camera::Factor::Unitary::CC::Get(Aczz, bcz).AssertEqual(A2.m_Aczz, verbose, str + ".m_Aczz",
                                                            epsAbs, epsRel);
  }
  inline void AssertEqual(const Factor::Full::L &L, const Factor::Full::A1 &A1,
                          const Factor::Full::A2 &A2,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    UT::AssertEqual(m_F, L.m_F, verbose, str + ".m_F");
    Factor::DD::Get(m_add, m_bd).AssertEqual(A2.m_adx.m_add, verbose, str + ".m_add",
                                             epsAbs, epsRel);
    Factor::DC::Get(m_adcx.GetVector6f()).AssertEqual(A2.m_adx.m_adc, verbose, str + ".m_adcx",
                                                      epsAbs, epsRel);
    Factor::DC::Get(m_adcz.GetVector6f()).AssertEqual(A1.m_adcz, verbose, str + ".m_adcz",
                                                      epsAbs, epsRel);
    const LA::SymmetricMatrix6x6f Acxx = m_Acxx.GetSymmetricMatrix6x6f();
    const LA::Vector6f bcx = m_bcx.GetVector6f();
    Camera::Factor::Unitary::CC::Get(Acxx, bcx).AssertEqual(A2.m_Acxx, verbose, str + ".m_Acxx",
                                                            epsAbs, epsRel);
    const LA::AlignedMatrix6x6f Acxz = m_Acxz.GetAlignedMatrix6x6f();
    Camera::Factor::Binary::CC(Acxz).AssertEqual(A2.m_Acxz, verbose, str + ".m_Acxz",
                                                 epsAbs, epsRel);
    const LA::SymmetricMatrix6x6f Aczz = m_Aczz.GetSymmetricMatrix6x6f();
    const LA::Vector6f bcz = m_bcz.GetVector6f();
    Camera::Factor::Unitary::CC::Get(Aczz, bcz).AssertEqual(A2.m_Aczz, verbose, str + ".m_Aczz",
                                                            epsAbs, epsRel);
  }
 public:
  float m_F;
  float m_add;
  DC m_adcx, m_adcz;
  float m_bd;
  EigenMatrix6x6f m_Acxx, m_Acxz;
  EigenVector6f m_bcx;
  EigenMatrix6x6f m_Aczz;
  EigenVector6f m_bcz;
};
EigenErrorJacobian EigenGetErrorJacobian(const Rigid3D &C1, const Source &x1, const Depth::InverseGaussian &d1, 
                                         const Rigid3D &C2, const Point2D &z2, const bool cx, const bool cz
#ifdef CFG_STEREO
                                       , const Point3D *br = NULL
#endif
                                       );
template<int ME_FUNCTION>
inline EigenFactor EigenGetFactor(const float w, const Rigid3D &C1, const Source &x1,
                                  const Depth::InverseGaussian &d1, const Rigid3D &C2,
                                  const Measurement &z2, const bool cx, const bool cz
#ifdef CFG_STEREO
                                , const Point3D &br
#endif
                                ) {
#ifdef CFG_STEREO
  float F = 0.0f;
  EigenMatrix13x14f e_A;
  e_A.setZero();
  if (z2.m_z.Valid()) {
    const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_z, cx, cz);
    const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_W);
    const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
    const float _w = w * ME::Weight<ME_FUNCTION>(r2);
    const EigenMatrix2x13f e_J = EigenMatrix2x13f(e_Je.m_Jd, e_Je.m_Jcx, e_Je.m_Jcz);
    const EigenMatrix2x13f e_WJ = EigenMatrix2x13f(_w * e_W * e_J);
    F += _w * r2;
    e_A += e_WJ.transpose() * EigenMatrix2x14f(e_J, e_Je.m_e);
  }
  if (z2.m_zr.Valid()) {
    const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_zr, cx, cz, &br);
    const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_Wr);
    const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
    const float _w = w * ME::Weight<ME_FUNCTION>(r2);
    const EigenMatrix2x13f e_J = EigenMatrix2x13f(e_Je.m_Jd, e_Je.m_Jcx, e_Je.m_Jcz);
    const EigenMatrix2x13f e_WJ = EigenMatrix2x13f(_w * e_W * e_J);
    F += _w * r2;
    e_A += e_WJ.transpose() * EigenMatrix2x14f(e_J, e_Je.m_e);
  }
#else
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_z, cx, cz);
  const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_W);
  const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
  const float _w = w * ME::Weight<ME_FUNCTION>(r2);
  const EigenMatrix2x13f e_J = EigenMatrix2x13f(e_Je.m_Jd, e_Je.m_Jcx, e_Je.m_Jcz);
  const EigenMatrix2x13f e_WJ = EigenMatrix2x13f(_w * e_W * e_J);
  const float F = _w * r2;
  const EigenMatrix13x14f e_A = EigenMatrix13x14f(e_WJ.transpose() *
                                EigenMatrix2x14f(e_J, e_Je.m_e));
#endif
  return EigenFactor(F, e_A);
}
template<int ME_FUNCTION>
inline float EigenGetCost(const float w, const Rigid3D &C1, const Source &x1,
                          const Depth::InverseGaussian &d1, const Rigid3D &C2,
                          const Measurement &z2, const EigenVector6f *e_xcx,
                          const EigenVector6f *e_xcz, const float xd
#ifdef CFG_STEREO
                        , const Point3D &br
#endif
                        ) {
#ifdef CFG_STEREO
  float F = 0.0f;
  if (z2.m_z.Valid()) {
    const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_z,
                                                          e_xcx != NULL, e_xcz != NULL);
    const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_W);
    const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
    const float _w = w * ME::Weight<ME_FUNCTION>(r2);
    EigenVector2f e_e = e_Je.m_e;
    if (e_xcx) {
      e_e += e_Je.m_Jcx * *e_xcx;
    }
    if (e_xcz) {
      e_e += e_Je.m_Jcz * *e_xcz;
    }
    e_e += e_Je.m_Jd * xd;
    F += _w * (e_W * e_e).dot(e_e);
  }
  if (z2.m_zr.Valid()) {
    const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_zr,
                                                          e_xcx != NULL, e_xcz != NULL, &br);
    const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_Wr);
    const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
    const float _w = w * ME::Weight<ME_FUNCTION>(r2);
    EigenVector2f e_e = EigenVector2f(e_Je.m_e + e_Je.m_Jd * xd);
    if (e_xcx) {
      e_e += e_Je.m_Jcx * *e_xcx;
    }
    if (e_xcz) {
      e_e += e_Je.m_Jcz * *e_xcz;
    }
    F += _w * (e_W * e_e).dot(e_e);
  }
#else
  const EigenErrorJacobian e_Je = EigenGetErrorJacobian(C1, x1, d1, C2, z2.m_z,
                                                        e_xcx != NULL, e_xcz != NULL);
  const EigenMatrix2x2f e_W = EigenMatrix2x2f(z2.m_W);
  const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
  const float _w = w * ME::Weight<ME_FUNCTION>(r2);
  EigenVector2f e_e = EigenVector2f(e_Je.m_e + e_Je.m_Jd * xd);
  if (e_xcx) {
    e_e += e_Je.m_Jcx * *e_xcx;
  }
  if (e_xcz) {
    e_e += e_Je.m_Jcz * *e_xcz;
  }
  const float F = _w * (e_W * e_e).dot(e_e);
#endif
  return F;
}
#ifdef CFG_STEREO
EigenErrorJacobian::Stereo EigenGetErrorJacobian(const Point3D &br,
                                                 const Depth::InverseGaussian &d,
                                                 const Source &x);
template<int ME_FUNCTION>
inline EigenFactor::Stereo EigenGetFactor(const float w, const Point3D &br, 
                                          const Depth::InverseGaussian &d, const Source &x) {
  const EigenErrorJacobian::Stereo e_Je = EigenGetErrorJacobian(br, d, x);
  const EigenMatrix2x2f e_W = EigenMatrix2x2f(x.m_Wr);
  const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
  const float _w = w * ME::Weight<ME_FUNCTION>(r2);
  const EigenVector2f e_WJ(_w * e_W * e_Je.m_Jd);
  const float add = e_WJ.dot(e_Je.m_Jd), bd = e_WJ.dot(e_Je.m_e);
  const float F = _w * r2;
  return EigenFactor::Stereo(F, add, bd);
}
template<int ME_FUNCTION>
inline float EigenGetCost(const float w, const Point3D &br, const Depth::InverseGaussian &d,
                          const Source &x, const float xd) {
  const EigenErrorJacobian::Stereo e_Je = EigenGetErrorJacobian(br, d, x);
  const EigenMatrix2x2f e_W = EigenMatrix2x2f(x.m_Wr);
  const float r2 = (e_W * e_Je.m_e).dot(e_Je.m_e);
  const float _w = w * ME::Weight<ME_FUNCTION>(r2);
  const EigenVector2f e_e = EigenVector2f(e_Je.m_e + e_Je.m_Jd * xd);
  return _w * (e_W * e_e).dot(e_e);
}
#endif
#endif
}
#endif
