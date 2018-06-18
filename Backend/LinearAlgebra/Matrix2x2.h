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
#ifndef _MATRIX_2x2_H_
#define _MATRIX_2x2_H_

#include "Vector2.h"
#include "LinearSystem.h"

namespace LA {

class AlignedMatrix2x2f {

 public:

  inline const xp128f& m_00_01_10_11() const { return m_data; }   inline xp128f& m_00_01_10_11() { return m_data; }

  inline const float& m00() const { return m_data[0]; } inline float& m00() { return m_data[0]; }
  inline const float& m01() const { return m_data[1]; } inline float& m01() { return m_data[1]; }
  inline const float& m10() const { return m_data[2]; } inline float& m10() { return m_data[2]; }
  inline const float& m11() const { return m_data[3]; } inline float& m11() { return m_data[3]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { Scale(s); }

  inline const float& operator() (int row, int col) const {
    return m_data[row * 2 + col];
  }

  inline float& operator() (int row, int col) {
    return m_data[row * 2 + col];
  }
  inline AlignedMatrix2x2f operator - (const AlignedMatrix2x2f &B) const {
    AlignedMatrix2x2f AmB;
    AmB.m_00_01_10_11() = m_00_01_10_11() - B.m_00_01_10_11();
    return AmB;
  }
  inline Vector2f operator * (const Vector2f &b) const {
    Vector2f Ab;
    AlignedMatrix2x2f::Ab(*this, b, Ab);
    return Ab;
  }

  inline void SetDiagonal(const Vector2f &d) { m00() = d.v0(); m11() = d.v1(); }
  inline void GetDiagonal(Vector2f &d) const { d.v0() = m00(); d.v1() = m11(); }
  inline void ScaleDiagonal(const float s) { m00() *= s; m11() *= s; }
  inline void IncreaseDiagonal(const float d) { m00() = d + m00(); m11() = d + m11(); }
  inline void IncreaseDiagonal(const float d0, const float d1) { m00() = d0 + m00(); m11() = d1 + m11(); }
  inline void SetLowerFromUpper() { m10() = m01(); }
  inline void GetTranspose(AlignedMatrix2x2f &MT) const {
    MT.m_00_01_10_11().vset_all_lane(m00(), m10(), m01(), m11());
  }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x2f)); }
  inline void MakeIdentity() {
    m_00_01_10_11().vset_all_lane(1.0f, 0.0f, 0.0f, 1.0f);
  }

  inline bool Valid() const { return m00() != FLT_MAX; }
  inline bool Invalid() const { return m00() == FLT_MAX; }
  inline void Invalidate() { m00() = FLT_MAX; }

  inline void Scale(const float s) {
    xp128f _s; _s.vdup_all_lane(s);
    Scale(_s);
  }
  inline void Scale(const xp128f &s) {
    m_00_01_10_11() *= s;
  }
  inline float Determinant() const { return m00() * m11() - m01() * m10(); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e\n", m00(), m01());
      UT::Print("%e %e\n", m10(), m11());
    } else {
      UT::Print("%f %f\n", m00(), m01());
      UT::Print("%f %f\n", m10(), m11());
    }
  }
  inline bool AssertEqual(const AlignedMatrix2x2f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 4, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix2x2f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m00(), 4, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  static inline void Ab(const AlignedMatrix2x2f &A, const Vector2f &b, Vector2f &Ab) {
    const xp128f t = A.m_00_01_10_11() * xp128f::get(b.v0(), b.v1(), b.v0(), b.v1());
    Ab.v0() = t[0] + t[1];
    Ab.v1() = t[2] + t[3];
  }
  static inline void AddAbTo(const AlignedMatrix2x2f &A, const float *b, Vector2f &Ab) {
    const xp128f t = A.m_00_01_10_11() * xp128f::get(b[0], b[1], b[0], b[1]);
    Ab.v0() += t[0] + t[1];
    Ab.v1() += t[2] + t[3];
  }

  static inline void ABT(const AlignedMatrix2x2f &A, const AlignedMatrix2x2f &B, AlignedMatrix2x2f &ABT) {
    const xp128f t = A.m_00_01_10_11() * B.m_00_01_10_11();

    ABT.m00() = t[0] + t[1];
    ABT.m11() = t[2] + t[3];

    ABT.m01() = A.m00() * B.m10() + A.m01() * B.m11();
    ABT.m10() = A.m10() * B.m00() + A.m11() * B.m01();

  }

 protected:

  xp128f m_data;
};

template<typename TYPE> class SymmetricMatrix2x2 {

 public:

  inline SymmetricMatrix2x2() {}
  inline SymmetricMatrix2x2(const TYPE m00, const TYPE m01, const TYPE m11) { Set(m00, m01, m11); }

  inline const TYPE& m00() const { return m_data[0]; }  inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }  inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m10() const { return m_data[1]; }
  inline const TYPE& m11() const { return m_data[2]; }  inline TYPE& m11() { return m_data[2]; }

  inline bool operator == (const SymmetricMatrix2x2<TYPE> &M) const {
    return m00() == M.m00() && m01() == M.m01() && m11() == M.m11();
  }
  inline void operator += (const SymmetricMatrix2x2<TYPE> &M) {
    m00() = M.m00() + m00();
    m01() = M.m01() + m01();
    m11() = M.m11() + m11();
  }
  inline void operator *= (const TYPE s) { Scale(s); }
  inline void operator *= (const SymmetricMatrix2x2<TYPE> &s) {
    m00() *= s.m00();
    m01() *= s.m01();
    m11() *= s.m11();
  }
  inline void operator /= (const TYPE d) { Scale(1 / d); }
  inline SymmetricMatrix2x2<TYPE> operator - (const SymmetricMatrix2x2<TYPE> &B) const {
    SymmetricMatrix2x2<TYPE> AmB;
    AmB.m00() = m00() - B.m00();
    AmB.m01() = m01() - B.m01();
    AmB.m11() = m11() - B.m11();
  }
  inline SymmetricMatrix2x2<TYPE> operator * (const TYPE s) const {
    return SymmetricMatrix2x2<TYPE>(m00() * s, m01() * s, m11() * s);
  }
  inline Vector2<TYPE> operator * (const Vector2<TYPE> &b) const {
    Vector2<TYPE> _Ab;
    Ab(*this, b, _Ab);
    return _Ab;
  }

  inline void Set(const TYPE M[2][2]) { m00() = M[0][0]; m01() = M[0][1]; m11() = M[1][1]; }
  inline void Set(const TYPE m00, const TYPE m01, const TYPE m11) { this->m00() = m00; this->m01() = m01; this->m11() = m11; }
  inline void Get(SymmetricMatrix2x2<TYPE> &M) const { M = *this; }
  inline void Get(TYPE M[2][2]) const { M[0][0] = m00(); M[0][1] = M[1][0] = m01(); M[1][1] = m11(); }
  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix2x2<TYPE>)); }
  inline void MakeIdentity() { m00() = 1; m01() = 0; m11() = 1; }
  inline void MakeDiagonal(const TYPE d) { m00() = m11() = d; m01() = 0.0f; }
  inline void IncreaseDiagonal(const TYPE d) { m00() = d + m00(); m11() = d + m11(); }
  inline TYPE Determinant() const { return m00() * m11() - m01() * m01(); }
  inline void Scale(const TYPE s) { m00() *= s; m01() *= s; m11() *= s; }
  inline void GetScaled(const TYPE s, SymmetricMatrix2x2<TYPE> &M) const {
    M.m00() = m00() * s;
    M.m01() = m01() * s;
    M.m11() = m11() * s;
  }
  inline bool GetInverse(SymmetricMatrix2x2<TYPE> &MI) const {
    const TYPE d = Determinant();
    if (UT::IsNAN<TYPE>(d) || d <= 0) {
      MI.Invalidate();
      return false;
    }
    MI.m11() = 1 / d;
    MI.m00() =  m11() * MI.m11();
    MI.m01() = -m01() * MI.m11();
    MI.m11() =  m00() * MI.m11();
    return true;
  }
  inline bool GetInverse(AlignedMatrix2x2f &MI) const {
    const TYPE d = Determinant();
    if (UT::IsNAN<TYPE>(d) || d <= 0) {
      MI.Invalidate();
      return false;
    }
    MI.m11() = 1 / d;
    MI.m00() = m11() * MI.m11();
    MI.m01() = MI.m10() = -m01() * MI.m11();
    MI.m11() = m00() * MI.m11();
    return true;
  }
  inline SymmetricMatrix2x2<TYPE> GetInverse() const {
    SymmetricMatrix2x2<TYPE> MI;
    GetInverse(MI);
    return MI;
  }
  inline bool GetInverseLDL(SymmetricMatrix2x2<TYPE> &MI, const TYPE *eps) const {
    TYPE M[2][2];
    Get(M);
    TYPE *_M[2] = {M[0], M[1]};
    if (LA::LS::InverseLDL(2, _M, eps)) {
      MI.Set(M);
      return true;
    } else {
      MI.Invalidate();
      return false;
    }
  }
  inline void MakeMinus() { m00() = -m00(); m01() = -m01(); m11() = -m11(); }

  inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }
  inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }

  inline void SaveB(FILE *fp) const { UT::SaveB(*this, fp); }
  inline void LoadB(FILE *fp) { UT::LoadB(*this, fp); }
  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e\n", m00(), m01());
      UT::Print("%e %e\n", m01(), m11());
    } else {
      UT::Print("%f %f\n", m00(), m01());
      UT::Print("%f %f\n", m01(), m11());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    if (e) {
      UT::Print("%s%e %e\n",  str.c_str(), m00(), m01());
      UT::Print("%s%e %e\n", _str.c_str(), m10(), m11());
    } else {
      UT::Print("%s%f %f\n",  str.c_str(), m00(), m01());
      UT::Print("%s%f %f\n", _str.c_str(), m10(), m11());
    }
  }
  inline void PrintDiagonal(const bool e = false) const {
    if (e) {
      UT::Print("%e %e\n", m00(), m11());
    } else {
      UT::Print("%f %f\n", m00(), m11());
    }
  }
  inline void PrintDiagonal(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    PrintDiagonal(e);
  }

  inline bool AssertEqual(const SymmetricMatrix2x2<TYPE> &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const SymmetricMatrix2x2<TYPE> E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  
  inline void Random(const TYPE mMax) { Random(-mMax, mMax); }
  inline void Random(const TYPE mMin, const TYPE mMax) { UT::Random(&m00(), 3, mMin, mMax); }
  static inline SymmetricMatrix2x2<TYPE> GetRandom(const TYPE mMax) {
    SymmetricMatrix2x2<TYPE> v;
    v.Random(mMax);
    return v;
  }
  static inline SymmetricMatrix2x2<TYPE> GetRandom(const TYPE mMin, const TYPE mMax) {
    SymmetricMatrix2x2<TYPE> v;
    v.Random(mMin, mMax);
    return v;
  }
  
  static inline TYPE MahalanobisDistance(const SymmetricMatrix2x2<TYPE> &W, const TYPE d) {
    return (W.m00() + W.m01() + W.m10() + W.m11()) * d * d;
  }
  static inline TYPE MahalanobisDistance(const SymmetricMatrix2x2<TYPE> &W, const Vector2<TYPE> &d) {
    return (W.m00() * d.x() + W.m01() * d.y()) * d.x()
         + (W.m10() * d.x() + W.m11() * d.y()) * d.y();
  }

  static inline void aaT(const Vector2<TYPE> &a, SymmetricMatrix2x2<TYPE> &aaT) {
    aaT.m00() = a.v0() * a.v0();
    aaT.m01() = a.v0() * a.v1();
    aaT.m11() = a.v1() * a.v1();
  }
  static inline void Ab(const SymmetricMatrix2x2<TYPE> &A, const Vector2<TYPE> &b,
                        Vector2<TYPE> &Ab) {
    Ab.v0() = A.m00() * b.v0() + A.m01() * b.v1();
    Ab.v1() = A.m01() * b.v0() + A.m11() * b.v1();
  }
  static inline void AddAbTo(const SymmetricMatrix2x2<TYPE> &A, const Vector2<TYPE> &b,
                             Vector2<TYPE> &Ab) {
    Ab.v0() += A.m00() * b.v0() + A.m01() * b.v1();
    Ab.v1() += A.m01() * b.v0() + A.m11() * b.v1();
  }
  static inline void AB(const AlignedMatrix2x2f &A, SymmetricMatrix2x2<TYPE> &B, AlignedMatrix2x2f &AB) {
    AB.m00() = A.m00() * B.m00() + A.m01() * B.m10();
    AB.m01() = A.m00() * B.m01() + A.m01() * B.m11();
    AB.m10() = A.m10() * B.m00() + A.m11() * B.m10();
    AB.m11() = A.m10() * B.m01() + A.m11() * B.m11();
  }
  static inline void AAT(const AlignedMatrix2x2f &A, SymmetricMatrix2x2<TYPE> &AAT) {
    const xp128f t = A.m_00_01_10_11() * A.m_00_01_10_11();
    AAT.m01() = A.m00() * A.m10() + A.m01() * A.m11();
    AAT.m00() = t[0] + t[1];
    AAT.m11() = t[2] + t[3];
  }
  static inline void AAT(const AlignedMatrix2x2f &A, AlignedMatrix2x2f &AAT) {
    const xp128f t = A.m_00_01_10_11() * A.m_00_01_10_11();
    AAT.m01() = AAT.m10() = A.m00() * A.m10() + A.m01() * A.m11();
    AAT.m00() = t[0] + t[1];
    AAT.m11() = t[2] + t[3];
  }
  static inline void ABT(const AlignedMatrix2x2f &A, const AlignedMatrix2x2f &B, SymmetricMatrix2x2<TYPE> &ABT) {
    const xp128f t = A.m_00_01_10_11() * B.m_00_01_10_11();
    ABT.m00() = t[0] + t[1];
    ABT.m11() = t[2] + t[3];
    ABT.m01() = A.m00() * B.m10() + A.m01() * B.m11();
  }

 protected:

  TYPE m_data[3];
};

typedef SymmetricMatrix2x2<float> SymmetricMatrix2x2f;
typedef SymmetricMatrix2x2<double> SymmetricMatrix2x2d;

}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x2f : public Eigen::Matrix2f {
 public:
  inline EigenMatrix2x2f() : Eigen::Matrix2f() {}
  inline EigenMatrix2x2f(const Eigen::Matrix2f &e_M) : Eigen::Matrix2f(e_M) {}
  inline EigenMatrix2x2f(const LA::AlignedMatrix2x2f &M) : Eigen::Matrix2f() {
    Eigen::Matrix2f &e_M = *this;
    e_M(0, 0) = M.m00();  e_M(0, 1) = M.m01();
    e_M(1, 0) = M.m10();  e_M(1, 1) = M.m11();
  }
  inline EigenMatrix2x2f(const LA::SymmetricMatrix2x2f &M) : Eigen::Matrix2f() {
    Eigen::Matrix2f &e_M = *this;
    e_M(0, 0) = M.m00();  e_M(0, 1) = M.m01();
    e_M(1, 0) = M.m01();  e_M(1, 1) = M.m11();
  }
  inline void operator = (const Eigen::Matrix2f &e_M) { *((Eigen::Matrix2f *) this) = e_M; }
  inline LA::AlignedMatrix2x2f GetAlignedMatrix2x2f() const {
    LA::AlignedMatrix2x2f M;
    const Eigen::Matrix2f &e_M = *this;
    M.m00() = e_M(0, 0);  M.m01() = e_M(0, 1);
    M.m10() = e_M(1, 0);  M.m11() = e_M(1, 1);
    return M;
  }
  inline LA::SymmetricMatrix2x2f GetSymmetricMatrix2x2f() const {
    const Eigen::Matrix2f &e_M = *this;
#ifdef CFG_DEBUG
    UT_ASSERT(e_M(0, 1) == e_M(1, 0));
#endif
    LA::SymmetricMatrix2x2f M;
    M.m00() = e_M(0, 0);
    M.m01() = e_M(0, 1);
    M.m11() = e_M(1, 1);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix2x2f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix2x2f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x2f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix2x2f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix2x2f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x2f().AssertZero(verbose, str, epsAbs, epsRel);
  }
};
#endif

#endif
