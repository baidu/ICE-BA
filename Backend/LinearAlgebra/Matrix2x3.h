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
#ifndef _MATRIX_2x3_H_
#define _MATRIX_2x3_H_

#include "Matrix2x2.h"
#include "Matrix3x2.h"
#include "Matrix3x3.h"

namespace LA {

class AlignedMatrix2x3f {

 public:

  inline const xp128f& m_00_01_02_r0() const { return m_data[0]; }  inline xp128f& m_00_01_02_r0() { return m_data[0]; }
  inline const xp128f& m_10_11_12_r1() const { return m_data[1]; }  inline xp128f& m_10_11_12_r1() { return m_data[1]; }

  inline const float& m00() const { return m_data[0][0]; }       inline float& m00() { return m_data[0][0]; }
  inline const float& m01() const { return m_data[0][1]; }       inline float& m01() { return m_data[0][1]; }
  inline const float& m02() const { return m_data[0][2]; }       inline float& m02() { return m_data[0][2]; }
  inline const float& r0 () const { return m_data[0][3]; }       inline float& r0 () { return m_data[0][3]; }
  inline const float& m10() const { return m_data[1][0]; }       inline float& m10() { return m_data[1][0]; }
  inline const float& m11() const { return m_data[1][1]; }       inline float& m11() { return m_data[1][1]; }
  inline const float& m12() const { return m_data[1][2]; }       inline float& m12() { return m_data[1][2]; }
  inline const float& r1 () const { return m_data[1][3]; }       inline float& r1 () { return m_data[1][3]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline void operator += (const AlignedMatrix2x3f &M) {
    m_00_01_02_r0() += M.m_00_01_02_r0();
    m_10_11_12_r1() += M.m_10_11_12_r1();
  }
  inline void operator -= (const AlignedMatrix2x3f &M) {
    m_00_01_02_r0() -= M.m_00_01_02_r0();
    m_10_11_12_r1() -= M.m_10_11_12_r1();
  }
  inline void operator *= (const float s) {
    m_00_01_02_r0() *= s;
    m_10_11_12_r1() *= s;
  }
  inline void operator *= (const xp128f &s) {
    m_00_01_02_r0() *= s;
    m_10_11_12_r1() *= s;
  }
  inline AlignedMatrix2x3f operator - (const AlignedMatrix2x3f &B) const {
    AlignedMatrix2x3f AmB;
    AmB.m_00_01_02_r0() = m_00_01_02_r0() - B.m_00_01_02_r0();
    AmB.m_10_11_12_r1() = m_10_11_12_r1() - B.m_10_11_12_r1();
    return AmB;
  }
  inline AlignedMatrix2x3f operator * (const AlignedMatrix3x3f &B) const {
    const AlignedMatrix3x3f BT = B.GetTranspose();
    AlignedMatrix2x3f AB;
    ABT(*this, BT, AB);
    return AB;
  }

  inline void Set(const float m00, const float m01, const float m02, const float m10, const float m11,
                  const float m12) {
    m_00_01_02_r0().vset_all_lane(m00, m01, m02, 0);
    m_10_11_12_r1().vset_all_lane(m10, m11, m12, 0);
  }
  inline void Set(const float *M) { memcpy(&m00(), M, 12); memcpy(&m10(), M + 3, 12); }
  inline void Get(float *M) const { memcpy(M, &m00(), 12); memcpy(M + 3, &m10(), 12); }

  inline void MakeIdentity() { MakeZero(); m00() = m11() = 1.0f; }
  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x3f)); }
  inline void MakeZero2x2() { m00() = 0.0f; m01() = 0.0f; m10() = 0.0f; m11() = 0.0f; }

  inline void GetTranspose(Matrix3x2f &M) const {
    M[0][0] = m00();  M[1][0] = m01();  M[2][0] = m02();
    M[0][1] = m10();  M[1][1] = m11();  M[2][1] = m12();
  }
  inline void GetMinus(AlignedMatrix2x3f &M) const {
    const xp128f zero = xp128f::get(0.0f);
    M.m_00_01_02_r0() = zero - m_00_01_02_r0();
    M.m_10_11_12_r1() = zero - m_10_11_12_r1();
  }
  inline void GetScaled(const xp128f &s, AlignedMatrix2x3f &M) const {
    M.m_00_01_02_r0() = s * m_00_01_02_r0();
    M.m_10_11_12_r1() = s * m_10_11_12_r1();
  }

  inline bool Valid() const { return m00() != FLT_MAX; }
  inline bool Invalid() const { return m00() == FLT_MAX; }
  inline void Invalidate() { m00() = FLT_MAX; }

  inline void Scale(const xp128f &s) {
    m_00_01_02_r0() *= s;
    m_10_11_12_r1() *= s;
  }

  inline void AddATBTo(const Vector2f &B, AlignedVector3f &ATB) const {
    ATB.v012r() += m_00_01_02_r0() * B.v0() + m_10_11_12_r1() * B.v1();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", m00(), m01(), m02());
      UT::Print("%e %e %e\n", m10(), m11(), m12());
    } else {
      UT::Print("%f %f %f\n", m00(), m01(), m02());
      UT::Print("%f %f %f\n", m10(), m11(), m12());
    }
  }
  inline bool AssertEqual(const AlignedMatrix2x3f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m10(), &M.m10(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix2x3f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m00(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertZero(&m10(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  inline void SetInfinite() {
    m_00_01_02_r0().vdup_all_lane(FLT_MAX);
    m_10_11_12_r1().vdup_all_lane(FLT_MAX);
  }
  inline void AssertInfinite() const {
    UT_ASSERT(m00() == FLT_MAX);  UT_ASSERT(m01() == FLT_MAX);  UT_ASSERT(m02() == FLT_MAX);
    UT_ASSERT(r0() == FLT_MAX);
    UT_ASSERT(m10() == FLT_MAX);  UT_ASSERT(m11() == FLT_MAX);  UT_ASSERT(m12() == FLT_MAX);
    UT_ASSERT(r1() == FLT_MAX);
  }
  inline void AssertFinite() const {
    UT_ASSERT(m00() != FLT_MAX);  UT_ASSERT(m01() != FLT_MAX);  UT_ASSERT(m02() != FLT_MAX);
    UT_ASSERT(r0() != FLT_MAX);
    UT_ASSERT(m10() != FLT_MAX);  UT_ASSERT(m11() != FLT_MAX);  UT_ASSERT(m12() != FLT_MAX);
    UT_ASSERT(r1() != FLT_MAX);
  }

  inline void Random(const float mMax) { Random(-mMax, mMax); }
  inline void Random(const float mMin, const float mMax) {
    UT::Random(&m00(), 3, mMin, mMax);
    UT::Random(&m10(), 3, mMin, mMax);
  }
  static inline AlignedMatrix2x3f GetRandom(const float mMax) {
    AlignedMatrix2x3f v;
    v.Random(mMax);
    return v;
  }
  static inline AlignedMatrix2x3f GetRandom(const float mMin, const float mMax) {
    AlignedMatrix2x3f v;
    v.Random(mMin, mMax);
    return v;
  }
  
  static inline void Ab(const AlignedMatrix2x3f &A, const AlignedVector3f &b, float *Ab) {
    Ab[0] = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab[1] = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo(const AlignedMatrix2x3f &A, const AlignedVector3f &b, float *Ab) {
    Ab[0] += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab[1] += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
  }
  static inline void AB(const SymmetricMatrix2x2f &A, const AlignedMatrix2x3f &B,
                        AlignedMatrix2x3f &AB) {
    const xp128f a01 = xp128f::get(A.m01());
    AB.m_00_01_02_r0() = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * a01;
    AB.m_10_11_12_r1() = B.m_00_01_02_r0() * a01 + B.m_10_11_12_r1() * A.m11();
  }
  //static inline void AB(const SymmetricMatrix2x2f &A, const AlignedMatrix2x3f &B1,
  //                      const AlignedMatrix2x3f &B2,
  //                      AlignedMatrix2x3f &AB1, AlignedMatrix2x3f &AB2) {
  //  AB1.m_00_01_02_r0() = B1.m_00_01_02_r0() * A.m00() + B1.m_10_11_12_r1() * A.m01();
  //  AB1.m_10_11_12_r1() = B1.m_00_01_02_r0() * A.m01() + B1.m_10_11_12_r1() * A.m11();
  //  AB2.m_00_01_02_r0() = B2.m_00_01_02_r0() * A.m00() + B2.m_10_11_12_r1() * A.m01();
  //  AB2.m_10_11_12_r1() = B2.m_00_01_02_r0() * A.m01() + B2.m_10_11_12_r1() * A.m11();
  //}
  static inline void ABT(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                         SymmetricMatrix2x2f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
  }
  static inline void AddABTTo(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                              SymmetricMatrix2x2f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
  }
  static inline void ABT(const AlignedMatrix2x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix2x3f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo(const AlignedMatrix2x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix2x3f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ATB(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                         AlignedMatrix3x3f &ATB) {
    ATB.m_00_01_02_r0() = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * A.m10();
    ATB.m_10_11_12_r1() = B.m_00_01_02_r0() * A.m01() + B.m_10_11_12_r1() * A.m11();
    ATB.m_20_21_22_r2() = B.m_00_01_02_r0() * A.m02() + B.m_10_11_12_r1() * A.m12();
  }
  static inline void ATBToUpper(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                                AlignedMatrix3x3f &ATB) {
    ATB.m_00_01_02_r0() = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * A.m10();
    ATB.m11() = A.m01() * B.m01() + A.m11() * B.m11();
    ATB.m12() = A.m01() * B.m02() + A.m11() * B.m12();
    ATB.m22() = A.m02() * B.m02() + A.m12() * B.m12();
  }
  static inline void ATb(const AlignedMatrix2x3f &A, const Vector2f &b, AlignedVector3f &ATb) {
    ATb.v012r() = A.m_00_01_02_r0() * b.v0() + A.m_10_11_12_r1() * b.v1();
  }
  static inline void AddATbTo(const AlignedMatrix2x3f &A, const Vector2f &b,
                              AlignedVector3f &ATb) {
    ATb.v012r() += A.m_00_01_02_r0() * b.v0() + A.m_10_11_12_r1() * b.v1();
  }
  static inline void ATb(const AlignedMatrix2x3f &A, const xp128f &b0, const xp128f &b1,
                         AlignedVector3f &ATb) {
    ATb.v012r() = A.m_00_01_02_r0() * b0 + A.m_10_11_12_r1() * b1;
  }
  static inline void AddATbTo(const AlignedMatrix2x3f &A, const xp128f &b0, const xp128f &b1,
                              AlignedVector3f &ATb) {
    ATb.v012r() += A.m_00_01_02_r0() * b0 + A.m_10_11_12_r1() * b1;
  }

 protected:
  xp128f m_data[2];
};

template<typename TYPE> class Matrix2x3 {
 public:
  inline const TYPE& m00() const { return m_data[0][0]; }    inline TYPE& m00() { return m_data[0][0]; }
  inline const TYPE& m01() const { return m_data[0][1]; }    inline TYPE& m01() { return m_data[0][1]; }
  inline const TYPE& m02() const { return m_data[0][2]; }    inline TYPE& m02() { return m_data[0][2]; }
  inline const TYPE& m10() const { return m_data[1][0]; }    inline TYPE& m10() { return m_data[1][0]; }
  inline const TYPE& m11() const { return m_data[1][1]; }    inline TYPE& m11() { return m_data[1][1]; }
  inline const TYPE& m12() const { return m_data[1][2]; }    inline TYPE& m12() { return m_data[1][2]; }

  inline const TYPE* operator [] (const int i) const { return m_data[i]; }
  inline       TYPE* operator [] (const int i)       { return m_data[i]; }

  inline void MakeZero() { memset(this, 0, sizeof(Matrix2x3<TYPE>)); }

  inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }

  inline void Set(const AlignedMatrix2x3f &M);
  inline void Get(AlignedMatrix2x3f &M) const;

  inline void SetTranspose(const Matrix3x2<TYPE> &M) {
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        m_data[i][j] = M[j][i];
      }
    }
  }
  inline void GetTranspose(Matrix3x2<TYPE> &M) const {
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 3; ++j) {
        M[j][i] = m_data[i][j];
      }
    }
  }

  static inline void AddAbTo(const Matrix2x3<TYPE> &A, const AlignedVector3f &b, float *Ab) {
    Ab[0] += A.m00() * b.v0() + A.m01() * b.v1() + A.m02() * b.v2();
    Ab[1] += A.m10() * b.v0() + A.m11() * b.v1() + A.m12() * b.v2();
  }

 public:
  TYPE m_data[2][3];
};

typedef Matrix2x3<float> Matrix2x3f;
typedef Matrix2x3<double> Matrix2x3d;

template<> inline void Matrix2x3f::Set(const AlignedMatrix2x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m10(), &M.m10(), 12);
}
template<> inline void Matrix2x3f::Get(AlignedMatrix2x3f &M) const {
  memcpy(&M.m00(), &m00(), 12);
  memcpy(&M.m10(), &m10(), 12);
}

}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x3f : public Eigen::Matrix<float, 2, 3> {
 public:
  inline EigenMatrix2x3f() : Eigen::Matrix<float, 2, 3>() {}
  inline EigenMatrix2x3f(const Eigen::Matrix<float, 2, 3> &e_M) : Eigen::Matrix<float, 2, 3>(e_M) {}
  inline EigenMatrix2x3f(const LA::AlignedMatrix2x3f &M) : Eigen::Matrix<float, 2, 3>() {
    Eigen::Matrix<float, 2, 3> &e_M = *this;
    e_M(0, 0) = M.m00();  e_M(0, 1) = M.m01();  e_M(0, 2) = M.m02();
    e_M(1, 0) = M.m10();  e_M(1, 1) = M.m11();  e_M(1, 2) = M.m12();
  }
  inline void operator = (const Eigen::Matrix<float, 2, 3> &e_M) { *((Eigen::Matrix<float, 2, 3> *) this) = e_M; }
  inline LA::AlignedMatrix2x3f GetAlignedMatrix2x3f() const {
    LA::AlignedMatrix2x3f M;
    const Eigen::Matrix<float, 2, 3> &e_M = *this;
    M.m00() = e_M(0, 0);  M.m01() = e_M(0, 1);  M.m02() = e_M(0, 2);
    M.m10() = e_M(1, 0);  M.m11() = e_M(1, 1);  M.m12() = e_M(1, 2);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix2x3f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix2x3f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x3f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix2x3f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix2x3f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
