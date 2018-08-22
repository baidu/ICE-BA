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
#ifndef LINEARALGEBRA_MATRIX3X3_H_
#define LINEARALGEBRA_MATRIX3X3_H_

#include "Vector3.h"
#include "Matrix2x2.h"
#include "LinearSystem.h"
#include <string>
namespace LA {

class AlignedMatrix3x3f {
 public:
  inline const xp128f& m_00_01_02_r0() const {
    return m_data[0];
  }
  inline xp128f& m_00_01_02_r0() {
    return m_data[0];
  }
  inline const xp128f& m_10_11_12_r1() const {
    return m_data[1];
  }
  inline xp128f& m_10_11_12_r1() {
    return m_data[1];
  }
  inline const xp128f& m_20_21_22_r2() const {
    return m_data[2];
  }
  inline xp128f& m_20_21_22_r2() {
    return m_data[2];
  }

  inline const float& m00() const { return m_data[0][0]; }
  inline float& m00() { return m_data[0][0]; }
  inline const float& m01() const { return m_data[0][1]; }
  inline float& m01() { return m_data[0][1]; }
  inline const float& m02() const { return m_data[0][2]; }
  inline float& m02() { return m_data[0][2]; }
  inline const float& r0 () const { return m_data[0][3]; }
  inline float& r0 () { return m_data[0][3]; }
  inline const float& m10() const { return m_data[1][0]; }
  inline float& m10() { return m_data[1][0]; }
  inline const float& m11() const { return m_data[1][1]; }
  inline float& m11() { return m_data[1][1]; }
  inline const float& m12() const { return m_data[1][2]; }
  inline float& m12() { return m_data[1][2]; }
  inline const float& r1 () const { return m_data[1][3]; }
  inline float& r1 () { return m_data[1][3]; }
  inline const float& m20() const { return m_data[2][0]; }
  inline float& m20() { return m_data[2][0]; }
  inline const float& m21() const { return m_data[2][1]; }
  inline float& m21() { return m_data[2][1]; }
  inline const float& m22() const { return m_data[2][2]; }
  inline float& m22() { return m_data[2][2]; }
  inline const float& r2 () const { return m_data[2][3]; }
  inline float& r2 () { return m_data[2][3]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline const float& operator() (const int i, const int j) const { return m_data[i][j]; }
  inline       float& operator() (const int i, const int j)       { return m_data[i][j]; }

  inline bool operator == (const AlignedMatrix3x3f &M) const {
    return m00() == M.m00() && m01() == M.m01() && m02() == M.m02() &&
           m10() == M.m10() && m11() == M.m11() && m12() == M.m12() &&
           m20() == M.m20() && m21() == M.m21() && m22() == M.m22();
  }
  inline void operator += (const AlignedMatrix3x3f &M) {
    m_00_01_02_r0() += M.m_00_01_02_r0();
    m_10_11_12_r1() += M.m_10_11_12_r1();
    m_20_21_22_r2() += M.m_20_21_22_r2();
  }
  inline void operator += (const float *M) {
    AlignedMatrix3x3f _M;
    _M.Set(M);
    *this += _M;
  }
  inline void operator -= (const AlignedMatrix3x3f &M) {
    m_00_01_02_r0() -= M.m_00_01_02_r0();
    m_10_11_12_r1() -= M.m_10_11_12_r1();
    m_20_21_22_r2() -= M.m_20_21_22_r2();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { Scale(s); }
  inline void operator /= (const float d) { Scale(1.0f / d); }
  inline AlignedMatrix3x3f operator + (const AlignedMatrix3x3f &B) const {
    AlignedMatrix3x3f _ApB; ApB(*this, B, _ApB); return _ApB;
  }
  inline AlignedMatrix3x3f operator - (const AlignedMatrix3x3f &B) const {
    AlignedMatrix3x3f _AmB; AmB(*this, B, _AmB); return _AmB;
  }
  inline AlignedMatrix3x3f operator * (const float s) const {
    AlignedMatrix3x3f sA; GetScaled(s, sA); return sA;
  }
  inline AlignedMatrix3x3f operator * (const xp128f &s) const {
    AlignedMatrix3x3f sA; GetScaled(s, sA); return sA;
  }
  inline AlignedMatrix3x3f operator * (const AlignedMatrix3x3f &B) const {
    AlignedMatrix3x3f AB;
    AlignedMatrix3x3f::AB(*this, B, AB);
    return AB;
  }
  inline AlignedVector3f operator * (const AlignedVector3f &b) const {
    AlignedVector3f Ab;
    AlignedMatrix3x3f::Ab(*this, b, (float *) &Ab);
    return Ab;
  }

  inline void Set(const float *M) { Set(M, M + 3, M + 6); }
  inline void Set(const float *M0, const float *M1, const float *M2) {
    memcpy(&m00(), M0, 12);
    memcpy(&m10(), M1, 12);
    memcpy(&m20(), M2, 12);
  }
  inline void Set(const float M[3][3]) { Set(M[0], M[1], M[2]); }
  inline void Set(const SymmetricMatrix2x2f &M) {
    m_00_01_02_r0() = xp128f::get(M.m00(), M.m01(), 0.0f, 0.0f);
    m_10_11_12_r1() = xp128f::get(M.m10(), M.m11(), 0.0f, 0.0f);
    m_20_21_22_r2() = xp128f::get(0.0f);
  }
  inline void Set(const double *M) {
    m00() = static_cast<float>(M[0]);
    m01() = static_cast<float>(M[1]);
    m02() = static_cast<float>(M[2]);
    m10() = static_cast<float>(M[3]);
    m11() = static_cast<float>(M[4]);
    m12() = static_cast<float>(M[5]);
    m20() = static_cast<float>(M[6]);
    m21() = static_cast<float>(M[7]);
    m22() = static_cast<float>(M[8]);
  }
  inline void Get(float *M) const { Get(M, M + 3, M + 6); }
  inline void Get(float *M0, float *M1, float *M2) const {
    memcpy(M0, &m00(), 12);
    memcpy(M1, &m10(), 12);
    memcpy(M2, &m20(), 12);
  }
  inline void Get(float M[3][3]) const { Get(M[0], M[1], M[2]); }
  inline void GetMinus(AlignedMatrix3x3f &M) const {
    const xp128f zero = xp128f::get(0.0f);
    M.m_00_01_02_r0() = zero - m_00_01_02_r0();
    M.m_10_11_12_r1() = zero - m_10_11_12_r1();
    M.m_20_21_22_r2() = zero - m_20_21_22_r2();
  }
  inline AlignedMatrix3x3f GetMinus() const { AlignedMatrix3x3f M; GetMinus(M); return M; }
  inline void SetColumn0(const float *v) { m00() = v[0]; m10() = v[1]; m20() = v[2]; }
  inline void SetColumn1(const float *v) { m01() = v[0]; m11() = v[1]; m21() = v[2]; }
  inline void SetColumn2(const float *v) { m02() = v[0]; m12() = v[1]; m22() = v[2]; }
  inline void IncreaseColumn2(const AlignedVector3f &v) {
    m02() += v.v0();
    m12() += v.v1();
    m22() += v.v2();
  }
  inline void GetColumn0(AlignedVector3f &v) const {
    v.v012r().vset_all_lane(m00(), m10(), m20(), 0.0f);
  }
  inline void GetColumn1(AlignedVector3f &v) const {
    v.v012r().vset_all_lane(m01(), m11(), m21(), 0.0f);
  }
  inline void GetColumn2(AlignedVector3f &v) const {
    v.v012r().vset_all_lane(m02(), m12(), m22(), 0.0f);
  }
  inline AlignedVector3f GetColumn0() const {
    AlignedVector3f v;
    v.v012r().vset_all_lane(m00(), m10(), m20(), 0.0f);
    return v;
  }
  inline AlignedVector3f GetColumn1() const {
    AlignedVector3f v;
    v.v012r().vset_all_lane(m01(), m11(), m21(), 0.0f);
    return v;
  }
  inline AlignedVector3f GetColumn2() const {
    AlignedVector3f v;
    v.v012r().vset_all_lane(m02(), m12(), m22(), 0.0f);
    return v;
  }

  inline void MakeZero() {
    memset(this, 0, sizeof(AlignedMatrix3x3f));
  }
  inline void MakeZero2x2() {
    m00() = 0.0f; m01() = 0.0f; m10() = 0.0f; m11() = 0.0f;
  }
  inline void MakeZero2x3() {
    m_00_01_02_r0().vdup_all_lane(0.f);
    m_10_11_12_r1().vdup_all_lane(0.f);
  }
  inline void MakeZero3x2() {
    MakeZero2x2(); m20() = 0.0f; m21() = 0.0f;
  }
  inline void MakeMinus() {
    m_00_01_02_r0().vmake_minus();
    m_10_11_12_r1().vmake_minus();
    m_20_21_22_r2().vmake_minus();
  }
  inline void MakeIdentity() {
    MakeZero();
    m00() = m11() = m22() = 1.0f;
  }

  inline bool Valid() const { return m00() != FLT_MAX; }
  inline bool Invalid() const { return m00() == FLT_MAX; }
  inline void Invalidate() { m00() = FLT_MAX; }

  static AlignedMatrix3x3f GetDiagonal(const float d) {
    AlignedMatrix3x3f M;
    M.MakeDiagonal(d);
    return M;
  }
  inline void MakeDiagonal(const float d) {
    MakeZero(); SetDiagonal(d);
  }
  inline void SetDiagonal(const float d) {
    m00() = d;
    m11() = d;
    m22() = d;
  }
  inline void MakeDiagonal(const SymmetricMatrix2x2f &D0, const float d1) {
    MakeZero(); SetDiagonal(D0, d1);
  }
  inline void SetDiagonal(const SymmetricMatrix2x2f &D0, const float d1) {
    m00() = D0.m00(); m01() = D0.m01();
    m10() = D0.m10(); m11() = D0.m11();
    m22() = d1;
  }
  inline void SetDiagonal(const AlignedVector3f &d) {
    m00() = d.v0();
    m11() = d.v1();
    m22() = d.v2();
  }
  inline void GetDiagonal(AlignedVector3f &d) const {
    d.v0() = m00(); d.v1() = m11(); d.v2() = m22();
  }
  inline AlignedVector3f GetDiagonal() const {
    AlignedVector3f d; GetDiagonal(d); return d;
  }
  inline void ScaleDiagonal(const float s) {
    m00() *= s; m11() *= s; m22() *= s;
  }
  inline void IncreaseDiagonal(const float d) {
    m00() += d;
    m11() += d;
    m22() += d;
  }
  inline void IncreaseDiagonal(const float d0, const float d1, const float d2) {
    m00() += d0;
    m11() += d1;
    m22() += d2;
  }
  inline void SetLowerFromUpper() {
    m10() = m01(); m20() = m02(); m21() = m12();
  }
  inline void GetTranspose(AlignedMatrix3x3f &MT) const {
    MT.m00() = m00(); MT.m01() = m10(); MT.m02() = m20();
    MT.m10() = m01(); MT.m11() = m11(); MT.m12() = m21();
    MT.m20() = m02(); MT.m21() = m12(); MT.m22() = m22();
  }
  inline AlignedMatrix3x3f GetTranspose() const {
    AlignedMatrix3x3f MT;
    GetTranspose(MT);
    return MT;
  }
  inline void Transpose() {
    UT_SWAP(m01(), m10());
    UT_SWAP(m02(), m20());
    UT_SWAP(m12(), m21());
  }
  inline bool GetInverse(AlignedMatrix3x3f &MI) const {
    MI.m00() = m11() * m22() - m12() * m21();
    MI.m01() = m02() * m21() - m01() * m22();
    MI.m02() = m01() * m12() - m02() * m11();
    MI.m10() = m12() * m20() - m10() * m22();
    MI.m11() = m00() * m22() - m02() * m20();
    MI.m12() = m02() * m10() - m00() * m12();
    MI.m20() = m10() * m21() - m11() * m20();
    MI.m21() = m01() * m20() - m00() * m21();
    MI.m22() = m00() * m11() - m01() * m10();
    const float d = m00() * MI.m00() + m01() * MI.m10() + m02() * MI.m20();
    if (d <= 0) {
      MI.Invalidate();
      return false;
    } else {
      MI /= d;
      return true;
    }
  }
  inline AlignedMatrix3x3f GetInverse() const {
    AlignedMatrix3x3f MI;
    GetInverse(MI);
    return MI;
  }
  inline bool InverseLDL(const float *eps = NULL) {
    float *A[3] = {&m00(), &m10(), &m20()};
    if (LS::InverseLDL<float>(3, A, eps)) {
      SetLowerFromUpper();
      return true;
    } else {
      Invalidate();
      return false;
    }
  }
  inline bool GetInverseLDL(AlignedMatrix3x3f &MI, const float *eps = NULL) const {
    MI = *this;
    return MI.InverseLDL(eps);
  }
  inline AlignedMatrix3x3f GetInverseLDL(const float *eps = NULL) const {
    AlignedMatrix3x3f MI;
    GetInverseLDL(MI, eps);
    return MI;
  }

  inline void Scale(const float s) {
    m_00_01_02_r0() *= s;
    m_10_11_12_r1() *= s;
    m_20_21_22_r2() *= s;
  }
  inline void Scale(const xp128f &s) {
    m_00_01_02_r0() *= s;
    m_10_11_12_r1() *= s;
    m_20_21_22_r2() *= s;
  }
  inline void GetScaled(const float s, AlignedMatrix3x3f &M) const {
    M.m_00_01_02_r0() = m_00_01_02_r0() * s;
    M.m_10_11_12_r1() = m_10_11_12_r1() * s;
    M.m_20_21_22_r2() = m_20_21_22_r2() * s;
  }

  inline void GetScaled(const xp128f &s, AlignedMatrix3x3f &M) const {
    M.m_00_01_02_r0() = m_00_01_02_r0() * s;
    M.m_10_11_12_r1() = m_10_11_12_r1() * s;
    M.m_20_21_22_r2() = m_20_21_22_r2() * s;
  }
  inline void GetScaledToUpper(const xp128f &s, AlignedMatrix3x3f &M) const {
    M.m_00_01_02_r0() = m_00_01_02_r0() * s;
    M.m11() = s[0] * m11();
    M.m12() = s[0] * m12();
    M.m22() = s[0] * m22();
  }
  inline AlignedMatrix3x3f GetScaled(const float s) const {
    AlignedMatrix3x3f M;
    GetScaled(s, M);
    return M;
  }
  inline AlignedMatrix3x3f GetScaled(const xp128f &s) const {
    AlignedMatrix3x3f M;
    GetScaled(s, M);
    return M;
  }
  inline float Determinant() const {
    return m00() * (m11() * m22() - m12() * m21()) +
           m01() * (m12() * m20() - m10() * m22()) +
           m02() * (m10() * m21() - m11() * m20());
  }
  inline float Trace() const { return m00() + m11() + m22(); }
  inline float SquaredFrobeniusNorm() const {
    return (m_00_01_02_r0() * m_00_01_02_r0() +
            m_10_11_12_r1() * m_10_11_12_r1() +
            m_20_21_22_r2() * m_20_21_22_r2()).vsum_012();
  }
  inline void EnforceUnitFrobeniusNorm() { Scale(sqrtf(1.0f / SquaredFrobeniusNorm())); }
  inline void EnforceUnitLastEntry() { Scale(1.0f / m22()); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", m00(), m01(), m02());
      UT::Print("%e %e %e\n", m10(), m11(), m12());
      UT::Print("%e %e %e\n", m20(), m21(), m22());
    } else {
      UT::Print("%f %f %f\n", m00(), m01(), m02());
      UT::Print("%f %f %f\n", m10(), m11(), m12());
      UT::Print("%f %f %f\n", m20(), m21(), m22());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    if (e) {
      UT::Print("%s%e %e %e\n",  str.c_str(), m00(), m01(), m02());
      UT::Print("%s%e %e %e\n", _str.c_str(), m10(), m11(), m12());
      UT::Print("%s%e %e %e\n", _str.c_str(), m20(), m21(), m22());
    } else {
      UT::Print("%s%f %f %f\n",  str.c_str(), m00(), m01(), m02());
      UT::Print("%s%f %f %f\n", _str.c_str(), m10(), m11(), m12());
      UT::Print("%s%f %f %f\n", _str.c_str(), m20(), m21(), m22());
    }
  }
  inline void Save(FILE *fp) const {
    fprintf(fp, "%f %f %f\n", m00(), m01(), m02());
    fprintf(fp, "%f %f %f\n", m10(), m11(), m12());
    fprintf(fp, "%f %f %f\n", m20(), m21(), m22());
  }
  inline void Load(FILE *fp) {
    fscanf(fp, "%f %f %f", &m00(), &m01(), &m02());
    fscanf(fp, "%f %f %f", &m10(), &m11(), &m12());
    fscanf(fp, "%f %f %f", &m20(), &m21(), &m22());
  }

  inline bool AssertEqual(const AlignedMatrix3x3f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m10(), &M.m10(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m20(), &M.m20(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix3x3f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m00(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertZero(&m10(), 3, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertZero(&m20(), 3, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }
  inline void AssertIdentity() const {
    UT_ASSERT(m00() == 1.0f && m01() == 0.0f && m02() == 0.0f);
    UT_ASSERT(m10() == 0.0f && m11() == 1.0f && m12() == 0.0f);
    UT_ASSERT(m20() == 0.0f && m21() == 0.0f && m22() == 1.0f);
  }
  inline void AssertSymmetric() const {
    UT_ASSERT(m01() == m10() && m02() == m20() && m12() == m21());
  }
  inline void AssertSymmetric(const LA::AlignedMatrix3x3f &M) const {
    UT_ASSERT(m00() == M.m00() && m01() == M.m10() && m02() == M.m20());
    UT_ASSERT(m10() == M.m01() && m11() == M.m11() && m12() == M.m21());
    UT_ASSERT(m20() == M.m02() && m21() == M.m12() && m22() == M.m22());
  }
  inline void SetInfinite() {
    m_00_01_02_r0().vdup_all_lane(FLT_MAX);
    m_10_11_12_r1().vdup_all_lane(FLT_MAX);
    m_20_21_22_r2().vdup_all_lane(FLT_MAX);
  }
  inline void AssertInfinite() const {
    UT_ASSERT(m00() == FLT_MAX);  UT_ASSERT(m01() == FLT_MAX);  UT_ASSERT(m02() == FLT_MAX);
    UT_ASSERT(r0() == FLT_MAX);
    UT_ASSERT(m10() == FLT_MAX);  UT_ASSERT(m11() == FLT_MAX);  UT_ASSERT(m12() == FLT_MAX);
    UT_ASSERT(r1() == FLT_MAX);
    UT_ASSERT(m20() == FLT_MAX);  UT_ASSERT(m21() == FLT_MAX);  UT_ASSERT(m22() == FLT_MAX);
    UT_ASSERT(r2() == FLT_MAX);
  }
  inline void AssertFinite() const {
    UT_ASSERT(m00() != FLT_MAX);  UT_ASSERT(m01() != FLT_MAX);  UT_ASSERT(m02() != FLT_MAX);
    UT_ASSERT(r0() != FLT_MAX);
    UT_ASSERT(m10() != FLT_MAX);  UT_ASSERT(m11() != FLT_MAX);  UT_ASSERT(m12() != FLT_MAX);
    UT_ASSERT(r1() != FLT_MAX);
    UT_ASSERT(m20() != FLT_MAX);  UT_ASSERT(m21() != FLT_MAX);  UT_ASSERT(m22() != FLT_MAX);
    UT_ASSERT(r2() != FLT_MAX);
  }

  inline void Random(const float mMax) { Random(-mMax, mMax); }
  inline void Random(const float mMin, const float mMax) {
    UT::Random(&m00(), 3, mMin, mMax);
    UT::Random(&m10(), 3, mMin, mMax);
    UT::Random(&m20(), 3, mMin, mMax);
  }
  static inline AlignedMatrix3x3f GetRandom(const float mMax) {
    AlignedMatrix3x3f M;
    M.Random(mMax);
    return M;
  }
  static inline AlignedMatrix3x3f GetRandom(const float mMin, const float mMax) {
    AlignedMatrix3x3f M;
    M.Random(mMin, mMax);
    return M;
  }

  static inline void aTB(float *a, const AlignedMatrix3x3f &B) {
    const xp128f t = B.m_00_01_02_r0() * a[0] +
                     B.m_10_11_12_r1() * a[1] +
                     B.m_20_21_22_r2() * a[2];
    memcpy(a, &t, 12);
  }

  static inline void aaT(const AlignedVector3f &a, AlignedMatrix3x3f &aaT) {
    aaT.m_00_01_02_r0() = a.v012r() * a.v0();
    aaT.m11() = a.v1() * a.v1();
    aaT.m12() = a.v1() * a.v2();
    aaT.m22() = a.v2() * a.v2();
    aaT.SetLowerFromUpper();
  }
  static inline void abT(const AlignedVector3f &a,
                         const AlignedVector3f &b, AlignedMatrix3x3f &abT) {
    abT.m_00_01_02_r0() = b.v012r() * a.v0();
    abT.m_10_11_12_r1() = b.v012r() * a.v1();
    abT.m_20_21_22_r2() = b.v012r() * a.v2();
  }

  static inline void AddsATo(const xp128f &s, const AlignedMatrix3x3f &A, AlignedMatrix3x3f &sA) {
    sA.m_00_01_02_r0() += A.m_00_01_02_r0() * s;
    sA.m_10_11_12_r1() += A.m_10_11_12_r1() * s;
    sA.m_20_21_22_r2() += A.m_20_21_22_r2() * s;
  }
  static inline void AddsAToUpper(const xp128f &s, const AlignedMatrix3x3f &A, AlignedMatrix3x3f &sA) {
    sA.m_00_01_02_r0() += A.m_00_01_02_r0() * s;
    sA.m11() += A.m11() * s[0];
    sA.m12() += A.m12() * s[0];
    sA.m22() += A.m22() * s[0];
  }
  static inline void AddsATo(const xp128f &s, const AlignedMatrix3x3f &A, AlignedMatrix3x3f &sA,
                             const bool symmetric) {
    if (symmetric) {
      AddsAToUpper(s, A, sA);
      sA.SetLowerFromUpper();
    } else {
      AddsATo(s, A, sA);
    }
  }

  static inline void Ab_simd(const AlignedMatrix3x3f &A, const AlignedVector3f &b, float* Ab) {
    Ab[0] = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab[1] = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab[2] = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  template<typename TYPE>
  static inline void Ab(const AlignedMatrix3x3f &A, const AlignedVector3f &b, TYPE* Ab) {
    const float *Ar0 = &A.m00(), *Ar1 = &A.m10(), *Ar2 = &A.m20();
    const float *br = b;
    Ab[0] = *(Ar0 + 0) * *(br + 0) + *(Ar0 + 1) * *(br + 1) + *(Ar0 + 2) * *(br + 2);
    Ab[1] = *(Ar1 + 0) * *(br + 0) + *(Ar1 + 1) * *(br + 1) + *(Ar1 + 2) * *(br + 2);
    Ab[2] = *(Ar2 + 0) * *(br + 0) + *(Ar2 + 1) * *(br + 1) + *(Ar2 + 2) * *(br + 2);
  }

  static inline void AddAbTo_simd(const AlignedMatrix3x3f &A,
                                      const AlignedVector3f &b, float *Ab) {
    Ab[0] += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab[1] += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab[2] += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  template<typename TYPE>
  static inline void AddAbTo(const AlignedMatrix3x3f &A, const AlignedVector3f &b, TYPE *Ab) {
    const float *Ar0 = &A.m00(), *Ar1 = &A.m10(), *Ar2 = &A.m20();
    const float *br = b;
    Ab[0] += *(Ar0 + 0) * *(br + 0) + *(Ar0 + 1) * *(br + 1) + *(Ar0 + 2) * *(br + 2);
    Ab[1] += *(Ar1 + 0) * *(br + 0) + *(Ar1 + 1) * *(br + 1) + *(Ar1 + 2) * *(br + 2);
    Ab[2] += *(Ar2 + 0) * *(br + 0) + *(Ar2 + 1) * *(br + 1) + *(Ar2 + 2) * *(br + 2);
  }

  template<typename TYPE>
  static inline void SubtractAbFrom(const AlignedMatrix3x3f &A,
                                    const AlignedVector3f &b, TYPE *Ab) {
    Ab[0] -= (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab[1] -= (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab[2] -= (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void ApB(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ApB) {
    ApB.m_00_01_02_r0() = A.m_00_01_02_r0() + B.m_00_01_02_r0();
    ApB.m_10_11_12_r1() = A.m_10_11_12_r1() + B.m_10_11_12_r1();
    ApB.m_20_21_22_r2() = A.m_20_21_22_r2() + B.m_20_21_22_r2();
  }
  static inline void AmB(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &AmB) {
    AmB.m_00_01_02_r0() = A.m_00_01_02_r0() - B.m_00_01_02_r0();
    AmB.m_10_11_12_r1() = A.m_10_11_12_r1() - B.m_10_11_12_r1();
    AmB.m_20_21_22_r2() = A.m_20_21_22_r2() - B.m_20_21_22_r2();
  }
  // this is confusing, fucntion name AB means C = A * B, but in fact, it's A * BT
  static inline void AB(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                        AlignedMatrix3x3f &AB) {
    const AlignedMatrix3x3f BT = B.GetTranspose();
    ABT(A, BT, AB);
  }
  // this is confusing, fucntion name AB means C += A * B, but in fact, it's A * BT
  static inline void AddABTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix3x3f &AB) {
    const AlignedMatrix3x3f BT = B.GetTranspose();
    AddABTTo(A, BT, AB);
  }
  static inline void ABT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m20() = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m21() = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABT_simd(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m20() = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m21() = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         float *ABT0, float *ABT1, float *ABT2) {
    ABT0[0] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT0[1] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT0[2] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT1[0] = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT1[1] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT1[2] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT2[0] = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT2[1] = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT2[2] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTToUpper(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix3x3f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ABT, const bool symmetric) {
    if (symmetric) {
      AlignedMatrix3x3f::ABTToUpper(A, B, ABT);
      ABT.SetLowerFromUpper();
    } else {
      AlignedMatrix3x3f::ABT(A, B, ABT);
    }
  }
  static inline void AddABTTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix3x3f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m20() += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m21() += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTToUpper(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                   AlignedMatrix3x3f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix3x3f &ABT, const bool symmetric) {
    if (symmetric) {
      AddABTToUpper(A, B, ABT);
      ABT.SetLowerFromUpper();
    } else {
      AddABTTo(A, B, ABT);
    }
  }
  static inline void SubtractABTFrom(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                     AlignedMatrix3x3f &ABT) {
    ABT.m00() -= (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() -= (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() -= (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() -= (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() -= (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() -= (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m20() -= (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m21() -= (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m22() -= (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void SubtractABTFromUpper(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                          AlignedMatrix3x3f &ABT) {
    ABT.m00() -= (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() -= (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() -= (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() -= (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() -= (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() -= (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo_simd(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix3x3f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m10() += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m20() += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m21() += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void aBT(const AlignedVector3f &a, const AlignedMatrix3x3f &B,
                         AlignedVector3f &aBT) {
    aBT.v0() = (a.v012r() * B.m_00_01_02_r0()).vsum_012();
    aBT.v1() = (a.v012r() * B.m_10_11_12_r1()).vsum_012();
    aBT.v2() = (a.v012r() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline AlignedMatrix3x3f GetABT(const AlignedMatrix3x3f &A,
                                         const AlignedMatrix3x3f &B) {
    AlignedMatrix3x3f _ABT;
    ABT(A, B, _ABT);
    return _ABT;
  }
  static inline void ATB(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ATB) {
    ATB.m_00_01_02_r0() = B.m_00_01_02_r0() * A.m00() +
                          B.m_10_11_12_r1() * A.m10() +
                          B.m_20_21_22_r2() * A.m20();
    ATB.m_10_11_12_r1() = B.m_00_01_02_r0() * A.m01() +
                          B.m_10_11_12_r1() * A.m11() +
                          B.m_20_21_22_r2() * A.m21();
    ATB.m_20_21_22_r2() = B.m_00_01_02_r0() * A.m02() +
                          B.m_10_11_12_r1() * A.m12() +
                          B.m_20_21_22_r2() * A.m22();
  }
  static inline void AddATBTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix3x3f &ATB) {
    ATB.m_00_01_02_r0() += B.m_00_01_02_r0() * A.m00() +
                           B.m_10_11_12_r1() * A.m10() +
                           B.m_20_21_22_r2() * A.m20();
    ATB.m_10_11_12_r1() += B.m_00_01_02_r0() * A.m01() +
                           B.m_10_11_12_r1() * A.m11() +
                           B.m_20_21_22_r2() * A.m21();
    ATB.m_20_21_22_r2() += B.m_00_01_02_r0() * A.m02() +
                           B.m_10_11_12_r1() * A.m12() +
                           B.m_20_21_22_r2() * A.m22();
  }
  static inline void SubtractATBFrom(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                     AlignedMatrix3x3f &ATB) {
    ATB.m_00_01_02_r0() -= B.m_00_01_02_r0() * A.m00() +
                           B.m_10_11_12_r1() * A.m10() +
                           B.m_20_21_22_r2() * A.m20();
    ATB.m_10_11_12_r1() -= B.m_00_01_02_r0() * A.m01() +
                           B.m_10_11_12_r1() * A.m11() +
                           B.m_20_21_22_r2() * A.m21();
    ATB.m_20_21_22_r2() -= B.m_00_01_02_r0() * A.m02() +
                           B.m_10_11_12_r1() * A.m12() +
                           B.m_20_21_22_r2() * A.m22();
  }
  template<typename TYPE> static inline void ATb(const AlignedMatrix3x3f &A, const Vector2<TYPE> &b,
                                                 AlignedVector3f &ATb) {
    ATb.v012r() = A.m_00_01_02_r0() * b.v0() + A.m_10_11_12_r1() * b.v1() + A.m_20_21_22_r2();
  }
  template<typename TYPE> static inline LA::AlignedVector3f GetATb(const AlignedMatrix3x3f &A,
                                                                   const Vector2<TYPE> &b) {
    AlignedVector3f _ATb;
    ATb(A, b, _ATb);
    return _ATb;
  }

 protected:
  xp128f m_data[3];
};

template<typename TYPE> class Matrix3x3 {
 public:
  //inline Matrix3x3<TYPE>() {}
  //inline Matrix3x3<TYPE>(const TYPE *M) { Set(M); }

  inline const TYPE& m00() const { return m_data[0][0]; }    inline TYPE& m00() { return m_data[0][0]; }
  inline const TYPE& m01() const { return m_data[0][1]; }    inline TYPE& m01() { return m_data[0][1]; }
  inline const TYPE& m02() const { return m_data[0][2]; }    inline TYPE& m02() { return m_data[0][2]; }
  inline const TYPE& m10() const { return m_data[1][0]; }    inline TYPE& m10() { return m_data[1][0]; }
  inline const TYPE& m11() const { return m_data[1][1]; }    inline TYPE& m11() { return m_data[1][1]; }
  inline const TYPE& m12() const { return m_data[1][2]; }    inline TYPE& m12() { return m_data[1][2]; }
  inline const TYPE& m20() const { return m_data[2][0]; }    inline TYPE& m20() { return m_data[2][0]; }
  inline const TYPE& m21() const { return m_data[2][1]; }    inline TYPE& m21() { return m_data[2][1]; }
  inline const TYPE& m22() const { return m_data[2][2]; }    inline TYPE& m22() { return m_data[2][2]; }

  inline const TYPE* operator [] (const int i) const { return m_data[i]; }
  inline       TYPE* operator [] (const int i)       { return m_data[i]; }
  inline void operator = (const AlignedMatrix3x3f &M);

  inline void Set(const TYPE *M) { memcpy(this, M, sizeof(Matrix3x3<TYPE>)); }
  inline void SetLowerFromUpper() { m10() = m01(); m20() = m02(); m21() = m12(); }
  inline void GetTranspose(Matrix3x3<TYPE> &MT) const {
    MT.m00() = m00(); MT.m01() = m10(); MT.m02() = m20();
    MT.m10() = m01(); MT.m11() = m11(); MT.m12() = m21();
    MT.m20() = m02(); MT.m21() = m12(); MT.m22() = m22();
  }
  inline void Transpose() {
    UT_SWAP(m01(), m10());
    UT_SWAP(m02(), m20());
    UT_SWAP(m12(), m21());
  }
  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", m00(), m01(), m02());
      UT::Print("%e %e %e\n", m10(), m11(), m12());
      UT::Print("%e %e %e\n", m20(), m21(), m22());
    } else {
      UT::Print("%f %f %f\n", m00(), m01(), m02());
      UT::Print("%f %f %f\n", m10(), m11(), m12());
      UT::Print("%f %f %f\n", m20(), m21(), m22());
    }
  }

 public:
  TYPE m_data[3][3];
};

typedef Matrix3x3<float> Matrix3x3f;
typedef Matrix3x3<double> Matrix3x3d;

template<> inline void Matrix3x3f::operator = (const AlignedMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m10(), &M.m10(), 12);
  memcpy(&m20(), &M.m20(), 12);
}

template<typename TYPE> class SymmetricMatrix3x3 {
 public:

  //inline SymmetricMatrix3x3<TYPE>() {}
  //inline SymmetricMatrix3x3<TYPE>(const TYPE *M) { Set(M); }

  inline const TYPE& m00() const { return m_data[0]; }    inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }    inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m02() const { return m_data[2]; }    inline TYPE& m02() { return m_data[2]; }
  inline const TYPE& m10() const { return m_data[1]; }    inline TYPE& m10() { return m_data[1]; }
  inline const TYPE& m11() const { return m_data[3]; }    inline TYPE& m11() { return m_data[3]; }
  inline const TYPE& m12() const { return m_data[4]; }    inline TYPE& m12() { return m_data[4]; }
  inline const TYPE& m20() const { return m_data[2]; }    inline TYPE& m20() { return m_data[2]; }
  inline const TYPE& m21() const { return m_data[4]; }    inline TYPE& m21() { return m_data[4]; }
  inline const TYPE& m22() const { return m_data[5]; }    inline TYPE& m22() { return m_data[5]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  inline void operator *= (const TYPE s) { Scale(s); }
  inline void operator /= (const TYPE d) { Scale(1 / d); }
  inline void operator += (const SymmetricMatrix3x3<TYPE> &M) {
    m00() += M.m00();
    m01() += M.m01();
    m02() += M.m02();
    m11() += M.m11();
    m12() += M.m12();
    m22() += M.m22();
  }
  inline void operator += (const AlignedMatrix3x3f &M) {
    m00() += M.m00();
    m01() += M.m01();
    m02() += M.m02();
    m11() += M.m11();
    m12() += M.m12();
    m22() += M.m22();
  }
  inline SymmetricMatrix3x3<TYPE> operator * (const TYPE s) const {
    SymmetricMatrix3x3<TYPE> M;
    GetScaled(s, M);
    return M;
  }
  inline SymmetricMatrix3x3<TYPE> operator / (const TYPE s) const {
    SymmetricMatrix3x3<TYPE> M;
    GetScaled(1 / s, M);
    return M;
  }

  inline void Set(const TYPE *M) { memcpy(this, M, sizeof(SymmetricMatrix3x3<TYPE>)); }
  inline void Set(const AlignedMatrix3x3f &M);
  inline void SetRow0(const xp128f &v);
  inline void Get(AlignedMatrix3x3f &M) const {
    M.m_00_01_02_r0().vset_all_lane(static_cast<float>(m00()), static_cast<float>(m01()),
                                    static_cast<float>(m02()), 0.0f);
    M.m_10_11_12_r1().vset_all_lane(static_cast<float>(m01()), static_cast<float>(m11()),
                                    static_cast<float>(m12()), 0.0f);
    M.m_20_21_22_r2().vset_all_lane(static_cast<float>(m02()), static_cast<float>(m12()),
                                    static_cast<float>(m22()), 0.0f);
  }
  inline void Get(SymmetricMatrix3x3<float> &M) const;
  inline void Get(SymmetricMatrix3x3<double> &M) const;
  inline void Get(SymmetricMatrix2x2<TYPE> &M) const {
    M.m00() = m00();
    M.m01() = m01();
    M.m11() = m11();
  }

  inline AlignedMatrix3x3f GetAlignedMatrix3x3f() const { AlignedMatrix3x3f M; Get(M); return M; }

  inline void MakeIdentity() {
    MakeZero();
    m00() = m11() = m22() = 1.0f;
  }
  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix3x3<TYPE>)); }
  inline void MakeMinus() {
    m00() = -m00();
    m01() = -m01();
    m02() = -m02();
    m11() = -m11();
    m12() = -m12();
    m22() = -m22();
  }
  inline void GetMinus(AlignedMatrix3x3f &M) const {
    M.m00() = -m00();
    M.m01() = M.m10() = -m01();
    M.m02() = M.m20() = -m02();
    M.m11() = -m11();
    M.m12() = M.m21() = -m12();
    M.m22() = -m22();
  }
  inline TYPE Determinant() const {
    return m00() * (m11() * m22() - m12() * m12()) +
           m01() * (m12() * m02() - m01() * m22()) +
           m02() * (m01() * m12() - m11() * m02());
  }

  inline void Scale(const TYPE s) {
    m00() *= s;
    m01() *= s;
    m02() *= s;
    m11() *= s;
    m12() *= s;
    m22() *= s;
  }
  inline void GetScaled(const TYPE s, SymmetricMatrix3x3<TYPE> &M) const {
    M.m00() = m00() * s;
    M.m01() = m01() * s;
    M.m02() = m02() * s;
    M.m11() = m11() * s;
    M.m12() = m12() * s;
    M.m22() = m22() * s;
  }
  inline void GetScaled(const TYPE s, SymmetricMatrix2x2<TYPE> &M) const {
    M.m00() = m00() * s;
    M.m01() = m01() * s;
    M.m11() = m11() * s;
  }
  inline void GetScaled(const TYPE s, AlignedMatrix3x3f &M) const {
    M.m00() = m00() * s;
    M.m01() = M.m10() = m01() * s;
    M.m02() = M.m20() = m02() * s;
    M.m11() = m11() * s;
    M.m12() = M.m21() = m12() * s;
    M.m22() = m22() * s;
  }

  inline void MakeDiagonal(const TYPE d) { MakeZero(); SetDiagonal(d); }
  inline void MakeDiagonal(const SymmetricMatrix2x2<TYPE> &D0, const TYPE d1) {
    MakeZero();
    SetDiagonal(D0, d1);
  }
  inline void SetDiagonal(const TYPE d) { m00() = d; m11() = d; m22() = d; }
  inline void SetDiagonal(const SymmetricMatrix2x2<TYPE> &D0, const TYPE d1) {
    m00() = D0.m00();
    m01() = D0.m01();
    m11() = D0.m11();
    m22() = d1;
  }
  inline void GetDiagonal(AlignedVector3f &v) const {
    v.v012r() = xp128f::get(m00(), m11(), m22());
  }
  inline void ScaleDiagonal(const TYPE s) { m00() *= s; m11() *= s; m22() *= s; }
  inline void IncreaseDiagonal(const TYPE d) {
    m00() += d;
    m11() += d;
    m22() += d;
  }
  inline void IncreaseDiagonal(const TYPE d0, const TYPE d1, const TYPE d2) {
    m00() += d0;
    m11() += d1;
    m22() += d2;
  }
  inline void IncreaseDiagonal(const SymmetricMatrix2x2<TYPE> &D0, const TYPE d1) {
    m00() += D0.m00();
    m01() += D0.m01();
    m11() += D0.m11();
    m22() += d1;
  }

  inline bool GetInverse(SymmetricMatrix3x3<TYPE> &MI) const {
    MI.m00() = m11() * m22() - m12() * m12();
    MI.m01() = m02() * m12() - m01() * m22();
    MI.m02() = m01() * m12() - m02() * m11();
    MI.m11() = m00() * m22() - m02() * m02();
    MI.m12() = m02() * m01() - m00() * m12();
    MI.m22() = m00() * m11() - m01() * m01();
    const TYPE d = m00() * MI.m00() + m01() * MI.m01() + m02() * MI.m02();
    if (UT::IsNAN<TYPE>(d) || d <= 0) {
      MI.Invalidate();
      return false;
    } else {
      MI /= d;
      return true;
    }
  }
  inline SymmetricMatrix3x3<TYPE> GetInverse() const {
    SymmetricMatrix3x3<TYPE> MI;
    GetInverse(MI);
    return MI;
  }

  inline bool GetInverse(AlignedMatrix3x3f &MI, const TYPE mMax = 0, const TYPE eps = 0) const {
    if (mMax > 0 && (m00() > mMax || m11() > mMax || m22() > mMax)) {
      //MI.Invalidate();
      //return false;
      SymmetricMatrix3x3<double> M, _MI;
      Get(M);
      if (M.GetInverse(_MI)) {
        _MI.Get(MI);
        return true;
      } else {
        MI.Invalidate();
        return false;
      }
    }
    MI.m00() = static_cast<float>(m11() * m22() - m12() * m12());
    MI.m01() = static_cast<float>(m02() * m12() - m01() * m22());
    MI.m02() = static_cast<float>(m01() * m12() - m02() * m11());
    MI.m11() = static_cast<float>(m00() * m22() - m02() * m02());
    MI.m12() = static_cast<float>(m02() * m01() - m00() * m12());
    MI.m22() = static_cast<float>(m00() * m11() - m01() * m01());
    const TYPE d = m00() * MI.m00() + m01() * MI.m01() + m02() * MI.m02();
    if (UT::IsNAN<TYPE>(d) || d <= 0) {
      MI.Invalidate();
      return false;
    } else {
      const TYPE s = 1 / d;
      MI.m_00_01_02_r0() *= s;
      MI.m10() = MI.m01();
      MI.m20() = MI.m02();
      MI.m11() = static_cast<float>(MI.m11() * s);
      MI.m12() = MI.m21() = static_cast<float>(MI.m12() * s);
      MI.m22() = static_cast<float>(MI.m22() * s);
      if (eps > 0 &&
         (fabs(m00() * MI.m00() + m01() * MI.m10() + m02() * MI.m20() - 1) > eps ||
          fabs(m00() * MI.m01() + m01() * MI.m11() + m02() * MI.m21()    ) > eps ||
          fabs(m00() * MI.m02() + m01() * MI.m12() + m02() * MI.m22()    ) > eps ||
          fabs(m10() * MI.m01() + m11() * MI.m11() + m12() * MI.m21() - 1) > eps ||
          fabs(m10() * MI.m02() + m11() * MI.m12() + m12() * MI.m22()    ) > eps ||
          fabs(m20() * MI.m02() + m21() * MI.m12() + m22() * MI.m22() - 1) > eps)) {
        MI.Invalidate();
        return false;
      }
      return true;
    }
  }
  inline bool GetInverseLDL(AlignedMatrix3x3f &MI, const float *eps = NULL) const {
    Get(MI);
    return MI.InverseLDL(eps);
  }

  inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e\n", m00(), m01(), m02());
      UT::Print("%e %e %e\n", m10(), m11(), m12());
      UT::Print("%e %e %e\n", m20(), m21(), m22());
    } else {
      UT::Print("%f %f %f\n", m00(), m01(), m02());
      UT::Print("%f %f %f\n", m10(), m11(), m12());
      UT::Print("%f %f %f\n", m20(), m21(), m22());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    if (e) {
      UT::Print("%s%e %e %e\n",  str.c_str(), m00(), m01(), m02());
      UT::Print("%s%e %e %e\n", _str.c_str(), m10(), m11(), m12());
      UT::Print("%s%e %e %e\n", _str.c_str(), m20(), m21(), m22());
    } else {
      UT::Print("%s%f %f %f\n",  str.c_str(), m00(), m01(), m02());
      UT::Print("%s%f %f %f\n", _str.c_str(), m10(), m11(), m12());
      UT::Print("%s%f %f %f\n", _str.c_str(), m20(), m21(), m22());
    }
  }

  static inline float MahalanobisDistance(const AlignedMatrix3x3f &W, const AlignedVector3f &d) {
#ifdef CFG_DEBUG
    W.AssertSymmetric();
#endif
    const LA::AlignedVector3f Wd = W * d;
    return d.Dot(Wd);
  }
  static inline TYPE MahalanobisDistance(const SymmetricMatrix3x3<TYPE> &W, const Vector2<TYPE> &d0,
                                         const TYPE d1) {
    return (W.m00() * d0.v0() + W.m01() * d0.v1() + W.m02() * d1) * d0.v0() +
           (W.m01() * d0.v0() + W.m11() * d0.v1() + W.m12() * d1) * d0.v1() +
           (W.m02() * d0.v0() + W.m12() * d0.v1() + W.m22() * d1) * d1;
  }
  static inline TYPE MahalanobisDistance(const SymmetricMatrix3x3<TYPE> &W, const Vector2<TYPE> &d) {
    return (W.m00() * d.v0() + W.m01() * d.v1()) * d.v0() +
           (W.m01() * d.v0() + W.m11() * d.v1()) * d.v1();
  }

  
  static inline void AddATo(const SymmetricMatrix3x3<TYPE> &A, AlignedMatrix3x3f &SA) {
    SA.m00() += A.m00();  SA.m01() += A.m01();  SA.m02() += A.m02();
                          SA.m11() += A.m11();  SA.m12() += A.m12();
                                                SA.m22() += A.m22();
  }
  static inline void aaT(const AlignedVector3f &a, TYPE *aaT);
  static inline void aaT(const Vector2<TYPE> &a0, const TYPE a1,
                         SymmetricMatrix3x3<TYPE> &aaT) {
    aaT.m00() = a0.v0() * a0.v0();  aaT.m01() = a0.v0() * a0.v1();  aaT.m02() = a0.v0() * a1;
    aaT.m11() = a0.v1() * a0.v1();  aaT.m12() = a0.v1() * a1;
    aaT.m22() = a1 * a1;
  }
  static inline void Ab(const SymmetricMatrix2x2<TYPE> &A0, const TYPE A1, const AlignedVector3f &b,
                        AlignedVector3f &Ab) {
    Ab.v0() = A0.m00() * b.v0() + A0.m01() * b.v1();
    Ab.v1() = A0.m01() * b.v0() + A0.m11() * b.v1();
    Ab.v2() = A1 * b.v2();
  }
  static inline void AAT(const AlignedMatrix3x3f &A, SymmetricMatrix3x3<TYPE> &AAT) {
    AAT.m00() = (A.m_00_01_02_r0() * A.m_00_01_02_r0()).vsum_012();
    AAT.m01() = (A.m_00_01_02_r0() * A.m_10_11_12_r1()).vsum_012();
    AAT.m02() = (A.m_00_01_02_r0() * A.m_20_21_22_r2()).vsum_012();
    AAT.m11() = (A.m_10_11_12_r1() * A.m_10_11_12_r1()).vsum_012();
    AAT.m12() = (A.m_10_11_12_r1() * A.m_20_21_22_r2()).vsum_012();
    AAT.m22() = (A.m_20_21_22_r2() * A.m_20_21_22_r2()).vsum_012();
  }
  static inline void AAT(const AlignedMatrix3x3f &A, AlignedMatrix3x3f &AAT) {
    AAT.m00() = (A.m_00_01_02_r0() * A.m_00_01_02_r0()).vsum_012();
    AAT.m01() = (A.m_00_01_02_r0() * A.m_10_11_12_r1()).vsum_012();
    AAT.m02() = (A.m_00_01_02_r0() * A.m_20_21_22_r2()).vsum_012();
    AAT.m11() = (A.m_10_11_12_r1() * A.m_10_11_12_r1()).vsum_012();
    AAT.m12() = (A.m_10_11_12_r1() * A.m_20_21_22_r2()).vsum_012();
    AAT.m22() = (A.m_20_21_22_r2() * A.m_20_21_22_r2()).vsum_012();
    AAT.SetLowerFromUpper();
  }
  static inline void ABT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         SymmetricMatrix3x3<TYPE> &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                         AlignedMatrix3x3f &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
    ABT.SetLowerFromUpper();
  }
  static inline void AddABTTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              SymmetricMatrix3x3<TYPE> &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                              AlignedMatrix3x3f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
    ABT.SetLowerFromUpper();
  }
  static inline void AddsATo(const xp128f &s, const AlignedMatrix3x3f &A,
                             SymmetricMatrix3x3<TYPE> &sA) {
    const xp128f sa0 = s * A.m_00_01_02_r0();
    sA.m00() += sa0[0];
    sA.m01() += sa0[1];
    sA.m02() += sa0[2];
    sA.m11() += s[0] * A.m11();
    sA.m12() += s[0] * A.m12();
    sA.m22() += s[0] * A.m22();
  }
  static inline void sAAT(const xp128f &s, const AlignedMatrix3x3f &A, AlignedMatrix3x3f &sA,
                          SymmetricMatrix3x3<TYPE> &sAAT) {
    A.GetScaled(s, sA);
    ABT(sA, A, sAAT);
  }
  static inline void sAAT(const xp128f &s, const AlignedMatrix3x3f &A,
                          AlignedMatrix3x3f &sA, AlignedMatrix3x3f &sAAT) {
    A.GetScaled(s, sA);
    ABT(sA, A, sAAT);
  }
  static inline void AddsAATTo(const xp128f &s, const AlignedMatrix3x3f &A,
                               AlignedMatrix3x3f &sA, SymmetricMatrix3x3<TYPE> &sAAT) {
    A.GetScaled(s, sA);
    AddABTTo(sA, A, sAAT);
  }
  static inline void ASAT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &S,
                          AlignedMatrix3x3f &AS, SymmetricMatrix3x3<TYPE> &ASAT) {
#ifdef CFG_DEBUG
    S.AssertSymmetric();
#endif
    AlignedMatrix3x3f::ABT(A, S, AS);
    ABT(AS, A, ASAT);
  }
  static inline void ASAT(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &S,
                          AlignedMatrix3x3f &AS, AlignedMatrix3x3f &ASAT) {
#ifdef CFG_DEBUG
    S.AssertSymmetric();
#endif
    AlignedMatrix3x3f::ABT(A, S, AS);
    ABT(AS, A, ASAT);
  }
  static inline void AddASATTo(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &S,
                               AlignedMatrix3x3f &AS, SymmetricMatrix3x3<TYPE> &ASAT) {
#ifdef CFG_DEBUG
    S.AssertSymmetric();
#endif
    AlignedMatrix3x3f::ABT(A, S, AS);
    AddABTTo(AS, A, ASAT);
  }
  static inline void ApB(const SymmetricMatrix3x3<TYPE> &A, const SymmetricMatrix3x3<TYPE> &B,
                         SymmetricMatrix3x3<TYPE> &ApB) {
    ApB.m00() = A.m00() + B.m00();
    ApB.m01() = A.m01() + B.m01();
    ApB.m02() = A.m02() + B.m02();
    ApB.m11() = A.m11() + B.m11();
    ApB.m12() = A.m12() + B.m12();
    ApB.m22() = A.m22() + B.m22();
  }

 protected:
  TYPE m_data[6];
};

typedef SymmetricMatrix3x3<float> SymmetricMatrix3x3f;
typedef SymmetricMatrix3x3<double> SymmetricMatrix3x3d;

template<> inline void SymmetricMatrix3x3f::Set(const AlignedMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m11(), &M.m11(), 8);
  m22() = M.m22();
}
template<> inline void SymmetricMatrix3x3d::Set(const AlignedMatrix3x3f &M) {
  m00() = static_cast<double>(M.m00());
  m01() = static_cast<double>(M.m01());
  m02() = static_cast<double>(M.m02());
  m11() = static_cast<double>(M.m11());
  m12() = static_cast<double>(M.m12());
  m22() = static_cast<double>(M.m22());
}
template<> inline void SymmetricMatrix3x3f::SetRow0(const xp128f &v) { memcpy(&m00(), &v, 12); }
template<> inline void SymmetricMatrix3x3f::Get(SymmetricMatrix3x3f &M) const { M = *this; }
template<> inline void SymmetricMatrix3x3f::Get(SymmetricMatrix3x3d &M) const {
  M.m00() = static_cast<double>(m00());
  M.m01() = static_cast<double>(m01());
  M.m02() = static_cast<double>(m02());
  M.m11() = static_cast<double>(m11());
  M.m12() = static_cast<double>(m12());
  M.m22() = static_cast<double>(m22());
}
template<> inline void SymmetricMatrix3x3d::Get(SymmetricMatrix3x3f &M) const {
  M.m00() = static_cast<float>(m00());
  M.m01() = static_cast<float>(m01());
  M.m02() = static_cast<float>(m02());
  M.m11() = static_cast<float>(m11());
  M.m12() = static_cast<float>(m12());
  M.m22() = static_cast<float>(m22());
}
template<> inline void SymmetricMatrix3x3d::Get(SymmetricMatrix3x3d &M) const { M = *this; }
template<> inline void SymmetricMatrix3x3f::aaT(const LA::AlignedVector3f &a, float *aaT) {
  const xp128f t = a.v012r() * a.v0();
  memcpy(aaT, &t, 12);
  aaT[3] = a.v1() * a.v1();
  aaT[4] = a.v1() * a.v2();
  aaT[5] = a.v2() * a.v2();
}
}  // namespace LA

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix3x3f : public Eigen::Matrix3f {
 public:
  inline EigenMatrix3x3f() : Eigen::Matrix3f() {}
  explicit inline EigenMatrix3x3f(const Eigen::Matrix3f &e_M) : Eigen::Matrix3f(e_M) {}
  explicit inline EigenMatrix3x3f(const LA::AlignedMatrix3x3f &M) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_M = *this;
    e_M(0, 0) = M.m00();  e_M(0, 1) = M.m01();  e_M(0, 2) = M.m02();
    e_M(1, 0) = M.m10();  e_M(1, 1) = M.m11();  e_M(1, 2) = M.m12();
    e_M(2, 0) = M.m20();  e_M(2, 1) = M.m21();  e_M(2, 2) = M.m22();
  }
  explicit inline EigenMatrix3x3f(const LA::SymmetricMatrix3x3f &M) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_M = *this;
    e_M(0, 0) = M.m00();  e_M(0, 1) = M.m01();  e_M(0, 2) = M.m02();
    e_M(1, 0) = M.m01();  e_M(1, 1) = M.m11();  e_M(1, 2) = M.m12();
    e_M(2, 0) = M.m02();  e_M(2, 1) = M.m12();  e_M(2, 2) = M.m22();
  }
  inline EigenMatrix3x3f(const LA::SymmetricMatrix2x2f &M0, const float M1) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_M = *this;
    e_M(0, 0) = M0.m00(); e_M(0, 1) = M0.m01(); e_M(0, 2) = 0.0f;
    e_M(1, 0) = M0.m01(); e_M(1, 1) = M0.m11(); e_M(1, 2) = 0.0f;
    e_M(2, 0) = 0.0f;   e_M(2, 1) = 0.0f;   e_M(2, 2) = M1;
  }
  explicit inline EigenMatrix3x3f(const float m) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_M = *this;
    e_M.setZero();
    e_M(0, 0) = m;
    e_M(1, 1) = m;
    e_M(2, 2) = m;
  }
  inline EigenMatrix3x3f(const float m0, const float m1, const float m2) : Eigen::Matrix3f() {
    Eigen::Matrix3f &e_M = *this;
    e_M.setZero();
    e_M(0, 0) = m0;
    e_M(1, 1) = m1;
    e_M(2, 2) = m2;
  }
  inline void operator = (const Eigen::Matrix3f &e_M) { *((Eigen::Matrix3f *) this) = e_M; }
  inline LA::AlignedMatrix3x3f GetAlignedMatrix3x3f() const {
    LA::AlignedMatrix3x3f M;
    const Eigen::Matrix3f &e_M = *this;
    M.m00() = e_M(0, 0);  M.m01() = e_M(0, 1);  M.m02() = e_M(0, 2);
    M.m10() = e_M(1, 0);  M.m11() = e_M(1, 1);  M.m12() = e_M(1, 2);
    M.m20() = e_M(2, 0);  M.m21() = e_M(2, 1);  M.m22() = e_M(2, 2);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix3x3f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix3x3f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix3x3f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::SymmetricMatrix3x3f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix3x3f().AssertEqual(
                M.GetAlignedMatrix3x3f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix3x3f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix3x3f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix3x3f().AssertZero(verbose, str, epsAbs, epsRel);
  }
  static inline EigenMatrix3x3f Zero() {
    EigenMatrix3x3f e_M;
    e_M.setZero();
    return e_M;
  }
  static inline EigenMatrix3x3f Identity() {
    EigenMatrix3x3f e_I;
    e_I.setIdentity();
    return e_I;
  }
};
#endif
#endif  // LINEARALGEBRA_MATRIX3X3_H_
