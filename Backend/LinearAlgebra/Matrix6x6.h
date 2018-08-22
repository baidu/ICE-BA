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
#ifndef LINEARALGEBRA_MATRIX6X6_H_
#define LINEARALGEBRA_MATRIX6X6_H_

#include "Vector6.h"
#include "Matrix2x3.h"
#include "Matrix2x6.h"
#include "Matrix3x3.h"
#include "LinearSystem.h"
#include <string>

namespace LA {

class AlignedMatrix6x6f {
 public:
  inline const xp128f& m_00_01_02_03() const { return m_data4[0]; } inline xp128f& m_00_01_02_03() { return m_data4[0]; }
  inline const xp128f& m_04_05_10_11() const { return m_data4[1]; } inline xp128f& m_04_05_10_11() { return m_data4[1]; }
  inline const xp128f& m_12_13_14_15() const { return m_data4[2]; } inline xp128f& m_12_13_14_15() { return m_data4[2]; }
  inline const xp128f& m_20_21_22_23() const { return m_data4[3]; } inline xp128f& m_20_21_22_23() { return m_data4[3]; }
  inline const xp128f& m_24_25_30_31() const { return m_data4[4]; } inline xp128f& m_24_25_30_31() { return m_data4[4]; }
  inline const xp128f& m_32_33_34_35() const { return m_data4[5]; } inline xp128f& m_32_33_34_35() { return m_data4[5]; }
  inline const xp128f& m_40_41_42_43() const { return m_data4[6]; } inline xp128f& m_40_41_42_43() { return m_data4[6]; }
  inline const xp128f& m_44_45_50_51() const { return m_data4[7]; } inline xp128f& m_44_45_50_51() { return m_data4[7]; }
  inline const xp128f& m_52_53_54_55() const { return m_data4[8]; } inline xp128f& m_52_53_54_55() { return m_data4[8]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float* operator[] (const int i) const { return m_data[i]; }
  inline       float* operator[] (const int i)       { return m_data[i]; }

  inline const float& operator() (int row, int col) const {
    return m_data[row][col];
  }

  inline float& operator() (int row, int col) {
    return m_data[row][col];
  }

  inline void operator += (const AlignedMatrix6x6f& A) {
    for (int i = 0; i < 9; ++i) {
      m_data4[i] += A.m_data4[i];
    }
  }
  inline void operator -= (const AlignedMatrix6x6f &A) {
    for (int i = 0; i < 9; ++i) {
      m_data4[i] -= A.m_data4[i];
    }
  }
  inline void operator = (const AlignedMatrix6x6f &A) {
    if (this != &A) {
      // TODO(yanghongtian) : NEON implementation??
      memcpy(m_data, A.m_data, sizeof(AlignedMatrix6x6f));
    }
  }
  inline AlignedMatrix6x6f operator - (const AlignedMatrix6x6f &B) const {
    AlignedMatrix6x6f _AmB;
    AmB(*this, B, _AmB);
    return _AmB;
  }
  inline void operator *= (const float s) { const xp128f _s = xp128f::get(s); Scale(_s); }
  inline void operator *= (const xp128f &s) { Scale(s); }
  inline AlignedMatrix6x6f operator * (const xp128f &s) const {
    AlignedMatrix6x6f A;
    GetScaled(s, A);
    return A;
  }
  inline bool operator == (const AlignedMatrix6x6f &M) const {
    bool equal = true;
    for (int i = 0; i < 6 && equal; ++i)
      for (int j = 0; j < 6 && equal; ++j)
        equal = m_data[i][j] == M.m_data[i][j];
    return equal;
  }
  inline void Set(const float M[6][6]) { memcpy(&m_data[0][0], &M[0][0], sizeof(m_data)); }
  inline void Set(const SymmetricMatrix3x3f &M00, const AlignedMatrix3x3f &M01,
                  const SymmetricMatrix3x3f &M11) {
    Set00(M00);
    Set03(M01);
    Set33(M11);
    SetLowerFromUpper();
  }
  inline void Set(const AlignedMatrix3x3f &M00, const AlignedMatrix3x3f &M01,
                  const AlignedMatrix3x3f &M10, const AlignedMatrix3x3f &M11) {
    Set00(M00);   Set03(M01);
    Set30(M10);   Set33(M11);
  }
  inline void Set(const Matrix3x3f &M00, const Matrix3x3f &M01,
                  const Matrix3x3f &M10, const Matrix3x3f &M11) {
    Set00(M00);   Set03(M01);
    Set30(M10);   Set33(M11);
  }
  inline void Set(const AlignedMatrix3x3f *M0, const AlignedMatrix3x3f *M1) {
    Set(M0[0], M0[1], M1[0], M1[1]);
  }
  inline void Set00(const AlignedMatrix3x3f &M) {
    memcpy(m_data[0], &M.m00(), 12);
    memcpy(m_data[1], &M.m10(), 12);
    memcpy(m_data[2], &M.m20(), 12);
  }
  inline void Set00(const Matrix3x3f &M) {
    memcpy(m_data[0], M[0], 12);
    memcpy(m_data[1], M[1], 12);
    memcpy(m_data[2], M[2], 12);
  }
  inline void Set00(const SymmetricMatrix3x3f &M) {
    memcpy(m_data[0], &M.m00(), 12);
    memcpy(m_data[1] + 1, &M.m11(), 8);
    m_data[2][2] = M.m22();
  }
  inline void Set03(const AlignedMatrix3x3f &M) {
    memcpy(m_data[0] + 3, &M.m00(), 12);
    memcpy(m_data[1] + 3, &M.m10(), 12);
    memcpy(m_data[2] + 3, &M.m20(), 12);
  }
  inline void Set03(const Matrix3x3f &M) {
    memcpy(m_data[0] + 3, M[0], 12);
    memcpy(m_data[1] + 3, M[1], 12);
    memcpy(m_data[2] + 3, M[2], 12);
  }
  inline void Set30(const AlignedMatrix3x3f &M) {
    memcpy(m_data[3], &M.m00(), 12);
    memcpy(m_data[4], &M.m10(), 12);
    memcpy(m_data[5], &M.m20(), 12);
  }
  inline void Set30(const Matrix3x3f &M) {
    memcpy(m_data[3], M[0], 12);
    memcpy(m_data[4], M[1], 12);
    memcpy(m_data[5], M[2], 12);
  }
  inline void Set33(const AlignedMatrix3x3f &M) {
    memcpy(m_data[3] + 3, &M.m00(), 12);
    memcpy(m_data[4] + 3, &M.m10(), 12);
    memcpy(m_data[5] + 3, &M.m20(), 12);
  }
  inline void Set33(const Matrix3x3f &M) {
    memcpy(m_data[3] + 3, M[0], 12);
    memcpy(m_data[4] + 3, M[1], 12);
    memcpy(m_data[5] + 3, M[2], 12);
  }
  inline void Set33(const SymmetricMatrix3x3f &M) {
    memcpy(m_data[3] + 3, &M.m00(), 12);
    memcpy(m_data[4] + 4, &M.m11(), 8);
    m_data[5][5] = M.m22();
  }
  inline void Get(float M[6][6]) const { memcpy(&M[0][0], &m_data[0][0], sizeof(m_data)); }
  inline void Get(AlignedMatrix3x3f &M00, AlignedMatrix3x3f &M01,
                  AlignedMatrix3x3f &M10, AlignedMatrix3x3f &M11) const {
    Get00(M00);   Get03(M01);
    Get30(M10);   Get33(M11);
  }
  inline void Get(Matrix3x3f &M00, Matrix3x3f &M01,
                  Matrix3x3f &M10, Matrix3x3f &M11) const {
    Get00(M00);   Get03(M01);
    Get30(M10);   Get33(M11);
  }
  inline void Get00(AlignedMatrix3x3f &M) const {
    memcpy(&M.m00(), m_data[0], 12);
    memcpy(&M.m10(), m_data[1], 12);
    memcpy(&M.m20(), m_data[2], 12);
  }
  inline void Get00(Matrix3x3f &M) const {
    memcpy(M[0], m_data[0], 12);
    memcpy(M[1], m_data[1], 12);
    memcpy(M[2], m_data[2], 12);
  }
  inline void Get03(AlignedMatrix3x3f &M) const {
    memcpy(&M.m00(), m_data[0] + 3, 12);
    memcpy(&M.m10(), m_data[1] + 3, 12);
    memcpy(&M.m20(), m_data[2] + 3, 12);
  }
  inline void Get03(Matrix3x3f &M) const {
    memcpy(M[0], m_data[0] + 3, 12);
    memcpy(M[1], m_data[1] + 3, 12);
    memcpy(M[2], m_data[2] + 3, 12);
  }
  inline void Get30(AlignedMatrix3x3f &M) const {
    memcpy(&M.m00(), m_data[3], 12);
    memcpy(&M.m10(), m_data[4], 12);
    memcpy(&M.m20(), m_data[5], 12);
  }
  inline void Get30(Matrix3x3f &M) const {
    memcpy(M[0], m_data[3], 12);
    memcpy(M[1], m_data[4], 12);
    memcpy(M[2], m_data[5], 12);
  }
  inline void Get33(AlignedMatrix3x3f &M) const {
    memcpy(&M.m00(), m_data[3] + 3, 12);
    memcpy(&M.m10(), m_data[4] + 3, 12);
    memcpy(&M.m20(), m_data[5] + 3, 12);
  }
  inline void Get33(Matrix3x3f &M) const {
    memcpy(M[0], m_data[3] + 3, 12);
    memcpy(M[1], m_data[4] + 3, 12);
    memcpy(M[2], m_data[5] + 3, 12);
  }
  inline void GetDiagonal(LA::Vector6f &d) const {
    d.v0() = m_data[0][0];  d.v1() = m_data[1][1];  d.v2() = m_data[2][2];
    d.v3() = m_data[3][3];  d.v4() = m_data[4][4];  d.v5() = m_data[5][5];
  }
  inline void GetDiagonal(LA::Vector3f &d012, LA::Vector3f &d345) const {
    d012.v0() = m_data[0][0];  d012.v1() = m_data[1][1];  d012.v2() = m_data[2][2];
    d345.v0() = m_data[3][3];  d345.v1() = m_data[4][4];  d345.v2() = m_data[5][5];
  }
  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix6x6f)); }
  inline void MakeZero3x3() {
    memset(m_data[0], 0, 12);
    memset(m_data[1], 0, 12);
    memset(m_data[2], 0, 12);
  }
  inline void MakeZero3x6() { memset(this, 0, 72); }
  inline void MakeMinus() {
    for (int i = 0; i < 9; ++i) {
      m_data4[i].vmake_minus();
    }
  }
  inline void GetMinus(AlignedMatrix6x6f &A) const {
    const xp128f zero = xp128f::get(0.0f);
    for (int i = 0; i < 9; ++i) {
      A.m_data4[i] = zero - m_data4[i];
    }
  }
  inline void MakeIdentity() {
    MakeZero();
    m_data[0][0] = 1.f;
    m_data[1][1] = 1.f;
    m_data[2][2] = 1.f;
    m_data[3][3] = 1.f;
    m_data[4][4] = 1.f;
    m_data[5][5] = 1.f;
  }
  inline void MakeDiagonal(const float d012, const float d345) {
    MakeZero();
    SetDiagonal(d012, d345);
  }
  inline void MakeDiagonal(const AlignedVector3f &d012, const AlignedVector3f &d345) {
    MakeZero();
    SetDiagonal(d012, d345);
  }
  inline void SetDiagonal(const float d012, const float d345) {
    m_data[0][0] = m_data[1][1] = m_data[2][2] = d012;
    m_data[3][3] = m_data[4][4] = m_data[5][5] = d345;
  }
  inline void SetDiagonal(const AlignedVector3f &d012, const AlignedVector3f &d345) {
    m_data[0][0] = d012.v0();
    m_data[1][1] = d012.v1();
    m_data[2][2] = d012.v2();
    m_data[3][3] = d345.v0();
    m_data[4][4] = d345.v1();
    m_data[5][5] = d345.v2();
  }
  inline void IncreaseDiagonal(const float d012, const float d345) {
    m_data[0][0] += d012;
    m_data[1][1] += d012;
    m_data[2][2] += d012;
    m_data[3][3] += d345;
    m_data[4][4] += d345;
    m_data[5][5] += d345;
  }
  inline void Increase00(const SymmetricMatrix3x3f &M) {
    m_data[0][0] += M.m00();  m_data[0][1] += M.m01();  m_data[0][2] += M.m02();
                              m_data[1][1] += M.m11();  m_data[1][2] += M.m12();
                                                        m_data[2][2] += M.m22();
  }
  inline void Increase03(const AlignedMatrix3x3f &M) {
    m_data[0][3] += M.m00();  m_data[0][4] += M.m01();  m_data[0][5] += M.m02();
    m_data[1][3] += M.m10();  m_data[1][4] += M.m11();  m_data[1][5] += M.m12();
    m_data[2][3] += M.m20();  m_data[2][4] += M.m21();  m_data[2][5] += M.m22();
  }
  inline void Increase33(const SymmetricMatrix3x3f &M) {
    m_data[3][3] += M.m00();  m_data[3][4] += M.m01();  m_data[3][5] += M.m02();
                              m_data[4][4] += M.m11();  m_data[4][5] += M.m12();
                                                        m_data[5][5] += M.m22();
  }

  inline bool Valid() const { return m_data[0][0] != FLT_MAX; }
  inline bool Invalid() const { return m_data[0][0] == FLT_MAX; }
  inline void Invalidate() { m_data[0][0] = FLT_MAX; }

  inline void SetLowerFromUpper() {
    m_data[1][0] = m_data[0][1];
    m_data[2][0] = m_data[0][2];    m_data[2][1] = m_data[1][2];
    m_data[3][0] = m_data[0][3];    m_data[3][1] = m_data[1][3];    m_data[3][2] = m_data[2][3];
    m_data[4][0] = m_data[0][4];    m_data[4][1] = m_data[1][4];    m_data[4][2] = m_data[2][4];
    m_data[4][3] = m_data[3][4];
    m_data[5][0] = m_data[0][5];    m_data[5][1] = m_data[1][5];    m_data[5][2] = m_data[2][5];
    m_data[5][3] = m_data[3][5];    m_data[5][4] = m_data[4][5];
  }
  inline void GetTranspose(AlignedMatrix6x6f &M) const {
    M.m_data[0][0] = m_data[0][0];  M.m_data[0][1] = m_data[1][0];  M.m_data[0][2] = m_data[2][0];
    M.m_data[0][3] = m_data[3][0];  M.m_data[0][4] = m_data[4][0];  M.m_data[0][5] = m_data[5][0];
    M.m_data[1][0] = m_data[0][1];  M.m_data[1][1] = m_data[1][1];  M.m_data[1][2] = m_data[2][1];
    M.m_data[1][3] = m_data[3][1];  M.m_data[1][4] = m_data[4][1];  M.m_data[1][5] = m_data[5][1];
    M.m_data[2][0] = m_data[0][2];  M.m_data[2][1] = m_data[1][2];  M.m_data[2][2] = m_data[2][2];
    M.m_data[2][3] = m_data[3][2];  M.m_data[2][4] = m_data[4][2];  M.m_data[2][5] = m_data[5][2];
    M.m_data[3][0] = m_data[0][3];  M.m_data[3][1] = m_data[1][3];  M.m_data[3][2] = m_data[2][3];
    M.m_data[3][3] = m_data[3][3];  M.m_data[3][4] = m_data[4][3];  M.m_data[3][5] = m_data[5][3];
    M.m_data[4][0] = m_data[0][4];  M.m_data[4][1] = m_data[1][4];  M.m_data[4][2] = m_data[2][4];
    M.m_data[4][3] = m_data[3][4];  M.m_data[4][4] = m_data[4][4];  M.m_data[4][5] = m_data[5][4];
    M.m_data[5][0] = m_data[0][5];  M.m_data[5][1] = m_data[1][5];  M.m_data[5][2] = m_data[2][5];
    M.m_data[5][3] = m_data[3][5];  M.m_data[5][4] = m_data[4][5];  M.m_data[5][5] = m_data[5][5];
  }
  inline void Transpose() {
    UT_SWAP(m_data[0][1], m_data[1][0]);
    UT_SWAP(m_data[0][2], m_data[2][0]);
    UT_SWAP(m_data[0][3], m_data[3][0]);
    UT_SWAP(m_data[0][4], m_data[4][0]);
    UT_SWAP(m_data[0][5], m_data[5][0]);
    UT_SWAP(m_data[1][2], m_data[2][1]);
    UT_SWAP(m_data[1][3], m_data[3][1]);
    UT_SWAP(m_data[1][4], m_data[4][1]);
    UT_SWAP(m_data[1][5], m_data[5][1]);
    UT_SWAP(m_data[2][3], m_data[3][2]);
    UT_SWAP(m_data[2][4], m_data[4][2]);
    UT_SWAP(m_data[2][5], m_data[5][2]);
    UT_SWAP(m_data[3][4], m_data[4][3]);
    UT_SWAP(m_data[3][5], m_data[5][3]);
    UT_SWAP(m_data[4][5], m_data[5][4]);
  }

  inline void Scale(const xp128f &s) {
    for (int i = 0; i < 9; ++i) {
      m_data4[i] *= s;
    }
  }
  inline void GetScaled(const xp128f &s, AlignedMatrix6x6f &M) const {
    for (int i = 0; i < 9; ++i) {
      M.m_data4[i] = m_data4[i] * s;
    }
  }
  inline void GetScaledColumn(const ProductVector6f &s, AlignedMatrix6x6f &M) const {
    M.m_00_01_02_03() = m_00_01_02_03() * s.v0123();
    M.m_04_05_10_11() = m_04_05_10_11() * s.v4501();
    M.m_12_13_14_15() = m_12_13_14_15() * s.v2345();
    M.m_20_21_22_23() = m_20_21_22_23() * s.v0123();
    M.m_24_25_30_31() = m_24_25_30_31() * s.v4501();
    M.m_32_33_34_35() = m_32_33_34_35() * s.v2345();
    M.m_40_41_42_43() = m_40_41_42_43() * s.v0123();
    M.m_44_45_50_51() = m_44_45_50_51() * s.v4501();
    M.m_52_53_54_55() = m_52_53_54_55() * s.v2345();
  }
  inline void ScaleRow(const Vector6f &s) {
    m_00_01_02_03() *= s.v0();
    m_04_05_10_11() *= xp128f::get(s.v0(), s.v0(), s.v1(), s.v1());
    m_12_13_14_15() *= s.v1();
    m_20_21_22_23() *= s.v2();
    m_24_25_30_31() *= xp128f::get(s.v2(), s.v2(), s.v3(), s.v3());
    m_32_33_34_35() *= s.v3();
    m_40_41_42_43() *= s.v4();
    m_44_45_50_51() *= xp128f::get(s.v4(), s.v4(), s.v5(), s.v5());
    m_52_53_54_55() *= s.v5();
  }

  inline bool InverseLDL(const float *eps = NULL) { return InverseLDL(*this, eps); }
  static inline bool InverseLDL(AlignedMatrix6x6f &A, const float *eps = NULL) {
    float* _A[6] = {A[0], A[1], A[2], A[3], A[4], A[5]};
    if (LS::InverseLDL<float>(6, _A, eps)) {
      A.SetLowerFromUpper();
      return true;
    } else {
      A.Invalidate();
      return false;
    }
  }
  inline bool GetInverseLDL(AlignedMatrix6x6f &A, const float *eps = NULL) const {
    A = *this;
    return A.InverseLDL(eps);
  }
  inline AlignedMatrix6x6f GetInverseLDL(const float *eps = NULL) const {
    AlignedMatrix6x6f A = *this;
    A.InverseLDL(eps);
    return A;
  }
  static inline bool SolveLDL(AlignedMatrix6x6f &A, AlignedVector6f &b, const float *eps = NULL) {
    float* _A[6] = {A[0], A[1], A[2], A[3], A[4], A[5]};
    return LS::SolveLDL(6, _A, &b.v0(), eps);
  }

  inline void Print(const bool e = false) const {
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        if (e) {
          UT::Print("%e ", m_data[i][j]);
        } else {
          UT::Print("%f ", m_data[i][j]);
        }
      }
      UT::Print("\n");
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    for (int i = 0; i < 6; ++i) {
      UT::Print("%s", i == 0 ? str.c_str() : _str.c_str());
      for (int j = 0; j < 6; ++j) {
        if (e) {
          UT::Print("%e ", m_data[i][j]);
        } else {
          UT::Print("%f ", m_data[i][j]);
        }
      }
      UT::Print("\n");
    }
  }
  inline void PrintDiagonal(const bool e = false) const {
    for (int i = 0; i < 6; ++i) {
      if (e) {
        UT::Print("%e ", m_data[i][i]);
      } else {
        UT::Print("%f ", m_data[i][i]);
      }
    }
    UT::Print("\n");
  }
  inline void PrintDiagonal(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    PrintDiagonal(e);
  }
  inline bool AssertEqual(const AlignedMatrix6x6f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m_data[0][0], &M[0][0], 36, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix6x6f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m_data[0][0], 36, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }
  inline void AssertSymmetric() const {
    for (int i = 0; i < 6; ++i) {
      for (int j = i + 1; j < 6; ++j) {
        UT_ASSERT(m_data[i][j] == m_data[j][i]);
      }
    }
  }
  inline void AssertSymmetric(const AlignedMatrix6x6f &M) const {
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        UT_ASSERT(m_data[i][j] == M.m_data[j][i]);
      }
    }
  }

  inline void Random(const float mMax) { Random(-mMax, mMax); }
  inline void Random(const float mMin, const float mMax) {
   UT::Random(&m_data[0][0], 36, mMin, mMax);
  }
  static inline AlignedMatrix6x6f GetRandom(const float mMax) {
    AlignedMatrix6x6f M;
    M.Random(mMax);
    return M;
  }
  static inline AlignedMatrix6x6f GetRandom(const float mMin, const float mMax) {
    AlignedMatrix6x6f M;
    M.Random(mMin, mMax);
    return M;
  }

  static inline float MahalanobisDistance(const AlignedMatrix6x6f &W, const float d012,
                                          const float d345) {
    return ((W[0][0] + W[0][1] + W[0][2]) * d012 + (W[0][3] + W[0][4] + W[0][5]) * d345 +
            (W[1][0] + W[1][1] + W[1][2]) * d012 + (W[1][3] + W[1][4] + W[1][5]) * d345 +
            (W[2][0] + W[2][1] + W[2][2]) * d012 + (W[2][3] + W[2][4] + W[2][5]) * d345) * d012 +
           ((W[3][0] + W[3][1] + W[3][2]) * d012 + (W[3][3] + W[3][4] + W[3][5]) * d345 +
            (W[4][0] + W[4][1] + W[4][2]) * d012 + (W[4][3] + W[4][4] + W[4][5]) * d345 +
            (W[5][0] + W[5][1] + W[5][2]) * d012 + (W[5][3] + W[5][4] + W[5][5]) * d345) * d345;
  }

  static inline void ApB(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                         AlignedMatrix6x6f &ApB) {
    for (int i = 0; i < 9; ++i) {
      ApB.m_data4[i] = A.m_data4[i] + B.m_data4[i];
    }
  }
  static inline void AmB(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                         AlignedMatrix6x6f &AmB) {
    for (int i = 0; i < 9; ++i) {
      AmB.m_data4[i] = A.m_data4[i] - B.m_data4[i];
    }
  }

  static inline void abT(const float *a, const ProductVector6f &b, AlignedMatrix6x6f &abT) {
    xp128f t;
    t.vset_all_lane(a[0], a[0], a[1], a[1]);
    abT.m_00_01_02_03() = b.v0123() * a[0];
    abT.m_04_05_10_11() = b.v4501() * t;
    abT.m_12_13_14_15() = b.v2345() * a[1];

    t.vset_all_lane(a[2], a[2], a[3], a[3]);
    abT.m_20_21_22_23() = b.v0123() * a[2];
    abT.m_24_25_30_31() = b.v4501() * t;
    abT.m_32_33_34_35() = b.v2345() * a[3];

    t.vset_all_lane(a[4], a[4], a[5], a[5]);
    abT.m_40_41_42_43() = b.v0123() * a[4];
    abT.m_44_45_50_51() = b.v4501() * t;
    abT.m_52_53_54_55() = b.v2345() * a[5];
  }

  template<typename TYPE>
  static inline void Ab(const AlignedMatrix6x6f &A, const ProductVector6f &b, TYPE *Ab) {
    xp128f t;
    t = A.m_04_05_10_11() * b.v4501();
    Ab[0] = t[0] + t[1] + (A.m_00_01_02_03() * b.v0123()).vsum_all();
    Ab[1] = t[2] + t[3] + (A.m_12_13_14_15() * b.v2345()).vsum_all();
    t = A.m_24_25_30_31() * b.v4501();
    Ab[2] = t[0] + t[1] + (A.m_20_21_22_23() * b.v0123()).vsum_all();
    Ab[3] = t[2] + t[3] + (A.m_32_33_34_35() * b.v2345()).vsum_all();
    t = A.m_44_45_50_51() * b.v4501();
    Ab[4] = t[0] + t[1] + (A.m_40_41_42_43() * b.v0123()).vsum_all();
    Ab[5] = t[2] + t[3] + (A.m_52_53_54_55() * b.v2345()).vsum_all();
  }
  template<typename TYPE>
  static inline void AddAbTo(const AlignedMatrix6x6f &A, const ProductVector6f &b, TYPE *Ab) {
    xp128f t;
    t = A.m_04_05_10_11() * b.v4501();
    Ab[0] += t[0] + t[1] + (A.m_00_01_02_03() * b.v0123()).vsum_all();
    Ab[1] += t[2] + t[3] + (A.m_12_13_14_15() * b.v2345()).vsum_all();
    t = A.m_24_25_30_31() * b.v4501();
    Ab[2] += t[0] + t[1] + (A.m_20_21_22_23() * b.v0123()).vsum_all();
    Ab[3] += t[2] + t[3] + (A.m_32_33_34_35() * b.v2345()).vsum_all();
    t = A.m_44_45_50_51() * b.v4501();
    Ab[4] += t[0] + t[1] + (A.m_40_41_42_43() * b.v0123()).vsum_all();
    Ab[5] += t[2] + t[3] + (A.m_52_53_54_55() * b.v2345()).vsum_all();
  }
  template<typename TYPE>
  static inline void Ab(const AlignedMatrix6x6f &A, const AlignedVector6f &b, TYPE *Ab) {
    const xp128f b2345 = xp128f::get(&b.v2());
    const xp128f b4501 = xp128f::get(b.v4(), b.v5(), b.v0(), b.v1());
    xp128f t;
    t = A.m_04_05_10_11() * b4501;
    Ab[0] = t[0] + t[1] + (A.m_00_01_02_03() * b.v0123()).vsum_all();
    Ab[1] = t[2] + t[3] + (A.m_12_13_14_15() * b2345).vsum_all();
    t = A.m_24_25_30_31() * b4501;
    Ab[2] = t[0] + t[1] + (A.m_20_21_22_23() * b.v0123()).vsum_all();
    Ab[3] = t[2] + t[3] + (A.m_32_33_34_35() * b2345).vsum_all();
    t = A.m_44_45_50_51() * b4501;
    Ab[4] = t[0] + t[1] + (A.m_40_41_42_43() * b.v0123()).vsum_all();
    Ab[5] = t[2] + t[3] + (A.m_52_53_54_55() * b2345).vsum_all();
  }
  template<typename TYPE>
  static inline void AddAbTo(const AlignedMatrix6x6f &A, const AlignedVector6f &b, TYPE *Ab) {
    const xp128f b2345 = xp128f::get(&b.v2());
    const xp128f b4501 = xp128f::get(b.v4(), b.v5(), b.v0(), b.v1());
    xp128f t;
    t = A.m_04_05_10_11() * b4501;
    Ab[0] += t[0] + t[1] + (A.m_00_01_02_03() * b.v0123()).vsum_all();
    Ab[1] += t[2] + t[3] + (A.m_12_13_14_15() * b2345).vsum_all();
    t = A.m_24_25_30_31() * b4501;
    Ab[2] += t[0] + t[1] + (A.m_20_21_22_23() * b.v0123()).vsum_all();
    Ab[3] += t[2] + t[3] + (A.m_32_33_34_35() * b2345).vsum_all();
    t = A.m_44_45_50_51() * b4501;
    Ab[4] += t[0] + t[1] + (A.m_40_41_42_43() * b.v0123()).vsum_all();
    Ab[5] += t[2] + t[3] + (A.m_52_53_54_55() * b2345).vsum_all();
  }

  static inline void ATB(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                         AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *B0 = B[0], *B1 = B[1];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = 0; j < 6; ++j) {
        ATBi[j] = A0[i] * B0[j] + A1[i] * B1[j];
      }
    }
  }
  static inline void AddATBTo(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                              AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *B0 = B[0], *B1 = B[1];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = 0; j < 6; ++j) {
        ATBi[j] += A0[i] * B0[j] + A1[i] * B1[j];
      }
    }
  }
  static inline void AddATBToUpper(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                                   AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *B0 = B[0], *B1 = B[1];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = i; j < 6; ++j) {
        ATBi[j] += A0[i] * B0[j] + A[1][i] * B1[j];
      }
    }
  }
  static inline void ATB(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                         AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *A2 = A[2], *A3 = A[3], *A4 = A[4], *A5 = A[5];
    const float *B0 = B[0], *B1 = B[1], *B2 = B[2], *B3 = B[3], *B4 = B[4], *B5 = B[5];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = 0; j < 6; ++j) {
        ATBi[j] = A0[i] * B0[j] + A1[i] * B1[j] + A2[i] * B2[j] +
                  A3[i] * B3[j] + A4[i] * B4[j] + A5[i] * B5[j];
      }
    }
  }
  static inline void AddATBTo(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                              AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *A2 = A[2], *A3 = A[3], *A4 = A[4], *A5 = A[5];
    const float *B0 = B[0], *B1 = B[1], *B2 = B[2], *B3 = B[3], *B4 = B[4], *B5 = B[5];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = 0; j < 6; ++j) {
        ATBi[j] += A0[i] * B0[j] + A1[i] * B1[j] + A2[i] * B2[j] +
                   A3[i] * B3[j] + A4[i] * B4[j] + A5[i] * B5[j];
      }
    }
  }
  static inline void AddATBToUpper(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                                   AlignedMatrix6x6f &ATB) {
    const float *A0 = A[0], *A1 = A[1], *A2 = A[2], *A3 = A[3], *A4 = A[4], *A5 = A[5];
    const float *B0 = B[0], *B1 = B[1], *B2 = B[2], *B3 = B[3], *B4 = B[4], *B5 = B[5];
    for (int i = 0; i < 6; ++i) {
      float *ATBi = ATB[i];
      for (int j = i; j < 6; ++j) {
        ATBi[j] += A0[i] * B0[j] + A1[i] * B1[j] + A2[i] * B2[j] +
                   A3[i] * B3[j] + A4[i] * B4[j] + A5[i] * B5[j];
      }
    }
  }
  static inline void AddATbTo(const AlignedMatrix6x6f &A, const float *b, float *ATb) {
    const float *A0 = A[0], *A1 = A[1], *A2 = A[2], *A3 = A[3], *A4 = A[4], *A5 = A[5];
    for (int i = 0; i < 6; ++i) {
      ATb[i] += A0[i] * b[0] + A1[i] * b[1] + A2[i] * b[2] +
                A3[i] * b[3] + A4[i] * b[4] + A5[i] * b[5];
    }
  }

  static inline void ABT(const AlignedMatrix2x6f &A, const AlignedMatrix6x6f &B,
                         AlignedMatrix2x6f &ABT) {
    xp128f t;
    const xp128f a1 = xp128f::get(&A[0][2]), a2 = xp128f::get(A[1]);
    t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT[0][0] = (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[0][1] = A[0][0] * B[1][0] + A[0][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[1][0] = (a2 * B.m_00_01_02_03()).vsum_all() + A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT[1][1] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
    t = A.m_04_05_10_11() * B.m_24_25_30_31();
    ABT[0][2] = (A.m_00_01_02_03() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[0][3] = A[0][0] * B[3][0] + A[0][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[1][2] = (a2 * B.m_20_21_22_23()).vsum_all() + A[1][4] * B[2][4] + A[1][5] * B[2][5];
    ABT[1][3] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_32_33_34_35()).vsum_all();
    t = A.m_04_05_10_11() * B.m_44_45_50_51();
    ABT[0][4] = (A.m_00_01_02_03() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[0][5] = A[0][0] * B[5][0] + A[0][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[1][4] = (a2 * B.m_40_41_42_43()).vsum_all() + A[1][4] * B[4][4] + A[1][5] * B[4][5];
    ABT[1][5] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_52_53_54_55()).vsum_all();
  }
  static inline void AddABTTo(const AlignedMatrix2x6f &A, const AlignedMatrix6x6f &B,
                              AlignedMatrix2x6f &ABT) {
    xp128f t;
    const xp128f a1 = xp128f::get(&A[0][2]), a2 = xp128f::get(A[1]);
    t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT[0][0] += (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[0][1] += A[0][0] * B[1][0] + A[0][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[1][0] += (a2 * B.m_00_01_02_03()).vsum_all() + A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT[1][1] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
    t = A.m_04_05_10_11() * B.m_24_25_30_31();
    ABT[0][2] += (A.m_00_01_02_03() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[0][3] += A[0][0] * B[3][0] + A[0][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[1][2] += (a2 * B.m_20_21_22_23()).vsum_all() + A[1][4] * B[2][4] + A[1][5] * B[2][5];
    ABT[1][3] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_32_33_34_35()).vsum_all();
    t = A.m_04_05_10_11() * B.m_44_45_50_51();
    ABT[0][4] += (A.m_00_01_02_03() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[0][5] += A[0][0] * B[5][0] + A[0][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[1][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[1][4] * B[4][4] + A[1][5] * B[4][5];
    ABT[1][5] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_52_53_54_55()).vsum_all();
  }
  static inline void ABT(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                         AlignedMatrix6x6f &ABT) {
    xp128f a1, a2, t;
    a1.vload_unalign(&A[0][2]);
    a2.vload_unalign(&A[1][0]);
    t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT[0][0] = (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[0][1] = A[0][0] * B[1][0] + A[0][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[1][0] = (a2 * B.m_00_01_02_03()).vsum_all() + A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT[1][1] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
    t = A.m_04_05_10_11() * B.m_24_25_30_31();
    ABT[0][2] = (A.m_00_01_02_03() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[0][3] = A[0][0] * B[3][0] + A[0][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[1][2] = (a2 * B.m_20_21_22_23()).vsum_all() + A[1][4] * B[2][4] + A[1][5] * B[2][5];
    ABT[1][3] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_32_33_34_35()).vsum_all();
    t = A.m_04_05_10_11() * B.m_44_45_50_51();
    ABT[0][4] = (A.m_00_01_02_03() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[0][5] = A[0][0] * B[5][0] + A[0][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[1][4] = (a2 * B.m_40_41_42_43()).vsum_all() + A[1][4] * B[4][4] + A[1][5] * B[4][5];
    ABT[1][5] = t[2] + t[3] + (A.m_12_13_14_15() * B.m_52_53_54_55()).vsum_all();
    a1.vload_unalign(&A[2][2]);
    a2.vload_unalign(&A[3][0]);
    t = A.m_24_25_30_31() * B.m_04_05_10_11();
    ABT[2][0] = (A.m_20_21_22_23() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[2][1] = A[2][0] * B[1][0] + A[2][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[3][0] = (a2 * B.m_00_01_02_03()).vsum_all() + A[3][4] * B[0][4] + A[3][5] * B[0][5];
    ABT[3][1] = t[2] + t[3] + (A.m_32_33_34_35() * B.m_12_13_14_15()).vsum_all();
    t = A.m_24_25_30_31() * B.m_24_25_30_31();
    ABT[2][2] = (A.m_20_21_22_23() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[2][3] = A[2][0] * B[3][0] + A[2][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[3][2] = (a2 * B.m_20_21_22_23()).vsum_all() + A[3][4] * B[2][4] + A[3][5] * B[2][5];
    ABT[3][3] = t[2] + t[3] + (A.m_32_33_34_35() * B.m_32_33_34_35()).vsum_all();
    t = A.m_24_25_30_31() * B.m_44_45_50_51();
    ABT[2][4] = (A.m_20_21_22_23() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[2][5] = A[2][0] * B[5][0] + A[2][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[3][4] = (a2 * B.m_40_41_42_43()).vsum_all() + A[3][4] * B[4][4] + A[3][5] * B[4][5];
    ABT[3][5] = t[2] + t[3] + (A.m_32_33_34_35() * B.m_52_53_54_55()).vsum_all();
    a1.vload_unalign(&A[4][2]);
    a2.vload_unalign(&A[5][0]);
    t = A.m_44_45_50_51() * B.m_04_05_10_11();
    ABT[4][0] = (A.m_40_41_42_43() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[4][1] = A[4][0] * B[1][0] + A[4][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[5][0] = (a2 * B.m_00_01_02_03()).vsum_all() + A[5][4] * B[0][4] + A[5][5] * B[0][5];
    ABT[5][1] = t[2] + t[3] + (A.m_52_53_54_55() * B.m_12_13_14_15()).vsum_all();
    t = A.m_44_45_50_51() * B.m_24_25_30_31();
    ABT[4][2] = (A.m_40_41_42_43() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[4][3] = A[4][0] * B[3][0] + A[4][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[5][2] = (a2 * B.m_20_21_22_23()).vsum_all() + A[5][4] * B[2][4] + A[5][5] * B[2][5];
    ABT[5][3] = t[2] + t[3] + (A.m_52_53_54_55() * B.m_32_33_34_35()).vsum_all();
    t = A.m_44_45_50_51() * B.m_44_45_50_51();
    ABT[4][4] = (A.m_40_41_42_43() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[4][5] = A[4][0] * B[5][0] + A[4][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[5][4] = (a2 * B.m_40_41_42_43()).vsum_all() + A[5][4] * B[4][4] + A[5][5] * B[4][5];
    ABT[5][5] = t[2] + t[3] + (A.m_52_53_54_55() * B.m_52_53_54_55()).vsum_all();
  }
  static inline void AddABTTo(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                              AlignedMatrix6x6f &ABT) {
    xp128f a1, a2, t;
    a1.vload_unalign(&A[0][2]);
    a2.vload_unalign(&A[1][0]);
    t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT[0][0] += (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[0][1] += A[0][0] * B[1][0] + A[0][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[1][0] += (a2 * B.m_00_01_02_03()).vsum_all() + A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT[1][1] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
    t = A.m_04_05_10_11() * B.m_24_25_30_31();
    ABT[0][2] += (A.m_00_01_02_03() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[0][3] += A[0][0] * B[3][0] + A[0][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[1][2] += (a2 * B.m_20_21_22_23()).vsum_all() + A[1][4] * B[2][4] + A[1][5] * B[2][5];
    ABT[1][3] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_32_33_34_35()).vsum_all();
    t = A.m_04_05_10_11() * B.m_44_45_50_51();
    ABT[0][4] += (A.m_00_01_02_03() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[0][5] += A[0][0] * B[5][0] + A[0][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[1][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[1][4] * B[4][4] + A[1][5] * B[4][5];
    ABT[1][5] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_52_53_54_55()).vsum_all();
    a1.vload_unalign(&A[2][2]);
    a2.vload_unalign(&A[3][0]);
    t = A.m_24_25_30_31() * B.m_04_05_10_11();
    ABT[2][0] += (A.m_20_21_22_23() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[2][1] += A[2][0] * B[1][0] + A[2][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[3][0] += (a2 * B.m_00_01_02_03()).vsum_all() + A[3][4] * B[0][4] + A[3][5] * B[0][5];
    ABT[3][1] += t[2] + t[3] + (A.m_32_33_34_35() * B.m_12_13_14_15()).vsum_all();
    t = A.m_24_25_30_31() * B.m_24_25_30_31();
    ABT[2][2] += (A.m_20_21_22_23() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[2][3] += A[2][0] * B[3][0] + A[2][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[3][2] += (a2 * B.m_20_21_22_23()).vsum_all() + A[3][4] * B[2][4] + A[3][5] * B[2][5];
    ABT[3][3] += t[2] + t[3] + (A.m_32_33_34_35() * B.m_32_33_34_35()).vsum_all();
    t = A.m_24_25_30_31() * B.m_44_45_50_51();
    ABT[2][4] += (A.m_20_21_22_23() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[2][5] += A[2][0] * B[5][0] + A[2][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[3][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[3][4] * B[4][4] + A[3][5] * B[4][5];
    ABT[3][5] += t[2] + t[3] + (A.m_32_33_34_35() * B.m_52_53_54_55()).vsum_all();
    a1.vload_unalign(&A[4][2]);
    a2.vload_unalign(&A[5][0]);
    t = A.m_44_45_50_51() * B.m_04_05_10_11();
    ABT[4][0] += (A.m_40_41_42_43() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[4][1] += A[4][0] * B[1][0] + A[4][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[5][0] += (a2 * B.m_00_01_02_03()).vsum_all() + A[5][4] * B[0][4] + A[5][5] * B[0][5];
    ABT[5][1] += t[2] + t[3] + (A.m_52_53_54_55() * B.m_12_13_14_15()).vsum_all();
    t = A.m_44_45_50_51() * B.m_24_25_30_31();
    ABT[4][2] += (A.m_40_41_42_43() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[4][3] += A[4][0] * B[3][0] + A[4][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[5][2] += (a2 * B.m_20_21_22_23()).vsum_all() + A[5][4] * B[2][4] + A[5][5] * B[2][5];
    ABT[5][3] += t[2] + t[3] + (A.m_52_53_54_55() * B.m_32_33_34_35()).vsum_all();
    t = A.m_44_45_50_51() * B.m_44_45_50_51();
    ABT[4][4] += (A.m_40_41_42_43() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[4][5] += A[4][0] * B[5][0] + A[4][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[5][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[5][4] * B[4][4] + A[5][5] * B[4][5];
    ABT[5][5] += t[2] + t[3] + (A.m_52_53_54_55() * B.m_52_53_54_55()).vsum_all();
  }
  static inline void AddABTToUpper(const AlignedMatrix6x6f &A, const AlignedMatrix6x6f &B,
                                   AlignedMatrix6x6f &ABT) {
    xp128f a1, a2, t;
    a1.vload_unalign(&A[0][2]);
    a2.vload_unalign(&A[1][0]);
    t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT[0][0] += (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT[0][1] += A[0][0] * B[1][0] + A[0][1] * B[1][1] + (a1 * B.m_12_13_14_15()).vsum_all();
    ABT[1][1] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
    t = A.m_04_05_10_11() * B.m_24_25_30_31();
    ABT[0][2] += (A.m_00_01_02_03() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[0][3] += A[0][0] * B[3][0] + A[0][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[1][2] += (a2 * B.m_20_21_22_23()).vsum_all() + A[1][4] * B[2][4] + A[1][5] * B[2][5];
    ABT[1][3] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_32_33_34_35()).vsum_all();
    t = A.m_04_05_10_11() * B.m_44_45_50_51();
    ABT[0][4] += (A.m_00_01_02_03() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[0][5] += A[0][0] * B[5][0] + A[0][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[1][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[1][4] * B[4][4] + A[1][5] * B[4][5];
    ABT[1][5] += t[2] + t[3] + (A.m_12_13_14_15() * B.m_52_53_54_55()).vsum_all();

    a1.vload_unalign(&A[2][2]);
    a2.vload_unalign(&A[3][0]);
    t = A.m_24_25_30_31() * B.m_24_25_30_31();
    ABT[2][2] += (A.m_20_21_22_23() * B.m_20_21_22_23()).vsum_all() + t[0] + t[1];
    ABT[2][3] += A[2][0] * B[3][0] + A[2][1] * B[3][1] + (a1 * B.m_32_33_34_35()).vsum_all();
    ABT[3][3] += t[2] + t[3] + (A.m_32_33_34_35() * B.m_32_33_34_35()).vsum_all();
    t = A.m_24_25_30_31() * B.m_44_45_50_51();
    ABT[2][4] += (A.m_20_21_22_23() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[2][5] += A[2][0] * B[5][0] + A[2][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[3][4] += (a2 * B.m_40_41_42_43()).vsum_all() + A[3][4] * B[4][4] + A[3][5] * B[4][5];
    ABT[3][5] += t[2] + t[3] + (A.m_32_33_34_35() * B.m_52_53_54_55()).vsum_all();

    a1.vload_unalign(&A[4][2]);
    a2.vload_unalign(&A[5][0]);
    t = A.m_44_45_50_51() * B.m_44_45_50_51();
    ABT[4][4] += (A.m_40_41_42_43() * B.m_40_41_42_43()).vsum_all() + t[0] + t[1];
    ABT[4][5] += A[4][0] * B[5][0] + A[4][1] * B[5][1] + (a1 * B.m_52_53_54_55()).vsum_all();
    ABT[5][5] += t[2] + t[3] + (A.m_52_53_54_55() * B.m_52_53_54_55()).vsum_all();
  }

  //static inline void AddAbTo(const AlignedMatrix6x6f &A, const float* b, float *Ab) {
  //  float t[4];
  //  t[0] = A(0, 4) * b[4];
  //  t[1] = A(0, 5) * b[5];
  //  t[2] = A(1, 0) * b[0];
  //  t[3] = A(1, 1) * b[1];
  //  Ab[0] += t[0] + t[1] + (A(0, 0) * b[0] + A(0, 1) * b[1] + A(0, 2) * b[2] + A(0, 3) * b[3]);
  //  Ab[1] += t[2] + t[3] + (A(1, 2) * b[2] + A(1, 3) * b[3] + A(1, 4) * b[4] + A(1, 5) * b[5]);


  //  t[0] = A(2, 4) * b[4];
  //  t[1] = A(2, 5) * b[5];
  //  t[2] = A(3, 0) * b[0];
  //  t[3] = A(3, 1) * b[1];
  //  Ab[2] += t[0] + t[1] + (A(2, 0) * b[0] + A(2, 1) * b[1] + A(2, 2) * b[2] + A(2, 3) * b[3]);
  //  Ab[3] += t[2] + t[3] + (A(3, 2) * b[2] + A(3, 3) * b[3] + A(3, 4) * b[4] + A(3, 5) * b[5]);

  //  t[0] = A(4, 4) * b[4];
  //  t[1] = A(4, 5) * b[5];
  //  t[2] = A(5, 0) * b[0];
  //  t[3] = A(5, 1) * b[1];
  //  Ab[4] += t[0] + t[1] + (A(4, 0) * b[0] + A(4, 1) * b[1] + A(4, 2) * b[2] + A(4, 3) * b[3]);
  //  Ab[5] += t[2] + t[3] + (A(5, 2) * b[2] + A(5, 3) * b[3] + A(5, 4) * b[4] + A(5, 5) * b[5]);
  //}

  static inline void AddAbTo_loop(const AlignedMatrix6x6f &A, const float* b, float *Ab) {
    for (int i = 0; i < 6; ++i) {
      float sum = 0.f;
      for (int j = 0; j < 6; ++j) {
        sum += A(i, j) * b[j];
      }
      Ab[i] += sum;
    }
  }

  static inline void ABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[0][0] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][1] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][2] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][0] = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[1][1] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][2] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][0] = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[2][1] = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[2][2] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[0][0] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][1] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][2] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][0] += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[1][1] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][2] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][0] += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[2][1] += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[2][2] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo03(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[0][3] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][4] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][5] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][3] = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[1][4] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][5] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][3] = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[2][4] = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[2][5] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo03(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[0][3] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][4] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][5] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][3] += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[1][4] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][5] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][3] += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[2][4] += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[2][5] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo30(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[3][0] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][1] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][2] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][0] = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[4][1] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][2] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][0] = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[5][1] = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[5][2] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo30(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[3][0] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][1] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][2] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][0] += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[4][1] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][2] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][0] += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[5][1] += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[5][2] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[3][3] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][4] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][5] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][3] = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[4][4] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][5] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][3] = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[5][4] = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[5][5] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[3][3] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][4] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][5] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][3] += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT[4][4] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][5] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][3] += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT[5][4] += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT[5][5] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ATBTo30(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                             AlignedMatrix6x6f &ABT) {
    xp128f t;
    t = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * A.m10();  memcpy(ABT[3], &t, 12);
    t = B.m_00_01_02_r0() * A.m01() + B.m_10_11_12_r1() * A.m11();  memcpy(ABT[4], &t, 12);
    t = B.m_00_01_02_r0() * A.m02() + B.m_10_11_12_r1() * A.m12();  memcpy(ABT[5], &t, 12);
  }
  static inline void ATBTo33(const AlignedMatrix2x3f &A, const AlignedMatrix2x3f &B,
                             AlignedMatrix6x6f &ABT) {
    xp128f t;
    t = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * A.m10();  memcpy(&ABT[3][3], &t, 12);
    t = B.m_00_01_02_r0() * A.m01() + B.m_10_11_12_r1() * A.m11();  memcpy(&ABT[4][3], &t, 12);
    t = B.m_00_01_02_r0() * A.m02() + B.m_10_11_12_r1() * A.m12();  memcpy(&ABT[5][3], &t, 12);
  }

 public:
  union {
    float m_data[6][6];
    xp128f m_data4[9];
  };
};

template<typename TYPE> class SymmetricMatrix6x6 {
 public:
  inline const TYPE& m00() const { return m_data[0]; }    inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }    inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m02() const { return m_data[2]; }    inline TYPE& m02() { return m_data[2]; }
  inline const TYPE& m03() const { return m_data[3]; }    inline TYPE& m03() { return m_data[3]; }
  inline const TYPE& m04() const { return m_data[4]; }    inline TYPE& m04() { return m_data[4]; }
  inline const TYPE& m05() const { return m_data[5]; }    inline TYPE& m05() { return m_data[5]; }
  inline const TYPE& m11() const { return m_data[6]; }    inline TYPE& m11() { return m_data[6]; }
  inline const TYPE& m12() const { return m_data[7]; }    inline TYPE& m12() { return m_data[7]; }
  inline const TYPE& m13() const { return m_data[8]; }    inline TYPE& m13() { return m_data[8]; }
  inline const TYPE& m14() const { return m_data[9]; }    inline TYPE& m14() { return m_data[9]; }
  inline const TYPE& m15() const { return m_data[10]; }   inline TYPE& m15() { return m_data[10]; }
  inline const TYPE& m22() const { return m_data[11]; }   inline TYPE& m22() { return m_data[11]; }
  inline const TYPE& m23() const { return m_data[12]; }   inline TYPE& m23() { return m_data[12]; }
  inline const TYPE& m24() const { return m_data[13]; }   inline TYPE& m24() { return m_data[13]; }
  inline const TYPE& m25() const { return m_data[14]; }   inline TYPE& m25() { return m_data[14]; }
  inline const TYPE& m33() const { return m_data[15]; }   inline TYPE& m33() { return m_data[15]; }
  inline const TYPE& m34() const { return m_data[16]; }   inline TYPE& m34() { return m_data[16]; }
  inline const TYPE& m35() const { return m_data[17]; }   inline TYPE& m35() { return m_data[17]; }
  inline const TYPE& m44() const { return m_data[18]; }   inline TYPE& m44() { return m_data[18]; }
  inline const TYPE& m45() const { return m_data[19]; }   inline TYPE& m45() { return m_data[19]; }
  inline const TYPE& m55() const { return m_data[20]; }   inline TYPE& m55() { return m_data[20]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  inline void operator *= (const xp128f &s);
  inline SymmetricMatrix6x6<TYPE> operator * (const xp128f &s) const;
  inline SymmetricMatrix6x6<TYPE> operator - (const SymmetricMatrix6x6<TYPE> &B) const {
    SymmetricMatrix6x6<TYPE> AmB;
    for (int i = 0; i < 21; ++i) {
      AmB.m_data[i] = m_data[i] - B.m_data[i];
    }
    return AmB;
  }
  inline bool operator == (const SymmetricMatrix6x6<TYPE> &A) const {
    bool equal = true;
    for (int i = 0; i < 21 && equal; ++i) {
      equal = m_data[i] == A.m_data[i];
    }
    return equal;
  }

  //inline void Set(const TYPE *M) { memcpy(this, M, sizeof(SymmetricMatrix6x6<TYPE>)); }
  inline void Set(const AlignedMatrix6x6f &M);
  inline void Set(const SymmetricMatrix3x3<TYPE> &M00, const AlignedMatrix3x3f &M01,
                  const SymmetricMatrix3x3<TYPE> &M11) {
    Set00(M00);
    Set03(M01);
    Set33(M11);
  }
  inline void Set(const AlignedMatrix3x3f &M00, const AlignedMatrix3x3f &M01,
                  const AlignedMatrix3x3f &M11) {
    Set00(M00);
    Set03(M01);
    Set33(M11);
  }
  inline void Set00(const SymmetricMatrix3x3<TYPE> &M);
  inline void Set00(const AlignedMatrix3x3f &M);
  inline void Set03(const AlignedMatrix3x3f &M);
  inline void Set33(const SymmetricMatrix3x3<TYPE> &M);
  inline void Set33(const AlignedMatrix3x3f &M);
  inline void Get00(SymmetricMatrix3x3f *M) const;
  inline void Get00(SymmetricMatrix3x3d *M) const;
  inline void Get00(AlignedMatrix3x3f *M) const;
  inline void Get33(SymmetricMatrix3x3f *M) const;
  inline void Get33(SymmetricMatrix3x3d *M) const;
  inline void Get33(AlignedMatrix3x3f *M) const;

  inline void Increase00(const SymmetricMatrix3x3<TYPE> &M) {
    m00() += M.m00(); m01() += M.m01(); m02() += M.m02();
                      m11() += M.m11(); m12() += M.m12();
                                        m22() += M.m22();
  }
  inline void Increase03(const AlignedMatrix3x3f &M) {
    m03() += M.m00(); m04() += M.m01(); m05() += M.m02();
    m13() += M.m10(); m14() += M.m11(); m15() += M.m12();
    m23() += M.m20(); m24() += M.m21(); m25() += M.m22();
  }
  inline void Increase33(const SymmetricMatrix3x3<TYPE> &M) {
    m33() += M.m00(); m34() += M.m01(); m35() += M.m02();
                      m44() += M.m11(); m45() += M.m12();
                                        m55() += M.m22();
  }

  inline AlignedMatrix6x6f GetAlignedMatrix6x6f() const {
    AlignedMatrix6x6f M;
    GetAlignedMatrix6x6f(M);
    return M;
  }
  inline void GetAlignedMatrix6x6f(AlignedMatrix6x6f &M) const;

  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix6x6<TYPE>)); }
  inline void MakeDiagonal(const TYPE d) { MakeZero(); SetDiagonal(d); }
  inline void SetDiagonal(const TYPE d) {
    m00() = d; m11() = d; m22() = d;
    m33() = d; m44() = d; m55() = d;
  }
  inline void IncreaseDiagonal(const TYPE d012, const TYPE d345) {
    m00() += d012;  m11() += d012;  m22() += d012;
    m33() += d345;  m44() += d345;  m55() += d345;
  }

  inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }

  inline void Print(const bool e = false, const bool f = false) const {
    if (f) {
      if (e) {
        UT::Print("%e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(), m05());
        UT::Print("%e %e %e %e %e %e\n", m01(), m11(), m12(), m13(), m14(), m15());
        UT::Print("%e %e %e %e %e %e\n", m02(), m12(), m22(), m23(), m24(), m25());
        UT::Print("%e %e %e %e %e %e\n", m03(), m13(), m23(), m33(), m34(), m35());
        UT::Print("%e %e %e %e %e %e\n", m04(), m14(), m24(), m34(), m44(), m45());
        UT::Print("%e %e %e %e %e %e\n", m05(), m15(), m25(), m35(), m45(), m55());
      } else {
        UT::Print("%f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(), m05());
        UT::Print("%f %f %f %f %f %f\n", m01(), m11(), m12(), m13(), m14(), m15());
        UT::Print("%f %f %f %f %f %f\n", m02(), m12(), m22(), m23(), m24(), m25());
        UT::Print("%f %f %f %f %f %f\n", m03(), m13(), m23(), m33(), m34(), m35());
        UT::Print("%f %f %f %f %f %f\n", m04(), m14(), m24(), m34(), m44(), m45());
        UT::Print("%f %f %f %f %f %f\n", m05(), m15(), m25(), m35(), m45(), m55());
      }
    } else {
      for (int i = 0; i < 21; ++i) {
        if (e)
          UT::Print("%e ", m_data[i]);
        else
          UT::Print("%f ", m_data[i]);
      }
      UT::Print("\n");
    }
  }

  inline bool AssertEqual(const SymmetricMatrix6x6<TYPE> &M,
                          const int verbose = 1, const std::string str = "",
                          const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 21, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const SymmetricMatrix6x6 E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertZero<TYPE>(&m00(), 21, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  static inline void ABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[0][0] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][1] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][2] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][1] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][2] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][2] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[0][0] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[0][1] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[0][2] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[1][1] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[1][2] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[2][2] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo03(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m03() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m04() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m05() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m13() = (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m14() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m15() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m23() = (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m24() = (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m25() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo03(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m03() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m04() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m05() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m13() += (A.m_10_11_12_r1() * B.m_00_01_02_r0()).vsum_012();
    ABT.m14() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m15() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m23() += (A.m_20_21_22_r2() * B.m_00_01_02_r0()).vsum_012();
    ABT.m24() += (A.m_20_21_22_r2() * B.m_10_11_12_r1()).vsum_012();
    ABT.m25() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m33() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m34() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m35() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m44() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m45() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m55() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                SymmetricMatrix6x6<TYPE> &ABT) {
    ABT.m33() += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m34() += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m35() += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m44() += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m45() += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m55() += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             AlignedMatrix6x6f &ABT) {
    ABT[3][3] = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][4] = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][5] = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][4] = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][5] = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][5] = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void AddABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                                AlignedMatrix6x6f &ABT) {
    ABT[3][3] += (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT[3][4] += (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT[3][5] += (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT[4][4] += (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT[4][5] += (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT[5][5] += (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }

 protected:
  TYPE m_data[21];
};

typedef SymmetricMatrix6x6<float> SymmetricMatrix6x6f;
typedef SymmetricMatrix6x6<double> SymmetricMatrix6x6d;

template<> inline void SymmetricMatrix6x6f::Set(const AlignedMatrix6x6f &M) {
  memcpy(&m00(), &M[0][0], 24);
  memcpy(&m11(), &M[1][1], 20);
  memcpy(&m22(), &M[2][2], 16);
  memcpy(&m33(), &M[3][3], 12);
  memcpy(&m44(), &M[4][4], 8);
  m55() = M[5][5];
}

template<> inline void SymmetricMatrix6x6f::operator *= (const xp128f &s) {
  xp128f *m = reinterpret_cast<xp128f*>(m_data);
  for (int i = 0; i < 5; ++i, ++m)
    *m *= s;
  m_data[20] *= s[0];
}
template<> inline SymmetricMatrix6x6f SymmetricMatrix6x6f::operator * (const xp128f &s) const {
  SymmetricMatrix6x6f _M;
  const xp128f *m = reinterpret_cast<const xp128f *>(m_data);
  xp128f *_m = reinterpret_cast<xp128f *>(_M.m_data);
  for (int i = 0; i < 5; ++i, ++m, ++_m)
    *_m = *m * s;
  _M.m_data[20] = m_data[20] * s[0];
  return _M;
}

template<> inline void SymmetricMatrix6x6f::Set00(const SymmetricMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m11(), &M.m11(), 8);
  m22() = M.m22();
}
template<> inline void SymmetricMatrix6x6f::Set00(const AlignedMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m11(), &M.m11(), 8);
  m22() = M.m22();
}
template<> inline void SymmetricMatrix6x6f::Set03(const AlignedMatrix3x3f &M) {
  memcpy(&m03(), &M.m00(), 12);
  memcpy(&m13(), &M.m10(), 12);
  memcpy(&m23(), &M.m20(), 12);
}
template<> inline void SymmetricMatrix6x6f::Set33(const SymmetricMatrix3x3f &M) {
  memcpy(&m33(), &M.m00(), 12);
  memcpy(&m44(), &M.m11(), 8);
  m55() = M.m22();
}
template<> inline void SymmetricMatrix6x6f::Set33(const AlignedMatrix3x3f &M) {
  memcpy(&m33(), &M.m00(), 12);
  memcpy(&m44(), &M.m11(), 8);
  m55() = M.m22();
}
template<> inline void SymmetricMatrix6x6f::Get00(SymmetricMatrix3x3f *M) const {
  memcpy(&M->m00(), &m00(), 12);
  memcpy(&M->m11(), &m11(), 8);
  M->m22() = m22();
}
template<> inline void SymmetricMatrix6x6f::Get00(SymmetricMatrix3x3d *M) const {
  M->m00() = double(m00());
  M->m01() = double(m01());
  M->m02() = double(m02());
  M->m11() = double(m11());
  M->m12() = double(m12());
  M->m22() = double(m22());
}
template<> inline void SymmetricMatrix6x6f::Get00(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m00(), 12);
  memcpy(&M->m11(), &m11(), 8);
  M->m22() = m22();
  M->SetLowerFromUpper();
}
template<> inline void SymmetricMatrix6x6f::Get33(SymmetricMatrix3x3f *M) const {
  memcpy(&M->m00(), &m33(), 12);
  memcpy(&M->m11(), &m44(), 8);
  M->m22() = m55();
}
template<> inline void SymmetricMatrix6x6f::Get33(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m33(), 12);
  memcpy(&M->m11(), &m44(), 8);
  M->m22() = m55();
  M->SetLowerFromUpper();
}
template<> inline void SymmetricMatrix6x6f::Get33(SymmetricMatrix3x3d *M) const {
  M->m00() = double(m33());
  M->m01() = double(m34());
  M->m02() = double(m35());
  M->m11() = double(m44());
  M->m12() = double(m45());
  M->m22() = double(m55());
}

template<> inline void SymmetricMatrix6x6f::GetAlignedMatrix6x6f(AlignedMatrix6x6f &M) const {
  memcpy(M[0], &m00(), 24);
  memcpy(M[1] + 1, &m11(), 20);
  memcpy(M[2] + 2, &m22(), 16);
  memcpy(M[3] + 3, &m33(), 12);
  memcpy(M[4] + 4, &m44(), 8);
  M[5][5] = m55();
  M.SetLowerFromUpper();
}
}  // namespace LA

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix6x6f : public Eigen::Matrix<float, 6, 6, Eigen::RowMajor> {
 public:
  inline EigenMatrix6x6f() : Eigen::Matrix<float, 6, 6, Eigen::RowMajor>() {}
  inline EigenMatrix6x6f(const Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M) :
                         Eigen::Matrix<float, 6, 6, Eigen::RowMajor>(e_M) {}
  inline EigenMatrix6x6f(const LA::AlignedMatrix6x6f &M) :
                         Eigen::Matrix<float, 6, 6, Eigen::RowMajor>() {
    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        e_M(i, j) = M[i][j];
      }
    }
  }
  inline EigenMatrix6x6f(const LA::SymmetricMatrix6x6f &M) :
                         Eigen::Matrix<float, 6, 6, Eigen::RowMajor>() {
    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M = *this;
    const float *_M = M;
    for (int i = 0, k = 0; i < 6; ++i) {
      for (int j = i; j < 6; ++j, ++k) {
        e_M(i, j) = e_M(j, i) = _M[k];
      }
    }
  }
  inline EigenMatrix6x6f(const EigenMatrix3x3f &e_M00, const EigenMatrix3x3f &e_M01,
                         const EigenMatrix3x3f &e_M10, const EigenMatrix3x3f &e_M11) {
    block<3, 3>(0, 0) = e_M00;      block<3, 3>(0, 3) = e_M01;
    block<3, 3>(3, 0) = e_M10;      block<3, 3>(3, 3) = e_M11;
  }
  inline EigenMatrix6x6f(const LA::SymmetricMatrix3x3f &M00, const LA::AlignedMatrix3x3f &M01,
                         const LA::SymmetricMatrix3x3f &M11) {
    block<3, 3>(0, 0) = EigenMatrix3x3f(M00);           block<3, 3>(0, 3) = EigenMatrix3x3f(M01);
    block<3, 3>(3, 0) = block<3, 3>(0, 3).transpose();  block<3, 3>(3, 3) = EigenMatrix3x3f(M11);
  }
  inline EigenMatrix6x6f(const LA::AlignedMatrix3x3f &M00, const LA::AlignedMatrix3x3f &M01,
                         const LA::AlignedMatrix3x3f &M10, const LA::AlignedMatrix3x3f &M11) {
    block<3, 3>(0, 0) = EigenMatrix3x3f(M00);       block<3, 3>(0, 3) = EigenMatrix3x3f(M01);
    block<3, 3>(3, 0) = EigenMatrix3x3f(M10);       block<3, 3>(3, 3) = EigenMatrix3x3f(M11);
  }
  inline void operator = (const Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M) {
    *((Eigen::Matrix<float, 6, 6, Eigen::RowMajor> *) this) = e_M;
  }
  inline void Get(LA::AlignedMatrix6x6f &M) const {
    const Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        M[i][j] = e_M(i, j);
      }
    }
  }
  inline LA::SymmetricMatrix6x6f GetSymmetricMatrix6x6f() const {
    LA::SymmetricMatrix6x6f M;
    float *_M = &M.m00();
    const Eigen::Matrix<float, 6, 6, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 6; ++i) {
      for (int j = i; j < 6; ++j) {
        *_M++ = e_M(i, j);
      }
    }
    return M;
  }
  inline LA::AlignedMatrix6x6f GetAlignedMatrix6x6f() const {
    LA::AlignedMatrix6x6f M;
    Get(M);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix6x6f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix6x6f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix6x6f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::SymmetricMatrix6x6f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix6x6f().AssertEqual(M.GetAlignedMatrix6x6f(),
                                              verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix6x6f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix6x6f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix6x6f().AssertZero(verbose, str, epsAbs, epsRel);
  }
  static inline EigenMatrix6x6f Zero() {
    EigenMatrix6x6f e_M;
    e_M.setZero();
    return e_M;
  }
  static inline EigenMatrix6x6f Identity() {
    EigenMatrix6x6f e_I;
    e_I.setIdentity();
    return e_I;
  }
};
#endif
#endif  // LINEARALGEBRA_MATRIX6X6_H_
