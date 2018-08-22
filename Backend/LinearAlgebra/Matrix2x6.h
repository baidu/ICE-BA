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
#ifndef _MATRIX_2x6_H_
#define _MATRIX_2x6_H_

#include "Matrix2x3.h"
#include "Vector6.h"

namespace LA {
class AlignedMatrix2x6f {
 public:
  inline const xp128f& m_00_01_02_03() const { return m_data4[0]; } inline xp128f& m_00_01_02_03() { return m_data4[0]; }
  inline const xp128f& m_04_05_10_11() const { return m_data4[1]; } inline xp128f& m_04_05_10_11() { return m_data4[1]; }
  inline const xp128f& m_12_13_14_15() const { return m_data4[2]; } inline xp128f& m_12_13_14_15() { return m_data4[2]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float* operator[] (const int i) const { return m_data[i]; }
  inline       float* operator[] (const int i)       { return m_data[i]; }
  inline const float& operator() (const int row, const int col) const {
    return m_data[row][col];
  }
  inline float& operator() (const int row, const int col) {
    return m_data[row][col];
  }
  inline void operator += (const AlignedMatrix2x6f &M) {
    m_00_01_02_03() += M.m_00_01_02_03();
    m_04_05_10_11() += M.m_04_05_10_11();
    m_12_13_14_15() += M.m_12_13_14_15();
  }
  inline AlignedMatrix2x6f operator + (const AlignedMatrix2x6f &B) const {
    AlignedMatrix2x6f _ApB;
    ApB(*this, B, _ApB);
    return _ApB;
  }
  inline AlignedMatrix2x6f operator - (const AlignedMatrix2x6f &B) const {
    AlignedMatrix2x6f _AmB;
    AmB(*this, B, _AmB);
    return _AmB;
  }

  inline void Set(const AlignedMatrix2x3f &M0, const AlignedMatrix2x3f &M1) {
    memcpy(&m_data[0][0], &M0.m00(), 12);   memcpy(&m_data[0][3], &M1.m00(), 12);
    memcpy(&m_data[1][0], &M0.m10(), 12);   memcpy(&m_data[1][3], &M1.m10(), 12);
  }
  inline void Set(const Matrix2x3f &M0, const Matrix2x3f &M1) {
    memcpy(&m_data[0][0], M0[0], 12);   memcpy(&m_data[0][3], M1[0], 12);
    memcpy(&m_data[1][0], M0[1], 12);   memcpy(&m_data[1][3], M1[1], 12);
  }
  inline void Get(AlignedMatrix2x3f &M0, AlignedMatrix2x3f &M1) const {
    memcpy(&M0.m00(), &m_data[0][0], 12);   memcpy(&M1.m00(), &m_data[0][3], 12);
    memcpy(&M0.m10(), &m_data[1][0], 12);   memcpy(&M1.m10(), &m_data[1][3], 12);
  }
  inline void Get(Matrix2x3f &M0, Matrix2x3f &M1) const {
    memcpy(M0[0], &m_data[0][0], 12);       memcpy(M1[0], &m_data[0][3], 12);
    memcpy(M0[1], &m_data[1][0], 12);       memcpy(M1[1], &m_data[1][3], 12);
  }
  inline void GetScaled(const xp128f &w, AlignedMatrix2x6f &M) const {
    M.m_00_01_02_03() = w * m_00_01_02_03();
    M.m_04_05_10_11() = w * m_04_05_10_11();
    M.m_12_13_14_15() = w * m_12_13_14_15();
  }
  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x6f)); }
  inline void MakeZero2x3() { memset(m_data[0], 0, 12); memset(m_data[1], 0, 12); }

  inline bool Valid() const { return m_data[0][0] != FLT_MAX; }
  inline bool Invalid() const { return m_data[0][0] == FLT_MAX; }
  inline void Invalidate() { m_data[0][0] = FLT_MAX; }

  inline void Print(const bool e = false) const {
    for (int i = 0; i < 2; ++i) {
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
    for (int i = 0; i < 2; ++i) {
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
  inline bool AssertEqual(const AlignedMatrix2x6f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m_data[0][0], &M[0][0], 12, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix2x6f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m_data[0][0], 12, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  inline void Random(const float mMax) { Random(-mMax, mMax); }
  inline void Random(const float mMin, const float mMax) {
   UT::Random(&m_data[0][0], 12, mMin, mMax);
  }
  static inline AlignedMatrix2x6f GetRandom(const float mMax) {
    AlignedMatrix2x6f M;
    M.Random(mMax);
    return M;
  }
  static inline AlignedMatrix2x6f GetRandom(const float mMin, const float mMax) {
    AlignedMatrix2x6f M;
    M.Random(mMin, mMax);
    return M;
  }

  static inline void ApB(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                         AlignedMatrix2x6f &ApB) {
    ApB.m_00_01_02_03() = A.m_00_01_02_03() + B.m_00_01_02_03();
    ApB.m_04_05_10_11() = A.m_04_05_10_11() + B.m_04_05_10_11();
    ApB.m_12_13_14_15() = A.m_12_13_14_15() + B.m_12_13_14_15();
  }
  static inline void AmB(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                         AlignedMatrix2x6f &AmB) {
    AmB.m_00_01_02_03() = A.m_00_01_02_03() - B.m_00_01_02_03();
    AmB.m_04_05_10_11() = A.m_04_05_10_11() - B.m_04_05_10_11();
    AmB.m_12_13_14_15() = A.m_12_13_14_15() - B.m_12_13_14_15();
  }
  
  static inline void Ab(const AlignedMatrix2x6f &A, const AlignedVector6f &b, Vector2f &Ab) {
    Ab.v0() = (A.m_00_01_02_03() * b.v0123()).vsum_all() +
               A[0][4] * b.v4() + A[0][5] * b.v5();
    Ab.v1() = (A.m_12_13_14_15() * xp128f::get(&b.v2())).vsum_all() +
               A[1][0] * b.v0() + A[1][1] * b.v1();
  }
  static inline void AddAbTo(const AlignedMatrix2x6f &A, const AlignedVector6f &b, Vector2f &Ab) {
    Ab.v0() += (A.m_00_01_02_03() * b.v0123()).vsum_all() +
                A[0][4] * b.v4() + A[0][5] * b.v5();
    Ab.v1() += (A.m_12_13_14_15() * xp128f::get(&b.v2())).vsum_all() +
                A[1][0] * b.v0() + A[1][1] * b.v1();
  }
  static inline void Ab(const AlignedMatrix2x6f &A, const ProductVector6f &b, Vector2f &Ab) {
    const xp128f t = A.m_04_05_10_11() * b.v4501();
    Ab.v0() = (A.m_00_01_02_03() * b.v0123()).vsum_all() + t[0] + t[1];
    Ab.v1() = (A.m_12_13_14_15() * b.v2345()).vsum_all() + t[2] + t[3];
  }
  static inline void AddAbTo(const AlignedMatrix2x6f &A, const ProductVector6f &b, Vector2f &Ab) {
    const xp128f t = A.m_04_05_10_11() * b.v4501();
    Ab.v0() += (A.m_00_01_02_03() * b.v0123()).vsum_all() + t[0] + t[1];
    Ab.v1() += (A.m_12_13_14_15() * b.v2345()).vsum_all() + t[2] + t[3];
  }
  
  static inline void ABT(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                         AlignedMatrix2x2f &ABT) {
    const xp128f t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT.m00() = (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT.m01() = A[0][0] * B[1][0] + A[0][1] * B[1][1] +
                (xp128f::get(&A[0][2]) * B.m_12_13_14_15()).vsum_all();
    ABT.m10() = (xp128f::get(A[1]) * B.m_00_01_02_03()).vsum_all() +
                A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT.m11() = t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
  }
  static inline void AddABTTo(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                              AlignedMatrix2x2f &ABT) {
    const xp128f t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT.m00() += (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT.m01() += A[0][0] * B[1][0] + A[0][1] * B[1][1] +
                 (xp128f::get(&A[0][2]) * B.m_12_13_14_15()).vsum_all();
    ABT.m10() += (xp128f::get(A[1]) * B.m_00_01_02_03()).vsum_all() +
                 A[1][4] * B[0][4] + A[1][5] * B[0][5];
    ABT.m11() += t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
  }
  static inline void ABT(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                         SymmetricMatrix2x2f &ABT) {
    const xp128f t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT.m00() = (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT.m01() = A[0][0] * B[1][0] + A[0][1] * B[1][1] +
                (xp128f::get(&A[0][2]) * B.m_12_13_14_15()).vsum_all();
    ABT.m11() = t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
  }
  static inline void AddABTTo(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B,
                              SymmetricMatrix2x2f &ABT) {
    const xp128f t = A.m_04_05_10_11() * B.m_04_05_10_11();
    ABT.m00() += (A.m_00_01_02_03() * B.m_00_01_02_03()).vsum_all() + t[0] + t[1];
    ABT.m01() += A[0][0] * B[1][0] + A[0][1] * B[1][1] +
                 (xp128f::get(&A[0][2]) * B.m_12_13_14_15()).vsum_all();
    ABT.m11() += t[2] + t[3] + (A.m_12_13_14_15() * B.m_12_13_14_15()).vsum_all();
  }
  static inline void ATB(const SymmetricMatrix2x2f &A, const AlignedMatrix2x6f &B,
                         AlignedMatrix2x6f &ATB) {
    const float *B0 = B[0], *B1 = B[1];
    float *ATB0 = ATB[0], *ATB1 = ATB[1];
    for (int i = 0; i < 6; ++i) {
      ATB0[i] = A.m00() * B0[i] + A.m10() * B1[i];
      ATB1[i] = A.m01() * B0[i] + A.m11() * B1[i];
    }
  }
  static inline void ATb(const AlignedMatrix2x6f &A, const Vector2f &b, AlignedVector6f &ATb) {
    ATb.v0123() = A.m_00_01_02_03() * b.v0() + xp128f::get(A[1]) * b.v1();
    ATb.v4() = A[0][4] * b.v0() + A[1][4] * b.v1();
    ATb.v5() = A[0][5] * b.v0() + A[1][5] * b.v1();
  }
  static inline void AddATbTo(const AlignedMatrix2x6f &A, const Vector2f &b, AlignedVector6f &ATb) {
    ATb.v0123() += A.m_00_01_02_03() * b.v0() + xp128f::get(A[1]) * b.v1();
    ATb.v4() += A[0][4] * b.v0() + A[1][4] * b.v1();
    ATb.v5() += A[0][5] * b.v0() + A[1][5] * b.v1();
  }
  static inline void ABTo03(const SymmetricMatrix2x2f &A, const AlignedMatrix2x3f &B,
                            AlignedMatrix2x6f &AB) {
    const xp128f a01 = xp128f::get(A.m01());
    const xp128f ab0 = B.m_00_01_02_r0() * A.m00() + B.m_10_11_12_r1() * a01;
    const xp128f ab1 = B.m_00_01_02_r0() * a01 + B.m_10_11_12_r1() * A.m11();
    memcpy(&AB.m_data[0][3], &ab0, 12);
    memcpy(&AB.m_data[1][3], &ab1, 12);
  }

 public:
  union {
    float m_data[2][6];
    xp128f m_data4[3];
  };
};
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x6f : public Eigen::Matrix<float, 2, 6, Eigen::RowMajor> {
 public:
  inline EigenMatrix2x6f() : Eigen::Matrix<float, 2, 6, Eigen::RowMajor>() {}
  inline EigenMatrix2x6f(const Eigen::Matrix<float, 2, 6, Eigen::RowMajor> &e_M) :
                         Eigen::Matrix<float, 2, 6, Eigen::RowMajor>(e_M) {}
  inline EigenMatrix2x6f(const LA::AlignedMatrix2x6f &M) :
                         Eigen::Matrix<float, 2, 6, Eigen::RowMajor>() {
    Eigen::Matrix<float, 2, 6, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 6; ++j)
        e_M(i, j) = M[i][j];
  }
  inline EigenMatrix2x6f(const EigenMatrix2x3f &e_M0, const EigenMatrix2x3f &e_M1) {
    block<2, 3>(0, 0) = e_M0;
    block<2, 3>(0, 3) = e_M1;
  }
  inline EigenMatrix2x6f(const LA::AlignedMatrix2x3f &M0, const LA::AlignedMatrix2x3f &M1) {
    block<2, 3>(0, 0) = EigenMatrix2x3f(M0);
    block<2, 3>(0, 3) = EigenMatrix2x3f(M1);
  }
  inline void operator = (const Eigen::Matrix<float, 2, 6, Eigen::RowMajor> &e_M) {
    *((Eigen::Matrix<float, 2, 6, Eigen::RowMajor> *) this) = e_M;
  }
  inline LA::AlignedMatrix2x6f GetAlignedMatrix2x6f() const {
    LA::AlignedMatrix2x6f M;
    const Eigen::Matrix<float, 2, 6, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 6; ++j)
        M[i][j] = e_M(i, j);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix2x6f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix2x6f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x6f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix2x6f &e_M, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix2x6f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x6f().AssertZero(verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
