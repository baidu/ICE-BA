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
#ifndef _MATRIX_2x8_H_
#define _MATRIX_2x8_H_

#include "Vector8.h"
#include "Matrix2x7.h"

namespace LA {

class AlignedMatrix2x8f {

 public:

  inline const xp128f& m_00_01_02_03() const { return m_data[0]; }  inline xp128f& m_00_01_02_03() { return m_data[0]; }
  inline const xp128f& m_04_05_06_07() const { return m_data[1]; }  inline xp128f& m_04_05_06_07() { return m_data[1]; }
  inline const xp128f& m_10_11_12_13() const { return m_data[2]; }  inline xp128f& m_10_11_12_13() { return m_data[2]; }
  inline const xp128f& m_14_15_16_17() const { return m_data[3]; }  inline xp128f& m_14_15_16_17() { return m_data[3]; }

  inline const float& m00() const { return m_data[0][0]; } inline float& m00() { return m_data[0][0]; }
  inline const float& m01() const { return m_data[0][1]; } inline float& m01() { return m_data[0][1]; }
  inline const float& m02() const { return m_data[0][2]; } inline float& m02() { return m_data[0][2]; }
  inline const float& m03() const { return m_data[0][3]; } inline float& m03() { return m_data[0][3]; }
  inline const float& m04() const { return m_data[1][0]; } inline float& m04() { return m_data[1][0]; }
  inline const float& m05() const { return m_data[1][1]; } inline float& m05() { return m_data[1][1]; }
  inline const float& m06() const { return m_data[1][2]; } inline float& m06() { return m_data[1][2]; }
  inline const float& m07() const { return m_data[1][3]; } inline float& m07() { return m_data[1][3]; }
  inline const float& m10() const { return m_data[2][0]; } inline float& m10() { return m_data[2][0]; }
  inline const float& m11() const { return m_data[2][1]; } inline float& m11() { return m_data[2][1]; }
  inline const float& m12() const { return m_data[2][2]; } inline float& m12() { return m_data[2][2]; }
  inline const float& m13() const { return m_data[2][3]; } inline float& m13() { return m_data[2][3]; }
  inline const float& m14() const { return m_data[3][0]; } inline float& m14() { return m_data[3][0]; }
  inline const float& m15() const { return m_data[3][1]; } inline float& m15() { return m_data[3][1]; }
  inline const float& m16() const { return m_data[3][2]; } inline float& m16() { return m_data[3][2]; }
  inline const float& m17() const { return m_data[3][3]; } inline float& m17() { return m_data[3][3]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline void operator += (const AlignedMatrix2x8f &M) {
    m_00_01_02_03() += M.m_00_01_02_03();
    m_04_05_06_07() += M.m_04_05_06_07();
    m_10_11_12_13() += M.m_10_11_12_13();
    m_14_15_16_17() += M.m_14_15_16_17();
  }
  inline void operator -= (const AlignedMatrix2x8f &M) {
    m_00_01_02_03() -= M.m_00_01_02_03();
    m_04_05_06_07() -= M.m_04_05_06_07();
    m_10_11_12_13() -= M.m_10_11_12_13();
    m_14_15_16_17() -= M.m_14_15_16_17();
  }
  inline void operator *= (const xp128f &s) {
    m_00_01_02_03() *= s;
    m_04_05_06_07() *= s;
    m_10_11_12_13() *= s;
    m_14_15_16_17() *= s;
  }
  inline AlignedMatrix2x8f operator - (const AlignedMatrix2x8f &B) const {
    AlignedMatrix2x8f AmB;
    AmB.m_00_01_02_03() = m_00_01_02_03() - B.m_00_01_02_03();
    AmB.m_04_05_06_07() = m_04_05_06_07() - B.m_04_05_06_07();
    AmB.m_10_11_12_13() = m_10_11_12_13() - B.m_10_11_12_13();
    AmB.m_14_15_16_17() = m_14_15_16_17() - B.m_14_15_16_17();
    return AmB;
  }

  inline void Set(const AlignedMatrix2x7f &M0, const Vector2f &M1) {
    memcpy(this, &M0, sizeof(AlignedMatrix2x7f));
    m07() = M1.v0();
    m17() = M1.v1();
  }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x8f)); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(), m05(), m06(), m07());
      UT::Print("%e %e %e %e %e %e %e %e\n", m10(), m11(), m12(), m13(), m14(), m15(), m16(), m17());
    } else {
      UT::Print("%f %f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(), m05(), m06(), m07());
      UT::Print("%f %f %f %f %f %f %f %f\n", m10(), m11(), m12(), m13(), m14(), m15(), m16(), m17());
    }
  }
  inline bool AssertEqual(const AlignedMatrix2x8f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 16, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix2x8f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&m00(), 16, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

 protected:

  xp128f m_data[4];
};
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x8f : public Eigen::Matrix<float, 2, 8> {
 public:
  inline EigenMatrix2x8f() : Eigen::Matrix<float, 2, 8>() {}
  inline EigenMatrix2x8f(const Eigen::Matrix<float, 2, 8> &e_M) : Eigen::Matrix<float, 2, 8>(e_M) {}
  inline EigenMatrix2x8f(const LA::AlignedMatrix2x8f &M) : Eigen::Matrix<float, 2, 8>() {
    const float* _M[2] = {&M.m00(), &M.m10()};
    Eigen::Matrix<float, 2, 8> &e_M = *this;
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 8; ++j)
        e_M(i, j) = _M[i][j];
  }
  inline void operator = (const Eigen::Matrix<float, 2, 8> &e_M) { *((Eigen::Matrix<float, 2, 8> *) this) = e_M; }
  inline LA::AlignedMatrix2x8f GetAlignedMatrix2x8f() const {
    LA::AlignedMatrix2x8f M;
    float* _M[2] = {&M.m00(), &M.m10()};
    const Eigen::Matrix<float, 2, 8> &e_M = *this;
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 8; ++j)
        _M[i][j] = e_M(i, j);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix2x8f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix2x8f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix2x8f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix2x8f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix2x8f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
