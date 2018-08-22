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
#ifndef _MATRIX_9x9_H_
#define _MATRIX_9x9_H_

#include "Vector9.h"

namespace LA {

template<typename TYPE> class Matrix9x9 {
 public:

  inline Matrix9x9<TYPE>() {}
  inline Matrix9x9<TYPE>(const TYPE *M) { Set(M); }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  inline const TYPE* operator[] (const int i) const { return m_data[i]; }
  inline       TYPE* operator[] (const int i)       { return m_data[i]; }

  inline const TYPE& operator() (const int i, const int j) const { return m_data[i][j]; }
  inline       TYPE& operator() (const int i, const int j)       { return m_data[i][j]; }

  inline void operator *= (const TYPE s) {
    for (int i = 0; i < 9; ++i) {
      for (int j = 0; j < 9; ++j) {
        m_data[i][j] *= s;
      }
    }
  }
  inline Matrix9x9<TYPE> operator - (const Matrix9x9<TYPE> &B) const {
    Matrix9x9<TYPE> AmB;
    for (int i = 0; i < 9; ++i) {
      for (int j = 0; j < 9; ++j) {
        AmB.m_data[i][j] = m_data[i][j] - B.m_data[i][j];
      }
    }
    return AmB;
  }
  
  inline void SetLowerFromUpper() {
    for (int i = 0; i < 9; ++i) {
      for (int j = i; j < 9; ++j) {
        m_data[j][i] = m_data[i][j];
      }
    }
  }

  inline bool Valid() const { return m_data[0][0] != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m_data[0][0] == UT::Invalid<TYPE>(); }
  inline void Invalidate() { m_data[0][0] = UT::Invalid<TYPE>(); }

  inline void Set(const TYPE *M) { memcpy(this, M, sizeof(Matrix9x9<TYPE>)); }
  inline void SetBlock(const int i, const int j, const Matrix3x3f &B);
  inline void GetBlock(const int i, const int j, Matrix3x3f &B) const;
  inline void MakeZero() { memset(this, 0, sizeof(Matrix9x9<TYPE>)); }

  inline bool SolveLDL(Vector9<TYPE> &b, const TYPE *eps = NULL, const bool decomposed = false) {
    Matrix9x9<TYPE> &A = *this;
    TYPE* _A[9] = {A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]};
    return LS::SolveLDL<TYPE>(9, _A, b, eps, decomposed);
  }
  inline bool InverseLDL(const TYPE *eps = NULL, const bool decomposed = false) {
    return InverseLDL(*this, eps, decomposed);
  }
  static inline bool InverseLDL(Matrix9x9<TYPE> &A, const TYPE *eps = NULL,
                                const bool decomposed = false) {
    TYPE* _A[9] = {A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]};
    if (LS::InverseLDL<TYPE>(9, _A, eps, decomposed)) {
      A.SetLowerFromUpper();
      return true;
    } else {
      A.Invalidate();
      return false;
    }
  }

  inline void Print(const bool e = false) const {
    for (int i = 0; i < 9; ++i) {
      for (int j = 0; j < 9; ++j) {
        if (e) {
          UT::Print("%e ", m_data[i][j]);
        } else {
          UT::Print("%.4f ", m_data[i][j]);
        }
      }
      UT::Print("\n");
    }
  }
  inline bool AssertEqual(const Matrix9x9<TYPE> &M,
                          const int verbose = 1, const std::string str = "",
                          const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertEqual(&m_data[0][0], &M.m_data[0][0], 81, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const Matrix9x9<TYPE> E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }

 public:
  TYPE m_data[9][9];
};

typedef Matrix9x9<float> Matrix9x9f;
typedef Matrix9x9<double> Matrix9x9d;

template<> inline void Matrix9x9f::SetBlock(const int i, const int j, const Matrix3x3f &M) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= 9);
  UT_ASSERT(j >= 0 && j + 3 <= 9);
#endif
  memcpy(m_data[i] + j, M[0], 12);
  memcpy(m_data[i + 1] + j, M[1], 12);
  memcpy(m_data[i + 2] + j, M[2], 12);
}
template<> inline void Matrix9x9d::SetBlock(const int i, const int j, const Matrix3x3f &M) {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= 9);
  UT_ASSERT(j >= 0 && j + 3 <= 9);
#endif
  for (int _i = 0; _i < 3; ++_i) {
    for (int _j = 0; _j < 3; ++_j) {
      m_data[i + _i][j + _j] = static_cast<double>(M[_i][_j]);
    }
  }
}
template<> inline void Matrix9x9f::GetBlock(const int i, const int j, Matrix3x3f &M) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= 9);
  UT_ASSERT(j >= 0 && j + 3 <= 9);
#endif
  memcpy(M[0], m_data[i] + j, 12);
  memcpy(M[1], m_data[i + 1] + j, 12);
  memcpy(M[2], m_data[i + 2] + j, 12);
}
template<> inline void Matrix9x9d::GetBlock(const int i, const int j, Matrix3x3f &M) const {
#ifdef CFG_DEBUG
  UT_ASSERT(i >= 0 && i + 3 <= 9);
  UT_ASSERT(j >= 0 && j + 3 <= 9);
#endif
  for (int _i = 0; _i < 3; ++_i) {
    for (int _j = 0; _j < 3; ++_j) {
      M[_i][_j] = static_cast<float>(m_data[i + _i][j + _j]);
    }
  }
}

template<typename TYPE> class SymmetricMatrix9x9 {

 public:

  //inline SymmetricMatrix9x9<TYPE>() {}
  //inline SymmetricMatrix9x9<TYPE>(const TYPE *M) { Set(M); }

  inline const TYPE& m00() const { return m_data[0]; }  inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }  inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m02() const { return m_data[2]; }  inline TYPE& m02() { return m_data[2]; }
  inline const TYPE& m03() const { return m_data[3]; }  inline TYPE& m03() { return m_data[3]; }
  inline const TYPE& m04() const { return m_data[4]; }  inline TYPE& m04() { return m_data[4]; }
  inline const TYPE& m05() const { return m_data[5]; }  inline TYPE& m05() { return m_data[5]; }
  inline const TYPE& m06() const { return m_data[6]; }  inline TYPE& m06() { return m_data[6]; }
  inline const TYPE& m07() const { return m_data[7]; }  inline TYPE& m07() { return m_data[7]; }
  inline const TYPE& m08() const { return m_data[8]; }  inline TYPE& m08() { return m_data[8]; }
  inline const TYPE& m11() const { return m_data[9]; }  inline TYPE& m11() { return m_data[9]; }
  inline const TYPE& m12() const { return m_data[10]; } inline TYPE& m12() { return m_data[10]; }
  inline const TYPE& m13() const { return m_data[11]; } inline TYPE& m13() { return m_data[11]; }
  inline const TYPE& m14() const { return m_data[12]; } inline TYPE& m14() { return m_data[12]; }
  inline const TYPE& m15() const { return m_data[13]; } inline TYPE& m15() { return m_data[13]; }
  inline const TYPE& m16() const { return m_data[14]; } inline TYPE& m16() { return m_data[14]; }
  inline const TYPE& m17() const { return m_data[15]; } inline TYPE& m17() { return m_data[15]; }
  inline const TYPE& m18() const { return m_data[16]; } inline TYPE& m18() { return m_data[16]; }
  inline const TYPE& m22() const { return m_data[17]; } inline TYPE& m22() { return m_data[17]; }
  inline const TYPE& m23() const { return m_data[18]; } inline TYPE& m23() { return m_data[18]; }
  inline const TYPE& m24() const { return m_data[19]; } inline TYPE& m24() { return m_data[19]; }
  inline const TYPE& m25() const { return m_data[20]; } inline TYPE& m25() { return m_data[20]; }
  inline const TYPE& m26() const { return m_data[21]; } inline TYPE& m26() { return m_data[21]; }
  inline const TYPE& m27() const { return m_data[22]; } inline TYPE& m27() { return m_data[22]; }
  inline const TYPE& m28() const { return m_data[23]; } inline TYPE& m28() { return m_data[23]; }
  inline const TYPE& m33() const { return m_data[24]; } inline TYPE& m33() { return m_data[24]; }
  inline const TYPE& m34() const { return m_data[25]; } inline TYPE& m34() { return m_data[25]; }
  inline const TYPE& m35() const { return m_data[26]; } inline TYPE& m35() { return m_data[26]; }
  inline const TYPE& m36() const { return m_data[27]; } inline TYPE& m36() { return m_data[27]; }
  inline const TYPE& m37() const { return m_data[28]; } inline TYPE& m37() { return m_data[28]; }
  inline const TYPE& m38() const { return m_data[29]; } inline TYPE& m38() { return m_data[29]; }
  inline const TYPE& m44() const { return m_data[30]; } inline TYPE& m44() { return m_data[30]; }
  inline const TYPE& m45() const { return m_data[31]; } inline TYPE& m45() { return m_data[31]; }
  inline const TYPE& m46() const { return m_data[32]; } inline TYPE& m46() { return m_data[32]; }
  inline const TYPE& m47() const { return m_data[33]; } inline TYPE& m47() { return m_data[33]; }
  inline const TYPE& m48() const { return m_data[34]; } inline TYPE& m48() { return m_data[34]; }
  inline const TYPE& m55() const { return m_data[35]; } inline TYPE& m55() { return m_data[35]; }
  inline const TYPE& m56() const { return m_data[36]; } inline TYPE& m56() { return m_data[36]; }
  inline const TYPE& m57() const { return m_data[37]; } inline TYPE& m57() { return m_data[37]; }
  inline const TYPE& m58() const { return m_data[38]; } inline TYPE& m58() { return m_data[38]; }
  inline const TYPE& m66() const { return m_data[39]; } inline TYPE& m66() { return m_data[39]; }
  inline const TYPE& m67() const { return m_data[40]; } inline TYPE& m67() { return m_data[40]; }
  inline const TYPE& m68() const { return m_data[41]; } inline TYPE& m68() { return m_data[41]; }
  inline const TYPE& m77() const { return m_data[42]; } inline TYPE& m77() { return m_data[42]; }
  inline const TYPE& m78() const { return m_data[43]; } inline TYPE& m78() { return m_data[43]; }
  inline const TYPE& m88() const { return m_data[44]; } inline TYPE& m88() { return m_data[44]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }

  //inline void Set(const TYPE *M) { memcpy(this, M, sizeof(SymmetricMatrix9x9<TYPE>)); }
  inline void Set(const SymmetricMatrix3x3<TYPE> &M00, const AlignedMatrix3x3f &M01,
                  const AlignedMatrix3x3f &M02, const SymmetricMatrix3x3<TYPE> &M11,
                  const AlignedMatrix3x3f &M12, const SymmetricMatrix3x3<TYPE> &M22) {
    Set00(M00);
    Set03(M01);
    Set06(M02);
    Set33(M11);
    Set36(M12);
    Set66(M22);
  }
  inline void Set(const AlignedMatrix3x3f &M00, const AlignedMatrix3x3f &M01,
                  const AlignedMatrix3x3f &M02, const AlignedMatrix3x3f &M11,
                  const AlignedMatrix3x3f &M12, const AlignedMatrix3x3f &M22) {
    Set00(M00);
    Set03(M01);
    Set06(M02);
    Set33(M11);
    Set36(M12);
    Set66(M22);
  }
  inline void Set(const SymmetricMatrix3x3<TYPE> &M00, const TYPE M11, const TYPE M22) {
    MakeZero();
    Set00(M00);
    m33() = m44() = m55() = M11;
    m66() = m77() = m88() = M22;
  }
  inline void Set00(const SymmetricMatrix3x3<TYPE> &M);
  inline void Set00(const AlignedMatrix3x3f &M);
  inline void Set03(const AlignedMatrix3x3f &M);
  inline void Set06(const AlignedMatrix3x3f &M);
  inline void Set33(const SymmetricMatrix3x3<TYPE> &M);
  inline void Set33(const AlignedMatrix3x3f &M);
  inline void Set33(const TYPE *M0, const TYPE *M1, const TYPE *M2,
                    const TYPE *M3, const TYPE *M4, const TYPE *M5);
  inline void Set36(const AlignedMatrix3x3f &M);
  inline void Set66(const SymmetricMatrix3x3<TYPE> &M);
  inline void Set66(const AlignedMatrix3x3f &M);

  inline void Get00(SymmetricMatrix3x3<TYPE> *M) const;
  inline void Get00(AlignedMatrix3x3f *M) const;
  inline void Get03(AlignedMatrix3x3f *M) const;
  inline void Get06(AlignedMatrix3x3f *M) const;
  inline void Get33(SymmetricMatrix3x3<TYPE> *M) const;
  inline void Get33(AlignedMatrix3x3f *M) const;
  inline void Get36(AlignedMatrix3x3f *M) const;
  inline void Get66(SymmetricMatrix3x3<TYPE> *M) const;
  inline void Get66(AlignedMatrix3x3f *M) const;

  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix9x9<TYPE>)); }
  inline void MakeDiagonal(const TYPE d) { MakeZero(); SetDiagonal(d); }
  inline void SetDiagonal(const TYPE d) {
    m00() = d; m11() = d; m22() = d;
    m33() = d; m44() = d; m55() = d;
    m66() = d; m77() = d; m88() = d;
  }
  inline void IncreaseDiagonal012(const TYPE d) {
    m00() += d;
    m11() += d;
    m22() += d;
  }
  inline void IncreaseDiagonal345(const TYPE d) {
    m33() += d;
    m44() += d;
    m55() += d;
  }
  inline void IncreaseDiagonal678(const TYPE d) {
    m66() += d;
    m77() += d;
    m88() += d;
  }

  inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }

  inline void Print(const bool e = false, const bool f = false) const {
    if (f) {
      if (e) {
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m00(), m01(), m02(),
                                                  m03(), m04(), m05(),
                                                  m06(), m07(), m08());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m01(), m11(), m12(),
                                                  m13(), m14(), m15(),
                                                  m16(), m17(), m18());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m02(), m12(), m22(),
                                                  m23(), m24(), m25(),
                                                  m26(), m27(), m28());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m03(), m13(), m23(),
                                                  m33(), m34(), m35(),
                                                  m36(), m37(), m38());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m04(), m14(), m24(),
                                                  m34(), m44(), m45(),
                                                  m46(), m47(), m48());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m05(), m15(), m25(),
                                                  m35(), m45(), m55(),
                                                  m56(), m57(), m58());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m06(), m16(), m26(),
                                                  m36(), m46(), m56(),
                                                  m66(), m67(), m68());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m07(), m17(), m27(),
                                                  m37(), m47(), m47(),
                                                  m67(), m77(), m78());
        UT::Print("%e %e %e %e %e %e %e %e %e\n", m08(), m18(), m28(),
                                                  m38(), m48(), m58(),
                                                  m68(), m78(), m88());
      } else {
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m00(), m01(), m02(),
                                                  m03(), m04(), m05(),
                                                  m06(), m07(), m08());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m01(), m11(), m12(),
                                                  m13(), m14(), m15(),
                                                  m16(), m17(), m18());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m02(), m12(), m22(),
                                                  m23(), m24(), m25(),
                                                  m26(), m27(), m28());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m03(), m13(), m23(),
                                                  m33(), m34(), m35(),
                                                  m36(), m37(), m38());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m04(), m14(), m24(),
                                                  m34(), m44(), m45(),
                                                  m46(), m47(), m48());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m05(), m15(), m25(),
                                                  m35(), m45(), m55(),
                                                  m56(), m57(), m58());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m06(), m16(), m26(),
                                                  m36(), m46(), m56(),
                                                  m66(), m67(), m68());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m07(), m17(), m27(),
                                                  m37(), m47(), m47(),
                                                  m67(), m77(), m78());
        UT::Print("%f %f %f %f %f %f %f %f %f\n", m08(), m18(), m28(),
                                                  m38(), m48(), m58(),
                                                  m68(), m78(), m88());
      }
    } else {
      for (int i = 0; i < 45; ++i) {
        if (e) {
          UT::Print("%e ", m_data[i]);
        } else {
          UT::Print("%f ", m_data[i]);
        }
      }
      UT::Print("\n");
    }
  }
  inline void Print(const std::string str, const bool e) const {
    const std::string _str(str.size(), ' ');
    if (e) {
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n",  str.c_str(), m00(), m01(), m02(),
                                                                m03(), m04(), m05(),
                                                                m06(), m07(), m08());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m01(), m11(), m12(),
                                                                m13(), m14(), m15(),
                                                                m16(), m17(), m18());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m02(), m12(), m22(),
                                                                m23(), m24(), m25(),
                                                                m26(), m27(), m28());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m03(), m13(), m23(),
                                                                m33(), m34(), m35(),
                                                                m36(), m37(), m38());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m04(), m14(), m24(),
                                                                m34(), m44(), m45(),
                                                                m46(), m47(), m48());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m05(), m15(), m25(),
                                                                m35(), m45(), m55(),
                                                                m56(), m57(), m58());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m06(), m16(), m26(),
                                                                m36(), m46(), m56(),
                                                                m66(), m67(), m68());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m07(), m17(), m27(),
                                                                m37(), m47(), m47(),
                                                                m67(), m77(), m78());
      UT::Print("%s%e %e %e %e %e %e %e %e %e\n", _str.c_str(), m08(), m18(), m28(),
                                                                m38(), m48(), m58(),
                                                                m68(), m78(), m88());
    } else {
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n",  str.c_str(), m00(), m01(), m02(),
                                                                m03(), m04(), m05(),
                                                                m06(), m07(), m08());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m01(), m11(), m12(),
                                                                m13(), m14(), m15(),
                                                                m16(), m17(), m18());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m02(), m12(), m22(),
                                                                m23(), m24(), m25(),
                                                                m26(), m27(), m28());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m03(), m13(), m23(),
                                                                m33(), m34(), m35(),
                                                                m36(), m37(), m38());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m04(), m14(), m24(),
                                                                m34(), m44(), m45(),
                                                                m46(), m47(), m48());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m05(), m15(), m25(),
                                                                m35(), m45(), m55(),
                                                                m56(), m57(), m58());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m06(), m16(), m26(),
                                                                m36(), m46(), m56(),
                                                                m66(), m67(), m68());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m07(), m17(), m27(),
                                                                m37(), m47(), m47(),
                                                                m67(), m77(), m78());
      UT::Print("%s%f %f %f %f %f %f %f %f %f\n", _str.c_str(), m08(), m18(), m28(),
                                                                m38(), m48(), m58(),
                                                                m68(), m78(), m88());
    }
  }
  // TODO(yanghongtian) : update comments here for this function.
  static inline void ABTTo00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix9x9<TYPE> &ABT) {
    ABT.m00() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m01() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m02() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m11() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m12() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m22() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo03(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix9x9<TYPE> &ABT) {
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
  // this is confusing too, A * BT is not promised to be a symmetrix matrix.
  static inline void ABTTo33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix9x9<TYPE> &ABT) {
    ABT.m33() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m34() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m35() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m44() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m45() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m55() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }
  static inline void ABTTo66(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B,
                             SymmetricMatrix9x9<TYPE> &ABT) {
    ABT.m66() = (A.m_00_01_02_r0() * B.m_00_01_02_r0()).vsum_012();
    ABT.m67() = (A.m_00_01_02_r0() * B.m_10_11_12_r1()).vsum_012();
    ABT.m68() = (A.m_00_01_02_r0() * B.m_20_21_22_r2()).vsum_012();
    ABT.m77() = (A.m_10_11_12_r1() * B.m_10_11_12_r1()).vsum_012();
    ABT.m78() = (A.m_10_11_12_r1() * B.m_20_21_22_r2()).vsum_012();
    ABT.m88() = (A.m_20_21_22_r2() * B.m_20_21_22_r2()).vsum_012();
  }

 protected:
  TYPE m_data[45];
};

typedef SymmetricMatrix9x9<float> SymmetricMatrix9x9f;
typedef SymmetricMatrix9x9<double> SymmetricMatrix9x9d;

template<> inline void SymmetricMatrix9x9f::Set00(const SymmetricMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m11(), &M.m11(), 8);
  m22() = M.m22();
}
template<> inline void SymmetricMatrix9x9f::Set00(const AlignedMatrix3x3f &M) {
  memcpy(&m00(), &M.m00(), 12);
  memcpy(&m11(), &M.m11(), 8);
  m22() = M.m22();
}
template<> inline void SymmetricMatrix9x9f::Set03(const AlignedMatrix3x3f &M) {
  memcpy(&m03(), &M.m00(), 12);
  memcpy(&m13(), &M.m10(), 12);
  memcpy(&m23(), &M.m20(), 12);
}
template<> inline void SymmetricMatrix9x9f::Set06(const AlignedMatrix3x3f &M) {
  memcpy(&m06(), &M.m00(), 12);
  memcpy(&m16(), &M.m10(), 12);
  memcpy(&m26(), &M.m20(), 12);
}
template<> inline void SymmetricMatrix9x9f::Set33(const SymmetricMatrix3x3f &M) {
  memcpy(&m33(), &M.m00(), 12);
  memcpy(&m44(), &M.m11(), 8);
  m55() = M.m22();
}
template<> inline void SymmetricMatrix9x9f::Set33(const AlignedMatrix3x3f &M) {
  memcpy(&m33(), &M.m00(), 12);
  memcpy(&m44(), &M.m11(), 8);
  m55() = M.m22();
}
template<> inline void SymmetricMatrix9x9f::Set33(const float *M0, const float *M1,
                                                  const float *M2, const float *M3,
                                                  const float *M4, const float *M5) {
  memcpy(&m33(), M0, 24);
  memcpy(&m44(), M1 + 1, 20);
  memcpy(&m55(), M2 + 2, 16);
  memcpy(&m66(), M3 + 3, 12);
  memcpy(&m77(), M4 + 4, 8);
  m88() = M5[5];
}
template<> inline void SymmetricMatrix9x9f::Set36(const AlignedMatrix3x3f &M) {
  memcpy(&m36(), &M.m00(), 12);
  memcpy(&m46(), &M.m10(), 12);
  memcpy(&m56(), &M.m20(), 12);
}
template<> inline void SymmetricMatrix9x9f::Set66(const SymmetricMatrix3x3f &M) {
  memcpy(&m66(), &M.m00(), 12);
  memcpy(&m77(), &M.m11(), 8);
  m88() = M.m22();
}
template<> inline void SymmetricMatrix9x9f::Set66(const AlignedMatrix3x3f &M) {
  memcpy(&m66(), &M.m00(), 12);
  memcpy(&m77(), &M.m11(), 8);
  m88() = M.m22();
}

template<> inline void SymmetricMatrix9x9f::Get00(SymmetricMatrix3x3f *M) const {
  memcpy(&M->m00(), &m00(), 12);
  memcpy(&M->m11(), &m11(), 8);
  M->m22() = m22();
}
template<> inline void SymmetricMatrix9x9f::Get00(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m00(), 12);
  memcpy(&M->m11(), &m11(), 8);
  M->m22() = m22();
  M->SetLowerFromUpper();
}
template<> inline void SymmetricMatrix9x9f::Get03(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m03(), 12);
  memcpy(&M->m10(), &m13(), 12);
  memcpy(&M->m20(), &m23(), 12);
}
template<> inline void SymmetricMatrix9x9f::Get06(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m06(), 12);
  memcpy(&M->m10(), &m16(), 12);
  memcpy(&M->m20(), &m26(), 12);
}
template<> inline void SymmetricMatrix9x9f::Get33(SymmetricMatrix3x3f *M) const {
  memcpy(&M->m00(), &m33(), 12);
  memcpy(&M->m11(), &m44(), 8);
  M->m22() = m55();
}
template<> inline void SymmetricMatrix9x9f::Get33(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m33(), 12);
  memcpy(&M->m11(), &m44(), 8);
  M->m22() = m55();
  M->SetLowerFromUpper();
}
template<> inline void SymmetricMatrix9x9f::Get36(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m36(), 12);
  memcpy(&M->m10(), &m46(), 12);
  memcpy(&M->m20(), &m56(), 12);
}
template<> inline void SymmetricMatrix9x9f::Get66(SymmetricMatrix3x3f *M) const {
  memcpy(&M->m00(), &m66(), 12);
  memcpy(&M->m11(), &m77(), 8);
  M->m22() = m88();
}
template<> inline void SymmetricMatrix9x9f::Get66(AlignedMatrix3x3f *M) const {
  memcpy(&M->m00(), &m66(), 12);
  memcpy(&M->m11(), &m77(), 8);
  M->m22() = m88();
  M->SetLowerFromUpper();
}

}

//#ifdef CFG_DEBUG_EIGEN
#if 0
class EigenMatrix9x9f : public Eigen::Matrix<float, 9, 9, Eigen::RowMajor> {
 public:
  inline EigenMatrix9x9f() : Eigen::Matrix<float, 9, 9, Eigen::RowMajor>() {}
  inline EigenMatrix9x9f(const Eigen::Matrix<float, 9, 9, Eigen::RowMajor> &e_M) :
                         Eigen::Matrix<float, 9, 9, Eigen::RowMajor>(e_M) {}
  inline EigenMatrix9x9f(const LA::Matrix9x9f &M) : Eigen::Matrix<float, 9, 9, Eigen::RowMajor>() {
    *this = M;
  }
  inline EigenMatrix9x9f(const LA::SymmetricMatrix9x9f &M) :
                         Eigen::Matrix<float, 9, 9, Eigen::RowMajor>() {
    *this = M;
  }
  inline EigenMatrix9x9f(const LA::SymmetricMatrix3x3f &M00, const LA::AlignedMatrix3x3f &M01,
                         const LA::AlignedMatrix3x3f &M02, const LA::SymmetricMatrix3x3f &M11,
                         const LA::AlignedMatrix3x3f &M12, const LA::SymmetricMatrix3x3f &M22) {
    block<3, 3>(0, 0) = EigenMatrix3x3f(M00);
    block<3, 3>(0, 3) = EigenMatrix3x3f(M01);
    block<3, 3>(0, 6) = EigenMatrix3x3f(M02);
    block<3, 3>(3, 0) = block<3, 3>(0, 3).transpose();
    block<3, 3>(3, 3) = EigenMatrix3x3f(M11);
    block<3, 3>(3, 6) = EigenMatrix3x3f(M12);
    block<3, 3>(6, 0) = block<3, 3>(0, 6).transpose();
    block<3, 3>(6, 3) = block<3, 3>(3, 6).transpose();
    block<3, 3>(6, 6) = EigenMatrix3x3f(M22);
  }
  inline EigenMatrix9x9f(const LA::AlignedMatrix3x3f &M00, const LA::AlignedMatrix3x3f &M01,
                         const LA::AlignedMatrix3x3f &M02, const LA::AlignedMatrix3x3f &M10,
                         const LA::AlignedMatrix3x3f &M11, const LA::AlignedMatrix3x3f &M12,
                         const LA::AlignedMatrix3x3f &M20, const LA::AlignedMatrix3x3f &M21,
                         const LA::AlignedMatrix3x3f &M22) {
    block<3, 3>(0, 0) = EigenMatrix3x3f(M00);   block<3, 3>(0, 3) = EigenMatrix3x3f(M01);
    block<3, 3>(0, 6) = EigenMatrix3x3f(M02);   block<3, 3>(3, 0) = EigenMatrix3x3f(M10);
    block<3, 3>(3, 3) = EigenMatrix3x3f(M11);   block<3, 3>(3, 6) = EigenMatrix3x3f(M12);
    block<3, 3>(6, 0) = EigenMatrix3x3f(M20);   block<3, 3>(6, 3) = EigenMatrix3x3f(M21);
    block<3, 3>(6, 6) = EigenMatrix3x3f(M22);
  }
  inline void operator = (const Eigen::Matrix<float, 9, 9, Eigen::RowMajor> &e_M) {
    *((Eigen::Matrix<float, 9, 9, Eigen::RowMajor> *) this) = e_M;
  }
  inline void operator = (const LA::Matrix9x9f &M) {
    Eigen::Matrix<float, 9, 9, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        e_M(i, j) = M[i][j];
  }
  inline void operator = (const LA::SymmetricMatrix9x9f &M) {
    Eigen::Matrix<float, 9, 9, Eigen::RowMajor> &e_M = *this;
    const float *_M = M;
    for (int i = 0, k = 0; i < 9; ++i)
      for (int j = i; j < 9; ++j, ++k)
        e_M(i, j) = e_M(j, i) = _M[k];
  }
  inline LA::Matrix9x9f GetMatrix9x9f() const {
    LA::Matrix9x9f M;
    const Eigen::Matrix<float, 9, 9, Eigen::RowMajor> &e_M = *this;
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        M[i][j] = e_M(i, j);
    return M;
  }
  inline void Print(const bool e = false) const { GetMatrix9x9f().Print(e); }
  inline bool AssertEqual(const LA::Matrix9x9f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetMatrix9x9f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix9x9f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetMatrix9x9f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
