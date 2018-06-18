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
#ifndef _MATRIX_8x8_H_
#define _MATRIX_8x8_H_

#include "Vector8.h"
#include "LinearSystem.h"

namespace LA {

class AlignedMatrix8x8f {
 public:
  inline const xp128f& m_00_01_02_03() const { return m_data[0]; }  inline xp128f& m_00_01_02_03() { return m_data[0]; }
  inline const xp128f& m_04_05_06_07() const { return m_data[1]; }  inline xp128f& m_04_05_06_07() { return m_data[1]; }
  inline const xp128f& m_10_11_12_13() const { return m_data[2]; }  inline xp128f& m_10_11_12_13() { return m_data[2]; }
  inline const xp128f& m_14_15_16_17() const { return m_data[3]; }  inline xp128f& m_14_15_16_17() { return m_data[3]; }
  inline const xp128f& m_20_21_22_23() const { return m_data[4]; }  inline xp128f& m_20_21_22_23() { return m_data[4]; }
  inline const xp128f& m_24_25_26_27() const { return m_data[5]; }  inline xp128f& m_24_25_26_27() { return m_data[5]; }
  inline const xp128f& m_30_31_32_33() const { return m_data[6]; }  inline xp128f& m_30_31_32_33() { return m_data[6]; }
  inline const xp128f& m_34_35_36_37() const { return m_data[7]; }  inline xp128f& m_34_35_36_37() { return m_data[7]; }
  inline const xp128f& m_40_41_42_43() const { return m_data[8]; }  inline xp128f& m_40_41_42_43() { return m_data[8]; }
  inline const xp128f& m_44_45_46_47() const { return m_data[9]; }  inline xp128f& m_44_45_46_47() { return m_data[9]; }
  inline const xp128f& m_50_51_52_53() const { return m_data[10]; } inline xp128f& m_50_51_52_53() { return m_data[10]; }
  inline const xp128f& m_54_55_56_57() const { return m_data[11]; } inline xp128f& m_54_55_56_57() { return m_data[11]; }
  inline const xp128f& m_60_61_62_63() const { return m_data[12]; } inline xp128f& m_60_61_62_63() { return m_data[12]; }
  inline const xp128f& m_64_65_66_67() const { return m_data[13]; } inline xp128f& m_64_65_66_67() { return m_data[13]; }
  inline const xp128f& m_70_71_72_73() const { return m_data[14]; } inline xp128f& m_70_71_72_73() { return m_data[14]; }
  inline const xp128f& m_74_75_76_77() const { return m_data[15]; } inline xp128f& m_74_75_76_77() { return m_data[15]; }

  inline const float& m00() const { return m_data[0][0]; }  inline float& m00() { return m_data[0][0]; }
  inline const float& m01() const { return m_data[0][1]; }  inline float& m01() { return m_data[0][1]; }
  inline const float& m02() const { return m_data[0][2]; }  inline float& m02() { return m_data[0][2]; }
  inline const float& m03() const { return m_data[0][3]; }  inline float& m03() { return m_data[0][3]; }
  inline const float& m04() const { return m_data[1][0]; }  inline float& m04() { return m_data[1][0]; }
  inline const float& m05() const { return m_data[1][1]; }  inline float& m05() { return m_data[1][1]; }
  inline const float& m06() const { return m_data[1][2]; }  inline float& m06() { return m_data[1][2]; }
  inline const float& m07() const { return m_data[1][3]; }  inline float& m07() { return m_data[1][3]; }
  inline const float& m10() const { return m_data[2][0]; }  inline float& m10() { return m_data[2][0]; }
  inline const float& m11() const { return m_data[2][1]; }  inline float& m11() { return m_data[2][1]; }
  inline const float& m12() const { return m_data[2][2]; }  inline float& m12() { return m_data[2][2]; }
  inline const float& m13() const { return m_data[2][3]; }  inline float& m13() { return m_data[2][3]; }
  inline const float& m14() const { return m_data[3][0]; }  inline float& m14() { return m_data[3][0]; }
  inline const float& m15() const { return m_data[3][1]; }  inline float& m15() { return m_data[3][1]; }
  inline const float& m16() const { return m_data[3][2]; }  inline float& m16() { return m_data[3][2]; }
  inline const float& m17() const { return m_data[3][3]; }  inline float& m17() { return m_data[3][3]; }
  inline const float& m20() const { return m_data[4][0]; }  inline float& m20() { return m_data[4][0]; }
  inline const float& m21() const { return m_data[4][1]; }  inline float& m21() { return m_data[4][1]; }
  inline const float& m22() const { return m_data[4][2]; }  inline float& m22() { return m_data[4][2]; }
  inline const float& m23() const { return m_data[4][3]; }  inline float& m23() { return m_data[4][3]; }
  inline const float& m24() const { return m_data[5][0]; }  inline float& m24() { return m_data[5][0]; }
  inline const float& m25() const { return m_data[5][1]; }  inline float& m25() { return m_data[5][1]; }
  inline const float& m26() const { return m_data[5][2]; }  inline float& m26() { return m_data[5][2]; }
  inline const float& m27() const { return m_data[5][3]; }  inline float& m27() { return m_data[5][3]; }
  inline const float& m30() const { return m_data[6][0]; }  inline float& m30() { return m_data[6][0]; }
  inline const float& m31() const { return m_data[6][1]; }  inline float& m31() { return m_data[6][1]; }
  inline const float& m32() const { return m_data[6][2]; }  inline float& m32() { return m_data[6][2]; }
  inline const float& m33() const { return m_data[6][3]; }  inline float& m33() { return m_data[6][3]; }
  inline const float& m34() const { return m_data[7][0]; }  inline float& m34() { return m_data[7][0]; }
  inline const float& m35() const { return m_data[7][1]; }  inline float& m35() { return m_data[7][1]; }
  inline const float& m36() const { return m_data[7][2]; }  inline float& m36() { return m_data[7][2]; }
  inline const float& m37() const { return m_data[7][3]; }  inline float& m37() { return m_data[7][3]; }
  inline const float& m40() const { return m_data[8][0]; }  inline float& m40() { return m_data[8][0]; }
  inline const float& m41() const { return m_data[8][1]; }  inline float& m41() { return m_data[8][1]; }
  inline const float& m42() const { return m_data[8][2]; }  inline float& m42() { return m_data[8][2]; }
  inline const float& m43() const { return m_data[8][3]; }  inline float& m43() { return m_data[8][3]; }
  inline const float& m44() const { return m_data[9][0]; }  inline float& m44() { return m_data[9][0]; }
  inline const float& m45() const { return m_data[9][1]; }  inline float& m45() { return m_data[9][1]; }
  inline const float& m46() const { return m_data[9][2]; }  inline float& m46() { return m_data[9][2]; }
  inline const float& m47() const { return m_data[9][3]; }  inline float& m47() { return m_data[9][3]; }
  inline const float& m50() const { return m_data[10][0]; } inline float& m50() { return m_data[10][0]; }
  inline const float& m51() const { return m_data[10][1]; } inline float& m51() { return m_data[10][1]; }
  inline const float& m52() const { return m_data[10][2]; } inline float& m52() { return m_data[10][2]; }
  inline const float& m53() const { return m_data[10][3]; } inline float& m53() { return m_data[10][3]; }
  inline const float& m54() const { return m_data[11][0]; } inline float& m54() { return m_data[11][0]; }
  inline const float& m55() const { return m_data[11][1]; } inline float& m55() { return m_data[11][1]; }
  inline const float& m56() const { return m_data[11][2]; } inline float& m56() { return m_data[11][2]; }
  inline const float& m57() const { return m_data[11][3]; } inline float& m57() { return m_data[11][3]; }
  inline const float& m60() const { return m_data[12][0]; } inline float& m60() { return m_data[12][0]; }
  inline const float& m61() const { return m_data[12][1]; } inline float& m61() { return m_data[12][1]; }
  inline const float& m62() const { return m_data[12][2]; } inline float& m62() { return m_data[12][2]; }
  inline const float& m63() const { return m_data[12][3]; } inline float& m63() { return m_data[12][3]; }
  inline const float& m64() const { return m_data[13][0]; } inline float& m64() { return m_data[13][0]; }
  inline const float& m65() const { return m_data[13][1]; } inline float& m65() { return m_data[13][1]; }
  inline const float& m66() const { return m_data[13][2]; } inline float& m66() { return m_data[13][2]; }
  inline const float& m67() const { return m_data[13][3]; } inline float& m67() { return m_data[13][3]; }
  inline const float& m70() const { return m_data[14][0]; } inline float& m70() { return m_data[14][0]; }
  inline const float& m71() const { return m_data[14][1]; } inline float& m71() { return m_data[14][1]; }
  inline const float& m72() const { return m_data[14][2]; } inline float& m72() { return m_data[14][2]; }
  inline const float& m73() const { return m_data[14][3]; } inline float& m73() { return m_data[14][3]; }
  inline const float& m74() const { return m_data[15][0]; } inline float& m74() { return m_data[15][0]; }
  inline const float& m75() const { return m_data[15][1]; } inline float& m75() { return m_data[15][1]; }
  inline const float& m76() const { return m_data[15][2]; } inline float& m76() { return m_data[15][2]; }
  inline const float& m77() const { return m_data[15][3]; } inline float& m77() { return m_data[15][3]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline const float& operator() (int row, int col) const {
    return data[row * 8 + col];
  }

  inline float& operator() (int row, int col) {
    return data[row * 8 + col];
  }
  inline AlignedMatrix8x8f operator - (const AlignedMatrix8x8f &B) const {
    AlignedMatrix8x8f AmB;
    for (int i = 0; i < 16; ++i)
      AmB.m_data[i] = m_data[i] - B.m_data[i];
    return AmB;
  }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix8x8f)); }
  inline void MakeIdentity() {
    MakeZero();
    m00() = m11() = m22() = m33() = m44() = m55() = m66() = m77() = 1.0f;
  }

  inline void SetLowerFromUpper() {
    m10() = m01();
    m20() = m02();  m21() = m12();
    m30() = m03();  m31() = m13();  m32() = m23();
    m40() = m04();  m41() = m14();  m42() = m24();  m43() = m34();
    m50() = m05();  m51() = m15();  m52() = m25();  m53() = m35();  m54() = m45();
    m60() = m06();  m61() = m16();  m62() = m26();  m63() = m36();  m64() = m46();  m65() = m56();
    m70() = m07();  m71() = m17();  m72() = m27();  m73() = m37();  m74() = m47();  m75() = m57();
    m76() = m67();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(), m05(), m06(), m07());
      UT::Print("%e %e %e %e %e %e %e %e\n", m10(), m11(), m12(), m13(), m14(), m15(), m16(), m17());
      UT::Print("%e %e %e %e %e %e %e %e\n", m20(), m21(), m22(), m23(), m24(), m25(), m26(), m27());
      UT::Print("%e %e %e %e %e %e %e %e\n", m30(), m31(), m32(), m33(), m34(), m35(), m36(), m37());
      UT::Print("%e %e %e %e %e %e %e %e\n", m40(), m41(), m42(), m43(), m44(), m45(), m46(), m47());
      UT::Print("%e %e %e %e %e %e %e %e\n", m50(), m51(), m52(), m53(), m54(), m55(), m56(), m57());
      UT::Print("%e %e %e %e %e %e %e %e\n", m60(), m61(), m62(), m63(), m64(), m65(), m66(), m67());
      UT::Print("%e %e %e %e %e %e %e %e\n", m70(), m71(), m72(), m73(), m74(), m75(), m76(), m77());
    } else {
      UT::Print("%f %f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(), m05(), m06(), m07());
      UT::Print("%f %f %f %f %f %f %f %f\n", m10(), m11(), m12(), m13(), m14(), m15(), m16(), m17());
      UT::Print("%f %f %f %f %f %f %f %f\n", m20(), m21(), m22(), m23(), m24(), m25(), m26(), m27());
      UT::Print("%f %f %f %f %f %f %f %f\n", m30(), m31(), m32(), m33(), m34(), m35(), m36(), m37());
      UT::Print("%f %f %f %f %f %f %f %f\n", m40(), m41(), m42(), m43(), m44(), m45(), m46(), m47());
      UT::Print("%f %f %f %f %f %f %f %f\n", m50(), m51(), m52(), m53(), m54(), m55(), m56(), m57());
      UT::Print("%f %f %f %f %f %f %f %f\n", m60(), m61(), m62(), m63(), m64(), m65(), m66(), m67());
      UT::Print("%f %f %f %f %f %f %f %f\n", m70(), m71(), m72(), m73(), m74(), m75(), m76(), m77());
    }
  }
  inline bool AssertEqual(const AlignedMatrix8x8f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 64, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix8x8f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }

 protected:
  union {
    xp128f m_data[16];
    float data[16 * 4];
  };
};

template<typename TYPE> class SymmetricMatrix8x8 {

 public:

  inline SymmetricMatrix8x8<TYPE>() {}
  inline SymmetricMatrix8x8<TYPE>(const TYPE *M) { Set(M); }

  inline const TYPE& m00() const { return m_data[0]; }  inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }  inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m02() const { return m_data[2]; }  inline TYPE& m02() { return m_data[2]; }
  inline const TYPE& m03() const { return m_data[3]; }  inline TYPE& m03() { return m_data[3]; }
  inline const TYPE& m04() const { return m_data[4]; }  inline TYPE& m04() { return m_data[4]; }
  inline const TYPE& m05() const { return m_data[5]; }  inline TYPE& m05() { return m_data[5]; }
  inline const TYPE& m06() const { return m_data[6]; }  inline TYPE& m06() { return m_data[6]; }
  inline const TYPE& m07() const { return m_data[7]; }  inline TYPE& m07() { return m_data[7]; }
  inline const TYPE& m11() const { return m_data[8]; }  inline TYPE& m11() { return m_data[8]; }
  inline const TYPE& m12() const { return m_data[9]; }  inline TYPE& m12() { return m_data[9]; }
  inline const TYPE& m13() const { return m_data[10]; } inline TYPE& m13() { return m_data[10]; }
  inline const TYPE& m14() const { return m_data[11]; } inline TYPE& m14() { return m_data[11]; }
  inline const TYPE& m15() const { return m_data[12]; } inline TYPE& m15() { return m_data[12]; }
  inline const TYPE& m16() const { return m_data[13]; } inline TYPE& m16() { return m_data[13]; }
  inline const TYPE& m17() const { return m_data[14]; } inline TYPE& m17() { return m_data[14]; }
  inline const TYPE& m22() const { return m_data[15]; } inline TYPE& m22() { return m_data[15]; }
  inline const TYPE& m23() const { return m_data[16]; } inline TYPE& m23() { return m_data[16]; }
  inline const TYPE& m24() const { return m_data[17]; } inline TYPE& m24() { return m_data[17]; }
  inline const TYPE& m25() const { return m_data[18]; } inline TYPE& m25() { return m_data[18]; }
  inline const TYPE& m26() const { return m_data[19]; } inline TYPE& m26() { return m_data[19]; }
  inline const TYPE& m27() const { return m_data[20]; } inline TYPE& m27() { return m_data[20]; }
  inline const TYPE& m33() const { return m_data[21]; } inline TYPE& m33() { return m_data[21]; }
  inline const TYPE& m34() const { return m_data[22]; } inline TYPE& m34() { return m_data[22]; }
  inline const TYPE& m35() const { return m_data[23]; } inline TYPE& m35() { return m_data[23]; }
  inline const TYPE& m36() const { return m_data[24]; } inline TYPE& m36() { return m_data[24]; }
  inline const TYPE& m37() const { return m_data[25]; } inline TYPE& m37() { return m_data[25]; }
  inline const TYPE& m44() const { return m_data[26]; } inline TYPE& m44() { return m_data[26]; }
  inline const TYPE& m45() const { return m_data[27]; } inline TYPE& m45() { return m_data[27]; }
  inline const TYPE& m46() const { return m_data[28]; } inline TYPE& m46() { return m_data[28]; }
  inline const TYPE& m47() const { return m_data[29]; } inline TYPE& m47() { return m_data[29]; }
  inline const TYPE& m55() const { return m_data[30]; } inline TYPE& m55() { return m_data[30]; }
  inline const TYPE& m56() const { return m_data[31]; } inline TYPE& m56() { return m_data[31]; }
  inline const TYPE& m57() const { return m_data[32]; } inline TYPE& m57() { return m_data[32]; }
  inline const TYPE& m66() const { return m_data[33]; } inline TYPE& m66() { return m_data[33]; }
  inline const TYPE& m67() const { return m_data[34]; } inline TYPE& m67() { return m_data[34]; }
  inline const TYPE& m77() const { return m_data[35]; } inline TYPE& m77() { return m_data[35]; }

  inline const float& operator() (int row, int col) const {
    return m_data[row * 8 + col];
  }

  inline float& operator() (int row, int col) {
    return m_data[row * 8 + col];
  }

  inline operator const TYPE* () const { return m_data; }
  inline operator     TYPE* ()     { return m_data; }
  inline void operator += (const SymmetricMatrix8x8<TYPE> &M);
  inline void operator *= (const xp128f &s);

  inline void Set(const TYPE *M) { memcpy(this, M, sizeof(SymmetricMatrix8x8<TYPE>)); }

  inline AlignedMatrix8x8f GetAlignedMatrix8x8f() const;

  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix8x8<TYPE>)); }

  static inline void aaT(const AlignedVector8f &v, TYPE *aaT);
  static inline void ApB(const SymmetricMatrix8x8<TYPE> &A, const SymmetricMatrix8x8<TYPE> &B,
                         SymmetricMatrix8x8<TYPE> &ApB);

 protected:
  TYPE m_data[36];
};

typedef SymmetricMatrix8x8<float> SymmetricMatrix8x8f;
typedef SymmetricMatrix8x8<double> SymmetricMatrix8x8d;

template<> inline void SymmetricMatrix8x8f::operator += (const SymmetricMatrix8x8f &M) {
  xp128f *m1 = (xp128f *) this;
  const xp128f *m2 = (xp128f *) M.m_data;
  for (int i = 0; i < 9; ++i) {
    m1[i] += m2[i];
  }
}

template<> inline void SymmetricMatrix8x8f::operator *= (const xp128f &s) {
  xp128f *m = (xp128f *) m_data;
  for (int i = 0; i < 9; ++i) {
    m[i] *= s;
  }
}

template<> inline AlignedMatrix8x8f SymmetricMatrix8x8f::GetAlignedMatrix8x8f() const {
  AlignedMatrix8x8f M;
  M.m00() = m00();
  M.m01() = M.m10() = m01();
  M.m02() = M.m20() = m02();
  M.m03() = M.m30() = m03();
  M.m04() = M.m40() = m04();
  M.m05() = M.m50() = m05();
  M.m06() = M.m60() = m06();
  M.m07() = M.m70() = m07();
  M.m11() = m11();
  M.m12() = M.m21() = m12();
  M.m13() = M.m31() = m13();
  M.m14() = M.m41() = m14();
  M.m15() = M.m51() = m15();
  M.m16() = M.m61() = m16();
  M.m17() = M.m71() = m17();
  M.m22() = m22();
  M.m23() = M.m32() = m23();
  M.m24() = M.m42() = m24();
  M.m25() = M.m52() = m25();
  M.m26() = M.m62() = m26();
  M.m27() = M.m72() = m27();
  M.m33() = m33();
  M.m34() = M.m43() = m34();
  M.m35() = M.m53() = m35();
  M.m36() = M.m63() = m36();
  M.m37() = M.m73() = m37();
  M.m44() = m44();
  M.m45() = M.m54() = m45();
  M.m46() = M.m64() = m46();
  M.m47() = M.m74() = m47();
  M.m55() = m55();
  M.m56() = M.m65() = m56();
  M.m57() = M.m75() = m57();
  M.m66() = m66();
  M.m67() = M.m76() = m67();
  M.m77() = m77();
  return M;
}

template<> inline void SymmetricMatrix8x8f::aaT(const AlignedVector8f &a, float *aaT) {
  AlignedVector8f t;
  t.v4567().vset_all_lane(a.v0());
  t.v0123() = t.v4567() * a.v0123();
  t.v4567() = t.v4567() * a.v4567();
  memcpy(aaT, t, sizeof(t));
  t.v4567().vdup_all_lane(a.v1());
  t.v0123() = t.v4567() * a.v0123();
  t.v4567() = t.v4567() * a.v4567();
  memcpy(aaT + 8, &t.v1(), 28);
  aaT[15] = a.v2() * a.v2();
  aaT[16] = a.v2() * a.v3();
  t.v4567() = a.v4567() * a.v2();
  memcpy(aaT + 17, &t.v4(), 16);
  aaT[21] = a.v3() * a.v3();
  t.v4567() = a.v4567() * a.v3();
  memcpy(aaT + 22, &t.v4(), 16);
  t.v4567() = a.v4567() * a.v4();
  memcpy(aaT + 26, &t.v4(), 16);
  t.v4567() = a.v4567() * a.v5();
  memcpy(aaT + 30, &t.v5(), 12);
  aaT[33] = a.v6() * a.v6();
  aaT[34] = a.v6() * a.v7();
  aaT[35] = a.v7() * a.v7();
}

template<> inline void SymmetricMatrix8x8f::ApB(const SymmetricMatrix8x8f &A,
                                                const SymmetricMatrix8x8f &B, SymmetricMatrix8x8f &ApB) {
  const xp128f *a = (xp128f *) A.m_data, *b = (xp128f *) B.m_data;
  xp128f *apb = (xp128f *) ApB.m_data;
  for (int i = 0; i < 9; ++i) {
    apb[i] = a[i] + b[i];
  }
}

}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix8x8f : public Eigen::Matrix<float, 8, 8> {
 public:
  inline EigenMatrix8x8f() : Eigen::Matrix<float, 8, 8>() {}
  inline EigenMatrix8x8f(const Eigen::Matrix<float, 8, 8> &e_M) : Eigen::Matrix<float, 8, 8>(e_M) {}
  inline EigenMatrix8x8f(const LA::AlignedMatrix8x8f &M) : Eigen::Matrix<float, 8, 8>() {
    const float* _M[8] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(), &M.m40(), &M.m50(), &M.m60(), &M.m70()};
    Eigen::Matrix<float, 8, 8> &e_M = *this;
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        e_M(i, j) = _M[i][j];
  }
  inline EigenMatrix8x8f(const LA::SymmetricMatrix8x8f &M) : Eigen::Matrix<float, 8, 8>() {
    Eigen::Matrix<float, 8, 8> &e_M = *this;
    const float *_M = M;
    for (int i = 0, k = 0; i < 8; ++i)
      for (int j = i; j < 8; ++j, ++k)
        e_M(i, j) = e_M(j, i) = _M[k];
  }
  inline void operator = (const Eigen::Matrix<float, 8, 8> &e_M) { *((Eigen::Matrix<float, 8, 8> *) this) = e_M; }
  inline LA::AlignedMatrix8x8f GetAlignedMatrix8x8f() const {
    LA::AlignedMatrix8x8f M;
    float* _M[8] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(), &M.m40(), &M.m50(), &M.m60(), &M.m70()};
    const Eigen::Matrix<float, 8, 8> &e_M = *this;
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        _M[i][j] = e_M(i, j);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix8x8f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix8x8f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix8x8f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::SymmetricMatrix8x8f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix8x8f().AssertEqual(M.GetAlignedMatrix8x8f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix8x8f &e_M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix8x8f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
