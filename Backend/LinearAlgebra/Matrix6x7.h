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
#ifndef _MATRIX_6x7_H_
#define _MATRIX_6x7_H_

#include "Matrix2x7.h"
#include "Matrix3x7.h"
#include "Matrix6x6.h"
#include "LinearSystem.h"

namespace LA {

class AlignedMatrix6x7f {
 public:
  inline const xp128f& m_00_01_02_03() const { return m_data[0]; }  inline xp128f& m_00_01_02_03() { return m_data[0]; }
  inline const xp128f& m_04_05_06_x () const { return m_data[1]; }  inline xp128f& m_04_05_06_x () { return m_data[1]; }
  inline const xp128f& m_10_11_12_13() const { return m_data[2]; }  inline xp128f& m_10_11_12_13() { return m_data[2]; }
  inline const xp128f& m_14_15_16_x () const { return m_data[3]; }  inline xp128f& m_14_15_16_x () { return m_data[3]; }
  inline const xp128f& m_20_21_22_23() const { return m_data[4]; }  inline xp128f& m_20_21_22_23() { return m_data[4]; }
  inline const xp128f& m_24_25_26_x () const { return m_data[5]; }  inline xp128f& m_24_25_26_x () { return m_data[5]; }
  inline const xp128f& m_30_31_32_33() const { return m_data[6]; }  inline xp128f& m_30_31_32_33() { return m_data[6]; }
  inline const xp128f& m_34_35_36_x () const { return m_data[7]; }  inline xp128f& m_34_35_36_x () { return m_data[7]; }
  inline const xp128f& m_40_41_42_43() const { return m_data[8]; }  inline xp128f& m_40_41_42_43() { return m_data[8]; }
  inline const xp128f& m_44_45_46_x () const { return m_data[9]; }  inline xp128f& m_44_45_46_x () { return m_data[9]; }
  inline const xp128f& m_50_51_52_53() const { return m_data[10]; } inline xp128f& m_50_51_52_53() { return m_data[10]; }
  inline const xp128f& m_54_55_56_x () const { return m_data[11]; } inline xp128f& m_54_55_56_x () { return m_data[11]; }
  inline const float& m00() const { return m_data[0][0]; } inline float& m00() { return m_data[0][0]; }
  inline const float& m01() const { return m_data[0][1]; } inline float& m01() { return m_data[0][1]; }
  inline const float& m02() const { return m_data[0][2]; } inline float& m02() { return m_data[0][2]; }
  inline const float& m03() const { return m_data[0][3]; } inline float& m03() { return m_data[0][3]; }
  inline const float& m04() const { return m_data[1][0]; } inline float& m04() { return m_data[1][0]; }
  inline const float& m05() const { return m_data[1][1]; } inline float& m05() { return m_data[1][1]; }
  inline const float& m06() const { return m_data[1][2]; } inline float& m06() { return m_data[1][2]; }
  inline const float& m10() const { return m_data[2][0]; } inline float& m10() { return m_data[2][0]; }
  inline const float& m11() const { return m_data[2][1]; } inline float& m11() { return m_data[2][1]; }
  inline const float& m12() const { return m_data[2][2]; } inline float& m12() { return m_data[2][2]; }
  inline const float& m13() const { return m_data[2][3]; } inline float& m13() { return m_data[2][3]; }
  inline const float& m14() const { return m_data[3][0]; } inline float& m14() { return m_data[3][0]; }
  inline const float& m15() const { return m_data[3][1]; } inline float& m15() { return m_data[3][1]; }
  inline const float& m16() const { return m_data[3][2]; } inline float& m16() { return m_data[3][2]; }
  inline const float& m20() const { return m_data[4][0]; } inline float& m20() { return m_data[4][0]; }
  inline const float& m21() const { return m_data[4][1]; } inline float& m21() { return m_data[4][1]; }
  inline const float& m22() const { return m_data[4][2]; } inline float& m22() { return m_data[4][2]; }
  inline const float& m23() const { return m_data[4][3]; } inline float& m23() { return m_data[4][3]; }
  inline const float& m24() const { return m_data[5][0]; } inline float& m24() { return m_data[5][0]; }
  inline const float& m25() const { return m_data[5][1]; } inline float& m25() { return m_data[5][1]; }
  inline const float& m26() const { return m_data[5][2]; } inline float& m26() { return m_data[5][2]; }
  inline const float& m30() const { return m_data[6][0]; } inline float& m30() { return m_data[6][0]; }
  inline const float& m31() const { return m_data[6][1]; } inline float& m31() { return m_data[6][1]; }
  inline const float& m32() const { return m_data[6][2]; } inline float& m32() { return m_data[6][2]; }
  inline const float& m33() const { return m_data[6][3]; } inline float& m33() { return m_data[6][3]; }
  inline const float& m34() const { return m_data[7][0]; } inline float& m34() { return m_data[7][0]; }
  inline const float& m35() const { return m_data[7][1]; } inline float& m35() { return m_data[7][1]; }
  inline const float& m36() const { return m_data[7][2]; } inline float& m36() { return m_data[7][2]; }
  inline const float& m40() const { return m_data[8][0]; } inline float& m40() { return m_data[8][0]; }
  inline const float& m41() const { return m_data[8][1]; } inline float& m41() { return m_data[8][1]; }
  inline const float& m42() const { return m_data[8][2]; } inline float& m42() { return m_data[8][2]; }
  inline const float& m43() const { return m_data[8][3]; } inline float& m43() { return m_data[8][3]; }
  inline const float& m44() const { return m_data[9][0]; } inline float& m44() { return m_data[9][0]; }
  inline const float& m45() const { return m_data[9][1]; } inline float& m45() { return m_data[9][1]; }
  inline const float& m46() const { return m_data[9][2]; } inline float& m46() { return m_data[9][2]; }
  inline const float& m50() const { return m_data[10][0]; }  inline float& m50() { return m_data[10][0]; }
  inline const float& m51() const { return m_data[10][1]; }  inline float& m51() { return m_data[10][1]; }
  inline const float& m52() const { return m_data[10][2]; }  inline float& m52() { return m_data[10][2]; }
  inline const float& m53() const { return m_data[10][3]; }  inline float& m53() { return m_data[10][3]; }
  inline const float& m54() const { return m_data[11][0]; }  inline float& m54() { return m_data[11][0]; }
  inline const float& m55() const { return m_data[11][1]; }  inline float& m55() { return m_data[11][1]; }
  inline const float& m56() const { return m_data[11][2]; }  inline float& m56() { return m_data[11][2]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float& operator() (int row, int col) const {
    return data[row * 7 + col];
  }

  inline float& operator() (int row, int col) {
    return data[row * 7 + col];
  }
  inline void operator += (const AlignedMatrix6x7f &M) {
    m_00_01_02_03() += M.m_00_01_02_03();
    m_04_05_06_x () += M.m_04_05_06_x ();
    m_10_11_12_13() += M.m_10_11_12_13();
    m_14_15_16_x () += M.m_14_15_16_x ();
    m_20_21_22_23() += M.m_20_21_22_23();
    m_24_25_26_x () += M.m_24_25_26_x ();
    m_30_31_32_33() += M.m_30_31_32_33();
    m_34_35_36_x () += M.m_34_35_36_x ();
    m_40_41_42_43() += M.m_40_41_42_43();
    m_44_45_46_x () += M.m_44_45_46_x ();
    m_50_51_52_53() += M.m_50_51_52_53();
    m_54_55_56_x () += M.m_54_55_56_x ();
  }
  inline void operator -= (const AlignedMatrix6x7f &M) {
    m_00_01_02_03() -= M.m_00_01_02_03();
    m_04_05_06_x () -= M.m_04_05_06_x ();
    m_10_11_12_13() -= M.m_10_11_12_13();
    m_14_15_16_x () -= M.m_14_15_16_x ();
    m_20_21_22_23() -= M.m_20_21_22_23();
    m_24_25_26_x () -= M.m_24_25_26_x ();
    m_30_31_32_33() -= M.m_30_31_32_33();
    m_34_35_36_x () -= M.m_34_35_36_x ();
    m_40_41_42_43() -= M.m_40_41_42_43();
    m_44_45_46_x () -= M.m_44_45_46_x ();
    m_50_51_52_53() -= M.m_50_51_52_53();
    m_54_55_56_x () -= M.m_54_55_56_x ();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline AlignedMatrix6x7f operator - (const AlignedMatrix6x7f &B) const {
    AlignedMatrix6x7f AmB;
    AmB.m_00_01_02_03() = m_00_01_02_03() - B.m_00_01_02_03();
    AmB.m_04_05_06_x () = m_04_05_06_x () - B.m_04_05_06_x ();
    AmB.m_10_11_12_13() = m_10_11_12_13() - B.m_10_11_12_13();
    AmB.m_14_15_16_x () = m_14_15_16_x () - B.m_14_15_16_x ();
    AmB.m_20_21_22_23() = m_20_21_22_23() - B.m_20_21_22_23();
    AmB.m_24_25_26_x () = m_24_25_26_x () - B.m_24_25_26_x ();
    AmB.m_30_31_32_33() = m_30_31_32_33() - B.m_30_31_32_33();
    AmB.m_34_35_36_x () = m_34_35_36_x () - B.m_34_35_36_x ();
    AmB.m_40_41_42_43() = m_40_41_42_43() - B.m_40_41_42_43();
    AmB.m_44_45_46_x () = m_44_45_46_x () - B.m_44_45_46_x ();
    AmB.m_50_51_52_53() = m_50_51_52_53() - B.m_50_51_52_53();
    AmB.m_54_55_56_x () = m_54_55_56_x () - B.m_54_55_56_x ();
    return AmB;
  }

  inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix6x7f)); }

  inline void Scale(const float s) {
    xp128f _s; _s.vdup_all_lane(s);
    Scale(_s);
  }
  inline void Scale(const xp128f &s) {
    for (int i = 0; i < 12; ++i)
      m_data[i] *= s;
  }

  inline void SetLowerFromUpper() {
    m10() = m01();
    m20() = m02();  m21() = m12();
    m30() = m03();  m31() = m13();  m32() = m23();
    m40() = m04();  m41() = m14();  m42() = m24();  m43() = m34();
    m50() = m05();  m51() = m15();  m52() = m25();  m53() = m35();  m54() = m45();
  }

  static inline void AddABToSymmetric(const AlignedMatrix3x3f &A0, const AlignedMatrix3x3f &A1,
                                      const AlignedMatrix3x7f &B, AlignedMatrix6x7f &AB) {
    xp128f tmp;
    float t1, t2, t3;
    t1 = A0.m00();
    t2 = A0.m01();
    t3 = A0.m02();
    AB.m_00_01_02_03() += B.m_00_01_02_03() * t1 + B.m_10_11_12_13() * t2 + B.m_20_21_22_23() * t3;

    AB.m_04_05_06_x () += B.m_04_05_06_x () * t1 + B.m_14_15_16_x () * t2 + B.m_24_25_26_x () * t3;
    t1 = A0.m10(); t2 = A0.m11(); t3 = A0.m12();
    AB.m_10_11_12_13() += B.m_00_01_02_03() * t1 + B.m_10_11_12_13() * t2 + B.m_20_21_22_23() * t3;

    AB.m_14_15_16_x () += B.m_04_05_06_x () * t1 + B.m_14_15_16_x () * t2 + B.m_24_25_26_x () * t3;
    tmp.vset_all_lane(B.m02(), B.m12(), B.m22(), 0.0);
    AB.m22() += (A0.m_20_21_22_r2() * tmp).vsum_012();
    tmp.vset_all_lane(B.m03(), B.m13(), B.m23(), 0.0f);

    AB.m23() += (A0.m_20_21_22_r2() * tmp).vsum_012();

    AB.m_24_25_26_x() += B.m_04_05_06_x() * A0.m20() + B.m_14_15_16_x() * A0.m21() + B.m_24_25_26_x() * A0.m22();

    AB.m33() += (A1.m_00_01_02_r0() * tmp).vsum_012();
    AB.m_34_35_36_x() += B.m_04_05_06_x() * A1.m00() + B.m_14_15_16_x() * A1.m01() + B.m_24_25_26_x() * A1.m02();
    AB.m_44_45_46_x() += B.m_04_05_06_x() * A1.m10() + B.m_14_15_16_x() * A1.m11() + B.m_24_25_26_x() * A1.m12();
    tmp.vset_all_lane(B.m05(), B.m15(), B.m25(), 0.0f);
    AB.m55() += (A1.m_20_21_22_r2() * tmp).vsum_012();
    tmp.vset_all_lane(B.m06(), B.m16(), B.m26(), 0.0f);
    AB.m56() = (A1.m_20_21_22_r2() * tmp).vsum_012();
  }
  static inline void ATBToSymmetric(const AlignedMatrix3x6f &A, const AlignedMatrix3x7f &B,
                                    AlignedMatrix6x7f &ATB) {
    xp128f a0, a1, a2;
    a0.vdup_all_lane(A.m00());  a1.vdup_all_lane(A.m10());  a2.vdup_all_lane(A.m20());
    ATB.m_00_01_02_03() = B.m_00_01_02_03() * a0 + B.m_10_11_12_13() * a1 + B.m_20_21_22_23() * a2;
    ATB.m_04_05_06_x () = B.m_04_05_06_x () * a0 + B.m_14_15_16_x () * a1 + B.m_24_25_26_x () * a2;
    a0.vdup_all_lane(A.m01());  a1.vdup_all_lane(A.m11());  a2.vdup_all_lane(A.m21());
    ATB.m_10_11_12_13() = B.m_00_01_02_03() * a0 + B.m_10_11_12_13() * a1 + B.m_20_21_22_23() * a2;
    ATB.m_14_15_16_x () = B.m_04_05_06_x () * a0 + B.m_14_15_16_x () * a1 + B.m_24_25_26_x () * a2;
    ATB.m22() = A.m02() * B.m02() + A.m12() * B.m12() + A.m22() * B.m22();
    ATB.m23() = A.m02() * B.m03() + A.m12() * B.m13() + A.m22() * B.m23();

    ATB.m_24_25_26_x() = B.m_04_05_06_x() * A.m02() + B.m_14_15_16_x() * A.m12() + B.m_24_25_26_x() * A.m22();
    ATB.m33() = A.m03() * B.m03() + A.m13() * B.m13() + A.m23() * B.m23();
    ATB.m_34_35_36_x() = B.m_04_05_06_x() * A.m03() + B.m_14_15_16_x() * A.m13() + B.m_24_25_26_x() * A.m23();
    ATB.m_44_45_46_x() = B.m_04_05_06_x() * A.m04() + B.m_14_15_16_x() * A.m14() + B.m_24_25_26_x() * A.m24();
    ATB.m55() = A.m05() * B.m05() + A.m15() * B.m15() + A.m25() * B.m25();
    ATB.m56() = A.m05() * B.m06() + A.m15() * B.m16() + A.m25() * B.m26();
  }
  static inline void AddATBToSymmetric(const AlignedMatrix3x6f &A, const AlignedMatrix3x7f &B,
                                       AlignedMatrix6x7f &ATB) {
    xp128f a0, a1, a2;
    a0.vdup_all_lane(A.m00());  a1.vdup_all_lane(A.m10());  a2.vdup_all_lane(A.m20());
    ATB.m_00_01_02_03() += B.m_00_01_02_03() * a0 + B.m_10_11_12_13() * a1 + B.m_20_21_22_23() * a2;
    ATB.m_04_05_06_x () += B.m_04_05_06_x () * a0 + B.m_14_15_16_x () * a1 + B.m_24_25_26_x () * a2;
    a0.vdup_all_lane(A.m01());  a1.vdup_all_lane(A.m11());  a2.vdup_all_lane(A.m21());
    ATB.m_10_11_12_13() += B.m_00_01_02_03() * a0 + B.m_10_11_12_13() * a1 + B.m_20_21_22_23() * a2;
    ATB.m_14_15_16_x () += B.m_04_05_06_x () * a0 + B.m_14_15_16_x () * a1 + B.m_24_25_26_x () * a2;
    ATB.m22() += A.m02() * B.m02() + A.m12() * B.m12() + A.m22() * B.m22();
    ATB.m23() += A.m02() * B.m03() + A.m12() * B.m13() + A.m22() * B.m23();
    ATB.m_24_25_26_x() += B.m_04_05_06_x() * A.m02() + B.m_14_15_16_x() * A.m12() + B.m_24_25_26_x() * A.m22();
    ATB.m33() += A.m03() * B.m03() + A.m13() * B.m13() + A.m23() * B.m23();
    ATB.m_34_35_36_x() += B.m_04_05_06_x() * A.m03() + B.m_14_15_16_x() * A.m13() + B.m_24_25_26_x() * A.m23();
    ATB.m_44_45_46_x() += B.m_04_05_06_x() * A.m04() + B.m_14_15_16_x() * A.m14() + B.m_24_25_26_x() * A.m24();
    ATB.m55() += A.m05() * B.m05() + A.m15() * B.m15() + A.m25() * B.m25();
    ATB.m56() += A.m05() * B.m06() + A.m15() * B.m16() + A.m25() * B.m26();
  }
  static inline void AddABTToSymmetric00(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B0,
                                         const AlignedVector3f &b1, AlignedMatrix6x7f &ABT) {
    ABT.m00() += (A.m_00_01_02_r0() * B0.m_00_01_02_r0()).vsum_012();
    ABT.m01() += (A.m_00_01_02_r0() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m02() += (A.m_00_01_02_r0() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m06() += (A.m_00_01_02_r0() * b1.v012r()).vsum_012();
    ABT.m11() += (A.m_10_11_12_r1() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m12() += (A.m_10_11_12_r1() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m16() += (A.m_10_11_12_r1() * b1.v012r()).vsum_012();
    ABT.m22() += (A.m_20_21_22_r2() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m26() += (A.m_20_21_22_r2() * b1.v012r()).vsum_012();
  }
  static inline void AddABTToSymmetric33(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B0,
                                         const AlignedVector3f &b1, AlignedMatrix6x7f &ABT) {
    ABT.m33() += (A.m_00_01_02_r0() * B0.m_00_01_02_r0()).vsum_012();
    ABT.m34() += (A.m_00_01_02_r0() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m35() += (A.m_00_01_02_r0() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m36() += (A.m_00_01_02_r0() * b1.v012r()).vsum_012();
    ABT.m44() += (A.m_10_11_12_r1() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m45() += (A.m_10_11_12_r1() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m46() += (A.m_10_11_12_r1() * b1.v012r()).vsum_012();
    ABT.m55() += (A.m_20_21_22_r2() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m56() += (A.m_20_21_22_r2() * b1.v012r()).vsum_012();
  }
  static inline void SubtractABTFromSymmetric(const AlignedMatrix3x3f &A, const AlignedMatrix3x3f &B0,
                                              const AlignedMatrix3x3f &B1, const AlignedVector3f &b2, AlignedMatrix6x7f &ABT) {
    ABT.m00() -= (A.m_00_01_02_r0() * B0.m_00_01_02_r0()).vsum_012();
    ABT.m01() -= (A.m_00_01_02_r0() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m02() -= (A.m_00_01_02_r0() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m03() -= (A.m_00_01_02_r0() * B1.m_00_01_02_r0()).vsum_012();
    ABT.m04() -= (A.m_00_01_02_r0() * B1.m_10_11_12_r1()).vsum_012();
    ABT.m05() -= (A.m_00_01_02_r0() * B1.m_20_21_22_r2()).vsum_012();
    ABT.m06() -= (A.m_00_01_02_r0() * b2.v012r()).vsum_012();

    ABT.m11() -= (A.m_10_11_12_r1() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m12() -= (A.m_10_11_12_r1() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m13() -= (A.m_10_11_12_r1() * B1.m_00_01_02_r0()).vsum_012();
    ABT.m14() -= (A.m_10_11_12_r1() * B1.m_10_11_12_r1()).vsum_012();
    ABT.m15() -= (A.m_10_11_12_r1() * B1.m_20_21_22_r2()).vsum_012();
    ABT.m16() -= (A.m_10_11_12_r1() * b2.v012r()).vsum_012();

    ABT.m22() -= (A.m_20_21_22_r2() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m23() -= (A.m_20_21_22_r2() * B1.m_00_01_02_r0()).vsum_012();
    ABT.m24() -= (A.m_20_21_22_r2() * B1.m_10_11_12_r1()).vsum_012();
    ABT.m25() -= (A.m_20_21_22_r2() * B1.m_20_21_22_r2()).vsum_012();
    ABT.m26() -= (A.m_20_21_22_r2() * b2.v012r()).vsum_012();
  }
  static inline void SubtractABTFromSymmetric33(const AlignedMatrix3x3f &A,
                                                const AlignedMatrix3x3f &B0, const AlignedVector3f &b1, AlignedMatrix6x7f &ABT) {
    ABT.m33() -= (A.m_00_01_02_r0() * B0.m_00_01_02_r0()).vsum_012();
    ABT.m34() -= (A.m_00_01_02_r0() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m35() -= (A.m_00_01_02_r0() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m36() -= (A.m_00_01_02_r0() * b1.v012r()).vsum_012();
    ABT.m44() -= (A.m_10_11_12_r1() * B0.m_10_11_12_r1()).vsum_012();
    ABT.m45() -= (A.m_10_11_12_r1() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m46() -= (A.m_10_11_12_r1() * b1.v012r()).vsum_012();
    ABT.m55() -= (A.m_20_21_22_r2() * B0.m_20_21_22_r2()).vsum_012();
    ABT.m56() -= (A.m_20_21_22_r2() * b1.v012r()).vsum_012();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(), m05(), m06());
      UT::Print("%e %e %e %e %e %e %e\n", m10(), m11(), m12(), m13(), m14(), m15(), m16());
      UT::Print("%e %e %e %e %e %e %e\n", m20(), m21(), m22(), m23(), m24(), m25(), m26());
      UT::Print("%e %e %e %e %e %e %e\n", m30(), m31(), m32(), m33(), m34(), m35(), m36());
      UT::Print("%e %e %e %e %e %e %e\n", m40(), m41(), m42(), m43(), m44(), m45(), m46());
      UT::Print("%e %e %e %e %e %e %e\n", m50(), m51(), m52(), m53(), m54(), m55(), m56());
    } else {
      UT::Print("%f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(), m05(), m06());
      UT::Print("%f %f %f %f %f %f %f\n", m10(), m11(), m12(), m13(), m14(), m15(), m16());
      UT::Print("%f %f %f %f %f %f %f\n", m20(), m21(), m22(), m23(), m24(), m25(), m26());
      UT::Print("%f %f %f %f %f %f %f\n", m30(), m31(), m32(), m33(), m34(), m35(), m36());
      UT::Print("%f %f %f %f %f %f %f\n", m40(), m41(), m42(), m43(), m44(), m45(), m46());
      UT::Print("%f %f %f %f %f %f %f\n", m50(), m51(), m52(), m53(), m54(), m55(), m56());
    }
  }
  inline bool AssertEqual(const AlignedMatrix6x7f &M,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&m00(), &M.m00(), 7, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m10(), &M.m10(), 7, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m20(), &M.m20(), 7, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m30(), &M.m30(), 7, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m40(), &M.m40(), 7, verbose, str, epsAbs, epsRel) &&
        UT::VectorAssertEqual(&m50(), &M.m50(), 7, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      M.Print(verbose > 1);
      const AlignedMatrix6x7f E = *this - M;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }

 protected:
  union {
    xp128f m_data[12];
    float data[12 * 4];
  };
};

template<typename TYPE> class SymmetricMatrix6x7 {

 public:

  inline SymmetricMatrix6x7<TYPE>() {}
  inline SymmetricMatrix6x7<TYPE>(const TYPE *M) { Set(M); }

  inline const TYPE& m00() const { return m_data[0]; }  inline TYPE& m00() { return m_data[0]; }
  inline const TYPE& m01() const { return m_data[1]; }  inline TYPE& m01() { return m_data[1]; }
  inline const TYPE& m02() const { return m_data[2]; }  inline TYPE& m02() { return m_data[2]; }
  inline const TYPE& m03() const { return m_data[3]; }  inline TYPE& m03() { return m_data[3]; }
  inline const TYPE& m04() const { return m_data[4]; }  inline TYPE& m04() { return m_data[4]; }
  inline const TYPE& m05() const { return m_data[5]; }  inline TYPE& m05() { return m_data[5]; }
  inline const TYPE& m06() const { return m_data[6]; }  inline TYPE& m06() { return m_data[6]; }
  inline const TYPE& m11() const { return m_data[7]; }  inline TYPE& m11() { return m_data[7]; }
  inline const TYPE& m12() const { return m_data[8]; }  inline TYPE& m12() { return m_data[8]; }
  inline const TYPE& m13() const { return m_data[9]; }  inline TYPE& m13() { return m_data[9]; }
  inline const TYPE& m14() const { return m_data[10]; } inline TYPE& m14() { return m_data[10]; }
  inline const TYPE& m15() const { return m_data[11]; } inline TYPE& m15() { return m_data[11]; }
  inline const TYPE& m16() const { return m_data[12]; } inline TYPE& m16() { return m_data[12]; }
  inline const TYPE& m22() const { return m_data[13]; } inline TYPE& m22() { return m_data[13]; }
  inline const TYPE& m23() const { return m_data[14]; } inline TYPE& m23() { return m_data[14]; }
  inline const TYPE& m24() const { return m_data[15]; } inline TYPE& m24() { return m_data[15]; }
  inline const TYPE& m25() const { return m_data[16]; } inline TYPE& m25() { return m_data[16]; }
  inline const TYPE& m26() const { return m_data[17]; } inline TYPE& m26() { return m_data[17]; }
  inline const TYPE& m33() const { return m_data[18]; } inline TYPE& m33() { return m_data[18]; }
  inline const TYPE& m34() const { return m_data[19]; } inline TYPE& m34() { return m_data[19]; }
  inline const TYPE& m35() const { return m_data[20]; } inline TYPE& m35() { return m_data[20]; }
  inline const TYPE& m36() const { return m_data[21]; } inline TYPE& m36() { return m_data[21]; }
  inline const TYPE& m44() const { return m_data[22]; } inline TYPE& m44() { return m_data[22]; }
  inline const TYPE& m45() const { return m_data[23]; } inline TYPE& m45() { return m_data[23]; }
  inline const TYPE& m46() const { return m_data[24]; } inline TYPE& m46() { return m_data[24]; }
  inline const TYPE& m55() const { return m_data[25]; } inline TYPE& m55() { return m_data[25]; }
  inline const TYPE& m56() const { return m_data[26]; } inline TYPE& m56() { return m_data[26]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  inline void operator *= (const xp128f &s);
  inline void operator - (const SymmetricMatrix6x7<TYPE> &B) const {
    SymmetricMatrix6x7<TYPE> AmB;
    for (int i = 0; i < 27; ++i)
      AmB.m_data[i] = m_data[i] - B.m_data[i];
    return AmB;
  }

  inline void Set(const TYPE *M) { memcpy(this, M, sizeof(SymmetricMatrix6x6<TYPE>)); }

  inline AlignedMatrix6x6f GetAlignedMatrix6x6f() const;

  inline void MakeZero() { memset(this, 0, sizeof(SymmetricMatrix6x6<TYPE>)); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(), m05(), m06());
      UT::Print("%e %e %e %e %e %e %e\n", m01(), m11(), m12(), m13(), m14(), m15(), m16());
      UT::Print("%e %e %e %e %e %e %e\n", m02(), m12(), m22(), m23(), m24(), m25(), m26());
      UT::Print("%e %e %e %e %e %e %e\n", m03(), m13(), m23(), m33(), m34(), m35(), m36());
      UT::Print("%e %e %e %e %e %e %e\n", m04(), m14(), m24(), m34(), m44(), m45(), m46());
      UT::Print("%e %e %e %e %e %e %e\n", m05(), m15(), m25(), m35(), m45(), m55(), m56());
    } else {
      UT::Print("%f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(), m05(), m06());
      UT::Print("%f %f %f %f %f %f %f\n", m01(), m11(), m12(), m13(), m14(), m15(), m16());
      UT::Print("%f %f %f %f %f %f %f\n", m02(), m12(), m22(), m23(), m24(), m25(), m26());
      UT::Print("%f %f %f %f %f %f %f\n", m03(), m13(), m23(), m33(), m34(), m35(), m36());
      UT::Print("%f %f %f %f %f %f %f\n", m04(), m14(), m24(), m34(), m44(), m45(), m46());
      UT::Print("%f %f %f %f %f %f %f\n", m05(), m15(), m25(), m35(), m45(), m55(), m56());
    }
  }

  inline bool AssertEqual(const SymmetricMatrix6x7<TYPE> &A,
                          const int verbose = 1, const std::string str = "",
                          const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertEqual<TYPE>(m_data, A.m_data, 27, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      UT::PrintSeparator();
      A.Print(verbose > 1);
      const SymmetricMatrix6x7<TYPE> E = *this - A;
      UT::PrintSeparator();
      E.Print(verbose > 1);
    }
    return false;
  }

 protected:

  TYPE m_data[27];
};

typedef SymmetricMatrix6x7<float> SymmetricMatrix6x7f;
typedef SymmetricMatrix6x7<double> SymmetricMatrix6x7d;

}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix6x7f : public Eigen::Matrix<float, 6, 7> {
 public:
  inline EigenMatrix6x7f() : Eigen::Matrix<float, 6, 7>() {}
  inline EigenMatrix6x7f(const Eigen::Matrix<float, 6, 7> &e_M) : Eigen::Matrix<float, 6, 7>(e_M) {}
  inline EigenMatrix6x7f(const LA::AlignedMatrix6x7f &M) : Eigen::Matrix<float, 6, 7>() {
    const float* _M[6] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(), &M.m40(), &M.m50()};
    Eigen::Matrix<float, 6, 7> &e_M = *this;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j)
        e_M(i, j) = _M[i][j];
  }
  inline EigenMatrix6x7f(const EigenMatrix3x3f &e_M00, const EigenMatrix3x3f &e_M01,
                         const EigenVector3f &e_m02,
                         const EigenMatrix3x3f &e_M10, const EigenMatrix3x3f &e_M11, const EigenVector3f &e_m12) {
    block<3, 3>(0, 0) = e_M00;  block<3, 3>(0, 3) = e_M01;  block<3, 1>(0, 6) = e_m02;
    block<3, 3>(3, 0) = e_M10;  block<3, 3>(3, 3) = e_M11;  block<3, 1>(3, 6) = e_m12;
  }
  inline void operator = (const Eigen::Matrix<float, 6, 7> &e_M) { *((Eigen::Matrix<float, 6, 7> *) this) = e_M; }
  inline LA::AlignedMatrix6x7f GetAlignedMatrix6x7f() const {
    LA::AlignedMatrix6x7f M;
    float* _M[6] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(), &M.m40(), &M.m50()};
    const Eigen::Matrix<float, 6, 7> &e_M = *this;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 7; ++j)
        _M[i][j] = e_M(i, j);
    return M;
  }
  inline void Print(const bool e = false) const { GetAlignedMatrix6x7f().Print(e); }
  inline bool AssertEqual(const LA::AlignedMatrix6x7f &M, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedMatrix6x7f().AssertEqual(M, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenMatrix6x7f &e_M, const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_M.GetAlignedMatrix6x7f(), verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
