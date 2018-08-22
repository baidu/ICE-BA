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
#ifndef _VECTOR_9_H_
#define _VECTOR_9_H_

#include "Utility.h"
#include "Matrix3x3.h"

namespace LA {

class AlignedVector9f {

 public:

  inline const xp128f &v0123() const { return m_data4[0]; }      inline xp128f &v0123() { return m_data4[0]; }
  inline const xp128f &v4567() const { return m_data4[1]; }      inline xp128f &v4567() { return m_data4[1]; }
  inline const float& v0() const { return m_data[0]; }      inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }      inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }      inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }      inline float& v3() { return m_data[3]; }
  inline const float& v4() const { return m_data[4]; }      inline float& v4() { return m_data[4]; }
  inline const float& v5() const { return m_data[5]; }      inline float& v5() { return m_data[5]; }
  inline const float& v6() const { return m_data[6]; }      inline float& v6() { return m_data[6]; }
  inline const float& v7() const { return m_data[7]; }      inline float& v7() { return m_data[7]; }
  inline const float& v8() const { return m_data[8]; }      inline float& v8() { return m_data[8]; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }
  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }
  inline void operator += (const AlignedVector9f &v) {
    v0123() += v.v0123();
    v4567() += v.v4567();
    v8() += v.v8();
  }
  inline void operator -= (const AlignedVector9f &v) {
    v0123() -= v.v0123();
    v4567() -= v.v4567();
    v8() -= v.v8();
  }
  inline void operator *= (const float s) {
    v0123() *= s;
    v4567() *= s;
    v8() *= s;
  }
  // TODO (yanghongtian) : remove this interfaces.
  inline void operator *= (const xp128f &s) {
    v0123() *= s;
    v4567() *= s;
    v8() *= s[0];
  }
  inline AlignedVector9f operator - (const AlignedVector9f &b) const {
    AlignedVector9f amb;
    AlignedVector9f::amb(*this, b, amb);
    return amb;
  }

  inline AlignedVector9f operator + (const AlignedVector9f& b) const {
    AlignedVector9f apb;
    AlignedVector9f::apb(*this, b, apb);
    return apb;
  }

  inline AlignedVector9f operator * (const xp128f &s) const {
    AlignedVector9f v; GetScaled(s, v); return v;
  }

  inline void Set(const float *v) {
    memcpy(this, v, 36);
  }
  inline void Set(const double *v) {
    for (int i = 0; i < 9; ++i) {
      m_data[i] = static_cast<float>(v[i]);
    }
  }
  inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1,
                  const AlignedVector3f &v2) {
    Set012(v0);
    Set345(v1);
    Set678(v2);
  }
  inline void Set012(const float *v) { memcpy(&v0(), v, 12); }
  inline void Set345(const float *v) { memcpy(&v3(), v, 12); }
  inline void Set678(const float *v) { memcpy(&v6(), v, 12); }
  inline void Get(float *v) const {
    memcpy(v, this, 36);
  }
  inline void Get(Vector3f &v0, Vector3f &v1, Vector3f &v2) const {
    Get012(v0);
    Get345(v1);
    Get678(v2);
  }
  inline void Get(double *v) const {
    for (int i = 0; i < 9; ++i) {
      v[i] = static_cast<double>(m_data[i]);
    }
  }
  inline void Get012(Vector3f &v) const { v.Set(&v0()); }
  inline void Get345(Vector3f &v) const { v.Set(&v3()); }
  inline void Get678(Vector3f &v) const { v.Set(&v6()); }
  inline void Get(AlignedVector3f &v0, AlignedVector3f &v1, AlignedVector3f &v2) const {
    Get012(v0);
    Get345(v1);
    Get678(v2);
  }
  inline void Get012(AlignedVector3f &v) const { v.Set(&v0()); }
  inline void Get345(AlignedVector3f &v) const { v.Set(&v3()); }
  inline void Get678(AlignedVector3f &v) const { v.Set(&v6()); }

  inline bool Valid() const { return v0() != FLT_MAX; }
  inline bool Invalid() const { return v0() == FLT_MAX; }
  inline void Invalidate() { v0() = FLT_MAX; }

  inline void MakeZero() {
    memset(this, 0, sizeof(AlignedVector9f));
  }
  inline void MakeMinus() {
    v0123().vmake_minus();
    v4567().vmake_minus();
    v8() = -v8();
  }
  inline void GetMinus(AlignedVector9f &v) const {
    const xp128f zero = xp128f::get(0.0f);
    v.v0123() = zero - v0123();
    v.v4567() = zero - v4567();
    v.v8() = -v8();
  }
  // TODO (yanghongtian) : remove this two interface.
  inline void Scale(const float s) {
    xp128f _s; _s.vdup_all_lane(s);
    Scale(_s);
  }
  inline void Scale(const xp128f &s) {
    v0123() *= s;
    v4567() *= s;
    v8() *= s[0];
  }
  inline void GetScaled(const float s, AlignedVector9f &v) const {
    const xp128f _s = xp128f::get(s);
    GetScaled(_s, v);
  }
  inline void GetScaled(const xp128f &s, AlignedVector9f &v) const {
    v.v0123() = v0123() * s;
    v.v4567() = v4567() * s;
    v.v8() = v8() * s[0];
  }
  inline float SquaredLength() const {
    return (v0123() * v0123() + v4567() * v4567()).vsum_all() + v8() * v8();
  }
  inline float Dot(const AlignedVector9f &v) const {
    return (v0123() * v.v0123() + v4567() * v.v4567()).vsum_all() + v8() * v.v8();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7(), v8());
    } else {
      UT::Print("%f %f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7(), v8());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    Print(e);
  }

  inline void Random(const float vMax) { UT::Random(&v0(), 9, -vMax, vMax); }
  static inline AlignedVector9f GetRandom(const float vMax) {
    AlignedVector9f v;
    v.Random(vMax);
    return v;
  }

  inline bool AssertEqual(const LA::AlignedVector9f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 9, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const LA::AlignedVector9f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 9, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  static inline void apb(const AlignedVector9f &a, const AlignedVector9f &b, AlignedVector9f &apb) {
    apb.v0123() = a.v0123() + b.v0123();
    apb.v4567() = a.v4567() + b.v4567();
    apb.v8() = a.v8() + b.v8();
  }
  static inline void amb(const AlignedVector9f &a, const AlignedVector9f &b, AlignedVector9f &amb) {
    amb.v0123() = a.v0123() - b.v0123();
    amb.v4567() = a.v4567() - b.v4567();
    amb.v8() = a.v8() - b.v8();
  }

 protected:
  union {
    float m_data[9];
    xp128f m_data4[3];
  };
};

template<typename TYPE> class Vector9 {

 public:

  //inline Vector9() {}
  //inline Vector9(const TYPE *v) { Set(v); }

  static inline Vector9<TYPE> Get(const float *v) { Vector9<TYPE> _v; _v.Set(v); return _v; }

  inline const TYPE& v0() const { return m_data[0]; }   inline TYPE& v0() { return m_data[0]; }
  inline const TYPE& v1() const { return m_data[1]; }   inline TYPE& v1() { return m_data[1]; }
  inline const TYPE& v2() const { return m_data[2]; }   inline TYPE& v2() { return m_data[2]; }
  inline const TYPE& v3() const { return m_data[3]; }   inline TYPE& v3() { return m_data[3]; }
  inline const TYPE& v4() const { return m_data[4]; }   inline TYPE& v4() { return m_data[4]; }
  inline const TYPE& v5() const { return m_data[5]; }   inline TYPE& v5() { return m_data[5]; }
  inline const TYPE& v6() const { return m_data[6]; }   inline TYPE& v6() { return m_data[6]; }
  inline const TYPE& v7() const { return m_data[7]; }   inline TYPE& v7() { return m_data[7]; }
  inline const TYPE& v8() const { return m_data[8]; }   inline TYPE& v8() { return m_data[8]; }

  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }
  //inline void operator = (const TYPE *v) { Set(v); }
  inline Vector9<TYPE> operator * (const TYPE s) const { Vector9<TYPE> v; GetScaled(s, v); return v; }
  inline Vector9<TYPE> operator * (const xp128f &s) const { Vector9<TYPE> v; GetScaled(TYPE(s[0]), v); return v; }

  inline void operator += (const Vector9<TYPE> &v) {
    v0() += v.v0();
    v1() += v.v1();
    v2() += v.v2();
    v3() += v.v3();
    v4() += v.v4();
    v5() += v.v5();
    v6() += v.v6();
    v7() += v.v7();
    v8() += v.v8();
  }
  inline Vector9<TYPE> operator - (const Vector9<TYPE> &b) const {
    Vector9<TYPE> amb;
    amb.v0() = v0() - b.v0();
    amb.v1() = v1() - b.v1();
    amb.v2() = v2() - b.v2();
    amb.v3() = v3() - b.v3();
    amb.v4() = v4() - b.v4();
    amb.v5() = v5() - b.v5();
    amb.v6() = v6() - b.v6();
    amb.v7() = v7() - b.v7();
    amb.v8() = v8() - b.v8();
    return amb;
  }

  inline void Set(const float v) {
    v0() = v; v1() = v; v2() = v; v3() = v; v4() = v;
    v5() = v; v6() = v; v7() = v; v8() = v;
  }
  inline void Set(const float *v);
  inline void Set(const double *v);
  inline void Set(const TYPE *v0, const TYPE *v1, const TYPE *v2) {
    Set012(v0);
    Set345(v1);
    Set678(v2);
  }
  inline void Set012(const float *v);
  inline void Set345(const float *v);
  inline void Set678(const float *v);
  inline void Get(TYPE *v) const { memcpy(v, this, sizeof(Vector9<TYPE>)); }
  inline void Get(AlignedVector3f &v0, AlignedVector3f &v1, AlignedVector3f &v2) const {
    Get012(v0);
    Get345(v1);
    Get678(v2);
  }
  inline void Get012(float *v) const;
  inline void Get345(float *v) const;
  inline void Get678(float *v) const;
  inline Vector3<TYPE> Get012() const { Vector3<TYPE> v; Get012(v); return v; }
  inline Vector3<TYPE> Get345() const { Vector3<TYPE> v; Get345(v); return v; }
  inline Vector3<TYPE> Get678() const { Vector3<TYPE> v; Get678(v); return v; }

  inline void Increase(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i <= 6);
#endif
    m_data[i] += v.v0();
    m_data[i + 1] += v.v1();
    m_data[i + 2] += v.v2();
  }
  inline void Decrease(const int i, const AlignedVector3f &v) {
#ifdef CFG_DEBUG
    UT_ASSERT(i >= 0 && i <= 6);
#endif
    m_data[i] -= v.v0();
    m_data[i + 1] -= v.v1();
    m_data[i + 2] -= v.v2();
  }

  inline void MakeZero() { memset(this, 0, sizeof(Vector9<TYPE>)); }
  inline void MakeZero012() { memset(&v0(), 0, sizeof(TYPE) * 3); }
  inline void MakeZero345() { memset(&v3(), 0, sizeof(TYPE) * 3); }
  inline void MakeZero678() { memset(&v6(), 0, sizeof(TYPE) * 3); }

  inline void MakeMinus() {
    for (int i = 0; i < 9; ++i) {
      m_data[i] = -m_data[i];
    }
  }

  inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }

  inline void GetScaled(const TYPE s, Vector9<TYPE> &v) const {
    for (int i = 0; i < 9; ++i) {
      v[i] = s * m_data[i];
    }
  }

  inline TYPE Sum() const {
    return v0() + v1() + v2() + v3() + v4() + v5() + v6() + v7() + v8();
  }
  // TODO (yanghongtian) : rooms for optimizations
  inline TYPE Maximal() const {
    return std::max(std::max(std::max(std::max(v0(), v1()),
                                      std::max(v2(), v3())),
                             std::max(std::max(v4(), v5()),
                                      std::max(v6(), v7()))),
                                               v8());
  }
  inline TYPE SquaredLength() const {
    return v0() * v0() + v1() * v1() + v2() * v2() +
           v3() * v3() + v4() * v4() + v5() * v5() +
           v6() * v6() + v7() * v7() + v8() * v8();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7(), v8());
    } else {
      UT::Print("%f %f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5(), v6(), v7(), v8());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    Print(e);
  }

  inline bool AssertEqual(const Vector9<TYPE> &v,
                          const int verbose = 1, const std::string str = "",
                          const TYPE epsAbs = 0, const TYPE epsRel = 0) const {
    if (UT::VectorAssertEqual<TYPE>(&v0(), &v.v0(), 9, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const Vector9<TYPE> e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }

  inline void Random(const TYPE vMin, const TYPE vMax) { UT::Random(&v0(), 9, vMin, vMax); }
  static inline Vector9<TYPE> GetRandom(const TYPE vMin, const TYPE vMax) {
    Vector9<TYPE> v;
    v.Random(vMin, vMax);
    return v;
  }

  static inline void AbTo0(const AlignedMatrix3x3f &A, const AlignedVector3f &b, Vector9<TYPE> &Ab) {
    Ab.v0() = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v1() = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v2() = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo0(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                              Vector9<TYPE> &Ab) {
    Ab.v0() += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v1() += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v2() += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AbTo3(const AlignedMatrix3x3f &A, const AlignedVector3f &b, Vector9<TYPE> &Ab) {
    Ab.v3() = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v4() = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v5() = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo3(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                              Vector9<TYPE> &Ab) {
    Ab.v3() += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v4() += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v5() += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AbTo6(const AlignedMatrix3x3f &A, const AlignedVector3f &b, Vector9<TYPE> &Ab) {
    Ab.v6() = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v7() = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v8() = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo6(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                              Vector9<TYPE> &Ab) {
    Ab.v6() += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v7() += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v8() += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }

 protected:

  TYPE m_data[9];
};

typedef Vector9<float> Vector9f;
typedef Vector9<double> Vector9d;

template<> inline void Vector9f::Set(const float *v) { memcpy(this, v, sizeof(Vector9f)); }
template<> inline void Vector9d::Set(const float *v) {
  for (int i = 0; i < 9; ++i) {
    m_data[i] = static_cast<double>(v[i]);
  }
}
template<> inline void Vector9f::Set012(const float *v) { memcpy(&v0(), v, 12); }
template<> inline void Vector9f::Set345(const float *v) { memcpy(&v3(), v, 12); }
template<> inline void Vector9f::Set678(const float *v) { memcpy(&v6(), v, 12); }
template<> inline void Vector9d::Set012(const float *v) {
  v0() = static_cast<double>(v[0]);
  v1() = static_cast<double>(v[1]);
  v2() = static_cast<double>(v[2]);
}
template<> inline void Vector9d::Set345(const float *v) {
  v3() = static_cast<double>(v[0]);
  v4() = static_cast<double>(v[1]);
  v5() = static_cast<double>(v[2]);
}
template<> inline void Vector9d::Set678(const float *v) {
  v6() = static_cast<double>(v[0]);
  v7() = static_cast<double>(v[1]);
  v8() = static_cast<double>(v[2]);
}
template<> inline void Vector9f::Get012(float *v) const { memcpy(v, &v0(), 12); }
template<> inline void Vector9f::Get345(float *v) const { memcpy(v, &v3(), 12); }
template<> inline void Vector9f::Get678(float *v) const { memcpy(v, &v6(), 12); }
template<> inline void Vector9d::Get012(float *v) const {
  v[0] = static_cast<float>(v0());
  v[1] = static_cast<float>(v1());
  v[2] = static_cast<float>(v2());
}
template<> inline void Vector9d::Get345(float *v) const {
  v[0] = static_cast<float>(v3());
  v[1] = static_cast<float>(v4());
  v[2] = static_cast<float>(v5());
}
template<> inline void Vector9d::Get678(float *v) const {
  v[0] = static_cast<float>(v6());
  v[1] = static_cast<float>(v7());
  v[2] = static_cast<float>(v8());
}

}  // namespace LA

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector9f : public Eigen::Matrix<float, 9, 1> {
 public:
  inline EigenVector9f() : Eigen::Matrix<float, 9, 1>() {}
  inline EigenVector9f(const Eigen::Matrix<float, 9, 1> &e_v) : Eigen::Matrix<float, 9, 1>(e_v) {}
  inline EigenVector9f(const float *v) : Eigen::Matrix<float, 9, 1>() { *this = v; }
  inline EigenVector9f(const LA::AlignedVector3f &v0, const LA::AlignedVector3f &v1,
                       const LA::AlignedVector3f &v2) {
    block<3, 1>(0, 0) = EigenVector3f(v0);
    block<3, 1>(3, 0) = EigenVector3f(v1);
    block<3, 1>(6, 0) = EigenVector3f(v2);
  }
  inline void operator = (const Eigen::Matrix<float, 9, 1> &e_v) {
    *((Eigen::Matrix<float, 9, 1> *) this) = e_v;
  }
  inline void operator = (const float *v) {
    Eigen::Matrix<float, 9, 1> &e_v = *this;
    for (int i = 0; i < 9; ++i) {
      e_v(i, 0) = v[i];
    }
  }
  inline LA::AlignedVector9f GetAlignedVector9f() const {
    LA::AlignedVector9f v;
    float* _v = v;
    const Eigen::Matrix<float, 9, 1> &e_v = *this;
    for (int i = 0; i < 9; ++i) {
      _v[i] = e_v(i, 0);
    }
    return v;
  }
  inline LA::Vector9f GetVector9f() const {
    LA::Vector9f v;
    float* _v = v;
    const Eigen::Matrix<float, 9, 1> &e_v = *this;
    for (int i = 0; i < 9; ++i) {
      _v[i] = e_v(i, 0);
    }
    return v;
  }
  inline float SquaredLength() const { return GetAlignedVector9f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector9f().Print(e); }
  static inline EigenVector9f GetRandom(const float vMax) { return EigenVector9f(LA::AlignedVector9f::GetRandom(vMax)); }
  inline bool AssertEqual(const LA::AlignedVector9f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector9f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::Vector9f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetVector9f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector9f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector9f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector9f().AssertZero(verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif
