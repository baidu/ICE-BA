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
#ifndef LINEARALGEBRA_VECTOR6_H_
#define LINEARALGEBRA_VECTOR6_H_

#include "Matrix3x3.h"
#include <string>
#include <algorithm>

namespace LA {

class AlignedVector6f {
 public:
  static inline AlignedVector6f Get(const float *v) { AlignedVector6f _v; _v.Set(v); return _v; }

  inline operator const float* () const { return (const float *) this; }
  inline operator       float* ()       { return (      float *) this; }

  inline const xp128f &v0123() const { return m_data4[0]; } inline xp128f &v0123() { return m_data4[0]; }
  inline const xp128f &v45xx() const { return m_data4[1]; } inline xp128f &v45xx() { return m_data4[1]; }
  inline const float& v0() const { return m_data[0]; }      inline float& v0() { return m_data[0]; }
  inline const float& v1() const { return m_data[1]; }      inline float& v1() { return m_data[1]; }
  inline const float& v2() const { return m_data[2]; }      inline float& v2() { return m_data[2]; }
  inline const float& v3() const { return m_data[3]; }      inline float& v3() { return m_data[3]; }
  inline const float& v4() const { return m_data[4]; }      inline float& v4() { return m_data[4]; }
  inline const float& v5() const { return m_data[5]; }      inline float& v5() { return m_data[5]; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }
  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  inline void operator += (const AlignedVector6f &v) {
    v0123() += v.v0123();
    v4() += v.v4();
    v5() += v.v5();
  }
  inline void operator *= (const float s) { Scale(s); }
  inline void operator *= (const xp128f &s) { Scale(s); }
  inline AlignedVector6f operator - (const AlignedVector6f &b) const {
    AlignedVector6f amb;
    amb.v0123() = v0123() - b.v0123();
    amb.v4() = v4() - b.v4();
    amb.v5() = v5() - b.v5();
    return amb;
  }
  inline AlignedVector6f operator * (const float s) const {
    AlignedVector6f v;
    GetScaled(s, v);
    return v;
  }
  inline AlignedVector6f operator * (const xp128f &s) const {
    AlignedVector6f v;
    GetScaled(s, v);
    return v;
  }
  inline bool operator == (const AlignedVector6f &v) const {
    return v0() == v.v0() && v1() == v.v1() && v2() == v.v2() &&
           v3() == v.v3() && v4() == v.v4() && v5() == v.v5();
  }

  inline void Set(const float *v) { memcpy(this, v, 24); }
  inline void Set(const double *v) {
    v0() = float(v[0]);
    v1() = float(v[1]);
    v2() = float(v[2]);
    v3() = float(v[3]);
    v4() = float(v[4]);
    v5() = float(v[5]);
  }
  inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1) {
    v0123() = v0.v012r();
    memcpy(&this->v3(), v1, 12);
  }
  inline void Set(const Vector3f &v0, const Vector3f &v1) {
    memcpy(&this->v0(), v0, 12);
    memcpy(&this->v3(), v1, 12);
  }
  inline void Set012(const float *v) { memcpy(this, v, 12); }
  inline void Set345(const float *v) { memcpy(&v3(), v, 12); }
  inline void Get(float *v) const { memcpy(v, this, 24); }
  inline void Get(AlignedVector3f &v0, AlignedVector3f &v1) const {
    v0.v012r() = v0123();
    memcpy(v1, &this->v3(), 12);
  }
  inline void Get(Vector3f &v0, Vector3f &v1) const {
    memcpy(v0, &this->v0(), 12);
    memcpy(v1, &this->v3(), 12);
  }
  inline void Get012(AlignedVector3f &v) const { v.v012r() = v0123(); }
  inline void Get345(AlignedVector3f &v) const { memcpy(&v, &this->v3(), 12); }
  inline AlignedVector3f Get012() const {
    AlignedVector3f v;
    Get012(v);
    return v;
  }
  inline AlignedVector3f Get345() const {
    AlignedVector3f v;
    Get345(v);
    return v;
  }

  inline void MakeZero() {
    memset(this, 0, sizeof(AlignedVector6f));
  }
  inline void MakeZero012() {
    memset(this, 0, 12);
  }
  inline void MakeMinus() {
    v0123().vmake_minus();
    v4() = -v4();
    v5() = -v5();
  }

  inline bool Valid() const { return v0() != FLT_MAX; }
  inline bool Invalid() const { return v0() == FLT_MAX; }
  inline void Invalidate() { v0() = FLT_MAX; }

  inline void Scale(const float s) {
    v0123() *= s;
    v4() *= s;
    v5() *= s;
  }
  inline void GetScaled(const float s, AlignedVector6f &v) const {
    v.v0123() = v0123() * s;
    v.v4() = v4() * s;
    v.v5() = v5() * s;
  }

  inline void Scale(const xp128f &s) {
    v0123() *= s;
    v4() *= s[0];
    v5() *= s[0];
  }
  inline void GetScaled(const xp128f &s, AlignedVector6f &v) const {
    v.v0123() = v0123() * s;
    v.v4() = s[0] * v4();
    v.v5() = s[0] * v5();
  }

  inline float SquaredLength() const {
    return (v0123() * v0123()).vsum_all() + v4() * v4() + v5() * v5();
  }
  inline float Dot(const AlignedVector6f &v) const {
    return (v0123() * v.v0123()).vsum_all() + v4() * v.v4() + v5() * v.v5();
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5());
    } else {
      UT::Print("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5());
    }
  }
  inline void Print(const std::string str, const bool e) const {
    UT::Print("%s", str.c_str());
    Print(e);
  }

  inline void Random(const float vMax) { UT::Random(&v0(), 6, -vMax, vMax); }
  static inline AlignedVector6f GetRandom(const float vMax) {
    AlignedVector6f v;
    v.Random(vMax);
    return v;
  }

  inline bool AssertEqual(const LA::AlignedVector6f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual(&v0(), &v.v0(), 6, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const AlignedVector6f e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero(&v0(), 6, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

 public:

  static inline void apb(const AlignedVector6f &a, const AlignedVector6f &b, AlignedVector6f &apb) {
    apb.v0123() = a.v0123() + b.v0123();
    apb.v4() = a.v4() + b.v4();
    apb.v5() = a.v5() + b.v5();
  }

  static inline void AddsaTo(const float s, const AlignedVector6f &a, AlignedVector6f &sa) {
    sa.v0123() += a.v0123() * s;
    sa.v4() += a.v4() * s;
    sa.v5() += a.v5() * s;
  }

 protected:
  union {
    float m_data[6];
    xp128f m_data4[2];
  };
};

template<typename TYPE> class Vector6 {

 public:

  static inline Vector6<TYPE> Get(const float *v) { Vector6<TYPE> _v; _v.Set(v); return _v; }

  inline const TYPE& v0() const { return m_data[0]; }   inline TYPE& v0() { return m_data[0]; }
  inline const TYPE& v1() const { return m_data[1]; }   inline TYPE& v1() { return m_data[1]; }
  inline const TYPE& v2() const { return m_data[2]; }   inline TYPE& v2() { return m_data[2]; }
  inline const TYPE& v3() const { return m_data[3]; }   inline TYPE& v3() { return m_data[3]; }
  inline const TYPE& v4() const { return m_data[4]; }   inline TYPE& v4() { return m_data[4]; }
  inline const TYPE& v5() const { return m_data[5]; }   inline TYPE& v5() { return m_data[5]; }

  //inline const TYPE* Data() const {
  //  return reinterpret_cast<const TYPE*>(m_data);
  //}
  //inline TYPE* Data() {
  //  return reinterpret_cast<TYPE*>(m_data);
  //}
  inline operator const TYPE* () const { return m_data; }
  inline operator       TYPE* ()       { return m_data; }

  inline const float& operator() (const int row, const int col = 0) const {
    return m_data[row];
  }

  inline float& operator() (const int row, const int col = 0) {
    return m_data[row];
  }

  //inline void operator = (const TYPE *v) { Set(v); }
  inline Vector6<TYPE> operator * (const TYPE s) const {
    Vector6<TYPE> v;
    GetScaled(s, v);
    return v;
  }

  inline Vector6<TYPE> operator * (const xp128f &s) const {
    Vector6<TYPE> v;
    GetScaled(TYPE(s[0]), v);
    return v;
  }

  inline void operator += (const Vector6<TYPE> &v) {
    v0() += v.v0();
    v1() += v.v1();
    v2() += v.v2();
    v3() += v.v3();
    v4() += v.v4();
    v5() += v.v5();
  }
  inline void operator -= (const Vector6<TYPE> &v) {
    v0() -= v.v0();
    v1() -= v.v1();
    v2() -= v.v2();
    v3() -= v.v3();
    v4() -= v.v4();
    v5() -= v.v5();
  }
  inline void operator *= (const TYPE s) {
    v0() *= s;
    v1() *= s;
    v2() *= s;
    v3() *= s;
    v4() *= s;
    v5() *= s;
  }
  inline Vector6<TYPE> operator - (const Vector6<TYPE> &b) const {
    Vector6<TYPE> amb;
    amb.v0() = v0() - b.v0();
    amb.v1() = v1() - b.v1();
    amb.v2() = v2() - b.v2();
    amb.v3() = v3() - b.v3();
    amb.v4() = v4() - b.v4();
    amb.v5() = v5() - b.v5();
    return amb;
  }
  inline bool operator == (const Vector6<TYPE> &v) const {
    return v0() == v.v0() && v1() == v.v1() && v2() == v.v2() &&
           v3() == v.v3() && v4() == v.v4() && v5() == v.v5();
  }

  inline void Set(const float v) { v0() = v; v1() = v; v2() = v; v3() = v; v4() = v; v5() = v; }
  inline void Set(const float *v);
  inline void Set(const double *v);
  inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1) { Set012(v0); Set345(v1); }
  inline void Set012(const float *v);
  inline void Set345(const float *v);
  inline void Get(TYPE *v) const { memcpy(v, this, sizeof(Vector6<TYPE>)); }
  inline void Get(AlignedVector3f &v0, AlignedVector3f &v1) const { Get012(v0); Get345(v1); }
  inline void Get012(float *v) const;
  inline void Get345(float *v) const;
  inline Vector3<TYPE> Get012() const { Vector3<TYPE> v; Get012(v); return v; }
  inline Vector3<TYPE> Get345() const { Vector3<TYPE> v; Get345(v); return v; }
  inline void Increase345(const Vector3<TYPE> &v) {
    v3() += v.v0();
    v4() += v.v1();
    v5() += v.v2();
  }

  inline void MakeZero() { memset(this, 0, sizeof(Vector6<TYPE>)); }

  inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
  inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }
  inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }

  inline void GetScaled(const TYPE s, Vector6<TYPE> &v) const {
    v.v0() = s * v0();
    v.v1() = s * v1();
    v.v2() = s * v2();
    v.v3() = s * v3();
    v.v4() = s * v4();
    v.v5() = s * v5();
  }
  inline TYPE Dot(const Vector6<TYPE> &v) const {
    return v0() * v.v0() + v1() * v.v1() + v2() * v.v2() +
           v3() * v.v3() + v4() * v.v4() + v5() * v.v5();
  }
  inline TYPE SquaredLength() const {
    return v0() * v0() + v1() * v1() + v2() * v2() +
           v3() * v3() + v4() * v4() + v5() * v5();
  }
  inline TYPE Sum() const { return v0() + v1() + v2() + v3() + v4() + v5(); }
  inline TYPE Maximal() const {
    // TODO(yanghongtian) : rooms for optimization
    return std::max(std::max(std::max(v0(), v1()),
                             std::max(v2(), v3())),
                             std::max(v4(), v5()));
  }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5());
    } else {
      UT::Print("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5());
    }
  }
  inline void Print(const std::string str, const bool e, const bool n) const {
    UT::Print("%s", str.c_str());
    if (e) {
      UT::Print("%e %e %e %e %e %e", v0(), v1(), v2(), v3(), v4(), v5());
    } else {
      UT::Print("%f %f %f %f %f %f", v0(), v1(), v2(), v3(), v4(), v5());
    }
    if (n) {
      UT::Print("\n");
    }
  }

  inline bool AssertEqual(const Vector6<TYPE> &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertEqual<TYPE>(&v0(), &v.v0(), 6, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
      v.Print(verbose > 1);
      const Vector6<TYPE> e = *this - v;
      e.Print(verbose > 1);
    }
    return false;
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    if (UT::VectorAssertZero<TYPE>(&v0(), 6, verbose, str, epsAbs, epsRel)) {
      return true;
    } else if (verbose) {
      UT::PrintSeparator();
      Print(verbose > 1);
    }
    return false;
  }

  inline void Random(const TYPE vMax) { Random(-vMax, vMax); }
  inline void Random(const TYPE vMin, const TYPE vMax) { UT::Random(&v0(), 6, vMin, vMax); }
  static inline Vector6<TYPE> GetRandom(const TYPE vMin, const TYPE vMax) {
    Vector6<TYPE> v;
    v.Random(vMin, vMax);
    return v;
  }

  static inline void AbTo0(const AlignedMatrix3x3f &A,
                           const AlignedVector3f &b,
                           Vector6<TYPE> &Ab) {
    Ab.v0() = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v1() = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v2() = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo0(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                              Vector6<TYPE> &Ab) {
    Ab.v0() += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v1() += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v2() += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AbTo3(const AlignedMatrix3x3f &A,
                           const AlignedVector3f &b,
                           Vector6<TYPE> &Ab) {
    Ab.v3() = (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v4() = (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v5() = (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }
  static inline void AddAbTo3(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                              Vector6<TYPE> &Ab) {
    Ab.v3() += (A.m_00_01_02_r0() * b.v012r()).vsum_012();
    Ab.v4() += (A.m_10_11_12_r1() * b.v012r()).vsum_012();
    Ab.v5() += (A.m_20_21_22_r2() * b.v012r()).vsum_012();
  }

 protected:
  TYPE m_data[6];
};

typedef Vector6<float> Vector6f;
typedef Vector6<double> Vector6d;

template<> inline void Vector6f::Set(const float *v) { memcpy(this, v, sizeof(Vector6f)); }
template<> inline void Vector6f::Set012(const float *v) { memcpy(&v0(), v, 12); }
template<> inline void Vector6f::Set345(const float *v) { memcpy(&v3(), v, 12); }
// TODO (yanghongtian) : check this operation.
template<> inline void Vector6f::Get012(float *v) const { memcpy(v, &v0(), 12); }
template<> inline void Vector6f::Get345(float *v) const { memcpy(v, &v3(), 12); }

template<> inline void Vector6d::Set(const float *v) {
  for (int i = 0; i < 6; ++i) {
    m_data[i] = static_cast<double>(v[i]);
  }
}
template<> inline void Vector6d::Set(const double *v) { memcpy(this, v, sizeof(Vector6d)); }
template<> inline void Vector6d::Get012(float *v) const {
  v[0] = static_cast<float>(v0());
  v[1] = static_cast<float>(v1());
  v[2] = static_cast<float>(v2());
}
template<> inline void Vector6d::Get345(float *v) const {
  v[0] = static_cast<float>(v3());
  v[1] = static_cast<float>(v4());
  v[2] = static_cast<float>(v5());
}

class ProductVector6f : public AlignedVector6f {
 public:
  static inline ProductVector6f Get(const float *v) {
    ProductVector6f _v;
    _v.Set(v);
    return _v;
  }

  inline const xp128f& v4501() const { return v45xx(); }  inline xp128f& v4501() { return v45xx(); }
  inline const xp128f& v2345() const { return m_v2345; }  inline xp128f& v2345() { return m_v2345; }

  inline void Set(const float *v) { LA::AlignedVector6f::Set(v); Update(); }
  inline void Set(const double *v) { LA::AlignedVector6f::Set(v); Update(); }
  inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1) {
    AlignedVector6f::Set(v0, v1);
    Update();
  }
  inline void Get(AlignedVector3f &v0, AlignedVector3f &v1) const {
    AlignedVector6f::Get(v0, v1);
  }
  inline void Update() {
    memcpy(&m_data4[1][2], &v0(), 24);
  }

  inline void MakeZero() { memset(this, 0, sizeof(ProductVector6f)); }

  inline void Print(const bool e = false) const {
    if (e) {
      UT::Print("%e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(), v5());
    } else {
      UT::Print("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(), v5());
    }
  }

 protected:
  xp128f m_v2345;
};
}  // namespace LA

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector6f : public Eigen::Matrix<float, 6, 1> {
 public:
  inline EigenVector6f() : Eigen::Matrix<float, 6, 1>() {}
  inline EigenVector6f(const Eigen::Matrix<float, 6, 1> &e_v) : Eigen::Matrix<float, 6, 1>(e_v) {}
  inline EigenVector6f(const float *v) : Eigen::Matrix<float, 6, 1>() { *this = v; }
  inline EigenVector6f(const EigenVector3f &e_v0, const EigenVector3f &e_v1) {
    block<3, 1>(0, 0) = e_v0;
    block<3, 1>(3, 0) = e_v1;
  }
  inline EigenVector6f(const LA::AlignedVector3f &v0, const LA::AlignedVector3f &v1) {
    block<3, 1>(0, 0) = EigenVector3f(v0);
    block<3, 1>(3, 0) = EigenVector3f(v1);
  }
  inline void operator = (const Eigen::Matrix<float, 6, 1> &e_v) {
    *((Eigen::Matrix<float, 6, 1> *) this) = e_v;
  }
  inline void operator = (const float *v) {
    Eigen::Matrix<float, 6, 1> &e_v = *this;
    for (int i = 0; i < 6; ++i) {
      e_v(i, 0) = v[i];
    }
  }
  inline LA::AlignedVector6f GetAlignedVector6f() const {
    LA::AlignedVector6f v;
    float* _v = v;
    const Eigen::Matrix<float, 6, 1> &e_v = *this;
    for (int i = 0; i < 6; ++i) {
      _v[i] = e_v(i, 0);
    }
    return v;
  }
  inline LA::Vector6f GetVector6f() const {
    LA::Vector6f v;
    float* _v = v;
    const Eigen::Matrix<float, 6, 1> &e_v = *this;
    for (int i = 0; i < 6; ++i) {
      _v[i] = e_v(i, 0);
    }
    return v;
  }
  inline float SquaredLength() const { return GetAlignedVector6f().SquaredLength(); }
  inline void Print(const bool e = false) const { GetAlignedVector6f().Print(e); }
  static inline EigenVector6f GetRandom(const float vMax) {
    return EigenVector6f(LA::AlignedVector6f::GetRandom(vMax));
  }
  inline bool AssertEqual(const LA::AlignedVector6f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector6f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const LA::Vector6f &v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetVector6f().AssertEqual(v, verbose, str, epsAbs, epsRel);
  }
  inline bool AssertEqual(const EigenVector6f &e_v,
                          const int verbose = 1, const std::string str = "",
                          const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return AssertEqual(e_v.GetAlignedVector6f(), verbose, str, epsAbs, epsRel);
  }
  inline bool AssertZero(const int verbose = 1, const std::string str = "",
                         const float epsAbs = 0.0f, const float epsRel = 0.0f) const {
    return GetAlignedVector6f().AssertZero(verbose, str, epsAbs, epsRel);
  }
};
#endif
#endif  // LINEARALGEBRA_VECTOR6_H_
